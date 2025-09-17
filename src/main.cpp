#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <esp_pm.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
constexpr uint8_t BATTERY_PIN = 0;
constexpr uint8_t LED_CONTROL_PIN = 22;
constexpr uint8_t LED_BUILTIN_PIN = 15;
constexpr uint32_t ACTIVE_WINDOW_MS = 5000;                               // awake time per burst
constexpr uint64_t SLEEP_INTERVAL_US = 5ULL * 1000000ULL;                 // 60 seconds
constexpr uint64_t STATE_PERSIST_INTERVAL_US = 5ULL * 60ULL * 1000000ULL; // 5 minutes

// 1C:69:20:CE:68:50 Master-Info
static const uint8_t MASTER_MAC[6] = {0x1C, 0x69, 0x20, 0xCE, 0x68, 0x50};
static const uint8_t MASTER_CHANNEL = 1;

// -----------------------------------------------------------------------------
// Protocol primitives
// -----------------------------------------------------------------------------
#pragma pack(push, 1)
enum MsgType : uint8_t
{
  MSG_CMD = 1,
  MSG_TELEMETRY = 2,
  MSG_ACK = 3
};

struct MsgHdr
{
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t channel;
  uint16_t len;
  uint16_t seq;
  uint16_t crc;
};

struct CmdPayload
{
  uint16_t brightness;
  uint8_t state;
  uint8_t reqTelem;
};

struct TelemetryPayload
{
  uint16_t appliedBrightness;
  uint8_t appliedState;
  uint16_t halfVoltageMv;
  uint32_t operatingSeconds;
  uint32_t lastChargedDate;
};
#pragma pack(pop)

// -----------------------------------------------------------------------------
// Device-level state
// -----------------------------------------------------------------------------
namespace fc
{

  struct DeviceState
  {
    uint16_t brightness = 100;
    bool ledOn = false;
    uint32_t baseSeconds = 0;
    uint64_t onStartedUs = 0;
    uint8_t peerMac[6] = {0};
    bool peerKnown = false;
    uint8_t lastLogicalChannel = 0;
    uint16_t lastSeq = 0;
    bool stateDirty = false;
    uint64_t lastPersistUs = 0;
  };

  struct PendingCommand
  {
    bool hasData = false;
    MsgHdr header{};
    CmdPayload payload{};
    uint8_t mac[6] = {0};
  };

  static DeviceState state{};
  static PendingCommand pending{};
  static portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;
  static Preferences preferences;
  static esp_now_peer_info_t peerInfo{};
  static esp_pm_lock_handle_t pm_lock;

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  static uint16_t crc16_ccitt(const uint8_t *buf, size_t len)
  {
    uint16_t crc = 0xFFFF;
    for (size_t idx = 0; idx < len; ++idx)
    {
      crc ^= static_cast<uint16_t>(buf[idx]) << 8;
      for (int bit = 0; bit < 8; ++bit)
      {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
      }
    }
    return crc;
  }

  static uint64_t microsNow()
  {
    return static_cast<uint64_t>(esp_timer_get_time());
  }

  static uint32_t secondsSinceOn()
  {
    if (!state.ledOn || state.onStartedUs == 0)
      return 0;
    uint64_t now = microsNow();
    if (now < state.onStartedUs)
      return 0;
    return static_cast<uint32_t>((now - state.onStartedUs) / 1000000ULL);
  }

  static uint32_t totalOperatingSeconds()
  {
    return state.baseSeconds + secondsSinceOn();
  }

  static void applyLedState()
  {
    uint32_t duty = state.ledOn ? static_cast<uint32_t>(state.brightness) * 255U / 100U : 0U;
    analogWrite(LED_CONTROL_PIN, static_cast<int>(duty));
    digitalWrite(LED_BUILTIN_PIN, state.ledOn ? HIGH : LOW);
  }

  static void markLedState(bool newState)
  {
    if (newState == state.ledOn)
      return;

    if (state.ledOn)
    {
      state.baseSeconds += secondsSinceOn();
      state.onStartedUs = 0;
    }

    state.ledOn = newState;
    state.onStartedUs = state.ledOn ? microsNow() : 0;
    state.stateDirty = true;
  }

  static void updateBrightness(uint16_t brightness)
  {
    if (brightness > 100)
      brightness = 100;
    if (brightness == state.brightness)
      return;

    state.brightness = brightness;
    state.stateDirty = true;
  }

  static void addMasterPeer()
  {
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, MASTER_MAC, 6);
    peer.channel = MASTER_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
  }

  static void ensurePeer(const uint8_t mac[6], uint8_t logicalChannel)
  {
    state.lastLogicalChannel = logicalChannel;

    if (state.peerKnown && memcmp(state.peerMac, mac, 6) == 0)
      return;

    if (state.peerKnown)
    {
      esp_now_del_peer(state.peerMac);
      state.peerKnown = false;
    }

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.channel = MASTER_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK)
    {
      memcpy(state.peerMac, mac, 6);
      state.peerKnown = true;
    }
  }

  static void persistState()
  {
    uint32_t totalSeconds = totalOperatingSeconds();
    preferences.begin("framecontrol", false);
    preferences.putBool("led_status", state.ledOn);
    preferences.putULong("op_sec", totalSeconds);
    preferences.putUShort("brightness", state.brightness);
    preferences.end();

    state.baseSeconds = totalSeconds;
    state.onStartedUs = state.ledOn ? microsNow() : 0;
    state.stateDirty = false;
    state.lastPersistUs = microsNow();
  }

  static void loadState()
  {
    preferences.begin("framecontrol", false);
    state.ledOn = preferences.getBool("led_status", false);
    state.baseSeconds = preferences.getULong("op_sec", 0);
    state.brightness = preferences.getUShort("brightness", 100);
    preferences.end();

    state.onStartedUs = state.ledOn ? microsNow() : 0;
    state.stateDirty = false;
    state.lastPersistUs = microsNow();
  }

  static void sendAck(const uint8_t mac[6], uint16_t seq, uint8_t logicalChannel)
  {
    MsgHdr hdr{};
    hdr.magic = 0xA5;
    hdr.version = 0x01;
    hdr.type = MSG_ACK;
    hdr.channel = logicalChannel;
    hdr.len = 0;
    hdr.seq = seq;
    hdr.crc = 0;

    uint16_t head = crc16_ccitt(reinterpret_cast<const uint8_t *>(&hdr), sizeof(MsgHdr));
    hdr.crc = head ^ crc16_ccitt(nullptr, 0);

    esp_now_send(mac, reinterpret_cast<const uint8_t *>(&hdr), sizeof(MsgHdr));
  }

  static void sendTelemetry(const uint8_t mac[6], uint8_t logicalChannel, uint16_t seq)
  {
    TelemetryPayload telem{};
    telem.appliedBrightness = state.brightness;
    telem.appliedState = state.ledOn ? 1 : 0;
    telem.halfVoltageMv = static_cast<uint16_t>(analogReadMilliVolts(BATTERY_PIN));
    telem.operatingSeconds = totalOperatingSeconds();
    telem.lastChargedDate = 0;

    MsgHdr hdr{};
    hdr.magic = 0xA5;
    hdr.version = 0x01;
    hdr.type = MSG_TELEMETRY;
    hdr.channel = logicalChannel;
    hdr.len = sizeof(TelemetryPayload);
    hdr.seq = seq;
    hdr.crc = 0;

    uint8_t buffer[sizeof(MsgHdr) + sizeof(TelemetryPayload)];
    memcpy(buffer, &hdr, sizeof(hdr));
    memcpy(buffer + sizeof(hdr), &telem, sizeof(telem));

    uint16_t head = crc16_ccitt(buffer, sizeof(MsgHdr));
    uint16_t tail = crc16_ccitt(buffer + sizeof(MsgHdr), sizeof(TelemetryPayload));
    reinterpret_cast<MsgHdr *>(buffer)->crc = head ^ tail;

    esp_now_send(mac, buffer, sizeof(buffer));
  }

  static bool decodeCommand(const uint8_t *data, int len, MsgHdr &hdr, CmdPayload &cmd)
  {
    if (!data || len < static_cast<int>(sizeof(MsgHdr)))
      return false;

    memcpy(&hdr, data, sizeof(MsgHdr));
    if (hdr.magic != 0xA5 || hdr.version != 0x01)
      return false;
    if (hdr.type != MSG_CMD || hdr.len != sizeof(CmdPayload))
      return false;
    if (len != static_cast<int>(sizeof(MsgHdr) + hdr.len))
      return false;

    MsgHdr tmp = hdr;
    tmp.crc = 0;
    uint16_t head = crc16_ccitt(reinterpret_cast<const uint8_t *>(&tmp), sizeof(MsgHdr));
    uint16_t tail = crc16_ccitt(data + sizeof(MsgHdr), hdr.len);
    if ((head ^ tail) != hdr.crc)
      return false;

    memcpy(&cmd, data + sizeof(MsgHdr), sizeof(CmdPayload));
    return true;
  }

  static bool takePending(PendingCommand &out)
  {
    bool result = false;
    portENTER_CRITICAL(&pendingMux);
    if (pending.hasData)
    {
      out = pending;
      pending.hasData = false;
      result = true;
    }
    portEXIT_CRITICAL(&pendingMux);
    return result;
  }

  static void handleCommand(const PendingCommand &cmd)
  {
    ensurePeer(cmd.mac, cmd.header.channel);
    state.lastSeq = cmd.header.seq;

    updateBrightness(cmd.payload.brightness);
    markLedState(cmd.payload.state != 0);
    applyLedState();

    if (state.stateDirty)
    {
      persistState();
    }

    sendAck(cmd.mac, cmd.header.seq, cmd.header.channel);
    if (cmd.payload.reqTelem)
    {
      sendTelemetry(cmd.mac, cmd.header.channel, cmd.header.seq);
    }
  }

  static void processPendingCommands()
  {
    PendingCommand cmd;
    while (takePending(cmd))
    {
      handleCommand(cmd);
    }
  }

  static void maybePersistRuntime()
  {
    if (!state.ledOn)
      return;

    uint64_t now = microsNow();
    if (now - state.lastPersistUs >= STATE_PERSIST_INTERVAL_US)
    {
      persistState();
    }
  }

  static void sendHeartbeat()
  {
    if (!state.peerKnown)
      return;

    // if (++telemetrySeq == 0)
    // {
    //   ++telemetrySeq;
    // }

    sendTelemetry(state.peerMac, state.lastLogicalChannel, 123);
  }

  static void initHardware()
  {
    pinMode(BATTERY_PIN, INPUT);
    pinMode(LED_CONTROL_PIN, OUTPUT);
    pinMode(LED_BUILTIN_PIN, OUTPUT);
  }

  static void onEspNowReceive(const esp_now_recv_info *info, const uint8_t *data, int len)
  {
    if (!info)
      return;

    MsgHdr hdr;
    CmdPayload cmd;
    if (!decodeCommand(data, len, hdr, cmd))
      return;

    portENTER_CRITICAL(&pendingMux);
    pending.header = hdr;
    pending.payload = cmd;
    memcpy(pending.mac, info->src_addr, 6);
    pending.hasData = true;
    portEXIT_CRITICAL(&pendingMux);
  }

  static void initRadio()
  {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(MASTER_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK)
    {
      ESP.restart();
    }

    esp_now_register_recv_cb(onEspNowReceive);
  }

  static void initRadioAfterWake()
  {
    WiFi.mode(WIFI_STA);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);

    wifi_country_t c = {.cc = "EU", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    esp_wifi_set_country(&c);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(MASTER_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    ESP_ERROR_CHECK(esp_now_init());
    addMasterPeer();
    esp_now_register_recv_cb(onEspNowReceive);

    vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms RF settle
  }

  static void suspendRadioBeforeSleep()
  {
    esp_now_deinit();
    esp_wifi_stop();
  }

  static void enterDeepSleep()
  {
    persistState();
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
    esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
    // digitalWrite(LED_CONTROL_PIN, LOW);
    esp_deep_sleep_start();
  }

  static void enterLightSleep()
  {
    suspendRadioBeforeSleep();

    // esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
    // esp_light_sleep_start();

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US));
    esp_err_t err = esp_light_sleep_start();
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_sleep_get_wakeup_cause());

    initRadioAfterWake();
  }

  static void activeWindow()
  {
    processPendingCommands();
    maybePersistRuntime();
    sendHeartbeat();
  }

} // namespace fc

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------

void blink(int times, int delayMs)
{
  for (int i = 0; i < times; ++i)
  {
    digitalWrite(LED_BUILTIN_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_BUILTIN_PIN, LOW);
    delay(delayMs);
  }
}

void setup()
{
  fc::initHardware();
  fc::loadState();
  fc::applyLedState();
  fc::initRadioAfterWake();

  Serial.begin(115200);
}

void loop()
{
  blink(2, 300); // just for debugging - delete later

  // Now we have a window to receive commands for ACTIVE_WINDOW_MS.
  uint64_t start = fc::microsNow();
  while (fc::microsNow() - start < (ACTIVE_WINDOW_MS * 1000ULL))
  {
    fc::processPendingCommands();
    delay(20); // Small delay to prevent busy-waiting
  }

  // After the window, send a heartbeat and persist state if needed.
  fc::sendHeartbeat();
  fc::maybePersistRuntime();

  // Now go back to sleep.
  if (!fc::state.ledOn)
  {
    fc::enterDeepSleep();
  }
  else
  {
    fc::enterLightSleep();
  }
}
