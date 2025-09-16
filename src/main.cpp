#include <esp_now.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <Preferences.h>
#include <Arduino.h>
#include <esp_wifi.h>

// config
constexpr uint8_t NUM_LEDS = 1;
constexpr uint32_t DEEP_SLEEP_TIMEOUT = 5 * 1000000; // 10 Sekunden in µs
constexpr uint16_t WAKE_UP_TIME = 300;
constexpr uint8_t BATTERY_PIN = 0;
constexpr uint8_t LED_CONTROL_PIN = 22;
constexpr uint8_t LED_BUILTIN_PIN = 10; // On-board LED for status indication

// ESPNOW master channel (override via -D MASTER_CHANNEL=<n>)
#ifndef MASTER_CHANNEL
#define MASTER_CHANNEL 0
#endif

// Debug settings
// #define DEBUG_MODE // Comment out for production

// Protocol: header + payloads aligned with master
#pragma pack(push, 1)
enum MsgType : uint8_t
{
  MSG_CMD = 1,
  MSG_TELEMETRY = 2,
  MSG_ACK = 3
};

struct MsgHdr
{
  uint8_t magic;   // 0xA5
  uint8_t version; // 0x01
  uint8_t type;    // MsgType
  uint8_t channel; // 0..N-1
  uint16_t len;    // payload length
  uint16_t seq;    // sequence id
  uint16_t crc;    // CRC16-CCITT over header(with crc=0) XOR payload CRC
};

struct CmdPayload
{
  uint16_t brightness; // 0..100
  uint8_t state;       // 0/1
  uint8_t reqTelem;    // 0/1
};

struct TelemetryPayload
{
  uint16_t appliedBrightness; // 0..100 actually set
  uint8_t appliedState;       // 0/1 actually set
  uint16_t halfVoltageMv;     // half of battery voltage in mV
  uint32_t operatingHours;    // total operating hours
  uint32_t lastChargedDate;   // device-defined
};
#pragma pack(pop)

// global variables
esp_now_peer_info_t peerInfo{};
static uint16_t currentBrightness = 0;
static bool currentState = false;
uint8_t senderMac[6] = {0};
Preferences preferences;

static bool pwmInit = false;
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint32_t PWM_FREQ = 5000; // Hz
constexpr uint8_t PWM_RES = 8;      // bits -> 0..20

// Operating time tracking: accumulate only when LED is ON, incl. deep sleep
static uint32_t baseTotalSeconds = 0; // persisted total seconds loaded at boot
static uint32_t sessionStartMs = 0;   // legacy, not used for accumulation anymore
static uint32_t onStartMs = 0;        // millis() when LED was last turned ON in this wake session

// function definitions
void updateLedState(uint8_t brightness, bool state);
void receiveMessage(const esp_now_recv_info *info, const uint8_t *data, int len);
void initEspNow();
void updatePeerConnection();
void sendAck(uint16_t seq, uint8_t channel);
void sendTelemetry(uint8_t channel, uint16_t seq);
void initHardware();
void enterDeepSleep();
void resumeRadio();
void printMacAddress();
void saveStatus();
void loadStatus();
static uint32_t getCurrentTotalSeconds();

// function implementations

void updateLedState(uint8_t brightness, bool state)
{
  if (state)
  {
    analogWrite(LED_CONTROL_PIN, brightness * 255 / 100);
  }
  else
  {
    analogWrite(LED_CONTROL_PIN, 0);
  }
}

static uint16_t crc16_ccitt(const uint8_t *buf, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= (uint16_t)buf[i] << 8;
    for (int b = 0; b < 8; ++b)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void receiveMessage(const esp_now_recv_info *info, const uint8_t *data, int len)
{
  memcpy(senderMac, info->src_addr, 6);

  if (len < (int)sizeof(MsgHdr))
    return;
  const MsgHdr *hdr = reinterpret_cast<const MsgHdr *>(data);
  if (hdr->magic != 0xA5 || hdr->version != 0x01)
    return;
  if ((uint16_t)len != (uint16_t)(sizeof(MsgHdr) + hdr->len))
    return;

  // CRC check
  MsgHdr tmp = *hdr;
  tmp.crc = 0;
  uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t *>(&tmp), sizeof(MsgHdr));
  uint16_t tail = crc16_ccitt(reinterpret_cast<const uint8_t *>(data) + sizeof(MsgHdr), hdr->len);
  crc ^= tail;
  if (crc != hdr->crc)
    return;

  const uint8_t *payload = reinterpret_cast<const uint8_t *>(data) + sizeof(MsgHdr);
  if (hdr->type == MSG_CMD && hdr->len == sizeof(CmdPayload))
  {
    // Apply brightness with PWM (0..100 -> duty)
    const CmdPayload *cmd = reinterpret_cast<const CmdPayload *>(payload);
    {
      // Clamp to 0..100 and remember
      uint16_t bri = cmd->brightness;
      if (bri > 100)
        bri = 100;
      currentBrightness = bri;
    }

    // Handle LED state transitions to update operating time accumulator
    bool newState = cmd->state ? true : false;
    Serial.println("New state: " + String(newState) + ", brightness: " + String(currentBrightness));
    currentState = newState;

    updateLedState(currentBrightness, currentState);
    updatePeerConnection();
    saveStatus();

    // Always ACK; send telemetry if requested
    sendAck(hdr->seq, hdr->channel);
    if (cmd->reqTelem)
      sendTelemetry(hdr->channel, hdr->seq);
  }
}

void initEspNow()
{
  WiFi.mode(WIFI_STA);
  // Ensure ESPNOW uses the intended channel when not associated to an AP
  esp_wifi_set_channel(MASTER_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK)
  {
#ifdef DEBUG_MODE
    Serial.println("ESP-NOW initialisation failed");
#endif
    ESP.restart();
  }
  // deactivate pin hold from previous deep sleep
  gpio_hold_dis(static_cast<gpio_num_t>(LED_CONTROL_PIN));
  esp_now_register_recv_cb(receiveMessage);
}

void updatePeerConnection()
{
  if (memcmp(peerInfo.peer_addr, senderMac, 6) != 0)
  {
    esp_now_del_peer(peerInfo.peer_addr);

    memcpy(peerInfo.peer_addr, senderMac, 6);
    peerInfo.channel = MASTER_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
#ifdef DEBUG_MODE
      Serial.println("Peer connection failed");
#endif
    }
  }
}

void sendAck(uint16_t seq, uint8_t channel)
{
  MsgHdr hdr{};
  hdr.magic = 0xA5;
  hdr.version = 0x01;
  hdr.type = MSG_ACK;
  hdr.channel = channel;
  hdr.len = 0;
  hdr.seq = seq;
  hdr.crc = 0;

  uint16_t head = crc16_ccitt(reinterpret_cast<const uint8_t *>(&hdr), sizeof(MsgHdr));
  uint16_t tail = crc16_ccitt(nullptr, 0); // == 0xFFFF by our impl
  hdr.crc = head ^ tail;

  esp_now_send(senderMac, reinterpret_cast<const uint8_t *>(&hdr), sizeof(MsgHdr));
}

void sendTelemetry(uint8_t channel, uint16_t seq)
{
  TelemetryPayload telem{};
  telem.appliedBrightness = currentBrightness;
  telem.appliedState = currentState ? 1 : 0;
  telem.halfVoltageMv = (uint16_t)analogReadMilliVolts(BATTERY_PIN);
  // Send total operating time in seconds (no division) for testing/precision
  telem.operatingHours = 33 + getCurrentTotalSeconds(); // Later: / 3600U for hours
  telem.lastChargedDate = 44;                           // TODO: set appropriately

  MsgHdr hdr{};
  hdr.magic = 0xA5;
  hdr.version = 0x01;
  hdr.type = MSG_TELEMETRY;
  hdr.channel = channel;
  hdr.len = sizeof(TelemetryPayload);
  hdr.seq = seq; // tie telemetry to the last command
  hdr.crc = 0;

  uint8_t buffer[sizeof(MsgHdr) + sizeof(TelemetryPayload)];
  memcpy(buffer, &hdr, sizeof(hdr));
  memcpy(buffer + sizeof(hdr), &telem, sizeof(telem));
  uint16_t head = crc16_ccitt(buffer, sizeof(MsgHdr));
  uint16_t tail = crc16_ccitt(buffer + sizeof(MsgHdr), sizeof(TelemetryPayload));
  reinterpret_cast<MsgHdr *>(buffer)->crc = head ^ tail;

  esp_now_send(senderMac, buffer, sizeof(buffer));
}

void initHardware()
{
  pinMode(BATTERY_PIN, INPUT);
  pinMode(LED_CONTROL_PIN, OUTPUT);
  pinMode(LED_BUILTIN_PIN, OUTPUT);

#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
}

void enterDeepSleep()
{
  saveStatus(); // Status vor dem Schlafen gehen speichern
#ifdef DEBUG_MODE
  Serial.println("Shutting down radio and entering deep sleep");
#endif
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIMEOUT);

  // If LED is off, turn off the pin to save power
  // Dies ist für Deep Sleep nicht zwingend notwendig, da die meisten GPIOs ihren Zustand verlieren, schadet aber nicht.
  digitalWrite(LED_CONTROL_PIN, LOW);
  // gpio_hold_dis(static_cast<gpio_num_t>(LED_CONTROL_PIN));
  esp_deep_sleep_start();
}

void enterLightSleep()
{
  saveStatus(); // Status vor dem Schlafen gehen speichern
#ifdef DEBUG_MODE
  Serial.println("Shutting down radio for light sleep...");
#endif
  // WLAN und ESP-NOW vor dem Light Sleep ordnungsgemäß herunterfahren
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);

  Serial.println("Entering light sleep");

  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIMEOUT);
  esp_err_t err = esp_light_sleep_start();
#ifdef DEBUG_MODE
  Serial.printf("Woke: err=%d, cause=%d\n", (int)err, (int)esp_sleep_get_wakeup_cause());
#endif
  // After light sleep resume WiFi/ESP-NOW for communication
  resumeRadio();
}

void resumeRadio()
{
#ifdef DEBUG_MODE
  Serial.println("Resuming radio (WiFi/ESP-NOW)");
#endif
  // initEspNow() kümmert sich um die Re-Initialisierung von WLAN und ESP-NOW
  initEspNow();

  // Re-add known peer on fixed channel if available
  bool known = false;
  for (int i = 0; i < 6; ++i)
  {
    if (senderMac[i] != 0)
    {
      known = true;
      break;
    }
  }
  if (known)
  {
    memcpy(peerInfo.peer_addr, senderMac, 6);
    peerInfo.channel = MASTER_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
#ifdef DEBUG_MODE
      Serial.println("Failed to re-add peer");
#endif
    }
  }
}

void printMacAddress()
{
  // Variable to store the MAC address
  uint8_t baseMac[6];

  // Get MAC address of the WiFi station interface
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.print("Station MAC: ");
  for (int i = 0; i < 5; i++)
  {
    Serial.printf("%02X:", baseMac[i]);
  }
  Serial.printf("%02X\n", baseMac[5]);
}

void saveStatus()
{
  preferences.begin("framecontrol", false);
  preferences.putBool("led_status", currentState);
  uint32_t nowMs = millis();
  uint32_t seconds = baseTotalSeconds + ((currentState && onStartMs != 0) ? ((nowMs - onStartMs) / 1000U) : 0U);
  preferences.putULong("op_sec", seconds);
  preferences.putUShort("brightness", currentBrightness);
  preferences.end();
}

void loadStatus()
{
  preferences.begin("framecontrol", false);
  currentState = preferences.getBool("led_status", false);
  baseTotalSeconds = preferences.getULong("op_sec", 0);
  currentBrightness = preferences.getUShort("brightness", 100);
  preferences.end();
  sessionStartMs = millis();
  onStartMs = currentState ? millis() : 0;
}

// Compute total seconds = persisted base + this session elapsed
static uint32_t getCurrentTotalSeconds()
{
  uint32_t nowMs = millis();
  uint32_t onDelta = (currentState && onStartMs != 0) ? ((nowMs - onStartMs) / 1000U) : 0U;
  return baseTotalSeconds + onDelta;
}

void setup()
{
  initHardware();
  initEspNow();

  loadStatus();                                    // Load the status on startup
  updateLedState(currentBrightness, currentState); // Update the LED state based on the loaded status

#ifdef DEBUG_MODE
  printMacAddress(); // Read the MAC address of the ESP32
  Serial.println("Receiver ready");
  // stay awake for 10 sec -> for debugging
  delay(5000);
#endif
}

void loop()
{
  if (currentState)
  {
    // Stay awake briefly to receive commands, then sleep
    loadStatus();
    updateLedState(currentBrightness, currentState);
    receiveMessage(nullptr, nullptr, 0); // Dummy call to allow processing incoming messages
    delay(WAKE_UP_TIME);                 // Kurz wach bleiben, um Befehle zu empfangen
    enterLightSleep();
    // Nach Wake wird ggf. receiveMessage() aufgerufen und currentState/Helligkeit angepasst
  }
  else
  {
    enterDeepSleep(); // LED aus -> echter Deep‑Sleep
  }
}
