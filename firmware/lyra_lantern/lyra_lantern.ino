// =============================================================================
// LYRA - LANTERN BUILD   (ESP32-C3 SuperMini inside the power-battery lantern)
// =============================================================================
// The lantern is the voice + the dock. It connects to the ring over BLE ("LYRA-RING",
// see lyra_ring.ino for the text protocol) and plays lines through a DFPlayer Mini
// (MP3/WAV from a microSD) + speaker:
//   ring knocked            -> "Ring at N percent charge"        (folder 01, one clip per 5 %)
//   ring docked (charging)  -> "Charging" + bright breathing glow (folder 02 / 003)
//   ring full               -> "Ring fully charged"               (folder 02 / 002)
//   ring pressed to emblem  -> hall sensor feels the ring's magnet: plays the oath
//                              (folder 02 / 001) and tells the ring "SHOW CHARGEUP"
// Clips: tools/make_voice_clips.sh writes the whole SD card layout.
//
// Wiring (jumper wires, no soldering on modules with headers):
//   DFPlayer RX <- GPIO21 (via 1k), VCC 5V (USB), GND. Leave DFPlayer TX UNCONNECTED: this
//   code never reads it, and at 5 V supply it may drive more than the ESP32's 3.3 V pins take.
//   DFPlayer SPK1/SPK2 -> speaker (3 W max, 4-8 ohm)
//   Hall sensor: BARE A3144 (TO-92), VCC 5V (datasheet minimum 3.8 V: 3.3 V is out of spec), GND,
//   OUT -> GPIO3 with the ESP32's internal pull-up (open collector: the pin only ever sees 3.3 V).
//   NOT a KY-003 module: those pull OUT up to VCC = 5 V. A3144 is unipolar: it only switches on a
//   magnet's SOUTH pole (Bop 30-80 G) -> fit the ring's magnet south side out (test before gluing).
//   Lantern glow LED (or LED module) -> GPIO10   (on-board LED GPIO8 mirrors it)
// =============================================================================
#include <NimBLEDevice.h>

#define SVC_UUID "6E400201-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID  "6E400202-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400203-B5A3-F393-E0A9-E50E24DCCA9E"

static constexpr int PIN_DF_RX = 20, PIN_DF_TX = 21, PIN_HALL = 3, PIN_GLOW = 10, PIN_BOARD_LED = 8;
static constexpr uint8_t VOLUME = 24;                   // 0..30

// ---- DFPlayer Mini (its serial protocol, no library needed) ---------------------
HardwareSerial df(1);
void dfCmd(uint8_t cmd, uint16_t param) {
  uint8_t b[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, (uint8_t)(param >> 8), (uint8_t)param, 0, 0, 0xEF};
  int16_t sum = 0; for (int i = 1; i < 7; i++) sum -= b[i];
  b[7] = sum >> 8; b[8] = sum & 0xFF;
  df.write(b, 10);
}
void playClip(uint8_t folder, uint8_t track) { dfCmd(0x0F, (folder << 8) | track); }   // /NN/NNN.wav
void playLevel(int pct) { playClip(1, constrain((pct + 2) / 5, 0, 20) + 1); }          // 001 = 0 %, 021 = 100 %
enum { CLIP_OATH = 1, CLIP_FULL = 2, CLIP_CHARGING = 3, CLIP_CONNECTED = 4, CLIP_LOW = 5 };

// ---- glow ---------------------------------------------------------------------------
enum GlowMode { GLOW_IDLE, GLOW_CHARGING, GLOW_FLASH };
GlowMode glow = GLOW_IDLE; uint32_t glowT0 = 0;
void glowService() {
  uint32_t t = millis();
  int level;
  if (glow == GLOW_FLASH) {                                  // 2 s burst after the oath / full
    uint32_t e = t - glowT0; level = e < 2000 ? 255 - (int)(e * 255 / 2000) / 2 : 0;
    if (e >= 2000) glow = GLOW_IDLE;
  } else {
    uint32_t period = glow == GLOW_CHARGING ? 1600 : 6000, top = glow == GLOW_CHARGING ? 255 : 60;
    uint32_t p = t % period, tri = p < period / 2 ? p : period - p;
    level = 8 + (int)(tri * top * 2 / period);
  }
  ledcWrite(PIN_GLOW, level * level / 255);
  digitalWrite(PIN_BOARD_LED, level > 128 ? LOW : HIGH);
}

// ---- BLE client -----------------------------------------------------------------------
NimBLEClient* client = nullptr;
NimBLERemoteCharacteristic* ringRx = nullptr;
const NimBLEAdvertisedDevice* found = nullptr;
volatile bool doConnect = false;
char line[64]; volatile bool lineReady = false;
bool ringCharging = false;
uint32_t lastRingMsgMs = 0, lastDisconnectMs = 0;
bool everConnected = false;
// On the dock the ring sleeps and checks in every ~2 min (see lyra_ring.ino, dock mode), so the
// link drops between reports: keep the charging glow through the gaps, and only say
// "connected" when the ring has really been away.
static constexpr uint32_t CHARGING_HOLD_MS = 5UL * 60 * 1000;    // no report this long -> not docked
static constexpr uint32_t CONNECTED_QUIET_MS = 10UL * 60 * 1000; // reconnect within this: no voice

class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    if (d->isAdvertisingService(NimBLEUUID(SVC_UUID))) { found = d; doConnect = true; NimBLEDevice::getScan()->stop(); }
  }
};
void onNotify(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool) {
  size_t n = len < sizeof(line) - 1 ? len : sizeof(line) - 1;
  memcpy(line, data, n); line[n] = 0;
  char* nl = strchr(line, '\n'); if (nl) *nl = 0;
  lineReady = true;
}
void startScan() { NimBLEDevice::getScan()->start(0, false, true); }

bool connectRing() {
  if (!client) client = NimBLEDevice::createClient();
  if (!client->connect(found)) return false;
  NimBLERemoteService* svc = client->getService(SVC_UUID);
  if (!svc) { client->disconnect(); return false; }
  NimBLERemoteCharacteristic* tx = svc->getCharacteristic(TX_UUID);
  ringRx = svc->getCharacteristic(RX_UUID);
  if (!tx || !tx->subscribe(true, onNotify)) { client->disconnect(); return false; }
  Serial.println("# ring connected");
  return true;
}
void tellRing(const char* s) { if (ringRx && client && client->isConnected()) ringRx->writeValue((const uint8_t*)s, strlen(s), false); }

// ---- ring messages -> voice / glow -----------------------------------------------------
void handleLine(const char* s) {
  Serial.print("ring: "); Serial.println(s);
  lastRingMsgMs = millis();
  int pct;
  if (sscanf(s, "KNOCK %d", &pct) == 1) playLevel(pct);
  else if (!strncmp(s, "FULL", 4)) { playClip(2, CLIP_FULL); glow = GLOW_FLASH; glowT0 = millis(); ringCharging = false; }
  else if (!strncmp(s, "BAT ", 4) || !strncmp(s, "HELLO ", 6)) {
    bool ch = strstr(s, "CHG") != nullptr;
    if (ch && !ringCharging) playClip(2, CLIP_CHARGING);
    ringCharging = ch; if (glow != GLOW_FLASH) glow = ch ? GLOW_CHARGING : GLOW_IDLE;
  }
  else if (!strncmp(s, "RESERVE", 7)) playClip(2, CLIP_LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_HALL, INPUT_PULLUP); pinMode(PIN_BOARD_LED, OUTPUT);
  ledcAttach(PIN_GLOW, 5000, 8);
  df.begin(9600, SERIAL_8N1, PIN_DF_RX, PIN_DF_TX);
  delay(800);                                          // DFPlayer boot
  dfCmd(0x06, VOLUME);
  NimBLEDevice::init("LYRA-LANTERN");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCB()); scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(80);
  startScan();
  Serial.println("# LYRA-LANTERN ready, looking for LYRA-RING");
}

void loop() {
  glowService();
  if (doConnect) {
    doConnect = false;
    if (connectRing()) {
      if (!everConnected || millis() - lastDisconnectMs > CONNECTED_QUIET_MS) playClip(2, CLIP_CONNECTED);
      everConnected = true; tellRing("BAT?");
    } else startScan();
  }
  if (client && !client->isConnected() && !NimBLEDevice::getScan()->isScanning()) {
    if (ringRx) lastDisconnectMs = millis();
    ringRx = nullptr; startScan();                 // keep the glow: a docked ring is just asleep
  }
  if (ringCharging && !(client && client->isConnected()) && millis() - lastRingMsgMs > CHARGING_HOLD_MS) {
    ringCharging = false; if (glow != GLOW_FLASH) glow = GLOW_IDLE;   // ring left the dock
  }
  if (lineReady) { lineReady = false; handleLine(line); }

  // ring (worn) pressed to the emblem: the hall sensor feels its magnet
  static bool hallWas = false; static uint32_t lastShow = 0;
  bool hall = digitalRead(PIN_HALL) == LOW;
  if (hall && !hallWas && millis() - lastShow > 5000) {
    lastShow = millis();
    playClip(2, CLIP_OATH); tellRing("SHOW CHARGEUP");
    glow = GLOW_FLASH; glowT0 = millis();
  }
  hallWas = hall;
  delay(5);
}
