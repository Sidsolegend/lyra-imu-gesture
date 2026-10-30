// =============================================================================
// LYRA - GLASSES BUILD   (Seeed XIAO ESP32-S3, clip-on teleprompter for the temple arm)
// =============================================================================
// A cue display seen through a 12 mm lens + 45 deg combiner in front of the glasses lens.
// It connects to the ring over BLE as a second host ("LYRA-RING", see lyra_ring.ino for the
// text protocol; the ring takes two hosts, so the phone can stay connected) and steps
// through a script:
//   EVT FLICK,*      -> next cue            TOUCH SWIPE,LR -> next cue
//   EVT TWIST,*      -> previous cue        TOUCH SWIPE,RL -> previous cue
//   EVT PINCH        -> blank / unblank     TOUCH DOUBLE,* -> restart the timer
// Gestures only arrive while the ring is armed (double flick) or in clutch mode, which is
// the one to present with: gestures count only while the thumb is on the emblem. Touch
// LONG/TAP are ignored here because holding the emblem for clutch produces them.
//
// Screen: 4 cue lines + a status line "S<slide>/<slides>  <ring>mm:ss". Ring mark:
//   *  connected, armed or clutch seen     -  connected, not seen armed yet     ?  no ring
// While blanked, the next flick or twist only brings the cue back.
//
// Wiring:  OLED SDA -> D4 (GPIO5), SCL -> D5 (GPIO6), VCC 3V3, GND
//          LiPo on the BAT+/BAT- pads under the board, slide switch in BAT+.
// OLED: 0.42" 72x40 SSD1306 by default (a 72x40 window inside the 128x64 controller RAM);
// build with -DLYRA_OLED_128X32 for the 0.91" strip.
//
// Script: plain text over USB serial (tools/glasses_send_script.py), kept in LittleFS,
// parsed by lyra_cues.h (host-tested in tests/test_glasses_cues.py).
//   ## Title        starts a new slide (the title itself is not shown)
//   text lines      one cue; a blank line ends it. Words wrap to the screen width;
//                   text past the 4th line is cut (the loader prints a warning).
// Serial commands (115200, USB CDC On Boot must be Enabled):
//   SCRIPT ... END   replace the script        NEXT | BACK | BLANK | RESET   as gestures
//   MIRROR H|V 0|1   flip for the combiner     OFFSET <x> <y>   panel window in RAM
//   CONTRAST <0-255> brightness                TEST   border + centre cross for alignment
//   RING <cmd>       forward to the ring (e.g. RING CLUTCH 1)        INFO   state dump
// Mirror, offset and contrast are saved in NVS.
// Auto-off: 10 min with no ring and no serial command -> deep sleep until power-cycled.
// =============================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <Preferences.h>
#include "lyra_cues.h"

#define SVC_UUID "6E400201-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID  "6E400202-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400203-B5A3-F393-E0A9-E50E24DCCA9E"

static constexpr int PIN_SDA = 5, PIN_SCL = 6;           // XIAO D4 / D5
static constexpr uint8_t OLED_ADDR = 0x3C;

#ifdef LYRA_OLED_128X32
static constexpr int RAM_W = 128, RAM_H = 32, W = 128, H = 32;
static constexpr int DEF_OFF_X = 0, DEF_OFF_Y = 0;
#else
static constexpr int RAM_W = 128, RAM_H = 64, W = 72, H = 40;
static constexpr int DEF_OFF_X = 28, DEF_OFF_Y = 0;      // 72x40 glass sits on columns 28..99
#endif
static constexpr int COLS = W / 6, ROWS = H / 8, CUE_LINES = ROWS - 1;   // 6x8 font
static constexpr int MAX_CUES = 200;
static_assert(COLS <= cues::MAX_COLS && CUE_LINES <= cues::MAX_LINES, "screen larger than a Cue holds");

Adafruit_SSD1306 oled(RAM_W, RAM_H, &Wire, -1);
Preferences prefs;

// ---- settings (NVS) -------------------------------------------------------------------
bool mirrorH = true, mirrorV = false;                     // one 45 deg reflection: mirror H
int  offX = DEF_OFF_X, offY = DEF_OFF_Y;
uint8_t contrast = 255;                                   // the combiner loses most of the light

void loadSettings() {
  prefs.begin("glasses", true);
  mirrorH = prefs.getBool("mirH", mirrorH); mirrorV = prefs.getBool("mirV", mirrorV);
  offX = prefs.getInt("offX", offX); offY = prefs.getInt("offY", offY);
  contrast = prefs.getUChar("contrast", contrast);
  prefs.end();
}
void saveSettings() {
  prefs.begin("glasses", false);
  prefs.putBool("mirH", mirrorH); prefs.putBool("mirV", mirrorV);
  prefs.putInt("offX", offX); prefs.putInt("offY", offY); prefs.putUChar("contrast", contrast);
  prefs.end();
}

// ---- script (parsing + wrap in lyra_cues.h) ------------------------------------------
using cues::Cue;
Cue* cues_ = nullptr;                                     // MAX_CUES x 90 B = ~18 KB of heap
int cueCount = 0, slideCount = 0, cur = 0;

void onCueCut(int cueNo, int lineNo) {
  Serial.printf("! cue %d (near line %d) is cut after %d lines\n", cueNo, lineNo, CUE_LINES);
}

void loadScript() {
  cues::Parser p;
  p.begin(cues_, MAX_CUES, COLS, CUE_LINES);
  p.onCut = onCueCut;
  cur = 0;
  File f = LittleFS.open("/script.txt", "r");
  if (!f) p.line("No script. Send one over USB.");
  while (f && f.available()) p.line(f.readStringUntil('\n').c_str());
  if (f) f.close();
  p.finish();
  cueCount = p.count; slideCount = p.slides();
  if (p.dropped) Serial.printf("! script cut at %d cues\n", MAX_CUES);
  Serial.printf("# script: %d cues, %d slides\n", cueCount, slideCount);
}

// ---- display --------------------------------------------------------------------------
bool blanked = false, testPattern = false;
uint32_t timerStartMs = 0;                                // 0 = not started
bool ringConnected = false, ringArmed = false;
// Left on in a bag, BLE scanning drains a 250 mAh cell in a few hours. With no ring and no
// serial command for this long, the glasses switch themselves off (deep sleep with no wake
// source: only the power switch or reset turns them back on). A connected ring never times out.
static constexpr uint32_t AUTO_OFF_MS = 10UL * 60 * 1000;
uint32_t lastActivityMs = 0;

// Where the visible window sits in RAM. Segment remap reverses all 128 columns, so the
// window moves; COM remap only reverses the H multiplexed rows, so it does not.
int winX() { return mirrorH ? RAM_W - W - offX : offX; }
int winY() { return offY; }

void applyPanel() {
#ifndef LYRA_OLED_128X32
  oled.ssd1306_command(SSD1306_SETMULTIPLEX); oled.ssd1306_command(H - 1);   // 40-row glass
  oled.ssd1306_command(0xAD); oled.ssd1306_command(0x30);                  // internal IREF: brighter (as U8g2's 72x40 init)
#endif
  oled.ssd1306_command(mirrorH ? 0xA0 : 0xA1);            // segment remap = left-right
  oled.ssd1306_command(mirrorV ? 0xC0 : 0xC8);            // COM scan = top-bottom
  oled.ssd1306_command(SSD1306_SETCONTRAST); oled.ssd1306_command(contrast);
}

void draw() {
  oled.clearDisplay();
  int x0 = winX(), y0 = winY();
  if (testPattern) {
    oled.drawRect(x0, y0, W, H, SSD1306_WHITE);
    oled.drawFastHLine(x0 + W / 2 - 4, y0 + H / 2, 9, SSD1306_WHITE);
    oled.drawFastVLine(x0 + W / 2, y0 + H / 2 - 4, 9, SSD1306_WHITE);
    oled.setCursor(x0 + 2, y0 + 2); oled.print("TEST");
  } else if (!blanked && cueCount) {
    const Cue& c = cues_[cur];
    for (int i = 0; i < c.lines; i++) { oled.setCursor(x0, y0 + 8 * i); oled.print(c.text[i]); }
    char st[cues::MAX_COLS + 1];
    uint32_t s = timerStartMs ? (millis() - timerStartMs) / 1000 : 0;
    char mark = !ringConnected ? '?' : (ringArmed ? '*' : '-');
    snprintf(st, sizeof(st), "S%d/%d", c.slide, slideCount);
    oled.setCursor(x0, y0 + 8 * CUE_LINES); oled.print(st);
    // "S12/20*04:12" = 12 chars: just fits the 72 px glass
    snprintf(st, sizeof(st), "%c%02lu:%02lu", mark, (unsigned long)(s / 60 % 100), (unsigned long)(s % 60));
    oled.setCursor(x0 + W - 6 * (int)strlen(st), y0 + 8 * CUE_LINES); oled.print(st);
  }
  oled.display();
}

// ---- actions --------------------------------------------------------------------------
// While blanked, next/back only bring the current cue back, so a stray gesture never skips.
void next() { if (blanked) { blanked = false; return; } if (!timerStartMs) timerStartMs = millis(); if (cur < cueCount - 1) cur++; }
void back() { if (blanked) { blanked = false; return; } if (cur > 0) cur--; }
void toggleBlank() { blanked = !blanked; }

void handleRingLine(const char* s) {
  Serial.print("ring: "); Serial.println(s);
  if (!strncmp(s, "EVT ", 4)) ringArmed = true;           // gestures only arrive armed or in clutch
  if (!strncmp(s, "EVT FLICK", 9) || !strcmp(s, "TOUCH SWIPE,LR")) next();
  else if (!strncmp(s, "EVT TWIST", 9) || !strcmp(s, "TOUCH SWIPE,RL")) back();
  else if (!strcmp(s, "EVT PINCH")) toggleBlank();
  else if (!strncmp(s, "TOUCH DOUBLE", 12)) timerStartMs = millis();
  else if (!strcmp(s, "ARMED") || !strcmp(s, "CLUTCH 1")) ringArmed = true;
  else if (!strcmp(s, "DISARMED") || !strcmp(s, "CLUTCH 0")) ringArmed = false;
  else return;
  draw();
}

// ---- BLE client (same pattern as lyra_lantern.ino) --------------------------------------
NimBLEClient* client = nullptr;
NimBLERemoteCharacteristic* ringRx = nullptr;
const NimBLEAdvertisedDevice* found = nullptr;
volatile bool doConnect = false;
QueueHandle_t ringLines;                                  // notify callback -> loop()

class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    if (d->isAdvertisingService(NimBLEUUID(SVC_UUID))) { found = d; doConnect = true; NimBLEDevice::getScan()->stop(); }
  }
};
void onNotify(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool) {
  char line[64];
  size_t n = len < sizeof(line) - 1 ? len : sizeof(line) - 1;
  memcpy(line, data, n); line[n] = 0;
  char* nl = strchr(line, '\n'); if (nl) *nl = 0;
  xQueueSend(ringLines, line, 0);                         // a full queue drops the line, never blocks BLE
}
void startScan() { NimBLEDevice::getScan()->start(0, false, true); }

bool connectRing() {
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setConnectionParams(12, 24, 0, 200);          // 15-30 ms interval: fast cue steps
  }
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

// ---- serial commands ------------------------------------------------------------------
void receiveScript() {
  File f = LittleFS.open("/script.txt", "w");
  if (!f) { Serial.println("! cannot write /script.txt"); return; }
  Serial.println("# send the script, end with a line END");
  String l; uint32_t t0 = millis();
  while (millis() - t0 < 30000) {
    if (!Serial.available()) { delay(1); continue; }
    l = Serial.readStringUntil('\n'); t0 = millis();
    String t = l; t.trim();
    if (t == "END") break;
    f.println(t);
  }
  f.close();
  loadScript(); draw();
}

void handleSerial(String c) {
  c.trim();
  int a, b;
  if (c == "SCRIPT") { receiveScript(); return; }
  else if (c == "NEXT") next();
  else if (c == "BACK") back();
  else if (c == "BLANK") toggleBlank();
  else if (c == "RESET") { timerStartMs = 0; cur = 0; blanked = false; }
  else if (c == "TEST") testPattern = !testPattern;
  else if (c.startsWith("MIRROR H ")) { mirrorH = c.endsWith("1"); saveSettings(); applyPanel(); }
  else if (c.startsWith("MIRROR V ")) { mirrorV = c.endsWith("1"); saveSettings(); applyPanel(); }
  else if (sscanf(c.c_str(), "OFFSET %d %d", &a, &b) == 2) { offX = constrain(a, 0, RAM_W - W); offY = constrain(b, 0, RAM_H - H); saveSettings(); }
  else if (sscanf(c.c_str(), "CONTRAST %d", &a) == 1) { contrast = constrain(a, 0, 255); saveSettings(); applyPanel(); }
  else if (c.startsWith("RING ")) { tellRing(c.substring(5).c_str()); return; }
  else if (c == "INFO") {
    Serial.printf("# cue %d/%d slide %d/%d blank %d ring %s%s mirror H%d V%d offset %d %d contrast %u\n",
                  cur + 1, cueCount, cueCount ? cues_[cur].slide : 0, slideCount, blanked,
                  ringConnected ? "connected" : "none", ringArmed ? " armed" : "",
                  mirrorH, mirrorV, offX, offY, contrast);
    return;
  }
  else { Serial.println("? SCRIPT NEXT BACK BLANK RESET TEST MIRROR OFFSET CONTRAST RING INFO"); return; }
  draw();
}

// ---- main -----------------------------------------------------------------------------
void setup() {
  Serial.setRxBufferSize(4096);                           // a script arrives faster than LittleFS writes
  Serial.begin(115200);
  setCpuFrequencyMhz(80);                                 // enough for BLE + the OLED, saves battery
  cues_ = (Cue*)calloc(MAX_CUES, sizeof(Cue));
  ringLines = xQueueCreate(8, 64);
  loadSettings();
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false)) Serial.println("! OLED not found on D4/D5");
  applyPanel();
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setTextWrap(false);
  if (!LittleFS.begin(true)) Serial.println("! LittleFS mount failed");
  loadScript(); draw();
  NimBLEDevice::init("LYRA-GLASSES");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCB()); scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(80);
  startScan();
  Serial.println("# LYRA-GLASSES ready, looking for LYRA-RING");
}

void powerOff() {
  Serial.println("# auto-off: no ring for 10 min");
  oled.clearDisplay();
  oled.setCursor(winX(), winY()); oled.print("Off. Switch");
  oled.setCursor(winX(), winY() + 8); oled.print("off + on.");
  oled.display(); delay(3000);
  oled.ssd1306_command(SSD1306_DISPLAYOFF);               // panel sleep: a few uA
  NimBLEDevice::deinit(true);
  esp_deep_sleep_start();                                 // no wake source: never returns
}

void loop() {
  if (doConnect) {
    doConnect = false;
    if (connectRing()) { ringConnected = true; lastActivityMs = millis(); draw(); } else startScan();
  }
  if (client && !client->isConnected() && !NimBLEDevice::getScan()->isScanning()) {
    if (ringConnected) { ringConnected = false; ringArmed = false; lastActivityMs = millis(); draw(); }
    ringRx = nullptr; startScan();
  }
  char line[64];
  while (xQueueReceive(ringLines, line, 0) == pdTRUE) { lastActivityMs = millis(); handleRingLine(line); }
  if (Serial.available()) { lastActivityMs = millis(); handleSerial(Serial.readStringUntil('\n')); }
  if (!ringConnected && millis() - lastActivityMs > AUTO_OFF_MS) powerOff();
  static uint32_t lastTickMs = 0;                         // timer: redraw once a second
  if (timerStartMs && !blanked && millis() - lastTickMs >= 1000) { lastTickMs = millis(); draw(); }
  delay(5);
}
