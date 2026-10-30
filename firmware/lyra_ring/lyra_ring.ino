// =============================================================================
// LYRA - RING BUILD   (ESP32-C3FH4 ring PCB: LSM6DS3TR-C, 2x TTP223, LED, coin motor)
// =============================================================================
// Everything the ring does on its own, plus a small BLE text protocol that the phone app
// and the lantern dock both speak.
//
//   Motion gestures   same hybrid classifier as the wrist builds (flick L/R, twist CW/CCW,
//                     pinch, custom templates); double-flick arms/disarms, or hold-to-gesture
//                     ("clutch") mode: gestures only count while a thumb is on the emblem.
//   Touch             2 emblem pads -> tap, double-tap, long-press, swipe L->R / R->L.
//   Knock             LSM6 hardware double-tap: knock the ring on a surface -> it flashes its
//                     battery level, buzzes if low, and sends "BAT <pct> ..." so the lantern /
//                     phone can say "Ring at N% charge".
//   Battery           VBAT sense + TP4054 status: breathing glow while charging, "FULL" when
//                     done, a 5 % reserve that keeps the knock alive on a "dead" ring.
//   Sleep             deep sleep after 5 min idle (connected or not); wakes on either touch
//                     pad or a knock. On the dock: sleeps after 1 min, checks in every 2 min.
//   LED / motor       glow + tick on every gesture/touch, charge-up show on command.
//
// BLE "LYRA-RING" (Nordic-UART style, 6E4002xx): newline-terminated text, up to two hosts
// at once (e.g. phone + lyra_glasses); every notify goes to all subscribed hosts.
//   ring -> host (TX notify):  HELLO <pct> <CHG|DIS>   EVT <gesture>   TOUCH <event>
//                              BAT <pct> <CHG|DIS|FULL>   KNOCK <pct>   FULL
//   host -> ring (RX write):   SHOW CHARGEUP | BUZZ | FIND | BAT? | CLUTCH 1|0 | SLEEP
//
// Build for the dev board (SuperMini + breakouts) with -DLYRA_DEVBOARD: LED on GPIO8
// (active low); otherwise ring PCB pins (see ring/kicad/gen_sch.py).
// =============================================================================
#ifdef LYRA_DEVBOARD
#define LYRA_PIN_LED 8
#define LYRA_LED_ACTIVE_LOW 1
#else
#define LYRA_PIN_LED 10                 // ring PCB: GPIO10 -> 100R -> green LED (active high)
#define LYRA_LED_ACTIVE_LOW 0
#endif

#include <Wire.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "../lib/lyra_imu.h"            // pin constants (SDA 6 / SCL 7) + shared helpers
#include "../lib/lyra_features.h"
#include "../lib/lyra_classifier.h"
#include "../lib/lyra_system.h"
#include "../lib/lyra_led.h"
#include "../lib/lyra_lsm6.h"
#include "../lib/lyra_touch.h"
#include "../lib/lyra_haptic.h"
#include "../lib/lyra_power.h"
#include "../lib/lyra_ota.h"             // firmware update over BLE (ring is sealed)

using namespace lyra;

#define SVC_UUID "6E400201-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID  "6E400202-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400203-B5A3-F393-E0A9-E50E24DCCA9E"

static constexpr uint32_t IDLE_SLEEP_MS   = 5UL * 60 * 1000;   // deep sleep after 5 min idle
// Dock mode. The charger gives ~21 mA and the ring's own load comes out of that: awake (13-23 mA
// with the CPU idle, ESP32-C3 datasheet table 5-8, plus BLE) it would eat the whole charge current
// and the TP4054 would never see its 2.1 mA end-of-charge current. So on the dock the ring sleeps
// (~tens of uA), wakes every DOCK_SLEEP_S to report to the lantern for a few seconds, sleeps again.
static constexpr uint32_t DOCK_IDLE_MS    = 60UL * 1000;       // awake on the dock this long after activity
static constexpr uint64_t DOCK_SLEEP_US   = 120ULL * 1000000;  // check-in period while charging
static constexpr uint32_t DOCK_AWAKE_MS   = 6000;              // check-in: wait this long for the lantern
static constexpr uint32_t RESERVE_AWAKE_MS = 8000;            // knock in reserve: time to report
static constexpr uint32_t DOUBLE_FLICK_MS = 1400;

NimBLECharacteristic* txChar = nullptr;
volatile bool clientConnected = false;
Preferences prefs;

GyroBias     bias;
SampleClock  clock_;
Stage1Params s1;
TreeParams   tree;
GTemplate    customGestures[MAX_CUSTOM];
int          customCount = 0;

float accelBuf[WINDOW], gxBuf[WINDOW], gyBuf[WINDOW], gzBuf[WINDOW];
bool  capturing = false;
int   captureIndex = 0;
float lastDt = DT_TRAIN_S;

bool     armed = false, clutchMode = false;
bool     flickPending = false, pendingFlickRight = false;
uint32_t flickPendingMs = 0, lastActivityMs = 0, lastBattMs = 0;

// command from the BLE RX callback, handled in loop() (keeps the BLE task short)
char     pendingCmd[32] = {0};
volatile bool cmdReady = false;

// ---- BLE ---------------------------------------------------------------------
void send(const char* s) {
  Serial.println(s);
  if (txChar && clientConnected) {
    char out[64]; int n = snprintf(out, sizeof(out), "%s\n", s);
    txChar->setValue((uint8_t*)out, n); txChar->notify();
  }
}
void sendBattery(const char* tag) {
  char b[40];
  snprintf(b, sizeof(b), "%s %u %s", tag, batt.pct, batt.charging ? "CHG" : (batt.mv > 4150 ? "FULL" : "DIS"));
  send(b);
}

// Up to MAX_HOSTS at once (phone/lantern + glasses): keep advertising until the second host
// joins. NimBLE updates the connected count before either callback runs.
static constexpr uint8_t MAX_HOSTS = 2;
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& c) override {
    clientConnected = true;
    if (s->getConnectedCount() < MAX_HOSTS) NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& c, int reason) override {
    clientConnected = s->getConnectedCount() > 0; NimBLEDevice::startAdvertising();
  }
};
class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo& c) override {
    std::string v = ch->getValue();
    size_t n = v.size() < sizeof(pendingCmd) - 1 ? v.size() : sizeof(pendingCmd) - 1;
    memcpy(pendingCmd, v.data(), n); pendingCmd[n] = 0; cmdReady = true;
  }
};

void bleInit() {
  NimBLEDevice::init("LYRA-RING");
  NimBLEDevice::setPower(0);                       // 0 dBm: desk range, gentler on a 30 mAh cell
  NimBLEDevice::setMTU(517);                       // big OTA chunks
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  NimBLEService* svc = server->createService(SVC_UUID);
  txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* rx = svc->createCharacteristic(RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCB());
  lyra_ota::sendLine = send;
  lyra_ota::powerOk = [] { return batt.charging || batt.mv == 0 || batt.pct >= OTA_MIN_PCT; };
  lyra_ota::addTo(svc);
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID); adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();
}

// ---- feedback + sleep ----------------------------------------------------------
void feedback() { ledGesture(); if (!batteryInReserve()) hapticTick(); lastActivityMs = millis(); }

void goToSleep(uint64_t timerUs = 0) {
  if (lyra_ota::busy()) return;                    // never sleep mid-update
  send("SLEEP");
  ledBreathe(false); ledWriteRaw(0); hapticWrite(0);
  NimBLEDevice::deinit(true);
  lsmConfigureSleep();
  deepSleepUntilTouchOrKnock(timerUs, touchSinglePad);   // never returns (wake = reboot)
}

// knock: show + report the battery (the "Ring at N% charge" moment)
void onKnock() {
  batteryUpdate();
  if (batteryInReserve()) { ledLowFlicker(); hapticLowBat(); }
  else ledLevel(batt.pct);
  char b[24]; snprintf(b, sizeof(b), "KNOCK %u", batt.pct); send(b);
  sendBattery("BAT");
  lastActivityMs = millis();
}

// ---- commands -------------------------------------------------------------------
void handleCommand(const char* c) {
  lastActivityMs = millis();
  if (!strncmp(c, "SHOW CHARGEUP", 13)) { ledChargeUp(); if (!batteryInReserve()) hapticChargeUp(); }
  else if (!strncmp(c, "BUZZ", 4)) hapticDouble();
  else if (!strncmp(c, "FIND", 4)) { hapticLong(); ledLevel(100); }
  else if (!strncmp(c, "BAT?", 4)) { batteryUpdate(); sendBattery("BAT"); }
  else if (!strncmp(c, "CLUTCH", 6)) {
    clutchMode = c[7] == '1';
    prefs.begin(LYRA_NVS_NS, false); prefs.putBool("clutch", clutchMode); prefs.end();
    send(clutchMode ? "CLUTCH 1" : "CLUTCH 0");
  }
  else if (!strncmp(c, "SLEEP", 5)) goToSleep();
  lastActivityMs = millis();
}

// ---- gestures (same pipeline as the wrist builds) --------------------------------
void act(Gesture g, bool flickRight, bool twistCW) {
  if (clutchMode) {                                // clutch: every gesture counts, no arming
    if (g == G_FLICK) send(flickRight ? "EVT FLICK,R" : "EVT FLICK,L");
    else if (g == G_PINCH) send("EVT PINCH");
    else if (g == G_TWIST) send(twistCW ? "EVT TWIST,CW" : "EVT TWIST,CCW");
    return;
  }
  if (g == G_FLICK) {
    uint32_t now = millis();
    if (flickPending && now - flickPendingMs <= DOUBLE_FLICK_MS) {
      flickPending = false; armed = !armed; ledArmed(armed);
      send(armed ? "ARMED" : "DISARMED");
    } else { flickPending = true; flickPendingMs = now; pendingFlickRight = flickRight; }
    return;
  }
  if (!armed) return;
  if (g == G_PINCH) send("EVT PINCH");
  else if (g == G_TWIST) send(twistCW ? "EVT TWIST,CW" : "EVT TWIST,CCW");
}

void servicePendingFlick() {
  if (flickPending && millis() - flickPendingMs > DOUBLE_FLICK_MS) {
    flickPending = false;
    if (armed) send(pendingFlickRight ? "EVT FLICK,R" : "EVT FLICK,L");
  }
}

void analyzeAndAct() {
  Features f = extractFeatures(accelBuf, gxBuf, gyBuf, gzBuf, s1.minGyroFloor, lastDt);
  Gesture g = classifyHybrid(f, s1, tree);
  if (clutchMode && !touchHeld()) return;          // clutch: thumb must be on the emblem
  if (g != G_NONE) feedback();
  if (customCount > 0 && g == G_NONE && (armed || clutchMode)) {
    float feat[NUM_FEATURES]; featuresToVec(f, feat);
    int m = matchCustom(customGestures, customCount, feat);
    if (m >= 0) { feedback(); char ev[40]; snprintf(ev, sizeof(ev), "EVT CUSTOM,%s", customGestures[m].name); send(ev); return; }
  }
  act(g, f.peakGYSigned > 0, f.peakGZSigned > 0);
}

void loadConfigNVS() {
  prefs.begin(LYRA_NVS_NS, true);
  loadStage1NVS(prefs, s1); loadTreeNVS(prefs, tree);
  customCount = loadCustomNVS(prefs, customGestures);
  clutchMode = prefs.getBool("clutch", false);
  prefs.end();
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  systemBoot();
  ledInit(); hapticInit(); touchInit(); powerPinsInit();
  WakeReason wr = wakeReason();

  if (!lsmBringUp(LYRA_PIN_SDA, LYRA_PIN_SCL)) Serial.println("# LSM6 IMU not found - check I2C");
  touchSetSinglePad(lsmIsDSV);                     // v1.5 board (LSM6DSV) has one touch pad, no swipe
  powerSetBoardV15(lsmIsDSV);                      // ...and its charger status on GPIO1
  batteryUpdate();                                 // after the board check: v1 / v1.5 use different charger pins
  lsmKnocked();                                    // clear the latched knock that woke us

  // dock check-in (timer wake while charging, or v1.5 put on the dock while asleep): report to the
  // lantern, then straight back to sleep
  if (wr == WAKE_TIMER || wr == WAKE_CHARGE) {
    bleInit();
    uint32_t t0 = millis();
    while (!clientConnected && millis() - t0 < DOCK_AWAKE_MS) delay(10);
    if (clientConnected) {
      delay(300);                                    // lantern subscribes to notifications
      if (batt.justFull) send("FULL");
      sendBattery("BAT");
      delay(300);
    }
    goToSleep(batt.charging ? DOCK_SLEEP_US : 0);   // full / lifted off: normal sleep (touch / knock)
  }

  // "dead" ring (reserve): wake only to answer a knock, then straight back to sleep
  if (batteryInReserve()) {
    if (wr == WAKE_KNOCK) {
      bleInit();
      uint32_t t0 = millis();
      ledLowFlicker(); hapticLowBat();
      while (millis() - t0 < RESERVE_AWAKE_MS) {   // give the lantern/phone time to connect
        ledService(); hapticService();
        if (clientConnected) { delay(300); onKnock(); delay(500); break; }
        delay(10);
      }
    }
    goToSleep();
  }

  Serial.println("# hold still - gyro zero...");
  lsmCalibrateGyro(bias);
  loadConfigNVS();
  bleInit();
  wdtInit();
  lastActivityMs = millis();
  if (wr == WAKE_KNOCK) onKnock();
  else if (wr == WAKE_TOUCH) feedback();
  Serial.println(clutchMode ? "# LYRA-RING ready (clutch mode)" : "# LYRA-RING ready (double-flick to arm)");
}

void loop() {
  wdtFeed();
  ledService(); hapticService();
  lyra_ota::service();                             // reboot after a finished update

  if (cmdReady) { cmdReady = false; handleCommand(pendingCmd); }

  static bool helloSent = false;
  if (clientConnected && !helloSent) { helloSent = true; sendBattery("HELLO"); }
  if (!clientConnected) helloSent = false;

  // battery / charging, once a second
  if (millis() - lastBattMs > 1000) {
    lastBattMs = millis();
    bool wasCharging = batt.charging;
    batteryUpdate();
    ledBreathe(batt.charging);
    if (batt.charging != wasCharging) sendBattery("BAT");
    if (batt.justFull) { send("FULL"); ledChargeUp(); }
    if (batteryInReserve()) { send("RESERVE"); goToSleep(); }
  }

  // touch
  TouchEvent te = touchService();
  if (te != T_NONE) { feedback(); char b[24]; snprintf(b, sizeof(b), "TOUCH %s", touchName(te)); send(b); }

  // knock (INT1 high = latched double tap)
  if (digitalRead(LYRA_PIN_IMU_INT) == HIGH && lsmKnocked()) onKnock();

  // idle -> deep sleep (never while charging or connected)
  // (a connected lantern is not activity: it is always connected on the desk, and staying awake
  //  for it would drain the 30 mAh cell in ~1.5 h)
  if (batt.charging) { if (millis() - lastActivityMs > DOCK_IDLE_MS) goToSleep(DOCK_SLEEP_US); }
  else if (millis() - lastActivityMs > IDLE_SLEEP_MS) goToSleep();

  servicePendingFlick();

  // motion sampling + gesture capture (fixed 10 ms clock, same as the wrist builds)
  float dt;
  if (!clock_.due(&dt)) return;
  LsmSample s;
  if (!lsmReadSample(s)) { capturing = false; return; }
  float accelMag = sqrt(s.a[0] * s.a[0] + s.a[1] * s.a[1] + s.a[2] * s.a[2]);
  float gx = (s.g[0] - bias.GX0) / LSM_GYRO_LSB_PER_DPS;
  float gy = (s.g[1] - bias.GY0) / LSM_GYRO_LSB_PER_DPS;
  float gz = (s.g[2] - bias.GZ0) / LSM_GYRO_LSB_PER_DPS;
  if (!capturing) zuptUpdate(bias, accelMag, gx, gy, gz, s.g[0], s.g[1], s.g[2]);
  if (!capturing && accelMag > TRIGGER_THRESHOLD) { capturing = true; captureIndex = 0; lastDt = dt; lastActivityMs = millis(); }
  if (capturing) {
    if (captureIndex < WINDOW) {
      accelBuf[captureIndex] = accelMag; gxBuf[captureIndex] = gx; gyBuf[captureIndex] = gy; gzBuf[captureIndex] = gz;
      captureIndex++;
    } else { analyzeAndAct(); capturing = false; lastActivityMs = millis(); }
  }
}
