// =============================================================================
// LYRA - EV3 BRIDGE BUILD (optimized)   output stage: BLE-UART gesture events
// =============================================================================
// Optimized duplicate of firmware/lyra_ev3_bridge/...ino. Shared DSP now lives in
// ../lib/*.h; this file is ONLY the event-emit stage:
//   sample -> capture -> hybrid classify -> ARM/double-flick -> notify one event.
//
// Review fixes vs the original:
//   #2 fixed-Δt sampling (SampleClock) instead of delay(10)
//   #3 events emitted via snprintf into a fixed buffer, not String concatenation
//      (the original built `String("CUSTOM,") + name` and `String(s)+"\n"` per event)
//   #4 shared mpuBringUp() cold-boot recovery+retry (original had a bare
//      Wire.begin(21,22) + single 0x6B wake, no recovery - lines 327-331)
//   #5 stage-2 tree splits loaded from NVS so app calibration reaches the arm too
//
// BLE (Nordic-UART-style):  DEDICATED LYRA-ARM service 6E400101..., TX 6E400103
//   (notify) as "LYRA-ARM". T14: this base (6E4001xx) is DISTINCT from the
//   calibration build's LYRA-CAL (6E400001.../...0003) so a host can't
//   cross-connect the two once both exist.
// EVENTS: over BLE, the bare event string (newline-terminated). Over SERIAL, the
//   same event on a STRUCTURED line prefixed "EVT " (E7/T13) so the bridge no
//   longer parses the human "->" debug cosmetic. Event vocabulary (only when
//   ARMED): FLICK,R / FLICK,L / TWIST,CW / TWIST,CCW / PINCH / CUSTOM,<name> ;
//   plus ARMED / DISARMED status toggles.
// =============================================================================
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "../lib/lyra_imu.h"
#include "../lib/lyra_features.h"
#include "../lib/lyra_classifier.h"
#include "../lib/lyra_system.h"     // change #8: wdt + reset forensics + I2C recovery
#include "../lib/lyra_led.h"        // glow on every recognised gesture

using namespace lyra;

// T14: dedicated LYRA-ARM UUIDs (6E4001xx), distinct from LYRA-CAL's 6E400001.
#define SVC_UUID "6E400101-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400103-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* txChar = nullptr;
bool clientConnected = false;

Preferences prefs;

// ---- runtime state -----------------------------------------------------------
GyroBias     bias;
SampleClock  clock_;
I2cHealth    i2cHealth;             // change #8
Stage1Params s1;
TreeParams   tree;
GTemplate    customGestures[MAX_CUSTOM];
int          customCount = 0;

float accelBuf[WINDOW], gxBuf[WINDOW], gyBuf[WINDOW], gzBuf[WINDOW];
bool  capturing = false;
int   captureIndex = 0;
float lastDt = DT_TRAIN_S;

bool     armed = false;
bool     flickPending = false;
bool     pendingFlickRight = false;
uint32_t flickPendingMs = 0;
const uint32_t DOUBLE_FLICK_MS = 1400;

// ---- BLE server callbacks (NimBLE-Arduino 2.x signatures) --------------------
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override { clientConnected = true; }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
    clientConnected = false;
    NimBLEDevice::startAdvertising();
  }
};

// change #3: emit with snprintf into a fixed buffer, no String.
// E7/T13: the SERIAL line is now the structured, machine-facing "EVT <event>"
// (the bridge parses this prefix instead of the old "->" debug cosmetic). Over
// BLE the bare event string is sent, unchanged. A given firmware emits exactly
// ONE serial form, so the bridge never double-fires (it accepts "EVT " first,
// "->" only as a legacy fallback for old firmware).
void notifyLine(const char* s) {
  Serial.print("EVT "); Serial.println(s);             // E7: structured serial event
  if (txChar != nullptr) {
    char out[48];
    int len = snprintf(out, sizeof(out), "%s\n", s);   // newline-delimited, matches the bridge
    txChar->setValue((uint8_t*)out, len);
    txChar->notify();                                  // safe no-op when no host subscribed
  }
}

void loadConfigNVS() {
  prefs.begin(LYRA_NVS_NS, true);
  loadStage1NVS(prefs, s1);
  loadTreeNVS(prefs, tree);           // change #5
  customCount = loadCustomNVS(prefs, customGestures);
  prefs.end();
}

// Act on a classified gesture: emit an event (only when armed). Flicks deferred
// so a double-flick can arm/disarm instead.
void act(Gesture g, bool flickRight, bool twistCW) {
  if (g == G_FLICK) {
    uint32_t now = millis();
    if (flickPending && (now - flickPendingMs) <= DOUBLE_FLICK_MS) {
      flickPending = false;
      armed = !armed;
      ledArmed(armed);
      notifyLine(armed ? "ARMED" : "DISARMED");
      Serial.println(armed ? "*** ARMED ***" : "*** DISARMED ***");
    } else {
      flickPending = true;
      flickPendingMs = now;
      pendingFlickRight = flickRight;
    }
    return;
  }
  if (!armed) return;
  if (g == G_PINCH) notifyLine("PINCH");
  else if (g == G_TWIST) notifyLine(twistCW ? "TWIST,CW" : "TWIST,CCW");
}

void servicePendingFlick() {
  if (flickPending && (millis() - flickPendingMs) > DOUBLE_FLICK_MS) {
    flickPending = false;
    if (armed) notifyLine(pendingFlickRight ? "FLICK,R" : "FLICK,L");
  }
}

// =============================================================================
void analyzeAndAct() {
  Features f = extractFeatures(accelBuf, gxBuf, gyBuf, gzBuf, s1.minGyroFloor, lastDt);
  Gesture g = classifyHybrid(f, s1, tree);
  if (g != G_NONE) ledGesture();

  bool flickRight = (f.peakGYSigned > 0);
  bool twistCW    = (f.peakGZSigned > 0);

  Serial.print("gesture: "); Serial.print(gestureName(g));
  Serial.print("  (peakGyro="); Serial.print(f.peakGyro, 0);
  Serial.print(" ratio="); Serial.print(f.ratio, 2);
  Serial.print(" dur="); Serial.print(f.durationCount, 1);
  Serial.print(" aStd="); Serial.print(f.accelStd, 2); Serial.println(")");

  if (armed && customCount > 0 && g == G_NONE) {
    float feat[NUM_FEATURES]; featuresToVec(f, feat);
    int m = matchCustom(customGestures, customCount, feat);
    if (m >= 0) {
      ledGesture();
      char ev[32];
      snprintf(ev, sizeof(ev), "CUSTOM,%s", customGestures[m].name);   // change #3
      notifyLine(ev);
      return;
    }
  }
  act(g, flickRight, twistCW);
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  systemBoot();                       // change #8: reset forensics + power opts
  ledInit();                          // gesture-feedback LED

  // change #4: shared cold-boot bus recovery + retry (original had none).
  mpuBringUp(LYRA_PIN_SDA, LYRA_PIN_SCL);

  Serial.println("# Calibrating gyro zero -- hold the device STILL...");
  delay(300);
  calibrateGyroBiasStable(bias, 200);   // T19/C4: reject-and-retry if the wrist moved

  loadConfigNVS();
  Serial.print("# active profile: MIN_GYRO_FLOOR="); Serial.print(s1.minGyroFloor);
  Serial.print(" RATIO_CUT="); Serial.print(s1.ratioCut);
  Serial.print("  customGestures="); Serial.println(customCount);

  NimBLEDevice::init("LYRA-ARM");
  NimBLEDevice::setMTU(185);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  NimBLEService* svc = server->createService(SVC_UUID);
  txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();

  wdtInit();                          // change #8: armed AFTER blocking bring-up
  Serial.println("# LYRA-ARM ready. Advertising as LYRA-ARM. Run bridge/lyra_to_ev3.py.");
  Serial.println("# Double-flick to ARM, then flick/twist/pinch to drive the arm.");
}

// =============================================================================
void loop() {
  wdtFeed();                          // change #8
  ledService();                       // advance the gesture glow (non-blocking)

  while (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();   // cold-path config only
    if (s.equalsIgnoreCase("@zero")) {
      Serial.println("# manual re-zero -- hold still...");
      delay(200); calibrateGyroBias(bias, 200);
    }
  }

  servicePendingFlick();

  // change #2: fixed-Δt gate replaces delay(10).
  float dt;
  if (!clock_.due(&dt)) return;

  uint8_t data[14];
  if (!i2cGuardedRead(i2cHealth, 0x3B, 14, data, LYRA_PIN_SDA, LYRA_PIN_SCL)) {
    capturing = false;                // change #8: never emit from a torn window
    return;
  }

  int16_t ax = (data[0] << 8) | data[1];
  int16_t ay = (data[2] << 8) | data[3];
  int16_t az = (data[4] << 8) | data[5];
  int16_t gx = (data[8] << 8) | data[9];
  int16_t gy = (data[10] << 8) | data[11];
  int16_t gz = (data[12] << 8) | data[13];

  float ax_g = ax / ACCEL_SCALE, ay_g = ay / ACCEL_SCALE, az_g = az / ACCEL_SCALE;
  float accelMag = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float gx_dps = (gx - bias.GX0) / GYRO_SCALE;
  float gy_dps = (gy - bias.GY0) / GYRO_SCALE;
  float gz_dps = (gz - bias.GZ0) / GYRO_SCALE;

  if (!capturing) zuptUpdate(bias, accelMag, gx_dps, gy_dps, gz_dps, gx, gy, gz);

  if (!capturing && accelMag > TRIGGER_THRESHOLD) {
    capturing = true; captureIndex = 0; lastDt = dt;
  }
  if (capturing) {
    if (captureIndex < WINDOW) {
      accelBuf[captureIndex] = accelMag;
      gxBuf[captureIndex] = gx_dps;
      gyBuf[captureIndex] = gy_dps;
      gzBuf[captureIndex] = gz_dps;
      captureIndex++;
    } else {
      analyzeAndAct();
      capturing = false;
    }
  }
}
