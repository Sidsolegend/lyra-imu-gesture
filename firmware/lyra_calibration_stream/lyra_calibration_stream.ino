// =============================================================================
// LYRA - CALIBRATION-STREAM BUILD (optimized)   output stage: BLE GATT + write-back
// =============================================================================
// Optimized duplicate of firmware/lyra_calibration_stream_new/...ino. Shared DSP
// now lives in ../lib/*.h; this file is ONLY the streaming + config-intake stage:
//   sample -> capture window -> fast cascade -> notify one CSV row -> app.
//   RX write-back persists the app's profile / tree / custom gestures to NVS.
//
// Review fixes vs the original:
//   #2 fixed-Δt sampling (SampleClock) instead of delay(10) in the hot loop
//   #3 the streamed CSV row is built with snprintf into a fixed char buffer
//      (no per-gesture String concatenation -> no heap fragmentation)
//   #4 shared mpuBringUp() cold-boot recovery+retry - THIS was the fragile build
//      the app onboards through; the original had a bare Wire.begin(21,22) + single
//      0x6B wake and NO recovery/retry (lines 327-331). Now hardened like the rest.
//   #5 new 'T' write-back command lets the app push the stage-2 tree splits, which
//      the optimized daily build reads from NVS (so calibration shapes daily behavior).
//
// BLE (Nordic-UART-style):
//   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E   (advertised as "LYRA-CAL")
//   TX 6E400003 (notify, device->app)   RX 6E400002 (write, app->device)
//
// WRITE-BACK PROTOCOL (newline-terminated ASCII on RX):
//   P,<gyroFloor>,<ratioCut>,<pinchGate>                       set profile
//   T,<dur>,<aMean>,<pgLo>,<spike>,<aStd>,<pgHi>               set stage-2 tree (change #5)
//   G,<name>,<action>,<m0..m8>,<s0..s8>,<thresh>              add a custom template
//   C                                                          clear custom gestures
// =============================================================================
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "../lib/lyra_imu.h"
#include "../lib/lyra_features.h"
#include "../lib/lyra_classifier.h"
#include "../lib/lyra_system.h"     // change #8: wdt + reset forensics + I2C recovery

using namespace lyra;

#define SVC_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* txChar = nullptr;
NimBLECharacteristic* rxChar = nullptr;
bool clientConnected = false;

Preferences prefs;

// ---- runtime state -----------------------------------------------------------
GyroBias    bias;
SampleClock clock_;
I2cHealth   i2cHealth;              // change #8
GTemplate   customGestures[MAX_CUSTOM];
int         customCount = 0;

float accelBuf[WINDOW], gxBuf[WINDOW], gyBuf[WINDOW], gzBuf[WINDOW];
bool  capturing = false;
int   captureIndex = 0;
float lastDt = DT_TRAIN_S;

// fast-cascade profile (writable via RX + NVS), same defaults as the original.
float gyroFloor = 220.0f;
float ratioCut  = 1.2f;
float pinchGate = 0.40f;
const float PINCH_TWIST_BOUNDARY = 700.0f;  // FIXED (project decision)

// T15/R5: the streamed CSV is TRAINING data, and durationCount = "samples above
// the gyro floor" is only comparable across the corpus if counted above the SAME
// floor every row was collected at. The whole committed dataset used the fixed
// 220 dps floor, so the streamed durationCount is always counted at this fixed
// value -- NOT at the app-tuned `gyroFloor` (which the classifier still uses for
// its NONE gate). Without this, a wearer who tunes gyroFloor would log rows whose
// durationCount is feature-incompatible with everything already in the corpus.
const float TRAIN_GYRO_FLOOR = 220.0f;
bool durFloorWarned = false;

String currentLabel = "unlabeled";
String currentSubject = "me";

// ---- BLE server callbacks (NimBLE-Arduino 2.x signatures, `override`) --------
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override { clientConnected = true; }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
    clientConnected = false;
    NimBLEDevice::startAdvertising();     // 2.x no longer auto-restarts on disconnect
  }
};

// ---- NVS persistence (config intake = cold path) ----------------------------
void saveProfileNVS() {
  prefs.begin(LYRA_NVS_NS, false);
  prefs.putFloat("gf", gyroFloor);
  prefs.putFloat("rc", ratioCut);
  prefs.putFloat("ps", pinchGate);
  prefs.putBool("hasP", true);
  prefs.end();
}

// change #5: persist the six stage-2 tree splits under their documented keys.
void saveTreeNVS(const TreeParams& tp) {
  prefs.begin(LYRA_NVS_NS, false);
  prefs.putFloat("tDur",   tp.durSplit);
  prefs.putFloat("tAmean", tp.aMeanSplit);
  prefs.putFloat("tPgLo",  tp.pgLowSplit);
  prefs.putFloat("tSpike", tp.spikeSplit);
  prefs.putFloat("tAstd",  tp.aStdSplit);
  prefs.putFloat("tPgHi",  tp.pgHighSplit);
  prefs.end();
}

void saveTemplatesNVS() {
  // T9/N1: write VERSIONED records via the shared helper so save and load agree.
  // (This build used to putBytes() the bare 96-byte struct while loadConfigNVS()
  // reads with the versioned loadCustomNVS() -- so a saved template would fail
  // the version check and be discarded on the next boot. saveCustomNVS() stamps
  // each record [GTEMPLATE_SCHEMA_VERSION][GTemplate] and writes the count.)
  prefs.begin(LYRA_NVS_NS, false);
  saveCustomNVS(prefs, customGestures, customCount);
  prefs.end();
}

void loadConfigNVS() {
  prefs.begin(LYRA_NVS_NS, true);
  if (prefs.getBool("hasP", false)) {
    gyroFloor = prefs.getFloat("gf", gyroFloor);
    ratioCut  = prefs.getFloat("rc", ratioCut);
    pinchGate = prefs.getFloat("ps", pinchGate);
  }
  customCount = loadCustomNVS(prefs, customGestures);
  prefs.end();
}

// ---- write-back parsing (cold path: runs only on an app write) --------------
int splitCsv(const String& s, String* out, int maxTok) {
  int n = 0, start = 0;
  for (int i = 0; i <= s.length() && n < maxTok; i++) {
    if (i == s.length() || s.charAt(i) == ',') { out[n++] = s.substring(start, i); start = i + 1; }
  }
  return n;
}

void handleConfigLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  String tok[24];
  int n = splitCsv(line, tok, 24);
  char cmd = tok[0].length() ? tok[0].charAt(0) : '?';

  if (cmd == 'P' && n >= 4) {
    gyroFloor = tok[1].toFloat(); ratioCut = tok[2].toFloat(); pinchGate = tok[3].toFloat();
    durFloorWarned = false;              // T15: re-arm the fixed-floor note for the new profile
    saveProfileNVS();
    Serial.print("# got profile: gyroFloor="); Serial.print(gyroFloor);
    Serial.print(" ratioCut="); Serial.print(ratioCut);
    Serial.print(" pinchGate="); Serial.println(pinchGate);
  } else if (cmd == 'T' && n >= 7) {                     // change #5
    TreeParams tp;
    tp.durSplit=tok[1].toFloat(); tp.aMeanSplit=tok[2].toFloat(); tp.pgLowSplit=tok[3].toFloat();
    tp.spikeSplit=tok[4].toFloat(); tp.aStdSplit=tok[5].toFloat(); tp.pgHighSplit=tok[6].toFloat();
    saveTreeNVS(tp);
    Serial.println("# got stage-2 tree splits -> NVS (applied by the daily build)");
  } else if (cmd == 'G' && n >= 22) {
    if (customCount >= MAX_CUSTOM) { Serial.println("# custom store full -- send 'C' to clear"); return; }
    GTemplate t; memset(&t, 0, sizeof(t));
    tok[1].toCharArray(t.name, sizeof(t.name));
    t.action = tok[2].length() ? tok[2].charAt(0) : 'P';
    for (int j = 0; j < NUM_FEATURES; j++) t.mean[j] = tok[3 + j].toFloat();
    for (int j = 0; j < NUM_FEATURES; j++) t.std[j]  = tok[12 + j].toFloat();
    t.thresh = tok[21].toFloat();
    customGestures[customCount++] = t;
    saveTemplatesNVS();
    Serial.print("# stored gesture: "); Serial.print(t.name);
    Serial.print(" ("); Serial.print(customCount); Serial.println(" total)");
  } else if (cmd == 'C') {
    customCount = 0; saveTemplatesNVS();
    Serial.println("# cleared custom gestures");
  } else {
    Serial.print("# unrecognized config line: "); Serial.println(line);
  }
}

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    String buf = String(c->getValue().c_str());
    int start = 0;
    for (int i = 0; i <= buf.length(); i++) {
      if (i == buf.length() || buf.charAt(i) == '\n') { handleConfigLine(buf.substring(start, i)); start = i + 1; }
    }
  }
};

// ---- change #3: stream a row built with snprintf, no String concatenation ----
void streamRow(const char* row) {
  Serial.println(row);
  if (txChar != nullptr) {
    char out[200];
    int len = snprintf(out, sizeof(out), "%s\n", row);   // newline-delimited for the app
    txChar->setValue((uint8_t*)out, len);
    txChar->notify();                                    // safe no-op when unsubscribed
  }
}

// =============================================================================
void analyzeAndStream() {
  // T15/R5: count durationCount at the FIXED training floor so streamed rows stay
  // corpus-compatible even after the app tunes gyroFloor. peakGyro/ratio/accel*
  // don't depend on the floor arg, so the fast cascade below (which gates on the
  // active gyroFloor, not durationCount) is unaffected.
  Features f = extractFeatures(accelBuf, gxBuf, gyBuf, gzBuf, TRAIN_GYRO_FLOOR, lastDt);
  if (gyroFloor != TRAIN_GYRO_FLOOR && !durFloorWarned) {
    Serial.print("# NOTE: durationCount is logged at the FIXED training floor ");
    Serial.print(TRAIN_GYRO_FLOOR); Serial.print(" dps (active gyroFloor=");
    Serial.print(gyroFloor); Serial.println(") so rows stay comparable to the corpus.");
    durFloorWarned = true;
  }
  Gesture g = classifyFast(f, gyroFloor, ratioCut, pinchGate, PINCH_TWIST_BOUNDARY);

  // change #3: fixed-buffer CSV, same 12 columns + precisions as the original.
  // durationCount/spikeCount printed as integers to keep the app's parser happy
  // (they are Δt-normalized floats internally; identity on the reference board).
  char row[200];
  snprintf(row, sizeof(row),
           "%s,%s,%.1f,%.3f,%ld,%.1f,%.1f,%.3f,%.3f,%.3f,%ld,%s",
           currentSubject.c_str(), currentLabel.c_str(),
           f.peakGyro, f.ratio, lround(f.durationCount),
           f.peakGYSigned, f.peakGZSigned,
           f.peakAccel, f.accelMean, f.accelStd,
           lround(f.spikeCount), gestureName(g));
  streamRow(row);
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  systemBoot();                       // change #8: reset forensics + power opts

  // change #4: shared cold-boot bus recovery + retry (was the fragile build).
  // Original used pins 21/22; kept here for the big-ESP32 the app was onboarded on.
  mpuBringUp(LYRA_PIN_SDA, LYRA_PIN_SCL);

  Serial.println("# Calibrating gyro zero -- hold the device STILL...");
  delay(300);
  calibrateGyroBiasStable(bias, 200);   // T19/C4: reject-and-retry if the wrist moved

  loadConfigNVS();

  NimBLEDevice::init("LYRA-CAL");
  NimBLEDevice::setMTU(185);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  NimBLEService* svc = server->createService(SVC_UUID);
  txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  rxChar = svc->createCharacteristic(RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxChar->setCallbacks(new RxCB());
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();

  wdtInit();                          // change #8: armed AFTER blocking bring-up

  Serial.println("subject,label,peakGyro,ratio,durationCount,gySigned,gzSigned,peakAccel,accelMean,accelStd,spikeCount,classified");
  Serial.println("# CALIBRATION-STREAM READY. Advertising as LYRA-CAL. Connect from the app.");
  Serial.print("# active profile: gyroFloor="); Serial.print(gyroFloor);
  Serial.print(" ratioCut="); Serial.print(ratioCut);
  Serial.print(" pinchGate="); Serial.print(pinchGate);
  Serial.print("  customGestures="); Serial.println(customCount);
}

// =============================================================================
void loop() {
  wdtFeed();                          // change #8

  while (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();   // cold-path label/subject control
    if (s.length() > 0) {
      if (s.equalsIgnoreCase("@zero")) {
        Serial.println("# manual re-zero -- hold still...");
        delay(200); calibrateGyroBias(bias, 200);
      } else if (s.charAt(0) == '@') {
        currentSubject = s.substring(1);
        Serial.print("# subject set: "); Serial.println(currentSubject);
      } else {
        currentLabel = s;
        Serial.print("# label set: "); Serial.println(currentLabel);
      }
    }
  }

  // change #2: fixed-Δt gate replaces delay(10).
  float dt;
  if (!clock_.due(&dt)) return;

  uint8_t data[14];
  if (!i2cGuardedRead(i2cHealth, 0x3B, 14, data, LYRA_PIN_SDA, LYRA_PIN_SCL)) {
    capturing = false;                // change #8: never stream a torn window
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
      analyzeAndStream();
      capturing = false;
    }
  }
}
