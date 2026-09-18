// =============================================================================
// LYRA - DAILY HID BUILD (optimized)   output stage: BLE-HID media keys
// =============================================================================
// Optimized duplicate of firmware/newlyra_final/newlyra_final.ino. All shared
// DSP (I2C bring-up, ZUPT gyro bias, feature extraction, hybrid classifier,
// custom-gesture matcher) now lives in ../lib/*.h, so this file is ONLY:
//   sample -> capture window -> classify -> ARM/double-flick logic -> BLE-HID.
//
// Behavior is preserved bit-for-bit vs the original EXCEPT the review fixes:
//   #2 fixed-Δt sampling (SampleClock) instead of delay(10) in the hot loop
//   #3 enum/const char* instead of Arduino String in the classify+act path
//   #4 shared mpuBringUp() cold-boot recovery+retry (unchanged here; already had it)
//   #5 stage-2 tree splits loaded from NVS (TreeParams) so app calibration applies
//   #8 runtime hardening (lyra_system.h): task watchdog, reset-reason forensics
//      (brownout visibility), mid-session I2C fault recovery, BLE connection
//      supervision, power-option flags. See OPTIMIZATIONS.md #8.
//
// DEPENDENCY: a NimBLE-based BLE-keyboard library exposing media keys, included
//   as <NimBleKeyboard.h> (same as the original V1/daily build).
// =============================================================================
#include <Wire.h>
#include <NimBleKeyboard.h>
#include <Preferences.h>
#include "../lib/lyra_imu.h"
#include "../lib/lyra_features.h"
#include "../lib/lyra_classifier.h"
#include "../lib/lyra_system.h"

using namespace lyra;

BleKeyboard bleKeyboard("LYRA", "LYRA", 100);
Preferences prefs;

// ---- runtime state -----------------------------------------------------------
GyroBias    bias;
SampleClock clock_;                     // change #2: fixed-Δt scheduler
Stage1Params s1;                        // gf/rc from NVS
TreeParams   tree;                      // change #5: stage-2 splits from NVS
GTemplate    customGestures[MAX_CUSTOM];
int          customCount = 0;

I2cHealth   i2cHealth;                  // change #8: mid-session bus recovery

float accelBuf[WINDOW], gxBuf[WINDOW], gyBuf[WINDOW], gzBuf[WINDOW];
bool  capturing = false;
int   captureIndex = 0;
float lastDt = DT_TRAIN_S;              // Δt served for the current window

// change #8: BLE connection supervision. NimBleKeyboard re-advertises on its
// own after a disconnect; this block makes link drops VISIBLE (and abandons a
// pending capture so a half-window from mid-drop jostling can't misfire).
bool     wasConnected = false;
uint32_t lastLinkLog = 0;
void superviseLink() {
  bool now = bleKeyboard.isConnected();
  if (now != wasConnected) {
    Serial.println(now ? "# BLE host connected" : "# BLE host lost -- re-advertising");
    wasConnected = now;
  } else if (!now && millis() - lastLinkLog > 30000) {
    Serial.println("# (still advertising as 'LYRA' -- no host)");
    lastLinkLog = millis();
  }
}

// ---- arm / disarm + single-vs-double-flick disambiguation --------------------
bool     armed = false;
bool     flickPending = false;
bool     pendingFlickRight = false;
uint32_t flickPendingMs = 0;
const uint32_t DOUBLE_FLICK_MS = 1400;  // two flicks within this = double-flick

// ROADMAP #10: twist<->flick remap. The paper's accessibility fix for
// anatomy-limited wearers who can't reliably twist: swap which motion carries
// VOLUME vs TRACK-SKIP so volume rides on the flick they CAN make.
//   normal (false): twist = volume,  single-flick = track skip
//   remap  (true) : twist = track skip, single-flick = volume
// Double-flick still arms/disarms in BOTH modes. Toggled by the serial "@remap"
// command (persisted to NVS "remap"); the calibration build can also push it.
bool     twistFlickRemap = false;

// =============================================================================
void sendMedia(const MediaKeyReport key, const char *label) {
  if (bleKeyboard.isConnected()) {
    bleKeyboard.write(key);
    Serial.print("  -> "); Serial.println(label);
  } else {
    Serial.print("  (not connected, would send "); Serial.print(label); Serial.println(")");
  }
}

// change #3: char action code + const char* label; no String.
void fireAction(char a, const char* label) {
  switch (a) {
    case 'N': sendMedia(KEY_MEDIA_NEXT_TRACK, label); break;
    case 'B': sendMedia(KEY_MEDIA_PREVIOUS_TRACK, label); break;
    case 'U': sendMedia(KEY_MEDIA_VOLUME_UP, label); break;
    case 'D': sendMedia(KEY_MEDIA_VOLUME_DOWN, label); break;
    case 'M': sendMedia(KEY_MEDIA_MUTE, label); break;
    case 'P':
    default:  sendMedia(KEY_MEDIA_PLAY_PAUSE, label); break;
  }
}

// Act on a classified gesture. Flicks are deferred so a double-flick can be
// caught (it toggles arm/disarm instead of skipping a track).
void act(Gesture g, bool flickRight, bool twistCW) {
  if (g == G_FLICK) {
    uint32_t now = millis();
    if (flickPending && (now - flickPendingMs) <= DOUBLE_FLICK_MS) {
      flickPending = false;
      armed = !armed;
      Serial.println(armed ? "*** ARMED ***" : "*** DISARMED ***");
    } else {
      flickPending = true;
      flickPendingMs = now;
      pendingFlickRight = flickRight;
    }
    return;
  }
  if (!armed) return;                                   // non-flick fires only when armed
  if (g == G_PINCH) {
    sendMedia(KEY_MEDIA_PLAY_PAUSE, "play/pause");      // pinch unaffected by remap
  } else if (g == G_TWIST) {
    if (twistFlickRemap) {                              // remap: twist -> track skip
      if (twistCW) sendMedia(KEY_MEDIA_NEXT_TRACK, "next track");
      else         sendMedia(KEY_MEDIA_PREVIOUS_TRACK, "previous track");
    } else {                                            // normal: twist -> volume
      if (twistCW) sendMedia(KEY_MEDIA_VOLUME_UP, "volume up");
      else         sendMedia(KEY_MEDIA_VOLUME_DOWN, "volume down");
    }
  }
}

void servicePendingFlick() {
  if (flickPending && (millis() - flickPendingMs) > DOUBLE_FLICK_MS) {
    flickPending = false;
    if (armed) {
      if (twistFlickRemap) {                            // remap: single-flick -> volume
        if (pendingFlickRight) sendMedia(KEY_MEDIA_VOLUME_UP, "volume up");
        else                   sendMedia(KEY_MEDIA_VOLUME_DOWN, "volume down");
      } else {                                          // normal: single-flick -> track skip
        if (pendingFlickRight) sendMedia(KEY_MEDIA_NEXT_TRACK, "next track");
        else                   sendMedia(KEY_MEDIA_PREVIOUS_TRACK, "previous track");
      }
    }
  }
}

// ---- NVS: restore the app-pushed profile + tree + custom gestures ------------
void loadConfigNVS() {
  prefs.begin(LYRA_NVS_NS, true);   // read-only
  loadStage1NVS(prefs, s1);         // gf/rc (pinch gate "ps" is subsumed by the tree)
  loadTreeNVS(prefs, tree);         // change #5: stage-2 splits (default to trained consts)
  customCount = loadCustomNVS(prefs, customGestures);
  twistFlickRemap = prefs.getBool("remap", false);   // ROADMAP #10
  prefs.end();
}

// =============================================================================
void analyzeAndAct() {
  Features f = extractFeatures(accelBuf, gxBuf, gyBuf, gzBuf, s1.minGyroFloor, lastDt);
  Gesture g = classifyHybrid(f, s1, tree);

  bool flickRight = (f.peakGYSigned > 0);
  bool twistCW    = (f.peakGZSigned > 0);

  Serial.print("gesture: "); Serial.print(gestureName(g));
  Serial.print("  (peakGyro="); Serial.print(f.peakGyro, 0);
  Serial.print(" ratio="); Serial.print(f.ratio, 2);
  Serial.print(" dur="); Serial.print(f.durationCount, 1);
  Serial.print(" aStd="); Serial.print(f.accelStd, 2); Serial.println(")");

  // custom gestures occupy the "not a built-in" space: only when armed + NONE.
  if (armed && customCount > 0 && g == G_NONE) {
    float feat[NUM_FEATURES]; featuresToVec(f, feat);
    int m = matchCustom(customGestures, customCount, feat);
    if (m >= 0) { fireAction(customGestures[m].action, customGestures[m].name); return; }
  }
  act(g, flickRight, twistCW);
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  systemBoot();                       // change #8: reset forensics + power opts

  // change #4: shared cold-boot bus recovery + init retry (C3 pins 6/7).
  mpuBringUp(LYRA_PIN_SDA, LYRA_PIN_SCL);

  Serial.println("# Calibrating gyro zero -- hold the device STILL...");
  delay(300);
  calibrateGyroBiasStable(bias, 200);   // T19/C4: reject-and-retry if the wrist moved

  loadConfigNVS();
  Serial.print("# active profile: MIN_GYRO_FLOOR="); Serial.print(s1.minGyroFloor);
  Serial.print(" RATIO_CUT="); Serial.print(s1.ratioCut);
  Serial.print("  tree.durSplit="); Serial.print(tree.durSplit);
  Serial.print(" customGestures="); Serial.println(customCount);

  bleKeyboard.begin();
  wdtInit();                          // change #8: armed AFTER blocking bring-up
  Serial.println("# LYRA ready. Pair 'LYRA' over Bluetooth. Double-flick to ARM.");
  Serial.println("# Send \"@zero\" (hold still) any time to re-zero the gyro.");
}

// =============================================================================
void loop() {
  wdtFeed();                          // change #8: loop alive -> no reboot

  // serial command: @zero
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();   // (cold-path config only)
    if (s.equalsIgnoreCase("@zero")) {
      Serial.println("# manual re-zero -- hold still...");
      delay(200); calibrateGyroBias(bias, 200);
    } else if (s.equalsIgnoreCase("@remap")) {           // ROADMAP #10: toggle + persist
      twistFlickRemap = !twistFlickRemap;
      prefs.begin(LYRA_NVS_NS, false);
      prefs.putBool("remap", twistFlickRemap);
      prefs.end();
      Serial.print("# twist<->flick remap ");
      Serial.println(twistFlickRemap ? "ON  (flick=volume, twist=track)"
                                     : "OFF (twist=volume, flick=track)");
    }
  }

  servicePendingFlick();
  superviseLink();                    // change #8: visible connect/disconnect

  // change #2: fixed-Δt gate replaces delay(10). Only proceed on a real tick.
  float dt;
  if (!clock_.due(&dt)) return;

  uint8_t data[14];
  if (!i2cGuardedRead(i2cHealth, 0x3B, 14, data, LYRA_PIN_SDA, LYRA_PIN_SCL)) {
    capturing = false;                // change #8: never classify a torn window
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
