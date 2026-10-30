// =============================================================================
// LYRA - GENERAL KEYBOARD-HID BUILD (optimized)   output stage: full BLE keyboard
// =============================================================================
// ROADMAP #11 (app notes §8): the daily build is a MEDIA-only HID, so the
// app's Presentation / Reading modes could never reach the device. This sketch
// is the firmware half of that fix: it emits NON-media keycodes (arrows, Page
// Up/Down, Escape, 'b') as well as media keys, and carries a runtime MODE plus
// the twist<->flick remap (ROADMAP #10). It reuses the SAME shared DSP libs and
// the SAME arm/double-flick logic as lyra_daily_hid; only the act() table and the
// output device differ.
//
// It is a SEPARATE sketch (not a patch to the daily build) so the validated
// media-only daily firmware stays byte-stable; flash this one only when the
// non-media modes are wanted. NOT yet run on hardware -- keep edits conservative.
//
//   MODE_MEDIA  (0): twist=volume,      single-flick=track skip,  pinch=play/pause
//   MODE_SLIDES (1): twist=Page Up/Down, single-flick=Left/Right arrow, pinch='b'
//   MODE_READ   (2): twist=Up/Down arrow, single-flick=Page Up/Down, pinch=Space
//   (twistFlickRemap swaps the twist vs single-flick ROLES within a mode, exactly
//    as in the daily build: volume/track, arrows/pages, etc.)
//   Double-flick arms/disarms in every mode.
//
// PROTOCOL (serial cold-path, mirrors the daily build's @zero):
//   @zero            re-zero the gyro (hold still)
//   @mode <0|1|2>    set mode (persisted NVS "mode"); no arg = cycle
//   @remap           toggle twist<->flick remap (persisted NVS "remap")
//
// DEPENDENCY: a NimBLE BLE-keyboard library exposing BOTH normal keys and media
//   keys as <NimBleKeyboard.h> (BleKeyboard::write(uint8_t) / write(MediaKeyReport)
//   + the KEY_* constants). Same family as the daily build's dependency.
// =============================================================================
#include <Wire.h>
#include <NimBleKeyboard.h>
#include <Preferences.h>
#include "../lib/lyra_imu.h"
#include "../lib/lyra_features.h"
#include "../lib/lyra_classifier.h"
#include "../lib/lyra_system.h"
#include "../lib/lyra_led.h"        // glow on every recognised gesture

using namespace lyra;

BleKeyboard bleKeyboard("LYRA-KB", "LYRA", 100);
Preferences prefs;

// ---- runtime state -----------------------------------------------------------
GyroBias    bias;
SampleClock clock_;
Stage1Params s1;
TreeParams   tree;
GTemplate    customGestures[MAX_CUSTOM];
int          customCount = 0;
I2cHealth    i2cHealth;

float accelBuf[WINDOW], gxBuf[WINDOW], gyBuf[WINDOW], gzBuf[WINDOW];
bool  capturing = false;
int   captureIndex = 0;
float lastDt = DT_TRAIN_S;

bool     armed = false;
bool     flickPending = false;
bool     pendingFlickRight = false;
uint32_t flickPendingMs = 0;
const uint32_t DOUBLE_FLICK_MS = 1400;

// ---- output modes (ROADMAP #11) + twist<->flick remap (ROADMAP #10) ----------
enum OutputMode { MODE_MEDIA = 0, MODE_SLIDES = 1, MODE_READ = 2, MODE_COUNT = 3 };
int  outputMode      = MODE_MEDIA;
bool twistFlickRemap = false;

const char* modeName(int m) {
  switch (m) {
    case MODE_SLIDES: return "SLIDES";
    case MODE_READ:   return "READING";
    default:          return "MEDIA";
  }
}

// ---- link supervision (identical intent to the daily build) ------------------
bool     wasConnected = false;
uint32_t lastLinkLog = 0;
void superviseLink() {
  bool now = bleKeyboard.isConnected();
  if (now != wasConnected) {
    Serial.println(now ? "# BLE host connected" : "# BLE host lost -- re-advertising");
    wasConnected = now;
  } else if (!now && millis() - lastLinkLog > 30000) {
    Serial.println("# (still advertising as 'LYRA-KB' -- no host)");
    lastLinkLog = millis();
  }
}

// ---- output helpers ----------------------------------------------------------
void sendKey(uint8_t key, const char* label) {
  if (bleKeyboard.isConnected()) { bleKeyboard.write(key); Serial.print("  -> "); Serial.println(label); }
  else { Serial.print("  (not connected, would send "); Serial.print(label); Serial.println(")"); }
}
void sendMedia(const MediaKeyReport key, const char* label) {
  if (bleKeyboard.isConnected()) { bleKeyboard.write(key); Serial.print("  -> "); Serial.println(label); }
  else { Serial.print("  (not connected, would send "); Serial.print(label); Serial.println(")"); }
}

// The two "axes" every mode maps onto: a PRIMARY pair (positive/negative) and a
// SECONDARY pair. In each mode, twist drives one axis and single-flick the other;
// twistFlickRemap swaps which motion drives which axis. `pos` is CW / flick-right.
void fireAxisTwist(bool pos) {
  // twist axis (unless remapped to the flick axis)
  bool flickAxis = twistFlickRemap;
  if (!flickAxis) {   // twist on its native (secondary) axis
    switch (outputMode) {
      case MODE_SLIDES: pos ? sendKey(KEY_PAGE_DOWN, "page down") : sendKey(KEY_PAGE_UP, "page up"); break;
      case MODE_READ:   pos ? sendKey(KEY_DOWN_ARROW, "scroll down") : sendKey(KEY_UP_ARROW, "scroll up"); break;
      default:          pos ? sendMedia(KEY_MEDIA_VOLUME_UP, "volume up") : sendMedia(KEY_MEDIA_VOLUME_DOWN, "volume down"); break;
    }
  } else {            // remapped: twist drives the primary (flick) axis instead
    switch (outputMode) {
      case MODE_SLIDES: pos ? sendKey(KEY_RIGHT_ARROW, "next slide") : sendKey(KEY_LEFT_ARROW, "prev slide"); break;
      case MODE_READ:   pos ? sendKey(KEY_PAGE_DOWN, "page down") : sendKey(KEY_PAGE_UP, "page up"); break;
      default:          pos ? sendMedia(KEY_MEDIA_NEXT_TRACK, "next track") : sendMedia(KEY_MEDIA_PREVIOUS_TRACK, "previous track"); break;
    }
  }
}
void fireAxisFlick(bool pos) {
  // single-flick axis (unless remapped to the twist axis)
  bool twistAxis = twistFlickRemap;
  if (!twistAxis) {   // flick on its native (primary) axis
    switch (outputMode) {
      case MODE_SLIDES: pos ? sendKey(KEY_RIGHT_ARROW, "next slide") : sendKey(KEY_LEFT_ARROW, "prev slide"); break;
      case MODE_READ:   pos ? sendKey(KEY_PAGE_DOWN, "page down") : sendKey(KEY_PAGE_UP, "page up"); break;
      default:          pos ? sendMedia(KEY_MEDIA_NEXT_TRACK, "next track") : sendMedia(KEY_MEDIA_PREVIOUS_TRACK, "previous track"); break;
    }
  } else {            // remapped: flick drives the secondary (twist) axis instead
    switch (outputMode) {
      case MODE_SLIDES: pos ? sendKey(KEY_PAGE_DOWN, "page down") : sendKey(KEY_PAGE_UP, "page up"); break;
      case MODE_READ:   pos ? sendKey(KEY_DOWN_ARROW, "scroll down") : sendKey(KEY_UP_ARROW, "scroll up"); break;
      default:          pos ? sendMedia(KEY_MEDIA_VOLUME_UP, "volume up") : sendMedia(KEY_MEDIA_VOLUME_DOWN, "volume down"); break;
    }
  }
}
void firePinch() {
  switch (outputMode) {
    case MODE_SLIDES: sendKey('b', "blank slide"); break;      // PowerPoint/Keynote blank toggle
    case MODE_READ:   sendKey(' ', "page/space"); break;
    default:          sendMedia(KEY_MEDIA_PLAY_PAUSE, "play/pause"); break;
  }
}

// ---- arm / double-flick logic (identical to the daily build) -----------------
void act(Gesture g, bool flickRight, bool twistCW) {
  if (g == G_FLICK) {
    uint32_t now = millis();
    if (flickPending && (now - flickPendingMs) <= DOUBLE_FLICK_MS) {
      flickPending = false;
      armed = !armed;
      ledArmed(armed);
      Serial.println(armed ? "*** ARMED ***" : "*** DISARMED ***");
    } else {
      flickPending = true; flickPendingMs = now; pendingFlickRight = flickRight;
    }
    return;
  }
  if (!armed) return;
  if (g == G_PINCH)      firePinch();
  else if (g == G_TWIST) fireAxisTwist(twistCW);
}

void servicePendingFlick() {
  if (flickPending && (millis() - flickPendingMs) > DOUBLE_FLICK_MS) {
    flickPending = false;
    if (armed) fireAxisFlick(pendingFlickRight);
  }
}

// ---- NVS ---------------------------------------------------------------------
void loadConfigNVS() {
  prefs.begin(LYRA_NVS_NS, true);
  loadStage1NVS(prefs, s1);
  loadTreeNVS(prefs, tree);
  customCount = loadCustomNVS(prefs, customGestures);
  twistFlickRemap = prefs.getBool("remap", false);
  outputMode = prefs.getInt("mode", MODE_MEDIA);
  if (outputMode < 0 || outputMode >= MODE_COUNT) outputMode = MODE_MEDIA;
  prefs.end();
}
void persistMode() { prefs.begin(LYRA_NVS_NS, false); prefs.putInt("mode", outputMode); prefs.end(); }
void persistRemap() { prefs.begin(LYRA_NVS_NS, false); prefs.putBool("remap", twistFlickRemap); prefs.end(); }

// =============================================================================
void analyzeAndAct() {
  Features f = extractFeatures(accelBuf, gxBuf, gyBuf, gzBuf, s1.minGyroFloor, lastDt);
  Gesture g = classifyHybrid(f, s1, tree);
  if (g != G_NONE) ledGesture();
  bool flickRight = (f.peakGYSigned > 0);
  bool twistCW    = (f.peakGZSigned > 0);

  Serial.print("gesture: "); Serial.print(gestureName(g));
  Serial.print("  mode="); Serial.print(modeName(outputMode));
  Serial.print(" remap="); Serial.println(twistFlickRemap ? "on" : "off");

  if (armed && customCount > 0 && g == G_NONE) {
    float feat[NUM_FEATURES]; featuresToVec(f, feat);
    int m = matchCustom(customGestures, customCount, feat);
    if (m >= 0) {
      ledGesture();
      // custom-gesture actions stay media (same 1-char action codes as the app)
      switch (customGestures[m].action) {
        case 'N': sendMedia(KEY_MEDIA_NEXT_TRACK, customGestures[m].name); break;
        case 'B': sendMedia(KEY_MEDIA_PREVIOUS_TRACK, customGestures[m].name); break;
        case 'U': sendMedia(KEY_MEDIA_VOLUME_UP, customGestures[m].name); break;
        case 'D': sendMedia(KEY_MEDIA_VOLUME_DOWN, customGestures[m].name); break;
        case 'M': sendMedia(KEY_MEDIA_MUTE, customGestures[m].name); break;
        default:  sendMedia(KEY_MEDIA_PLAY_PAUSE, customGestures[m].name); break;
      }
      return;
    }
  }
  act(g, flickRight, twistCW);
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  systemBoot();
  ledInit();                          // gesture-feedback LED
  mpuBringUp(LYRA_PIN_SDA, LYRA_PIN_SCL);

  Serial.println("# Calibrating gyro zero -- hold the device STILL...");
  delay(300);
  calibrateGyroBiasStable(bias, 200);   // T19/C4

  loadConfigNVS();
  Serial.print("# active mode: "); Serial.print(modeName(outputMode));
  Serial.print("  remap="); Serial.print(twistFlickRemap ? "on" : "off");
  Serial.print("  customGestures="); Serial.println(customCount);

  bleKeyboard.begin();
  wdtInit();
  Serial.println("# LYRA-KB ready. Pair 'LYRA-KB'. Double-flick to ARM.");
  Serial.println("# @mode <0|1|2> = media/slides/reading   @remap = swap twist<->flick   @zero = re-zero");
}

// =============================================================================
void loop() {
  wdtFeed();
  ledService();                       // advance the gesture glow (non-blocking)

  while (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s.equalsIgnoreCase("@zero")) {
      Serial.println("# manual re-zero -- hold still...");
      delay(200); calibrateGyroBias(bias, 200);
    } else if (s.startsWith("@mode")) {
      String arg = s.substring(5); arg.trim();
      if (arg.length()) outputMode = constrain(arg.toInt(), 0, MODE_COUNT - 1);
      else              outputMode = (outputMode + 1) % MODE_COUNT;   // no arg = cycle
      persistMode();
      Serial.print("# mode -> "); Serial.println(modeName(outputMode));
    } else if (s.equalsIgnoreCase("@remap")) {
      twistFlickRemap = !twistFlickRemap; persistRemap();
      Serial.print("# twist<->flick remap "); Serial.println(twistFlickRemap ? "ON" : "OFF");
    }
  }

  servicePendingFlick();
  superviseLink();

  float dt;
  if (!clock_.due(&dt)) return;

  uint8_t data[14];
  if (!i2cGuardedRead(i2cHealth, 0x3B, 14, data, LYRA_PIN_SDA, LYRA_PIN_SCL)) {
    capturing = false;
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
