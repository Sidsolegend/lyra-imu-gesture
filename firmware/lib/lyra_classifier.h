// =============================================================================
// lyra_classifier.h  -  Shared hybrid classifier + custom-gesture matcher
// =============================================================================
// Extracted from the classifier blocks that were copy-pasted (with drift)
// across the sketches:
//   stage2 tree + classifyHybrid   (newlyra_final 216-243, ev3 166-191)
//   classifyFastGesture cascade    (calib 264-280, collect 172-185)
//   GTemplate + customDistance + matchCustom  (newlyra_final 285-304, ev3 219-236)
//
// THREE optimizations live here:
//  #3 NO String: every classifier returns a `Gesture` enum (+ gestureName() for
//     logging) instead of an Arduino String, so the hot path never touches the
//     heap. customDistance keeps the firmware's 1e-3f std floor (which the app
//     lacked - see the Flutter fix, change #6).
//  #5 UNIFIED CLASSIFIER: the stage-2 tree splits are no longer hardcoded. They
//     live in a TreeParams struct that the daily build loads from NVS (falling
//     back to the trained constants), so the app's per-user calibration finally
//     reaches the daily decision, not just the flick/floor gates.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "lyra_features.h"

namespace lyra {

// ---- gesture result (change #3: enum, not String) ---------------------------
enum Gesture { G_NONE = 0, G_FLICK, G_TWIST, G_PINCH };

inline const char* gestureName(Gesture g) {
  switch (g) {
    case G_FLICK: return "FLICK";
    case G_TWIST: return "TWIST";
    case G_PINCH: return "PINCH";
    default:      return "NONE";
  }
}

// ---- stage-1 gates (runtime; overridden by NVS in the builds that load it) ---
struct Stage1Params {
  float minGyroFloor = 220.0f;   // stage-1 floor      (app-tunable: NVS "gf")
  float flickGyro    = 700.0f;   // stage-1 flick gate (FIXED by design)
  float ratioCut     = 1.20f;    // stage-1 flick gate (app-tunable: NVS "rc")
};

// ---- stage-2 learned decision tree splits (change #5: was hardcoded) --------
// Defaults are the exact values the original stage2() baked in
// (newlyra_final.ino 222-232). The daily build now overrides these from NVS so
// per-user calibration actually shapes the twist/pinch/none decision.
struct TreeParams {
  float durSplit   = 18.5f;    // durationCount split          (NVS "tDur")
  float aMeanSplit = 1.159f;   // accelMean split              (NVS "tAmean")
  float pgLowSplit = 354.8f;   // peakGyro low-branch split    (NVS "tPgLo")
  float spikeSplit = 2.5f;     // spikeCount split             (NVS "tSpike")
  float aStdSplit  = 0.692f;   // accelStd (pinch) split       (NVS "tAstd")
  float pgHighSplit= 422.05f;  // peakGyro high-branch split   (NVS "tPgHi")
};

// stage 2: the decision tree learned on every non-flick rep across all three subjects.
// Call ONLY when stage 1 returns neither NONE nor FLICK.
// Same structure as newlyra_final.ino stage2(), splits now parameterized.
inline Gesture stage2(const Features& f, const TreeParams& tp) {
  if (f.durationCount <= tp.durSplit) {
    if (f.accelMean <= tp.aMeanSplit) {
      if (f.peakGyro <= tp.pgLowSplit) return G_NONE;
      else return (f.spikeCount <= tp.spikeSplit) ? G_TWIST : G_NONE;
    } else {
      if (f.accelStd <= tp.aStdSplit) return G_PINCH;   // pinch: high accel, low variance
      else return G_NONE;                                // high accel variance -> scratch/garbage
    }
  } else {
    if (f.peakGyro <= tp.pgHighSplit) return G_NONE;
    else return G_TWIST;                                 // sustained + real gyro -> twist
  }
}

// HYBRID classifier (daily / ev3 builds): stage-1 rules own flick + floor, the
// stage-2 tree owns twist/pinch/none. Identical decision to newlyra_final.ino.
inline Gesture classifyHybrid(const Features& f, const Stage1Params& s1,
                             const TreeParams& tp) {
  if (f.peakGyro < s1.minGyroFloor) return G_NONE;                       // stage-1 floor
  if (f.peakGyro >= s1.flickGyro && f.ratio > s1.ratioCut) return G_FLICK; // stage-1 flick
  return stage2(f, tp);                                                  // stage-2 tree
}

// FAST cascade (calibration-stream / collect builds): the simpler flat rule set
// that produces the CSV "classified" column. Identical to classifyFastGesture()
// in calib/collect (peakGyro floor -> pinch gate -> flick/twist by ratio).
inline Gesture classifyFast(const Features& f, float gyroFloor, float ratioCut,
                           float pinchGate, float pinchTwistBoundary) {
  if (f.peakGyro < gyroFloor) return G_NONE;
  if (f.peakGyro < pinchTwistBoundary) {
    if (f.accelStd < pinchGate) return G_NONE;
    return G_PINCH;
  }
  else if (f.ratio > ratioCut) return G_FLICK;
  else return G_TWIST;
}

// ---- custom-gesture templates (shared NVS blob, byte-identical layout) -------
static constexpr int NUM_FEATURES = 9;
static constexpr int MAX_CUSTOM   = 4;
// Layout MUST stay byte-identical across every build (shared NVS blob) - see the
// GTEMPLATE_SCHEMA_VERSION note below on how a future layout change is made safe.
struct GTemplate {
  char  name[16];
  char  action;            // 'P' play/pause 'N' next 'B' prev 'U' vol+ 'D' vol- 'M' mute
  float mean[NUM_FEATURES];
  float std[NUM_FEATURES];
  float thresh;            // RMS z-score match threshold
};

// NVS schema version for the persisted GTemplate blob (fixes N1 / T9).
// BUMP THIS by one whenever the GTemplate field layout changes in ANY way that
// alters its bytes - add / remove / reorder a field, resize name[], change a
// type, or add/remove struct padding (e.g. an __attribute__((packed)) on one
// build but not another). An old stored blob is then REJECTED at load instead of
// being silently reinterpreted with the new layout (which would corrupt
// calibration data with no error and no crash).
//
// The version is written as the FIRST BYTE of each persisted per-template NVS
// record: record = [uint8_t schemaVersion][GTemplate bytes]. GTemplate ITSELF is
// unchanged (still the byte-identical 96-byte layout the app and the parity tests
// pin) - the version framing lives AROUND the struct, not inside it, so:
//   * test_gtemplate_blob_layout_is_96_bytes (the 96-byte pin) still holds, and
//   * the BLE wire format is unaffected: the app<->device custom-gesture protocol
//     is the ASCII `G,<name>,<action>,<means..>,<stds..>,<thresh>` line (see
//     app/lib/custom_gesture.dart -> sendCustomGesture), which carries no binary
//     struct at all. It needs NO version byte; only the NVS blob does.
static constexpr uint8_t GTEMPLATE_SCHEMA_VERSION = 1;
// One persisted NVS record = the 1-byte version stamp + the raw GTemplate.
static constexpr size_t GTEMPLATE_RECORD_SIZE = 1 + sizeof(GTemplate);

// RMS z-score distance of a feature vector to a template (lower = closer).
// Keeps the 1e-3f std floor - the guard the app's distance() was missing (#6).
inline float customDistance(const GTemplate& t, const float* feat) {
  float s = 0;
  for (int j = 0; j < NUM_FEATURES; j++) {
    float sd = t.std[j] > 1e-3f ? t.std[j] : 1e-3f;
    float z = (feat[j] - t.mean[j]) / sd;
    s += z * z;
  }
  return sqrt(s / NUM_FEATURES);
}

// Index of the closest custom gesture within its own threshold, or -1.
inline int matchCustom(const GTemplate* customs, int count, const float* feat) {
  int best = -1; float bestDist = 1e9f;
  for (int i = 0; i < count; i++) {
    float d = customDistance(customs[i], feat);
    if (d < customs[i].thresh && d < bestDist) { bestDist = d; best = i; }
  }
  return best;
}

// ---- shared NVS helpers -----------------------------------------------------
// Namespace + keys shared by every build. Change #5 adds the six "t*" tree keys.
#define LYRA_NVS_NS "lyracfg"

// Load stage-1 gates (gf/rc) if a profile was pushed. pinchGate ("ps") is read
// separately by the builds that use the fast cascade.
inline void loadStage1NVS(Preferences& prefs, Stage1Params& s1) {
  if (prefs.getBool("hasP", false)) {
    s1.minGyroFloor = prefs.getFloat("gf", s1.minGyroFloor);
    s1.ratioCut     = prefs.getFloat("rc", s1.ratioCut);
  }
}

// change #5: load the stage-2 tree splits from NVS, defaulting to the trained
// constants when the app has not pushed them. Keys are documented in
// optimized/README.md and OPTIMIZATIONS.md.
inline void loadTreeNVS(Preferences& prefs, TreeParams& tp) {
  tp.durSplit    = prefs.getFloat("tDur",   tp.durSplit);
  tp.aMeanSplit  = prefs.getFloat("tAmean", tp.aMeanSplit);
  tp.pgLowSplit  = prefs.getFloat("tPgLo",  tp.pgLowSplit);
  tp.spikeSplit  = prefs.getFloat("tSpike", tp.spikeSplit);
  tp.aStdSplit   = prefs.getFloat("tAstd",  tp.aStdSplit);
  tp.pgHighSplit = prefs.getFloat("tPgHi",  tp.pgHighSplit);
}

// Persist custom-gesture templates. Writes each as a versioned record
// [GTEMPLATE_SCHEMA_VERSION][GTemplate] under key "g%d", then the count under
// "gn". Pair with loadCustomNVS below (which rejects any record not stamped with
// the current version). Builds that used to write the bare struct via
// putBytes(&t, sizeof(GTemplate)) should call THIS instead so the version byte
// is present - an unversioned legacy blob now loads as zero customs (defaults).
inline void saveCustomNVS(Preferences& prefs, const GTemplate* customs, int count) {
  if (count > MAX_CUSTOM) count = MAX_CUSTOM;
  for (int i = 0; i < count; i++) {
    uint8_t rec[GTEMPLATE_RECORD_SIZE];
    rec[0] = GTEMPLATE_SCHEMA_VERSION;
    memcpy(rec + 1, &customs[i], sizeof(GTemplate));
    char key[4]; snprintf(key, sizeof(key), "g%d", i);
    prefs.putBytes(key, rec, sizeof(rec));
  }
  prefs.putInt("gn", count);
}

// Load custom-gesture templates. Returns the count (clamped to MAX_CUSTOM).
// Every record must carry the current GTEMPLATE_SCHEMA_VERSION as its first byte;
// a record that is the wrong length (a legacy bare-struct blob) or stamped with a
// different version (an old firmware's layout) is REJECTED - we discard the WHOLE
// stored set and fall back to defaults (return 0) rather than reinterpret stale
// bytes as the current struct. Never crashes: a mismatch is logged and skipped.
inline int loadCustomNVS(Preferences& prefs, GTemplate* customs) {
  int n = prefs.getInt("gn", 0);
  if (n > MAX_CUSTOM) n = MAX_CUSTOM;
  for (int i = 0; i < n; i++) {
    char key[4]; snprintf(key, sizeof(key), "g%d", i);
    uint8_t rec[GTEMPLATE_RECORD_SIZE];
    size_t got = prefs.getBytes(key, rec, sizeof(rec));
    if (got != sizeof(rec) || rec[0] != GTEMPLATE_SCHEMA_VERSION) {
      Serial.print("# custom gesture NVS: g"); Serial.print(i);
      Serial.print(" schema v"); Serial.print(got ? rec[0] : 0);
      Serial.print(" != current v"); Serial.print(GTEMPLATE_SCHEMA_VERSION);
      Serial.println(" -- discarding stored custom gestures (using defaults)");
      return 0;                       // reject: do not reinterpret an old layout
    }
    memcpy(&customs[i], rec + 1, sizeof(GTemplate));
  }
  return n;
}

}  // namespace lyra
