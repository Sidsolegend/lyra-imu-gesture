// =============================================================================
// lyra_features.h  -  Shared windowing, ZUPT gyro-bias, 9-feature extraction
// =============================================================================
// Extracted from the identical DSP blocks copy-pasted across all four sketches:
//   calibrateGyroBias        (newlyra_final 167-194, calib 295-322,
//                             ev3 118-145, collect 89-117)
//   ZUPT rest re-zero + DRIFT_CLAMP  (newlyra_final 470-491, calib 454-475,
//                             ev3 392-413, collect 268-298)
//   countAboveFloor / countAccelSpikes (newlyra_final 197-214, etc.)
//   the 9-feature extraction (newlyra_final analyzeAndAct 346-368, calib 372-401,
//                             ev3 271-293, collect 187-212)
//
// TWO optimizations live here (changes #2 and #3):
//  #2 FIXED-Δt: SampleClock replaces delay(10). The originals sampled inside a
//     delay(10) hot loop, so the real rate drifted with BLE/serial load and the
//     count features (durationCount, spikeCount) drifted with it. SampleClock
//     paces on micros() for a true fixed interval, and extractFeatures() rescales
//     the count features from the achieved interval to the training interval
//     (DT_TRAIN_S = delay(10) => 10 ms) so the learned thresholds stay valid AND
//     the counts are portable ESP32 <-> ESP32-C3.
//  #3 NO String: nothing here allocates. Everything is POD structs + plain math.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "lyra_imu.h"

namespace lyra {

// ---- window + trigger (identical constants across all four originals) --------
static constexpr int   WINDOW            = 60;
static constexpr float TRIGGER_THRESHOLD = 1.35f;

// ---- ZUPT rest re-zero tuning (identical across all four originals) ----------
static constexpr float   REST_ACCEL_BAND = 0.05f;  // |accelMag-1g| < this == still
static constexpr float   REST_GYRO_BAND  = 8.0f;   // every axis < this dps == still
static constexpr int     REST_SAMPLES    = 40;     // ~0.4 s still -> consider re-zero
static constexpr int32_t DRIFT_CLAMP     = 120;    // re-zero may move <= ~7 dps from trusted

// ---- change #2: fixed-Δt scheduler ------------------------------------------
// The original training data was collected inside a delay(10) loop, i.e. a
// nominal 10 ms period. We keep that as the reference so learned thresholds and
// count features mean exactly what they did at capture time.
static constexpr uint32_t SAMPLE_INTERVAL_US = 10000;  // 10 ms == the delay(10) rate
static constexpr float    DT_TRAIN_S         = 0.010f; // reference period for count norm

// micros()-based fixed-interval gate. Replaces delay(10): the loop calls due()
// every pass and only samples when a full interval has elapsed, so BLE/serial
// jitter no longer stretches the effective Δt. Handles micros() wraparound via
// signed subtraction.
struct SampleClock {
  uint32_t nextUs = 0;
  bool     primed = false;
  // true exactly once per SAMPLE_INTERVAL_US; when it returns true, dtSecondsOut
  // is the interval actually served (== DT_TRAIN_S on the reference board).
  bool due(float* dtSecondsOut) {
    uint32_t now = micros();
    if (!primed) { primed = true; nextUs = now + SAMPLE_INTERVAL_US; *dtSecondsOut = DT_TRAIN_S; return true; }
    if ((int32_t)(now - nextUs) < 0) return false;   // not yet
    nextUs += SAMPLE_INTERVAL_US;                     // advance on a fixed grid (no drift)
    *dtSecondsOut = SAMPLE_INTERVAL_US / 1e6f;
    return true;
  }
};

// ---- pre-trigger ring buffer (ROADMAP #8 / F7) ------------------------------
// Capture ~PRETRIGGER samples BEFORE the 1.35 g trigger crossing, so the window
// includes the gesture's onset instead of starting mid-motion. This is CODE ONLY
// and OFF by default: it CHANGES the captured window (and therefore every count/
// stat feature), so it must ship only alongside a fresh data collection whose
// training corpus is captured the same way. A build opts in by #defining
// LYRA_OPT_PRETRIGGER before including this header, feeding every non-capturing
// sample to PreTrigRing::push(), and seeding the capture buffers from
// drainInto() at the trigger instant (captureIndex starts at the returned count).
static constexpr int PRETRIGGER = 10;   // samples retained ahead of the trigger

struct PreTrigRing {
  float accel[PRETRIGGER], gx[PRETRIGGER], gy[PRETRIGGER], gz[PRETRIGGER];
  int head = 0;     // next write slot
  int count = 0;    // valid samples held (saturates at PRETRIGGER)

  void push(float a, float x, float y, float z) {
    accel[head] = a; gx[head] = x; gy[head] = y; gz[head] = z;
    head = (head + 1) % PRETRIGGER;
    if (count < PRETRIGGER) count++;
  }

  void reset() { head = 0; count = 0; }

  // Copy the buffered samples OLDEST-first into the destination arrays (which
  // must hold >= PRETRIGGER). Returns how many were copied (<= PRETRIGGER).
  int drainInto(float* da, float* dx, float* dy, float* dz) const {
    for (int i = 0; i < count; i++) {
      int idx = (head - count + i + PRETRIGGER) % PRETRIGGER;   // oldest -> newest
      da[i] = accel[idx]; dx[i] = gx[idx]; dy[i] = gy[idx]; dz[i] = gz[idx];
    }
    return count;
  }
};

// ---- gyro zero-rate bias state (was ~8 duplicated globals per sketch) --------
struct GyroBias {
  int32_t GX0 = 34, GY0 = 657, GZ0 = 152;            // live zero (seeded, overwritten at boot)
  int32_t GX0ref = 34, GY0ref = 657, GZ0ref = 152;   // last trusted zero (boot cal / @zero)
  int64_t restSumX = 0, restSumY = 0, restSumZ = 0;
  int     restN = 0;
};

// Average raw gyro while still to (re)establish the trusted zero. Boot + "@zero".
// (Identical maths to all four originals' calibrateGyroBias.)
inline void calibrateGyroBias(GyroBias& b, int samples) {
  int64_t sx = 0, sy = 0, sz = 0;
  int16_t mnx = 32767, mny = 32767, mnz = 32767;
  int16_t mxx = -32768, mxy = -32768, mxz = -32768;
  int got = 0;
  for (int i = 0; i < samples; i++) {
    uint8_t d[14];
    if (readMPU(0x3B, 14, d)) {
      int16_t rx = (d[8] << 8) | d[9];
      int16_t ry = (d[10] << 8) | d[11];
      int16_t rz = (d[12] << 8) | d[13];
      sx += rx; sy += ry; sz += rz;
      mnx = min(mnx, rx); mxx = max(mxx, rx);
      mny = min(mny, ry); mxy = max(mxy, ry);
      mnz = min(mnz, rz); mxz = max(mxz, rz);
      got++;
    }
    delay(3);
  }
  if (got > 0) { b.GX0 = sx / got; b.GY0 = sy / got; b.GZ0 = sz / got; }
  b.GX0ref = b.GX0; b.GY0ref = b.GY0; b.GZ0ref = b.GZ0;
  b.restSumX = b.restSumY = b.restSumZ = 0; b.restN = 0;
  Serial.print("# gyro zero: GX0="); Serial.print(b.GX0);
  Serial.print(" GY0="); Serial.print(b.GY0);
  Serial.print(" GZ0="); Serial.println(b.GZ0);
  int spread = max((int)(mxx - mnx), max((int)(mxy - mny), (int)(mxz - mnz)));
  if (spread > 400) Serial.println("# WARNING: motion during cal -- hold still & redo");
}

// ---- boot gyro cal with motion-reject + retry (fixes C4 / T19) ---------------
// C4/T19: calibrateGyroBias() above only WARNS when the wrist moved during the
// sample (peak-to-peak spread > 400 raw counts) yet still ADOPTS that noisy zero,
// and because the corrupted value also becomes the trusted reference (GX0ref...),
// a bad boot zero can persist inside the +/-DRIFT_CLAMP band of itself - the ZUPT
// re-zero can't repair it. This boot variant instead RE-SAMPLES: it takes up to
// `maxAttempts` windows, keeps the calmest (lowest-spread) one, and accepts as
// soon as a window is still enough (spread <= motionSpread). If every attempt
// still shows motion it adopts the BEST (lowest-spread) attempt as a best-effort
// zero rather than hang - bounded work (maxAttempts windows), never blocks
// forever. Use this at BOOT in place of calibrateGyroBias(); the runtime "@zero"
// command keeps calling the plain calibrateGyroBias (there the wearer is actively
// asked to hold still, and it must stay quick + single-shot).
static constexpr int   GYRO_CAL_MAX_ATTEMPTS   = 5;
static constexpr int   GYRO_CAL_MOTION_SPREAD  = 400;  // peak-to-peak counts == "moved" (matches the warn above)
static constexpr int   GYRO_CAL_RETRY_DELAY_MS = 80;   // brief settle between windows
inline void calibrateGyroBiasStable(GyroBias& b, int samples,
                                    int maxAttempts  = GYRO_CAL_MAX_ATTEMPTS,
                                    int motionSpread = GYRO_CAL_MOTION_SPREAD) {
  int32_t bestGX0 = b.GX0, bestGY0 = b.GY0, bestGZ0 = b.GZ0;
  int     bestSpread = INT32_MAX;
  bool    settled = false;
  for (int attempt = 1; attempt <= maxAttempts && !settled; attempt++) {
    int64_t sx = 0, sy = 0, sz = 0;
    int16_t mnx = 32767, mny = 32767, mnz = 32767;
    int16_t mxx = -32768, mxy = -32768, mxz = -32768;
    int got = 0;
    for (int i = 0; i < samples; i++) {
      uint8_t d[14];
      if (readMPU(0x3B, 14, d)) {
        int16_t rx = (d[8] << 8) | d[9];
        int16_t ry = (d[10] << 8) | d[11];
        int16_t rz = (d[12] << 8) | d[13];
        sx += rx; sy += ry; sz += rz;
        mnx = min(mnx, rx); mxx = max(mxx, rx);
        mny = min(mny, ry); mxy = max(mxy, ry);
        mnz = min(mnz, rz); mxz = max(mxz, rz);
        got++;
      }
      delay(3);
    }
    if (got == 0) {                 // sensor silent this window: skip, don't div0
      Serial.print("# boot gyro cal attempt "); Serial.print(attempt);
      Serial.println(": no samples (I2C?) -- retrying");
      delay(GYRO_CAL_RETRY_DELAY_MS);
      continue;
    }
    int spread = max((int)(mxx - mnx), max((int)(mxy - mny), (int)(mxz - mnz)));
    if (spread < bestSpread) {      // remember the calmest window seen so far
      bestSpread = spread;
      bestGX0 = sx / got; bestGY0 = sy / got; bestGZ0 = sz / got;
    }
    if (spread <= motionSpread) {
      settled = true;               // still enough: accept immediately
    } else {
      Serial.print("# boot gyro cal attempt "); Serial.print(attempt);
      Serial.print(": motion (spread "); Serial.print(spread);
      Serial.print(" > "); Serial.print(motionSpread);
      Serial.println(") -- hold still, retrying");
      delay(GYRO_CAL_RETRY_DELAY_MS);
    }
  }
  b.GX0 = bestGX0; b.GY0 = bestGY0; b.GZ0 = bestGZ0;   // adopt best (or settled) zero
  b.GX0ref = b.GX0; b.GY0ref = b.GY0; b.GZ0ref = b.GZ0;
  b.restSumX = b.restSumY = b.restSumZ = 0; b.restN = 0;
  Serial.print("# gyro zero: GX0="); Serial.print(b.GX0);
  Serial.print(" GY0="); Serial.print(b.GY0);
  Serial.print(" GZ0="); Serial.print(b.GZ0);
  Serial.print("  (spread "); Serial.print(bestSpread);
  Serial.println(settled ? ", still)" : ", MOTION persisted -- best-effort zero)");
}

// ZUPT drift correction: when the wrist is held still, snap the zero to the mean
// of the still window (clamped to DRIFT_CLAMP from the trusted zero). Call every
// non-capturing sample with the corrected rates + the raw gyro counts.
// (Identical logic to all four originals' rest re-zero block.)
inline void zuptUpdate(GyroBias& b, float accelMag,
                       float gx_dps, float gy_dps, float gz_dps,
                       int16_t gxRaw, int16_t gyRaw, int16_t gzRaw) {
  bool stillAccel = fabs(accelMag - 1.0f) < REST_ACCEL_BAND;
  bool stillGyro  = fabs(gx_dps) < REST_GYRO_BAND &&
                    fabs(gy_dps) < REST_GYRO_BAND &&
                    fabs(gz_dps) < REST_GYRO_BAND;
  if (stillAccel && stillGyro) {
    b.restSumX += gxRaw; b.restSumY += gyRaw; b.restSumZ += gzRaw; b.restN++;
    if (b.restN >= REST_SAMPLES) {
      int32_t cx = (int32_t)lround((double)b.restSumX / b.restN);
      int32_t cy = (int32_t)lround((double)b.restSumY / b.restN);
      int32_t cz = (int32_t)lround((double)b.restSumZ / b.restN);
      if (abs(cx - b.GX0ref) <= DRIFT_CLAMP &&
          abs(cy - b.GY0ref) <= DRIFT_CLAMP &&
          abs(cz - b.GZ0ref) <= DRIFT_CLAMP) {
        b.GX0 = cx; b.GY0 = cy; b.GZ0 = cz;
      }
      b.restSumX = b.restSumY = b.restSumZ = 0; b.restN = 0;
    }
  } else {
    b.restSumX = b.restSumY = b.restSumZ = 0; b.restN = 0;
  }
}

// ---- count features (identical across all four originals) --------------------
inline int countAboveFloor(const float *gx, const float *gy, const float *gz,
                           int len, float floor) {
  int count = 0;
  for (int i = 0; i < len; i++) {
    float g = max(fabs(gx[i]), max(fabs(gy[i]), fabs(gz[i])));
    if (g > floor) count++;
  }
  return count;
}

inline int countAccelSpikes(const float *buf, int len, float threshold) {
  int bursts = 0; bool above = false;
  for (int i = 0; i < len; i++) {
    bool nowAbove = buf[i] > threshold;
    if (nowAbove && !above) bursts++;
    above = nowAbove;
  }
  return bursts;
}

// ---- the 9 features fed to the classifier -----------------------------------
// Same order + meaning as the app's featuresOf() and the on-device tree's inputs.
struct Features {
  float peakGyro;       // [0] max of the three axis peaks
  float ratio;          // [1] dominant-axis concentration
  float durationCount;  // [2] samples above gyro floor  (Δt-normalized, change #2)
  float absGy;          // [3] peak |gy|
  float absGz;          // [4] peak |gz|
  float peakAccel;      // [5]
  float accelMean;      // [6]
  float accelStd;       // [7]
  float spikeCount;     // [8] accel bursts             (Δt-normalized, change #2)
  // extra signed peaks for direction (not part of the 9-vector)
  float peakGX, peakGYVal, peakGZVal;
  float peakGYSigned, peakGZSigned;
};

// Flatten the 9-vector in the canonical order (for the custom-gesture matcher).
inline void featuresToVec(const Features& f, float out[9]) {
  out[0]=f.peakGyro; out[1]=f.ratio; out[2]=f.durationCount;
  out[3]=f.absGy;    out[4]=f.absGz; out[5]=f.peakAccel;
  out[6]=f.accelMean;out[7]=f.accelStd; out[8]=f.spikeCount;
}

// Extract all 9 features from a captured window. `gyroFloor` is the same value
// the classifier uses (durationCount is counted above it, as in the originals).
// `dtSeconds` is the achieved sample interval; count features are rescaled from
// it to the training interval so a faster/slower board matches the trained tree.
inline Features extractFeatures(const float* accelBuf, const float* gxBuf,
                               const float* gyBuf, const float* gzBuf,
                               float gyroFloor, float dtSeconds) {
  Features f;
  float peakGX = 0, peakGY = 0, peakGZ = 0, peakGYSigned = 0, peakGZSigned = 0;
  for (int i = 0; i < WINDOW; i++) {
    if (fabs(gxBuf[i]) > peakGX) peakGX = fabs(gxBuf[i]);
    if (fabs(gyBuf[i]) > peakGY) { peakGY = fabs(gyBuf[i]); peakGYSigned = gyBuf[i]; }
    if (fabs(gzBuf[i]) > peakGZ) peakGZ = fabs(gzBuf[i]);
    if (fabs(gzBuf[i]) > fabs(peakGZSigned)) peakGZSigned = gzBuf[i];
  }
  float peakGyro = max(peakGX, max(peakGY, peakGZ));
  float sorted[3] = {peakGX, peakGY, peakGZ};
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++)
      if (sorted[j] > sorted[i]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
  float ratio = sorted[0] / (sorted[1] + sorted[2] + 0.001f);

  float peakAccel = 0, accelSum = 0;
  for (int i = 0; i < WINDOW; i++) { if (accelBuf[i] > peakAccel) peakAccel = accelBuf[i]; accelSum += accelBuf[i]; }
  float accelMean = accelSum / WINDOW;
  float accelVar = 0;
  for (int i = 0; i < WINDOW; i++) { float d = accelBuf[i] - accelMean; accelVar += d * d; }
  float accelStd = sqrt(accelVar / WINDOW);

  int spikeCountRaw    = countAccelSpikes(accelBuf, WINDOW, TRIGGER_THRESHOLD);
  int durationCountRaw = countAboveFloor(gxBuf, gyBuf, gzBuf, WINDOW, gyroFloor);

  // change #2: rescale the rate-coupled counts from the achieved interval to the
  // training interval. countScale == 1.0 on the reference board (10 ms fixed Δt),
  // so learned thresholds are preserved exactly; it only corrects a board that
  // cannot hold - or deliberately runs - a different fixed rate.
  float countScale = dtSeconds / DT_TRAIN_S;

  f.peakGyro      = peakGyro;
  f.ratio         = ratio;
  f.durationCount = durationCountRaw * countScale;
  f.absGy         = peakGY;
  f.absGz         = peakGZ;
  f.peakAccel     = peakAccel;
  f.accelMean     = accelMean;
  f.accelStd      = accelStd;
  f.spikeCount    = spikeCountRaw * countScale;
  f.peakGX = peakGX; f.peakGYVal = peakGY; f.peakGZVal = peakGZ;
  f.peakGYSigned = peakGYSigned; f.peakGZSigned = peakGZSigned;
  return f;
}

}  // namespace lyra
