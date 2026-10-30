// =============================================================================
// LYRA - SERIAL COLLECTION BUILD (optimized)   output stage: serial CSV log
// =============================================================================
// Optimized duplicate of firmware/lyra_collect_serialnew/...ino (no BLE). Shared
// DSP now lives in ../lib/*.h; this file is ONLY the datalog stage:
//   sample -> capture -> fast cascade -> print one 12-col CSV row to Serial.
//
// Review fixes vs the original:
//   #2 fixed-Δt sampling (SampleClock) instead of delay(10) - important here:
//      the collection rate now cannot drift, so the counts you TRAIN on are the
//      counts the daily build will REPRODUCE. Count features are Δt-normalized too.
//   #3 the CSV row is built with snprintf into a fixed char buffer (the original
//      emitted the row with ~12 sequential Serial.print() calls).
//   #4 shared mpuBringUp() cold-boot recovery+retry (original had a bare
//      Wire.begin(21,22) + single 0x6B wake, no recovery - lines 122-126).
//
// Serial controls (unchanged):  "@name" set subject, "@zero" re-zero gyro,
//   FLICK/TWIST/PINCH/GARBAGE  set the ground-truth label for the next reps.
// =============================================================================
#include <Wire.h>
#include "../lib/lyra_imu.h"
#include "../lib/lyra_features.h"
#include "../lib/lyra_classifier.h"
#include "../lib/lyra_system.h"     // change #8: wdt + reset forensics + I2C recovery

using namespace lyra;

// ---- runtime state -----------------------------------------------------------
GyroBias    bias;
SampleClock clock_;
I2cHealth   i2cHealth;              // change #8

float accelBuf[WINDOW], gxBuf[WINDOW], gyBuf[WINDOW], gzBuf[WINDOW];
bool  capturing = false;
int   captureIndex = 0;
float lastDt = DT_TRAIN_S;

// fast-cascade constants (verbatim from the original collect build).
const float MIN_GYRO_FLOOR      = 220.0f;
const float MIN_PINCH_ACCEL_STD = 0.40f;   // provisional; affects the classified col only
const float RATIO_CUT           = 1.2f;
const float PINCH_TWIST_BOUNDARY= 700.0f;

String currentLabel = "unlabeled";
String currentSubject = "me";

// =============================================================================
void analyzeAndPrint() {
  Features f = extractFeatures(accelBuf, gxBuf, gyBuf, gzBuf, MIN_GYRO_FLOOR, lastDt);
  Gesture g = classifyFast(f, MIN_GYRO_FLOOR, RATIO_CUT, MIN_PINCH_ACCEL_STD, PINCH_TWIST_BOUNDARY);

  // change #3: one snprintf, same 12 columns + precisions as the original.
  char row[200];
  snprintf(row, sizeof(row),
           "%s,%s,%.1f,%.3f,%ld,%.1f,%.1f,%.3f,%.3f,%.3f,%ld,%s",
           currentSubject.c_str(), currentLabel.c_str(),
           f.peakGyro, f.ratio, lround(f.durationCount),
           f.peakGYSigned, f.peakGZSigned,
           f.peakAccel, f.accelMean, f.accelStd,
           lround(f.spikeCount), gestureName(g));
  Serial.println(row);
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  systemBoot();                       // change #8: reset forensics + power opts

  // change #4: shared cold-boot bus recovery + retry (original had none).
  mpuBringUp(LYRA_PIN_SDA, LYRA_PIN_SCL);

  Serial.println("# Calibrating gyro zero -- hold the device STILL...");
  delay(300);
  calibrateGyroBias(bias, 200);

  wdtInit();                          // change #8: armed AFTER blocking bring-up
  Serial.print("# free heap at boot: "); Serial.println(ESP.getFreeHeap());
  Serial.println("subject,label,peakGyro,ratio,durationCount,gySigned,gzSigned,peakAccel,accelMean,accelStd,spikeCount,classified");
  Serial.println("# READY (serial-only, no BLE). Type \"@name\", then FLICK/TWIST/PINCH/GARBAGE + Enter before each rep.");
}

// =============================================================================
void loop() {
  wdtFeed();                          // change #8

  while (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();   // cold-path label/subject control
    if (s.length() > 0) {
      if (s.charAt(0) == '@') {
        if (s.equalsIgnoreCase("@zero")) {
          Serial.println("# manual re-zero -- hold the device STILL...");
          delay(200); calibrateGyroBias(bias, 200);
        } else {
          currentSubject = s.substring(1);
          Serial.print("# subject set: "); Serial.println(currentSubject);
        }
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
    capturing = false;                // change #8: never log a torn window
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
      analyzeAndPrint();
      capturing = false;
    }
  }
}
