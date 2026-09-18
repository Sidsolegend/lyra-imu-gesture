// =============================================================================
// lyra_imu.h  -  Shared MPU-6050 I2C bring-up + register access (LYRA optimized)
// =============================================================================
// Extracted from the ~70% copy-pasted DSP core of the four original sketches:
//   firmware/newlyra_final/newlyra_final.ino                (writeMPU/readMPU/
//     i2cBusRecover/initMPU/mpuWhoAmIOK, lines 103-163)
//   firmware/lyra_calibration_stream_new/...ino             (writeMPU/readMPU,
//     lines 225-242 + the fragile setup on 327-343 with NO bus recovery)
//   firmware/lyra_ev3_bridge/...ino                         (lines 100-116, 327-340)
//   firmware/lyra_collect_serialnew/...ino                  (lines 68-85, 119-146)
//
// Only newlyra_final.ino had the cold-boot bus recovery + init retry. This header
// gives EVERY optimized sketch that same hardening (see mpuBringUp()), which is
// the fix for change #4: the calibration build - the one the app onboards through -
// was the fragile one (bare Wire.begin + single 0x6B wake, no recovery, no retry).
//
// Header-only, all functions `inline` so a sketch can #include it in its single
// translation unit without multiple-definition errors.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace lyra {

// ---- fixed sensor configuration (identical across all four original sketches) --
static constexpr uint8_t MPU_ADDR    = 0x68;
static constexpr float   ACCEL_SCALE = 8192.0f;  // +/-4g  (register 0x1C = 0x08)
static constexpr float   GYRO_SCALE  = 16.4f;    // +/-2000 dps (register 0x1B = 0x18)

// Default I2C pins. NOTE (README change #7): the daily ESP32-C3 SuperMini build
// uses SDA=GPIO6 / SCL=GPIO7. The original big-ESP32 builds hardcoded 21/22
// (calibration/ev3/collect setups). Each sketch passes its own pins to mpuBringUp.
static constexpr int PIN_SDA_C3 = 6;
static constexpr int PIN_SCL_C3 = 7;
static constexpr int PIN_SDA_32 = 21;
static constexpr int PIN_SCL_32 = 22;

// [FIX #2 — single-board target] SINGLE SOURCE OF TRUTH for the board's I2C pins.
// ALL optimized sketches run on the SAME physical ESP32-C3 SuperMini and share one
// NVS blob, so they must all use the C3 pins. GPIO21/22 DO NOT EXIST on the C3 —
// the old ev3/collect/calibration builds passing PIN_SDA_32/PIN_SCL_32 (21/22)
// would hang the bus on the real hardware. Every optimized sketch now passes
// LYRA_PIN_SDA / LYRA_PIN_SCL to mpuBringUp(); change these two lines to retarget.
#ifndef LYRA_PIN_SDA
#define LYRA_PIN_SDA (lyra::PIN_SDA_C3)   // ESP32-C3 SuperMini SDA = GPIO6
#endif
#ifndef LYRA_PIN_SCL
#define LYRA_PIN_SCL (lyra::PIN_SCL_C3)   // ESP32-C3 SuperMini SCL = GPIO7
#endif

// ---- register access --------------------------------------------------------
inline void writeMPU(byte reg, byte data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(data);
  Wire.endTransmission(true);
}

inline bool readMPU(byte reg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, count);
  for (int i = 0; i < count; i++) {
    if (Wire.available()) buf[i] = Wire.read();
    else return false;
  }
  return true;
}

// WHO_AM_I: genuine 0x68; clones report 0x70/71/72/98 and work fine. Only
// 0x00 / 0xFF means "nothing answering on the bus".
inline bool whoAmIOK() {
  uint8_t w[1] = {0};
  if (!readMPU(0x75, 1, w)) return false;
  return !(w[0] == 0x00 || w[0] == 0xFF);
}

inline uint8_t whoAmIRaw() {
  uint8_t w[1] = {0};
  readMPU(0x75, 1, w);
  return w[0];
}

// Free a slave clamping SDA low ("dead bus" after a reset mid-transfer): manually
// clock SCL until SDA releases, then issue a STOP. Run BEFORE Wire.begin.
// (Verbatim from newlyra_final.ino i2cBusRecover, lines 137-148.)
inline void i2cBusRecover(int sda, int scl) {
  pinMode(scl, OUTPUT_OPEN_DRAIN); pinMode(sda, INPUT_PULLUP);
  digitalWrite(scl, HIGH);
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    digitalWrite(scl, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
  }
  pinMode(sda, OUTPUT_OPEN_DRAIN);            // STOP: SDA low->high while SCL high
  digitalWrite(sda, LOW);  delayMicroseconds(5);
  digitalWrite(scl, HIGH); delayMicroseconds(5);
  digitalWrite(sda, HIGH); delayMicroseconds(5);
}

// Full reset + reconfigure. Returns true once the sensor answers WHO_AM_I sane.
// (Verbatim from newlyra_final.ino initMPU, lines 151-163 - the "software EN press"
// that the clones need on a cold boot; the C3 SuperMini has no EN button.)
inline bool initMPU() {
  writeMPU(0x6B, 0x80); delay(100);          // DEVICE_RESET
  for (int i = 0; i < 20; i++) {             // wait for reset bit to self-clear
    uint8_t p[1];
    if (readMPU(0x6B, 1, p) && !(p[0] & 0x80)) break;
    delay(10);
  }
  writeMPU(0x68, 0x07); delay(100);          // SIGNAL_PATH_RESET (gyro+accel+temp)
  writeMPU(0x6B, 0x01); delay(50);           // wake, clock = PLL/X-gyro (stable on clones)
  writeMPU(0x1B, 0x18); delay(10);           // gyro +/-2000 dps
  writeMPU(0x1C, 0x08); delay(10);           // accel +/-4g
  return whoAmIOK();
}

// One-call cold-boot bring-up: settle -> recover a hung bus -> Wire.begin ->
// full reset + retry up to `attempts` times. This is change #4 - the hardening
// that only newlyra_final.ino had, now available to every optimized sketch.
// Returns true if the sensor came up sane.
inline bool mpuBringUp(int sda, int scl, int attempts = 10) {
  delay(400);                       // let the MPU rail + oscillator settle (cold-boot fix)
  i2cBusRecover(sda, scl);          // free a hung bus before Wire claims the pins
  Wire.begin(sda, scl);
  Wire.setClock(100000);            // 100 kHz: reliable on clones + dupont wiring

  bool ok = false;
  for (int attempt = 1; attempt <= attempts && !ok; attempt++) {
    ok = initMPU();                 // full reset + reconfigure = software "EN press"
    if (!ok) {
      Serial.print("# MPU init attempt "); Serial.print(attempt);
      Serial.println(" failed -- retrying (software EN)");
      i2cBusRecover(sda, scl);
      delay(150);
    }
  }
  Serial.print("# WHO_AM_I 0x"); Serial.println(whoAmIRaw(), HEX);
  if (!ok) {
    // Two distinct failure modes, one diagnostic each -- so a builder debugging
    // a fresh assembly knows whether to check wiring or check the sensor/bus.
    uint8_t w[1] = {0};
    bool acked = readMPU(0x75, 1, w);
    if (!acked) {
      Serial.print("# ERROR: no I2C ack -- MPU never responded. Check SDA=");
      Serial.print(sda); Serial.print(" SCL="); Serial.print(scl);
      Serial.println(", 3V3, GND, AD0->GND (addr 0x68)");
    } else {
      Serial.print("# ERROR: WHO_AM_I garbage 0x"); Serial.print(w[0], HEX);
      Serial.println(" -- chip answered but ID is 0x00/0xFF (bad clone or bus"
                      " noise); check solder joints + pull-ups");
    }
  } else {
    Serial.println("# MPU online.");
  }
  return ok;
}

// ---- motion-interrupt configuration (ROADMAP #9 / P section) -----------------
// Configure the MPU-6050's hardware motion detector so its INT pin can WAKE the
// C3 from light sleep, instead of the CPU busy-polling accelMag > 1.35 g every
// 10 ms. This is the enabler for the ~1-2 mA idle power mode. It is CODE ONLY and
// OFF unless a build opts in (see LYRA_OPT_MOTION_WAKE in lyra_system.h): enabling
// the interrupt-driven trigger path CHANGES how a gesture window starts (edge
// wake vs polled threshold) and MUST be re-validated against the dataset first.
//
// Register sequence is the classic MPU-6050 motion-detect recipe (datasheet +
// InvenSense app-note AN-MPU-6000A): high-pass the accel path, set a threshold
// (0x1F, 1 LSB ~= 32 mg) and duration (0x20, 1 LSB = 1 ms), then enable MOT_EN
// (INT_ENABLE bit6). INT_PIN_CFG (0x37) is left active-high, push-pull, latched
// until the INT_STATUS (0x3A) read that clears it.
inline void configureMotionInterrupt(uint8_t motThr = 20, uint8_t motDur = 1) {
  writeMPU(0x6B, 0x01);              // ensure awake, PLL clock
  writeMPU(0x1C, 0x01);             // accel +/-2g range w/ DHPF reset path
  writeMPU(0x1F, motThr);           // MOT_THR (motion threshold)
  writeMPU(0x20, motDur);           // MOT_DUR (ms the threshold must be exceeded)
  writeMPU(0x69, 0x15);             // MOT_DETECT_CTRL: accel power-on delay (settle)
  writeMPU(0x1C, 0x08 | 0x07);      // restore +/-4g (0x08) + DHPF = HOLD (0x07)
  writeMPU(0x37, 0x20);             // INT_PIN_CFG: latch INT until status read
  writeMPU(0x38, 0x40);             // INT_ENABLE: MOT_EN
}

// Clear a latched motion interrupt by reading INT_STATUS (0x3A). Call after a
// wake so the next motion re-triggers the pin.
inline uint8_t clearMotionInterrupt() {
  uint8_t st[1] = {0};
  readMPU(0x3A, 1, st);
  return st[0];
}

// Restore the normal continuous-sampling config used by the polled trigger path
// (undo configureMotionInterrupt): re-enable the full-rate gyro/accel ranges and
// disable the motion interrupt. Call when leaving the power-saving mode.
inline void restoreContinuousMode() {
  writeMPU(0x38, 0x00);             // INT_ENABLE: off
  writeMPU(0x1B, 0x18);             // gyro +/-2000 dps
  writeMPU(0x1C, 0x08);             // accel +/-4g (no DHPF hold)
}

}  // namespace lyra
