// =============================================================================
// lyra_lsm6.h  -  LSM6DS3TR-C / LSM6DSV driver for the LYRA ring (replaces the MPU-6050)
// =============================================================================
// v1 ring PCB: ST LSM6DS3TR-C (WHO_AM_I 6Ah). v1.5 ring PCB: ST LSM6DSV (WHO_AM_I 70h, JLC
// Economic-PCBA friendly). Same LGA-14 pins, same I2C address 0x6A (SA0 = GND), same output
// registers (22h gyro, 28h accel); the chip is detected at start-up and configured to match. This driver keeps
// the classifier's world identical to the MPU-6050 builds:
//   - same full scales: +/-4 g and +/-2000 dps  (CTRL1_XL = 0x48, CTRL2_G = 0x4C)
//   - samples returned in g and dps, so extractFeatures()/classifyHybrid() are untouched
//   - raw gyro counts kept for the GyroBias / ZUPT logic (DRIFT_CLAMP of 120 counts is
//     ~8.4 dps here vs ~7.3 dps on the MPU - same intent)
// Plus the hardware double-tap detector ("knock") routed to INT1, which is also the
// deep-sleep wake pin (GPIO3 on the ring).
//
// Registers / bit fields from the ST datasheet (DocID030071 Rev 3), section 9:
//   CTRL1_XL 10h  ODR_XL[7:4] FS_XL[3:2]   (0100 = 104 Hz, 10 = +/-4 g)
//   CTRL2_G  11h  ODR_G[7:4]  FS_G[3:2]    (0100 = 104 Hz, 11 = 2000 dps)
//   CTRL3_C  12h  BDU bit6, IF_INC bit2
//   CTRL6_C  15h  XL_HM_MODE bit4 (1 = high-performance OFF -> low-power ODRs)
//   OUTX_L_G 22h..27h gyro, OUTX_L_XL 28h..2Dh accel (little endian)
//   TAP_CFG  58h  INTERRUPTS_ENABLE bit7, TAP_X/Y/Z_EN bits3..1, LIR bit0
//   TAP_THS_6D 59h TAP_THS[4:0] (1 LSb = FS_XL / 32)
//   INT_DUR2 5Ah  DUR[7:4] (x32/ODR) QUIET[3:2] (x4/ODR) SHOCK[1:0] (x8/ODR)
//   WAKE_UP_THS 5Bh SINGLE_DOUBLE_TAP bit7
//   MD1_CFG  5Eh  INT1_DOUBLE_TAP bit3
//   TAP_SRC  1Ch  DOUBLE_TAP bit4, SINGLE_TAP bit5, TAP_IA bit6 (read clears)
// LSM6DSV (DS13476; same layout as the LSM6DSV16X family) differs in the control / tap registers:
//   CTRL1 10h  OP_MODE_XL[6:4] ODR_XL[3:0]   (000 = high-perf, 0110 = 120 Hz; 100 = low-power 1)
//   CTRL2 11h  OP_MODE_G[6:4]  ODR_G[3:0]    (0000 = gyro off)
//   CTRL6 15h  FS_G[3:0]  (0100 = 2000 dps, 70 mdps/LSb)     CTRL8 17h  FS_XL[1:0] (01 = +/-4 g)
//   FUNCTIONS_ENABLE 50h INTERRUPTS_ENABLE bit7
//   TAP_CFG0 56h TAP_X/Y/Z_EN bits3..1, LIR bit0   TAP_CFG1/2 57h/58h, TAP_THS_6D 59h: TAP_THS_X/Y/Z[4:0]
//   TAP_DUR 5Ah DUR[7:4] QUIET[3:2] SHOCK[1:0]     WAKE_UP_THS 5Bh SINGLE_DOUBLE_TAP bit7
//   MD1_CFG 5Eh INT1_DOUBLE_TAP bit3               TAP_SRC 46h (DOUBLE_TAP bit4), WAKE_UP_SRC 45h
// Every address and bit field above checked against ST's own driver (STMicroelectronics/lsm6dsv-pid,
// lsm6dsv_reg.h). No I3C write on purpose: the LSM6DSV has no I3C-only disable bit, and IF_CFG (03h)
// bit0 I2C_I3C_DISABLE turns off I2C as well (it's for SPI-only use), which would make the IMU go silent.
// FIRST FLASH ON A v1.5 BOARD: check the "# LSM6DSV" line on serial and that a knock wakes the ring.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace lyra {

static constexpr uint8_t LSM_ADDR      = 0x6A;
static constexpr uint8_t LSM_WHO_AM_I  = 0x6A;      // value of reg 0Fh: LSM6DS3TR-C (v1 board)
static constexpr uint8_t LSM_WHO_AM_I_DSV = 0x70;   // LSM6DSV (v1.5 board)
static bool lsmIsDSV = false;                       // set by lsmBringUp()
static constexpr float   LSM_ACCEL_LSB_PER_G   = 1000.0f / 0.122f;  // +/-4 g: 0.122 mg/LSb
static constexpr float   LSM_GYRO_LSB_PER_DPS  = 1000.0f / 70.0f;   // 2000 dps: 70 mdps/LSb

// Axis remap: ring board axes -> the axes the classifier was trained on (wrist MPU).
// Each entry picks a source axis (0=x,1=y,2=z) and a sign. Identity until the first
// on-finger capture says otherwise (flick must land on GY, twist on GZ - see
// lyra_collect_serial and re-collect training data on the finger).
struct AxisMap { uint8_t src[3]; int8_t sign[3]; };
static constexpr AxisMap RING_AXES = {{0, 1, 2}, {1, 1, 1}};

inline void lsmWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(LSM_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}
inline bool lsmRead(uint8_t reg, uint8_t n, uint8_t* buf) {
  Wire.beginTransmission(LSM_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)LSM_ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
inline uint8_t lsmReg(uint8_t reg) { uint8_t v = 0; lsmRead(reg, 1, &v); return v; }

// Normal running mode: 104 Hz accel + gyro, same full scales as the MPU builds,
// double-tap ("knock") detection on INT1.
inline void lsmConfigureRunDSV() {
  lsmWrite(0x12, 0x44);   // CTRL3: BDU + IF_INC
  lsmWrite(0x15, 0x04);   // CTRL6: FS_G 2000 dps
  lsmWrite(0x17, 0x01);   // CTRL8: FS_XL +/-4 g
  lsmWrite(0x10, 0x06);   // CTRL1: accel high-performance, 120 Hz
  lsmWrite(0x11, 0x06);   // CTRL2: gyro high-performance, 120 Hz
  // knock = double tap, same thresholds / timing as the DS3 build (1.25 g; 120 Hz is close to 104 Hz)
  lsmWrite(0x56, 0x0F);   // TAP_CFG0: tap X/Y/Z, latched
  lsmWrite(0x57, 0x0A);   // TAP_CFG1: TAP_THS_X = 10 x 4g/32
  lsmWrite(0x58, 0x0A);   // TAP_CFG2: TAP_THS_Y
  lsmWrite(0x59, 0x0A);   // TAP_THS_6D: TAP_THS_Z
  lsmWrite(0x5A, 0x26);   // TAP_DUR: DUR=2 QUIET=1 SHOCK=2
  lsmWrite(0x5B, 0x80);   // WAKE_UP_THS: single + double tap
  lsmWrite(0x50, 0x80);   // FUNCTIONS_ENABLE: INTERRUPTS_ENABLE
  lsmWrite(0x5E, 0x08);   // MD1_CFG: double tap -> INT1
}

inline void lsmConfigureRun() {
  if (lsmIsDSV) { lsmConfigureRunDSV(); return; }
  lsmWrite(0x12, 0x44);   // CTRL3_C: BDU + IF_INC
  lsmWrite(0x15, 0x00);   // CTRL6_C: accel high-performance
  lsmWrite(0x10, 0x48);   // CTRL1_XL: 104 Hz, +/-4 g
  lsmWrite(0x11, 0x4C);   // CTRL2_G: 104 Hz, 2000 dps
  // knock = double tap. 104 Hz ODR: DUR 2 -> 0.62 s window, QUIET 1 -> 38 ms, SHOCK 2 -> 154 ms
  lsmWrite(0x58, 0x8F);   // TAP_CFG: INTERRUPTS_ENABLE, tap X/Y/Z, latched
  lsmWrite(0x59, 0x0A);   // TAP_THS_6D: 10 x 4g/32 = 1.25 g (tune on the real ring)
  lsmWrite(0x5A, 0x26);   // INT_DUR2: DUR=2 QUIET=1 SHOCK=2
  lsmWrite(0x5B, 0x80);   // WAKE_UP_THS: single + double tap
  lsmWrite(0x5E, 0x08);   // MD1_CFG: double tap -> INT1
}

// Deep-sleep mode: gyro off, accel low-power 104 Hz, knock detection still on INT1
// (that's what wakes the ESP32 from deep sleep / the battery reserve).
inline void lsmConfigureSleep() {
  if (lsmIsDSV) {
    lsmWrite(0x11, 0x00);   // CTRL2: gyro off
    lsmWrite(0x10, 0x46);   // CTRL1: accel low-power mode 1, 120 Hz, knock detection stays on
    lsmReg(0x46);           // clear any latched tap (TAP_SRC)
    lsmReg(0x45);           // + WAKE_UP_SRC
    return;
  }
  lsmWrite(0x11, 0x00);   // CTRL2_G: gyro power-down
  lsmWrite(0x15, 0x10);   // CTRL6_C: XL_HM_MODE = 1 (low-power/normal ODRs)
  lsmWrite(0x10, 0x48);   // CTRL1_XL: 104 Hz normal mode, +/-4 g
  lsmReg(0x1C);           // clear any latched tap (TAP_SRC)
  lsmReg(0x1B);           // + WAKE_UP_SRC: belt and braces, a stuck latched INT1 would wake us in a loop
}

inline bool lsmBringUp(int sda, int scl, int attempts = 10) {
  Wire.begin(sda, scl); Wire.setClock(400000);
  for (int i = 0; i < attempts; i++) {
    uint8_t who = lsmReg(0x0F);
    if (who == LSM_WHO_AM_I || who == LSM_WHO_AM_I_DSV) {
      lsmIsDSV = (who == LSM_WHO_AM_I_DSV);
      Serial.println(lsmIsDSV ? "# LSM6DSV found (v1.5 board)" : "# LSM6DS3TR-C found (v1 board)");
      lsmConfigureRun(); return true;
    }
    Serial.print("# LSM6 WHO_AM_I="); Serial.print(who, HEX); Serial.println(" (want 6A or 70), retry");
    delay(100);
  }
  return false;
}

// One sample. Raw gyro counts (remapped) for the bias logic, accel in g, gyro raw.
struct LsmSample { int16_t g[3]; float a[3]; };
inline bool lsmReadSample(LsmSample& s) {
  uint8_t d[12];
  if (!lsmRead(0x22, 12, d)) return false;
  int16_t gr[3], ar[3];
  for (int i = 0; i < 3; i++) {
    gr[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));
    ar[i] = (int16_t)(d[6 + 2 * i] | (d[6 + 2 * i + 1] << 8));
  }
  for (int i = 0; i < 3; i++) {
    s.g[i] = RING_AXES.sign[i] * gr[RING_AXES.src[i]];
    s.a[i] = RING_AXES.sign[i] * ar[RING_AXES.src[i]] / LSM_ACCEL_LSB_PER_G;
  }
  return true;
}

// Returns true once per detected knock (double tap); reading TAP_SRC clears the latch.
inline bool lsmKnocked() { return (lsmReg(lsmIsDSV ? 0x46 : 0x1C) & 0x10) != 0; }

// Gyro zero with motion-reject (same idea as calibrateGyroBiasStable for the MPU).
template <typename Bias>
inline void lsmCalibrateGyro(Bias& b, int samples = 200, int attempts = 5) {
  long best = 1L << 30;
  for (int a = 0; a < attempts; a++) {
    long sx = 0, sy = 0, sz = 0; int got = 0;
    int16_t mn[3] = {32767, 32767, 32767}, mx[3] = {-32768, -32768, -32768};
    for (int i = 0; i < samples; i++) {
      LsmSample s;
      if (lsmReadSample(s)) {
        sx += s.g[0]; sy += s.g[1]; sz += s.g[2]; got++;
        for (int k = 0; k < 3; k++) { mn[k] = min(mn[k], s.g[k]); mx[k] = max(mx[k], s.g[k]); }
      }
      delay(10);
    }
    if (!got) continue;
    long spread = max((long)(mx[0] - mn[0]), max((long)(mx[1] - mn[1]), (long)(mx[2] - mn[2])));
    if (spread < best) {
      best = spread;
      b.GX0 = b.GX0ref = sx / got; b.GY0 = b.GY0ref = sy / got; b.GZ0 = b.GZ0ref = sz / got;
    }
    if (spread <= 350) break;            // ~25 dps peak-to-peak = still enough
  }
  b.restSumX = b.restSumY = b.restSumZ = 0; b.restN = 0;
  Serial.print("# gyro zero: "); Serial.print(b.GX0); Serial.print(' ');
  Serial.print(b.GY0); Serial.print(' '); Serial.println(b.GZ0);
}

}  // namespace lyra
