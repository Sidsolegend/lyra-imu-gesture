// =============================================================================
// lyra_system.h  -  Shared runtime hardening: watchdog, reset forensics,
//                   runtime I2C fault recovery, power-management options
// =============================================================================
// New in the hardening pass (change #8; see OPTIMIZATIONS.md). The original
// sketches had strong COLD-BOOT bring-up (mpuBringUp) but nothing for faults
// that happen HOURS into a session:
//   - a wedged I2C transaction mid-wear (dupont wiring + a wrist that moves)
//     made loop() spin on readMPU() failures forever with no self-heal
//     (newlyra_final.ino:454 just `delay(10); return;` per failure),
//   - a firmware hang (BLE stack stall, I2C clock-stretch deadlock) required
//     a physical power-cycle - and the C3 SuperMini has no EN button,
//   - a brownout reboot (LiPo sag during a BLE burst) was indistinguishable
//     from a normal boot in the serial log, hiding a hardware problem.
//
// Everything here is header-only + inline, no heap, and safe on both the
// classic ESP32 (Arduino core 2.x / IDF 4.4) and the ESP32-C3 (core 3.x /
// IDF 5.x) via the version guards below.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "lyra_imu.h"

namespace lyra {

// ---------------------------------------------------------------------------
// 1) Task watchdog: if loop() stops being scheduled (I2C deadlock, BLE stack
//    stall), reboot instead of hanging until the battery dies. 8 s is far
//    above the worst legitimate blocking call in these sketches
//    (calibrateGyroBias(200) ~= 0.8 s incl. I2C time).
// ---------------------------------------------------------------------------
static constexpr uint32_t WDT_TIMEOUT_S = 8;

inline void wdtInit() {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // Arduino core 3.x (C3 SuperMini ships on this): reconfigure + subscribe.
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;            // watch only tasks we add explicitly
  cfg.trigger_panic = true;          // panic -> clean reboot with backtrace
  esp_task_wdt_reconfigure(&cfg);    // WDT already inited by the core
#else
  // Arduino core 2.x (classic ESP32 / IDF 4.4)
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);            // subscribe the Arduino loopTask
  Serial.print("# watchdog armed: "); Serial.print(WDT_TIMEOUT_S);
  Serial.println(" s");
}

// Call once per loop() pass. Cheap (a register write).
inline void wdtFeed() { esp_task_wdt_reset(); }

// ---------------------------------------------------------------------------
// 2) Reset forensics: report WHY we booted. A brownout reset showing up in
//    the log is the difference between "weird BLE bug" and "battery sagged
//    below 2.9 V during a radio burst" (the C3's brownout detector is on by
//    default; this makes its resets visible instead of silent).
// ---------------------------------------------------------------------------
inline const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_SW:        return "software reset";
    case ESP_RST_PANIC:     return "panic (crash)";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog (loop hung)";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply sagged - check battery/wiring)";
    case ESP_RST_DEEPSLEEP: return "deep-sleep wake";
    default:                return "other/unknown";
  }
}

inline void reportResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print("# reset reason: "); Serial.println(resetReasonName(r));
  if (r == ESP_RST_BROWNOUT)
    Serial.println("# !! brownout: LiPo sag or thin supply wire. BLE TX bursts "
                   "draw ~300 mA peaks; add bulk capacitance or shorter leads.");
  if (r == ESP_RST_TASK_WDT)
    Serial.println("# !! last session hung and the watchdog recovered it.");
}

// ---------------------------------------------------------------------------
// 3) Runtime I2C fault recovery: consecutive-failure counter around readMPU.
//    Transient NACKs happen (wire flex); a run of them means the sensor or
//    bus is wedged, so re-run the full cold-boot bring-up (bus recover +
//    DEVICE_RESET + reconfigure). The gyro zero (raw counts) is preserved -
//    a re-init does not invalidate it, so no still-hold recalibration is
//    forced on the wearer mid-session.
// ---------------------------------------------------------------------------
struct I2cHealth {
  uint16_t consecutiveFailures = 0;
  uint32_t totalFailures = 0;
  uint32_t recoveries = 0;
  static constexpr uint16_t RECOVER_AFTER = 25;   // ~250 ms of dead bus @ 10 ms
};

// Wrap every hot-loop readMPU with this. Returns true if the read (or a
// recovery) succeeded; on recovery the current sample is skipped.
inline bool i2cGuardedRead(I2cHealth& h, byte reg, uint8_t count, uint8_t* buf,
                           int sda, int scl) {
  if (readMPU(reg, count, buf)) {
    h.consecutiveFailures = 0;
    return true;
  }
  h.consecutiveFailures++;
  h.totalFailures++;
  if (h.consecutiveFailures >= I2cHealth::RECOVER_AFTER) {
    Serial.print("# I2C wedged ("); Serial.print(h.consecutiveFailures);
    Serial.println(" consecutive failures) -- recovering bus + re-init MPU");
    i2cBusRecover(sda, scl);
    if (initMPU()) {
      h.recoveries++;
      h.consecutiveFailures = 0;
      Serial.println("# MPU recovered.");
    }
    // If init failed we fall through; the counter stays saturated and we
    // retry the full recovery every RECOVER_AFTER further failures. The
    // watchdog is NOT starved because the caller keeps looping + feeding.
    else h.consecutiveFailures = 0;
  }
  return false;
}

// ---------------------------------------------------------------------------
// 4) Power management - applied options + an honest analysis of the rest.
//
//    WHAT THE FIRMWARE DOES TODAY (all builds): full-speed busy loop. The
//    loop polls micros() (SampleClock) between 10 ms ticks, CPU at the
//    default 160 MHz, radio always on, MPU in continuous mode. Rough C3
//    budget: CPU ~20-30 mA + BLE advertising/connection on top; a 500 mAh
//    LiPo gives on the order of a day, not a week.
//
//    WHAT IS SAFE TO ENABLE (compile-time flags below):
//    - LYRA_OPT_CPU80MHZ: drop the CPU to 80 MHz. The DSP here is trivial
//      (9 features over 60 floats every ~600 ms event); BLE + Wi-Fi coexist
//      fine at 80 MHz. ~30-40% CPU-power cut, zero behavioral change.
//      Left OFF by default only because it was not what was flashed for the
//      validated study runs.
//
//    WHAT IS DELIBERATELY *NOT* ENABLED (and why - measure before shipping):
//    - Light sleep between samples: automatic light sleep with BLE requires
//      an external 32 kHz crystal on classic ESP32 and careful
//      esp_pm_configure + NimBLE modem-sleep coordination on the C3; with a
//      10 ms sample grid the radio would thrash sleep/wake. The RIGHT design
//      is coarser: sleep between GESTURE WINDOWS, waking on the MPU-6050's
//      hardware motion interrupt (register 0x38 MOT_EN, INT pin -> a C3 RTC
//      GPIO). That turns idle wrist time (most of the day) into ~1-2 mA.
//      It changes the trigger path (interrupt vs 1.35 g polling), so it MUST
//      be re-validated against the dataset before it ships - which is why it
//      is a documented roadmap item (reviews/ROADMAP.md), not a flag here.
//    - BLE connection-interval tuning: as a HID *peripheral* we can request
//      a longer interval (e.g. 30-50 ms) to cut radio duty, but hosts are
//      free to reject it and media-key latency rides on it. Needs on-air
//      measurement; see ENGINEERING_REVIEW.md (BLE section).
// ---------------------------------------------------------------------------
// #define LYRA_OPT_CPU80MHZ 1   // uncomment to run the CPU at 80 MHz

// ---------------------------------------------------------------------------
// 4b) Motion-interrupt power mode (ROADMAP #9). COMPILE-FLAG GATED and OFF by
//     default. When LYRA_OPT_MOTION_WAKE is defined, a build can put the C3 into
//     light sleep between gesture windows and let the MPU-6050's MOT_EN INT pin
//     (configured via lyra_imu.h configureMotionInterrupt) wake it on the RTC
//     GPIO given by LYRA_MOTION_INT_GPIO. This turns idle wrist time into ~1-2 mA
//     instead of the full-speed busy loop.
//
//     LEFT OFF because it changes the trigger path (edge wake vs polled 1.35 g)
//     and MUST be re-validated against the dataset before shipping (ROADMAP #9 /
//     the analysis in section 4 above). The code path is provided so that
//     validation work has something concrete to measure. Pair with
//     LYRA_OPT_CPU80MHZ for the CPU-side saving.
// ---------------------------------------------------------------------------
// #define LYRA_OPT_MOTION_WAKE 1        // uncomment to enable light-sleep + INT wake
// #define LYRA_MOTION_INT_GPIO 3        // C3 RTC-capable GPIO wired to MPU INT

#ifdef LYRA_OPT_MOTION_WAKE
#include "esp_sleep.h"
#ifndef LYRA_MOTION_INT_GPIO
#define LYRA_MOTION_INT_GPIO 3
#endif
// Arm the MPU motion interrupt, then light-sleep until the INT pin rises (motion)
// or `maxIdleMs` elapses (safety wake so BLE/watchdog housekeeping still runs).
// Returns true if a motion edge woke us (a gesture is likely starting). The
// caller then restoreContinuousMode() and runs the normal capture path.
inline bool motionWakeIdle(uint32_t maxIdleMs = 2000) {
  configureMotionInterrupt();
  clearMotionInterrupt();                                  // start from a clean latch
  gpio_num_t pin = (gpio_num_t)LYRA_MOTION_INT_GPIO;
  esp_sleep_enable_ext0_wakeup(pin, 1);                    // wake on INT HIGH (active-high)
  esp_sleep_enable_timer_wakeup((uint64_t)maxIdleMs * 1000ULL);
  esp_light_sleep_start();                                 // <-- CPU idles here at ~mA
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  restoreContinuousMode();
  clearMotionInterrupt();
  return cause == ESP_SLEEP_WAKEUP_EXT0;
}
#endif  // LYRA_OPT_MOTION_WAKE

inline void powerInit() {
#ifdef LYRA_OPT_CPU80MHZ
  setCpuFrequencyMhz(80);
  Serial.println("# power: CPU locked to 80 MHz");
#endif
#ifdef LYRA_OPT_MOTION_WAKE
  Serial.print("# power: motion-interrupt wake ENABLED on GPIO");
  Serial.print(LYRA_MOTION_INT_GPIO);
  Serial.println(" (trigger path differs -- re-validate against the dataset)");
#endif
}

// One call that applies the whole hardening block. Place at the TOP of
// setup() (before mpuBringUp) so the watchdog covers bring-up too... except
// bring-up legitimately blocks up to ~5 s on a bad sensor, so we arm the
// watchdog AFTER bring-up instead. Call order in the sketches:
//   systemBoot()  ->  mpuBringUp()  ->  wdtInit()
inline void systemBoot() {
  reportResetReason();
  powerInit();
}

}  // namespace lyra
