// =============================================================================
// lyra_power.h  -  Ring battery, charging, reserve and deep sleep
// =============================================================================
// Hardware (ring schematic): VBAT -> 1M/1M divider -> GPIO4 (ADC1_CH4, 100 nF), TP4054
// ~CHRG (open drain, LOW while charging) -> GPIO20 with a 10k pull-up (R31). v1.5 board: ~CHRG is on
// GPIO1 instead (powerSetBoardV15(), called once the LSM6DSV says it's a v1.5 board). After a charge
// ends the TP4054 keeps a weak 8-35 uA pull-down on CHRG: the 45k internal pull-up alone would
// leave the pin between VIL and VIH, so the external 10k is what makes "done" read HIGH.
// Wake sources in deep sleep: TOUCH_L (GPIO0), TOUCH_R (GPIO1), IMU INT1 / knock (GPIO3)
// - the only ESP32-C3 pins that can wake it from deep sleep are GPIO0..5.
//
// Battery reserve: below RESERVE_PCT the ring stops gestures/BLE advertising and sleeps,
// keeping just enough charge to wake on a knock, report "Ring at N% charge" and sleep
// again (that's the "dead ring knock" moment). The 301012 cell's own protection circuit
// cuts off around 3.0 V regardless.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "esp_sleep.h"

#ifndef LYRA_PIN_VBAT
#define LYRA_PIN_VBAT 4
#endif
#ifndef LYRA_PIN_CHRG
#define LYRA_PIN_CHRG 20
#endif
#ifndef LYRA_PIN_IMU_INT
#define LYRA_PIN_IMU_INT 3
#endif

namespace lyra {

static constexpr uint8_t RESERVE_PCT = 5;

// LiPo open-circuit voltage -> % (typical 1-cell curve, light load)
inline uint8_t lipoPercent(int mv) {
  static const int16_t mvT[] = {3300, 3500, 3600, 3700, 3750, 3800, 3850, 3900, 4000, 4100, 4200};
  static const uint8_t pcT[] = {   0,    5,   10,   20,   30,   40,   50,   60,   75,   90,  100};
  if (mv <= mvT[0]) return 0;
  for (int i = 1; i < 11; i++)
    if (mv < mvT[i]) return pcT[i - 1] + (pcT[i] - pcT[i - 1]) * (mv - mvT[i - 1]) / (mvT[i] - mvT[i - 1]);
  return 100;
}

struct Battery { int mv = 0; uint8_t pct = 0; bool charging = false; bool justFull = false; };
static Battery batt;
// survives deep sleep: was the cell charging when we went to sleep? (lets the first reading after a
// timer wake on the dock still see the charging -> done edge and report FULL)
RTC_DATA_ATTR static bool rtcWasCharging = false;
// survives deep sleep: v1.5 board? (GPIO1 is touch R on v1 but the charger status on v1.5, and
// wakeReason() runs before the IMU tells us which board this is)
RTC_DATA_ATTR static bool rtcBoardV15 = false;

static uint8_t chrgPin = LYRA_PIN_CHRG;      // 20 on v1, 1 on v1.5

inline void powerPinsInit() {
  pinMode(chrgPin, INPUT_PULLUP);
  analogReadResolution(12);
}
inline void powerSetBoardV15(bool v15) {
  if (!v15) return;
  pinMode(LYRA_PIN_CHRG, INPUT);               // GPIO20 is unconnected on v1.5
  chrgPin = 1; pinMode(chrgPin, INPUT_PULLUP);
  rtcBoardV15 = true;
}

// Call ~once a second. Sets batt.justFull for one call when a charge cycle ends.
inline void batteryUpdate() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(LYRA_PIN_VBAT);
  int mv = (int)(sum / 8) * 2;                       // 1M/1M divider
  batt.mv = batt.mv ? (batt.mv * 3 + mv) / 4 : mv;   // light smoothing
  batt.pct = lipoPercent(batt.mv);
  bool ch = digitalRead(chrgPin) == LOW;
  static bool first = true;
  bool prev = first ? rtcWasCharging : batt.charging; first = false;
  batt.justFull = prev && !ch && batt.mv > 4100;
  batt.charging = ch; rtcWasCharging = ch;
}

inline bool batteryInReserve() { return !batt.charging && batt.mv > 0 && batt.pct <= RESERVE_PCT; }

enum WakeReason : uint8_t { WAKE_POWER_ON = 0, WAKE_TOUCH, WAKE_KNOCK, WAKE_OTHER, WAKE_TIMER, WAKE_CHARGE };
inline WakeReason wakeReason() {
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) return WAKE_TIMER;
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_GPIO) return WAKE_POWER_ON;
  uint64_t m = esp_sleep_get_gpio_wakeup_status();
  if (m & (1ULL << LYRA_PIN_IMU_INT)) return WAKE_KNOCK;
  if (rtcBoardV15 && (m & (1ULL << 1))) return WAKE_CHARGE;   // v1.5: ~CHRG went low (put on the dock)
  if (m & ((1ULL << 0) | (1ULL << 1))) return WAKE_TOUCH;
  return WAKE_OTHER;
}

// Deep sleep until a touch or a knock. Caller first puts the IMU into lsmConfigureSleep()
// and turns the LED / motor off. Wake = full reboot (setup() runs again; wakeReason()
// says why). ~tens of uA total: ESP32-C3 deep sleep + TTP223 (2-3.5 uA each; one on v1.5) + LSM6
// low-power accel + divider (2 uA) + LDO (1 uA on v1, ~6.5 uA TPS7A20 on v1.5).
// timerUs > 0 also wakes after that long (dock mode: check in with the lantern, sleep again).
// v1.5: also wakes when the charger starts (~CHRG low on GPIO1), so a sleeping ring put on the dock
// checks in at once. Only armed while NOT charging: during a charge the pin sits low and would wake
// it in a loop (the dock timer wake covers that time instead).
inline void deepSleepUntilTouchOrKnock(uint64_t timerUs = 0, bool singlePad = false) {
  // v1.5 (one touch pad): GPIO1 is the charger status line there, not a touch pad
  uint64_t mask = (1ULL << 0) | (singlePad ? 0 : (1ULL << 1)) | (1ULL << LYRA_PIN_IMU_INT);
  esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_HIGH);
  if (singlePad && !batt.charging) esp_deep_sleep_enable_gpio_wakeup(1ULL << 1, ESP_GPIO_WAKEUP_GPIO_LOW);   // R31 holds it high
  if (timerUs) esp_sleep_enable_timer_wakeup(timerUs);
  Serial.flush();
  esp_deep_sleep_start();
}

}  // namespace lyra
