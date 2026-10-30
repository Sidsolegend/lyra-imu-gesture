// =============================================================================
// lyra_haptic.h  -  Coin-motor haptics (AO3400A low-side switch, PWM), non-blocking
// =============================================================================
// Same step-pattern engine as lyra_led.h: start a pattern, call hapticService() every
// loop pass. Strength is a PWM duty (the motor runs straight off VBAT, 3.0-4.2 V; 6 mm
// coin motors are rated ~3 V, so full duty is capped at ~75 %).
// Don't buzz during BLE bursts on a weak cell: the ring sketch skips haptics when the
// battery is in reserve.
// =============================================================================
#pragma once
#include <Arduino.h>

#ifndef LYRA_PIN_MOTOR
#define LYRA_PIN_MOTOR 5            // ring: GPIO5 -> 100R -> AO3400A gate (10k pull-down)
#endif

namespace lyra {

static constexpr uint8_t HAPTIC_MAX = 190;      // ~75 % of 255
static constexpr uint8_t HAPTIC_LEDC_CH = 1;    // core 2.x only (LED uses 0)

struct HapStep { uint8_t level; uint16_t ms; };   // hold `level` for `ms`
static const HapStep HAP_TICK[]     = {{HAPTIC_MAX, 35}, {0, 1}};
static const HapStep HAP_DOUBLE[]   = {{HAPTIC_MAX, 40}, {0, 90}, {HAPTIC_MAX, 40}, {0, 1}};
static const HapStep HAP_LONG[]     = {{HAPTIC_MAX, 220}, {0, 1}};
static const HapStep HAP_LOWBAT[]   = {{90, 60}, {0, 120}, {90, 60}, {0, 120}, {90, 60}, {0, 1}};
static const HapStep HAP_CHARGEUP[] = {{60, 250}, {100, 250}, {140, 250}, {HAPTIC_MAX, 450}, {0, 1}};

struct HapState { const HapStep* steps = nullptr; uint8_t count = 0, idx = 0; uint32_t t0 = 0; };
static HapState hap;

inline void hapticWrite(uint8_t v) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LYRA_PIN_MOTOR, v);
#else
  ledcWrite(HAPTIC_LEDC_CH, v);
#endif
}
inline void hapticInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LYRA_PIN_MOTOR, 20000, 8);       // 20 kHz: above hearing, no coil whine
#else
  ledcSetup(HAPTIC_LEDC_CH, 20000, 8); ledcAttachPin(LYRA_PIN_MOTOR, HAPTIC_LEDC_CH);
#endif
  hapticWrite(0);
}
inline void hapticStart(const HapStep* s, uint8_t n) { hap.steps = s; hap.count = n; hap.idx = 0; hap.t0 = millis(); hapticWrite(s[0].level); }
#define LYRA_HAP(p) hapticStart(p, sizeof(p) / sizeof(p[0]))
inline void hapticTick()     { LYRA_HAP(HAP_TICK); }
inline void hapticDouble()   { LYRA_HAP(HAP_DOUBLE); }
inline void hapticLong()     { LYRA_HAP(HAP_LONG); }
inline void hapticLowBat()   { LYRA_HAP(HAP_LOWBAT); }
inline void hapticChargeUp() { LYRA_HAP(HAP_CHARGEUP); }
inline bool hapticBusy() { return hap.steps != nullptr; }

inline void hapticService() {
  if (!hap.steps) return;
  uint32_t now = millis();
  if (now - hap.t0 < hap.steps[hap.idx].ms) return;
  hap.t0 = now;
  if (++hap.idx >= hap.count) { hap.steps = nullptr; hapticWrite(0); return; }
  hapticWrite(hap.steps[hap.idx].level);
}

}  // namespace lyra
