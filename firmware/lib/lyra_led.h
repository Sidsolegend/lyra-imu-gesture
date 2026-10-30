// =============================================================================
// lyra_led.h  -  Gesture-feedback glow on one PWM LED (non-blocking)
// =============================================================================
// The ring's emblem LED (and, on the dev board, the SuperMini's on-board LED)
// glows whenever a gesture is recognised, so the wearer sees that the ring
// "heard" them without looking at a phone:
//   - any recognised gesture (flick / twist / pinch / custom): one glow
//     (fast rise, short hold, slow fade - reads as a pulse of light, not a blink)
//   - ARMED:    two quick glows
//   - DISARMED: one long, dim fade
//
// Non-blocking: ledGesture()/ledArmed() only start a pattern; ledService() must
// be called every loop() pass and advances it from millis(). No delay(), so the
// fixed-dt IMU sampling (SampleClock) is never disturbed.
//
// Power: the LED only lights for ~0.5 s per gesture, so at ~5 mA it averages
// microamps over a session - negligible next to BLE.
//
// Header-only + inline, no heap. Works on Arduino core 2.x (ledcSetup +
// ledcAttachPin) and 3.x (ledcAttach) via the version guard below.
// =============================================================================
#pragma once
#include <Arduino.h>

// Retarget for the ring PCB by defining these before the include.
#ifndef LYRA_PIN_LED
#define LYRA_PIN_LED 8              // ESP32-C3 SuperMini on-board LED = GPIO8
#endif
#ifndef LYRA_LED_ACTIVE_LOW
#define LYRA_LED_ACTIVE_LOW 1       // SuperMini LED lights when the pin is LOW
#endif

namespace lyra {

static constexpr uint32_t LED_PWM_HZ   = 5000;
static constexpr uint8_t  LED_PWM_BITS = 8;
static constexpr uint8_t  LED_MAX      = 255;   // cap here if the emblem is too bright
static constexpr uint8_t  LED_LEDC_CH  = 0;     // core 2.x only

// One step of a pattern: ramp from the previous level to `level` over `ms`.
struct LedStep { uint8_t level; uint16_t ms; };

static const LedStep LED_GLOW[]     = {{LED_MAX, 60}, {LED_MAX, 120}, {0, 380}};
static const LedStep LED_ARMED[]    = {{LED_MAX, 40}, {0, 120}, {LED_MAX, 40}, {0, 200}};
static const LedStep LED_DISARMED[] = {{LED_MAX / 3, 80}, {0, 700}};

struct LedState {
  const LedStep* steps = nullptr;
  uint8_t count = 0, idx = 0;
  uint8_t from = 0, level = 0;
  uint32_t stepStartMs = 0;
};
static LedState ledState;

inline void ledWriteRaw(uint8_t v) {
  // Perceptual (gamma ~2) curve so the fade looks smooth, not front-loaded.
  uint32_t duty = (uint32_t)v * v / LED_MAX;
  if (LYRA_LED_ACTIVE_LOW) duty = LED_MAX - duty;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LYRA_PIN_LED, duty);
#else
  ledcWrite(LED_LEDC_CH, duty);
#endif
}

inline void ledInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LYRA_PIN_LED, LED_PWM_HZ, LED_PWM_BITS);
#else
  ledcSetup(LED_LEDC_CH, LED_PWM_HZ, LED_PWM_BITS);
  ledcAttachPin(LYRA_PIN_LED, LED_LEDC_CH);
#endif
  ledWriteRaw(0);
}

inline void ledStart(const LedStep* steps, uint8_t count) {
  ledState.steps = steps; ledState.count = count; ledState.idx = 0;
  ledState.from = ledState.level;           // restart smoothly from wherever it is
  ledState.stepStartMs = millis();
}

static const LedStep LED_CHARGEUP[] = {{LED_MAX / 4, 300}, {LED_MAX / 2, 400}, {LED_MAX, 500}, {LED_MAX, 300}, {0, 600}};
static const LedStep LED_FLICKER[]  = {{LED_MAX / 6, 40}, {0, 80}, {LED_MAX / 5, 40}, {0, 160}, {LED_MAX / 8, 40}, {0, 60}};
static LedStep ledLevelBuf[10];               // built by ledLevel()
static bool ledBreathing = false;             // slow glow while charging (when no pattern runs)

inline void ledGesture()       { ledStart(LED_GLOW, sizeof(LED_GLOW) / sizeof(LED_GLOW[0])); }
inline void ledChargeUp()      { ledStart(LED_CHARGEUP, sizeof(LED_CHARGEUP) / sizeof(LED_CHARGEUP[0])); }
inline void ledLowFlicker()    { ledStart(LED_FLICKER, sizeof(LED_FLICKER) / sizeof(LED_FLICKER[0])); }
inline void ledBreathe(bool on) { ledBreathing = on; }
// battery level: 1-5 quick glows (20 % each, at least 1)
inline void ledLevel(uint8_t pct) {
  uint8_t n = constrain((pct + 19) / 20, 1, 5), k = 0;
  for (uint8_t i = 0; i < n; i++) { ledLevelBuf[k++] = {LED_MAX, 50}; ledLevelBuf[k++] = {0, 220}; }
  ledStart(ledLevelBuf, k);
}
inline void ledArmed(bool on) {
  if (on) ledStart(LED_ARMED, sizeof(LED_ARMED) / sizeof(LED_ARMED[0]));
  else    ledStart(LED_DISARMED, sizeof(LED_DISARMED) / sizeof(LED_DISARMED[0]));
}

// Call every loop() pass (before the sample-clock early return).
inline void ledService() {
  LedState& s = ledState;
  if (s.steps == nullptr) {
    if (ledBreathing) {                       // 4 s breathing cycle, 0..40 % brightness
      uint32_t t = millis() % 4000;
      uint32_t tri = t < 2000 ? t : 4000 - t;
      s.level = (uint8_t)(tri * (LED_MAX * 2 / 5) / 2000); ledWriteRaw(s.level);
    }
    return;
  }
  uint32_t now = millis();
  const LedStep& st = s.steps[s.idx];
  uint32_t t = now - s.stepStartMs;
  if (t >= st.ms) {
    s.level = st.level; ledWriteRaw(s.level);
    s.from = s.level; s.stepStartMs = now;
    if (++s.idx >= s.count) s.steps = nullptr;   // pattern finished
    return;
  }
  s.level = s.from + (int32_t)(st.level - s.from) * (int32_t)t / (int32_t)st.ms;
  ledWriteRaw(s.level);
}

}  // namespace lyra
