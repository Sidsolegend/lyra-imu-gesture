// =============================================================================
// lyra_touch.h  -  Two-pad emblem touch (TTP223 x2) -> touch gestures
// =============================================================================
// The emblem has two half-ring pads (left / right of the finger axis), each with its own
// TTP223 (momentary, active-high output). From the two digital lines this makes:
//   TAP_L / TAP_R         short touch (< 350 ms), no second tap within 300 ms
//   DOUBLE_L / DOUBLE_R   two taps on the same pad within 300 ms
//   LONG_L / LONG_R       held >= 700 ms (fires once while still held)
//   SWIPE_LR / SWIPE_RL   one pad then the other within 250 ms (thumb sliding across)
// plus touchHeld() for hold-to-gesture ("clutch") mode.
// v1.5 ring: ONE pad over the lens (one TTP223 on GPIO0; GPIO1 carries the charger status there). Call
// touchSetSinglePad(true) (the ring sketch does it when it finds the v1.5 board's LSM6DSV): the R line
// is ignored, so taps / doubles / longs come out as the _L events and there is no swipe.
// TTP223 note: a single touch longer than ~6 s auto-releases (chip recalibration), so
// the clutch is meant for a gesture, not a permanent hold.
// Non-blocking: call touchService() every loop pass; it returns at most one event.
// =============================================================================
#pragma once
#include <Arduino.h>

#ifndef LYRA_PIN_TOUCH_L
#define LYRA_PIN_TOUCH_L 0          // ring: GPIO0 (deep-sleep wake capable)
#endif
#ifndef LYRA_PIN_TOUCH_R
#define LYRA_PIN_TOUCH_R 1          // ring: GPIO1 (deep-sleep wake capable)
#endif

namespace lyra {

enum TouchEvent : uint8_t { T_NONE = 0, T_TAP_L, T_TAP_R, T_DOUBLE_L, T_DOUBLE_R,
                            T_LONG_L, T_LONG_R, T_SWIPE_LR, T_SWIPE_RL };

inline const char* touchName(TouchEvent e) {
  switch (e) {
    case T_TAP_L: return "TAP,L";       case T_TAP_R: return "TAP,R";
    case T_DOUBLE_L: return "DOUBLE,L"; case T_DOUBLE_R: return "DOUBLE,R";
    case T_LONG_L: return "LONG,L";     case T_LONG_R: return "LONG,R";
    case T_SWIPE_LR: return "SWIPE,LR"; case T_SWIPE_RL: return "SWIPE,RL";
    default: return "NONE";
  }
}

static constexpr uint16_t TOUCH_DEBOUNCE_MS = 15;
static constexpr uint16_t TOUCH_TAP_MAX_MS  = 350;
static constexpr uint16_t TOUCH_DOUBLE_MS   = 300;
static constexpr uint16_t TOUCH_LONG_MS     = 700;
static constexpr uint16_t TOUCH_SWIPE_MS    = 250;

struct Pad {
  uint8_t pin; bool state = false, raw = false;
  uint32_t rawSince = 0, downAt = 0, upAt = 0;
  bool longFired = false, swipeUsed = false;
  uint8_t taps = 0;                        // completed short taps waiting for a possible double
};
static Pad padL{LYRA_PIN_TOUCH_L}, padR{LYRA_PIN_TOUCH_R};
static bool touchSinglePad = false;             // v1.5 board: only the L pad exists

inline void touchInit() { pinMode(LYRA_PIN_TOUCH_L, INPUT); pinMode(LYRA_PIN_TOUCH_R, INPUT); }
inline void touchSetSinglePad(bool on) {
  touchSinglePad = on;
  if (on) { padR.state = padR.raw = false; }   // pin left to powerSetBoardV15() (charger status)
}
inline bool touchHeld() { return padL.state || padR.state; }

// debounced edge: returns +1 press, -1 release, 0 nothing
inline int padEdge(Pad& p, uint32_t now) {
  bool r = digitalRead(p.pin) == HIGH;
  if (r != p.raw) { p.raw = r; p.rawSince = now; }
  if (p.raw != p.state && now - p.rawSince >= TOUCH_DEBOUNCE_MS) { p.state = p.raw; return p.state ? 1 : -1; }
  return 0;
}

inline TouchEvent touchService() {
  uint32_t now = millis();
  int eL = padEdge(padL, now), eR = touchSinglePad ? 0 : padEdge(padR, now);
  if (eL == 1) { padL.downAt = now; padL.longFired = false; padL.swipeUsed = false; }
  if (eR == 1) { padR.downAt = now; padR.longFired = false; padR.swipeUsed = false; }
  // swipe: the second pad goes down shortly after the first one
  if (eR == 1 && padL.downAt && now - padL.downAt <= TOUCH_SWIPE_MS && !padL.swipeUsed) {
    padL.swipeUsed = padR.swipeUsed = true; padL.taps = padR.taps = 0; return T_SWIPE_LR;
  }
  if (eL == 1 && padR.downAt && now - padR.downAt <= TOUCH_SWIPE_MS && !padR.swipeUsed) {
    padL.swipeUsed = padR.swipeUsed = true; padL.taps = padR.taps = 0; return T_SWIPE_RL;
  }
  // long press
  if (padL.state && !padL.longFired && !padL.swipeUsed && now - padL.downAt >= TOUCH_LONG_MS) { padL.longFired = true; padL.taps = 0; return T_LONG_L; }
  if (padR.state && !padR.longFired && !padR.swipeUsed && now - padR.downAt >= TOUCH_LONG_MS) { padR.longFired = true; padR.taps = 0; return T_LONG_R; }
  // releases -> taps
  if (eL == -1) { padL.upAt = now; if (!padL.longFired && !padL.swipeUsed && now - padL.downAt < TOUCH_TAP_MAX_MS) padL.taps++; }
  if (eR == -1) { padR.upAt = now; if (!padR.longFired && !padR.swipeUsed && now - padR.downAt < TOUCH_TAP_MAX_MS) padR.taps++; }
  if (padL.taps >= 2) { padL.taps = 0; return T_DOUBLE_L; }
  if (padR.taps >= 2) { padR.taps = 0; return T_DOUBLE_R; }
  if (padL.taps == 1 && !padL.state && now - padL.upAt > TOUCH_DOUBLE_MS) { padL.taps = 0; return T_TAP_L; }
  if (padR.taps == 1 && !padR.state && now - padR.upAt > TOUCH_DOUBLE_MS) { padR.taps = 0; return T_TAP_R; }
  // forget a released pad's press time once it can no longer start a swipe
  if (!padL.state && padL.downAt && now - padL.upAt > TOUCH_SWIPE_MS) padL.downAt = 0;
  if (!padR.state && padR.downAt && now - padR.upAt > TOUCH_SWIPE_MS) padR.downAt = 0;
  return T_NONE;
}

}  // namespace lyra
