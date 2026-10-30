#!/usr/bin/env python3
"""
optimized/ev3/lyra_config.py — single source of truth for the EV3 arm's tuning
constants, its command vocabulary, and the wrist-gesture -> arm-command mapping.

Fixes T12 / E6: the same block of arm constants (BASE_SPEED, LIFT_STEP, the
BASE/LIFT limits, GRIP_DUTY, ...) was copy-pasted into BOTH ev3/lyra_arm.py and
ev3/lyra_routines.py, and lyra_routines.py's header even admits "mirror
ev3/lyra_arm.py ... keep the two in sync if you retune" — a manual sync that had
already started to drift. Import from here instead so a retune happens once:

    from lyra_config import (BASE_SPEED, LIFT_SPEED, GRIP_SPEED, BASE_STEP,
                             LIFT_STEP, BASE_MIN, BASE_MAX, LIFT_TOP,
                             LIFT_BOTTOM, LIFT_TRAVEL, GRIP_DUTY, clamp)

Pure standard library (NO pybricks import) so it loads on the brick AND under
desktop CPython for the unit tests. The MicroPython on the EV3 supports plain
module import of a co-located file, so dropping this next to lyra_arm.py /
lyra_routines.py on the brick is enough.

Provenance of every value (kept byte-identical to what shipped):
  ev3/lyra_arm.py:42-72   and   ev3/lyra_routines.py:39-47
"""

# --- motor speeds (deg/s) ----------------------------------------------------
BASE_SPEED = 150          # base rotate
LIFT_SPEED = 130          # shoulder / lift
GRIP_SPEED = 200          # gripper

# --- per-step travel (motor-deg) ---------------------------------------------
BASE_STEP = 60            # base is high-geared
LIFT_STEP = 45

# --- soft limits, relative to the launch position ----------------------------
# Base: + = right, - = left (confirmed on hardware); left has far more range.
BASE_MIN, BASE_MAX = -360, 270
# Lift: - = UP, 0 = bottom (it SAGS to the bottom when a motor is released).
LIFT_TOP = -180           # most-up (physical max)
LIFT_BOTTOM = 0           # rest / sagged-down
LIFT_TRAVEL = -160        # clearance the lift must reach before the base swings

# --- gripper -----------------------------------------------------------------
GRIP_DUTY = 75            # squeeze torque (%) before "closed"

# --- homing (lyra_arm.py only, but kept here so all tuning lives in one file) -
AUTO_HOME = False
HOME_SPEED = 80
HOME_SEARCH_MS = 1500

# --- command vocabulary (shared by lyra_arm.py dispatch + lyra_routines.py do) -
# The exact set the bridge, the on-brick player, and the .lyra routine files all
# speak. STEP_COMMANDS are the movement primitives a routine may contain.
STEP_COMMANDS = ("BASE+", "BASE-", "LIFT+", "LIFT-", "GRIP", "RAISE", "PARK")
# Extra control commands lyra_arm.py's dispatch() accepts over the live link.
CONTROL_COMMANDS = ("HOME", "STOP", "TEST", "PING")
COMMANDS = STEP_COMMANDS + CONTROL_COMMANDS

# --- wrist gesture event -> arm command (mirrors bridge/lyra_to_ev3.py MAPPING) -
# Kept here so the bridge and any on-brick tooling agree on the mapping.
GESTURE_TO_COMMAND = {
    "FLICK,R":   "BASE+",   # flick right  -> base rotates right
    "FLICK,L":   "BASE-",   # flick left   -> base rotates left
    "TWIST,CW":  "LIFT+",   # twist CW     -> lift up
    "TWIST,CCW": "LIFT-",   # twist CCW    -> lift down
    "PINCH":     "GRIP",    # pinch        -> open/close gripper
}


def clamp(v, lo, hi):
    """Identical helper both ev3 scripts defined locally (lyra_arm.py:75,
    lyra_routines.py:59). Centralized so there is exactly one."""
    return lo if v < lo else (hi if v > hi else v)
