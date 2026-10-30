#!/usr/bin/env pybricks-micropython
"""
Tuning helper: raise the lift to clearance and HOLD it, then swing the base.

Usage (on the brick):
    pybricks-micropython jog_base.py <base_angle> [lift_clearance=-160] [speed=100]

The lift sags to its bottom (0) when released, so it's raised relative to that
each run. The base swing happens with the lift held up, matching how the arm
must actually move (base only clears when lift is high). Reports base angle.
"""
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import sys

base = Motor(Port.A)
lift = Motor(Port.B)

base_angle = float(sys.argv[1])
lift_clear = float(sys.argv[2]) if len(sys.argv) > 2 else -160.0
speed = float(sys.argv[3]) if len(sys.argv) > 3 else 100.0

# raise lift to clearance (relative from sagged bottom) and hold it there
lift.run_angle(120, lift_clear, then=Stop.HOLD)
# now swing the base while the lift stays up
b0 = base.angle()
base.run_angle(speed, base_angle, then=Stop.HOLD)
print("lift held at", lift_clear, "| base", b0, "->", base.angle())
wait(1500)   # hold so you can see it, then both motors release (lift sags)
