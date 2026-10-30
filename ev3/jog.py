#!/usr/bin/env pybricks-micropython
"""
Tuning helper: jog ONE motor by a precise relative angle, hold briefly, report.

Usage (on the brick):
    pybricks-micropython jog.py <A|B|C> <angle_deg> [speed]

  A = base, B = lift, C = gripper.  angle may be negative.
  Prints the motor's cumulative angle so we can map safe ranges.

Used during movement tuning to find each joint's safe travel without crashing.
"""
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import sys

PORTS = {'A': Port.A, 'B': Port.B, 'C': Port.C}

letter = sys.argv[1].upper()
angle = float(sys.argv[2])
speed = float(sys.argv[3]) if len(sys.argv) > 3 else 70   # gentle default

m = Motor(PORTS[letter])
start = m.angle()
m.run_angle(speed, angle, then=Stop.HOLD)
print("motor", letter, "moved", angle, "deg :", start, "->", m.angle())
wait(1000)   # hold a moment so you can see the result before it releases
