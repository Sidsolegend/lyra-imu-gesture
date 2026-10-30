#!/usr/bin/env pybricks-micropython
"""
Quick base-range test, measured cleanly FROM the start position.

Usage (on the brick):
    pybricks-micropython range_test.py [right=270] [left=-360] [lift_clear=-160]

Raises + holds the lift, then: start -> RIGHT -> start -> LEFT -> start,
lowers the lift, done. Each extreme is reached from zero so right/left are
independent (unlike a continuous sweep). + = right, - = left.
"""
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import sys

base = Motor(Port.A)
lift = Motor(Port.B)

right = float(sys.argv[1]) if len(sys.argv) > 1 else 270.0
left = float(sys.argv[2]) if len(sys.argv) > 2 else -360.0
lift_clear = float(sys.argv[3]) if len(sys.argv) > 3 else -160.0

base.reset_angle(0)
lift.reset_angle(0)

lift.run_angle(130, lift_clear, then=Stop.HOLD)   # raise + hold (relative from sag)
wait(500)
base.run_target(150, right, then=Stop.HOLD); wait(1200)   # RIGHT from start
base.run_target(150, 0,     then=Stop.HOLD); wait(800)    # back to start
base.run_target(150, left,  then=Stop.HOLD); wait(1200)   # LEFT from start
base.run_target(150, 0,     then=Stop.HOLD); wait(800)    # back to start
lift.run_target(130, 0,     then=Stop.HOLD); wait(300)    # lower lift to start
base.stop(); lift.stop()
print("range test done: right", right, "| left", left)
