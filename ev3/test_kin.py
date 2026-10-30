"""Desktop sanity tests for ev3/lyra_kin.py:  python3 ev3/test_kin.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lyra_kin as k

# 1. FK(IK(p)) == p for any point on the reachable surface
rmin, rmax, zmin, zmax = k.reach_envelope()
worst = 0.0
for yaw in range(-30, 31, 10):
    for pitch in range(-20, 50, 10):
        b0 = k.yaw_to_motor(yaw)
        l0 = k.clamp(k.pitch_to_motor(pitch), k.LIFT_TOP, k.LIFT_BOTTOM)
        p = k.forward(b0, l0)
        b, l, miss = k.inverse(*p)
        worst = max(worst, miss, abs(b - b0) / k.BASE_DEG_PER_YAW)
assert worst < 1e-6, worst

# 2. unreachable target reports a real miss, and the tip lands on the nearest point
b, l, miss = k.inverse(40, 0, 12)
assert 20 < miss < 30, miss

# 3. min-jerk: endpoints exact, monotonic, zero end velocity
assert k.min_jerk(0) == 0 and abs(k.min_jerk(1) - 1) < 1e-12
vals = [k.min_jerk(i / 100) for i in range(101)]
assert all(b2 >= a for a, b2 in zip(vals, vals[1:]))
assert (vals[1] - vals[0]) < 1e-3 and (vals[-1] - vals[-2]) < 1e-3

# 4. sampled joint move never exceeds motor speed caps and ends exactly on target
start, end = (0, 0), (-120, -150)
prev, pt, peak = start, 0.0, 0.0
for t, b, l in k.sample_joint_move(start, end):
    dt = t - pt
    peak = max(peak, abs(b - prev[0]) / dt / k.BASE_VMAX, abs(l - prev[1]) / dt / k.LIFT_VMAX)
    prev, pt = (b, l), t
assert abs(prev[0] - end[0]) < 1e-9 and abs(prev[1] - end[1]) < 1e-9
assert peak <= 1.0 + 1e-6, peak

# 5. safe planner: a yaw swing with the lift low gets a raise-swing-lower route
path = k.plan_safe((0, 0), (-120, 0))
assert len(path) == 3 and path[0][1] <= k.LIFT_TRAVEL and path[1][1] <= k.LIFT_TRAVEL and path[2] == (-120, 0)
assert k.plan_safe((0, -170), (-120, -170)) == [(-120, -170)]
assert k.plan_safe((0, 0), (0, -100)) == [(0, -100)]

# 6. tool-space line stays within miss tolerance on a reachable arc
ok = True
p0 = k.forward(0, -60)
p1 = k.forward(-80, -60)                 # same pitch, different yaw -> on the surface
for _, b, l, miss in k.sample_line(p0, p1):
    ok &= miss < 0.5
assert ok

print("all kinematics tests passed  (reach r=%.1f..%.1f cm, z=%.1f..%.1f cm)" % (rmin, rmax, zmin, zmax))
