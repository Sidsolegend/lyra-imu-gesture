"""
LYRA EV3 arm -- kinematics + motion planning (pure math, no pybricks import, so
it runs on the brick AND under desktop CPython for tests: ev3/test_kin.py).

The arm is a 2-joint positioner (base YAW about vertical, lift PITCH at the
shoulder) plus a gripper. With one pitch joint the tool tip can only sit on a
circle of radius LINK_L in the vertical plane of the arm, so "IK" here is:

    given a target (x, y, z):   yaw   = atan2(y, x)
                                pitch = atan2(z - SHOULDER_H, r)      r = hypot(x, y)

which is the closest point on the reachable circle to the target (a true
least-squares projection). It returns that pitch plus the residual miss
distance, so callers can see when a target is unreachable instead of guessing.

Frame: origin on the table under the base axis, +x = forward at yaw 0,
+y = left of that (yaw is CCW-positive), +z up. Units: cm, degrees.

!! The four GEOMETRY numbers and the gear ratios below are PLACEHOLDERS -- measure
   your build (see EV3.md "Calibrating the kinematics") before trusting absolute
   positions. Relative moves and the maths itself don't depend on them.
"""
import math

# ---------------------------------------------------------------- geometry --
SHOULDER_H = 12.0         # shoulder axis height above the table (cm)
LINK_L     = 16.0         # shoulder axis -> gripper tip (cm)

# ---------------------------------------------- motor <-> joint conversion --
# Base: + motor = RIGHT = clockwise from above, so yaw (CCW+) = -motor/ratio.
BASE_DEG_PER_YAW   = 8.0      # motor deg per 1 deg of yaw   (high-geared)
YAW_AT_BASE_ZERO   = 0.0      # yaw (deg) at launch position (base motor angle 0)
# Lift: - motor = UP, 0 = bottom (sagged). pitch = P0 + (-motor)/ratio.
LIFT_DEG_PER_PITCH = 2.5      # motor deg per 1 deg of pitch
PITCH_AT_LIFT_ZERO = -25.0    # arm pitch (deg above horizontal) at the sagged bottom

# Joint limits in MOTOR degrees (match ev3/lyra_arm.py)
BASE_MIN, BASE_MAX = -360, 270
LIFT_TOP, LIFT_BOTTOM = -180, 0
LIFT_TRAVEL = -160            # lift must be at/above this before the base swings
YAW_SWING_DEG = 3.0           # yaw changes bigger than this need the lift clear

# ------------------------------------------------------------ velocity caps --
BASE_VMAX = 150.0             # motor deg/s
LIFT_VMAX = 130.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# -------------------------------------------------------- joint <-> motor ----
def yaw_to_motor(yaw):   return -(yaw - YAW_AT_BASE_ZERO) * BASE_DEG_PER_YAW
def motor_to_yaw(m):     return YAW_AT_BASE_ZERO - m / BASE_DEG_PER_YAW
def pitch_to_motor(p):   return -(p - PITCH_AT_LIFT_ZERO) * LIFT_DEG_PER_PITCH
def motor_to_pitch(m):   return PITCH_AT_LIFT_ZERO - m / LIFT_DEG_PER_PITCH


# ------------------------------------------------------------- kinematics ----
def forward(base_motor, lift_motor):
    """Motor angles -> tip (x, y, z) in cm."""
    yaw = math.radians(motor_to_yaw(base_motor))
    pitch = math.radians(motor_to_pitch(lift_motor))
    r = LINK_L * math.cos(pitch)
    return (r * math.cos(yaw), r * math.sin(yaw), SHOULDER_H + LINK_L * math.sin(pitch))


def inverse(x, y, z):
    """Target (x, y, z) -> (base_motor, lift_motor, miss_cm).

    Motor targets are clamped into the joint limits; miss_cm is the distance
    between the target and where the tip will really end up (0 = exact)."""
    yaw = math.degrees(math.atan2(y, x)) if (x or y) else motor_to_yaw(0)
    r = math.hypot(x, y)
    pitch = math.degrees(math.atan2(z - SHOULDER_H, r))
    b = clamp(yaw_to_motor(yaw), BASE_MIN, BASE_MAX)
    l = clamp(pitch_to_motor(pitch), LIFT_TOP, LIFT_BOTTOM)
    fx, fy, fz = forward(b, l)
    miss = math.sqrt((fx - x) ** 2 + (fy - y) ** 2 + (fz - z) ** 2)
    return b, l, miss


def reach_envelope():
    """(min_r, max_r, min_z, max_z) the tip can reach within the lift limits."""
    ps = [math.radians(motor_to_pitch(m)) for m in (LIFT_BOTTOM, LIFT_TOP)]
    rs = [LINK_L * math.cos(p) for p in ps]
    zs = [SHOULDER_H + LINK_L * math.sin(p) for p in ps]
    return min(rs), max(rs), min(zs), max(zs)


# ------------------------------------------------------------ trajectories ---
def min_jerk(s):
    """Smooth 0..1 -> 0..1 profile with zero velocity AND acceleration at both
    ends (no start/stop jolt; gentler on the geartrain than a trapezoid)."""
    s = clamp(s, 0.0, 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def move_duration(db, dl, min_t=0.25):
    """Time (s) for a min-jerk move that respects both motors' speed caps.
    Min-jerk peak speed is 1.875x the mean, so divide that out."""
    tb = abs(db) * 1.875 / BASE_VMAX
    tl = abs(dl) * 1.875 / LIFT_VMAX
    return max(min_t, tb, tl)


def sample_joint_move(start, end, dt=0.04):
    """Yield (t, base_motor, lift_motor) along a synchronised min-jerk move so
    both joints start and finish together. start/end = (base, lift) motor deg."""
    db, dl = end[0] - start[0], end[1] - start[1]
    T = move_duration(db, dl)
    n = max(1, int(T / dt + 0.5))
    for i in range(1, n + 1):
        s = min_jerk(i / n)
        yield (i * T / n, start[0] + db * s, start[1] + dl * s)


def sample_line(p0, p1, dt=0.04):
    """Straight-line TOOL-SPACE move from p0 to p1 (each (x, y, z)): the line is
    sampled, every sample goes through IK, so the tip follows it as closely as the
    2-joint arm allows. Yields (t, base_motor, lift_motor, miss_cm)."""
    b0, l0, _ = inverse(*p0)
    b1, l1, _ = inverse(*p1)
    T = move_duration(b1 - b0, l1 - l0)
    n = max(1, int(T / dt + 0.5))
    for i in range(1, n + 1):
        s = min_jerk(i / n)
        p = (p0[0] + (p1[0] - p0[0]) * s,
             p0[1] + (p1[1] - p0[1]) * s,
             p0[2] + (p1[2] - p0[2]) * s)
        b, l, miss = inverse(*p)
        yield (i * T / n, b, l, miss)


def plan_safe(start, end):
    """Order the joint moves so the base never swings with the lift low.

    start/end = (base, lift) motor deg. Returns a list of (base, lift) waypoints
    to travel through with sample_joint_move: raise -> swing -> lower if the yaw
    changes noticeably and either end is below LIFT_TRAVEL, else a direct move."""
    swing = abs(motor_to_yaw(end[0]) - motor_to_yaw(start[0])) > YAW_SWING_DEG
    if not swing or (start[1] <= LIFT_TRAVEL and end[1] <= LIFT_TRAVEL):
        return [end]
    up = min(start[1], end[1], LIFT_TRAVEL)
    return [(start[0], up), (end[0], up), end]
