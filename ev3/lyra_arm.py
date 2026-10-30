#!/usr/bin/env pybricks-micropython
"""
LYRA -> EV3 arm controller (Pybricks / EV3 MicroPython).

Drives a 3-motor arm from one text command per line on stdin. It is meant to be
launched by the Mac bridge over SSH:

    ssh robot@ev3dev.local "brickrun -r -- pybricks-micropython ~/lyra_arm.py"

...but it also runs perfectly standalone for bring-up: on launch it homes the
base (if a touch sensor is wired) and jogs every motor once as a wiring
self-test, then waits for commands. So just running this file is enough to
prove the arm is wired correctly -- before any wrist/Bluetooth is involved.

WIRING (must match the ports below):
  Port A   Large motor    base rotate (swivel left/right)
  Port B   Large motor    shoulder / lift (raise/lower)
  Port C   Medium motor   gripper / claw
  Port S1  Touch sensor   base home switch        (optional)
  Port S3  Color sensor   unused in v1            (optional)

COMMANDS (newline-terminated, case-insensitive):
  BASE+ / BASE-   step base right / left
  LIFT+ / LIFT-   step lift up / down
  GRIP            toggle gripper open <-> close
  HOME            re-run base homing
  STOP            hold all motors in place
  TEST            run the wiring self-test again
  PING            reply 'PONG' on stdout

CARTESIAN COMMANDS (inverse kinematics, see ev3/lyra_kin.py -- deploy it next to
this file; geometry constants there must be calibrated, see EV3.md):
  GOTO x y z      move the tip to (x,y,z) cm, joints synchronised, min-jerk smooth,
                  auto raise->swing->lower so the base never swings with the lift low
  LINE x y z      like GOTO but the tip follows a straight tool-space line (IK per tick)
  JOG X+ X- Y+ Y- Z+ Z-   nudge the tip 2 cm along a world axis
  POS             print tip (x,y,z) and the joint angles
  WP SAVE name    remember the current tip position as a waypoint
  WP GO name      GOTO a saved waypoint
  SEQ a b c ...   visit saved waypoints in order, then stop
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import sys
import math

try:
    import lyra_kin as kin
except ImportError:                       # kinematics file not deployed: old commands only
    kin = None

ev3 = EV3Brick()

# ----------------------------------------------------------------------------
# Tuning -- all the numbers you'll want to play with live here.
# ----------------------------------------------------------------------------
BASE_SPEED = 150          # deg/s for base moves
LIFT_SPEED = 130          # deg/s for lift moves
GRIP_SPEED = 200          # deg/s for the gripper

BASE_STEP  = 60           # base motor-deg per step (base is high-geared)
LIFT_STEP  = 45           # lift motor-deg per step

# Base: + = right, - = left (confirmed on hardware). Lots of range; left has far
# more than right. Generous limits, relative to wherever the base sits at launch
# (there's no home switch). Tighten if the base's cables ever start to tug.
BASE_MIN, BASE_MAX = -360, 270

# Lift: - = UP, 0 = bottom. It SAGS to the bottom whenever a motor is released,
# so the program must stay running to keep holding it. -180 is the physical top.
# The base only clears obstacles with the lift raised, so base moves auto-raise
# the lift to LIFT_TRAVEL first.
LIFT_TOP    = -180        # most-up (physical max)
LIFT_BOTTOM = 0           # rest / sagged-down
LIFT_TRAVEL = -160        # clearance the lift must reach before the base swings

GRIP_DUTY  = 75           # gripper squeeze torque (%) before "closed" (firmer grip)

# Homing: the touch sensor on S1 IS a wired base home switch. home() searches for
# it (one way, then the other, both BOUNDED so it can't run into an end-stop) and
# calls that angle 0 -> absolute, repeatable base positioning (no hand-centering).
# Kept opt-in because an untested home can over-rotate: run the HOME command first
# to confirm it finds the switch, flip the sign of HOME_SPEED if it searches the
# wrong way, then set AUTO_HOME = True to home on every launch.
AUTO_HOME      = False
HOME_SPEED     = 80       # deg/s while searching for the switch (gentle)
HOME_SEARCH_MS = 1500     # bounded search time per direction


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ----------------------------------------------------------------------------
# Hardware. Motors are required; sensors are optional (wrap so a missing
# sensor never crashes the controller).
# ----------------------------------------------------------------------------
base = Motor(Port.A)
lift = Motor(Port.B)
grip = Motor(Port.C)

try:
    touch = TouchSensor(Port.S1)
except Exception:
    touch = None

try:
    color = ColorSensor(Port.S3)
except Exception:
    color = None

base_target = 0
lift_target = 0
grip_open = True


# ----------------------------------------------------------------------------
# Actions
# ----------------------------------------------------------------------------
def home():
    """Find the base home switch (touch on S1) and call it zero -> absolute,
    repeatable base positioning. Searches one way then the other, each bounded by
    HOME_SEARCH_MS so it can't drive into an end-stop. Falls back to 'current = 0'
    if the switch isn't found (flip HOME_SPEED's sign if it never finds it)."""
    global base_target
    if touch is None:
        base.reset_angle(0); base_target = 0
        ev3.screen.print("No touch: skip home")
        return
    ev3.screen.print("Homing...")
    if not touch.pressed():
        base.run(HOME_SPEED)                       # search one way
        ms = 0
        while not touch.pressed() and ms < HOME_SEARCH_MS:
            wait(10); ms += 10
        if not touch.pressed():
            base.run(-HOME_SPEED)                  # reverse, search the other way
            ms = 0
            while not touch.pressed() and ms < 2 * HOME_SEARCH_MS:
                wait(10); ms += 10
    base.stop()
    base.reset_angle(0)
    base_target = 0
    ev3.screen.print("Homed." if touch.pressed() else "Home not found")


def ensure_lift_clear():
    """Raise the lift to travel-clearance height (and hold) so the base can
    swing without colliding. No-op if it's already at/above clearance."""
    global lift_target
    if lift_target > LIFT_TRAVEL:          # currently lower than clearance
        lift_target = LIFT_TRAVEL
        lift.run_target(LIFT_SPEED, lift_target)   # wait until fully raised


def base_step(delta):
    global base_target
    ensure_lift_clear()                    # never swing the base with the lift low
    base_target = clamp(base_target + delta, BASE_MIN, BASE_MAX)
    base.run_target(BASE_SPEED, base_target, wait=False)


def lift_up():
    global lift_target
    lift_target = clamp(lift_target - LIFT_STEP, LIFT_TOP, LIFT_BOTTOM)
    lift.run_target(LIFT_SPEED, lift_target, wait=False)


def lift_down():
    global lift_target
    lift_target = clamp(lift_target + LIFT_STEP, LIFT_TOP, LIFT_BOTTOM)
    lift.run_target(LIFT_SPEED, lift_target, wait=False)


def grip_toggle():
    global grip_open
    if grip_open:
        grip.run_until_stalled(GRIP_SPEED, then=Stop.HOLD, duty_limit=GRIP_DUTY)
        grip_open = False
    else:
        grip.run_until_stalled(-GRIP_SPEED, then=Stop.HOLD, duty_limit=GRIP_DUTY)
        grip_open = True


def park():
    """Return to the launch 'zero': base centered, lift lowered, then release.
    Runs before the program stops so the arm always ends where it started."""
    global base_target, lift_target, grip_open
    ensure_lift_clear()                          # lift up so the base can swing back
    base_target = 0
    base.run_target(BASE_SPEED, 0)               # base back to start (wait)
    grip.run_until_stalled(-GRIP_SPEED, then=Stop.COAST, duty_limit=GRIP_DUTY)
    grip_open = True                             # always leave the gripper OPEN
    lift_target = LIFT_BOTTOM
    lift.run_target(LIFT_SPEED, LIFT_BOTTOM)      # lower the lift gently to its start
    base.stop(); lift.stop(); grip.stop()        # release all motors


def selftest():
    """Gently twitch every motor so you can confirm each port + direction.
    Angles are deliberately small/slow so a tightly-geared joint can't slam
    into its end-stop. Bump them up once you know each joint's safe range."""
    ev3.screen.clear()
    ev3.screen.print("Self-test:")
    for name, m, ang in (("A base", base, 15), ("B lift", lift, 15),
                         ("C grip", grip, 20)):
        ev3.screen.print("  " + name)
        m.run_angle(80, ang)
        wait(250)
        m.run_angle(80, -ang)
        wait(250)
    ev3.speaker.beep()
    ev3.screen.print("Self-test done")


# ----------------------------------------------------------------------------
# Cartesian layer (inverse kinematics). Streams a time-parameterised trajectory
# so both joints move together instead of one-after-the-other.
# ----------------------------------------------------------------------------
JOG_CM = 2.0
waypoints = {}


def tip():
    return kin.forward(base.angle(), lift.angle())


def _track(b, l, dt_ms):
    """Command both motors toward (b, l) for the next tick. Uses track_target when
    this Pybricks build has it (true streaming); otherwise re-issues run_target with
    the speed needed to arrive within one tick."""
    global base_target, lift_target
    base_target, lift_target = b, l
    if hasattr(base, "track_target"):
        base.track_target(b); lift.track_target(l)
    else:
        sec = dt_ms / 1000.0
        base.run_target(min(kin.BASE_VMAX, max(20, abs(b - base.angle()) / sec)), b, wait=False)
        lift.run_target(min(kin.LIFT_VMAX, max(20, abs(l - lift.angle()) / sec)), l, wait=False)


def _stream(samples):
    t_prev = 0.0
    for sample in samples:
        t, b, l = sample[0], sample[1], sample[2]
        dt = int((t - t_prev) * 1000)
        _track(b, l, dt)
        wait(dt)
        t_prev = t


def goto(x, y, z):
    b, l, miss = kin.inverse(x, y, z)
    if miss > 0.5:
        print("WARN unreachable by %.1f cm, going to nearest" % miss)
    here = (base.angle(), lift.angle())
    for wp in kin.plan_safe(here, (b, l)):
        _stream(kin.sample_joint_move((base.angle(), lift.angle()), wp))
    print("AT %.1f %.1f %.1f" % tip())


def line_to(x, y, z):
    # a straight tool-space line only makes sense without a swing detour; if the
    # lift is below clearance and yaw changes, fall back to the safe joint route
    b1, l1, _ = kin.inverse(x, y, z)
    if len(kin.plan_safe((base.angle(), lift.angle()), (b1, l1))) > 1:
        return goto(x, y, z)
    _stream(kin.sample_line(tip(), (x, y, z)))
    print("AT %.1f %.1f %.1f" % tip())


def jog(axis):
    x, y, z = tip()
    d = JOG_CM if axis[1] == "+" else -JOG_CM
    if   axis[0] == "X": x += d
    elif axis[0] == "Y": y += d
    elif axis[0] == "Z": z += d
    line_to(x, y, z)


def cart(cmd):
    """Handle the Cartesian commands. Returns False if cmd isn't one."""
    if kin is None:
        print("?no lyra_kin.py on brick")
        return True
    w = cmd.split()
    op = w[0]
    if op in ("GOTO", "LINE") and len(w) == 4:
        (goto if op == "GOTO" else line_to)(float(w[1]), float(w[2]), float(w[3]))
    elif op == "JOG" and len(w) == 2 and w[1][:1] in "XYZ" and w[1][1:] in ("+", "-"):
        jog(w[1])
    elif op == "POS":
        print("POS %.1f %.1f %.1f  base=%d lift=%d" % (tip() + (base.angle(), lift.angle())))
    elif op == "WP" and len(w) == 3 and w[1] == "SAVE":
        waypoints[w[2]] = tip()
        print("SAVED " + w[2])
    elif op == "WP" and len(w) == 3 and w[1] == "GO" and w[2] in waypoints:
        goto(*waypoints[w[2]])
    elif op == "SEQ":
        for name in w[1:]:
            if name in waypoints:
                goto(*waypoints[name])
    else:
        return False
    return True


def dispatch(cmd):
    if   cmd == "BASE+": base_step(+BASE_STEP)   # right
    elif cmd == "BASE-": base_step(-BASE_STEP)   # left
    elif cmd == "LIFT+": lift_up()               # up
    elif cmd == "LIFT-": lift_down()             # down
    elif cmd == "GRIP":  grip_toggle()
    elif cmd == "RAISE": ensure_lift_clear()     # lift to travel/clearance height
    elif cmd == "PARK":  park()                  # return to start position
    elif cmd == "HOME":  home()
    elif cmd == "STOP":  base.hold(); lift.hold(); grip.hold()
    elif cmd == "TEST":  selftest()
    elif cmd == "PING":  print("PONG")
    elif cmd == "":      pass
    elif cmd.split()[0] in ("GOTO", "LINE", "JOG", "POS", "WP", "SEQ") and cart(cmd):
        pass
    else:                print("?cmd " + cmd)


# ----------------------------------------------------------------------------
# Main: home, self-test, then loop on stdin.
# ----------------------------------------------------------------------------
ev3.screen.clear()
ev3.screen.print("LYRA arm ready")
ev3.speaker.beep()
# NOTE: the S1 touch sensor IS a wired base home switch. Auto-home is opt-in
# (AUTO_HOME, above): the original home() drove the base the WRONG way into its
# end-stop, so home() now does a bounded two-way search. Confirm it with the HOME
# command, then set AUTO_HOME = True. Until then we treat the launch position as
# zero. Auto-selftest stays off; run it on demand with the TEST command.
base.reset_angle(0)
lift.reset_angle(0)     # the lift sags to its bottom when released -> call it 0
if AUTO_HOME:           # opt-in: home the base off the S1 touch switch on launch
    home()
print("READY")          # the controller waits for this line

while True:
    line = sys.stdin.readline()
    if not line:        # stdin closed (controller disconnected)
        break
    dispatch(line.strip().upper())

park()                  # always return to the start position before stopping
