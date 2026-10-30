#!/usr/bin/env pybricks-micropython
"""
LYRA arm -- standalone ROUTINE PLAYER (presets + recorded gesture sessions).

Runs ON the brick with NO computer attached: pick a routine with the brick
buttons, press Center, and it plays then parks. Deploy once over USB, unplug,
and it's a self-contained demo -- which fits the "USB only while deploying"
constraint perfectly.

A routine is a list of (command, delay_ms) where delay is waited BEFORE the
command. Commands are the SAME vocabulary as ev3/lyra_arm.py:
    BASE+  BASE-  LIFT+  LIFT-  GRIP  RAISE  PARK

Two sources of routines:
  * PRESETS below -- hand-written choreography.
  * Recorded gesture sessions -- capture a live run with the bridge:
        python3 bridge/lyra_to_ev3.py --serial --record myrun.lyra
    then copy myrun.lyra next to this script on the brick; it shows up in the
    menu prefixed with '*'. (A .lyra file is just "COMMAND DELAY_MS" lines.)
    That is the "record gestures, replay as a preset" workflow.

Motion model MIRRORS ev3/lyra_arm.py (base + = right / - = left; lift - = up and
sags to 0; lift auto-raised to clearance before the base swings; firm grip;
park returns to start with the gripper open). Keep the two in sync if you retune.

!! Written right after live gesture control worked, but NOT yet run on hardware
   (the rig was powered down). Sanity-check the very first play, and tune the
   PRESETS angles/delays to your build.

Brick buttons:  Up/Down = choose routine   Center = play   (Back = stop program)
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait

ev3 = EV3Brick()

# --- tuned constants (mirror ev3/lyra_arm.py) ---
BASE_SPEED = 150
LIFT_SPEED = 130
GRIP_SPEED = 200
BASE_STEP  = 60
LIFT_STEP  = 45
BASE_MIN, BASE_MAX = -360, 270
LIFT_TOP, LIFT_BOTTOM, LIFT_TRAVEL = -180, 0, -160
GRIP_DUTY = 75

base = Motor(Port.A)
lift = Motor(Port.B)
grip = Motor(Port.C)
base.reset_angle(0)
lift.reset_angle(0)
base_pos = 0
lift_pos = 0
grip_open_state = True


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def ensure_clear():
    global lift_pos
    if lift_pos > LIFT_TRAVEL:            # lower than clearance -> raise first
        lift_pos = LIFT_TRAVEL
        lift.run_target(LIFT_SPEED, lift_pos)


def park():
    global base_pos, lift_pos, grip_open_state
    ensure_clear()
    base_pos = 0
    base.run_target(BASE_SPEED, 0)
    grip.run_until_stalled(-GRIP_SPEED, then=Stop.COAST, duty_limit=GRIP_DUTY)
    grip_open_state = True
    lift_pos = LIFT_BOTTOM
    lift.run_target(LIFT_SPEED, 0)
    base.stop(); lift.stop(); grip.stop()


def do(cmd):
    """Execute one command from the shared vocabulary."""
    global base_pos, lift_pos, grip_open_state
    if cmd == "BASE+":
        ensure_clear(); base_pos = clamp(base_pos + BASE_STEP, BASE_MIN, BASE_MAX)
        base.run_target(BASE_SPEED, base_pos, wait=False)
    elif cmd == "BASE-":
        ensure_clear(); base_pos = clamp(base_pos - BASE_STEP, BASE_MIN, BASE_MAX)
        base.run_target(BASE_SPEED, base_pos, wait=False)
    elif cmd == "LIFT+":
        lift_pos = clamp(lift_pos - LIFT_STEP, LIFT_TOP, LIFT_BOTTOM)
        lift.run_target(LIFT_SPEED, lift_pos, wait=False)
    elif cmd == "LIFT-":
        lift_pos = clamp(lift_pos + LIFT_STEP, LIFT_TOP, LIFT_BOTTOM)
        lift.run_target(LIFT_SPEED, lift_pos, wait=False)
    elif cmd == "GRIP":
        if grip_open_state:
            grip.run_until_stalled(GRIP_SPEED, then=Stop.HOLD, duty_limit=GRIP_DUTY)
            grip_open_state = False
        else:
            grip.run_until_stalled(-GRIP_SPEED, then=Stop.HOLD, duty_limit=GRIP_DUTY)
            grip_open_state = True
    elif cmd == "RAISE":
        ensure_clear()
    elif cmd == "PARK":
        park()


# --- preset routines: (command, delay_ms_before) ---
PRESETS = {
    "Wave": [
        ("RAISE", 0),
        ("BASE+", 700), ("BASE+", 700), ("BASE+", 700),
        ("BASE-", 700), ("BASE-", 700), ("BASE-", 700),
        ("BASE+", 700), ("BASE+", 700), ("BASE+", 700),
    ],
    "Pick L>R": [
        ("RAISE", 0),
        ("BASE-", 600), ("BASE-", 600), ("BASE-", 600), ("BASE-", 800),  # swing left
        ("LIFT-", 400), ("LIFT-", 400), ("LIFT-", 600),                  # lower onto object
        ("GRIP", 700),                                                   # close on it
        ("LIFT+", 500), ("LIFT+", 500),                                  # lift it clear
        ("BASE+", 600), ("BASE+", 600), ("BASE+", 600), ("BASE+", 600),
        ("BASE+", 600), ("BASE+", 600), ("BASE+", 600), ("BASE+", 800),  # swing right
        ("LIFT-", 400), ("LIFT-", 400), ("LIFT-", 600),                  # lower
        ("GRIP", 700),                                                   # release
    ],
}


def load_recorded():
    """Load *.lyra files (lines 'CMD DELAY_MS') sitting next to this script."""
    import os
    out = {}
    try:
        here = "/".join(__file__.split("/")[:-1]) or "."
    except Exception:
        here = "."
    try:
        for fn in os.listdir(here):
            if fn.endswith(".lyra"):
                steps = []
                with open(here + "/" + fn) as fh:
                    for line in fh:
                        p = line.split()
                        if len(p) >= 2:
                            steps.append((p[0].upper(), int(p[1])))
                        elif len(p) == 1 and p[0]:
                            steps.append((p[0].upper(), 0))
                out[fn[:-5]] = steps
    except Exception:
        pass
    return out


def play(name, steps):
    ev3.screen.clear()
    ev3.screen.print("Playing:")
    ev3.screen.print(" " + name)
    for cmd, delay in steps:
        wait(delay)
        do(cmd)
    park()                                # always finish at the start position
    ev3.speaker.beep()
    ev3.screen.print("done")


# --- brick-button menu ---
routines = [(k, PRESETS[k]) for k in sorted(PRESETS)]
for k, v in sorted(load_recorded().items()):
    routines.append(("*" + k, v))         # '*' marks a recorded routine

idx = 0
def show():
    ev3.screen.clear()
    ev3.screen.print("LYRA routines:")
    for i, (n, _) in enumerate(routines):
        ev3.screen.print(("> " if i == idx else "  ") + n)
    ev3.screen.print("Up/Dn pick  C=play")

if not routines:
    ev3.screen.clear()
    ev3.screen.print("No routines found")
else:
    show()
    while True:
        while not ev3.buttons.pressed():
            wait(20)
        b = ev3.buttons.pressed()
        if Button.CENTER in b:
            play(*routines[idx]); show()
        elif Button.UP in b:
            idx = (idx - 1) % len(routines); show()
        elif Button.DOWN in b:
            idx = (idx + 1) % len(routines); show()
        while ev3.buttons.pressed():       # debounce: wait for release
            wait(20)
