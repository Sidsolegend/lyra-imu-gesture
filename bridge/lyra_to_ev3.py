#!/usr/bin/env python3
"""
LYRA wrist -> EV3 arm bridge (hardened duplicate of bridge/lyra_to_ev3.py).

Same architecture and command vocabulary as the original:

    wrist (ESP32) --serial or BLE--> THIS SCRIPT --stdin over SSH--> EV3 lyra_arm.py

Hardening changes vs the original (each cites the original file:line):

  B1  EV3 stdout is drained in a background thread and the link WAITS for the
      controller's "READY" before accepting gestures. The original opened
      stdout=PIPE and never read it (bridge/lyra_to_ev3.py:63) — a latent
      pipe-full deadlock — and Popen returning proved nothing had launched, so
      commands could be silently dropped forever.
  B2  Graceful shutdown: close stdin (which triggers the on-brick park()),
      then WAIT for the remote to finish before terminate(). The original
      terminated immediately after closing stdin (:79-82), racing the park.
  B3  Heartbeat: periodic PING with a PONG freshness check, so a dead SSH hop
      is reported instead of silently eating commands.
  B4  Serial unplug detection + reconnect with backoff. The original spun
      silently forever on empty readline() after an unplug (:161-169).
  B5  BLE re-scan/reconnect loop with backoff (original made one attempt and
      exited, :189-212), and the reassembly buffer is bounded (original
      buf["s"] grew without limit if no newline arrived, :198-202).
  B6  Hosts come from --args OR the LYRA_PI_HOST / LYRA_EV3_TARGET environment
      variables; no personal username@IP baked into source (original :228-229,
      whose default EV3 IP was a link-local address documented to change every
      session — EV3.md:72). Also fixes the original's garbled --pi-host help
      string (:228, the "" was swallowed by string-literal concatenation).

Run:
    export LYRA_PI_HOST="pi_user@pi_address"
    export LYRA_EV3_TARGET="robot@ev3_address"
    python3 lyra_to_ev3.py --serial --dry-run     # confirm gestures arrive
    python3 lyra_to_ev3.py --serial               # full pipeline to the EV3
"""
import argparse
import glob
import os
import subprocess
import threading
import time

# Gesture event (from the wrist firmware) -> EV3 command (to lyra_arm.py).
MAPPING = {
    "FLICK,R":   "BASE+",   # flick right  -> base rotates right
    "FLICK,L":   "BASE-",   # flick left   -> base rotates left
    "TWIST,CW":  "LIFT+",   # twist CW     -> lift up
    "TWIST,CCW": "LIFT-",   # twist CCW    -> lift down
    "PINCH":     "GRIP",    # pinch        -> open/close gripper
}

SERIAL_BAUD = 115200
# T14: LYRA-ARM now advertises a DEDICATED service UUID, distinct from the
# calibration build's LYRA-CAL (6E400001.../...0003), so a phone/host can never
# cross-connect the two once both exist. The ARM base is 6E4001xx.
TX_UUID = "6e400103-b5a3-f393-e0a9-e50e24dcca9e"  # BLE LYRA-ARM TX (notify)
TX_UUID_LEGACY = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # pre-T14 shared UUID
READY_TIMEOUT_S = 20.0      # how long to wait for lyra_arm.py to say READY
PING_PERIOD_S = 10.0        # heartbeat interval (B3)
MAX_BLE_BUF = 4096          # bound on the notification reassembly buffer (B5)

# E7 / T13: the firmware emits a STRUCTURED serial event line prefixed with this
# token, so the bridge no longer parses the human debug prefix ("->", which was
# log cosmetics being load-bearing). Old firmware that still emits "-> EVENT" is
# accepted as a fallback for backward compatibility.
EVENT_PREFIX = "EVT "           # new structured prefix (machine-facing)
EVENT_PREFIX_LEGACY = "->"      # pre-E7 debug prefix (backward compat)


def parse_event_line(line):
    """Extract a gesture-event string from one serial line, or None if the line
    is firmware debug/boot chatter.

    Recognizes, in order:
      * the E7 structured prefix   "EVT FLICK,R"   -> "FLICK,R"
      * the legacy debug prefix    "-> FLICK,R"    -> "FLICK,R"  (old firmware)
    Anything else (comments, "# ...", boot banners) returns None so the caller
    can print it as [fw] output without treating it as a command.
    """
    if line is None:
        return None
    s = line.strip()
    if not s:
        return None
    if s.startswith(EVENT_PREFIX):
        return s[len(EVENT_PREFIX):].strip() or None
    if s.startswith(EVENT_PREFIX_LEGACY):
        return s[len(EVENT_PREFIX_LEGACY):].strip() or None
    return None


class Ev3Link:
    """Pipes command lines into lyra_arm.py on the brick, via the Pi hub.

        Mac -> ssh pi_host -> ssh ev3_target -> brickrun ... lyra_arm.py (stdin)

    Set pi_host='' to ssh the brick directly (if ever reachable)."""

    def __init__(self, pi_host, ev3_target, remote_script, dry_run):
        self.dry_run = dry_run
        self.proc = None
        self.ready = threading.Event()
        self.last_pong = time.monotonic()
        self._last_ping = 0.0
        if dry_run:
            return
        run = "brickrun -r -- pybricks-micropython " + remote_script
        if pi_host:
            inner = 'ssh %s "%s"' % (ev3_target, run)
            cmd = ["ssh", pi_host, inner]
            print("EV3: launching lyra_arm.py on", ev3_target, "via", pi_host)
        else:
            cmd = ["ssh", ev3_target, run]
            print("EV3: launching lyra_arm.py on", ev3_target)
        self.proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, bufsize=1, text=True)
        # B1: drain remote output forever (prevents pipe-full deadlock) and
        # watch for READY / PONG.
        threading.Thread(target=self._drain, daemon=True).start()
        if self.ready.wait(READY_TIMEOUT_S):
            print("EV3: controller is READY.")
        else:
            print("EV3: WARNING -- no READY within %.0fs. Check SSH reachability"
                  " (is the EV3 address current?). Commands may be dropped."
                  % READY_TIMEOUT_S)

    def _drain(self):
        for line in self.proc.stdout:           # ends when the remote exits
            line = line.strip()
            if not line:
                continue
            if line == "READY":
                self.ready.set()
            if line == "PONG":
                self.last_pong = time.monotonic()
            print("  [ev3]", line)
        print("  [ev3] remote controller exited.")

    def alive(self):
        return self.dry_run or (self.proc is not None and self.proc.poll() is None)

    def heartbeat(self):
        """B3: call periodically from the event loop."""
        if self.dry_run or not self.proc:
            return
        now = time.monotonic()
        if now - self._last_ping >= PING_PERIOD_S:
            self._last_ping = now
            self._write("PING")
            if now - self.last_pong > 3 * PING_PERIOD_S:
                print("EV3: WARNING -- no PONG for %.0fs; link looks dead."
                      % (now - self.last_pong))

    def _write(self, command):
        try:
            self.proc.stdin.write(command + "\n")
            self.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError) as e:
            print("   EV3 link error:", e)
            return False

    def send(self, command):
        print("   -> EV3:", command)
        if self.dry_run or not self.proc:
            return
        if not self.alive():
            print("   EV3: link is down -- command dropped (restart the bridge)")
            return
        self._write(command)

    def close(self):
        """B2: closing stdin makes lyra_arm.py park(); WAIT for that before
        tearing the SSH channel down."""
        if not self.proc:
            return
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.wait(timeout=10)          # let park() finish
        except subprocess.TimeoutExpired:
            print("EV3: remote did not exit after stdin close; terminating.")
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class Recorder:
    """Logs each command as 'CMD DELAY_MS' lines — a .lyra routine that
    ev3/lyra_routines.py can replay standalone."""

    def __init__(self, path):
        self.fh = open(path, "w") if path else None
        self.last = None
        if self.fh:
            print("Recording routine ->", path)

    def log(self, cmd):
        if not self.fh:
            return
        now = time.monotonic()
        gap = 0 if self.last is None else int((now - self.last) * 1000)
        self.last = now
        self.fh.write("%s %d\n" % (cmd, gap))
        self.fh.flush()

    def close(self):
        if self.fh:
            self.fh.close()


def on_event(ev, link, recorder=None):
    """Shared handler: one gesture event string -> maybe an EV3 command."""
    if not ev:
        return
    print("gesture:", ev)
    if ev in ("ARMED", "DISARMED"):
        return                      # status only; movement is gated on-device
    cmd = MAPPING.get(ev)
    if cmd:
        link.send(cmd)
        if recorder:
            recorder.log(cmd)
    elif ev.startswith("CUSTOM,"):
        print("   (custom gesture, no mapping yet:", ev, ")")


# ---------------------------------------------------------------------------
# Serial mode (default)
# ---------------------------------------------------------------------------
def autodetect_port():
    cands = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.SLAB_USBtoUART*") \
        + glob.glob("/dev/cu.wchusbserial*") + glob.glob("/dev/ttyUSB*") \
        + glob.glob("/dev/ttyACM*")
    return cands[0] if cands else None


def open_serial(port):
    import serial  # pyserial
    # Open WITHOUT asserting DTR/RTS: on an ESP32 those drive the EN/boot
    # lines, and pyserial's default (both asserted) holds the board in a
    # reset loop.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = SERIAL_BAUD
    ser.timeout = 1
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def run_serial(args):
    import serial  # pyserial

    link = Ev3Link(args.pi_host, args.ev3_target, args.remote_script, args.dry_run)
    recorder = Recorder(args.record)
    explicit = args.serial if isinstance(args.serial, str) else None
    backoff = 1.0
    ser = None
    try:
        while True:
            # B4: (re)open the port, with backoff, surviving unplug/replug.
            port = explicit or autodetect_port()
            if not port:
                print("No serial port found; retrying in %.0fs "
                      "(plug in the ESP32, or pass --serial PORT)" % backoff)
                time.sleep(backoff)
                backoff = min(backoff * 2, 15.0)
                continue
            try:
                ser = open_serial(port)
            except serial.SerialException as e:
                print("Could not open %s (%s); retrying in %.0fs" % (port, e, backoff))
                time.sleep(backoff)
                backoff = min(backoff * 2, 15.0)
                continue
            backoff = 1.0
            print("Opened serial", port, "@", SERIAL_BAUD)
            print("Listening. Double-flick to ARM the wrist, then gesture. Ctrl-C to quit.")
            try:
                while True:
                    raw = ser.readline()
                    link.heartbeat()
                    if not raw:
                        # timeout tick; detect a vanished device (unplug often
                        # keeps the fd but the node disappears)
                        if not os.path.exists(port):
                            raise serial.SerialException("port vanished")
                        continue
                    line = raw.decode("utf-8", "ignore").strip()
                    ev = parse_event_line(line)   # E7: structured "EVT ..." (or legacy "->")
                    if ev is not None:
                        on_event(ev, link, recorder)
                    elif line:                    # firmware debug / boot output
                        print("  [fw]", line)
            except (serial.SerialException, OSError) as e:
                print("Serial lost (%s) -- will reconnect." % e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        recorder.close()
        link.close()
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# BLE mode (--ble)
# ---------------------------------------------------------------------------
def run_ble(args):
    import asyncio
    from bleak import BleakScanner, BleakClient

    async def session(link, recorder):
        """One scan+connect+listen session. Returns when the link drops."""
        print("Scanning for '%s' ..." % args.name)
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: (d.name or "") == args.name, timeout=15.0)
        if device is None:
            print("Could not find '%s'. Is the wristband advertising?" % args.name)
            return False
        print("Found", device.address, "- connecting...")
        buf = {"s": ""}

        def handle(_, data):
            buf["s"] += data.decode("utf-8", "ignore")
            while "\n" in buf["s"]:
                ln, buf["s"] = buf["s"].split("\n", 1)
                on_event(ln.strip(), link, recorder)
            if len(buf["s"]) > MAX_BLE_BUF:       # B5: bound the buffer
                print("  (dropping %d unframed BLE bytes)" % len(buf["s"]))
                buf["s"] = ""

        async with BleakClient(device) as client:
            try:                                   # T14: dedicated LYRA-ARM UUID
                await client.start_notify(TX_UUID, handle)
            except Exception:                      # fall back to the pre-T14 shared UUID
                print("  (TX %s not found; trying legacy UUID)" % TX_UUID)
                await client.start_notify(TX_UUID_LEGACY, handle)
            print("Connected. Double-flick to ARM, then gesture. Ctrl-C to quit.")
            while client.is_connected:
                link.heartbeat()
                await asyncio.sleep(1.0)
        print("BLE link dropped.")
        return True

    async def go():
        link = Ev3Link(args.pi_host, args.ev3_target, args.remote_script, args.dry_run)
        recorder = Recorder(args.record)
        backoff = 2.0
        try:
            while True:                            # B5: reconnect forever
                ok = await session(link, recorder)
                backoff = 2.0 if ok else min(backoff * 2, 30.0)
                print("Re-scanning in %.0fs ..." % backoff)
                await asyncio.sleep(backoff)
        finally:
            recorder.close()
            link.close()

    try:
        asyncio.run(go())
    except KeyboardInterrupt:
        print("\nbye")


def main():
    p = argparse.ArgumentParser(description="Bridge LYRA wrist gestures to an EV3 arm.")
    p.add_argument("--serial", nargs="?", const=True, default=None,
                   metavar="PORT",
                   help="read over USB serial (optionally give the port; "
                        "auto-detects if omitted)")
    p.add_argument("--ble", action="store_true", help="read over BLE instead of serial")
    p.add_argument("--name", default="LYRA-ARM", help="BLE name (--ble mode)")
    # B6: no personal hosts in source; use flags or environment variables.
    p.add_argument("--pi-host", default=os.environ.get("LYRA_PI_HOST", ""),
                   help="SSH target for the Pi hub, e.g. user@pi.local "
                        "(empty = ssh the brick directly). Env: LYRA_PI_HOST")
    p.add_argument("--ev3-target", default=os.environ.get("LYRA_EV3_TARGET", ""),
                   help="SSH target for the EV3 brick, reachable from the Pi. "
                        "Env: LYRA_EV3_TARGET")
    p.add_argument("--remote-script", default="lyra_arm.py",
                   help="lyra_arm.py path on the brick")
    p.add_argument("--record", default=None, metavar="FILE.lyra",
                   help="also save the session as a replayable routine "
                        "(for lyra_routines.py)")
    p.add_argument("--dry-run", action="store_true",
                   help="print commands instead of sending them to the EV3")
    args = p.parse_args()

    if not args.dry_run and not (args.pi_host or args.ev3_target):
        p.error("no EV3 target: set --ev3-target/--pi-host or the "
                "LYRA_EV3_TARGET/LYRA_PI_HOST environment variables "
                "(or use --dry-run)")

    if args.ble:
        run_ble(args)
    else:
        if args.serial is None:       # default to serial auto-detect
            args.serial = True
        run_serial(args)


if __name__ == "__main__":
    main()
