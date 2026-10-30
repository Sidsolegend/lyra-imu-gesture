#!/usr/bin/env python3
"""Send a teleprompter script to the LYRA glasses over USB serial.

  pip install pyserial
  python3 glasses_send_script.py /dev/cu.usbmodem1101 talk.txt

Script format (lyra_glasses.ino): "## Title" starts a slide, a blank line ends a cue.
Each cue must fit the screen (4 lines of 12 characters on the 0.42" OLED); the glasses
print a warning for every cue they had to cut, and this tool shows those warnings.
"""
import sys, time
import serial


def main(port, path):
    lines = open(path, encoding="utf-8").read().splitlines()
    if any(l.strip() == "END" for l in lines):
        sys.exit('a line reading just "END" would end the upload early: reword it')
    with serial.Serial(port, 115200, timeout=0.2) as s:
        time.sleep(0.5)
        s.reset_input_buffer()
        s.write(b"SCRIPT\n")
        time.sleep(0.2)
        for l in lines:
            s.write((l.rstrip() + "\n").encode("utf-8"))
            time.sleep(0.005)                       # LittleFS writes slower than USB
        s.write(b"END\n")
        t0 = time.time()
        while time.time() - t0 < 3:                 # echo the loader's report
            out = s.readline().decode("utf-8", "replace").strip()
            if out:
                print(out)
                if out.startswith("# script:"):
                    break


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: glasses_send_script.py <serial port> <script.txt>")
    main(sys.argv[1], sys.argv[2])
