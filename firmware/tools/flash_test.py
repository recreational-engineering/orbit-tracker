#!/usr/bin/env python3
"""Exercise every view, the menu, pans, taps and glides with the board's flash monitor on,
and report frames where part of the screen jumped for exactly one frame.

    python3 tools/flash_test.py
"""
import re
import time

from devserial import open_console

ROWS = {"spin": 254, "ground": 315, "satellite": 376, "pilot": 437, "flat": 499}
DRAGS = ["G 220 400 500 320 700", "G 500 320 260 450 500", "G 300 250 420 500 900", "G 430 480 280 260 400"]


def main():
    s = open_console(timeout=0.05)
    log = []

    def pump(sec):
        end = time.time() + sec
        while time.time() < end:
            d = s.read(65536)
            if d:
                log.append(d.decode(errors="replace"))

    def cmd(c, wait):
        s.write((c + "\n").encode())
        pump(wait)

    cmd(f"T{time.time():.3f}", 0.5)
    cmd("F", 0.2)                                  # monitor on
    import sys
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    no_pan = "--no-pan" in sys.argv   # skip drags and flings (fast moving objects trip the monitor)
    views = args[0].split(",") if args else list(ROWS)
    for view in views:
        cmd("K 360 655", 0.4)                      # menu open
        cmd("K 360 655", 0.4)                      # and closed again by the tab
        cmd("K 360 655", 0.8)                      # open
        cmd(f"K 360 {ROWS[view]}", 3.0)            # pick the view, glide into it
        for d in [] if no_pan else DRAGS:
            cmd(d, 1.0)
        pump(1.0)
        cmd("K 360 60", 1.5)                       # top tab: back to the default framing
        if not no_pan:
            cmd("G 200 360 520 360 300", 0.2)      # fling
        pump(1.5)
        cmd("K 360 360", 0.12)                     # double tap
        cmd("K 360 360", 5.0)                      # reset; wait out the glide back
        print(f"{view}: done")
    cmd("F", 1.0)                                  # monitor off: summary
    text = "".join(log)
    for line in text.splitlines():
        if "FLASH?" in line or "flash monitor" in line:
            print(line[line.find("FLASH?") if "FLASH?" in line else line.find("flash monitor"):])
    if not re.search(r"flash monitor", text):
        print("(no summary from the board)")


if __name__ == "__main__":
    main()
