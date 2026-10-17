#!/usr/bin/env python3
"""Frame-rate test over all views: for each view (picked through the on-screen menu),
the still frame rate, then while panning (simulated finger drags over the globe).

    python3 tools/perf_test.py [--views spin,ground,...] [--pan-seconds 4]
"""
import argparse
import re
import time

from devserial import open_console

# Menu rows (drawMenu() at 720 px): row centres for the five views.
ROWS = {"spin": 253.8, "ground": 315.0, "satellite": 376.2, "pilot": 437.4, "flat": 498.6}
DRAGS = [(220, 400, 500, 320), (500, 320, 260, 450), (300, 250, 420, 500), (430, 480, 280, 260)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--views", default="spin,ground,satellite,pilot,flat")
    ap.add_argument("--pan-seconds", type=float, default=4)
    ap.add_argument("--port")
    ap.add_argument("--time", type=float, help="fixed start time (unix s) for comparable runs")
    args = ap.parse_args()
    s = open_console(args.port, timeout=0.05)
    buf = []

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            data = s.read(65536)
            if data:
                buf.append(data.decode(errors="replace"))

    def cmd(c, wait=0.0):
        s.write((c + "\n").encode())
        pump(wait)

    def stats():
        buf.clear()
        cmd("P", 0.6)
        m = re.search(r"STATS .*", "".join(buf))
        return m.group(0) if m else "(no stats)"

    if not args.time:
        cmd(f"T{time.time():.3f}", 0.5)
    for view in args.views.split(","):
        if args.time:
            cmd(f"T{args.time:.3f}", 0.3)   # same scene for every run
        cmd("K 360 655", 1.2)                    # bottom tab: open the menu
        cmd(f"K 360 {ROWS[view]:.0f}", 2.5)      # pick the view, let it settle
        stats()
        pump(3)
        print(f"{view:9s} still : {stats()}")
        stats()
        end = time.time() + args.pan_seconds
        k = 0
        while time.time() < end:
            x0, y0, x1, y1 = DRAGS[k % len(DRAGS)]
            cmd(f"G {x0} {y0} {x1} {y1} 700", 0.75)
            k += 1
        print(f"{view:9s} pan   : {stats()}")
        pump(4.5)                                # glide back before the next view


if __name__ == "__main__":
    main()
