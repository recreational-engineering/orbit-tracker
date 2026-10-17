#!/usr/bin/env python3
"""Pilot-view flash check: restart the board (cold tile cache), enter the pilot view several
times (with look-around pans), then stay in it idle. Counts panel refreshes that came late
(the panel then shows a broken, white frame), DSI underruns, tile decodes and one-frame
jumps from the flash monitor.

    python3 tools/pilot_flash_test.py [--idle-seconds 180] [--cycles 6]
"""
import argparse
import re
import time

from devserial import open_console


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--idle-seconds", type=float, default=180)
    ap.add_argument("--cycles", type=int, default=6)
    args = ap.parse_args()
    s = open_console(timeout=0.05, reset=True)
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
    cmd("F", 0.2)
    for i in range(args.cycles):
        cmd("K 360 655", 1.0)
        cmd("K 360 254", 3.0)                      # Earth spin
        cmd("K 360 655", 1.0)
        cmd("K 360 437", 6.0)                      # pilot: tiles load
        for d in ("G 360 450 360 250 800", "G 200 360 520 360 800", "G 520 360 200 360 800"):
            cmd(d, 2.5)
        cmd("K 360 60", 4.0)                       # reset the look direction
        print(f"cycle {i + 1} done", flush=True)
    pump(args.idle_seconds)
    cmd("F", 1.5)
    text = "".join(log)
    late = sum(int(m) for m in re.findall(r"late refreshes (\d+)", text))
    decodes = sum(int(m) for m in re.findall(r"tile decodes (\d+)", text))
    worst = max([float(m) for m in re.findall(r"refresh max ([\d.]+) ms", text)] or [0])
    print(f"late panel refreshes: {late} (longest refresh {worst} ms)")
    print(f"DSI underruns: {text.count('underrun')}")
    print(f"tile decodes: {decodes}, JPEG errors: {text.count('jpeg.decoder')}")
    for line in text.splitlines():
        if "FLASH?" in line or "flash monitor" in line:
            print(line[line.find("FLASH?") if "FLASH?" in line else line.find("flash monitor"):])


if __name__ == "__main__":
    main()
