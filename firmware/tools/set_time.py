#!/usr/bin/env python3
"""Set the board's clock from this computer (the board has no network time yet).

    python3 tools/set_time.py [--port /dev/cu.usbmodem...]
"""
import argparse
import time

from devserial import open_console

ap = argparse.ArgumentParser()
ap.add_argument("--port")
args = ap.parse_args()

with open_console(args.port) as s:
    s.reset_input_buffer()
    s.write(f"T{time.time():.3f}\n".encode())
    deadline = time.time() + 3
    while time.time() < deadline:
        line = s.readline().decode(errors="replace").strip()
        if "TIME set" in line:
            print(line)
            break
    else:
        raise SystemExit("no confirmation from the board")
