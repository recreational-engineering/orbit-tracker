#!/usr/bin/env python3
"""Grab the frame currently on the round display as a PNG (needs Pillow).

    python3 tools/screenshot.py out.png [--port /dev/cu.usbmodem...] [--round]
"""
import argparse
import time

import numpy as np
from PIL import Image, ImageDraw

from devserial import open_console

W = H = 720
MARK = b"@@FRAME 720 720 RGB565LE@@\n"

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--port")
ap.add_argument("--round", action="store_true", help="black out the corners the round panel does not show")
args = ap.parse_args()

with open_console(args.port, timeout=10) as s:
    s.reset_input_buffer()
    s.write(b"S\n")
    buf = b""
    deadline = time.time() + 10
    while MARK not in buf:
        buf += s.read(256)
        if time.time() > deadline:
            raise SystemExit("no frame header from the board")
    data = buf.split(MARK, 1)[1]
    while len(data) < W * H * 2:
        chunk = s.read(W * H * 2 - len(data))
        if not chunk:
            raise SystemExit(f"timed out after {len(data)} bytes")
        data += chunk

px = np.frombuffer(data[: W * H * 2], dtype="<u2").reshape(H, W)
r = ((px >> 11) & 31).astype(np.uint16) * 255 // 31
g = ((px >> 5) & 63).astype(np.uint16) * 255 // 63
b = (px & 31).astype(np.uint16) * 255 // 31
img = Image.fromarray(np.stack([r, g, b], -1).astype(np.uint8))
if args.round:
    m = Image.new("L", (W, H), 0)
    ImageDraw.Draw(m).ellipse((0, 0, W - 1, H - 1), fill=255)
    img = Image.composite(img, Image.new("RGB", (W, H)), m)
img.save(args.out)
print("saved", args.out)
