#!/usr/bin/env python3
"""Drive the board over its console and grab screenshots, in one session.

    python3 tools/drive.py STEP [STEP ...]

Steps: a console command ("K 360 655", "G 220 400 500 320 700", "T", "P"), "wait:SECONDS",
or "shot:FILE.png" (round-masked screenshot). "T" sends the computer's clock.
"""
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

from devserial import open_console

W = H = 720
MARK = b"@@FRAME 720 720 RGB565LE@@\n"


def shot(s, path):
    s.reset_input_buffer()
    s.write(b"S\n")
    buf, deadline = b"", time.time() + 10
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
    m = Image.new("L", (W, H), 0)
    ImageDraw.Draw(m).ellipse((0, 0, W - 1, H - 1), fill=255)
    Image.composite(img, Image.new("RGB", (W, H)), m).save(path)
    print("saved", path)


def main():
    s = open_console(timeout=0.2)
    for step in sys.argv[1:]:
        if step.startswith("wait:"):
            end = time.time() + float(step[5:])
            while time.time() < end:
                s.read(65536)
        elif step.startswith("shot:"):
            shot(s, step[5:])
        else:
            if step == "T":
                step = f"T{time.time():.3f}"
            s.write((step + "\n").encode())
            if step == "P":
                end, out = time.time() + 1.0, b""
                while time.time() < end:
                    out += s.read(65536)
                for line in out.decode(errors="replace").splitlines():
                    if "STATS" in line:
                        print(line[line.index("STATS"):])
            else:
                time.sleep(0.05)


if __name__ == "__main__":
    main()
