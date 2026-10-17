#!/usr/bin/env python3
"""Bake the web prototype's data into firmware assets (firmware/main/generated/).

    python3 tools/gen_assets.py [--catnr 100614]

Needs Pillow, numpy and fontTools (+ brotli for the woff2 fonts). Produces:
  texture.bin   2048x1024 equirectangular RGB565 in 8x8 tiles, with sea, lakes and
                coastline applied (loadTexture(), buildLandMask(), renderGlobePixels());
                texture_half.bin the same at 1024x512
  lights.bin    city-light intensity mips 2048, 1024, 512, 256 (loadNightLights())
  tiles.bin     (--tiles) high-res terrain and lights JPEG tiles for the pilot view
  borders.bin   country borders (110m)  } polylines as int16 unit vectors,
  graticule.bin d3.geoGraticule10()     } resampled to <= 1 deg per segment
  fonts.h       Barlow Condensed glyphs at the sizes the HUD and menu use
  elements.h    OMM orbital elements from ../cache/gp_<catnr>.json
"""
import argparse
import datetime as dt
import json
import math
import os
import io
import struct
import tempfile

import numpy as np
from fontTools.ttLib import TTFont
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
REPO = os.path.dirname(FW)
VENDOR = os.path.join(REPO, "web", "vendor")
OUT = os.path.join(FW, "main", "generated")

SEA = (34, 78, 172)
COAST = (15, 25, 15)
DISPLAY = 720
MAX_STEP_DEG = 1.0


def rgb565(r, g, b):
    r = np.asarray(r, np.uint32)
    g = np.asarray(g, np.uint32)
    b = np.asarray(b, np.uint32)
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


# ---------- topojson ----------
def decode_arcs(topo):
    sx, sy = topo["transform"]["scale"]
    tx, ty = topo["transform"]["translate"]
    arcs = []
    for arc in topo["arcs"]:
        x = y = 0
        pts = []
        for dx, dy in arc:
            x += dx
            y += dy
            pts.append((x * sx + tx, y * sy + ty))
        arcs.append(pts)
    return arcs


def ring_coords(arcs, ring):
    pts = []
    for i in ring:
        a = arcs[i] if i >= 0 else arcs[~i][::-1]
        pts.extend(a if not pts else a[1:])
    return pts


def polygons(obj):
    geoms = obj["geometries"] if obj["type"] == "GeometryCollection" else [obj]
    for g in geoms:
        if g["type"] == "Polygon":
            yield g["arcs"]
        elif g["type"] == "MultiPolygon":
            yield from g["arcs"]


# ---------- polylines ----------
def unit(lon, lat):
    l, p = math.radians(lon), math.radians(lat)
    return (math.cos(p) * math.cos(l), math.cos(p) * math.sin(l), math.sin(p))


def resample(lonlat):
    """Great-circle subdivision so no segment spans more than MAX_STEP_DEG."""
    out = []
    prev = None
    for lon, lat in lonlat:
        v = unit(lon, lat)
        if prev is not None:
            ang = math.degrees(math.acos(max(-1, min(1, sum(a * b for a, b in zip(prev, v))))))
            n = int(ang / MAX_STEP_DEG)
            if n > 0 and ang > 1e-9:
                s = math.sin(math.radians(ang))
                for k in range(1, n + 1):
                    t = k / (n + 1)
                    a = math.sin((1 - t) * math.radians(ang)) / s
                    b = math.sin(t * math.radians(ang)) / s
                    out.append(tuple(a * p + b * q for p, q in zip(prev, v)))
        out.append(v)
        prev = v
    return out


def write_lines(path, lines):
    lines = [l for l in lines if len(l) >= 2]
    with open(path, "wb") as f:
        f.write(struct.pack("<II", len(lines), sum(len(l) for l in lines)))
        for l in lines:
            f.write(struct.pack("<H", len(l)))
        for l in lines:
            for v in l:
                f.write(struct.pack("<hhh", *(int(round(c * 32767)) for c in v)))
    return len(lines), sum(len(l) for l in lines)


def graticule10():
    """d3.geoGraticule10(): vertices every 2.5 deg, as d3 generates them."""
    step = 2.5
    lines = []
    for lon in range(-180, 180, 10):
        lim = 90 if lon % 90 == 0 else 80
        lines.append([(lon, -lim + k * step) for k in range(int(2 * lim / step) + 1)])
    for lat in range(-80, 81, 10):
        lines.append([(-180 + k * step, lat) for k in range(int(360 / step) + 1)])
    return lines


# ---------- texture ----------
def land_mask(land_topo, land_arcs, w, h, ss=4):
    """buildLandMask(): land-50m filled into an equirectangular grid, 1 = land."""
    mask = Image.new("L", (w * ss, h * ss), 0)
    d = ImageDraw.Draw(mask)
    to_px = lambda p: ((p[0] + 180) / 360 * w * ss, (90 - p[1]) / 180 * h * ss)
    for poly in polygons(land_topo["objects"]["land"]):
        for i, ring in enumerate(poly):
            # Rings crossing the antimeridian jump by 360 deg: unwrap, then draw shifted copies.
            pts = ring_coords(land_arcs, ring)
            unwrapped, off = [], 0
            for k, (lon, lat) in enumerate(pts):
                if k and lon + off - unwrapped[-1][0] > 180:
                    off -= 360
                elif k and lon + off - unwrapped[-1][0] < -180:
                    off += 360
                unwrapped.append((lon + off, lat))
            # A ring around a pole (Antarctica) ends 360 deg from where it started: close it
            # along that pole, or the cap inside it stays sea.
            span = unwrapped[-1][0] - unwrapped[0][0]
            if abs(abs(span) - 360) < 1:
                pole = -90 if min(p[1] for p in unwrapped) < 0 else 90
                unwrapped += [(unwrapped[-1][0], pole), (unwrapped[0][0], pole)]
            for shift in (-360, 0, 360):
                d.polygon([to_px((lon + shift, lat)) for lon, lat in unwrapped], fill=0 if i else 255)
    if ss == 1:
        return np.asarray(mask) >= 128
    return np.asarray(mask.resize((w, h), Image.BOX)) >= 128


# boostTerrain(): saturation x2.4, then gamma 1.35 through a 256-entry LUT; white = lake.
GAMMA_LUT = np.array([round(255 * (i / 255) ** 1.35) for i in range(256)], np.uint8)
# LIGHTS_LUT: city-light intensity from the red channel of the NASA night image.
LIGHTS_LUT = np.array([int(255 * min(1, max(0, (r - 45) / 170) ** 1.1)) for r in range(256)], np.uint8)


def boost(img):
    """img: uint8 HxWx3 -> (boosted uint8 HxWx3, lake bool HxW), as boostTerrain() in app.js."""
    f = img.astype(np.float64)
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    lake = (img[..., 0] > 242) & (img[..., 1] > 242) & (img[..., 2] > 242)
    m = (r + g + b) / 3
    out = np.empty_like(img)
    for k in range(3):
        idx = np.clip(np.floor(m + (f[..., k] - m) * 2.4 + 0.5), 0, 255).astype(np.uint8)
        out[..., k] = GAMMA_LUT[idx]
    return out, lake


def finish(day, land, lake, wrap):
    """Sea, lakes and the coastline (mask edge to the right or below) into the colours."""
    day = day.astype(np.float64)
    sea = ~land | lake
    for k in range(3):
        day[..., k][sea] = SEA[k]
    if wrap:
        right = np.roll(land, -1, axis=1)
        right_ok = np.ones_like(land)
    else:
        right = np.hstack([land[:, 1:], land[:, -1:]])
        right_ok = np.ones_like(land); right_ok[:, -1] = False
    below = np.vstack([land[1:], land[-1:]])
    below_ok = np.ones_like(land); below_ok[-1] = False
    coast = ((land != right) & right_ok) | ((land != below) & below_ok)
    for k in range(3):
        day[..., k][coast] = day[..., k][coast] * 0.45 + COAST[k] * 0.55
    return np.clip(np.rint(day), 0, 255).astype(np.uint8)


def write_rgb565_tiles(name, day):
    """RGB565 in 8x8 texel tiles (128 bytes, one ESP32-P4 L2 cache line)."""
    h, w, _ = day.shape
    px = rgb565(day[..., 0], day[..., 1], day[..., 2]).astype("<u2")
    np.ascontiguousarray(px.reshape(h // 8, 8, w // 8, 8).transpose(0, 2, 1, 3)).tofile(os.path.join(OUT, name))


def halve(px):
    """halve(): 2x2 box filter with a mild gain (x0.32 of the sum), truncated like a Uint8Array."""
    h, w = px.shape
    s = px.astype(np.float64).reshape(h // 2, 2, w // 2, 2).sum(axis=(1, 3))
    return np.minimum(255, np.floor(s * 0.32)).astype(np.uint8)


def texture(land_topo, land_arcs):
    img = np.asarray(Image.open(os.path.join(VENDOR, "terrain-2048.jpg")).convert("RGB"))
    h, w, _ = img.shape
    boosted, lake = boost(img)
    land = land_mask(land_topo, land_arcs, w, h)
    day = finish(boosted, land, lake, wrap=True)
    write_rgb565_tiles("texture.bin", day)
    # Half-size level (2x2 box filter) for the half-resolution render while the view moves.
    box = day.astype(np.float64).reshape(h // 2, 2, w // 2, 2, 3).mean(axis=(1, 3))
    write_rgb565_tiles("texture_half.bin", np.clip(np.rint(box), 0, 255).astype(np.uint8))
    Image.fromarray(day).save(os.path.join(OUT, "texture_preview.png"))

    # City lights: level 0 at 2048 and the mip chain 1024 / 512 / 256 (loadNightLights()).
    night = np.asarray(Image.open(os.path.join(VENDOR, "night-2048.jpg")).convert("RGB"))
    lv = [LIGHTS_LUT[night[..., 0]]]
    while len(lv) < 4:
        lv.append(halve(lv[-1]))
    with open(os.path.join(OUT, "lights.bin"), "wb") as f:
        for m in lv:
            f.write(m.tobytes())
    return w, h


# ---------- high-res tiles for the pilot view ----------
HI_T, HI_ROWS, HI_COLS = 360, 15, 30
TILES_SRC = os.path.join(REPO, "web", "tiles")


def tiles_pack(land_topo, land_arcs):
    """tiles.bin for the 'tiles' flash partition: per tile a 4:4:4 JPEG of the final terrain
    colours (boost, land/sea from the land-50m mask, lakes and coastline baked in, as
    requestHiTile() + tileLandMask() + renderGlobePixels() do) and a greyscale JPEG of the
    city-light intensity (requestLitTile()). Header: b'OTTL', count, then per tile
    (terrain offset, length, lights offset, length)."""
    W, H = HI_COLS * HI_T, HI_ROWS * HI_T
    land_all = land_mask(land_topo, land_arcs, W, H, ss=1)
    n = HI_ROWS * HI_COLS
    blobs = []
    for key in range(n):
        row, col = divmod(key, HI_COLS)
        img = np.asarray(Image.open(os.path.join(TILES_SRC, "hyp", f"{row}_{col}.jpg")).convert("RGB"))
        boosted, lake = boost(img)
        land = land_all[row * HI_T:(row + 1) * HI_T, col * HI_T:(col + 1) * HI_T]
        day = finish(boosted, land, lake, wrap=False)
        b1 = io.BytesIO()
        Image.fromarray(day).save(b1, "JPEG", quality=88, subsampling=0)
        lit = np.asarray(Image.open(os.path.join(TILES_SRC, "lights", f"{row}_{col}.jpg")).convert("RGB"))
        b2 = io.BytesIO()
        Image.fromarray(LIGHTS_LUT[lit[..., 0]], "L").save(b2, "JPEG", quality=85)
        blobs.append((b1.getvalue(), b2.getvalue()))
    with open(os.path.join(OUT, "tiles.bin"), "wb") as f:
        hdr = 8 + n * 16
        off = (hdr + 15) & ~15
        table, data = [], bytearray()
        for t, l in blobs:
            table.append((off + len(data), len(t)))
            data += t + b"\0" * (-len(t) % 16)
            table.append((off + len(data), len(l)))
            data += l + b"\0" * (-len(l) % 16)
        f.write(b"OTTL" + struct.pack("<I", n))
        for k in range(n):
            f.write(struct.pack("<IIII", *table[2 * k], *table[2 * k + 1]))
        f.write(b"\0" * (off - hdr))
        f.write(data)
    return os.path.getsize(os.path.join(OUT, "tiles.bin"))


# ---------- fonts ----------
GLYPHS = " 0123456789:.-/°ABCDEFGHIJKLMNOPQRSTUVWXYZkm"
# (weight, px size, outline stroke px): the sizes the HUD, menu, Munich label, compass rose and
# pilot HUD use at 720 px. Outlines stand in for hudText()'s dark stroke and the HUD shadow.
FONTS = [(600, 24, 0), (600, 27, 0), (600, 20, 0), (500, 12, 0), (500, 14, 0), (500, 10, 0),
         (500, 20, 2), (500, 19, 0), (600, 13, 0), (600, 15, 1), (600, 22, 1), (500, 15, 1), (600, 19, 1)]


def font_header():
    tmp = tempfile.mkdtemp()
    ttf = {}
    for wt in (500, 600):
        f = TTFont(os.path.join(VENDOR, f"barlow-condensed-{wt}.woff2"))
        f.flavor = None
        ttf[wt] = (os.path.join(tmp, f"barlow-{wt}.ttf"), f)
        f.save(ttf[wt][0])
    lines = ["// Generated by tools/gen_assets.py from Barlow Condensed (SIL OFL, web/vendor). Do not edit.",
             "#pragma once", '#include "gfx.h"', ""]
    bits = []
    for wt, size, outline in FONTS:
        path, tt = ttf[wt]
        upem = tt["head"].unitsPerEm
        cmap = tt.getBestCmap()
        hhea = tt["hhea"]
        pil = ImageFont.truetype(path, size)
        name = f"FONT_{wt}_{size}"
        entries = []
        for ch in GLYPHS:
            adv = tt["hmtx"][cmap[ord(ch)]][0] * size / upem
            glyph = []
            for stroke in ((0, outline) if outline else (0,)):
                im, (ox, oy) = pil.getmask2(ch, mode="L", anchor="ls", stroke_width=stroke)
                gw, gh = im.size
                data = bytes(im) if gw and gh else b""
                glyph.append((len(bits), gw, gh, ox, oy))
                bits.extend(data)
            entries.append((ord(ch), adv, glyph))
        lines.append(f"static const gfx_glyph_t {name}_glyphs[] = {{")
        for cp, adv, glyph in entries:
            off, gw, gh, ox, oy = glyph[0]
            o_off, o_w, o_h, o_ox, o_oy = glyph[1] if outline else (0, 0, 0, 0, 0)
            lines.append(f"  {{{cp}, {adv:.4f}f, {off}, {gw}, {gh}, {ox}, {oy}, {o_off}, {o_w}, {o_h}, {o_ox}, {o_oy}}},")
        lines.append("};")
        # Canvas textBaseline 'middle': halfway between the font's ascent and descent.
        middle = (hhea.ascent + hhea.descent) / 2 * size / upem
        lines.append(f"static const gfx_font_t {name} = {{{name}_glyphs, {len(entries)}, {middle:.3f}f, {str(outline).lower()}, font_bits}};")
        lines.append("")
    lines.insert(4, f"static const uint8_t font_bits[{len(bits)}] = {{" + ",".join(map(str, bits)) + "};")
    with open(os.path.join(OUT, "fonts.h"), "w") as f:
        f.write("\n".join(lines) + "\n")


# ---------- elements ----------
def elements_header(catnr):
    with open(os.path.join(REPO, "cache", f"gp_{catnr}.json")) as f:
        omm = json.load(f)[0]
    epoch = dt.datetime.fromisoformat(omm["EPOCH"]).replace(tzinfo=dt.timezone.utc)
    now = dt.datetime.now(dt.timezone.utc)
    with open(os.path.join(OUT, "elements.h"), "w") as f:
        f.write(f"""// Generated by tools/gen_assets.py from cache/gp_{catnr}.json. Do not edit.
#pragma once
#define OMM_OBJECT_NAME "{omm['OBJECT_NAME']}"
#define OMM_OBJECT_ID "{omm['OBJECT_ID']}"
#define OMM_NORAD_CAT_ID {omm['NORAD_CAT_ID']}
#define OMM_EPOCH_STR "{omm['EPOCH']}"
#define OMM_EPOCH_UNIX {epoch.timestamp():.6f}
#define OMM_MEAN_MOTION {omm['MEAN_MOTION']!r}
#define OMM_ECCENTRICITY {omm['ECCENTRICITY']!r}
#define OMM_INCLINATION {omm['INCLINATION']!r}
#define OMM_RA_OF_ASC_NODE {omm['RA_OF_ASC_NODE']!r}
#define OMM_ARG_OF_PERICENTER {omm['ARG_OF_PERICENTER']!r}
#define OMM_MEAN_ANOMALY {omm['MEAN_ANOMALY']!r}
#define OMM_BSTAR {omm['BSTAR']!r}
#define OMM_MEAN_MOTION_DOT {omm['MEAN_MOTION_DOT']!r}
#define OMM_MEAN_MOTION_DDOT {omm['MEAN_MOTION_DDOT']!r}
// Clock fallback until the time is set over serial (tools/set_time.py).
#define ASSETS_GEN_UNIX {int(now.timestamp())}
""")
    return omm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--catnr", type=int, default=100614)
    ap.add_argument("--tiles", action="store_true", help="also build tiles.bin (pilot view, slow)")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for stale in ("coast.bin", "font.h", "texture_preview.png"):
        if os.path.exists(os.path.join(OUT, stale)):
            os.remove(os.path.join(OUT, stale))

    with open(os.path.join(VENDOR, "land-50m.json")) as f:
        land_topo = json.load(f)
    with open(os.path.join(VENDOR, "countries-110m.json")) as f:
        countries = json.load(f)
    land_arcs = decode_arcs(land_topo)
    country_arcs = decode_arcs(countries)

    print("texture", texture(land_topo, land_arcs))
    if args.tiles:
        print("tiles.bin bytes", tiles_pack(land_topo, land_arcs))

    # topojson.mesh(countries, objects.countries, (a, b) => a !== b): arcs shared by two countries.
    users = {}
    for gi, g in enumerate(countries["objects"]["countries"]["geometries"]):
        polys = [g["arcs"]] if g["type"] == "Polygon" else g["arcs"] if g["type"] == "MultiPolygon" else []
        for poly in polys:
            for ring in poly:
                for i in ring:
                    users.setdefault(i if i >= 0 else ~i, set()).add(gi)
    borders = [resample(country_arcs[i]) for i, u in sorted(users.items()) if len(u) > 1]
    print("border lines/points", write_lines(os.path.join(OUT, "borders.bin"), borders))

    grat = [[unit(lon, lat) for lon, lat in l] for l in graticule10()]
    print("graticule lines/points", write_lines(os.path.join(OUT, "graticule.bin"), grat))

    font_header()
    omm = elements_header(args.catnr)
    print("elements", omm["OBJECT_NAME"], omm["EPOCH"])


if __name__ == "__main__":
    main()
