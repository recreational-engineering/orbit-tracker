# Orbit tracker firmware

The web prototype (`../web/app.js`) on the Waveshare ESP32-P4-WIFI6-Touch-LCD-4C
(4" round 720×720 MIPI-DSI panel, GT911 touch): the round display with its five views
and the on-screen view menu; the settings panel is not ported. It boots into Earth spin.

- Views (menu: tap the bottom tab): Earth spin, ground reference, satellite reference,
  pilot reference, flat earth
- Mounted with the USB-C power connector at the bottom: frames and touch are turned 180° from
  the panel's native orientation (`GFX_ROT180` in `main/gfx.h`); screenshots come out upright
- Touch: drag to pan (trackball; the flat earth turns about the pole, the pilot view
  looks around), flick for inertia; tap the top tab or double-tap to reset; the view
  glides back 4 s after a pan

Code:

- `main/scene.c` – port of `draw()` and the views, track, rose, HUD, menu, easing and touch
- `main/globe.c` – the globe's pixels (texture, lighting, night, city lights; pilot ray cast)
- `main/dlist.c`, `main/gfx.c` – display list and the antialiased subset of Canvas 2D
- `main/model.c`, `components/sgp4` – SGP4 propagation (Vallado), track, ticks, next pass
- `main/tiles.c` – high-resolution terrain and city-light tiles for the pilot view (JPEG,
  hardware decoder) from the `tiles` flash partition: 12° rows, with fewer, wider tiles toward
  the poles (so a polar view needs about as many tiles as any other); nearest tiles load first,
  and a tile in view is never evicted for a farther one
- `tools/gen_assets.py` – bakes textures, borders, graticule, fonts and the orbital
  elements (`../cache/gp_<NORAD>.json`) into `main/generated/`; `--tiles` builds `tiles.bin`

## Build and flash

ESP-IDF v6.1. `gen_assets.py` needs Pillow and numpy.

    python3 tools/gen_assets.py            # once, and whenever the elements should be refreshed
    python3 tools/gen_assets.py --tiles    # once: pilot-view tiles
    . ~/esp/esp-idf/export.sh
    idf.py build
    idf.py -p /dev/cu.usbmodem* flash
    tools/flash_tiles.sh                   # once (and after the tile layout changes): writes tiles.bin

## Standby

From 22:00 to 08:00 (Munich time) the display goes dark after 5 minutes without a touch: the
backlight is switched off (no light, unlike a black frame) and nothing is rendered, while the
touch controller keeps polling. A touch wakes it (that touch is not taken as a tap) for another
5 minutes; at 08:00 it comes on by itself. The console logs `standby: display off/on`.

## Clock and console

The board has no network time yet. It starts at the time the assets were generated;
set the real time over USB after each power-up:

    python3 tools/set_time.py

The console runs at 2 Mbaud and logs frame rate and the object's position every 5 s.
The tools find the board's CH343 port by its USB id. Console commands:

| Command | |
|---|---|
| `T<unix s>` | set the clock |
| `P` | print the frame statistics now |
| `S` | dump the frame on screen (`tools/screenshot.py out.png --round`) |
| `K x y` / `G x0 y0 x1 y1 ms` | simulated tap / drag |
| `F` | flash monitor on/off: logs frames where part of the screen jumps for one frame |
| `N` | network sync now (time + orbital elements) |

## Network (time and orbital elements)

Wi-Fi runs on the board's on-board ESP32-C6 co-processor via ESP-Hosted (`esp_wifi_remote`).
The device joins only to read two things and then takes the radio back down:

- the current time, from NTP (`pool.ntp.org`);
- fresh orbital elements for the compiled-in NORAD id, from CelesTrak, applied live and saved
  to NVS so they survive a reboot (loaded at the next boot before the first sync).

It syncs once at boot and then once a day at 00:00 UTC (and on the `N` command). Between syncs
the radio is off. It joins as a station only (never an access point), presents one nondescript
DHCP host name (an office-printer style name), and makes no other connection — there is no
server, no discovery service, and no path that sends device data anywhere.

Credentials and the host name live in `main/net_secret.h` (git-ignored; copy
`main/net_secret.h.example`). The element fetch uses plain HTTP: the response is public orbital
data and the request reveals only the public NORAD id (which TLS would expose via SNI anyway).
TLS does not currently complete over this board's co-processor firmware; see the note below.

> The C6 reports an ESP-Hosted firmware version mismatch (`coprocessor=0.0.0` vs host 3.0.9).
> Basic traffic (DNS, TCP, UDP/NTP, HTTP) works, but the TLS handshake does not complete and the
> boot auto-init can occasionally fault. Updating the C6 slave firmware to match should fix both.

Test tools: `tools/drive.py` (scripted taps, drags and screenshots), `tools/perf_test.py`
(frame rate per view, still and panning; `--time` for a fixed scene),
`tools/flash_test.py` (all views, menu, resets and glides with the flash monitor on),
`tools/pilot_flash_test.py` (cold start, pilot view entered repeatedly and left idle; counts late
panel refreshes, DSI underruns and tile decodes).

The `STATS` line splits the strip time of the busier core into globe pixels (`px`), vector
layers (`replay`) and the copy to the frame buffer (`copy`). The `PANEL` line gives frames that
missed their refresh slot (`frames off target`, see frame pacing below), whether the render
strips are in fast RAM (`strips fast`), and the pilot view's tile cache: tiles decoded, the most
tiles in view in one frame (terrain + lights; the caches hold 22 and 14) and the most still
missing.

The stats line `PANEL refresh max … late refreshes …` counts panel refreshes that ended late (the
panel's DMA is restarted from an interrupt each ~15.4 ms refresh; a late restart shows as a white
flash). The display interrupts are cache-safe (`CONFIG_LCD_DSI_ISR_CACHE_SAFE`) and flash reads
are avoided while the panel runs. (The tiles partition lies beyond the 16 MB the cache can map,
so tile reads still use flash operations; see the open task on the flash layout.)

## Rendering and performance

Each frame is recorded into a display list on both cores, then drawn in 20-row strips
of internal RAM (globe pixels, then the vector layers), each core taking every other
strip, and copied to one of three frame buffers; the panel swaps at the end of a
refresh, so frames never tear.

- **Panel and memory clocks.** The panel refreshes at ~65 Hz (`main/panel_rate.c` overrides the
  board package's 80 MHz pixel clock, ~131 Hz): every refresh streams the 1 MB frame buffer out of
  PSRAM, so the slower scan leaves that bandwidth to rendering. PSRAM runs at 250 MHz (chip
  revision ≥ 3; checked by the boot memory test).
- **Frame pacing.** Each frame picks the refresh it will appear at (from a running estimate of
  its render time) and is drawn for exactly that instant; a presenter task hands it to the panel
  during the refresh before, while the loop already draws the next one. Motion (the spin, glides,
  inertia) therefore advances evenly; without it, frames shown for 4 or 5 refreshes alternately
  made the spin judder.
- **Shading.** The light is stored per pixel as 32 dithered levels and the product is dithered
  again when it is truncated to RGB565 (and the twilight blend too): plain truncation biased the
  shading in visible rings around the highlight.
- **Pilot texture filtering.** Terrain tiles carry two mip levels (made when a tile is decoded);
  each 2x2 block samples the level matching its ground footprint (geometric mean of the
  across and along-view sizes) bilinearly, the global texture beyond. Point samples of a finer
  texture made coastlines and ridges shimmer as the view moved. The graticule is antialiased and
  the haze dithered. (The strips are
kept small so the renderer's internal RAM coexists with the Wi-Fi/ESP-Hosted stack. They must
be in the main SRAM: the P4's low-power SRAM also counts as internal RAM but is several times
slower, so a strip there quarters the frame rate. The console and network task stacks live in
the low-power SRAM to leave the main SRAM to the strips and Wi-Fi.)

- Earth spin is drawn at full resolution every frame (the globe turns all the time).
- Ground, satellite reference and flat earth show a full-resolution globe rendered by a
  background task while the view is still, refreshed as it drifts.
- While the view moves (finger down, inertia, a fast glide), the globe is drawn at half
  resolution (2×2 blocks, exact anti-aliased rim); the vector layers stay at full resolution.
- The pilot view casts one ray per 2×2 block, exactly every 4 blocks and interpolated between.

Frame rates (fps) measured with `tools/perf_test.py`:

| View | still | panning |
|---|---|---|
| Earth spin | 13 | 19–20 |
| Ground reference | 30 | 21 |
| Satellite reference | 20 | 19 |
| Pilot reference | 13 | 12 |
| Flat earth | 25 | 17 |

(`perf_test.py --time 1792300000`, a fixed scene; the night side costs more, so other times
differ.) Per frame on the busier core, globe pixels take ~50 ms in Earth spin and ~60 ms in the
pilot view (CPU-bound: ~140 cycles a pixel, the per-pixel light table is only ~10 % of it), the
vector layers ~12 ms (17 ms flat earth) and the copy out ~6 ms.
