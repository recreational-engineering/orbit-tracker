# Orbit tracker firmware

The web prototype (`../web/app.js`) on the Waveshare ESP32-P4-WIFI6-Touch-LCD-4C
(4" round 720×720 MIPI-DSI panel, GT911 touch): the round display with its five views
and the on-screen view menu; the settings panel is not ported. It boots into Earth spin.

- Views (menu: tap the bottom tab): Earth spin, ground reference, satellite reference,
  pilot reference, flat earth
- Touch: drag to pan (trackball; the flat earth turns about the pole, the pilot view
  looks around), flick for inertia; tap the top tab or double-tap to reset; the view
  glides back 4 s after a pan

Code:

- `main/scene.c` – port of `draw()` and the views, track, rose, HUD, menu, easing and touch
- `main/globe.c` – the globe's pixels (texture, lighting, night, city lights; pilot ray cast)
- `main/dlist.c`, `main/gfx.c` – display list and the antialiased subset of Canvas 2D
- `main/model.c`, `components/sgp4` – SGP4 propagation (Vallado), track, ticks, next pass
- `main/tiles.c` – high-resolution terrain and city-light tiles for the pilot view (JPEG,
  hardware decoder) from the `tiles` flash partition
- `tools/gen_assets.py` – bakes textures, borders, graticule, fonts and the orbital
  elements (`../cache/gp_<NORAD>.json`) into `main/generated/`; `--tiles` builds `tiles.bin`

## Build and flash

ESP-IDF v6.1. `gen_assets.py` needs Pillow and numpy.

    python3 tools/gen_assets.py            # once, and whenever the elements should be refreshed
    python3 tools/gen_assets.py --tiles    # once: pilot-view tiles
    . ~/esp/esp-idf/export.sh
    idf.py build
    idf.py -p /dev/cu.usbmodem* flash
    tools/flash_tiles.sh                   # once: writes tiles.bin to the tiles partition

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

The stats line `PANEL refresh max … late refreshes …` counts panel refreshes that ended late (the
panel's DMA is restarted from an interrupt each ~7.9 ms refresh; a late restart shows as a white
flash). Tiles are therefore read from memory-mapped flash (a flash read operation would pause
interrupts) and the display interrupts are cache-safe (`CONFIG_LCD_DSI_ISR_CACHE_SAFE`).

## Rendering and performance

Each frame is recorded into a display list on both cores, then drawn in 20-row strips
of internal RAM (globe pixels, then the vector layers), each core taking every other
strip, and copied to one of three frame buffers; the panel swaps at the end of a
refresh, so frames never tear and the loop never waits for the panel. (The strips are
kept small so the renderer's internal RAM coexists with the Wi-Fi/ESP-Hosted stack.)

- Earth spin is drawn at full resolution every frame (the globe turns all the time).
- Ground, satellite reference and flat earth show a full-resolution globe rendered by a
  background task while the view is still, refreshed as it drifts.
- While the view moves (finger down, inertia, a fast glide), the globe is drawn at half
  resolution (2×2 blocks, exact anti-aliased rim); the vector layers stay at full resolution.
- The pilot view casts one ray per 2×2 block, exactly every 4 blocks and interpolated between.

Frame rates (fps) measured with `tools/perf_test.py`:

| View | still | panning |
|---|---|---|
| Earth spin | 12 | 18 |
| Ground reference | 27 | 19–20 |
| Satellite reference | 23 | 19 |
| Pilot reference | 12–16 | 13–15 |
| Flat earth | 24 | 15 |

The limit is the CPU: about 70–120 cycles per globe pixel on the two 400 MHz cores, with
textures and per-pixel tables in PSRAM.
