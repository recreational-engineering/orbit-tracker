# Orbit Tracker

Prototype of an orbit tracker for a round 4" display on an ESP32.
The web app in `web/` prototypes the GUI; it renders into a square canvas
the size of the display (480×480 or 720×720).

## Run

    python3 server.py        # then open http://localhost:8000

`server.py` serves `web/` and proxies orbital elements from CelesTrak
(`/api/gp?catnr=<NORAD>`, cached for 2 h in `cache/`).
Libraries and map data are vendored in `web/vendor/`, so only the element
fetch needs internet.

## GUI behaviour (reference for the hardware port)

The web prototype is the spec for the firmware. Everything is drawn in
`web/app.js` with plain Canvas 2D primitives into a square buffer of the
display size.

**Data**
- Elements: CelesTrak OMM JSON (`gp.php?CATNR=<id>&FORMAT=JSON`). Use OMM,
  not TLE — 6-digit catalog numbers (e.g. 100614) do not fit in TLE lines.
- Propagation: SGP4 (satellite.js) → ECI position. Track = ±60 min in 30 s steps,
  refreshed once a second; current position refreshed at 10 Hz.
- Geocentric lon/lat from ECI with a sidereal angle (GMST), see `geoOf()`.

**View modes** (`state.mode`, toggled with M / panel buttons)
- `orbit` — object fixed at screen centre, flight direction up. The track is
  converted with the *current* GMST for every point (inertial orbit → great
  circle → straight vertical line); the Earth rotates below in real time.
  View roll γ comes from the projected flight direction (`targetRotation()`).
- `home` — Earth fixed, Munich (48.137°N, 11.575°E) centred, north up. Track
  uses each point's own GMST (ground-relative path).
- `flat` ("Flat earth") — azimuthal equidistant projection centred on the north
  pole, whole Earth in the disc (south pole = rim), Greenwich pointing down,
  longitude increasing counter-clockwise. Per-pixel inverse: polar distance =
  (r / R)·180°, lon = atan2(x, −y) − λ. Ground-relative track drawn on the
  ground (nothing hidden), 5-min ticks, horizon circle; no sphere lighting
  (rim vignette + glint only), no reticle. Dragging only spins around the pole.
- `pilot` ("Pilot view") — perspective view from the object looking prograde
  (camera at the ECI position, forward = inertial velocity, up = radial out;
  `updateCamera()`), 55° from centre to screen edge, filling the whole round
  screen. Per pixel: ray–sphere intersection with the Earth (R = 6371 km) →
  ECI hit point → Earth-fixed via GMST → same terrain/coast/night shading,
  plus a 10° graticule sized to the pixel footprint and aerial haze toward the
  horizon (∝ (t / t_horizon)⁵). Rays that miss: dark space with a thin limb
  glow exp(−h_min / 90 km), lit on the day side. Horizon dips acos(R/(R+h))
  (~23° at 550 km) below prograde. Track in the inertial frame via perspective
  projection (`projectPilot()`): only the visible part, red (prograde →
  horizon); the part behind the Earth is not drawn. Munich marker if in view.
  No stage model, footprint or globe shading.
  Fighter-style HUD (`drawPilotHud()`), green #4dff7a with a dark shadow,
  fixed to the screen so it stays put while looking around:
  - pitch ladder conformal to the local horizontal (radial = up): horizon line
    at 0°, solid bars above / dashed bars below every 10° with numbers,
    clipped to a circle of 0.34·s (the visible Earth horizon sits at ≈ −23°);
  - heading tape under the time tab (0° = north along the local meridian,
    ticks every 5°, labels every 30° with N/E/S/W), caret + 3-digit box;
  - fixed boresight "W" at the screen centre;
  - speed box (km/s) left, altitude box (km) right, at centre height;
  - prograde marker (circle + 3 stubs) at the inertial velocity direction.
  Dragging looks around (yaw/pitch); tapping the time resets to prograde.
  High-res terrain (pilot view only, see `HI` / `requestHiTile()` in `app.js`):
  - full HYP_50M_SR (10800×5400, ~3.7 km/texel) as 15×30 JPEG tiles of
    360×360 px = 12°×12°, `web/tiles/hyp/<row>_<col>.jpg` (row 0 = 90°N…78°N,
    col 0 = 180°W…168°W), ~9 MB total, ocean tiles ~4 KB;
  - per pixel, a tile is used only where the ground footprint is fine enough
    (hit distance × pixel angle < 12 km, i.e. the nearer ground); toward the
    horizon the 2048-wide global texture is used, so typically only ~6–15
    tiles are needed at once;
  - tiles load on demand (max 3 in flight) into a fixed LRU cache of 24 tiles;
    until a tile is there, the global texture is shown (no stalls);
  - water = white texels, coastline = water/land edge inside the tile;
  - cost: on desktop the tile lookup adds ~1 ms to the ~38 ms pilot frame;
  - ESP port: bake the colour boost into the tiles offline, decode with the
    P4 hardware JPEG decoder to RGB565 (360×360×2 B = 253 KB per tile, 24 tiles
    ≈ 6 MB PSRAM), keep the same footprint threshold and LRU policy.
  Tiles were cut with `sips` from the Natural Earth TIFF (crop 360-px strips,
    then 360×360 tiles, JPEG quality 78).
- Dragging pans temporarily; the view glides back after 4 s.
- On-screen menu (`drawMenu()`, `onTap()`): a tap (pointer moved < 6 px) on the
  bottom HUD tab opens a centred "VIEW" panel with
  ORBIT VIEW / GROUND VIEW / FLAT EARTH / PILOT VIEW (icon + upper-case label, active row highlighted with a white
  bar). Tapping an option switches mode and closes; tapping outside closes.
  `home` mode is labelled "Ground view" in the UI.
- Tapping the top HUD tab (time) resets the view: drops any manual pan and
  glides back to the current mode's default centring (`resetView()`).

**Rendering layers** (back to front, `draw()`)
1. Dark instrument face + offset radial drop shadow around the globe.
2. Globe radius = 0.43 × display size, orthographic projection, rotation [λ, φ, γ].
3. One per-pixel pass (`renderGlobePixels()`, inverse orthographic lookup):
   - land/sea from a land mask: `land-50m.json` rasterised once to a
     2048×1024 equirectangular byte grid (`buildLandMask()`);
   - sea flat RGB(34, 78, 172); land from `web/vendor/terrain-2048.jpg`
     (Natural Earth HYP_50M_SR), colours boosted at load (saturation ×2.4,
     gamma 1.35); white texels inside land = lakes → sea colour;
   - coastline = mask edge (neighbour texel differs), blended 55 % to RGB(15, 25, 15);
   - night side: sun elevation from the approximate subsolar point → night
     factor n with a soft twilight band; mix 0.78·n toward RGB(4, 8, 28), then
     add warm city lights from `web/vendor/night-2048.jpg` (NASA Earth at
     Night 2012, red channel) × n.
   No per-frame vector clipping: on desktop this pass is ~50–65 ms at 480².
4. Dashed country borders (110m), faint white 10° graticule (vector).
4b. Both views: the object's horizon on the ground — small circle around
   the sub-satellite point with radius acos(R / (R + h)), thin yellow line
   (`drawFootprint()`).
5. Shading: radial light from upper-left, limb darkening, elliptical glass glint.
6. Centre reticle.
7. Track: future solid red, past thinner light red, fading with |Δt|. Points are
   lifted by altitude (radius × (1 + h/6371)); hidden = behind the Earth disk
   (`project3d()`). Hidden parts are NOT drawn through the globe.
7a. Behind-the-Earth display (`drawBehindEarth()`, globe views only): hidden
   parts keep the real orbit's projected shape but are drawn as a solid grey
   (#c4c8cc) line printed on the visible surface (inside the globe clip, under
   the glass shading), so the globe stays opaque. A hidden object is a hollow grey
   ring (radius s/55, stroke = track width s/260) at its projected position (no stage model); in ground view hidden 5-min
   ticks are grey.
7b. 5-minute travel-time marks at absolute times (`state.marks`, ground point at
   that moment). Munich view: short screen-space ticks across the track
   (`drawMarkTicks()`). Orbit view: no ticks; instead ±60° great-circle lines
   perpendicular to the ground track, printed on the globe under the glass
   shading, turning with the Earth (`drawMarkMesh()`).
8. Object: simplified upper stage (nozzle, aft dome, white tank, blue ring,
   payload adapter, gold payload) from lit frustums, nose along the projected
   3D flight direction; not drawn when obstructed (`drawStage()`).
8b. Home marker: blue dot with white rim + "MUNICH" label (`drawHome()`).
9. HUD (`drawHud()`), instrument-cluster style: Barlow Condensed (OFL,
   `web/vendor/barlow-condensed-*.woff2`), white digits, small superscript
   units, line icons. Two dark tab-shaped panels that run into the round screen
   edge (`edgeBand()`): a flat edge only as wide as the content, then 58° flanks
   out to the screen circle, softened corners, hairline along the edge:
   - top tab (edge at 0.068·s, text 0.034·s): UTC time;
   - bottom tab (edge at 0.88·s, rows at 0.912·s / 0.957·s): primary row (0.038·s) = dimension-line icon |←→| + distance
     from Munich at ground level (10 Hz, `rangeToHome()`) | minimum-curve icon (∪ with a dot) + Munich
     local time of the next range minimum (20 s scan up to 4 h, ternary
     refinement to 1 s, `findNextPass()`); secondary row (0.028·s) =
     latitude | longitude | altitude icon + altitude.
   Items in a row are separated by thin vertical rules (`readoutRow()`).

Side-panel controls (time offset, altitude exaggeration, layer toggles,
night side) are prototyping aids, not part of the device UI.
