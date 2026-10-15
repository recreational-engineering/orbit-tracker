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
- Dragging pans temporarily; the view glides back after 4 s.
- On-screen menu (`drawMenu()`, `onTap()`): a tap (pointer moved < 6 px) on the
  bottom HUD tab opens a centred "VIEW" panel with
  ORBIT VIEW / GROUND VIEW / FLAT EARTH (icon + upper-case label, active row highlighted with a white
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
7. Track: future solid red, past thinner light red, fading with |Δt|; segments
   hidden behind the Earth are drawn faint and dashed.
   Points are lifted by altitude (radius × (1 + h/6371)); hidden = behind
   the Earth disk (`project3d()`).
7b. 5-minute travel-time marks at absolute times (`state.marks`, ground point at
   that moment). Munich view: short screen-space ticks across the track
   (`drawMarkTicks()`). Orbit view: no ticks; instead ±60° great-circle lines
   perpendicular to the ground track, printed on the globe under the glass
   shading, turning with the Earth (`drawMarkMesh()`).
8. Object: simplified upper stage (nozzle, aft dome, white tank, blue ring,
   payload adapter, gold payload) from lit frustums, nose along the projected
   3D flight direction; 30 % opacity when obstructed (`drawStage()`).
8b. Home marker: blue dot with white rim + "MUNICH" label (`drawHome()`).
9. HUD (`drawHud()`), instrument-cluster style: Barlow Condensed (OFL,
   `web/vendor/barlow-condensed-*.woff2`), white digits, small superscript
   units, line icons. Two dark tab-shaped panels that run into the round screen
   edge (`edgeBand()`): a flat edge only as wide as the content, then 58° flanks
   out to the screen circle, softened corners, hairline along the edge:
   - top tab (edge at 0.068·s, text 0.034·s): UTC time;
   - bottom tab (edge at 0.88·s, rows at 0.912·s / 0.957·s): primary row (0.038·s) = pin icon + distance
     from Munich at ground level (10 Hz, `rangeToHome()`) | clock icon + Munich
     local time of the next range minimum (20 s scan up to 4 h, ternary
     refinement to 1 s, `findNextPass()`); secondary row (0.028·s) =
     latitude | longitude | altitude icon + altitude.
   Items in a row are separated by thin vertical rules (`readoutRow()`).

Side-panel controls (time offset, altitude exaggeration, layer toggles,
night side) are prototyping aids, not part of the device UI.
