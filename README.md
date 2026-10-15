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
- Dragging pans temporarily; the view glides back after 4 s.

**Rendering layers** (back to front, `draw()`)
1. Dark instrument face + offset radial drop shadow around the globe.
2. Globe radius = 0.43 × display size, orthographic projection, rotation [λ, φ, γ].
3. Sea: flat RGB(34, 78, 172). Land: per-pixel inverse orthographic lookup into
   `web/vendor/terrain-2048.jpg` (Natural Earth HYP_50M_SR, equirectangular),
   colours boosted at load (saturation ×2.4, gamma 1.35), clipped to the
   vector coastline (`land-50m.json`). See `renderTexturedGlobe()`.
4. Coastlines, dashed country borders, faint white 10° graticule.
4b. Night side (same per-pixel pass, `renderGlobePixels()`): sun elevation from
   the approximate subsolar point → night factor with a soft twilight band;
   darken with RGBA(4, 8, 28, 200·n), then add warm city lights (additive) from
   `web/vendor/night-2048.jpg` (NASA Earth at Night 2012, red channel).
5. Shading: radial light from upper-left, limb darkening, elliptical glass glint.
6. Centre reticle.
7. Track: future solid red, past thinner light red, fading with |Δt|; segments
   hidden behind the Earth are drawn faint and dashed. Ticks every 10 min.
   Points are lifted by altitude (radius × (1 + h/6371)); hidden = behind
   the Earth disk (`project3d()`).
8. Object: simplified upper stage (nozzle, aft dome, white tank, blue ring,
   payload adapter, gold payload) from lit frustums, nose along the projected
   3D flight direction; 30 % opacity when obstructed (`drawStage()`).
8b. Home marker: blue dot with white rim + "MUNICH" label (`drawHome()`).
9. HUD pills: UTC time (top), lat/lon/alt (bottom).

Side-panel controls (time offset, altitude exaggeration, layer toggles,
night side) are prototyping aids, not part of the device UI.
