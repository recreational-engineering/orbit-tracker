/* Orbit tracker prototype — round display globe renderer.
 *
 * Everything is drawn with plain Canvas 2D calls into a square buffer that
 * matches the target display, so the drawing steps map onto what an ESP32
 * graphics library (LVGL / LovyanGFX / TFT_eSPI) can do later.
 */
(() => {
  'use strict';

  const EARTH_R_KM = 6371;
  const TRACK_SPAN_S = 3600;  // ±1 h
  const TRACK_STEP_S = 30;
  const MARK_STEP_S = 300;     // 5-minute travel-time marks
  const MESH_HALF_DEG = 60;    // half length of the cross lines printed on the globe (orbit view)
  const HOME = { name: 'Munich', lon: 11.575, lat: 48.137, tz: 'Europe/Berlin' };
  const PASS_SEARCH_STEP_S = 20;
  const PASS_SEARCH_SPAN_S = 4 * 3600;
  const homeTime = new Intl.DateTimeFormat('de-DE', { timeZone: HOME.tz, hour: '2-digit', minute: '2-digit' });
  const DEG = 180 / Math.PI;

  // Aviation-chart palette: saturated blue sea, terrain-tinted land.
  const C = {
    face: '#1a1d1c',
    sea: [34, 78, 172],
    land: '#7a8f3c',              // fallback until the terrain texture has loaded
    coast: 'rgba(15, 25, 15, .55)',
    border: 'rgba(20, 20, 20, .5)',
    graticule: 'rgba(255, 255, 255, .16)',
    trackFuture: '#c8102e',
    trackPast: '#ff6b6b',
    mesh: 'rgba(255, 120, 120, .55)',
    sat: '#ffcc00',
    reticle: 'rgba(30, 30, 30, .55)',
    text: '#e9e3cf',
  };

  // ---------- DOM ----------
  const $ = (id) => document.getElementById(id);
  const canvas = $('display');
  const ctx = canvas.getContext('2d');
  const ui = {
    norad: $('norad'), load: $('load'), status: $('status'), readout: $('readout'),
    time: $('time'), timeOut: $('timeOut'), exag: $('exag'), exagOut: $('exagOut'),
    size: $('size'), modes: document.querySelectorAll('[data-mode]'), graticule: $('graticule'),
    borders: $('borders'), reticle: $('reticle'), night: $('night'),
  };

  // ---------- state ----------
  const state = {
    size: 480,
    mode: 'orbit',            // 'orbit': object centred, heading up · 'home': fixed view on HOME
    rotate: [-10, -45, 0],    // d3 rotation [λ, φ, γ]: view centre at (-λ, -φ), γ spins the view
    vel: [0, 0],              // pan inertia (deg / frame)
    manualUntil: 0,           // after a pan, hold the user's view until this time (ms)
    satrec: null,
    omm: null,
    now: null,                // {r, gmst} of the object at the displayed time
    ahead: null,              // same, a few seconds later: flight direction
    sat: null,                // geodetic {lon, lat, alt, vel} for the readouts
    track: [],                // [{t, r, gmst}] ±1 h, ECI
    nextPass: null,           // Date of the next range minimum to HOME
    marks: [],                // 5-min marks at absolute times: {t, lon, lat, alt, dir: {lon, lat, alt}}
    land: null,
    borders: null,
    tex: null,                // terrain texture {w, h, px: Uint8ClampedArray}, alpha 0 = water
    lights: null,             // city lights {w, h, px: Uint8Array}, one byte per texel
    dirty: true,
  };

  const projection = d3.geoOrthographic().clipAngle(90).precision(0.3);
  const path = d3.geoPath(projection, ctx);
  const graticule = d3.geoGraticule10();
  const sphere = { type: 'Sphere' };

  function layout() {
    const s = state.size;
    canvas.width = s;
    canvas.height = s;
    canvas.style.width = s + 'px';
    canvas.style.height = s + 'px';
    // Globe fills most of the round screen, leaving a narrow ring for the shadow.
    projection.scale(s * 0.43).translate([s / 2, s / 2]);
    state.dirty = true;
  }

  // ---------- data ----------
  async function loadMap() {
    const [land, countries] = await Promise.all([
      fetch('vendor/land-50m.json').then((r) => r.json()),
      fetch('vendor/countries-110m.json').then((r) => r.json()),
    ]);
    state.land = topojson.feature(land, land.objects.land);
    state.borders = topojson.mesh(countries, countries.objects.countries, (a, b) => a !== b);
    state.dirty = true;
  }

  // Natural Earth cross-blended hypsometric tints with shaded relief (equirectangular).
  // The source tints are pastel; push saturation and darken the lowlands toward the
  // deep greens / olive / brown of an aviation terrain chart. White = water.
  async function loadTexture() {
    const img = new Image();
    img.src = 'vendor/terrain-2048.jpg';
    await img.decode();
    const c = document.createElement('canvas');
    c.width = img.width; c.height = img.height;
    const g = c.getContext('2d');
    g.drawImage(img, 0, 0);
    const data = g.getImageData(0, 0, c.width, c.height);
    const px = data.data;
    for (let i = 0; i < px.length; i += 4) {
      const r = px[i], gg = px[i + 1], b = px[i + 2];
      if (r > 242 && gg > 242 && b > 242) { px[i + 3] = 0; continue; }
      const m = (r + gg + b) / 3;
      for (let k = 0; k < 3; k++) {
        const v = (m + (px[i + k] - m) * 2.4) / 255;
        px[i + k] = 255 * Math.pow(Math.min(1, Math.max(0, v)), 1.35);
      }
    }
    state.tex = { w: c.width, h: c.height, px };
    state.dirty = true;
  }

  // NASA "Earth at Night" (Suomi NPP VIIRS, 2012), equirectangular. The image has a dim
  // blue land/sea background; the warm city lights stand out in the red channel.
  async function loadNightLights() {
    const img = new Image();
    img.src = 'vendor/night-2048.jpg';
    await img.decode();
    const c = document.createElement('canvas');
    c.width = img.width; c.height = img.height;
    const g = c.getContext('2d');
    g.drawImage(img, 0, 0);
    const src = g.getImageData(0, 0, c.width, c.height).data;
    const px = new Uint8Array(c.width * c.height);
    for (let i = 0; i < px.length; i++) {
      const v = Math.max(0, (src[i * 4] - 45) / 170);
      px[i] = 255 * Math.min(1, Math.pow(v, 1.1));
    }
    state.lights = { w: c.width, h: c.height, px };
    state.dirty = true;
  }

  const smoothstep = (e0, e1, x) => {
    const t = Math.min(1, Math.max(0, (x - e0) / (e1 - e0)));
    return t * t * (3 - 2 * t);
  };

  // Per-pixel inverse orthographic projection onto the globe. One pass fills three buffers:
  // terrain colour, night darkening (alpha) and city lights (added on top).
  const bufs = {};
  function buffer(name, s) {
    let b = bufs[name];
    if (!b || b.width !== s) {
      b = bufs[name] = document.createElement('canvas');
      b.width = b.height = s;
    }
    return b;
  }

  function renderGlobePixels(withNight) {
    const s = state.size, r = projection.scale();
    const [cx, cy] = projection.translate();
    const terrain = buffer('terrain', s), shade = buffer('shade', s), glow = buffer('glow', s);
    const out = terrain.getContext('2d').createImageData(s, s), o = out.data;
    const shOut = shade.getContext('2d').createImageData(s, s), so = shOut.data;
    const glOut = glow.getContext('2d').createImageData(s, s), go = glOut.data;
    const tex = state.tex, lights = withNight ? state.lights : null;
    // Sun direction in Earth-fixed coordinates.
    const [slon, slat] = subsolarPoint(simTime()).map((d) => d / DEG);
    const sx = Math.cos(slat) * Math.cos(slon), sy = Math.cos(slat) * Math.sin(slon), sz = Math.sin(slat);
    const lam = state.rotate[0] / DEG, phi = state.rotate[1] / DEG, gam = (state.rotate[2] || 0) / DEG;
    const cp = Math.cos(phi), sp = Math.sin(phi), cg = Math.cos(gam), sg2 = Math.sin(gam);
    const [sr, sg, sb] = C.sea;
    const x0 = Math.max(0, Math.floor(cx - r)), x1 = Math.min(s, Math.ceil(cx + r));
    const y0 = Math.max(0, Math.floor(cy - r)), y1 = Math.min(s, Math.ceil(cy + r));
    for (let py = y0; py < y1; py++) {
      const Ys = (cy - py - 0.5) / r;
      for (let pxl = x0; pxl < x1; pxl++) {
        const Xs = (pxl + 0.5 - cx) / r;
        const d = Xs * Xs + Ys * Ys;
        if (d > 1) continue;
        const Z = Math.sqrt(1 - d);
        // Undo d3's [λ, φ, γ] rotation: spin back by γ, tilt back by φ, shift longitude by λ.
        const X = Xs * cg + Ys * sg2;
        const Y = Ys * cg - Xs * sg2;
        const xr = Z * cp + Y * sp;
        const zr = Y * cp - Z * sp;
        const lon = Math.atan2(X, xr) - lam;
        const lat = Math.asin(Math.max(-1, Math.min(1, zr)));
        const fu = ((lon / (2 * Math.PI) + 0.5) % 1 + 1) % 1, fv = Math.min(0.9999, 0.5 - lat / Math.PI);
        const oi = (py * s + pxl) * 4;
        if (tex) {
          const ti = (Math.floor(fv * tex.h) * tex.w + Math.floor(fu * tex.w)) * 4, px = tex.px;
          if (px[ti + 3] === 0) { o[oi] = sr; o[oi + 1] = sg; o[oi + 2] = sb; }
          else { o[oi] = px[ti]; o[oi + 1] = px[ti + 1]; o[oi + 2] = px[ti + 2]; }
          o[oi + 3] = 255;
        }
        if (withNight) {
          // Sun elevation → night factor, with a soft twilight band (~+6° … −7°).
          const cl = Math.cos(lat);
          const sunDot = cl * Math.cos(lon) * sx + cl * Math.sin(lon) * sy + Math.sin(lat) * sz;
          const n = smoothstep(0.1, -0.12, sunDot);
          if (n > 0) {
            so[oi] = 4; so[oi + 1] = 8; so[oi + 2] = 28; so[oi + 3] = 200 * n;
            if (lights) {
              const L = lights.px[Math.floor(fv * lights.h) * lights.w + Math.floor(fu * lights.w)] * n;
              go[oi] = L; go[oi + 1] = L * 0.8; go[oi + 2] = L * 0.45; go[oi + 3] = 255;
            }
          }
        }
      }
    }
    terrain.getContext('2d').putImageData(out, 0, 0);
    shade.getContext('2d').putImageData(shOut, 0, 0);
    glow.getContext('2d').putImageData(glOut, 0, 0);
    return { terrain, shade, glow };
  }

  async function loadSatellite(catnr) {
    ui.status.className = 'status';
    ui.status.textContent = `Fetching elements for ${catnr}…`;
    try {
      const res = await fetch(`/api/gp?catnr=${catnr}`);
      const data = await res.json();
      if (!res.ok || !Array.isArray(data) || !data.length) {
        throw new Error(data.error || 'No element set found');
      }
      state.omm = data[0];
      // OMM/JSON instead of TLE: TLE lines cannot hold 6-digit catalog numbers.
      state.satrec = satellite.json2satrec(state.omm);
      ui.status.textContent = `Elements epoch ${state.omm.EPOCH.replace('T', ' ').slice(0, 19)} UTC`;
      updateTrack();
      updateNow();
      state.rotate = targetRotation() || state.rotate;
    } catch (e) {
      state.satrec = null;
      state.sat = null;
      state.now = state.ahead = null;
      state.track = [];
      ui.status.className = 'status error';
      ui.status.textContent = `Could not load ${catnr}: ${e.message}`;
    }
    state.dirty = true;
  }

  // ---------- orbit ----------
  function simTime() {
    return new Date(Date.now() + Number(ui.time.value) * 60000);
  }

  function propagateEci(date) {
    const pv = satellite.propagate(state.satrec, date);
    if (!pv || !pv.position) return null;
    const { x, y, z } = pv.position;
    return { r: [x, y, z], v: pv.velocity, gmst: satellite.gstime(date) };
  }

  // Geocentric lon/lat/alt of an ECI position, seen in the Earth frame of sidereal angle gmst.
  // Using the point's own gmst gives the ground-relative path; using one common gmst for all
  // points freezes the Earth and gives the inertial orbit (a great circle).
  function geoOf(r, gmst) {
    const [x, y, z] = r;
    let lon = (Math.atan2(y, x) - gmst) * DEG;
    lon = ((lon + 540) % 360) - 180;
    return { lon, lat: Math.atan2(z, Math.hypot(x, y)) * DEG, alt: Math.hypot(x, y, z) - EARTH_R_KM };
  }

  // Frame in which the track and the flight direction are drawn.
  function frameGmst(pt) {
    return state.mode === 'orbit' ? state.now.gmst : pt.gmst;
  }

  // Runs every frame: object position at the displayed time.
  function updateNow() {
    if (!state.satrec) return;
    const t = simTime();
    state.now = propagateEci(t);
    state.ahead = propagateEci(new Date(t.getTime() + 5000));
  }

  // Runs once a second: ±1 h track and the readouts.
  function updateTrack() {
    if (!state.satrec) return;
    const now = simTime();
    const track = [];
    for (let dt = -TRACK_SPAN_S; dt <= TRACK_SPAN_S; dt += TRACK_STEP_S) {
      const p = propagateEci(new Date(now.getTime() + dt * 1000));
      if (p) track.push({ t: dt, ...p });
    }
    state.track = track;

    // 5-minute marks pinned to absolute times (so they don't jump as "now" advances),
    // in the Earth-fixed frame: the ground point the object passes at that moment.
    const marks = [];
    const step = MARK_STEP_S * 1000, t0 = now.getTime();
    for (let ms = Math.ceil((t0 - TRACK_SPAN_S * 1000) / step) * step; ms <= t0 + TRACK_SPAN_S * 1000; ms += step) {
      const a = propagateEci(new Date(ms)), b = propagateEci(new Date(ms + 10000));
      if (a && b) marks.push({ t: (ms - t0) / 1000, ...geoOf(a.r, a.gmst), dir: geoOf(b.r, b.gmst) });
    }
    state.marks = marks;
    state.nextPass = findNextPass(now);
    const cur = propagateEci(now);
    if (cur) {
      const g = satellite.eciToGeodetic({ x: cur.r[0], y: cur.r[1], z: cur.r[2] }, cur.gmst);
      state.sat = {
        lon: g.longitude * DEG, lat: g.latitude * DEG, alt: g.height,
        vel: Math.hypot(cur.v.x, cur.v.y, cur.v.z),
      };
    }
    updateReadout();
    state.dirty = true;
  }

  // Straight-line distance (km) from HOME at ground level to the object.
  let homeEcf = null;
  function rangeToHome(eci) {
    if (!homeEcf) homeEcf = satellite.geodeticToEcf({ longitude: HOME.lon / DEG, latitude: HOME.lat / DEG, height: 0 });
    const e = satellite.eciToEcf({ x: eci.r[0], y: eci.r[1], z: eci.r[2] }, eci.gmst);
    return Math.hypot(e.x - homeEcf.x, e.y - homeEcf.y, e.z - homeEcf.z);
  }

  // Next pass = first local minimum of the range to HOME after `from`:
  // coarse scan, then ternary-search refinement to ~1 s.
  function findNextPass(from) {
    const t0 = from.getTime();
    const rangeAt = (ms) => { const p = propagateEci(new Date(ms)); return p ? rangeToHome(p) : Infinity; };
    let prev = rangeAt(t0), cur = rangeAt(t0 + PASS_SEARCH_STEP_S * 1000);
    for (let dt = 2 * PASS_SEARCH_STEP_S; dt <= PASS_SEARCH_SPAN_S; dt += PASS_SEARCH_STEP_S) {
      const next = rangeAt(t0 + dt * 1000);
      if (cur <= prev && cur < next) {
        let lo = t0 + (dt - 2 * PASS_SEARCH_STEP_S) * 1000, hi = t0 + dt * 1000;
        while (hi - lo > 1000) {
          const m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
          if (rangeAt(m1) < rangeAt(m2)) hi = m2; else lo = m1;
        }
        return new Date((lo + hi) / 2);
      }
      prev = cur; cur = next;
    }
    return null;
  }

  // View orientation the current mode wants.
  function targetRotation() {
    if (state.mode === 'home') return [-HOME.lon, -HOME.lat, 0];
    if (!state.now || !state.ahead) return null;
    const p = geoOf(state.now.r, state.now.gmst);
    const q = geoOf(state.ahead.r, state.now.gmst);
    // Centre on the object, then spin the view so the flight direction points up.
    const saved = projection.rotate();
    projection.rotate([-p.lon, -p.lat, 0]);
    const a = project3d(q.lon, q.lat, q.alt), o = project3d(p.lon, p.lat, p.alt);
    projection.rotate(saved);
    const heading = Math.atan2(a.x - o.x, -(a.y - o.y)) * DEG;   // clockwise from screen-up
    return [-p.lon, -p.lat, heading];
  }

  // Project a point at altitude: returns screen x/y plus whether the Earth hides it.
  function project3d(lon, lat, altKm) {
    const [l, p] = d3.geoRotation(projection.rotate())([lon, lat]).map((d) => d / DEG);
    const r = projection.scale();
    const k = r * (1 + (Number(ui.exag.value) * altKm) / EARTH_R_KM);
    const [cx, cy] = projection.translate();
    const ex = Math.cos(p) * Math.sin(l);
    const ey = Math.sin(p);
    const ez = Math.cos(p) * Math.cos(l);
    const x = cx + k * ex;
    const y = cy - k * ey;
    const hidden = ez < 0 && k * Math.hypot(ex, ey) < r;
    // v: view-space position (x right, y up, z toward the viewer), in screen pixels.
    return { x, y, hidden, front: ez >= 0, v: [k * ex, k * ey, k * ez] };
  }

  // Approximate subsolar point (good to ~0.5°, plenty for shading).
  function subsolarPoint(date) {
    const d = (date.getTime() - Date.UTC(2000, 0, 1, 12)) / 864e5;
    const g = (357.529 + 0.98560028 * d) / DEG;
    const q = 280.459 + 0.98564736 * d;
    const L = (q + 1.915 * Math.sin(g) + 0.020 * Math.sin(2 * g)) / DEG;
    const e = (23.439 - 0.00000036 * d) / DEG;
    const ra = Math.atan2(Math.cos(e) * Math.sin(L), Math.cos(L)) * DEG;
    const dec = Math.asin(Math.sin(e) * Math.sin(L)) * DEG;
    const gmst = satellite.gstime(date) * DEG;
    let lon = ra - gmst;
    lon = ((lon + 540) % 360) - 180;
    return [lon, dec];
  }

  // ---------- rendering ----------
  function draw() {
    const s = state.size;
    const r = projection.scale();
    const [cx, cy] = projection.translate();
    projection.rotate(state.rotate);

    // Instrument face behind the globe.
    ctx.clearRect(0, 0, s, s);
    const face = ctx.createRadialGradient(cx, cy, r * 0.9, cx, cy, s * 0.5);
    face.addColorStop(0, '#2a2e2c');
    face.addColorStop(1, C.face);
    ctx.fillStyle = face;
    ctx.fillRect(0, 0, s, s);

    // Cast shadow, offset down-right, so the globe floats above the face.
    ctx.save();
    const sh = ctx.createRadialGradient(cx + r * 0.05, cy + r * 0.07, r * 0.9, cx + r * 0.05, cy + r * 0.07, r * 1.12);
    sh.addColorStop(0, 'rgba(0,0,0,.75)');
    sh.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = sh;
    ctx.beginPath();
    ctx.arc(cx + r * 0.05, cy + r * 0.07, r * 1.12, 0, 2 * Math.PI);
    ctx.fill();
    ctx.restore();

    // Sea + terrain.
    ctx.save();
    ctx.beginPath(); path(sphere); ctx.clip();
    ctx.fillStyle = `rgb(${C.sea})`; ctx.fill();
    const night = ui.night.checked;
    const px = state.tex || night ? renderGlobePixels(night) : null;
    if (state.land) {
      // Terrain only inside the vector coastline keeps the sea a clean, flat blue.
      ctx.save();
      ctx.beginPath(); path(state.land); ctx.clip();
      if (state.tex) ctx.drawImage(px.terrain, 0, 0);
      else { ctx.fillStyle = C.land; ctx.fill(); }
      ctx.restore();
    }

    if (state.land) {
      ctx.lineJoin = 'round';
      ctx.beginPath(); path(state.land);
      ctx.strokeStyle = C.coast; ctx.lineWidth = Math.max(0.6, s / 800); ctx.stroke();
    }

    if (ui.borders.checked && state.borders) {
      ctx.beginPath(); path(state.borders);
      ctx.setLineDash([3, 2]);
      ctx.strokeStyle = C.border; ctx.lineWidth = 0.7; ctx.stroke();
      ctx.setLineDash([]);
    }

    if (ui.graticule.checked) {
      ctx.beginPath(); path(graticule);
      ctx.strokeStyle = C.graticule; ctx.lineWidth = 0.6; ctx.stroke();
    }

    if (night) {
      // Darken the night side, then add the city lights on top.
      ctx.drawImage(px.shade, 0, 0);
      ctx.globalCompositeOperation = 'lighter';
      ctx.drawImage(px.glow, 0, 0);
      ctx.globalCompositeOperation = 'source-over';
    }

    if (state.mode === 'orbit') drawMarkMesh();

    drawGlobeShading(cx, cy, r);
    ctx.restore();

    if (ui.reticle.checked) drawReticle(cx, cy, r);
    drawHome();
    drawTrack();
    if (state.mode === 'home') drawMarkTicks();
    drawSatellite();
    drawHud();
  }

  // Soviet-instrument style lighting: lit from upper left, dark limb, glass glint.
  function drawGlobeShading(cx, cy, r) {
    const lx = cx - r * 0.35, ly = cy - r * 0.4;
    const light = ctx.createRadialGradient(lx, ly, 0, lx, ly, r * 1.55);
    light.addColorStop(0, 'rgba(255,255,240,.28)');
    light.addColorStop(0.35, 'rgba(255,255,240,0)');
    light.addColorStop(0.7, 'rgba(0,0,0,.18)');
    light.addColorStop(1, 'rgba(0,0,0,.7)');
    ctx.fillStyle = light;
    ctx.fillRect(cx - r, cy - r, 2 * r, 2 * r);

    const limb = ctx.createRadialGradient(cx, cy, r * 0.72, cx, cy, r);
    limb.addColorStop(0, 'rgba(0,0,0,0)');
    limb.addColorStop(1, 'rgba(0,0,0,.45)');
    ctx.fillStyle = limb;
    ctx.fillRect(cx - r, cy - r, 2 * r, 2 * r);

    // Soft glass reflection.
    ctx.save();
    ctx.translate(cx - r * 0.38, cy - r * 0.48);
    ctx.rotate(-0.6);
    const glint = ctx.createRadialGradient(0, 0, 0, 0, 0, r * 0.32);
    glint.addColorStop(0, 'rgba(255,255,255,.35)');
    glint.addColorStop(1, 'rgba(255,255,255,0)');
    ctx.scale(1, 0.55);
    ctx.fillStyle = glint;
    ctx.beginPath(); ctx.arc(0, 0, r * 0.32, 0, 2 * Math.PI); ctx.fill();
    ctx.restore();
  }

  // Home location marker: blue dot with white rim, like an ownship symbol on a chart.
  function drawHome() {
    const p = project3d(HOME.lon, HOME.lat, 0);
    if (!p.front) return;
    const s = state.size, rr = s / 150;
    ctx.save();
    ctx.fillStyle = '#1e7bff';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = Math.max(1.2, s / 360);
    ctx.beginPath(); ctx.arc(p.x, p.y, rr, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.font = `600 ${Math.round(s / 48)}px "Courier New", monospace`;
    ctx.textBaseline = 'middle';
    ctx.lineWidth = 3;
    ctx.strokeStyle = 'rgba(0,0,0,.6)';
    ctx.strokeText(HOME.name.toUpperCase(), p.x + rr * 2, p.y);
    ctx.fillStyle = '#ffffff';
    ctx.fillText(HOME.name.toUpperCase(), p.x + rr * 2, p.y);
    ctx.restore();
  }

  function drawReticle(cx, cy, r) {
    ctx.save();
    ctx.strokeStyle = C.reticle;
    ctx.lineWidth = 1;
    const a = r * 0.06, gap = r * 0.02;
    ctx.beginPath();
    ctx.moveTo(cx - a, cy); ctx.lineTo(cx - gap, cy);
    ctx.moveTo(cx + gap, cy); ctx.lineTo(cx + a, cy);
    ctx.moveTo(cx, cy - a); ctx.lineTo(cx, cy - gap);
    ctx.moveTo(cx, cy + gap); ctx.lineTo(cx, cy + a);
    ctx.stroke();
    ctx.restore();
  }

  function drawTrack() {
    const tr = state.track;
    if (tr.length < 2) return;
    const pts = tr.map((p) => {
      const g = geoOf(p.r, frameGmst(p));
      return { t: p.t, ...project3d(g.lon, g.lat, g.alt) };
    });
    const s = state.size;
    ctx.save();
    ctx.lineCap = 'round';
    for (let i = 1; i < pts.length; i++) {
      const a = pts[i - 1], b = pts[i];
      const past = b.t <= 0;
      const fade = 1 - 0.65 * (Math.abs(b.t) / TRACK_SPAN_S);
      // Behind the Earth: faint dashed line. In view: solid, past thinner than future.
      const hidden = a.hidden || b.hidden;
      ctx.globalAlpha = hidden ? 0.45 * fade : fade;
      ctx.strokeStyle = past ? C.trackPast : C.trackFuture;
      ctx.lineWidth = hidden ? s / 400 : past ? s / 300 : s / 200;
      ctx.setLineDash(hidden ? [s / 100, s / 120] : []);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.restore();
  }

  // Great-circle helpers (degrees).
  function azimuth(lon1, lat1, lon2, lat2) {
    const p1 = lat1 / DEG, p2 = lat2 / DEG, dl = (lon2 - lon1) / DEG;
    return Math.atan2(Math.sin(dl) * Math.cos(p2),
      Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl)) * DEG;
  }
  function destination(lon, lat, azDeg, distDeg) {
    const p1 = lat / DEG, th = azDeg / DEG, d = distDeg / DEG;
    const p2 = Math.asin(Math.sin(p1) * Math.cos(d) + Math.cos(p1) * Math.sin(d) * Math.cos(th));
    const l2 = lon / DEG + Math.atan2(Math.sin(th) * Math.sin(d) * Math.cos(p1),
      Math.cos(d) - Math.sin(p1) * Math.sin(p2));
    return [l2 * DEG, p2 * DEG];
  }

  // Orbit view: lines across the ground track at each 5-min mark, printed on the globe
  // (great-circle arcs, so they curve with the surface and turn with the Earth).
  function drawMarkMesh() {
    const s = state.size;
    ctx.save();
    ctx.strokeStyle = C.mesh;
    ctx.lineWidth = Math.max(0.7, s / 600);
    for (const m of state.marks) {
      const az = azimuth(m.lon, m.lat, m.dir.lon, m.dir.lat) + 90;
      const coords = [];
      for (let d = -MESH_HALF_DEG; d <= MESH_HALF_DEG; d += 5) coords.push(destination(m.lon, m.lat, az, d));
      ctx.globalAlpha = 1 - 0.5 * Math.abs(m.t) / TRACK_SPAN_S;
      ctx.beginPath(); path({ type: 'LineString', coordinates: coords }); ctx.stroke();
    }
    ctx.restore();
  }

  // Munich view: short ticks across the drawn track at each 5-min mark.
  function drawMarkTicks() {
    const s = state.size, half = s / 90;
    ctx.save();
    ctx.lineCap = 'round';
    for (const m of state.marks) {
      const p = project3d(m.lon, m.lat, m.alt), q = project3d(m.dir.lon, m.dir.lat, m.dir.alt);
      const dx = q.x - p.x, dy = q.y - p.y, n = Math.hypot(dx, dy);
      if (n < 1e-6) continue;
      const nx = -dy / n * half, ny = dx / n * half;   // screen-space perpendicular
      const fade = 1 - 0.65 * Math.abs(m.t) / TRACK_SPAN_S;
      ctx.globalAlpha = p.hidden ? 0.45 * fade : fade;
      ctx.strokeStyle = m.t > 0 ? C.trackFuture : C.trackPast;
      ctx.lineWidth = p.hidden ? s / 400 : s / 240;
      ctx.beginPath(); ctx.moveTo(p.x - nx, p.y - ny); ctx.lineTo(p.x + nx, p.y + ny); ctx.stroke();
    }
    ctx.restore();
  }

  function drawSatellite() {
    if (!state.now) return;
    const s = state.size;
    const sat = geoOf(state.now.r, state.now.gmst);
    const p = project3d(sat.lon, sat.lat, sat.alt);
    const g = project3d(sat.lon, sat.lat, 0);
    ctx.save();
    if (p.hidden) ctx.globalAlpha = 0.3;   // obstructed by the Earth

    // Drop line to the sub-satellite point.
    if (g.front) {
      ctx.strokeStyle = 'rgba(30,30,30,.6)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(g.x, g.y); ctx.lineTo(p.x, p.y); ctx.stroke();
      ctx.fillStyle = 'rgba(30,30,30,.7)';
      ctx.beginPath(); ctx.arc(g.x, g.y, s / 300, 0, 2 * Math.PI); ctx.fill();
    }

    // Upper stage, nose pointing along the flight direction.
    let dir = [1, 0, 0];
    if (state.ahead) {
      const ahead = geoOf(state.ahead.r, frameGmst(state.ahead));
      const q = project3d(ahead.lon, ahead.lat, ahead.alt);
      const d = [q.v[0] - p.v[0], q.v[1] - p.v[1], q.v[2] - p.v[2]];
      const n = Math.hypot(...d) || 1;
      dir = d.map((c) => c / n);
    }
    drawStage(p.x, p.y, dir, s / 13);
    ctx.restore();
  }

  // Simplified rocket upper stage: vacuum nozzle, aft dome, tank body, blue
  // separation ring, conical payload adapter and a small payload. Built from
  // lit frustums so it reads as 3D at any orientation.
  // Segments run tail → nose: [t0, r0, t1, r1, colors]; t in body lengths, r in body radii.
  const STAGE = [
    [-0.80, 0.80, -0.50, 0.30, { lit: '#a3a3a3', base: '#4a4a4a', dark: '#121212', cap: '#0a0a0a' }], // nozzle
    [-0.50, 0.62, -0.42, 1.00, { lit: '#8d9297', base: '#50555a', dark: '#1e2124', cap: '#3a3e42' }], // aft dome
    [-0.42, 1.00, 0.30, 1.00, { lit: '#ffffff', base: '#d8dce0', dark: '#646b72', cap: '#c4c9cd' }],  // tank
    [0.30, 1.04, 0.37, 1.04, { lit: '#7cb9ff', base: '#1f6fd1', dark: '#0a2b5c', cap: '#1a5bb0' }],   // ring
    [0.37, 0.92, 0.48, 0.55, { lit: '#7b8086', base: '#3d4247', dark: '#14171a', cap: '#2a2e33' }],   // adapter
    [0.48, 0.72, 0.68, 0.72, { lit: '#f0d27a', base: '#b08a2e', dark: '#43350e', cap: '#8a6c22' }],   // payload
  ];
  const STAGE_RADIUS = 0.17;               // body radius / body length
  const LIGHT = [-0.55, -0.83];            // screen direction toward the light (upper left)

  function drawStage(x, y, dir, len) {
    const [dx, dy, dz] = dir;              // view space, y up, z toward viewer
    const theta = Math.atan2(-dy, dx);
    const f = Math.max(0.12, Math.hypot(dx, dy));   // foreshortening of the axis
    const capK = Math.max(0.06, Math.abs(dz));      // end caps seen as ellipses
    const R = len * STAGE_RADIUS;

    // Which side of the body faces the light, in the rotated frame.
    const ly = LIGHT[0] * -Math.sin(theta) + LIGHT[1] * Math.cos(theta);
    const sign = ly >= 0 ? 1 : -1;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(theta);
    ctx.lineJoin = 'round';
    ctx.strokeStyle = 'rgba(0,0,0,.55)';
    ctx.lineWidth = 0.7;

    // Painter's order: far end first. Nose toward the viewer → start at the tail.
    const noseNear = dz >= 0;
    const order = noseNear ? STAGE : [...STAGE].reverse();
    for (const [t0, r0, t1, r1, col] of order) {
      const a0 = t0 * len * f, a1 = t1 * len * f;
      const rr0 = r0 * R, rr1 = r1 * R, rm = Math.max(rr0, rr1);

      const g = ctx.createLinearGradient(0, -rm * sign, 0, rm * sign);
      g.addColorStop(0, col.dark);
      g.addColorStop(0.55, col.base);
      g.addColorStop(0.85, col.lit);
      g.addColorStop(1, col.base);
      ctx.fillStyle = g;

      // Side silhouette including the far cap's half-ellipse, so the outline stays closed.
      const farA = noseNear ? a0 : a1, farR = noseNear ? rr0 : rr1;
      const nearA = noseNear ? a1 : a0, nearR = noseNear ? rr1 : rr0;
      const ex = (r) => Math.max(0.3, r * capK * (noseNear ? 1 : -1));
      ctx.beginPath();
      ctx.moveTo(nearA, -nearR);
      ctx.lineTo(farA, -farR);
      ctx.ellipse(farA, 0, Math.abs(ex(farR)), farR, 0, -Math.PI / 2, Math.PI / 2, !noseNear);
      ctx.lineTo(nearA, nearR);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Near cap.
      ctx.fillStyle = col.cap;
      ctx.beginPath();
      ctx.ellipse(nearA, 0, Math.abs(ex(nearR)), nearR, 0, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
    }
    ctx.restore();
  }

  // Minimal on-screen readout, laid out for a round screen (centered, near top/bottom).
  function drawHud() {
    const s = state.size;
    const fs = Math.round(s / 34);
    ctx.save();
    ctx.font = `600 ${fs}px "Courier New", monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    const pill = (text, y) => {
      const w = ctx.measureText(text).width + fs * 1.4;
      const h = fs * 1.6;
      ctx.fillStyle = 'rgba(20,22,22,.72)';
      ctx.beginPath();
      ctx.roundRect(s / 2 - w / 2, y - h / 2, w, h, h / 2);
      ctx.fill();
      ctx.fillStyle = C.text;
      ctx.fillText(text, s / 2, y);
    };

    const t = simTime();
    pill(t.toISOString().slice(11, 19) + ' UTC', s * 0.1);
    if (state.sat && state.now) {
      const { lat, lon, alt } = state.sat;
      const ll = `${Math.abs(lat).toFixed(1)}°${lat >= 0 ? 'N' : 'S'} ${Math.abs(lon).toFixed(1)}°${lon >= 0 ? 'E' : 'W'}`;
      const lines = [
        `${ll}  ${alt.toFixed(0)} km`,
        `Distance: ${Math.round(rangeToHome(state.now))} km`,
        `Next Pass: ${state.nextPass ? homeTime.format(state.nextPass) : '--:--'}`,
      ];
      // Stacked block at the bottom of the round screen; last line where the chord is still wide.
      const lh = fs * 1.3, yLast = s * 0.905, pad = fs * 0.7;
      const w = Math.max(...lines.map((l) => ctx.measureText(l).width)) + pad * 2;
      const h = lh * lines.length + pad * 0.6;
      ctx.fillStyle = 'rgba(20,22,22,.72)';
      ctx.beginPath();
      ctx.roundRect(s / 2 - w / 2, yLast + lh / 2 + pad * 0.3 - h, w, h, fs * 0.8);
      ctx.fill();
      ctx.fillStyle = C.text;
      lines.forEach((l, i) => ctx.fillText(l, s / 2, yLast - (lines.length - 1 - i) * lh));
    }
    ctx.restore();
  }

  function updateReadout() {
    const sat = state.sat, omm = state.omm;
    if (!sat || !omm) { ui.readout.innerHTML = ''; return; }
    const period = 1440 / omm.MEAN_MOTION;
    const rows = [
      ['Object', omm.OBJECT_NAME],
      ['Intl. designator', omm.OBJECT_ID],
      ['Latitude', sat.lat.toFixed(3) + '°'],
      ['Longitude', sat.lon.toFixed(3) + '°'],
      ['Altitude', sat.alt.toFixed(1) + ' km'],
      ['Speed', sat.vel.toFixed(2) + ' km/s'],
      ['Inclination', omm.INCLINATION.toFixed(2) + '°'],
      ['Period', period.toFixed(1) + ' min'],
      ['Next pass (' + HOME.name + ')', state.nextPass ? homeTime.format(state.nextPass) : '–'],
    ];
    ui.readout.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  }

  // ---------- interaction ----------
  // Ease the view toward the mode's orientation (short way round for λ and γ).
  function easeToTarget() {
    const t = targetRotation();
    if (!t) return;
    const wrap = (d) => ((d + 540) % 360) - 180;
    const d = [wrap(t[0] - state.rotate[0]), t[1] - state.rotate[1], wrap(t[2] - (state.rotate[2] || 0))];
    if (d.every((v) => Math.abs(v) < 0.005)) { state.rotate = t; return; }
    const k = d.every((v) => Math.abs(v) < 0.5) ? 1 : 0.12;   // glide in, then lock on
    state.rotate = [wrap(state.rotate[0] + d[0] * k), state.rotate[1] + d[1] * k, wrap((state.rotate[2] || 0) + d[2] * k)];
    state.dirty = true;
  }

  function setMode(mode) {
    state.mode = mode;
    state.manualUntil = 0;
    for (const b of ui.modes) b.classList.toggle('active', b.dataset.mode === mode);
    state.dirty = true;
  }

  let drag = null;
  canvas.addEventListener('pointerdown', (e) => {
    canvas.setPointerCapture(e.pointerId);
    canvas.classList.add('dragging');
    drag = { x: e.clientX, y: e.clientY };
    state.vel = [0, 0];
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag) return;
    const k = DEG / projection.scale();   // one globe radius ≈ 57°
    const dx = (e.clientX - drag.x) * k;
    const dy = (e.clientY - drag.y) * k;
    drag = { x: e.clientX, y: e.clientY };
    rotateBy(dx, dy);
    state.vel = [dx, dy];
  });
  const endDrag = () => {
    drag = null;
    canvas.classList.remove('dragging');
    state.manualUntil = performance.now() + 4000;   // then glide back to the mode's view
  };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);
  canvas.addEventListener('dblclick', () => { state.manualUntil = 0; state.vel = [0, 0]; });

  function rotateBy(dLon, dLat) {
    state.rotate[0] = ((state.rotate[0] + dLon + 540) % 360) - 180;
    state.rotate[1] = Math.max(-90, Math.min(90, state.rotate[1] - dLat));
    state.dirty = true;
  }

  // ---------- controls ----------
  ui.load.addEventListener('click', () => loadSatellite(Number(ui.norad.value)));
  ui.norad.addEventListener('keydown', (e) => { if (e.key === 'Enter') ui.load.click(); });
  ui.time.addEventListener('input', () => {
    const m = Number(ui.time.value);
    ui.timeOut.textContent = m === 0 ? 'now' : `${m > 0 ? '+' : '−'}${Math.floor(Math.abs(m) / 60)}h ${String(Math.abs(m) % 60).padStart(2, '0')}m`;
    updateTrack();
  });
  ui.exag.addEventListener('input', () => {
    ui.exagOut.textContent = Number(ui.exag.value).toFixed(1) + '×';
    state.dirty = true;
  });
  ui.size.addEventListener('change', () => { state.size = Number(ui.size.value); layout(); });
  for (const b of ui.modes) b.addEventListener('click', () => setMode(b.dataset.mode));
  window.addEventListener('keydown', (e) => {
    if (e.key === 'm' && e.target === document.body) setMode(state.mode === 'orbit' ? 'home' : 'orbit');
  });
  for (const el of [ui.graticule, ui.borders, ui.reticle, ui.night]) {
    el.addEventListener('change', () => { state.dirty = true; });
  }

  // ---------- main loop ----------
  // Object position refreshes at 10 Hz (smooth enough for ~7 km/s, and a sane target
  // for the ESP32), the ±1 h track once a second.
  let lastTrack = 0, lastNow = 0;
  function frame(ts) {
    if (ts - lastTrack > 1000) { lastTrack = ts; updateTrack(); }
    if (ts - lastNow > 100) { lastNow = ts; updateNow(); state.dirty = true; }
    if (!drag && (Math.abs(state.vel[0]) > 0.01 || Math.abs(state.vel[1]) > 0.01)) {
      rotateBy(state.vel[0], state.vel[1]);
      state.vel = state.vel.map((v) => v * 0.92);
    }
    if (!drag && ts > state.manualUntil) easeToTarget();
    if (state.dirty) { state.dirty = false; draw(); }
    requestAnimationFrame(frame);
  }

  layout();
  setMode(state.mode);
  loadMap();
  loadTexture();
  loadNightLights();
  loadSatellite(Number(ui.norad.value));
  requestAnimationFrame(frame);
})();
