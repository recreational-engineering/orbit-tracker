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
    landRgb: [122, 143, 60],      // fallback until the terrain texture has loaded
    coastRgb: [15, 25, 15],
    border: 'rgba(20, 20, 20, .5)',
    graticule: 'rgba(255, 255, 255, .16)',
    trackFuture: '#c8102e',
    trackPast: '#ff6b6b',
    mesh: 'rgba(255, 120, 120, .55)',
    footprint: 'rgba(255, 226, 122, .9)',
    hud: '#4dff7a',                   // pilot HUD green
    hudShadow: 'rgba(0, 0, 0, .65)',
    behind: '#c4c8cc',
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
    menuOpen: false,          // on-screen view menu
    menuHits: [],             // [{x, y, w, h, mode}] option rects from the last draw
    topTab: null,             // {yCut, halfWidth} of the top HUD tab (tap = reset view)
    bottomTab: null,          // {yCut, halfWidth} of the bottom HUD tab (tap = menu)
    nextPass: null,           // Date of the next range minimum to HOME
    marks: [],                // 5-min marks at absolute times: {t, lon, lat, alt, dir: {lon, lat, alt}}
    land: null,
    borders: null,
    mask: null,               // land mask, Uint8Array MASK_W × MASK_H, 1 = land
    tex: null,                // terrain texture {w, h, px: Uint8ClampedArray}, alpha 0 = water
    lights: null,             // city lights {w, h, px: Uint8Array}, one byte per texel
    dirty: true,
  };

  // Two projections share the same disc on screen: orthographic globe (orbit / ground view)
  // and a north-pole-centred azimuthal equidistant map (flat view, whole Earth in the disc).
  const GLOBE_FRAC = 0.43;   // disc radius / display size
  const ortho = d3.geoOrthographic().clipAngle(90).precision(0.7);
  const azeq = d3.geoAzimuthalEquidistant().clipAngle(179.999).precision(0.7);
  let projection = ortho;
  const path = d3.geoPath(projection, ctx);
  const isFlat = () => state.mode === 'flat';
  const isPilot = () => state.mode === 'pilot';
  const viewKind = (m) => (m === 'flat' ? 'flat' : m === 'pilot' ? 'pilot' : 'globe');
  const PILOT_HALF_FOV = 55 / DEG;   // angle from view centre to the screen edge
  const discRadius = () => state.size * GLOBE_FRAC;
  const graticule = d3.geoGraticule10();
  const sphere = { type: 'Sphere' };

  function layout() {
    const s = state.size;
    canvas.width = s;
    canvas.height = s;
    canvas.style.width = s + 'px';
    canvas.style.height = s + 'px';
    // Globe fills most of the round screen, leaving a narrow ring for the shadow.
    ortho.scale(s * GLOBE_FRAC).translate([s / 2, s / 2]);
    azeq.scale(s * GLOBE_FRAC / Math.PI).translate([s / 2, s / 2]);   // 180° from the pole = rim
    state.dirty = true;
  }

  // ---------- data ----------
  async function loadMap() {
    const [land, countries] = await Promise.all([
      fetch('vendor/land-50m.json').then((r) => r.json()),
      fetch('vendor/countries-110m.json').then((r) => r.json()),
    ]);
    state.land = topojson.feature(land, land.objects.land);
    buildLandMask();
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
    const px = g.getImageData(0, 0, c.width, c.height).data;
    boostTerrain(px);
    state.tex = { w: c.width, h: c.height, px };
    state.dirty = true;
  }

  // Terrain colour boost (saturation ×2.4, gamma 1.35 via a 256-entry LUT); white → water
  // (alpha 0). Shared by the global texture and the high-res tiles. On the ESP this should
  // be baked into the tiles offline.
  const GAMMA_LUT = new Uint8Array(256).map((_, i) => Math.round(255 * Math.pow(i / 255, 1.35)));
  function boostTerrain(px) {
    for (let i = 0; i < px.length; i += 4) {
      const r = px[i], g = px[i + 1], b = px[i + 2];
      if (r > 242 && g > 242 && b > 242) { px[i + 3] = 0; continue; }
      const m = (r + g + b) / 3;
      px[i] = GAMMA_LUT[Math.max(0, Math.min(255, Math.round(m + (r - m) * 2.4)))];
      px[i + 1] = GAMMA_LUT[Math.max(0, Math.min(255, Math.round(m + (g - m) * 2.4)))];
      px[i + 2] = GAMMA_LUT[Math.max(0, Math.min(255, Math.round(m + (b - m) * 2.4)))];
    }
  }

  // ---------- high-res terrain tiles (pilot view only) ----------
  // Full HYP_50M_SR (10800×5400, ~3.7 km/texel) cut into 15×30 tiles of 360×360 px (12°×12°)
  // at web/tiles/hyp/<row>_<col>.jpg. Only tiles under the pilot camera are loaded, into a
  // fixed-size LRU cache; pixels whose ground footprint is coarse (toward the horizon) keep
  // using the 2048-wide global texture. ESP: HW JPEG decode → RGB565, 24 × 253 KB PSRAM.
  const HI = { T: 360, ROWS: 15, COLS: 30, CACHE: 24, MAX_LOADS: 3, FOOTPRINT_KM: 12 };
  const hiTiles = new Map();       // key → {px, used}
  const hiPending = new Set();
  let hiFrame = 0;
  const hiCanvas = document.createElement('canvas');
  hiCanvas.width = hiCanvas.height = HI.T;
  const hiCtx = hiCanvas.getContext('2d', { willReadFrequently: true });

  function requestHiTile(key) {
    if (hiPending.has(key) || hiPending.size >= HI.MAX_LOADS) return;
    hiPending.add(key);
    const img = new Image();
    img.onload = () => {
      hiCtx.drawImage(img, 0, 0);
      const px = hiCtx.getImageData(0, 0, HI.T, HI.T).data;
      boostTerrain(px);
      hiTiles.set(key, { px, used: hiFrame });
      if (hiTiles.size > HI.CACHE) {   // evict the least recently used tile
        let oldK = -1, oldU = Infinity;
        for (const [k, t] of hiTiles) if (t.used < oldU) { oldU = t.used; oldK = k; }
        hiTiles.delete(oldK);
      }
      hiPending.delete(key);
      state.dirty = true;
    };
    img.onerror = () => hiPending.delete(key);
    img.src = `tiles/hyp/${Math.floor(key / HI.COLS)}_${key % HI.COLS}.jpg`;
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

  // Land mask: land-50m rasterised once to an equirectangular byte grid (1 = land).
  // Replaces per-frame vector clipping, and its edges give the coastline for free.
  const MASK_W = 2048, MASK_H = 1024;
  function buildLandMask() {
    const c = document.createElement('canvas');
    c.width = MASK_W; c.height = MASK_H;
    const g = c.getContext('2d');
    const eq = d3.geoEquirectangular().scale(MASK_W / (2 * Math.PI)).translate([MASK_W / 2, MASK_H / 2]);
    g.fillStyle = '#fff';
    g.beginPath(); d3.geoPath(eq, g)(state.land); g.fill();
    const src = g.getImageData(0, 0, MASK_W, MASK_H).data;
    const m = new Uint8Array(MASK_W * MASK_H);
    for (let i = 0; i < m.length; i++) m[i] = src[i * 4] > 127 ? 1 : 0;
    state.mask = m;
    state.dirty = true;
  }

  // Per-pixel inverse orthographic projection onto the globe, one pass into one buffer:
  // sea / terrain (land mask), coastline (mask edges), night darkening and city lights.
  let globeBuf = null, globeImg = null;
  function renderGlobePixels(withNight) {
    const s = state.size, flat = isFlat(), cam = isPilot() ? state.cam : null;
    const r = cam ? s / 2 : discRadius();   // pilot view fills the whole round screen
    const [cx, cy] = projection.translate();
    if (!globeBuf || globeBuf.width !== s) {
      globeBuf = document.createElement('canvas');
      globeBuf.width = globeBuf.height = s;
      globeImg = globeBuf.getContext('2d').createImageData(s, s);
    }
    const o = globeImg.data;
    o.fill(0);
    const tex = state.tex, mask = state.mask, lights = withNight ? state.lights : null;
    // Sun direction in Earth-fixed coordinates.
    const [slon, slat] = subsolarPoint(simTime()).map((d) => d / DEG);
    const sx = Math.cos(slat) * Math.cos(slon), sy = Math.cos(slat) * Math.sin(slon), sz = Math.sin(slat);
    const lam = state.rotate[0] / DEG, phi = state.rotate[1] / DEG, gam = (state.rotate[2] || 0) / DEG;
    const cp = Math.cos(phi), sp = Math.sin(phi), cg = Math.cos(gam), sg2 = Math.sin(gam);
    const cl0 = Math.cos(lam), sl0 = Math.sin(lam);
    // Pilot camera: ray = f + X·tanH·rt + Y·tanH·u; Earth is a sphere of radius EARTH_R_KM.
    const tanH = Math.tan(PILOT_HALF_FOV), pixAng = 2 * tanH / s;
    const gm = cam ? cam.gmst : 0, cgm = Math.cos(gm), sgm = Math.sin(gm);
    const RE = EARTH_R_KM, cc2 = cam ? cam.c[0] ** 2 + cam.c[1] ** 2 + cam.c[2] ** 2 : 0;
    const tHorizon = Math.sqrt(Math.max(0, cc2 - RE * RE));
    const grat = cam && ui.graticule.checked;
    const hiMaxT = cam ? HI.FOOTPRINT_KM / pixAng : 0;   // use tiles while hitT < this
    let hiKey = -1, hiTile = null;
    if (cam) hiFrame++;
    const [sr, sg, sb] = C.sea, [lr, lg, lb] = C.landRgb, [kr, kg, kb] = C.coastRgb;
    const x0 = Math.max(0, Math.floor(cx - r)), x1 = Math.min(s, Math.ceil(cx + r));
    const y0 = Math.max(0, Math.floor(cy - r)), y1 = Math.min(s, Math.ceil(cy + r));
    for (let py = y0; py < y1; py++) {
      const Ys = (cy - py - 0.5) / r;
      for (let pxl = x0; pxl < x1; pxl++) {
        const Xs = (pxl + 0.5 - cx) / r;
        const d = Xs * Xs + Ys * Ys;
        if (d > 1) continue;
        let lon, lat, ex, ey, ez;   // geographic position and Earth-fixed unit normal
        let hitT = 0;               // pilot: distance from the camera to the ground point (km)
        if (cam) {
          const { c, f, u, rt } = cam;
          let dx = f[0] + (Xs * rt[0] + Ys * u[0]) * tanH;
          let dy = f[1] + (Xs * rt[1] + Ys * u[1]) * tanH;
          let dz = f[2] + (Xs * rt[2] + Ys * u[2]) * tanH;
          const dn = Math.hypot(dx, dy, dz); dx /= dn; dy /= dn; dz /= dn;
          const b = c[0] * dx + c[1] * dy + c[2] * dz;
          const disc = b * b - (cc2 - RE * RE);
          const t = disc >= 0 ? -b - Math.sqrt(disc) : -1;
          if (t <= 0) {
            // Sky: black space with a thin atmosphere glow along the limb, lit on the day side.
            const tc = Math.max(0, -b);                     // closest approach along the ray
            const qx = c[0] + tc * dx, qy = c[1] + tc * dy, qz = c[2] + tc * dz;
            const qn = Math.hypot(qx, qy, qz);
            const glow = Math.exp(-Math.max(0, qn - RE) / 90);
            const sd = ((qx * cgm + qy * sgm) * sx + (-qx * sgm + qy * cgm) * sy + qz * sz) / qn;
            const lit = smoothstep(-0.2, 0.15, sd);
            const oi = (py * s + pxl) * 4;
            o[oi] = 4 + 90 * glow * lit; o[oi + 1] = 6 + 150 * glow * lit; o[oi + 2] = 14 + 255 * glow * (0.15 + 0.85 * lit);
            o[oi + 3] = 255;
            continue;
          }
          hitT = t;
          const nx = (c[0] + t * dx) / RE, ny = (c[1] + t * dy) / RE, nz = (c[2] + t * dz) / RE;
          ex = nx * cgm + ny * sgm; ey = -nx * sgm + ny * cgm; ez = nz;   // ECI → Earth-fixed
          lon = Math.atan2(ey, ex);
          lat = Math.asin(Math.max(-1, Math.min(1, ez)));
        } else if (flat) {
          // Azimuthal equidistant around the north pole: radius ∝ polar distance,
          // Greenwich pointing down, longitude increasing counter-clockwise.
          lat = Math.PI / 2 - Math.sqrt(d) * Math.PI;
          lon = Math.atan2(Xs, -Ys) - lam;
          const cl = Math.cos(lat);
          ex = cl * Math.cos(lon); ey = cl * Math.sin(lon); ez = Math.sin(lat);
        } else {
          const Z = Math.sqrt(1 - d);
          // Undo d3's [λ, φ, γ] rotation: spin back by γ, tilt back by φ, shift longitude by λ.
          const X = Xs * cg + Ys * sg2;
          const Y = Ys * cg - Xs * sg2;
          const xr = Z * cp + Y * sp;
          const zr = Y * cp - Z * sp;
          lon = Math.atan2(X, xr) - lam;
          lat = Math.asin(Math.max(-1, Math.min(1, zr)));
          // Surface normal in Earth-fixed axes = (xr, X, zr) turned back by λ; no extra trig.
          ex = xr * cl0 + X * sl0; ey = X * cl0 - xr * sl0; ez = zr;
        }
        const fu = ((lon / (2 * Math.PI) + 0.5) % 1 + 1) % 1, fv = Math.min(0.9999, 0.5 - lat / Math.PI);
        const oi = (py * s + pxl) * 4;

        let R = sr, G = sg, B = sb;
        let coast = false, hiDone = false;
        if (cam && hitT < hiMaxT) {
          // High-res tile lookup; consecutive pixels mostly hit the same tile.
          const X = fu * HI.COLS * HI.T, Y = fv * HI.ROWS * HI.T;
          const col = (X / HI.T) | 0, row = (Y / HI.T) | 0, key = row * HI.COLS + col;
          if (key !== hiKey) {
            hiKey = key; hiTile = hiTiles.get(key) || null;
            if (hiTile) hiTile.used = hiFrame; else requestHiTile(key);
          }
          if (hiTile) {
            const tx = (X | 0) - col * HI.T, ty = (Y | 0) - row * HI.T, t = hiTile.px;
            const ti = (ty * HI.T + tx) * 4, water = t[ti + 3] === 0;
            if (!water) { R = t[ti]; G = t[ti + 1]; B = t[ti + 2]; }
            coast = (tx + 1 < HI.T && (t[ti + 7] === 0) !== water) || (ty + 1 < HI.T && (t[ti + HI.T * 4 + 3] === 0) !== water);
            hiDone = true;
          }
        }
        if (!hiDone && mask) {
          const mu = Math.floor(fu * MASK_W), mv = Math.floor(fv * MASK_H), mi = mv * MASK_W + mu;
          const land = mask[mi];
          coast = land !== mask[mv * MASK_W + ((mu + 1) % MASK_W)] || (mv + 1 < MASK_H && land !== mask[mi + MASK_W]);
          if (land) {
            if (tex) {
              const ti = (Math.floor(fv * tex.h) * tex.w + Math.floor(fu * tex.w)) * 4, t = tex.px;
              if (t[ti + 3] !== 0) { R = t[ti]; G = t[ti + 1]; B = t[ti + 2]; }   // else: lake
            } else { R = lr; G = lg; B = lb; }
          }
        }
        if (coast) { R = R * 0.45 + kr * 0.55; G = G * 0.45 + kg * 0.55; B = B * 0.45 + kb * 0.55; }

        if (withNight) {
          // Sun elevation → night factor, with a soft twilight band (~+6° … −7°).
          const sunDot = ex * sx + ey * sy + ez * sz;
          const n = smoothstep(0.1, -0.12, sunDot);
          if (n > 0) {
            const a = 0.78 * n;
            R = R * (1 - a) + 4 * a; G = G * (1 - a) + 8 * a; B = B * (1 - a) + 28 * a;
            if (lights) {
              const L = lights.px[Math.floor(fv * lights.h) * lights.w + Math.floor(fu * lights.w)] * n;
              R += L; G += L * 0.8; B += L * 0.45;
            }
          }
        }
        if (cam) {
          if (grat) {
            // Graticule every 10°, line width ≈ one pixel's footprint on the ground.
            const fp = hitT * pixAng / RE * DEG * 0.8;
            const la = lat * DEG, lo = lon * DEG;
            const dLat = Math.abs(la - Math.round(la / 10) * 10);
            const dLon = Math.abs(lo - Math.round(lo / 10) * 10) * Math.cos(lat);
            if (dLat < fp || dLon < fp) { R = R * 0.8 + 51; G = G * 0.8 + 51; B = B * 0.8 + 51; }
          }
          // Aerial haze toward the horizon.
          const sd = ex * sx + ey * sy + ez * sz, lit = smoothstep(-0.12, 0.1, sd);
          const hz = 0.6 * Math.pow(Math.min(1, hitT / tHorizon), 5);
          R = R * (1 - hz) + (12 + 120 * lit) * hz; G = G * (1 - hz) + (18 + 160 * lit) * hz; B = B * (1 - hz) + (40 + 210 * lit) * hz;
        }
        o[oi] = R; o[oi + 1] = G; o[oi + 2] = B; o[oi + 3] = 255;   // Uint8Clamped clamps
      }
    }
    globeBuf.getContext('2d').putImageData(globeImg, 0, 0);
    return globeBuf;
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
    return state.mode === 'orbit' || state.mode === 'pilot' ? state.now.gmst : pt.gmst;
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
    if (state.mode === 'flat') return [0, -90, 0];
    if (state.mode === 'pilot') return [0, 0, 0];   // no look-around offset
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

  // Pilot view camera at the object: forward = prograde (inertial velocity), up = radial
  // out, then the user's look-around offsets (yaw = −rotate[0], pitch = −rotate[1]).
  const vnorm = (a) => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
  const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const vcross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  function updateCamera() {
    if (!state.now) { state.cam = null; return; }
    const c = state.now.r, v = state.now.v;
    let f = vnorm([v.x, v.y, v.z]);
    let u = vnorm(c.map((x, i) => x - vdot(c, f) * f[i]));
    let rt = vcross(f, u);
    const yaw = -state.rotate[0] / DEG, pitch = -state.rotate[1] / DEG;
    const f1 = f.map((x, i) => x * Math.cos(yaw) + rt[i] * Math.sin(yaw));
    rt = rt.map((x, i) => x * Math.cos(yaw) - f[i] * Math.sin(yaw));
    f = f1.map((x, i) => x * Math.cos(pitch) + u[i] * Math.sin(pitch));
    u = u.map((x, i) => x * Math.cos(pitch) - f1[i] * Math.sin(pitch));
    state.cam = { c, f, u, rt, gmst: state.now.gmst };
  }

  // Perspective projection from the pilot camera. Lon/lat are in the frame of the camera's
  // GMST (inertial frame, like orbit view). inView = in front of the camera.
  function projectPilot(lon, lat, altKm) {
    const cam = state.cam, s = state.size;
    const rr = EARTH_R_KM + altKm, la = lat / DEG, lo = lon / DEG + cam.gmst;
    const P = [rr * Math.cos(la) * Math.cos(lo), rr * Math.cos(la) * Math.sin(lo), rr * Math.sin(la)];
    const rel = [P[0] - cam.c[0], P[1] - cam.c[1], P[2] - cam.c[2]];
    const z = vdot(rel, cam.f);
    if (z <= 1e-6) return { x: 0, y: 0, hidden: true, front: false, inView: false, v: [0, 0, 0] };
    const k = (s / 2) / Math.tan(PILOT_HALF_FOV);
    const x = s / 2 + k * vdot(rel, cam.rt) / z, y = s / 2 - k * vdot(rel, cam.u) / z;
    // Hidden if the line of sight meets the Earth before reaching the point.
    const dist = Math.hypot(...rel), d = rel.map((q) => q / dist);
    const b = vdot(cam.c, d), disc = b * b - (vdot(cam.c, cam.c) - EARTH_R_KM ** 2);
    const t1 = disc >= 0 ? -b - Math.sqrt(disc) : -1;
    const hidden = t1 > 0 && t1 < dist - 1e-3;
    return { x, y, hidden, front: !hidden, inView: true, v: [0, 0, 0] };
  }

  // Project a point at altitude: returns screen x/y plus whether the Earth hides it.
  function project3d(lon, lat, altKm) {
    if (isPilot()) return state.cam ? projectPilot(lon, lat, altKm) : { x: 0, y: 0, hidden: true, front: false, inView: false, v: [0, 0, 0] };
    if (isFlat()) {
      // Flat map: ground position only, nothing is ever hidden.
      const [x, y] = projection([lon, lat]);
      const [cx, cy] = projection.translate();
      return { x, y, hidden: false, front: true, inView: true, v: [x - cx, cy - y, 0] };
    }
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
    return { x, y, hidden, front: ez >= 0, inView: true, v: [k * ex, k * ey, k * ez] };
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
  const perf = window.__perf = { pixels: 0, total: 0 };   // last frame's timings (ms)
  function draw() {
    if (isPilot()) { drawPilot(); return; }
    const s = state.size;
    const r = discRadius();
    const [cx, cy] = projection.translate();
    projection.rotate(state.rotate);

    // Instrument face behind the globe.
    ctx.clearRect(0, 0, s, s);
    const face = ctx.createRadialGradient(cx, cy, r * 0.9, cx, cy, s * 0.5);
    face.addColorStop(0, '#2a2e2c');
    face.addColorStop(1, C.face);
    ctx.fillStyle = face;
    ctx.fillRect(0, 0, s, s);

    // The globe stands out of the face: a soft cast shadow away from the lamp (down-right)
    // and a tight contact shadow where the sphere meets the window.
    ctx.save();
    const ox = -LIGHT_DIR[0] * r * 0.11, oy = LIGHT_DIR[1] * r * 0.11;
    const sh = ctx.createRadialGradient(cx + ox, cy + oy, r * 0.85, cx + ox, cy + oy, r * 1.16);
    sh.addColorStop(0, 'rgba(0,0,0,.7)');
    sh.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = sh;
    ctx.beginPath(); ctx.arc(cx + ox, cy + oy, r * 1.16, 0, 2 * Math.PI); ctx.fill();
    const contact = ctx.createRadialGradient(cx, cy, r * 0.97, cx, cy, r * 1.045);
    contact.addColorStop(0, 'rgba(0,0,0,.85)');
    contact.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = contact;
    ctx.beginPath(); ctx.arc(cx, cy, r * 1.045, 0, 2 * Math.PI); ctx.fill();
    ctx.restore();

    // Sea, terrain, coastline, night side and city lights: one per-pixel pass.
    ctx.save();
    ctx.beginPath(); path(sphere); ctx.clip();
    const tp = performance.now();
    ctx.drawImage(renderGlobePixels(ui.night.checked), 0, 0);
    perf.pixels = performance.now() - tp;

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

    if (state.mode === 'orbit') drawMarkMesh();
    drawFootprint();
    drawBehindEarth();

    if (isFlat()) drawFlatShading(cx, cy, r); else drawGlobeShading(cx, cy, r);
    ctx.restore();

    if (ui.reticle.checked && !isFlat()) drawReticle(cx, cy, r);
    drawHome();
    drawTrack();
    if (state.mode !== 'orbit') drawMarkTicks();
    drawSatellite();
    drawHud();
    if (state.menuOpen) drawMenu();
  }

  // Pilot view: what you'd see from the object looking prograde. Ray-cast Earth below the
  // horizon, space and limb glow above; whole round screen, no globe frame.
  function drawPilot() {
    const s = state.size, c = s / 2;
    updateCamera();
    ctx.clearRect(0, 0, s, s);
    ctx.fillStyle = '#04060e';
    ctx.fillRect(0, 0, s, s);
    ctx.save();
    ctx.beginPath(); ctx.arc(c, c, c, 0, 2 * Math.PI); ctx.clip();
    if (state.cam) {
      const tp = performance.now();
      ctx.drawImage(renderGlobePixels(ui.night.checked), 0, 0);
      perf.pixels = performance.now() - tp;
    }
    const rim = ctx.createRadialGradient(c, c, c * 0.75, c, c, c);
    rim.addColorStop(0, 'rgba(0,0,0,0)');
    rim.addColorStop(1, 'rgba(0,0,0,.45)');
    ctx.fillStyle = rim;
    ctx.fillRect(0, 0, s, s);
    ctx.restore();
    if (state.cam) {
      drawHome();
      drawTrack();
      drawPilotHud();
    }
    drawHud();
    if (state.menuOpen) drawMenu();
  }

  // Prograde symbol (as on a flight director): circle with three stubs, at the
  // prograde direction (screen centre unless the user is looking around).
  function drawProgradeMarker() {
    const s = state.size, cam = state.cam, v = state.now.v;
    const f = vnorm([v.x, v.y, v.z]), z = vdot(f, cam.f);
    if (z <= 0.05) return;
    const k = (s / 2) / Math.tan(PILOT_HALF_FOV);
    const x = s / 2 + k * vdot(f, cam.rt) / z, y = s / 2 - k * vdot(f, cam.u) / z;
    const r = s / 48;
    const shape = () => {
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
      ctx.moveTo(x - r, y); ctx.lineTo(x - r * 2, y);
      ctx.moveTo(x + r, y); ctx.lineTo(x + r * 2, y);
      ctx.moveTo(x, y - r); ctx.lineTo(x, y - r * 1.8);
    };
    ctx.save();
    ctx.lineCap = 'round';
    ctx.strokeStyle = C.hudShadow; ctx.lineWidth = s / 160; shape(); ctx.stroke();
    ctx.strokeStyle = C.hud; ctx.lineWidth = s / 300; shape(); ctx.stroke();
    ctx.restore();
  }

  // Screen position of a world direction (unit vector, ECI) through the pilot camera.
  function projectDir(d) {
    const cam = state.cam, s = state.size, z = vdot(d, cam.f);
    if (z <= 0.05) return null;
    const k = (s / 2) / Math.tan(PILOT_HALF_FOV);
    return { x: s / 2 + k * vdot(d, cam.rt) / z, y: s / 2 - k * vdot(d, cam.u) / z };
  }

  // Fighter-style HUD, fixed to the screen: pitch ladder (conformal to the local
  // horizontal), heading tape, boresight, speed and altitude boxes, prograde marker.
  function drawPilotHud() {
    const s = state.size, c = s / 2, cam = state.cam;
    const up = vnorm(cam.c);                                         // local vertical
    const north = vnorm([-up[2] * up[0], -up[2] * up[1], 1 - up[2] * up[2]]);   // ECI z on the horizontal plane
    const east = vcross(north, up);
    // Camera forward split into horizontal heading + pitch.
    const fu = vdot(cam.f, up);
    const hf = vnorm(cam.f.map((x, i) => x - fu * up[i]));
    const heading = ((Math.atan2(vdot(hf, east), vdot(hf, north)) * DEG) + 360) % 360;

    const lw = Math.max(1.2, s / 360), fs = Math.round(s * 0.03);
    ctx.save();
    ctx.strokeStyle = C.hud; ctx.fillStyle = C.hud;
    ctx.lineWidth = lw; ctx.lineCap = 'round';
    ctx.shadowColor = C.hudShadow; ctx.shadowBlur = s / 160;
    ctx.font = `600 ${fs}px ${HUD_FONT}`;
    ctx.textBaseline = 'middle';

    // Pitch ladder, clipped to the HUD glass in the middle of the screen.
    ctx.save();
    ctx.beginPath(); ctx.arc(c, c, s * 0.34, 0, 2 * Math.PI); ctx.clip();
    for (let p = -80; p <= 80; p += 10) {
      const pr = p / DEG;
      const pt = projectDir(hf.map((x, i) => x * Math.cos(pr) + up[i] * Math.sin(pr)));
      if (!pt) continue;
      const half = p === 0 ? s * 0.2 : s * 0.09, gap = s * 0.045, tick = s * 0.018 * (p > 0 ? 1 : -1);
      ctx.setLineDash(p < 0 ? [s / 70, s / 110] : []);
      ctx.beginPath();
      ctx.moveTo(pt.x - half, pt.y + (p ? tick : 0)); ctx.lineTo(pt.x - half, pt.y); ctx.lineTo(pt.x - gap, pt.y);
      ctx.moveTo(pt.x + gap, pt.y); ctx.lineTo(pt.x + half, pt.y); ctx.lineTo(pt.x + half, pt.y + (p ? tick : 0));
      ctx.stroke();
      ctx.setLineDash([]);
      if (p) {
        ctx.textAlign = 'right'; ctx.fillText(String(Math.abs(p)), pt.x - half - s * 0.01, pt.y);
        ctx.textAlign = 'left'; ctx.fillText(String(Math.abs(p)), pt.x + half + s * 0.01, pt.y);
      }
    }
    ctx.restore();

    // Boresight "W" fixed at the screen centre (where you are looking).
    const w = s * 0.022;
    ctx.beginPath();
    ctx.moveTo(c - w * 2, c); ctx.lineTo(c - w, c); ctx.lineTo(c - w * 0.5, c + w * 0.6);
    ctx.lineTo(c, c); ctx.lineTo(c + w * 0.5, c + w * 0.6); ctx.lineTo(c + w, c); ctx.lineTo(c + w * 2, c);
    ctx.stroke();

    // Heading tape under the time tab.
    const ty = s * 0.135, span = 35, pxDeg = s * 0.0065;
    ctx.textAlign = 'center';
    for (let hd = Math.ceil((heading - span) / 5) * 5; hd <= heading + span; hd += 5) {
      const x = c + (hd - heading) * pxDeg, h = ((hd % 360) + 360) % 360;
      const major = h % 10 === 0;
      ctx.beginPath(); ctx.moveTo(x, ty); ctx.lineTo(x, ty - (major ? s * 0.014 : s * 0.008)); ctx.stroke();
      if (h % 30 === 0) {
        const lbl = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[h] ?? String(h / 10).padStart(2, '0');
        ctx.fillText(lbl, x, ty - s * 0.032);
      }
    }
    ctx.beginPath();   // caret + current heading box
    ctx.moveTo(c - s * 0.012, ty + s * 0.022); ctx.lineTo(c, ty + s * 0.006); ctx.lineTo(c + s * 0.012, ty + s * 0.022);
    ctx.stroke();
    hudBox(String(Math.round(heading) % 360).padStart(3, '0'), c, ty + s * 0.048, 'center', fs);

    // Speed (left) and altitude (right) boxes.
    const g = geoOf(state.now.r, state.now.gmst);
    const v = state.now.v, spd = Math.hypot(v.x, v.y, v.z);
    hudBox(spd.toFixed(2), s * 0.11, c, 'left', fs, 'KM/S');
    hudBox(g.alt.toFixed(0), s * 0.89, c, 'right', fs, 'KM');
    ctx.restore();

    drawProgradeMarker();
  }

  // Outlined HUD value box; caption under it. align: which screen side it hugs.
  function hudBox(text, x, y, align, fs, caption) {
    const s = state.size;
    ctx.font = `600 ${fs}px ${HUD_FONT}`;
    const w = ctx.measureText(text).width + fs * 0.8, h = fs * 1.35;
    const x0 = align === 'left' ? x : align === 'right' ? x - w : x - w / 2;
    ctx.strokeRect(x0, y - h / 2, w, h);
    ctx.textAlign = 'center';
    ctx.fillText(text, x0 + w / 2, y + fs * 0.04);
    if (caption) {
      ctx.font = `500 ${Math.round(fs * 0.7)}px ${HUD_FONT}`;
      ctx.fillText(caption, x0 + w / 2, y + h / 2 + fs * 0.6);
    }
  }

  // Soviet-instrument style lighting: lit from upper left, dark limb, glass glint.
  // Physical sphere lighting, like a real globe behind the instrument window: Lambert
  // diffuse from a lamp at the upper left/front, darkening toward the limb all round, and a
  // small glossy highlight. It depends only on the pixel's place on the disc, never on the
  // globe's rotation, so it is computed once per display size into two layers:
  //   shade    – multiplied over the finished globe (map, lines, everything printed on it)
  //   specular – added on top
  // ESP: the same two layers as per-pixel lookup tables (one byte each).
  const LIGHT_DIR = (() => { const v = [-0.5, 0.6, 0.62], n = Math.hypot(...v); return v.map((x) => x / n); })();
  const SPHERE = { AMB: 0.16, DIF: 1.25, RIM: 0.42, SPEC: 0.42, SPEC_N: 70, SHEEN: 0.05, SHEEN_N: 6 };
  let sphereLight = null;
  function sphereLayers(s, r, cx, cy) {
    if (sphereLight && sphereLight.s === s && sphereLight.r === r) return sphereLight;
    const mk = () => { const c = document.createElement('canvas'); c.width = c.height = s; return c; };
    const shade = mk(), spec = mk();
    const si = shade.getContext('2d').createImageData(s, s), pi = spec.getContext('2d').createImageData(s, s);
    const [lx, ly, lz] = LIGHT_DIR;
    const hn = Math.hypot(lx, ly, lz + 1), hx = lx / hn, hy = ly / hn, hz = (lz + 1) / hn;   // half vector, viewer at +z
    for (let py = 0; py < s; py++) {
      const Y = (cy - py - 0.5) / r;
      for (let px = 0; px < s; px++) {
        const X = (px + 0.5 - cx) / r, d = X * X + Y * Y, i = (py * s + px) * 4;
        si.data[i + 3] = pi.data[i + 3] = 255;
        if (d > 1) { si.data[i] = si.data[i + 1] = si.data[i + 2] = 255; continue; }
        const Z = Math.sqrt(1 - d);
        const ndl = Math.max(0, X * lx + Y * ly + Z * lz);
        const k = (SPHERE.AMB + SPHERE.DIF * ndl) * (1 - SPHERE.RIM + SPHERE.RIM * Math.sqrt(Z));
        si.data[i] = si.data[i + 1] = si.data[i + 2] = 255 * Math.min(1, k);
        const ndh = Math.max(0, X * hx + Y * hy + Z * hz);
        const sp = SPHERE.SPEC * Math.pow(ndh, SPHERE.SPEC_N) + SPHERE.SHEEN * Math.pow(ndh, SPHERE.SHEEN_N);
        pi.data[i] = pi.data[i + 1] = pi.data[i + 2] = 255 * Math.min(1, sp);
      }
    }
    shade.getContext('2d').putImageData(si, 0, 0);
    spec.getContext('2d').putImageData(pi, 0, 0);
    sphereLight = { s, r, shade, spec };
    return sphereLight;
  }

  function drawGlobeShading(cx, cy, r) {
    const L = sphereLayers(state.size, r, cx, cy);
    ctx.globalCompositeOperation = 'multiply';
    ctx.drawImage(L.shade, 0, 0);
    ctx.globalCompositeOperation = 'lighter';
    ctx.drawImage(L.spec, 0, 0);
    ctx.globalCompositeOperation = 'source-over';
  }

  // Home location marker: blue dot with white rim, like an ownship symbol on a chart.
  function drawHome() {
    const p = project3d(HOME.lon, HOME.lat, 0);
    if (!p.front || !p.inView) return;
    const s = state.size, rr = s / 150;
    ctx.save();
    ctx.fillStyle = '#1e7bff';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = Math.max(1.2, s / 360);
    ctx.beginPath(); ctx.arc(p.x, p.y, rr, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'left';
    hudText(HOME.name.toUpperCase(), p.x + rr * 2, p.y, Math.round(s / 36), 500);
    ctx.restore();
  }

  // Flat map: no sphere lighting, just a soft rim vignette and the same glass glint.
  function drawFlatShading(cx, cy, r) {
    const rim = ctx.createRadialGradient(cx, cy, r * 0.8, cx, cy, r);
    rim.addColorStop(0, 'rgba(0,0,0,0)');
    rim.addColorStop(1, 'rgba(0,0,0,.35)');
    ctx.fillStyle = rim;
    ctx.fillRect(cx - r, cy - r, 2 * r, 2 * r);
    ctx.save();
    ctx.translate(cx - r * 0.38, cy - r * 0.48);
    ctx.rotate(-0.6);
    const glint = ctx.createRadialGradient(0, 0, 0, 0, 0, r * 0.32);
    glint.addColorStop(0, 'rgba(255,255,255,.22)');
    glint.addColorStop(1, 'rgba(255,255,255,0)');
    ctx.scale(1, 0.55);
    ctx.fillStyle = glint;
    ctx.beginPath(); ctx.arc(0, 0, r * 0.32, 0, 2 * Math.PI); ctx.fill();
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
      // Behind the Earth: not drawn here; drawBehindEarth() prints it grey on the surface.
      if (a.hidden || b.hidden || !a.inView || !b.inView) continue;
      ctx.globalAlpha = fade;
      ctx.strokeStyle = past ? C.trackPast : C.trackFuture;
      ctx.lineWidth = past ? s / 300 : s / 200;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    ctx.restore();
  }

  // Whatever is behind the Earth keeps the real orbit's shape (its straight-through
  // projection), but is drawn as a solid grey line printed on the visible surface:
  // drawn inside the globe clip before the glass shading, so the globe stays opaque.
  // Grey track, grey 5-min ticks (ground view) and a grey dot for a hidden object.
  function drawBehindEarth() {
    if (isFlat() || state.track.length < 2) return;
    const s = state.size;
    ctx.save();
    ctx.lineCap = 'round';
    ctx.strokeStyle = C.behind;

    let prev = null;
    for (const p of state.track) {
      const g = geoOf(p.r, frameGmst(p));
      const cur = { t: p.t, ...project3d(g.lon, g.lat, g.alt) };
      if (prev && (prev.hidden || cur.hidden) && prev.inView && cur.inView) {
        ctx.globalAlpha = 1 - 0.6 * Math.abs(cur.t) / TRACK_SPAN_S;
        ctx.lineWidth = cur.t <= 0 ? s / 340 : s / 260;
        ctx.beginPath(); ctx.moveTo(prev.x, prev.y); ctx.lineTo(cur.x, cur.y); ctx.stroke();
      }
      prev = cur;
    }

    if (state.mode === 'home') {
      const half = s / 110;
      for (const m of state.marks) {
        const a = project3d(m.lon, m.lat, m.alt);
        if (!a.hidden) continue;
        const b = project3d(m.dir.lon, m.dir.lat, m.dir.alt);
        const dx = b.x - a.x, dy = b.y - a.y, n = Math.hypot(dx, dy);
        if (n < 1e-6) continue;
        ctx.globalAlpha = 1 - 0.6 * Math.abs(m.t) / TRACK_SPAN_S;
        ctx.lineWidth = s / 300;
        ctx.beginPath();
        ctx.moveTo(a.x + dy / n * half, a.y - dx / n * half);
        ctx.lineTo(a.x - dy / n * half, a.y + dx / n * half);
        ctx.stroke();
      }
    }

    if (state.now) {
      const g = geoOf(state.now.r, state.now.gmst), a = project3d(g.lon, g.lat, g.alt);
      if (a.hidden && a.inView) {
        // Hollow ring, same stroke width as the (future) grey track line.
        ctx.globalAlpha = 1;
        ctx.strokeStyle = C.behind;
        ctx.lineWidth = s / 260;
        ctx.beginPath(); ctx.arc(a.x, a.y, s / 55, 0, 2 * Math.PI); ctx.stroke();
      }
    }
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

  // Both views: the object's horizon on the ground — every point that sees it at 0°
  // elevation lies on a small circle around the sub-satellite point, radius acos(R / (R + h)).
  function drawFootprint() {
    if (!state.now) return;
    const g = geoOf(state.now.r, state.now.gmst);
    const radius = Math.acos(EARTH_R_KM / (EARTH_R_KM + g.alt)) * DEG;
    const s = state.size;
    ctx.save();
    ctx.strokeStyle = C.footprint;
    ctx.lineWidth = Math.max(1, s / 420);
    ctx.beginPath(); path(d3.geoCircle().center([g.lon, g.lat]).radius(radius).precision(2)()); ctx.stroke();
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
      if (p.hidden) continue;   // drawn grey by drawBehindEarth()
      ctx.globalAlpha = 1 - 0.65 * Math.abs(m.t) / TRACK_SPAN_S;
      ctx.strokeStyle = m.t > 0 ? C.trackFuture : C.trackPast;
      ctx.lineWidth = s / 240;
      ctx.beginPath(); ctx.moveTo(p.x - nx, p.y - ny); ctx.lineTo(p.x + nx, p.y + ny); ctx.stroke();
    }
    ctx.restore();
  }

  function drawSatellite() {
    if (!state.now) return;
    const s = state.size;
    const sat = geoOf(state.now.r, state.now.gmst);
    const p = project3d(sat.lon, sat.lat, sat.alt);
    if (p.hidden) return;   // behind the Earth: grey dot from drawBehindEarth() instead
    const g = project3d(sat.lon, sat.lat, 0);
    ctx.save();

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

  // ---------- HUD ----------
  // Instrument-cluster style (cf. marine chart plotters): white condensed DIN-like digits,
  // small superscript units, simple line icons, no boxes except the top time tag.
  // Readouts sit around the globe's rim so the globe itself stays clear.
  const HUD_FONT = '"Barlow Condensed", "DIN Condensed", "Arial Narrow", sans-serif';

  async function loadFonts() {
    try {
      await Promise.all([500, 600].map(async (w) => {
        const f = new FontFace('Barlow Condensed', `url(vendor/barlow-condensed-${w}.woff2)`, { weight: String(w) });
        document.fonts.add(await f.load());
      }));
    } catch (e) { /* falls back to system condensed fonts */ }
    state.dirty = true;
  }

  // Text, optionally with a thin dark outline so it stays legible over bright terrain.
  function hudText(text, x, y, size, weight = 600, outline = true) {
    ctx.font = `${weight} ${size}px ${HUD_FONT}`;
    if (outline) {
      ctx.lineJoin = 'round';
      ctx.lineWidth = Math.max(2, size * 0.16);
      ctx.strokeStyle = 'rgba(0,0,0,.7)';
      ctx.strokeText(text, x, y);
    }
    ctx.fillStyle = '#ffffff';
    ctx.fillText(text, x, y);
    return ctx.measureText(text).width;
  }

  // Icons: plain paths so they port to any embedded graphics lib.
  const ICONS = {
    // Map pin = distance from home.
    pin(x, y, h) {
      const r = h * 0.3, cy = y - h * 0.12;
      ctx.beginPath();
      ctx.arc(x, cy, r, Math.PI * 0.8, Math.PI * 2.2);
      ctx.lineTo(x, y + h * 0.48);
      ctx.closePath();
      ctx.fill(); ctx.stroke();
      ctx.save(); ctx.fillStyle = '#111'; ctx.beginPath(); ctx.arc(x, cy, r * 0.4, 0, 2 * Math.PI); ctx.fill(); ctx.restore();
    },
    // Clock = time of next pass.
    clock(x, y, h) {
      const r = h * 0.42;
      ctx.save();
      ctx.lineWidth = Math.max(1.5, h * 0.12);
      ctx.strokeStyle = 'rgba(0,0,0,.7)'; ctx.lineWidth += 2;
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.stroke();
      ctx.lineWidth -= 2; ctx.strokeStyle = '#fff';
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
      ctx.moveTo(x, y); ctx.lineTo(x, y - r * 0.65);
      ctx.moveTo(x, y); ctx.lineTo(x + r * 0.5, y + r * 0.1);
      ctx.stroke();
      ctx.restore();
    },
    // Dimension line |←→| = distance.
    dist(x, y, h) {
      const w = h * 0.55, a = h * 0.2;
      ctx.save();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = Math.max(1.5, h * 0.11); ctx.lineCap = 'round'; ctx.lineJoin = 'round';
      ctx.beginPath();
      ctx.moveTo(x - w, y - h * 0.32); ctx.lineTo(x - w, y + h * 0.32);   // end bars
      ctx.moveTo(x + w, y - h * 0.32); ctx.lineTo(x + w, y + h * 0.32);
      ctx.moveTo(x - w * 0.7, y); ctx.lineTo(x + w * 0.7, y);               // shaft
      ctx.moveTo(x - w * 0.7 + a, y - a); ctx.lineTo(x - w * 0.7, y); ctx.lineTo(x - w * 0.7 + a, y + a);
      ctx.moveTo(x + w * 0.7 - a, y - a); ctx.lineTo(x + w * 0.7, y); ctx.lineTo(x + w * 0.7 - a, y + a);
      ctx.stroke();
      ctx.restore();
    },
    // Curve dipping to a marked minimum = time of closest approach (next pass).
    pass(x, y, h) {
      const w = h * 0.5;
      ctx.save();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = Math.max(1.5, h * 0.11); ctx.lineCap = 'round';
      const by = y + h * 0.22;   // bottom of the U
      ctx.beginPath();
      ctx.moveTo(x - w, y - h * 0.48);
      ctx.bezierCurveTo(x - w * 0.75, by + h * 0.1, x + w * 0.75, by + h * 0.1, x + w, y - h * 0.48);
      ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.beginPath(); ctx.arc(x, by - h * 0.02, h * 0.19, 0, 2 * Math.PI); ctx.fill();
      ctx.restore();
    },
    // Up-triangle over a ground line = altitude.
    alt(x, y, h) {
      const w = h * 0.42;
      ctx.beginPath();
      ctx.moveTo(x, y - h * 0.42); ctx.lineTo(x + w, y + h * 0.22); ctx.lineTo(x - w, y + h * 0.22);
      ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.fillRect(x - w, y + h * 0.34, 2 * w, Math.max(1.5, h * 0.1));
    },
  };

  // One readout: [icon] VALUE ᵘⁿⁱᵗ starting at x. Returns its width; draw=false only measures.
  function readout(x, y, { icon, value, unit }, fs, draw = true) {
    const us = Math.round(fs * 0.5), ih = fs * 0.75, gap = fs * 0.22;
    ctx.font = `600 ${fs}px ${HUD_FONT}`;
    const vw = ctx.measureText(value).width;
    ctx.font = `500 ${us}px ${HUD_FONT}`;
    const uw = unit ? ctx.measureText(unit).width + fs * 0.06 : 0;
    const iw = icon ? ih + gap : 0;
    if (!draw) return iw + vw + uw;
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'left';
    if (icon) {
      ctx.fillStyle = '#fff';
      ctx.strokeStyle = 'rgba(0,0,0,0)';
      ctx.lineWidth = 1;
      ICONS[icon](x + ih / 2, y, ih);
    }
    hudText(value, x + iw, y, fs, 600, false);
    if (unit) hudText(unit, x + iw + vw + fs * 0.06, y - fs * 0.22, us, 500, false);
    return iw + vw + uw;
  }

  // Widest digit of the HUD font at this size, so value slots fit any number.
  const widestDigitCache = new Map();
  function widestDigit(fs) {
    if (!widestDigitCache.has(fs) || !document.fonts.check(`600 ${fs}px "Barlow Condensed"`)) {
      ctx.save();
      ctx.font = `600 ${fs}px ${HUD_FONT}`;
      let best = '0', bw = 0;
      for (const d of '0123456789') { const w = ctx.measureText(d).width; if (w > bw) { bw = w; best = d; } }
      ctx.restore();
      widestDigitCache.set(fs, best);
    }
    return widestDigitCache.get(fs);
  }

  // A centred row of readouts separated by thin vertical rules. Returns the row width.
  // `slots`: worst-case items ('#' = any digit); each readout gets a fixed slot at least
  // that wide, left-aligned, so the row never changes width while values change.
  function readoutRow(items, y, fs, draw = true, slots = null) {
    const sep = fs * 0.9, wd = widestDigit(fs);
    const widths = items.map((it, i) => {
      const w = readout(0, 0, it, fs, false);
      if (!slots) return w;
      const t = slots[i];
      return Math.max(w, readout(0, 0, { ...t, value: t.value.replace(/#/g, wd) }, fs, false));
    });
    const total = widths.reduce((a, b) => a + b, 0) + sep * (items.length - 1);
    if (!draw) return total;
    let x = state.size / 2 - total / 2;
    items.forEach((it, i) => {
      readout(x, y, it, fs);
      x += widths[i];
      if (i < items.length - 1) {
        ctx.fillStyle = 'rgba(255,255,255,.28)';
        ctx.fillRect(Math.round(x + sep / 2), y - fs * 0.38, 1, fs * 0.76);
        x += sep;
      }
    });
    return total;
  }

  // Tab-shaped dark panel that runs into the round screen edge: a flat edge just as wide as
  // its content, then steep angled flanks out to the screen circle, so it covers little globe.
  const BAND_ANGLE = 58 / DEG;   // flank angle from horizontal
  function edgeBand(yCut, halfWidth, top) {
    const s = state.size, c = s / 2, dir = top ? -1 : 1;
    const L = s, dx = Math.cos(BAND_ANGLE) * L, dy = Math.sin(BAND_ANGLE) * L * dir;
    const rad = s * 0.012;   // softened corners
    const edge = () => {
      ctx.moveTo(c - halfWidth - dx, yCut + dy);
      ctx.arcTo(c - halfWidth, yCut, c + halfWidth, yCut, rad);
      ctx.arcTo(c + halfWidth, yCut, c + halfWidth + dx, yCut + dy, rad);
      ctx.lineTo(c + halfWidth + dx, yCut + dy);
    };
    ctx.save();
    ctx.beginPath(); ctx.arc(c, c, c, 0, 2 * Math.PI); ctx.clip();
    const g = ctx.createLinearGradient(0, yCut, 0, top ? 0 : s);
    g.addColorStop(0, 'rgba(12,13,14,.78)');
    g.addColorStop(1, 'rgba(12,13,14,.92)');
    ctx.fillStyle = g;
    ctx.beginPath(); edge(); ctx.closePath(); ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.18)';   // hairline along the edge
    ctx.lineWidth = 1;
    ctx.beginPath(); edge(); ctx.stroke();
    ctx.restore();
  }

  function drawHud() {
    const s = state.size, pad = s * 0.03;
    ctx.save();

    // Top tab: UTC time.
    const timeRow = [{ value: simTime().toISOString().slice(11, 19), unit: 'UTC' }];
    const tfs = Math.round(s * 0.034);
    const timeSlots = [{ value: '##:##:##', unit: 'UTC' }];
    const tHalf = readoutRow(timeRow, 0, tfs, false, timeSlots) / 2 + pad;
    edgeBand(s * 0.068, tHalf, true);
    state.topTab = { yCut: s * 0.068, halfWidth: tHalf };
    readoutRow(timeRow, s * 0.038, tfs, true, timeSlots);

    // Bottom tab: primary row (distance to Munich, next pass), secondary row (position, altitude).
    if (state.sat && state.now) {
      const { lat, lon, alt } = state.sat;
      const primary = [
        { icon: 'dist', value: String(Math.round(rangeToHome(state.now))), unit: 'km' },
        { icon: 'pass', value: state.nextPass ? homeTime.format(state.nextPass) : '--:--' },
      ];
      const secondary = [
        { value: Math.abs(lat).toFixed(1) + '°', unit: lat >= 0 ? 'N' : 'S' },
        { value: Math.abs(lon).toFixed(1) + '°', unit: lon >= 0 ? 'E' : 'W' },
        { icon: 'alt', value: alt.toFixed(0), unit: 'km' },
      ];
      // Worst cases: 5-digit km (up to 99 999 km), HH:MM, 88.8°N/S, 188.8°E/W ('W' widest).
      const primarySlots = [
        { icon: 'dist', value: '#####', unit: 'km' },
        { icon: 'pass', value: '##:##' },
      ];
      const secondarySlots = [
        { value: '##.#°', unit: 'W' },
        { value: '###.#°', unit: 'W' },
        { icon: 'alt', value: '#####', unit: 'km' },
      ];
      const pfs = Math.round(s * 0.038), sfs = Math.round(s * 0.028);
      const yCut = s * 0.88, y1 = s * 0.912, y2 = s * 0.957;
      // Fixed tab width: fits the first row at the flat edge and the second row between
      // the flanks, which have widened by (y2 − yCut) / tan(angle) at its height.
      const w1 = readoutRow(primary, 0, pfs, false, primarySlots);
      const w2 = readoutRow(secondary, 0, sfs, false, secondarySlots);
      const half = Math.max(w1 / 2 + pad, w2 / 2 + pad * 0.6 - (y2 - yCut) / Math.tan(BAND_ANGLE));
      edgeBand(yCut, half, false);
      state.bottomTab = { yCut, halfWidth: half };
      readoutRow(primary, y1, pfs, true, primarySlots);
      readoutRow(secondary, y2, sfs, true, secondarySlots);
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

  // ---------- on-screen menu ----------
  const MODES = [
    { mode: 'orbit', label: 'Orbit view', icon: 'orbit' },
    { mode: 'home', label: 'Ground view', icon: 'pin' },
    { mode: 'flat', label: 'Flat earth', icon: 'flat' },
    { mode: 'pilot', label: 'Pilot view', icon: 'pilot' },
  ];

  // Pilot-view icon: prograde symbol.
  ICONS.pilot = (x, y, h) => {
    const r = h * 0.24;
    ctx.save();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = Math.max(1.2, h * 0.09); ctx.lineCap = 'round';
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.moveTo(x - r, y); ctx.lineTo(x - r * 2, y);
    ctx.moveTo(x + r, y); ctx.lineTo(x + r * 2, y);
    ctx.moveTo(x, y - r); ctx.lineTo(x, y - r * 1.8);
    ctx.stroke();
    ctx.restore();
  };

  // Flat-map icon: disc with polar-grid rings and meridians.
  ICONS.flat = (x, y, h) => {
    const r = h * 0.45;
    ctx.save();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = Math.max(1.2, h * 0.08);
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(x, y, r * 0.5, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x - r, y); ctx.lineTo(x + r, y); ctx.moveTo(x, y - r); ctx.lineTo(x, y + r);
    ctx.stroke();
    ctx.restore();
  };

  // Orbit-view icon: globe with the vertical orbit line and the object at its centre.
  ICONS.orbit = (x, y, h) => {
    const r = h * 0.42;
    ctx.save();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = Math.max(1.5, h * 0.1);
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x, y - r * 1.25); ctx.lineTo(x, y + r * 1.25); ctx.stroke();
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.arc(x, y, h * 0.13, 0, 2 * Math.PI); ctx.fill();
    ctx.restore();
  };

  function drawMenu() {
    const s = state.size, c = s / 2;
    ctx.save();
    // Dim everything behind the menu.
    ctx.beginPath(); ctx.arc(c, c, c, 0, 2 * Math.PI); ctx.clip();
    ctx.fillStyle = 'rgba(0,0,0,.45)';
    ctx.fillRect(0, 0, s, s);

    const w = s * 0.5, rowH = s * 0.085, head = s * 0.075;
    const h = head + rowH * MODES.length + s * 0.03;
    const x = c - w / 2, y = c - h / 2;
    ctx.fillStyle = 'rgba(16,17,19,.94)';
    ctx.strokeStyle = 'rgba(255,255,255,.18)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.roundRect(x, y, w, h, s * 0.02); ctx.fill(); ctx.stroke();

    ctx.textBaseline = 'middle'; ctx.textAlign = 'center';
    ctx.font = `500 ${Math.round(s * 0.026)}px ${HUD_FONT}`;
    ctx.letterSpacing = `${(s * 0.004).toFixed(1)}px`;
    ctx.fillStyle = 'rgba(255,255,255,.55)';
    ctx.fillText('VIEW', c, y + head * 0.55);
    ctx.letterSpacing = '0px';

    state.menuHits = [];
    const fs = Math.round(s * 0.038);
    MODES.forEach((m, i) => {
      const ry = y + head + i * rowH;
      const active = state.mode === m.mode;
      if (active) {
        ctx.fillStyle = 'rgba(255,255,255,.10)';
        ctx.fillRect(x + s * 0.012, ry + rowH * 0.08, w - s * 0.024, rowH * 0.84);
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(x + s * 0.012, ry + rowH * 0.08, Math.max(2, s * 0.005), rowH * 0.84);
      }
      ctx.globalAlpha = active ? 1 : 0.7;
      ctx.fillStyle = '#fff';
      ICONS[m.icon](x + s * 0.06, ry + rowH / 2, fs * 0.8);
      ctx.textAlign = 'left';
      hudText(m.label.toUpperCase(), x + s * 0.1, ry + rowH / 2, fs, 600, false);
      ctx.globalAlpha = 1;
      state.menuHits.push({ x, y: ry, w, h: rowH, mode: m.mode });
    });
    ctx.restore();
  }

  function openMenu(open) {
    state.menuOpen = open;
    state.dirty = true;
  }

  // A tap (no drag) on the display, in canvas pixels.
  function onTap(px, py) {
    const s = state.size, c = s / 2;
    if (state.menuOpen) {
      const hit = state.menuHits.find((r) => px >= r.x && px <= r.x + r.w && py >= r.y && py <= r.y + r.h);
      if (hit) setMode(hit.mode);
      openMenu(false);   // any tap closes: selection or outside
      return;
    }
    // Inside a tab: beyond its flat edge and between its angled flanks (on screen).
    const inTab = (b, top) => {
      if (!b || Math.hypot(px - c, py - c) > c) return false;
      const depth = top ? b.yCut - py : py - b.yCut;
      return depth >= 0 && Math.abs(px - c) <= b.halfWidth + depth / Math.tan(BAND_ANGLE);
    };
    if (inTab(state.bottomTab, false)) openMenu(true);
    else if (inTab(state.topTab, true)) resetView();
  }

  // Drop any manual pan and glide back to the current mode's default orientation.
  function resetView() {
    state.manualUntil = 0;
    state.vel = [0, 0];
    state.dirty = true;
  }

  // ---------- interaction ----------
  // Ease the view toward the mode's orientation (short way round for λ and γ).
  // Time-based, so the glide takes the same ~0.6 s at any frame rate (ESP32 included).
  const GLIDE_TAU_S = 0.15;
  let lastEase = 0;
  function easeToTarget() {
    const now = performance.now(), dt = Math.min(0.5, (now - (lastEase || now)) / 1000);
    lastEase = now;
    const t = targetRotation();
    if (!t) return;
    const wrap = (d) => ((d + 540) % 360) - 180;
    const d = [wrap(t[0] - state.rotate[0]), t[1] - state.rotate[1], wrap(t[2] - (state.rotate[2] || 0))];
    if (d.every((v) => Math.abs(v) < 0.005)) { state.rotate = t; return; }
    const k = d.every((v) => Math.abs(v) < 0.5) ? 1 : 1 - Math.exp(-dt / GLIDE_TAU_S);   // glide in, then lock on
    state.rotate = [wrap(state.rotate[0] + d[0] * k), state.rotate[1] + d[1] * k, wrap((state.rotate[2] || 0) + d[2] * k)];
    state.dirty = true;
  }

  function setMode(mode) {
    const wasKind = viewKind(state.mode);
    state.mode = mode;
    state.manualUntil = 0;
    projection = isFlat() ? azeq : ortho;
    path.projection(projection);
    if (wasKind !== viewKind(mode)) {   // no glide across different kinds of view
      if (isPilot()) state.rotate = [0, 0, 0];
      else state.rotate = targetRotation() || (isFlat() ? [0, -90, 0] : state.rotate);
    }
    for (const b of ui.modes) b.classList.toggle('active', b.dataset.mode === mode);
    state.dirty = true;
  }

  // Pointer: a drag rotates the globe; a tap without movement goes to onTap().
  const TAP_SLOP_PX = 6;
  let drag = null;
  canvas.addEventListener('pointerdown', (e) => {
    canvas.setPointerCapture(e.pointerId);
    drag = { x: e.clientX, y: e.clientY, x0: e.clientX, y0: e.clientY, moved: false };
    state.vel = [0, 0];
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag || state.menuOpen) return;
    if (!drag.moved && Math.hypot(e.clientX - drag.x0, e.clientY - drag.y0) < TAP_SLOP_PX) return;
    drag.moved = true;
    canvas.classList.add('dragging');
    const k = DEG / discRadius();   // one disc radius ≈ 57°
    const dx = (e.clientX - drag.x) * k;
    const dy = (e.clientY - drag.y) * k;
    drag.x = e.clientX; drag.y = e.clientY;
    rotateBy(dx, dy);
    state.vel = [dx, dy];
  });
  canvas.addEventListener('pointerup', (e) => {
    if (drag && !drag.moved) {
      const rect = canvas.getBoundingClientRect();
      onTap((e.clientX - rect.left) * canvas.width / rect.width, (e.clientY - rect.top) * canvas.height / rect.height);
    } else if (drag) {
      state.manualUntil = performance.now() + 4000;   // then glide back to the mode's view
    }
    drag = null;
    canvas.classList.remove('dragging');
  });
  canvas.addEventListener('pointercancel', () => { drag = null; canvas.classList.remove('dragging'); });
  canvas.addEventListener('dblclick', resetView);
  window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && state.menuOpen) openMenu(false); });

  function rotateBy(dLon, dLat) {
    state.rotate[0] = ((state.rotate[0] + dLon + 540) % 360) - 180;
    if (isFlat()) { state.dirty = true; return; }   // flat map only spins around the pole
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
    if (e.key === 'm' && e.target === document.body) {
      const order = MODES.map((m) => m.mode);
      setMode(order[(order.indexOf(state.mode) + 1) % order.length]);
    }
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
    if (state.dirty) {
      state.dirty = false;
      const t0 = performance.now();
      draw();
      perf.total = performance.now() - t0;
    }
    requestAnimationFrame(frame);
  }

  layout();
  setMode(state.mode);
  loadMap();
  loadTexture();
  loadFonts();
  loadNightLights();
  loadSatellite(Number(ui.norad.value));
  requestAnimationFrame(frame);
})();
