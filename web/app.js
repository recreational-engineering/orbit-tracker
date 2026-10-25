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
  const MARK_STEP_S = 60;      // 1-minute travel-time marks
  const MARK_MAJOR = 5;        // every 5th mark (whole 5 minutes) is a longer tick
  const SPIN_PERIOD_S = 60;    // Earth-spin view: one turn on screen per minute
  // Orbit-line opacity: the line ahead is solid; the part already flown fades out
  // linearly until it is gone at −TRACK_SPAN_S. Used for line, ticks and hidden parts.
  const trackAlpha = (t) => (t >= 0 ? 1 : Math.max(0, 1 + t / TRACK_SPAN_S));
  const BEHIND_ALPHA = 0.28;   // hidden (behind-the-Earth) line and ticks: just a faint marking
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
    track: '#c8102e',                 // orbit line and its ticks: one dark, crisp red
    tickBehind: '#c4c8cc',            // hidden ticks: same grey as the hidden line
    footprint: '#ffc800',             // horizon circle: aviation-display amber/yellow
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
    mode: 'orbit',            // spin | home | orbit | pilot | flat (see MODES)
    spinT0: 0,                // Earth-spin view: time the spin started (performance.now ms)
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

  // d3 rotation [λ, φ, γ] (degrees) as a 3×3 matrix on Earth unit vectors; row 0 points at
  // the viewer (visible side of the globe: > 0), rows 1 / 2 are screen right / up. Cached for
  // the last rotation, since project3d() runs a few thousand times per frame.
  let rotKey = [NaN, NaN, NaN], rotM = null;
  function rotationMatrix(rot) {
    const g = rot[2] || 0;
    if (rot[0] === rotKey[0] && rot[1] === rotKey[1] && g === rotKey[2]) return rotM;
    const l = rot[0] / DEG, p = rot[1] / DEG, gg = g / DEG;
    const cl = Math.cos(l), sl = Math.sin(l), cp = Math.cos(p), sp = Math.sin(p), cg = Math.cos(gg), sg = Math.sin(gg);
    rotKey = [rot[0], rot[1], g];
    rotM = [
      cl * cp, -sl * cp, -sp,
      sl * cg - cl * sp * sg, cl * cg + sl * sp * sg, -cp * sg,
      cl * sp * cg + sl * sg, -sl * sp * cg + cl * sg, cp * cg,
    ];
    return rotM;
  }

  // Borders and graticule on the orthographic globe without d3's per-frame adaptive
  // resampling and clipping: each line is densified once into unit vectors (steps ≤ 2.5°,
  // so a straight chord stays within 0.1 px of the great-circle arc even at 720 px, finer
  // than d3's own 0.7 px resampling), then a frame only rotates the points and cuts the
  // lines where they cross the horizon.
  const LINE_STEP = 2.5 / DEG;
  const unitLineCache = new WeakMap();
  function unitLines(geom) {
    let out = unitLineCache.get(geom);
    if (out) return out;
    const lines = geom.type === 'LineString' ? [geom.coordinates] : geom.coordinates;   // MultiLineString
    out = lines.map((line) => {
      const pts = [];
      let px = 0, py = 0, pz = 0;
      line.forEach(([lo, la], i) => {
        const l = lo / DEG, p = la / DEG;
        const x = Math.cos(p) * Math.cos(l), y = Math.cos(p) * Math.sin(l), z = Math.sin(p);
        if (i > 0) {
          const ang = Math.acos(Math.max(-1, Math.min(1, px * x + py * y + pz * z)));
          const k = Math.ceil(ang / LINE_STEP);
          for (let j = 1; j < k; j++) {   // slerp: points along the great circle
            const a = Math.sin((1 - j / k) * ang) / Math.sin(ang), b = Math.sin(j / k * ang) / Math.sin(ang);
            pts.push(a * px + b * x, a * py + b * y, a * pz + b * z);
          }
        }
        pts.push(x, y, z);
        px = x; py = y; pz = z;
      });
      return Float64Array.from(pts);
    });
    unitLineCache.set(geom, out);
    return out;
  }
  function globeLinePath(geom) {
    const M = rotationMatrix(projection.rotate()), k = projection.scale(), [tx, ty] = projection.translate();
    const [m0, m1, m2, m3, m4, m5, m6, m7, m8] = M;
    for (const L of unitLines(geom)) {
      let pX = 0, pY = 0, pZ = 0, pVis = false;
      for (let i = 0; i < L.length; i += 3) {
        const x = L[i], y = L[i + 1], z = L[i + 2];
        const X = m0 * x + m1 * y + m2 * z, Y = m3 * x + m4 * y + m5 * z, Z = m6 * x + m7 * y + m8 * z;
        const vis = X > 0;
        if (i > 0 && vis !== pVis) {
          // Horizon crossing between the two points (X = 0 on the great circle).
          const t = pX / (pX - X), cY = pY + (Y - pY) * t, cZ = pZ + (Z - pZ) * t, n = Math.hypot(cY, cZ) || 1;
          const hx = tx + k * cY / n, hy = ty - k * cZ / n;
          if (pVis) ctx.lineTo(hx, hy); else ctx.moveTo(hx, hy);
        }
        if (vis) {
          if (pVis || i > 0) ctx.lineTo(tx + k * Y, ty - k * Z); else ctx.moveTo(tx + k * Y, ty - k * Z);
        }
        pX = X; pY = Y; pZ = Z; pVis = vis;
      }
    }
  }
  // Path of a line geometry in the current projection (adds to the current canvas path).
  const linePath = (geom) => (projection === ortho ? globeLinePath(geom) : path(geom));

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
      hiTiles.set(key, { col: bakeTile(px, tileLandMask(key)), used: hiFrame });   // see renderPilotPixels()
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

  // Land/sea for a high-res tile from the land-50m vectors (the raster's own white/tint is not
  // a reliable sea mark, e.g. Arctic seas are tinted like land). Rasterised once per tile
  // load at 360×360; 1 = land. ESP: precompute these offline as 1-bit tiles.
  const maskCanvas = document.createElement('canvas');
  maskCanvas.width = maskCanvas.height = HI.T;
  const maskCtx = maskCanvas.getContext('2d', { willReadFrequently: true });
  function tileLandMask(key) {
    const row = Math.floor(key / HI.COLS), col = key % HI.COLS, W = HI.COLS * HI.T, H = HI.ROWS * HI.T;
    const eq = d3.geoEquirectangular().scale(W / (2 * Math.PI)).translate([W / 2 - col * HI.T, H / 2 - row * HI.T])
      .clipExtent([[-2, -2], [HI.T + 2, HI.T + 2]]);
    maskCtx.clearRect(0, 0, HI.T, HI.T);
    if (state.land) {
      maskCtx.fillStyle = '#fff';
      maskCtx.beginPath(); d3.geoPath(eq, maskCtx)(state.land); maskCtx.fill();
    }
    const src = maskCtx.getImageData(0, 0, HI.T, HI.T).data, m = new Uint8Array(HI.T * HI.T);
    for (let i = 0; i < m.length; i++) m[i] = src[i * 4] > 127 ? 1 : 0;
    return m;
  }

  // High-res city lights for the pilot view: the same NASA 2012 night image at 13500×6750
  // (~2.7 km/px) scaled to 10800×5400 and cut on the same 15×30 grid as the terrain
  // (web/tiles/lights/<row>_<col>.jpg). Decoded to one byte per texel (LIGHTS_LUT) and only
  // requested for night-side pixels, so day-side passes load nothing extra.
  // 2×2 box-filter downsample of a one-byte-per-texel image (lights mip levels).
  function halve(px, w, h) {
    const w2 = w >> 1, h2 = h >> 1, out = new Uint8Array(w2 * h2);
    for (let y = 0; y < h2; y++) {
      const r0 = 2 * y * w, r1 = r0 + w;
      for (let x = 0; x < w2; x++) {
        const i = 2 * x;
        // Average with a mild gain, so averaged-out small towns don't fade too much.
        out[y * w2 + x] = Math.min(255, (px[r0 + i] + px[r0 + i + 1] + px[r1 + i] + px[r1 + i + 1]) * 0.32);
      }
    }
    return out;
  }
  const LIGHTS_LUT = new Uint8Array(256).map((_, r) => 255 * Math.min(1, Math.pow(Math.max(0, (r - 45) / 170), 1.1)));
  const litTiles = new Map();      // key → {px: Uint8Array T×T, used}
  const litPending = new Set();
  function requestLitTile(key) {
    if (litPending.has(key) || litPending.size >= HI.MAX_LOADS) return;
    litPending.add(key);
    const img = new Image();
    img.onload = () => {
      hiCtx.drawImage(img, 0, 0);
      const src = hiCtx.getImageData(0, 0, HI.T, HI.T).data, px = new Uint8Array(HI.T * HI.T);
      for (let i = 0; i < px.length; i++) px[i] = LIGHTS_LUT[src[i * 4]];
      litTiles.set(key, { px, px1: halve(px, HI.T, HI.T), used: hiFrame });   // + one mip level
      if (litTiles.size > HI.CACHE) {
        let oldK = -1, oldU = Infinity;
        for (const [k, t] of litTiles) if (t.used < oldU) { oldU = t.used; oldK = k; }
        litTiles.delete(oldK);
      }
      litPending.delete(key);
      state.dirty = true;
    };
    img.onerror = () => litPending.delete(key);
    img.src = `tiles/lights/${Math.floor(key / HI.COLS)}_${key % HI.COLS}.jpg`;
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
    for (let i = 0; i < px.length; i++) px[i] = LIGHTS_LUT[src[i * 4]];
    // Mip pyramid (2048 → 256 wide): sample the level whose texel is at least as large as the
    // pixel's ground footprint, so tiny city lights don't flicker while the view moves.
    const mips = [{ w: c.width, h: c.height, px }];
    while (mips.length < 4) {
      const m = mips[mips.length - 1];
      mips.push({ w: m.w >> 1, h: m.h >> 1, px: halve(m.px, m.w, m.h) });
    }
    state.lightMips = mips;
    state.lights = { w: c.width, h: c.height, px };
    state.dirty = true;
  }

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

  // ---------- per-pixel globe renderer ----------
  // Inverse projection of every screen pixel onto the Earth, one pass into one buffer: sea /
  // terrain (land mask), coastline (mask edges), night darkening and city lights.
  //
  // Speed (web): everything that does not change from frame to frame is precomputed.
  //  - surfaceColors(): sea / terrain / lakes / coastline baked into one RGBA word per texel
  //    of the 2048×1024 grid shared by land mask, terrain and lights (high-res tiles likewise
  //    on load), so a pixel is one lookup.
  //  - viewGeometry(): globe and flat views keep the per-pixel inverse projection (texel row,
  //    longitude before the λ turn, surface normal, lights level of detail) until tilt φ, roll γ
  //    or the size change. Earth spin and ground view only turn λ, so a frame there is just a
  //    shift of the texel column and a 3-term dot product for the sun.
  //  - pilotRays(): the camera-space ray of each pixel, normalised once per display size.
  //  - Output is written as 32-bit words; the buffer is cleared only when the footprint changes.
  const TEX_W = 2048, TEX_H = 1024;   // = MASK_W × MASK_H = terrain = lights level 0
  const TWO_PI = 2 * Math.PI;
  const packRgb = (R, G, B) => {   // floats → clamped, rounded RGBA word (little endian)
    R = R >= 255 ? 255 : (R + 0.5) | 0; G = G >= 255 ? 255 : (G + 0.5) | 0; B = B >= 255 ? 255 : (B + 0.5) | 0;
    return (0xff000000 | (B << 16) | (G << 8) | R) >>> 0;
  };
  const [KR, KG, KB] = C.coastRgb;
  const coastMix = (R, G, B) => packRgb(R * 0.45 + KR * 0.55, G * 0.45 + KG * 0.55, B * 0.45 + KB * 0.55);

  let surface = null;   // {tex, mask, px: Uint32Array TEX_W × TEX_H}
  function surfaceColors() {
    const tex = state.tex, mask = state.mask;
    if (surface && surface.tex === tex && surface.mask === mask) return surface.px;
    const px = new Uint32Array(TEX_W * TEX_H), [sr, sg, sb] = C.sea, [lr, lg, lb] = C.landRgb;
    const sea = packRgb(sr, sg, sb), seaCoast = coastMix(sr, sg, sb);
    const t = tex && tex.w === TEX_W && tex.h === TEX_H ? tex.px : null;
    if (!mask) px.fill(sea);
    else {
      for (let mv = 0; mv < TEX_H; mv++) {
        for (let mu = 0; mu < TEX_W; mu++) {
          const mi = mv * TEX_W + mu, land = mask[mi];
          const coast = land !== mask[mv * TEX_W + ((mu + 1) % TEX_W)] || (mv + 1 < TEX_H && land !== mask[mi + TEX_W]);
          let R = sr, G = sg, B = sb;
          if (land) {
            if (t) { const ti = mi * 4; if (t[ti + 3] !== 0) { R = t[ti]; G = t[ti + 1]; B = t[ti + 2]; } }   // else: lake
            else { R = lr; G = lg; B = lb; }
          }
          px[mi] = R === sr && G === sg && B === sb ? (coast ? seaCoast : sea) : coast ? coastMix(R, G, B) : packRgb(R, G, B);
        }
      }
    }
    surface = { tex, mask, px };
    return px;
  }

  // High-res tile: colour from the raster, land / sea and coastline from the vector mask.
  function bakeTile(t, m) {
    const T = HI.T, out = new Uint32Array(T * T), [sr, sg, sb] = C.sea;
    for (let ty = 0; ty < T; ty++) {
      for (let tx = 0; tx < T; tx++) {
        const mi = ty * T + tx, land = m[mi], ti = mi * 4;
        let R = sr, G = sg, B = sb;
        if (land && t[ti + 3] !== 0) { R = t[ti]; G = t[ti + 1]; B = t[ti + 2]; }
        const coast = (tx + 1 < T && m[mi + 1] !== land) || (ty + 1 < T && m[mi + T] !== land);
        out[mi] = coast ? coastMix(R, G, B) : packRgb(R, G, B);
      }
    }
    return out;
  }

  // atan2 without the library call: one range reduction to |t| ≤ tan 15°, then the Taylor
  // series (error < 3e-9 rad, ~1/1000 of a texel of the finest tiles).
  const SQRT3 = Math.sqrt(3), TAN15 = 2 - SQRT3;
  function fastAtan2(y, x) {
    const ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
    const swap = ay > ax;
    if (!swap && ax === 0) return 0;
    let t = swap ? ax / ay : ay / ax, off = 0;
    if (t > TAN15) { t = (t * SQRT3 - 1) / (SQRT3 + t); off = Math.PI / 6; }
    const t2 = t * t;
    let r = off + t * (1 - t2 * (1 / 3 - t2 * (1 / 5 - t2 * (1 / 7 - t2 * (1 / 9 - t2 / 11)))));
    if (swap) r = Math.PI / 2 - r;
    if (x < 0) r = Math.PI - r;
    return y < 0 ? -r : r;
  }
  // Texel row floor(H · min(0.9999, 0.5 − asin(z)/π)) of a point with sin(latitude) = z, without
  // asin: a guess from a table over z, corrected against the exact row boundaries cos(jπ/H).
  function rowTable(H) {
    const Q = 1 << 16, bound = new Float64Array(H + 1), guess = new Int32Array(Q + 1);
    for (let j = 0; j <= H; j++) bound[j] = Math.cos(j * Math.PI / H);
    for (let q = 0; q <= Q; q++) guess[q] = Math.min(H - 1, Math.floor(H * (0.5 - Math.asin(Math.min(1, q / Q * 2 - 1)) / Math.PI)));
    return { H, Q2: Q / 2, bound, guess };
  }
  function latRow(z, tb) {
    const b = tb.bound, last = tb.H - 1;
    let j = tb.guess[((z + 1) * tb.Q2) | 0];
    while (j > 0 && z > b[j]) j--;
    while (j < last && z <= b[j + 1]) j++;
    return j;
  }
  const ROWS_TEX = rowTable(TEX_H), ROWS_HI = rowTable(HI.ROWS * HI.T);

  // Ground footprint (km) → global lights mip level: the level whose texel covers it.
  const LIGHT_TEXEL_KM = 2 * Math.PI * EARTH_R_KM / 2048;   // global lights level 0 (~19.5 km)
  const lightLevel = (fpKm) => {
    const q = fpKm / LIGHT_TEXEL_KM;   // = min(3, max(0, ceil(log2(q))))
    return q <= 1 ? 0 : q <= 2 ? 1 : q <= 4 ? 2 : 3;
  };

  // Pixels of the globe / flat disc for one (size, radius, centre): output index, disc
  // coordinates Xs, Ys, Z = √(1 − Xs² − Ys²), and the lights mip level, which depends only on
  // the place on the disc (footprint stretched toward the limb).
  const discs = new Map();   // key → disc (one per view kind and resolution in use)
  function discPixels(flat, s, r, cx, cy) {
    const key = `${flat ? 'f' : 'g'}|${s}|${r}|${cx}|${cy}`;
    let disc = discs.get(key);
    if (disc) return disc;
    if (discs.size > 5) discs.clear();
    const x0 = Math.max(0, Math.floor(cx - r)), x1 = Math.min(s, Math.ceil(cx + r));
    const y0 = Math.max(0, Math.floor(cy - r)), y1 = Math.min(s, Math.ceil(cy + r));
    const list = [];
    for (let py = y0; py < y1; py++) {
      const Ys = (cy - py - 0.5) / r;
      for (let pxl = x0; pxl < x1; pxl++) {
        const Xs = (pxl + 0.5 - cx) / r;
        if (Xs * Xs + Ys * Ys <= 1) list.push(py * s + pxl);
      }
    }
    const n = list.length, idx = Int32Array.from(list);
    const Xa = new Float64Array(n), Ya = new Float64Array(n), Za = new Float64Array(n), lod = new Uint8Array(n);
    const kmPerPxGlobe = EARTH_R_KM / r, flatLod = lightLevel(Math.PI * EARTH_R_KM / r);
    for (let i = 0; i < n; i++) {
      const py = Math.floor(idx[i] / s), pxl = idx[i] - py * s;
      const Xs = (pxl + 0.5 - cx) / r, Ys = (cy - py - 0.5) / r, Z = Math.sqrt(1 - (Xs * Xs + Ys * Ys));
      Xa[i] = Xs; Ya[i] = Ys; Za[i] = Z;
      lod[i] = flat ? flatLod : lightLevel(kmPerPxGlobe / (Z > 0.15 ? Z : 0.15));
    }
    disc = { key, n, idx, Xa, Ya, Za, lod };
    discs.set(key, disc);
    return disc;
  }

  // Per-pixel geometry of the globe / flat view for one disc and (φ, γ); λ is applied per frame.
  //   ub  longitude before the λ turn, in texels     row  texel row (fv · TEX_H)
  //   nP, nQ, nZ  surface normal: Earth-fixed normal = (nP·cosλ + nQ·sinλ, nQ·cosλ − nP·sinλ, nZ)
  // (+ idx, lod from discPixels())
  let geo = null;
  function viewGeometry(flat, s, r, cx, cy, phi, gam) {
    const D = discPixels(flat, s, r, cx, cy);
    const key = flat ? D.key : `${D.key}|${phi}|${gam}`;
    if (geo && geo.key === key) return geo;
    const n = D.n;
    let g = geo;
    if (!g || g.cap < n) {   // reused while dragging: no per-frame allocation
      g = { cap: n, ub: new Float64Array(n), row: new Int32Array(n), nP: new Float32Array(n), nQ: new Float32Array(n), nZ: new Float32Array(n) };
    }
    const { ub, row, nP, nQ, nZ } = g, { Xa, Ya, Za } = D;
    const U = TEX_W / TWO_PI;
    if (flat) {
      for (let i = 0; i < n; i++) {
        // Azimuthal equidistant around the north pole: radius ∝ polar distance,
        // Greenwich pointing down, longitude increasing counter-clockwise.
        const Xs = Xa[i], Ys = Ya[i];
        const lat = Math.PI / 2 - Math.sqrt(Xs * Xs + Ys * Ys) * Math.PI, lon = Math.atan2(Xs, -Ys);
        const cl = Math.cos(lat);
        nP[i] = cl * Math.cos(lon); nQ[i] = cl * Math.sin(lon); nZ[i] = Math.sin(lat);
        ub[i] = lon * U;
        row[i] = Math.floor(Math.min(0.9999, 0.5 - lat / Math.PI) * TEX_H);
      }
    } else {
      const cp = Math.cos(phi), sp = Math.sin(phi), cg = Math.cos(gam), sg = Math.sin(gam);
      for (let i = 0; i < n; i++) {
        // Undo d3's [λ, φ, γ] rotation: spin back by γ, tilt back by φ (λ: per frame).
        const Xs = Xa[i], Ys = Ya[i], Z = Za[i];
        const X = Xs * cg + Ys * sg, Y = Ys * cg - Xs * sg;
        const xr = Z * cp + Y * sp, zr = Y * cp - Z * sp;
        nP[i] = xr; nQ[i] = X; nZ[i] = zr;
        ub[i] = fastAtan2(X, xr) * U;
        row[i] = latRow(zr > 1 ? 1 : zr < -1 ? -1 : zr, ROWS_TEX);
      }
    }
    g.n = n; g.idx = D.idx; g.lod = D.lod; g.key = key;
    geo = g;
    return g;
  }

  // Pilot view: unit ray of each screen pixel in camera axes (right, up, forward).
  const raysBySize = new Map();
  function pilotRays(s) {
    let rays = raysBySize.get(s);
    if (rays) return rays;
    if (raysBySize.size > 3) raysBySize.clear();
    const c = s / 2, tanH = Math.tan(PILOT_HALF_FOV), list = [];
    for (let py = 0; py < s; py++) {
      const Ys = (c - py - 0.5) / c;
      for (let pxl = 0; pxl < s; pxl++) {
        const Xs = (pxl + 0.5 - c) / c;
        if (Xs * Xs + Ys * Ys <= 1) list.push(py * s + pxl);
      }
    }
    const n = list.length, idx = Int32Array.from(list);
    const ra = new Float32Array(n), ru = new Float32Array(n), rf = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const py = Math.floor(idx[i] / s), pxl = idx[i] - py * s;
      const a = (pxl + 0.5 - c) / c * tanH, b = (c - py - 0.5) / c * tanH, k = 1 / Math.sqrt(1 + a * a + b * b);
      ra[i] = a * k; ru[i] = b * k; rf[i] = k;
    }
    rays = { s, n, idx, ra, ru, rf };
    raysBySize.set(s, rays);
    return rays;
  }

  // Renders into a buffer of scale × the display size (scale < 1 only while the view moves on a
  // machine too slow for full resolution, see pixelScale()); returns the buffer canvas.
  const globeBufs = new Map();   // buffer size → {canvas, ctx, img, u32, footprint}
  function renderGlobePixels(withNight, scale = 1) {
    const s = Math.round(state.size * scale), k = s / state.size, flat = isFlat(), cam = isPilot() ? state.cam : null;
    const r = (cam ? state.size / 2 : discRadius()) * k;   // pilot view fills the whole round screen
    const cx = projection.translate()[0] * k, cy = projection.translate()[1] * k;
    let buf = globeBufs.get(s);
    if (!buf) {
      if (globeBufs.size > 3) globeBufs.clear();   // display size changed
      const canvas = document.createElement('canvas');
      canvas.width = canvas.height = s;
      const bctx = canvas.getContext('2d'), img = bctx.createImageData(s, s);
      buf = { canvas, ctx: bctx, img, u32: new Uint32Array(img.data.buffer), footprint: '' };
      globeBufs.set(s, buf);
    }
    // Pixels outside the disc stay transparent; clear only when the disc changes.
    const footprint = `${r}|${cx}|${cy}`;
    if (footprint !== buf.footprint) { buf.u32.fill(0); buf.footprint = footprint; }
    const out = buf.u32, surf = surfaceColors();
    const lights = withNight ? state.lights : null;
    const mips = lights && lights.w === TEX_W && lights.h === TEX_H ? state.lightMips : null;   // texel grid shared with surf
    // Sun direction in Earth-fixed coordinates.
    const [slon, slat] = subsolarPoint(simTime()).map((d) => d / DEG);
    const sx = Math.cos(slat) * Math.cos(slon), sy = Math.cos(slat) * Math.sin(slon), sz = Math.sin(slat);
    if (cam) renderPilotPixels(out, surf, withNight, mips, s, cam, sx, sy, sz);
    else {
      const lam = state.rotate[0] / DEG, phi = state.rotate[1] / DEG, gam = (state.rotate[2] || 0) / DEG;
      const g = viewGeometry(flat, s, r, cx, cy, flat ? 0 : phi, flat ? 0 : gam);
      const cl0 = Math.cos(lam), sl0 = Math.sin(lam);
      const sunP = sx * cl0 - sy * sl0, sunQ = sx * sl0 + sy * cl0;   // sun · normal = nP·sunP + nQ·sunQ + nZ·sz
      // Column = (ub − λ·TEX_W/2π + TEX_W/2) mod TEX_W; + TEX_W keeps it positive for the mask.
      const lamU = TEX_W / 2 + TEX_W - lam * TEX_W / TWO_PI;
      const { n, idx, ub, row, lod, nP, nQ, nZ } = g;
      const m0 = mips && mips[0].px, m1 = mips && mips[1].px, m2 = mips && mips[2].px, m3 = mips && mips[3].px;
      for (let i = 0; i < n; i++) {
        const col = ((ub[i] + lamU) | 0) & (TEX_W - 1), rw = row[i];
        let c = surf[rw * TEX_W + col];
        if (withNight) {
          // Sun elevation → night factor, with a soft twilight band (~+6° … −7°).
          const sunDot = nP[i] * sunP + nQ[i] * sunQ + nZ[i] * sz;
          if (sunDot < 0.1) {
            let t = (sunDot - 0.1) / -0.22; if (t > 1) t = 1;
            const nf = t * t * (3 - 2 * t), a = 0.78 * nf, ia = 1 - a;
            let R = (c & 255) * ia + 4 * a, G = ((c >> 8) & 255) * ia + 8 * a, B = ((c >> 16) & 255) * ia + 28 * a;
            if (m0) {
              const lv = lod[i];
              let L = lv === 0 ? m0[rw * TEX_W + col]
                : lv === 1 ? m1[(rw >> 1) * (TEX_W >> 1) + (col >> 1)]
                  : lv === 2 ? m2[(rw >> 2) * (TEX_W >> 2) + (col >> 2)]
                    : m3[(rw >> 3) * (TEX_W >> 3) + (col >> 3)];
              L *= nf;
              R += L; G += L * 0.8; B += L * 0.45;
            }
            c = packRgb(R, G, B);
          }
        }
        out[idx[i]] = c;
      }
    }
    buf.ctx.putImageData(buf.img, 0, 0);
    return buf.canvas;
  }

  // Pilot camera: ray = rt·a + u·b + f·c per pixel (pilotRays); Earth is a sphere of radius
  // EARTH_R_KM. Sky: black space with a thin atmosphere glow along the limb, lit on the day
  // side. Ground: high-res tiles near the camera, the global texture further out, aerial haze
  // toward the horizon, graticule every 10°.
  const GLOW_KM = 2500, GLOW = Float64Array.from({ length: GLOW_KM + 1 }, (_, h) => Math.exp(-h / 90));
  function renderPilotPixels(out, surf, withNight, mips, s, cam, sx, sy, sz) {
    const { n, idx, ra, ru, rf } = pilotRays(s);
    const tanH = Math.tan(PILOT_HALF_FOV), pixAng = 2 * tanH / s;
    const gm = cam.gmst, cgm = Math.cos(gm), sgm = Math.sin(gm);
    const RE = EARTH_R_KM, [c0, c1, c2] = cam.c, cc2 = c0 * c0 + c1 * c1 + c2 * c2, cc2R = cc2 - RE * RE;
    const [f0, f1, f2] = cam.f, [u0, u1, u2] = cam.u, [r0, r1, r2] = cam.rt;
    const tHorizon = Math.sqrt(Math.max(0, cc2R));
    const grat = ui.graticule.checked;
    const hiMaxT = HI.FOOTPRINT_KM / pixAng;   // use tiles while hitT < this
    const T = HI.T, HW = HI.COLS * T, HH = HI.ROWS * T;
    let hiKey = -1, hiTile = null, litKey = -1, litTile = null;
    hiFrame++;
    // Sun in the camera's (ECI) frame, for the sky glow.
    const ssx = sx * cgm - sy * sgm, ssy = sx * sgm + sy * cgm;
    const m0 = mips && mips[0].px, m1 = mips && mips[1].px, m2 = mips && mips[2].px, m3 = mips && mips[3].px;
    const iRE = 1 / RE;
    for (let i = 0; i < n; i++) {
      const a = ra[i], b = ru[i], cf = rf[i];
      const dx = r0 * a + u0 * b + f0 * cf, dy = r1 * a + u1 * b + f1 * cf, dz = r2 * a + u2 * b + f2 * cf;
      const bb = c0 * dx + c1 * dy + c2 * dz;
      const disc = bb * bb - cc2R;
      const t = disc >= 0 ? -bb - Math.sqrt(disc) : -1;
      if (t <= 0) {
        const tc = bb < 0 ? -bb : 0;                     // closest approach along the ray
        const qx = c0 + tc * dx, qy = c1 + tc * dy, qz = c2 + tc * dz;
        const qn = Math.sqrt(qx * qx + qy * qy + qz * qz);
        // glow = exp(−height / 90 km), linearly interpolated from a 1 km table.
        const hq = qn > RE ? qn - RE : 0, hi = hq | 0;
        const glow = hi < GLOW_KM ? GLOW[hi] + (GLOW[hi + 1] - GLOW[hi]) * (hq - hi) : 0;
        const sd = (qx * ssx + qy * ssy + qz * sz) / qn;
        let lt = (sd + 0.2) / 0.35; lt = lt < 0 ? 0 : lt > 1 ? 1 : lt;
        const lit = lt * lt * (3 - 2 * lt);
        out[idx[i]] = packRgb(4 + 90 * glow * lit, 6 + 150 * glow * lit, 14 + 255 * glow * (0.15 + 0.85 * lit));
        continue;
      }
      const nx = (c0 + t * dx) * iRE, ny = (c1 + t * dy) * iRE, nz = (c2 + t * dz) * iRE;
      const ex = nx * cgm + ny * sgm, ey = -nx * sgm + ny * cgm, ez = nz;   // ECI → Earth-fixed
      const ezc = ez > 1 ? 1 : ez < -1 ? -1 : ez;
      const lon = fastAtan2(ey, ex);
      let fu = lon / TWO_PI + 0.5; if (fu >= 1) fu -= 1;

      let c = -1, hiIdx = -1, key = -1, grow = -1, gcol = 0;   // grow / gcol: global texel, when needed
      if (t < hiMaxT) {
        // High-res tile lookup; consecutive pixels mostly hit the same tile.
        const X = fu * HW, Yi = latRow(ezc, ROWS_HI);
        const col = (X / T) | 0, row = (Yi / T) | 0;
        key = row * HI.COLS + col;
        hiIdx = (Yi - row * T) * T + ((X | 0) - col * T);   // texel inside the tile
        if (key !== hiKey) {
          hiKey = key; hiTile = hiTiles.get(key) || null;
          if (hiTile) hiTile.used = hiFrame; else requestHiTile(key);
        }
        if (hiTile) c = hiTile.col[hiIdx];
      }
      if (c === -1) { gcol = Math.floor(fu * TEX_W); grow = latRow(ezc, ROWS_TEX); c = surf[grow * TEX_W + gcol]; }
      let R = c & 255, G = (c >> 8) & 255, B = (c >> 16) & 255;
      const sunDot = ex * sx + ey * sy + ez * sz;
      if (withNight && sunDot < 0.1) {
        let tt = (sunDot - 0.1) / -0.22; if (tt > 1) tt = 1;
        const nf = tt * tt * (3 - 2 * tt), aa = 0.78 * nf, ia = 1 - aa;
        R = R * ia + 4 * aa; G = G * ia + 8 * aa; B = B * ia + 28 * aa;
        if (m0) {
          // Level of detail by the pixel's ground footprint: high-res tile (≤ 5 km), its
          // half-res mip (≤ 10 km), else the global mip whose texel covers the footprint.
          const cosi = -(nx * dx + ny * dy + nz * dz);   // incidence of the view ray on the ground
          const fpKm = t * pixAng / (cosi > 0.15 ? cosi : 0.15);
          let L = -1;
          if (key >= 0 && fpKm < 10) {
            if (key !== litKey) {
              litKey = key; litTile = litTiles.get(key) || null;
              if (litTile) litTile.used = hiFrame; else requestLitTile(key);
            }
            if (litTile) {
              if (fpKm < 5) L = litTile.px[hiIdx];
              else { const ty = (hiIdx / T) | 0, tx = hiIdx - ty * T; L = litTile.px1[(ty >> 1) * (T >> 1) + (tx >> 1)]; }
            }
          }
          if (L < 0) {
            if (grow < 0) { gcol = Math.floor(fu * TEX_W); grow = latRow(ezc, ROWS_TEX); }
            const lv = lightLevel(fpKm);
            L = lv === 0 ? m0[grow * TEX_W + gcol]
              : lv === 1 ? m1[(grow >> 1) * (TEX_W >> 1) + (gcol >> 1)]
                : lv === 2 ? m2[(grow >> 2) * (TEX_W >> 2) + (gcol >> 2)]
                  : m3[(grow >> 3) * (TEX_W >> 3) + (gcol >> 3)];
          }
          L *= nf;
          R += L; G += L * 0.8; B += L * 0.45;
        }
      }
      if (grat) {
        // Graticule every 10°, line width ≈ one pixel's footprint on the ground.
        const fp = t * pixAng / RE * DEG * 0.8, cosLat = Math.sqrt(1 - ezc * ezc);
        const la = fastAtan2(ezc, cosLat) * DEG, lo = lon * DEG;
        const dLat = Math.abs(la - Math.round(la / 10) * 10);
        const dLon = Math.abs(lo - Math.round(lo / 10) * 10) * cosLat;
        if (dLat < fp || dLon < fp) { R = R * 0.8 + 51; G = G * 0.8 + 51; B = B * 0.8 + 51; }
      }
      // Aerial haze toward the horizon.
      let lt = (sunDot + 0.12) / 0.22; lt = lt < 0 ? 0 : lt > 1 ? 1 : lt;
      const lit = lt * lt * (3 - 2 * lt);
      const h1 = t < tHorizon ? t / tHorizon : 1, hz = 0.6 * h1 * h1 * h1 * h1 * h1, ih = 1 - hz;
      R = R * ih + (12 + 120 * lit) * hz; G = G * ih + (18 + 160 * lit) * hz; B = B * ih + (40 + 210 * lit) * hz;
      out[idx[i]] = packRgb(R, G, B);
    }
  }

  // Element sources, first that answers wins: the local dev server's CelesTrak proxy; on the
  // static site (GitHub Pages) the element sets baked in by the deploy workflow, then
  // CelesTrak directly for any other object. The last good set is kept in localStorage so a
  // reload while offline or while CelesTrak is down still shows the object.
  const LOCAL_DEV = /^(localhost|127\.0\.0\.1|\[::1\])$/.test(location.hostname);
  const gpSources = (catnr) => (LOCAL_DEV ? [`/api/gp?catnr=${catnr}`] : [
    `gp/${catnr}.json`,
    `https://celestrak.org/NORAD/elements/gp.php?CATNR=${catnr}&FORMAT=JSON`,
  ]);

  async function fetchGp(catnr) {
    let err = new Error('No element set found');
    for (const url of gpSources(catnr)) {
      try {
        const res = await fetch(url, { signal: AbortSignal.timeout(10000) });
        const data = await res.json();   // CelesTrak answers errors in plain text
        if (res.ok && Array.isArray(data) && data.length) return data;
        if (data.error) err = new Error(data.error);
      } catch (e) { err = e; }
    }
    try {
      const saved = JSON.parse(localStorage.getItem(`gp_${catnr}`));
      if (Array.isArray(saved) && saved.length) return saved;
    } catch (e) { /* storage unavailable */ }
    throw err;
  }

  async function loadSatellite(catnr) {
    ui.status.className = 'status';
    ui.status.textContent = `Fetching elements for ${catnr}…`;
    try {
      const data = await fetchGp(catnr);
      try { localStorage.setItem(`gp_${catnr}`, JSON.stringify(data)); } catch (e) { /* ignore */ }
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
      state.marks = [];
      state.nextPass = null;
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

    // 1-minute marks pinned to absolute times (so they don't jump as "now" advances),
    // in the Earth-fixed frame: the ground point the object passes at that moment.
    const marks = [];
    const step = MARK_STEP_S * 1000, t0 = now.getTime();
    for (let ms = Math.ceil((t0 - TRACK_SPAN_S * 1000) / step) * step; ms <= t0 + TRACK_SPAN_S * 1000; ms += step) {
      const a = propagateEci(new Date(ms)), b = propagateEci(new Date(ms + 10000));
      if (a && b) marks.push({ t: (ms - t0) / 1000, major: Math.round(ms / step) % MARK_MAJOR === 0, a, b });
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
    if (state.mode === 'spin') {
      // Seen from the equatorial plane, north up, turning about the real axis: the surface
      // drifts left → right (eastward rotation), one turn per SPIN_PERIOD_S.
      const t = (performance.now() - state.spinT0) / 1000;
      return [(((-HOME.lon + 360 * t / SPIN_PERIOD_S) + 540) % 360) - 180, 0, 0];
    }
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
    // Base frame: up = radial, forward = horizontal part of the velocity (prograde).
    const up = vnorm(c);
    const vv = [v.x, v.y, v.z], vu = vdot(vv, up);
    let f = vnorm(vv.map((x, i) => x - vu * up[i]));
    let rt = vcross(f, up);
    // Look-around: yaw about the local vertical, then pitch about the (horizontal) right
    // axis. The camera never rolls, and `hf` stays a well-defined horizontal heading even
    // when looking straight down or up.
    const yaw = -state.rotate[0] / DEG, pitch = -state.rotate[1] / DEG;
    const hf = f.map((x, i) => x * Math.cos(yaw) + rt[i] * Math.sin(yaw));
    rt = rt.map((x, i) => x * Math.cos(yaw) - f[i] * Math.sin(yaw));
    f = hf.map((x, i) => x * Math.cos(pitch) + up[i] * Math.sin(pitch));
    const u = up.map((x, i) => x * Math.cos(pitch) - hf[i] * Math.sin(pitch));
    state.cam = { c, f, u, rt, up, hf, gmst: state.now.gmst };
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
    // Rotated unit vector: ex = screen right, ey = screen up, ez = toward the viewer.
    const M = rotationMatrix(projection.rotate());
    const la = lat / DEG, lo = lon / DEG, cla = Math.cos(la);
    const ux = cla * Math.cos(lo), uy = cla * Math.sin(lo), uz = Math.sin(la);
    const r = projection.scale();
    const k = r * (1 + (Number(ui.exag.value) * altKm) / EARTH_R_KM);
    const [cx, cy] = projection.translate();
    const ex = M[3] * ux + M[4] * uy + M[5] * uz;
    const ey = M[6] * ux + M[7] * uy + M[8] * uz;
    const ez = M[0] * ux + M[1] * uy + M[2] * uz;
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
    drawGlobePixels();

    drawMapLines();

    drawFootprint();
    drawBehindEarth();

    if (isFlat()) drawFlatShading(cx, cy, r); else drawGlobeShading(cx, cy, r);
    ctx.restore();

    if (ui.reticle.checked && !isFlat()) drawReticle(cx, cy, r);
    drawHome();
    drawTrack();
    drawMarkTicks();
    drawSatellite();
    drawHud();
    if (state.menuOpen) drawMenu();
  }

  // Borders and graticule. Globe: stroked straight onto the display (cheap, globeLinePath()).
  // Flat map: d3's azimuthal path is costly and the map only turns while dragged, so the lines
  // live in a layer that is redrawn only when the turn, the size or the toggles change.
  function strokeMapLines(g, lp) {
    if (ui.borders.checked && state.borders) {
      g.beginPath(); lp(state.borders);
      g.setLineDash([3, 2]);
      g.strokeStyle = C.border; g.lineWidth = 0.7; g.stroke();
      g.setLineDash([]);
    }
    if (ui.graticule.checked) {
      g.beginPath(); lp(graticule);
      g.strokeStyle = C.graticule; g.lineWidth = 0.6; g.stroke();
    }
  }
  let flatLines = null;
  function drawMapLines() {
    if (!isFlat()) { strokeMapLines(ctx, linePath); return; }
    const s = state.size;
    const key = `${s}|${state.rotate.join()}|${ui.borders.checked}|${ui.graticule.checked}|${!!state.borders}`;
    if (!flatLines || flatLines.key !== key) {
      let c = flatLines && flatLines.c;
      if (!c || c.width !== s) { c = document.createElement('canvas'); c.width = c.height = s; }
      const g = c.getContext('2d');
      g.clearRect(0, 0, s, s);
      strokeMapLines(g, d3.geoPath(projection, g));
      flatLines = { key, c };
    }
    ctx.drawImage(flatLines.c, 0, 0);
  }

  // Per-pixel layer onto the display, at the resolution pixelScale() picked for this frame.
  function drawGlobePixels() {
    const t0 = performance.now(), s = state.size, scale = frameScale;
    const buf = renderGlobePixels(ui.night.checked, scale);
    if (buf.width === s) ctx.drawImage(buf, 0, 0);
    else ctx.drawImage(buf, 0, 0, buf.width, buf.width, 0, 0, s, s);   // smooth upscale
    perf.pixels = performance.now() - t0;
    perf.scale = scale;
    // Cost per buffer pixel while the view moves, per kind of view (smoothed). The first frame
    // after a change of resolution also rebuilds its caches, so it is left out.
    if (frameMoving && buf.width === lastBufWidth) {
      const kind = viewKind(state.mode), ms = perf.pixels / (buf.width * buf.width);
      quality[kind] = quality[kind] ? quality[kind] * 0.7 + ms * 0.3 : ms;
    }
    lastBufWidth = buf.width;
  }

  // Adaptive resolution of the per-pixel layer. While the view moves (drag, inertia, glide
  // back, Earth spin) a frame whose pixel pass would take longer than PIXEL_BUDGET_MS renders
  // the layer at 3/4 or 1/2 resolution and scales it up smoothly; once the view rests, it is
  // drawn at full resolution again. A fast machine always stays at full resolution; ?fullres
  // in the URL turns the adaptation off.
  const PIXEL_BUDGET_MS = 11, ADAPTIVE = !/[?&]fullres\b/.test(location.search);
  const quality = {};          // view kind → smoothed time per buffer pixel while moving (ms)
  let frameScale = 1, frameMoving = false, lastBufWidth = 0;
  function pixelScale(moving) {
    if (!moving || !ADAPTIVE) return 1;
    const perPx = quality[viewKind(state.mode)];
    if (!perPx) return frameScale;
    const full = perPx * state.size * state.size;
    // Step down at once when over budget; step up only with some headroom (no flicker).
    for (const sc of [1, 0.75, 0.5]) {
      const est = full * sc * sc;
      if (est <= PIXEL_BUDGET_MS * (sc > frameScale ? 0.85 : 1)) return sc;
    }
    return 0.5;
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
    if (state.cam) drawGlobePixels();
    const rim = ctx.createRadialGradient(c, c, c * 0.75, c, c, c);
    rim.addColorStop(0, 'rgba(0,0,0,0)');
    rim.addColorStop(1, 'rgba(0,0,0,.45)');
    ctx.fillStyle = rim;
    ctx.fillRect(0, 0, s, s);
    ctx.restore();
    if (state.cam) {
      drawHome();
      drawPilotTrack();
      drawPilotHud();
    }
    drawHud();
    if (state.menuOpen) drawMenu();
  }

  // Prograde symbol (as on a flight director): circle with three stubs, at the
  // prograde direction (screen centre unless the user is looking around).
  function drawProgradeMarker() {
    const s = state.size, c = s / 2, cam = state.cam, v = state.now.v;
    const f = vnorm([v.x, v.y, v.z]);
    const vx = vdot(f, cam.rt), vy = vdot(f, cam.u), vz = vdot(f, cam.f);
    const k = c / Math.tan(PILOT_HALF_FOV), R = s * 0.2, r = s / 48;   // R: clear of the heading tape and the side boxes
    let x, y, clamped = false;
    if (vz > 0.05) { x = c + k * vx / vz; y = c - k * vy / vz; }
    if (vz <= 0.05 || Math.hypot(x - c, y - c) > R) {
      // Off the HUD: pin the marker to the HUD edge in the direction of travel.
      let dx = vx, dy = -vy; const n = Math.hypot(dx, dy);
      if (n < 1e-6) { dx = 0; dy = 1; } else { dx /= n; dy /= n; }
      x = c + dx * R; y = c + dy * R; clamped = true;
    }
    const shape = () => {
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
      ctx.moveTo(x - r, y); ctx.lineTo(x - r * 2, y);
      ctx.moveTo(x + r, y); ctx.lineTo(x + r * 2, y);
      ctx.moveTo(x, y - r); ctx.lineTo(x, y - r * 1.8);
    };
    ctx.save();
    ctx.lineCap = 'round';
    ctx.setLineDash(clamped ? [s / 120, s / 160] : []);
    ctx.strokeStyle = C.hudShadow; ctx.lineWidth = s / 160; shape(); ctx.stroke();
    ctx.strokeStyle = C.hud; ctx.lineWidth = s / 300; shape(); ctx.stroke();
    ctx.setLineDash([]);
    if (clamped) {
      // Arrow pointing on toward the travel direction + angle between view and travel.
      const dx = (x - c) / R, dy = (y - c) / R, ax = x + dx * r * 2.6, ay = y + dy * r * 2.6, a = r * 0.8;
      ctx.fillStyle = C.hud; ctx.shadowColor = C.hudShadow; ctx.shadowBlur = s / 160;
      ctx.beginPath();
      ctx.moveTo(ax + dx * a, ay + dy * a);
      ctx.lineTo(ax - dx * a - dy * a, ay - dy * a + dx * a);
      ctx.lineTo(ax - dx * a + dy * a, ay - dy * a - dx * a);
      ctx.closePath(); ctx.fill();
      ctx.font = `600 ${Math.round(s * 0.026)}px ${HUD_FONT}`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      const off = Math.round(Math.acos(Math.max(-1, Math.min(1, vz))) * DEG);
      ctx.fillText(`${off}°`, x - dx * r * 2.8, y - dy * r * 2.8);
    }
    ctx.restore();
  }

  // Pilot-view orbit line: track segments split into short pieces, each clipped against a
  // near plane in front of the camera (the orbit runs through the camera itself) and
  // dropped where the Earth blocks the line of sight. Only what is really visible is drawn.
  const PILOT_NEAR_KM = 5, PILOT_SUBDIV = 6;
  function drawPilotTrack() {
    const tr = state.track, cam = state.cam, s = state.size, c = s / 2;
    if (tr.length < 2) return;
    const k = c / Math.tan(PILOT_HALF_FOV), RE2 = EARTH_R_KM ** 2, cc = vdot(cam.c, cam.c);
    const toCam = (P) => {
      const rel = [P[0] - cam.c[0], P[1] - cam.c[1], P[2] - cam.c[2]];
      return { x: vdot(rel, cam.rt), y: vdot(rel, cam.u), z: vdot(rel, cam.f), rel };
    };
    const visible = (q) => {   // line of sight from the camera reaches q before the Earth?
      const dist = Math.hypot(...q.rel), b = vdot(cam.c, q.rel) / dist, disc = b * b - (cc - RE2);
      const t1 = disc >= 0 ? -b - Math.sqrt(disc) : -1;
      return !(t1 > 0 && t1 < dist - 1e-3);
    };
    const scr = (q) => [c + k * q.x / q.z, c - k * q.y / q.z];
    // Point on the arc between two orbit positions (radius interpolated, direction slerp-ish).
    const along = (a, b, t) => {
      const p = a.map((x, i) => x + (b[i] - x) * t), n = Math.hypot(...p);
      const rr = Math.hypot(...a) * (1 - t) + Math.hypot(...b) * t;
      return p.map((x) => x / n * rr);
    };
    ctx.save();
    ctx.lineCap = 'round';
    for (let i = 1; i < tr.length; i++) {
      const A = tr[i - 1], B = tr[i];
      for (let j = 0; j < PILOT_SUBDIV; j++) {
        let p = toCam(along(A.r, B.r, j / PILOT_SUBDIV)), q = toCam(along(A.r, B.r, (j + 1) / PILOT_SUBDIV));
        if (p.z < PILOT_NEAR_KM && q.z < PILOT_NEAR_KM) continue;
        if (!visible(p) || !visible(q)) continue;
        if (p.z < PILOT_NEAR_KM || q.z < PILOT_NEAR_KM) {   // clip to the near plane
          const t = (PILOT_NEAR_KM - p.z) / (q.z - p.z), m = { x: p.x + (q.x - p.x) * t, y: p.y + (q.y - p.y) * t, z: PILOT_NEAR_KM };
          if (p.z < PILOT_NEAR_KM) p = m; else q = m;
        }
        const t = A.t + (B.t - A.t) * (j + 0.5) / PILOT_SUBDIV;
        ctx.globalAlpha = trackAlpha(t);
        ctx.strokeStyle = C.track;
        ctx.lineWidth = s / 200;
        const [x1, y1] = scr(p), [x2, y2] = scr(q);
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
      }
    }
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
    const up = cam.up, hf = cam.hf;                                  // local vertical, look heading
    const north = vnorm([-up[2] * up[0], -up[2] * up[1], 1 - up[2] * up[2]]);   // ECI z on the horizontal plane
    const east = vcross(north, up);
    const rightH = vcross(hf, up);                                   // horizontal, to the right
    const hdgOf = (d) => ((Math.atan2(vdot(d, east), vdot(d, north)) * DEG) + 360) % 360;
    const heading = hdgOf(hf);
    const v = state.now.v, vel = vnorm([v.x, v.y, v.z]);
    const track = hdgOf(vnorm(vel.map((x, i) => x - vdot(vel, up) * up[i])));   // direction of travel

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
      const d = hf.map((x, i) => x * Math.cos(pr) + up[i] * Math.sin(pr));
      const pt = projectDir(d);
      if (!pt) continue;
      // Bar orientation = the local horizontal (rightH) as seen at that point.
      const q = projectDir(vnorm(d.map((x, i) => x + rightH[i] * 0.01)));
      const ang = q ? Math.atan2(q.y - pt.y, q.x - pt.x) : 0;
      const half = p === 0 ? s * 0.2 : s * 0.09, gap = s * 0.045, tick = s * 0.018 * (p > 0 ? 1 : -1);
      ctx.save();
      ctx.translate(pt.x, pt.y); ctx.rotate(ang);
      ctx.setLineDash(p < 0 ? [s / 70, s / 110] : []);
      ctx.beginPath();
      ctx.moveTo(-half, p ? tick : 0); ctx.lineTo(-half, 0); ctx.lineTo(-gap, 0);
      ctx.moveTo(gap, 0); ctx.lineTo(half, 0); ctx.lineTo(half, p ? tick : 0);
      ctx.stroke();
      ctx.setLineDash([]);
      if (p) {
        ctx.textAlign = 'right'; ctx.fillText(String(Math.abs(p)), -half - s * 0.01, 0);
        ctx.textAlign = 'left'; ctx.fillText(String(Math.abs(p)), half + s * 0.01, 0);
      }
      ctx.restore();
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
    // Track bug: direction of travel on the tape (hollow diamond), or an arrow at the tape
    // end pointing toward it when it is off the tape.
    const toTrack = ((track - heading + 540) % 360) - 180, by = ty - s * 0.05, bs = s * 0.011;
    if (Math.abs(toTrack) <= span) {
      const bx = c + toTrack * pxDeg;
      ctx.beginPath();
      ctx.moveTo(bx, by - bs); ctx.lineTo(bx + bs, by); ctx.lineTo(bx, by + bs); ctx.lineTo(bx - bs, by); ctx.closePath();
      ctx.stroke();
    } else {
      const side = Math.sign(toTrack), ax = c + side * (span * pxDeg + s * 0.02);
      ctx.beginPath();
      ctx.moveTo(ax - side * bs, by - bs); ctx.lineTo(ax + side * bs, by); ctx.lineTo(ax - side * bs, by + bs); ctx.closePath();
      ctx.fill();
    }
    ctx.beginPath();   // caret + current heading box
    ctx.moveTo(c - s * 0.012, ty + s * 0.022); ctx.lineTo(c, ty + s * 0.006); ctx.lineTo(c + s * 0.012, ty + s * 0.022);
    ctx.stroke();
    hudBox(String(Math.round(heading) % 360).padStart(3, '0'), c, ty + s * 0.048, 'center', fs);

    // Speed (left) and altitude (right) boxes.
    const g = geoOf(state.now.r, state.now.gmst);
    const spd = Math.hypot(v.x, v.y, v.z);
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
      // Behind the Earth: not drawn here; drawBehindEarth() prints it grey on the surface.
      if (a.hidden || b.hidden || !a.inView || !b.inView) continue;
      ctx.globalAlpha = trackAlpha((a.t + b.t) / 2);
      ctx.strokeStyle = C.track;
      ctx.lineWidth = s / 200;
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
        ctx.globalAlpha = BEHIND_ALPHA * trackAlpha((prev.t + cur.t) / 2);
        ctx.lineWidth = s / 260;
        ctx.beginPath(); ctx.moveTo(prev.x, prev.y); ctx.lineTo(cur.x, cur.y); ctx.stroke();
      }
      prev = cur;
    }

    // Hidden ticks in grey, on the grey line (same frame as the line, see markPoints()).
    ctx.lineCap = 'butt';
    for (const m of state.marks) {
      const [a, b] = markPoints(m);
      if (!a.hidden) continue;
      drawTick(a, b, m.major, C.tickBehind, BEHIND_ALPHA * trackAlpha(m.t));
    }
    ctx.lineCap = 'round';

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

  // Both views: the object's horizon on the ground — every point that sees it at 0°
  // elevation lies on a small circle around the sub-satellite point, radius acos(R / (R + h)).
  // Elevation rings around the sub-satellite point, like the dashed range rings of an
  // aircraft nav display: from inside each ring the object stands at least ε above the
  // horizon. Ground radius for elevation ε: acos(R·cos ε / (R + h)) − ε. Outermost = 0°
  // (the object's horizon, solid, with a compass rose); the inner ones dashed. Printed on the surface, under
  // the orbit line.
  const ELEV_RINGS = [0, 10, 20];
  const ELEV_LABEL_AZ = 270;   // ring labels on the W arm (the orbit usually runs N–S)
  const ROSE_LABELS = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };

  // Ring of ground radius `radiusDeg` around (lon, lat) as a screen polyline, broken where it
  // is hidden or inside one of the `gaps` boxes ({x, y, hw, hh}) kept free for labels.
  function strokeRing(lon, lat, radiusDeg, gaps) {
    let pen = false;
    ctx.beginPath();
    for (let az = 0; az <= 360; az += 2) {
      const [lo, la] = destination(lon, lat, az, radiusDeg);
      const q = project3d(lo, la, 0);
      const ok = q.front && q.inView && !gaps.some((b) => Math.abs(q.x - b.x) < b.hw && Math.abs(q.y - b.y) < b.hh);
      if (ok && pen) ctx.lineTo(q.x, q.y); else if (ok) ctx.moveTo(q.x, q.y);
      pen = ok;
    }
    ctx.stroke();
  }

  // Straight ground line from the centre out along azimuth `az`, broken at the label gaps.
  function strokeArm(lon, lat, az, lenDeg, gaps) {
    let pen = false;
    ctx.beginPath();
    for (let i = 0; i <= 60; i++) {
      const [lo, la] = destination(lon, lat, az, lenDeg * i / 60);
      const q = project3d(lo, la, 0);
      const ok = q.front && q.inView && !gaps.some((b) => Math.abs(q.x - b.x) < b.hw && Math.abs(q.y - b.y) < b.hh);
      if (ok && pen) ctx.lineTo(q.x, q.y); else if (ok) ctx.moveTo(q.x, q.y);
      pen = ok;
    }
    ctx.stroke();
  }

  function drawFootprint() {
    if (!state.now) return;
    const g = geoOf(state.now.r, state.now.gmst);
    const s = state.size, k = EARTH_R_KM / (EARTH_R_KM + g.alt);
    const fs = Math.round(s * 0.021), rfs = Math.round(s * 0.018), apex = project3d(g.lon, g.lat, g.alt);
    const ringDeg = (e) => (Math.acos(k * Math.cos(e / DEG)) - e / DEG) * DEG;
    const horizonDeg = ringDeg(0);
    ctx.save();
    ctx.strokeStyle = C.footprint;
    ctx.font = `600 ${rfs}px ${HUD_FONT}`;
    ctx.textBaseline = 'middle'; ctx.textAlign = 'center';

    // Elevation labels first, so the rings can leave gaps for them.
    const labels = [];
    for (const e of ELEV_RINGS) {
      const [lo, la] = destination(g.lon, g.lat, ELEV_LABEL_AZ, ringDeg(e));
      const q = project3d(lo, la, 0);
      if (!q.front || !q.inView) continue;
      if (!apex.hidden && Math.hypot(apex.x - q.x, apex.y - q.y) < s * 0.03) continue;   // under the object
      const hw = ctx.measureText(`${e}°`).width / 2 + s * 0.003, hh = rfs * 0.55;
      if (labels.some((b) => Math.abs(b.x - q.x) < b.hw + hw && Math.abs(b.y - q.y) < b.hh + hh)) continue;
      labels.push({ text: `${e}°`, x: q.x, y: q.y, hw, hh });
    }

    // Rings, all solid: the horizon a little heavier, the inner range rings fine.
    for (const e of ELEV_RINGS) {
      ctx.lineWidth = e === 0 ? Math.max(1.2, s / 320) : Math.max(0.8, s / 600);
      strokeRing(g.lon, g.lat, ringDeg(e), labels);
    }
    for (const L of labels) hudText(L.text, L.x, L.y, rfs, 600, false, C.footprint);

    // Thin cross inside the horizon ring, linking N–S and E–W through the sub-satellite
    // point; broken where the ring labels sit on the W arm.
    ctx.strokeStyle = C.footprint;
    ctx.lineWidth = Math.max(0.8, s / 600);
    for (const az of [0, 90, 180, 270]) strokeArm(g.lon, g.lat, az, horizonDeg, labels);

    // Compass rose outside the horizon ring (Earth frame, azimuth from the sub-satellite
    // point): ticks every 10°, long every 30°, and N / E / S / W placed in screen space
    // beyond the tick ends so they never touch the ticks.
    const pxDeg = (isFlat() ? 180 : DEG) / discRadius();
    ctx.lineWidth = Math.max(1, s / 480);
    ctx.beginPath();
    const ends = [];
    for (let az = 0; az < 360; az += 10) {
      const major = az % 30 === 0, len = (major ? s / 50 : s / 95) * pxDeg;
      const [lo1, la1] = destination(g.lon, g.lat, az, horizonDeg);
      const [lo2, la2] = destination(g.lon, g.lat, az, horizonDeg + len);
      const a = project3d(lo1, la1, 0), b = project3d(lo2, la2, 0);
      if (!a.front || !a.inView || !b.front) continue;
      ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
      if (az % 90 === 0) ends.push({ az, a, b });
    }
    ctx.stroke();
    for (const { az, a, b } of ends) {
      const dx = b.x - a.x, dy = b.y - a.y, n = Math.hypot(dx, dy) || 1;
      const txt = ROSE_LABELS[az], lfs = fs;
      ctx.font = `600 ${lfs}px ${HUD_FONT}`;
      const off = Math.max(ctx.measureText(txt).width, lfs) * 0.6 + s * 0.006;
      hudText(txt, b.x + dx / n * off, b.y + dy / n * off, lfs, 600, true, C.footprint);
    }
    ctx.restore();
  }

  // Munich view: short ticks across the drawn track at each 5-min mark.
  // Travel-time ticks, one per minute (longer every 5 min), pinned to the ground point the
  // object passes at that minute — so in every view the object visibly moves past them,
  // and in orbit view they ride along with the turning Earth. Fine 1-px ruler lines,
  // muted, across the ground-relative direction of motion.
  function drawTick(p, q, major, color, alpha) {
    const s = state.size, dx = q.x - p.x, dy = q.y - p.y, n = Math.hypot(dx, dy);
    if (n < 1e-6) return;
    const half = major ? s / 95 : s / 170, nx = -dy / n * half, ny = dx / n * half;
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color;
    ctx.lineWidth = Math.max(1, s / 480) * (major ? 1.25 : 1);
    ctx.beginPath(); ctx.moveTo(p.x - nx, p.y - ny); ctx.lineTo(p.x + nx, p.y + ny); ctx.stroke();
  }

  // A mark's screen position and a point 10 s later (for the tick direction), in the same
  // frame as the drawn orbit line (frameGmst): ground-relative in ground / spin / flat
  // views, inertial in satellite reference — so the ticks always sit on the line.
  function markPoints(m) {
    const g = geoOf(m.a.r, frameGmst(m.a)), h = geoOf(m.b.r, frameGmst(m.b));
    return [project3d(g.lon, g.lat, g.alt), project3d(h.lon, h.lat, h.alt)];
  }

  function drawMarkTicks() {
    ctx.save();
    ctx.lineCap = 'butt';
    for (const m of state.marks) {
      const [p, q] = markPoints(m);
      if (p.hidden || !p.inView) continue;   // hidden ones: grey, from drawBehindEarth()
      drawTick(p, q, m.major, C.track, trackAlpha(m.t));
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
  function hudText(text, x, y, size, weight = 600, outline = true, color = '#ffffff') {
    ctx.font = `${weight} ${size}px ${HUD_FONT}`;
    if (outline) {
      ctx.lineJoin = 'round';
      ctx.lineWidth = Math.max(2, size * 0.16);
      ctx.strokeStyle = 'rgba(0,0,0,.7)';
      ctx.strokeText(text, x, y);
    }
    ctx.fillStyle = color;
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
  // Menu / M-key order. Internal keys stay short; labels are what the user sees.
  const MODES = [
    { mode: 'spin', label: 'Earth spin', icon: 'spin' },
    { mode: 'home', label: 'Ground reference', icon: 'pin' },
    { mode: 'orbit', label: 'Satellite reference', icon: 'orbit' },
    { mode: 'pilot', label: 'Pilot reference', icon: 'pilot' },
    { mode: 'flat', label: 'Flat earth', icon: 'flat' },
  ];

  // Earth-spin icon: globe with its axis and a curved arrow around it.
  ICONS.spin = (x, y, h) => {
    const r = h * 0.36;
    ctx.save();
    ctx.strokeStyle = '#fff'; ctx.fillStyle = '#fff'; ctx.lineWidth = Math.max(1.2, h * 0.08); ctx.lineCap = 'round';
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x, y - r * 1.45); ctx.lineTo(x, y + r * 1.45); ctx.stroke();
    ctx.beginPath(); ctx.ellipse(x, y, r * 1.35, r * 0.42, 0, 0.15 * Math.PI, 0.85 * Math.PI); ctx.stroke();
    const ax = x - r * 1.2, ay = y + r * 0.22, a = h * 0.14;   // arrowhead at the left end, pointing back
    ctx.beginPath(); ctx.moveTo(ax - a, ay - a * 0.2); ctx.lineTo(ax + a * 0.6, ay - a); ctx.lineTo(ax + a * 0.3, ay + a * 0.6); ctx.closePath(); ctx.fill();
    ctx.restore();
  };

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
    if (!t) return false;
    const wrap = (d) => ((d + 540) % 360) - 180;
    const d = [wrap(t[0] - state.rotate[0]), t[1] - state.rotate[1], wrap(t[2] - (state.rotate[2] || 0))];
    if (d.every((v) => Math.abs(v) < 0.005)) { state.rotate = t; return false; }
    const lock = d.every((v) => Math.abs(v) < 0.5);
    const k = lock ? 1 : 1 - Math.exp(-dt / GLIDE_TAU_S);   // glide in, then lock on
    state.rotate = [wrap(state.rotate[0] + d[0] * k), state.rotate[1] + d[1] * k, wrap((state.rotate[2] || 0) + d[2] * k)];
    state.dirty = true;
    return !lock;   // true while gliding (the view is moving)
  }

  function setMode(mode) {
    const wasKind = viewKind(state.mode);
    if (mode === 'spin' && state.mode !== 'spin') state.spinT0 = performance.now();
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
    // CSS scales the canvas to fit the window: client px → canvas px.
    const scale = canvas.width / (canvas.getBoundingClientRect().width || canvas.width);
    drag = { x: e.clientX, y: e.clientY, x0: e.clientX, y0: e.clientY, moved: false, scale };
    state.vel = [0, 0];
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag || state.menuOpen) return;
    if (!drag.moved && Math.hypot(e.clientX - drag.x0, e.clientY - drag.y0) < TAP_SLOP_PX) return;
    drag.moved = true;
    canvas.classList.add('dragging');
    const k = DEG / discRadius() * drag.scale;   // one disc radius ≈ 57°
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

  // Quaternion helpers for d3 rotations [λ, φ, γ] (as in d3 "versor dragging").
  function versor([l, p, g = 0]) {
    l /= 2 * DEG; p /= 2 * DEG; g /= 2 * DEG;
    const sl = Math.sin(l), cl = Math.cos(l), sp = Math.sin(p), cp = Math.cos(p), sg = Math.sin(g), cg = Math.cos(g);
    return [cl * cp * cg + sl * sp * sg, sl * cp * cg - cl * sp * sg, cl * sp * cg + sl * cp * sg, cl * cp * sg - sl * sp * cg];
  }
  function versorMultiply(a, b) {
    return [
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ];
  }
  function versorRotation(q) {
    return [
      Math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * DEG,
      Math.asin(Math.max(-1, Math.min(1, 2 * (q[0] * q[2] - q[3] * q[1])))) * DEG,
      Math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * DEG,
    ];
  }

  // dx / dy: screen drag converted to degrees (x right, y down).
  // Globe views: trackball — the globe turns about the screen's own axes (left-multiplied
  // quaternion), so it follows the finger at any orientation, pole or view roll.
  // Flat earth: spin around the pole. Pilot: yaw / pitch of the look direction.
  function rotateBy(dx, dy) {
    if (isFlat()) {
      state.rotate[0] = ((state.rotate[0] + dx + 540) % 360) - 180;
    } else if (isPilot()) {
      state.rotate[0] = ((state.rotate[0] + dx + 540) % 360) - 180;
      state.rotate[1] = Math.max(-90, Math.min(90, state.rotate[1] - dy));
    } else {
      state.rotate = versorRotation(versorMultiply(versor([dx, -dy, 0]), versor(state.rotate)));
    }
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
    // Scheduled first so an exception in one frame cannot stop the loop for good.
    requestAnimationFrame(frame);
    if (ts - lastTrack > 1000) { lastTrack = ts; updateTrack(); }
    if (ts - lastNow > 100) { lastNow = ts; updateNow(); state.dirty = true; }
    if (state.mode === 'spin') state.dirty = true;   // continuous rotation
    if (!drag && (Math.abs(state.vel[0]) > 0.01 || Math.abs(state.vel[1]) > 0.01)) {
      rotateBy(state.vel[0], state.vel[1]);
      state.vel = state.vel.map((v) => v * 0.92);
    }
    const inertia = !drag && (Math.abs(state.vel[0]) > 0.01 || Math.abs(state.vel[1]) > 0.01);
    const glide = !drag && ts > state.manualUntil && easeToTarget();
    const moving = !!(drag && drag.moved) || inertia || glide || state.mode === 'spin';
    if (!moving && frameScale !== 1) state.dirty = true;   // sharpen the last reduced frame
    if (state.dirty) {
      state.dirty = false;
      frameMoving = moving;
      frameScale = pixelScale(moving);
      const t0 = performance.now();
      draw();
      perf.total = performance.now() - t0;
      if (fps) fps.drawn(perf.total);
    }
    if (fps) fps.tick(ts);
  }

  // Optional frame-time overlay: open the page with ?fps.
  const fps = /[?&]fps\b/.test(location.search) ? (() => {
    const box = document.createElement('div');
    box.style.cssText = 'position:fixed;left:8px;bottom:8px;z-index:10;padding:4px 6px;border-radius:4px;'
      + 'background:rgba(0,0,0,.7);color:#8f8;font:11px/1.35 ui-monospace,monospace;white-space:pre;pointer-events:none';
    document.body.appendChild(box);
    let t0 = 0, frames = 0, draws = 0, sum = 0, max = 0, gap = 0, last = 0;
    return {
      drawn(ms) { draws++; sum += ms; max = Math.max(max, ms); },
      tick(ts) {
        frames++;
        if (last) gap = Math.max(gap, ts - last);
        last = ts;
        if (ts - t0 < 1000) return;
        box.textContent = `${(frames * 1000 / (ts - t0)).toFixed(0)} fps  longest gap ${gap.toFixed(0)} ms\n`
          + `draw ${draws ? (sum / draws).toFixed(1) : '-'} ms avg, ${max.toFixed(1)} max\n`
          + `pixels ${(perf.pixels || 0).toFixed(1)} ms @ ${Math.round((perf.scale || 1) * 100)}%`;
        t0 = ts; frames = draws = sum = max = gap = 0;
      },
    };
  })() : null;

  layout();
  setMode(state.mode);
  loadMap();
  loadTexture();
  loadFonts();
  loadNightLights();
  loadSatellite(Number(ui.norad.value));
  requestAnimationFrame(frame);
})();
