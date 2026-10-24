/* Service worker for the static (GitHub Pages) build: makes reloads instant and the page
 * usable offline. The deploy workflow fills in the two version strings; the local dev
 * server never registers it.
 *
 *   shell (html/js/css)       precached per build, served from cache
 *   vendor/ and tiles/        cache-first in a cache keyed by their content hash, so a
 *                             deploy that does not touch them keeps them
 *   gp/*.json, CelesTrak      network-first, cached copy when offline
 */
const BUILD = '__BUILD__';
const ASSETS = '__ASSETS__';
const SHELL_CACHE = `shell-${BUILD}`;
const ASSET_CACHE = `assets-${ASSETS}`;
const DATA_CACHE = 'data';

const SHELL = ['./', 'index.html', 'app.js', 'style.css'];
const PRELOAD = [
  'vendor/d3-array.min.js', 'vendor/d3-geo.min.js', 'vendor/topojson-client.min.js',
  'vendor/satellite.min.js', 'vendor/land-50m.json', 'vendor/countries-110m.json',
  'vendor/terrain-2048.jpg', 'vendor/night-2048.jpg',
  'vendor/barlow-condensed-500.woff2', 'vendor/barlow-condensed-600.woff2',
];

self.addEventListener('install', (e) => {
  e.waitUntil((async () => {
    await (await caches.open(SHELL_CACHE)).addAll(SHELL.map((u) => new Request(u, { cache: 'reload' })));
    const assets = await caches.open(ASSET_CACHE);
    await Promise.all(PRELOAD.map(async (u) => { if (!(await assets.match(u))) await assets.add(u); }));
    await self.skipWaiting();
  })());
});

self.addEventListener('activate', (e) => {
  e.waitUntil((async () => {
    const keep = [SHELL_CACHE, ASSET_CACHE, DATA_CACHE];
    for (const k of await caches.keys()) if (!keep.includes(k)) await caches.delete(k);
    await self.clients.claim();
  })());
});

async function cacheFirst(req, name) {
  const cache = await caches.open(name);
  const hit = await cache.match(req, { ignoreSearch: true });
  if (hit) return hit;
  const res = await fetch(req);
  if (res.ok) cache.put(req, res.clone());
  return res;
}

async function networkFirst(req) {
  const cache = await caches.open(DATA_CACHE);
  try {
    const res = await fetch(req);
    if (res.ok) cache.put(req, res.clone());
    return res;
  } catch (err) {
    const hit = await cache.match(req);
    if (hit) return hit;
    throw err;
  }
}

self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);
  if (url.hostname === 'celestrak.org') { e.respondWith(networkFirst(req)); return; }
  if (url.origin !== location.origin) return;
  const path = url.pathname.slice(new URL(self.registration.scope).pathname.length);
  if (path.startsWith('gp/')) e.respondWith(networkFirst(req));
  else if (path.startsWith('vendor/') || path.startsWith('tiles/')) e.respondWith(cacheFirst(req, ASSET_CACHE));
  else if (req.mode === 'navigate') e.respondWith(caches.match('index.html').then((r) => r || fetch(req)));
  else e.respondWith(caches.match(req, { ignoreSearch: true }).then((r) => r || fetch(req)));
});
