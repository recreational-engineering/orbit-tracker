



#include <math.h>
#include <string.h>
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "globe.h"
#include "tiles.h"

static const char *TAG = "globe";

#define S GFX_W
static const float R = GLOBE_R, CX = GLOBE_CX, CY = GLOBE_CY;
#define EARTH_R_KM 6371.0f
static const rgb_t PILOT_BG = {4, 6, 14};

// ---------- textures ----------
extern const uint8_t texture_bin_start[] asm("_binary_texture_bin_start");
extern const uint8_t texture_half_bin_start[] asm("_binary_texture_half_bin_start");
extern const uint8_t lights_bin_start[] asm("_binary_lights_bin_start");
#define TEX_W 2048
#define TEX_H 1024
// RGB565 in 8x8 texel tiles (128 bytes, one L2 cache line): cache friendly in any direction.
// Level 0 at 2048x1024 (full resolution), level 1 at 1024x512 (half resolution).
#define TEX_INDEX(u, v, tiles_x) (((((v) >> 3) * (tiles_x) + ((u) >> 3)) << 6) | (((v) & 7) << 3) | ((u) & 7))
static uint16_t *texture[2];
// City-light intensity, one byte per texel, mips 2048 / 1024 / 512 / 256 wide.
static uint8_t *lights[4];
// Pixel footprint over the level-0 lights texel: the globe at the disc centre (RE / r km per
// pixel) and the flat map (π RE / r).
#define LIGHT_TEXEL_KM (2 * (float)M_PI * EARTH_R_KM / 2048)

// ---------- static layers ----------
static uint16_t *face;   // instrument face, cast and contact shadow
// Globe disc: spans per row (any coverage), inner spans (full coverage), per-pixel tables
// indexed row_off[y] + x - span_x0[y]: lighting as (mult | add << 8), [0] sphere lighting,
// [1] flat map vignette and glint; coverage of the antialiased disc edge.
static int16_t span_x0[S], span_x1[S], in_x0[S], in_x1[S];
static int32_t row_off[S];
static uint16_t *light_tab[2];
static uint8_t *cover;
#define SG_C 8
#define SG_N (S / SG_C)
static uint16_t *shade_grid[2];   // in PSRAM: 32 KB, read only for strokes on the globe
// Earth spin: u (turn, 16 bit) | sqrt(1 - t²)·255 << 16 for t = Xs / cos(lat) in [-1, 1].
#define SPIN_N 4096
// One guard entry at each end (rounding at the row ends): spin_lut[-1 .. SPIN_N].
static uint32_t spin_lut_g[SPIN_N + 2];
#define spin_lut (spin_lut_g + 1)
// Per disc pixel tables for the polar-axis paths: [0] equatorial globe: u0 | Z·255 << 16,
// [1] flat map: u0 | texture row << 16 (u0: longitude at λ = 0 as a fraction of a turn);
// flat_nrm: the flat map's surface normal at λ = 0 as int8 A | B << 8 | ez << 16.
static uint32_t *axis_tab[2], *flat_nrm;
// Lights level of detail by Z·255 (globe: footprint (RE / r) / max(Z, .15)).
static uint8_t lod_z[256];
// Night side by sun elevation (Q12): darkening (·256) and light gain (·256), smoothstep(.1, -.12).
#define NIGHT_LO (-492)
#define NIGHT_HI 410
#define NIGHT_N (NIGHT_HI - NIGHT_LO + 1)
static uint8_t night_a[NIGHT_N];
static uint16_t night_n[NIGHT_N];
static int16_t eq_vrow[S];   // equatorial globe: texture row (level 0) of each screen row
// Half resolution: 2x2 blocks, block spans per block row, lighting per block, inner flags.
#define HS (S / 2)
static int16_t hspan_x0[HS], hspan_x1[HS];
static int32_t hrow_off[HS];
static uint16_t *hlight_tab[2];
static uint8_t *inner;
// Pilot view: the whole round screen; its rim vignette as a lookup on (r / c)².
#define RIM_LUT_N 1024
static uint8_t pilot_rim[RIM_LUT_N + 1];
static int16_t pspan_x0[S], pspan_x1[S];

typedef struct { float t, r, g, b, a; } stop_t;

static void grad_eval(const stop_t *s, int n, float t, float out[4])
{
    if (t <= s[0].t) { out[0] = s[0].r; out[1] = s[0].g; out[2] = s[0].b; out[3] = s[0].a; return; }
    for (int i = 1; i < n; i++) {
        if (t <= s[i].t) {
            float k = (t - s[i - 1].t) / (s[i].t - s[i - 1].t);
            out[0] = s[i - 1].r + (s[i].r - s[i - 1].r) * k;
            out[1] = s[i - 1].g + (s[i].g - s[i - 1].g) * k;
            out[2] = s[i - 1].b + (s[i].b - s[i - 1].b) * k;
            out[3] = s[i - 1].a + (s[i].a - s[i - 1].a) * k;
            return;
        }
    }
    out[0] = s[n - 1].r; out[1] = s[n - 1].g; out[2] = s[n - 1].b; out[3] = s[n - 1].a;
}

// Canvas radial gradient with concentric circles: t = (d - r0) / (r1 - r0).
static void radial(const stop_t *s, int n, float px, float py, float cx, float cy, float r0, float r1, float out[4])
{
    grad_eval(s, n, (hypotf(px - cx, py - cy) - r0) / (r1 - r0), out);
}

static inline void over(float c[3], const float l[4], float cov)
{
    float a = l[3] * cov;
    for (int k = 0; k < 3; k++) c[k] += (l[k] - c[k]) * a;
}

static const uint8_t BAYER4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

static uint16_t dither565(const float c[3], int x, int y)
{
    float d = (BAYER4[y & 3][x & 3] + 0.5f) / 16.0f;
    int r = (int)(c[0] / 255.0f * 31 + d), g = (int)(c[1] / 255.0f * 63 + d), b = (int)(c[2] / 255.0f * 31 + d);
    r = r > 31 ? 31 : r; g = g > 63 ? 63 : g; b = b > 31 ? 31 : b;
    return (uint16_t)(r << 11 | g << 5 | b);
}

// sphereLayers(): Lambert diffuse from the lamp, darkening toward the limb, glossy highlight.
// Returns shade (multiplied, 0..255) | specular (added, 0..255) << 8.
static uint16_t sphere_light(float px, float py)
{
    const float AMB = 0.16f, DIF = 1.25f, RIM = 0.42f, SPEC = 0.42f, SHEEN = 0.05f;
    const float n = sqrtf(0.25f + 0.36f + 0.62f * 0.62f), lx = -0.5f / n, ly = 0.6f / n, lz = 0.62f / n;
    const float hn = sqrtf(lx * lx + ly * ly + (lz + 1) * (lz + 1)), hx = lx / hn, hy = ly / hn, hz = (lz + 1) / hn;
    float X = (px - CX) / R, Y = (CY - py) / R, d = X * X + Y * Y;
    if (d > 1) d = 1;
    float Z = sqrtf(1 - d);
    float ndl = fmaxf(0, X * lx + Y * ly + Z * lz);
    float k = (AMB + DIF * ndl) * (1 - RIM + RIM * sqrtf(Z));
    float ndh = fmaxf(0, X * hx + Y * hy + Z * hz);
    float sp = SPEC * powf(ndh, 70) + SHEEN * powf(ndh, 6);
    int shade = (int)(255 * fminf(1, k) + 0.5f), spec = (int)(255 * fminf(1, sp) + 0.5f);
    return (uint16_t)(shade | spec << 8);
}

// drawFlatShading(): rim vignette and the glass glint, as out = in * mult + add.
static uint16_t flat_light(float px, float py)
{
    static const stop_t rim[] = {{0, 0, 0, 0, 0}, {1, 0, 0, 0, .35f}};
    static const stop_t glint[] = {{0, 255, 255, 255, .22f}, {1, 255, 255, 255, 0}};
    float mult = 1, add = 0, l[4];
    radial(rim, 2, px, py, CX, CY, R * 0.8f, R, l);
    mult *= 1 - l[3];
    add *= 1 - l[3];
    const float gx = CX - R * 0.38f, gy = CY - R * 0.48f, gr = R * 0.32f, gc = cosf(-0.6f), gs = sinf(-0.6f);
    float qx = px - gx, qy = py - gy;
    float ux = qx * gc + qy * gs, uy = (-qx * gs + qy * gc) / 0.55f;
    float gd = sqrtf(ux * ux + uy * uy);
    if (gd < gr) {
        grad_eval(glint, 2, gd / gr, l);
        mult *= 1 - l[3];
        add = add * (1 - l[3]) + 255 * l[3];
    }
    return (uint16_t)((int)(mult * 255 + 0.5f) | (int)(add + 0.5f) << 8);
}

// Lighting as stored per pixel: out = in · (m + 1) / 32 + add, with m in bits 0-4 and the
// grey add as RGB565 components (5 bits in 5-9, 6 bits in 10-15). The multiplier and add are
// dithered here, and the product is dithered again when it is truncated to RGB565 (sw_light):
// truncating without it biased the result in rings around the highlight.
static uint16_t pack_light(uint16_t l, int x, int y)
{
    float d = (BAYER4[y & 3][x & 3] + 0.5f) / 16.0f;
    float mult = (l & 255) / 255.0f, add = (l >> 8) / 255.0f;
    int m = (int)(mult * 32 - 1 + d);
    m = m < 0 ? 0 : m > 31 ? 31 : m;
    int s5 = (int)(add * 31 + d), s6 = (int)(add * 63 + d);
    s5 = s5 > 31 ? 31 : s5;
    s6 = s6 > 63 ? 63 : s6;
    return (uint16_t)(m | s5 << 5 | s6 << 10);
}

static uint8_t coverage(float px, float py, float rad)
{
    float cov = rad + 0.5f - hypotf(px - CX, py - CY);
    return (uint8_t)(cov >= 1 ? 255 : cov <= 0 ? 0 : cov * 255 + 0.5f);
}

static void build_static_layers(void)
{
    // Instrument face; a soft cast shadow away from the lamp and a tight contact shadow.
    static const stop_t face_g[] = {{0, 0x2a, 0x2e, 0x2c, 1}, {1, 0x1a, 0x1d, 0x1c, 1}};
    static const stop_t cast[] = {{0, 0, 0, 0, .7f}, {1, 0, 0, 0, 0}};
    static const stop_t contact[] = {{0, 0, 0, 0, .85f}, {1, 0, 0, 0, 0}};
    const float n = sqrtf(0.25f + 0.36f + 0.62f * 0.62f);
    const float shx = CX + 0.5f / n * R * 0.11f, shy = CY + 0.6f / n * R * 0.11f;
    for (int y = 0; y < S; y++)
        for (int x = 0; x < S; x++) {
            float px = x + 0.5f, py = y + 0.5f, c[4], l[4];
            radial(face_g, 2, px, py, CX, CY, R * 0.9f, S * 0.5f, c);
            float cov = R * 1.16f + 0.5f - hypotf(px - shx, py - shy);
            if (cov > 0) { radial(cast, 2, px, py, shx, shy, R * 0.85f, R * 1.16f, l); over(c, l, fminf(1, cov)); }
            cov = R * 1.045f + 0.5f - hypotf(px - CX, py - CY);
            if (cov > 0) { radial(contact, 2, px, py, CX, CY, R * 0.97f, R * 1.045f, l); over(c, l, fminf(1, cov)); }
            face[y * S + x] = dither565(c, x, y);
        }

    int32_t n_px = 0;
    for (int y = 0; y < S; y++) {
        float dy = y + 0.5f - CY, half2 = (R + 0.5f) * (R + 0.5f) - dy * dy;
        row_off[y] = n_px;
        span_x0[y] = span_x1[y] = in_x0[y] = in_x1[y] = 0;
        if (half2 > 0) {
            float half = sqrtf(half2);
            int x0 = (int)floorf(CX - half), x1 = (int)ceilf(CX + half);
            span_x0[y] = x0 < 0 ? 0 : x0;
            span_x1[y] = x1 > S ? S : x1;
            n_px += span_x1[y] - span_x0[y];
        }
        // Pilot view: the whole round screen.
        float ph2 = (S / 2.0f + 0.5f) * (S / 2.0f + 0.5f) - dy * dy;
        float ph = ph2 > 0 ? sqrtf(ph2) : 0;
        pspan_x0[y] = (int16_t)fmaxf(0, floorf(CX - ph));
        pspan_x1[y] = (int16_t)fminf(S, ceilf(CX + ph));
    }
    for (int i = 0; i < SPIN_N; i++) {
        float t = i * 2.0f / (SPIN_N - 1) - 1;
        uint32_t u = (uint32_t)(int)floorf((asinf(t) / (2 * (float)M_PI) + 0.5f) * 65536) & 0xFFFF;
        spin_lut[i] = u | (uint32_t)(sqrtf(fmaxf(0, 1 - t * t)) * 255 + 0.5f) << 16;
    }
    spin_lut[-1] = spin_lut[0];
    spin_lut[SPIN_N] = spin_lut[SPIN_N - 1];
    for (int k = 0; k < 2; k++) {
        light_tab[k] = heap_caps_malloc(n_px * 2, MALLOC_CAP_SPIRAM);
        axis_tab[k] = heap_caps_malloc(n_px * 4, MALLOC_CAP_SPIRAM);
    }
    flat_nrm = heap_caps_malloc(n_px * 4, MALLOC_CAP_SPIRAM);
    cover = heap_caps_malloc(n_px, MALLOC_CAP_SPIRAM);
    for (int y = 0; y < S; y++) {
        float Ys = (CY - y - 0.5f) / R;
        eq_vrow[y] = (int16_t)fminf(TEX_H - 1, fmaxf(0, (0.5f - asinf(fmaxf(-1, fminf(1, Ys))) / (float)M_PI) * TEX_H));
        in_x0[y] = span_x1[y];
        in_x1[y] = span_x1[y];
        for (int x = span_x0[y]; x < span_x1[y]; x++) {
            int32_t i = row_off[y] + x - span_x0[y];
            float px = x + 0.5f, py = y + 0.5f, Xs = (px - CX) / R;
            light_tab[0][i] = pack_light(sphere_light(px, py), x, y);
            light_tab[1][i] = pack_light(flat_light(px, py), x, y);
            cover[i] = coverage(px, py, R);
            if (cover[i] == 255) {
                if (in_x0[y] == span_x1[y]) in_x0[y] = x;
                in_x1[y] = x + 1;
            }
            float d = fminf(1, Xs * Xs + Ys * Ys), Z = sqrtf(1 - d), rho = sqrtf(d);
            uint32_t u_eq = (uint32_t)(int)floorf((atan2f(Xs, Z) / (2 * (float)M_PI) + 0.5f) * 65536) & 0xFFFF;
            uint32_t u_fl = (uint32_t)(int)floorf((atan2f(Xs, -Ys) / (2 * (float)M_PI) + 0.5f) * 65536) & 0xFFFF;
            axis_tab[0][i] = u_eq | (uint32_t)(Z * 255 + 0.5f) << 16;
            axis_tab[1][i] = u_fl | (uint32_t)fminf(TEX_H - 1, rho * TEX_H) << 16;
            // flat normal at λ = 0: (sin ρπ cos θ, sin ρπ sin θ, cos ρπ), (sin θ, cos θ) = (Xs, -Ys) / ρ
            float sp = sinf(rho * (float)M_PI), cp = cosf(rho * (float)M_PI);
            float ct = rho > 1e-6f ? -Ys / rho : 1, st = rho > 1e-6f ? Xs / rho : 0;
            int A = (int)lroundf(sp * ct * 127), B = (int)lroundf(sp * st * 127), E = (int)lroundf(cp * 127);
            flat_nrm[i] = (uint32_t)(A & 255) | (uint32_t)(B & 255) << 8 | (uint32_t)(E & 255) << 16;
        }
        if (in_x0[y] >= in_x1[y]) in_x0[y] = in_x1[y] = span_x1[y];
    }

    int32_t hn = 0;
    for (int by = 0; by < HS; by++) {
        int a = 2 * by, b = a + 1, lo = S, hi = 0;
        if (span_x1[a] > span_x0[a]) { lo = span_x0[a]; hi = span_x1[a]; }
        if (span_x1[b] > span_x0[b]) { if (span_x0[b] < lo) lo = span_x0[b]; if (span_x1[b] > hi) hi = span_x1[b]; }
        hrow_off[by] = hn;
        if (hi <= lo) { hspan_x0[by] = hspan_x1[by] = 0; continue; }
        hspan_x0[by] = lo / 2;
        hspan_x1[by] = (hi + 1) / 2;
        hn += hspan_x1[by] - hspan_x0[by];
    }
    for (int k = 0; k < 2; k++) hlight_tab[k] = heap_caps_malloc(hn * 2, MALLOC_CAP_SPIRAM);
    inner = heap_caps_malloc(hn, MALLOC_CAP_SPIRAM);
    for (int by = 0; by < HS; by++)
        for (int bx = hspan_x0[by]; bx < hspan_x1[by]; bx++) {
            int32_t j = hrow_off[by] + bx - hspan_x0[by];
            float px = 2 * bx + 1.0f, py = 2 * by + 1.0f;   // block centre
            hlight_tab[0][j] = pack_light(sphere_light(px, py), bx, by);
            hlight_tab[1][j] = pack_light(flat_light(px, py), bx, by);
            bool in = true;
            for (int q = 0; q < 4; q++) in &= coverage(2 * bx + (q & 1) + 0.5f, 2 * by + (q >> 1) + 0.5f, R) == 255;
            inner[j] = in;
        }
    for (int z = 0; z < 256; z++) {
        float ratio = (EARTH_R_KM / R) / fmaxf(z / 255.0f, 0.15f) / LIGHT_TEXEL_KM;
        lod_z[z] = ratio <= 1 ? 0 : ratio <= 2 ? 1 : ratio <= 4 ? 2 : 3;
    }
    for (int k = 0; k < NIGHT_N; k++) {
        float sun = (k + NIGHT_LO) / 4096.0f, t = (sun - 0.1f) / (-0.22f);
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        float n = t * t * (3 - 2 * t);
        night_a[k] = (uint8_t)(0.78f * n * 256);
        night_n[k] = (uint16_t)(n * 256);
    }
    // Pilot rim vignette: radial gradient from 0.75 c (transparent) to c (.45 black).
    for (int i = 0; i <= RIM_LUT_N; i++) {
        float r = sqrtf((float)i / RIM_LUT_N), t = (r - 0.75f) / 0.25f;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        pilot_rim[i] = (uint8_t)(255 * (1 - 0.45f * t) + 0.5f);
    }
    ESP_LOGI(TAG, "disc pixels %ld, blocks %ld", (long)n_px, (long)hn);
}

// ---------- shared per-pixel helpers ----------
// The P4's FPU divides and takes square roots slowly (~40 cycles, not pipelined); the
// per-pixel paths use Newton-refined reciprocal square roots (multiplies only).
static inline float rsqrt_fast(float x)
{
    union { float f; uint32_t i; } u = {x};
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y *= 1.5f - 0.5f * x * y * y;
    y *= 1.5f - 0.5f * x * y * y;   // relative error ~5e-6
    return y;
}

// atan2 with max error ~1e-5 rad (0.003 texels at 2048 px / 360°).
static inline float fast_atan2f(float y, float x)
{
    float ax = fabsf(x), ay = fabsf(y);
    float mn = fminf(ax, ay), mx = fmaxf(ax, ay) + 1e-30f;
    float rm = rsqrt_fast(mx);
    float a = mn * rm * rm, s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (ay > ax) r = 1.57079637f - r;
    if (x < 0) r = 3.14159274f - r;
    return y < 0 ? -r : r;
}

// Texture row (level 0) of sin(latitude), (0.5 - asin(z) / π) * TEX_H, tabulated; asin is
// steep near the poles, so |z| > ASIN_LUT_MAX uses the exact formula. Polar-distance
// sin(ρπ) / ρ and cos(ρπ) for the flat map, tabulated over ρ in [0, 1].
#define LUT_N 4096
#define ASIN_LUT_MAX 0.98f
static float *asin_lut, *polar_lut;
// exp(-h / 90 km) for the pilot view's limb glow, over 0 .. 2000 km in 1 km steps.
#define GLOW_N 2048
// exp(-h / 90) · 255 < 1 beyond h = 90 ln 255 ≈ 499 km
#define GLOW_FAINT 500
static float *glow_lut;

static void build_luts(void)
{
    asin_lut = heap_caps_malloc((LUT_N + 2) * sizeof(float), MALLOC_CAP_INTERNAL);
    polar_lut = heap_caps_malloc((LUT_N + 2) * 2 * sizeof(float), MALLOC_CAP_INTERNAL);
    glow_lut = heap_caps_malloc(GLOW_N * sizeof(float), MALLOC_CAP_SPIRAM);
    for (int i = 0; i <= LUT_N + 1; i++) {
        double z = fmin(1.0, -1.0 + 2.0 * i / LUT_N);
        asin_lut[i] = (float)((0.5 - asin(z) / M_PI) * TEX_H);
        double rho = fmin(1.0, (double)i / LUT_N);
        polar_lut[2 * i] = (float)(rho > 1e-9 ? sin(rho * M_PI) / rho : M_PI);
        polar_lut[2 * i + 1] = (float)cos(rho * M_PI);
    }
    for (int i = 0; i < GLOW_N; i++) glow_lut[i] = expf(-i / 90.0f);
}

static inline float smoothstep(float e0, float e1, float x)
{
    float t = (x - e0) / (e1 - e0);
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return t * t * (3 - 2 * t);
}

static inline void texel_rgb(int level, int u, int v, int *r, int *g, int *b)
{
    // u, v in level-0 texels
    u >>= level;
    v >>= level;
    gfx_unpack(texture[level][TEX_INDEX(u, v, (TEX_W / 8) >> level)], r, g, b);
}

// Night side: darken toward deep blue with a soft twilight band, add warm city lights.
static inline void night(int *r, int *g, int *b, float sun, int light_level, float fu, float fv)
{
    float n = smoothstep(0.1f, -0.12f, sun);
    int a = (int)(0.78f * n * 256);
    *r += ((4 - *r) * a) >> 8;
    *g += ((8 - *g) * a) >> 8;
    *b += ((28 - *b) * a) >> 8;
    int w = TEX_W >> light_level, h = TEX_H >> light_level;
    int lu = (int)(fu * w), lv = (int)(fv * h);
    if (lu >= w) lu = w - 1;
    if (lv >= h) lv = h - 1;
    if (lv < 0) lv = 0;
    int L = (int)(lights[light_level][lv * w + lu] * n);
    *r += L; *g += (L * 205) >> 8; *b += (L * 115) >> 8;
}

// RGB565 spread over 32 bits (r, b in the low half, g in the high half) with a free bit above
// each component: all three channels in one multiply / add.
#define SW_MASK 0x07E0F81Fu
#define SPREAD(p) ((((uint32_t)(p)) | ((uint32_t)(p) << 16)) & SW_MASK)
#define UNSPREAD(x) ((uint16_t)((x) | ((x) >> 16)))

static inline __attribute__((always_inline)) uint32_t sw_sat_add(uint32_t x, uint32_t y)
{
    uint32_t s = x + y, o = s & 0x08010020u;   // carries out of b, r, g
    return (s | (o - (o >> 5))) & SW_MASK;
}

// Output dither: a second 4x4 Bayer pattern (shifted against the one in the light table),
// 0 .. 31 in each spread field, added before the >> 5 of the light multiply.
static uint32_t DQ[4][4];

static void build_dither(void)
{
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) {
            uint32_t d = BAYER4[(y + 2) & 3][(x + 1) & 3] * 2 + 1;
            DQ[y][x] = d | d << 11 | d << 21;
        }
}

// The same threshold as a plain 0 .. 31 (twilight blend).
static inline uint32_t dq5(uint32_t dq) { return dq & 31; }

static inline __attribute__((always_inline)) uint32_t sw_light(uint32_t x, uint16_t light, uint32_t dq)
{
    x = ((x * ((light & 31) + 1) + dq) >> 5) & SW_MASK;
    uint32_t s5 = (light >> 5) & 31, s6 = light >> 10;
    return light >> 5 ? sw_sat_add(x, s5 | s5 << 11 | s6 << 21) : x;
}

// 8-bit colour lit to RGB565 (exact rim pixels).
static inline uint16_t lit_pack(int r, int g, int b, uint16_t light, uint32_t dq)
{
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    return UNSPREAD(sw_light(SPREAD(gfx_pack(r, g, b)), light, dq));
}

// Night (integer): sun in Q12, L the city-light texel.
static inline void night_q(int *r, int *g, int *b, int sun, int L)
{
    int k = sun - NIGHT_LO;
    if (k < 0) k = 0;
    int a = night_a[k], n = night_n[k];
    *r += ((4 - *r) * a) >> 8;
    *g += ((8 - *g) * a) >> 8;
    *b += ((28 - *b) * a) >> 8;
    L = (L * n) >> 8;
    *r += L; *g += (L * 205) >> 8; *b += (L * 115) >> 8;
    if (*r > 255) *r = 255;
    if (*g > 255) *g = 255;
    if (*b > 255) *b = 255;
}

static inline uint16_t tex0(int u, int v) { return texture[0][TEX_INDEX(u, v, TEX_W / 8)]; }

// Night side on a spread pixel: blend toward RGB(4, 8, 28), then add the warm city lights.
static inline __attribute__((always_inline)) uint32_t sw_night(uint32_t x, int sun, int L, uint32_t d)
{
    int k = sun - NIGHT_LO;
    if (k < 0) k = 0;
    uint32_t a = (night_a[k] + (dq5(d) >> 2)) >> 3;   // /32, dithered: no steps across the twilight band
    const uint32_t deep = SPREAD((4 >> 3) << 11 | (8 >> 2) << 5 | (28 >> 3));
    x = ((x * (32 - a) + deep * a) >> 5) & SW_MASK;
    int l = (L * night_n[k]) >> 8;
    if (l) x = sw_sat_add(x, (uint32_t)(l >> 3) << 11 | (uint32_t)((l * 205) >> 10) << 21 | (uint32_t)((l * 115) >> 11));
    return x;
}

static inline __attribute__((always_inline)) uint16_t lit_px(uint16_t t, int sun, int lights_texel, uint16_t light, uint32_t dq)
{
    uint32_t x = SPREAD(t);
    if (sun < NIGHT_HI) x = sw_night(x, sun, lights_texel, dq);
    return UNSPREAD(sw_light(x, light, dq));
}

// Antialiased disc edge: blend toward the face (or pilot background).
static inline uint16_t edge_mix(uint16_t p, uint16_t under, int cov)
{
    int r, g, b, br, bg, bb;
    gfx_unpack(p, &r, &g, &b);
    gfx_unpack(under, &br, &bg, &bb);
    return gfx_pack(br + (((r - br) * cov) >> 8), bg + (((g - bg) * cov) >> 8), bb + (((b - bb) * cov) >> 8));
}

static inline int globe_light_level(float z)
{
    // ceil(log2(footprint / lights texel)), footprint = (RE / r) / max(Z, .15)
    float ratio = (EARTH_R_KM / R) / fmaxf(z, 0.15f) / LIGHT_TEXEL_KM;
    return ratio <= 1 ? 0 : ratio <= 2 ? 1 : ratio <= 4 ? 2 : 3;
}

// ---------- render context ----------
typedef struct {
    const globe_view_t *v;
    uint16_t *fb;
    int level;                               // texture level
    float lam, cp, sp, cg, sg, cl0, sl0;     // orthographic rotation
    float sx, sy, sz;                        // sun
    uint32_t uoff;                           // polar-axis paths: -λ as a fraction of a turn << 16
    float P, Q;                              // polar-axis paths: sun dot coefficients
    // pilot
    float tanH, pix_ang, cc2, t_horizon, inv_t_horizon, hi_max_t, cgm, sgm;
    float ccx, ccy, ccz;                     // camera position as floats (double converts in software)
    int32_t qt[3], ql[3], hi_max_tq, grat_w; // pilot_shade's fixed-point thresholds and scales
    uint32_t haze_k;
} rctx_t;

static void rctx_init(rctx_t *c, const globe_view_t *v, uint16_t *fb, int level)
{
    const float d = (float)(M_PI / 180);
    c->v = v; c->fb = fb; c->level = level;
    c->lam = v->rot[0] * d;
    c->cp = cosf(v->rot[1] * d); c->sp = sinf(v->rot[1] * d);
    c->cg = cosf(v->rot[2] * d); c->sg = sinf(v->rot[2] * d);
    c->cl0 = cosf(c->lam); c->sl0 = sinf(c->lam);
    c->sx = v->sun[0]; c->sy = v->sun[1]; c->sz = v->sun[2];
    float f = -c->lam / (2 * (float)M_PI);
    f -= floorf(f);
    c->uoff = (uint32_t)(f * 65536) & 0xFFFF;
    if (v->kind == GLOBE_FLAT) {
        c->P = c->cl0 * c->sx - c->sl0 * c->sy;
        c->Q = c->sl0 * c->sx + c->cl0 * c->sy;
    } else {
        c->P = c->cl0 * c->sx - c->sl0 * c->sy;   // × Z
        c->Q = c->sl0 * c->sx + c->cl0 * c->sy;   // × Xs
    }
    if (v->kind == GLOBE_PILOT) {
        const pilot_cam_t *cam = &v->cam;
        c->tanH = tanf((float)(PILOT_HALF_FOV_DEG * M_PI / 180));
        c->pix_ang = 2 * c->tanH / S;
        c->cc2 = (float)(cam->c[0] * cam->c[0] + cam->c[1] * cam->c[1] + cam->c[2] * cam->c[2]);
        c->ccx = (float)cam->c[0]; c->ccy = (float)cam->c[1]; c->ccz = (float)cam->c[2];
        c->t_horizon = sqrtf(fmaxf(0, c->cc2 - EARTH_R_KM * EARTH_R_KM));
        c->inv_t_horizon = 1.0f / fmaxf(1, c->t_horizon);
        c->hi_max_t = 12.0f / c->pix_ang;   // tiles where the ground footprint is under 12 km
        c->cgm = cosf((float)cam->gmst); c->sgm = sinf((float)cam->gmst);
        // pilot_shade: t · fp (km² Q8) against the squared footprints of the texture levels
        // (q = t · pixel angle · fp < size²), t Q4, the graticule's 1.5 px in degrees Q16 per
        // unit of t Q4 (Q12), and the haze's t / t_horizon (Q15 per unit of t Q4, Q16).
        const float T2 = HI_TEXEL_KM * HI_TEXEL_KM, L2 = LIGHT_TEXEL_KM * LIGHT_TEXEL_KM;
        for (int k = 0; k < 3; k++) {
            c->qt[k] = (int32_t)(T2 * (1 << (2 * k)) / c->pix_ang * 256);
            c->ql[k] = (int32_t)(L2 * (1 << (2 * k)) / c->pix_ang * 256);
        }
        c->hi_max_tq = (int32_t)(c->hi_max_t * 16);
        c->grat_w = (int32_t)(1.5f * c->pix_ang * 57.29578f / EARTH_R_KM * 65536 / 16 * 4096);
        c->haze_k = (uint32_t)(134217728.0f * c->inv_t_horizon);   // 2^27 / t_horizon
    }
}

// ---------- orthographic, any rotation ----------
// Where a screen point lands: texture column fu in [0, 1), row fv in level-0 texels, the
// sun's elevation and Z (for the lights level of detail). These vary smoothly, so between
// exactly computed points they are interpolated (SPAN).
typedef struct { float fu, fv, sun, z; } gp_t;

static inline void ortho_param(const rctx_t *c, float Xs, float Ys, gp_t *o)
{
    const float inv_2pi = 0.159154943f, inv_pi = 0.318309886f;
    float d = Xs * Xs + Ys * Ys;
    if (d > 1) d = 1;
    float Z = d < 1 ? (1 - d) * rsqrt_fast(1 - d) : 0;
    // Undo d3's [λ, φ, γ] rotation: spin back by γ, tilt back by φ, shift longitude by λ.
    float X = Xs * c->cg + Ys * c->sg;
    float Y = Ys * c->cg - Xs * c->sg;
    float xr = Z * c->cp + Y * c->sp, zr = Y * c->cp - Z * c->sp;
    float fu = (fast_atan2f(X, xr) - c->lam) * inv_2pi + 0.5f;
    float fv;
    if (fabsf(zr) < ASIN_LUT_MAX) {
        float fi = (zr + 1) * (LUT_N * 0.5f);
        int li = (int)fi;
        fv = asin_lut[li] + (asin_lut[li + 1] - asin_lut[li]) * (fi - li);
    } else {
        fv = (0.5f - fast_atan2f(zr, sqrtf(X * X + xr * xr)) * inv_pi) * TEX_H;
    }
    // Surface normal in Earth-fixed axes = (xr, X, zr) turned back by λ.
    float ex = xr * c->cl0 + X * c->sl0, ey = X * c->cl0 - xr * c->sl0;
    fu += 2;
    o->fu = fu - (float)(int)fu;
    o->fv = fv;
    o->sun = ex * c->sx + ey * c->sy + zr * c->sz;
    o->z = Z;
}

static inline void ortho_color(const rctx_t *c, const gp_t *p, int *r, int *g, int *b)
{
    float fu = p->fu + 2;
    fu -= (float)(int)fu;
    int u = (int)(fu * TEX_W), v = (int)p->fv;
    if (u >= TEX_W) u = TEX_W - 1;
    if (v >= TEX_H) v = TEX_H - 1;
    if (v < 0) v = 0;
    texel_rgb(c->level, u, v, r, g, b);
    if (p->sun < 0.1f) night(r, g, b, p->sun, globe_light_level(p->z), fu, p->fv * (1.0f / TEX_H));
}

#define SPAN 8

static inline bool span_linear(const gp_t *a, const gp_t *b, float *du)
{
    float d = b->fu - a->fu;
    if (d > 0.5f) d -= 1;
    if (d < -0.5f) d += 1;
    *du = d;
    return fabsf(d) < 0.03f && fabsf(b->fv - a->fv) < 32;
}

static inline void lerp_gp(const gp_t *a, const gp_t *b, float du, float t, gp_t *o)
{
    o->fu = a->fu + du * t;
    o->fv = a->fv + (b->fv - a->fv) * t;
    o->sun = a->sun + (b->sun - a->sun) * t;
    o->z = a->z + (b->z - a->z) * t;
}

static inline uint16_t ortho_px_exact(const rctx_t *c, int x, int y)
{
    gp_t p;
    int r, g, b;
    ortho_param(c, (x + 0.5f - CX) / R, (CY - y - 0.5f) / R, &p);
    ortho_color(c, &p, &r, &g, &b);
    int32_t i = row_off[y] + x - span_x0[y];
    uint16_t px = lit_pack(r, g, b, light_tab[0][i], DQ[y & 3][x & 3]);
    return cover[i] < 255 ? edge_mix(px, face[y * S + x], cover[i]) : px;
}

static void copy_face_row(uint16_t *row, int y)
{
    if (span_x1[y] <= span_x0[y]) { memcpy(row, face + y * S, S * 2); return; }
    memcpy(row, face + y * S, span_x0[y] * 2);
    memcpy(row + span_x1[y], face + y * S + span_x1[y], (S - span_x1[y]) * 2);
}

// A sample in fixed point: texture column as a fraction of a turn (u32 wraps by itself), row
// in level-0 texels Q8, sun elevation Q12, Z·255 Q8.
typedef struct { uint32_t u; int32_t v, sun, z; } gq_t;

static inline void to_fixed(const gp_t *p, gq_t *q)
{
    q->u = (uint32_t)(int32_t)((p->fu - 0.5f) * 4294967296.0f) + 0x80000000u;   // no 64-bit convert
    // (inside the texture with a margin: interpolated steps may fall a few 1/256 short of an end)
    q->v = (int32_t)(fminf(fmaxf(p->fv, 0.05f), TEX_H - 0.05f) * 256);
    q->sun = (int32_t)(p->sun * 4096);
    q->z = (int32_t)(p->z * 255 * 256);
}

static inline __attribute__((always_inline)) uint16_t sample_px(const rctx_t *c, const gq_t *q, uint16_t light, uint32_t d)
{
    const int lev = c->level;
    int u = q->u >> (21 + lev), v = (q->v >> 8);
    if (v > TEX_H - 1) v = TEX_H - 1;
    if (v < 0) v = 0;
    uint16_t t = texture[lev][TEX_INDEX(u, v >> lev, (TEX_W / 8) >> lev)];
    int L = 0;
    if (q->sun < NIGHT_HI) {
        int lv = lod_z[q->z >> 8];
        L = lights[lv][(v >> lv) * (TEX_W >> lv) + (q->u >> (21 + lv))];
    }
    return lit_px(t, q->sun, L, light, d);
}

// One row of samples at screen x = x0 + k·step (k < n), exact every SPAN samples and linear in
// between, written through put(k, pixel).
#define SAMPLE_ROW(X_OF, PUT, SPANPUT) do { \
    gp_t a_, b_, p_; gq_t qa_, qb_, q_; \
    ortho_param(c, (X_OF(0) - CX) * inv_r, Ys, &a_); \
    to_fixed(&a_, &qa_); \
    for (int k0_ = 0; k0_ < n_; ) { \
        int m_ = n_ - k0_ < SPAN ? n_ - k0_ : SPAN; \
        ortho_param(c, (X_OF(k0_ + m_) - CX) * inv_r, Ys, &b_); \
        to_fixed(&b_, &qb_); \
        float du_; \
        if (span_linear(&a_, &b_, &du_)) { \
            if (m_ == SPAN) { \
                q_ = qa_; \
                SPANPUT(k0_, q_, (int32_t)(du_ * (4294967296.0f / SPAN)), (qb_.v - qa_.v) >> 3, \
                        (qb_.sun - qa_.sun) >> 3, (qb_.z - qa_.z) >> 3); \
            } else { \
                LERP_RUN(PUT, k0_, m_, qa_, qb_, du_); \
            } \
        } else if (m_ >= 4) { \
            /* Not linear over the span (poles, limb): try its halves before exact samples. */ \
            const int h_ = m_ >> 1; \
            gp_t mid_; gq_t qm_; \
            ortho_param(c, (X_OF(k0_ + h_) - CX) * inv_r, Ys, &mid_); \
            to_fixed(&mid_, &qm_); \
            float dh_; \
            if (span_linear(&a_, &mid_, &dh_)) { LERP_RUN(PUT, k0_, h_, qa_, qm_, dh_); } \
            else EXACT_RUN(X_OF, PUT, k0_, h_, qa_); \
            if (span_linear(&mid_, &b_, &dh_)) { LERP_RUN(PUT, k0_ + h_, m_ - h_, qm_, qb_, dh_); } \
            else EXACT_RUN(X_OF, PUT, k0_ + h_, m_ - h_, qm_); \
        } else { \
            EXACT_RUN(X_OF, PUT, k0_, m_, qa_); \
        } \
        a_ = b_; qa_ = qb_; k0_ += m_; \
    } } while (0)

// Samples k0 .. k0 + m - 1 interpolated from qa (at k0) to qb (at k0 + m); du: the turn between.
#define LERP_RUN(PUT, k0, m, qa, qb, du) do { \
    const float im_ = inv_n[m]; \
    const int32_t ddu_ = (int32_t)((du) * 4294967296.0f * im_); \
    const int32_t dv_ = (int32_t)(((qb).v - (qa).v) * im_), ds_ = (int32_t)(((qb).sun - (qa).sun) * im_), \
                  dz_ = (int32_t)(((qb).z - (qa).z) * im_); \
    gq_t r_ = (qa); \
    for (int k_ = 0; k_ < (m); k_++) { \
        PUT((k0) + k_, r_); \
        r_.u += ddu_; r_.v += dv_; r_.sun += ds_; r_.z += dz_; \
    } } while (0)

// Samples k0 .. k0 + m - 1 exactly; qa: the sample at k0, known.
#define EXACT_RUN(X_OF, PUT, k0, m, qa) do { \
    PUT((k0), (qa)); \
    for (int k_ = 1; k_ < (m); k_++) { \
        ortho_param(c, (X_OF((k0) + k_) - CX) * inv_r, Ys, &p_); \
        to_fixed(&p_, &q_); \
        PUT((k0) + k_, q_); \
    } } while (0)

static const float inv_n[SPAN + 1] = {0, 1, 1 / 2.0f, 1 / 3.0f, 1 / 4.0f, 1 / 5.0f, 1 / 6.0f, 1 / 7.0f, 1 / 8.0f};

static void ortho_row_full(const rctx_t *c, int y)
{
    uint16_t *row = c->fb + y * S;
    copy_face_row(row, y);
    const float inv_r = 1.0f / R, Ys = (CY - y - 0.5f) * inv_r;
    const int32_t base = row_off[y] - span_x0[y];
    for (int x = span_x0[y]; x < in_x0[y]; x++) row[x] = ortho_px_exact(c, x, y);
    for (int x = in_x1[y]; x < span_x1[y]; x++) row[x] = ortho_px_exact(c, x, y);
    const int x0 = in_x0[y], n_ = in_x1[y] - x0;
    if (n_ <= 0) return;
    const uint16_t *lt = light_tab[0] + base + x0;
    const uint32_t *drow = DQ[y & 3];
    uint16_t *out = row + x0;
#define XF(k) (x0 + (k) + 0.5f)
#define PUTF(k, q) out[k] = sample_px(c, &(q), lt[k], drow[(x0 + (k)) & 3])
#define SPANF(k0, q, du, dv, ds, dz) for (int k_ = 0; k_ < SPAN; k_++) { PUTF(k0 + k_, q); q.u += du; q.v += dv; q.sun += ds; q.z += dz; }
    SAMPLE_ROW(XF, PUTF, SPANF);
#undef SPANF
#undef XF
#undef PUTF
}

static void ortho_rows_half(const rctx_t *c, int by)
{
    const float inv_r = 1.0f / R;
    int y0 = 2 * by;
    uint16_t *row0 = c->fb + y0 * S, *row1 = row0 + S;
    copy_face_row(row0, y0);
    copy_face_row(row1, y0 + 1);
    float Ys = (CY - y0 - 1.0f) * inv_r;
    const int32_t base = hrow_off[by] - hspan_x0[by];
    const int bx0 = hspan_x0[by], bx1 = hspan_x1[by], n_ = bx1 - bx0;
    if (n_ <= 0) return;
    // Every block from one sample (2x2 pixels); the blocks on the disc edge are then blended
    // into the face per pixel by its coverage.
    const uint16_t *lt = hlight_tab[0] + base + bx0;
    const uint32_t *drow = DQ[by & 3];   // dither per 2x2 block
    uint32_t *o0 = (uint32_t *)(row0 + 2 * bx0), *o1 = (uint32_t *)(row1 + 2 * bx0);   // 2 px each, aligned
#define XH(k) (2 * (bx0 + (k)) + 1.0f)
#define PUTH(k, q) do { uint32_t px_ = sample_px(c, &(q), lt[k], drow[(bx0 + (k)) & 3]); px_ |= px_ << 16; o0[k] = px_; o1[k] = px_; } while (0)
    // A whole linear span at texture level 1 (half resolution). The row needs no clamp here:
    // to_fixed keeps the span's ends inside the texture and the steps stay between them.
    const uint32_t hd[4] = {drow[bx0 & 3], drow[(bx0 + 1) & 3], drow[(bx0 + 2) & 3], drow[(bx0 + 3) & 3]};
#define SPANH(k0, q, du, dv, ds, dz) do { \
        const uint16_t *l_ = lt + (k0); \
        uint32_t *p0_ = o0 + (k0), *p1_ = o1 + (k0); \
        gq_t r_ = (q); \
        for (int k_ = 0; k_ < SPAN; k_++) { \
            const uint32_t d_ = hd[k_ & 3]; \
            const uint32_t u_ = r_.u >> 22, v_ = (uint32_t)r_.v >> 9; \
            uint32_t x_ = SPREAD(tex1[((v_ >> 3) * (TEX_W / 16) + (u_ >> 3)) << 6 | (v_ & 7) << 3 | (u_ & 7)]); \
            if (__builtin_expect(r_.sun < NIGHT_HI, 0)) { \
                int lv_ = lod_z[r_.z >> 8], vv_ = r_.v >> 8; \
                x_ = sw_night(x_, r_.sun, lights[lv_][(vv_ >> lv_) * (TEX_W >> lv_) + (r_.u >> (21 + lv_))], d_); \
            } \
            uint32_t px_ = UNSPREAD(sw_light(x_, l_[k_], d_)); \
            px_ |= px_ << 16; p0_[k_] = px_; p1_[k_] = px_; \
            r_.u += (du); r_.v += (dv); r_.sun += (ds); r_.z += (dz); \
        } \
    } while (0)
    const uint16_t *tex1 = texture[1];
    SAMPLE_ROW(XH, PUTH, SPANH);
#undef SPANH
#undef XH
#undef PUTH
    int ib = bx0, ie = bx1;
    while (ib < ie && !inner[base + ib]) ib++;
    while (ie > ib && !inner[base + ie - 1]) ie--;
    for (int bx = bx0; bx < bx1; bx++) {
        if (bx == ib && ie > ib) { bx = ie - 1; continue; }
        uint16_t px = c->fb[y0 * S + 2 * bx];
        for (int q = 0; q < 4; q++) {
            int x = 2 * bx + (q & 1), y = y0 + (q >> 1);
            uint16_t *o = &c->fb[y * S + x];
            if (x < span_x0[y] || x >= span_x1[y]) { *o = face[y * S + x]; continue; }
            int cv = cover[row_off[y] + x - span_x0[y]];
            *o = cv < 255 ? edge_mix(px, face[y * S + x], cv) : px;
        }
    }
}

// ---------- polar-axis paths: equatorial globe, flat map ----------
// The view only turns about the Earth's axis, so a pixel's longitude is a fixed table value
// plus the turn, and its latitude never changes: no trigonometry per pixel. Integer only.
// Exact pixel (disc edge pixels, with coverage).
static uint16_t axis_px(const rctx_t *c, bool flat, int x, int y, int32_t i)
{
    const float Xs = (x + 0.5f - CX) / R, Ys = (CY - y - 0.5f) / R;
    uint32_t t = axis_tab[flat][i], uf = ((t & 0xFFFF) + c->uoff) & 0xFFFF;
    int u = uf >> 5, v, level, sun;
    if (!flat) {
        v = eq_vrow[y];
        int z = (t >> 16) & 255;
        sun = (int)((z / 255.0f * c->P + Xs * c->Q + Ys * c->sz) * 4096);
        level = lod_z[z];
    } else {
        v = (t >> 16) & 1023;
        uint32_t nn = flat_nrm[i];
        sun = (int)(((int8_t)nn * c->P + (int8_t)(nn >> 8) * c->Q + (int8_t)(nn >> 16) * c->sz) * (4096 / 127.0f));
        level = 2;
    }
    int L = 0;
    if (sun < NIGHT_HI) { int w = TEX_W >> level; L = lights[level][(v >> level) * w + (uf >> (5 + level))]; }
    uint16_t px = lit_px(tex0(u, v), sun, L, light_tab[flat][i], DQ[y & 3][x & 3]);
    return cover[i] < 255 ? edge_mix(px, face[y * S + x], cover[i]) : px;
}

// Face background and the exact rim pixels of row y (the parts outside the inner span).
static void row_frame(const rctx_t *c, int y, bool flat)
{
    uint16_t *row = c->fb + y * S;
    copy_face_row(row, y);
    const int32_t base = row_off[y] - span_x0[y];
    for (int x = span_x0[y]; x < in_x0[y]; x++) row[x] = axis_px(c, flat, x, y, base + x);
    for (int x = in_x1[y]; x < span_x1[y]; x++) row[x] = axis_px(c, flat, x, y, base + x);
}

// Inner span of row y; step 2 samples every other pixel and writes it twice (moving views).
static inline __attribute__((always_inline)) void spin_inner(const rctx_t *c, int y, const int step)
{
    uint16_t *row = c->fb + y * S;
    const int32_t base = row_off[y] - span_x0[y];
    const int v = eq_vrow[y];
    const uint16_t *trow = texture[0] + ((v >> 3) * (TEX_W / 8) << 6) + ((v & 7) << 3);
    const uint8_t *lrow[4];
    for (int l = 1; l < 4; l++) lrow[l] = lights[l] + (v >> l) * (TEX_W >> l);
    // sun (Q12) = Z·P + Xs·Q + Ys·sz; the Xs term in Q20 so the per-pixel step does not drift
    const float Ys = (CY - y - 0.5f) / R;
    // Z·P in Q20 (8 extra bits): as an integer per unit of z it would change in 6% steps while
    // the Earth turns, and the terminator would move in jumps.
    const int Pz = (int)lrintf(c->P * 4096 * 256 / 255), rowq = (int)(Ys * c->sz * 4096);
    const float x0 = in_x0[y] + 0.5f * step;   // sample at the middle of each run of step pixels
    int32_t xq = (int32_t)((x0 - CX) / R * c->Q * 1048576), dxq = (int32_t)(c->Q / R * 1048576) * step;
    // Along a row of the equatorial view, longitude and depth are functions of t = Xs / cos(lat)
    // alone: one table in internal RAM instead of a per-pixel table in external RAM.
    const float cr = sqrtf(fmaxf(1e-6f, 1 - Ys * Ys));
    const float kq = (SPIN_N - 1) * 0.5f * 65536;
    int32_t tq = (int32_t)(((x0 - CX) / (R * cr) + 1) * kq + 32768), dtq = (int32_t)(kq / (R * cr)) * step;
    const int crq = (int)(cr * 256 + 0.5f);
    const uint16_t *lt = light_tab[0] + base;
    const uint32_t *drow = DQ[y & 3];
    const uint32_t uoff = c->uoff;
    const int xe = in_x1[y];
    int x = in_x0[y];
    const int ns = (xe - x + step - 1) / step;   // samples in the row
    if (ns >= 8 && tq >> 16 >= -1 && (tq + dtq * (ns - 1)) >> 16 <= SPIN_N) {
        // The table index in range across the row (no clamp): four samples per iteration with
        // their dither thresholds in registers. The sun's elevation as
        // zt·Kz + sacc (Q12 after >> 8) with zt = Z·255 / cos(lat) from the table.
        const int Kz = (crq * Pz) >> 8;
        int32_t sacc = xq + (rowq << 8);
        const int32_t dsq = dxq;
        const uint32_t d0 = drow[x & 3], d1 = drow[(x + step) & 3], d2 = drow[(x + 2 * step) & 3], d3 = drow[(x + 3 * step) & 3];
#define SPIN_PX(XX, D) do { \
            uint32_t t_ = spin_lut[tq >> 16], zt_ = t_ >> 16, uf_ = (t_ + uoff) & 0xFFFF, u_ = uf_ >> 5; \
            uint32_t px_ = SPREAD(trow[(u_ & ~7u) << 3 | (u_ & 7)]); \
            int sun_ = (((int)zt_ * Kz) >> 8) + (sacc >> 8); \
            if (__builtin_expect(sun_ < NIGHT_HI, 0)) { \
                int lv_ = lod_z[(zt_ * crq) >> 8]; \
                px_ = sw_night(px_, sun_, lrow[lv_][uf_ >> (5 + lv_)], D); \
            } \
            uint32_t l_ = lt[XX]; \
            px_ = ((px_ * ((l_ & 31) + 1) + (D)) >> 5) & SW_MASK; \
            if (l_ >> 5) { uint32_t s5_ = (l_ >> 5) & 31, s6_ = l_ >> 10; px_ = sw_sat_add(px_, s5_ | s5_ << 11 | s6_ << 21); } \
            row[XX] = UNSPREAD(px_); \
            if (step == 2 && (XX) + 1 < xe) row[(XX) + 1] = row[XX]; \
            tq += dtq; sacc += dsq; \
        } while (0)
        for (; x + 4 * step <= xe; x += 4 * step) {
            SPIN_PX(x, d0);
            SPIN_PX(x + step, d1);
            SPIN_PX(x + 2 * step, d2);
            SPIN_PX(x + 3 * step, d3);
        }
        for (; x < xe; x += step) SPIN_PX(x, drow[x & 3]);
#undef SPIN_PX
        return;
    }
    for (; x < xe; x += step, xq += dxq, tq += dtq) {
        int ti = tq >> 16;
        ti = ti < 0 ? 0 : ti > SPIN_N - 1 ? SPIN_N - 1 : ti;
        uint32_t t = spin_lut[ti], uf = (t + uoff) & 0xFFFF;
        int z = ((t >> 16) * crq) >> 8;
        uint32_t u = uf >> 5, px = SPREAD(trow[(u & ~7u) << 3 | (u & 7)]);
        int sun = ((z * Pz) >> 8) + (xq >> 8) + rowq;
        uint32_t d = drow[x & 3];
        if (__builtin_expect(sun < NIGHT_HI, 0)) { int lv = lod_z[z]; px = sw_night(px, sun, lrow[lv][uf >> (5 + lv)], d); }
        uint32_t l = lt[x];
        px = ((px * ((l & 31) + 1) + d) >> 5) & SW_MASK;
        if (l >> 5) { uint32_t s5 = (l >> 5) & 31, s6 = l >> 10; px = sw_sat_add(px, s5 | s5 << 11 | s6 << 21); }
        uint16_t o = UNSPREAD(px);
        row[x] = o;
        if (step == 2 && x + 1 < xe) row[x + 1] = o;
    }
}

static void spin_row(const rctx_t *c, int y)
{
    row_frame(c, y, false);
    spin_inner(c, y, 1);
}

// Rows of the texture (from the north pole) inside which the flat view uses texture level 1:
// where one pixel spans two texels of longitude, r < TEX_W / (2π · 2) px from the centre.
#define FLAT_MIP_V ((int)(TEX_H * TEX_W / (4 * 3.14159265f) / GLOBE_R))

static inline __attribute__((always_inline)) void flat_inner(const rctx_t *c, int y, const int step)
{
    uint16_t *row = c->fb + y * S;
    const int32_t base = row_off[y] - span_x0[y];
    const float k = 4096 / 127.0f;
    const float k8 = k * 256;   // Q8 extra precision, so the terminator turns smoothly
    const int Pq = (int)lrintf(c->P * k8), Qq = (int)lrintf(c->Q * k8), Sq = (int)lrintf(c->sz * k8);
    const uint32_t *ta = axis_tab[1] + base, *nr = flat_nrm + base;
    const uint16_t *lt = light_tab[1] + base;
    const uint32_t *drow = DQ[y & 3];
    const uint32_t uoff = c->uoff;
    const int xe = in_x1[y];
    for (int x = in_x0[y]; x < xe; x += step) {
        uint32_t d = drow[x & 3];
        uint32_t t = ta[x], uf = (t + uoff) & 0xFFFF, nn = nr[x];
        int u = uf >> 5, v = (t >> 16) & 1023;
        int sun = ((int8_t)nn * Pq + (int8_t)(nn >> 8) * Qq + (int8_t)(nn >> 16) * Sq) >> 8;
        // Near the pole a pixel spans several texels of longitude: the half-resolution level
        // there (less aliasing, and the texel walk stays in the cache).
        uint32_t px = SPREAD(v < FLAT_MIP_V ? texture[1][TEX_INDEX(u >> 1, v >> 1, TEX_W / 16)] : tex0(u, v));
        if (__builtin_expect(sun < NIGHT_HI, 0)) {   // lights footprint ≈ 65 km, coarser near the pole
            int L = v < FLAT_MIP_V ? lights[3][(v >> 3) * (TEX_W / 8) + (uf >> 8)] : lights[2][(v >> 2) * (TEX_W / 4) + (uf >> 7)];
            px = sw_night(px, sun, L, d);
        }
        uint32_t l = lt[x];
        px = ((px * ((l & 31) + 1) + d) >> 5) & SW_MASK;
        if (l >> 5) { uint32_t s5 = (l >> 5) & 31, s6 = l >> 10; px = sw_sat_add(px, s5 | s5 << 11 | s6 << 21); }
        uint16_t o = UNSPREAD(px);
        row[x] = o;
        if (step == 2 && x + 1 < xe) row[x + 1] = o;
    }
}

static void flat_row(const rctx_t *c, int y)
{
    row_frame(c, y, true);
    flat_inner(c, y, 1);
}

// Rows y, y + 1 (y even) at half resolution inside the disc: row y sampled every other pixel,
// row y + 1 copied from it where both rows are inner and exact elsewhere.
static void axis_rows_half(const rctx_t *c, int y, bool flat)
{
    row_frame(c, y, flat);
    if (flat) flat_inner(c, y, 2);
    else spin_inner(c, y, 2);
    if (y + 1 >= S) return;
    const int y1 = y + 1;
    row_frame(c, y1, flat);
    uint16_t *r0 = c->fb + y * S, *r1 = r0 + S;
    int a = in_x0[y] > in_x0[y1] ? in_x0[y] : in_x0[y1], b = in_x1[y] < in_x1[y1] ? in_x1[y] : in_x1[y1];
    if (a >= b) { a = b = in_x0[y1]; }
    if (b > a) memcpy(r1 + a, r0 + a, (b - a) * 2);
    const int32_t base = row_off[y1] - span_x0[y1];
    for (int x = in_x0[y1]; x < a; x++) r1[x] = axis_px(c, flat, x, y1, base + x);
    for (int x = b > in_x0[y1] ? b : in_x0[y1]; x < in_x1[y1]; x++) r1[x] = axis_px(c, flat, x, y1, base + x);
}

// ---------- pilot view ----------
typedef struct { bool ground; float fu, fv, sun, t, fp, cl; uint16_t px; } pp_t;

// One ray from the camera through screen point (Xs, Ys) (in screen radii, y up).
static void pilot_ray(const rctx_t *c, float Xs, float Ys, pp_t *o)
{
    const pilot_cam_t *cam = &c->v->cam;
    const float RE = EARTH_R_KM;
    float dx = cam->f[0] + (Xs * cam->rt[0] + Ys * cam->u[0]) * c->tanH;
    float dy = cam->f[1] + (Xs * cam->rt[1] + Ys * cam->u[1]) * c->tanH;
    float dz = cam->f[2] + (Xs * cam->rt[2] + Ys * cam->u[2]) * c->tanH;
    float inv = rsqrt_fast(dx * dx + dy * dy + dz * dz);
    dx *= inv; dy *= inv; dz *= inv;
    const float cx = c->ccx, cy = c->ccy, cz = c->ccz;
    float b = cx * dx + cy * dy + cz * dz;
    float disc = b * b - (c->cc2 - RE * RE);
    float t = disc >= 0 ? -b - disc * rsqrt_fast(fmaxf(disc, 1e-6f)) : -1;
    if (t <= 0) {
        // Sky: black space with a thin atmosphere glow along the limb, lit on the day side.
        float tc = fmaxf(0, -b);
        float qx = cx + tc * dx, qy = cy + tc * dy, qz = cz + tc * dz;
        float qq = qx * qx + qy * qy + qz * qz, qinv = rsqrt_fast(qq), qn = qq * qinv;
        int gi = (int)fmaxf(0, qn - RE);
        o->ground = false;
        // Beyond GLOW_FAINT km the glow adds less than one step to any channel: plain space.
        if (gi >= GLOW_FAINT) { o->px = gfx_pack(PILOT_BG.r, PILOT_BG.g, PILOT_BG.b); return; }
        float glow = glow_lut[gi];
        float sd = ((qx * c->cgm + qy * c->sgm) * c->sx + (-qx * c->sgm + qy * c->cgm) * c->sy + qz * c->sz) * qinv;
        float lit = smoothstep(-0.2f, 0.15f, sd);
        o->px = gfx_pack((int)(4 + 90 * glow * lit), (int)(6 + 150 * glow * lit),
                         (int)fminf(255, 14 + 255 * glow * (0.15f + 0.85f * lit)));
        return;
    }
    float nx = (cx + t * dx) / RE, ny = (cy + t * dy) / RE, nz = (cz + t * dz) / RE;
    float cosi = -(nx * dx + ny * dy + nz * dz);   // incidence of the view ray on the ground
    float ex = nx * c->cgm + ny * c->sgm, ey = -nx * c->sgm + ny * c->cgm;   // ECI → Earth-fixed
    float fu = fast_atan2f(ey, ex) * 0.159154943f + 0.5f;
    fu -= floorf(fu);
    float cl = sqrtf(ex * ex + ey * ey);
    float fv = (0.5f - fast_atan2f(nz, cl) * 0.318309886f) * TEX_H;
    o->ground = true;
    o->cl = cl;
    o->fu = fu;
    o->fv = fv;
    o->sun = ex * c->sx + ey * c->sy + nz * c->sz;
    o->t = t;
    o->fp = t * c->pix_ang / (cosi > 0.15f ? cosi : 0.15f);
}

// Filtered texture lookups for the pilot view. Each 2x2 block shows one sample, so a point
// sample of a texture finer than the block flickers as the view moves (coastlines, ridges):
// the level is chosen by the block's ground footprint and sampled bilinearly.
#define SW_RND (16u | 16u << 11 | 16u << 21)   // +0.5 in each spread field before >> 5

// A ground sample in fixed point (integer steps are cheap on this core, float ones have a long
// latency): texture column as a fraction of a turn (wraps by itself), row in level-0 texels Q16
// (inside the texture), sun elevation Q12, distance t (km Q4), footprint fp (km Q8), cos(lat) Q12.
typedef struct { uint32_t u; int32_t v, sun, t, fp, cl; } pq_t;

static inline void pilot_fixed(const pp_t *p, pq_t *q)
{
    q->u = (uint32_t)(int32_t)((p->fu - 0.5f) * 4294967296.0f) + 0x80000000u;   // no 64-bit convert
    q->v = (int32_t)(fminf(fmaxf(p->fv, 0), TEX_H * 0.9999f) * 65536);
    q->sun = (int32_t)(p->sun * 4096);
    q->t = (int32_t)(p->t * 16);
    q->fp = (int32_t)(p->fp * 256);
    q->cl = (int32_t)(p->cl * 4096);
}

static inline uint32_t mulhu(uint32_t a, uint32_t b) { return (uint32_t)(((uint64_t)a * b) >> 32); }

// Bilinear in an n x n RGB565 image at (x, y) in its texels Q16 (centres at +0.5), clamped at
// the edges; returns a spread pixel.
static inline uint32_t bil565(const uint16_t *img, int n, int32_t x, int32_t y)
{
    const int32_t mx = (n - 1) << 16;
    x -= 0x8000; y -= 0x8000;
    x = x < 0 ? 0 : x > mx ? mx : x;
    y = y < 0 ? 0 : y > mx ? mx : y;
    int x0 = x >> 16, y0 = y >> 16, x1 = x0 + (x0 < n - 1), y1 = y0 + (y0 < n - 1);
    uint32_t wx = (x >> 11) & 31, wy = (y >> 11) & 31;
    const uint16_t *r0 = img + y0 * n, *r1 = img + y1 * n;
    uint32_t top = ((SPREAD(r0[x0]) * (32 - wx) + SPREAD(r0[x1]) * wx + SW_RND) >> 5) & SW_MASK;
    uint32_t bot = ((SPREAD(r1[x0]) * (32 - wx) + SPREAD(r1[x1]) * wx + SW_RND) >> 5) & SW_MASK;
    return ((top * (32 - wy) + bot * wy + SW_RND) >> 5) & SW_MASK;
}

// The same for an n x n grey image.
static inline int bil8(const uint8_t *img, int n, int32_t x, int32_t y)
{
    const int32_t mx = (n - 1) << 16;
    x -= 0x8000; y -= 0x8000;
    x = x < 0 ? 0 : x > mx ? mx : x;
    y = y < 0 ? 0 : y > mx ? mx : y;
    int x0 = x >> 16, y0 = y >> 16, x1 = x0 + (x0 < n - 1), y1 = y0 + (y0 < n - 1);
    int wx = (x >> 8) & 255, wy = (y >> 8) & 255;
    const uint8_t *r0 = img + y0 * n, *r1 = img + y1 * n;
    int top = r0[x0] * (256 - wx) + r0[x1] * wx, bot = r1[x0] * (256 - wx) + r1[x1] * wx;
    return (top * (256 - wy) + bot * wy + 32768) >> 16;
}

// Global texture level lev (0: 2048 x 1024, 1: half), bilinear, wrapping east-west.
static inline uint32_t bil_global(int lev, uint32_t u, int32_t v)
{
    const int w = TEX_W >> lev, h = TEX_H >> lev, tx = (TEX_W / 8) >> lev;
    const int32_t gu = (int32_t)mulhu(u, (uint32_t)w << 16) - 0x8000, gv = (v >> lev) - 0x8000;
    int u0 = gu >> 16, v0 = gv >> 16;
    uint32_t wu = (gu >> 11) & 31, wv = (gv >> 11) & 31;
    if (u0 < 0) u0 += w;
    int u1 = u0 + 1 < w ? u0 + 1 : 0, v1 = v0 + 1 < h ? v0 + 1 : h - 1;
    if (v0 < 0) v0 = 0;
    const uint16_t *t = texture[lev];
    uint32_t top = ((SPREAD(t[TEX_INDEX(u0, v0, tx)]) * (32 - wu) + SPREAD(t[TEX_INDEX(u1, v0, tx)]) * wu + SW_RND) >> 5) & SW_MASK;
    uint32_t bot = ((SPREAD(t[TEX_INDEX(u0, v1, tx)]) * (32 - wu) + SPREAD(t[TEX_INDEX(u1, v1, tx)]) * wu + SW_RND) >> 5) & SW_MASK;
    return ((top * (32 - wv) + bot * wv + SW_RND) >> 5) & SW_MASK;
}

// City-light level lev of the global mip chain, bilinear.
static inline int bil_global_lights(int lev, uint32_t u, int32_t v)
{
    const int w = TEX_W >> lev, h = TEX_H >> lev;
    const int32_t gu = (int32_t)mulhu(u, (uint32_t)w << 16) - 0x8000, gv = (v >> lev) - 0x8000;
    int u0 = gu >> 16, v0 = gv >> 16;
    int wu = (gu >> 8) & 255, wv = (gv >> 8) & 255;
    if (u0 < 0) u0 += w;
    int u1 = u0 + 1 < w ? u0 + 1 : 0, v1 = v0 + 1 < h ? v0 + 1 : h - 1;
    if (v0 < 0) v0 = 0;
    const uint8_t *l0 = lights[lev] + v0 * w, *l1 = lights[lev] + v1 * w;
    int top = l0[u0] * (256 - wu) + l0[u1] * wu, bot = l1[u0] * (256 - wu) + l1[u1] * wu;
    return (top * (256 - wv) + bot * wv + 32768) >> 16;
}

#define SW_HAZE_DAY SPREAD(gfx_pack(12 + 120, 18 + 160, 40 + 210))
#define DEG_Q (1 << 16)   // degrees Q16
#define TEN_DEG (10 * DEG_Q)

// Distance from x (degrees Q16, |x| <= 180°) to the nearest multiple of 10°.
static inline int32_t to_ten(int32_t x)
{
    int32_t r = (int32_t)((uint32_t)(x + 100 * TEN_DEG) % TEN_DEG);
    return r < TEN_DEG / 2 ? r : TEN_DEG - r;
}

// The tiles a row of blocks last used (lookups are per tile, not per block).
typedef struct { int key, lkey, llev; const uint16_t *terrain; const uint8_t *lit; } tile_memo_t;

// Ground colour, in packed RGB565 integer steps: terrain (high-res tile levels near the camera,
// the global texture beyond), night with city lights, graticule, aerial haze. dq: the block's
// ordered-dither threshold (DQ).
static uint16_t pilot_shade(const rctx_t *c, const pq_t *p, uint32_t dq, tile_memo_t *memo)
{
    const uint32_t d = dq5(dq) << 3;   // the same threshold, Q8
    // Ground footprint of the 2x2 block and the texture level that matches it. Toward the
    // horizon the footprint is long in depth only (fp, stretched by 1 / cos incidence) and one
    // pixel across: their geometric mean sqrt(t · pixel angle · fp) picks the level, as the
    // longer side alone would blur. Compared as t · fp against the squared level sizes.
    const int32_t q = (int32_t)(((int64_t)p->t * p->fp) >> 4);   // t · fp, km² Q8 (64 bit: high orbits)
    const int tl = q < c->qt[0] ? 0 : q < c->qt[1] ? 1 : q < c->qt[2] ? 2 : -1;
    // The high-resolution tile under the sample and the position in it (level-0 texels Q16).
    int key = -1;
    int32_t fx = 0, fy = 0;
    if (tl >= 0 && p->t < c->hi_max_tq) {
        const int row = (p->v * HI_ROWS) >> 26;   // (fv / TEX_H) · rows
        const int cols = HI_ROW_COLS[row];
        const int col = (int)mulhu(p->u, cols);
        fy = (int32_t)(((int64_t)p->v * (HI_ROWS * HI_T)) >> 10) - row * (HI_T << 16);
        fx = (int32_t)mulhu(p->u, (uint32_t)(cols * HI_T) << 16) - col * (HI_T << 16);
        key = HI_ROW_START[row] + col;
        if (key != memo->key) { memo->key = key; memo->terrain = tiles_terrain(key, p->fp * (1 / 256.0f)); }
    }
    uint32_t x;
    if (key >= 0 && memo->terrain) {
        const uint16_t *lv = memo->terrain + (tl == 0 ? 0 : tl == 1 ? HI_T0_PX : HI_T0_PX + HI_T1_PX);
        x = bil565(lv, HI_T >> tl, fx >> tl, fy >> tl);
    } else {
        x = bil_global(q > c->ql[0] ? 1 : 0, p->u, p->v);
    }
    const int sunq = p->sun;
    int litq = 256;   // day side, for the haze colour
    if (sunq < NIGHT_HI) {
        int k = sunq - NIGHT_LO;
        if (k < 0) k = 0;
        litq = 256 - night_n[k];
        // City lights: tile levels 0 / 1 near the camera, else the global mip chain.
        int L = -1;
        const int ll = q < c->qt[0] ? 0 : q < c->qt[1] ? 1 : -1;
        if (ll >= 0 && key >= 0) {
            if (key != memo->lkey || ll != memo->llev) { memo->lkey = key; memo->llev = ll; memo->lit = tiles_lights(key, ll, p->fp * (1 / 256.0f)); }
            if (memo->lit) L = ll ? bil8(memo->lit, HI_T / 2, fx >> 1, fy >> 1) : bil8(memo->lit, HI_T, fx, fy);
        }
        if (L < 0) L = bil_global_lights(q <= c->ql[0] ? 0 : q <= c->ql[1] ? 1 : q <= c->ql[2] ? 2 : 3, p->u, p->v);
        x = sw_night(x, sunq, L, dq);
    }
    // Graticule every 10°, about one pixel wide, antialiased: blended by how close the block's
    // centre is to the line (a hard on/off line crawls as the view moves).
    const int32_t la = 90 * DEG_Q - (int32_t)(((uint32_t)p->v * 45u) >> 8);   // 90° - fv / TEX_H · 180°
    const int32_t lo = (int32_t)mulhu(p->u, 360u << 16) - 180 * DEG_Q;
    const int32_t dlat = to_ten(la), dlon = (to_ten(lo) * p->cl) >> 12;
    const int32_t dm = dlat < dlon ? dlat : dlon, w = (p->t * c->grat_w) >> 12;   // w: 1.5 px
    if (dm < w) {   // (rarely true: the division only near a line)
        float cov = 1 - (float)dm / w;
        uint32_t a = (uint32_t)(cov * 7 + (d >> 5) * (1.0f / 8));   // up to ~0.2 toward light grey
        if (a) x = ((x * (32 - a) + SPREAD(gfx_pack(225, 230, 235)) * a) >> 5) & SW_MASK;
    }
    // Aerial haze toward the horizon, 0.6 · (t / t_horizon)⁵ (dithered: 32 steps would band).
    uint32_t hq = ((uint32_t)p->t * c->haze_k) >> 16;   // Q15
    if (hq > 32768) hq = 32768;
    const uint32_t h2 = (hq * hq) >> 15, h5 = (((h2 * h2) >> 15) * hq) >> 15;
    const int a = (int)((h5 * 4915 >> 15) + d) >> 8;   // 19.2 = 0.6 · 32, Q8
    if (a) {
        uint32_t hc = litq == 256 ? SW_HAZE_DAY : SPREAD(gfx_pack(12 + (120 * litq >> 8), 18 + (160 * litq >> 8), 40 + (210 * litq >> 8)));
        x = ((x * (32 - a) + hc * a) >> 5) & SW_MASK;
    }
    return UNSPREAD(x);
}

// Rim vignette at (dx, dy) from the centre (whole pixels): pilot_rim on (r / c)²,
// RIM_LUT_N · 4 / S² as 8285 / 2^20.
static inline uint16_t pilot_finish(uint16_t px, int dx, int dy)
{
    const int i = ((dx * dx + dy * dy) * 8285) >> 20;
    const int m = pilot_rim[i < RIM_LUT_N ? i : RIM_LUT_N];
    return UNSPREAD(((SPREAD(px) * ((m + 4) >> 3)) >> 5) & SW_MASK);
}

static inline bool pilot_span_linear(const pp_t *a, const pp_t *b, float *du)
{
    if (!a->ground || !b->ground) return false;
    float d = b->fu - a->fu;
    if (d > 0.5f) d -= 1;
    if (d < -0.5f) d += 1;
    *du = d;
    // Near the horizon the distance changes fast and perspective bends the mapping: exact there.
    return fabsf(d) < 0.002f && fabsf(b->fv - a->fv) < 6 && fabsf(b->t - a->t) < 0.03f * a->t;
}

#define PSPAN 4

static void pilot_rows_half(const rctx_t *c, int by)
{
    const float inv_c = 2.0f / S;
    int y0 = 2 * by;
    uint16_t *row0 = c->fb + y0 * S, *row1 = row0 + S;
    const uint16_t bg = gfx_pack(PILOT_BG.r, PILOT_BG.g, PILOT_BG.b);
    int x0 = pspan_x0[y0] < pspan_x0[y0 + 1] ? pspan_x0[y0] : pspan_x0[y0 + 1];
    int x1 = pspan_x1[y0] > pspan_x1[y0 + 1] ? pspan_x1[y0] : pspan_x1[y0 + 1];
    const float Ys = (CY - y0 - 1.0f) * inv_c;
    const int bx0 = x0 / 2, bx1 = (x1 + 1) / 2;
    // Background outside the blocks (the edge blocks fill their uncovered pixels themselves).
    const int bg0 = bx1 > bx0 ? 2 * bx0 : S, bg1 = bx1 > bx0 ? 2 * bx1 : S;
    for (int x = 0; x < bg0; x++) row0[x] = row1[x] = bg;
    for (int x = bg1; x < S; x++) row0[x] = row1[x] = bg;
    if (bx1 <= bx0) return;
    // Blocks wholly inside the round screen need no edge coverage: those with
    // max(|x + 0.5 - CX|, |x + 1.5 - CX|)² <= hin2, i.e. block columns x in [xin0, xin1].
    const float rin = S / 2.0f - 1.5f, dyc = fmaxf(fabsf(y0 + 0.5f - CY), fabsf(y0 + 1.5f - CY));
    const float hin2 = rin * rin - dyc * dyc, hin = hin2 > 0 ? sqrtf(hin2) : -1;
    const int xin0 = hin < 0 ? S : (int)ceilf(CX - 0.5f - hin), xin1 = hin < 0 ? -S : (int)floorf(CX - 1.5f + hin);
    // The rim vignette starts at 0.75 of the radius (pilot_rim is 255 inside: no change): it
    // applies outside (x + 1 - CX)² < rim_in2, x in (xr0, xr1).
    const float dyb = y0 + 1.0f - CY, rim_in2 = (0.75f * S / 2) * (0.75f * S / 2) - dyb * dyb - 1.0f;
    const float rr = rim_in2 > 0 ? sqrtf(rim_in2) : -1;
    const int xr0 = rr < 0 ? S : (int)floorf(CX - 1 - rr), xr1 = rr < 0 ? -S : (int)ceilf(CX - 1 + rr);
    pp_t a, b, p;
    pq_t qa, qb, q;
    tile_memo_t memo = {-1, -1, -1, NULL, NULL};
    pilot_ray(c, (2 * bx0 + 1.0f - CX) * inv_c, Ys, &a);
    if (a.ground) pilot_fixed(&a, &qa);
    for (int k0 = bx0; k0 < bx1;) {
        int m = bx1 - k0 < PSPAN ? bx1 - k0 : PSPAN;
        pilot_ray(c, (2 * (k0 + m) + 1.0f - CX) * inv_c, Ys, &b);
        if (b.ground) pilot_fixed(&b, &qb);
        float du;
        bool lin = pilot_span_linear(&a, &b, &du);
        // Linear: fixed-point steps from a to b (the turn by the shorter way round).
        pq_t dq = {0};
        if (lin) {
            const float im = 1.0f / m;
            dq.u = (uint32_t)(int32_t)(du * 4294967296.0f * im);
            if (m == PSPAN) {
                dq.v = (qb.v - qa.v) >> 2; dq.sun = (qb.sun - qa.sun) >> 2; dq.t = (qb.t - qa.t) >> 2;
                dq.fp = (qb.fp - qa.fp) >> 2; dq.cl = (qb.cl - qa.cl) >> 2;
            } else {
                dq.v = (int32_t)((qb.v - qa.v) * im); dq.sun = (int32_t)((qb.sun - qa.sun) * im);
                dq.t = (int32_t)((qb.t - qa.t) * im); dq.fp = (int32_t)((qb.fp - qa.fp) * im);
                dq.cl = (int32_t)((qb.cl - qa.cl) * im);
            }
            q = qa;
        }
        // Sky at both ends: the sky's colour changes slowly (glow along the limb), blend it.
        const bool sky = !a.ground && !b.ground;
        const uint32_t sa = SPREAD(a.px), sb = SPREAD(b.px);
        for (int k = 0; k < m; k++) {
            int bx = k0 + k, x = 2 * bx;
            const uint32_t dth = DQ[by & 3][bx & 3];
            uint16_t px;
            if (k == 0) {
                px = a.ground ? pilot_shade(c, &qa, dth, &memo) : a.px;
            } else if (sky && m == PSPAN) {
                px = UNSPREAD(((sa * (PSPAN - k) + sb * k) >> 2) & SW_MASK);
            } else if (lin) {
                q.u += dq.u; q.v += dq.v; q.sun += dq.sun; q.t += dq.t; q.fp += dq.fp; q.cl += dq.cl;
                px = pilot_shade(c, &q, dth, &memo);
            } else {
                pilot_ray(c, (x + 1.0f - CX) * inv_c, Ys, &p);
                if (p.ground) { pq_t qe; pilot_fixed(&p, &qe); px = pilot_shade(c, &qe, dth, &memo); }
                else px = p.px;
            }
            if (x <= xr0 || x >= xr1) px = pilot_finish(px, x + 1 - (int)CX, y0 + 1 - (int)CY);
            if (x >= xin0 && x <= xin1) {
                const uint32_t pp = px | (uint32_t)px << 16;   // x is even: aligned pairs
                *(uint32_t *)(row0 + x) = pp;
                *(uint32_t *)(row1 + x) = pp;
                continue;
            }
            for (int e = 0; e < 4; e++) {
                int xx = x + (e & 1), yy = y0 + (e >> 1);
                float ddx = xx + 0.5f - CX, ddy = yy + 0.5f - CY;
                float cov = S / 2.0f + 0.5f - sqrtf(ddx * ddx + ddy * ddy);
                c->fb[yy * S + xx] = cov <= 0 ? bg : cov >= 1 ? px : edge_mix(px, bg, (int)(cov * 255));
            }
        }
        a = b;
        qa = qb;
        k0 += m;
    }
}

// ---------- entry points ----------
bool globe_is_equatorial(const globe_view_t *v)
{
    return v->kind == GLOBE_ORTHO && fabsf(v->rot[1]) < 1e-4f && fabsf(v->rot[2]) < 1e-4f;
}

void globe_rows(uint16_t *fbv, const globe_view_t *v, int y0, int y1, bool half)
{
    rctx_t c;
    rctx_init(&c, v, fbv, half ? 1 : 0);
    if (v->kind == GLOBE_PILOT) {
        for (int by = y0 / 2; by < y1 / 2; by++) pilot_rows_half(&c, by);
    } else if ((v->kind == GLOBE_FLAT || globe_is_equatorial(v)) && half) {
        for (int y = y0 & ~1; y < y1; y += 2) axis_rows_half(&c, y, v->kind == GLOBE_FLAT);
    } else if (v->kind == GLOBE_FLAT || globe_is_equatorial(v)) {
        if (v->kind == GLOBE_FLAT) for (int y = y0; y < y1; y++) flat_row(&c, y);
        else for (int y = y0; y < y1; y++) spin_row(&c, y);
    } else if (half) {
        for (int by = y0 / 2; by < y1 / 2; by++) ortho_rows_half(&c, by);
    } else {
        for (int y = y0; y < y1; y++) ortho_row_full(&c, y);
    }
}

rgb_t globe_shaded_stroke(int x, int y, rgb_t col, bool flat, float *alpha)
{
    if ((unsigned)y >= S || x < span_x0[y] || x >= span_x1[y]) { *alpha = 0; return col; }
    // The rim fade from the full table (external RAM) only near the edge; the light per cell.
    // (in half pixels from the centre, integers: 2·(x + 0.5 - CX))
    const int dx = 2 * x + 1 - (int)(2 * CX), dy = 2 * y + 1 - (int)(2 * CY);
    static const int RIN2 = (int)(4 * (GLOBE_R - 3) * (GLOBE_R - 3));
    *alpha = 1;
    if (dx * dx + dy * dy > RIN2) *alpha = cover[row_off[y] + x - span_x0[y]] * (1 / 255.0f);
    uint16_t l = shade_grid[flat][(y / SG_C) * SG_N + x / SG_C];
    uint32_t m = (l & 31) + 1, add = ((l >> 5) & 31) << 3;
    int r = ((col.r * m) >> 5) + add, g = ((col.g * m) >> 5) + add, b = ((col.b * m) >> 5) + add;
    return (rgb_t){(uint8_t)(r > 255 ? 255 : r), (uint8_t)(g > 255 ? 255 : g), (uint8_t)(b > 255 ? 255 : b)};
}

// ---------- row jobs on both cores ----------
typedef struct {
    void (*fn)(int row, const void *ctx);
    const void *ctx;
    int nrows;
    SemaphoreHandle_t start, done;
} pool_t;

static void pool_helper(void *arg)
{
    pool_t *p = arg;
    for (;;) {
        xSemaphoreTake(p->start, portMAX_DELAY);
        for (int r = 1; r < p->nrows; r += 2) p->fn(r, p->ctx);
        xSemaphoreGive(p->done);
    }
}

static void pool_init(pool_t *p, const char *name, int core, int prio, int stack)
{
    p->start = xSemaphoreCreateBinary();
    p->done = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(pool_helper, name, stack, p, prio, NULL, core);
}

static void pool_run(pool_t *p, void (*fn)(int, const void *), const void *ctx, int nrows)
{
    p->fn = fn; p->ctx = ctx; p->nrows = nrows;
    xSemaphoreGive(p->start);
    for (int r = 0; r < nrows; r += 2) fn(r, ctx);
    xSemaphoreTake(p->done, portMAX_DELAY);
}

// Frame loop (core 0, priority 5) + helper on core 1 above the background render.
static pool_t frame_pool;
// Background task (core 1, priority 3) + helper on core 0 below the frame loop.
static pool_t bg_pool;

void globe_both_cores(void (*fn)(int part, const void *ctx), const void *ctx)
{
    pool_run(&frame_pool, fn, ctx, 2);
}

// ---------- background full-resolution cache ----------
static uint16_t *cache[2];
static globe_view_t cache_view[2];
static double cache_t[2];
static int front = -1;
static SemaphoreHandle_t cache_lock;
static TaskHandle_t bg_task;
static globe_view_t req_view;
static double req_t;
static volatile bool bg_busy, cancel_bg;
static volatile int renders;
static volatile float render_ms_sum;

static void bg_row(int y, const void *vc)
{
    if (cancel_bg) return;
    const rctx_t *c = vc;
    if (c->v->kind == GLOBE_FLAT) flat_row(c, y);
    else ortho_row_full(c, y);
}

static void (*bake_fn)(uint16_t *fb, const globe_view_t *v);

void globe_cache_bake(void (*fn)(uint16_t *fb, const globe_view_t *v)) { bake_fn = fn; }

static void bg_task_fn(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int back = front < 0 ? 0 : 1 - front;
        globe_view_t v = req_view;
        double t = req_t;
        int64_t t0 = esp_timer_get_time();
        rctx_t c;
        rctx_init(&c, &v, cache[back], 0);
        pool_run(&bg_pool, bg_row, &c, S);
        if (!cancel_bg && bake_fn) bake_fn(cache[back], &v);
        if (!cancel_bg) {
            render_ms_sum += (esp_timer_get_time() - t0) / 1000.0f;
            renders++;
            xSemaphoreTake(cache_lock, portMAX_DELAY);
            cache_view[back] = v;
            cache_t[back] = t;
            front = back;
            xSemaphoreGive(cache_lock);
        }
        bg_busy = false;
    }
}

void globe_cache_request(const globe_view_t *v, double t)
{
    if (bg_busy) return;
    bg_busy = true;
    cancel_bg = false;
    req_view = *v;
    req_t = t;
    xTaskNotifyGive(bg_task);
}

bool globe_cache_busy(void) { return bg_busy; }
void globe_cache_cancel(void) { if (bg_busy) cancel_bg = true; }

const uint16_t *globe_cache_hold(globe_view_t *v, double *t)
{
    xSemaphoreTake(cache_lock, portMAX_DELAY);
    if (front < 0) { xSemaphoreGive(cache_lock); return NULL; }
    *v = cache_view[front];
    *t = cache_t[front];
    return cache[front];
}

void globe_cache_release(void) { xSemaphoreGive(cache_lock); }

void globe_cache_stats(int *count, float *avg_ms)
{
    *count = renders;
    *avg_ms = renders ? render_ms_sum / renders : 0;
    renders = 0;
    render_ms_sum = 0;
}

void globe_init(void)
{
    texture[0] = heap_caps_malloc(TEX_W * TEX_H * 2, MALLOC_CAP_SPIRAM);
    memcpy(texture[0], texture_bin_start, TEX_W * TEX_H * 2);
    texture[1] = heap_caps_malloc(TEX_W * TEX_H / 2, MALLOC_CAP_SPIRAM);
    memcpy(texture[1], texture_half_bin_start, TEX_W * TEX_H / 2);

    const uint8_t *lp = lights_bin_start;
    for (int k = 0; k < 4; k++) {
        size_t n = (TEX_W >> k) * (TEX_H >> k);
        lights[k] = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
        memcpy(lights[k], lp, n);
        lp += n;
    }
    face = heap_caps_malloc(S * S * 2, MALLOC_CAP_SPIRAM);
    build_luts();
    build_dither();
    int64_t t0 = esp_timer_get_time();
    build_static_layers();
    ESP_LOGI(TAG, "static layers built in %lld ms", (esp_timer_get_time() - t0) / 1000);
    for (int k = 0; k < 2; k++) {
        cache[k] = heap_caps_malloc(S * S * 2, MALLOC_CAP_SPIRAM);
        for (int y = 0; y < S; y++) copy_face_row(cache[k] + y * S, y);
    }
    cache_lock = xSemaphoreCreateMutex();
    pool_init(&frame_pool, "frame_c1", 1, 6, 16384);
    pool_init(&bg_pool, "globe_b", 0, 2, 4096);
    xTaskCreatePinnedToCore(bg_task_fn, "globe_bg", 4096, NULL, 3, &bg_task, 1);
    tiles_init();
    // Light per 8x8 pixel cell, for colouring the strokes printed on the globe.
    for (int k = 0; k < 2; k++) shade_grid[k] = heap_caps_malloc(SG_N * SG_N * 2, MALLOC_CAP_SPIRAM);
    for (int k = 0; k < 2; k++)
        for (int cy = 0; cy < SG_N; cy++)
            for (int cx = 0; cx < SG_N; cx++) {
                int x = cx * SG_C + SG_C / 2, y = cy * SG_C + SG_C / 2;
                bool in = x >= span_x0[y] && x < span_x1[y];
                shade_grid[k][cy * SG_N + cx] = in ? light_tab[k][row_off[y] + x - span_x0[y]] : 31;
            }
}
