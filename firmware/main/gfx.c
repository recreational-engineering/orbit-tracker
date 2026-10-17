#include <math.h>
#include <string.h>
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include <stdlib.h>
#include "gfx.h"

// One drawing context per CPU core, so a task on each core can draw at the same time.
#define COV_LUT_K 16      // thin-stroke coverage table: entries per pixel of distance
#define COV_LUT_N 48
typedef struct {
    uint16_t *fb;
    int ylo, yhi;           // rows this core draws: ylo <= y < yhi
    float acc[GFX_W + 1];   // polygon coverage row
    float lut_w;            // stroke width the coverage table is for
    uint16_t lut[COV_LUT_N];
} gfx_ctx_t;

static gfx_ctx_t ctxs[2];

static inline gfx_ctx_t *cur(void) { return &ctxs[esp_cpu_get_core_id()]; }
static inline float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
static inline int imin(int a, int b) { return a < b ? a : b; }
static inline int imax(int a, int b) { return a > b ? a : b; }

void gfx_init(void)
{
    for (int i = 0; i < 2; i++) { ctxs[i].ylo = 0; ctxs[i].yhi = GFX_H; }
}

void gfx_target(uint16_t *target) { cur()->fb = target; }

void gfx_clip_rows(int y0, int y1)
{
    gfx_ctx_t *g = cur();
    g->ylo = imax(0, y0);
    g->yhi = imin(GFX_H, y1);
}
uint16_t *gfx_get_target(void) { return cur()->fb; }

void gfx_get_clip_rows(int *y0, int *y1)
{
    const gfx_ctx_t *g = cur();
    *y0 = g->ylo;
    *y1 = g->yhi;
}

static inline void blend256(uint16_t *p, rgb_t c, int a)
{
    if (a <= 0) return;
    if (a >= 256) { *p = gfx_pack(c.r, c.g, c.b); return; }
    int r, g, b;
    gfx_unpack(*p, &r, &g, &b);
    r += ((c.r - r) * a) >> 8;
    g += ((c.g - g) * a) >> 8;
    b += ((c.b - b) * a) >> 8;
    *p = gfx_pack(r, g, b);
}

void gfx_blend(int x, int y, rgb_t c, float a)
{
    gfx_ctx_t *g = cur();
    if ((unsigned)x >= GFX_W || y < g->ylo || y >= g->yhi) return;
    blend256(&g->fb[y * GFX_W + x], c, (int)(a * 256.0f + 0.5f));
}

// Box-filter coverage of a pixel centre at distance d from a stroke's centre line.
static inline float band(float d, float hw)
{
    return clamp01(fminf(d + 0.5f, hw) - fmaxf(d - 0.5f, -hw));
}

// ---------- thin strokes ----------
static inline int ifloor(float v) { return (int)(v + 4096.0f) - 4096; }

// Packed RGB565 with gaps between the channels, so one multiply blends all three (alpha 0..32).
static inline uint32_t spread565(uint32_t p) { return (p | p << 16) & 0x07E0F81Fu; }
static inline uint16_t unspread565(uint32_t v) { v &= 0x07E0F81Fu; return (uint16_t)(v | v >> 16); }
// 4x4 ordered dither for the 5-bit alpha, in units of 1/2048 (alpha scale 65536 -> 32).
static const uint16_t dither_a[16] = {64, 1088, 320, 1344, 1600, 576, 1856, 832,
                                      448, 1472, 192, 1216, 1984, 960, 1728, 704};

static void thin_impl(gfx_ctx_t *g, float x0, float y0, float x1, float y1, float w, rgb_t c, int ai,
                      float on, float off, float dash_start, gfx_pixel_color_t color, void *ctx)
{
    float dx = x1 - x0, dy = y1 - y0;
    if (fabsf(dx) + fabsf(dy) < 1e-4f) return;
    if (g->lut_w != w) {   // coverage by perpendicular distance, for this width
        g->lut_w = w;
        for (int k = 0; k < COV_LUT_N; k++) g->lut[k] = (uint16_t)(band((k + 0.5f) / COV_LUT_K, w * 0.5f) * 256 + 0.5f);
    }
    bool steep = fabsf(dy) > fabsf(dx);
    // Work in (major, minor) coordinates; m runs along the major axis.
    float m0 = steep ? y0 : x0, n0 = steep ? x0 : y0, m1 = steep ? y1 : x1, n1 = steep ? x1 : y1;
    float dm = m1 - m0, grad = (n1 - n0) / dm;
    float sec = sqrtf(1 + grad * grad);          // 1 / cos of the angle to the major axis
    float lut_k = COV_LUT_K / sec, reach = (w * 0.5f + 0.5f) * sec;
    // Pixel centres i + 0.5 between m0 and m1, half open in the stroke direction, limited to
    // this core's rows: directly when the major axis is vertical, else to the columns whose
    // centre line passes within reach of them.
    int m_lo = steep ? g->ylo : 0, m_hi = steep ? g->yhi : GFX_W;
    const int n_lo = steep ? 0 : g->ylo, n_hi = steep ? GFX_W : g->yhi;
    if (!steep && fabsf(grad) > 1e-6f) {
        float ia = m0 - 0.5f + (g->ylo - reach - n0) / grad, ib = m0 - 0.5f + (g->yhi + reach - n0) / grad;
        if (ia > ib) { float t = ia; ia = ib; ib = t; }
        ia = fmaxf(ia, -8.0f); ib = fminf(ib, GFX_W + 8.0f);
        m_lo = imax(m_lo, ifloor(ia));
        m_hi = imin(m_hi, ifloor(ib) + 2);
    } else if (!steep && (n0 + reach < g->ylo || n0 - reach > g->yhi)) {
        return;
    }
    int i0, i1, step;
    if (dm > 0) {
        i0 = imax(-ifloor(0.5f - m0), m_lo);   // ceil(m0 - 0.5)
        i1 = imin(-ifloor(0.5f - m1), m_hi);
        step = 1;
        if (i0 >= i1) return;
    } else {
        i0 = imin(ifloor(m0 - 0.5f), m_hi - 1);
        i1 = imax(ifloor(m1 - 0.5f), m_lo - 1);
        step = -1;
        if (i0 <= i1) return;
    }
    float centre = n0 + (i0 + 0.5f - m0) * grad, dcentre = grad * step;
    // Dash phase along the stroke, advanced per step.
    float phase = 0, dphase = 0, period = on + off;
    bool dashed = on > 0;
    if (dashed) {
        float len = sqrtf(dx * dx + dy * dy);
        phase = fmodf(dash_start + (i0 + 0.5f - m0) / dm * len, period);
        if (phase < 0) phase += period;
        dphase = len / fabsf(dm);
    }
    uint16_t *fb = g->fb;
    if (color) {
        for (int i = i0; i != i1; i += step, centre += dcentre) {
            if (dashed) {
                float ph = phase;
                phase += dphase;
                while (phase >= period) phase -= period;
                if (ph >= on) continue;
            }
            int j0 = imax(n_lo, ifloor(centre - reach)), j1 = imin(n_hi - 1, ifloor(centre + reach));
            for (int j = j0; j <= j1; j++) {
                int k = (int)(fabsf(j + 0.5f - centre) * lut_k);
                if (k >= COV_LUT_N) continue;
                int cov = g->lut[k];
                if (!cov) continue;
                int x = steep ? j : i, y = steep ? i : j;
                float ka = 1;
                rgb_t col = color(x, y, c, ctx, &ka);
                blend256(fb + y * GFX_W + x, col, (int)(cov * ai * ka) >> 8);
            }
        }
        return;
    }
    const uint32_t cs = spread565(gfx_pack(c.r, c.g, c.b));
    if (w <= 1.0f) {
        // Hairline: the two pixels around the centre line, weighted by distance (Wu), with
        // the column's total coverage w·sec like the box filter below.
        const int32_t tot = (int32_t)(fminf(1.0f, w * sec) * ai * 16.0f);   // alpha·4096
        int32_t cq = (int32_t)((centre - 0.5f) * 4096.0f), dcq = (int32_t)(dcentre * 4096.0f);
        for (int i = i0; i != i1; i += step, cq += dcq) {
            if (dashed) {
                float ph = phase;
                phase += dphase;
                while (phase >= period) phase -= period;
                if (ph >= on) continue;
            }
            int j = cq >> 12, f = cq & 4095;
            int wt[2] = {4096 - f, f};
            for (int h = 0; h < 2; h++, j++) {
                if (j < n_lo || j >= n_hi) continue;
                int x = steep ? j : i, y = steep ? i : j;
                int a = ((wt[h] * tot >> 12) + (dither_a[(y & 3) << 2 | (x & 3)] >> 4)) >> 7;
                if (!a) continue;
                uint16_t *p = fb + y * GFX_W + x;
                *p = a >= 32 ? unspread565(cs) : unspread565((cs * a + spread565(*p) * (32 - a)) >> 5);
            }
        }
        return;
    }
    // Fixed point (Q12) along the minor axis; packed-colour blend with dithered 5-bit alpha.
    const uint16_t *lut = g->lut;
    int32_t cq = (int32_t)((centre - 0.5f) * 4096.0f), dcq = (int32_t)(dcentre * 4096.0f);
    const int32_t rq = (int32_t)(reach * 4096.0f) + 2048;
    const int32_t lk = (int32_t)(lut_k * 256.0f);
    for (int i = i0; i != i1; i += step, cq += dcq) {
        if (dashed) {
            float ph = phase;
            phase += dphase;
            while (phase >= period) phase -= period;
            if (ph >= on) continue;
        }
        int j0 = imax(n_lo, (cq + 2048 - rq) >> 12), j1 = imin(n_hi - 1, (cq + 2048 + rq) >> 12);
        for (int j = j0; j <= j1; j++) {
            int32_t d = (j << 12) - cq;
            int k = ((d < 0 ? -d : d) * lk) >> 20;
            if (k >= COV_LUT_N) continue;
            int x = steep ? j : i, y = steep ? i : j;
            int a = (lut[k] * ai + dither_a[(y & 3) << 2 | (x & 3)]) >> 11;
            if (!a) continue;
            uint16_t *p = fb + y * GFX_W + x;
            *p = a >= 32 ? unspread565(cs) : unspread565((cs * a + spread565(*p) * (32 - a)) >> 5);
        }
    }
}

void gfx_thin_line(float x0, float y0, float x1, float y1, float w, rgb_t c, float a,
                   gfx_dash_t *dash, gfx_pixel_color_t color, void *ctx)
{
    gfx_ctx_t *g = cur();
    float start = 0, on = 0, off = 0;
    if (dash) {
        start = dash->pos; on = dash->on; off = dash->off;
        dash->pos += sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    }
    float m = w * 0.5f + 2;
    if (fminf(y0, y1) - m >= g->yhi || fmaxf(y0, y1) + m < g->ylo) return;
    thin_impl(g, x0, y0, x1, y1, w, c, (int)(a * 256.0f + 0.5f), on, off, start, color, ctx);
}

void gfx_thin_seg(float x0, float y0, float x1, float y1, float w, rgb_t c, int a256,
                  float on, float off, float phase)
{
    thin_impl(cur(), x0, y0, x1, y1, w, c, a256, on, off, phase, NULL, NULL);
}

// ---------- thick strokes ----------
typedef struct {
    float ax, ay, ux, uy, len, hw, reach;
    bool round;
    int x0, y0, x1, y1;   // pixel bbox, inclusive, clipped
} seg_t;

static bool seg_setup(seg_t *s, float x0, float y0, float x1, float y1, float w, bool round_cap)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    s->ax = x0; s->ay = y0; s->len = len; s->hw = w * 0.5f; s->round = round_cap;
    s->reach = s->hw + 0.75f;   // beyond this a pixel centre cannot be covered (0.5 * sqrt 2 < 0.75)
    if (len > 1e-6f) { s->ux = dx / len; s->uy = dy / len; }
    else { s->ux = 1; s->uy = 0; }
    float ext = s->hw + 1.0f;
    s->x0 = imax(0, (int)floorf(fminf(x0, x1) - ext));
    s->x1 = imin(GFX_W - 1, (int)ceilf(fmaxf(x0, x1) + ext));
    s->y0 = imax(0, (int)floorf(fminf(y0, y1) - ext));
    s->y1 = imin(GFX_H - 1, (int)ceilf(fmaxf(y0, y1) + ext));
    return s->x0 <= s->x1 && s->y0 <= s->y1 && (len > 1e-6f || round_cap);
}

// Columns of row y the stroke can cover (long diagonal strokes skip most of their box).
static bool seg_row(const seg_t *s, int y, int *xa, int *xb)
{
    if (s->len < 4) { *xa = s->x0; *xb = s->x1; return true; }
    float py = y + 0.5f - s->ay;
    float lo = s->x0 + 0.5f, hi = s->x1 + 0.5f;
    if (fabsf(s->uy) > 1e-4f) {
        float c = s->ax + py * s->ux / s->uy, r = s->reach / fabsf(s->uy);
        lo = fmaxf(lo, c - r); hi = fminf(hi, c + r);
    } else if (fabsf(py * s->ux) > s->reach) {
        return false;
    }
    if (fabsf(s->ux) > 1e-4f) {
        float t0 = (-s->reach - py * s->uy) / s->ux, t1 = (s->len + s->reach - py * s->uy) / s->ux;
        lo = fmaxf(lo, s->ax + fminf(t0, t1)); hi = fminf(hi, s->ax + fmaxf(t0, t1));
    } else {
        float t = py * s->uy;
        if (t < -s->reach || t > s->len + s->reach) return false;
    }
    *xa = imax(s->x0, (int)ceilf(lo - 0.5f));
    *xb = imin(s->x1, (int)floorf(hi - 0.5f));
    return *xa <= *xb;
}

static inline float seg_cov(const seg_t *s, float px, float py)
{
    float dx = px - s->ax, dy = py - s->ay;
    float t = dx * s->ux + dy * s->uy;
    float d = fabsf(dx * s->uy - dy * s->ux);
    if (d > s->reach || t < -s->reach || t > s->len + s->reach) return 0;
    if (s->round) {
        float e = t < 0 ? t : (t > s->len ? t - s->len : 0);
        return band(sqrtf(e * e + d * d), s->hw);
    }
    float along = clamp01(fminf(t + 0.5f, s->len) - fmaxf(t - 0.5f, 0.0f));
    return band(d, s->hw) * along;
}

void gfx_line(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap)
{
    seg_t s;
    if (!seg_setup(&s, x0, y0, x1, y1, w, round_cap)) return;
    gfx_ctx_t *g = cur();
    s.y0 = imax(s.y0, g->ylo);
    s.y1 = imin(s.y1, g->yhi - 1);
    uint16_t *fb = g->fb;
    float a256 = a * 256.0f;
    for (int y = s.y0; y <= s.y1; y++) {
        int xa, xb;
        if (!seg_row(&s, y, &xa, &xb)) continue;
        uint16_t *row = fb + y * GFX_W;
        for (int x = xa; x <= xb; x++) {
            float cov = seg_cov(&s, x + 0.5f, y + 0.5f);
            if (cov > 0) blend256(row + x, c, (int)(cov * a256 + 0.5f));
        }
    }
}

void gfx_line_dashed(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap,
                     float on, float off)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-6f) return;
    dx /= len; dy /= len;
    for (float p = 0; p < len; p += on + off) {
        float q = fminf(p + on, len);
        gfx_line(x0 + dx * p, y0 + dy * p, x0 + dx * q, y0 + dy * q, w, c, a, round_cap);
    }
}

// ---------- polygons ----------
#define POLY_SUB 4
#define POLY_MAX_X 256

static void add_span(float *acc, int base, int n, float xa, float xb, float k)
{
    xa -= base; xb -= base;
    if (xa < 0) xa = 0;
    if (xb > n) xb = n;
    if (xb <= xa) return;
    int ia = (int)xa, ib = (int)xb;
    if (ia == ib) { acc[ia] += (xb - xa) * k; return; }
    acc[ia] += (ia + 1 - xa) * k;
    for (int i = ia + 1; i < ib; i++) acc[i] += k;
    if (ib < n) acc[ib] += (xb - ib) * k;
}

static void fill_poly(const float *xy, int n, gfx_shader_t shader, void *ctx, rgb_t flat, float a,
                      float (*row_alpha)(float y, void *ctx))
{
    gfx_ctx_t *g = cur();
    float *acc = g->acc;
    float xs[POLY_MAX_X];
    int ws[POLY_MAX_X];
    float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
    for (int i = 0; i < n; i++) {
        minx = fminf(minx, xy[2 * i]); maxx = fmaxf(maxx, xy[2 * i]);
        miny = fminf(miny, xy[2 * i + 1]); maxy = fmaxf(maxy, xy[2 * i + 1]);
    }
    int x0 = imax(0, (int)floorf(minx)), x1 = imin(GFX_W - 1, (int)ceilf(maxx));
    int y0 = imax(g->ylo, (int)floorf(miny)), y1 = imin(g->yhi - 1, (int)ceilf(maxy));
    if (x0 > x1 || y0 > y1) return;
    int nx = x1 - x0 + 1;
    float a256 = a * 256.0f;
    for (int y = y0; y <= y1; y++) {
        memset(acc, 0, sizeof(float) * (nx + 1));
        bool any = false;
        for (int s = 0; s < POLY_SUB; s++) {
            float sy = y + (s + 0.5f) / POLY_SUB;
            int m = 0;
            for (int i = 0, j = n - 1; i < n; j = i++) {
                float yi = xy[2 * i + 1], yj = xy[2 * j + 1];
                if ((yi <= sy) == (yj <= sy) || m >= POLY_MAX_X) continue;
                float xi = xy[2 * i], xj = xy[2 * j];
                float x = xj + (sy - yj) * (xi - xj) / (yi - yj);
                int w = yi > yj ? 1 : -1;
                int k = m++;
                while (k > 0 && xs[k - 1] > x) { xs[k] = xs[k - 1]; ws[k] = ws[k - 1]; k--; }
                xs[k] = x; ws[k] = w;
            }
            int wind = 0;
            for (int k = 0; k + 1 < m; k++) {
                wind += ws[k];
                if (wind != 0) { add_span(acc, x0, nx, xs[k], xs[k + 1], 1.0f / POLY_SUB); any = true; }
            }
        }
        if (!any) continue;
        uint16_t *row = g->fb + y * GFX_W;
        float ra = row_alpha ? a256 * row_alpha(y + 0.5f, ctx) : a256;
        for (int i = 0; i < nx; i++) {
            float cov = acc[i];
            if (cov <= 0.002f) continue;
            if (cov > 1) cov = 1;
            if (shader) {
                float pa = 1;
                rgb_t c = shader(x0 + i + 0.5f, y + 0.5f, ctx, &pa);
                blend256(row + x0 + i, c, (int)(cov * pa * ra + 0.5f));
            } else {
                blend256(row + x0 + i, flat, (int)(cov * ra + 0.5f));
            }
        }
    }
}

void gfx_fill_poly(const float *xy, int n, gfx_shader_t shader, void *ctx, rgb_t flat, float a)
{
    fill_poly(xy, n, shader, ctx, flat, a, NULL);
}

void gfx_fill_poly_rows(const float *xy, int n, rgb_t c, float (*row_alpha)(float y, void *ctx), void *ctx)
{
    fill_poly(xy, n, NULL, ctx, c, 1, row_alpha);
}

void gfx_stroke_poly(const float *xy, int n, float w, rgb_t c, float a)
{
    for (int i = 0, j = n - 1; i < n; j = i++)
        gfx_thin_line(xy[2 * j], xy[2 * j + 1], xy[2 * i], xy[2 * i + 1], w, c, a, NULL, NULL, NULL);
}

static float alpha_one(float y, void *ctx) { return 1; }

gfx_mask_t *gfx_mask_build(const float *xy, int n, float (*row_alpha)(float y, void *ctx), void *ctx)
{
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < n; i++) { lo = fminf(lo, xy[2 * i + 1]); hi = fmaxf(hi, xy[2 * i + 1]); }
    int y0 = imax(0, (int)floorf(lo) - 1), y1 = imin(GFX_H, (int)ceilf(hi) + 2);
    if (y1 <= y0) return NULL;
    int h = y1 - y0;
    // Rasterise white on black into a scratch buffer; green (6 bits) is the coverage.
    uint16_t *tmp = heap_caps_calloc(h * GFX_W, 2, MALLOC_CAP_SPIRAM);
    gfx_mask_t *m = heap_caps_calloc(1, sizeof *m, MALLOC_CAP_SPIRAM);
    if (!tmp || !m) { free(tmp); free(m); return NULL; }
    gfx_ctx_t *g = cur();
    uint16_t *keep_fb = g->fb;
    int keep_lo = g->ylo, keep_hi = g->yhi;
    g->fb = tmp - y0 * GFX_W; g->ylo = y0; g->yhi = y1;
    fill_poly(xy, n, NULL, NULL, (rgb_t){255, 255, 255}, 1, alpha_one);
    g->fb = keep_fb; g->ylo = keep_lo; g->yhi = keep_hi;
    m->y0 = y0; m->y1 = y1;
    m->rows = heap_caps_calloc(h, sizeof *m->rows, MALLOC_CAP_SPIRAM);
    uint32_t edge = 0;
    for (int y = 0; y < h; y++) {
        const uint16_t *r = tmp + y * GFX_W;
        int x0 = 0, x1 = GFX_W;
        while (x0 < GFX_W && !((r[x0] >> 5) & 63)) x0++;
        while (x1 > x0 && !((r[x1 - 1] >> 5) & 63)) x1--;
        int i0 = x0, i1 = x1;
        while (i0 < x1 && ((r[i0] >> 5) & 63) != 63) i0++;
        while (i1 > i0 && ((r[i1 - 1] >> 5) & 63) != 63) i1--;
        if (i0 >= i1) i0 = i1 = x1;   // no full pixels: all edge
        for (int x = i0; x < i1; x++)
            if (((r[x] >> 5) & 63) != 63) { i0 = i1 = x1; break; }   // not one interior run
        gfx_mask_row_t *mr = &m->rows[y];
        mr->x0 = x0; mr->i0 = i0; mr->i1 = i1; mr->x1 = x1;
        mr->a = (uint8_t)fminf(255, row_alpha(y0 + y + 0.5f, ctx) * 255 + 0.5f);
        mr->off = edge;
        edge += (i0 - x0) + (x1 - i1);
    }
    m->cov = heap_caps_malloc(edge + 1, MALLOC_CAP_SPIRAM);
    for (int y = 0; y < h; y++) {
        const uint16_t *r = tmp + y * GFX_W;
        const gfx_mask_row_t *mr = &m->rows[y];
        uint8_t *o = m->cov + mr->off;
        for (int x = mr->x0; x < mr->i0; x++) *o++ = (uint8_t)(((r[x] >> 5) & 63) * 255 / 63);
        for (int x = mr->i1; x < mr->x1; x++) *o++ = (uint8_t)(((r[x] >> 5) & 63) * 255 / 63);
    }
    free(tmp);
    return m;
}

void gfx_mask_free(gfx_mask_t *m)
{
    if (!m) return;
    free(m->rows);
    free(m->cov);
    free(m);
}

void gfx_blend_mask(const gfx_mask_t *m, rgb_t c)
{
    gfx_ctx_t *g = cur();
    const int ya = imax(g->ylo, m->y0), yb = imin(g->yhi, m->y1);
    const uint32_t cs = spread565(gfx_pack(c.r, c.g, c.b));
    for (int y = ya; y < yb; y++) {
        const gfx_mask_row_t *mr = &m->rows[y - m->y0];
        uint16_t *row = g->fb + y * GFX_W;
        const uint8_t *cov = m->cov + mr->off;
        const int ra = mr->a;
        for (int x = mr->x0; x < mr->i0; x++) {
            int a = (*cov++ * ra * 32 + dither_a[(y & 3) << 2 | (x & 3)] * 32) >> 16;
            if (a) row[x] = unspread565((cs * a + spread565(row[x]) * (32 - a)) >> 5);
        }
        // Interior at the row's alpha: the colour's share is the same for every pixel.
        const int ai = (ra * 32 + 128) >> 8;
        const uint32_t ca = cs * ai, keep = 32 - ai;
        for (int x = mr->i0; x < mr->i1; x++) row[x] = unspread565((ca + spread565(row[x]) * keep) >> 5);
        for (int x = mr->i1; x < mr->x1; x++) {
            int a = (*cov++ * ra * 32 + dither_a[(y & 3) << 2 | (x & 3)] * 32) >> 16;
            if (a) row[x] = unspread565((cs * a + spread565(row[x]) * (32 - a)) >> 5);
        }
    }
}

// ---------- circles, rectangles ----------
static void radial_shape(float cx, float cy, float ext, rgb_t c, float a,
                         float (*cov_fn)(float d, const float *p), const float *p)
{
    int x0 = imax(0, (int)floorf(cx - ext)), x1 = imin(GFX_W - 1, (int)ceilf(cx + ext));
    gfx_ctx_t *g = cur();
    int y0 = imax(g->ylo, (int)floorf(cy - ext)), y1 = imin(g->yhi - 1, (int)ceilf(cy + ext));
    uint16_t *fb = g->fb;
    float a256 = a * 256.0f;
    for (int y = y0; y <= y1; y++) {
        float dy = y + 0.5f - cy;
        for (int x = x0; x <= x1; x++) {
            float dx = x + 0.5f - cx;
            float cov = cov_fn(sqrtf(dx * dx + dy * dy), p);
            if (cov > 0) blend256(fb + y * GFX_W + x, c, (int)(cov * a256 + 0.5f));
        }
    }
}

static float disc_cov(float d, const float *p) { return clamp01(p[0] + 0.5f - d); }
static float ring_cov(float d, const float *p) { return band(d - p[0], p[1]); }

void gfx_fill_circle(float cx, float cy, float rad, rgb_t c, float a)
{
    float p[1] = {rad};
    radial_shape(cx, cy, rad + 1, c, a, disc_cov, p);
}

void gfx_stroke_circle(float cx, float cy, float rad, float w, rgb_t c, float a)
{
    float p[2] = {rad, w * 0.5f};
    radial_shape(cx, cy, rad + w + 1, c, a, ring_cov, p);
}

void gfx_fill_rect(float x, float y, float w, float h, rgb_t c, float a)
{
    int x0 = imax(0, (int)floorf(x)), x1 = imin(GFX_W - 1, (int)ceilf(x + w) - 1);
    gfx_ctx_t *g = cur();
    int y0 = imax(g->ylo, (int)floorf(y)), y1 = imin(g->yhi - 1, (int)ceilf(y + h) - 1);
    uint16_t *fb = g->fb;
    float a256 = a * 256.0f;
    for (int py = y0; py <= y1; py++) {
        float cy = clamp01(fminf(py + 1.0f, y + h) - fmaxf((float)py, y));
        for (int px = x0; px <= x1; px++) {
            float cx = clamp01(fminf(px + 1.0f, x + w) - fmaxf((float)px, x));
            blend256(fb + py * GFX_W + px, c, (int)(cx * cy * a256 + 0.5f));
        }
    }
}

void gfx_dim(float a)
{
    gfx_ctx_t *g = cur();
    uint16_t *p = g->fb;
    // Spread channels (a free gap above each), so the multiply cannot carry between them.
    const uint32_t k = (uint32_t)((1 - a) * 32 + 0.5f);
    for (int i = g->ylo * GFX_W; i < g->yhi * GFX_W; i++) p[i] = unspread565(spread565(p[i]) * k >> 5);
}

// ---------- text ----------
static int next_cp(const char **s)
{
    const unsigned char *p = (const unsigned char *)*s;
    int cp = *p++;
    if (cp >= 0xC0 && (*p & 0xC0) == 0x80) cp = ((cp & 0x1F) << 6) | (*p++ & 0x3F);
    *s = (const char *)p;
    return cp;
}

static const gfx_glyph_t *find_glyph(const gfx_font_t *f, int cp)
{
    for (int i = 0; i < f->n; i++)
        if (f->glyphs[i].cp == cp) return &f->glyphs[i];
    return NULL;
}

float gfx_text_width(const gfx_font_t *f, const char *utf8, float spacing)
{
    float w = 0;
    while (*utf8) {
        const gfx_glyph_t *g = find_glyph(f, next_cp(&utf8));
        if (g) w += g->adv + spacing;
    }
    return w;
}

static void blit_glyph(uint16_t *fb, const uint8_t *bits, int w, int h, int x0, int y0, rgb_t c, float a)
{
    int k = (int)(a * 256.0f + 0.5f);
    const gfx_ctx_t *g = cur();
    for (int j = 0; j < h; j++) {
        int y = y0 + j;
        if (y < g->ylo || y >= g->yhi) continue;
        for (int i = 0; i < w; i++) {
            int x = x0 + i, v = bits[j * w + i];
            if (v && (unsigned)x < GFX_W) blend256(fb + y * GFX_W + x, c, ((v + (v >> 7)) * k) >> 8);
        }
    }
}

float gfx_text(const gfx_font_t *f, const char *utf8, float x, float y, gfx_align_t align, rgb_t c, float a,
               float spacing)
{
    float width = gfx_text_width(f, utf8, spacing);
    if (align == GFX_CENTER) x -= width * 0.5f;
    int baseline = (int)lroundf(y + f->middle);
    uint16_t *fb = cur()->fb;
    for (int pass = f->outlined ? 0 : 1; pass < 2; pass++) {
        const char *s = utf8;
        float px = x;
        while (*s) {
            const gfx_glyph_t *g = find_glyph(f, next_cp(&s));
            if (!g) continue;
            int gx = (int)lroundf(px);
            if (pass == 0) blit_glyph(fb, f->bits + g->o_off, g->o_w, g->o_h, gx + g->o_ox, baseline + g->o_oy,
                                      RGB(0, 0, 0), 0.7f * a);
            else blit_glyph(fb, f->bits + g->off, g->w, g->h, gx + g->ox, baseline + g->oy, c, a);
            px += g->adv + spacing;
        }
    }
    return width;
}
