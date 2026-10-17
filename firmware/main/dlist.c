#include <math.h>
#include <string.h>
#include "dlist.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"

typedef enum { K_THIN, K_LINE, K_DLINE, K_FCIRC, K_SCIRC, K_RECT, K_TEXT, K_POLY, K_POLY_ROWS, K_SPOLY, K_DIM, K_TRUN, K_MASK } kind_t;

typedef struct {
    uint8_t kind, flag;     // flag: round cap / text align / dash present
    rgb_t c;
    float a, w;
    float x0, y0, x1, y1;   // geometry (line ends, circle centre + radius, rect, text anchor)
    float p0, p1, p2;       // dash on, off, phase / spacing
    int16_t ylo, yhi;       // rows touched (inclusive)
    uint16_t n;             // polygon points
    uint32_t data;          // arena offset: points, string, shader context
    const void *fn;         // shader / row alpha / font
} item_t;

#define ITEM_CAP 12000
#define ARENA_CAP (256 * 1024)
#define BIN_CAP (3 * 2 * ITEM_CAP)

typedef struct {
    item_t *items;
    int n;
    uint8_t *arena;
    uint32_t used;
} list_t;

static list_t lists[2];
static uint32_t *bin_idx;   // list << 31 | index
static int bin_start[DL_NSTRIPS + 1];

static inline list_t *cur(void) { return &lists[esp_cpu_get_core_id()]; }

void dl_init(void)
{
    for (int i = 0; i < 2; i++) {
        lists[i].items = heap_caps_malloc(ITEM_CAP * sizeof(item_t), MALLOC_CAP_SPIRAM);
        lists[i].arena = heap_caps_malloc(ARENA_CAP, MALLOC_CAP_SPIRAM);
    }
    bin_idx = heap_caps_malloc(BIN_CAP * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
}

void dl_begin(void)
{
    for (int i = 0; i < 2; i++) { lists[i].n = 0; lists[i].used = 0; }
}

int dl_count(void) { return lists[0].n + lists[1].n; }

static item_t *add(kind_t k, float ylo, float yhi)
{
    list_t *l = cur();
    if (l->n >= ITEM_CAP || yhi < 0 || ylo >= GFX_H) return NULL;
    item_t *it = &l->items[l->n++];
    memset(it, 0, sizeof *it);
    it->kind = k;
    it->ylo = (int16_t)fmaxf(0, floorf(ylo));
    it->yhi = (int16_t)fminf(GFX_H - 1, ceilf(yhi));
    return it;
}

static void *arena(uint32_t n, uint32_t *off)
{
    list_t *l = cur();
    n = (n + 3) & ~3u;
    if (l->used + n > ARENA_CAP) return NULL;
    *off = l->used;
    l->used += n;
    return l->arena + *off;
}

// Thin segments in a row with the same width, alpha and dash pattern go into one run item
// of compact segments (coordinates in 1/16 px), as long as the run stays within a few strips.
typedef struct { int16_t x0, y0, x1, y1, ylo, yhi; rgb_t c; uint8_t pad; uint16_t phase, pad2; } tseg_t;

static inline int strip_span(int ylo, int yhi) { return yhi / DL_STRIP_H - ylo / DL_STRIP_H + 1; }

static inline int16_t q4(float v) { return (int16_t)((int)(v * 16 + 32768.5f) - 32768); }

static bool thin_run(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, const gfx_dash_t *dash)
{
    if (fmaxf(fmaxf(fabsf(x0), fabsf(x1)), fmaxf(fabsf(y0), fabsf(y1))) > 2000) return false;
    float m = w * 0.5f + 2;
    float flo = fminf(y0, y1) - m, fhi = fmaxf(y0, y1) + m;
    if (fhi < 0 || flo >= GFX_H) return true;
    int ylo = (int)(flo + 4096.0f) - 4096, yhi = 4096 - (int)(4096.0f - fhi);   // floor, ceil
    if (ylo < 0) ylo = 0;
    if (yhi > GFX_H - 1) yhi = GFX_H - 1;
    float on = dash ? dash->on : 0, off = dash ? dash->off : 0;
    list_t *l = cur();
    item_t *it = l->n ? &l->items[l->n - 1] : NULL;
    bool append = it && it->kind == K_TRUN && it->n < 65000 && it->w == w && it->a == a && it->p0 == on && it->p1 == off &&
                  l->used == it->data + it->n * sizeof(tseg_t) && l->used + sizeof(tseg_t) <= ARENA_CAP;
    if (append) {
        int lo = it->ylo < ylo ? it->ylo : ylo, hi = it->yhi > yhi ? it->yhi : yhi;
        int span = strip_span(lo, hi), keep = strip_span(it->ylo, it->yhi), own = strip_span(ylo, yhi);
        append = span <= (keep > own ? keep : own) || span <= 2;
        if (append) { it->ylo = lo; it->yhi = hi; }
    }
    if (!append) {
        if (l->n >= ITEM_CAP || l->used + sizeof(tseg_t) > ARENA_CAP) return true;
        it = &l->items[l->n++];
        memset(it, 0, sizeof *it);
        it->kind = K_TRUN;
        it->ylo = ylo; it->yhi = yhi;
        it->w = w; it->a = a; it->p0 = on; it->p1 = off;
        it->data = l->used;
    }
    tseg_t *sg = (tseg_t *)(l->arena + l->used);
    l->used += sizeof(tseg_t);
    it->n++;
    sg->x0 = q4(x0); sg->y0 = q4(y0); sg->x1 = q4(x1); sg->y1 = q4(y1);
    sg->ylo = ylo; sg->yhi = yhi; sg->c = c;
    sg->phase = 0;
    if (dash) {
        float pos = fmaxf(dash->pos, 0), period = on + off;
        sg->phase = (uint16_t)((pos - period * (int)(pos / period)) * 16);
    }
    return true;
}

void dl_thin(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, const gfx_dash_t *dash)
{
    if (thin_run(x0, y0, x1, y1, w, c, a, dash)) return;
    float m = w * 0.5f + 2;
    item_t *it = add(K_THIN, fminf(y0, y1) - m, fmaxf(y0, y1) + m);
    if (!it) return;
    it->c = c; it->a = a; it->w = w;
    it->x0 = x0; it->y0 = y0; it->x1 = x1; it->y1 = y1;
    if (dash) { it->flag = 1; it->p0 = dash->on; it->p1 = dash->off; it->p2 = dash->pos; }
}

void dl_line(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap)
{
    float m = w * 0.5f + 2;
    item_t *it = add(K_LINE, fminf(y0, y1) - m, fmaxf(y0, y1) + m);
    if (!it) return;
    it->c = c; it->a = a; it->w = w; it->flag = round_cap;
    it->x0 = x0; it->y0 = y0; it->x1 = x1; it->y1 = y1;
}

void dl_line_dashed(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap, float on, float off)
{
    float m = w * 0.5f + 2;
    item_t *it = add(K_DLINE, fminf(y0, y1) - m, fmaxf(y0, y1) + m);
    if (!it) return;
    it->c = c; it->a = a; it->w = w; it->flag = round_cap;
    it->x0 = x0; it->y0 = y0; it->x1 = x1; it->y1 = y1; it->p0 = on; it->p1 = off;
}

void dl_fill_circle(float cx, float cy, float rad, rgb_t c, float a)
{
    item_t *it = add(K_FCIRC, cy - rad - 2, cy + rad + 2);
    if (!it) return;
    it->c = c; it->a = a; it->x0 = cx; it->y0 = cy; it->x1 = rad;
}

void dl_stroke_circle(float cx, float cy, float rad, float w, rgb_t c, float a)
{
    item_t *it = add(K_SCIRC, cy - rad - w - 2, cy + rad + w + 2);
    if (!it) return;
    it->c = c; it->a = a; it->w = w; it->x0 = cx; it->y0 = cy; it->x1 = rad;
}

void dl_fill_rect(float x, float y, float w, float h, rgb_t c, float a)
{
    item_t *it = add(K_RECT, y - 1, y + h + 1);
    if (!it) return;
    it->c = c; it->a = a; it->x0 = x; it->y0 = y; it->x1 = w; it->y1 = h;
}

float dl_text(const gfx_font_t *f, const char *s, float x, float y, gfx_align_t align, rgb_t c, float a, float spacing)
{
    float width = gfx_text_width(f, s, spacing);
    float h = f->middle * 3 + 4;
    item_t *it = add(K_TEXT, y - h, y + h);
    if (!it) return width;
    uint32_t off;
    size_t len = strlen(s) + 1;
    char *dst = arena(len, &off);
    if (!dst) { cur()->n--; return width; }
    memcpy(dst, s, len);
    it->c = c; it->a = a; it->flag = align; it->x0 = x; it->y0 = y; it->p2 = spacing;
    it->data = off; it->fn = f;
    return width;
}

static item_t *poly_item(kind_t k, const float *xy, int n, uint32_t extra)
{
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < n; i++) { lo = fminf(lo, xy[2 * i + 1]); hi = fmaxf(hi, xy[2 * i + 1]); }
    item_t *it = add(k, lo - 2, hi + 2);
    if (!it) return NULL;
    uint32_t off;
    float *dst = arena(n * 8 + extra, &off);
    if (!dst) { cur()->n--; return NULL; }
    memcpy(dst, xy, n * 8);
    it->n = n;
    it->data = off;
    return it;
}

void dl_fill_poly(const float *xy, int n, gfx_shader_t shader, const void *ctx, int ctx_size, rgb_t flat, float a)
{
    item_t *it = poly_item(K_POLY, xy, n, ctx_size);
    if (!it) return;
    it->c = flat; it->a = a; it->fn = shader; it->p2 = ctx_size;
    if (ctx_size) memcpy(cur()->arena + it->data + n * 8, ctx, ctx_size);
}

void dl_fill_poly_rows(const float *xy, int n, rgb_t c, float (*row_alpha)(float y, void *ctx), const void *ctx, int ctx_size)
{
    item_t *it = poly_item(K_POLY_ROWS, xy, n, ctx_size);
    if (!it) return;
    it->c = c; it->a = 1; it->fn = row_alpha;
    if (ctx_size) memcpy(cur()->arena + it->data + n * 8, ctx, ctx_size);
}

void dl_stroke_poly(const float *xy, int n, float w, rgb_t c, float a)
{
    item_t *it = poly_item(K_SPOLY, xy, n, 0);
    if (!it) return;
    it->c = c; it->a = a; it->w = w;
}

void dl_mask(const gfx_mask_t *m, rgb_t c)
{
    if (!m) return;
    item_t *it = add(K_MASK, m->y0, m->y1 - 1);
    if (!it) return;
    it->c = c; it->fn = m;
}

void dl_dim(float a)
{
    item_t *it = add(K_DIM, 0, GFX_H - 1);
    if (!it) return;
    it->a = a;
}

void dl_finish(void)
{
    int count[DL_NSTRIPS] = {0};
    for (int l = 0; l < 2; l++)
        for (int i = 0; i < lists[l].n; i++) {
            const item_t *it = &lists[l].items[i];
            for (int k = it->ylo / DL_STRIP_H; k <= it->yhi / DL_STRIP_H; k++) count[k]++;
        }
    bin_start[0] = 0;
    for (int k = 0; k < DL_NSTRIPS; k++) bin_start[k + 1] = bin_start[k] + count[k];
    if (bin_start[DL_NSTRIPS] > BIN_CAP) { memset(bin_start, 0, sizeof bin_start); return; }
    int fill[DL_NSTRIPS];
    memcpy(fill, bin_start, sizeof fill);
    for (int l = 0; l < 2; l++)
        for (int i = 0; i < lists[l].n; i++) {
            const item_t *it = &lists[l].items[i];
            for (int k = it->ylo / DL_STRIP_H; k <= it->yhi / DL_STRIP_H; k++) bin_idx[fill[k]++] = (uint32_t)l << 31 | i;
        }
}

void dl_replay(int strip)
{
    for (int b = bin_start[strip]; b < bin_start[strip + 1]; b++) {
        uint32_t ref = bin_idx[b];
        const list_t *l = &lists[ref >> 31];
        const item_t *it = &l->items[ref & 0x7FFFFFFF];
        const uint8_t *data = l->arena + it->data;
        switch (it->kind) {
        case K_THIN: {
            gfx_dash_t d = {it->p0, it->p1, it->p2};
            gfx_thin_line(it->x0, it->y0, it->x1, it->y1, it->w, it->c, it->a, it->flag ? &d : NULL, NULL, NULL);
            break;
        }
        case K_LINE: gfx_line(it->x0, it->y0, it->x1, it->y1, it->w, it->c, it->a, it->flag); break;
        case K_DLINE: gfx_line_dashed(it->x0, it->y0, it->x1, it->y1, it->w, it->c, it->a, it->flag, it->p0, it->p1); break;
        case K_FCIRC: gfx_fill_circle(it->x0, it->y0, it->x1, it->c, it->a); break;
        case K_SCIRC: gfx_stroke_circle(it->x0, it->y0, it->x1, it->w, it->c, it->a); break;
        case K_RECT: gfx_fill_rect(it->x0, it->y0, it->x1, it->y1, it->c, it->a); break;
        case K_TEXT: gfx_text(it->fn, (const char *)data, it->x0, it->y0, it->flag, it->c, it->a, it->p2); break;
        case K_POLY:
            gfx_fill_poly((const float *)data, it->n, (gfx_shader_t)it->fn, it->p2 > 0 ? (void *)(data + it->n * 8) : NULL,
                          it->c, it->a);
            break;
        case K_POLY_ROWS:
            gfx_fill_poly_rows((const float *)data, it->n, it->c, (float (*)(float, void *))it->fn,
                               (void *)(data + it->n * 8));
            break;
        case K_SPOLY: gfx_stroke_poly((const float *)data, it->n, it->w, it->c, it->a); break;
        case K_DIM: gfx_dim(it->a); break;
        case K_MASK: gfx_blend_mask((const gfx_mask_t *)it->fn, it->c); break;
        case K_TRUN: {
            int ylo, yhi;
            gfx_get_clip_rows(&ylo, &yhi);
            const tseg_t *sg = (const tseg_t *)data;
            int a256 = (int)(it->a * 256.0f + 0.5f);
            for (int i = 0; i < it->n; i++, sg++) {
                if (sg->yhi < ylo || sg->ylo >= yhi) continue;
                gfx_thin_seg(sg->x0 * (1 / 16.0f), sg->y0 * (1 / 16.0f), sg->x1 * (1 / 16.0f), sg->y1 * (1 / 16.0f),
                             it->w, sg->c, a256, it->p0, it->p1, sg->phase * (1 / 16.0f));
            }
            break;
        }
        }
    }
}
