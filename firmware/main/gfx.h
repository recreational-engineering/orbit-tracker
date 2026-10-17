/* Minimal anti-aliased raster primitives into an RGB565 frame buffer.
 *
 * Just enough of Canvas 2D for web/app.js: thin strokes (dashes, per-pixel colour),
 * thick round-capped strokes, nonzero polygon fills with a per-pixel shader,
 * circles, rectangles and bitmap text. Alpha is 0..1 like canvas globalAlpha
 * times the colour's alpha. Each CPU core has its own target, so two tasks on
 * different cores can draw at the same time.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define GFX_W 720
#define GFX_H 720

typedef struct { uint8_t r, g, b; } rgb_t;
#define RGB(r, g, b) ((rgb_t){(r), (g), (b)})

// Per-pixel fill colour; *alpha comes in as 1 and may be lowered.
typedef rgb_t (*gfx_shader_t)(float x, float y, void *ctx, float *alpha);
// Per-pixel stroke colour (e.g. lit like the surface below it); *alpha comes in as 1
// and may be lowered (0 clips the pixel).
typedef rgb_t (*gfx_pixel_color_t)(int x, int y, rgb_t c, void *ctx, float *alpha);

static inline uint16_t gfx_pack(int r, int g, int b)
{
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static inline void gfx_unpack(uint16_t p, int *r, int *g, int *b)
{
    int r5 = (p >> 11) & 31, g6 = (p >> 5) & 63, b5 = p & 31;
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}

void gfx_init(void);
void gfx_target(uint16_t *fb);
// Only rows y0 <= y < y1 are drawn (per core; lets both cores share one frame).
void gfx_clip_rows(int y0, int y1);
void gfx_get_clip_rows(int *y0, int *y1);
uint16_t *gfx_get_target(void);

void gfx_blend(int x, int y, rgb_t c, float a);

// Thin stroke (width up to ~2 px), one pass along the major axis. Half open, so the
// segments of a polyline do not blend twice at shared vertices. dash may be NULL.
typedef struct { float on, off, pos; } gfx_dash_t;
void gfx_thin_line(float x0, float y0, float x1, float y1, float w, rgb_t c, float a,
                   gfx_dash_t *dash, gfx_pixel_color_t color, void *ctx);
// The same for a display list: alpha 0..256, dashes as on/off lengths (on = 0: solid) and the
// phase at the start; the caller has already checked the segment reaches this core's rows.
void gfx_thin_seg(float x0, float y0, float x1, float y1, float w, rgb_t c, int a256,
                  float on, float off, float phase);

// Thick stroke with exact coverage (butt or round caps), blended on its own like
// one canvas stroke() per segment.
void gfx_line(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap);
void gfx_line_dashed(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap,
                     float on, float off);

// Nonzero fill of a closed polygon (xy pairs). shader may be NULL for a flat colour.
void gfx_fill_poly(const float *xy, int n, gfx_shader_t shader, void *ctx, rgb_t flat, float a);
// Flat colour whose alpha depends only on the row (vertical gradients).
void gfx_fill_poly_rows(const float *xy, int n, rgb_t c, float (*row_alpha)(float y, void *ctx), void *ctx);
// Outline of a closed polygon with thin strokes.
// A polygon fill rasterised once (coverage per pixel, alpha per row) and blended cheaply on
// every frame after: interior runs at the row's alpha, the anti-aliased edges per pixel.
typedef struct { int16_t x0, i0, i1, x1; uint8_t a; uint8_t pad[3]; uint32_t off; } gfx_mask_row_t;
typedef struct { int y0, y1; gfx_mask_row_t *rows; uint8_t *cov; } gfx_mask_t;
gfx_mask_t *gfx_mask_build(const float *xy, int n, float (*row_alpha)(float y, void *ctx), void *ctx);
void gfx_mask_free(gfx_mask_t *m);
void gfx_blend_mask(const gfx_mask_t *m, rgb_t c);

void gfx_stroke_poly(const float *xy, int n, float w, rgb_t c, float a);

void gfx_fill_circle(float cx, float cy, float rad, rgb_t c, float a);
void gfx_stroke_circle(float cx, float cy, float rad, float w, rgb_t c, float a);
void gfx_fill_rect(float x, float y, float w, float h, rgb_t c, float a);
// Darken every pixel by (1 - a) toward black (a dim layer over the whole screen).
void gfx_dim(float a);

// Bitmap fonts (generated/fonts.h).
typedef struct {
    uint16_t cp;
    float adv;
    uint32_t off; uint8_t w, h; int8_t ox, oy;        // glyph coverage
    uint32_t o_off; uint8_t o_w, o_h; int8_t o_ox, o_oy;   // with a 2 px outline (outlined fonts)
} gfx_glyph_t;
typedef struct {
    const gfx_glyph_t *glyphs;
    int n;
    float middle;    // textBaseline 'middle' sits this far above the alphabetic baseline
    bool outlined;
    const uint8_t *bits;
} gfx_font_t;

typedef enum { GFX_LEFT, GFX_CENTER } gfx_align_t;
float gfx_text_width(const gfx_font_t *f, const char *utf8, float spacing);
// Text with textBaseline 'middle' at y; returns its width. Outlined fonts get a dark rim.
float gfx_text(const gfx_font_t *f, const char *utf8, float x, float y, gfx_align_t align, rgb_t c, float a,
               float spacing);
