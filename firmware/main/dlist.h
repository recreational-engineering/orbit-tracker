/* Per-frame display list. The scene is recorded once per frame (both cores record, each into
 * its own list), binned by the frame strips each item touches, and replayed strip by strip
 * into internal RAM. Items replay in recording order: list 0 first, then list 1.
 * The calls mirror gfx.h. */
#pragma once
#include <stdbool.h>
#include "gfx.h"

// 20-row strips keep the per-core internal-RAM scratch small (two buffers of 20*720*2 B), so the
// renderer coexists with the Wi-Fi/ESP-Hosted stack, which reserves internal RAM at start-up.
#define DL_STRIP_H 20
#define DL_NSTRIPS ((GFX_H + DL_STRIP_H - 1) / DL_STRIP_H)

void dl_init(void);
// Start a frame (both lists empty).
void dl_begin(void);
// Bin both lists by strip; call once after both cores have recorded.
void dl_finish(void);
// Replay strip k into the current gfx target (gfx_clip_rows already set to the strip).
void dl_replay(int strip);
int dl_count(void);

void dl_thin(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, const gfx_dash_t *dash);
// Hairlines of the surface line sets (one core, before anything else is recorded): a set of
// shared stroke parameters (dashes on / off, on = 0: solid), then its segments with the dash
// position at their start. They are drawn before every other item.
int dl_hair_set(float w, float a, float on, float off);
void dl_hair(int set, float x0, float y0, float x1, float y1, rgb_t c, float dash_pos);
void dl_line(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap);
void dl_line_dashed(float x0, float y0, float x1, float y1, float w, rgb_t c, float a, bool round_cap, float on, float off);
void dl_fill_circle(float cx, float cy, float rad, rgb_t c, float a);
void dl_stroke_circle(float cx, float cy, float rad, float w, rgb_t c, float a);
void dl_fill_rect(float x, float y, float w, float h, rgb_t c, float a);
float dl_text(const gfx_font_t *f, const char *s, float x, float y, gfx_align_t align, rgb_t c, float a, float spacing);
// Polygons: points are copied; the shader context (ctx_size bytes) is copied too.
void dl_fill_poly(const float *xy, int n, gfx_shader_t shader, const void *ctx, int ctx_size, rgb_t flat, float a);
void dl_fill_poly_rows(const float *xy, int n, rgb_t c, float (*row_alpha)(float y, void *ctx), const void *ctx, int ctx_size);
void dl_stroke_poly(const float *xy, int n, float w, rgb_t c, float a);
// A cached mask (must stay valid until the frame is drawn).
void dl_mask(const gfx_mask_t *m, rgb_t c);
// Darken the whole screen (menu backdrop).
void dl_dim(float a);
