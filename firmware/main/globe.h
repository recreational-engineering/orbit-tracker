/* The view's pixels: renderGlobePixels() of web/app.js (inverse orthographic, azimuthal-
 * equidistant or pilot ray-cast lookup of terrain, coastline, night side and city lights),
 * the instrument face behind the globe and the sphere lighting over it, rendered strip by
 * strip on whichever core asks.
 *
 * Paths, fastest first:
 *  - globe seen from the equatorial plane (Earth spin) and the flat map: full resolution
 *    from per-pixel tables, since the view only turns about the polar axis;
 *  - any other globe orientation: exact every few pixels, interpolated in between, at full
 *    resolution (background cache render) or half resolution (while the view moves);
 *  - pilot view: ray cast at half resolution. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "gfx.h"

#define GLOBE_FRAC 0.43f
#define GLOBE_R (GFX_W * GLOBE_FRAC)
#define GLOBE_CX (GFX_W / 2.0f)
#define GLOBE_CY (GFX_H / 2.0f)
#define PILOT_HALF_FOV_DEG 55.0

typedef enum { GLOBE_ORTHO, GLOBE_FLAT, GLOBE_PILOT } globe_kind_t;

// Pilot camera (updateCamera()): position c (ECI km), forward f, up u, right rt.
typedef struct { double c[3]; float f[3], u[3], rt[3]; double gmst; } pilot_cam_t;

typedef struct {
    globe_kind_t kind;
    float rot[3];       // d3 rotation [λ, φ, γ] in degrees (ortho, flat)
    float sun[3];       // unit vector toward the sun, Earth-fixed
    pilot_cam_t cam;    // pilot
} globe_view_t;

void globe_init(void);

// True when this orthographic view only turns about the polar axis (full-res table path).
bool globe_is_equatorial(const globe_view_t *v);

// Rows y0 <= y < y1 (even) of the base layer into fbv, a frame-buffer pointer that only has to
// be valid for those rows (a strip buffer offset by -y0 rows). half: 2x2 blocks.
void globe_rows(uint16_t *fbv, const globe_view_t *v, int y0, int y1, bool half);

// Run fn(0, ctx) here and fn(1, ctx) on core 1 at frame priority; returns when both are done.
void globe_both_cores(void (*fn)(int part, const void *ctx), const void *ctx);

// Background full-resolution cache for still orthographic views.
void globe_cache_request(const globe_view_t *v, double t);   // ignored while one is running
bool globe_cache_busy(void);
void globe_cache_cancel(void);
// The newest finished cache, locked against swapping until globe_cache_release(); NULL if none.
const uint16_t *globe_cache_hold(globe_view_t *v, double *t);
void globe_cache_release(void);
void globe_cache_stats(int *count, float *avg_ms);
// Called by the background task on each finished cache frame before it is shown (lines
// printed on the globe), with the view it was rendered for.
void globe_cache_bake(void (*fn)(uint16_t *fb, const globe_view_t *v));

// Stroke colour lit like the surface under pixel (x, y) (sphere lighting or the flat map's
// vignette), and its coverage by the disc in *alpha (0 outside).
rgb_t globe_shaded_stroke(int x, int y, rgb_t c, bool flat, float *alpha);
