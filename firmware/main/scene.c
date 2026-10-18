/* Round-display GUI: a port of draw(), drawPilot(), the five views, the HUD, the on-screen
 * menu and the pointer handling of web/app.js.
 *
 * Per frame: the scene is recorded once into a display list (dlist.c, both cores), then
 * the frame is built in strips of internal RAM on both cores: the view's pixels (globe.c),
 * then the recorded items, then the strip is copied to the frame buffer. Lines printed on
 * the globe (borders, graticule, compass rose, the grey hidden orbit) take the sphere
 * lighting of the surface under them, which equals drawing them under the glass shading.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "dlist.h"
#include "esp_memory_utils.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "gfx.h"
#include "globe.h"
#include "model.h"
#include "scene.h"
#include "tiles.h"
#include "generated/fonts.h"

#define S GFX_W
static const float R = GLOBE_R, CX = GLOBE_CX, CY = GLOBE_CY;

static const rgb_t TRACK = {0xc8, 0x10, 0x2e};       // orbit line and its ticks
static const rgb_t BEHIND = {0xc4, 0xc8, 0xcc};      // hidden line, ticks and object
static const rgb_t AMBER = {0xff, 0xc8, 0x00};       // compass rose / horizon circle
static const rgb_t HUD_GREEN = {0x4d, 0xff, 0x7a};   // pilot HUD
static const rgb_t WHITE = {255, 255, 255};
static const rgb_t BLACK = {0, 0, 0};
#define BEHIND_ALPHA 0.28f
#define SPIN_PERIOD_S 60.0
#define GLIDE_TAU_S 0.15f
#define TAP_SLOP_PX 6
#define BAND_ANGLE (58.0f * (float)M_PI / 180)
#define PILOT_NEAR_KM 5.0f
#define PILOT_SUBDIV 6

typedef enum { MODE_SPIN, MODE_HOME, MODE_ORBIT, MODE_PILOT, MODE_FLAT, MODE_COUNT } view_mode_t;
static const char *MODE_NAMES[] = {"spin", "ground", "satellite", "pilot", "flat"};

static float track_alpha(double t) { return t >= 0 ? 1.0f : (float)fmax(0, 1 + t / TRACK_SPAN_S); }

static struct {
    view_mode_t mode;
    float rotate[3];            // d3 rotation [λ, φ, γ] (pilot: look-around yaw/pitch offsets)
    bool aimed;                 // rotate has been set from the first position
    int64_t spin_t0;            // Earth spin: time the spin started (ms)
    int64_t now_ms;
    // pointer
    bool down, moved;
    int x0, y0, lx, ly;
    int64_t move_ms, last_tap_ms;
    int tap_x, tap_y;
    float vel[2];               // pan inertia (deg / s)
    int64_t manual_until;       // after a pan, hold the user's view until this time (ms)
    bool gliding;               // easing toward the mode's view, not locked on yet
    // model
    double t;
    eci_t now, ahead;
    // HUD / menu hit areas from the last draw
    bool menu_open;
    struct { float x, y, w, h; view_mode_t mode; } menu_hits[MODE_COUNT];
    float top_cut, top_half, bottom_cut, bottom_half;
    bool have_bottom;
} st = {.mode = MODE_SPIN, .rotate = {-10, -45, 0}};

// ---------- view math ----------
typedef struct { float m[9]; } mat3_t;

typedef struct {
    double c[3];                          // camera position (ECI km)
    float f[3], u[3], rt[3], up[3], hf[3];
    double gmst;
    bool ok;
} cam_t;

typedef struct {
    globe_kind_t kind;
    float rot[3];
    mat3_t M;
    cam_t cam;
} view_t;

static cam_t cam;   // pilot camera of the current frame

static void rotate_vec(const float rot[3], const double in[3], double out[3])
{
    double dl = rot[0] / DEG, dp = rot[1] / DEG, dg = rot[2] / DEG;
    double x1 = in[0] * cos(dl) - in[1] * sin(dl), y1 = in[0] * sin(dl) + in[1] * cos(dl), z1 = in[2];
    double k = z1 * cos(dp) + x1 * sin(dp);
    out[0] = y1 * cos(dg) - k * sin(dg);
    out[1] = k * cos(dg) + y1 * sin(dg);
    out[2] = x1 * cos(dp) - z1 * sin(dp);
}

// d3.geoRotation([λ, φ, γ]) + orthographic projection as a matrix from Earth-fixed unit
// vectors (x: 0°E, y: 90°E, z: north) to view space (x right, y up, z toward the viewer).
static mat3_t view_matrix(const float rot[3])
{
    mat3_t M;
    for (int j = 0; j < 3; j++) {
        double e[3] = {j == 0, j == 1, j == 2}, o[3];
        rotate_vec(rot, e, o);
        for (int i = 0; i < 3; i++) M.m[i * 3 + j] = (float)o[i];
    }
    return M;
}

static inline void mul3(const mat3_t *M, float x, float y, float z, float out[3])
{
    out[0] = M->m[0] * x + M->m[1] * y + M->m[2] * z;
    out[1] = M->m[3] * x + M->m[4] * y + M->m[5] * z;
    out[2] = M->m[6] * x + M->m[7] * y + M->m[8] * z;
}

static float wrap180(float d) { return fmodf(fmodf(d + 540.0f, 360.0f) + 360.0f, 360.0f) - 180.0f; }
static inline float dot3f(const float *a, const float *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static void norm3f(float *a)
{
    float n = sqrtf(dot3f(a, a));
    if (n > 0) { a[0] /= n; a[1] /= n; a[2] /= n; }
}
static void cross3f(const float *a, const float *b, float *o)
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

static globe_kind_t mode_kind(view_mode_t m)
{
    return m == MODE_FLAT ? GLOBE_FLAT : m == MODE_PILOT ? GLOBE_PILOT : GLOBE_ORTHO;
}

// Flat map (azimuthal equidistant around the north pole, Greenwich down): screen point.
// atan2 for y >= 0, |error| about 1e-5 rad.
static inline float atan2_pos(float y, float x)
{
    float ax = fabsf(x), mn = fminf(ax, y), mx = fmaxf(ax, y);
    if (mx <= 0) return 0;
    float t = mn / mx, t2 = t * t;
    float r = t * (0.99997726f + t2 * (-0.33262347f + t2 * (0.19354346f + t2 * (-0.11643287f +
              t2 * (0.05265332f + t2 * -0.01172120f)))));
    if (y > ax) r = (float)(M_PI / 2) - r;
    return x < 0 ? (float)M_PI - r : r;
}

// Azimuthal equidistant about the north pole, turned by rot0 (deg): no libm trig per point.
static void flat_xy(float rot0, float ux, float uy, float uz, float *x, float *y)
{
    static float last[2] = {NAN, NAN}, cs[2][2];
    const int core = esp_cpu_get_core_id();
    if (rot0 != last[core]) {
        cs[core][0] = cosf(rot0 / (float)DEG); cs[core][1] = sinf(rot0 / (float)DEG);
        last[core] = rot0;
    }
    const float cr = cs[core][0], sr = cs[core][1];
    float h2 = ux * ux + uy * uy, h = sqrtf(h2);
    float rho = atan2_pos(h, uz) * (float)(1 / M_PI);   // colatitude / 180°
    float k = h > 1e-9f ? R * rho / h : 0;
    *x = CX + k * (uy * cr + ux * sr);
    *y = CY + k * (ux * cr - uy * sr);
}

typedef struct { float x, y, v[3]; bool hidden, front, in_view; } proj_t;

static const float PILOT_K = (S / 2.0f) / 1.42814801f;   // (s / 2) / tan(55°)

// projectPilot(): perspective from the camera; u is Earth-fixed in the frame of the camera's
// sidereal angle (so ECI = u turned by +gmst).
static proj_t project_pilot(const view_t *v, const double u[3], double rk)
{
    proj_t p = {0};
    const cam_t *c = &v->cam;
    double g = c->gmst, cg = cos(g), sg = sin(g);
    double P[3] = {rk * (u[0] * cg - u[1] * sg), rk * (u[0] * sg + u[1] * cg), rk * u[2]};
    float rel[3] = {(float)(P[0] - c->c[0]), (float)(P[1] - c->c[1]), (float)(P[2] - c->c[2])};
    float z = dot3f(rel, c->f);
    if (z <= 1e-6f) { p.hidden = true; return p; }
    p.x = S / 2.0f + PILOT_K * dot3f(rel, c->rt) / z;
    p.y = S / 2.0f - PILOT_K * dot3f(rel, c->u) / z;
    // Hidden if the line of sight meets the Earth before reaching the point.
    float dist = sqrtf(dot3f(rel, rel)), d[3] = {rel[0] / dist, rel[1] / dist, rel[2] / dist};
    float cc[3] = {(float)c->c[0], (float)c->c[1], (float)c->c[2]};
    float b = dot3f(cc, d), disc = b * b - (dot3f(cc, cc) - (float)(EARTH_R_KM * EARTH_R_KM));
    float t1 = disc >= 0 ? -b - sqrtf(disc) : -1;
    p.hidden = t1 > 0 && t1 < dist - 1e-3f;
    p.front = !p.hidden;
    p.in_view = true;
    return p;
}

// project3d(): a point at radius rk (km) from the Earth's centre in direction u.
static proj_t project3d(const view_t *v, const double u[3], double rk)
{
    proj_t p = {0};
    if (v->kind == GLOBE_PILOT) return project_pilot(v, u, rk);
    if (v->kind == GLOBE_FLAT) {   // flat map: ground position only, nothing is ever hidden
        flat_xy(v->rot[0], (float)u[0], (float)u[1], (float)u[2], &p.x, &p.y);
        p.v[0] = p.x - CX; p.v[1] = CY - p.y;
        p.front = p.in_view = true;
        return p;
    }
    float e[3], k = (float)(R * rk / EARTH_R_KM);
    mul3(&v->M, (float)u[0], (float)u[1], (float)u[2], e);
    p.x = CX + k * e[0];
    p.y = CY - k * e[1];
    p.hidden = e[2] < 0 && k * hypotf(e[0], e[1]) < R;
    p.front = e[2] >= 0;
    p.in_view = true;
    p.v[0] = k * e[0]; p.v[1] = k * e[1]; p.v[2] = k * e[2];
    return p;
}

// A point on the ground (unit vector), in floats (doubles are software on this chip).
static proj_t project_unitf(const view_t *v, const float u[3])
{
    if (v->kind == GLOBE_PILOT) {
        double d[3] = {u[0], u[1], u[2]};
        return project3d(v, d, EARTH_R_KM);
    }
    proj_t p = {0};
    if (v->kind == GLOBE_FLAT) {
        flat_xy(v->rot[0], u[0], u[1], u[2], &p.x, &p.y);
        p.v[0] = p.x - CX; p.v[1] = CY - p.y;
        p.front = p.in_view = true;
        return p;
    }
    float e[3];
    mul3(&v->M, u[0], u[1], u[2], e);
    p.x = CX + R * e[0];
    p.y = CY - R * e[1];
    p.hidden = e[2] < 0 && e[0] * e[0] + e[1] * e[1] < 1;
    p.front = e[2] >= 0;
    p.in_view = true;
    p.v[0] = R * e[0]; p.v[1] = R * e[1]; p.v[2] = R * e[2];
    return p;
}

// Frame in which the track and the flight direction are drawn: satellite and pilot views
// use the current sidereal angle for every point (the inertial orbit), the others each
// point's own (the ground-relative path).
static bool inertial_frame(void) { return st.mode == MODE_ORBIT || st.mode == MODE_PILOT; }

static void frame_unit(const orbpt_t *p, double u[3])
{
    if (inertial_frame()) {
        // One sidereal angle per frame: its cosine and sine once (doubles are software here).
        static double g = NAN, c = 1, s = 0;
        static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        if (st.now.gmst != g) {
            double nc = cos(st.now.gmst), ns = sin(st.now.gmst);
            taskENTER_CRITICAL(&lock);
            c = nc; s = ns; g = st.now.gmst;
            taskEXIT_CRITICAL(&lock);
        }
        taskENTER_CRITICAL(&lock);
        const double cc = c, ss = s;
        taskEXIT_CRITICAL(&lock);
        u[0] = p->eci[0] * cc + p->eci[1] * ss;
        u[1] = -p->eci[0] * ss + p->eci[1] * cc;
        u[2] = p->eci[2];
    } else {
        u[0] = p->ecef[0]; u[1] = p->ecef[1]; u[2] = p->ecef[2];
    }
}

// cos, sin of this frame's sidereal angle, set before the frame is recorded (float paths).
static float frame_gc = 1, frame_gs = 0;

// project3d() in floats, for the globe and flat views (the pilot view needs doubles).
static proj_t project3f(const view_t *v, const float u[3], float rk)
{
    proj_t p = {0};
    if (v->kind == GLOBE_FLAT) {
        flat_xy(v->rot[0], u[0], u[1], u[2], &p.x, &p.y);
        p.v[0] = p.x - CX; p.v[1] = CY - p.y;
        p.front = p.in_view = true;
        return p;
    }
    float e[3], k = R * rk * (float)(1 / EARTH_R_KM);
    mul3(&v->M, u[0], u[1], u[2], e);
    p.x = CX + k * e[0];
    p.y = CY - k * e[1];
    p.hidden = e[2] < 0 && k * k * (e[0] * e[0] + e[1] * e[1]) < R * R;
    p.front = e[2] >= 0;
    p.in_view = true;
    p.v[0] = k * e[0]; p.v[1] = k * e[1]; p.v[2] = k * e[2];
    return p;
}

static proj_t project_orbpt(const view_t *v, const orbpt_t *p)
{
    if (v->kind == GLOBE_PILOT) {
        double u[3];
        frame_unit(p, u);
        return project3d(v, u, p->rk);
    }
    float u[3];
    if (inertial_frame()) {
        u[0] = p->eci[0] * frame_gc + p->eci[1] * frame_gs;
        u[1] = -p->eci[0] * frame_gs + p->eci[1] * frame_gc;
        u[2] = p->eci[2];
    } else {
        u[0] = p->ecef[0]; u[1] = p->ecef[1]; u[2] = p->ecef[2];
    }
    return project3f(v, u, (float)p->rk);
}

// ---------- strokes printed on the surface ----------
// bake: draw straight into this context (background cache) instead of the display list;
// hair: a dl_hair_set + 1 to record into (line sets), 0: dl_thin.
typedef struct { float w; rgb_t c; float a; const gfx_dash_t *dash; gfx_ctx_t *bake; int hair; } stroke_t;

static inline float q16(float v) { return ((int)(v * 16 + 32768.5f) - 32768) * (1 / 16.0f); }

// A segment printed on the globe: coloured like the lit surface at its middle.
static void surface_segment(const view_t *v, float x0, float y0, float x1, float y1, const stroke_t *sk, gfx_dash_t *dash)
{
    int mx = (int)((x0 + x1) * 0.5f), my = (int)((y0 + y1) * 0.5f);
    float k = 0;
    rgb_t c = sk->c;
    if ((unsigned)mx < S && (unsigned)my < S) { k = 1; c = globe_shaded_stroke(mx, my, sk->c, v->kind == GLOBE_FLAT, &k); }
    if (k > 0) {
        if (sk->bake) {
            // As the display list stores it: 1/16 px, the dash phase within its period.
            float on = dash ? dash->on : 0, off = dash ? dash->off : 0, ph = 0;
            if (dash) {
                float pos = fmaxf(dash->pos, 0), period = on + off;
                ph = (int)((pos - period * (int)(pos / period)) * 16) * (1 / 16.0f);
            }
            gfx_thin_seg_on(sk->bake, q16(x0), q16(y0), q16(x1), q16(y1), sk->w, c, (int)(sk->a * 256.0f + 0.5f), on, off, ph);
        } else if (sk->hair) {
            dl_hair(sk->hair - 1, x0, y0, x1, y1, c, dash ? dash->pos : 0);
        } else {
            dl_thin(x0, y0, x1, y1, sk->w, c, sk->a, dash);
        }
    }
    if (dash) dash->pos += sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

// Vertices closer than this (squared, px) to the last drawn one are merged.
#define MERGE_PX2 64.0f

// Stroke a path of Earth-fixed unit vectors on the surface, cutting at the horizon like
// clipAngle(90).
// fxy: the points' flat-map positions at rot0 = 0 relative to the centre (flat_xy without the
// turn), or NULL; cs: cos, sin of the turn.
static void draw_path(const view_t *v, const float *pts, const float *fxy, const float cs[2], int n, bool closed,
                      const stroke_t *sk)
{
    gfx_dash_t dash = sk->dash ? *sk->dash : (gfx_dash_t){0};
    gfx_dash_t *dp = sk->dash ? &dash : NULL;
    float px = 0, py = 0;
    bool open = false;
    int count = closed ? n + 1 : n;
    float a[3], b[3];
    if (v->kind == GLOBE_FLAT) {
        px = CX + fxy[1] * cs[0] + fxy[0] * cs[1];
        py = CY + fxy[0] * cs[0] - fxy[1] * cs[1];
        for (int j = 1; j < count; j++) {
            const float *q = fxy + 2 * (j % n);
            float x = CX + q[1] * cs[0] + q[0] * cs[1], y = CY + q[0] * cs[0] - q[1] * cs[1];
            float dx = x - px, dy = y - py;
            if (j == count - 1 || dx * dx + dy * dy >= MERGE_PX2) {
                surface_segment(v, px, py, x, y, sk, dp);
                px = x; py = y;
            }
        }
        return;
    }
    mul3(&v->M, pts[0], pts[1], pts[2], a);
    for (int j = 1; j < count; j++) {
        const float *q = pts + 3 * (j % n);
        mul3(&v->M, q[0], q[1], q[2], b);
        if (a[2] >= 0 || b[2] >= 0) {
            float s[2] = {a[0], a[1]}, e[2] = {b[0], b[1]};
            if (a[2] < 0 || b[2] < 0) {
                // Horizon crossing: the point where z = 0, pushed out to the limb.
                float t = a[2] / (a[2] - b[2]);
                float hx = a[0] + (b[0] - a[0]) * t, hy = a[1] + (b[1] - a[1]) * t;
                float hn = sqrtf(hx * hx + hy * hy);
                if (hn > 0) { hx /= hn; hy /= hn; }
                float *out = a[2] < 0 ? s : e;
                out[0] = hx; out[1] = hy;
            }
            if (!open || a[2] < 0) {   // new subpath; dashes restart
                px = CX + R * s[0]; py = CY - R * s[1];
                open = true;
                dash.pos = 0;
            }
            float x1 = CX + R * e[0], y1 = CY - R * e[1];
            bool last = b[2] < 0 || j == count - 1;
            float dx = x1 - px, dy = y1 - py;
            if (last || dx * dx + dy * dy >= MERGE_PX2) {
                surface_segment(v, px, py, x1, y1, sk, dp);
                px = x1; py = y1;
            }
            if (b[2] < 0) open = false;
        } else {
            open = false;
        }
        memcpy(a, b, sizeof a);
    }
}

// Line sets from generated/*.bin (int16 unit vectors), converted to floats at start-up.
typedef struct { uint32_t nlines; uint16_t *lens; float *pts, *flat; } lineset_t;
static lineset_t borders, graticule;
extern const uint8_t borders_bin_start[] asm("_binary_borders_bin_start");
extern const uint8_t graticule_bin_start[] asm("_binary_graticule_bin_start");

static lineset_t load_lines(const uint8_t *bin)
{
    uint32_t hdr[2];
    memcpy(hdr, bin, sizeof hdr);
    lineset_t ls = {.nlines = hdr[0]};
    ls.lens = heap_caps_malloc(hdr[0] * 2, MALLOC_CAP_SPIRAM);
    ls.pts = heap_caps_malloc(hdr[1] * 3 * sizeof(float), MALLOC_CAP_SPIRAM);
    memcpy(ls.lens, bin + 8, hdr[0] * 2);
    const uint8_t *p = bin + 8 + hdr[0] * 2;
    for (uint32_t i = 0; i < hdr[1] * 3; i++) {
        int16_t q;
        memcpy(&q, p + 2 * i, 2);
        ls.pts[i] = q / 32767.0f;
    }
    // Flat-map positions before the turn: the map only turns about its centre.
    ls.flat = heap_caps_malloc(hdr[1] * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    for (uint32_t i = 0; i < hdr[1]; i++) {
        float x, y;
        flat_xy(0, ls.pts[3 * i], ls.pts[3 * i + 1], ls.pts[3 * i + 2], &x, &y);
        // flat_xy(0): x = CX + k·uy, y = CY + k·ux
        ls.flat[2 * i] = y - CY;
        ls.flat[2 * i + 1] = x - CX;
    }
    return ls;
}

static void draw_lineset(const view_t *v, const lineset_t *ls, const stroke_t *s)
{
    const float *p = ls->pts, *f = ls->flat;
    const float cs[2] = {cosf(v->rot[0] / (float)DEG), sinf(v->rot[0] / (float)DEG)};
    for (uint32_t i = 0; i < ls->nlines; i++) {
        draw_path(v, p, f, cs, ls->lens[i], false, s);
        p += 3 * ls->lens[i];
        f += 2 * ls->lens[i];
    }
}

// ---------- compass rose with elevation rings (drawFootprint()) ----------
typedef struct { float x, y, hw, hh; int e; } label_box_t;
typedef struct { float c[3], north[3], east[3]; } geo_frame_t;

static void geo_frame(const float c[3], geo_frame_t *g)
{
    memcpy(g->c, c, sizeof g->c);
    g->east[0] = -c[1]; g->east[1] = c[0]; g->east[2] = 0;
    float n = hypotf(g->east[0], g->east[1]);
    if (n < 1e-6f) { g->east[0] = 0; g->east[1] = 1; } else { g->east[0] /= n; g->east[1] /= n; }
    cross3f(c, g->east, g->north);
}

// destination(): the point `dist` (rad) from the centre along azimuth `az` (rad, from north),
// from the cosines and sines of both.
static inline void destination_cs(const geo_frame_t *g, float ca, float sa, float cd, float sd, float out[3])
{
    for (int k = 0; k < 3; k++) out[k] = g->c[k] * cd + (g->north[k] * ca + g->east[k] * sa) * sd;
}

static void destination(const geo_frame_t *g, float az, float dist, float out[3])
{
    destination_cs(g, cosf(az), sinf(az), cosf(dist), sinf(dist), out);
}

// Azimuths every 2° (the rings), cos and sin.
static float az_cs[181][2];

static void rose_init(void)
{
    for (int a = 0; a <= 180; a++) {
        az_cs[a][0] = cosf(a * 2 * (float)M_PI / 180);
        az_cs[a][1] = sinf(a * 2 * (float)M_PI / 180);
    }
}

static bool in_gaps(float x, float y, const label_box_t *gaps, int n)
{
    for (int i = 0; i < n; i++)
        if (fabsf(x - gaps[i].x) < gaps[i].hw && fabsf(y - gaps[i].y) < gaps[i].hh) return true;
    return false;
}

// A screen polyline through points on the ground, broken where hidden or inside a label gap.
static void ground_polyline(const view_t *v, const float *pts, int n, const label_box_t *gaps, int ngaps, const stroke_t *sk)
{
    bool pen = false;
    float px = 0, py = 0;
    for (int i = 0; i < n; i++) {
        proj_t q = project_unitf(v, pts + 3 * i);
        bool ok = q.front && q.in_view && !in_gaps(q.x, q.y, gaps, ngaps);
        if (ok && pen) surface_segment(v, px, py, q.x, q.y, sk, NULL);
        px = q.x; py = q.y;
        pen = ok;
    }
}

static void draw_rose(const view_t *v)
{
    if (!st.now.ok) return;
    double cd[3];
    double rk = model_geo(st.now.r, st.now.gmst, cd);
    float c[3] = {(float)cd[0], (float)cd[1], (float)cd[2]};
    geo_frame_t g;
    geo_frame(c, &g);
    const float k = (float)(EARTH_R_KM / rk);
    const int RINGS[3] = {0, 10, 20};
    float ring[3];
    for (int i = 0; i < 3; i++) {
        float e = RINGS[i] * (float)M_PI / 180;
        ring[i] = acosf(k * cosf(e)) - e;   // ground radius for elevation ε
    }
    proj_t apex = project3d(v, cd, rk);
    const bool flat = v->kind == GLOBE_FLAT;

    // Elevation labels first (on the W arm), so the rings can leave gaps for them.
    label_box_t labels[3];
    int nl = 0;
    char txt[3][8];
    for (int i = 0; i < 3; i++) {
        float p[3];
        destination(&g, 1.5f * (float)M_PI, ring[i], p);
        proj_t q = project_unitf(v, p);
        if (!q.front || !q.in_view) continue;
        if (!apex.hidden && hypotf(apex.x - q.x, apex.y - q.y) < S * 0.03f) continue;   // under the object
        snprintf(txt[i], sizeof txt[i], "%d°", RINGS[i]);
        float hw = gfx_text_width(&FONT_600_13, txt[i], 0) / 2 + S * 0.003f, hh = 13 * 0.55f;
        bool clash = false;
        for (int j = 0; j < nl; j++)
            if (fabsf(labels[j].x - q.x) < labels[j].hw + hw && fabsf(labels[j].y - q.y) < labels[j].hh + hh) clash = true;
        if (clash) continue;
        labels[nl++] = (label_box_t){q.x, q.y, hw, hh, i};
    }
    // Rings: the horizon a little heavier, the inner range rings fine.
    float pts[3 * 181];
    for (int i = 0; i < 3; i++) {
        const float cd = cosf(ring[i]), sd = sinf(ring[i]);
        for (int a = 0; a <= 180; a++) destination_cs(&g, az_cs[a][0], az_cs[a][1], cd, sd, pts + 3 * a);
        stroke_t sk = {i == 0 ? fmaxf(1.2f, S / 320.0f) : fmaxf(0.8f, S / 600.0f), AMBER, 1, NULL, NULL, 0};
        ground_polyline(v, pts, 181, labels, nl, &sk);
    }
    for (int j = 0; j < nl; j++) {
        float a = 1;
        rgb_t col = globe_shaded_stroke((int)labels[j].x, (int)labels[j].y, AMBER, flat, &a);
        dl_text(&FONT_600_13, txt[labels[j].e], labels[j].x, labels[j].y, GFX_CENTER, col, 1, 0);
    }
    // Thin cross through the sub-satellite point, broken where the labels sit.
    stroke_t thin = {fmaxf(0.8f, S / 600.0f), AMBER, 1, NULL, NULL, 0};
    const float cstep = cosf(ring[0] / 60), sstep = sinf(ring[0] / 60);
    for (int az = 0; az < 4; az++) {
        float arm[3 * 61], cd = 1, sd = 0;   // distance along the arm by rotation steps
        for (int i = 0; i <= 60; i++) {
            destination_cs(&g, az_cs[45 * az][0], az_cs[45 * az][1], cd, sd, arm + 3 * i);
            float nc = cd * cstep - sd * sstep;
            sd = sd * cstep + cd * sstep;
            cd = nc;
        }
        ground_polyline(v, arm, 61, labels, nl, &thin);
    }
    // Ticks outside the horizon ring every 10°, long every 30°, and N / E / S / W beyond them.
    const float px_rad = (flat ? (float)M_PI : 1.0f) / R;
    stroke_t tk = {fmaxf(1, S / 480.0f), AMBER, 1, NULL, NULL, 0};
    static const char *ROSE[4] = {"N", "E", "S", "W"};
    for (int az = 0; az < 360; az += 10) {
        bool major = az % 30 == 0;
        float len = (major ? S / 50.0f : S / 95.0f) * px_rad, ar = az * (float)M_PI / 180;
        float p1[3], p2[3];
        const float ca = az_cs[az / 2][0], sa = az_cs[az / 2][1];
        destination_cs(&g, ca, sa, cosf(ring[0]), sinf(ring[0]), p1);
        destination_cs(&g, ca, sa, cosf(ring[0] + len), sinf(ring[0] + len), p2);
        (void)ar;
        proj_t a = project_unitf(v, p1), b = project_unitf(v, p2);
        if (!a.front || !a.in_view || !b.front) continue;
        surface_segment(v, a.x, a.y, b.x, b.y, &tk, NULL);
        if (az % 90 == 0) {
            float dx = b.x - a.x, dy = b.y - a.y, n = hypotf(dx, dy);
            if (n < 1e-6f) n = 1;
            const char *t = ROSE[az / 90];
            float off = fmaxf(gfx_text_width(&FONT_600_15, t, 0), 15) * 0.6f + S * 0.006f;
            float lx = b.x + dx / n * off, ly = b.y + dy / n * off, la = 1;
            rgb_t col = globe_shaded_stroke((int)lx, (int)ly, AMBER, flat, &la);
            if (la <= 0) col = AMBER;
            dl_text(&FONT_600_15, t, lx, ly, GFX_CENTER, col, 1, 0);
        }
    }
}

// ---------- orbit line, ticks, object ----------
static proj_t track_pts[TRACK_N];
static double track_t[TRACK_N];
static int track_n;

static void project_track(const view_t *v, const snapshot_t *s)
{
    track_n = 0;
    if (!s || s->ntrack < 2 || !st.now.ok) return;
    double shift = st.t - s->t;   // the snapshot is up to a second old
    for (int i = 0; i < s->ntrack; i++) {
        track_pts[i] = project_orbpt(v, &s->track[i].p);
        track_t[i] = s->track[i].t - shift;
    }
    track_n = s->ntrack;
}

// One travel-time tick across the line (fine ruler line, longer every 5 minutes).
static void tick(const view_t *v, const proj_t *p, const proj_t *q, bool major, rgb_t col, float alpha, bool printed)
{
    float dx = q->x - p->x, dy = q->y - p->y, n = hypotf(dx, dy);
    if (n < 1e-6f) return;
    float half = major ? S / 95.0f : S / 170.0f, nx = -dy / n * half, ny = dx / n * half;
    float w = fmaxf(1, S / 480.0f) * (major ? 1.25f : 1);
    if (printed) {
        stroke_t sk = {w, col, alpha, NULL, NULL, 0};
        surface_segment(v, p->x - nx, p->y - ny, p->x + nx, p->y + ny, &sk, NULL);
    } else {
        dl_thin(p->x - nx, p->y - ny, p->x + nx, p->y + ny, w, col, alpha, NULL);
    }
}

// drawBehindEarth(): what is behind the Earth keeps the orbit's projected shape, printed grey
// on the visible surface under the glass, so the globe stays opaque.
static void draw_behind(const view_t *v, const snapshot_t *s)
{
    if (v->kind != GLOBE_ORTHO || track_n < 2) return;
    for (int i = 1; i < track_n; i++) {
        const proj_t *a = &track_pts[i - 1], *b = &track_pts[i];
        if (!(a->hidden || b->hidden)) continue;
        stroke_t sk = {S / 260.0f, BEHIND, BEHIND_ALPHA * track_alpha((track_t[i - 1] + track_t[i]) / 2), NULL, NULL, 0};
        surface_segment(v, a->x, a->y, b->x, b->y, &sk, NULL);
    }
    double shift = st.t - s->t;
    for (int i = 0; i < s->nmarks; i++) {
        const mark_t *m = &s->marks[i];
        proj_t a = project_orbpt(v, &m->a);
        if (!a.hidden) continue;
        proj_t b = project_orbpt(v, &m->b);
        tick(v, &a, &b, m->major, BEHIND, BEHIND_ALPHA * track_alpha(m->t - shift), true);
    }
    if (st.now.ok) {
        double u[3];
        double rk = model_geo(st.now.r, st.now.gmst, u);
        proj_t a = project3d(v, u, rk);
        if (a.hidden && a.in_view) {   // hollow ring for a hidden object
            stroke_t sk = {S / 260.0f, BEHIND, 1, NULL, NULL, 0};
            float r = S / 55.0f, px = a.x + r, py = a.y;
            for (int k = 1; k <= 48; k++) {
                float t = k * 2 * (float)M_PI / 48, x = a.x + r * cosf(t), y = a.y + r * sinf(t);
                surface_segment(v, px, py, x, y, &sk, NULL);
                px = x; py = y;
            }
        }
    }
}

static void draw_track(void)
{
    for (int i = 1; i < track_n; i++) {
        const proj_t *a = &track_pts[i - 1], *b = &track_pts[i];
        if (a->hidden || b->hidden || !a->in_view || !b->in_view) continue;   // grey under the glass instead
        dl_line(a->x, a->y, b->x, b->y, S / 200.0f, TRACK, track_alpha((track_t[i - 1] + track_t[i]) / 2), true);
    }
}

static void draw_mark_ticks(const view_t *v, const snapshot_t *s)
{
    if (!s) return;
    double shift = st.t - s->t;
    for (int i = 0; i < s->nmarks; i++) {
        const mark_t *m = &s->marks[i];
        proj_t p = project_orbpt(v, &m->a);
        if (p.hidden || !p.in_view) continue;
        proj_t q = project_orbpt(v, &m->b);
        tick(v, &p, &q, m->major, TRACK, track_alpha(m->t - shift), false);
    }
}

static void draw_reticle(void)
{
    const rgb_t c = {30, 30, 30};
    const float a = R * 0.06f, gap = R * 0.02f;
    dl_line(CX - a, CY, CX - gap, CY, 1, c, .55f, false);
    dl_line(CX + gap, CY, CX + a, CY, 1, c, .55f, false);
    dl_line(CX, CY - a, CX, CY - gap, 1, c, .55f, false);
    dl_line(CX, CY + gap, CX, CY + a, 1, c, .55f, false);
}

// Home marker: blue dot with white rim, like an ownship symbol on a chart, and its name.
static void draw_home(const view_t *v)
{
    double lo = HOME_LON / DEG, la = HOME_LAT / DEG, u[3] = {cos(la) * cos(lo), cos(la) * sin(lo), sin(la)};
    proj_t p = project3d(v, u, EARTH_R_KM);
    if (!p.front || !p.in_view) return;
    const float rr = S / 150.0f;
    dl_fill_circle(p.x, p.y, rr, RGB(0x1e, 0x7b, 0xff), 1);
    dl_stroke_circle(p.x, p.y, rr, fmaxf(1.2f, S / 360.0f), WHITE, 1);
    dl_text(&FONT_500_20, HOME_NAME, p.x + rr * 2, p.y, GFX_LEFT, WHITE, 1, 0);
}

// Simplified rocket upper stage (drawStage). Segments run tail -> nose: t in body lengths, r in body radii.
typedef struct { float t0, r0, t1, r1; rgb_t lit, base, dark, cap; } stage_seg_t;
static const stage_seg_t STAGE[] = {
    {-0.80f, 0.80f, -0.50f, 0.30f, {0xa3, 0xa3, 0xa3}, {0x4a, 0x4a, 0x4a}, {0x12, 0x12, 0x12}, {0x0a, 0x0a, 0x0a}}, // nozzle
    {-0.50f, 0.62f, -0.42f, 1.00f, {0x8d, 0x92, 0x97}, {0x50, 0x55, 0x5a}, {0x1e, 0x21, 0x24}, {0x3a, 0x3e, 0x42}}, // aft dome
    {-0.42f, 1.00f, 0.30f, 1.00f, {0xff, 0xff, 0xff}, {0xd8, 0xdc, 0xe0}, {0x64, 0x6b, 0x72}, {0xc4, 0xc9, 0xcd}},  // tank
    {0.30f, 1.04f, 0.37f, 1.04f, {0x7c, 0xb9, 0xff}, {0x1f, 0x6f, 0xd1}, {0x0a, 0x2b, 0x5c}, {0x1a, 0x5b, 0xb0}},   // ring
    {0.37f, 0.92f, 0.48f, 0.55f, {0x7b, 0x80, 0x86}, {0x3d, 0x42, 0x47}, {0x14, 0x17, 0x1a}, {0x2a, 0x2e, 0x33}},   // adapter
    {0.48f, 0.72f, 0.68f, 0.72f, {0xf0, 0xd2, 0x7a}, {0xb0, 0x8a, 0x2e}, {0x43, 0x35, 0x0e}, {0x8a, 0x6c, 0x22}},   // payload
};
#define STAGE_RADIUS 0.17f
static const float LIGHT[2] = {-0.55f, -0.83f};   // screen direction toward the light (upper left)

typedef struct { float x, y, s, c, rm, sign; stage_seg_t seg; } stage_shader_t;

static rgb_t mix(rgb_t a, rgb_t b, float k)
{
    return (rgb_t){(uint8_t)(a.r + (b.r - a.r) * k + 0.5f), (uint8_t)(a.g + (b.g - a.g) * k + 0.5f),
                   (uint8_t)(a.b + (b.b - a.b) * k + 0.5f)};
}

// createLinearGradient(0, -rm*sign, 0, rm*sign) in the stage's rotated frame.
static rgb_t stage_shader(float px, float py, void *vctx, float *alpha)
{
    const stage_shader_t *g = vctx;
    float ly = -(px - g->x) * g->s + (py - g->y) * g->c;
    float t = ly * g->sign / (2 * g->rm) + 0.5f;
    const stage_seg_t *c = &g->seg;
    if (t <= 0) return c->dark;
    if (t < 0.55f) return mix(c->dark, c->base, t / 0.55f);
    if (t < 0.85f) return mix(c->base, c->lit, (t - 0.55f) / 0.30f);
    if (t < 1) return mix(c->lit, c->base, (t - 0.85f) / 0.15f);
    return c->base;
}

#define ARC_STEPS 16

static void draw_stage(float x, float y, const float dir[3], float len)
{
    float dx = dir[0], dy = dir[1], dz = dir[2];   // view space, y up, z toward viewer
    float theta = atan2f(-dy, dx);
    float f = fmaxf(0.12f, hypotf(dx, dy));        // foreshortening of the axis
    float capK = fmaxf(0.06f, fabsf(dz));          // end caps seen as ellipses
    float Rb = len * STAGE_RADIUS;
    float sn = sinf(theta), cs = cosf(theta);
    float ly = LIGHT[0] * -sn + LIGHT[1] * cs;
    float sign = ly >= 0 ? 1 : -1;
    bool nose_near = dz >= 0;
    const int n = sizeof STAGE / sizeof STAGE[0];
#define TX(a, b) (x + (a) * cs - (b) * sn)
#define TY(a, b) (y + (a) * sn + (b) * cs)
    for (int k = 0; k < n; k++) {
        const stage_seg_t *sg = &STAGE[nose_near ? k : n - 1 - k];   // painter's order: far end first
        float a0 = sg->t0 * len * f, a1 = sg->t1 * len * f;
        float rr0 = sg->r0 * Rb, rr1 = sg->r1 * Rb, rm = fmaxf(rr0, rr1);
        float farA = nose_near ? a0 : a1, farR = nose_near ? rr0 : rr1;
        float nearA = nose_near ? a1 : a0, nearR = nose_near ? rr1 : rr0;
        float exFar = fabsf(fmaxf(0.3f, farR * capK * (nose_near ? 1 : -1)));
        float exNear = fabsf(fmaxf(0.3f, nearR * capK * (nose_near ? 1 : -1)));
        float poly[2 * (ARC_STEPS + 4)];
        int m = 0;
        poly[m++] = TX(nearA, -nearR); poly[m++] = TY(nearA, -nearR);
        for (int i = 0; i <= ARC_STEPS; i++) {
            float t = -(float)M_PI_2 + (nose_near ? 1 : -1) * (float)M_PI * i / ARC_STEPS;
            float ea = farA + exFar * cosf(t), eb = farR * sinf(t);
            poly[m++] = TX(ea, eb); poly[m++] = TY(ea, eb);
        }
        poly[m++] = TX(nearA, nearR); poly[m++] = TY(nearA, nearR);
        stage_shader_t sh = {x, y, sn, cs, rm, sign, *sg};
        dl_fill_poly(poly, m / 2, stage_shader, &sh, sizeof sh, sg->base, 1);
        dl_stroke_poly(poly, m / 2, 0.7f, BLACK, .55f);
        float cap[2 * 2 * ARC_STEPS];
        int c = 0;
        for (int i = 0; i < 2 * ARC_STEPS; i++) {
            float t = (float)M_PI * i / ARC_STEPS;
            float ea = nearA + exNear * cosf(t), eb = nearR * sinf(t);
            cap[c++] = TX(ea, eb); cap[c++] = TY(ea, eb);
        }
        dl_fill_poly(cap, c / 2, NULL, NULL, 0, sg->cap, 1);
        dl_stroke_poly(cap, c / 2, 0.7f, BLACK, .55f);
    }
#undef TX
#undef TY
}

static void draw_satellite(const view_t *v)
{
    if (!st.now.ok) return;
    double u[3];
    double rk = model_geo(st.now.r, st.now.gmst, u);
    proj_t p = project3d(v, u, rk);
    if (p.hidden) return;   // behind the Earth: grey ring from draw_behind() instead
    proj_t g = project3d(v, u, EARTH_R_KM);
    if (g.front) {   // drop line to the sub-satellite point
        dl_line(g.x, g.y, p.x, p.y, 1, RGB(30, 30, 30), .6f, false);
        dl_fill_circle(g.x, g.y, S / 300.0f, RGB(30, 30, 30), .7f);
    }
    float dir[3] = {1, 0, 0};
    if (st.ahead.ok) {
        double ua[3];
        double ra = model_geo(st.ahead.r, inertial_frame() ? st.now.gmst : st.ahead.gmst, ua);
        proj_t q = project3d(v, ua, ra);
        float d[3] = {q.v[0] - p.v[0], q.v[1] - p.v[1], q.v[2] - p.v[2]};
        float n = sqrtf(dot3f(d, d));
        if (n == 0) n = 1;
        for (int i = 0; i < 3; i++) dir[i] = d[i] / n;
    }
    draw_stage(p.x, p.y, dir, S / 13.0f);
}

// ---------- pilot view ----------
// updateCamera(): forward = horizontal prograde, up = radial, then yaw and pitch look-around.
static void update_camera(void)
{
    cam.ok = st.now.ok;
    if (!cam.ok) return;
    memcpy(cam.c, st.now.r, sizeof cam.c);
    cam.gmst = st.now.gmst;
    float up[3] = {(float)st.now.r[0], (float)st.now.r[1], (float)st.now.r[2]};
    norm3f(up);
    float vv[3] = {(float)st.now.v[0], (float)st.now.v[1], (float)st.now.v[2]};
    float vu = dot3f(vv, up), f[3] = {vv[0] - vu * up[0], vv[1] - vu * up[1], vv[2] - vu * up[2]};
    norm3f(f);
    float rt[3];
    cross3f(f, up, rt);
    float yaw = -st.rotate[0] / (float)DEG, pitch = -st.rotate[1] / (float)DEG;
    float cy = cosf(yaw), sy = sinf(yaw), cp = cosf(pitch), sp = sinf(pitch);
    for (int i = 0; i < 3; i++) {
        float hf = f[i] * cy + rt[i] * sy, r2 = rt[i] * cy - f[i] * sy;
        cam.hf[i] = hf;
        cam.rt[i] = r2;
        cam.f[i] = hf * cp + up[i] * sp;
        cam.u[i] = up[i] * cp - hf * sp;
        cam.up[i] = up[i];
    }
}

static bool project_dir(const float d[3], float *x, float *y)
{
    float z = dot3f(d, cam.f);
    if (z <= 0.05f) return false;
    *x = S / 2.0f + PILOT_K * dot3f(d, cam.rt) / z;
    *y = S / 2.0f - PILOT_K * dot3f(d, cam.u) / z;
    return true;
}

// drawPilotTrack(): 30 s segments split into short arc pieces, clipped at a near plane (the
// orbit runs through the camera) and dropped where the Earth blocks the line of sight.
static void draw_pilot_track(const snapshot_t *s)
{
    if (!s || s->ntrack < 2) return;
    const float cc[3] = {(float)cam.c[0], (float)cam.c[1], (float)cam.c[2]};
    const float RE2 = (float)(EARTH_R_KM * EARTH_R_KM), c2 = dot3f(cc, cc);
    double shift = st.t - s->t;
    for (int i = 1; i < s->ntrack; i++) {
        const orbpt_t *A = &s->track[i - 1].p, *B = &s->track[i].p;
        float pa[3], pb[3];
        for (int k = 0; k < 3; k++) { pa[k] = A->eci[k] * (float)A->rk; pb[k] = B->eci[k] * (float)B->rk; }
        float ra = (float)A->rk, rb = (float)B->rk;
        for (int j = 0; j < PILOT_SUBDIV; j++) {
            float q[2][4];   // camera x, y, z and visibility
            for (int e = 0; e < 2; e++) {
                float t = (float)(j + e) / PILOT_SUBDIV, p[3];
                for (int k = 0; k < 3; k++) p[k] = pa[k] + (pb[k] - pa[k]) * t;
                float n = sqrtf(dot3f(p, p)), rr = ra * (1 - t) + rb * t;
                for (int k = 0; k < 3; k++) p[k] = p[k] / n * rr;
                float rel[3] = {p[0] - cc[0], p[1] - cc[1], p[2] - cc[2]};
                q[e][0] = dot3f(rel, cam.rt); q[e][1] = dot3f(rel, cam.u); q[e][2] = dot3f(rel, cam.f);
                float dist = sqrtf(dot3f(rel, rel)), b = dist > 0 ? dot3f(cc, rel) / dist : 0, disc = b * b - (c2 - RE2);
                float t1 = disc >= 0 ? -b - sqrtf(disc) : -1;
                q[e][3] = !(t1 > 0 && t1 < dist - 1e-3f);
            }
            if (q[0][2] < PILOT_NEAR_KM && q[1][2] < PILOT_NEAR_KM) continue;
            if (!q[0][3] || !q[1][3]) continue;
            if (q[0][2] < PILOT_NEAR_KM || q[1][2] < PILOT_NEAR_KM) {   // clip to the near plane
                int lo = q[0][2] < PILOT_NEAR_KM ? 0 : 1;
                float t = (PILOT_NEAR_KM - q[lo][2]) / (q[1 - lo][2] - q[lo][2]);
                for (int k = 0; k < 2; k++) q[lo][k] = q[lo][k] + (q[1 - lo][k] - q[lo][k]) * t;
                q[lo][2] = PILOT_NEAR_KM;
            }
            double tt = s->track[i - 1].t + (s->track[i].t - s->track[i - 1].t) * (j + 0.5) / PILOT_SUBDIV - shift;
            float x0 = S / 2.0f + PILOT_K * q[0][0] / q[0][2], y0 = S / 2.0f - PILOT_K * q[0][1] / q[0][2];
            float x1 = S / 2.0f + PILOT_K * q[1][0] / q[1][2], y1 = S / 2.0f - PILOT_K * q[1][1] / q[1][2];
            if (fmaxf(fabsf(x0 - S / 2), fabsf(x1 - S / 2)) > 4 * S || fmaxf(fabsf(y0 - S / 2), fabsf(y1 - S / 2)) > 4 * S) continue;
            dl_line(x0, y0, x1, y1, S / 200.0f, TRACK, track_alpha(tt), true);
        }
    }
}

// HUD strokes: a dark halo under the green line (canvas shadowBlur).
#define HUD_LW 2.0f   // max(1.2, s/360)
static void hud_seg(float x0, float y0, float x1, float y1, const gfx_dash_t *dash)
{
    dl_thin(x0, y0, x1, y1, HUD_LW + 2.5f, BLACK, .35f, dash);
    dl_thin(x0, y0, x1, y1, HUD_LW, HUD_GREEN, 1, dash);
}

static void hud_text(const gfx_font_t *f, const char *s, float x, float y, gfx_align_t align)
{
    dl_text(f, s, x, y, align, HUD_GREEN, 1, 0);
}

// Clip a segment to a circle (the ladder's HUD glass); false if nothing is inside.
static bool clip_circle(float *x0, float *y0, float *x1, float *y1, float cx, float cy, float r)
{
    float dx = *x1 - *x0, dy = *y1 - *y0, fx = *x0 - cx, fy = *y0 - cy;
    float a = dx * dx + dy * dy, b = 2 * (fx * dx + fy * dy), c = fx * fx + fy * fy - r * r;
    if (a < 1e-9f) return c <= 0;
    float disc = b * b - 4 * a * c;
    if (disc <= 0) return false;
    float sq = sqrtf(disc), t0 = (-b - sq) / (2 * a), t1 = (-b + sq) / (2 * a);
    if (t1 < 0 || t0 > 1) return false;
    t0 = fmaxf(0, t0); t1 = fminf(1, t1);
    float ox = *x0, oy = *y0;
    *x0 = ox + dx * t0; *y0 = oy + dy * t0;
    *x1 = ox + dx * t1; *y1 = oy + dy * t1;
    return true;
}

// hudBox(): outlined value box with a caption under it; align: which screen side it hugs.
static void hud_box(const char *text, float x, float y, int align, const char *caption)
{
    const float fs = 22;
    float w = gfx_text_width(&FONT_600_22, text, 0) + fs * 0.8f, h = fs * 1.35f;
    float x0 = align < 0 ? x : align > 0 ? x - w : x - w / 2;
    hud_seg(x0, y - h / 2, x0 + w, y - h / 2, NULL);
    hud_seg(x0 + w, y - h / 2, x0 + w, y + h / 2, NULL);
    hud_seg(x0 + w, y + h / 2, x0, y + h / 2, NULL);
    hud_seg(x0, y + h / 2, x0, y - h / 2, NULL);
    hud_text(&FONT_600_22, text, x0 + w / 2, y + fs * 0.04f, GFX_CENTER);
    if (caption) hud_text(&FONT_500_15, caption, x0 + w / 2, y + h / 2 + fs * 0.6f, GFX_CENTER);
}

// Prograde symbol: circle with three stubs at the travel direction; parked on a circle with
// an arrow and the view-to-travel angle when it is off the HUD.
static void draw_prograde(void)
{
    const float c = S / 2.0f;
    float f[3] = {(float)st.now.v[0], (float)st.now.v[1], (float)st.now.v[2]};
    norm3f(f);
    float vx = dot3f(f, cam.rt), vy = dot3f(f, cam.u), vz = dot3f(f, cam.f);
    const float Rm = S * 0.2f, r = S / 48.0f;
    float x = 0, y = 0;
    bool clamped = false;
    if (vz > 0.05f) { x = c + PILOT_K * vx / vz; y = c - PILOT_K * vy / vz; }
    if (vz <= 0.05f || hypotf(x - c, y - c) > Rm) {
        float dx = vx, dy = -vy, n = hypotf(dx, dy);
        if (n < 1e-6f) { dx = 0; dy = 1; } else { dx /= n; dy /= n; }
        x = c + dx * Rm; y = c + dy * Rm;
        clamped = true;
    }
    const gfx_dash_t dash = {S / 120.0f, S / 160.0f, 0};
    for (int pass = 0; pass < 2; pass++) {
        float w = pass ? S / 300.0f : S / 160.0f, a = pass ? 1 : .65f;
        rgb_t col = pass ? HUD_GREEN : BLACK;
        gfx_dash_t d = dash;
        float px = x + r, py = y;
        for (int i = 1; i <= 32; i++) {
            float t = i * (float)M_PI / 16, qx = x + r * cosf(t), qy = y + r * sinf(t);
            dl_thin(px, py, qx, qy, w, col, a, clamped ? &d : NULL);
            d.pos += hypotf(qx - px, qy - py);
            px = qx; py = qy;
        }
        dl_line(x - r, y, x - r * 2, y, w, col, a, true);
        dl_line(x + r, y, x + r * 2, y, w, col, a, true);
        dl_line(x, y - r, x, y - r * 1.8f, w, col, a, true);
    }
    if (clamped) {
        float dx = (x - c) / Rm, dy = (y - c) / Rm, ax = x + dx * r * 2.6f, ay = y + dy * r * 2.6f, a = r * 0.8f;
        float tri[6] = {ax + dx * a, ay + dy * a, ax - dx * a - dy * a, ay - dy * a + dx * a, ax - dx * a + dy * a, ay - dy * a - dx * a};
        dl_fill_poly(tri, 3, NULL, NULL, 0, HUD_GREEN, 1);
        char t[16];
        snprintf(t, sizeof t, "%d°", (int)lroundf(acosf(fmaxf(-1, fminf(1, vz))) * (float)DEG));
        dl_text(&FONT_600_19, t, x - dx * r * 2.8f, y - dy * r * 2.8f, GFX_CENTER, HUD_GREEN, 1, 0);
    }
}

// drawPilotHud(): pitch ladder conformal to the local horizontal, heading tape, boresight,
// speed and altitude boxes, prograde marker.
static void draw_pilot_hud(void)
{
    const float c = S / 2.0f;
    const float *up = cam.up, *hf = cam.hf;
    float north[3] = {-up[2] * up[0], -up[2] * up[1], 1 - up[2] * up[2]};
    norm3f(north);
    float east[3], right_h[3];
    cross3f(north, up, east);
    cross3f(hf, up, right_h);
#define HDG(d) fmodf(atan2f(dot3f(d, east), dot3f(d, north)) * (float)DEG + 360, 360)
    float heading = HDG(hf);
    float vel[3] = {(float)st.now.v[0], (float)st.now.v[1], (float)st.now.v[2]};
    norm3f(vel);
    float vu = dot3f(vel, up), vh[3] = {vel[0] - vu * up[0], vel[1] - vu * up[1], vel[2] - vu * up[2]};
    norm3f(vh);
    float trk = HDG(vh);
#undef HDG
    // Pitch ladder, clipped to the HUD glass in the middle of the screen.
    const float glass = S * 0.34f;
    const gfx_dash_t below = {S / 70.0f, S / 110.0f, 0};
    for (int p = -80; p <= 80; p += 10) {
        float pr = p / (float)DEG, d[3];
        for (int i = 0; i < 3; i++) d[i] = hf[i] * cosf(pr) + up[i] * sinf(pr);
        float px, py, qx, qy;
        if (!project_dir(d, &px, &py)) continue;
        float d2[3] = {d[0] + right_h[0] * 0.01f, d[1] + right_h[1] * 0.01f, d[2] + right_h[2] * 0.01f};
        norm3f(d2);
        float ang = project_dir(d2, &qx, &qy) ? atan2f(qy - py, qx - px) : 0;
        float ca = cosf(ang), sa = sinf(ang);
        float half = p == 0 ? S * 0.2f : S * 0.09f, gap = S * 0.045f, tk = S * 0.018f * (p > 0 ? 1 : -1);
#define LX(a, b) (px + (a) * ca - (b) * sa)
#define LY(a, b) (py + (a) * sa + (b) * ca)
        const float segs[4][4] = {
            {-half, p ? tk : 0, -half, 0}, {-half, 0, -gap, 0}, {gap, 0, half, 0}, {half, 0, half, p ? tk : 0}};
        for (int k = 0; k < 4; k++) {
            if (!p && (k == 0 || k == 3)) continue;
            float x0 = LX(segs[k][0], segs[k][1]), y0 = LY(segs[k][0], segs[k][1]);
            float x1 = LX(segs[k][2], segs[k][3]), y1 = LY(segs[k][2], segs[k][3]);
            if (clip_circle(&x0, &y0, &x1, &y1, c, c, glass)) hud_seg(x0, y0, x1, y1, p < 0 ? &below : NULL);
        }
        if (p) {
            char t[8];
            snprintf(t, sizeof t, "%d", abs(p));
            float lx = LX(-half - S * 0.01f, 0), ly = LY(-half - S * 0.01f, 0);
            float rx = LX(half + S * 0.01f, 0), ry = LY(half + S * 0.01f, 0);
            float tw = gfx_text_width(&FONT_600_22, t, 0);
            if (hypotf(lx - c, ly - c) < glass - tw / 2) hud_text(&FONT_600_22, t, lx - tw, ly, GFX_LEFT);
            if (hypotf(rx - c, ry - c) < glass - tw / 2) hud_text(&FONT_600_22, t, rx, ry, GFX_LEFT);
        }
#undef LX
#undef LY
    }
    // Boresight "W" fixed at the screen centre.
    const float w = S * 0.022f;
    const float W[7][2] = {{c - w * 2, c}, {c - w, c}, {c - w * 0.5f, c + w * 0.6f}, {c, c}, {c + w * 0.5f, c + w * 0.6f}, {c + w, c}, {c + w * 2, c}};
    for (int i = 1; i < 7; i++) hud_seg(W[i - 1][0], W[i - 1][1], W[i][0], W[i][1], NULL);
    // Heading tape under the time tab.
    const float ty = S * 0.135f, span = 35, px_deg = S * 0.0065f;
    for (int hd = (int)ceilf((heading - span) / 5) * 5; hd <= heading + span; hd += 5) {
        float x = c + (hd - heading) * px_deg;
        int h = ((hd % 360) + 360) % 360;
        hud_seg(x, ty, x, ty - (h % 10 == 0 ? S * 0.014f : S * 0.008f), NULL);
        if (h % 30 == 0) {
            char t[4];
            if (h % 90 == 0) snprintf(t, sizeof t, "%c", "NESW"[h / 90]);
            else snprintf(t, sizeof t, "%02d", h / 10);
            hud_text(&FONT_600_22, t, x, ty - S * 0.032f, GFX_CENTER);
        }
    }
    // Track bug: hollow diamond at the direction of travel, or an arrow at the tape end.
    float to_trk = fmodf(trk - heading + 540, 360) - 180, by = ty - S * 0.05f, bs = S * 0.011f;
    if (fabsf(to_trk) <= span) {
        float bx = c + to_trk * px_deg;
        hud_seg(bx, by - bs, bx + bs, by, NULL); hud_seg(bx + bs, by, bx, by + bs, NULL);
        hud_seg(bx, by + bs, bx - bs, by, NULL); hud_seg(bx - bs, by, bx, by - bs, NULL);
    } else {
        float side = to_trk > 0 ? 1 : -1, ax = c + side * (span * px_deg + S * 0.02f);
        float tri[6] = {ax - side * bs, by - bs, ax + side * bs, by, ax - side * bs, by + bs};
        dl_fill_poly(tri, 3, NULL, NULL, 0, HUD_GREEN, 1);
    }
    hud_seg(c - S * 0.012f, ty + S * 0.022f, c, ty + S * 0.006f, NULL);   // caret
    hud_seg(c, ty + S * 0.006f, c + S * 0.012f, ty + S * 0.022f, NULL);
    char t[16];
    snprintf(t, sizeof t, "%03d", (int)lroundf(heading) % 360);
    hud_box(t, c, ty + S * 0.048f, 0, NULL);
    // Speed (left) and altitude (right) boxes.
    double vn = sqrt(st.now.v[0] * st.now.v[0] + st.now.v[1] * st.now.v[1] + st.now.v[2] * st.now.v[2]);
    double rn = sqrt(st.now.r[0] * st.now.r[0] + st.now.r[1] * st.now.r[1] + st.now.r[2] * st.now.r[2]);
    snprintf(t, sizeof t, "%.2f", vn);
    hud_box(t, S * 0.11f, c, -1, "KM/S");
    snprintf(t, sizeof t, "%.0f", rn - EARTH_R_KM);
    hud_box(t, S * 0.89f, c, 1, "KM");
    draw_prograde();
}

// ---------- HUD ----------
typedef enum { ICON_NONE, ICON_PIN, ICON_DIST, ICON_PASS, ICON_ALT, ICON_ORBIT, ICON_FLAT, ICON_SPIN, ICON_PILOT } icon_t;

static void icon_polyline(const float *p, int n, float w, float a)
{
    for (int i = 1; i < n; i++) dl_line(p[2 * i - 2], p[2 * i - 1], p[2 * i], p[2 * i + 1], w, WHITE, a, true);
}

static void draw_icon(icon_t icon, float x, float y, float h, float a)
{
    switch (icon) {
    case ICON_PIN: {   // map pin
        float r = h * 0.3f, cy = y - h * 0.12f, pts[2 * 32];
        int n = 0;
        for (int i = 0; i <= 28; i++) {
            float t = (float)M_PI * (0.8f + 1.4f * i / 28);
            pts[n++] = x + r * cosf(t); pts[n++] = cy + r * sinf(t);
        }
        pts[n++] = x; pts[n++] = y + h * 0.48f;
        dl_fill_poly(pts, n / 2, NULL, NULL, 0, WHITE, a);
        dl_fill_circle(x, cy, r * 0.4f, RGB(0x11, 0x11, 0x11), a);
        break;
    }
    case ICON_DIST: {   // dimension line |←→| = distance
        float w = h * 0.55f, ah = h * 0.2f, lw = fmaxf(1.5f, h * 0.11f);
        dl_line(x - w, y - h * 0.32f, x - w, y + h * 0.32f, lw, WHITE, a, true);
        dl_line(x + w, y - h * 0.32f, x + w, y + h * 0.32f, lw, WHITE, a, true);
        dl_line(x - w * 0.7f, y, x + w * 0.7f, y, lw, WHITE, a, true);
        float l[6] = {x - w * 0.7f + ah, y - ah, x - w * 0.7f, y, x - w * 0.7f + ah, y + ah};
        float rr[6] = {x + w * 0.7f - ah, y - ah, x + w * 0.7f, y, x + w * 0.7f - ah, y + ah};
        icon_polyline(l, 3, lw, a);
        icon_polyline(rr, 3, lw, a);
        break;
    }
    case ICON_PASS: {   // curve dipping to a marked minimum = time of the next close pass
        float w = h * 0.5f, lw = fmaxf(1.5f, h * 0.11f), by = y + h * 0.22f, p[2 * 17];
        float P0x = x - w, P0y = y - h * 0.48f, P1x = x - w * 0.75f, P1y = by + h * 0.1f;
        float P2x = x + w * 0.75f, P2y = by + h * 0.1f, P3x = x + w, P3y = y - h * 0.48f;
        for (int i = 0; i <= 16; i++) {
            float t = i / 16.0f, u = 1 - t;
            p[2 * i] = u * u * u * P0x + 3 * u * u * t * P1x + 3 * u * t * t * P2x + t * t * t * P3x;
            p[2 * i + 1] = u * u * u * P0y + 3 * u * u * t * P1y + 3 * u * t * t * P2y + t * t * t * P3y;
        }
        icon_polyline(p, 17, lw, a);
        dl_fill_circle(x, by - h * 0.02f, h * 0.19f, WHITE, a);
        break;
    }
    case ICON_ALT: {   // up-triangle over a ground line = altitude
        float w = h * 0.42f;
        float tri[6] = {x, y - h * 0.42f, x + w, y + h * 0.22f, x - w, y + h * 0.22f};
        dl_fill_poly(tri, 3, NULL, NULL, 0, WHITE, a);
        dl_fill_rect(x - w, y + h * 0.34f, 2 * w, fmaxf(1.5f, h * 0.1f), WHITE, a);
        break;
    }
    case ICON_ORBIT: {   // globe with the vertical orbit line and the object at its centre
        float r = h * 0.42f, lw = fmaxf(1.5f, h * 0.1f);
        dl_stroke_circle(x, y, r, lw, WHITE, a);
        dl_line(x, y - r * 1.25f, x, y + r * 1.25f, lw, WHITE, a, false);
        dl_fill_circle(x, y, h * 0.13f, WHITE, a);
        break;
    }
    case ICON_FLAT: {   // disc with polar-grid rings and meridians
        float r = h * 0.45f, lw = fmaxf(1.2f, h * 0.08f);
        dl_stroke_circle(x, y, r, lw, WHITE, a);
        dl_stroke_circle(x, y, r * 0.5f, lw, WHITE, a);
        dl_line(x - r, y, x + r, y, lw, WHITE, a, false);
        dl_line(x, y - r, x, y + r, lw, WHITE, a, false);
        break;
    }
    case ICON_SPIN: {   // globe with its axis and a curved arrow around it
        float r = h * 0.36f, lw = fmaxf(1.2f, h * 0.08f), p[2 * 13];
        dl_stroke_circle(x, y, r, lw, WHITE, a);
        dl_line(x, y - r * 1.45f, x, y + r * 1.45f, lw, WHITE, a, true);
        for (int i = 0; i <= 12; i++) {
            float t = (float)M_PI * (0.15f + 0.7f * i / 12);
            p[2 * i] = x + r * 1.35f * cosf(t); p[2 * i + 1] = y + r * 0.42f * sinf(t);
        }
        icon_polyline(p, 13, lw, a);
        float ax = x - r * 1.2f, ay = y + r * 0.22f, ah = h * 0.14f;
        float tri[6] = {ax - ah, ay - ah * 0.2f, ax + ah * 0.6f, ay - ah, ax + ah * 0.3f, ay + ah * 0.6f};
        dl_fill_poly(tri, 3, NULL, NULL, 0, WHITE, a);
        break;
    }
    case ICON_PILOT: {   // prograde symbol
        float r = h * 0.24f, lw = fmaxf(1.2f, h * 0.09f);
        dl_stroke_circle(x, y, r, lw, WHITE, a);
        dl_line(x - r, y, x - r * 2, y, lw, WHITE, a, true);
        dl_line(x + r, y, x + r * 2, y, lw, WHITE, a, true);
        dl_line(x, y - r, x, y - r * 1.8f, lw, WHITE, a, true);
        break;
    }
    default: break;
    }
}

typedef struct { icon_t icon; const char *value, *unit; } readout_t;
typedef struct { int fs; const gfx_font_t *value, *unit; char widest; } readout_font_t;
static readout_font_t RF_TIME = {24, &FONT_600_24, &FONT_500_12, '0'};
static readout_font_t RF_PRIMARY = {27, &FONT_600_27, &FONT_500_14, '0'};
static readout_font_t RF_SECONDARY = {20, &FONT_600_20, &FONT_500_10, '0'};

static void find_widest(readout_font_t *f)
{
    float best = 0;
    for (char d = '0'; d <= '9'; d++) {
        char s[2] = {d, 0};
        float w = gfx_text_width(f->value, s, 0);
        if (w > best) { best = w; f->widest = d; }
    }
}

// One readout: [icon] VALUE ᵘⁿⁱᵗ starting at x. Returns its width; draw=false only measures.
static float readout(float x, float y, const readout_t *it, const readout_font_t *f, bool draw)
{
    float fs = f->fs, ih = fs * 0.75f, gap = fs * 0.22f;
    float vw = gfx_text_width(f->value, it->value, 0);
    float uw = it->unit ? gfx_text_width(f->unit, it->unit, 0) + fs * 0.06f : 0;
    float iw = it->icon ? ih + gap : 0;
    if (!draw) return iw + vw + uw;
    if (it->icon) draw_icon(it->icon, x + ih / 2, y, ih, 1);
    dl_text(f->value, it->value, x + iw, y, GFX_LEFT, WHITE, 1, 0);
    if (it->unit) dl_text(f->unit, it->unit, x + iw + vw + fs * 0.06f, y - fs * 0.22f, GFX_LEFT, WHITE, 1, 0);
    return iw + vw + uw;
}

// A centred row of readouts in fixed slots (sized for the worst-case value, '#' = widest digit),
// separated by thin vertical rules. Returns the row width.
static float readout_row(const readout_t *items, const readout_t *slots, int n, float y, const readout_font_t *f, bool draw)
{
    float sep = f->fs * 0.9f, widths[4], total = sep * (n - 1);
    for (int i = 0; i < n; i++) {
        char v[16];
        int k = 0;
        for (const char *p = slots[i].value; *p && k < 15; p++) v[k++] = *p == '#' ? f->widest : *p;
        v[k] = 0;
        readout_t slot = slots[i];
        slot.value = v;
        widths[i] = fmaxf(readout(0, 0, &items[i], f, false), readout(0, 0, &slot, f, false));
        total += widths[i];
    }
    if (!draw) return total;
    float x = S / 2.0f - total / 2;
    for (int i = 0; i < n; i++) {
        readout(x, y, &items[i], f, true);
        x += widths[i];
        if (i < n - 1) {
            dl_fill_rect(roundf(x + sep / 2), y - f->fs * 0.38f, 1, f->fs * 0.76f, WHITE, .28f);
            x += sep;
        }
    }
    return total;
}

// canvas arcTo(): rounded corner at p1 between the edges toward p0 and p2, appended as points.
static int arc_to(float *out, int n, float p0x, float p0y, float p1x, float p1y, float p2x, float p2y, float r)
{
    float v1x = p0x - p1x, v1y = p0y - p1y, v2x = p2x - p1x, v2y = p2y - p1y;
    float l1 = hypotf(v1x, v1y), l2 = hypotf(v2x, v2y);
    v1x /= l1; v1y /= l1; v2x /= l2; v2y /= l2;
    float th = acosf(fmaxf(-1, fminf(1, v1x * v2x + v1y * v2y)));
    float d = r / tanf(th / 2), cd = r / sinf(th / 2);
    float bx = v1x + v2x, by = v1y + v2y, bl = hypotf(bx, by);
    float cx = p1x + bx / bl * cd, cy = p1y + by / bl * cd;
    float t1x = p1x + v1x * d, t1y = p1y + v1y * d, t2x = p1x + v2x * d, t2y = p1y + v2y * d;
    float a1 = atan2f(t1y - cy, t1x - cx), a2 = atan2f(t2y - cy, t2x - cx), da = a2 - a1;
    while (da > M_PI) da -= 2 * (float)M_PI;
    while (da < -M_PI) da += 2 * (float)M_PI;
    for (int i = 0; i <= 6; i++) {
        float a = a1 + da * i / 6;
        out[n++] = cx + r * cosf(a); out[n++] = cy + r * sinf(a);
    }
    return n;
}

typedef struct { float y_cut, y_far; } band_t;

// Band fill: rgba(12,13,14) from .78 at the edge to .92 at the screen rim.
static float band_alpha(float y, void *ctx)
{
    const band_t *b = ctx;
    float t = (y - b->y_cut) / (b->y_far - b->y_cut);
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return 0.78f + 0.14f * t;
}

// Where a ray from (px, py) along (dx, dy) (unit) leaves the round screen.
static void ray_to_rim(float px, float py, float dx, float dy, float *x, float *y)
{
    const float c = S / 2.0f;
    float ox = px - c, oy = py - c, b = ox * dx + oy * dy, k = ox * ox + oy * oy - c * c;
    float t = -b + sqrtf(fmaxf(0, b * b - k));
    *x = px + dx * t; *y = py + dy * t;
}

// edgeBand(): tab-shaped dark panel running into the round screen edge (built clipped to it).
static void edge_band(float y_cut, float half_w, bool top)
{
    const float c = S / 2.0f, dir = top ? -1 : 1, rad = S * 0.012f;
    const float fx = cosf(BAND_ANGLE), fy = sinf(BAND_ANGLE) * dir;
    float lx, ly, rx, ry;
    ray_to_rim(c - half_w, y_cut, -fx, fy, &lx, &ly);
    ray_to_rim(c + half_w, y_cut, fx, fy, &rx, &ry);
    float pts[2 * 48];
    int n = 0;
    pts[n++] = lx; pts[n++] = ly;
    n = arc_to(pts, n, lx, ly, c - half_w, y_cut, c + half_w, y_cut, rad);
    n = arc_to(pts, n, c - half_w, y_cut, c + half_w, y_cut, rx, ry, rad);
    pts[n++] = rx; pts[n++] = ry;
    int edge_n = n;
    float a0 = atan2f(ry - c, rx - c), a1 = atan2f(ly - c, lx - c), da = a1 - a0;
    while (da > M_PI) da -= 2 * (float)M_PI;
    while (da < -M_PI) da += 2 * (float)M_PI;
    for (int i = 1; i < 20; i++) {
        float a = a0 + da * i / 20;
        pts[n++] = c + c * cosf(a); pts[n++] = c + c * sinf(a);
    }
    // The band only changes shape with its cut and width: rasterise it once.
    static struct { float y_cut, half_w; gfx_mask_t *m; } cache[2];
    int k = top ? 0 : 1;
    if (!cache[k].m || cache[k].y_cut != y_cut || cache[k].half_w != half_w) {
        band_t b = {y_cut, top ? 0 : S};
        gfx_mask_free(cache[k].m);   // safe: the frame that used it is finished
        cache[k].m = gfx_mask_build(pts, n / 2, band_alpha, &b);
        cache[k].y_cut = y_cut; cache[k].half_w = half_w;
    }
    dl_mask(cache[k].m, RGB(12, 13, 14));
    for (int i = 2; i < edge_n; i += 2)   // hairline along the edge
        dl_thin(pts[i - 2], pts[i - 1], pts[i], pts[i + 1], 1, WHITE, .18f, NULL);
}

static void draw_hud(const snapshot_t *s)
{
    const float pad = S * 0.03f;
    char tbuf[16];
    time_t secs = (time_t)floor(st.t);
    struct tm tm;
    gmtime_r(&secs, &tm);
    snprintf(tbuf, sizeof tbuf, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    readout_t time_row[] = {{ICON_NONE, tbuf, "UTC"}}, time_slots[] = {{ICON_NONE, "##:##:##", "UTC"}};
    st.top_cut = S * 0.068f;
    st.top_half = readout_row(time_row, time_slots, 1, 0, &RF_TIME, false) / 2 + pad;
    edge_band(st.top_cut, st.top_half, true);
    readout_row(time_row, time_slots, 1, S * 0.038f, &RF_TIME, true);

    st.have_bottom = s && s->valid && st.now.ok;
    if (!st.have_bottom) return;
    char range[16], pass[16], lat[16], lon[16], alt[16];
    snprintf(range, sizeof range, "%d", (int)lround(model_range_home(&st.now)));
    if (s->next_pass > 0) {
        time_t pt = (time_t)lround(s->next_pass);
        struct tm lt;
        localtime_r(&pt, &lt);   // TZ = Europe/Berlin
        snprintf(pass, sizeof pass, "%02d:%02d", lt.tm_hour, lt.tm_min);
    } else {
        snprintf(pass, sizeof pass, "--:--");
    }
    // Position from this frame's propagation: the snapshot lags after a clock change.
    double la, lo, h;
    model_geodetic(&st.now, &la, &lo, &h);
    snprintf(lat, sizeof lat, "%.1f°", fabs(la));
    snprintf(lon, sizeof lon, "%.1f°", fabs(lo));
    snprintf(alt, sizeof alt, "%.0f", h);
    readout_t primary[] = {{ICON_DIST, range, "km"}, {ICON_PASS, pass, NULL}};
    readout_t primary_slots[] = {{ICON_DIST, "#####", "km"}, {ICON_PASS, "##:##", NULL}};
    readout_t secondary[] = {{ICON_NONE, lat, la >= 0 ? "N" : "S"}, {ICON_NONE, lon, lo >= 0 ? "E" : "W"},
                             {ICON_ALT, alt, "km"}};
    readout_t secondary_slots[] = {{ICON_NONE, "##.#°", "W"}, {ICON_NONE, "###.#°", "W"}, {ICON_ALT, "#####", "km"}};
    // Fixed tab width: fits the first row at the flat edge and the second row between the flanks.
    const float y_cut = S * 0.88f, y1 = S * 0.912f, y2 = S * 0.957f;
    float w1 = readout_row(primary, primary_slots, 2, 0, &RF_PRIMARY, false);
    float w2 = readout_row(secondary, secondary_slots, 3, 0, &RF_SECONDARY, false);
    st.bottom_cut = y_cut;
    st.bottom_half = fmaxf(w1 / 2 + pad, w2 / 2 + pad * 0.6f - (y2 - y_cut) / tanf(BAND_ANGLE));
    edge_band(y_cut, st.bottom_half, false);
    readout_row(primary, primary_slots, 2, y1, &RF_PRIMARY, true);
    readout_row(secondary, secondary_slots, 3, y2, &RF_SECONDARY, true);
}

// ---------- on-screen menu ----------
static const struct { view_mode_t mode; const char *label; icon_t icon; } MODES[MODE_COUNT] = {
    {MODE_SPIN, "EARTH SPIN", ICON_SPIN},
    {MODE_HOME, "GROUND REFERENCE", ICON_PIN},
    {MODE_ORBIT, "SATELLITE REFERENCE", ICON_ORBIT},
    {MODE_PILOT, "PILOT REFERENCE", ICON_PILOT},
    {MODE_FLAT, "FLAT EARTH", ICON_FLAT},
};

static int round_rect(float *pts, float x, float y, float w, float h, float r)
{
    int n = 0;
    const float cx[4] = {x + w - r, x + w - r, x + r, x + r}, cy[4] = {y + r, y + h - r, y + h - r, y + r};
    for (int k = 0; k < 4; k++)
        for (int i = 0; i <= 6; i++) {
            float a = (float)M_PI * (-0.5f + 0.5f * k + 0.5f * i / 6);
            pts[n++] = cx[k] + r * cosf(a); pts[n++] = cy[k] + r * sinf(a);
        }
    return n / 2;
}

static void draw_menu(void)
{
    const float c = S / 2.0f;
    dl_dim(.45f);   // dim everything behind the menu
    const float w = S * 0.5f, row_h = S * 0.085f, head = S * 0.075f;
    const float h = head + row_h * MODE_COUNT + S * 0.03f, x = c - w / 2, y = c - h / 2;
    float pts[2 * 28];
    int n = round_rect(pts, x, y, w, h, S * 0.02f);
    dl_fill_poly(pts, n, NULL, NULL, 0, RGB(16, 17, 19), .94f);
    dl_stroke_poly(pts, n, 1, WHITE, .18f);
    dl_text(&FONT_500_19, "VIEW", c, y + head * 0.55f, GFX_CENTER, WHITE, .55f, S * 0.004f);
    for (int i = 0; i < MODE_COUNT; i++) {
        float ry = y + head + i * row_h;
        bool active = st.mode == MODES[i].mode;
        if (active) {
            dl_fill_rect(x + S * 0.012f, ry + row_h * 0.08f, w - S * 0.024f, row_h * 0.84f, WHITE, .10f);
            dl_fill_rect(x + S * 0.012f, ry + row_h * 0.08f, fmaxf(2, S * 0.005f), row_h * 0.84f, WHITE, 1);
        }
        float a = active ? 1 : 0.7f;
        draw_icon(MODES[i].icon, x + S * 0.06f, ry + row_h / 2, 27 * 0.8f, a);
        dl_text(&FONT_600_27, MODES[i].label, x + S * 0.1f, ry + row_h / 2, GFX_LEFT, WHITE, a, 0);
        st.menu_hits[i].x = x; st.menu_hits[i].y = ry; st.menu_hits[i].w = w; st.menu_hits[i].h = row_h;
        st.menu_hits[i].mode = MODES[i].mode;
    }
}

// ---------- view targets, easing, pointer ----------
// targetRotation(): the orientation the current mode wants.
static bool target_rotation(float out[3])
{
    switch (st.mode) {
    case MODE_HOME: out[0] = -HOME_LON; out[1] = -HOME_LAT; out[2] = 0; return true;
    case MODE_SPIN: {
        // Seen from the equatorial plane, north up, turning about the real axis: the surface
        // drifts left to right (eastward rotation), one turn per SPIN_PERIOD_S.
        double t = (st.now_ms - st.spin_t0) / 1000.0;
        out[0] = wrap180((float)fmod(-HOME_LON + 360 * t / SPIN_PERIOD_S, 360.0));
        out[1] = 0; out[2] = 0;
        return true;
    }
    case MODE_FLAT: out[0] = 0; out[1] = -90; out[2] = 0; return true;
    case MODE_PILOT: out[0] = out[1] = out[2] = 0; return true;   // no look-around offset
    default: break;
    }
    if (!st.now.ok || !st.ahead.ok) return false;
    double p[3], q[3];
    double pr = model_geo(st.now.r, st.now.gmst, p), qr = model_geo(st.ahead.r, st.now.gmst, q);
    float plon = (float)(atan2(p[1], p[0]) * DEG), plat = (float)(atan2(p[2], hypot(p[0], p[1])) * DEG);
    // Centre on the object, then spin the view so the flight direction points up.
    view_t v = {.kind = GLOBE_ORTHO, .rot = {-plon, -plat, 0}};
    v.M = view_matrix(v.rot);
    proj_t a = project3d(&v, q, qr), o = project3d(&v, p, pr);
    out[0] = -plon; out[1] = -plat; out[2] = (float)(atan2(a.x - o.x, -(a.y - o.y)) * DEG);
    return true;
}

// easeToTarget(): time-based glide, then lock on (short way round for λ and γ).
static void ease_to_target(float dt)
{
    float t[3];
    st.gliding = false;
    if (!target_rotation(t)) return;
    float d[3] = {wrap180(t[0] - st.rotate[0]), t[1] - st.rotate[1], wrap180(t[2] - st.rotate[2])};
    if (fabsf(d[0]) < 0.005f && fabsf(d[1]) < 0.005f && fabsf(d[2]) < 0.005f) {
        memcpy(st.rotate, t, sizeof t);
        return;
    }
    bool close = fabsf(d[0]) < 0.5f && fabsf(d[1]) < 0.5f && fabsf(d[2]) < 0.5f;
    st.gliding = !close;
    float k = close ? 1 : 1 - expf(-fminf(0.5f, dt) / GLIDE_TAU_S);
    st.rotate[0] = wrap180(st.rotate[0] + d[0] * k);
    st.rotate[1] += d[1] * k;
    st.rotate[2] = wrap180(st.rotate[2] + d[2] * k);
}

static void set_mode(view_mode_t mode)
{
    globe_kind_t was = mode_kind(st.mode);
    if (mode == MODE_SPIN && st.mode != MODE_SPIN) st.spin_t0 = st.now_ms;
    st.mode = mode;
    st.manual_until = 0;
    st.vel[0] = st.vel[1] = 0;
    if (was != mode_kind(mode)) {   // no glide across different kinds of view
        float t[3];
        if (target_rotation(t)) memcpy(st.rotate, t, sizeof t);
    }
}

// resetView(): drop any manual pan and glide back to the mode's default orientation.
static void reset_view(void)
{
    st.manual_until = 0;
    st.vel[0] = st.vel[1] = 0;
}

// d3 "versor" quaternion helpers for the trackball.
static void versor(const float r[3], float q[4])
{
    float l = r[0] / (2 * (float)DEG), p = r[1] / (2 * (float)DEG), g = r[2] / (2 * (float)DEG);
    float sl = sinf(l), cl = cosf(l), sp = sinf(p), cp = cosf(p), sg = sinf(g), cg = cosf(g);
    q[0] = cl * cp * cg + sl * sp * sg;
    q[1] = sl * cp * cg - cl * sp * sg;
    q[2] = cl * sp * cg + sl * cp * sg;
    q[3] = cl * cp * sg - sl * sp * cg;
}

static void versor_mul(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void versor_rot(const float q[4], float r[3])
{
    r[0] = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * (float)DEG;
    r[1] = asinf(fmaxf(-1, fminf(1, 2 * (q[0] * q[2] - q[3] * q[1])))) * (float)DEG;
    r[2] = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * (float)DEG;
}

static float screen_angle(float x, float y) { return atan2f(x - CX, -(CY - y)); }

// rotateBy(): dx / dy are the screen drag in degrees (x right, y down). Globe views turn as a
// trackball about the screen's own axes, so the surface follows the finger at any orientation;
// the pilot view yaws / pitches the look direction; the flat map spins about the pole.
static void rotate_by(float dx, float dy)
{
    if (st.mode == MODE_PILOT) {
        st.rotate[0] = wrap180(st.rotate[0] + dx);
        st.rotate[1] = fmaxf(-90, fminf(90, st.rotate[1] - dy));
    } else if (st.mode == MODE_FLAT) {
        st.rotate[0] = wrap180(st.rotate[0] + dx);
    } else {
        float a[4], b[4], q[4], r[3] = {dx, -dy, 0};
        versor(r, a);
        versor(st.rotate, b);
        versor_mul(a, b, q);
        versor_rot(q, st.rotate);
    }
}

// onTap(): menu choice, menu open (bottom tab) or view reset (top tab).
static void on_tap(int px, int py)
{
    const float c = S / 2.0f;
    if (st.menu_open) {
        for (int i = 0; i < MODE_COUNT; i++)
            if (px >= st.menu_hits[i].x && px <= st.menu_hits[i].x + st.menu_hits[i].w &&
                py >= st.menu_hits[i].y && py <= st.menu_hits[i].y + st.menu_hits[i].h)
                set_mode(st.menu_hits[i].mode);
        st.menu_open = false;   // any tap closes: selection or outside
        return;
    }
    if (hypotf(px - c, py - c) > c) return;
    float depth_b = py - st.bottom_cut, depth_t = st.top_cut - py;
    if (st.have_bottom && depth_b >= 0 && fabsf(px - c) <= st.bottom_half + depth_b / tanf(BAND_ANGLE))
        st.menu_open = true;
    else if (depth_t >= 0 && fabsf(px - c) <= st.top_half + depth_t / tanf(BAND_ANGLE))
        reset_view();
}

void scene_touch(bool pressed, int x, int y, int64_t now_ms)
{
    if (pressed && !st.down) {
        st.down = true;
        st.moved = false;
        st.x0 = st.lx = x; st.y0 = st.ly = y;
        st.move_ms = now_ms;
        st.vel[0] = st.vel[1] = 0;
    } else if (pressed) {
        if (st.menu_open) return;
        if (!st.moved && hypotf(x - st.x0, y - st.y0) < TAP_SLOP_PX) return;
        st.moved = true;
        float dt = fmaxf(0.005f, (now_ms - st.move_ms) / 1000.0f);
        float dx, dy;
        if (st.mode == MODE_FLAT) {   // follow the finger's angle around the pole
            float d = screen_angle(x, y) - screen_angle(st.lx, st.ly);
            while (d > M_PI) d -= 2 * (float)M_PI;
            while (d < -M_PI) d += 2 * (float)M_PI;
            dx = d * (float)DEG; dy = 0;
        } else {
            const float k = (float)DEG / R;   // one disc radius ≈ 57°
            dx = (x - st.lx) * k; dy = (y - st.ly) * k;
        }
        rotate_by(dx, dy);
        st.vel[0] = dx / dt; st.vel[1] = dy / dt;
        st.lx = x; st.ly = y;
        st.move_ms = now_ms;
    } else if (st.down) {
        st.down = false;
        if (!st.moved) {
            on_tap(st.lx, st.ly);
            // Double tap (dblclick in the web app): reset the view.
            if (now_ms - st.last_tap_ms < 350 && abs(st.lx - st.tap_x) < 30 && abs(st.ly - st.tap_y) < 30) reset_view();
            st.last_tap_ms = now_ms;
            st.tap_x = st.lx; st.tap_y = st.ly;
        } else {
            if (now_ms - st.move_ms > 100) st.vel[0] = st.vel[1] = 0;   // finger rested before lifting
            st.manual_until = now_ms + 4000;   // then glide back to the mode's view
        }
    }
}

static bool panning(void) { return (st.down && st.moved) || st.vel[0] != 0 || st.vel[1] != 0; }

void scene_update(double unix_s, float dt, int64_t now_ms)
{
    st.t = unix_s;
    st.now_ms = now_ms;
    const snapshot_t *s = model_snapshot();
    if (!s || fabs(unix_s - s->t) >= 1.0) model_request(unix_s);
    st.now = model_propagate(unix_s);
    st.ahead = model_propagate(unix_s + 5);
    if (!st.aimed && st.now.ok) {
        float t[3];
        if (target_rotation(t)) { memcpy(st.rotate, t, sizeof t); st.aimed = true; }
    }
    // Pan inertia: 0.92 per 60 Hz frame, stops below 0.01° per frame.
    if (!st.down && (st.vel[0] != 0 || st.vel[1] != 0)) {
        rotate_by(st.vel[0] * dt, st.vel[1] * dt);
        float decay = powf(0.92f, dt * 60);
        st.vel[0] *= decay; st.vel[1] *= decay;
        if (fabsf(st.vel[0]) < 0.6f && fabsf(st.vel[1]) < 0.6f) st.vel[0] = st.vel[1] = 0;
    }
    if (!st.down && now_ms > st.manual_until) ease_to_target(dt);
    else st.gliding = false;
    if (st.mode == MODE_PILOT) update_camera();
}

void scene_clock_changed(double unix_s)
{
    model_request(unix_s);
    st.aimed = false;
    st.manual_until = 0;
}

// ---------- frame ----------
static float view_diff(const globe_view_t *a, const globe_view_t *b)
{
    float d = fabsf(wrap180(a->rot[0] - b->rot[0]));
    return fmaxf(d, fmaxf(fabsf(a->rot[1] - b->rot[1]), fabsf(wrap180(a->rot[2] - b->rot[2]))));
}

// Still orthographic views (ground and satellite reference) use the full-resolution render from
// the background task while it is within SHOW_DEG of the wanted view, refreshed past REFRESH_DEG.
#define REFRESH_DEG 0.05f
#define SHOW_DEG 0.25f
#define REFRESH_S 30

typedef struct {
    uint16_t *fb;
    view_t v;
    globe_view_t gv;
    const uint16_t *base;   // cached full frame, or NULL: render the view's pixels per strip
    bool half;
    const snapshot_t *snap;
} frame_job_t;

// A finished strip (rows y0 .. y0 + n) into the frame buffer, turned 180° if the board is
// mounted that way: each row is read backwards from the (internal RAM) strip, its pixel pairs
// swapped, and written forward to its mirrored row (writing PSRAM backwards pixel by pixel is
// several times slower).
static inline uint32_t swap16(uint32_t v) { return v >> 16 | v << 16; }

// (in internal RAM like the other per-pixel code: scene.c runs from flash)
static IRAM_ATTR void strip_out(uint16_t *fb, uint16_t *sb, int y0, int n)
{
#if GFX_ROT180
    for (int r = 0; r < n; r++) {
        const uint32_t *src = (const uint32_t *)(sb + r * S) + S / 2;
        uint32_t *dst = (uint32_t *)(fb + (S - 1 - y0 - r) * S), *end = dst + S / 2;
        for (; dst < end; dst += 4, src -= 4) {
            uint32_t a = src[-1], b = src[-2], c = src[-3], d = src[-4];
            dst[0] = swap16(a); dst[1] = swap16(b); dst[2] = swap16(c); dst[3] = swap16(d);
        }
    }
#else
    memcpy(fb + y0 * S, sb, n * S * 2);
#endif
}

// One strip buffer in internal RAM per core.
static uint16_t *strip_buf[2];

// Borders and graticule, into the display list or (bake) straight into a frame.
static void surface_lines(const view_t *v, gfx_ctx_t *bake)
{
    static const gfx_dash_t border_dash = {3, 2, 0};
    const stroke_t bs = {0.7f, RGB(20, 20, 20), .5f, &border_dash, bake, bake ? 0 : dl_hair_set(0.7f, .5f, 3, 2) + 1};
    draw_lineset(v, &borders, &bs);
    const stroke_t gs = {0.6f, WHITE, .16f, NULL, bake, bake ? 0 : dl_hair_set(0.6f, .16f, 0, 0) + 1};
    draw_lineset(v, &graticule, &gs);
}

// The background task's full-resolution globe gets the lines printed on it, so still frames
// that show it neither record nor draw them.
static gfx_ctx_t *bake_ctx;

static void bake_lines(uint16_t *fb, const globe_view_t *gv)
{
    view_t v = {0};
    v.kind = gv->kind;
    memcpy(v.rot, gv->rot, sizeof v.rot);
    v.M = view_matrix(gv->rot);
    gfx_ctx_target(bake_ctx, fb);
    surface_lines(&v, bake_ctx);
}

// Record the frame (part 0: borders and graticule, or the rose when the lines are baked into
// the cached globe; part 1: everything above them).
static void record(int part, const void *vctx)
{
    const frame_job_t *j = vctx;
    const view_t *v = &j->v;
    if (part == 0) {
        // (a cached full-resolution globe has them printed on it already); the rose above them,
        // here when the lines are not drawn (it balances the two cores' share of the recording)
        if (v->kind != GLOBE_PILOT && !j->base) surface_lines(v, NULL);
        if (v->kind != GLOBE_PILOT && j->base) draw_rose(v);
        return;
    }
    const snapshot_t *s = j->snap;
    if (v->kind == GLOBE_PILOT) {
        if (cam.ok) {
            draw_home(v);
            draw_pilot_track(s);
            draw_pilot_hud();
        }
    } else {
        project_track(v, s);
        if (!j->base) draw_rose(v);
        if (s) draw_behind(v, s);
        if (v->kind != GLOBE_FLAT) draw_reticle();
        draw_home(v);
        draw_track();
        draw_mark_ticks(v, s);
        draw_satellite(v);
    }
    draw_hud(s);
    if (st.menu_open) draw_menu();
}

// Time per core in the last frame's strips: globe pixels, vector replay, copy out (µs).
static int64_t strip_us[2][3];

// The frame in strips of internal RAM, each core taking every other strip.
static void frame_strips(int part, const void *vctx)
{
    const frame_job_t *j = vctx;
    uint16_t *sb = strip_buf[part];
    if (!sb) return;   // out of internal RAM at start-up: skip rather than write through NULL
    int64_t *acc = strip_us[part];
    acc[0] = acc[1] = acc[2] = 0;
    for (int k = part; k < DL_NSTRIPS; k += 2) {
        int y0 = k * DL_STRIP_H, y1 = y0 + DL_STRIP_H < S ? y0 + DL_STRIP_H : S;
        uint16_t *fbv = sb - y0 * S;   // frame coordinates for this strip's rows
        int64_t t0 = esp_timer_get_time();
        if (j->base) memcpy(sb, j->base + y0 * S, (y1 - y0) * S * 2);
        else globe_rows(fbv, &j->gv, y0, y1, j->half);
        int64_t t1 = esp_timer_get_time();
        gfx_target(fbv);
        gfx_clip_rows(y0, y1);
        dl_replay(k);
        int64_t t2 = esp_timer_get_time();
        strip_out(j->fb, sb, y0, y1 - y0);
        int64_t t3 = esp_timer_get_time();
        acc[0] += t1 - t0; acc[1] += t2 - t1; acc[2] += t3 - t2;
    }
    gfx_clip_rows(0, S);
}

void scene_render(uint16_t *fb, scene_timing_t *tm)
{
    int64_t t0 = esp_timer_get_time();
    static frame_job_t job;
    job.fb = fb;
    job.snap = model_snapshot();
    job.base = NULL;
    job.half = false;
    globe_view_t *gv = &job.gv;
    gv->kind = mode_kind(st.mode);
    memcpy(gv->rot, st.rotate, sizeof gv->rot);
    model_sun(st.t, gv->sun);
    if (gv->kind == GLOBE_PILOT) {
        memcpy(gv->cam.c, cam.c, sizeof cam.c);
        memcpy(gv->cam.f, cam.f, sizeof cam.f);
        memcpy(gv->cam.u, cam.u, sizeof cam.u);
        memcpy(gv->cam.rt, cam.rt, sizeof cam.rt);
        gv->cam.gmst = cam.gmst;
        tiles_frame();
    }
    // Moving: a finger on it, inertia, or a glide faster than the views' own slow drift (the
    // Earth spin turns 6°/s, the ground track a fraction of that): those frames are drawn at
    // half resolution to keep the frame rate up.
    static float prev_rot[3];
    static int64_t prev_us;
    static bool fast;
    float dts = (t0 - prev_us) / 1e6f, speed = 0;
    if (dts > 0 && dts < 1) {
        float d = fmaxf(fabsf(wrap180(st.rotate[0] - prev_rot[0])),
                        fmaxf(fabsf(st.rotate[1] - prev_rot[1]), fabsf(wrap180(st.rotate[2] - prev_rot[2]))));
        speed = d / dts;
    }
    memcpy(prev_rot, st.rotate, sizeof prev_rot);
    prev_us = t0;
    fast = speed > (fast ? 9.0f : 12.0f);
    const bool moving = panning() || (st.gliding && fast);
    bool held = false;
    if (gv->kind == GLOBE_PILOT) {
        job.half = true;
    } else if ((gv->kind == GLOBE_ORTHO && !globe_is_equatorial(gv)) || gv->kind == GLOBE_FLAT) {
        if (moving) {
            globe_cache_cancel();
            job.half = true;
        } else {
            globe_view_t cv;
            double ct;
            const uint16_t *cache = globe_cache_hold(&cv, &ct);
            float diff = cache && cv.kind == gv->kind ? view_diff(&cv, gv) : 1e9f;
            if (!(diff <= REFRESH_DEG && fabs(st.t - ct) < REFRESH_S) && !globe_cache_busy()) globe_cache_request(gv, st.t);
            if (diff <= SHOW_DEG && fabs(st.t - ct) < 4 * REFRESH_S) {
                job.base = cache;
                *gv = cv;   // draw everything with the rotation the cache was rendered with
                held = true;
            } else {
                if (cache) globe_cache_release();
                job.half = true;
            }
        }
    } else if (moving) {
        job.half = true;
    }
    job.v.kind = gv->kind;
    memcpy(job.v.rot, gv->rot, sizeof job.v.rot);
    job.v.M = view_matrix(gv->rot);
    job.v.cam = cam;

    if (st.now.ok) { frame_gc = (float)cos(st.now.gmst); frame_gs = (float)sin(st.now.gmst); }
    dl_begin();
    globe_both_cores(record, &job);
    dl_finish();
    int64_t t1 = esp_timer_get_time();
    globe_both_cores(frame_strips, &job);
    if (held) globe_cache_release();
    int64_t t2 = esp_timer_get_time();
    if (tm) {
        tm->base_ms = (t1 - t0) / 1000.0f;    // record the display list
        tm->lines_ms = (t2 - t1) / 1000.0f;   // strips: pixels, replay, copy out
        tm->overlay_ms = 0;
        int busy = strip_us[1][0] + strip_us[1][1] + strip_us[1][2] > strip_us[0][0] + strip_us[0][1] + strip_us[0][2];
        tm->px_ms = strip_us[busy][0] / 1000.0f;
        tm->replay_ms = strip_us[busy][1] / 1000.0f;
        tm->copy_ms = strip_us[busy][2] / 1000.0f;
        tm->total_ms = (t2 - t0) / 1000.0f;
        tm->half_res = job.half;
    }
}

scene_sat_t scene_sat(void)
{
    const snapshot_t *s = model_snapshot();
    scene_sat_t r = {0};
    if (s && s->valid) { r.valid = true; r.lat = s->lat; r.lon = s->lon; r.alt = s->alt; r.vel = s->vel; }
    return r;
}

void scene_rotation(float out[3]) { memcpy(out, st.rotate, sizeof st.rotate); }
const char *scene_mode_name(void) { return MODE_NAMES[st.mode]; }
void scene_globe_stats(int *count, float *avg_ms) { globe_cache_stats(count, avg_ms); }

void scene_init(void)
{
    // Munich local time for the next-pass readout (Europe/Berlin).
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    gfx_init();
    dl_init();
    model_init();
    globe_init();
    borders = load_lines(borders_bin_start);
    graticule = load_lines(graticule_bin_start);
    rose_init();
    bake_ctx = gfx_ctx_new(NULL, 0, S);
    if (bake_ctx) globe_cache_bake(bake_lines);
    find_widest(&RF_TIME);
    find_widest(&RF_PRIMARY);
    find_widest(&RF_SECONDARY);
    st.spin_t0 = esp_timer_get_time() / 1000;   // boot into Earth spin
    scene_reserve_strips();
    ESP_LOGI("scene", "internal RAM free %u after strips", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

bool scene_strips_fast(void)
{
    for (int i = 0; i < 2; i++)
        if (!strip_buf[i] || !esp_ptr_dma_capable(strip_buf[i])) return false;
    return true;
}

void scene_reserve_strips(void)
{
    // In the main SRAM (L2MEM, the DMA-capable internal RAM). "Internal" alone also admits the
    // P4's low-power SRAM, which is several times slower: a strip there quarters the frame rate,
    // and the allocator picks it as soon as the main SRAM has no contiguous block left. So the
    // strips are reserved first thing at boot, and a fallback is reported, not silent.
    for (int i = 0; i < 2; i++) {
        if (strip_buf[i]) continue;
        strip_buf[i] = heap_caps_aligned_alloc(16, DL_STRIP_H * S * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (!strip_buf[i]) {
            strip_buf[i] = heap_caps_aligned_alloc(16, DL_STRIP_H * S * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            ESP_LOGE("scene", "strip buffer %d not in main SRAM (%p): rendering will be slow", i, strip_buf[i]);
        }
    }
}

bool scene_background_busy(void) { return globe_cache_busy(); }
