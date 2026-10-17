#include <math.h>
#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "model.h"
#include "orbit.h"
#include "generated/elements.h"

static const char *TAG = "model";

#define PASS_SEARCH_STEP_S 20
#define PASS_SEARCH_SPAN_S (4 * 3600)

static snapshot_t snaps[2];
static volatile int published = -1;
static volatile double requested_t;
static volatile bool busy;
static TaskHandle_t worker;
static double home_ecf[3];
// Serialises SGP4 state (re-init vs. the per-call propagation on the frame loop and the worker).
static SemaphoreHandle_t orbit_lock;

eci_t model_propagate(double t)
{
    eci_t e = {.ok = false};
    xSemaphoreTake(orbit_lock, portMAX_DELAY);
    bool ok = orbit_propagate(t, e.r, e.v);
    xSemaphoreGive(orbit_lock);
    if (ok) { e.gmst = orbit_gmst(t); e.ok = true; }
    return e;
}

static void orbpt(const eci_t *e, orbpt_t *o)
{
    double u[3];
    o->rk = model_geo(e->r, e->gmst, u);
    for (int k = 0; k < 3; k++) { o->eci[k] = (float)(e->r[k] / o->rk); o->ecef[k] = (float)u[k]; }
}

double model_geo(const double r[3], double gmst, double u[3])
{
    double n = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    double c = cos(gmst), s = sin(gmst);
    u[0] = (r[0] * c + r[1] * s) / n;
    u[1] = (-r[0] * s + r[1] * c) / n;
    u[2] = r[2] / n;
    return n;
}

// satellite.js WGS-84 ellipsoid
static const double WGS_A = 6378.137, WGS_B = 6356.7523142;

static void geodetic_to_ecf(double lon, double lat, double h, double out[3])
{
    double f = (WGS_A - WGS_B) / WGS_A, e2 = 2 * f - f * f;
    double n = WGS_A / sqrt(1 - e2 * sin(lat) * sin(lat));
    out[0] = (n + h) * cos(lat) * cos(lon);
    out[1] = (n + h) * cos(lat) * sin(lon);
    out[2] = (n * (1 - e2) + h) * sin(lat);
}

// satellite.eciToGeodetic()
static void eci_to_geodetic(const double r[3], double gmst, double *lat, double *lon, double *h)
{
    double f = (WGS_A - WGS_B) / WGS_A, e2 = 2 * f - f * f;
    double R2 = sqrt(r[0] * r[0] + r[1] * r[1]);
    double lo = atan2(r[1], r[0]) - gmst;
    while (lo < -M_PI) lo += 2 * M_PI;
    while (lo > M_PI) lo -= 2 * M_PI;
    double la = atan2(r[2], R2), C = 1;
    for (int k = 0; k < 20; k++) {
        C = 1 / sqrt(1 - e2 * sin(la) * sin(la));
        la = atan2(r[2] + WGS_A * C * e2 * sin(la), R2);
    }
    *lat = la * DEG;
    *lon = lo * DEG;
    *h = R2 / cos(la) - WGS_A * C;
}

void model_geodetic(const eci_t *e, double *lat, double *lon, double *alt)
{
    eci_to_geodetic(e->r, e->gmst, lat, lon, alt);
}

double model_range_home(const eci_t *e)
{
    double c = cos(e->gmst), s = sin(e->gmst);
    double x = e->r[0] * c + e->r[1] * s, y = -e->r[0] * s + e->r[1] * c, z = e->r[2];
    double dx = x - home_ecf[0], dy = y - home_ecf[1], dz = z - home_ecf[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static double range_at(double t)
{
    eci_t e = model_propagate(t);
    return e.ok ? model_range_home(&e) : INFINITY;
}

// findNextPass(): first local minimum of the range to Munich, coarse scan + ternary search.
static double find_next_pass(double t0)
{
    double prev = range_at(t0), cur = range_at(t0 + PASS_SEARCH_STEP_S);
    for (int dt = 2 * PASS_SEARCH_STEP_S; dt <= PASS_SEARCH_SPAN_S; dt += PASS_SEARCH_STEP_S) {
        double next = range_at(t0 + dt);
        if (cur <= prev && cur < next) {
            double lo = t0 + dt - 2 * PASS_SEARCH_STEP_S, hi = t0 + dt;
            while (hi - lo > 1) {
                double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
                if (range_at(m1) < range_at(m2)) hi = m2; else lo = m1;
            }
            return (lo + hi) / 2;
        }
        prev = cur;
        cur = next;
    }
    return 0;
}

// Approximate subsolar point (good to ~0.5°), as a unit vector.
void model_sun(double t, float out[3])
{
    double d = (t - 946728000.0) / 86400.0;   // days since 2000-01-01 12:00 UTC
    double g = (357.529 + 0.98560028 * d) / DEG;
    double q = 280.459 + 0.98564736 * d;
    double L = (q + 1.915 * sin(g) + 0.020 * sin(2 * g)) / DEG;
    double e = (23.439 - 0.00000036 * d) / DEG;
    double ra = atan2(cos(e) * sin(L), cos(L));
    double dec = asin(sin(e) * sin(L));
    double lon = ra - orbit_gmst(t);
    out[0] = (float)(cos(dec) * cos(lon));
    out[1] = (float)(cos(dec) * sin(lon));
    out[2] = (float)sin(dec);
}

// updateTrack() and friends for time t.
static void compute(snapshot_t *s, double t)
{
    s->t = t;
    s->ntrack = 0;
    for (int dt = -TRACK_SPAN_S; dt <= TRACK_SPAN_S; dt += TRACK_STEP_S) {
        eci_t p = model_propagate(t + dt);
        if (!p.ok) continue;
        trackpt_t *q = &s->track[s->ntrack++];
        q->t = dt;
        orbpt(&p, &q->p);
    }
    // 1-minute marks pinned to absolute times (so they don't jump as "now" advances).
    s->nmarks = 0;
    double first = ceil((t - TRACK_SPAN_S) / MARK_STEP_S) * MARK_STEP_S;
    for (double mt = first; mt <= t + TRACK_SPAN_S && s->nmarks < MARK_MAX; mt += MARK_STEP_S) {
        eci_t a = model_propagate(mt), b = model_propagate(mt + 10);
        if (!a.ok || !b.ok) continue;
        mark_t *m = &s->marks[s->nmarks++];
        m->t = mt - t;
        m->major = ((long long)llround(mt / MARK_STEP_S)) % MARK_MAJOR == 0;
        orbpt(&a, &m->a);
        orbpt(&b, &m->b);
    }
    s->next_pass = find_next_pass(t);
    double r[3], v[3];
    xSemaphoreTake(orbit_lock, portMAX_DELAY);
    s->valid = orbit_propagate(t, r, v);
    xSemaphoreGive(orbit_lock);
    if (s->valid) {
        eci_to_geodetic(r, orbit_gmst(t), &s->lat, &s->lon, &s->alt);
        s->vel = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
}

static void worker_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int slot = published == 0 ? 1 : 0;
        compute(&snaps[slot], requested_t);
        published = slot;
        busy = false;
    }
}

void model_request(double t)
{
    if (busy) return;
    busy = true;
    requested_t = t;
    xTaskNotifyGive(worker);
}

const snapshot_t *model_snapshot(void)
{
    return published < 0 ? NULL : &snaps[published];
}

#define OMM_NVS_NS "orbit"
#define OMM_NVS_KEY "omm"

// Load elements saved by a previous network refresh (same NORAD id only). Returns false if
// none, invalid, or for a different object, so the compiled-in elements are used instead.
static bool load_saved_omm(omm_t *out)
{
    nvs_handle_t h;
    if (nvs_open(OMM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t n = sizeof *out;
    bool ok = nvs_get_blob(h, OMM_NVS_KEY, out, &n) == ESP_OK && n == sizeof *out &&
              out->norad_cat_id == OMM_NORAD_CAT_ID && out->mean_motion > 0;
    nvs_close(h);
    return ok;
}

bool model_set_omm(const omm_t *omm)
{
    if (omm->norad_cat_id != OMM_NORAD_CAT_ID || !(omm->mean_motion > 0)) return false;   // not our object
    xSemaphoreTake(orbit_lock, portMAX_DELAY);
    bool ok = orbit_init(omm);
    xSemaphoreGive(orbit_lock);
    if (!ok) return false;
    nvs_handle_t h;
    if (nvs_open(OMM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, OMM_NVS_KEY, omm, sizeof *omm);
        nvs_commit(h);
        nvs_close(h);
    }
    model_request((double)time(NULL));   // recompute the track with the new elements
    ESP_LOGI(TAG, "elements updated (epoch %.0f UTC) and saved", omm->epoch_unix);
    return true;
}

void model_init(void)
{
    orbit_lock = xSemaphoreCreateMutex();
    omm_t omm = {
        .epoch_unix = OMM_EPOCH_UNIX, .mean_motion = OMM_MEAN_MOTION, .eccentricity = OMM_ECCENTRICITY,
        .inclination = OMM_INCLINATION, .ra_of_asc_node = OMM_RA_OF_ASC_NODE,
        .arg_of_pericenter = OMM_ARG_OF_PERICENTER, .mean_anomaly = OMM_MEAN_ANOMALY, .bstar = OMM_BSTAR,
        .mean_motion_dot = OMM_MEAN_MOTION_DOT, .mean_motion_ddot = OMM_MEAN_MOTION_DDOT,
        .norad_cat_id = OMM_NORAD_CAT_ID,
    };
    bool saved = load_saved_omm(&omm);
    if (!orbit_init(&omm)) ESP_LOGE(TAG, "SGP4 init failed for %d", OMM_NORAD_CAT_ID);
    ESP_LOGI(TAG, "%s (%s), elements epoch %s%s", OMM_OBJECT_NAME, OMM_OBJECT_ID,
             saved ? "from saved refresh" : OMM_EPOCH_STR, saved ? "" : " UTC");
    geodetic_to_ecf(HOME_LON / DEG, HOME_LAT / DEG, 0, home_ecf);
    // Below the frame loop and the globe renders; it needs ~1 s of slack per second at most.
    xTaskCreatePinnedToCore(worker_task, "orbit", 6144, NULL, 1, &worker, 1);
}
