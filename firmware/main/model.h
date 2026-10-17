/* Orbit data of web/app.js: SGP4 positions, the ±1 h track, 1-minute marks, the
 * HUD readouts and the next pass over Munich. The slow parts (track, marks, pass
 * search) run once a second in a low-priority task and are published as snapshots. */
#pragma once
#include <stdbool.h>
#include "orbit.h"

#define EARTH_R_KM 6371.0
#define TRACK_SPAN_S 3600
#define TRACK_STEP_S 30
#define TRACK_N (2 * TRACK_SPAN_S / TRACK_STEP_S + 1)
#define MARK_STEP_S 60
#define MARK_MAJOR 5          // every 5th mark (whole 5 minutes) is a longer tick
#define MARK_MAX (2 * TRACK_SPAN_S / MARK_STEP_S + 2)
#define DEG (180.0 / M_PI)

#define HOME_LON 11.575
#define HOME_LAT 48.137
#define HOME_NAME "MUNICH"

typedef struct { double r[3], v[3]; double gmst; bool ok; } eci_t;
// A propagated point: ECI direction (drawn in the frame of the current sidereal angle in the
// satellite and pilot views: the inertial orbit) and Earth-fixed direction at its own time
// (the ground-relative path), radius in km.
typedef struct { float eci[3], ecef[3]; double rk; } orbpt_t;
typedef struct { int t; orbpt_t p; } trackpt_t;
// A 1-minute mark at an absolute time, and the point 10 s later (tick direction).
typedef struct { double t; bool major; orbpt_t a, b; } mark_t;

typedef struct {
    bool valid;
    double t;                         // the time this snapshot was computed for
    trackpt_t track[TRACK_N];
    int ntrack;
    mark_t marks[MARK_MAX];
    int nmarks;
    double lat, lon, alt, vel;        // geodetic (satellite.eciToGeodetic), for the HUD
    double next_pass;                 // Unix time of the next range minimum to Munich, 0 = none in 4 h
} snapshot_t;

void model_init(void);
// Replace the orbital elements at runtime (from a network refresh) and persist them to NVS.
// Ignored unless the elements are for this device's compiled-in NORAD id. Thread-safe.
bool model_set_omm(const omm_t *omm);
// Wake the worker to compute a snapshot for time t (no-op while one is in progress).
void model_request(double t);
// Latest published snapshot (stable for at least a second after it is returned).
const snapshot_t *model_snapshot(void);

eci_t model_propagate(double t);
// Earth-fixed unit vector of an ECI position in the frame of sidereal angle gmst; returns |r| (km).
double model_geo(const double r[3], double gmst, double u[3]);
// Geodetic latitude, longitude (degrees) and height (km) of a propagated position.
void model_geodetic(const eci_t *e, double *lat, double *lon, double *alt);
// Straight-line distance (km) from Munich at ground level.
double model_range_home(const eci_t *e);
// Unit vector toward the sun in Earth-fixed axes (subsolarPoint()).
void model_sun(double t, float out[3]);
