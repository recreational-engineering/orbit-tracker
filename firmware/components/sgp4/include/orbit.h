// C interface to Vallado's SGP4 (SGP4.cpp), mirroring satellite.js as used by web/app.js.
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// OMM mean elements as CelesTrak publishes them (angles in degrees, mean motion in rev/day).
typedef struct {
    double epoch_unix;      // EPOCH as Unix seconds (UTC)
    double mean_motion, eccentricity, inclination, ra_of_asc_node;
    double arg_of_pericenter, mean_anomaly, bstar, mean_motion_dot, mean_motion_ddot;
    int norad_cat_id;
} omm_t;

// Like satellite.json2satrec(): WGS-72, improved ('i') operation mode.
bool orbit_init(const omm_t *omm);

// Like satellite.propagate(): TEME position (km) and velocity (km/s) at a Unix time.
bool orbit_propagate(double unix_s, double r[3], double v[3]);

// Like satellite.gstime(): Greenwich mean sidereal time (rad) at a Unix time.
double orbit_gmst(double unix_s);

#ifdef __cplusplus
}
#endif
