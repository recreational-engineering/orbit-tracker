#include <math.h>
#include <stdio.h>
#include "orbit.h"
#include "SGP4.h"

static elsetrec satrec;
static double jd_epoch;
static bool ready;

static double unix_to_jd(double unix_s)
{
    return unix_s / 86400.0 + 2440587.5;
}

bool orbit_init(const omm_t *omm)
{
    const double deg = M_PI / 180.0;
    const double xpdotp = 1440.0 / (2.0 * M_PI);   // rev/day -> rad/min
    char satn[9];
    snprintf(satn, sizeof satn, "%d", omm->norad_cat_id);
    jd_epoch = unix_to_jd(omm->epoch_unix);
    // satellite.js passes epoch as days since 1949-12-31 00:00 UT; ndot/nddot are unused by SGP4.
    ready = SGP4Funcs::sgp4init(wgs72, 'i', satn, jd_epoch - 2433281.5,
                                omm->bstar, omm->mean_motion_dot, omm->mean_motion_ddot,
                                omm->eccentricity, omm->arg_of_pericenter * deg,
                                omm->inclination * deg, omm->mean_anomaly * deg,
                                omm->mean_motion / xpdotp, omm->ra_of_asc_node * deg, satrec);
    return ready && satrec.error == 0;
}

bool orbit_propagate(double unix_s, double r[3], double v[3])
{
    if (!ready) return false;
    double tsince = (unix_to_jd(unix_s) - jd_epoch) * 1440.0;
    return SGP4Funcs::sgp4(satrec, tsince, r, v) && satrec.error == 0;
}

double orbit_gmst(double unix_s)
{
    return SGP4Funcs::gstime_SGP4(unix_to_jd(unix_s));
}
