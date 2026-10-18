/* Port of the round-display GUI of web/app.js: orbit, ground (Munich) and flat-earth
 * views, on-screen view menu, instrument-cluster HUD, drag to look around.
 * Defaults of the prototype's side panel: graticule, borders, reticle and night side
 * on, altitude exaggeration 1x, no time offset. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float base_ms;      // globe pixels for this frame (copy of the cached globe or a half-res render)
    float lines_ms, overlay_ms, total_ms;
    float px_ms, replay_ms, copy_ms;   // strips on the busier core: globe pixels, vector replay, copy out
    bool half_res;      // globe rendered at half resolution this frame (view moving)
} scene_timing_t;

typedef struct {
    bool valid;
    double lat, lon, alt, vel;   // geodetic, as the HUD shows them
} scene_sat_t;

void scene_init(void);
// Reserve the render strip buffers in fast internal RAM; call first thing at boot.
void scene_reserve_strips(void);
// Whether both render strips are in the main SRAM (else rendering is several times slower).
bool scene_strips_fast(void);
// Advance the model to Unix time unix_s (wall clock dt since the last call, seconds).
void scene_update(double unix_s, float dt, int64_t now_ms);
void scene_render(uint16_t *fb, scene_timing_t *timing);
// Touch: one point, pressed or released, in display pixels.
void scene_touch(bool pressed, int x, int y, int64_t now_ms);
// Called when the clock jumps: recompute the orbit data and re-centre.
void scene_clock_changed(double unix_s);

scene_sat_t scene_sat(void);
void scene_rotation(float out[3]);
const char *scene_mode_name(void);
// Background globe renders since the last call and their average duration.
void scene_globe_stats(int *count, float *avg_ms);
// A full-resolution globe is being rendered in the background (the frame loop yields more).
bool scene_background_busy(void);
