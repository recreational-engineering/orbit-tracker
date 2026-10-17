/* High-resolution terrain and city-light tiles for the pilot view (requestHiTile() /
 * requestLitTile() in web/app.js): 15 rows of 12° in latitude, each HI_ROW_COLS[row] tiles
 * wide (fewer, wider tiles toward the poles, where a degree of longitude is short), of
 * 360 x 360 texels, in the "tiles" flash partition. They are decoded on demand by the hardware
 * JPEG decoder into caches in PSRAM, nearest tiles first. Lookups never block: a missing tile
 * is requested and NULL returned, and the caller falls back to the global texture until it is
 * there. */
#pragma once
#include <stdint.h>

#define HI_T 360
#define HI_ROWS 15
#define HI_NTILES 330
// As HI_ROW_COLS in tools/gen_assets.py; HI_ROW_START is the first key of each row.
static const uint8_t HI_ROW_COLS[HI_ROWS] = {6, 12, 18, 24, 30, 30, 30, 30, 30, 30, 30, 24, 18, 12, 6};
static const uint16_t HI_ROW_START[HI_ROWS] = {0, 6, 18, 36, 60, 90, 120, 150, 180, 210, 240, 270, 294, 312, 324};

// The tile under texture coordinates fu (0..1 from 180° W) and fvn (0..1 from the north pole),
// and the texel in it.
static inline int tiles_key(float fu, float fvn, int *tx, int *ty)
{
    float Y = fvn * (HI_ROWS * HI_T);
    int row = (int)Y / HI_T;
    if (row > HI_ROWS - 1) row = HI_ROWS - 1;
    int cols = HI_ROW_COLS[row];
    float X = fu * (cols * HI_T);
    int col = (int)X / HI_T;
    if (col > cols - 1) col = cols - 1;
    *tx = (int)X - col * HI_T;
    *ty = (int)Y - row * HI_T;
    if (*tx > HI_T - 1) *tx = HI_T - 1;
    if (*ty > HI_T - 1) *ty = HI_T - 1;
    return HI_ROW_START[row] + col;
}

void tiles_init(void);
// Tiles decoded since the last call.
int tiles_decoded(void);
// Most tiles in view in one frame, and most tiles wanted but missing, since the last call
// ([0] terrain, [1] lights).
void tiles_stats(int in_view[2], int missing[2]);
// Call once per pilot frame: commits a finished tile and starts the next frame's bookkeeping.
void tiles_frame(void);
// Terrain colours (RGB565, row-major HI_T x HI_T) or NULL. fp: the ground footprint of the
// pixel asking (km); the nearest (smallest footprint) wanted tiles are loaded first.
const uint16_t *tiles_terrain(int key, float fp);
// City lights, level 0 (HI_T x HI_T) or 1 (half size), one byte per texel, or NULL.
const uint8_t *tiles_lights(int key, int level, float fp);
