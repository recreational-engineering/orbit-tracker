/* High-resolution terrain and city-light tiles for the pilot view (requestHiTile() /
 * requestLitTile() in web/app.js): 15 x 30 tiles of 360 x 360 texels (12° x 12°) in the
 * "tiles" flash partition, decoded on demand by the hardware JPEG decoder into LRU caches
 * in PSRAM. Lookups never block: a missing tile is requested and NULL returned, and the
 * caller falls back to the global texture until it is there. */
#pragma once
#include <stdint.h>

#define HI_T 360
#define HI_ROWS 15
#define HI_COLS 30

void tiles_init(void);
// Tiles decoded since the last call.
int tiles_decoded(void);
// Call once per pilot frame (LRU clock).
void tiles_frame(void);
// Terrain colours (RGB565, row-major HI_T x HI_T) or NULL.
const uint16_t *tiles_terrain(int key);
// City lights, level 0 (HI_T x HI_T) or 1 (half size), one byte per texel, or NULL.
const uint8_t *tiles_lights(int key, int level);
