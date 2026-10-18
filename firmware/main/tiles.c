#include <string.h>
#include "driver/jpeg_decode.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tiles.h"

static const char *TAG = "tiles";

#define NTILES HI_NTILES
// Tiles per cache. Terrain tiles carry their mip levels (~340 KB each); a view has up to ~16
// in sight. City lights are only looked up near the camera at night: a few at a time.
#define CACHE_N 22
static const int cache_n[2] = {CACHE_N, 14};
#define IN_MAX (96 * 1024)      // largest compressed tile (a whole number of cache lines)
#define FAR 1e30f               // "not wanted" / "not in view"

// As gen_assets.py writes it: offset and length of the terrain JPEG, then of the lights JPEG.
typedef struct { struct { uint32_t off, len; } k[2]; } entry_t;   // k[0] terrain, k[1] lights

// A cache slot. Pixels are only swapped at frame boundaries (tiles_frame()), so a frame never
// sees a tile change under it. near: the smallest footprint that used it this frame; seen: the
// same for the last finished frame (FAR: not in view).
typedef struct { int key; uint32_t used; float near, seen; void *px; } slot_t;

static const esp_partition_t *part;
static entry_t *table;
static slot_t slots[2][CACHE_N];
static int16_t slot_of[2][NTILES];
// Missing tiles asked for: the smallest footprint asking this frame (want) and in the last
// finished frame (wanted, what the loader serves).
static float want[2][NTILES], wanted[2][NTILES];
static uint8_t no_data[2][NTILES];   // nothing to decode (e.g. no city lights over the ocean)
static volatile uint32_t frame;
// One finished decode waiting to be committed at the next frame boundary.
static struct { volatile bool ready; int kind, key; void *px; } done;
static void *spare[2];          // the loader's free output buffer per kind
static TaskHandle_t loader_task;
static jpeg_decoder_handle_t jpeg;
static uint8_t *in_buf;
static size_t in_cap;
static const void *map;   // the tiles partition, memory-mapped
static int view_max[2], miss_max[2];

// The decoder's output size must be a whole number of cache lines.
#define LINE_UP(n) (((n) + 127) & ~(size_t)127)

static size_t tile_bytes(int kind)
{
    // terrain: levels 0, 1, 2 (RGB565); lights: levels 0, 1 (grey)
    return kind == 0 ? HI_T0_PX * 2 + HI_T1_PX * 2 + HI_T2_PX * 2 : LINE_UP(HI_T * HI_T) + (HI_T / 2) * (HI_T / 2);
}

// Both render cores call the lookups; a minimum lost between them only reorders loading.
static inline void *lookup(int kind, int key, float fp)
{
    if (!part || (unsigned)key >= NTILES) return NULL;
    int s = slot_of[kind][key];
    if (s >= 0) {
        slot_t *sl = &slots[kind][s];
        sl->used = frame;
        if (fp < sl->near) sl->near = fp;
        return sl->px;
    }
    if (fp < want[kind][key]) want[kind][key] = fp;
    return NULL;
}

const uint16_t *tiles_terrain(int key, float fp) { return lookup(0, key, fp); }

const uint8_t *tiles_lights(int key, int level, float fp)
{
    const uint8_t *px = lookup(1, key, fp);
    return px && level ? px + LINE_UP(HI_T * HI_T) : px;
}

// The slot a new tile would replace: one not in view in the last frame (the least recently
// used), else the one in view farthest away. Returns that slot's footprint (FAR if not in view).
static float victim(int kind, int *out)
{
    int idle = -1, far = 0;
    for (int i = 0; i < cache_n[kind]; i++) {
        const slot_t *s = &slots[kind][i];
        if (s->seen >= FAR) {
            if (idle < 0 || s->used < slots[kind][idle].used) idle = i;
        } else if (s->seen > slots[kind][far].seen) {
            far = i;
        }
    }
    *out = idle >= 0 ? idle : far;
    return idle >= 0 ? FAR : slots[kind][far].seen;
}

void tiles_stats(int in_view[2], int missing[2])
{
    for (int k = 0; k < 2; k++) {
        in_view[k] = view_max[k]; missing[k] = miss_max[k];
        view_max[k] = miss_max[k] = 0;
    }
}

void tiles_frame(void)
{
    if (!part) return;
    // Close the frame: what each slot and each missing tile was used for becomes "last frame".
    for (int k = 0; k < 2; k++) {
        int nv = 0, nm = 0;
        for (int i = 0; i < cache_n[k]; i++) {
            slots[k][i].seen = slots[k][i].near;
            slots[k][i].near = FAR;
            nv += slots[k][i].seen < FAR;
        }
        for (int i = 0; i < NTILES; i++) {
            wanted[k][i] = want[k][i];
            want[k][i] = FAR;
            nm += wanted[k][i] < FAR;
        }
        if (nv > view_max[k]) view_max[k] = nv;
        if (nm > miss_max[k]) miss_max[k] = nm;
    }
    frame++;
    if (done.ready) {
        // Commit the finished tile, unless every slot holds a tile in view nearer than it (the
        // view moved on while it was decoded); the replaced buffer is the loader's next output.
        int kind = done.kind, key = done.key, v;
        float d = victim(kind, &v);
        if (d >= FAR || d > wanted[kind][key]) {
            slot_t *s = &slots[kind][v];
            if (s->key >= 0) slot_of[kind][s->key] = -1;
            void *old = s->px;
            s->px = done.px;
            s->key = key;
            s->used = frame;
            s->seen = wanted[kind][key];
            slot_of[kind][key] = (int16_t)v;
            wanted[kind][key] = FAR;
            spare[kind] = old;
        }
        done.ready = false;
    }
    if (loader_task) xTaskNotifyGive(loader_task);
}

// halve(): 2x2 box filter with a mild gain, so averaged-out small towns don't fade too much.
static void halve(const uint8_t *src, uint8_t *dst)
{
    for (int y = 0; y < HI_T / 2; y++)
        for (int x = 0; x < HI_T / 2; x++) {
            const uint8_t *p = src + 2 * y * HI_T + 2 * x;
            int v = (int)((p[0] + p[1] + p[HI_T] + p[HI_T + 1]) * 0.32f);
            dst[y * (HI_T / 2) + x] = v > 255 ? 255 : v;
        }
}

// One RGB565 mip level: n x n -> n/2 x n/2, 2x2 box filter.
static void mip565(const uint16_t *src, int n, uint16_t *dst)
{
    for (int y = 0; y < n / 2; y++) {
        const uint16_t *a = src + 2 * y * n, *b = a + n;
        uint16_t *o = dst + y * (n / 2);
        for (int x = 0; x < n / 2; x++, a += 2, b += 2) {
            uint32_t p0 = a[0], p1 = a[1], p2 = b[0], p3 = b[1];
            uint32_t r = ((p0 >> 11) + (p1 >> 11) + (p2 >> 11) + (p3 >> 11) + 2) >> 2;
            uint32_t g = (((p0 >> 5) & 63) + ((p1 >> 5) & 63) + ((p2 >> 5) & 63) + ((p3 >> 5) & 63) + 2) >> 2;
            uint32_t bl = ((p0 & 31) + (p1 & 31) + (p2 & 31) + (p3 & 31) + 2) >> 2;
            o[x] = (uint16_t)(r << 11 | g << 5 | bl);
        }
    }
}

static volatile int decoded;
int tiles_decoded(void) { int n = decoded; decoded = 0; return n; }

static bool decode(int kind, int key, void *out)
{
    const entry_t *e = &table[key];
    uint32_t len = e->k[kind].len;
    if (len == 0 || len > in_cap) return false;
    // From memory-mapped flash: a plain esp_partition_read() disables the caches and
    // interrupts while it runs, which delays the panel's per-refresh DMA restart (white flashes).
    if (map) {
        memcpy(in_buf, (const uint8_t *)map + e->k[kind].off, len);
        esp_cache_msync(in_buf, LINE_UP(len), ESP_CACHE_MSYNC_FLAG_DIR_C2M);   // the decoder reads RAM by DMA
    }
    else if (esp_partition_read(part, e->k[kind].off, in_buf, len) != ESP_OK) return false;
    jpeg_decode_cfg_t cfg = {
        .output_format = kind == 0 ? JPEG_DECODE_OUT_FORMAT_RGB565 : JPEG_DECODE_OUT_FORMAT_GRAY,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t out_size = 0;
    size_t cap = kind == 0 ? HI_T * HI_T * 2 : LINE_UP(HI_T * HI_T);
    if (jpeg_decoder_process(jpeg, &cfg, in_buf, len, out, cap, &out_size) != ESP_OK) return false;
    if (kind == 1) halve(out, (uint8_t *)out + LINE_UP(HI_T * HI_T));
    else {
        // Mip levels for the pilot view's filtered lookups (2x2 box filter, rounded).
        uint16_t *l0 = out, *l1 = l0 + HI_T0_PX, *l2 = l1 + HI_T1_PX;
        mip565(l0, HI_T, l1);
        mip565(l1, HI_T / 2, l2);
    }
    return true;
}

// The missing tile of one kind wanted nearest in the last frame, or -1.
static int nearest_wanted(int kind, float *d)
{
    int best = -1;
    *d = FAR;
    for (int i = 0; i < NTILES; i++)
        if (wanted[kind][i] < *d && slot_of[kind][i] < 0 && !no_data[kind][i]) { *d = wanted[kind][i]; best = i; }
    return best;
}

// The next tile to load: the nearest wanted one (terrain first at equal footprint) that would
// get a slot, i.e. its cache has a slot out of view or one in view farther away.
static bool pick(int *kind_out, int *key_out)
{
    float d[2];
    int key[2] = {nearest_wanted(0, &d[0]), nearest_wanted(1, &d[1])};
    int first = d[1] < d[0];
    for (int n = 0; n < 2; n++) {
        int k = n ? !first : first, v;
        if (key[k] >= 0 && victim(k, &v) > d[k]) { *kind_out = k; *key_out = key[k]; return true; }
    }
    return false;
}

static void loader(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // once per pilot frame
        int kind, key;
        while (!done.ready && pick(&kind, &key)) {
            if (!decode(kind, key, spare[kind])) { no_data[kind][key] = 1; continue; }
            decoded++;
            done.kind = kind;
            done.key = key;
            done.px = spare[kind];
            done.ready = true;
        }
    }
}

static void *alloc_out(size_t n)
{
    jpeg_decode_memory_alloc_cfg_t mc = {.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER};
    size_t got = 0;
    return jpeg_alloc_decoder_mem(n, &mc, &got);
}

void tiles_init(void)
{
    for (int k = 0; k < 2; k++)
        for (int i = 0; i < NTILES; i++) { slot_of[k][i] = -1; want[k][i] = wanted[k][i] = FAR; }
    part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "tiles");
    uint32_t hdr[2] = {0};
    if (!part || esp_partition_read(part, 0, hdr, sizeof hdr) != ESP_OK || memcmp(hdr, "OTTL", 4) || hdr[1] != NTILES) {
        ESP_LOGW(TAG, "no tile partition, or an older layout (run tools/flash_tiles.sh): "
                      "the pilot view uses the global texture only");
        part = NULL;
        return;
    }
    esp_partition_mmap_handle_t mh;
    if (esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, &map, &mh) != ESP_OK) {
        ESP_LOGW(TAG, "tiles partition not mapped: reading with flash operations");
        map = NULL;
    }
    table = heap_caps_malloc(NTILES * sizeof(entry_t), MALLOC_CAP_SPIRAM);
    esp_partition_read(part, 8, table, NTILES * sizeof(entry_t));
    jpeg_decode_engine_cfg_t ec = {.intr_priority = 0, .timeout_ms = 200};
    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&ec, &jpeg));
    jpeg_decode_memory_alloc_cfg_t mc = {.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER};
    in_buf = jpeg_alloc_decoder_mem(IN_MAX, &mc, &in_cap);
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < cache_n[k]; i++) {
            slots[k][i].key = -1;
            slots[k][i].used = 0;
            slots[k][i].near = slots[k][i].seen = FAR;
            slots[k][i].px = alloc_out(LINE_UP(tile_bytes(k)));
        }
        spare[k] = alloc_out(LINE_UP(tile_bytes(k)));
    }
    // Check the decoder's colour order on a known tile (top-left: Arctic Ocean, sea colour).
    uint16_t *probe = spare[0];
    if (decode(0, 0, probe))
        ESP_LOGI(TAG, "tile partition ok, tile 0 pixel 0 = 0x%04x (sea 0x%04x)", probe[0],
                 (34 >> 3) << 11 | (78 >> 2) << 5 | (172 >> 3));
    xTaskCreatePinnedToCore(loader, "tiles", 4096, NULL, 2, &loader_task, 0);
}
