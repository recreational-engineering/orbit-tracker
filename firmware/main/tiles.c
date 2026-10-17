#include <string.h>
#include "driver/jpeg_decode.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tiles.h"

static const char *TAG = "tiles";

#define NTILES (HI_ROWS * HI_COLS)
#define CACHE_N 24              // tiles per cache (HI.CACHE)
#define IN_MAX (96 * 1024)      // largest compressed tile (a whole number of cache lines)

// As gen_assets.py writes it: offset and length of the terrain JPEG, then of the lights JPEG.
typedef struct { struct { uint32_t off, len; } k[2]; } entry_t;   // k[0] terrain, k[1] lights

// A cache slot. Pixels are only swapped at frame boundaries (tiles_frame()), so a frame never
// sees a tile change under it.
typedef struct { int key; uint32_t used; void *px; } slot_t;

static const esp_partition_t *part;
static entry_t *table;
static slot_t slots[2][CACHE_N];
static int16_t slot_of[2][NTILES];
static volatile uint8_t pending[2][NTILES];
static QueueHandle_t requests;
static volatile uint32_t frame;
// One finished decode waiting to be committed at the next frame boundary.
static struct { volatile bool ready; int kind, key; void *px; } done;
static void *spare[2];          // the loader's free output buffer per kind
static jpeg_decoder_handle_t jpeg;
static uint8_t *in_buf;
static size_t in_cap;
static const void *map;   // the tiles partition, memory-mapped

// The decoder's output size must be a whole number of cache lines.
#define LINE_UP(n) (((n) + 127) & ~(size_t)127)

static size_t tile_bytes(int kind)
{
    return kind == 0 ? HI_T * HI_T * 2 : LINE_UP(HI_T * HI_T) + (HI_T / 2) * (HI_T / 2);   // lights: level 0 + 1
}

static void request(int kind, int key)
{
    if (!part || pending[kind][key]) return;
    pending[kind][key] = 1;
    uint16_t msg = (uint16_t)(kind << 15 | key);
    if (xQueueSend(requests, &msg, 0) != pdTRUE) pending[kind][key] = 0;   // full: ask again later
}

const uint16_t *tiles_terrain(int key)
{
    if ((unsigned)key >= NTILES) return NULL;
    int s = slot_of[0][key];
    if (s >= 0) { slots[0][s].used = frame; return slots[0][s].px; }
    request(0, key);
    return NULL;
}

const uint8_t *tiles_lights(int key, int level)
{
    if ((unsigned)key >= NTILES) return NULL;
    int s = slot_of[1][key];
    if (s >= 0) {
        slots[1][s].used = frame;
        const uint8_t *px = slots[1][s].px;
        return level ? px + LINE_UP(HI_T * HI_T) : px;
    }
    request(1, key);
    return NULL;
}

void tiles_frame(void)
{
    frame++;
    if (!done.ready) return;
    // Commit the finished tile into the least recently used slot; its old buffer becomes the
    // loader's next output buffer.
    int kind = done.kind, best = 0;
    for (int i = 1; i < CACHE_N; i++)
        if (slots[kind][i].used < slots[kind][best].used) best = i;
    slot_t *s = &slots[kind][best];
    if (s->key >= 0) { slot_of[kind][s->key] = -1; pending[kind][s->key] = 0; }
    void *old = s->px;
    s->px = done.px;
    s->key = done.key;
    s->used = frame;
    slot_of[kind][done.key] = (int16_t)best;
    spare[kind] = old;
    done.ready = false;
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
    return true;
}

static void loader(void *arg)
{
    for (;;) {
        uint16_t msg;
        xQueueReceive(requests, &msg, portMAX_DELAY);
        int kind = msg >> 15, key = msg & 0x7FFF;
        if (slot_of[kind][key] >= 0) continue;
        while (done.ready) vTaskDelay(1);   // the previous tile is not committed yet
        void *out = spare[kind];
        // A tile that cannot be decoded (no data, e.g. no city lights over the ocean) stays
        // marked pending, so it is not requested again every frame.
        if (!decode(kind, key, out)) continue;
        decoded++;
        done.kind = kind;
        done.key = key;
        done.px = out;
        done.ready = true;
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
        for (int i = 0; i < NTILES; i++) slot_of[k][i] = -1;
    part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "tiles");
    uint32_t hdr[2] = {0};
    if (!part || esp_partition_read(part, 0, hdr, sizeof hdr) != ESP_OK || memcmp(hdr, "OTTL", 4) || hdr[1] != NTILES) {
        ESP_LOGW(TAG, "no tile partition: the pilot view uses the global texture only");
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
        for (int i = 0; i < CACHE_N; i++) {
            slots[k][i].key = -1;
            slots[k][i].used = 0;
            slots[k][i].px = alloc_out(LINE_UP(tile_bytes(k)));
        }
        spare[k] = alloc_out(LINE_UP(tile_bytes(k)));
    }
    requests = xQueueCreate(16, sizeof(uint16_t));
    xTaskCreatePinnedToCore(loader, "tiles", 4096, NULL, 2, NULL, 0);
    // Check the decoder's colour order on a known tile (top-left: Arctic Ocean, sea colour).
    uint16_t *probe = spare[0];
    if (decode(0, 0, probe))
        ESP_LOGI(TAG, "tile partition ok, tile 0 pixel 0 = 0x%04x (sea 0x%04x)", probe[0],
                 (34 >> 3) << 11 | (78 >> 2) << 5 | (172 >> 3));
}
