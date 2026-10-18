/* Orbit tracker on the Waveshare ESP32-P4-WIFI6-Touch-LCD-4C (720x720 round MIPI-DSI).
 *
 * Renders the globe of the web prototype (web/app.js) into two DPI frame buffers
 * and flips between them. Serial console (2 Mbaud) commands, one per line:
 *   T<unix seconds>   set the clock (tools/set_time.py)
 *   S                 dump the frame on screen as raw RGB565 (tools/screenshot.py)
 *   G x0 y0 x1 y1 ms  simulated finger drag (testing without touching the panel)
 *   K x y             simulated tap
 *   P                 print frame statistics since the last P (and stop the periodic ones)
 *
 * Standby: weekdays 20:00-08:00 and all of Saturday and Sunday (local time) the backlight is
 * off; a touch wakes it for 30 minutes.
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "bsp/esp-bsp.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gfx.h"
#include "scene.h"
#include "tiles.h"
#include "panel_rate.h"
#include "net.h"
#include "nvs_flash.h"
#include "generated/elements.h"

static const char *TAG = "orbit";

#define FRAME_HZ 30

// Standby: on weekdays from 20:00 to 08:00 and all day on Saturday and Sunday (local time) the
// backlight is switched off (its LED driver gets no on-time at all, so no light leaks as a black
// frame would) and nothing is rendered; the touch controller stays on. A touch wakes the
// display, which goes dark again after STANDBY_IDLE_S without a touch.
#define STANDBY_FROM_H 20
#define STANDBY_TO_H 8
#ifndef STANDBY_IDLE_S
#define STANDBY_IDLE_S (30 * 60)
#endif

static bool standby_hours(void)
{
    time_t t = time(NULL);
    struct tm lt;
    localtime_r(&t, &lt);   // TZ = Europe/Berlin
    if (lt.tm_wday == 0 || lt.tm_wday == 6) return true;   // Sunday, Saturday
    return lt.tm_hour >= STANDBY_FROM_H || lt.tm_hour < STANDBY_TO_H;
}

static esp_lcd_panel_handle_t panel;
static esp_lcd_touch_handle_t touch;
static SemaphoreHandle_t fb_released;
static volatile bool want_screenshot, clock_changed, want_stats, stats_on_demand, flash_watch;

// Flash monitor (console "F" toggles it): a 16x16 grid of cell brightnesses per frame. A cell
// that jumps away for exactly one frame and comes back (frame n differs from n-1 and n+1 while
// those two agree) is the signature of a glitch frame; such frames are counted and logged.
#define FW_N 16
static int fw_hist[3][FW_N * FW_N], fw_frames, fw_glitches;

static void flash_check(const uint16_t *fb, bool half)
{
    int *cur = fw_hist[fw_frames % 3];
    for (int cy = 0; cy < FW_N; cy++)
        for (int cx = 0; cx < FW_N; cx++) {
            int sum = 0;   // 6 x 6 samples over the 45 x 45 px cell: moving fine detail averages out
            for (int q = 0; q < 36; q++) {
                int x = cx * (GFX_W / FW_N) + 3 + (q % 6) * 7, y = cy * (GFX_H / FW_N) + 3 + (q / 6) * 7;
                uint16_t p = fb[y * GFX_W + x];
                sum += ((p >> 11) & 31) * 2 + ((p >> 5) & 63) * 2 + (p & 31);   // ~ luminance, max 219
            }
            cur[cy * FW_N + cx] = sum;
        }
    fw_frames++;
    if (fw_frames < 3) return;
    const int *a = fw_hist[(fw_frames - 3) % 3], *b = fw_hist[(fw_frames - 2) % 3], *c = cur;
    int odd = 0;
    for (int i = 0; i < FW_N * FW_N; i++) {
        int ab = abs(b[i] - a[i]), bc = abs(b[i] - c[i]), ac = abs(a[i] - c[i]);
        // A jump of over a sixth of full brightness (36 · 219 / 6) that comes back.
        if (ab > 1300 && bc > 1300 && ac * 3 < (ab < bc ? ab : bc)) odd++;
    }
    if (odd >= 2) {
        fw_glitches++;
        int first = -1;
        for (int i = 0; i < FW_N * FW_N && first < 0; i++) {
            int ab = abs(b[i] - a[i]), bc = abs(b[i] - c[i]), ac = abs(a[i] - c[i]);
            if (ab > 1300 && bc > 1300 && ac * 3 < (ab < bc ? ab : bc)) first = i;
        }
        ESP_LOGW(TAG, "FLASH? frame %d (%s, %s): %d of %d cells jumped for one frame, first at cell (%d, %d): %d -> %d -> %d",
                 fw_frames - 2, scene_mode_name(), half ? "half-res" : "full/cached", odd, FW_N * FW_N, first % FW_N, first / FW_N,
                 a[first], b[first], c[first]);
    }
}

// Simulated touch from the console: a drag from (x0, y0) to (x1, y1) over `ms`, or a tap.
static struct { volatile bool pending; int x0, y0, x1, y1, ms; } sim;
static struct { bool active; int64_t start; int x0, y0, x1, y1, ms, phase; } sim_run;

// Three frame buffers, drawn round robin: the panel scans one, a finished one waits for its
// refresh slot (the presenter task hands it over then), the third is drawn. At the end of each
// refresh the panel starts on the last buffer handed over.
#define NFB 3
static volatile int fb_submitted, fb_scanning, fb_pending = -1;
static void *fbs[NFB];

// Panel refresh health: a refresh that ends much later than the 60 Hz period means the panel's
// DMA was restarted late and it showed a broken frame (seen as a white flash).
static volatile uint32_t refresh_late, refresh_max_us;
static volatile int64_t refresh_last;
static volatile uint32_t refresh_count;   // refreshes completed (each one starts the next)
static TaskHandle_t presenter;
static volatile int missed;               // frames that appeared after their target refresh

static bool IRAM_ATTR on_frame_buf_complete(esp_lcd_panel_handle_t p, esp_lcd_dpi_panel_event_data_t *e, void *ctx)
{
    int64_t t = esp_timer_get_time();
    if (refresh_last) {
        uint32_t d = (uint32_t)(t - refresh_last);
        if (d > refresh_max_us) refresh_max_us = d;
        if (d > PANEL_REFRESH_US * 3 / 2) refresh_late++;   // 1.5 refresh periods
    }
    refresh_last = t;
    refresh_count++;
    fb_scanning = fb_submitted;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(fb_released, &woken);
    if (presenter) vTaskNotifyGiveFromISR(presenter, &woken);
    return woken == pdTRUE;
}

// Frame pacing, the hand-over half: a finished frame is given to the panel during the refresh
// before its target, so it appears exactly at the instant it was drawn for. A task of its own,
// so the frame loop starts on the next frame meanwhile instead of waiting.
typedef struct { int fb; uint32_t target; } present_t;
static QueueHandle_t present_q;

static void presenter_task(void *arg)
{
    present_t f;
    for (;;) {
        xQueueReceive(present_q, &f, portMAX_DELAY);
        while ((int32_t)(refresh_count - (f.target - 1)) < 0) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        esp_lcd_panel_draw_bitmap(panel, 0, 0, GFX_W, GFX_H, fbs[f.fb]);
        fb_submitted = f.fb;
        if ((int32_t)(refresh_count - f.target) >= 0) missed++;
        fb_pending = -1;
        xSemaphoreGive(fb_released);   // the frame loop may be waiting to queue the next one
    }
}

static double now_unix(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1e6;
}

static void touch_init(void)
{
    ESP_ERROR_CHECK(bsp_i2c_init());
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    io_cfg.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ;
    esp_lcd_panel_io_handle_t io;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bsp_i2c_get_handle(), &io_cfg, &io));
    const esp_lcd_touch_config_t cfg = {
        .x_max = GFX_W, .y_max = GFX_H,
        .rst_gpio_num = GPIO_NUM_NC, .int_gpio_num = GPIO_NUM_NC,
    };
    if (esp_lcd_touch_new_i2c_gt911(io, &cfg, &touch) != ESP_OK) {
        ESP_LOGW(TAG, "touch controller not found, panning disabled");
        touch = NULL;
    }
}

// Network updates: once at boot, then once a day at 00:00 UTC while the device keeps running.
// The radio is off except during a sync (net_sync brings it up and takes it down).
static void net_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));            // let the UI come up first
    net_sync((net_sync_t){.time = true, .elements = true});
    for (;;) {
        time_t now = time(NULL);
        struct tm tm;
        gmtime_r(&now, &tm);
        // Seconds until the next 00:00 UTC.
        int secs = ((23 - tm.tm_hour) * 60 + (59 - tm.tm_min)) * 60 + (60 - tm.tm_sec);
        vTaskDelay(pdMS_TO_TICKS((secs + 1) * 1000));
        net_sync((net_sync_t){.time = true, .elements = true});
    }
}

static void console_task(void *arg)
{
    char line[64];
    int n = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &c, 1, portMAX_DELAY) != 1) continue;
        if (c != '\n' && c != '\r') {
            if (n < (int)sizeof line - 1) line[n++] = c;
            continue;
        }
        line[n] = 0;
        n = 0;
        // Opening the port can put a stray byte on the line: start at the command letter.
        const char *cmd = line;
        while (*cmd && (*cmd < 'A' || *cmd > 'Z')) cmd++;
        if (cmd[0] == 'T') {
            double t = atof(cmd + 1);
            struct timeval tv = {.tv_sec = (time_t)t, .tv_usec = (suseconds_t)((t - (time_t)t) * 1e6)};
            settimeofday(&tv, NULL);
            clock_changed = true;
            ESP_LOGI(TAG, "TIME set to %.3f", t);
        } else if (cmd[0] == 'N') {
            if (!net_busy()) xTaskCreatePinnedToCore(net_sync_now, "net-now", 12288, NULL, 4, NULL, 0);
        } else if (cmd[0] == 'G' || cmd[0] == 'K') {
            int v[5] = {0};
            int n = sscanf(cmd + 1, "%d %d %d %d %d", &v[0], &v[1], &v[2], &v[3], &v[4]);
            if (cmd[0] == 'K' && n == 2) { v[2] = v[0]; v[3] = v[1]; v[4] = 0; n = 5; }
            if (n == 5) {
                sim.x0 = v[0]; sim.y0 = v[1]; sim.x1 = v[2]; sim.y1 = v[3]; sim.ms = v[4];
                sim.pending = true;
            }
        } else if (cmd[0] == 'F') {
            flash_watch = !flash_watch;
        } else if (cmd[0] == 'P') {
            want_stats = true;
            stats_on_demand = true;
        } else if (cmd[0] == 'S') {
            want_screenshot = true;
        }
    }
}

static void dump_frame(const uint16_t *fb)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    vTaskDelay(pdMS_TO_TICKS(20));
    const int port = CONFIG_ESP_CONSOLE_UART_NUM;
    const char *hdr = "\n@@FRAME 720 720 RGB565LE@@\n";
    uart_write_bytes(port, hdr, strlen(hdr));
    // Upright, as seen on the mounted display (the frame buffer may hold it turned 180°).
    // (A row buffer in PSRAM, only while dumping: internal RAM is kept for the renderer.)
    uint16_t *row = heap_caps_malloc(GFX_W * 2, MALLOC_CAP_SPIRAM);
    for (int y = 0; y < GFX_H && row; y++) {
        const uint16_t *src = fb + (GFX_ROT180 ? GFX_H - 1 - y : y) * GFX_W;
        for (int x = 0; x < GFX_W; x++) row[x] = src[GFX_ROT180 ? GFX_W - 1 - x : x];
        uart_write_bytes(port, (const char *)row, GFX_W * 2);
    }
    free(row);
    uart_write_bytes(port, "@@END@@\n", 8);
    uart_wait_tx_done(port, portMAX_DELAY);
    esp_log_level_set("*", ESP_LOG_INFO);
}

// A task that mostly waits (console, network): its stack goes into the low-power SRAM, which is
// slow but otherwise unused, so the main SRAM stays free for the render strips and Wi-Fi.
static void idle_task(TaskFunction_t fn, const char *name, int stack, int prio)
{
    if (xTaskCreatePinnedToCoreWithCaps(fn, name, stack, NULL, prio, NULL, 0, MALLOC_CAP_RTCRAM) != pdPASS)
        xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, NULL, 0);
}

void app_main(void)
{
    // Console: driver-backed so the console task can read commands.
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 1024, 64 * 1024, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // No RTC battery or network time yet: never run earlier than the asset build.
    if (now_unix() < ASSETS_GEN_UNIX) {
        struct timeval tv = {.tv_sec = ASSETS_GEN_UNIX};
        settimeofday(&tv, NULL);
        ESP_LOGW(TAG, "clock not set, starting at asset build time; send T<unix> to set it");
    }

    esp_lcd_panel_io_handle_t io;
    ESP_ERROR_CHECK(bsp_display_new(NULL, &panel, &io));
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(panel, NFB, &fbs[0], &fbs[1], &fbs[2]));
    fb_released = xSemaphoreCreateBinary();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {.on_frame_buf_complete = on_frame_buf_complete};
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(panel, &cbs, NULL));
    touch_init();

    vTaskPrioritySet(NULL, 5);   // frame loop above the background globe redraws
    scene_init();
    idle_task(console_task, "console", 4096, 5);
    net_init();
    idle_task(net_task, "net", 12288, 4);

    bool first = true;
    int frames = 0, half_frames = 0;
    float worst_ms = 0, worst_gap_ms = 0;
    int64_t last_present = 0;
    scene_timing_t sum = {0}, tm;
    int64_t last = esp_timer_get_time(), stats_at = last;
    // Frame pacing: a frame handed over during refresh n - 1 is on screen from the end of it
    // (refresh_count == n). Each frame aims at the first such boundary after its expected render
    // time and is drawn for exactly that moment; finished early, it is held back to that
    // refresh. So every frame shows the scene at the instant it appears, and motion (the spin,
    // glides, inertia) advances evenly instead of by the jittery render time.
    float work_us = 60000;      // render time estimate (rises fast, decays slowly)
    uint32_t last_target = 0;
    int last_fb = 0;
    present_q = xQueueCreate(1, sizeof(present_t));
    xTaskCreatePinnedToCore(presenter_task, "present", 3072, NULL, 6, &presenter, 0);
    bool was_pressed = false;
    bool dark = false;          // standby: backlight off, no rendering
    bool swallow = false;       // the touch that woke the display is not passed on as a tap
    int64_t last_touch_ms = 0;
    for (;;) {
        if (dark) {
            // Poll the touch controller at 20 Hz; wake on a touch (real or simulated) or when
            // the standby hours end (08:00 on a weekday).
            uint8_t cnt = 0;
            if (touch) {
                esp_lcd_touch_point_data_t pt;
                esp_lcd_touch_read_data(touch);
                esp_lcd_touch_get_data(touch, &pt, &cnt, 1);
            }
            bool touched = cnt > 0 || sim.pending;
            if (!touched && standby_hours()) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            dark = false;
            swallow = cnt > 0;
            sim.pending = false;
            was_pressed = false;
            last = esp_timer_get_time();
            last_touch_ms = last / 1000;
            first = true;   // backlight on once a fresh frame is on the panel
            ESP_LOGI(TAG, "standby: display on (%s)", touched ? "touch" : "morning");
        }
        int64_t now = esp_timer_get_time();
        const uint32_t rc = refresh_count;
        const int64_t rl = refresh_last ? refresh_last : now;
        // The boundary this frame will appear at: after the render, and never before or at the
        // previous frame's.
        uint32_t target = rc + 1 + (uint32_t)((now - rl + (int64_t)work_us) / PANEL_REFRESH_US);
        if ((int32_t)(target - last_target) <= 0) target = last_target + 1;
        const int64_t show = rl + (int64_t)(target - rc) * PANEL_REFRESH_US;
        float dt = (show - last) / 1e6f;
        if (dt < 0) dt = 0;
        last = show;
        int64_t ms = show / 1000;
        const double show_unix = now_unix() + (show - now) / 1e6;

        if (sim.pending) {
            sim.pending = false;
            sim_run = (typeof(sim_run)){true, ms, sim.x0, sim.y0, sim.x1, sim.y1, sim.ms, 0};
        }
        if (sim_run.active) {
            // Finger down at the start, moved along the line, lifted one frame after the end.
            float k = sim_run.ms > 0 ? fminf(1, (ms - sim_run.start) / (float)sim_run.ms) : 1;
            if (sim_run.phase < 2) {
                scene_touch(true, (int)lroundf(sim_run.x0 + (sim_run.x1 - sim_run.x0) * k),
                            (int)lroundf(sim_run.y0 + (sim_run.y1 - sim_run.y0) * k), ms);
                if (k >= 1) sim_run.phase++;
            } else {
                scene_touch(false, 0, 0, ms);
                sim_run.active = false;
            }
        } else if (touch) {
            esp_lcd_touch_point_data_t pt;
            uint8_t cnt = 0;
            esp_lcd_touch_read_data(touch);
            esp_lcd_touch_get_data(touch, &pt, &cnt, 1);
            if (GFX_ROT180) { pt.x = GFX_W - 1 - pt.x; pt.y = GFX_H - 1 - pt.y; }
            if (cnt) last_touch_ms = ms;
            if (swallow) {
                swallow = cnt > 0;   // until the waking finger is lifted
            } else {
                if (cnt) scene_touch(true, pt.x, pt.y, ms);
                else if (was_pressed) scene_touch(false, 0, 0, ms);
                was_pressed = cnt > 0;
            }
        }
        if (sim_run.active) last_touch_ms = ms;
        if (standby_hours() && ms - last_touch_ms > STANDBY_IDLE_S * 1000LL) {
            bsp_display_brightness_set(0);
            if (was_pressed) scene_touch(false, 0, 0, ms);
            was_pressed = false;
            dark = true;
            ESP_LOGI(TAG, "standby: display off");
            continue;
        }
        if (clock_changed) { clock_changed = false; scene_clock_changed(show_unix); }
        scene_update(show_unix, dt, ms);

        // The next buffer round robin is free once the panel no longer scans it and it is not
        // the one waiting for its slot.
        int back = (last_fb + 1) % NFB;
        while (back == fb_scanning || back == fb_pending) xSemaphoreTake(fb_released, pdMS_TO_TICKS(40));
        uint16_t *fb = fbs[back];
        scene_render(fb, &tm);
        float took = (float)(esp_timer_get_time() - now);
        work_us = took > work_us ? 0.5f * (work_us + took) + 1000 : 0.97f * work_us + 0.03f * took;
        // One frame waits for its slot at a time; the presenter hands it over.
        while (fb_pending >= 0) xSemaphoreTake(fb_released, pdMS_TO_TICKS(40));
        fb_pending = back;
        last_fb = back;
        last_target = target;
        present_t pf = {back, target};
        xQueueSend(present_q, &pf, portMAX_DELAY);
        if (flash_watch) flash_check(fb, tm.half_res);
        else if (fw_frames) {
            ESP_LOGI(TAG, "flash monitor: %d frames, %d suspicious", fw_frames, fw_glitches);
            fw_frames = fw_glitches = 0;
        }
        int64_t presented = esp_timer_get_time();
        if (last_present && (presented - last_present) / 1000.0f > worst_gap_ms) worst_gap_ms = (presented - last_present) / 1000.0f;
        last_present = presented;
        if (first) { bsp_display_brightness_set(100); first = false; }

        if (want_screenshot) { want_screenshot = false; dump_frame(fb); last = esp_timer_get_time(); }

        // Cap at FRAME_HZ; while a full-resolution globe is rendered in the background (still
        // views), leave it a share of both cores.
        int64_t spent = esp_timer_get_time() - now;
        int min_ms = scene_background_busy() ? 8 : 1;
        int wait_ms = (int)((1000000 / FRAME_HZ - spent) / 1000);
        vTaskDelay(pdMS_TO_TICKS(wait_ms > min_ms ? wait_ms : min_ms));

        frames++;
        half_frames += tm.half_res;
        if (tm.total_ms > worst_ms) worst_ms = tm.total_ms;
        sum.base_ms += tm.base_ms; sum.lines_ms += tm.lines_ms; sum.overlay_ms += tm.overlay_ms;
        sum.total_ms += tm.total_ms;
        sum.px_ms += tm.px_ms; sum.replay_ms += tm.replay_ms; sum.copy_ms += tm.copy_ms;
        if (want_stats || (!stats_on_demand && now - stats_at > 5000000)) {
            want_stats = false;
            float secs = (now - stats_at) / 1e6f;
            scene_sat_t s = scene_sat();
            float rot[3];
            scene_rotation(rot);
            int redraws;
            float redraw_ms;
            scene_globe_stats(&redraws, &redraw_ms);
            ESP_LOGI(TAG, "STATS %s %.1f fps | gap max %.0f ms | frame %.1f ms, worst %.1f (record %.1f, strips %.1f: px %.1f replay %.1f copy %.1f) | "
                     "half-res %d/%d | full-res renders %d x %.0f ms",
                     scene_mode_name(), frames / secs, worst_gap_ms, sum.total_ms / frames, worst_ms, sum.base_ms / frames, sum.lines_ms / frames,
                     sum.px_ms / frames, sum.replay_ms / frames, sum.copy_ms / frames, half_frames, frames, redraws, redraw_ms);
            int tv[2], tmiss[2];
            tiles_stats(tv, tmiss);
            ESP_LOGI(TAG, "PANEL refresh max %.1f ms, late refreshes %lu, frames off target %d, strips %s, tile decodes %d, tiles in view %d+%d, missing %d+%d",
                     refresh_max_us / 1000.0f, (unsigned long)refresh_late, missed, scene_strips_fast() ? "fast" : "SLOW RAM", tiles_decoded(), tv[0], tv[1], tmiss[0], tmiss[1]);
            missed = 0;
            refresh_max_us = 0;
            refresh_late = 0;
            ESP_LOGI(TAG, "SAT t=%.0f %s lat=%.3f lon=%.3f alt=%.1f rot=[%.2f %.2f %.2f]", now_unix(),
                     scene_mode_name(), s.lat, s.lon, s.alt, rot[0], rot[1], rot[2]);
            frames = half_frames = 0;
            worst_ms = worst_gap_ms = 0;
            memset(&sum, 0, sizeof sum);
            stats_at = now;
        }
    }
}
