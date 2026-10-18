/* Panel refresh rate. The board package creates the JD9365 panel with an 80 MHz pixel clock,
 * which scans the frame buffer out of PSRAM ~131 times a second (~136 MB/s, the same memory
 * every frame is rendered through). The linker wraps esp_lcd_new_panel_jd9365 (see
 * CMakeLists.txt) so the panel gets PANEL_DPI_MHZ instead: ~65 Hz, plenty for a display that
 * renders 15-30 frames a second, and it frees that bandwidth for rendering. */
#include "esp_lcd_jd9365.h"
#include "esp_lcd_mipi_dsi.h"
#include "panel_rate.h"

esp_err_t __real_esp_lcd_new_panel_jd9365(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *cfg,
                                          esp_lcd_panel_handle_t *ret_panel);

esp_err_t __wrap_esp_lcd_new_panel_jd9365(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *cfg,
                                          esp_lcd_panel_handle_t *ret_panel)
{
    static esp_lcd_dpi_panel_config_t dpi;
    static jd9365_vendor_config_t vendor;
    static esp_lcd_panel_dev_config_t dev;
    const jd9365_vendor_config_t *v = cfg->vendor_config;
    if (!v || !v->mipi_config.dpi_config) return __real_esp_lcd_new_panel_jd9365(io, cfg, ret_panel);
    dpi = *v->mipi_config.dpi_config;
    dpi.dpi_clock_freq_mhz = PANEL_DPI_MHZ;
    vendor = *v;
    vendor.mipi_config.dpi_config = &dpi;
    dev = *cfg;
    dev.vendor_config = &vendor;
    return __real_esp_lcd_new_panel_jd9365(io, &dev, ret_panel);
}
