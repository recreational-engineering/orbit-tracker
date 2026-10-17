# Assets in generated/ come from tools/gen_assets.py (run it before the first build).
# Adding any REQUIRES turns off main's implicit "depend on every component", so list them all.
idf_component_register(SRCS "main.c" "gfx.c" "globe.c" "dlist.c" "model.c" "scene.c" "tiles.c" "net.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES sgp4 esp32_p4_wifi6_touch_lcd_xc esp_lcd esp_lcd_touch_gt911 esp_hosted
                                     esp_driver_uart esp_driver_jpeg esp_timer esp_partition esp_mm
                                     nvs_flash esp_netif esp_event esp_http_client esp-tls
                                     esp_wifi esp_wifi_remote
                       LDFRAGMENTS "linker.lf"
                       EMBED_FILES "generated/texture.bin" "generated/texture_half.bin" "generated/lights.bin"
                                   "generated/borders.bin" "generated/graticule.bin")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffast-math)
