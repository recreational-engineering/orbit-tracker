# Assets in generated/ come from tools/gen_assets.py (run it before the first build).
idf_component_register(SRCS "main.c" "gfx.c" "globe.c" "dlist.c" "model.c" "scene.c" "tiles.c"
                       INCLUDE_DIRS "."
                       LDFRAGMENTS "linker.lf"
                       EMBED_FILES "generated/texture.bin" "generated/texture_half.bin" "generated/lights.bin"
                                   "generated/borders.bin" "generated/graticule.bin")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffast-math)
