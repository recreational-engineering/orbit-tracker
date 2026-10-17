# SGP4.cpp / SGP4.h: David Vallado's reference SGP4, as distributed with
# python-sgp4 2.27 (MIT, see LICENSE). orbit.cpp wraps it for C.
idf_component_register(SRCS "SGP4.cpp" "orbit.cpp"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ".")
# Reference code kept verbatim: silence its warnings instead of patching it.
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
