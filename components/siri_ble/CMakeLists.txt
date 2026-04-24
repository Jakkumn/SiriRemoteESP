idf_component_register(
    SRCS "siri_ble.c" "peer.c"
    INCLUDE_DIRS "include" "."
    REQUIRES bt nvs_flash
)

# Enforce strict warnings on our own code. peer.c is copied from IDF's
# nimble_central_utils example and only gets IDF's defaults.
set_source_files_properties(siri_ble.c PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra;-Werror"
)
