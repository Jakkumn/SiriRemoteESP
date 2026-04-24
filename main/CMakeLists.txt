idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt siri_ble
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
