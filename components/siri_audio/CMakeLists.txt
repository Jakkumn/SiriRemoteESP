idf_component_register(SRCS "siri_audio.c" "siri_audio_validate.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_common log freertos esp_timer
                       PRIV_REQUIRES nvs_flash)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
