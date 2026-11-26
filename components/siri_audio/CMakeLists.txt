idf_component_register(SRCS "siri_audio.c" "siri_audio_validate.c" "siri_audio_pcm_tcp.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_common log freertos esp_timer
                       PRIV_REQUIRES nvs_flash lwip)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
