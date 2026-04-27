idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_wifi esp_netif esp_event esp_timer mqtt
             siri_ble report_decoder event_state
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
