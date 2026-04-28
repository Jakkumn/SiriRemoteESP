idf_component_register(SRCS "mqtt_entity.c"
                       INCLUDE_DIRS "include"
                       REQUIRES mqtt nvs_flash esp_common log)
