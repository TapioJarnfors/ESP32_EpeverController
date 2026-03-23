idf_component_register(SRCS "main.c" "epever_data.c" "wifi_manager.c" "mqtt_manager.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos esp_timer driver log esp_wifi esp_event nvs_flash mqtt)