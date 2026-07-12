idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_wifi esp_driver_mcpwm esp_driver_gpio nvs_flash esp_netif esp_adc esp_driver_ledc
                    INCLUDE_DIRS ".")

littlefs_create_partition_image(storage ${CMAKE_CURRENT_SOURCE_DIR}/../files FLASH_IN_PROJECT)