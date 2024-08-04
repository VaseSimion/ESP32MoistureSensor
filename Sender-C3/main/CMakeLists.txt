idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_adc esp_littlefs nvs_flash)