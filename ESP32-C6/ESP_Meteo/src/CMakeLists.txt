# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

idf_component_register(SRCS "main.c"
                       REQUIRES bme280 i2c_bus driver esp_adc esp_hw_support esp_wifi esp_event esp_netif nvs_flash esp_http_client)
