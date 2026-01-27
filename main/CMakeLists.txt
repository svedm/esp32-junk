idf_component_register(SRCS "app_time.c" "network.c" "main.c" "wifi.c" "bluetooth.c" "network.c"
                    REQUIRES ili9341 xpt2046 esp_wifi nvs_flash bt lvgl__lvgl esp-tls esp_event esp_netif esp_http_client
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS ".")
