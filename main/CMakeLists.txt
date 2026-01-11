idf_component_register(SRCS "main.c" "wifi.c" "bluetooth.c"
                    REQUIRES ili9341 xpt2046 esp_wifi nvs_flash bt lvgl__lvgl
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS ".")
