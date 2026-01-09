idf_component_register(SRCS "main.c"
                    REQUIRES ili9341 xpt2046
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "")
