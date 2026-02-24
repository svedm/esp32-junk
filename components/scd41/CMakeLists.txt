idf_component_register(SRCS "scd41.c" "scd4x_i2c.c" "sensirion_common.c" "sensirion_i2c.c" "sensirion_i2c_hal.c"
                    INCLUDE_DIRS "include" "."
                    REQUIRES driver)
