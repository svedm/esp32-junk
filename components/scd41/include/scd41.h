#ifndef SCD41_H
#define SCD41_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t co2_ppm;
    int32_t temperature_m_deg_c;   // milli-degrees Celsius (25123 = 25.123 C)
    int32_t humidity_m_percent_rh; // milli-percent RH (50500 = 50.5 %RH)
    bool valid;
} scd41_data_t;

/**
 * @brief Initialize I2C bus and SCD41 sensor
 * @return ESP_OK on success
 */
esp_err_t scd41_init(void);

/**
 * @brief Start periodic measurements and background reading task
 * @return ESP_OK on success
 */
esp_err_t scd41_start(void);

/**
 * @brief Get the latest measurement data (thread-safe)
 * @return Copy of the latest sensor data
 */
scd41_data_t scd41_get_data(void);

#ifdef __cplusplus
}
#endif

#endif // SCD41_H
