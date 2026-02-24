#include "scd41.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "scd41";

static scd41_data_t s_data = {0};
static SemaphoreHandle_t s_mutex = NULL;

static void scd41_task(void *pvParameter)
{
    (void)pvParameter;

    ESP_LOGI(TAG, "Measurement task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_SCD41_MEASUREMENT_INTERVAL_MS));

        bool data_ready = false;
        int16_t err = scd4x_get_data_ready_status(&data_ready);
        if (err != NO_ERROR) {
            ESP_LOGW(TAG, "get_data_ready_status failed: %d", err);
            continue;
        }

        if (!data_ready) {
            continue;
        }

        uint16_t co2 = 0;
        int32_t temp = 0;
        int32_t hum = 0;
        err = scd4x_read_measurement(&co2, &temp, &hum);
        if (err != NO_ERROR) {
            ESP_LOGW(TAG, "read_measurement failed: %d", err);
            continue;
        }

        if (co2 == 0) {
            continue;
        }

        ESP_LOGI(TAG, "CO2: %u ppm, T: %.2f C, RH: %.2f %%",
                 co2, temp / 1000.0f, hum / 1000.0f);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_data.co2_ppm = co2;
        s_data.temperature_m_deg_c = temp;
        s_data.humidity_m_percent_rh = hum;
        s_data.valid = true;
        xSemaphoreGive(s_mutex);
    }
}

esp_err_t scd41_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    sensirion_i2c_hal_init();
    scd4x_init(SCD41_I2C_ADDR_62);

    // Stop any potentially running measurement
    scd4x_stop_periodic_measurement();

    // Read and log serial number
    uint16_t serial[3] = {0};
    int16_t err = scd4x_get_serial_number(serial, 3);
    if (err == NO_ERROR) {
        ESP_LOGI(TAG, "SCD41 serial: 0x%04x%04x%04x",
                 serial[0], serial[1], serial[2]);
    } else {
        ESP_LOGW(TAG, "Failed to read serial number: %d", err);
    }

    ESP_LOGI(TAG, "Initialized");
    return ESP_OK;
}

esp_err_t scd41_start(void)
{
    int16_t err = scd4x_start_periodic_measurement();
    if (err != NO_ERROR) {
        ESP_LOGE(TAG, "start_periodic_measurement failed: %d", err);
        return ESP_FAIL;
    }

    BaseType_t ret = xTaskCreate(scd41_task, "scd41_task", 4096, NULL, 3, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Periodic measurement started");
    return ESP_OK;
}

scd41_data_t scd41_get_data(void)
{
    scd41_data_t copy = {0};
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        copy = s_data;
        xSemaphoreGive(s_mutex);
    }
    return copy;
}
