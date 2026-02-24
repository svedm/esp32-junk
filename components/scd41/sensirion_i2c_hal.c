#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensirion_hal";

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t i2c_dev = NULL;

int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
    (void)bus_idx;
    return NO_ERROR;
}

void sensirion_i2c_hal_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_SCD41_I2C_SDA_PIN,
        .scl_io_num = CONFIG_SCD41_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x62,
        .scl_speed_hz = CONFIG_SCD41_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(i2c_bus, &dev_config, &i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "I2C initialized (SDA=%d, SCL=%d, freq=%d Hz)",
             CONFIG_SCD41_I2C_SDA_PIN, CONFIG_SCD41_I2C_SCL_PIN,
             CONFIG_SCD41_I2C_FREQ_HZ);
}

void sensirion_i2c_hal_free(void) {
    if (i2c_dev) {
        i2c_master_bus_rm_device(i2c_dev);
        i2c_dev = NULL;
    }
    if (i2c_bus) {
        i2c_del_master_bus(i2c_bus);
        i2c_bus = NULL;
    }
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    (void)address;
    if (!i2c_dev) {
        return I2C_BUS_ERROR;
    }
    esp_err_t err = i2c_master_receive(i2c_dev, data, count, 1000);
    if (err != ESP_OK) {
        return I2C_NACK_ERROR;
    }
    return NO_ERROR;
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data,
                               uint8_t count) {
    (void)address;
    if (!i2c_dev) {
        return I2C_BUS_ERROR;
    }
    esp_err_t err = i2c_master_transmit(i2c_dev, data, count, 1000);
    if (err != ESP_OK) {
        return I2C_NACK_ERROR;
    }
    return NO_ERROR;
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    uint32_t ms = useconds / 1000;
    if (ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        esp_rom_delay_us(useconds);
    }
}
