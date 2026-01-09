#include "xpt2046.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "XPT2046";

// Внутренняя функция для чтения данных через SPI
static uint16_t xpt2046_spi_read(xpt2046_t *touch, uint8_t command)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t tx_data[3] = {command, 0x00, 0x00};
    uint8_t rx_data[3] = {0};

    t.length = 24; // 3 байта
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    esp_err_t ret = spi_device_polling_transmit(touch->spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        return 0;
    }

    // Данные приходят в формате: [X, D11-D4, D3-D0]
    uint16_t result = ((uint16_t)rx_data[1] << 4) | (rx_data[2] >> 4);
    return result;
}

esp_err_t xpt2046_init(xpt2046_t *touch, spi_host_device_t host,
                       int8_t pin_miso, int8_t pin_mosi, int8_t pin_clk,
                       int8_t pin_cs, int8_t pin_irq,
                       uint16_t width, uint16_t height)
{
    if (touch == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing XPT2046 touchscreen");
    ESP_LOGI(TAG, "Pins: MISO=%d, MOSI=%d, CLK=%d, CS=%d, IRQ=%d",
             pin_miso, pin_mosi, pin_clk, pin_cs, pin_irq);

    touch->pin_cs = pin_cs;
    touch->pin_irq = pin_irq;
    touch->width = width;
    touch->height = height;
    touch->rotation = false;
    touch->invert_x = false;
    touch->invert_y = false;
    touch->swap_xy = false;

    // Значения калибровки по умолчанию (будут настроены под конкретный экран)
    touch->cal_x_min = 200;
    touch->cal_x_max = 3900;
    touch->cal_y_min = 200;
    touch->cal_y_max = 3900;

    // Инициализация SPI шины для тачскрина (если еще не инициализирована)
    spi_bus_config_t buscfg = {
        .miso_io_num = pin_miso,
        .mosi_io_num = pin_mosi,
        .sclk_io_num = pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    // Пытаемся инициализировать SPI шину
    esp_err_t ret = spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        // ESP_ERR_INVALID_STATE означает, что шина уже инициализирована - это нормально
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    if (ret == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGI(TAG, "SPI bus already initialized (shared with display)");
    }

    // Настройка SPI устройства для touchscreen
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = pin_cs,
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL,
    };

    ret = spi_bus_add_device(host, &devcfg, &touch->spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Настройка пина IRQ как вход
    if (pin_irq >= 0)
    {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << pin_irq),
            .pull_down_en = 0,
            .pull_up_en = 1,
        };
        gpio_config(&io_conf);
    }

    ESP_LOGI(TAG, "XPT2046 initialized successfully");
    return ESP_OK;
}

bool xpt2046_is_touched(xpt2046_t *touch)
{
    if (touch == NULL)
    {
        return false;
    }

    // Если есть IRQ пин, проверяем его состояние
    if (touch->pin_irq >= 0)
    {
        // IRQ активен на низком уровне при касании
        return gpio_get_level(touch->pin_irq) == 0;
    }

    // Если IRQ нет, проверяем давление
    uint16_t z = xpt2046_read_raw(touch, XPT2046_START | XPT2046_ZPOS);
    return z > 100; // Порог давления
}

uint16_t xpt2046_read_raw(xpt2046_t *touch, uint8_t command)
{
    if (touch == NULL)
    {
        return 0;
    }

    // Читаем несколько раз и усредняем для стабильности
    const int samples = 3;
    uint32_t sum = 0;

    for (int i = 0; i < samples; i++)
    {
        uint16_t value = xpt2046_spi_read(touch, command);
        sum += value;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return sum / samples;
}

xpt2046_touch_t xpt2046_get_touch(xpt2046_t *touch)
{
    xpt2046_touch_t result = {0};

    if (touch == NULL)
    {
        return result;
    }

    // Читаем сырые координаты
    uint16_t raw_x = xpt2046_read_raw(touch, XPT2046_START | XPT2046_XPOS);
    uint16_t raw_y = xpt2046_read_raw(touch, XPT2046_START | XPT2046_YPOS);
    uint16_t raw_z = xpt2046_read_raw(touch, XPT2046_START | XPT2046_ZPOS);

    // Проверяем валидность данных
    if (raw_z < 100)
    { // Слишком слабое нажатие
        result.touched = false;
        return result;
    }

    // Калибруем и масштабируем координаты
    int32_t x = raw_x - touch->cal_x_min;
    x = x * touch->width / (touch->cal_x_max - touch->cal_x_min);

    int32_t y = raw_y - touch->cal_y_min;
    y = y * touch->height / (touch->cal_y_max - touch->cal_y_min);

    // Применяем инверсию осей
    if (touch->invert_x)
    {
        x = touch->width - 1 - x;
    }
    if (touch->invert_y)
    {
        y = touch->height - 1 - y;
    }

    // Меняем местами X и Y если нужно
    if (touch->swap_xy)
    {
        int32_t temp = x;
        x = y;
        y = temp;
    }

    // Применяем поворот если нужно (устаревший метод, используйте swap_xy и invert вместо этого)
    if (touch->rotation)
    {
        int32_t temp = x;
        x = y;
        y = touch->width - temp;
    }

    // Ограничиваем координаты в пределах экрана
    if (x < 0)
        x = 0;
    if (x >= touch->width)
        x = touch->width - 1;
    if (y < 0)
        y = 0;
    if (y >= touch->height)
        y = touch->height - 1;

    result.x = (uint16_t)x;
    result.y = (uint16_t)y;
    result.z = raw_z;
    result.touched = true;

    // Отладочный лог для проверки координат (можно убрать позже)
    static int touch_log_counter = 0;
    if (touch_log_counter++ % 20 == 0)
    {
        ESP_LOGI(TAG, "Touch: raw(%d,%d) -> cal(%d,%d)", raw_x, raw_y, result.x, result.y);
    }

    return result;
}

void xpt2046_set_calibration(xpt2046_t *touch,
                             int16_t x_min, int16_t x_max,
                             int16_t y_min, int16_t y_max)
{
    if (touch == NULL)
    {
        return;
    }

    touch->cal_x_min = x_min;
    touch->cal_x_max = x_max;
    touch->cal_y_min = y_min;
    touch->cal_y_max = y_max;

    ESP_LOGI(TAG, "Calibration set: X[%d-%d], Y[%d-%d]",
             x_min, x_max, y_min, y_max);
}

void xpt2046_set_rotation(xpt2046_t *touch, bool rotate)
{
    if (touch == NULL)
    {
        return;
    }

    touch->rotation = rotate;
    ESP_LOGI(TAG, "Rotation %s", rotate ? "enabled" : "disabled");
}

void xpt2046_set_transform(xpt2046_t *touch, bool invert_x, bool invert_y, bool swap_xy)
{
    if (touch == NULL)
    {
        return;
    }

    touch->invert_x = invert_x;
    touch->invert_y = invert_y;
    touch->swap_xy = swap_xy;

    ESP_LOGI(TAG, "Transform set: invert_x=%d, invert_y=%d, swap_xy=%d",
             invert_x, invert_y, swap_xy);
}
