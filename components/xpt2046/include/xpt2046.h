#ifndef XPT2046_H
#define XPT2046_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

// XPT2046 команды
#define XPT2046_START   0x80
#define XPT2046_XPOS    0x50
#define XPT2046_YPOS    0x10
#define XPT2046_ZPOS    0x30

// Режимы работы
#define XPT2046_MODE_12BIT  0x00
#define XPT2046_MODE_8BIT   0x08
#define XPT2046_DIFFERENTIAL 0x00
#define XPT2046_SINGLE_ENDED 0x04

// Структура данных тачскрина
typedef struct {
    spi_device_handle_t spi;
    int8_t pin_cs;
    int8_t pin_irq;
    uint16_t width;       // Ширина дисплея
    uint16_t height;      // Высота дисплея

    // Калибровка
    int16_t cal_x_min;
    int16_t cal_x_max;
    int16_t cal_y_min;
    int16_t cal_y_max;

    // Флаги
    bool rotation;        // Поворот на 90 градусов
    bool invert_x;        // Инвертировать ось X
    bool invert_y;        // Инвертировать ось Y
    bool swap_xy;         // Поменять местами X и Y
} xpt2046_t;

// Структура для координат касания
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;  // Давление
    bool touched;
} xpt2046_touch_t;

// Функции API
esp_err_t xpt2046_init(xpt2046_t *touch, spi_host_device_t host,
                       int8_t pin_miso, int8_t pin_mosi, int8_t pin_clk,
                       int8_t pin_cs, int8_t pin_irq,
                       uint16_t width, uint16_t height);

bool xpt2046_is_touched(xpt2046_t *touch);
xpt2046_touch_t xpt2046_get_touch(xpt2046_t *touch);
uint16_t xpt2046_read_raw(xpt2046_t *touch, uint8_t command);

// Калибровка
void xpt2046_set_calibration(xpt2046_t *touch,
                             int16_t x_min, int16_t x_max,
                             int16_t y_min, int16_t y_max);
void xpt2046_set_rotation(xpt2046_t *touch, bool rotate);
void xpt2046_set_transform(xpt2046_t *touch, bool invert_x, bool invert_y, bool swap_xy);

#endif // XPT2046_H
