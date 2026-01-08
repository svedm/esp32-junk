/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ili9341.h"

static const char *TAG = "ILI9341_DEMO";

// Настройки пинов для дисплея ILI9341
#define LCD_HOST       SPI2_HOST
#define PIN_NUM_MOSI   7
#define PIN_NUM_CLK    6
#define PIN_NUM_CS     17
#define PIN_NUM_DC     15
#define PIN_NUM_RST    16
#define PIN_NUM_BCKL   5

// Функция для рисования демонстрационного паттерна
static void draw_demo_pattern(ili9341_t *lcd)
{
    ESP_LOGI(TAG, "Drawing demo pattern...");

    // Заполняем фон белым цветом
    ili9341_fill_screen(lcd, ILI9341_WHITE);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Рисуем разноцветные полосы
    int stripe_height = 320 / 6;
    ili9341_fill_rect(lcd, 0, 0 * stripe_height, 240, stripe_height, ILI9341_RED);
    ili9341_fill_rect(lcd, 0, 1 * stripe_height, 240, stripe_height, ILI9341_GREEN);
    ili9341_fill_rect(lcd, 0, 2 * stripe_height, 240, stripe_height, ILI9341_BLUE);
    ili9341_fill_rect(lcd, 0, 3 * stripe_height, 240, stripe_height, ILI9341_YELLOW);
    ili9341_fill_rect(lcd, 0, 4 * stripe_height, 240, stripe_height, ILI9341_CYAN);
    ili9341_fill_rect(lcd, 0, 5 * stripe_height, 240, stripe_height, ILI9341_MAGENTA);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Рисуем квадраты по углам
    ili9341_fill_screen(lcd, ILI9341_BLACK);
    ili9341_fill_rect(lcd, 10, 10, 60, 60, ILI9341_RED);
    ili9341_fill_rect(lcd, 240 - 70, 10, 60, 60, ILI9341_GREEN);
    ili9341_fill_rect(lcd, 10, 320 - 70, 60, 60, ILI9341_BLUE);
    ili9341_fill_rect(lcd, 240 - 70, 320 - 70, 60, 60, ILI9341_YELLOW);

    // Рисуем квадрат в центре
    ili9341_fill_rect(lcd, 240/2 - 40, 320/2 - 40, 80, 80, ILI9341_WHITE);
}


static void draw_text_demo(ili9341_t *lcd)
{
    ESP_LOGI(TAG, "Drawing text demo with Russian support");

    // Заливаем экран черным цветом
    ili9341_fill_screen(lcd, ILI9341_BLACK);

    // Рисуем английский текст разного размера
    ili9341_draw_string(lcd, 10, 10, "Hello World!", ILI9341_WHITE, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 25, "ESP32-S3", ILI9341_CYAN, ILI9341_BLACK, 2);

    // Рисуем русский текст
    ili9341_draw_string(lcd, 10, 55, "Привет Мир!", ILI9341_GREEN, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 80, "Тест кириллицы", ILI9341_YELLOW, ILI9341_BLACK, 1);

    // Разные цвета на русском
    ili9341_draw_string(lcd, 10, 100, "КРАСНЫЙ", ILI9341_RED, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 120, "ЗЕЛЕНЫЙ", ILI9341_GREEN, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 140, "СИНИЙ", ILI9341_BLUE, ILI9341_BLACK, 2);

    // Русский алфавит (выборочно)
    ili9341_draw_string(lcd, 10, 170, "АБВГДЕЖЗ", ILI9341_MAGENTA, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 185, "абвгдежз", ILI9341_MAGENTA, ILI9341_BLACK, 1);

    // Цифры
    ili9341_draw_string(lcd, 10, 210, "0123456789", ILI9341_CYAN, ILI9341_BLACK, 2);

    // Смешанный текст
    ili9341_draw_string(lcd, 10, 240, "ESP32 + Русский", ILI9341_WHITE, ILI9341_BLACK, 2);

    // Информация внизу
    ili9341_draw_string(lcd, 10, 280, "ILI9341 Driver", ILI9341_ORANGE, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 295, "UTF-8 Support", ILI9341_ORANGE, ILI9341_BLACK, 1);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ILI9341 LCD Demo with custom driver");
    ESP_LOGI(TAG, "Pin configuration: MOSI=%d, CLK=%d, CS=%d, DC=%d, RST=%d, BCKL=%d",
             PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_RST, PIN_NUM_BCKL);

    // Создаем структуру дисплея
    ili9341_t lcd;

    // Инициализация дисплея
    esp_err_t ret = ili9341_init(&lcd, LCD_HOST, PIN_NUM_MOSI, PIN_NUM_CLK,
                                 PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_RST, PIN_NUM_BCKL);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ILI9341: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "LCD initialized successfully!");

    // Включаем подсветку
    ili9341_backlight(&lcd, true);
    ESP_LOGI(TAG, "Backlight ON");

    // Рисуем демонстрационный текст
    draw_text_demo(&lcd);
}
