/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ili9341.h"
#include "xpt2046.h"
#include "wifi.h"
#include "esp_wifi.h"
#include <string.h>
#include "bluetooth.h"

static const char *TAG = "ILI9341_DEMO";

// Настройки пинов для дисплея ILI9341
#define LCD_HOST       SPI2_HOST
#define PIN_NUM_MOSI   7
#define PIN_NUM_CLK    6
#define PIN_NUM_CS     17
#define PIN_NUM_DC     15
#define PIN_NUM_RST    16
#define PIN_NUM_BCKL   5

// Настройки пинов для тачскрина XPT2046
// Используем SPI3_HOST так как пины отличаются от дисплея
#define TOUCH_HOST     SPI3_HOST
#define TOUCH_MISO     2   // T_DO
#define TOUCH_MOSI     42  // T_DIN
#define TOUCH_CLK      40  // T_CLK
#define TOUCH_CS       41  // T_CS
#define TOUCH_IRQ      1   // T_IRQ

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

// Функция калибровки тачскрина
static void calibrate_touchscreen(ili9341_t *lcd, xpt2046_t *touch)
{
    ESP_LOGI(TAG, "Starting touchscreen calibration");

    // Точки калибровки: верхний левый, верхний правый, нижний правый, нижний левый, центр
    const struct {
        uint16_t x;
        uint16_t y;
        const char *name;
    } calibration_points[] = {
        {20, 20, "Top-Left"},
        {220, 20, "Top-Right"},
        {220, 300, "Bottom-Right"},
        {20, 300, "Bottom-Left"},
        {120, 160, "Center"}
    };

    uint16_t raw_x_values[5];
    uint16_t raw_y_values[5];

    for (int i = 0; i < 5; i++) {
        // Очищаем экран
        ili9341_fill_screen(lcd, ILI9341_WHITE);

        // Рисуем инструкцию
        char instruction[50];
        sprintf(instruction, "Touch %s point", calibration_points[i].name);
        ili9341_draw_string(lcd, 10, 10, instruction, ILI9341_BLACK, ILI9341_WHITE, 1);

        // Рисуем мишень (крест)
        uint16_t cx = calibration_points[i].x;
        uint16_t cy = calibration_points[i].y;

        // Рисуем крест
        for (int d = -15; d <= 15; d++) {
            if (cx + d >= 0 && cx + d < 240) {
                ili9341_draw_pixel(lcd, cx + d, cy, ILI9341_RED);
                ili9341_draw_pixel(lcd, cx + d, cy - 1, ILI9341_RED);
                ili9341_draw_pixel(lcd, cx + d, cy + 1, ILI9341_RED);
            }
            if (cy + d >= 0 && cy + d < 320) {
                ili9341_draw_pixel(lcd, cx, cy + d, ILI9341_RED);
                ili9341_draw_pixel(lcd, cx - 1, cy + d, ILI9341_RED);
                ili9341_draw_pixel(lcd, cx + 1, cy + d, ILI9341_RED);
            }
        }

        // Рисуем центральную точку
        for (int dy = -3; dy <= 3; dy++) {
            for (int dx = -3; dx <= 3; dx++) {
                if (cx + dx >= 0 && cx + dx < 240 && cy + dy >= 0 && cy + dy < 320) {
                    ili9341_draw_pixel(lcd, cx + dx, cy + dy, ILI9341_RED);
                }
            }
        }

        ESP_LOGI(TAG, "Waiting for touch on %s (%d, %d)...",
                 calibration_points[i].name, cx, cy);

        // Ждем касания
        bool touched = false;
        while (!touched) {
            // Читаем сырые значения напрямую
            uint16_t raw_x = xpt2046_read_raw(touch, 0xD0);  // X position
            uint16_t raw_y = xpt2046_read_raw(touch, 0x90);  // Y position
            uint16_t raw_z = xpt2046_read_raw(touch, 0xB0);  // Pressure

            if (raw_z > 100) {
                raw_x_values[i] = raw_x;
                raw_y_values[i] = raw_y;

                ESP_LOGI(TAG, "Point %d (%s): raw_x=%d, raw_y=%d, raw_z=%d",
                         i, calibration_points[i].name, raw_x, raw_y, raw_z);

                // Показываем подтверждение
                ili9341_fill_rect(lcd, cx - 5, cy - 5, 10, 10, ILI9341_GREEN);
                vTaskDelay(pdMS_TO_TICKS(500));

                // Ждем отпускания
                while (raw_z > 100) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    raw_z = xpt2046_read_raw(touch, 0xB0);
                }

                touched = true;
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    // Вычисляем калибровочные значения
    ESP_LOGI(TAG, "\n========== CALIBRATION RESULTS ==========");
    ESP_LOGI(TAG, "Point 0 (Top-Left):     raw_x=%d, raw_y=%d", raw_x_values[0], raw_y_values[0]);
    ESP_LOGI(TAG, "Point 1 (Top-Right):    raw_x=%d, raw_y=%d", raw_x_values[1], raw_y_values[1]);
    ESP_LOGI(TAG, "Point 2 (Bottom-Right): raw_x=%d, raw_y=%d", raw_x_values[2], raw_y_values[2]);
    ESP_LOGI(TAG, "Point 3 (Bottom-Left):  raw_x=%d, raw_y=%d", raw_x_values[3], raw_y_values[3]);
    ESP_LOGI(TAG, "Point 4 (Center):       raw_x=%d, raw_y=%d", raw_x_values[4], raw_y_values[4]);

    // Находим минимумы и максимумы
    uint16_t x_min = raw_x_values[0];
    uint16_t x_max = raw_x_values[0];
    uint16_t y_min = raw_y_values[0];
    uint16_t y_max = raw_y_values[0];

    for (int i = 1; i < 5; i++) {
        if (raw_x_values[i] < x_min) x_min = raw_x_values[i];
        if (raw_x_values[i] > x_max) x_max = raw_x_values[i];
        if (raw_y_values[i] < y_min) y_min = raw_y_values[i];
        if (raw_y_values[i] > y_max) y_max = raw_y_values[i];
    }

    ESP_LOGI(TAG, "\nRecommended calibration values:");
    ESP_LOGI(TAG, "xpt2046_set_calibration(&touch, %d, %d, %d, %d);",
             x_min, x_max, y_min, y_max);
    ESP_LOGI(TAG, "==========================================\n");

    // Показываем результаты на экране
    ili9341_fill_screen(lcd, ILI9341_WHITE);
    ili9341_draw_string(lcd, 10, 10, "Calibration Done!", ILI9341_GREEN, ILI9341_WHITE, 2);

    char result[60];
    sprintf(result, "X: %d - %d", x_min, x_max);
    ili9341_draw_string(lcd, 10, 40, result, ILI9341_BLACK, ILI9341_WHITE, 1);

    sprintf(result, "Y: %d - %d", y_min, y_max);
    ili9341_draw_string(lcd, 10, 55, result, ILI9341_BLACK, ILI9341_WHITE, 1);

    ili9341_draw_string(lcd, 10, 80, "Check serial output", ILI9341_BLUE, ILI9341_WHITE, 1);
    ili9341_draw_string(lcd, 10, 95, "for calibration code", ILI9341_BLUE, ILI9341_WHITE, 1);

    xpt2046_set_calibration(touch, x_min, x_max, y_min, y_max);

    vTaskDelay(pdMS_TO_TICKS(5000));
}

static void draw_text_demo(ili9341_t *lcd, xpt2046_t *touch)
{
    ESP_LOGI(TAG, "Drawing text demo with Russian support");

    // Заливаем экран черным цветом
    ili9341_fill_screen(lcd, ILI9341_BLACK);

    // Рисуем английский текст разного размера
    ili9341_draw_string(lcd, 10, 10, "Hello World!", ILI9341_WHITE, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 25, "ESP32-S3", ILI9341_CYAN, ILI9341_BLACK, 2);

    // Рисуем русский текст
    ili9341_draw_string(lcd, 10, 55, "Hello, Владислав!", ILI9341_GREEN, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 80, "Тест тестович", ILI9341_YELLOW, ILI9341_BLACK, 1);

    // Разные цвета на русском
    ili9341_draw_string(lcd, 10, 100, "КРАСНЫЙ", ILI9341_RED, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 120, "ЗЕЛЕНЫЙ", ILI9341_GREEN, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 140, "СИНИЙ", ILI9341_BLUE, ILI9341_BLACK, 2);

    // Русский алфавит (выборочно)
    ili9341_draw_string(lcd, 10, 170, "АБВГДЕЖЗ", ILI9341_MAGENTA, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 185, "абвгдежз", ILI9341_MAGENTA, ILI9341_BLACK, 1);

    // Цифры
    ili9341_draw_string(lcd, 10, 210, "0123456789", ILI9341_CYAN, ILI9341_BLACK, 2);

    // Информация внизу
    ili9341_draw_string(lcd, 10, 280, "ILI9341 Driver", ILI9341_ORANGE, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 295, "UTF-8 Support", ILI9341_ORANGE, ILI9341_BLACK, 1);

    if (touch == NULL) {
        return;
    }

    while (1)
    {
        xpt2046_touch_t touch_data = xpt2046_get_touch(touch);
        if (touch_data.touched) {
            ili9341_draw_pixel(lcd, touch_data.x, touch_data.y, ILI9341_WHITE);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Функция для рисования на экране с помощью тачскрина
static void touch_draw_mode(ili9341_t *lcd, xpt2046_t *touch)
{
    ESP_LOGI(TAG, "Starting touch draw mode");

    // Очищаем экран белым цветом
    ili9341_fill_screen(lcd, ILI9341_WHITE);

    // Рисуем инструкцию
    ili9341_draw_string(lcd, 10, 10, "Touch to draw!", ILI9341_BLACK, ILI9341_WHITE, 2);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Очищаем экран для рисования
    ili9341_fill_screen(lcd, ILI9341_WHITE);

    // Переменные для отслеживания предыдущей позиции (для рисования линий)
    int16_t last_x = -1;
    int16_t last_y = -1;
    bool was_touched = false;

    // Размер кисти
    const int brush_size = 3;

    ESP_LOGI(TAG, "Ready to draw! Touch the screen...");

    while (1) {
        xpt2046_touch_t touch_data = xpt2046_get_touch(touch);

        if (touch_data.touched) {

            // Рисуем точку или линию
            if (was_touched && last_x >= 0 && last_y >= 0) {
                // Рисуем линию от предыдущей точки к текущей (плавное рисование)
                int dx = touch_data.x - last_x;
                int dy = touch_data.y - last_y;
                int steps = (abs(dx) > abs(dy)) ? abs(dx) : abs(dy);

                if (steps > 0) {
                    for (int i = 0; i <= steps; i++) {
                        int x = last_x + (dx * i) / steps;
                        int y = last_y + (dy * i) / steps;

                        // Рисуем круглую кисть
                        for (int by = -brush_size; by <= brush_size; by++) {
                            for (int bx = -brush_size; bx <= brush_size; bx++) {
                                if (bx*bx + by*by <= brush_size*brush_size) {
                                    int px = x + bx;
                                    int py = y + by;
                                    if (px >= 0 && px < 240 && py >= 0 && py < 320) {
                                        ili9341_draw_pixel(lcd, px, py, ILI9341_BLUE);
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                // Первая точка - просто рисуем круг
                for (int by = -brush_size; by <= brush_size; by++) {
                    for (int bx = -brush_size; bx <= brush_size; bx++) {
                        if (bx*bx + by*by <= brush_size*brush_size) {
                            int px = touch_data.x + bx;
                            int py = touch_data.y + by;
                            if (px >= 0 && px < 240 && py >= 0 && py < 320) {
                                ili9341_draw_pixel(lcd, px, py, ILI9341_BLUE);
                            }
                        }
                    }
                }
            }

            last_x = touch_data.x;
            last_y = touch_data.y;
            was_touched = true;
        } else {
            // Сбрасываем предыдущую позицию когда отпускаем палец
            if (was_touched) {
                last_x = -1;
                last_y = -1;
                was_touched = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void show_wifi_networks(ili9341_t* lcd) {
    uint16_t font_scale = 2;

    ili9341_fill_screen(lcd, ILI9341_BLACK);
    ili9341_draw_string(lcd, 10, 2, "Scanning Wi-Fi...", ILI9341_YELLOW, ILI9341_BLACK, font_scale);

    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    memset(ap_info, 0, sizeof(ap_info));
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    scan_wifi(ap_info, &number);

    uint16_t y_pos = 2;
    uint16_t line_height = 7 * font_scale + 2;

    ili9341_fill_screen(lcd, ILI9341_BLACK);
    ili9341_draw_string(lcd, 10, 2, "Wi-Fi networks: ", ILI9341_YELLOW, ILI9341_BLACK, font_scale);
    y_pos += 8;

    for (size_t i = 0; i < number; i++)
    {
        ESP_LOGI(TAG, "%s", ap_info[i].ssid);
        y_pos += line_height;

        ili9341_draw_string(lcd, 10, y_pos, (const char *)ap_info[i].ssid, ILI9341_WHITE, ILI9341_BLACK, font_scale);
    }
}

// Display BLE devices with color coding
static void display_ble_devices(ili9341_t *lcd)
{
    device_list_t *device_list = get_device_list();
    if (!device_list || !device_list->mutex) {
        return;
    }

    // Clear screen
    ili9341_fill_screen(lcd, ILI9341_BLACK);

    // Title
    ili9341_draw_string(lcd, 10, 2, "BLE Devices:", ILI9341_YELLOW, ILI9341_BLACK, 2);

    uint16_t y_pos = 20;
    uint16_t line_height = 9; // 7 pixel font height + 2 pixel spacing
    time_t now = time(NULL);

    // Lock mutex to safely read device list
    xSemaphoreTake(device_list->mutex, portMAX_DELAY);

    int displayed_count = 0;
    for (int i = 0; i < MAX_TRACKED_DEVICES && displayed_count < 30; i++) {
        if (!device_list->devices[i].is_active) {
            continue;
        }

        tracked_device_t *tracked = &device_list->devices[i];
        ble_device_t *device = &tracked->device;

        time_t age = now - tracked->last_seen;
        time_t device_age = now - tracked->first_seen;

        // Determine color based on device age
        uint16_t color;
        if (device_age < 30) {
            // New device (less than 30 seconds) - GREEN
            color = ILI9341_GREEN;
        } else if (age > 300) {
            // Not seen for more than 5 minutes - RED
            color = ILI9341_RED;
        } else {
            // Normal device - WHITE
            color = ILI9341_WHITE;
        }

        // Display device name or address
        char display_text[50];
        if (device->name[0] != '\0') {
            snprintf(display_text, sizeof(display_text), "%s", device->name);
        } else if (strcmp(device->manufacturer_name, "Unknown") != 0) {
            // Show manufacturer if name not available
            snprintf(display_text, sizeof(display_text), "[%s]", device->manufacturer_name);
        } else {
            snprintf(display_text, sizeof(display_text), "%02X:%02X:%02X:%02X:%02X:%02X",
                     device->bda[0], device->bda[1], device->bda[2],
                     device->bda[3], device->bda[4], device->bda[5]);
        }

        // Draw device info - main line
        ili9341_draw_string(lcd, 10, y_pos, display_text, color, ILI9341_BLACK, 1);

        // Display RSSI on the right side
        char rssi_text[10];
        snprintf(rssi_text, sizeof(rssi_text), "%ddBm", device->rssi);
        ili9341_draw_string(lcd, 180, y_pos, rssi_text, color, ILI9341_BLACK, 1);

        y_pos += line_height;
        displayed_count++;

        // Show manufacturer and service info on second line if available
        if (y_pos <= 310) {
            char info_text[60] = "";
            bool has_info = false;

            // Add manufacturer if not already shown in name
            if (device->name[0] != '\0' && strcmp(device->manufacturer_name, "Unknown") != 0) {
                snprintf(info_text, sizeof(info_text), "%s", device->manufacturer_name);
                has_info = true;
            }

            // Add service UUIDs - show all services
            if (device->service_count > 0) {
                if (has_info && strlen(info_text) > 0) {
                    strncat(info_text, " | ", sizeof(info_text) - strlen(info_text) - 1);
                }

                for (int j = 0; j < device->service_count && j < 3; j++) {
                    char service_info[30];
                    const char* service_name = get_ble_service_name(device->service_uuids[j]);

                    if (service_name) {
                        snprintf(service_info, sizeof(service_info), "%s%s",
                                j > 0 ? "," : "", service_name);
                    } else {
                        snprintf(service_info, sizeof(service_info), "%s0x%04X",
                                j > 0 ? "," : "", device->service_uuids[j]);
                    }

                    strncat(info_text, service_info, sizeof(info_text) - strlen(info_text) - 1);
                }

                // Show count if there are more services
                if (device->service_count > 3) {
                    char more[10];
                    snprintf(more, sizeof(more), "+%d", device->service_count - 3);
                    strncat(info_text, more, sizeof(info_text) - strlen(info_text) - 1);
                }

                has_info = true;
            }

            if (has_info) {
                ili9341_draw_string(lcd, 15, y_pos, info_text, ILI9341_DARKGREY, ILI9341_BLACK, 1);
                y_pos += line_height - 1;  // Less spacing for info line
            }
        }

        // Stop if we run out of screen space
        if (y_pos > 308) {
            break;
        }
    }

    // Display device count at bottom
    char count_text[30];
    snprintf(count_text, sizeof(count_text), "Total: %d", device_list->count);
    ili9341_draw_string(lcd, 10, 305, count_text, ILI9341_CYAN, ILI9341_BLACK, 1);

    xSemaphoreGive(device_list->mutex);
}

// Task to periodically update the display
static void display_update_task(void *pvParameters)
{
    ili9341_t *lcd = (ili9341_t *)pvParameters;

    ESP_LOGI(TAG, "Display update task started");

    while (1) {
        display_ble_devices(lcd);
        vTaskDelay(pdMS_TO_TICKS(10 * 1000)); // Update every 2 seconds
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ILI9341 LCD Demo with touchscreen support");
    ESP_LOGI(TAG, "LCD Pin configuration: MOSI=%d, CLK=%d, CS=%d, DC=%d, RST=%d, BCKL=%d",
             PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_RST, PIN_NUM_BCKL);
    ESP_LOGI(TAG, "Touch Pin configuration: MISO=%d, MOSI=%d, CLK=%d, CS=%d, IRQ=%d",
             TOUCH_MISO, TOUCH_MOSI, TOUCH_CLK, TOUCH_CS, TOUCH_IRQ);

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

    // Инициализация тачскрина
    xpt2046_t touch;
    ret = xpt2046_init(&touch, TOUCH_HOST, TOUCH_MISO, TOUCH_MOSI, TOUCH_CLK,
                       TOUCH_CS, TOUCH_IRQ, 240, 320);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize XPT2046: %s", esp_err_to_name(ret));
        draw_text_demo(&lcd, NULL);
        return;
    }

    ESP_LOGI(TAG, "Touchscreen initialized successfully!");

    // calibrate_touchscreen(&lcd, &touch);

    xpt2046_set_calibration(&touch, 315, 1784, 237, 1825);
    xpt2046_set_transform(&touch, false, true, false);  // Инвертируем Y

    // draw_text_demo(&lcd, &touch);
    setup_nvs();

    // Initialize BLE
    ESP_LOGI(TAG, "Initializing BLE...");
    setup_bluetooth();

    // Start BLE scanning task
    ESP_LOGI(TAG, "Starting BLE scan task...");
    start_ble_scan_task();

    // Start display update task
    ESP_LOGI(TAG, "Starting display update task...");
    xTaskCreate(display_update_task, "display_update", 4096, &lcd, 5, NULL);

    // Main loop - can be used for other tasks or removed
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
