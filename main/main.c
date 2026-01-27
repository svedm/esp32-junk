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
#include "esp_timer.h"
#include "driver/gpio.h"
#include "ili9341.h"
#include "xpt2046.h"
#include "wifi.h"
#include "esp_wifi.h"
#include <string.h>
#include "bluetooth.h"
#include "lvgl.h"
#include "network.h"

static const char *TAG = "ILI9341_DEMO";

// Pin configuration for ILI9341 display
#define LCD_HOST       SPI2_HOST
#define PIN_NUM_MOSI   7
#define PIN_NUM_CLK    6
#define PIN_NUM_CS     17
#define PIN_NUM_DC     15
#define PIN_NUM_RST    16
#define PIN_NUM_BCKL   5

// Pin configuration for XPT2046 touchscreen
// Using SPI3_HOST because the pins differ from the display
#define TOUCH_HOST     SPI3_HOST
#define TOUCH_MISO     2   // T_DO
#define TOUCH_MOSI     42  // T_DIN
#define TOUCH_CLK      40  // T_CLK
#define TOUCH_CS       41  // T_CS
#define TOUCH_IRQ      1   // T_IRQ

// Function to draw a demo pattern
static void draw_demo_pattern(ili9341_t *lcd)
{
    ESP_LOGI(TAG, "Drawing demo pattern...");

    // Fill background with white color
    ili9341_fill_screen(lcd, ILI9341_WHITE);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Draw colored stripes
    int stripe_height = 320 / 6;
    ili9341_fill_rect(lcd, 0, 0 * stripe_height, 240, stripe_height, ILI9341_RED);
    ili9341_fill_rect(lcd, 0, 1 * stripe_height, 240, stripe_height, ILI9341_GREEN);
    ili9341_fill_rect(lcd, 0, 2 * stripe_height, 240, stripe_height, ILI9341_BLUE);
    ili9341_fill_rect(lcd, 0, 3 * stripe_height, 240, stripe_height, ILI9341_YELLOW);
    ili9341_fill_rect(lcd, 0, 4 * stripe_height, 240, stripe_height, ILI9341_CYAN);
    ili9341_fill_rect(lcd, 0, 5 * stripe_height, 240, stripe_height, ILI9341_MAGENTA);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Draw squares in corners
    ili9341_fill_screen(lcd, ILI9341_BLACK);
    ili9341_fill_rect(lcd, 10, 10, 60, 60, ILI9341_RED);
    ili9341_fill_rect(lcd, 240 - 70, 10, 60, 60, ILI9341_GREEN);
    ili9341_fill_rect(lcd, 10, 320 - 70, 60, 60, ILI9341_BLUE);
    ili9341_fill_rect(lcd, 240 - 70, 320 - 70, 60, 60, ILI9341_YELLOW);

    // Draw square in center
    ili9341_fill_rect(lcd, 240/2 - 40, 320/2 - 40, 80, 80, ILI9341_WHITE);
}

// Touchscreen calibration function
static void calibrate_touchscreen(ili9341_t *lcd, xpt2046_t *touch)
{
    ESP_LOGI(TAG, "Starting touchscreen calibration");

    // Calibration points: top-left, top-right, bottom-right, bottom-left, center
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
        // Clear screen
        ili9341_fill_screen(lcd, ILI9341_WHITE);

        // Draw instruction
        char instruction[50];
        sprintf(instruction, "Touch %s point", calibration_points[i].name);
        ili9341_draw_string(lcd, 10, 10, instruction, ILI9341_BLACK, ILI9341_WHITE, 1);

        // Draw target (crosshair)
        uint16_t cx = calibration_points[i].x;
        uint16_t cy = calibration_points[i].y;

        // Draw crosshair
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

        // Draw center dot
        for (int dy = -3; dy <= 3; dy++) {
            for (int dx = -3; dx <= 3; dx++) {
                if (cx + dx >= 0 && cx + dx < 240 && cy + dy >= 0 && cy + dy < 320) {
                    ili9341_draw_pixel(lcd, cx + dx, cy + dy, ILI9341_RED);
                }
            }
        }

        ESP_LOGI(TAG, "Waiting for touch on %s (%d, %d)...",
                 calibration_points[i].name, cx, cy);

        // Wait for touch
        bool touched = false;
        while (!touched) {
            // Read raw values directly
            uint16_t raw_x = xpt2046_read_raw(touch, 0xD0);  // X position
            uint16_t raw_y = xpt2046_read_raw(touch, 0x90);  // Y position
            uint16_t raw_z = xpt2046_read_raw(touch, 0xB0);  // Pressure

            if (raw_z > 100) {
                raw_x_values[i] = raw_x;
                raw_y_values[i] = raw_y;

                ESP_LOGI(TAG, "Point %d (%s): raw_x=%d, raw_y=%d, raw_z=%d",
                         i, calibration_points[i].name, raw_x, raw_y, raw_z);

                // Show confirmation
                ili9341_fill_rect(lcd, cx - 5, cy - 5, 10, 10, ILI9341_GREEN);
                vTaskDelay(pdMS_TO_TICKS(500));

                // Wait for release
                while (raw_z > 100) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    raw_z = xpt2046_read_raw(touch, 0xB0);
                }

                touched = true;
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    // Calculate calibration values
    ESP_LOGI(TAG, "\n========== CALIBRATION RESULTS ==========");
    ESP_LOGI(TAG, "Point 0 (Top-Left):     raw_x=%d, raw_y=%d", raw_x_values[0], raw_y_values[0]);
    ESP_LOGI(TAG, "Point 1 (Top-Right):    raw_x=%d, raw_y=%d", raw_x_values[1], raw_y_values[1]);
    ESP_LOGI(TAG, "Point 2 (Bottom-Right): raw_x=%d, raw_y=%d", raw_x_values[2], raw_y_values[2]);
    ESP_LOGI(TAG, "Point 3 (Bottom-Left):  raw_x=%d, raw_y=%d", raw_x_values[3], raw_y_values[3]);
    ESP_LOGI(TAG, "Point 4 (Center):       raw_x=%d, raw_y=%d", raw_x_values[4], raw_y_values[4]);

    // Find min and max values
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

    // Show results on screen
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

    // Fill screen with black color
    ili9341_fill_screen(lcd, ILI9341_BLACK);

    // Draw English text in different sizes
    ili9341_draw_string(lcd, 10, 10, "Hello World!", ILI9341_WHITE, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 25, "ESP32-S3", ILI9341_CYAN, ILI9341_BLACK, 2);

    // Draw Russian text
    ili9341_draw_string(lcd, 10, 55, "Hello, Владислав!", ILI9341_GREEN, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 80, "Тест тестович", ILI9341_YELLOW, ILI9341_BLACK, 1);

    // Different colors in Russian
    ili9341_draw_string(lcd, 10, 100, "КРАСНЫЙ", ILI9341_RED, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 120, "ЗЕЛЕНЫЙ", ILI9341_GREEN, ILI9341_BLACK, 2);
    ili9341_draw_string(lcd, 10, 140, "СИНИЙ", ILI9341_BLUE, ILI9341_BLACK, 2);

    // Russian alphabet (selected characters)
    ili9341_draw_string(lcd, 10, 170, "АБВГДЕЖЗ", ILI9341_MAGENTA, ILI9341_BLACK, 1);
    ili9341_draw_string(lcd, 10, 185, "абвгдежз", ILI9341_MAGENTA, ILI9341_BLACK, 1);

    // Numbers
    ili9341_draw_string(lcd, 10, 210, "0123456789", ILI9341_CYAN, ILI9341_BLACK, 2);

    // Info at the bottom
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

// Function for drawing on screen using touchscreen
static void touch_draw_mode(ili9341_t *lcd, xpt2046_t *touch)
{
    ESP_LOGI(TAG, "Starting touch draw mode");

    // Clear screen with white color
    ili9341_fill_screen(lcd, ILI9341_WHITE);

    // Draw instruction
    ili9341_draw_string(lcd, 10, 10, "Touch to draw!", ILI9341_BLACK, ILI9341_WHITE, 2);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Clear screen for drawing
    ili9341_fill_screen(lcd, ILI9341_WHITE);

    // Variables to track previous position (for drawing lines)
    int16_t last_x = -1;
    int16_t last_y = -1;
    bool was_touched = false;

    // Brush size
    const int brush_size = 3;

    ESP_LOGI(TAG, "Ready to draw! Touch the screen...");

    while (1) {
        xpt2046_touch_t touch_data = xpt2046_get_touch(touch);

        if (touch_data.touched) {

            // Draw point or line
            if (was_touched && last_x >= 0 && last_y >= 0) {
                // Draw line from previous point to current (smooth drawing)
                int dx = touch_data.x - last_x;
                int dy = touch_data.y - last_y;
                int steps = (abs(dx) > abs(dy)) ? abs(dx) : abs(dy);

                if (steps > 0) {
                    for (int i = 0; i <= steps; i++) {
                        int x = last_x + (dx * i) / steps;
                        int y = last_y + (dy * i) / steps;

                        // Draw circular brush
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
                // First point - just draw a circle
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
            // Reset previous position when finger is released
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

// Global LCD and touch objects for LVGL
static ili9341_t lcd;
static xpt2046_t touch;

// Forward declarations
static uint32_t my_get_millis(void);

// LVGL flush callback - called when LVGL needs to update the display
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    // Set the drawing window to the area that needs to be updated
    ili9341_set_addr_window(&lcd, area->x1, area->y1, area->x2, area->y2);

    // Calculate the size of the area
    uint32_t width = area->x2 - area->x1 + 1;
    uint32_t height = area->y2 - area->y1 + 1;
    uint32_t pixel_count = width * height;

    // Swap bytes: LVGL outputs little-endian RGB565, but ILI9341 expects big-endian
    uint16_t *pixels = (uint16_t *)color_p;
    for (uint32_t i = 0; i < pixel_count; i++) {
        pixels[i] = (pixels[i] >> 8) | (pixels[i] << 8);
    }

    // Send all pixel data
    ili9341_write_data(&lcd, color_p, pixel_count * 2);

    // Tell LVGL we're done flushing
    lv_display_flush_ready(disp);
}

// LVGL touchscreen read callback
static void lvgl_touchpad_read(lv_indev_t *indev_drv, lv_indev_data_t *data)
{
    xpt2046_touch_t touch_data = xpt2046_get_touch(&touch);

    if (touch_data.touched) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = touch_data.x;
        data->point.y = touch_data.y;

        // Debug: print touch coordinates
        static uint32_t last_log = 0;
        uint32_t now = my_get_millis();
        if (now - last_log > 500) {  // Log every 500ms
            ESP_LOGI(TAG, "Touch detected: x=%d, y=%d, pressure=%d",
                     touch_data.x, touch_data.y, touch_data.z);
            last_log = now;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// Button event handler
static void button_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Button clicked!");
    } else if (code == LV_EVENT_PRESSED) {
        ESP_LOGI(TAG, "Button pressed");
    } else if (code == LV_EVENT_RELEASED) {
        ESP_LOGI(TAG, "Button released");
    }
}

// Create a simple UI with a button
static void create_demo_ui(void)
{
    // Create a button in the center of the screen
    lv_obj_t *btn = lv_button_create(lv_screen_active());
    static lv_style_t btn_style;
    lv_style_init(&btn_style);
    lv_style_set_bg_color(&btn_style, lv_color_hex(0xFF0000));


    lv_obj_set_size(btn, 120, 50);
    lv_obj_add_style(btn, &btn_style, 0);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, -40);
    lv_obj_add_event_cb(btn, button_event_handler, LV_EVENT_ALL, NULL);

    // Create a label on the button
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "Click Me!");
    lv_obj_center(label);

    // Create a title label
    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "LVGL + ILI9341 Demo");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Create an info label
    lv_obj_t *info = lv_label_create(lv_screen_active());
    lv_label_set_text(info, "Touch the button!");
    lv_obj_align(info, LV_ALIGN_CENTER, 0, 40);
}

// Custom tick callback for LVGL - returns milliseconds since boot
static uint32_t my_get_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// LVGL task that handles the timer and events
static void lvgl_task(void *pvParameter)
{
    ESP_LOGI(TAG, "LVGL task started");

    while (1) {
        // Handle LVGL tasks - this processes all widgets, animations, input devices, etc.
        uint32_t time_till_next = lv_timer_handler();

        // Sleep for the time until next timer needs to be run
        // Minimum 1ms delay to allow IDLE task to run and reset watchdog
        uint32_t sleep_ms = (time_till_next < 1) ? 1 : (time_till_next > 10 ? 10 : time_till_next);
        vTaskDelay(pdMS_TO_TICKS(sleep_ms));
    }
}

// WiFi task - runs WiFi connection in background
static void wifi_task(void *pvParameter)
{
    ESP_LOGI(TAG, "WiFi task started");
    vTaskDelay(pdMS_TO_TICKS(500)); // Give LVGL time to render first frame
    connect_to_wifi();
    ESP_LOGI(TAG, "WiFi connection initiated");
    vTaskDelete(NULL);
}

void http_response_callback(int status_code, const char *response) {
    ESP_LOGI(TAG, "HTTP Response: %d, Body: %s\n", status_code, response);
}

void print_time(void) {
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;

    time(&now);
    // Set timezone to Moscow Standard Time (MSK = UTC+3)
    // Note: POSIX format uses opposite signs, so UTC+3 is "MSK-3"
    setenv("TZ", "MSK-3", 1);
    tzset();

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Moscow is: %s", strftime_buf);
}

// Task that performs network requests after Wi-Fi is ready
static void network_request_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Wi-Fi is ready, making network requests...");
    http_get("https://www.google.com", http_response_callback);
    print_time();
    vTaskDelete(NULL);
}

// Callback that runs when Wi-Fi gets IP address
void on_wifi_ready(void) {
    // Create a separate task for network requests to avoid blocking the event handler
    xTaskCreate(network_request_task, "network_req", 4096, NULL, 3, NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LVGL Demo with ILI9341 and XPT2046");
    ESP_LOGI(TAG, "LCD Pin configuration: MOSI=%d, CLK=%d, CS=%d, DC=%d, RST=%d, BCKL=%d",
             PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_RST, PIN_NUM_BCKL);
    ESP_LOGI(TAG, "Touch Pin configuration: MISO=%d, MOSI=%d, CLK=%d, CS=%d, IRQ=%d",
             TOUCH_MISO, TOUCH_MOSI, TOUCH_CLK, TOUCH_CS, TOUCH_IRQ);

    // Initialize NVS first
    setup_nvs();

    // Initialize ILI9341 LCD
    ESP_LOGI(TAG, "Initializing ILI9341 LCD...");
    esp_err_t ret = ili9341_init(&lcd, LCD_HOST, PIN_NUM_MOSI, PIN_NUM_CLK,
                                 PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_RST, PIN_NUM_BCKL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LCD");
        return;
    }
    ESP_LOGI(TAG, "LCD initialized successfully");

     // Turn on backlight
    ili9341_backlight(&lcd, true);
    ESP_LOGI(TAG, "Backlight ON");

    // Initialize XPT2046 touchscreen
    ESP_LOGI(TAG, "Initializing XPT2046 touchscreen...");
    ret = xpt2046_init(&touch, TOUCH_HOST, TOUCH_MISO, TOUCH_MOSI, TOUCH_CLK,
                       TOUCH_CS, TOUCH_IRQ, 240, 320);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize touchscreen");
        return;
    }

    xpt2046_set_calibration(&touch, 315, 1784, 237, 1825);
    xpt2046_set_transform(&touch, false, true, false);  // Invert Y
    ESP_LOGI(TAG, "Touchscreen initialized successfully");

    // Initialize LVGL
    ESP_LOGI(TAG, "Initializing LVGL...");
    lv_init();

    // Set custom tick callback to get system time in milliseconds
    lv_tick_set_cb(my_get_millis);

    // Create LVGL display object
    lv_display_t *display = lv_display_create(240, 320);

    // Set up the display buffer (1/6 of screen size for faster rendering)
    static uint8_t buf[240 * 320 / 6 * 2];
    lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Set the flush callback
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    // Set color format to RGB565
    // Note: Our ILI9341 driver is configured for RGB order (MADCTL=0x40)
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);

    ESP_LOGI(TAG, "LVGL display initialized");

    // Create an input device (touchscreen)
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touchpad_read);

    ESP_LOGI(TAG, "LVGL input device initialized");

    // Create the demo UI
    create_demo_ui();

    ESP_LOGI(TAG, "Demo UI created");

    // Create LVGL task with larger stack (8KB instead of 4KB)
    xTaskCreate(lvgl_task, "lvgl_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "LVGL initialization complete!");

    // Wi-Fi
    wifi_set_ready_callback(on_wifi_ready);
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 3, NULL);
}
