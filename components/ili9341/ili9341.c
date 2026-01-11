#include "ili9341.h"
#include "font5x7_cyrillic.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "ILI9341";

#define LCD_WIDTH  240
#define LCD_HEIGHT 320

// Send command
void ili9341_write_command(ili9341_t *lcd, uint8_t cmd)
{
    gpio_set_level(lcd->pin_dc, 0); // Command mode

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(lcd->spi, &t);
}

// Send data
void ili9341_write_data(ili9341_t *lcd, const uint8_t *data, int len)
{
    if (len == 0) return;

    gpio_set_level(lcd->pin_dc, 1); // Data mode

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(lcd->spi, &t);
}

// Send one byte of data
void ili9341_write_data_byte(ili9341_t *lcd, uint8_t data)
{
    ili9341_write_data(lcd, &data, 1);
}

// Set drawing window
void ili9341_set_addr_window(ili9341_t *lcd, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    ili9341_write_command(lcd, ILI9341_CASET); // Column addr set
    uint8_t data[] = {
        (x0 >> 8) & 0xFF,
        x0 & 0xFF,
        (x1 >> 8) & 0xFF,
        x1 & 0xFF
    };
    ili9341_write_data(lcd, data, 4);

    ili9341_write_command(lcd, ILI9341_PASET); // Row addr set
    uint8_t data2[] = {
        (y0 >> 8) & 0xFF,
        y0 & 0xFF,
        (y1 >> 8) & 0xFF,
        y1 & 0xFF
    };
    ili9341_write_data(lcd, data2, 4);

    ili9341_write_command(lcd, ILI9341_RAMWR); // write to RAM
}

// Draw pixel
void ili9341_draw_pixel(ili9341_t *lcd, uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= lcd->width) || (y >= lcd->height)) return;

    ili9341_set_addr_window(lcd, x, y, x, y);

    uint8_t data[] = { color >> 8, color & 0xFF };
    ili9341_write_data(lcd, data, 2);
}

// Fill rectangle
void ili9341_fill_rect(ili9341_t *lcd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((x >= lcd->width) || (y >= lcd->height)) return;
    if ((x + w - 1) >= lcd->width) w = lcd->width - x;
    if ((y + h - 1) >= lcd->height) h = lcd->height - y;

    // ESP_LOGI(TAG, "fill_rect: x=%d, y=%d, w=%d, h=%d, color=0x%04X", x, y, w, h, color);

    ili9341_set_addr_window(lcd, x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    // ESP_LOGI(TAG, "Color bytes: hi=0x%02X, lo=0x%02X", hi, lo);

    gpio_set_level(lcd->pin_dc, 1); // Data mode

    // Send data line-by-line for better performance
    uint32_t num_pixels = w * h;

    // Create buffer for one line
    uint8_t *line_buffer = malloc(w * 2);
    if (line_buffer) {
        // Fill line buffer with color
        for (int i = 0; i < w; i++) {
            line_buffer[i * 2] = hi;
            line_buffer[i * 2 + 1] = lo;
        }

        // Send line h times
        for (int i = 0; i < h; i++) {
            spi_transaction_t t;
            memset(&t, 0, sizeof(t));
            t.length = w * 16;
            t.tx_buffer = line_buffer;
            spi_device_polling_transmit(lcd->spi, &t);
        }

        free(line_buffer);
    } else {
        // If not enough memory, send pixel by pixel
        for (uint32_t i = 0; i < num_pixels; i++) {
            uint8_t data[] = { hi, lo };
            spi_transaction_t t;
            memset(&t, 0, sizeof(t));
            t.length = 16;
            t.tx_buffer = data;
            spi_device_polling_transmit(lcd->spi, &t);
        }
    }
}

// Fill entire screen
void ili9341_fill_screen(ili9341_t *lcd, uint16_t color)
{
    ili9341_fill_rect(lcd, 0, 0, lcd->width, lcd->height, color);
}

// Backlight control
void ili9341_backlight(ili9341_t *lcd, bool on)
{
    if (lcd->pin_bckl >= 0) {
        gpio_set_level(lcd->pin_bckl, on ? 1 : 0);
    }
}

// Display initialization
esp_err_t ili9341_init(ili9341_t *lcd, spi_host_device_t host, int8_t pin_mosi, int8_t pin_clk,
                       int8_t pin_cs, int8_t pin_dc, int8_t pin_rst, int8_t pin_bckl)
{
    lcd->pin_dc = pin_dc;
    lcd->pin_rst = pin_rst;
    lcd->pin_bckl = pin_bckl;
    lcd->width = LCD_WIDTH;
    lcd->height = LCD_HEIGHT;

    // GPIO configuration for DC
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin_dc),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // GPIO configuration for RST
    if (pin_rst >= 0) {
        io_conf.pin_bit_mask = (1ULL << pin_rst);
        gpio_config(&io_conf);
    }

    // GPIO configuration for backlight
    if (pin_bckl >= 0) {
        io_conf.pin_bit_mask = (1ULL << pin_bckl);
        gpio_config(&io_conf);
        gpio_set_level(pin_bckl, 0); // Turn off backlight
    }

    // SPI bus initialization
    spi_bus_config_t buscfg = {
        .mosi_io_num = pin_mosi,
        .miso_io_num = -1,
        .sclk_io_num = pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2 + 8
    };

    esp_err_t ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    // Add device to SPI bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 80 * 1000 * 1000,  // 80 MHz
        .mode = 0,
        .spics_io_num = pin_cs,
        .queue_size = 7,
        .pre_cb = NULL,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    ret = spi_bus_add_device(host, &devcfg, &lcd->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }

    // Hardware reset
    if (pin_rst >= 0) {
        gpio_set_level(pin_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(pin_rst, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Sending init commands (TFT_eSPI compatible)...");

    // Software reset
    ili9341_write_command(lcd, 0x01); // SWRESET
    vTaskDelay(pdMS_TO_TICKS(5));

    ili9341_write_command(lcd, 0x28); // Display OFF

    // Power Control A
    ili9341_write_command(lcd, 0xCB);
    uint8_t pca[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
    ili9341_write_data(lcd, pca, 5);

    // Power Control B
    ili9341_write_command(lcd, 0xCF);
    uint8_t pcb[] = {0x00, 0xC1, 0x30};
    ili9341_write_data(lcd, pcb, 3);

    // Driver timing control A
    ili9341_write_command(lcd, 0xE8);
    uint8_t dtca[] = {0x85, 0x00, 0x78};
    ili9341_write_data(lcd, dtca, 3);

    // Driver timing control B
    ili9341_write_command(lcd, 0xEA);
    uint8_t dtcb[] = {0x00, 0x00};
    ili9341_write_data(lcd, dtcb, 2);

    // Power on sequence control
    ili9341_write_command(lcd, 0xED);
    uint8_t posc[] = {0x64, 0x03, 0x12, 0x81};
    ili9341_write_data(lcd, posc, 4);

    // Pump ratio control
    ili9341_write_command(lcd, 0xF7);
    uint8_t prc[] = {0x20};
    ili9341_write_data(lcd, prc, 1);

    // Power Control 1
    ili9341_write_command(lcd, 0xC0);
    uint8_t pc1[] = {0x23};
    ili9341_write_data(lcd, pc1, 1);

    // Power Control 2
    ili9341_write_command(lcd, 0xC1);
    uint8_t pc2[] = {0x10};
    ili9341_write_data(lcd, pc2, 1);

    // VCOM Control 1
    ili9341_write_command(lcd, 0xC5);
    uint8_t vc1[] = {0x3E, 0x28};
    ili9341_write_data(lcd, vc1, 2);

    // VCOM Control 2
    ili9341_write_command(lcd, 0xC7);
    uint8_t vc2[] = {0x86};
    ili9341_write_data(lcd, vc2, 1);

    // MADCTL
    ili9341_write_command(lcd, 0x36);
    uint8_t madctl[] = {0x48}; // MX, BGR
    ili9341_write_data(lcd, madctl, 1);
    ESP_LOGI(TAG, "MADCTL: 0x48");

    // Pixel Format
    ili9341_write_command(lcd, 0x3A);
    uint8_t pixfmt[] = {0x55}; // 16 bit
    ili9341_write_data(lcd, pixfmt, 1);

    // Frame Rate Control
    ili9341_write_command(lcd, 0xB1);
    uint8_t frc[] = {0x00, 0x18};
    ili9341_write_data(lcd, frc, 2);

    // Display Function Control
    ili9341_write_command(lcd, 0xB6);
    uint8_t dfc[] = {0x08, 0x82, 0x27};
    ili9341_write_data(lcd, dfc, 3);

    // 3Gamma Function Disable
    ili9341_write_command(lcd, 0xF2);
    uint8_t gfd[] = {0x00};
    ili9341_write_data(lcd, gfd, 1);

    // Gamma curve selected
    ili9341_write_command(lcd, 0x26);
    uint8_t gcs[] = {0x01};
    ili9341_write_data(lcd, gcs, 1);

    // Positive Gamma Correction
    ili9341_write_command(lcd, 0xE0);
    uint8_t pgc[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
    ili9341_write_data(lcd, pgc, 15);

    // Negative Gamma Correction
    ili9341_write_command(lcd, 0xE1);
    uint8_t ngc[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
    ili9341_write_data(lcd, ngc, 15);

    // Sleep Out
    ili9341_write_command(lcd, 0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    // Display ON
    ili9341_write_command(lcd, 0x29);
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_LOGI(TAG, "Display ON");

    ESP_LOGI(TAG, "ILI9341 initialized successfully");

    return ESP_OK;
}

// Get glyph for character (ASCII and Cyrillic support)
static const uint8_t* get_char_glyph(uint32_t unicode_char, bool *found)
{
    *found = true;

    // ASCII characters (32-126)
    if (unicode_char >= 32 && unicode_char <= 126) {
        return font5x7_ascii[unicode_char - 32];
    }

    // Cyrillic uppercase А-Я (U+0410 - U+042F)
    if (unicode_char >= 0x0410 && unicode_char <= 0x042F) {
        return font5x7_cyrillic_upper[unicode_char - 0x0410];
    }

    // Cyrillic lowercase а-п (U+0430 - U+043F)
    if (unicode_char >= 0x0430 && unicode_char <= 0x043F) {
        return font5x7_cyrillic_lower[unicode_char - 0x0430];
    }

    // Cyrillic lowercase р-я (U+0440 - U+044F)
    if (unicode_char >= 0x0440 && unicode_char <= 0x044F) {
        return font5x7_cyrillic_lower[unicode_char - 0x0440 + 16];
    }

    // Character not found - return space
    *found = false;
    return font5x7_ascii[0];
}

// Decode UTF-8 to Unicode code point
static uint32_t utf8_decode(const char **str)
{
    const uint8_t *s = (const uint8_t *)*str;
    uint32_t unicode = 0;

    if ((s[0] & 0x80) == 0) {
        // 1-byte character (ASCII)
        unicode = s[0];
        *str += 1;
    } else if ((s[0] & 0xE0) == 0xC0) {
        // 2-byte character
        unicode = ((s[0] & 0x1F) << 6) | (s[1] & 0x3F);
        *str += 2;
    } else if ((s[0] & 0xF0) == 0xE0) {
        // 3-byte character
        unicode = ((s[0] & 0x0F) << 12) | ((s[1] & 0x3F) << 6) | (s[2] & 0x3F);
        *str += 3;
    } else if ((s[0] & 0xF8) == 0xF0) {
        // 4-byte character
        unicode = ((s[0] & 0x07) << 18) | ((s[1] & 0x3F) << 12) | ((s[2] & 0x3F) << 6) | (s[3] & 0x3F);
        *str += 4;
    } else {
        // Invalid sequence
        unicode = '?';
        *str += 1;
    }

    return unicode;
}

// Draw one Unicode character (optimized version with buffering)
static void ili9341_draw_char_unicode(ili9341_t *lcd, uint16_t x, uint16_t y, uint32_t unicode_char, uint16_t color, uint16_t bg, uint8_t size)
{
    if ((x >= lcd->width) || (y >= lcd->height)) return;

    // Get character glyph
    bool found;
    const uint8_t *glyph = get_char_glyph(unicode_char, &found);

    // Calculate character size with scaling
    uint16_t char_width = FONT_WIDTH * size;
    uint16_t char_height = FONT_HEIGHT * size;

    // Create buffer for entire character
    uint16_t buffer_size = char_width * char_height;
    uint16_t *buffer = (uint16_t *)malloc(buffer_size * 2); // 2 bytes per pixel

    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer for character");
        return;
    }

    // Fill buffer
    uint16_t buf_index = 0;
    for (uint16_t row = 0; row < char_height; row++) {
        for (uint16_t col = 0; col < char_width; col++) {
            // Determine which pixel in the original font
            uint8_t orig_col = col / size;
            uint8_t orig_row = row / size;

            uint8_t column = glyph[orig_col];
            bool pixel_on = column & (1 << orig_row);

            buffer[buf_index++] = pixel_on ? color : bg;
        }
    }

    // Set window for character
    ili9341_set_addr_window(lcd, x, y, x + char_width - 1, y + char_height - 1);

    // Send entire buffer at once
    gpio_set_level(lcd->pin_dc, 1); // Data mode

    // Convert buffer to big-endian format for sending
    uint8_t *byte_buffer = (uint8_t *)malloc(buffer_size * 2);
    if (byte_buffer) {
        for (uint16_t i = 0; i < buffer_size; i++) {
            byte_buffer[i * 2] = buffer[i] >> 8;       // high byte
            byte_buffer[i * 2 + 1] = buffer[i] & 0xFF; // low byte
        }

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = buffer_size * 16; // bits
        t.tx_buffer = byte_buffer;
        spi_device_polling_transmit(lcd->spi, &t);

        free(byte_buffer);
    }

    free(buffer);
}

// Backward compatibility: draw one ASCII character
void ili9341_draw_char(ili9341_t *lcd, uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size)
{
    ili9341_draw_char_unicode(lcd, x, y, (uint32_t)c, color, bg, size);
}

// Draw string with UTF-8 support
void ili9341_draw_string(ili9341_t *lcd, uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size)
{
    uint16_t current_x = x;

    while (*str) {
        // Check if character exceeds screen bounds
        if (current_x + FONT_WIDTH * size >= lcd->width) {
            break; // Reached right edge of screen
        }

        // Decode UTF-8 character
        const char *str_before = str;
        uint32_t unicode_char = utf8_decode(&str);

        // Draw character
        ili9341_draw_char_unicode(lcd, current_x, y, unicode_char, color, bg, size);

        // Move to next position (character width + 1 pixel spacing)
        current_x += (FONT_WIDTH + 1) * size;

        // Check for end of string (if decoder didn't advance)
        if (str == str_before) {
            break;
        }
    }
}
