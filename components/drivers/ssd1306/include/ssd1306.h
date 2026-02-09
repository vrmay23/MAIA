/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SSD1306 OLED Display Driver for MAIA Project
 *
 * Copyright (c) 2026 Vinicius May
 *
 * Driver for Solomon Systech SSD1306 128x32 OLED display controller.
 * Supports I2C communication, framebuffer rendering, text output with
 * dual font sizes (5x8 and 8x16), and power management.
 *
 * Hardware:
 * - Display: 128x32 monochrome OLED
 * - Controller: SSD1306
 * - Interface: I2C (default address 0x3C)
 * - Framebuffer: 512 bytes (128 * 32 / 8)
 *
 * Features:
 * - Dual fonts: 5x8 (small) and 8x16 (large)
 * - Full ASCII support (0x20-0x7E) for both fonts
 * - Runtime contrast adjustment (0-255)
 * - Display rotation (0 and 180 degrees)
 * - Power management (screen on/off)
 * - Pixel-level framebuffer control
 *
 * References:
 * - SSD1306 Datasheet: Solomon Systech SSD1306 Rev 1.1
 * - I2C Protocol: ESP-IDF I2C Master Driver API (v6.0+)
 * - 5x7 bitmap font - Standard hardware ROM pattern
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Font size selection for text rendering
 *
 * Display 128x32 layout:
 * - FONT_SMALL (5x8): ~25 characters per line, 4 lines total
 * - FONT_LARGE (8x16): ~16 characters per line, 2 lines total
 *
 * Both fonts support full ASCII printable range (0x20-0x7E):
 * - Space, punctuation, digits, uppercase, lowercase, symbols
 */
typedef enum
{
    SSD1306_FONT_SMALL = 0, /**< 5x8 font for normal text */
    SSD1306_FONT_LARGE = 1  /**< 8x16 font for titles/emphasis */
} ssd1306_font_t;

/**
 * @brief Initialize SSD1306 display driver
 *
 * Performs full initialization sequence:
 * 1. Verify I2C bus is initialized (via maia_board)
 * 2. Send hardware initialization commands:
 *    - Display OFF
 *    - Set multiplex ratio (32 lines for 128x32 display)
 *    - Set display offset (0)
 *    - Set display start line (0)
 *    - Configure charge pump (enable for OLED power)
 *    - Set memory addressing mode (horizontal)
 *    - Configure segment/COM mapping per rotation config
 *    - Set contrast per Kconfig setting
 *    - Disable entire display ON (use RAM content)
 *    - Set normal display mode (non-inverted)
 *    - Set display clock divider and oscillator frequency
 *    - Set precharge period
 *    - Set VCOMH deselect level
 *    - Display ON
 * 3. Clear framebuffer (all pixels OFF)
 * 4. Flush framebuffer to display
 *
 * Must be called before any other ssd1306_* functions.
 *
 * @return
 *     - ESP_OK: Initialization successful
 *     - ESP_ERR_INVALID_STATE: I2C bus not initialized
 *     - ESP_FAIL: Communication error with display
 */
esp_err_t ssd1306_init(void);

/**
 * @brief Clear framebuffer (set all pixels OFF)
 *
 * Zeros internal 512-byte framebuffer. Call ssd1306_display() to
 * update screen with cleared buffer.
 *
 * @return
 *     - ESP_OK: Framebuffer cleared successfully
 */
esp_err_t ssd1306_clear(void);

/**
 * @brief Flush framebuffer to display GDDRAM
 *
 * Transmits entire 512-byte framebuffer (128x32 bits) to SSD1306
 * GDDRAM via I2C. Updates visible screen content.
 *
 * Uses horizontal addressing mode: data written sequentially from
 * column 0-127, then advances to next page (row group).
 *
 * @return
 *     - ESP_OK: Display updated successfully
 *     - ESP_FAIL: I2C transmission error
 */
esp_err_t ssd1306_display(void);

/**
 * @brief Set individual pixel state in framebuffer
 *
 * Coordinates are in display orientation (after rotation applied).
 * Does not automatically update screen - call ssd1306_display().
 *
 * @param[in] x X coordinate (0-127, left to right)
 * @param[in] y Y coordinate (0-31, top to bottom)
 * @param[in] on true = pixel ON (white), false = pixel OFF (black)
 *
 * @return
 *     - ESP_OK: Pixel set successfully
 *     - ESP_ERR_INVALID_ARG: Coordinates out of bounds
 */
esp_err_t ssd1306_set_pixel(uint8_t x, uint8_t y, bool on);

/**
 * @brief Draw single ASCII character at specified position
 *
 * Character is drawn using selected font. Non-printable characters
 * (outside 0x20-0x7E range) are silently ignored.
 *
 * Does not automatically update screen - call ssd1306_display().
 *
 * @param[in] x X coordinate in pixels (left edge of character)
 * @param[in] y Y coordinate in pixels (top edge of character)
 * @param[in] ch ASCII character to draw (0x20-0x7E supported)
 * @param[in] font Font size (SMALL = 5x8, LARGE = 8x16)
 *
 * @return
 *     - ESP_OK: Character drawn to framebuffer
 *     - ESP_ERR_INVALID_ARG: Coordinates out of bounds or
 *                            unsupported character
 */
esp_err_t ssd1306_draw_char(uint8_t x, uint8_t y, char ch,
                            ssd1306_font_t font);

/**
 * @brief Draw null-terminated string at specified position
 *
 * Renders string character-by-character using specified font.
 * Characters are spaced appropriately (6 pixels for 5x8, 9 pixels
 * for 8x16). Text wrapping not supported - characters beyond screen
 * edge are clipped.
 *
 * Does not automatically update screen - call ssd1306_display().
 *
 * @param[in] x X coordinate in pixels (left edge of first char)
 * @param[in] y Y coordinate in pixels (top edge of text)
 * @param[in] str Null-terminated ASCII string
 * @param[in] font Font size (SMALL = 5x8, LARGE = 8x16)
 *
 * @return
 *     - ESP_OK: String rendered to framebuffer
 *     - ESP_ERR_INVALID_ARG: NULL pointer or invalid coordinates
 */
esp_err_t ssd1306_draw_string(uint8_t x, uint8_t y, const char *str,
                              ssd1306_font_t font);

/**
 * @brief Turn display ON (wake from sleep)
 *
 * Sends hardware command 0xAF to enable display panel. Framebuffer
 * contents preserved. Fast wake time (~1ms).
 *
 * Display consumes normal power when ON.
 *
 * @return
 *     - ESP_OK: Display turned ON successfully
 *     - ESP_FAIL: I2C communication error
 */
esp_err_t ssd1306_screen_on(void);

/**
 * @brief Turn display OFF (enter sleep mode)
 *
 * Sends hardware command 0xAE to disable display panel. Framebuffer
 * contents preserved in RAM. Reduces power consumption significantly.
 *
 * Screen appears blank but controller remains responsive to I2C.
 *
 * @return
 *     - ESP_OK: Display turned OFF successfully
 *     - ESP_FAIL: I2C communication error
 */
esp_err_t ssd1306_screen_off(void);

/**
 * @brief Set display contrast/brightness at runtime
 *
 * Adjusts OLED pixel brightness via contrast register.
 * Does not affect framebuffer contents.
 *
 * @param[in] contrast Contrast level (0-255)
 *                     0 = minimum brightness (dimmest)
 *                     127 = medium brightness (50%)
 *                     255 = maximum brightness (brightest)
 *
 * @return
 *     - ESP_OK: Contrast updated successfully
 *     - ESP_FAIL: I2C communication error
 */
esp_err_t ssd1306_set_contrast(uint8_t contrast);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
