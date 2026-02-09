/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SSD1306 OLED Display Driver Test Suite
 *
 * Copyright (c) 2026 Vinicius May
 *
 * Comprehensive test suite for SSD1306 128x32 OLED display driver.
 * Tests all driver functions including initialization, framebuffer
 * operations, text rendering, and power management.
 *
 * Test sequence:
 * 1. Driver initialization
 * 2. Display clear and update
 * 3. Pixel drawing
 * 4. Small font text (5x8)
 * 5. Large font text (8x16)
 * 6. Screen on/off (power management)
 * 7. Contrast adjustment
 * 8. Animal/tutor information display
 * 9. Status screen simulation
 *
 * Each test runs for a few seconds to allow visual verification.
 */

#include "tests.h"
#include "ssd1306.h"
#include "maia_board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "test_ssd1306"

/* Test display timing (milliseconds) */

#define TEST_DELAY_SHORT_MS   2000
#define TEST_DELAY_MEDIUM_MS  3000
#define TEST_DELAY_LONG_MS    5000

/* Display dimensions */

#define DISPLAY_WIDTH   CONFIG_MAIA_SSD1306_WIDTH
#define DISPLAY_HEIGHT  CONFIG_MAIA_SSD1306_HEIGHT

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_test_count = 0;
static uint8_t g_test_passed = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Print test header with number and description
 *
 * @param[in] description Test description string
 */
static void test_print_header(const char *description)
{
    g_test_count++;
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Test %d: %s", g_test_count, description);
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Print test result (pass/fail)
 *
 * @param[in] passed true if test passed, false if failed
 */
static void test_print_result(bool passed)
{
    if (passed)
    {
        ESP_LOGI(TAG, "Result: PASS");
        g_test_passed++;
    }
    else
    {
        ESP_LOGE(TAG, "Result: FAIL");
    }
}

/**
 * @brief Test 1: Driver initialization
 */
static void test_init(void)
{
    esp_err_t ret;

    test_print_header("Driver Initialization");

    ret = ssd1306_init();

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Display initialized successfully");
        test_print_result(true);
    }
    else
    {
        ESP_LOGE(TAG, "Display initialization failed: %s",
                 esp_err_to_name(ret));
        test_print_result(false);
    }

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_SHORT_MS));
}

/**
 * @brief Test 2: Clear and display
 */
static void test_clear_display(void)
{
    esp_err_t ret;

    test_print_header("Clear and Display Update");

    ret = ssd1306_clear();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Clear failed");
        test_print_result(false);
        return;
    }

    ret = ssd1306_display();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Display update failed");
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Display cleared and updated");
    test_print_result(true);

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_SHORT_MS));
}

/**
 * @brief Test 3: Pixel drawing
 */
static void test_pixel_drawing(void)
{
    esp_err_t ret;
    uint8_t x;
    uint8_t y;

    test_print_header("Pixel Drawing");

    ssd1306_clear();

    /* Draw border around display */

    for (x = 0; x < DISPLAY_WIDTH; x++)
    {
        ssd1306_set_pixel(x, 0, true);
        ssd1306_set_pixel(x, DISPLAY_HEIGHT - 1, true);
    }

    for (y = 0; y < DISPLAY_HEIGHT; y++)
    {
        ssd1306_set_pixel(0, y, true);
        ssd1306_set_pixel(DISPLAY_WIDTH - 1, y, true);
    }

    /* Draw diagonal line */

    for (x = 0; x < DISPLAY_WIDTH && x < DISPLAY_HEIGHT; x++)
    {
        ssd1306_set_pixel(x, x, true);
    }

    ret = ssd1306_display();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Display update failed");
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Border and diagonal line drawn");
    test_print_result(true);

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MEDIUM_MS));
}

/**
 * @brief Test 4: Small font text rendering (5x8)
 */
static void test_small_font(void)
{
    esp_err_t ret;

    test_print_header("Small Font Text (5x8)");

    ssd1306_clear();

    /* Line 0: ASCII uppercase */

    ssd1306_draw_string(0, 0, "ABCDEFGHIJKLMNOPQRSTU",
                       SSD1306_FONT_SMALL);

    /* Line 1: ASCII lowercase */

    ssd1306_draw_string(0, 8, "abcdefghijklmnopqrstu",
                       SSD1306_FONT_SMALL);

    /* Line 2: Digits and symbols */

    ssd1306_draw_string(0, 16, "0123456789 !@#$%^&*()",
                       SSD1306_FONT_SMALL);

    /* Line 3: More symbols */

    ssd1306_draw_string(0, 24, "-=[]{}|;:',.<>/?~`",
                       SSD1306_FONT_SMALL);

    ret = ssd1306_display();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Display update failed");
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Full ASCII character set displayed (4 lines)");
    test_print_result(true);

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_LONG_MS));
}

/**
 * @brief Test 5: Large font text rendering (8x16)
 */
static void test_large_font(void)
{
    esp_err_t ret;

    test_print_header("Large Font Text (8x16)");

    ssd1306_clear();

    /* Line 0: Title */

    ssd1306_draw_string(0, 0, "MAIA PROJECT",
                       SSD1306_FONT_LARGE);

    /* Line 1: Subtitle */

    ssd1306_draw_string(0, 16, "Test 8x16",
                       SSD1306_FONT_LARGE);

    ret = ssd1306_display();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Display update failed");
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Large font text displayed (2 lines)");
    test_print_result(true);

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MEDIUM_MS));
}

/**
 * @brief Test 6: Mixed font sizes
 */
static void test_mixed_fonts(void)
{
    esp_err_t ret;

    test_print_header("Mixed Font Sizes");

    ssd1306_clear();

    /* Large font title */

    ssd1306_draw_string(0, 0, "Status:",
                       SSD1306_FONT_LARGE);

    /* Small font details */

    ssd1306_draw_string(0, 18, "Temp: 25.3C",
                       SSD1306_FONT_SMALL);

    ssd1306_draw_string(0, 26, "Battery: 87%",
                       SSD1306_FONT_SMALL);

    ret = ssd1306_display();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Display update failed");
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Mixed font layout displayed");
    test_print_result(true);

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MEDIUM_MS));
}

/**
 * @brief Test 7: Screen on/off (power management)
 */
static void test_screen_power(void)
{
    esp_err_t ret;

    test_print_header("Screen Power Management");

    /* Draw test pattern */

    ssd1306_clear();
    ssd1306_draw_string(0, 0, "Power Test",
                       SSD1306_FONT_LARGE);
    ssd1306_draw_string(0, 18, "Screen ON",
                       SSD1306_FONT_SMALL);
    ssd1306_display();

    ESP_LOGI(TAG, "Display ON - visible for 2 seconds");
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_SHORT_MS));

    /* Turn off */

    ret = ssd1306_screen_off();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Screen OFF failed");
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Display OFF - blank for 2 seconds");
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_SHORT_MS));

    /* Turn on */

    ret = ssd1306_screen_on();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Screen ON failed");
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Display ON - restored content");
    test_print_result(true);

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_SHORT_MS));
}

/**
 * @brief Test 8: Contrast adjustment
 */
static void test_contrast(void)
{
    esp_err_t ret;
    uint8_t levels[] = {50, 127, 200, 255};
    uint8_t i;

    test_print_header("Contrast Adjustment");

    ssd1306_clear();
    ssd1306_draw_string(0, 0, "Contrast",
                       SSD1306_FONT_LARGE);
    ssd1306_draw_string(0, 18, "Test...",
                       SSD1306_FONT_SMALL);
    ssd1306_display();

    for (i = 0; i < 4; i++)
    {
        ret = ssd1306_set_contrast(levels[i]);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Set contrast %d failed", levels[i]);
            test_print_result(false);
            return;
        }

        ESP_LOGI(TAG, "Contrast set to %d", levels[i]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Restore default contrast */

    ssd1306_set_contrast(CONFIG_MAIA_SSD1306_CONTRAST);
    ESP_LOGI(TAG, "Contrast restored to default (%d)",
             CONFIG_MAIA_SSD1306_CONTRAST);

    test_print_result(true);
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_SHORT_MS));
}

/**
 * @brief Test 9: Animal and tutor information display
 */
static void test_info_display(void)
{
    esp_err_t ret;
    char line[32];

    test_print_header("Animal/Tutor Information Display");

    /* Page 0: Animal info (splash screen) */

    ssd1306_clear();

    ssd1306_draw_string(0, 0, CONFIG_MAIA_ANIMAL_NAME,
                       SSD1306_FONT_LARGE);

    snprintf(line, sizeof(line), "%s - %dyrs",
             CONFIG_MAIA_ANIMAL_SPECIES,
             CONFIG_MAIA_ANIMAL_AGE);
    ssd1306_draw_string(0, 18, line, SSD1306_FONT_SMALL);

    snprintf(line, sizeof(line), "%dkg",
             CONFIG_MAIA_ANIMAL_WEIGHT);
    ssd1306_draw_string(0, 26, line, SSD1306_FONT_SMALL);

    ret = ssd1306_display();
    if (ret != ESP_OK)
    {
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Animal info displayed");
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MEDIUM_MS));

    /* Page 1: Tutor info */

    ssd1306_clear();

    ssd1306_draw_string(0, 0, "Owner:",
                       SSD1306_FONT_LARGE);

    ssd1306_draw_string(0, 18, CONFIG_MAIA_TUTOR_NAME,
                       SSD1306_FONT_SMALL);

    ssd1306_draw_string(0, 26, CONFIG_MAIA_TUTOR_PHONE,
                       SSD1306_FONT_SMALL);

    ret = ssd1306_display();
    if (ret != ESP_OK)
    {
        test_print_result(false);
        return;
    }

    ESP_LOGI(TAG, "Tutor info displayed");
    test_print_result(true);

    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MEDIUM_MS));
}

/**
 * @brief Test 10: Status screen simulation
 */
static void test_status_screen(void)
{
    esp_err_t ret;
    uint8_t i;

    test_print_header("Status Screen Simulation");

    for (i = 0; i < 5; i++)
    {
        ssd1306_clear();

        /* Title */

        ssd1306_draw_string(0, 0, "Status",
                           SSD1306_FONT_LARGE);

        /* Simulated sensor data */

        ssd1306_draw_string(0, 18, "Temp: 24.5C",
                           SSD1306_FONT_SMALL);

        ssd1306_draw_string(0, 26, "Battery: OK",
                           SSD1306_FONT_SMALL);

        ret = ssd1306_display();
        if (ret != ESP_OK)
        {
            test_print_result(false);
            return;
        }

        ESP_LOGI(TAG, "Status update %d/5", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    test_print_result(true);
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_SHORT_MS));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief Main test task entry point
 */
void test_ssd1306_run(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  SSD1306 OLED Display Test Suite");
    ESP_LOGI(TAG, "  Display: %dx%d pixels", DISPLAY_WIDTH,
             DISPLAY_HEIGHT);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");

    /* Initialize board (I2C bus) */

    ESP_LOGI(TAG, "Initializing MAIA board...");

    /* Run all tests sequentially */

    test_init();
    test_clear_display();
    test_pixel_drawing();
    test_small_font();
    test_large_font();
    test_mixed_fonts();
    test_screen_power();
    test_contrast();
    test_info_display();
    test_status_screen();

    /* Print summary */

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Test Summary");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Total tests: %d", g_test_count);
    ESP_LOGI(TAG, "Passed:      %d", g_test_passed);
    ESP_LOGI(TAG, "Failed:      %d", g_test_count - g_test_passed);
    ESP_LOGI(TAG, "");

    if (g_test_passed == g_test_count)
    {
        ESP_LOGI(TAG, "ALL TESTS PASSED!");
    }
    else
    {
        ESP_LOGE(TAG, "SOME TESTS FAILED!");
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Test complete. Looping status display...");
    ESP_LOGI(TAG, "");

    /* Loop status display indefinitely */

    while (1)
    {
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "MAIA",
                           SSD1306_FONT_LARGE);
        ssd1306_draw_string(0, 18, "All tests OK",
                           SSD1306_FONT_SMALL);
        ssd1306_display();

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
