/*
 * Copyright 2026 Vinicius May
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************
 * main/tests/test_ds18b20.c
 *
 * DS18B20 temperature sensor test
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "tests.h"
#include "ds18b20.h"
#include "maia_board.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[TEST_DS18B20]"

#define TEST_READ_INTERVAL_MS   2000   /* Read every 2 seconds */
#define TEST_DURATION_MS        30000  /* Run for 30 seconds */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_ds18b20_blocking
 *
 * Description:
 *   Test blocking temperature read (ds18b20_read_temperature).
 *
 ****************************************************************************/

static void test_ds18b20_blocking(void)
{
  esp_err_t ret;
  float temp;
  uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

  ESP_LOGI(TAG, "=== TEST: Blocking Read ===");

  while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) <
         TEST_DURATION_MS)
    {
      ret = ds18b20_read_temperature(&temp, NULL);
      if (ret == ESP_OK)
        {
          ESP_LOGI(TAG, "Temperature: %.2f°", temp);
        }
      else
        {
          ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
        }

      vTaskDelay(pdMS_TO_TICKS(TEST_READ_INTERVAL_MS));
    }
}

/****************************************************************************
 * Name: test_ds18b20_async
 *
 * Description:
 *   Test async temperature read (trigger + wait + read scratchpad).
 *
 ****************************************************************************/

static void test_ds18b20_async(void)
{
  esp_err_t ret;
  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  int16_t raw;
  float temp;
  uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

  ESP_LOGI(TAG, "=== TEST: Async Read ===");

  while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < 
         TEST_DURATION_MS)
    {
      /* Trigger conversion */

      ret = ds18b20_trigger_conversion(NULL);
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "Trigger failed: %s", esp_err_to_name(ret));
          vTaskDelay(pdMS_TO_TICKS(TEST_READ_INTERVAL_MS));
          continue;
        }

      ESP_LOGI(TAG, "Conversion triggered, doing other tasks...");

      /* Simulate other work during conversion */

      maia_led_toggle();
      vTaskDelay(pdMS_TO_TICKS(200));
      maia_led_toggle();
      vTaskDelay(pdMS_TO_TICKS(200));
      maia_led_toggle();

      /* Wait remaining conversion time */

#if defined(CONFIG_MAIA_DS18B20_RESOLUTION_9BIT)
      vTaskDelay(pdMS_TO_TICKS(94 - 400));
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_10BIT)
      vTaskDelay(pdMS_TO_TICKS(188 - 400));
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_11BIT)
      vTaskDelay(pdMS_TO_TICKS(375 - 400));
#else
      vTaskDelay(pdMS_TO_TICKS(750 - 400));
#endif

      /* Read scratchpad */

      ret = ds18b20_read_scratchpad(scratchpad, NULL);
      if (ret == ESP_OK)
        {
          raw = (int16_t)((scratchpad[1] << 8) | scratchpad[0]);
          temp = (float)raw / 16.0f;

#if defined(CONFIG_MAIA_DS18B20_UNIT_FAHRENHEIT)
          temp = (temp * 9.0f / 5.0f) + 32.0f;
#elif defined(CONFIG_MAIA_DS18B20_UNIT_KELVIN)
          temp = temp + 273.15f;
#endif

          ESP_LOGI(TAG, "Temperature: %.2f° (async)", temp);
          ESP_LOGI(TAG, "Scratchpad: %02X %02X %02X %02X %02X %02X %02X "
                   "%02X CRC=%02X",
                   scratchpad[0], scratchpad[1], scratchpad[2],
                   scratchpad[3], scratchpad[4], scratchpad[5],
                   scratchpad[6], scratchpad[7], scratchpad[8]);
        }
      else
        {
          ESP_LOGE(TAG, "Read scratchpad failed: %s",
                   esp_err_to_name(ret));
        }

      vTaskDelay(pdMS_TO_TICKS(TEST_READ_INTERVAL_MS));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_ds18b20_run
 *
 * Description:
 *   Run DS18B20 temperature sensor tests.
 *
 ****************************************************************************/

void test_ds18b20_run(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Starting DS18B20 test");
  ESP_LOGI(TAG, "Hardware: DS18B20 on GPIO%d", CONFIG_MAIA_DS18B20_GPIO);

#if defined(CONFIG_MAIA_DS18B20_UNIT_CELSIUS)
  ESP_LOGI(TAG, "Unit: Celsius");
#elif defined(CONFIG_MAIA_DS18B20_UNIT_FAHRENHEIT)
  ESP_LOGI(TAG, "Unit: Fahrenheit");
#else
  ESP_LOGI(TAG, "Unit: Kelvin");
#endif

#if defined(CONFIG_MAIA_DS18B20_RESOLUTION_9BIT)
  ESP_LOGI(TAG, "Resolution: 9-bit (0.5°, 94ms)");
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_10BIT)
  ESP_LOGI(TAG, "Resolution: 10-bit (0.25°, 188ms)");
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_11BIT)
  ESP_LOGI(TAG, "Resolution: 11-bit (0.125°, 375ms)");
#else
  ESP_LOGI(TAG, "Resolution: 12-bit (0.0625°, 750ms)");
#endif

  /* Initialize driver */

  ret = ds18b20_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to initialize DS18B20: %s",
               esp_err_to_name(ret));
      return;
    }

  /* Test 1: Blocking read */

  test_ds18b20_blocking();

  /* Test 2: Async read */

  test_ds18b20_async();

  /* Deinitialize */

  ds18b20_deinit();

  ESP_LOGI(TAG, "DS18B20 test completed");
}