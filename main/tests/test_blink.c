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
 * main/tests/test_blink.c
 *
 * MAIA - LED blink test
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include "maia_board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[TEST_BLINK]"
#define LED_PIN CONFIG_MAIA_LED_STATUS_PIN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_blink_run
 *
 * Description:
 *   Run LED blink test. Simple hardware validation.
 *
 ****************************************************************************/

void test_blink_run(void)
{
  ESP_LOGI(TAG, "=== LED Blink Test ===");
  ESP_LOGI(TAG, "Blinking LED on GPIO%d", LED_PIN);

  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  while (1)
    {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}