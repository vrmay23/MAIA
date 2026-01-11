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
 * main/tests/test_button.c
 *
 * MAIA - Button driver test
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "button.h"
#include "maia_board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[TEST_BUTTON]"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_test_handler
 *
 * Description:
 *   Button event handler for testing all button events.
 *
 ****************************************************************************/

static void button_test_handler(button_event_t event)
{
  switch(event)
    {
      case BUTTON_EVENT_PRESSED:
        ESP_LOGI(TAG, ">>> BUTTON PRESSED");
        maia_led_set(true);
        break;

      case BUTTON_EVENT_RELEASED:
        ESP_LOGI(TAG, ">>> BUTTON RELEASED");
        maia_led_set(false);
        break;

      case BUTTON_EVENT_SINGLE_CLICK:
        ESP_LOGI(TAG, ">>> SINGLE CLICK");
        break;

      case BUTTON_EVENT_DOUBLE_CLICK:
        ESP_LOGI(TAG, ">>> DOUBLE CLICK");
        break;

      case BUTTON_EVENT_LONG_PRESS:
        ESP_LOGI(TAG, ">>> LONG PRESS (held for 2s)");
        break;

      case BUTTON_EVENT_EXTRA_LONG_PRESS_1:
        ESP_LOGI(TAG, ">>> EXTRA LONG PRESS 1 (held for 7s)");
        break;

      case BUTTON_EVENT_EXTRA_LONG_PRESS_2:
        ESP_LOGI(TAG, ">>> EXTRA LONG PRESS 2 (held for 12s)");
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_button_run
 *
 * Description:
 *   Run button driver test. Tests all 7 button events:
 *   - PRESSED/RELEASED (immediate)
 *   - SINGLE_CLICK (< 2s press)
 *   - DOUBLE_CLICK (2 presses < 500ms apart)
 *   - LONG_PRESS (hold 2s)
 *   - EXTRA_LONG_PRESS_1 (hold 7s)
 *   - EXTRA_LONG_PRESS_2 (hold 12s)
 *
 ****************************************************************************/

void test_button_run(void)
{
  ESP_LOGI(TAG, "=== Button Driver Test ===");
  ESP_LOGI(TAG, "Test scenarios:");
  ESP_LOGI(TAG, "  1. Short press:         SINGLE_CLICK");
  ESP_LOGI(TAG, "  2. Two short presses:   DOUBLE_CLICK");
  ESP_LOGI(TAG, "  3. Hold 2 seconds:      LONG_PRESS");
  ESP_LOGI(TAG, "  4. Hold 7 seconds:      EXTRA_LONG_PRESS_1");
  ESP_LOGI(TAG, "  5. Hold 12 seconds:     EXTRA_LONG_PRESS_2");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Initializing board...");

  /* Initialize button driver */

  esp_err_t ret = button_init(button_test_handler);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to initialize button driver!");
      return;
    }

  ESP_LOGI(TAG, "Button driver initialized successfully");
  ESP_LOGI(TAG, "Press the button to start testing...");
  ESP_LOGI(TAG, "");

  /* Keep test running */

  while (1)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}