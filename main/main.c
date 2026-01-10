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
 * main/main.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * Application entry point
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef CONFIG_MAIA_TEST_BLINK
#  include <driver/gpio.h>
#endif

#include "app.h"
#include "maia_board.h"  


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MAIA_TEST_BLINK
#  define LED_PIN 21
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_main
 *
 * Description:
 *   Application entry point.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void app_main(void)
{
#ifdef CONFIG_MAIA_TEST_BLINK
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  while (1)
    {
      gpio_set_level(LED_PIN, 1);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
#else
  maia_board_init();
  app_init();
#endif
}