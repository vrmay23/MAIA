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
 * components/maia_board/src/maia_led.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * Status LED control
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_board.h"
#include <driver/gpio.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_led_set
 *
 * Description:
 *   Set status LED on or off.
 *
 * Input Parameters:
 *   state - true for ON, false for OFF
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_led_set(bool state)
{
  return gpio_set_level(MAIA_GPIO_LED_STATUS, state ? 1 : 0);
}

/****************************************************************************
 * Name: maia_led_toggle
 *
 * Description:
 *   Toggle status LED state.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_led_toggle(void)
{
  int level = gpio_get_level(MAIA_GPIO_LED_STATUS);
  return gpio_set_level(MAIA_GPIO_LED_STATUS, !level);
}