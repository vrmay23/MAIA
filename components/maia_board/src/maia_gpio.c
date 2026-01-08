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
 * components/maia_board/src/maia_gpio.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * GPIO initialization for interrupts, button, and ToF control pins
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_board.h"
#include <driver/gpio.h>
#include <esp_log.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "maia_gpio"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_gpio_init
 *
 * Description:
 *   Initialize GPIO pins:
 *   - ToF LPn pins (output, active low enable)
 *   - ToF INT pins (input, internal pull-up)
 *   - IMU INT pin (input, internal pull-up)
 *   - Button pin (input, external pull-up)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_gpio_init(void)
{
  esp_err_t ret;
  gpio_config_t io_conf;

  ESP_LOGI(TAG, "Initializing GPIO pins");

  /* ToF LPn pins - Output, default HIGH (sensor disabled at boot)
   * LPn is active low: LOW = sensor enabled, HIGH = sensor disabled
   * External 47k pull-up as per VL53L5CX datasheet
   */

  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_TOF1_LPN) |
                         (1ULL << MAIA_GPIO_TOF2_LPN);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure ToF LPn pins");
      return ret;
    }

  /* Start with both ToF sensors disabled */

  gpio_set_level(MAIA_GPIO_TOF1_LPN, 1);
  gpio_set_level(MAIA_GPIO_TOF2_LPN, 1);

  /* ToF INT pins - Input with internal pull-up
   * INT is active low, open-drain output from sensor
   */

  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_TOF1_INT) |
                         (1ULL << MAIA_GPIO_TOF2_INT);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure ToF INT pins");
      return ret;
    }

  /* IMU INT pin - Input with internal pull-up
   * MPU6050 INT is configurable, default active high
   */

  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_IMU_INT);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure IMU INT pin");
      return ret;
    }

  /* Button pin - Input, external pull-up provided
   * Active low (pressed = LOW)
   */

  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_BUTTON);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;  /* External pull-up */

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure button pin");
      return ret;
    }

  ESP_LOGI(TAG, "GPIO pins initialized successfully");

  return ESP_OK;
}