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
 * components/maia_board/src/maia_board.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * Board initialization
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_board.h"
#include <esp_log.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIA_BOARD]"
#define TAG_LOG_EMPTY   ""

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_board_init
 *
 * Description:
 *   Initialize all board peripherals (I2C, GPIO, PWM, OneWire).
 *   Must be called before any other board function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_board_init(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "============ Initializing MAIA board ============");

#ifdef CONFIG_MAIA_LOG_GENERAL_CONFIG
  maia_config_log();
#endif

  /* Initialize GPIO pins */

  ret = maia_gpio_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to initialize GPIO");
      return ret;
    }
  
  
  /* Initialize LED */
  ret = maia_led_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to initialize LED");
      return ret;
    }

  /* Initialize I2C bus */

  ret = maia_i2c_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to initialize I2C");
      return ret;
    }

  /* Initialize PWM for motors */

  ret = maia_pwm_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to initialize PWM");
      return ret;
    }

  /* Initialize OneWire for DS18B20 */

#ifdef CONFIG_MAIA_DS18B20_ENABLE
  ret = maia_onewire_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to initialize OneWire");
      return ret;
    }
#endif

  ESP_LOGI(TAG, "MAIA board initialized successfully");

  return ESP_OK;
}