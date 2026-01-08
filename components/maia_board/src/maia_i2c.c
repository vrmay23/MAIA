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
 * components/maia_board/src/maia_i2c.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * I2C bus initialization and management
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

#define TAG "maia_i2c"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static i2c_master_bus_handle_t g_i2c_bus_handle = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_i2c_init
 *
 * Description:
 *   Initialize I2C master bus with SDA=GPIO5, SCL=GPIO6 at 400kHz.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_i2c_init(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Initializing I2C bus (SDA=%d, SCL=%d, %dHz)",
           MAIA_GPIO_I2C_SDA, MAIA_GPIO_I2C_SCL, MAIA_I2C_FREQ_HZ);

  i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = MAIA_I2C_PORT,
      .scl_io_num = MAIA_GPIO_I2C_SCL,
      .sda_io_num = MAIA_GPIO_I2C_SDA,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = false,  /* External 2.2k pull-ups */
  };

  ret = i2c_new_master_bus(&bus_config, &g_i2c_bus_handle);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to create I2C master bus: %s",
               esp_err_to_name(ret));
      return ret;
    }

  ESP_LOGI(TAG, "I2C bus initialized successfully");

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_i2c_get_bus_handle
 *
 * Description:
 *   Get the I2C master bus handle for device registration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   I2C master bus handle, or NULL if not initialized.
 *
 ****************************************************************************/

i2c_master_bus_handle_t maia_i2c_get_bus_handle(void)
{
  return g_i2c_bus_handle;
}