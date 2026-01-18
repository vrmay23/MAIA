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
 * components/drivers/drv2605l/src/drv2605l.c
 *
 * DRV2605L Haptic Motor Driver Implementation
 *
 * Reference: SLOS850D - DRV2605L Datasheet (Rev. D, October 2013)
 * https://www.ti.com/lit/ds/symlink/drv2605l.pdf
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "drv2605l.h"
#include "maia_board.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[DRV2605L]"

/* I2C timeout */

#define DRV2605L_I2C_TIMEOUT_MS 1000

/* Auto-calibration timeout (datasheet: max 1.2s) */

#define DRV2605L_AUTOCAL_TIMEOUT_MS 2000

/* Status register bits (Section 8.5.1, Page 37) */

#define DRV2605L_STATUS_DEVICE_ID     0xE0  /* Device ID mask */
#define DRV2605L_STATUS_DIAG_RESULT   0x08  /* Diagnostic result bit */
#define DRV2605L_STATUS_OVER_TEMP     0x02  /* Over-temperature bit */
#define DRV2605L_STATUS_OC_DETECT     0x01  /* Over-current bit */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_initialized = false;
static drv2605l_config_t g_config;
static i2c_master_dev_handle_t g_dev_handle = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605l_i2c_write_reg
 *
 * Description:
 *   Write single byte to DRV2605L register using new I2C driver.
 *
 ****************************************************************************/

static esp_err_t drv2605l_i2c_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t write_buf[2] = {reg, value};
  
  return i2c_master_transmit(g_dev_handle, write_buf, 2,
                              DRV2605L_I2C_TIMEOUT_MS);
}

/****************************************************************************
 * Name: drv2605l_i2c_read_reg
 *
 * Description:
 *   Read single byte from DRV2605L register using new I2C driver.
 *
 ****************************************************************************/

static esp_err_t drv2605l_i2c_read_reg(uint8_t reg, uint8_t *value)
{
  if (value == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  return i2c_master_transmit_receive(g_dev_handle, &reg, 1, value, 1,
                                      DRV2605L_I2C_TIMEOUT_MS);
}

/****************************************************************************
 * Name: drv2605l_get_library_from_config
 *
 * Description:
 *   Get library selection from Kconfig.
 *
 ****************************************************************************/

static drv2605l_library_t drv2605l_get_library_from_config(void)
{
#ifdef CONFIG_MAIA_DRV2605L_ACTUATOR_LRA
  return DRV2605L_LIB_LRA;
#else
  #ifdef CONFIG_MAIA_DRV2605L_LIBRARY_A
    return DRV2605L_LIB_ERM_A;
  #elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_B)
    return DRV2605L_LIB_ERM_B;
  #elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_C)
    return DRV2605L_LIB_ERM_C;
  #elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_D)
    return DRV2605L_LIB_ERM_D;
  #elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_E)
    return DRV2605L_LIB_ERM_E;
  #else
    return DRV2605L_LIB_ERM_A;  /* Default to Library A */
  #endif
#endif
}

/****************************************************************************
 * Name: drv2605l_run_autocalibration
 *
 * Description:
 *   Run auto-calibration procedure.
 *   Datasheet Section 9.2 (Auto-Calibration), Page 78
 *
 ****************************************************************************/

static esp_err_t drv2605l_run_autocalibration(void)
{
  esp_err_t ret;
  uint8_t status;
  uint32_t timeout_start;

  ESP_LOGI(TAG, "Running auto-calibration (takes ~1-2 seconds)...");

  /* Set auto-calibration mode */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_MODE,
                                DRV2605L_MODE_AUTOCAL);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set auto-cal mode");
      return ret;
    }

  /* Trigger calibration by setting GO bit */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_GO, DRV2605L_GO_BIT);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to trigger auto-calibration");
      return ret;
    }

  /* Wait for calibration to complete (GO bit clears when done) */

  timeout_start = xTaskGetTickCount();
  while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(100));

      ret = drv2605l_i2c_read_reg(DRV2605L_REG_GO, &status);
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "Failed to read GO register");
          return ret;
        }

      if ((status & DRV2605L_GO_BIT) == 0)
        {
          break;  /* Calibration complete */
        }

      if ((xTaskGetTickCount() - timeout_start) >
          pdMS_TO_TICKS(DRV2605L_AUTOCAL_TIMEOUT_MS))
        {
          ESP_LOGE(TAG, "Auto-calibration timeout");
          return ESP_ERR_TIMEOUT;
        }
    }

  /* Check calibration result in status register */

  ret = drv2605l_i2c_read_reg(DRV2605L_REG_STATUS, &status);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to read status register");
      return ret;
    }

  if (status & DRV2605L_STATUS_DIAG_RESULT)
    {
      ESP_LOGE(TAG, "Auto-calibration failed (DIAG_RESULT=1)");
      return ESP_FAIL;
    }

  /* Read calibration results */

  uint8_t comp_result;
  uint8_t bemf_result;

  drv2605l_i2c_read_reg(DRV2605L_REG_AUTOCALCOMP, &comp_result);
  drv2605l_i2c_read_reg(DRV2605L_REG_AUTOCALEMP, &bemf_result);

  ESP_LOGI(TAG, "Auto-calibration successful:");
  ESP_LOGI(TAG, "  Compensation: 0x%02X", comp_result);
  ESP_LOGI(TAG, "  Back-EMF:     0x%02X", bemf_result);

  /* Return to standby mode */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_MODE,
                                DRV2605L_MODE_STANDBY);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to return to standby");
      return ret;
    }

  return ESP_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605l_init
 *
 * Description:
 *   Initialize DRV2605L driver.
 *   Datasheet Section 9.1 (Power-On Initialization), Page 76
 *
 ****************************************************************************/

esp_err_t drv2605l_init(const drv2605l_config_t *config)
{
  esp_err_t ret;
  uint8_t status;

  if (config == NULL)
    {
      ESP_LOGE(TAG, "Invalid config (NULL)");
      return ESP_ERR_INVALID_ARG;
    }

  if (g_initialized)
    {
      ESP_LOGW(TAG, "Already initialized");
      return ESP_OK;
    }

  /* Store configuration */

  memcpy(&g_config, config, sizeof(drv2605l_config_t));

  ESP_LOGI(TAG, "Initializing DRV2605L (I2C addr: 0x%02X)",
           g_config.i2c_addr);

  /* Get I2C bus handle from maia_board */

  i2c_master_bus_handle_t bus_handle = maia_i2c_get_bus_handle();
  if (bus_handle == NULL)
    {
      ESP_LOGE(TAG, "I2C bus not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  /* Register device on I2C bus */

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = g_config.i2c_addr,
      .scl_speed_hz = CONFIG_MAIA_I2C_FREQ_HZ,
  };

  ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &g_dev_handle);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to add device to I2C bus: %s",
               esp_err_to_name(ret));
      return ret;
    }

  /* Read status register to verify I2C communication */

  ret = drv2605l_i2c_read_reg(DRV2605L_REG_STATUS, &status);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to communicate with device (I2C error)");
      return ret;
    }

  ESP_LOGI(TAG, "Device ID: 0x%02X", (status & DRV2605L_STATUS_DEVICE_ID)
           >> 5);

  /* Clear standby bit (wake up device) */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_MODE, 0x00);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to wake up device");
      return ret;
    }

  /* Set actuator type (ERM vs LRA)
   * Datasheet Section 8.5.19 (Feedback Control), Page 66
   */

  uint8_t feedback = (g_config.actuator == DRV2605L_ACTUATOR_LRA) ?
                     DRV2605L_FEEDBACK_LRA : DRV2605L_FEEDBACK_ERM;

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_FEEDBACK, feedback);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set actuator type");
      return ret;
    }

  /* Set library selection
   * Datasheet Section 8.5.4 (Library Selection), Page 51
   */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_LIBRARY, g_config.library);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set library");
      return ret;
    }

  ESP_LOGI(TAG, "Actuator: %s, Library: %d",
           (g_config.actuator == DRV2605L_ACTUATOR_ERM) ? "ERM" : "LRA",
           g_config.library);

  /* Set rated voltage and overdrive clamp
   * Datasheet Section 8.5.16-17, Page 64-65
   */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_RATEDV,
                                g_config.rated_voltage);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set rated voltage");
      return ret;
    }

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_CLAMPV,
                                g_config.overdrive_clamp);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set overdrive clamp");
      return ret;
    }

  ESP_LOGI(TAG, "Rated voltage: %d, Overdrive clamp: %d",
           g_config.rated_voltage, g_config.overdrive_clamp);

  /* Run auto-calibration if enabled */

  if (g_config.auto_calibrate)
    {
      ret = drv2605l_run_autocalibration();
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "Auto-calibration failed");
          return ret;
        }
    }

  /* Set internal trigger mode (library effects) */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_MODE,
                                DRV2605L_MODE_INTTRIG);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set internal trigger mode");
      return ret;
    }

  g_initialized = true;
  ESP_LOGI(TAG, "DRV2605L initialized successfully");

  return ESP_OK;
}

/****************************************************************************
 * Name: drv2605l_play_effect
 *
 * Description:
 *   Play single library effect.
 *   Datasheet Section 8.5.5 (Waveform Sequencer), Page 53
 *
 ****************************************************************************/

esp_err_t drv2605l_play_effect(uint8_t effect_id)
{
  esp_err_t ret;

  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  if (effect_id < DRV2605L_EFFECT_MIN || effect_id > DRV2605L_EFFECT_MAX)
    {
      ESP_LOGE(TAG, "Invalid effect ID: %d (valid range: 1-123)",
               effect_id);
      return ESP_ERR_INVALID_ARG;
    }

  /* Load effect into sequencer register 1 */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_WAVESEQ1, effect_id);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to write effect to sequencer");
      return ret;
    }

  /* Set end-of-sequence marker in register 2 */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_WAVESEQ2,
                                DRV2605L_EFFECT_STOP);
  if (ret != ESP_OK)
    {
      return ret;
    }

  /* Trigger playback by setting GO bit */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_GO, DRV2605L_GO_BIT);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to trigger effect playback");
      return ret;
    }

  return ESP_OK;
}

/****************************************************************************
 * Name: drv2605l_play_sequence
 *
 * Description:
 *   Play sequence of up to 8 effects.
 *   Datasheet Section 8.5.5-8.5.8 (Waveform Sequencer), Page 53-57
 *
 ****************************************************************************/

esp_err_t drv2605l_play_sequence(const uint8_t *effects,
                                  uint8_t num_effects)
{
  esp_err_t ret;

  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  if (effects == NULL || num_effects == 0 || num_effects > 8)
    {
      ESP_LOGE(TAG, "Invalid sequence (must be 1-8 effects)");
      return ESP_ERR_INVALID_ARG;
    }

  /* Load effects into sequencer registers */

  for (uint8_t i = 0; i < num_effects; i++)
    {
      if (effects[i] < DRV2605L_EFFECT_MIN ||
          effects[i] > DRV2605L_EFFECT_MAX)
        {
          ESP_LOGE(TAG, "Invalid effect ID in sequence: %d", effects[i]);
          return ESP_ERR_INVALID_ARG;
        }

      ret = drv2605l_i2c_write_reg(DRV2605L_REG_WAVESEQ1 + i,
                                    effects[i]);
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "Failed to write effect %d to sequencer", i);
          return ret;
        }
    }

  /* Set end-of-sequence marker */

  if (num_effects < 8)
    {
      ret = drv2605l_i2c_write_reg(DRV2605L_REG_WAVESEQ1 + num_effects,
                                    DRV2605L_EFFECT_STOP);
      if (ret != ESP_OK)
        {
          return ret;
        }
    }

  /* Trigger playback */

  ret = drv2605l_i2c_write_reg(DRV2605L_REG_GO, DRV2605L_GO_BIT);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to trigger sequence playback");
      return ret;
    }

  return ESP_OK;
}

/****************************************************************************
 * Name: drv2605l_stop
 *
 * Description:
 *   Stop current playback immediately.
 *
 ****************************************************************************/

esp_err_t drv2605l_stop(void)
{
  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  /* Clear GO bit to stop playback */

  return drv2605l_i2c_write_reg(DRV2605L_REG_GO, 0x00);
}

/****************************************************************************
 * Name: drv2605l_set_mode
 *
 * Description:
 *   Set device operation mode.
 *   Datasheet Section 8.5.2 (Mode Register), Page 39
 *
 ****************************************************************************/

esp_err_t drv2605l_set_mode(drv2605l_mode_t mode)
{
  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  return drv2605l_i2c_write_reg(DRV2605L_REG_MODE, mode);
}

/****************************************************************************
 * Name: drv2605l_set_library
 *
 * Description:
 *   Change effect library.
 *
 ****************************************************************************/

esp_err_t drv2605l_set_library(drv2605l_library_t library)
{
  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  g_config.library = library;
  return drv2605l_i2c_write_reg(DRV2605L_REG_LIBRARY, library);
}

/****************************************************************************
 * Name: drv2605l_standby
 *
 * Description:
 *   Enter low-power standby mode.
 *
 ****************************************************************************/

esp_err_t drv2605l_standby(void)
{
  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  return drv2605l_i2c_write_reg(DRV2605L_REG_MODE,
                                 DRV2605L_MODE_STANDBY);
}

/****************************************************************************
 * Name: drv2605l_wakeup
 *
 * Description:
 *   Wake from standby mode.
 *
 ****************************************************************************/

esp_err_t drv2605l_wakeup(void)
{
  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  return drv2605l_i2c_write_reg(DRV2605L_REG_MODE,
                                 DRV2605L_MODE_INTTRIG);
}

/****************************************************************************
 * Name: drv2605l_get_status
 *
 * Description:
 *   Read device status register.
 *   Datasheet Section 8.5.1 (Status Register), Page 37
 *
 ****************************************************************************/

esp_err_t drv2605l_get_status(uint8_t *status)
{
  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  if (status == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  return drv2605l_i2c_read_reg(DRV2605L_REG_STATUS, status);
}

/****************************************************************************
 * Name: drv2605l_set_rtp_value
 *
 * Description:
 *   Set real-time playback (RTP) intensity value.
 *   Used in composer mode for custom patterns.
 *   Datasheet Section 8.5.3 (Real-Time Playback), Page 50
 *
 * Input Parameters:
 *   value - Intensity (0-255, where 0=off, 255=max)
 *
 * Notes:
 *   Device must be in RTP mode (Mode 5) before calling this.
 *   Call drv2605l_set_mode(DRV2605L_OP_MODE_REALTIME) first.
 *
 ****************************************************************************/

esp_err_t drv2605l_set_rtp_value(uint8_t value)
{
  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  return drv2605l_i2c_write_reg(DRV2605L_REG_RTPIN, value);
}