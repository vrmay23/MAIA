/*
 * Copyright 2026 Vinicius May
 *
 * Licensed under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of
 * the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in
 * writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing
 * permissions and limitations under the License.
 */

/**********************************************************
 * components/drivers/vl53l5cx/src/vl53l5cx_platform.c
 *
 * MAIA VL53L5CX Platform I2C Implementation
 *
 * Implements platform callbacks declared in platform.h
 * and required by the ST VL53L5CX ULD (vl53l5cx_api.c).
 * ST ULD is BSD-3-Clause, (c) STMicroelectronics.
 * Source: github.com/STMicroelectronics/stm32-vl53l5cx
 *
 * All I2C operations use ESP-IDF v6.0 new master API.
 **********************************************************/

/**********************************************************
 * Included Files
 **********************************************************/

#include "platform.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**********************************************************
 * Pre-processor Definitions
 **********************************************************/

#define TAG  "[VL53L5CX_PLAT]"

/* Per-transaction I2C timeout (ms).
 * 1000ms gives large margin for 80KB firmware upload. */

#define MAIA_TOF_I2C_TIMEOUT_MS  1000

/* ESP-IDF's new I2C master driver can occasionally
 * report ESP_ERR_INVALID_STATE for a transaction that
 * actually completed on the wire (observed directly:
 * an address-reassignment WrByte "failed" yet the
 * sensor had verifiably taken the new address). Retry
 * a few times before giving up. */

#define MAIA_TOF_I2C_MAX_ATTEMPTS    3
#define MAIA_TOF_I2C_RETRY_DELAY_MS  2

/**********************************************************
 * Public Functions
 **********************************************************/

/**********************************************************
 * Name: WrMulti
 *
 * Description:
 *   Write N bytes to a 16-bit register address.
 *   Uses scatter-gather transmit: two-buffer array
 *   [reg_addr(2), data(N)] sent in one transaction.
 *   No heap allocation. No intermediate copy.
 *
 * Input Parameters:
 *   p_platform      - Platform context
 *   RegisterAddress - 16-bit register (big-endian)
 *   p_values        - Data to write
 *   size            - Byte count
 *
 * Returned Value:
 *   0 on success, 1 on error (ST ULD convention).
 *
 **********************************************************/

uint8_t WrMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values,
                uint32_t size)
{
  i2c_master_transmit_multi_buffer_info_t bufs[2];
  uint8_t                                 reg[2];
  esp_err_t                               ret;
  int                                     attempt;

  reg[0] = (uint8_t)((RegisterAddress >> 8) & 0xFF);
  reg[1] = (uint8_t)(RegisterAddress & 0xFF);

  bufs[0].write_buffer = reg;
  bufs[0].buffer_size  = sizeof(reg);
  bufs[1].write_buffer = p_values;
  bufs[1].buffer_size  = (size_t)size;

  for (attempt = 0; attempt < MAIA_TOF_I2C_MAX_ATTEMPTS;
       attempt++)
    {
      ret = i2c_master_multi_buffer_transmit(
                p_platform->dev_handle,
                bufs,
                2,
                MAIA_TOF_I2C_TIMEOUT_MS);
      if (ret == ESP_OK)
        {
          break;
        }

      vTaskDelay(pdMS_TO_TICKS(
          MAIA_TOF_I2C_RETRY_DELAY_MS));
    }

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "WrMulti 0x%02x reg=0x%04x "
               "size=%lu: %s",
               p_platform->address,
               RegisterAddress,
               (unsigned long)size,
               esp_err_to_name(ret));
      return 1;
    }

  return 0;
}

/**********************************************************
 * Name: WrByte
 *
 * Description:
 *   Write one byte to a 16-bit register address.
 *   Packs reg(2) + value(1) into 3-byte stack buffer.
 *   Single i2c_master_transmit() call, no intermediary.
 *
 * Input Parameters:
 *   p_platform      - Platform context
 *   RegisterAddress - 16-bit register (big-endian)
 *   value           - Byte to write
 *
 * Returned Value:
 *   0 on success, 1 on error (ST ULD convention).
 *
 **********************************************************/

uint8_t WrByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress,
               uint8_t value)
{
  uint8_t   buf[3];
  esp_err_t ret;
  int       attempt;

  buf[0] = (uint8_t)((RegisterAddress >> 8) & 0xFF);
  buf[1] = (uint8_t)(RegisterAddress & 0xFF);
  buf[2] = value;

  for (attempt = 0; attempt < MAIA_TOF_I2C_MAX_ATTEMPTS;
       attempt++)
    {
      ret = i2c_master_transmit(
                p_platform->dev_handle,
                buf,
                sizeof(buf),
                MAIA_TOF_I2C_TIMEOUT_MS);
      if (ret == ESP_OK)
        {
          break;
        }

      vTaskDelay(pdMS_TO_TICKS(
          MAIA_TOF_I2C_RETRY_DELAY_MS));
    }

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "WrByte 0x%02x reg=0x%04x "
               "val=0x%02x: %s",
               p_platform->address,
               RegisterAddress,
               value,
               esp_err_to_name(ret));
      return 1;
    }

  return 0;
}

/**********************************************************
 * Name: RdMulti
 *
 * Description:
 *   Read N bytes from a 16-bit register address.
 *   Atomic write-then-read via transmit_receive().
 *   Repeated START handled automatically by ESP-IDF.
 *
 * Input Parameters:
 *   p_platform      - Platform context
 *   RegisterAddress - 16-bit register (big-endian)
 *   p_values        - Receive buffer
 *   size            - Byte count
 *
 * Returned Value:
 *   0 on success, 1 on error (ST ULD convention).
 *
 **********************************************************/

uint8_t RdMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values,
                uint32_t size)
{
  uint8_t   reg[2];
  esp_err_t ret;
  int       attempt;

  reg[0] = (uint8_t)((RegisterAddress >> 8) & 0xFF);
  reg[1] = (uint8_t)(RegisterAddress & 0xFF);

  for (attempt = 0; attempt < MAIA_TOF_I2C_MAX_ATTEMPTS;
       attempt++)
    {
      ret = i2c_master_transmit_receive(
                p_platform->dev_handle,
                reg,
                sizeof(reg),
                p_values,
                (size_t)size,
                MAIA_TOF_I2C_TIMEOUT_MS);
      if (ret == ESP_OK)
        {
          break;
        }

      vTaskDelay(pdMS_TO_TICKS(
          MAIA_TOF_I2C_RETRY_DELAY_MS));
    }

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "RdMulti 0x%02x reg=0x%04x "
               "size=%lu: %s",
               p_platform->address,
               RegisterAddress,
               (unsigned long)size,
               esp_err_to_name(ret));
      return 1;
    }

  return 0;
}

/**********************************************************
 * Name: RdByte
 *
 * Description:
 *   Read one byte from a 16-bit register address.
 *   Same atomic transaction as RdMulti, size fixed to 1.
 *
 * Input Parameters:
 *   p_platform      - Platform context
 *   RegisterAddress - 16-bit register (big-endian)
 *   p_value         - Receive buffer (1 byte)
 *
 * Returned Value:
 *   0 on success, 1 on error (ST ULD convention).
 *
 **********************************************************/

uint8_t RdByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress,
               uint8_t *p_value)
{
  uint8_t   reg[2];
  esp_err_t ret;
  int       attempt;

  reg[0] = (uint8_t)((RegisterAddress >> 8) & 0xFF);
  reg[1] = (uint8_t)(RegisterAddress & 0xFF);

  for (attempt = 0; attempt < MAIA_TOF_I2C_MAX_ATTEMPTS;
       attempt++)
    {
      ret = i2c_master_transmit_receive(
                p_platform->dev_handle,
                reg,
                sizeof(reg),
                p_value,
                1,
                MAIA_TOF_I2C_TIMEOUT_MS);
      if (ret == ESP_OK)
        {
          break;
        }

      vTaskDelay(pdMS_TO_TICKS(
          MAIA_TOF_I2C_RETRY_DELAY_MS));
    }

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "RdByte 0x%02x reg=0x%04x: %s",
               p_platform->address,
               RegisterAddress,
               esp_err_to_name(ret));
      return 1;
    }

  return 0;
}

/**********************************************************
 * Name: SwapBuffer
 *
 * Description:
 *   Reverse byte order of each 4-byte word in buffer.
 *   In-place swap using two tmp variables per word.
 *   Buffer size is always a multiple of 4.
 *
 * Input Parameters:
 *   buffer - Buffer to swap in-place
 *   size   - Size in bytes (multiple of 4)
 *
 * Returned Value:
 *   None
 *
 **********************************************************/

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
  uint16_t i;
  uint8_t  tmp_a;
  uint8_t  tmp_b;

  for (i = 0; i < size; i += 4)
    {
      tmp_a        = buffer[i];
      tmp_b        = buffer[i + 1];
      buffer[i]    = buffer[i + 3];
      buffer[i + 1] = buffer[i + 2];
      buffer[i + 2] = tmp_b;
      buffer[i + 3] = tmp_a;
    }
}

/**********************************************************
 * Name: WaitMs
 *
 * Description:
 *   Delay via FreeRTOS vTaskDelay() — cooperative yield,
 *   allows other tasks to run during sensor waits.
 *   p_platform unused but required by ST ULD signature.
 *
 * Input Parameters:
 *   p_platform - Platform context (unused)
 *   TimeMs     - Delay in milliseconds
 *
 * Returned Value:
 *   0 always (ST ULD convention).
 *
 **********************************************************/

uint8_t WaitMs(VL53L5CX_Platform *p_platform,
               uint32_t TimeMs)
{
  (void)p_platform;
  vTaskDelay(pdMS_TO_TICKS(TimeMs));
  return 0;
}
