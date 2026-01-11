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
 * components/drivers/ds18b20/src/ds18b20.c
 *
 * DS18B20 OneWire temperature sensor driver
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "ds18b20.h"
#include "maia_board.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[DS18B20]"

/* DS18B20 ROM Commands */

#define DS18B20_CMD_SEARCH_ROM      0xF0
#define DS18B20_CMD_READ_ROM        0x33
#define DS18B20_CMD_MATCH_ROM       0x55
#define DS18B20_CMD_SKIP_ROM        0xCC
#define DS18B20_CMD_ALARM_SEARCH    0xEC

/* DS18B20 Function Commands */

#define DS18B20_CMD_CONVERT_T       0x44
#define DS18B20_CMD_WRITE_SCRATCH   0x4E
#define DS18B20_CMD_READ_SCRATCH    0xBE
#define DS18B20_CMD_COPY_SCRATCH    0x48
#define DS18B20_CMD_RECALL_E2       0xB8
#define DS18B20_CMD_READ_POWER      0xB4

/* Resolution configuration bits (scratchpad byte 4) */

#define DS18B20_RES_9BIT            0x1F  /* 0 0 0 1 1 1 1 1 */
#define DS18B20_RES_10BIT           0x3F  /* 0 0 1 1 1 1 1 1 */
#define DS18B20_RES_11BIT           0x5F  /* 0 1 0 1 1 1 1 1 */
#define DS18B20_RES_12BIT           0x7F  /* 0 1 1 1 1 1 1 1 */

/* Conversion times (milliseconds) */

#define DS18B20_CONV_TIME_9BIT      94
#define DS18B20_CONV_TIME_10BIT     188
#define DS18B20_CONV_TIME_11BIT     375
#define DS18B20_CONV_TIME_12BIT     750

/****************************************************************************
 * Private Data
 ****************************************************************************/

static gpio_num_t g_onewire_pin;
static uint8_t g_resolution_config;
static uint16_t g_conversion_time_ms;
static bool g_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ds18b20_get_resolution_config
 *
 * Description:
 *   Get resolution configuration byte from Kconfig.
 *
 ****************************************************************************/

static uint8_t ds18b20_get_resolution_config(void)
{
#if defined(CONFIG_MAIA_DS18B20_RESOLUTION_9BIT)
  return DS18B20_RES_9BIT;
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_10BIT)
  return DS18B20_RES_10BIT;
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_11BIT)
  return DS18B20_RES_11BIT;
#else
  return DS18B20_RES_12BIT;  /* Default 12-bit */
#endif
}

/****************************************************************************
 * Name: ds18b20_get_conversion_time
 *
 * Description:
 *   Get conversion time in milliseconds from Kconfig.
 *
 ****************************************************************************/

static uint16_t ds18b20_get_conversion_time(void)
{
#if defined(CONFIG_MAIA_DS18B20_RESOLUTION_9BIT)
  return DS18B20_CONV_TIME_9BIT;
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_10BIT)
  return DS18B20_CONV_TIME_10BIT;
#elif defined(CONFIG_MAIA_DS18B20_RESOLUTION_11BIT)
  return DS18B20_CONV_TIME_11BIT;
#else
  return DS18B20_CONV_TIME_12BIT;  /* Default 12-bit */
#endif
}

/****************************************************************************
 * Name: ds18b20_convert_temperature
 *
 * Description:
 *   Convert raw temperature data to configured unit.
 *
 ****************************************************************************/

static float ds18b20_convert_temperature(int16_t raw)
{
  float temp_c = (float)raw / 16.0f;

#if defined(CONFIG_MAIA_DS18B20_UNIT_FAHRENHEIT)
  return (temp_c * 9.0f / 5.0f) + 32.0f;
#elif defined(CONFIG_MAIA_DS18B20_UNIT_KELVIN)
  return temp_c + 273.15f;
#else
  return temp_c;  /* Celsius (default) */
#endif
}

/****************************************************************************
 * Name: ds18b20_select_device
 *
 * Description:
 *   Select device using configured ROM mode.
 *
 ****************************************************************************/

static esp_err_t ds18b20_select_device(const ds18b20_rom_t *rom)
{
#ifdef CONFIG_MAIA_DS18B20_ROM_SKIP
  /* Skip ROM: single device on bus */

  maia_onewire_write_byte(g_onewire_pin, DS18B20_CMD_SKIP_ROM);
  return ESP_OK;

#else
  /* Match ROM: multiple devices */

  if (rom == NULL)
    {
      ESP_LOGE(TAG, "ROM address required in Match ROM mode");
      return ESP_ERR_INVALID_ARG;
    }

  maia_onewire_write_byte(g_onewire_pin, DS18B20_CMD_MATCH_ROM);
  for (uint8_t i = 0; i < DS18B20_ROM_SIZE; i++)
    {
      maia_onewire_write_byte(g_onewire_pin, rom->rom[i]);
    }
  return ESP_OK;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ds18b20_init
 *
 * Description:
 *   Initialize DS18B20 driver and configure sensor resolution.
 *
 ****************************************************************************/

esp_err_t ds18b20_init(void)
{
  esp_err_t ret;

  if (g_initialized)
    {
      ESP_LOGW(TAG, "Already initialized");
      return ESP_OK;
    }

  g_onewire_pin = CONFIG_MAIA_DS18B20_GPIO;
  g_resolution_config = ds18b20_get_resolution_config();
  g_conversion_time_ms = ds18b20_get_conversion_time();

  ESP_LOGI(TAG, "Initializing DS18B20 (GPIO%d, resolution=%d-bit)",
           g_onewire_pin, 9 + ((g_resolution_config >> 5) & 0x03));

  /* Check if device is present */

  if (!maia_onewire_reset(g_onewire_pin))
    {
      ESP_LOGE(TAG, "No DS18B20 device found on bus");
      return ESP_ERR_NOT_FOUND;
    }

  /* Write scratchpad to set resolution */

  if (!maia_onewire_reset(g_onewire_pin))
    {
      return ESP_FAIL;
    }

  ret = ds18b20_select_device(NULL);
  if (ret != ESP_OK)
    {
      return ret;
    }

  maia_onewire_write_byte(g_onewire_pin, DS18B20_CMD_WRITE_SCRATCH);
  maia_onewire_write_byte(g_onewire_pin, 0x00);  /* TH alarm (unused) */
  maia_onewire_write_byte(g_onewire_pin, 0x00);  /* TL alarm (unused) */
  maia_onewire_write_byte(g_onewire_pin, g_resolution_config);

  /* Copy scratchpad to EEPROM */

  if (!maia_onewire_reset(g_onewire_pin))
    {
      return ESP_FAIL;
    }

  ret = ds18b20_select_device(NULL);
  if (ret != ESP_OK)
    {
      return ret;
    }

  maia_onewire_write_byte(g_onewire_pin, DS18B20_CMD_COPY_SCRATCH);
  vTaskDelay(pdMS_TO_TICKS(10));  /* EEPROM write time */

  g_initialized = true;
  ESP_LOGI(TAG, "DS18B20 initialized successfully");

  return ESP_OK;
}

/****************************************************************************
 * Name: ds18b20_deinit
 *
 * Description:
 *   Deinitialize DS18B20 driver.
 *
 ****************************************************************************/

esp_err_t ds18b20_deinit(void)
{
  g_initialized = false;
  ESP_LOGI(TAG, "DS18B20 deinitialized");
  return ESP_OK;
}

/****************************************************************************
 * Name: ds18b20_trigger_conversion
 *
 * Description:
 *   Trigger temperature conversion (non-blocking).
 *   Caller must wait conversion time before reading.
 *
 ****************************************************************************/

esp_err_t ds18b20_trigger_conversion(const ds18b20_rom_t *rom)
{
  esp_err_t ret;

  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  if (!maia_onewire_reset(g_onewire_pin))
    {
      ESP_LOGE(TAG, "Device not responding");
      return ESP_ERR_NOT_FOUND;
    }

  ret = ds18b20_select_device(rom);
  if (ret != ESP_OK)
    {
      return ret;
    }

  maia_onewire_write_byte(g_onewire_pin, DS18B20_CMD_CONVERT_T);

  return ESP_OK;
}

/****************************************************************************
 * Name: ds18b20_read_scratchpad
 *
 * Description:
 *   Read scratchpad memory (9 bytes) from DS18B20.
 *
 ****************************************************************************/

esp_err_t ds18b20_read_scratchpad(uint8_t *data, const ds18b20_rom_t *rom)
{
  esp_err_t ret;

  if (!g_initialized)
    {
      ESP_LOGE(TAG, "Driver not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  if (data == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  if (!maia_onewire_reset(g_onewire_pin))
    {
      ESP_LOGE(TAG, "Device not responding");
      return ESP_ERR_NOT_FOUND;
    }

  ret = ds18b20_select_device(rom);
  if (ret != ESP_OK)
    {
      return ret;
    }

  maia_onewire_write_byte(g_onewire_pin, DS18B20_CMD_READ_SCRATCH);

  for (uint8_t i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
    {
      data[i] = maia_onewire_read_byte(g_onewire_pin);
    }

  /* Verify CRC */

  uint8_t crc = maia_onewire_crc8(data, 8);
  if (crc != data[8])
    {
      ESP_LOGE(TAG, "CRC mismatch: calculated=0x%02X, received=0x%02X",
               crc, data[8]);
      return ESP_ERR_INVALID_CRC;
    }

  return ESP_OK;
}

/****************************************************************************
 * Name: ds18b20_read_temperature
 *
 * Description:
 *   Read temperature from DS18B20 (blocking).
 *   Triggers conversion, waits, then reads result.
 *
 ****************************************************************************/

esp_err_t ds18b20_read_temperature(float *temp, const ds18b20_rom_t *rom)
{
  esp_err_t ret;
  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  int16_t raw;

  if (temp == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  /* Trigger conversion */

  ret = ds18b20_trigger_conversion(rom);
  if (ret != ESP_OK)
    {
      return ret;
    }

  /* Wait for conversion to complete */

  vTaskDelay(pdMS_TO_TICKS(g_conversion_time_ms));

  /* Read scratchpad */

  ret = ds18b20_read_scratchpad(scratchpad, rom);
  if (ret != ESP_OK)
    {
      return ret;
    }

  /* Extract temperature (12-bit signed, LSB = 0.0625°C) */

  raw = (int16_t)((scratchpad[1] << 8) | scratchpad[0]);
  *temp = ds18b20_convert_temperature(raw);

  return ESP_OK;
}

/****************************************************************************
 * Name: ds18b20_search_roms
 *
 * Description:
 *   Search all DS18B20 devices on OneWire bus.
 *   Only implemented if Match ROM mode is enabled.
 *
 ****************************************************************************/

esp_err_t ds18b20_search_roms(ds18b20_rom_t *roms, uint8_t max_roms,
                               uint8_t *num_found)
{
#ifdef CONFIG_MAIA_DS18B20_ROM_MATCH
  /* TODO: Implement ROM search algorithm */
  ESP_LOGE(TAG, "ROM search not yet implemented");
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_LOGW(TAG, "ROM search only available in Match ROM mode");
  return ESP_ERR_NOT_SUPPORTED;
#endif
}