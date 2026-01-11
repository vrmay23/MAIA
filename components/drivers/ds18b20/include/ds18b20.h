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
 * components/drivers/ds18b20/include/ds18b20.h
 *
 * DS18B20 OneWire temperature sensor driver
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_DS18B20_INCLUDE_DS18B20_H
#define __COMPONENTS_DRIVERS_DS18B20_INCLUDE_DS18B20_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DS18B20 scratchpad size (9 bytes) */

#define DS18B20_SCRATCHPAD_SIZE 9

/* DS18B20 ROM address size (64-bit) */

#define DS18B20_ROM_SIZE 8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DS18B20 ROM address structure */

typedef struct
{
  uint8_t rom[DS18B20_ROM_SIZE];
} ds18b20_rom_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ds18b20_init
 *
 * Description:
 *   Initialize DS18B20 temperature sensor driver.
 *   Configures OneWire bus and sensor resolution from Kconfig.
 *
 * Input Parameters:
 *   None (uses Kconfig settings)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure
 *
 ****************************************************************************/

esp_err_t ds18b20_init(void);

/****************************************************************************
 * Name: ds18b20_deinit
 *
 * Description:
 *   Deinitialize DS18B20 driver and release resources.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success
 *
 ****************************************************************************/

esp_err_t ds18b20_deinit(void);

/****************************************************************************
 * Name: ds18b20_read_temperature
 *
 * Description:
 *   Read temperature from DS18B20 sensor (blocking).
 *   Triggers conversion, waits, and reads result.
 *   Returns value in unit configured via Kconfig (C, F, or K).
 *
 * Input Parameters:
 *   temp - Pointer to store temperature reading
 *   rom  - ROM address (NULL for Skip ROM mode)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure
 *
 ****************************************************************************/

esp_err_t ds18b20_read_temperature(float *temp,
                                    const ds18b20_rom_t *rom);

/****************************************************************************
 * Name: ds18b20_trigger_conversion
 *
 * Description:
 *   Trigger temperature conversion without blocking (async).
 *   Caller must wait conversion time before reading result.
 *
 * Input Parameters:
 *   rom - ROM address (NULL for Skip ROM mode)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure
 *
 * Notes:
 *   Conversion time depends on resolution:
 *     9-bit:  94ms
 *     10-bit: 188ms
 *     11-bit: 375ms
 *     12-bit: 750ms
 *
 ****************************************************************************/

esp_err_t ds18b20_trigger_conversion(const ds18b20_rom_t *rom);

/****************************************************************************
 * Name: ds18b20_read_scratchpad
 *
 * Description:
 *   Read DS18B20 scratchpad memory (9 bytes).
 *   Useful for debugging and direct register access.
 *
 * Input Parameters:
 *   data - Buffer to store scratchpad (9 bytes)
 *   rom  - ROM address (NULL for Skip ROM mode)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure
 *
 * Scratchpad Layout:
 *   [0] = Temperature LSB
 *   [1] = Temperature MSB
 *   [2] = TH register (alarm high)
 *   [3] = TL register (alarm low)
 *   [4] = Configuration register (resolution)
 *   [5-7] = Reserved
 *   [8] = CRC
 *
 ****************************************************************************/

esp_err_t ds18b20_read_scratchpad(uint8_t *data,
                                   const ds18b20_rom_t *rom);

/****************************************************************************
 * Name: ds18b20_search_roms
 *
 * Description:
 *   Search all DS18B20 devices on OneWire bus.
 *   Only useful in Match ROM mode with multiple sensors.
 *
 * Input Parameters:
 *   roms      - Array to store found ROM addresses
 *   max_roms  - Maximum number of ROMs array can hold
 *   num_found - Pointer to store number of devices found
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure
 *
 ****************************************************************************/

esp_err_t ds18b20_search_roms(ds18b20_rom_t *roms, uint8_t max_roms,
                               uint8_t *num_found);

#endif /* __COMPONENTS_DRIVERS_DS18B20_INCLUDE_DS18B20_H */