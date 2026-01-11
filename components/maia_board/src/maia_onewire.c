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
 * components/maia_board/src/maia_onewire.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * OneWire protocol implementation (Dallas/Maxim)
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_board.h"
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <driver/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIA_ONEWIRE]"

/* OneWire timing constants (microseconds) per Dallas/Maxim spec */

#define OW_RESET_LOW_US         480    /* Reset pulse duration */
#define OW_RESET_WAIT_US        70     /* Wait before sampling presence */
#define OW_RESET_RELEASE_US     410    /* Wait after presence */
#define OW_WRITE_0_LOW_US       60     /* Write 0: low duration */
#define OW_WRITE_0_HIGH_US      10     /* Write 0: recovery time */
#define OW_WRITE_1_LOW_US       6      /* Write 1: low duration */
#define OW_WRITE_1_HIGH_US      64     /* Write 1: recovery time */
#define OW_READ_LOW_US          6      /* Read: initiate slot */
#define OW_READ_WAIT_US         9      /* Read: wait before sample */
#define OW_READ_RELEASE_US      55     /* Read: recovery time */

/* CRC8 polynomial for Dallas/Maxim (X^8 + X^5 + X^4 + 1) */

#define OW_CRC8_POLY            0x8C

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: onewire_set_input
 *
 * Description:
 *   Configure GPIO as input (high-Z with pull-up).
 *
 ****************************************************************************/

static inline void onewire_set_input(gpio_num_t pin)
{
  gpio_set_direction(pin, GPIO_MODE_INPUT);
}

/****************************************************************************
 * Name: onewire_set_output_low
 *
 * Description:
 *   Configure GPIO as output and drive low.
 *
 ****************************************************************************/

static inline void onewire_set_output_low(gpio_num_t pin)
{
  gpio_set_level(pin, 0);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

/****************************************************************************
 * Name: onewire_read_pin
 *
 * Description:
 *   Read current GPIO level.
 *
 ****************************************************************************/

static inline uint8_t onewire_read_pin(gpio_num_t pin)
{
  return gpio_get_level(pin);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_onewire_init
 *
 * Description:
 *   Initialize OneWire GPIO pin for DS18B20 temperature sensor.
 *   Configures pin as input with internal pull-up disabled.
 *   External 4.7kΩ pull-up resistor required.
 *
 ****************************************************************************/

#ifdef CONFIG_MAIA_DS18B20_ENABLE
esp_err_t maia_onewire_init(void)
{
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << MAIA_GPIO_ONEWIRE),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure OneWire GPIO: %s",
               esp_err_to_name(ret));
      return ret;
    }

  ESP_LOGI(TAG, "OneWire initialized on GPIO%d (4.7k external pull-up)",
           MAIA_GPIO_ONEWIRE);
  return ESP_OK;
}
#endif

/****************************************************************************
 * Name: maia_onewire_reset
 *
 * Description:
 *   Send reset pulse and detect presence.
 *   Returns true if at least one device responded.
 *
 ****************************************************************************/

bool maia_onewire_reset(gpio_num_t pin)
{
  bool presence;

  /* Pull bus low for reset pulse */

  onewire_set_output_low(pin);
  esp_rom_delay_us(OW_RESET_LOW_US);

  /* Release bus and wait for presence pulse */

  onewire_set_input(pin);
  esp_rom_delay_us(OW_RESET_WAIT_US);

  /* Sample bus: presence = low */

  presence = !onewire_read_pin(pin);

  /* Wait for presence pulse to finish */

  esp_rom_delay_us(OW_RESET_RELEASE_US);

  return presence;
}

/****************************************************************************
 * Name: maia_onewire_write_bit
 *
 * Description:
 *   Write single bit on OneWire bus.
 *   Write 0: Pull low 60µs
 *   Write 1: Pull low 6µs, release
 *
 ****************************************************************************/

void maia_onewire_write_bit(gpio_num_t pin, uint8_t bit)
{
  if (bit & 1)
    {
      /* Write 1: short low pulse */

      onewire_set_output_low(pin);
      esp_rom_delay_us(OW_WRITE_1_LOW_US);
      onewire_set_input(pin);
      esp_rom_delay_us(OW_WRITE_1_HIGH_US);
    }
  else
    {
      /* Write 0: long low pulse */

      onewire_set_output_low(pin);
      esp_rom_delay_us(OW_WRITE_0_LOW_US);
      onewire_set_input(pin);
      esp_rom_delay_us(OW_WRITE_0_HIGH_US);
    }
}

/****************************************************************************
 * Name: maia_onewire_read_bit
 *
 * Description:
 *   Read single bit from OneWire bus.
 *   Pull low briefly, release, then sample.
 *
 ****************************************************************************/

uint8_t maia_onewire_read_bit(gpio_num_t pin)
{
  uint8_t bit;

  /* Initiate read slot */

  onewire_set_output_low(pin);
  esp_rom_delay_us(OW_READ_LOW_US);
  onewire_set_input(pin);

  /* Wait for slave to drive bus */

  esp_rom_delay_us(OW_READ_WAIT_US);

  /* Sample bus */

  bit = onewire_read_pin(pin);

  /* Complete read slot */

  esp_rom_delay_us(OW_READ_RELEASE_US);

  return bit;
}

/****************************************************************************
 * Name: maia_onewire_write_byte
 *
 * Description:
 *   Write byte on OneWire bus (LSB first).
 *
 ****************************************************************************/

void maia_onewire_write_byte(gpio_num_t pin, uint8_t byte)
{
  for (uint8_t i = 0; i < 8; i++)
    {
      maia_onewire_write_bit(pin, byte & 0x01);
      byte >>= 1;
    }
}

/****************************************************************************
 * Name: maia_onewire_read_byte
 *
 * Description:
 *   Read byte from OneWire bus (LSB first).
 *
 ****************************************************************************/

uint8_t maia_onewire_read_byte(gpio_num_t pin)
{
  uint8_t byte = 0;

  for (uint8_t i = 0; i < 8; i++)
    {
      byte >>= 1;
      if (maia_onewire_read_bit(pin))
        {
          byte |= 0x80;
        }
    }

  return byte;
}

/****************************************************************************
 * Name: maia_onewire_crc8
 *
 * Description:
 *   Calculate Dallas/Maxim CRC8 checksum.
 *   Polynomial: X^8 + X^5 + X^4 + 1 (0x8C)
 *
 ****************************************************************************/

uint8_t maia_onewire_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;

  for (uint8_t i = 0; i < len; i++)
    {
      uint8_t byte = data[i];
      for (uint8_t j = 0; j < 8; j++)
        {
          uint8_t mix = (crc ^ byte) & 0x01;
          crc >>= 1;
          if (mix)
            {
              crc ^= OW_CRC8_POLY;
            }
          byte >>= 1;
        }
    }

  return crc;
}