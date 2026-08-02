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
 * main/tests/test_blink.c
 *
 * MAIA - LED blink test
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include "maia_board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[TEST_BLINK]"
#define LED_PIN CONFIG_MAIA_LED_STATUS_PIN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_blink_run
 *
 * Description:
 *   Run LED blink test. Simple hardware validation.
 *
 ****************************************************************************/

/* TEMPORARY DIAGNOSTIC
 *
 * Dump what is actually in the pin's registers at each step, so the
 * chip tells us whether GPIO<LED_PIN> is really being driven. Revert
 * to the plain reset/set_direction/toggle sequence when done.
 */

static void dump_pin(const char *when)
{
  ESP_LOGI(TAG, "---------- pin state: %s ----------", when);
  gpio_dump_io_configuration(stdout, 1ULL << LED_PIN);
  fflush(stdout);
}

void test_blink_run(void)
{
  int expected;
  int readback;

  ESP_LOGI(TAG, "=== LED Blink Test ===");
  ESP_LOGI(TAG, "Blinking LED on GPIO%d", LED_PIN);

  /* Not the power-on state: maia_board_init() already ran and both
   * maia_gpio_init() and maia_led_init() configured this pin.
   */

  dump_pin("on entry, as left by maia_board_init()");

  gpio_reset_pin(LED_PIN);
  dump_pin("after gpio_reset_pin()");

  /* INPUT_OUTPUT rather than OUTPUT so the pad level can be read back.
   * It still drives push-pull, so the LED behaviour is unchanged.
   */

  gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);
  dump_pin("after gpio_set_direction(INPUT_OUTPUT)");

  gpio_set_level(LED_PIN, 1);
  dump_pin("after first gpio_set_level(1)");

  gpio_set_level(LED_PIN, 0);
  dump_pin("after first gpio_set_level(0)");

  ESP_LOGI(TAG, "---------- entering blink loop ----------");

  expected = 0;

  while (1)
    {
      expected = !expected;
      gpio_set_level(LED_PIN, expected);
      readback = gpio_get_level(LED_PIN);

      ESP_LOGI(TAG, "set %d -> readback %d  %s",
               expected, readback,
               (readback == expected) ? "ok" : "MISMATCH (pad not following)");

      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}