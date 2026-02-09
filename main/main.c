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
 * main/main.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * Application entry point
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_log.h>
#include "maia_board.h"

#ifdef CONFIG_MAIA_TEST_ENABLE
#  include "tests/tests.h"
#else
#  include "app.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIN]"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_main
 *
 * Description:
 *   Application entry point.
 *
 ****************************************************************************/

void app_main(void)
{
#ifdef CONFIG_MAIA_TEST_ENABLE

  /* Test mode enabled */

  ESP_LOGI(TAG, "=== MAIA MODE: TEST MODE ===");
  maia_board_init();

#ifdef CONFIG_MAIA_TEST_BLINK
  test_blink_run();
#elif defined(CONFIG_MAIA_TEST_BUTTON)
  test_button_run();
#elif defined(CONFIG_MAIA_TEST_TEMPERATURE_SENSOR)
  test_ds18b20_run();
#elif defined(CONFIG_MAIA_TEST_HAPTIC_MOTOR)
  test_drv2605l_run();
#elif defined(CONFIG_MAIA_TEST_DISPLAY)
  test_ssd1306_run();
#endif

#else

  /* Normal application mode */

  ESP_LOGI(TAG, "=== MAIA MODE: REAL APPLICATION ===");
  maia_board_init();
  app_init();

#endif
}
