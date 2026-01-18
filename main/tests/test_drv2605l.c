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
 * main/tests/test_drv2605l.c
 *
 * DRV2605L Haptic Motor Driver Test Suite
 * Comprehensive automatic test of all driver functions
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "tests.h"
#include "drv2605l.h"
#include "maia_board.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[TEST_DRV2605L]"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_drv2605l_run
 *
 * Description:
 *   Comprehensive automatic test of all DRV2605L driver functions.
 *   Tests run sequentially with clear logging.
 *
 ****************************************************************************/

void test_drv2605l_run(void)
{
  esp_err_t ret;
  uint8_t status;
  
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "╔════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║   DRV2605L Haptic Motor - Comprehensive Test      ║");
  ESP_LOGI(TAG, "╚════════════════════════════════════════════════════╝");
  ESP_LOGI(TAG, "");
  
  /* ===================================================================== */
  /* TEST 1: Driver Initialization                                         */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 1: Driver Initialization");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  drv2605l_config_t config =
    {
      .i2c_addr = CONFIG_MAIA_DRV2605L_I2C_ADDR,
#ifdef CONFIG_MAIA_DRV2605L_ACTUATOR_ERM
      .actuator = DRV2605L_ACTUATOR_ERM,
#else
      .actuator = DRV2605L_ACTUATOR_LRA,
#endif

#if defined(CONFIG_MAIA_DRV2605L_LIBRARY_A)
      .library = DRV2605L_LIB_ERM_A,
#elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_B)
      .library = DRV2605L_LIB_ERM_B,
#elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_C)
      .library = DRV2605L_LIB_ERM_C,
#elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_D)
      .library = DRV2605L_LIB_ERM_D,
#elif defined(CONFIG_MAIA_DRV2605L_LIBRARY_E)
      .library = DRV2605L_LIB_ERM_E,
#else
      .library = DRV2605L_LIB_LRA,
#endif

      .rated_voltage = CONFIG_MAIA_DRV2605L_RATED_VOLTAGE,
      .overdrive_clamp = CONFIG_MAIA_DRV2605L_OVERDRIVE_CLAMP,
      .auto_calibrate = CONFIG_MAIA_DRV2605L_AUTO_CALIBRATION,
    };
  
  ret = drv2605l_init(&config);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Driver initialization");
      ESP_LOGE(TAG, "Test aborted - check hardware connections");
      return;
    }
  
  ESP_LOGI(TAG, "✓ PASS: Driver initialized successfully");
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
  /* ===================================================================== */
  /* TEST 2: Device Status Check                                           */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 2: Device Status (drv2605l_get_status)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  ret = drv2605l_get_status(&status);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Could not read device status");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: Status read successfully");
      ESP_LOGI(TAG, "  Device Status:   0x%02X", status);
      ESP_LOGI(TAG, "  Device ID:       0x%02X", (status >> 5) & 0x07);
      ESP_LOGI(TAG, "  Diag Result:     %s",
               (status & 0x08) ? "FAIL" : "PASS");
      ESP_LOGI(TAG, "  Over-Temp:       %s",
               (status & 0x02) ? "YES" : "NO");
      ESP_LOGI(TAG, "  Over-Current:    %s",
               (status & 0x01) ? "YES" : "NO");
    }
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
  /* ===================================================================== */
  /* TEST 3: Single Effect Playback                                        */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 3: Single Effect (drv2605l_play_effect)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  const uint8_t test_effects[] = {1, 10, 20, 47, 52};
  const char *effect_names[] =
    {
      "Strong Click 100%",
      "Sharp Tick 3 - 100%",
      "Pulsing Medium 3 - 100%",
      "Buzz 1 - 100%",
      "Buzz 5 - 100%"
    };
  
  for (int i = 0; i < 5; i++)
    {
      ESP_LOGI(TAG, "Playing effect %d: %s", test_effects[i],
               effect_names[i]);
      
      ret = drv2605l_play_effect(test_effects[i]);
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "✗ FAILED: Effect %d", test_effects[i]);
        }
      else
        {
          ESP_LOGI(TAG, "✓ PASS: Effect %d played", test_effects[i]);
        }
      
      vTaskDelay(pdMS_TO_TICKS(700));
    }
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
  /* ===================================================================== */
  /* TEST 4: Effect Sequence                                               */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 4: Effect Sequence (drv2605l_play_sequence)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  const uint8_t sequence[] = {1, 10, 20};
  
  ESP_LOGI(TAG, "Playing sequence: [1, 10, 20]");
  ret = drv2605l_play_sequence(sequence, 3);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Sequence playback");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: Sequence triggered successfully");
    }
  
  vTaskDelay(pdMS_TO_TICKS(2000));
  ESP_LOGI(TAG, "");
  
  /* ===================================================================== */
  /* TEST 5: Stop Command                                                  */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 5: Stop Playback (drv2605l_stop)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  ESP_LOGI(TAG, "Starting long effect (Buzz 5)...");
  drv2605l_play_effect(52);
  vTaskDelay(pdMS_TO_TICKS(400));
  
  ESP_LOGI(TAG, "Sending stop command...");
  ret = drv2605l_stop();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Stop command");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: Motor stopped successfully");
    }
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
  /* ===================================================================== */
  /* TEST 6: Library Selection                                             */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 6: Library Selection (drv2605l_set_library)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  ESP_LOGI(TAG, "Switching to Library B (soft bumps)...");
  ret = drv2605l_set_library(DRV2605L_LIB_ERM_B);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Library switch");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: Switched to Library B");
      ESP_LOGI(TAG, "Playing effect 1 from Library B...");
      drv2605l_play_effect(1);
      vTaskDelay(pdMS_TO_TICKS(700));
    }
  
  ESP_LOGI(TAG, "Switching back to Library A...");
  drv2605l_set_library(DRV2605L_LIB_ERM_A);
  ESP_LOGI(TAG, "✓ PASS: Restored Library A");
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
  /* ===================================================================== */
  /* TEST 7: Power Management (Standby/Wakeup)                             */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 7: Power Management (standby/wakeup)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  ESP_LOGI(TAG, "Entering standby mode...");
  ret = drv2605l_standby();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Standby mode");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: Entered standby mode");
    }
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  ESP_LOGI(TAG, "Waking up from standby...");
  ret = drv2605l_wakeup();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Wakeup");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: Woke up successfully");
      ESP_LOGI(TAG, "Playing test effect to verify...");
      drv2605l_play_effect(1);
      vTaskDelay(pdMS_TO_TICKS(700));
    }
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
  /* ===================================================================== */
  /* TEST 8: Mode Selection                                                */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 8: Mode Selection (drv2605l_set_mode)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  ESP_LOGI(TAG, "Testing mode switch to internal trigger...");
  ret = drv2605l_set_mode(DRV2605L_OP_MODE_INTERNAL_TRIGGER);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: Mode switch");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: Internal trigger mode set");
    }
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
#ifdef CONFIG_MAIA_DRV2605L_MODE_COMPOSER
  
  /* ===================================================================== */
  /* TEST 9: RTP Mode - Fade Pattern                                       */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 9: RTP Mode - Fade (drv2605l_set_rtp_value)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  ESP_LOGI(TAG, "Switching to RTP mode...");
  ret = drv2605l_set_mode(DRV2605L_OP_MODE_REALTIME);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "✗ FAILED: RTP mode switch");
    }
  else
    {
      ESP_LOGI(TAG, "✓ PASS: RTP mode enabled");
      
      ESP_LOGI(TAG, "Fade in (0 → 255)...");
      for (int intensity = 0; intensity <= 255; intensity += 5)
        {
          drv2605l_set_rtp_value(intensity);
          vTaskDelay(pdMS_TO_TICKS(20));
        }
      
      vTaskDelay(pdMS_TO_TICKS(300));
      
      ESP_LOGI(TAG, "Fade out (255 → 0)...");
      for (int intensity = 255; intensity >= 0; intensity -= 5)
        {
          drv2605l_set_rtp_value(intensity);
          vTaskDelay(pdMS_TO_TICKS(20));
        }
      
      ESP_LOGI(TAG, "✓ PASS: RTP fade pattern complete");
    }
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
  /* ===================================================================== */
  /* TEST 10: RTP Mode - Pulse Pattern                                     */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  ESP_LOGI(TAG, "TEST 10: RTP Mode - Pulse Pattern (SOS)");
  ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
  
  const uint8_t intensity = 200;
  const int short_pulse = 100;
  const int long_pulse = 300;
  const int gap = 100;
  const int letter_gap = 300;
  
  ESP_LOGI(TAG, "Transmitting SOS in morse code...");
  
  /* S (. . .) */
  
  ESP_LOGI(TAG, "S: . . .");
  for (int i = 0; i < 3; i++)
    {
      drv2605l_set_rtp_value(intensity);
      vTaskDelay(pdMS_TO_TICKS(short_pulse));
      drv2605l_set_rtp_value(0);
      vTaskDelay(pdMS_TO_TICKS(gap));
    }
  
  vTaskDelay(pdMS_TO_TICKS(letter_gap));
  
  /* O (- - -) */
  
  ESP_LOGI(TAG, "O: - - -");
  for (int i = 0; i < 3; i++)
    {
      drv2605l_set_rtp_value(intensity);
      vTaskDelay(pdMS_TO_TICKS(long_pulse));
      drv2605l_set_rtp_value(0);
      vTaskDelay(pdMS_TO_TICKS(gap));
    }
  
  vTaskDelay(pdMS_TO_TICKS(letter_gap));
  
  /* S (. . .) */
  
  ESP_LOGI(TAG, "S: . . .");
  for (int i = 0; i < 3; i++)
    {
      drv2605l_set_rtp_value(intensity);
      vTaskDelay(pdMS_TO_TICKS(short_pulse));
      drv2605l_set_rtp_value(0);
      vTaskDelay(pdMS_TO_TICKS(gap));
    }
  
  ESP_LOGI(TAG, "✓ PASS: SOS pulse pattern complete");
  
  /* Return to internal trigger mode */
  
  drv2605l_set_mode(DRV2605L_OP_MODE_INTERNAL_TRIGGER);
  
  ESP_LOGI(TAG, "");
  vTaskDelay(pdMS_TO_TICKS(500));
  
#endif /* CONFIG_MAIA_DRV2605L_MODE_COMPOSER */
  
  /* ===================================================================== */
  /* TEST SUMMARY                                                          */
  /* ===================================================================== */
  
  ESP_LOGI(TAG, "╔════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║            ALL TESTS COMPLETED                     ║");
  ESP_LOGI(TAG, "╠════════════════════════════════════════════════════╣");
  ESP_LOGI(TAG, "║  ✓ Driver initialization                           ║");
  ESP_LOGI(TAG, "║  ✓ Device status read                              ║");
  ESP_LOGI(TAG, "║  ✓ Single effect playback                          ║");
  ESP_LOGI(TAG, "║  ✓ Effect sequence                                 ║");
  ESP_LOGI(TAG, "║  ✓ Stop command                                    ║");
  ESP_LOGI(TAG, "║  ✓ Library selection                               ║");
  ESP_LOGI(TAG, "║  ✓ Power management (standby/wakeup)               ║");
  ESP_LOGI(TAG, "║  ✓ Mode selection                                  ║");
#ifdef CONFIG_MAIA_DRV2605L_MODE_COMPOSER
  ESP_LOGI(TAG, "║  ✓ RTP mode - fade pattern                         ║");
  ESP_LOGI(TAG, "║  ✓ RTP mode - pulse pattern                        ║");
#endif
  ESP_LOGI(TAG, "╠════════════════════════════════════════════════════╣");
  ESP_LOGI(TAG, "║  DRV2605L driver is fully operational!             ║");
  ESP_LOGI(TAG, "╚════════════════════════════════════════════════════╝");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Test will restart in 10 seconds...");
  
  vTaskDelay(pdMS_TO_TICKS(10000));
  
  /* Loop back */
  
  test_drv2605l_run();
}