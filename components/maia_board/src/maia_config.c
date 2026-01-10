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
 * components/maia_board/src/maia_config.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * General configuration logging
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

#define TAG "[MAIA_CONFIG]"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_config_log
 *
 * Description:
 *   Log all general configuration parameters at startup.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MAIA_LOG_GENERAL_CONFIG
void maia_config_log(void)
{
  ESP_LOGI(TAG, "========== MAIA General Configuration ==========");

  /* Software Version */

  ESP_LOGI(TAG, "Software Version: v%d.%d.%d",
           CONFIG_MAIA_SW_VERSION_MAJOR,
           CONFIG_MAIA_SW_VERSION_MINOR,
           CONFIG_MAIA_SW_VERSION_PATCH);

  /* Timeouts */

  ESP_LOGI(TAG, "Timeout - Sync Data: %d min",
           CONFIG_MAIA_TIMEOUT_SYNC_DATA_MIN);
  ESP_LOGI(TAG, "Timeout - Display Off: %d sec",
           CONFIG_MAIA_TIMEOUT_DISPLAY_OFF_SEC);
  ESP_LOGI(TAG, "Timeout - Screen Change: %d sec",
           CONFIG_MAIA_TIMEOUT_DISPLAY_SCREEN_SEC);

  /* Animal Information */

  ESP_LOGI(TAG, "Animal - Species: %s", CONFIG_MAIA_ANIMAL_SPECIES);
  ESP_LOGI(TAG, "Animal - Breed: %s", CONFIG_MAIA_ANIMAL_BREED);
  ESP_LOGI(TAG, "Animal - Name: %s", CONFIG_MAIA_ANIMAL_NAME);
  ESP_LOGI(TAG, "Animal - Age: %d years", CONFIG_MAIA_ANIMAL_AGE);
  ESP_LOGI(TAG, "Animal - Weight: %d kg", CONFIG_MAIA_ANIMAL_WEIGHT);
  ESP_LOGI(TAG, "Animal - Blood Type: %s", CONFIG_MAIA_ANIMAL_BLOOD_TYPE);
  ESP_LOGI(TAG, "Animal - Comorbidity: %s", CONFIG_MAIA_ANIMAL_COMORBIDITY);
  ESP_LOGI(TAG, "Animal - Allergy: %s", CONFIG_MAIA_ANIMAL_ALLERGY);

  /* Tutor Information */

  ESP_LOGI(TAG, "Tutor - Name: %s %s",
           CONFIG_MAIA_TUTOR_NAME, CONFIG_MAIA_TUTOR_SURNAME);
  ESP_LOGI(TAG, "Tutor - Phone: %s", CONFIG_MAIA_TUTOR_PHONE);
  ESP_LOGI(TAG, "Tutor - Email: %s", CONFIG_MAIA_TUTOR_EMAIL);
  ESP_LOGI(TAG, "Tutor - Social Media: %s", CONFIG_MAIA_TUTOR_SOCIAL_MEDIA);
  ESP_LOGI(TAG, "Tutor - Location: %s, %s, %s",
           CONFIG_MAIA_TUTOR_CITY,
           CONFIG_MAIA_TUTOR_STATE,
           CONFIG_MAIA_TUTOR_COUNTRY);
  ESP_LOGI(TAG, "Tutor - Street: %s", CONFIG_MAIA_TUTOR_STREET);

  ESP_LOGI(TAG, "================================================");
}
#endif