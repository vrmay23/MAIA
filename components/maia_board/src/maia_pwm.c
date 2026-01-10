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
 * components/maia_board/src/maia_pwm.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * PWM (LEDC) initialization for ERM motor control
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

#define TAG "[MAIA_PWM]"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_pwm_init
 *
 * Description:
 *   Initialize LEDC PWM for motor control.
 *   - Timer Left:  Motor Left frequency, 8-bit resolution (0-255 duty)
 *   - Timer Right: Motor Right frequency, 8-bit resolution (0-255 duty)
 *   - Channel 0: Motor Left (GPIO7)
 *   - Channel 1: Motor Right (GPIO8)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_pwm_init(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Initializing PWM (freq=%dHz, res=%d-bit)",
           MAIA_PWM_FREQ_MOTOR_LEFT, MAIA_PWM_RESOLUTION);

  /* Configure LEDC timer for Motor Left */

  ledc_timer_config_t timer_left = {
      .speed_mode = MAIA_PWM_MODE,
      .duty_resolution = MAIA_PWM_RESOLUTION,
      .timer_num = MAIA_PWM_TIMER_LEFT,
      .freq_hz = MAIA_PWM_FREQ_MOTOR_LEFT,
      .clk_cfg = LEDC_AUTO_CLK,
  };

  ret = ledc_timer_config(&timer_left);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure LEDC timer left: %s",
               esp_err_to_name(ret));
      return ret;
    }

  /* Configure LEDC timer for Motor Right */

  ledc_timer_config_t timer_right = {
      .speed_mode = MAIA_PWM_MODE,
      .duty_resolution = MAIA_PWM_RESOLUTION,
      .timer_num = MAIA_PWM_TIMER_RIGHT,
      .freq_hz = MAIA_PWM_FREQ_MOTOR_RIGHT,
      .clk_cfg = LEDC_AUTO_CLK,
  };

  ret = ledc_timer_config(&timer_right);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure LEDC timer right: %s",
               esp_err_to_name(ret));
      return ret;
    }

  /* Configure Motor Left channel */

  ledc_channel_config_t ch_left = {
      .gpio_num = MAIA_GPIO_MOTOR_LEFT,
      .speed_mode = MAIA_PWM_MODE,
      .channel = MAIA_PWM_CH_MOTOR_LEFT,
      .timer_sel = MAIA_PWM_TIMER_LEFT,
      .duty = 0,
      .hpoint = 0,
  };

  ret = ledc_channel_config(&ch_left);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure motor left channel: %s",
               esp_err_to_name(ret));
      return ret;
    }

  /* Configure Motor Right channel */

  ledc_channel_config_t ch_right = {
      .gpio_num = MAIA_GPIO_MOTOR_RIGHT,
      .speed_mode = MAIA_PWM_MODE,
      .channel = MAIA_PWM_CH_MOTOR_RIGHT,
      .timer_sel = MAIA_PWM_TIMER_RIGHT,
      .duty = 0,
      .hpoint = 0,
  };

  ret = ledc_channel_config(&ch_right);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure motor right channel: %s",
               esp_err_to_name(ret));
      return ret;
    }

  ESP_LOGI(TAG, "PWM initialized (Left=GPIO%d, Right=GPIO%d)",
           MAIA_GPIO_MOTOR_LEFT, MAIA_GPIO_MOTOR_RIGHT);

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_pwm_set_duty
 *
 * Description:
 *   Set PWM duty cycle for a motor channel.
 *
 * Input Parameters:
 *   channel - LEDC channel (MAIA_PWM_CH_MOTOR_LEFT or _RIGHT)
 *   duty    - Duty cycle value (0-255 for 8-bit resolution)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_pwm_set_duty(ledc_channel_t channel, uint8_t duty)
{
  esp_err_t ret;

  ret = ledc_set_duty(MAIA_PWM_MODE, channel, duty);
  if (ret != ESP_OK)
    {
      return ret;
    }

  ret = ledc_update_duty(MAIA_PWM_MODE, channel);

  return ret;
}