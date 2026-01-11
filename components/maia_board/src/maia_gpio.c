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
 * components/maia_board/src/maia_gpio.c
 *
 * MAIA - Motion Assistance for Impaired Animals
 * GPIO initialization for interrupts, button, and ToF control pins
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_board.h"
#include <driver/gpio.h>
#include <esp_log.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIA_GPIO]"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_gpio_init
 *
 * Description:
 *   Initialize GPIO pins:
 *   - IMU (accel) INT pin  (input, internal pull-up)
 *   - ToF LPn pins (output, active low enable)
 *   - ToF INT pins (input, internal pull-up)
 *   - Button  pin  (input, external pull-up)
 *   - Status LED   (output)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_gpio_init(void)
{
  esp_err_t ret;
  gpio_config_t io_conf;

  ESP_LOGI(TAG, "Initializing GPIO pins");

  /*
   * LIDAR: LOW_POWER_ENABLE_PIN
   * * * * * * * * * * * * * * * */

#ifdef CONFIG_MAIA_VL53L5CX_ENABLE
   /*
    Bit 0  = GPIO00
    Bit 1  = GPIO01
    Bit 2  = GPIO02
    ...
    Bit 43 = GPIO43
   
    0000 0000 0000 0000 0000 0000 0000 0010
                                         ^
                                     GPIO1 ---> MAIA_GPIO_TOF1_LPN

    0000 0000 0000 0000 0000 0000 0000 0100
                                        ^
                                     GPIO2 ---> MAIA_GPIO_TOF2_LPN

    We have to set both GPIO as 1
  */

  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_TOF1_LPN) |
                         (1ULL << MAIA_GPIO_TOF2_LPN);

  io_conf.intr_type = GPIO_INTR_DISABLE;           /* disabling interrupt    */
  io_conf.mode = GPIO_MODE_OUTPUT;                 /* setting as output      */
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    /* we have external 47k   */
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;        /* ensure no conlicts     */

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure ToF LPn pins");
      return ret;
    }

  gpio_set_level(MAIA_GPIO_TOF1_LPN, 1);       /* Start with Lidar 1 disabled */
  gpio_set_level(MAIA_GPIO_TOF2_LPN, 1);       /* Start with Lidar 2 disabled */

  /*
   * LIDAR: INTERRUPT_PIN
   * * * * * * * * * * * * * * * */
  
   /* 
     Input with internal pull-up
     INT is active low, open-drain output from sensor
   */

  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_TOF1_INT) |
                         (1ULL << MAIA_GPIO_TOF2_INT);

  io_conf.intr_type = GPIO_INTR_NEGEDGE;           /* trigger on fall        */
  io_conf.mode = GPIO_MODE_INPUT;                  /* set as input           */
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    /* disable pulldown       */
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;         /* enable internal pullup */

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure ToF INT pins");
      return ret;
    }
#endif

  /*
   * Accelerometer: Interrupt_PIN
   * * * * * * * * * * * * * * * */

#ifdef CONFIG_MAIA_MPU6050_ENABLE
  /*
   Input with internal pull-up
   MPU6050 INT is configurable, default active high
  */

  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_IMU_INT);
  io_conf.intr_type = GPIO_INTR_POSEDGE;           /* trigger on rise        */
  io_conf.mode = GPIO_MODE_INPUT;                  /* set as input           */
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    /* disable pulldown       */
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;         /* enable internal pullup */

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure IMU INT pin");
      return ret;
    }
#endif

   /*
   * Button: Interrupt_PIN
   * * * * * * * * * * * * * * * */

  /* Button pin - Input, external pull-up provided
   * Active low (pressed = LOW)
   */

  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_BUTTON);
  io_conf.intr_type = GPIO_INTR_NEGEDGE;              /* trigger on fall      */
  io_conf.mode = GPIO_MODE_INPUT;                     /* set as input         */
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;       /* disable pulldown     */
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;           /* use external pull-up */

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure button pin");
      return ret;
    }

  /*
   * Status LED
   * * * * * * * * * * * * * * * */

  /* Output pin for status indication */

  io_conf.pin_bit_mask = (1ULL << MAIA_GPIO_LED_STATUS);
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to configure status LED pin");
      return ret;
    }

  gpio_set_level(MAIA_GPIO_LED_STATUS, 0);         /* Start with LED off */

  /*
   * Install GPIO ISR service (global, required for all interrupts)
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  /* This service is shared by all GPIO interrupts in the system
   * Must be called once before any gpio_isr_handler_add()
   * ESP_ERR_INVALID_STATE returned if already installed (safe to ignore)
   */

  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
      ESP_LOGE(TAG, "Failed to install GPIO ISR service");
      return ret;
    }

  ESP_LOGI(TAG, "GPIO pins initialized successfully");

  return ESP_OK;
}