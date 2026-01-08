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
 * components/maia_board/include/maia_board.h
 *
 * MAIA - Motion Assistance for Impaired Animals
 * Board Support Package for XIAO ESP32S3 Plus
 *
 ****************************************************************************/

#ifndef __COMPONENTS_MAIA_BOARD_INCLUDE_MAIA_BOARD_H
#define __COMPONENTS_MAIA_BOARD_INCLUDE_MAIA_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Pin Definitions (XIAO ESP32S3 Plus)
 *
 *  -------------------------------------------
 * | XIAO Pin |   GPIO   | PERIPHERAL FUNCTION |
 *  ----------|----------|---------------------
 *      D0    |  GPIO01  |   VL53L5CX #1 LPn
 *      D1    |  GPIO02  |   VL53L5CX #2 LPn
 *      D2    |  GPIO03  |   DS18B20 DATA
 *      D3    |  GPIO04  |   MPU6050 INT
 *      D4    |  GPIO05  |   I2C SDA
 *      D5    |  GPIO06  |   I2C SCL
 *      D6    |  GPIO43  |   VL53L5CX #1 INT
 *      D7    |  GPIO44  |   VL53L5CX #2 INT
 *      D8    |  GPIO07  |   Motor Left PWM
 *      D9    |  GPIO08  |   Motor Right PWM
 *      D10   |  GPIO09  |   Button
 *  |------------------------------------------|
 */

/* ToF Sensor Pins */

#define MAIA_GPIO_TOF1_LPN          1
#define MAIA_GPIO_TOF2_LPN          2
#define MAIA_GPIO_TOF1_INT          43
#define MAIA_GPIO_TOF2_INT          44

/* Temperature Sensor (1-Wire) */

#define MAIA_GPIO_ONEWIRE           3

/* IMU Interrupt */

#define MAIA_GPIO_IMU_INT           4

/* I2C Bus */

#define MAIA_I2C_FREQ_HZ            400000
#define MAIA_GPIO_I2C_SDA           5
#define MAIA_GPIO_I2C_SCL           6

#define MAIA_I2C_PORT               I2C_NUM_0

/* PWM Motors */

#define MAIA_PWM_FREQ_HZ            1000
#define MAIA_GPIO_MOTOR_LEFT        7
#define MAIA_GPIO_MOTOR_RIGHT       8

#define MAIA_PWM_MODE               LEDC_LOW_SPEED_MODE
#define MAIA_PWM_RESOLUTION         LEDC_TIMER_8_BIT
#define MAIA_PWM_CH_MOTOR_LEFT      LEDC_CHANNEL_0
#define MAIA_PWM_CH_MOTOR_RIGHT     LEDC_CHANNEL_1
#define MAIA_PWM_TIMER              LEDC_TIMER_0

/* Button */

#define MAIA_GPIO_BUTTON            9

/* I2C Device Addresses */

#define MAIA_I2C_ADDR_SSD1306       0x3C
#define MAIA_I2C_ADDR_TOF1          0x54
#define MAIA_I2C_ADDR_TOF2          0x52
#define MAIA_I2C_ADDR_DRV2605L      0x5A
#define MAIA_I2C_ADDR_MPU6050       0x68

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: maia_board_init
 *
 * Description:
 *   Initialize all board peripherals (I2C, GPIO, PWM, OneWire).
 *   Must be called before any other board function, such as a bringup
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_board_init(void);

/****************************************************************************
 * Name: maia_i2c_init
 *
 * Description:
 *   Initialize I2C master bus.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_i2c_init(void);

/****************************************************************************
 * Name: maia_i2c_get_bus_handle
 *
 * Description:
 *   Get the I2C master bus handle for device registration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   I2C master bus handle.
 *
 ****************************************************************************/

i2c_master_bus_handle_t maia_i2c_get_bus_handle(void);

/****************************************************************************
 * Name: maia_gpio_init
 *
 * Description:
 *   Initialize GPIO pins (interrupts, button, ToF control pins).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_gpio_init(void);

/****************************************************************************
 * Name: maia_pwm_init
 *
 * Description:
 *   Initialize PWM (via LEDC) for motor control (ERMs).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_pwm_init(void);

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

esp_err_t maia_pwm_set_duty(ledc_channel_t channel, uint8_t duty);

/****************************************************************************
 * Name: maia_onewire_init
 *
 * Description:
 *   Initialize OneWire bus for DS18B20 temperature sensor.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t maia_onewire_init(void);

#endif /* __COMPONENTS_MAIA_BOARD_INCLUDE_MAIA_BOARD_H */
