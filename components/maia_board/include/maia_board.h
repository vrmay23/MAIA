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
     *
     ****************************************************************************/

    #ifndef __COMPONENTS_MAIA_BOARD_INCLUDE_MAIA_BOARD_H
    #define __COMPONENTS_MAIA_BOARD_INCLUDE_MAIA_BOARD_H

    /****************************************************************************
     * Included Files
     ****************************************************************************/

    #include <esp_err.h>
    #include <driver/gpio.h>
    #include <driver/i2c_master.h>
    #include <driver/ledc.h>

    /****************************************************************************
     * Pre-processor Definitions
     ****************************************************************************/

    /* GPIO Pin Definitions (MAIA rev0 - ESP32-S3-WROOM-1U-N16R8)
    *
    *  ------------------------------------------------------------------
    * | Mod pin |  GPIO  | Net                | Function                 |
    *  ---------|--------|--------------------|--------------------------|
    *      1    |   -    | GND                |
    *      2    |   -    | 3v3_esp            | 3V3 supply
    *      3    |   -    | enable_esp         | EN / reset
    *      4    | GPIO04 | MPU_int            | IMU interrupt
    *      5    | GPIO05 | I2C_SDA            | I2C bus SDA
    *      6    | GPIO06 | I2C_SCL            | I2C bus SCL
    *      7    | GPIO07 | Motor_Left_PWM     | Left ERM motor PWM
    *      8    | GPIO15 | CAM_Y2             | Camera data
    *      9    | GPIO16 | CAM_Y5             | Camera data
    *     10    | GPIO17 | CAM_Y3             | Camera data
    *     11    | GPIO18 | CAM_Y4             | Camera data
    *     12    | GPIO08 | Motor_Right_PWM    | Right ERM motor PWM
    *     13    | GPIO19 | usb_N              | Native USB D-
    *     14    | GPIO20 | usb_P              | Native USB D+
    *     15    | GPIO03 | DS18B20_DATA       | OneWire data
    *     16    | GPIO46 | -                  | Not connected (DNU)
    *     17    | GPIO09 | BTN_01             | Button 1
    *     18    | GPIO10 | CAM_XCLK           | Camera clock in
    *     19    | GPIO11 | CAM_Y8             | Camera data
    *     20    | GPIO12 | CAM_Y7             | Camera data
    *     21    | GPIO13 | CAM_PCLK           | Camera pixel clock
    *     22    | GPIO14 | CAM_PWDN           | Camera power down
    *     23    | GPIO21 | CAM_Y9             | Camera data
    *     24    | GPIO47 | CAM_HREF           | Camera HREF
    *     25    | GPIO48 | USER_LED           | User / status LED
    *     26    | GPIO45 | CAM_Y6             | Camera data
    *     27    | GPIO00 | boot_esp           | Boot strapping pin
    *     28    | GPIO35 | -                  | Not connected (DNU)
    *     29    | GPIO36 | -                  | Not connected (DNU)
    *     30    | GPIO37 | -                  | Not connected (DNU)
    *     31    | GPIO38 | CAM_VSYNC          | Camera VSYNC
    *     32    | GPIO39 | SCCB_SCL           | Camera SCCB clock
    *     33    | GPIO40 | SCCB_SDA           | Camera SCCB data
    *     34    | GPIO41 | VL53L5CX_INT_RIGHT | ToF right INT
    *     35    | GPIO42 | VL53L5CX_INT_LEFT  | ToF left INT
    *     36    | GPIO44 | LED_FLASH          | Camera flash LED  (U0RXD)
    *     37    | GPIO43 | BTN_02             | Button 2          (U0TXD)
    *     38    | GPIO02 | VL53L5CX_LP_RIGHT  | ToF right LPn
    *     39    | GPIO01 | VL53L5CX_LP_LEFT   | ToF left LPn
    *     40    |   -    | GND                |
    *     41    |   -    | GND                |
    *  ------------------------------------------------------------------
    *
    * NOTE: GPIO43/GPIO44 are the ROM/UART0 console pins (U0TXD/U0RXD).
    *       They are only free for general purpose use while the IDF
    *       console is routed to the USB Serial/JTAG controller
    *       (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG). The first stage ROM
    *       bootloader still drives U0TXD briefly on every reset.
    */

    /* ToF Sensor Pins (VL53L5CX) - TOF1 = LEFT, TOF2 = RIGHT */

    #define MAIA_GPIO_TOF1_LPN          1      /* low_power_enable - left  */
    #define MAIA_GPIO_TOF1_INT          42     /* interrupt_pin    - left  */
    #define MAIA_GPIO_TOF2_LPN          2      /* low_power_enable - right */
    #define MAIA_GPIO_TOF2_INT          41     /* interrupt_pin    - right */

    /* Temperature Sensor (1-Wire) */

    #define MAIA_GPIO_ONEWIRE           3

    /* IMU Interrupt */

    #define MAIA_GPIO_IMU_INT           4

    /* PWM Motors (ERM Left and Right) */

    #define MAIA_PWM_FREQ_MOTOR_LEFT    CONFIG_MAIA_PWM_FREQ_MOTOR_LEFT
    #define MAIA_PWM_FREQ_MOTOR_RIGHT   CONFIG_MAIA_PWM_FREQ_MOTOR_RIGHT
    #define MAIA_GPIO_MOTOR_LEFT        7
    #define MAIA_GPIO_MOTOR_RIGHT       8
    #define MAIA_PWM_MODE               LEDC_LOW_SPEED_MODE
    #define MAIA_PWM_RESOLUTION         LEDC_TIMER_8_BIT
    #define MAIA_PWM_CH_MOTOR_LEFT      LEDC_CHANNEL_0
    #define MAIA_PWM_CH_MOTOR_RIGHT     LEDC_CHANNEL_1
    #define MAIA_PWM_TIMER_LEFT         LEDC_TIMER_0
    #define MAIA_PWM_TIMER_RIGHT        LEDC_TIMER_1

    /* Buttons */

    #define MAIA_GPIO_BUTTON            9      /* BTN_01                   */
    #define MAIA_GPIO_BUTTON_2          43     /* BTN_02 (U0TXD)           */

    /* LEDs */

    #define MAIA_GPIO_LED_USER          48     /* USER_LED                 */
    #define MAIA_GPIO_LED_FLASH         44     /* LED_FLASH (U0RXD)        */
    #define MAIA_GPIO_LED_STATUS        CONFIG_MAIA_LED_STATUS_PIN

    /* Camera (OV2640) - no driver yet, pinout for reference */

    #define MAIA_GPIO_CAM_XCLK          10
    #define MAIA_GPIO_CAM_PCLK          13
    #define MAIA_GPIO_CAM_VSYNC         38
    #define MAIA_GPIO_CAM_HREF          47
    #define MAIA_GPIO_CAM_PWDN          14
    #define MAIA_GPIO_CAM_SCCB_SDA      40
    #define MAIA_GPIO_CAM_SCCB_SCL      39
    #define MAIA_GPIO_CAM_Y2            15
    #define MAIA_GPIO_CAM_Y3            17
    #define MAIA_GPIO_CAM_Y4            18
    #define MAIA_GPIO_CAM_Y5            16
    #define MAIA_GPIO_CAM_Y6            45
    #define MAIA_GPIO_CAM_Y7            12
    #define MAIA_GPIO_CAM_Y8            11
    #define MAIA_GPIO_CAM_Y9            21

    /* I2C Bus */

    #define MAIA_I2C_FREQ_HZ            CONFIG_MAIA_I2C_FREQ_HZ
    #define MAIA_GPIO_I2C_SDA           5
    #define MAIA_GPIO_I2C_SCL           6
    #define MAIA_I2C_PORT               I2C_NUM_0

    /* I2C Device Addresses */

    #define MAIA_I2C_ADDR_SSD1306       CONFIG_MAIA_SSD1306_I2C_ADDR
    #define MAIA_I2C_ADDR_TOF1          CONFIG_MAIA_VL53L5CX_LEFT_I2C_ADDR
    #define MAIA_I2C_ADDR_TOF2          CONFIG_MAIA_VL53L5CX_RIGHT_I2C_ADDR
    #define MAIA_I2C_ADDR_DRV2605L      CONFIG_MAIA_DRV2605L_I2C_ADDR

    /* IMU address - MAIA_IMU_DEVICE is a Kconfig choice, so exactly one
     * of these is defined at a time. MAIA_I2C_ADDR_IMU is the device
     * agnostic alias for code that does not care which IMU is fitted.
     */

    #ifdef CONFIG_MAIA_IMU_MPU6050
    #  define MAIA_I2C_ADDR_MPU6050     CONFIG_MAIA_MPU6050_I2C_ADDR
    #  define MAIA_I2C_ADDR_IMU         MAIA_I2C_ADDR_MPU6050
    #endif

    #ifdef CONFIG_MAIA_IMU_ADXL345
    #  define MAIA_I2C_ADDR_ADXL345     CONFIG_MAIA_ADXL345_I2C_ADDR
    #  define MAIA_I2C_ADDR_IMU         MAIA_I2C_ADDR_ADXL345
    #endif

    #ifdef CONFIG_MAIA_IMU_LSM6DSOX
    #  define MAIA_I2C_ADDR_LSM6DSOX    CONFIG_MAIA_LSM6DSOX_I2C_ADDR
    #  define MAIA_I2C_ADDR_IMU         MAIA_I2C_ADDR_LSM6DSOX
    #endif

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
    void maia_config_log(void);
    #endif

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
     *   Initialize OneWire GPIO pin for DS18B20 temperature sensor.
     *
     * Input Parameters:
     *   None
     *
     * Returned Value:
     *   ESP_OK on success; ESP_FAIL on failure.
     *
     ****************************************************************************/

    esp_err_t maia_onewire_init(void);

    /****************************************************************************
     * Name: maia_onewire_reset
     *
     * Description:
     *   Send reset pulse on OneWire bus and check for presence.
     *
     * Input Parameters:
     *   pin - GPIO pin number
     *
     * Returned Value:
     *   true if device present; false otherwise
     *
     ****************************************************************************/

    bool maia_onewire_reset(gpio_num_t pin);

    /****************************************************************************
     * Name: maia_onewire_write_bit
     *
     * Description:
     *   Write single bit on OneWire bus.
     *
     * Input Parameters:
     *   pin - GPIO pin number
     *   bit - Bit value (0 or 1)
     *
     * Returned Value:
     *   None
     *
     ****************************************************************************/

    void maia_onewire_write_bit(gpio_num_t pin, uint8_t bit);

    /****************************************************************************
     * Name: maia_onewire_read_bit
     *
     * Description:
     *   Read single bit from OneWire bus.
     *
     * Input Parameters:
     *   pin - GPIO pin number
     *
     * Returned Value:
     *   Bit value (0 or 1)
     *
     ****************************************************************************/

    uint8_t maia_onewire_read_bit(gpio_num_t pin);

    /****************************************************************************
     * Name: maia_onewire_write_byte
     *
     * Description:
     *   Write byte on OneWire bus (LSB first).
     *
     * Input Parameters:
     *   pin  - GPIO pin number
     *   byte - Byte to write
     *
     * Returned Value:
     *   None
     *
     ****************************************************************************/

    void maia_onewire_write_byte(gpio_num_t pin, uint8_t byte);

    /****************************************************************************
     * Name: maia_onewire_read_byte
     *
     * Description:
     *   Read byte from OneWire bus (LSB first).
     *
     * Input Parameters:
     *   pin - GPIO pin number
     *
     * Returned Value:
     *   Byte read from bus
     *
     ****************************************************************************/

    uint8_t maia_onewire_read_byte(gpio_num_t pin);

    /****************************************************************************
     * Name: maia_onewire_crc8
     *
     * Description:
     *   Calculate Dallas/Maxim CRC8 checksum for OneWire data.
     *
     * Input Parameters:
     *   data - Data buffer
     *   len  - Data length in bytes
     *
     * Returned Value:
     *   CRC8 checksum
     *
     ****************************************************************************/

    uint8_t maia_onewire_crc8(const uint8_t *data, uint8_t len);

    /****************************************************************************
     * Name: maia_led_init
     *
     * Description:
     *   Initialize status LED GPIO.
     *
     * Input Parameters:
     *   None
     *
     * Returned Value:
     *   ESP_OK on success; ESP_FAIL on failure.
     *
     ****************************************************************************/

    esp_err_t maia_led_init(void);

    /****************************************************************************
     * Name: maia_led_set
     *
     * Description:
     *   Set status LED on or off.
     *
     * Input Parameters:
     *   state - true for ON, false for OFF
     *
     * Returned Value:
     *   ESP_OK on success; ESP_FAIL on failure.
     *
     ****************************************************************************/

    esp_err_t maia_led_set(bool state);

    /****************************************************************************
     * Name: maia_led_toggle
     *
     * Description:
     *   Toggle status LED state.
     *
     * Input Parameters:
     *   None
     *
     * Returned Value:
     *   ESP_OK on success; ESP_FAIL on failure.
     *
     ****************************************************************************/

    esp_err_t maia_led_toggle(void);

    #endif /* __COMPONENTS_MAIA_BOARD_INCLUDE_MAIA_BOARD_H */