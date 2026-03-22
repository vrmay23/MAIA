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
 * components/drivers/mpu6050/include/mpu6050.h
 *
 * MPU-6050 6-Axis IMU Driver
 * InvenSense MPU-6050 Accelerometer + Gyroscope with DMP
 *
 * Reference: PS-MPU-6000A-00 - MPU-6000/6050 Product Specification
 * Reference: RM-MPU-6000A-00 - MPU-6000/6050 Register Map
 * Reference: AN-MPU-6050_DMP_System_Specification_v3_1.pdf
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_MPU6050_INCLUDE_MPU6050_H
#define __COMPONENTS_DRIVERS_MPU6050_INCLUDE_MPU6050_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Addresses
 * AD0 pin low  (default) -> 0x68
 * AD0 pin high           -> 0x69
 * Register Map Section 9.2
 */

#define MPU6050_I2C_ADDR_DEFAULT    0x68
#define MPU6050_I2C_ADDR_ALT        0x69

/* Device Identity
 * WHO_AM_I register returns 0x68 regardless of AD0 pin state
 */

#define MPU6050_WHO_AM_I_VAL        0x68

/* Register Map
 * Register Map Document, Section 3
 */

#define MPU6050_REG_XG_OFFS_TC      0x00  /* Gyro X offset (TC) */
#define MPU6050_REG_YG_OFFS_TC      0x01  /* Gyro Y offset (TC) */
#define MPU6050_REG_ZG_OFFS_TC      0x02  /* Gyro Z offset (TC) */
#define MPU6050_REG_SELF_TEST_X     0x0D  /* Self-test X */
#define MPU6050_REG_SELF_TEST_Y     0x0E  /* Self-test Y */
#define MPU6050_REG_SELF_TEST_Z     0x0F  /* Self-test Z */
#define MPU6050_REG_SELF_TEST_A     0x10  /* Self-test Accel */
#define MPU6050_REG_XG_OFFS_USRH   0x13  /* Gyro X offset high */
#define MPU6050_REG_XG_OFFS_USRL   0x14  /* Gyro X offset low */
#define MPU6050_REG_YG_OFFS_USRH   0x15  /* Gyro Y offset high */
#define MPU6050_REG_YG_OFFS_USRL   0x16  /* Gyro Y offset low */
#define MPU6050_REG_ZG_OFFS_USRH   0x17  /* Gyro Z offset high */
#define MPU6050_REG_ZG_OFFS_USRL   0x18  /* Gyro Z offset low */
#define MPU6050_REG_SMPLRT_DIV      0x19  /* Sample rate divider */
#define MPU6050_REG_CONFIG          0x1A  /* Configuration */
#define MPU6050_REG_GYRO_CONFIG     0x1B  /* Gyroscope config */
#define MPU6050_REG_ACCEL_CONFIG    0x1C  /* Accelerometer config */
#define MPU6050_REG_FIFO_EN         0x23  /* FIFO enable */
#define MPU6050_REG_INT_PIN_CFG     0x37  /* INT pin config */
#define MPU6050_REG_INT_ENABLE      0x38  /* Interrupt enable */
#define MPU6050_REG_INT_STATUS      0x3A  /* Interrupt status */
#define MPU6050_REG_ACCEL_XOUT_H    0x3B  /* Accel X high byte */
#define MPU6050_REG_ACCEL_XOUT_L    0x3C  /* Accel X low byte */
#define MPU6050_REG_ACCEL_YOUT_H    0x3D  /* Accel Y high byte */
#define MPU6050_REG_ACCEL_YOUT_L    0x3E  /* Accel Y low byte */
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F  /* Accel Z high byte */
#define MPU6050_REG_ACCEL_ZOUT_L    0x40  /* Accel Z low byte */
#define MPU6050_REG_TEMP_OUT_H      0x41  /* Temperature high byte */
#define MPU6050_REG_TEMP_OUT_L      0x42  /* Temperature low byte */
#define MPU6050_REG_GYRO_XOUT_H     0x43  /* Gyro X high byte */
#define MPU6050_REG_GYRO_XOUT_L     0x44  /* Gyro X low byte */
#define MPU6050_REG_GYRO_YOUT_H     0x45  /* Gyro Y high byte */
#define MPU6050_REG_GYRO_YOUT_L     0x46  /* Gyro Y low byte */
#define MPU6050_REG_GYRO_ZOUT_H     0x47  /* Gyro Z high byte */
#define MPU6050_REG_GYRO_ZOUT_L     0x48  /* Gyro Z low byte */
#define MPU6050_REG_USER_CTRL       0x6A  /* User control */
#define MPU6050_REG_PWR_MGMT_1      0x6B  /* Power management 1 */
#define MPU6050_REG_PWR_MGMT_2      0x6C  /* Power management 2 */
#define MPU6050_REG_FIFO_COUNTH     0x72  /* FIFO count high */
#define MPU6050_REG_FIFO_COUNTL     0x73  /* FIFO count low */
#define MPU6050_REG_FIFO_R_W        0x74  /* FIFO read/write */
#define MPU6050_REG_WHO_AM_I        0x75  /* Device identity */

/* DMP Registers
 * DMP System Specification v3.1
 */

#define MPU6050_REG_DMP_CFG_1       0x70  /* DMP config 1 */
#define MPU6050_REG_DMP_CFG_2       0x71  /* DMP config 2 */
#define MPU6050_DMP_MEMORY_BANK     0x6D  /* DMP memory bank select */
#define MPU6050_DMP_MEMORY_START    0x6E  /* DMP memory start address */
#define MPU6050_DMP_MEMORY_R_W      0x6F  /* DMP memory read/write */

/* PWR_MGMT_1 bits (0x6B)
 * Register Map Section 4.28
 */

#define MPU6050_PWR1_DEVICE_RESET   0x80  /* Reset all registers */
#define MPU6050_PWR1_SLEEP          0x40  /* Sleep mode */
#define MPU6050_PWR1_CYCLE          0x20  /* Cycle between sleep/wake */
#define MPU6050_PWR1_TEMP_DIS       0x08  /* Disable temperature sensor */
#define MPU6050_PWR1_CLKSEL_XGYRO  0x01  /* PLL with X-axis gyro ref */

/* INT_PIN_CFG bits (0x37)
 * Register Map Section 4.7
 */

#define MPU6050_INT_CFG_ACTIVE_LOW  0x80  /* INT pin active low */
#define MPU6050_INT_CFG_OPEN_DRAIN  0x40  /* INT pin open-drain */
#define MPU6050_INT_CFG_LATCH_INT   0x20  /* Latch INT until cleared */
#define MPU6050_INT_CFG_RD_CLEAR    0x10  /* Clear INT on any read */

/* INT_ENABLE bits (0x38)
 * Register Map Section 4.8
 */

#define MPU6050_INT_EN_FIFO_OFLOW   0x10  /* FIFO overflow interrupt */
#define MPU6050_INT_EN_DMP          0x02  /* DMP interrupt */
#define MPU6050_INT_EN_DATA_RDY     0x01  /* Data ready interrupt */

/* USER_CTRL bits (0x6A)
 * Register Map Section 4.27
 */

#define MPU6050_USERCTRL_DMP_EN     0x80  /* Enable DMP */
#define MPU6050_USERCTRL_FIFO_EN    0x40  /* Enable FIFO */
#define MPU6050_USERCTRL_DMP_RESET  0x08  /* Reset DMP */
#define MPU6050_USERCTRL_FIFO_RESET 0x04  /* Reset FIFO */

/* Gyroscope full-scale range bits (GYRO_CONFIG 0x1B)
 * Register Map Section 4.4
 */

#define MPU6050_GYRO_FS_250         0x00  /* ±250  °/s -> 131.0 LSB/°/s */
#define MPU6050_GYRO_FS_500         0x08  /* ±500  °/s ->  65.5 LSB/°/s */
#define MPU6050_GYRO_FS_1000        0x10  /* ±1000 °/s ->  32.8 LSB/°/s */
#define MPU6050_GYRO_FS_2000        0x18  /* ±2000 °/s ->  16.4 LSB/°/s */

/* Accelerometer full-scale range bits (ACCEL_CONFIG 0x1C)
 * Register Map Section 4.5
 */

#define MPU6050_ACCEL_FS_2G         0x00  /* ±2g  -> 16384 LSB/g */
#define MPU6050_ACCEL_FS_4G         0x08  /* ±4g  ->  8192 LSB/g */
#define MPU6050_ACCEL_FS_8G         0x10  /* ±8g  ->  4096 LSB/g */
#define MPU6050_ACCEL_FS_16G        0x18  /* ±16g ->  2048 LSB/g */

/* Sensitivity scale factors (LSB per unit)
 * Product Specification Section 6.1 / 6.2
 */

#define MPU6050_ACCEL_SENS_2G       16384.0f
#define MPU6050_ACCEL_SENS_4G        8192.0f
#define MPU6050_ACCEL_SENS_8G        4096.0f
#define MPU6050_ACCEL_SENS_16G       2048.0f
#define MPU6050_GYRO_SENS_250         131.0f
#define MPU6050_GYRO_SENS_500          65.5f
#define MPU6050_GYRO_SENS_1000         32.8f
#define MPU6050_GYRO_SENS_2000         16.4f

/* Temperature conversion constants
 * Register Map Section 4.18: Temp_degC = TEMP_OUT/340 + 36.53
 */

#define MPU6050_TEMP_SENSITIVITY    340.0f
#define MPU6050_TEMP_OFFSET         36.53f

/* DMP FIFO packet size for quaternion output (16-bit)
 * 4 quaternion values x 2 bytes each
 */

#define MPU6050_DMP_FIFO_RATE_DIVISOR   0x01  /* 200Hz / (1+1) = 100Hz */
#define MPU6050_DMP_PACKET_SIZE         28    /* Bytes per DMP FIFO packet */

/* Calibration constants */

#define MPU6050_CALIB_SAMPLES       1000  /* Samples for calibration */
#define MPU6050_GRAVITY_LSB_2G      16384 /* 1g in LSB at ±2g range */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Accelerometer full-scale range */

typedef enum
{
  MPU6050_ACCEL_RANGE_2G  = MPU6050_ACCEL_FS_2G,
  MPU6050_ACCEL_RANGE_4G  = MPU6050_ACCEL_FS_4G,
  MPU6050_ACCEL_RANGE_8G  = MPU6050_ACCEL_FS_8G,
  MPU6050_ACCEL_RANGE_16G = MPU6050_ACCEL_FS_16G,
} mpu6050_accel_range_t;

/* Gyroscope full-scale range */

typedef enum
{
  MPU6050_GYRO_RANGE_250  = MPU6050_GYRO_FS_250,
  MPU6050_GYRO_RANGE_500  = MPU6050_GYRO_FS_500,
  MPU6050_GYRO_RANGE_1000 = MPU6050_GYRO_FS_1000,
  MPU6050_GYRO_RANGE_2000 = MPU6050_GYRO_FS_2000,
} mpu6050_gyro_range_t;

/* Raw sensor data (straight from registers, no scaling) */

typedef struct
{
  int16_t accel_x;  /* Raw accelerometer X */
  int16_t accel_y;  /* Raw accelerometer Y */
  int16_t accel_z;  /* Raw accelerometer Z */
  int16_t gyro_x;   /* Raw gyroscope X */
  int16_t gyro_y;   /* Raw gyroscope Y */
  int16_t gyro_z;   /* Raw gyroscope Z */
  int16_t temp_raw; /* Raw temperature */
} mpu6050_raw_data_t;

/* Scaled sensor data (physical units) */

typedef struct
{
  float accel_x;  /* Accelerometer X [g] */
  float accel_y;  /* Accelerometer Y [g] */
  float accel_z;  /* Accelerometer Z [g] */
  float gyro_x;   /* Gyroscope X [deg/s] */
  float gyro_y;   /* Gyroscope Y [deg/s] */
  float gyro_z;   /* Gyroscope Z [deg/s] */
  float temp_c;   /* Temperature [Celsius] */
} mpu6050_data_t;

/* Quaternion output from DMP
 * Values normalized to range [-1.0, 1.0]
 * DMP System Specification Section 4
 */

typedef struct
{
  float w;  /* Scalar component */
  float x;  /* X component */
  float y;  /* Y component */
  float z;  /* Z component */
} mpu6050_quaternion_t;

/* Euler angles derived from quaternion */

typedef struct
{
  float roll;   /* Rotation around X axis [deg] */
  float pitch;  /* Rotation around Y axis [deg] */
  float yaw;    /* Rotation around Z axis [deg] */
} mpu6050_euler_t;

/* Gyroscope bias offsets (from calibration) */

typedef struct
{
  int16_t gyro_x;  /* Gyro X bias offset */
  int16_t gyro_y;  /* Gyro Y bias offset */
  int16_t gyro_z;  /* Gyro Z bias offset */
} mpu6050_offsets_t;

/* Interrupt callback signature.
 * Called from ISR context — keep it minimal (set a flag or
 * post to a FreeRTOS queue; never block).
 */

typedef void (*mpu6050_isr_cb_t)(void *arg);

/* Driver configuration structure */

typedef struct
{
  uint8_t               i2c_addr;    /* I2C address (default 0x68) */
  mpu6050_accel_range_t accel_range; /* Accelerometer range */
  mpu6050_gyro_range_t  gyro_range;  /* Gyroscope range */
  uint8_t               sample_rate; /* Sample rate divider (0-255) */
  bool                  dmp_enable;  /* Enable DMP for quaternions */
  bool                  int_enable;  /* Enable data-ready interrupt */
  mpu6050_isr_cb_t      isr_cb;      /* ISR callback (NULL = none) */
  void                 *isr_arg;     /* ISR callback argument */
} mpu6050_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: mpu6050_init
 *
 * Description:
 *   Initialize MPU-6050: I2C device registration, device reset, clock
 *   source selection, range configuration, optional DMP load, and
 *   interrupt setup on MAIA_GPIO_IMU_INT (GPIO4).
 *
 * Input Parameters:
 *   config - Driver configuration structure (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_init(const mpu6050_config_t *config);

/****************************************************************************
 * Name: mpu6050_deinit
 *
 * Description:
 *   De-initialize driver: remove ISR, put device to sleep, remove I2C
 *   device handle.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_deinit(void);

/****************************************************************************
 * Name: mpu6050_read_raw
 *
 * Description:
 *   Read raw 16-bit values for accel, gyro and temperature in a single
 *   14-byte burst starting at ACCEL_XOUT_H (0x3B).
 *
 * Input Parameters:
 *   data - Pointer to raw data structure (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data);

/****************************************************************************
 * Name: mpu6050_read_scaled
 *
 * Description:
 *   Read and convert sensor data to physical units (g, deg/s, Celsius).
 *
 * Input Parameters:
 *   data - Pointer to scaled data structure (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_read_scaled(mpu6050_data_t *data);

/****************************************************************************
 * Name: mpu6050_read_dmp
 *
 * Description:
 *   Read one DMP FIFO packet and extract quaternion.
 *   Only valid when DMP is enabled (config.dmp_enable = true).
 *   Block until packet available or timeout.
 *
 * Input Parameters:
 *   quat    - Pointer to quaternion output (must not be NULL)
 *   timeout - Timeout in milliseconds (0 = non-blocking)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_TIMEOUT if no packet within timeout;
 *   error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_read_dmp(mpu6050_quaternion_t *quat,
                            uint32_t timeout_ms);

/****************************************************************************
 * Name: mpu6050_quaternion_to_euler
 *
 * Description:
 *   Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.
 *   Uses ZYX convention (yaw-pitch-roll).
 *
 * Input Parameters:
 *   quat  - Quaternion input (must not be NULL)
 *   euler - Euler angles output (must not be NULL)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu6050_quaternion_to_euler(const mpu6050_quaternion_t *quat,
                                  mpu6050_euler_t *euler);

/****************************************************************************
 * Name: mpu6050_calibrate
 *
 * Description:
 *   Compute gyroscope bias offsets by averaging MAIA_CALIB_SAMPLES
 *   readings while device is stationary. Writes offsets to hardware
 *   offset registers. Device must be flat and still during calibration.
 *
 * Input Parameters:
 *   offsets - Optional pointer to store computed offsets (NULL = discard)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_calibrate(mpu6050_offsets_t *offsets);

/****************************************************************************
 * Name: mpu6050_self_test
 *
 * Description:
 *   Run factory self-test sequence per Product Specification Section 4.
 *   Compares factory-trimmed values against measured response.
 *
 * Returned Value:
 *   ESP_OK if self-test passes; ESP_FAIL otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_self_test(void);

/****************************************************************************
 * Name: mpu6050_set_accel_range
 *
 * Description:
 *   Change accelerometer full-scale range at runtime.
 *
 * Input Parameters:
 *   range - New accelerometer range
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_set_accel_range(mpu6050_accel_range_t range);

/****************************************************************************
 * Name: mpu6050_set_gyro_range
 *
 * Description:
 *   Change gyroscope full-scale range at runtime.
 *
 * Input Parameters:
 *   range - New gyroscope range
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_set_gyro_range(mpu6050_gyro_range_t range);

/****************************************************************************
 * Name: mpu6050_sleep
 *
 * Description:
 *   Put device into low-power sleep mode (SLEEP bit in PWR_MGMT_1).
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_sleep(void);

/****************************************************************************
 * Name: mpu6050_wakeup
 *
 * Description:
 *   Wake device from sleep mode.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_wakeup(void);

/****************************************************************************
 * Name: mpu6050_who_am_i
 *
 * Description:
 *   Read WHO_AM_I register. Expected value: 0x68.
 *
 * Input Parameters:
 *   val - Pointer to store register value (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t mpu6050_who_am_i(uint8_t *val);

#ifdef __cplusplus
}
#endif

#endif /* __COMPONENTS_DRIVERS_MPU6050_INCLUDE_MPU6050_H */