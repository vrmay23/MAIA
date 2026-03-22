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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

/**********************************************************
 * components/drivers/mpu6050/src/mpu6050.c
 *
 * MPU-6050 6-Axis IMU Driver
 * InvenSense MPU-6050 Accelerometer + Gyroscope with DMP
 *
 * Architecture:
 *   - I2C via maia_i2c_get_bus_handle() (ESP-IDF new
 *     master API)
 *   - Interrupt-driven data-ready on MAIA_GPIO_IMU_INT
 *     (GPIO4)
 *   - Optional DMP firmware upload for quaternion output
 *   - Calibration writes offsets to hardware offset
 *     registers
 *
 * Note on I2C reads (GY-521 clone compatibility):
 *   Some MPU6050 clones do not support I2C repeated
 *   START condition. i2c_master_transmit_receive() uses
 *   repeated START and causes timeouts on these modules.
 *   We use separate transmit + receive calls with a
 *   small delay between them instead.
 *
 * Reference: PS-MPU-6000A-00 v3.4
 * Reference: RM-MPU-6000A-00 v4.2
 * Reference: AN-MPU-6050_DMP_System_Specification_v3_1
 *
 **********************************************************/

/**********************************************************
 * Included Files
 **********************************************************/

#include "mpu6050.h"
#include "maia_board.h"
#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/**********************************************************
 * Pre-processor Definitions
 **********************************************************/

#define TAG                     "[MPU6050]"

/* I2C transaction timeout (ms) */

#define I2C_TIMEOUT_MS          100

/* Delay between transmit and receive for GY-521 clone
 * compatibility. Some clones do not support repeated
 * START — separate calls with a small delay work around
 * this limitation.
 */

#define I2C_READ_DELAY_MS       10

/* I2C clock for GY-521 clone — 100kHz more reliable
 * than 400kHz
 */

#define MPU6050_I2C_SPEED_HZ    100000

/* Reset stabilization delay (datasheet: 100ms after
 * reset)
 */

#define MPU6050_RESET_DELAY_MS  100

/* DMP startup delay */

#define MPU6050_DMP_DELAY_MS    50

/* Maximum FIFO size in bytes */

#define MPU6050_FIFO_SIZE       1024

/* Self-test: tolerance percentage (14% per datasheet
 * Section 4)
 */

#define MPU6050_SELFTEST_TOL    14

/* DMP firmware blob size */

#define MPU6050_DMP_FW_SIZE     3062

/* DMP firmware start address in MPU memory */

#define MPU6050_DMP_FW_START    0x0400

/* DMP update chunk size */

#define MPU6050_DMP_CHUNK       16

/**********************************************************
 * Private Types
 **********************************************************/

struct mpu6050_dev_s
{
  i2c_master_dev_handle_t  i2c_dev;
  mpu6050_accel_range_t    accel_range;
  mpu6050_gyro_range_t     gyro_range;
  float                    accel_sens;
  float                    gyro_sens;
  bool                     dmp_enabled;
  bool                     initialized;
  SemaphoreHandle_t        dmp_sem;
  mpu6050_isr_cb_t         isr_cb;
  void                    *isr_arg;
};

/**********************************************************
 * DMP Firmware
 *
 * InvenSense DMP firmware v6.1.2 for MPU-6050.
 * Source: Jeff Rowberg i2cdevlib (MIT License)
 *
 * IMPORTANT: This is a PLACEHOLDER array filled with
 * zeros. The full 3062-byte firmware blob must be
 * copied from:
 *   https://github.com/jrowberg/i2cdevlib/blob/master/
 *     Arduino/MPU6050/
 *     MPU6050_6Axis_MotionApps612.cpp
 *
 * Search for: dmpMemory[MPU6050_DMP_CODE_SIZE]
 * Copy the entire array content (3062 bytes).
 *
 * DMP will NOT work until this blob is replaced.
 * Non-DMP mode (accel/gyro raw/scaled reads) works
 * regardless of this array.
 **********************************************************/

static const uint8_t g_dmp_fw[MPU6050_DMP_FW_SIZE] =
{
  0  /* TODO: paste full 3062-byte DMP v6.12 blob here */
};

static const uint8_t g_dmp_config[][6] =
{
  {0x03, 0x7b, 0x03, 0x4c, 0xcd, 0x6c},
  {0x02, 0x16, 0x02, 0x00,
   MPU6050_DMP_FIFO_RATE_DIVISOR},
};

/**********************************************************
 * Private Data
 **********************************************************/

static struct mpu6050_dev_s g_dev;

/**********************************************************
 * Private Functions
 **********************************************************/

static esp_err_t mpu6050_write_reg(uint8_t reg,
                                    uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  return i2c_master_transmit(g_dev.i2c_dev, buf,
                              sizeof(buf),
                              I2C_TIMEOUT_MS);
}

static esp_err_t mpu6050_read_reg(uint8_t  reg,
                                   uint8_t *val)
{
  esp_err_t ret;

  ret = i2c_master_transmit(g_dev.i2c_dev, &reg, 1,
                             I2C_TIMEOUT_MS);
  if (ret != ESP_OK)
    {
      return ret;
    }

  vTaskDelay(pdMS_TO_TICKS(I2C_READ_DELAY_MS));

  return i2c_master_receive(g_dev.i2c_dev, val, 1,
                             I2C_TIMEOUT_MS);
}

static esp_err_t mpu6050_read_burst(uint8_t  reg,
                                     uint8_t *buf,
                                     size_t   len)
{
  esp_err_t ret;

  ret = i2c_master_transmit(g_dev.i2c_dev, &reg, 1,
                             I2C_TIMEOUT_MS);
  if (ret != ESP_OK)
    {
      return ret;
    }

  vTaskDelay(pdMS_TO_TICKS(I2C_READ_DELAY_MS));

  return i2c_master_receive(g_dev.i2c_dev, buf, len,
                             I2C_TIMEOUT_MS);
}

static void mpu6050_set_accel_sens(
  mpu6050_accel_range_t range)
{
  switch (range)
    {
      case MPU6050_ACCEL_RANGE_2G:
        g_dev.accel_sens = MPU6050_ACCEL_SENS_2G;
        break;
      case MPU6050_ACCEL_RANGE_4G:
        g_dev.accel_sens = MPU6050_ACCEL_SENS_4G;
        break;
      case MPU6050_ACCEL_RANGE_8G:
        g_dev.accel_sens = MPU6050_ACCEL_SENS_8G;
        break;
      case MPU6050_ACCEL_RANGE_16G:
        g_dev.accel_sens = MPU6050_ACCEL_SENS_16G;
        break;
      default:
        g_dev.accel_sens = MPU6050_ACCEL_SENS_2G;
        break;
    }
}

static void mpu6050_set_gyro_sens(
  mpu6050_gyro_range_t range)
{
  switch (range)
    {
      case MPU6050_GYRO_RANGE_250:
        g_dev.gyro_sens = MPU6050_GYRO_SENS_250;
        break;
      case MPU6050_GYRO_RANGE_500:
        g_dev.gyro_sens = MPU6050_GYRO_SENS_500;
        break;
      case MPU6050_GYRO_RANGE_1000:
        g_dev.gyro_sens = MPU6050_GYRO_SENS_1000;
        break;
      case MPU6050_GYRO_RANGE_2000:
        g_dev.gyro_sens = MPU6050_GYRO_SENS_2000;
        break;
      default:
        g_dev.gyro_sens = MPU6050_GYRO_SENS_250;
        break;
    }
}

static void IRAM_ATTR mpu6050_isr_handler(void *arg)
{
  BaseType_t higher_prio_woken = pdFALSE;

  if (g_dev.dmp_sem != NULL)
    {
      xSemaphoreGiveFromISR(g_dev.dmp_sem,
                             &higher_prio_woken);
    }

  if (g_dev.isr_cb != NULL)
    {
      g_dev.isr_cb(g_dev.isr_arg);
    }

  portYIELD_FROM_ISR(higher_prio_woken);
}

static esp_err_t mpu6050_init_interrupt(void)
{
  esp_err_t ret;

  g_dev.dmp_sem = xSemaphoreCreateBinary();
  if (g_dev.dmp_sem == NULL)
    {
      ESP_LOGE(TAG, "Failed to create DMP semaphore");
      return ESP_ERR_NO_MEM;
    }

  ret = gpio_isr_handler_add(MAIA_GPIO_IMU_INT,
                              mpu6050_isr_handler,
                              NULL);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to add ISR handler: %s",
               esp_err_to_name(ret));
      vSemaphoreDelete(g_dev.dmp_sem);
      g_dev.dmp_sem = NULL;
      return ret;
    }

  ret = gpio_set_intr_type(MAIA_GPIO_IMU_INT,
                            GPIO_INTR_POSEDGE);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set intr type: %s",
               esp_err_to_name(ret));
      gpio_isr_handler_remove(MAIA_GPIO_IMU_INT);
      vSemaphoreDelete(g_dev.dmp_sem);
      g_dev.dmp_sem = NULL;
      return ret;
    }

  ret = gpio_intr_enable(MAIA_GPIO_IMU_INT);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to enable interrupt: %s",
               esp_err_to_name(ret));
      gpio_isr_handler_remove(MAIA_GPIO_IMU_INT);
      vSemaphoreDelete(g_dev.dmp_sem);
      g_dev.dmp_sem = NULL;
    }

  return ret;
}

static esp_err_t mpu6050_dmp_write_mem(
  uint16_t       mem_addr,
  const uint8_t *data,
  uint16_t       len)
{
  esp_err_t ret;
  uint16_t  written = 0;
  uint8_t   chunk[MPU6050_DMP_CHUNK + 1];
  uint8_t   bank;
  uint8_t   addr;
  uint16_t  chunk_len;

  while (written < len)
    {
      bank = (uint8_t)((mem_addr + written) >> 8);
      addr = (uint8_t)((mem_addr + written) & 0xFF);

      ret = mpu6050_write_reg(
              MPU6050_DMP_MEMORY_BANK, bank);
      if (ret != ESP_OK)
        {
          return ret;
        }

      ret = mpu6050_write_reg(
              MPU6050_DMP_MEMORY_START, addr);
      if (ret != ESP_OK)
        {
          return ret;
        }

      chunk_len = len - written;
      if (chunk_len > MPU6050_DMP_CHUNK)
        {
          chunk_len = MPU6050_DMP_CHUNK;
        }

      if ((uint16_t)addr + chunk_len > 256)
        {
          chunk_len = 256 - addr;
        }

      chunk[0] = MPU6050_DMP_MEMORY_R_W;
      memcpy(&chunk[1], &data[written], chunk_len);

      ret = i2c_master_transmit(g_dev.i2c_dev, chunk,
                                 chunk_len + 1,
                                 I2C_TIMEOUT_MS);
      if (ret != ESP_OK)
        {
          return ret;
        }

      written += chunk_len;
    }

  return ESP_OK;
}

static esp_err_t mpu6050_dmp_load(void)
{
  esp_err_t ret;
  uint8_t   buf[2];
  size_t    i;

  ESP_LOGI(TAG, "Loading DMP firmware (%u bytes)",
           MPU6050_DMP_FW_SIZE);

  ret = mpu6050_dmp_write_mem(MPU6050_DMP_FW_START,
                               g_dmp_fw,
                               MPU6050_DMP_FW_SIZE);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "DMP firmware upload failed: %s",
               esp_err_to_name(ret));
      return ret;
    }

  for (i = 0;
       i < sizeof(g_dmp_config) /
           sizeof(g_dmp_config[0]);
       i++)
    {
      const uint8_t *e = g_dmp_config[i];
      uint16_t mem_addr =
        ((uint16_t)e[0] << 8) | e[1];

      ret = mpu6050_dmp_write_mem(mem_addr,
                                   &e[3], e[2]);
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "DMP config [%u] failed",
                   (unsigned)i);
          return ret;
        }
    }

  buf[0] = MPU6050_REG_DMP_CFG_1;
  buf[1] = (uint8_t)(MPU6050_DMP_FW_START >> 8);
  ret = i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                             I2C_TIMEOUT_MS);
  if (ret != ESP_OK)
    {
      return ret;
    }

  buf[0] = MPU6050_REG_DMP_CFG_2;
  buf[1] = (uint8_t)(MPU6050_DMP_FW_START & 0xFF);
  ret = i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                             I2C_TIMEOUT_MS);

  ESP_LOGI(TAG, "DMP firmware loaded successfully");
  return ret;
}

/**********************************************************
 * Public Functions
 **********************************************************/

esp_err_t mpu6050_init(const mpu6050_config_t *config)
{
  esp_err_t               ret;
  i2c_master_bus_handle_t bus;
  i2c_device_config_t     dev_cfg;
  uint8_t                 who;

  if (config == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  if (g_dev.initialized)
    {
      ESP_LOGW(TAG, "Already initialized");
      return ESP_OK;
    }

  memset(&g_dev, 0, sizeof(g_dev));
  g_dev.isr_cb  = config->isr_cb;
  g_dev.isr_arg = config->isr_arg;

  bus = maia_i2c_get_bus_handle();
  if (bus == NULL)
    {
      ESP_LOGE(TAG, "I2C bus not initialized");
      return ESP_ERR_INVALID_STATE;
    }

  /* Zero the entire dev_cfg struct before filling
   * known fields. ESP-IDF v6.0 i2c_device_config_t
   * contains internal fields (flags, scl_wait_us,
   * etc.) that MUST be zero — stack garbage in these
   * fields causes i2c_master_bus_add_device() to
   * create a corrupted device handle, leading to
   * ESP_ERR_INVALID_STATE on the first transmit.
   */

  memset(&dev_cfg, 0, sizeof(dev_cfg));
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address  = config->i2c_addr;
  dev_cfg.scl_speed_hz    = MPU6050_I2C_SPEED_HZ;

  ret = i2c_master_bus_add_device(bus, &dev_cfg,
                                   &g_dev.i2c_dev);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "Failed to register I2C device: %s",
               esp_err_to_name(ret));
      return ret;
    }

  vTaskDelay(pdMS_TO_TICKS(100));

  /* Wake device from sleep (POR default is sleep) */

  ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1,
                           0x00);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Wake write failed: %s",
               esp_err_to_name(ret));
      goto init_fail;
    }

  vTaskDelay(pdMS_TO_TICKS(50));

  ret = mpu6050_who_am_i(&who);
  if (ret != ESP_OK || who != MPU6050_WHO_AM_I_VAL)
    {
      ESP_LOGE(TAG,
               "WHO_AM_I failed: got 0x%02x "
               "expected 0x%02x",
               who, MPU6050_WHO_AM_I_VAL);
      g_dev.initialized = false;
      return (ret != ESP_OK) ? ret : ESP_ERR_NOT_FOUND;
    }

  /* Full device reset */

  ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1,
                           MPU6050_PWR1_DEVICE_RESET);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  vTaskDelay(pdMS_TO_TICKS(MPU6050_RESET_DELAY_MS));

  /* Select PLL with X-axis gyro as clock source */

  ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1,
                           MPU6050_PWR1_CLKSEL_XGYRO);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  /* Set sample rate divider */

  ret = mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV,
                           config->sample_rate);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  /* Configure gyroscope range */

  ret = mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG,
                           (uint8_t)config->gyro_range);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  g_dev.gyro_range = config->gyro_range;
  mpu6050_set_gyro_sens(config->gyro_range);

  /* Configure accelerometer range */

  ret = mpu6050_write_reg(
          MPU6050_REG_ACCEL_CONFIG,
          (uint8_t)config->accel_range);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  g_dev.accel_range = config->accel_range;
  mpu6050_set_accel_sens(config->accel_range);

  /* Optional DMP firmware load */

  if (config->dmp_enable)
    {
      ret = mpu6050_dmp_load();
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "DMP load failed: %s",
                   esp_err_to_name(ret));
          goto init_fail;
        }

      ret = mpu6050_write_reg(
              MPU6050_REG_USER_CTRL,
              MPU6050_USERCTRL_FIFO_RESET |
              MPU6050_USERCTRL_DMP_RESET);
      if (ret != ESP_OK)
        {
          goto init_fail;
        }

      vTaskDelay(pdMS_TO_TICKS(MPU6050_DMP_DELAY_MS));

      ret = mpu6050_write_reg(
              MPU6050_REG_USER_CTRL,
              MPU6050_USERCTRL_DMP_EN |
              MPU6050_USERCTRL_FIFO_EN);
      if (ret != ESP_OK)
        {
          goto init_fail;
        }

      ret = mpu6050_write_reg(
              MPU6050_REG_INT_ENABLE,
              MPU6050_INT_EN_DMP);
      if (ret != ESP_OK)
        {
          goto init_fail;
        }

      g_dev.dmp_enabled = true;
      ESP_LOGI(TAG, "DMP enabled");
    }
  else if (config->int_enable)
    {
      ret = mpu6050_write_reg(
              MPU6050_REG_INT_ENABLE,
              MPU6050_INT_EN_DATA_RDY);
      if (ret != ESP_OK)
        {
          goto init_fail;
        }
    }

  /* Configure INT pin: clear on any read */

  ret = mpu6050_write_reg(MPU6050_REG_INT_PIN_CFG,
                           MPU6050_INT_CFG_RD_CLEAR);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  /* Setup GPIO interrupt if needed */

  if (config->int_enable || config->dmp_enable)
    {
      ret = mpu6050_init_interrupt();
      if (ret != ESP_OK)
        {
          goto init_fail;
        }
    }

  g_dev.initialized = true;
  ESP_LOGI(TAG,
           "Initialized: addr=0x%02x accel=+-%dg "
           "gyro=+-%ddps dmp=%s",
           config->i2c_addr,
           (config->accel_range ==
            MPU6050_ACCEL_RANGE_2G)  ?  2 :
           (config->accel_range ==
            MPU6050_ACCEL_RANGE_4G)  ?  4 :
           (config->accel_range ==
            MPU6050_ACCEL_RANGE_8G)  ?  8 : 16,
           (config->gyro_range ==
            MPU6050_GYRO_RANGE_250)  ?  250 :
           (config->gyro_range ==
            MPU6050_GYRO_RANGE_500)  ?  500 :
           (config->gyro_range ==
            MPU6050_GYRO_RANGE_1000) ? 1000 : 2000,
           config->dmp_enable ? "yes" : "no");

  return ESP_OK;

init_fail:
  ESP_LOGE(TAG, "Init failed: %s",
           esp_err_to_name(ret));
  i2c_master_bus_rm_device(g_dev.i2c_dev);
  return ret;
}

esp_err_t mpu6050_deinit(void)
{
  if (!g_dev.initialized)
    {
      return ESP_OK;
    }

  gpio_intr_disable(MAIA_GPIO_IMU_INT);
  gpio_isr_handler_remove(MAIA_GPIO_IMU_INT);

  if (g_dev.dmp_sem != NULL)
    {
      vSemaphoreDelete(g_dev.dmp_sem);
      g_dev.dmp_sem = NULL;
    }

  mpu6050_sleep();
  i2c_master_bus_rm_device(g_dev.i2c_dev);

  g_dev.initialized = false;
  ESP_LOGI(TAG, "De-initialized");
  return ESP_OK;
}

esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data)
{
  esp_err_t ret;
  uint8_t   buf[14];

  if (data == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ret = mpu6050_read_burst(MPU6050_REG_ACCEL_XOUT_H,
                            buf, sizeof(buf));
  if (ret != ESP_OK)
    {
      return ret;
    }

  data->accel_x  = (int16_t)((buf[0]  << 8) | buf[1]);
  data->accel_y  = (int16_t)((buf[2]  << 8) | buf[3]);
  data->accel_z  = (int16_t)((buf[4]  << 8) | buf[5]);
  data->temp_raw = (int16_t)((buf[6]  << 8) | buf[7]);
  data->gyro_x   = (int16_t)((buf[8]  << 8) | buf[9]);
  data->gyro_y   = (int16_t)((buf[10] << 8) | buf[11]);
  data->gyro_z   = (int16_t)((buf[12] << 8) | buf[13]);

  return ESP_OK;
}

esp_err_t mpu6050_read_scaled(mpu6050_data_t *data)
{
  esp_err_t          ret;
  mpu6050_raw_data_t raw;

  if (data == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ret = mpu6050_read_raw(&raw);
  if (ret != ESP_OK)
    {
      return ret;
    }

  data->accel_x = (float)raw.accel_x /
                  g_dev.accel_sens;
  data->accel_y = (float)raw.accel_y /
                  g_dev.accel_sens;
  data->accel_z = (float)raw.accel_z /
                  g_dev.accel_sens;
  data->gyro_x  = (float)raw.gyro_x  /
                  g_dev.gyro_sens;
  data->gyro_y  = (float)raw.gyro_y  /
                  g_dev.gyro_sens;
  data->gyro_z  = (float)raw.gyro_z  /
                  g_dev.gyro_sens;
  data->temp_c  = (float)raw.temp_raw /
                  MPU6050_TEMP_SENSITIVITY +
                  MPU6050_TEMP_OFFSET;

  return ESP_OK;
}

esp_err_t mpu6050_read_dmp(mpu6050_quaternion_t *quat,
                            uint32_t              timeout_ms)
{
  esp_err_t  ret;
  uint8_t    fifo_buf[MPU6050_DMP_PACKET_SIZE];
  uint8_t    cnt_h;
  uint8_t    cnt_l;
  uint16_t   fifo_count;
  int32_t    qw;
  int32_t    qx;
  int32_t    qy;
  int32_t    qz;
  TickType_t ticks;

  if (!g_dev.dmp_enabled)
    {
      ESP_LOGE(TAG, "DMP not enabled");
      return ESP_ERR_INVALID_STATE;
    }

  if (quat == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ticks = (timeout_ms == 0) ? 0 :
          pdMS_TO_TICKS(timeout_ms);
  if (xSemaphoreTake(g_dev.dmp_sem, ticks) != pdTRUE)
    {
      return ESP_ERR_TIMEOUT;
    }

  ret = mpu6050_read_reg(MPU6050_REG_FIFO_COUNTH,
                          &cnt_h);
  if (ret != ESP_OK)
    {
      return ret;
    }

  ret = mpu6050_read_reg(MPU6050_REG_FIFO_COUNTL,
                          &cnt_l);
  if (ret != ESP_OK)
    {
      return ret;
    }

  fifo_count = ((uint16_t)cnt_h << 8) | cnt_l;

  if (fifo_count < MPU6050_DMP_PACKET_SIZE)
    {
      ESP_LOGW(TAG, "FIFO underflow: %u bytes",
               fifo_count);
      return ESP_ERR_INVALID_SIZE;
    }

  if (fifo_count >= MPU6050_FIFO_SIZE)
    {
      ESP_LOGW(TAG, "FIFO overflow — resetting");
      mpu6050_write_reg(
        MPU6050_REG_USER_CTRL,
        MPU6050_USERCTRL_FIFO_RESET |
        MPU6050_USERCTRL_DMP_EN     |
        MPU6050_USERCTRL_FIFO_EN);
      return ESP_ERR_INVALID_STATE;
    }

  ret = mpu6050_read_burst(MPU6050_REG_FIFO_R_W,
                            fifo_buf,
                            MPU6050_DMP_PACKET_SIZE);
  if (ret != ESP_OK)
    {
      return ret;
    }

  qw = (int32_t)(((uint32_t)fifo_buf[0]  << 24) |
                 ((uint32_t)fifo_buf[1]  << 16) |
                 ((uint32_t)fifo_buf[2]  <<  8) |
                  (uint32_t)fifo_buf[3]);

  qx = (int32_t)(((uint32_t)fifo_buf[4]  << 24) |
                 ((uint32_t)fifo_buf[5]  << 16) |
                 ((uint32_t)fifo_buf[6]  <<  8) |
                  (uint32_t)fifo_buf[7]);

  qy = (int32_t)(((uint32_t)fifo_buf[8]  << 24) |
                 ((uint32_t)fifo_buf[9]  << 16) |
                 ((uint32_t)fifo_buf[10] <<  8) |
                  (uint32_t)fifo_buf[11]);

  qz = (int32_t)(((uint32_t)fifo_buf[12] << 24) |
                 ((uint32_t)fifo_buf[13] << 16) |
                 ((uint32_t)fifo_buf[14] <<  8) |
                  (uint32_t)fifo_buf[15]);

  quat->w = (float)qw / (float)(1L << 30);
  quat->x = (float)qx / (float)(1L << 30);
  quat->y = (float)qy / (float)(1L << 30);
  quat->z = (float)qz / (float)(1L << 30);

  return ESP_OK;
}

void mpu6050_quaternion_to_euler(
  const mpu6050_quaternion_t *quat,
  mpu6050_euler_t            *euler)
{
  float sinr_cosp;
  float cosr_cosp;
  float sinp;
  float siny_cosp;
  float cosy_cosp;

  sinr_cosp = 2.0f * (quat->w * quat->x +
                       quat->y * quat->z);
  cosr_cosp = 1.0f - 2.0f * (quat->x * quat->x +
                               quat->y * quat->y);
  euler->roll = atan2f(sinr_cosp, cosr_cosp) *
                (180.0f / (float)M_PI);

  sinp = 2.0f * (quat->w * quat->y -
                  quat->z * quat->x);
  if (fabsf(sinp) >= 1.0f)
    {
      euler->pitch = copysignf(90.0f, sinp);
    }
  else
    {
      euler->pitch = asinf(sinp) *
                     (180.0f / (float)M_PI);
    }

  siny_cosp = 2.0f * (quat->w * quat->z +
                       quat->x * quat->y);
  cosy_cosp = 1.0f - 2.0f * (quat->y * quat->y +
                               quat->z * quat->z);
  euler->yaw = atan2f(siny_cosp, cosy_cosp) *
               (180.0f / (float)M_PI);
}

esp_err_t mpu6050_calibrate(mpu6050_offsets_t *offsets)
{
  esp_err_t          ret;
  mpu6050_raw_data_t raw;
  int32_t            sum_gx = 0;
  int32_t            sum_gy = 0;
  int32_t            sum_gz = 0;
  int32_t            mean_gx;
  int32_t            mean_gy;
  int32_t            mean_gz;
  int16_t            off_val;
  uint8_t            buf[2];
  int                i;

  ESP_LOGI(TAG,
           "Calibrating — keep flat and still "
           "(%d samples)",
           MPU6050_CALIB_SAMPLES);

  for (i = 0; i < MPU6050_CALIB_SAMPLES; i++)
    {
      ret = mpu6050_read_raw(&raw);
      if (ret != ESP_OK)
        {
          return ret;
        }

      sum_gx += raw.gyro_x;
      sum_gy += raw.gyro_y;
      sum_gz += raw.gyro_z;
    }

  mean_gx = sum_gx / MPU6050_CALIB_SAMPLES;
  mean_gy = sum_gy / MPU6050_CALIB_SAMPLES;
  mean_gz = sum_gz / MPU6050_CALIB_SAMPLES;

  off_val = (int16_t)(-mean_gx / 4);
  buf[0] = MPU6050_REG_XG_OFFS_USRH;
  buf[1] = (uint8_t)(off_val >> 8);
  i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                       I2C_TIMEOUT_MS);
  buf[0] = MPU6050_REG_XG_OFFS_USRL;
  buf[1] = (uint8_t)(off_val & 0xFF);
  i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                       I2C_TIMEOUT_MS);

  off_val = (int16_t)(-mean_gy / 4);
  buf[0] = MPU6050_REG_YG_OFFS_USRH;
  buf[1] = (uint8_t)(off_val >> 8);
  i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                       I2C_TIMEOUT_MS);
  buf[0] = MPU6050_REG_YG_OFFS_USRL;
  buf[1] = (uint8_t)(off_val & 0xFF);
  i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                       I2C_TIMEOUT_MS);

  off_val = (int16_t)(-mean_gz / 4);
  buf[0] = MPU6050_REG_ZG_OFFS_USRH;
  buf[1] = (uint8_t)(off_val >> 8);
  i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                       I2C_TIMEOUT_MS);
  buf[0] = MPU6050_REG_ZG_OFFS_USRL;
  buf[1] = (uint8_t)(off_val & 0xFF);
  ret = i2c_master_transmit(g_dev.i2c_dev, buf, 2,
                             I2C_TIMEOUT_MS);

  if (offsets != NULL)
    {
      offsets->gyro_x = (int16_t)mean_gx;
      offsets->gyro_y = (int16_t)mean_gy;
      offsets->gyro_z = (int16_t)mean_gz;
    }

  ESP_LOGI(TAG,
           "Calibration done: gx=%ld gy=%ld gz=%ld",
           (long)mean_gx, (long)mean_gy,
           (long)mean_gz);

  return ret;
}

esp_err_t mpu6050_self_test(void)
{
  esp_err_t ret;
  uint8_t   st_data[4];
  uint8_t   xa_test;
  uint8_t   ya_test;
  uint8_t   za_test;
  uint8_t   xg_test;
  uint8_t   yg_test;
  uint8_t   zg_test;

  ret = mpu6050_read_burst(MPU6050_REG_SELF_TEST_X,
                            st_data, 4);
  if (ret != ESP_OK)
    {
      return ret;
    }

  xg_test = st_data[0] & 0x1F;
  yg_test = st_data[1] & 0x1F;
  zg_test = st_data[2] & 0x1F;
  xa_test = ((st_data[0] >> 3) & 0x1C) |
            ((st_data[3] >> 4) & 0x03);
  ya_test = ((st_data[1] >> 3) & 0x1C) |
            ((st_data[3] >> 2) & 0x03);
  za_test = ((st_data[2] >> 3) & 0x1C) |
             (st_data[3] & 0x03);

  ESP_LOGI(TAG,
           "Self-test: XG=%u YG=%u ZG=%u "
           "XA=%u YA=%u ZA=%u",
           xg_test, yg_test, zg_test,
           xa_test, ya_test, za_test);

  if (xg_test == 0 && yg_test == 0 && zg_test == 0)
    {
      ESP_LOGE(TAG,
               "Self-test FAIL: all gyro trim "
               "values zero");
      return ESP_FAIL;
    }

  ESP_LOGI(TAG, "Self-test PASS");
  return ESP_OK;
}

esp_err_t mpu6050_set_accel_range(
  mpu6050_accel_range_t range)
{
  esp_err_t ret;

  ret = mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG,
                           (uint8_t)range);
  if (ret == ESP_OK)
    {
      g_dev.accel_range = range;
      mpu6050_set_accel_sens(range);
    }

  return ret;
}

esp_err_t mpu6050_set_gyro_range(
  mpu6050_gyro_range_t range)
{
  esp_err_t ret;

  ret = mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG,
                           (uint8_t)range);
  if (ret == ESP_OK)
    {
      g_dev.gyro_range = range;
      mpu6050_set_gyro_sens(range);
    }

  return ret;
}

esp_err_t mpu6050_sleep(void)
{
  return mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1,
                            MPU6050_PWR1_SLEEP);
}

esp_err_t mpu6050_wakeup(void)
{
  return mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1,
                            MPU6050_PWR1_CLKSEL_XGYRO);
}

esp_err_t mpu6050_who_am_i(uint8_t *val)
{
  return mpu6050_read_reg(MPU6050_REG_WHO_AM_I, val);
}