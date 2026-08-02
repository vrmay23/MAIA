/*
 * Copyright 2026 Vinicius Rodrigo May
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
 * components/drivers/lsm6dsox/src/lsm6dsox.c
 *
 * LSM6DSOX 6-Axis IMU Driver
 * STMicroelectronics iNEMO inertial module
 *
 * Architecture:
 *   - I2C via maia_i2c_get_bus_handle() (ESP-IDF new
 *     master API)
 *   - Interrupt-driven data-ready on MAIA_GPIO_IMU_INT
 *     (GPIO4)
 *   - Layered feature set: see the layer map at the top
 *     of lsm6dsox.h. Layers L0-L2 are implemented here;
 *     the remaining layers are stubs returning
 *     ESP_ERR_NOT_SUPPORTED and can be filled in one at
 *     a time without touching what already works.
 *
 * TRANSPORT: I2C ONLY. SPI and MIPI I3C are out of scope
 * for now — SPI is unreachable on this board (CS strapped
 * high, SDO/SA0 strapped to GND) and I3C has no
 * controller on the ESP32-S3. See the TRANSPORT note in
 * lsm6dsox.h for the full rationale.
 *
 * Every bus operation lives in the L0 primitives below.
 * Nothing in L1-L9 calls i2c_master_* directly, so a
 * future SPI or I3C backend replaces L0 alone.
 *
 * Unlike the MPU-6050 clones, the LSM6DSOX honours the
 * I2C repeated START condition, so
 * i2c_master_transmit_receive() is used directly and no
 * inter-transaction delay is needed.
 *
 * Reference: DS12814 - LSM6DSOX Datasheet
 * Reference: AN5272 - LSM6DSOX Application Note
 *
 **********************************************************/

/**********************************************************
 * Included Files
 **********************************************************/

#include "lsm6dsox.h"
#include "maia_board.h"
#include <string.h>
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

#define TAG                      "[LSM6DSOX]"

/* I2C transaction timeout (ms) */

#define I2C_TIMEOUT_MS           100

/* The LSM6DSOX supports up to 400 kHz fast mode */

#define LSM6DSOX_I2C_SPEED_HZ    400000

/* Boot time from power-on before registers are
 * accessible (datasheet: 10 ms)
 */

#define LSM6DSOX_BOOT_DELAY_MS   15

/* Software reset poll budget. The reset itself takes
 * ~50 us; this is a generous ceiling.
 */

#define LSM6DSOX_RESET_TRIES     20
#define LSM6DSOX_RESET_POLL_MS   1

/**********************************************************
 * Private Types
 **********************************************************/

struct lsm6dsox_dev_s
{
  i2c_master_dev_handle_t  i2c_dev;
  lsm6dsox_accel_range_t   accel_range;
  lsm6dsox_gyro_range_t    gyro_range;

  /* Rates currently programmed into the chip */

  lsm6dsox_odr_t           accel_odr;
  lsm6dsox_odr_t           gyro_odr;

  /* Rates to restore on wakeup. Kept separate from the
   * live rates so that sleep() -> wakeup() round-trips
   * instead of latching the powered-down state.
   */

  lsm6dsox_odr_t           accel_odr_saved;
  lsm6dsox_odr_t           gyro_odr_saved;

  float                    accel_sens;  /* mg/LSB */
  float                    gyro_sens;   /* mdps/LSB */

  /* Gyro bias in raw LSB. The LSM6DSOX has no gyro
   * offset registers, so this is subtracted in software
   * on every read.
   */

  int16_t                  gyro_bias_x;
  int16_t                  gyro_bias_y;
  int16_t                  gyro_bias_z;

  lsm6dsox_fifo_mode_t     fifo_mode;
  bool                     initialized;
  SemaphoreHandle_t        drdy_sem;
  lsm6dsox_isr_cb_t        isr_cb;
  void                    *isr_arg;
};

/**********************************************************
 * Private Data
 **********************************************************/

static struct lsm6dsox_dev_s g_dev;

/**********************************************************
 * Private Functions
 **********************************************************/

static void lsm6dsox_cache_accel_sens(
  lsm6dsox_accel_range_t range)
{
  switch (range)
    {
      case LSM6DSOX_ACCEL_RANGE_2G:
        g_dev.accel_sens = LSM6DSOX_ACCEL_SENS_2G;
        break;
      case LSM6DSOX_ACCEL_RANGE_4G:
        g_dev.accel_sens = LSM6DSOX_ACCEL_SENS_4G;
        break;
      case LSM6DSOX_ACCEL_RANGE_8G:
        g_dev.accel_sens = LSM6DSOX_ACCEL_SENS_8G;
        break;
      case LSM6DSOX_ACCEL_RANGE_16G:
        g_dev.accel_sens = LSM6DSOX_ACCEL_SENS_16G;
        break;
      default:
        g_dev.accel_sens = LSM6DSOX_ACCEL_SENS_2G;
        break;
    }
}

static void lsm6dsox_cache_gyro_sens(
  lsm6dsox_gyro_range_t range)
{
  switch (range)
    {
      case LSM6DSOX_GYRO_RANGE_125:
        g_dev.gyro_sens = LSM6DSOX_GYRO_SENS_125;
        break;
      case LSM6DSOX_GYRO_RANGE_250:
        g_dev.gyro_sens = LSM6DSOX_GYRO_SENS_250;
        break;
      case LSM6DSOX_GYRO_RANGE_500:
        g_dev.gyro_sens = LSM6DSOX_GYRO_SENS_500;
        break;
      case LSM6DSOX_GYRO_RANGE_1000:
        g_dev.gyro_sens = LSM6DSOX_GYRO_SENS_1000;
        break;
      case LSM6DSOX_GYRO_RANGE_2000:
        g_dev.gyro_sens = LSM6DSOX_GYRO_SENS_2000;
        break;
      default:
        g_dev.gyro_sens = LSM6DSOX_GYRO_SENS_250;
        break;
    }
}

static void IRAM_ATTR lsm6dsox_isr_handler(void *arg)
{
  BaseType_t higher_prio_woken = pdFALSE;

  if (g_dev.drdy_sem != NULL)
    {
      xSemaphoreGiveFromISR(g_dev.drdy_sem,
                             &higher_prio_woken);
    }

  if (g_dev.isr_cb != NULL)
    {
      g_dev.isr_cb(g_dev.isr_arg);
    }

  portYIELD_FROM_ISR(higher_prio_woken);
}

static esp_err_t lsm6dsox_init_interrupt(void)
{
  esp_err_t ret;

  g_dev.drdy_sem = xSemaphoreCreateBinary();
  if (g_dev.drdy_sem == NULL)
    {
      ESP_LOGE(TAG, "Failed to create DRDY semaphore");
      return ESP_ERR_NO_MEM;
    }

  ret = gpio_isr_handler_add(MAIA_GPIO_IMU_INT,
                              lsm6dsox_isr_handler,
                              NULL);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to add ISR handler: %s",
               esp_err_to_name(ret));
      goto fail;
    }

  ret = gpio_set_intr_type(MAIA_GPIO_IMU_INT,
                            GPIO_INTR_POSEDGE);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set intr type: %s",
               esp_err_to_name(ret));
      gpio_isr_handler_remove(MAIA_GPIO_IMU_INT);
      goto fail;
    }

  ret = gpio_intr_enable(MAIA_GPIO_IMU_INT);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to enable interrupt: %s",
               esp_err_to_name(ret));
      gpio_isr_handler_remove(MAIA_GPIO_IMU_INT);
      goto fail;
    }

  return ESP_OK;

fail:
  vSemaphoreDelete(g_dev.drdy_sem);
  g_dev.drdy_sem = NULL;
  return ret;
}

/* Read a register that lives in a non-user bank,
 * restoring the user bank afterwards even on failure.
 */

static esp_err_t lsm6dsox_read_banked(
  lsm6dsox_bank_t bank,
  uint8_t         reg,
  uint8_t        *val)
{
  esp_err_t ret;

  ret = lsm6dsox_set_mem_bank(bank);
  if (ret != ESP_OK)
    {
      return ret;
    }

  ret = lsm6dsox_read_reg(reg, val);

  lsm6dsox_set_mem_bank(LSM6DSOX_BANK_USER);
  return ret;
}

/* Read-modify-write a register in a non-user bank,
 * restoring the user bank afterwards even on failure.
 */

static esp_err_t lsm6dsox_modify_banked(
  lsm6dsox_bank_t bank,
  uint8_t         reg,
  uint8_t         mask,
  uint8_t         val)
{
  esp_err_t ret;

  ret = lsm6dsox_set_mem_bank(bank);
  if (ret != ESP_OK)
    {
      return ret;
    }

  ret = lsm6dsox_modify_reg(reg, mask, val);

  lsm6dsox_set_mem_bank(LSM6DSOX_BANK_USER);
  return ret;
}

/**********************************************************
 * Public Functions — L0: Register Access
 **********************************************************/

esp_err_t lsm6dsox_read_reg(uint8_t reg, uint8_t *val)
{
  if (val == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  return i2c_master_transmit_receive(g_dev.i2c_dev,
                                      &reg, 1, val, 1,
                                      I2C_TIMEOUT_MS);
}

esp_err_t lsm6dsox_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};

  return i2c_master_transmit(g_dev.i2c_dev, buf,
                              sizeof(buf),
                              I2C_TIMEOUT_MS);
}

esp_err_t lsm6dsox_modify_reg(uint8_t reg,
                               uint8_t mask,
                               uint8_t val)
{
  esp_err_t ret;
  uint8_t   cur;

  ret = lsm6dsox_read_reg(reg, &cur);
  if (ret != ESP_OK)
    {
      return ret;
    }

  cur = (uint8_t)((cur & ~mask) | (val & mask));

  return lsm6dsox_write_reg(reg, cur);
}

esp_err_t lsm6dsox_read_burst(uint8_t  reg,
                               uint8_t *buf,
                               size_t   len)
{
  if (buf == NULL || len == 0)
    {
      return ESP_ERR_INVALID_ARG;
    }

  return i2c_master_transmit_receive(g_dev.i2c_dev,
                                      &reg, 1, buf, len,
                                      I2C_TIMEOUT_MS);
}

esp_err_t lsm6dsox_set_mem_bank(lsm6dsox_bank_t bank)
{
  return lsm6dsox_write_reg(LSM6DSOX_REG_FUNC_CFG_ACCESS,
                             (uint8_t)bank);
}

/**********************************************************
 * Public Functions — L1: Core
 **********************************************************/

esp_err_t lsm6dsox_init(const lsm6dsox_config_t *config)
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

  /* Zero the entire dev_cfg struct before filling known
   * fields. ESP-IDF v6.0 i2c_device_config_t contains
   * internal fields (flags, scl_wait_us, etc.) that MUST
   * be zero — stack garbage there yields a corrupted
   * device handle and ESP_ERR_INVALID_STATE on the first
   * transmit.
   */

  memset(&dev_cfg, 0, sizeof(dev_cfg));
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address  = config->i2c_addr;
  dev_cfg.scl_speed_hz    = LSM6DSOX_I2C_SPEED_HZ;

  ret = i2c_master_bus_add_device(bus, &dev_cfg,
                                   &g_dev.i2c_dev);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "Failed to register I2C device: %s",
               esp_err_to_name(ret));
      return ret;
    }

  vTaskDelay(pdMS_TO_TICKS(LSM6DSOX_BOOT_DELAY_MS));

  ret = lsm6dsox_who_am_i(&who);
  if (ret != ESP_OK || who != LSM6DSOX_WHO_AM_I_VAL)
    {
      ESP_LOGE(TAG,
               "WHO_AM_I failed: got 0x%02x "
               "expected 0x%02x",
               who, LSM6DSOX_WHO_AM_I_VAL);
      ret = (ret != ESP_OK) ? ret : ESP_ERR_NOT_FOUND;
      goto init_fail;
    }

  ret = lsm6dsox_reset();
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  /* IF_INC is required for every burst read in this
   * driver. It is set by default at power-on, but the
   * write makes the dependency explicit and survives a
   * caller that cleared it.
   */

  ret = lsm6dsox_write_reg(
          LSM6DSOX_REG_CTRL3_C,
          (uint8_t)(LSM6DSOX_CTRL3_C_IF_INC |
                    (config->bdu_enable ?
                     LSM6DSOX_CTRL3_C_BDU : 0)));
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  ret = lsm6dsox_set_accel_range(config->accel_range);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  ret = lsm6dsox_set_gyro_range(config->gyro_range);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  ret = lsm6dsox_set_accel_odr(config->accel_odr);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  ret = lsm6dsox_set_gyro_odr(config->gyro_odr);
  if (ret != ESP_OK)
    {
      goto init_fail;
    }

  if (config->int_enable)
    {
      ret = lsm6dsox_set_drdy_int(config->int_pin,
                                   true, true);
      if (ret != ESP_OK)
        {
          goto init_fail;
        }

      ret = lsm6dsox_init_interrupt();
      if (ret != ESP_OK)
        {
          goto init_fail;
        }
    }

  g_dev.initialized = true;

  ESP_LOGI(TAG,
           "Initialized: addr=0x%02x accel=+-%dg "
           "gyro=+-%ddps int=%s",
           config->i2c_addr,
           (config->accel_range ==
            LSM6DSOX_ACCEL_RANGE_2G)  ?  2 :
           (config->accel_range ==
            LSM6DSOX_ACCEL_RANGE_4G)  ?  4 :
           (config->accel_range ==
            LSM6DSOX_ACCEL_RANGE_8G)  ?  8 : 16,
           (config->gyro_range ==
            LSM6DSOX_GYRO_RANGE_125)  ?  125 :
           (config->gyro_range ==
            LSM6DSOX_GYRO_RANGE_250)  ?  250 :
           (config->gyro_range ==
            LSM6DSOX_GYRO_RANGE_500)  ?  500 :
           (config->gyro_range ==
            LSM6DSOX_GYRO_RANGE_1000) ? 1000 : 2000,
           config->int_enable ? "yes" : "no");

  return ESP_OK;

init_fail:
  ESP_LOGE(TAG, "Init failed: %s",
           esp_err_to_name(ret));
  i2c_master_bus_rm_device(g_dev.i2c_dev);
  g_dev.i2c_dev = NULL;
  return ret;
}

esp_err_t lsm6dsox_deinit(void)
{
  if (!g_dev.initialized)
    {
      return ESP_OK;
    }

  gpio_intr_disable(MAIA_GPIO_IMU_INT);
  gpio_isr_handler_remove(MAIA_GPIO_IMU_INT);

  if (g_dev.drdy_sem != NULL)
    {
      vSemaphoreDelete(g_dev.drdy_sem);
      g_dev.drdy_sem = NULL;
    }

  lsm6dsox_sleep();
  i2c_master_bus_rm_device(g_dev.i2c_dev);
  g_dev.i2c_dev = NULL;

  g_dev.initialized = false;
  ESP_LOGI(TAG, "De-initialized");
  return ESP_OK;
}

esp_err_t lsm6dsox_who_am_i(uint8_t *val)
{
  return lsm6dsox_read_reg(LSM6DSOX_REG_WHO_AM_I, val);
}

esp_err_t lsm6dsox_reset(void)
{
  esp_err_t ret;
  uint8_t   reg;
  int       tries;

  ret = lsm6dsox_modify_reg(LSM6DSOX_REG_CTRL3_C,
                             LSM6DSOX_CTRL3_C_SW_RESET,
                             LSM6DSOX_CTRL3_C_SW_RESET);
  if (ret != ESP_OK)
    {
      return ret;
    }

  /* The device clears SW_RESET itself once the reset
   * completes. Polling it is more reliable than a fixed
   * delay.
   */

  for (tries = 0; tries < LSM6DSOX_RESET_TRIES; tries++)
    {
      vTaskDelay(pdMS_TO_TICKS(LSM6DSOX_RESET_POLL_MS));

      ret = lsm6dsox_read_reg(LSM6DSOX_REG_CTRL3_C,
                               &reg);
      if (ret != ESP_OK)
        {
          return ret;
        }

      if ((reg & LSM6DSOX_CTRL3_C_SW_RESET) == 0)
        {
          return ESP_OK;
        }
    }

  ESP_LOGE(TAG, "Software reset did not complete");
  return ESP_ERR_TIMEOUT;
}

esp_err_t lsm6dsox_set_accel_range(
  lsm6dsox_accel_range_t range)
{
  esp_err_t ret;

  ret = lsm6dsox_modify_reg(LSM6DSOX_REG_CTRL1_XL,
                             LSM6DSOX_ACCEL_FS_MASK,
                             (uint8_t)range);
  if (ret == ESP_OK)
    {
      g_dev.accel_range = range;
      lsm6dsox_cache_accel_sens(range);
    }

  return ret;
}

esp_err_t lsm6dsox_set_gyro_range(
  lsm6dsox_gyro_range_t range)
{
  esp_err_t ret;

  ret = lsm6dsox_modify_reg(LSM6DSOX_REG_CTRL2_G,
                             LSM6DSOX_GYRO_FS_MASK,
                             (uint8_t)range);
  if (ret == ESP_OK)
    {
      g_dev.gyro_range = range;
      lsm6dsox_cache_gyro_sens(range);
    }

  return ret;
}

esp_err_t lsm6dsox_set_accel_odr(lsm6dsox_odr_t odr)
{
  esp_err_t ret;

  ret = lsm6dsox_modify_reg(LSM6DSOX_REG_CTRL1_XL,
                             LSM6DSOX_ODR_MASK,
                             (uint8_t)odr);
  if (ret == ESP_OK)
    {
      g_dev.accel_odr = odr;
      if (odr != LSM6DSOX_ODR_OFF)
        {
          g_dev.accel_odr_saved = odr;
        }
    }

  return ret;
}

esp_err_t lsm6dsox_set_gyro_odr(lsm6dsox_odr_t odr)
{
  esp_err_t ret;

  /* 1.6 Hz is an accelerometer-only low-power rate */

  if (odr == LSM6DSOX_ODR_1_6HZ)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ret = lsm6dsox_modify_reg(LSM6DSOX_REG_CTRL2_G,
                             LSM6DSOX_ODR_MASK,
                             (uint8_t)odr);
  if (ret == ESP_OK)
    {
      g_dev.gyro_odr = odr;
      if (odr != LSM6DSOX_ODR_OFF)
        {
          g_dev.gyro_odr_saved = odr;
        }
    }

  return ret;
}

esp_err_t lsm6dsox_set_power_mode(
  lsm6dsox_power_mode_t mode)
{
  /* TODO(L1): XL_HM_MODE (CTRL6_C bit 4) and G_HM_MODE
   * (CTRL7_G bit 7) select high-performance vs normal;
   * XL_ULP_EN (CTRL5_C bit 7) adds the accelerometer
   * ultra-low-power mode. Datasheet Section 5.3.
   */

  (void)mode;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_sleep(void)
{
  esp_err_t ret;

  ret = lsm6dsox_set_accel_odr(LSM6DSOX_ODR_OFF);
  if (ret != ESP_OK)
    {
      return ret;
    }

  return lsm6dsox_set_gyro_odr(LSM6DSOX_ODR_OFF);
}

esp_err_t lsm6dsox_wakeup(void)
{
  esp_err_t ret;

  ret = lsm6dsox_set_accel_odr(g_dev.accel_odr_saved);
  if (ret != ESP_OK)
    {
      return ret;
    }

  return lsm6dsox_set_gyro_odr(g_dev.gyro_odr_saved);
}

/**********************************************************
 * Public Functions — L2: Polled Data Acquisition
 **********************************************************/

esp_err_t lsm6dsox_get_status(uint8_t *status)
{
  return lsm6dsox_read_reg(LSM6DSOX_REG_STATUS_REG,
                            status);
}

esp_err_t lsm6dsox_read_raw(lsm6dsox_raw_data_t *data)
{
  esp_err_t ret;
  uint8_t   buf[14];

  if (data == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  /* 0x20..0x2D: temp(2) + gyro(6) + accel(6).
   * Output words are little-endian.
   */

  ret = lsm6dsox_read_burst(LSM6DSOX_REG_OUT_TEMP_L,
                             buf, sizeof(buf));
  if (ret != ESP_OK)
    {
      return ret;
    }

  data->temp_raw = (int16_t)((uint16_t)buf[1]  << 8 |
                              buf[0]);
  data->gyro_x   = (int16_t)((uint16_t)buf[3]  << 8 |
                              buf[2]);
  data->gyro_y   = (int16_t)((uint16_t)buf[5]  << 8 |
                              buf[4]);
  data->gyro_z   = (int16_t)((uint16_t)buf[7]  << 8 |
                              buf[6]);
  data->accel_x  = (int16_t)((uint16_t)buf[9]  << 8 |
                              buf[8]);
  data->accel_y  = (int16_t)((uint16_t)buf[11] << 8 |
                              buf[10]);
  data->accel_z  = (int16_t)((uint16_t)buf[13] << 8 |
                              buf[12]);

  data->gyro_x = (int16_t)(data->gyro_x -
                            g_dev.gyro_bias_x);
  data->gyro_y = (int16_t)(data->gyro_y -
                            g_dev.gyro_bias_y);
  data->gyro_z = (int16_t)(data->gyro_z -
                            g_dev.gyro_bias_z);

  return ESP_OK;
}

esp_err_t lsm6dsox_read_scaled(lsm6dsox_data_t *data)
{
  esp_err_t           ret;
  lsm6dsox_raw_data_t raw;

  if (data == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ret = lsm6dsox_read_raw(&raw);
  if (ret != ESP_OK)
    {
      return ret;
    }

  /* accel_sens is mg/LSB and gyro_sens is mdps/LSB, so
   * both need the milli- prefix divided out.
   */

  data->accel_x = (float)raw.accel_x *
                  g_dev.accel_sens / 1000.0f;
  data->accel_y = (float)raw.accel_y *
                  g_dev.accel_sens / 1000.0f;
  data->accel_z = (float)raw.accel_z *
                  g_dev.accel_sens / 1000.0f;
  data->gyro_x  = (float)raw.gyro_x *
                  g_dev.gyro_sens / 1000.0f;
  data->gyro_y  = (float)raw.gyro_y *
                  g_dev.gyro_sens / 1000.0f;
  data->gyro_z  = (float)raw.gyro_z *
                  g_dev.gyro_sens / 1000.0f;
  data->temp_c  = (float)raw.temp_raw /
                  LSM6DSOX_TEMP_SENSITIVITY +
                  LSM6DSOX_TEMP_OFFSET;

  return ESP_OK;
}

esp_err_t lsm6dsox_read_temperature(float *temp_c)
{
  esp_err_t ret;
  uint8_t   buf[2];
  int16_t   raw;

  if (temp_c == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ret = lsm6dsox_read_burst(LSM6DSOX_REG_OUT_TEMP_L,
                             buf, sizeof(buf));
  if (ret != ESP_OK)
    {
      return ret;
    }

  raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);

  *temp_c = (float)raw / LSM6DSOX_TEMP_SENSITIVITY +
            LSM6DSOX_TEMP_OFFSET;

  return ESP_OK;
}

/**********************************************************
 * Public Functions — L3: Data-Ready Interrupt
 **********************************************************/

esp_err_t lsm6dsox_set_drdy_int(lsm6dsox_int_pin_t pin,
                                 bool               accel_en,
                                 bool               gyro_en)
{
  esp_err_t ret;
  uint8_t   mask;
  uint8_t   val;

  mask = LSM6DSOX_INT1_DRDY_XL | LSM6DSOX_INT1_DRDY_G;
  val  = (uint8_t)((accel_en ? LSM6DSOX_INT1_DRDY_XL : 0) |
                   (gyro_en  ? LSM6DSOX_INT1_DRDY_G  : 0));

  if (pin & LSM6DSOX_INT_PIN_1)
    {
      ret = lsm6dsox_modify_reg(LSM6DSOX_REG_INT1_CTRL,
                                 mask, val);
      if (ret != ESP_OK)
        {
          return ret;
        }
    }

  if (pin & LSM6DSOX_INT_PIN_2)
    {
      ret = lsm6dsox_modify_reg(LSM6DSOX_REG_INT2_CTRL,
                                 mask, val);
      if (ret != ESP_OK)
        {
          return ret;
        }
    }

  return ESP_OK;
}

esp_err_t lsm6dsox_set_int_notification(bool active_low,
                                         bool open_drain,
                                         bool latched)
{
  esp_err_t ret;

  ret = lsm6dsox_modify_reg(
          LSM6DSOX_REG_CTRL3_C,
          LSM6DSOX_CTRL3_C_H_LACTIVE |
          LSM6DSOX_CTRL3_C_PP_OD,
          (uint8_t)((active_low ?
                     LSM6DSOX_CTRL3_C_H_LACTIVE : 0) |
                    (open_drain ?
                     LSM6DSOX_CTRL3_C_PP_OD : 0)));
  if (ret != ESP_OK)
    {
      return ret;
    }

  return lsm6dsox_modify_reg(
           LSM6DSOX_REG_TAP_CFG0,
           LSM6DSOX_TAP_CFG0_LIR,
           (uint8_t)(latched ?
                     LSM6DSOX_TAP_CFG0_LIR : 0));
}

esp_err_t lsm6dsox_get_all_int_src(uint8_t *src)
{
  return lsm6dsox_read_reg(LSM6DSOX_REG_ALL_INT_SRC,
                            src);
}

/**********************************************************
 * Public Functions — L4: Calibration and Self-Test
 **********************************************************/

esp_err_t lsm6dsox_calibrate(lsm6dsox_offsets_t *offsets)
{
  /* TODO(L4): average LSM6DSOX_CALIB_SAMPLES stationary
   * reads. Gyro mean goes into g_dev.gyro_bias_* (no
   * hardware gyro offset registers exist on this part).
   * Accel mean, minus 1 g on Z, converts to the
   * X/Y/Z_OFS_USR weight selected by USR_OFF_W
   * (CTRL6_C bit 3) and needs USR_OFF_ON_OUT
   * (CTRL7_G bit 1) set to take effect.
   * Datasheet Section 9.19.
   */

  (void)offsets;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_set_offsets(
  const lsm6dsox_offsets_t *offsets)
{
  /* TODO(L4): write X/Y/Z_OFS_USR (0x73..0x75) and copy
   * the gyro bias into g_dev.gyro_bias_*.
   */

  (void)offsets;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_self_test(void)
{
  /* TODO(L4): follow the sequence in Datasheet Section
   * 6.5 — configure ODR/FS, discard the first sample,
   * average 5, set ST_XL/ST_G in CTRL5_C, wait for
   * settling, average 5 more, and check the delta
   * against the datasheet min/max window.
   */

  return ESP_ERR_NOT_SUPPORTED;
}

/**********************************************************
 * Public Functions — L5: FIFO
 **********************************************************/

esp_err_t lsm6dsox_fifo_config(lsm6dsox_fifo_mode_t mode,
                                uint16_t             watermark)
{
  /* TODO(L5): watermark is 9 bits — low 8 into
   * FIFO_CTRL1, bit 8 into FIFO_CTRL2 bit 0. Mode goes
   * into FIFO_CTRL4 bits 2:0. Datasheet Section 9.5-9.8.
   */

  (void)mode;
  (void)watermark;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_fifo_set_batch_rate(
  lsm6dsox_odr_t accel_bdr,
  lsm6dsox_odr_t gyro_bdr)
{
  /* TODO(L5): FIFO_CTRL3 packs BDR_GY in bits 7:4 and
   * BDR_XL in bits 3:0. The BDR encoding matches the ODR
   * encoding shifted right by 4.
   */

  (void)accel_bdr;
  (void)gyro_bdr;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_fifo_get_level(uint16_t *samples,
                                   uint8_t  *flags)
{
  /* TODO(L5): burst-read FIFO_STATUS1/2 (0x3A) and
   * assemble DIFF_FIFO from status1 plus the low 2 bits
   * of status2.
   */

  (void)samples;
  (void)flags;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_fifo_read(
  lsm6dsox_fifo_sample_t *samples,
  uint16_t                max_samples,
  uint16_t               *read_count)
{
  /* TODO(L5): each record is 7 bytes from 0x78 — tag
   * byte then three little-endian words. The sensor id
   * is bits 7:3 of the tag.
   */

  (void)samples;
  (void)max_samples;
  (void)read_count;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_fifo_flush(void)
{
  /* TODO(L5): switch FIFO_CTRL4 to bypass and back to
   * g_dev.fifo_mode.
   */

  return ESP_ERR_NOT_SUPPORTED;
}

/**********************************************************
 * Public Functions — L6: Hardware Events
 **********************************************************/

esp_err_t lsm6dsox_route_hw_event(
  lsm6dsox_hw_event_t events,
  lsm6dsox_int_pin_t  pin)
{
  esp_err_t ret;

  if (pin & LSM6DSOX_INT_PIN_1)
    {
      ret = lsm6dsox_modify_reg(LSM6DSOX_REG_MD1_CFG,
                                 (uint8_t)events,
                                 (uint8_t)events);
      if (ret != ESP_OK)
        {
          return ret;
        }
    }

  if (pin & LSM6DSOX_INT_PIN_2)
    {
      ret = lsm6dsox_modify_reg(LSM6DSOX_REG_MD2_CFG,
                                 (uint8_t)events,
                                 (uint8_t)events);
      if (ret != ESP_OK)
        {
          return ret;
        }
    }

  /* Nothing reaches the pins until the global event
   * enable is set.
   */

  return lsm6dsox_modify_reg(
           LSM6DSOX_REG_TAP_CFG2,
           LSM6DSOX_TAP_CFG2_INTERRUPTS_EN,
           LSM6DSOX_TAP_CFG2_INTERRUPTS_EN);
}

esp_err_t lsm6dsox_config_wakeup(uint8_t threshold,
                                  uint8_t duration)
{
  /* TODO(L6): WK_THS is bits 5:0 of WAKE_UP_THS (0x5B);
   * WAKE_DUR is bits 6:5 of WAKE_UP_DUR (0x5C).
   * Datasheet Section 9.43-9.44.
   */

  (void)threshold;
  (void)duration;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_config_free_fall(uint8_t threshold,
                                     uint8_t duration)
{
  /* TODO(L6): FREE_FALL (0x5D) holds FF_THS in bits 2:0
   * and FF_DUR[4:0] in bits 7:3; the 6th duration bit is
   * FF_DUR5 in WAKE_UP_DUR bit 7.
   */

  (void)threshold;
  (void)duration;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_config_tap(uint8_t axis_mask,
                               uint8_t threshold,
                               bool    double_tap)
{
  /* TODO(L6): axis enables in TAP_CFG0 bits 3:1;
   * thresholds in TAP_CFG1[4:0], TAP_CFG2[4:0] and
   * TAP_THS_6D[4:0]; timing in INT_DUR2; double-tap via
   * SINGLE_DOUBLE_TAP in WAKE_UP_THS bit 7.
   * Datasheet Section 9.38-9.42.
   */

  (void)axis_mask;
  (void)threshold;
  (void)double_tap;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_config_6d(uint8_t threshold,
                              bool    four_d)
{
  /* TODO(L6): SIXD_THS is bits 6:5 and D4D_EN is bit 7
   * of TAP_THS_6D (0x59).
   */

  (void)threshold;
  (void)four_d;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_get_event_src(uint8_t *wake_src,
                                  uint8_t *tap_src,
                                  uint8_t *d6d_src)
{
  esp_err_t ret;
  uint8_t   buf[3];

  ret = lsm6dsox_read_burst(LSM6DSOX_REG_WAKE_UP_SRC,
                             buf, sizeof(buf));
  if (ret != ESP_OK)
    {
      return ret;
    }

  if (wake_src != NULL)
    {
      *wake_src = buf[0];
    }

  if (tap_src != NULL)
    {
      *tap_src = buf[1];
    }

  if (d6d_src != NULL)
    {
      *d6d_src = buf[2];
    }

  return ESP_OK;
}

/**********************************************************
 * Public Functions — L7: Embedded Functions
 **********************************************************/

esp_err_t lsm6dsox_pedometer_enable(bool enable)
{
  /* TODO(L7): set PEDO_EN in EMB_FUNC_EN_A and pulse
   * PEDO_INIT in EMB_FUNC_INIT_A, both in the EMB_FUNC
   * bank — use lsm6dsox_modify_banked(). The
   * accelerometer must run at 26 Hz or above.
   * Datasheet Section 10.3, AN5272 Section 6.
   */

  (void)enable;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_pedometer_read(uint16_t *steps)
{
  /* TODO(L7): burst-read STEP_COUNTER_L/H (0x62/0x63) in
   * the EMB_FUNC bank. Note 0x62 is I3C_BUS_AVB in the
   * user bank, so the bank switch is mandatory.
   */

  (void)steps;
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_pedometer_reset(void)
{
  /* TODO(L7): set PEDO_RST_STEP in EMB_FUNC_SRC (0x64,
   * EMB_FUNC bank).
   */

  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6dsox_tilt_enable(bool enable)
{
  return lsm6dsox_modify_banked(
           LSM6DSOX_BANK_EMBEDDED,
           LSM6DSOX_EMB_FUNC_EN_A,
           LSM6DSOX_EMB_EN_A_TILT,
           (uint8_t)(enable ?
                     LSM6DSOX_EMB_EN_A_TILT : 0));
}

esp_err_t lsm6dsox_significant_motion_enable(bool enable)
{
  return lsm6dsox_modify_banked(
           LSM6DSOX_BANK_EMBEDDED,
           LSM6DSOX_EMB_FUNC_EN_A,
           LSM6DSOX_EMB_EN_A_SIGN_MOTION,
           (uint8_t)(enable ?
                     LSM6DSOX_EMB_EN_A_SIGN_MOTION : 0));
}

esp_err_t lsm6dsox_get_emb_func_status(uint8_t *status)
{
  return lsm6dsox_read_reg(
           LSM6DSOX_REG_EMB_FUNC_STATUS_MP, status);
}

/**********************************************************
 * Public Functions — L8: MLC and FSM
 **********************************************************/

esp_err_t lsm6dsox_ucf_load(
  const lsm6dsox_ucf_line_t *ucf_data,
  size_t                     lines)
{
  esp_err_t ret;
  size_t    i;

  if (ucf_data == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  /* A .ucf stream performs its own bank switching by
   * writing FUNC_CFG_ACCESS, so the pairs must be
   * replayed verbatim and in order.
   */

  for (i = 0; i < lines; i++)
    {
      ret = lsm6dsox_write_reg(ucf_data[i].address,
                                ucf_data[i].data);
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG,
                   "UCF line %u (reg 0x%02x) failed: %s",
                   (unsigned)i, ucf_data[i].address,
                   esp_err_to_name(ret));
          return ret;
        }
    }

  ESP_LOGI(TAG, "UCF loaded: %u lines",
           (unsigned)lines);
  return ESP_OK;
}

esp_err_t lsm6dsox_mlc_get_status(uint8_t *status)
{
  return lsm6dsox_read_reg(LSM6DSOX_REG_MLC_STATUS_MP,
                            status);
}

esp_err_t lsm6dsox_mlc_get_output(uint8_t  mlc_num,
                                   uint8_t *class_result)
{
  if (mlc_num > 7 || class_result == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  return lsm6dsox_read_banked(
           LSM6DSOX_BANK_EMBEDDED,
           (uint8_t)(LSM6DSOX_EMB_MLC0_SRC + mlc_num),
           class_result);
}

esp_err_t lsm6dsox_fsm_get_status(uint16_t *status)
{
  esp_err_t ret;
  uint8_t   buf[2];

  if (status == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ret = lsm6dsox_read_burst(LSM6DSOX_REG_FSM_STATUS_A_MP,
                             buf, sizeof(buf));
  if (ret != ESP_OK)
    {
      return ret;
    }

  *status = (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
  return ESP_OK;
}

esp_err_t lsm6dsox_fsm_get_output(uint8_t  fsm_num,
                                   uint8_t *output)
{
  if (fsm_num > 15 || output == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  return lsm6dsox_read_banked(
           LSM6DSOX_BANK_EMBEDDED,
           (uint8_t)(LSM6DSOX_EMB_FSM_OUTS1 + fsm_num),
           output);
}

/**********************************************************
 * Public Functions — L9: Timestamp
 **********************************************************/

esp_err_t lsm6dsox_timestamp_enable(bool enable)
{
  return lsm6dsox_modify_reg(
           LSM6DSOX_REG_CTRL10_C,
           LSM6DSOX_CTRL10_C_TIMESTAMP_EN,
           (uint8_t)(enable ?
                     LSM6DSOX_CTRL10_C_TIMESTAMP_EN : 0));
}

esp_err_t lsm6dsox_timestamp_read(uint32_t *timestamp)
{
  esp_err_t ret;
  uint8_t   buf[4];

  if (timestamp == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  ret = lsm6dsox_read_burst(LSM6DSOX_REG_TIMESTAMP0,
                             buf, sizeof(buf));
  if (ret != ESP_OK)
    {
      return ret;
    }

  *timestamp = ((uint32_t)buf[3] << 24) |
               ((uint32_t)buf[2] << 16) |
               ((uint32_t)buf[1] <<  8) |
                (uint32_t)buf[0];

  return ESP_OK;
}
