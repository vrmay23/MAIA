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
 * main/tests/test_mpu6050.c
 *
 * MPU-6050 IMU Driver Test Suite
 *
 * Tests (in order):
 *   1. WHO_AM_I         - device identity check
 *   2. Init             - driver initialization
 *   3. Self-test        - factory self-test
 *   4. Read raw         - raw register burst read
 *   5. Read scaled      - physical unit conversion
 *   6. Calibration      - gyro bias offset computation
 *   7. Range change     - runtime range switching
 *   8. Sleep / wakeup   - power management
 *   9. DMP quaternion   - orientation output (10 samples)
 *  10. Euler angles     - quaternion -> roll/pitch/yaw
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "tests.h"
#include "mpu6050.h"
#include "maia_board.h"
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG                 "[TEST_MPU6050]"
#define PASS                "PASS"
#define FAIL                "FAIL"
#define SEPARATOR \
  "============================================================"

/* Number of DMP quaternion samples to collect */

#define DMP_SAMPLES         10

/* DMP sample timeout in milliseconds */

#define DMP_TIMEOUT_MS      500

/* Delay between scaled reads (ms) */

#define READ_DELAY_MS       100

/* Number of scaled read samples to print */

#define SCALED_SAMPLES      5

/* Test task stack size — I2C new master requires dedicated task */

#define TEST_TASK_STACK     8192

/* Test task priority */

#define TEST_TASK_PRIORITY  5

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void print_header(const char *title)
{
  ESP_LOGI(TAG, "%s", SEPARATOR);
  ESP_LOGI(TAG, " %s", title);
  ESP_LOGI(TAG, "%s", SEPARATOR);
}

static void print_result(const char *test, bool ok)
{
  if (ok)
    {
      ESP_LOGI(TAG, "[%s] %s", PASS, test);
    }
  else
    {
      ESP_LOGE(TAG, "[%s] %s", FAIL, test);
    }
}

/****************************************************************************
 * Name: test_who_am_i
 ****************************************************************************/

static bool test_who_am_i(void)
{
  esp_err_t ret;
  uint8_t   val;

  ret = mpu6050_who_am_i(&val);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(ret));
      return false;
    }

  ESP_LOGI(TAG, "WHO_AM_I = 0x%02x (expected 0x%02x)",
           val, MPU6050_WHO_AM_I_VAL);

  return (val == MPU6050_WHO_AM_I_VAL);
}

/****************************************************************************
 * Name: test_init
 ****************************************************************************/

static bool test_init(bool dmp_enable)
{
  esp_err_t        ret;
  mpu6050_config_t cfg;

  cfg.i2c_addr    = MAIA_I2C_ADDR_MPU6050;
  cfg.accel_range = MPU6050_ACCEL_RANGE_2G;
  cfg.gyro_range  = MPU6050_GYRO_RANGE_250;
  cfg.sample_rate = 7;  /* 1kHz / (7+1) = 125 Hz */
  cfg.dmp_enable  = dmp_enable;
  cfg.int_enable  = true;
  cfg.isr_cb      = NULL;
  cfg.isr_arg     = NULL;

  ret = mpu6050_init(&cfg);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    }

  return (ret == ESP_OK);
}

/****************************************************************************
 * Name: test_self_test
 ****************************************************************************/

static bool test_self_test(void)
{
  esp_err_t ret = mpu6050_self_test();
  return (ret == ESP_OK);
}

/****************************************************************************
 * Name: test_read_raw
 ****************************************************************************/

static bool test_read_raw(void)
{
  esp_err_t          ret;
  mpu6050_raw_data_t raw;

  ret = mpu6050_read_raw(&raw);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Read raw failed: %s", esp_err_to_name(ret));
      return false;
    }

  ESP_LOGI(TAG,
           "Raw: AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d T=%d",
           raw.accel_x, raw.accel_y, raw.accel_z,
           raw.gyro_x,  raw.gyro_y,  raw.gyro_z,
           raw.temp_raw);

  if (raw.accel_x == 0 && raw.accel_y == 0 && raw.accel_z == 0 &&
      raw.gyro_x  == 0 && raw.gyro_y  == 0 && raw.gyro_z  == 0)
    {
      ESP_LOGW(TAG, "All raw values are zero — check device");
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: test_read_scaled
 ****************************************************************************/

static bool test_read_scaled(void)
{
  esp_err_t      ret;
  mpu6050_data_t data;
  int            i;
  float          total_g;

  for (i = 0; i < SCALED_SAMPLES; i++)
    {
      ret = mpu6050_read_scaled(&data);
      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "Read scaled failed: %s",
                   esp_err_to_name(ret));
          return false;
        }

      ESP_LOGI(TAG,
               "[%d] A[g]: x=%.3f y=%.3f z=%.3f | "
               "G[d/s]: x=%.2f y=%.2f z=%.2f | T=%.1fC",
               i,
               data.accel_x, data.accel_y, data.accel_z,
               data.gyro_x,  data.gyro_y,  data.gyro_z,
               data.temp_c);

      vTaskDelay(pdMS_TO_TICKS(READ_DELAY_MS));
    }

  total_g = sqrtf(data.accel_x * data.accel_x +
                  data.accel_y * data.accel_y +
                  data.accel_z * data.accel_z);

  ESP_LOGI(TAG, "Total accel magnitude: %.3f g (expect ~1.0)",
           total_g);

  return (total_g > 0.5f && total_g < 2.0f);
}

/****************************************************************************
 * Name: test_calibration
 ****************************************************************************/

static bool test_calibration(void)
{
  esp_err_t         ret;
  mpu6050_offsets_t off;

  ESP_LOGI(TAG, "Keep device flat and still for calibration...");
  vTaskDelay(pdMS_TO_TICKS(1000));

  ret = mpu6050_calibrate(&off);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(ret));
      return false;
    }

  ESP_LOGI(TAG, "Bias: GX=%d GY=%d GZ=%d",
           off.gyro_x, off.gyro_y, off.gyro_z);

  return true;
}

/****************************************************************************
 * Name: test_range_change
 ****************************************************************************/

static bool test_range_change(void)
{
  esp_err_t      ret;
  mpu6050_data_t data;

  ret = mpu6050_set_accel_range(MPU6050_ACCEL_RANGE_8G);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Accel range change failed");
      return false;
    }

  ret = mpu6050_set_gyro_range(MPU6050_GYRO_RANGE_500);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Gyro range change failed");
      return false;
    }

  ESP_LOGI(TAG, "Ranges changed: accel=+-8g gyro=+-500dps");

  ret = mpu6050_read_scaled(&data);
  if (ret != ESP_OK)
    {
      return false;
    }

  ESP_LOGI(TAG, "Post-range A[g]: x=%.3f y=%.3f z=%.3f",
           data.accel_x, data.accel_y, data.accel_z);

  mpu6050_set_accel_range(MPU6050_ACCEL_RANGE_2G);
  mpu6050_set_gyro_range(MPU6050_GYRO_RANGE_250);

  return true;
}

/****************************************************************************
 * Name: test_power_management
 ****************************************************************************/

static bool test_power_management(void)
{
  esp_err_t          ret;
  mpu6050_raw_data_t raw;

  ret = mpu6050_sleep();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Sleep failed");
      return false;
    }

  ESP_LOGI(TAG, "Device in sleep mode");
  vTaskDelay(pdMS_TO_TICKS(200));

  ret = mpu6050_wakeup();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Wakeup failed");
      return false;
    }

  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Device awake");

  ret = mpu6050_read_raw(&raw);
  return (ret == ESP_OK);
}

/****************************************************************************
 * Name: test_dmp_quaternion
 ****************************************************************************/

static bool test_dmp_quaternion(void)
{
  esp_err_t            ret;
  mpu6050_quaternion_t quat;
  mpu6050_euler_t      euler;
  float                norm;
  int                  i;

  ESP_LOGI(TAG, "Reading %d DMP quaternion samples:", DMP_SAMPLES);

  for (i = 0; i < DMP_SAMPLES; i++)
    {
      ret = mpu6050_read_dmp(&quat, DMP_TIMEOUT_MS);
      if (ret == ESP_ERR_TIMEOUT)
        {
          ESP_LOGW(TAG, "[%d] DMP timeout", i);
          continue;
        }

      if (ret != ESP_OK)
        {
          ESP_LOGE(TAG, "[%d] DMP read failed: %s",
                   i, esp_err_to_name(ret));
          return false;
        }

      mpu6050_quaternion_to_euler(&quat, &euler);

      norm = sqrtf(quat.w * quat.w + quat.x * quat.x +
                   quat.y * quat.y + quat.z * quat.z);

      ESP_LOGI(TAG,
               "[%d] Q(w=%.4f x=%.4f y=%.4f z=%.4f) "
               "|q|=%.4f R=%.1f P=%.1f Y=%.1f",
               i,
               quat.w, quat.x, quat.y, quat.z, norm,
               euler.roll, euler.pitch, euler.yaw);

      if (norm < 0.9f || norm > 1.1f)
        {
          ESP_LOGE(TAG, "Quaternion norm out of range: %.4f", norm);
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: mpu6050_test_task
 *
 * Description:
 *   Dedicated FreeRTOS task for MPU-6050 test suite.
 *   ESP-IDF new I2C master driver requires I2C transactions to run
 *   from a task context (not app_main directly) due to internal
 *   queue/semaphore synchronization with the I2C ISR.
 *
 ****************************************************************************/

static void mpu6050_test_task(void *pvParameters)
{
  int  pass  = 0;
  int  total = 0;
  bool ok;

  print_header("MPU-6050 IMU DRIVER TEST SUITE");
  print_header("Phase 1: Basic (no DMP)");

  ok = test_init(false);
  print_result("Init (no DMP)", ok);
  total++; if (ok) pass++;

  if (!ok)
    {
      ESP_LOGE(TAG, "Init failed — aborting test suite");
      goto results;
    }

  ok = test_who_am_i();
  print_result("WHO_AM_I", ok);
  total++; if (ok) pass++;

  ok = test_self_test();
  print_result("Self-test", ok);
  total++; if (ok) pass++;

  ok = test_read_raw();
  print_result("Read raw", ok);
  total++; if (ok) pass++;

  ok = test_read_scaled();
  print_result("Read scaled", ok);
  total++; if (ok) pass++;

  ok = test_calibration();
  print_result("Gyro calibration", ok);
  total++; if (ok) pass++;

  ok = test_range_change();
  print_result("Runtime range change", ok);
  total++; if (ok) pass++;

  ok = test_power_management();
  print_result("Sleep / wakeup", ok);
  total++; if (ok) pass++;

  print_header("Phase 2: DMP quaternion");

  mpu6050_deinit();
  vTaskDelay(pdMS_TO_TICKS(100));

  ok = test_init(true);
  print_result("Init (with DMP)", ok);
  total++; if (ok) pass++;

  if (ok)
    {
      ok = test_dmp_quaternion();
      print_result("DMP quaternion + Euler", ok);
      total++; if (ok) pass++;
    }
  else
    {
      ESP_LOGW(TAG, "Skipping DMP test — init failed");
      total++;
    }

results:
  print_header("RESULTS");
  ESP_LOGI(TAG, "Passed: %d / %d", pass, total);

  if (pass == total)
    {
      ESP_LOGI(TAG, "ALL TESTS PASSED");
    }
  else
    {
      ESP_LOGE(TAG, "%d TEST(S) FAILED", total - pass);
    }

  print_header("END");

  mpu6050_deinit();
  vTaskDelete(NULL);
}

void test_mpu6050_run(void)
{
  xTaskCreatePinnedToCore(mpu6050_test_task,
                           "mpu6050_test",
                           TEST_TASK_STACK,
                           NULL,
                           TEST_TASK_PRIORITY,
                           NULL,
                           APP_CPU_NUM);

  while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
}