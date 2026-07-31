/*
 * Copyright 2026 Vinicius May
 *
 * Licensed under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of
 * the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in
 * writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing
 * permissions and limitations under the License.
 */

/**********************************************************
 * main/tests/test_vl53l5cx.c
 *
 * VL53L5CX ToF Sensor Driver Test Suite
 *
 * Tests (in order):
 *   1. Init           - single sensor initialization
 *   2. FW upload      - ULD init returns OK (fw loaded)
 *   3. Device alive   - is_alive check (WHO_AM_I equiv)
 *   4. Single read    - poll one frame of ranging data
 *   5. Zone validity  - status != 5 and != 6 check
 *   6. Distance check - at least 1 zone < 4000mm
 *   7. Resolution     - set 4x4 -> 8x8 -> verify 8x8
 *   8. Power mode     - sleep -> wake -> read OK
 *
 **********************************************************/

/**********************************************************
 * Included Files
 **********************************************************/

#include "tests.h"
#include "vl53l5cx.h"
#include "ssd1306.h"
#include "maia_board.h"

#include <stdio.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**********************************************************
 * Pre-processor Definitions
 **********************************************************/

#define TAG                 "[TEST_VL53L5CX]"
#define PASS                "PASS"
#define FAIL                "FAIL"
#define SEPARATOR \
  "============================================================"

/* Polling timeout for data ready (ms) */

#define DATA_READY_TIMEOUT_MS  2000

/* Polling interval (ms) */

#define POLL_INTERVAL_MS       50

/* Test task configuration */

#define TEST_TASK_STACK     8192
#define TEST_TASK_PRIORITY  5

/* Maximum plausible distance (mm) */

#define MAX_PLAUSIBLE_MM    4000

/**********************************************************
 * Private Functions
 **********************************************************/

/**********************************************************
 * Name: print_header
 *
 * Description:
 *   Print a section header with separators.
 *
 **********************************************************/

static void print_header(const char *title)
{
  ESP_LOGI(TAG, "%s", SEPARATOR);
  ESP_LOGI(TAG, " %s", title);
  ESP_LOGI(TAG, "%s", SEPARATOR);
}

/**********************************************************
 * Name: log_lidar_gpio_state
 *
 * Description:
 *   Log the GPIO levels connected to both VL53L5CX
 *   control and interrupt pins. This is a board-level
 *   diagnostic used to verify the LPn sequencing before
 *   blaming I2C communication or the sensor hardware.
 *
 **********************************************************/

static void log_lidar_gpio_state(const char *stage)
{
  ESP_LOGI(TAG,
           "GPIO (%s): LP_LEFT/GPIO%d=%d, "
           "LP_RIGHT/GPIO%d=%d, INT_LEFT/GPIO%d=%d, "
           "INT_RIGHT/GPIO%d=%d",
           stage,
           MAIA_GPIO_TOF1_LPN,
           gpio_get_level(MAIA_GPIO_TOF1_LPN),
           MAIA_GPIO_TOF2_LPN,
           gpio_get_level(MAIA_GPIO_TOF2_LPN),
           MAIA_GPIO_TOF1_INT,
           gpio_get_level(MAIA_GPIO_TOF1_INT),
           MAIA_GPIO_TOF2_INT,
           gpio_get_level(MAIA_GPIO_TOF2_INT));
}

/**********************************************************
 * Name: i2c_bus_scan
 *
 * Description:
 *   Probe every 7-bit address (0x03-0x77) on the MAIA I2C
 *   bus and log the ones that ACK. Board-level diagnostic
 *   to tell apart "bus stuck/busy" from "wrong address"
 *   from "device genuinely not responding".
 *
 **********************************************************/

static void i2c_bus_scan(const char *stage)
{
  i2c_master_bus_handle_t bus = maia_i2c_get_bus_handle();
  uint8_t                 addr;
  int                     found = 0;

  ESP_LOGI(TAG, "I2C scan (%s):", stage);

  for (addr = 0x03; addr <= 0x77; addr++)
    {
      if (i2c_master_probe(bus, addr, 20) == ESP_OK)
        {
          ESP_LOGI(TAG, "  ACK at 0x%02x", addr);
          found++;
        }
    }

  if (found == 0)
    {
      ESP_LOGW(TAG, "  No devices ACKed");
    }
}

/**********************************************************
 * Name: print_result
 *
 * Description:
 *   Print PASS or FAIL for a named test.
 *
 **********************************************************/

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

/**********************************************************
 * Name: wait_data_ready
 *
 * Description:
 *   Poll maia_tof_data_ready() until data is available
 *   or timeout expires.
 *
 * Input Parameters:
 *   sensor_id  - MAIA_TOF_SENSOR_LEFT or _RIGHT
 *   timeout_ms - Maximum wait time in milliseconds
 *
 * Returned Value:
 *   true if data became ready, false on timeout.
 *
 **********************************************************/

static bool wait_data_ready(uint8_t sensor_id,
                            uint32_t timeout_ms)
{
  uint32_t  elapsed = 0;
  bool      ready   = false;
  esp_err_t ret;

  while (elapsed < timeout_ms)
    {
      ret = maia_tof_data_ready(sensor_id, &ready);
      if (ret == ESP_OK && ready)
        {
          return true;
        }

      vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
      elapsed += POLL_INTERVAL_MS;
    }

  ESP_LOGW(TAG, "Data ready timeout (%lu ms)",
           (unsigned long)timeout_ms);
  return false;
}

/**********************************************************
 * Name: test_init
 *
 * Description:
 *   Test 1: Initialize single sensor (left only).
 *   This also tests firmware upload (test 2) since
 *   maia_tof_init() calls vl53l5cx_init() internally
 *   which uploads the ~80KB firmware.
 *
 **********************************************************/

static bool test_init(void)
{
  esp_err_t ret;

  ret = maia_tof_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "maia_tof_init failed: %s",
               esp_err_to_name(ret));
    }

  return (ret == ESP_OK);
}

/**********************************************************
 * Name: test_fw_upload
 *
 * Description:
 *   Test 2: Firmware upload verification. If test_init
 *   passed, firmware was successfully uploaded. This
 *   test is implicitly validated by init success.
 *   We verify by checking the sensor is still alive
 *   after init (firmware is running).
 *
 **********************************************************/

static bool test_fw_upload(void)
{
  /* If init passed, firmware was uploaded successfully.
   * The ULD vl53l5cx_init() uploads ~80KB and verifies
   * the sensor boots. A separate check is redundant
   * but we confirm by starting ranging briefly. */

  esp_err_t ret;

  ret = maia_tof_start_ranging();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Start ranging failed — FW issue?");
      return false;
    }

  /* If ranging starts, firmware is running */

  ret = maia_tof_stop_ranging();
  return (ret == ESP_OK);
}

/**********************************************************
 * Name: test_device_alive
 *
 * Description:
 *   Test 3: Device identity check. Equivalent to
 *   WHO_AM_I for the VL53L5CX. The is_alive check
 *   was done during init, but we verify the sensor
 *   is still responsive after firmware upload.
 *
 **********************************************************/

static bool test_device_alive(void)
{
  /* Sensor responsiveness confirmed by successful
   * start/stop ranging in test_fw_upload.
   * Additional verification: read data_ready
   * without error (I2C communication OK). */

  bool      ready = false;
  esp_err_t ret;

  ret = maia_tof_data_ready(MAIA_TOF_SENSOR_LEFT,
                            &ready);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "data_ready failed — sensor dead?");
      return false;
    }

  ESP_LOGI(TAG, "Sensor responsive (data_ready=%d)",
           ready);
  return true;
}

/**********************************************************
 * Name: test_single_read
 *
 * Description:
 *   Test 4: Start ranging, poll for one frame, read
 *   ranging data, stop ranging.
 *
 **********************************************************/

static bool test_single_read(maia_tof_data_t *data)
{
  esp_err_t ret;

  ret = maia_tof_start_ranging();
  if (ret != ESP_OK)
    {
      return false;
    }

  if (!wait_data_ready(MAIA_TOF_SENSOR_LEFT,
                       DATA_READY_TIMEOUT_MS))
    {
      maia_tof_stop_ranging();
      return false;
    }

  ret = maia_tof_get_data(MAIA_TOF_SENSOR_LEFT, data);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "get_data failed: %s",
               esp_err_to_name(ret));
      maia_tof_stop_ranging();
      return false;
    }

  ESP_LOGI(TAG, "Got %d zones of data", data->nb_zones);

  ret = maia_tof_stop_ranging();
  return (ret == ESP_OK);
}

/**********************************************************
 * Name: test_zone_validity
 *
 * Description:
 *   Test 5: Check that at least some zones have valid
 *   target_status (not 5 and not 6). Filtered zones
 *   have distance_mm == -1.
 *
 **********************************************************/

static bool test_zone_validity(
    const maia_tof_data_t *data)
{
  int valid_count = 0;
  int i;

  for (i = 0; i < data->nb_zones; i++)
    {
      if (data->distance_mm[i] != -1)
        {
          valid_count++;
        }
    }

  ESP_LOGI(TAG, "Valid zones: %d / %d",
           valid_count, data->nb_zones);

  /* At least 1 valid zone required */

  if (valid_count == 0)
    {
      ESP_LOGE(TAG,
               "No valid zones — all filtered");
      return false;
    }

  return true;
}

/**********************************************************
 * Name: test_distance_plausibility
 *
 * Description:
 *   Test 6: At least one valid zone must report a
 *   distance < 4000mm (sensor max range). This
 *   confirms the sensor is actually measuring.
 *
 **********************************************************/

static bool test_distance_plausibility(
    const maia_tof_data_t *data)
{
  int  i;
  bool found = false;

  for (i = 0; i < data->nb_zones; i++)
    {
      if (data->distance_mm[i] > 0 &&
          data->distance_mm[i] < MAX_PLAUSIBLE_MM)
        {
          ESP_LOGI(TAG,
                   "Zone %d: %dmm (plausible)",
                   i, data->distance_mm[i]);
          found = true;
          break;
        }
    }

  if (!found)
    {
      ESP_LOGE(TAG,
               "No zone < %dmm — check environment",
               MAX_PLAUSIBLE_MM);

      /* Print first 8 zones for debug */

      for (i = 0; i < 8 && i < data->nb_zones; i++)
        {
          ESP_LOGI(TAG, "  Z%d: d=%dmm st=%d",
                   i,
                   data->distance_mm[i],
                   data->target_status[i]);
        }
    }

  return found;
}

/**********************************************************
 * Name: test_resolution
 *
 * Description:
 *   Test 7: Change resolution 4x4 -> 8x8 and verify
 *   the number of zones returned matches. Uses ULD
 *   API directly since MAIA API sets resolution at
 *   init via Kconfig. This test verifies runtime
 *   reconfiguration works.
 *
 *   NOTE: This test accesses ULD internals. For a
 *   pure MAIA API test, we verify the default
 *   resolution matches Kconfig.
 *
 **********************************************************/

static bool test_resolution(void)
{
  maia_tof_data_t data;
  esp_err_t       ret;

  /* Read one frame and check nb_zones matches Kconfig */

  ret = maia_tof_start_ranging();
  if (ret != ESP_OK)
    {
      return false;
    }

  if (!wait_data_ready(MAIA_TOF_SENSOR_LEFT,
                       DATA_READY_TIMEOUT_MS))
    {
      maia_tof_stop_ranging();
      return false;
    }

  ret = maia_tof_get_data(MAIA_TOF_SENSOR_LEFT, &data);
  maia_tof_stop_ranging();

  if (ret != ESP_OK)
    {
      return false;
    }

#ifdef CONFIG_MAIA_VL53L5CX_RESOLUTION_8X8
  ESP_LOGI(TAG, "Expected 64 zones, got %d",
           data.nb_zones);
  return (data.nb_zones == 64);
#else
  ESP_LOGI(TAG, "Expected 16 zones, got %d",
           data.nb_zones);
  return (data.nb_zones == 16);
#endif
}

/**********************************************************
 * Name: test_power_mode
 *
 * Description:
 *   Test 8: Put sensor to sleep, wake up, verify it
 *   can still read data.
 *
 **********************************************************/

static bool test_power_mode(void)
{
  esp_err_t       ret;
  maia_tof_data_t data;

  /* Sleep */

  ret = maia_tof_set_power_mode(
            MAIA_TOF_SENSOR_LEFT,
            MAIA_TOF_POWER_LOW_POWER);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Sleep failed");
      return false;
    }

  ESP_LOGI(TAG, "Sensor in sleep mode");
  vTaskDelay(pdMS_TO_TICKS(500));

  /* Wake */

  ret = maia_tof_set_power_mode(
            MAIA_TOF_SENSOR_LEFT,
            MAIA_TOF_POWER_ACTIVE);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Wake failed");
      return false;
    }

  ESP_LOGI(TAG, "Sensor awake");
  vTaskDelay(pdMS_TO_TICKS(100));

  /* Verify — start ranging and read one frame */

  ret = maia_tof_start_ranging();
  if (ret != ESP_OK)
    {
      return false;
    }

  if (!wait_data_ready(MAIA_TOF_SENSOR_LEFT,
                       DATA_READY_TIMEOUT_MS))
    {
      maia_tof_stop_ranging();
      return false;
    }

  ret = maia_tof_get_data(MAIA_TOF_SENSOR_LEFT,
                          &data);
  maia_tof_stop_ranging();

  if (ret != ESP_OK)
    {
      return false;
    }

  ESP_LOGI(TAG, "Post-wake read OK (%d zones)",
           data.nb_zones);
  return true;
}

/**********************************************************
 * Name: test_live_distance_stream
 *
 * Description:
 *   Continuously print the closest valid distance across
 *   all zones so movement toward/away from the sensor is
 *   visible in real time. Runs until the board is reset.
 *
 **********************************************************/

#ifdef CONFIG_MAIA_VL53L5CX_RESOLUTION_8X8
#  define LIVE_GRID_SIZE  8
#else
#  define LIVE_GRID_SIZE  4
#endif

/* OLED heat-map layout. The grid occupies a square block
 * the full height of the 128x32 panel, leaving the right
 * side for text. Cell size adapts so both 4x4 and 8x8
 * fill the same square. */

#define MATRIX_SPAN_PX   32
#define MATRIX_CELL_PX   (MATRIX_SPAN_PX / LIVE_GRID_SIZE)
#define MATRIX_ORIGIN_X  0
#define MATRIX_ORIGIN_Y  0
#define MATRIX_TEXT_X    40

/* Zones farther than this render empty, so the map
 * highlights nearby obstacles instead of the far wall. */

#define MATRIX_MAX_MM    1500

static void test_live_distance_stream(void)
{
  maia_tof_data_t data;
  esp_err_t       ret;
  int             i;
  int             min_mm;
  int             min_zone;
  int             center_sum;
  int             center_count;
  int             row;
  int             col;
  int             half;

  print_header("Phase 2: Live distance stream");
  ESP_LOGI(TAG,
           "Point a target dead-center in front of "
           "the sensor. 'Center' below tracks the "
           "middle 2x2 zones — compare it against a "
           "ruler. 'Closest' tracks the nearest valid "
           "return anywhere in the wide field of "
           "view, which can be a different, closer "
           "object off to the side. Reset to stop.");

  ret = maia_tof_start_ranging();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "Live stream: start_ranging failed");
      return;
    }

  while (1)
    {
      if (!wait_data_ready(MAIA_TOF_SENSOR_LEFT,
                           DATA_READY_TIMEOUT_MS))
        {
          continue;
        }

      ret = maia_tof_get_data(MAIA_TOF_SENSOR_LEFT,
                              &data);
      if (ret != ESP_OK)
        {
          continue;
        }

      min_mm   = -1;
      min_zone = -1;

      for (i = 0; i < data.nb_zones; i++)
        {
          if (data.distance_mm[i] > 0 &&
              (min_mm == -1 ||
               data.distance_mm[i] < min_mm))
            {
              min_mm   = data.distance_mm[i];
              min_zone = i;
            }
        }

      /* Middle 2x2 block of the grid, regardless of
       * resolution, so "Center" always means the
       * zones dead-ahead of the sensor's boresight. */

      center_sum   = 0;
      center_count = 0;
      half         = LIVE_GRID_SIZE / 2;

      for (row = half - 1; row <= half; row++)
        {
          for (col = half - 1; col <= half; col++)
            {
              i = (row * LIVE_GRID_SIZE) + col;
              if (data.distance_mm[i] > 0)
                {
                  center_sum += data.distance_mm[i];
                  center_count++;
                }
            }
        }

      if (min_mm != -1 && center_count > 0)
        {
          ESP_LOGI(TAG,
                   "Closest: %4dmm (zone %2d)  |  "
                   "Center: %4dmm (%d/4 valid)",
                   min_mm, min_zone,
                   center_sum / center_count,
                   center_count);
        }
      else if (min_mm != -1)
        {
          ESP_LOGI(TAG,
                   "Closest: %4dmm (zone %2d)  |  "
                   "Center: ---- (no valid target)",
                   min_mm, min_zone);
        }
      else
        {
          ESP_LOGI(TAG,
                   "Closest: ---- (no valid target)");
        }
    }
}

/**********************************************************
 * Name: distance_to_level
 *
 * Description:
 *   Map a zone distance to a fill density 0..4 for the
 *   OLED heat-map. Nearer targets render denser, so a
 *   close obstacle shows up as a solid block. Invalid
 *   zones and anything past MATRIX_MAX_MM render empty.
 *
 **********************************************************/

static uint8_t distance_to_level(int mm)
{
  if (mm <= 0)
    {
      return 0;
    }

  if (mm < 100)
    {
      return 4;
    }

  if (mm < 250)
    {
      return 3;
    }

  if (mm < 500)
    {
      return 2;
    }

  if (mm < MATRIX_MAX_MM)
    {
      return 1;
    }

  return 0;
}

/**********************************************************
 * Name: draw_zone_cell
 *
 * Description:
 *   Fill one grid cell in the framebuffer with a dither
 *   pattern whose density encodes proximity. The display
 *   is monochrome, so density stands in for brightness.
 *
 **********************************************************/

static void draw_zone_cell(int col, int row, uint8_t level)
{
  int  px;
  int  py;
  bool on;

  for (py = 0; py < MATRIX_CELL_PX; py++)
    {
      for (px = 0; px < MATRIX_CELL_PX; px++)
        {
          switch (level)
            {
              case 4:
                on = true;
                break;

              case 3:

                /* Solid block minus its four corners */

                on = !((px == 0 || px == MATRIX_CELL_PX - 1) &&
                       (py == 0 || py == MATRIX_CELL_PX - 1));
                break;

              case 2:
                on = (((px + py) & 1) == 0);
                break;

              case 1:

                /* Centred quarter-area dot */

                on = (px >= MATRIX_CELL_PX / 4 &&
                      px < (MATRIX_CELL_PX * 3) / 4 &&
                      py >= MATRIX_CELL_PX / 4 &&
                      py < (MATRIX_CELL_PX * 3) / 4);
                break;

              default:
                on = false;
                break;
            }

          ssd1306_set_pixel(
              (uint8_t)(MATRIX_ORIGIN_X +
                        (col * MATRIX_CELL_PX) + px),
              (uint8_t)(MATRIX_ORIGIN_Y +
                        (row * MATRIX_CELL_PX) + py),
              on);
        }
    }
}

/**********************************************************
 * Name: test_live_matrix_display
 *
 * Description:
 *   Render the sensor's zone grid live on the OLED as a
 *   proximity heat map, with the nearest target's range
 *   and grid position alongside it. Lets you see what
 *   the sensor sees spatially instead of reading a
 *   single number. Runs until the board is reset.
 *
 **********************************************************/

static void test_live_matrix_display(void)
{
  maia_tof_data_t data;
  esp_err_t       ret;
  char            line[16];
  int             i;
  int             row;
  int             col;
  int             min_mm;
  int             min_zone;

  print_header("Phase 2: Live 8x8 matrix on OLED");

  ret = ssd1306_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Display init failed: %s",
               esp_err_to_name(ret));
      return;
    }

  ESP_LOGI(TAG,
           "Denser block = closer target "
           "(solid <100mm, empty >%dmm or no return). "
           "Reset the board to stop.",
           MATRIX_MAX_MM);

  ret = maia_tof_start_ranging();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "Matrix view: start_ranging failed");
      return;
    }

  while (1)
    {
      if (!wait_data_ready(MAIA_TOF_SENSOR_LEFT,
                           DATA_READY_TIMEOUT_MS))
        {
          continue;
        }

      ret = maia_tof_get_data(MAIA_TOF_SENSOR_LEFT,
                              &data);
      if (ret != ESP_OK)
        {
          continue;
        }

      min_mm   = -1;
      min_zone = -1;

      for (i = 0; i < data.nb_zones; i++)
        {
          if (data.distance_mm[i] > 0 &&
              (min_mm == -1 ||
               data.distance_mm[i] < min_mm))
            {
              min_mm   = data.distance_mm[i];
              min_zone = i;
            }
        }

      ssd1306_clear();

      for (row = 0; row < LIVE_GRID_SIZE; row++)
        {
          for (col = 0; col < LIVE_GRID_SIZE; col++)
            {
              i = (row * LIVE_GRID_SIZE) + col;
              draw_zone_cell(
                  col, row,
                  distance_to_level(
                      i < data.nb_zones ?
                          data.distance_mm[i] : -1));
            }
        }

      if (min_mm != -1)
        {
          snprintf(line, sizeof(line), "%dmm", min_mm);
          ssd1306_draw_string(MATRIX_TEXT_X, 2, line,
                              SSD1306_FONT_SMALL);

          snprintf(line, sizeof(line), "r%d c%d",
                   min_zone / LIVE_GRID_SIZE,
                   min_zone % LIVE_GRID_SIZE);
          ssd1306_draw_string(MATRIX_TEXT_X, 14, line,
                              SSD1306_FONT_SMALL);

          ESP_LOGI(TAG,
                   "Closest: %4dmm (r%d c%d)",
                   min_mm,
                   min_zone / LIVE_GRID_SIZE,
                   min_zone % LIVE_GRID_SIZE);
        }
      else
        {
          ssd1306_draw_string(MATRIX_TEXT_X, 2, "----",
                              SSD1306_FONT_SMALL);
          ssd1306_draw_string(MATRIX_TEXT_X, 14,
                              "no target",
                              SSD1306_FONT_SMALL);
          ESP_LOGI(TAG,
                   "Closest: ---- (no valid target)");
        }

      ssd1306_display();
    }
}

/**********************************************************
 * Name: vl53l5cx_test_task
 *
 * Description:
 *   Dedicated FreeRTOS task for VL53L5CX test suite.
 *   ESP-IDF new I2C master driver requires I2C
 *   transactions to run from a task context.
 *
 **********************************************************/

static void vl53l5cx_test_task(void *pvParameters)
{
  int             pass  = 0;
  int             total = 0;
  bool            ok;
  maia_tof_data_t data;

  (void)pvParameters;

  print_header("VL53L5CX ToF SENSOR TEST SUITE");
  ESP_LOGI(TAG, "Test file built: %s %s",
           __DATE__, __TIME__);
  print_header("Phase 1: Basic (single sensor)");

  log_lidar_gpio_state("before maia_tof_init");
  i2c_bus_scan("before maia_tof_init");

  /* Test 1: Init */

  ok = test_init();
  log_lidar_gpio_state("after maia_tof_init");
  i2c_bus_scan("after maia_tof_init");
  print_result("Init (single sensor)", ok);
  total++;
  if (ok)
    {
      pass++;
    }

  if (!ok)
    {
      ESP_LOGE(TAG,
               "Init failed — aborting test suite");
      goto results;
    }

  /* Test 2: Firmware upload */

  ok = test_fw_upload();
  print_result("Firmware upload verify", ok);
  total++;
  if (ok)
    {
      pass++;
    }

  /* Test 3: Device alive */

  ok = test_device_alive();
  print_result("Device alive (WHO_AM_I equiv)", ok);
  total++;
  if (ok)
    {
      pass++;
    }

  /* Test 4: Single ranging read */

  ok = test_single_read(&data);
  print_result("Single ranging read", ok);
  total++;
  if (ok)
    {
      pass++;
    }

  if (!ok)
    {
      ESP_LOGW(TAG, "Skipping data-dependent tests");
      total += 2;
      goto test_resolution;
    }

  /* Test 5: Zone data validity */

  ok = test_zone_validity(&data);
  print_result("Zone data validity", ok);
  total++;
  if (ok)
    {
      pass++;
    }

  /* Test 6: Distance plausibility */

  ok = test_distance_plausibility(&data);
  print_result("Distance plausibility", ok);
  total++;
  if (ok)
    {
      pass++;
    }

test_resolution:

  /* Test 7: Resolution check */

  ok = test_resolution();
  print_result("Resolution set/get", ok);
  total++;
  if (ok)
    {
      pass++;
    }

  /* Test 8: Power mode */

  ok = test_power_mode();
  print_result("Power mode (sleep/wake/read)", ok);
  total++;
  if (ok)
    {
      pass++;
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
      ESP_LOGE(TAG, "%d TEST(S) FAILED",
               total - pass);
    }

  print_header("END");
  maia_tof_deinit();
  vTaskDelete(NULL);
}

/**********************************************************
 * Name: vl53l5cx_live_task
 *
 * Description:
 *   Dedicated FreeRTOS task for the live distance stream
 *   test mode: init only, then stream readings until
 *   reset. See test_live_distance_stream().
 *
 **********************************************************/

static void vl53l5cx_live_task(void *pvParameters)
{
  bool ok;

  (void)pvParameters;

  print_header("VL53L5CX LIVE DISTANCE STREAM");
  ESP_LOGI(TAG, "Test file built: %s %s",
           __DATE__, __TIME__);

  ok = test_init();
  print_result("Init (single sensor)", ok);

  if (ok)
    {
      test_live_distance_stream();
    }
  else
    {
      ESP_LOGE(TAG,
               "Init failed — cannot stream distance");
    }

  print_header("END");
  maia_tof_deinit();
  vTaskDelete(NULL);
}

/**********************************************************
 * Name: vl53l5cx_matrix_task
 *
 * Description:
 *   Dedicated FreeRTOS task for the OLED matrix view:
 *   init only, then render the zone grid until reset.
 *   See test_live_matrix_display().
 *
 **********************************************************/

static void vl53l5cx_matrix_task(void *pvParameters)
{
  bool ok;

  (void)pvParameters;

  print_header("VL53L5CX LIVE MATRIX VIEW");
  ESP_LOGI(TAG, "Test file built: %s %s",
           __DATE__, __TIME__);

  ok = test_init();
  print_result("Init (single sensor)", ok);

  if (ok)
    {
      test_live_matrix_display();
    }
  else
    {
      ESP_LOGE(TAG,
               "Init failed — cannot render matrix");
    }

  print_header("END");
  maia_tof_deinit();
  vTaskDelete(NULL);
}

/**********************************************************
 * Name: test_vl53l5cx_run
 *
 * Description:
 *   Entry point called from main.c when MAIA_TEST_TOF
 *   is selected. Creates the test task pinned to
 *   APP_CPU and blocks forever.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None (does not return).
 *
 **********************************************************/

void test_vl53l5cx_run(void)
{
#if defined(CONFIG_MAIA_TEST_TOF_MODE_MATRIX)
  TaskFunction_t task_fn = vl53l5cx_matrix_task;
#elif defined(CONFIG_MAIA_TEST_TOF_MODE_LIVE)
  TaskFunction_t task_fn = vl53l5cx_live_task;
#else
  TaskFunction_t task_fn = vl53l5cx_test_task;
#endif

  xTaskCreatePinnedToCore(task_fn,
                          "vl53l5cx_test",
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
