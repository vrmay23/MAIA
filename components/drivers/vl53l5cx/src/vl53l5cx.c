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
 * components/drivers/vl53l5cx/src/vl53l5cx.c
 *
 * MAIA VL53L5CX ToF Sensor Driver
 *
 * Manages dual VL53L5CX sensors with LPn-based address
 * reassignment. All configuration is Kconfig-driven.
 * Uses ST ULD API (vl53l5cx_api.h/c) for sensor control.
 *
 * Public API uses maia_tof_* prefix to avoid collision
 * with ST ULD vl53l5cx_* symbols.
 *
 * Platform I2C callbacks (WrByte/WrMulti/RdMulti/WaitMs)
 * are in vl53l5cx_platform.c via ESP-IDF v6.0 new I2C.
 *
 **********************************************************/

/**********************************************************
 * Included Files
 **********************************************************/

#include "platform.h"
#include "vl53l5cx_api.h"
#include "vl53l5cx.h"
#include "maia_board.h"
#include <string.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**********************************************************
 * Pre-processor Definitions
 **********************************************************/

#define TAG  "[VL53L5CX]"

#define VL53L5CX_DEFAULT_ADDR  UINT16_C(0x52)

/* The driver's primary/"LEFT" sensor slot normally
 * drives the physical LEFT sensor's GPIOs. In
 * single-sensor "right only" mode (Kconfig, e.g. when
 * the physical LEFT unit is known bad — a solder
 * defect, not a firmware issue — while RIGHT is fine),
 * it drives the physical RIGHT sensor's GPIOs instead,
 * still targeting TOF1_ADDR for consistency with the
 * other modes. */

#if defined(CONFIG_MAIA_VL53L5CX_MODE_RIGHT_ONLY)
#  define TOF1_LPN_GPIO  MAIA_GPIO_TOF2_LPN
#  define TOF1_INT_GPIO  MAIA_GPIO_TOF2_INT
#else
#  define TOF1_LPN_GPIO  MAIA_GPIO_TOF1_LPN
#  define TOF1_INT_GPIO  MAIA_GPIO_TOF1_INT
#endif

#define TOF2_LPN_GPIO  MAIA_GPIO_TOF2_LPN
#define TOF2_INT_GPIO  MAIA_GPIO_TOF2_INT

#define TOF1_ADDR  CONFIG_MAIA_VL53L5CX_LEFT_I2C_ADDR
#define TOF2_ADDR  CONFIG_MAIA_VL53L5CX_RIGHT_I2C_ADDR

#define RANGING_FREQ_HZ \
    CONFIG_MAIA_VL53L5CX_RANGING_FREQ_HZ
#define INTEGRATION_TIME_MS \
    CONFIG_MAIA_VL53L5CX_INTEGRATION_TIME_MS

#ifdef CONFIG_MAIA_VL53L5CX_RESOLUTION_4X4
#  define RESOLUTION  VL53L5CX_RESOLUTION_4X4
#  define NUM_ZONES   16
#  define GRID_SIZE   4
#else
#  define RESOLUTION  VL53L5CX_RESOLUTION_8X8
#  define NUM_ZONES   64
#  define GRID_SIZE   8
#endif

#ifdef CONFIG_MAIA_VL53L5CX_RANGING_MODE_CONTINUOUS
#  define RANGING_MODE \
     VL53L5CX_RANGING_MODE_CONTINUOUS
#else
#  define RANGING_MODE \
     VL53L5CX_RANGING_MODE_AUTONOMOUS
#endif

#ifdef CONFIG_MAIA_VL53L5CX_TARGET_ORDER_STRONGEST
#  define TARGET_ORDER \
     VL53L5CX_TARGET_ORDER_STRONGEST
#else
#  define TARGET_ORDER \
     VL53L5CX_TARGET_ORDER_CLOSEST
#endif

/* VL53L5CX target_status values referenced below. Per
 * UM2884 rev 3, ch. 5.5 "Results interpretation", table
 * 4 (also confirmed by ST's own comment in
 * managed_components/stm32-vl53l5cx/modules/
 * vl53l5cx_api.h: "5 & 9 means ranging OK"):
 *
 *   5   Range valid (100% confidence). Always kept.
 *   9   Range valid, large pulse: a real target with
 *       something else behind it in the same zone.
 *   6   Wrap-around check not performed: only happens
 *       on the first ranging after boot, since that
 *       check needs the following measurement.
 *
 * Every other value, including 255 (no target
 * detected), is rejected outright.
 */

#define STATUS_RANGE_VALID              5
#define STATUS_RANGE_VALID_LARGE_PULSE  9
#define STATUS_WRAPAROUND_SKIPPED       6
#define DISTANCE_FILTERED               (-1)

#define MIN_SIGNAL_PER_SPAD \
    CONFIG_MAIA_VL53L5CX_MIN_SIGNAL_PER_SPAD

/**********************************************************
 * Private Types
 **********************************************************/

/**********************************************************
 * Name: tof_sensor_t
 *
 * Description:
 *   Internal state for one VL53L5CX sensor.
 *   uld_cfg.platform holds dev_handle and address,
 *   filled by register_sensor().
 *
 **********************************************************/

typedef struct
{
  VL53L5CX_Configuration  uld_cfg;
  gpio_num_t              lpn_gpio;
  gpio_num_t              int_gpio;
  bool                    initialized;
} tof_sensor_t;

/**********************************************************
 * Private Data
 **********************************************************/

static tof_sensor_t g_sensors[MAIA_TOF_SENSOR_MAX];
static bool         g_driver_initialized = false;

/**********************************************************
 * Private Functions
 **********************************************************/

/**********************************************************
 * Name: lpn_set
 *
 * Description:
 *   Set LPn GPIO. HIGH = on, LOW = standby.
 *
 **********************************************************/

static void lpn_set(gpio_num_t gpio, uint32_t level)
{
  gpio_set_level(gpio, level);
}

/**********************************************************
 * Name: init_lpn_gpio
 *
 * Description:
 *   Configure LPn as push-pull output.
 *
 **********************************************************/

static esp_err_t init_lpn_gpio(gpio_num_t gpio)
{
  gpio_config_t cfg;

  memset(&cfg, 0, sizeof(cfg));
  cfg.pin_bit_mask = (1ULL << (uint32_t)gpio);
  cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
  cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type    = GPIO_INTR_DISABLE;

  return gpio_config(&cfg);
}

/**********************************************************
 * Name: init_int_gpio
 *
 * Description:
 *   Configure INT as input with pull-up (active LOW).
 *
 **********************************************************/

static esp_err_t init_int_gpio(gpio_num_t gpio)
{
  gpio_config_t cfg;

  memset(&cfg, 0, sizeof(cfg));
  cfg.pin_bit_mask = (1ULL << (uint32_t)gpio);
  cfg.mode         = GPIO_MODE_INPUT;
  cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type    = GPIO_INTR_DISABLE;

  return gpio_config(&cfg);
}

/**********************************************************
 * Name: register_sensor
 *
 * Description:
 *   Add sensor to I2C bus and fill VL53L5CX_Platform.
 *   memset dev_cfg mandatory for ESP-IDF v6.0 hidden
 *   internal fields.
 *
 **********************************************************/

static esp_err_t register_sensor(
    VL53L5CX_Platform *p_platform,
    uint16_t address)
{
  i2c_master_bus_handle_t bus;
  i2c_device_config_t     dev_cfg;
  esp_err_t               ret;

  bus = maia_i2c_get_bus_handle();

  memset(&dev_cfg, 0, sizeof(dev_cfg));
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;

  /* address is ST's 8-bit wire convention; ESP-IDF's
   * new I2C master driver wants the plain 7-bit
   * address for I2C_ADDR_BIT_LEN_7. */

  dev_cfg.device_address  = (uint16_t)(address >> 1);
  dev_cfg.scl_speed_hz    = MAIA_I2C_FREQ_HZ;

  /* VL53L5CX's internal MCU stretches SCL while
   * servicing a real register access (unlike a bare
   * i2c_master_probe(), which never touches a register
   * and hardcodes a generous 20ms stretch tolerance).
   * ESP-IDF's per-device default is only 2ms
   * (I2C_LL_SCL_WAIT_US_VAL_DEFAULT on ESP32-S3), too
   * tight for this sensor. Match the probe's margin. */

  dev_cfg.scl_wait_us     = 20000;

  ret = i2c_master_bus_add_device(
            bus, &dev_cfg,
            &p_platform->dev_handle);

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG,
               "I2C add 0x%02x failed: %s",
               address, esp_err_to_name(ret));
      return ret;
    }

  p_platform->address = address;
  return ESP_OK;
}

/**********************************************************
 * Name: unregister_sensor
 *
 * Description:
 *   Remove sensor from I2C bus.
 *
 **********************************************************/

static void unregister_sensor(
    VL53L5CX_Platform *p_platform)
{
  if (p_platform->dev_handle != NULL)
    {
      i2c_master_bus_rm_device(
          p_platform->dev_handle);
      p_platform->dev_handle = NULL;
    }
}

/**********************************************************
 * Name: configure_sensor
 *
 * Description:
 *   Apply Kconfig: resolution, freq, integration,
 *   ranging mode, target order.
 *
 **********************************************************/

static esp_err_t configure_sensor(
    VL53L5CX_Configuration *p_cfg)
{
  uint8_t status = 0;

  status |= vl53l5cx_set_resolution(
                p_cfg, RESOLUTION);
  status |= vl53l5cx_set_ranging_frequency_hz(
                p_cfg, RANGING_FREQ_HZ);
  status |= vl53l5cx_set_integration_time_ms(
                p_cfg, INTEGRATION_TIME_MS);
  status |= vl53l5cx_set_ranging_mode(
                p_cfg, RANGING_MODE);
  status |= vl53l5cx_set_target_order(
                p_cfg, TARGET_ORDER);

  if (status != 0)
    {
      ESP_LOGE(TAG, "Sensor config failed");
      return ESP_FAIL;
    }

  return ESP_OK;
}

/**********************************************************
 * Name: init_single_sensor
 *
 * Description:
 *   Full init for one sensor:
 *   1. LPn HIGH (enable)
 *   2. Register on I2C at boot_addr
 *   3. Verify alive
 *   4. Upload ULD firmware (~80KB)
 *   5. Reassign to target_addr if different
 *   6. Apply Kconfig configuration
 *
 **********************************************************/

static esp_err_t init_single_sensor(
    tof_sensor_t *sensor,
    uint16_t boot_addr,
    uint16_t target_addr)
{
  esp_err_t ret;
  uint8_t   status;
  uint8_t   alive = 0;

  /* Community VL53L5CX multi-sensor designs cite 100ms
   * as the minimum reliable wait after LPn goes high
   * before the first I2C access; 200ms keeps a 2x safety
   * margin without adding meaningful boot latency. */

  lpn_set(sensor->lpn_gpio, 1);
  vTaskDelay(pdMS_TO_TICKS(200));

  ret = register_sensor(
            &sensor->uld_cfg.platform, boot_addr);
  if (ret != ESP_OK)
    {
      return ret;
    }

  status = vl53l5cx_is_alive(
               &sensor->uld_cfg, &alive);
  if (status != 0 || alive == 0)
    {
      ESP_LOGE(TAG, "0x%02x not alive", boot_addr);
      unregister_sensor(&sensor->uld_cfg.platform);
      return ESP_FAIL;
    }

  ESP_LOGI(TAG, "Alive at 0x%02x", boot_addr);
  ESP_LOGI(TAG, "Uploading FW to 0x%02x...",
           boot_addr);

  status = vl53l5cx_init(&sensor->uld_cfg);
  if (status != 0)
    {
      ESP_LOGE(TAG, "ULD init failed 0x%02x",
               boot_addr);
      unregister_sensor(&sensor->uld_cfg.platform);
      return ESP_FAIL;
    }

  ESP_LOGI(TAG, "FW uploaded at 0x%02x", boot_addr);

  if (target_addr != boot_addr)
    {
      /* target_addr is already 8-bit wire format,
       * same convention as boot_addr.
       *
       * NOTE: vl53l5cx_set_i2c_address() issues 3
       * WrByte calls (select bank, write new address,
       * restore bank). The sensor starts responding at
       * the NEW address as soon as the 2nd call lands,
       * but our dev_handle is still bound to boot_addr,
       * so the 3rd call always NACKs — a deterministic
       * false failure, not a real one. So we don't trust
       * this status; we re-register at target_addr and
       * confirm with a fresh is_alive() there instead. */

      status = vl53l5cx_set_i2c_address(
                   &sensor->uld_cfg,
                   target_addr);
      if (status != 0)
        {
          ESP_LOGW(TAG,
                   "Addr 0x%02x->0x%02x reported "
                   "failure (may be a false "
                   "negative); verifying at 0x%02x",
                   boot_addr, target_addr,
                   target_addr);
        }

      unregister_sensor(&sensor->uld_cfg.platform);

      ret = register_sensor(
                &sensor->uld_cfg.platform,
                target_addr);
      if (ret != ESP_OK)
        {
          return ret;
        }

      status = vl53l5cx_is_alive(
                   &sensor->uld_cfg, &alive);
      if (status != 0 || alive == 0)
        {
          ESP_LOGE(TAG,
                   "Addr 0x%02x->0x%02x truly failed",
                   boot_addr, target_addr);
          unregister_sensor(
              &sensor->uld_cfg.platform);
          return ESP_FAIL;
        }

      ESP_LOGI(TAG, "Reassigned to 0x%02x",
               target_addr);
    }

  ret = configure_sensor(&sensor->uld_cfg);
  if (ret != ESP_OK)
    {
      return ret;
    }

  sensor->initialized = true;

  ESP_LOGI(TAG,
           "Sensor 0x%02x: %dx%d %dHz %dms",
           target_addr, GRID_SIZE, GRID_SIZE,
           RANGING_FREQ_HZ, INTEGRATION_TIME_MS);

  /* Per-frame I2C result transfer size for the currently
   * enabled output set (see include/platform.h). Logged
   * here so it stays a visible, runtime-checkable number
   * whenever that set changes. */

  ESP_LOGI(TAG, "I2C result block: %u bytes",
           (unsigned int)VL53L5CX_MAX_RESULTS_SIZE);

  return ESP_OK;
}

/**********************************************************
 * Name: filter_data
 *
 * Description:
 *   Mark filtered zones with DISTANCE_FILTERED.
 *
 **********************************************************/

static void filter_data(maia_tof_data_t *data)
{
  int     i;
  uint8_t st;
  bool    accept;

  for (i = 0; i < data->nb_zones; i++)
    {
      st     = data->target_status[i];
      accept = (st == STATUS_RANGE_VALID);

#ifdef CONFIG_MAIA_VL53L5CX_ACCEPT_STATUS_9
      accept = accept ||
               (st == STATUS_RANGE_VALID_LARGE_PULSE);
#endif

#ifdef CONFIG_MAIA_VL53L5CX_ACCEPT_STATUS_6
      accept = accept ||
               (st == STATUS_WRAPAROUND_SKIPPED);
#endif

      if (!accept)
        {
          data->distance_mm[i] = DISTANCE_FILTERED;
          continue;
        }

#if MIN_SIGNAL_PER_SPAD > 0

      /* Reject zones whose return signal is too weak
       * relative to the sensor's noise floor — the
       * main lever against unreliable readings in
       * bright ambient light (sunlight raises the
       * noise floor and starves weak returns). */

      if (data->signal_per_spad[i] < MIN_SIGNAL_PER_SPAD)
        {
          data->distance_mm[i] = DISTANCE_FILTERED;
        }
#endif
    }
}

/**********************************************************
 * Public Functions
 **********************************************************/

/**********************************************************
 * Name: maia_tof_init
 *
 * Description:
 *   Initialize sensors with LPn address reassignment:
 *   1. Both LPn LOW
 *   2. Sensor LEFT:  0x52 -> TOF1_ADDR (0x54)
 *   3. Sensor RIGHT: 0x52 -> TOF2_ADDR (0x52)
 *   Only the "LEFT"/primary slot is initialized unless
 *   CONFIG_MAIA_VL53L5CX_MODE_DUAL is set (Kconfig
 *   choice MAIA_VL53L5CX_MODE: dual / left only / right
 *   only). In "right only" mode, TOF1_LPN_GPIO/
 *   TOF1_INT_GPIO are remapped to the physical RIGHT
 *   sensor's GPIOs (see pre-processor section above).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *
 **********************************************************/

esp_err_t maia_tof_init(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Driver file built: %s %s",
           __DATE__, __TIME__);

  if (g_driver_initialized)
    {
      ESP_LOGW(TAG, "Already initialized");
      return ESP_OK;
    }

  memset(g_sensors, 0, sizeof(g_sensors));

  g_sensors[MAIA_TOF_SENSOR_LEFT].lpn_gpio =
      TOF1_LPN_GPIO;
  g_sensors[MAIA_TOF_SENSOR_LEFT].int_gpio =
      TOF1_INT_GPIO;
  g_sensors[MAIA_TOF_SENSOR_RIGHT].lpn_gpio =
      TOF2_LPN_GPIO;
  g_sensors[MAIA_TOF_SENSOR_RIGHT].int_gpio =
      TOF2_INT_GPIO;

  ret = init_lpn_gpio(TOF1_LPN_GPIO);
  if (ret != ESP_OK)
    {
      return ret;
    }

  lpn_set(TOF1_LPN_GPIO, 0);

#ifdef CONFIG_MAIA_VL53L5CX_MODE_DUAL
  ret = init_lpn_gpio(TOF2_LPN_GPIO);
  if (ret != ESP_OK)
    {
      return ret;
    }

  lpn_set(TOF2_LPN_GPIO, 0);
#endif

  ret = init_int_gpio(TOF1_INT_GPIO);
  if (ret != ESP_OK)
    {
      return ret;
    }

#ifdef CONFIG_MAIA_VL53L5CX_MODE_DUAL
  ret = init_int_gpio(TOF2_INT_GPIO);
  if (ret != ESP_OK)
    {
      return ret;
    }
#endif

  vTaskDelay(pdMS_TO_TICKS(50));

  ESP_LOGI(TAG, "--- LEFT sensor ---");
  ret = init_single_sensor(
            &g_sensors[MAIA_TOF_SENSOR_LEFT],
            VL53L5CX_DEFAULT_ADDR,
            TOF1_ADDR);
  if (ret != ESP_OK)
    {
      return ret;
    }

#ifdef CONFIG_MAIA_VL53L5CX_MODE_DUAL
  ESP_LOGI(TAG, "--- RIGHT sensor ---");
  ret = init_single_sensor(
            &g_sensors[MAIA_TOF_SENSOR_RIGHT],
            VL53L5CX_DEFAULT_ADDR,
            TOF2_ADDR);
  if (ret != ESP_OK)
    {
      return ret;
    }
#endif

  g_driver_initialized = true;
  ESP_LOGI(TAG, "Driver initialized OK");
  return ESP_OK;
}

/**********************************************************
 * Name: maia_tof_deinit
 *
 * Description:
 *   Stop ranging, remove I2C devices, disable sensors.
 *
 **********************************************************/

esp_err_t maia_tof_deinit(void)
{
  int i;

  for (i = 0; i < MAIA_TOF_SENSOR_MAX; i++)
    {
      if (g_sensors[i].initialized)
        {
          vl53l5cx_stop_ranging(
              &g_sensors[i].uld_cfg);
          unregister_sensor(
              &g_sensors[i].uld_cfg.platform);
          lpn_set(g_sensors[i].lpn_gpio, 0);
          g_sensors[i].initialized = false;
        }
    }

  g_driver_initialized = false;
  ESP_LOGI(TAG, "Driver deinitialized");
  return ESP_OK;
}

/**********************************************************
 * Name: maia_tof_start_ranging
 *
 * Description:
 *   Start ranging on all initialized sensors.
 *
 **********************************************************/

esp_err_t maia_tof_start_ranging(void)
{
  int     i;
  uint8_t status;

  for (i = 0; i < MAIA_TOF_SENSOR_MAX; i++)
    {
      if (g_sensors[i].initialized)
        {
          status = vl53l5cx_start_ranging(
                       &g_sensors[i].uld_cfg);
          if (status != 0)
            {
              ESP_LOGE(TAG,
                       "Start ranging %d failed",
                       i);
              return ESP_FAIL;
            }

          ESP_LOGI(TAG,
                   "Sensor %d ranging started", i);
        }
    }

  return ESP_OK;
}

/**********************************************************
 * Name: maia_tof_stop_ranging
 *
 * Description:
 *   Stop ranging on all initialized sensors.
 *
 **********************************************************/

esp_err_t maia_tof_stop_ranging(void)
{
  int     i;
  uint8_t status;

  for (i = 0; i < MAIA_TOF_SENSOR_MAX; i++)
    {
      if (g_sensors[i].initialized)
        {
          status = vl53l5cx_stop_ranging(
                       &g_sensors[i].uld_cfg);
          if (status != 0)
            {
              ESP_LOGE(TAG,
                       "Stop ranging %d failed", i);
              return ESP_FAIL;
            }
        }
    }

  return ESP_OK;
}

/**********************************************************
 * Name: maia_tof_data_ready
 *
 * Description:
 *   Poll sensor for new data availability.
 *
 **********************************************************/

esp_err_t maia_tof_data_ready(uint8_t sensor_id,
                               bool *ready)
{
  uint8_t is_ready = 0;
  uint8_t status;

  if (sensor_id >= MAIA_TOF_SENSOR_MAX ||
      !g_sensors[sensor_id].initialized)
    {
      return ESP_ERR_INVALID_ARG;
    }

  status = vl53l5cx_check_data_ready(
               &g_sensors[sensor_id].uld_cfg,
               &is_ready);
  if (status != 0)
    {
      *ready = false;
      return ESP_FAIL;
    }

  *ready = (is_ready != 0);
  return ESP_OK;
}

/**********************************************************
 * Name: maia_tof_get_data
 *
 * Description:
 *   Get ranging data from one sensor.
 *   Applies status filters per Kconfig.
 *
 **********************************************************/

esp_err_t maia_tof_get_data(uint8_t sensor_id,
                             maia_tof_data_t *data)
{
  VL53L5CX_ResultsData results;
  uint8_t              status;
  int                  i;

  if (sensor_id >= MAIA_TOF_SENSOR_MAX ||
      !g_sensors[sensor_id].initialized ||
      data == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  status = vl53l5cx_get_ranging_data(
               &g_sensors[sensor_id].uld_cfg,
               &results);
  if (status != 0)
    {
      ESP_LOGE(TAG, "Get data sensor %d failed",
               sensor_id);
      return ESP_FAIL;
    }

  data->nb_zones = NUM_ZONES;

  for (i = 0; i < NUM_ZONES; i++)
    {
      data->distance_mm[i] =
          results.distance_mm[
              VL53L5CX_NB_TARGET_PER_ZONE * i];
      data->target_status[i] =
          results.target_status[
              VL53L5CX_NB_TARGET_PER_ZONE * i];
      data->signal_per_spad[i] =
          results.signal_per_spad[
              VL53L5CX_NB_TARGET_PER_ZONE * i];
      data->ambient_per_spad[i] =
          results.ambient_per_spad[i];
    }

  filter_data(data);

#ifdef CONFIG_MAIA_VL53L5CX_LOG_RAW_DATA
  for (i = 0; i < NUM_ZONES; i++)
    {
      ESP_LOGI(TAG,
               "Z%02d: d=%dmm st=%d sig=%lu",
               i,
               data->distance_mm[i],
               data->target_status[i],
               (unsigned long)
                   data->signal_per_spad[i]);
    }
#endif

  return ESP_OK;
}

/**********************************************************
 * Name: maia_tof_set_power_mode
 *
 * Description:
 *   Set sensor power mode. Low-power retains firmware.
 *
 **********************************************************/

esp_err_t maia_tof_set_power_mode(uint8_t sensor_id,
                                   uint8_t mode)
{
  uint8_t status;
  uint8_t uld_mode;

  if (sensor_id >= MAIA_TOF_SENSOR_MAX ||
      !g_sensors[sensor_id].initialized)
    {
      return ESP_ERR_INVALID_ARG;
    }

  uld_mode = (mode == MAIA_TOF_POWER_LOW_POWER)
             ? VL53L5CX_POWER_MODE_SLEEP
             : VL53L5CX_POWER_MODE_WAKEUP;

  status = vl53l5cx_set_power_mode(
               &g_sensors[sensor_id].uld_cfg,
               uld_mode);
  if (status != 0)
    {
      ESP_LOGE(TAG, "Power mode %d failed",
               sensor_id);
      return ESP_FAIL;
    }

  ESP_LOGI(TAG, "Sensor %d: %s", sensor_id,
           mode == MAIA_TOF_POWER_LOW_POWER
               ? "sleep" : "active");

  return ESP_OK;
}
