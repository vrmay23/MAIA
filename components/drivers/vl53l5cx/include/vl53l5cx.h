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
 * components/drivers/vl53l5cx/include/vl53l5cx.h
 *
 * MAIA VL53L5CX ToF Sensor Driver - Public API
 *
 * All public functions are prefixed maia_tof_* to avoid
 * symbol collision with the ST ULD API functions that
 * share the vl53l5cx_* namespace.
 *
 **********************************************************/

#ifndef __COMPONENTS_DRIVERS_VL53L5CX_INCLUDE_VL53L5CX_H
#define __COMPONENTS_DRIVERS_VL53L5CX_INCLUDE_VL53L5CX_H

/**********************************************************
 * Included Files
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

/**********************************************************
 * Pre-processor Definitions
 **********************************************************/

/* Sensor IDs */

#define MAIA_TOF_SENSOR_LEFT   0
#define MAIA_TOF_SENSOR_RIGHT  1
#define MAIA_TOF_SENSOR_MAX    2

/* Power modes */

#define MAIA_TOF_POWER_ACTIVE     0
#define MAIA_TOF_POWER_LOW_POWER  1

/* Maximum zones in 8x8 resolution */

#define MAIA_TOF_MAX_ZONES  64

/**********************************************************
 * Public Types
 **********************************************************/

/**********************************************************
 * Name: maia_tof_data_t
 *
 * Description:
 *   Ranging data for one sensor. Contains per-zone
 *   distance, status, signal, and ambient measurements
 *   for up to 64 zones (8x8 grid).
 *
 *   A zone is kept only if its target_status is 5
 *   (always) or, when enabled per Kconfig, 9 or 6 (see
 *   filter_data() in vl53l5cx.c). Every other status,
 *   including 255 (no target detected), is filtered.
 *   Filtered zones have distance_mm set to -1.
 *   target_status itself is never rewritten, so it
 *   always holds the sensor's raw, unfiltered value.
 *
 *   Zone validity has a single test: distance_mm != -1
 *   (DISTANCE_FILTERED). There is no separate per-zone
 *   "target detected" flag.
 *
 **********************************************************/

typedef struct
{
  int16_t  distance_mm[MAIA_TOF_MAX_ZONES];
  uint8_t  target_status[MAIA_TOF_MAX_ZONES];
  uint32_t signal_per_spad[MAIA_TOF_MAX_ZONES];
  uint32_t ambient_per_spad[MAIA_TOF_MAX_ZONES];
  uint8_t  nb_zones;
} maia_tof_data_t;

/**********************************************************
 * Public Function Prototypes
 **********************************************************/

/**********************************************************
 * Name: maia_tof_init
 *
 * Description:
 *   Initialize one or both VL53L5CX sensors. Performs
 *   LPn-based address reassignment for dual-sensor
 *   setups. Uploads ULD firmware (~80KB per sensor).
 *   Applies Kconfig settings for resolution, ranging
 *   frequency, integration time, mode, and target order.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *
 **********************************************************/

esp_err_t maia_tof_init(void);

/**********************************************************
 * Name: maia_tof_deinit
 *
 * Description:
 *   Stop ranging and release I2C device handles for
 *   all initialized sensors.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *
 **********************************************************/

esp_err_t maia_tof_deinit(void);

/**********************************************************
 * Name: maia_tof_start_ranging
 *
 * Description:
 *   Start continuous or autonomous ranging on all
 *   initialized sensors.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *
 **********************************************************/

esp_err_t maia_tof_start_ranging(void);

/**********************************************************
 * Name: maia_tof_stop_ranging
 *
 * Description:
 *   Stop ranging on all initialized sensors.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *
 **********************************************************/

esp_err_t maia_tof_stop_ranging(void);

/**********************************************************
 * Name: maia_tof_data_ready
 *
 * Description:
 *   Check if new ranging data is available from the
 *   specified sensor (polling mode).
 *
 * Input Parameters:
 *   sensor_id - MAIA_TOF_SENSOR_LEFT or _RIGHT
 *   ready     - Pointer to boolean result
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *   ESP_ERR_INVALID_ARG if sensor_id is invalid.
 *
 **********************************************************/

esp_err_t maia_tof_data_ready(uint8_t sensor_id,
                              bool *ready);

/**********************************************************
 * Name: maia_tof_get_data
 *
 * Description:
 *   Retrieve the latest ranging data from the specified
 *   sensor. Applies target_status filtering per Kconfig.
 *   Zones that fail filtering have distance_mm = -1.
 *
 * Input Parameters:
 *   sensor_id - MAIA_TOF_SENSOR_LEFT or _RIGHT
 *   data      - Pointer to output data structure
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *   ESP_ERR_INVALID_ARG if sensor_id is invalid.
 *
 **********************************************************/

esp_err_t maia_tof_get_data(uint8_t sensor_id,
                            maia_tof_data_t *data);

/**********************************************************
 * Name: maia_tof_set_power_mode
 *
 * Description:
 *   Set sensor power mode. Low-power mode retains
 *   firmware but stops ranging and reduces current.
 *
 * Input Parameters:
 *   sensor_id - MAIA_TOF_SENSOR_LEFT or _RIGHT
 *   mode      - MAIA_TOF_POWER_ACTIVE or _LOW_POWER
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on error.
 *   ESP_ERR_INVALID_ARG if sensor_id is invalid.
 *
 **********************************************************/

esp_err_t maia_tof_set_power_mode(uint8_t sensor_id,
                                  uint8_t mode);

#endif /* __COMPONENTS_DRIVERS_VL53L5CX_INCLUDE_VL53L5CX_H */
