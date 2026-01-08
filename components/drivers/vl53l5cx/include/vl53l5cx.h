/****************************************************************************
 * components/drivers/vl53l5cx/include/vl53l5cx.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_VL53L5CX_INCLUDE_VL53L5CX_H
#define __COMPONENTS_DRIVERS_VL53L5CX_INCLUDE_VL53L5CX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l5cx_init
 *
 * Description:
 *   Initialize the vl53l5cx driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t vl53l5cx_init(void);

#endif /* __COMPONENTS_DRIVERS_VL53L5CX_INCLUDE_VL53L5CX_H */
