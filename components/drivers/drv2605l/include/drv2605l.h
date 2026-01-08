/****************************************************************************
 * components/drivers/drv2605l/include/drv2605l.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_DRV2605L_INCLUDE_DRV2605L_H
#define __COMPONENTS_DRIVERS_DRV2605L_INCLUDE_DRV2605L_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605l_init
 *
 * Description:
 *   Initialize the drv2605l driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t drv2605l_init(void);

#endif /* __COMPONENTS_DRIVERS_DRV2605L_INCLUDE_DRV2605L_H */
