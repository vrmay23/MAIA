/****************************************************************************
 * components/drivers/ds18b20/include/ds18b20.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_DS18B20_INCLUDE_DS18B20_H
#define __COMPONENTS_DRIVERS_DS18B20_INCLUDE_DS18B20_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ds18b20_init
 *
 * Description:
 *   Initialize the ds18b20 driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t ds18b20_init(void);

#endif /* __COMPONENTS_DRIVERS_DS18B20_INCLUDE_DS18B20_H */
