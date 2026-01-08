/****************************************************************************
 * components/drivers/ssd1306/include/ssd1306.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_SSD1306_INCLUDE_SSD1306_H
#define __COMPONENTS_DRIVERS_SSD1306_INCLUDE_SSD1306_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ssd1306_init
 *
 * Description:
 *   Initialize the ssd1306 driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t ssd1306_init(void);

#endif /* __COMPONENTS_DRIVERS_SSD1306_INCLUDE_SSD1306_H */
