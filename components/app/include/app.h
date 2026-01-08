/****************************************************************************
 * components/app/include/app.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMPONENTS_APP_INCLUDE_APP_H
#define __COMPONENTS_APP_INCLUDE_APP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: app_init
 *
 * Description:
 *   Initialize the application and start all tasks.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t app_init(void);

#endif /* __COMPONENTS_APP_INCLUDE_APP_H */
