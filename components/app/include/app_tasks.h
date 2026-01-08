/****************************************************************************
 * components/app/include/app_tasks.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMPONENTS_APP_INCLUDE_APP_TASKS_H
#define __COMPONENTS_APP_INCLUDE_APP_TASKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void task_sensing(void *pvParameters);
void task_haptic(void *pvParameters);
void task_display(void *pvParameters);
void task_monitor(void *pvParameters);

#endif /* __COMPONENTS_APP_INCLUDE_APP_TASKS_H */
