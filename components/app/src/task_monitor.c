/****************************************************************************
 * components/app/src/task_monitor.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "app_tasks.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_monitor
 *
 * Description:
 *   FreeRTOS task function.
 *
 * Input Parameters:
 *   pvParameters - Task parameters (unused)
 *
 * Returned Value:
 *   None (task never returns)
 *
 ****************************************************************************/

void task_monitor(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
    {
      /* TODO: Implement task_monitor */
    }
}
