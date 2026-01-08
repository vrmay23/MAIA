/****************************************************************************
 * components/drivers/mpu6050/include/mpu6050.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_MPU6050_INCLUDE_MPU6050_H
#define __COMPONENTS_DRIVERS_MPU6050_INCLUDE_MPU6050_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpu6050_init
 *
 * Description:
 *   Initialize the mpu6050 driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure.
 *
 ****************************************************************************/

esp_err_t mpu6050_init(void);

#endif /* __COMPONENTS_DRIVERS_MPU6050_INCLUDE_MPU6050_H */
