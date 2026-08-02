/*
 * Copyright 2026 Vinicius May
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************
 * main/tests/tests.h
 *
 * MAIA - Test suite interface
 *
 ****************************************************************************/

#ifndef __MAIN_TESTS_TESTS_H
#define __MAIN_TESTS_TESTS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdkconfig.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Only the tests whose device is enabled are compiled, so each
 * prototype carries the same condition as its source file. A test
 * selected without its device then fails at compile time instead of
 * at link time.
 */

void test_blink_run(void);
void test_button_run(void);

#ifdef CONFIG_MAIA_DS18B20_ENABLE
void test_ds18b20_run(void);
#endif

#ifdef CONFIG_MAIA_DRV2605L_ENABLE
void test_drv2605l_run(void);
#endif

#ifdef CONFIG_MAIA_SSD1306_ENABLE
void test_ssd1306_run(void);
#endif

#ifdef CONFIG_MAIA_VL53L5CX_ENABLE
void test_vl53l5cx_run(void);
#endif

#ifdef CONFIG_MAIA_IMU_MPU6050
void test_mpu6050_run(void);
#endif

#endif /* __MAIN_TESTS_TESTS_H */
