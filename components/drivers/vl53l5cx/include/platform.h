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
 * components/drivers/vl53l5cx/include/platform.h
 *
 * MAIA VL53L5CX Platform Definition
 *
 * This file satisfies the #include "platform.h" inside
 * the ST VL53L5CX ULD (vl53l5cx_api.h). It is found
 * first because vl53l5cx/include/ appears in INCLUDE_DIRS
 * before managed_components/. The ST porting/ directory
 * is intentionally excluded from INCLUDE_DIRS.
 **********************************************************/

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

/**********************************************************
 * Included Files
 **********************************************************/

#include <stdint.h>
#include <string.h>
#include <driver/i2c_master.h>

/**********************************************************
 * Pre-processor Definitions
 **********************************************************/

/* Number of targets per zone. 1 = closest only. */

#ifndef VL53L5CX_NB_TARGET_PER_ZONE
#  define VL53L5CX_NB_TARGET_PER_ZONE  (1U)
#endif

/* Disable unused result fields to reduce RAM usage */

#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_AMBIENT_DMAX
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM

/* ESP32-S3 is little-endian */

#define PROCESSOR_LITTLE_ENDIAN

#ifdef PROCESSOR_LITTLE_ENDIAN
#  define SWAP_UINT16(x)  (x)
#  define SWAP_UINT32(x)  (x)
#else
#  define SWAP_UINT16(x) \
     (((x) >> 8) | ((x) << 8))
#  define SWAP_UINT32(x) \
     (((x) >> 24) | (((x) & 0x00FF0000UL) >> 8) | \
      (((x) & 0x0000FF00UL) << 8) | ((x) << 24))
#endif

/**********************************************************
 * Public Types
 **********************************************************/

/**********************************************************
 * Name: VL53L5CX_Platform
 *
 * Description:
 *   Platform context passed to all ULD I2C callbacks.
 *   Uses ESP-IDF v6.0 i2c_master_dev_handle_t directly.
 *   This type does not exist in any ST implementation.
 *
 **********************************************************/

typedef struct
{
  i2c_master_dev_handle_t dev_handle;
  uint16_t                address;
} VL53L5CX_Platform;

/**********************************************************
 * Public Function Prototypes
 *
 * Required by ST ULD. Implemented in
 * vl53l5cx_platform.c using ESP-IDF v6.0 exclusively.
 *
 **********************************************************/

uint8_t RdByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress,
               uint8_t *p_value);

uint8_t WrByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress,
               uint8_t value);

uint8_t RdMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values,
                uint32_t size);

uint8_t WrMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values,
                uint32_t size);

void SwapBuffer(uint8_t *buffer,
                uint16_t size);

uint8_t WaitMs(VL53L5CX_Platform *p_platform,
               uint32_t TimeMs);

#endif /* _PLATFORM_H_ */
