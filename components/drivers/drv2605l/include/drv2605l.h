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
 * components/drivers/drv2605l/include/drv2605l.h
 *
 * DRV2605L Haptic Motor Driver
 * Texas Instruments DRV2605L Low-Voltage Haptic Driver for ERM and LRA
 *
 * Reference: SLOS850D - DRV2605L Datasheet (Rev. D, October 2013)
 * https://www.ti.com/lit/ds/symlink/drv2605l.pdf
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_DRV2605L_INCLUDE_DRV2605L_H
#define __COMPONENTS_DRIVERS_DRV2605L_INCLUDE_DRV2605L_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DRV2605L I2C Default Address
 * Datasheet Section 7.3.1, Page 15
 */

#define DRV2605L_I2C_ADDR_DEFAULT 0x5A

/* Register Map
 * Datasheet Table 7, Page 23
 */

#define DRV2605L_REG_STATUS       0x00  /* Status register */
#define DRV2605L_REG_MODE         0x01  /* Mode register */
#define DRV2605L_REG_RTPIN        0x02  /* Real-time playback input */
#define DRV2605L_REG_LIBRARY      0x03  /* Waveform library selection */
#define DRV2605L_REG_WAVESEQ1     0x04  /* Waveform sequencer 1 */
#define DRV2605L_REG_WAVESEQ2     0x05  /* Waveform sequencer 2 */
#define DRV2605L_REG_WAVESEQ3     0x06  /* Waveform sequencer 3 */
#define DRV2605L_REG_WAVESEQ4     0x07  /* Waveform sequencer 4 */
#define DRV2605L_REG_WAVESEQ5     0x08  /* Waveform sequencer 5 */
#define DRV2605L_REG_WAVESEQ6     0x09  /* Waveform sequencer 6 */
#define DRV2605L_REG_WAVESEQ7     0x0A  /* Waveform sequencer 7 */
#define DRV2605L_REG_WAVESEQ8     0x0B  /* Waveform sequencer 8 */
#define DRV2605L_REG_GO           0x0C  /* GO register (trigger) */
#define DRV2605L_REG_OVERDRIVE    0x0D  /* Overdrive time offset */
#define DRV2605L_REG_SUSTAINPOS   0x0E  /* Sustain time offset (positive) */
#define DRV2605L_REG_SUSTAINNEG   0x0F  /* Sustain time offset (negative) */
#define DRV2605L_REG_BREAK        0x10  /* Brake time offset */
#define DRV2605L_REG_AUDIOCTRL    0x11  /* Audio-to-vibe control */
#define DRV2605L_REG_AUDIOVIBE    0x12  /* Audio-to-vibe min input level */
#define DRV2605L_REG_AUDIOMAX     0x13  /* Audio-to-vibe max input level */
#define DRV2605L_REG_RATEDV       0x16  /* Rated voltage */
#define DRV2605L_REG_CLAMPV       0x17  /* Overdrive clamp voltage */
#define DRV2605L_REG_AUTOCALCOMP  0x18  /* Auto-calibration compensation */
#define DRV2605L_REG_AUTOCALEMP   0x19  /* Auto-calibration back-EMF */
#define DRV2605L_REG_FEEDBACK     0x1A  /* Feedback control */
#define DRV2605L_REG_CONTROL1     0x1B  /* Control register 1 */
#define DRV2605L_REG_CONTROL2     0x1C  /* Control register 2 */
#define DRV2605L_REG_CONTROL3     0x1D  /* Control register 3 */
#define DRV2605L_REG_CONTROL4     0x1E  /* Control register 4 */
#define DRV2605L_REG_VBAT         0x21  /* Battery voltage monitor */
#define DRV2605L_REG_LRARESON     0x22  /* LRA resonance period */

/* Mode Register (0x01) Bits
 * Datasheet Section 8.5.2, Page 39
 */

#define DRV2605L_MODE_INTTRIG     0x00  /* Internal trigger mode */
#define DRV2605L_MODE_EXTTRIGEDGE 0x01  /* External edge trigger */
#define DRV2605L_MODE_EXTTRIGLVL  0x02  /* External level trigger */
#define DRV2605L_MODE_PWMANALOG   0x03  /* PWM/Analog input mode */
#define DRV2605L_MODE_AUDIOVIBE   0x04  /* Audio-to-vibe mode */
#define DRV2605L_MODE_REALTIME    0x05  /* Real-time playback (RTP) */
#define DRV2605L_MODE_DIAGNOSE    0x06  /* Diagnostics mode */
#define DRV2605L_MODE_AUTOCAL     0x07  /* Auto-calibration mode */
#define DRV2605L_MODE_STANDBY     0x40  /* Standby bit (software shutdown) */

/* Library Selection (0x03)
 * Datasheet Section 8.5.4, Page 51-52
 * 
 * ERM Libraries:
 *   - Library A: Sharp clicks, strong vibrations (general purpose)
 *   - Library B: Soft bumps, light taps (gentle feedback)
 *   - Library C: Medium response (balanced)
 *   - Library D: Alert patterns (notifications)
 *   - Library E: Buzz patterns (continuous vibration)
 * 
 * LRA Library:
 *   - Library F: Linear Resonant Actuator effects
 *
 * Each library contains 123 waveform effects (indexed 1-123)
 */

#define DRV2605L_LIBRARY_EMPTY    0x00  /* No library (disabled) */
#define DRV2605L_LIBRARY_ERM_A    0x01  /* ERM Library A */
#define DRV2605L_LIBRARY_ERM_B    0x02  /* ERM Library B */
#define DRV2605L_LIBRARY_ERM_C    0x03  /* ERM Library C */
#define DRV2605L_LIBRARY_ERM_D    0x04  /* ERM Library D */
#define DRV2605L_LIBRARY_ERM_E    0x05  /* ERM Library E */
#define DRV2605L_LIBRARY_LRA      0x06  /* LRA Library F */

/* GO Register (0x0C)
 * Datasheet Section 8.5.9, Page 58
 */

#define DRV2605L_GO_BIT           0x01  /* Set to 1 to trigger playback */

/* Feedback Control (0x1A) Bits
 * Datasheet Section 8.5.19, Page 66
 */

#define DRV2605L_FEEDBACK_ERM     0x00  /* ERM mode (bit 7 = 0) */
#define DRV2605L_FEEDBACK_LRA     0x80  /* LRA mode (bit 7 = 1) */

/* Effect IDs (Library Waveforms)
 * Datasheet Section 11.2, Page 72-75
 * Each library has 123 effects indexed from 1 to 123
 * Effect 0 = End of sequence marker
 */

#define DRV2605L_EFFECT_STOP      0     /* Stop/end of sequence */
#define DRV2605L_EFFECT_MIN       1     /* Minimum valid effect ID */
#define DRV2605L_EFFECT_MAX       123   /* Maximum valid effect ID */

/* Status Register (0x00) Bits
 * Datasheet Section 8.5.1, Page 37
 */

#define DRV2605L_STATUS_OVER_TEMP  0x02  /* Over-temperature flag */
#define DRV2605L_STATUS_OC_DETECT  0x01  /* Over-current flag */
#define DRV2605L_STATUS_DIAG_RESULT 0x08 /* Diagnostic result flag */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Driver operation mode
 * See MODE register definitions above
 */

typedef enum
{
  DRV2605L_OP_MODE_INTERNAL_TRIGGER = DRV2605L_MODE_INTTRIG,
  DRV2605L_OP_MODE_EXTERNAL_EDGE    = DRV2605L_MODE_EXTTRIGEDGE,
  DRV2605L_OP_MODE_EXTERNAL_LEVEL   = DRV2605L_MODE_EXTTRIGLVL,
  DRV2605L_OP_MODE_PWM_ANALOG       = DRV2605L_MODE_PWMANALOG,
  DRV2605L_OP_MODE_AUDIO_VIBE       = DRV2605L_MODE_AUDIOVIBE,
  DRV2605L_OP_MODE_REALTIME         = DRV2605L_MODE_REALTIME,
  DRV2605L_OP_MODE_DIAGNOSE         = DRV2605L_MODE_DIAGNOSE,
  DRV2605L_OP_MODE_AUTO_CALIBRATION = DRV2605L_MODE_AUTOCAL,
  DRV2605L_OP_MODE_STANDBY          = DRV2605L_MODE_STANDBY,
} drv2605l_mode_t;

/* Actuator type
 * Datasheet Section 8.5.19, Page 66 (Feedback Control Register)
 */

typedef enum
{
  DRV2605L_ACTUATOR_ERM = 0,  /* Eccentric Rotating Mass motor */
  DRV2605L_ACTUATOR_LRA = 1,  /* Linear Resonant Actuator */
} drv2605l_actuator_t;

/* Library selection
 * See LIBRARY register definitions above
 */

typedef enum
{
  DRV2605L_LIB_EMPTY = DRV2605L_LIBRARY_EMPTY,
  DRV2605L_LIB_ERM_A = DRV2605L_LIBRARY_ERM_A,
  DRV2605L_LIB_ERM_B = DRV2605L_LIBRARY_ERM_B,
  DRV2605L_LIB_ERM_C = DRV2605L_LIBRARY_ERM_C,
  DRV2605L_LIB_ERM_D = DRV2605L_LIBRARY_ERM_D,
  DRV2605L_LIB_ERM_E = DRV2605L_LIBRARY_ERM_E,
  DRV2605L_LIB_LRA   = DRV2605L_LIBRARY_LRA,
} drv2605l_library_t;

/* Device configuration structure */

typedef struct
{
  uint8_t i2c_addr;               /* I2C address (default 0x5A) */
  drv2605l_actuator_t actuator;   /* Actuator type (ERM/LRA) */
  drv2605l_library_t library;     /* Effect library selection */
  uint8_t rated_voltage;          /* Rated voltage register value */
  uint8_t overdrive_clamp;        /* Overdrive clamp voltage */
  bool auto_calibrate;            /* Run auto-calibration on init */
} drv2605l_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: drv2605l_init
 *
 * Description:
 *   Initialize DRV2605L haptic driver with given configuration.
 *   Performs auto-calibration if enabled in config.
 *
 * Input Parameters:
 *   config - Device configuration structure
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 * Reference:
 *   Datasheet Section 9.1 (Initialization), Page 76
 *
 ****************************************************************************/

esp_err_t drv2605l_init(const drv2605l_config_t *config);

/****************************************************************************
 * Name: drv2605l_play_effect
 *
 * Description:
 *   Play a single haptic effect from the selected library.
 *   Effect IDs range from 1 to 123.
 *
 * Input Parameters:
 *   effect_id - Waveform effect ID (1-123)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 * Reference:
 *   Datasheet Section 11.2 (Waveform Library), Page 72-75
 *
 ****************************************************************************/

esp_err_t drv2605l_play_effect(uint8_t effect_id);

/****************************************************************************
 * Name: drv2605l_play_sequence
 *
 * Description:
 *   Play a sequence of up to 8 haptic effects.
 *   Effects play sequentially with automatic timing.
 *
 * Input Parameters:
 *   effects     - Array of effect IDs (1-123)
 *   num_effects - Number of effects in sequence (1-8)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 * Reference:
 *   Datasheet Section 8.5.5-8.5.8 (Waveform Sequencer), Page 53-57
 *
 ****************************************************************************/

esp_err_t drv2605l_play_sequence(const uint8_t *effects,
                                  uint8_t num_effects);

/****************************************************************************
 * Name: drv2605l_stop
 *
 * Description:
 *   Stop current haptic playback immediately.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t drv2605l_stop(void);

/****************************************************************************
 * Name: drv2605l_set_mode
 *
 * Description:
 *   Set device operation mode.
 *
 * Input Parameters:
 *   mode - Operation mode (see drv2605l_mode_t)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 * Reference:
 *   Datasheet Section 8.5.2 (Mode Register), Page 39
 *
 ****************************************************************************/

esp_err_t drv2605l_set_mode(drv2605l_mode_t mode);

/****************************************************************************
 * Name: drv2605l_set_library
 *
 * Description:
 *   Change effect library selection.
 *
 * Input Parameters:
 *   library - Library selection (see drv2605l_library_t)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 * Reference:
 *   Datasheet Section 8.5.4 (Library Selection), Page 51-52
 *
 ****************************************************************************/

esp_err_t drv2605l_set_library(drv2605l_library_t library);

/****************************************************************************
 * Name: drv2605l_standby
 *
 * Description:
 *   Put device into low-power standby mode.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 * Reference:
 *   Datasheet Section 8.5.2 (Standby Bit), Page 40
 *
 ****************************************************************************/

esp_err_t drv2605l_standby(void);

/****************************************************************************
 * Name: drv2605l_wakeup
 *
 * Description:
 *   Wake device from standby mode to active state.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t drv2605l_wakeup(void);

/****************************************************************************
 * Name: drv2605l_get_status
 *
 * Description:
 *   Read device status register for diagnostics.
 *
 * Input Parameters:
 *   status - Pointer to store status byte
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 * Reference:
 *   Datasheet Section 8.5.1 (Status Register), Page 37
 *
 ****************************************************************************/

esp_err_t drv2605l_get_status(uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif /* __COMPONENTS_DRIVERS_DRV2605L_INCLUDE_DRV2605L_H */