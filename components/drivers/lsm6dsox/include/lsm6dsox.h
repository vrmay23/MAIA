/*
 * Copyright 2026 Vinicius Rodrigo May
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
 * components/drivers/lsm6dsox/include/lsm6dsox.h
 *
 * LSM6DSOX 6-Axis IMU Driver
 * STMicroelectronics iNEMO inertial module: 3D accelerometer + 3D gyroscope
 * with Machine Learning Core and Finite State Machine
 *
 * Reference: DS12814 - LSM6DSOX Datasheet
 * Reference: AN5272 - LSM6DSOX Application Note
 *
 * ---------------------------------------------------------------------------
 * TRANSPORT: I2C ONLY
 * ---------------------------------------------------------------------------
 * This driver speaks I2C and nothing else. SPI and MIPI I3C are deliberately
 * out of scope for now and will be addressed later.
 *
 * The chip itself supports all three (Datasheet Section 5), and the MAIA
 * board wiring selects I2C: CS is strapped high, which per Table 1 means
 * "SPI idle mode / I2C / MIPI I3C communication enabled", and SDO/SA0 is
 * strapped low, giving slave address 0x6A.
 *
 * Two things block the other transports today:
 *
 *   SPI  - Not reachable on this board. CS is hardwired to VDDIO, and SPI
 *          requires it driven low by a GPIO. Worse, SDO/SA0 is tied to GND,
 *          and that pin becomes the SDO output in 4-wire SPI. Enabling SPI
 *          would short an output to ground. This needs a board revision,
 *          not a driver change.
 *
 *   I3C  - Reachable on this board (same SDA/SCL pins, and CS high already
 *          enables it), but the ESP32-S3 has no I3C controller. Espressif
 *          provides one on the ESP32-P4 only. This needs different silicon.
 *
 * When either becomes viable, only the L0 layer below changes. Nothing in
 * L1-L9 performs a bus operation directly, so the feature layers are
 * transport-agnostic already. The one protocol difference to remember is
 * that SPI carries the read/write flag in bit 7 of the address byte
 * (Datasheet Section 5.1.3.1), so an SPI backend would OR 0x80 on reads;
 * I2C has no such bit.
 *
 * ---------------------------------------------------------------------------
 * INCREMENTAL DEVELOPMENT MODEL
 * ---------------------------------------------------------------------------
 * This header is organized in independent feature layers. Each layer can be
 * implemented, tested and shipped on its own; no layer depends on a layer
 * above it, so features can be added over time without rewriting what
 * already works.
 *
 *   L0  Register access ....... read/write/modify/burst + page selection
 *   L1  Core .................. init, WHO_AM_I, reset, power, ranges, ODR
 *   L2  Polled data ........... status, raw and scaled accel/gyro/temp
 *   L3  Data-ready interrupt .. INT1/INT2 routing of DRDY
 *   L4  Calibration / self-test
 *   L5  FIFO .................. batching, watermark, tagged decode
 *   L6  Basic events .......... wake-up, free-fall, 6D/4D, tap, activity
 *   L7  Embedded functions .... pedometer, tilt, significant motion
 *   L8  MLC / FSM ............. UCF program upload and output readback
 *   L9  Timestamp
 *
 * Layers L0 and L1 are mandatory. Everything else is optional: unimplemented
 * entry points return ESP_ERR_NOT_SUPPORTED so callers can probe at runtime.
 *
 * L0 is deliberately public. Any feature not yet wrapped by a typed API can
 * still be driven from application code via lsm6dsox_read_reg() /
 * lsm6dsox_write_reg() / lsm6dsox_set_mem_bank(), which means a missing
 * feature never blocks progress.
 *
 * ---------------------------------------------------------------------------
 * REGISTER BANKS
 * ---------------------------------------------------------------------------
 * The LSM6DSOX multiplexes three register banks onto the same I2C
 * sub-address space, selected by FUNC_CFG_ACCESS (0x01):
 *
 *   USER bank      - default; all LSM6DSOX_REG_* below
 *   EMB_FUNC bank  - LSM6DSOX_EMB_* ; pedometer, tilt, FSM, MLC
 *   SENSOR_HUB bank- LSM6DSOX_SHUB_*; external I2C master
 *
 * Addresses collide across banks (for example 0x62 is I3C_BUS_AVB in the
 * USER bank but STEP_COUNTER_L in the EMB_FUNC bank), so every access to a
 * non-user register must be wrapped by lsm6dsox_set_mem_bank().
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_LSM6DSOX_INCLUDE_LSM6DSOX_H
#define __COMPONENTS_DRIVERS_LSM6DSOX_INCLUDE_LSM6DSOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <esp_err.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Addresses
 * SDO/SA0 pin low  (default) -> 0x6A
 * SDO/SA0 pin high           -> 0x6B
 * Datasheet Section 5.1.1
 */

#define LSM6DSOX_I2C_ADDR_DEFAULT       0x6A
#define LSM6DSOX_I2C_ADDR_ALT           0x6B

/* Device Identity
 * WHO_AM_I returns 0x6C regardless of SA0 pin state.
 * Datasheet Section 9.11
 */

#define LSM6DSOX_WHO_AM_I_VAL           0x6C

/****************************************************************************
 * USER Bank Register Map
 * Datasheet Section 9 (Table 45)
 ****************************************************************************/

#define LSM6DSOX_REG_FUNC_CFG_ACCESS    0x01  /* Bank select */
#define LSM6DSOX_REG_PIN_CTRL            0x02  /* SDO/OCS pull-up control */
#define LSM6DSOX_REG_S4S_TPH_L           0x04  /* Sensor sync TPH low */
#define LSM6DSOX_REG_S4S_TPH_H           0x05  /* Sensor sync TPH high */
#define LSM6DSOX_REG_S4S_RR              0x06  /* Sensor sync resolution */
#define LSM6DSOX_REG_FIFO_CTRL1          0x07  /* FIFO watermark [7:0] */
#define LSM6DSOX_REG_FIFO_CTRL2          0x08  /* FIFO watermark [8], compr */
#define LSM6DSOX_REG_FIFO_CTRL3          0x09  /* FIFO batch data rates */
#define LSM6DSOX_REG_FIFO_CTRL4          0x0A  /* FIFO mode, temp/ts batch */
#define LSM6DSOX_REG_COUNTER_BDR_REG1    0x0B  /* BDR counter threshold H */
#define LSM6DSOX_REG_COUNTER_BDR_REG2    0x0C  /* BDR counter threshold L */
#define LSM6DSOX_REG_INT1_CTRL           0x0D  /* INT1 pin routing */
#define LSM6DSOX_REG_INT2_CTRL           0x0E  /* INT2 pin routing */
#define LSM6DSOX_REG_WHO_AM_I            0x0F  /* Device identity */
#define LSM6DSOX_REG_CTRL1_XL            0x10  /* Accel ODR + full scale */
#define LSM6DSOX_REG_CTRL2_G             0x11  /* Gyro ODR + full scale */
#define LSM6DSOX_REG_CTRL3_C             0x12  /* BDU, IF_INC, reset, boot */
#define LSM6DSOX_REG_CTRL4_C             0x13  /* DRDY mask, I2C disable */
#define LSM6DSOX_REG_CTRL5_C             0x14  /* Self-test, rounding */
#define LSM6DSOX_REG_CTRL6_C             0x15  /* Gyro LPF1, accel HM mode */
#define LSM6DSOX_REG_CTRL7_G             0x16  /* Gyro HPF, user offset */
#define LSM6DSOX_REG_CTRL8_XL            0x17  /* Accel filtering path */
#define LSM6DSOX_REG_CTRL9_XL            0x18  /* DEN config, I3C disable */
#define LSM6DSOX_REG_CTRL10_C            0x19  /* Timestamp enable */
#define LSM6DSOX_REG_ALL_INT_SRC         0x1A  /* Aggregated event source */
#define LSM6DSOX_REG_WAKE_UP_SRC         0x1B  /* Wake-up / free-fall src */
#define LSM6DSOX_REG_TAP_SRC             0x1C  /* Tap event source */
#define LSM6DSOX_REG_D6D_SRC             0x1D  /* 6D/4D orientation source */
#define LSM6DSOX_REG_STATUS_REG          0x1E  /* Data-available flags */

/* Output registers. IF_INC (CTRL3_C) is set at init so a single burst read
 * from OUT_TEMP_L walks 0x20..0x2D: temp(2) + gyro(6) + accel(6) = 14 bytes.
 * All output words are little-endian (low byte first).
 * Datasheet Section 9.16
 */

#define LSM6DSOX_REG_OUT_TEMP_L          0x20
#define LSM6DSOX_REG_OUT_TEMP_H          0x21
#define LSM6DSOX_REG_OUTX_L_G            0x22
#define LSM6DSOX_REG_OUTX_H_G            0x23
#define LSM6DSOX_REG_OUTY_L_G            0x24
#define LSM6DSOX_REG_OUTY_H_G            0x25
#define LSM6DSOX_REG_OUTZ_L_G            0x26
#define LSM6DSOX_REG_OUTZ_H_G            0x27
#define LSM6DSOX_REG_OUTX_L_A            0x28
#define LSM6DSOX_REG_OUTX_H_A            0x29
#define LSM6DSOX_REG_OUTY_L_A            0x2A
#define LSM6DSOX_REG_OUTY_H_A            0x2B
#define LSM6DSOX_REG_OUTZ_L_A            0x2C
#define LSM6DSOX_REG_OUTZ_H_A            0x2D

/* Embedded-function status mirrored into the USER bank. Reading these does
 * NOT require a bank switch — that is the whole point of the *_MAINPAGE
 * aliases.
 */

#define LSM6DSOX_REG_EMB_FUNC_STATUS_MP  0x35
#define LSM6DSOX_REG_FSM_STATUS_A_MP     0x36
#define LSM6DSOX_REG_FSM_STATUS_B_MP     0x37
#define LSM6DSOX_REG_MLC_STATUS_MP       0x38
#define LSM6DSOX_REG_STATUS_MASTER_MP    0x39

#define LSM6DSOX_REG_FIFO_STATUS1        0x3A  /* DIFF_FIFO [7:0] */
#define LSM6DSOX_REG_FIFO_STATUS2        0x3B  /* DIFF_FIFO [9:8] + flags */

#define LSM6DSOX_REG_TIMESTAMP0          0x40
#define LSM6DSOX_REG_TIMESTAMP1          0x41
#define LSM6DSOX_REG_TIMESTAMP2          0x42
#define LSM6DSOX_REG_TIMESTAMP3          0x43

/* OIS (auxiliary SPI) readback from the primary interface */

#define LSM6DSOX_REG_UI_STATUS_REG_OIS   0x49
#define LSM6DSOX_REG_UI_OUTX_L_G_OIS     0x4A
#define LSM6DSOX_REG_UI_OUTX_L_A_OIS     0x50

#define LSM6DSOX_REG_TAP_CFG0            0x56  /* Tap axes, latch, slope */
#define LSM6DSOX_REG_TAP_CFG1            0x57  /* Tap X threshold, priority */
#define LSM6DSOX_REG_TAP_CFG2            0x58  /* Tap Y threshold, int en */
#define LSM6DSOX_REG_TAP_THS_6D          0x59  /* Tap Z threshold, 6D ths */
#define LSM6DSOX_REG_INT_DUR2            0x5A  /* Tap dur/quiet/shock */
#define LSM6DSOX_REG_WAKE_UP_THS         0x5B  /* Wake threshold, tap mode */
#define LSM6DSOX_REG_WAKE_UP_DUR         0x5C  /* Wake/sleep duration */
#define LSM6DSOX_REG_FREE_FALL           0x5D  /* Free-fall ths + duration */
#define LSM6DSOX_REG_MD1_CFG             0x5E  /* Event routing to INT1 */
#define LSM6DSOX_REG_MD2_CFG             0x5F  /* Event routing to INT2 */

#define LSM6DSOX_REG_S4S_ST_CMD_CODE     0x60
#define LSM6DSOX_REG_S4S_DT_REG          0x61
#define LSM6DSOX_REG_I3C_BUS_AVB         0x62
#define LSM6DSOX_REG_INTERNAL_FREQ_FINE  0x63  /* ODR trim, for timestamp */

#define LSM6DSOX_REG_UI_INT_OIS          0x6F
#define LSM6DSOX_REG_UI_CTRL1_OIS        0x70
#define LSM6DSOX_REG_UI_CTRL2_OIS        0x71
#define LSM6DSOX_REG_UI_CTRL3_OIS        0x72

/* Accelerometer user offsets, int8_t each. Weight selected by USR_OFF_W
 * (CTRL6_C bit 3): 2^-10 g/LSB or 2^-6 g/LSB.
 * NOTE: the LSM6DSOX has NO gyroscope offset registers. Gyro bias must be
 * subtracted in software — see lsm6dsox_calibrate().
 */

#define LSM6DSOX_REG_X_OFS_USR           0x73
#define LSM6DSOX_REG_Y_OFS_USR           0x74
#define LSM6DSOX_REG_Z_OFS_USR           0x75

/* FIFO output. 0x78 is the TAG byte identifying which sensor produced the
 * following 6 data bytes. A tagged sample is therefore 7 bytes and a burst
 * read from 0x78 yields one complete record.
 */

#define LSM6DSOX_REG_FIFO_DATA_OUT_TAG   0x78
#define LSM6DSOX_REG_FIFO_DATA_OUT_X_L   0x79
#define LSM6DSOX_REG_FIFO_DATA_OUT_X_H   0x7A
#define LSM6DSOX_REG_FIFO_DATA_OUT_Y_L   0x7B
#define LSM6DSOX_REG_FIFO_DATA_OUT_Y_H   0x7C
#define LSM6DSOX_REG_FIFO_DATA_OUT_Z_L   0x7D
#define LSM6DSOX_REG_FIFO_DATA_OUT_Z_H   0x7E

/****************************************************************************
 * EMB_FUNC Bank Register Map
 * Reachable only while LSM6DSOX_BANK_EMBEDDED is selected.
 * Datasheet Section 10
 ****************************************************************************/

#define LSM6DSOX_EMB_PAGE_SEL            0x02
#define LSM6DSOX_EMB_FUNC_EN_A           0x04  /* Pedo, tilt, sig-motion */
#define LSM6DSOX_EMB_FUNC_EN_B           0x05  /* FSM, MLC, FIFO compr */
#define LSM6DSOX_EMB_PAGE_ADDRESS        0x08  /* Advanced-page address */
#define LSM6DSOX_EMB_PAGE_VALUE          0x09  /* Advanced-page data */
#define LSM6DSOX_EMB_FUNC_INT1           0x0A  /* Emb events -> INT1 */
#define LSM6DSOX_EMB_FSM_INT1_A          0x0B
#define LSM6DSOX_EMB_FSM_INT1_B          0x0C
#define LSM6DSOX_EMB_MLC_INT1            0x0D
#define LSM6DSOX_EMB_FUNC_INT2           0x0E  /* Emb events -> INT2 */
#define LSM6DSOX_EMB_FSM_INT2_A          0x0F
#define LSM6DSOX_EMB_FSM_INT2_B          0x10
#define LSM6DSOX_EMB_MLC_INT2            0x11
#define LSM6DSOX_EMB_FUNC_STATUS         0x12
#define LSM6DSOX_EMB_FSM_STATUS_A        0x13
#define LSM6DSOX_EMB_FSM_STATUS_B        0x14
#define LSM6DSOX_EMB_MLC_STATUS          0x15
#define LSM6DSOX_EMB_PAGE_RW             0x17  /* Advanced-page R/W enable */
#define LSM6DSOX_EMB_FUNC_FIFO_CFG       0x44
#define LSM6DSOX_EMB_FSM_ENABLE_A        0x46
#define LSM6DSOX_EMB_FSM_ENABLE_B        0x47
#define LSM6DSOX_EMB_FSM_LONG_COUNTER_L  0x48
#define LSM6DSOX_EMB_FSM_LONG_COUNTER_H  0x49
#define LSM6DSOX_EMB_FSM_LONG_CNT_CLEAR  0x4A
#define LSM6DSOX_EMB_FSM_OUTS1           0x4C  /* OUTS1..OUTS16 = 0x4C..0x5B */
#define LSM6DSOX_EMB_FUNC_ODR_CFG_B      0x5F  /* Pedometer ODR */
#define LSM6DSOX_EMB_FUNC_ODR_CFG_C      0x60  /* MLC ODR */
#define LSM6DSOX_EMB_STEP_COUNTER_L      0x62
#define LSM6DSOX_EMB_STEP_COUNTER_H      0x63
#define LSM6DSOX_EMB_FUNC_SRC            0x64  /* Step overflow/detect/reset */
#define LSM6DSOX_EMB_FUNC_INIT_A         0x66  /* Pedo/tilt/sigmot init */
#define LSM6DSOX_EMB_FUNC_INIT_B         0x67  /* FSM/MLC init */
#define LSM6DSOX_EMB_MLC0_SRC            0x70  /* MLC0..MLC7 = 0x70..0x77 */

/****************************************************************************
 * SENSOR_HUB Bank Register Map
 * Reachable only while LSM6DSOX_BANK_SENSOR_HUB is selected.
 * Datasheet Section 11
 ****************************************************************************/

#define LSM6DSOX_SHUB_SENSOR_HUB_1       0x02  /* SENSOR_HUB_1..18 = 0x02..0x13 */
#define LSM6DSOX_SHUB_MASTER_CONFIG      0x14
#define LSM6DSOX_SHUB_SLV0_ADD           0x15
#define LSM6DSOX_SHUB_SLV0_SUBADD        0x16
#define LSM6DSOX_SHUB_SLV0_CONFIG        0x17
#define LSM6DSOX_SHUB_SLV1_ADD           0x18
#define LSM6DSOX_SHUB_SLV2_ADD           0x1B
#define LSM6DSOX_SHUB_SLV3_ADD           0x1E
#define LSM6DSOX_SHUB_DATAWRITE_SLV0     0x21
#define LSM6DSOX_SHUB_STATUS_MASTER      0x22

/****************************************************************************
 * Embedded Advanced Features Pages
 * Indirectly addressed through PAGE_SEL / PAGE_ADDRESS / PAGE_VALUE while
 * the EMB_FUNC bank is selected. Addresses are 16-bit.
 * Datasheet Section 10.2
 ****************************************************************************/

#define LSM6DSOX_ADV_FSM_LC_TIMEOUT_L    0x17A
#define LSM6DSOX_ADV_FSM_LC_TIMEOUT_H    0x17B
#define LSM6DSOX_ADV_FSM_PROGRAMS        0x17C
#define LSM6DSOX_ADV_FSM_START_ADD_L     0x17E
#define LSM6DSOX_ADV_FSM_START_ADD_H     0x17F
#define LSM6DSOX_ADV_PEDO_CMD_REG        0x183
#define LSM6DSOX_ADV_PEDO_DEB_STEPS_CONF 0x184
#define LSM6DSOX_ADV_PEDO_SC_DELTAT_L    0x1D0
#define LSM6DSOX_ADV_PEDO_SC_DELTAT_H    0x1D1

/* FSM program area base address */

#define LSM6DSOX_FSM_START_ADD           0x0400

/****************************************************************************
 * Register Bit Definitions
 ****************************************************************************/

/* FUNC_CFG_ACCESS (0x01) — Datasheet Section 9.1 */

#define LSM6DSOX_FUNC_CFG_EMB_ACCESS     0x80  /* Select EMB_FUNC bank */
#define LSM6DSOX_FUNC_CFG_SHUB_ACCESS    0x40  /* Select SENSOR_HUB bank */
#define LSM6DSOX_FUNC_CFG_OIS_FROM_UI    0x01

/* INT1_CTRL (0x0D) — Datasheet Section 9.9 */

#define LSM6DSOX_INT1_DEN_DRDY_FLAG      0x80
#define LSM6DSOX_INT1_CNT_BDR            0x40
#define LSM6DSOX_INT1_FIFO_FULL          0x20
#define LSM6DSOX_INT1_FIFO_OVR           0x10
#define LSM6DSOX_INT1_FIFO_TH            0x08
#define LSM6DSOX_INT1_BOOT               0x04
#define LSM6DSOX_INT1_DRDY_G             0x02
#define LSM6DSOX_INT1_DRDY_XL            0x01

/* INT2_CTRL (0x0E) — Datasheet Section 9.10 */

#define LSM6DSOX_INT2_CNT_BDR            0x40
#define LSM6DSOX_INT2_FIFO_FULL          0x20
#define LSM6DSOX_INT2_FIFO_OVR           0x10
#define LSM6DSOX_INT2_FIFO_TH            0x08
#define LSM6DSOX_INT2_DRDY_TEMP          0x04
#define LSM6DSOX_INT2_DRDY_G             0x02
#define LSM6DSOX_INT2_DRDY_XL            0x01

/* CTRL3_C (0x12) — Datasheet Section 9.14 */

#define LSM6DSOX_CTRL3_C_BOOT            0x80  /* Reboot memory content */
#define LSM6DSOX_CTRL3_C_BDU             0x40  /* Block data update */
#define LSM6DSOX_CTRL3_C_H_LACTIVE       0x20  /* INT pins active low */
#define LSM6DSOX_CTRL3_C_PP_OD           0x10  /* INT pins open-drain */
#define LSM6DSOX_CTRL3_C_SIM             0x08  /* SPI 3-wire mode */
#define LSM6DSOX_CTRL3_C_IF_INC          0x04  /* Auto-increment on burst */
#define LSM6DSOX_CTRL3_C_SW_RESET        0x01  /* Software reset */

/* CTRL4_C (0x13) — Datasheet Section 9.15 */

#define LSM6DSOX_CTRL4_C_SLEEP_G         0x40  /* Gyro sleep mode */
#define LSM6DSOX_CTRL4_C_INT2_ON_INT1    0x20  /* Route all INT2 to INT1 */
#define LSM6DSOX_CTRL4_C_DRDY_MASK       0x08  /* Mask DRDY on settling */
#define LSM6DSOX_CTRL4_C_I2C_DISABLE     0x04  /* SPI-only mode */
#define LSM6DSOX_CTRL4_C_LPF1_SEL_G      0x02  /* Enable gyro LPF1 */

/* CTRL5_C (0x14) self-test fields — Datasheet Section 9.16 */

#define LSM6DSOX_CTRL5_C_ST_XL_MASK      0x03
#define LSM6DSOX_CTRL5_C_ST_XL_NORMAL    0x00
#define LSM6DSOX_CTRL5_C_ST_XL_POS       0x01
#define LSM6DSOX_CTRL5_C_ST_XL_NEG       0x02
#define LSM6DSOX_CTRL5_C_ST_G_MASK       0x0C
#define LSM6DSOX_CTRL5_C_ST_G_NORMAL     0x00
#define LSM6DSOX_CTRL5_C_ST_G_POS        0x04
#define LSM6DSOX_CTRL5_C_ST_G_NEG        0x0C

/* CTRL6_C (0x15) — Datasheet Section 9.17 */

#define LSM6DSOX_CTRL6_C_XL_HM_MODE      0x10  /* 1 = disable high-perf XL */
#define LSM6DSOX_CTRL6_C_USR_OFF_W       0x08  /* Offset weight 2^-6 g/LSB */

/* CTRL7_G (0x16) — Datasheet Section 9.18 */

#define LSM6DSOX_CTRL7_G_G_HM_MODE       0x80  /* 1 = disable high-perf G */
#define LSM6DSOX_CTRL7_G_HP_EN_G         0x40  /* Gyro digital HPF enable */
#define LSM6DSOX_CTRL7_G_USR_OFF_ON_OUT  0x02  /* Apply XL offsets to output */

/* CTRL9_XL (0x18) — Datasheet Section 9.20 */

#define LSM6DSOX_CTRL9_XL_I3C_DISABLE    0x02

/* CTRL10_C (0x19) — Datasheet Section 9.21 */

#define LSM6DSOX_CTRL10_C_TIMESTAMP_EN   0x20

/* STATUS_REG (0x1E) — Datasheet Section 9.26 */

#define LSM6DSOX_STATUS_TDA              0x04  /* Temperature data ready */
#define LSM6DSOX_STATUS_GDA              0x02  /* Gyroscope data ready */
#define LSM6DSOX_STATUS_XLDA             0x01  /* Accelerometer data ready */

/* ALL_INT_SRC (0x1A) — Datasheet Section 9.22 */

#define LSM6DSOX_ALL_INT_TIMESTAMP       0x80
#define LSM6DSOX_ALL_INT_SLEEP_CHANGE    0x20
#define LSM6DSOX_ALL_INT_D6D             0x10
#define LSM6DSOX_ALL_INT_DOUBLE_TAP      0x08
#define LSM6DSOX_ALL_INT_SINGLE_TAP      0x04
#define LSM6DSOX_ALL_INT_WU              0x02
#define LSM6DSOX_ALL_INT_FF              0x01

/* FIFO_CTRL2 (0x08) — Datasheet Section 9.6 */

#define LSM6DSOX_FIFO_CTRL2_STOP_ON_WTM  0x80
#define LSM6DSOX_FIFO_CTRL2_WTM8         0x01  /* Watermark bit 8 */

/* FIFO_STATUS2 (0x3B) — Datasheet Section 9.30 */

#define LSM6DSOX_FIFO_STS2_WTM_IA        0x80  /* Watermark reached */
#define LSM6DSOX_FIFO_STS2_OVR_IA        0x40  /* Overrun */
#define LSM6DSOX_FIFO_STS2_FULL_IA       0x20  /* Full */
#define LSM6DSOX_FIFO_STS2_CNT_BDR_IA    0x10
#define LSM6DSOX_FIFO_STS2_OVR_LATCHED   0x08
#define LSM6DSOX_FIFO_STS2_DIFF_MASK     0x03  /* DIFF_FIFO [9:8] */

/* FIFO depth in tagged samples (7 bytes each) — Datasheet Section 6 */

#define LSM6DSOX_FIFO_MAX_SAMPLES        512
#define LSM6DSOX_FIFO_TAG_SAMPLE_BYTES   7

/* TAP_CFG0 (0x56) — Datasheet Section 9.38 */

#define LSM6DSOX_TAP_CFG0_INT_CLR_ON_RD  0x40
#define LSM6DSOX_TAP_CFG0_SLEEP_STATUS   0x20
#define LSM6DSOX_TAP_CFG0_SLOPE_FDS      0x10  /* Slope vs HPF for events */
#define LSM6DSOX_TAP_CFG0_TAP_X_EN       0x08
#define LSM6DSOX_TAP_CFG0_TAP_Y_EN       0x04
#define LSM6DSOX_TAP_CFG0_TAP_Z_EN       0x02
#define LSM6DSOX_TAP_CFG0_LIR            0x01  /* Latched interrupt */

/* TAP_CFG2 (0x58) — Datasheet Section 9.40 */

#define LSM6DSOX_TAP_CFG2_INTERRUPTS_EN  0x80  /* Master enable for events */

/* WAKE_UP_THS (0x5B) — Datasheet Section 9.43 */

#define LSM6DSOX_WAKE_UP_THS_SINGLE_DBL  0x80  /* Enable double-tap */
#define LSM6DSOX_WAKE_UP_THS_USR_OFF_WU  0x40
#define LSM6DSOX_WAKE_UP_THS_MASK        0x3F  /* WK_THS [5:0] */

/* WAKE_UP_SRC (0x1B) — Datasheet Section 9.23 */

#define LSM6DSOX_WAKE_UP_SRC_SLEEP_CHG   0x40
#define LSM6DSOX_WAKE_UP_SRC_FF_IA       0x20
#define LSM6DSOX_WAKE_UP_SRC_SLEEP_STATE 0x10
#define LSM6DSOX_WAKE_UP_SRC_WU_IA       0x08
#define LSM6DSOX_WAKE_UP_SRC_X_WU        0x04
#define LSM6DSOX_WAKE_UP_SRC_Y_WU        0x02
#define LSM6DSOX_WAKE_UP_SRC_Z_WU        0x01

/* TAP_SRC (0x1C) — Datasheet Section 9.24 */

#define LSM6DSOX_TAP_SRC_TAP_IA          0x40
#define LSM6DSOX_TAP_SRC_SINGLE_TAP      0x20
#define LSM6DSOX_TAP_SRC_DOUBLE_TAP      0x10
#define LSM6DSOX_TAP_SRC_TAP_SIGN        0x08
#define LSM6DSOX_TAP_SRC_X_TAP           0x04
#define LSM6DSOX_TAP_SRC_Y_TAP           0x02
#define LSM6DSOX_TAP_SRC_Z_TAP           0x01

/* D6D_SRC (0x1D) — Datasheet Section 9.25 */

#define LSM6DSOX_D6D_SRC_D6D_IA          0x40
#define LSM6DSOX_D6D_SRC_ZH              0x20
#define LSM6DSOX_D6D_SRC_ZL              0x10
#define LSM6DSOX_D6D_SRC_YH              0x08
#define LSM6DSOX_D6D_SRC_YL              0x04
#define LSM6DSOX_D6D_SRC_XH              0x02
#define LSM6DSOX_D6D_SRC_XL              0x01

/* EMB_FUNC_EN_A (0x04, EMB bank) — Datasheet Section 10.3 */

#define LSM6DSOX_EMB_EN_A_SIGN_MOTION    0x20
#define LSM6DSOX_EMB_EN_A_TILT           0x10
#define LSM6DSOX_EMB_EN_A_PEDO           0x08

/* EMB_FUNC_EN_B (0x05, EMB bank) — Datasheet Section 10.4 */

#define LSM6DSOX_EMB_EN_B_MLC            0x10
#define LSM6DSOX_EMB_EN_B_FIFO_COMPR     0x08
#define LSM6DSOX_EMB_EN_B_FSM            0x01

/* EMB_FUNC_SRC (0x64, EMB bank) — Datasheet Section 10.x */

#define LSM6DSOX_EMB_SRC_PEDO_RST_STEP   0x80
#define LSM6DSOX_EMB_SRC_STEP_DETECTED   0x20
#define LSM6DSOX_EMB_SRC_STEP_CNT_DELTA  0x10
#define LSM6DSOX_EMB_SRC_STEP_OVERFLOW   0x08

/* EMB_FUNC_INIT_A / _B (0x66 / 0x67, EMB bank) */

#define LSM6DSOX_EMB_INIT_A_PEDO         0x08
#define LSM6DSOX_EMB_INIT_A_TILT         0x10
#define LSM6DSOX_EMB_INIT_A_SIGN_MOTION  0x20
#define LSM6DSOX_EMB_INIT_B_FSM          0x01
#define LSM6DSOX_EMB_INIT_B_MLC          0x10

/* PAGE_RW (0x17, EMB bank) — Datasheet Section 10.2 */

#define LSM6DSOX_PAGE_RW_EMB_FUNC_LIR    0x80
#define LSM6DSOX_PAGE_RW_WRITE           0x40
#define LSM6DSOX_PAGE_RW_READ            0x20

/****************************************************************************
 * Full-Scale Selection Bits
 ****************************************************************************/

/* Accelerometer FS_XL[1:0], CTRL1_XL bits 3:2.
 * NOTE the non-monotonic encoding — 16g sits between 2g and 4g.
 * Datasheet Section 9.12, Table 51
 */

#define LSM6DSOX_ACCEL_FS_2G             0x00
#define LSM6DSOX_ACCEL_FS_16G            0x04
#define LSM6DSOX_ACCEL_FS_4G             0x08
#define LSM6DSOX_ACCEL_FS_8G             0x0C
#define LSM6DSOX_ACCEL_FS_MASK           0x0C

/* Gyroscope FS_G[1:0] (bits 3:2) plus FS_125 (bit 1), CTRL2_G.
 * Datasheet Section 9.13, Table 55
 */

#define LSM6DSOX_GYRO_FS_125             0x02
#define LSM6DSOX_GYRO_FS_250             0x00
#define LSM6DSOX_GYRO_FS_500             0x04
#define LSM6DSOX_GYRO_FS_1000            0x08
#define LSM6DSOX_GYRO_FS_2000            0x0C
#define LSM6DSOX_GYRO_FS_MASK            0x0E

/* ODR fields occupy bits 7:4 of CTRL1_XL / CTRL2_G */

#define LSM6DSOX_ODR_MASK                0xF0

/****************************************************************************
 * Sensitivity Scale Factors
 * Datasheet Section 4.1, Table 3
 ****************************************************************************/

/* Accelerometer, mg/LSB */

#define LSM6DSOX_ACCEL_SENS_2G           0.061f
#define LSM6DSOX_ACCEL_SENS_4G           0.122f
#define LSM6DSOX_ACCEL_SENS_8G           0.244f
#define LSM6DSOX_ACCEL_SENS_16G          0.488f

/* Gyroscope, mdps/LSB */

#define LSM6DSOX_GYRO_SENS_125           4.375f
#define LSM6DSOX_GYRO_SENS_250           8.750f
#define LSM6DSOX_GYRO_SENS_500          17.500f
#define LSM6DSOX_GYRO_SENS_1000         35.000f
#define LSM6DSOX_GYRO_SENS_2000         70.000f

/* Temperature: Temp_degC = OUT_TEMP/256 + 25
 * Datasheet Section 4.3
 */

#define LSM6DSOX_TEMP_SENSITIVITY      256.0f
#define LSM6DSOX_TEMP_OFFSET            25.0f

/* Timestamp resolution, ~25 us/LSB (nominal 40 kHz internal clock).
 * Datasheet Section 9.36
 */

#define LSM6DSOX_TIMESTAMP_RES_US       25.0f

/* Number of stationary samples averaged by lsm6dsox_calibrate() */

#define LSM6DSOX_CALIB_SAMPLES          1000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Register bank selector. Passed to lsm6dsox_set_mem_bank() to reach the
 * embedded-function or sensor-hub register pages.
 */

typedef enum
{
  LSM6DSOX_BANK_USER       = 0x00,
  LSM6DSOX_BANK_EMBEDDED   = LSM6DSOX_FUNC_CFG_EMB_ACCESS,
  LSM6DSOX_BANK_SENSOR_HUB = LSM6DSOX_FUNC_CFG_SHUB_ACCESS,
} lsm6dsox_bank_t;

/* Accelerometer full-scale range */

typedef enum
{
  LSM6DSOX_ACCEL_RANGE_2G  = LSM6DSOX_ACCEL_FS_2G,
  LSM6DSOX_ACCEL_RANGE_16G = LSM6DSOX_ACCEL_FS_16G,
  LSM6DSOX_ACCEL_RANGE_4G  = LSM6DSOX_ACCEL_FS_4G,
  LSM6DSOX_ACCEL_RANGE_8G  = LSM6DSOX_ACCEL_FS_8G,
} lsm6dsox_accel_range_t;

/* Gyroscope full-scale range */

typedef enum
{
  LSM6DSOX_GYRO_RANGE_125  = LSM6DSOX_GYRO_FS_125,
  LSM6DSOX_GYRO_RANGE_250  = LSM6DSOX_GYRO_FS_250,
  LSM6DSOX_GYRO_RANGE_500  = LSM6DSOX_GYRO_FS_500,
  LSM6DSOX_GYRO_RANGE_1000 = LSM6DSOX_GYRO_FS_1000,
  LSM6DSOX_GYRO_RANGE_2000 = LSM6DSOX_GYRO_FS_2000,
} lsm6dsox_gyro_range_t;

/* Output data rate, pre-shifted into bits 7:4.
 * LSM6DSOX_ODR_1_6HZ is accelerometer-only and valid only in low-power mode.
 * Datasheet Section 9.12, Table 50
 */

typedef enum
{
  LSM6DSOX_ODR_OFF    = 0x00,
  LSM6DSOX_ODR_12_5HZ = 0x10,
  LSM6DSOX_ODR_26HZ   = 0x20,
  LSM6DSOX_ODR_52HZ   = 0x30,
  LSM6DSOX_ODR_104HZ  = 0x40,
  LSM6DSOX_ODR_208HZ  = 0x50,
  LSM6DSOX_ODR_416HZ  = 0x60,
  LSM6DSOX_ODR_833HZ  = 0x70,
  LSM6DSOX_ODR_1667HZ = 0x80,
  LSM6DSOX_ODR_3333HZ = 0x90,
  LSM6DSOX_ODR_6667HZ = 0xA0,
  LSM6DSOX_ODR_1_6HZ  = 0xB0,
} lsm6dsox_odr_t;

/* Power mode. Selected via XL_HM_MODE (CTRL6_C) and G_HM_MODE (CTRL7_G).
 * Datasheet Section 5.3
 */

typedef enum
{
  LSM6DSOX_POWER_HIGH_PERF = 0,
  LSM6DSOX_POWER_NORMAL    = 1,
  LSM6DSOX_POWER_LOW       = 2,
} lsm6dsox_power_mode_t;

/* Interrupt pin selector, usable as a bitmask to route an event to both */

typedef enum
{
  LSM6DSOX_INT_PIN_1   = (1 << 0),
  LSM6DSOX_INT_PIN_2   = (1 << 1),
  LSM6DSOX_INT_PIN_ALL = 0x03,
} lsm6dsox_int_pin_t;

/* FIFO mode, FIFO_CTRL4 bits 2:0.
 * Datasheet Section 9.8, Table 46
 */

typedef enum
{
  LSM6DSOX_FIFO_BYPASS              = 0x00, /* FIFO disabled and flushed */
  LSM6DSOX_FIFO_MODE                = 0x01, /* Stop collecting when full */
  LSM6DSOX_FIFO_CONTINUOUS_TO_FIFO  = 0x03,
  LSM6DSOX_FIFO_BYPASS_TO_CONTINUOUS = 0x04,
  LSM6DSOX_FIFO_CONTINUOUS          = 0x06, /* Overwrite oldest sample */
  LSM6DSOX_FIFO_BYPASS_TO_FIFO      = 0x07,
} lsm6dsox_fifo_mode_t;

/* FIFO sample tag, FIFO_DATA_OUT_TAG bits 7:3.
 * Identifies the producer of the 6 data bytes that follow.
 * Datasheet Section 9.31, Table 130
 */

typedef enum
{
  LSM6DSOX_FIFO_TAG_GYRO_NC      = 0x01,
  LSM6DSOX_FIFO_TAG_ACCEL_NC     = 0x02,
  LSM6DSOX_FIFO_TAG_TEMPERATURE  = 0x03,
  LSM6DSOX_FIFO_TAG_TIMESTAMP    = 0x04,
  LSM6DSOX_FIFO_TAG_CFG_CHANGE   = 0x05,
  LSM6DSOX_FIFO_TAG_SHUB_SLAVE0  = 0x0E,
  LSM6DSOX_FIFO_TAG_SHUB_SLAVE1  = 0x0F,
  LSM6DSOX_FIFO_TAG_SHUB_SLAVE2  = 0x10,
  LSM6DSOX_FIFO_TAG_SHUB_SLAVE3  = 0x11,
  LSM6DSOX_FIFO_TAG_STEP_COUNTER = 0x12,
} lsm6dsox_fifo_tag_t;

/* Hardware event routing mask. Values match the MD1_CFG / MD2_CFG bit
 * layout exactly, so a set of these OR'ed together can be written directly.
 * Datasheet Section 9.45
 *
 * Pedometer, tilt and significant-motion do not have dedicated bits: they
 * are reported through LSM6DSOX_EVENT_EMB_FUNC and further discriminated by
 * EMB_FUNC_STATUS.
 */

typedef enum
{
  LSM6DSOX_EVENT_SLEEP_CHANGE = (1 << 7),
  LSM6DSOX_EVENT_SINGLE_TAP   = (1 << 6),
  LSM6DSOX_EVENT_WAKEUP       = (1 << 5),
  LSM6DSOX_EVENT_FREE_FALL    = (1 << 4),
  LSM6DSOX_EVENT_DOUBLE_TAP   = (1 << 3),
  LSM6DSOX_EVENT_6D_ORIENT    = (1 << 2),
  LSM6DSOX_EVENT_EMB_FUNC     = (1 << 1),
  LSM6DSOX_EVENT_SHUB_OR_TS   = (1 << 0),
} lsm6dsox_hw_event_t;

/* Raw sensor data, straight from the registers with no scaling */

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp_raw;
} lsm6dsox_raw_data_t;

/* Scaled sensor data in physical units */

typedef struct
{
  float accel_x;  /* Accelerometer X [g] */
  float accel_y;  /* Accelerometer Y [g] */
  float accel_z;  /* Accelerometer Z [g] */
  float gyro_x;   /* Gyroscope X [deg/s] */
  float gyro_y;   /* Gyroscope Y [deg/s] */
  float gyro_z;   /* Gyroscope Z [deg/s] */
  float temp_c;   /* Temperature [Celsius] */
} lsm6dsox_data_t;

/* Bias offsets produced by lsm6dsox_calibrate().
 *
 * The accelerometer offsets map onto the X/Y/Z_OFS_USR hardware registers.
 * The gyroscope has no offset registers on this part, so gyro bias is kept
 * in the driver and subtracted from every raw read.
 */

typedef struct
{
  int8_t  accel_x;  /* Hardware offset, weight per USR_OFF_W */
  int8_t  accel_y;
  int8_t  accel_z;
  int16_t gyro_x;   /* Software bias in raw LSB */
  int16_t gyro_y;
  int16_t gyro_z;
} lsm6dsox_offsets_t;

/* One decoded FIFO record */

typedef struct
{
  lsm6dsox_fifo_tag_t tag;  /* Which sensor produced this sample */
  int16_t             x;
  int16_t             y;
  int16_t             z;
} lsm6dsox_fifo_sample_t;

/* One line of an ST Unico-generated .ucf configuration file.
 * Used to program the MLC and the FSM.
 * Reference: AN5272 Section 8
 */

typedef struct
{
  uint8_t address;
  uint8_t data;
} lsm6dsox_ucf_line_t;

/* Interrupt callback signature.
 * Called from ISR context — keep it minimal (set a flag or post to a
 * FreeRTOS queue; never block, never do I2C from here).
 */

typedef void (*lsm6dsox_isr_cb_t)(void *arg);

/* Driver configuration structure.
 *
 * i2c_addr is I2C-specific and would be replaced by a transport descriptor
 * if SPI or I3C support is ever added; see the TRANSPORT note at the top of
 * this file.
 */

typedef struct
{
  uint8_t                i2c_addr;    /* I2C address (default 0x6A) */
  lsm6dsox_accel_range_t accel_range; /* Accelerometer full scale */
  lsm6dsox_gyro_range_t  gyro_range;  /* Gyroscope full scale */
  lsm6dsox_odr_t         accel_odr;   /* Accelerometer output data rate */
  lsm6dsox_odr_t         gyro_odr;    /* Gyroscope output data rate */
  bool                   bdu_enable;  /* Block data update (recommended) */
  bool                   int_enable;  /* Route data-ready to an INT pin */
  lsm6dsox_int_pin_t     int_pin;     /* Which pin data-ready uses */
  lsm6dsox_isr_cb_t      isr_cb;      /* ISR callback (NULL = none) */
  void                  *isr_arg;     /* ISR callback argument */
} lsm6dsox_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* ==========================================================================
 * L0 — Register Access
 *
 * Exposed so that any chip feature can be reached from application code
 * before this driver grows a typed wrapper for it.
 *
 * This is the ONLY layer that touches the bus. It is I2C-only today; see the
 * TRANSPORT note at the top of this file. Adding SPI or I3C later means
 * reimplementing these five calls and nothing else.
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_read_reg
 *
 * Description:
 *   Read a single register from the currently selected bank.
 *
 * Input Parameters:
 *   reg - Register address
 *   val - Pointer to store the value read (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_read_reg(uint8_t reg, uint8_t *val);

/****************************************************************************
 * Name: lsm6dsox_write_reg
 *
 * Description:
 *   Write a single register in the currently selected bank.
 *
 * Input Parameters:
 *   reg - Register address
 *   val - Value to write
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_write_reg(uint8_t reg, uint8_t val);

/****************************************************************************
 * Name: lsm6dsox_modify_reg
 *
 * Description:
 *   Read-modify-write a register: bits set in mask are replaced by the
 *   corresponding bits of val, all other bits are preserved.
 *
 * Input Parameters:
 *   reg  - Register address
 *   mask - Bits to modify
 *   val  - New values for the masked bits (already in position)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_modify_reg(uint8_t reg, uint8_t mask, uint8_t val);

/****************************************************************************
 * Name: lsm6dsox_read_burst
 *
 * Description:
 *   Read consecutive registers in one transaction. Requires IF_INC
 *   (CTRL3_C bit 2), which lsm6dsox_init() leaves enabled.
 *
 * Input Parameters:
 *   reg - First register address
 *   buf - Destination buffer (must not be NULL)
 *   len - Number of bytes to read
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_read_burst(uint8_t reg, uint8_t *buf, size_t len);

/****************************************************************************
 * Name: lsm6dsox_set_mem_bank
 *
 * Description:
 *   Select which register bank subsequent accesses address, by writing
 *   FUNC_CFG_ACCESS (0x01).
 *
 *   Every access to an embedded-function or sensor-hub register must be
 *   bracketed by a switch to the target bank and a switch back to
 *   LSM6DSOX_BANK_USER. Leaving a non-user bank selected makes all normal
 *   data reads return garbage.
 *
 * Input Parameters:
 *   bank - Bank to select
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_mem_bank(lsm6dsox_bank_t bank);

/* ==========================================================================
 * L1 — Core: Initialization and Power
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_init
 *
 * Description:
 *   Initialize the LSM6DSOX: register the I2C device on the MAIA bus,
 *   verify WHO_AM_I, software reset, enable IF_INC and optionally BDU,
 *   apply the configured ranges and output data rates, and set up the
 *   data-ready interrupt on MAIA_GPIO_IMU_INT when requested.
 *
 * Input Parameters:
 *   config - Driver configuration structure (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_NOT_FOUND if WHO_AM_I mismatches;
 *   error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_init(const lsm6dsox_config_t *config);

/****************************************************************************
 * Name: lsm6dsox_deinit
 *
 * Description:
 *   De-initialize the driver: remove the ISR, power down both sensors and
 *   remove the I2C device handle.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_deinit(void);

/****************************************************************************
 * Name: lsm6dsox_who_am_i
 *
 * Description:
 *   Read the WHO_AM_I register. Expected value: 0x6C.
 *
 * Input Parameters:
 *   val - Pointer to store the register value (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_who_am_i(uint8_t *val);

/****************************************************************************
 * Name: lsm6dsox_reset
 *
 * Description:
 *   Issue a software reset (CTRL3_C SW_RESET) and block until the device
 *   clears the bit, restoring all registers to their power-on defaults.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_TIMEOUT if the bit never clears
 *
 ****************************************************************************/

esp_err_t lsm6dsox_reset(void);

/****************************************************************************
 * Name: lsm6dsox_set_accel_range
 *
 * Description:
 *   Change the accelerometer full-scale range at runtime and update the
 *   cached sensitivity used for scaling.
 *
 * Input Parameters:
 *   range - New accelerometer range
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_accel_range(lsm6dsox_accel_range_t range);

/****************************************************************************
 * Name: lsm6dsox_set_gyro_range
 *
 * Description:
 *   Change the gyroscope full-scale range at runtime and update the cached
 *   sensitivity used for scaling.
 *
 * Input Parameters:
 *   range - New gyroscope range
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_gyro_range(lsm6dsox_gyro_range_t range);

/****************************************************************************
 * Name: lsm6dsox_set_accel_odr
 *
 * Description:
 *   Change the accelerometer output data rate at runtime.
 *   LSM6DSOX_ODR_OFF powers the accelerometer down.
 *
 * Input Parameters:
 *   odr - New output data rate
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_accel_odr(lsm6dsox_odr_t odr);

/****************************************************************************
 * Name: lsm6dsox_set_gyro_odr
 *
 * Description:
 *   Change the gyroscope output data rate at runtime.
 *   LSM6DSOX_ODR_OFF powers the gyroscope down. LSM6DSOX_ODR_1_6HZ is not
 *   valid for the gyroscope.
 *
 * Input Parameters:
 *   odr - New output data rate
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG for an unsupported rate
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_gyro_odr(lsm6dsox_odr_t odr);

/****************************************************************************
 * Name: lsm6dsox_set_power_mode
 *
 * Description:
 *   Select high-performance, normal or low-power operation for both
 *   sensors, via XL_HM_MODE (CTRL6_C) and G_HM_MODE (CTRL7_G).
 *
 * Input Parameters:
 *   mode - Desired power mode
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_power_mode(lsm6dsox_power_mode_t mode);

/****************************************************************************
 * Name: lsm6dsox_sleep
 *
 * Description:
 *   Power down both sensors by setting their ODR fields to zero. The
 *   configured rates are remembered so lsm6dsox_wakeup() can restore them.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_sleep(void);

/****************************************************************************
 * Name: lsm6dsox_wakeup
 *
 * Description:
 *   Restore the output data rates that were active before lsm6dsox_sleep().
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_wakeup(void);

/* ==========================================================================
 * L2 — Polled Data Acquisition
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_get_status
 *
 * Description:
 *   Read STATUS_REG (0x1E) to test data availability without consuming a
 *   sample. Compare against LSM6DSOX_STATUS_XLDA / _GDA / _TDA.
 *
 * Input Parameters:
 *   status - Pointer to store the register value (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_get_status(uint8_t *status);

/****************************************************************************
 * Name: lsm6dsox_read_raw
 *
 * Description:
 *   Read temperature, gyroscope and accelerometer as raw 16-bit values in a
 *   single 14-byte burst starting at OUT_TEMP_L (0x20). Output words are
 *   little-endian. Any gyroscope bias from lsm6dsox_calibrate() is applied.
 *
 * Input Parameters:
 *   data - Pointer to raw data structure (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_read_raw(lsm6dsox_raw_data_t *data);

/****************************************************************************
 * Name: lsm6dsox_read_scaled
 *
 * Description:
 *   Read and convert sensor data to physical units (g, deg/s, Celsius).
 *
 * Input Parameters:
 *   data - Pointer to scaled data structure (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_read_scaled(lsm6dsox_data_t *data);

/****************************************************************************
 * Name: lsm6dsox_read_temperature
 *
 * Description:
 *   Read only the temperature sensor, in degrees Celsius.
 *
 * Input Parameters:
 *   temp_c - Pointer to store the temperature (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_read_temperature(float *temp_c);

/* ==========================================================================
 * L3 — Data-Ready Interrupt
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_set_drdy_int
 *
 * Description:
 *   Route the accelerometer and/or gyroscope data-ready signal to an
 *   interrupt pin, by writing INT1_CTRL (0x0D) or INT2_CTRL (0x0E).
 *
 * Input Parameters:
 *   pin       - Which interrupt pin to drive
 *   accel_en  - Route the accelerometer data-ready signal
 *   gyro_en   - Route the gyroscope data-ready signal
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_drdy_int(lsm6dsox_int_pin_t pin,
                                bool               accel_en,
                                bool               gyro_en);

/****************************************************************************
 * Name: lsm6dsox_set_int_notification
 *
 * Description:
 *   Configure interrupt pin electrical behaviour and latching: active
 *   level and push-pull versus open-drain (CTRL3_C), and latched versus
 *   pulsed event interrupts (TAP_CFG0 LIR).
 *
 * Input Parameters:
 *   active_low  - true for active-low INT pins
 *   open_drain  - true for open-drain INT pins
 *   latched     - true to latch event interrupts until the source is read
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_int_notification(bool active_low,
                                        bool open_drain,
                                        bool latched);

/****************************************************************************
 * Name: lsm6dsox_get_all_int_src
 *
 * Description:
 *   Read ALL_INT_SRC (0x1A), the aggregated event source register. Reading
 *   it clears latched event interrupts. Compare against the
 *   LSM6DSOX_ALL_INT_* bit definitions.
 *
 * Input Parameters:
 *   src - Pointer to store the register value (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_get_all_int_src(uint8_t *src);

/* ==========================================================================
 * L4 — Calibration and Self-Test
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_calibrate
 *
 * Description:
 *   Average LSM6DSOX_CALIB_SAMPLES readings with the device flat and
 *   stationary to derive bias offsets.
 *
 *   Gyroscope bias is stored in the driver and subtracted on every
 *   subsequent read — this part has no gyroscope offset registers.
 *   Accelerometer bias is written to X/Y/Z_OFS_USR and applied by the
 *   hardware once USR_OFF_ON_OUT (CTRL7_G) is set.
 *
 *   The device must be flat and still for the whole call.
 *
 * Input Parameters:
 *   offsets - Optional pointer to store the computed offsets (NULL to
 *             discard)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_calibrate(lsm6dsox_offsets_t *offsets);

/****************************************************************************
 * Name: lsm6dsox_set_offsets
 *
 * Description:
 *   Apply previously computed offsets without re-running calibration, so a
 *   calibration stored in NVS can be restored at boot.
 *
 * Input Parameters:
 *   offsets - Offsets to apply (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_set_offsets(const lsm6dsox_offsets_t *offsets);

/****************************************************************************
 * Name: lsm6dsox_self_test
 *
 * Description:
 *   Run the factory self-test procedure of Datasheet Section 6.5: capture
 *   an averaged baseline, enable the electrostatic self-test actuation,
 *   capture a second average, and verify that the difference falls inside
 *   the datasheet min/max window for both sensors.
 *
 *   Blocks for roughly one second and disturbs normal acquisition.
 *
 * Returned Value:
 *   ESP_OK if both sensors pass; ESP_FAIL otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_self_test(void);

/* ==========================================================================
 * L5 — FIFO
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_fifo_config
 *
 * Description:
 *   Configure FIFO mode and watermark level (FIFO_CTRL1..4). The watermark
 *   is expressed in samples and spans 9 bits, so it is split across
 *   FIFO_CTRL1 and bit 0 of FIFO_CTRL2.
 *
 * Input Parameters:
 *   mode      - FIFO operating mode
 *   watermark - Watermark threshold in samples (0..511)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if watermark exceeds 511
 *
 ****************************************************************************/

esp_err_t lsm6dsox_fifo_config(lsm6dsox_fifo_mode_t mode,
                               uint16_t             watermark);

/****************************************************************************
 * Name: lsm6dsox_fifo_set_batch_rate
 *
 * Description:
 *   Set the per-sensor batch data rates written into the FIFO (FIFO_CTRL3).
 *   These are independent of the sensor output data rates; a sensor batched
 *   at LSM6DSOX_ODR_OFF is not stored in the FIFO at all.
 *
 * Input Parameters:
 *   accel_bdr - Accelerometer batch data rate
 *   gyro_bdr  - Gyroscope batch data rate
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_fifo_set_batch_rate(lsm6dsox_odr_t accel_bdr,
                                       lsm6dsox_odr_t gyro_bdr);

/****************************************************************************
 * Name: lsm6dsox_fifo_get_level
 *
 * Description:
 *   Read the number of unread samples currently in the FIFO, from the
 *   10-bit DIFF_FIFO field spanning FIFO_STATUS1 and FIFO_STATUS2.
 *
 * Input Parameters:
 *   samples - Pointer to store the sample count (must not be NULL)
 *   flags   - Optional pointer to store the FIFO_STATUS2 flag bits
 *             (NULL to discard); compare against LSM6DSOX_FIFO_STS2_*
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_fifo_get_level(uint16_t *samples, uint8_t *flags);

/****************************************************************************
 * Name: lsm6dsox_fifo_read
 *
 * Description:
 *   Pop up to max_samples tagged records from the FIFO, decoding the tag
 *   byte and the three little-endian data words of each 7-byte record.
 *
 * Input Parameters:
 *   samples     - Destination array (must not be NULL)
 *   max_samples - Capacity of the destination array
 *   read_count  - Pointer to store how many records were decoded
 *                 (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_fifo_read(lsm6dsox_fifo_sample_t *samples,
                             uint16_t                max_samples,
                             uint16_t               *read_count);

/****************************************************************************
 * Name: lsm6dsox_fifo_flush
 *
 * Description:
 *   Discard all FIFO contents by cycling through bypass mode and restoring
 *   the previously configured mode.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_fifo_flush(void);

/* ==========================================================================
 * L6 — Hardware Events: Wake-Up, Free-Fall, 6D, Tap, Activity
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_route_hw_event
 *
 * Description:
 *   Route one or more hardware events to an interrupt pin, by setting the
 *   matching bits of MD1_CFG (0x5E) and/or MD2_CFG (0x5F). The
 *   lsm6dsox_hw_event_t values are already bit-aligned with those
 *   registers, so several events may be OR'ed together in one call.
 *
 *   Note that events only reach the pins once INTERRUPTS_ENABLE
 *   (TAP_CFG2 bit 7) is set — lsm6dsox_config_wakeup() and the other event
 *   configuration calls set it.
 *
 * Input Parameters:
 *   events - Bitmask of events to route
 *   pin    - Which interrupt pin to drive
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_route_hw_event(lsm6dsox_hw_event_t events,
                                  lsm6dsox_int_pin_t  pin);

/****************************************************************************
 * Name: lsm6dsox_config_wakeup
 *
 * Description:
 *   Configure the wake-up (activity) detector: threshold in WAKE_UP_THS
 *   (0x5B) and duration in WAKE_UP_DUR (0x5C).
 *
 *   Threshold resolution is FS_XL/64 per LSB, or FS_XL/256 when
 *   WAKE_THS_W is set. Duration counts ODR periods.
 *
 * Input Parameters:
 *   threshold - Wake-up threshold, 6 bits (0..63)
 *   duration  - Number of ODR periods above threshold, 2 bits (0..3)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if a field is out of range
 *
 ****************************************************************************/

esp_err_t lsm6dsox_config_wakeup(uint8_t threshold, uint8_t duration);

/****************************************************************************
 * Name: lsm6dsox_config_free_fall
 *
 * Description:
 *   Configure the free-fall detector (FREE_FALL 0x5D plus FF_DUR5 in
 *   WAKE_UP_DUR). Threshold encodes 156 mg to 500 mg.
 *
 * Input Parameters:
 *   threshold - FF_THS field, 3 bits (0..7)
 *   duration  - Free-fall duration in ODR periods, 6 bits (0..63)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if a field is out of range
 *
 ****************************************************************************/

esp_err_t lsm6dsox_config_free_fall(uint8_t threshold, uint8_t duration);

/****************************************************************************
 * Name: lsm6dsox_config_tap
 *
 * Description:
 *   Configure single and double-tap detection: per-axis enables in
 *   TAP_CFG0, per-axis thresholds in TAP_CFG1/TAP_CFG2/TAP_THS_6D, and the
 *   shock/quiet/duration timing window in INT_DUR2.
 *
 * Input Parameters:
 *   axis_mask  - Bitwise OR of LSM6DSOX_TAP_CFG0_TAP_X_EN / _Y_EN / _Z_EN
 *   threshold  - Tap threshold applied to every enabled axis, 5 bits (0..31)
 *   double_tap - Enable double-tap recognition in addition to single tap
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if a field is out of range
 *
 ****************************************************************************/

esp_err_t lsm6dsox_config_tap(uint8_t axis_mask,
                              uint8_t threshold,
                              bool    double_tap);

/****************************************************************************
 * Name: lsm6dsox_config_6d
 *
 * Description:
 *   Configure 6D/4D orientation detection: the SIXD_THS threshold angle in
 *   TAP_THS_6D (0x59) and the D4D_EN mode bit.
 *
 * Input Parameters:
 *   threshold - SIXD_THS field, 2 bits: 0=80deg 1=70deg 2=60deg 3=50deg
 *   four_d    - true for 4D mode (Z axis ignored), false for 6D
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if threshold exceeds 3
 *
 ****************************************************************************/

esp_err_t lsm6dsox_config_6d(uint8_t threshold, bool four_d);

/****************************************************************************
 * Name: lsm6dsox_get_event_src
 *
 * Description:
 *   Read the three event source registers WAKE_UP_SRC (0x1B), TAP_SRC
 *   (0x1C) and D6D_SRC (0x1D) in one burst, to determine which event fired
 *   and on which axis. Reading them clears latched interrupts.
 *
 *   Any pointer may be NULL if that source is not of interest.
 *
 * Input Parameters:
 *   wake_src - Pointer to store WAKE_UP_SRC (NULL to discard)
 *   tap_src  - Pointer to store TAP_SRC (NULL to discard)
 *   d6d_src  - Pointer to store D6D_SRC (NULL to discard)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_get_event_src(uint8_t *wake_src,
                                 uint8_t *tap_src,
                                 uint8_t *d6d_src);

/* ==========================================================================
 * L7 — Embedded Functions: Pedometer, Tilt, Significant Motion
 *
 * These live in the EMB_FUNC bank; each call brackets its own bank switch.
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_pedometer_enable
 *
 * Description:
 *   Enable or disable the step counter (EMB_FUNC_EN_A PEDO_EN, plus
 *   PEDO_INIT in EMB_FUNC_INIT_A). The accelerometer must be running at
 *   26 Hz or above for the pedometer to work.
 *
 * Input Parameters:
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_pedometer_enable(bool enable);

/****************************************************************************
 * Name: lsm6dsox_pedometer_read
 *
 * Description:
 *   Read the 16-bit step count from STEP_COUNTER_L/H (0x62/0x63 in the
 *   EMB_FUNC bank).
 *
 * Input Parameters:
 *   steps - Pointer to store the step count (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_pedometer_read(uint16_t *steps);

/****************************************************************************
 * Name: lsm6dsox_pedometer_reset
 *
 * Description:
 *   Reset the step counter to zero via PEDO_RST_STEP in EMB_FUNC_SRC.
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_pedometer_reset(void);

/****************************************************************************
 * Name: lsm6dsox_tilt_enable
 *
 * Description:
 *   Enable or disable tilt detection (EMB_FUNC_EN_A TILT_EN). The event is
 *   reported through LSM6DSOX_EVENT_EMB_FUNC.
 *
 * Input Parameters:
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_tilt_enable(bool enable);

/****************************************************************************
 * Name: lsm6dsox_significant_motion_enable
 *
 * Description:
 *   Enable or disable significant-motion detection (EMB_FUNC_EN_A
 *   SIGN_MOTION_EN). The event is reported through
 *   LSM6DSOX_EVENT_EMB_FUNC.
 *
 * Input Parameters:
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_significant_motion_enable(bool enable);

/****************************************************************************
 * Name: lsm6dsox_get_emb_func_status
 *
 * Description:
 *   Read EMB_FUNC_STATUS_MAINPAGE (0x35) to discriminate which embedded
 *   function raised LSM6DSOX_EVENT_EMB_FUNC. Uses the USER-bank mirror, so
 *   no bank switch is performed.
 *
 * Input Parameters:
 *   status - Pointer to store the register value (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_get_emb_func_status(uint8_t *status);

/* ==========================================================================
 * L8 — Machine Learning Core and Finite State Machine
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_ucf_load
 *
 * Description:
 *   Play back an ST Unico-generated .ucf configuration, which is a flat
 *   list of register/value pairs that programs the MLC decision trees
 *   and/or the FSM programs.
 *
 *   The .ucf stream drives its own bank switching through FUNC_CFG_ACCESS
 *   writes, so this function must not second-guess it: write the pairs
 *   verbatim and in order.
 *
 * Input Parameters:
 *   ucf_data - Array of register/value pairs (must not be NULL)
 *   lines    - Number of entries in the array
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_ucf_load(const lsm6dsox_ucf_line_t *ucf_data,
                            size_t                     lines);

/****************************************************************************
 * Name: lsm6dsox_mlc_get_status
 *
 * Description:
 *   Read MLC_STATUS_MAINPAGE (0x38), whose bits 0..7 flag which of the
 *   eight MLC decision trees produced a new result. Uses the USER-bank
 *   mirror, so no bank switch is performed.
 *
 * Input Parameters:
 *   status - Pointer to store the register value (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_mlc_get_status(uint8_t *status);

/****************************************************************************
 * Name: lsm6dsox_mlc_get_output
 *
 * Description:
 *   Read the classification result of one MLC decision tree, from
 *   MLC0_SRC + mlc_num (0x70..0x77 in the EMB_FUNC bank). The meaning of
 *   the value is defined by the .ucf that was loaded.
 *
 * Input Parameters:
 *   mlc_num      - Decision tree index (0..7)
 *   class_result - Pointer to store the class value (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if mlc_num exceeds 7
 *
 ****************************************************************************/

esp_err_t lsm6dsox_mlc_get_output(uint8_t mlc_num, uint8_t *class_result);

/****************************************************************************
 * Name: lsm6dsox_fsm_get_status
 *
 * Description:
 *   Read the 16 FSM status flags from FSM_STATUS_A_MAINPAGE (0x36) and
 *   FSM_STATUS_B_MAINPAGE (0x37), returned as one 16-bit mask with FSM1 in
 *   bit 0. Uses the USER-bank mirrors, so no bank switch is performed.
 *
 * Input Parameters:
 *   status - Pointer to store the combined mask (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_fsm_get_status(uint16_t *status);

/****************************************************************************
 * Name: lsm6dsox_fsm_get_output
 *
 * Description:
 *   Read the output byte of one finite state machine, from FSM_OUTS1 +
 *   fsm_num (0x4C..0x5B in the EMB_FUNC bank).
 *
 * Input Parameters:
 *   fsm_num - State machine index (0..15)
 *   output  - Pointer to store the output byte (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if fsm_num exceeds 15
 *
 ****************************************************************************/

esp_err_t lsm6dsox_fsm_get_output(uint8_t fsm_num, uint8_t *output);

/* ==========================================================================
 * L9 — Timestamp
 * ========================================================================== */

/****************************************************************************
 * Name: lsm6dsox_timestamp_enable
 *
 * Description:
 *   Enable or disable the 32-bit hardware timestamp counter
 *   (CTRL10_C TIMESTAMP_EN).
 *
 * Input Parameters:
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_timestamp_enable(bool enable);

/****************************************************************************
 * Name: lsm6dsox_timestamp_read
 *
 * Description:
 *   Read the 32-bit timestamp counter from TIMESTAMP0..3 (0x40..0x43).
 *   One LSB is nominally 25 us; INTERNAL_FREQ_FINE (0x63) carries the trim
 *   needed to correct that figure.
 *
 * Input Parameters:
 *   timestamp - Pointer to store the raw counter (must not be NULL)
 *
 * Returned Value:
 *   ESP_OK on success; error code otherwise
 *
 ****************************************************************************/

esp_err_t lsm6dsox_timestamp_read(uint32_t *timestamp);

#ifdef __cplusplus
}
#endif

#endif /* __COMPONENTS_DRIVERS_LSM6DSOX_INCLUDE_LSM6DSOX_H */
