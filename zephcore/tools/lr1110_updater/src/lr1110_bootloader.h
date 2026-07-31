/*!
 * @file      lr1110_bootloader.h
 *
 * @brief     Bootloader driver definition for LR1110
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LR1110_BOOTLOADER_H
#define LR1110_BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "lr11xx_types.h"

/* Compatibility aliases — the older lr1110_driver used lr1110_status_t,
 * the newer lr11xx_driver (SWDR001) uses lr11xx_status_t.
 * Both are the same enum: OK=0, ERROR=3. */
typedef lr11xx_status_t lr1110_status_t;
#define LR1110_STATUS_OK    LR11XX_STATUS_OK
#define LR1110_STATUS_ERROR LR11XX_STATUS_ERROR

/* ── Types ─────────────────────────────────────────────────── */

#define LR1110_BL_VERSION_LENGTH   4
#define LR1110_BL_HASH_LENGTH      16
#define LR1110_BL_PIN_LENGTH       4
#define LR1110_BL_CHIP_EUI_LENGTH  8
#define LR1110_BL_JOIN_EUI_LENGTH  8

typedef uint8_t lr1110_bootloader_hash_t[LR1110_BL_HASH_LENGTH];
typedef uint8_t lr1110_bootloader_pin_t[LR1110_BL_PIN_LENGTH];
typedef uint8_t lr1110_bootloader_chip_eui_t[LR1110_BL_CHIP_EUI_LENGTH];
typedef uint8_t lr1110_bootloader_join_eui_t[LR1110_BL_JOIN_EUI_LENGTH];

typedef uint32_t lr1110_bootloader_irq_mask_t;

typedef enum {
	LR1110_BOOTLOADER_CMD_STATUS_FAIL = 0x00,
	LR1110_BOOTLOADER_CMD_STATUS_PERR = 0x01,
	LR1110_BOOTLOADER_CMD_STATUS_OK   = 0x02,
	LR1110_BOOTLOADER_CMD_STATUS_DATA = 0x03,
} lr1110_bootloader_command_status_t;

typedef enum {
	LR1110_BOOTLOADER_CHIP_MODE_SLEEP     = 0x00,
	LR1110_BOOTLOADER_CHIP_MODE_STBY_RC   = 0x01,
	LR1110_BOOTLOADER_CHIP_MODE_STBY_XOSC = 0x02,
	LR1110_BOOTLOADER_CHIP_MODE_FS        = 0x03,
	LR1110_BOOTLOADER_CHIP_MODE_RX        = 0x04,
	LR1110_BOOTLOADER_CHIP_MODE_TX        = 0x05,
	LR1110_BOOTLOADER_CHIP_MODE_LOC       = 0x06,
} lr1110_bootloader_chip_modes_t;

typedef enum {
	LR1110_BOOTLOADER_RESET_STATUS_CLEARED      = 0x00,
	LR1110_BOOTLOADER_RESET_STATUS_ANALOG       = 0x01,
	LR1110_BOOTLOADER_RESET_STATUS_EXTERNAL     = 0x02,
	LR1110_BOOTLOADER_RESET_STATUS_SYSTEM       = 0x03,
	LR1110_BOOTLOADER_RESET_STATUS_WATCHDOG     = 0x04,
	LR1110_BOOTLOADER_RESET_STATUS_IOCD_RESTART = 0x05,
	LR1110_BOOTLOADER_RESET_STATUS_RTC_RESTART  = 0x06,
} lr1110_bootloader_reset_status_t;

typedef struct {
	lr1110_bootloader_command_status_t command_status;
	bool                               is_interrupt_active;
} lr1110_bootloader_stat1_t;

typedef struct {
	lr1110_bootloader_reset_status_t reset_status;
	lr1110_bootloader_chip_modes_t   chip_mode;
	bool                             is_running_from_flash;
} lr1110_bootloader_stat2_t;

typedef struct {
	uint8_t  hw;
	uint8_t  type;
	uint16_t fw;
} lr1110_bootloader_version_t;

/* ── Functions ─────────────────────────────────────────────── */

lr1110_status_t lr1110_bootloader_get_status(const void *context,
	lr1110_bootloader_stat1_t *stat1,
	lr1110_bootloader_stat2_t *stat2,
	lr1110_bootloader_irq_mask_t *irq_status);

lr1110_status_t lr1110_bootloader_get_version(const void *context,
	lr1110_bootloader_version_t *version);

lr1110_status_t lr1110_bootloader_erase_flash(const void *context);

/* 16-byte digest of the image currently in flash (opcode 0x8004). Lets the
 * host confirm what actually landed, rather than trusting fire-and-forget
 * WriteFlashEncrypted calls that never read back. */
lr1110_status_t lr1110_bootloader_get_hash(const void *context,
	lr1110_bootloader_hash_t hash);

/* TCXO supply voltage codes for lr1110_bootloader_set_tcxo_mode() */
#define LR1110_TCXO_CTRL_1_6V 0x00
#define LR1110_TCXO_CTRL_1_7V 0x01
#define LR1110_TCXO_CTRL_1_8V 0x02
#define LR1110_TCXO_CTRL_2_2V 0x03
#define LR1110_TCXO_CTRL_2_4V 0x04
#define LR1110_TCXO_CTRL_2_7V 0x05
#define LR1110_TCXO_CTRL_3_0V 0x06
#define LR1110_TCXO_CTRL_3_3V 0x07

/* Calibrate all blocks (LF_RC|HF_RC|PLL|ADC|IMG|PLL_TX) */
#define LR1110_CALIB_ALL 0x3F

/* Power the TCXO from DIO3 and use it as the XOSC source. `timeout` is in
 * 32.768 kHz RTC ticks (1 tick = 30.52 us). Must be followed by a
 * calibration — changing the clock source invalidates the previous one. */
lr1110_status_t lr1110_bootloader_set_tcxo_mode(const void *context,
	uint8_t tune, uint32_t timeout);

lr1110_status_t lr1110_bootloader_calibrate(const void *context, uint8_t calib_param);

lr1110_status_t lr1110_bootloader_write_flash_encrypted(const void *context,
	uint32_t offset, const uint32_t *data, uint8_t length);

lr1110_status_t lr1110_bootloader_write_flash_encrypted_full(const void *context,
	uint32_t offset, const uint32_t *buffer, uint32_t length);

lr1110_status_t lr1110_bootloader_reboot(const void *context,
	bool stay_in_bootloader);

lr1110_status_t lr1110_bootloader_read_pin(const void *context,
	lr1110_bootloader_pin_t pin);

lr1110_status_t lr1110_bootloader_read_chip_eui(const void *context,
	lr1110_bootloader_chip_eui_t chip_eui);

lr1110_status_t lr1110_bootloader_read_join_eui(const void *context,
	lr1110_bootloader_join_eui_t join_eui);

#ifdef __cplusplus
}
#endif

#endif /* LR1110_BOOTLOADER_H */
