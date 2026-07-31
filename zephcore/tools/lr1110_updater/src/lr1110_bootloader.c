/*!
 * @file      lr1110_bootloader.c
 *
 * @brief     Bootloader driver implementation for LR1110
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

/*
 * Source: https://github.com/Lora-net/lr1110_driver
 * Adapted for ZephCore updater tool — only include path changes.
 */

#include "lr1110_bootloader.h"
#include "lr11xx_hal.h"

/* ── Constants ─────────────────────────────────────────────── */

#define LR1110_FLASH_DATA_MAX_LENGTH_UINT32 (64)
#define LR1110_FLASH_DATA_MAX_LENGTH_UINT8  (LR1110_FLASH_DATA_MAX_LENGTH_UINT32 * 4)

#define LR1110_BL_CMD_NO_PARAM_LENGTH           (2)
#define LR1110_BL_GET_STATUS_CMD_LENGTH         (2 + 4)
#define LR1110_BL_VERSION_CMD_LENGTH            LR1110_BL_CMD_NO_PARAM_LENGTH
#define LR1110_BL_ERASE_FLASH_CMD_LENGTH        LR1110_BL_CMD_NO_PARAM_LENGTH
#define LR1110_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH (LR1110_BL_CMD_NO_PARAM_LENGTH + 4)
#define LR1110_BL_REBOOT_CMD_LENGTH             (LR1110_BL_CMD_NO_PARAM_LENGTH + 1)
#define LR1110_BL_GET_HASH_CMD_LENGTH           LR1110_BL_CMD_NO_PARAM_LENGTH
#define LR1110_BL_SET_TCXO_MODE_CMD_LENGTH      (LR1110_BL_CMD_NO_PARAM_LENGTH + 4)
#define LR1110_BL_CALIBRATE_CMD_LENGTH          (LR1110_BL_CMD_NO_PARAM_LENGTH + 1)
#define LR1110_BL_GET_PIN_CMD_LENGTH            LR1110_BL_CMD_NO_PARAM_LENGTH
#define LR1110_BL_READ_CHIP_EUI_CMD_LENGTH      LR1110_BL_CMD_NO_PARAM_LENGTH
#define LR1110_BL_READ_JOIN_EUI_CMD_LENGTH      LR1110_BL_CMD_NO_PARAM_LENGTH

/* ── Opcodes ───────────────────────────────────────────────── */

enum {
	LR1110_BL_GET_STATUS_OC            = 0x0100,
	LR1110_BL_GET_VERSION_OC           = 0x0101,
	/* System commands the bootloader also answers (same space as
	 * GetStatus/GetVersion above) — used to bring up the TCXO before
	 * flashing on boards whose 32 MHz reference is powered from DIO3. */
	LR1110_BL_CALIBRATE_OC             = 0x010F,
	LR1110_BL_SET_TCXO_MODE_OC         = 0x0117,
	LR1110_BL_ERASE_FLASH_OC           = 0x8000,
	LR1110_BL_WRITE_FLASH_ENCRYPTED_OC = 0x8003,
	LR1110_BL_GET_HASH_OC              = 0x8004,
	LR1110_BL_REBOOT_OC                = 0x8005,
	LR1110_BL_GET_PIN_OC               = 0x800B,
	LR1110_BL_READ_CHIP_EUI_OC         = 0x800C,
	LR1110_BL_READ_JOIN_EUI_OC         = 0x800D,
};

/* ── Helper ────────────────────────────────────────────────── */

static uint8_t get_min_block_size(uint32_t operand)
{
	return (operand > LR1110_FLASH_DATA_MAX_LENGTH_UINT32)
		? LR1110_FLASH_DATA_MAX_LENGTH_UINT32
		: (uint8_t)operand;
}

/* ── Implementation ────────────────────────────────────────── */

lr1110_status_t lr1110_bootloader_get_status(const void *context,
	lr1110_bootloader_stat1_t *stat1,
	lr1110_bootloader_stat2_t *stat2,
	lr1110_bootloader_irq_mask_t *irq_status)
{
	uint8_t data[LR1110_BL_GET_STATUS_CMD_LENGTH];

	const lr1110_status_t status = (lr1110_status_t)
		lr11xx_hal_direct_read(context, data, LR1110_BL_GET_STATUS_CMD_LENGTH);

	if (status == LR1110_STATUS_OK) {
		stat1->is_interrupt_active = ((data[0] & 0x01) != 0);
		stat1->command_status = (lr1110_bootloader_command_status_t)(data[0] >> 1);

		stat2->is_running_from_flash = ((data[1] & 0x01) != 0);
		stat2->chip_mode = (lr1110_bootloader_chip_modes_t)((data[1] & 0x0F) >> 1);
		stat2->reset_status = (lr1110_bootloader_reset_status_t)((data[1] & 0xF0) >> 4);

		*irq_status = ((lr1110_bootloader_irq_mask_t)data[2] << 24) +
			      ((lr1110_bootloader_irq_mask_t)data[3] << 16) +
			      ((lr1110_bootloader_irq_mask_t)data[4] << 8) +
			      ((lr1110_bootloader_irq_mask_t)data[5] << 0);
	}

	return status;
}

lr1110_status_t lr1110_bootloader_get_version(const void *context,
	lr1110_bootloader_version_t *version)
{
	const uint8_t cbuffer[LR1110_BL_VERSION_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_GET_VERSION_OC >> 8),
		(uint8_t)(LR1110_BL_GET_VERSION_OC >> 0),
	};
	uint8_t rbuffer[LR1110_BL_VERSION_LENGTH] = { 0 };

	const lr1110_status_t status = (lr1110_status_t)
		lr11xx_hal_read(context, cbuffer, LR1110_BL_VERSION_CMD_LENGTH,
				rbuffer, LR1110_BL_VERSION_LENGTH);

	if (status == LR1110_STATUS_OK) {
		version->hw   = rbuffer[0];
		version->type = rbuffer[1];
		version->fw   = ((uint16_t)rbuffer[2] << 8) + (uint16_t)rbuffer[3];
	}

	return status;
}

lr1110_status_t lr1110_bootloader_erase_flash(const void *context)
{
	const uint8_t cbuffer[LR1110_BL_ERASE_FLASH_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_ERASE_FLASH_OC >> 8),
		(uint8_t)(LR1110_BL_ERASE_FLASH_OC >> 0),
	};

	return (lr1110_status_t)
		lr11xx_hal_write(context, cbuffer, LR1110_BL_ERASE_FLASH_CMD_LENGTH, 0, 0);
}

lr1110_status_t lr1110_bootloader_get_hash(const void *context,
	lr1110_bootloader_hash_t hash)
{
	const uint8_t cbuffer[LR1110_BL_GET_HASH_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_GET_HASH_OC >> 8),
		(uint8_t)(LR1110_BL_GET_HASH_OC >> 0),
	};

	return (lr1110_status_t)
		lr11xx_hal_read(context, cbuffer, LR1110_BL_GET_HASH_CMD_LENGTH,
				hash, LR1110_BL_HASH_LENGTH);
}

lr1110_status_t lr1110_bootloader_set_tcxo_mode(const void *context,
	uint8_t tune, uint32_t timeout)
{
	const uint8_t cbuffer[LR1110_BL_SET_TCXO_MODE_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_SET_TCXO_MODE_OC >> 8),
		(uint8_t)(LR1110_BL_SET_TCXO_MODE_OC >> 0),
		tune,
		(uint8_t)(timeout >> 16),
		(uint8_t)(timeout >> 8),
		(uint8_t)(timeout >> 0),
	};

	return (lr1110_status_t)
		lr11xx_hal_write(context, cbuffer, LR1110_BL_SET_TCXO_MODE_CMD_LENGTH, 0, 0);
}

lr1110_status_t lr1110_bootloader_calibrate(const void *context, uint8_t calib_param)
{
	const uint8_t cbuffer[LR1110_BL_CALIBRATE_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_CALIBRATE_OC >> 8),
		(uint8_t)(LR1110_BL_CALIBRATE_OC >> 0),
		calib_param,
	};

	return (lr1110_status_t)
		lr11xx_hal_write(context, cbuffer, LR1110_BL_CALIBRATE_CMD_LENGTH, 0, 0);
}

lr1110_status_t lr1110_bootloader_write_flash_encrypted(const void *context,
	uint32_t offset, const uint32_t *data, uint8_t length)
{
	const uint8_t cbuffer[LR1110_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_WRITE_FLASH_ENCRYPTED_OC >> 8),
		(uint8_t)(LR1110_BL_WRITE_FLASH_ENCRYPTED_OC >> 0),
		(uint8_t)(offset >> 24),
		(uint8_t)(offset >> 16),
		(uint8_t)(offset >> 8),
		(uint8_t)(offset >> 0),
	};

	/* Convert uint32_t words to big-endian byte array for SPI */
	uint8_t cdata[256] = { 0 };
	for (uint8_t i = 0; i < length; i++) {
		uint8_t *p = &cdata[i * sizeof(uint32_t)];
		p[0] = (uint8_t)(data[i] >> 24);
		p[1] = (uint8_t)(data[i] >> 16);
		p[2] = (uint8_t)(data[i] >> 8);
		p[3] = (uint8_t)(data[i] >> 0);
	}

	return (lr1110_status_t)
		lr11xx_hal_write(context, cbuffer, LR1110_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH,
				 cdata, length * sizeof(uint32_t));
}

lr1110_status_t lr1110_bootloader_write_flash_encrypted_full(const void *context,
	uint32_t offset, const uint32_t *buffer, uint32_t length)
{
	uint32_t remaining = length;
	uint32_t local_offset = offset;
	uint32_t loop = 0;

	while (remaining != 0) {
		const lr1110_status_t status = lr1110_bootloader_write_flash_encrypted(
			context, local_offset,
			buffer + loop * LR1110_FLASH_DATA_MAX_LENGTH_UINT32,
			get_min_block_size(remaining));

		if (status != LR1110_STATUS_OK) {
			return status;
		}

		local_offset += LR1110_FLASH_DATA_MAX_LENGTH_UINT8;
		remaining = (remaining < LR1110_FLASH_DATA_MAX_LENGTH_UINT32)
			? 0
			: (remaining - LR1110_FLASH_DATA_MAX_LENGTH_UINT32);

		loop++;
	}

	return LR1110_STATUS_OK;
}

lr1110_status_t lr1110_bootloader_reboot(const void *context,
	bool stay_in_bootloader)
{
	const uint8_t cbuffer[LR1110_BL_REBOOT_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_REBOOT_OC >> 8),
		(uint8_t)(LR1110_BL_REBOOT_OC >> 0),
		stay_in_bootloader ? 0x03 : 0x00,
	};

	return (lr1110_status_t)
		lr11xx_hal_write(context, cbuffer, LR1110_BL_REBOOT_CMD_LENGTH, 0, 0);
}

lr1110_status_t lr1110_bootloader_read_pin(const void *context,
	lr1110_bootloader_pin_t pin)
{
	const uint8_t cbuffer[LR1110_BL_GET_PIN_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_GET_PIN_OC >> 8),
		(uint8_t)(LR1110_BL_GET_PIN_OC >> 0),
	};

	return (lr1110_status_t)
		lr11xx_hal_read(context, cbuffer, LR1110_BL_GET_PIN_CMD_LENGTH,
				pin, LR1110_BL_PIN_LENGTH);
}

lr1110_status_t lr1110_bootloader_read_chip_eui(const void *context,
	lr1110_bootloader_chip_eui_t chip_eui)
{
	const uint8_t cbuffer[LR1110_BL_READ_CHIP_EUI_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_READ_CHIP_EUI_OC >> 8),
		(uint8_t)(LR1110_BL_READ_CHIP_EUI_OC >> 0),
	};

	return (lr1110_status_t)
		lr11xx_hal_read(context, cbuffer, LR1110_BL_READ_CHIP_EUI_CMD_LENGTH,
				chip_eui, LR1110_BL_CHIP_EUI_LENGTH);
}

lr1110_status_t lr1110_bootloader_read_join_eui(const void *context,
	lr1110_bootloader_join_eui_t join_eui)
{
	const uint8_t cbuffer[LR1110_BL_READ_JOIN_EUI_CMD_LENGTH] = {
		(uint8_t)(LR1110_BL_READ_JOIN_EUI_OC >> 8),
		(uint8_t)(LR1110_BL_READ_JOIN_EUI_OC >> 0),
	};

	return (lr1110_status_t)
		lr11xx_hal_read(context, cbuffer, LR1110_BL_READ_JOIN_EUI_CMD_LENGTH,
				join_eui, LR1110_BL_JOIN_EUI_LENGTH);
}
