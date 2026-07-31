/*
 * SPDX-License-Identifier: MIT
 *
 * LR1110 bootloader-updater (loader) driver — see lr1110_bl_updater.h.
 * Command frames ported from Semtech SWTL001 bootloader_updater_driver
 * (Clear BSD, Semtech Corporation 2026).
 */

#include "lr1110_bl_updater.h"
#include "lr11xx_hal.h"

/*
 * Embed the loader image. Both Semtech firmware headers define the same
 * array name (lr11xx_firmware_image) AND the same include guard
 * (LR11XX_FW_H), so the loader header may only be included in THIS
 * translation unit (main.c holds the transceiver image), with the array
 * renamed to avoid the duplicate symbol.
 */
#define lr11xx_firmware_image lr11xx_firmware_loader_image
#include "lr1110_loader_6500.h"
#undef lr11xx_firmware_image

/* Loader ("bootloader updater firmware") command opcodes */
#define LR1110_BL_UPDATER_UPDATE_BOOTLOADER_OC 0x8100
#define LR1110_BL_UPDATER_VERIFY_BOOTLOADER_OC 0x8101
#define LR1110_BL_UPDATER_REBOOT_OC            0x8102

uint16_t lr1110_bl_loader_version(void)
{
	return LR11XX_FIRMWARE_VERSION; /* 0x6500 — from lr1110_loader_6500.h */
}

const uint32_t *lr1110_bl_loader_image(void)
{
	return lr11xx_firmware_loader_image;
}

uint32_t lr1110_bl_loader_image_size(void)
{
	return LR11XX_FIRMWARE_IMAGE_SIZE; /* 4884 words */
}

lr1110_status_t lr1110_bl_updater_update_bootloader(const void *context)
{
	const uint8_t cbuffer[2] = {
		(uint8_t)(LR1110_BL_UPDATER_UPDATE_BOOTLOADER_OC >> 8),
		(uint8_t)(LR1110_BL_UPDATER_UPDATE_BOOTLOADER_OC >> 0),
	};

	return (lr1110_status_t)lr11xx_hal_write(context, cbuffer, sizeof(cbuffer), 0, 0);
}

lr1110_status_t lr1110_bl_updater_verify_bootloader(const void *context,
	lr1110_bl_updater_report_t *report)
{
	const uint8_t cbuffer[2] = {
		(uint8_t)(LR1110_BL_UPDATER_VERIFY_BOOTLOADER_OC >> 8),
		(uint8_t)(LR1110_BL_UPDATER_VERIFY_BOOTLOADER_OC >> 0),
	};
	uint8_t rbuffer[4] = { 0 };

	const lr11xx_hal_status_t status =
		lr11xx_hal_read(context, cbuffer, sizeof(cbuffer), rbuffer, sizeof(rbuffer));

	if (status == LR11XX_HAL_STATUS_OK) {
		const uint8_t checks = rbuffer[0];

		report->signature_verified     = (checks & 0x01) != 0;
		report->version_verified       = (checks & 0x02) != 0;
		report->use_case_verified      = (checks & 0x04) != 0;
		report->version_major_verified = (checks & 0x08) != 0;
		report->version_minor_verified = (checks & 0x10) != 0;
		report->anti_rollback_verified = (checks & 0x20) != 0;
		report->use_case               = rbuffer[1];
		report->version_major          = rbuffer[2];
		report->version_minor          = rbuffer[3];
	}

	return (lr1110_status_t)status;
}

bool lr1110_bl_updater_report_ok(const lr1110_bl_updater_report_t *report)
{
	return report->signature_verified &&
	       report->version_verified &&
	       report->use_case_verified &&
	       report->version_major_verified &&
	       report->version_minor_verified &&
	       report->anti_rollback_verified;
}

lr1110_status_t lr1110_bl_updater_reboot(const void *context, bool stay_in_bootloader)
{
	const uint8_t cbuffer[3] = {
		(uint8_t)(LR1110_BL_UPDATER_REBOOT_OC >> 8),
		(uint8_t)(LR1110_BL_UPDATER_REBOOT_OC >> 0),
		stay_in_bootloader ? 0x03 : 0x00,
	};

	return (lr1110_status_t)lr11xx_hal_write(context, cbuffer, sizeof(cbuffer), 0, 0);
}
