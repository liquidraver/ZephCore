/*
 * SPDX-License-Identifier: MIT
 *
 * Driver for the LR1110 "bootloader updater" (loader) firmware — the
 * commands below are understood by the lr1110_loader_6500 image while it
 * is RUNNING on the chip (i.e. after it has been flashed and rebooted
 * into). They are NOT bootloader-mode commands.
 *
 * Background (AN1200.57, Semtech SWTL001): transceiver firmware 0x0402
 * only boots on chip bootloader 0x1001. Chips with the legacy bootloader
 * 0x6500 must first flash + run this loader, tell it to rewrite the
 * bootloader (0x8100), verify (0x8101), then reboot (0x8102) — after
 * which the chip reports bootloader 0x1001 and accepts 0x0402.
 *
 * Command set ported from SWTL001 bootloader_updater_driver (Clear BSD,
 * Semtech Corporation 2026). GetVersion/GetStatus share the classic
 * bootloader wire format — reuse lr1110_bootloader_get_version/_get_status.
 */

#ifndef LR1110_BL_UPDATER_H
#define LR1110_BL_UPDATER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "lr1110_bootloader.h"

/* On-chip bootloader versions (LR1110) */
#define LR1110_BL_VERSION_LEGACY  0x6500  /* shipped since 2020; loader targets this */
#define LR1110_BL_VERSION_UPDATED 0x1001  /* after the loader ran; required by FW 0x0402 */

/* Result of the loader's VerifyBootloader (0x8101) command */
typedef struct {
	bool signature_verified;
	bool version_verified;
	bool use_case_verified;
	bool version_major_verified;
	bool version_minor_verified;
	bool anti_rollback_verified;
	uint8_t use_case;
	uint8_t version_major;
	uint8_t version_minor;
} lr1110_bl_updater_report_t;

/* Embedded lr1110_loader_6500 image (lib/lr1110_loader_6500.h) */
uint16_t        lr1110_bl_loader_version(void);     /* 0x6500 */
const uint32_t *lr1110_bl_loader_image(void);
uint32_t        lr1110_bl_loader_image_size(void);  /* in 32-bit words */

/* Ask the running loader to rewrite the chip bootloader (opcode 0x8100).
 * BUSY stays high for the duration of the rewrite — the HAL grants this
 * opcode the long timeout. */
lr1110_status_t lr1110_bl_updater_update_bootloader(const void *context);

/* Read the loader's verification report for the new bootloader (0x8101). */
lr1110_status_t lr1110_bl_updater_verify_bootloader(const void *context,
	lr1110_bl_updater_report_t *report);

/* All six verification checks passed? */
bool lr1110_bl_updater_report_ok(const lr1110_bl_updater_report_t *report);

/* Reboot out of the loader (opcode 0x8102, NOT the bootloader's 0x8005). */
lr1110_status_t lr1110_bl_updater_reboot(const void *context,
	bool stay_in_bootloader);

#ifdef __cplusplus
}
#endif

#endif /* LR1110_BL_UPDATER_H */
