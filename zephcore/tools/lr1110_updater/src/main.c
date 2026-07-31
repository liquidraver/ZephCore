/*
 * ZephCore LR1110 Firmware Updater
 * SPDX-License-Identifier: MIT
 *
 * Standalone tool that updates the LR1110 radio firmware to v0x0402
 * (Semtech H1_2026 security release: CVE-2025-14857/14858/14859).
 * Flash this UF2 first, let it update the radio, then flash ZephCore main firmware.
 *
 * Firmware image source: https://github.com/Lora-net/radio_firmware_images
 * Bootloader protocol source: https://github.com/Lora-net/lr1110_driver
 * Loader flow reference: https://github.com/Lora-net/SWTL001 (AN1200.57)
 * License: Clear BSD (Semtech Corporation 2021-2026)
 *
 * Firmware 0x0402 only boots on chip bootloader 0x1001. Chips made before
 * the H1_2026 release run legacy bootloader 0x6500, so the update is
 * TWO-STAGE (matching Semtech's SWTL001 reference):
 *
 *   Stage A — bootloader update (only when the chip reports BL 0x6500):
 *     1. Force bootloader mode (reset with BUSY held LOW)
 *     2. Erase flash, WriteFlashEncrypted the lr1110_loader_6500 image
 *     3. Reboot — the chip now RUNS the loader firmware
 *     4. Loader command 0x8100: rewrite the on-chip bootloader
 *     5. Loader command 0x8101: verification report (6 checks)
 *     6. Loader command 0x8102: reboot; chip lands back in bootloader
 *     7. Verify GetVersion now reports BL 0x1001
 *
 *   Stage B — transceiver firmware:
 *     8. Erase flash, WriteFlashEncrypted the 0x0402 image (61320 words)
 *     9. Reboot into firmware, verify version == 0x0402
 *    10. Reboot MCU into UF2 DFU mode (nRF52) / idle for reflash (ESP32)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#include "lr11xx_hal_updater.h"
#include "lr1110_bootloader.h"
#include "lr1110_bl_updater.h"
/*
 * Which transceiver image to flash. 0x0402 is the current release (and the
 * only one Semtech's own tool will attempt on bootloader 0x1001).
 *
 * 0x0401 is selectable to TEST whether the downgrade block is enforced by
 * the CHIP or merely by the host tool: the compatibility table that pairs
 * 0x0401 with bootloader 0x6500 lives in Semtech's reference *application*
 * (lr11xx_update_utils.c), not in the silicon. Since 0x0402 is a CVE fix,
 * anti-rollback in the new bootloader is plausible — but unverified, and a
 * chip that runs 0x0401 is fully usable by ZephCore (T1000-E ships it).
 *
 *   west build ... -- -DUPDATER_TARGET_FW=0x0401
 */
#ifndef UPDATER_TARGET_FW
#define UPDATER_TARGET_FW 0x0402
#endif

/*
 * Perform the 0x6500 -> 0x1001 chip-bootloader update when firmware 0x0402 is
 * the target (0x0402 will not run on the old bootloader).
 *
 * Verified end-to-end on a T1000-E 2026-07-21: Stage A then Stage B, chip
 * afterwards reports TYPE=0x01 FW=0x0402 and runs from flash. An earlier
 * belief that this update bricks radios came from a sample where every
 * observed 0x1001 chip was on one board (ThinkNode M9) — that board has a
 * separate, still-open problem, so a board fault was being read as a
 * bootloader fault.
 *
 * It IS one-way: Semtech ships loaders in the forward direction only, and the
 * new bootloader lives inside the encrypted loader payload, so there is
 * nothing to flash back. Set this to 0 to flash firmware only and leave the
 * chip bootloader untouched.
 */
#ifndef UPDATER_ALLOW_BOOTLOADER_UPDATE
#define UPDATER_ALLOW_BOOTLOADER_UPDATE 1
#endif

#if UPDATER_TARGET_FW == 0x0401
#include "lr1110_transceiver_0401.h"
#elif UPDATER_TARGET_FW == 0x0303
/* 0x0303 is the version another M9/LR1110 in this exact broken state was
 * recovered with (reported running as "Base FW 3.3" under RadioLib), which
 * also demonstrates the bootloader does NOT enforce anti-rollback. */
#include "lr1110_transceiver_0303.h"
#else
#include "lr1110_transceiver_0402.h"
#endif

#if defined(CONFIG_SOC_SERIES_NRF52)
#include <hal/nrf_power.h>
#endif

/* Adafruit UF2 bootloader magic — enter mass storage DFU mode */
#define BOOTLOADER_DFU_UF2_MAGIC 0x57

/* Target firmware version */
#define TARGET_FW_VERSION LR11XX_FIRMWARE_VERSION  /* 0x0402 */

/* LR1110 type field values */
#define LR1110_TYPE_TRANSCEIVER  0x01
#define LR1110_TYPE_BOOTLOADER   0xDF
/* Reported while the lr1110_loader_6500 image is RUNNING. Not in Semtech's
 * headers — SWTL001 never checks TYPE after booting the loader, it relies on
 * fw==0x6500, which the ROM bootloader ALSO reports when the loader image
 * failed to boot and the chip fell back. Field-observed (M9, 2026-07-19):
 * a chip verifiably running the loader answers TYPE=0xDE, FW=0x6500. TYPE
 * is therefore the only reliable loader-vs-fallback discriminator. */
#define LR1110_TYPE_BL_UPDATER   0xDE

/* ── LED feedback (optional) ─────────────────────────────────── */

#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static void led_init(void)  { gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); }
static void led_on(void)    { gpio_pin_set_dt(&led, 1); }
static void led_off(void)   { gpio_pin_set_dt(&led, 0); }
static void led_toggle(void) { gpio_pin_toggle_dt(&led); }
#else
#define HAS_LED 0
static void led_init(void)  {}
static void led_on(void)    {}
static void led_off(void)   {}
static void led_toggle(void) {}
#endif

/* Idle gap inserted after every flash page program.
 *
 * Diagnostic for a marginal supply. A page program pulls current for ~3.6 ms
 * and we issue 959 of them back to back, so a rail that cannot sustain that
 * burst will sag — and a sagging rail corrupts what gets programmed while the
 * write still reports OK (the bootloader never reads back). Spacing the pages
 * lets the supply recover between them. 10 ms stretches a full image write
 * from ~6.6 s to ~16 s, which costs nothing.
 *   west build ... -- -DUPDATER_CHUNK_DELAY_MS=10
 */
#ifndef UPDATER_CHUNK_DELAY_MS
#define UPDATER_CHUNK_DELAY_MS 0
#endif

/* Refuse to flash with an SD card in the slot.
 *
 * On the M9 the slot shares SPI2 with the radio, and a card present during
 * flashing produces a silently corrupt image: every write reports OK, the
 * chip spends real time programming, and the result fails its integrity check
 * at boot with no error anywhere to point at. Field-confirmed 2026-07-21 —
 * removing the card was what finally made that board flash.
 *
 * Gated rather than warned because the asymmetry is stark: the cost of the
 * gate is ejecting a card, the cost of missing it is days of debugging.
 *   west build ... -- -DUPDATER_IGNORE_SDCARD=1
 */
#ifndef UPDATER_IGNORE_SDCARD
#define UPDATER_IGNORE_SDCARD 0
#endif

/* ── Helpers ──────────────────────────────────────────────────── */

static void updater_done(void)
{
#if defined(CONFIG_SOC_SERIES_NRF52)
	printk("\nRebooting into UF2 DFU mode...\n");
	printk("You can now drag-drop ZephCore firmware UF2.\n");
	k_msleep(500);

	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_UF2_MAGIC);
	sys_reboot(SYS_REBOOT_COLD);
#else
	/* ESP32: no persistent DFU mode to reboot into, and a plain reboot
	 * would just re-run the updater. Idle here — west flash / esptool
	 * resets the chip itself when writing the main firmware. */
	printk("\nFlash ZephCore main firmware over USB now:\n");
	printk("  west flash --esp-device <COMx>\n");
	while (1) {
		led_toggle();
		k_msleep(1000);
	}
#endif
}

static void fatal_error(const char *msg)
{
	printk("\n!!! FATAL: %s\n", msg);
	printk("Please power cycle the device and try again.\n");
	led_off();

	/* Blink LED rapidly to indicate error */
	while (1) {
		led_toggle();
		k_msleep(200);
	}
}

static void check_sdcard(void)
{
	int rc = lr1110_updater_probe_sdcard();

	if (rc == -ENOTSUP) {
		return; /* board has no SD slot on the radio's bus */
	}
	if (rc < 0) {
		printk("  SD slot: probe failed (%d) — continuing\n", rc);
		return;
	}
	if (rc == 0) {
		printk("  SD slot: empty\n");
		return;
	}

	printk("  SD slot: CARD PRESENT\n");
	if (UPDATER_IGNORE_SDCARD) {
		printk("  WARNING: flashing anyway (UPDATER_IGNORE_SDCARD=1).\n");
		return;
	}
	printk("\n");
	printk("  The SD slot shares SPI2 with the radio. Flashing with a\n");
	printk("  card inserted corrupts the image while every write still\n");
	printk("  reports success — the chip ends up running nothing and\n");
	printk("  there is no error to point at.\n");
	printk("\n");
	printk("  Remove the SD card and try again.\n");
	fatal_error("SD card present — refusing to flash");
}

/* Map a millivolt value from devicetree to the chip's TCXO supply code. */
static uint8_t tcxo_code_from_mv(uint16_t mv)
{
	if (mv >= 3300) return LR1110_TCXO_CTRL_3_3V;
	if (mv >= 3000) return LR1110_TCXO_CTRL_3_0V;
	if (mv >= 2700) return LR1110_TCXO_CTRL_2_7V;
	if (mv >= 2400) return LR1110_TCXO_CTRL_2_4V;
	if (mv >= 2200) return LR1110_TCXO_CTRL_2_2V;
	if (mv >= 1800) return LR1110_TCXO_CTRL_1_8V;
	if (mv >= 1700) return LR1110_TCXO_CTRL_1_7V;
	return LR1110_TCXO_CTRL_1_6V;
}

/* Power the TCXO and switch the chip onto it before touching flash.
 *
 * On a crystal board the chip can start its 32 MHz XOSC by itself. On a
 * TCXO board the oscillator is powered from DIO3, which stays OFF until
 * SetTcxoMode is issued — so the chip would otherwise run the ENTIRE
 * update on its internal RC oscillator, which is both slower and far less
 * accurate. Flash program/erase pulse timing and the charge-pump sequencing
 * derive from that clock, so a marginal clock can produce writes that
 * report OK but do not survive the image integrity check at boot.
 *
 * Not in Semtech's update documentation (their reference hardware does not
 * need it), but field-reported to make LR1110 flashing succeed on boards
 * where it otherwise fails. Best-effort: a chip that rejects the command
 * simply stays on RC, exactly as before. */
/*
 * Off by default. Neither Semtech's SWTL001 nor RadioLib touches the TCXO
 * while flashing, and SetTcxoMode (0x0117) / Calibrate (0x010F) are *system*
 * opcodes that the bootloader command set does not include — so issuing them
 * here is unverified behaviour on a bootloader we need to keep in a
 * well-defined state. The field report that TCXO helps was not reproducible
 * on this board (the failure predates and survives it). Set to 1 to re-test.
 */
#define UPDATER_ENABLE_TCXO 0

static void configure_tcxo(void *ctx)
{
	const uint16_t mv = lr1110_updater_tcxo_voltage_mv();

	if (mv == 0) {
		printk("  No TCXO in devicetree — chip uses its own crystal\n");
		return;
	}

	const uint8_t code = tcxo_code_from_mv(mv);
	/* SetTcxoMode timeout is in 32.768 kHz ticks (1 tick = 30.52 us).
	 * Use a generous window — at least 10 ms — so a slow-starting TCXO
	 * is never declared failed. */
	uint32_t delay_ms = lr1110_updater_tcxo_startup_delay_ms();

	if (delay_ms < 10) {
		delay_ms = 10;
	}
	const uint32_t ticks = delay_ms * 32768U / 1000U;

	printk("  Powering TCXO: %u mV (code 0x%02X), startup %u ms\n",
	       mv, code, delay_ms);

	if (lr1110_bootloader_set_tcxo_mode(ctx, code, ticks) != LR1110_STATUS_OK) {
		printk("  WARNING: SetTcxoMode rejected — continuing on RC clock\n");
		return;
	}
	k_msleep(delay_ms + 10);

	/* Changing the clock source invalidates the factory calibration —
	 * recalibrate every block before using the chip. */
	if (lr1110_bootloader_calibrate(ctx, LR1110_CALIB_ALL) != LR1110_STATUS_OK) {
		printk("  WARNING: Calibrate rejected after TCXO enable\n");
		return;
	}
	k_msleep(50);

	printk("  TCXO active, chip recalibrated\n");
}

/* Dump the chip's flash digest at a labelled point in the sequence.
 *
 * Sampled before erase, after erase and after write, these three values
 * answer the question no other command can: does anything we do actually
 * change the flash? If all three match, the writes are not landing at all
 * (or GetHash is inert on this bootloader). If erase changes it but the
 * write does not, the payload is being discarded. */
static void dump_flash_hash(void *ctx, const char *when)
{
	lr1110_bootloader_hash_t hash = { 0 };

	if (lr1110_bootloader_get_hash(ctx, hash) != LR1110_STATUS_OK) {
		printk("  Flash hash (%s): READ FAILED\n", when);
		return;
	}

	printk("  Flash hash (%-12s): ", when);
	for (int i = 0; i < LR1110_BL_HASH_LENGTH; i++) {
		printk("%02X", hash[i]);
	}
	printk("\n");
}

/* Ask the chip how the last command went.
 *
 * WriteFlashEncrypted is fire-and-forget — the HAL only knows the SPI
 * transfer happened, not that the chip accepted it. Sampling the command
 * status during the transfer turns a silent corruption into a located one.
 * Returns false only for a definite chip-reported failure. */
static const char *cmd_status_name(uint8_t s)
{
	switch (s) {
	case LR1110_BOOTLOADER_CMD_STATUS_FAIL: return "FAIL";
	case LR1110_BOOTLOADER_CMD_STATUS_PERR: return "PERR";
	case LR1110_BOOTLOADER_CMD_STATUS_OK:   return "OK";
	case LR1110_BOOTLOADER_CMD_STATUS_DATA: return "DATA";
	default:                                return "?";
	}
}

static bool command_status_ok(void *ctx, uint32_t chunk_idx, uint8_t *status_out)
{
	lr1110_bootloader_stat1_t stat1 = { 0 };
	lr1110_bootloader_stat2_t stat2 = { 0 };
	lr1110_bootloader_irq_mask_t irq = 0;

	if (status_out) {
		*status_out = 0xFF; /* unknown until GetStatus succeeds */
	}

	if (lr1110_bootloader_get_status(ctx, &stat1, &stat2, &irq) != LR1110_STATUS_OK) {
		printk("  WARNING: GetStatus failed near chunk %u\n", chunk_idx);
		return true; /* inconclusive — do not abort the flash */
	}

	if (status_out) {
		*status_out = (uint8_t)stat1.command_status;
	}

	if (stat1.command_status == LR1110_BOOTLOADER_CMD_STATUS_FAIL ||
	    stat1.command_status == LR1110_BOOTLOADER_CMD_STATUS_PERR) {
		printk("  ERROR: chip rejected a write near chunk %u (cmd_status=%u)\n",
		       chunk_idx, (unsigned)stat1.command_status);
		return false;
	}

	return true;
}

/* Erase the chip flash, then write `total` words in 64-word chunks with
 * progress output. Used for both the loader and the transceiver image. */
static void erase_and_flash_image(void *ctx, const char *what,
				  const uint32_t *image, uint32_t total)
{
	lr1110_status_t rc;

	/* Bring the TCXO up here rather than once at startup: every reset —
	 * including the forced bootloader entries in Stage A — drops the chip
	 * back onto its RC oscillator, so the clock must be re-established
	 * immediately before each erase/write pass. */
	if (UPDATER_ENABLE_TCXO) {
		configure_tcxo(ctx);
	}

	dump_flash_hash(ctx, "before erase");

	printk("  Erasing LR1110 flash (~2.5 seconds)...\n");
	led_toggle();
	const int64_t erase_start = k_uptime_get();
	rc = lr1110_bootloader_erase_flash(ctx);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("Flash erase failed");
	}
	/* A full-flash erase genuinely takes ~2.5 s. If this reports a few
	 * milliseconds then BUSY was never observed and we are about to write
	 * into flash that is still erasing — which would corrupt everything
	 * while every write still reports OK. */
	printk("  Erase returned after %lld ms\n", k_uptime_get() - erase_start);

	/* Let the flash controller settle before the first program cycle —
	 * the erase leaves the charge pump loaded and neither reference tool
	 * writes this promptly after a 2.5 s full-array erase. */
	k_msleep(100);
	dump_flash_hash(ctx, "after erase");

	printk("  Writing %s (%u words = %u KB)...\n",
	       what, total, (total * 4) / 1024);

	const int64_t write_start = k_uptime_get();

	/* MUST be 64 words (256 bytes = one flash page). A 14-word write is
	 * rejected outright with PERR at chunk 0 — the chip validates the byte
	 * count and only accepts whole pages (a short final chunk is fine).
	 *
	 * NOTE: that PERR result does NOT clear the SPI path, though it was
	 * once read that way. A 14-word write is a 62-byte frame — one hardware
	 * transaction on the ESP32-S3 — so it says nothing about whether a
	 * 262-byte frame survives being split across five. Matches Semtech's
	 * LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 = 64 regardless. */
	const uint32_t chunk_size = 64;
	uint32_t num_chunks = (total + chunk_size - 1) / chunk_size;
	uint32_t progress_step = num_chunks / 10; /* Print every 10% */
	if (progress_step == 0) progress_step = 1;

	uint32_t remaining = total;
	uint32_t offset = 0;
	uint32_t chunk_idx = 0;

	while (remaining > 0) {
		uint8_t this_chunk = (remaining > chunk_size)
			? (uint8_t)chunk_size : (uint8_t)remaining;

		rc = lr1110_bootloader_write_flash_encrypted(
			ctx, offset,
			&image[chunk_idx * chunk_size],
			this_chunk);

		if (rc != LR1110_STATUS_OK) {
			printk("\n");
			printk("ERROR: Write failed at offset 0x%08X (chunk %u/%u)\n",
			       offset, chunk_idx + 1, num_chunks);
			fatal_error("Firmware write failed");
		}

		/* Check EVERY chunk, not just the progress points. Sampling at
		 * 10% intervals leaves ~95 unexamined writes between samples,
		 * and cmd_status only reflects the most recent command — so a
		 * rejection in between is invisible. This is the diagnostic
		 * that pins down the exact chunk (and flash offset) where the
		 * chip first refuses data, if it refuses at all. */
		/* Capture BUSY timing for THIS write before GetStatus below
		 * overwrites it with its own command's timing. */
		const uint32_t busy_rise_us = lr1110_updater_last_busy_rise_us();
		const uint32_t busy_hold_us = lr1110_updater_last_busy_hold_us();
		const bool     busy_seen    = lr1110_updater_last_busy_seen();
		const int      spi_ret      = lr1110_updater_last_spi_ret();

		uint8_t cmd_status = 0xFF;
		const bool accepted = command_status_ok(ctx, chunk_idx, &cmd_status);

		/* Full per-chunk trace. The two columns that matter most are
		 * busy=... (a chunk that never asserted BUSY did no flash
		 * programming, however cleanly it "succeeded") and hold=...
		 * (a page program is ~3.8 ms; a near-zero hold is a write that
		 * did not happen). */
		printk("  [%4u/%4u] off=0x%06X len=%2u spi=%d busy=%s rise=%uus hold=%uus stat=%s\n",
		       chunk_idx, num_chunks, offset, this_chunk, spi_ret,
		       busy_seen ? "yes" : "NO ", busy_rise_us, busy_hold_us,
		       cmd_status == 0xFF ? "??" : cmd_status_name(cmd_status));

		if (!accepted) {
			printk("  First rejection at chunk %u of %u, flash offset 0x%08X\n",
			       chunk_idx, num_chunks, offset);
			fatal_error("Chip rejected firmware data mid-flash");
		}

		if ((chunk_idx % progress_step) == 0) {
			uint32_t pct = (chunk_idx * 100) / num_chunks;
			printk("  ---- %3u%%  (%u / %u words) ----\n",
			       pct, total - remaining + this_chunk, total);
			led_toggle();
		}

		if (UPDATER_CHUNK_DELAY_MS > 0) {
			k_msleep(UPDATER_CHUNK_DELAY_MS);
		}

		offset += this_chunk * sizeof(uint32_t);
		remaining -= this_chunk;
		chunk_idx++;
	}

	printk("  100%%  (%u / %u words)\n", total, total);
	printk("  %s write complete in %lld ms\n", what, k_uptime_get() - write_start);

	dump_flash_hash(ctx, "after write");
}

/* The loader is RUNNING (TYPE 0xDE) — command the bootloader rewrite,
 * verify it, then reboot and confirm the chip reports BL 0x1001. Shared by
 * run_bootloader_update() and the resume path in main() (chip found already
 * running a loader at startup, e.g. a previous attempt that died between
 * flashing the loader and the rewrite). */
static void loader_rewrite_and_verify(void *ctx)
{
	lr1110_bootloader_version_t version = { 0 };
	lr1110_bl_updater_report_t report = { 0 };
	lr1110_status_t rc;

	/* No configure_tcxo() here, deliberately.  The 0x8100 rewrite below is a
	 * flash write and the chip is back on its RC oscillator after the reboot
	 * into the loader, so this looks like a hole in the TCXO workaround that
	 * erase_and_flash_image() applies — it is not:
	 *
	 * - SWTL001 configures no TCXO at ANY point of the update (its
	 *   lr11xx_update_firmware() is just erase + write; the driver's
	 *   set_tcxo_mode is never called by the update application). Our
	 *   configure_tcxo() is a ZephCore-only field workaround with no
	 *   counterpart in the reference flow.
	 * - The loader image's documented command set is only 0x8100/01/02 plus
	 *   GetVersion/GetStatus. It exposes no system commands, so SetTcxoMode
	 *   (0x0117) is undocumented against a RUNNING loader — sending it is a
	 *   guess, not a fix.
	 * - The failure modes are not symmetric. A bad transceiver flash leaves
	 *   the chip falling back to the bootloader, which is retryable. A bad
	 *   bootloader rewrite may not be recoverable at all. That asymmetry is
	 *   what decides it: do not fire an undocumented opcode at the chip in
	 *   the seconds before the one irreversible operation in this tool.
	 *
	 * If M9 bring-up ever shows the rewrite failing on a TCXO board, revisit
	 * with the GetStatus output below as evidence — do not add it blind. */

	/* Ask the loader to rewrite the bootloader — BUSY is held for the
	 * duration; the HAL grants opcode 0x8100 the long timeout. */
	printk("  Rewriting chip bootloader (this takes a few seconds)...\n");
	rc = lr1110_bl_updater_update_bootloader(ctx);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("Bootloader rewrite command failed");
	}

	/* SWTL001 issues a GetStatus here (the loader answers the classic
	 * bootloader wire format) as its "wait for update termination" sync
	 * point. The read also returns the 0x8100 command status — the only
	 * way to tell a failed rewrite (FAIL) from a verify report that was
	 * corrupted on a marginal bus. Diagnostic only: like SWTL001, the
	 * verify below remains the gate. */
	{
		lr1110_bootloader_stat1_t stat1 = { 0 };
		lr1110_bootloader_stat2_t stat2 = { 0 };
		lr1110_bootloader_irq_mask_t irq = 0;

		rc = lr1110_bootloader_get_status(ctx, &stat1, &stat2, &irq);
		if (rc != LR1110_STATUS_OK) {
			fatal_error("Loader GetStatus after rewrite failed");
		}
		printk("  Loader status: cmd=%u (%s) mode=%u flash=%d reset=%u\n",
		       stat1.command_status,
		       stat1.command_status == LR1110_BOOTLOADER_CMD_STATUS_OK   ? "OK" :
		       stat1.command_status == LR1110_BOOTLOADER_CMD_STATUS_DATA ? "DATA" :
		       stat1.command_status == LR1110_BOOTLOADER_CMD_STATUS_PERR ? "PERR" : "FAIL",
		       stat2.chip_mode, stat2.is_running_from_flash, stat2.reset_status);
	}

	/* Verification report — all six checks must pass */
	rc = lr1110_bl_updater_verify_bootloader(ctx, &report);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("VerifyBootloader command failed");
	}
	printk("  Verify: sig=%d ver=%d usecase=%d major=%d minor=%d rollback=%d (v%u.%u usecase %u)\n",
	       report.signature_verified, report.version_verified,
	       report.use_case_verified, report.version_major_verified,
	       report.version_minor_verified, report.anti_rollback_verified,
	       report.version_major, report.version_minor, report.use_case);
	if (!lr1110_bl_updater_report_ok(&report)) {
		fatal_error("New bootloader failed verification");
	}

	/* Reboot out of the loader (loader opcode 0x8102). The new bootloader
	 * refuses to execute the loader image left in flash, so the chip lands
	 * in bootloader mode — force a clean re-entry and confirm 0x1001. */
	printk("  Rebooting with the new bootloader...\n");
	lr1110_bl_updater_reboot(ctx, false);
	k_msleep(500);

	if (lr1110_updater_reset_to_bootloader() != 0) {
		fatal_error("Failed to re-enter bootloader after update");
	}
	rc = lr1110_bootloader_get_version(ctx, &version);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("GetVersion after bootloader update failed");
	}
	printk("  Chip bootloader now: 0x%04X\n", version.fw);
	if (version.fw != LR1110_BL_VERSION_UPDATED) {
		fatal_error("Bootloader version mismatch after update");
	}

	printk("--- Stage A complete: bootloader 0x%04X ---\n\n",
	       LR1110_BL_VERSION_UPDATED);
}

/* Stage A — flash + run the loader so it rewrites the chip bootloader
 * 0x6500 -> 0x1001. Chip must currently be in bootloader mode. On return
 * the chip is back in bootloader mode running BL 0x1001. */
static void run_bootloader_update(void *ctx)
{
	lr1110_bootloader_version_t version = { 0 };
	lr1110_status_t rc;

	printk("\n--- Stage A: chip bootloader update 0x%04X -> 0x%04X ---\n",
	       LR1110_BL_VERSION_LEGACY, LR1110_BL_VERSION_UPDATED);
	printk("(required once per chip before firmware 0x0402 can run)\n\n");

	/* Flash the loader image */
	erase_and_flash_image(ctx, "bootloader-updater (loader)",
			      lr1110_bl_loader_image(),
			      lr1110_bl_loader_image_size());

	/* Boot the loader */
	printk("  Rebooting into the loader...\n");
	lr1110_bootloader_reboot(ctx, false);
	k_msleep(500);

	/* The loader answers the standard GetVersion frame with FW 0x6500 —
	 * but so does the ROM bootloader after a fallback, so TYPE (0xDE
	 * loader vs 0xDF ROM bootloader) is the real started-check. */
	rc = lr1110_bootloader_get_version(ctx, &version);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("GetVersion after loader boot failed");
	}
	printk("  Loader GetVersion: TYPE=0x%02X FW=0x%04X\n", version.type, version.fw);
	if (version.type == LR1110_TYPE_BOOTLOADER) {
		fatal_error("Loader did not start — chip fell back to the ROM "
			    "bootloader, so the loader flash write did not "
			    "stick. Check the radio power rail, then retry");
	}
	if (version.type != LR1110_TYPE_BL_UPDATER ||
	    version.fw != lr1110_bl_loader_version()) {
		fatal_error("Loader did not start (unexpected version)");
	}

	loader_rewrite_and_verify(ctx);
}

/* ── Main ─────────────────────────────────────────────────────── */

int main(void)
{
	lr1110_bootloader_version_t version = { 0 };
	void *ctx;
	lr1110_status_t rc;

	/* Brief delay for USB CDC to enumerate */
	k_msleep(2000);

	printk("\n");
	printk("============================================\n");
	printk("  ZephCore LR1110 Firmware Updater\n");
	printk("  Target: transceiver FW 0x%04X\n", TARGET_FW_VERSION);
	printk("  Image:  %u words (%u KB)\n",
	       LR11XX_FIRMWARE_IMAGE_SIZE,
	       (LR11XX_FIRMWARE_IMAGE_SIZE * 4) / 1024);
	printk("  Build:  SPI=%s  chunk-delay=%ums  bl-update=%s\n",
#ifdef UPDATER_SPI_DMA
	       "DMA",
#else
	       "PIO",
#endif
	       (unsigned)UPDATER_CHUNK_DELAY_MS,
	       UPDATER_ALLOW_BOOTLOADER_UPDATE ? "yes" : "no");
	printk("============================================\n");
	printk("\n");

	led_init();
	led_on();

	/* ── Step 1: Initialize HAL ── */
	printk("[1/8] Initializing SPI and GPIOs...\n");
	if (lr1110_updater_hal_init() != 0) {
		fatal_error("HAL init failed");
	}
	ctx = lr1110_updater_get_context();
	check_sdcard();

	/* ── Step 2: Hardware reset ── */
	printk("[2/8] Hardware reset LR1110...\n");
	if (lr1110_updater_hw_reset() != 0) {
		fatal_error("Hardware reset failed (BUSY stuck)");
	}

	/* ── Step 3: Read current version ── */
	printk("[3/8] Reading current firmware version...\n");
	rc = lr1110_bootloader_get_version(ctx, &version);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("GetVersion failed");
	}

	printk("  HW   = 0x%02X\n", version.hw);
	printk("  TYPE = 0x%02X", version.type);
	if (version.type == LR1110_TYPE_TRANSCEIVER) {
		printk(" (transceiver firmware)\n");
	} else if (version.type == LR1110_TYPE_BOOTLOADER) {
		printk(" (bootloader mode)\n");
	} else if (version.type == LR1110_TYPE_BL_UPDATER) {
		printk(" (bootloader-updater loader)\n");
	} else {
		printk(" (unknown)\n");
	}
	printk("  FW   = 0x%04X\n", version.fw);

	/* Seeing "bootloader mode" here does NOT prove the flash is empty: the
	 * hardware reset above releases NRESET while BUSY reads LOW, which is
	 * itself the bootloader-entry condition on this wiring. So ask the
	 * bootloader to run whatever is in flash and look again — otherwise a
	 * chip that is already up to date gets pointlessly reflashed. */
	if (version.type == LR1110_TYPE_BOOTLOADER) {
		lr1110_bootloader_version_t booted = { 0 };

		printk("  Probing for firmware in flash...\n");
		lr1110_bootloader_reboot(ctx, false);
		k_msleep(500);

		if (lr1110_bootloader_get_version(ctx, &booted) == LR1110_STATUS_OK &&
		    booted.type == LR1110_TYPE_TRANSCEIVER) {
			printk("  Flash holds bootable firmware: TYPE=0x%02X FW=0x%04X\n",
			       booted.type, booted.fw);
			version = booted;
		} else {
			printk("  No bootable firmware in flash\n");
		}
	}

	if (version.type == LR1110_TYPE_TRANSCEIVER && version.fw == TARGET_FW_VERSION) {
		printk("\nAlready running target firmware 0x%04X — no update needed!\n",
		       TARGET_FW_VERSION);
		led_off();
		updater_done();
		return 0;
	}

	/* ── Steps 4-6: bootloader path, or resume a running loader ── */
	if (version.type == LR1110_TYPE_BL_UPDATER) {
		/* A previous attempt (ours or an external tool) flashed the
		 * loader and the chip now RUNS it. Resume directly with the
		 * rewrite — the running loader is proven good, and a BUSY-held
		 * reset back to the ROM bootloader to re-flash it would only
		 * add another chance for the flash write to fail. Identity
		 * reads are bootloader-mode commands, so they are skipped. */
		if (version.fw != lr1110_bl_loader_version()) {
			fatal_error("Unknown loader version — refusing to proceed");
		}
		printk("[4/8] Loader (bootloader-updater) already running — resuming Stage A\n");
		printk("[5/8] Chip identity: not readable in loader mode — skipped\n");
		printk("[6/8] Chip bootloader: 0x%04X (legacy)\n", LR1110_BL_VERSION_LEGACY);
		printk("\n--- Stage A (resumed): chip bootloader update 0x%04X -> 0x%04X ---\n\n",
		       LR1110_BL_VERSION_LEGACY, LR1110_BL_VERSION_UPDATED);
		loader_rewrite_and_verify(ctx);
	} else {
		if (version.type != LR1110_TYPE_TRANSCEIVER &&
		    version.type != LR1110_TYPE_BOOTLOADER) {
			fatal_error("Unknown chip type — cannot proceed");
		}

		/* Enter the bootloader the way RadioLib does: the SOFTWARE reboot
		 * command with stay_in_bootloader=true.
		 *
		 * We previously used the BUSY-held hardware reset (what SWTL001
		 * does). Both are documented, but RadioLib is the reference that
		 * demonstrably flashes this chip family from an ESP32 host, and the
		 * entry method is the last structural difference left between its
		 * flow and ours — opcodes, offsets, byte order and 64-word pages are
		 * all now verified identical. The BUSY-held reset stays as the
		 * fallback for a chip that will not answer commands at all. */
		printk("[4/8] Entering bootloader (software reboot, stay=true)...\n");
		lr1110_bootloader_reboot(ctx, true);
		k_msleep(500);

		rc = lr1110_bootloader_get_version(ctx, &version);
		if (rc != LR1110_STATUS_OK || version.type != LR1110_TYPE_BOOTLOADER) {
			printk("  Software entry did not land in bootloader —"
			       " falling back to BUSY-held hardware reset\n");
			if (lr1110_updater_reset_to_bootloader() != 0) {
				fatal_error("Failed to enter bootloader mode");
			}
		}

		rc = lr1110_bootloader_get_version(ctx, &version);
		if (rc != LR1110_STATUS_OK) {
			fatal_error("GetVersion in bootloader failed");
		}
		printk("  Bootloader: HW=0x%02X TYPE=0x%02X BL=0x%04X\n",
		       version.hw, version.type, version.fw);
		if (version.type != LR1110_TYPE_BOOTLOADER) {
			fatal_error("Not in bootloader mode after reset");
		}

		/* ── Step 5: Read chip identity ── */
		{
			lr1110_bootloader_pin_t pin = { 0 };
			lr1110_bootloader_chip_eui_t chip_eui = { 0 };
			lr1110_bootloader_join_eui_t join_eui = { 0 };

			lr1110_bootloader_read_pin(ctx, pin);
			lr1110_bootloader_read_chip_eui(ctx, chip_eui);
			lr1110_bootloader_read_join_eui(ctx, join_eui);

			printk("[5/8] Chip identity:\n");
			printk("  PIN     = 0x%02X%02X%02X%02X\n",
			       pin[0], pin[1], pin[2], pin[3]);
			printk("  ChipEUI = 0x%02X%02X%02X%02X%02X%02X%02X%02X\n",
			       chip_eui[0], chip_eui[1], chip_eui[2], chip_eui[3],
			       chip_eui[4], chip_eui[5], chip_eui[6], chip_eui[7]);
			printk("  JoinEUI = 0x%02X%02X%02X%02X%02X%02X%02X%02X\n",
			       join_eui[0], join_eui[1], join_eui[2], join_eui[3],
			       join_eui[4], join_eui[5], join_eui[6], join_eui[7]);
		}

		/* ── Step 6: Chip bootloader — update if legacy ── */
		printk("[6/8] Checking chip bootloader version...\n");
		if (version.fw != LR1110_BL_VERSION_UPDATED &&
		    version.fw != LR1110_BL_VERSION_LEGACY) {
			printk("  Unexpected bootloader version 0x%04X\n", version.fw);
			fatal_error("Unsupported chip bootloader — refusing to flash");
		}

		if (TARGET_FW_VERSION == 0x0402) {
			if (version.fw == LR1110_BL_VERSION_UPDATED) {
				printk("  Bootloader 0x%04X — already up to date\n", version.fw);
			} else if (!UPDATER_ALLOW_BOOTLOADER_UPDATE) {
				printk("\n");
				printk("  Bootloader update DISABLED for this build.\n");
				printk("\n");
				printk("  Flash an image that runs on bootloader 0x6500:\n");
				printk("    west build ... -- -DUPDATER_TARGET_FW=0x0401\n");
				printk("\n");
				printk("  Rebuild without -DUPDATER_ALLOW_BOOTLOADER_UPDATE=0\n");
				printk("  to perform the bootloader update.\n");
				fatal_error("Bootloader update disabled by build option");
			} else {
				printk("\n");
				printk("  Performing the ONE-WAY bootloader update 0x%04X -> 0x%04X.\n",
				       version.fw, LR1110_BL_VERSION_UPDATED);
				printk("  Semtech ships no reverse loader: this is permanent.\n");
				printk("  (Build with -DUPDATER_ALLOW_BOOTLOADER_UPDATE=0 to\n");
				printk("   flash firmware only and leave the bootloader alone.)\n");
				printk("\n");
				run_bootloader_update(ctx);
			}
		} else {
			/* Older target image — never touch the bootloader. */
			printk("  Bootloader 0x%04X, target FW 0x%04X\n",
			       version.fw, TARGET_FW_VERSION);
			if (version.fw == LR1110_BL_VERSION_UPDATED) {
				printk("  NOTE: Semtech's host-side table pairs FW 0x0401 with\n");
				printk("  bootloader 0x6500 only. Trying anyway, to find out\n");
				printk("  whether the chip itself enforces anti-rollback.\n");
			}
		}
	}

	/* ── Step 7: Flash transceiver firmware ── */
	printk("[7/8] Flashing transceiver firmware 0x%04X...\n", TARGET_FW_VERSION);
	erase_and_flash_image(ctx, "transceiver firmware",
			      lr11xx_firmware_image, LR11XX_FIRMWARE_IMAGE_SIZE);

	/* Boot the freshly written firmware with the bootloader's own reboot
	 * command (0x8005, stay=false) — and then DO NOT TOUCH NRESET.
	 *
	 * The LR1110 samples BUSY as NRESET is released and enters the
	 * bootloader whenever it reads LOW. Nothing holds that line high while
	 * the chip is in reset, so a hardware reset here lands back in the
	 * bootloader every time — which is precisely what the old "re-reset for
	 * a clean state" did, making this step report "firmware not running"
	 * no matter how good the flash was. Stage A reboots the very same way
	 * *without* a trailing hardware reset, and its image always came up
	 * running: that asymmetry is what exposed this.
	 *
	 * Firmware boot takes ~273 ms (datasheet); 500 ms is comfortable. */
	printk("  Rebooting LR1110 into new firmware...\n");
	lr1110_bootloader_reboot(ctx, false);
	k_msleep(500);

	/* The chip's own view of what it is executing. is_running_from_flash
	 * distinguishes "the image is bad" from "the image never got a chance
	 * to run", which the version read alone cannot. */
	{
		lr1110_bootloader_stat1_t stat1 = { 0 };
		lr1110_bootloader_stat2_t stat2 = { 0 };
		lr1110_bootloader_irq_mask_t irq = 0;

		if (lr1110_bootloader_get_status(ctx, &stat1, &stat2, &irq) == LR1110_STATUS_OK) {
			printk("  Post-reboot status: running_from_flash=%d chip_mode=%u "
			       "reset_status=%u cmd_status=%u\n",
			       stat2.is_running_from_flash, (unsigned)stat2.chip_mode,
			       (unsigned)stat2.reset_status, (unsigned)stat1.command_status);
		}
	}

	/* ── Step 8: Verify ── */
	printk("[8/8] Verifying new firmware version...\n");
	rc = lr1110_bootloader_get_version(ctx, &version);
	if (rc != LR1110_STATUS_OK) {
		fatal_error("GetVersion after flash failed");
	}

	printk("  HW   = 0x%02X\n", version.hw);
	printk("  TYPE = 0x%02X", version.type);
	if (version.type == LR1110_TYPE_TRANSCEIVER) {
		printk(" (transceiver firmware)\n");
	} else if (version.type == LR1110_TYPE_BOOTLOADER) {
		printk(" (bootloader — firmware not running!)\n");
	} else {
		printk(" (unknown)\n");
	}
	printk("  FW   = 0x%04X\n", version.fw);

	if (version.type == LR1110_TYPE_TRANSCEIVER && version.fw == TARGET_FW_VERSION) {
		printk("\n============================================\n");
		printk("  UPDATE SUCCESSFUL!\n");
		printk("  LR1110 firmware: 0x%04X\n", version.fw);
		printk("============================================\n");
		led_on();
	} else {
		printk("\nWARNING: Expected FW 0x%04X but got TYPE=0x%02X FW=0x%04X\n",
		       TARGET_FW_VERSION, version.type, version.fw);
		printk("The update may have failed. Try again.\n");
		led_off();
	}

	/* ── Hand off to main-firmware flashing ── */
	printk("\nDone!\n");
	updater_done();

	return 0; /* never reached */
}
