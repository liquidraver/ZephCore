/*
 * ZephCore Universal Flash Formatter
 *
 * Erases all filesystem partitions and reboots into Adafruit UF2 DFU mode
 * for clean firmware installation.
 *
 * Only TWO UF2 binaries are needed for all nRF52840 boards:
 *   - SDv6: covers RAK4631, ThinkNode M1/M3/M6, ProMicro LR2021, RAK WisMesh Tag
 *   - SDv7: covers T1000-E, Wio Tracker L1, Ikoka Nano 30dBm
 *
 * Internal flash uses DTS FIXED_PARTITION_EXISTS — partition layout is identical
 * across all boards with the same SoftDevice version.
 *
 * QSPI external flash uses bare-metal register probing (qspi_probe.c) — no
 * Zephyr QSPI driver needed. The probe table contains known pin configurations
 * for all QSPI-capable boards; it auto-detects which board is running and
 * erases the external flash if present, or silently skips if not.
 *
 * Build:
 *   SDv6:  west build -b rak4631/nrf52840  zephcore/tools/formatter --pristine
 *   SDv7:  west build -b t1000_e/nrf52840  zephcore/tools/formatter --pristine
 *
 * Flash: drag-drop build/zephyr/zephyr.uf2 onto UF2 drive
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>
#include "qspi_probe.h"

#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF52)
#include <hal/nrf_power.h>
#define IS_NRF52 1
#else
#define IS_NRF52 0
#endif

/* Adafruit UF2 bootloader magic — enter mass storage DFU mode */
#define BOOTLOADER_DFU_UF2_MAGIC  0x57

/* ── LED feedback (optional — may not match actual board) ──── */

#if DT_NODE_EXISTS(DT_ALIAS(led0)) && !IS_NRF52
/* Only use DTS LED on non-nRF52 (board-specific builds).
 * On nRF52 universal builds, skip LED to avoid toggling
 * wrong pins on boards other than the build target. */
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static void led_init(void)  { gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); }
static void led_on(void)    { gpio_pin_set_dt(&led, 1); }
static void led_off(void)   { gpio_pin_set_dt(&led, 0); }
#else
#define HAS_LED 0
static void led_init(void)  {}
static void led_on(void)    {}
static void led_off(void)   {}
#endif

/* ── Flash erase helper ──────────────────────────────────────── */

static int erase_partition(uint8_t id, const char *name)
{
	const struct flash_area *fa;
	int rc;

	rc = flash_area_open(id, &fa);
	if (rc) {
		printk("  %s: open failed (rc %d) — skipped\n", name, rc);
		return rc;
	}

	printk("  %s: erasing 0x%lx - 0x%lx (%u KB)...",
	       name,
	       (unsigned long)fa->fa_off,
	       (unsigned long)(fa->fa_off + fa->fa_size),
	       (unsigned)(fa->fa_size / 1024));

	rc = flash_area_erase(fa, 0, fa->fa_size);
	if (rc) {
		printk(" FAILED (rc %d)\n", rc);
	} else {
		printk(" OK\n");
	}

	flash_area_close(fa);
	return rc;
}

/* ── Main ────────────────────────────────────────────────────── */

int main(void)
{
	int errors = 0;

	/* Brief delay for USB CDC to enumerate */
	k_msleep(2000);

	printk("\n");
	printk("=== ZephCore Flash Formatter ===\n");
	printk("\n");

	led_init();
	led_on();

	/* ── LittleFS partition (all platforms) ── */
#if FIXED_PARTITION_EXISTS(lfs_partition)
	if (erase_partition(FIXED_PARTITION_ID(lfs_partition), "LittleFS (/lfs)")) {
		errors++;
	}
#endif

	/* ── NVS storage partition (ESP32 etc.) ── */
#if FIXED_PARTITION_EXISTS(storage_partition)
	if (erase_partition(FIXED_PARTITION_ID(storage_partition), "NVS (storage)")) {
		errors++;
	}
#endif

	/* ── QSPI external flash (auto-detect via pin probing) ── */
	qspi_probe_and_erase();  /* Probes all known boards; skips if no flash found */

	led_off();

	/* ── Summary ── */
	printk("\n");
	if (errors) {
		printk("Completed with %d error(s).\n", errors);
	} else {
		printk("All partitions erased successfully.\n");
	}

	printk("Rebooting into UF2 DFU mode...\n");
	printk("You can now drag-drop your new firmware UF2.\n");
	k_msleep(500);

	/* ── Reboot to UF2 bootloader ── */
#if IS_NRF52
	nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_DFU_UF2_MAGIC);
#endif
	sys_reboot(SYS_REBOOT_COLD);

	return 0; /* never reached */
}
