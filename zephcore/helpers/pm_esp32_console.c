/*
 * SPDX-License-Identifier: MIT
 * ESP32 light-sleep console guards.
 *
 * Two independent problems sit between ESP32 light sleep and a usable serial
 * console.  This file solves both; neither fix substitutes for the other.
 *
 * 1. TRUNCATED OUTPUT.  soc/espressif/common/power.c calls
 *    esp_light_sleep_start() with no wait for the console UART to drain — the
 *    TX-idle flush that ESP-IDF performs lives behind its own console Kconfig,
 *    which is not part of a Zephyr build.  Sleeping with bytes still in the
 *    FIFO cuts the line mid-character.  A pm_notifier's state_entry callback
 *    runs immediately before pm_state_set() (subsys/pm/pm.c), which is exactly
 *    the right moment to poll the UART to idle.
 *
 * 2. UNREACHABLE INPUT.  Nothing arms a UART wake source — Zephyr's PM path
 *    never calls esp_sleep_enable_uart_wakeup(), and on the boards this runs on
 *    the console RX pad (GPIO44 on an S3 uart0) is outside the RTC range that
 *    could carry a GPIO wake instead.  Characters typed at a sleeping node are
 *    therefore dropped, and no amount of TX handling changes that.
 *
 *    The answer here is a boot window rather than a wake source: sleep is
 *    blocked outright for ZEPHCORE_PM_BOOT_AWAKE_MS after boot, so a node is
 *    always reachable for that long.  This works better in practice than it
 *    sounds, because the USB bridge on these boards drives EN from DTR — a
 *    terminal that asserts DTR on open resets the board, which re-arms the
 *    window at the moment someone connects.  A terminal that does not toggle
 *    DTR gets no window, and the node has to be power-cycled to answer over
 *    USB; that is the known limitation of this approach.  Arming
 *    esp_sleep_enable_uart_wakeup() and re-taking the lock on console activity
 *    would remove it, at the cost of the first keystroke (the hardware
 *    consumes it as the wake trigger).
 *
 * Remote admin over LoRa is unaffected by any of this: DIO1 is armed as a wake
 * source by patches/zephyr/0012 and wakes the SoC on a received packet.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>

#include <esp_rom_uart.h>

#include "pm_sleep_guard.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_pm, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* ========== 1. Flush the console before every sleep ========== */

static void pm_console_flush(enum pm_state state)
{
	if (state != PM_STATE_STANDBY) {
		return;
	}

	/* Polls a status register until the shifter empties — no locking, so it
	 * is safe in this context (called with the scheduler locked).  Bounded
	 * by the FIFO depth at the configured baud: ~11 ms worst case for a full
	 * 128-byte FIFO at 115200, and normally microseconds. */
	esp_rom_uart_tx_wait_idle(CONFIG_ZEPHCORE_PM_CONSOLE_UART_NUM);
}

static struct pm_notifier console_notifier = {
	.state_entry = pm_console_flush,
};

/* ========== 2. Keep the node awake for a window after boot ========== */

#if CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS > 0

static void boot_window_expired(struct k_work *work)
{
	ARG_UNUSED(work);

	zc_pm_unblock_sleep();
	LOG_INF("boot window elapsed — light sleep enabled "
		"(USB console answers again after a reset)");
}

static K_WORK_DELAYABLE_DEFINE(boot_window_work, boot_window_expired);

#endif /* CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS > 0 */

static int pm_console_init(void)
{
	pm_notifier_register(&console_notifier);

#if CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS > 0
	/* Taken here rather than released here: the lock is held from init and
	 * dropped by the work item, so there is no window at startup in which
	 * the node could sleep before the guard is in place. */
	zc_pm_block_sleep();
	k_work_schedule(&boot_window_work,
			K_MSEC(CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS));

	LOG_INF("light sleep deferred %d ms (console configuration window)",
		CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS);
#endif

	return 0;
}

/* POST_KERNEL: needs the kernel work queue, and must be in place before the
 * application can idle long enough to sleep. */
SYS_INIT(pm_console_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
