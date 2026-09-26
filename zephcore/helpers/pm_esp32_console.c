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
 *    The answer here is a console window rather than a wake source: sleep is
 *    blocked outright for ZEPHCORE_PM_BOOT_AWAKE_MS after boot, and again
 *    after every press of the user button (helpers/pm_esp32_wake.c), so a
 *    node is reachable for that long whenever someone is standing at it.
 *    On Heltec V3 a terminal that asserts DTR on open also resets the board
 *    through its USB bridge, which opens the window too.  Boards that console
 *    on the S3's own USB Serial/JTAG (V4, V4.3, Wireless Tracker V2, XIAO,
 *    Station G2) do not reset on open, and their USB device detaches on every
 *    sleep; the button is the way back in, or `powersaving off` over LoRa.
 *
 * Remote admin over LoRa is unaffected by any of this: the radio's IRQ line
 * wakes the SoC on a received packet (helpers/pm_esp32_wake.c).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>

#include <esp_rom_serial_output.h>

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
	esp_rom_output_tx_wait_idle(CONFIG_ZEPHCORE_PM_CONSOLE_UART_NUM);
}

static struct pm_notifier console_notifier = {
	.state_entry = pm_console_flush,
};

/* ========== 2. Keep the node awake for a console window ========== */

/* 1 while the window holds its sleep lock. The lock is taken only on the
 * 0 -> 1 edge and released only on 1 -> 0, so re-opening an open window just
 * moves its deadline. */
static atomic_t window_locked;

static void console_window_expired(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_cas(&window_locked, 1, 0)) {
		zc_pm_unblock_sleep();
		LOG_INF("console window closed — light sleep enabled "
			"(press the user button to reopen it)");
	}
}

static K_WORK_DELAYABLE_DEFINE(console_window_work, console_window_expired);

void zc_pm_console_window_open(void)
{
#if CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS > 0
	if (atomic_cas(&window_locked, 0, 1)) {
		zc_pm_block_sleep();
	}
	k_work_reschedule(&console_window_work,
			  K_MSEC(CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS));
#endif
}

uint32_t zc_pm_console_window_remaining_ms(void)
{
	if (!atomic_get(&window_locked)) {
		return 0;
	}
	return (uint32_t)k_ticks_to_ms_floor64(
		k_work_delayable_remaining_get(&console_window_work));
}

static int pm_console_init(void)
{
	pm_notifier_register(&console_notifier);

	/* Opened here, at POST_KERNEL, so there is no stretch at startup in
	 * which the node could sleep before the guard is in place. */
	zc_pm_console_window_open();
	LOG_INF("light sleep deferred %d ms (console configuration window)",
		CONFIG_ZEPHCORE_PM_BOOT_AWAKE_MS);

	return 0;
}

/* POST_KERNEL: needs the kernel work queue, and must be in place before the
 * application can idle long enough to sleep. */
SYS_INIT(pm_console_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
