/*
 * SPDX-License-Identifier: MIT
 *
 * Production reboot-on-fatal-error handler.
 *
 * Zephyr has no built-in Kconfig to auto-reboot on a fatal error — the
 * default k_sys_fatal_error_handler() (weak, in kernel/fatal.c) flushes the
 * log and calls arch_system_halt(), spinning forever.  For unattended mesh
 * nodes that is the wrong behaviour: a transient stack overflow / CPU
 * exception / k_panic should recover the device rather than wedge it until a
 * manual power-cycle.
 *
 * When CONFIG_ZEPHCORE_RESET_ON_FATAL_ERROR=y (production default; debug.conf
 * forces it n so a debugger can inspect the halted core) we override the weak
 * handler to cold-reboot after flushing the panic log.  When it is n this
 * translation unit is empty and the weak default applies.
 */

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_RESET_ON_FATAL_ERROR)

#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(zephcore_fatal, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);

	/* Put logging into panic (synchronous) mode and flush so the fault
	 * reason actually reaches the console/RTT before we reset. */
	LOG_PANIC();
	LOG_ERR("Fatal error (reason %u) — cold rebooting", reason);

	sys_reboot(SYS_REBOOT_COLD);
	CODE_UNREACHABLE;
}

#endif /* CONFIG_ZEPHCORE_RESET_ON_FATAL_ERROR */
