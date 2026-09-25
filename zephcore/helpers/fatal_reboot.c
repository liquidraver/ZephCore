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
 * handler to cold-reboot after flushing the panic log.  When it is n the weak
 * default applies and there is no crash record.
 *
 * The handler also leaves {reason, pc, lr, thread} in RAM the boot does not
 * clear, so the next boot can say what crashed (get pwrmgt.bootreason, the
 * boot log) without a debugger: resolve pc with addr2line on the same build.
 */

#include <zephyr/kernel.h>
#include <string.h>

#include "boot_info.h"

#if IS_ENABLED(CONFIG_ZEPHCORE_RESET_ON_FATAL_ERROR)

#include <zephyr/init.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(zephcore_fatal, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

#define CRASH_MAGIC 0x7A43F417u

/* Survives the warm reboot on nRF; on ESP32 the bootloader in between may
 * reuse the RAM, which the magic and check reject. */
static __noinit struct {
	uint32_t magic;
	struct zephcore_crash crash;
	uint32_t check;
} s_record;

static struct zephcore_crash s_last;
static bool s_last_valid;

static uint32_t record_check(void)
{
	const uint32_t *w = (const uint32_t *)&s_record.crash;
	uint32_t sum = CRASH_MAGIC;

	for (size_t i = 0; i < sizeof(s_record.crash) / sizeof(uint32_t); i++) {
		sum = (sum << 5 | sum >> 27) ^ w[i];
	}
	return sum;
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	memset(&s_record.crash, 0, sizeof(s_record.crash));
	s_record.crash.reason = reason;
	if (esf != NULL) {
#if defined(CONFIG_ARM)
		s_record.crash.pc = esf->basic.pc;
		s_record.crash.lr = esf->basic.lr;
#elif defined(CONFIG_RISCV)
		s_record.crash.pc = (uint32_t)esf->mepc;
		s_record.crash.lr = (uint32_t)esf->ra;
#endif
	}
	const char *name = k_thread_name_get(k_current_get());

	if (name != NULL) {
		strncpy(s_record.crash.thread, name, sizeof(s_record.crash.thread) - 1);
	}
	s_record.magic = CRASH_MAGIC;
	s_record.check = record_check();

	/* Put logging into panic (synchronous) mode and flush so the fault
	 * reason actually reaches the console/RTT before we reset. */
	LOG_PANIC();
	LOG_ERR("Fatal error (reason %u) in %s pc 0x%08x lr 0x%08x — cold rebooting",
		reason, s_record.crash.thread, s_record.crash.pc, s_record.crash.lr);

	sys_reboot(SYS_REBOOT_COLD);
	CODE_UNREACHABLE;
}

/* POST_KERNEL, like the reset cause: take the record once, so it is reported
 * for the boot it caused and not again. */
static int crash_record_init(void)
{
	if (s_record.magic == CRASH_MAGIC && s_record.check == record_check()) {
		s_last = s_record.crash;
		s_last.thread[sizeof(s_last.thread) - 1] = '\0';
		s_last_valid = true;
		LOG_WRN("Last run crashed: reason %u in %s pc 0x%08x lr 0x%08x",
			s_last.reason, s_last.thread, s_last.pc, s_last.lr);
	}
	s_record.magic = 0;
	return 0;
}

SYS_INIT(crash_record_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

bool zephcore_boot_crash(struct zephcore_crash *out)
{
	if (!s_last_valid) {
		return false;
	}
	*out = s_last;
	return true;
}

#else

bool zephcore_boot_crash(struct zephcore_crash *out)
{
	ARG_UNUSED(out);
	return false;
}

#endif /* CONFIG_ZEPHCORE_RESET_ON_FATAL_ERROR */
