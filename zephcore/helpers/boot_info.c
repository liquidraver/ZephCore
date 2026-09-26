/*
 * SPDX-License-Identifier: MIT
 * Boot-time reset-cause capture. See boot_info.h.
 */

#include "boot_info.h"

#include <zephyr/init.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <stdio.h>

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_system.h>
#endif

LOG_MODULE_REGISTER(zephcore_boot, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

static uint32_t s_reset_cause;
static bool s_reset_cause_valid;

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
/* Zephyr's ESP32 hwinfo driver reports 0 for the reasons it has no case for,
 * the commonest being ESP_RST_USB: esptool's reset over USB-Serial-JTAG, i.e.
 * every flash. Map those onto the nearest RESET_* bit so they are labelled
 * instead of "Unknown". */
static uint32_t esp_unmapped_reset_cause(void)
{
	switch (esp_reset_reason()) {
	case ESP_RST_USB:
	case ESP_RST_JTAG:
		return RESET_DEBUG;
	case ESP_RST_PWR_GLITCH:
		return RESET_BROWNOUT;
	case ESP_RST_CPU_LOCKUP:
		return RESET_CPU_LOCKUP;
	case ESP_RST_EFUSE:
		return RESET_HARDWARE;
	default:
		return 0;
	}
}
#endif

/* POST_KERNEL: before main() on every role. */
static int boot_info_init(void)
{
	uint32_t cause = 0;

	if (hwinfo_get_reset_cause(&cause) != 0) {
		/* Nothing to capture; the accessor reports false. */
		return 0;
	}

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	if (cause == 0) {
		cause = esp_unmapped_reset_cause();
	}
#endif

	s_reset_cause = cause;
	s_reset_cause_valid = true;

	/* Logged here for every role: the clear below would otherwise lose the
	 * cause on roles that never read it. */
	char labels[128];

	(void)zephcore_boot_reset_cause_str(labels, sizeof(labels), false);
	LOG_INF("Reset cause: 0x%08x%s", cause, labels);

	/* Not reported again next boot. -ENOSYS where unimplemented (ESP32). */
	(void)hwinfo_clear_reset_cause();

	return 0;
}

SYS_INIT(boot_info_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

bool zephcore_boot_reset_cause(uint32_t *out)
{
	if (!s_reset_cause_valid) {
		return false;
	}

	*out = s_reset_cause;
	return true;
}

/*
 * One row per RESET_* bit in <zephyr/drivers/hwinfo.h>. A bit with no row is
 * rendered as nothing and is invisible to zephcore_boot_reset_cause_labelled().
 * hint is the plain-English suffix for the user-facing notice, NULL where
 * the label already says it or no honest one fits. SOFTWARE has none: it is
 * a reboot command, a firmware update and a fatal-error reboot alike.
 */
static const struct {
	uint32_t bit;
	const char *label;
	const char *hint;
} s_labels[] = {
	{ RESET_PIN,            "PIN",         "(reset button)" },
	{ RESET_SOFTWARE,       "SOFTWARE" },
	{ RESET_BROWNOUT,       "BROWNOUT",    "(low voltage)" },
	{ RESET_POR,            "POR",         "(power-on)" },
	{ RESET_WATCHDOG,       "WATCHDOG",    "(hang)" },
	{ RESET_DEBUG,          "DEBUG",       "(debugger)" },
	{ RESET_SECURITY,       "SECURITY" },
	{ RESET_LOW_POWER_WAKE, "LOWPOWER",    "(wake from off)" },
	{ RESET_CPU_LOCKUP,     "LOCKUP",      "(crash)" },
	{ RESET_PARITY,         "PARITY",      "(memory error)" },
	{ RESET_PLL,            "PLL",         "(clock fault)" },
	{ RESET_CLOCK,          "CLOCK" },
	{ RESET_HARDWARE,       "HARDWARE" },
	{ RESET_USER,           "USER" },
	{ RESET_TEMPERATURE,    "TEMPERATURE", "(overheat)" },
	{ RESET_BOOTLOADER,     "BOOTLOADER" },
	{ RESET_FLASH,          "FLASH" },
};

int zephcore_boot_reset_cause_str(char *buf, size_t cap, bool hints)
{
	if (buf == NULL || cap == 0) {
		return 0;
	}

	buf[0] = '\0';

	if (!s_reset_cause_valid) {
		return 0;
	}

	size_t n = 0;

	for (size_t i = 0; i < ARRAY_SIZE(s_labels); i++) {
		if ((s_reset_cause & s_labels[i].bit) == 0) {
			continue;
		}

		const char *hint = (hints && s_labels[i].hint) ? s_labels[i].hint : "";
		int w = snprintf(buf + n, cap - n, " %s%s", s_labels[i].label, hint);

		if (w < 0 || (size_t)w >= cap - n) {
			/* Stop rather than emit half a label. */
			buf[n] = '\0';
			break;
		}

		n += (size_t)w;
	}

	return (int)n;
}

uint32_t zephcore_boot_reset_cause_labelled(void)
{
	uint32_t mask = 0;

	if (!s_reset_cause_valid) {
		return 0;
	}

	for (size_t i = 0; i < ARRAY_SIZE(s_labels); i++) {
		mask |= s_reset_cause & s_labels[i].bit;
	}

	return mask;
}
