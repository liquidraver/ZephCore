/*
 * SPDX-License-Identifier: MIT
 *
 * The MCU's reset cause, captured and cleared once at POST_KERNEL on every
 * role. hwinfo's flags accumulate between resets on some platforms, so
 * clearing after reading is what isolates the most recent one.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The cause hwinfo reported at boot. out must be non-NULL. False means hwinfo
 * could not report one, which differs from a cause of 0. Call from main() or
 * later; an initialiser at or before POST_KERNEL may get false.
 */
bool zephcore_boot_reset_cause(uint32_t *out);

/*
 * Render as space-prefixed labels, e.g. " PIN SOFTWARE", or with hints set,
 * " PIN(reset button) SOFTWARE". Returns characters written, excluding the
 * NUL; 0 means nothing to render. Writes nothing when buf is NULL or cap is 0,
 * otherwise always NUL-terminates, truncating at the first label that does not
 * fit. 127 bytes hold every label without hints, 239 with.
 */
int zephcore_boot_reset_cause_str(char *buf, size_t cap, bool hints);

/*
 * The part of the cause the renderer would name. Use this rather than the raw
 * cause when deciding whether a cause is worth reporting, so an unlabelled bit
 * cannot trigger a report that then names unrelated labels.
 */
uint32_t zephcore_boot_reset_cause_labelled(void);

/* The fatal error that ended the previous run (helpers/fatal_reboot.c). pc
 * and lr are 0 on Xtensa. */
struct zephcore_crash {
	uint32_t reason;        /* K_ERR_* */
	uint32_t pc;
	uint32_t lr;
	char thread[20];
};

/* True if the previous run ended in a fatal error and rebooted (production
 * builds, CONFIG_ZEPHCORE_RESET_ON_FATAL_ERROR). Call from main() or later. */
bool zephcore_boot_crash(struct zephcore_crash *out);

#ifdef __cplusplus
}
#endif
