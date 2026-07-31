/*
 * SPDX-License-Identifier: MIT
 * Sleep guard — block SoC light sleep across a critical stretch.
 *
 * Zephyr's policy locks are reference-counted, so every zc_pm_block_sleep()
 * needs exactly one matching zc_pm_unblock_sleep(); nested holders compose.
 * While at least one lock is held the idle path skips PM_STATE_STANDBY and the
 * SoC idles normally instead — device power management is untouched either way.
 *
 * Both calls compile to nothing without CONFIG_PM, so callers do not need to
 * guard their own use sites.
 */

#pragma once

#if defined(CONFIG_PM)

#include <zephyr/pm/policy.h>

static inline void zc_pm_block_sleep(void)
{
	pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
}

static inline void zc_pm_unblock_sleep(void)
{
	pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
}

#else /* !CONFIG_PM */

static inline void zc_pm_block_sleep(void) { }
static inline void zc_pm_unblock_sleep(void) { }

#endif /* CONFIG_PM */
