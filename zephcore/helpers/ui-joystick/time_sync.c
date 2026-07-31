/*
 * SPDX-License-Identifier: MIT
 */

#include <time_sync.h>
#include <adapters/gps/ZephyrGPSManager.h>
#include <zephyr/kernel.h>
#include <stdio.h>

static enum time_sync_source s_last = TIME_SYNC_NONE;
static int64_t s_last_uptime;   /* k_uptime_get() at the last report */

static const char *source_short_name(enum time_sync_source src)
{
	switch (src) {
	case TIME_SYNC_GPS:  return "GPS";
	case TIME_SYNC_APP:  return "App";
	case TIME_SYNC_WIFI: return "WiFi";
	case TIME_SYNC_CLI:  return "CLI";
	case TIME_SYNC_MESH: return "Mesh";
	default:             return NULL;
	}
}

void time_sync_report(enum time_sync_source src)
{
	s_last = src;
	s_last_uptime = k_uptime_get();
}

/* Freshness-windowed source for the top-bar clock tag. Local/manual (CLI)
 * and "never synced" both read as local; an external sync ages out after
 * CONFIG_ZEPHCORE_UI_TIME_SOURCE_FRESH_HOURS. (Distinct from the System >
 * Info > Time label below, which is not time-windowed.) */
enum time_sync_source time_sync_get_source(void)
{
	if (s_last == TIME_SYNC_NONE || s_last == TIME_SYNC_CLI) {
		return TIME_SYNC_CLI;
	}

#if CONFIG_ZEPHCORE_UI_TIME_SOURCE_FRESH_HOURS > 0
	int64_t age_ms = k_uptime_get() - s_last_uptime;
	int64_t window_ms =
		(int64_t)CONFIG_ZEPHCORE_UI_TIME_SOURCE_FRESH_HOURS * 3600 * 1000;

	if (age_ms > window_ms) {
		return TIME_SYNC_CLI;
	}
#endif

	return s_last;
}

const char *time_sync_display_label(void)
{
	if (gps_has_time_sync()) {
		return "GPS";
	}

	const char *src = source_short_name(s_last);
	if (!src) {
		return "None";
	}

	if (s_last != TIME_SYNC_GPS) {
		return src;
	}

	static char label[20];
	snprintf(label, sizeof(label), "Local (%s)", src);
	return label;
}
