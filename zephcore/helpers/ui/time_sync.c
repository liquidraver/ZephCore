/*
 * SPDX-License-Identifier: MIT
 * Which source last set the RTC, for the UIs' top-bar clock tag and the
 * joystick UI's System > Info > Time label. One file for both UIs.
 *
 * The tag reflects *freshness*: an external sync (GPS / app / network) is
 * shown only while it is recent (CONFIG_ZEPHCORE_UI_TIME_SOURCE_FRESH_HOURS).
 * Once it ages out — or after a reboot, before any sync — the source reads as
 * local (the RTC is free-running on its own oscillator).
 */

#include <time_sync.h>
#include <adapters/gps/ZephyrGPSManager.h>
#include <zephyr/kernel.h>
#include <stdio.h>

static enum time_sync_source s_last = TIME_SYNC_NONE;
static int64_t s_last_uptime;   /* k_uptime_get() at the last report */

void time_sync_report(enum time_sync_source src)
{
	s_last = src;
	s_last_uptime = k_uptime_get();
}

enum time_sync_source time_sync_get_source(void)
{
	/* Live GPS hold is authoritative and uses the SAME signal that blocks
	 * phone time-sync (gps_has_time_sync(), expires 2h after the last fix),
	 * so the tag can never disagree with that policy: while GPS holds the
	 * clock the tag is G regardless of the freshness window below. After a
	 * reboot the flag is reset, so this returns false and we fall through to
	 * the local/freshness logic (→ L until something re-syncs). */
	if (gps_has_time_sync()) {
		return TIME_SYNC_GPS;
	}

	/* Local/manual (CLI) and "never synced" are both just local time. */
	if (s_last == TIME_SYNC_NONE || s_last == TIME_SYNC_CLI) {
		return TIME_SYNC_CLI;
	}

#if CONFIG_ZEPHCORE_UI_TIME_SOURCE_FRESH_HOURS > 0
	int64_t age_ms = k_uptime_get() - s_last_uptime;
	int64_t window_ms =
		(int64_t)CONFIG_ZEPHCORE_UI_TIME_SOURCE_FRESH_HOURS * 3600 * 1000;

	if (age_ms > window_ms) {
		/* External sync has gone stale — fall back to local. */
		return TIME_SYNC_CLI;
	}
#endif

	return s_last;   /* GPS / APP / WIFI / MESH, still fresh */
}

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

/* The last source, not time-windowed (unlike the tag above): a GPS that
 * no longer holds the clock reads "Local (GPS)". */
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
