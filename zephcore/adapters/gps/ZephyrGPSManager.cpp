/*
 * SPDX-License-Identifier: MIT
 * Zephyr GPS Manager - GNSS power management, fix acquisition, constellation config
 *
 * Event-driven state machine with no polling loops:
 * - LoRa/BLE events trigger GPS enable/disable
 * - k_work_delayable handles standby/timeout timers
 * - GNSS callback fires on fix data from driver
 *
 * Power strategy:
 * - Direct GPIO toggle via gps-enable alias (all boards)
 * - T1000-E warm standby: VRTC stays powered during standby, preserving
 *   ephemeris/almanac/RTC in backup RAM for fast re-acquisition (3-8s vs 15-45s)
 * - GNSS UARTE suspended (device PM) while GPS is off/standby — releases
 *   HFCLK on nRF52840 (~0.5-1 mA), resumed before every wake
 * - Full power-off only on user-disable or System OFF
 */

#include "gps_internal.h"
#include "ZephyrGPSManager.h"
#include <helpers/NodePrefs.h>  /* clampGpsInterval */
#include "../../helpers/pm_sleep_guard.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/timeutil.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(zephcore_gps, CONFIG_ZEPHCORE_GPS_LOG_LEVEL);

/* ========== GPS State - Power Management ========== */
#if HAS_GNSS
static struct gps_position current_pos;
static struct gnss_time current_utc;
static bool gps_enabled = false;
static bool gps_available = false;
static K_MUTEX_DEFINE(gps_mutex);
static gps_enable_callback_t gps_enable_cb = NULL;
static gps_fix_callback_t gps_fix_cb = NULL;
static gps_event_callback_t gps_event_cb = NULL;

/* Pending GPS actions — set by work handlers (system work queue),
 * consumed by gps_process_event() (main thread).
 * This avoids calling blocking GNSS APIs from the system work queue,
 * which deadlocks because modem_chat_run_script() blocks on a semaphore
 * that's signaled from the same work queue. */
#define GPS_ACTION_WAKE     BIT(0)  /* Wake from standby → start acquiring */
#define GPS_ACTION_TIMEOUT  BIT(1)  /* Acquisition timeout → go to standby */
#define GPS_ACTION_FIX_DONE BIT(2)  /* Got enough good fixes → go to standby */
#define GPS_ACTION_FIX      BIT(3)  /* A validated fix for gps_fix_cb (fix_pending) */
static atomic_t pending_gps_actions;

/* GPS Power Management State Machine */
enum gps_state {
	GPS_STATE_OFF,          /* GPS disabled by user */
	GPS_STATE_STANDBY,      /* GPS enabled but asleep until the next duty wake */
	GPS_STATE_ACQUIRING,    /* GPS awake, waiting for fixes */
};

static enum gps_state gps_current_state = GPS_STATE_OFF;
static uint8_t consecutive_good_fixes = 0;
static bool first_fix_acquired = false;  /* True after first 3-good-fix cycle since enable. Cleared on gps_enable(false) and at boot. */
static bool first_acquire_used = false;  /* True once the one-time long cold-start window has ended (fix or timeout). Cleared on gps_enable(false) and at boot. */
static bool gps_time_synced = false;     /* True after GPS syncs RTC. Starts false at boot (RTC reset),
										  * set true after 3 good fixes, cleared when GPS disabled. */
static int64_t last_fix_uptime_ms = 0;  /* k_uptime when last validated fix was acquired */
static int64_t standby_start_ms = 0;    /* k_uptime when standby started (for next-wake calc) */
static uint64_t standby_interval_ms = 0; /* How long standby lasts (for next-wake calc) */

#define GPS_GOOD_FIX_COUNT       3       /* Need 3 consecutive good fixes */
#define GPS_MIN_SATELLITES       4       /* Minimum satellites for valid fix */

/* Acquire windows (Kconfig) and the duty interval (prefs, set at boot). */
static const uint32_t gps_acquire_timeout_ms   = CONFIG_ZEPHCORE_GPS_FIX_TIMEOUT_SEC * 1000U;
static const uint32_t gps_first_fix_timeout_ms = CONFIG_ZEPHCORE_GPS_FIRST_FIX_TIMEOUT_SEC * 1000U;
static uint32_t gps_wake_interval_ms           = CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC * 1000U;

/* Duty cycle vs always-on: a non-zero standby interval duty-cycles; interval 0
 * keeps the GPS in continuous acquisition (never sleeps) so it streams fresh
 * fixes for telemetry and can download a full almanac. */
static inline bool gps_duty_cycling(void)
{
	return gps_wake_interval_ms != 0;
}

/* k_uptime of the last always-on fix-callback invocation — rate-limits the
 * RTC sync / node-position update to gps_acquire_timeout_ms while streaming
 * (see gnss_data_cb), so 1Hz fixes don't fire the callback continuously. */
static int64_t last_promote_ms = 0;

/* Repeater acquire window — GPS only for time sync. The standby interval is
 * unified with companions via gps_wake_interval_ms (prefs.gps_interval). */
#define GPS_REPEATER_SYNC_TIMEOUT_MS   (5 * 60 * 1000)           /* 5 minutes */

/* The validated fix waiting for gps_process_event() to hand to gps_fix_cb on
 * the main thread. Under gps_mutex. */
static struct {
	int64_t lat_ndeg;
	int64_t lon_ndeg;
	struct gnss_time utc;
	int64_t at_ms;          /* k_uptime at validation, to age the UTC */
} fix_pending;

static bool gps_repeater_mode = false;  /* True = repeater (time sync only), False = companion */
static bool gnss_activity_seen_this_cycle = false;  /* Runtime-only: set by GNSS callback while acquiring */

/* Forward declarations for work handlers and state functions */
static void gps_wake_work_fn(struct k_work *work);
static void gps_timeout_work_fn(struct k_work *work);
static void gps_go_to_standby(void);
static void gps_start_acquiring(void);

/* Delayable work for event-driven timers (no polling!) */
static K_WORK_DELAYABLE_DEFINE(gps_wake_work, gps_wake_work_fn);
static K_WORK_DELAYABLE_DEFINE(gps_timeout_work, gps_timeout_work_fn);

#else
static gps_enable_callback_t gps_enable_cb = NULL;
#endif

void gps_set_enable_callback(gps_enable_callback_t cb)
{
	gps_enable_cb = cb;
}

void gps_set_fix_callback(gps_fix_callback_t cb)
{
#if HAS_GNSS
	gps_fix_cb = cb;
#else
	ARG_UNUSED(cb);
#endif
}

void gps_set_event_callback(gps_event_callback_t cb)
{
#if HAS_GNSS
	gps_event_cb = cb;
#else
	ARG_UNUSED(cb);
#endif
}

#if HAS_GNSS

/* GNSS UTC to Unix time; 0 if the date is out of range. */
static int64_t gnss_time_to_unix(const struct gnss_time *t)
{
	/* The GNSS driver is trusted for ranges no further than this. */
	if (t->month < 1 || t->month > 12 || t->month_day < 1 || t->month_day > 31) {
		return 0;
	}
	struct tm tm = {
		.tm_sec = t->millisecond / 1000,
		.tm_min = t->minute,
		.tm_hour = t->hour,
		.tm_mday = t->month_day,
		.tm_mon = t->month - 1,
		.tm_year = 100 + t->century_year,
	};
	return (int64_t)timeutil_timegm(&tm);
}

/* Snapshot a validated fix for gps_process_event(). gps_mutex held; the
 * caller posts the event after unlocking. */
static void gps_post_fix_locked(const struct gnss_data *data)
{
	fix_pending.lat_ndeg = data->nav_data.latitude;
	fix_pending.lon_ndeg = data->nav_data.longitude;
	fix_pending.utc = data->utc;
	fix_pending.at_ms = k_uptime_get();
	atomic_or(&pending_gps_actions, GPS_ACTION_FIX);
}

/* GNSS callback - called when new fix data is available */
static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	ARG_UNUSED(dev);

	if (!gps_enabled || gps_current_state == GPS_STATE_STANDBY) {
		/* GPS disabled or in standby — ignore NMEA data.
		 * The GNSS driver fires callbacks as long as the UART has data,
		 * even after we de-assert GPS_EN (module drains its buffer).
		 * On boards without GPS power control (e.g. RAK3401 where 3V3_S
		 * rail is shared with LoRa FEM), the GPS module stays powered in
		 * standby and keeps streaming NMEA — suppress those callbacks to
		 * avoid log spam and wasted CPU for the entire standby period. */
		return;
	}

	if (gps_current_state == GPS_STATE_ACQUIRING) {
		/* Any callback means GNSS hardware/UART path is alive, even without a fix. */
		gnss_activity_seen_this_cycle = true;
	}

	LOG_DBG("GNSS callback: fix=%d sats=%d state=%d",
		data->info.fix_status, data->info.satellites_cnt, gps_current_state);

	k_mutex_lock(&gps_mutex, K_FOREVER);

	if (data->info.fix_status >= GNSS_FIX_STATUS_GNSS_FIX) {
		/* Reject "Null Island" (0,0) fixes. Zephyr's NMEA parser splits
		 * data across callbacks: gnss_nmea0183_parse_gga fills altitude +
		 * fix_status but NOT lat/lon; gnss_nmea0183_parse_rmc fills
		 * lat/lon. A merged publish fires when both GGA and RMC share a
		 * UTC. If the chip emits GGA quality=1 while RMC is still 'V'
		 * (or reports null-island coords), parse_rmc's early-exit on 'V'
		 * leaves lat/lon at their previous value (zero at first boot, or
		 * stale) while altitude advances — the caller sees valid
		 * fix_status + altitude-only motion + (0,0) coords. Observed on
		 * AT6558R (RAK WisMesh Tag) during early acquisition; Air530Z
		 * (ThinkNode M1) doesn't desync GGA/RMC this way. (0,0) is never
		 * a real fix — skip so we don't poison current_pos (and with it
		 * telemetry and the node position) or promote
		 * consecutive_good_fixes. */
		if (data->nav_data.latitude == 0 && data->nav_data.longitude == 0) {
			LOG_DBG("GPS: Ignoring (0,0) fix — GGA/RMC desync "
				"(fix=%d sats=%d alt_mm=%d)",
				data->info.fix_status,
				data->info.satellites_cnt,
				data->nav_data.altitude);
			if (gps_current_state == GPS_STATE_ACQUIRING &&
			    consecutive_good_fixes > 0) {
				consecutive_good_fixes = 0;
			}
			k_mutex_unlock(&gps_mutex);
			return;
		}

		current_pos.latitude_ndeg = data->nav_data.latitude;
		current_pos.longitude_ndeg = data->nav_data.longitude;
		current_pos.altitude_mm = data->nav_data.altitude;
		current_pos.satellites = data->info.satellites_cnt;
		current_pos.valid = true;
		current_pos.timestamp_ms = k_uptime_get();
		current_utc = data->utc;

		/* Fix validation during acquisition */
		if (gps_current_state == GPS_STATE_ACQUIRING) {
			if (data->info.satellites_cnt >= GPS_MIN_SATELLITES) {
				consecutive_good_fixes++;
				LOG_INF("GPS: Good fix %d/%d (sats=%d) lat=%lld lon=%lld",
					consecutive_good_fixes, GPS_GOOD_FIX_COUNT,
					data->info.satellites_cnt,
					current_pos.latitude_ndeg / 1000000,
					current_pos.longitude_ndeg / 1000000);

				if (consecutive_good_fixes >= GPS_GOOD_FIX_COUNT) {
					bool duty = gps_duty_cycling();
					bool first_ever = !first_fix_acquired;

					LOG_INF("GPS: Got %d good fixes, updating location/time",
						GPS_GOOD_FIX_COUNT);

					/* Mark first fix acquired (enables timeout for future cycles) */
					first_fix_acquired = true;

					/* Mark time as synced from GPS - blocks phone time sync */
					gps_time_synced = true;
					last_fix_uptime_ms = k_uptime_get();

					if (duty) {
						/* Cancel timeout */
						k_work_cancel_delayable(&gps_timeout_work);

						/* One fix per window: further good fixes before the
						 * main thread's standby must not deliver it again. */
						consecutive_good_fixes = 0;

						/* The fix, then standby, both on the main thread —
						 * we're on the system workqueue here (GNSS callback),
						 * can't call PM suspend (modem_chat_run_script
						 * deadlocks on same workqueue). */
						gps_post_fix_locked(data);
						atomic_or(&pending_gps_actions, GPS_ACTION_FIX_DONE);
						k_mutex_unlock(&gps_mutex);
						if (gps_event_cb) {
							gps_event_cb();
						}
						return;
					}

					/* Always-on: keep streaming (no standby). Throttle the fix
					 * callback (RTC sync + node-position update in main) to
					 * once per gps_acquire_timeout_ms — at 1Hz fixes it would
					 * otherwise fire constantly. Always fire on the first-ever
					 * fix so the clock syncs right away. current_pos (the
					 * telemetry source) is updated on every fix above. */
					consecutive_good_fixes = 0;
					bool promote = first_ever ||
						(k_uptime_get() - last_promote_ms >=
						 (int64_t)gps_acquire_timeout_ms);
					if (promote) {
						last_promote_ms = k_uptime_get();
						gps_post_fix_locked(data);
					}
					k_mutex_unlock(&gps_mutex);
					if (promote && gps_event_cb) {
						gps_event_cb();
					}
					return;
				}
			} else {
				/* Reset counter on bad fix (< 4 satellites) */
				if (consecutive_good_fixes > 0) {
					LOG_DBG("GPS: Poor fix (sats=%d), resetting counter",
						data->info.satellites_cnt);
				}
				consecutive_good_fixes = 0;
			}
		}
	} else {
		/* Don't clear current_pos — preserve last good fix for telemetry.
		 * Only reset the consecutive fix counter during acquisition. */
		if (gps_current_state == GPS_STATE_ACQUIRING && consecutive_good_fixes > 0) {
			LOG_DBG("GPS: No fix, resetting counter");
			consecutive_good_fixes = 0;
		}

		/* Periodic status at INF level so user knows NMEA is flowing.
		 * Without this, GPS is completely silent until first fix (all
		 * NMEA parsing is at DBG level in the driver). */
		if (gps_current_state == GPS_STATE_ACQUIRING) {
			static int64_t last_status_ms;
			int64_t now = k_uptime_get();
			if (now - last_status_ms >= 10000) {
				LOG_INF("GPS: Searching... sats=%d fix=%d",
					data->info.satellites_cnt,
					data->info.fix_status);
				last_status_ms = now;
			}
		}
	}

	k_mutex_unlock(&gps_mutex);
}

/* Register GNSS callback for all GNSS devices */
GNSS_DATA_CALLBACK_DEFINE(NULL, gnss_data_cb);

#ifdef CONFIG_ZEPHCORE_GPS_SAT_DIAG
/* ========== Per-constellation satellite tally (diagnostic) ==========
 * The Zephyr GSV parser fills gnss_satellite.system from the NMEA talker ID
 * ($GPGSV/$GLGSV/$GAGSV/$GBGSV), so this is direct evidence of which
 * constellations the module is actually tracking — the only way to confirm
 * that the boot-time PMTK353 / UBX-CFG-GNSS configuration was accepted.
 * A module still in its GPS-only default yields sats_gps only.
 *
 * Counts only tracked satellites (is_tracked), not merely visible ones. */
static uint8_t sat_count[5];    /* gps, glonass, galileo, beidou, other */
static int64_t sat_seen_ms[5];  /* uptime when each bucket was last reported */

/* A constellation absent for this long is reported as zero. Long enough to
 * ride out a missed GSV cycle (they repeat at the fix rate), short enough
 * that a constellation which genuinely drops out stops being claimed. */
#define SAT_TALLY_STALE_MS 30000

static void gnss_satellites_cb(const struct device *dev,
			       const struct gnss_satellite *satellites,
			       uint16_t size)
{
	ARG_UNUSED(dev);
	uint8_t tally[5] = { 0 };
	bool seen[5] = { false };

	for (uint16_t i = 0; i < size; i++) {
		int idx;

		switch (satellites[i].system) {
		case GNSS_SYSTEM_GPS:     idx = 0; break;
		case GNSS_SYSTEM_GLONASS: idx = 1; break;
		case GNSS_SYSTEM_GALILEO: idx = 2; break;
		case GNSS_SYSTEM_BEIDOU:  idx = 3; break;
		default:                  idx = 4; break;
		}

		/* Mark the constellation as reported even when nothing in it is
		 * tracked — that is a real "zero", distinct from "not heard". */
		seen[idx] = true;
		if (satellites[i].is_tracked) {
			tally[idx]++;
		}
	}

	/* One GSV burst carries ONE constellation: the parser publishes each
	 * talker's group separately (satellites_length == number_of_svs for
	 * that group). So update only the buckets this burst reported —
	 * replacing all five wholesale wipes the constellations that arrived
	 * in the previous burst, which reads as G0 next to a healthy fix. */
	k_mutex_lock(&gps_mutex, K_FOREVER);
	for (int i = 0; i < 5; i++) {
		if (seen[i]) {
			sat_count[i] = tally[i];
			sat_seen_ms[i] = k_uptime_get();
		}
	}
	k_mutex_unlock(&gps_mutex);
}

GNSS_SATELLITES_CALLBACK_DEFINE(NULL, gnss_satellites_cb);

void gps_sat_tally(uint8_t out[5])
{
	/* Age out constellations that have stopped reporting, so a stale count
	 * is never presented as current. */
	k_mutex_lock(&gps_mutex, K_FOREVER);
	int64_t now_ms = k_uptime_get();
	for (int i = 0; i < 5; i++) {
		out[i] = ((now_ms - sat_seen_ms[i]) > SAT_TALLY_STALE_MS) ? 0 : sat_count[i];
	}
	k_mutex_unlock(&gps_mutex);
}
#endif /* CONFIG_ZEPHCORE_GPS_SAT_DIAG */

/* Find and initialize GNSS device */
const struct device *gnss_dev = NULL;

/* The SoC light-sleep lock, held only while ACQUIRING: the GNSS UART is not a
 * wake source, so a sleeping SoC would drop NMEA mid-stream, but a GPS in
 * standby (48 h on a repeater) must not keep the SoC awake. Tracked, so the
 * get/put stay 1:1 whichever path ends the window. Nothing without CONFIG_PM. */
static bool gps_sleep_locked;

static void gps_hold_sleep_lock(bool hold)
{
	if (hold == gps_sleep_locked) {
		return;
	}
	gps_sleep_locked = hold;
	if (hold) {
		zc_pm_block_sleep();
	} else {
		zc_pm_unblock_sleep();
	}
}

/* Acquire-window timeout (ms) for the current phase.
 * - Repeater: fixed 5-min time-sync window.
 * - First acquisition after enable (cold start, no fix yet): a generous but
 *   bounded window so almanac download has time, without pinning the module
 *   on forever when there's no sky. Spent once (first_acquire_used set on the
 *   first standby), after which the node uses the normal duty cycle regardless
 *   of whether a fix was obtained.
 * - All later windows: the normal (warm) acquire timeout. */
static uint32_t gps_acquire_window_ms(void)
{
	if (gps_repeater_mode) {
		return GPS_REPEATER_SYNC_TIMEOUT_MS;
	}
	if (!first_fix_acquired && !first_acquire_used) {
		return gps_first_fix_timeout_ms;
	}
	return gps_acquire_timeout_ms;
}

/* Go to standby and schedule next wake.
 * GPIO power control only — keep VRTC for warm start on T1000-E,
 * FORCE_ON de-asserted for L76K hardware standby. */
static void gps_go_to_standby(void)
{
	/* Unified standby interval for both roles — set from prefs.gps_interval
	 * at boot (companion default 300s, repeater default 48h). Always-on
	 * (interval 0) never reaches here. */
	uint64_t wake_interval = gps_wake_interval_ms;

	LOG_INF("GPS: Going to standby for %llu s%s",
		(unsigned long long)(wake_interval / 1000),
		gps_repeater_mode ? " (repeater time sync)" : "");
	gps_current_state = GPS_STATE_STANDBY;
	consecutive_good_fixes = 0;
	/* The one-time long cold-start window (if any) is now spent — later
	 * wakes use the normal (warm) acquire timeout via gps_acquire_window_ms(). */
	first_acquire_used = true;

	/* Record standby timing for UI (next-wake calculation) */
	standby_start_ms = k_uptime_get();
	standby_interval_ms = wake_interval;

	/* Power down the GPS module.
	 * GPIO boards: hardware power-off (keep VRTC for warm start on T1000-E).
	 * Regulator boards: cut the main rail entirely (both roles). The AXP2101
	 *   VBACKUP charger keeps the receiver's V_BCKP domain alive, so ephemeris/
	 *   RTC survive the cut and re-acquisition is a warm/hot start, not cold.
	 * Other non-GPIO boards: software sleep via UART commands (PMTK + UBX). */
	gps_module_power(false);

	/* Module is off/asleep — release the UART until the next wake
	 * (nRF: drops the HFCLK request held by the armed RX). */
	gps_uart_set_power(false);
	gps_hold_sleep_lock(false);

	/* NOTE: gnss_configured stays true — L76K retains PCAS settings in
	 * flash across power cycles. Re-running gps_module_configure() after GPIO
	 * wake would call modem_chat_run_script() before the chip has booted,
	 * risking a deadlock (modem_chat blocks on system work queue). */

	/* Schedule next wake (event-driven, no polling!) */
	k_work_schedule(&gps_wake_work, K_MSEC(wake_interval));
}

/* Wake GPS and start acquiring.
 * GPIO boards: hardware power-on.
 * Non-GPIO boards: UART wake byte (wakes L76K from standby, ZOE-M8Q from backup).
 * Does NOT call gps_module_configure() — constellation/fix-rate settings persist
 * in L76K flash across power cycles. Calling modem_chat_run_script() here
 * would deadlock: the chip needs ~300ms to boot after GPIO power restore,
 * but modem_chat blocks the calling thread waiting for the system work
 * queue which may be processing stale UART data. */
static void gps_start_acquiring(void)
{
	LOG_INF("GPS: Waking for %s", gps_repeater_mode ? "time sync" : "position fix");
	gps_current_state = GPS_STATE_ACQUIRING;
	consecutive_good_fixes = 0;
	gnss_activity_seen_this_cycle = false;
	gps_hold_sleep_lock(true);

	/* Bring the UART back before the module powers up / the wake byte
	 * goes out, so the first NMEA sentences aren't lost. */
	gps_uart_set_power(true);
	gps_module_power(true);

	/* Schedule the standby timeout — unless always-on (interval 0), where the
	 * GPS stays in continuous acquisition and never sleeps. Every duty window
	 * is bounded (the first cold-start window is just longer; see
	 * gps_acquire_window_ms). */
	if (gps_duty_cycling()) {
		uint32_t timeout_ms = gps_acquire_window_ms();
		LOG_INF("GPS: Acquire window %u s", timeout_ms / 1000U);
		k_work_schedule(&gps_timeout_work, K_MSEC(timeout_ms));
	} else {
		LOG_INF("GPS: Always-on (continuous, no standby)");
	}
}

/* Work handler: wake GPS for fix.
 * Runs on system work queue — MUST NOT call blocking GNSS APIs directly!
 * modem_chat_run_script() blocks on a semaphore signaled from this same
 * work queue, causing a deadlock. Instead, set a flag and signal the
 * main thread to do the actual wake via gps_process_event(). */
static void gps_wake_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!gps_enabled || gps_current_state != GPS_STATE_STANDBY) {
		return;
	}

	atomic_or(&pending_gps_actions, GPS_ACTION_WAKE);
	if (gps_event_cb) {
		gps_event_cb();
	}
}

/* Work handler: timeout waiting for fix.
 * Runs on system work queue — MUST NOT call blocking GNSS APIs directly!
 * Same deadlock risk as gps_wake_work_fn. Defer to main thread. */
static void gps_timeout_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (gps_current_state != GPS_STATE_ACQUIRING) {
		return;
	}

	LOG_WRN("GPS: Timeout after %d/%d fixes, deferring standby to main thread",
		consecutive_good_fixes, GPS_GOOD_FIX_COUNT);

	if (gps_repeater_mode && !gnss_activity_seen_this_cycle) {
		LOG_WRN("GPS: Repeater acquire window had no GNSS callbacks; retrying on next cycle");
	}

	atomic_or(&pending_gps_actions, GPS_ACTION_TIMEOUT);
	if (gps_event_cb) {
		gps_event_cb();
	}
}

/* ========== GNSS Init ========== */

static int gnss_init(void)
{
	/* Try to find a GNSS device - prefer chip-specific drivers
	 * (they support constellation config, fix rate, etc.) over
	 * the generic NMEA parser which is passive only.
	 * Power control is done lazily in gps_enable(). */
#if DT_HAS_COMPAT_STATUS_OKAY(quectel_lc76g)
	gnss_dev = DEVICE_DT_GET_ANY(quectel_lc76g);
#elif DT_HAS_COMPAT_STATUS_OKAY(luatos_air530z)
	gnss_dev = DEVICE_DT_GET_ANY(luatos_air530z);
#elif DT_HAS_COMPAT_STATUS_OKAY(gnss_nmea_generic)
	gnss_dev = DEVICE_DT_GET_ANY(gnss_nmea_generic);
#endif

	if (gnss_dev == NULL) {
		LOG_WRN("No GNSS device found in device tree");
		return -ENODEV;
	}

	if (!device_is_ready(gnss_dev)) {
		/* Root cause: GPS transmits NMEA immediately at power-up before
		 * modem_chat opens its DMA pipe. UARTE accumulates overrun/framing
		 * errors, causing modem_pipe_open() to fail and device_init to return
		 * an error.
		 *
		 * Strategy: use UARTE ERRORSRC as a real signal. Wait until errors
		 * appear (GPS is transmitting), clear them, then call device_init.
		 * This avoids arbitrary delays — we act when the hardware tells us
		 * conditions are ready, not after a fixed sleep.
		 *
		 * IMPORTANT: Do NOT use uart_poll_in() — it corrupts nRF52840 UARTE
		 * DMA state and breaks modem_pipe async receive. */
		LOG_INF("GNSS device not ready — waiting for GPS activity on UART");
		gps_power_control(true);

#if HAS_GPS_POWER_CONTROL
		gps_dump_gpio_states();
#endif

#ifdef GPS_NRF_UARTE
		NRF_UARTE_Type *uart = GPS_NRF_UARTE;

		/* Wait up to 2s for UARTE errors — their presence means the GPS
		 * module is alive and transmitting (ERRORSRC gets set because no
		 * DMA buffer is configured yet). */
		bool gps_active = false;
		for (int t = 0; t < 200; t++) {
			if (uart->ERRORSRC != 0) {
				LOG_INF("GPS UART activity detected after ~%dms "
					"(ERRORSRC=0x%x)", t * 10, uart->ERRORSRC);
				gps_active = true;
				break;
			}
			k_msleep(10);
		}
		if (!gps_active) {
			LOG_WRN("No GPS UART activity within 2s — module may not be "
				"powered or transmitting");
		}
#else
		/* Non-nRF52840: no direct UARTE register access, fall back to
		 * a brief fixed wait for the GPS to start transmitting. */
		k_msleep(500);
#endif

		bool init_ok = false;
		for (int attempt = 0; attempt < 3 && !init_ok; attempt++) {
			/* Clear accumulated UART errors before opening the modem pipe */
			gps_uart_dump_hw_state();

			int ret = device_init(gnss_dev);
			if (ret != 0 && ret != -EALREADY) {
				LOG_WRN("GNSS device_init attempt %d failed: %d",
					attempt + 1, ret);
				/* Small wait for UART to settle, then retry */
				k_msleep(100);
				continue;
			}

			/* Poll for readiness — modem_chat needs a brief moment to
			 * complete pipe setup after device_init returns. */
			for (int t = 0; t < 50; t++) {
				if (device_is_ready(gnss_dev)) {
					LOG_INF("GNSS ready after ~%dms (attempt %d)",
						t * 10, attempt + 1);
					init_ok = true;
					break;
				}
				k_msleep(10);
			}

			if (!init_ok) {
				LOG_WRN("GNSS not ready after attempt %d", attempt + 1);
			}
		}

		if (!init_ok) {
			LOG_ERR("GNSS device failed to initialize");
			gps_uart_dump_hw_state();
			return -ENODEV;
		}
	}

#ifdef CONFIG_PM_DEVICE
	/* Some upstream GNSS drivers (gnss-nmea-generic) start suspended under
	 * CONFIG_PM_DEVICE and never open their modem pipe until resumed — no
	 * NMEA would ever flow (the old "PM broke GPS" trap). Resume once
	 * here: main thread at boot, the one safe context for the modem_chat
	 * scripts a resume may run. PM-less drivers (luatos,air530z) return
	 * -ENOSYS. Retried like device_init above — opening the pipe while
	 * the module is mid-sentence can fail transiently. */
	int pm_ret = pm_device_action_run(gnss_dev, PM_DEVICE_ACTION_RESUME);
	for (int attempt = 1; pm_ret != 0 && pm_ret != -EALREADY &&
	     pm_ret != -ENOSYS && attempt < 3; attempt++) {
		LOG_WRN("GNSS PM resume failed (%d), retrying", pm_ret);
		k_msleep(100);
		pm_ret = pm_device_action_run(gnss_dev, PM_DEVICE_ACTION_RESUME);
	}
	if (pm_ret != 0 && pm_ret != -EALREADY && pm_ret != -ENOSYS) {
		LOG_ERR("GNSS PM resume failed: %d", pm_ret);
		return -ENODEV;
	}
#endif

	LOG_INF("GNSS device %s initialized", gnss_dev->name);
	gps_available = true;
	return 0;
}
#endif /* HAS_GNSS */

/* ========== Public API ========== */

int gps_manager_init(void)
{
#if HAS_GNSS
	/* Nothing to configure on a module that did not come up: the GNSS API
	 * would drive modem_chat on a failed device. */
	if (gnss_init() != 0) {
		return 0;
	}

	/* Configure constellations + fix rate NOW while chip is powered
	 * and the modem pipe is open (driver init already ran).
	 * This is the ONLY safe place to call modem_chat_run_script() —
	 * after power cycles the chip needs ~300ms boot time and calling
	 * modem_chat from the main thread risks deadlock. L76K retains
	 * PCAS settings in flash, so one-time config at boot is enough. */
	gps_module_configure();
#endif
	return 0;
}

void gps_park(void)
{
#if HAS_GNSS
	if (gnss_init() == 0) {
		gps_ensure_power_state(false);
	}
#endif
}

bool gps_is_available(void)
{
#if HAS_GNSS
	return gps_available;
#else
	return false;
#endif
}

bool gps_is_enabled(void)
{
#if HAS_GNSS
	return gps_enabled;
#else
	return false;
#endif
}

void gps_ensure_power_state(bool should_be_enabled)
{
#if HAS_GNSS
	if (!gps_available) {
		return;
	}

	/* At boot, GPS hardware is powered (bootloader/pull-up).
	 * If it should be disabled, explicitly power it off now. */
	if (!should_be_enabled) {
		LOG_INF("GPS: Powering off at boot (disabled in prefs)");
		gps_module_power(false, false);
		/* GPS stays off — release the UART too. Without this, the RX
		 * armed at driver init would hold HFCLK for the entire uptime
		 * of every GPS-disabled node. */
		gps_uart_set_power(false);
		gps_current_state = GPS_STATE_OFF;
	}
#else
	ARG_UNUSED(should_be_enabled);
#endif
}

void gps_set_repeater_mode(bool repeater)
{
#if HAS_GNSS
	if (!gps_available) {
		return;
	}

	/* Only the mode: the acquire window becomes the time-sync one. Whether
	 * the GPS runs is prefs.gps_enabled, applied through gps_enable(). */
	gps_repeater_mode = repeater;
	LOG_INF("GPS: %s mode", repeater ? "Time-sync (server)" : "Companion");
#else
	ARG_UNUSED(repeater);
#endif
}

void gps_enable(bool enable)
{
#if HAS_GNSS
	if (!gps_available) {
		LOG_WRN("GPS not available");
		return;
	}

	if (enable == gps_enabled) {
		return;
	}

	gps_enabled = enable;

	if (enable) {
		LOG_INF("GPS enabled - starting acquisition");

		/* The same wake as the duty cycle's: UART, module power, sleep
		 * lock, and the first (longer) acquire window unless always-on. */
		gps_start_acquiring();

		/* gps_module_configure() runs once at boot (see gps_manager_init path).
		 * L76K retains PCAS settings in flash across power cycles.
		 * Do NOT call modem_chat_run_script() here — the chip needs
		 * ~300ms to boot after GPIO power restore and calling it
		 * immediately deadlocks the main thread. */
		gps_diag_maybe_reconfigure();
	} else {
		LOG_INF("GPS disabled - canceling timers and powering off");

		/* Cancel any pending work */
		k_work_cancel_delayable(&gps_wake_work);
		k_work_cancel_delayable(&gps_timeout_work);

		/* Power off GPS — warm standby if VRTC available (Arduino sleep_gps),
		 * full power off otherwise. Warm standby preserves ephemeris/RTC
		 * in AG3335 backup RAM for fast re-acquisition (1-8s vs 15-45s).
		 * Boards with no power line get the UART sleep commands, as in
		 * the duty cycle's standby. */
		gps_module_power(false);

		/* GPS is off until re-enabled — release the UART. */
		gps_uart_set_power(false);
		gps_hold_sleep_lock(false);

		gps_current_state = GPS_STATE_OFF;
		consecutive_good_fixes = 0;

		/* Zero the stale satellite count so a re-enable doesn't briefly
		 * report a live fix (e.g. joystick UI showing "3D FIX") off old
		 * data before any new NMEA sentence arrives. lat/lon/valid are
		 * deliberately left alone — telemetry/UI "last known position"
		 * reads (gps_get_position) intentionally survive an on/off
		 * toggle; only the live fix-quality indicator resets. */
		k_mutex_lock(&gps_mutex, K_FOREVER);
		current_pos.satellites = 0;
		k_mutex_unlock(&gps_mutex);

		/* Clear first-fix flags so the next enable gets a fresh long
		 * first-acquisition window again — the user explicitly toggled GPS
		 * expecting it to try hard for a fix. */
		first_fix_acquired = false;
		first_acquire_used = false;

		/* Clear time sync flag - time will drift, allow phone sync again */
		gps_time_synced = false;
	}

	/* Notify callback (for persistence in main.cpp) */
	if (gps_enable_cb) {
		gps_enable_cb(enable);
	}
#else
	ARG_UNUSED(enable);
#endif
}

void gps_get_position(struct gps_position *pos)
{
#if HAS_GNSS
	k_mutex_lock(&gps_mutex, K_FOREVER);
	*pos = current_pos;
	k_mutex_unlock(&gps_mutex);
#else
	memset(pos, 0, sizeof(*pos));
#endif
}

uint32_t gps_get_poll_interval_sec(void)
{
#if HAS_GNSS
	return gps_wake_interval_ms / 1000U;
#else
	return CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC;
#endif
}

void gps_set_poll_interval_sec(uint32_t interval)
{
#if HAS_GNSS
	interval = clampGpsInterval(interval);  /* 0 = always-on */
	gps_wake_interval_ms = interval * 1000U;
	LOG_INF("GPS poll interval set to %u seconds%s", interval,
		interval == 0 ? " (always on)" : "");

	/* Live re-arm so a runtime change takes effect without a reboot. */
	if (!gps_enabled) {
		return;
	}
	if (gps_current_state == GPS_STATE_ACQUIRING) {
		if (interval == 0) {
			/* Switch to always-on: drop the standby timeout so it won't sleep. */
			k_work_cancel_delayable(&gps_timeout_work);
		} else if (!k_work_delayable_is_pending(&gps_timeout_work)) {
			/* Was always-on: arm a timeout so it starts duty cycling. */
			k_work_reschedule(&gps_timeout_work, K_MSEC(gps_acquire_window_ms()));
		}
	} else if (gps_current_state == GPS_STATE_STANDBY) {
		k_work_cancel_delayable(&gps_wake_work);
		if (interval == 0) {
			/* Wake now and stay on. */
			atomic_or(&pending_gps_actions, GPS_ACTION_WAKE);
			if (gps_event_cb) {
				gps_event_cb();
			}
		} else {
			/* The wake is re-armed from now, so the countdown restarts too. */
			standby_start_ms = k_uptime_get();
			standby_interval_ms = gps_wake_interval_ms;
			k_work_reschedule(&gps_wake_work, K_MSEC(gps_wake_interval_ms));
		}
	}
#else
	ARG_UNUSED(interval);
#endif
}

int64_t gps_get_utc_time(void)
{
#if HAS_GNSS
	k_mutex_lock(&gps_mutex, K_FOREVER);
	bool valid = current_pos.valid;
	struct gnss_time t = current_utc;
	k_mutex_unlock(&gps_mutex);

	return valid ? gnss_time_to_unix(&t) : 0;
#else
	return 0;
#endif
}

bool gps_has_time_sync(void)
{
#if HAS_GNSS
	/* Returns true if GPS has recently synced the RTC.
	 * Expires after 2 hours without a fix so the phone can re-sync
	 * (e.g. node moved indoors, GPS lost sky, RTC drifting). */
	if (!gps_time_synced) {
		return false;
	}
	int64_t age_ms = k_uptime_get() - last_fix_uptime_ms;

	if (age_ms > (2 * 60 * 60 * 1000LL)) {
		gps_time_synced = false;
		return false;
	}
	return true;
#else
	return false;
#endif
}

bool gps_get_last_known_position(struct gps_position *pos)
{
#if HAS_GNSS
	k_mutex_lock(&gps_mutex, K_FOREVER);
	if (current_pos.valid) {
		*pos = current_pos;
		k_mutex_unlock(&gps_mutex);
		return true;
	}
	k_mutex_unlock(&gps_mutex);
#endif
	memset(pos, 0, sizeof(*pos));
	return false;
}

void gps_request_fresh_fix(void)
{
#if HAS_GNSS
	if (!gps_available || !gps_enabled) {
		return;
	}

	if (gps_current_state == GPS_STATE_STANDBY) {
		LOG_INF("GPS: Fresh fix requested, waking early");
		/* Cancel scheduled wake and wake immediately */
		k_work_cancel_delayable(&gps_wake_work);
		gps_start_acquiring();
	} else if (gps_current_state == GPS_STATE_ACQUIRING) {
		/* Already acquiring — reschedule the timeout so the caller's
		 * fresh-fix request gets a full window from now. Otherwise, a
		 * telemetry request that arrives 25s into a 30s acquire window
		 * only has 5s left, which in marginal signal usually means the
		 * chip goes to standby before producing a fix the requester
		 * could use. Each duty phase has a bounded window
		 * (gps_acquire_window_ms); in always-on there's no timeout to extend
		 * (GPS is continuously acquiring and current_pos is always fresh). */
		if (gps_duty_cycling()) {
			uint32_t timeout_ms = gps_acquire_window_ms();
			LOG_INF("GPS: Fresh fix requested, extending acquire timeout to %u s",
				timeout_ms / 1000U);
			k_work_reschedule(&gps_timeout_work, K_MSEC(timeout_ms));
		}
	}
#endif
}

void gps_get_state_info(struct gps_state_info *info)
{
	memset(info, 0, sizeof(*info));
#if HAS_GNSS
	info->state = (uint8_t)gps_current_state;
	info->satellites = current_pos.satellites;

	if (last_fix_uptime_ms > 0) {
		/* Seconds since last validated fix */
		info->last_fix_age_s = (uint32_t)((k_uptime_get() - last_fix_uptime_ms) / 1000);
	} else {
		info->last_fix_age_s = UINT32_MAX;  /* No fix yet */
	}

	if (gps_current_state == GPS_STATE_STANDBY && standby_interval_ms > 0) {
		int64_t wake_at = standby_start_ms + (int64_t)standby_interval_ms;
		int64_t remaining = wake_at - k_uptime_get();

		info->next_search_s = (remaining > 0) ? (uint32_t)(remaining / 1000) : 0;
	} else if (gps_current_state == GPS_STATE_ACQUIRING) {
		info->next_search_s = 0;  /* Searching right now */
	}
#endif
}

/* Process pending GPS state transitions — called from main thread.
 * Work handlers on the system work queue set flags + signal the main
 * thread via gps_event_cb(). The main thread then calls this function,
 * which safely executes blocking GNSS configuration (modem_chat_run_script
 * blocks on a semaphore signaled from the system work queue — calling it
 * FROM the work queue deadlocks). */
void gps_process_event(void)
{
#if HAS_GNSS
	uint32_t actions = (uint32_t)atomic_clear(&pending_gps_actions);

	if (actions == 0) {
		return;
	}

	/* A validated fix: the clock and position, here on the main thread. */
	if ((actions & GPS_ACTION_FIX) && gps_fix_cb) {
		k_mutex_lock(&gps_mutex, K_FOREVER);
		int64_t lat_ndeg = fix_pending.lat_ndeg;
		int64_t lon_ndeg = fix_pending.lon_ndeg;
		struct gnss_time utc = fix_pending.utc;
		int64_t at_ms = fix_pending.at_ms;
		k_mutex_unlock(&gps_mutex);

		int64_t utc_time = gnss_time_to_unix(&utc);
		if (utc_time > 0) {
			utc_time += (k_uptime_get() - at_ms) / 1000;  /* time spent queued */
		}
		gps_fix_cb((double)lat_ndeg / 1000000000.0, (double)lon_ndeg / 1000000000.0,
			   utc_time);
	}

	/* Wake takes priority — if both wake and timeout/fix-done are pending
	 * (shouldn't happen, but be safe), wake wins. */
	if (actions & GPS_ACTION_WAKE) {
		if (gps_enabled && gps_current_state == GPS_STATE_STANDBY) {
			gps_start_acquiring();
		}
	} else if (actions & (GPS_ACTION_TIMEOUT | GPS_ACTION_FIX_DONE)) {
		if (gps_current_state == GPS_STATE_ACQUIRING) {
			gps_go_to_standby();
		}
	}
#endif
}
