/*
 * SPDX-License-Identifier: Apache-2.0
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
 * - Full power-off only on user-disable or System OFF
 */

#include "ZephyrGPSManager.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/fs/fs.h>
#include <string.h>
#if defined(CONFIG_SOC_NRF52840)
#include <nrfx.h>
#endif

LOG_MODULE_REGISTER(zephcore_gps, CONFIG_ZEPHCORE_GPS_LOG_LEVEL);

/* ========== GNSS Support ========== */
#if DT_HAS_COMPAT_STATUS_OKAY(gnss_nmea_generic) || \
    DT_HAS_COMPAT_STATUS_OKAY(u_blox_m8) || \
    DT_HAS_COMPAT_STATUS_OKAY(u_blox_f9p) || \
    DT_HAS_COMPAT_STATUS_OKAY(quectel_lcx6g) || \
    DT_HAS_COMPAT_STATUS_OKAY(quectel_lc76g) || \
    DT_HAS_COMPAT_STATUS_OKAY(luatos_air530z)
#define HAS_GNSS 1
#include <zephyr/drivers/gnss.h>
#else
#define HAS_GNSS 0
#endif

/* ========== GPS Module Capability Flags ==========
 * GPS power management uses GPIO only (no PM_DEVICE):
 * - Wio Tracker L1 (L76K): FORCE_ON pin LOW = hardware standby (~360µA,
 *   Vcc stays on, ephemeris/almanac/RTC preserved, hot-start 1-2s)
 * - T1000-E (AG3335): GPS_EN LOW + VRTC HIGH = warm standby (ephemeris
 *   preserved via backup RAM, ~1-2µA VRTC current)
 * - All boards: gps-enable alias → GPIO power control
 *
 * PM_DEVICE is intentionally NOT used — the system-managed PM subsystem
 * auto-suspends devices during idle, calling modem_chat_run_script() from
 * an unexpected context which can deadlock the system. */

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
static atomic_t pending_gps_actions;

/* GPS Power Management State Machine */
enum gps_state {
	GPS_STATE_OFF,          /* GPS disabled by user */
	GPS_STATE_STANDBY,      /* GPS enabled but sleeping (5 min cycle) */
	GPS_STATE_ACQUIRING,    /* GPS awake, waiting for fixes */
};

static enum gps_state gps_current_state = GPS_STATE_OFF;
static uint8_t consecutive_good_fixes = 0;
static bool first_fix_acquired = false;  /* True after first 3-good-fix cycle since enable. Cleared on gps_enable(false) and at boot. */
static bool gps_time_synced = false;     /* True after GPS syncs RTC. Starts false at boot (RTC reset),
                                          * set true after 3 good fixes, cleared when GPS disabled. */
static int64_t last_fix_uptime_ms = 0;  /* k_uptime when last validated fix was acquired */
static int64_t standby_start_ms = 0;    /* k_uptime when standby started (for next-wake calc) */
static uint64_t standby_interval_ms = 0; /* How long standby lasts (for next-wake calc) */

#define GPS_GOOD_FIX_COUNT       3       /* Need 3 consecutive good fixes */
#define GPS_MIN_SATELLITES       4       /* Minimum satellites for valid fix */

/* Runtime-configurable intervals, initialized from Kconfig defaults */
static uint32_t gps_acquire_timeout_ms = CONFIG_ZEPHCORE_GPS_FIX_TIMEOUT_SEC * 1000U;
static uint32_t gps_wake_interval_ms   = CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC * 1000U;

/* Repeater mode intervals - GPS only for time sync */
#define GPS_REPEATER_SYNC_INTERVAL_MS  (48ULL * 60 * 60 * 1000)  /* 48 hours */
#define GPS_REPEATER_SYNC_TIMEOUT_MS   (5 * 60 * 1000)           /* 5 minutes */

static bool gps_repeater_mode = false;  /* True = repeater (time sync only), False = companion */
static bool gnss_activity_seen_this_cycle = false;  /* Runtime-only: set by GNSS callback while acquiring */
static bool gps_runtime_no_gnss_activity = false;   /* Runtime-only: repeater disables periodic GPS until reboot */

/* Forward declarations for work handlers and state functions */
static void gps_wake_work_fn(struct k_work *work);
static void gps_timeout_work_fn(struct k_work *work);
static void gps_go_to_standby(void);
static void gps_start_acquiring(void);

/* Delayable work for event-driven timers (no polling!) */
static K_WORK_DELAYABLE_DEFINE(gps_wake_work, gps_wake_work_fn);
static K_WORK_DELAYABLE_DEFINE(gps_timeout_work, gps_timeout_work_fn);

/* ========== Last-known position persistence ========== */
#define GPS_POS_FILE "/lfs/gps_pos"

/* On-disk format: lat(8) + lon(8) + alt(4) = 20 bytes */
struct gps_pos_record {
	int64_t latitude_ndeg;
	int64_t longitude_ndeg;
	int32_t altitude_mm;
};

static void gps_save_position(const struct gps_position *pos)
{
	struct fs_file_t file;
	struct gps_pos_record rec = {
		.latitude_ndeg = pos->latitude_ndeg,
		.longitude_ndeg = pos->longitude_ndeg,
		.altitude_mm = pos->altitude_mm,
	};

	fs_file_t_init(&file);
	if (fs_open(&file, GPS_POS_FILE, FS_O_CREATE | FS_O_WRITE) == 0) {
		fs_write(&file, &rec, sizeof(rec));
		fs_close(&file);
	}
}

static bool gps_load_position(void)
{
	struct fs_file_t file;
	struct gps_pos_record rec;

	fs_file_t_init(&file);
	if (fs_open(&file, GPS_POS_FILE, FS_O_READ) < 0) {
		return false;
	}
	ssize_t n = fs_read(&file, &rec, sizeof(rec));
	fs_close(&file);

	if (n != sizeof(rec)) {
		return false;
	}

	current_pos.latitude_ndeg = rec.latitude_ndeg;
	current_pos.longitude_ndeg = rec.longitude_ndeg;
	current_pos.altitude_mm = rec.altitude_mm;
	current_pos.valid = true;
	current_pos.satellites = 0;
	current_pos.timestamp_ms = 0;  /* unknown — loaded from flash */
	LOG_INF("GPS: Restored last position from flash lat=%lld lon=%lld",
		rec.latitude_ndeg / 1000000, rec.longitude_ndeg / 1000000);
	return true;
}

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

/* GNSS callback - called when new fix data is available */
static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	ARG_UNUSED(dev);

	if (!gps_enabled || gps_current_state == GPS_STATE_STANDBY) {
		/* GPS disabled or in standby — ignore NMEA data.
		 * The GNSS driver fires callbacks as long as the UART has data,
		 * even after we drive GPS_EN LOW (module drains its buffer).
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
		 * a real fix — skip so we don't poison current_pos, persist
		 * zeros to flash, or promote consecutive_good_fixes. */
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
					LOG_INF("GPS: Got %d good fixes, updating location/time",
						GPS_GOOD_FIX_COUNT);

					/* Mark first fix acquired (enables timeout for future cycles) */
					first_fix_acquired = true;

					/* Mark time as synced from GPS - blocks phone time sync */
					gps_time_synced = true;
					last_fix_uptime_ms = k_uptime_get();

					/* Persist position to flash for reboot survival */
					gps_save_position(&current_pos);

					/* Cancel timeout */
					k_work_cancel_delayable(&gps_timeout_work);

					/* Notify fix callback with validated position */
					if (gps_fix_cb) {
						double lat = (double)data->nav_data.latitude / 1000000000.0;
						double lon = (double)data->nav_data.longitude / 1000000000.0;
						k_mutex_unlock(&gps_mutex);
						gps_fix_cb(lat, lon, gps_get_utc_time());
					} else {
						k_mutex_unlock(&gps_mutex);
					}

					/* Defer standby to main thread — we're on the system
					 * workqueue here (GNSS callback), can't call PM suspend
					 * (modem_chat_run_script deadlocks on same workqueue). */
					atomic_or(&pending_gps_actions, GPS_ACTION_FIX_DONE);
					if (gps_event_cb) {
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

/* Find and initialize GNSS device */
static const struct device *gnss_dev = NULL;

/* Multi-constellation configuration — runs ONCE at boot.
 * modem_chat_run_script() blocks on a semaphore signaled from the system
 * work queue. Calling it after a GPIO power cycle can deadlock because:
 * 1. The L76K needs ~300ms to boot after power restore
 * 2. Meanwhile the modem_chat may be processing stale UART data
 * 3. The script completion callback competes with NMEA processing
 *
 * Safe to call at boot because the driver init already ran and the chip
 * is powered and outputting NMEA. After power cycles, the L76K retains
 * constellation + fix rate settings in internal flash (PCAS commands
 * persist). So we only need to configure once. */
static bool gnss_configured = false;

/* ========== Vendor-Specific Configuration Commands ==========
 *
 * The RAK WisBlock GPS slot accepts multiple modules (L76K, ZOE-M8Q, etc.)
 * and we use gnss-nmea-generic which is a passive NMEA listener — it has no
 * GNSS API for configuration.
 *
 * Strategy: send BOTH Quectel PMTK and u-blox UBX configuration commands.
 * Each module ignores the protocol it doesn't understand.
 *
 * This runs once at boot. Both modules persist config to internal flash,
 * so these are effectively no-ops on subsequent boots. */

#if HAS_GPS_UART

/* --- Quectel L76K (PMTK) configuration --- */

/* PMTK353: Enable GPS + GLONASS + Galileo + BeiDou (no QZSS).
 * Default is GPS-only. Multi-constellation dramatically improves TTFF
 * and fix reliability, especially indoors or with limited sky view. */
static const char pmtk_constellations[] = "$PMTK353,1,1,1,1,0*2B\r\n";

/* PMTK869: Enable EASY (Embedded Assist System).
 * Caches predicted satellite ephemeris in the GNSS module's internal flash.
 * Reduces TTFF from 15-45s (cold) to 1-3s (warm) for up to 3 days after
 * last fix. Setting persists in flash — resending is a harmless no-op. */
static const char pmtk_easy[] = "$PMTK869,1,1*35\r\n";

/* PMTK286: Enable AIC (Active Interference Cancellation).
 * Filters out narrowband jammers (e.g. harmonics from nearby electronics,
 * LoRa radio leakage). Improves sensitivity by ~2dB in noisy environments.
 * Especially useful when GPS antenna is near the SX1262 + SKY66122 PA. */
static const char pmtk_aic[] = "$PMTK286,1*23\r\n";

/* --- u-blox ZOE-M8Q (UBX binary) configuration --- */

/* UBX-CFG-GNSS: Enable GPS + Galileo + BeiDou + GLONASS.
 * ZOE-M8Q defaults to GPS-only. Multi-constellation dramatically improves
 * TTFF and fix reliability — more visible satellites in any sky condition.
 * 32 tracking channels allocated across 4 active systems.
 * SBAS disabled — needs 30-60s to download corrections, useless for our
 *   quick-fix-then-sleep pattern (companions: 30s, repeaters: 5min).
 * QZSS disabled — Japan regional, wastes tracking channels elsewhere. */
static const uint8_t ubx_cfg_gnss[] = {
	0xB5, 0x62, 0x06, 0x3E, 0x27, 0x00, 0x00, 0x20, 0x20, 0x05, 0x00, 0x08,
	0x10, 0x01, 0x00, 0x01, 0x00, 0x02, 0x04, 0x0A, 0x01, 0x00, 0x01, 0x00,
	0x03, 0x04, 0x0A, 0x01, 0x00, 0x01, 0x00, 0x05, 0x00, 0x03, 0x00, 0x00,
	0x01, 0x00, 0x06, 0x04, 0x0A, 0x01, 0x00, 0x01, 0x00, 0x0E, 0x13
};

/* UBX-CFG-NAV5: Set 5° minimum satellite elevation.
 * Ignore satellites below 5° elevation — they have more atmospheric
 * noise and multipath, degrading fix quality. The default 0° lets in
 * everything including horizon-level junk.
 * Dynamic model left at factory default (Portable) — works for fixed
 * repeaters, walking companions, and vehicles alike.
 * apply mask 0x0002 = minEl(bit1) only */
static const uint8_t ubx_cfg_nav5_minelev[] = {
	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x02, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x37
};

/* UBX-CFG-NAVX5: Enable AssistNow Autonomous (AOP).
 * u-blox equivalent of Quectel EASY — the receiver autonomously predicts
 * satellite orbits from previously downloaded ephemeris data. Predictions
 * stay valid for 3-6 days, reducing TTFF from 26-30s (cold) to 2-5s.
 * No server connection needed — runs entirely on-chip.
 * mask1 bit 14 = aop, aopCfg bit 0 = enable. */
static const uint8_t ubx_cfg_navx5_aop[] = {
	0xB5, 0x62, 0x06, 0x23, 0x28, 0x00, 0x04, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x66
};

/* UBX-CFG-CFG: Save all configuration to BBR + Flash + EEPROM.
 * Persists constellation, nav model, SBAS settings across power cycles
 * and backup mode. Without this, ZOE-M8Q reverts to factory defaults
 * after a full power loss (though BBR survives backup mode). */
static const uint8_t ubx_cfg_save[] = {
	0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF
};

/* Send a PMTK command string (including \r\n). Adds small delay after. */
static void gps_send_pmtk(const char *cmd)
{
	gps_uart_send((const uint8_t *)cmd, strlen(cmd));
	k_msleep(20);  /* Let module process before next command */
}

/* Send a UBX binary frame. Adds small delay after for processing. */
static void gps_send_ubx(const uint8_t *frame, size_t len)
{
	gps_uart_send(frame, len);
	k_msleep(50);  /* UBX needs more time to ACK + apply config */
}

/* Configure the GPS module with optimal settings for a mesh repeater.
 * Sends both PMTK (Quectel) and UBX (u-blox) commands — the module that
 * isn't present ignores bytes it doesn't understand. */
static void gps_configure_via_uart(void)
{
	LOG_INF("GPS: Configuring via UART (PMTK + UBX dual-protocol)");

	/* --- Quectel L76K (PMTK) --- */
	gps_send_pmtk(pmtk_constellations);
	gps_send_pmtk(pmtk_easy);
	gps_send_pmtk(pmtk_aic);
	LOG_INF("GPS: PMTK config sent (constellations, EASY, AIC)");

	/* --- u-blox ZOE-M8Q (UBX) --- */
	gps_send_ubx(ubx_cfg_gnss, sizeof(ubx_cfg_gnss));
	gps_send_ubx(ubx_cfg_nav5_minelev, sizeof(ubx_cfg_nav5_minelev));
	gps_send_ubx(ubx_cfg_navx5_aop, sizeof(ubx_cfg_navx5_aop));
	gps_send_ubx(ubx_cfg_save, sizeof(ubx_cfg_save));
	LOG_INF("GPS: UBX config sent (multi-GNSS, 5° min elev, AOP, saved to flash)");
}
#endif /* HAS_GPS_UART */

static void gnss_configure(void)
{
	if (gnss_configured || gnss_dev == NULL) {
		return;
	}

	/* Enable all available constellation systems for faster TTFF.
	 * Try GPS+GLONASS+Galileo+BeiDou first (AG3335 supports all).
	 * Fall back to GPS+GLONASS+BeiDou if Galileo not supported (L76KB). */
	gnss_systems_t systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS |
				 GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU;
	int ret = gnss_set_enabled_systems(gnss_dev, systems);
	if (ret == -EINVAL) {
		/* Some systems not supported — try without Galileo */
		systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_BEIDOU;
		ret = gnss_set_enabled_systems(gnss_dev, systems);
	}
	if (ret == 0) {
		LOG_INF("GPS: Multi-constellation enabled via GNSS API");
	} else if (ret == -ENOSYS || ret == -ENOTSUP) {
#if HAS_GPS_UART
		/* gnss-nmea-generic is a passive listener — no GNSS API.
		 * Configure everything via direct UART commands instead. */
		gps_configure_via_uart();
#else
		LOG_INF("GPS: No GNSS API and no UART access — using module defaults");
#endif
	} else {
		LOG_WRN("GPS: Failed to set constellations: %d", ret);
		/* Will retry on next power-on cycle */
		return;
	}

	/* Set 1Hz fix rate (explicit, don't rely on chip defaults) */
	ret = gnss_set_fix_rate(gnss_dev, 1000);
	if (ret == 0) {
		LOG_INF("GPS: Fix rate set to 1Hz");
	} else if (ret != -ENOSYS && ret != -ENOTSUP) {
		LOG_WRN("GPS: Failed to set fix rate: %d", ret);
	}

	gnss_configured = true;
}

#endif /* HAS_GNSS - GPS power GPIO section is unconditional (needed for shutdown) */

/* ========== GPS Power GPIO Control ==========
 * These are unconditional (not gated by HAS_GNSS) because
 * gps_power_off_for_shutdown() must be available for System OFF
 * even on boards without a GNSS driver.
 *
 * IMPORTANT: Do NOT touch GPIO during init! The GNSS driver needs the GPS
 * to be powered and outputting NMEA for the modem pipe to work.
 * We only configure GPIO lazily on first power-off request.
 *
 * Board-specific pins (defined in board overlays as gps-enable alias):
 * - T1000-E: P1.11 (GPS_EN), P0.8 (GPS_VRTC_EN), P1.15 (GPS_RESET), P1.12 (GPS_SLEEP_INT)
 * - Wio Tracker L1: P1.09 (GPS power, shared with luatos,air530z on-off-gpios)
 */
#if DT_NODE_EXISTS(DT_ALIAS(gps_enable))
static const struct gpio_dt_spec gps_enable_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_enable), gpios);
#define HAS_GPS_POWER_CONTROL 1
#else
#define HAS_GPS_POWER_CONTROL 0
#endif

/* T1000-E specific GPS control pins */
#if DT_NODE_EXISTS(DT_ALIAS(gps_vrtc_enable))
static const struct gpio_dt_spec gps_vrtc_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_vrtc_enable), gpios);
#define HAS_GPS_VRTC 1
#else
#define HAS_GPS_VRTC 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(gps_reset))
static const struct gpio_dt_spec gps_reset_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_reset), gpios);
#define HAS_GPS_RESET 1
#else
#define HAS_GPS_RESET 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(gps_sleep_int))
static const struct gpio_dt_spec gps_sleep_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_sleep_int), gpios);
#define HAS_GPS_SLEEP 1
#else
#define HAS_GPS_SLEEP 0
#endif

/* GPS RTC interrupt pin — held LOW during normal operation */
#if DT_NODE_EXISTS(DT_ALIAS(gps_rtc_int))
static const struct gpio_dt_spec gps_rtcint_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_rtc_int), gpios);
#define HAS_GPS_RTCINT 1
#else
#define HAS_GPS_RTCINT 0
#endif

/* GPS RESETB (active-LOW reset) — must be INPUT_PULLUP for normal operation.
 * Without the pull-up, this pin floats LOW and holds the AG3335 in permanent
 * reset, preventing any UART output. */
#if DT_NODE_EXISTS(DT_ALIAS(gps_resetb))
static const struct gpio_dt_spec gps_resetb_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(gps_resetb), gpios);
#define HAS_GPS_RESETB 1
#else
#define HAS_GPS_RESETB 0
#endif

/* T1000-E has extra GPS control pins that require a specific init sequence */
#define HAS_T1000_GPS_CONTROL (HAS_GPS_VRTC || HAS_GPS_RESET || HAS_GPS_SLEEP)

#if HAS_GPS_POWER_CONTROL
static bool gps_gpio_configured = false;
#endif

/* GPS power control with warm standby support.
 * @param on        true = power on, false = power off
 * @param keep_vrtc When powering off: true = keep VRTC alive (warm standby,
 *                  preserves ephemeris/almanac/RTC for fast re-acquisition),
 *                  false = full power-off (cold start on next wake).
 *                  Only relevant on T1000-E (HAS_GPS_VRTC); ignored on other boards. */
static void gps_power_control(bool on, bool keep_vrtc = false)
{
#if HAS_GPS_POWER_CONTROL
	/* Direct GPIO power control — works on all boards.
	 * We toggle the GPS power pin ourselves rather than using driver PM
	 * (driver PM can hang on modem_pipe_close / modem_chat_run_script).
	 * The GNSS driver's modem pipe stays open.
	 *
	 * T1000-E (HAS_GPS_VRTC): Use warm standby (keep VRTC) for app toggle
	 *   so UART/chip state is preserved. Matches Arduino sleep_gps().
	 * Simple boards (Wio etc.): Full power off/on via GPS_EN. */
	if (on) {
#if HAS_T1000_GPS_CONTROL
		/* T1000-E power-on sequence (from Arduino target.cpp start_gps())
		 * Must follow this exact order with delays:
		 * 1. GPS_EN HIGH, delay 10ms
		 * 2. GPS_VRTC_EN HIGH, delay 10ms (critical - RTC power)
		 * 3. GPS_RESET HIGH, delay 10ms, then LOW
		 * 4. GPS_SLEEP_INT HIGH
		 */
		if (gpio_is_ready_dt(&gps_enable_gpio)) {
			gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_HIGH);
		}
		k_msleep(10);

#if HAS_GPS_VRTC
		if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
			gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_HIGH);
		}
		k_msleep(10);
#endif

#if HAS_GPS_RESET
		if (gpio_is_ready_dt(&gps_reset_gpio)) {
			gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_HIGH);
			k_msleep(10);
			gpio_pin_set_dt(&gps_reset_gpio, 0);  /* Release reset */
		}
#endif

#if HAS_GPS_SLEEP
		if (gpio_is_ready_dt(&gps_sleep_gpio)) {
			gpio_pin_configure_dt(&gps_sleep_gpio, GPIO_OUTPUT_HIGH);
		}
#endif

#if HAS_GPS_RTCINT
		/* GPS_RTC_INT (P0.15) — held LOW during normal operation */
		if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
			gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_LOW);
		}
#endif

#if HAS_GPS_RESETB
		/* GPS_RESETB (P1.14) — active-LOW reset, must be pulled HIGH.
		 * INPUT_PULLUP de-asserts reset so the AG3335 can boot.
		 * Without this the pin floats LOW → chip stuck in reset → no UART. */
		if (gpio_is_ready_dt(&gps_resetb_gpio)) {
			gpio_pin_configure_dt(&gps_resetb_gpio, GPIO_INPUT | GPIO_PULL_UP);
		}
#endif
		gps_gpio_configured = true;
		LOG_INF("GPS power ON (T1000-E sequence)");
#else
		/* Simple boards - just GPS_EN */
		if (!gps_gpio_configured) {
			if (gpio_is_ready_dt(&gps_enable_gpio)) {
				gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_HIGH);
				gps_gpio_configured = true;
				LOG_INF("GPS power GPIO configured, set HIGH");
			} else {
				LOG_WRN("GPS power GPIO not ready");
				return;
			}
		} else {
			gpio_pin_set_dt(&gps_enable_gpio, 1);
			LOG_INF("GPS power ON");
		}
#endif
	} else {
		/* Power off sequence */
#if HAS_GPS_RESET
		/* Hold GPS in reset during power-off — matches Arduino sleep_gps()/stop_gps().
		 * Ensures chip sees RESET asserted when GPS_EN goes HIGH on next
		 * power-on, preventing uncontrolled startup before the reset pulse. */
		if (gpio_is_ready_dt(&gps_reset_gpio)) {
			gpio_pin_set_dt(&gps_reset_gpio, 1);
		}
#endif

#if HAS_GPS_VRTC
		if (!keep_vrtc) {
			/* Full power-off: VRTC off too (cold start on next wake) */
			if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
				if (!gps_gpio_configured) {
					gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_LOW);
				} else {
					gpio_pin_set_dt(&gps_vrtc_gpio, 0);
				}
			}
		}
		/* else: warm standby — VRTC stays HIGH, preserving
		 * ephemeris/almanac/RTC for fast re-acquisition (~1-2 µA) */
#endif
		if (gpio_is_ready_dt(&gps_enable_gpio)) {
			if (!gps_gpio_configured) {
				gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_LOW);
				gps_gpio_configured = true;
			} else {
				gpio_pin_set_dt(&gps_enable_gpio, 0);
			}
		}

#if HAS_GPS_RESETB
		/* Drive RESETB LOW when GPS is off (Arduino sleep_gps/stop_gps) */
		if (gpio_is_ready_dt(&gps_resetb_gpio)) {
			gpio_pin_configure_dt(&gps_resetb_gpio, GPIO_OUTPUT_LOW);
		}
#endif

#if HAS_GPS_RTCINT
		/* GPS_RTC_INT stays LOW during sleep/off (same as normal operation) */
		if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
			gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_LOW);
		}
#endif

#if HAS_GPS_VRTC
		LOG_INF("GPS power OFF (%s)", keep_vrtc ?
			"standby — VRTC retained" : "full");
#else
		LOG_INF("GPS power OFF");
#endif
	}
#else
	ARG_UNUSED(keep_vrtc);
#endif
}

/* Drive all GPS power-enable GPIOs LOW for System OFF.
 * Uses gpio_pin_configure_dt() so pins are properly set even if
 * gps_power_control() was never called (GPIO not yet configured). */
void gps_power_off_for_shutdown(void)
{
#if HAS_GPS_POWER_CONTROL
	if (gpio_is_ready_dt(&gps_enable_gpio)) {
		gpio_pin_configure_dt(&gps_enable_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_VRTC
	if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
		gpio_pin_configure_dt(&gps_vrtc_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_RESET
	if (gpio_is_ready_dt(&gps_reset_gpio)) {
		gpio_pin_configure_dt(&gps_reset_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_SLEEP
	if (gpio_is_ready_dt(&gps_sleep_gpio)) {
		gpio_pin_configure_dt(&gps_sleep_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_RTCINT
	if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
		gpio_pin_configure_dt(&gps_rtcint_gpio, GPIO_OUTPUT_LOW);
	}
#endif
#if HAS_GPS_RESETB
	if (gpio_is_ready_dt(&gps_resetb_gpio)) {
		gpio_pin_configure_dt(&gps_resetb_gpio, GPIO_OUTPUT_LOW);
	}
#endif
}

#if HAS_GNSS  /* Resume GNSS-specific code */

/* ========== Software Sleep/Wake (no GPIO required) ==========
 *
 * On boards without dedicated GPS power control (e.g. RAK3401 where the
 * 3V3_S rail is shared with the LoRa FEM), we send vendor-specific UART
 * commands to put the GPS module into low-power mode.
 *
 * Strategy: send BOTH Quectel and u-blox sleep commands — the module that
 * isn't present simply ignores the bytes it doesn't understand.
 *
 * - Quectel L76K (RAK1910):  $PMTK161,0*28\r\n → standby (~1mA), wake on UART
 * - u-blox ZOE-M8Q (RAK12500): UBX-RXM-PMREQ   → backup  (~7µA), wake on UART
 *
 * Wake: any byte on UART wakes both modules from their low-power modes.
 * After wake, the module resumes outputting NMEA autonomously.
 */

/* Get the UART device that the GNSS module is connected to.
 * Works for any GNSS-on-UART node regardless of compatible string. */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gnss), okay) && \
    DT_NODE_HAS_STATUS(DT_BUS(DT_NODELABEL(gnss)), okay)
#define HAS_GPS_UART 1
#if !HAS_GPS_POWER_CONTROL
static const struct device *gps_uart_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(gnss)));
#endif
#else
#define HAS_GPS_UART 0
#endif

#if HAS_GPS_UART && !HAS_GPS_POWER_CONTROL
/* Send raw bytes to the GPS UART using blocking poll_out.
 * Safe to call even though modem_chat/modem_ubx owns the UART pipe:
 * uart_poll_out writes one byte at a time through the TX register,
 * and GNSS modules are receive-only (no TX contention). */
static void gps_uart_send(const uint8_t *data, size_t len)
{
	if (!device_is_ready(gps_uart_dev)) {
		return;
	}
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(gps_uart_dev, data[i]);
	}
}

/* Quectel L76K: $PMTK161,0*28\r\n → enter standby mode
 * Module stops NMEA output and draws ~1mA. Wakes on any UART RX byte. */
static const uint8_t pmtk_standby[] = "$PMTK161,0*28\r\n";

/* u-blox ZOE-M8Q: UBX-RXM-PMREQ → enter backup mode
 * UBX frame: B5 62 | 02 41 | 10 00 | payload(16) | CK_A CK_B
 * Payload (protocol 23+, 16 bytes):
 *   version=0, reserved[3]=0,
 *   duration=0x00000000 (infinite),
 *   flags=0x00000006 (backup + force),
 *   wakeupSources=0x00000020 (UART RX)
 * Module stops all output and draws ~7µA. Wakes on any UART RX byte. */
static const uint8_t ubx_pmreq_backup[] = {
	0xB5, 0x62,             /* UBX sync chars */
	0x02, 0x41,             /* Class: RXM, ID: PMREQ */
	0x10, 0x00,             /* Length: 16 bytes (little-endian) */
	/* Payload */
	0x00,                   /* version */
	0x00, 0x00, 0x00,       /* reserved1[3] */
	0x00, 0x00, 0x00, 0x00, /* duration: 0 = infinite */
	0x06, 0x00, 0x00, 0x00, /* flags: backup(0x02) | force(0x04) */
	0x20, 0x00, 0x00, 0x00, /* wakeupSources: UART RX (bit 5) */
	/* Checksum (Fletcher-8 over class..payload) */
	0x79, 0xCB
};

/* Put GPS module into software sleep (for boards without GPIO power control).
 * Sends both Quectel PMTK and u-blox UBX commands — the wrong one is
 * harmlessly ignored by whichever module is actually connected. */
static void gps_software_sleep(void)
{
	LOG_INF("GPS: Sending software sleep (PMTK + UBX)");

	/* Quectel L76K standby */
	gps_uart_send(pmtk_standby, sizeof(pmtk_standby) - 1);  /* exclude null terminator */

	/* Small delay between commands — let the first one drain */
	k_msleep(50);

	/* u-blox ZOE-M8Q backup */
	gps_uart_send(ubx_pmreq_backup, sizeof(ubx_pmreq_backup));

	LOG_DBG("GPS: Software sleep commands sent");
}

/* Wake GPS module from software sleep.
 * A single 0xFF byte on UART triggers wake on both Quectel and u-blox.
 * After wake, the module resumes NMEA output within ~100-500ms. */
static void gps_software_wake(void)
{
	LOG_INF("GPS: Sending UART wake byte");
	const uint8_t wake = 0xFF;
	gps_uart_send(&wake, 1);
	/* Give the module time to boot and start NMEA output */
	k_msleep(200);
}
#endif /* HAS_GPS_UART && !HAS_GPS_POWER_CONTROL */

/* Go to standby and schedule next wake.
 * GPIO power control only — keep VRTC for warm start on T1000-E,
 * FORCE_ON pin LOW for L76K hardware standby. */
static void gps_go_to_standby(void)
{
	if (gps_repeater_mode && gps_runtime_no_gnss_activity) {
		LOG_WRN("GPS: No GNSS activity detected; disabling repeater GPS cycles until reboot");
		gps_current_state = GPS_STATE_OFF;
		consecutive_good_fixes = 0;
		standby_start_ms = 0;
		standby_interval_ms = 0;

#if HAS_GPS_POWER_CONTROL
		gps_power_control(false, true);
#elif HAS_GPS_UART
		gps_software_sleep();
#endif
		return;
	}

	uint64_t wake_interval = gps_repeater_mode ?
		GPS_REPEATER_SYNC_INTERVAL_MS : gps_wake_interval_ms;

	if (gps_repeater_mode) {
		LOG_INF("GPS: Going to standby for 48 hours (repeater mode)");
	} else {
		LOG_INF("GPS: Going to standby for %d minutes", (int)(wake_interval / 60000));
	}
	gps_current_state = GPS_STATE_STANDBY;
	consecutive_good_fixes = 0;

	/* Record standby timing for UI (next-wake calculation) */
	standby_start_ms = k_uptime_get();
	standby_interval_ms = wake_interval;

	/* Power down the GPS module.
	 * GPIO boards: hardware power-off (keep VRTC for warm start on T1000-E).
	 * Non-GPIO boards: software sleep via UART commands (PMTK + UBX). */
#if HAS_GPS_POWER_CONTROL
	gps_power_control(false, true);
#elif HAS_GPS_UART
	gps_software_sleep();
#endif

	/* NOTE: gnss_configured stays true — L76K retains PCAS settings in
	 * flash across power cycles. Re-running gnss_configure() after GPIO
	 * wake would call modem_chat_run_script() before the chip has booted,
	 * risking a deadlock (modem_chat blocks on system work queue). */

	/* Schedule next wake (event-driven, no polling!) */
	k_work_schedule(&gps_wake_work, K_MSEC(wake_interval));
}

/* Wake GPS and start acquiring.
 * GPIO boards: hardware power-on.
 * Non-GPIO boards: UART wake byte (wakes L76K from standby, ZOE-M8Q from backup).
 * Does NOT call gnss_configure() — constellation/fix-rate settings persist
 * in L76K flash across power cycles. Calling modem_chat_run_script() here
 * would deadlock: the chip needs ~300ms to boot after GPIO power restore,
 * but modem_chat blocks the calling thread waiting for the system work
 * queue which may be processing stale UART data. */
static void gps_start_acquiring(void)
{
	if (gps_repeater_mode && gps_runtime_no_gnss_activity) {
		LOG_INF("GPS: Repeater cycles are runtime-disabled (no GNSS activity detected)");
		return;
	}

	LOG_INF("GPS: Waking for %s", gps_repeater_mode ? "time sync" : "position fix");
	gps_current_state = GPS_STATE_ACQUIRING;
	consecutive_good_fixes = 0;
	gnss_activity_seen_this_cycle = false;

#if HAS_GPS_POWER_CONTROL
	gps_power_control(true);
#elif HAS_GPS_UART
	gps_software_wake();
#endif

	/* Start timeout timer */
	if (gps_repeater_mode) {
		/* Repeater mode: always use 5 min timeout for time sync */
		k_work_schedule(&gps_timeout_work, K_MSEC(GPS_REPEATER_SYNC_TIMEOUT_MS));
	} else if (first_fix_acquired) {
		/* Companion mode: use short timeout after first fix acquired */
		k_work_schedule(&gps_timeout_work, K_MSEC(gps_acquire_timeout_ms));
	} else {
		LOG_INF("GPS: First fix mode - no timeout");
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
		LOG_WRN("GPS: Repeater acquire window had no GNSS callbacks; disabling periodic GPS until reboot");
		gps_runtime_no_gnss_activity = true;
	}

	atomic_or(&pending_gps_actions, GPS_ACTION_TIMEOUT);
	if (gps_event_cb) {
		gps_event_cb();
	}
}

/* ========== GPS UART Diagnostics ========== */

/**
 * Dump nRF52840 UARTE0 hardware register state.
 * Reads PSEL (pin select), ENABLE, BAUDRATE, and ERRORSRC directly
 * from the peripheral registers — no assumptions, just facts.
 */
static void gps_uart_dump_hw_state(void)
{
#if defined(CONFIG_SOC_NRF52840)
	NRF_UARTE_Type *uart = NRF_UARTE0;

	uint32_t psel_txd = uart->PSEL.TXD;
	uint32_t psel_rxd = uart->PSEL.RXD;
	uint32_t enable   = uart->ENABLE;
	uint32_t baudrate = uart->BAUDRATE;
	uint32_t errorsrc = uart->ERRORSRC;

	/* PSEL format: bit 31 = CONNECT (0=connected, 1=disconnected),
	 * bits 4:0 = pin, bit 5 = port */
	bool txd_connected = !(psel_txd & (1U << 31));
	bool rxd_connected = !(psel_rxd & (1U << 31));
	uint8_t txd_port = (psel_txd >> 5) & 1;
	uint8_t txd_pin  = psel_txd & 0x1F;
	uint8_t rxd_port = (psel_rxd >> 5) & 1;
	uint8_t rxd_pin  = psel_rxd & 0x1F;

	LOG_INF("UART0 HW state:");
	LOG_INF("  ENABLE=0x%02x (8=enabled)", enable);
	LOG_INF("  PSEL.TXD=0x%08x → P%d.%02d %s",
		psel_txd, txd_port, txd_pin,
		txd_connected ? "CONNECTED" : "DISCONNECTED");
	LOG_INF("  PSEL.RXD=0x%08x → P%d.%02d %s",
		psel_rxd, rxd_port, rxd_pin,
		rxd_connected ? "CONNECTED" : "DISCONNECTED");
	LOG_INF("  BAUDRATE=0x%08x ERRORSRC=0x%x", baudrate, errorsrc);

	/* Clear any error flags */
	if (errorsrc) {
		uart->ERRORSRC = errorsrc;
		LOG_WRN("  UART errors cleared: overrun=%d parity=%d framing=%d break=%d",
			(errorsrc >> 0) & 1, (errorsrc >> 1) & 1,
			(errorsrc >> 2) & 1, (errorsrc >> 3) & 1);
	}
#endif
}

#if HAS_GPS_POWER_CONTROL
/**
 * Log actual GPIO pin states after power-up sequence.
 * Reads back each configured pin to verify the hardware accepted our config.
 */
static void gps_dump_gpio_states(void)
{
	LOG_INF("GPS GPIO states after power-up:");
	if (gpio_is_ready_dt(&gps_enable_gpio)) {
		LOG_INF("  GPS_EN (P1.11): %d", gpio_pin_get_dt(&gps_enable_gpio));
	}
#if HAS_GPS_VRTC
	if (gpio_is_ready_dt(&gps_vrtc_gpio)) {
		LOG_INF("  GPS_VRTC_EN (P0.08): %d", gpio_pin_get_dt(&gps_vrtc_gpio));
	}
#endif
#if HAS_GPS_RESET
	if (gpio_is_ready_dt(&gps_reset_gpio)) {
		LOG_INF("  GPS_RESET (P1.15): %d", gpio_pin_get_dt(&gps_reset_gpio));
	}
#endif
#if HAS_GPS_SLEEP
	if (gpio_is_ready_dt(&gps_sleep_gpio)) {
		LOG_INF("  GPS_SLEEP_INT (P1.12): %d", gpio_pin_get_dt(&gps_sleep_gpio));
	}
#endif
#if HAS_GPS_RTCINT
	if (gpio_is_ready_dt(&gps_rtcint_gpio)) {
		LOG_INF("  GPS_RTC_INT (P0.15): %d", gpio_pin_get_dt(&gps_rtcint_gpio));
	}
#endif
#if HAS_GPS_RESETB
	if (gpio_is_ready_dt(&gps_resetb_gpio)) {
		LOG_INF("  GPS_RESETB (P1.14): %d (INPUT_PULLUP, expect 1)",
			gpio_pin_get_dt(&gps_resetb_gpio));
	}
#endif
}
#endif /* HAS_GPS_POWER_CONTROL */

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
#elif DT_HAS_COMPAT_STATUS_OKAY(quectel_lcx6g)
	gnss_dev = DEVICE_DT_GET_ANY(quectel_lcx6g);
#elif DT_HAS_COMPAT_STATUS_OKAY(u_blox_m8)
	gnss_dev = DEVICE_DT_GET_ANY(u_blox_m8);
#elif DT_HAS_COMPAT_STATUS_OKAY(u_blox_f9p)
	gnss_dev = DEVICE_DT_GET_ANY(u_blox_f9p);
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

#if defined(CONFIG_SOC_NRF52840)
		NRF_UARTE_Type *uart = NRF_UARTE0;

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

	LOG_INF("GNSS device %s initialized", gnss_dev->name);
	gps_available = true;
	return 0;
}
#endif /* HAS_GNSS */

/* ========== Public API ========== */

int gps_manager_init(void)
{
#if HAS_GNSS
	gnss_init();

	/* Restore last known position from flash (survives reboot) */
	gps_load_position();

	/* Configure constellations + fix rate NOW while chip is powered
	 * and the modem pipe is open (driver init already ran).
	 * This is the ONLY safe place to call modem_chat_run_script() —
	 * after power cycles the chip needs ~300ms boot time and calling
	 * modem_chat from the main thread risks deadlock. L76K retains
	 * PCAS settings in flash, so one-time config at boot is enough. */
	gnss_configure();
#endif
	return 0;
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
		gps_power_control(false);
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

	gps_repeater_mode = repeater;

	if (repeater) {
		LOG_INF("GPS: Repeater mode - starting initial time sync, then every 48h");

		gps_enabled = true;  /* Logically enabled */
		gps_runtime_no_gnss_activity = false;

		/* Start acquiring immediately for initial time sync at boot.
		 * GPS hardware is already powered from bootloader, so we just
		 * start the acquisition state machine. */
		gps_start_acquiring();
	} else {
		LOG_INF("GPS: Companion mode");
	}
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

		/* Start acquiring immediately (no delay for first wake) */
		gps_current_state = GPS_STATE_ACQUIRING;
		consecutive_good_fixes = 0;

		/* Power on GPS - uses lazy GPIO init */
		gps_power_control(true);

		/* gnss_configure() runs once at boot (see gps_manager_init path).
		 * L76K retains PCAS settings in flash across power cycles.
		 * Do NOT call modem_chat_run_script() here — the chip needs
		 * ~300ms to boot after GPIO power restore and calling it
		 * immediately deadlocks the main thread. */

		/* No timeout for first fix after enable - wait as long as needed */
		if (first_fix_acquired) {
			k_work_schedule(&gps_timeout_work, K_MSEC(gps_acquire_timeout_ms));
		} else {
			LOG_INF("GPS: First fix mode - no timeout until first successful fix");
		}
	} else {
		LOG_INF("GPS disabled - canceling timers and powering off");

		/* Cancel any pending work */
		k_work_cancel_delayable(&gps_wake_work);
		k_work_cancel_delayable(&gps_timeout_work);

		/* Power off GPS — warm standby if VRTC available (Arduino sleep_gps),
		 * full power off otherwise. Warm standby preserves ephemeris/RTC
		 * in AG3335 backup RAM for fast re-acquisition (1-8s vs 15-45s). */
#if HAS_GPS_VRTC
		gps_power_control(false, true);  /* Warm standby — keep VRTC */
#else
		gps_power_control(false);        /* No VRTC — full power off */
#endif
		gps_current_state = GPS_STATE_OFF;
		consecutive_good_fixes = 0;

		/* Clear first-fix flag so the next enable gets the "no timeout"
		 * grace period again — in marginal signal, a 30–120s timeout may
		 * never be enough, and the user explicitly toggled GPS expecting
		 * it to try hard for a fix. */
		first_fix_acquired = false;

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
	if (interval < 10) interval = 10;
	if (interval > 86400) interval = 86400;
	gps_wake_interval_ms = interval * 1000U;
	LOG_INF("GPS poll interval set to %u seconds", interval);
#else
	ARG_UNUSED(interval);
#endif
}

int64_t gps_get_utc_time(void)
{
#if HAS_GNSS
	k_mutex_lock(&gps_mutex, K_FOREVER);
	if (!current_pos.valid) {
		k_mutex_unlock(&gps_mutex);
		return 0;
	}

	struct gnss_time t = current_utc;
	k_mutex_unlock(&gps_mutex);

	int year = 2000 + t.century_year;
	int days = 0;

	for (int y = 1970; y < year; y++) {
		days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
	}

	static const int month_days[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	for (int m = 1; m < t.month; m++) {
		days += month_days[m];
		if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
			days++;
		}
	}

	days += t.month_day - 1;

	int64_t timestamp = (int64_t)days * 86400;
	timestamp += t.hour * 3600;
	timestamp += t.minute * 60;
	timestamp += t.millisecond / 1000;

	return timestamp;
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
		 * could use. Repeater mode uses its own 5min timeout; companion
		 * mode pre-first-fix has no timeout (nothing to reschedule). */
		uint32_t timeout_ms = 0;
		if (gps_repeater_mode) {
			timeout_ms = GPS_REPEATER_SYNC_TIMEOUT_MS;
		} else if (first_fix_acquired) {
			timeout_ms = gps_acquire_timeout_ms;
		}
		if (timeout_ms > 0) {
			LOG_INF("GPS: Fresh fix requested, extending acquire "
				"timeout by %u ms", timeout_ms);
			k_work_reschedule(&gps_timeout_work, K_MSEC(timeout_ms));
		} else {
			LOG_DBG("GPS: Fresh fix requested, already acquiring "
				"(no timeout active)");
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
