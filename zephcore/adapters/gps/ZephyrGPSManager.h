/*
 * SPDX-License-Identifier: MIT
 * Zephyr GPS Manager - GNSS power management, fix acquisition, constellation config
 *
 * Event-driven state machine: OFF → ACQUIRING → STANDBY → ACQUIRING → ...
 * Uses Zephyr GNSS API with chip-specific drivers (quectel,lc76g / luatos,air530z / gnss-nmea-generic)
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GPS position data */
struct gps_position {
	int64_t latitude_ndeg;   /* Latitude in nano-degrees */
	int64_t longitude_ndeg;  /* Longitude in nano-degrees */
	int32_t altitude_mm;     /* Altitude in millimeters */
	uint16_t satellites;     /* Number of satellites */
	bool valid;              /* True if fix is valid */
	int64_t timestamp_ms;    /* Timestamp when fix was acquired */
};

/* GPS state info for UI display */
struct gps_state_info {
	uint8_t state;          /* 0=OFF, 1=STANDBY (sleeping), 2=ACQUIRING (searching) */
	uint16_t satellites;    /* Current/last satellite count */
	uint32_t last_fix_age_s;  /* Seconds since last validated fix (UINT32_MAX = never) */
	uint32_t next_search_s;   /* Seconds until next search (0 = searching now or off) */
};

/* GPS enable callback - called when GPS is enabled/disabled (for power management) */
typedef void (*gps_enable_callback_t)(bool enabled);
void gps_set_enable_callback(gps_enable_callback_t cb);

/* GPS fix callback - called when GPS acquires a validated fix (3 consecutive good fixes)
 * Parameters: latitude (degrees), longitude (degrees), utc_time (Unix timestamp)
 * Use this to update node position for mesh advertising */
typedef void (*gps_fix_callback_t)(double lat, double lon, int64_t utc_time);
void gps_set_fix_callback(gps_fix_callback_t cb);

/* GPS event callback - called when the GPS state machine needs the main thread
 * to process a state transition (wake from standby, timeout, etc.).
 *
 * IMPORTANT: GNSS drivers use modem_chat_run_script() which blocks on a
 * semaphore signaled from the system work queue. GPS timer work items also
 * run on the system work queue → calling blocking GNSS APIs from a timer
 * work handler deadlocks the system work queue.
 *
 * Solution: timer work handlers signal this callback, which posts an event
 * to the main mesh thread. The main thread then calls gps_process_event()
 * which safely executes the blocking GNSS configuration. */
typedef void (*gps_event_callback_t)(void);
void gps_set_event_callback(gps_event_callback_t cb);

/* Process pending GPS state transitions — MUST be called from the main
 * thread (not from system work queue) because GNSS configuration uses
 * blocking modem_chat calls.  Called by the mesh event loop when
 * MESH_EVENT_GPS_ACTION is signaled. */
void gps_process_event(void);

/* Initialize GPS manager (call once at boot) */
int gps_manager_init(void);

/* GPS functions */
bool gps_is_available(void);
bool gps_is_enabled(void);
void gps_enable(bool enable);

/* Ensure GPS power state matches prefs at boot.
 * Call this after loading prefs - if GPS should be disabled, powers it off.
 * This is needed because GPS hardware starts powered (bootloader/pull-up). */
void gps_ensure_power_state(bool should_be_enabled);
void gps_get_position(struct gps_position *pos);
uint32_t gps_get_poll_interval_sec(void);
void gps_set_poll_interval_sec(uint32_t interval);

/* Get RTC time from GPS (returns Unix timestamp, 0 if no valid time) */
int64_t gps_get_utc_time(void);

/* Check if we have GPS-synced time (used to reject phone time sync attempts) */
bool gps_has_time_sync(void);

/* Get last known GPS position (even when GPS is sleeping/disabled)
 * Returns true if a valid last position exists, false otherwise.
 * This is useful for telemetry when GPS is in power-save mode. */
bool gps_get_last_known_position(struct gps_position *pos);

/* Trigger GPS wake for a fresh fix (if GPS is enabled but sleeping).
 * Call this after telemetry requests so next request has fresh location.
 * Does nothing if GPS is disabled or already acquiring. */
void gps_request_fresh_fix(void);

/* GPS state info for UI display */
void gps_get_state_info(struct gps_state_info *info);

/* Drive all GPS power-enable GPIOs LOW for System OFF.
 * Safe to call from any context — only touches GPIO, no BLE or SPI.
 * Must be called before sys_poweroff() to prevent GPS module drawing
 * current while nRF52840 is in System OFF (GPIO latches persist). */
void gps_power_off_for_shutdown(void);

/* Set repeater mode for GPS - time sync only, minimal power usage.
 * In repeater mode:
 * - GPS starts powered OFF
 * - Wakes every 48 hours for RTC time sync
 * - 5 minute timeout to acquire fix
 * - Powers off after fix or timeout
 * Call this at boot for repeater role before any gps_enable() calls. */
void gps_set_repeater_mode(bool repeater);

#ifdef __cplusplus
}
#endif
