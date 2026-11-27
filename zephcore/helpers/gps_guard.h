/*
 * SPDX-License-Identifier: MIT
 * GpsGuard - anti-spoofing checks on GPS fixes and GPS time, upstream's
 * GPS_GUARD (src/helpers/sensors/GpsGuard.h) with the clock and the build date
 * passed in, so it runs on the host. No Zephyr dependency.
 *
 * checkFix(): satellites, HDOP, optional region box, fence around a reference
 * position, jump speed from the last trusted fix, and GPS_GUARD_STABLE_FIXES
 * consecutive fixes within GPS_GUARD_STABLE_M of each other.
 * acceptTime(): GPS time only while a trusted fix is current, inside
 * [build date - 1 day, +10 years], and once the clock is plausible only a step
 * up to GPS_GUARD_TIME_MAX_STEP, after the first accepted sync only drift-sized.
 * A GPS time that disagrees with the clock by the same offset for
 * GPS_GUARD_ALERT_MIN minutes raises an alert; acceptAlertedTime() clears it.
 * devdocs/lld/10-peripherals-power.md, "GPS guard".
 */

#ifndef ZEPHCORE_GPS_GUARD_H
#define ZEPHCORE_GPS_GUARD_H

#include <stddef.h>
#include <stdint.h>

#define GPS_GUARD_MIN_SATS        4
#define GPS_GUARD_MAX_HDOP10      40      /* HDOP 4.0 */
#define GPS_GUARD_STABLE_FIXES    5
#define GPS_GUARD_STABLE_M        100
#define GPS_GUARD_MAX_SPEED_KMH   250
#define GPS_GUARD_TIME_MAX_STEP   600     /* s */
#define GPS_GUARD_TIME_DRIFT_BASE 5       /* s, plus ... */
#define GPS_GUARD_TIME_DRIFT_PPM  100     /* ... this much of the time since the last sync */
#define GPS_GUARD_ALERT_MIN       60
#define GPS_GUARD_ALERT_TOLERANCE 60      /* s */
#define GPS_GUARD_TRUSTED_MS      5000
#define GPS_GUARD_FENCE_MAX_M     20000000UL  /* half the Earth's circumference */
#define GPS_GUARD_HDOP_UNKNOWN    255

class GpsGuard {
public:
	enum Verdict {
		REJECT,  /* implausible: not stored, restarts the stable count */
		SETTLE,  /* plausible, stable count not reached yet: not stored */
		TRUSTED, /* store it */
	};

	/* build_epoch: the firmware's build day (FIRMWARE_BUILD_EPOCH);
	 * fence_m: the default fence radius, 0 = off. */
	GpsGuard(uint32_t build_epoch, uint32_t fence_m);

	/* lat/lon in degrees, hdop10 in tenths (GPS_GUARD_HDOP_UNKNOWN = none),
	 * gps_time = UTC epoch of the fix, now_ms = a monotonic millisecond clock. */
	Verdict checkFix(double lat, double lon, uint32_t sats, uint8_t hdop10,
			 uint32_t gps_time, uint32_t now_ms);
	/* The receiver has no fix: restart the stable count. */
	void noFix(uint32_t sats);

	bool acceptTime(uint32_t gps_time, uint32_t clock_time, uint32_t now_ms);
	/* Another source set the clock: drop the drift anchor and any alert. */
	void clockSetManually();
	/* The operator confirms the alerted GPS time; offset = GPS minus clock.
	 * True: the caller writes gps_time to the clock now. */
	bool acceptAlertedTime(int32_t offset, uint32_t gps_time, uint32_t now_ms);

	/* "ok 7sat time ok +1s", "settle 2/5 time wait", "rej fence 3.2km time -",
	 * "... ALERT gps-clk +3600s 75m", or "off". */
	void formatStatus(char *out, size_t out_size, uint32_t now_ms) const;

	void setEnabled(bool enabled);
	bool isEnabled() const { return _enabled; }
	/* 0,0 = no reference: the fence is off. */
	void setReference(double lat, double lon) { _ref_lat = lat; _ref_lon = lon; }
	double refLat() const { return _ref_lat; }
	double refLon() const { return _ref_lon; }
	void setFence(uint32_t metres) { _fence_m = metres; }
	uint32_t fence() const { return _fence_m; }
	/* Fixes outside this box are rejected; not set = no region check. */
	void setRegion(double lat_min, double lat_max, double lon_min, double lon_max);
	bool isTimeAlert() const { return _time_alert; }
	bool trustedFixIsCurrent(uint32_t now_ms) const;
	uint32_t minEpoch() const { return _min_epoch; }
	uint32_t maxEpoch() const { return _min_epoch + 10UL * 365UL * 86400UL; }

	static double distanceM(double lat1, double lon1, double lat2, double lon2);

private:
	void trackTimeDisagreement(int32_t offset, uint32_t now_ms);

	uint32_t _min_epoch;
	bool _enabled = true;
	double _ref_lat = 0, _ref_lon = 0;
	uint32_t _fence_m;
	bool _have_region = false;
	double _lat_min = 0, _lat_max = 0, _lon_min = 0, _lon_max = 0;

	bool _have_trusted = false;
	uint32_t _last_trusted_ms = 0;
	double _last_trusted_lat = 0, _last_trusted_lon = 0;
	double _cand_lat = 0, _cand_lon = 0;
	uint32_t _cand_time = 0;
	uint8_t _stable_count = 0;
	char _fix_status[24];

	char _time_status[24];
	bool _time_accepted = false;
	uint32_t _last_time_accept_ms = 0;
	bool _time_disagree = false;
	int32_t _time_disagree_offset = 0;
	uint32_t _time_disagree_since_ms = 0;
	bool _time_alert = false;
};

#endif /* ZEPHCORE_GPS_GUARD_H */
