/*
 * SPDX-License-Identifier: MIT
 * GpsGuard - anti-spoofing checks on GPS fixes and GPS time. See gps_guard.h.
 */

#include "gps_guard.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

GpsGuard::GpsGuard(uint32_t build_epoch, uint32_t fence_m)
	: _min_epoch(build_epoch > 86400U ? build_epoch - 86400U : 0U), _fence_m(fence_m)
{
	strcpy(_fix_status, "no fix");
	strcpy(_time_status, "-");
}

/* Haversine. The differences are taken in double, the trigonometry in float
 * (the nRF52840 FPU is single precision): still sub-metre at fence distances. */
double GpsGuard::distanceM(double lat1, double lon1, double lat2, double lon2)
{
	const float R = 6371000.0f, D2R = 0.017453292f;
	float dlat = (float)(lat2 - lat1) * D2R, dlon = (float)(lon2 - lon1) * D2R;
	float s1 = sinf(dlat / 2), s2 = sinf(dlon / 2);
	float a = s1 * s1 + cosf((float)lat1 * D2R) * cosf((float)lat2 * D2R) * s2 * s2;

	if (a > 1.0f) {
		a = 1.0f;
	}
	return 2.0 * R * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

void GpsGuard::setRegion(double lat_min, double lat_max, double lon_min, double lon_max)
{
	_have_region = true;
	_lat_min = lat_min;
	_lat_max = lat_max;
	_lon_min = lon_min;
	_lon_max = lon_max;
}

void GpsGuard::setEnabled(bool enabled)
{
	if (enabled == _enabled) {
		return;
	}
	_enabled = enabled;
	/* A re-enabled guard starts over: fresh stable fixes, no alert. */
	_have_trusted = false;
	_stable_count = 0;
	_time_accepted = false;
	_time_disagree = false;
	_time_alert = false;
	strcpy(_fix_status, "no fix");
	strcpy(_time_status, "-");
}

bool GpsGuard::trustedFixIsCurrent(uint32_t now_ms) const
{
	return _have_trusted && (uint32_t)(now_ms - _last_trusted_ms) <= GPS_GUARD_TRUSTED_MS;
}

GpsGuard::Verdict GpsGuard::checkFix(double lat, double lon, uint32_t sats, uint8_t hdop10,
				     uint32_t gps_time, uint32_t now_ms)
{
	if (!_enabled) {
		return TRUSTED;
	}

	const char *why = NULL;
	double dist = 0;

	if (sats < GPS_GUARD_MIN_SATS) {
		why = "sats";
	} else if (hdop10 == GPS_GUARD_HDOP_UNKNOWN || hdop10 > GPS_GUARD_MAX_HDOP10) {
		why = "hdop";
	} else if (_have_region &&
		   (lat < _lat_min || lat > _lat_max || lon < _lon_min || lon > _lon_max)) {
		why = "region";
	} else if (_fence_m > 0 && (_ref_lat != 0 || _ref_lon != 0) &&
		   (dist = distanceM(_ref_lat, _ref_lon, lat, lon)) > _fence_m) {
		why = "fence";
	} else if (_have_trusted) {
		dist = distanceM(_last_trusted_lat, _last_trusted_lon, lat, lon);
		double secs = (uint32_t)(now_ms - _last_trusted_ms) / 1000.0;

		if (secs < 1) {
			secs = 1;
		}
		if (dist / secs > GPS_GUARD_MAX_SPEED_KMH / 3.6) {
			why = "jump";
		}
	}

	if (why != NULL) {
		_stable_count = 0;
		if (dist > 0) {
			snprintf(_fix_status, sizeof(_fix_status), "rej %s %.1fkm", why, dist / 1000.0);
		} else {
			snprintf(_fix_status, sizeof(_fix_status), "rej %s", why);
		}
		return REJECT;
	}

	if (!_have_trusted || _stable_count < GPS_GUARD_STABLE_FIXES) {
		/* Consecutive fixes within GPS_GUARD_STABLE_M of the previous one,
		 * each GPS second counted once. */
		if (_stable_count > 0 &&
		    distanceM(_cand_lat, _cand_lon, lat, lon) > GPS_GUARD_STABLE_M) {
			_stable_count = 0;
		}
		if ((_stable_count == 0 || gps_time != _cand_time) && _stable_count < 255) {
			_stable_count++;
		}
		_cand_lat = lat;
		_cand_lon = lon;
		_cand_time = gps_time;
		if (_stable_count < GPS_GUARD_STABLE_FIXES) {
			snprintf(_fix_status, sizeof(_fix_status), "settle %u/%u",
				 (unsigned)_stable_count, (unsigned)GPS_GUARD_STABLE_FIXES);
			return SETTLE;
		}
	}

	_have_trusted = true;
	_last_trusted_ms = now_ms;
	_last_trusted_lat = lat;
	_last_trusted_lon = lon;
	snprintf(_fix_status, sizeof(_fix_status), "ok %usat", (unsigned)sats);
	return TRUSTED;
}

void GpsGuard::noFix(uint32_t sats)
{
	_stable_count = 0;
	snprintf(_fix_status, sizeof(_fix_status), "no fix %usat", (unsigned)sats);
}

void GpsGuard::trackTimeDisagreement(int32_t offset, uint32_t now_ms)
{
	int32_t diff = offset - _time_disagree_offset;

	if (!_time_disagree || diff > GPS_GUARD_ALERT_TOLERANCE ||
	    diff < -GPS_GUARD_ALERT_TOLERANCE) {
		_time_disagree = true;
		_time_disagree_offset = offset;
		_time_disagree_since_ms = now_ms;
		_time_alert = false;
		return;
	}
	if (!_time_alert &&
	    (uint32_t)(now_ms - _time_disagree_since_ms) >= GPS_GUARD_ALERT_MIN * 60000UL) {
		_time_alert = true;
	}
}

bool GpsGuard::acceptTime(uint32_t gps_time, uint32_t clock_time, uint32_t now_ms)
{
	if (!_enabled) {
		return true;
	}

	const char *why = NULL;
	int32_t step = 0;

	if (!trustedFixIsCurrent(now_ms)) {
		why = "nofix";
	} else if (gps_time < minEpoch() || gps_time > maxEpoch()) {
		why = "range";
	} else if (clock_time >= minEpoch() && clock_time <= maxEpoch()) {
		/* A clock outside the plausible window counts as unset: GPS sets it. */
		step = (int32_t)(gps_time - clock_time);
		int32_t limit = GPS_GUARD_TIME_MAX_STEP;

		if (_time_accepted) {
			uint32_t elapsed_s = (uint32_t)(now_ms - _last_time_accept_ms) / 1000U;
			uint64_t drift = GPS_GUARD_TIME_DRIFT_BASE +
				(uint64_t)elapsed_s * GPS_GUARD_TIME_DRIFT_PPM / 1000000U;

			limit = drift < GPS_GUARD_TIME_MAX_STEP ? (int32_t)drift : GPS_GUARD_TIME_MAX_STEP;
		}
		if (step > limit || step < -limit) {
			why = "step";
		}
	}

	if (why != NULL) {
		if (step != 0) {
			snprintf(_time_status, sizeof(_time_status), "rej %s %lds", why, (long)step);
			trackTimeDisagreement(step, now_ms);
		} else {
			snprintf(_time_status, sizeof(_time_status), "rej %s", why);
		}
		return false;
	}
	snprintf(_time_status, sizeof(_time_status), "ok %+lds",
		 (long)(int32_t)(gps_time - clock_time));
	/* Every accepted sync anchors the drift budget. */
	_time_disagree = false;
	_time_alert = false;
	_time_accepted = true;
	_last_time_accept_ms = now_ms;
	return true;
}

void GpsGuard::clockSetManually()
{
	_time_accepted = false;
	_time_disagree = false;
	_time_alert = false;
}

bool GpsGuard::acceptAlertedTime(int32_t offset, uint32_t gps_time, uint32_t now_ms)
{
	int32_t diff = offset - _time_disagree_offset;

	if (!_time_alert || !trustedFixIsCurrent(now_ms)) {
		return false;
	}
	if (diff > GPS_GUARD_ALERT_TOLERANCE || diff < -GPS_GUARD_ALERT_TOLERANCE) {
		return false;
	}
	if (gps_time < minEpoch() || gps_time > maxEpoch()) {
		return false;
	}
	_time_accepted = true;
	_last_time_accept_ms = now_ms;
	_time_disagree = false;
	_time_alert = false;
	strcpy(_time_status, "ok manual");
	return true;
}

void GpsGuard::formatStatus(char *out, size_t out_size, uint32_t now_ms) const
{
	if (out_size == 0) {
		return;
	}
	if (!_enabled) {
		snprintf(out, out_size, "off");
		return;
	}
	/* An accepted time is shown as current only while its trusted fix is. */
	const char *time_status =
		(_time_status[0] == 'o' && !trustedFixIsCurrent(now_ms)) ? "wait" : _time_status;

	if (_time_alert) {
		snprintf(out, out_size, "%s time %s ALERT gps-clk %+lds %lum", _fix_status,
			 time_status, (long)_time_disagree_offset,
			 (unsigned long)((uint32_t)(now_ms - _time_disagree_since_ms) / 60000U));
	} else {
		snprintf(out, out_size, "%s time %s", _fix_status, time_status);
	}
}
