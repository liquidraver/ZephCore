/*
 * SPDX-License-Identifier: MIT
 * The `gps` CLI's parsing and replies. See gps_cli_text.h.
 */

#include "gps_cli_text.h"

#include <stdio.h>

void gps_cli_duration(char *out, size_t cap, uint32_t seconds)
{
	if (seconds < 60) {
		snprintf(out, cap, "%lus", (unsigned long)seconds);
	} else if (seconds < 3600) {
		snprintf(out, cap, "%lum", (unsigned long)(seconds / 60));
	} else {
		snprintf(out, cap, "%luh", (unsigned long)(seconds / 3600));
	}
}

bool gps_cli_parse_period(const char *arg, uint32_t *seconds)
{
	if (*arg < '0' || *arg > '9') {
		return false;
	}
	uint64_t value = 0;

	while (*arg >= '0' && *arg <= '9') {
		if (value <= UINT32_MAX) {
			value = value * 10 + (uint64_t)(*arg - '0');
		}
		arg++;
	}
	uint32_t mult = 1;

	switch (*arg) {
	case 's': arg++; break;
	case 'm': mult = 60; arg++; break;
	case 'h': mult = 3600; arg++; break;
	case 'd': mult = 86400; arg++; break;
	default: break;
	}
	if (*arg != '\0') {
		return false;
	}
	value *= mult;
	*seconds = value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
	return true;
}

static void mode_text(char *out, size_t cap, uint32_t period_s)
{
	if (period_s == 0) {
		snprintf(out, cap, "continuous");
	} else {
		snprintf(out, cap, "periodic (%lus)", (unsigned long)period_s);
	}
}

void gps_cli_status(char *out, size_t cap, const struct gps_cli_view *v)
{
	char mode[32];

	mode_text(mode, sizeof(mode), v->period_s);
	if (!v->on) {
		snprintf(out, cap, "> off, %s", mode);
		return;
	}
	static const char *const activity[] = { "active", "waiting", "inactive" };
	const char *act = v->activity <= GPS_CLI_INACTIVE ? activity[v->activity] : "inactive";
	char fix[32];

	if (v->has_fix) {
		char age[12];

		gps_cli_duration(age, sizeof(age), v->fix_age_s);
		snprintf(fix, sizeof(fix), "%u sats %s ago", (unsigned)v->sats, age);
	} else {
		snprintf(fix, sizeof(fix), "no fix");
	}
	snprintf(out, cap, "> on, %s, %s, %s", act, fix, mode);
}

void gps_cli_fix(char *out, size_t cap, const struct gps_cli_view *v)
{
	char acq[16] = "";

	if (v->on && v->activity == GPS_CLI_ACTIVE) {
		char t[12];

		gps_cli_duration(t, sizeof(t), v->acq_s);
		snprintf(acq, sizeof(acq), "acq %s", t);
	}
	if (!v->has_fix) {
		snprintf(out, cap, "> No fix%s%s", acq[0] ? ", " : "", acq);
		return;
	}
	char pos[40];

	snprintf(pos, sizeof(pos), "%.6f, %.6f", v->lat, v->lon);
	if (v->fix_live) {
		snprintf(out, cap, "> Fix (%u sat): %s", (unsigned)v->sats, pos);
	} else {
		char age[12];

		gps_cli_duration(age, sizeof(age), v->fix_age_s);
		snprintf(out, cap, "> Fix %s ago (%u sat)%s%s: %s", age, (unsigned)v->sats,
			 acq[0] ? ", " : "", acq, pos);
	}
}

void gps_cli_fix_stats(char *out, size_t cap, uint32_t last_ttff_ms, uint32_t max_ttff_ms,
		       uint32_t ok, uint32_t miss)
{
	char last[12] = "-";
	char max[12] = "-";

	if (max_ttff_ms > 0) {
		gps_cli_duration(last, sizeof(last), (uint32_t)(((uint64_t)last_ttff_ms + 500) / 1000));
		gps_cli_duration(max, sizeof(max), (uint32_t)(((uint64_t)max_ttff_ms + 500) / 1000));
	}
	snprintf(out, cap, "> ttf:%s/%s ok:%lu miss:%lu", last, max,
		 (unsigned long)ok, (unsigned long)miss);
}
