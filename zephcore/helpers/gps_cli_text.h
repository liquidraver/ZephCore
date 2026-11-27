/*
 * SPDX-License-Identifier: MIT
 * The `gps` CLI's parsing and replies, in upstream's form (CommonGpsPrefs,
 * upstream 6c9600e1 and after). No Zephyr dependency, host-tested.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/* One unit, rounded down: "45s", "12m", "3h". */
void gps_cli_duration(char *out, size_t cap, uint32_t seconds);

/* `gps period <value>`: digits, then optional s, m, h or d. Saturates at
 * UINT32_MAX seconds. False for anything else (empty, sign, fraction, junk). */
bool gps_cli_parse_period(const char *arg, uint32_t *seconds);

/* What the GPS manager reports, as the replies need it. */
struct gps_cli_view {
	bool on;             /* requested on (prefs, or a `gps once` window) */
	uint8_t activity;    /* GPS_CLI_ACTIVE, GPS_CLI_WAITING, GPS_CLI_INACTIVE */
	bool has_fix;        /* a fix since boot */
	bool fix_live;       /* that fix is current (acquiring, last sentence just now) */
	uint16_t sats;       /* satellites of the last fix */
	uint32_t fix_age_s;
	uint32_t acq_s;      /* seconds into the acquire window, while acquiring */
	double lat, lon;
	uint32_t period_s;   /* 0 = continuous */
};

#define GPS_CLI_ACTIVE    0
#define GPS_CLI_WAITING   1
#define GPS_CLI_INACTIVE  2

/* `gps`: "> on, active, 7 sats 3m ago, periodic (300s)" or "> off, continuous". */
void gps_cli_status(char *out, size_t cap, const struct gps_cli_view *v);

/* `gps fix`: "> Fix (7 sat): lat, lon", "> Fix 3m ago (7 sat), acq 20s: lat, lon",
 * "> No fix, acq 20s". */
void gps_cli_fix(char *out, size_t cap, const struct gps_cli_view *v);

/* `gps fix_stats`: "> ttf:12s/3m ok:4 miss:0", "-" before any fix. */
void gps_cli_fix_stats(char *out, size_t cap, uint32_t last_ttff_ms, uint32_t max_ttff_ms,
		       uint32_t ok, uint32_t miss);
