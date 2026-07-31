/*
 * SPDX-License-Identifier: MIT
 * meshcoretomqtt-compatible JSON builders.
 *
 * Shared by ObserverMesh (mesh:: namespace) and the RepeaterMesh MQTT uplink
 * (global namespace).  Header-only `static inline` so both translation units
 * can include it without a shared .cpp / CMake change.  The packet and status
 * JSON shapes were byte-identical across the two; only the field *values*
 * differ, so callers fill the param struct and the format lives here once.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>

struct MeshcorePacketJson {
	const char *origin;     /* node name */
	const char *origin_id;  /* pubkey hex */
	uint32_t    epoch;      /* unix seconds (timestamp/time/date fields) */
	int         raw_len;
	unsigned    packet_type;
	const char *route;      /* "D" or "F" */
	unsigned    payload_len;
	const char *raw_hex;
	int         snr;
	int         rssi;
	int         score_x1000;
	const char *hash_hex;
};

static inline int meshcore_build_packet_json(char *out, size_t out_size,
					     const struct MeshcorePacketJson *p)
{
	struct tm tm_now;
	time_t t = (time_t)p->epoch;
	gmtime_r(&t, &tm_now);

	char ts_buf[48], time_buf[12], date_buf[32];
	snprintf(ts_buf, sizeof(ts_buf), "%04d-%02d-%02dT%02d:%02d:%02d.000000",
		 tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
		 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
	snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02d",
		 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
	snprintf(date_buf, sizeof(date_buf), "%d/%d/%04d",
		 tm_now.tm_mday, tm_now.tm_mon + 1, tm_now.tm_year + 1900);

	return snprintf(out, out_size,
		"{"
		"\"type\":\"PACKET\","
		"\"origin\":\"%s\","
		"\"origin_id\":\"%s\","
		"\"timestamp\":\"%s\","
		"\"direction\":\"rx\","
		"\"time\":\"%s\","
		"\"date\":\"%s\","
		"\"len\":\"%d\","
		"\"packet_type\":\"%u\","
		"\"route\":\"%s\","
		"\"payload_len\":\"%u\","
		"\"raw\":\"%s\","
		"\"SNR\":\"%d\","
		"\"RSSI\":\"%d\","
		"\"score\":\"%d\","
		"\"hash\":\"%s\""
		"}",
		p->origin, p->origin_id, ts_buf, time_buf, date_buf,
		p->raw_len, p->packet_type, p->route, p->payload_len,
		p->raw_hex, p->snr, p->rssi, p->score_x1000, p->hash_hex);
}

struct MeshcoreStatusJson {
	const char *status;
	uint32_t    epoch;
	const char *origin;
	const char *origin_id;
	const char *radio;
	const char *model;
	const char *firmware_version;
	unsigned    battery_mv;
	unsigned    uptime_secs;
	unsigned    debug_flags;
	unsigned    queue_len;
	int         noise_floor;
	unsigned    tx_air_secs;
	unsigned    rx_air_secs;
	unsigned    recv_errors;
};

static inline int meshcore_build_status_json(char *out, size_t out_size,
					     const struct MeshcoreStatusJson *s)
{
	struct tm tm_now;
	time_t t = (time_t)s->epoch;
	gmtime_r(&t, &tm_now);

	char ts_buf[48];
	snprintf(ts_buf, sizeof(ts_buf), "%04d-%02d-%02dT%02d:%02d:%02d.000000",
		 tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
		 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);

	return snprintf(out, out_size,
		"{"
		"\"status\":\"%s\","
		"\"timestamp\":\"%s\","
		"\"origin\":\"%s\","
		"\"origin_id\":\"%s\","
		"\"radio\":\"%s\","
		"\"model\":\"%s\","
		"\"firmware_version\":\"%s\","
		"\"client_version\":\"zephcoretomqtt/1.1\","
		"\"stats\":{"
			"\"battery_mv\":%u,"
			"\"uptime_secs\":%u,"
			"\"debug_flags\":%u,"
			"\"queue_len\":%u,"
			"\"noise_floor\":%d,"
			"\"tx_air_secs\":%u,"
			"\"rx_air_secs\":%u,"
			"\"recv_errors\":%u"
		"}"
		"}",
		s->status, ts_buf, s->origin, s->origin_id, s->radio,
		s->model, s->firmware_version,
		s->battery_mv, s->uptime_secs, s->debug_flags, s->queue_len,
		s->noise_floor, s->tx_air_secs, s->rx_air_secs, s->recv_errors);
}
