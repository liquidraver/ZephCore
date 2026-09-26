/*
 * SPDX-License-Identifier: MIT
 * Uplink credentials: WiFi, MQTT broker and location for the observer and the
 * repeater uplink. Read by the WiFi station and the MQTT publisher; loaded,
 * saved and edited by the roles (app/observer_creds.h).
 *
 * The struct is persisted as raw bytes (/lfs/repeater/obs_creds): keep the
 * field order and sizes.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ObserverCreds {
	char wifi_ssid[64];      /* WiFi network name */
	char wifi_psk[64];       /* WiFi password (empty = open network) */
	char mqtt_host[128];     /* MQTT broker hostname, e.g. "abydos.hu" */
	uint16_t mqtt_port;      /* MQTT broker port, e.g. 8883 */
	uint8_t  mqtt_tls;       /* 1 = TLS (no cert verify), 0 = plaintext */
	char mqtt_user[64];      /* MQTT username */
	char mqtt_password[64];  /* MQTT password */
	char mqtt_iata[8];       /* IATA location code, e.g. "BUD", "BTS", "VIE" */
	uint8_t  _reserved[1];   /* alignment / future use */
	int32_t  lat_e6;          /* latitude  × 1 000 000 (e.g. 47497900 = 47.497900°N) */
	int32_t  lon_e6;          /* longitude × 1 000 000 (e.g. 19040200 = 19.040200°E) */
};

/* Apply sensible defaults to a freshly-zeroed creds struct. */
static inline void observer_creds_init(struct ObserverCreds *creds)
{
	creds->mqtt_port = 8883;
	creds->mqtt_tls  = 1;
}

#ifdef __cplusplus
}
#endif
