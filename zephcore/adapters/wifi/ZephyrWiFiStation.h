/*
 * SPDX-License-Identifier: MIT
 * ZephyrWiFiStation — WiFi STA connection manager: the observer, the repeater
 * uplink and the WiFi companion.
 *
 * Connects to a WPA2-PSK (or open) network, obtains a DHCP lease, syncs
 * time via SNTP (when CONFIG_SNTP), then signals g_wifi_events /
 * WIFI_READY_BIT so the MQTT publisher thread can start.  Auto-reconnects on
 * link loss.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Event object and bit shared with ZephyrMQTTPublisher so it can wait
 * for WiFi+SNTP to be ready before connecting to the broker.
 */
extern struct k_event g_wifi_events;
#define WIFI_READY_BIT   BIT(0)   /* WiFi connected + DHCP + SNTP done */
#define WIFI_RECONNECT_BIT BIT(1) /* Internal: trigger reconnect */

/*
 * Start the WiFi STA subsystem.
 *   ssid, psk     — the network; pointers kept (read at every connect), so they
 *                   must remain valid. psk "" = open network.
 *   time_sync_cb  — called once after SNTP with Unix timestamp; may be NULL.
 *
 * Must be called once from main() before any other wifi_station_* calls.
 */
void zc_wifi_station_start(const char *ssid, const char *psk,
			void (*time_sync_cb)(uint32_t unix_ts));

/*
 * Trigger an immediate reconnect (e.g. after credential change via CLI).
 * Safe to call from any thread.
 */
void zc_wifi_station_reconnect(void);

/* Returns true when WiFi is connected AND DHCP+SNTP have completed. */
bool zc_wifi_station_is_connected(void);

/* Returns the SSID currently connected to, or an empty string. */
const char *zc_wifi_station_ssid(void);

/* The interface's IPv4 address as text into buf; false when there is none. */
bool zc_wifi_station_ip(char *buf, size_t len);

#ifdef __cplusplus
}
#endif
