/*
 * SPDX-License-Identifier: MIT
 * ZephyrWiFiStation — WiFi STA connection manager for the Observer role.
 *
 * Flow:
 *   wifi_station_start() → issues NET_REQUEST_WIFI_CONNECT
 *   NET_EVENT_WIFI_CONNECT_RESULT      → link up, wait for DHCP
 *   NET_EVENT_IPV4_DHCP_BOUND          → IP assigned → run SNTP → signal WIFI_READY_BIT
 *   NET_EVENT_WIFI_DISCONNECT_RESULT   → clear WIFI_READY_BIT, schedule reconnect in 5s
 */

#include "ZephyrWiFiStation.h"
#include "observer_creds.h"
#include "../../helpers/pm_sleep_guard.h"

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/sntp.h>
#include <zephyr/logging/log.h>

#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(wifi_station, CONFIG_LOG_DEFAULT_LEVEL);

/* ========== Shared event object ========== */

K_EVENT_DEFINE(g_wifi_events);

/* ========== Module state ========== */

static const struct ObserverCreds *s_creds;
static void (*s_time_sync_cb)(uint32_t unix_ts);

/* Protect link-up vs. SNTP state */
static volatile bool s_wifi_link_up;   /* WiFi associate event received */
static volatile bool s_wifi_ready;     /* DHCP + SNTP done */

/* ========== Reconnect work ========== */

static void connect_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(connect_work, connect_work_fn);

/* ========== net_mgmt callbacks ========== */

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void do_sntp_and_signal(void)
{
	struct sntp_time ts;
	int rc = sntp_simple("pool.ntp.org", 8000, &ts);
	if (rc == 0) {
		LOG_INF("SNTP synced: %llu", (unsigned long long)ts.seconds);
		if (s_time_sync_cb) {
			s_time_sync_cb((uint32_t)ts.seconds);
		}
	} else {
		LOG_WRN("SNTP failed (rc=%d) — continuing without time sync", rc);
	}

	s_wifi_ready = true;
	k_event_post(&g_wifi_events, WIFI_READY_BIT);
	LOG_INF("WiFi ready (DHCP+SNTP done)");
}

/* Called when DHCP has bound an address */
static void ipv4_event_handler(struct net_mgmt_event_callback *cb,
			       uint64_t event, struct net_if *iface)
{
	ARG_UNUSED(iface);
	if (event == NET_EVENT_IPV4_DHCP_BOUND) {
		LOG_INF("DHCP bound — starting SNTP sync");
		/* Run SNTP — this is called from the net_mgmt work queue.
		 * sntp_simple() may block for up to 8s; net_mgmt stack must be
		 * large enough (CONFIG_NET_MGMT_EVENT_STACK_SIZE >= 3072). */
		do_sntp_and_signal();
	}
}

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
			       uint64_t event, struct net_if *iface)
{
	switch (event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
#ifdef CONFIG_NET_MGMT_EVENT_INFO
		const struct wifi_status *status =
			(const struct wifi_status *)cb->info;
		bool success = (status == NULL ||
				status->conn_status == WIFI_STATUS_CONN_SUCCESS);
		int reason = status ? (int)status->conn_status : 0;
#else
		bool success = true;
		int reason = 0;
#endif
		if (success) {
			LOG_INF("WiFi link up (SSID: %s)", s_creds ? s_creds->wifi_ssid : "?");
			s_wifi_link_up = true;
			/* Disable power save — WIFI_PS_MIN_MODEM (default) sleeps between
			 * DTIM beacons, which delays ACKs and causes the MQTT broker's TCP
			 * stack to retransmit and eventually RST the connection (-ECONNRESET
			 * in the poll loop). WIFI_PS_DISABLED keeps the radio always awake. */
			struct wifi_ps_params ps = {
				.enabled = WIFI_PS_DISABLED,
				.type    = WIFI_PS_PARAM_STATE,
			};
			if (net_mgmt(NET_REQUEST_WIFI_PS, iface, &ps, sizeof(ps)) < 0) {
				LOG_WRN("Failed to disable WiFi power save");
			} else {
				LOG_INF("WiFi power save disabled");
			}
			/* DHCP will fire ipv4_event_handler when lease is obtained */
		} else {
			LOG_WRN("WiFi connect failed (reason=%d) — retry in 10s", reason);
			k_work_reschedule(&connect_work, K_SECONDS(10));
		}
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		LOG_INF("WiFi disconnected");
		s_wifi_link_up = false;
		s_wifi_ready   = false;
		k_event_clear(&g_wifi_events, WIFI_READY_BIT);
		/* Reconnect after 5s backoff */
		k_work_reschedule(&connect_work, K_SECONDS(5));
		break;
	default:
		break;
	}
}

/* ========== Connect work ========== */

static void connect_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!s_creds || s_creds->wifi_ssid[0] == '\0') {
		LOG_WRN("No WiFi SSID configured — set with: set wifi.ssid <name>");
		return;
	}

	struct net_if *iface = net_if_get_default();
	if (!iface) {
		LOG_ERR("No network interface");
		return;
	}

	struct wifi_connect_req_params params = {
		.ssid        = (const uint8_t *)s_creds->wifi_ssid,
		.ssid_length = (uint8_t)strlen(s_creds->wifi_ssid),
		.channel     = WIFI_CHANNEL_ANY,
		.band        = WIFI_FREQ_BAND_UNKNOWN,
		.mfp         = WIFI_MFP_OPTIONAL,
	};

	if (s_creds->wifi_psk[0] != '\0') {
		params.psk        = (const uint8_t *)s_creds->wifi_psk;
		params.psk_length = (uint8_t)strlen(s_creds->wifi_psk);
		params.security   = WIFI_SECURITY_TYPE_PSK;
	} else {
		params.security = WIFI_SECURITY_TYPE_NONE;
	}

	LOG_INF("Connecting to WiFi: %s", s_creds->wifi_ssid);
	int rc = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	if (rc < 0 && rc != -EALREADY) {
		LOG_ERR("WiFi connect request failed: %d — retry in 10s", rc);
		k_work_reschedule(&connect_work, K_SECONDS(10));
	}
}

/* ========== Public API ========== */

void zc_wifi_station_start(const struct ObserverCreds *creds,
			void (*time_sync_cb)(uint32_t unix_ts))
{
	s_creds        = creds;
	s_time_sync_cb = time_sync_cb;
	s_wifi_link_up = false;
	s_wifi_ready   = false;

	/* Permanent, and intentionally never released: there is no
	 * zc_wifi_station_stop() — once an uplink or observer node associates it
	 * stays associated for the life of the boot. Light sleep would drop the
	 * association (nothing here coordinates with the WiFi modem's own sleep),
	 * so a node built with WiFi simply does not light-sleep. */
	zc_pm_block_sleep();

	/* Register WiFi event callback */
	net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT |
				     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);

	/* Register IPv4 DHCP callback */
	net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler,
				     NET_EVENT_IPV4_DHCP_BOUND);
	net_mgmt_add_event_callback(&ipv4_cb);

	/* Trigger first connect attempt */
	k_work_schedule(&connect_work, K_MSEC(500));
}

void zc_wifi_station_reconnect(void)
{
	/* Attempt to disconnect first; ignore errors (may already be disconnected) */
	struct net_if *iface = net_if_get_default();
	if (iface) {
		net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
	}

	/* Clear state and immediately reschedule connect */
	s_wifi_link_up = false;
	s_wifi_ready   = false;
	k_event_clear(&g_wifi_events, WIFI_READY_BIT);
	k_work_reschedule(&connect_work, K_MSEC(200));
}

bool zc_wifi_station_is_connected(void)
{
	return s_wifi_ready;
}

const char *zc_wifi_station_ssid(void)
{
	return (s_creds && s_creds->wifi_ssid[0]) ? s_creds->wifi_ssid : "";
}
