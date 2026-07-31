/*
 * SPDX-License-Identifier: MIT
 * ZephyrMQTTPublisher — MQTT client thread for the Observer role.
 *
 * Thread flow:
 *   1. Wait for WIFI_READY_BIT (WiFi + DHCP + SNTP done)
 *   2. Resolve broker hostname via zsock_getaddrinfo()
 *   3. Connect MQTT (TLS with TLS_PEER_VERIFY_NONE, or plaintext)
 *   4. Publish retained "online" status (LWT handles "offline" on disconnect)
 *   5. Poll loop: drain publish queue + mqtt_input() keepalive
 *   6. On error/disconnect: publish "offline", back off 5s, goto 1
 */

#include "ZephyrMQTTPublisher.h"
#include "ZephyrWiFiStation.h"   /* g_wifi_events, WIFI_READY_BIT, zc_wifi_station_* */
#include "observer_creds.h"

#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(mqtt_pub, CONFIG_LOG_DEFAULT_LEVEL);

/* ========== Publish queue ========== */

/* Pre-serialized message: JSON payload + 1-byte index into the two fixed
 * topic strings registered at start (see enum mqtt_pub_topic). */
#define PUB_TOPIC_MAX   160
#define PUB_PAYLOAD_MAX 1024
#define PUB_QUEUE_LEN   8

struct pub_msg {
	uint16_t payload_len;
	uint8_t  topic;   /* enum mqtt_pub_topic */
	char payload[PUB_PAYLOAD_MAX];
};

K_MSGQ_DEFINE(s_pub_queue, sizeof(struct pub_msg), PUB_QUEUE_LEN, 4);

/* Producer staging slot: callers build JSON directly into this buffer via
 * mqtt_publisher_stage() then mqtt_publisher_commit() copies it into the
 * queue. Single producer (mesh main thread) — no lock. */
static struct pub_msg s_stage;

/* Consumer drain slot: only the MQTT thread touches it (keeps a ~1 KB
 * struct off the thread stack). */
static struct pub_msg s_drain;

/* ========== Module state ========== */

static const struct ObserverCreds *s_creds;
static char s_client_id[64];
static char s_status_topic[PUB_TOPIC_MAX];
static char s_packets_topic[PUB_TOPIC_MAX];

static volatile bool s_connected;
static void (*s_connect_cb)(void);

/* Event bit to trigger reconnect from external callers */
#define PUB_RECONNECT_BIT BIT(0)
static K_EVENT_DEFINE(s_pub_events);

/* ========== MQTT buffers ========== */

#define MQTT_RX_BUF_SIZE 256
#define MQTT_TX_BUF_SIZE 512

static uint8_t s_rx_buf[MQTT_RX_BUF_SIZE];
static uint8_t s_tx_buf[MQTT_TX_BUF_SIZE];

/* ========== MQTT event callback ========== */

static void mqtt_evt_handler(struct mqtt_client *client,
			     const struct mqtt_evt *evt)
{
	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result == 0) {
			LOG_INF("MQTT connected");
			s_connected = true;
			if (s_connect_cb) {
				s_connect_cb();
			}
		} else {
			/* Broker return codes: 1=bad proto, 2=id rejected, 3=server unavail,
			 * 4=bad user/pass, 5=not authorized */
			LOG_WRN("MQTT CONNACK rejected: return_code=%d",
				evt->param.connack.return_code);
		}
		break;
	case MQTT_EVT_DISCONNECT:
		LOG_INF("MQTT disconnected");
		s_connected = false;
		break;
	case MQTT_EVT_PUBLISH:
		/* Observer never subscribes — ignore incoming publishes */
		break;
	case MQTT_EVT_PUBACK:
	case MQTT_EVT_PINGRESP:
		break;
	default:
		LOG_DBG("MQTT evt %d", evt->type);
		break;
	}
}

/* ========== Publish helpers ========== */

static uint16_t s_pkt_id;

static int do_publish(struct mqtt_client *client,
		      const char *topic, const char *payload, int payload_len,
		      bool retain)
{
	struct mqtt_publish_param p = {
		.message = {
			.topic = {
				.topic = {
					.utf8 = (const uint8_t *)topic,
					.size = strlen(topic),
				},
				.qos = MQTT_QOS_0_AT_MOST_ONCE,
			},
			.payload = {
				.data = (uint8_t *)payload,
				.len  = (uint32_t)payload_len,
			},
		},
		.message_id  = ++s_pkt_id,
		.dup_flag    = 0,
		.retain_flag = retain ? 1 : 0,
	};
	return mqtt_publish(client, &p);
}

static void publish_status(struct mqtt_client *client, bool online)
{
	static const char online_json[]  = "{\"status\":\"online\"}";
	static const char offline_json[] = "{\"status\":\"offline\"}";
	const char *msg = online ? online_json : offline_json;
	do_publish(client, s_status_topic, msg, strlen(msg), true);
}

/* ========== DNS resolution ========== */

static int resolve_host(const char *host, uint16_t port,
			struct sockaddr_in *out_addr)
{
	struct zsock_addrinfo hints = {
		.ai_family   = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct zsock_addrinfo *result;

	char port_str[8];
	snprintf(port_str, sizeof(port_str), "%u", port);

	int rc = zsock_getaddrinfo(host, port_str, &hints, &result);
	if (rc != 0) {
		LOG_ERR("DNS lookup failed for %s: %d", host, rc);
		return -EHOSTUNREACH;
	}

	memcpy(out_addr, result->ai_addr, sizeof(*out_addr));
	zsock_freeaddrinfo(result);
	LOG_INF("Resolved %s → port %u", host, port);
	return 0;
}

/* ========== Client setup ========== */

static struct mqtt_client s_client;
static struct sockaddr_in s_broker_addr;

/* LWT strings (must outlive mqtt_connect()) */
static const char s_lwt_offline[] = "{\"status\":\"offline\"}";
static struct mqtt_topic    s_lwt_topic;
static struct mqtt_utf8     s_lwt_msg;
static struct mqtt_utf8     s_mqtt_user;
static struct mqtt_utf8     s_mqtt_pass;
static struct mqtt_utf8     s_mqtt_client_id;

static int setup_client(void)
{
	mqtt_client_init(&s_client);

	/* Client ID */
	s_mqtt_client_id.utf8 = (const uint8_t *)s_client_id;
	s_mqtt_client_id.size = strlen(s_client_id);
	s_client.client_id    = s_mqtt_client_id;

	/* Broker address (already resolved) */
	s_client.broker = &s_broker_addr;

	/* RX/TX buffers */
	s_client.rx_buf      = s_rx_buf;
	s_client.rx_buf_size = sizeof(s_rx_buf);
	s_client.tx_buf      = s_tx_buf;
	s_client.tx_buf_size = sizeof(s_tx_buf);

	/* Keepalive (matches CONFIG_MQTT_KEEPALIVE=60) */
	s_client.keepalive    = 60;
	s_client.clean_session = 1;

	/* Credentials */
	if (s_creds->mqtt_user[0] != '\0') {
		s_mqtt_user.utf8  = (const uint8_t *)s_creds->mqtt_user;
		s_mqtt_user.size  = strlen(s_creds->mqtt_user);
		s_client.user_name = &s_mqtt_user;
	}
	if (s_creds->mqtt_password[0] != '\0') {
		s_mqtt_pass.utf8  = (const uint8_t *)s_creds->mqtt_password;
		s_mqtt_pass.size  = strlen(s_creds->mqtt_password);
		s_client.password = &s_mqtt_pass;
	}

	/* LWT — retained "offline" published if connection drops */
	s_lwt_topic.topic.utf8  = (const uint8_t *)s_status_topic;
	s_lwt_topic.topic.size  = strlen(s_status_topic);
	s_lwt_topic.qos         = MQTT_QOS_0_AT_MOST_ONCE;
	s_lwt_msg.utf8          = (const uint8_t *)s_lwt_offline;
	s_lwt_msg.size          = strlen(s_lwt_offline);
	s_client.will_topic     = &s_lwt_topic;
	s_client.will_message   = &s_lwt_msg;
	s_client.will_retain    = 1;

	/* Transport */
	if (s_creds->mqtt_tls) {
		s_client.transport.type = MQTT_TRANSPORT_SECURE;
		struct mqtt_sec_config *tls = &s_client.transport.tls.config;
		/* No certificate pinning — encrypted but no CA verification.
		 * This avoids cert rotation issues (e.g. Let's Encrypt renewals). */
		tls->peer_verify  = TLS_PEER_VERIFY_NONE;
		tls->cipher_count = 0;
		tls->cipher_list  = NULL;
		tls->sec_tag_count = 0;
		tls->sec_tag_list  = NULL;
		tls->hostname      = s_creds->mqtt_host;  /* SNI */
	} else {
		s_client.transport.type = MQTT_TRANSPORT_NON_SECURE;
	}

	s_client.evt_cb = mqtt_evt_handler;

	return 0;
}

/* ========== MQTT poll loop ========== */

static int get_sock(void)
{
	if (s_creds->mqtt_tls) {
		return s_client.transport.tls.sock;
	}
	return s_client.transport.tcp.sock;
}

/* Wait for socket to be readable, with a timeout_ms cap.
 * Returns > 0 if data available, 0 if timeout, < 0 on error/hangup. */
static int poll_socket(int sock, int timeout_ms)
{
	struct zsock_pollfd pfd = {
		.fd     = sock,
		.events = ZSOCK_POLLIN,
	};
	int rc = zsock_poll(&pfd, 1, timeout_ms);
	if (rc < 0) {
		return -errno;
	}
	if (pfd.revents & (ZSOCK_POLLHUP | ZSOCK_POLLERR)) {
		return -ECONNRESET;
	}
	return rc; /* 0 = timeout, 1 = data ready */
}

static void run_poll_loop(void)
{
	int sock = get_sock();

	for (;;) {
		/* Check for external reconnect request */
		if (k_event_test(&s_pub_events, PUB_RECONNECT_BIT)) {
			k_event_clear(&s_pub_events, PUB_RECONNECT_BIT);
			LOG_INF("Reconnect requested");
			break;
		}

		/* Drain publish queue */
		while (k_msgq_get(&s_pub_queue, &s_drain, K_NO_WAIT) == 0) {
			const char *topic = (s_drain.topic == MQTT_PUB_TOPIC_PACKETS)
					    ? s_packets_topic : s_status_topic;
			int rc = do_publish(&s_client, topic,
					    s_drain.payload, s_drain.payload_len, false);
			if (rc < 0) {
				LOG_WRN("Publish failed: %d", rc);
				s_connected = false;
				return;
			}
		}

		/* Wait up to 100ms for incoming data (keepalive PINGRESP, etc.) */
		int pr = poll_socket(sock, 100);
		if (pr < 0) {
			LOG_WRN("Socket error in poll loop: %d", pr);
			s_connected = false;
			return;
		}
		if (pr > 0) {
			int rc = mqtt_input(&s_client);
			if (rc < 0 && rc != -EAGAIN && rc != -EWOULDBLOCK) {
				LOG_WRN("mqtt_input error: %d", rc);
				s_connected = false;
				return;
			}
		}

		/* Send keepalive ping if needed */
		int rc = mqtt_live(&s_client);
		if (rc < 0 && rc != -EAGAIN) {
			LOG_WRN("mqtt_live error: %d", rc);
			s_connected = false;
			return;
		}

		if (!s_connected) {
			return;
		}
	}
}

/* ========== Publisher thread ========== */

#define MQTT_THREAD_STACK_SIZE 12288
#define MQTT_THREAD_PRIORITY   5

/* Escalating retry backoff: DNS/connect/CONNACK failures double the delay up
 * to the cap (a broker rejecting credentials won't fix itself in 10 s, and
 * every TLS retry costs a full handshake). Reset on successful CONNACK. */
#define RETRY_BACKOFF_MIN_S 5
#define RETRY_BACKOFF_MAX_S 300

static uint32_t s_retry_backoff_s = RETRY_BACKOFF_MIN_S;

static void backoff_sleep(void)
{
	LOG_INF("MQTT retry in %u s", s_retry_backoff_s);
	k_sleep(K_SECONDS(s_retry_backoff_s));
	s_retry_backoff_s = MIN(s_retry_backoff_s * 2, RETRY_BACKOFF_MAX_S);
}

static void mqtt_thread_fn(void *p1, void *p2, void *p3);
K_THREAD_DEFINE(mqtt_pub_thread, MQTT_THREAD_STACK_SIZE,
		mqtt_thread_fn, NULL, NULL, NULL,
		MQTT_THREAD_PRIORITY, 0, 0);

static void mqtt_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	LOG_INF("MQTT publisher thread started");

	for (;;) {
		s_connected = false;

		/* Wait until WiFi is ready (blocks indefinitely) */
		LOG_INF("Waiting for WiFi ready...");
		k_event_wait(&g_wifi_events, WIFI_READY_BIT, false, K_FOREVER);

		if (!s_creds || s_creds->mqtt_host[0] == '\0') {
			LOG_WRN("No MQTT host configured — set with: set mqtt.host <hostname>");
			k_sleep(K_SECONDS(10));
			continue;
		}

		/* Clear reconnect flag before connecting */
		k_event_clear(&s_pub_events, PUB_RECONNECT_BIT);

		/* Resolve broker hostname */
		int rc = resolve_host(s_creds->mqtt_host, s_creds->mqtt_port,
				      &s_broker_addr);
		if (rc < 0) {
			LOG_WRN("DNS failed");
			backoff_sleep();
			continue;
		}

		/* Configure MQTT client */
		setup_client();

		/* Connect */
		LOG_INF("Connecting to MQTT broker %s:%u (TLS=%u)",
			s_creds->mqtt_host, s_creds->mqtt_port,
			(unsigned)s_creds->mqtt_tls);

		rc = mqtt_connect(&s_client);
		if (rc < 0) {
			LOG_WRN("mqtt_connect failed: %d", rc);
			backoff_sleep();
			continue;
		}

		/* Wait for CONNACK — poll up to 5s total, 500ms per iteration */
		{
			int sock = get_sock();
			for (int i = 0; i < 10 && !s_connected; i++) {
				int pr = poll_socket(sock, 500);
				if (pr < 0) {
					LOG_WRN("CONNACK wait: socket error %d", pr);
					break;
				}
				if (pr > 0) {
					int rc2 = mqtt_input(&s_client);
					if (rc2 < 0 && rc2 != -EAGAIN &&
					    rc2 != -EWOULDBLOCK) {
						LOG_WRN("CONNACK wait: mqtt_input error %d", rc2);
						break;
					}
				}
			}
		}

		if (!s_connected) {
			LOG_WRN("CONNACK not accepted (broker rejected or no response)");
			mqtt_disconnect(&s_client, NULL);
			backoff_sleep();
			continue;
		}

		/* Connected — reset the failure backoff */
		s_retry_backoff_s = RETRY_BACKOFF_MIN_S;

		/* Publish retained "online" status */
		publish_status(&s_client, true);

		/* Run poll loop until disconnect or reconnect request */
		run_poll_loop();

		/* Publish "offline" before closing (best-effort) */
		if (s_connected) {
			publish_status(&s_client, false);
		}
		mqtt_disconnect(&s_client, NULL);

		LOG_INF("MQTT session ended");
		backoff_sleep();
	}
}

/* ========== Public API ========== */

void mqtt_publisher_start(const struct ObserverCreds *creds,
			  const char *client_id,
			  const char *status_topic,
			  const char *packets_topic)
{
	s_creds = creds;
	strncpy(s_client_id,     client_id,     sizeof(s_client_id) - 1);
	strncpy(s_status_topic,  status_topic,  sizeof(s_status_topic) - 1);
	strncpy(s_packets_topic, packets_topic, sizeof(s_packets_topic) - 1);
	s_client_id[sizeof(s_client_id) - 1]         = '\0';
	s_status_topic[sizeof(s_status_topic) - 1]   = '\0';
	s_packets_topic[sizeof(s_packets_topic) - 1] = '\0';

	/* Thread is already created by K_THREAD_DEFINE — nothing more to do. */
	LOG_INF("MQTT publisher ready (client_id=%s)", s_client_id);
}

char *mqtt_publisher_stage(size_t *size)
{
	if (size) {
		*size = sizeof(s_stage.payload);
	}
	return s_stage.payload;
}

void mqtt_publisher_commit(enum mqtt_pub_topic topic, int payload_len)
{
	if (payload_len <= 0) {
		return;
	}
	if (payload_len >= (int)sizeof(s_stage.payload)) {
		payload_len = (int)sizeof(s_stage.payload) - 1;
	}
	s_stage.payload[payload_len] = '\0';
	s_stage.payload_len = (uint16_t)payload_len;
	s_stage.topic = (uint8_t)topic;

	if (k_msgq_put(&s_pub_queue, &s_stage, K_NO_WAIT) != 0) {
		LOG_WRN("Publish queue full — message dropped");
	}
}

bool mqtt_publisher_is_connected(void)
{
	return s_connected;
}

void mqtt_publisher_reconnect(void)
{
	k_event_post(&s_pub_events, PUB_RECONNECT_BIT);
}

void mqtt_publisher_set_connect_cb(void (*cb)(void))
{
	s_connect_cb = cb;
}
