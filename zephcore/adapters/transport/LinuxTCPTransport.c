/*
 * SPDX-License-Identifier: MIT
 * ZephCore TCP companion transport — drop-in replacement for ZephyrBLE.
 *
 * Implements the exact zephcore_ble_* C API from adapters/ble/ZephyrBLE.h
 * over a TCP socket.
 *
 * Wire format: MeshCore SerialWifiInterface framing (ESP32 Arduino reference
 * implementation: src/helpers/esp32/SerialWifiInterface.cpp):
 *
 *   App → Node:  [ '<' (0x3C) | length_LSB | length_MSB | payload... ]
 *   Node → App:  [ '>' (0x3E) | length_LSB | length_MSB | payload... ]
 *
 * 3-byte header: 1-byte direction marker + 2-byte LE payload length.
 * Frames with type != '<' are silently skipped (matches Arduino behavior).
 *
 * A single client at a time is supported (one BT_MAX_CONN=1 analogue).
 *
 * Architecture:
 *   - Listen thread accepts a client, then loops reading frames and
 *     firing on_rx_frame (which queues to ble_recv_queue via the callback).
 *   - TX is a work-queue item driven by zephcore_ble_kick_tx(), draining
 *     ble_send_queue with zsock_send() until empty or the socket dies.
 *
 * Used only when CONFIG_ZEPHCORE_TRANSPORT_TCP=y and CONFIG_BT=n
 * (the linux_common.conf preset turns CONFIG_BT off).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>

#include "ZephyrBLE.h"
#include "companion_framing.h"

LOG_MODULE_REGISTER(linux_tcp_transport, LOG_LEVEL_INF);

#define FRAME_QUEUE_SIZE CONFIG_ZEPHCORE_BLE_QUEUE_SIZE
#define LISTEN_PORT      CONFIG_ZEPHCORE_LINUX_TCP_PORT
#define LISTEN_BACKLOG   1

/* TX congestion control (mirrors ZephyrBLE.cpp). Lossy (>= 0x80) vs lossless
 * (< 0x80) classification is shared via companion_is_lossless_protocol_frame()
 * in companion_framing.h. */
#define TX_OVERFLOW_RETRY_MS 250

/* Bound on a single blocking send. tx_drain_work_fn runs on the system
 * workqueue, so an unbounded zsock_send() against a peer that has stopped
 * reading would hang the whole queue (incl. overflow_retry_work). A peer
 * that can't accept one frame within this window is wedged — close it and
 * let the app reconnect rather than stall the node. */
#define TX_SEND_TIMEOUT_SEC  2

/* struct frame + framing constants + lossless classification are shared via
 * companion_framing.h (included above). */

K_MSGQ_DEFINE(ble_send_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ble_recv_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);

static const struct ble_callbacks *transport_cbs;
static enum zephcore_iface active_iface = ZEPHCORE_IFACE_NONE;

static bool transport_enabled;
static uint32_t passkey;

/* Socket state */
static int listen_fd = -1;
static int client_fd = -1;
static struct k_mutex sock_mu;

/* Listen + RX thread */
static K_KERNEL_STACK_DEFINE(listen_thread_stack, 4096);
static struct k_thread listen_thread;
static bool listen_thread_started;

/* TX work — drains ble_send_queue */
static void tx_drain_work_fn(struct k_work *work);
static K_WORK_DEFINE(tx_drain_work, tx_drain_work_fn);

/* TX congestion control — ported from ZephyrBLE.cpp so the TCP transport
 * back-pressures instead of dropping frames when the send queue fills (e.g.
 * an advert/message push colliding with a channel/contact import burst).
 *
 *   1. Queue full  → set sticky tx_congested; senders check
 *      zephcore_ble_is_congested() and hold off.
 *   2. Lossless protocol frame (data[0] < 0x80) → return 0 so the caller
 *      retries — never dropped.
 *   3. Non-lossless push → stash one frame in overflow, retried every 250ms.
 *   4. Drain below 1/3 (or empty) → clear congestion (hysteresis).
 *   5. Disconnect → reset everything.
 */
static bool tx_congested;
static struct frame overflow_frame;
static bool overflow_pending;

static void overflow_retry_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(overflow_retry_work, overflow_retry_work_fn);

static void close_client_locked(void)
{
	if (client_fd >= 0) {
		zsock_close(client_fd);
		client_fd = -1;
	}

	/* Reset TX congestion state on disconnect (mirrors ZephyrBLE). */
	overflow_pending = false;
	tx_congested = false;
	k_work_cancel_delayable(&overflow_retry_work);
}

/* Write `len` bytes, retrying on partial sends. Returns 0 or -errno. */
static int sock_send_all(int fd, const uint8_t *buf, size_t len)
{
	size_t off = 0;

	while (off < len) {
		ssize_t n = zsock_send(fd, buf + off, len - off, 0);

		if (n < 0) {
			if (errno == EINTR) {
				continue;
			}
			return -errno;
		}
		if (n == 0) {
			return -ECONNRESET;
		}
		off += (size_t)n;
	}
	return 0;
}

/* Read exactly `len` bytes (blocking). Returns 0 on success, -errno on
 * error, -ECONNRESET on clean EOF. */
static int sock_recv_all(int fd, uint8_t *buf, size_t len)
{
	size_t off = 0;

	while (off < len) {
		ssize_t n = zsock_recv(fd, buf + off, len - off, 0);

		if (n < 0) {
			if (errno == EINTR) {
				continue;
			}
			return -errno;
		}
		if (n == 0) {
			return -ECONNRESET;
		}
		off += (size_t)n;
	}
	return 0;
}

static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	struct frame f;

	while (k_msgq_get(&ble_send_queue, &f, K_NO_WAIT) == 0) {
		/* Clear congestion at low water mark (1/3) — hysteresis vs full. */
		if (tx_congested &&
		    k_msgq_num_used_get(&ble_send_queue) <= FRAME_QUEUE_SIZE / 3) {
			tx_congested = false;
			LOG_INF("tx_drain: congestion cleared (queue=%u/%u)",
				k_msgq_num_used_get(&ble_send_queue),
				(unsigned)FRAME_QUEUE_SIZE);
		}

		k_mutex_lock(&sock_mu, K_FOREVER);
		int fd = client_fd;

		if (fd < 0) {
			k_mutex_unlock(&sock_mu);
			/* No client — drop the frame. */
			LOG_DBG("tx: no client, dropping len=%u", f.len);
			continue;
		}

		/* SerialWifiInterface framing: '>' + length LE + payload */
		uint8_t hdr[3];

		hdr[0] = '>';
		hdr[1] = (uint8_t)(f.len & 0xFF);
		hdr[2] = (uint8_t)(f.len >> 8);

		int err = sock_send_all(fd, hdr, 3);

		if (err == 0) {
			err = sock_send_all(fd, f.buf, f.len);
		}
		k_mutex_unlock(&sock_mu);

		if (err != 0) {
			if (err == -EAGAIN || err == -EWOULDBLOCK) {
				LOG_WRN("tx send timeout (peer not reading), closing client");
			} else {
				LOG_WRN("tx send err=%d, closing client", err);
			}
			k_mutex_lock(&sock_mu, K_FOREVER);
			close_client_locked();
			active_iface = ZEPHCORE_IFACE_NONE;
			k_mutex_unlock(&sock_mu);

			if (transport_cbs && transport_cbs->on_disconnected) {
				transport_cbs->on_disconnected();
			}
			break;
		}
	}

	/* Queue fully drained — clear any residual congestion. */
	if (tx_congested && k_msgq_num_used_get(&ble_send_queue) == 0) {
		tx_congested = false;
		LOG_INF("tx_drain: congestion cleared (queue empty)");
	}

	if (transport_cbs && transport_cbs->on_tx_idle &&
	    k_msgq_num_used_get(&ble_send_queue) == 0) {
		transport_cbs->on_tx_idle();
	}
}

/* Re-inject the stashed overflow push once the queue has room. Mirrors
 * ZephyrBLE.cpp's overflow_retry_work_fn. */
static void overflow_retry_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!overflow_pending) {
		return;
	}

	/* Abandon overflow if the client is gone. */
	k_mutex_lock(&sock_mu, K_FOREVER);
	bool connected = (client_fd >= 0);

	k_mutex_unlock(&sock_mu);

	if (!connected) {
		overflow_pending = false;
		tx_congested = false;
		LOG_INF("overflow cleared (disconnected)");
		return;
	}

	if (k_msgq_put(&ble_send_queue, &overflow_frame, K_NO_WAIT) == 0) {
		overflow_pending = false;
		LOG_DBG("overflow frame queued hdr=0x%02x, kicking drain",
			overflow_frame.buf[0]);
		k_work_submit(&tx_drain_work);
		/* Congestion flag cleared by tx_drain at low water mark. */
	} else {
		/* Still full — retry at reduced rate. */
		LOG_DBG("overflow retry: queue still full, retry in %dms",
			TX_OVERFLOW_RETRY_MS);
		k_work_schedule(&overflow_retry_work, K_MSEC(TX_OVERFLOW_RETRY_MS));
	}
}

static void listen_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct net_sockaddr_in addr = {0};

	addr.sin_family = NET_AF_INET;
	addr.sin_addr.s_addr = 0;            /* NET_INADDR_ANY */
	addr.sin_port = net_htons(LISTEN_PORT);

	listen_fd = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	if (listen_fd < 0) {
		LOG_ERR("zsock_socket failed: %d", errno);
		return;
	}

	int one = 1;
	(void)zsock_setsockopt(listen_fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR,
				&one, sizeof(one));

	if (zsock_bind(listen_fd, (struct net_sockaddr *)&addr, sizeof(addr)) < 0) {
		LOG_ERR("bind(%d) failed: %d", LISTEN_PORT, errno);
		zsock_close(listen_fd);
		listen_fd = -1;
		return;
	}

	if (zsock_listen(listen_fd, LISTEN_BACKLOG) < 0) {
		LOG_ERR("listen failed: %d", errno);
		zsock_close(listen_fd);
		listen_fd = -1;
		return;
	}

	LOG_INF("TCP companion transport listening on :%u", (unsigned)LISTEN_PORT);

	while (true) {
		struct net_sockaddr_in caddr;
		net_socklen_t clen = sizeof(caddr);
		int fd = zsock_accept(listen_fd, (struct net_sockaddr *)&caddr, &clen);

		if (fd < 0) {
			if (errno == EINTR) {
				continue;
			}
			LOG_ERR("accept failed: %d", errno);
			k_sleep(K_MSEC(100));
			continue;
		}

		/* Bound blocking sends so a non-reading peer can't hang the
		 * system workqueue (see TX_SEND_TIMEOUT_SEC). */
		struct zsock_timeval sndtimeo = {
			.tv_sec = TX_SEND_TIMEOUT_SEC,
			.tv_usec = 0,
		};
		(void)zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_SNDTIMEO,
				       &sndtimeo, sizeof(sndtimeo));

		k_mutex_lock(&sock_mu, K_FOREVER);
		if (client_fd >= 0) {
			LOG_WRN("Second client rejected (already connected)");
			zsock_close(fd);
			k_mutex_unlock(&sock_mu);
			continue;
		}
		client_fd = fd;
		active_iface = ZEPHCORE_IFACE_BLE;
		k_mutex_unlock(&sock_mu);

		LOG_INF("TCP companion client connected");

		if (transport_cbs && transport_cbs->on_connected) {
			transport_cbs->on_connected();
		}

		/* RX loop: SerialWifiInterface framing.
		 * Each frame: ['<':1][length:2LE][payload].
		 * Frames with type != '<' are skipped (matches Arduino). */
		while (true) {
			uint8_t hdr[3];
			int err = sock_recv_all(fd, hdr, 3);

			if (err != 0) {
				break;
			}

			uint16_t flen = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);

			/* Skip frames not from app ('<'), or bad length. */
			if (hdr[0] != '<' || flen == 0 || flen > MAX_FRAME_SIZE) {
				/* Drain and discard the payload. */
				for (uint16_t i = 0; i < flen && flen <= MAX_FRAME_SIZE; i++) {
					uint8_t discard;
					if (sock_recv_all(fd, &discard, 1) != 0) {
						goto client_done;
					}
				}
				LOG_WRN("Skipping frame: type=0x%02x len=%u", hdr[0], flen);
				continue;
			}

			struct frame f;

			f.len = flen;
			err = sock_recv_all(fd, f.buf, flen);
			if (err != 0) {
				break;
			}

			/* Do NOT k_msgq_put here — ble_on_rx_frame() in
			 * main_companion.cpp handles queueing, matching the
			 * ZephyrBLE.cpp pattern (secure_nus_rx_write just calls
			 * on_rx_frame without touching the recv queue). */
			if (transport_cbs && transport_cbs->on_rx_frame) {
				transport_cbs->on_rx_frame(f.buf, f.len);
			}
		}
		client_done:

		k_mutex_lock(&sock_mu, K_FOREVER);
		close_client_locked();
		active_iface = ZEPHCORE_IFACE_NONE;
		k_mutex_unlock(&sock_mu);

		LOG_INF("TCP companion client disconnected");

		if (transport_cbs && transport_cbs->on_disconnected) {
			transport_cbs->on_disconnected();
		}
	}
}

/* ========== Public API (matches ZephyrBLE.h) ========== */

void zephcore_ble_init(const struct ble_callbacks *cbs)
{
	transport_cbs = cbs;
	k_mutex_init(&sock_mu);
	transport_enabled = true;
}

void zephcore_ble_start(const char *device_name)
{
	ARG_UNUSED(device_name);

	if (listen_thread_started) {
		return;
	}

	k_thread_create(&listen_thread, listen_thread_stack,
			K_KERNEL_STACK_SIZEOF(listen_thread_stack),
			listen_thread_fn, NULL, NULL, NULL,
			K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&listen_thread, "linux_tcp_listen");
	listen_thread_started = true;
}

size_t zephcore_ble_send(const uint8_t *data, uint16_t len)
{
	if (len == 0 || len > MAX_FRAME_SIZE) {
		return 0;
	}

	k_mutex_lock(&sock_mu, K_FOREVER);
	int fd = client_fd;

	k_mutex_unlock(&sock_mu);

	if (fd < 0) {
		return 0;
	}

	struct frame f;

	f.len = len;
	memcpy(f.buf, data, len);

	if (k_msgq_put(&ble_send_queue, &f, K_NO_WAIT) != 0) {
		/* Queue full — enter congestion mode (mirrors ZephyrBLE.cpp).
		 * Senders hold off via zephcore_ble_is_congested(). */
		if (!tx_congested) {
			LOG_WRN("TX queue full (%u/%u), entering congestion",
				k_msgq_num_used_get(&ble_send_queue),
				(unsigned)FRAME_QUEUE_SIZE);
			tx_congested = true;
		}

		/* Lossless protocol response frames are never dropped — report
		 * failure so the caller retries instead of clobbering overflow. */
		if (companion_is_lossless_protocol_frame(data, len)) {
			LOG_DBG("TX queue full for lossless frame hdr=0x%02x, retry later",
				data[0]);
			return 0;
		}

		/* Non-lossless push: stash one frame in overflow, retried at
		 * 250ms. If overflow is occupied, drop rather than clobber it. */
		if (overflow_pending) {
			LOG_WRN("overflow full, dropping push hdr=0x%02x", data[0]);
			return 0;
		}
		overflow_frame = f;
		overflow_pending = true;
		k_work_schedule(&overflow_retry_work, K_MSEC(TX_OVERFLOW_RETRY_MS));
		return len;  /* Accepted into overflow — will be retried. */
	}

	k_work_submit(&tx_drain_work);
	return len;
}

void zephcore_ble_set_enabled(bool enable)
{
	transport_enabled = enable;
	if (!enable) {
		k_mutex_lock(&sock_mu, K_FOREVER);
		close_client_locked();
		k_mutex_unlock(&sock_mu);
	}
}

bool zephcore_ble_is_enabled(void)
{
	return transport_enabled;
}

bool zephcore_ble_is_active(void)
{
	bool active;

	k_mutex_lock(&sock_mu, K_FOREVER);
	active = (client_fd >= 0);
	k_mutex_unlock(&sock_mu);
	return active;
}

bool zephcore_ble_is_connected(void)
{
	return zephcore_ble_is_active();
}

bool zephcore_ble_is_congested(void)
{
	/* Sticky flag (mirrors ZephyrBLE): set at full, cleared at 1/3.
	 * Callers also check the 2/3 high-water mark on the raw queue. */
	return tx_congested;
}

bool zephcore_ble_is_advertising(void)
{
	/* TCP transport "advertises" by listening. Always true once started. */
	return listen_fd >= 0;
}

void zephcore_ble_set_passkey(uint32_t pk)
{
	passkey = pk;
}

uint32_t zephcore_ble_get_passkey(void)
{
	return passkey;
}

enum zephcore_iface zephcore_ble_get_active_iface(void)
{
	return active_iface;
}

void zephcore_ble_set_active_iface(enum zephcore_iface iface)
{
	active_iface = iface;
}

struct k_msgq *zephcore_ble_get_recv_queue(void)
{
	return &ble_recv_queue;
}

struct k_msgq *zephcore_ble_get_send_queue(void)
{
	return &ble_send_queue;
}

void zephcore_ble_kick_tx(void)
{
	k_work_submit(&tx_drain_work);
}

void zephcore_ble_disconnect(void)
{
	k_mutex_lock(&sock_mu, K_FOREVER);
	close_client_locked();
	active_iface = ZEPHCORE_IFACE_NONE;
	k_mutex_unlock(&sock_mu);

	if (transport_cbs && transport_cbs->on_disconnected) {
		transport_cbs->on_disconnected();
	}
}

void zephcore_ble_conn_params_ready(void)
{
	/* No BLE conn-param negotiation on TCP — no-op. */
}

void zephcore_ble_update_name(const char *new_name)
{
	/* TCP transport has no advertising payload to update. */
	ARG_UNUSED(new_name);
}
