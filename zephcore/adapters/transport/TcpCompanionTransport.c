/*
 * SPDX-License-Identifier: MIT
 * ZephCore TCP companion transport — see TcpCompanionTransport.h.
 *
 * Wire format: MeshCore SerialWifiInterface framing (Arduino reference:
 * src/helpers/wifi/SerialWifiInterface.cpp):
 *
 *   App → Node:  [ '<' (0x3C) | length_LSB | length_MSB | payload... ]
 *   Node → App:  [ '>' (0x3E) | length_LSB | length_MSB | payload... ]
 *
 * Frames with type != '<' are skipped (matches Arduino). One client at a time.
 *
 * Architecture:
 *   - The listener thread accepts a client, then reads frames into the recv
 *     queue and raises on_rx.
 *   - TX is a work item draining the frame_txq with zsock_send() until empty
 *     or the socket dies.
 */

#include "TcpCompanionTransport.h"
#include "frame_txq.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tcp_companion, LOG_LEVEL_INF);

#define FRAME_QUEUE_SIZE CONFIG_ZEPHCORE_TCP_QUEUE_SIZE
#define LISTEN_BACKLOG   1

/* Bound on a single blocking send. tx_drain_work_fn runs on the system
 * workqueue, so an unbounded zsock_send() against a peer that has stopped
 * reading would hang the whole queue (incl. the txq's overflow retry). A peer
 * that can't accept one frame within this window is wedged — close it and
 * let the app reconnect rather than stall the node. */
#define TX_SEND_TIMEOUT_SEC  2

/* Longest the RX thread waits for room in the recv queue before dropping. */
#define RX_QUEUE_WAIT_MS     250

K_MSGQ_DEFINE(tcp_send_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(tcp_recv_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
static struct frame_txq tcp_txq;

static const struct companion_link_cbs *s_link;
static uint16_t s_port;
static bool transport_enabled = true;

/* Socket state */
static int listen_fd = -1;
static int client_fd = -1;
static struct k_mutex sock_mu;

/* Listen + RX thread. Peak 1524 B with WiFi (XIAO ESP32-S3, 2026-09-24,
 * CONFIG_ZEPHCORE_MEM_STATS under the protocol golden and a bulk dump over
 * TCP); was 4096. */
static K_KERNEL_STACK_DEFINE(listen_thread_stack, 3072);
static struct k_thread listen_thread;
static bool listen_thread_started;

/* TX work — drains the txq */
static void tx_drain_work_fn(struct k_work *work);
static K_WORK_DEFINE(tx_drain_work, tx_drain_work_fn);

static bool tcp_link_up(void)
{
	bool up;

	k_mutex_lock(&sock_mu, K_FOREVER);
	up = (client_fd >= 0);
	k_mutex_unlock(&sock_mu);
	return up;
}

static void tcp_kick(void)
{
	k_work_submit(&tx_drain_work);
}

static void close_client_locked(void)
{
	if (client_fd >= 0) {
		zsock_close(client_fd);
		client_fd = -1;
	}
	frame_txq_reset(&tcp_txq);
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

	while (frame_txq_get(&tcp_txq, &f) == 0) {
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

		hdr[0] = COMPANION_FRAME_TX_SYNC;
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
			bool was_open = (client_fd >= 0);

			close_client_locked();
			k_mutex_unlock(&sock_mu);

			if (was_open && s_link && s_link->on_disconnected) {
				s_link->on_disconnected();
			}
			return;
		}
	}

	if (s_link && s_link->on_tx_idle) {
		s_link->on_tx_idle();
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
	addr.sin_port = net_htons(s_port);

	listen_fd = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	if (listen_fd < 0) {
		LOG_ERR("zsock_socket failed: %d", errno);
		return;
	}

	int one = 1;
	(void)zsock_setsockopt(listen_fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR,
				&one, sizeof(one));

	if (zsock_bind(listen_fd, (struct net_sockaddr *)&addr, sizeof(addr)) < 0) {
		LOG_ERR("bind(%u) failed: %d", (unsigned)s_port, errno);
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

	LOG_INF("TCP companion transport listening on :%u", (unsigned)s_port);

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

		if (!transport_enabled) {
			LOG_INF("TCP companion disabled, client refused");
			zsock_close(fd);
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
		k_mutex_unlock(&sock_mu);

		LOG_INF("TCP companion client connected");

		if (s_link && s_link->on_connected) {
			s_link->on_connected();
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
			if (flen > MAX_FRAME_SIZE) {
				LOG_WRN("Oversized frame: len=%u, closing client", flen);
				break;
			}

			/* Skip frames not from app ('<'), or bad length. */
			if (hdr[0] != COMPANION_FRAME_RX_SYNC || flen == 0) {
				/* Drain and discard the payload. */
				for (uint16_t i = 0; i < flen; i++) {
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

			/* Wait for room rather than drop: until this returns the
			 * socket is not read, so a burst backs up into TCP flow
			 * control instead of losing a command. Bounded, so a
			 * companion that stopped draining cannot pin the thread. */
			if (k_msgq_put(&tcp_recv_queue, &f, K_MSEC(RX_QUEUE_WAIT_MS)) != 0) {
				LOG_WRN("recv queue full, frame 0x%02x dropped", f.buf[0]);
				continue;
			}
			if (s_link && s_link->on_rx) {
				s_link->on_rx();
			}
		}
		client_done:

		k_mutex_lock(&sock_mu, K_FOREVER);
		/* The TX path may already have closed it (and reported it) */
		bool was_open = (client_fd == fd);

		if (was_open) {
			close_client_locked();
		}
		k_msgq_purge(&tcp_recv_queue);
		k_mutex_unlock(&sock_mu);

		LOG_INF("TCP companion client disconnected");

		if (was_open && s_link && s_link->on_disconnected) {
			s_link->on_disconnected();
		}
	}
}

/* ========== Public API ========== */

void tcp_companion_init(const struct companion_link_cbs *link)
{
	s_link = link;
	k_mutex_init(&sock_mu);
	frame_txq_init(&tcp_txq, &tcp_send_queue, FRAME_QUEUE_SIZE, "tcp",
		       tcp_link_up, tcp_kick);
}

void tcp_companion_start(uint16_t port)
{
	if (listen_thread_started) {
		return;
	}
	s_port = port;

	k_thread_create(&listen_thread, listen_thread_stack,
			K_KERNEL_STACK_SIZEOF(listen_thread_stack),
			listen_thread_fn, NULL, NULL, NULL,
			K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&listen_thread, "tcp_companion");
	listen_thread_started = true;
}

size_t tcp_companion_send(const uint8_t *data, uint16_t len)
{
	if (!tcp_link_up()) {
		return 0;
	}
	return frame_txq_put(&tcp_txq, data, len);
}

size_t tcp_companion_recv(uint8_t *dest)
{
	struct frame f;

	if (k_msgq_get(&tcp_recv_queue, &f, K_NO_WAIT) != 0) {
		return 0;
	}
	memcpy(dest, f.buf, f.len);
	return f.len;
}

void tcp_companion_set_enabled(bool enable)
{
	transport_enabled = enable;
	if (!enable) {
		k_mutex_lock(&sock_mu, K_FOREVER);
		/* Shutting the socket ends the listener's blocking recv; the
		 * listener then reports the disconnect. */
		if (client_fd >= 0) {
			(void)zsock_shutdown(client_fd, ZSOCK_SHUT_RDWR);
		}
		k_mutex_unlock(&sock_mu);
	}
}

bool tcp_companion_is_enabled(void)
{
	return transport_enabled;
}

bool tcp_companion_is_connected(void)
{
	return tcp_link_up();
}

bool tcp_companion_is_write_busy(void)
{
	return frame_txq_busy(&tcp_txq);
}

bool tcp_companion_tx_idle(void)
{
	/* No client — the drain drops queued frames rather than holding them,
	 * so there is nothing to wait for. */
	if (!tcp_link_up()) {
		return true;
	}

	/* Sends are synchronous inside tx_drain_work_fn, so an empty queue means
	 * every frame has reached the socket. No "in progress" flag is needed:
	 * the drain work and the reboot poller share the system workqueue and
	 * never run concurrently. */
	return frame_txq_empty(&tcp_txq);
}
