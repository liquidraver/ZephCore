/*
 * SPDX-License-Identifier: MIT
 * ZephCore serial (UART) companion transport — drop-in zephcore_ble_* provider.
 *
 * For boards with NO Bluetooth and NO USB device controller (e.g. the Seeed
 * LoRa-E5 / STM32WL family), the companion app talks to the node over a plain
 * UART — on the LoRa-E5 mini that UART is bridged to USB-C by an onboard
 * USB-UART chip, so the official MeshCore serial client connects to it exactly
 * like a native-USB companion.
 *
 * This file is the UART analogue of adapters/transport/LinuxTCPTransport.c: it
 * implements the full zephcore_ble_* C API (adapters/ble/ZephyrBLE.h) so the
 * companion application is transport-agnostic. It is compiled INSTEAD of
 * ZephyrBLE.cpp when CONFIG_BT=n and CONFIG_ZEPHCORE_TRANSPORT_TCP=n
 * (see the companion-transport selection in CMakeLists.txt). The MeshCore
 * '<'/'>' framing + struct frame + congestion classification are shared with
 * the TCP transport via companion_framing.h.
 *
 * I/O is interrupt-driven (uart_irq) on both directions so a contact-sync burst
 * (many ~148B frames back-to-back) drains at full line rate with real
 * back-pressure instead of blocking the system workqueue on uart_poll_out.
 *
 * Session model: a UART has no connect/disconnect line event (no DTR — the
 * official client never asserts it), and a connected companion may sit idle for
 * many minutes between frames. We therefore mark "connected" on the first
 * inbound frame (fires on_connected once, lights the UI indicator) and STAY
 * connected; we never infer a disconnect from idleness. Per-session reset on a
 * reconnecting client is driven by the protocol's CMD_APP_START handler, not by
 * any inactivity timer. The only timeout here is a parser-level resync for a
 * partial frame that never completes (below) — it touches the RX byte parser
 * only, never the session/connection state.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "ZephyrBLE.h"
#include "companion_framing.h"

LOG_MODULE_REGISTER(serial_companion, LOG_LEVEL_INF);

#define FRAME_QUEUE_SIZE         CONFIG_ZEPHCORE_BLE_QUEUE_SIZE
#define TX_RING_SIZE             2048   /* ~13 contact frames of headroom */
#define TX_OVERFLOW_RETRY_MS     250

/* Parser-only resync: if a frame has STARTED ('<' seen) but its payload never
 * completes within this window, the next inbound byte resets the parser to
 * IDLE so a lost/truncated frame can't swallow the following frame. This is NOT
 * a session/idle timeout — an idle-but-connected companion sits in RX_IDLE
 * between frames and never arms this, and it never affects connection state. */
#define FRAME_PARTIAL_TIMEOUT_MS 2000

/* Companion UART — the chosen console UART (USART1 on the LoRa-E5 mini). */
static const struct device *uart_dev;

K_MSGQ_DEFINE(ble_send_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ble_recv_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);

static const struct ble_callbacks *transport_cbs;
static enum zephcore_iface active_iface = ZEPHCORE_IFACE_NONE;
static bool transport_enabled;
static uint32_t passkey;

/* ---- RX byte-assembly state (runs in UART ISR) ---------------------------- */
enum rx_state {
	RX_IDLE = 0,  /* waiting for '<' */
	RX_LEN_LO,
	RX_LEN_HI,
	RX_PAYLOAD,
};
static enum rx_state rx_st;
static uint16_t rx_idx;
static uint16_t rx_expect;
static uint32_t rx_frame_start;   /* k_uptime_get_32() when the '<' arrived */
static struct frame rx_frame;

/* ---- TX side: interrupt-driven ring (whole frames committed atomically) ---- */
static uint8_t tx_ring_data[TX_RING_SIZE];
static struct ring_buf tx_ring;
static struct k_spinlock tx_lock;

/* Order-preserving holdover: when the ring is momentarily full, the popped
 * frame is parked here (NOT requeued at the tail, which would reorder it behind
 * later frames) and re-tried first on the next drain. Touched only by
 * tx_drain_work_fn, which runs solely on the system workqueue (never concurrent
 * with itself), so it needs no lock. */
static struct frame held_frame;
static bool held_pending;

/* ---- TX queue drain (moves frames from ble_send_queue into the TX ring) ----
 * Runs on the system workqueue; never blocks on the wire (the ISR does the
 * byte-level draining), so a full sync burst can't starve the workqueue. */
static void tx_drain_work_fn(struct k_work *work);
static K_WORK_DEFINE(tx_drain_work, tx_drain_work_fn);

/* on_connected is deferred off the ISR onto the system workqueue (it ends up
 * in ui_notify(), which is not guaranteed ISR-safe). */
static void connect_work_fn(struct k_work *work);
static K_WORK_DEFINE(connect_work, connect_work_fn);

/* ---- TX congestion control (mirrors LinuxTCPTransport.c / ZephyrBLE.cpp) ---- */
static bool tx_congested;
static struct frame overflow_frame;
static bool overflow_pending;
static void overflow_retry_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(overflow_retry_work, overflow_retry_work_fn);

static void connect_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("serial companion session started");
	if (transport_cbs && transport_cbs->on_connected) {
		transport_cbs->on_connected();
	}
}

/* Mark the session connected on first inbound traffic (fires on_connected once
 * — the UI "connected" indicator). Called from the UART ISR, so the callback is
 * deferred to a workqueue rather than run inline. */
static void mark_connected(void)
{
	if (active_iface != ZEPHCORE_IFACE_NONE) {
		return;
	}
	active_iface = ZEPHCORE_IFACE_BLE;  /* reuse the BLE path in main_companion */
	k_work_submit(&connect_work);
}

/* ---- UART ISR: drains RX into the frame parser, fills TX from the ring ----- */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int n = uart_fifo_read(dev, buf, sizeof(buf));

			/* Parser-only resync: drop a partial frame that stalled
			 * mid-payload so it can't consume the bytes of the next
			 * frame. Connection state is untouched. */
			if (rx_st != RX_IDLE &&
			    (k_uptime_get_32() - rx_frame_start) > FRAME_PARTIAL_TIMEOUT_MS) {
				rx_st = RX_IDLE;
			}

			for (int i = 0; i < n; i++) {
				uint8_t b = buf[i];

				switch (rx_st) {
				case RX_IDLE:
					if (b == COMPANION_FRAME_RX_SYNC) {
						rx_st = RX_LEN_LO;
						rx_frame_start = k_uptime_get_32();
					}
					break;
				case RX_LEN_LO:
					rx_expect = b;
					rx_st = RX_LEN_HI;
					break;
				case RX_LEN_HI:
					rx_expect |= ((uint16_t)b) << 8;
					rx_idx = 0;
					if (rx_expect == 0 || rx_expect > MAX_FRAME_SIZE) {
						rx_st = RX_IDLE;  /* bad length, resync */
					} else {
						rx_st = RX_PAYLOAD;
					}
					break;
				default: /* RX_PAYLOAD */
					rx_frame.buf[rx_idx++] = b;
					if (rx_idx >= rx_expect) {
						rx_frame.len = rx_expect;
						mark_connected();
						/* on_rx_frame queues to ble_recv_queue and
						 * wakes the mesh thread (ISR-safe: K_NO_WAIT
						 * k_msgq_put + k_event_post), same as
						 * ZephyrBLE's secure_nus_rx_write. */
						if (transport_cbs && transport_cbs->on_rx_frame) {
							transport_cbs->on_rx_frame(rx_frame.buf,
										   rx_frame.len);
						}
						rx_st = RX_IDLE;
					}
					break;
				}
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t *out;
			bool empty;
			k_spinlock_key_t key = k_spin_lock(&tx_lock);
			uint32_t claimed = ring_buf_get_claim(&tx_ring, &out, 64);

			if (claimed > 0) {
				int sent = uart_fifo_fill(dev, out, claimed);
				ring_buf_get_finish(&tx_ring, sent > 0 ? sent : 0);
			}
			empty = ring_buf_is_empty(&tx_ring);
			if (empty) {
				/* Disable inside the lock so a concurrent enqueue
				 * can't be stranded (it re-enables after its put). */
				uart_irq_tx_disable(dev);
			}
			k_spin_unlock(&tx_lock, key);

			if (empty) {
				/* Ring drained — pull the next batch from the queue.
				 * tx_drain_work fires on_tx_idle when fully drained. */
				k_work_submit(&tx_drain_work);
			}
		}
	}
}

/* Queue one framed frame into the TX ring. Returns true if it fit (committed
 * atomically — header + payload together — so frames never tear). */
static bool tx_ring_put_frame(const struct frame *f)
{
	uint8_t hdr[3] = {
		COMPANION_FRAME_TX_SYNC,
		(uint8_t)(f->len & 0xFF),
		(uint8_t)((f->len >> 8) & 0xFF),
	};
	size_t total = sizeof(hdr) + f->len;

	k_spinlock_key_t key = k_spin_lock(&tx_lock);
	if (ring_buf_space_get(&tx_ring) < total) {
		k_spin_unlock(&tx_lock, key);
		return false;
	}
	ring_buf_put(&tx_ring, hdr, sizeof(hdr));
	ring_buf_put(&tx_ring, f->buf, f->len);
	k_spin_unlock(&tx_lock, key);

	uart_irq_tx_enable(uart_dev);
	return true;
}

static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	struct frame f;

	/* Re-try the order-preserving holdover first. If it still doesn't fit,
	 * leave it parked — the TX ISR re-submits this work when the ring drains. */
	if (held_pending) {
		if (!tx_ring_put_frame(&held_frame)) {
			return;
		}
		held_pending = false;
	}

	/* Move queued frames into the TX ring while it has room. On ring-full,
	 * park the popped frame in the single holdover slot (preserving FIFO
	 * order) rather than requeuing it behind later frames. */
	while (k_msgq_get(&ble_send_queue, &f, K_NO_WAIT) == 0) {
		if (!tx_ring_put_frame(&f)) {
			held_frame = f;
			held_pending = true;
			return;
		}

		/* Clear congestion at the 1/3 low-water mark (hysteresis). */
		if (tx_congested &&
		    k_msgq_num_used_get(&ble_send_queue) <= FRAME_QUEUE_SIZE / 3) {
			tx_congested = false;
			LOG_INF("tx: congestion cleared");
		}
	}

	/* Queue + holdover fully drained into the ring. Fire on_tx_idle only once
	 * the wire is also idle (ring empty) so the contact pump paces to real
	 * throughput. */
	if (transport_cbs && transport_cbs->on_tx_idle && !held_pending) {
		bool ring_empty;
		k_spinlock_key_t key = k_spin_lock(&tx_lock);
		ring_empty = ring_buf_is_empty(&tx_ring);
		k_spin_unlock(&tx_lock, key);
		if (ring_empty) {
			transport_cbs->on_tx_idle();
		}
	}
}

static void overflow_retry_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!overflow_pending) {
		return;
	}
	if (k_msgq_put(&ble_send_queue, &overflow_frame, K_NO_WAIT) == 0) {
		overflow_pending = false;
		k_work_submit(&tx_drain_work);
	} else {
		k_work_schedule(&overflow_retry_work, K_MSEC(TX_OVERFLOW_RETRY_MS));
	}
}

/* ========== Public API (matches ZephyrBLE.h) ========== */

void zephcore_ble_init(const struct ble_callbacks *cbs)
{
	transport_cbs = cbs;
	transport_enabled = true;
	ring_buf_init(&tx_ring, sizeof(tx_ring_data), tx_ring_data);
}

void zephcore_ble_start(const char *device_name)
{
	ARG_UNUSED(device_name);

	uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("companion UART not ready");
		uart_dev = NULL;
		return;
	}

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);
	LOG_INF("serial companion transport ready on %s", uart_dev->name);
}

size_t zephcore_ble_send(const uint8_t *data, uint16_t len)
{
	if (!uart_dev || len == 0 || len > MAX_FRAME_SIZE) {
		return 0;
	}

	struct frame f;

	f.len = len;
	memcpy(f.buf, data, len);

	if (k_msgq_put(&ble_send_queue, &f, K_NO_WAIT) != 0) {
		/* Queue full — congestion mode (mirrors LinuxTCPTransport.c). */
		if (!tx_congested) {
			LOG_WRN("TX queue full (%u/%u), entering congestion",
				k_msgq_num_used_get(&ble_send_queue),
				(unsigned)FRAME_QUEUE_SIZE);
			tx_congested = true;
		}
		/* Lossless protocol responses are never dropped — report failure
		 * so the caller retries instead of clobbering overflow. */
		if (companion_is_lossless_protocol_frame(data, len)) {
			return 0;
		}
		if (overflow_pending) {
			LOG_WRN("overflow full, dropping push hdr=0x%02x", data[0]);
			return 0;
		}
		overflow_frame = f;
		overflow_pending = true;
		k_work_schedule(&overflow_retry_work, K_MSEC(TX_OVERFLOW_RETRY_MS));
		return len;
	}

	k_work_submit(&tx_drain_work);
	return len;
}

void zephcore_ble_set_enabled(bool enable)
{
	transport_enabled = enable;
	if (!uart_dev) {
		return;
	}
	/* Actually gate the link, not just a flag: stop accepting frames when
	 * disabled (mirrors the TCP transport closing its socket). */
	if (enable) {
		uart_irq_rx_enable(uart_dev);
	} else {
		uart_irq_rx_disable(uart_dev);
	}
}

bool zephcore_ble_is_enabled(void)
{
	return transport_enabled;
}

bool zephcore_ble_is_active(void)
{
	return uart_dev != NULL && active_iface == ZEPHCORE_IFACE_BLE;
}

bool zephcore_ble_is_connected(void)
{
	return active_iface == ZEPHCORE_IFACE_BLE;
}

bool zephcore_ble_is_congested(void)
{
	return tx_congested;
}

bool zephcore_ble_is_advertising(void)
{
	/* A UART is "always advertising" once the transport is up — keeps the
	 * companion housekeeping loop from trying to (re)start advertising. */
	return uart_dev != NULL;
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

bool zephcore_ble_iface_try_claim(enum zephcore_iface who)
{
	if (active_iface == ZEPHCORE_IFACE_NONE || active_iface == who) {
		active_iface = who;
		return true;
	}
	return false;
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
	active_iface = ZEPHCORE_IFACE_NONE;
	if (transport_cbs && transport_cbs->on_disconnected) {
		transport_cbs->on_disconnected();
	}
}

void zephcore_ble_conn_params_ready(void)
{
	/* No link-layer connection parameters on a UART — no-op. */
}

void zephcore_ble_update_name(const char *new_name)
{
	/* No advertising payload on a UART — no-op. */
	ARG_UNUSED(new_name);
}
