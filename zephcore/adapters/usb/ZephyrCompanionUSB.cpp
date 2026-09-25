/*
 * SPDX-License-Identifier: MIT
 * ZephCore USB CDC Companion Transport
 *
 * V3-framed companion link over USB CDC or a plain UART, with the text CLI.
 * One transport among several: CompanionInterfaces.h wraps it as a
 * BaseSerialInterface for the MultiSerialInterface.
 *
 * USBD lifecycle + 1200-baud DFU detection + DTR state tracking live in
 * the shared ZephyrUSBCDC module; this file just runs the V3 frame parser
 * on top of the CDC ACM UART and reacts to DTR-drop events from there.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_usb, CONFIG_ZEPHCORE_USB_LOG_LEVEL);

#include "ZephyrCompanionUSB.h"
#include "ZephyrUSBCDC.h"
#include "companion_framing.h"

#define USB_RING_BUF_SIZE     512     /* USB RX ring buffer size */
#define USB_TX_RING_BUF_SIZE  2048    /* USB TX ring buffer (~13 contact frames of headroom) */
#define USB_FRAME_TIMEOUT_MS  2000    /* Partial frame timeout - reset parser after 2s of no completion */
#define USB_RECV_QUEUE_DEPTH  4       /* complete frames waiting for the main thread */

/* Companion serial framing (MeshCore ArduinoSerialInterface):
 *   app  → device:  '<' len_lo len_hi <payload...>
 *   device → app:   '>' len_lo len_hi <payload...>
 * The leading sync byte is mandatory — the official app keys off it, and we
 * must ignore any noise/banner bytes until it arrives. */
#define USB_FRAME_RX_SYNC     '<'
#define USB_FRAME_TX_SYNC     '>'

enum usb_rx_state {
	USB_RX_IDLE = 0,  /* waiting for '<' sync byte or first text byte */
	USB_RX_LEN_LO,    /* got sync, waiting len LSB */
	USB_RX_LEN_HI,    /* got len LSB, waiting len MSB */
	USB_RX_PAYLOAD,   /* accumulating V3 payload */
	USB_RX_TEXT,      /* text CLI line mode — accumulate until CR/LF */
};

#define USB_TEXT_LINE_MAX 128

/* ---- Backend selection --------------------------------------------------
 * The companion byte transport is normally a native-USB CDC-ACM UART, but the
 * same frame parser / TX ring / CLI runs unchanged over a plain UART too — for
 * boards whose USB-C is a USB-UART bridge (e.g. Heltec V3 / CP2102) or that
 * have no USB device controller at all.  A board selects the backend with the
 * `zephcore,companion-uart` chosen node; absent that we fall back to the sole
 * cdc-acm-uart, so existing native-USB and nRF builds resolve identically.
 *
 * COMPANION_HAS_DTR is true only for the CDC backend — a plain UART has no DTR
 * line, so a session there, once started, lasts until reboot. */
#if DT_HAS_CHOSEN(zephcore_companion_uart)
#  define COMPANION_UART_DEV    DEVICE_DT_GET(DT_CHOSEN(zephcore_companion_uart))
#  define COMPANION_HAS_DTR     DT_NODE_HAS_COMPAT(DT_CHOSEN(zephcore_companion_uart), zephyr_cdc_acm_uart)
#  define COMPANION_HAS_BACKEND 1
#elif DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart) && \
	(IS_ENABLED(CONFIG_USB_CDC_ACM) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS))
#  define COMPANION_UART_DEV    DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart)
#  define COMPANION_HAS_DTR     1
#  define COMPANION_HAS_BACKEND 1
#else
#  define COMPANION_HAS_DTR     0
#  define COMPANION_HAS_BACKEND 0
#endif

/* USB CDC state */
static const struct device *usb_dev;
static uint8_t usb_ring_buf_data[USB_RING_BUF_SIZE];
static struct ring_buf usb_ring_buf;
static uint8_t usb_rx_buf[MAX_FRAME_SIZE];
static enum usb_rx_state usb_rx_st;
static uint16_t usb_rx_idx;     /* payload bytes received so far */
static uint16_t usb_frame_len;  /* Expected payload length (0 = none in progress) */
static uint32_t usb_frame_start_time;  /* Timestamp of sync byte for current frame */

/* TX side: interrupt-driven so the contact pump gets real backpressure +
 * a "drained" event (the USB analogue of BLE's notify-complete) instead of a
 * fixed delay. write_frame queues whole frames here under usb_tx_lock; the TX
 * ISR drains into the CDC FIFO and raises on_tx_idle when the ring empties. */
static uint8_t usb_tx_ring_buf_data[USB_TX_RING_BUF_SIZE];
static struct ring_buf usb_tx_ring_buf;
static struct k_spinlock usb_tx_lock;

/* Callbacks to main (set by init): frame waiting, TX drained, session
 * start/end.  The byte assembly below runs on sysworkq, but V3-protocol
 * parsing must happen on the main thread (handleCmdFrame mutates mesh state
 * shared with loop()), so a complete frame is queued and on_rx posts the
 * event instead of running the parser here. */
static const struct companion_link_cbs *s_link;

/* Complete binary frames, drained by zephcore_usb_companion_recv() */
K_MSGQ_DEFINE(usb_recv_queue, sizeof(struct frame), USB_RECV_QUEUE_DEPTH, 4);

/* A session starts on the first traffic after open (a binary frame or a CLI
 * line) and ends when the host drops DTR. */
static bool usb_session_active;

/* CLI text line callback — fired when a complete line arrives in text mode. */
static void (*s_cli_line_cb)(const char *line);
static char usb_text_line[USB_TEXT_LINE_MAX];
static uint8_t usb_text_len;

/* Banner shown once per text-CLI session, matching the repeater's serial CLI so
 * the flasher.meshcore.io console renders identically. Emitted only after text
 * mode is detected (first printable byte) — never on the binary V3 path, so an
 * official client connected over USB never receives stray text. CRLF endings:
 * the console's LineBreakTransformer only splits on "\r\n". */
#define USB_CLI_BANNER "\r\n=== ZephCore Companion ===\r\n"
static bool usb_text_banner_sent;

/* True when the session was opened by the text CLI (not a binary V3 companion
 * app). While set the session reports not connected, so no binary frame or
 * push reaches the serial console. Set when the session starts, reset when it
 * ends. */
static bool usb_session_is_text;

/* Echo text-mode bytes back via the interrupt-driven TX ring (NOT uart_poll_out,
 * unlike the repeater) so echo stays off the polling path and ordered with the
 * reply. Only ever called from the text branches, so binary sessions see no echo. */
static inline void usb_cli_echo(const char *s, size_t n)
{
	zephcore_usb_companion_write_text(s, n);
}

/* Work items */
static void usb_rx_work_fn(struct k_work *work);

K_WORK_DEFINE(usb_rx_work, usb_rx_work_fn);

/* USB CDC UART interrupt callback - puts bytes in ring buffer */
static void usb_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	for (;;) {
		uart_irq_update(dev);
		if (uart_irq_is_pending(dev) <= 0) {
			break;
		}

		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
			if (recv_len > 0) {
				ring_buf_put(&usb_ring_buf, buf, recv_len);
				k_work_submit(&usb_rx_work);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			/* Push as much of our TX ring as the CDC FIFO will take.
			 * uart_fifo_fill returns the count actually accepted, so the
			 * remainder stays queued and the callback fires again when the
			 * FIFO drains — backpressure all the way to the wire. */
			uint8_t *out;
			bool empty;
			k_spinlock_key_t key = k_spin_lock(&usb_tx_lock);
			uint32_t claimed = MIN(ring_buf_get_ptr(&usb_tx_ring_buf, &out, 0), 64U);
			if (claimed > 0) {
				int sent = uart_fifo_fill(dev, out, claimed);

				if (sent > 0) {
					ring_buf_consume(&usb_tx_ring_buf, sent);
				}
			}
			empty = ring_buf_is_empty(&usb_tx_ring_buf);
			if (empty) {
				/* Disable inside the lock so a concurrent write_frame can't
				 * enqueue+enable in the gap and then have us disable it,
				 * stranding the frame. write_frame's enable always runs
				 * after its put, so it re-arms us correctly. */
				uart_irq_tx_disable(dev);
			}
			k_spin_unlock(&usb_tx_lock, key);

			if (empty && s_link && s_link->on_tx_idle) {
				/* Channel idle — let the pump queue the next batch. */
				s_link->on_tx_idle();
			}
		}
	}
}

/* Start a session on its first inbound traffic — a binary frame or a complete
 * CLI line. The official client opens with CMD_DEVICE_QUERY (0x16), not
 * CMD_APP_START, so any first traffic starts it. Other transports (BLE, WiFi)
 * may be live at the same time: every connected one is served. */
static void usb_session_begin(uint8_t log_tag, bool is_text)
{
	if (usb_session_active) {
		return;
	}
	usb_session_active = true;
	usb_session_is_text = is_text;
	LOG_INF("usb_rx: first traffic 0x%02x, session started (%s)", log_tag,
		is_text ? "text" : "binary");
	/* Arduino shows "connected" for serial transports too */
	if (s_link && s_link->on_connected) {
		s_link->on_connected();
	}
}

/* USB RX work - parses V3 frames from ring buffer */
static void usb_rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	uint8_t byte;

	/* Timeout partial input — if we've been mid-frame or mid-text-line too
	 * long without completing, reset the parser and resync. usb_frame_start_time
	 * is refreshed on every text byte (below), so for USB_RX_TEXT this acts as an
	 * inactivity watchdog: it recovers a stray printable byte back to IDLE (so a
	 * later binary frame parses) without truncating a line that is actively being
	 * typed. Note this only runs when bytes arrive — it is not a timer and never
	 * wakes a sleeping node. */
	if (usb_rx_st != USB_RX_IDLE &&
	    (k_uptime_get_32() - usb_frame_start_time) > USB_FRAME_TIMEOUT_MS) {
		LOG_WRN("usb_rx: partial input timeout (state=%d, expected=%u), resync",
			usb_rx_st, usb_frame_len);
		usb_rx_st = USB_RX_IDLE;
		usb_frame_len = 0;
		usb_rx_idx = 0;
		usb_text_len = 0;
	}

	while (ring_buf_get(&usb_ring_buf, &byte, 1) == 1) {
		switch (usb_rx_st) {
		case USB_RX_IDLE:
			if (byte == USB_FRAME_RX_SYNC) {
				/* V3 binary framing — normal app protocol. */
				usb_rx_st = USB_RX_LEN_LO;
				usb_frame_start_time = k_uptime_get_32();
			} else if (byte >= 0x20 && byte <= 0x7E) {
				/* Printable ASCII. Enter text CLI mode only before a
				 * session starts, or in a text session (subsequent command
				 * lines). Never in a binary session — an official client's
				 * bytes must not be parsed as CLI. The session starts
				 * later, on Enter. */
				if (!usb_session_active || usb_session_is_text) {
					/* Arm the inactivity watchdog so a stray byte resyncs
					 * to IDLE on its own. */
					usb_text_len = 0;
					usb_text_line[usb_text_len++] = (char)byte;
					usb_frame_start_time = k_uptime_get_32();
					usb_rx_st = USB_RX_TEXT;
					/* Banner once per session, then echo. */
					if (!usb_text_banner_sent) {
						usb_text_banner_sent = true;
						usb_cli_echo(USB_CLI_BANNER, sizeof(USB_CLI_BANNER) - 1);
					}
					usb_cli_echo((const char *)&byte, 1);
				}
				/* else: printable in a binary session — ignore */
			}
			/* else: ignore control bytes / noise */
			break;

		case USB_RX_TEXT:
			/* Any text byte is activity — refresh the inactivity watchdog. */
			usb_frame_start_time = k_uptime_get_32();
			if (byte == '\n' || byte == '\r') {
				/* Line complete — dispatch if non-empty. The command runs
				 * on the main thread (companion_cli_dispatch queues it), so it
				 * is safe beside a live BLE or WiFi app. The reply emits its
				 * own leading CRLF, so the Enter keystroke itself is not
				 * echoed — matching the repeater. */
				if (usb_text_len > 0) {
					usb_text_line[usb_text_len] = '\0';
					usb_session_begin((uint8_t)usb_text_line[0], true);
					if (s_cli_line_cb) {
						s_cli_line_cb(usb_text_line);
					}
				}
				usb_text_len = 0;
				usb_rx_st = USB_RX_IDLE;
			} else if (byte == 0x7F || byte == '\b') {
				/* Backspace — erase one char on the terminal too. */
				if (usb_text_len > 0) {
					usb_text_len--;
					usb_cli_echo("\b \b", 3);
				}
			} else if (byte >= 0x20 && byte <= 0x7E) {
				if (usb_text_len < USB_TEXT_LINE_MAX - 1) {
					usb_text_line[usb_text_len++] = (char)byte;
					usb_cli_echo((const char *)&byte, 1);
				}
			}
			break;
		case USB_RX_LEN_LO:
			usb_frame_len = byte;  /* LSB */
			usb_rx_st = USB_RX_LEN_HI;
			break;
		case USB_RX_LEN_HI:
			usb_frame_len |= ((uint16_t)byte) << 8;  /* MSB */
			usb_rx_idx = 0;
			if (usb_frame_len == 0 || usb_frame_len > MAX_FRAME_SIZE) {
				LOG_WRN("usb_rx: invalid frame len %u, resync", usb_frame_len);
				usb_rx_st = USB_RX_IDLE;
				usb_frame_len = 0;
			} else {
				usb_rx_st = USB_RX_PAYLOAD;
			}
			break;
		default: /* USB_RX_PAYLOAD */
			usb_rx_buf[usb_rx_idx++] = byte;

			if (usb_rx_idx >= usb_frame_len) {
				/* Frame complete - queue it */
				uint8_t *payload = usb_rx_buf;
				uint16_t payload_len = usb_frame_len;

				LOG_DBG("usb_rx: frame complete len=%u hdr=0x%02x", payload_len, payload[0]);

				usb_session_begin(payload[0], false);
				struct frame f;

				f.len = payload_len;
				memcpy(f.buf, payload, payload_len);
				/* Queue the frame and wake the main thread to parse it
				 * (parsing on sysworkq would race loop()). */
				if (k_msgq_put(&usb_recv_queue, &f, K_NO_WAIT) == 0) {
					if (s_link && s_link->on_rx) {
						s_link->on_rx();
					}
				} else {
					LOG_WRN("usb_rx: recv queue full, frame 0x%02x dropped", payload[0]);
				}

				/* Reset for next frame */
				usb_rx_st = USB_RX_IDLE;
				usb_frame_len = 0;
				usb_rx_idx = 0;
			}
			break;
		}
	}
}

#if COMPANION_HAS_DTR
/* DTR-transition callback from the shared ZephyrUSBCDC module.
 * On drop: host closed the port → reset parser, hand control back to BLE.
 * CDC-only — a plain-UART backend has no DTR and never registers this. */
static void on_dtr_change(bool dtr_active)
{
	if (dtr_active) {
		return;
	}
	LOG_INF("usb_dtr: DTR dropped, USB disconnected");
	bool was_active = usb_session_active;

	usb_session_active = false;
	ring_buf_reset(&usb_ring_buf);
	usb_rx_st = USB_RX_IDLE;
	usb_frame_len = 0;
	usb_rx_idx = 0;
	usb_text_len = 0;
	usb_text_banner_sent = false;  /* re-banner the next text session */
	usb_session_is_text = false;
	k_msgq_purge(&usb_recv_queue);

	/* Discard any pending TX from the closed session. */
	uart_irq_tx_disable(usb_dev);
	k_spinlock_key_t key = k_spin_lock(&usb_tx_lock);
	ring_buf_reset(&usb_tx_ring_buf);
	k_spin_unlock(&usb_tx_lock, key);

	/* The session is over: main runs its per-session cleanup (contact dump,
	 * sync, sign buffer) once no other transport is still connected. */
	if (was_active && s_link && s_link->on_disconnected) {
		s_link->on_disconnected();
	}
}
#endif /* COMPANION_HAS_DTR */

/* Queue a frame for interrupt-driven TX (sync byte + LE length + payload,
 * matching the MeshCore ArduinoSerialInterface framing). The whole frame is
 * committed atomically under usb_tx_lock — either it all fits or none of it
 * does (returns 0), so frames never tear and tx_has_space() stays truthful.
 * 0 means "ring full, retry when drained"; the caller (contact pump) backs off
 * and the TX-drain callback re-kicks it. */
size_t zephcore_usb_companion_write_frame(const uint8_t *src, size_t len)
{
	if (!usb_dev || len == 0 || len > MAX_FRAME_SIZE) {
		return 0;
	}

	uint8_t hdr[3] = {
		USB_FRAME_TX_SYNC,
		(uint8_t)(len & 0xFF),
		(uint8_t)((len >> 8) & 0xFF),
	};
	size_t total = sizeof(hdr) + len;

	k_spinlock_key_t key = k_spin_lock(&usb_tx_lock);
	if (ring_buf_space_get(&usb_tx_ring_buf) < total) {
		k_spin_unlock(&usb_tx_lock, key);
		return 0;
	}
	ring_buf_put(&usb_tx_ring_buf, hdr, sizeof(hdr));
	ring_buf_put(&usb_tx_ring_buf, src, len);
	k_spin_unlock(&usb_tx_lock, key);

	/* Kick the TX ISR; harmless if already enabled. */
	uart_irq_tx_enable(usb_dev);

	LOG_DBG("usb_write_frame: queued len=%u hdr=0x%02x", (unsigned)len, src[0]);
	return len;
}

/* True if the TX ring can hold one more frame of `payload_len` (+3 framing).
 * The pump checks this before each contact so write_frame can't fail mid-dump. */
/* True when the TX ring has drained — every framed byte handed to the CDC
 * interrupt writer.  Mirrors zephcore_ble_tx_idle() for the USB transport. */
bool zephcore_usb_companion_tx_idle(void)
{
	if (!usb_dev) {
		return true;
	}
	k_spinlock_key_t key = k_spin_lock(&usb_tx_lock);
	bool idle = ring_buf_is_empty(&usb_tx_ring_buf);
	k_spin_unlock(&usb_tx_lock, key);
	return idle;
}

bool zephcore_usb_companion_tx_has_space(size_t payload_len)
{
	if (!usb_dev) {
		return false;
	}
	k_spinlock_key_t key = k_spin_lock(&usb_tx_lock);
	bool ok = ring_buf_space_get(&usb_tx_ring_buf) >= payload_len + 3;
	k_spin_unlock(&usb_tx_lock, key);
	return ok;
}

size_t zephcore_usb_companion_recv(uint8_t *dest)
{
	struct frame f;

	if (k_msgq_get(&usb_recv_queue, &f, K_NO_WAIT) != 0) {
		return 0;
	}
	memcpy(dest, f.buf, f.len);
	return f.len;
}

bool zephcore_usb_companion_is_connected(void)
{
	return usb_session_active && !usb_session_is_text;
}

/* Only a connected session is ever busy: MultiSerialInterface asks every
 * enabled transport, and this one is always enabled. */
bool zephcore_usb_companion_is_write_busy(void)
{
	return zephcore_usb_companion_is_connected() &&
	       !zephcore_usb_companion_tx_has_space(MAX_FRAME_SIZE);
}

bool zephcore_usb_companion_is_text_session(void)
{
	return usb_session_is_text;
}

void zephcore_usb_companion_set_cli_line_cb(void (*cb)(const char *line))
{
	s_cli_line_cb = cb;
}

void zephcore_usb_companion_write_text(const char *text, size_t len)
{
	if (!usb_dev || !text || len == 0) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&usb_tx_lock);
	ring_buf_put(&usb_tx_ring_buf, (const uint8_t *)text, len);
	k_spin_unlock(&usb_tx_lock, key);
	uart_irq_tx_enable(usb_dev);
}

void zephcore_usb_companion_init(const struct companion_link_cbs *link)
{
	s_link = link;

	/* COMPANION_UART_DEV resolves to the chosen `zephcore,companion-uart` node,
	 * or the sole cdc-acm-uart for back-compat (see the backend block above).
	 * The cdc_acm_uart DT node may be present without the class driver compiled
	 * (shared esp32s3_usb_otg.dtsi exposes the node unconditionally; the class
	 * is only enabled with esp32s3_usb.conf) — the COMPANION_HAS_BACKEND gate
	 * folds that case in, so DEVICE_DT_GET never references an undefined ordinal
	 * on, e.g., a debug ESP32-S3 companion built without esp32s3_usb.conf. */
#if COMPANION_HAS_BACKEND
	usb_dev = COMPANION_UART_DEV;
	if (device_is_ready(usb_dev)) {
		LOG_INF("Companion UART ready: %s (%s)", usb_dev->name,
			COMPANION_HAS_DTR ? "CDC" : "serial");
		ring_buf_init(&usb_ring_buf, sizeof(usb_ring_buf_data), usb_ring_buf_data);
		ring_buf_init(&usb_tx_ring_buf, sizeof(usb_tx_ring_buf_data), usb_tx_ring_buf_data);

		/* Set up UART interrupt callback (RX enabled now, TX enabled on demand
		 * by write_frame and disabled by the ISR when the TX ring drains). */
		uart_irq_callback_set(usb_dev, usb_uart_isr);
		uart_irq_rx_enable(usb_dev);

#if COMPANION_HAS_DTR
		/* DTR state changes (including disconnect) reach us via the
		 * shared usbd_msg_callback — no polling work needed.  Plain-UART
		 * backends have no DTR; their session reset is protocol-driven. */
		zephcore_usbd_set_dtr_cb(on_dtr_change);
#endif
	} else {
		LOG_WRN("Companion UART not ready");
		usb_dev = NULL;
	}
#endif
}
