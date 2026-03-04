/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore USB CDC Companion Transport
 *
 * V3-framed USB CDC for companion mode. Extracted from main_companion.cpp.
 * Only compiled when CONFIG_LOG is enabled (debug builds).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_usb, CONFIG_ZEPHCORE_USB_LOG_LEVEL);

#include <ZephyrBLE.h>
#include <adapters/board/ZephyrBoard.h>
#include <app/CompanionMesh.h>

#include "ZephyrCompanionUSB.h"

/* MAX_FRAME_SIZE defined in CompanionMesh.h */

#define USB_DTR_CHECK_MS      10000   /* USB DTR poll interval - 10s for power savings */
#define USB_RING_BUF_SIZE     512     /* USB RX ring buffer size */
#define USB_FRAME_TIMEOUT_MS  2000    /* Partial frame timeout - reset parser after 2s of no completion */

/* USB CDC state */
static const struct device *usb_dev;
static uint8_t usb_ring_buf_data[USB_RING_BUF_SIZE];
static struct ring_buf usb_ring_buf;
static uint8_t usb_rx_buf[MAX_FRAME_SIZE + 2];  /* +2 for length prefix */
static uint16_t usb_rx_idx;
static uint16_t usb_frame_len;  /* Expected frame length (0 = waiting for header) */
static uint32_t usb_frame_start_time;  /* Timestamp of first byte in current frame */
static bool usb_dtr_active;

/* Pointers to mesh event infrastructure (set by init) */
static struct k_event *s_mesh_events;
static struct k_work *s_rx_work;
static uint32_t s_mesh_event_ble_rx;
static mesh::ZephyrBoard *s_board;

/* Work items */
static void usb_rx_work_fn(struct k_work *work);
static void usb_dtr_check_work_fn(struct k_work *work);

K_WORK_DEFINE(usb_rx_work, usb_rx_work_fn);
K_WORK_DELAYABLE_DEFINE(usb_dtr_check_work, usb_dtr_check_work_fn);

/**
 * Reboot into the Adafruit nRF52 bootloader DFU mode.
 * Triggered by host opening USB CDC at 1200 baud then dropping DTR.
 */
static void reboot_to_bootloader_dfu(void)
{
	LOG_INF("*** REBOOTING TO BOOTLOADER DFU ***");
	s_board->rebootToBootloader();
	CODE_UNREACHABLE;
}

/* USB CDC UART interrupt callback - puts bytes in ring buffer */
static void usb_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
			if (recv_len > 0) {
				ring_buf_put(&usb_ring_buf, buf, recv_len);
				k_work_submit(&usb_rx_work);
			}
		}
	}
}

/* USB RX work - parses V3 frames from ring buffer */
static void usb_rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	uint8_t byte;

	/* Timeout partial frames — if we've been accumulating bytes for too long
	 * without completing a frame, reset the parser state */
	if (usb_rx_idx > 0 && (k_uptime_get_32() - usb_frame_start_time) > USB_FRAME_TIMEOUT_MS) {
		LOG_WRN("usb_rx: partial frame timeout (idx=%u, expected=%u), resync",
			usb_rx_idx, usb_frame_len);
		usb_frame_len = 0;
		usb_rx_idx = 0;
	}

	while (ring_buf_get(&usb_ring_buf, &byte, 1) == 1) {
		/* V3 framing: [len_lo][len_hi][payload...] */
		if (usb_rx_idx == 0) {
			/* First byte of length */
			usb_rx_buf[0] = byte;
			usb_rx_idx = 1;
			usb_frame_start_time = k_uptime_get_32();
		} else if (usb_rx_idx == 1) {
			/* Second byte of length */
			usb_rx_buf[1] = byte;
			usb_frame_len = usb_rx_buf[0] | (usb_rx_buf[1] << 8);
			usb_rx_idx = 2;

			if (usb_frame_len == 0 || usb_frame_len > MAX_FRAME_SIZE) {
				LOG_WRN("usb_rx: invalid frame len %u, resync", usb_frame_len);
				usb_frame_len = 0;
				usb_rx_idx = 0;
			}
		} else {
			/* Payload bytes */
			usb_rx_buf[usb_rx_idx++] = byte;

			if (usb_rx_idx >= usb_frame_len + 2) {
				/* Frame complete - queue it */
				uint8_t *payload = &usb_rx_buf[2];
				uint16_t payload_len = usb_frame_len;

				LOG_DBG("usb_rx: frame complete len=%u hdr=0x%02x", payload_len, payload[0]);

				/* Check for CMD_APP_START to switch interface */
				if (payload_len >= 1 && payload[0] == 0x03 /* CMD_APP_START */) {
					if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_BLE &&
					    zephcore_ble_is_connected()) {
						LOG_INF("usb_rx: CMD_APP_START, disconnecting BLE");
						zephcore_ble_disconnect();
					}
					zephcore_ble_set_active_iface(ZEPHCORE_IFACE_USB);
					LOG_INF("usb_rx: active_iface = IFACE_USB");
				}

				/* Only process if USB is active interface */
				if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
					struct {
						uint16_t len;
						uint8_t buf[MAX_FRAME_SIZE];
					} f;
					f.len = payload_len;
					memcpy(f.buf, payload, payload_len);
					if (k_msgq_put(zephcore_ble_get_recv_queue(), &f, K_NO_WAIT) == 0) {
						k_work_submit(s_rx_work);
						k_event_post(s_mesh_events, s_mesh_event_ble_rx);
					}
				}

				/* Reset for next frame */
				usb_frame_len = 0;
				usb_rx_idx = 0;
			}
		}
	}
}

/* USB DTR check - monitors DTR signal for disconnect detection
 * AND detects 1200bps baud rate for bootloader DFU trigger. */
static void usb_dtr_check_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!usb_dev) {
		return;
	}

	uint32_t dtr = 0;
	int err = uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
	if (err) {
		/* Line control not supported or error */
		k_work_schedule(&usb_dtr_check_work, K_MSEC(USB_DTR_CHECK_MS));
		return;
	}

	bool dtr_now = (dtr != 0);

	/* 1200bps DFU trigger: when host opens port at 1200 baud then
	 * drops DTR, reboot into bootloader DFU mode.
	 * This is the Arduino convention used by Adafruit nRF52 boards. */
	if (usb_dtr_active && !dtr_now) {
		uint32_t baud = 0;
		uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_BAUD_RATE, &baud);
		if (baud == 1200) {
			reboot_to_bootloader_dfu();
			/* Never returns */
		}

		/* Normal DTR drop - USB disconnected */
		LOG_INF("usb_dtr: DTR dropped, USB disconnected");
		if (zephcore_ble_get_active_iface() == ZEPHCORE_IFACE_USB) {
			zephcore_ble_set_active_iface(ZEPHCORE_IFACE_NONE);
			LOG_INF("usb_dtr: active_iface = IFACE_NONE");
		}
		/* Reset RX state */
		ring_buf_reset(&usb_ring_buf);
		usb_frame_len = 0;
		usb_rx_idx = 0;
	}

	usb_dtr_active = dtr_now;
	k_work_schedule(&usb_dtr_check_work, K_MSEC(USB_DTR_CHECK_MS));
}

/* Send frame over USB with V3 length prefix */
size_t zephcore_usb_companion_write_frame(const uint8_t *src, size_t len)
{
	if (!usb_dev || len == 0 || len > MAX_FRAME_SIZE) {
		return 0;
	}

	/* Send length prefix (little-endian) */
	uint8_t len_lo = (uint8_t)(len & 0xFF);
	uint8_t len_hi = (uint8_t)((len >> 8) & 0xFF);

	uart_poll_out(usb_dev, len_lo);
	uart_poll_out(usb_dev, len_hi);

	/* Send payload */
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(usb_dev, src[i]);
	}

	LOG_INF("usb_write_frame: sent len=%u hdr=0x%02x", (unsigned)len, src[0]);
	return len;
}

void zephcore_usb_companion_reset_rx(void)
{
	ring_buf_reset(&usb_ring_buf);
	usb_frame_len = 0;
	usb_rx_idx = 0;
}

void zephcore_usb_companion_init(struct k_event *mesh_events,
				 struct k_work *rx_work,
				 uint32_t mesh_event_ble_rx,
				 void *board)
{
	s_mesh_events = mesh_events;
	s_rx_work = rx_work;
	s_mesh_event_ble_rx = mesh_event_ble_rx;
	s_board = static_cast<mesh::ZephyrBoard *>(board);

#if DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart)
	usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (device_is_ready(usb_dev)) {
		LOG_INF("USB CDC device ready: %s", usb_dev->name);
		ring_buf_init(&usb_ring_buf, sizeof(usb_ring_buf_data), usb_ring_buf_data);

		/* Set up UART interrupt callback */
		uart_irq_callback_set(usb_dev, usb_uart_isr);
		uart_irq_rx_enable(usb_dev);

		/* Start DTR monitoring to detect terminal disconnect */
		k_work_schedule(&usb_dtr_check_work, K_MSEC(USB_DTR_CHECK_MS));
	} else {
		LOG_WRN("USB CDC device not ready");
		usb_dev = NULL;
	}
#endif
}
