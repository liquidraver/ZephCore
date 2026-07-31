/*
 * SPDX-License-Identifier: MIT
 * ZephCore USB CDC Companion Transport
 *
 * V3-framed USB CDC for companion mode: ISR, ring buffer, frame parsing,
 * DTR monitoring, DFU trigger, and frame write.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize USB CDC companion transport.
 * Sets up ring buffer, UART ISR, and DTR polling.
 * Call after device_is_ready() for the CDC ACM device.
 *
 * @param mesh_events  Pointer to the k_event used by the mesh event loop
 * @param mesh_event_ble_rx  Bitmask for MESH_EVENT_BLE_RX (posted to wake the
 *                           main thread when a complete frame is assembled)
 * @param board        Opaque pointer to ZephyrBoard (for DFU reboot)
 */
void zephcore_usb_companion_init(struct k_event *mesh_events,
				 uint32_t mesh_event_ble_rx,
				 void *board);

/**
 * Queue a frame for interrupt-driven USB TX (sync byte + LE length + payload).
 * @return number of payload bytes queued, or 0 if the TX ring can't fit the
 *         whole frame (caller should back off and retry on the drain callback)
 */
size_t zephcore_usb_companion_write_frame(const uint8_t *src, size_t len);

/**
 * @return true if the TX ring can hold one more frame carrying `payload_len`
 *         bytes (plus 3 bytes of framing). Used by the contact pump to pace
 *         itself against real TX backpressure instead of a fixed delay.
 */
bool zephcore_usb_companion_tx_has_space(size_t payload_len);

/**
 * Register a callback fired when the TX ring fully drains (USB analogue of the
 * BLE on_tx_idle notify-complete event). Used to resume the contact pump. May
 * be NULL. Invoked from the CDC TX interrupt-callback context.
 */
void zephcore_usb_companion_set_tx_drain_cb(void (*cb)(void));

/**
 * Reset USB RX state (e.g., on BLE disconnect).
 */
void zephcore_usb_companion_reset_rx(void);

/**
 * Register a callback invoked when a USB companion session starts — i.e. the
 * first inbound frame claims the interface for USB. Mirrors BLE on_connected
 * (e.g. light the UI connection indicator). May be NULL.
 */
void zephcore_usb_companion_set_session_start_cb(void (*cb)(void));

/**
 * Register a callback invoked when a USB companion session ends — i.e. the
 * host closed the port (DTR dropped) while USB owned the interface. Mirrors
 * the BLE on_disconnected cleanup: cancel in-flight contact iteration and
 * message sync, free the sign buffer. May be NULL.
 */
void zephcore_usb_companion_set_session_end_cb(void (*cb)(void));

/**
 * Register a callback fired when a complete text CLI line arrives over USB.
 * Activated when the first byte of a session is not the V3 sync byte ('<').
 * The line is null-terminated and has any trailing CR/LF stripped. May be NULL.
 */
void zephcore_usb_companion_set_cli_line_cb(void (*cb)(const char *line));

/**
 * Write a raw text reply to the USB CDC port (no V3 framing).
 * Used to send CLI responses when in text mode.
 */
void zephcore_usb_companion_write_text(const char *text, size_t len);

/**
 * True when the active USB session was opened by the text CLI rather than a
 * binary V3 companion app. Main uses this to suppress binary frame/push output
 * to USB so the serial console only ever receives text. False when no USB
 * session or when a binary companion owns the interface.
 */
bool zephcore_usb_companion_is_text_session(void);

#ifdef __cplusplus
}
#endif
