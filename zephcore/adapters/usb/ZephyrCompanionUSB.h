/*
 * SPDX-License-Identifier: MIT
 * ZephCore wired companion transport (USB CDC or plain UART)
 *
 * V3 framing plus the text CLI: ISR, ring buffers, frame parser, DTR
 * monitoring. One transport among several: CompanionInterfaces.h wraps it as
 * a BaseSerialInterface for the MultiSerialInterface.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "companion_framing.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the transport: ring buffers, UART ISR, DTR tracking.
 * @param link  frame waiting / TX drained / session start / session end
 *              (DTR drop). Raised from the ISR and sysworkq: post events only.
 */
void zephcore_usb_companion_init(const struct companion_link_cbs *link);

/**
 * Queue a frame for interrupt-driven TX (sync byte + LE length + payload).
 * @return number of payload bytes queued, or 0 if the TX ring can't fit the
 *         whole frame (caller should back off and retry on TX idle)
 */
size_t zephcore_usb_companion_write_frame(const uint8_t *src, size_t len);

/** Take the next received binary frame into dest (MAX_FRAME_SIZE). Returns its length, 0 if none. */
size_t zephcore_usb_companion_recv(uint8_t *dest);

/** A binary companion session is open (a text-CLI session is not a client). */
bool zephcore_usb_companion_is_connected(void);

/** The TX ring cannot take another full-size frame. */
bool zephcore_usb_companion_is_write_busy(void);

/**
 * @return true if the TX ring can hold one more frame carrying `payload_len`
 *         bytes (plus 3 bytes of framing).
 */
bool zephcore_usb_companion_tx_has_space(size_t payload_len);

/** True when the companion TX ring has fully drained. True when no USB device
 *  is bound (nothing to wait for). Counterpart to zephcore_ble_tx_idle(). */
bool zephcore_usb_companion_tx_idle(void);

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
 * True when the USB session was opened by the text CLI rather than a binary
 * V3 companion app. Such a session reports not connected, so no binary frame
 * or push reaches the serial console.
 */
bool zephcore_usb_companion_is_text_session(void);

#ifdef __cplusplus
}
#endif
