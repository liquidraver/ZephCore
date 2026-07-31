/*
 * SPDX-License-Identifier: MIT
 * ZephCore — Unified USB CDC ACM init (companion + repeater + observer)
 *
 * Owns the USBD device + CDC ACM class registration, the usbd_msg_callback
 * for 1200-baud touch DFU + DTR state tracking, and the boot-waiter event.
 * Replaces CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT auto-init so we control
 * the message callback path and can deliver event-driven DTR notifications
 * to consumers (no DTR polling work).
 */

#ifndef ZEPHCORE_ZEPHYR_USB_CDC_H
#define ZEPHCORE_ZEPHYR_USB_CDC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Per-transition DTR callback. Fires on every host CONTROL_LINE_STATE change,
 * value = current DTR level. NULL is allowed (cb cleared). */
typedef void (*zephcore_usbd_cdc_dtr_cb_t)(bool dtr_active);

/* Initialize USB device + CDC ACM class and enumerate.
 * Idempotent; subsequent calls are no-ops.
 * Returns 0 on success or a negative errno. */
int zephcore_usbd_init(void);

/* Block until the host opens the CDC ACM port (DTR transitions high) or the
 * timeout elapses. Returns 0 on DTR-high, -EAGAIN on timeout, -ENODEV if
 * zephcore_usbd_init() has not been called. */
int zephcore_usbd_wait_dtr(uint32_t timeout_ms);

/* Last-known DTR state from the most recent CONTROL_LINE_STATE message. */
bool zephcore_usbd_is_dtr_active(void);

/* Register a callback fired on every DTR transition.
 * Companion uses this to reset its RX state and flip active_iface on drop. */
void zephcore_usbd_set_dtr_cb(zephcore_usbd_cdc_dtr_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_ZEPHYR_USB_CDC_H */
