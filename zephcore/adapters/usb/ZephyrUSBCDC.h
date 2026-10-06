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
 * value = current DTR level, and with false when the host goes away without
 * one (VBUS loss, bus reset). NULL is allowed (cb cleared). */
typedef void (*zephcore_usbd_cdc_dtr_cb_t)(bool dtr_active);

/* Bus state callback: the bus was suspended, resumed or reset. Read the state
 * with zephcore_usbd_is_suspended(). NULL is allowed (cb cleared). */
typedef void (*zephcore_usbd_cdc_bus_cb_t)(void);

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
 * Companion uses this to end its session on a drop. */
void zephcore_usbd_set_dtr_cb(zephcore_usbd_cdc_dtr_cb_t cb);

/* The host has suspended the bus (asleep with the port open): nothing we send
 * leaves the device until it resumes. The USB stack's own state. */
bool zephcore_usbd_is_suspended(void);

/* Register a callback fired on a bus suspend, resume or reset. */
void zephcore_usbd_set_bus_cb(zephcore_usbd_cdc_bus_cb_t cb);

/* Detach from the USB bus (drop the D+ pull-up) so the host sees an unplug.
 * Call before a reset: a soft reset leaves the PHY attached on some SoCs, and
 * the host then keeps a stale endpoint open across the reboot. No-op if the
 * stack was never initialized; after this, zephcore_usbd_init() would have to
 * run again to come back. */
void zephcore_usbd_detach(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHCORE_ZEPHYR_USB_CDC_H */
