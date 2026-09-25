/*
 * SPDX-License-Identifier: MIT
 * ZephCore BLE Adapter — NUS service, advertising, security, TX/RX
 *
 * One companion transport among several: CompanionInterfaces.h wraps it as a
 * BaseSerialInterface for the MultiSerialInterface. Nothing here knows about
 * the other transports.
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "companion_framing.h"  /* MAX_FRAME_SIZE, struct frame, companion_link_cbs */

#ifdef __cplusplus
extern "C" {
#endif

struct ble_callbacks {
	/* Frame waiting (zephcore_ble_recv), TX drained, (dis)connected. Raised
	 * from the BT and system work queue threads: post events only. */
	struct companion_link_cbs link;
	/* Buttonless DFU control-point write — defer reboot into the
	 * bootloader's BLE OTA mode. May be NULL (feature disabled). */
	void (*on_dfu_request)(void);
};

/** Register callbacks and auth handlers. Call before bt_enable(). */
void zephcore_ble_init(const struct ble_callbacks *cbs);

/** Load settings, build adv data, start advertising if enabled. Call from bt_ready(). */
void zephcore_ble_start(const char *device_name);

/** Queue a frame for BLE TX. Returns bytes queued, or 0 on failure. */
size_t zephcore_ble_send(const uint8_t *data, uint16_t len);

/** Take the next received frame into dest (MAX_FRAME_SIZE). Returns its length, 0 if none. */
size_t zephcore_ble_recv(uint8_t *dest);

/** Enable/disable BLE. Disabling disconnects and stops advertising. Before
 *  zephcore_ble_start() this only records the wish, which start() honours. */
void zephcore_ble_set_enabled(bool enable);

/** True if BLE is enabled */
bool zephcore_ble_is_enabled(void);

/** True when a secured client is connected: frames can be exchanged. */
bool zephcore_ble_is_active(void);

/** True if BLE has a connection (secured or not). */
bool zephcore_ble_is_connected(void);

/** True while senders should hold off: TX congested, or the queue at its 2/3 high-water mark. */
bool zephcore_ble_is_write_busy(void);

/** True when every queued frame has been transmitted and link-layer acked:
 *  send queue empty, nothing in flight, no retry or overflow frame held back.
 *  True when disconnected (nothing to wait for). Reboot-class CLI commands
 *  poll this so a reset cannot cut off a reply or delivery-ack mid-flight. */
bool zephcore_ble_tx_idle(void);

/** True if the controller is currently broadcasting advertising PDUs.
 *  Returns FALSE during an active connection (Zephyr stops adv when the
 *  BT_MAX_CONN=1 slot is consumed) and FALSE after any explicit stop.
 *  Companion main loop polls this each housekeeping tick (~5s) and calls
 *  zephcore_ble_set_enabled(true) if adv ever stops outside a connection. */
bool zephcore_ble_is_advertising(void);

void zephcore_ble_set_passkey(uint32_t passkey);
uint32_t zephcore_ble_get_passkey(void);

/**
 * Apply deferred connection parameters.
 * Call after the initial app sync is complete (channels + contacts +
 * offline messages) so the param negotiation doesn't disrupt throughput
 * during the sync burst.
 */
void zephcore_ble_conn_params_ready(void);

/**
 * Rebuild advertising payload + GATT device name from a new prefs name.
 * If currently advertising (no active connection), stops and restarts
 * adv so the new name is published immediately. If connected, the new
 * payload takes effect on the next adv cycle after disconnect.
 *
 * Call from CMD_SET_ADVERT_NAME handler after persisting prefs.
 */
void zephcore_ble_update_name(const char *new_name);

#ifdef __cplusplus
}
#endif
