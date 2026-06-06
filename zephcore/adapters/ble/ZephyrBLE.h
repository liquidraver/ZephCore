/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore BLE Adapter — NUS service, advertising, security, TX/RX
 */
#pragma once

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Maximum companion transport frame size.
 * +4 over the base 172 to accommodate transport codes (region scoping). */
#define MAX_FRAME_SIZE  176

#ifdef __cplusplus
extern "C" {
#endif

/* Callbacks from BLE adapter to main */
struct ble_callbacks {
	/* RX frame received; runs on system work queue — must not block */
	void (*on_rx_frame)(const uint8_t *data, uint16_t len);
	/* TX queue drained */
	void (*on_tx_idle)(void);
	/* BLE connected */
	void (*on_connected)(void);
	/* BLE disconnected */
	void (*on_disconnected)(void);
	/* Buttonless DFU control-point write — defer reboot into the
	 * bootloader's BLE OTA mode. May be NULL (feature disabled). */
	void (*on_dfu_request)(void);
};

enum zephcore_iface {
	ZEPHCORE_IFACE_NONE,
	ZEPHCORE_IFACE_BLE,
	ZEPHCORE_IFACE_USB,
};

/** Register callbacks and auth handlers. Call before bt_enable(). */
void zephcore_ble_init(const struct ble_callbacks *cbs);

/** Load settings, build adv data, start advertising. Call from bt_ready(). */
void zephcore_ble_start(const char *device_name);

/** Queue a frame for BLE TX. Returns bytes queued, or 0 on failure. */
size_t zephcore_ble_send(const uint8_t *data, uint16_t len);

/** Enable/disable BLE. Disabling disconnects and stops advertising. */
void zephcore_ble_set_enabled(bool enable);

/** True if BLE is enabled */
bool zephcore_ble_is_enabled(void);

/** True if BLE is the active transport and ready to send. */
bool zephcore_ble_is_active(void);

/** True if BLE has an active connection (regardless of interface state). */
bool zephcore_ble_is_connected(void);

/** True if TX queue is full and overflow retry is active. */
bool zephcore_ble_is_congested(void);

/** True if the controller is currently broadcasting advertising PDUs.
 *  Returns FALSE during an active connection (Zephyr stops adv when the
 *  BT_MAX_CONN=1 slot is consumed) and FALSE after any explicit stop.
 *  Companion main loop polls this each housekeeping tick (~5s) and calls
 *  zephcore_ble_set_enabled(true) if adv ever stops outside a connection. */
bool zephcore_ble_is_advertising(void);

void zephcore_ble_set_passkey(uint32_t passkey);
uint32_t zephcore_ble_get_passkey(void);

/** Get/set active interface (BLE/USB coexistence). Both are thread-safe —
 *  active_iface is mutated from the BLE callback thread and the USB workqueue. */
enum zephcore_iface zephcore_ble_get_active_iface(void);
void zephcore_ble_set_active_iface(enum zephcore_iface iface);

/** Atomically claim the active interface for `who` unless the other transport
 *  already owns it. Succeeds (returns true) if the interface is idle or already
 *  held by `who`; fails if a different interface is active. Thread-safe — use
 *  this instead of a get-then-set sequence to avoid a check-then-act race. */
bool zephcore_ble_iface_try_claim(enum zephcore_iface who);

/** Get recv/send queues for USB path sharing. */
struct k_msgq *zephcore_ble_get_recv_queue(void);
struct k_msgq *zephcore_ble_get_send_queue(void);

/** Kick the TX drain work. Call after putting frames in the send queue. */
void zephcore_ble_kick_tx(void);

/**
 * Disconnect the current BLE connection (if any).
 * Used by USB when it takes over as active interface.
 */
void zephcore_ble_disconnect(void);

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
