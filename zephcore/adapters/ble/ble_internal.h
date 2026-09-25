/*
 * SPDX-License-Identifier: MIT
 * ZephCore BLE - hooks between ZephyrBLE.cpp and the parts split out of it
 */

#pragma once

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>

/* GATT layout versioning / Service Changed (ble_gatt_layout.cpp,
 * CONFIG_BT_GATT_SERVICE_CHANGED). */
void ble_gatt_layout_check_after_settings_load(void);
void ble_gatt_layout_security_ready(struct bt_conn *conn);
void ble_gatt_layout_disconnected(void);
void ble_gatt_layout_paired(const bt_addr_le_t *addr);
void ble_gatt_layout_bond_deleted(const bt_addr_le_t *addr);

/* Buttonless DFU (ble_dfu.cpp, CONFIG_ZEPHCORE_BLE_DFU): the control-point
 * write asks ZephyrBLE to run main's on_dfu_request. */
void zephcore_ble_dfu_request(void);
