/*
 * SPDX-License-Identifier: MIT
 * ZephCore BLE - GATT layout versioning and Service Changed
 *
 * When the static GATT table changes shape, bonded phones still hold the old
 * attribute handles. A per-peer layout version in settings decides whether a
 * reconnecting bonded peer gets a Service Changed indication. Split out of
 * ZephyrBLE.cpp; ZephyrBLE calls the ble_gatt_layout_* hooks.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zephcore_ble, CONFIG_ZEPHCORE_BLE_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "ble_internal.h"

/* Bump when static BT_GATT_SERVICE_DEFINE layout or registration order changes.
 * 1 = pre-2cf4b97 (dfu_svc before secure_nus_svc)
 * 2 = secure_nus_svc_dfu after secure_nus_svc (+ packet/revision chars) */
#define ZEPHCORE_GATT_LAYOUT_VERSION 2

/* Delay before sending Service Changed after L2 security is established.
 * On a fresh pairing, pairing_complete() fires within this window and marks the
 * peer current, so only genuine bonded reconnects with a stale layout get an SC.
 * The delay also gives the peer time to subscribe to the Service Changed CCC. */
#define GATT_SC_INDICATE_DELAY_MS 1000

static bool gatt_sc_ind_in_flight;
static const struct bt_gatt_attr *gatt_sc_value_attr;
static struct bt_gatt_indicate_params gatt_sc_ind_params;
static uint16_t gatt_sc_ind_range[2];

static void gatt_peer_settings_key(char *key, size_t key_len, const bt_addr_le_t *addr)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	snprintk(key, key_len, "ble/gatt_peer/%s", addr_str);
}

static int gatt_peer_layout_load(const bt_addr_le_t *addr, uint8_t *ver_out)
{
#if IS_ENABLED(CONFIG_SETTINGS)
	char key[64];
	ssize_t len;

	gatt_peer_settings_key(key, sizeof(key), addr);
	len = settings_load_one(key, ver_out, sizeof(*ver_out));
	if (len == (ssize_t)sizeof(*ver_out)) {
		return 0;
	}
#endif
	return -ENOENT;
}

static int gatt_peer_layout_save(const bt_addr_le_t *addr, uint8_t ver)
{
#if IS_ENABLED(CONFIG_SETTINGS)
	char key[64];
	int err;

	gatt_peer_settings_key(key, sizeof(key), addr);
	err = settings_save_one(key, &ver, sizeof(ver));
	return err;
#else
	ARG_UNUSED(addr);
	ARG_UNUSED(ver);
	return -ENOTSUP;
#endif
}

static void gatt_peer_layout_delete(const bt_addr_le_t *addr)
{
#if IS_ENABLED(CONFIG_SETTINGS)
	char key[64];

	gatt_peer_settings_key(key, sizeof(key), addr);
	settings_delete(key);
#else
	ARG_UNUSED(addr);
#endif
}

static int gatt_global_layout_save(uint8_t ver)
{
#if IS_ENABLED(CONFIG_SETTINGS)
	return settings_save_one("ble/gatt_layout", &ver, sizeof(ver));
#else
	ARG_UNUSED(ver);
	return -ENOTSUP;
#endif
}

struct gatt_layout_bond_ctx {
	int count;
};

static void gatt_layout_count_bonds(const struct bt_bond_info *info, void *user_data)
{
	struct gatt_layout_bond_ctx *ctx =
		static_cast<struct gatt_layout_bond_ctx *>(user_data);

	ARG_UNUSED(info);
	ctx->count++;
}

void ble_gatt_layout_check_after_settings_load(void)
{
#if IS_ENABLED(CONFIG_SETTINGS)
	struct gatt_layout_bond_ctx bond_ctx = { 0 };
	uint8_t global;
	uint8_t prev;
	ssize_t len;

	bt_foreach_bond(BT_ID_DEFAULT, gatt_layout_count_bonds, &bond_ctx);

	len = settings_load_one("ble/gatt_layout", &global, sizeof(global));
	if (len != (ssize_t)sizeof(global)) {
		if (bond_ctx.count == 0) {
			/* Fresh device — no stale phone caches to fix. */
			if (gatt_global_layout_save(ZEPHCORE_GATT_LAYOUT_VERSION) == 0) {
				LOG_DBG("GATT layout v%u seeded (no bonds)",
					ZEPHCORE_GATT_LAYOUT_VERSION);
			}
			return;
		}

		prev = 1; /* bonded before layout tracking existed */
	} else {
		prev = global;
	}

	if (prev != ZEPHCORE_GATT_LAYOUT_VERSION) {
		LOG_INF("GATT layout v%u -> v%u (per-peer SC on connect)",
			prev, ZEPHCORE_GATT_LAYOUT_VERSION);
		if (gatt_global_layout_save(ZEPHCORE_GATT_LAYOUT_VERSION) != 0) {
			LOG_WRN("GATT layout global version save failed");
		}
	}
#endif
}

static uint8_t gatt_sc_find_value_attr(const struct bt_gatt_attr *attr, uint16_t handle,
				       void *user_data)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(user_data);

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		const struct bt_gatt_chrc *chrc =
			static_cast<const struct bt_gatt_chrc *>(attr->user_data);

		if (!bt_uuid_cmp(chrc->uuid, BT_UUID_GATT_SC)) {
			gatt_sc_value_attr = bt_gatt_attr_next(attr);
			return BT_GATT_ITER_STOP;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static void gatt_sc_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
				uint8_t err)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);

	ARG_UNUSED(params);

	gatt_sc_ind_in_flight = false;

	if (err) {
		LOG_WRN("Service Changed indicate failed: 0x%02x (will retry)", err);
		return;
	}

	if (gatt_peer_layout_save(addr, ZEPHCORE_GATT_LAYOUT_VERSION) != 0) {
		LOG_WRN("GATT peer layout save failed (will retry on reconnect)");
		return;
	}

	LOG_INF("GATT layout v%u: Service Changed confirmed", ZEPHCORE_GATT_LAYOUT_VERSION);
}

static void gatt_peer_mark_current_no_sc(const bt_addr_le_t *addr)
{
	if (gatt_peer_layout_save(addr, ZEPHCORE_GATT_LAYOUT_VERSION) != 0) {
		LOG_WRN("GATT peer layout save failed");
		return;
	}

	LOG_DBG("GATT layout v%u: peer marked current (no SC needed)", ZEPHCORE_GATT_LAYOUT_VERSION);
}

static void maybe_indicate_service_changed(struct bt_conn *conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	uint8_t peer_ver;
	int err;

	if (gatt_sc_ind_in_flight) {
		return;
	}

	if (gatt_peer_layout_load(addr, &peer_ver) == 0 &&
	    peer_ver == ZEPHCORE_GATT_LAYOUT_VERSION) {
		return;
	}

	if (!gatt_sc_value_attr) {
		bt_gatt_foreach_attr(0x0001, 0xffff, gatt_sc_find_value_attr, NULL);
		if (!gatt_sc_value_attr) {
			LOG_ERR("Service Changed characteristic not found");
			return;
		}
	}

	gatt_sc_ind_range[0] = sys_cpu_to_le16(0x0001);
	gatt_sc_ind_range[1] = sys_cpu_to_le16(0xffff);

	memset(&gatt_sc_ind_params, 0, sizeof(gatt_sc_ind_params));
	gatt_sc_ind_params.attr = gatt_sc_value_attr;
	gatt_sc_ind_params.func = gatt_sc_indicate_cb;
	gatt_sc_ind_params.data = gatt_sc_ind_range;
	gatt_sc_ind_params.len = sizeof(gatt_sc_ind_range);

	err = bt_gatt_indicate(conn, &gatt_sc_ind_params);
	if (err) {
		LOG_WRN("Service Changed indicate err %d (will retry)", err);
		return;
	}

	gatt_sc_ind_in_flight = true;
	LOG_INF("GATT layout migration: Service Changed indicated");
}

/* The connection security_changed() handed us. Not a reference: ZephyrBLE
 * holds that, and ble_gatt_layout_disconnected() clears this and cancels the
 * work before the connection goes away. */
static struct bt_conn *sc_conn;

/* Deferred from security_changed() — see GATT_SC_INDICATE_DELAY_MS. Runs on the
 * single active connection (BT_MAX_CONN=1); cancelled on disconnect. */
static void gatt_sc_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (sc_conn) {
		maybe_indicate_service_changed(sc_conn);
	}
}
static K_WORK_DELAYABLE_DEFINE(gatt_sc_work, gatt_sc_work_fn);

void ble_gatt_layout_security_ready(struct bt_conn *conn)
{
	sc_conn = conn;
	k_work_reschedule(&gatt_sc_work, K_MSEC(GATT_SC_INDICATE_DELAY_MS));
}

void ble_gatt_layout_disconnected(void)
{
	k_work_cancel_delayable(&gatt_sc_work);
	gatt_sc_ind_in_flight = false;
	sc_conn = NULL;
}

void ble_gatt_layout_paired(const bt_addr_le_t *addr)
{
	gatt_peer_mark_current_no_sc(addr);
}

void ble_gatt_layout_bond_deleted(const bt_addr_le_t *addr)
{
	gatt_peer_layout_delete(addr);
}
