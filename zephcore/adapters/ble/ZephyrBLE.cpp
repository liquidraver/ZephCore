/*
 * SPDX-License-Identifier: MIT
 * ZephCore BLE Adapter — NUS service, advertising, security, TX/RX
 *
 * Security: SMP pairing with SC + MITM + Bonding, DisplayOnly IO (app_passkey).
 * Pairing is triggered reactively by ATT_ERR_AUTHENTICATION on secured GATT
 * attributes (Apple §55 compliant — no proactive Security Request on connect).
 *
 * Advertising: always uses the identity address (BT_LE_ADV_OPT_USE_IDENTITY in
 * start_adv).  On ESP32, CONFIG_BT_PRIVACY=y is load-bearing (the Espressif
 * controller's privacy-OFF Secure-Connections path MIC-fails against iOS — see
 * findings.md Issue #34), but USE_IDENTITY keeps us off the RPA so the Android
 * companion's "connect from app" still works.  nRF/MG24 keep privacy OFF and
 * already advertise identity, so USE_IDENTITY is a no-op there.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_ble, CONFIG_ZEPHCORE_BLE_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

#include "ZephyrBLE.h"
#include "ble_internal.h"
#include "frame_txq.h"

/* ========== Constants ========== */

#define DEVICE_NAME_MAX 29
#define FRAME_QUEUE_SIZE CONFIG_ZEPHCORE_BLE_QUEUE_SIZE
#define BLE_TX_POWER 8
#define BLE_TX_RETRY_MS 20

/* BLE connection parameters */
#define BLE_DEFAULT_MIN_INTERVAL CONFIG_ZEPHCORE_BLE_CONN_MIN_INTERVAL
#define BLE_DEFAULT_MAX_INTERVAL CONFIG_ZEPHCORE_BLE_CONN_MAX_INTERVAL
#define BLE_DEFAULT_LATENCY      CONFIG_ZEPHCORE_BLE_CONN_LATENCY
#define BLE_DEFAULT_TIMEOUT      CONFIG_ZEPHCORE_BLE_CONN_TIMEOUT

/* TX timeout watchdog - reset ble_tx_in_progress if callback never fires */
#define BLE_TX_TIMEOUT_MS 2000



/* Advertising intervals (Apple Accessory Design Guidelines §5.5) */
#define BT_ADV_FAST_INTERVAL     32            /* 20ms in 0.625ms units */
#define BT_ADV_FAST_DURATION_MS  (60 * 1000)  /* fast window after boot/disconnect */
#define BT_ADV_INTERVAL          CONFIG_ZEPHCORE_BLE_ADV_SLOW_INTERVAL

/* ========== Static state ========== */

/* Deferred connection parameter update — applied after initial sync
 * completes (NO_MORE_MSGS) rather than immediately on security_changed,
 * because the negotiation disrupts BLE throughput during channel/contact sync. */
static bool conn_params_pending;

/* Callbacks to main */
static const struct ble_callbacks *ble_cbs;

#if IS_ENABLED(CONFIG_ZEPHCORE_BLE_DFU)
void zephcore_ble_dfu_request(void)
{
	if (ble_cbs && ble_cbs->on_dfu_request) {
		ble_cbs->on_dfu_request();
	}
}
#endif

/* Advertising data */
static char device_name[DEVICE_NAME_MAX];
static const uint8_t ad_flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
static const int8_t ad_tx_power = BLE_TX_POWER;
static const uint8_t nus_uuid[] = { BT_UUID_NUS_SRV_VAL };
static struct bt_data ad[3];
static struct bt_data sd[1];
static size_t ad_len;
static size_t sd_len;

/* Queues — ISR-safe, no mutex needed. The send queue is driven through
 * ble_txq: congestion, the overflow slot and the lossless rule (frame_txq.h). */
K_MSGQ_DEFINE(ble_send_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ble_recv_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
static struct frame_txq ble_txq;

/* TX retry buffer - used when BLE returns -ENOMEM/-EAGAIN */
static struct frame tx_retry_frame;
static bool tx_retry_pending = false;

/* Connection state */
static struct bt_conn *current_conn;
static bool nus_notif_enabled;
static bool ble_tx_ready = false;
static bool ble_tx_in_progress = false;
static int64_t ble_tx_start_time = 0;


/* DLE tracking — set after successful DLE request to avoid double-request */
static bool dle_requested;

/* Fast advertising — true for BT_ADV_FAST_DURATION_MS after boot or disconnect */
static bool fast_adv_active;

/* Set before bt_le_adv_stop() in adv_slow_work_fn to suppress recycled() from
 * restarting fast advertising. Zephyr fires recycled() when the pre-allocated
 * connection slot is freed after any adv stop — not just on disconnect.
 * Cleared by recycled() itself (both run on the cooperative system work queue). */
static bool adv_stop_for_interval_change;

/* Ground truth for "controller is currently broadcasting adv PDUs":
 *   set TRUE  : bt_le_adv_start() returned success
 *   set FALSE : bt_le_adv_stop() called explicitly  (set_enabled(false),
 *               adv_slow_work interval change, update_name restart)
 *   set FALSE : a phone connected — Zephyr stops adv internally to consume
 *               the BT_MAX_CONN=1 slot (no slot left to advertise from).
 *               Re-set TRUE later when recycled() → start_adv() runs.
 * Exposed via zephcore_ble_is_advertising() for the companion advertising
 * watchdog (main_companion.cpp housekeeping) that catches transient
 * bt_le_adv_start failures.  Arduino nrf52 has an equivalent 10s watchdog
 * (SerialBLEInterface.cpp:343). */
static bool adv_running;

/* Administrative BLE state */
static bool ble_enabled = true;

/* zephcore_ble_start() has run: advertising can be started or stopped */
static bool ble_started;

/* Runtime BLE passkey */
static uint32_t ble_passkey = CONFIG_ZEPHCORE_BLE_PASSKEY;

/* NUS TX characteristic attribute — resolved at init, avoids hard-coded offset */
static const struct bt_gatt_attr *nus_tx_attr;

/* ========== Forward declarations ========== */

static void ble_tx_complete_cb(struct bt_conn *conn, void *user_data);
static int secure_nus_send(struct bt_conn *conn, const void *data, uint16_t len);
static void secure_nus_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
static void request_dle(struct bt_conn *conn);
#endif
static ssize_t secure_nus_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static void start_adv(void);
static void start_fast_adv(void);
static void kick_tx_drain(void);

/* ========== GATT Service ========== */

/*
 * NUS service — secured with AUTHEN permissions.
 * Matches Arduino's SECMODE_ENC_WITH_MITM on bleuart.
 *
 * When the phone tries to subscribe (CCC write) or send data (RX write),
 * Zephyr returns ATT_ERR_AUTHENTICATION. The phone's BLE stack then
 * initiates pairing (PIN dialog). After pairing succeeds,
 * security_changed() fires at L3+ and the phone retries the operation.
 */
BT_GATT_SERVICE_DEFINE(secure_nus_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX_CHAR,
		BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_NONE,
		NULL, NULL, NULL),
	BT_GATT_CCC(secure_nus_ccc_changed,
		BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX_CHAR,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE_AUTHEN,
		NULL, secure_nus_rx_write, NULL),
);


/* ========== Work items ========== */

static void tx_drain_work_fn(struct k_work *work);
static void adv_slow_work_fn(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(tx_drain_work, tx_drain_work_fn);
K_WORK_DELAYABLE_DEFINE(adv_slow_work, adv_slow_work_fn);

/* ========== Unpaired-connection timeout ==========
 *
 * A connection that never reaches L2 holds the node's only peripheral slot.
 * With CONFIG_BT_MAX_CONN=1 Zephyr stops advertising while that slot is taken,
 * and the companion's advertising watchdog (main_companion.cpp) deliberately
 * skips any state where a connection exists — so a client that connects and
 * never pairs makes the node invisible to everyone else until it is power
 * cycled.  A BLE scanner app left connected does it by accident; iOS does it
 * routinely.  Nothing else times the connection out: pairing here is reactive
 * by design (Apple §55 — we never send a Security Request, we wait for the
 * phone to hit ATT insufficient-authentication and start pairing itself), so
 * "connected but idle forever" is a state the node otherwise accepts happily.
 *
 * Dropping it costs a legitimate client nothing: every characteristic on both
 * services is *_AUTHEN, so an unsecured connection cannot read, write or
 * subscribe to anything.  The window has to cover discovery plus the phone's
 * own pairing dialog — 15 s matches upstream MeshCore PR #3263, which measured
 * a real unpaired connection being dropped at ~13 s. */
#define BLE_SECURITY_TIMEOUT_MS 15000

static void sec_timeout_conn_cb(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(user_data);

	if (bt_conn_get_security(conn) >= BT_SECURITY_L2) {
		return;
	}

	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("security timeout: %s unpaired after %d ms, disconnecting",
		addr, BLE_SECURITY_TIMEOUT_MS);

	/* recycled() restarts advertising once the stack releases the slot. */
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

/* Runs on the system work queue; current_conn belongs to the Bluetooth
 * callback thread.  The connection is therefore reached through
 * bt_conn_foreach(), which hands the callback a reference held for its
 * duration, rather than by dereferencing current_conn across threads. */
static void sec_timeout_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	bt_conn_foreach(BT_CONN_TYPE_LE, sec_timeout_conn_cb, NULL);
}

K_WORK_DELAYABLE_DEFINE(sec_timeout_work, sec_timeout_work_fn);

/* ========== TX completion callback ========== */

/*
 * TX completion callback - chains to next frame (event-driven like Arduino)
 * This is called by bt_gatt_notify_cb when the notification is sent over the air.
 */
static void ble_tx_complete_cb(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(user_data);

	ble_tx_in_progress = false;

	/* Chain to next frame immediately via work queue */
	k_work_schedule(&tx_drain_work, K_NO_WAIT);
}

/* Helper function to send via our secure NUS TX characteristic with callback */
static int secure_nus_send(struct bt_conn *conn, const void *data, uint16_t len)
{
	struct bt_gatt_notify_params params = {
		.attr = nus_tx_attr,
		.data = data,
		.len = len,
		.func = ble_tx_complete_cb,
		.user_data = NULL,
	};

	return bt_gatt_notify_cb(conn, &params);
}

/* ========== Device name and advertising data ========== */

static void build_device_name_and_adv(const char *name_from_prefs)
{
	if (name_from_prefs && name_from_prefs[0]) {
		/* Prepend "MeshCore-" prefix so apps that filter on it can find us */
		snprintf(device_name, sizeof(device_name), "MeshCore-%s", name_from_prefs);

		/* Sanitize the BLE-advertised name to printable ASCII, compacting
		 * in place (write index w never outpaces read index r):
		 *   - ':' / ';'        -> '-'  (Apple Accessory Design Guidelines)
		 *   - non-ASCII bytes  -> dropped (e.g. emoji in the node name).
		 *     iOS's BLE scanner blanks the WHOLE advertised name if it
		 *     contains any non-ASCII byte, showing the device nameless.
		 * This only sanitizes the BLE/GAP copy; the mesh node name (emoji
		 * and all) is untouched and still shown by the companion app.
		 */
		size_t w = 0;
		for (size_t r = 0; device_name[r]; r++) {
			unsigned char c = (unsigned char)device_name[r];
			if (c == ':' || c == ';') {
				device_name[w++] = '-';
			} else if (c >= 0x20 && c < 0x7F) {
				device_name[w++] = (char)c;
			}
			/* else: drop control / non-ASCII (multi-byte UTF-8) bytes */
		}
		device_name[w] = '\0';
	} else {
		/* Fallback - should never happen since prefs.node_name has default */
		snprintf(device_name, sizeof(device_name), "MeshCore");
	}

	bt_set_name(device_name);

	ad[0].type = BT_DATA_FLAGS;
	ad[0].data_len = 1;
	ad[0].data = &ad_flags;
	ad[1].type = BT_DATA_TX_POWER;
	ad[1].data_len = 1;
	ad[1].data = (const uint8_t *)&ad_tx_power;
	ad[2].type = BT_DATA_UUID128_ALL;
	ad[2].data_len = sizeof(nus_uuid);
	ad[2].data = nus_uuid;
	ad_len = 3;

	sd[0].type = BT_DATA_NAME_COMPLETE;
	sd[0].data_len = (uint8_t)strlen(device_name);
	sd[0].data = (const uint8_t *)device_name;
	sd_len = 1;

	LOG_DBG("%s", device_name);
}

/* ========== Connection callbacks ========== */

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_WRN("connection failed: %s err 0x%02x", addr, err);
		return;
	}
	LOG_INF("connected: %s", addr);

	current_conn = bt_conn_ref(conn);

	/* Zephyr stops advertising internally when the conn slot is consumed
	 * (BT_MAX_CONN=1 — there's no slot left to advertise from).  Sync our
	 * adv_running flag so zephcore_ble_is_advertising() reflects ground
	 * truth, not just "we last called bt_le_adv_start()". */
	adv_running = false;

	/* Cancel fast→slow transition — already connected, no need to switch */
	k_work_cancel_delayable(&adv_slow_work);

	/* Arm the unpaired-connection timeout.  Cancelled by security_changed()
	 * at L2+, and by disconnected() whichever way the connection ends. */
	k_work_reschedule(&sec_timeout_work, K_MSEC(BLE_SECURITY_TIMEOUT_MS));

	/* DLE is NOT requested here — the phone may start a PHY update LL
	 * procedure immediately, and BLE allows only one at a time.
	 * DLE is deferred to le_phy_updated() (after PHY negotiation completes)
	 * with a fallback in security_changed() if PHY update never fires. */
	dle_requested = false;

	/* Do NOT proactively request security here.
	 *
	 * Apple Accessory Design Guidelines §55 (Pairing): the accessory should
	 * not request pairing until an ATT request is rejected with "Insufficient
	 * Authentication."  Pairing is triggered reactively when the phone tries
	 * to access our AUTHEN-secured GATT attributes (CCC write / RX write).
	 *
	 * For bonded reconnects, Zephyr auto-encrypts with stored keys when
	 * CONFIG_BT_SMP and CONFIG_BT_BONDABLE are enabled. */

	/* Notify main of BLE connection */
	if (ble_cbs && ble_cbs->link.on_connected) {
		ble_cbs->link.on_connected();
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("disconnected: %s reason 0x%02x", addr, reason);

	k_work_cancel_delayable(&sec_timeout_work);

	if (conn == current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	nus_notif_enabled = false;

	/* Reset BLE TX state */
	ble_tx_in_progress = false;
	ble_tx_ready = false;
	conn_params_pending = false;

	/* Clear queues, retry state, and congestion */
	frame_txq_reset(&ble_txq);
	k_msgq_purge(&ble_recv_queue);
	tx_retry_pending = false;

	k_work_cancel_delayable(&tx_drain_work);

#if IS_ENABLED(CONFIG_BT_GATT_SERVICE_CHANGED)
	ble_gatt_layout_disconnected();
#endif

	/* Notify main of BLE disconnection */
	if (ble_cbs && ble_cbs->link.on_disconnected) {
		ble_cbs->link.on_disconnected();
	}
}

static void recycled(void)
{
	if (adv_stop_for_interval_change) {
		adv_stop_for_interval_change = false;
		return;
	}
	LOG_DBG("restart advertising");
	start_fast_adv();
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_WRN("security failed: %s level %u err %d", addr, level, err);
		return;
	}
	LOG_INF("%s level %u", addr, level);

	/* Paired (or bonded reconnect) — the connection has earned its slot,
	 * so stand the unpaired-connection timeout down. */
	if (level >= BT_SECURITY_L2) {
		k_work_cancel_delayable(&sec_timeout_work);
	}

	/* Enable TX when we have sufficient security (level 2+ = encrypted).
	 * This is the ONLY place that sets ble_tx_ready — security_changed is
	 * the authority.  CCC subscription (secure_nus_ccc_changed) only kicks
	 * the TX drain; it never sets ble_tx_ready.
	 */
	if (level >= BT_SECURITY_L2 && !ble_tx_ready) {
		LOG_INF("security established, enabling TX");
		ble_tx_ready = true;

		/* If CCC was already subscribed (bonded reconnect — phone writes
		 * CCC before security_changed fires), kick TX now.  On fresh
		 * pairing CCC hasn't been written yet, so this is a no-op and
		 * TX starts when CCC fires later. */
		if (nus_notif_enabled) {
			kick_tx_drain();
		}
	}

	if (level >= BT_SECURITY_L2) {
#if IS_ENABLED(CONFIG_BT_GATT_SERVICE_CHANGED)
		/* After a GATT layout change, bonded phones may hold stale handles.
		 * Service Changed forces rediscovery without breaking the bond.
		 * Deferred so a fresh pairing (pairing_complete marks the peer
		 * current) doesn't trigger a needless SC. */
		ble_gatt_layout_security_ready(conn);
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
		/* Fallback DLE request — if le_phy_updated() already sent it,
		 * request_dle() returns immediately (dle_requested flag). */
		request_dle(conn);
#endif
		/* Defer connection parameter update until after the initial
		 * app sync finishes (channels + contacts + offline messages).
		 * Requesting a param change now would disrupt BLE throughput
		 * during the sync burst.  CompanionMesh calls
		 * zephcore_ble_conn_params_ready() when sync is done. */
		conn_params_pending = true;
		LOG_INF("conn param update deferred until post-sync");
	}
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			    uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	LOG_INF("BLE conn params updated: interval=%dms latency=%d timeout=%dms",
		interval * 5 / 4, latency, timeout * 10);
}

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
/* Request max DLE (251 bytes).  Called from le_phy_updated() after PHY
 * negotiation completes, and from security_changed() as a fallback if
 * PHY update never fires.  The dle_requested flag prevents double-request. */
static void request_dle(struct bt_conn *conn)
{
	if (dle_requested) {
		return;
	}
	struct bt_conn_le_data_len_param data_len_param = {
		.tx_max_len = BT_GAP_DATA_LEN_MAX,
		.tx_max_time = BT_GAP_DATA_TIME_MAX,
	};
	int err = bt_conn_le_data_len_update(conn, &data_len_param);
	if (err) {
		LOG_WRN("Failed to request data length update: %d", err);
	} else {
		LOG_INF("Requested max data length (251 bytes)");
		dle_requested = true;
	}
}

static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	ARG_UNUSED(conn);
	LOG_INF("BLE data length updated: TX=%u/%uus RX=%u/%uus",
		info->tx_max_len, info->tx_max_time,
		info->rx_max_len, info->rx_max_time);
}
#endif

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static const char *phy_name(uint8_t phy)
{
	switch (phy) {
	case BT_GAP_LE_PHY_1M:    return "1M";
	case BT_GAP_LE_PHY_2M:    return "2M";
	case BT_GAP_LE_PHY_CODED: return "Coded";
	default:                   return "Unknown";
	}
}

static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	LOG_INF("BLE PHY updated: TX=%s RX=%s", phy_name(param->tx_phy),
		phy_name(param->rx_phy));

	/* Accept whatever PHY the phone chose — overriding 2M with Coded|1M
	 * caused iPhone to freeze the connection (and the whole node).
	 * PHY is settled — request DLE.  Deferred here from connected()
	 * because the phone starts a PHY procedure on connect and BLE
	 * only allows one LL procedure at a time. */
	request_dle(conn);
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.le_param_updated = le_param_updated,
	.security_changed = security_changed,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
	.le_phy_updated = le_phy_updated,
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
	.le_data_len_updated = le_data_len_updated,
#endif
};

/* ========== Authentication callbacks ========== */

static uint32_t auth_app_passkey(struct bt_conn *conn)
{
	return ble_passkey;
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	LOG_WRN("pairing cancelled");
}

static struct bt_conn_auth_cb auth_cb = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
	.app_passkey = auth_app_passkey,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	LOG_INF("pairing complete: bonded=%d", bonded);

#if IS_ENABLED(CONFIG_BT_GATT_SERVICE_CHANGED)
	/* Fresh pairing always does full GATT discovery — no Service Changed needed. */
	if (bonded) {
		ble_gatt_layout_paired(bt_conn_get_dst(conn));
	}
#endif
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_WRN("pairing failed: reason %d", reason);
	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

#if IS_ENABLED(CONFIG_BT_GATT_SERVICE_CHANGED)
/* Fires on explicit unpair and on BT_MAX_PAIRED overwrite-oldest eviction (both
 * route through bt_unpair). Drop the per-peer layout key so /lfs/settings does
 * not accumulate orphaned ble/gatt_peer entries over the device lifetime. */
static void bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	ARG_UNUSED(id);
	ble_gatt_layout_bond_deleted(peer);
}
#endif

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
#if IS_ENABLED(CONFIG_BT_GATT_SERVICE_CHANGED)
	.bond_deleted = bond_deleted,
#endif
};

/* ========== TX drain work ========== */

static void kick_tx_drain(void)
{
	k_work_schedule(&tx_drain_work, K_NO_WAIT);
}

/* The txq abandons its parked push when this goes false */
static bool ble_link_up(void)
{
	return current_conn != NULL;
}

static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	struct frame f;
	int err;

	/*
	 * BLE TX path - Event-driven (like Arduino's HVN_TX_COMPLETE)
	 * Uses bt_gatt_notify_cb() callback to chain TX without polling.
	 * Re-entrancy guard prevents concurrent notify calls.
	 *
	 * IMPORTANT: Must wait for ble_tx_ready before sending. This is set in
	 * security_changed() after encryption is established. Sending before the
	 * connection is fully secured causes "No ATT channel for MTU" errors.
	 */
	if (!current_conn || !nus_notif_enabled || !ble_tx_ready) {
		LOG_DBG("tx_drain[BLE]: not ready (conn=%p notif=%d ready=%d)",
			current_conn, nus_notif_enabled, ble_tx_ready);
		return;
	}

	/* Take a reference snapshot — prevents use-after-free if
	 * disconnected() fires from another context between our check
	 * and use of the connection pointer. */
	struct bt_conn *conn = bt_conn_ref(current_conn);
	if (!conn) {
		return;
	}

	/* Re-entrancy guard - only one TX in flight at a time */
	if (ble_tx_in_progress) {
		/* TX timeout watchdog - if callback never fired, reset state */
		if ((k_uptime_get() - ble_tx_start_time) > BLE_TX_TIMEOUT_MS) {
			LOG_WRN("tx_drain[BLE]: TX timeout, resetting state");
			ble_tx_in_progress = false;
			/* Fall through to try next TX */
		} else {
			LOG_DBG("tx_drain[BLE]: TX in progress, callback will chain");
			bt_conn_unref(conn);
			return;
		}
	}

	/* Check retry buffer first */
	if (tx_retry_pending) {
		LOG_DBG("tx_drain[BLE]: retrying len=%u hdr=0x%02x", (unsigned)tx_retry_frame.len, tx_retry_frame.buf[0]);
		ble_tx_in_progress = true;
		ble_tx_start_time = k_uptime_get();
		err = secure_nus_send(conn, tx_retry_frame.buf, tx_retry_frame.len);
		if (err == 0) {
			tx_retry_pending = false;
			LOG_DBG("tx_drain[BLE]: retry success");
			bt_conn_unref(conn);
			return;  /* Callback will chain to next */
		} else if (err == -EAGAIN || err == -ENOMEM) {
			ble_tx_in_progress = false;
			LOG_DBG("tx_drain[BLE]: retry still busy, wait %dms", BLE_TX_RETRY_MS);
			k_work_schedule(&tx_drain_work, K_MSEC(BLE_TX_RETRY_MS));
			bt_conn_unref(conn);
			return;
		} else {
			ble_tx_in_progress = false;
			tx_retry_pending = false;
			LOG_WRN("tx_drain[BLE]: retry failed err=%d, dropped", err);
			/* Fall through to try next frame */
		}
	}

	/* Get next frame from queue (the txq clears congestion at 1/3 and empty) */
	if (frame_txq_get(&ble_txq, &f) != 0) {
		if (ble_cbs && ble_cbs->link.on_tx_idle) {
			ble_cbs->link.on_tx_idle();
		}
		bt_conn_unref(conn);
		return;
	}

	LOG_DBG("tx_drain[BLE]: sending len=%u hdr=0x%02x queue=%u",
		(unsigned)f.len, f.buf[0], k_msgq_num_used_get(&ble_send_queue));

	/* Mark TX in progress before calling notify_cb */
	ble_tx_in_progress = true;
	ble_tx_start_time = k_uptime_get();
	err = secure_nus_send(conn, f.buf, f.len);

	if (err == 0) {
		/* Success - callback will chain to next */
		bt_conn_unref(conn);
		return;
	} else if (err == -EAGAIN || err == -ENOMEM) {
		/* BLE buffer full - save for retry */
		ble_tx_in_progress = false;
		tx_retry_frame = f;
		tx_retry_pending = true;
		LOG_DBG("tx_drain[BLE]: BLE busy (err=%d), saved for retry", err);
		k_work_schedule(&tx_drain_work, K_MSEC(BLE_TX_RETRY_MS));
		bt_conn_unref(conn);
		return;
	} else {
		/* Other error - drop frame.  Re-kick rather than returning: on_tx_idle
		 * only fires from the queue-empty path below, and it is the sole re-arm
		 * for the contact dump — bailing out here stranded the dump forever.
		 * This terminates: the frame is already off the queue, so each pass
		 * either drains one or reaches empty and signals idle. */
		ble_tx_in_progress = false;
		LOG_WRN("tx_drain[BLE]: send failed err=%d, dropped frame", err);
		kick_tx_drain();
		bt_conn_unref(conn);
		return;
	}
}

/* ========== NUS service callbacks ========== */

static void secure_nus_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	bool enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("CCCD notif %s (value=0x%04x)", enabled ? "enabled" : "disabled", value);
	nus_notif_enabled = enabled;
	if (enabled) {
		/* Kick TX drain — if security_changed already set ble_tx_ready,
		 * data starts flowing.  If security hasn't fired yet (bonded
		 * reconnect race), kick_tx_drain bails harmlessly and
		 * security_changed will kick again once ble_tx_ready is set. */
		kick_tx_drain();
	}
}

static ssize_t secure_nus_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);

	if (len == 0 || len > MAX_FRAME_SIZE) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *data = (const uint8_t *)buf;
	uint8_t cmd = data[0];

	LOG_DBG("NUS RX: len=%u cmd=0x%02x", len, cmd);

	struct frame f;

	f.len = len;
	memcpy(f.buf, data, len);
	if (k_msgq_put(&ble_recv_queue, &f, K_NO_WAIT) != 0) {
		LOG_WRN("recv queue full");
		return len;
	}
	if (ble_cbs && ble_cbs->link.on_rx) {
		ble_cbs->link.on_rx();
	}

	return len;
}

/* ========== Advertising ========== */

static void adv_slow_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("fast adv window expired, switching to slow interval");
	fast_adv_active = false;
	adv_stop_for_interval_change = true;  /* suppress recycled() restart */
	bt_le_adv_stop();
	adv_running = false;
	start_adv();
	/* adv_stop_for_interval_change cleared by recycled() on the work queue */
}

static void start_adv(void)
{
	if (!ble_enabled) {
		return;
	}

	uint16_t interval = fast_adv_active ? BT_ADV_FAST_INTERVAL : BT_ADV_INTERVAL;

	/* USE_IDENTITY: advertise the stable identity address instead of an RPA,
	 * even when CONFIG_BT_PRIVACY=y.  Keeps the controller in its (working)
	 * privacy-enabled SC encryption path for iOS while exposing a fixed address
	 * so the Android companion app's "connect from app" flow works.  See
	 * findings.md Issue #34 iOS-privacy capture (controller MIC failure 0x3d on
	 * the privacy-off SC path). */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
		.interval_min = interval,
		.interval_max = interval,
	};

	int err = bt_le_adv_start(&adv_param, ad, ad_len, sd, sd_len);
	if (err && err != -EALREADY) {
		LOG_ERR("adv start failed: %d", err);
		adv_running = false;
	} else {
		LOG_INF("BLE advertising: %s",
			fast_adv_active ? "20ms fast (60s)" : "211ms slow");
		adv_running = true;
	}
}

/* Enter the post-boot/disconnect fast-advertising window, then start adv.
 * The adv_slow_work timer flips back to the slow interval after
 * BT_ADV_FAST_DURATION_MS. */
static void start_fast_adv(void)
{
	fast_adv_active = true;
	k_work_reschedule(&adv_slow_work, K_MSEC(BT_ADV_FAST_DURATION_MS));
	start_adv();
}

/* ========== Public API ========== */

void zephcore_ble_init(const struct ble_callbacks *cbs)
{
	ble_cbs = cbs;
	frame_txq_init(&ble_txq, &ble_send_queue, FRAME_QUEUE_SIZE, "ble",
		       ble_link_up, kick_tx_drain);

	/* Resolve NUS TX characteristic attribute once — avoids hard-coded
	 * array offset in secure_nus_send(). attrs[2] = TX char value
	 * (attrs[0]=service, attrs[1]=TX char decl, attrs[2]=TX char value,
	 * attrs[3]=CCC, attrs[4]=RX char decl, attrs[5]=RX char value). */
	nus_tx_attr = &secure_nus_svc.attrs[2];

	bt_conn_auth_cb_register(&auth_cb);
	bt_conn_auth_info_cb_register(&auth_info_cb);
}

void zephcore_ble_start(const char *name)
{
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		/* NVS self-initializes the storage_partition on first mount: a
		 * region carved from the old app slot reads as "all sectors closed",
		 * which nvs_startup() reformats (erase-all) before settings load.
		 * No app-side seeding needed. */
		settings_load();
#if IS_ENABLED(CONFIG_BT_GATT_SERVICE_CHANGED)
		ble_gatt_layout_check_after_settings_load();
#endif
	}

	build_device_name_and_adv(name);
	ble_started = true;
	if (ble_enabled) {
		LOG_DBG("init complete, starting adv");
		start_fast_adv();
	}
}

size_t zephcore_ble_send(const uint8_t *data, uint16_t len)
{
	/* Nothing is queued without a connection */
	if (!current_conn) {
		LOG_DBG("no BLE conn, dropping len=%u hdr=0x%02x",
			(unsigned)len, len ? data[0] : 0);
		return 0;
	}
	return frame_txq_put(&ble_txq, data, len);
}

size_t zephcore_ble_recv(uint8_t *dest)
{
	struct frame f;

	if (k_msgq_get(&ble_recv_queue, &f, K_NO_WAIT) != 0) {
		return 0;
	}
	memcpy(dest, f.buf, f.len);
	return f.len;
}

void zephcore_ble_set_enabled(bool enable)
{
	ble_enabled = enable;
	if (!ble_started) {
		return;  /* zephcore_ble_start() honours it */
	}
	if (!enable) {
		/* Disconnect current connection if any */
		if (current_conn) {
			bt_conn_disconnect(current_conn,
					   BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		/* Stop advertising */
		bt_le_adv_stop();
		adv_running = false;

		/* No future fast->slow transition needed */
		k_work_cancel_delayable(&adv_slow_work);

		LOG_INF("BLE disabled");
	} else {
		/* Re-enable advertising — start fast window */
		start_fast_adv();
		LOG_INF("BLE enabled");
	}
}

bool zephcore_ble_is_enabled(void)
{
	return ble_enabled;
}

bool zephcore_ble_is_active(void)
{
	return current_conn != NULL && ble_tx_ready;
}

bool zephcore_ble_is_connected(void)
{
	return current_conn != NULL;
}

bool zephcore_ble_is_write_busy(void)
{
	return frame_txq_busy(&ble_txq);
}

bool zephcore_ble_tx_idle(void)
{
	/* Nothing connected — nothing can be in flight, and nothing ever will be. */
	if (!current_conn) {
		return true;
	}
	return frame_txq_empty(&ble_txq) &&
	       !ble_tx_in_progress &&
	       !tx_retry_pending;
}

bool zephcore_ble_is_advertising(void)
{
	return adv_running;
}

void zephcore_ble_set_passkey(uint32_t passkey)
{
	if (passkey >= 100000 && passkey <= 999999) {
		ble_passkey = passkey;
	} else {
		ble_passkey = CONFIG_ZEPHCORE_BLE_PASSKEY;
	}
	LOG_INF("BLE passkey updated to %06u (effective on next pairing)", ble_passkey);
}

uint32_t zephcore_ble_get_passkey(void)
{
	return ble_passkey;
}

void zephcore_ble_update_name(const char *new_name)
{
	build_device_name_and_adv(new_name);

	/* If currently advertising (not connected), restart so the new name
	 * is published immediately.  Restart at fast interval so anyone
	 * scanning sees the new name quickly. */
	if (!current_conn) {
		LOG_INF("name updated, restarting adv");
		adv_stop_for_interval_change = true;  /* suppress recycled() restart */
		bt_le_adv_stop();
		adv_running = false;
		start_fast_adv();
	}
	/* If connected: GATT device name (via bt_set_name in build_device_name_and_adv)
	 * is live now; advertising payload updates on next adv cycle after disconnect. */
}

void zephcore_ble_conn_params_ready(void)
{
	if (!conn_params_pending || !current_conn) {
		return;
	}
	conn_params_pending = false;

	struct bt_le_conn_param conn_param = {
		.interval_min = BLE_DEFAULT_MIN_INTERVAL,
		.interval_max = BLE_DEFAULT_MAX_INTERVAL,
		.latency = BLE_DEFAULT_LATENCY,
		.timeout = BLE_DEFAULT_TIMEOUT,
	};
	int err = bt_conn_le_param_update(current_conn, &conn_param);
	if (err) {
		LOG_WRN("Post-sync conn param update failed: %d", err);
	} else {
		LOG_INF("Post-sync conn params: %d-%dms interval, latency=%d",
			BLE_DEFAULT_MIN_INTERVAL * 5 / 4,
			BLE_DEFAULT_MAX_INTERVAL * 5 / 4,
			BLE_DEFAULT_LATENCY);
	}
}
