/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore BLE Adapter — NUS service, advertising, security, TX/RX
 *
 * Security: SMP pairing with SC + MITM + Bonding, DisplayOnly IO (app_passkey).
 * Pairing is triggered reactively by ATT_ERR_AUTHENTICATION on secured GATT
 * attributes (Apple §55 compliant — no proactive Security Request on connect).
 *
 * Advertising: ESP32 enables CONFIG_BT_PRIVACY in esp32_common.conf; nRF uses public identity.
 * Android Flutter may fail connect-from-app to RPA unless bonded; Arduino MeshCore uses public.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_ble, CONFIG_ZEPHCORE_BLE_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/bluetooth/services/nus/inst.h>
#include <zephyr/settings/settings.h>

#include "ZephyrBLE.h"

/* MAX_FRAME_SIZE from CompanionMesh.h — keep in sync */
#ifndef MAX_FRAME_SIZE
#define MAX_FRAME_SIZE 172
#endif

/* ========== Constants ========== */

#define DEVICE_NAME_MAX 29
#define FRAME_QUEUE_SIZE CONFIG_ZEPHCORE_BLE_QUEUE_SIZE
#define BLE_TX_POWER 4
#define BLE_TX_RETRY_MS 20

/* BLE connection parameters */
#define BLE_DEFAULT_MIN_INTERVAL CONFIG_ZEPHCORE_BLE_CONN_MIN_INTERVAL
#define BLE_DEFAULT_MAX_INTERVAL CONFIG_ZEPHCORE_BLE_CONN_MAX_INTERVAL
#define BLE_DEFAULT_LATENCY      CONFIG_ZEPHCORE_BLE_CONN_LATENCY
#define BLE_DEFAULT_TIMEOUT      CONFIG_ZEPHCORE_BLE_CONN_TIMEOUT

/* TX timeout watchdog - reset ble_tx_in_progress if callback never fires */
#define BLE_TX_TIMEOUT_MS 2000

/* Congestion overflow retry interval — when the TX queue is full, the
 * stuck frame retries at this cadence.  Slow enough to not hammer the
 * BLE stack when the link is marginal, fast enough to recover quickly. */
#define BLE_TX_OVERFLOW_RETRY_MS 250

/* App push notifications are 0x80+; protocol response packets are < 0x80. */
#define PUSH_CODE_BASE 0x80

/* Advertising intervals (Apple Accessory Design Guidelines §5.5) */
#define BT_ADV_FAST_INTERVAL     32            /* 20ms in 0.625ms units */
#define BT_ADV_FAST_DURATION_MS  (60 * 1000)  /* fast window after boot/disconnect */
#define BT_ADV_INTERVAL          CONFIG_ZEPHCORE_BLE_ADV_SLOW_INTERVAL

/* ========== Frame type for queues ========== */

struct frame {
	uint16_t len;
	uint8_t buf[MAX_FRAME_SIZE];
};

/* ========== Static state ========== */

/* Deferred connection parameter update — applied after initial sync
 * completes (NO_MORE_MSGS) rather than immediately on security_changed,
 * because the negotiation disrupts BLE throughput during channel/contact sync. */
static bool conn_params_pending;

/* Callbacks to main */
static const struct ble_callbacks *ble_cbs;

/* Advertising data */
static char device_name[DEVICE_NAME_MAX];
static const uint8_t ad_flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
static const int8_t ad_tx_power = BLE_TX_POWER;
static const uint8_t nus_uuid[] = { BT_UUID_NUS_SRV_VAL };
static struct bt_data ad[3];
static struct bt_data sd[1];
static size_t ad_len;
static size_t sd_len;

/* Queues — ISR-safe, no mutex needed */
K_MSGQ_DEFINE(ble_send_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ble_recv_queue, sizeof(struct frame), FRAME_QUEUE_SIZE, 4);

/* TX retry buffer - used when BLE returns -ENOMEM/-EAGAIN */
static struct frame tx_retry_frame;
static bool tx_retry_pending = false;

/* TX congestion control — flow-control mechanism for queue-full conditions.
 *
 * When the TX queue is full, instead of blocking or dropping:
 *   1. Set ble_tx_congested flag → callers (contact iteration, etc.) stop sending
 *   2. Save the stuck frame to overflow_frame → retried every 250ms
 *   3. When queue drains below low water mark (1/3) → clear congestion
 *   4. On disconnect → clear everything
 *
 * Water marks (with default FRAME_QUEUE_SIZE=12):
 *   - Contact iteration pauses:  2/3 = 8 frames (high water)
 *   - Congestion mode enters:    12/12 = full
 *   - Congestion mode clears:    1/3 = 4 frames (low water)
 */
static bool ble_tx_congested;
static struct frame overflow_frame;
static bool overflow_pending;

/* Connection state */
static struct bt_conn *current_conn;
static bool nus_notif_enabled;
static bool ble_tx_ready = false;
static bool ble_tx_in_progress = false;
static int64_t ble_tx_start_time = 0;

/* Active interface tracking */
static enum zephcore_iface active_iface = ZEPHCORE_IFACE_NONE;

/* DLE tracking — set after successful DLE request to avoid double-request */
static bool dle_requested;

/* Fast advertising — true for BT_ADV_FAST_DURATION_MS after boot or disconnect */
static bool fast_adv_active;

/* Set before bt_le_adv_stop() in adv_slow_work_fn to suppress recycled() from
 * restarting fast advertising. Zephyr fires recycled() when the pre-allocated
 * connection slot is freed after any adv stop — not just on disconnect.
 * Cleared by recycled() itself (both run on the cooperative system work queue). */
static bool adv_stop_for_interval_change;


/* Runtime BLE passkey */
static uint32_t ble_passkey = CONFIG_ZEPHCORE_BLE_PASSKEY;

/* NUS TX characteristic attribute — resolved at init, avoids hard-coded offset */
static const struct bt_gatt_attr *nus_tx_attr;

static bool is_lossless_protocol_frame(const uint8_t *data, uint16_t len)
{
	if (!data || len == 0) {
		return false;
	}
	return data[0] < PUSH_CODE_BASE;
}

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

static sys_slist_t secure_nus_cbs_list = SYS_SLIST_STATIC_INIT(&secure_nus_cbs_list);
STRUCT_SECTION_ITERABLE(bt_nus_inst, secure_nus) = {
	.svc = &secure_nus_svc,
	.cbs = &secure_nus_cbs_list,
};

/* ========== Work items ========== */

static void tx_drain_work_fn(struct k_work *work);
static void overflow_retry_work_fn(struct k_work *work);
static void adv_slow_work_fn(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(tx_drain_work, tx_drain_work_fn);
K_WORK_DELAYABLE_DEFINE(overflow_retry_work, overflow_retry_work_fn);
K_WORK_DELAYABLE_DEFINE(adv_slow_work, adv_slow_work_fn);

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

		/* Apple BLE Accessory Design Guidelines: device name must not
		 * contain ':' or ';' characters. Replace with '-'.
		 */
		for (size_t i = 0; device_name[i]; i++) {
			if (device_name[i] == ':' || device_name[i] == ';') {
				device_name[i] = '-';
			}
		}
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

	/* Cancel fast→slow transition — already connected, no need to switch */
	k_work_cancel_delayable(&adv_slow_work);

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
	if (ble_cbs && ble_cbs->on_connected) {
		ble_cbs->on_connected();
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("disconnected: %s reason 0x%02x", addr, reason);

	if (conn == current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	nus_notif_enabled = false;

	/* Reset BLE TX state */
	ble_tx_in_progress = false;
	ble_tx_ready = false;
	conn_params_pending = false;

	/* Clear interface state if BLE was active */
	if (active_iface == ZEPHCORE_IFACE_BLE) {
		active_iface = ZEPHCORE_IFACE_NONE;
	}

	/* Clear queues, retry state, and congestion */
	k_msgq_purge(&ble_send_queue);
	k_msgq_purge(&ble_recv_queue);
	tx_retry_pending = false;
	overflow_pending = false;
	ble_tx_congested = false;

	k_work_cancel_delayable(&tx_drain_work);
	k_work_cancel_delayable(&overflow_retry_work);

	/* Notify main of BLE disconnection */
	if (ble_cbs && ble_cbs->on_disconnected) {
		ble_cbs->on_disconnected();
	}
}

static void recycled(void)
{
	if (adv_stop_for_interval_change) {
		adv_stop_for_interval_change = false;
		return;
	}
	LOG_DBG("restart advertising");
	fast_adv_active = true;
	k_work_reschedule(&adv_slow_work, K_MSEC(BT_ADV_FAST_DURATION_MS));
	start_adv();
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

	/* Enable TX when we have sufficient security (level 2+ = encrypted).
	 * This is the ONLY place that sets ble_tx_ready — security_changed is
	 * the authority.  CCC subscription (secure_nus_ccc_changed) only kicks
	 * the TX drain; it never sets ble_tx_ready.
	 */
	if (level >= BT_SECURITY_L2 && !ble_tx_ready) {
		LOG_INF("security established, enabling TX");
		ble_tx_ready = true;
		active_iface = ZEPHCORE_IFACE_BLE;

		/* If CCC was already subscribed (bonded reconnect — phone writes
		 * CCC before security_changed fires), kick TX now.  On fresh
		 * pairing CCC hasn't been written yet, so this is a no-op and
		 * TX starts when CCC fires later. */
		if (nus_notif_enabled) {
			kick_tx_drain();
		}
	}

	if (level >= BT_SECURITY_L2) {
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
	ARG_UNUSED(conn);
	LOG_INF("pairing complete: bonded=%d", bonded);

	/* Switch to BLE interface (fresh pairing only).
	 * Conn params and TX enable are handled in security_changed(),
	 * which fires for both fresh pairing and bonded reconnects. */
	if (active_iface == ZEPHCORE_IFACE_USB) {
		LOG_INF("switching from USB to BLE");
		/* Main handles USB state clearing via on_connected callback */
	}
	active_iface = ZEPHCORE_IFACE_BLE;
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_WRN("pairing failed: reason %d", reason);
	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* ========== TX congestion overflow retry ========== */

static void overflow_retry_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!overflow_pending) {
		return;
	}

	/* Abandon overflow if connection is gone */
	if (!current_conn || active_iface == ZEPHCORE_IFACE_NONE) {
		overflow_pending = false;
		ble_tx_congested = false;
		LOG_INF("overflow cleared (disconnected)");
		return;
	}

	if (k_msgq_put(&ble_send_queue, &overflow_frame, K_NO_WAIT) == 0) {
		overflow_pending = false;
		LOG_DBG("overflow frame queued hdr=0x%02x, kicking drain",
			overflow_frame.buf[0]);
		kick_tx_drain();
		/* Congestion flag cleared by tx_drain at low water mark */
	} else {
		/* Still full — retry at reduced rate */
		LOG_DBG("overflow retry: queue still full, retry in 250ms");
		k_work_schedule(&overflow_retry_work, K_MSEC(BLE_TX_OVERFLOW_RETRY_MS));
	}
}

/* ========== TX drain work ========== */

static void kick_tx_drain(void)
{
	k_work_schedule(&tx_drain_work, K_NO_WAIT);
}

static void tx_drain_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	struct frame f;
	int err;

	/* USB TX path is handled in main — only BLE TX here */
	if (active_iface == ZEPHCORE_IFACE_USB) {
		/* Let main handle USB TX — just signal tx_idle if queue empty */
		if (k_msgq_num_used_get(&ble_send_queue) == 0) {
			if (ble_cbs && ble_cbs->on_tx_idle) {
				ble_cbs->on_tx_idle();
			}
		}
		return;
	}

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

	/* Get next frame from queue */
	if (k_msgq_get(&ble_send_queue, &f, K_NO_WAIT) != 0) {
		/* TX queue empty — clear congestion and signal idle */
		if (ble_tx_congested) {
			ble_tx_congested = false;
			LOG_INF("tx_drain: congestion cleared (queue empty)");
		}
		if (ble_cbs && ble_cbs->on_tx_idle) {
			ble_cbs->on_tx_idle();
		}
		bt_conn_unref(conn);
		return;
	}

	/* Clear congestion at low water mark (1/3 of queue) — gives headroom
	 * before hitting full again.  Hysteresis: ON at full, OFF at 1/3. */
	if (ble_tx_congested) {
		uint32_t used = k_msgq_num_used_get(&ble_send_queue);
		if (used <= FRAME_QUEUE_SIZE / 3) {
			ble_tx_congested = false;
			LOG_INF("tx_drain: congestion cleared (queue=%u/%u)",
				used, (unsigned)FRAME_QUEUE_SIZE);
		}
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
		/* Other error - drop frame */
		ble_tx_in_progress = false;
		LOG_WRN("tx_drain[BLE]: send failed err=%d, dropped frame", err);
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

	/* Notify main via callback */
	if (ble_cbs && ble_cbs->on_rx_frame) {
		ble_cbs->on_rx_frame(data, len);
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
	start_adv();
	/* adv_stop_for_interval_change cleared by recycled() on the work queue */
}

static void start_adv(void)
{
	uint16_t interval = fast_adv_active ? BT_ADV_FAST_INTERVAL : BT_ADV_INTERVAL;

	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = interval,
		.interval_max = interval,
	};

	int err = bt_le_adv_start(&adv_param, ad, ad_len, sd, sd_len);
	if (err && err != -EALREADY) {
		LOG_ERR("adv start failed: %d", err);
	} else {
		LOG_INF("BLE advertising: %s",
			fast_adv_active ? "20ms fast (60s)" : "211ms slow");
	}
}

/* ========== Public API ========== */

void zephcore_ble_init(const struct ble_callbacks *cbs)
{
	ble_cbs = cbs;

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
		settings_load();
	}

	build_device_name_and_adv(name);
	LOG_DBG("init complete, starting adv");
	fast_adv_active = true;
	k_work_reschedule(&adv_slow_work, K_MSEC(BT_ADV_FAST_DURATION_MS));
	start_adv();
}

size_t zephcore_ble_send(const uint8_t *data, uint16_t len)
{
	if (len == 0 || len > MAX_FRAME_SIZE) {
		LOG_WRN("invalid len=%u", (unsigned)len);
		return 0;
	}

	/* Don't queue frames if no active transport */
	if (active_iface == ZEPHCORE_IFACE_BLE && !current_conn) {
		LOG_DBG("no BLE conn, dropping len=%u hdr=0x%02x",
			(unsigned)len, data[0]);
		return 0;
	}
	if (active_iface == ZEPHCORE_IFACE_NONE) {
		LOG_DBG("no active iface, dropping len=%u hdr=0x%02x",
			(unsigned)len, data[0]);
		return 0;
	}

	struct frame f;
	f.len = len;
	memcpy(f.buf, data, len);

	if (k_msgq_put(&ble_send_queue, &f, K_NO_WAIT) != 0) {
		/* Queue full — enter congestion mode.
		 *
		 * Instead of blocking (would stall LoRa) or dropping (loses
		 * frames), we signal congestion so all senders stop, then
		 * save this frame and retry at a reduced 250ms cadence until
		 * the queue drains or the connection drops.
		 *
		 * Callers check zephcore_ble_is_congested() and hold off:
		 *   - contact_iter_work: pauses iteration
		 *   - main thread (sendPush): pushes are best-effort signals,
		 *     actual message data is safe in the offline queue
		 */
		if (!ble_tx_congested) {
			LOG_WRN("TX queue full (%u/%u), entering congestion",
				k_msgq_num_used_get(&ble_send_queue),
				(unsigned)FRAME_QUEUE_SIZE);
			ble_tx_congested = true;
		}

		/* Protocol response frames are lossless. If queue is full, report
		 * failure so the caller can retry instead of overflow replacement. */
		if (is_lossless_protocol_frame(data, len)) {
			LOG_DBG("TX queue full for lossless protocol frame hdr=0x%02x, retry later", data[0]);
			return 0;
		}

		/* Save to overflow — retried at 250ms intervals.
		 * If overflow already pending, replace with newest frame
		 * (push notifications are idempotent MSG_WAITING signals). */
		if (overflow_pending) {
			LOG_DBG("overflow replaced: 0x%02x → 0x%02x",
				overflow_frame.buf[0], data[0]);
		}
		overflow_frame = f;
		overflow_pending = true;
		k_work_schedule(&overflow_retry_work, K_MSEC(BLE_TX_OVERFLOW_RETRY_MS));
		return len;  /* Accepted into overflow — will be retried */
	}

	LOG_DBG("queued len=%u hdr=0x%02x queue=%u",
		(unsigned)len, data[0], k_msgq_num_used_get(&ble_send_queue));

	kick_tx_drain();
	return len;
}

void zephcore_ble_set_enabled(bool enable)
{
	if (!enable) {
		/* Disconnect current connection if any */
		if (current_conn) {
			bt_conn_disconnect(current_conn,
					   BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		/* Stop advertising */
		bt_le_adv_stop();
		LOG_INF("BLE disabled");
	} else {
		/* Re-enable advertising — start fast window */
		fast_adv_active = true;
		k_work_reschedule(&adv_slow_work, K_MSEC(BT_ADV_FAST_DURATION_MS));
		start_adv();
		LOG_INF("BLE enabled");
	}
}

bool zephcore_ble_is_active(void)
{
	return active_iface == ZEPHCORE_IFACE_BLE && current_conn != NULL && ble_tx_ready;
}

bool zephcore_ble_is_connected(void)
{
	return current_conn != NULL;
}

bool zephcore_ble_is_congested(void)
{
	return ble_tx_congested;
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

enum zephcore_iface zephcore_ble_get_active_iface(void)
{
	return active_iface;
}

void zephcore_ble_set_active_iface(enum zephcore_iface iface)
{
	active_iface = iface;
}

struct k_msgq *zephcore_ble_get_recv_queue(void)
{
	return &ble_recv_queue;
}

struct k_msgq *zephcore_ble_get_send_queue(void)
{
	return &ble_send_queue;
}

void zephcore_ble_kick_tx(void)
{
	kick_tx_drain();
}

void zephcore_ble_disconnect(void)
{
	if (current_conn) {
		bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
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
