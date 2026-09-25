/*
 * SPDX-License-Identifier: MIT
 * ZephCore BLE - legacy Nordic/Adafruit buttonless DFU service
 *
 * Split out of ZephyrBLE.cpp. The service symbol name keeps the GATT handle
 * order (see below); the jump goes through zephcore_ble_dfu_request().
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

/* ========== Legacy Nordic/Adafruit buttonless DFU service ==========
 *
 * Mirrors Adafruit BLEDfu (Arduino MeshCore >=1.15.0) so the same DFU tools
 * interoperate: a paired phone writes 0x01 to the control point and the device
 * resets into the bootloader's BLE OTA mode. We use the *unbonded* OTA reset
 * (GPREGRET 0xA8, same as `start ota`): the bootloader comes up as a fresh DFU
 * target and the tool re-scans for it (the legacy buttonless flow). Adafruit's
 * 0xB1 bonded-resume path needs SoftDevice peer-data enrollment we can't
 * replicate on Zephyr, so the phone makes a fresh connection to the bootloader.
 *
 * The service exposes the FULL legacy DFU shape Adafruit ships (not just the
 * control point): iOS's LegacyDFUService treats the DFU Packet (1532) char as
 * a *required* characteristic — discovery fails (DFU "instantly fails") without
 * it — and reads the DFU Revision (1534 = 0x0001 "app mode") to classify the
 * device as an application that supports the buttonless jump (missing → the app
 * can't identify the device → shows it nameless). Packet is a no-op here: the
 * real image upload happens in the bootloader, not the running app. All three
 * chars sit behind AUTHEN, matching the Arduino companion's service-wide MITM
 * floor (`bledfu.setPermission(SECMODE_ENC_WITH_MITM, ...)`); a bonded phone's
 * DFU app reuses the OS-level bond, an unpaired stranger is rejected.
 *
 * NOTE on the service symbol name: Zephyr registers static GATT services in
 * the order ld's SORT_BY_NAME emits them — i.e. alphabetically by the symbol
 * passed to BT_GATT_SERVICE_DEFINE. This service MUST sort *after*
 * `secure_nus_svc` so the NUS attribute handles stay fixed; otherwise every
 * NUS handle shifts and bonded phones with cached handles get ATT 0x03
 * (Write Not Permitted) on the NUS RX write. Hence the `secure_nus_svc_dfu`
 * name (a string sorts before its own extensions, so NUS keeps the low range).
 */
static struct bt_uuid_128 dfu_svc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001530, 0x1212, 0xefde, 0x1523, 0x785feabcd123));
static struct bt_uuid_128 dfu_ctrl_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001531, 0x1212, 0xefde, 0x1523, 0x785feabcd123));
static struct bt_uuid_128 dfu_packet_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001532, 0x1212, 0xefde, 0x1523, 0x785feabcd123));
static struct bt_uuid_128 dfu_revision_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001534, 0x1212, 0xefde, 0x1523, 0x785feabcd123));

/* DFU Revision = 0x0001 (DFU_REV_APPMODE), little-endian — tells the DFU app
 * "this is an application that supports the buttonless jump to bootloader". */
static const uint8_t dfu_revision[2] = { 0x01, 0x00 };

static void dfu_jump_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dfu_jump_work, dfu_jump_work_fn);

static void dfu_jump_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	zephcore_ble_dfu_request();  /* sets GPREGRET + resets; never returns */
}

static ssize_t dfu_ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);
	/* Adafruit BLEDfu jump command: first byte 0x01 (1-2 byte write). */
	if (len >= 1 && ((const uint8_t *)buf)[0] == 0x01) {
		LOG_INF("buttonless DFU requested - rebooting to BLE OTA");
		/* Defer so the ATT write response flushes and the DFU tool can
		 * arm its disconnect/rescan before we reset. */
		k_work_schedule(&dfu_jump_work, K_MSEC(250));
	}
	return len;
}

static void dfu_ctrl_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	ARG_UNUSED(value);
}

/* DFU Packet — present only so the DFU library's characteristic discovery
 * succeeds; the actual image transfer happens in the bootloader, not here. */
static ssize_t dfu_packet_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(buf);
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);
	return len;  /* no-op in app mode */
}

static ssize_t dfu_revision_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 dfu_revision, sizeof(dfu_revision));
}

BT_GATT_SERVICE_DEFINE(secure_nus_svc_dfu,
	BT_GATT_PRIMARY_SERVICE(&dfu_svc_uuid),
	BT_GATT_CHARACTERISTIC(&dfu_ctrl_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_WRITE_AUTHEN,
		NULL, dfu_ctrl_write, NULL),
	BT_GATT_CCC(dfu_ctrl_ccc_changed,
		BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(&dfu_packet_uuid.uuid,
		BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE_AUTHEN,
		NULL, dfu_packet_write, NULL),
	BT_GATT_CHARACTERISTIC(&dfu_revision_uuid.uuid,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ_AUTHEN,
		dfu_revision_read, NULL, NULL),
);
