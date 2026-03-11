/*
 * SPDX-License-Identifier: Apache-2.0
 * CompanionMesh implementation
 */

#include "CompanionMesh.h"
#include <mesh/Utils.h>
#include <helpers/TxtDataHelpers.h>
#include <ed_25519.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <ZephyrSensorManager.h>
#include <adapters/sensors/SimpleLPP.h>
#include <adapters/ble/ZephyrBLE.h>
LOG_MODULE_REGISTER(zephcore_companion, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* Protocol commands (matches Arduino companion_radio) - sorted by opcode */
#define CMD_APP_START            0x01
#define CMD_SEND_TXT_MSG         0x02
#define CMD_SEND_CHANNEL_TXT_MSG 0x03
#define CMD_GET_CONTACTS         0x04
#define CMD_GET_DEVICE_TIME      0x05
#define CMD_SET_DEVICE_TIME      0x06
#define CMD_SEND_SELF_ADVERT     0x07
#define CMD_SET_ADVERT_NAME      0x08
#define CMD_ADD_UPDATE_CONTACT   0x09
#define CMD_SYNC_NEXT_MESSAGE    0x0A
#define CMD_SET_RADIO_PARAMS     0x0B
#define CMD_SET_RADIO_TX_POWER   0x0C
#define CMD_RESET_PATH           0x0D
#define CMD_SET_ADVERT_LATLON    0x0E
#define CMD_REMOVE_CONTACT       0x0F
#define CMD_SHARE_CONTACT        0x10
#define CMD_EXPORT_CONTACT       0x11
#define CMD_IMPORT_CONTACT       0x12
#define CMD_REBOOT               0x13
#define CMD_GET_BATT_AND_STORAGE 0x14
#define CMD_SET_TUNING_PARAMS    0x15
#define CMD_DEVICE_QUERY         0x16
#define CMD_EXPORT_PRIVATE_KEY   0x17
#define CMD_IMPORT_PRIVATE_KEY   0x18
#define CMD_SEND_RAW_DATA        0x19
#define CMD_SEND_LOGIN           0x1A
#define CMD_SEND_STATUS_REQ      0x1B
#define CMD_HAS_CONNECTION       0x1C
#define CMD_LOGOUT               0x1D
#define CMD_GET_CONTACT_BY_KEY   0x1E
#define CMD_GET_CHANNEL          0x1F
#define CMD_SET_CHANNEL          0x20
#define CMD_SIGN_START           0x21
#define CMD_SIGN_DATA            0x22
#define CMD_SIGN_FINISH          0x23
#define CMD_SEND_TRACE_PATH      0x24
#define CMD_SET_DEVICE_PIN       0x25
#define CMD_SET_OTHER_PARAMS     0x26
#define CMD_SEND_TELEMETRY_REQ   0x27
#define CMD_GET_CUSTOM_VARS      0x28
#define CMD_SET_CUSTOM_VAR       0x29
#define CMD_GET_ADVERT_PATH      0x2A
#define CMD_GET_TUNING_PARAMS    0x2B
#define CMD_SEND_BINARY_REQ      0x32
#define CMD_FACTORY_RESET        0x33
#define CMD_SEND_PATH_DISCOVERY  0x34
#define CMD_SET_FLOOD_SCOPE      0x36
#define CMD_SEND_CONTROL_DATA    0x37
#define CMD_GET_STATS            0x38
#define CMD_SEND_ANON_REQ        0x39
#define CMD_SET_AUTOADD_CONFIG   0x3A
#define CMD_GET_AUTOADD_CONFIG   0x3B
#define CMD_GET_ALLOWED_REPEAT_FREQ 0x3C
#define CMD_SET_PATH_HASH_MODE      0x3D

/* Response packet types */
#define PACKET_OK               0x00
#define PACKET_ERROR            0x01
#define PACKET_CONTACT_START    0x02
#define PACKET_CONTACT          0x03
#define PACKET_CONTACT_END      0x04
#define PACKET_SELF_INFO        0x05
#define PACKET_SENT             0x06
#define PACKET_CONTACT_MSG_RECV 0x07  /* Legacy - ver < 3 */
#define PACKET_CHANNEL_MSG_RECV 0x08  /* Legacy - ver < 3 */
#define PACKET_CURR_TIME        0x09
#define PACKET_NO_MORE_MSGS     0x0A
#define PACKET_EXPORT_CONTACT   0x0B
#define PACKET_BATTERY          0x0C
#define PACKET_DEVICE_INFO      0x0D
#define PACKET_PRIVATE_KEY      0x0E
#define PACKET_DISABLED         0x0F
#define PACKET_CONTACT_MSG_V3   0x10  /* Contact message for app ver >= 3 */
#define PACKET_CHANNEL_MSG_V3   0x11  /* Channel message for app ver >= 3 */
#define PACKET_CHANNEL_INFO     0x12
#define PACKET_SIGN_START       0x13
#define PACKET_SIGNATURE        0x14
#define PACKET_CUSTOM_VARS      0x15
#define PACKET_ADVERT_PATH      0x16
#define PACKET_TUNING_PARAMS    0x17
#define PACKET_STATS            0x18
#define PACKET_AUTOADD_CONFIG   0x19
#define PACKET_ALLOWED_REPEAT_FREQ 0x1A

#define MAX_SIGN_DATA_LEN       (8 * 1024)

/* Error codes */
#define ERR_UNSUPPORTED         0x01
#define ERR_NOT_FOUND           0x02
#define ERR_TABLE_FULL          0x03
#define ERR_BAD_STATE           0x04
#define ERR_ILLEGAL_ARG         0x06

/* Max LoRa TX power (dBm) for SX1262 */
#define MAX_LORA_TX_POWER       22

/* Helper to put little-endian values */
static inline void put_le16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void put_le32(uint8_t *p, uint32_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF; }
static inline uint32_t get_le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24); }

/* Allowed client repeat frequency ranges (matches Arduino) */
struct FreqRange {
	uint32_t lower_freq, upper_freq;
};

static const FreqRange repeat_freq_ranges[] = {
	{ 433000, 433000 },   /* 433 MHz (ISM band) */
	{ 869000, 869000 },   /* 869 MHz (EU ISM band) */
	{ 918000, 918000 },   /* 918 MHz (US ISM band) */
};

static bool isValidClientRepeatFreq(uint32_t f)
{
	for (size_t i = 0; i < sizeof(repeat_freq_ranges) / sizeof(repeat_freq_ranges[0]); i++) {
		if (f >= repeat_freq_ranges[i].lower_freq && f <= repeat_freq_ranges[i].upper_freq) {
			return true;
		}
	}
	return false;
}

CompanionMesh::CompanionMesh(mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
	mesh::RTCClock &rtc, mesh::PacketManager &mgr, mesh::MeshTables &tables,
	ZephyrDataStore &store)
	: BaseChatMesh(radio, ms, rng, rtc, mgr, tables), _store(&store)
{
	_push_cb = nullptr;
	_write_cb = nullptr;
	_batt_cb = nullptr;
	_radio_reconfig_cb = nullptr;
	_pin_change_cb = nullptr;
	_contact_iter_active = false;
	_contact_iter_idx = 0;
	_contact_iter_lastmod = 0;
	_contact_iter_since = 0;
	_offline_queue_head = 0;
	_offline_queue_tail = 0;
	_offline_queue_count = 0;
	_sync_pending = false;
	memset(_ack_table, 0, sizeof(_ack_table));
	_ack_next_overwrite = 0;
	memset(_advert_paths, 0, sizeof(_advert_paths));
	_next_advert_path_idx = 0;
	_sign_data = nullptr;
	_sign_data_len = 0;
	_sign_data_capacity = 0;
	_pending_login = 0;
	_pending_status = 0;
	_pending_telemetry = 0;
	_pending_discovery = 0;
	_pending_req = 0;
	_app_target_ver = 0;
	_dirty_contacts_expiry = 0;
	_dirty_channels_expiry = 0;
	memset(_send_scope.key, 0, sizeof(_send_scope.key));
	memset(&prefs, 0, sizeof(prefs));
	prefs.node_lat = 0;
	prefs.node_lon = 0;
}

void CompanionMesh::begin()
{
	BaseChatMesh::begin();
}

bool CompanionMesh::allowPacketForward(const mesh::Packet *packet)
{
	(void)packet;
	return prefs.client_repeat != 0;
}

void CompanionMesh::sendFloodScoped(const ContactInfo &recipient, mesh::Packet *pkt, uint32_t delay_millis)
{
	if (_send_scope.isNull()) {
		sendFlood(pkt, delay_millis, prefs.path_hash_mode + 1);
	} else {
		uint16_t codes[2];
		codes[0] = _send_scope.calcTransportCode(pkt);
		codes[1] = 0;
		sendFlood(pkt, codes, delay_millis, prefs.path_hash_mode + 1);
	}
}

void CompanionMesh::sendFloodScoped(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t delay_millis)
{
	if (_send_scope.isNull()) {
		sendFlood(pkt, delay_millis, prefs.path_hash_mode + 1);
	} else {
		uint16_t codes[2];
		codes[0] = _send_scope.calcTransportCode(pkt);
		codes[1] = 0;
		sendFlood(pkt, codes, delay_millis, prefs.path_hash_mode + 1);
	}
}

bool CompanionMesh::onContactPathRecv(ContactInfo &from, uint8_t *in_path, uint8_t in_path_len,
	uint8_t *out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len)
{
	if (extra_type == PAYLOAD_TYPE_RESPONSE && extra_len > 4) {
		uint32_t tag;
		memcpy(&tag, extra, 4);

		if (tag == _pending_discovery) {
			_pending_discovery = 0;

			if (in_path_len <= MAX_PATH_SIZE && out_path_len <= MAX_PATH_SIZE) {
				uint8_t rsp[172];
				int i = 0;
				rsp[i++] = PUSH_CODE_PATH_DISCOVERY_RESP;
				rsp[i++] = 0;  // reserved
				memcpy(&rsp[i], from.id.pub_key, 6);
				i += 6;
				rsp[i++] = out_path_len;
				memcpy(&rsp[i], out_path, out_path_len);
				i += out_path_len;
				rsp[i++] = in_path_len;
				memcpy(&rsp[i], in_path, in_path_len);
				i += in_path_len;
				sendPush(rsp[0], &rsp[1], i - 1);
			} else {
				LOG_WRN("onContactPathRecv: invalid path sizes: out=%d in=%d",
					out_path_len, in_path_len);
			}
			return false;  // don't send reciprocal path
		}
	}
	// let base class handle non-discovery path responses
	return BaseChatMesh::onContactPathRecv(from, in_path, in_path_len,
		out_path, out_path_len, extra_type, extra, extra_len);
}

void CompanionMesh::loop()
{
	BaseChatMesh::loop();

	/* Check for pending lazy contact/channel writes */
	int64_t now = _ms->getMillis();
	if (_dirty_contacts_expiry && now >= _dirty_contacts_expiry) {
		flushDirtyContacts();
	}
	if (_dirty_channels_expiry && now >= _dirty_channels_expiry) {
		flushDirtyChannels();
	}
}

void CompanionMesh::markContactsDirty()
{
	/* Only set the timer on first dirty — don't keep pushing
	 * the deadline forward or a busy mesh never flushes. */
	if (!_dirty_contacts_expiry) {
		_dirty_contacts_expiry = _ms->getMillis() + LAZY_WRITE_DELAY_MS;
	}
}

void CompanionMesh::markChannelsDirty()
{
	if (!_dirty_channels_expiry) {
		_dirty_channels_expiry = _ms->getMillis() + LAZY_WRITE_DELAY_MS;
	}
}

void CompanionMesh::flushDirtyContacts()
{
	if (_dirty_contacts_expiry) {
		LOG_INF("flushDirtyContacts: saving contacts (lazy write)");
		_store->saveContacts(this);
		_dirty_contacts_expiry = 0;
	}
}

void CompanionMesh::flushDirtyChannels()
{
	if (_dirty_channels_expiry) {
		LOG_INF("flushDirtyChannels: saving channels (lazy write)");
		_store->saveChannels(this);
		_dirty_channels_expiry = 0;
	}
}

void CompanionMesh::writeFrame(const uint8_t *data, size_t len)
{
	LOG_DBG("RSP: 0x%02x len=%u", data[0], (unsigned)len);
	if (_write_cb) {
		_write_cb(data, len);
	}
}

void CompanionMesh::sendPacketOk()
{
	uint8_t rsp[] = { PACKET_OK };
	writeFrame(rsp, sizeof(rsp));
}

void CompanionMesh::sendPacketError(uint8_t code)
{
	uint8_t rsp[] = { PACKET_ERROR, code };
	writeFrame(rsp, sizeof(rsp));
}

void CompanionMesh::sendPush(uint8_t code, const uint8_t *data, size_t len)
{
	LOG_DBG("sendPush: code=0x%02x len=%u _push_cb=%s", code, (unsigned)len, _push_cb ? "set" : "NULL");
	if (_push_cb) {
		_push_cb(code, data, len);
	} else {
		LOG_WRN("sendPush: _push_cb is NULL, push lost!");
	}
}

size_t CompanionMesh::serializeContact(uint8_t *buf, const ContactInfo &c, uint8_t header)
{
	size_t i = 0;
	if (header) {
		buf[i++] = header;
	}
	memcpy(&buf[i], c.id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;
	buf[i++] = c.type;
	buf[i++] = c.flags;
	buf[i++] = c.out_path_len;
	memcpy(&buf[i], c.out_path, MAX_PATH_SIZE); i += MAX_PATH_SIZE;
	StrHelper::strzcpy((char *)&buf[i], c.name, 32); i += 32;
	put_le32(&buf[i], c.last_advert_timestamp); i += 4;
	put_le32(&buf[i], (uint32_t)c.gps_lat); i += 4;
	put_le32(&buf[i], (uint32_t)c.gps_lon); i += 4;
	put_le32(&buf[i], c.lastmod); i += 4;
	return i;
}

static bool isChannelMessage(const uint8_t *buf)
{
	// Channel messages have PACKET_CHANNEL_MSG_V3 (0x11) or PACKET_CHANNEL_MSG_RECV (0x08)
	return buf[0] == PACKET_CHANNEL_MSG_V3 || buf[0] == PACKET_CHANNEL_MSG_RECV;
}

void CompanionMesh::queueOfflineMessage(const uint8_t *data, size_t len)
{
	LOG_INF("queueOfflineMessage: len=%u type=0x%02x count_before=%d", (unsigned)len, data[0], _offline_queue_count);
	if (_offline_queue_count >= OFFLINE_QUEUE_SIZE) {
		// Queue full - try to drop oldest channel message first
		int pos = _offline_queue_head;
		for (int i = 0; i < _offline_queue_count; i++) {
			int idx = (pos + i) % OFFLINE_QUEUE_SIZE;
			if (isChannelMessage(_offline_queue[idx].buf)) {
				// Found a channel message, remove it
				for (int j = i; j < _offline_queue_count - 1; j++) {
					int from = (_offline_queue_head + j + 1) % OFFLINE_QUEUE_SIZE;
					int to = (_offline_queue_head + j) % OFFLINE_QUEUE_SIZE;
					_offline_queue[to] = _offline_queue[from];
				}
				_offline_queue_tail = (_offline_queue_tail - 1 + OFFLINE_QUEUE_SIZE) % OFFLINE_QUEUE_SIZE;
				_offline_queue_count--;
				break;
			}
		}
		// If still full (no channel messages found), drop oldest
		if (_offline_queue_count >= OFFLINE_QUEUE_SIZE) {
			_offline_queue_head = (_offline_queue_head + 1) % OFFLINE_QUEUE_SIZE;
			_offline_queue_count--;
		}
	}
	QueuedFrame *f = &_offline_queue[_offline_queue_tail];
	f->len = (len > sizeof(f->buf)) ? sizeof(f->buf) : len;
	memcpy(f->buf, data, f->len);
	_offline_queue_tail = (_offline_queue_tail + 1) % OFFLINE_QUEUE_SIZE;
	_offline_queue_count++;
	LOG_INF("queueOfflineMessage: count_after=%d", _offline_queue_count);
}

bool CompanionMesh::dequeueOfflineMessage(uint8_t *dest, size_t &len)
{
	if (_offline_queue_count == 0) return false;
	QueuedFrame *f = &_offline_queue[_offline_queue_head];
	len = f->len;
	memcpy(dest, f->buf, len);
	_offline_queue_head = (_offline_queue_head + 1) % OFFLINE_QUEUE_SIZE;
	_offline_queue_count--;
	return true;
}

bool CompanionMesh::peekOfflineMessage(uint8_t *dest, size_t &len)
{
	if (_offline_queue_count == 0) return false;
	QueuedFrame *f = &_offline_queue[_offline_queue_head];
	len = f->len;
	memcpy(dest, f->buf, len);
	return true;  /* head NOT advanced — message stays in queue */
}

void CompanionMesh::confirmOfflineMessage()
{
	if (_offline_queue_count == 0) return;
	_offline_queue_head = (_offline_queue_head + 1) % OFFLINE_QUEUE_SIZE;
	_offline_queue_count--;
}

void CompanionMesh::resetContactIterator()
{
	if (_contact_iter_active) {
		// Send contact end if interrupted
		uint8_t rsp[5];
		rsp[0] = PACKET_CONTACT_END;
		put_le32(&rsp[1], _contact_iter_lastmod);
		writeFrame(rsp, sizeof(rsp));
	}
	_contact_iter_active = false;
}

bool CompanionMesh::continueContactIteration()
{
	if (!_contact_iter_active) return false;

	if (_contact_iter_idx < getNumContacts()) {
		ContactInfo c;
		if (getContactByIdx(_contact_iter_idx, c)) {
			// Apply 'since' filter - only send contacts modified after the timestamp
			if (c.lastmod > _contact_iter_since) {
				if (c.lastmod > _contact_iter_lastmod) {
					_contact_iter_lastmod = c.lastmod;
				}
				uint8_t rsp[CONTACT_FRAME_SIZE];
				size_t n = serializeContact(rsp, c, PACKET_CONTACT);
				writeFrame(rsp, n);
			}
		}
		_contact_iter_idx++;
		return true;
	} else {
		// Send PACKET_CONTACT_END with most_recent_lastmod
		uint8_t rsp[5];
		rsp[0] = PACKET_CONTACT_END;
		put_le32(&rsp[1], _contact_iter_lastmod);
		writeFrame(rsp, sizeof(rsp));
		_contact_iter_active = false;
		return false;
	}
}

void CompanionMesh::addPendingAck(uint32_t expected, int contact_idx)
{
	uint32_t now_ms = (uint32_t)_ms->getMillis();
	for (int i = 0; i < ACK_TABLE_SIZE; i++) {
		if (!_ack_table[i].active) {
			_ack_table[i].expected_ack = expected;
			_ack_table[i].contact_idx = contact_idx;
			_ack_table[i].sent_time = now_ms;
			_ack_table[i].active = true;
			return;
		}
	}
	// Table full — circular overwrite
	int idx = _ack_next_overwrite;
	_ack_table[idx].expected_ack = expected;
	_ack_table[idx].contact_idx = contact_idx;
	_ack_table[idx].sent_time = now_ms;
	_ack_table[idx].active = true;
	_ack_next_overwrite = (idx + 1) % ACK_TABLE_SIZE;
}

int CompanionMesh::findAndRemoveAck(uint32_t ack, uint32_t *out_sent_time)
{
	for (int i = 0; i < ACK_TABLE_SIZE; i++) {
		if (_ack_table[i].active && _ack_table[i].expected_ack == ack) {
			if (out_sent_time) {
				*out_sent_time = _ack_table[i].sent_time;
			}
			_ack_table[i].active = false;
			return _ack_table[i].contact_idx;
		}
	}
	return -1;
}

/* DataStoreHost interface */
bool CompanionMesh::onContactLoaded(const ContactInfo &c)
{
	return addContact(c);
}

bool CompanionMesh::getContactForSave(uint32_t idx, ContactInfo &c)
{
	return getContactByIdx(idx, c);
}

bool CompanionMesh::onChannelLoaded(uint8_t idx, const ChannelDetails &ch)
{
	return setChannel(idx, ch);
}

bool CompanionMesh::getChannelForSave(uint8_t idx, ChannelDetails &ch)
{
	// Only save channels that have content (non-empty name or non-zero secret)
	if (getChannel(idx, ch)) {
		// Check if channel has any content
		if (ch.name[0] != '\0') return true;
		// Check if secret is non-zero
		static const uint8_t zeroes[32] = {0};
		if (memcmp(ch.channel.secret, zeroes, 32) != 0) return true;
	}
	return false;
}

/* Storage interface */
int CompanionMesh::getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[])
{
	return _store->getBlobByKey(key, key_len, dest_buf);
}

bool CompanionMesh::putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], int len)
{
	return _store->putBlobByKey(key, key_len, src_buf, len);
}

/* BaseChatMesh virtual implementations */
void CompanionMesh::onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path)
{
	LOG_INF("onDiscoveredContact: '%s' is_new=%d path_len=%d num_contacts=%d",
		contact.name, is_new, path_len, getNumContacts());

	// Mark contacts dirty for lazy save
	markContactsDirty();

	// Update advert path table
	if (path && mesh::Packet::isValidPathLen(path_len)) {
		AdvertPath *ap = &_advert_paths[_next_advert_path_idx];
		memcpy(ap->pubkey_prefix, contact.id.pub_key, 7);
		memcpy(ap->name, contact.name, sizeof(ap->name));
		ap->recv_timestamp = (uint32_t)getRTCClock()->getCurrentTime();
		ap->path_len = mesh::Packet::copyPath(ap->path, path, path_len);
		_next_advert_path_idx = (_next_advert_path_idx + 1) % ADVERT_PATH_TABLE_SIZE;
	}

	// Send push notification
	if (is_new) {
		uint8_t rsp[CONTACT_FRAME_SIZE];
		size_t n = serializeContact(rsp, contact);  /* no header — push code is separate */
		LOG_INF("onDiscoveredContact: sending PUSH_CODE_NEW_ADVERT (full contact)");
		sendPush(PUSH_CODE_NEW_ADVERT, rsp, n);
	} else {
		// ADVERT: send just pubkey
		LOG_INF("onDiscoveredContact: sending PUSH_CODE_ADVERT (pubkey only)");
		sendPush(PUSH_CODE_ADVERT, contact.id.pub_key, PUB_KEY_SIZE);
	}
}

ContactInfo *CompanionMesh::processAck(const uint8_t *data)
{
	uint32_t ack_crc;
	memcpy(&ack_crc, data, 4);

	uint32_t sent_time = 0;
	int contact_idx = findAndRemoveAck(ack_crc, &sent_time);
	if (contact_idx >= 0) {
		ContactInfo ci;
		if (getContactByIdx(contact_idx, ci)) {
			uint8_t ack_push[8];
			memcpy(ack_push, data, 4);
			uint32_t now = (uint32_t)_ms->getMillis();
			uint32_t trip_time = now - sent_time;
			put_le32(&ack_push[4], trip_time);
			sendPush(PUSH_CODE_SEND_CONFIRMED, ack_push, 8);

			return lookupContactByPubKey(ci.id.pub_key, PUB_KEY_SIZE);
		}
	}
	return checkConnectionsAck(data);
}

void CompanionMesh::onContactPathUpdated(const ContactInfo &contact)
{
	markContactsDirty();
	sendPush(PUSH_CODE_PATH_UPDATED, contact.id.pub_key, PUB_KEY_SIZE);
}

void CompanionMesh::onMessageRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const char *text)
{
	LOG_INF("onMessageRecv: from '%s' timestamp=%u text='%s'",
		contact.name, sender_timestamp, text);

	markConnectionActive(contact);
	queueContactMessage(contact, pkt, TXT_TYPE_PLAIN, sender_timestamp, nullptr, 0, text);
	sendPush(PUSH_CODE_MSG_WAITING);
}

void CompanionMesh::queueContactMessage(const ContactInfo &contact, mesh::Packet *pkt,
	uint8_t txt_type, uint32_t sender_timestamp, const uint8_t *extra, int extra_len, const char *text)
{
	// Build incoming message frame for offline queue
	uint8_t frame[MAX_FRAME_SIZE];
	int i = 0;

	if (_app_target_ver >= 3) {
		// V3 format: [0x10][SNR*4][0][0][6-byte pubkey prefix][path_len|0xFF][txt_type][4-byte timestamp][extra][text]
		frame[i++] = PACKET_CONTACT_MSG_V3;
		frame[i++] = pkt ? (int8_t)(pkt->getSNR() * 4) : 0;  // SNR * 4
		frame[i++] = 0;  // reserved1
		frame[i++] = 0;  // reserved2
	} else {
		// V2 format: [0x07][6-byte pubkey prefix][path_len|0xFF][txt_type][4-byte timestamp][extra][text]
		frame[i++] = PACKET_CONTACT_MSG_RECV;
	}

	// 6-byte pubkey prefix
	memcpy(&frame[i], contact.id.pub_key, 6);
	i += 6;

	// path_len: 0xFF if direct, else actual path_len
	frame[i++] = (pkt && pkt->isRouteFlood()) ? pkt->path_len : 0xFF;

	// txt_type
	frame[i++] = txt_type;

	// 4-byte timestamp (little-endian)
	put_le32(&frame[i], sender_timestamp);
	i += 4;

	// extra data (e.g., for signed messages)
	if (extra_len > 0 && extra) {
		memcpy(&frame[i], extra, extra_len);
		i += extra_len;
	}

	// text
	size_t text_len = strlen(text);
	if (i + text_len > sizeof(frame)) {
		text_len = sizeof(frame) - i;
	}
	memcpy(&frame[i], text, text_len);
	i += text_len;

	LOG_INF("queueContactMessage: frame_len=%d type=0x%02x", i, frame[0]);
	queueOfflineMessage(frame, i);
}

void CompanionMesh::onCommandDataRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const char *text)
{
	LOG_INF("onCommandDataRecv: from '%s' timestamp=%u text='%s'",
		contact.name, sender_timestamp, text);

	markConnectionActive(contact);
	queueContactMessage(contact, pkt, TXT_TYPE_CLI_DATA, sender_timestamp, nullptr, 0, text);
	sendPush(PUSH_CODE_MSG_WAITING);
}

void CompanionMesh::onSignedMessageRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text)
{
	LOG_INF("onSignedMessageRecv: from '%s' timestamp=%u text='%s'",
		contact.name, sender_timestamp, text);

	markConnectionActive(contact);
	markContactsDirty();
	// sender_prefix is 4 bytes
	queueContactMessage(contact, pkt, TXT_TYPE_SIGNED_PLAIN, sender_timestamp, sender_prefix, 4, text);
	sendPush(PUSH_CODE_MSG_WAITING);
}

uint32_t CompanionMesh::calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const
{
	return 500 + (uint32_t)(16.0f * pkt_airtime_millis);
}

uint32_t CompanionMesh::calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const
{
	return 500 + (uint32_t)((pkt_airtime_millis * 6.0f + 250) * (path_len + 1));
}

void CompanionMesh::onSendTimeout()
{
	// Message send timed out - could notify UI
}

void CompanionMesh::onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt,
	uint32_t timestamp, const char *text)
{
	LOG_INF("onChannelMessageRecv: timestamp=%u text='%s'", timestamp, text);

	// Build incoming channel message frame
	uint8_t frame[MAX_FRAME_SIZE];
	int i = 0;

	if (_app_target_ver >= 3) {
		// V3: [0x11][SNR*4][0][0][channel_idx][path_len|0xFF][txt_type][4-byte timestamp][text]
		frame[i++] = PACKET_CHANNEL_MSG_V3;
		frame[i++] = pkt ? (int8_t)(pkt->getSNR() * 4) : 0;
		frame[i++] = 0;  // reserved1
		frame[i++] = 0;  // reserved2
	} else {
		// V2: [0x08][channel_idx][path_len|0xFF][txt_type][4-byte timestamp][text]
		frame[i++] = PACKET_CHANNEL_MSG_RECV;
	}

	// channel index
	uint8_t channel_idx = findChannelIdx(channel);
	frame[i++] = channel_idx;

	// path_len: 0xFF if direct, else actual path_len
	frame[i++] = (pkt && pkt->isRouteFlood()) ? pkt->path_len : 0xFF;

	// txt_type
	frame[i++] = TXT_TYPE_PLAIN;

	// 4-byte timestamp (little-endian)
	put_le32(&frame[i], timestamp);
	i += 4;

	// text
	size_t text_len = strlen(text);
	if (i + text_len > sizeof(frame)) {
		text_len = sizeof(frame) - i;
	}
	memcpy(&frame[i], text, text_len);
	i += text_len;

	LOG_INF("onChannelMessageRecv: frame_len=%d channel_idx=%d", i, channel_idx);
	queueOfflineMessage(frame, i);
	sendPush(PUSH_CODE_MSG_WAITING);
}

uint8_t CompanionMesh::onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp,
	const uint8_t *data, uint8_t len, uint8_t *reply)
{
	if (len < 1) return 0;

	if (data[0] == REQ_TYPE_GET_TELEMETRY_DATA) {
		// Calculate permissions based on prefs and contact flags
		uint8_t permissions = 0;
		uint8_t cp = contact.flags >> 1;  // LSB is 'favourite' bit, use upper bits for perms

		if (prefs.telemetry_mode_base == TELEM_MODE_ALLOW_ALL) {
			permissions = TELEM_PERM_BASE;
		} else if (prefs.telemetry_mode_base == TELEM_MODE_ALLOW_FLAGS) {
			permissions = cp & TELEM_PERM_BASE;
		}

		if (prefs.telemetry_mode_loc == TELEM_MODE_ALLOW_ALL) {
			permissions |= TELEM_PERM_LOCATION;
		} else if (prefs.telemetry_mode_loc == TELEM_MODE_ALLOW_FLAGS) {
			permissions |= cp & TELEM_PERM_LOCATION;
		}

		if (prefs.telemetry_mode_env == TELEM_MODE_ALLOW_ALL) {
			permissions |= TELEM_PERM_ENVIRONMENT;
		} else if (prefs.telemetry_mode_env == TELEM_MODE_ALLOW_FLAGS) {
			permissions |= cp & TELEM_PERM_ENVIRONMENT;
		}

		// Apply permission mask from request (if present)
		if (len > 1) {
			uint8_t perm_mask = ~data[1];  // First reserved byte is inverse mask
			permissions &= perm_mask;
		}

		// Only respond if base permission is granted
		if (permissions & TELEM_PERM_BASE) {
			LOG_INF("onContactRequest: telemetry authorized (perms=0x%02x)", permissions);

			// Build Cayenne LPP telemetry response
			int i = 0;

			// Reflect sender_timestamp back as tag (4 bytes)
			memcpy(reply, &sender_timestamp, 4);
			i += 4;

			const uint8_t CH_SELF = 1;

			// Battery voltage: [channel][LPP_VOLTAGE=116][2-byte 0.01V big-endian]
			uint16_t batt_mv = _batt_cb ? _batt_cb() : 0;
			reply[i++] = CH_SELF;
			reply[i++] = 116;  // LPP_VOLTAGE
			uint16_t batt_scaled = batt_mv / 10;
			reply[i++] = (batt_scaled >> 8) & 0xFF;
			reply[i++] = batt_scaled & 0xFF;

			// GPS position if authorized and available
			if (permissions & TELEM_PERM_LOCATION) {
				struct gps_position pos;
				if (gps_is_available() && gps_get_last_known_position(&pos)) {
					reply[i++] = CH_SELF;
					reply[i++] = 136;  // LPP_GPS
					int32_t lat = (int32_t)(pos.latitude_ndeg / 100000);
					int32_t lon = (int32_t)(pos.longitude_ndeg / 100000);
					int32_t alt = pos.altitude_mm / 10;
					reply[i++] = (lat >> 16) & 0xFF;
					reply[i++] = (lat >> 8) & 0xFF;
					reply[i++] = lat & 0xFF;
					reply[i++] = (lon >> 16) & 0xFF;
					reply[i++] = (lon >> 8) & 0xFF;
					reply[i++] = lon & 0xFF;
					reply[i++] = (alt >> 16) & 0xFF;
					reply[i++] = (alt >> 8) & 0xFF;
					reply[i++] = alt & 0xFF;
				} else if (prefs.node_lat != 0 || prefs.node_lon != 0) {
					// Use configured position
					reply[i++] = CH_SELF;
					reply[i++] = 136;  // LPP_GPS
					int32_t lat = (int32_t)(prefs.node_lat * 10000);
					int32_t lon = (int32_t)(prefs.node_lon * 10000);
					int32_t alt = 0;
					reply[i++] = (lat >> 16) & 0xFF;
					reply[i++] = (lat >> 8) & 0xFF;
					reply[i++] = lat & 0xFF;
					reply[i++] = (lon >> 16) & 0xFF;
					reply[i++] = (lon >> 8) & 0xFF;
					reply[i++] = lon & 0xFF;
					reply[i++] = (alt >> 16) & 0xFF;
					reply[i++] = (alt >> 8) & 0xFF;
					reply[i++] = alt & 0xFF;
				}
			}

			// Environment sensors if authorized and available
			if (permissions & TELEM_PERM_ENVIRONMENT) {
				struct env_data env;
				if (env_sensors_read(&env) == 0) {
					if (env.has_temperature) {
						reply[i++] = CH_SELF;
						reply[i++] = LPP_TEMPERATURE;
						int16_t temp = (int16_t)(env.temperature_c * 10);
						reply[i++] = (temp >> 8) & 0xFF;
						reply[i++] = temp & 0xFF;
					} else if (env.has_mcu_temperature) {
						// MCU die temp as fallback when no external sensor
						reply[i++] = CH_SELF;
						reply[i++] = LPP_TEMPERATURE;
						int16_t temp = (int16_t)(env.mcu_temperature_c * 10);
						reply[i++] = (temp >> 8) & 0xFF;
						reply[i++] = temp & 0xFF;
					}
					if (env.has_humidity) {
						reply[i++] = CH_SELF;
						reply[i++] = LPP_RELATIVE_HUMIDITY;
						reply[i++] = (uint8_t)(env.humidity_pct * 2);
					}
					if (env.has_pressure) {
						reply[i++] = CH_SELF;
						reply[i++] = LPP_BAROMETRIC_PRESSURE;
						uint16_t press = (uint16_t)(env.pressure_hpa * 10);
						reply[i++] = (press >> 8) & 0xFF;
						reply[i++] = press & 0xFF;
					}
				}

				// Power monitor telemetry (INA219/INA3221/ina2xx)
				if (power_sensors_available()) {
					struct power_data pwr;
					if (power_sensors_read(&pwr) == 0) {
						uint8_t ch = CH_SELF + 1;
						for (int j = 0; j < pwr.num_channels; j++) {
							if (pwr.channels[j].valid) {
								// Voltage: [ch][LPP_VOLTAGE=116][2-byte 0.01V]
								reply[i++] = ch;
								reply[i++] = 116;
								uint16_t v = (uint16_t)(pwr.channels[j].voltage_v * 100);
								reply[i++] = (v >> 8) & 0xFF;
								reply[i++] = v & 0xFF;
								// Current: [ch][LPP_CURRENT=117][2-byte 0.001A]
								reply[i++] = ch;
								reply[i++] = 117;
								uint16_t c = (uint16_t)(pwr.channels[j].current_a * 1000);
								reply[i++] = (c >> 8) & 0xFF;
								reply[i++] = c & 0xFF;
								// Power: [ch][LPP_POWER=128][2-byte 1W]
								reply[i++] = ch;
								reply[i++] = 128;
								uint16_t p = (uint16_t)(pwr.channels[j].power_w);
								reply[i++] = (p >> 8) & 0xFF;
								reply[i++] = p & 0xFF;
								ch++;
							}
						}
					}
				}
			}

			// Trigger GPS wake for fresh fix on next request
			if (gps_is_available() && gps_is_enabled()) {
				gps_request_fresh_fix();
			}

			return i;
		} else {
			LOG_INF("onContactRequest: telemetry denied for contact");
		}
	}

	return 0;  // Unknown request or denied
}

void CompanionMesh::onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len)
{
	LOG_INF("onContactResponse: len=%d from contact", len);
	if (len < 4) return;  // Need at least 4-byte tag

	uint32_t tag;
	memcpy(&tag, data, 4);
	LOG_INF("onContactResponse: tag=%08x, _pending_login=%08x", tag, _pending_login);

	// Check for login response
	if (_pending_login && memcmp(&_pending_login, contact.id.pub_key, 4) == 0) {
		LOG_INF("onContactResponse: LOGIN RESPONSE MATCHED!");
		_pending_login = 0;

		uint8_t rsp[16];
		int i = 0;
		if (len > 4 && memcmp(&data[4], "OK", 2) == 0) {
			// Legacy Repeater login OK response
			rsp[i++] = PUSH_CODE_LOGIN_SUCCESS;
			rsp[i++] = 0;  // legacy: is_admin = false
			memcpy(&rsp[i], contact.id.pub_key, 6);
			i += 6;
		} else if (len > 4 && data[4] == RESP_SERVER_LOGIN_OK) {
			// New login response format — start keepalive connection
			if (len > 5) {
				uint16_t keep_alive_secs = ((uint16_t)data[5]) * 16;
				if (keep_alive_secs > 0) {
					startConnection(contact, keep_alive_secs);
				}
			}
			rsp[i++] = PUSH_CODE_LOGIN_SUCCESS;
			rsp[i++] = (len > 6) ? data[6] : 0;  // permissions (is_admin)
			memcpy(&rsp[i], contact.id.pub_key, 6);
			i += 6;
			memcpy(&rsp[i], &tag, 4);  // server timestamp
			i += 4;
			if (len > 7) rsp[i++] = data[7];  // ACL permissions
			else rsp[i++] = 0;
			if (len > 12) rsp[i++] = data[12];  // FIRMWARE_VER_LEVEL
			else rsp[i++] = 0;
		} else {
			// Login failed
			rsp[i++] = PUSH_CODE_LOGIN_FAIL;
			rsp[i++] = 0;  // reserved
			memcpy(&rsp[i], contact.id.pub_key, 6);
			i += 6;
		}
		sendPush(rsp[0], &rsp[1], i - 1);
		return;
	}

	// Check for status response
	if (_pending_status && len > 4 && memcmp(&_pending_status, contact.id.pub_key, 4) == 0) {
		_pending_status = 0;

		uint8_t rsp[128];
		int i = 0;
		rsp[i++] = PUSH_CODE_STATUS_RESPONSE;
		rsp[i++] = 0;  // reserved
		memcpy(&rsp[i], contact.id.pub_key, 6);
		i += 6;
		int data_len = len - 4;
		if (data_len > (int)sizeof(rsp) - i) data_len = sizeof(rsp) - i;
		memcpy(&rsp[i], &data[4], data_len);
		i += data_len;
		sendPush(rsp[0], &rsp[1], i - 1);
		return;
	}

	// Check for telemetry response
	if (len > 4 && tag == _pending_telemetry) {
		_pending_telemetry = 0;

		uint8_t rsp[128];
		int i = 0;
		rsp[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
		rsp[i++] = 0;  // reserved
		memcpy(&rsp[i], contact.id.pub_key, 6);
		i += 6;
		int data_len = len - 4;
		if (data_len > (int)sizeof(rsp) - i) data_len = sizeof(rsp) - i;
		memcpy(&rsp[i], &data[4], data_len);
		i += data_len;
		sendPush(rsp[0], &rsp[1], i - 1);
		return;
	}

	// Path discovery responses are now handled by onContactPathRecv() override,
	// which has access to the actual in_path/out_path routing data.

	// Check for binary request response
	if (len > 4 && tag == _pending_req) {
		_pending_req = 0;

		uint8_t rsp[MAX_FRAME_SIZE];
		int i = 0;
		rsp[i++] = PUSH_CODE_BINARY_RESPONSE;
		rsp[i++] = 0;  // reserved
		// Arduino: sends tag (4 bytes) so app can match response to request
		memcpy(&rsp[i], &tag, 4);
		i += 4;
		int data_len = len - 4;
		if (data_len > (int)sizeof(rsp) - i) data_len = sizeof(rsp) - i;
		memcpy(&rsp[i], &data[4], data_len);
		i += data_len;
		sendPush(rsp[0], &rsp[1], i - 1);
		return;
	}
}

/* Raw packet logging - sends all RX packets to app for "heard X repeats" etc */
void CompanionMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len)
{
	// Arduino: PUSH_CODE_LOG_RX_DATA (0x88), snr*4, rssi, raw_bytes...
	if (len + 3 > MAX_FRAME_SIZE) return;  // buffer overflow protection

	uint8_t buf[MAX_FRAME_SIZE];
	int i = 0;
	buf[i++] = PUSH_CODE_LOG_RX_DATA;
	buf[i++] = (int8_t)(snr * 4);
	buf[i++] = (int8_t)(rssi);
	memcpy(&buf[i], raw, len);
	i += len;

	sendPush(buf[0], &buf[1], i - 1);
}

/* Trace path response - for ping/TRACE commands */
void CompanionMesh::onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
	const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len)
{
	LOG_INF("onTraceRecv: tag=0x%08x auth=0x%08x flags=0x%02x path_len=%d",
		tag, auth_code, flags, path_len);

	// path_sz is encoded in flags bits 0-1 (0=1, 1=2, 2=4, 3=8 byte hash)
	uint8_t path_sz = flags & 0x03;
	// Calculate total size: 12 header + path_hashes + path_snrs + final SNR
	size_t needed = 12 + path_len + (path_len >> path_sz) + 1;
	if (needed > MAX_FRAME_SIZE) return;  // buffer overflow protection

	uint8_t buf[MAX_FRAME_SIZE];
	int i = 0;
	buf[i++] = PUSH_CODE_TRACE_DATA;
	buf[i++] = 0;  // reserved
	buf[i++] = path_len;
	buf[i++] = flags;
	memcpy(&buf[i], &tag, 4); i += 4;
	memcpy(&buf[i], &auth_code, 4); i += 4;
	memcpy(&buf[i], path_hashes, path_len); i += path_len;
	memcpy(&buf[i], path_snrs, path_len >> path_sz); i += (path_len >> path_sz);
	buf[i++] = (int8_t)(packet->getSNR() * 4);

	sendPush(buf[0], &buf[1], i - 1);
}

/* Control data response - for repeater discovery, etc */
void CompanionMesh::onControlDataRecv(mesh::Packet *packet)
{
	LOG_INF("onControlDataRecv: payload_len=%d path_len=%d", packet->payload_len, packet->path_len);

	// Buffer: [PUSH_CODE_CONTROL_DATA][snr*4][rssi][path_len][payload...]
	if (packet->payload_len + 4 > MAX_FRAME_SIZE) {
		LOG_WRN("onControlDataRecv: payload too long (%d)", packet->payload_len);
		return;
	}

	uint8_t buf[MAX_FRAME_SIZE];
	int i = 0;
	buf[i++] = PUSH_CODE_CONTROL_DATA;
	buf[i++] = (int8_t)(packet->getSNR() * 4);
	buf[i++] = (int8_t)(_radio->getLastRSSI());
	buf[i++] = packet->path_len;
	memcpy(&buf[i], packet->payload, packet->payload_len);
	i += packet->payload_len;

	sendPush(buf[0], &buf[1], i - 1);
}

/* Raw data response - for custom packet types */
void CompanionMesh::onRawDataRecv(mesh::Packet *packet)
{
	LOG_INF("onRawDataRecv: payload_len=%d", packet->payload_len);

	// Buffer: [PUSH_CODE_RAW_DATA][snr*4][rssi][reserved][payload...]
	if (packet->payload_len + 4 > MAX_FRAME_SIZE) {
		LOG_WRN("onRawDataRecv: payload too long (%d)", packet->payload_len);
		return;
	}

	uint8_t buf[MAX_FRAME_SIZE];
	int i = 0;
	buf[i++] = PUSH_CODE_RAW_DATA;
	buf[i++] = (int8_t)(packet->getSNR() * 4);
	buf[i++] = (int8_t)(_radio->getLastRSSI());
	buf[i++] = 0xFF;  // reserved (possibly path_len in future)
	memcpy(&buf[i], packet->payload, packet->payload_len);
	i += packet->payload_len;

	sendPush(buf[0], &buf[1], i - 1);
}

uint32_t CompanionMesh::getRetransmitDelay(const mesh::Packet *packet)
{
	float factor = getContentionTracker().getFloodDelayFactor();
	uint32_t t = (uint32_t)(_radio->getEstAirtimeFor(
		packet->getPathByteLen() + packet->payload_len + 2) * factor);
	uint32_t max_jitter = 5 * t;
	/* Cap jitter to 2000ms to avoid excessive latency in very dense areas.
	 * Reactive backoff will fine-tune further if needed. */
	if (max_jitter > 2000) max_jitter = 2000;
	/* Floor: give downstream nodes time to finish RX processing
	 * and return to RX mode before we TX (~20ms settle) */
	return 20 + getRNG()->nextInt(0, max_jitter + 1);
}

uint32_t CompanionMesh::getDirectRetransmitDelay(const mesh::Packet *packet)
{
	uint32_t t = _radio->getEstAirtimeFor(
		packet->getPathByteLen() + packet->payload_len + 2);
	/* Floor: give downstream nodes time to finish RX processing
	 * and return to RX mode before we TX (~20ms settle + jitter) */
	return 20 + getRNG()->nextInt(0, t / 10 + 1);
}

uint8_t CompanionMesh::getDutyCyclePercent() const
{
	return (uint8_t)prefs.airtime_factor;
}

uint8_t CompanionMesh::getExtraAckTransmitCount() const
{
	return prefs.multi_acks;
}

/* Auto-add filtering */
bool CompanionMesh::isAutoAddEnabled() const
{
	return (prefs.manual_add_contacts & 1) == 0;
}

bool CompanionMesh::shouldAutoAddContactType(uint8_t contact_type) const
{
	if ((prefs.manual_add_contacts & 1) == 0) {
		return true;  // Auto-add all when not in manual mode
	}

	uint8_t type_bit = 0;
	switch (contact_type) {
	case ADV_TYPE_CHAT:
		type_bit = AUTO_ADD_CHAT;
		break;
	case ADV_TYPE_REPEATER:
		type_bit = AUTO_ADD_REPEATER;
		break;
	case ADV_TYPE_ROOM:
		type_bit = AUTO_ADD_ROOM_SERVER;
		break;
	case ADV_TYPE_SENSOR:
		type_bit = AUTO_ADD_SENSOR;
		break;
	default:
		return false;
	}
	return (prefs.autoadd_config & type_bit) != 0;
}

bool CompanionMesh::shouldOverwriteWhenFull() const
{
	return (prefs.autoadd_config & AUTO_ADD_OVERWRITE_OLDEST) != 0;
}

uint8_t CompanionMesh::getAutoAddMaxHops() const
{
	return prefs.autoadd_max_hops;
}

void CompanionMesh::onContactsFull()
{
	sendPush(PUSH_CODE_CONTACTS_FULL);
}

void CompanionMesh::onContactOverwrite(const uint8_t *pub_key)
{
	_store->deleteBlobByKey(pub_key, PUB_KEY_SIZE);
	sendPush(PUSH_CODE_CONTACT_DELETED, pub_key, PUB_KEY_SIZE);
}

/* Advert path tracking — return the N most recently heard entries,
 * sorted by recv_timestamp descending (newest first).
 * The advert path table is a circular buffer, so we can't just
 * take the first N entries — we must find the N with highest timestamps. */
int CompanionMesh::getRecentlyHeard(AdvertPath dest[], int max_num)
{
	int count = 0;

	/* Collect all non-empty entries (up to max_num) keeping the
	 * most recent ones.  Simple insertion: for each table entry,
	 * if the dest array isn't full, append; otherwise replace the
	 * oldest entry in dest if this one is newer. */
	for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {
		if (_advert_paths[i].recv_timestamp == 0) {
			continue;
		}
		if (count < max_num) {
			dest[count++] = _advert_paths[i];
		} else {
			/* Find oldest entry in dest */
			int oldest = 0;
			for (int j = 1; j < count; j++) {
				if (dest[j].recv_timestamp < dest[oldest].recv_timestamp) {
					oldest = j;
				}
			}
			/* Replace if this entry is newer */
			if (_advert_paths[i].recv_timestamp > dest[oldest].recv_timestamp) {
				dest[oldest] = _advert_paths[i];
			}
		}
	}

	/* Sort descending by recv_timestamp (newest first) */
	for (int i = 0; i < count - 1; i++) {
		for (int j = i + 1; j < count; j++) {
			if (dest[j].recv_timestamp > dest[i].recv_timestamp) {
				AdvertPath tmp = dest[i];
				dest[i] = dest[j];
				dest[j] = tmp;
			}
		}
	}

	return count;
}

const AdvertPath *CompanionMesh::findAdvertPath(const uint8_t *pubkey_prefix, int prefix_len)
{
	int match_len = (prefix_len < 7) ? prefix_len : 7;
	for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {
		if (_advert_paths[i].recv_timestamp != 0 &&
			memcmp(_advert_paths[i].pubkey_prefix, pubkey_prefix, match_len) == 0) {
			return &_advert_paths[i];
		}
	}
	return nullptr;
}

bool CompanionMesh::handleProtocolFrame(const uint8_t *data, size_t len)
{
	if (len < 1) return false;

	/* Debug: log all incoming commands */
	LOG_DBG("CMD: 0x%02x len=%u", data[0], (unsigned)len);

	// Reset contact iterator when any new command is received (except during iteration)
	if (data[0] != CMD_GET_CONTACTS && _contact_iter_active) {
		resetContactIterator();
	}

	switch (data[0]) {
	case CMD_APP_START:
		if (len >= 8) {
			// Reset contact iterator for fresh start
			_contact_iter_active = false;

			// Return SELF_INFO
			uint8_t rsp[90];  // 58 fixed + up to 32 bytes name
			size_t i = 0;
			rsp[i++] = PACKET_SELF_INFO;
			rsp[i++] = ADV_TYPE_CHAT;
			rsp[i++] = prefs.tx_power_dbm;
			rsp[i++] = MAX_LORA_TX_POWER;  // Max allowed TX power
			memcpy(&rsp[i], self_id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;
			int32_t lat = (int32_t)(prefs.node_lat * 1000000.0);
			int32_t lon = (int32_t)(prefs.node_lon * 1000000.0);
			put_le32(&rsp[i], lat); i += 4;
			put_le32(&rsp[i], lon); i += 4;
			rsp[i++] = prefs.multi_acks;
			rsp[i++] = prefs.advert_loc_policy;
			// Telemetry modes: (env << 4) | (loc << 2) | base
			rsp[i++] = (prefs.telemetry_mode_env << 4) | (prefs.telemetry_mode_loc << 2) | prefs.telemetry_mode_base;
			rsp[i++] = prefs.manual_add_contacts;
			put_le32(&rsp[i], (uint32_t)(prefs.freq * 1000)); i += 4;
			put_le32(&rsp[i], (uint32_t)(prefs.bw * 1000)); i += 4;
			rsp[i++] = prefs.sf;
			rsp[i++] = prefs.cr;
			size_t name_len = strnlen(prefs.node_name, sizeof(prefs.node_name) - 1);
			memcpy(&rsp[i], prefs.node_name, name_len);
			i += name_len;
			writeFrame(rsp, i);
			return true;
		}
		break;

	case CMD_GET_CONTACTS:
		if (_contact_iter_active) {
			sendPacketError(ERR_BAD_STATE);
		} else {
			// Parse optional 'since' parameter for incremental sync
			if (len >= 5) {
				_contact_iter_since = get_le32(&data[1]);
			} else {
				_contact_iter_since = 0;
			}

			// Send PACKET_CONTACT_START with total count (unfiltered)
			uint8_t rsp[5];
			rsp[0] = PACKET_CONTACT_START;
			put_le32(&rsp[1], (uint32_t)getNumContacts());
			writeFrame(rsp, sizeof(rsp));
			_contact_iter_idx = 0;
			_contact_iter_lastmod = 0;
			_contact_iter_active = true;
		}
		return true;

	case CMD_ADD_UPDATE_CONTACT:
		if (len >= 1 + PUB_KEY_SIZE + 1 + 1 + 1 + MAX_PATH_SIZE + 32 + 4) {
			const uint8_t *pub_key = &data[1];
			ContactInfo *existing = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);

			ContactInfo c;
			size_t i = 1;
			memcpy(c.id.pub_key, &data[i], PUB_KEY_SIZE); i += PUB_KEY_SIZE;
			c.type = data[i++];
			c.flags = data[i++];
			c.out_path_len = data[i++];
			memcpy(c.out_path, &data[i], MAX_PATH_SIZE); i += MAX_PATH_SIZE;
			memcpy(c.name, &data[i], 32); i += 32;
			c.last_advert_timestamp = get_le32(&data[i]); i += 4;
			c.gps_lat = 0;
			c.gps_lon = 0;
			c.lastmod = (uint32_t)getRTCClock()->getCurrentTime();
			if (len >= i + 8) {
				c.gps_lat = (int32_t)get_le32(&data[i]); i += 4;
				c.gps_lon = (int32_t)get_le32(&data[i]); i += 4;
				if (len >= i + 4) {
					c.lastmod = get_le32(&data[i]);
				}
			}
			c.sync_since = 0;
			c.shared_secret_valid = false;

			if (existing) {
				// Update existing
				*existing = c;
				markContactsDirty();
				sendPacketOk();
			} else if (addContact(c)) {
				markContactsDirty();
				sendPacketOk();
			} else {
				sendPacketError(ERR_TABLE_FULL);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_REMOVE_CONTACT:
		if (len >= 1 + PUB_KEY_SIZE) {
			ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (c && removeContact(*c)) {
				_store->deleteBlobByKey(&data[1], PUB_KEY_SIZE);
				markContactsDirty();
				sendPacketOk();
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_RESET_PATH:
		if (len >= 1 + PUB_KEY_SIZE) {
			ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (c) {
				resetPathTo(*c);
				markContactsDirty();
				sendPacketOk();
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_GET_CONTACT_BY_KEY:
		if (len >= 1 + PUB_KEY_SIZE) {
			ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (c) {
				uint8_t rsp[CONTACT_FRAME_SIZE];
				size_t n = serializeContact(rsp, *c, PACKET_CONTACT);
				writeFrame(rsp, n);
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_GET_CHANNEL:
		// Format: [cmd][channel_idx]
		// Response: [code][idx][32 name][16 secret] = 50 bytes (matches Arduino)
		// Arduino returns channel info even if slot is empty (name[0]=='\0')
		if (len >= 2 && data[1] < MAX_GROUP_CHANNELS) {
			ChannelDetails ch;
			if (getChannel(data[1], ch)) {
				uint8_t rsp[50];
				int i = 0;
				rsp[i++] = PACKET_CHANNEL_INFO;
				rsp[i++] = data[1];
				memcpy(&rsp[i], ch.name, 32);
				i += 32;
				memcpy(&rsp[i], ch.channel.secret, 16);  // return 128-bit secret
				i += 16;
				writeFrame(rsp, i);
			} else {
				sendPacketError(ERR_NOT_FOUND);  // Only if index out of range
			}
			return true;
		}
		break;

	case CMD_SET_CHANNEL:
		// Format: [cmd][idx][32 name][16 or 32 secret]
		// Arduino rejects 32-byte secrets, but we accept 16-byte (50 bytes total)
		if (len >= 2 + 32 + 32 && data[1] < MAX_GROUP_CHANNELS) {
			// 32-byte secret not supported (matches Arduino)
			sendPacketError(ERR_UNSUPPORTED);
			return true;
		}
		if (len >= 2 + 32 + 16 && data[1] < MAX_GROUP_CHANNELS) {
			ChannelDetails ch;
			// Copy name (null-terminate if needed)
			memcpy(ch.name, &data[2], 32);
			ch.name[31] = '\0';  // ensure null termination
			// Copy 16-byte secret, zero upper 16 bytes
			memset(ch.channel.secret, 0, sizeof(ch.channel.secret));
			memcpy(ch.channel.secret, &data[2 + 32], 16);
			// setChannel computes the hash based on whether upper 16 bytes are zero
			if (setChannel(data[1], ch)) {
				markChannelsDirty();
				sendPacketOk();
			} else {
				sendPacketError(ERR_NOT_FOUND);  // bad channel_idx
			}
			return true;
		}
		break;

	case CMD_GET_BATT_AND_STORAGE: {
		uint8_t rsp[11];
		rsp[0] = PACKET_BATTERY;
		put_le16(&rsp[1], _batt_cb ? _batt_cb() : 0);
		put_le32(&rsp[3], _store->getStorageUsedKb());
		put_le32(&rsp[7], _store->getStorageTotalKb());
		writeFrame(rsp, sizeof(rsp));
		return true;
	}

	case CMD_GET_STATS:
		if (len >= 2) {
			uint8_t stats_type = data[1];
			if (stats_type == 0) {
				// STATS_TYPE_CORE: battery_mv(2) + uptime_secs(4) + err_flags(2) + queue_len(1)
				uint8_t rsp[11];
				size_t i = 0;
				rsp[i++] = PACKET_STATS;
				rsp[i++] = 0;  // STATS_TYPE_CORE
				put_le16(&rsp[i], _batt_cb ? _batt_cb() : 0); i += 2;
				put_le32(&rsp[i], (uint32_t)(_ms->getMillis() / 1000)); i += 4;  // uptime_secs
				put_le16(&rsp[i], getErrFlags()); i += 2;
				rsp[i++] = (uint8_t)_mgr->getOutboundCount((uint32_t)_ms->getMillis());
				writeFrame(rsp, i);
			} else if (stats_type == 1) {
				// STATS_TYPE_RADIO: noise_floor(2) + last_rssi(1) + last_snr(1) + tx_air_secs(4) + rx_air_secs(4)
				uint8_t rsp[14];
				size_t i = 0;
				rsp[i++] = PACKET_STATS;
				rsp[i++] = 1;  // STATS_TYPE_RADIO
				put_le16(&rsp[i], (int16_t)_radio->getNoiseFloor()); i += 2;
				rsp[i++] = (int8_t)_radio->getLastRSSI();
				rsp[i++] = (int8_t)(_radio->getLastSNR() * 4);  // scaled by 4
				put_le32(&rsp[i], getTotalAirTime() / 1000); i += 4;  // tx_air_secs
				put_le32(&rsp[i], getReceiveAirTime() / 1000); i += 4;  // rx_air_secs
				writeFrame(rsp, i);
			} else if (stats_type == 2) {
				// STATS_TYPE_PACKETS: recv(4) + sent(4) + sent_flood(4) + sent_direct(4) + recv_flood(4) + recv_direct(4) + recv_errors(4)
				uint8_t rsp[30];
				size_t i = 0;
				rsp[i++] = PACKET_STATS;
				rsp[i++] = 2;  // STATS_TYPE_PACKETS
				uint32_t sent_flood = getNumSentFlood();
				uint32_t sent_direct = getNumSentDirect();
				uint32_t recv_flood = getNumRecvFlood();
				uint32_t recv_direct = getNumRecvDirect();
				put_le32(&rsp[i], recv_flood + recv_direct); i += 4;  // total recv
				put_le32(&rsp[i], sent_flood + sent_direct); i += 4;  // total sent
				put_le32(&rsp[i], sent_flood); i += 4;
				put_le32(&rsp[i], sent_direct); i += 4;
				put_le32(&rsp[i], recv_flood); i += 4;
				put_le32(&rsp[i], recv_direct); i += 4;
				put_le32(&rsp[i], _radio->getPacketsRecvErrors()); i += 4;
				writeFrame(rsp, i);
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SHARE_CONTACT:
		if (len >= 1 + PUB_KEY_SIZE) {
			ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (c && shareContactZeroHop(*c)) {
				sendPacketOk();
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_EXPORT_CONTACT:
		if (len < 1 + PUB_KEY_SIZE) {
			// Export SELF - create self advert packet
			mesh::Packet *pkt;
			if (prefs.advert_loc_policy == 0) {  // ADVERT_LOC_NONE
				pkt = createSelfAdvert(prefs.node_name);
			} else {
				pkt = createSelfAdvert(prefs.node_name, prefs.node_lat, prefs.node_lon);
			}
			if (pkt) {
				pkt->header |= ROUTE_TYPE_FLOOD;
				uint8_t rsp[128];
				rsp[0] = PACKET_EXPORT_CONTACT;
				uint8_t out_len = pkt->writeTo(&rsp[1]);
				releasePacket(pkt);
				writeFrame(rsp, out_len + 1);
			} else {
				sendPacketError(ERR_TABLE_FULL);
			}
		} else {
			// Export specific contact by pubkey
			ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (c) {
				uint8_t rsp[128];
				rsp[0] = PACKET_EXPORT_CONTACT;
				uint8_t export_len = exportContact(*c, &rsp[1]);
				if (export_len > 0) {
					writeFrame(rsp, export_len + 1);
				} else {
					sendPacketError(ERR_BAD_STATE);
				}
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		}
		return true;

	case CMD_IMPORT_CONTACT:
		// Arduino: len > 2 + 32 + 64 = 98 bytes (header + pubkey + signature)
		if (len > 2 + 32 + 64) {
			if (importContact(&data[1], len - 1)) {
				markContactsDirty();
				sendPacketOk();
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);  // Match Arduino error code
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_TXT_MSG:
		// Frame format: cmd(1) + txt_type(1) + attempt(1) + timestamp(4) + pub_key_prefix(6) + text(N)
		// Minimum: 1 + 1 + 1 + 4 + 6 + 1 = 14 bytes
		LOG_INF("CMD_SEND_TXT_MSG: len=%u (min=14)", (unsigned)len);
		if (len >= 14) {
			int i = 1;
			uint8_t txt_type = data[i++];
			uint8_t attempt = data[i++];
			uint32_t msg_timestamp = get_le32(&data[i]); i += 4;
			const uint8_t *pub_key_prefix = &data[i]; i += 6;

			// Copy text and ensure null termination
			size_t text_len = len - 13;  // Everything after header (13 bytes)
			char text_buf[MAX_TEXT_LEN + 1];
			if (text_len > MAX_TEXT_LEN) text_len = MAX_TEXT_LEN;
			memcpy(text_buf, &data[i], text_len);
			text_buf[text_len] = '\0';
			const char *text = text_buf;

			LOG_INF("CMD_SEND_TXT_MSG: txt_type=%d attempt=%d pubkey=%02x%02x%02x%02x%02x%02x",
				txt_type, attempt, pub_key_prefix[0], pub_key_prefix[1], pub_key_prefix[2],
				pub_key_prefix[3], pub_key_prefix[4], pub_key_prefix[5]);

			ContactInfo *contact = lookupContactByPubKey(pub_key_prefix, 6);
			if (contact && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_CLI_DATA)) {
				LOG_INF("CMD_SEND_TXT_MSG: contact='%s' text='%s' text_len=%u", contact->name, text, (unsigned)text_len);

				uint32_t expected_ack = 0, est_timeout;
				int result;

				if (txt_type == TXT_TYPE_CLI_DATA) {
					// Use node's RTC instead of app timestamp
					msg_timestamp = getRTCClock()->getCurrentTimeUnique();
					result = sendCommandData(*contact, msg_timestamp, attempt, text, est_timeout);
				} else {
					result = sendMessage(*contact, msg_timestamp, attempt, text, expected_ack, est_timeout);
				}

				LOG_INF("CMD_SEND_TXT_MSG: sendMessage result=%d expected_ack=0x%08x", result, expected_ack);

				if (result != MSG_SEND_FAILED) {
					if (expected_ack) {
						// Track ACK
						int idx = -1;
						ContactInfo ci;
						for (int k = 0; k < getNumContacts(); k++) {
							if (getContactByIdx(k, ci) && ci.id.matches(contact->id)) {
								idx = k;
								break;
							}
						}
						addPendingAck(expected_ack, idx);
					}

					// Response: RESP_CODE_SENT + is_flood(1) + expected_ack(4) + est_timeout(4)
					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
					put_le32(&rsp[2], expected_ack);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_TABLE_FULL);
				}
			} else {
				LOG_WRN("CMD_SEND_TXT_MSG: contact not found or unsupported txt_type=%d", txt_type);
				sendPacketError(contact == nullptr ? ERR_NOT_FOUND : ERR_UNSUPPORTED);
			}
			return true;
		}
		LOG_WRN("CMD_SEND_TXT_MSG: frame too short (len=%u, need 14)", (unsigned)len);
		sendPacketError(ERR_ILLEGAL_ARG);
		return true;

	case CMD_SEND_CHANNEL_TXT_MSG:
		// Arduino format: [cmd][txt_type][channel_idx][4-byte timestamp][text...]
		// Minimum: 1 + 1 + 1 + 4 + 1 = 8 bytes
		if (len >= 8) {
			int i = 1;
			uint8_t txt_type = data[i++];
			uint8_t ch_idx = data[i++];
			uint32_t msg_timestamp = get_le32(&data[i]);
			i += 4;
			const char *text = (const char *)&data[i];
			size_t text_len = len - i;

			if (txt_type != TXT_TYPE_PLAIN) {
				sendPacketError(ERR_UNSUPPORTED);
			} else {
				ChannelDetails ch;
				if (getChannel(ch_idx, ch)) {
					if (sendGroupMessage(msg_timestamp, ch.channel, prefs.node_name, text, text_len)) {
						sendPacketOk();
					} else {
						sendPacketError(ERR_BAD_STATE);
					}
				} else {
					sendPacketError(ERR_NOT_FOUND);
				}
			}
			return true;
		}
		break;

	case CMD_SEND_SELF_ADVERT: {
		mesh::Packet *adv;
		if (prefs.advert_loc_policy == 0) {  /* ADVERT_LOC_NONE */
			adv = createSelfAdvert(prefs.node_name);
		} else {
			adv = createSelfAdvert(prefs.node_name, prefs.node_lat, prefs.node_lon);
		}
		if (adv) {
			/* Optional param: data[1] == 1 means flood, else zero-hop */
			if (len >= 2 && data[1] == 1) {
				sendFlood(adv, (uint32_t)0, prefs.path_hash_mode + 1);
			} else {
				sendZeroHop(adv);
			}
			sendPacketOk();
		} else {
			sendPacketError(ERR_TABLE_FULL);
		}
		return true;
	}

	case CMD_SYNC_NEXT_MESSAGE: {
		LOG_INF("CMD_SYNC_NEXT_MESSAGE: queue_count=%d pending=%d",
			_offline_queue_count, _sync_pending);

		/* Phone asking for next = implicit ACK for the previously-peeked message */
		if (_sync_pending) {
			confirmOfflineMessage();
			_sync_pending = false;
		}

		uint8_t buf[MAX_FRAME_SIZE];
		size_t msg_len;
		if (peekOfflineMessage(buf, msg_len)) {
			LOG_INF("CMD_SYNC_NEXT_MESSAGE: peeked msg_len=%u type=0x%02x", (unsigned)msg_len, buf[0]);
			writeFrame(buf, msg_len);
			_sync_pending = true;  /* will be confirmed on next request or lost on disconnect */
		} else {
			LOG_INF("CMD_SYNC_NEXT_MESSAGE: queue empty, sending NO_MORE_MSGS");
			uint8_t rsp[] = { PACKET_NO_MORE_MSGS };
			writeFrame(rsp, sizeof(rsp));

			/* Initial sync is done — safe to apply deferred
			 * connection parameters now without disrupting
			 * channel/contact/message throughput. */
			zephcore_ble_conn_params_ready();
		}
		return true;
	}

	case CMD_SET_ADVERT_NAME:
		if (len >= 2) {
			size_t nlen = len - 1;
			if (nlen >= sizeof(prefs.node_name)) nlen = sizeof(prefs.node_name) - 1;
			memcpy(prefs.node_name, &data[1], nlen);
			prefs.node_name[nlen] = '\0';
			_store->savePrefs(prefs);
		}
		sendPacketOk();
		return true;

	case CMD_SET_RADIO_PARAMS:
		if (len >= 11) {
			uint32_t freq = get_le32(&data[1]);
			uint32_t bw = get_le32(&data[5]);
			uint8_t sf = data[9];
			uint8_t cr = data[10];
			uint8_t repeat = 0;
			if (len > 11) {
				repeat = data[11];  // v9+: client repeat flag
			}
			// If repeat requested, validate frequency against allowed bands
			if (repeat && !isValidClientRepeatFreq(freq)) {
				sendPacketError(ERR_ILLEGAL_ARG);
			} else if (freq >= 300000 && freq <= 2500000 &&
			    bw >= 7000 && bw <= 500000 &&
			    sf >= 5 && sf <= 12 &&
			    cr >= 5 && cr <= 8) {
				prefs.freq = (float)freq / 1000.0f;
				prefs.bw = (float)bw / 1000.0f;
				prefs.sf = sf;
				prefs.cr = cr;
				prefs.client_repeat = repeat;
				_store->savePrefs(prefs);
				if (_radio_reconfig_cb) _radio_reconfig_cb();
				LOG_INF("SET_RADIO_PARAMS: client_repeat=%d", repeat);
				sendPacketOk();
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SET_RADIO_TX_POWER:
		if (len >= 2) {
			int8_t power = (int8_t)data[1];
			if (power >= -9 && power <= MAX_LORA_TX_POWER) {
				prefs.tx_power_dbm = power;
				_store->savePrefs(prefs);
				if (_radio_reconfig_cb) _radio_reconfig_cb();
				sendPacketOk();
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SET_ADVERT_LATLON:
		if (len >= 9) {
			int32_t lat = (int32_t)get_le32(&data[1]);
			int32_t lon = (int32_t)get_le32(&data[5]);
			// Arduino validation: lat ±90°, lon ±180° (in microdegrees)
			if (lat >= -90000000 && lat <= 90000000 &&
			    lon >= -180000000 && lon <= 180000000) {
				prefs.node_lat = (double)lat / 1000000.0;
				prefs.node_lon = (double)lon / 1000000.0;
				_store->savePrefs(prefs);
				LOG_INF("SET_ADVERT_LATLON: lat=%d lon=%d policy=%d",
					lat, lon, prefs.advert_loc_policy);
				sendPacketOk();
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_GET_DEVICE_TIME: {
		uint8_t rsp[5];
		rsp[0] = PACKET_CURR_TIME;
		put_le32(&rsp[1], (uint32_t)getRTCClock()->getCurrentTime());
		writeFrame(rsp, sizeof(rsp));
		return true;
	}

	case CMD_SET_DEVICE_TIME:
		if (len >= 5) {
			/* If we have GPS-synced time, ignore phone time sync attempts.
			 * GPS time is more accurate than phone time. Still return OK
			 * so the app doesn't keep retrying. */
			if (gps_has_time_sync()) {
				LOG_INF("Ignoring phone time sync - GPS time sync active");
				sendPacketOk();
				return true;
			}

			uint32_t secs = get_le32(&data[1]);
			uint32_t curr = getRTCClock()->getCurrentTime();
			// Arduino: only allow setting time forward (prevents time attacks)
			if (secs >= curr) {
				getRTCClock()->setCurrentTime(secs);
				sendPacketOk();
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_DEVICE_QUERY:
		if (len >= 2) {
			_app_target_ver = data[1];  // Which version of protocol does app understand
			#ifndef FIRMWARE_BUILD_DATE
			#define FIRMWARE_BUILD_DATE __DATE__
			#endif
			static const uint8_t fw_build[12] = FIRMWARE_BUILD_DATE;
			static const uint8_t model[40] = CONFIG_ZEPHCORE_BOARD_NAME;
			static const uint8_t version[20] = "v1.14.0-zephyr";
			uint8_t rsp[82];
			rsp[0] = PACKET_DEVICE_INFO;
			rsp[1] = 10;  // FIRMWARE_VER_CODE - v10 = path_hash_mode support
			rsp[2] = (MAX_CONTACTS / 2 > 255) ? 255 : (MAX_CONTACTS / 2);  // protocol byte, app multiplies by 2
			rsp[3] = MAX_GROUP_CHANNELS;
			put_le32(&rsp[4], prefs.ble_pin ? prefs.ble_pin : 123456);  // BLE PIN
			memcpy(&rsp[8], fw_build, 12);
			memcpy(&rsp[20], model, 40);
			memcpy(&rsp[60], version, 20);
			rsp[80] = prefs.client_repeat;  // v9+: offgrid mode state
			rsp[81] = prefs.path_hash_mode;  // v10+: path hash mode
			writeFrame(rsp, sizeof(rsp));
			return true;
		}
		break;

	case CMD_GET_TUNING_PARAMS: {
		uint8_t rsp[9];
		rsp[0] = PACKET_TUNING_PARAMS;
		put_le32(&rsp[1], (uint32_t)(prefs.rx_delay_base * 1000));
		put_le32(&rsp[5], (uint32_t)(prefs.airtime_factor * 1000));
		writeFrame(rsp, sizeof(rsp));
		return true;
	}

	case CMD_SET_TUNING_PARAMS:
		if (len >= 9) {
			prefs.rx_delay_base = (float)get_le32(&data[1]) / 1000.0f;
			prefs.airtime_factor = (float)get_le32(&data[5]) / 1000.0f;
			_store->savePrefs(prefs);
		}
		sendPacketOk();
		return true;

	case CMD_EXPORT_PRIVATE_KEY: {
		// Response: [PACKET_PRIVATE_KEY=0x0E][64 bytes: 32 private + 32 public]
		uint8_t rsp[65];
		rsp[0] = PACKET_PRIVATE_KEY;
		self_id.writeTo(&rsp[1], 64);
		writeFrame(rsp, sizeof(rsp));
		return true;
	}

	case CMD_FACTORY_RESET:
		// Arduino: requires "reset" magic string for safety
		if (len >= 6 && memcmp(&data[1], "reset", 5) == 0) {
			LOG_INF("Factory reset requested");
			_store->factoryReset();
			sendPacketOk();
			// Reboot to regenerate identity
			sys_reboot(SYS_REBOOT_COLD);
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_REBOOT:
		if (len >= 7 && memcmp(&data[1], "reboot", 6) == 0) {
			LOG_INF("Reboot requested");
			/* Flush any pending lazy writes before reboot */
			flushDirtyContacts();
			flushDirtyChannels();
			sendPacketOk();
			sys_reboot(SYS_REBOOT_COLD);
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SET_OTHER_PARAMS:
		if (len >= 2) {
			prefs.manual_add_contacts = data[1];
			if (len >= 3) {
				// Telemetry modes: (env << 4) | (loc << 2) | base
				prefs.telemetry_mode_base = data[2] & 0x03;
				prefs.telemetry_mode_loc = (data[2] >> 2) & 0x03;
				prefs.telemetry_mode_env = (data[2] >> 4) & 0x03;
				if (len >= 4) {
					prefs.advert_loc_policy = data[3];
					LOG_INF("SET_OTHER_PARAMS: advert_loc_policy=%d", prefs.advert_loc_policy);
					if (len >= 5) {
						prefs.multi_acks = data[4];
					}
				}
			}
			_store->savePrefs(prefs);
		}
		sendPacketOk();
		return true;

	case CMD_SET_DEVICE_PIN:
		if (len >= 5) {
			uint32_t pin;
			memcpy(&pin, &data[1], 4);
			if (pin == 0 || (pin >= 100000 && pin <= 999999)) {
				prefs.ble_pin = pin;
				_store->savePrefs(prefs);
				if (_pin_change_cb) _pin_change_cb(pin);
				sendPacketOk();
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SIGN_START:
		// Free any previous signing state
		if (_sign_data) {
			delete[] _sign_data;
			_sign_data = nullptr;
		}
		_sign_data_len = 0;
		_sign_data_capacity = 0;
		// Allocate new buffer
		_sign_data = new uint8_t[MAX_SIGN_DATA_LEN];
		if (_sign_data) {
			_sign_data_capacity = MAX_SIGN_DATA_LEN;
			// Response: [code][reserved][4-byte max_len] = 6 bytes (matches Arduino)
			uint8_t rsp[6];
			rsp[0] = PACKET_SIGN_START;
			rsp[1] = 0;  // reserved
			uint32_t max_len = MAX_SIGN_DATA_LEN;
			put_le32(&rsp[2], max_len);
			writeFrame(rsp, sizeof(rsp));
		} else {
			sendPacketError(ERR_BAD_STATE);
		}
		return true;

	case CMD_SIGN_DATA:
		if (_sign_data && len > 1) {
			size_t chunk_len = len - 1;
			if (_sign_data_len + chunk_len <= _sign_data_capacity) {
				memcpy(&_sign_data[_sign_data_len], &data[1], chunk_len);
				_sign_data_len += chunk_len;
				sendPacketOk();
			} else {
				sendPacketError(ERR_TABLE_FULL);
			}
		} else {
			sendPacketError(ERR_BAD_STATE);
		}
		return true;

	case CMD_SIGN_FINISH:
		if (_sign_data && _sign_data_len > 0) {
			uint8_t signature[64];
			// Sign the data using Ed25519 via LocalIdentity method
			self_id.sign(signature, _sign_data, _sign_data_len);
			uint8_t rsp[1 + 64];
			rsp[0] = PACKET_SIGNATURE;
			memcpy(&rsp[1], signature, 64);
			writeFrame(rsp, sizeof(rsp));
			// Clean up
			delete[] _sign_data;
			_sign_data = nullptr;
			_sign_data_len = 0;
			_sign_data_capacity = 0;
		} else {
			sendPacketError(ERR_BAD_STATE);
		}
		return true;

	case CMD_SEND_TRACE_PATH:
		/* Trace path: [cmd][4-byte tag][4-byte auth][flags][path...] */
		if (len > 10 && (len - 10) < MAX_PACKET_PAYLOAD - 5) {
			uint8_t path_len = len - 10;
			uint8_t flags = data[9];
			uint8_t path_sz = flags & 0x03;  /* Path hash size encoding */
			/* Validate path length is multiple of hash size */
			if ((path_len >> path_sz) > MAX_PATH_SIZE || (path_len % (1 << path_sz)) != 0) {
				sendPacketError(ERR_ILLEGAL_ARG);
			} else {
				uint32_t tag = get_le32(&data[1]);
				uint32_t auth_code = get_le32(&data[5]);
				mesh::Packet *trace = createTrace(tag, auth_code, flags);
				if (trace) {
					sendDirect(trace, &data[10], path_len);

					uint32_t airtime = _radio->getEstAirtimeFor(trace->payload_len + trace->path_len + 2);
					uint32_t est_timeout = calcDirectTimeoutMillisFor(airtime, path_len >> path_sz);

					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = 0;  /* Not flood */
					put_le32(&rsp[2], tag);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_TABLE_FULL);
				}
			}
		} else if (len >= 9) {
			/* Legacy: zero-hop trace (backwards compat) */
			uint32_t tag = get_le32(&data[1]);
			uint32_t auth_code = get_le32(&data[5]);
			mesh::Packet *trace = createTrace(tag, auth_code);
			if (trace) {
				sendZeroHop(trace);
				sendPacketOk();
			} else {
				sendPacketError(ERR_BAD_STATE);
			}
		} else {
			sendPacketOk();  /* Backwards compat */
		}
		return true;

	case CMD_IMPORT_PRIVATE_KEY:
		/* Import private key: [cmd][64 bytes prv_key] */
		if (len >= 1 + PRV_KEY_SIZE) {
			if (!mesh::LocalIdentity::validatePrivateKey(&data[1])) {
				sendPacketError(ERR_ILLEGAL_ARG); /* invalid key */
			} else {
				mesh::LocalIdentity new_identity;
				new_identity.readFrom(&data[1], PRV_KEY_SIZE);
				if (_store->saveMainIdentity(new_identity)) {
					self_id = new_identity;
					/* Reload contacts to invalidate ECDH shared secrets */
					resetContacts();
					_store->loadContacts(this);
					sendPacketOk();
				} else {
					sendPacketError(ERR_BAD_STATE);
				}
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_RAW_DATA:
		/* Raw data packet: [cmd][path_len][path...][payload...] */
		if (len >= 6) {  /* min: cmd + path_len + 4 byte payload */
			int i = 1;
			int8_t path_len = (int8_t)data[i++];
			if (path_len >= 0 && i + path_len + 4 <= (int)len) {
				const uint8_t *path = &data[i];
				i += path_len;
				mesh::Packet *pkt = createRawData(&data[i], len - i);
				if (pkt) {
					sendDirect(pkt, path, path_len);
					sendPacketOk();
				} else {
					sendPacketError(ERR_TABLE_FULL);
				}
			} else {
				/* Flood mode not supported (path_len < 0) */
				sendPacketError(ERR_UNSUPPORTED);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_LOGIN:
		if (len >= 1 + PUB_KEY_SIZE) {
			ContactInfo *contact = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			size_t pw_len = len - 1 - PUB_KEY_SIZE;
			char pw_buf[64];
			if (pw_len >= sizeof(pw_buf)) pw_len = sizeof(pw_buf) - 1;
			memcpy(pw_buf, &data[1 + PUB_KEY_SIZE], pw_len);
			pw_buf[pw_len] = '\0';
			const char *password = pw_buf;
			if (contact) {
				uint32_t est_timeout;
				LOG_INF("CMD_SEND_LOGIN: sending to contact, path_len=%d", contact->out_path_len);
				int result = sendLogin(*contact, password, est_timeout);
				LOG_INF("CMD_SEND_LOGIN: sendLogin returned %d, est_timeout=%u", result, est_timeout);
				if (result != MSG_SEND_FAILED) {
					clearPendingReqs();
					memcpy(&_pending_login, contact->id.pub_key, 4);  // match in onContactResponse()
					LOG_INF("CMD_SEND_LOGIN: _pending_login set to %08x", _pending_login);
					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
					memcpy(&rsp[2], &_pending_login, 4);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_TABLE_FULL);
				}
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_STATUS_REQ:
		if (len >= 1 + PUB_KEY_SIZE) {
			ContactInfo *contact = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (contact) {
				uint32_t tag, est_timeout;
				int result = sendRequest(*contact, REQ_TYPE_GET_STATUS, tag, est_timeout);
				if (result != MSG_SEND_FAILED) {
					clearPendingReqs();
					memcpy(&_pending_status, contact->id.pub_key, 4);  // legacy matching scheme
					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
					put_le32(&rsp[2], tag);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_BAD_STATE);
				}
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_HAS_CONNECTION:
		if (len >= 1 + PUB_KEY_SIZE) {
			if (hasConnectionTo(&data[1])) {
				sendPacketOk();
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_LOGOUT:
		if (len >= 1 + PUB_KEY_SIZE) {
			stopConnection(&data[1]);
		}
		sendPacketOk();
		return true;

	case CMD_SEND_TELEMETRY_REQ:
		if (len == 4) {
			// Self-telemetry request: return battery, GPS, and environment data
			// Format: Cayenne LPP: [channel][type][data...]
			// Response: [PUSH_CODE_TELEMETRY_RESPONSE][reserved][6-byte pubkey][telemetry_data]
			uint8_t rsp[96];  // header(8)+batt(4)+gps(11)+env(11)+pwr(36)=70 worst case
			int i = 0;
			rsp[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
			rsp[i++] = 0;  // reserved
			memcpy(&rsp[i], self_id.pub_key, 6);
			i += 6;  // pubkey prefix

			// Channel 1 = TELEM_CHANNEL_SELF
			const uint8_t CH_SELF = 1;

			// Battery voltage: [channel][LPP_VOLTAGE=116][2-byte 0.01V big-endian]
			uint16_t batt_mv = _batt_cb ? _batt_cb() : 0;
			rsp[i++] = CH_SELF;
			rsp[i++] = 116;  // LPP_VOLTAGE
			uint16_t batt_scaled = batt_mv / 10;  // 0.01V resolution
			rsp[i++] = (batt_scaled >> 8) & 0xFF;  // Big-endian
			rsp[i++] = batt_scaled & 0xFF;

			// GPS position if available: [channel][LPP_GPS=136][3-byte lat][3-byte lon][3-byte alt]
			// Use last known position even when GPS is sleeping (power-save mode)
			struct gps_position pos;
			if (gps_is_available() && gps_get_last_known_position(&pos)) {
				rsp[i++] = CH_SELF;
				rsp[i++] = 136;  // LPP_GPS
				// Lat/lon in 0.0001 degrees (signed 24-bit)
				int32_t lat = (int32_t)(pos.latitude_ndeg / 100000);  // nano-degrees to 0.0001 degrees
				int32_t lon = (int32_t)(pos.longitude_ndeg / 100000);
				int32_t alt = pos.altitude_mm / 10;  // mm to 0.01m
				rsp[i++] = (lat >> 16) & 0xFF;
				rsp[i++] = (lat >> 8) & 0xFF;
				rsp[i++] = lat & 0xFF;
				rsp[i++] = (lon >> 16) & 0xFF;
				rsp[i++] = (lon >> 8) & 0xFF;
				rsp[i++] = lon & 0xFF;
				rsp[i++] = (alt >> 16) & 0xFF;
				rsp[i++] = (alt >> 8) & 0xFF;
				rsp[i++] = alt & 0xFF;
			} else if (prefs.node_lat != 0 || prefs.node_lon != 0) {
				// Use configured position
				rsp[i++] = CH_SELF;
				rsp[i++] = 136;  // LPP_GPS
				int32_t lat = (int32_t)(prefs.node_lat * 10000);
				int32_t lon = (int32_t)(prefs.node_lon * 10000);
				int32_t alt = 0;
				rsp[i++] = (lat >> 16) & 0xFF;
				rsp[i++] = (lat >> 8) & 0xFF;
				rsp[i++] = lat & 0xFF;
				rsp[i++] = (lon >> 16) & 0xFF;
				rsp[i++] = (lon >> 8) & 0xFF;
				rsp[i++] = lon & 0xFF;
				rsp[i++] = (alt >> 16) & 0xFF;
				rsp[i++] = (alt >> 8) & 0xFF;
				rsp[i++] = alt & 0xFF;
			}

			// Environment sensors
			{
				struct env_data env;
				if (env_sensors_read(&env) == 0) {
					// Temperature: [channel][LPP_TEMPERATURE=103][2-byte 0.1C signed big-endian]
					if (env.has_temperature) {
						rsp[i++] = CH_SELF;
						rsp[i++] = LPP_TEMPERATURE;
						int16_t temp = (int16_t)(env.temperature_c * 10);
						rsp[i++] = (temp >> 8) & 0xFF;
						rsp[i++] = temp & 0xFF;
					} else if (env.has_mcu_temperature) {
						// MCU die temp as fallback when no external sensor
						rsp[i++] = CH_SELF;
						rsp[i++] = LPP_TEMPERATURE;
						int16_t temp = (int16_t)(env.mcu_temperature_c * 10);
						rsp[i++] = (temp >> 8) & 0xFF;
						rsp[i++] = temp & 0xFF;
					}
					// Humidity: [channel][LPP_RELATIVE_HUMIDITY=104][1-byte 0.5%]
					if (env.has_humidity) {
						rsp[i++] = CH_SELF;
						rsp[i++] = LPP_RELATIVE_HUMIDITY;
						rsp[i++] = (uint8_t)(env.humidity_pct * 2);
					}
					// Pressure: [channel][LPP_BAROMETRIC_PRESSURE=115][2-byte 0.1hPa big-endian]
					if (env.has_pressure) {
						rsp[i++] = CH_SELF;
						rsp[i++] = LPP_BAROMETRIC_PRESSURE;
						uint16_t press = (uint16_t)(env.pressure_hpa * 10);
						rsp[i++] = (press >> 8) & 0xFF;
						rsp[i++] = press & 0xFF;
					}
				}
			}

			// Power monitor telemetry (INA219/INA3221/ina2xx)
			if (power_sensors_available()) {
				struct power_data pwr;
				if (power_sensors_read(&pwr) == 0) {
					uint8_t ch = CH_SELF + 1;
					for (int j = 0; j < pwr.num_channels; j++) {
						if (pwr.channels[j].valid) {
							// Voltage: [ch][LPP_VOLTAGE=116][2-byte 0.01V]
							rsp[i++] = ch;
							rsp[i++] = 116;
							uint16_t v = (uint16_t)(pwr.channels[j].voltage_v * 100);
							rsp[i++] = (v >> 8) & 0xFF;
							rsp[i++] = v & 0xFF;
							// Current: [ch][LPP_CURRENT=117][2-byte 0.001A]
							rsp[i++] = ch;
							rsp[i++] = 117;
							uint16_t c = (uint16_t)(pwr.channels[j].current_a * 1000);
							rsp[i++] = (c >> 8) & 0xFF;
							rsp[i++] = c & 0xFF;
							// Power: [ch][LPP_POWER=128][2-byte 1W]
							rsp[i++] = ch;
							rsp[i++] = 128;
							uint16_t p = (uint16_t)(pwr.channels[j].power_w);
							rsp[i++] = (p >> 8) & 0xFF;
							rsp[i++] = p & 0xFF;
							ch++;
						}
					}
				}
			}

			writeFrame(rsp, i);

			// Trigger GPS wake so next telemetry request has fresh location
			if (gps_is_available() && gps_is_enabled()) {
				gps_request_fresh_fix();
			}
		} else if (len >= 4 + PUB_KEY_SIZE) {
			// Contact telemetry request: [cmd][3 reserved bytes][32-byte pubkey]
			ContactInfo *contact = lookupContactByPubKey(&data[4], PUB_KEY_SIZE);
			if (contact) {
				uint32_t tag, est_timeout;
				int result = sendRequest(*contact, REQ_TYPE_GET_TELEMETRY_DATA, tag, est_timeout);
				if (result != MSG_SEND_FAILED) {
					clearPendingReqs();
					_pending_telemetry = tag;
					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
					put_le32(&rsp[2], tag);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_BAD_STATE);
				}
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_BINARY_REQ:
		// Format: [cmd][32-byte pubkey][req_data...]
		if (len >= 1 + PUB_KEY_SIZE + 1) {  // Need at least cmd + pubkey + 1 byte req_data
			ContactInfo *contact = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (contact) {
				uint32_t tag, est_timeout;
				// req_data starts after pubkey
				const uint8_t *req_data = &data[1 + PUB_KEY_SIZE];
				size_t req_len = len - (1 + PUB_KEY_SIZE);
				int result = sendRequest(*contact, req_data, req_len, tag, est_timeout);
				if (result != MSG_SEND_FAILED) {
					clearPendingReqs();
					_pending_req = tag;
					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
					put_le32(&rsp[2], tag);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_BAD_STATE);
				}
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_PATH_DISCOVERY:
		if (len >= 2 + PUB_KEY_SIZE && data[1] == 0) {
			// Path Discovery is a special flood + Telemetry request
			ContactInfo *contact = lookupContactByPubKey(&data[2], PUB_KEY_SIZE);
			if (contact) {
				uint32_t tag, est_timeout;
				// Build telemetry request with only BASE permissions
				uint8_t req_data[9];
				req_data[0] = REQ_TYPE_GET_TELEMETRY_DATA;
				req_data[1] = ~(TELEM_PERM_BASE);  // inverse permissions mask
				memset(&req_data[2], 0, 3);  // reserved
				getRNG()->random(&req_data[5], 4);  // random to make packet-hash unique

				// Temporarily force flood
				auto save = contact->out_path_len;
				contact->out_path_len = OUT_PATH_UNKNOWN;
				int result = sendRequest(*contact, req_data, sizeof(req_data), tag, est_timeout);
				contact->out_path_len = save;

				if (result != MSG_SEND_FAILED) {
					clearPendingReqs();
					_pending_discovery = tag;
					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
					put_le32(&rsp[2], tag);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_BAD_STATE);
				}
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_GET_ADVERT_PATH:
		// Format: [cmd][reserved][7-byte pubkey prefix]
		if (len >= 2 + 7) {
			const AdvertPath *ap = findAdvertPath(&data[2], 7);  // pubkey at offset 2
			if (ap) {
				// Response: [code][4-byte timestamp][path_len][path...]
				uint8_t rsp[6 + MAX_PATH_SIZE];
				size_t i = 0;
				rsp[i++] = PACKET_ADVERT_PATH;
				put_le32(&rsp[i], ap->recv_timestamp); i += 4;
				rsp[i++] = ap->path_len;
				i += mesh::Packet::writePath(&rsp[i], ap->path, ap->path_len);
				writeFrame(rsp, i);
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_GET_CUSTOM_VARS: {
		// Return GPS and sensor settings as comma-separated key:value pairs
		// Format: [PACKET_CUSTOM_VARS][key1:val1,key2:val2,...]
		uint8_t rsp[64];
		char *dp = (char *)&rsp[1];
		rsp[0] = PACKET_CUSTOM_VARS;

		// GPS settings
		bool first = true;
		if (gps_is_available()) {
			dp += snprintf(dp, 20, "gps:%d", gps_is_enabled() ? 1 : 0);
			first = false;
		}
		uint32_t gps_interval = gps_get_poll_interval_sec();
		if (gps_interval > 0) {
			if (!first) *dp++ = ',';
			dp += snprintf(dp, 20, "gps_interval:%u", gps_interval);
			first = false;
		}
		// Note: Environment sensors are auto-detected, no settings needed

		writeFrame(rsp, (size_t)(dp - (char *)rsp));
		return true;
	}

	case CMD_SET_CUSTOM_VAR:
		// Format: [cmd][key:value]
		if (len >= 4) {
			char buf[64];
			size_t copy_len = (len - 1 < sizeof(buf) - 1) ? len - 1 : sizeof(buf) - 1;
			memcpy(buf, &data[1], copy_len);
			buf[copy_len] = '\0';

			char *sep = strchr(buf, ':');
			if (sep) {
				*sep = '\0';
				char *key = buf;
				char *val = sep + 1;

				if (strcmp(key, "gps") == 0) {
					bool enable = (val[0] == '1');
					gps_enable(enable);
					/* Persist GPS state across reboots */
					prefs.gps_enabled = enable ? 1 : 0;
					_store->savePrefs(prefs);
					sendPacketOk();
				} else if (strcmp(key, "gps_interval") == 0) {
					uint32_t interval = (uint32_t)atoi(val);
					if (interval > 0 && interval <= 86400) {
						gps_set_poll_interval_sec(interval);
						/* Persist GPS interval across reboots */
						prefs.gps_interval = interval;
						_store->savePrefs(prefs);
						sendPacketOk();
					} else {
						sendPacketError(ERR_ILLEGAL_ARG);
					}
				} else {
					sendPacketError(ERR_ILLEGAL_ARG);
				}
			} else {
				sendPacketError(ERR_ILLEGAL_ARG);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SET_FLOOD_SCOPE:
		/* Flood scope: [cmd][0][16-byte key] or [cmd][0] (null key) */
		if (len >= 2 && data[1] == 0) {
			if (len >= 2 + 16) {
				memcpy(_send_scope.key, &data[2], sizeof(_send_scope.key));
			} else {
				memset(_send_scope.key, 0, sizeof(_send_scope.key));
			}
			sendPacketOk();
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_CONTROL_DATA:
		/* Control data: [cmd][flags | 0x80][data...] */
		if (len >= 2 && (data[1] & 0x80) != 0) {
			mesh::Packet *pkt = createControlData(&data[1], len - 1);
			if (pkt) {
				sendZeroHop(pkt);
				sendPacketOk();
			} else {
				sendPacketError(ERR_TABLE_FULL);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SET_AUTOADD_CONFIG:
		if (len >= 2) {
			prefs.autoadd_config = data[1];
			if (len >= 3) {
				prefs.autoadd_max_hops = (data[2] > 64) ? 64 : data[2];
			}
			LOG_INF("SET_AUTOADD_CONFIG: autoadd_config=0x%02x max_hops=%u",
				prefs.autoadd_config, prefs.autoadd_max_hops);
			_store->savePrefs(prefs);
		}
		sendPacketOk();
		return true;

	case CMD_GET_AUTOADD_CONFIG: {
		uint8_t rsp[3];
		rsp[0] = PACKET_AUTOADD_CONFIG;
		rsp[1] = prefs.autoadd_config;
		rsp[2] = prefs.autoadd_max_hops;
		writeFrame(rsp, sizeof(rsp));
		return true;
	}

	case CMD_GET_ALLOWED_REPEAT_FREQ: {
		uint8_t rsp[1 + sizeof(repeat_freq_ranges)];
		int i = 0;
		rsp[i++] = PACKET_ALLOWED_REPEAT_FREQ;
		for (size_t k = 0; k < sizeof(repeat_freq_ranges) / sizeof(repeat_freq_ranges[0]); k++) {
			put_le32(&rsp[i], repeat_freq_ranges[k].lower_freq); i += 4;
			put_le32(&rsp[i], repeat_freq_ranges[k].upper_freq); i += 4;
		}
		writeFrame(rsp, i);
		return true;
	}

	case CMD_SET_PATH_HASH_MODE:
		if (len >= 3 && data[1] == 0) {
			if (data[2] >= 3) {
				sendPacketError(ERR_ILLEGAL_ARG);
			} else {
				prefs.path_hash_mode = data[2];
				_store->savePrefs(prefs);
				sendPacketOk();
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	case CMD_SEND_ANON_REQ:
		if (len >= 1 + PUB_KEY_SIZE + 1) {
			ContactInfo *contact = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			if (contact) {
				uint32_t tag, est_timeout;
				int result = sendAnonReq(*contact, &data[1 + PUB_KEY_SIZE], len - 1 - PUB_KEY_SIZE, tag, est_timeout);
				if (result != MSG_SEND_FAILED) {
					clearPendingReqs();
					_pending_req = tag;
					uint8_t rsp[10];
					rsp[0] = PACKET_SENT;
					rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;  // is_flood
					put_le32(&rsp[2], tag);
					put_le32(&rsp[6], est_timeout);
					writeFrame(rsp, sizeof(rsp));
				} else {
					sendPacketError(ERR_BAD_STATE);
				}
			} else {
				sendPacketError(ERR_NOT_FOUND);
			}
		} else {
			sendPacketError(ERR_ILLEGAL_ARG);
		}
		return true;

	default:
		break;
	}

	return false;
}
