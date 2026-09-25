/*
 * SPDX-License-Identifier: MIT
 * CompanionMesh implementation
 */

#include "CompanionMesh.h"
#include <mesh/Utils.h>
#include <helpers/TxtDataHelpers.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <ZephyrSensorManager.h>
#include <adapters/sensors/SimpleLPP.h>
#if IS_ENABLED(CONFIG_BT)
#include <adapters/ble/ZephyrBLE.h>
#endif
#include <adapters/gps/ZephyrGPSManager.h>
#include <helpers/time_sync.h>
#include <adapters/clock/ZephyrRTCDiscover.h>
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_BUTTON) || IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
#include <ui_task.h>
#define ZEPHCORE_HAS_UI_TASK 1
#endif
#include <joystick_ui_hooks.h>
#include <helpers/PacketLog.h>
LOG_MODULE_REGISTER(zephcore_companion, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#include "CompanionProtocol.h"

/* UI notification helper => path_len extraction + uniform call through ui_task.h */
#if ZEPHCORE_HAS_UI_TASK
static void notify_contact_msg_ui(
	const ContactInfo &contact, mesh::Packet *pkt, const char *text, int queue_count
){
	uint8_t path_len = (pkt && pkt->isRouteFlood()) ? pkt->path_len : OUT_PATH_UNKNOWN;
	ui_notify_contact_msg(path_len, contact.name, text, (uint16_t)queue_count);
}
#endif

/* Allowed client repeat frequency ranges (matches Arduino) */
struct FreqRange {
	uint32_t lower_freq, upper_freq;
};

static const FreqRange repeat_freq_ranges[] = {
	{ 433000, 433000 },   /* 433 MHz (ISM band) */
	{ 869495, 869495 },   /* 869.495 MHz (EU 869.4-869.65 band, 10% DC, 500mW ERP) */
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
	_serial = nullptr;
	_batt_cb = nullptr;
	_radio_reconfig_cb = nullptr;
	_pin_change_cb = nullptr;
	_contact_iter_active = false;
	_contact_iter_idx = 0;
	_contact_iter_num = 0;
	_contact_iter_vc = false;
	_contact_iter_lastmod = 0;
	_contact_iter_since = 0;
	_offline_queue_head = 0;
	_offline_queue_tail = 0;
	_offline_queue_count = 0;
	_sync_pending = false;
	memset(_ack_table, 0, sizeof(_ack_table));
	memset(_advert_paths, 0, sizeof(_advert_paths));
	_sign_data = nullptr;
	_sign_data_len = 0;
	_sign_data_capacity = 0;
	_pending_login = 0;
	_pending_status = 0;
	_pending_telemetry = 0;
	_pending_discovery = 0;
	_pending_req = 0;
#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
	_pending_joystick_ping_tag = 0;
	_pending_joystick_admin_tag = 0;
#endif
	_pending_channel_head = 0;
	_pending_channel_tail = 0;
	_pending_channel_count = 0;
	memset(_pending_channel_idx, 0, sizeof(_pending_channel_idx));
	_app_target_ver = 0;
	_dirty_contacts_expiry = 0;
	_dirty_channels_expiry = 0;
	memset(_send_scope.key, 0, sizeof(_send_scope.key));
	_send_scope_force_unscoped = false;
	_cli_exec_cb = nullptr;
	memset(_vcontact_pubkey, 0, sizeof(_vcontact_pubkey));
	_vcontact_lastmod = 0;
	memset(_vcontact_recent_ts, 0, sizeof(_vcontact_recent_ts));
	_vcontact_recent_head = 0;
	memset(_vcontact_pending, 0, sizeof(_vcontact_pending));
	_vcontact_pending_count = 0;
	_vcontact_hold_msgwait = false;
	_vcontact_hold_expiry = 0;
	_last_msgwait_ms = 0;
	_last_sync_req_ms = 0;
	_vcontact_app_hidden = false;
	_vcontact_confirm_ack = 0;
	_vcontact_confirm_work.self = this;
	k_work_init_delayable(&_vcontact_confirm_work.work, vcontactConfirmWorkHandler);
	memset(&prefs, 0, sizeof(prefs));
	prefs.node_lat = 0;
	prefs.node_lon = 0;
}

void CompanionMesh::begin()
{
	BaseChatMesh::begin();

	/* Derive the v-contact pubkey from our identity: stable per node, unique
	 * per device. A real Ed25519 point (strict clients reject anything else),
	 * but its private half is derived-and-dropped — never stored, never used —
	 * and only this node can recompute it. */
	deriveVContactKey();
	/* Stamp lastmod only if a time source already ran (hardware RTC restore
	 * happens before begin()). Otherwise stay deferred (lastmod = 0) until
	 * vcontactClockSynced() — an advert stamped now would show as 1970. */
	if (vcontactClockValid()) {
		_vcontact_lastmod = (uint32_t)getRTCClock()->getCurrentTime();
	}
}

bool CompanionMesh::allowPacketForward(const mesh::Packet *packet)
{
	(void)packet;
	return prefs.client_repeat != 0;
}

void CompanionMesh::sendFloodScoped(const TransportKey &scope, mesh::Packet *pkt, uint32_t delay_millis)
{
	if (scope.isNull()) {
		sendFlood(pkt, delay_millis, prefs.path_hash_mode + 1);
	} else {
		uint16_t codes[2];
		codes[0] = scope.calcTransportCode(pkt);
		codes[1] = 0;
		sendFlood(pkt, codes, delay_millis, prefs.path_hash_mode + 1);
	}
}

void CompanionMesh::sendFloodScopedDefault(mesh::Packet *pkt, uint32_t delay_millis)
{
	if (_send_scope_force_unscoped) {
		TransportKey no_scope;
		memset(no_scope.key, 0, sizeof(no_scope.key));
		sendFloodScoped(no_scope, pkt, delay_millis);
		return;
	}

	TransportKey default_scope;
	memcpy(default_scope.key, prefs.default_scope_key, sizeof(default_scope.key));

	const TransportKey &scope = _send_scope.isNull() ? default_scope : _send_scope;
	sendFloodScoped(scope, pkt, delay_millis);
}

void CompanionMesh::sendFloodScoped(const ContactInfo &recipient, mesh::Packet *pkt, uint32_t delay_millis)
{
	/* TODO: dynamic _send_scope, depending on recipient and current 'home' Region */
	(void)recipient;
	sendFloodScopedDefault(pkt, delay_millis);
}

void CompanionMesh::sendFloodScoped(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t delay_millis)
{
	/* TODO: have per-channel send_scope */
	(void)channel;
	sendFloodScopedDefault(pkt, delay_millis);
}

bool CompanionMesh::sendSelfAdvert(bool flood)
{
	mesh::Packet *adv;
	if (prefs.advert_loc_policy == 0) {  /* ADVERT_LOC_NONE */
		adv = createSelfAdvert(prefs.node_name);
	} else {
		adv = createSelfAdvert(prefs.node_name, prefs.node_lat, prefs.node_lon);
	}
	if (!adv) {
		return false;
	}
	if (flood) {
		TransportKey default_scope;
		memcpy(default_scope.key, prefs.default_scope_key, sizeof(default_scope.key));
		sendFloodScoped(default_scope, adv, (uint32_t)0);
	} else {
		sendZeroHop(adv);
	}
	return true;
}

bool CompanionMesh::onContactPathRecv(ContactInfo &from, uint8_t *in_path, uint8_t in_path_len,
	uint8_t *out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len)
{
	if (extra_type == PAYLOAD_TYPE_RESPONSE && extra_len > 4) {
		uint32_t tag;
		memcpy(&tag, extra, 4);

		if (tag == _pending_discovery) {
			_pending_discovery = 0;

			if (mesh::Packet::isValidPathLen(in_path_len) && mesh::Packet::isValidPathLen(out_path_len)) {
				uint8_t rsp[172];
				int i = 0;
				rsp[i++] = PUSH_CODE_PATH_DISCOVERY_RESP;
				rsp[i++] = 0;  // reserved
				memcpy(&rsp[i], from.id.pub_key, 6);
				i += 6;
				rsp[i++] = out_path_len;
				i += mesh::Packet::writePath(&rsp[i], out_path, MAX_PATH_SIZE, out_path_len);
				rsp[i++] = in_path_len;
				i += mesh::Packet::writePath(&rsp[i], in_path, MAX_PATH_SIZE, in_path_len);
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
	drainPendingChannelInfos();

	/* Check for pending lazy contact/channel writes */
	int64_t now = _ms->getMillis();
	if (_dirty_contacts_expiry && now >= _dirty_contacts_expiry) {
		flushDirtyContacts();
	}
	if (_dirty_channels_expiry && now >= _dirty_channels_expiry) {
		flushDirtyChannels();
	}
}

void CompanionMesh::onAdvertTimeSample(const mesh::Identity &id, uint32_t timestamp,
				       uint8_t hops)
{
	/* Signature already verified by mesh::Mesh before this hook fires. */
	_timesync.onAdvertHeard(id.pub_key, timestamp, hops,
				(uint32_t)(k_uptime_get() / 1000));
}

void CompanionMesh::timeSyncTick()
{
	if (!prefs.meshtimesync) return;
	/* Shared policy (suppression/pedigree, forward-only) lives in runTick;
	 * no companion-side bookkeeping needs shifting on a step. */
	_timesync.runTick(*getRTCClock());
}

void CompanionMesh::markContactsDirty(bool substantive)
{
	int64_t deadline = _ms->getMillis() +
			   (substantive ? LAZY_WRITE_DELAY_MS
					: LAZY_WRITE_LIVENESS_MS);

	/* Only set the timer on first dirty — don't keep pushing
	 * the deadline forward or a busy mesh never flushes. */
	if (!_dirty_contacts_expiry) {
		_dirty_contacts_expiry = deadline;
		return;
	}

	/* ...but a substantive change must not have to sit out a liveness wait
	 * that is already pending.  Pulling the deadline IN cannot starve the
	 * flush, only hasten it. */
	if (substantive && deadline < _dirty_contacts_expiry) {
		_dirty_contacts_expiry = deadline;
	}
}

void CompanionMesh::markChannelsDirty()
{
	/* Channels import arrives as a burst of CMD_SET_CHANNEL writes.
	 * Keep pushing the flush deadline so we save once after the burst. */
	_dirty_channels_expiry = _ms->getMillis() + LAZY_WRITE_DELAY_MS;
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

void CompanionMesh::startInterface(BaseSerialInterface &serial)
{
	_serial = &serial;
	serial.enable();
}

bool CompanionMesh::writeFrame(const uint8_t *data, size_t len)
{
	LOG_DBG("RSP: 0x%02x len=%u", data[0], (unsigned)len);
	return _serial && _serial->writeFrame(data, len) == len;
}

void CompanionMesh::writeOKFrame()
{
	uint8_t rsp[] = { PACKET_OK };
	writeFrame(rsp, sizeof(rsp));
}

void CompanionMesh::writeErrFrame(uint8_t code)
{
	uint8_t rsp[] = { PACKET_ERROR, code };
	writeFrame(rsp, sizeof(rsp));
}

void CompanionMesh::sendPacketSent(uint8_t result, uint32_t tag, uint32_t est_timeout)
{
	uint8_t rsp[10];
	rsp[0] = PACKET_SENT;
	rsp[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
	put_le32(&rsp[2], tag);
	put_le32(&rsp[6], est_timeout);
	writeFrame(rsp, sizeof(rsp));
}

/* [push_code][data...], only while an app is listening (pushes are lossy) */
void CompanionMesh::sendPush(uint8_t code, const uint8_t *data, size_t len)
{
	if (!_serial || !_serial->isConnected()) {
		return;
	}

	uint8_t frame[MAX_FRAME_SIZE];

	if (1 + len > MAX_FRAME_SIZE) {
		LOG_WRN("push 0x%02x: data too long %u", code, (unsigned)len);
		len = MAX_FRAME_SIZE - 1;
	}
	frame[0] = code;
	if (len > 0 && data != nullptr) {
		memcpy(&frame[1], data, len);
	}
	LOG_DBG("sendPush: code=0x%02x len=%u", code, (unsigned)(1 + len));
	writeFrame(frame, 1 + len);
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
	// Channel messages have PACKET_CHANNEL_MSG_V3 (0x11) or PACKET_CHANNEL_MSG_RECV (0x08) or PACKET_CHANNEL_DATA_RECV (0x1B)
	return buf[0] == PACKET_CHANNEL_MSG_V3 || buf[0] == PACKET_CHANNEL_MSG_RECV || buf[0] == PACKET_CHANNEL_DATA_RECV;
}

bool CompanionMesh::enqueuePendingChannelInfo(uint8_t idx)
{
	if (_pending_channel_count >= MAX_GROUP_CHANNELS) {
		return false;
	}
	_pending_channel_idx[_pending_channel_tail] = idx;
	_pending_channel_tail = (uint8_t)((_pending_channel_tail + 1) % MAX_GROUP_CHANNELS);
	_pending_channel_count++;
	return true;
}

bool CompanionMesh::sendChannelInfoFrame(uint8_t idx)
{
	ChannelDetails ch;
	if (!getChannel(idx, ch)) {
		return false;
	}
	uint8_t rsp[50];
	int i = 0;
	rsp[i++] = PACKET_CHANNEL_INFO;
	rsp[i++] = idx;
	memcpy(&rsp[i], ch.name, 32);
	i += 32;
	memcpy(&rsp[i], ch.channel.secret, 16);
	i += 16;
	return writeFrame(rsp, i);
}

void CompanionMesh::drainPendingChannelInfos()
{
	while (_pending_channel_count > 0) {
		uint8_t idx = _pending_channel_idx[_pending_channel_head];
		if (!sendChannelInfoFrame(idx)) {
			return;
		}
		_pending_channel_head = (uint8_t)((_pending_channel_head + 1) % MAX_GROUP_CHANNELS);
		_pending_channel_count--;
	}
}

void CompanionMesh::addToOfflineQueue(const uint8_t *data, size_t len)
{
	LOG_DBG("addToOfflineQueue: len=%u type=0x%02x count_before=%d", (unsigned)len, data[0], _offline_queue_count);
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
	LOG_DBG("addToOfflineQueue: count_after=%d", _offline_queue_count);
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

/* How long CMD_APP_START may suppress v-contact notice prompts.
 *
 * This deadline is also the worst-case freeze a mid-session CMD_APP_START can
 * impose, because the app is under no obligation to follow one with the
 * message sync that clears the latch — so it must be short enough that nobody
 * notices, not merely short enough to recover eventually.
 *
 * It only has to cover the handoff between CMD_APP_START and the
 * CMD_GET_CONTACTS that normally follows it, which is where a prompt could
 * make the app fetch a message from a contact it has not been told about yet.
 * The dump itself is gated directly by _contact_iter_active below, for however
 * long it takes, and once the dump is done a prompt is harmless. Measured on
 * hardware, a full 350-contact initial sync runs APP_START to
 * PACKET_NO_MORE_MSGS in ~2.9 s, so the handoff is a small fraction of that.
 *
 * History, because the sizing is the whole bug: 60 s produced the ~54 s
 * "v-contact is frozen" reports. 10 s only looked fixed — hardware logs showed
 * the user's own settings-screen-to-message time landing at 10.5-11.7 s, so
 * the latch was expiring inline microseconds before the prompt and the feature
 * was winning a race by half a second. Do not raise this to "be safe"; raising
 * it re-creates the bug, and the thing it guards does not need the time. */
#define VCONTACT_HOLD_MAX_MS 3000

bool CompanionMesh::vcontactMsgWaitHeld()
{
	/* A live contact dump is the concrete hazard the hold exists for: a
	 * MSG_WAITING mid-dump makes the app interleave message-sync into the
	 * contact stream, which trips the "reset iterator on any other command"
	 * guard and truncates the sync. Checked directly rather than inferred from
	 * the latch, so the deadline below can be short without ever exposing a
	 * slow dump. */
	if (_contact_iter_active) {
		return true;
	}
	if (!_vcontact_hold_msgwait) {
		return false;
	}
	if (_vcontact_hold_expiry && _ms->getMillis() >= _vcontact_hold_expiry) {
		LOG_WRN("vcontact: msgwait hold expired without a completed sync");
		_vcontact_hold_msgwait = false;
		_vcontact_hold_expiry = 0;
		return false;
	}
	return true;
}

void CompanionMesh::pushMsgWaiting()
{
	_last_msgwait_ms = _ms->getMillis();
	LOG_INF("msgwait: prompting app, %d queued", _offline_queue_count);
	sendPush(PUSH_CODE_MSG_WAITING);
}

/* Re-prompt cadence.
 *
 * Observed on hardware: the app ignores the first MSG_WAITING after a settings
 * write (the GPS toggle) but honours a later identical one, so the reply is
 * recovered rather than lost — it just arrives one cadence late. At the
 * original 15 s that was a ~20 s wait with the 5 s housekeeping granularity on
 * top, which reads as broken even though nothing is. The prompt is a 1-byte
 * idempotent frame, so the cost of asking again sooner is nil, and the 5 s tick
 * puts the real floor here anyway: 4 s means the first re-prompt lands on the
 * first tick at least 4 s after the original, i.e. 4-5 s. */
#define MSGWAIT_REPROMPT_MS 4000

void CompanionMesh::msgWaitingWatchdog()
{
	if (_offline_queue_count == 0) {
		return;
	}
	/* Inside the initial-sync window or a live contact dump — the sync itself
	 * will drain the queue, and prompting now would truncate it. Both cases
	 * live in vcontactMsgWaitHeld(); the dump has its own stall watchdog and
	 * _last_sync_req_ms covers an active message drain, so nothing is stranded
	 * by waiting here. */
	if (vcontactMsgWaitHeld()) {
		return;
	}
	int64_t now = _ms->getMillis();
	int64_t last = (_last_sync_req_ms > _last_msgwait_ms) ? _last_sync_req_ms
							      : _last_msgwait_ms;
	if (now - last < MSGWAIT_REPROMPT_MS) {
		return;
	}
	LOG_WRN("msgwait watchdog: %d queued, app quiet for %ds, re-prompting",
		_offline_queue_count, (int)((now - last) / 1000));
	pushMsgWaiting();
}

bool CompanionMesh::continueContactIteration()
{
	if (!_contact_iter_active) return false;

	if (_contact_iter_idx < _contact_iter_num) {
		ContactInfo c;
		/* getContactByIdx bounds-checks against the live table, so a slot that
		 * disappeared under us is skipped rather than read stale. */
		if (getContactByIdx(_contact_iter_idx, c)) {
			// Skip transient/anon contacts (ADV_TYPE_NONE) — never synced to the app.
			// Apply 'since' filter - only send contacts modified after the timestamp
			if (c.type != ADV_TYPE_NONE && c.lastmod > _contact_iter_since) {
				if (c.lastmod > _contact_iter_lastmod) {
					_contact_iter_lastmod = c.lastmod;
				}
				uint8_t rsp[CONTACT_FRAME_SIZE];
				size_t n = serializeContact(rsp, c, PACKET_CONTACT);
				if (!writeFrame(rsp, n)) {
					return true;
				}
			}
		}
		_contact_iter_idx++;
		return true;
	} else if (_contact_iter_idx == _contact_iter_num) {
		// Virtual tail entry: the v-contact (never in the real table).
		// _contact_iter_vc implies lastmod != 0 — deferred (clock-invalid)
		// state is excluded so the app never sees a 1970 timestamp.
		if (_contact_iter_vc && _vcontact_lastmod > _contact_iter_since) {
			if (_vcontact_lastmod > _contact_iter_lastmod) {
				_contact_iter_lastmod = _vcontact_lastmod;
			}
			ContactInfo vc;
			buildVContact(vc);
			uint8_t rsp[CONTACT_FRAME_SIZE];
			size_t n = serializeContact(rsp, vc, PACKET_CONTACT);
			if (!writeFrame(rsp, n)) {
				return true;  /* retry this step when TX drains */
			}
		}
		_contact_iter_idx++;
		return true;
	} else {
		// Send PACKET_CONTACT_END with most_recent_lastmod
		uint8_t rsp[5];
		rsp[0] = PACKET_CONTACT_END;
		put_le32(&rsp[1], _contact_iter_lastmod);
		if (!writeFrame(rsp, sizeof(rsp))) {
			return true;
		}
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
	// Table full — evict the entry that has been waiting longest. A rotating
	// write cursor is not equivalent: it advances independently of when each
	// slot was filled, so under a burst it can discard an ACK registered
	// moments ago while a much older one survives. Whichever entry we drop
	// can never be confirmed, so drop the one least likely to still be
	// answered.
	int idx = 0;
	for (int i = 1; i < ACK_TABLE_SIZE; i++) {
		if ((int32_t)(_ack_table[i].sent_time - _ack_table[idx].sent_time) < 0) {
			idx = i;
		}
	}
	_ack_table[idx].expected_ack = expected;
	_ack_table[idx].contact_idx = contact_idx;
	_ack_table[idx].sent_time = now_ms;
	_ack_table[idx].active = true;
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
	/* Keep fixed-slot channel serialization compatible with Arduino.
	 * DataStore::saveChannels() expects contiguous indexes until false. */
	return getChannel(idx, ch);
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

	/* A re-heard advert from a contact we already have is liveness only —
	 * the base class has just refreshed last_advert_timestamp and lastmod,
	 * and nothing else typically moved.  Persist it lazily rather than
	 * spending a full-file rewrite per advert; a genuinely new contact, a
	 * path change (onContactPathUpdated) or a message all still flush on the
	 * short deadline. */
	markContactsDirty(is_new);

	// Update advert path table
	if (path && mesh::Packet::isValidPathLen(path_len)) {
		/* Update this node's existing slot if it has one, else take the least
		 * recently heard (an empty slot has recv_timestamp 0 and wins first).
		 *
		 * This used to round-robin into a write cursor with no lookup, so a
		 * node heard N times occupied N of the slots, evicting other nodes,
		 * and findAdvertPath() -- which returns the first array match, not the
		 * newest -- could hand the app a stale path for a node whose route had
		 * since changed. */
		AdvertPath *ap = &_advert_paths[0];
		uint32_t oldest = 0xFFFFFFFF;
		for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {
			if (memcmp(_advert_paths[i].pubkey_prefix, contact.id.pub_key,
				   sizeof(AdvertPath::pubkey_prefix)) == 0) {
				ap = &_advert_paths[i];
				break;
			}
			if (_advert_paths[i].recv_timestamp < oldest) {
				oldest = _advert_paths[i].recv_timestamp;
				ap = &_advert_paths[i];
			}
		}
		memcpy(ap->pubkey_prefix, contact.id.pub_key, 7);
		memcpy(ap->name, contact.name, sizeof(ap->name));
		ap->recv_timestamp = (uint32_t)getRTCClock()->getCurrentTime();
		/* path source is from inbound advert; upstream parser bounds it within
		 * the packet payload.  AdvertPath::path is MAX_PATH_SIZE-sized. */
		ap->path_len = mesh::Packet::copyPath(ap->path, path, MAX_PATH_SIZE, path_len);
	}

	// Send push notification
	if (is_new) {
		uint8_t rsp[CONTACT_FRAME_SIZE];
		size_t n = serializeContact(rsp, contact);  /* no header — push code is separate */
		LOG_DBG("onDiscoveredContact: sending PUSH_CODE_NEW_ADVERT (full contact)");
		sendPush(PUSH_CODE_NEW_ADVERT, rsp, n);
	} else {
		// ADVERT: send just pubkey
		LOG_DBG("onDiscoveredContact: sending PUSH_CODE_ADVERT (pubkey only)");
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
	{
		uint8_t recipient_pubkey[6];
		if (ui_joystick_try_match_ack(ack_crc, recipient_pubkey)) {
			return lookupContactByPubKey(recipient_pubkey, 6);
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
	queueMessage(contact, TXT_TYPE_PLAIN, pkt, sender_timestamp, nullptr, 0, text);
	pushMsgWaiting();
#if ZEPHCORE_HAS_UI_TASK
	notify_contact_msg_ui(contact, pkt, text, _offline_queue_count);
#endif
}

void CompanionMesh::queueMessage(const ContactInfo &contact, uint8_t txt_type, mesh::Packet *pkt,
	 uint32_t sender_timestamp, const uint8_t *extra, int extra_len, const char *text)
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

	frame[i++] = (pkt && pkt->isRouteFlood()) ? pkt->path_len : OUT_PATH_UNKNOWN;

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

	LOG_DBG("queueMessage: frame_len=%d type=0x%02x", i, frame[0]);
	addToOfflineQueue(frame, i);
}

void CompanionMesh::queueLocalSentContactMessage(const ContactInfo &contact,
	uint32_t timestamp, const char *text, bool delivered)
{
	if (!text) return;
	uint8_t frame[MAX_FRAME_SIZE];
	int i = 0;

	if (_app_target_ver >= 3) {
		frame[i++] = PACKET_CONTACT_MSG_V3;
		frame[i++] = 0;  // SNR (local origin, no incoming packet)
		frame[i++] = 0;  // reserved1
		frame[i++] = 0;  // reserved2
	} else {
		frame[i++] = PACKET_CONTACT_MSG_RECV;
	}
	memcpy(&frame[i], contact.id.pub_key, 6);
	i += 6;
	/* path_len = 0 → phone app renders "0 hops" / direct. OUT_PATH_SENT
	 * (0xFE) gets misrendered as "62 hops" because the app does
	 * path_len & 63 unconditionally. We're not over the air for the
	 * BLE-app mirror; "0 hops between sender and viewer" is literally
	 * true when you sent it yourself. */
	frame[i++] = 0;
	frame[i++] = TXT_TYPE_PLAIN;
	put_le32(&frame[i], timestamp);
	i += 4;

	/* DM wire-text has no sender prefix (the sender is the pubkey field).
	 * For local-sent mirrors the phone app would otherwise show them as
	 * messages *from* the contact, indistinguishable from incoming. Prepend
	 * a visible delivery indicator so the user can tell at a glance both
	 * which side sent it and whether it was acked. */
	const char *marker = delivered ? "(>>\xe2\x9c\x93) "    /* (>>✓) UTF-8 */
								   : "(>>\xe2\x9c\x97) ";   /* (>>✗) UTF-8 */
	size_t marker_len = strlen(marker);
	if (i + marker_len <= sizeof(frame)) {
		memcpy(&frame[i], marker, marker_len);
		i += marker_len;
	}
	size_t text_len = strlen(text);
	if (i + text_len > sizeof(frame)) {
		text_len = sizeof(frame) - i;
	}
	memcpy(&frame[i], text, text_len);
	i += text_len;

	LOG_DBG("queueLocalSentContactMessage: frame_len=%d delivered=%d", i, (int)delivered);
	addToOfflineQueue(frame, i);
	pushMsgWaiting();
}

void CompanionMesh::queueLocalSentChannelMessage(uint8_t channel_idx,
	uint32_t timestamp, const char *text, bool heard_repeat)
{
	if (!text) return;
	uint8_t frame[MAX_FRAME_SIZE];
	int i = 0;

	if (_app_target_ver >= 3) {
		frame[i++] = PACKET_CHANNEL_MSG_V3;
		frame[i++] = 0;  // SNR
		frame[i++] = 0;  // reserved1
		frame[i++] = 0;  // reserved2
	} else {
		frame[i++] = PACKET_CHANNEL_MSG_RECV;
	}
	frame[i++] = channel_idx;
	/* path_len = 0 (matches the DM mirror — phone renders "0 hops"
	 * instead of garbled-modulo-63 for OUT_PATH_SENT). */
	frame[i++] = 0;
	frame[i++] = TXT_TYPE_PLAIN;
	put_le32(&frame[i], timestamp);
	i += 4;

	/* Channel wire-text is "<sender_name>: <body>" — the same prefix
	 * BaseChatMesh::sendGroupMessage applied over LoRa.  Prepend a
	 * heard/unheard marker so the phone app can distinguish whether the
	 * mesh propagated our flood. */
	const char *marker = heard_repeat ? "(>>\xe2\x9c\x93) "    /* (>>✓) UTF-8 */
									  : "(>>\xe2\x9c\x97) ";   /* (>>✗) UTF-8 */
	int n = snprintf((char *)&frame[i], sizeof(frame) - i, "%s%s: ", marker, prefs.node_name);
	if (n < 0) n = 0;
	if ((size_t)n > sizeof(frame) - i) n = sizeof(frame) - i;
	i += n;
	size_t text_len = strlen(text);
	if (i + text_len > sizeof(frame)) {
		text_len = sizeof(frame) - i;
	}
	memcpy(&frame[i], text, text_len);
	i += text_len;

	LOG_DBG("queueLocalSentChannelMessage: frame_len=%d channel_idx=%d heard=%d",
		i, channel_idx, (int)heard_repeat);
	addToOfflineQueue(frame, i);
	pushMsgWaiting();
}

void CompanionMesh::onCommandDataRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const char *text)
{
	LOG_INF("onCommandDataRecv: from '%s' timestamp=%u text='%s'",
		contact.name, sender_timestamp, text);

	markConnectionActive(contact);
	queueMessage(contact, TXT_TYPE_CLI_DATA, pkt, sender_timestamp, nullptr, 0, text);
	pushMsgWaiting();
	{
		struct ui_joystick_cli_data cli = { text };
		ui_notify_joystick_event(UI_JOYSTICK_CLI_RESPONSE, contact.id.pub_key, &cli);
	}
}

void CompanionMesh::onCLICommandRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const char *text, char *reply)
{
	markConnectionActive(contact);  // in case this is from a server, and we have a connection
	if (contact.isRemoteCLIAllowed()) {
		if (!handleCommand(text, sender_timestamp, reply)) {
			strcat(reply, "Unknown command");  // reply may have cmd prefix from 'text'
		}
	} else {
		queueMessage(contact, TXT_TYPE_CLI_COMMAND, pkt, sender_timestamp, nullptr, 0, text);
		pushMsgWaiting();
	}
}

/* Over the air, a contact with flag 0x10 reaches the commands upstream's
 * companion handles (MyMesh::handleCommand and CommonRadioPrefs) and nothing
 * else: the ZephCore-only commands (start ota/dfu, erase, prv.key, clkreboot,
 * v.contact, autoshutdown, ...) stay local. */
static bool isUpstreamCompanionCommand(const char *cmd)
{
	static const char *const exact[] = {
		"get radio", "get freq", "get af", "get dutycycle", "get int.thresh", "get cad",
		"get radio.rxgain", "get tx", "get rxdelay", "get agc.reset.interval",
		"get path.hash.mode", "get multi.acks", "get txdelay", "get direct.txdelay",
		"reboot", "poweroff", "shutdown", "get name", "get wifi.pwd", "set wifi.clear",
		"get wifi.ssid", "get wifi.enabled", "get wifi.status", "get wifi.ip",
		"board", "ver", "get tz.offset",
	};
	static const char *const prefix[] = {
		"set radio ", "set af ", "set dutycycle ", "set int.thresh ", "set cad ",
		"set radio.rxgain ", "get tx ", "set tx ", "set rxdelay ", "set agc.reset.interval ",
		"set path.hash.mode ", "set multi.acks ", "set txdelay ", "set direct.txdelay ",
		"set name ", "set pin ", "set wifi.ssid ", "set wifi.pwd ", "set wifi.enabled ",
		"set tz.offset ",
	};
	for (const char *e : exact) {
		if (strcmp(cmd, e) == 0) return true;
	}
	for (const char *p : prefix) {
		if (strncmp(cmd, p, strlen(p)) == 0) return true;
	}
	return false;
}

bool CompanionMesh::handleCommand(const char *command, uint32_t sender_timestamp, char *reply)
{
	while (*command == ' ') command++;  // skip leading spaces

	uint8_t hdr_used = 0;
	if (strlen(command) > 4 && command[2] == '|') {  // optional prefix (for companion radio CLI)
		memcpy(reply, command, 3);                    // reflect the prefix back
		reply += 3;
		command += 3;
		hdr_used = 3;
	}
	*reply = 0;

	if (sender_timestamp != 0 && !isUpstreamCompanionCommand(command)) {
		return false;
	}
	if (_cli_exec_cb == nullptr) {
		strcpy(reply, "CLI not available");
		return true;
	}
	_cli_exec_cb(command, sender_timestamp, hdr_used, reply);
	return *reply != 0;
}

void CompanionMesh::onSignedMessageRecv(const ContactInfo &contact, mesh::Packet *pkt,
	uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text)
{
	LOG_INF("onSignedMessageRecv: from '%s' timestamp=%u text='%s'",
		contact.name, sender_timestamp, text);

	markConnectionActive(contact);
	markContactsDirty();
	// sender_prefix is 4 bytes
	queueMessage(contact, TXT_TYPE_SIGNED_PLAIN, pkt, sender_timestamp, sender_prefix, 4, text);
	pushMsgWaiting();
#if ZEPHCORE_HAS_UI_TASK
	notify_contact_msg_ui(contact, pkt, text, _offline_queue_count);
#endif
}

uint32_t CompanionMesh::calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const
{
	return 500 + (uint32_t)(16.0f * pkt_airtime_millis);
}

uint32_t CompanionMesh::calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const
{
	/* path_len is the packed hash-size-encoded byte (top 2 bits = hash_size-1,
	 * bottom 6 = hop count) — mask to the real hop count before scaling, or
	 * any path learned at path_hash_mode>0 inflates this by up to ~17x
	 * (e.g. a 3-hop path at 2-byte hashes encodes as 67, not 3). */
	uint8_t path_hash_count = path_len & 63;
	return 500 + (uint32_t)((pkt_airtime_millis * 6.0f + 250) * (path_hash_count + 1));
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

	uint8_t path_len = (pkt && pkt->isRouteFlood()) ? pkt->path_len : OUT_PATH_UNKNOWN;
	frame[i++] = path_len;

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

	LOG_DBG("onChannelMessageRecv: frame_len=%d channel_idx=%d", i, channel_idx);
	addToOfflineQueue(frame, i);
	pushMsgWaiting();
#if ZEPHCORE_HAS_UI_TASK
	{
		ChannelDetails ch;
		const char *ch_name = "";
		if (getChannel(channel_idx, ch)) ch_name = ch.name;
		ui_notify_channel_msg(
			ch_name, text, timestamp, path_len, (uint16_t)_offline_queue_count
		);
	}
#endif
}

void CompanionMesh::onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt,
	uint16_t data_type, const uint8_t *data, size_t data_len)
{
	if (data_len > MAX_CHANNEL_DATA_LENGTH) {
		LOG_WRN("onChannelDataRecv: dropping payload_len=%d exceeds frame limit=%d",
			(int)data_len, (int)MAX_CHANNEL_DATA_LENGTH);
		return;
	}

	uint8_t frame[MAX_FRAME_SIZE];
	int i = 0;
	frame[i++] = PACKET_CHANNEL_DATA_RECV;
	frame[i++] = pkt ? (int8_t)(pkt->getSNR() * 4) : 0;
	frame[i++] = 0;  // reserved1
	frame[i++] = 0;  // reserved2

	uint8_t channel_idx = findChannelIdx(channel);
	frame[i++] = channel_idx;
	frame[i++] = (pkt && pkt->isRouteFlood()) ? pkt->path_len : OUT_PATH_UNKNOWN;
	frame[i++] = (uint8_t)(data_type & 0xFF);
	frame[i++] = (uint8_t)(data_type >> 8);
	frame[i++] = (uint8_t)data_len;

	int copy_len = (int)data_len;
	if (copy_len > 0) {
		memcpy(&frame[i], data, copy_len);
		i += copy_len;
	}

	LOG_DBG("onChannelDataRecv: frame_len=%d channel_idx=%d data_type=%d",
		i, channel_idx, (int)data_type);
	addToOfflineQueue(frame, i);
	pushMsgWaiting();
}

int CompanionMesh::appendSelfTelemetry(uint8_t *reply, uint8_t permissions)
{
	int i = 0;
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

	/* Sensors are read once here.  External temp/humidity/pressure require
	 * TELEM_PERM_ENVIRONMENT, but the MCU die temperature is reported under
	 * base permission — matching Arduino MeshCore (15e259c5) and ZephCore's
	 * own repeater/room-server telemetry, neither of which gates it. */
	struct env_data env;
	bool env_ok = (env_sensors_read(&env) == 0);
	bool temp_reported = false;

	if (permissions & TELEM_PERM_ENVIRONMENT) {
		if (env_ok) {
			if (env.has_temperature) {
				temp_reported = true;
				reply[i++] = CH_SELF;
				reply[i++] = LPP_TEMPERATURE;
				int16_t temp = (int16_t)(env.temperature_c * 10);
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
			if (env.has_luminosity) {
				reply[i++] = CH_SELF;
				reply[i++] = LPP_LUMINOSITY;
				float lum = env.luminosity;
				if (lum < 0.0f) lum = 0.0f;
				if (lum > 65535.0f) lum = 65535.0f;
				uint16_t lux = (uint16_t)lum;
				reply[i++] = (lux >> 8) & 0xFF;
				reply[i++] = lux & 0xFF;
			}
		}

		// Power monitor telemetry (INA219/INA3221/ina2xx)
		if (power_sensors_available()) {
			struct power_data pwr;
			if (power_sensors_read(&pwr) == 0) {
				uint8_t ch = CH_SELF + 1;
				for (int j = 0; j < pwr.num_channels; j++) {
					if (pwr.channels[j].valid) {
						// Voltage: [ch][LPP_VOLTAGE=116][2-byte 0.01V signed]
						reply[i++] = ch;
						reply[i++] = 116;
						int16_t v = (int16_t)(pwr.channels[j].voltage_v * 100);
						reply[i++] = (v >> 8) & 0xFF;
						reply[i++] = v & 0xFF;
						// Current: [ch][LPP_CURRENT=117][2-byte 0.001A signed]
						// Signed: a bidirectional monitor (INA219) reports discharge
						// as negative, and an unsigned cast saturates it to 0.
						reply[i++] = ch;
						reply[i++] = 117;
						int16_t c = (int16_t)(pwr.channels[j].current_a * 1000);
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

	/* MCU die temperature — reported under base permission, but only when no
	 * external sensor already supplied a CH_SELF temperature (never emit two). */
	if (!temp_reported && env_ok && env.has_mcu_temperature) {
		reply[i++] = CH_SELF;
		reply[i++] = LPP_TEMPERATURE;
		int16_t temp = (int16_t)(env.mcu_temperature_c * 10);
		reply[i++] = (temp >> 8) & 0xFF;
		reply[i++] = temp & 0xFF;
	}

	// Trigger GPS wake for fresh fix on next request
	if (gps_is_available() && gps_is_enabled()) {
		gps_request_fresh_fix();
	}

	return i;
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

			// Build Cayenne LPP telemetry response: reflect sender_timestamp
			// back as a 4-byte tag, then append battery/GPS/env/power.
			memcpy(reply, &sender_timestamp, 4);
			int n = appendSelfTelemetry(&reply[4], permissions);
			return 4 + n;
		} else {
			LOG_INF("onContactRequest: telemetry denied for contact");
		}
	}

	return 0;  // Unknown request or denied
}

void CompanionMesh::logRx(mesh::Packet *pkt, int len, float score)
{
	packet_log_rx(getLogDateTime(), pkt, _radio->getLastRSSI(), score, _radio->getEstAirtimeFor(len));
}

void CompanionMesh::logTx(mesh::Packet *pkt, int)
{
	packet_log_tx(getLogDateTime(), pkt);
#if ZEPHCORE_HAS_UI_TASK
	ui_notify_packet_sent();
#endif
}

void CompanionMesh::onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len)
{
	LOG_DBG("onContactResponse: len=%d from contact", len);
	if (len < 4) return;  // Need at least 4-byte tag

	uint32_t tag;
	memcpy(&tag, data, 4);
	LOG_DBG("onContactResponse: tag=%08x, _pending_login=%08x", tag, _pending_login);

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
		{
			struct ui_joystick_login_data login = {
				rsp[0] == PUSH_CODE_LOGIN_SUCCESS,
				rsp[1],  /* permissions (0 for fail/legacy) */
				tag,     /* server_time */
			};
			ui_notify_joystick_event(UI_JOYSTICK_LOGIN_RESULT, contact.id.pub_key, &login);
		}
		return;
	}

#if IS_ENABLED(CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK)
	/* Joystick-originated requests (ping and admin binary): tag-based match takes priority
	 * over the pubkey-based _pending_status check below.  Without this, a stale
	 * _pending_status for the same repeater would intercept any response and forward it
	 * to BLE, causing a joystick timeout. */
	if ((_pending_joystick_ping_tag  && tag == _pending_joystick_ping_tag) ||
	    (_pending_joystick_admin_tag && tag == _pending_joystick_admin_tag)) {
		_pending_joystick_ping_tag  = 0;
		_pending_joystick_admin_tag = 0;
		int8_t snr_remote = INT8_MIN;
		if (len >= 48) {
			int16_t rs;
			memcpy(&rs, &data[46], 2);
			snr_remote = (int8_t)(rs / 4);
		}
		struct ui_joystick_req_response_data rr = {
			(int8_t)_radio->getLastSNR(),
			snr_remote,
			len > 4 ? &data[4] : nullptr,
			(uint8_t)(len > 4 ? len - 4 : 0),
		};
		ui_notify_joystick_event(UI_JOYSTICK_REQ_RESPONSE, contact.id.pub_key, &rr);
		return;
	}
#endif

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

	{
		int8_t snr_remote = INT8_MIN;
		if (len >= 48) {
			int16_t rs;
			memcpy(&rs, &data[46], 2);
			snr_remote = (int8_t)(rs / 4);
		}
		struct ui_joystick_req_response_data rr = {
			(int8_t)_radio->getLastSNR(),
			snr_remote,
			len > 4 ? &data[4] : nullptr,
			(uint8_t)(len > 4 ? len - 4 : 0),
		};
		ui_notify_joystick_event(UI_JOYSTICK_REQ_RESPONSE, contact.id.pub_key, &rr);
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
	LOG_DBG("onTraceRecv: tag=0x%08x auth=0x%08x flags=0x%02x path_len=%d",
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
	LOG_DBG("onControlDataRecv: payload_len=%d path_len=%d", packet->payload_len, packet->path_len);

	// Buffer: [PUSH_CODE_CONTROL_DATA][snr*4][rssi][path_len][payload...]
	if (packet->payload_len + 4 > MAX_FRAME_SIZE) {
		LOG_WRN("onControlDataRecv: payload too long (%d)", packet->payload_len);
		return;
	}

	if (packet->payload_len >= 6 + PUB_KEY_SIZE &&
	    (packet->payload[0] & 0xF0) == CTL_TYPE_NODE_DISCOVER_RESP &&
	    (packet->payload[0] & 0x0F) == ADV_TYPE_REPEATER) {
		struct ui_joystick_discover_data dd = {
			(int8_t)packet->getSNR(), (int8_t)((int8_t)packet->payload[1] / 4), packet->path_len
		};
		ui_notify_joystick_event(UI_JOYSTICK_DISCOVER_RESP, &packet->payload[6], &dd);
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
	LOG_DBG("onRawDataRecv: payload_len=%d", packet->payload_len);

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
	return computeAdaptiveFloodDelay(packet);
}

uint32_t CompanionMesh::getDirectRetransmitDelay(const mesh::Packet *packet)
{
	return computeAdaptiveDirectDelay(packet);
}

uint32_t CompanionMesh::getInitialFloodJitter(const mesh::Packet *packet)
{
	(void)packet;
	/* Fixed companion origin jitter window: 20..150ms (inclusive). */
	return 20 + getRNG()->nextInt(0, 131);
}

uint8_t CompanionMesh::getDutyCyclePercent() const
{
	/* Arduino formula: duty% = 100 / (af + 1). af=0 → 100%, af=9 → 10%. */
	return (uint8_t)(100.0f / (prefs.airtime_factor + 1.0f) + 0.5f);
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

bool CompanionMesh::handleCmdFrame(const uint8_t *data, size_t len)
{
	if (len < 1) return false;

	/* Debug: log all incoming commands */
	LOG_DBG("CMD: 0x%02x len=%u", data[0], (unsigned)len);

	/* An active contact dump survives interleaved commands — the app is free to
	 * talk to us mid-sync and does (it sets the clock on a cold boot, and
	 * pipelines CMD_GET_CHANNEL bursts).  Aborting the iterator here truncated
	 * the dump with a premature PACKET_CONTACT_END after whatever had streamed,
	 * so the app waited forever for the rest.  Upstream interleaves the same way
	 * (MyMesh::checkSerialInterface); CMD_APP_START remains the only reset. */

	/* V-contact interception — must run before any contact lookup so a frame
	 * addressed to the loopback contact can never create a radio packet. */
	if (vcontactHandleFrame(data, len)) {
		return true;
	}

	switch (data[0]) {
	case CMD_DEVICE_QUERY: return handleCmdDeviceQuery(data, len);
	case CMD_APP_START: return handleCmdAppStart(data, len);
	case CMD_RUN_CLI_COMMAND: return handleCmdRunCliCommand(data, len);
	case CMD_SEND_TXT_MSG: return handleCmdSendTxtMsg(data, len);
	case CMD_SEND_CHANNEL_TXT_MSG: return handleCmdSendChannelTxtMsg(data, len);
	case CMD_SEND_CHANNEL_DATA: return handleCmdSendChannelData(data, len);
	case CMD_GET_CONTACTS: return handleCmdGetContacts(data, len);
	case CMD_SET_ADVERT_NAME: return handleCmdSetAdvertName(data, len);
	case CMD_SET_ADVERT_LATLON: return handleCmdSetAdvertLatlon(data, len);
	case CMD_GET_DEVICE_TIME: return handleCmdGetDeviceTime(data, len);
	case CMD_SET_DEVICE_TIME: return handleCmdSetDeviceTime(data, len);
	case CMD_SEND_SELF_ADVERT: return handleCmdSendSelfAdvert(data, len);
	case CMD_RESET_PATH: return handleCmdResetPath(data, len);
	case CMD_ADD_UPDATE_CONTACT: return handleCmdAddUpdateContact(data, len);
	case CMD_REMOVE_CONTACT: return handleCmdRemoveContact(data, len);
	case CMD_SHARE_CONTACT: return handleCmdShareContact(data, len);
	case CMD_GET_CONTACT_BY_KEY: return handleCmdGetContactByKey(data, len);
	case CMD_EXPORT_CONTACT: return handleCmdExportContact(data, len);
	case CMD_IMPORT_CONTACT: return handleCmdImportContact(data, len);
	case CMD_SYNC_NEXT_MESSAGE: return handleCmdSyncNextMessage(data, len);
	case CMD_SET_RADIO_PARAMS: return handleCmdSetRadioParams(data, len);
	case CMD_SET_RADIO_TX_POWER: return handleCmdSetRadioTxPower(data, len);
	case CMD_SET_TUNING_PARAMS: return handleCmdSetTuningParams(data, len);
	case CMD_GET_TUNING_PARAMS: return handleCmdGetTuningParams(data, len);
	case CMD_SET_OTHER_PARAMS: return handleCmdSetOtherParams(data, len);
	case CMD_SET_PATH_HASH_MODE: return handleCmdSetPathHashMode(data, len);
	case CMD_REBOOT: return handleCmdReboot(data, len);
	case CMD_GET_BATT_AND_STORAGE: return handleCmdGetBattAndStorage(data, len);
	case CMD_EXPORT_PRIVATE_KEY: return handleCmdExportPrivateKey(data, len);
	case CMD_IMPORT_PRIVATE_KEY: return handleCmdImportPrivateKey(data, len);
	case CMD_SEND_RAW_DATA: return handleCmdSendRawData(data, len);
	case CMD_SEND_LOGIN: return handleCmdSendLogin(data, len);
	case CMD_SEND_ANON_REQ: return handleCmdSendAnonReq(data, len);
	case CMD_SEND_STATUS_REQ: return handleCmdSendStatusReq(data, len);
	case CMD_SEND_PATH_DISCOVERY_REQ: return handleCmdSendPathDiscoveryReq(data, len);
	case CMD_SEND_TELEMETRY_REQ: return handleCmdSendTelemetryReq(data, len);
	case CMD_SEND_BINARY_REQ: return handleCmdSendBinaryReq(data, len);
	case CMD_HAS_CONNECTION: return handleCmdHasConnection(data, len);
	case CMD_LOGOUT: return handleCmdLogout(data, len);
	case CMD_GET_CHANNEL: return handleCmdGetChannel(data, len);
	case CMD_SET_CHANNEL: return handleCmdSetChannel(data, len);
	case CMD_SIGN_START: return handleCmdSignStart(data, len);
	case CMD_SIGN_DATA: return handleCmdSignData(data, len);
	case CMD_SIGN_FINISH: return handleCmdSignFinish(data, len);
	case CMD_SEND_TRACE_PATH: return handleCmdSendTracePath(data, len);
	case CMD_SET_DEVICE_PIN: return handleCmdSetDevicePin(data, len);
	case CMD_GET_CUSTOM_VARS: return handleCmdGetCustomVars(data, len);
	case CMD_SET_CUSTOM_VAR: return handleCmdSetCustomVar(data, len);
	case CMD_GET_ADVERT_PATH: return handleCmdGetAdvertPath(data, len);
	case CMD_GET_STATS: return handleCmdGetStats(data, len);
	case CMD_FACTORY_RESET: return handleCmdFactoryReset(data, len);
	case CMD_SET_FLOOD_SCOPE_KEY: return handleCmdSetFloodScopeKey(data, len);
	case CMD_SET_DEFAULT_FLOOD_SCOPE: return handleCmdSetDefaultFloodScope(data, len);
	case CMD_GET_DEFAULT_FLOOD_SCOPE: return handleCmdGetDefaultFloodScope(data, len);
	case CMD_SEND_CONTROL_DATA: return handleCmdSendControlData(data, len);
	case CMD_SET_AUTOADD_CONFIG: return handleCmdSetAutoaddConfig(data, len);
	case CMD_GET_AUTOADD_CONFIG: return handleCmdGetAutoaddConfig(data, len);
	case CMD_GET_ALLOWED_REPEAT_FREQ: return handleCmdGetAllowedRepeatFreq(data, len);
	case CMD_SEND_RAW_PACKET: return handleCmdSendRawPacket(data, len);
	default:
		break;
	}

	return false;
}

bool CompanionMesh::handleCmdDeviceQuery(const uint8_t *data, size_t len)
{
	if (len < 2) {
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}
	{
		_app_target_ver = data[1];  // Which version of protocol does app understand
		#ifndef FIRMWARE_BUILD_DATE
		#define FIRMWARE_BUILD_DATE __DATE__
		#endif
		#ifndef FIRMWARE_VERSION
		#define FIRMWARE_VERSION "v0.0.0-dev"  // real value injected by CMakeLists.txt
		#endif
		/* Wire format reserves 40 bytes for the board name. Require room
		 * for a null terminator within those 40 bytes so a phone parsing
		 * it as a C-string never reads past the field. */
		static_assert(sizeof(CONFIG_ZEPHCORE_BOARD_NAME) <= 40,
			"CONFIG_ZEPHCORE_BOARD_NAME must fit in 40 bytes including null terminator");
		static const uint8_t fw_build[12] = FIRMWARE_BUILD_DATE;
		static const uint8_t model[40] = CONFIG_ZEPHCORE_BOARD_NAME;
		static const uint8_t version[20] = FIRMWARE_VERSION;  // injected by CMakeLists.txt
		uint8_t rsp[82];
		rsp[0] = PACKET_DEVICE_INFO;
		/* v14 = CMD_RUN_CLI_COMMAND / PACKET_CLI_REPLY, and TXT_TYPE_CLI_COMMAND
		 * both sent (CMD_SEND_TXT_MSG) and executed from a contact with flag
		 * 0x10 (onCLICommandRecv), as upstream. */
		rsp[1] = 14;  // FIRMWARE_VER_CODE
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
}

bool CompanionMesh::handleCmdAppStart(const uint8_t *data, size_t len)
{
	if (len < 8) {
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}
	LOG_INF("CMD_APP_START: session reset, %d queued", _offline_queue_count);
	// Reset per-session state for a fresh app session. BLE/USB also run
	// this cleanup on disconnect, but the serial transport has no
	// disconnect event, so APP_START is its authoritative session-reset
	// point: drop a stale contact iteration (a leftover PACKET_CONTACT_END
	// would confuse the new session's sync state machine), a half-finished
	// message sync, and free an abandoned Ed25519 sign buffer (else an 8KB
	// leak if the previous session dropped mid-CMD_SIGN). Idempotent — all
	// no-ops when the state is already clean.
	_contact_iter_active = false;
	cancelSyncPending();
	cleanupSignState();
	/* Drop a deferred v-contact confirm from the previous session: its ack
	 * matches an outbound message the new session never sent, so delivering
	 * it here would push an unmatched SEND_CONFIRMED into a fresh app. */
	_vcontact_confirm_ack = 0;
	k_work_cancel_delayable(&_vcontact_confirm_work.work);

	/* New session: suppress v-contact notice MSG_WAITING until the initial
	 * sync (contacts + messages) completes at PACKET_NO_MORE_MSGS, or the
	 * deadline expires — see _vcontact_hold_expiry for why the latch needs
	 * a second exit. */
	_vcontact_hold_msgwait = true;
	_vcontact_hold_expiry = _ms->getMillis() + VCONTACT_HOLD_MAX_MS;
	/* An app-side delete only hides the v-contact for the session it
	 * happened in — this is that session boundary, so it comes back. */
	_vcontact_app_hidden = false;

	/* If a time source already ran (hardware RTC, GPS), activate the
	 * deferred v-contact and flush buffered notices for this session. */
	vcontactClockSynced();

	/* Re-stamp the v-contact once per app session so it stays "fresh".
	 * _vcontact_lastmod feeds both lastmod and last_advert_timestamp, and
	 * it used to move only on boot / rename / identity import: the app
	 * showed an ever-growing "last seen" age, and — worse — the contact
	 * sync gate is `_vcontact_lastmod > _contact_iter_since`, so after the
	 * first sync the v-contact was never streamed again and app-side state
	 * (flags, name) could never be corrected. Bumping here, before the
	 * CMD_GET_CONTACTS that follows APP_START, means every session's sync
	 * carries a current timestamp; deferring it to the end of sync would
	 * always land one session late. Silent on purpose — no NEW_ADVERT push
	 * mid-handshake; the sync itself delivers it. */
	if (vcontactReady() && vcontactClockValid()) {
		_vcontact_lastmod = (uint32_t)getRTCClock()->getCurrentTime();
	}

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

bool CompanionMesh::handleCmdRunCliCommand(const uint8_t *data, size_t len)
{
	/* Runs a CLI line on behalf of the connected app.  This is the same
	 * local CLI the USB text sideband and the v-contact chat already reach,
	 * over the same already-paired transport, so it adds no reach that an
	 * app connected to this node did not already have — it is executed with
	 * sender_timestamp 0 (local) for exactly that reason. */
	if (len >= 2) {
		if (_cli_exec_cb == nullptr) {
			writeErrFrame(ERR_UNSUPPORTED);
			return true;
		}

		/* `data` is const, so the line has to be copied out to be
		 * null-terminated.  A command can never exceed one frame. */
		char line[MAX_FRAME_SIZE];
		size_t cmd_len = len - 1;
		if (cmd_len > sizeof(line) - 1) cmd_len = sizeof(line) - 1;
		memcpy(line, &data[1], cmd_len);
		line[cmd_len] = '\0';

		/* Reply is built in place behind the frame type byte. */
		uint8_t rsp[1 + COMPANION_CLI_PREFIX_ROOM + COMPANION_CLI_REPLY_SIZE];
		char *reply = (char *)&rsp[1];
		rsp[0] = PACKET_CLI_REPLY;
		if (!handleCommand(line, 0, reply)) {
			strcat(reply, "Unknown command");  // reply may have cmd prefix from 'line'
		}

		/* A CommonCLI reply can be longer than one companion frame and the
		 * app protocol has no continuation for this response, so truncate
		 * rather than drop. */
		size_t rlen = strlen(reply);
		if (rlen > MAX_FRAME_SIZE - 1) rlen = MAX_FRAME_SIZE - 1;
		writeFrame(rsp, rlen + 1);
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendTxtMsg(const uint8_t *data, size_t len)
{
	// Frame format: cmd(1) + txt_type(1) + attempt(1) + timestamp(4) + pub_key_prefix(6) + text(N)
	// Minimum: 1 + 1 + 1 + 4 + 6 + 1 = 14 bytes
	LOG_DBG("CMD_SEND_TXT_MSG: len=%u (min=14)", (unsigned)len);
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

		LOG_DBG("CMD_SEND_TXT_MSG: txt_type=%d attempt=%d pubkey=%02x%02x%02x%02x%02x%02x",
			txt_type, attempt, pub_key_prefix[0], pub_key_prefix[1], pub_key_prefix[2],
			pub_key_prefix[3], pub_key_prefix[4], pub_key_prefix[5]);

		ContactInfo *contact = lookupContactByPubKey(pub_key_prefix, 6);
		if (contact && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_CLI_DATA ||
				txt_type == TXT_TYPE_CLI_COMMAND)) {
			LOG_DBG("CMD_SEND_TXT_MSG: contact='%s' text='%s' text_len=%u", contact->name, text, (unsigned)text_len);

			uint32_t expected_ack = 0, est_timeout;
			int result;

			if (txt_type == TXT_TYPE_CLI_DATA || txt_type == TXT_TYPE_CLI_COMMAND) {
				// Use node's RTC instead of app timestamp
				msg_timestamp = getRTCClock()->getCurrentTimeUnique();
				result = sendCommandData(*contact, msg_timestamp, attempt, txt_type, text,
							 est_timeout);
			} else {
				result = sendMessage(*contact, msg_timestamp, attempt, text, expected_ack, est_timeout);
			}

			LOG_DBG("CMD_SEND_TXT_MSG: sendMessage result=%d expected_ack=0x%08x", result, expected_ack);

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
				sendPacketSent(result, expected_ack, est_timeout);
			} else {
				writeErrFrame(ERR_TABLE_FULL);
			}
		} else {
			LOG_WRN("CMD_SEND_TXT_MSG: contact not found or unsupported txt_type=%d", txt_type);
			writeErrFrame(contact == nullptr ? ERR_NOT_FOUND : ERR_UNSUPPORTED);
		}
		return true;
	}
	LOG_WRN("CMD_SEND_TXT_MSG: frame too short (len=%u, need 14)", (unsigned)len);
	writeErrFrame(ERR_ILLEGAL_ARG);
	return true;
}

bool CompanionMesh::handleCmdSendChannelTxtMsg(const uint8_t *data, size_t len)
{
	// Arduino format: [cmd][txt_type][channel_idx][4-byte timestamp][text...]
	// Minimum: 1 + 1 + 1 + 4 + 1 = 8 bytes
	if (len < 8) {
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}
	{
		int i = 1;
		uint8_t txt_type = data[i++];
		uint8_t ch_idx = data[i++];
		uint32_t msg_timestamp = get_le32(&data[i]);
		i += 4;
		const char *text = (const char *)&data[i];
		size_t text_len = len - i;

		if (txt_type != TXT_TYPE_PLAIN) {
			writeErrFrame(ERR_UNSUPPORTED);
		} else {
			ChannelDetails ch;
			if (getChannel(ch_idx, ch)) {
				if (sendGroupMessage(msg_timestamp, ch.channel, prefs.node_name, text, text_len)) {
					writeOKFrame();
				} else {
					writeErrFrame(ERR_BAD_STATE);
				}
			} else {
				writeErrFrame(ERR_NOT_FOUND);
			}
		}
		return true;
	}
}

bool CompanionMesh::handleCmdSendChannelData(const uint8_t *data, size_t len)
{
	/* Minimum frame: cmd(1) + channel_idx(1) + path_len(1) + path(0..) +
	 * data_type(2) + payload(0..) = 5 bytes when path is empty. */
	if (len < 5) {
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}
	int i = 1;
	uint8_t channel_idx = data[i++];
	uint8_t path_len = data[i++];

	if (!mesh::Packet::isValidPathLen(path_len) && path_len != OUT_PATH_UNKNOWN) {
		LOG_WRN("CMD_SEND_CHANNEL_DATA invalid path size: %d", path_len);
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}

	/* Compute decoded path byte count and ensure source frame has room
	 * for path + data_type. (path_len is the 6-bit hash_count + 2-bit
	 * hash_size encoding; writePath itself will reject if src_len is
	 * too small, but failing here also rejects truncated data_type.) */
	uint8_t hash_count = path_len & 63;
	uint8_t hash_size  = (path_len >> 6) + 1;
	size_t path_bytes  = (path_len == OUT_PATH_UNKNOWN) ? 0
	                                                  : (size_t)hash_count * hash_size;
	if ((size_t)i + path_bytes + 2 > len) {
		LOG_WRN("CMD_SEND_CHANNEL_DATA short frame: len=%u need >=%u",
			(unsigned)len, (unsigned)(i + path_bytes + 2));
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}

	uint8_t path[MAX_PATH_SIZE];
	if (path_len != OUT_PATH_UNKNOWN) {
		/* src_len = remaining bytes from data[i] onward. */
		i += mesh::Packet::writePath(path, &data[i], len - i, path_len);
	}

	uint16_t data_type = ((uint16_t)data[i]) | (((uint16_t)data[i + 1]) << 8);
	i += 2;
	const uint8_t *payload = &data[i];
	int payload_len = (len > (size_t)i) ? (int)(len - i) : 0;

	ChannelDetails channel;
	if (!getChannel(channel_idx, channel)) {
		writeErrFrame(ERR_NOT_FOUND);
	} else if (data_type == DATA_TYPE_RESERVED) {
		writeErrFrame(ERR_ILLEGAL_ARG);
	} else if (payload_len > MAX_GROUP_DATA_LENGTH) {
		/* Radio-side bound (165), NOT the host-frame bound (167).  These
		 * disagree, and sendGroupData() enforces the smaller one, so gating
		 * here on MAX_CHANNEL_DATA_LENGTH let a 166/167-byte payload pass the
		 * frame check, fail the radio check, and come back to the app as
		 * ERR_TABLE_FULL — the "outbound queue full, retry later" code — for a
		 * send that can never succeed at any queue depth.  Upstream 0a82fcd2. */
		LOG_WRN("CMD_SEND_CHANNEL_DATA payload too long: %d > %d", payload_len, MAX_GROUP_DATA_LENGTH);
		writeErrFrame(ERR_ILLEGAL_ARG);
	} else if (sendGroupData(channel.channel, path, path_len, data_type, payload, payload_len)) {
		writeOKFrame();
	} else {
		writeErrFrame(ERR_TABLE_FULL);
	}
	return true;
}

bool CompanionMesh::handleCmdGetContacts(const uint8_t *data, size_t len)
{
	if (_contact_iter_active) {
		writeErrFrame(ERR_BAD_STATE);
	} else {
		// Parse optional 'since' parameter for incremental sync
		if (len >= 5) {
			_contact_iter_since = get_le32(&data[1]);
		} else {
			_contact_iter_since = 0;
		}

		// Send PACKET_CONTACT_START with total count (unfiltered, but excluding
		// transient anon slots -- continueContactIteration() never streams those)
		_contact_iter_num = getNumContacts();
		_contact_iter_vc = vcontactReady();  /* virtual tail entry (post time sync) */
		uint32_t total = 0;
		for (int i = 0; i < _contact_iter_num; i++) {
			ContactInfo c;
			if (getContactByIdx(i, c) && c.type != ADV_TYPE_NONE) total++;
		}
		if (_contact_iter_vc) total++;
		uint8_t rsp[5];
		rsp[0] = PACKET_CONTACT_START;
		put_le32(&rsp[1], total);
		writeFrame(rsp, sizeof(rsp));
		_contact_iter_idx = 0;
		_contact_iter_lastmod = 0;
		_contact_iter_active = true;
	}
	return true;
}

bool CompanionMesh::handleCmdSetAdvertName(const uint8_t *data, size_t len)
{
	if (len >= 2) {
		size_t nlen = len - 1;
		if (nlen >= sizeof(prefs.node_name)) nlen = sizeof(prefs.node_name) - 1;
		memcpy(prefs.node_name, &data[1], nlen);
		prefs.node_name[nlen] = '\0';
		_store->savePrefs(prefs);
#if IS_ENABLED(CONFIG_BT)
		/* Push the new name to BLE so scanners see it without a reboot. */
		zephcore_ble_update_name(prefs.node_name);
#endif
		/* v-contact name tracks the node name — update the app's copy. */
		vcontactPushAdvert();
	}
	writeOKFrame();
	return true;
}

bool CompanionMesh::handleCmdSetAdvertLatlon(const uint8_t *data, size_t len)
{
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
			writeOKFrame();
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdGetDeviceTime(const uint8_t *data, size_t len)
{
	uint8_t rsp[5];
	rsp[0] = PACKET_CURR_TIME;
	put_le32(&rsp[1], (uint32_t)getRTCClock()->getCurrentTime());
	writeFrame(rsp, sizeof(rsp));
	return true;
}

bool CompanionMesh::handleCmdSetDeviceTime(const uint8_t *data, size_t len)
{
	if (len >= 5) {
		/* If we have GPS-synced time, ignore phone time sync attempts.
		 * GPS time is more accurate than phone time. Still return OK
		 * so the app doesn't keep retrying. */
		if (gps_has_time_sync()) {
			LOG_DBG("Ignoring phone time sync - GPS time sync active");
			writeOKFrame();
			vcontactClockSynced();  /* GPS time counts as valid too */
			return true;
		}

		uint32_t secs = get_le32(&data[1]);
		uint32_t curr = getRTCClock()->getCurrentTime();
		// Arduino: only allow setting time forward (prevents time attacks)
		if (secs >= curr) {
			getRTCClock()->setCurrentTime(secs);
			time_sync_report(TIME_SYNC_APP);
			zephcore_rtc_save(secs);  /* persist to hardware RTC */
			_timesync.noteManualSync((uint32_t)(k_uptime_get() / 1000));
			writeOKFrame();
			/* App just gave us wall time — activate the deferred
			 * v-contact and flush buffered notices. */
			vcontactClockSynced();
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendSelfAdvert(const uint8_t *data, size_t len)
{
	/* Optional param: data[1] == 1 means flood, else zero-hop */
	bool flood = (len >= 2 && data[1] == 1);
	if (sendSelfAdvert(flood)) {
		writeOKFrame();
	} else {
		writeErrFrame(ERR_TABLE_FULL);
	}
	return true;
}

bool CompanionMesh::handleCmdResetPath(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE) {
		ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
		if (c) {
			resetPathTo(*c);
			markContactsDirty();
			writeOKFrame();
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdAddUpdateContact(const uint8_t *data, size_t len)
{
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
		c.name[31] = '\0';  /* defensive null-term — matches CMD_SET_CHANNEL pattern */
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
			writeOKFrame();
		} else if (addContact(c)) {
			markContactsDirty();
			writeOKFrame();
		} else {
			writeErrFrame(ERR_TABLE_FULL);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdRemoveContact(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE) {
		ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
		if (c && removeContact(*c)) {
			_store->deleteBlobByKey(&data[1], PUB_KEY_SIZE);
			markContactsDirty();
			writeOKFrame();
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdShareContact(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE) {
		ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
		if (c && shareContactZeroHop(*c)) {
			writeOKFrame();
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdGetContactByKey(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE) {
		ContactInfo *c = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
		if (c) {
			uint8_t rsp[CONTACT_FRAME_SIZE];
			size_t n = serializeContact(rsp, *c, PACKET_CONTACT);
			writeFrame(rsp, n);
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdExportContact(const uint8_t *data, size_t len)
{
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
			writeErrFrame(ERR_TABLE_FULL);
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
				writeErrFrame(ERR_BAD_STATE);
			}
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	}
	return true;
}

bool CompanionMesh::handleCmdImportContact(const uint8_t *data, size_t len)
{
	// Arduino: len > 2 + 32 + 64 = 98 bytes (header + pubkey + signature)
	if (len > 2 + 32 + 64) {
		if (importContact(&data[1], len - 1)) {
			markContactsDirty();
			writeOKFrame();
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);  // Match Arduino error code
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSyncNextMessage(const uint8_t *data, size_t len)
{
	LOG_DBG("CMD_SYNC_NEXT_MESSAGE: queue_count=%d pending=%d",
		_offline_queue_count, _sync_pending);

	/* The app is draining — hold the watchdog off for another window. */
	_last_sync_req_ms = _ms->getMillis();
	LOG_INF("msgwait: app asked for next msg, %d queued",
		_offline_queue_count);

	/* Phone asking for next = implicit ACK for the previously-peeked message */
	if (_sync_pending) {
		confirmOfflineMessage();
		_sync_pending = false;
#if ZEPHCORE_HAS_UI_TASK
		ui_set_msg_count((uint16_t)_offline_queue_count);
#endif
	}

	uint8_t buf[MAX_FRAME_SIZE];
	size_t msg_len;
	if (peekOfflineMessage(buf, msg_len)) {
		LOG_DBG("CMD_SYNC_NEXT_MESSAGE: peeked msg_len=%u type=0x%02x", (unsigned)msg_len, buf[0]);
		if (writeFrame(buf, msg_len)) {
			_sync_pending = true;  /* confirmed on next request or lost on disconnect */
		}
	} else {
		LOG_DBG("CMD_SYNC_NEXT_MESSAGE: queue empty, sending NO_MORE_MSGS");
		uint8_t rsp[] = { PACKET_NO_MORE_MSGS };
		if (!writeFrame(rsp, sizeof(rsp))) {
			return true;
		}

		/* Initial sync is done — safe to apply deferred
		 * connection parameters now without disrupting
		 * channel/contact/message throughput. */
#if IS_ENABLED(CONFIG_BT)
		zephcore_ble_conn_params_ready();
#endif

		/* Full initial sync (contacts + messages) complete: stop
		 * suppressing v-contact notice prompts. Any notice queued during
		 * the window was just drained by this message-sync; if one raced in
		 * after the last peek, prompt for it now. */
		_vcontact_hold_msgwait = false;
		_vcontact_hold_expiry = 0;
		if (_offline_queue_count > 0) {
			pushMsgWaiting();
		}
	}
	return true;
}

bool CompanionMesh::handleCmdSetRadioParams(const uint8_t *data, size_t len)
{
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
			writeErrFrame(ERR_ILLEGAL_ARG);
		/* freq arrives in kHz, bw in Hz. Same limits as the USB
		 * CLI and the prefs loader — see NodePrefs.h. Accepting
		 * here what loadPrefs() later rejects is how a 2.4 GHz
		 * setting used to vanish on reboot. */
		} else if (freq >= (uint32_t)(ZC_RADIO_FREQ_MIN_MHZ * 1000.0f) &&
		    freq <= (uint32_t)(ZC_RADIO_FREQ_MAX_MHZ * 1000.0f) &&
		    bw >= (uint32_t)(ZC_RADIO_BW_MIN_KHZ * 1000.0f) &&
		    bw <= (uint32_t)(ZC_RADIO_BW_MAX_KHZ * 1000.0f) &&
		    sf >= 5 && sf <= 12 &&
		    cr >= 5 && cr <= 8) {
			/* Coding rate is deliberately not in this test:
			 * it changes airtime, not the channel or the
			 * per-SF/per-bandwidth detPeak base, so it does
			 * not invalidate the learned CAD offset. */
			bool preset_changed =
				prefs.freq != (float)freq / 1000.0f ||
				prefs.bw != (float)bw / 1000.0f ||
				prefs.sf != sf;
			prefs.freq = (float)freq / 1000.0f;
			prefs.bw = (float)bw / 1000.0f;
			prefs.sf = sf;
			prefs.cr = cr;
			prefs.client_repeat = repeat;
			_store->savePrefs(prefs);
			if (_radio_reconfig_cb) _radio_reconfig_cb(preset_changed);
			LOG_INF("SET_RADIO_PARAMS: client_repeat=%d", repeat);
			writeOKFrame();
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSetRadioTxPower(const uint8_t *data, size_t len)
{
	if (len >= 2) {
		int8_t power = (int8_t)data[1];
		if (power >= -9 && power <= MAX_LORA_TX_POWER) {
			prefs.tx_power_dbm = power;
			_store->savePrefs(prefs);
			if (_radio_reconfig_cb) _radio_reconfig_cb(false);
			writeOKFrame();
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSetTuningParams(const uint8_t *data, size_t len)
{
	if (len < 9) {
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}
	prefs.rx_delay_base = (float)get_le32(&data[1]) / 1000.0f;
	prefs.airtime_factor = (float)get_le32(&data[5]) / 1000.0f;
	_store->savePrefs(prefs);
	writeOKFrame();
	return true;
}

bool CompanionMesh::handleCmdGetTuningParams(const uint8_t *data, size_t len)
{
	uint8_t rsp[9];
	rsp[0] = PACKET_TUNING_PARAMS;
	put_le32(&rsp[1], (uint32_t)(prefs.rx_delay_base * 1000));
	put_le32(&rsp[5], (uint32_t)(prefs.airtime_factor * 1000));
	writeFrame(rsp, sizeof(rsp));
	return true;
}

bool CompanionMesh::handleCmdSetOtherParams(const uint8_t *data, size_t len)
{
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
	writeOKFrame();
	return true;
}

bool CompanionMesh::handleCmdSetPathHashMode(const uint8_t *data, size_t len)
{
	if (len >= 3 && data[1] == 0) {
		if (data[2] >= 3) {
			writeErrFrame(ERR_ILLEGAL_ARG);
		} else {
			prefs.path_hash_mode = data[2];
			_store->savePrefs(prefs);
			writeOKFrame();
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdReboot(const uint8_t *data, size_t len)
{
	if (len >= 7 && memcmp(&data[1], "reboot", 6) == 0) {
		LOG_INF("Reboot requested");
		/* Flush any pending lazy writes before reboot */
		flushDirtyContacts();
		flushDirtyChannels();
		writeOKFrame();
		sys_reboot(SYS_REBOOT_COLD);
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdGetBattAndStorage(const uint8_t *data, size_t len)
{
	uint8_t rsp[11];
	rsp[0] = PACKET_BATTERY;
	put_le16(&rsp[1], _batt_cb ? _batt_cb() : 0);
	put_le32(&rsp[3], _store->getStorageUsedKb());
	put_le32(&rsp[7], _store->getStorageTotalKb());
	writeFrame(rsp, sizeof(rsp));
	return true;
}

bool CompanionMesh::handleCmdExportPrivateKey(const uint8_t *data, size_t len)
{
	// Response: [PACKET_PRIVATE_KEY=0x0E][64 bytes: 32 private + 32 public]
	uint8_t rsp[65];
	rsp[0] = PACKET_PRIVATE_KEY;
	self_id.writeTo(&rsp[1], 64);
	writeFrame(rsp, sizeof(rsp));
	return true;
}

bool CompanionMesh::handleCmdImportPrivateKey(const uint8_t *data, size_t len)
{
	/* Import private key: [cmd][64 bytes prv_key] */
	if (len >= 1 + PRV_KEY_SIZE) {
		if (!mesh::LocalIdentity::validatePrivateKey(&data[1])) {
			writeErrFrame(ERR_ILLEGAL_ARG); /* invalid key */
		} else {
			mesh::LocalIdentity new_identity;
			new_identity.readFrom(&data[1], PRV_KEY_SIZE);
			if (_store->saveMainIdentity(new_identity)) {
				/* The v-contact key is derived from our identity, so it
				 * changes with the new key. Tell the connected app to drop
				 * the old v-contact (old key), re-derive, then re-advertise
				 * the new one — otherwise the app's cached loopback contact
				 * points at a key we no longer recognise (messaging + its
				 * advert-path query break until a reboot re-syncs). Order
				 * matters: delete uses the CURRENT (old) key. */
				bool vc_was_enabled = isVContactEnabled();
				if (vc_was_enabled) vcontactPushDeleted();
				self_id = new_identity;
				deriveVContactKey();
				/* New identity → old advert timestamp is meaningless. Clear
				 * it so vcontactPushAdvert() re-activates cleanly: with a
				 * valid clock it stamps + emits now; without one it defers
				 * and vcontactClockSynced() (the lastmod==0 path) emits at the
				 * next time-sync instead of only after a reboot. */
				_vcontact_lastmod = 0;
				/* Different key → different contact in the app; the old
				 * favourite/telemetry flags don't carry over. Persist
				 * before the push so a reboot can't resurrect them. */
				if (prefs.v_contact_flags != 0) {
					prefs.v_contact_flags = 0;
					_store->savePrefs(prefs);
				}
				if (vc_was_enabled) vcontactPushAdvert();
				/* Reload contacts to invalidate ECDH shared secrets */
				resetContacts();
				_store->loadContacts(this);
				writeOKFrame();
			} else {
				writeErrFrame(ERR_BAD_STATE);
			}
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendRawData(const uint8_t *data, size_t len)
{
	/* Raw data packet: [cmd][path_len][path...][payload...]
	 *
	 * path_len is the encoded 2-bit (hash_size - 1) + 6-bit hop count, not
	 * a byte count, so it has to be decoded rather than added to an offset.
	 * Treating it as a length rejected every path with a hash size above 1:
	 * hash_size 2 sets bit 6 (path_len 0x42 for two hops read as 66 bytes,
	 * failing the length check) and hash_size 3 sets bit 7, which used to
	 * read back as a negative int8_t. */
	if (len >= 6) {  /* min: cmd + path_len + 4 byte payload */
		size_t i = 1;
		uint8_t path_len = data[i++];
		if (mesh::Packet::isValidPathLen(path_len)) {
			uint8_t path[MAX_PATH_SIZE];
			/* Untrusted phone frame: bound the source at the bytes that
			 * actually remain, so a crafted path_len cannot over-read. */
			size_t path_bytes = mesh::Packet::writePath(path, &data[i], len - i, path_len);
			if (path_bytes == 0 && (path_len & 63) != 0) {
				/* Zero bytes written with a non-zero hop count (low 6
				 * bits): the decoded path runs past the end of the frame. */
				writeErrFrame(ERR_ILLEGAL_ARG);
				return true;
			}
			i += path_bytes;
			if (i + 4 > len) {  /* min payload 4 bytes */
				writeErrFrame(ERR_ILLEGAL_ARG);
			} else {
				mesh::Packet *pkt = createRawData(&data[i], len - i);
				if (pkt) {
					sendDirect(pkt, path, path_len);
					writeOKFrame();
				} else {
					writeErrFrame(ERR_TABLE_FULL);
				}
			}
		} else {
			/* Flood mode not supported */
			writeErrFrame(ERR_UNSUPPORTED);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendLogin(const uint8_t *data, size_t len)
{
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
			LOG_DBG("CMD_SEND_LOGIN: sending to contact, path_len=%d", contact->out_path_len);
			int result = sendLogin(*contact, password, est_timeout);
			LOG_DBG("CMD_SEND_LOGIN: sendLogin returned %d, est_timeout=%u", result, est_timeout);
			if (result != MSG_SEND_FAILED) {
				clearPendingReqs();
				memcpy(&_pending_login, contact->id.pub_key, 4);  // match this to onContactResponse()
				LOG_DBG("CMD_SEND_LOGIN: _pending_login set to %08x", _pending_login);
				sendPacketSent(result, _pending_login, est_timeout);
			} else {
				writeErrFrame(ERR_TABLE_FULL);
			}
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendAnonReq(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE + 1) {
		ContactInfo *contact = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
		if (contact == nullptr) {
			/* FIRMWARE_VER_CODE 13+: allow requests to a non-contact pubkey by
			 * creating a transient "anon" contact (ADV_TYPE_NONE). These are never
			 * persisted or synced to the app; re-lookup to get the stable slot. */
			ContactInfo anon{};
			memcpy(anon.id.pub_key, &data[1], PUB_KEY_SIZE);
			anon.out_path_len = 0;       // zero-hop direct by default
			anon.type = ADV_TYPE_NONE;   // transient/unknown
			anon.lastmod = getRTCClock()->getCurrentTime();  // so slot recycling is LRU, not always slot 0
			if (addContact(anon)) {
				contact = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
			}
		}
		if (contact) {
			uint32_t tag, est_timeout;
			int result = sendAnonReq(*contact, &data[1 + PUB_KEY_SIZE], len - 1 - PUB_KEY_SIZE, tag, est_timeout);
			if (result != MSG_SEND_FAILED) {
				clearPendingReqs();
				_pending_req = tag;
				sendPacketSent(result, tag, est_timeout);
			} else {
				writeErrFrame(ERR_BAD_STATE);
			}
		} else {
			writeErrFrame(ERR_TABLE_FULL);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendStatusReq(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE) {
		ContactInfo *contact = lookupContactByPubKey(&data[1], PUB_KEY_SIZE);
		if (contact) {
			uint32_t tag, est_timeout;
			int result = sendRequest(*contact, REQ_TYPE_GET_STATUS, tag, est_timeout);
			if (result != MSG_SEND_FAILED) {
				clearPendingReqs();
				memcpy(&_pending_status, contact->id.pub_key, 4);  // legacy matching scheme
				sendPacketSent(result, tag, est_timeout);
			} else {
				writeErrFrame(ERR_BAD_STATE);
			}
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendPathDiscoveryReq(const uint8_t *data, size_t len)
{
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
				sendPacketSent(result, tag, est_timeout);
			} else {
				writeErrFrame(ERR_BAD_STATE);
			}
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendTelemetryReq(const uint8_t *data, size_t len)
{
	if (len == 4) {
		// Self-telemetry request: return battery, GPS, and environment data
		// Format: Cayenne LPP: [channel][type][data...]
		// Response: [PUSH_CODE_TELEMETRY_RESPONSE][reserved][6-byte pubkey][telemetry_data]
		// Worst-case size tracks POWER_MAX_CHANNELS so a future bump can't
		// silently overflow this stack buffer. With current value 4:
		// header(8) + batt(4) + gps(11) + env(temp4+hum3+press4+lum4=15)
		// + power(POWER_MAX_CHANNELS * 12 = 48) + 8 byte safety pad = 94.
		uint8_t rsp[8 + 4 + 11 + 15 + (12 * POWER_MAX_CHANNELS) + 8];
		int i = 0;
		rsp[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
		rsp[i++] = 0;  // reserved
		memcpy(&rsp[i], self_id.pub_key, 6);
		i += 6;  // pubkey prefix

		// Self-telemetry is unconditional (all permission bits set).
		i += appendSelfTelemetry(&rsp[i], TELEM_PERM_BASE | TELEM_PERM_LOCATION | TELEM_PERM_ENVIRONMENT);
		writeFrame(rsp, i);
	} else if (len >= 4 + PUB_KEY_SIZE) {
		// Contact telemetry request: [cmd][3 reserved bytes][32-byte pubkey]
		ContactInfo *contact = lookupContactByPubKey(&data[4], PUB_KEY_SIZE);
		if (contact) {
			uint32_t tag, est_timeout;
			int result = sendRequest(*contact, REQ_TYPE_GET_TELEMETRY_DATA, tag, est_timeout);
			if (result != MSG_SEND_FAILED) {
				clearPendingReqs();
				_pending_telemetry = tag;
				sendPacketSent(result, tag, est_timeout);
			} else {
				writeErrFrame(ERR_BAD_STATE);
			}
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSendBinaryReq(const uint8_t *data, size_t len)
{
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
				sendPacketSent(result, tag, est_timeout);
			} else {
				writeErrFrame(ERR_BAD_STATE);
			}
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdHasConnection(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE) {
		if (hasConnectionTo(&data[1])) {
			writeOKFrame();
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdLogout(const uint8_t *data, size_t len)
{
	if (len >= 1 + PUB_KEY_SIZE) {
		stopConnection(&data[1]);
	}
	writeOKFrame();
	return true;
}

bool CompanionMesh::handleCmdGetChannel(const uint8_t *data, size_t len)
{
	// Format: [cmd][channel_idx]
	// Response: [code][idx][32 name][16 secret] = 50 bytes (matches Arduino)
	// Arduino returns channel info even if slot is empty (name[0]=='\0')
	if (len < 2 || data[1] >= MAX_GROUP_CHANNELS) {
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}
	{
		static int64_t last_get_channel_ms;
		int64_t now_ms = _ms->getMillis();
		uint32_t dt_ms = last_get_channel_ms ? (uint32_t)(now_ms - last_get_channel_ms) : 0;
		last_get_channel_ms = now_ms;
		LOG_DBG("CMD_GET_CHANNEL idx=%u dt=%ums", data[1], dt_ms);
		drainPendingChannelInfos();
		if (!enqueuePendingChannelInfo(data[1])) {
			writeErrFrame(ERR_BAD_STATE);
			return true;
		}
		drainPendingChannelInfos();
		return true;
	}
}

bool CompanionMesh::handleCmdSetChannel(const uint8_t *data, size_t len)
{
	// Format: [cmd][idx][32 name][16 or 32 secret]
	// Arduino rejects 32-byte secrets, but we accept 16-byte (50 bytes total)
	if (len < 2 || data[1] >= MAX_GROUP_CHANNELS) {
		writeErrFrame(ERR_ILLEGAL_ARG);
		return true;
	}
	if (len >= 2 + 32 + 32) {
		// 32-byte secret not supported (matches Arduino)
		writeErrFrame(ERR_UNSUPPORTED);
		return true;
	}
	if (len >= 2 + 32 + 16) {
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
			writeOKFrame();
		} else {
			writeErrFrame(ERR_NOT_FOUND);  // bad channel_idx
		}
		return true;
	}
	writeErrFrame(ERR_ILLEGAL_ARG);
	return true;
}

bool CompanionMesh::handleCmdSignStart(const uint8_t *data, size_t len)
{
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
		writeErrFrame(ERR_BAD_STATE);
	}
	return true;
}

bool CompanionMesh::handleCmdSignData(const uint8_t *data, size_t len)
{
	if (_sign_data && len > 1) {
		size_t chunk_len = len - 1;
		if (_sign_data_len + chunk_len <= _sign_data_capacity) {
			memcpy(&_sign_data[_sign_data_len], &data[1], chunk_len);
			_sign_data_len += chunk_len;
			writeOKFrame();
		} else {
			writeErrFrame(ERR_TABLE_FULL);
		}
	} else {
		writeErrFrame(ERR_BAD_STATE);
	}
	return true;
}

bool CompanionMesh::handleCmdSignFinish(const uint8_t *data, size_t len)
{
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
		writeErrFrame(ERR_BAD_STATE);
	}
	return true;
}

bool CompanionMesh::handleCmdSendTracePath(const uint8_t *data, size_t len)
{
	/* Trace path: [cmd][4-byte tag][4-byte auth][flags][path...] */
	if (len > 10 && (len - 10) < MAX_PACKET_PAYLOAD - 5) {
		uint8_t path_len = len - 10;
		uint8_t flags = data[9];
		uint8_t path_sz = flags & 0x03;  /* Path hash size encoding */
		/* Validate path length is multiple of hash size */
		if ((path_len >> path_sz) > MAX_PATH_SIZE || (path_len % (1 << path_sz)) != 0) {
			writeErrFrame(ERR_ILLEGAL_ARG);
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
				writeErrFrame(ERR_TABLE_FULL);
			}
		}
	} else if (len >= 9) {
		/* Legacy: zero-hop trace (backwards compat) */
		uint32_t tag = get_le32(&data[1]);
		uint32_t auth_code = get_le32(&data[5]);
		mesh::Packet *trace = createTrace(tag, auth_code);
		if (trace) {
			sendZeroHop(trace);
			writeOKFrame();
		} else {
			writeErrFrame(ERR_BAD_STATE);
		}
	} else {
		writeOKFrame();  /* Backwards compat */
	}
	return true;
}

bool CompanionMesh::handleCmdSetDevicePin(const uint8_t *data, size_t len)
{
	if (len >= 5) {
		uint32_t pin;
		memcpy(&pin, &data[1], 4);
		if (pin == 0 || (pin >= 100000 && pin <= 999999)) {
			prefs.ble_pin = pin;
			_store->savePrefs(prefs);
			if (_pin_change_cb) _pin_change_cb(pin);
			writeOKFrame();
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdGetCustomVars(const uint8_t *data, size_t len)
{
	// Return GPS and sensor settings as comma-separated key:value pairs
	// Format: [PACKET_CUSTOM_VARS][key1:val1,key2:val2,...]
	uint8_t rsp[64];
	char *dp = (char *)&rsp[1];
	char *const rsp_end = (char *)&rsp[sizeof(rsp)];
	rsp[0] = PACKET_CUSTOM_VARS;

	// snprintf returns the would-be length, NOT bytes actually written.
	// We must check truncation and only advance dp on real progress —
	// otherwise dp can outrun the initialized portion of rsp and
	// writeFrame(rsp, dp-rsp) leaks adjacent stack to the phone.

	bool first = true;
	if (gps_is_available()) {
		size_t remaining = (size_t)(rsp_end - dp);
		int n = snprintf(dp, remaining, "gps:%d", gps_is_enabled() ? 1 : 0);
		if (n > 0 && (size_t)n < remaining) {
			dp += n;
			first = false;
		}
	}
	uint32_t gps_interval = gps_get_poll_interval_sec();
	if (gps_interval > 0) {
		size_t remaining = (size_t)(rsp_end - dp);
		// Reserve 1 byte for the leading ',' if needed
		if (!first && remaining > 0) {
			*dp++ = ',';
			remaining--;
		}
		int n = snprintf(dp, remaining, "gps_interval:%u", (unsigned)gps_interval);
		if (n > 0 && (size_t)n < remaining) {
			dp += n;
			first = false;
		}
		// If snprintf would have truncated, dp stays put — writeFrame
		// sends only what we successfully wrote.
	}
	{
		size_t remaining = (size_t)(rsp_end - dp);
		if (!first && remaining > 0) {
			*dp++ = ',';
			remaining--;
		}
		int n = snprintf(dp, remaining, "meshtimesync:%d",
				 prefs.meshtimesync ? 1 : 0);
		if (n > 0 && (size_t)n < remaining) {
			dp += n;
		}
	}
	// Note: Environment sensors are auto-detected, no settings needed

	writeFrame(rsp, (size_t)(dp - (char *)rsp));
	return true;
}

bool CompanionMesh::handleCmdSetCustomVar(const uint8_t *data, size_t len)
{
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
				writeOKFrame();
			} else if (strcmp(key, "gps_interval") == 0) {
				uint32_t interval = (uint32_t)atoi(val);
				if (interval > 0 && interval <= 86400) {
					gps_set_poll_interval_sec(interval);
					/* Persist GPS interval across reboots */
					prefs.gps_interval = interval;
					_store->savePrefs(prefs);
					writeOKFrame();
				} else {
					writeErrFrame(ERR_ILLEGAL_ARG);
				}
			} else if (strcmp(key, "meshtimesync") == 0) {
				prefs.meshtimesync = (val[0] == '1') ? 1 : 0;
				_store->savePrefs(prefs);
				writeOKFrame();
			} else {
				writeErrFrame(ERR_ILLEGAL_ARG);
			}
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdGetAdvertPath(const uint8_t *data, size_t len)
{
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
			/* Trusted source: AdvertPath::path is MAX_PATH_SIZE-sized. */
			i += mesh::Packet::writePath(&rsp[i], ap->path, MAX_PATH_SIZE, ap->path_len);
			writeFrame(rsp, i);
		} else {
			writeErrFrame(ERR_NOT_FOUND);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdGetStats(const uint8_t *data, size_t len)
{
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
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdFactoryReset(const uint8_t *data, size_t len)
{
	// Arduino: requires "reset" magic string for safety
	if (len >= 6 && memcmp(&data[1], "reset", 5) == 0) {
		LOG_INF("Factory reset requested");
		_store->factoryReset();
		writeOKFrame();
		// Reboot to regenerate identity
		sys_reboot(SYS_REBOOT_COLD);
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSetFloodScopeKey(const uint8_t *data, size_t len)
{
	/* Set send scope mode (Arduino MeshCore PR #2492, ver 12+):
	 * [cmd][0][16-byte key] = explicit scoped override
	 * [cmd][0]             = clear scope override (use default_scope from prefs)
	 * [cmd][1]             = explicit unscoped (sticky flag, bypasses default_scope)
	 */
	if (len >= 2 && data[1] == 0) {
		if (len >= 2 + 16) {
			memcpy(_send_scope.key, &data[2], sizeof(_send_scope.key));
		} else {
			memset(_send_scope.key, 0, sizeof(_send_scope.key));
		}
		_send_scope_force_unscoped = false;
		writeOKFrame();
	} else if (len == 2 && data[1] == 1) {
		_send_scope_force_unscoped = true;
		memset(_send_scope.key, 0, sizeof(_send_scope.key));
		writeOKFrame();
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSetDefaultFloodScope(const uint8_t *data, size_t len)
{
	/* Set default flood scope: [cmd][name:31][key:16] or [cmd] alone (clear) */
	if (len >= 1 + 31 + 16) {
		int n = strnlen((const char *)&data[1], 31);
		if (n > 0 && n < 31) {
			memset(prefs.default_scope_name, 0, sizeof(prefs.default_scope_name));
			memcpy(prefs.default_scope_name, &data[1], n);
			memcpy(prefs.default_scope_key, &data[1 + 31], 16);
			_store->savePrefs(prefs);
			writeOKFrame();
		} else {
			writeErrFrame(ERR_ILLEGAL_ARG);
		}
	} else {
		memset(prefs.default_scope_name, 0, sizeof(prefs.default_scope_name));
		memset(prefs.default_scope_key, 0, sizeof(prefs.default_scope_key));
		_store->savePrefs(prefs);
		writeOKFrame();
	}
	return true;
}

bool CompanionMesh::handleCmdGetDefaultFloodScope(const uint8_t *data, size_t len)
{
	uint8_t rsp[1 + 31 + 16];
	rsp[0] = PACKET_DEFAULT_FLOOD_SCOPE;
	if (strlen(prefs.default_scope_name) > 0) {
		memcpy(&rsp[1], prefs.default_scope_name, 31);
		memcpy(&rsp[1 + 31], prefs.default_scope_key, 16);
		writeFrame(rsp, sizeof(rsp));
	} else {
		writeFrame(rsp, 1);  /* no name or key means null */
	}
	return true;
}

bool CompanionMesh::handleCmdSendControlData(const uint8_t *data, size_t len)
{
	/* Control data: [cmd][flags | 0x80][data...] */
	if (len >= 2 && (data[1] & 0x80) != 0) {
		mesh::Packet *pkt = createControlData(&data[1], len - 1);
		if (pkt) {
			sendZeroHop(pkt);
			writeOKFrame();
		} else {
			writeErrFrame(ERR_TABLE_FULL);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}

bool CompanionMesh::handleCmdSetAutoaddConfig(const uint8_t *data, size_t len)
{
	if (len >= 2) {
		prefs.autoadd_config = data[1];
		if (len >= 3) {
			prefs.autoadd_max_hops = (data[2] > 64) ? 64 : data[2];
		}
		LOG_INF("SET_AUTOADD_CONFIG: autoadd_config=0x%02x max_hops=%u",
			prefs.autoadd_config, prefs.autoadd_max_hops);
		_store->savePrefs(prefs);
	}
	writeOKFrame();
	return true;
}

bool CompanionMesh::handleCmdGetAutoaddConfig(const uint8_t *data, size_t len)
{
	uint8_t rsp[3];
	rsp[0] = PACKET_AUTOADD_CONFIG;
	rsp[1] = prefs.autoadd_config;
	rsp[2] = prefs.autoadd_max_hops;
	writeFrame(rsp, sizeof(rsp));
	return true;
}

bool CompanionMesh::handleCmdGetAllowedRepeatFreq(const uint8_t *data, size_t len)
{
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

bool CompanionMesh::handleCmdSendRawPacket(const uint8_t *data, size_t len)
{
	if (len >= 4) {
		mesh::Packet *pkt = obtainNewPacket();
		if (pkt) {
			uint8_t priority = data[1];
			if (tryParsePacket(pkt, &data[2], len - 2)) {
				sendPacket(pkt, priority, 0);
				writeOKFrame();
			} else {
				releasePacket(pkt);
				writeErrFrame(ERR_ILLEGAL_ARG);
			}
		} else {
			writeErrFrame(ERR_TABLE_FULL);
		}
	} else {
		writeErrFrame(ERR_ILLEGAL_ARG);
	}
	return true;
}
