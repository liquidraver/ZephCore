/*
 * SPDX-License-Identifier: MIT
 * The v-contact ("v<node_name>"): a chat contact that exists only toward the
 * connected app. Messages to it run the companion CLI; replies and notices
 * (battery alert, restart reason) come back as its messages via the offline
 * queue. Invariants:
 *   - it never enters the contacts table, so never the RF RX matching path;
 *   - it is intercepted before any send path, so no packet is ever created;
 *   - nobody holds its private key, so over-the-air traffic to it is inert.
 */

#include "CompanionMesh.h"
#include "CompanionProtocol.h"
#include <mesh/Utils.h>
#include <ZephyrSensorManager.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zephcore_companion, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

void CompanionMesh::buildVContact(ContactInfo &c) const
{
	memcpy(c.id.pub_key, _vcontact_pubkey, PUB_KEY_SIZE);
	c.type = ADV_TYPE_CHAT;
	/* The app owns the flags (favourite, telemetry permissions); with no table
	 * record they live in prefs. Echoing 0 cleared the favourite on every sync. */
	c.flags = prefs.v_contact_flags;
	c.out_path_len = 0;  /* zero-hop direct — renders as "0 hops" in the app */
	c.shared_secret_valid = false;
	memset(c.out_path, 0, sizeof(c.out_path));
	c.name[0] = 'v';
	StrHelper::strzcpy(&c.name[1], prefs.node_name, sizeof(c.name) - 1);
	c.last_advert_timestamp = _vcontact_lastmod;
	c.lastmod = _vcontact_lastmod;
	c.gps_lat = 0;
	c.gps_lon = 0;
	c.sync_since = 0;
}

bool CompanionMesh::isVContactKey(const uint8_t *key, int prefix_len) const
{
	if (!isVContactEnabled()) return false;
	if (prefix_len > PUB_KEY_SIZE) prefix_len = PUB_KEY_SIZE;
	return memcmp(key, _vcontact_pubkey, prefix_len) == 0;
}

bool CompanionMesh::vcontactClockValid()
{
	/* Anything before the firmware build epoch is a never-synced clock. */
	return (uint32_t)getRTCClock()->getCurrentTime() >= (uint32_t)FIRMWARE_BUILD_EPOCH;
}

void CompanionMesh::vcontactClockSynced()
{
	if (!isVContactEnabled() || !vcontactClockValid()) {
		return;
	}
	if (_vcontact_lastmod == 0) {
		/* Deferred activation: first valid time source — stamp and announce.
		 * From here the contact also appears in CMD_GET_CONTACTS syncs. */
		vcontactPushAdvert();
	}
	/* Flush the notices held for a valid clock, now with real timestamps. */
	for (uint8_t i = 0; i < _vcontact_pending_count; i++) {
		vcontactQueueText(_vcontact_pending[i]);
	}
	_vcontact_pending_count = 0;
}

void CompanionMesh::vcontactQueueText(const char *text)
{
	/* A queued frame caps at 172 bytes and the V3 header takes 16: split into
	 * <=150-char messages, preferring line breaks. getCurrentTimeUnique() keeps
	 * their timestamps increasing so the app orders them. */
	static const size_t CHUNK_MAX = 150;
	ContactInfo vc;
	buildVContact(vc);

	const char *p = text;
	size_t remaining = strlen(text);
	while (remaining > 0) {
		size_t take = remaining;
		if (take > CHUNK_MAX) {
			take = CHUNK_MAX;
			for (size_t i = take; i > CHUNK_MAX / 2; i--) {
				if (p[i - 1] == '\n') { take = i; break; }
			}
		}
		char chunk[CHUNK_MAX + 1];
		memcpy(chunk, p, take);
		chunk[take] = '\0';
		size_t adv = take;
		for (size_t i = 0; i < take; i++) {
			if (chunk[i] == '\r') chunk[i] = ' ';  /* CRLF CLI output → LF */
		}
		while (take > 0 && (chunk[take - 1] == '\n' || chunk[take - 1] == ' ')) {
			chunk[--take] = '\0';  /* trim trailing break of this bubble */
		}
		if (take > 0) {
			queueMessage(vc, TXT_TYPE_PLAIN, nullptr,
				getRTCClock()->getCurrentTimeUnique(), nullptr, 0, chunk);
		}
		p += adv;
		remaining -= adv;
	}
	/* No prompt while held (the app's own initial sync drains the queue) or
	 * while the app has hidden the v-contact this session; the messages stay
	 * queued either way. */
	if (!vcontactMsgWaitHeld() && !_vcontact_app_hidden) {
		pushMsgWaiting();
	} else {
		LOG_INF("msgwait: suppressed (hold=%d hidden=%d), %d queued",
			(int)_vcontact_hold_msgwait, (int)_vcontact_app_hidden,
			_offline_queue_count);
	}
}

void CompanionMesh::vcontactNotify(const char *text)
{
	if (!isVContactEnabled() || !text || !text[0]) return;
	LOG_INF("vcontact notify: %s", text);
	if (!vcontactClockValid()) {
		/* Would show as 1970: hold it for vcontactClockSynced(). Drop-oldest
		 * (restart reason + battery is the whole expected population). */
		if (_vcontact_pending_count >= (uint8_t)ARRAY_SIZE(_vcontact_pending)) {
			memmove(_vcontact_pending[0], _vcontact_pending[1],
				sizeof(_vcontact_pending[0]) * (ARRAY_SIZE(_vcontact_pending) - 1));
			_vcontact_pending_count = ARRAY_SIZE(_vcontact_pending) - 1;
		}
		strncpy(_vcontact_pending[_vcontact_pending_count], text,
			sizeof(_vcontact_pending[0]) - 1);
		_vcontact_pending[_vcontact_pending_count][sizeof(_vcontact_pending[0]) - 1] = '\0';
		_vcontact_pending_count++;
		return;
	}
	vcontactQueueText(text);
}

/* How long SEND_CONFIRMED is held after its PACKET_SENT. A timer, not "the
 * next inbound frame": the app answers our MSG_WAITING within one connection
 * interval, which would flush it almost as early as inline. Must land well
 * inside the 3000 ms est_timeout, or the app's resend races it. */
#define VCONTACT_CONFIRM_DELAY_MS 300

void CompanionMesh::vcontactConfirmWorkHandler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	ConfirmWork *cw = CONTAINER_OF(dwork, ConfirmWork, work);
	cw->self->vcontactFlushConfirm();
}

void CompanionMesh::vcontactFlushConfirm()
{
	/* No lock: a race can at worst emit the (idempotent) push twice. */
	uint32_t ack = _vcontact_confirm_ack;
	if (ack == 0) {
		return;
	}
	_vcontact_confirm_ack = 0;
	k_work_cancel_delayable(&_vcontact_confirm_work.work);

	uint8_t ack_push[8];
	memcpy(ack_push, &ack, 4);
	memset(&ack_push[4], 0, 4);        /* trip time: 0 ms — never left the box */
	LOG_DBG("vcontact: emitting deferred ack 0x%08x", ack);
	sendPush(PUSH_CODE_SEND_CONFIRMED, ack_push, 8);
}

void CompanionMesh::deriveVContactKey()
{
	/* The Ed25519 public point of a keypair seeded from
	 * SHA256("zc-vcontact" || self prv_key || counter).
	 *  - A real point: strict clients decompress peer keys and reject the ~50%
	 *    of random 32-byte strings that are not valid points.
	 *  - Seeded from the private key, so nobody else can link the v-contact to
	 *    this node or derive its private half, which is dropped here unused.
	 *  - The counter skips pub_key[0] of 0x00/0xFF, which MeshCore reserves
	 *    (P = 2/256 per try, so still deterministic). */
	static const char vc_salt[] = "zc-vcontact";
	uint8_t material[PRV_KEY_SIZE + 1];
	uint8_t seed[SEED_SIZE];
	mesh::LocalIdentity vc;

	/* writeTo() writes prv || pub; max_len PRV_KEY_SIZE asks for prv alone. */
	if (self_id.writeTo(material, PRV_KEY_SIZE) != PRV_KEY_SIZE) {
		/* Cannot happen; fails loudly instead of seeding off stack garbage. */
		LOG_ERR("vcontact: private key unavailable, key not derived");
		mesh::Utils::secureZeroize(material, sizeof(material));
		return;
	}

	for (int counter = 0; counter < 256; counter++) {
		material[PRV_KEY_SIZE] = (uint8_t)counter;
		mesh::Utils::sha256(seed, SEED_SIZE,
			(const uint8_t *)vc_salt, sizeof(vc_salt) - 1,
			material, sizeof(material));
		vc.fromSeed(seed);
		if (vc.pub_key[0] != 0x00 && vc.pub_key[0] != 0xFF) break;
	}
	memcpy(_vcontact_pubkey, vc.pub_key, PUB_KEY_SIZE);

	/* Wipe the identity key and the discarded private half. LocalIdentity has
	 * no wipe and no virtuals, so it is cleared wholesale. */
	mesh::Utils::secureZeroize(material, sizeof(material));
	mesh::Utils::secureZeroize(seed, sizeof(seed));
	mesh::Utils::secureZeroize(&vc, sizeof(vc));
}

void CompanionMesh::vcontactPushAdvert()
{
	if (!isVContactEnabled()) return;
	/* Hidden by the app this session; it returns at the next CMD_APP_START. */
	if (_vcontact_app_hidden) return;
	if (!vcontactClockValid()) {
		/* Deferred to vcontactClockSynced(); now it would say 1970. */
		return;
	}
	/* Bump lastmod so incremental syncs pick up the rename or re-enable. */
	_vcontact_lastmod = (uint32_t)getRTCClock()->getCurrentTime();
	ContactInfo vc;
	buildVContact(vc);
	uint8_t rsp[CONTACT_FRAME_SIZE];
	size_t n = serializeContact(rsp, vc);  /* no header — push code is separate */
	sendPush(PUSH_CODE_NEW_ADVERT, rsp, n);
}

void CompanionMesh::vcontactPushDeleted()
{
	sendPush(PUSH_CODE_CONTACT_DELETED, _vcontact_pubkey, PUB_KEY_SIZE);
}

bool CompanionMesh::vcontactHandleFrame(const uint8_t *data, size_t len)
{
	if (!isVContactEnabled()) return false;

	switch (data[0]) {
	case CMD_SEND_TXT_MSG:
		/* cmd(1) + txt_type(1) + attempt(1) + timestamp(4) + pub_key_prefix(6) + text(N) */
		if (len >= 14 && isVContactKey(&data[7], 6)) {
			uint8_t txt_type = data[1];
			if (txt_type != TXT_TYPE_PLAIN && txt_type != TXT_TYPE_CLI_DATA &&
			    txt_type != TXT_TYPE_CLI_COMMAND) {
				writeErrFrame(ERR_UNSUPPORTED);
				return true;
			}
			uint32_t msg_timestamp = get_le32(&data[3]);

			char line[MAX_TEXT_LEN + 1];
			size_t text_len = len - 13;
			if (text_len > MAX_TEXT_LEN) text_len = MAX_TEXT_LEN;
			memcpy(line, &data[13], text_len);
			line[text_len] = '\0';

			/* The delivery ack the app derives for this (timestamp, attempt,
			 * text), as composeMsgPacket() computes it; a random one made the
			 * app resend. */
			uint8_t hbuf[5 + MAX_TEXT_LEN];
			memcpy(hbuf, &data[3], 4);          /* timestamp, on-wire LE bytes */
			hbuf[4] = (data[2] & 3);            /* attempt & 3 */
			memcpy(&hbuf[5], line, text_len);
			uint32_t ack = 0;
			mesh::Utils::sha256((uint8_t *)&ack, 4, hbuf, 5 + text_len,
				self_id.pub_key, PUB_KEY_SIZE);
			if (ack == 0) ack = 1;

			sendPacketSent(MSG_SEND_SENT_DIRECT, ack, 3000);
			/* One slot: flush a still-pending confirm first (it is past the
			 * window that needed deferring) rather than overwrite it. */
			vcontactFlushConfirm();
			_vcontact_confirm_ack = ack;
			k_work_reschedule(&_vcontact_confirm_work.work,
					  K_MSEC(VCONTACT_CONFIRM_DELAY_MS));

			/* A resend reuses the timestamp: re-ack it (done above) without
			 * re-running the CLI. */
			bool dup = false;
			if (msg_timestamp != 0) {
				for (uint8_t i = 0; i < VCONTACT_DEDUP_SLOTS; i++) {
					if (_vcontact_recent_ts[i] == msg_timestamp) { dup = true; break; }
				}
				if (!dup) {
					_vcontact_recent_ts[_vcontact_recent_head] = msg_timestamp;
					_vcontact_recent_head =
						(_vcontact_recent_head + 1) % VCONTACT_DEDUP_SLOTS;
				}
			}

			if (!dup) {
				/* Undo a phone keyboard's autocapitalisation ("Get cad"), on
				 * character 0 only: it is always inside the command verb.
				 * Folding more once lowercased admin passwords. After the ack
				 * hash, which covers the original text. */
				if (line[0] >= 'A' && line[0] <= 'Z') {
					line[0] = (char)(line[0] - 'A' + 'a');
				}

				LOG_INF("vcontact CLI: '%s'", line);
				char reply[COMPANION_CLI_PREFIX_ROOM + COMPANION_CLI_REPLY_SIZE];
				handleCommand(line, 0, reply);
				if (reply[0] != '\0') {
					vcontactQueueText(reply);
				}
			} else {
				LOG_DBG("vcontact CLI: dup ts=%u, re-ack only", msg_timestamp);
			}
			return true;
		}
		return false;

	case CMD_GET_CONTACT_BY_KEY:
		if (len >= 1 + PUB_KEY_SIZE && isVContactKey(&data[1], PUB_KEY_SIZE)) {
			ContactInfo vc;
			buildVContact(vc);
			uint8_t rsp[CONTACT_FRAME_SIZE];
			size_t n = serializeContact(rsp, vc, PACKET_CONTACT);
			writeFrame(rsp, n);
			return true;
		}
		return false;

	case CMD_GET_ADVERT_PATH:
		/* It has no over-the-air advert, and the app treats ERR_NOT_FOUND here
		 * as fatal. It is this node, so the path is direct.
		 * [cmd][reserved][7-byte pubkey prefix] */
		if (len >= 2 + 7 && isVContactKey(&data[2], 7)) {
			uint8_t rsp[6];
			size_t i = 0;
			rsp[i++] = PACKET_ADVERT_PATH;
			put_le32(&rsp[i], _vcontact_lastmod ? _vcontact_lastmod
				: (uint32_t)getRTCClock()->getCurrentTime()); i += 4;
			rsp[i++] = 0;  // path_len = 0 → direct (zero hop)
			writeFrame(rsp, i);
			return true;
		}
		return false;

	case CMD_ADD_UPDATE_CONTACT:
		/* Never into the contacts table; keep only the flags byte the app owns.
		 * [cmd][32-byte pubkey][type][flags][...] */
		if (len >= 1 + PUB_KEY_SIZE && isVContactKey(&data[1], PUB_KEY_SIZE)) {
			if (len >= 1 + PUB_KEY_SIZE + 2) {
				uint8_t flags = data[1 + PUB_KEY_SIZE + 1];
				if (flags != prefs.v_contact_flags) {
					prefs.v_contact_flags = flags;
					_store->savePrefs(prefs);
				}
			}
			writeOKFrame();
			return true;
		}
		return false;

	case CMD_RESET_PATH:
		/* Meaningless for a loopback contact: accept and drop. */
		if (len >= 1 + PUB_KEY_SIZE && isVContactKey(&data[1], PUB_KEY_SIZE)) {
			writeOKFrame();
			return true;
		}
		return false;

	case CMD_REMOVE_CONTACT:
		/* Hide for this session only (see _vcontact_app_hidden); flags are kept
		 * so the favourite returns with it. */
		if (len >= 1 + PUB_KEY_SIZE && isVContactKey(&data[1], PUB_KEY_SIZE)) {
			_vcontact_app_hidden = true;
			LOG_INF("vcontact: hidden by app delete (this session only)");
			writeOKFrame();
			return true;
		}
		return false;

	case CMD_SEND_TELEMETRY_REQ:
		/* [cmd][3 reserved][32-byte pubkey]. Its telemetry is our own: answer
		 * the SENT the app waits on, then push a TELEMETRY_RESPONSE under the
		 * v-contact key. */
		if (len >= 4 + PUB_KEY_SIZE && isVContactKey(&data[4], PUB_KEY_SIZE)) {
			uint32_t tag = 0;
			getRNG()->random((uint8_t *)&tag, 4);
			if (tag == 0) tag = 1;
			sendPacketSent(MSG_SEND_SENT_DIRECT, tag, 3000);

			uint8_t rsp[8 + 4 + 11 + 15 + (12 * POWER_MAX_CHANNELS) + 8];
			int i = 0;
			rsp[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
			rsp[i++] = 0;  /* reserved */
			memcpy(&rsp[i], _vcontact_pubkey, 6);
			i += 6;
			i += appendSelfTelemetry(&rsp[i],
				TELEM_PERM_BASE | TELEM_PERM_LOCATION | TELEM_PERM_ENVIRONMENT);
			sendPush(rsp[0], &rsp[1], i - 1);
			return true;
		}
		return false;

	default:
		/* Every other pubkey-addressed opcode looks the contact up in the table,
		 * misses, and fails with ERR_NOT_FOUND before any packet exists. */
		return false;
	}
}
