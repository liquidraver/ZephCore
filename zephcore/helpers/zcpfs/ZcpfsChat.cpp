/*
 * SPDX-License-Identifier: MIT
 * zcpfs for the chat client. See ZcpfsChat.h.
 */

#include "ZcpfsChat.h"
#include "ZcpfsCommon.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

#include <helpers/AdvertDataHelpers.h>
#include <helpers/BaseChatMesh.h>
#include <helpers/TxtDataHelpers.h>

/* A handshake 1 that follows a path packet waits this long, so that the path
 * packet, which has the lower priority in the send queue, leaves first. */
#define HS_AFTER_PATH_MS 200

/* Delay of an ACK or a path return, as BaseChatMesh's TXT_ACK_DELAY. */
#define ACK_DELAY_MS 200

/* Engine time: uptime seconds. The mesh's millisecond clock wraps after 49
 * days; this one does not. */
uint32_t ZcpfsChat::now()
{
	return (uint32_t)(k_uptime_get() / 1000);
}

/* Set the engine up for the node's identity, again if that has changed. */
void ZcpfsChat::ensure()
{
	if (_ready && memcmp(_S, _m.self_id.pub_key, PUB_KEY_SIZE) == 0) {
		return;
	}

	uint8_t keys[PRV_KEY_SIZE + PUB_KEY_SIZE];
	uint8_t hw[32], more[32];

	_m.self_id.writeTo(keys, sizeof(keys));
	memcpy(_a, keys, sizeof(_a));		/* SPEC 4: the first half of the private key */
	memcpy(_S, _m.self_id.pub_key, PUB_KEY_SIZE);
	zcpfs_wipe(keys, sizeof(keys));

	_m.getRNG()->random(hw, sizeof(hw));
	_m.getRNG()->random(more, sizeof(more));
	zcpfs_drbg_seed(&_drbg, hw, more);
	zcpfs_wipe(hw, sizeof(hw));
	zcpfs_wipe(more, sizeof(more));

	_ops.send = opSend;
	_ops.reciprocal_path = opReciprocalPath;
	_ops.pin = opPin;
	_ops.contact = opContact;
	_ops.granted = nullptr;
	zcpfs_node_init(&_node, _S, _a, &_drbg, _peers, CONFIG_ZEPHCORE_ZCPFS_PEERS, &_ops, this);
	_in_kind = nullptr;
	_rx_ack = false;
	_back_valid = false;
	_ready = true;
}

/* What the node would wait for the answer to a packet of this payload length
 * sent to the contact now (SPEC 10.4). */
uint32_t ZcpfsChat::timeoutFor(const ContactInfo &c, size_t payload_len)
{
	if (c.out_path_len == OUT_PATH_UNKNOWN) {
		return _m.calcFloodTimeoutMillisFor(_m._radio->getEstAirtimeFor((int)(2 + payload_len)));
	}

	size_t path_bytes = (size_t)(c.out_path_len & 63) * ((c.out_path_len >> 6) + 1);

	return _m.calcDirectTimeoutMillisFor(
		_m._radio->getEstAirtimeFor((int)(2 + path_bytes + payload_len)), c.out_path_len);
}

zcpfs_contact ZcpfsChat::view(const ContactInfo &c)
{
	zcpfs_contact v;

	v.S = c.id.pub_key;
	v.ss = c.getSharedSecret(_m.self_id);
	/* passive: no contact counts as pinned, so legacy is read from all and
	 * nothing unopened starts a handshake (8.17) */
	v.pinned = on() && (c.flags & ZCPFS_PIN_FLAG) != 0;
	v.has_path = c.out_path_len != OUT_PATH_UNKNOWN;
	v.timeout_ms = timeoutFor(c, 2 + ZCPFS_HS_BODY_LEN);
	return v;
}

/* The length legacy decryption would have produced: whole AES blocks; the
 * plaintext is followed by zero bytes up to there (SPEC 7.1). */
static int legacyLen(size_t len)
{
	return (int)((len + CIPHER_BLOCK_SIZE - 1) / CIPHER_BLOCK_SIZE * CIPHER_BLOCK_SIZE);
}

/* ---- the engine's callbacks ---------------------------------------------- */

void ZcpfsChat::opSend(void *user, const uint8_t S[32], uint8_t type, const uint8_t *payload,
		       size_t len, int flood)
{
	ZcpfsChat &z = *static_cast<ZcpfsChat *>(user);
	ContactInfo *c = z._m.lookupContactByPubKey(S, PUB_KEY_SIZE);
	mesh::Packet *pkt = c != nullptr ? z._m.obtainNewPacket() : nullptr;

	if (pkt == nullptr) {
		return;
	}
	if (type == PAYLOAD_TYPE_TXT_MSG) {
		len == ZCPFS_DATA_OVERHEAD ? z._n.tx_confirm++ : z._n.tx_text++;
	} else if (type == PAYLOAD_TYPE_ANON_REQ) {
		z._n.tx_login++;
	} else if (type == PAYLOAD_TYPE_RESPONSE) {
		z._n.tx_hs2++;
	} else {
		len == 2 + ZCPFS_HS_BODY_LEN ? z._n.tx_hs1++ : z._n.tx_probe++;
	}
	pkt->header = (uint8_t)(type << PH_TYPE_SHIFT);		/* the route type is set by the send */
	memcpy(pkt->payload, payload, len);
	pkt->payload_len = (uint16_t)len;

	if (flood) {
		z._m.sendFloodScoped(*c, pkt);
	} else {
		uint32_t delay = 0;

		if (type == PAYLOAD_TYPE_REQ && z._hs_after_path) {
			delay = HS_AFTER_PATH_MS;
		}
		z._m.sendDirect(pkt, c->out_path, c->out_path_len, delay);
	}
	z._hs_after_path = false;
}

void ZcpfsChat::opReciprocalPath(void *user, const uint8_t S[32])
{
	ZcpfsChat &z = *static_cast<ZcpfsChat *>(user);
	ContactInfo *c = z._m.lookupContactByPubKey(S, PUB_KEY_SIZE);

	if (c == nullptr || c->out_path_len == OUT_PATH_UNKNOWN || !z._back_valid ||
	    memcmp(z._back_S, S, PUB_KEY_SIZE) != 0) {
		return;
	}
	/* what Mesh sends after a flooded path return, once more */
	mesh::Packet *rpath = z._m.createPathReturn(c->id, c->getSharedSecret(z._m.self_id),
						    z._back_path, z._back_len, 0, NULL, 0);
	if (rpath != nullptr) {
		z._m.sendDirect(rpath, c->out_path, c->out_path_len, 0);
		z._hs_after_path = true;
	}
}

void ZcpfsChat::opPin(void *user, const uint8_t S[32])
{
	ZcpfsChat &z = *static_cast<ZcpfsChat *>(user);
	ContactInfo *c = z._m.lookupContactByPubKey(S, PUB_KEY_SIZE);

	if (c != nullptr && !(c->flags & ZCPFS_PIN_FLAG)) {
		c->flags |= ZCPFS_PIN_FLAG;
		z._m.onContactPathUpdated(*c);	/* the contact changed: have it saved */
	}
}

int ZcpfsChat::opContact(void *user, const uint8_t S[32], zcpfs_contact *out)
{
	ZcpfsChat &z = *static_cast<ZcpfsChat *>(user);
	const ContactInfo *c = z._m.lookupContactByPubKey(S, PUB_KEY_SIZE);

	if (c == nullptr) {
		return 0;
	}
	*out = z.view(*c);
	return 1;
}

/* ---- receiving ----------------------------------------------------------- */

/* Acknowledge a text without delivering it again (SPEC 9.2, duplicates). */
void ZcpfsChat::ackOnly(mesh::Packet *pkt, ContactInfo &from, const uint8_t ack4[4])
{
	if (pkt->isRouteFlood()) {
		mesh::Packet *path = _m.createPathReturn(from.id, from.getSharedSecret(_m.self_id),
							 pkt->path, pkt->path_len, PAYLOAD_TYPE_ACK,
							 ack4, 4);
		if (path != nullptr) {
			_m.sendFloodScoped(from, path, ACK_DELAY_MS);
		}
	} else {
		_m.sendAckTo(from, ack4, 4);
	}
}

int ZcpfsChat::rx(mesh::Packet *pkt, ContactInfo &from, uint8_t *data, int *len)
{
	static zcpfs_rx_out out;	/* 196 bytes; the mesh runs on one thread */

	BUILD_ASSERT(sizeof(out.plain) == MAX_PACKET_PAYLOAD);
	if (off()) {
		return mesh::Mesh::SEALED_LEGACY;	/* as firmware without zcpfs */
	}
	ensure();
	_in_kind = nullptr;
	_in_flood = pkt->isRouteFlood();
	_rx_ack = false;

	zcpfs_contact v = view(from);
	int r = zcpfs_rx(&_node, &v, pkt->getPayloadType(), pkt->payload, pkt->payload_len,
			 pkt->isRouteFlood(), now(), &out);
	_next_tick = 0;
	if (r != ZCPFS_RX_LEGACY && r != ZCPFS_RX_NEXT) {
		_heard.mark(from.id.pub_key, now());
	}

	switch (r) {
	case ZCPFS_RX_LEGACY:
		return mesh::Mesh::SEALED_LEGACY;
	case ZCPFS_RX_NEXT:
		return mesh::Mesh::SEALED_NEXT;
	case ZCPFS_RX_DUPLICATE: {
		uint8_t txt_type = (uint8_t)(out.plain[4] >> 2);

		_in_kind = &from;
		_n.rx_dup++;
		if (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_SIGNED_PLAIN) {
			ackOnly(pkt, from, out.ack4);	/* the kinds legacy acknowledges */
		}
		zcpfs_wipe(&out, sizeof(out));
		return mesh::Mesh::SEALED_CONSUMED;
	}
	case ZCPFS_RX_PROBE: {
		/* as a flooded message without an ACK is answered (SPEC 11) */
		_in_kind = &from;
		_n.rx_probe++;
		mesh::Packet *path = _m.createPathReturn(from.id, from.getSharedSecret(_m.self_id),
							 pkt->path, pkt->path_len, 0, NULL, 0);
		if (path != nullptr) {
			_m.sendFloodScoped(from, path, ACK_DELAY_MS);
		}
		return mesh::Mesh::SEALED_CONSUMED;
	}
	case ZCPFS_RX_TEXT:
		memcpy(_rx_ack4, out.ack4, sizeof(_rx_ack4));
		_rx_ack = true;
		[[fallthrough]];
	case ZCPFS_RX_PAIR:
		/* the legacy view (SPEC 7.1): the plaintext, then zero bytes */
		_in_kind = &from;
		r == ZCPFS_RX_TEXT ? _n.rx_text++ : _n.rx_pair++;
		memcpy(data, out.plain, sizeof(out.plain));
		*len = legacyLen(out.len);
		zcpfs_wipe(&out, sizeof(out));
		return mesh::Mesh::SEALED_DATA;
	case ZCPFS_RX_LOGIN:
		/* the reply of a login, as a RESPONSE packet (SPEC 19.4) */
		_n.rx_login++;
		legacyLoginResponse(from, out.plain, data);
		*len = CIPHER_BLOCK_SIZE;
		zcpfs_wipe(&out, sizeof(out));
		return mesh::Mesh::SEALED_DATA;
	default:
		return mesh::Mesh::SEALED_CONSUMED;
	}
}

/* The 13-byte legacy login response the application expects, from the 6
 * bytes of a zcpfs one (SPEC 19.3): the rest is constant or filler. */
void ZcpfsChat::legacyLoginResponse(const ContactInfo &server, const uint8_t *r, uint8_t *data)
{
	uint8_t perms = r[4];
	bool admin = (perms & 3) == 3;		/* PERM_ACL_ADMIN in the role bits */

	memset(data, 0, CIPHER_BLOCK_SIZE);
	memcpy(data, r, 4);			/* the server's time */
	data[4] = RESP_SERVER_LOGIN_OK;
	data[5] = 0;				/* no keep-alive, as both servers send */
	data[6] = admin ? 1 : (server.type == ADV_TYPE_ROOM && perms == 0 ? 2 : 0);
	data[7] = perms;
	_m.getRNG()->random(&data[8], 4);
	data[12] = r[5];			/* firmware level */
}

bool ZcpfsChat::pathResponse(const ContactInfo &from, uint8_t *extra, uint8_t &extra_len)
{
	/* A login reply comes sealed, starts with its length (SPEC 18.1) and
	 * then a branch count, and only while a login to this server waits. */
	if (off() || !_ready || _in_kind != &from || !_node.login.used ||
	    memcmp(_node.login.S, from.id.pub_key, PUB_KEY_SIZE) != 0 || extra_len < 2 ||
	    extra[1] < 1 || extra[1] > ZCPFS_LOGIN_MAX_BRANCHES ||
	    extra[0] < 1 + extra[1] * ZCPFS_BRANCH_LEN || extra[0] > extra_len - 1) {
		return true;
	}

	static zcpfs_rx_out out;
	zcpfs_contact v = view(from);
	_n.login_path = from.out_path_len;
	int r = zcpfs_rx_login_reply(&_node, &v, extra + 1, extra[0], _in_flood, now(), &out);

	_next_tick = 0;
	if (r != ZCPFS_RX_LOGIN) {
		return false;			/* no branch opened: a wrong password (19.4) */
	}
	_n.rx_login++;
	legacyLoginResponse(from, out.plain, extra);
	extra_len = 13;
	zcpfs_wipe(&out, sizeof(out));
	return true;
}

void ZcpfsChat::rxDone()
{
	_in_kind = nullptr;
	_rx_ack = false;
}

bool ZcpfsChat::takeAck(uint8_t ack4[4])
{
	if (!_rx_ack) {
		return false;
	}
	memcpy(ack4, _rx_ack4, 4);
	_rx_ack = false;
	return true;
}

/* ---- sending ------------------------------------------------------------- */

int ZcpfsChat::sealOutgoing(uint8_t type, const uint8_t *dest_hash, const uint8_t *secret,
			    const uint8_t *data, size_t data_len, uint8_t *payload)
{
	const ContactInfo *c = nullptr;

	if (off()) {
		return 0;
	}
	ensure();
	for (int i = 0; i < _m.num_contacts; i++) {
		const ContactInfo &k = _m.contacts[i];

		if (k.id.isHashMatch(dest_hash) &&
		    memcmp(k.getSharedSecret(_m.self_id), secret, PUB_KEY_SIZE) == 0) {
			c = &k;
			break;
		}
	}
	if (c == nullptr) {
		return 0;
	}

	/* passive: in kind only, and legacy text even to a pinned contact */
	bool pinned = on() && (c->flags & ZCPFS_PIN_FLAG) != 0;

	if (type == PAYLOAD_TYPE_TXT_MSG) {
		return pinned ? -1 : 0;		/* SPEC 6.2: never legacy text to a pinned contact */
	}
	if (!pinned && c != _in_kind) {
		return 0;			/* SPEC 18.2: pinned, or in kind */
	}

	uint8_t n6[6];
	size_t len = 0;

	zcpfs_drbg_generate(&_drbg, n6, sizeof(n6));
	if (zcpfs_pair_seal(secret, _S, c->id.pub_key, type, n6, data, data_len, payload, &len) !=
	    ZCPFS_OK) {
		return -1;
	}
	_n.tx_pair++;
	return (int)len;
}

bool ZcpfsChat::sendText(const ContactInfo &to, const uint8_t *plain, size_t len,
			 uint32_t *expected_ack, uint32_t &est_timeout, int &rc)
{
	if (off() || !(to.flags & ZCPFS_PIN_FLAG)) {
		return false;
	}
	ensure();
	if (!on() && !(hasSession(to.id.pub_key) && _heard.since(to.id.pub_key))) {
		return false;			/* passive: sealed only to who still pins us (8.17) */
	}

	zcpfs_contact v = view(to);
	uint32_t token = 0, held_for = 0;

	v.pinned = 1;				/* checked above, in both modes */

	if (expected_ack != nullptr) {
		do {	/* what the application will see confirmed; never an on-air value */
			_m.getRNG()->random((uint8_t *)&token, sizeof(token));
		} while (token == 0);
	}

	int r = zcpfs_send_text(&_node, &v, plain, len, token, now(), &held_for);

	_next_tick = 0;
	if (r == ZCPFS_LEGACY) {
		return false;
	}
	if (r != ZCPFS_SENT && r != ZCPFS_HELD) {
		rc = MSG_SEND_FAILED;
		return true;
	}
	if (expected_ack != nullptr) {
		*expected_ack = token;
	}
	if (r == ZCPFS_HELD) {
		_n.held++;
	}
	/* SPEC 10.4: a held message is dropped when the application gives up */
	est_timeout = r == ZCPFS_HELD ? held_for * 1000
				      : timeoutFor(to, len + ZCPFS_DATA_OVERHEAD + 1);
	rc = v.has_path ? MSG_SEND_SENT_DIRECT : MSG_SEND_SENT_FLOOD;
	return true;
}

bool ZcpfsChat::sendLogin(const ContactInfo &to, const char *password, uint32_t &est_timeout,
			  int &rc)
{
	if (!on() || !(to.flags & ZCPFS_PIN_FLAG)) {
		return false;			/* passive starts nothing: the legacy login */
	}
	ensure();

	/* the legacy login plaintext without the password (SPEC 19.2) */
	uint8_t plain[8];
	size_t plain_len = 4;
	uint32_t t = _m.getRTCClock()->getCurrentTimeUnique();
	size_t pw_len = strlen(password);

	memcpy(plain, &t, 4);
	if (to.type == ADV_TYPE_ROOM) {
		memcpy(&plain[4], &to.sync_since, 4);
		plain_len = 8;
	}
	if (pw_len > 15) {
		pw_len = 15;			/* as the legacy login */
	}

	zcpfs_contact v = view(to);
	uint32_t wait_for = 0;

	v.timeout_ms = timeoutFor(to, ZCPFS_LOGIN_OVERHEAD + plain_len + 1);
	int r = zcpfs_login_send(&_node, &v, (const uint8_t *)password, pw_len, plain, plain_len,
				 now(), &wait_for);

	_next_tick = 0;
	if (r == ZCPFS_LEGACY) {
		return false;
	}
	if (r != ZCPFS_SENT) {
		rc = MSG_SEND_FAILED;
		return true;
	}
	est_timeout = wait_for * 1000;
	rc = v.has_path ? MSG_SEND_SENT_DIRECT : MSG_SEND_SENT_FLOOD;
	return true;
}

void ZcpfsChat::translateAck(uint32_t &ack)
{
	uint32_t token;
	uint8_t who[PUB_KEY_SIZE];

	if (_ready && zcpfs_on_ack(&_node, (const uint8_t *)&ack, &token, who)) {
		ack = token;
		_n.acked++;
	}
}

/* ---- channels (SPEC 17) ---------------------------------------------------- */

/* What a channel's settings entry is found by: the first 4 bytes of SHA-256
 * of its 32-byte secret field, so it follows the secret, not the slot. */
static void channelDigest(const uint8_t *secret, uint8_t out[4])
{
	mesh::Utils::sha256(out, 4, secret, PUB_KEY_SIZE);
}

uint8_t ZcpfsChat::chanState(const uint8_t *secret) const
{
	uint8_t d[4];

	if (_chan == nullptr) {
		return ZCPFS_CHAN_OFF;
	}
	channelDigest(secret, d);
	for (int i = 0; i < ZCPFS_CHAN_SETTINGS; i++) {
		if (_chan[i][4] != ZCPFS_CHAN_OFF && memcmp(_chan[i], d, 4) == 0) {
			return _chan[i][4];
		}
	}
	return ZCPFS_CHAN_OFF;
}

int ZcpfsChat::groupRx(mesh::Packet *pkt, const mesh::GroupChannel &ch, uint8_t *data, int *len)
{
	if (off()) {
		return mesh::Mesh::SEALED_LEGACY;	/* as firmware without zcpfs */
	}

	/* the length decides (17.2); the setting counts only when on */
	bool sealed = (pkt->payload_len - 3) % 16 != 0;
	uint8_t state = on() ? chanState(ch.secret) : ZCPFS_CHAN_OFF;

	if (!sealed) {
		return state == ZCPFS_CHAN_STRICT ? mesh::Mesh::SEALED_NEXT : mesh::Mesh::SEALED_LEGACY;
	}

	size_t plain_len = 0;

	memset(data, 0, MAX_PACKET_PAYLOAD);
	if (pkt->payload_len > ZCPFS_MAX_PAYLOAD ||
	    zcpfs_chan_open(ch.secret, ch.hash[0], pkt->getPayloadType(), pkt->payload,
			    pkt->payload_len, data, &plain_len) != ZCPFS_OK) {
		return mesh::Mesh::SEALED_NEXT;		/* another channel's, or forged */
	}
	_n.rx_chan++;
	*len = legacyLen(plain_len);
	return mesh::Mesh::SEALED_DATA;
}

int ZcpfsChat::groupSeal(uint8_t type, const mesh::GroupChannel &ch, const uint8_t *data,
			 size_t len, uint8_t *payload)
{
	if (!on() || chanState(ch.secret) == ZCPFS_CHAN_OFF) {
		return 0;
	}
	if (len > ZCPFS_CHAN_MAX_PLAIN) {
		if (type != PAYLOAD_TYPE_GRP_TXT) {
			return -1;			/* data does not fit sealed: not sent */
		}
		/* text is cut as legacy cuts it, 4 bytes shorter, never inside a character */
		len = ZCPFS_CHAN_MAX_PLAIN;
		while (len > 5 && (data[len] & 0xC0) == 0x80) {
			len--;
		}
	}

	uint8_t n6[6];
	size_t payload_len = 0;

	ensure();
	zcpfs_drbg_generate(&_drbg, n6, sizeof(n6));
	if (zcpfs_chan_seal(ch.secret, ch.hash[0], type, n6, data, len, payload, &payload_len) !=
	    ZCPFS_OK) {
		return -1;
	}
	_n.tx_chan++;
	return (int)payload_len;
}

static bool channelEmpty(const ChannelDetails &c)
{
	static const uint8_t zero[PUB_KEY_SIZE] = {0};

	return memcmp(c.channel.secret, zero, PUB_KEY_SIZE) == 0;
}

static const char *chanStateName(uint8_t s)
{
	return s == ZCPFS_CHAN_STRICT ? "strict" : s == ZCPFS_CHAN_ON ? "on" : "off";
}

/* `get zcpfs.channel` (all set ones), `get zcpfs.channel <name|index>`,
 * `set zcpfs.channel <name|index> off|on|strict`. A name is matched exactly;
 * a number is an index unless a channel has that name. */
int ZcpfsChat::channelCommand(const char *line, char *reply, size_t size)
{
	bool set = line[0] == 's';
	const char *arg = line + 17;

	if (_chan == nullptr) {
		return ZCPFS_CMD_NONE;
	}
	if (*arg == 0 && !set) {
		int n = snprintf(reply, size, ">");
		bool any = false;

		for (int i = 0; i < MAX_GROUP_CHANNELS && n > 0 && (size_t)n < size; i++) {
			uint8_t s = channelEmpty(_m.channels[i]) ? ZCPFS_CHAN_OFF
								 : chanState(_m.channels[i].channel.secret);
			if (s != ZCPFS_CHAN_OFF) {
				n += snprintf(reply + n, size - (size_t)n, "%s %d %s: %s", any ? "," : "", i,
					      _m.channels[i].name, chanStateName(s));
				any = true;
			}
		}
		if (!any) {
			snprintf(reply, size, "> no channel is sealed");
		}
		return ZCPFS_CMD_DONE;
	}
	if (*arg != ' ') {
		return ZCPFS_CMD_NONE;
	}
	arg++;

	/* for set, the last word is the state */
	char sel[40];
	uint8_t state = ZCPFS_CHAN_OFF;
	const char *end = arg + strlen(arg);

	if (set) {
		const char *sp = strrchr(arg, ' ');

		if (sp == nullptr) {
			snprintf(reply, size, "Err - set zcpfs.channel <name|index> off|on|strict");
			return ZCPFS_CMD_DONE;
		}
		if (strcmp(sp + 1, "on") == 0) {
			state = ZCPFS_CHAN_ON;
		} else if (strcmp(sp + 1, "strict") == 0) {
			state = ZCPFS_CHAN_STRICT;
		} else if (strcmp(sp + 1, "off") != 0) {
			snprintf(reply, size, "Err - off, on or strict");
			return ZCPFS_CMD_DONE;
		}
		end = sp;
	}
	if ((size_t)(end - arg) >= sizeof(sel)) {
		snprintf(reply, size, "Err - no such channel");
		return ZCPFS_CMD_DONE;
	}
	memcpy(sel, arg, (size_t)(end - arg));
	sel[end - arg] = 0;

	int idx = -1, named = 0;

	for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
		if (!channelEmpty(_m.channels[i]) && strcmp(_m.channels[i].name, sel) == 0) {
			idx = i;
			named++;
		}
	}
	if (named > 1) {
		snprintf(reply, size, "Err - %d channels are named %s: use the index", named, sel);
		return ZCPFS_CMD_DONE;
	}
	if (named == 0) {
		char *rest = nullptr;
		long v = strtol(sel, &rest, 10);

		if (sel[0] != 0 && *rest == 0 && v >= 0 && v < MAX_GROUP_CHANNELS &&
		    !channelEmpty(_m.channels[v])) {
			idx = (int)v;
		}
	}
	if (idx < 0) {
		snprintf(reply, size, "Err - no such channel");
		return ZCPFS_CMD_DONE;
	}

	const ChannelDetails &c = _m.channels[idx];
	bool known_secret = strcmp(c.name, "Public") == 0 || c.name[0] == '#';

	if (!set) {
		snprintf(reply, size, "> %d %s: %s", idx, c.name, chanStateName(chanState(c.channel.secret)));
		return ZCPFS_CMD_DONE;
	}

	uint8_t d[4];
	int slot = -1, free_slot = -1;

	channelDigest(c.channel.secret, d);
	for (int i = 0; i < ZCPFS_CHAN_SETTINGS; i++) {
		if (_chan[i][4] != ZCPFS_CHAN_OFF && memcmp(_chan[i], d, 4) == 0) {
			slot = i;
		} else if (_chan[i][4] == ZCPFS_CHAN_OFF && free_slot < 0) {
			free_slot = i;
		}
	}
	if (slot < 0 && state == ZCPFS_CHAN_OFF) {
		snprintf(reply, size, "OK - %s: off", c.name);
		return ZCPFS_CMD_DONE;		/* it was off */
	}
	if (slot < 0 && free_slot < 0) {
		/* make room: entries of channels this node no longer has */
		for (int i = 0; i < ZCPFS_CHAN_SETTINGS && free_slot < 0; i++) {
			bool held = false;

			for (int j = 0; j < MAX_GROUP_CHANNELS && !held; j++) {
				uint8_t dj[4];

				if (!channelEmpty(_m.channels[j])) {
					channelDigest(_m.channels[j].channel.secret, dj);
					held = memcmp(dj, _chan[i], 4) == 0;
				}
			}
			if (!held) {
				memset(_chan[i], 0, ZCPFS_CHAN_ENTRY);
				free_slot = i;
			}
		}
		if (free_slot < 0) {
			snprintf(reply, size, "Err - %d channels are sealed already", ZCPFS_CHAN_SETTINGS);
			return ZCPFS_CMD_DONE;
		}
	}
	if (slot < 0) {
		slot = free_slot;
	}
	snprintf(reply, size, "OK - %s: %s%s", c.name, chanStateName(state),
		 state != ZCPFS_CHAN_OFF && known_secret ? " (its secret is public)" : "");
	if (_chan[slot][4] == state && memcmp(_chan[slot], d, 4) == 0) {
		return ZCPFS_CMD_DONE;		/* unchanged */
	}
	if (state == ZCPFS_CHAN_OFF) {
		memset(_chan[slot], 0, ZCPFS_CHAN_ENTRY);
	} else {
		memcpy(_chan[slot], d, 4);
		_chan[slot][4] = state;
	}
	return ZCPFS_CMD_CHANGED;
}

/* ---- paths, adverts, timers ---------------------------------------------- */

void ZcpfsChat::onPath(ContactInfo &from, const mesh::Packet *pkt)
{
	if (off()) {
		return;
	}
	ensure();
	if (pkt->isRouteFlood()) {
		memcpy(_back_S, from.id.pub_key, PUB_KEY_SIZE);
		_back_len = mesh::Packet::copyPath(_back_path, pkt->path, pkt->path_len);
		_back_valid = true;
	}

	zcpfs_contact v = view(from);

	_n.paths++;
	_n.path_len = from.out_path_len;
	zcpfs_on_path(&_node, &v, pkt->isRouteFlood(), now());
	_next_tick = 0;
}

void ZcpfsChat::onAdvert(ContactInfo &c, const uint8_t *app_data, size_t app_data_len)
{
	static const uint8_t zero[PUB_KEY_SIZE] = {0};
	bool capable = zcpfsAdvertCapability(app_data, app_data_len) == 1 &&
		       memcmp(c.getSharedSecret(_m.self_id), zero, PUB_KEY_SIZE) != 0;	/* SPEC 4 */

	if (capable) {
		c.flags |= ZCPFS_PIN_FLAG;
	} else if (c.flags & ZCPFS_PIN_FLAG) {
		c.flags &= (uint8_t)~ZCPFS_PIN_FLAG;	/* its owner went back to legacy */
		forget(c.id.pub_key);
	}
}

void ZcpfsChat::forget(const uint8_t *pub_key)
{
	if (_ready) {
		zcpfs_forget(&_node, pub_key);
	}
	if (_back_valid && memcmp(_back_S, pub_key, PUB_KEY_SIZE) == 0) {
		_back_valid = false;
	}
}

void ZcpfsChat::loop()
{
	if (!_ready) {
		return;
	}

	uint32_t t = now();

	if ((int32_t)(t - _next_tick) >= 0) {
		uint32_t wait = zcpfs_tick(&_node, t);

		_next_tick = t + (wait == UINT32_MAX ? 3600 : wait);
	}
}

void ZcpfsChat::status(char *reply, size_t size)
{
	int records = 0, sessions = 0, pinned = 0;

	for (int i = 0; _ready && i < CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) {
		records += _peers[i].used ? 1 : 0;
		sessions += _peers[i].used && _peers[i].current.used ? 1 : 0;
	}
	for (int i = 0; i < _m.num_contacts; i++) {
		pinned += (_m.contacts[i].flags & ZCPFS_PIN_FLAG) ? 1 : 0;
	}
	snprintf(reply, size,
		 "> %s ch t%u r%u pinned %d records %d/%d sessions %d tx t%u c%u h%u/%u p%u s%u l%u rx t%u d%u s%u p%u l%u held %u acked %u path %u/%u/%u",
		 zcpfsModeName(_mode != nullptr ? *_mode : ZCPFS_MODE_OFF), _n.tx_chan, _n.rx_chan,
		 pinned, records,
		 CONFIG_ZEPHCORE_ZCPFS_PEERS, sessions, _n.tx_text, _n.tx_confirm,
		 _n.tx_hs1, _n.tx_hs2, _n.tx_probe, _n.tx_pair, _n.tx_login, _n.rx_text, _n.rx_dup,
		 _n.rx_pair, _n.rx_probe, _n.rx_login, _n.held, _n.acked,
		 _n.paths, _n.path_len, _n.login_path);
}

uint8_t ZcpfsChat::advert(uint8_t *app_data, uint8_t len)
{
	_heard.advertised(now());
	return on() ? zcpfsTagAdvert(app_data, len) : len;
}

bool ZcpfsChat::hasSession(const uint8_t *pub_key) const
{
	for (int i = 0; _ready && i < CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) {
		if (_peers[i].used && _peers[i].current.used &&
		    memcmp(_peers[i].S, pub_key, PUB_KEY_SIZE) == 0) {
			return true;
		}
	}
	return false;
}

/* Off: every session, handshake and held message goes, at once. */
void ZcpfsChat::wipe()
{
	zcpfs_wipe(_peers, sizeof(_peers));
	zcpfs_wipe(&_node, sizeof(_node));
	zcpfs_wipe(_a, sizeof(_a));
	_ready = false;
	_in_kind = nullptr;
	_rx_ack = false;
	_back_valid = false;
}

int ZcpfsChat::command(const char *line, char *reply, size_t size)
{
	if (strcmp(line, "get zcpfs.status") == 0) {
		status(reply, size);
		return ZCPFS_CMD_DONE;
	}
	if (strncmp(line, "get zcpfs.channel", 17) == 0 || strncmp(line, "set zcpfs.channel", 17) == 0) {
		return channelCommand(line, reply, size);
	}
	if (_mode == nullptr) {
		return ZCPFS_CMD_NONE;
	}

	int r = _no_legacy_login != nullptr
			? zcpfsLegacyLoginCommand(line, *_no_legacy_login, reply, size)
			: ZCPFS_CMD_NONE;

	if (r != ZCPFS_CMD_NONE) {
		return r;
	}
	r = zcpfsModeCommand(line, *_mode, reply, size);

	if (r == ZCPFS_CMD_CHANGED && off()) {
		wipe();
	}
	return r;
}

int BaseChatMesh::zcpfsCommand(const char *line, char *reply, size_t size)
{
	return _zc.command(line, reply, size);
}

/* ---- BaseChatMesh's side of the hooks in mesh::Mesh ---------------------- */

int BaseChatMesh::onPeerSealedRecv(mesh::Packet *packet, int sender_idx, const uint8_t *secret,
				   uint8_t *data, int *len)
{
	int i = matching_peer_indexes[sender_idx];

	(void)secret;
	if (i < 0 || i >= num_contacts) {
		return SEALED_LEGACY;
	}
	return _zc.rx(packet, contacts[i], data, len);
}

void BaseChatMesh::onPeerRecvDone()
{
	_zc.rxDone();
}

int BaseChatMesh::sealOutgoing(uint8_t type, const uint8_t *dest_hash, const uint8_t *secret,
			       const uint8_t *data, size_t data_len, uint8_t *payload)
{
	return _zc.sealOutgoing(type, dest_hash, secret, data, data_len, payload);
}

int BaseChatMesh::onGroupSealedRecv(mesh::Packet *packet, const mesh::GroupChannel &channel,
				    uint8_t *data, int *len)
{
	return _zc.groupRx(packet, channel, data, len);
}

int BaseChatMesh::sealGroupOutgoing(uint8_t type, const mesh::GroupChannel &channel,
				    const uint8_t *data, size_t data_len, uint8_t *payload)
{
	return _zc.groupSeal(type, channel, data, data_len, payload);
}
