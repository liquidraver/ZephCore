/*
 * SPDX-License-Identifier: MIT
 * zcpfs for the chat client. See ZcpfsChat.h.
 */

#include "ZcpfsChat.h"

#include <stdio.h>
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
	v.pinned = (c.flags & ZCPFS_PIN_FLAG) != 0;
	v.has_path = c.out_path_len != OUT_PATH_UNKNOWN;
	v.timeout_ms = timeoutFor(c, 2 + ZCPFS_HS_BODY_LEN);
	return v;
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
	ensure();
	_in_kind = nullptr;
	_rx_ack = false;

	zcpfs_contact v = view(from);
	int r = zcpfs_rx(&_node, &v, pkt->getPayloadType(), pkt->payload, pkt->payload_len,
			 pkt->isRouteFlood(), now(), &out);
	_next_tick = 0;

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
		*len = (int)out.len;
		zcpfs_wipe(&out, sizeof(out));
		return mesh::Mesh::SEALED_DATA;
	default:	/* consumed; a login reply is not handled here yet */
		return mesh::Mesh::SEALED_CONSUMED;
	}
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

	bool pinned = (c->flags & ZCPFS_PIN_FLAG) != 0;

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
	ensure();
	if (!(to.flags & ZCPFS_PIN_FLAG)) {
		return false;
	}

	zcpfs_contact v = view(to);
	uint32_t token = 0, held_for = 0;

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

void ZcpfsChat::translateAck(uint32_t &ack)
{
	uint32_t token;
	uint8_t who[PUB_KEY_SIZE];

	if (_ready && zcpfs_on_ack(&_node, (const uint8_t *)&ack, &token, who)) {
		ack = token;
		_n.acked++;
	}
}

/* ---- paths, adverts, timers ---------------------------------------------- */

void ZcpfsChat::onPath(ContactInfo &from, const mesh::Packet *pkt)
{
	ensure();
	if (pkt->isRouteFlood()) {
		memcpy(_back_S, from.id.pub_key, PUB_KEY_SIZE);
		_back_len = mesh::Packet::copyPath(_back_path, pkt->path, pkt->path_len);
		_back_valid = true;
	}

	zcpfs_contact v = view(from);

	zcpfs_on_path(&_node, &v, pkt->isRouteFlood(), now());
	_next_tick = 0;
}

/* The capability an advert's name field carries (SPEC 6.1): 0 if none. */
static int advertCapability(const uint8_t *app_data, size_t len)
{
	size_t i = 1;

	if (len < 1 || !(app_data[0] & ADV_NAME_MASK)) {
		return 0;
	}
	if (app_data[0] & ADV_LATLON_MASK) {
		i += 8;
	}
	if (app_data[0] & ADV_FEAT1_MASK) {
		i += 2;
	}
	if (app_data[0] & ADV_FEAT2_MASK) {
		i += 2;
	}
	for (; i < len; i++) {
		if (app_data[i] == 0) {
			return i + 1 < len ? app_data[i + 1] : 1;
		}
	}
	return 0;
}

void ZcpfsChat::onAdvert(ContactInfo &c, const uint8_t *app_data, size_t app_data_len)
{
	static const uint8_t zero[PUB_KEY_SIZE] = {0};
	bool capable = advertCapability(app_data, app_data_len) == 1 &&
		       memcmp(c.getSharedSecret(_m.self_id), zero, PUB_KEY_SIZE) != 0;	/* SPEC 4 */

	if (capable) {
		c.flags |= ZCPFS_PIN_FLAG;
	} else if (c.flags & ZCPFS_PIN_FLAG) {
		c.flags &= (uint8_t)~ZCPFS_PIN_FLAG;	/* its owner went back to legacy */
		forget(c.id.pub_key);
	}
}

uint8_t ZcpfsChat::tagAdvert(uint8_t *app_data, uint8_t len)
{
	if (!(app_data[0] & ADV_NAME_MASK)) {
		app_data[0] |= ADV_NAME_MASK;		/* the field is always sent, even empty */
	} else if (len >= MAX_ADVERT_DATA_SIZE) {
		/* make room: drop the name's last character, whole */
		while (len > 1 && (app_data[len - 1] & 0xC0) == 0x80) {
			len--;
		}
		len--;
	}
	app_data[len++] = 0;
	return len;
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
		 "> pinned %d records %d/%d sessions %d tx t%u c%u h%u/%u p%u s%u rx t%u d%u s%u p%u held %u acked %u",
		 pinned, records, CONFIG_ZEPHCORE_ZCPFS_PEERS, sessions, _n.tx_text, _n.tx_confirm,
		 _n.tx_hs1, _n.tx_hs2, _n.tx_probe, _n.tx_pair, _n.rx_text, _n.rx_dup, _n.rx_pair,
		 _n.rx_probe, _n.held, _n.acked);
}

bool BaseChatMesh::zcpfsCommand(const char *line, char *reply, size_t size)
{
	if (strcmp(line, "get zcpfs") != 0) {
		return false;
	}
	_zc.status(reply, size);
	return true;
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
