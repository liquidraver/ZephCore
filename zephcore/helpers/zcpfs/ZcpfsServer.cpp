/*
 * SPDX-License-Identifier: MIT
 * zcpfs for the server roles. See ZcpfsServer.h.
 */

#include "ZcpfsServer.h"

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

#include <helpers/TxtDataHelpers.h>

/* Delays of a reply and of an ACK, as the server roles use them. */
#define REPLY_DELAY_MS 300
#define ACK_DELAY_MS 200

/* Engine time: uptime seconds, which do not wrap. */
uint32_t ZcpfsServer::now()
{
	return (uint32_t)(k_uptime_get() / 1000);
}

/* Set the engine up for the node's identity, again if that has changed. */
void ZcpfsServer::ensure()
{
	if (_ready && memcmp(_S, _m.self_id.pub_key, PUB_KEY_SIZE) == 0) {
		return;
	}

	uint8_t keys[PRV_KEY_SIZE + PUB_KEY_SIZE];

	_m.self_id.writeTo(keys, sizeof(keys));
	memcpy(_a, keys, sizeof(_a));		/* SPEC 4: the first half of the private key */
	memcpy(_S, _m.self_id.pub_key, PUB_KEY_SIZE);
	zcpfs_wipe(keys, sizeof(keys));

	_seed = (uint8_t)zcpfsSeed(&_drbg, *_m.getRNG());

	_ops.send = opSend;
	_ops.reciprocal_path = opReciprocalPath;
	_ops.pin = opPin;
	_ops.contact = opContact;
	_ops.granted = opGranted;
	zcpfs_node_init(&_node, _S, _a, &_drbg, _peers, CONFIG_ZEPHCORE_ZCPFS_PEERS, &_ops, this);
	zcpfs_node_serve(&_node, _offers, ZCPFS_N_LOGIN);
	memset(_logins, 0, sizeof(_logins));
	_kind = false;
	_rx_ack = false;
	_ready = true;
}

/* The firmware's own send timeout (as the companion's) for a packet of this
 * payload length: over the path, or flooded if it is OUT_PATH_UNKNOWN. */
uint32_t ZcpfsServer::timeoutFor(uint8_t path_len, size_t payload_len)
{
	if (path_len == OUT_PATH_UNKNOWN) {
		return 500 + 16 * _host.zcpfsAirtime((int)(2 + payload_len));
	}

	uint8_t hops = path_len & 63;
	size_t path_bytes = (size_t)hops * ((path_len >> 6) + 1);

	return 500 + (6 * _host.zcpfsAirtime((int)(2 + path_bytes + payload_len)) + 250) * (hops + 1);
}

zcpfs_contact ZcpfsServer::view(const ClientInfo &c)
{
	zcpfs_contact v;

	v.S = c.id.pub_key;
	v.ss = c.shared_secret;
	/* passive: no client counts as pinned, so legacy is read from all and
	 * nothing unopened starts a handshake (8.17) */
	v.pinned = on() && c.zcpfs_pinned;
	v.has_path = c.out_path_len != OUT_PATH_UNKNOWN;
	v.timeout_ms = timeoutFor(c.out_path_len, 2 + ZCPFS_HS_BODY_LEN);
	return v;
}

ClientInfo *ZcpfsServer::client(const uint8_t *S)
{
	return _acl.getClient(S, PUB_KEY_SIZE);
}

/* Replies to this key are sealed although it is not pinned (SPEC 18.2). */
void ZcpfsServer::inKind(const uint8_t *S, const uint8_t *ss)
{
	memcpy(_kind_S, S, PUB_KEY_SIZE);
	memcpy(_kind_ss, ss, PUB_KEY_SIZE);
	_kind = true;
}

/* A reply to a client: direct over its path, else flooded as the role floods. */
void ZcpfsServer::replyTo(const ClientInfo &c, mesh::Packet *pkt, uint32_t delay_ms)
{
	if (c.out_path_len == OUT_PATH_UNKNOWN) {
		_host.zcpfsFlood(pkt, delay_ms);
	} else {
		_m.sendDirect(pkt, c.out_path, c.out_path_len, delay_ms);
	}
}

/* The length legacy decryption would have produced: whole AES blocks; the
 * plaintext is followed by zero bytes up to there (SPEC 7.1). */
static int legacyLen(size_t len)
{
	return (int)((len + CIPHER_BLOCK_SIZE - 1) / CIPHER_BLOCK_SIZE * CIPHER_BLOCK_SIZE);
}

/* ---- the engine's callbacks ---------------------------------------------- */

void ZcpfsServer::opSend(void *user, const uint8_t S[32], uint8_t type, const uint8_t *payload,
			 size_t len, int flood)
{
	ZcpfsServer &z = *static_cast<ZcpfsServer *>(user);
	ClientInfo *c = z.client(S);
	mesh::Packet *pkt = c != nullptr ? z._m.obtainNewPacket() : nullptr;

	if (pkt == nullptr) {
		return;
	}
	if (type == PAYLOAD_TYPE_TXT_MSG) {
		len == ZCPFS_DATA_OVERHEAD ? z._n.tx_confirm++ : z._n.tx_text++;
	} else {
		type == PAYLOAD_TYPE_RESPONSE ? z._n.tx_hs2++ : z._n.tx_hs1++;
	}
	pkt->header = (uint8_t)(type << PH_TYPE_SHIFT);		/* the route type is set by the send */
	memcpy(pkt->payload, payload, len);
	pkt->payload_len = (uint16_t)len;
	if (flood) {
		z._host.zcpfsFlood(pkt, z._send_delay);
	} else {
		z._m.sendDirect(pkt, c->out_path, c->out_path_len, z._send_delay);
	}
}

/* A server sends no reciprocal paths, as legacy (RepeaterMesh::onPeerPathRecv). */
void ZcpfsServer::opReciprocalPath(void *user, const uint8_t S[32])
{
	(void)user;
	(void)S;
}

void ZcpfsServer::opPin(void *user, const uint8_t S[32])
{
	ZcpfsServer &z = *static_cast<ZcpfsServer *>(user);
	ClientInfo *c = z.client(S);

	if (c != nullptr && !c->zcpfs_pinned) {
		c->zcpfs_pinned = 1;
		z._host.zcpfsAclChanged(*c);
	}
}

int ZcpfsServer::opContact(void *user, const uint8_t S[32], zcpfs_contact *out)
{
	ZcpfsServer &z = *static_cast<ZcpfsServer *>(user);
	const ClientInfo *c = z.client(S);

	if (c == nullptr) {
		return 0;
	}
	*out = z.view(*c);
	return 1;
}

void ZcpfsServer::opGranted(void *user, const uint8_t S[32], uint8_t id)
{
	ZcpfsServer &z = *static_cast<ZcpfsServer *>(user);
	const uint8_t *plain = nullptr;
	size_t len = 0;

	for (int i = 0; i < ZCPFS_N_LOGIN; i++) {
		if (z._logins[i].len > 0 && memcmp(z._logins[i].S, S, PUB_KEY_SIZE) == 0) {
			plain = z._logins[i].plain;
			len = z._logins[i].len;
		}
	}

	ClientInfo *c = z._host.zcpfsGrant(mesh::Identity(S), id, plain, len);

	for (int i = 0; i < ZCPFS_N_LOGIN; i++) {
		if (memcmp(z._logins[i].S, S, PUB_KEY_SIZE) == 0) {
			memset(&z._logins[i], 0, sizeof(z._logins[i]));
		}
	}
	if (c == nullptr) {
		return;
	}
	z._n.granted++;
	c->zcpfs_pinned = 1;
	z._host.zcpfsAclChanged(*c);
	z.inKind(c->id.pub_key, c->shared_secret);
}

/* ---- receiving ----------------------------------------------------------- */

int ZcpfsServer::rx(mesh::Packet *pkt, ClientInfo &from, uint8_t *data, int *len)
{
	static zcpfs_rx_out out;	/* 196 bytes; the mesh runs on one thread */

	BUILD_ASSERT(sizeof(out.plain) == MAX_PACKET_PAYLOAD);
	if (off()) {
		return mesh::Mesh::SEALED_LEGACY;	/* as firmware without zcpfs */
	}
	ensure();
	_kind = false;
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
		inKind(from.id.pub_key, from.shared_secret);
		_n.rx_dup++;
		if ((out.plain[4] >> 2) == TXT_TYPE_PLAIN) {	/* the kind legacy acknowledges */
			mesh::Packet *ack = _m.createAck(out.ack4, 4);

			if (ack != nullptr) {
				replyTo(from, ack, ACK_DELAY_MS);
			}
		}
		zcpfs_wipe(&out, sizeof(out));
		return mesh::Mesh::SEALED_CONSUMED;
	}
	case ZCPFS_RX_PROBE: {
		/* as a flooded message without an ACK is answered (SPEC 11) */
		inKind(from.id.pub_key, from.shared_secret);
		_n.rx_probe++;
		mesh::Packet *path = _m.createPathReturn(from.id, from.shared_secret, pkt->path,
							 pkt->path_len, 0, NULL, 0);
		if (path != nullptr) {
			_host.zcpfsFlood(path, ACK_DELAY_MS);
		}
		return mesh::Mesh::SEALED_CONSUMED;
	}
	case ZCPFS_RX_TEXT:
		memcpy(_rx_ack4, out.ack4, sizeof(_rx_ack4));
		_rx_ack = true;
		[[fallthrough]];
	case ZCPFS_RX_PAIR:
		/* the legacy view (SPEC 7.1): the plaintext, then zero bytes */
		inKind(from.id.pub_key, from.shared_secret);
		r == ZCPFS_RX_TEXT ? _n.rx_text++ : _n.rx_pair++;
		memcpy(data, out.plain, sizeof(out.plain));
		*len = legacyLen(out.len);
		zcpfs_wipe(&out, sizeof(out));
		return mesh::Mesh::SEALED_DATA;
	default:
		return mesh::Mesh::SEALED_CONSUMED;
	}
}

int ZcpfsServer::rxStranger(mesh::Packet *pkt, uint8_t *data, int *len, ClientInfo **who)
{
	static zcpfs_rx_out out;
	uint8_t S[PUB_KEY_SIZE];

	*who = nullptr;
	if (off()) {
		return mesh::Mesh::SEALED_NEXT;
	}
	ensure();

	int r = zcpfs_rx_offer(&_node, pkt->payload, pkt->payload_len, now(), S, &out);

	_next_tick = 0;
	if (r == ZCPFS_RX_NEXT) {
		return mesh::Mesh::SEALED_NEXT;
	}
	*who = client(S);		/* entered by opGranted */
	_heard.mark(S, now());
	if (*who == nullptr || r != ZCPFS_RX_TEXT) {
		zcpfs_wipe(&out, sizeof(out));
		return mesh::Mesh::SEALED_CONSUMED;
	}
	/* the confirmation was lost and this text confirms the login (19.5) */
	memcpy(_rx_ack4, out.ack4, sizeof(_rx_ack4));
	_rx_ack = true;
	_n.rx_text++;
	memcpy(data, out.plain, sizeof(out.plain));
	*len = legacyLen(out.len);
	zcpfs_wipe(&out, sizeof(out));
	return mesh::Mesh::SEALED_DATA;
}

void ZcpfsServer::rxDone()
{
	_kind = false;
	_rx_ack = false;
}

bool ZcpfsServer::takeAck(uint8_t ack4[4])
{
	if (!_rx_ack) {
		return false;
	}
	memcpy(ack4, _rx_ack4, 4);
	_rx_ack = false;
	return true;
}

bool ZcpfsServer::budgetLeft(uint32_t t) const
{
	return _budget_n < ZCPFS_R_LOGIN_FLOOD ||
	       (uint32_t)(t - _budget_at[_budget_i]) >= ZCPFS_R_LOGIN_FLOOD_SECONDS;
}

void ZcpfsServer::budgetTake(uint32_t t)
{
	_budget_at[_budget_i] = t;
	_budget_i = (uint8_t)((_budget_i + 1) % ZCPFS_R_LOGIN_FLOOD);
	if (_budget_n < ZCPFS_R_LOGIN_FLOOD) {
		_budget_n++;
	}
}

/* The login plaintext of the offer just made to S: over the client's own,
 * else over one no offer uses any more. There is always one, as there are
 * as many as offers. */
void ZcpfsServer::keepLogin(const uint8_t *S, const uint8_t *plain, size_t len)
{
	int k = -1;

	for (int i = 0; i < ZCPFS_N_LOGIN && k < 0; i++) {
		if (memcmp(_logins[i].S, S, PUB_KEY_SIZE) == 0) {
			k = i;
		}
	}
	for (int i = 0; i < ZCPFS_N_LOGIN && k < 0; i++) {
		bool live = false;

		for (int j = 0; j < ZCPFS_N_LOGIN; j++) {
			live = live || (_offers[j].used &&
					memcmp(_offers[j].S, _logins[i].S, PUB_KEY_SIZE) == 0);
		}
		if (!live) {
			k = i;
		}
	}
	if (k < 0) {
		return;
	}
	memcpy(_logins[k].S, S, PUB_KEY_SIZE);
	_logins[k].len = (uint8_t)(len < sizeof(_logins[k].plain) ? len : sizeof(_logins[k].plain));
	memcpy(_logins[k].plain, plain, _logins[k].len);
}

int ZcpfsServer::rxLogin(mesh::Packet *pkt)
{
	static zcpfs_login_req req;
	static uint8_t reply[ZCPFS_MAX_PAYLOAD + 1];
	enum { NONE, PATH_FLOOD, PATH_BACK, RESPONSE_DIRECT, RESPONSE_FLOOD } how;

	if (off() || pkt->payload_len < 1 + PUB_KEY_SIZE) {
		return mesh::Mesh::SEALED_LEGACY;
	}
	ensure();

	ClientInfo *known = client(&pkt->payload[1]);
	uint32_t t = now();
	int r = zcpfs_login_rx(&_node, pkt->payload, pkt->payload_len, known != nullptr, t, &req);

	if (r == ZCPFS_LEGACY) {
		return mesh::Mesh::SEALED_LEGACY;
	}
	if (r != ZCPFS_OK) {
		_n.login_refused += r == ZCPFS_ERR_FULL ? 1 : 0;
		return mesh::Mesh::SEALED_NEXT;
	}
	_n.login++;

	uint32_t sent_at;
	ZcpfsRole roles[2];
	int count = 0;

	memcpy(&sent_at, req.plain, 4);
	if (known == nullptr || sent_at > known->last_timestamp) {	/* 19.3 step 2: no replay */
		count = _host.zcpfsRoles(known, req.empty_password != 0, roles);
	}

	/* How the reply travels (SPEC 19.3): as legacy's, with the budget for
	 * flooded replies to strangers. */
	bool flood_in = pkt->isRouteFlood();
	bool may_flood = known != nullptr || budgetLeft(t);
	uint8_t path_len = pkt->path_len;

	if (count <= 0) {
		how = NONE;
	} else if (flood_in) {
		how = may_flood ? PATH_FLOOD : PATH_BACK;
	} else if (known != nullptr && known->out_path_len != OUT_PATH_UNKNOWN) {
		how = RESPONSE_DIRECT;
		path_len = known->out_path_len;
	} else {
		how = may_flood ? RESPONSE_FLOOD : NONE;
	}
	if (how == PATH_FLOOD || how == RESPONSE_FLOOD) {
		path_len = OUT_PATH_UNKNOWN;
	}

	size_t reply_len = 0;

	if (how != NONE) {
		/* the reply's own send timeout sets how long the offer is kept:
		 * count, branches and at most two alignment bytes, in a RESPONSE or
		 * as the extra (type, length) of a path return */
		size_t body = 1 + (size_t)count * ZCPFS_BRANCH_LEN + 2;
		size_t est = how == PATH_FLOOD || how == PATH_BACK
				     ? ZCPFS_PAIR_OVERHEAD + 3 + pkt->getPathByteLen() + body
				     : 2 + body;

		uint32_t server_time = _m.getRTCClock()->getCurrentTimeUnique();
		uint8_t response[2][ZCPFS_LOGIN_RESPONSE_LEN];
		zcpfs_branch b[2];

		for (int i = 0; i < count; i++) {
			memcpy(response[i], &server_time, 4);
			response[i][4] = roles[i].perms;
			response[i][5] = _fw_level;
			b[i].password = (const uint8_t *)roles[i].password;
			b[i].password_len = strlen(roles[i].password);
			b[i].response = response[i];
			b[i].id = roles[i].perms;
		}
		r = zcpfs_login_answer(&_node, &req, b, (size_t)count, timeoutFor(path_len, est), t,
				       reply + 1, &reply_len);
		if (r != ZCPFS_OK) {
			_n.login_refused++;
			how = NONE;
		}
	}
	_n.last_hdr = pkt->header;
	_n.last_path = pkt->path_len;
	_n.last_known = known != nullptr;
	_n.last_roles = (uint8_t)count;
	_n.last_route = (uint8_t)how;
	if (how == NONE) {
		zcpfs_wipe(&req, sizeof(req));
		pkt->markDoNotRetransmit();	/* it was for this node */
		return mesh::Mesh::SEALED_CONSUMED;
	}

	_next_tick = 0;
	if (known == nullptr && (how == PATH_FLOOD || how == RESPONSE_FLOOD)) {
		budgetTake(t);			/* whatever becomes of it */
	}
	if (known != nullptr) {
		known->last_timestamp = sent_at;
		if (flood_in) {
			known->out_path_len = OUT_PATH_UNKNOWN;		/* as legacy: to be learnt again */
		}
	}
	keepLogin(req.S_c, req.plain, req.plain_len);
	inKind(req.S_c, req.ss);

	mesh::Identity client_id(req.S_c);
	mesh::Packet *out;

	if (how == PATH_FLOOD || how == PATH_BACK) {
		/* the extra of a path return, with its length first (SPEC 18.1) */
		reply[0] = (uint8_t)reply_len;
		out = _m.createPathReturn(client_id, req.ss, pkt->path, pkt->path_len,
					  PAYLOAD_TYPE_RESPONSE, reply, 1 + reply_len);
	} else {
		out = _m.obtainNewPacket();
		if (out != nullptr) {
			out->header = (uint8_t)(PAYLOAD_TYPE_RESPONSE << PH_TYPE_SHIFT);
			out->payload[0] = req.S_c[0];
			out->payload[1] = _S[0];
			memcpy(&out->payload[2], reply + 1, reply_len);
			out->payload_len = (uint16_t)(2 + reply_len);
		}
	}
	if (out != nullptr) {
		if (how == PATH_BACK) {
			/* over the reverse of the path the login came in on */
			uint8_t back[MAX_PATH_SIZE];
			uint8_t size = pkt->getPathHashSize(), hops = pkt->getPathHashCount();

			for (uint8_t i = 0; i < hops; i++) {
				memcpy(&back[i * size], &pkt->path[(hops - 1 - i) * size], size);
			}
			_m.sendDirect(out, back, pkt->path_len, REPLY_DELAY_MS);
			_n.answered_back++;
		} else if (how == RESPONSE_DIRECT) {
			_m.sendDirect(out, known->out_path, known->out_path_len, REPLY_DELAY_MS);
			_n.answered++;
		} else {
			_host.zcpfsFlood(out, REPLY_DELAY_MS);
			_n.answered_flood++;
		}
	}
	_kind = false;
	zcpfs_wipe(&req, sizeof(req));
	zcpfs_wipe(reply, sizeof(reply));
	pkt->markDoNotRetransmit();		/* last: it overwrites the header */
	return mesh::Mesh::SEALED_CONSUMED;
}

/* ---- sending ------------------------------------------------------------- */

int ZcpfsServer::sealOutgoing(uint8_t type, const uint8_t *dest_hash, const uint8_t *secret,
			      const uint8_t *data, size_t data_len, uint8_t *payload)
{
	const uint8_t *S = nullptr;
	bool pinned = false;

	if (off()) {
		return 0;
	}
	ensure();
	for (int i = 0; i < _acl.getNumClients(); i++) {
		const ClientInfo &k = *_acl.getClientByIdx(i);

		if (k.id.isHashMatch(dest_hash) &&
		    memcmp(k.shared_secret, secret, PUB_KEY_SIZE) == 0) {
			S = k.id.pub_key;
			pinned = on() && k.zcpfs_pinned != 0;	/* passive: in kind only */
			break;
		}
	}
	if (type == PAYLOAD_TYPE_TXT_MSG) {
		return pinned ? -1 : 0;		/* SPEC 6.2: never legacy text to a pinned client */
	}
	if (_kind && _kind_S[0] == dest_hash[0] && memcmp(_kind_ss, secret, PUB_KEY_SIZE) == 0) {
		S = _kind_S;			/* SPEC 18.2: in kind */
	} else if (!pinned) {
		return 0;
	}

	uint8_t n6[6];
	size_t len = 0;

	zcpfs_drbg_generate(&_drbg, n6, sizeof(n6));
	if (zcpfs_pair_seal(secret, _S, S, type, n6, data, data_len, payload, &len) != ZCPFS_OK) {
		return -1;
	}
	_n.tx_pair++;
	return (int)len;
}

bool ZcpfsServer::sendText(const ClientInfo &to, const uint8_t *plain, size_t len,
			   uint32_t delay_ms, uint32_t token, uint32_t *held_ms)
{
	if (held_ms != nullptr) {
		*held_ms = 0;
	}
	if (off() || !to.zcpfs_pinned) {
		return false;
	}
	ensure();
	if (!on() && !(hasSession(to.id.pub_key) && _heard.since(to.id.pub_key))) {
		return false;			/* passive: sealed only to who still pins us (8.17) */
	}

	zcpfs_contact v = view(to);
	uint32_t held_for = 0;

	v.pinned = 1;				/* checked above, in both modes */
	_send_delay = delay_ms;

	int r = zcpfs_send_text(&_node, &v, plain, len, token, now(), &held_for);

	_send_delay = 0;
	_next_tick = 0;
	if (r == ZCPFS_HELD && held_ms != nullptr) {
		*held_ms = held_for * 1000;
	}
	return r != ZCPFS_LEGACY;
}

void ZcpfsServer::translateAck(uint32_t &ack)
{
	uint32_t token;
	uint8_t who[PUB_KEY_SIZE];

	if (_ready && zcpfs_on_ack(&_node, (const uint8_t *)&ack, &token, who)) {
		ack = token;
	}
}

/* ---- paths, timers ------------------------------------------------------- */

void ZcpfsServer::onPath(ClientInfo &c, bool flood)
{
	if (off()) {
		return;
	}
	ensure();

	zcpfs_contact v = view(c);

	zcpfs_on_path(&_node, &v, flood, now());
	_next_tick = 0;
}

void ZcpfsServer::onLegacyLogin(ClientInfo &c)
{
	if (c.zcpfs_pinned) {
		c.zcpfs_pinned = 0;
		forget(c.id.pub_key);
		_host.zcpfsAclChanged(c);
	}
}

void ZcpfsServer::forget(const uint8_t *pub_key)
{
	if (_ready) {
		zcpfs_forget(&_node, pub_key);
	}
}

void ZcpfsServer::loop()
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

uint32_t ZcpfsServer::msUntilTick() const
{
	if (!_ready) {
		return UINT32_MAX;
	}

	int32_t left = (int32_t)(_next_tick - now());

	return left <= 0 ? 0 : (uint32_t)left * 1000;
}

uint8_t ZcpfsServer::advert(uint8_t *app_data, uint8_t len)
{
	_heard.advertised(now());
	return on() ? zcpfsTagAdvert(app_data, len) : len;
}

bool ZcpfsServer::hasSession(const uint8_t *pub_key) const
{
	for (int i = 0; _ready && i < CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) {
		if (_peers[i].used && _peers[i].current.used &&
		    memcmp(_peers[i].S, pub_key, PUB_KEY_SIZE) == 0) {
			return true;
		}
	}
	return false;
}

/* Off: every session, handshake, offer and held message goes, at once. */
void ZcpfsServer::wipe()
{
	zcpfs_wipe(_peers, sizeof(_peers));
	zcpfs_wipe(_offers, sizeof(_offers));
	zcpfs_wipe(&_node, sizeof(_node));
	zcpfs_wipe(_a, sizeof(_a));
	memset(_logins, 0, sizeof(_logins));
	_ready = false;
	_kind = false;
	_rx_ack = false;
}

int ZcpfsServer::command(const char *line, char *reply, size_t size)
{
	if (strcmp(line, "get zcpfs.status") != 0) {
		int r = zcpfsLegacyLoginCommand(line, _no_legacy_login, reply, size);

		if (r != ZCPFS_CMD_NONE) {
			return r;
		}
		r = zcpfsModeCommand(line, _mode, reply, size);

		if (r == ZCPFS_CMD_CHANGED && off()) {
			wipe();
		}
		return r;
	}

	int records = 0, sessions = 0, pinned = 0, offers = 0;

	for (int i = 0; _ready && i < CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) {
		records += _peers[i].used ? 1 : 0;
		sessions += _peers[i].used && _peers[i].current.used ? 1 : 0;
	}
	for (int i = 0; _ready && i < ZCPFS_N_LOGIN; i++) {
		offers += _offers[i].used ? 1 : 0;
	}
	for (int i = 0; i < _acl.getNumClients(); i++) {
		pinned += _acl.getClientByIdx(i)->zcpfs_pinned ? 1 : 0;
	}
	snprintf(reply, size,
		 "> %s seed %u pinned %d records %d/%d sessions %d offers %d tx t%u c%u h%u/%u s%u rx t%u d%u s%u p%u"
		 " login %u refused %u answered %u/%u/%u granted %u legacy refused %u last %02x/%02x/%u/%u/%u",
		 zcpfsModeName(_mode), _seed, pinned, records, CONFIG_ZEPHCORE_ZCPFS_PEERS, sessions, offers,
		 _n.tx_text,
		 _n.tx_confirm, _n.tx_hs1, _n.tx_hs2, _n.tx_pair, _n.rx_text, _n.rx_dup, _n.rx_pair,
		 _n.rx_probe, _n.login, _n.login_refused, _n.answered, _n.answered_flood,
		 _n.answered_back, _n.granted, _n.legacy_refused,
		 _n.last_hdr, _n.last_path, _n.last_known, _n.last_roles, _n.last_route);
	return ZCPFS_CMD_DONE;
}
