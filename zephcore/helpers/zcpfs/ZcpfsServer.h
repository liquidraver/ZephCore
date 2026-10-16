/*
 * SPDX-License-Identifier: MIT
 * zcpfs for the server roles (repeater, room server): what a role whose
 * peers live in a ClientACL needs to speak lib/zcpfs (protocol:
 * lib/zcpfs/SPEC.md). The design is in devdocs/lld/14-zcpfs.md section 8.15.
 */

#pragma once

#include <mesh/Mesh.h>
#include <helpers/ClientACL.h>
#include "ZcpfsCommon.h"

extern "C" {
#include <zcpfs.h>
}

/* Logins answered and not yet confirmed, at the same time (SPEC 19.3 step 2). */
#define ZCPFS_N_LOGIN 2
/* Flooded login replies to clients the server does not know (SPEC 19.3). */
#define ZCPFS_R_LOGIN_FLOOD 4
#define ZCPFS_R_LOGIN_FLOOD_SECONDS 180

/* A password the server accepts and the permissions it grants. */
struct ZcpfsRole {
	const char *password;
	uint8_t perms;
};

/* What the role provides. */
class ZcpfsServerHost {
public:
	/* Airtime in ms of a packet of `len` bytes on the air. */
	virtual uint32_t zcpfsAirtime(int len) = 0;
	/* Flood a packet as the role floods its replies to the packet in hand. */
	virtual void zcpfsFlood(mesh::Packet *pkt, uint32_t delay_ms) = 0;
	/* The roles a login from this client can get, one or two (SPEC 19.3
	 * step 3): with no password typed, the one an empty password gets;
	 * otherwise those of the passwords that are set. `known` is the
	 * client's entry, NULL for a stranger. 0: not answered. */
	virtual int zcpfsRoles(const ClientInfo *known, bool empty_password, ZcpfsRole roles[2]) = 0;
	/* The client proved the password of `perms` (SPEC 19.5): enter it with
	 * those permissions, as a legacy login would. `login` is the login
	 * plaintext (timestamp, for a room the sync point). NULL if it could not
	 * be entered. */
	virtual ClientInfo *zcpfsGrant(const mesh::Identity &id, uint8_t perms, const uint8_t *login,
				       size_t login_len) = 0;
	/* What the ACL file keeps of the client has changed. */
	virtual void zcpfsAclChanged(ClientInfo &c) = 0;
};

class ZcpfsServer {
public:
	/* `mode` is the run-time mode in the node's prefs (ZcpfsCommon.h). */
	ZcpfsServer(mesh::Mesh &mesh, ClientACL &acl, ZcpfsServerHost &host, uint8_t fw_level,
		    uint8_t &mode, uint8_t &no_legacy_login)
		: _m(mesh), _acl(acl), _host(host), _fw_level(fw_level), _mode(mode),
		  _no_legacy_login(no_legacy_login) {}

	/* A legacy login arrived: true if it is to be dropped unread
	 * (`zcpfs.legacylogin off` while on, SPEC 19.6). */
	bool refuseLegacyLogin()
	{
		if (!on() || !_no_legacy_login) {
			return false;
		}
		_n.legacy_refused++;
		return true;
	}

	bool on() const { return _mode == ZCPFS_MODE_ON; }
	/* An own advert is being built: the tag when on (SPEC 6.1); returns the
	 * new length. */
	uint8_t advert(uint8_t *app_data, uint8_t len);

	/* A packet for this node from one ACL client whose hash matches, before
	 * legacy decryption. Returns one of mesh::Mesh::SEALED_*; with
	 * SEALED_DATA `data` and `*len` hold the legacy view. */
	int rx(mesh::Packet *pkt, ClientInfo &from, uint8_t *data, int *len);
	/* A TXT_MSG no client claimed: the first packet of a confirmed login
	 * (SPEC 19.5)? SEALED_NEXT if not; otherwise `*client` is the client
	 * the login entered, and with SEALED_DATA `data` and `*len` hold the
	 * text in the legacy view. */
	int rxStranger(mesh::Packet *pkt, uint8_t *data, int *len, ClientInfo **client);
	/* The packet of rx() has been dealt with. */
	void rxDone();
	/* The ACK value of the text rx() has just opened. */
	bool takeAck(uint8_t ack4[4]);
	/* An ANON_REQ for this node: one of mesh::Mesh::SEALED_LEGACY, _NEXT,
	 * _CONSUMED (SPEC 19.3). */
	int rxLogin(mesh::Packet *pkt);

	/* As ZcpfsChat::sealOutgoing, for ACL clients. */
	int sealOutgoing(uint8_t type, const uint8_t *dest_hash, const uint8_t *secret,
			 const uint8_t *data, size_t data_len, uint8_t *payload);
	/* A text to a client (SPEC 9.3), sent `delay_ms` from now. False: the
	 * client is not pinned, send it the legacy way. True: handled; `token`
	 * comes back from translateAck() when the client acknowledges it, and
	 * *held_ms (if given) is how long it waits for a session, 0 if it went
	 * out (10.4). */
	bool sendText(const ClientInfo &to, const uint8_t *plain, size_t len, uint32_t delay_ms,
		      uint32_t token = 0, uint32_t *held_ms = nullptr);
	/* An ACK value from the air: if it acknowledges a sealed text, replace it
	 * by the token of that send. */
	void translateAck(uint32_t &ack);

	/* The client's path return has been stored. */
	void onPath(ClientInfo &c, bool flood);
	/* The client logged in the legacy way: it no longer speaks zcpfs. */
	void onLegacyLogin(ClientInfo &c);
	void forget(const uint8_t *pub_key);
	/* Timers. */
	void loop();
	/* Milliseconds until loop() has something to do. */
	uint32_t msUntilTick() const;
	/* `get zcpfs`, `set zcpfs ...`, `get zcpfs.status`: one of ZCPFS_CMD_*;
	 * with ZCPFS_CMD_CHANGED the caller stores its prefs. */
	int command(const char *line, char *reply, size_t size);

private:
	mesh::Mesh &_m;
	ClientACL &_acl;
	ZcpfsServerHost &_host;
	uint8_t _fw_level;
	uint8_t &_mode;
	uint8_t &_no_legacy_login;
	ZcpfsHeard _heard;

	bool _ready = false;
	uint8_t _seed = 0;		/* ZCPFS_SEED_* of the last seeding */
	uint8_t _S[PUB_KEY_SIZE];
	uint8_t _a[32];
	zcpfs_drbg _drbg;
	zcpfs_ops _ops;
	zcpfs_node _node;
	zcpfs_peer _peers[CONFIG_ZEPHCORE_ZCPFS_PEERS];
	zcpfs_offer _offers[ZCPFS_N_LOGIN];
	uint32_t _next_tick = 0;
	uint32_t _send_delay = 0;	/* for what the engine sends inside a call */

	/* The login plaintext of each offer, for the grant (SPEC 19.5). */
	struct {
		uint8_t S[PUB_KEY_SIZE];
		uint8_t plain[8];
		uint8_t len;
	} _logins[ZCPFS_N_LOGIN] = {};

	/* R_LOGIN_FLOOD: when the last flooded replies to strangers left, oldest
	 * at _budget_i once all are used; at most that many in any window */
	uint32_t _budget_at[ZCPFS_R_LOGIN_FLOOD] = {};
	uint8_t _budget_i = 0;
	uint8_t _budget_n = 0;

	/* the packet in hand: who replies are sealed for although not pinned */
	uint8_t _kind_S[PUB_KEY_SIZE];
	uint8_t _kind_ss[PUB_KEY_SIZE];
	bool _kind = false;
	uint8_t _rx_ack4[4];
	bool _rx_ack = false;

	struct {			/* since boot, for `get zcpfs` */
		uint16_t tx_text, tx_confirm, tx_hs1, tx_hs2, tx_pair;
		uint16_t rx_text, rx_dup, rx_pair, rx_probe;
		uint16_t login, login_refused, answered, answered_flood, answered_back, granted;
		uint16_t legacy_refused;
		uint8_t last_hdr, last_path, last_known, last_roles, last_route;	/* the last login answered or not */
	} _n = {};

	bool off() const { return _mode == ZCPFS_MODE_OFF; }
	bool hasSession(const uint8_t *pub_key) const;
	void wipe();
	void ensure();
	static uint32_t now();
	zcpfs_contact view(const ClientInfo &c);
	uint32_t timeoutFor(uint8_t path_len, size_t payload_len);
	ClientInfo *client(const uint8_t *S);
	void inKind(const uint8_t *S, const uint8_t *ss);
	void replyTo(const ClientInfo &c, mesh::Packet *pkt, uint32_t delay_ms);
	bool budgetLeft(uint32_t t) const;
	void budgetTake(uint32_t t);
	void keepLogin(const uint8_t *S, const uint8_t *plain, size_t len);

	static void opSend(void *user, const uint8_t S[32], uint8_t type, const uint8_t *payload,
			   size_t len, int flood);
	static void opReciprocalPath(void *user, const uint8_t S[32]);
	static void opPin(void *user, const uint8_t S[32]);
	static int opContact(void *user, const uint8_t S[32], zcpfs_contact *c);
	static void opGranted(void *user, const uint8_t S[32], uint8_t id);
};
