/*
 * SPDX-License-Identifier: MIT
 * zcpfs for the chat client: what BaseChatMesh needs to speak lib/zcpfs
 * (protocol: lib/zcpfs/SPEC.md). The Shared mesh files only call into this
 * class; the design is in devdocs/lld/14-zcpfs.md section 8.12.
 */

#pragma once

#include <mesh/Mesh.h>
#include "ContactInfo.h"
#include "ZcpfsCommon.h"
#include "NodePrefs.h"	/* ZCPFS_CHAN_ENTRY */

extern "C" {
#include <zcpfs.h>
}

class BaseChatMesh;

/* ContactInfo.flags: the contact is pinned (SPEC 6.2). Stored with the contact. */
#define ZCPFS_PIN_FLAG 0x80

class ZcpfsChat {
public:
	explicit ZcpfsChat(BaseChatMesh &mesh) : _m(mesh) {}

	/* The run-time mode lives in the node's prefs (ZcpfsCommon.h); until it
	 * is bound the glue is off. */
	void bind(uint8_t *mode, uint8_t *no_legacy_login, uint8_t (*channels)[ZCPFS_CHAN_ENTRY])
	{
		_mode = mode;
		_no_legacy_login = no_legacy_login;
		_chan = channels;
	}
	/* A login to a server that is not pinned must not go out (SPEC 19.6). */
	bool refusesLegacyLogin() const
	{
		return on() && _no_legacy_login != nullptr && *_no_legacy_login;
	}
	bool on() const { return _mode != nullptr && *_mode == ZCPFS_MODE_ON; }
	/* An own advert is being built: the tag when on (SPEC 6.1); returns the
	 * new length. */
	uint8_t advert(uint8_t *app_data, uint8_t len);

	/* A packet for this node, for one contact whose hash matches, before
	 * legacy decryption. Returns one of mesh::Mesh::SEALED_*; with
	 * SEALED_DATA `data` and `*len` hold the legacy view of what was opened. */
	int rx(mesh::Packet *pkt, ContactInfo &from, uint8_t *data, int *len);
	/* The packet of rx() has been dealt with: replies are no longer "in kind". */
	void rxDone();
	/* The ACK value for the text rx() has just opened, in place of the
	 * legacy hash. False if the text was a legacy one. */
	bool takeAck(uint8_t ack4[4]);

	/* Payload of a REQ, RESPONSE or PATH to the contact with this hash and
	 * secret. Returns its length; 0 for a legacy packet; -1 if the packet
	 * must not go out (legacy text to a pinned contact, or too long). */
	int sealOutgoing(uint8_t type, const uint8_t *dest_hash, const uint8_t *secret,
			 const uint8_t *data, size_t data_len, uint8_t *payload);

	/* One send attempt of a text plaintext (SPEC 9.3). False: not a pinned
	 * contact, send it the legacy way. True: `rc` is the MSG_SEND_* result,
	 * `est_timeout` the application's timeout in ms, and *expected_ack (if
	 * wanted) the value the application will see confirmed. */
	bool sendText(const ContactInfo &to, const uint8_t *plain, size_t len,
		      uint32_t *expected_ack, uint32_t &est_timeout, int &rc);

	/* A login to a server (SPEC 19.2). False: not a pinned server, send the
	 * legacy login. True: `rc` is the MSG_SEND_* result and `est_timeout`
	 * the application's timeout in ms. */
	bool sendLogin(const ContactInfo &to, const char *password, uint32_t &est_timeout, int &rc);
	/* The RESPONSE extra of a path return from the contact, before the
	 * application sees it: a login reply (19.4) is replaced by the legacy
	 * login response. False: drop it, it is a login reply that did not open. */
	bool pathResponse(const ContactInfo &from, uint8_t *extra, uint8_t &extra_len);

	/* A channel packet, per channel whose hash matches, before legacy
	 * decryption (SPEC 17.2): one of mesh::Mesh::SEALED_*. */
	int groupRx(mesh::Packet *pkt, const mesh::GroupChannel &ch, uint8_t *data, int *len);
	/* The payload of a channel datagram, sealed if the channel is set to it:
	 * its length, 0 for legacy, -1 if it must not go out. */
	int groupSeal(uint8_t type, const mesh::GroupChannel &ch, const uint8_t *data, size_t len,
		      uint8_t *payload);

	/* The plaintext of a legacy text to the contact: with the padding hint
	 * (SPEC 6.3) when on, not pinned, and it costs no AES block. Returns the
	 * new length; `cap` is the size of `p`. */
	size_t addHint(const ContactInfo &to, uint8_t *p, size_t len, size_t cap, uint8_t attempt);
	/* A plain text from the contact was delivered (legacy view in `data`): a
	 * legacy one with the hint asks for a handshake (SPEC 6.3). */
	void onText(ContactInfo &from, const uint8_t *data, size_t len);

	/* An ACK value from the air: if it acknowledges a sealed text, replace it
	 * by the value the application was given for that send. */
	void translateAck(uint32_t &ack);

	/* A path return from the contact has been processed. */
	void onPath(ContactInfo &from, const mesh::Packet *pkt);
	/* An advert of the contact was accepted: set or clear the pin (SPEC 6.2). */
	void onAdvert(ContactInfo &c, const uint8_t *app_data, size_t app_data_len);

	void forget(const uint8_t *pub_key);
	void loop();
	/* `get zcpfs`, `set zcpfs ...`, `get zcpfs.status`: one of ZCPFS_CMD_*. */
	int command(const char *line, char *reply, size_t size);

private:
	BaseChatMesh &_m;
	uint8_t *_mode = nullptr;
	uint8_t *_no_legacy_login = nullptr;
	uint8_t (*_chan)[ZCPFS_CHAN_ENTRY] = nullptr;	/* ZCPFS_CHAN_SETTINGS entries */
	ZcpfsHeard _heard;
	bool _ready = false;
	uint8_t _seed = 0;		/* ZCPFS_SEED_* of the last seeding */
	uint8_t _S[PUB_KEY_SIZE];	/* the identity the node was set up for */
	uint8_t _a[32];			/* its X25519 scalar */
	zcpfs_drbg _drbg;
	zcpfs_ops _ops;
	zcpfs_node _node;
	zcpfs_peer _peers[CONFIG_ZEPHCORE_ZCPFS_PEERS];
	uint32_t _next_tick = 0;

	/* the packet rx() is handling */
	const ContactInfo *_in_kind = nullptr;	/* its sender, if it arrived sealed */
	bool _in_flood = false;			/* it was flooded */
	uint8_t _rx_ack4[4];
	bool _rx_ack = false;

	/* The path the last flooded path return took to reach us: what a
	 * reciprocal path tells its sender (SPEC 11). */
	uint8_t _back_S[PUB_KEY_SIZE];
	uint8_t _back_path[MAX_PATH_SIZE];
	uint8_t _back_len = 0;
	bool _back_valid = false;
	bool _hs_after_path = false;	/* the next handshake follows a path packet */

	struct {			/* since boot, for `get zcpfs` */
		uint16_t tx_text, tx_confirm, tx_hs1, tx_hs2, tx_probe, tx_pair, tx_login;
		uint16_t rx_text, rx_dup, rx_pair, rx_probe, rx_login;
		uint16_t held, acked;
		uint16_t paths;		/* path returns stored */
		uint8_t path_len;	/* the last one's length byte */
		uint8_t login_path;	/* the path length a login reply found */
		uint16_t tx_chan, rx_chan;	/* sealed channel packets */
		uint16_t tx_hint, rx_hint;	/* padding hints */
	} _n = {};

	bool off() const { return _mode == nullptr || *_mode == ZCPFS_MODE_OFF; }
	bool hasSession(const uint8_t *pub_key) const;
	void wipe();
	void status(char *reply, size_t size);
	uint8_t chanState(const uint8_t *secret) const;
	int channelCommand(const char *line, char *reply, size_t size);
	void ensure();
	static uint32_t now();
	zcpfs_contact view(const ContactInfo &c);
	uint32_t timeoutFor(const ContactInfo &c, size_t payload_len);
	void ackOnly(mesh::Packet *pkt, ContactInfo &from, const uint8_t ack4[4]);
	void legacyLoginResponse(const ContactInfo &server, const uint8_t *r, uint8_t *data);

	static void opSend(void *user, const uint8_t S[32], uint8_t type, const uint8_t *payload,
			   size_t len, int flood);
	static void opReciprocalPath(void *user, const uint8_t S[32]);
	static void opPin(void *user, const uint8_t S[32]);
	static int opContact(void *user, const uint8_t S[32], zcpfs_contact *c);
};
