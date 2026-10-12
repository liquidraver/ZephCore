/*
 * SPDX-License-Identifier: MIT
 * zcpfs for the chat client: what BaseChatMesh needs to speak lib/zcpfs
 * (protocol: lib/zcpfs/SPEC.md). The Shared mesh files only call into this
 * class; the design is in devdocs/lld/14-zcpfs.md section 8.12.
 */

#pragma once

#include <mesh/Mesh.h>
#include "ContactInfo.h"

extern "C" {
#include <zcpfs.h>
}

class BaseChatMesh;

/* ContactInfo.flags: the contact is pinned (SPEC 6.2). Stored with the contact. */
#define ZCPFS_PIN_FLAG 0x80

class ZcpfsChat {
public:
	explicit ZcpfsChat(BaseChatMesh &mesh) : _m(mesh) {}

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

	/* An ACK value from the air: if it acknowledges a sealed text, replace it
	 * by the value the application was given for that send. */
	void translateAck(uint32_t &ack);

	/* A path return from the contact has been processed. */
	void onPath(ContactInfo &from, const mesh::Packet *pkt);
	/* An advert of the contact was accepted: set or clear the pin (SPEC 6.2). */
	void onAdvert(ContactInfo &c, const uint8_t *app_data, size_t app_data_len);
	/* Append the capability tag to the name field of an own advert (SPEC 6.1). */
	static uint8_t tagAdvert(uint8_t *app_data, uint8_t len);

	void forget(const uint8_t *pub_key);
	void loop();
	/* The reply to `get zcpfs`: state, and counters since boot. */
	void status(char *reply, size_t size);

private:
	BaseChatMesh &_m;
	bool _ready = false;
	uint8_t _S[PUB_KEY_SIZE];	/* the identity the node was set up for */
	uint8_t _a[32];			/* its X25519 scalar */
	zcpfs_drbg _drbg;
	zcpfs_ops _ops;
	zcpfs_node _node;
	zcpfs_peer _peers[CONFIG_ZEPHCORE_ZCPFS_PEERS];
	uint32_t _next_tick = 0;

	/* the packet rx() is handling */
	const ContactInfo *_in_kind = nullptr;	/* its sender, if it arrived sealed */
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
		uint16_t tx_text, tx_confirm, tx_hs1, tx_hs2, tx_probe, tx_pair;
		uint16_t rx_text, rx_dup, rx_pair, rx_probe;
		uint16_t held, acked;
	} _n = {};

	void ensure();
	static uint32_t now();
	zcpfs_contact view(const ContactInfo &c);
	uint32_t timeoutFor(const ContactInfo &c, size_t payload_len);
	void ackOnly(mesh::Packet *pkt, ContactInfo &from, const uint8_t ack4[4]);

	static void opSend(void *user, const uint8_t S[32], uint8_t type, const uint8_t *payload,
			   size_t len, int flood);
	static void opReciprocalPath(void *user, const uint8_t S[32]);
	static void opPin(void *user, const uint8_t S[32]);
	static int opContact(void *user, const uint8_t S[32], zcpfs_contact *c);
};
