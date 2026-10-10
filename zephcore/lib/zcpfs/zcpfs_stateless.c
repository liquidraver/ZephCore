/*
 * SPDX-License-Identifier: MIT
 * Packets that need no session: the path probe (section 11), channel packets
 * (17) and requests, responses and path returns (18). Each derives a one-time
 * key from a long-term secret and the random bytes the packet carries.
 */

#include "zcpfs_internal.h"

#include <string.h>

#define MSG_PROBE 0x03

/* ---- 11: path probe ------------------------------------------------------ */

static void probe_tag(const uint8_t ss[32], const uint8_t S_sender[32],
		      const uint8_t S_receiver[32], const uint8_t nonce4[4],
		      uint8_t tag[ZCPFS_TAG_LEN])
{
	const struct zcpfs_part parts[] = {
		{ ss, 32 }, { S_sender, 32 }, { S_receiver, 32 }, { nonce4, 4 },
	};
	uint8_t kp[32], ad[5];

	zcpfs_kdf("probe", parts, 4, kp, sizeof(kp));
	ad[0] = MSG_PROBE;
	memcpy(ad + 1, nonce4, 4);
	zcpfs_auth(kp, ad, sizeof(ad), tag);
	zcpfs_wipe(kp, sizeof(kp));
}

void zcpfs_probe_make(const uint8_t ss[32], const uint8_t S_sender[32],
		      const uint8_t S_receiver[32], const uint8_t nonce4[4],
		      uint8_t body[ZCPFS_PROBE_BODY_LEN])
{
	memcpy(body, nonce4, 4);
	probe_tag(ss, S_sender, S_receiver, nonce4, body + 4);
}

int zcpfs_probe_verify(const uint8_t ss[32], const uint8_t S_sender[32],
		       const uint8_t S_receiver[32], const uint8_t body[ZCPFS_PROBE_BODY_LEN])
{
	uint8_t tag[ZCPFS_TAG_LEN];

	probe_tag(ss, S_sender, S_receiver, body, tag);
	return zcpfs_ct_equal(tag, body + 4, ZCPFS_TAG_LEN) ? ZCPFS_OK : ZCPFS_ERR_AUTH;
}

/* ---- 17: channels --------------------------------------------------------
 * payload = channel hash ‖ n6 ‖ C ‖ T8
 */

#define CHAN_HDR 7	/* channel hash, n6 */

/* k = KDF("chan", secret, n6; 32);  AD = channel hash ‖ type ‖ n6 */
static void chan_key(const uint8_t secret[32], uint8_t chan_hash, uint8_t ptype,
		     const uint8_t n6[6], uint8_t k[32], uint8_t ad[8])
{
	const struct zcpfs_part parts[] = { { secret, 32 }, { n6, 6 } };

	zcpfs_kdf("chan", parts, 2, k, 32);
	ad[0] = chan_hash;
	ad[1] = ptype;
	memcpy(ad + 2, n6, 6);
}

int zcpfs_chan_seal(const uint8_t secret[32], uint8_t chan_hash, uint8_t ptype,
		    const uint8_t n6[6], const uint8_t *plain, size_t plain_len,
		    uint8_t *payload, size_t *payload_len)
{
	uint8_t k[32], ad[8];
	size_t len = plain_len;

	if (plain_len > ZCPFS_CHAN_MAX_PLAIN) {
		return ZCPFS_ERR_ARG;
	}
	payload[0] = chan_hash;
	memcpy(payload + 1, n6, 6);
	if (plain_len > 0) {
		memcpy(payload + CHAN_HDR, plain, plain_len);
	}
	/* hash, a 2-byte MAC and whole blocks would be a legacy channel packet */
	if ((len + ZCPFS_CHAN_OVERHEAD) % 16 == 3) {
		payload[CHAN_HDR + len++] = 0;
	}
	chan_key(secret, chan_hash, ptype, n6, k, ad);
	zcpfs_aead_seal(k, k + 16, ad, sizeof(ad), payload + CHAN_HDR, len, payload + CHAN_HDR);
	zcpfs_wipe(k, sizeof(k));
	*payload_len = len + ZCPFS_CHAN_OVERHEAD;
	return ZCPFS_OK;
}

int zcpfs_chan_open(const uint8_t secret[32], uint8_t chan_hash, uint8_t ptype,
		    const uint8_t *payload, size_t payload_len, uint8_t *plain,
		    size_t *plain_len)
{
	uint8_t k[32], ad[8];
	int rc;

	if (payload_len < ZCPFS_CHAN_OVERHEAD || payload_len > ZCPFS_MAX_PAYLOAD ||
	    payload_len % 16 == 3) {
		return ZCPFS_ERR_ARG;
	}
	if (payload[0] != chan_hash) {
		return ZCPFS_ERR_AUTH;
	}
	chan_key(secret, chan_hash, ptype, payload + 1, k, ad);
	rc = zcpfs_aead_open(k, k + 16, ad, sizeof(ad), payload + CHAN_HDR,
			     payload_len - CHAN_HDR, plain);
	zcpfs_wipe(k, sizeof(k));
	*plain_len = payload_len - ZCPFS_CHAN_OVERHEAD;
	return rc;
}

/* ---- 18: requests, responses, path returns -------------------------------
 * payload = dst ‖ src ‖ n6 ‖ C ‖ T8
 */

#define PAIR_HDR 8	/* dst, src, n6 */

/* k = KDF("pair", ss, S_sender, S_receiver, n6; 32);  AD = dst ‖ src ‖ type ‖ n6 */
static void pair_key(const uint8_t ss[32], const uint8_t S_sender[32],
		     const uint8_t S_receiver[32], uint8_t ptype, const uint8_t *payload,
		     uint8_t k[32], uint8_t ad[9])
{
	const uint8_t *n6 = payload + 2;
	const struct zcpfs_part parts[] = {
		{ ss, 32 }, { S_sender, 32 }, { S_receiver, 32 }, { n6, 6 },
	};

	zcpfs_kdf("pair", parts, 4, k, 32);
	ad[0] = payload[0];
	ad[1] = payload[1];
	ad[2] = ptype;
	memcpy(ad + 3, n6, 6);
}

int zcpfs_pair_seal(const uint8_t ss[32], const uint8_t S_sender[32],
		    const uint8_t S_receiver[32], uint8_t ptype, const uint8_t n6[6],
		    const uint8_t *plain, size_t plain_len, uint8_t *payload,
		    size_t *payload_len)
{
	uint8_t k[32], ad[9];
	size_t len = zcpfs_fit(ptype, plain_len, ZCPFS_PAIR_OVERHEAD);

	if (len + ZCPFS_PAIR_OVERHEAD > ZCPFS_MAX_PAYLOAD) {
		return ZCPFS_ERR_ARG;
	}
	payload[0] = S_receiver[0];
	payload[1] = S_sender[0];
	memcpy(payload + 2, n6, 6);
	if (plain_len > 0) {
		memcpy(payload + PAIR_HDR, plain, plain_len);
	}
	if (len > plain_len) {
		payload[PAIR_HDR + plain_len] = 0;
	}
	pair_key(ss, S_sender, S_receiver, ptype, payload, k, ad);
	zcpfs_aead_seal(k, k + 16, ad, sizeof(ad), payload + PAIR_HDR, len, payload + PAIR_HDR);
	zcpfs_wipe(k, sizeof(k));
	*payload_len = len + ZCPFS_PAIR_OVERHEAD;
	return ZCPFS_OK;
}

int zcpfs_pair_open(const uint8_t ss[32], const uint8_t S_sender[32],
		    const uint8_t S_receiver[32], uint8_t ptype, const uint8_t *payload,
		    size_t payload_len, uint8_t *plain, size_t *plain_len)
{
	uint8_t k[32], ad[9];
	int rc;

	if (payload_len < ZCPFS_PAIR_OVERHEAD || payload_len > ZCPFS_MAX_PAYLOAD) {
		return ZCPFS_ERR_ARG;
	}
	pair_key(ss, S_sender, S_receiver, ptype, payload, k, ad);
	rc = zcpfs_aead_open(k, k + 16, ad, sizeof(ad), payload + PAIR_HDR,
			     payload_len - PAIR_HDR, plain);
	zcpfs_wipe(k, sizeof(k));
	*plain_len = payload_len - ZCPFS_PAIR_OVERHEAD;
	return rc;
}
