/*
 * SPDX-License-Identifier: MIT
 * Section 8: the handshake. Two packets, four Diffie-Hellman terms:
 *   es  initiator one-time key  x  responder identity
 *   ss  the two identities (MeshCore's shared secret, passed in)
 *   ee  the two one-time keys
 *   se  initiator identity  x  responder one-time key
 */

#include "zcpfs_internal.h"

#include <string.h>

#define MSG_HS1 0x01
#define MSG_HS2 0x02

void zcpfs_keys_from_master(const uint8_t master[32], struct zcpfs_keys *keys)
{
	const struct zcpfs_part parts[] = { { master, 32 } };

	zcpfs_kdf("chain.i2r", parts, 1, keys->ck_i2r, 32);
	zcpfs_kdf("chain.r2i", parts, 1, keys->ck_r2i, 32);
	zcpfs_kdf("drbg.feed", parts, 1, keys->feed, 32);
}

/* tag = AUTH(k, msg ‖ E) */
static void hs_tag(const uint8_t k[32], uint8_t msg, const uint8_t E[32],
		   uint8_t tag[ZCPFS_TAG_LEN])
{
	uint8_t ad[33];

	ad[0] = msg;
	memcpy(ad + 1, E, 32);
	zcpfs_auth(k, ad, sizeof(ad), tag);
}

static void hs1_tag(const uint8_t ss[32], const uint8_t es[32], const uint8_t S_i[32],
		    const uint8_t S_r[32], const uint8_t E_i[32], uint8_t tag[ZCPFS_TAG_LEN])
{
	const struct zcpfs_part parts[] = {
		{ ss, 32 }, { es, 32 }, { S_i, 32 }, { S_r, 32 }, { E_i, 32 },
	};
	uint8_t k1[32];

	zcpfs_kdf("hs1", parts, 5, k1, sizeof(k1));
	hs_tag(k1, MSG_HS1, E_i, tag);
	zcpfs_wipe(k1, sizeof(k1));
}

/* The master secret, the tag of handshake 2 and the session keys. */
static void hs2_keys(const uint8_t ss[32], const uint8_t es[32], const uint8_t ee[32],
		     const uint8_t se[32], const uint8_t S_i[32], const uint8_t S_r[32],
		     const uint8_t E_i[32], const uint8_t E_r[32], uint8_t tag[ZCPFS_TAG_LEN],
		     struct zcpfs_keys *keys)
{
	const struct zcpfs_part parts[] = {
		{ ss, 32 }, { es, 32 }, { ee, 32 }, { se, 32 },
		{ S_i, 32 }, { S_r, 32 }, { E_i, 32 }, { E_r, 32 },
	};
	uint8_t master[32], k2[32];
	const struct zcpfs_part m[] = { { master, 32 } };

	zcpfs_kdf("master", parts, 8, master, sizeof(master));
	zcpfs_kdf("hs2", m, 1, k2, sizeof(k2));
	hs_tag(k2, MSG_HS2, E_r, tag);
	zcpfs_keys_from_master(master, keys);
	zcpfs_wipe(master, sizeof(master));
	zcpfs_wipe(k2, sizeof(k2));
}

int zcpfs_hs1_make(struct zcpfs_drbg *g, const uint8_t ss[32], const uint8_t S_i[32],
		   const uint8_t S_r[32], struct zcpfs_hs_out *out,
		   uint8_t body[ZCPFS_HS_BODY_LEN])
{
	uint8_t A_r[32];

	zcpfs_drbg_generate(g, out->e, 32);
	zcpfs_dh_public(out->E, out->e);
	zcpfs_ed_to_x(A_r, S_r);
	if (zcpfs_dh(out->es, out->e, A_r) != ZCPFS_OK) {
		zcpfs_wipe(out, sizeof(*out));
		return ZCPFS_ERR_WEAK;
	}
	memcpy(body, out->E, 32);
	hs1_tag(ss, out->es, S_i, S_r, out->E, body + 32);
	return ZCPFS_OK;
}

int zcpfs_hs1_verify(const uint8_t ss[32], const uint8_t a_r[32], const uint8_t S_i[32],
		     const uint8_t S_r[32], const uint8_t body[ZCPFS_HS_BODY_LEN],
		     uint8_t es[32])
{
	const uint8_t *E_i = body;
	uint8_t tag[ZCPFS_TAG_LEN];
	int rc = ZCPFS_ERR_WEAK;

	if (zcpfs_dh(es, a_r, E_i) == ZCPFS_OK) {
		hs1_tag(ss, es, S_i, S_r, E_i, tag);
		rc = zcpfs_ct_equal(tag, body + 32, ZCPFS_TAG_LEN) ? ZCPFS_OK : ZCPFS_ERR_AUTH;
	}
	if (rc != ZCPFS_OK) {
		zcpfs_wipe(es, 32);
	}
	return rc;
}

int zcpfs_hs2_make(struct zcpfs_drbg *g, const uint8_t ss[32], const uint8_t es[32],
		   const uint8_t S_i[32], const uint8_t S_r[32], const uint8_t E_i[32],
		   uint8_t body[ZCPFS_HS_BODY_LEN], struct zcpfs_keys *keys)
{
	uint8_t e[32], A_i[32], ee[32], se[32];
	int rc = ZCPFS_ERR_WEAK;

	zcpfs_drbg_generate(g, e, sizeof(e));
	zcpfs_dh_public(body, e);			/* E_r */
	zcpfs_ed_to_x(A_i, S_i);
	if (zcpfs_dh(ee, e, E_i) == ZCPFS_OK && zcpfs_dh(se, e, A_i) == ZCPFS_OK) {
		hs2_keys(ss, es, ee, se, S_i, S_r, E_i, body, body + 32, keys);
		rc = ZCPFS_OK;
	}
	zcpfs_wipe(e, sizeof(e));
	zcpfs_wipe(ee, sizeof(ee));
	zcpfs_wipe(se, sizeof(se));
	return rc;
}

int zcpfs_hs2_process(const uint8_t ss[32], const struct zcpfs_hs_out *out,
		      const uint8_t a_i[32], const uint8_t S_i[32], const uint8_t S_r[32],
		      const uint8_t body[ZCPFS_HS_BODY_LEN], struct zcpfs_keys *keys)
{
	const uint8_t *E_r = body;
	uint8_t ee[32], se[32], tag[ZCPFS_TAG_LEN];
	int rc = ZCPFS_ERR_WEAK;

	if (zcpfs_dh(ee, out->e, E_r) == ZCPFS_OK && zcpfs_dh(se, a_i, E_r) == ZCPFS_OK) {
		hs2_keys(ss, out->es, ee, se, S_i, S_r, out->E, E_r, tag, keys);
		rc = zcpfs_ct_equal(tag, body + 32, ZCPFS_TAG_LEN) ? ZCPFS_OK : ZCPFS_ERR_AUTH;
	}
	if (rc != ZCPFS_OK) {
		zcpfs_wipe(keys, sizeof(*keys));
	}
	zcpfs_wipe(ee, sizeof(ee));
	zcpfs_wipe(se, sizeof(se));
	return rc;
}
