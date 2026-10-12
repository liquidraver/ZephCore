/*
 * SPDX-License-Identifier: MIT
 * Sections 3 and 7: KDF, AUTH, the truncated-tag AEAD, constant-time compare,
 * wipe and the allowed-length rule. The only file that calls Ascon
 * (lib/ascon).
 */

#include "zcpfs_internal.h"

#include <ascon.h>
#include <string.h>

#define PREFIX		"zcpfs1/"
#define PREFIX_LEN	(sizeof(PREFIX) - 1)
#define LABEL_MAX	24

void zcpfs_kdf_split(const char *label, const struct zcpfs_part *parts, size_t n_parts,
		     uint8_t *out1, size_t len1, uint8_t *out2, size_t len2)
{
	struct ascon_xof x;
	uint8_t z[PREFIX_LEN + LABEL_MAX];
	size_t label_len = strlen(label);

	if (label_len > LABEL_MAX) {
		label_len = LABEL_MAX;
	}
	memcpy(z, PREFIX, PREFIX_LEN);
	memcpy(z + PREFIX_LEN, label, label_len);
	/* at most 31 bytes, far below the limit that makes this fail */
	(void)ascon_cxof128_init(&x, z, PREFIX_LEN + label_len);

	/* each part is framed by its length, 16 bits little-endian */
	for (size_t i = 0; i < n_parts; i++) {
		uint8_t le16[2];

		le16[0] = (uint8_t)(parts[i].len & 0xff);
		le16[1] = (uint8_t)((parts[i].len >> 8) & 0xff);
		ascon_xof_absorb(&x, le16, 2);
		ascon_xof_absorb(&x, parts[i].data, parts[i].len);
	}
	ascon_xof_squeeze(&x, out1, len1);
	ascon_xof_squeeze(&x, out2, len2);
	ascon_xof_wipe(&x);
}

void zcpfs_kdf(const char *label, const struct zcpfs_part *parts, size_t n_parts,
	       uint8_t *out, size_t out_len)
{
	zcpfs_kdf_split(label, parts, n_parts, out, out_len, NULL, 0);
}

void zcpfs_auth(const uint8_t k[32], const uint8_t *ad, size_t ad_len,
		uint8_t tag[ZCPFS_TAG_LEN])
{
	uint8_t full[ASCON_AEAD128_TAG_LEN];

	ascon_aead128_encrypt(k, k + 16, ad, ad_len, NULL, 0, NULL, full);
	memcpy(tag, full, ZCPFS_TAG_LEN);
	zcpfs_wipe(full, sizeof(full));
}

void zcpfs_aead_seal_tag(const uint8_t key[16], const uint8_t nonce[16], const uint8_t *ad,
			 size_t ad_len, const uint8_t *pt, size_t len, uint8_t *out,
			 size_t tag_len)
{
	uint8_t full[ASCON_AEAD128_TAG_LEN];

	ascon_aead128_encrypt(key, nonce, ad, ad_len, pt, len, out, full);
	memcpy(out + len, full, tag_len);
	zcpfs_wipe(full, sizeof(full));
}

int zcpfs_aead_open_tag(const uint8_t key[16], const uint8_t nonce[16], const uint8_t *ad,
			size_t ad_len, const uint8_t *body, size_t body_len, uint8_t *pt,
			size_t tag_len)
{
	size_t len = body_len - tag_len;

	if (ascon_aead128_decrypt(key, nonce, ad, ad_len, body, len, body + len, tag_len,
				  pt) != 0) {
		return ZCPFS_ERR_AUTH;
	}
	return ZCPFS_OK;
}

void zcpfs_aead_seal(const uint8_t key[16], const uint8_t nonce[16], const uint8_t *ad,
		     size_t ad_len, const uint8_t *pt, size_t len, uint8_t *out)
{
	zcpfs_aead_seal_tag(key, nonce, ad, ad_len, pt, len, out, ZCPFS_TAG_LEN);
}

int zcpfs_aead_open(const uint8_t key[16], const uint8_t nonce[16], const uint8_t *ad,
		    size_t ad_len, const uint8_t *body, size_t body_len, uint8_t *pt)
{
	return zcpfs_aead_open_tag(key, nonce, ad, ad_len, body, body_len, pt, ZCPFS_TAG_LEN);
}

int zcpfs_ct_equal(const uint8_t *a, const uint8_t *b, size_t len)
{
	uint8_t diff = 0;

	for (size_t i = 0; i < len; i++) {
		diff = (uint8_t)(diff | (a[i] ^ b[i]));
	}
	/* 1 when diff is 0, without a branch on it */
	return (int)((((uint32_t)diff - 1) >> 8) & 1);
}

void zcpfs_wipe(void *p, size_t len)
{
	volatile uint8_t *v = (volatile uint8_t *)p;

	while (len-- > 0) {
		*v++ = 0;
	}
}

size_t zcpfs_fit(uint8_t ptype, size_t plain_len, size_t overhead)
{
	for (;;) {
		size_t n = plain_len + overhead;
		int bad;

		if (ptype == ZCPFS_PT_ANON) {
			/* dst, a 32-byte key, a 2-byte MAC and whole blocks: a legacy login */
			bad = n % 16 == 3;
		} else {
			/* dst, src, a 2-byte MAC and whole blocks: a legacy packet */
			bad = n % 16 == 4;
			/* the lengths that mean a probe or a handshake */
			bad = bad || (ptype == ZCPFS_PT_REQ && (n == 14 || n == 42));
			bad = bad || (ptype == ZCPFS_PT_RESPONSE && n == 42);
		}
		if (!bad) {
			return plain_len;
		}
		plain_len++;
	}
}
