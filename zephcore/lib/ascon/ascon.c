/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ZephCore contributors
 *
 * Ascon-AEAD128, Ascon-Hash256, Ascon-XOF128 and Ascon-CXOF128, written from
 * NIST SP 800-232. See ascon.h and README.md.
 */

#include "ascon.h"

#include <string.h>

/* Initial values, SP 800-232 sections 4.1.1 and 5. */
#define IV_AEAD128	0x00001000808c0001ULL
#define IV_HASH256	0x0000080100cc0002ULL
#define IV_XOF128	0x0000080000cc0003ULL
#define IV_CXOF128	0x0000080000cc0004ULL

/* Round constants of Ascon-p; a permutation of n rounds uses the last n. */
static const uint8_t RC[16] = {
	0x3c, 0x2d, 0x1e, 0x0f, 0xf0, 0xe1, 0xd2, 0xc3,
	0xb4, 0xa5, 0x96, 0x87, 0x78, 0x69, 0x5a, 0x4b,
};

static uint64_t rotr(uint64_t x, unsigned n)
{
	return (x >> n) | (x << (64 - n));
}

static void permute(uint64_t s[5], unsigned rounds)
{
	uint64_t x0 = s[0], x1 = s[1], x2 = s[2], x3 = s[3], x4 = s[4];

	for (unsigned i = 16 - rounds; i < 16; i++) {
		uint64_t t0, t1, t2, t3, t4;

		x2 ^= RC[i];
		/* substitution layer */
		x0 ^= x4;
		x4 ^= x3;
		x2 ^= x1;
		t0 = ~x0 & x1;
		t1 = ~x1 & x2;
		t2 = ~x2 & x3;
		t3 = ~x3 & x4;
		t4 = ~x4 & x0;
		x0 ^= t1;
		x1 ^= t2;
		x2 ^= t3;
		x3 ^= t4;
		x4 ^= t0;
		x1 ^= x0;
		x0 ^= x4;
		x3 ^= x2;
		x2 = ~x2;
		/* linear diffusion layer */
		x0 ^= rotr(x0, 19) ^ rotr(x0, 28);
		x1 ^= rotr(x1, 61) ^ rotr(x1, 39);
		x2 ^= rotr(x2, 1) ^ rotr(x2, 6);
		x3 ^= rotr(x3, 10) ^ rotr(x3, 17);
		x4 ^= rotr(x4, 7) ^ rotr(x4, 41);
	}
	s[0] = x0;
	s[1] = x1;
	s[2] = x2;
	s[3] = x3;
	s[4] = x4;
}

/* Up to 8 bytes, little-endian, zero-extended. */
static uint64_t load(const uint8_t *b, size_t n)
{
	uint64_t v = 0;

	for (size_t i = 0; i < n; i++) {
		v |= (uint64_t)b[i] << (8 * i);
	}
	return v;
}

static void store(uint8_t *b, uint64_t v, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		b[i] = (uint8_t)(v >> (8 * i));
	}
}

/* The padding bit that follows n bytes (n < 8) in a 64-bit word. */
static uint64_t pad(size_t n)
{
	return (uint64_t)1 << (8 * n);
}

/* Low n bytes set (n <= 8). */
static uint64_t mask(size_t n)
{
	return n >= 8 ? ~(uint64_t)0 : pad(n) - 1;
}

/* A write the compiler may not remove. */
static void wipe(void *p, size_t len)
{
	volatile uint8_t *v = (volatile uint8_t *)p;

	while (len-- > 0) {
		*v++ = 0;
	}
}

/* ---- Ascon-AEAD128 ------------------------------------------------------- */

static void aead_start(uint64_t s[5], uint64_t k[2], const uint8_t key[16],
		       const uint8_t nonce[16], const uint8_t *ad, size_t ad_len)
{
	k[0] = load(key, 8);
	k[1] = load(key + 8, 8);
	s[0] = IV_AEAD128;
	s[1] = k[0];
	s[2] = k[1];
	s[3] = load(nonce, 8);
	s[4] = load(nonce + 8, 8);
	permute(s, 12);
	s[3] ^= k[0];
	s[4] ^= k[1];

	if (ad_len > 0) {
		while (ad_len >= 16) {
			s[0] ^= load(ad, 8);
			s[1] ^= load(ad + 8, 8);
			permute(s, 8);
			ad += 16;
			ad_len -= 16;
		}
		if (ad_len >= 8) {
			s[0] ^= load(ad, 8);
			s[1] ^= load(ad + 8, ad_len - 8) ^ pad(ad_len - 8);
		} else {
			s[0] ^= load(ad, ad_len) ^ pad(ad_len);
		}
		permute(s, 8);
	}
	s[4] ^= (uint64_t)1 << 63;	/* domain separation */
}

static void aead_finish(uint64_t s[5], const uint64_t k[2], uint8_t tag[16])
{
	s[2] ^= k[0];
	s[3] ^= k[1];
	permute(s, 12);
	store(tag, s[3] ^ k[0], 8);
	store(tag + 8, s[4] ^ k[1], 8);
}

void ascon_aead128_encrypt(const uint8_t key[ASCON_AEAD128_KEY_LEN],
			   const uint8_t nonce[ASCON_AEAD128_NONCE_LEN],
			   const uint8_t *ad, size_t ad_len, const uint8_t *pt, size_t len,
			   uint8_t *ct, uint8_t tag[ASCON_AEAD128_TAG_LEN])
{
	uint64_t s[5], k[2];

	aead_start(s, k, key, nonce, ad, ad_len);
	while (len >= 16) {
		s[0] ^= load(pt, 8);
		s[1] ^= load(pt + 8, 8);
		store(ct, s[0], 8);
		store(ct + 8, s[1], 8);
		permute(s, 8);
		pt += 16;
		ct += 16;
		len -= 16;
	}
	if (len >= 8) {
		s[0] ^= load(pt, 8);
		s[1] ^= load(pt + 8, len - 8);
		store(ct, s[0], 8);
		store(ct + 8, s[1], len - 8);
		s[1] ^= pad(len - 8);
	} else {
		s[0] ^= load(pt, len);
		store(ct, s[0], len);
		s[0] ^= pad(len);
	}
	aead_finish(s, k, tag);
	wipe(s, sizeof(s));
	wipe(k, sizeof(k));
}

int ascon_aead128_decrypt(const uint8_t key[ASCON_AEAD128_KEY_LEN],
			  const uint8_t nonce[ASCON_AEAD128_NONCE_LEN],
			  const uint8_t *ad, size_t ad_len, const uint8_t *ct, size_t len,
			  const uint8_t *tag, size_t tag_len, uint8_t *pt)
{
	uint64_t s[5], k[2], c0, c1;
	uint8_t expected[ASCON_AEAD128_TAG_LEN];
	uint8_t *out = pt;
	size_t total = len;
	uint8_t diff = 0;

	if (tag_len < ASCON_AEAD128_TAG_MIN || tag_len > ASCON_AEAD128_TAG_LEN) {
		wipe(pt, total);
		return -1;
	}
	aead_start(s, k, key, nonce, ad, ad_len);
	while (len >= 16) {
		c0 = load(ct, 8);
		c1 = load(ct + 8, 8);
		store(out, s[0] ^ c0, 8);
		store(out + 8, s[1] ^ c1, 8);
		s[0] = c0;
		s[1] = c1;
		permute(s, 8);
		ct += 16;
		out += 16;
		len -= 16;
	}
	/* The rate becomes the ciphertext bytes followed by the untouched
	 * state bytes, with the padding bit after them. */
	if (len >= 8) {
		size_t m = len - 8;

		c0 = load(ct, 8);
		c1 = load(ct + 8, m);
		store(out, s[0] ^ c0, 8);
		store(out + 8, s[1] ^ c1, m);
		s[0] = c0;
		s[1] = ((s[1] & ~mask(m)) | c1) ^ pad(m);
	} else {
		c0 = load(ct, len);
		store(out, s[0] ^ c0, len);
		s[0] = ((s[0] & ~mask(len)) | c0) ^ pad(len);
	}
	aead_finish(s, k, expected);

	for (size_t i = 0; i < tag_len; i++) {
		diff = (uint8_t)(diff | (expected[i] ^ tag[i]));
	}
	wipe(s, sizeof(s));
	wipe(k, sizeof(k));
	wipe(expected, sizeof(expected));
	if (diff != 0) {
		wipe(pt, total);
		return -1;
	}
	return 0;
}

/* ---- the sponge of Hash256, XOF128 and CXOF128 --------------------------- */

static void sponge_init(struct ascon_xof *x, uint64_t iv)
{
	memset(x, 0, sizeof(*x));
	x->s[0] = iv;
	permute(x->s, 12);
}

/* Close the string being absorbed: its last, padded block. */
static void absorb_end(struct ascon_xof *x)
{
	x->s[0] ^= load(x->buf, x->fill) ^ pad(x->fill);
	permute(x->s, 12);
	x->fill = 0;
}

void ascon_xof_absorb(struct ascon_xof *x, const uint8_t *m, size_t len)
{
	while (len > 0) {
		size_t take = 8u - x->fill;

		if (take > len) {
			take = len;
		}
		memcpy(x->buf + x->fill, m, take);
		x->fill = (uint8_t)(x->fill + take);
		m += take;
		len -= take;
		if (x->fill == 8) {
			x->s[0] ^= load(x->buf, 8);
			permute(x->s, 12);
			x->fill = 0;
		}
	}
}

void ascon_xof_squeeze(struct ascon_xof *x, uint8_t *out, size_t len)
{
	if (!x->squeezing) {
		absorb_end(x);
		x->squeezing = 1;
	}
	while (len > 0) {
		if (x->fill == 8) {
			permute(x->s, 12);
			x->fill = 0;
		}
		*out++ = (uint8_t)(x->s[0] >> (8 * x->fill));
		x->fill++;
		len--;
	}
}

void ascon_xof_wipe(struct ascon_xof *x)
{
	wipe(x, sizeof(*x));
}

void ascon_xof128_init(struct ascon_xof *x)
{
	sponge_init(x, IV_XOF128);
}

int ascon_cxof128_init(struct ascon_xof *x, const uint8_t *z, size_t z_len)
{
	if (z_len > ASCON_CXOF128_Z_MAX) {
		wipe(x, sizeof(*x));
		return -1;
	}
	sponge_init(x, IV_CXOF128);
	x->s[0] ^= (uint64_t)z_len * 8;		/* Z0: the length of Z in bits */
	permute(x->s, 12);
	ascon_xof_absorb(x, z, z_len);
	absorb_end(x);
	return 0;
}

void ascon_hash256(uint8_t out[ASCON_HASH256_LEN], const uint8_t *m, size_t len)
{
	struct ascon_xof x;

	sponge_init(&x, IV_HASH256);
	ascon_xof_absorb(&x, m, len);
	ascon_xof_squeeze(&x, out, ASCON_HASH256_LEN);
	wipe(&x, sizeof(x));
}
