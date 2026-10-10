/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ZephCore contributors
 *
 * Ascon as standardised in NIST SP 800-232: Ascon-AEAD128, Ascon-Hash256,
 * Ascon-XOF128 and Ascon-CXOF128.
 *
 * One header, one source file, C99, no dependencies beyond <stdint.h>,
 * <stddef.h> and memcpy. No heap, no global state, no tables, no branch or
 * memory index that depends on secret data.
 */

#ifndef ASCON_H
#define ASCON_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ASCON_AEAD128_KEY_LEN	16
#define ASCON_AEAD128_NONCE_LEN	16
#define ASCON_AEAD128_TAG_LEN	16
#define ASCON_AEAD128_TAG_MIN	4	/* shortest truncated tag SP 800-232 allows */
#define ASCON_HASH256_LEN	32
#define ASCON_CXOF128_Z_MAX	256	/* longest customization string, bytes */

/* ---- Ascon-AEAD128 --------------------------------------------------------
 * A nonce must never be used twice with the same key.
 */

/* Encrypt `len` bytes. `ct` receives `len` bytes and may equal `pt`; `tag`
 * receives the full tag. A caller that sends a truncated tag sends its
 * leftmost bytes. */
void ascon_aead128_encrypt(const uint8_t key[ASCON_AEAD128_KEY_LEN],
			   const uint8_t nonce[ASCON_AEAD128_NONCE_LEN],
			   const uint8_t *ad, size_t ad_len, const uint8_t *pt, size_t len,
			   uint8_t *ct, uint8_t tag[ASCON_AEAD128_TAG_LEN]);

/* Decrypt `len` bytes and check the leftmost `tag_len` bytes of the tag,
 * ASCON_AEAD128_TAG_MIN to ASCON_AEAD128_TAG_LEN. Returns 0 if the tag
 * verifies. Returns -1 otherwise, with the `len` bytes of `pt` set to zero.
 * `pt` may equal `ct`. The comparison takes the same time whatever the tag. */
int ascon_aead128_decrypt(const uint8_t key[ASCON_AEAD128_KEY_LEN],
			  const uint8_t nonce[ASCON_AEAD128_NONCE_LEN],
			  const uint8_t *ad, size_t ad_len, const uint8_t *ct, size_t len,
			  const uint8_t *tag, size_t tag_len, uint8_t *pt);

/* ---- Ascon-XOF128 and Ascon-CXOF128 ---------------------------------------
 * Incremental: init, absorb the message in any number of pieces, then squeeze
 * the output in any number of pieces. Absorbing after the first squeeze is
 * not defined.
 */

struct ascon_xof {
	uint64_t s[5];
	uint8_t buf[8];
	uint8_t fill;		/* bytes in buf while absorbing; position while squeezing */
	uint8_t squeezing;
};

void ascon_xof128_init(struct ascon_xof *x);

/* `z` is the customization string. Returns -1, leaving `x` unusable, if it is
 * longer than ASCON_CXOF128_Z_MAX; 0 otherwise. */
int ascon_cxof128_init(struct ascon_xof *x, const uint8_t *z, size_t z_len);

void ascon_xof_absorb(struct ascon_xof *x, const uint8_t *m, size_t len);
void ascon_xof_squeeze(struct ascon_xof *x, uint8_t *out, size_t len);

/* Overwrite a state that held secret input. */
void ascon_xof_wipe(struct ascon_xof *x);

/* ---- Ascon-Hash256 ------------------------------------------------------- */

void ascon_hash256(uint8_t out[ASCON_HASH256_LEN], const uint8_t *m, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ASCON_H */
