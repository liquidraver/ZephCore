/*
 * SPDX-License-Identifier: MIT
 * Internal to the zcpfs library: the helpers its files share. Ascon (lib/ascon)
 * is reached only through zcpfs_prim.c, Curve25519 (lib/monocypher) only
 * through zcpfs_curve.c.
 */

#ifndef ZCPFS_INTERNAL_H
#define ZCPFS_INTERNAL_H

#include "zcpfs.h"

/* ---- zcpfs_prim.c -------------------------------------------------------- */

/* KDF whose output of len1 + len2 bytes goes to two buffers. */
void zcpfs_kdf_split(const char *label, const struct zcpfs_part *parts, size_t n_parts,
		     uint8_t *out1, size_t len1, uint8_t *out2, size_t len2);

/* Ascon-AEAD128 with the tag cut to ZCPFS_TAG_LEN. `out` receives len +
 * ZCPFS_TAG_LEN bytes, ciphertext then tag; it may equal `pt`. */
void zcpfs_aead_seal(const uint8_t key[16], const uint8_t nonce[16], const uint8_t *ad,
		     size_t ad_len, const uint8_t *pt, size_t len, uint8_t *out);

/* `body` is ciphertext then tag, at least ZCPFS_TAG_LEN bytes. `pt` receives
 * body_len - ZCPFS_TAG_LEN bytes, all zero if the tag does not verify. */
int zcpfs_aead_open(const uint8_t key[16], const uint8_t nonce[16], const uint8_t *ad,
		    size_t ad_len, const uint8_t *body, size_t body_len, uint8_t *pt);

/* ---- zcpfs_curve.c ------------------------------------------------------- */

/* X25519(scalar, point). ZCPFS_ERR_WEAK if the result is all zero. */
int zcpfs_dh(uint8_t out[32], const uint8_t scalar[32], const uint8_t point[32]);

void zcpfs_dh_public(uint8_t pub[32], const uint8_t scalar[32]);

/* The X25519 public key that belongs to an Ed25519 public key. */
void zcpfs_ed_to_x(uint8_t x[32], const uint8_t ed[32]);

/* ELL2 of section 19.1. */
void zcpfs_ell2(uint8_t point[32], const uint8_t in[32]);

/* ---- zcpfs_handshake.c --------------------------------------------------- */

/* The two chain keys and the generator feed of a master secret (8.3, 19.3). */
void zcpfs_keys_from_master(const uint8_t master[32], struct zcpfs_keys *keys);

#endif /* ZCPFS_INTERNAL_H */
