/*
 * SPDX-License-Identifier: MIT
 * Curve25519 for zcpfs, from the vendored Monocypher. The only file that calls
 * it.
 */

#include "zcpfs_internal.h"

#include <monocypher.h>

int zcpfs_dh(uint8_t out[32], const uint8_t scalar[32], const uint8_t point[32])
{
	static const uint8_t zero[32];

	crypto_x25519(out, scalar, point);
	return zcpfs_ct_equal(out, zero, 32) ? ZCPFS_ERR_WEAK : ZCPFS_OK;
}

void zcpfs_dh_public(uint8_t pub[32], const uint8_t scalar[32])
{
	crypto_x25519_public_key(pub, scalar);
}

void zcpfs_ed_to_x(uint8_t x[32], const uint8_t ed[32])
{
	crypto_eddsa_to_x25519(x, ed);
}

void zcpfs_ell2(uint8_t point[32], const uint8_t in[32])
{
	crypto_elligator_map(point, in);
}
