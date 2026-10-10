/*
 * SPDX-License-Identifier: MIT
 * Section 5: the generator. Every output replaces the key it came from, so an
 * extracted state does not give earlier outputs.
 */

#include "zcpfs_internal.h"

#include <string.h>

void zcpfs_drbg_seed(struct zcpfs_drbg *g, const uint8_t hw[32], const uint8_t saved[32])
{
	const struct zcpfs_part parts[] = { { hw, 32 }, { saved, 32 } };

	zcpfs_kdf("drbg.seed", parts, 2, g->k, sizeof(g->k));
}

void zcpfs_drbg_generate(struct zcpfs_drbg *g, uint8_t *out, size_t len)
{
	const struct zcpfs_part parts[] = { { g->k, 32 } };
	uint8_t next[32];

	/* the first 32 bytes are the next key, the rest is the output */
	zcpfs_kdf_split("drbg.gen", parts, 1, next, sizeof(next), out, len);
	memcpy(g->k, next, sizeof(g->k));
	zcpfs_wipe(next, sizeof(next));
}

void zcpfs_drbg_mix(struct zcpfs_drbg *g, const uint8_t *x, size_t len)
{
	const struct zcpfs_part parts[] = { { g->k, 32 }, { x, len } };
	uint8_t next[32];

	zcpfs_kdf("drbg.mix", parts, 2, next, sizeof(next));
	memcpy(g->k, next, sizeof(g->k));
	zcpfs_wipe(next, sizeof(next));
}
