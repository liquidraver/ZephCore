/*
 * SPDX-License-Identifier: MIT
 * The seed of the zcpfs generator (lib/zcpfs/SPEC.md 5, 5.1). See ZcpfsCommon.h.
 */

#include "ZcpfsCommon.h"

#include <adapters/datastore/ZephyrFsUtil.h>
#include <adapters/rng/ZephyrRNG.h>

/* One file for the node, whatever its role. */
#define ZCPFS_SEED_FILE "/lfs/zcpfs_seed"

int zcpfsSeed(zcpfs_drbg *g, mesh::RNG &rng)
{
	uint8_t hw[32], saved[32], next[32];
	size_t n = 0;
	int r = 0;

	rng.random(hw, sizeof(hw));
	if (zephcore_fs_read_file(ZCPFS_SEED_FILE, saved, sizeof(saved), &n) && n == sizeof(saved)) {
		r |= ZCPFS_SEED_FROM_FILE;
	} else {
		/* no seed yet: as good as the identity key (5.1) */
		mesh::ZephyrRNG::mixIdentitySeed(saved, sizeof(saved));
	}
	zcpfs_drbg_seed(g, hw, saved);

	/* the next boot's seed, before anything else is drawn (5) */
	zcpfs_drbg_generate(g, next, sizeof(next));
	if (zephcore_fs_atomic_replace(ZCPFS_SEED_FILE, next, sizeof(next), "zcpfs seed")) {
		r |= ZCPFS_SEED_WRITTEN;
	}

	zcpfs_wipe(hw, sizeof(hw));
	zcpfs_wipe(saved, sizeof(saved));
	zcpfs_wipe(next, sizeof(next));
	return r;
}
