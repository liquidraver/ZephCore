/*
 * SPDX-License-Identifier: MIT
 * The seed of the zcpfs generator (lib/zcpfs/SPEC.md 5, 5.1). See ZcpfsCommon.h.
 */

#include "ZcpfsCommon.h"

#include <adapters/datastore/ZephyrFsUtil.h>
#include <adapters/rng/ZephyrRNG.h>

/* One file for the node, whatever its role. */
#define ZCPFS_SEED_FILE "/lfs/zcpfs_seed"

/* Fresh entropy: the identity mixer at boot, else more of the RNG. */
static void fresh(uint8_t out[32], mesh::RNG &rng, bool boot)
{
	if (boot) {
		mesh::ZephyrRNG::mixIdentitySeed(out, 32);
	} else {
		rng.random(out, 32);
	}
}

int zcpfsSeed(zcpfs_drbg *g, mesh::RNG &rng, bool boot)
{
	uint8_t hw[32], saved[32], next[32];
	size_t n = 0;
	int r = boot ? 0 : ZCPFS_SEED_LATE;

	rng.random(hw, sizeof(hw));
	if (zephcore_fs_read_file(ZCPFS_SEED_FILE, saved, sizeof(saved), &n) && n == sizeof(saved)) {
		r |= ZCPFS_SEED_FROM_FILE;
	} else {
		/* no seed yet: at boot, as good as the identity key (5.1) */
		fresh(saved, rng, boot);
	}
	zcpfs_drbg_seed(g, hw, saved);

	/* the next boot's seed, before anything else is drawn (5) */
	zcpfs_drbg_generate(g, next, sizeof(next));
	if (zephcore_fs_atomic_replace(ZCPFS_SEED_FILE, next, sizeof(next), "zcpfs seed")) {
		r |= ZCPFS_SEED_WRITTEN;
	} else {
		/* The next boot will read this boot's seed again. Leave the state it
		 * would rebuild from a repeated hardware output: mix in fresh entropy,
		 * at boot the identity mixer, which has its own timing entropy. */
		fresh(next, rng, boot);
		zcpfs_drbg_mix(g, next, sizeof(next));
		r |= ZCPFS_SEED_MIXED;
	}

	zcpfs_wipe(hw, sizeof(hw));
	zcpfs_wipe(saved, sizeof(saved));
	zcpfs_wipe(next, sizeof(next));
	return r;
}

static zcpfs_drbg s_boot;
static int s_boot_seed = -1;		/* ZCPFS_SEED_* of s_boot, -1: none */

void zcpfsSeedAtBoot(mesh::RNG &rng)
{
	s_boot_seed = zcpfsSeed(&s_boot, rng, true);
}

int zcpfsTakeSeed(zcpfs_drbg *g, mesh::RNG &rng)
{
	int r = s_boot_seed;

	if (r < 0) {
		return zcpfsSeed(g, rng, false);
	}
	*g = s_boot;
	zcpfs_wipe(&s_boot, sizeof(s_boot));
	s_boot_seed = -1;
	return r;
}
