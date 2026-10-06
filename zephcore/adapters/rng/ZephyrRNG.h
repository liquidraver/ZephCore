/*
 * SPDX-License-Identifier: MIT
 * Zephyr CSPRNG implementation
 */

#pragma once

#include <mesh/Utils.h>
#include <mesh/Identity.h>
#include <stddef.h>

namespace mesh {

class ZephyrRNG : public RNG {
public:
	void random(uint8_t *dest, size_t sz) override;

	/* Layered entropy mixer for the one-time identity key: CSPRNG draws early and
	 * late, the device ID, optional caller data and two windows of the two-clock
	 * beat, conditioned via AES-256-CTR. Health-checked; reboots on degenerate
	 * output. Blocks ~450 ms. */
	static void mixIdentitySeed(uint8_t *out, size_t out_len,
				    const uint8_t *extra = nullptr,
				    size_t extra_len = 0);

#if defined(ZEPHCORE_RNG_TEST_HOOKS)
	/* TEST ONLY — enabled by a compile define, set only by tools/rng_selftest
	 * (never by production builds). When active, mixIdentitySeed zeroes the
	 * HWRNG (sys_csrand_get) contribution to the pool, so a diversity test
	 * measures the two-clock beat alone. Answers "if the hardware RNG returns
	 * nothing, does ZephyrRNG still produce diverse keys?" On a single device
	 * the device-id (stage 2) is constant across runs, so the beat is then the
	 * ONLY varying source. */
	static void setTestKillHWRNG(bool kill);
#endif

	/* Silence mixIdentitySeed's per-stage health report. Default off; for
	 * tools/rng_selftest only. */
	static void setSeedHealthQuiet(bool quiet);

	/* First-boot identity generation: mixes a seed, derives the Ed25519 keypair,
	 * retries (up to 100 times) on a reserved 0x00/0xFF public-key prefix, and
	 * wipes the seed. */
	static void generateFirstBootIdentity(LocalIdentity &out_identity);
};

} /* namespace mesh */
