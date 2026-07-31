/*
 * SPDX-License-Identifier: MIT
 * Zephyr CSPRNG implementation
 */

#pragma once

#include <mesh/RNG.h>
#include <mesh/Identity.h>
#include <stddef.h>

namespace mesh {

class ZephyrRNG : public RNG {
public:
	void random(uint8_t *dest, size_t sz) override;

	/* Layered entropy mixer for one-time first-boot identity-key
	 * generation. Combines:
	 *   1. sys_csrand_get (early)        — CSPRNG; on ESP32 only real
	 *                                       because bootloader_random seeds
	 *                                       the HWRNG (else pre-RF = 0 bits)
	 *   2. HWINFO unique device ID       — per-device uniqueness (public)
	 *   3. Optional caller-supplied data — e.g. external noise samples
	 *   4. Hardware-timing entropy       — two-clock beat (ESP32 RTC-slow,
	 *                                       nRF/MG24 32 kHz RTC); skipped on
	 *                                       boards with no independent slow
	 *                                       clock (they rely on their TRNG)
	 *   5. sys_csrand_get (late)         — second HWRNG draw
	 *   6. Hardware-timing entropy #2    — independent window
	 * Conditioned via AES-256-CTR (SHA-256(pool) → key, ECB on counter).
	 * On ESP32 the primary source is the bootloader_random-seeded HWRNG
	 * (stages 1+5); the beat (4+6) is a physical second source. On nRF the
	 * CSPRNG and the beat are both independently strong. Health-checked;
	 * reboots on degenerate output. Blocks ~450ms — first-boot identity only.
	 *
	 * Output is suitable as an Ed25519 seed regardless of platform
	 * TRNG state at boot. */
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

	/* Silence mixIdentitySeed's per-stage entropy health report.
	 *
	 * Default OFF — the node prints it once, at first-boot identity
	 * generation, and it is the only record of what the entropy sources
	 * actually did for a key that is then permanent. Do not silence it there.
	 *
	 * Exists for tools/rng_selftest, which calls mixIdentitySeed thousands of
	 * times: without this the ~12 lines per seed bury the summary. */
	static void setSeedHealthQuiet(bool quiet);

	/* End-to-end first-boot identity generation. Mixes a fresh seed,
	 * derives the Ed25519 keypair, and retries (up to 100×) if the
	 * MeshCore protocol-reserved 0x00/0xFF public-key prefix happens
	 * to land. Panics-and-reboots on cap exhaustion (essentially
	 * impossible with a working CSPRNG: P(100 reserved in a row) ≈ 10⁻²¹¹).
	 * Wipes the intermediate seed before return.
	 *
	 * Use this from main()'s `loadIdentity` fall-through path instead
	 * of open-coding the mix+derive+retry+zeroize sequence. */
	static void generateFirstBootIdentity(LocalIdentity &out_identity);
};

} /* namespace mesh */
