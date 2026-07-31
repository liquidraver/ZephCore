/*
 * SPDX-License-Identifier: MIT
 */

#include "ZephyrRNG.h"
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/timing/timing.h>   /* portable CPU cycle counter for the beat */
#include <psa/crypto.h>
#include <string.h>
#include <mesh/Utils.h>

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
/* Pre-RF entropy for the ESP32 HWRNG — see esp32_entropy_begin() below.
 * Source file is added to the build by CMakeLists.txt (ESP32 only). */
#include <bootloader_random.h>
/* RTC-slow clock read for the two-clock beat entropy source (sample_rtc_beat).
 * esp_rtc_get_time_us() links in an app build (verified via the selftest). */
#include <esp_rtc_time.h>
#endif


BUILD_ASSERT(IS_ENABLED(CONFIG_CSPRNG_ENABLED),
	"ZephyrRNG requires CONFIG_CSPRNG_ENABLED for cryptographic key derivation");

namespace mesh {

void ZephyrRNG::random(uint8_t *dest, size_t sz)
{
	/* Retry handles transient TRNG-warmup races; cold-reboot on persistent
	 * failure. Fabricating entropy here would silently produce weak keys
	 * forever (cf. Debian-OpenSSL 2008). k_msleep is illegal from ISR —
	 * all current callers run on main thread or syswq. */
	for (int attempt = 0; attempt < 4; attempt++) {
		if (sys_csrand_get(dest, sz) == 0) return;
		k_msleep(10);
	}
	Utils::cryptoPanicReboot("CSPRNG unavailable after retries");
}

/* ===== Timing-entropy health check =======================================
 *
 * Online health check (NIST SP 800-90B style) for the two-clock beat source
 * below: repetition count + a distinct-value check tracked across all samples
 * in the window with scalar state — no per-sample buffer needed. Detects
 * stuck-source catastrophic failure (e.g. a frozen slow clock). Does not
 * statistically prove entropy quality — that's what the selftest
 * output-diversity run is for.
 *
 * (The former CPU-jitter fallback — Stephan Müller style k_cycle_get_32()
 * delta sampling — was removed: it only ever carried entropy where the cycle
 * counter was already cross-domain from the CPU, and every such board is
 * exactly a board the beat covers. Where the beat is unavailable the counter
 * is same-domain, the loop is deterministic, and jitter yields ~0 bits —
 * measured dead on ESP32 hardware.) */

/* Health statistics for one beat window.  Timing statistics only — never
 * pool contents or derived key material.  Reporting these is standard practice
 * for a NIST SP 800-90B style noise source; reporting the bytes would not be. */
/* Per-stage health reporting can be silenced. The node wants it — it fires
 * once, at first-boot identity generation, and is the only record of what the
 * entropy sources actually did. The selftest tool calls mixIdentitySeed
 * thousands of times and must be able to shut it up after the first few, or
 * the summary drowns in ~12 lines x N. */
static bool s_seed_report_quiet;

#define RNG_RPT(...) do { if (!s_seed_report_quiet) printk(__VA_ARGS__); } while (0)

void ZephyrRNG::setSeedHealthQuiet(bool quiet)
{
	s_seed_report_quiet = quiet;
}

#if defined(ZEPHCORE_RNG_TEST_HOOKS)
/* When set, the HWRNG contribution to mixIdentitySeed is zeroed after each
 * draw — see the header. Test scaffolding, compiled out of production. */
static bool s_test_kill_hwrng;
void ZephyrRNG::setTestKillHWRNG(bool kill) { s_test_kill_hwrng = kill; }
#endif

struct beat_stats {
	int      n_samples;
	int      n_distinct;   /* distinct delta values seen, capped at 8 */
	int      max_consec;   /* longest run of identical deltas */
	uint32_t min_delta;
	uint32_t max_delta;
	bool     ok;
};

/* ===== Universal two-clock beat entropy ==================================
 *
 * One physical entropy source for every board: count CPU cycles elapsed across
 * a fixed interval of an INDEPENDENT low-frequency oscillator. The two clocks
 * come from different sources, so the count fluctuates with the slow
 * oscillator's phase noise — real physical entropy, not the deterministic
 * same-domain loop that CPU-jitter degenerates to where the cycle counter and
 * CPU share a clock (measured dead on ESP32: thousands of identical deltas).
 *
 * FAST counter = timing_counter_get() — portable CPU cycle counter (DWT on
 *   Cortex-M, CCOUNT on Xtensa; both at CPU frequency). Needs
 *   CONFIG_TIMING_FUNCTIONS and a one-time timing_init()/timing_start().
 *
 * SLOW clock = an oscillator in a DIFFERENT domain from the CPU, selected by a
 *   principled rule so the choice is coherent across boards:
 *     - ESP32: the RTC-slow oscillator via esp_rtc_get_time_us() (internal
 *       ~136 kHz RC, independent of the XTAL->PLL CPU path).
 *     - Any board whose Zephyr system timer runs < 1 MHz: that timer IS a
 *       low-frequency oscillator cross-domain from the CPU (e.g. nRF's
 *       32.768 kHz RTC off LFXO/LFRC), so k_cycle_get_32() is a valid slow
 *       clock. REQUIRES the LF clock to be LFXO/LFRC, not synthesised from
 *       HFCLK — true for every BLE-capable nRF config; the health check below
 *       catches it if a board ever violates that.
 *     - Otherwise (system timer at CPU frequency, e.g. bare SysTick): no
 *       independent slow clock is identified and the timing stages are
 *       SKIPPED — a same-domain counter measures a deterministic loop
 *       (~0 bits, measured), so sampling it would only pretend to add
 *       entropy. Such boards (STM32WL SysTick, nRF54L 1 MHz GRTC) rely on
 *       their true TRNG via the CSPRNG stages, which is what the removed
 *       CPU-jitter fallback effectively did anyway.
 *
 * Window = 500 us, from an on-hardware ESP32 sweep (memory/findings.md):
 * 120/250/500/1000 us gave 1.85/3.04/3.81/5.14 bits/sample; 500 us is the knee.
 * Full 32-bit delta is mixed. On ESP32 this is a SECONDARY source (the
 * bootloader_random-seeded HWRNG is primary); on nRF it is the strong
 * non-HWRNG leg.
 *
 * CAVEAT (memory/findings.md): the health stats show the beat VARIES, not that
 * it is random — the selftest output-diversity run is what validates it. */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#define BEAT_SLOW_HZ        1000000ULL
static inline uint64_t beat_slow_ticks(void) { return esp_rtc_get_time_us(); }
#define HAVE_TWO_CLOCK_BEAT 1
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC < 1000000)
#define BEAT_SLOW_HZ        ((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)
static inline uint64_t beat_slow_ticks(void) { return k_cycle_get_32(); }
#define HAVE_TWO_CLOCK_BEAT 1
#endif

#ifdef HAVE_TWO_CLOCK_BEAT
#define BEAT_WINDOW_US    500
/* slow-clock ticks per window; >= 1 guaranteed for any BEAT_SLOW_HZ >= 2 kHz */
#define BEAT_WINDOW_TICKS ((uint32_t)((BEAT_SLOW_HZ * BEAT_WINDOW_US) / 1000000ULL))

static bool sample_two_clock_beat(uint8_t *pool, size_t pool_size,
				  size_t pool_offset, uint32_t duration_ms,
				  struct beat_stats *st = nullptr)
{
	/* Enable the CPU cycle counter once (DWT on Cortex-M; no-op-ish on
	 * Xtensa where CCOUNT always runs). */
	static bool timing_ready;
	if (!timing_ready) {
		timing_init();
		timing_start();
		timing_ready = true;
	}

	int64_t deadline = k_uptime_get() + duration_ms;
	size_t idx = pool_offset;
	uint32_t min_delta = UINT32_MAX, max_delta = 0;
	uint32_t prev_delta = 0;
	int cur_consec = 0, max_consec = 0;
	uint32_t distinct[8] = {0};
	int n_distinct = 0, n_samples = 0;

	while (k_uptime_get() < deadline) {
		uint64_t s0 = beat_slow_ticks();
		uint32_t f0 = (uint32_t)timing_counter_get();
		while ((beat_slow_ticks() - s0) < BEAT_WINDOW_TICKS) {
			/* CPU cycle counter advances while the independent slow
			 * oscillator defines the window; the two drift */
		}
		uint32_t f1 = (uint32_t)timing_counter_get();
		uint32_t delta = f1 - f0;

		/* Mix the FULL delta — entropy spans ~11 bits, not the low byte. */
		pool[idx++ % pool_size] ^= (uint8_t)delta;
		pool[idx++ % pool_size] ^= (uint8_t)(delta >> 8);
		pool[idx++ % pool_size] ^= (uint8_t)(delta >> 16);
		pool[idx++ % pool_size] ^= (uint8_t)(delta >> 24);

		/* Health stats on the low 14 bits (where the beat lives): a
		 * frozen/domain-locked slow clock freezes the delta and trips
		 * the repetition count. */
		uint32_t d14 = delta & 0x3FFF;
		if (n_samples > 0 && d14 == prev_delta) {
			if (++cur_consec > max_consec) max_consec = cur_consec;
		} else {
			cur_consec = 1;
		}
		prev_delta = d14;

		if (n_distinct < 8) {
			bool found = false;
			for (int j = 0; j < n_distinct; j++) {
				if (distinct[j] == d14) { found = true; break; }
			}
			if (!found) distinct[n_distinct++] = d14;
		}

		if (d14 < min_delta) min_delta = d14;
		if (d14 > max_delta) max_delta = d14;

		n_samples++;
	}

	bool ok = (n_samples >= 16)      /* enough samples             */
		&& (max_consec < 32)     /* slow clock not frozen      */
		&& (n_distinct >= 5);    /* beat actually varies       */

	if (st) {
		st->n_samples  = n_samples;
		st->n_distinct = n_distinct;
		st->max_consec = max_consec;
		st->min_delta  = (n_samples > 0) ? min_delta : 0;
		st->max_delta  = max_delta;
		st->ok         = ok;
	}
	return ok;
}
#endif /* HAVE_TWO_CLOCK_BEAT */

/* Count distinct byte values in a buffer — a repetition/adaptive-proportion
 * style health indicator for a CSPRNG draw.  A stuck source collapses this to
 * 1.  Deliberately coarse: one integer per draw, which detects catastrophic
 * failure without meaningfully describing the bytes themselves. */
static int distinct_bytes(const uint8_t *buf, size_t len)
{
	bool seen[256] = {false};
	int n = 0;
	for (size_t i = 0; i < len; i++) {
		if (!seen[buf[i]]) { seen[buf[i]] = true; n++; }
	}
	return n;
}

/* One line per beat window.  `distinct` is capped at 8 by the sampler, so 8/8
 * means "at least 8" — the pass threshold is 5.  `maxrep` is the longest run of
 * identical deltas; >=32 fails.  The span/distinct/maxrep line IS the beat's
 * health — per-sample entropy is characterised offline by the selftest window
 * sweep, not estimated here (an in-path MCV estimate would need a 16k-slot
 * histogram).  Deliberately no statistics on the conditioned output:
 * AES-256-CTR makes any input look uniform, so output statistics would read
 * perfect even for a near-zero-entropy seed.  Entropy is a property of the
 * source. */
#ifdef HAVE_TWO_CLOCK_BEAT
static void report_beat(const char *label, const struct beat_stats *st)
{
	RNG_RPT("[RNG] %s: samples=%d distinct=%d/8 maxrep=%d "
	       "delta=[%u..%u] -> %s\n",
	       label, st->n_samples, st->n_distinct, st->max_consec,
	       st->min_delta, st->max_delta, st->ok ? "PASS" : "FAIL");
}
#endif /* HAVE_TWO_CLOCK_BEAT */

/* ===== Entropy extraction via AES-256-CTR ================================
 *
 * Per crypto consultant (MeshCore upstream PR#2280 author): the
 * conditioning step is most correctly an XOF or stream cipher, not a
 * truncated hash. For our 32-byte Ed25519-seed output the difference
 * is design hygiene rather than security, but the cost is the same
 * order of magnitude (~one SHA-512 vs SHA-256 + two AES-ECB blocks).
 *
 * Construction (NIST SP 800-108 KDF-in-Counter-Mode style):
 *   1. Extract: SHA-256(pool) → 32-byte AES-256 key.
 *   2. Expand:  AES-256-ECB(counter_i) for counter_i = 0, 1, 2 ...
 *               output = concatenation of ciphertext blocks.
 * Plaintext-XOR (true CTR mode) is omitted because plaintext would be
 * all-zero — we want just the keystream.
 *
 * Uses PSA crypto API (already enabled via PSA_WANT_KEY_TYPE_AES +
 * PSA_WANT_ALG_ECB_NO_PADDING in zephcore_common.conf).
 */
static int extract_via_aes_ctr(const uint8_t *pool, size_t pool_len,
			       uint8_t *out, size_t out_len)
{
	psa_status_t status;
	uint8_t key[32];
	size_t key_len = 0;

	/* PSA is idempotent — already initialized via mbedTLS but a defensive
	 * call here costs nothing if it returns PSA_ERROR_ALREADY_EXISTS. */
	(void)psa_crypto_init();

	/* Extract: SHA-256(pool) → AES key.  Open-coded here (NOT Utils::sha256)
	 * on purpose: that wrapper returns void and silently zeroes its output on
	 * PSA failure.  A zeroed key imports fine and AES-ECB(key=0) derives a
	 * fixed, device-independent seed that the all-zero/all-FF degenerate check
	 * cannot catch — every affected unit would share one Ed25519 identity.  We
	 * must hard-fail so the caller (mixIdentitySeed) cryptoPanicReboots. */
	status = psa_hash_compute(PSA_ALG_SHA_256, pool, pool_len,
				  key, sizeof(key), &key_len);
	if (status != PSA_SUCCESS || key_len != sizeof(key)) {
		Utils::secureZeroize(key, sizeof(key));
		return -1;
	}

	/* Import key for AES-256-ECB */
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
	psa_set_key_bits(&attr, 256);

	psa_key_id_t key_id = 0;
	status = psa_import_key(&attr, key, sizeof(key), &key_id);
	/* Wipe stack-resident AES key — secureZeroize survives -Os DSE. */
	Utils::secureZeroize(key, sizeof(key));
	if (status != PSA_SUCCESS) {
		return -1;
	}

	/* Expand: AES-ECB(counter_i) for i = 0, 1, ... */
	uint8_t counter[16] = {0};
	size_t pos = 0;
	int ret = 0;
	while (pos < out_len) {
		uint8_t block[16];
		size_t block_out = 0;
		status = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING,
					    counter, sizeof(counter),
					    block, sizeof(block), &block_out);
		if (status != PSA_SUCCESS || block_out != sizeof(block)) {
			ret = -1;
			break;
		}
		size_t chunk = (out_len - pos < sizeof(block))
			? (out_len - pos) : sizeof(block);
		memcpy(out + pos, block, chunk);
		pos += chunk;

		/* Increment 128-bit counter, big-endian — overflow rolls over.
		 * For our 32-byte output we only ever hit counters 0 and 1. */
		for (int i = sizeof(counter) - 1; i >= 0; i--) {
			if (++counter[i] != 0) break;
		}
		Utils::secureZeroize(block, sizeof(block));
	}

	psa_destroy_key(key_id);
	Utils::secureZeroize(counter, sizeof(counter));
	return ret;
}

void ZephyrRNG::mixIdentitySeed(uint8_t *out, size_t out_len,
				const uint8_t *extra, size_t extra_len)
{
	uint8_t pool[512];
	memset(pool, 0, sizeof(pool));

	/* ESP32 only: give the HWRNG a real entropy source for the duration of
	 * this function.
	 *
	 * WDEV_RANDOM is a PRNG that receives hardware entropy only "provided
	 * Wi-Fi or BT are enabled" (Zephyr drivers/entropy/entropy_esp32.c), and
	 * sys_csrand_get() maps straight to it. Every caller of this function
	 * runs before RF is up, and repeater / room-server builds never enable
	 * RF at all — so stages 1 and 5 below contributed NOTHING on ESP32,
	 * leaving CPU jitter as the only real source. That was observed failing
	 * its health check on ThinkNode M9 hardware while deriving a permanent
	 * identity key.
	 *
	 * bootloader_random_enable() puts the SAR ADC into continuous sampling
	 * and mixes its noise into the HWRNG; Espressif's header explicitly
	 * sanctions calling it from app code when RF is not up. It must be
	 * disabled again before anything else touches the ADC or RF — done at
	 * the end of the collection phase, before AES extraction, so the ADC is
	 * held for as short a window as possible.
	 *
	 * WARNING for future callers: this is unsafe if RF or the ADC is already
	 * in use. Do not call mixIdentitySeed() after bt_enable() or alongside a
	 * battery read on ESP32. */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	bootloader_random_enable();
	RNG_RPT("[RNG] === identity seed health ===\n");
	RNG_RPT("[RNG] esp32 pre-RF entropy (bootloader_random): ENABLED\n");
#else
	RNG_RPT("[RNG] === identity seed health ===\n");
	RNG_RPT("[RNG] platform TRNG is radio-independent (no pre-RF workaround needed)\n");
#endif

	/* Stage 1: early CSPRNG (strong on nRF/MG24; on ESP32 this is only real
	 * because bootloader_random_enable() above is feeding the HWRNG) */
	int rc1 = sys_csrand_get(pool, 64);
#if defined(ZEPHCORE_RNG_TEST_HOOKS)
	if (s_test_kill_hwrng) memset(pool, 0, 64);   /* simulate dead HWRNG */
#endif
	RNG_RPT("[RNG] stage1 csrand    : rc=%d distinct=%d/64\n",
	       rc1, distinct_bytes(pool, 64));

	/* Stage 2: HWINFO unique device ID — uniqueness across devices */
	uint8_t devid[16] = {0};
	ssize_t devid_len = hwinfo_get_device_id(devid, sizeof(devid));
	for (ssize_t i = 0; i < devid_len && i < (ssize_t)sizeof(devid); i++) {
		pool[64 + i] ^= devid[i];
	}
	/* NOT secret — this is the efuse/FICR serial, public and printed at boot.
	 * It contributes uniqueness between devices, never unpredictability. */
	RNG_RPT("[RNG] stage2 hwinfo id  : %d bytes (public — uniqueness only)\n",
	       (int)devid_len);

	/* Stage 3: caller-supplied entropy. A hook for a caller that has its own
	 * physical noise (e.g. an externally sampled ADC/RF value); unused today,
	 * so normally a no-op. Kept because it costs nothing when null and gives
	 * a board a way to inject a source without touching this file. The
	 * internal battery-ADC experiment was removed — a driven divider yielded
	 * no reliable entropy and did not justify the complexity in the key path. */
	if (extra && extra_len > 0) {
		size_t n = (extra_len < 32) ? extra_len : 32;
		for (size_t i = 0; i < n; i++) pool[80 + i] ^= extra[i];
		RNG_RPT("[RNG] stage3 extra    : %d bytes\n", (int)n);
	}

	/* Stage 4: hardware-timing entropy, 200ms — the two-clock beat.
	 *
	 * Skipped where no independent slow clock exists (see the beat header
	 * comment): a same-domain counter would sample a deterministic loop and
	 * only pretend to add entropy. Those boards rely on their true TRNG via
	 * stages 1 and 5.
	 *
	 * Also skipped on POSIX arch (native_sim / Linux): the simulated clock
	 * only advances when Zephyr threads yield, so k_uptime_get() is frozen
	 * while this loop spins → infinite loop.  On Linux we have /dev/urandom
	 * (via sys_csrand_get in stages 1 and 5) which is a far stronger source
	 * than this sampling anyway. */
#if defined(HAVE_TWO_CLOCK_BEAT) && !defined(CONFIG_ARCH_POSIX)
	struct beat_stats js = {};
	bool health_ok = sample_two_clock_beat(pool, sizeof(pool), 112, 200, &js);
	report_beat("stage4 beat 200ms", &js);
	if (!health_ok) {
		RNG_RPT("[RNG] stage4 FAILED — resampling at 400ms\n");
		health_ok = sample_two_clock_beat(pool, sizeof(pool), 112, 400, &js);
		report_beat("stage4 beat 400ms", &js);
		if (!health_ok) {
			RNG_RPT("[RNG] stage4 STILL FAILING — continuing with mixed sources\n");
		}
	}
#else
	RNG_RPT("[RNG] stage4/6 skipped — no independent slow clock (TRNG via csrand only)\n");
#endif /* HAVE_TWO_CLOCK_BEAT && !CONFIG_ARCH_POSIX */

	/* Stage 5: late CSPRNG — catches any mid-boot radio init that
	 * warmed the TRNG during the stage-4 window */
	int rc5 = sys_csrand_get(pool + 368, 64);
#if defined(ZEPHCORE_RNG_TEST_HOOKS)
	if (s_test_kill_hwrng) memset(pool + 368, 0, 64);   /* simulate dead HWRNG */
#endif
	RNG_RPT("[RNG] stage5 csrand    : rc=%d distinct=%d/64\n",
	       rc5, distinct_bytes(pool + 368, 64));

	/* Stage 6: second hardware-timing sample, independent window */
#if defined(HAVE_TWO_CLOCK_BEAT) && !defined(CONFIG_ARCH_POSIX)
	struct beat_stats js6 = {};
	(void)sample_two_clock_beat(pool, sizeof(pool), 432, 50, &js6);
	report_beat("stage6 beat  50ms", &js6);
#endif /* HAVE_TWO_CLOCK_BEAT && !CONFIG_ARCH_POSIX */

	/* Collection done — release the SAR ADC before anything else needs it.
	 * Unconditional: every path below this point either returns normally or
	 * reboots, so there is no path that leaves it enabled. */
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
	bootloader_random_disable();
	RNG_RPT("[RNG] esp32 pre-RF entropy: DISABLED (ADC released)\n");
#endif
	RNG_RPT("[RNG] === end (extracting %u bytes via AES-256-CTR) ===\n",
	       (unsigned)out_len);

	/* Final conditioning: AES-256-CTR over the pool. Extracts a 32-byte
	 * AES key via SHA-256(pool), then expands to out_len bytes via
	 * AES-ECB on a 128-bit counter. Per crypto consultant guidance —
	 * see extract_via_aes_ctr() for full rationale. */
	if (extract_via_aes_ctr(pool, sizeof(pool), out, out_len) != 0) {
		Utils::cryptoPanicReboot("AES-CTR seed extraction failed");
	}

	/* Output sanity check — reject all-zero / all-0xFF (catastrophic
	 * failure of every source). */
	bool all_zero = true, all_ff = true;
	for (size_t i = 0; i < out_len; i++) {
		if (out[i] != 0x00) all_zero = false;
		if (out[i] != 0xFF) all_ff = false;
	}
	if (all_zero || all_ff) {
		Utils::cryptoPanicReboot("degenerate seed output (all-zero / all-FF)");
	}

	/* Wipe sensitive intermediate buffers — secureZeroize survives the
	 * -Os dead-store-elimination that would silently elide plain memset
	 * on stack locals that are never read again. */
	Utils::secureZeroize(pool, sizeof(pool));
	Utils::secureZeroize(devid, sizeof(devid));
}

void ZephyrRNG::generateFirstBootIdentity(LocalIdentity &out_identity)
{
	uint8_t seed[32];

	mixIdentitySeed(seed, sizeof(seed));
	out_identity.fromSeed(seed);

	/* Reserved-prefix guard — MeshCore protocol treats pub_key[0] of
	 * 0x00/0xFF as reserved markers. With a working CSPRNG the first
	 * attempt almost always passes (P(reserved) = 2/256); the cap +
	 * panic-reboot is a stuck-source backstop. */
	int attempt = 0;
	while (out_identity.pub_key[0] == 0x00 || out_identity.pub_key[0] == 0xFF) {
		if (++attempt > 100) {
			Utils::cryptoPanicReboot("identity gen stuck on reserved prefix");
		}
		mixIdentitySeed(seed, sizeof(seed));
		out_identity.fromSeed(seed);
	}

	Utils::secureZeroize(seed, sizeof(seed));
}

} /* namespace mesh */
