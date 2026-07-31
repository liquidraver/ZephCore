/*
 * ZephCore RNG selftest
 * SPDX-License-Identifier: MIT
 *
 * Generates N identity seeds through the real ZephyrRNG::mixIdentitySeed and
 * checks them for the failure mode that actually matters: lack of diversity.
 *
 * WHY NOT "test one key and see if it looks random":
 * mixIdentitySeed ends in AES-256-CTR conditioning, and a cryptographic
 * conditioner makes ANY input look uniform. A seed built from one bit of real
 * entropy passes every statistical test you can point at it. Entropy is a
 * property of the generating process, not of the bytes produced — which is why
 * NIST SP 800-90B health-tests the noise source and does not accept output
 * testing as evidence.
 *
 * The real-world failure of a weak generator is not "the key looks wrong", it
 * is "the key is not unique" — Debian OpenSSL 2008 (~15 bits, 32767 possible
 * keys) was found by noticing duplicates, and "Mining your Ps and Qs" (2012)
 * found thousands of embedded devices sharing keys because they generated them
 * at boot before entropy was available. That is exactly our situation, so that
 * is exactly what this measures.
 *
 * WHY REPEAT ON ONE DEVICE rather than compare across devices:
 * stage 2 of the mixer folds in the hwinfo device ID, which differs between
 * devices but is constant on one. A cross-device comparison would therefore
 * look healthy even with zero real entropy — the device IDs alone would
 * separate the keys and mask the bug. Repeating on a single device holds that
 * differentiator constant and isolates the sources under suspicion.
 *
 * The tool never reads or writes stored identity. A device under test keeps
 * whatever key it already had.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include <ZephyrRNG.h>

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
/* RTC-slow-clock beat diagnostic. ESP32's k_cycle_get_32() is CCOUNT, same
 * clock domain as the CPU, so CPU-jitter sampling measures a deterministic
 * loop (proven on hardware: maxrep in the thousands, ~0 bits). The RTC slow
 * clock is a SEPARATE oscillator (internal ~136 kHz RC, or a 32 kHz crystal)
 * that drifts independently of the main-crystal->PLL->240 MHz path — so the
 * number of CPU cycles that elapse across a fixed RTC interval fluctuates, and
 * that fluctuation is real physical entropy. This is what nRF gets for free
 * (its system timer IS the 32 kHz RTC). This diagnostic MEASURES that beat; it
 * does not touch identity generation. */
#include <esp_rtc_time.h>
#define HAVE_RTC_BEAT 1
#endif

#ifndef RNG_SELFTEST_N
#define RNG_SELFTEST_N 1024
#endif

#define SEED_LEN 32          /* Ed25519 seed size — what identity gen uses */
#define FP_LEN    8          /* fingerprint bytes kept per seed            */

/* 8-byte fingerprints, not 4. At N=5000 a 4-byte fingerprint has a ~0.3%
 * chance of colliding BY ACCIDENT (5000^2 / 2 / 2^32), which would report a
 * spurious duplicate and discredit the whole run. At 8 bytes the accidental
 * rate is ~10^-12 — any duplicate reported is a real one. */
static uint8_t fingerprints[RNG_SELFTEST_N][FP_LEN];

static int popcount8(uint8_t x)
{
	int n = 0;
	while (x) { n += x & 1; x >>= 1; }
	return n;
}

/* Hamming distance between two seeds, in bits. Two independent 256-bit values
 * differ in ~128 bits on average; a distribution pulled well below that means
 * the seeds are correlated even when no exact duplicate ever appears. */
static int hamming(const uint8_t *a, const uint8_t *b, size_t len)
{
	int d = 0;
	for (size_t i = 0; i < len; i++) d += popcount8(a[i] ^ b[i]);
	return d;
}

#ifdef HAVE_RTC_BEAT
/* -log2(mcv/n) in milli-bits — SP 800-90B Most Common Value min-entropy,
 * integer math. Understates slightly (linear log2 interpolation), the safe
 * direction for an entropy claim. */
static uint32_t min_entropy_mb(uint32_t mcv, uint32_t n)
{
	if (mcv == 0 || mcv >= n) return 0;
	uint32_t ratio_q10 = (uint32_t)(((uint64_t)n * 1024u) / mcv); /* (n/mcv)<<10 */
	uint32_t ip   = 31u - (uint32_t)__builtin_clz(ratio_q10);
	uint32_t base = 1u << ip;
	uint32_t frac = (uint32_t)(((uint64_t)(ratio_q10 - base) * 1000u) / base);
	uint32_t lg   = ip * 1000u + frac;             /* log2(ratio_q10)*1000 */
	return (lg > 10000u) ? (lg - 10000u) : 0u;     /* subtract log2(1024) */
}

/* 14-bit fold of the delta (0x3FFF = 16384 slots). The first run showed a
 * ~1681-cycle span, so the entropy lives across ~11 bits, not the low 8 — the
 * low-byte-only estimate threw most of it away. 14 bits covers spans up to
 * 16384 without wrapping; a wider span folds and only ever UNDERcounts
 * distinctness (merges values), which understates entropy — safe. uint16_t is
 * enough since per-window sample count < 65535. */
static uint16_t beat_hist[16384];

/* Sweep several RTC-window sizes and report full-delta min-entropy for each,
 * so we can pick the parameter before touching the key path. Characterization
 * ONLY — nothing here feeds identity generation. */
static void rtc_beat_diagnostic(void)
{
	const int      K = 4096;                       /* samples per window   */
	const uint32_t windows_us[] = { 120, 250, 500, 1000 };

	printk("\n");
	printk("--------------------------------------------------------\n");
	printk(" RTC-beat window sweep (ESP32) — %d samples per window\n", K);
	printk(" CPU cycles per fixed RTC-slow interval; full-delta entropy\n");
	printk("--------------------------------------------------------\n");
	printk(" window   span   distinct  mcv      bits/samp   bits/sec\n");
	printk(" ------   ----   --------  -------  ---------   --------\n");

	for (size_t w = 0; w < ARRAY_SIZE(windows_us); w++) {
		uint32_t win = windows_us[w];
		memset(beat_hist, 0, sizeof(beat_hist));
		uint32_t cyc_min = UINT32_MAX, cyc_max = 0;

		for (int i = 0; i < K; i++) {
			uint64_t r0 = esp_rtc_get_time_us();
			uint32_t c0 = k_cycle_get_32();
			while ((esp_rtc_get_time_us() - r0) < win) {
				/* CPU clock advances while the independent RTC
				 * oscillator defines the window; they drift */
			}
			uint32_t c1 = k_cycle_get_32();
			uint32_t d  = c1 - c0;

			if (d < cyc_min) cyc_min = d;
			if (d > cyc_max) cyc_max = d;
			beat_hist[d & 0x3FFF]++;
		}

		uint32_t mcv = 0;
		int distinct = 0;
		for (int v = 0; v < 16384; v++) {
			if (beat_hist[v]) distinct++;
			if (beat_hist[v] > mcv) mcv = beat_hist[v];
		}
		uint32_t per_mb = min_entropy_mb(mcv, K);
		/* bits/sec = bits/sample * 1e6 / window_us */
		uint32_t bits_per_sec = (uint32_t)(((uint64_t)per_mb * 1000000u)
						   / (win * 1000u));

		printk(" %4u us  %5u  %6d    %6u   %u.%03u      %u\n",
		       win, cyc_max - cyc_min, distinct, mcv,
		       per_mb / 1000u, per_mb % 1000u, bits_per_sec);
	}

	printk("--------------------------------------------------------\n");
	printk(" pick: highest bits/sample for cleanest per-sample quality,\n");
	printk(" or highest bits/sec if throughput-bound (200ms budget needs\n");
	printk(" >256 bits, so any row over ~1300 bits/sec has 10x margin).\n");
	printk("\n");
}
#endif /* HAVE_RTC_BEAT */

int main(void)
{
	/* Console settle — USB CDC enumerates after boot on some boards. */
	k_msleep(2000);

#ifdef HAVE_RTC_BEAT
	/* Characterize the candidate second source BEFORE the diversity run,
	 * so its numbers are the first thing on the console. */
	rtc_beat_diagnostic();
#endif

	printk("\n");
	printk("========================================================\n");
	printk(" ZephCore RNG selftest — identity seed diversity\n");
	printk(" N=%d seeds x 32 bytes, ~250 ms each (~%d s total)\n",
	       RNG_SELFTEST_N, (RNG_SELFTEST_N * 250) / 1000);
	printk("========================================================\n\n");

#ifdef RNG_KILL_HWRNG
	/* Zero the HWRNG contribution: the diversity run below then measures the
	 * two-clock beat ALONE. On this single device the device-id (stage 2) is
	 * constant across all seeds, so if this run still shows 0 duplicates and
	 * Hamming ~128, the beat by itself carries the key — i.e. ZephyrRNG
	 * survives a completely dead hardware RNG. */
	mesh::ZephyrRNG::setTestKillHWRNG(true);
	printk("*** KILL_HWRNG: hardware RNG zeroed — measuring the beat ALONE ***\n\n");
#endif

	uint8_t seed[SEED_LEN];
	uint8_t prev[SEED_LEN];

	int  duplicates   = 0;
	int  ham_min      = 10000;
	int  ham_max      = -1;
	int64_t ham_sum   = 0;
	int  ham_n        = 0;
	int  first_dup_a  = -1, first_dup_b = -1;

	int64_t t_start = k_uptime_get();

	for (int i = 0; i < RNG_SELFTEST_N; i++) {
		/* Per-stage health output comes from mixIdentitySeed itself. Let
		 * it through for the first two seeds so the sources are on the
		 * record, then actually silence it — at N=1024 that is ~12,000
		 * lines that would bury the summary. */
		if (i == 2) {
			printk("\n[selftest] per-stage output silenced from here "
			       "(%d more seeds); summary follows at the end\n\n",
			       RNG_SELFTEST_N - 2);
			mesh::ZephyrRNG::setSeedHealthQuiet(true);
		}

		mesh::ZephyrRNG::mixIdentitySeed(seed, sizeof(seed));

		/* Fingerprint + duplicate scan against everything so far. */
		memcpy(fingerprints[i], seed, FP_LEN);
		for (int j = 0; j < i; j++) {
			if (memcmp(fingerprints[i], fingerprints[j], FP_LEN) == 0) {
				duplicates++;
				if (first_dup_a < 0) { first_dup_a = j; first_dup_b = i; }
				break;
			}
		}

		/* Consecutive Hamming distance. */
		if (i > 0) {
			int d = hamming(seed, prev, SEED_LEN);
			if (d < ham_min) ham_min = d;
			if (d > ham_max) ham_max = d;
			ham_sum += d;
			ham_n++;
		}
		memcpy(prev, seed, SEED_LEN);

		/* Progress every 10%, so a long run visibly lives. */
		int step = RNG_SELFTEST_N / 10;
		if (step > 0 && ((i + 1) % step) == 0) {
			printk("[selftest] %d/%d seeds (%d%%) dup=%d\n",
			       i + 1, RNG_SELFTEST_N,
			       ((i + 1) * 100) / RNG_SELFTEST_N, duplicates);
		}
	}

	int64_t elapsed = k_uptime_get() - t_start;

	/* Mean to 2 decimals without floating point. */
	int ham_mean_x100 = ham_n ? (int)((ham_sum * 100) / ham_n) : 0;

	printk("\n");
	printk("========================================================\n");
	printk(" RESULTS  (%d seeds in %lld s)\n", RNG_SELFTEST_N, elapsed / 1000);
	printk("--------------------------------------------------------\n");
	printk(" exact duplicates : %d\n", duplicates);
	if (duplicates) {
		printk("   first collision: seed #%d == seed #%d\n",
		       first_dup_a, first_dup_b);
	}
	printk(" hamming distance : min=%d max=%d mean=%d.%02d (expect ~128)\n",
	       ham_min, ham_max, ham_mean_x100 / 100, ham_mean_x100 % 100);
	printk("--------------------------------------------------------\n");

	/* Verdict. Deliberately conservative wording: passing means "no evidence
	 * of a problem at this sample count", never "the entropy is good". A
	 * clean run at N only rules out an effective-entropy floor below about
	 * 2*log2(N) bits — see the note in CMakeLists.txt. */
	bool bad_dup  = (duplicates > 0);
	bool bad_ham  = (ham_n > 0) && (ham_mean_x100 < 11000 || ham_mean_x100 > 14500);

	if (bad_dup) {
		printk(" VERDICT: FAIL — duplicate seeds. The generator is\n");
		printk("          producing a small key space. Do NOT ship.\n");
	} else if (bad_ham) {
		printk(" VERDICT: FAIL — seeds are correlated (mean far from\n");
		printk("          128 bits). Do NOT ship.\n");
	} else {
		printk(" VERDICT: no evidence of correlation at N=%d.\n",
		       RNG_SELFTEST_N);
		printk("          Rules out an entropy floor below ~%d bits.\n",
		       2 * (31 - __builtin_clz(RNG_SELFTEST_N)));
		printk("          This is NOT proof the entropy is sufficient.\n");
	}
	printk("========================================================\n");
	printk("\n[selftest] done — halting. Reflash normal firmware.\n");

	return 0;
}
