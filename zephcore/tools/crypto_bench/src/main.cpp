/*
 * ZephCore crypto benchmark
 * SPDX-License-Identifier: MIT
 *
 * Answers one question: does ZephCore's crypto cost enough to be worth
 * accelerating? See HANDOVER_crypto_hw_accel.md — every option in that
 * document (a second Zephyr-crypto-API backend in Utils.cpp, nrfxlib PSA
 * drivers in west.yml, or the free software wins) is gated on numbers that
 * nobody has ever taken.
 *
 * WHY A SEPARATE IMAGE rather than a CLI command on a live node:
 * the measurement has to be free of mesh-loop, BLE and LoRa-RX interference,
 * and it runs each primitive hundreds of times back to back — which would
 * starve exactly those subsystems. Same reasoning as tools/rng_selftest.
 *
 * WHAT IT MEASURES, and why the answer is a RATIO not a number:
 * the absolute cost of SHA-256 is uninteresting on its own. What decides the
 * question is crypto time as a fraction of packet AIRTIME — at SF11/250 kHz a
 * packet occupies the channel for hundreds of milliseconds, so a per-packet
 * crypto bill in the tens of microseconds is noise, and the whole hardware
 * exploration closes with a documented "no". The final table does that
 * division for you, per LoRa preset, using the SAME airtime formula the node
 * uses (LoRaRadio::getAirtimeMillis) so the comparison is apples to
 * apples.
 *
 * It measures the SHIPPING code: src/Utils.cpp, src/Identity.cpp and
 * src/Packet.cpp are compiled straight from the main tree, not copied. What
 * is timed here is byte-for-byte what the node runs.
 *
 * The tool never reads or writes stored identity, and never touches the
 * radio, flash or BLE. A device under test keeps whatever key it had.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>
#include <psa/crypto.h>
#include <string.h>

#include <mesh/Utils.h>
#include <mesh/Identity.h>
#include <mesh/Packet.h>

using namespace mesh;

#ifndef CRYPTO_BENCH_ITERS
#define CRYPTO_BENCH_ITERS 200
#endif

/* Ed25519 is ~3 orders of magnitude slower than the symmetric primitives.
 * At the default iteration count a full sweep would take minutes and tell us
 * nothing extra — the variance on an operation that long is negligible. */
#ifndef CRYPTO_BENCH_ITERS_SLOW
#define CRYPTO_BENCH_ITERS_SLOW 20
#endif

/* Payload sizes. 184 is MAX_PACKET_PAYLOAD; calculatePacketHash hashes
 * 1 + payload_len, so the hash input tops out at 185. 60 is a typical text
 * message, 100 a typical advert. */
#define PAYLOAD_TYPICAL   60
#define PAYLOAD_ADVERT   100
#define PAYLOAD_MAX      MAX_PACKET_PAYLOAD

/* Worst case for the group-channel loop at Mesh.cpp:398. */
#ifndef CRYPTO_BENCH_CHANNELS
#define CRYPTO_BENCH_CHANNELS 40
#endif

/* Anti-elision. Without a visible consumer the optimiser is entitled to
 * delete calls whose output is unused, and at -Os it does.
 *
 * It reads only the first and last byte, NOT the whole buffer: the sink runs
 * INSIDE the timed region, and summing 176 bytes would add real work to every
 * AES measurement. Two bytes is enough — the compiler cannot prove the rest
 * of the buffer was unnecessary to produce them. */
static volatile uint32_t g_sink;

static inline void sink(const uint8_t *p, size_t n)
{
	g_sink += (uint32_t)p[0] + (uint32_t)p[n - 1];
}

/* ---------------------------------------------------------------- timing */

/* Zephyr's portable cycle counter (DWT on Cortex-M, CCOUNT on Xtensa) — the
 * same mechanism ZephyrRNG's two-clock beat uses. See
 * memory/zephyrrng-entropy.md for the platform traps.
 *
 * MIN, not mean, is the headline figure: every perturbation (interrupt,
 * cache miss, flash wait state) can only ADD time, so the minimum over N runs
 * is the closest estimate of the true cost. Mean is printed alongside because
 * a large min/mean gap is itself information — it says the operation is being
 * interfered with, which matters on a node where it shares a CPU with the
 * radio ISR. */
struct BenchResult {
	const char *name;
	uint64_t min_ns;
	uint64_t mean_ns;
	int iters;
};

static uint64_t cycles_to_ns(uint64_t cycles)
{
	return timing_cycles_to_ns(cycles);
}

/* Parameter names are underscore-prefixed on purpose: the preprocessor
 * substitutes macro arguments after `.` too, so a parameter called `iters`
 * would rewrite the `(dst).iters` member access into `(dst).200`. */
#define BENCH(dst, _label, _n, stmt)                                        \
	do {                                                                \
		uint64_t _min = UINT64_MAX, _sum = 0;                       \
		for (int _i = 0; _i < (_n); _i++) {                         \
			timing_t _t0 = timing_counter_get();                \
			{ stmt; }                                           \
			timing_t _t1 = timing_counter_get();                \
			uint64_t _c = timing_cycles_get(&_t0, &_t1);        \
			if (_c < _min) _min = _c;                           \
			_sum += _c;                                         \
		}                                                           \
		(dst).name    = (_label);                                   \
		(dst).min_ns  = cycles_to_ns(_min);                         \
		(dst).mean_ns = cycles_to_ns(_sum / (uint64_t)(_n));        \
		(dst).iters   = (_n);                                       \
	} while (0)

/* us with 3 decimals, integer math — no CBPRINTF_FP_SUPPORT needed. */
static void print_result(const BenchResult *r)
{
	uint64_t mn = r->min_ns, mu = r->mean_ns;
	printk(" %-34s %6llu.%03llu %8llu.%03llu   %4d\n",
	       r->name, mn / 1000u, mn % 1000u, mu / 1000u, mu % 1000u, r->iters);
}

static void print_header(const char *section)
{
	printk("\n%s\n", section);
	printk(" %-34s %10s %12s   %4s\n", "operation", "min us", "mean us", "N");
	printk(" ---------------------------------- ---------- ------------   ----\n");
}

/* --------------------------------------------------------------- airtime */

/* Integer port of LoRaRadio::getAirtimeMillis(), in microseconds.
 * Deliberately mirrors that function line for line — including the
 * preamble-length rule and the LDRO threshold tied to symbol time (not to
 * SF), because a divergence here would silently distort the only comparison
 * this tool exists to make. */
static uint32_t preamble_syms(uint8_t sf)
{
	return (sf <= 8) ? 32u : 16u;   /* PR #1954 parity */
}

static uint64_t airtime_us(uint8_t sf, uint32_t bw_hz, uint8_t cr, uint32_t len)
{
	/* Tsym in ns, to keep the LDRO comparison exact at narrow BW. */
	uint64_t tsym_ns = ((uint64_t)(1u << sf) * 1000000000ull) / bw_hz;

	/* 4.25 preamble symbols -> multiply by 425, divide by 100. */
	uint64_t t_pre_ns = ((uint64_t)preamble_syms(sf) * 100ull + 425ull) * tsym_ns / 100ull;

	int de = (tsym_ns > 16380000ull) ? 1 : 0;   /* > 16.38 ms */

	int64_t num = 8ll * (int64_t)len - 4ll * sf + 28 + 16;
	int64_t den = 4ll * ((int64_t)sf - 2ll * de);
	if (den < 1) den = 4;
	int64_t ceil_div = (num + den - 1) / den;
	int64_t n_pay = 8 + (ceil_div * (int64_t)cr > 0 ? ceil_div * (int64_t)cr : 0);

	return (t_pre_ns + (uint64_t)n_pay * tsym_ns) / 1000ull;
}

struct Preset {
	const char *name;
	uint8_t sf;
	uint32_t bw_hz;
	uint8_t cr;         /* 5..8, used as (cr-4+4) == cr in the formula */
};

static const Preset presets[] = {
	{ "SF7  BW62.5k CR4/5",  7,  62500, 5 },   /* EmpireMesh, since 2026-07-25 */
	{ "SF8  BW62.5k CR4/8",  8,  62500, 8 },   /* EmpireMesh, before that      */
	{ "SF10 BW125k  CR4/5", 10, 125000, 5 },
	{ "SF11 BW250k  CR4/5", 11, 250000, 5 },   /* MeshCore default             */
};

/* ------------------------------------------------------------------ main */

int main(void)
{
	k_msleep(2000);   /* USB CDC enumerates after boot on some boards */

	timing_init();
	timing_start();

	/* Utils.cpp assumes PSA is initialised. On a node that happens inside
	 * ZephyrRNG (adapters/rng/ZephyrRNG.cpp:285), which this tool
	 * deliberately does not link — so do it here, and fail loudly rather
	 * than silently benchmarking a pile of PSA_ERROR returns. */
	psa_status_t ps = psa_crypto_init();
	if (ps != PSA_SUCCESS) {
		printk("FATAL: psa_crypto_init failed: %d\n", (int)ps);
		return 0;
	}

	printk("\n");
	printk("========================================================\n");
	printk(" ZephCore crypto benchmark\n");
	printk(" board: %s   N=%d (slow ops N=%d)\n",
	       CONFIG_BOARD, CRYPTO_BENCH_ITERS, CRYPTO_BENCH_ITERS_SLOW);
	printk("========================================================\n");

	/* Test vectors. Fixed, not random: this is a timing measurement and a
	 * deterministic input makes runs comparable across boards and builds.
	 * Ed25519/X25519 timing is data-dependent at the margins, which the
	 * min-over-N already absorbs. */
	static uint8_t buf[1 + MAX_PACKET_PAYLOAD + CIPHER_BLOCK_SIZE];
	static uint8_t out[MAX_PACKET_PAYLOAD + CIPHER_BLOCK_SIZE * 2];
	static uint8_t key[PUB_KEY_SIZE];
	static uint8_t hash[MAX_HASH_SIZE];
	static uint8_t sig[SIGNATURE_SIZE];
	static uint8_t secret[CIPHER_KEY_SIZE * 2];

	for (size_t i = 0; i < sizeof(buf); i++) buf[i] = (uint8_t)(i * 7 + 3);
	for (size_t i = 0; i < sizeof(key); i++) key[i] = (uint8_t)(i * 11 + 5);

	uint8_t seed[SEED_SIZE];
	for (size_t i = 0; i < sizeof(seed); i++) seed[i] = (uint8_t)(i * 13 + 1);
	LocalIdentity self;
	self.fromSeed(seed);

	uint8_t seed2[SEED_SIZE];
	for (size_t i = 0; i < sizeof(seed2); i++) seed2[i] = (uint8_t)(i * 17 + 9);
	LocalIdentity peer;
	peer.fromSeed(seed2);

	self.sign(sig, buf, PAYLOAD_ADVERT);
	if (!self.verify(sig, buf, PAYLOAD_ADVERT)) {
		printk("FATAL: sign/verify self-check failed — results would be "
		       "meaningless\n");
		return 0;
	}

	BenchResult r;

	/* ---- instrument baseline ---------------------------------------- */
	print_header("Instrument baseline — subtract this from everything below");

	/* The cost of the two timing_counter_get() calls and the sink, with no
	 * crypto between them. Every figure in this run carries it. If it is
	 * not small compared to the fastest operation measured, the fastest
	 * rows are measuring the ruler, not the thing. */
	BENCH(r, "empty (timer pair + sink)", CRYPTO_BENCH_ITERS,
	      sink(buf, 16));
	print_result(&r);
	uint64_t overhead = r.min_ns;

	/* ---- primitives: SHA-256 ---------------------------------------- */
	print_header("SHA-256 (PSA one-shot) — the dedup hash");

	BENCH(r, "sha256, 16 B", CRYPTO_BENCH_ITERS,
	      Utils::sha256(hash, MAX_HASH_SIZE, buf, 16); sink(hash, MAX_HASH_SIZE));
	print_result(&r);
	uint64_t sha_16 = r.min_ns;

	BENCH(r, "sha256, 61 B (typical msg pkt)", CRYPTO_BENCH_ITERS,
	      Utils::sha256(hash, MAX_HASH_SIZE, buf, 1 + PAYLOAD_TYPICAL); sink(hash, MAX_HASH_SIZE));
	print_result(&r);
	uint64_t sha_typical = r.min_ns;

	BENCH(r, "sha256, 101 B (typical advert)", CRYPTO_BENCH_ITERS,
	      Utils::sha256(hash, MAX_HASH_SIZE, buf, 1 + PAYLOAD_ADVERT); sink(hash, MAX_HASH_SIZE));
	print_result(&r);

	BENCH(r, "sha256, 185 B (max payload)", CRYPTO_BENCH_ITERS,
	      Utils::sha256(hash, MAX_HASH_SIZE, buf, 1 + PAYLOAD_MAX); sink(hash, MAX_HASH_SIZE));
	print_result(&r);
	uint64_t sha_max = r.min_ns;

	BENCH(r, "sha256, 2-frag (32+32 B)", CRYPTO_BENCH_ITERS,
	      Utils::sha256(hash, MAX_HASH_SIZE, buf, 32, buf + 32, 32); sink(hash, MAX_HASH_SIZE));
	print_result(&r);

	/* ---- primitives: AES ------------------------------------------- */
	print_header("AES-128-ECB (PSA, key imported+destroyed per call)");

	BENCH(r, "encrypt, 16 B (1 block)", CRYPTO_BENCH_ITERS,
	      int n = Utils::encrypt(key, out, buf, 16); sink(out, n > 0 ? n : 1));
	print_result(&r);
	uint64_t aes_1blk = r.min_ns;

	BENCH(r, "encrypt, 60 B (4 blocks)", CRYPTO_BENCH_ITERS,
	      int n = Utils::encrypt(key, out, buf, PAYLOAD_TYPICAL); sink(out, n > 0 ? n : 1));
	print_result(&r);

	BENCH(r, "encrypt, 176 B (11 blocks)", CRYPTO_BENCH_ITERS,
	      int n = Utils::encrypt(key, out, buf, 176); sink(out, n > 0 ? n : 1));
	print_result(&r);
	uint64_t aes_max = r.min_ns;

	/* decrypt needs valid ciphertext-shaped input; ECB accepts anything
	 * block-aligned, and the timing does not depend on the plaintext. */
	BENCH(r, "decrypt, 176 B (11 blocks)", CRYPTO_BENCH_ITERS,
	      int n = Utils::decrypt(key, out, buf, 176); sink(out, n > 0 ? n : 1));
	print_result(&r);

	/* ---- the per-call key import ------------------------------------ */
	print_header("PSA key handling — the per-call import/destroy overhead");

	/* Isolates what src/Utils.cpp:86,101 and :149,159 spend on key
	 * lifecycle ALONE, with no cipher or MAC work attached. This is the
	 * number that decides whether caching a key handle per channel secret
	 * is worth changing Utils.cpp for: compare it against "AES encrypt,
	 * 16 B" above. If import+destroy dominates a one-block encrypt, the
	 * group-channel loop is paying for key management, not crypto. */
	BENCH(r, "psa_import_key + psa_destroy_key", CRYPTO_BENCH_ITERS, {
		psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
		psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
		psa_set_key_bits(&attr, CIPHER_KEY_SIZE * 8);
		psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
		psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
		psa_key_id_t kid;
		if (psa_import_key(&attr, key, CIPHER_KEY_SIZE, &kid) == PSA_SUCCESS) {
			g_sink += (uint32_t)kid;
			psa_destroy_key(kid);
		}
	});
	print_result(&r);
	uint64_t key_import = r.min_ns;

	/* ---- HMAC, by subtraction --------------------------------------- */
	print_header("HMAC-SHA256 (via encryptThenMAC / MACThenDecrypt)");

	/* compute_hmac_truncated() is static in Utils.cpp, so it cannot be
	 * called directly and is NOT worth un-staticing for a benchmark. It is
	 * derived instead: encryptThenMAC = encrypt + HMAC, and the failing
	 * MACThenDecrypt path is HMAC + a constant-time compare and nothing
	 * else (it returns before decrypt on MAC mismatch) — so the mismatch
	 * case below is very nearly a direct HMAC measurement. */
	BENCH(r, "encryptThenMAC, 60 B (TX path)", CRYPTO_BENCH_ITERS,
	      int n = Utils::encryptThenMAC(key, out, buf, PAYLOAD_TYPICAL); sink(out, n > 0 ? n : 1));
	print_result(&r);
	uint64_t etm_typical = r.min_ns;

	BENCH(r, "encryptThenMAC, 176 B (TX path)", CRYPTO_BENCH_ITERS,
	      int n = Utils::encryptThenMAC(key, out, buf, 176); sink(out, n > 0 ? n : 1));
	print_result(&r);

	/* A real ciphertext, so the success path actually reaches decrypt. */
	static uint8_t ct[MAX_PACKET_PAYLOAD + CIPHER_BLOCK_SIZE * 2];
	int ct_len = Utils::encryptThenMAC(key, ct, buf, PAYLOAD_TYPICAL);
	if (ct_len <= 0) {
		printk("FATAL: encryptThenMAC produced nothing\n");
		return 0;
	}

	BENCH(r, "MACThenDecrypt, MATCH (RX ours)", CRYPTO_BENCH_ITERS,
	      int n = Utils::MACThenDecrypt(key, out, ct, ct_len); sink(out, n > 0 ? n : 1));
	print_result(&r);
	uint64_t mtd_match = r.min_ns;

	/* Wrong key -> MAC mismatch -> returns before AES. This is the cost of
	 * ONE non-matching channel in the Mesh.cpp:398 loop. */
	static uint8_t wrong_key[PUB_KEY_SIZE];
	for (size_t i = 0; i < sizeof(wrong_key); i++) wrong_key[i] = (uint8_t)(i * 3 + 200);

	BENCH(r, "MACThenDecrypt, MISS (wrong chan)", CRYPTO_BENCH_ITERS,
	      int n = Utils::MACThenDecrypt(wrong_key, out, ct, ct_len); g_sink += (uint32_t)n);
	print_result(&r);
	uint64_t mtd_miss = r.min_ns;

	/* ---- asymmetric -------------------------------------------------- */
	print_header("Ed25519 / X25519 (Monocypher, software only)");

	BENCH(r, "Ed25519 verify (advert RX)", CRYPTO_BENCH_ITERS_SLOW,
	      bool ok = self.verify(sig, buf, PAYLOAD_ADVERT); g_sink += ok ? 1 : 0);
	print_result(&r);
	uint64_t ed_verify = r.min_ns;

	BENCH(r, "Ed25519 sign (advert TX)", CRYPTO_BENCH_ITERS_SLOW,
	      self.sign(sig, buf, PAYLOAD_ADVERT); sink(sig, SIGNATURE_SIZE));
	print_result(&r);

	BENCH(r, "X25519 shared secret (contact)", CRYPTO_BENCH_ITERS_SLOW,
	      self.calcSharedSecret(secret, peer); sink(secret, sizeof(secret)));
	print_result(&r);

	/* ---- composites: real call paths -------------------------------- */
	print_header("Per-packet totals — the real RX/TX call paths");

	/* Dedup as implemented today. SimpleMeshTables::wasSeen() and
	 * markSeen() each call packet->calculatePacketHash() independently,
	 * and since the 2026-07-02 hasSeen split every caller invokes markSeen
	 * right after wasSeen returns false — so a NEW packet hashes twice.
	 * The linear memcmp scan those functions also do is not crypto and is
	 * not timed here; it is a 160-entry 8-byte compare, nowhere near the
	 * hash. Packet.cpp is compiled from the main tree, so the buffer
	 * assembly (the memcpy into a stack buffer) is included, as it is on a
	 * node. */
	/* Not `static`: a function-local static with a non-trivial constructor
	 * emits __cxa_guard_acquire/release, which Zephyr's minimal C++ runtime
	 * does not provide. ~260 bytes on the stack instead. */
	Packet pkt;
	pkt.header = (PAYLOAD_TYPE_TXT_MSG << PH_TYPE_SHIFT) | ROUTE_TYPE_FLOOD;
	pkt.payload_len = PAYLOAD_TYPICAL;
	memcpy(pkt.payload, buf, PAYLOAD_TYPICAL);

	BENCH(r, "dedup, 1x calculatePacketHash", CRYPTO_BENCH_ITERS,
	      pkt.calculatePacketHash(hash); sink(hash, MAX_HASH_SIZE));
	print_result(&r);
	uint64_t dedup_1 = r.min_ns;

	BENCH(r, "dedup, 2x (wasSeen + markSeen)", CRYPTO_BENCH_ITERS,
	      pkt.calculatePacketHash(hash); sink(hash, MAX_HASH_SIZE);
	      pkt.calculatePacketHash(hash); sink(hash, MAX_HASH_SIZE));
	print_result(&r);
	uint64_t dedup_2 = r.min_ns;

	/* Group packet that matches no configured channel: the loop runs every
	 * channel and every one costs a full HMAC. */
	BENCH(r, "group RX, " STRINGIFY(CRYPTO_BENCH_CHANNELS) " channel misses",
	      CRYPTO_BENCH_ITERS / 4,
	      for (int c = 0; c < CRYPTO_BENCH_CHANNELS; c++) {
		      int n = Utils::MACThenDecrypt(wrong_key, out, ct, ct_len);
		      g_sink += (uint32_t)n;
	      });
	print_result(&r);
	uint64_t group_miss_all = r.min_ns;

	/* ---- the verdict table ------------------------------------------ */
	printk("\n");
	printk("========================================================\n");
	printk(" PER-PACKET CRYPTO BUDGET vs AIRTIME\n");
	printk("--------------------------------------------------------\n");

	struct Scenario {
		const char *name;
		uint64_t ns;
		uint32_t pkt_bytes;
	};

	/* Packet byte counts are the on-air length the airtime formula wants:
	 * 2 header bytes + path + payload. Path is assumed 3 hops x 1-byte
	 * hash, a common flood. */
	const uint32_t hdr_path = 2 + 3;

	const Scenario scen[] = {
		{ "flood not for us (dedup only)",
		  dedup_2, hdr_path + PAYLOAD_TYPICAL },
		{ "direct msg for us (dedup+MAC+AES)",
		  dedup_2 + mtd_match, hdr_path + PAYLOAD_TYPICAL },
		{ "group msg, no channel matches",
		  dedup_2 + group_miss_all, hdr_path + PAYLOAD_TYPICAL },
		{ "advert (dedup + Ed25519 verify)",
		  dedup_2 + ed_verify, hdr_path + PAYLOAD_ADVERT },
		{ "our TX (encryptThenMAC + dedup)",
		  dedup_1 + etm_typical, hdr_path + PAYLOAD_TYPICAL },
	};

	for (size_t p = 0; p < ARRAY_SIZE(presets); p++) {
		printk("\n %s\n", presets[p].name);
		printk("  %-36s %9s %9s %8s\n",
		       "scenario", "crypto us", "air us", "share");
		printk("  ------------------------------------ --------- --------- --------\n");
		for (size_t s = 0; s < ARRAY_SIZE(scen); s++) {
			uint64_t air = airtime_us(presets[p].sf, presets[p].bw_hz,
						  presets[p].cr, scen[s].pkt_bytes);
			uint64_t cry_us = scen[s].ns / 1000u;
			/* share in hundredths of a percent (basis points) */
			uint64_t bp = air ? (scen[s].ns * 10000ull) / (air * 1000ull) : 0;
			printk("  %-36s %9llu %9llu  %3llu.%02llu%%\n",
			       scen[s].name, cry_us, air, bp / 100u, bp % 100u);
		}
	}

	/* ---- what the software wins would buy --------------------------- */
	printk("\n");
	printk("--------------------------------------------------------\n");
	printk(" HEADROOM OF THE FREE SOFTWARE WINS (no hardware needed)\n");
	printk("--------------------------------------------------------\n");
	printk(" single-hash dedup saves           : %llu us/packet\n",
	       (dedup_2 - dedup_1) / 1000u);
	printk(" cached key handle would save up to: %llu us/packet\n",
	       ((uint64_t)CRYPTO_BENCH_CHANNELS * key_import) / 1000u);
	printk("   (%d channels x %llu.%03llu us import/destroy)\n",
	       CRYPTO_BENCH_CHANNELS, key_import / 1000u, key_import % 1000u);
	printk(" key import as share of a 1-block AES: %llu%%\n",
	       aes_1blk ? (key_import * 100u) / aes_1blk : 0);

	printk("\n");
	printk("--------------------------------------------------------\n");
	printk(" HOW TO READ THIS\n");
	printk("--------------------------------------------------------\n");
	printk(" The 'share' column is the whole question. Crypto competes\n");
	printk(" with airtime, not with itself. If every scenario sits well\n");
	printk(" under 1%%, hardware acceleration cannot buy throughput the\n");
	printk(" radio would ever notice, and the honest answer to\n");
	printk(" HANDOVER_crypto_hw_accel.md is Option C (do nothing) --\n");
	printk(" regardless of how large the microsecond figures look.\n");
	printk(" Only a scenario in the double-digit percent range, or one\n");
	printk(" whose absolute cost approaches the inter-packet gap,\n");
	printk(" justifies going further.\n");
	printk(" Power is NOT measured here. A CPU-bound millisecond also\n");
	printk(" costs battery; if the share numbers are borderline, measure\n");
	printk(" current draw before deciding.\n");

	printk("\n");
	printk("========================================================\n");
	printk(" reference: sha16=%llu.%03llu us  sha185=%llu.%03llu us\n",
	       sha_16 / 1000u, sha_16 % 1000u, sha_max / 1000u, sha_max % 1000u);
	printk(" reference: aes1blk=%llu.%03llu us  aes176=%llu.%03llu us\n",
	       aes_1blk / 1000u, aes_1blk % 1000u, aes_max / 1000u, aes_max % 1000u);
	printk(" reference: hmac_miss=%llu.%03llu us  ed_verify=%llu.%03llu us\n",
	       mtd_miss / 1000u, mtd_miss % 1000u, ed_verify / 1000u, ed_verify % 1000u);
	printk(" instrument overhead=%llu ns (already included in every row)\n",
	       overhead);
	printk(" (sha_typical=%llu ns, sink=%u — ignore, anti-elision)\n",
	       sha_typical, (unsigned)g_sink);
	printk("========================================================\n");
	printk("\n[crypto_bench] done — halting. Reflash normal firmware.\n");

	timing_stop();
	return 0;
}
