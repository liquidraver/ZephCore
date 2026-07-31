/*
 * SPDX-License-Identifier: MIT
 * Shared constants and utilities for all LoRa radio adapters.
 *
 * Anything duplicated between SX126xRadio and LR1110Radio belongs here.
 * Radio-specific constants (e.g. SX126x duty cycle math) stay in
 * their respective headers.
 */

#pragma once

#include <zephyr/drivers/lora.h>

/* --- Noise floor calibration (EMA) ---
 * Median-of-N RSSI reads → EMA.  alpha = 1/8, converges in ~8 ticks.
 * Samples above floor + SAMPLING_THRESHOLD rejected as interference. */
#define NOISE_FLOOR_EMA_SHIFT            3   /* alpha = 1/(1<<3) = 1/8 */
#define NOISE_FLOOR_SAMPLES_PER_TICK     8   /* median of 8 reads per tick */
#define NOISE_FLOOR_UNGUARDED_INTERVAL   16  /* ticks between unfiltered samples (power of 2) */
#define NOISE_FLOOR_SAMPLING_THRESHOLD   14  /* dB above floor to reject as interference */
#define DEFAULT_NOISE_FLOOR              0   /* sentinel: seed from first sample */

/* --- RSSI read timing (SX1261/2 DS rev 2.2 Table 13-82) ---
 *
 * The median-of-N above only rejects outliers if the N reads are actually
 * independent.  The chip updates RSSI once per "averaging window"; reads
 * issued faster than that return the same underlying sample repeatedly, and
 * the median degenerates into one read with extra SPI traffic.
 *
 * The published table is indexed by GFSK channel-filter bandwidth.  Across
 * all 19 rows the product window_us * BW_kHz lands in 921..938, so
 * 936 / BW_kHz reproduces every published value to within a microsecond.
 * The separate "RSSI delay" column (BUSY falling edge -> first valid sample)
 * is 12x to 15x the window across the same rows.
 *
 * CAVEAT: Semtech documents this for GFSK only and publishes no LoRa
 * equivalent.  The RSSI path is the same analog/AGC chain, so these are used
 * as the best available proxy — not as specified LoRa figures.  `get cad`
 * reports the measured burst spread so the assumption stays falsifiable on
 * real hardware.
 */
#define RSSI_WINDOW_BW_PRODUCT   936U  /* window_us * BW_kHz, DS Table 13-82 */
#define RSSI_SETTLE_WINDOWS      16U   /* delay/window is 12..15; round up */

/* Burst-stat rescale point.  The counters behind `get cad`'s sp field are
 * halved (all three together, so the mean and the zero-spread share are
 * preserved exactly) once the burst count reaches this.
 *
 * Two reasons, and the display one is the hard constraint: the remote CLI
 * reply is capped at CLI_REMOTE_REPLY_SIZE (161 B) and has to fit the header
 * plus three level lines, so no field may grow without bound.  At one burst
 * per 15 s a free-running counter passes 500 000 in three months and would
 * eat the level rows.  8192 keeps it to four digits forever.
 *
 * The second reason is that halving turns the totals into an exponential
 * forgetting window, so the numbers describe recent conditions instead of
 * being anchored to whatever the channel was doing at boot.  Same idea as
 * CAD_STATS_DECAY_MS below, which halves the CAD counters on a timer. */
#define RSSI_BURST_STATS_CAP     8192U

static inline uint32_t rssi_avg_window_us(uint16_t bw_khz)
{
	uint32_t bw = bw_khz ? bw_khz : 1U;

	/* ceil.  Non-zero for every LoRa bandwidth (BW 500 -> 2 us), so no
	 * zero-spacing guard is needed. */
	return (RSSI_WINDOW_BW_PRODUCT + bw - 1U) / bw;
}

static inline uint32_t rssi_settle_delay_us(uint16_t bw_khz)
{
	return rssi_avg_window_us(bw_khz) * RSSI_SETTLE_WINDOWS;
}

/* Sampling cadence.  The sampler used to run once per housekeeping tick and so
 * inherited that 5 s period; owning an explicit interval is what let the
 * periodic tick go away (see mesh/Maintenance.h).  Both the 8-sample warmup and
 * the every-16th-sample unguarded bypass are counted in samples, so they scale
 * with this value — see the Kconfig help before changing it. */
#define NOISE_FLOOR_INTERVAL_MS  CONFIG_ZEPHCORE_NOISE_FLOOR_INTERVAL_MS
/* A due sample that lands while the radio is mid-packet, transmitting, or in
 * its duty-cycle sleep window is retried on this deadline rather than waiting a
 * full interval.
 *
 * 5000 is not a tuned guess: before the deadline conversion a blocked sample
 * simply waited for the next 5 s housekeeping tick, so this reproduces the old
 * retry grid exactly.  It matters under RX duty cycle, where the chip is in its
 * sleep window a large fraction of the time and blocked attempts are the norm
 * rather than the exception — a shorter retry there can push the wake rate
 * ABOVE the fixed tick this conversion replaced, inverting the whole point. */
#define NOISE_FLOOR_RETRY_MS             5000
/* Blocked attempts allowed before standing down to the next full interval. */
#define NOISE_FLOOR_MAX_RETRIES          2

/* --- Adaptive CAD (LBT detPeak calibration) ---
 * Housekeeping-tick CAD probes accumulate per-level busy/free statistics;
 * a one-sided staircase converges on the lowest detPeak offset whose
 * false-positive rate stays under target.  Levels are signed offsets from
 * the chip family's per-SF base detPeak (SX126x: SF+13; LR11xx/LR20xx:
 * 56-68 table) so the C++ layer stays scale-independent. */
/* Operating-offset range (levels from the per-SF family base detPeak).  Wide
 * on purpose — a dense hilltop may need a much higher detPeak than a quiet
 * valley node; the per-family absolute clamp in the driver (SX126x 15-40,
 * LR 48-90) is a firmware guardrail against "CAD never/always fires", NOT a
 * chip limit (cadDetPeak is a full uint8_t).
 * MUST match CAD_OFFSET_MIN/MAX in helpers/NodePrefs.h. */
#define CAD_LEVEL_MIN            (-8)  /* most sensitive probe level */
#define CAD_LEVEL_MAX            12    /* least sensitive probe level */
#define CAD_NUM_LEVELS           (CAD_LEVEL_MAX - CAD_LEVEL_MIN + 1)
#define CAD_SWEEP_MIN            (-4)  /* dry-run sweep window (get cad with auto off) */
#define CAD_SWEEP_MAX            4
/* Knee-seeking staircase (replaces the earlier absolute-FP-target band).  The
 * FP-vs-detPeak curve falls as detPeak rises (less sensitive → fewer false
 * detects) and flattens past a knee; the sweet spot is the knee — the most
 * sensitive detPeak whose FP has already bottomed out.  The controller reads
 * the local curve SLOPE from three rungs (frontier op-1, operating op, op+1)
 * rather than an absolute FP level, so it converges the same way regardless of
 * a site's FP floor (which varies with traffic and classifier residual).
 *  - KNEE_SLOPE: the per-level FP change (permille) that counts as "steep".
 *    Below the knee the curve drops fast (step up toward the knee); at/above it
 *    the curve is flat (slope < KNEE_SLOPE).
 *  - PLATEAU_CLEAN: on a flat plateau, only reclaim sensitivity (step down) if
 *    FP is already this low — the guard that stops a flat-but-noisy curve from
 *    walking to the sensitive rail (there, holding is the least-bad move; a
 *    genuinely quiet flat-low site descends to the floor, which is correct). */
#define CAD_KNEE_SLOPE_PERMILLE     50    /* >=5%/level FP change = steep */
#define CAD_PLATEAU_CLEAN_PERMILLE  50    /* <=5% FP = clean enough to descend */
#define CAD_STEP_MIN_PROBES         120   /* per-level samples before a step call */
/* Airtime-protection cap on the TOTAL busy (defer) rate — false positives AND
 * real traffic.  The knee controller only minimises *false* busy, but on a
 * congested hilltop most busy verdicts are real distant traffic we'd never
 * actually collide with (capture effect), and deferring for all of it starves
 * the node's own airtime.  When the operating level's busy rate exceeds the
 * cap, step UP (less sensitive) regardless of FP — self-targeting, since a
 * quiet node's busy rate never reaches it.  HYST keeps a descend from bouncing
 * straight back into the cap.  The cap itself is a per-node pref
 * (`cad_busycap`, percent, `set cad.busycap`; default 25, 0 = off) since it is
 * a policy call (airtime vs. collision/capture), not a physical constant. */
#define CAD_BUSY_DEFER_HYST_PERMILLE 100  /* descend only if frontier busy <= cap-10% */
#define CAD_PROBE_RSSI_GUARD     7     /* dB above floor = channel visibly busy, skip probe */
#define CAD_STATS_DECAY_MS       (6UL * 3600UL * 1000UL)  /* halve counters every 6 h */
/* NOTE: the probe has no retry deadline and no wake of its own.  It runs off
 * the noise-floor sampler's measurement (LoRaRadioBase::cadMaintenance), which
 * already applies the idle-RX guards and yields a median-of-8.  Consequently
 * the effective probe rate is quantised to NOISE_FLOOR_INTERVAL_MS: setting
 * probe_interval below that just gets one probe per floor sample. */

/* --- RX ring buffer --- */
#define RX_RING_SIZE 8  /* ~2 KB; buffers burst arrivals at SF7/BW500 */

/* --- TX wait thread --- */
#define TX_WAIT_THREAD_STACK_SIZE 2048
#define TX_WAIT_THREAD_PRIORITY   10     /* preemptible, below main thread */
#define TX_TIMEOUT_MS             5000   /* hard timeout for TX completion signal */

/* --- SNR thresholds per spreading factor (SF7..SF12) --- */
inline constexpr float lora_snr_threshold[] = {
	-7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f
};

/* --- Callback types --- */
typedef void (*RadioRxCallback)(void *user_data);
typedef void (*RadioTxDoneCallback)(void *user_data);

/* --- Zephyr enum mapping --- */


static inline uint32_t bandwidth_to_hz(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_7_KHZ:   return 7812;
	case BW_10_KHZ:  return 10417;
	case BW_15_KHZ:  return 15625;
	case BW_20_KHZ:  return 20833;
	case BW_31_KHZ:  return 31250;
	case BW_41_KHZ:  return 41667;
	case BW_62_KHZ:  return 62500;
	case BW_125_KHZ: return 125000;
	case BW_250_KHZ: return 250000;
	case BW_500_KHZ: return 500000;
	default:         return 125000;
	}
}

/* Input is truncated kHz (e.g. 7.8→7, 10.4→10, 62.5→62) */
static inline enum lora_signal_bandwidth bw_khz_to_enum(uint16_t bw_khz)
{
	switch (bw_khz) {
	case 7:   return BW_7_KHZ;
	case 10:  return BW_10_KHZ;
	case 15:  return BW_15_KHZ;
	case 20:  return BW_20_KHZ;
	case 31:  return BW_31_KHZ;
	case 41:  return BW_41_KHZ;
	case 62:  return BW_62_KHZ;
	case 125: return BW_125_KHZ;
	case 250: return BW_250_KHZ;
	case 500: return BW_500_KHZ;
	default:  return BW_125_KHZ;
	}
}

/* Lowest physically-possible noise floor for a given bandwidth, in dBm.
 *
 * This is raw thermal noise — kTB at 290 K, with NO noise-figure term:
 *     -174 dBm/Hz + 10*log10(BW_Hz)
 * A passive receiver cannot read below it, so it is the one place a sanity
 * clamp belongs: anything under this line is a bad RSSI read, not a quiet
 * site.  Adding a receiver noise figure here would clamp ABOVE what the
 * hardware can legitimately report and manufacture a floor — which is exactly
 * what the old fixed -120 rail did to BW 62.5 kHz, pinning it on every EMA
 * update because -120 happens to be that preset's kTB+NF.
 *
 * Bandwidth is the only term that moves.  SF changes the SNR the demodulator
 * can decode at, not the noise power in the channel, so it must NOT appear.
 *
 * For reference, a typical SX126x (NF ~6 dB) reads about 6 dB above these:
 * BW 62.5 kHz measures ~-120 dBm on a quiet site against a -126 kTB limit. */
static inline int16_t noise_floor_min_dbm(uint16_t bw_khz)
{
	switch (bw_khz) {
	case 7:   return -135;   /* 10*log10(7800)   = 38.9 */
	case 10:  return -134;   /* 10*log10(10400)  = 40.2 */
	case 15:  return -132;   /* 10*log10(15600)  = 41.9 */
	case 20:  return -131;   /* 10*log10(20800)  = 43.2 */
	case 31:  return -129;   /* 10*log10(31250)  = 45.0 */
	case 41:  return -128;   /* 10*log10(41700)  = 46.2 */
	case 62:  return -126;   /* 10*log10(62500)  = 48.0 */
	case 125: return -123;   /* 10*log10(125000) = 51.0 */
	case 250: return -120;   /* 10*log10(250000) = 54.0 */
	case 500: return -117;   /* 10*log10(500000) = 57.0 */
	default:  return -123;   /* matches bw_khz_to_enum's 125 kHz fallback */
	}
}

/* CR 5-8 → Zephyr coding_rate enum */
static inline enum lora_coding_rate cr_to_enum(uint8_t cr)
{
	switch (cr) {
	case 5: return CR_4_5;
	case 6: return CR_4_6;
	case 7: return CR_4_7;
	case 8: return CR_4_8;
	default: return CR_4_5;
	}
}
