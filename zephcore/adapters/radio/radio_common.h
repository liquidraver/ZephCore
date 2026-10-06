/*
 * SPDX-License-Identifier: MIT
 * Shared constants and utilities for all LoRa radio adapters.
 *
 * Radio-family-independent pieces used by LoRaRadio.
 * Radio-specific constants (e.g. SX126x duty cycle math) stay in
 * their respective headers.
 */

#pragma once

#include <zephyr/drivers/lora.h>

#include "radio_tuning.h"

/* --- RSSI read timing (SX1261/2 DS rev 2.2 Table 13-82) ---
 * Reads faster than the chip's averaging window return the same sample.
 * window_us = 936 / BW_kHz reproduces the table; it is specified for GFSK and
 * used here for LoRa as the best available figure. */
#define RSSI_WINDOW_BW_PRODUCT   936U  /* window_us * BW_kHz, DS Table 13-82 */
#define RSSI_SETTLE_WINDOWS      16U   /* delay/window is 12..15; round up */

/* Burst-stat rescale point: the counters behind `get cad`'s sp field are
 * halved together at this count, which keeps the reply inside the remote CLI
 * limit and makes the totals describe recent conditions. */
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
/* A due sample that lands mid-packet, during TX or in the duty-cycle sleep
 * window is retried after this long (the old housekeeping grid). */
#define NOISE_FLOOR_RETRY_MS             5000
/* Blocked attempts allowed before standing down to the next full interval. */
#define NOISE_FLOOR_MAX_RETRIES          2

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
	/* The wide set — 2.4 GHz territory, LR2021 only today. These return the
	 * chip's TRUE bandwidths, not the enum's round names: airtime and the
	 * noise-floor clamp both hang off this number, so 203 must not be
	 * reported as 200. */
	case BW_200_KHZ:  return 203000;
	case BW_400_KHZ:  return 406000;
	case BW_800_KHZ:  return 812000;
	case BW_1000_KHZ: return 1000000;
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
	/* Wide bandwidths, under both spellings (the round name and the chip's true
	 * value). LR2021 only; the CLI rejects them elsewhere. */
	case 200: case 203:  return BW_200_KHZ;
	case 400: case 406:  return BW_400_KHZ;
	case 800: case 812:  return BW_800_KHZ;
	case 1000:           return BW_1000_KHZ;
	default:  return BW_125_KHZ;
	}
}

/* Lowest physically possible noise floor for a bandwidth, in dBm: thermal
 * noise kTB at 290 K, -174 dBm/Hz + 10*log10(BW_Hz), with no noise figure and
 * no SF term. A reading below it is a bad RSSI read. */
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
	case 200: case 203:  return -121;  /* 10*log10(203000)  = 53.1 */
	case 400: case 406:  return -118;  /* 10*log10(406000)  = 56.1 */
	case 800: case 812:  return -115;  /* 10*log10(812000)  = 59.1 */
	case 1000:           return -114;  /* 10*log10(1000000) = 60.0 */
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
