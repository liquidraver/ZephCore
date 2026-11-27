/*
 * SPDX-License-Identifier: MIT
 * ZephCore LoRa timing helpers shared by the sx126x, lr11xx and lr20xx drivers.
 *
 * Pure functions of the modem parameters, so every driver bounds its RX-busy
 * gate with the same figures, and the host unit tests can check them.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LORA_ZC_LORA_TIMING_H_
#define ZEPHYR_INCLUDE_DRIVERS_LORA_ZC_LORA_TIMING_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* True LoRa bandwidth in Hz for a bandwidth named in whole kHz, which is the
 * value of Zephyr's enum lora_signal_bandwidth (7, 10, 15 ... 1000). The
 * narrow ones are 500 kHz / 2^n and 125 kHz / (3 * 2^n), to the nearest Hz;
 * the datasheets print them rounded to 10 Hz. 0 = not a LoRa bandwidth. */
static inline uint32_t zc_lora_bw_hz(uint16_t bw_khz)
{
	switch (bw_khz) {
	case 7:    return 7812;
	case 10:   return 10417;
	case 15:   return 15625;
	case 20:   return 20833;
	case 31:   return 31250;
	case 41:   return 41667;
	case 62:   return 62500;
	case 125:  return 125000;
	case 250:  return 250000;
	case 500:  return 500000;
	case 200:  return 203000;
	case 400:  return 406000;
	case 800:  return 812000;
	case 1000: return 1000000;
	default:   return 0;
	}
}

/* Low data rate optimisation: on when a symbol lasts longer than 16.38 ms
 * (SX1261/2 DS 6.1.1.4). The header does not carry it, so every radio family
 * takes the decision here, from the bandwidths above. Unconfigured (bw_hz == 0
 * or an SF above 12) yields off. */
static inline bool zc_lora_ldro(uint8_t sf, uint32_t bw_hz)
{
	if (bw_hz == 0 || sf > 12) {
		return false;
	}
	return (((uint64_t)1U << sf) * 1000000ULL) / bw_hz > 16380U;
}

/* Preamble grace for the RX-busy gate: (preamble + 8) symbols, rounded up to a
 * whole ms. Enough for a real packet's header to land after PREAMBLE_DETECTED;
 * a foreign sync word releases the gate once it expires. bw_hz == 0 or an SF
 * outside 5..12 means "not configured yet" and yields 1 s. */
static inline uint32_t zc_lora_preamble_grace_ms(uint8_t sf, uint32_t bw_hz,
						  uint16_t preamble)
{
	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 1000;
	}
	uint64_t us = ((uint64_t)(preamble + 8U) << sf) * 1000000ULL / bw_hz;

	return (uint32_t)((us + 999U) / 1000U);
}

/* Upper bound on a packet's payload phase: airtime of a 255-byte explicit-header
 * packet at CR 4/8 with LDRO on (the larger symbol count), +25% +100 ms. Bounds
 * the header latch, which continuous RX has no timer to clear. Deliberately
 * generous: releasing early would let TX start over a packet still arriving.
 * Unconfigured (see above) yields 30 s. */
static inline uint32_t zc_lora_max_payload_ms(uint8_t sf, uint32_t bw_hz)
{
	if (bw_hz == 0 || sf < 5 || sf > 12) {
		return 30000;
	}

	/* n = 8 + ceil((8*PL - 4*SF + 28 + 16) / (4*(SF - 2*DE))) * 8,
	 * PL = 255, DE = 1: the divisor is 4*(SF-2), never zero at SF >= 5. */
	uint32_t numer = 8U * 255U + 28U + 16U;
	uint32_t denom = 4U * (uint32_t)(sf - 2U);

	if (numer > 4U * (uint32_t)sf) {
		numer -= 4U * (uint32_t)sf;
	}
	uint32_t n_sym = 8U + ((numer + denom - 1U) / denom) * 8U;

	uint64_t us = ((uint64_t)n_sym << sf) * 1000000ULL / bw_hz;
	uint32_t ms = (uint32_t)((us + 999U) / 1000U);

	return ms + (ms / 4U) + 100U;
}

/* ms -> a 24-bit chip timer count at step_hz, computed in 64 bits and
 * saturated at 0xFFFFFF. Masking instead of saturating wraps a long timeout
 * into a short one (SF12/BW7.8 exceeds 24 bits at 15.625 us steps). */
static inline uint32_t zc_lora_ms_to_steps24(uint32_t ms, uint32_t step_hz)
{
	uint64_t steps = ((uint64_t)ms * step_hz) / 1000U;

	return steps > 0x00FFFFFFU ? 0x00FFFFFFU : (uint32_t)steps;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_LORA_ZC_LORA_TIMING_H_ */
