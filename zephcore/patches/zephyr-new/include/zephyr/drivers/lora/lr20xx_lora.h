/*
 * SPDX-License-Identifier: MIT
 * LR20xx Zephyr LoRa driver — extension API used by the ZephCore adapter
 * (radio_ops_lr2021.cpp). Everything here takes the driver's SPI mutex; calls
 * that touch the chip bracket a running duty cycle (see lr20xx_lora.c).
 */

#ifndef LR20XX_LORA_H
#define LR20XX_LORA_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Instantaneous RSSI in dBm, or -128 if the read was refused. */
int16_t lr20xx_get_rssi_inst(const struct device *dev);

/* n RSSI samples spacing_us apart, one duty-cycle stand-down for the burst.
 * Returns the samples written (< n: refused partway, discard), or -EAGAIN when
 * a preamble/header landed in the window (a busy channel, not a sampler fault). */
int lr20xx_get_rssi_burst(const struct device *dev, int16_t *out, int n,
			  uint32_t spacing_us);

/* Carrier frequency error over received packets. One packet's offset is this
 * node's reference error plus the sender's; only the mean over many peers
 * says anything about ours. */
struct lr20xx_freq_offset_stats {
	int32_t last_hz;   /* most recent packet */
	int32_t mean_hz;   /* mean over count packets */
	int32_t min_hz;
	int32_t max_hz;
	uint32_t count;    /* 0 = nothing measured */
};

/* Fills *out; returns the packet count. Decoded from status bytes the
 * datasheet does not document (SDK v2.0.2), so values past +/-200 kHz are
 * dropped as implausible. */
uint32_t lr20xx_get_freq_offset(const struct device *dev,
				struct lr20xx_freq_offset_stats *out);
void lr20xx_reset_freq_offset(const struct device *dev);

/* Duty-cycle false-preamble re-arms since boot or reset (`get dc.restarts`). */
uint32_t lr20xx_get_dc_timeout_restarts(const struct device *dev);
void lr20xx_reset_dc_timeout_restarts(const struct device *dev);

/* Deaf time per duty-cycle wake in us: warm start 1 ms + STDBY_RC->Rx 115 us
 * (DS Table 3-23), plus the TCXO restart where one is fitted. */
uint32_t lr20xx_get_wakeup_time_us(const struct device *dev);

/* RX-busy gate: the DIO1-stamped header latch (bounded by the payload deadline)
 * and, without a duty cycle, the preamble grace. Never waits for the mutex. */
bool lr20xx_is_receiving(const struct device *dev);

/* RX boosted gain. */
void lr20xx_set_rx_boost(const struct device *dev, bool enable);

/* Up to 3 extra SFs demodulated alongside the configured one, same bandwidth
 * (num = 0 disables). Datasheet constraints, enforced: every side SF above the
 * main SF, all distinct, highest - lowest <= 4, and at BW >= 500 kHz at most 2
 * (1 when the main SF >= 10). Returns 0, or -EINVAL on a violation. */
int lr20xx_configure_side_detectors(const struct device *dev,
				    const uint8_t *sfs, uint8_t num);

/* Redo the frequency-dependent calibrations after temperature drift. Leaves the
 * driver out of RX: the caller must restart it. */
void lr20xx_recalibrate(const struct device *dev);

/* Adaptive CAD. The offset is added to the base detPeak (DS Table 6-19) on
 * every LBT CAD from the next one on, clamped to [lr20xx_cad_peak_min(),
 * lr20xx_cad_peak_max()]. */
void lr20xx_cad_set_peak_offset(const struct device *dev, int8_t offset);
uint8_t lr20xx_cad_base_peak(const struct device *dev);
uint8_t lr20xx_cad_peak_min(void);
uint8_t lr20xx_cad_peak_max(void);

/* One blocking calibration CAD at base + peak_offset. Returns 0 = free (chip in
 * standby, the caller restarts Rx), 2 = detected (chip in Rx on the signal; the
 * caller must not restart it and reads lr20xx_cad_rx_outcome() later), <0 on
 * error. Mesh thread only. */
int lr20xx_cad_probe(const struct device *dev, int8_t peak_offset);
/* The Rx bound a detecting probe programs, in ms. */
uint32_t lr20xx_cad_rx_timeout_ms(const struct device *dev);
/* 1 = a packet arrived, 2 = the Rx timed out with nothing decoded, 0 = nothing
 * armed or not resolved yet. Consumes the result. */
int lr20xx_cad_rx_outcome(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* LR20XX_LORA_H */
