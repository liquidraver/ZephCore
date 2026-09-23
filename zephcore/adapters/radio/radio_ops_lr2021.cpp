/*
 * SPDX-License-Identifier: MIT
 * LR20xx family (LR2021) for LoRaRadio.
 */

#include "LoRaRadioOps.h"
#include <stdio.h>

extern "C" {
#include <zephyr/drivers/lora/lr20xx_lora.h>
}

namespace mesh {

/* Carrier frequency error over received packets. Spread and count matter as
 * much as the mean: only a mean over many different peers approximates this
 * node's own reference error. */
static int lr2021_format_freq_error(const struct device *dev, char *buf, int cap)
{
	struct lr20xx_freq_offset_stats st;

	if (lr20xx_get_freq_offset(dev, &st) == 0) {
		return snprintf(buf, cap, "no packets measured yet");
	}
	return snprintf(buf, cap, "mean %d Hz, min %d, max %d, %u pkts",
			st.mean_hz, st.min_hz, st.max_hz, (unsigned)st.count);
}

/* No AGC reset (no such fault on this part). Drift recalibration is specified.
 * Adaptive CAD probing runs CAD_ONLY plus an explicit SetRx in the driver,
 * so the 524 ms cad_timeout never bounds it; not yet hardware-verified on an
 * LR2021 (LLD 05). */
const LoRaRadioOps kLoRaRadioOps = {
	.loramac_node = false,
	.rssi_inst = lr20xx_get_rssi_inst,
	.rssi_burst = lr20xx_get_rssi_burst,
	.is_receiving = lr20xx_is_receiving,
	.set_rx_boost = lr20xx_set_rx_boost,
	.set_fem_rx = nullptr,
	.is_chip_busy = nullptr,
	.wakeup_time_us = lr20xx_get_wakeup_time_us,
	.dc_restarts = lr20xx_get_dc_timeout_restarts,
	.reset_dc_restarts = lr20xx_reset_dc_timeout_restarts,
	.reset_agc = nullptr,
	.recalibrate = lr20xx_recalibrate,
	.cad_probe = lr20xx_cad_probe,
	.cad_rx_outcome = lr20xx_cad_rx_outcome,
	.cad_rx_timeout_ms = lr20xx_cad_rx_timeout_ms,
	.cad_set_peak_offset = lr20xx_cad_set_peak_offset,
	.cad_base_peak = lr20xx_cad_base_peak,
	.cad_peak_min = lr20xx_cad_peak_min,
	.cad_peak_max = lr20xx_cad_peak_max,
	.post_begin = nullptr,
	.side_detectors = lr20xx_configure_side_detectors,
	.reset_stats = lr20xx_reset_freq_offset,
	.format_freq_error = lr2021_format_freq_error,
};

} /* namespace mesh */
