/*
 * SPDX-License-Identifier: MIT
 * LR11xx family (LR1110) for LoRaRadio.
 */

#include "LoRaRadioOps.h"

extern "C" {
#include <zephyr/drivers/lora/lr11xx_lora.h>
}

namespace mesh {

/* No AGC reset: this part has no jammed-AGC fault, and running the SX126x
 * remedy anyway cost packets (LLD 03 §13). Drift recalibration is specified
 * (UM: image cal after > 10 C), with the temperature from the board. */
const LoRaRadioOps kLoRaRadioOps = {
	.loramac_node = false,
	.rssi_inst = lr11xx_get_rssi_inst,
	.rssi_burst = lr11xx_get_rssi_burst,
	.is_receiving = lr11xx_is_receiving,
	.set_rx_boost = lr11xx_set_rx_boost,
	.set_fem_rx = nullptr,
	.is_chip_busy = nullptr,
	.wakeup_time_us = lr11xx_get_wakeup_time_us,
	.dc_restarts = lr11xx_get_dc_timeout_restarts,
	.reset_dc_restarts = lr11xx_reset_dc_timeout_restarts,
	.reset_agc = nullptr,
	.recalibrate = lr11xx_recalibrate,
	.cad_probe = lr11xx_cad_probe,
	.cad_rx_outcome = lr11xx_cad_rx_outcome,
	.cad_rx_timeout_ms = lr11xx_cad_rx_timeout_ms,
	.cad_set_peak_offset = lr11xx_cad_set_peak_offset,
	.cad_base_peak = lr11xx_cad_base_peak,
	.cad_peak_min = lr11xx_cad_peak_min,
	.cad_peak_max = lr11xx_cad_peak_max,
	.post_begin = nullptr,
	.side_detectors = nullptr,
	.reset_stats = nullptr,
	.format_freq_error = nullptr,
};

} /* namespace mesh */
