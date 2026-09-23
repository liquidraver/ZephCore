/*
 * SPDX-License-Identifier: MIT
 * SX126x family (SX1261/2/8, LLCC68, STM32WL) for LoRaRadio.
 */

#include "LoRaRadioOps.h"

extern "C" {
#include <zephyr/drivers/lora/sx126x_ext.h>
}

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sx126x_radio, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

#if IS_ENABLED(CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH)
/* Undocumented register 0x8B5 RX improvement (MeshCore PR#1398), applied once
 * the first lora_config() has run. */
static void sx126x_post_begin(const struct device *dev)
{
	sx126x_apply_heltec_reg_patch(dev);
	LOG_INF("Applied Heltec reg 0x8B5 RX patch");
}
#endif

/* The one family with the jammed-AGC fault, so reset_agc is set (the remedy
 * re-issues CalibrateImage itself); drift recalibration is not specified by
 * the datasheet, so recalibrate stays NULL. */
const LoRaRadioOps kLoRaRadioOps = {
	.loramac_node = false,
	.rssi_inst = sx126x_get_rssi_inst,
	.rssi_burst = nullptr,
	.is_receiving = sx126x_is_receiving,
	.set_rx_boost = sx126x_set_rx_boost,
	.set_fem_rx = sx126x_set_fem_rx_enable,
	.is_chip_busy = sx126x_is_chip_busy,
	.wakeup_time_us = sx126x_get_wakeup_time_us,
	.dc_restarts = sx126x_get_dc_timeout_restarts,
	.reset_dc_restarts = sx126x_reset_dc_timeout_restarts,
	.reset_agc = sx126x_reset_agc,
	.recalibrate = nullptr,
	.cad_probe = sx126x_cad_probe,
	.cad_rx_outcome = sx126x_cad_rx_outcome,
	.cad_rx_timeout_ms = sx126x_cad_rx_timeout_ms,
	.cad_set_peak_offset = sx126x_cad_set_peak_offset,
	.cad_base_peak = sx126x_cad_base_peak,
	.cad_peak_min = sx126x_cad_peak_min,
	.cad_peak_max = sx126x_cad_peak_max,
#if IS_ENABLED(CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH)
	.post_begin = sx126x_post_begin,
#else
	.post_begin = nullptr,
#endif
	.side_detectors = nullptr,
	.reset_stats = nullptr,
	.format_freq_error = nullptr,
};

} /* namespace mesh */
