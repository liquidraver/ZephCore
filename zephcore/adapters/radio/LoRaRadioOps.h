/*
 * SPDX-License-Identifier: MIT
 * What LoRaRadio needs from a radio family beyond the Zephyr LoRa API: a const
 * table of the driver's extension functions. Every entry is optional; NULL
 * selects the documented default. Exactly one table, kLoRaRadioOps, is linked
 * per build, from radio_ops_<family>.cpp (chosen by the radio Kconfig).
 */

#pragma once

#include <zephyr/device.h>
#include <stdint.h>

namespace mesh {

struct LoRaRadioOps {
	/* loramac-node backend (SX127x): its TX and RX configs are disjoint
	 * state, so configure() must not take the direction-only fast path. */
	bool loramac_node;

	/* Instantaneous RSSI in dBm. NULL: no readout, -80 is reported. */
	int16_t (*rssi_inst)(const struct device *dev);
	/* A bracketed burst (see LoRaRadio::hwGetRssiBurst). NULL: one
	 * rssi_inst() per sample. */
	int (*rssi_burst)(const struct device *dev, int16_t *out, int n,
			  uint32_t spacing_us);
	/* RX-busy gate (latch + IRQ read). NULL: never busy. */
	bool (*is_receiving)(const struct device *dev);
	/* NULL: the chip has no RX boost; setRxBoost() reports false. */
	void (*set_rx_boost)(const struct device *dev, bool enable);
	/* External FEM LNA select. NULL: not software-selectable. */
	bool (*set_fem_rx)(const struct device *dev, bool enable);
	/* GPIO-only BUSY check. NULL: never busy. */
	bool (*is_chip_busy)(const struct device *dev);
	/* Deaf time per duty-cycle wake, us. NULL: 1500 (XTAL parts). */
	uint32_t (*wakeup_time_us)(const struct device *dev);
	/* Duty-cycle false-preamble re-arms. NULL: 0. */
	uint32_t (*dc_restarts)(const struct device *dev);
	void (*reset_dc_restarts)(const struct device *dev);

	/* Receiver hygiene. Non-NULL means the family needs it: reset_agc for
	 * the jammed-AGC fault (SX126x), recalibrate for temperature drift (LR).
	 * Both leave the driver out of RX. */
	void (*reset_agc)(const struct device *dev);
	void (*recalibrate)(const struct device *dev);

	/* Adaptive CAD (see CadController / LoRaRadio::cadMaintenance). NULL
	 * probe or rx_timeout_ms: no probing. NULL base_peak: no adaptive CAD. */
	int (*cad_probe)(const struct device *dev, int8_t level);
	int (*cad_rx_outcome)(const struct device *dev);
	uint32_t (*cad_rx_timeout_ms)(const struct device *dev);
	void (*cad_set_peak_offset)(const struct device *dev, int8_t offset);
	uint8_t (*cad_base_peak)(const struct device *dev);
	uint8_t (*cad_peak_min)(void);
	uint8_t (*cad_peak_max)(void);

	/* Family extras. */
	void (*post_begin)(const struct device *dev);
	int (*side_detectors)(const struct device *dev, const uint8_t *sfs,
			      uint8_t num);
	void (*reset_stats)(const struct device *dev);
	int (*format_freq_error)(const struct device *dev, char *buf, int cap);
};

extern const LoRaRadioOps kLoRaRadioOps;

} /* namespace mesh */
