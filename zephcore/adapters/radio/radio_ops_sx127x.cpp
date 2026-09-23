/*
 * SPDX-License-Identifier: MIT
 * SX127x family over Zephyr's loramac-node backend, for LoRaRadio.
 *
 * The backend exposes nothing beyond the Zephyr LoRa API: no RSSI readout
 * (the noise floor reads a flat -80), no RX-busy gate, no RX boost (the chip
 * has none), no CAD and no duty cycle. Every entry is therefore NULL.
 */

#include "LoRaRadioOps.h"

namespace mesh {

const LoRaRadioOps kLoRaRadioOps = {
	.loramac_node = true,
};

} /* namespace mesh */
