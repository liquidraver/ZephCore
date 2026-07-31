/*
 * SPDX-License-Identifier: MIT
 * ZephCore LoRa default parameters
 */

#pragma once

#include <stdint.h>

namespace mesh {

struct LoRaConfig {
	static constexpr uint32_t FREQ_HZ = 869618000;   /* MeshCore default channel (EU 869 MHz ISM) */
	static constexpr uint16_t BANDWIDTH = 62;         /* BW_62_KHZ — MeshCore default */
	static constexpr uint8_t SPREADING_FACTOR = 8;    /* SF8: balance of range vs airtime */
	static constexpr uint8_t CODING_RATE = 8;         /* CR_4_8: max FEC */
	static constexpr uint16_t PREAMBLE_LEN = 16;      /* MeshCore default; longer aids RX sync */
	static constexpr int8_t TX_POWER_DBM = 22;        /* SX1262 max */
};

} /* namespace mesh */
