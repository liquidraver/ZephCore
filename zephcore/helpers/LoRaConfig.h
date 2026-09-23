/*
 * SPDX-License-Identifier: MIT
 * ZephCore LoRa default parameters
 *
 * The one source of radio defaults. They only seed NodePrefs (initNodePrefs(),
 * RepeaterDataStore's invalid-prefs fallback); the radio itself always reads
 * prefs. Units match NodePrefs.
 */

#pragma once

#include <stdint.h>

namespace mesh {

struct LoRaConfig {
	static constexpr float FREQ_MHZ = 869.618f;       /* MeshCore default channel (EU 869 MHz ISM) */
	static constexpr float BANDWIDTH_KHZ = 62.5f;     /* MeshCore default */
	static constexpr uint8_t SPREADING_FACTOR = 7;    /* SF7: HU regional default since 2026-08 */
	static constexpr uint8_t CODING_RATE = 5;         /* CR 4/5: lowest overhead */
#ifdef CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM
	static constexpr int8_t TX_POWER_DBM = CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM; /* per board (external PAs) */
#else
	static constexpr int8_t TX_POWER_DBM = 22;        /* SX1262 max */
#endif
};

} /* namespace mesh */
