/*
 * SPDX-License-Identifier: Apache-2.0
 * NodePrefs - persisted node configuration (unified for all roles)
 *
 * Serialized field-by-field, not raw memcpy; struct layout does
 * not affect on-disk compatibility.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#define TELEM_MODE_DENY            0
#define TELEM_MODE_ALLOW_FLAGS     1
#define TELEM_MODE_ALLOW_ALL       2

#define ADVERT_LOC_NONE       0
#define ADVERT_LOC_SHARE      1
#define ADVERT_LOC_PREFS      2

#define LOOP_DETECT_OFF       0
#define LOOP_DETECT_MINIMAL   1
#define LOOP_DETECT_MODERATE  2
#define LOOP_DETECT_STRICT    3

struct NodePrefs {
	/* ---- Common fields (both roles) ---- */
	float airtime_factor;
	char node_name[32];
	double node_lat, node_lon;
	char password[16];
	float freq;
	int8_t tx_power_dbm;
	uint8_t disable_fwd;            // repeater: disable forwarding
	uint8_t advert_interval;        // stored as minutes / 2
	uint8_t flood_advert_interval;  // hours
	float rx_delay_base;
	float tx_delay_factor;
	char guest_password[16];
	float direct_tx_delay_factor;
	float backoff_multiplier;       // per-dupe reactive backoff (0.0 = disabled)
	uint32_t guard;
	uint8_t sf;
	uint8_t cr;
	uint8_t allow_read_only;
	uint8_t multi_acks;
	float bw;
	uint8_t flood_max;
	uint8_t interference_threshold;
	uint8_t agc_reset_interval;     // stored as secs / 4
	// Power saving
	uint8_t powersaving_enabled;
	// GPS settings
	uint8_t gps_enabled;
	uint32_t gps_interval;          // in seconds
	uint8_t advert_loc_policy;
	uint32_t discovery_mod_timestamp;
	float adc_multiplier;
	char owner_info[120];
	uint8_t rx_boost;               // 1 = boosted RX gain (+3dB), 0 = power save
	uint8_t rx_duty_cycle;          // 1 = RX duty cycle, 0 = continuous RX
	uint8_t apc_enabled;            // 1 = APC on, 0 = fixed TX power
	uint8_t apc_margin;             // APC target link margin dB (6-30)

	/* ---- Companion-only fields ---- */
	uint8_t manual_add_contacts;
	uint8_t telemetry_mode_base;
	uint8_t telemetry_mode_loc;
	uint8_t telemetry_mode_env;
	uint32_t ble_pin;
	uint8_t buzzer_quiet;
	uint8_t autoadd_config;
	uint8_t client_repeat;          // 1 = offgrid mode (forward packets)
	uint8_t path_hash_mode;         // path mode 0-2
	uint8_t autoadd_max_hops;       // 0 = no limit, N = up to N-1 hops
	uint8_t loop_detect;            // LOOP_DETECT_{OFF,MINIMAL,MODERATE,STRICT}
	uint8_t leds_disabled;          // 1 = LEDs off
	char default_scope_name[31];    // companion: default flood scope region name ("" = null)
	uint8_t default_scope_key[16];  // companion: default flood scope TransportKey
};

/* Default prefs -- must match LoRaConfig.h defaults for radio interop. */
static inline void initNodePrefs(NodePrefs* prefs) {
	memset(prefs, 0, sizeof(NodePrefs));
	prefs->airtime_factor = 9.0f;  /* Arduino formula: duty% = 100 / (af + 1) → 10% */
	prefs->node_lat = 0.0;
	prefs->node_lon = 0.0;
#ifdef CONFIG_ZEPHCORE_ADMIN_PASSWORD
	strncpy(prefs->password, CONFIG_ZEPHCORE_ADMIN_PASSWORD, sizeof(prefs->password) - 1);
#else
	strcpy(prefs->password, "password");
#endif
#ifdef CONFIG_ZEPHCORE_GUEST_PASSWORD
	strncpy(prefs->guest_password, CONFIG_ZEPHCORE_GUEST_PASSWORD, sizeof(prefs->guest_password) - 1);
#endif
	/* Radio params - MUST match LoRaConfig.h for interop with companion nodes */
	prefs->freq = 869.618f;           // LoRaConfig::FREQ_HZ / 1000000.0
	prefs->bw = 62.5f;                // LoRaConfig::BANDWIDTH
	prefs->sf = 8;                    // LoRaConfig::SPREADING_FACTOR
	prefs->cr = 8;                    // CR 4/8 (MeshCore uses 5-8 for CR 4/5 through 4/8)
#ifdef CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM
	prefs->tx_power_dbm = CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM;
#else
	prefs->tx_power_dbm = 22;         // LoRaConfig::TX_POWER_DBM
#endif
	prefs->disable_fwd = 0;
	prefs->advert_interval = 60;      // 2 minutes (value / 2)
	prefs->flood_advert_interval = 12; // 12 hours
	prefs->rx_delay_base = 0.0f;
	prefs->tx_delay_factor = 0.5f;
	prefs->direct_tx_delay_factor = 0.3f;
	prefs->allow_read_only = 0;
	prefs->multi_acks = 0;
	prefs->flood_max = 64;            // max hops for flood packets (0 = blocking all!)
	prefs->interference_threshold = 0;
	prefs->agc_reset_interval = 0;
	prefs->powersaving_enabled = 0;
	prefs->gps_enabled = 0;
	prefs->gps_interval = 300;        // 5 minutes
	prefs->advert_loc_policy = ADVERT_LOC_NONE;
	prefs->adc_multiplier = 0.0f;
	prefs->rx_boost = 1;              // Default to boosted RX for better sensitivity
	prefs->rx_duty_cycle = 0;         // Default OFF — continuous RX for best reliability
	prefs->apc_enabled = 0;           // Default OFF — fixed TX power
	prefs->apc_margin = 16;           // Default 16 dB target link margin
}
