/*
 * SPDX-License-Identifier: MIT
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

/* Adaptive-CAD operating detPeak offset range (levels from the family base).
 * Wide on purpose: a dense hilltop can need a much higher detPeak than a quiet
 * valley node.  The per-family absolute clamp inside the driver (SX126x 15-40,
 * LR11xx/LR20xx 48-90) is a firmware guardrail, NOT a chip limit — cadDetPeak
 * is a full uint8_t (0-255).  It just keeps the staircase from wandering into
 * "CAD never fires" (too high) or "CAD always busy" (too low) territory.  This
 * offset range limits how far the staircase / manual offset may roam; MUST
 * match CAD_LEVEL_MIN/MAX in adapters/radio/radio_common.h (they index the
 * per-level stats array). */
#define CAD_OFFSET_MIN  (-8)
#define CAD_OFFSET_MAX  12

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
	uint8_t flood_max_unscoped;     // hop limit for un-scoped (ROUTE_TYPE_FLOOD) floods
	uint8_t flood_max_advert;       // hop limit for ADVERT floods (curbs advert churn)
	uint8_t interference_threshold;
	uint8_t agc_reset_interval;     // RETIRED: read/written for on-disk layout
	                                // compatibility only, never acted on.
	                                // "set agc.reset.interval" replies
	                                // "use rxduty instead".
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
	/* RESERVED — formerly apc_enabled / apc_margin (Adaptive Power Control,
	 * removed in 1.16.6). These two bytes are still read and written at their
	 * original offsets in all three prefs serializers (companion new_prefs 94/95,
	 * repeater prefs 292/293, RepeaterDataStore) because every field after them
	 * is positional: dropping them would shift the rest of the layout and make
	 * every already-deployed node misparse its saved prefs on upgrade.
	 * Do not reuse for a new setting — an upgraded node still has the old APC
	 * values sitting in these bytes. */
	uint8_t _reserved_apc_enabled;
	uint8_t _reserved_apc_margin;
	uint8_t meshtimesync;           // 1 = mesh time-sync clock correction on (default off)
	uint8_t cad_auto;               // 1 = adaptive-CAD staircase acts on probe stats (default off = dry-run)
	int8_t cad_offset;              // operating detPeak offset from family base (-4..4)
	uint8_t probe_interval;         // seconds between periodic radio measurements:
	                                // one noise-floor sample, and the CAD probe that
	                                // consumes it (0 = CAD probing off, default 15)
	uint8_t cad_busycap;            // airtime-protection: max % of TX attempts deferred before backing off detPeak (0 = off, default 25)

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
	uint8_t ble_disabled;           // 1 = BLE advertising off
	uint8_t display_brightness;     // 0 = default (100%), else 10–100
	uint8_t wake_on_msg;            // 0 = don't wake display on message, 1 = wake (default)
	uint16_t screen_off_secs;       // 0 = default (Kconfig), else 5–300
	uint16_t auto_shutdown_mv;      // low-batt auto-shutdown threshold; 0 = off, else 2900–4200
	uint8_t v_contact_enabled;      // v-contact (loopback admin chat via BLE/USB); 1 = on (default)
	uint16_t v_battery_alert_mv;    // 0 = alert off; 0xFFFF = board default (auto_shutdown+200); else mV
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
	prefs->advert_interval = 0;       // 0 = periodic local advert off; else minutes = value * 2
	prefs->flood_advert_interval = 47;  // hours
	prefs->rx_delay_base = 0.0f;
	prefs->tx_delay_factor = 0.5f;
	prefs->direct_tx_delay_factor = 0.3f;
	prefs->allow_read_only = 0;
	prefs->multi_acks = 0;
	prefs->flood_max = 64;            // max hops for flood packets (0 = blocking all!)
	prefs->flood_max_unscoped = 64;  // un-scoped flood hop limit (defaults to flood_max)
	prefs->flood_max_advert = 8;     // ADVERT flood hop limit (upstream default)
	prefs->interference_threshold = 0;
	prefs->agc_reset_interval = 0;
	prefs->powersaving_enabled = 0;
	prefs->gps_enabled = 0;
	prefs->gps_interval = 300;        // 5 minutes
	prefs->advert_loc_policy = ADVERT_LOC_NONE;
	prefs->adc_multiplier = 0.0f;
	prefs->rx_boost = 1;              // Default to boosted RX for better sensitivity
	prefs->rx_duty_cycle = 0;         // Default OFF — continuous RX for best reliability
	prefs->_reserved_apc_enabled = 0; // reserved (was APC), see NodePrefs
	prefs->_reserved_apc_margin = 0;  // reserved (was APC), see NodePrefs
	prefs->cad_auto = 1;              // Default ON — adaptive staircase acts on probe stats
	prefs->cad_offset = 0;            // Start at family base detPeak (SF+13 on SX126x)
	prefs->probe_interval = 15;       // floor sample + CAD probe; staircase responds in ~1-2 h
	prefs->cad_busycap = 25;          // back off detPeak once >25% of TX attempts are deferred
	prefs->wake_on_msg = 1;           // Default ON — wake display when message arrives
	prefs->v_contact_enabled = 1;     // Default ON — v-contact loopback admin chat (companion)
	prefs->v_battery_alert_mv = 0xFFFF; // Sentinel: derive from board auto-shutdown threshold
}
