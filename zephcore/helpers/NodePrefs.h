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

/* LEDS_RADIO_* / LEDS_HB_* mode values for the leds_radio_mode and leds_hb_mode
 * fields below.  They live in led_gate.h because the heartbeat consumers are C
 * and this header is C++; see the note there. */
#include "led_gate.h"

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

/* leds_disabled, as stored in the repeater/room-server/observer prefs layout.
 *
 * It occupies the byte that used to hold agc_reset_interval (offset 120),
 * retired when periodic AGC recalibration was removed.  That byte is NOT
 * reusable as a plain 0/1 boolean: the old command stored seconds/4, so a node
 * upgrading from a build that had it configured has an arbitrary small integer
 * sitting there, and a bare non-zero test would silently kill its LEDs.  Hence
 * a magic encoding — anything that is not one of these two values is a legacy
 * AGC interval and decodes to the default (LEDs on).  The first savePrefs()
 * claims the byte for good.
 *
 * The companion layout is unaffected: it has always stored leds_disabled as a
 * plain 0/1 at its own offset 93. */
#define LEDS_PREF_ON    0xA0
#define LEDS_PREF_OFF   0xA1

/* Number of LR2021 side-detector slots stored in prefs.  Matches the chip's
 * ConfigureSideDetectors limit of 3. */
#define EXTRA_SF_MAX    3

/* Display timezone offset range, whole hours from UTC.  Matches upstream
 * MeshCore's `set tz.offset` so the same command works against both trees. */
#define TZ_OFFSET_MIN   (-12)
#define TZ_OFFSET_MAX   14

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
	uint8_t leds_disabled;          // 1 = all LEDs off (heartbeat, unread, LoRa TX)
	uint8_t leds_radio_mode;        // LEDS_RADIO_* — activity LED source (0 = TX, as before)
	uint8_t leds_hb_mode;           // LEDS_HB_* — heartbeat LED behaviour (0 = all, as before)
	uint8_t led_brightness;         // 0-100, PWM-capable boards only; ZEPHCORE_LED_DEFAULT_BRIGHTNESS_PCT on a new node
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
	uint8_t fem_rxgain;             // 1 = external FEM LNA active during RX, 0 = off (power save)
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
	uint8_t cad_auto;               // 1 = adaptive-CAD staircase acts on probe stats (default ON)
	int8_t cad_offset;              // operating detPeak offset from family base (CAD_OFFSET_MIN..MAX, -8..+12)
	uint8_t probe_interval;         // seconds between periodic radio measurements:
	                                // one noise-floor sample, and the CAD probe that
	                                // consumes it (0 = CAD probing off, default 15)
	uint8_t cad_busycap;            // faint-tolerance / airtime cap: raise detPeak once more than this %
	                                // of QUIET-MOMENT PROBES trip on a faint signal (0 = off, default 15).
	                                // Not "% of TX attempts deferred" -- probes are prefiltered to quiet
	                                // moments, so strong traffic never enters the statistic at all.
	/* LR2021 side detectors: extra spreading factors received concurrently
	 * with `sf`, on the same bandwidth.  Zero-terminated list — the first 0
	 * ends it, so an all-zero array means the feature is off (which is what
	 * every node upgrading from a build without this field has).  Every
	 * entry must be greater than `sf`, all distinct, spread <= 4; the
	 * driver rejects anything else.  Ignored on non-LR2021 radios. */
	uint8_t extra_sf[EXTRA_SF_MAX];

	/* Physical mounting orientation.  Common to both roles: a repeater board
	 * with an OLED (RAK4631, Heltec) can be mounted upside down just as a
	 * companion can, and the joystick boards ship a repeater artifact too.
	 *
	 * display_rotate rotates the panel 180 degrees in hardware and is only
	 * honoured on SSD1306/SH1106 (see MC_DISPLAY_ROTATE_SUPPORTED); other
	 * panels report it unsupported rather than silently ignoring it.
	 * input_rotate swaps the joystick/D-pad axes to match, and is kept
	 * separate because the two are not always wanted together — a screen can
	 * be remounted without moving the stick. */
	uint8_t display_rotate;         // 1 = panel rotated 180 degrees
	uint8_t input_rotate;           // 1 = joystick up/down and left/right swapped

	/* Whole-hour offset from UTC, -12..+14, applied ONLY when formatting a
	 * clock for the local display.  It must never reach RTCClock or any
	 * timestamp that leaves this node: a negative offset applied to the clock
	 * itself reads as a backward jump to every timestamp consumer at once
	 * (advert timestamps, the repeater ACL's monotonic sender_timestamp gate,
	 * discovery_mod_timestamp, MeshTimeSync), and a backward clock is a silent
	 * mesh-wide mute.  The CLI's `clock` / `time` commands stay UTC for the
	 * same reason -- they round-trip with each other and apps parse them.
	 *
	 * Whole hours only, matching upstream's `set tz.offset` so an app that
	 * speaks to both trees behaves the same.  Half-hour zones (India +5:30,
	 * Newfoundland -3:30) therefore cannot be expressed.
	 *
	 * Common to both roles: a repeater with an OLED shows a clock too.  On a
	 * headless repeater the field is simply inert. */
	int8_t tz_offset;

	/* The family base detPeak that cad_offset was learned against (0 = never
	 * stored, i.e. a node upgrading from a build without this field).
	 *
	 * cad_offset is a SIGNED OFFSET from a per-SF/per-bandwidth base table
	 * that lives in the driver, so changing that table silently re-points a
	 * stored offset at a different absolute detPeak.  The probe statistics
	 * behind the offset are RAM-only and die at the reboot a firmware upgrade
	 * involves, but the offset itself is persisted and survives — so after a
	 * table change a converged node quietly starts operating somewhere it
	 * never measured.  Recording the base turns that into something the
	 * firmware can correct at boot (see LoRaRadioBase::setCadParams), instead
	 * of a "run set cad.reset after upgrading" line in the release notes that
	 * most users will not read. */
	uint8_t cad_base;

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
	char default_scope_name[31];    // companion: default flood scope region name ("" = null)
	uint8_t default_scope_key[16];  // companion: default flood scope TransportKey
	uint8_t ble_disabled;           // 1 = BLE advertising off
	uint8_t display_brightness;     // 0 = default (100%), else 10–100
	uint8_t wake_on_msg;            // 0 = don't wake display on message, 1 = wake (default)
	uint16_t screen_off_secs;       // 0 = default (Kconfig), else 5–300
	uint16_t auto_shutdown_mv;      // low-batt auto-shutdown threshold; 0 = off, else 2900–4200
	uint8_t v_contact_enabled;      // v-contact (loopback admin chat via BLE/USB); 1 = on (default)
	uint16_t v_battery_alert_mv;    // 0 = alert off; 0xFFFF = board default (auto_shutdown+200); else mV
	/* App-owned ContactInfo.flags byte for the v-contact.  The v-contact never
	 * enters the contacts table, so it has no record to hold the flags the app
	 * sets via CMD_ADD_UPDATE_CONTACT -- bit 0 is the 'favourite' star, the
	 * upper bits are telemetry permissions.  Kept here so a toggle survives
	 * reconnects and reboots instead of being echoed back as 0. */
	uint8_t v_contact_flags;
};

/* Default prefs -- must match LoRaConfig.h defaults for radio interop. */
/* Range guards for prefs that came off flash.
 *
 * The atomic replace in every savePrefs() plus littlefs's own CRCs make a torn
 * write impossible, so this is not about power loss — it is about a blob that
 * is structurally intact and semantically wrong.  Several of these fields make
 * a node look bricked when they are: auto_shutdown_mv powers it off seconds
 * after boot, ble_pin locks pairing out, and the char fields are stored as
 * fixed-size blocks with no terminator in the file format, so an unterminated
 * one runs every later %s off the end of the struct.
 *
 * Called at the end of each role's loadPrefs().  Fields whose whole range is
 * legal (autoadd_config bitmask, discovery_mod_timestamp, the v_contact_*
 * sentinels) are deliberately left alone. */
template <typename T>
static inline T clampPref(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* Floats need the NaN test spelled out: every comparison against NaN is false,
 * so a plain clamp passes it straight through. */
template <typename T>
static inline T sanePrefFloat(T v, T lo, T hi, T fallback) {
	if (v != v || v < lo || v > hi) return fallback;
	return v;
}

/* A byte that is neither 0 nor 1 carries no user intent — take the default
 * rather than clamping, which would round every garbage value up to "on". */
template <typename T>
static inline T saneBool(T v, T fallback) { return (v == 0 || v == 1) ? v : fallback; }

/* Same idea as saneBool for a small enum: anything outside 0..max carries no
 * user intent, so fall back to 0 — which every LEDS_* enum defines as the
 * behaviour the firmware had before the setting existed. */
static inline uint8_t saneEnum(uint8_t v, uint8_t max) { return (v <= max) ? v : 0; }

static inline void sanitizeNodePrefs(NodePrefs* p) {
	p->node_name[sizeof(p->node_name) - 1] = '\0';
	p->password[sizeof(p->password) - 1] = '\0';
	p->guest_password[sizeof(p->guest_password) - 1] = '\0';
	p->owner_info[sizeof(p->owner_info) - 1] = '\0';
	p->default_scope_name[sizeof(p->default_scope_name) - 1] = '\0';

	p->airtime_factor = sanePrefFloat(p->airtime_factor, 0.0f, 9.0f, 9.0f);
	p->rx_delay_base  = sanePrefFloat(p->rx_delay_base, 0.0f, 3600.0f, 0.0f);
	p->adc_multiplier = sanePrefFloat(p->adc_multiplier, 0.0f, 30000.0f, 0.0f);
	/* Range matches the CLI (0.0-2.0).  0.0 is a legal value meaning "reactive
	 * backoff off" and must survive: this guard exists for NaN and corruption
	 * only.  RepeaterDataStore used to coerce 0.0 -> 0.2 here, which silently
	 * undid the documented way to disable the feature on every boot. */
	p->backoff_multiplier = sanePrefFloat(p->backoff_multiplier, 0.0f, 2.0f, 0.2f);
	p->node_lat = sanePrefFloat(p->node_lat, -90.0, 90.0, 0.0);
	p->node_lon = sanePrefFloat(p->node_lon, -180.0, 180.0, 0.0);

	p->cr           = clampPref<uint8_t>(p->cr, 5, 8);
	p->tx_power_dbm = clampPref<int8_t>(p->tx_power_dbm, -9, 30);
#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
	if (p->tx_power_dbm > CONFIG_ZEPHCORE_MAX_TX_POWER_DBM) {
		p->tx_power_dbm = (int8_t)CONFIG_ZEPHCORE_MAX_TX_POWER_DBM;
	}
#endif

	/* Booleans and enums fall back to a safe value rather than being clamped
	 * to their maximum: clamping is the wrong direction for anything that
	 * grants a permission, hides the node, or turns a radio mode on.  A byte
	 * that is neither 0 nor 1 carries no user intent, so the fallback is
	 * simply the field's own default. */
	p->client_repeat       = saneBool<uint8_t>(p->client_repeat, 0);
	p->manual_add_contacts = saneBool<uint8_t>(p->manual_add_contacts, 0);
	p->multi_acks          = saneBool<uint8_t>(p->multi_acks, 0);
	p->buzzer_quiet        = saneBool<uint8_t>(p->buzzer_quiet, 0);
	p->gps_enabled         = saneBool<uint8_t>(p->gps_enabled, 0);
	p->rx_duty_cycle       = saneBool<uint8_t>(p->rx_duty_cycle, 0);
	p->leds_disabled       = saneBool<uint8_t>(p->leds_disabled, 0);
	p->leds_radio_mode     = saneEnum(p->leds_radio_mode, LEDS_RADIO_MAX);
	p->leds_hb_mode        = saneEnum(p->leds_hb_mode, LEDS_HB_MAX);
	p->led_brightness      = clampPref<uint8_t>(p->led_brightness, 0, 100);
	p->meshtimesync        = saneBool<uint8_t>(p->meshtimesync, 0);
	/* Fallback must be the initNodePrefs() default (ON).  It was 0, so a byte
	 * that was neither 0 nor 1 silently switched adaptive CAD off instead of
	 * restoring the default — left over from when the default was dry-run. */
	p->cad_auto            = saneBool<uint8_t>(p->cad_auto, 1);
	p->allow_read_only     = saneBool<uint8_t>(p->allow_read_only, 0);
	p->powersaving_enabled = saneBool<uint8_t>(p->powersaving_enabled, 0);
	p->display_rotate      = saneBool<uint8_t>(p->display_rotate, 0);
	p->input_rotate        = saneBool<uint8_t>(p->input_rotate, 0);
	/* Defaults that are on, not off. */
	p->rx_boost            = saneBool<uint8_t>(p->rx_boost, 1);
	p->fem_rxgain          = saneBool<uint8_t>(p->fem_rxgain, 1);
	p->wake_on_msg         = saneBool<uint8_t>(p->wake_on_msg, 1);
	p->v_contact_enabled   = saneBool<uint8_t>(p->v_contact_enabled, 1);
	/* Never let a corrupt byte take the radio off the air or hide BLE — both
	 * remove the only ways left to fix the node. */
	p->ble_disabled        = saneBool<uint8_t>(p->ble_disabled, 0);

	/* Permission and privacy enums: out of range means deny/withhold. */
	if (p->telemetry_mode_base > TELEM_MODE_ALLOW_ALL) p->telemetry_mode_base = TELEM_MODE_DENY;
	if (p->telemetry_mode_loc  > TELEM_MODE_ALLOW_ALL) p->telemetry_mode_loc  = TELEM_MODE_DENY;
	if (p->telemetry_mode_env  > TELEM_MODE_ALLOW_ALL) p->telemetry_mode_env  = TELEM_MODE_DENY;
	if (p->advert_loc_policy   > ADVERT_LOC_PREFS)     p->advert_loc_policy   = ADVERT_LOC_NONE;

	if (p->loop_detect    > LOOP_DETECT_STRICT) p->loop_detect    = LOOP_DETECT_MINIMAL;
	if (p->path_hash_mode > 2)                  p->path_hash_mode = 0;
	p->autoadd_max_hops    = clampPref<uint8_t>(p->autoadd_max_hops, 0, 64);
	p->flood_max           = clampPref<uint8_t>(p->flood_max, 0, 64);
	p->flood_max_unscoped  = clampPref<uint8_t>(p->flood_max_unscoped, 0, 64);
	p->flood_max_advert    = clampPref<uint8_t>(p->flood_max_advert, 0, 64);
	p->cad_busycap         = clampPref<uint8_t>(p->cad_busycap, 0, 90);
	/* Neutral, not the boundary — a garbage offset is not a request to run
	 * the CAD staircase at one end of its range. */
	if (p->cad_offset < CAD_OFFSET_MIN || p->cad_offset > CAD_OFFSET_MAX) p->cad_offset = 0;
	/* UTC, not a boundary zone, for a byte that carries no user intent. */
	if (p->tz_offset < TZ_OFFSET_MIN || p->tz_offset > TZ_OFFSET_MAX) p->tz_offset = 0;
	if (p->probe_interval != 0 && p->probe_interval < 10) p->probe_interval = 10;

	/* 6-digit BLE passkey — anything else is rejected at pairing time and
	 * leaves no way back in over BLE. */
	if (p->ble_pin > 999999) p->ble_pin = 0;
	/* Seconds; 0 = off.  A year is already far past any sane setting. */
	if (p->gps_interval > 31536000UL) p->gps_interval = 0;
	/* 0 = board/Kconfig default on both, else the range the UI offers. */
	if (p->display_brightness != 0) p->display_brightness = clampPref<uint8_t>(p->display_brightness, 10, 100);
	if (p->screen_off_secs != 0) p->screen_off_secs = clampPref<uint16_t>(p->screen_off_secs, 5, 300);
	/* 0 = off.  Outside the cell range it either never fires or powers the
	 * node off the moment it boots — the one that looks like a dead device. */
	if (p->auto_shutdown_mv != 0 &&
	    (p->auto_shutdown_mv < 2900 || p->auto_shutdown_mv > 4200)) {
#ifdef CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS
		p->auto_shutdown_mv = CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS;
#else
		p->auto_shutdown_mv = 0;
#endif
	}
}

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
	prefs->sf = 7;                    // LoRaConfig::SPREADING_FACTOR
	prefs->cr = 5;                    // CR 4/5 (MeshCore uses 5-8 for CR 4/5 through 4/8)
#ifdef CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM
	prefs->tx_power_dbm = CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM;
#else
	prefs->tx_power_dbm = 22;         // LoRaConfig::TX_POWER_DBM
#endif
	prefs->disable_fwd = 0;
	prefs->advert_interval = 0;       // 0 = periodic local advert off; else minutes = value * 2
	prefs->flood_advert_interval = 47;  // hours
	prefs->rx_delay_base = 0.0f;
	/* Must match mesh::ContentionTracker::DEFAULT_BACKOFF_MULT.  Left at the
	 * memset 0 (= backoff disabled) until now, with RepeaterDataStore coercing
	 * 0.0 -> 0.2 on load to paper over it — which also undid a deliberate 0.0.
	 * Companions never apply this (only Repeater/RoomServer call
	 * setBackoffMultiplier), so setting it here changes no existing node. */
	prefs->backoff_multiplier = 0.2f;
	prefs->tx_delay_factor = 0.5f;
	prefs->direct_tx_delay_factor = 0.3f;
	prefs->allow_read_only = 0;
	prefs->multi_acks = 0;
	prefs->flood_max = 64;            // max hops for flood packets (0 = blocking all!)
	prefs->flood_max_unscoped = 64;  // un-scoped flood hop limit (defaults to flood_max)
	prefs->flood_max_advert = 8;     // ADVERT flood hop limit (upstream default)
	prefs->interference_threshold = 0;
	prefs->leds_disabled = 0;         // LEDs on
	prefs->leds_radio_mode = LEDS_RADIO_TX;  // activity LED on transmit, as before
	prefs->leds_hb_mode = LEDS_HB_ALL;       // heartbeat + unread, as before
	prefs->led_brightness = ZEPHCORE_LED_DEFAULT_BRIGHTNESS_PCT;  // 100%, new node
	prefs->powersaving_enabled = 0;
	prefs->gps_enabled = 0;
	prefs->gps_interval = 300;        // 5 minutes
	prefs->advert_loc_policy = ADVERT_LOC_NONE;
	prefs->adc_multiplier = 0.0f;
	prefs->rx_boost = 1;              // Default to boosted RX for better sensitivity
	/* Default ON: the FEM's chip-enable has always been asserted for RX on the
	 * boards that have one, so 1 is the historical behaviour and the only safe
	 * default -- 0 costs the FEM's RX gain (~16 dB on SKY66122). */
	prefs->fem_rxgain = 1;
	prefs->rx_duty_cycle = 0;         // Default OFF — continuous RX for best reliability
	prefs->_reserved_apc_enabled = 0; // reserved (was APC), see NodePrefs
	prefs->_reserved_apc_margin = 0;  // reserved (was APC), see NodePrefs
	prefs->cad_auto = 1;              // Default ON — adaptive staircase acts on probe stats
	prefs->cad_offset = 0;            // Start at family base detPeak (SF+13 on SX126x)
	prefs->probe_interval = 15;       // floor sample + CAD probe; staircase responds in ~1-2 h
	prefs->cad_busycap = 15;          // back off detPeak once >15% of quiet-moment probes trip on faint signals
	prefs->wake_on_msg = 1;           // Default ON — wake display when message arrives
	prefs->display_rotate = 0;        // Default OFF — panel in the stock case orientation
	prefs->input_rotate = 0;          // Default OFF — joystick axes as the board wires them
	prefs->v_contact_enabled = 1;     // Default ON — v-contact loopback admin chat (companion)
	prefs->v_battery_alert_mv = 0xFFFF; // Sentinel: derive from board auto-shutdown threshold
	/* Companion-only feature, and main_companion.cpp used to assign this by
	 * hand right after calling us — so no node ever ran without it.  It lives
	 * here now because a default listed only at one call site is invisible to
	 * every other caller of initNodePrefs(), which is the exact drift that
	 * zeroed probe_interval and cad_auto in the past.  Matches what
	 * loadPrefs()'s absent-field fallback and sanitizeNodePrefs() already use.
	 * 0 means disabled and is a legal stored value, so sanitize passes it
	 * through untouched — this default only applies to a fresh prefs struct. */
#ifdef CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS
	prefs->auto_shutdown_mv = CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS;
#else
	prefs->auto_shutdown_mv = 0;
#endif
}
