/*
 * SPDX-License-Identifier: MIT
 * PrefsCodec - byte codecs for the two binary prefs files. See PrefsCodec.h.
 */

#include "PrefsCodec.h"
#include <string.h>

namespace {

/* Sequential writer / reader over a byte buffer. Rd::get() mirrors fs_read()
 * on a short file, as the server store used to read it: a field that straddles
 * the end gets the bytes that exist. Rd::take() reads whole fields only. Past
 * the end, both leave the field as it was. */
struct Wr {
	uint8_t *b;
	size_t off;
	void put(const void *src, size_t n) { memcpy(b + off, src, n); off += n; }
	void zeros(size_t n) { memset(b + off, 0, n); off += n; }
};

struct Rd {
	const uint8_t *b;
	size_t len;
	size_t off;
	void get(void *dst, size_t n) {
		size_t k = off < len ? (len - off < n ? len - off : n) : 0;
		memcpy(dst, b + off, k);
		off += n;
	}
	void skip(size_t n) { off += n; }
	/* Whole fields only: one that does not fit keeps its value. */
	void take(void *dst, size_t n) {
		if (off + n <= len) {
			memcpy(dst, b + off, n);
		}
		off += n;
	}
};

}  // namespace

/* ── Companion: /lfs/new_prefs ─────────────────────────────────────── */

size_t companionPrefsEncode(const NodePrefs &p, uint8_t *buf, size_t cap)
{
	if (cap < COMPANION_PREFS_SIZE) {
		return 0;
	}
	Wr w = { buf, 0 };

	/* 0-91: the layout upstream's new_prefs had */
	w.put(&p.airtime_factor, 4);
	w.put(p.node_name, 32);
	w.zeros(4);
	w.put(&p.node_lat, 8);
	w.put(&p.node_lon, 8);
	w.put(&p.freq, 4);
	w.put(&p.sf, 1);
	w.put(&p.cr, 1);
	w.put(&p.client_repeat, 1);
	w.put(&p.manual_add_contacts, 1);
	w.put(&p.bw, 4);
	w.put(&p.tx_power_dbm, 1);
	w.put(&p.telemetry_mode_base, 1);
	w.put(&p.telemetry_mode_loc, 1);
	w.put(&p.telemetry_mode_env, 1);
	w.put(&p.rx_delay_base, 4);
	w.put(&p.advert_loc_policy, 1);
	w.put(&p.multi_acks, 1);
	w.put(&p.path_hash_mode, 1);
	w.zeros(1);
	w.put(&p.ble_pin, 4);
	w.put(&p.buzzer_quiet, 1);
	w.put(&p.gps_enabled, 1);
	w.put(&p.gps_interval, 4);
	w.put(&p.autoadd_config, 1);
	w.put(&p.autoadd_max_hops, 1);
	/* 92-: ZephCore. Upstream put default_scope_name at 93. */
	w.put(&p.rx_boost, 1);
	w.put(&p.leds_disabled, 1);
	w.put(&p._reserved_apc_enabled, 1);   /* 94-95: APC, removed in 1.16.6 */
	w.put(&p._reserved_apc_margin, 1);
	w.put(p.default_scope_name, 31);
	w.put(p.default_scope_key, 16);
	w.put(&p.ble_disabled, 1);
	w.put(&p.display_brightness, 1);
	w.put(&p.wake_on_msg, 1);
	w.put(&p.screen_off_secs, 2);
	w.put(&p.auto_shutdown_mv, 2);
	w.put(&p.rx_duty_cycle, 1);
	w.put(&p.meshtimesync, 1);
	w.put(&p.v_contact_enabled, 1);
	w.put(&p.v_battery_alert_mv, 2);
	w.put(&p.cad_auto, 1);
	w.put(&p.cad_offset, 1);
	w.put(&p.probe_interval, 1);
	w.put(&p.cad_busycap, 1);
	w.put(&p.adc_multiplier, 4);
	w.put(p.extra_sf, EXTRA_SF_MAX);
	w.put(&p.v_contact_flags, 1);
	w.put(&p.fem_rxgain, 1);
	w.put(&p.display_rotate, 1);
	w.put(&p.input_rotate, 1);
	w.put(&p.cad_base, 1);
	w.put(&p.tz_offset, 1);
	w.put(&p.leds_radio_mode, 1);
	w.put(&p.leds_hb_mode, 1);
	w.put(&p.wifi_enabled, 1);
	w.put(p.wifi_ssid, sizeof(p.wifi_ssid));
	w.put(p.wifi_pwd, sizeof(p.wifi_pwd));
	return w.off;
}

bool companionPrefsDecode(NodePrefs &prefs, const uint8_t *buf, size_t len)
{
	/* gps_interval ends at 90: anything shorter is not a prefs file */
	if (len < 90) {
		return false;
	}
	/* Committed only if the radio preset is plausible: a file in some other
	 * layout must not become our radio settings. */
	NodePrefs p = prefs;
	Rd r = { buf, len, 0 };

	r.take(&p.airtime_factor, 4);
	r.take(p.node_name, 32);
	r.skip(4);
	r.take(&p.node_lat, 8);
	r.take(&p.node_lon, 8);
	r.take(&p.freq, 4);
	r.take(&p.sf, 1);
	r.take(&p.cr, 1);
	r.take(&p.client_repeat, 1);
	r.take(&p.manual_add_contacts, 1);
	r.take(&p.bw, 4);
	if (p.freq < ZC_RADIO_FREQ_MIN_MHZ || p.freq > ZC_RADIO_FREQ_MAX_MHZ ||
	    p.sf < 5 || p.sf > 12 ||
	    p.bw < ZC_RADIO_BW_MIN_KHZ || p.bw > ZC_RADIO_BW_MAX_KHZ) {
		return false;
	}
	r.take(&p.tx_power_dbm, 1);
	r.take(&p.telemetry_mode_base, 1);
	r.take(&p.telemetry_mode_loc, 1);
	r.take(&p.telemetry_mode_env, 1);
	r.take(&p.rx_delay_base, 4);
	r.take(&p.advert_loc_policy, 1);
	r.take(&p.multi_acks, 1);
	r.take(&p.path_hash_mode, 1);
	r.skip(1);
	r.take(&p.ble_pin, 4);
	r.take(&p.buzzer_quiet, 1);
	r.take(&p.gps_enabled, 1);
	r.take(&p.gps_interval, 4);
	r.take(&p.autoadd_config, 1);
	r.take(&p.autoadd_max_hops, 1);
	/* Each field below is newer than some deployed files: one past the end
	 * keeps the caller's initNodePrefs() default. */
	r.take(&p.rx_boost, 1);
	r.take(&p.leds_disabled, 1);
	r.take(&p._reserved_apc_enabled, 1);
	r.take(&p._reserved_apc_margin, 1);
	r.take(p.default_scope_name, 31);
	r.take(p.default_scope_key, 16);
	r.take(&p.ble_disabled, 1);
	r.take(&p.display_brightness, 1);
	r.take(&p.wake_on_msg, 1);
	r.take(&p.screen_off_secs, 2);
	r.take(&p.auto_shutdown_mv, 2);
	r.take(&p.rx_duty_cycle, 1);
	r.take(&p.meshtimesync, 1);
	r.take(&p.v_contact_enabled, 1);
	r.take(&p.v_battery_alert_mv, 2);
	r.take(&p.cad_auto, 1);
	r.take(&p.cad_offset, 1);
	r.take(&p.probe_interval, 1);
	r.take(&p.cad_busycap, 1);
	r.take(&p.adc_multiplier, 4);
	r.take(p.extra_sf, EXTRA_SF_MAX);
	r.take(&p.v_contact_flags, 1);
	r.take(&p.fem_rxgain, 1);
	r.take(&p.display_rotate, 1);
	r.take(&p.input_rotate, 1);
	/* cad_base 0 = "no base recorded": setCadParams() then leaves the stored
	 * offset alone. */
	r.take(&p.cad_base, 1);
	r.take(&p.tz_offset, 1);
	r.take(&p.leds_radio_mode, 1);
	r.take(&p.leds_hb_mode, 1);
	r.take(&p.wifi_enabled, 1);
	r.take(p.wifi_ssid, sizeof(p.wifi_ssid));
	r.take(p.wifi_pwd, sizeof(p.wifi_pwd));

	/* The one validator: bounds, NaNs, and the unterminated char fields */
	sanitizeNodePrefs(&p);
	prefs = p;
	return true;
}

/* ── Server roles: /lfs/repeater/prefs ─────────────────────────────── */

size_t serverPrefsEncode(const NodePrefs &prefs, uint8_t *buf, size_t cap)
{
	if (cap < SERVER_PREFS_SIZE) {
		return 0;
	}
	Wr w = { buf, 0 };

	/* Same order as Arduino CommonCLI up to offset 290 */
	w.put(&prefs.airtime_factor, sizeof(prefs.airtime_factor));
	w.put(&prefs.node_name, sizeof(prefs.node_name));
	w.zeros(4);
	w.put(&prefs.node_lat, sizeof(prefs.node_lat));
	w.put(&prefs.node_lon, sizeof(prefs.node_lon));
	w.put(&prefs.password, sizeof(prefs.password));
	w.put(&prefs.freq, sizeof(prefs.freq));
	w.put(&prefs.tx_power_dbm, sizeof(prefs.tx_power_dbm));
	w.put(&prefs.disable_fwd, sizeof(prefs.disable_fwd));
	w.put(&prefs.advert_interval, sizeof(prefs.advert_interval));
	w.zeros(1);
	w.put(&prefs.rx_delay_base, sizeof(prefs.rx_delay_base));
	w.put(&prefs.tx_delay_factor, sizeof(prefs.tx_delay_factor));
	w.put(&prefs.guest_password, sizeof(prefs.guest_password));
	w.put(&prefs.direct_tx_delay_factor, sizeof(prefs.direct_tx_delay_factor));
	w.put(&prefs.backoff_multiplier, sizeof(prefs.backoff_multiplier));
	w.put(&prefs.sf, sizeof(prefs.sf));
	w.put(&prefs.cr, sizeof(prefs.cr));
	w.put(&prefs.allow_read_only, sizeof(prefs.allow_read_only));
	w.put(&prefs.multi_acks, sizeof(prefs.multi_acks));
	w.put(&prefs.bw, sizeof(prefs.bw));
	/* 120: leds_disabled, magic-encoded (was agc_reset_interval). */
	{
		uint8_t leds_byte = prefs.leds_disabled ? LEDS_PREF_OFF : LEDS_PREF_ON;
		w.put(&leds_byte, sizeof(leds_byte));
	}
	w.put(&prefs.path_hash_mode, sizeof(prefs.path_hash_mode));
	w.put(&prefs.loop_detect, sizeof(prefs.loop_detect));
	w.zeros(1);
	w.put(&prefs.flood_max, sizeof(prefs.flood_max));
	w.put(&prefs.flood_advert_interval, sizeof(prefs.flood_advert_interval));
	w.put(&prefs.interference_threshold, sizeof(prefs.interference_threshold));
	w.zeros(25);  // bridge settings
	w.put(&prefs.powersaving_enabled, sizeof(prefs.powersaving_enabled));
	w.zeros(3);
	w.put(&prefs.gps_enabled, sizeof(prefs.gps_enabled));
	w.put(&prefs.gps_interval, sizeof(prefs.gps_interval));
	w.put(&prefs.advert_loc_policy, sizeof(prefs.advert_loc_policy));
	w.put(&prefs.discovery_mod_timestamp, sizeof(prefs.discovery_mod_timestamp));
	w.put(&prefs.adc_multiplier, sizeof(prefs.adc_multiplier));
	w.put(prefs.owner_info, sizeof(prefs.owner_info));
	/* ZephCore extensions */
	w.put(&prefs.rx_boost, sizeof(prefs.rx_boost));
	w.put(&prefs.rx_duty_cycle, sizeof(prefs.rx_duty_cycle));
	/* RESERVED — formerly apc_enabled / apc_margin (removed in 1.16.6).
	 * Written back unchanged to hold the layout. */
	w.put(&prefs._reserved_apc_enabled, sizeof(prefs._reserved_apc_enabled));
	w.put(&prefs._reserved_apc_margin, sizeof(prefs._reserved_apc_margin));
	/* Flood hop-ceiling extensions (offsets 294-295) */
	w.put(&prefs.flood_max_unscoped, sizeof(prefs.flood_max_unscoped));
	w.put(&prefs.flood_max_advert, sizeof(prefs.flood_max_advert));
	/* Mesh time sync on/off (offset 296) */
	w.put(&prefs.meshtimesync, sizeof(prefs.meshtimesync));
	/* Adaptive CAD (offsets 297-300) */
	w.put(&prefs.cad_auto, sizeof(prefs.cad_auto));
	w.put(&prefs.cad_offset, sizeof(prefs.cad_offset));
	w.put(&prefs.probe_interval, sizeof(prefs.probe_interval));
	w.put(&prefs.cad_busycap, sizeof(prefs.cad_busycap));
	/* LR2021 side-detector SFs (offsets 301-303) */
	w.put(prefs.extra_sf, sizeof(prefs.extra_sf));
	/* External FEM RX gain (offset 304) */
	w.put(&prefs.fem_rxgain, sizeof(prefs.fem_rxgain));
	/* Mounting orientation (offsets 305-306) */
	w.put(&prefs.display_rotate, sizeof(prefs.display_rotate));
	w.put(&prefs.input_rotate, sizeof(prefs.input_rotate));
	/* Family base detPeak cad_offset was learned against (offset 307) */
	w.put(&prefs.cad_base, sizeof(prefs.cad_base));
	/* Display timezone offset (offset 308) — signed whole hours from UTC,
	 * applied only when formatting the on-device clock */
	w.put(&prefs.tz_offset, sizeof(prefs.tz_offset));
	/* LED activity/heartbeat modes (offsets 309-310) */
	w.put(&prefs.leds_radio_mode, sizeof(prefs.leds_radio_mode));
	w.put(&prefs.leds_hb_mode, sizeof(prefs.leds_hb_mode));
	return w.off;
}

void serverPrefsDecode(NodePrefs &prefs, const uint8_t *buf, size_t len)
{
	Rd r = { buf, len, 0 };

	/* Same order as Arduino CommonCLI up to offset 290 */
	r.get(&prefs.airtime_factor, sizeof(prefs.airtime_factor));
	r.get(&prefs.node_name, sizeof(prefs.node_name));
	r.skip(4);
	r.get(&prefs.node_lat, sizeof(prefs.node_lat));
	r.get(&prefs.node_lon, sizeof(prefs.node_lon));
	r.get(&prefs.password, sizeof(prefs.password));
	r.get(&prefs.freq, sizeof(prefs.freq));
	r.get(&prefs.tx_power_dbm, sizeof(prefs.tx_power_dbm));
	r.get(&prefs.disable_fwd, sizeof(prefs.disable_fwd));
	r.get(&prefs.advert_interval, sizeof(prefs.advert_interval));
	r.skip(1);
	r.get(&prefs.rx_delay_base, sizeof(prefs.rx_delay_base));
	r.get(&prefs.tx_delay_factor, sizeof(prefs.tx_delay_factor));
	r.get(&prefs.guest_password, sizeof(prefs.guest_password));
	r.get(&prefs.direct_tx_delay_factor, sizeof(prefs.direct_tx_delay_factor));
	r.get(&prefs.backoff_multiplier, sizeof(prefs.backoff_multiplier));
	r.get(&prefs.sf, sizeof(prefs.sf));
	r.get(&prefs.cr, sizeof(prefs.cr));
	r.get(&prefs.allow_read_only, sizeof(prefs.allow_read_only));
	r.get(&prefs.multi_acks, sizeof(prefs.multi_acks));
	r.get(&prefs.bw, sizeof(prefs.bw));
	/* 120: leds_disabled, magic-encoded. Formerly agc_reset_interval — see the
	 * LEDS_PREF_* comment in NodePrefs.h for why this is not a bare 0/1.
	 * leds_byte stays 0 (→ LEDs on) if the file is short. */
	uint8_t leds_byte = 0;
	r.get(&leds_byte, sizeof(leds_byte));
	r.get(&prefs.path_hash_mode, sizeof(prefs.path_hash_mode));
	r.get(&prefs.loop_detect, sizeof(prefs.loop_detect));
	r.skip(1);
	r.get(&prefs.flood_max, sizeof(prefs.flood_max));
	r.get(&prefs.flood_advert_interval, sizeof(prefs.flood_advert_interval));
	r.get(&prefs.interference_threshold, sizeof(prefs.interference_threshold));
	r.skip(25);  // bridge settings
	r.get(&prefs.powersaving_enabled, sizeof(prefs.powersaving_enabled));
	r.skip(3);
	r.get(&prefs.gps_enabled, sizeof(prefs.gps_enabled));
	r.get(&prefs.gps_interval, sizeof(prefs.gps_interval));
	r.get(&prefs.advert_loc_policy, sizeof(prefs.advert_loc_policy));
	r.get(&prefs.discovery_mod_timestamp, sizeof(prefs.discovery_mod_timestamp));
	r.get(&prefs.adc_multiplier, sizeof(prefs.adc_multiplier));
	r.get(prefs.owner_info, sizeof(prefs.owner_info));
	/* ZephCore extensions — absent in old files; a field past the end keeps
	 * the caller's initNodePrefs() default. */
	r.get(&prefs.rx_boost, sizeof(prefs.rx_boost));
	r.get(&prefs.rx_duty_cycle, sizeof(prefs.rx_duty_cycle));
	/* RESERVED — formerly apc_enabled / apc_margin (APC, removed in 1.16.6).
	 * Still consumed so the fields after them stay at their stored offsets. */
	r.get(&prefs._reserved_apc_enabled, sizeof(prefs._reserved_apc_enabled));
	r.get(&prefs._reserved_apc_margin, sizeof(prefs._reserved_apc_margin));
	r.get(&prefs.flood_max_unscoped, sizeof(prefs.flood_max_unscoped));
	r.get(&prefs.flood_max_advert, sizeof(prefs.flood_max_advert));
	r.get(&prefs.meshtimesync, sizeof(prefs.meshtimesync));
	r.get(&prefs.cad_auto, sizeof(prefs.cad_auto));
	r.get(&prefs.cad_offset, sizeof(prefs.cad_offset));
	r.get(&prefs.probe_interval, sizeof(prefs.probe_interval));
	r.get(&prefs.cad_busycap, sizeof(prefs.cad_busycap));
	r.get(prefs.extra_sf, sizeof(prefs.extra_sf));
	r.get(&prefs.fem_rxgain, sizeof(prefs.fem_rxgain));
	r.get(&prefs.display_rotate, sizeof(prefs.display_rotate));
	r.get(&prefs.input_rotate, sizeof(prefs.input_rotate));
	/* cad_base 0 = "no base recorded": setCadParams() then leaves the stored
	 * offset alone, since a node upgrading across a table change cannot know
	 * which base its offset came from. */
	r.get(&prefs.cad_base, sizeof(prefs.cad_base));
	r.get(&prefs.tz_offset, sizeof(prefs.tz_offset));
	r.get(&prefs.leds_radio_mode, sizeof(prefs.leds_radio_mode));
	r.get(&prefs.leds_hb_mode, sizeof(prefs.leds_hb_mode));

	/* Only the explicit "off" magic disables LEDs; a legacy AGC interval or an
	 * unwritten byte both mean "on". */
	prefs.leds_disabled = (leds_byte == LEDS_PREF_OFF) ? 1 : 0;

	if (prefs.freq < ZC_RADIO_FREQ_MIN_MHZ || prefs.freq > ZC_RADIO_FREQ_MAX_MHZ ||
	    prefs.sf < 5 || prefs.sf > 12 ||
	    prefs.bw < ZC_RADIO_BW_MIN_KHZ || prefs.bw > ZC_RADIO_BW_MAX_KHZ) {
		prefs.freq = mesh::LoRaConfig::FREQ_MHZ;
		prefs.bw = mesh::LoRaConfig::BANDWIDTH_KHZ;
		prefs.sf = mesh::LoRaConfig::SPREADING_FACTOR;
		prefs.cr = mesh::LoRaConfig::CODING_RATE;
		prefs.tx_power_dbm = mesh::LoRaConfig::TX_POWER_DBM;
	}
	/* Bounds, NaNs, and the char fields, which the file stores without
	 * terminators. */
	sanitizeNodePrefs(&prefs);
}
