// The two legacy binary prefs layouts as offset tables, written independently of
// helpers/PrefsCodec.cpp from the byte maps (memory prefs-format.md), a sample with
// every field off its default, and file bytes built from the tables.
#pragma once
#include "test.h"
#include "PrefsCodec.h"

#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

namespace prefs_layout {

struct Field { size_t file_off; size_t size; size_t struct_off; };
#define F(off, member) { off, sizeof(NodePrefs::member), offsetof(NodePrefs, member) }

// Companion /lfs/new_prefs, 272 bytes. Pads at 36-39 and 79; tz_offset at 171.
inline const Field kCompanion[] = {
    F(0, airtime_factor), F(4, node_name), F(40, node_lat), F(48, node_lon), F(56, freq),
    F(60, sf), F(61, cr), F(62, client_repeat), F(63, manual_add_contacts), F(64, bw),
    F(68, tx_power_dbm), F(69, telemetry_mode_base), F(70, telemetry_mode_loc),
    F(71, telemetry_mode_env), F(72, rx_delay_base), F(76, advert_loc_policy),
    F(77, multi_acks), F(78, path_hash_mode), F(80, ble_pin), F(84, buzzer_quiet),
    F(85, gps_enabled), F(86, gps_interval), F(90, autoadd_config), F(91, autoadd_max_hops),
    F(92, rx_boost), F(93, leds_disabled), F(94, _reserved_apc_enabled),
    F(95, _reserved_apc_margin), { 96, 31, offsetof(NodePrefs, default_scope_name) },
    F(127, default_scope_key), F(143, ble_disabled), F(144, display_brightness),
    F(145, wake_on_msg), F(146, screen_off_secs), F(148, auto_shutdown_mv),
    F(150, rx_duty_cycle), F(151, meshtimesync), F(152, v_contact_enabled),
    F(153, v_battery_alert_mv), F(155, cad_auto), F(156, cad_offset), F(157, probe_interval),
    F(158, cad_busycap), F(159, adc_multiplier), F(163, extra_sf), F(166, v_contact_flags),
    F(167, fem_rxgain), F(168, display_rotate), F(169, input_rotate), F(170, cad_base),
    F(172, leds_radio_mode), F(173, leds_hb_mode), F(174, wifi_enabled),
    F(175, wifi_ssid), F(208, wifi_pwd),
};

// Server /lfs/repeater/prefs, 311 bytes. Offset 120 is leds_disabled as a magic
// byte; pads at 36-39, 79, 123, 127-151 (bridge), 153-155; tz_offset at 308.
inline const Field kServer[] = {
    F(0, airtime_factor), F(4, node_name), F(40, node_lat), F(48, node_lon), F(56, password),
    F(72, freq), F(76, tx_power_dbm), F(77, disable_fwd), F(78, advert_interval),
    F(80, rx_delay_base), F(84, tx_delay_factor), F(88, guest_password),
    F(104, direct_tx_delay_factor), F(108, backoff_multiplier), F(112, sf), F(113, cr),
    F(114, allow_read_only), F(115, multi_acks), F(116, bw), F(121, path_hash_mode),
    F(122, loop_detect), F(124, flood_max), F(125, flood_advert_interval),
    F(126, interference_threshold), F(152, powersaving_enabled), F(156, gps_enabled),
    F(157, gps_interval), F(161, advert_loc_policy), F(162, discovery_mod_timestamp),
    F(166, adc_multiplier), F(170, owner_info), F(290, rx_boost), F(291, rx_duty_cycle),
    F(292, _reserved_apc_enabled), F(293, _reserved_apc_margin), F(294, flood_max_unscoped),
    F(295, flood_max_advert), F(296, meshtimesync), F(297, cad_auto), F(298, cad_offset),
    F(299, probe_interval), F(300, cad_busycap), F(301, extra_sf), F(304, fem_rxgain),
    F(305, display_rotate), F(306, input_rotate), F(307, cad_base),
    F(309, leds_radio_mode), F(310, leds_hb_mode),
};
#undef F

// tz_offset is float hours in NodePrefs but one signed whole-hours byte in both files.
const size_t kCompanionTz = 171;
const size_t kServerTz = 308;
const size_t kServerLeds = 120;

inline const uint8_t *field(const NodePrefs &p, const Field &f) {
    return reinterpret_cast<const uint8_t *>(&p) + f.struct_off;
}
inline uint8_t *field(NodePrefs &p, const Field &f) {
    return reinterpret_cast<uint8_t *>(&p) + f.struct_off;
}

inline NodePrefs defaults() { NodePrefs p; initNodePrefs(&p); return p; }

// Every serialized field different from its initNodePrefs() default, all in range.
inline NodePrefs sample() {
    NodePrefs p = defaults();
    p.airtime_factor = 3.5f; strcpy(p.node_name, "Node-Test");
    p.node_lat = 47.5; p.node_lon = 19.04;
    strcpy(p.password, "adminpw"); strcpy(p.guest_password, "guestpw");
    p.freq = 869.525f; p.sf = 9; p.cr = 6; p.bw = 125.0f; p.tx_power_dbm = 14;
    p.disable_fwd = 1; p.advert_interval = 30; p.flood_advert_interval = 12;
    p.rx_delay_base = 1.5f; p.tx_delay_factor = 0.7f; p.direct_tx_delay_factor = 0.4f;
    p.backoff_multiplier = 0.6f; p.allow_read_only = 1; p.multi_acks = 1;
    p.flood_max = 20; p.flood_max_unscoped = 10; p.flood_max_advert = 4;
    p.interference_threshold = 7; p.leds_disabled = 1; p.leds_radio_mode = LEDS_RADIO_RX;
    p.leds_hb_mode = LEDS_HB_UNREAD; p.led_brightness = 42; p.powersaving_enabled = 1; p.gps_enabled = 1;
    p.gps_interval = 600; p.advert_loc_policy = ADVERT_LOC_SHARE; p.discovery_mod_timestamp = 123456;
    p.adc_multiplier = 1.25f; strcpy(p.owner_info, "owner info");
    p.rx_boost = 0; p.fem_rxgain = 0; p.rx_duty_cycle = 1;
    p.meshtimesync = 1; p.cad_auto = 0; p.cad_offset = -3; p.probe_interval = 30; p.cad_busycap = 40;
    p.extra_sf[0] = 10; p.extra_sf[1] = 11; p.extra_sf[2] = 0;
    p.display_rotate = 1; p.input_rotate = 1; p.tz_offset = -5; p.cad_base = 21;
    p.manual_add_contacts = 1; p.telemetry_mode_base = 1; p.telemetry_mode_loc = 2;
    p.telemetry_mode_env = 1; p.ble_pin = 123456; p.buzzer_quiet = 1; p.autoadd_config = 0x1E;
    p.client_repeat = 1; p.path_hash_mode = 2; p.autoadd_max_hops = 5; p.loop_detect = 3;
    strcpy(p.default_scope_name, "#hu");
    for (int i = 0; i < 16; i++) p.default_scope_key[i] = (uint8_t)(i + 1);
    p.ble_disabled = 1; p.display_brightness = 50; p.wake_on_msg = 0; p.screen_off_secs = 60;
    p.auto_shutdown_mv = 3300; p.v_contact_enabled = 0; p.v_battery_alert_mv = 3500;
    p.v_contact_flags = 0x81; p.wifi_enabled = 0; p.offgrid_home_freq = 869.618f;
    strcpy(p.wifi_ssid, "MyNet"); strcpy(p.wifi_pwd, "secret");
    return p;
}

template <size_t N>
std::vector<uint8_t> fileBytes(const NodePrefs &p, const Field (&table)[N], size_t size, size_t tz_off) {
    std::vector<uint8_t> b(size, 0);
    for (const Field &f : table) std::memcpy(b.data() + f.file_off, field(p, f), f.size);
    b[tz_off] = (uint8_t)(int8_t)p.tz_offset;
    return b;
}
inline std::vector<uint8_t> companionFile(const NodePrefs &p) {
    return fileBytes(p, kCompanion, COMPANION_PREFS_SIZE, kCompanionTz);
}
inline std::vector<uint8_t> serverFile(const NodePrefs &p) {
    std::vector<uint8_t> b = fileBytes(p, kServer, SERVER_PREFS_SIZE, kServerTz);
    b[kServerLeds] = p.leds_disabled ? LEDS_PREF_OFF : LEDS_PREF_ON;
    return b;
}

// Throws naming the first field of the layout where a and b differ.
template <size_t N>
void checkFields(const NodePrefs &a, const NodePrefs &b, const Field (&table)[N]) {
    for (const Field &f : table)
        if (std::memcmp(field(a, f), field(b, f), f.size) != 0)
            throw std::runtime_error("field at file offset " + std::to_string(f.file_off));
    CHECK(a.tz_offset == b.tz_offset);
    CHECK(a.leds_disabled == b.leds_disabled);
}
inline void checkCompanion(const NodePrefs &a, const NodePrefs &b) { checkFields(a, b, kCompanion); }
inline void checkServer(const NodePrefs &a, const NodePrefs &b) { checkFields(a, b, kServer); }

}  // namespace prefs_layout
