// Prefs byte codecs (helpers/PrefsCodec.cpp): both on-disk layouts pinned
// against an offset table written independently of the codec, from the byte
// maps (memory prefs-format.md; offset comments in the old stores).
#include "test.h"
#include "PrefsCodec.h"

#include <cmath>
#include <cstddef>
#include <cstring>

namespace {

struct Field { size_t file_off; size_t size; size_t struct_off; };
#define F(off, member) { off, sizeof(NodePrefs::member), offsetof(NodePrefs, member) }

// Companion /lfs/new_prefs, 272 bytes. Pads at 36-39 and 79.
const Field kCompanion[] = {
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
    F(171, tz_offset), F(172, leds_radio_mode), F(173, leds_hb_mode), F(174, wifi_enabled),
    F(175, wifi_ssid), F(208, wifi_pwd),
};

// Server /lfs/repeater/prefs, 311 bytes. Offset 120 is leds_disabled as a
// magic byte (checked separately); pads at 36-39, 79, 123, 127-151 (bridge),
// 153-155.
const Field kServer[] = {
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
    F(305, display_rotate), F(306, input_rotate), F(307, cad_base), F(308, tz_offset),
    F(309, leds_radio_mode), F(310, leds_hb_mode),
};
#undef F

const uint8_t *field(const NodePrefs &p, const Field &f) {
    return reinterpret_cast<const uint8_t *>(&p) + f.struct_off;
}
uint8_t *field(NodePrefs &p, const Field &f) {
    return reinterpret_cast<uint8_t *>(&p) + f.struct_off;
}

NodePrefs defaults() {
    NodePrefs p;
    initNodePrefs(&p);
    return p;
}

// Every serialized field different from its initNodePrefs() default, all in range.
NodePrefs sample() {
    NodePrefs p = defaults();
    p.airtime_factor = 3.5f;
    strcpy(p.node_name, "Node-Test");
    p.node_lat = 47.5; p.node_lon = 19.04;
    strcpy(p.password, "adminpw"); strcpy(p.guest_password, "guestpw");
    p.freq = 869.525f; p.sf = 9; p.cr = 6; p.bw = 125.0f; p.tx_power_dbm = 14;
    p.disable_fwd = 1; p.advert_interval = 30; p.flood_advert_interval = 12;
    p.rx_delay_base = 1.5f; p.tx_delay_factor = 0.7f; p.direct_tx_delay_factor = 0.4f;
    p.backoff_multiplier = 0.6f; p.allow_read_only = 1; p.multi_acks = 1;
    p.flood_max = 20; p.flood_max_unscoped = 10; p.flood_max_advert = 4;
    p.interference_threshold = 7; p.leds_disabled = 1; p.leds_radio_mode = LEDS_RADIO_RX;
    p.leds_hb_mode = LEDS_HB_UNREAD; p.powersaving_enabled = 1; p.gps_enabled = 1;
    p.gps_interval = 600; p.advert_loc_policy = ADVERT_LOC_SHARE; p.discovery_mod_timestamp = 123456;
    p.adc_multiplier = 1.25f; strcpy(p.owner_info, "owner info");
    p.rx_boost = 0; p.fem_rxgain = 0; p.rx_duty_cycle = 1;
    p._reserved_apc_enabled = 0x11; p._reserved_apc_margin = 0x22;
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
    p.v_contact_flags = 0x81; p.wifi_enabled = 0;
    strcpy(p.wifi_ssid, "MyNet"); strcpy(p.wifi_pwd, "secret");
    return p;
}

bool zeros(const uint8_t *b, size_t from, size_t to) {
    for (size_t i = from; i < to; i++) if (b[i] != 0) return false;
    return true;
}

template <size_t N>
void checkLayout(const uint8_t *buf, const NodePrefs &p, const Field (&table)[N]) {
    for (const Field &f : table) {
        if (std::memcmp(buf + f.file_off, field(p, f), f.size) != 0)
            throw std::runtime_error("field at file offset " + std::to_string(f.file_off));
    }
}

// What a decode of the first len bytes must give: the caller's defaults, with
// each field wholly inside len taken from the file.
template <size_t N>
NodePrefs expectShort(const NodePrefs &file_p, size_t len, const Field (&table)[N]) {
    NodePrefs e = defaults();
    for (const Field &f : table)
        if (f.file_off + f.size <= len) std::memcpy(field(e, f), field(file_p, f), f.size);
    return e;
}

}  // namespace

TEST(prefs_companion_layout, "UNIT-PREFS-001", "Companion prefs encode to the documented 272-byte layout") {
    NodePrefs p = sample();
    uint8_t buf[COMPANION_PREFS_SIZE + 8];
    std::memset(buf, 0xEE, sizeof(buf));
    CHECK(companionPrefsEncode(p, buf, sizeof(buf)) == COMPANION_PREFS_SIZE);
    checkLayout(buf, p, kCompanion);
    CHECK(zeros(buf, 36, 40));
    CHECK(buf[79] == 0);
    CHECK(buf[COMPANION_PREFS_SIZE] == 0xEE);
    CHECK(companionPrefsEncode(p, buf, COMPANION_PREFS_SIZE - 1) == 0);
}

TEST(prefs_companion_roundtrip, "UNIT-PREFS-002", "Companion prefs decode what they encode") {
    NodePrefs p = sample();
    uint8_t a[COMPANION_PREFS_SIZE], b[COMPANION_PREFS_SIZE];
    companionPrefsEncode(p, a, sizeof(a));
    NodePrefs q = defaults();
    CHECK(companionPrefsDecode(q, a, sizeof(a)));
    companionPrefsEncode(q, b, sizeof(b));
    CHECK(std::memcmp(a, b, sizeof(a)) == 0);
}

TEST(prefs_companion_short, "UNIT-PREFS-003", "Short companion files keep the caller's defaults past the end") {
    NodePrefs p = sample();
    uint8_t full[COMPANION_PREFS_SIZE];
    companionPrefsEncode(p, full, sizeof(full));
    // Every field boundary: each length the format has had ends on one.
    const size_t lens[] = { 91, 92, 93, 94, 95, 96, 127, 143, 144, 145, 146, 148, 150, 151,
                            152, 153, 155, 156, 157, 158, 159, 163, 166, 167, 168, 169, 170,
                            171, 172, 173, 174, 175, 208, 272 };
    for (size_t len : lens) {
        NodePrefs got = defaults();
        CHECK(companionPrefsDecode(got, full, len));
        NodePrefs want = expectShort(p, len, kCompanion);
        uint8_t g[COMPANION_PREFS_SIZE], w[COMPANION_PREFS_SIZE];
        companionPrefsEncode(got, g, sizeof(g));
        companionPrefsEncode(want, w, sizeof(w));
        if (std::memcmp(g, w, sizeof(g)) != 0)
            throw std::runtime_error("length " + std::to_string(len));
    }
}

TEST(prefs_companion_reject, "UNIT-PREFS-004", "Companion decode rejects short or foreign data and leaves prefs untouched") {
    NodePrefs p = sample();
    uint8_t buf[COMPANION_PREFS_SIZE];
    companionPrefsEncode(p, buf, sizeof(buf));
    NodePrefs got = defaults();
    uint8_t before[sizeof(NodePrefs)];
    std::memcpy(before, &got, sizeof(got));
    CHECK(!companionPrefsDecode(got, buf, 89));
    CHECK(std::memcmp(before, &got, sizeof(got)) == 0);
    buf[60] = 1;  // sf out of range: not our layout
    CHECK(!companionPrefsDecode(got, buf, sizeof(buf)));
    CHECK(std::memcmp(before, &got, sizeof(got)) == 0);
    companionPrefsEncode(p, buf, sizeof(buf));
    float bad = 5000.0f;
    std::memcpy(&buf[56], &bad, 4);
    CHECK(!companionPrefsDecode(got, buf, sizeof(buf)));
    CHECK(std::memcmp(before, &got, sizeof(got)) == 0);
}

TEST(prefs_companion_corrupt, "UNIT-PREFS-005", "Corrupt companion bytes fall back to the sanitizeNodePrefs defaults") {
    NodePrefs p = sample();
    uint8_t buf[COMPANION_PREFS_SIZE];
    companionPrefsEncode(p, buf, sizeof(buf));
    buf[150] = 5;    // rx_duty_cycle
    buf[151] = 9;    // meshtimesync
    buf[152] = 9;    // v_contact_enabled
    buf[155] = 7;    // cad_auto
    buf[156] = 100;  // cad_offset
    buf[157] = 3;    // probe_interval
    buf[158] = 200;  // cad_busycap
    float nan = NAN;
    std::memcpy(&buf[159], &nan, 4);  // adc_multiplier
    buf[171] = 50;   // tz_offset
    buf[172] = 9;    // leds_radio_mode
    NodePrefs got = defaults();
    CHECK(companionPrefsDecode(got, buf, sizeof(buf)));
    CHECK(got.rx_duty_cycle == 0);
    CHECK(got.meshtimesync == 0);
    CHECK(got.v_contact_enabled == 1);
    CHECK(got.cad_auto == 1);  // the initNodePrefs() default, not "off"
    CHECK(got.cad_offset == 0);
    CHECK(got.probe_interval == 10);
    CHECK(got.cad_busycap == 90);
    CHECK(got.adc_multiplier == 0.0f);
    CHECK(got.tz_offset == 0);
    CHECK(got.leds_radio_mode == 0);
}

TEST(prefs_server_layout, "UNIT-PREFS-006", "Server prefs encode to the documented 311-byte layout") {
    NodePrefs p = sample();
    uint8_t buf[SERVER_PREFS_SIZE + 8];
    std::memset(buf, 0xEE, sizeof(buf));
    CHECK(serverPrefsEncode(p, buf, sizeof(buf)) == SERVER_PREFS_SIZE);
    checkLayout(buf, p, kServer);
    CHECK(buf[120] == LEDS_PREF_OFF);
    CHECK(zeros(buf, 36, 40));
    CHECK(buf[79] == 0 && buf[123] == 0);
    CHECK(zeros(buf, 127, 152));
    CHECK(zeros(buf, 153, 156));
    CHECK(buf[SERVER_PREFS_SIZE] == 0xEE);
    p.leds_disabled = 0;
    serverPrefsEncode(p, buf, sizeof(buf));
    CHECK(buf[120] == LEDS_PREF_ON);
    CHECK(serverPrefsEncode(p, buf, SERVER_PREFS_SIZE - 1) == 0);
}

TEST(prefs_server_roundtrip, "UNIT-PREFS-007", "Server prefs decode what they encode") {
    NodePrefs p = sample();
    uint8_t a[SERVER_PREFS_SIZE], b[SERVER_PREFS_SIZE];
    serverPrefsEncode(p, a, sizeof(a));
    NodePrefs q = defaults();
    serverPrefsDecode(q, a, sizeof(a));
    serverPrefsEncode(q, b, sizeof(b));
    CHECK(std::memcmp(a, b, sizeof(a)) == 0);
}

TEST(prefs_server_short, "UNIT-PREFS-008", "Short server files keep the caller's values past the end") {
    NodePrefs p = sample();
    uint8_t full[SERVER_PREFS_SIZE];
    serverPrefsEncode(p, full, sizeof(full));
    // Every length the format has had.
    const size_t lens[] = { 290, 292, 294, 296, 297, 300, 301, 304, 305, 307, 308, 309, 311 };
    for (size_t len : lens) {
        NodePrefs got = defaults();
        serverPrefsDecode(got, full, len);
        NodePrefs want = expectShort(p, len, kServer);
        want.leds_disabled = p.leds_disabled;  // offset 120, inside every length
        uint8_t g[SERVER_PREFS_SIZE], w[SERVER_PREFS_SIZE];
        serverPrefsEncode(got, g, sizeof(g));
        serverPrefsEncode(want, w, sizeof(w));
        if (std::memcmp(g, w, sizeof(g)) != 0)
            throw std::runtime_error("length " + std::to_string(len));
    }
}

TEST(prefs_server_legacy, "UNIT-PREFS-009", "Server LED magic byte and implausible radio params") {
    NodePrefs p = sample();
    uint8_t buf[SERVER_PREFS_SIZE];
    serverPrefsEncode(p, buf, sizeof(buf));
    const uint8_t legacy_agc[] = { 0, 15, 0xA0, 0xFF };  // an old AGC interval means "on"
    for (uint8_t v : legacy_agc) {
        buf[120] = v;
        NodePrefs got = defaults();
        serverPrefsDecode(got, buf, sizeof(buf));
        CHECK(got.leds_disabled == 0);
    }
    buf[120] = LEDS_PREF_OFF;
    buf[112] = 2;  // sf out of range
    NodePrefs got = defaults();
    serverPrefsDecode(got, buf, sizeof(buf));
    CHECK(got.leds_disabled == 1);
    CHECK(got.sf == mesh::LoRaConfig::SPREADING_FACTOR);
    CHECK(got.freq == mesh::LoRaConfig::FREQ_MHZ);
    CHECK(got.bw == mesh::LoRaConfig::BANDWIDTH_KHZ);
    CHECK(got.cr == mesh::LoRaConfig::CODING_RATE);
    CHECK(got.tx_power_dbm == mesh::LoRaConfig::TX_POWER_DBM);
    CHECK(std::strcmp(got.node_name, "Node-Test") == 0);  // the rest still loads
    CHECK(got.flood_max == 20);
}

TEST(prefs_gps_interval, "UNIT-PREFS-010", "GPS interval: one clamp, and a corrupt value is not turned into always-on") {
    CHECK(clampGpsInterval(0) == 0);                      // always on stays a choice
    CHECK(clampGpsInterval(5) == GPS_INTERVAL_MIN_SEC);
    CHECK(clampGpsInterval(300) == 300);
    CHECK(clampGpsInterval(GPS_INTERVAL_MAX_SEC + 1) == GPS_INTERVAL_MAX_SEC);
    NodePrefs p; initNodePrefs(&p);
    p.gps_interval = 0xFFFFFFFFu;                         // garbage in a prefs file
    sanitizeNodePrefs(&p);
    CHECK(p.gps_interval == GPS_INTERVAL_MAX_SEC);
}
