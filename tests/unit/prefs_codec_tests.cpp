// Legacy binary prefs decoders (helpers/PrefsCodec.cpp), read only when a node
// migrates to prefs.json. Files are built from the independent offset tables in
// support/prefs_layout.h.
#include "test.h"
#include "prefs_layout.h"

#include <cmath>

using namespace prefs_layout;

namespace {

// sample() plus the two reserved APC bytes, which only the binary files carry.
NodePrefs codecSample() {
    NodePrefs p = sample();
    p._reserved_apc_enabled = 0x11; p._reserved_apc_margin = 0x22;
    return p;
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

TEST(prefs_companion_layout, "UNIT-PREFS-001", "Companion decode reads every field at its documented offset and skips the pads") {
    NodePrefs p = codecSample();
    std::vector<uint8_t> buf = companionFile(p);
    std::memset(&buf[36], 0xEE, 4);
    buf[79] = 0xEE;
    NodePrefs got = defaults();
    CHECK(companionPrefsDecode(got, buf.data(), buf.size()));
    checkCompanion(got, p);
}

TEST(prefs_companion_short, "UNIT-PREFS-003", "Short companion files keep the caller's defaults past the end") {
    NodePrefs p = codecSample();
    std::vector<uint8_t> full = companionFile(p);
    // Every field boundary: each length the format has had ends on one.
    const size_t lens[] = { 91, 92, 93, 94, 95, 96, 127, 143, 144, 145, 146, 148, 150, 151,
                            152, 153, 155, 156, 157, 158, 159, 163, 166, 167, 168, 169, 170,
                            171, 172, 173, 174, 175, 208, 272 };
    for (size_t len : lens) {
        NodePrefs got = defaults();
        CHECK(companionPrefsDecode(got, full.data(), len));
        NodePrefs want = expectShort(p, len, kCompanion);
        if (len > kCompanionTz) want.tz_offset = p.tz_offset;
        try { checkCompanion(got, want); }
        catch (const std::exception &e) { throw std::runtime_error("length " + std::to_string(len) + ": " + e.what()); }
    }
}

TEST(prefs_companion_reject, "UNIT-PREFS-004", "Companion decode rejects short or foreign data and leaves prefs untouched") {
    std::vector<uint8_t> buf = companionFile(codecSample());
    NodePrefs got = defaults();
    uint8_t before[sizeof(NodePrefs)];
    std::memcpy(before, &got, sizeof(got));
    CHECK(!companionPrefsDecode(got, buf.data(), 89));
    CHECK(std::memcmp(before, &got, sizeof(got)) == 0);
    buf[60] = 1;  // sf out of range: not our layout
    CHECK(!companionPrefsDecode(got, buf.data(), buf.size()));
    CHECK(std::memcmp(before, &got, sizeof(got)) == 0);
    buf = companionFile(codecSample());
    float bad = 5000.0f;
    std::memcpy(&buf[56], &bad, 4);
    CHECK(!companionPrefsDecode(got, buf.data(), buf.size()));
    CHECK(std::memcmp(before, &got, sizeof(got)) == 0);
}

TEST(prefs_companion_corrupt, "UNIT-PREFS-005", "Corrupt companion bytes fall back to the sanitizeNodePrefs defaults") {
    std::vector<uint8_t> buf = companionFile(codecSample());
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
    CHECK(companionPrefsDecode(got, buf.data(), buf.size()));
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

TEST(prefs_server_layout, "UNIT-PREFS-006", "Server decode reads every field at its documented offset and skips the pads") {
    NodePrefs p = codecSample();
    std::vector<uint8_t> buf = serverFile(p);
    std::memset(&buf[36], 0xEE, 4);
    buf[79] = buf[123] = 0xEE;
    std::memset(&buf[127], 0xEE, 25);
    std::memset(&buf[153], 0xEE, 3);
    NodePrefs got = defaults();
    serverPrefsDecode(got, buf.data(), buf.size());
    checkServer(got, p);
}

TEST(prefs_server_short, "UNIT-PREFS-008", "Short server files keep the caller's values past the end") {
    NodePrefs p = codecSample();
    std::vector<uint8_t> full = serverFile(p);
    // Every length the format has had.
    const size_t lens[] = { 290, 292, 294, 296, 297, 300, 301, 304, 305, 307, 308, 309, 311 };
    for (size_t len : lens) {
        NodePrefs got = defaults();
        serverPrefsDecode(got, full.data(), len);
        NodePrefs want = expectShort(p, len, kServer);
        want.leds_disabled = p.leds_disabled;  // offset 120, inside every length
        if (len > kServerTz) want.tz_offset = p.tz_offset;
        try { checkServer(got, want); }
        catch (const std::exception &e) { throw std::runtime_error("length " + std::to_string(len) + ": " + e.what()); }
    }
}

TEST(prefs_server_legacy, "UNIT-PREFS-009", "Server LED magic byte and implausible radio params") {
    std::vector<uint8_t> buf = serverFile(codecSample());
    const uint8_t legacy_agc[] = { 0, 15, 0xA0, 0xFF };  // an old AGC interval means "on"
    for (uint8_t v : legacy_agc) {
        buf[kServerLeds] = v;
        NodePrefs got = defaults();
        serverPrefsDecode(got, buf.data(), buf.size());
        CHECK(got.leds_disabled == 0);
    }
    buf[kServerLeds] = LEDS_PREF_OFF;
    buf[112] = 2;  // sf out of range
    NodePrefs got = defaults();
    serverPrefsDecode(got, buf.data(), buf.size());
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

TEST(prefs_tz_quarter_hours, "UNIT-PREFS-011", "Timezone offset: quarter hours accepted, anything else snapped or reset to UTC") {
    CHECK(tz_offset_valid(0.0f) && tz_offset_valid(5.5f) && tz_offset_valid(5.75f));
    CHECK(tz_offset_valid(-3.5f) && tz_offset_valid(-12.0f) && tz_offset_valid(14.0f));
    CHECK(tz_offset_valid(12.75f) && tz_offset_valid(-9.5f));
    CHECK(!tz_offset_valid(5.3f) && !tz_offset_valid(5.249f) && !tz_offset_valid(-0.1f));
    CHECK(!tz_offset_valid(-12.25f) && !tz_offset_valid(14.25f) && !tz_offset_valid(NAN));
    CHECK(tz_offset_sanitize(5.3f) == 5.25f);
    CHECK(tz_offset_sanitize(-3.4f) == -3.5f);
    CHECK(tz_offset_sanitize(15.0f) == 0.0f && tz_offset_sanitize(-13.0f) == 0.0f);
    CHECK(tz_offset_sanitize(NAN) == 0.0f);
    CHECK(tz_offset_minutes(5.75f) == 345 && tz_offset_minutes(-3.5f) == -210);
    CHECK(tz_offset_minutes(14.0f) == 840 && tz_offset_minutes(-12.0f) == -720);
    NodePrefs p; initNodePrefs(&p);
    CHECK(p.tz_offset == 0.0f);
    p.tz_offset = NAN;
    sanitizeNodePrefs(&p);
    CHECK(p.tz_offset == 0.0f);
    p.tz_offset = 9.6f;
    sanitizeNodePrefs(&p);
    CHECK(p.tz_offset == 9.5f);
    // The binary layouts hold signed whole hours in one byte.
    std::vector<uint8_t> c = companionFile(sample()), s = serverFile(sample());
    c[kCompanionTz] = (uint8_t)-9;
    s[kServerTz] = (uint8_t)-9;
    NodePrefs q = defaults();
    CHECK(companionPrefsDecode(q, c.data(), c.size()));
    CHECK(q.tz_offset == -9.0f);
    q = defaults();
    serverPrefsDecode(q, s.data(), s.size());
    CHECK(q.tz_offset == -9.0f);
}

TEST(prefs_tz_text, "UNIT-PREFS-012", "Timezone offset text: exact decimal hours and an 8-character clock label") {
    struct { int16_t min; const char *hours; const char *label; } cases[] = {
        { 0, "0.0", "UTC" },          { 120, "2.0", "UTC+2" },       { -300, "-5.0", "UTC-5" },
        { 330, "5.5", "UTC+5:30" },   { 345, "5.75", "UTC+5:45" },   { -210, "-3.5", "UTC-3:30" },
        { -30, "-0.5", "UTC-0:30" },  { 765, "12.75", "UTC+1245" },  { 630, "10.5", "UTC+1030" },
        { 840, "14.0", "UTC+14" },    { -720, "-12.0", "UTC-12" },   { 525, "8.75", "UTC+8:45" },
    };
    for (const auto &t : cases) {
        char h[TZ_OFFSET_TEXT_LEN], l[TZ_OFFSET_TEXT_LEN];
        tz_offset_format_hours(t.min, h, sizeof(h));
        tz_offset_format_label(t.min, l, sizeof(l));
        if (std::strcmp(h, t.hours) != 0 || std::strcmp(l, t.label) != 0)
            throw std::runtime_error(std::to_string(t.min) + " -> " + h + " / " + l);
        CHECK(std::strlen(l) <= 8);
    }
}
