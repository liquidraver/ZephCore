// helpers/gps_cli_text.cpp: the `gps` CLI's parsing and replies, in upstream's form.
#include "test.h"
#include "gps_cli_text.h"

#include <string>

namespace {
std::string dur(uint32_t s) {
    char b[16];
    gps_cli_duration(b, sizeof(b), s);
    return b;
}

gps_cli_view view() {
    gps_cli_view v = {};
    v.on = true;
    v.activity = GPS_CLI_ACTIVE;
    v.period_s = 300;
    return v;
}

std::string status(const gps_cli_view &v) {
    char b[161];
    gps_cli_status(b, sizeof(b), &v);
    return b;
}

std::string fix(const gps_cli_view &v) {
    char b[161];
    gps_cli_fix(b, sizeof(b), &v);
    return b;
}
}

TEST(gpscli_duration, "UNIT-GPSCLI-001", "Durations use one unit, rounded down, as upstream's formatDuration") {
    CHECK(dur(0) == "0s");
    CHECK(dur(59) == "59s");
    CHECK(dur(60) == "1m");
    CHECK(dur(3599) == "59m");
    CHECK(dur(3600) == "1h");
    CHECK(dur(172800) == "48h");
}

TEST(gpscli_period, "UNIT-GPSCLI-002", "gps period takes digits with an optional s/m/h/d suffix and refuses anything else") {
    uint32_t s = 7;
    CHECK(gps_cli_parse_period("0", &s) && s == 0);
    CHECK(gps_cli_parse_period("45", &s) && s == 45);
    CHECK(gps_cli_parse_period("45s", &s) && s == 45);
    CHECK(gps_cli_parse_period("5m", &s) && s == 300);
    CHECK(gps_cli_parse_period("2h", &s) && s == 7200);
    CHECK(gps_cli_parse_period("2d", &s) && s == 172800);
    CHECK(gps_cli_parse_period("99999999999d", &s) && s == UINT32_MAX);
    s = 7;
    CHECK(!gps_cli_parse_period("", &s));
    CHECK(!gps_cli_parse_period("-5", &s));
    CHECK(!gps_cli_parse_period(" 5", &s));
    CHECK(!gps_cli_parse_period("1.5h", &s));
    CHECK(!gps_cli_parse_period("5x", &s));
    CHECK(!gps_cli_parse_period("5mm", &s));
    CHECK(!gps_cli_parse_period("m", &s));
    CHECK(s == 7);
}

TEST(gpscli_status, "UNIT-GPSCLI-003", "gps status line: on/off, activity, last fix and mode") {
    gps_cli_view v = view();
    CHECK(status(v) == "> on, active, no fix, periodic (300s)");
    v.has_fix = true;
    v.sats = 7;
    v.fix_age_s = 190;
    v.activity = GPS_CLI_WAITING;
    CHECK(status(v) == "> on, waiting, 7 sats 3m ago, periodic (300s)");
    v.period_s = 0;
    v.activity = GPS_CLI_ACTIVE;
    v.fix_age_s = 1;
    CHECK(status(v) == "> on, active, 7 sats 1s ago, continuous");
    v.on = false;
    CHECK(status(v) == "> off, continuous");
    v.period_s = 172800;
    CHECK(status(v) == "> off, periodic (172800s)");
}

TEST(gpscli_fix, "UNIT-GPSCLI-004", "gps fix: live fix, last fix with age and acquisition time, or no fix") {
    gps_cli_view v = view();
    v.acq_s = 20;
    CHECK(fix(v) == "> No fix, acq 20s");
    v.activity = GPS_CLI_WAITING;
    CHECK(fix(v) == "> No fix");
    v.has_fix = true;
    v.sats = 9;
    v.lat = 47.497912;
    v.lon = -19.040235;
    v.fix_age_s = 7200;
    CHECK(fix(v) == "> Fix 2h ago (9 sat): 47.497912, -19.040235");
    v.activity = GPS_CLI_ACTIVE;
    v.acq_s = 75;
    CHECK(fix(v) == "> Fix 2h ago (9 sat), acq 1m: 47.497912, -19.040235");
    v.fix_live = true;
    CHECK(fix(v) == "> Fix (9 sat): 47.497912, -19.040235");
}

TEST(gpscli_stats, "UNIT-GPSCLI-005", "gps fix_stats: rounded TTFF last/max, dashes before any fix, run counters") {
    char b[64];
    gps_cli_fix_stats(b, sizeof(b), 0, 0, 0, 3);
    CHECK(std::string(b) == "> ttf:-/- ok:0 miss:3");
    gps_cli_fix_stats(b, sizeof(b), 11500, 185000, 4, 0);
    CHECK(std::string(b) == "> ttf:12s/3m ok:4 miss:0");
}

TEST(gpscli_cap, "UNIT-GPSCLI-006", "Replies stay inside a small buffer, terminated") {
    gps_cli_view v = view();
    v.has_fix = true;
    v.sats = 12;
    v.lat = -33.123456;
    v.lon = 151.123456;
    v.fix_age_s = 100000;
    char b[20];
    gps_cli_fix(b, sizeof(b), &v);
    CHECK(std::string(b).size() == sizeof(b) - 1);
    gps_cli_status(b, sizeof(b), &v);
    CHECK(std::string(b).size() == sizeof(b) - 1);
}
