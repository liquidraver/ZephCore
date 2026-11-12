// helpers/gps_guard.cpp: upstream's GPS_GUARD checks on fixes and GPS time.
#include "test.h"
#include "gps_guard.h"

#include <string>

namespace {
const uint32_t BUILD = 1790000000u;      // build day
const uint32_t T0 = BUILD + 30u * 86400u;  // a plausible GPS time
const double LAT = 47.5, LON = 19.05;

std::string status(const GpsGuard &g, uint32_t now_ms) {
    char b[64];
    g.formatStatus(b, sizeof(b), now_ms);
    return b;
}

// Five consistent fixes, one per second, from now_ms; returns the last verdict.
GpsGuard::Verdict settle(GpsGuard &g, uint32_t &now_ms, uint32_t &t, double lat = LAT,
                         double lon = LON) {
    GpsGuard::Verdict v = GpsGuard::REJECT;
    for (int i = 0; i < GPS_GUARD_STABLE_FIXES; i++) {
        v = g.checkFix(lat, lon, 8, 12, t, now_ms);
        t++;
        now_ms += 1000;
    }
    return v;
}
}

TEST(gpsguard_settle, "UNIT-GPSGUARD-001", "Five consistent fixes make a trusted fix; a duplicate GPS second does not count") {
    GpsGuard g(BUILD, 1000);
    uint32_t now = 1000, t = T0;
    for (int i = 1; i < GPS_GUARD_STABLE_FIXES; i++) {
        CHECK(g.checkFix(LAT, LON, 8, 12, t, now) == GpsGuard::SETTLE);
        CHECK(g.checkFix(LAT, LON, 8, 12, t, now) == GpsGuard::SETTLE);  // GGA and RMC
        t++;
        now += 1000;
    }
    CHECK(status(g, now) == "settle 4/5 time -");
    CHECK(g.checkFix(LAT, LON, 8, 12, t, now) == GpsGuard::TRUSTED);
    CHECK(status(g, now) == "ok 8sat time -");
    // trusted: the next fix passes at once
    CHECK(g.checkFix(LAT, LON + 0.0001, 8, 12, t + 1, now + 1000) == GpsGuard::TRUSTED);
    // a fix more than 100 m from the candidate restarts the count before trust
    GpsGuard h(BUILD, 0);
    now = 1000;
    t = T0;
    CHECK(h.checkFix(LAT, LON, 8, 12, t++, now) == GpsGuard::SETTLE);
    CHECK(h.checkFix(LAT + 0.01, LON, 8, 12, t++, now += 1000) == GpsGuard::SETTLE);
    CHECK(status(h, now) == "settle 1/5 time -");
}

TEST(gpsguard_reject, "UNIT-GPSGUARD-002", "Satellites, HDOP, region, fence and jump reject a fix and restart the count") {
    GpsGuard g(BUILD, 1000);
    uint32_t now = 1000, t = T0;
    CHECK(g.checkFix(LAT, LON, 3, 12, t, now) == GpsGuard::REJECT);
    CHECK(status(g, now) == "rej sats time -");
    CHECK(g.checkFix(LAT, LON, 8, 41, t, now) == GpsGuard::REJECT);
    CHECK(g.checkFix(LAT, LON, 8, GPS_GUARD_HDOP_UNKNOWN, t, now) == GpsGuard::REJECT);
    CHECK(status(g, now) == "rej hdop time -");
    CHECK(g.checkFix(LAT, LON, 8, 40, t, now) == GpsGuard::SETTLE);

    g.setRegion(45.0, 49.0, 16.0, 23.0);
    CHECK(g.checkFix(51.5, -0.1, 8, 12, t, now) == GpsGuard::REJECT);
    CHECK(status(g, now) == "rej region time -");

    // fence: only with a reference, 0 = off
    g.setReference(LAT, LON);
    CHECK(g.checkFix(LAT + 0.02, LON, 8, 12, t, now) == GpsGuard::REJECT);  // ~2.2 km
    CHECK(status(g, now) == "rej fence 2.2km time -");
    g.setFence(0);
    CHECK(g.checkFix(LAT + 0.02, LON, 8, 12, t, now) == GpsGuard::SETTLE);
    g.setFence(1000);

    // jump: 0.9 km (inside the fence) in 1 s from the last trusted fix
    CHECK(settle(g, now, t) == GpsGuard::TRUSTED);
    CHECK(g.checkFix(LAT + 0.008, LON, 8, 12, t, now) == GpsGuard::REJECT);
    CHECK(status(g, now) == "rej jump 0.9km time -");
    // after a rejection the count starts over, even when trusted before
    CHECK(g.checkFix(LAT, LON, 8, 12, t + 1, now + 1000) == GpsGuard::SETTLE);
    g.noFix(2);
    CHECK(status(g, now) == "no fix 2sat time -");
}

TEST(gpsguard_time, "UNIT-GPSGUARD-003", "GPS time: trusted fix, build-date window, 10 min step, then drift only") {
    GpsGuard g(BUILD, 0);
    uint32_t now = 1000, t = T0;
    CHECK(!g.acceptTime(t, 0, now));                     // no trusted fix
    CHECK(settle(g, now, t) == GpsGuard::TRUSTED);
    now -= 1000;
    CHECK(!g.acceptTime(BUILD - 2u * 86400u, 0, now));   // before the build
    CHECK(status(g, now) == "ok 8sat time rej range");
    CHECK(g.acceptTime(t, 0, now));                      // unset clock: any step
    CHECK(status(g, now) == "ok 8sat time ok +" + std::to_string(t) + "s");

    // after the first sync: drift only (5 s + 100 ppm)
    GpsGuard h(BUILD, 0);
    now = 1000;
    t = T0;
    settle(h, now, t);
    now -= 1000;
    CHECK(!h.acceptTime(t, t - 601, now));               // over 10 min off a plausible clock
    CHECK(h.acceptTime(t, t - 600, now));
    CHECK(!h.acceptTime(t, t - 6, now));                 // 6 s > 5 s right after
    CHECK(h.acceptTime(t, t - 5, now));
    CHECK(status(h, now) == "ok 8sat time ok +5s");
    // a clock outside the plausible window counts as unset
    CHECK(h.acceptTime(t, BUILD + 20u * 365u * 86400u, now));
    // no current trusted fix: an old "ok" reads "wait"
    CHECK(status(h, now + GPS_GUARD_TRUSTED_MS + 1) == "ok 8sat time wait");
    CHECK(!h.acceptTime(t, t, now + GPS_GUARD_TRUSTED_MS + 1));
    CHECK(status(h, now + GPS_GUARD_TRUSTED_MS + 1) == "ok 8sat time rej nofix");
    // a manual clock set drops the anchor: 10 min allowed again
    h.clockSetManually();
    CHECK(h.acceptTime(t, t - 500, now));
}

TEST(gpsguard_alert, "UNIT-GPSGUARD-004", "A steady GPS-clock offset raises an alert after an hour; accept clears it") {
    GpsGuard g(BUILD, 0);
    uint32_t now = 1000, t = T0;
    settle(g, now, t);
    now -= 1000;
    CHECK(g.acceptTime(t, t, now));
    const int32_t off = 3600;
    for (int min = 0; min <= GPS_GUARD_ALERT_MIN; min++) {
        // a fresh trusted fix every minute, an hour ahead of the clock
        now += 60000;
        t += 60;
        CHECK(g.checkFix(LAT, LON, 8, 12, t, now) == GpsGuard::TRUSTED);
        CHECK(!g.acceptTime(t + off, t, now));
        CHECK(g.isTimeAlert() == (min >= GPS_GUARD_ALERT_MIN));
    }
    CHECK(status(g, now) == "ok 8sat time rej step 3600s ALERT gps-clk +3600s 60m");
    CHECK(!g.acceptAlertedTime(off + 61, t + off, now));   // not the alerted offset
    CHECK(!g.acceptAlertedTime(off, t + off, now + GPS_GUARD_TRUSTED_MS + 1));  // stale fix
    CHECK(g.acceptAlertedTime(off, t + off, now));
    CHECK(!g.isTimeAlert());
    CHECK(status(g, now) == "ok 8sat time ok manual");
    // a changing offset never alerts
    GpsGuard h(BUILD, 0);
    now = 1000;
    t = T0;
    settle(h, now, t);
    now -= 1000;
    h.acceptTime(t, t, now);
    for (int min = 0; min < 2 * GPS_GUARD_ALERT_MIN; min++) {
        now += 60000;
        t += 60;
        h.checkFix(LAT, LON, 8, 12, t, now);
        h.acceptTime(t + 1000 + (min % 2) * 200, t, now);
    }
    CHECK(!h.isTimeAlert());
}

TEST(gpsguard_off, "UNIT-GPSGUARD-005", "Off: every fix and time passes; on again starts over") {
    GpsGuard g(BUILD, 1000);
    uint32_t now = 1000, t = T0;
    g.setEnabled(false);
    CHECK(g.checkFix(0.5, 0.5, 1, GPS_GUARD_HDOP_UNKNOWN, t, now) == GpsGuard::TRUSTED);
    CHECK(g.acceptTime(1, t, now));
    CHECK(status(g, now) == "off");
    g.setEnabled(true);
    CHECK(status(g, now) == "no fix time -");
    CHECK(g.checkFix(LAT, LON, 8, 12, t, now) == GpsGuard::SETTLE);
    char small[8];
    g.formatStatus(small, sizeof(small), now);
    CHECK(std::string(small) == "settle ");
}

TEST(gpsguard_distance, "UNIT-GPSGUARD-006", "Haversine distance") {
    double d = GpsGuard::distanceM(47.4979, 19.0402, 48.2082, 16.3738);  // Budapest - Vienna
    CHECK(d > 213000 && d < 216000);
    CHECK(GpsGuard::distanceM(LAT, LON, LAT, LON) == 0);
}
