#include "test.h"
#include <MeshTimeSync.h>
#include <helpers/time_sync.h>
#include <adapters/clock/ZephyrRTCDiscover.h>
#include <algorithm>
#include <array>
#include <cstring>

namespace {
uint32_t uptime;
std::vector<uint32_t> saved;
std::vector<time_sync_source> reports;
constexpr uint32_t epoch = 1000000;
struct Clock : mesh::RTCClock {
    uint32_t now = epoch + 100;
    unsigned sets = 0;
    uint32_t getCurrentTime() override { return now; }
    // As ZephyrRTCClock: every set also goes to the hardware RTC.
    void setCurrentTime(uint32_t value) override { now = value; ++sets; saved.push_back(value); }
};
std::array<uint8_t, 32> key(unsigned id) {
    std::array<uint8_t, 32> result{};
    result[0] = id & 255; result[1] = id >> 8; return result;
}
struct Fixture {
    MeshTimeSync sync;
    Clock clock;
    Fixture(uint32_t build = 0, bool forward = false) : sync(build, forward) {
        uptime = 100; saved.clear(); reports.clear();
        CHECK(!sync.runTick(clock)); // arm first evaluation at uptime 1000
    }
    void advert(unsigned id, int32_t skew, uint32_t up, uint8_t hops = 0) {
        auto pub = key(id);
        sync.onAdvertHeard(pub.data(), uint32_t(int64_t(epoch) + up + skew), hops, up);
    }
    void group(unsigned count, int32_t skew, uint32_t first = 100, uint32_t second = 3700, unsigned idBase = 1) {
        for (unsigned i = 0; i < count; ++i) {
            advert(idBase + i, skew, first); advert(idBase + i, skew, second);
        }
    }
    bool evaluate(uint32_t up) {
        uptime = up; clock.now = epoch + up; return sync.runTick(clock);
    }
    std::string status(uint32_t up, bool enabled = true) {
        char out[4096]; int n = sync.formatStatus(out, sizeof(out), epoch + up, up, enabled);
        CHECK(n >= 0 && size_t(n) == std::strlen(out)); return out;
    }
};
}
extern "C" int64_t k_uptime_get() { return int64_t(uptime) * 1000; }
extern "C" void time_sync_report(time_sync_source source) { reports.push_back(source); }

TEST(timesync_pacing, "UNIT-SYNC-001", "Evaluation pacing and empty evidence leave clock untouched") {
    Fixture f;
    CHECK(f.sync.msUntilNextEval(100) == 900000);
    CHECK(!f.evaluate(999)); CHECK(f.sync.msUntilNextEval(999) == 1000);
    CHECK(!f.evaluate(1000)); CHECK(f.sync.msUntilNextEval(1000) == 900000);
    CHECK(f.status(1000).find("no-data") != std::string::npos);
    CHECK(f.clock.sets == 0 && saved.empty() && reports.empty());
}
TEST(timesync_quorum, "UNIT-SYNC-002", "Normal quorum needs six tenured senders and two adverts") {
    for (unsigned count : {5u, 6u}) {
        Fixture f; f.group(count, 1000);
        CHECK(f.evaluate(3700) == (count == 6));
        CHECK(f.clock.sets == (count == 6 ? 1u : 0u));
    }
    for (uint32_t now : {3699u, 3700u}) {
        Fixture f; f.group(6, 1000, 100, 200);
        CHECK(f.evaluate(now) == (now == 3700));
    }
    Fixture f;
    for (unsigned i = 1; i <= 6; ++i) f.advert(i, 1000, 100);
    CHECK(!f.evaluate(3700)); // elapsed tenure alone does not establish consistency
}
TEST(timesync_thresholds, "UNIT-SYNC-003", "Step threshold and positive negative one-hour caps") {
    for (int skew : {-5000, -600, -599, 599, 600, 5000}) {
        Fixture f; f.group(6, skew);
        bool expected = skew <= -600 || skew >= 600;
        CHECK(f.evaluate(3700) == expected);
        if (expected) {
            int delta = std::max(-3600, std::min(3600, skew));
            CHECK(f.sync.lastStepDelta() == delta);
            CHECK(f.clock.now == uint32_t(int64_t(epoch) + 3700 + delta));
            CHECK(saved == std::vector<uint32_t>({f.clock.now}));
            CHECK(reports == std::vector<time_sync_source>({TIME_SYNC_MESH}));
        } else CHECK(saved.empty() && reports.empty());
    }
}
TEST(timesync_majority, "UNIT-SYNC-004", "Split vote abstains and strict majority defeats outliers") {
    for (unsigned majority : {3u, 4u}) {
        Fixture f; f.group(majority, 1000); f.group(6 - majority, -2000, 100, 3700, 20);
        CHECK(f.evaluate(3700) == (majority == 4));
        if (majority == 4) CHECK(f.sync.lastStepDelta() == 1000);
        else CHECK(f.status(3700).find("no-majority") != std::string::npos);
    }
}
TEST(timesync_forward_only, "UNIT-SYNC-005", "Forward-only roles report but never apply backward corrections") {
    Fixture f(0, true); f.group(6, -1000);
    CHECK(!f.evaluate(3700)); CHECK(f.clock.sets == 0 && saved.empty() && reports.empty());
    CHECK(f.status(3700).find("skipped: forward-only") != std::string::npos);
    CHECK(f.sync.lastStepDelta() == 0);
}
TEST(timesync_bootstrap, "UNIT-SYNC-006", "Bootstrap needs three fresh peers and bypasses normal step cap") {
    for (unsigned count : {2u, 3u}) {
        Fixture f(epoch);
        for (unsigned i = 1; i <= count; ++i) f.advert(i, 0, 100);
        uptime = 1000; f.clock.now = 0;
        CHECK(f.sync.runTick(f.clock) == (count == 3));
        if (count == 3) CHECK(f.clock.now == epoch + 1000 - 150);
        else CHECK(saved.empty());
    }
    Fixture f(epoch); f.sync.noteManualSync(100);
    for (unsigned i = 1; i <= 3; ++i) f.advert(i, 0, 100);
    uptime = 1000; f.clock.now = 0;
    CHECK(!f.sync.runTick(f.clock) && saved.empty());
}
TEST(timesync_suppression, "UNIT-SYNC-007", "Manual and GPS suppression expire exactly at seven days") {
    for (bool gps : {false, true}) for (uint32_t elapsed : {604799u, 604800u}) {
        Fixture f;
        if (gps) f.sync.noteGPSSync(100); else f.sync.noteManualSync(100);
        uint32_t now = 100 + elapsed;
        f.group(6, 700, now - 3600, now); // fresh tenured evidence at expiry
        CHECK(f.evaluate(now) == (elapsed == 604800));
        if (elapsed < 604800) CHECK(f.status(now).find("suppressed") != std::string::npos);
    }
    Fixture f; f.sync.noteGPSSync(100); f.sync.noteGPSSync(600000);
    f.group(6, 700, 601300, 604900); CHECK(!f.evaluate(604900));
}
TEST(timesync_pedigree, "UNIT-SYNC-008", "Trusted-clock drift envelope rejects implausible consensus") {
    for (int skew : {781, 782}) { // floor(7 days * 300ppm) + 600 = 781 seconds
        Fixture f; f.sync.noteManualSync(100); f.group(6, skew, 601300, 604900);
        CHECK(f.evaluate(604900) == (skew == 781));
        if (skew == 782) CHECK(f.status(604900).find("pedigree-veto") != std::string::npos);
    }
}
TEST(timesync_rate_limit, "UNIT-SYNC-009", "Clock steps remain rate-limited for six hours") {
    for (uint32_t elapsed : {21599u, 21600u}) {
        Fixture f; f.group(6, 5000); CHECK(f.evaluate(3700)); // cap leaves 1400s to correct
        uint32_t now = 3700 + elapsed;
        uptime = now; f.clock.now = epoch + now + 3600;
        CHECK(f.sync.runTick(f.clock) == (elapsed == 21600));
        CHECK(f.clock.sets == (elapsed == 21600 ? 2u : 1u));
        if (elapsed == 21600) CHECK(f.sync.lastStepDelta() == 1400);
    }
}
TEST(timesync_admission, "UNIT-SYNC-010", "Hop limit monotonic dedup and eight-byte sender identity") {
    Fixture f; auto pub = key(1);
    f.advert(1, 1000, 100, 4); CHECK(f.sync.wouldAccept(pub.data(), epoch + 1100));
    f.advert(1, 1000, 100, 3); CHECK(!f.sync.wouldAccept(pub.data(), epoch + 1100));
    CHECK(!f.sync.wouldAccept(pub.data(), epoch + 1099));
    CHECK(f.sync.wouldAccept(pub.data(), epoch + 1101));
    auto separate = pub; separate[7] = 1;
    CHECK(f.sync.wouldAccept(separate.data(), epoch + 1100));
    separate = pub; separate[8] = 1;
    CHECK(!f.sync.wouldAccept(separate.data(), epoch + 1100)); // table key is exactly 8 bytes
    f.group(5, 1000, 100, 3700, 2);
    f.advert(1, 1000, 100, 3); // duplicate cannot earn the second advert
    CHECK(!f.evaluate(3700));
}
TEST(timesync_stale_inconsistent, "UNIT-SYNC-011", "Stale samples expire and inconsistent sender must re-earn tenure") {
    for (uint32_t age : {432000u, 432001u}) {
        Fixture f; f.group(6, 1000); CHECK(f.evaluate(3700 + age) == (age == 432000));
    }
    Fixture f; f.group(6, 1000);
    f.advert(1, 2000, 3800); // timestamp jumped an extra 1000 seconds
    CHECK(!f.evaluate(3800));
    CHECK(f.status(3800).find("no-quorum") != std::string::npos);
    f.advert(1, 1000, 7500); // inconsistent again: still cannot vote
    CHECK(!f.evaluate(7500));
}
TEST(timesync_eviction, "UNIT-SYNC-012", "Full evidence table protects mature and equally-close senders") {
    Fixture f;
    for (unsigned i = 1; i <= 32; ++i) f.advert(i, 0, 100, 3);
    auto stranger = key(99);
    f.advert(99, 0, 101, 3); CHECK(f.sync.wouldAccept(stranger.data(), epoch + 101));
    f.advert(99, 0, 102, 0); CHECK(!f.sync.wouldAccept(stranger.data(), epoch + 102));
    auto evicted = key(1); CHECK(f.sync.wouldAccept(evicted.data(), epoch + 100));
    Fixture mature;
    for (unsigned i = 1; i <= 32; ++i) {
        mature.advert(i, 0, 100, 3); mature.advert(i, 0, 3700, 3);
    }
    mature.advert(99, 0, 3701, 0); CHECK(mature.sync.wouldAccept(stranger.data(), epoch + 3701));
    mature.advert(99, 0, 3700 + 86401, 0);
    CHECK(!mature.sync.wouldAccept(stranger.data(), epoch + 3700 + 86401));
}
TEST(timesync_status_bounds, "UNIT-SYNC-013", "Status formatting respects every buffer capacity including zero") {
    Fixture f; f.group(6, 1000);
    for (size_t capacity = 0; capacity <= 256; ++capacity) {
        std::vector<char> out(capacity + 2, '#');
        int written = f.sync.formatStatus(out.data() + 1, capacity, epoch + 3700, 3700, true);
        CHECK(out.front() == '#' && out.back() == '#');
        CHECK(written >= 0 && size_t(written) <= (capacity ? capacity - 1 : 0));
        if (capacity) CHECK(std::strlen(out.data() + 1) == size_t(written));
    }
    CHECK(f.status(3700, false).find("off (dry-run)") == 0);
    CHECK(f.clock.sets == 0 && saved.empty());
}
TEST(timesync_reset, "UNIT-SYNC-014", "Reset discards evidence suppression counters and scheduling state") {
    Fixture f; f.group(6, 1000); CHECK(f.evaluate(3700)); f.sync.noteManualSync(3700);
    f.sync.reset(0, false);
    CHECK(f.sync.lastStepDelta() == 0 && f.sync.msUntilNextEval(3700) == 0);
    auto pub = key(1); CHECK(f.sync.wouldAccept(pub.data(), epoch + 1100));
    CHECK(f.status(3700).find("no-data") != std::string::npos);
    CHECK(f.status(3700).find("evals=0") != std::string::npos);
    f.group(6, 1000, 3800, 7400); CHECK(!f.evaluate(7400)); // re-arm pacing
    CHECK(f.evaluate(8300)); // old suppression is gone
}
