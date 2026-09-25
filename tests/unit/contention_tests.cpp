#include "test.h"
#include <mesh/ContentionTracker.h>
using namespace mesh;

TEST(contention_window, "UNIT-CONTENTION-001", "Observation window exact boundary and wrap") {
    ContentionTracker c;
    CHECK(c.msUntilNextTick(100) == MAINTENANCE_IDLE);
    c.trackRetransmit(42, 0xFFFFFFF0u);
    CHECK(c.msUntilNextTick(0xFFFFFFF0u) == 10001);
    CHECK(c.recordDupeIfTracked(42, uint32_t(0xFFFFFFF0u + 10000u)));
    CHECK(!c.recordDupeIfTracked(42, uint32_t(0xFFFFFFF0u + 10001u)));
    CHECK(c.extractDupeCount(42) == -1 && c.getContentionEstimate() == 1.0f);
}
TEST(contention_saturation, "UNIT-CONTENTION-002", "Duplicate saturation and exactly-once finalization") {
    ContentionTracker c; c.trackRetransmit(1, 100);
    for (int i = 0; i < 300; ++i) CHECK(c.recordDupeIfTracked(1, 101));
    CHECK(c.extractDupeCount(1) == 255); CHECK(c.extractDupeCount(1) == -1);
    CHECK(c.getContentionEstimate() == 255.0f);
    CHECK(!c.recordDupeIfTracked(99, 101));
}
TEST(contention_caps, "UNIT-CONTENTION-003", "Reactive airtime and hard caps plus disabled backoff") {
    ContentionTracker c; c.trackRetransmit(1, 100); c.setBackoffMultiplier(2.0f);
    CHECK(c.getReactiveHeadroom(1, 10) == 20);
    c.addReactiveExtension(1, 115); CHECK(c.getReactiveHeadroom(1, 10) == 5);
    c.addReactiveExtension(1, 5); CHECK(c.getReactiveHeadroom(1, 10) == 0);
    c.trackRetransmit(2, 100); c.addReactiveExtension(2, 1999);
    CHECK(c.getReactiveHeadroom(2, 1000) == 1);
    c.addReactiveExtension(2, 1); CHECK(c.getReactiveHeadroom(2, 1000) == 0);
    c.setBackoffMultiplier(0); CHECK(c.getReactiveHeadroom(1, 1000) == 0);
}
TEST(contention_decay, "UNIT-CONTENTION-004", "Warmup eviction and decay independent of tick frequency") {
    ContentionTracker c; CHECK(c.getFloodDelayFactor() == 0.5f);
    for (unsigned hash = 0; hash < 4; ++hash) {
        c.trackRetransmit(hash, 100); CHECK(c.extractDupeCount(hash) == 0);
    }
    CHECK(c.isWarmedUp() && c.getFloodDelayFactor() == 0.4f);
    ContentionTracker d; d.trackRetransmit(100, 100);
    for (int i = 0; i < 8; ++i) d.recordDupeIfTracked(100, 101);
    CHECK(d.extractDupeCount(100) == 8);
    d.tick(300100); CHECK(d.getContentionEstimate() == 8.0f);
    d.tick(300101); CHECK(d.getContentionEstimate() == 7.0f);
    for (int i = 0; i < 100; ++i) d.tick(300102);
    CHECK(d.getContentionEstimate() == 7.0f);
    d.tick(305101); CHECK(d.getContentionEstimate() == 6.125f);
    ContentionTracker e;
    for (unsigned i = 0; i < 25; ++i) e.trackRetransmit(i, 100);
    CHECK(e.extractDupeCount(0) == -1 && e.extractDupeCount(1) == 0);
}
