#include "test.h"
#include <mesh/StaticPoolPacketManager.h>
#include <set>
using namespace mesh;

TEST(pool_capacity, "UNIT-POOL-001", "Exhaust reclaim and reuse all pool slots") {
    StaticPoolPacketManager pool;
    std::set<Packet*> allocated;
    for (int i = 0; i < 32; ++i) {
        auto p = pool.allocNew(); CHECK(p != nullptr); CHECK(allocated.insert(p).second);
    }
    CHECK(pool.allocNew() == nullptr && pool.getFreeCount() == 0);
    for (auto p : allocated) pool.free(p);
    CHECK(pool.getFreeCount() == 32);
    for (int i = 0; i < 1000; ++i) {
        auto p = pool.allocNew(); CHECK(allocated.count(p) == 1); pool.free(p);
        CHECK(pool.getFreeCount() == 32);
    }
    pool.free(nullptr); CHECK(pool.getFreeCount() == 32);
}
TEST(pool_priority, "UNIT-POOL-002", "Only due packets dispatch in priority and tie order") {
    StaticPoolPacketManager pool;
    auto a = pool.allocNew(), b = pool.allocNew(), c = pool.allocNew();
    pool.queueOutbound(a, 2, 100); pool.queueOutbound(b, 1, 101); pool.queueOutbound(c, 2, 100);
    CHECK(pool.getNextOutbound(99) == nullptr);
    CHECK(pool.getOutboundCount(100) == 2 && pool.peekNextOutboundPriority(100) == 2);
    CHECK(pool.getNextOutbound(101) == b); pool.free(b);
    CHECK(pool.getNextOutbound(101) == a); pool.free(a);
    CHECK(pool.getNextOutbound(101) == c); pool.free(c);
    CHECK(pool.getOutboundTotal() == 0 && pool.getFreeCount() == 32);
}
TEST(pool_reschedule, "UNIT-POOL-003", "Reschedule remove and dispatch across clock wrap") {
    StaticPoolPacketManager pool;
    auto p = pool.allocNew(); pool.queueOutbound(p, 0, 0xFFFFFFF0u);
    CHECK(pool.getOutboundByIdx(0) == p);
    CHECK(pool.rescheduleOutbound(0, 0x10u)); CHECK(pool.getOutboundSchedule(0) == 0x10u);
    CHECK(pool.getNextOutbound(0xFFFFFFF0u) == nullptr);
    CHECK(pool.getNextOutbound(0x0Fu) == nullptr);
    CHECK(pool.getNextOutbound(0x10u) == p); pool.free(p);
    p = pool.allocNew(); pool.queueOutbound(p, 2, 500);
    CHECK(pool.removeOutboundByIdx(0) == p); pool.free(p);
    CHECK(!pool.rescheduleOutbound(0, 1)); CHECK(pool.removeOutboundByIdx(0) == nullptr);
    CHECK(pool.getFreeCount() == 32);
}
TEST(pool_sequence, "UNIT-POOL-004", "Seeded pool ownership conservation") {
    StaticPoolPacketManager pool;
    // Initialize the process-global pool through its public allocation API.
    auto initial = pool.allocNew(); pool.free(initial);
    std::vector<Packet*> held;
    uint32_t seed = 0x12345678;
    for (unsigned step = 0; step < 5000; ++step) {
        switch (nextRandom(seed) % 3) {
        case 0: if (auto p = pool.allocNew()) held.push_back(p); break;
        case 1: if (!held.empty()) { pool.queueOutbound(held.back(), nextRandom(seed) % 4, 0); held.pop_back(); } break;
        case 2: if (auto p = pool.getNextOutbound(0)) pool.free(p); break;
        }
        CHECK(pool.getFreeCount() + pool.getOutboundTotal() + int(held.size()) == 32);
        std::set<Packet*> owned(held.begin(), held.end());
        CHECK(owned.size() == held.size());
        for (int i = 0; i < pool.getOutboundTotal(); ++i) CHECK(owned.insert(pool.getOutboundByIdx(i)).second);
    }
    for (auto p : held) pool.free(p);
    while (auto p = pool.getNextOutbound(0)) pool.free(p);
    CHECK(pool.getFreeCount() == 32);
}
