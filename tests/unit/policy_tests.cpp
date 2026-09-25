#include "test.h"
#include <RoutingPolicy.h>
#include <RateLimiter.h>
#include <mesh/Maintenance.h>
using namespace mesh;

TEST(reply_routes, "UNIT-POLICY-001", "Reply routing and scope truth tables") {
    const ReplyRoute routes[] = {REPLY_ROUTE_FLOOD, REPLY_ROUTE_DIRECT_OUT_PATH,
        REPLY_ROUTE_DIRECT_SUPPLIED, REPLY_ROUTE_DIRECT_SUPPLIED,
        REPLY_ROUTE_PATH_RETURN, REPLY_ROUTE_PATH_RETURN, REPLY_ROUTE_PATH_RETURN, REPLY_ROUTE_PATH_RETURN};
    const ReplyScope scopes[] = {REPLY_SCOPE_NONE, REPLY_SCOPE_DEFAULT, REPLY_SCOPE_NONE,
        REPLY_SCOPE_NONE, REPLY_SCOPE_REQUEST, REPLY_SCOPE_REQUEST, REPLY_SCOPE_REQUEST, REPLY_SCOPE_REQUEST};
    for (int bits = 0; bits < 8; ++bits) {
        CHECK(chooseReplyRoute(bits & 4, bits & 2, bits & 1) == routes[bits]);
        CHECK(chooseReplyScope(bits & 4, bits & 2, bits & 1) == scopes[bits]);
    }
}
TEST(hop_limits, "UNIT-POLICY-002", "Scoped unscoped and advert hop boundaries") {
    Packet p;
    for (uint8_t route : {ROUTE_TYPE_FLOOD, ROUTE_TYPE_TRANSPORT_FLOOD}) {
        for (uint8_t type : {PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_TXT_MSG}) {
            p.header = route | (type << 2);
            unsigned limit = type == PAYLOAD_TYPE_ADVERT ? 2 : (route == ROUTE_TYPE_FLOOD ? 3 : 5);
            for (unsigned hops = 0; hops <= 6; ++hops) {
                p.setPathHashSizeAndCount(3, hops);
                CHECK(isFloodHopLimitExceeded(&p, 5, 3, 2) == (hops >= limit));
            }
        }
    }
    p.header = ROUTE_TYPE_FLOOD; p.path_len = 0;
    CHECK(isFloodHopLimitExceeded(&p, 5, 0, 2));
}
TEST(rate_window, "UNIT-RATE-001", "Rate window threshold expiry and reset") {
    RateLimiter limiter(2, 10);
    CHECK(limiter.allow(100)); CHECK(limiter.allow(101));
    CHECK(!limiter.allow(109)); CHECK(limiter.allow(110));
    CHECK(limiter.allow(111)); CHECK(!limiter.allow(112));
    limiter.reset(); CHECK(limiter.allow(113)); CHECK(limiter.allow(114));
    CHECK(!limiter.allow(115));
}
TEST(maintenance_wrap, "UNIT-TIME-001", "Maintenance deadlines across uint32 wrap") {
    CHECK(maintenanceUntil(100, 101) == 1);
    CHECK(maintenanceUntil(100, 100) == 0);
    CHECK(maintenanceUntil(100, 99) == 0);
    CHECK(maintenanceUntil(0xFFFFFFF0u, 0x10u) == 32);
    CHECK(maintenanceUntil(0x10u, 0xFFFFFFF0u) == 0);
    CHECK(maintenanceSooner(MAINTENANCE_IDLE, 32) == 32);
    CHECK(maintenanceSooner(0, MAINTENANCE_IDLE) == 0);
}
