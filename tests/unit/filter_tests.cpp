#include "test.h"
#include <FwdFilter.h>
#include <string.h>
using namespace mesh;

static Packet floodPkt(uint8_t type, uint8_t hash_size, std::initializer_list<uint8_t> path,
                       uint8_t dst = 0x80, uint8_t src = 0xB1) {
    Packet p;
    p.header = ROUTE_TYPE_FLOOD | (type << 2);
    p.setPathHashSizeAndCount(hash_size, (uint8_t)(path.size() / hash_size));
    size_t i = 0;
    for (uint8_t b : path) p.path[i++] = b;
    p.payload[0] = dst; p.payload[1] = src; p.payload_len = 20;
    return p;
}
static bool run(FwdFilter& f, const char* line, char* reply, bool* changed = nullptr) {
    bool c = false;
    reply[0] = 0;
    bool handled = f.command(line, reply, 161, &c);
    if (changed) *changed = c;
    return handled;
}

TEST(filter_locked, "UNIT-FILTER-001", "Locked: filter commands are not handled and nothing is dropped") {
    FwdFilter f;
    char r[161];
    bool changed = true;
    CHECK(!run(f, "filter", r)); CHECK(!run(f, "filter rpt add dead", r));
    CHECK(!run(f, "idkfa on", r)); CHECK(!run(f, "idkfax", r));
    CHECK(run(f, "idkfa", r, &changed)); CHECK(changed); CHECK(f.isEnabled());
    CHECK(run(f, "filter rpt add dead", r)); CHECK(run(f, "filter src add b1", r));
    CHECK(run(f, "filter req.limit 1", r));
    Packet p = floodPkt(PAYLOAD_TYPE_REQ, 2, {0xDE, 0xAD});
    CHECK(f.matches(&p));
    CHECK(run(f, "idkfa off", r, &changed)); CHECK(changed); CHECK(!f.isEnabled());
    CHECK(!f.matches(&p)); CHECK(!f.overLimit(&p, 0)); CHECK(!f.overLimit(&p, 1));
    CHECK(!run(f, "filter", r));
    CHECK(run(f, "idkfa", r)); CHECK(f.numRepeaters() == 1); CHECK(f.numSources() == 1);
    CHECK(f.matches(&p));
}
TEST(filter_rpt, "UNIT-FILTER-002", "Repeater rule matches any path entry on the shared prefix, floods only") {
    FwdFilter f;
    char r[161];
    run(f, "idkfa", r); run(f, "filter rpt add dead", r);
    Packet p2 = floodPkt(PAYLOAD_TYPE_TXT_MSG, 2, {0x11, 0x22, 0xDE, 0xAD, 0x33, 0x44});
    Packet p1 = floodPkt(PAYLOAD_TYPE_ADVERT, 1, {0x11, 0xDE});
    Packet p3 = floodPkt(PAYLOAD_TYPE_GRP_TXT, 3, {0xDE, 0xAD, 0xBE});
    Packet miss2 = floodPkt(PAYLOAD_TYPE_TXT_MSG, 2, {0xDE, 0xAE, 0xAD, 0xDE});
    Packet miss1 = floodPkt(PAYLOAD_TYPE_TXT_MSG, 1, {0xAD, 0x11});
    Packet empty = floodPkt(PAYLOAD_TYPE_TXT_MSG, 2, {});
    CHECK(f.matches(&p2)); CHECK(f.matches(&p1)); CHECK(f.matches(&p3));
    CHECK(!f.matches(&miss2)); CHECK(!f.matches(&miss1)); CHECK(!f.matches(&empty));
    CHECK(f.droppedRpt() == 3);
    p2.header = ROUTE_TYPE_DIRECT | (PAYLOAD_TYPE_TXT_MSG << 2);
    CHECK(!f.matches(&p2));
    CHECK(run(f, "filter rpt del dead", r)); CHECK(f.numRepeaters() == 0);
    CHECK(!f.matches(&p1));
    CHECK(run(f, "filter rpt add xx", r)); CHECK(strncmp(r, "Err", 3) == 0);
    CHECK(run(f, "filter rpt add deadbeef", r)); CHECK(strncmp(r, "Err", 3) == 0);
}
TEST(filter_src, "UNIT-FILTER-003", "Source rule: flood REQ only, narrowed by dst and first repeater") {
    FwdFilter f;
    char r[161];
    run(f, "idkfa", r);
    CHECK(run(f, "filter src add b1e3 dst 80", r));
    Packet hit = floodPkt(PAYLOAD_TYPE_REQ, 1, {0x9C}, 0x80, 0xB1);
    Packet other_dst = floodPkt(PAYLOAD_TYPE_REQ, 1, {0x9C}, 0xC4, 0xB1);
    Packet txt = floodPkt(PAYLOAD_TYPE_TXT_MSG, 1, {0x9C}, 0x80, 0xB1);
    CHECK(f.matches(&hit)); CHECK(!f.matches(&other_dst)); CHECK(!f.matches(&txt));
    hit.header = ROUTE_TYPE_DIRECT | (PAYLOAD_TYPE_REQ << 2);
    CHECK(!f.matches(&hit));

    CHECK(run(f, "filter src del 0", r)); CHECK(f.numSources() == 0);
    CHECK(run(f, "filter src add 9f from 69af", r));
    Packet from_ok = floodPkt(PAYLOAD_TYPE_REQ, 2, {0x69, 0xAF, 0x11, 0x22}, 0x99, 0x9F);
    Packet from_later = floodPkt(PAYLOAD_TYPE_REQ, 2, {0x11, 0x22, 0x69, 0xAF}, 0x99, 0x9F);
    Packet zero_hop = floodPkt(PAYLOAD_TYPE_REQ, 2, {}, 0x99, 0x9F);
    CHECK(f.matches(&from_ok)); CHECK(!f.matches(&from_later)); CHECK(!f.matches(&zero_hop));

    CHECK(run(f, "filter src add de", r)); CHECK(strstr(r, "1/256") != nullptr);
    Packet bare = floodPkt(PAYLOAD_TYPE_REQ, 2, {}, 0x01, 0xDE);
    CHECK(f.matches(&bare));
    CHECK(run(f, "filter src", r)); CHECK(strcmp(r, "> 0:9f@69af 1:de") == 0);
    CHECK(run(f, "filter src del 7", r)); CHECK(strncmp(r, "Err", 3) == 0);

    Packet mine = floodPkt(PAYLOAD_TYPE_REQ, 1, {0x9C}, 0x42, 0xB1);
    CHECK(FwdFilter::isAddressedTo(&mine, 0x42)); CHECK(!FwdFilter::isAddressedTo(&mine, 0x43));
    Packet adv = floodPkt(PAYLOAD_TYPE_ADVERT, 1, {0x9C}, 0x42, 0xB1);
    CHECK(!FwdFilter::isAddressedTo(&adv, 0x42));
}
TEST(filter_limit, "UNIT-FILTER-004", "Request limit: per pair per hour, off at 0, oldest pair evicted") {
    FwdFilter f;
    char r[161];
    run(f, "idkfa", r);
    Packet a = floodPkt(PAYLOAD_TYPE_REQ, 1, {0x9C}, 0x80, 0xB1);
    Packet b = floodPkt(PAYLOAD_TYPE_REQ, 1, {0x9C}, 0xC4, 0xB1);
    Packet txt = floodPkt(PAYLOAD_TYPE_TXT_MSG, 1, {0x9C}, 0x80, 0xB1);
    for (int i = 0; i < 20; i++) CHECK(!f.overLimit(&a, 1000 * i));
    CHECK(run(f, "filter req.limit 2", r));
    const uint32_t t0 = 0xFFFFF000u;  // the window spans the millisecond wrap
    CHECK(!f.overLimit(&a, t0)); CHECK(!f.overLimit(&a, t0 + 600000));
    CHECK(f.overLimit(&a, t0 + 1200000)); CHECK(f.overLimit(&a, t0 + FwdFilter::LIMIT_WINDOW_MS - 1));
    CHECK(!f.overLimit(&b, t0 + 1200000));
    CHECK(!f.overLimit(&txt, t0 + 1200000)); CHECK(!f.overLimit(&txt, t0 + 1200001)); CHECK(!f.overLimit(&txt, t0 + 1200002));
    CHECK(!f.overLimit(&a, t0 + FwdFilter::LIMIT_WINDOW_MS));
    CHECK(f.droppedLimit() == 2);

    // more pairs than slots: the pair with the oldest window is replaced
    FwdFilter g;
    run(g, "idkfa", r); run(g, "filter req.limit 1", r);
    for (int i = 0; i <= FwdFilter::LIMIT_SLOTS; i++) {
        Packet p = floodPkt(PAYLOAD_TYPE_REQ, 1, {}, (uint8_t)i, 0x10);
        CHECK(!g.overLimit(&p, 1000 + i));
    }
    Packet first = floodPkt(PAYLOAD_TYPE_REQ, 1, {}, 0, 0x10);
    Packet second = floodPkt(PAYLOAD_TYPE_REQ, 1, {}, 1, 0x10);
    CHECK(g.overLimit(&second, 2000));
    CHECK(!g.overLimit(&first, 2000));
}
TEST(filter_persist, "UNIT-FILTER-005", "Save and load round trip; a damaged blob changes nothing") {
    FwdFilter f, g;
    char r[161];
    run(f, "idkfa", r); run(f, "filter rpt add de", r); run(f, "filter rpt add c0ffee", r);
    run(f, "filter src add b1 dst 80 from 9c", r); run(f, "filter src add 9f", r);
    run(f, "filter req.limit 6", r);
    uint8_t blob[FwdFilter::BLOB_MAX];
    size_t len = f.save(blob, sizeof(blob));
    CHECK(len > 0); CHECK(f.save(blob, len - 1) == 0);
    len = f.save(blob, sizeof(blob));
    CHECK(g.load(blob, len));
    CHECK(g.isEnabled()); CHECK(g.reqLimit() == 6);
    CHECK(run(g, "filter rpt", r)); CHECK(strcmp(r, "> de c0ffee") == 0);
    CHECK(run(g, "filter src", r)); CHECK(strcmp(r, "> 0:b1>80@9c 1:9f") == 0);

    FwdFilter h;
    CHECK(!h.load(blob, len - 1)); CHECK(!h.load(blob, 3));
    blob[1] ^= 0xFF; CHECK(!h.load(blob, len)); blob[1] ^= 0xFF;
    blob[4] = FwdFilter::MAX_RPT + 1; CHECK(!h.load(blob, len));
    CHECK(!h.isEnabled()); CHECK(h.numRepeaters() == 0);

    // a full list is refused, and a long list is cut with '+' inside the cap
    FwdFilter full;
    run(full, "idkfa", r);
    char line[40];
    for (int i = 0; i < FwdFilter::MAX_SRC; i++) {
        snprintf(line, sizeof(line), "filter src add %02x dst 4f from a1b2c3", i);
        CHECK(run(full, line, r)); CHECK(strncmp(r, "OK", 2) == 0);
    }
    CHECK(run(full, "filter src add ff", r)); CHECK(strncmp(r, "Err", 3) == 0);
    char small[40];
    bool c;
    CHECK(full.command("filter src", small, sizeof(small), &c));
    CHECK(strlen(small) < sizeof(small)); CHECK(small[strlen(small) - 1] == '+');
    CHECK(run(full, "filter clear", r)); CHECK(full.numSources() == 0);
}
