// What every zcpfs firmware role shares (zephcore/helpers/zcpfs/ZcpfsCommon.h):
// the advert tag and how legacy nodes read it, the run-time mode and the
// legacy-login setting on the CLI, and the bookkeeping behind passive mode.
// The glue classes themselves run on the bench (devdocs/lld/14-zcpfs.md 8).
#define CONFIG_ZEPHCORE_ZCPFS 1
#define CONFIG_ZEPHCORE_ZCPFS_PEERS 4

#include "test.h"
#include <cstring>
#include <string>

#include <helpers/AdvertDataHelpers.h>
#include <helpers/zcpfs/ZcpfsCommon.h>

namespace {

struct Advert {
    uint8_t data[MAX_ADVERT_DATA_SIZE];
    uint8_t len;
};

Advert build(const char *name, bool latlon = false) {
    Advert a{};
    if (latlon) {
        AdvertDataBuilder b(ADV_TYPE_CHAT, name, 47.5, 19.04);
        a.len = b.encodeTo(a.data);
    } else {
        AdvertDataBuilder b(ADV_TYPE_CHAT, name);
        a.len = b.encodeTo(a.data);
    }
    return a;
}

std::string run(int (*cmd)(const char *, uint8_t &, char *, size_t), const char *line, uint8_t &v, int *r = nullptr) {
    char reply[64] = "untouched";
    int res = cmd(line, v, reply, sizeof(reply));
    if (r) *r = res;
    return reply;
}

}  // namespace

TEST(zcpfs_glue_advert_tag, "UNIT-ZCPFSG-001",
     "The advert tag is one zero byte after the name: zcpfs reads the capability, the legacy parser the same name") {
    const char *names[] = {"Bob", "", "a name of exactly 31 chars....!", "árvíztűrő tükörfúrógép"};
    for (const char *name : names) {
        for (bool latlon : {false, true}) {
            Advert a = build(name, latlon);
            AdvertDataParser plain(a.data, a.len);
            CHECK(zcpfsAdvertCapability(a.data, a.len) == 0);

            uint8_t len = zcpfsTagAdvert(a.data, a.len);
            CHECK(len <= MAX_ADVERT_DATA_SIZE);
            CHECK(zcpfsAdvertCapability(a.data, len) == 1);

            AdvertDataParser tagged(a.data, len);
            CHECK(tagged.isValid());
            CHECK(tagged.getType() == ADV_TYPE_CHAT);
            // the legacy view: the name ends at the tag; a name cut for room is a prefix, whole characters
            std::string before = plain.getName(), after = tagged.getName();
            CHECK(before.compare(0, after.size(), after) == 0);
            CHECK(after.size() == before.size() || a.len >= MAX_ADVERT_DATA_SIZE);
            CHECK(after.empty() || (static_cast<uint8_t>(after.back()) & 0xC0) != 0xC0);
        }
    }
    // an advert without a name field gets one, empty, so that the tag is always sent
    Advert none{};
    AdvertDataBuilder b(ADV_TYPE_REPEATER);
    none.len = b.encodeTo(none.data);
    uint8_t len = zcpfsTagAdvert(none.data, none.len);
    CHECK(zcpfsAdvertCapability(none.data, len) == 1);
    CHECK(std::string(AdvertDataParser(none.data, len).getName()).empty());
    // a version byte after the tag is read as that version
    Advert v2 = build("Ann");
    uint8_t l2 = zcpfsTagAdvert(v2.data, v2.len);
    v2.data[l2++] = 2;
    CHECK(zcpfsAdvertCapability(v2.data, l2) == 2);
}

TEST(zcpfs_glue_mode_cli, "UNIT-ZCPFSG-002",
     "get/set zcpfs: on, passive, off, 1, 0 and default; a change is reported once, anything else is refused") {
    uint8_t mode = ZCPFS_MODE_OFF;
    int r = 0;
    CHECK(run(zcpfsModeCommand, "get zcpfs", mode, &r) == "> off" && r == ZCPFS_CMD_DONE);
    CHECK(run(zcpfsModeCommand, "set zcpfs on", mode, &r) == "OK - zcpfs on" && r == ZCPFS_CMD_CHANGED);
    CHECK(mode == ZCPFS_MODE_ON);
    CHECK(run(zcpfsModeCommand, "set zcpfs 1", mode, &r) == "OK - zcpfs on" && r == ZCPFS_CMD_DONE);
    CHECK(run(zcpfsModeCommand, "set zcpfs passive", mode, &r) == "OK - zcpfs passive" && r == ZCPFS_CMD_CHANGED);
    CHECK(mode == ZCPFS_MODE_PASSIVE && run(zcpfsModeCommand, "get zcpfs", mode) == "> passive");
    CHECK(run(zcpfsModeCommand, "set zcpfs 0", mode, &r) == "OK - zcpfs off" && r == ZCPFS_CMD_CHANGED);
    CHECK(mode == ZCPFS_MODE_OFF);
    CHECK(run(zcpfsModeCommand, "set zcpfs on", mode, &r) == "OK - zcpfs on" && r == ZCPFS_CMD_CHANGED);
    CHECK(run(zcpfsModeCommand, "set zcpfs default", mode, &r) == "OK - zcpfs off" && r == ZCPFS_CMD_CHANGED);
    CHECK(run(zcpfsModeCommand, "set zcpfs maybe", mode, &r).rfind("Err", 0) == 0 && r == ZCPFS_CMD_DONE);
    CHECK(mode == ZCPFS_MODE_OFF);
    // other commands, including the dotted ones, are not this one's
    CHECK(run(zcpfsModeCommand, "get zcpfs.status", mode, &r) == "untouched" && r == ZCPFS_CMD_NONE);
    CHECK(run(zcpfsModeCommand, "set zcpfs.channel x on", mode, &r) == "untouched" && r == ZCPFS_CMD_NONE);
    CHECK(run(zcpfsModeCommand, "get zcpfsx", mode, &r) == "untouched" && r == ZCPFS_CMD_NONE);
}

TEST(zcpfs_glue_legacylogin_cli, "UNIT-ZCPFSG-003",
     "get/set zcpfs.legacylogin: stored as the refusal, so the zero default allows legacy logins; default is on") {
    uint8_t refuse = 0;
    int r = 0;
    CHECK(run(zcpfsLegacyLoginCommand, "get zcpfs.legacylogin", refuse, &r) == "> on" && r == ZCPFS_CMD_DONE);
    CHECK(run(zcpfsLegacyLoginCommand, "set zcpfs.legacylogin off", refuse, &r) == "OK - legacy logins off");
    CHECK(r == ZCPFS_CMD_CHANGED && refuse == 1);
    CHECK(run(zcpfsLegacyLoginCommand, "set zcpfs.legacylogin 0", refuse, &r) == "OK - legacy logins off");
    CHECK(r == ZCPFS_CMD_DONE);
    CHECK(run(zcpfsLegacyLoginCommand, "set zcpfs.legacylogin 1", refuse, &r) == "OK - legacy logins on");
    CHECK(r == ZCPFS_CMD_CHANGED && refuse == 0);
    CHECK(run(zcpfsLegacyLoginCommand, "set zcpfs.legacylogin 0", refuse, &r) == "OK - legacy logins off");
    CHECK(run(zcpfsLegacyLoginCommand, "set zcpfs.legacylogin default", refuse, &r) == "OK - legacy logins on");
    CHECK(r == ZCPFS_CMD_CHANGED && refuse == 0);
    CHECK(run(zcpfsLegacyLoginCommand, "set zcpfs.legacylogin never", refuse, &r).rfind("Err", 0) == 0);
    CHECK(refuse == 0);
    CHECK(run(zcpfsLegacyLoginCommand, "set zcpfs on", refuse, &r) == "untouched" && r == ZCPFS_CMD_NONE);
}

TEST(zcpfs_glue_heard, "UNIT-ZCPFSG-004",
     "Passive mode seals only to a contact heard sealed since the last own advert; the table keeps the newest") {
    ZcpfsHeard h;
    uint8_t a[32], b[32], c[32];
    memset(a, 0xA1, sizeof(a));
    memset(b, 0xB2, sizeof(b));
    memset(c, 0xC3, sizeof(c));

    CHECK(!h.since(a));                 // never heard
    h.mark(a, 100);
    CHECK(h.since(a));                  // no advert since boot: heard is enough
    h.advertised(150);
    CHECK(!h.since(a));                 // the peer may have seen the untagged advert
    h.mark(a, 150);
    CHECK(h.since(a));                  // spoke at the advert's second: has not processed it
    h.mark(b, 160);
    CHECK(h.since(b) && !h.since(c));

    // more contacts than records: the one heard longest ago gives way
    uint8_t keys[CONFIG_ZEPHCORE_ZCPFS_PEERS + 1][32];
    for (int i = 0; i <= CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) {
        memset(keys[i], 0x10 + i, sizeof(keys[i]));
        h.mark(keys[i], 200 + static_cast<uint32_t>(i));
    }
    CHECK(!h.since(a) && !h.since(b));
    for (int i = 1; i <= CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) CHECK(h.since(keys[i]));
    // wrap-safe: an advert after the 32-bit wrap still orders correctly
    ZcpfsHeard w;
    w.mark(a, 0xFFFFFFF0u);
    w.advertised(0x00000010u);
    CHECK(!w.since(a));
    w.mark(a, 0x00000011u);
    CHECK(w.since(a));
}
