// zcpfs inside the firmware: whole companions (mesh::Mesh, BaseChatMesh and the
// glue in helpers/zcpfs/, as built with CONFIG_ZEPHCORE_ZCPFS) on a simulated
// zero-hop channel. Legacy encryption is the firmware's own src/Utils.cpp over
// OpenSSL (support/psa/crypto.h). This binary is built with zcpfs on throughout,
// apart from zephcore_unit, because mesh::Mesh differs with it.
#include "test.h"
#include "dispatcher_fixture.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <mesh/SimpleMeshTables.h>
#include <helpers/BaseChatMesh.h>
#include <helpers/ClientACL.h>
#include <helpers/zcpfs/ZcpfsServer.h>
#include <adapters/datastore/ZephyrFsUtil.h>
#include <adapters/rng/ZephyrRNG.h>

// ---- host boundaries ---------------------------------------------------------

static uint64_t g_ms = 1000;

extern "C" int64_t k_uptime_get() { return (int64_t)g_ms; }
extern "C" uint32_t k_uptime_get_32() { return (uint32_t)g_ms; }
extern "C" int32_t k_msleep(int32_t ms) { g_ms += (uint64_t)ms; return 0; }

// The seed file of ZcpfsSeed.cpp, in memory.
static std::map<std::string, std::vector<uint8_t>> g_files;

extern "C" bool zephcore_fs_read_file(const char *path, uint8_t *buf, size_t cap, size_t *out_len) {
    auto f = g_files.find(path);
    if (f == g_files.end()) return false;
    *out_len = std::min(cap, f->second.size());
    memcpy(buf, f->second.data(), *out_len);
    return true;
}
static bool g_write_fails = false;
extern "C" bool zephcore_fs_atomic_replace(const char *path, const uint8_t *buf, size_t len, const char *) {
    if (g_write_fails) return false;
    g_files[path].assign(buf, buf + len);
    return true;
}

// Only the identity mixer's interface; deterministic here.
static int g_mixes = 0;
void mesh::ZephyrRNG::mixIdentitySeed(uint8_t *out, size_t len, const uint8_t *, size_t) {
    static uint32_t s = 0x5eed;
    g_mixes++;
    for (size_t i = 0; i < len; i++) out[i] = (uint8_t)(nextRandom(s) >> 24);
}

namespace {

struct Clock : mesh::MillisecondClock {
    unsigned long getMillis() override { return (unsigned long)g_ms; }
};
struct Rtc : mesh::RTCClock {
    uint32_t base = 1790000000;
    uint32_t getCurrentTime() override { return base + (uint32_t)(g_ms / 1000); }
    void setCurrentTime(uint32_t t) override { base = t - (uint32_t)(g_ms / 1000); }
};
struct Rng : mesh::RNG {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed) {}
    void random(uint8_t *dest, size_t n) override {
        for (size_t i = 0; i < n; i++) dest[i] = (uint8_t)(nextRandom(s) >> 24);
    }
};

// The firmware's StaticPoolPacketManager is one pool per process; every node
// here needs its own. Same order: the lowest priority value among the packets
// that are due, first queued first.
struct Pool : mesh::PacketManager {
    struct Entry { mesh::Packet *p; uint8_t pri; uint32_t at; };
    mesh::Packet packets[32];
    std::vector<mesh::Packet *> spare;
    std::vector<Entry> queue;
    Pool() { for (auto &p : packets) spare.push_back(&p); }
    static bool due(uint32_t at, uint32_t now) { return (int32_t)(at - now) <= 0; }
    mesh::Packet *allocNew() override {
        if (spare.empty()) return nullptr;
        mesh::Packet *p = spare.back();
        spare.pop_back();
        return p;
    }
    void free(mesh::Packet *p) override { spare.push_back(p); }
    void queueOutbound(mesh::Packet *p, uint8_t pri, uint32_t at) override { queue.push_back({p, pri, at}); }
    int best(uint32_t now) const {
        int k = -1;
        for (size_t i = 0; i < queue.size(); i++) {
            if (due(queue[i].at, now) && (k < 0 || queue[i].pri < queue[(size_t)k].pri)) k = (int)i;
        }
        return k;
    }
    mesh::Packet *getNextOutbound(uint32_t now) override {
        int k = best(now);
        return k < 0 ? nullptr : removeOutboundByIdx(k);
    }
    int getOutboundCount(uint32_t now) const override {
        if (now == 0xFFFFFFFF) return (int)queue.size();
        int n = 0;
        for (auto &e : queue) n += due(e.at, now) ? 1 : 0;
        return n;
    }
    int getOutboundTotal() const override { return (int)queue.size(); }
    int getFreeCount() const override { return (int)spare.size(); }
    mesh::Packet *getOutboundByIdx(int i) override { return queue[(size_t)i].p; }
    mesh::Packet *removeOutboundByIdx(int i) override {
        mesh::Packet *p = queue[(size_t)i].p;
        queue.erase(queue.begin() + i);
        return p;
    }
    uint32_t getOutboundSchedule(int i) const override { return queue[(size_t)i].at; }
    bool rescheduleOutbound(int i, uint32_t at) override {
        if (i >= (int)queue.size()) return false;
        queue[(size_t)i].at = at;
        return true;
    }
    uint8_t peekNextOutboundPriority(uint32_t now) const override {
        int k = best(now);
        return k < 0 ? 0xFF : queue[(size_t)k].pri;
    }
};

// What a node is made of, constructed before the mesh that refers to it.
struct Parts {
    test_support::Radio radio;
    Clock clock;
    Rng rng;
    Rtc rtc;
    Pool pool;
    SimpleMeshTables tables;
    explicit Parts(uint32_t seed) : rng(seed) {}
};

struct Node : Parts, BaseChatMesh {
    std::string name;
    uint8_t mode = ZCPFS_MODE_ON, no_legacy_login = 0, enforce = 0;
    uint8_t channels[ZCPFS_CHAN_SETTINGS][ZCPFS_CHAN_ENTRY] = {};
    std::vector<std::string> texts, channel_texts;
    std::vector<std::pair<uint32_t, ContactInfo *>> expected;
    std::vector<uint32_t> acked;

    Node(const char *n, uint32_t seed)
        : Parts(seed), BaseChatMesh(radio, clock, rng, rtc, pool, tables), name(n) {
        radio.completion = true;
        self_id = mesh::LocalIdentity(&rng);
        zcpfsBind(&mode, &no_legacy_login, &enforce, channels);
        begin();
    }

    std::string cli(const char *line) {
        char reply[256] = "";
        zcpfsCommand(line, reply, sizeof(reply));
        return reply;
    }
    ContactInfo *contact(const Node &other) { return lookupContactByPubKey(other.self_id.pub_key, PUB_KEY_SIZE); }
    ContactInfo *contact_of(const mesh::Mesh &other) { return lookupContactByPubKey(other.self_id.pub_key, PUB_KEY_SIZE); }
    void advert() {
        mesh::Packet *p = createSelfAdvert(name.c_str());
        CHECK(p != nullptr);
        sendZeroHop(p);
    }
    bool request(const Node &to) {
        uint8_t req[5] = {0x01, 0, 0, 0, 0};
        uint32_t tag = 0, timeout = 0;
        return sendRequest(*contact(to), req, sizeof(req), tag, timeout) != MSG_SEND_FAILED;
    }
    bool dm(const Node &to, const char *text, uint8_t attempt = 0) {
        ContactInfo *c = contact(to);
        CHECK(c != nullptr);
        uint32_t ack = 0, timeout = 0;
        int rc = sendMessage(*c, rtc.getCurrentTime(), attempt, text, ack, timeout);
        if (rc == MSG_SEND_FAILED) return false;
        expected.push_back({ack, c});
        return true;
    }

    // no airtime budget: the simulated channel is the only limit
    uint8_t getDutyCyclePercent() const override { return 0; }

    // BaseChatMesh's application side
    void onDiscoveredContact(ContactInfo &, bool, uint8_t, const uint8_t *) override {}
    ContactInfo *processAck(const uint8_t *data) override {
        uint32_t v;
        memcpy(&v, data, 4);
        for (size_t i = 0; i < expected.size(); i++) {
            if (expected[i].first == v) {
                ContactInfo *c = expected[i].second;
                acked.push_back(v);
                expected.erase(expected.begin() + (long)i);
                return c;
            }
        }
        return nullptr;
    }
    void onContactPathUpdated(const ContactInfo &) override {}
    void onMessageRecv(const ContactInfo &, mesh::Packet *, uint32_t, const char *text) override { texts.push_back(text); }
    std::vector<std::string> commands;
    void onCommandDataRecv(const ContactInfo &, mesh::Packet *, uint32_t, const char *text) override { commands.push_back(text); }
    void onCLICommandRecv(const ContactInfo &, mesh::Packet *, uint32_t, const char *, char *reply) override { reply[0] = 0; }
    void onSignedMessageRecv(const ContactInfo &, mesh::Packet *, uint32_t, const uint8_t *, const char *) override {}
    uint32_t calcFloodTimeoutMillisFor(uint32_t airtime) const override { return 500 + 16 * airtime; }
    uint32_t calcDirectTimeoutMillisFor(uint32_t airtime, uint8_t path_len) const override {
        return 500 + (airtime * 6 + 250) * ((path_len & 63) + 1);
    }
    void onSendTimeout() override {}
    void onChannelMessageRecv(const mesh::GroupChannel &, mesh::Packet *, uint32_t, const char *text) override {
        channel_texts.push_back(text);
    }
    void onChannelDataRecv(const mesh::GroupChannel &, mesh::Packet *, uint16_t, const uint8_t *, size_t) override {}
    // a request is answered with 60 bytes: the tag, then 56 bytes of 0x42
    uint8_t onContactRequest(const ContactInfo &, uint32_t ts, const uint8_t *, uint8_t, uint8_t *reply) override {
        memcpy(reply, &ts, 4);
        memset(reply + 4, 0x42, 56);
        return 60;
    }
    std::vector<std::vector<uint8_t>> responses;
    void onContactResponse(const ContactInfo &, const uint8_t *data, uint8_t len) override {
        responses.emplace_back(data, data + len);
    }
};

// Every node hears every other directly (zero hop), with no loss.
struct Air {
    std::vector<Node *> nodes;
    void run(uint32_t ms) {
        for (uint32_t t = 0; t < ms; t += 10) {
            for (Node *n : nodes) {
                n->radio.completion = true;
                n->loop();
            }
            for (Node *n : nodes) {
                for (auto &frame : n->radio.sent) {
                    if (getenv("ZCPFS_DEBUG")) {
                        std::printf("  t=%lu %s sends type %u, %zu bytes\n", (unsigned long)g_ms, n->name.c_str(),
                                    (frame[0] >> 2) & 0x0F, frame.size());
                    }
                    for (Node *o : nodes) {
                        if (o != n) o->radio.incoming.push_back(frame);
                    }
                }
                n->radio.sent.clear();
            }
            g_ms += 10;
        }
    }
};

uint32_t counter(Node &n, const char *field) {
    std::string s = n.cli("get zcpfs.status");
    size_t at = s.find(std::string(" ") + field);
    CHECK(at != std::string::npos);
    return (uint32_t)std::stoul(s.substr(at + 1 + strlen(field)));
}

}  // namespace

TEST(zcpfs_mesh_first_contact, "UNIT-ZCPFSM-001",
     "Two zcpfs companions pin each other by tagged adverts; a DM goes sealed and its ACK reaches the app") {
    Node a("alice", 11), b("bob", 22);
    Air air{{&a, &b}};
    a.advert();
    b.advert();
    air.run(3000);
    CHECK(a.contact(b) && b.contact(a));
    CHECK((a.contact(b)->flags & ZCPFS_PIN_FLAG) && (b.contact(a)->flags & ZCPFS_PIN_FLAG));

    CHECK(a.dm(b, "hello bob"));
    air.run(20000);
    CHECK(b.texts.size() == 1 && b.texts[0] == "hello bob");
    CHECK(a.acked.size() == 1 && a.expected.empty());
    CHECK(counter(a, "tx t") == 1 && counter(b, "rx t") == 1);

    CHECK(b.dm(a, "hi alice"));
    air.run(5000);
    CHECK(a.texts.size() == 1 && a.texts[0] == "hi alice");
    CHECK(b.acked.size() == 1);
}

namespace {
// Two pinned zcpfs companions with a session.
struct Pair {
    Node a{"alice", 11}, b{"bob", 22};
    Air air{{&a, &b}};
    Pair() {
        a.advert();
        b.advert();
        air.run(3000);
        CHECK(a.dm(b, "setup"));
        air.run(20000);
        CHECK(b.texts.size() == 1);
        b.texts.clear();
    }
};
bool pinned(Node &n, const Node &other) { return (n.contact(other)->flags & ZCPFS_PIN_FLAG) != 0; }
}  // namespace

TEST(zcpfs_mesh_legacy_peer, "UNIT-ZCPFSM-002",
     "A node with zcpfs off is a legacy peer: no tag, nothing pinned on the other side, DMs legacy both ways") {
    Node a("alice", 11), b("bob", 22);
    Air air{{&a, &b}};
    CHECK(b.cli("set zcpfs off") == "OK - zcpfs off");
    a.advert();
    b.advert();
    air.run(3000);
    CHECK(!pinned(a, b));
    CHECK(a.dm(b, "to legacy bob"));
    CHECK(b.dm(a, "from legacy bob"));
    air.run(15000);
    CHECK(b.texts.size() == 1 && b.texts[0] == "to legacy bob");
    CHECK(a.texts.size() == 1 && a.texts[0] == "from legacy bob");
    CHECK(a.acked.size() == 1 && b.acked.size() == 1);
    CHECK(counter(a, "tx t") == 0 && counter(a, "rx t") == 0);
    CHECK(counter(a, "hint t") == 1);           // a hints, as to any legacy peer; bob, off, ignores it
    CHECK(!pinned(a, b) && counter(a, "c0 h") == 0);   // no handshake started
}

TEST(zcpfs_mesh_passive, "UNIT-ZCPFSM-003",
     "Passive: sealed while the peer still pins us; after our untagged advert legacy both ways, nothing lost") {
    Pair p;
    CHECK(p.b.cli("set zcpfs passive") == "OK - zcpfs passive");
    CHECK(p.a.dm(p.b, "to passive bob"));
    p.air.run(5000);
    CHECK(p.b.texts.size() == 1 && p.a.acked.size() == 2);
    CHECK(p.b.dm(p.a, "from passive bob, still pinned"));
    p.air.run(5000);
    CHECK(p.a.texts.size() == 1 && p.b.acked.size() == 1);
    CHECK(counter(p.b, "tx t") == 1);           // sealed: alice spoke sealed since bob's last advert
    p.b.advert();                               // untagged
    p.air.run(3000);
    CHECK(!pinned(p.a, p.b));
    CHECK(p.b.dm(p.a, "from bob after his advert"));
    CHECK(p.a.dm(p.b, "to bob after his advert"));
    p.air.run(15000);
    CHECK(p.a.texts.size() == 2 && p.a.texts[1] == "from bob after his advert");
    CHECK(p.b.texts.size() == 2 && p.b.texts[1] == "to bob after his advert");
    CHECK(p.b.acked.size() == 2 && p.a.acked.size() == 3);
    CHECK(counter(p.b, "tx t") == 1);           // the last one went legacy
}

TEST(zcpfs_mesh_hint, "UNIT-ZCPFSM-004",
     "Two zcpfs nodes that missed each other's adverts find each other through the hint of a legacy DM") {
    Node a("alice", 11), b("bob", 22);
    Air air{{&a, &b}};
    a.cli("set zcpfs passive");                 // advertise untagged, then on without an advert
    b.cli("set zcpfs passive");
    a.advert();
    b.advert();
    air.run(3000);
    a.cli("set zcpfs on");
    b.cli("set zcpfs on");
    CHECK(!pinned(a, b) && !pinned(b, a));
    CHECK(a.dm(b, "legacy, with a hint"));
    air.run(20000);
    CHECK(b.texts.size() == 1 && a.acked.size() == 1);   // the legacy ACK value is unchanged
    CHECK(counter(a, "hint t") == 1 && counter(b, "hint t0 r") == 1);
    CHECK(pinned(a, b) && pinned(b, a));        // b handshook, a answered
    CHECK(a.dm(b, "now sealed"));
    air.run(5000);
    CHECK(b.texts.size() == 2 && b.texts[1] == "now sealed" && counter(a, "tx t") == 1);
}

TEST(zcpfs_mesh_channels, "UNIT-ZCPFSM-005",
     "Channels: on sends sealed and reads both, strict drops legacy, off reads sealed; the text is cut, not split") {
    Node a("alice", 11), b("bob", 22);
    Air air{{&a, &b}};
    uint8_t psk[16];
    for (int i = 0; i < 16; i++) psk[i] = (uint8_t)(0x30 + i);
    ChannelDetails *ca = a.addChannel("bench", psk, sizeof(psk));
    ChannelDetails *cb = b.addChannel("bench", psk, sizeof(psk));
    CHECK(ca && cb);
    auto send = [&](Node &n, ChannelDetails *c, const std::string &text) {
        CHECK(n.sendGroupMessage(n.rtc.getCurrentTime(), c->channel, n.name.c_str(), text.c_str(), (int)text.size()));
        air.run(3000);
    };
    send(a, ca, "legacy");
    CHECK(b.channel_texts.size() == 1 && counter(a, "ch t") == 0);
    CHECK(a.cli("set zcpfs.channel bench on") == "OK - bench: on");
    send(a, ca, "sealed");
    CHECK(b.channel_texts.size() == 2 && b.channel_texts[1] == "alice: sealed");
    CHECK(counter(a, "ch t") == 1 && counter(b, "ch t0 r") == 1);
    CHECK(b.cli("set zcpfs.channel 0 strict") == "OK - bench: strict");
    send(a, ca, "sealed to strict");
    CHECK(b.channel_texts.size() == 3);
    CHECK(a.cli("set zcpfs.channel bench default") == "OK - bench: off");
    send(a, ca, "legacy to strict");
    CHECK(b.channel_texts.size() == 3);         // dropped
    CHECK(a.cli("set zcpfs.channel bench on") == "OK - bench: on");
    std::string longer = std::string(150, 'x') + "\xc3\xa9\xc3\xa9";
    send(a, ca, longer);
    CHECK(b.channel_texts.size() == 4);
    std::string got = b.channel_texts[3];
    CHECK(got.size() == 161 - 5 && got.compare(0, 7, "alice: ") == 0);
    CHECK((static_cast<uint8_t>(got.back()) & 0xC0) != 0xC0);   // never ends inside a character
}

TEST(zcpfs_mesh_request, "UNIT-ZCPFSM-006",
     "A request and its response go sealed in pair mode; the response reaches the app at legacy's length") {
    Pair p;
    CHECK(p.a.request(p.b));
    p.air.run(5000);
    CHECK(p.a.responses.size() == 1);
    CHECK(p.a.responses[0].size() == 64);       // 60 bytes, zero-extended to AES blocks as legacy delivers them
    CHECK(p.a.responses[0][4] == 0x42 && p.a.responses[0][59] == 0x42 && p.a.responses[0][60] == 0);
    CHECK(counter(p.a, "tx t") == 1 && counter(p.b, "rx t1 d0 s") >= 1);
}

// ---- a server: the real ZcpfsServer and ClientACL inside a minimal role -----
//
// What RepeaterMesh's zcpfs block does (hooks, roles, the grant), and just enough
// of a repeater around it: a legacy login, CLI text, a request. The role code
// here is the test's; ZcpfsServer, ClientACL, Mesh and the library are the
// firmware's.
namespace {

struct Server : Parts, mesh::Mesh, ZcpfsServerHost {
    std::string name;
    std::string admin_pw = "admin1", guest_pw = "guest1";
    uint8_t mode = ZCPFS_MODE_ON, no_legacy_login = 0, enforce = 0;
    ClientACL acl;
    ZcpfsServer zs;
    int matching[MAX_CLIENTS];
    uint8_t hash_size = 0;

    Server(const char *n, uint32_t seed)
        : Parts(seed), mesh::Mesh(radio, clock, rng, rtc, pool, tables), name(n),
          zs(*this, acl, *this, 2, mode, no_legacy_login, enforce) {
        radio.completion = true;
        self_id = mesh::LocalIdentity(&rng);
        begin();
    }

    std::string cli(const char *line) {
        char reply[256] = "";
        zs.command(line, reply, sizeof(reply));
        return reply;
    }
    void advert() {
        uint8_t app_data[MAX_ADVERT_DATA_SIZE];
        AdvertDataBuilder b(ADV_TYPE_REPEATER, name.c_str());
        uint8_t len = zs.advert(app_data, b.encodeTo(app_data));
        sendZeroHop(createAdvert(self_id, app_data, len));
    }
    ClientInfo *client(const Node &n) { return acl.getClient(n.self_id.pub_key, PUB_KEY_SIZE); }
    void loop() {
        mesh::Mesh::loop();
        zs.loop();
    }

    uint8_t getDutyCyclePercent() const override { return 0; }

    // mesh::Mesh
    int searchPeersByHash(const uint8_t *hash) override {
        int n = 0;
        for (int i = 0; i < acl.getNumClients(); i++) {
            if (acl.getClientByIdx(i)->id.isHashMatch(hash)) matching[n++] = i;
        }
        return n;
    }
    void getPeerSharedSecret(uint8_t *dest, int idx) override {
        memcpy(dest, acl.getClientByIdx(matching[idx])->shared_secret, PUB_KEY_SIZE);
    }
    void reply(mesh::Packet *in, const ClientInfo &c, uint8_t type, const uint8_t *data, size_t len) {
        if (in->isRouteFlood()) {
            mesh::Packet *p = createPathReturn(c.id, c.shared_secret, in->path, in->path_len, type, data, len);
            if (p) sendFlood(p, 300, in->getPathHashSize());
        } else {
            mesh::Packet *p = createDatagram(type, c.id, c.shared_secret, data, len);
            if (p && c.out_path_len != OUT_PATH_UNKNOWN) sendDirect(p, c.out_path, c.out_path_len, 300);
            else if (p) sendFlood(p, 300, in->getPathHashSize());
        }
    }
    void onPeerDataRecv(mesh::Packet *pkt, uint8_t type, int idx, const uint8_t *, uint8_t *data, size_t len) override {
        ClientInfo &c = *acl.getClientByIdx(matching[idx]);
        if (type == PAYLOAD_TYPE_TXT_MSG && len > 5 && c.isAdmin()) {
            data[len] = 0;
            uint8_t temp[5 + 64];
            uint32_t t = rtc.getCurrentTime() + 1;
            memcpy(temp, &t, 4);
            temp[4] = (TXT_TYPE_CLI_DATA << 2);
            int n = snprintf((char *)&temp[5], 64, "> %s", (const char *)&data[5]);
            if (!zs.sendText(c, temp, 5 + (size_t)n, 300)) {
                mesh::Packet *p = createDatagram(PAYLOAD_TYPE_TXT_MSG, c.id, c.shared_secret, temp, 5 + (size_t)n);
                if (p && c.out_path_len != OUT_PATH_UNKNOWN) sendDirect(p, c.out_path, c.out_path_len, 300);
                else if (p) sendFlood(p, 300, 1);
            }
        } else if (type == PAYLOAD_TYPE_REQ && len >= 5) {
            uint8_t r[60];
            memcpy(r, data, 4);
            memset(r + 4, 0x24, 56);
            reply(pkt, c, PAYLOAD_TYPE_RESPONSE, r, sizeof(r));
        }
    }
    bool onPeerPathRecv(mesh::Packet *pkt, int idx, const uint8_t *, uint8_t *path, uint8_t path_len, uint8_t, uint8_t *,
                        uint8_t) override {
        ClientInfo &c = *acl.getClientByIdx(matching[idx]);
        c.out_path_len = mesh::Packet::copyPath(c.out_path, path, path_len);
        zs.onPath(c, pkt->isRouteFlood());
        return false;
    }
    // the legacy login: timestamp, password. As handleLoginReq: a known client's empty password logs it in
    // without a replay check; a password is checked against the stored timestamp.
    void onAnonDataRecv(mesh::Packet *pkt, const uint8_t *secret, const mesh::Identity &sender, uint8_t *data,
                        size_t len) override {
        if (pkt->getPayloadType() != PAYLOAD_TYPE_ANON_REQ || len < 4 || zs.refuseLegacyLogin(sender.pub_key)) return;
        data[len] = 0;
        uint32_t ts;
        memcpy(&ts, data, 4);
        ClientInfo *c = data[4] == 0 ? acl.getClient(sender.pub_key, PUB_KEY_SIZE) : nullptr;
        bool fresh = c == nullptr || ts > c->last_timestamp;
        if (c == nullptr) {
            if (admin_pw != (const char *)&data[4]) return;
            c = acl.putClient(sender, 0);
            if (ts <= c->last_timestamp) return;
            c->permissions = PERM_ACL_ADMIN;
            memcpy(c->shared_secret, secret, PUB_KEY_SIZE);
            c->last_timestamp = ts;
        }
        zs.onLegacyLogin(*c, fresh);
        uint8_t r[13] = {0};
        uint32_t t = rtc.getCurrentTime();
        memcpy(r, &t, 4);
        r[6] = 1;
        r[7] = c->permissions;
        reply(pkt, *c, PAYLOAD_TYPE_RESPONSE, r, sizeof(r));
    }

    // the zcpfs hooks, as RepeaterMesh's
    int onPeerSealedRecv(mesh::Packet *p, int idx, const uint8_t *, uint8_t *data, int *len) override {
        hash_size = p->getPathHashSize();
        return zs.rx(p, *acl.getClientByIdx(matching[idx]), data, len);
    }
    bool onStrangerSealedRecv(mesh::Packet *p) override {
        uint8_t data[MAX_PACKET_PAYLOAD];
        int len = 0;
        ClientInfo *c = nullptr;
        hash_size = p->getPathHashSize();
        int r = zs.rxStranger(p, data, &len, &c);
        if (r == SEALED_DATA && c != nullptr) {
            int n = searchPeersByHash(&p->payload[1]);
            for (int j = 0; j < n; j++) {
                if (acl.getClientByIdx(matching[j]) == c) onPeerDataRecv(p, PAYLOAD_TYPE_TXT_MSG, j, c->shared_secret, data, (size_t)len);
            }
        }
        return r != SEALED_NEXT;
    }
    void onPeerRecvDone() override {
        zs.rxDone();
        hash_size = 0;
    }
    int onAnonSealedRecv(mesh::Packet *p) override {
        hash_size = p->getPathHashSize();
        int r = zs.rxLogin(p);
        hash_size = 0;
        return r;
    }
    int sealOutgoing(uint8_t type, const uint8_t *dest_hash, const uint8_t *secret, const uint8_t *data, size_t len,
                     uint8_t *payload) override {
        return zs.sealOutgoing(type, dest_hash, secret, data, len, payload);
    }

    // ZcpfsServerHost, as RepeaterMesh's
    uint32_t zcpfsAirtime(int len) override { return radio.getEstAirtimeFor(len); }
    void zcpfsFlood(mesh::Packet *p, uint32_t delay) override { sendFlood(p, delay, hash_size ? hash_size : 1); }
    int zcpfsRoles(const ClientInfo *known, bool empty, ZcpfsRole roles[2]) override {
        int n = 0;
        if (empty) {
            if (known) roles[n++] = {"", known->permissions};
            else if (guest_pw.empty()) roles[n++] = {"", PERM_ACL_GUEST};
            return n;
        }
        if (!admin_pw.empty()) roles[n++] = {admin_pw.c_str(), PERM_ACL_ADMIN};
        if (!guest_pw.empty()) roles[n++] = {guest_pw.c_str(), PERM_ACL_GUEST};
        return n;
    }
    ClientInfo *zcpfsGrant(const mesh::Identity &id, uint8_t perms, const uint8_t *login, size_t len) override {
        ClientInfo *c = acl.putClient(id, 0);
        self_id.calcSharedSecret(c->shared_secret, id);
        if (len >= 4) memcpy(&c->last_timestamp, login, 4);
        c->permissions = perms;
        return c;
    }
    void zcpfsAclChanged(ClientInfo &) override {}
};

struct Bench {
    Node c{"carol", 33};
    Server s{"repeater", 44};
    void run(uint32_t ms) {
        for (uint32_t t = 0; t < ms; t += 10) {
            c.radio.completion = s.radio.completion = true;
            c.loop();
            s.loop();
            for (auto &f : c.radio.sent) s.radio.incoming.push_back(f);
            for (auto &f : s.radio.sent) c.radio.incoming.push_back(f);
            c.radio.sent.clear();
            s.radio.sent.clear();
            g_ms += 10;
        }
    }
    Bench() {
        s.advert();
        run(3000);
        CHECK(c.contact_of(s) && (c.contact_of(s)->flags & ZCPFS_PIN_FLAG));
    }
    // a login from the app: true when the app saw "login OK"
    bool login(const char *pw, uint32_t ms = 15000) {
        uint32_t timeout = 0;
        size_t before = c.responses.size();
        if (c.sendLogin(*c.contact_of(s), pw, timeout) == MSG_SEND_FAILED) return false;
        run(ms);
        for (size_t i = before; i < c.responses.size(); i++) {
            if (c.responses[i].size() >= 13 && c.responses[i][4] == 0) return true;
        }
        return false;
    }
    std::string cli(const char *text) {
        uint32_t timeout = 0;
        size_t before = c.commands.size();
        CHECK(c.sendCommandData(*c.contact_of(s), c.rtc.getCurrentTime(), 0, TXT_TYPE_CLI_DATA, text, timeout) !=
              MSG_SEND_FAILED);
        run(5000);
        return c.commands.size() > before ? c.commands.back() : "";
    }
};

uint32_t server_counter(Server &s, const char *field) {
    std::string st = s.cli("get zcpfs.status");
    size_t at = st.find(std::string(" ") + field + " ");
    CHECK(at != std::string::npos);
    return (uint32_t)std::stoul(st.substr(at + 2 + strlen(field)));
}

}  // namespace

TEST(zcpfs_mesh_server_login, "UNIT-ZCPFSM-007",
     "A zcpfs login to a server: the app sees login OK, the server grants on confirmation, CLI goes sealed") {
    Bench b;
    CHECK(b.login("admin1"));
    const std::vector<uint8_t> &r = b.c.responses.back();
    CHECK(r[6] == 1 && r[7] == PERM_ACL_ADMIN);         // the legacy response the app expects
    ClientInfo *ci = b.s.client(b.c);
    CHECK(ci != nullptr && ci->isAdmin() && ci->zcpfs_pinned);
    CHECK(server_counter(b.s, "granted") == 1);
    CHECK(b.c.contact_of(b.s)->out_path_len != OUT_PATH_UNKNOWN);   // the flooded reply carried a path
    CHECK(b.cli("ver") == "> ver");
    CHECK(counter(b.c, "tx t") == 1 && server_counter(b.s, "login") == 1);
}

TEST(zcpfs_mesh_server_wrong_password, "UNIT-ZCPFSM-008",
     "A wrong password is answered but opens nothing; nothing is granted; the right one then works") {
    Bench b;
    CHECK(!b.login("wrong"));
    CHECK(b.s.client(b.c) == nullptr && server_counter(b.s, "granted") == 0);
    CHECK(b.login("admin1"));
    CHECK(b.s.client(b.c) != nullptr);
}

TEST(zcpfs_mesh_server_budget, "UNIT-ZCPFSM-009",
     "Flooded login replies to a stranger: four in any 180 s, then over the reverse path; the right password still gets in") {
    Bench b;
    for (int i = 0; i < 5; i++) {
        b.c.contact_of(b.s)->out_path_len = OUT_PATH_UNKNOWN;   // every login floods
        CHECK(!b.login("wrong", 6000));
    }
    std::string st = b.s.cli("get zcpfs.status");
    CHECK(st.find(" answered 0/4/1 ") != std::string::npos);
    b.c.contact_of(b.s)->out_path_len = OUT_PATH_UNKNOWN;
    CHECK(b.login("admin1", 6000));
    CHECK(b.s.cli("get zcpfs.status").find(" answered 0/4/2 ") != std::string::npos);
}

TEST(zcpfs_mesh_server_legacylogin, "UNIT-ZCPFSM-010",
     "Legacy logins off: a server drops them unread, a client never sends one to a server that is not pinned") {
    Bench b;
    b.c.cli("set zcpfs passive");                       // a passive client logs in the legacy way
    CHECK(b.login("admin1"));
    CHECK(b.s.client(b.c) != nullptr && !b.s.client(b.c)->zcpfs_pinned);
    CHECK(b.s.cli("set zcpfs.legacylogin off") == "OK - legacy logins off");
    CHECK(!b.login("admin1"));
    CHECK(server_counter(b.s, "legacy refused") == 1);
    b.c.cli("set zcpfs on");
    CHECK(b.login("admin1"));                           // the zcpfs login is not affected
    // the client side: a server that does not pin
    b.s.mode = ZCPFS_MODE_PASSIVE;
    b.s.advert();
    b.run(3000);
    CHECK(!(b.c.contact_of(b.s)->flags & ZCPFS_PIN_FLAG));
    CHECK(b.c.cli("set zcpfs.legacylogin off") == "OK - legacy logins off");
    uint32_t timeout = 0;
    CHECK(b.c.sendLogin(*b.c.contact_of(b.s), "admin1", timeout) == MSG_SEND_FAILED);
}

TEST(zcpfs_mesh_server_modes, "UNIT-ZCPFSM-011",
     "A passive server answers a zcpfs login from a client that still pins it; a server switched off answers none") {
    Bench b;
    CHECK(b.s.cli("set zcpfs passive") == "OK - zcpfs passive");
    CHECK(b.login("admin1"));
    CHECK(b.s.client(b.c)->zcpfs_pinned);
    CHECK(b.s.cli("set zcpfs off") == "OK - zcpfs off");
    CHECK(!b.login("admin1"));
    CHECK(server_counter(b.s, "granted") == 1);         // only the passive one; off wipes sessions, not counters
}

TEST(zcpfs_mesh_seed_write_fails, "UNIT-ZCPFSM-012",
     "The seed file: read and replaced at every seeding; a failed write mixes in fresh entropy, so the next boot cannot repeat the stream") {
    g_files.clear();
    zcpfs_drbg g{};
    Rng first_boot(7);
    CHECK(zcpfsSeed(&g, first_boot, true) == ZCPFS_SEED_WRITTEN);      // no file yet: the identity mixer
    const std::vector<uint8_t> stale = g_files["/lfs/zcpfs_seed"];
    CHECK(stale.size() == 32);

    // a boot whose write fails, then a boot that reads the same file and gets the same hardware bytes
    uint8_t out_a[32], out_b[32];
    zcpfs_drbg a{}, b{};
    Rng hw_a(99), hw_b(99);
    g_write_fails = true;
    CHECK(zcpfsSeed(&a, hw_a, true) == (ZCPFS_SEED_FROM_FILE | ZCPFS_SEED_MIXED));
    CHECK(g_files["/lfs/zcpfs_seed"] == stale);
    g_write_fails = false;
    CHECK(zcpfsSeed(&b, hw_b, true) == (ZCPFS_SEED_FROM_FILE | ZCPFS_SEED_WRITTEN));
    zcpfs_drbg_generate(&a, out_a, sizeof(out_a));
    zcpfs_drbg_generate(&b, out_b, sizeof(out_b));
    CHECK(memcmp(out_a, out_b, sizeof(out_a)) != 0);
    CHECK(g_files["/lfs/zcpfs_seed"] != stale);                          // replaced once a write works
}

TEST(zcpfs_mesh_seed_after_boot, "UNIT-ZCPFSM-014",
     "The identity mixer runs only at boot: a seed taken later uses the RNG, says so, and still mixes on a failed write") {
    g_files.clear();
    zcpfs_drbg g{}, h{}, k{};
    Rng hw(5);
    int mixes = g_mixes;
    CHECK(zcpfsTakeSeed(&g, hw) == (ZCPFS_SEED_LATE | ZCPFS_SEED_WRITTEN));   // nothing seeded at boot, no file
    g_write_fails = true;
    CHECK(zcpfsTakeSeed(&h, hw) == (ZCPFS_SEED_LATE | ZCPFS_SEED_FROM_FILE | ZCPFS_SEED_MIXED));
    g_write_fails = false;
    CHECK(g_mixes == mixes);

    zcpfsSeedAtBoot(hw);                                                  // at boot: taken once, as seeded
    CHECK(g_mixes == mixes);                                              // the file was there: no mixer needed
    CHECK(zcpfsTakeSeed(&k, hw) == (ZCPFS_SEED_FROM_FILE | ZCPFS_SEED_WRITTEN));
    CHECK(zcpfsTakeSeed(&k, hw) & ZCPFS_SEED_LATE);                       // a second take seeds anew
    g_files.clear();
    zcpfsSeedAtBoot(hw);
    CHECK(g_mixes == mixes + 1);                                          // no file at boot: the mixer
}

TEST(zcpfs_mesh_advert_mark, "UNIT-ZCPFSM-013",
     "A sealed text raises the sender's advert mark: a recorded untagged advert from before can no longer unpin it") {
    Node a("alice", 11), b("bob", 22);
    Air air{{&a, &b}};
    a.cli("set zcpfs passive");                 // untagged adverts, then on: pinned by the hint's handshake
    b.cli("set zcpfs passive");
    a.advert();
    b.advert();
    air.run(3000);
    air.run(5000);
    b.advert();                                 // untagged, recorded, never delivered
    for (int i = 0; i < 1000 && b.radio.sent.empty(); i++) {
        b.radio.completion = true;
        b.loop();
        g_ms += 10;
    }
    std::vector<std::vector<uint8_t>> recorded = b.radio.sent;
    b.radio.sent.clear();
    CHECK(recorded.size() == 1);
    a.cli("set zcpfs on");
    b.cli("set zcpfs on");
    CHECK(a.dm(b, "legacy, with a hint"));
    air.run(20000);
    CHECK(pinned(a, b) && pinned(b, a));

    // a sealed text from bob more than the slack later: alice's mark passes the recording
    a.rtc.base += 2 * ZCPFS_ADVERT_MARK_SLACK;
    b.rtc.base += 2 * ZCPFS_ADVERT_MARK_SLACK;
    CHECK(b.dm(a, "sealed"));
    air.run(5000);
    CHECK(a.texts.size() == 1 && counter(b, "tx t") == 1);

    a.radio.incoming.push_back(recorded[0]);    // the replay
    air.run(3000);
    CHECK(pinned(a, b));
    CHECK(a.dm(b, "still sealed"));
    air.run(5000);
    CHECK(b.texts.size() == 2 && counter(a, "tx t") == 1);

    // a fresh untagged advert still unpins: within the slack of bob's last text
    b.cli("set zcpfs passive");
    b.advert();
    air.run(3000);
    CHECK(!pinned(a, b));
}

TEST(zcpfs_mesh_enforce_advert, "UNIT-ZCPFSM-015",
     "zcpfs.enforce on: a pinned contact's untagged advert is accepted but keeps the pin; with enforce off a newer one unpins") {
    Pair p;
    CHECK(p.a.cli("set zcpfs.enforce on") == "OK - zcpfs.enforce on");
    CHECK(p.a.cli("get zcpfs.enforce") == "> on");
    uint32_t before = p.a.contact(p.b)->last_advert_timestamp;
    p.b.cli("set zcpfs passive");
    p.b.advert();                               // untagged
    p.air.run(3000);
    CHECK(p.a.contact(p.b)->last_advert_timestamp > before);   // accepted
    CHECK(pinned(p.a, p.b));                    // and the pin kept
    CHECK(p.a.dm(p.b, "still sealed"));
    p.air.run(5000);
    CHECK(p.b.texts.size() == 1 && counter(p.a, "tx t") == 2);
    CHECK(p.a.cli("set zcpfs.enforce off") == "OK - zcpfs.enforce off");
    p.b.advert();
    p.air.run(3000);
    CHECK(!pinned(p.a, p.b));
}

TEST(zcpfs_mesh_server_enforce, "UNIT-ZCPFSM-016",
     "zcpfs.enforce on at a server: a pinned client's legacy login is dropped and the pin kept; an unpinned client logs in as before") {
    Bench b;
    CHECK(b.login("admin1"));
    CHECK(b.s.client(b.c)->zcpfs_pinned);
    CHECK(b.s.cli("set zcpfs.enforce on") == "OK - zcpfs.enforce on");
    b.c.cli("set zcpfs passive");               // the client logs in the legacy way
    CHECK(!b.login("admin1"));
    CHECK(b.s.client(b.c)->zcpfs_pinned && server_counter(b.s, "legacy refused") == 1);
    CHECK(b.s.cli("set zcpfs.enforce off") == "OK - zcpfs.enforce off");
    CHECK(b.login("admin1"));                   // a fresh legacy login unpins
    CHECK(!b.s.client(b.c)->zcpfs_pinned);
    CHECK(b.s.cli("set zcpfs.enforce on") == "OK - zcpfs.enforce on");
    CHECK(b.login("admin1"));                   // not pinned: enforce does not refuse it
    CHECK(server_counter(b.s, "legacy refused") == 1);
}

TEST(zcpfs_mesh_server_legacy_replay, "UNIT-ZCPFSM-017",
     "A recorded empty-password legacy login, replayed after the client's zcpfs login, keeps its pin; a fresh one unpins") {
    Bench b;
    b.c.cli("set zcpfs passive");
    CHECK(b.login("admin1"));                   // legacy: carol becomes a known client
    uint32_t timeout = 0;
    CHECK(b.c.sendLogin(*b.c.contact_of(b.s), "", timeout) != MSG_SEND_FAILED);
    for (int i = 0; i < 1000 && b.c.radio.sent.empty(); i++) {
        b.c.radio.completion = true;
        b.c.loop();
        g_ms += 10;
    }
    std::vector<std::vector<uint8_t>> recorded = b.c.radio.sent;   // recorded, never delivered
    b.c.radio.sent.clear();
    CHECK(recorded.size() == 1);
    b.run(2000);
    b.c.cli("set zcpfs on");
    CHECK(b.login("admin1"));                   // the zcpfs login pins carol
    CHECK(b.s.client(b.c)->zcpfs_pinned);
    b.s.radio.incoming.push_back(recorded[0]);  // the replay: older than carol's last login
    b.run(3000);
    CHECK(b.s.client(b.c)->zcpfs_pinned);
    b.c.cli("set zcpfs passive");
    CHECK(b.login(""));                         // a fresh empty-password legacy login
    CHECK(!b.s.client(b.c)->zcpfs_pinned);
}

TEST(zcpfs_mesh_advert_mark_cap, "UNIT-ZCPFSM-018",
     "The advert mark is capped at the receiver's clock: a sender's clock running ahead does not freeze its later adverts") {
    Pair p;
    const uint32_t ahead = 10 * 86400;
    p.b.rtc.base += ahead;                      // bob's clock is ten days ahead
    uint32_t future = p.b.rtc.getCurrentTime();
    CHECK(p.b.dm(p.a, "from the future"));
    p.air.run(5000);
    CHECK(p.a.texts.size() == 1);
    CHECK(p.a.contact(p.b)->last_advert_timestamp <= p.a.rtc.getCurrentTime());
    p.b.rtc.base -= ahead;                      // corrected
    p.b.cli("set zcpfs passive");
    p.b.advert();                               // untagged, signed at the right time
    p.air.run(3000);
    CHECK(p.a.contact(p.b)->last_advert_timestamp < future - ZCPFS_ADVERT_MARK_SLACK);
    CHECK(!pinned(p.a, p.b));                   // accepted
}
