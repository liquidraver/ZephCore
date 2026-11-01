#include "test.h"
#include "dispatcher_fixture.h"
#include <BaseChatMesh.h>
#include <cstring>

namespace {
struct Rng : mesh::RNG {
    void random(uint8_t *dest, size_t len) override { memset(dest, 0x42, len); }
};
struct Rtc : mesh::RTCClock {
    uint32_t now = 1700000000;
    uint32_t getCurrentTime() override { return now; }
    void setCurrentTime(uint32_t t) override { now = t; }
};
struct Tables : mesh::MeshTables {
    bool wasSeen(const mesh::Packet*) override { return false; }
    void markSeen(const mesh::Packet*) override {}
    void clear(const mesh::Packet*) override {}
};
struct ChatMesh : BaseChatMesh {
    ChatMesh(mesh::Radio& r, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc,
             mesh::PacketManager& mgr, mesh::MeshTables& t) : BaseChatMesh(r, ms, rng, rtc, mgr, t) {}
    void onDiscoveredContact(ContactInfo&, bool, uint8_t, const uint8_t*) override {}
    ContactInfo* processAck(const uint8_t*) override { return nullptr; }
    void onContactPathUpdated(const ContactInfo&) override {}
    void onMessageRecv(const ContactInfo&, mesh::Packet*, uint32_t, const char*) override {}
    void onCommandDataRecv(const ContactInfo&, mesh::Packet*, uint32_t, const char*) override {}
    void onCLICommandRecv(const ContactInfo&, mesh::Packet*, uint32_t, const char*, char*) override {}
    void onSignedMessageRecv(const ContactInfo&, mesh::Packet*, uint32_t, const uint8_t*, const char*) override {}
    uint32_t calcFloodTimeoutMillisFor(uint32_t) const override { return 1000; }
    uint32_t calcDirectTimeoutMillisFor(uint32_t, uint8_t) const override { return 1000; }
    void onSendTimeout() override {}
    void onChannelMessageRecv(const mesh::GroupChannel&, mesh::Packet*, uint32_t, const char*) override {}
    uint8_t onContactRequest(const ContactInfo&, uint32_t, const uint8_t*, uint8_t, uint8_t*) override { return 0; }
    void onContactResponse(const ContactInfo&, const uint8_t*, uint8_t) override {}
    using BaseChatMesh::searchPeersByHash;
};
struct ChatFixture {
    test_support::Clock clock;
    test_support::Radio radio;
    StaticPoolPacketManager pool;
    Rng rng;
    Rtc rtc;
    Tables tables;
    ChatMesh mesh{radio, clock, rng, rtc, pool, tables};
};
ContactInfo makeContact(uint8_t first_byte, uint8_t type, const char *name) {
    ContactInfo c{};
    for (int i = 0; i < PUB_KEY_SIZE; ++i) c.id.pub_key[i] = uint8_t(i + 1);
    c.id.pub_key[0] = first_byte;
    c.type = type;
    strncpy(c.name, name, sizeof(c.name) - 1);
    return c;
}
}  // namespace

TEST(empty_slot_skipped, "UNIT-CHAT-001", "An empty contact slot (all-zero key) matches no lookup") {
    ChatFixture f;
    ContactInfo empty{};                       // what a cleared anon slot holds
    empty.type = ADV_TYPE_NONE;
    CHECK(empty.isEmptySlot());
    CHECK(f.mesh.addContact(empty));           // occupies a slot inside num_contacts
    ContactInfo real = makeContact(0x00, ADV_TYPE_CHAT, "alice");
    CHECK(!real.isEmptySlot());
    CHECK(f.mesh.addContact(real));

    const uint8_t zero_hash[1] = {0x00};
    CHECK(f.mesh.searchPeersByHash(zero_hash) == 1);  // only the real contact with hash 0x00

    uint8_t zero_key[PUB_KEY_SIZE] = {0};
    CHECK(f.mesh.lookupContactByPubKey(zero_key, PUB_KEY_SIZE) == nullptr);
    CHECK(f.mesh.lookupContactByPubKey(zero_key, 1) != nullptr);
    CHECK(f.mesh.lookupContactByPubKey(zero_key, 1)->id.matches(real.id));

    ContactInfo *by_name = f.mesh.searchContactsByPrefix("");
    CHECK(by_name != nullptr && strcmp(by_name->name, "alice") == 0);

    CHECK(!f.mesh.removeContact(empty));       // the empty slot is not a removable match
    CHECK(f.mesh.removeContact(real));
    CHECK(f.mesh.searchPeersByHash(zero_hash) == 0);
}

TEST(group_msg_negative_len, "UNIT-CHAT-002", "sendGroupMessage refuses a negative text length") {
    ChatFixture f;
    mesh::GroupChannel ch{};
    CHECK(!f.mesh.sendGroupMessage(1700000000, ch, "me", "x", -4));
}
