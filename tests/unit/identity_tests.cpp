#include "test.h"
#include <mesh/Identity.h>
#include <algorithm>
#include <array>

namespace {
// RFC 8032 section 7.1, tests 1 and 2 (public test keys, never device identities).
// https://www.rfc-editor.org/rfc/rfc8032.txt
struct Vector { const char *seed; const char *pub; const char *message; const char *signature; };
const Vector vectors[] = {
    {"9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60",
     "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a", "",
     "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e06522490155"
     "5fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b"},
    {"4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb",
     "3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c", "72",
     "92a009a9f0d4cab8720e820b5f642540a2b27b5416503f8fb3762223ebdb69da"
     "085ac1e43e15996e458f3613d0f11d8c387b2eaeb4302aeeb00d291612bb0c00"}
};
std::vector<uint8_t> hex(const char *text) {
    std::string value(text); CHECK(value.size() % 2 == 0);
    std::vector<uint8_t> result;
    for (size_t i = 0; i < value.size(); i += 2) result.push_back(std::stoul(value.substr(i, 2), nullptr, 16));
    return result;
}
mesh::LocalIdentity identity(unsigned index = 0) {
    mesh::LocalIdentity id; auto seed = hex(vectors[index].seed); id.fromSeed(seed.data()); return id;
}
}
TEST(identity_vectors, "UNIT-IDENTITY-001", "RFC8032 Ed25519 public keys and exact signatures") {
    for (const auto& vector : vectors) {
        auto seed = hex(vector.seed), pub = hex(vector.pub), msg = hex(vector.message), signature = hex(vector.signature);
        mesh::LocalIdentity id; id.fromSeed(seed.data());
        CHECK(std::equal(pub.begin(), pub.end(), id.pub_key));
        uint8_t output[64], empty = 0;
        auto data = msg.empty() ? &empty : msg.data();
        id.sign(output, data, int(msg.size()));
        CHECK(std::equal(signature.begin(), signature.end(), output));
        CHECK(id.verify(signature.data(), data, int(msg.size())));
    }
}
TEST(identity_tamper, "UNIT-IDENTITY-002", "Reject altered signature message and wrong signer") {
    auto id = identity(1); auto signature = hex(vectors[1].signature);
    uint8_t msg = 0x72;
    for (unsigned byte = 0; byte < 64; ++byte) {
        auto bad = signature; bad[byte] ^= 1; CHECK(!id.verify(bad.data(), &msg, 1));
    }
    msg ^= 1; CHECK(!id.verify(signature.data(), &msg, 1));
    msg = 0x72; CHECK(!identity(0).verify(signature.data(), &msg, 1));
}
TEST(identity_storage, "UNIT-IDENTITY-003", "Canonical legacy and private-only storage retain signing identity") {
    auto id = identity();
    uint8_t wire[96], storage[96], privateOnly[64];
    CHECK(id.writeTo(wire, sizeof(wire)) == 96 && id.writeToStorage(storage, sizeof(storage)) == 96);
    CHECK(id.writeTo(privateOnly, sizeof(privateOnly)) == 64);
    CHECK(std::equal(wire, wire + 64, storage + 32));
    CHECK(std::equal(wire + 64, wire + 96, storage));
    for (auto fixture : {std::vector<uint8_t>(wire, wire + 96), std::vector<uint8_t>(storage, storage + 96),
                         std::vector<uint8_t>(privateOnly, privateOnly + 64)}) {
        mesh::LocalIdentity restored; CHECK(restored.readFromStorage(fixture.data(), fixture.size()));
        CHECK(restored.matches(id));
        uint8_t sig[64], empty = 0; restored.sign(sig, &empty, 0);
        auto expected = hex(vectors[0].signature); CHECK(std::equal(expected.begin(), expected.end(), sig));
    }
}
TEST(identity_storage_reject, "UNIT-IDENTITY-004", "Malformed storage fails without replacing existing identity") {
    auto id = identity(), other = identity(1);
    uint8_t before[96], bad[96], after[96]; id.writeToStorage(before, 96); other.writeToStorage(bad, 96);
    bad[0] ^= 1;
    CHECK(!id.readFromStorage(bad, 96));
    for (size_t size : {0u, 31u, 63u, 65u, 95u}) CHECK(!id.readFromStorage(bad, size));
    id.writeToStorage(after, 96); CHECK(std::equal(before, before + 96, after));
    std::array<uint8_t, 98> output; output.fill(0xCC);
    CHECK(id.writeToStorage(output.data() + 1, 95) == 0);
    CHECK(std::all_of(output.begin(), output.end(), [](auto b) { return b == 0xCC; }));
    CHECK(id.writeToStorage(output.data() + 1, 96) == 96);
    CHECK(output.front() == 0xCC && output.back() == 0xCC);
}
TEST(identity_ecdh, "UNIT-IDENTITY-005", "Real Ed25519-to-X25519 exchange agrees in both directions") {
    auto alice = identity(), bob = identity(1);
    uint8_t ab[32], ba[32]; alice.calcSharedSecret(ab, bob); bob.calcSharedSecret(ba, alice);
    CHECK(std::equal(ab, ab + 32, ba));
    CHECK(std::any_of(ab, ab + 32, [](auto b) { return b != 0; }));
    // Pairwise property only: an independent ECDH known-answer fixture remains planned.
}
