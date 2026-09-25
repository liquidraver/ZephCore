#include "test.h"
#include "hash_capture.h"
#include <mesh/Packet.h>
#include <array>
#include <algorithm>
using namespace mesh;

TEST(path_encodings, "UNIT-PACKET-001", "Exhaust all path encodings") {
    for (unsigned encoded = 0; encoded < 256; ++encoded) {
        unsigned width = 1 + encoded / 64, hops = encoded % 64;
        CHECK(Packet::isValidPathLen(encoded) == (width <= 3 && width * hops <= 64));
    }
}
TEST(empty_paths, "UNIT-PACKET-002", "Zero-hop hash-width regression and bounded copying") {
    uint8_t source[64] = {}, output[66];
    std::fill(std::begin(output), std::end(output), 0xCC);
    for (uint8_t encoded : {0x00, 0x40, 0x80}) {
        CHECK(Packet::copyPath(output + 1, source, 0, encoded) == encoded);
    }
    CHECK(Packet::copyPath(output + 1, source, 0, 0xC0) == 0);
    CHECK(Packet::writePath(output + 1, source, 3, 0x42) == 0);
    CHECK(std::all_of(std::begin(output), std::end(output), [](auto b) { return b == 0xCC; }));
    CHECK(Packet::writePath(output + 1, source, 4, 0x42) == 4);
    CHECK(output[0] == 0xCC && output[5] == 0xCC);
}
TEST(golden_wire, "UNIT-PACKET-003", "Independent transport-direct wire vector") {
    // Header = TXT_MSG + TRANSPORT_DIRECT; two little-endian transport codes,
    // two 2-byte path hashes, two payload bytes. Authored independently of writeTo.
    const uint8_t wire[] = {0x0B, 0x34, 0x12, 0xCD, 0xAB, 0x42, 1, 2, 3, 4, 0xAA, 0xBB};
    Packet p;
    CHECK(p.readFrom(wire, sizeof(wire)));
    CHECK(p.header == 0x0B && p.transport_codes[0] == 0x1234 && p.transport_codes[1] == 0xABCD);
    CHECK(p.getPathHashSize() == 2 && p.getPathHashCount() == 2 && p.payload_len == 2);
    CHECK(p.path[3] == 4 && p.payload[0] == 0xAA && p.payload[1] == 0xBB);
    uint8_t encoded[256] = {};
    CHECK(p.writeTo(encoded) == sizeof(wire));
    CHECK(std::equal(std::begin(wire), std::end(wire), encoded));
}
TEST(header_matrix, "UNIT-PACKET-004", "All header bytes and payload boundaries") {
    for (unsigned header = 0; header < 256; ++header) {
        for (unsigned payload : {0u, 1u, 183u, 184u, 185u}) {
            unsigned prefix = ((header & 3) == 0 || (header & 3) == 3) ? 6 : 2;
            std::vector<uint8_t> wire(prefix + payload, 0);
            wire[0] = header;
            Packet p;
            CHECK(p.readFrom(wire.data(), wire.size()) == (payload >= 1 && payload <= 184));
            if (payload >= 1 && payload <= 184) {
                CHECK(p.getRouteType() == (header & 3));
                CHECK(p.getPayloadType() == ((header >> 2) & 15));
                CHECK(p.getPayloadVer() == (header >> 6));
                CHECK(p.payload_len == payload);
            }
        }
    }
}
TEST(truncation, "UNIT-PACKET-005", "Reject truncated headers and paths") {
    const uint8_t wire[] = {0x0B, 0, 0, 0, 0, 0x42, 1, 2, 3, 4, 0xAA};
    for (unsigned length = 0; length < sizeof(wire); ++length) {
        std::vector<uint8_t> input(wire, wire + length); // exact input extent for ASan
        Packet p;
        CHECK(!p.readFrom(input.data(), length));
    }
    Packet p;
    CHECK(p.readFrom(wire, sizeof(wire)));
}
TEST(generated_roundtrip, "UNIT-PACKET-006", "Seeded valid packets preserve wire fields") {
    uint32_t seed = 0x5EED1234;
    for (int i = 0; i < 2000; ++i) {
        Packet p;
        p.header = nextRandom(seed) & 255;
        unsigned width = 1 + nextRandom(seed) % 3;
        unsigned maxHops = std::min(63u, 64u / width);
        p.setPathHashSizeAndCount(width, nextRandom(seed) % (maxHops + 1));
        p.transport_codes[0] = nextRandom(seed); p.transport_codes[1] = nextRandom(seed);
        p.payload_len = 1 + nextRandom(seed) % 184;
        for (auto& b : p.path) b = nextRandom(seed) >> 24;
        for (auto& b : p.payload) b = nextRandom(seed) >> 24;
        uint8_t wire[257]; std::fill(std::begin(wire), std::end(wire), 0xCC);
        auto size = p.writeTo(wire + 1);
        CHECK(size == p.getRawLength());
        CHECK(wire[0] == 0xCC && wire[size + 1] == 0xCC);
        Packet decoded;
        CHECK(decoded.readFrom(wire + 1, size));
        CHECK(decoded.header == p.header && decoded.path_len == p.path_len);
        CHECK(decoded.payload_len == p.payload_len);
        CHECK(std::equal(p.path, p.path + p.getPathByteLen(), decoded.path));
        CHECK(std::equal(p.payload, p.payload + p.payload_len, decoded.payload));
        CHECK(decoded.transport_codes[0] == (p.hasTransportCodes() ? p.transport_codes[0] : 0));
        CHECK(decoded.transport_codes[1] == (p.hasTransportCodes() ? p.transport_codes[1] : 0));
    }
}
TEST(arbitrary_packets, "UNIT-PACKET-007", "Seeded arbitrary input parser invariants") {
    uint32_t seed = 0xBADCAFE;
    for (unsigned len = 0; len <= 255; ++len) for (int n = 0; n < 32; ++n) {
        std::vector<uint8_t> input(len);
        for (auto& b : input) b = nextRandom(seed) >> 24;
        Packet p;
        if (p.readFrom(input.data(), len)) {
            CHECK(p.getPathHashSize() <= 3 && p.getPathByteLen() <= 64);
            CHECK(p.payload_len >= 1 && p.payload_len <= 184);
            CHECK(p.getRawLength() == int(len));
        }
    }
}
TEST(hash_preimage, "UNIT-PACKET-008", "Ordinary and TRACE hash input contracts (not crypto)") {
    Packet p; p.header = (PAYLOAD_TYPE_TXT_MSG << 2) | ROUTE_TYPE_FLOOD;
    p.path_len = 1; p.path[0] = 7; p.payload_len = 2; p.payload[0] = 0xAA; p.payload[1] = 0xBB;
    uint8_t hash[8]; p.calculatePacketHash(hash);
    CHECK(test_support::hash_input == std::vector<uint8_t>({2, 0xAA, 0xBB}));
    CHECK(test_support::hash_output_size == 8);
    p.path_len = 2; p.path[1] = 8; p.calculatePacketHash(hash);
    CHECK(test_support::hash_input == std::vector<uint8_t>({2, 0xAA, 0xBB}));
    p.header = (PAYLOAD_TYPE_TRACE << 2) | ROUTE_TYPE_DIRECT; p.calculatePacketHash(hash);
    CHECK(test_support::hash_input == std::vector<uint8_t>({9, 2, 0xAA, 0xBB}));
}
