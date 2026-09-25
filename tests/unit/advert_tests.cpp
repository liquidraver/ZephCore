#include "test.h"
#include <AdvertDataHelpers.h>
#include <UTF8Helpers.h>
#include <algorithm>
#include <cstring>

TEST(advert_golden, "UNIT-ADVERT-001", "Independent advertisement vector") {
    const uint8_t golden[] = {0xB1, 0x40, 0x42, 0x0F, 0, 0x80, 0x7B, 0xE1, 0xFF, 0x34, 0x12, 'A'};
    AdvertDataBuilder builder(ADV_TYPE_CHAT, "A", 1.0, -2.0); builder.setFeat1(0x1234);
    uint8_t bytes[32] = {}; auto len = builder.encodeTo(bytes);
    CHECK(len == sizeof(golden) && std::equal(std::begin(golden), std::end(golden), bytes));
    AdvertDataParser parser(golden, sizeof(golden));
    CHECK(parser.isValid() && parser.getType() == ADV_TYPE_CHAT);
    CHECK(parser.getIntLat() == 1000000 && parser.getIntLon() == -2000000);
    CHECK(parser.getFeat1() == 0x1234 && std::strcmp(parser.getName(), "A") == 0);
}
TEST(advert_truncation, "UNIT-ADVERT-002", "Optional fields reject truncation and names stay bounded") {
    for (unsigned flags = 0; flags < 8; ++flags) {
        unsigned required = 1 + ((flags & 1) ? 8 : 0) + ((flags & 2) ? 2 : 0) + ((flags & 4) ? 2 : 0);
        for (unsigned len = 0; len <= required; ++len) {
            std::vector<uint8_t> input(len, 0); if (len) input[0] = flags << 4;
            AdvertDataParser parser(input.data(), len);
            CHECK(parser.isValid() == (len == required));
        }
    }
    std::vector<uint8_t> oversized(255, 'X'); oversized[0] = ADV_NAME_MASK;
    AdvertDataParser parser(oversized.data(), 255);
    CHECK(parser.isValid() && std::strlen(parser.getName()) == 31);
}
TEST(utf8_boundaries, "UNIT-UTF8-001", "Complete codepoints only and invalid UTF-8 rejection") {
    const char* text = "A\xC3\xA9\xE2\x82\xAC\xF0\x9F\x98\x80";
    const unsigned expected[] = {0, 1, 1, 3, 3, 3, 6, 6, 6, 6, 10};
    for (unsigned limit = 0; limit <= 10; ++limit) CHECK(mesh::validUtf8PrefixLength(text, limit) == expected[limit]);
    for (const char* invalid : {"\x80", "\xC0\xAF", "\xED\xA0\x80", "\xF4\x90\x80\x80", "\xE2\x82"})
        CHECK(mesh::validUtf8PrefixLength(invalid, 32) == 0);
    CHECK(mesh::validUtf8PrefixLength(nullptr, 32) == 0);
    std::string name(30, 'A'); name += "\xC3\xA9";
    AdvertDataBuilder builder(ADV_TYPE_CHAT, name.c_str()); uint8_t buffer[34];
    std::fill(std::begin(buffer), std::end(buffer), 0xCC);
    CHECK(builder.encodeTo(buffer + 1) == 31);
    CHECK(buffer[0] == 0xCC && buffer[32] == 0xCC && buffer[33] == 0xCC);
}
