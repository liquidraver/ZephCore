// helpers/compat/CayenneLPP.h: upstream's wire format, rounded to nearest.
#include "test.h"
#include "CayenneLPP.h"

#include <vector>

namespace {
std::vector<uint8_t> bytes(CayenneLPP &l) {
    return std::vector<uint8_t>(l.getBuffer(), l.getBuffer() + l.getSize());
}
}

TEST(lpp_rounding, "UNIT-LPP-001", "Values round to the nearest step (3850 mV is 3.85 V, not the library's 3.84)") {
    CayenneLPP l(64);
    CHECK(l.addVoltage(1, 3850 / 1000.0f) == 4);
    CHECK(bytes(l) == std::vector<uint8_t>({1, 116, 0x01, 0x81}));   // 385
    l.reset();
    l.addTemperature(2, 21.96f);                                      // 220, not 219
    CHECK(bytes(l) == std::vector<uint8_t>({2, 103, 0x00, 0xDC}));
    l.reset();
    l.addRelativeHumidity(2, 45.3f);                                  // 90.6 -> 91
    CHECK(bytes(l) == std::vector<uint8_t>({2, 104, 91}));
}

TEST(lpp_signed, "UNIT-LPP-002", "Negative values are two's complement in the field, rounded on the magnitude") {
    CayenneLPP l(64);
    l.addTemperature(1, -2.56f);                                      // -25.6 -> -26
    CHECK(bytes(l) == std::vector<uint8_t>({1, 103, 0xFF, 0xE6}));
    l.reset();
    l.addCurrent(3, -0.0125f);                                        // -12.5 mA -> -13
    CHECK(bytes(l) == std::vector<uint8_t>({3, 117, 0xFF, 0xF3}));
    l.reset();
    l.addAltitude(2, -3.4f);
    CHECK(bytes(l) == std::vector<uint8_t>({2, 121, 0xFF, 0xFD}));
}

TEST(lpp_gps, "UNIT-LPP-003", "GPS: 0.0001 deg and 0.01 m, 3 bytes each, rounded") {
    CayenneLPP l(64);
    CHECK(l.addGPS(1, 47.49791f, -19.04023f, 123.456f) == 11);
    // 474979, -190402, 12346
    CHECK(bytes(l) == std::vector<uint8_t>({1, 136, 0x07, 0x3F, 0x63, 0xFD, 0x18, 0x3E, 0x00, 0x30, 0x3A}));
}

TEST(lpp_overflow, "UNIT-LPP-004", "A field that does not fit is refused whole; the buffer keeps what fit") {
    CayenneLPP l(7);
    CHECK(l.addVoltage(1, 4.0f) == 4);
    CHECK(l.addVoltage(1, 4.0f) == 0);
    CHECK(l.getSize() == 4);
    CHECK(l.addRelativeHumidity(1, 50.0f) == 7);
}
