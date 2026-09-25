#include "test.h"
#include <TxtDataHelpers.h>
#include <battery_curve.h>
#include <mesh/MeshCore.h>
#include <algorithm>
#include <cstring>

TEST(string_capacity, "UNIT-STRING-001", "String copies terminate and respect zero exact and short capacities") {
    for (size_t size = 0; size <= 10; ++size) {
        std::vector<char> out(size + 2, '#');
        StrHelper::strncpy(out.data() + 1, "abcd", size);
        CHECK(out.front() == '#' && out.back() == '#');
        if (size) CHECK(std::string(out.data() + 1) == std::string("abcd").substr(0, size - 1));
        std::fill(out.begin(), out.end(), '#');
        StrHelper::strzcpy(out.data() + 1, "abcd", size);
        CHECK(out.front() == '#' && out.back() == '#');
        if (size) {
            auto copied = std::min(size_t(4), size - 1);
            CHECK(std::string(out.data() + 1) == std::string("abcd").substr(0, copied));
            for (size_t i = copied; i < size; ++i) CHECK(out[i + 1] == 0);
        }
    }
}
TEST(string_normalization, "UNIT-STRING-002", "Decimal trimming and blank classification") {
    const char *input[] = {"250.000", "62.500", "0.0010", "-10.000", "1000", "", "1.25"};
    const char *expected[] = {"250", "62.5", "0.001", "-10", "1000", "", "1.25"};
    for (unsigned i = 0; i < 7; ++i) {
        char buffer[32]; std::strcpy(buffer, input[i]); StrHelper::stripTrailingZeros(buffer);
        CHECK(std::strcmp(buffer, expected[i]) == 0);
    }
    CHECK(StrHelper::isBlank(nullptr) && StrHelper::isBlank("") && StrHelper::isBlank(" \t\r\n"));
    CHECK(!StrHelper::isBlank(" a ") && !StrHelper::isBlank("0"));
}
TEST(rtc_unique, "UNIT-RTC-001", "Unique timestamps survive repeated and backward wall time") {
    struct RTC : mesh::RTCClock {
        uint32_t now = 100;
        uint32_t getCurrentTime() override { return now; }
        void setCurrentTime(uint32_t value) override { now = value; }
    } rtc;
    CHECK(rtc.getCurrentTimeUnique() == 100); CHECK(rtc.getCurrentTimeUnique() == 101);
    rtc.now = 50; CHECK(rtc.getCurrentTimeUnique() == 102);
    rtc.now = 200; CHECK(rtc.getCurrentTimeUnique() == 200);
}
TEST(battery_curve, "UNIT-BATTERY-001", "Battery endpoints interpolation multicell scaling and monotonicity") {
    CHECK(battery_curve_lookup(&battery_curve_default, 0) == 0);
    CHECK(battery_curve_lookup(&battery_curve_default, 3100) == 0);
    CHECK(battery_curve_lookup(&battery_curve_default, 4190) == 100);
    CHECK(battery_curve_lookup(&battery_curve_default, 3720) == 50);
    const uint16_t points[] = {4200, 3600, 3000};
    battery_curve_t twoCell{points, 3, 2};
    CHECK(battery_curve_lookup(&twoCell, 7800) == 75);
    CHECK(battery_curve_lookup(&twoCell, 6600) == 25);
    unsigned previous = 0;
    for (unsigned mv = 0; mv <= 65535; ++mv) {
        auto percent = battery_curve_lookup(&battery_curve_default, mv);
        CHECK(percent >= previous && percent <= 100); previous = percent;
    }
}
