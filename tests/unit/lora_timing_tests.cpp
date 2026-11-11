#include "test.h"
#include <zephyr/drivers/lora/zc_lora_timing.h>
#include <lr11xx_cad_peak.h>
#include <cmath>
#include "NodePrefs.h"

TEST(lora_grace, "UNIT-LORATIME-001", "Preamble grace is (preamble+8) symbols rounded up with an unconfigured default") {
    CHECK(zc_lora_preamble_grace_ms(7, 62500, 32) == 82);      // 40 * 2^7 / 62.5 kHz = 81.92 ms
    CHECK(zc_lora_preamble_grace_ms(12, 125000, 16) == 787);   // 24 * 2^12 / 125 kHz = 786.4 ms
    CHECK(zc_lora_preamble_grace_ms(7, 500000, 8) == 5);       // 4.096 ms rounds up
    CHECK(zc_lora_preamble_grace_ms(7, 0, 32) == 1000);
    CHECK(zc_lora_preamble_grace_ms(4, 125000, 32) == 1000);
    CHECK(zc_lora_preamble_grace_ms(13, 125000, 32) == 1000);
}
TEST(lora_max_payload, "UNIT-LORATIME-002", "Max-payload bound is 255 B at CR 4/8 with LDRO plus 25% and 100 ms") {
    CHECK(zc_lora_max_payload_ms(7, 125000) == 1165);          // 832 symbols: 852 ms + 213 + 100
    CHECK(zc_lora_max_payload_ms(12, 7812) == 272747);         // longest reachable preset, > 262 s
    CHECK(zc_lora_max_payload_ms(0, 125000) == 30000 && zc_lora_max_payload_ms(7, 0) == 30000);
    uint32_t prev = 0;
    for (uint8_t sf = 5; sf <= 12; ++sf) {                     // monotonic in SF at a fixed bandwidth
        uint32_t ms = zc_lora_max_payload_ms(sf, 62500);
        CHECK(ms > prev); prev = ms;
    }
    CHECK(zc_lora_max_payload_ms(9, 62500) > zc_lora_max_payload_ms(9, 125000));
}
TEST(lora_steps24, "UNIT-LORATIME-003", "Chip timer conversion saturates at 24 bits instead of wrapping") {
    CHECK(zc_lora_ms_to_steps24(1000, 32768) == 32768);
    CHECK(zc_lora_ms_to_steps24(262143, 64000) == 16777152);   // just inside the SX126x field
    CHECK(zc_lora_ms_to_steps24(262144, 64000) == 0x00FFFFFFu);
    CHECK(zc_lora_ms_to_steps24(272747, 64000) == 0x00FFFFFFu); // SF12/BW7.81 bound: masked it read ~11 s
    CHECK(zc_lora_ms_to_steps24(600000, 32768) == 0x00FFFFFFu);
    CHECK(zc_lora_ms_to_steps24(0xFFFFFFFFu, 64000) == 0x00FFFFFFu); // no 32-bit overflow on the way
    CHECK(zc_lora_ms_to_steps24(0, 32768) == 0);
}
TEST(lora_ldro, "UNIT-LORATIME-005", "LDRO follows the true symbol time on every SF and bandwidth") {
    // True bandwidths as exact fractions: 500 kHz / 2^n, and 125 kHz / (3 * 2^m).
    // The chip time 1e6 / BW is then a whole number of us, and Tsym = 2^SF chips.
    struct { uint16_t khz; uint32_t us_per_chip; } bws[] = {
        {500, 2}, {250, 4}, {125, 8}, {62, 16}, {31, 32}, {15, 64}, {7, 128},
        {41, 24}, {20, 48}, {10, 96},
    };
    for (auto &b : bws) {
        uint32_t hz = zc_lora_bw_hz(b.khz);
        // BW * chip time = 1e6 exactly; the table may be off by rounding to 1 Hz, not to 10 Hz.
        CHECK((uint64_t)hz * b.us_per_chip >= 999900u && (uint64_t)hz * b.us_per_chip <= 1000100u);
        for (uint8_t sf = 5; sf <= 12; ++sf) {
            bool want = ((uint32_t)b.us_per_chip << sf) > 16380u;
            CHECK(zc_lora_ldro(sf, hz) == want);
        }
    }
    // The pairs that sit exactly on 16.384 ms; SF8/BW15.6 read 16378 us with 15630 Hz.
    CHECK(zc_lora_ldro(7, zc_lora_bw_hz(7)) && zc_lora_ldro(8, zc_lora_bw_hz(15)));
    CHECK(zc_lora_ldro(9, zc_lora_bw_hz(31)) && zc_lora_ldro(10, zc_lora_bw_hz(62)));
    CHECK(zc_lora_ldro(11, zc_lora_bw_hz(125)) && zc_lora_ldro(12, zc_lora_bw_hz(250)));
    CHECK(!zc_lora_ldro(8, 15630));                            // the rounded figure, for the record
    CHECK(!zc_lora_ldro(7, zc_lora_bw_hz(15)) && !zc_lora_ldro(10, zc_lora_bw_hz(125)));
    CHECK(!zc_lora_ldro(12, zc_lora_bw_hz(500)) && !zc_lora_ldro(12, zc_lora_bw_hz(1000)));
    CHECK(zc_lora_bw_hz(200) == 203000 && zc_lora_bw_hz(400) == 406000 && zc_lora_bw_hz(800) == 812000);
    CHECK(zc_lora_bw_hz(13) == 0 && !zc_lora_ldro(8, 0) && !zc_lora_ldro(13, 125000));
}
TEST(radio_bw_valid, "UNIT-LORATIME-006", "Only LoRa bandwidths pass the radio bandwidth check") {
    for (float ok : {7.8f, 7.81f, 10.4f, 10.42f, 15.6f, 15.63f, 20.8f, 31.25f, 41.7f, 41.67f, 62.5f, 125.0f, 250.0f, 500.0f})
        CHECK(zcRadioBwValid(ok));
    // In range but not a LoRa bandwidth: these used to be accepted and run at 125 kHz.
    for (float bad : {8.0f, 16.0f, 21.0f, 42.0f, 63.0f, 100.0f, 126.0f, 200.0f, 499.0f})
        CHECK(!zcRadioBwValid(bad));
    for (float out : {0.0f, -62.5f, 6.9f, 501.0f, 1000.0f, NAN})
        CHECK(!zcRadioBwValid(out));
}
TEST(lr11xx_cad_peak, "UNIT-LORATIME-004", "LR11xx detPeak base table cells symbol adjustment and clamp headroom") {
    CHECK(lr11xx_cad_detect_peak(7, 62, 4) == 44);             // default preset, sub-125 row
    CHECK(lr11xx_cad_detect_peak(8, 125, 4) == 57);
    CHECK(lr11xx_cad_detect_peak(5, 500, 8) == 63);
    CHECK(lr11xx_cad_detect_peak(12, 250, 2) == 75);           // under 4 symbols: no adjustment
    CHECK(lr11xx_cad_detect_peak(3, 125, 2) == lr11xx_cad_detect_peak(9, 125, 2)); // out of range -> SF9
    for (uint8_t sf = 5; sf <= 12; ++sf)
        for (uint16_t bw : {31, 62, 125, 250, 500})
            for (uint8_t symb : {1, 2, 4, 8, 16}) {
                uint8_t p = lr11xx_cad_detect_peak(sf, bw, symb);
                CHECK(p >= LR11XX_CAD_PEAK_MIN && p <= LR11XX_CAD_PEAK_MAX);
            }
}
