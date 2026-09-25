#include "test.h"
#include <zephyr/drivers/lora/zc_lora_timing.h>
#include <lr11xx_cad_peak.h>

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
