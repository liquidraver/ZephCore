#pragma once
#include <cerrno>
#include <cstdint>
// Uptime only, not a simulated Zephyr scheduler: the time-sync target uses
// k_uptime_get(), the KISS modem k_uptime_get_32() and k_msleep(); each test
// file that needs one defines it over its own clock.
extern "C" int64_t k_uptime_get();
extern "C" uint32_t k_uptime_get_32();
extern "C" int32_t k_msleep(int32_t ms);
