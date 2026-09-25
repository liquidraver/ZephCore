#pragma once
#include <cstdint>
// The time-sync target uses only uptime, not a simulated Zephyr scheduler.
extern "C" int64_t k_uptime_get();
