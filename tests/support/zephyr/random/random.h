#pragma once
#include <cstdint>
// Deterministic boundary for Dispatcher default CAD retry jitter. Not entropy.
inline uint32_t sys_rand32_get() { return 17; }
