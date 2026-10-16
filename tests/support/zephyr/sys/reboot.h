#pragma once
// Host boundary: a reboot ends the test run, loudly.
#include <cstdlib>
#define SYS_REBOOT_COLD 1
inline void sys_reboot(int) { std::abort(); }
