#pragma once
// Host boundary: printk writes to stdout.
#include <cstdio>
#define printk(...) std::printf(__VA_ARGS__)
