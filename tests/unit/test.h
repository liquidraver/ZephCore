#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

struct TestCase { const char *id; const char *name; void (*run)(); };
inline std::vector<TestCase>& testCases() { static std::vector<TestCase> cases; return cases; }
struct RegisterTest {
    RegisterTest(const char *id, const char *name, void (*run)()) { testCases().push_back({id, name, run}); }
};
#define TEST(symbol, id, name) static void symbol(); \
    static RegisterTest registration_##symbol(id, name, symbol); static void symbol()
#define CHECK(expression) do { if (!(expression)) \
    throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #expression); } while (0)

// Specified generator: reproducible across C++ standard library implementations.
inline uint32_t nextRandom(uint32_t& state) { state = state * 1664525u + 1013904223u; return state; }
