#pragma once
// Host-only logging boundary. No kernel or driver behavior is emulated here.
#define LOG_MODULE_REGISTER(...)
#define LOG_MODULE_DECLARE(...)
namespace test_support {
template <typename... Args> inline void discardLog(const char*, Args...) {}
}
#define LOG_WRN(...) test_support::discardLog(__VA_ARGS__)
#define LOG_ERR(...) test_support::discardLog(__VA_ARGS__)
#define LOG_DBG(...) test_support::discardLog(__VA_ARGS__)
#define LOG_INF(...) test_support::discardLog(__VA_ARGS__)
// Only the explicitly supplied host configuration is supported.
#define IS_ENABLED(value) (value)
