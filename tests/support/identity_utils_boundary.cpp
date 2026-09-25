#include <mesh/Utils.h>
#include <stdexcept>
#include <cstring>

// Identity tests exercise fromSeed, sign/verify, ECDH and storage serialization.
// Other constructors/private-key recovery need Utils.cpp/PSA integration later.
// Fail loudly if a new test accidentally reaches those unimplemented boundaries.
// Hex helpers are pure; copied from src/Utils.cpp for the prefs.json tests
// (ConfigSerializer stores binary blobs as hex).
static uint8_t hexVal(char c) {
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= '0' && c <= '9') return c - '0';
    return 0;
}
bool mesh::Utils::fromHex(uint8_t *dest, int dest_size, const char *src_hex) {
    size_t len = strlen(src_hex);
    if (len != (size_t)(dest_size * 2)) return false;
    uint8_t *dp = dest;
    while ((size_t)(dp - dest) < (size_t)dest_size) {
        char ch = *src_hex++;
        char cl = *src_hex++;
        *dp++ = (uint8_t)((hexVal(ch) << 4) | hexVal(cl));
    }
    return true;
}
void mesh::Utils::printHex(Stream &s, const uint8_t *src, size_t len) {
    static const char hex_chars[] = "0123456789ABCDEF";
    while (len > 0) {
        uint8_t b = *src++;
        s.print(hex_chars[b >> 4]);
        s.print(hex_chars[b & 0x0F]);
        len--;
    }
}
void mesh::Utils::secureZeroize(void*, size_t) {
    throw std::runtime_error("Identity RNG/recovery path requires real Utils integration");
}
bool mesh::Utils::constantTimeEqual(const void*, const void*, size_t) {
    throw std::runtime_error("Identity validation requires real Utils integration");
}
