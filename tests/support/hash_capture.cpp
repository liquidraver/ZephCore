#include <mesh/Utils.h>
#include "hash_capture.h"
#include <stdexcept>

namespace test_support {
std::vector<uint8_t> hash_input;
size_t hash_output_size = 0;
}

// Tests inspect Packet's hash preimage, NOT SHA-256 correctness. Deterministic
// marker output must never be mistaken for a digest or used in firmware builds.
void mesh::Utils::sha256(uint8_t *hash, size_t size, const uint8_t *msg, int len)
{
    if (len < 0 || size != MAX_HASH_SIZE) throw std::runtime_error("unexpected hash call");
    test_support::hash_input.assign(msg, msg + len);
    test_support::hash_output_size = size;
    memset(hash, 0xA5, size);
}
