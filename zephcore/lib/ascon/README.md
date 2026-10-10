# ascon

A small, self-contained C99 implementation of Ascon as standardised in
[NIST SP 800-232](https://doi.org/10.6028/NIST.SP.800-232) (August 2025):

| Function | Use |
|---|---|
| Ascon-AEAD128 | authenticated encryption, 16-byte key and nonce, tag of 16 bytes or truncated down to 4 |
| Ascon-Hash256 | 32-byte hash |
| Ascon-XOF128 | hash with output of any length |
| Ascon-CXOF128 | the same with a customization string of up to 256 bytes |

Two files, `ascon.h` and `ascon.c`. They depend on nothing else in this repository and on nothing outside the C
library but `memcpy`. MIT licence (the repository's `LICENSE`); take the two files and use them anywhere.

Written from the text of SP 800-232, not derived from another implementation. zcpfs (`../zcpfs`) is its user
here.

## Properties

- C99, no heap, no global state, no lookup tables. Compiles without warnings under
  `-Wall -Wextra -Wpedantic -Wconversion -Wshadow`.
- Any byte order and word size: the state is five `uint64_t` and bytes are loaded and stored one at a time.
- No branch and no memory index depends on a key, a plaintext or a tag. Tags are compared without an early exit.
  This is a property of the source; no compiled output has been measured for timing.
- AEAD works in place (`ct == pt`). A failed decryption zeroes the plaintext buffer.
- The hash functions are incremental: absorb and squeeze in any number of pieces.
- Sizes on a 32-bit microcontroller at `-Os`: about 2.6 kB of code on Cortex-M4, no data; the XOF state is 56
  bytes, the AEAD uses about 160 bytes of stack.

## What it is not

- **Not fast.** It is plain 64-bit C. On a 32-bit core the permutation takes roughly twice as long as the
  assembly-tuned variants of the designers' [ascon-c](https://github.com/ascon/ascon-c). If throughput matters,
  use that.
- **Not hardened** against power or fault attacks; there is no masking.
- **Not reviewed** by anyone but its author.
- No streaming AEAD (one call per message), no Ascon-MAC or PRF, none of the pre-standard variants
  (Ascon-128, Ascon-128a, Ascon-80pq, Ascon-Hash).

## Use

```c
#include "ascon.h"

uint8_t key[16], nonce[16];      /* a nonce is used once per key, ever */
uint8_t ct[sizeof msg], tag[16];

ascon_aead128_encrypt(key, nonce, ad, ad_len, msg, sizeof msg, ct, tag);
if (ascon_aead128_decrypt(key, nonce, ad, ad_len, ct, sizeof ct, tag, 16, out) != 0) {
	/* forged or damaged: out is all zero */
}

struct ascon_xof x;
ascon_cxof128_init(&x, (const uint8_t *)"my-app/v1", 9);
ascon_xof_absorb(&x, input, input_len);
ascon_xof_squeeze(&x, okm, 64);
ascon_xof_wipe(&x);
```

A truncated tag is the leftmost bytes of the full one; pass its length to `ascon_aead128_decrypt`. SP 800-232
allows tags from 32 bits and asks for a limit on failed decryptions per key when the tag is shorter than 128
bits (its requirements R4 and R5); keeping that limit is the caller's job.

## Verification

- `tests/unit/ascon_tests.cpp` (cases `UNIT-ASCON-001` to `-003`, in the repository's quick profile, also under
  ASan and UBSan) checks subsets of the official known-answer files kept in `kat/`, plus tag truncation, every
  tag byte flipped, wrong key, nonce and associated data, in-place operation, and absorbing and squeezing in
  uneven pieces.
- On 2026-10-10 the complete official files passed: 1089 AEAD128, 1025 Hash256, 1025 XOF128 and 1089 CXOF128
  records.
- The official files stop at messages of 32 bytes (1024 for the hashes). On the same day the code was compared
  with the ascon-c reference implementation on 200,000 random cases per function with inputs and outputs up to
  600 bytes, including one forged ciphertext per AEAD case, checked with an 8-byte tag. All agreed.

The last two were one-off runs outside the repository's tests.

`kat/*.txt` are excerpts of the known-answer files of ascon-c (commit
`446347f21b209f3921c65ece70027c366cbe1693`), which are published under CC0. `kat/gen_kat_header.py` turns them
into the C tables the tests include:

```bash
python zephcore/lib/ascon/kat/gen_kat_header.py --check
```
