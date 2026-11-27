# Host unit tests

Production C++ sources from `zephcore/` built for the host and run without boards, Zephyr or west. Linux or
WSL (MinGW cannot link them); needs Python 3.10+, CMake, Ninja and GCC or Clang.

```sh
python3 tests/run.py run                       # build, then run every case
python3 tests/run.py run --sanitize            # ASan + UBSan, what CI runs
python3 tests/run.py run --case UNIT-PACKET-002
python3 tests/run.py list
```

Builds are incremental in `build-tests/host` and `build-tests/sanitize`; delete them to start clean. Each case
runs in its own process (the static packet pool is global state) with a 10 s limit, and passes only if it exits
0 and prints `PASS <id>`. Build output is shown only on a build failure.

## Adding a case

Add `TEST(function_name, "UNIT-AREA-NNN", "description")` to a `unit/*_tests.cpp` file (new files go into
`CMakeLists.txt`). The binary refuses duplicate IDs. Use `CHECK`, not `assert`, and independent expected
values: never copy the production algorithm into the test or loosen an expectation to match a defect.

## Boundaries

- `support/zephyr/`: logging, kernel and random headers reduced to what the sources include.
- `support/hash_capture.cpp`: SHA-256 records its input and returns marker bytes, so packet hashing is tested
  for the bytes it hashes, not for cryptography. `Utils.cpp` is not linked.
- `support/identity_utils_boundary.cpp`: the Utils entry points Identity needs for hex, RNG and key recovery
  throw if reached. Signing, verification, ECDH and storage run the real Identity and Monocypher.
- The Dispatcher runs over a scripted `Radio`, a fake millisecond clock and the real packet pool.
- `support/extract_sx126x_helpers.py` pulls the Heltec helpers out of `patches/zephyr/0003-lora-sx126x-native.patch`
  so they run against fake SPI.
