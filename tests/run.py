#!/usr/bin/env python3
"""Build the host unit tests and run each case in its own process."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
# Every test executable tests/CMakeLists.txt may define; those not built are skipped.
EXECUTABLES = ("zephcore_unit", "zephcore_zcpfs_mesh")


def quiet(cmd):
    """Run a build step; show its output only when it fails."""
    proc = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    if proc.returncode:
        print(proc.stdout + proc.stderr, file=sys.stderr)
        raise subprocess.CalledProcessError(proc.returncode, cmd)


def build(sanitize):
    out = ROOT / "build-tests" / ("sanitize" if sanitize else "host")
    if not (out / "CMakeCache.txt").is_file():
        quiet(["cmake", "-S", ROOT / "tests", "-B", out, "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Debug",
               f"-DZEPHCORE_TEST_SANITIZERS={'ON' if sanitize else 'OFF'}"])
    quiet(["cmake", "--build", out])
    exe = ".exe" if os.name == "nt" else ""
    return [b for b in (out / (name + exe) for name in EXECUTABLES) if b.is_file()]


def run_case(binary, case_id):
    # One process per case: the static packet pool is global state.
    try:
        proc = subprocess.run([binary, "--case", case_id], capture_output=True, text=True,
                              errors="replace", timeout=10)
    except subprocess.TimeoutExpired:
        return case_id, False, "timeout after 10 s"
    ok = proc.returncode == 0 and proc.stdout.strip() == f"PASS {case_id}"
    return case_id, ok, proc.stdout + proc.stderr


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("list", "run"))
    parser.add_argument("--case", help="run only this case ID")
    parser.add_argument("--sanitize", action="store_true", help="ASan + UBSan (Linux)")
    args = parser.parse_args(argv)

    os.environ.setdefault("ASAN_OPTIONS", "detect_leaks=1:halt_on_error=1")
    os.environ.setdefault("UBSAN_OPTIONS", "halt_on_error=1:print_stacktrace=1")
    try:
        binaries = build(args.sanitize)
    except subprocess.CalledProcessError:
        return 1
    cases = {}  # id -> (binary, name)
    for binary in binaries:
        listing = subprocess.run([binary, "--list"], capture_output=True, text=True, check=True).stdout
        for line in listing.splitlines():
            case_id, name = line.split("\t")
            if case_id in cases:
                print(f"duplicate case {case_id}", file=sys.stderr)
                return 2
            cases[case_id] = (binary, name)
    if args.action == "list":
        for case_id, (_, name) in cases.items():
            print(f"{case_id}  {name}")
        return 0

    ids = [i for i in cases if args.case in (None, i)]
    if not ids:
        print(f"no case {args.case!r}", file=sys.stderr)
        return 2
    with ThreadPoolExecutor(os.cpu_count()) as pool:
        results = list(pool.map(lambda i: run_case(cases[i][0], i), ids))
    failed = [(i, out) for i, ok, out in results if not ok]
    for case_id, out in failed:
        print(f"FAIL {case_id}\n{out}")
    print(f"{len(ids) - len(failed)}/{len(ids)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
