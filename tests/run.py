#!/usr/bin/env python3
"""Dependency-free host regression runner. No board discovery or flashing."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import unittest
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"


def command(args, *, timeout=30, env=None):
    """Bounded child execution, preserving failure and timeout output."""
    started = time.monotonic()
    try:
        proc = subprocess.run([str(arg) for arg in args], cwd=ROOT, env=env,
                              capture_output=True, text=True, encoding="utf-8",
                              errors="replace", timeout=timeout)
        return {"status": "PASS" if proc.returncode == 0 else "FAIL",
                "returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr,
                "seconds": time.monotonic() - started}
    except subprocess.TimeoutExpired as exc:
        def decode(value):
            return value.decode("utf-8", "replace") if isinstance(value, bytes) else (value or "")
        return {"status": "ERROR", "returncode": None, "stdout": decode(exc.stdout),
                "stderr": decode(exc.stderr) + "\nTimeout expired", "seconds": time.monotonic() - started}
    except OSError as exc:
        return {"status": "ERROR", "returncode": None, "stdout": "", "stderr": str(exc),
                "seconds": time.monotonic() - started}


def load_catalog(path=TESTS / "catalog.json"):
    data = json.loads(path.read_text(encoding="utf-8"))
    cases = data["cases"]
    if data.get("schema_version") != 1 or not cases:
        raise ValueError("Unsupported or empty catalog")
    ids = set()
    for case in cases:
        if not all(isinstance(case.get(key), str) and case[key] for key in ("id", "name", "source")):
            raise ValueError("Invalid case fields")
        if case["id"] in ids:
            raise ValueError(f"Duplicate case ID: {case['id']}")
        ids.add(case["id"])
    return cases


def select_cases(cases, requested):
    selected = cases if requested is None else [case for case in cases if case["id"] == requested]
    if not selected:
        raise ValueError(f"No implemented cases match {requested!r}")
    return selected


def validate_listing(output, cases):
    actual = {}
    for line in output.splitlines():
        parts = line.split("\t")
        if len(parts) != 2 or not all(parts) or parts[0] in actual:
            raise ValueError("Malformed or duplicate executable case listing")
        actual[parts[0]] = parts[1]
    expected = {case["id"]: case["name"] for case in cases}
    if not actual or actual != expected:
        raise ValueError(f"Executable/catalog mismatch: missing={sorted(expected.keys() - actual.keys())}, "
                         f"unexpected={sorted(actual.keys() - expected.keys())}; also check case names")


def execute_case(binary, case, env=None, timeout=10):
    result = command([binary, "--case", case["id"]], timeout=timeout, env=env)
    # Exit 0 alone is insufficient: reject stale/malformed/incomplete output.
    if result["status"] == "PASS" and result["stdout"].strip() != f"PASS {case['id']}":
        result["status"] = "ERROR"
        result["stderr"] += "\nMissing or invalid per-case success marker"
    return {"id": case["id"], "name": case["name"], **result}


def write_reports(directory, report):
    results = report["results"]
    report["counts"] = {status: sum(r["status"] == status for r in results)
                        for status in ("PASS", "FAIL", "ERROR", "BLOCKED")}
    (directory / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    suite = ET.Element("testsuite", name="zephcore.quick", tests=str(len(results)),
                       failures=str(report["counts"]["FAIL"]),
                       errors=str(report["counts"]["ERROR"] + report["counts"]["BLOCKED"]))
    for result in results:
        case = ET.SubElement(suite, "testcase", classname=result["id"], name=result["name"],
                             time=str(result.get("seconds", 0)))
        if result["status"] != "PASS":
            ET.SubElement(case, "failure" if result["status"] == "FAIL" else "error",
                          message=result["status"]).text = result.get("stderr", "")
        ET.SubElement(case, "system-out").text = result.get("stdout", "")
        ET.SubElement(case, "system-err").text = result.get("stderr", "")
    ET.ElementTree(suite).write(directory / "junit.xml", encoding="utf-8", xml_declaration=True)


class HarnessResult(unittest.TextTestResult):
    def startTest(self, test):
        self.started = time.monotonic()
        super().startTest(test)

    def record(self, test, status, detail=""):
        self.records.append({"id": test.id(), "name": test.shortDescription() or test.id(),
                             "status": status, "stderr": detail,
                             "seconds": time.monotonic() - self.started})

    def addSuccess(self, test):
        super().addSuccess(test)
        self.record(test, "PASS")

    def addFailure(self, test, err):
        super().addFailure(test, err)
        self.record(test, "FAIL", self._exc_info_to_string(err, test))

    def addError(self, test, err):
        super().addError(test, err)
        self.record(test, "ERROR", self._exc_info_to_string(err, test))

    def addSkip(self, test, reason):
        super().addSkip(test, reason)
        self.record(test, "BLOCKED", reason)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.records = []


def harness_tests(directory):
    suite = unittest.defaultTestLoader.discover(str(TESTS / "harness_tests"))
    if suite.countTestCases() == 0:
        raise ValueError("Harness test collection is empty")
    with (directory / "harness.log").open("w", encoding="utf-8") as stream:
        result = unittest.TextTestRunner(stream=stream, verbosity=2, resultclass=HarnessResult).run(suite)
    return result.records


def tool_environment():
    env = os.environ.copy()
    compiler = env.get("CXX")
    if not compiler:
        compiler = shutil.which("g++") or shutil.which("clang++")
    if not compiler and os.name == "nt":
        for path in ("C:/msys64/ucrt64/bin/g++.exe", "C:/msys64/mingw64/bin/g++.exe",
                     "C:/msys64/clang64/bin/clang++.exe"):
            if Path(path).is_file():
                compiler = path
                break
    if not compiler:
        raise ValueError("No host C++ compiler found. Set CXX to g++/clang++ (full path allowed).")
    resolved = shutil.which(compiler) or str(Path(compiler).resolve())
    env["PATH"] = str(Path(resolved).parent) + os.pathsep + env.get("PATH", "")
    env.setdefault("ASAN_OPTIONS", "detect_leaks=1:halt_on_error=1")
    env.setdefault("UBSAN_OPTIONS", "halt_on_error=1:print_stacktrace=1")
    return resolved, env


def metadata():
    def git(*args):
        result = command(["git", *args])
        return result["stdout"].strip() if result["status"] == "PASS" else None
    # Include untracked new test sources too. Runtime/build outputs live elsewhere.
    paths = list(TESTS.rglob("*")) + list((ROOT / "zephcore/include").rglob("*.h"))
    paths += list((ROOT / "zephcore/helpers").glob("*.h"))
    paths += list((ROOT / "zephcore/lib/monocypher").glob("*"))
    paths += [ROOT / "zephcore" / p for p in (
        "src/Packet.cpp", "src/StaticPoolPacketManager.cpp", "src/ContentionTracker.cpp",
        "src/Dispatcher.cpp", "src/Identity.cpp", "helpers/AdvertDataHelpers.cpp")]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in paths if p.is_file() and (p.suffix in (".py", ".cpp", ".c", ".h", ".json") or p.name == "CMakeLists.txt")}
    return {"git_commit": git("rev-parse", "HEAD"), "git_status": git("status", "--porcelain"),
            "tracked_diff_sha256": hashlib.sha256((git("diff", "HEAD", "--binary") or "").encode()).hexdigest(),
            "python": sys.version, "host": platform.platform(), "input_sha256": hashes,
            "fixed_generator_seeds": ["0x5EED1234", "0x0BADCAFE", "0x12345678", "0xD15CA7C4"],
            "limitations": ["No hardware or Zephyr kernel execution", "SHA-256 replaced by input capture",
                            "Identity hex/RNG/validation/recovery Utils boundaries are fail-fast stubs",
                            "Only implemented catalog cases; full plan remains incomplete"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("list", "run"))
    parser.add_argument("--profile", choices=("quick",), default="quick")
    parser.add_argument("--case", help="Exact native case ID")
    parser.add_argument("--sanitize", action="store_true", help="Linux ASan + UBSan")
    args = parser.parse_args(argv)
    try:
        cases = load_catalog()
        selected = select_cases(cases, args.case)
    except (ValueError, KeyError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if args.action == "list":
        for case in selected:
            print(f"{case['id']}  {case['name']}")
        print(f"{len(selected)} native cases; run also executes harness self-tests.")
        return 0

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    directory = ROOT / "test-results" / run_id
    directory.mkdir(parents=True)
    report = {"schema_version": 1, "run_id": run_id, "profile": args.profile,
              "selected_native_ids": [c["id"] for c in selected], "results": [],
              "sanitizers_requested": args.sanitize, "commands": []}
    binary = None
    try:
        report["metadata"] = metadata()
        report["results"].extend(harness_tests(directory))
        if any(r["status"] != "PASS" for r in report["results"]):
            raise ValueError("Harness self-tests failed; firmware tests blocked")
        compiler, env = tool_environment()
        version = command([compiler, "--version"], env=env)
        if version["status"] != "PASS":
            raise ValueError("Cannot query compiler: " + version["stderr"])
        report["compiler"] = version["stdout"]
        report["sanitizer_environment"] = {key: env[key] for key in ("ASAN_OPTIONS", "UBSAN_OPTIONS")}
        key = hashlib.sha256((compiler + version["stdout"] + str(args.sanitize)).encode()).hexdigest()[:12]
        # Unique per run: simultaneous invocations cannot race on build artifacts.
        build = ROOT / "build-tests" / f"{key}-{run_id}"
        configure = ["cmake", "-S", TESTS, "-B", build, "-G", "Ninja",
                     f"-DCMAKE_CXX_COMPILER={compiler}", "-DCMAKE_BUILD_TYPE=Debug",
                     f"-DZEPHCORE_TEST_SANITIZERS={'ON' if args.sanitize else 'OFF'}"]
        for phase, cmd in (("configure", configure), ("build", ["cmake", "--build", build, "--parallel", "2"])):
            print(f"{phase.capitalize()} host tests...", flush=True)
            report["commands"].append([str(part) for part in cmd])
            result = command(cmd, timeout=180, env=env)
            (directory / f"{phase}.log").write_text(result["stdout"] + result["stderr"], encoding="utf-8")
            if result["status"] != "PASS":
                raise ValueError(f"{phase} failed; see {directory / (phase + '.log')}")
        binary = build / ("zephcore_unit.exe" if os.name == "nt" else "zephcore_unit")
        report["binary_sha256"] = hashlib.sha256(binary.read_bytes()).hexdigest()
        listing = command([binary, "--list"], env=env)
        if listing["status"] != "PASS":
            raise ValueError("Executable discovery failed: " + listing["stderr"])
        validate_listing(listing["stdout"], cases)
        for case in selected:
            result = execute_case(binary, case, env)
            report["results"].append(result)
            (directory / (case["id"] + ".log")).write_text(result["stdout"] + result["stderr"], encoding="utf-8")
            print(f"{result['status']:5} {case['id']} {case['name']}", flush=True)
    except (ValueError, OSError, KeyboardInterrupt) as exc:
        report["results"].append({"id": "RUNNER", "name": "Run infrastructure", "status": "ERROR", "stderr": str(exc) or "Interrupted"})
    finally:
        completed = {r["id"] for r in report["results"]}
        for case in selected:
            if case["id"] not in completed:
                report["results"].append({"id": case["id"], "name": case["name"], "status": "BLOCKED",
                                          "stderr": "Prerequisite failed or run interrupted"})
        write_reports(directory, report)
    print(f"Reports: {directory}\nCounts: {report['counts']}")
    return 0 if all(r["status"] == "PASS" for r in report["results"]) else 1


if __name__ == "__main__":
    sys.exit(main())
