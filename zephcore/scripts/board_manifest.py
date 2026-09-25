#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""ZephCore board manifests: the one list of boards and what we publish for them.

Every board directory boards/<platform>/<board>/ carries a zephcore.yml; each
native-Linux preset boards/linux_native/<preset>.conf carries a sibling
<preset>.zephcore.yml. Schema (all keys optional unless marked):

    target: rak4631                  # REQUIRED  west -b string, with qualifiers
    capabilities:
      wifi: true                     # WiFi companion: RAM for WiFi + BLE together
      light_sleep: true              # DIO1 on an RTC-wake-capable GPIO AND
                                     # validated on hardware (ESP32 repeaters)
    release:                         # absent = not published (bring-up)
      roles: [companion, repeater]   # REQUIRED in release
      variants:                      # extra published builds of this board
        - id: noscreen               # artifact infix
          conf: no_display.conf      # relative to the board dir
          subtitle: No screen        # catalog role-row subtitle
    catalog:                         # absent = published but not in the catalog
      device: RAK WisBlock / WisMesh (RAK 4631)   # REQUIRED in catalog
      maker: rak                     # REQUIRED in catalog, key of MAKERS
      new: true                      # own tile instead of folding into MeshCore's
      img: lora.svg | own_img: x.jpg
      subtitle: 30 dBm
    linux:                           # native-Linux presets only
      host: arm | aarch64
      cross_compile: /usr/bin/arm-linux-gnueabihf-

Readers:
  - cmake/zephcore_board.cmake reads the capabilities with a line match (it
    runs before Zephyr has set up Python), so `check` enforces the literal
    two-space-indented `  <capability>: true|false` form.
  - build.sh:  board_manifest.py matrix <nrf|nrf54l|mg24|stm32wl|esp32|linux> [companions|repeaters]
  - gen_provider_catalog.py imports load_boards().
  - docs:      board_manifest.py docs  (rewrites the generated lists in docs/supported_boards.md)

`board_manifest.py check` validates every manifest; CI should run it.
"""
import os
import re
import sys

import yaml

APP_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
BOARDS_DIR = os.path.join(APP_DIR, "boards")

PLATFORM_DIRS = {"nrf52840": "nrf52", "nrf54l": "nrf54l", "esp32": "esp32",
                 "mg24": "mg24", "stm32wl": "stm32wl"}
ROLES = ("companion", "repeater", "room_server", "observer")
KEYS = {"target", "capabilities", "release", "catalog", "linux"}
CAPS = {"light_sleep", "wifi"}
CATALOG_KEYS = {"device", "maker", "new", "img", "own_img", "subtitle"}
CAP_LINE = re.compile(r"^  (\w+): (true|false)\s*$", re.M)


class Board:
    def __init__(self, name, platform, manifest_path, data, board_dir=None, conf=None):
        self.name = name                    # board dir name or linux preset name
        self.platform = platform            # nrf52 | nrf54l | esp32 | mg24 | stm32wl | linux
        self.manifest_path = manifest_path
        self.board_dir = board_dir          # None for linux presets
        self.conf = conf                    # linux preset conf, relative to APP_DIR
        self.target = data.get("target")
        self.capabilities = data.get("capabilities") or {}
        self.release = data.get("release")
        self.catalog = data.get("catalog")
        self.linux = data.get("linux") or {}

    @property
    def chip(self):
        """ESP32 chip name from the target (esp32, esp32s3, esp32c3, esp32c6)."""
        m = re.search(r"(esp32[^/]*)", self.target or "")
        return m.group(1) if m else None

    @property
    def stem(self):
        """Artifact filename stem, exactly as build.sh has always named files."""
        if self.platform == "linux":
            return f"zephcore_linux_{self.name}"
        return self.target.replace("/", "-")

    @property
    def sysbuild(self):
        """--sysbuild / --no-sysbuild / None (west default) for release builds."""
        if self.platform == "esp32":
            return True if self.chip != "esp32" else None   # classic ESP32: simple boot
        if self.platform == "nrf54l":
            return False
        return None


def _load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_boards():
    boards = []
    for plat_dir, platform in PLATFORM_DIRS.items():
        root = os.path.join(BOARDS_DIR, plat_dir)
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            bdir = os.path.join(root, name)
            path = os.path.join(bdir, "zephcore.yml")
            if os.path.isfile(path):
                boards.append(Board(name, platform, path, _load(path), board_dir=bdir))
    lroot = os.path.join(BOARDS_DIR, "linux_native")
    if os.path.isdir(lroot):
        for fn in sorted(os.listdir(lroot)):
            if fn.endswith(".zephcore.yml"):
                name = fn[: -len(".zephcore.yml")]
                boards.append(Board(name, "linux", os.path.join(lroot, fn),
                                    _load(os.path.join(lroot, fn)),
                                    conf=f"boards/linux_native/{name}.conf"))
    return boards


def check(boards, makers=None):
    errors = []
    for b in boards:
        where = os.path.relpath(b.manifest_path, APP_DIR)
        data = _load(b.manifest_path)
        for k in set(data) - KEYS:
            errors.append(f"{where}: unknown key '{k}'")
        if not b.target:
            errors.append(f"{where}: missing target")
        elif b.platform != "linux" and not re.match(rf"^{re.escape(b.name)}(@|/|$)", b.target):
            errors.append(f"{where}: target '{b.target}' does not name board '{b.name}'")
        for k, v in b.capabilities.items():
            if k not in CAPS:
                errors.append(f"{where}: unknown capability '{k}'")
            if not isinstance(v, bool):
                errors.append(f"{where}: capability '{k}' must be true/false")
        if b.capabilities:
            text = open(b.manifest_path, encoding="utf-8").read()
            block = re.search(r"^capabilities:\n((?:  .*\n?)*)", text, re.M)
            found = {m.group(1) for m in CAP_LINE.finditer(block.group(1))} if block else set()
            if found != set(b.capabilities):
                errors.append(f"{where}: write capabilities as two-space-indented "
                              f"'  <name>: true|false' lines (CMake reads them by line)")
        if b.release is not None:
            roles = b.release.get("roles") or []
            if not roles or any(r not in ROLES for r in roles):
                errors.append(f"{where}: release.roles must be a non-empty subset of {ROLES}")
            for v in b.release.get("variants") or []:
                if not v.get("id") or not v.get("conf"):
                    errors.append(f"{where}: variant needs id and conf")
                elif b.board_dir and not os.path.isfile(os.path.join(b.board_dir, v["conf"])):
                    errors.append(f"{where}: variant conf '{v['conf']}' not found")
        if b.catalog is not None:
            for k in set(b.catalog) - CATALOG_KEYS:
                errors.append(f"{where}: unknown catalog key '{k}'")
            if not b.catalog.get("device") or not b.catalog.get("maker"):
                errors.append(f"{where}: catalog needs device and maker")
            if makers is not None and b.catalog.get("maker") not in makers:
                errors.append(f"{where}: catalog maker '{b.catalog.get('maker')}' not in MAKERS")
            if b.release is None:
                errors.append(f"{where}: catalog entry without a release")
        if b.platform == "linux" and not {"host", "cross_compile"} <= set(b.linux):
            errors.append(f"{where}: linux presets need linux.host and linux.cross_compile")
    return errors


def matrix(boards, group, role_filter=None):
    """Release builds for one build.sh group: target|role|variant|confs|stem|sysbuild|host|cross."""
    want = {"nrf": "nrf52"}.get(group, group)
    if role_filter:
        role_filter = {"companions": "companion", "repeaters": "repeater"}[role_filter]
    rows = []
    for b in boards:
        if b.platform != want or b.release is None:
            continue
        sb = {True: "sysbuild", False: "no-sysbuild", None: ""}[b.sysbuild]
        base_confs = [b.conf] if b.conf else []
        builds = [("", base_confs)]
        for v in b.release.get("variants") or []:
            rel = os.path.relpath(os.path.join(b.board_dir, v["conf"]), APP_DIR).replace(os.sep, "/")
            builds.append((v["id"], base_confs + [rel]))
        for role in b.release["roles"]:
            if role_filter and role != role_filter:
                continue
            for variant, confs in builds:
                c = list(confs)
                if role != "companion":
                    # Role conf before the variant conf, as build.sh always ordered it.
                    c = ([c[0]] if b.conf else []) + [f"boards/common/{role}.conf"] + c[1 if b.conf else 0:]
                rows.append("|".join([b.target, role, variant, ";".join(c), b.stem, sb,
                                      b.linux.get("host", ""), b.linux.get("cross_compile", "")]))
    return rows


DOCS_GROUPS = [("nrf52", "nRF52840"), ("esp32", "ESP32"), ("stm32wl", "STM32WL"),
               ("mg24", "MG24"), ("nrf54l", "nRF54L")]


def docs(boards, path):
    """Rewrite each '<!-- boards:<platform> -->' ... '<!-- /boards -->' block."""
    text = open(path, encoding="utf-8").read()
    for platform, _ in DOCS_GROUPS:
        group = [b for b in boards if b.platform == platform]
        width = max((len(b.target) for b in group), default=0)
        lines = [b.target if b.release is not None
                 else f"{b.target:<{width}}   # source-only, no published firmware"
                 for b in group]
        block = "<!-- boards:%s -->\n```\n%s\n```\n<!-- /boards -->" % (platform, "\n".join(lines))
        text = re.sub(r"<!-- boards:%s -->.*?<!-- /boards -->" % platform, lambda _m: block,
                      text, flags=re.S)
    open(path, "w", encoding="utf-8", newline="\n").write(text)


def main(argv):
    boards = load_boards()
    cmd = argv[1] if len(argv) > 1 else "check"
    if cmd == "check":
        errs = check(boards)
        for e in errs:
            print(e, file=sys.stderr)
        print(f"{len(boards)} manifests, {len(errs)} errors")
        return 1 if errs else 0
    if cmd == "matrix":
        for row in matrix(boards, argv[2], argv[3] if len(argv) > 3 else None):
            print(row)
        return 0
    if cmd == "docs":
        docs(boards, argv[2] if len(argv) > 2 else os.path.join(APP_DIR, "..", "docs", "supported_boards.md"))
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
