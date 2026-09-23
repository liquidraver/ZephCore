#!/usr/bin/env python3
"""Generate the Mesh America configurator provider catalog for ZephCore.

The Mesh America Device Configurator (apps.meshamerica.com) merges third-party
"provider catalogs" with the official MeshCore catalog. This script emits our
catalog.json from a release's firmware assets, following the provider API spec
(the `version -> {notes, files:[{type,name,url}]}` shape with absolute HTTPS
URLs -- NOT MeshCore's internal `github`+regex+staticPath mechanism).

Hosting note: GitHub *Release assets* do NOT return Access-Control-Allow-Origin
(they redirect to release-assets.githubusercontent.com, which omits the header),
so the browser-based configurator cannot fetch them. Firmware + this catalog are
instead published to a force-pushed `firmware-dist` orphan branch and served from
CORS-clean hosts: firmware via jsDelivr (`--url-base`), the catalog itself via
raw.githubusercontent.com. `--url-base` is prepended to each firmware filename.

Usage (CI, scanning the built firmware/ dir):
    python gen_provider_catalog.py --firmware-dir firmware \
        --url-base "https://cdn.jsdelivr.net/gh/liquidraver/ZephCore@firmware-dist" \
        --version v20260704.220543 --out catalog.json

Usage (local test, from a GitHub release API json):
    python gen_provider_catalog.py --assets-json rel.json \
        --url-base "https://cdn.jsdelivr.net/gh/liquidraver/ZephCore@firmware-dist" \
        --version v20260704.220543 --out catalog.json
"""

import argparse
import io
import json
import os
import re
import sys

# ---------------------------------------------------------------------------
# Provider identity
# ---------------------------------------------------------------------------

DEVICE_CLASS = "zephcore"

# `maker` is the DEVICE MANUFACTURER (not the firmware provider — the ZephCore
# provider badge comes from Mesh America's registration, independent of this).
# Folded devices use MeshCore's exact maker key so they group under the same
# brand; new tiles use the hardware maker. Display names for the maker map:
MAKERS = {
    "rak": "RAK Wireless",
    "seeed": "Seeed Studio",
    "elecrow": "Elecrow",
    "lilygo": "LilyGo",
    "heltec": "Heltec",
    "promicro": "ProMicro",
    "gat-iot": "GAT-IoT",
    "uniteng": "UnitEng",
    "Ikoka": "Ikoka",
    "femtofox": "Femtofox",
    "muziworks": "muzi works",
    "minewsemi": "MinewSemi",
    "meshnology": "Meshnology",
}

DESCRIPTION = (
    "ZephCore is an independent, ground-up implementation of the MeshCore "
    "protocol on the Zephyr RTOS. It is wire-compatible with stock MeshCore "
    "and adds features such as an adaptive contention window, LoRa RX "
    "duty-cycling, and support for additional boards. Companion (BLE + USB), "
    "Repeater, and native-Linux builds are provided."
)

# Device thumbnails for our NEW tiles only (folded devices inherit MeshCore's on
# merge). Reused from the configurator's own image set via jsDelivr -- a stable,
# CORS-clean absolute URL. `<img>` display isn't CORS-restricted, but the spec
# asks for absolute URLs. Boards without dedicated art use the neutral lora icon.
# An `img` starting with http is taken as a full URL and used as-is (vendor
# product shots), instead of being resolved against IMG_BASE.
IMG_BASE = "https://cdn.jsdelivr.net/gh/meshcore-dev/flasher.meshcore.io@main/img"

# Art we ship ourselves, for devices the configurator has no picture of. Source
# of truth is this repo's img/ dir; build.sh copies it into firmware/ so it is
# published alongside the firmware, and the URL is resolved against --url-base
# like every other asset -- so nothing here hardcodes a host. Vendor product
# shots are copied in rather than hot-linked: a vendor CDN path is theirs to
# reshuffle, and a tile whose picture 404s is worse than one that never had it.
# Set `own_img` (a bare filename) instead of `img` to use one.

# A single custom role for the native-Linux companion, which speaks TCP rather
# than BLE/USB. Custom roles render with a plain label and never map to the
# in-app Configure step -- that is fine for these download-only entries.
CUSTOM_ROLES = {
    "companionTcp": {
        "icon": "lan",
        "title": "Companion",
        "subTitle": "TCP",
        "tooltip": "Native-Linux companion radio -- connect over TCP.",
    }
}

# ---------------------------------------------------------------------------
# Board -> device mapping, from the board manifests
#
# Every published board has a `catalog:` section in its manifest
# (zephcore/boards/<platform>/<board>/zephcore.yml, Linux presets
# zephcore/boards/linux_native/<preset>.zephcore.yml; schema in
# zephcore/scripts/board_manifest.py). Each entry below carries:
# `stem`   : the exact filename stem build.sh emits (target with / -> -).
# `kind`   : nrf | esp32 | nrf54l | mg24 | stm32wl | linux -- drives device
#            type, file types, extensions.
# `device` : the MeshCore-canonical device name to FOLD into, or a new tile name.
#            Folding is by exact name, so a `new` tile named to MeshCore's
#            convention merges by itself the day MeshCore adds that device.
# `maker`  : key of MAKERS (the hardware manufacturer).
# `new`    : True if this device is not in the official catalog (its own tile).
# `img`    : filename in MeshCore's own image set (resolved against IMG_BASE).
# `own_img`: filename we ship from this repo's img/ dir (resolved against
#            --url-base). Takes precedence over `img`.
# `variant`: optional filename infix (e.g. "noscreen") + shown as subTitle.
# `subtitle`: optional role-row subTitle to disambiguate hardware variants.
# `softdevice`: nRF52 only, 6 or 7, from the board's CONFIG_ZEPHCORE_SD_FWID.
# ---------------------------------------------------------------------------

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "zephcore", "scripts"))
import board_manifest  # noqa: E402

KIND = {"nrf52": "nrf", "esp32": "esp32", "nrf54l": "nrf54l",
        "mg24": "mg24", "stm32wl": "stm32wl", "linux": "linux"}
KIND_ORDER = ["nrf", "esp32", "nrf54l", "mg24", "stm32wl", "linux"]
SD_FWID = {0x00B6: 6, 0x0123: 7}   # S140 v6.1.1 / v7.3.0


def softdevice_of(board):
    text = open(os.path.join(board.board_dir, "board.conf"), encoding="utf-8").read()
    m = re.search(r"^CONFIG_ZEPHCORE_SD_FWID=(0x[0-9A-Fa-f]+)", text, re.M)
    if not m or int(m.group(1), 16) not in SD_FWID:
        raise SystemExit(f"{board.name}: no known CONFIG_ZEPHCORE_SD_FWID in board.conf")
    return SD_FWID[int(m.group(1), 16)]


def load_catalog_boards():
    """Catalog entries in tile order: platform group, folded before new, then name."""
    out = []
    for b in board_manifest.load_boards():
        if b.release is None or b.catalog is None:
            continue
        c = b.catalog
        base = dict(stem=b.stem, kind=KIND[b.platform], device=c["device"], maker=c["maker"])
        for k in ("new", "img", "own_img", "subtitle"):
            if c.get(k):
                base[k] = c[k]
        if b.platform == "nrf52":
            base["softdevice"] = softdevice_of(b)
        out.append(base)
        for v in b.release.get("variants") or []:
            e = dict(base, variant=v["id"])
            e.pop("subtitle", None)
            if v.get("subtitle"):
                e["subtitle"] = v["subtitle"]
            out.append(e)
    out.sort(key=lambda e: (KIND_ORDER.index(e["kind"]), bool(e.get("new")), e["device"].lower(),
                            e.get("variant") or ""))
    return out


DEVICE_TYPE = {"nrf": "nrf52", "esp32": "esp32", "linux": "noflash",
               "nrf54l": "noflash", "mg24": "noflash", "stm32wl": "noflash"}

# nRF52 erase package (spec §4a). ZephCore's LittleFS layout differs from MeshCore's,
# so the official erase would wipe the wrong region — we point `erase` at ZephCore's
# own formatter tool instead. The formatter is SoftDevice-specific (partition map
# differs between SD v6 and v7), so each board maps to its SD's formatter .zip.
# Files are published to firmware-dist by build.sh (stable names, no hash).
FORMATTER_FILE = {6: "SoftDevice_v6_formatter.zip", 7: "SoftDevice_v7_formatter.zip"}

# Companion firmware is a single image that serves both transports (and, on
# Linux, TCP). Expose it under each applicable role, all pointing at one file.
COMPANION_ROLES = {
    "nrf":   ["companionBle", "companionUsb"],
    "esp32": ["companionBle", "companionUsb"],
    "linux": ["companionTcp"],
    # nRF54L15 has no USB peripheral -- the companion is BLE-only there.
    "nrf54l": ["companionBle"],
    # MG24 has BLE (Silabs controller blob) but no USB device peripheral.
    "mg24": ["companionBle"],
    # STM32WL has neither BLE nor USB: the companion is MeshCore serial framing
    # on USART1, which reaches the host as a plain serial port over the board's
    # USB-UART bridge.
    "stm32wl": ["companionUsb"],
}

HASH_RE = r"[0-9a-f]{7,40}"


def find_file(assets, stem, token, variant, suffix):
    """Return the single asset filename for (stem, role token, variant, suffix).

    `suffix` is the trailing part after the git hash, e.g. r"\\.zip",
    r"-merged\\.bin", or "" (Linux, no extension). Returns None if absent.
    """
    mid = f"-{variant}" if variant else ""
    pat = re.compile(rf"^{re.escape(stem)}-{token}{mid}-{HASH_RE}{suffix}$")
    matches = sorted(n for n in assets if pat.match(n))
    return matches[0] if matches else None


def files_for(assets, board, token):
    """Build the spec `files[]` list for one role of one board, or [] if none."""
    stem, kind, variant = board["stem"], board["kind"], board.get("variant")
    out = []
    if kind == "nrf":
        zip_ = find_file(assets, stem, token, variant, r"\.zip")
        uf2 = find_file(assets, stem, token, variant, r"\.uf2")
        if zip_:
            out.append(("flash", zip_, "DFU package (install or update over USB)"))
        if uf2:
            out.append(("download", uf2, "UF2 -- drag-and-drop onto the bootloader drive"))
    elif kind == "esp32":
        merged = find_file(assets, stem, token, variant, r"-merged\.bin")
        if merged:
            out.append(("flash-wipe", merged, "Full install (bootloader + firmware)"))
        # App-only update, written at 0x10000 over an existing MCUboot. Our slot0
        # is at 0x10000 (matches Mesh America's fixed flash-update offset), so this
        # is valid. Classic-ESP32 (simple boot, no MCUboot) has no -update.bin and
        # is naturally skipped by the presence check.
        update = find_file(assets, stem, token, variant, r"-update\.bin")
        if update:
            out.append(("flash-update", update, "Update (app only; keeps settings)"))
    elif kind == "nrf54l":
        hexf = find_file(assets, stem, token, variant, r"\.hex")
        if hexf:
            out.append(("download", hexf,
                        "Firmware image -- flash over SWD (no USB bootloader on this SoC)"))
    elif kind in ("mg24", "stm32wl"):
        hexf = find_file(assets, stem, token, variant, r"\.hex")
        if hexf:
            out.append(("download", hexf,
                        "Firmware image -- flash over SWD (no bootloader on this board)"))
    elif kind == "linux":
        elf = find_file(assets, stem, token, variant, r"")
        if elf:
            out.append(("download", elf, "Linux executable (download and run)"))
    return out


def build(assets, url_base, version):
    base = url_base.rstrip("/")
    devices = {}   # name -> device object (first-seen order preserved)
    order = []
    used_roles = set()
    used_makers = set()
    stats = {"fold": 0, "new": 0, "skipped": []}

    def spec_files(triples):
        return [
            {"type": t, "name": name, "url": f"{base}/{name}", "title": title}
            for (t, name, title) in triples
        ]

    def add_option(dev, role, subtitle, version, triples, erase=None):
        opt = {"role": role, "version": {version: {"notes": "", "files": spec_files(triples)}}}
        if subtitle:
            opt["subTitle"] = subtitle
        if erase:
            opt["erase"] = erase   # spec §4a: on the firmware option, not the device
        dev["firmware"].append(opt)
        used_roles.add(role)

    for board in load_catalog_boards():
        name = board["device"]
        subtitle = board.get("subtitle")
        comp = files_for(assets, board, "companion")
        repe = files_for(assets, board, "repeater")
        if not comp and not repe:
            stats["skipped"].append(f"{board['stem']}"
                                    + (f" ({board['variant']})" if board.get("variant") else ""))
            continue

        if name not in devices:
            maker = board["maker"]
            used_makers.add(maker)
            dev = {
                "maker": maker,
                "class": DEVICE_CLASS,
                "name": name,
                "type": DEVICE_TYPE[board["kind"]],
                "firmware": [],
            }
            if board.get("own_img"):
                dev["tooltip"] = f"<img class='device' src='{base}/{board['own_img']}'>"
            elif board.get("img"):
                dev["tooltip"] = f"<img class='device' src='{IMG_BASE}/{board['img']}'>"
            devices[name] = dev
            order.append(name)
            stats["new" if board.get("new") else "fold"] += 1
        dev = devices[name]

        # nRF52 boards get ZephCore's own SoftDevice-specific formatter as `erase`.
        erase = None
        if board["kind"] == "nrf":
            erase = f"{base}/{FORMATTER_FILE[board['softdevice']]}"

        if comp:
            for role in COMPANION_ROLES[board["kind"]]:
                add_option(dev, role, subtitle, version, comp, erase)
        if repe:
            add_option(dev, "repeater", subtitle, version, repe, erase)

    catalog = {
        "description": DESCRIPTION,
        "maker": {k: {"name": MAKERS[k]} for k in sorted(used_makers)},
        "device": [devices[n] for n in order],
    }
    role_map = {k: v for k, v in CUSTOM_ROLES.items() if k in used_roles}
    if role_map:
        catalog["role"] = role_map
    return catalog, stats


def validate(catalog):
    """Enforce the spec's hard requirements; return a list of error strings."""
    errs = []
    file_types = {"flash-wipe", "flash-update", "flash", "download"}
    dev_types = {"esp32", "nrf52", "noflash"}
    makers = set(catalog.get("maker", {}))
    for dev in catalog.get("device", []):
        tag = f"device {dev.get('name')!r}"
        if not dev.get("name"):
            errs.append("a device is missing 'name'")
        if dev.get("maker") not in makers:
            errs.append(f"{tag}: maker {dev.get('maker')!r} not in maker map")
        if dev.get("type") not in dev_types:
            errs.append(f"{tag}: bad type {dev.get('type')!r}")
        fw = dev.get("firmware", [])
        if not fw:
            errs.append(f"{tag}: no firmware options")
        for opt in fw:
            if not opt.get("role"):
                errs.append(f"{tag}: firmware option missing 'role'")
            ver = opt.get("version", {})
            if not ver:
                errs.append(f"{tag}: role {opt.get('role')} has no version")
            for vname, v in ver.items():
                files = v.get("files", [])
                if not files:
                    errs.append(f"{tag}: version {vname} has no files")
                for f in files:
                    if f.get("type") not in file_types:
                        errs.append(f"{tag}: file bad type {f.get('type')!r}")
                    if not f.get("name"):
                        errs.append(f"{tag}: a file is missing 'name'")
                    url = f.get("url", "")
                    if not url.startswith("https://"):
                        errs.append(f"{tag}: file url not absolute https: {url!r}")
    return errs


def load_assets(args):
    if args.firmware_dir:
        return [n for n in os.listdir(args.firmware_dir)
                if os.path.isfile(os.path.join(args.firmware_dir, n))]
    if args.assets_json:
        with io.open(args.assets_json, encoding="utf-8") as fh:
            data = json.load(fh)
        return [a["name"] for a in data.get("assets", [])]
    raise SystemExit("provide --firmware-dir or --assets-json")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--firmware-dir", help="directory of built firmware assets")
    ap.add_argument("--assets-json", help="GitHub release API json (for testing)")
    ap.add_argument("--url-base", required=True,
                    help="prepended to each firmware filename, e.g. "
                         "https://cdn.jsdelivr.net/gh/liquidraver/ZephCore@firmware-dist")
    ap.add_argument("--version", required=True,
                    help="version label shown in the role picker, e.g. the release tag")
    ap.add_argument("--out", default="catalog.json")
    args = ap.parse_args()

    assets = load_assets(args)
    catalog, stats = build(assets, args.url_base, args.version)
    errs = validate(catalog)

    devcount = len(catalog["device"])
    optcount = sum(len(d["firmware"]) for d in catalog["device"])
    print(f"assets seen: {len(assets)}")
    print(f"devices: {devcount}  (fold={stats['fold']}, new={stats['new']})")
    print(f"firmware options: {optcount}")
    if stats["skipped"]:
        print("skipped (no assets found): " + ", ".join(stats["skipped"]))
    if errs:
        print("\nVALIDATION ERRORS:", file=sys.stderr)
        for e in errs:
            print("  - " + e, file=sys.stderr)
        raise SystemExit(1)

    with io.open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(catalog, fh, indent=2, ensure_ascii=False)
        fh.write("\n")
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
