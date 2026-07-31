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
}

# Manufacturer per device name.
MAKER_BY_DEVICE = {
    "RAK WisBlock / WisMesh (RAK 4631)": "rak",
    "RAK WisMesh 1W Booster (3401 + 13302)": "rak",
    "RAK WisMesh Tag": "rak",
    "Seeed Studio Wio Tracker L1 Pro": "seeed",
    "Seeed Studio SenseCAP T1000-E": "seeed",
    "Seeed Studio SenseCAP Solar": "seeed",
    "Seeed Studio Xiao nRF52 WIO": "seeed",
    "Seeed Studio Xiao C3": "seeed",
    "Seeed Studio Xiao S3 WIO": "seeed",
    "Seeed Studio Xiao ESP32-C6": "seeed",
    "Elecrow ThinkNode M1": "elecrow",
    "Elecrow ThinkNode M3": "elecrow",
    "Elecrow ThinkNode M6": "elecrow",
    "Elecrow ThinkNode M9": "elecrow",
    "Ikoka Nano": "Ikoka",
    "LilyGo T-Echo": "lilygo",
    "LilyGo T-Beam (SX1262)": "lilygo",
    "LilyGo T-Impulse Plus": "lilygo",
    "LilyGo T-Lora C6": "lilygo",
    "ProMicro nrf52 (faketec)": "promicro",
    "Heltec T114": "heltec",
    "Heltec Mesh Node T096": "heltec",
    "Heltec v3": "heltec",
    "Heltec v4": "heltec",
    "Heltec v4.3": "heltec",
    "Heltec Wireless Tracker": "heltec",
    "Heltec Wireless Tracker v2": "heltec",
    "GAT-IoT GAT562 30s": "gat-iot",
    "UnitEng Station G2": "uniteng",
    "Femtofox (Luckfox Pico Mini)": "femtofox",
    "RAK6421 WisMesh (Raspberry Pi)": "rak",
    "RAK6421 WisMesh (Raspberry Pi 5)": "rak",
    "muzi works R1 Neo": "muziworks",
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
IMG_BASE = "https://cdn.jsdelivr.net/gh/meshcore-dev/flasher.meshcore.io@main/img"

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
# Board -> device mapping
#
# `stem`   : the exact filename stem build.sh emits (board_clean_for_path).
# `kind`   : nrf | esp32 | linux -- drives device type, file types, extensions.
# `device` : the MeshCore-canonical device name to FOLD into, or a new tile name.
# `new`    : True if this device is not in the official catalog (its own tile).
# `variant`: optional filename infix (e.g. "noscreen") + shown as subTitle.
# `subtitle`: optional role-row subTitle to disambiguate hardware variants.
# ---------------------------------------------------------------------------

BOARDS = [
    # --- nRF52: fold into existing MeshCore devices -----------------------
    dict(stem="rak4631",          kind="nrf", device="RAK WisBlock / WisMesh (RAK 4631)"),
    dict(stem="rak3401_1watt",    kind="nrf", device="RAK WisMesh 1W Booster (3401 + 13302)"),
    dict(stem="wio_tracker_l1",   kind="nrf", device="Seeed Studio Wio Tracker L1 Pro"),
    dict(stem="t1000_e",          kind="nrf", device="Seeed Studio SenseCAP T1000-E"),
    dict(stem="thinknode_m1",     kind="nrf", device="Elecrow ThinkNode M1"),
    dict(stem="thinknode_m3",     kind="nrf", device="Elecrow ThinkNode M3"),
    dict(stem="thinknode_m6",     kind="nrf", device="Elecrow ThinkNode M6"),
    dict(stem="rak_wismesh_tag",  kind="nrf", device="RAK WisMesh Tag"),
    dict(stem="ikoka_nano_30dbm", kind="nrf", device="Ikoka Nano", subtitle="30 dBm"),
    dict(stem="sensecap_solar",   kind="nrf", device="Seeed Studio SenseCAP Solar"),
    dict(stem="xiao_nrf52840",    kind="nrf", device="Seeed Studio Xiao nRF52 WIO"),
    dict(stem="lilygo_techo",     kind="nrf", device="LilyGo T-Echo"),
    dict(stem="promicro_sx1262",  kind="nrf", device="ProMicro nrf52 (faketec)"),
    dict(stem="heltec_t114",      kind="nrf", device="Heltec T114"),
    dict(stem="heltec_t114",      kind="nrf", device="Heltec T114",
         variant="noscreen", subtitle="No screen"),
    dict(stem="gat562_30s",       kind="nrf", device="GAT-IoT GAT562 30s"),
    dict(stem="muziworks_r1neo",  kind="nrf", device="muzi works R1 Neo"),

    # --- nRF52: new ZephCore-only hardware (own tile) ---------------------
    dict(stem="lilygo_timpulse_plus", kind="nrf", device="LilyGo T-Impulse Plus", new=True, img="lora.svg"),
    dict(stem="heltec_t096",          kind="nrf", device="Heltec Mesh Node T096",  new=True, img="lora.svg"),

    # --- ESP32 (sysbuild/MCUboot, -merged.bin): fold ---------------------
    dict(stem="xiao_esp32c3",                      kind="esp32", device="Seeed Studio Xiao C3"),
    dict(stem="xiao_esp32s3-esp32s3-procpu",       kind="esp32", device="Seeed Studio Xiao S3 WIO"),
    dict(stem="station_g2-esp32s3-procpu",         kind="esp32", device="UnitEng Station G2"),
    dict(stem="heltec_wifi_lora32_v3-esp32s3-procpu", kind="esp32", device="Heltec v3"),
    dict(stem="heltec_wifi_lora32_v4-esp32s3-procpu", kind="esp32", device="Heltec v4"),
    dict(stem="heltec_wireless_tracker-esp32s3-procpu",    kind="esp32", device="Heltec Wireless Tracker"),
    dict(stem="heltec_wireless_tracker_v2-esp32s3-procpu", kind="esp32", device="Heltec Wireless Tracker v2"),
    # ThinkNode M9 pulled from the build.sh release matrix 2026-07-22 (bring-up
    # still in progress — keypad/GPS/battery unverified on hardware). Restore
    # this line together with the build.sh entry once it's release-ready.
    # dict(stem="thinknode_m9-esp32s3-procpu",             kind="esp32", device="Elecrow ThinkNode M9"),
    # Classic ESP32 T-Beam: ships -merged.bin (full-flash, 0x0) too.
    dict(stem="ttgo_tbeam-esp32-procpu",           kind="esp32", device="LilyGo T-Beam (SX1262)"),

    # --- ESP32: new ZephCore-only hardware (own tile) --------------------
    dict(stem="xiao_esp32c6-esp32c6-hpcore",   kind="esp32", device="Seeed Studio Xiao ESP32-C6", new=True, img="xiao_esp32c6.svg"),
    dict(stem="lilygo_tlora_c6-esp32c6-hpcore", kind="esp32", device="LilyGo T-Lora C6",           new=True, img="lilygo_tlora_c6.svg"),
    dict(stem="heltec_wifi_lora32_v43-esp32s3-procpu", kind="esp32", device="Heltec v4.3",         new=True, img="heltec_v4.svg"),

    # --- Native Linux (noflash, download only): new tiles ----------------
    dict(stem="zephcore_linux_femtofox",   kind="linux", device="Femtofox (Luckfox Pico Mini)", new=True, img="lora.svg"),
    dict(stem="zephcore_linux_rak6421",    kind="linux", device="RAK6421 WisMesh (Raspberry Pi)", new=True, img="rpi.svg"),
    dict(stem="zephcore_linux_rak6421_pi5", kind="linux", device="RAK6421 WisMesh (Raspberry Pi 5)", new=True, img="rpi.svg"),
]

DEVICE_TYPE = {"nrf": "nrf52", "esp32": "esp32", "linux": "noflash"}

# nRF52 erase package (spec §4a). ZephCore's LittleFS layout differs from MeshCore's,
# so the official erase would wipe the wrong region — we point `erase` at ZephCore's
# own formatter tool instead. The formatter is SoftDevice-specific (partition map
# differs between SD v6 and v7), so each board maps to its SD's formatter .zip.
# Files are published to firmware-dist by build.sh (stable names, no hash).
SOFTDEVICE = {
    "rak4631": 6, "rak3401_1watt": 6, "thinknode_m1": 6, "thinknode_m3": 6,
    "thinknode_m6": 6, "rak_wismesh_tag": 6, "lilygo_techo": 6,
    "lilygo_timpulse_plus": 6, "promicro_sx1262": 6, "heltec_t114": 6,
    "heltec_t096": 6, "gat562_30s": 6, "muziworks_r1neo": 6,
    "wio_tracker_l1": 7, "t1000_e": 7, "ikoka_nano_30dbm": 7,
    "sensecap_solar": 7, "xiao_nrf52840": 7,
}
FORMATTER_FILE = {6: "SoftDevice_v6_formatter.zip", 7: "SoftDevice_v7_formatter.zip"}

# Companion firmware is a single image that serves both transports (and, on
# Linux, TCP). Expose it under each applicable role, all pointing at one file.
COMPANION_ROLES = {
    "nrf":   ["companionBle", "companionUsb"],
    "esp32": ["companionBle", "companionUsb"],
    "linux": ["companionTcp"],
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

    for board in BOARDS:
        name = board["device"]
        subtitle = board.get("subtitle")
        comp = files_for(assets, board, "companion")
        repe = files_for(assets, board, "repeater")
        if not comp and not repe:
            stats["skipped"].append(f"{board['stem']}"
                                    + (f" ({board['variant']})" if board.get("variant") else ""))
            continue

        if name not in devices:
            maker = MAKER_BY_DEVICE[name]   # KeyError if a device is unmapped
            used_makers.add(maker)
            dev = {
                "maker": maker,
                "class": DEVICE_CLASS,
                "name": name,
                "type": DEVICE_TYPE[board["kind"]],
                "firmware": [],
            }
            if board.get("img"):
                dev["tooltip"] = f"<img class='device' src='{IMG_BASE}/{board['img']}'>"
            devices[name] = dev
            order.append(name)
            stats["new" if board.get("new") else "fold"] += 1
        dev = devices[name]

        # nRF52 boards get ZephCore's own SoftDevice-specific formatter as `erase`.
        erase = None
        if board["kind"] == "nrf":
            erase = f"{base}/{FORMATTER_FILE[SOFTDEVICE[board['stem']]]}"

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
