#!/usr/bin/env bash

set -euo pipefail
mkdir -p firmware

COMMIT_HASH=$(git rev-parse --short HEAD)

nRF_boards=(
    rak4631
    rak3401_1watt
    wio_tracker_l1
    t1000_e
    thinknode_m1
    thinknode_m3
    thinknode_m6
    rak_wismesh_tag
    ikoka_nano_30dbm
    sensecap_solar
)

ESP32_boards=(
    xiao_esp32c3
    xiao_esp32c6/esp32c6/hpcore
    xiao_esp32s3/esp32s3/procpu
    lilygo_tlora_c6/esp32c6/hpcore
    station_g2/esp32s3/procpu
    heltec_wifi_lora32_v3/esp32s3/procpu
    heltec_wifi_lora32_v4/esp32s3/procpu
    heltec_wifi_lora32_v43/esp32s3/procpu
)

if [[ $1 == "nrf" ]]; then
    for board in "${nRF_boards[@]}"; do
        board_clean_for_path=$(echo "$board" | sed -e 's/\//-/g')
        
        # build nRF companions
        echo "Now building $board companion"
        if [[ $board == "wio_tracker_l1" ]]; then
            west build -b "$board" zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/prod.conf" -DCONFIG_ZEPHCORE_EASTER_EGG_DOOM=y
        else
            west build -b "$board" zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/prod.conf"
        fi
        mv build/zephyr/zephyr.uf2 firmware/"$board"-companion-"$COMMIT_HASH".uf2
        mv build/zephyr/zephyr.zip firmware/"$board"-companion-"$COMMIT_HASH".zip
        
        # build nRF repeaters
        echo "Now building $board repeater"
        west build -b "$board" zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/prod.conf"
        mv build/zephyr/zephyr.uf2 firmware/"$board"-repeater-"$COMMIT_HASH".uf2
        mv build/zephyr/zephyr.zip firmware/"$board"-repeater-"$COMMIT_HASH".zip
    done
fi

if [[ $1 == "esp32" ]]; then
    for board in "${ESP32_boards[@]}"; do
        board_clean_for_path=$(echo "$board" | sed -e 's/\//-/g')
        
        if [[ $board =~ (esp32[^/]+) ]]; then
            chip="${BASH_REMATCH[1]}"
        else
            echo "Unknown chip for: $board"
            exit 1;
        fi
        
        if [[ $2 == "companions" ]]; then
            # build ESP32 companions
            echo "Now building $board companion"
            west build -b "$board" zephcore --pristine --sysbuild -- -DEXTRA_CONF_FILE="boards/common/prod.conf"
            FLASH_SIZE=$(
                python3 -c '
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "build/zephcore/zephyr/zephyr.dts"
dts = open(path, encoding="utf-8", errors="replace").read()

def parse_cell(tok: str) -> int:
    t = tok.strip()
    return int(t, 16) if t.lower().startswith("0x") else int(t)

m = re.search(
    r"flash0:\s*flash@[^{]*\{[^}]*?reg\s*=\s*<\s*(?:0x[0-9a-fA-F]+|[0-9]+)\s+"
    r"((?:0x)?[0-9a-fA-F]+)\s*>",
    dts,
    re.DOTALL,
)
if not m:
    m = re.search(
        r"compatible\s*=\s*\"soc-nv-flash\"\s*;\s*[\s\S]*?"
        r"reg\s*=\s*<\s*(?:0x[0-9a-fA-F]+|[0-9]+)\s+((?:0x)?[0-9a-fA-F]+)\s*>",
        dts,
    )
if not m:
    raise SystemExit("flash reg not found in " + path)

size = parse_cell(m.group(1))
print(str(size // 1048576) + "MB")
                ' "${ZEPHYR_DTS:-build/zephcore/zephyr/zephyr.dts}"
            )
            python -m esptool --chip "$chip" merge-bin \
            --output firmware/"$board_clean_for_path"-companion-"$COMMIT_HASH"-merged.bin \
            --flash-mode dio --flash-freq 40m --flash-size "$FLASH_SIZE" \
            0x00000 build/mcuboot/zephyr/zephyr.bin \
            0x20000 build/zephcore/zephyr/zephyr.signed.bin
            mv build/zephcore/zephyr/zephyr.signed.bin firmware/"$board_clean_for_path"-companion-"$COMMIT_HASH".bin
        fi
        
        if [[ $2 == "repeaters" ]]; then
            # build ESP32 repeaters
            echo "Now building $board repeater"
            west build -b "$board" zephcore --pristine --sysbuild -- -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/prod.conf"
            FLASH_SIZE=$(
                python3 -c '
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "build/zephcore/zephyr/zephyr.dts"
dts = open(path, encoding="utf-8", errors="replace").read()

def parse_cell(tok: str) -> int:
    t = tok.strip()
    return int(t, 16) if t.lower().startswith("0x") else int(t)

m = re.search(
    r"flash0:\s*flash@[^{]*\{[^}]*?reg\s*=\s*<\s*(?:0x[0-9a-fA-F]+|[0-9]+)\s+"
    r"((?:0x)?[0-9a-fA-F]+)\s*>",
    dts,
    re.DOTALL,
)
if not m:
    m = re.search(
        r"compatible\s*=\s*\"soc-nv-flash\"\s*;\s*[\s\S]*?"
        r"reg\s*=\s*<\s*(?:0x[0-9a-fA-F]+|[0-9]+)\s+((?:0x)?[0-9a-fA-F]+)\s*>",
        dts,
    )
if not m:
    raise SystemExit("flash reg not found in " + path)

size = parse_cell(m.group(1))
print(str(size // 1048576) + "MB")
                ' "${ZEPHYR_DTS:-build/zephcore/zephyr/zephyr.dts}"
            )
            python -m esptool --chip "$chip" merge-bin \
            --output firmware/"$board_clean_for_path"-repeater-"$COMMIT_HASH"-merged.bin \
            --flash-mode dio --flash-freq 40m --flash-size "$FLASH_SIZE" \
            0x00000 build/mcuboot/zephyr/zephyr.bin \
            0x20000 build/zephcore/zephyr/zephyr.signed.bin
            mv build/zephcore/zephyr/zephyr.signed.bin firmware/"$board_clean_for_path"-repeater-"$COMMIT_HASH".bin
        fi
    done
fi