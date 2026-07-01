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
    xiao_nrf52840
    lilygo_techo
    promicro_sx1262
    heltec_t114
    heltec_t096
    gat562_30s
)

# Native-Linux presets (not Zephyr boards — built with -b native_sim plus an
# EXTRA_CONF_FILE). Each targets a real SBC arch, so it is cross-compiled.
Linux_boards=(
    femtofox
    rak6421
    rak6421_pi5
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
    heltec_wireless_tracker/esp32s3/procpu
    ttgo_tbeam/esp32/procpu
)

if [[ $1 == "nrf" ]]; then
    for board in "${nRF_boards[@]}"; do
        board_clean_for_path=$(echo "$board" | sed -e 's/\//-/g')
        
        # build nRF companions (production is the default — no extra conf needed)
        echo "Now building $board companion"
        if [[ $board == "wio_tracker_l1" ]]; then
            west build -b "$board" zephcore --pristine -- -DCONFIG_ZEPHCORE_EASTER_EGG_DOOM=y
        else
            west build -b "$board" zephcore --pristine
        fi
        mv build/zephyr/zephyr.uf2 firmware/"$board"-companion-"$COMMIT_HASH".uf2
        mv build/zephyr/zephyr.zip firmware/"$board"-companion-"$COMMIT_HASH".zip

        # build nRF repeaters
        echo "Now building $board repeater"
        west build -b "$board" zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"
        mv build/zephyr/zephyr.uf2 firmware/"$board"-repeater-"$COMMIT_HASH".uf2
        mv build/zephyr/zephyr.zip firmware/"$board"-repeater-"$COMMIT_HASH".zip

        # Heltec T114 is sold both with and without the TFT module.
        # Build dedicated screenless variants matching upstream's
        # Heltec_t114_without_display_* PIO envs.
        if [[ $board == "heltec_t114" ]]; then
            echo "Now building $board companion (noscreen)"
            west build -b "$board" zephcore --pristine -- -DEXTRA_CONF_FILE="boards/nrf52840/heltec_t114/no_display.conf"
            mv build/zephyr/zephyr.uf2 firmware/"$board"-companion-noscreen-"$COMMIT_HASH".uf2
            mv build/zephyr/zephyr.zip firmware/"$board"-companion-noscreen-"$COMMIT_HASH".zip

            echo "Now building $board repeater (noscreen)"
            west build -b "$board" zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/nrf52840/heltec_t114/no_display.conf"
            mv build/zephyr/zephyr.uf2 firmware/"$board"-repeater-noscreen-"$COMMIT_HASH".uf2
            mv build/zephyr/zephyr.zip firmware/"$board"-repeater-noscreen-"$COMMIT_HASH".zip
        fi
    done
fi

if [[ $1 == "linux" ]]; then
    for board in "${Linux_boards[@]}"; do
        # Pick the native_sim variant + cross toolchain for the target SBC arch.
        case "$board" in
            femtofox)
                # Luckfox Pico Mini — ARMv7-A (32-bit) → native_sim (32-bit)
                zboard="native_sim"
                host="arm"
                cross="/usr/bin/arm-linux-gnueabihf-"
                ;;
            rak6421|rak6421_pi5)
                # Raspberry Pi — aarch64 (64-bit) → native_sim/native/64
                zboard="native_sim/native/64"
                host="aarch64"
                cross="/usr/bin/aarch64-linux-gnu-"
                ;;
            *)
                echo "Unknown linux board: $board"
                exit 1
                ;;
        esac

        # build native-Linux companion (TCP transport — the default role)
        echo "Now building $board companion (native linux)"
        west build -b "$zboard" zephcore --pristine -- \
            -DZEPHYR_TOOLCHAIN_VARIANT=cross-compile \
            -DNATIVE_TARGET_HOST="$host" \
            -DCROSS_COMPILE="$cross" \
            -DEXTRA_CONF_FILE="boards/linux_native/$board.conf"
        # native_sim emits zephcore_native_linux.exe; ship it as an extension-less
        # per-board name (zephcore_linux_<board>-<role>-<hash>).
        mv build/zephyr/zephcore_native_linux.exe firmware/zephcore_linux_"$board"-companion-"$COMMIT_HASH"

        # build native-Linux repeater
        echo "Now building $board repeater (native linux)"
        west build -b "$zboard" zephcore --pristine -- \
            -DZEPHYR_TOOLCHAIN_VARIANT=cross-compile \
            -DNATIVE_TARGET_HOST="$host" \
            -DCROSS_COMPILE="$cross" \
            -DEXTRA_CONF_FILE="boards/linux_native/$board.conf;boards/common/repeater.conf"
        mv build/zephyr/zephcore_native_linux.exe firmware/zephcore_linux_"$board"-repeater-"$COMMIT_HASH"
    done
fi

if [[ $1 == "esp32" ]]; then
    for board in "${ESP32_boards[@]}"; do
        board_clean_for_path=$(echo "$board" | sed -e 's/\//-/g')
        
        if [[ $board =~ (esp32[^/]*) ]]; then
            chip="${BASH_REMATCH[1]}"
        else
            echo "Unknown chip for: $board"
            exit 1;
        fi

        # Classic ESP32 (e.g. T-Beam, PICO-D4) uses Zephyr's simple-boot path for
        # both roles: a self-contained zephyr.bin flashed at the 0x1000 ROM
        # bootloader offset. The companion keeps BLE (whose controller reserves
        # ~50KB DRAM, leaving no room for MCUboot) and the repeater is CLI-only
        # (WiFi OTA's functional driver + 64KB heap overflow DRAM by ~10KB). The
        # S3/C-series have the DRAM headroom and use sysbuild + MCUboot below.
        if [[ $chip == "esp32" ]]; then
            if [[ $2 == "companions" ]]; then
                role="companion"
                echo "Now building $board companion (simple boot)"
                west build -b "$board" zephcore --pristine
            elif [[ $2 == "repeaters" ]]; then
                role="repeater"
                echo "Now building $board repeater (simple boot)"
                west build -b "$board" zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"
            else
                continue
            fi
            # Simple-boot zephyr.bin is already the complete bootable image;
            # wrap it in a full-flash merged image at the 0x1000 offset.
            python -m esptool --chip "$chip" merge-bin \
                --output firmware/"$board_clean_for_path"-"$role"-"$COMMIT_HASH"-merged.bin \
                --flash-mode dio --flash-freq 40m --flash-size 4MB \
                0x1000 build/zephyr/zephyr.bin
            cp build/zephyr/zephyr.bin firmware/"$board_clean_for_path"-"$role"-"$COMMIT_HASH".bin
            continue
        fi

        if [[ $2 == "companions" ]]; then
            # build ESP32 companions (production is the default)
            echo "Now building $board companion"
            west build -b "$board" zephcore --pristine --sysbuild
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
            west build -b "$board" zephcore --pristine --sysbuild -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"
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
