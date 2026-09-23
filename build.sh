#!/usr/bin/env bash
#
# Release matrix.
#
#   ./build.sh nrf | nrf54l | mg24 | stm32wl | linux
#   ./build.sh esp32 companions | repeaters
#
# WHICH boards are built, in which roles and variants, lives in the board
# manifests (zephcore/boards/<platform>/<board>/zephcore.yml and
# zephcore/boards/linux_native/<preset>.zephcore.yml; schema and validator in
# zephcore/scripts/board_manifest.py). This script only knows how each platform
# packages its images. A board without a `release:` section is not published.
#
# Everything a release build puts in the firmware comes from the build config,
# never from extra -D flags here, so a local build of the same board + role is
# the same firmware. Release-only differences are layout and packaging.

set -euo pipefail
mkdir -p firmware

COMMIT_HASH=$(git rev-parse --short HEAD)
GROUP="${1:?usage: build.sh <nrf|nrf54l|mg24|stm32wl|linux> | build.sh esp32 <companions|repeaters>}"
ROLE_FILTER="${2:-}"
if [[ $GROUP == "esp32" && -z $ROLE_FILTER ]]; then
    echo "build.sh esp32 needs 'companions' or 'repeaters'" >&2
    exit 1
fi

# Move the finished image(s) of one build into firmware/ under the published name.
package() {
    local platform=$1 target=$2 stem=$3 tag=$4
    local out="firmware/$stem-$tag-$COMMIT_HASH"
    case $platform in
        nrf)
            # UF2 for drag-and-drop, DFU .zip for the configurator.
            mv build/zephyr/zephyr.uf2 "$out.uf2"
            mv build/zephyr/zephyr.zip "$out.zip"
            ;;
        nrf54l | mg24 | stm32wl)
            # No USB bootloader on these SoCs (nRF54L15, EFR32MG24 and STM32WL
            # have no USB device peripheral): zephyr.hex links at the flash origin,
            # IS the whole image, and is flashed over SWD. Download-only in the
            # Mesh America catalog.
            mv build/zephyr/zephyr.hex "$out.hex"
            ;;
        linux)
            # native_sim emits zephcore_native_linux.exe; ship it extension-less.
            mv build/zephyr/zephcore_native_linux.exe "$out"
            ;;
        esp32)
            [[ $target =~ (esp32[^/]*) ]] || { echo "Unknown chip for: $target" >&2; exit 1; }
            local chip=${BASH_REMATCH[1]}
            if [[ $chip == "esp32" ]]; then
                # Classic ESP32 (T-Beam, PICO-D4): simple boot for both roles. The
                # companion's BLE controller leaves no DRAM for MCUboot and the
                # repeater is CLI-only (WiFi OTA overflows DRAM by ~10 KB), so
                # zephyr.bin is the complete bootable image at the 0x1000 ROM
                # bootloader offset; also wrapped as a full-flash merged image.
                python -m esptool --chip "$chip" merge-bin \
                    --output "$out-merged.bin" \
                    --flash-mode dio --flash-freq 40m --flash-size 4MB \
                    0x1000 build/zephyr/zephyr.bin
                cp build/zephyr/zephyr.bin "$out.bin"
            else
                # S3/C-series: sysbuild + MCUboot. Only the merged image (MCUboot @
                # 0x0 + signed app @ 0x10000) is bootable on a bare chip. The signed
                # app alone is the app-only update payload (configurator
                # "flash-update" at 0x10000, WiFi-OTA upload) and is NOT bootable
                # standalone -- never publish it as a plain .bin (bricked boards when
                # flashed like classic-ESP32's image, GH #42).
                local flash_size
                flash_size=$(python3 zephcore/scripts/dts_flash_size.py \
                    "${ZEPHYR_DTS:-build/zephcore/zephyr/zephyr.dts}")
                python -m esptool --chip "$chip" merge-bin \
                    --output "$out-merged.bin" \
                    --flash-mode dio --flash-freq 40m --flash-size "$flash_size" \
                    0x00000 build/mcuboot/zephyr/zephyr.bin \
                    0x10000 build/zephcore/zephyr/zephyr.signed.bin
                cp build/zephcore/zephyr/zephyr.signed.bin "$out-update.bin"
            fi
            ;;
    esac
}

# One manifest row: target|role|variant|confs|stem|sysbuild|host|cross_compile
build_row() {
    local target=$1 role=$2 variant=$3 confs=$4 stem=$5 sysbuild=$6 host=$7 cross=$8
    local tag="$role${variant:+-$variant}"
    local cmd=(west build -b "$target" zephcore --pristine)
    [[ -n $sysbuild ]] && cmd+=("--$sysbuild")
    local extra=()
    if [[ -n $host ]]; then
        # Native-Linux presets are cross-compiled for their SBC's arch.
        extra+=(-DZEPHYR_TOOLCHAIN_VARIANT=cross-compile
                -DNATIVE_TARGET_HOST="$host" -DCROSS_COMPILE="$cross")
    fi
    [[ -n $confs ]] && extra+=(-DEXTRA_CONF_FILE="$confs")
    ((${#extra[@]})) && cmd+=(-- "${extra[@]}")

    echo "Now building $target $tag"
    "${cmd[@]}"
    package "$GROUP" "$target" "$stem" "$tag"
}

python3 zephcore/scripts/board_manifest.py check
mapfile -t ROWS < <(python3 zephcore/scripts/board_manifest.py matrix "$GROUP" $ROLE_FILTER)
for row in "${ROWS[@]}"; do
    row=${row%$'\r'}    # Python on Windows prints CRLF
    IFS='|' read -r target role variant confs stem sysbuild host cross <<<"$row"
    build_row "$target" "$role" "$variant" "$confs" "$stem" "$sysbuild" "$host" "$cross"
done

if [[ $GROUP == "nrf" ]]; then
    # ZephCore's storage formatter, published as the `erase` package for the
    # Mesh America configurator (spec §4a). MeshCore's official erase targets a
    # different flash layout and only partially wipes a ZephCore node, so each
    # nRF52 board points `erase` at the formatter for its SoftDevice (v6/v7 have
    # different partition maps). Copied under stable, un-hashed names so the
    # catalog's erase URLs stay stable. The .zip drives the configurator's
    # automated DFU erase flow; the .uf2 is the manual drag-and-drop fallback.
    for sd in 6 7; do
        for ext in zip uf2; do
            f="formatter/SoftDevice_v${sd}_formatter.${ext}"
            if [[ -f "$f" ]]; then
                cp "$f" firmware/
                echo "Published formatter: $f"
            else
                echo "NOTE: $f not present — erase package for SoftDevice v${sd} will 404"
            fi
        done
    done

    # Device art we ship ourselves (`own_img` in the manifests). Published
    # alongside the firmware so the catalog resolves it against the same
    # --url-base as everything else, rather than hardcoding a host.
    if compgen -G "img/*" > /dev/null; then
        cp img/* firmware/
        echo "Published device art: $(ls img/ | tr '\n' ' ')"
    fi
fi
