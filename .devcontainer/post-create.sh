#!/usr/bin/env bash
#
# One-time setup for the ZephCore dev container.
#
# Initialises the west workspace at the repo root (topdir = the folder that
# *contains* zephcore/, exactly as README.md and build.sh assume), pulls Zephyr
# and its modules, installs the Python requirements those modules declare, and
# fetches the vendor BLE controller blobs that ESP32 and MG24 builds link
# against.
#
# Safe to re-run: every step is idempotent. Useful knobs:
#   ZEPHCORE_WEST_FULL=1   full (non-shallow) clones -- needed if you plan to
#                          bisect or `git log` inside zephyr/ or modules/
#   ZEPHCORE_SKIP_BLOBS=1  skip the ~100 MB of vendor blobs (nRF-only work)
set -euo pipefail

# postCreateCommand runs in the workspace folder, which is the west topdir.
TOPDIR="$(pwd)"

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
note() { printf '    %s\n' "$*"; }

# ---------------------------------------------------------------------------
# Named-volume mount points arrive root-owned and empty. Hand them to the
# container user before west tries to clone into them.
# ---------------------------------------------------------------------------
step "Preparing mount points"
for d in zephyr modules bootloader tools; do
    if [ -d "$TOPDIR/$d" ] && [ ! -w "$TOPDIR/$d" ]; then
        sudo chown -R "$(id -u):$(id -g)" "$TOPDIR/$d"
        note "took ownership of $d/"
    fi
done
if [ -d "$HOME/.cache/ccache" ] && [ ! -w "$HOME/.cache/ccache" ]; then
    sudo chown -R "$(id -u):$(id -g)" "$HOME/.cache/ccache"
    note "took ownership of ~/.cache/ccache"
fi

# The workspace is a bind mount from the host, so its files can carry a uid that
# does not match the container user. Without this, git refuses to operate and
# Zephyr's build-time `git describe` calls fail.
git config --global --get-all safe.directory | grep -qx '\*' \
    || git config --global --add safe.directory '*'

# ---------------------------------------------------------------------------
# West workspace
# ---------------------------------------------------------------------------
if [ -d "$TOPDIR/.west" ]; then
    step "West workspace already initialised"
else
    step "Initialising west workspace (topdir: $TOPDIR)"
    west init -l zephcore
fi

step "Fetching Zephyr and modules (this is the slow one on a cold start)"
if [ "${ZEPHCORE_WEST_FULL:-0}" = "1" ]; then
    west update
else
    # --narrow fetches only the manifest-pinned revision instead of every branch
    # and tag; --depth=1 makes each of those a shallow clone. Together they turn
    # a ~5 GB checkout into well under 1 GB. The trade-off is that zephyr/ and the
    # modules carry no history and no tags, so anything reading them with
    # `git describe` or `git log` falls back to the VERSION file. ZephCore's own
    # version string comes from ZEPHCORE_FIRMWARE_VERSION in
    # zephcore/CMakeLists.txt and is unaffected. Set ZEPHCORE_WEST_FULL=1 for
    # full history.
    west update --narrow -o=--depth=1 || {
        note "shallow update failed -- retrying with full clones"
        west update
    }
fi

step "Registering Zephyr CMake package"
west zephyr-export

# ---------------------------------------------------------------------------
# Python requirements
#
# `west packages pip --install` walks zephyr/scripts/requirements.txt plus every
# module that declares requirement files in its module.yml. hal_espressif is one
# of them, and that is where build.sh's `python -m esptool merge-bin` comes from.
# ---------------------------------------------------------------------------
step "Installing Zephyr Python requirements"
if ! west packages pip --install; then
    note "west packages unavailable -- falling back to zephyr/scripts/requirements.txt"
    pip install -r "$TOPDIR/zephyr/scripts/requirements.txt"
fi

step "Installing ZephCore Python requirements"
pip install -r "$TOPDIR/requirements.txt"

# ---------------------------------------------------------------------------
# Vendor blobs
#
# hal_espressif: BLE controller libraries -- ESP32 companion builds will not link
#                without them.
# hal_silabs:    same story for the EFR32MG24 (xiao_mg24) companion.
# ---------------------------------------------------------------------------
if [ "${ZEPHCORE_SKIP_BLOBS:-0}" = "1" ]; then
    step "Skipping vendor blobs (ZEPHCORE_SKIP_BLOBS=1)"
    note "ESP32 and MG24 companion builds will fail to link until you run:"
    note "  west blobs fetch hal_espressif && west blobs fetch hal_silabs"
else
    step "Fetching vendor BLE controller blobs"
    west blobs fetch hal_espressif
    west blobs fetch hal_silabs
fi

# ---------------------------------------------------------------------------
step "Environment"
printf '    %-24s %s\n' \
    "Zephyr SDK"        "$(cat "${ZEPHYR_SDK_INSTALL_DIR}/sdk_version" 2>/dev/null || echo '??') (${ZEPHYR_SDK_INSTALL_DIR})" \
    "west"              "$(west --version 2>/dev/null | head -1)" \
    "Zephyr"            "$(sed -n 's/^VERSION_MAJOR = //p' "$TOPDIR/zephyr/VERSION" 2>/dev/null).$(sed -n 's/^VERSION_MINOR = //p' "$TOPDIR/zephyr/VERSION" 2>/dev/null).$(sed -n 's/^PATCHLEVEL = //p' "$TOPDIR/zephyr/VERSION" 2>/dev/null)" \
    "Python"            "$(python --version 2>&1) ($(command -v python))" \
    "Toolchains"        "$(ls "${ZEPHYR_SDK_INSTALL_DIR}/gnu" 2>/dev/null | tr '\n' ' ')"

cat <<'EOF'

    Ready. Build from this directory (the west topdir), e.g.:

      west build -b rak4631 zephcore --pristine
      west build -b rak4631 zephcore --pristine -- -DEXTRA_CONF_FILE="boards/common/repeater.conf"
      bash build.sh nrf

    Artifacts land in build/zephyr/ and (for build.sh) firmware/, both of which
    are on the host bind mount -- flash them with your host's tools.
EOF
