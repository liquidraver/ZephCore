# ZephCore development container.
#
# Mirrors what .github/workflows/build.yml installs on the CI runner, so a build
# in here matches a build in CI: Zephyr SDK 1.0.1, the same four toolchains, the
# same Python version, and the arm/aarch64 Linux cross compilers that the
# native_sim SBC presets need.
#
# Ubuntu 24.04 is used because it ships Python 3.12 (CI's python_version) and
# CMake 3.28 (Zephyr 4.4 needs >= 3.20.5).
ARG BASE_VARIANT=ubuntu-24.04
FROM mcr.microsoft.com/devcontainers/base:${BASE_VARIANT}

# Keep these in sync with `zephyr_sdk` and `toolchains` in
# .github/workflows/build.yml.
ARG ZSDK_VERSION=1.0.1
ARG ZSDK_TOOLCHAINS="arm-zephyr-eabi xtensa-espressif_esp32s3_zephyr-elf xtensa-espressif_esp32_zephyr-elf riscv64-zephyr-elf"

ENV DEBIAN_FRONTEND=noninteractive

# Zephyr's documented host dependencies, plus:
#   - gcc/g++-arm-linux-gnueabihf + aarch64-linux-gnu: the `build.sh linux`
#     native_sim presets are cross-compiled for the target SBC. See
#     docs/LINUX_NATIVE.md.
#   - ccache: Zephyr picks it up automatically as a compiler launcher.
#   - dfu-util / usbutils / picocom: flashing and serial console on Linux hosts.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ccache \
        cmake \
        device-tree-compiler \
        dfu-util \
        file \
        gdb-multiarch \
        gperf \
        libmagic1 \
        make \
        ninja-build \
        picocom \
        python3 \
        python3-dev \
        python3-venv \
        udev \
        unzip \
        usbutils \
        wget \
        xz-utils \
        zip \
        gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf \
        gcc-aarch64-linux-gnu   g++-aarch64-linux-gnu \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Zephyr SDK
#
# The "minimal" bundle is the SDK scaffolding + host tools only; setup.sh then
# pulls just the toolchains named in ZSDK_TOOLCHAINS. Installing all ~35 GNU
# toolchains would add tens of GB for architectures ZephCore never targets.
#
# The `cmake -P zephyr_sdk_export.cmake` step registers the SDK in the *vscode*
# user's CMake package registry rather than root's -- that script writes to
# $HOME/.cmake unconditionally. ZEPHYR_SDK_INSTALL_DIR below already covers
# find_package(Zephyr-sdk); the registry entry is belt and braces for tooling
# that shells out with a different environment.
# ---------------------------------------------------------------------------
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-${ZSDK_VERSION}
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr

RUN set -eux; \
    arch="$(uname -m)"; \
    base="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}"; \
    bundle="zephyr-sdk-${ZSDK_VERSION}_linux-${arch}_minimal.tar.xz"; \
    cd /opt; \
    wget -q --progress=dot:giga -O "${bundle}" "${base}/${bundle}"; \
    wget -q -O sha256.sum "${base}/sha256.sum"; \
    grep "  ${bundle}\$" sha256.sum | sha256sum -c -; \
    rm -f sha256.sum; \
    tar xf "${bundle}"; \
    rm -f "${bundle}"; \
    cd "zephyr-sdk-${ZSDK_VERSION}"; \
    set --; \
    for t in ${ZSDK_TOOLCHAINS}; do set -- "$@" -t "$t"; done; \
    ./setup.sh "$@" -h; \
    HOME=/home/vscode cmake -P cmake/zephyr_sdk_export.cmake; \
    chown -R vscode:vscode /home/vscode/.cmake

# ---------------------------------------------------------------------------
# Python environment
#
# Lives in /opt (image layer, not the bind-mounted workspace) so it survives a
# host-side `git clean` and stays fast on Windows/macOS bind mounts. Ubuntu 24.04
# marks the system interpreter PEP 668 externally-managed, so a venv is required
# rather than merely tidy.
#
# Only west and the flash/DFU tools ZephCore's docs call for go in here. The bulk
# of Zephyr's Python requirements are installed by post-create.sh via
# `west packages pip --install`, which needs a populated workspace to enumerate
# the per-module requirement files (that is where esptool comes from).
# ---------------------------------------------------------------------------
ENV VIRTUAL_ENV=/opt/zephyr-venv
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}

RUN set -eux; \
    python3 -m venv "${VIRTUAL_ENV}"; \
    pip install --no-cache-dir --upgrade pip setuptools wheel; \
    pip install --no-cache-dir west pyocd adafruit-nrfutil; \
    chown -R vscode:vscode "${VIRTUAL_ENV}"

USER vscode
