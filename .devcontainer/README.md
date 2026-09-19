# ZephCore dev container

A ready-to-build Zephyr workspace: **Zephyr SDK 1.0.1**, **west**, and the four
toolchains ZephCore's boards need. It reproduces the environment from
[.github/workflows/build.yml](../.github/workflows/build.yml), so a local build
matches CI.

## Getting started

Open the repo in VS Code and run **Dev Containers: Reopen in Container**, or from
a terminal:

```bash
devcontainer up --workspace-folder .
```

First creation takes a while — it downloads the SDK toolchains (~1 GB), clones
Zephyr and its modules, and fetches the vendor BLE blobs. Subsequent starts are
instant.

When it finishes you can build immediately, from the workspace root:

```bash
west build -b rak4631 zephcore --pristine
west build -b xiao_esp32s3/esp32s3/procpu zephcore --pristine --sysbuild
bash build.sh nrf
```

Artifacts land in `build/zephyr/` and, for `build.sh`, `firmware/`.

## What's inside

| | |
|---|---|
| Base | `mcr.microsoft.com/devcontainers/base:ubuntu-24.04` (Python 3.12, CMake 3.28) |
| Zephyr SDK | 1.0.1 at `/opt/zephyr-sdk-1.0.1` |
| Toolchains | `arm-zephyr-eabi`, `xtensa-espressif_esp32_zephyr-elf`, `xtensa-espressif_esp32s3_zephyr-elf`, `riscv64-zephyr-elf` |
| Cross compilers | `arm-linux-gnueabihf`, `aarch64-linux-gnu` (for the `build.sh linux` native_sim presets) |
| Python | venv at `/opt/zephyr-venv`, first on `PATH` — holds `west`, `esptool`, `pyocd`, `adafruit-nrfutil`, and Zephyr's own requirements |
| Vendor blobs | `hal_espressif` and `hal_silabs` BLE controllers, fetched on create |

Only the toolchains ZephCore actually targets are installed. Adding a board on a
new architecture means adding its toolchain to `ZSDK_TOOLCHAINS` in
[devcontainer.json](devcontainer.json) (and to `toolchains` in the CI workflow)
and rebuilding the container.

## Workspace layout

The west topdir is the workspace root — the folder that *contains* `zephcore/` —
matching what `west init -l zephcore` and `build.sh` assume:

```
/workspaces/ZephCore/
├── zephcore/      the application (this git repo)
├── zephyr/        \
├── modules/        |  west-managed, in named volumes
├── bootloader/     |
├── tools/         /
├── build/         on the host bind mount
└── firmware/      on the host bind mount
```

`zephyr/`, `modules/`, `bootloader/` and `tools/` live in Docker named volumes
rather than on the host bind mount. They are tens of thousands of files nobody
edits from the host, and container-native storage is several times faster for
them on Windows and macOS. `build/` and `firmware/` stay on the bind mount so you
can reach the binaries from the host to flash them.

To reclaim that space after deleting the container: `docker volume prune`, or
`docker volume ls | grep zephcore-` to remove them individually.

## Setup knobs

[post-create.sh](post-create.sh) is idempotent — re-run it any time with
`bash .devcontainer/post-create.sh`. Two environment variables change what it
does:

- `ZEPHCORE_WEST_FULL=1` — clone Zephyr and modules with full history. The
  default uses `west update --narrow -o=--depth=1`, which cuts the checkout from
  ~5 GB to well under 1 GB. The trade-off is that `zephyr/` and the modules carry
  no history and no tags, so `git log`/`git describe`/`git bisect` in them don't
  work. Builds are unaffected — ZephCore's version string comes from
  `ZEPHCORE_FIRMWARE_VERSION` in `zephcore/CMakeLists.txt`. Set this if you need
  to dig through upstream history.
- `ZEPHCORE_SKIP_BLOBS=1` — skip the vendor BLE blobs. Fine for nRF-only work;
  ESP32 and MG24 *companion* builds will fail to link without them.

## Flashing

`west flash` from inside the container only works on a **Linux host** — Docker
Desktop on Windows and macOS cannot pass USB devices through. On Linux, uncomment
the `runArgs` line in [devcontainer.json](devcontainer.json).

Everywhere else, build in the container and flash from the host: the `.uf2`,
`.zip`, `.hex` and `.bin` files in `build/zephyr/` and `firmware/` are on the
bind mount and visible to host tools. Drag-and-drop UF2 flashing needs nothing
installed on the host at all.

## Note on `.vscode/settings.json`

If you have a local (gitignored) `.vscode/settings.json` with a Windows path in
`cmake.sourceDirectory`, it overrides the container setting and points CMake
Tools at a path that does not exist inside the container. Either delete that file
or change the value to `${workspaceFolder}/zephcore`.
