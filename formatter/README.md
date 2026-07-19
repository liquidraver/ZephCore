nRF formatter tools

- QSPI is formatted for all supported boards
- Watch out for softdevice version! Flashing the wrong version can corrupt the node and you'll need a full bootloader reflash with adafruit-nrfutil!
  - You can check what softdevice version you use if you open INFO_UF2.TXT on the storage drive when in DFU mode. Bootloader should say "sxxx 6.x.x" for v6 and "sxxx 7.x.x" for v7
  - That warning applies to the **`.uf2`** files, which have no version guard. The **`.zip`** packages carry an `--sd-req` guard (v6 = `0x00B6`, v7 = `0x0123`), so the bootloader *rejects* a mismatched package instead of corrupting the node.
- Formatter output logs from the process over serial
- After format, it puts back the device to Mass Storage DFU mode

## Files

| SoftDevice | Boards |
|------------|--------|
| **v6** | RAK4631, RAK3401 1W, ThinkNode M1/M3/M6, RAK WisMesh Tag, LilyGo T-Echo, LilyGo T-Impulse Plus, ProMicro SX1262, Heltec T114, Heltec T096, GAT562 30s, muzi works R1 Neo |
| **v7** | Wio Tracker L1, T1000-E, Ikoka Nano 30dBm, SenseCAP Solar, XIAO nRF52840 |

- **`.uf2`** — manual drag-and-drop onto the UF2 mass-storage drive.
- **`.zip`** — Adafruit DFU package. Used by the Mesh America configurator as ZephCore's
  `erase` package (its automated erase flow), and flashable by hand with
  `adafruit-nrfutil dfu serial -pkg <zip> -p COMx -b 115200 --singlebank --touch 1200`.
  See `PROVIDER_CATALOG.md` for how the catalog wires these up per board.

`build.sh nrf` copies all four files into `firmware/` under these exact names so the
catalog's `erase` URLs stay stable across releases — don't rename them.

## Rebuilding

Two universal builds, one per SoftDevice. Source is `zephcore/tools/formatter`. The
partition map comes from the build board's overlay, and `qspi_probe.c` probes QSPI
bare-metal, so one image covers every board on that SoftDevice.

```bash
# SoftDevice v6 (universal target: rak4631)
west build -b rak4631 zephcore/tools/formatter --pristine -d build_fmt6
cp build_fmt6/zephyr/zephyr.uf2 formatter/SoftDevice_v6_formatter.uf2
cp build_fmt6/zephyr/zephyr.zip formatter/SoftDevice_v6_formatter.zip

# SoftDevice v7 (universal target: t1000_e)
west build -b t1000_e zephcore/tools/formatter --pristine -d build_fmt7
cp build_fmt7/zephyr/zephyr.uf2 formatter/SoftDevice_v7_formatter.uf2
cp build_fmt7/zephyr/zephyr.zip formatter/SoftDevice_v7_formatter.zip
```

The `.zip` needs `adafruit-nrfutil` on PATH (`pip install adafruit-nrfutil`); the build
prints `Formatter DFU zip: ENABLED (sd-req=...)` when wired up, and silently skips the
zip otherwise. `--sd-req` is read automatically from the build board's `board.conf`
(`CONFIG_ZEPHCORE_SD_FWID`), so it always matches the SoftDevice being targeted.

To support a new board, add its QSPI pin mapping to `known_boards[]` in
`zephcore/tools/formatter/src/qspi_probe.c` — no DTS or Kconfig changes needed.
