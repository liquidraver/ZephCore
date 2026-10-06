nRF formatter tools

- Erases the internal LittleFS and BLE-bond NVS partitions, and the external QSPI flash on
  every supported board that has one (see [QSPI flash](#qspi-flash) below)
- Watch out for softdevice version! Flashing the wrong version can corrupt the node and you'll need a full bootloader reflash with adafruit-nrfutil!
  - You can check what softdevice version you use if you open INFO_UF2.TXT on the storage drive when in DFU mode. Bootloader should say "sxxx 6.x.x" for v6 and "sxxx 7.x.x" for v7
  - That warning applies to the **`.uf2`** files, which have no version guard. The **`.zip`** packages carry an `--sd-req` guard (v6 = `0x00B6`, v7 = `0x0123`), so the bootloader *rejects* a mismatched package instead of corrupting the node.
- Formatter output logs from the process over serial
- After format, it puts back the device to Mass Storage DFU mode

## Files

| SoftDevice | Boards |
|------------|--------|
| **v6** | RAK4631, RAK3401 1W, ThinkNode M1/M3/M6, RAK WisMesh Tag, LilyGo T-Echo, LilyGo T-Impulse Plus, ProMicro SX1262, ProMicro LR2021, Heltec T114, Heltec T096, Heltec Mesh Node T1, GAT562 30s, muzi works R1 Neo |
| **v7** | Wio Tracker L1, Wio Tracker L1 Pro 1W, T1000-E, MeshTracker X1, Ikoka Nano 30dBm, SenseCAP Solar, XIAO nRF52840 |

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

## QSPI flash

The formatter does not know which board it is running on. It tries each known QSPI pin
set in turn, and the first one where a flash answers its JEDEC ID gets a full chip erase.

| Pin set | Boards | Flash rail pin |
|---------|--------|----------------|
| Wio/XIAO/Ikoka/SenseCap | Wio Tracker L1, Wio Tracker L1 Pro 1W, XIAO nRF52840, Ikoka Nano 30dBm, SenseCAP Solar | — |
| ThinkNode M1 / TEcho | ThinkNode M1, LilyGo T-Echo | P0.12 |
| ThinkNode M6 | ThinkNode M6 (units with the flash fitted) | P0.21 |
| RAK4631 / GAT562 | RAK base boards with QSPI flash | — |
| LilyGo TEcho Lite | LilyGo T-Echo Lite | — |
| Nano G2 Ultra | Nano G2 Ultra | — |
| LilyGo T-Impulse Plus | LilyGo T-Impulse Plus (v6 build only) | P0.14 |
| MeshTracker X1 | MeshTracker X1 (v7 build only) | P0.15 |

- A flash left in deep power-down is woken first. Repeaters, room servers and observers
  put it to sleep, and the reset into the formatter does not wake it.
- Where the flash sits behind a switched supply rail, the formatter raises that pin and
  gives the part time to start; nothing else powers it after the bootloader.
- The erase is checked: the log shows the first bytes of the flash before the erase, and
  the erase is reported as failed unless they read back blank afterwards.
- The last lines of the log say what happened: `All partitions erased successfully
  (internal flash + QSPI)`, or that the internal flash was erased and no QSPI flash
  answered. The second is normal on a board without one (T1000-E, RAK4631 on a base
  board without flash, ...). On a board from the table above it means the QSPI was
  **not** erased.

Verified on hardware: Wio Tracker L1 (as companion and as repeater) and T1000-E. The
MeshTracker X1, T-Impulse Plus and ThinkNode M1 / T-Echo rail entries follow the board
devicetrees and have not been run on those boards.

To support a new board, add its QSPI pin mapping to `known_boards[]` in
`zephcore/tools/formatter/src/qspi_probe.c` — no DTS or Kconfig changes needed. Set
`pwr_pin` if a GPIO switches the flash's supply, and `sd` if the board exists on one
SoftDevice only: every probed pin set drives its pins on every board that build runs on,
so a set limited to its own build stays off unrelated boards.
