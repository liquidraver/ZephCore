LR1110 radio firmware updater

Prebuilt tools that update the **LR1110 radio chip's own firmware** to `0x0402`.
This is the radio's internal firmware — not ZephCore. Flash one of these, let it
run, then flash normal ZephCore firmware afterwards.

You almost certainly do **not** need this. It exists for the Semtech H1_2026
security release (CVE-2025-14857 / 14858 / 14859), which is only exploitable with
physical access to the radio's SPI pins. Boards shipping `0x0401` work fine with
ZephCore and can be left alone.

## Files

| File | Board | How to flash |
|------|-------|--------------|
| **`T1000-E_LR1110_updater_FW0402.uf2`** | SenseCAP T1000-E | Double-tap reset, drag onto the UF2 drive |
| **`ThinkNode-M9_LR1110_updater_FW0402.bin`** | ThinkNode M9 | `esptool --chip esp32s3 -p COMx write_flash 0x0 <file>` |

Watch the serial console while it runs (115200) — it prints every step, and a
per-chunk trace of the flash write. The whole thing takes about 20 seconds.

## Before you flash

- **Remove the SD card** (M9). The slot shares SPI2 with the radio, and a card
  present during flashing corrupts the image *while every write still reports
  success* — the radio ends up running nothing, with no error to point at. The
  tool detects a card and refuses to start, so you'll be told rather than bitten.
- **The bootloader update is one-way.** Firmware `0x0402` only runs on chip
  bootloader `0x1001`, so a chip on the original `0x6500` gets its bootloader
  rewritten first. Semtech ships no loader in the reverse direction, so that part
  cannot be undone. It is safe — verified end-to-end — but it is permanent.
- **Don't interrupt it.** If it fails partway, power-cycle and run it again; the
  tool detects whatever state the chip landed in and resumes appropriately.

Re-running once the radio is already on `0x0402` is harmless — it detects the
target firmware and exits without touching flash.

## Reading the output

```
[3/8] Reading current firmware version...
  HW   = 0x22          <- V2C production silicon, normal
  TYPE = 0x01          <- 0x01 transceiver, 0xDF bootloader, 0xDE loader running
  FW   = 0x0402
```

Success looks like `TYPE=0x01 FW=0x0402` at step 8. If it ends at `TYPE=0xDF`,
the firmware was written but isn't running — power-cycle and re-run, and check
the SD card and the radio's supply.

During Stage A the chip briefly stays in bootloader mode after the bootloader is
rewritten. That is expected, not a failure — the loader image left in flash was
built for the old bootloader, so the new one declines to run it. Stage B follows.

## Rebuilding

Source is `zephcore/tools/lr1110_updater`.

```bash
west build -b t1000_e zephcore/tools/lr1110_updater --pristine -d build_t1000
cp build_t1000/zephyr/zephyr.uf2 LR1110_updater/T1000-E_LR1110_updater_FW0402.uf2

west build -b thinknode_m9/esp32s3/procpu zephcore/tools/lr1110_updater --pristine -d build_m9
cp build_m9/zephyr/zephyr.bin LR1110_updater/ThinkNode-M9_LR1110_updater_FW0402.bin
```

The M9 image is a plain simple-boot ESP32-S3 binary loaded from `0x0` — no
MCUboot, nothing to merge, despite what the flash offset might suggest.

Useful build options:

| Option | Effect |
|--------|--------|
| `-DUPDATER_TARGET_FW=0x0401` | Flash the older image; skips the bootloader update entirely |
| `-DUPDATER_ALLOW_BOOTLOADER_UPDATE=0` | Firmware only — never touch the chip bootloader |
| `-DUPDATER_IGNORE_SDCARD=1` | Flash anyway with a card inserted (not advised) |
| `-DUPDATER_CHUNK_DELAY_MS=10` | Space out page writes; diagnostic for a marginal supply |

Adding a board needs a `boards/<platform>/<board>/board.overlay` under the source
directory that disables unused peripherals and parks anything sharing the radio's
SPI bus. See the M9's overlay for what a shared-bus board requires.
