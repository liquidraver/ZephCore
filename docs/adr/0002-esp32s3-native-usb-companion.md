# ADR 0002 — ESP32-S3 companions use native USB; `start dfu` restores USB-Serial-JTAG

Status: accepted (recorded 2026-09-25)

## Context

**ESP32-S3 companions speak the companion protocol over native USB by default** (since 2026-08-28).
`zephcore/CMakeLists.txt` auto-includes `boards/common/esp32s3_usb.conf` for **companion** builds on every S3
board whose `board.overlay` includes `esp32s3_usb_otg.dtsi` — `heltec_wifi_lora32_v4/v43/v4_r8`,
`heltec_wireless_tracker_v2`, `xiao_esp32s3`, `station_g2`, `lilygo_t3s3`, `meshnology_w12`.
Repeater/observer/room-server and **debug** builds are excluded (they keep the port for the console; ESP32 has no
RTT). This is in CMakeLists rather than `build.sh` on purpose, so a locally built companion is the same firmware
as the release. Cost: +14 KB DRAM (worst board lands at 94%), and the port is no longer USB-Serial-JTAG, **so
esptool's auto-reset into download mode does not work** — use `start dfu` or an Arduino-style 1200-baud touch
(see `ZephyrBoard::rebootToBootloader`), or the BOOT button. Both paths set `FORCE_DOWNLOAD_BOOT` **and** clear
the RTC-domain USB PHY mux (`RTC_CNTL_USB_CONF_REG` `SW_HW_USB_PHY_SEL`/`SW_USB_PHY_SEL`), so the ROM comes back
on USB-Serial-JTAG (`303a:1001`) and ordinary `esptool write-flash` works again with its normal auto-reset.
Clearing the mux is **not** optional: Zephyr's DWC2 quirk layer claims the internal PHY for USB OTG at init,
those bits live in the same always-on RTC domain as `FORCE_DOWNLOAD_BOOT` and survive the reboot, so before
2026-09-04 `start dfu` landed in ROM **USB-OTG** download mode (`303a:0009`) instead. That state is flashable
(esptool reports `USB mode: USB-OTG` and loads the stub) but needs `--before no-reset`, and browser flashers
cannot drive it at all because esptool-js special-cases only PID `0x1001`.

## Decision

Native USB CDC for S3 companions (not debug/server builds), selected in CMakeLists; the reboot-to-bootloader path
restores both the download flag and the PHY mux.

## Consequences

Flash an S3 companion with a 1200-baud touch (or `start dfu`), then esptool on the ROM's USB-Serial-JTAG port.
