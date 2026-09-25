# Architecture decision records

One decision per file: Context / Decision / Consequences / Status. Recorded 2026-09-25 at the end of the
restructure; most describe decisions taken earlier. The system view is in [../DESIGN.md](../DESIGN.md).

| # | Decision |
|---|---|
| [0001](0001-esp32-boot-and-flash-layout.md) | ESP32 boot: simple boot for plain builds, MCUboot for OTA builds and releases |
| [0002](0002-esp32s3-native-usb-companion.md) | ESP32-S3 companions use native USB; `start dfu` restores USB-Serial-JTAG |
| [0003](0003-devicetree-overlay-precedence.md) | Devicetree overlays: board.overlay first, conf-paired after it |
| [0004](0004-ble-privacy.md) | BLE privacy off on nRF/MG24, on for ESP32-S3 (identity advertising) |
| [0005](0005-zephyr-main-pin.md) | Track Zephyr `main`, pinned; the migration changes not to undo |
| [0006](0006-sx126x-rx-duty-cycle-and-busy-gating.md) | SX126x RX duty cycle and four-layer RX-busy gating |
| [0007](0007-adaptive-contention-window.md) | Adaptive contention window instead of static tx/rx delays |
| [0008](0008-storage-and-settings-backend.md) | Storage layout, power-loss writes, BLE bonds in NVS |
| [0009](0009-wifi-companion-boards.md) | WiFi companion boards and contact counts |
