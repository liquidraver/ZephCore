# ADR 0004 — BLE privacy: off on nRF/MG24, on for ESP32-S3 (with identity advertising)

Status: accepted (recorded 2026-09-25)

## Context

BLE privacy (`CONFIG_BT_PRIVACY`): **keep disabled on nRF / MG24** (they already advertise the identity address,
so both iOS and Android work). **ESP32-S3 enables privacy** (`boards/common/esp32_common.conf`) because the
Espressif controller's privacy-OFF Secure-Connections path MIC-fails against iOS (HCI disconnect `0x3d` at
encryption start; root cause captured 2026-06-15). Privacy ON keeps the
controller on its working SC path. **Android "connect from app" is preserved despite privacy ON** because
advertising uses `BT_LE_ADV_OPT_USE_IDENTITY` (`adapters/ble/ZephyrBLE.cpp` `start_adv`) — we expose the stable
identity address, never an RPA.

## Decision

Privacy per platform as above; advertising always uses the identity address.

## Consequences

Do **not** drop `USE_IDENTITY` while ESP32 privacy is on, or Android connect-from-app regresses. If Espressif ever
fixes the privacy-OFF controller path, the ESP32 privacy override can be dropped (`USE_IDENTITY` stays harmless
everywhere). Never send an SMP Security Request proactively: pairing is triggered reactively via ATT error 0x05
(Apple §55).
