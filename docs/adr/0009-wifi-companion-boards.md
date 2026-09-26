# ADR 0009 — WiFi companion: which boards, at what contact count

Status: accepted (recorded 2026-09-25)

## Context

**WiFi companion** (`CONFIG_ZEPHCORE_COMPANION_WIFI`, `boards/common/wifi_companion.conf`): auto-added to
companion builds of boards whose `zephcore.yml` declares `capabilities: wifi: true`. On the PSRAM ESP32-S3
boards the WiFi heap, network buffers and the `CompanionMesh` object go to PSRAM. Four no-PSRAM boards also
declare it (Heltec V3 and Wireless Tracker at 140/130 contacts, both C6 boards at 280; the offline queue stays
256): each sets `ZEPHCORE_MAX_CONTACTS` in `board.conf` to the largest count that links, and an S3 without PSRAM
also gets bounded WiFi buffers and a measured 72 KB heap (Kconfig, `HEAP_MEM_POOL_IGNORE_MIN`). Tracker V2, C3
and the classic ESP32 boards do not fit WiFi + BLE (DRAM). Upstream's
`set/get wifi.ssid|pwd|enabled`, `set wifi.clear`, `get wifi.status|ip`; applied on reboot; TCP port
`CONFIG_ZEPHCORE_TCP_PORT` (5000, as upstream). The Zephyr ESP32 WiFi stack needed patches `0017`/`0018` and
hal_espressif `0001` to survive traffic bursts; since the 2026-09-26 Zephyr bump only `0017` remains (the other
two fixes are upstream, `67151dfaf73` and hal_espressif `af97fb02df`) (§13).

## Decision

WiFi + BLE on every board that can hold both; fewer contacts on the no-PSRAM ones rather than dropping WiFi.

## Consequences

Watch patch `0017` on every west update: upstream's RX path still blocks for 100 ms.
