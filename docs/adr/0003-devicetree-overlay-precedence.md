# ADR 0003 — Devicetree overlays: board.overlay first, conf-paired overlays after it

Status: accepted (recorded 2026-09-25)

## Context

`board.overlay` goes into `EXTRA_DTC_OVERLAY_FILE` **first**, so overlays paired with later confs
(`repeater`/`pm_esp32`/`esp32s3_usb`/`no_display`) override it — matching the conf chain's own "user extras last"
rule. This was the opposite order until 2026-08-28, which silently broke `esp32s3_usb.overlay`'s console reroute
on the boards that choose a console in `board.overlay`, and `pm_esp32.overlay`'s SX1262 NSS sleep-hold on
`heltec_wireless_tracker`. `partitions.overlay` stays last.

## Decision

Overlay order mirrors the conf order: board first, conf-paired overlays next, `partitions.overlay` last.

## Consequences

A conf-paired overlay can always override a board default; a board that must win has to change its board.conf
pairing, not the order.
