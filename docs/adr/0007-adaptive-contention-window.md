# ADR 0007 — Adaptive contention window instead of Arduino's static tx/rx delays

Status: accepted (recorded 2026-09-25; ZephCore-only)

## Context

Replaces Arduino's static `txdelay`/`rxdelay` with three mechanisms:

- **EMA-based flood delay factor**: scales flood TX retransmit jitter adaptively based on local contention
  history. Jitter is double-capped: `min(2000ms, 6·airtime)` for repeaters — the airtime-scaled ceiling keeps
  SF7/narrow-BW configs from wasting time in oversized windows, while the absolute cap bounds per-hop latency in
  dense areas. Ring buffer holds 24 concurrent tracked retransmits (sized for ~50-neighbor hilltops).
- **Per-dupe reactive backoff**: each heard duplicate of a pending TX packet delays transmission by
  `backoff_multiplier * airtime` from NOW. Capped at `min(2000ms, 12·airtime)` total reactive extension per
  packet. After the cap, CAD handles channel-busy detection like Arduino.
- **Initial-flood jitter (companion-only)**: companions don't forward floods (when `client_repeat=0`) and use a
  fixed `20..150ms` window for originated TX — small enough to feel responsive, large enough to avoid lockstep
  collisions with nearby repeaters. Passive flood tracking is disabled in that mode (no EMA warming needed). When
  `client_repeat=1`, the EMA is warmed naturally via the forwarding path and `getRetransmitDelay()` falls back to
  the same adaptive math as repeaters.
- **Direct packets**: use minimal fixed jitter only (no adaptive scaling).

## Decision

`set backoff.multiplier X` controls per-dupe delay (default 0.2; 0.0 disables reactive backoff). The old CLI
commands (`txdelay`, `rxdelay`, `direct.txdelay`) are accepted for prefs compatibility but ignored; `get txdelay`
shows the adaptive state.

## Consequences

A divergence from upstream by design (memory `acw-reactive-backoff`); ports touching `Mesh`/`Dispatcher` timing
must keep it.
