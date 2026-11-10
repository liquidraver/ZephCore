# ADR 0007 — Adaptive contention window instead of Arduino's static tx/rx delays

Status: accepted (recorded 2026-09-25; ZephCore-only); amended 2026-11-10 (reactive backoff retired)

## Context

Replaces Arduino's static `txdelay`/`rxdelay` with adaptive mechanisms:

- **EMA-based flood delay factor**: scales flood TX retransmit jitter adaptively based on local contention
  history. Jitter is double-capped: `min(2000ms, 6·airtime)` for repeaters — the airtime-scaled ceiling keeps
  SF7/narrow-BW configs from wasting time in oversized windows, while the absolute cap bounds per-hop latency in
  dense areas. Ring buffer holds 24 concurrent tracked retransmits (sized for ~50-neighbor hilltops).
- **Per-dupe reactive backoff** (retired 2026-11-10, see Amendment): each heard duplicate of a pending TX packet
  delayed transmission by `backoff_multiplier * airtime` from NOW, capped at `min(2000ms, 12·airtime)` total
  reactive extension per packet.
- **Initial-flood jitter (companion-only)**: companions don't forward floods (when `client_repeat=0`) and use a
  fixed `20..150ms` window for originated TX — small enough to feel responsive, large enough to avoid lockstep
  collisions with nearby repeaters. Passive flood tracking is disabled in that mode (no EMA warming needed). When
  `client_repeat=1`, the EMA is warmed naturally via the forwarding path and `getRetransmitDelay()` falls back to
  the same adaptive math as repeaters.
- **Direct packets**: use minimal fixed jitter only (no adaptive scaling).

## Decision

The old CLI commands (`txdelay`, `rxdelay`, `direct.txdelay`) are accepted for prefs compatibility but ignored;
`get txdelay` shows the adaptive state.

## Consequences

A divergence from upstream by design; ports touching `Mesh`/`Dispatcher` timing must keep the adaptive flood jitter.

## Amendment (2026-11-10): reactive backoff retired

The per-dupe reactive backoff (`Mesh::extendPendingRetransmit`, the headroom in `ContentionTracker`,
`PacketManager::rescheduleOutbound`) and its `get/set backoff.multiplier` commands are removed. It was meant to
step a queued forward off the end-of-packet rush when nodes transmitted the instant the channel freed. Since
1.18.3 the node waits for every reception to end and then a random 1-10 CAD slots counted from that end
(LLD 02 §19-§20, ADR 0006). The per-dupe shift was the same for every node that heard the copy, so it lined
neighbours up instead of spreading them, and it stacked per copy past the slot window.

Measured on the roof repeater BlueLake: 10 min on 0.2 against 7.7 h on 0, 900 transmits. Waits, the share of
post-reception gaps won and the receptions a held packet sat through matched within noise at equal traffic.
At 44-60 receptions/min the median wait was 2.1 s on 0 against 2.5 s on 0.2, and the share of gaps won was
11.9% against 14.7%. The proactive EMA flood jitter stays. It is what spreads neighbours' first forwards.
`NodePrefs::backoff_multiplier` keeps its place in the stored prefs (binary layout and the JSON `backoff` key)
so a downgrade reads a valid value; nothing reads it.
