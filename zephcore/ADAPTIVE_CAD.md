# Adaptive CAD — Site-Calibrated Listen-Before-Talk

ZephCore's listen-before-talk (LBT) runs a hardware Channel Activity
Detection (CAD) before every transmission: the radio's LoRa correlator
searches the channel for chirps, and TX is deferred while activity is
detected. How sensitive that search is comes down to one register value,
`cadDetPeak` — and the right value depends on where the node lives.

- **Too sensitive** (detPeak too low): chirp-like interference — other
  LoRa networks, other spreading factors on the same frequency — triggers
  *false positives*. The node keeps deferring TX for nothing, wasting
  100–200 ms retry cycles and adding latency; in bad cases it hits the
  4-second CAD watchdog.
- **Not sensitive enough** (detPeak too high): real transmissions are
  *missed* and the node transmits over ongoing packets, causing on-air
  collisions.

A quiet valley node and a 50-network hilltop node need different values.
Adaptive CAD measures the site and finds the value, instead of shipping
one hardcoded number to everyone.

Important physics note: `cadDetPeak` is a correlation peak-to-noise
threshold inside the radio's despreader — **not** a dBm level. The RSSI
noise floor cannot be converted into a detPeak value, which is why this
feature measures CAD behaviour directly instead of deriving it from RSSI.

## The mental model: strong signals vs. faint ones

This is the picture that makes every knob below make sense. Skip it and
the numbers look arbitrary; read it and "SmartCAD" is one idea seen from
three angles.

**detPeak gates on correlation strength, which tracks link budget — and it
is blind to distance.** The despreader's correlation peak is essentially
the signal's SNR after processing gain, so a strong arrival produces a big
peak and a weak one a small peak, *regardless of how far it travelled*. A
90 km line-of-sight repeater that arrives strong and a 1 km neighbour
arriving strong both trip CAD; a faint straggler at the edge of
decodability, a multipath echo, or a distant node buried in noise all
produce small peaks. **Raising detPeak raises the bar to "strong only";
lowering it means "hear even the faint stuff."** There is no knob that
separates *near* from *far* — only *strong* from *faint* — because the
radio only ever knew signal strength, never distance.

**Semtech's recommended values (AN1200.48: 21–29 across all SF/BW for the
SX126x; our base is `SF+13`, which is 21 at SF8) are tuned for a
receiver** that wants to hear everything down to its sensitivity limit —
i.e. to *catch the faint*. Listen-before-talk on a busy backbone often
wants the opposite: deliberately sit *above* that band so it ignores faint
contention it would win on capture anyway. So "operating above 29" is not
a misconfiguration here; it is the point.

**Scale sanity, because the register is deceptive.** `cadDetPeak` is a
full 8-bit field (0–255), but the *useful* range is only ~18–32. The
driver's 40 ceiling is ~11 above the highest value Semtech recommends
anywhere — a near-blind guardrail, not an operating point. And the
LR11xx/LR2021 family's 56–68 numbers are a **different chip's correlation
scale**; porting them onto an SX126x makes CAD deaf. If a value feels like
it should be "mid-scale," that instinct is the trap: this scale is
compressed, not linear over 0–255.

**Why the probes only ever measure the faint side.** A calibration probe
is *skipped* whenever RSSI is more than 7 dB above the noise floor (a
strong signal is present — see the prefilter below). So every probe that
runs does so in a quiet moment, and the only thing CAD can trip on there
is a **sub-floor, faint, correlation-only-detectable** signal. That makes
the entire adaptive loop a **faint-rejection tuner**: the `busy%` you read
in `get cad` is "how often a quiet moment still held a faint signal", not
total channel occupancy. Strong signals never enter the statistics — they
were filtered out before the probe, and in real LBT they trip on their own
merits.

**So the busy cap is really a faint-tolerance dial.** `set cad.busycap`
says "keep raising detPeak until fewer than *N*% of quiet moments trip on
faint signals." A **lower** cap rejects faint traffic more aggressively
(ignore echoes and stragglers); a higher cap tolerates more of it. On a
saturated hilltop that should react only to strong contention, set the cap
low (e.g. `10`); the default `25` is a moderate setting suited to mixed
and leaf nodes. This is the same mechanism as "airtime protection" below —
two names for one lever, because the airtime a busy node wastes *is* the
airtime spent deferring for faint traffic.

**The one thing you cannot see locally.** Because strong signals are
prefiltered out of the measurement, a node can never confirm from its own
stats that it *still catches strong* — the numbers only cover the faint
side it is rejecting. The only check is behavioural: does the node still
defer for its known strong peers? If it climbs so high that even they stop
tripping, back the cap up. (This is the same "miss side is invisible"
caveat noted at the end, viewed through the strong/faint lens.)

## How it works

Every `probe.interval` seconds (default 15), when the radio is idle
in receive mode, the firmware runs one **calibration CAD probe** and
immediately re-arms RX. A probe takes one CAD duration — about 4 ms at
SF7/250 kHz, up to ~130 ms at SF12/125 kHz — so the added radio deaf time
is roughly 0.1%. Probes also run in RX duty-cycle (sniff) mode: the
duty cycle is briefly interrupted and re-armed, within the same
preamble-catch budget philosophy the sniff-mode math already accepts.

Each probe tests one **level**: a signed offset from the chip family's
per-SF base detPeak (SX126x: `SF+13`; LR11xx/LR2021: the 56–68 table).
Results accumulate per level:

- `probes` — how many CADs ran at this level
- `busy` — raw "activity detected" verdicts
- `fp` — suspected **false positives**: busy verdicts where no packet
  materialised right after (see filtering below)
- `tp` — **true positives**: busy verdicts confirmed by actual RX
  activity immediately after the probe

Filtering, because a "busy" verdict during apparent silence may be a real
below-noise-floor packet (detecting those is CAD's whole purpose):

1. Probes are skipped entirely when the channel is visibly busy (RSSI
   more than 7 dB above the learned noise floor) or a packet is being
   received — those teach nothing about false positives. (This is also the
   prefilter that limits probing to the faint regime; see the mental model
   above.)
2. After a busy verdict, RX is restarted and the firmware watches a
   ~12-symbol window for either signal of a real transmitter: (a) the
   restarted RX syncs on it, **or** (b) instantaneous RSSI climbs above the
   noise floor. If neither appears, the verdict counts as a suspected false
   positive. Path (b) matters: tearing RX down to run the probe routinely
   eats a real packet's preamble, so RX often fails to re-sync on a signal
   that is genuinely there — an earlier `isReceiving()`-only check booked
   those as false positives, a detPeak-independent floor that flattened the
   FP curve and drove the staircase into the ceiling. Channel energy does
   not depend on winning the preamble race, so it recovers them. Residual
   contamination by real below-floor traffic biases the estimate
   *conservative* (higher detPeak), which is the safe direction.

Statistics decay (halve) every 6 hours so the picture stays fresh, and
reset completely whenever the radio parameters change (frequency, SF, BW
— the data is only valid for one configuration).

### Auto mode (staircase) — ON by default

`cad.auto` ships **on**, on repeaters and companions alike. A one-sided
staircase controller acts on the probe stats:

The controller is **knee-seeking**. The false-positive-vs-detPeak curve
falls as detPeak rises (less sensitive → fewer false detects) and flattens
past a knee; the sweet spot is that knee — the most sensitive detPeak whose
FP has already bottomed out. Probes sample the operating level and **both
neighbours** (op−1 more sensitive, op+1 less sensitive; op weighted half, each
neighbour a quarter) so the staircase can read the local curve *slope*:

- **Airtime protection** (highest priority): step up when the operating
  level's *total* busy rate — real traffic plus false positives — exceeds
  the busy cap (`set cad.busycap`, percent; default **25%**, 0 = off). The
  knee logic only minimises *false* busy, but on a congested hilltop most
  busy verdicts are real distant traffic we'd win on capture anyway;
  deferring for all of it starves the node's own airtime. Self-targeting —
  a quiet node's busy rate never reaches the cap, so only a genuinely busy
  backbone backs off. Shown as `bc:` in `get cad`.
- **Step up** (less sensitive) when the level above is markedly cleaner —
  FP drops ≥5%/level (`CAD_KNEE_SLOPE_PERMILLE`). That means we're on the
  steep part below the knee; climb toward it.
- **Step down** (more sensitive) only on a flat plateau (frontier no worse
  than operating, slope <5%/level) that is **already clean** (FP ≤5%,
  `CAD_PLATEAU_CLEAN_PERMILLE`) — reclaim sensitivity that costs nothing.
- Otherwise **hold** — either at the knee (steep below, flat above) or on a
  noisy flat plateau.

Using slopes rather than an absolute FP target makes convergence independent
of a site's FP *floor* (which varies with traffic and with the classifier's
residual false-positive rate). The clean-plateau guard is what stops a
flat-but-noisy curve from walking to the sensitive rail: there, holding is the
least-bad move, while a genuinely quiet flat-low site correctly descends to
the floor. Each involved rung needs ≥120 samples (`CAD_STEP_MIN_PROBES`)
before a step, so a decision lands roughly every 1–2 hours at the default
interval.

A step decision requires the operating rung (and, for the direction chosen,
its neighbour) to be warm; `get cad`'s three-rung window shows exactly those
levels, so the slope the controller is acting on is visible directly. The
offset is persisted to flash whenever it steps.

The offset is clamped to **−8…+12** levels around the family base — wide
enough that a dense hilltop can settle much less sensitive and a quiet
valley node much more sensitive. The driver additionally clamps the
absolute detPeak (SX126x 15–40, LR11xx/LR20xx 48–90). That absolute clamp
is a *firmware guardrail*, not a chip limit — `cadDetPeak` is a full
8-bit register (0–255) — it simply stops the staircase from wandering
into "CAD never fires" (detPeak too high → LBT effectively off) or "CAD
always busy" (too low → node can't transmit) territory. Quiet sites
converge *below* the standard value — more sensitive LBT than any
fixed-config firmware, meaning fewer TX-over-RX stomps. Noisy sites
ratchet up until false positives vanish.

To watch or hand-tune instead of letting it self-adjust, set
`cad.auto off` (see below).

## Reading the status

Out of the box the node calibrates itself — you don't have to do
anything. To see what it's doing:

```
get cad
```

Compact output (kept short so it survives a truncated LoRa reply):

```
> a:on o:-1 pk:20(b21/4s) sp:0.9/84%(312) bc:25%
 -2(19) 241p 9b 7f 2t 3%
*-1(20) 900p 5b 3f 2t 0%
 +0(21) 300p 2b 0f 2t 0%
```

Header: `a` auto on/off · `o` operating offset · `pk` operating detPeak ·
`b` family base · `4s` 4 symbols · `sp` RSSI burst quality · `bc` busy cap.

**`sp` — are the noise-floor sampler's reads independent?** The floor
sampler takes a median of 8 RSSI reads. The chip refreshes RSSI only once
per averaging window (~16 µs at BW 62.5, ~134 µs at BW 7.8); reads issued
faster than that return the same underlying value repeatedly and the median
collapses to a single read. `sp` reports mean spread (max−min) across a
burst, the share of bursts whose spread was 0, and the burst count.

Read it this way:

- **A non-zero mean proves the reads are independent**, however high the
  zero-spread share climbs. A quiet or steadily-occupied channel genuinely
  reads the same value 8 times at integer-dB resolution — that is correct,
  not broken.
- **Mean 0.0 with a high share is the fault signature**: no burst ever
  spans anything, i.e. 8 copies of one sample.
- The count matters. The burst rate is not derivable from uptime, because
  the sampler's guards (TX, mid-RX, duty-cycle sleep) block an unknown
  fraction of attempts, so a share without its denominator is unreadable.
  It appears on the local USB console only — a remote reply is capped at
  161 B and the three level rows have first claim on it. All three
  counters halve together at 8192 bursts, so the count stays four digits
  and the figures describe a recent window rather than everything since
  boot.

Measured on-air at BW 62.5: `0.6/90%` on a quiet channel, `0.9/84%` with
the floor at −103 dBm. The share falls and the spread rises as ambient
comes up — independent reads, responding the right way.

Note what this median does and does not do. The burst spans ~300 µs against
a ~200 ms SF8/BW62.5 packet — about 0.15% of one transmission — so a
neighbour's packet is either wholly inside the burst or wholly outside it,
every read sees the same level, and the median returns it rather than
rejecting it. The median rejects sub-300 µs glitches and bad SPI reads.
Interference is handled by the mid-receive guard and the +14 dB threshold
filter, not here.

The probe interval used to sit in this slot; it moved out because it is a
pref you set, readable with `get probe.interval`, whereas burst spread is
only observable from inside the sampler.
Then a **three-rung window** centred on the operating offset (`*` marks
it): the frontier (one step more sensitive), the operating level, and one
step less sensitive — exactly the three rungs the staircase reads to judge
the local slope. Per level: `level(peak) probes busy fp tp fp%%` — `fp` =
suspected false positives, `tp` = busy verdicts confirmed by a real
signal, `fp%%` = integer false-positive rate.

Here the staircase has already stepped to offset −1 (peak 20): the −1
level is clean (0%) over 900 probes while −2 shows a 3% jump, so −1 is
this site's knee. Remember (mental model above) these numbers describe the
*faint* regime only — strong signals were prefiltered out before probing.

### Dry-run / manual tuning

Prefer to observe or pin the value yourself? Turn the staircase off:

```
set cad.auto off
```

Probing continues (stats keep updating) but the operating offset no
longer moves. In dry-run the probe sweep covers levels −4…+4 evenly, so
`get cad` shows the whole false-positive-vs-detPeak curve. Read it
bottom-up: the level where `fp%%` jumps from ~0 is the knee; sit one step
above it. Then pin it:

```
set cad.offset -1
```

Negative offset = more sensitive LBT (catches weaker signals, risks false
busy); positive = less sensitive. Range −8…+12. Re-enable the staircase
any time with `set cad.auto on`. Offset changes (manual or automatic)
appear in the log as `cad: step down/up -> offset N`.

To collect dry-run data faster, drop the interval (probing is temporary
then): `set probe.interval 10`. At the default 15 s, allow ~a day for
a knee to resolve clearly, longer to capture day/night variation.

## Command reference

| Command | Default | Description |
|---|---|---|
| `get cad` | | Status + per-level statistics (see above). |
| `set cad.auto <on\|off>` | **on** | Staircase controller acts on the stats. Off = observe/hand-tune. |
| `set cad.offset <n>` | 0 | Operating offset, −8…12. Negative = more sensitive. Applied live. |
| `set probe.interval <sec>` | 15 | Shared cadence for the noise-floor sample and the CAD probe that consumes it; 0 disables probing (and freezes auto), 10–255 otherwise. |
| `set cad.busycap <pct>` | 25 | Faint-tolerance / airtime cap: raise detPeak once more than this % of (quiet-moment) probes trip on faint signals. Lower = reject faint/echo harder (busy backbones); 0 = off. 10–90 otherwise. |
| `set cad.reset` | | Clear accumulated statistics (RAM only). |

All settings persist in prefs and apply to every role — repeater, room
server, and companion (companions reach the CLI via the v-contact admin
chat or USB serial). Command keywords are case-insensitive (`Get cad`
works); command *values* like passwords and node names are not, so `on`
/`off` must be lowercase.

## Notes and limits

- **SX127x boards** (TTGO LoRa32, T-Beam classic) have no hardware CAD;
  `get cad` reports `not available` and the settings are inert. Their LBT
  remains the RSSI-based `int.thresh` gate.
- **The false-positive side is measured; the miss side mostly is not.**
  A too-high detPeak shows up as on-air collisions, which a single node
  cannot observe. That is why the workflow is "find the lowest clean
  level and sit at it", never "raise it until problems stop being
  visible". The `tp` column is the one local miss-side signal: probes
  that detected real below-noise-floor traffic.
- **detPeak is the only adaptive knob.** `cadDetMin` stays at Semtech's
  universal 10; the symbol count is fixed at 4 (better mid-payload
  detection than 2 — most of a mesh packet's airtime is payload, and
  that is where a pre-TX CAD usually lands). The drivers scale their
  CAD timeout with symbol count and preset automatically.
- Statistics are RAM-only and restart after a reboot; the learned
  *offset* is persisted. Mixed fleets are fine — this feature changes
  only when *this* node decides the channel is busy, not anything on
  the air.
- Related but separate: `int.thresh` (RSSI-above-floor software gate)
  and `dc.restarts` (sniff-mode false-preamble counter) still work as
  before; adaptive CAD complements them.
