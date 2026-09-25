#include "test.h"
#include <CadController.h>
#include <NoiseFloorEstimator.h>

using mesh::CadController;
using mesh::NoiseFloorEstimator;

namespace {
struct FakeCadHw : mesh::CadHw {
    uint8_t base = 20, pmin = 0, pmax = 0;
    int programmed = 99;
    int writes = 0;
    uint8_t hwCadBasePeak() override { return base; }
    uint8_t hwCadPeakMin() override { return pmin; }
    uint8_t hwCadPeakMax() override { return pmax; }
    void hwCadSetPeakOffset(int8_t offset) override { programmed = offset; ++writes; }
};
// Books `probes` probes at `level`, `busy` of them busy, `fp` of those false positives.
void feed(CadController& c, int8_t level, int probes, int busy, int fp) {
    for (int i = 0; i < probes; ++i) c.recordProbe(level, i < busy);
    for (int i = 0; i < fp; ++i) { c.setPending(level, 0); c.resolvePending(2); }
}
}  // namespace

TEST(noise_seed, "UNIT-NOISE-001", "Noise floor seeds from the first sample within the clamp window") {
    NoiseFloorEstimator e;
    CHECK(!e.seeded() && e.floor() == DEFAULT_NOISE_FLOOR && e.isQuiet(-20));
    CHECK(e.update(-130, -123) == NoiseFloorEstimator::SEEDED && e.floor() == -123);
    NoiseFloorEstimator hot;
    CHECK(hot.update(-40, -123) == NoiseFloorEstimator::SEEDED && hot.floor() == NOISE_FLOOR_CEILING_DBM);
    NoiseFloorEstimator mid;
    mid.update(-104, -123);
    CHECK(mid.floor() == -104 && mid.tick() == 0);
}
TEST(noise_gate, "UNIT-NOISE-002", "Warm-up accepts everything then outliers are rejected except the periodic bypass") {
    NoiseFloorEstimator e;
    e.update(-110, -130);
    const int W = 1 << NOISE_FLOOR_EMA_SHIFT, P = NOISE_FLOOR_UNGUARDED_INTERVAL;
    for (int i = 0; i < W; ++i) CHECK(e.update(-60, -130) == NoiseFloorEstimator::UPDATED);
    int warmed = e.floor();
    CHECK(warmed > -110);
    // Ticks W..P-1: a sample SAMPLING_THRESHOLD above the floor is interference.
    for (int t = W; t < P; ++t)
        CHECK(e.update(warmed + NOISE_FLOOR_SAMPLING_THRESHOLD, -130) == NoiseFloorEstimator::REJECTED);
    CHECK(e.floor() == warmed);
    // Tick P passes unfiltered so a sustained rise is still tracked.
    CHECK(e.update(warmed + NOISE_FLOOR_SAMPLING_THRESHOLD, -130) == NoiseFloorEstimator::UPDATED);
    CHECK(e.floor() > warmed);
    // Just below the threshold is never interference.
    int f = e.floor();
    CHECK(e.update(f + NOISE_FLOOR_SAMPLING_THRESHOLD - 1, -130) == NoiseFloorEstimator::UPDATED);
}
TEST(noise_rounding, "UNIT-NOISE-003", "EMA step rounds to nearest symmetrically with a +-3 dB dead zone") {
    NoiseFloorEstimator down; down.update(-100, -130);
    for (int i = 0; i < 200; ++i) down.update(-110, -130);
    CHECK(down.floor() == -107);
    NoiseFloorEstimator up; up.update(-110, -130);
    for (int i = 0; i < 7; ++i) up.update(-100, -130);  // stay inside warm-up
    NoiseFloorEstimator one; one.update(-100, -130);
    one.update(-96, -130); CHECK(one.floor() == -99);   // (+4 + 4) / 8 = +1
    one.update(-103, -130); CHECK(one.floor() == -100); // (-4 - 4) / 8 = -1
    one.update(-97, -130); CHECK(one.floor() == -100);  // (+3 + 4) / 8 = 0
    CHECK(up.floor() > -110 && up.floor() <= -103);
}
TEST(noise_median_quiet, "UNIT-NOISE-004", "Median of an even burst and the quiet verdict against the floor") {
    int16_t s[8] = {-90, -110, -100, -120, -95, -105, -115, -101};
    CHECK(mesh::sortAndMedian(s, 8) == -103);  // (-105 + -101) / 2, truncated toward zero
    for (int i = 1; i < 8; ++i) CHECK(s[i - 1] <= s[i]);
    NoiseFloorEstimator e; e.update(-110, -130);
    CHECK(e.isQuiet(-110 + CAD_PROBE_RSSI_GUARD) && !e.isQuiet(-110 + CAD_PROBE_RSSI_GUARD + 1));
}

TEST(cad_window, "UNIT-CAD-001", "Level window narrows only where the driver clamp binds") {
    FakeCadHw hw; CadController c(hw);
    CHECK(c.levelMinEff() == CAD_LEVEL_MIN && c.levelMaxEff() == CAD_LEVEL_MAX);
    hw.pmin = 12; hw.pmax = 48;
    CHECK(c.levelMinEff() == CAD_LEVEL_MIN);           // 20 - 8 = 12, not below the clamp
    hw.base = 18; CHECK(c.levelMinEff() == -6);        // 18 - 8 = 10 < 12
    hw.base = 40; CHECK(c.levelMaxEff() == 8);         // 40 + 12 = 52 > 48
    hw.base = 0; CHECK(c.levelMinEff() == CAD_LEVEL_MIN && c.levelMaxEff() == CAD_LEVEL_MAX);
}
TEST(cad_reanchor, "UNIT-CAD-002", "Configure keeps the learned peak across a base change and clamps to the window") {
    FakeCadHw hw; hw.base = 44; CadController c(hw);
    c.configure(true, -7, 15, 51);
    CHECK(c.offset() == 0 && hw.programmed == 0 && c.autoEnabled() && c.busycapPct() == 15);
    c.configure(true, -7, 15, 0);                      // unknown base: no guess
    CHECK(c.offset() == -7);
    c.configure(true, -7, 15, 44);                     // same base
    CHECK(c.offset() == -7);
    hw.pmin = 40;                                      // min eff = -4
    c.configure(false, -7, 0, 0);
    CHECK(c.offset() == -4 && hw.programmed == -4);
    hw.pmin = 0; c.configure(false, 12, 0, 20);        // peak 32 is offset -12 at base 44: clamped
    CHECK(c.offset() == CAD_LEVEL_MIN);
}
TEST(cad_knee, "UNIT-CAD-003", "Staircase steps up below the knee, down on a clean plateau and holds at the knee") {
    FakeCadHw hw;
    { CadController c(hw); c.configure(true, 0, 0, 0);
      feed(c, 0, 200, 60, 60); feed(c, 1, 200, 20, 20);
      c.staircaseStep(); CHECK(c.offset() == 1 && hw.programmed == 1); }
    { CadController c(hw); c.configure(true, 0, 0, 0);
      feed(c, 0, 200, 4, 4); feed(c, -1, 200, 8, 8);
      c.staircaseStep(); CHECK(c.offset() == -1 && hw.programmed == -1); }
    { CadController c(hw); c.configure(true, 0, 0, 0);          // steep below, flat above
      feed(c, 0, 200, 4, 4); feed(c, -1, 200, 40, 40); feed(c, 1, 200, 2, 2);
      c.staircaseStep(); CHECK(c.offset() == 0); }
    { CadController c(hw); c.configure(true, 0, 0, 0);          // flat but noisy
      feed(c, 0, 200, 20, 20); feed(c, -1, 200, 22, 22);
      c.staircaseStep(); CHECK(c.offset() == 0); }
    { CadController c(hw); c.configure(true, 0, 0, 0);          // not warm yet
      feed(c, 0, CAD_STEP_MIN_PROBES - 1, 60, 60); feed(c, 1, 200, 0, 0);
      c.staircaseStep(); CHECK(c.offset() == 0); }
}
TEST(cad_busycap_hysteresis, "UNIT-CAD-004", "Descent into the airtime cap is blocked by the hysteresis band") {
    FakeCadHw hw;
    // cap 25% -> descend only if the frontier's busy rate is <= 15%.
    { CadController c(hw); c.configure(true, 0, 25, 0);
      feed(c, 0, 200, 4, 4); feed(c, -1, 200, 40, 8);
      c.staircaseStep(); CHECK(c.offset() == 0); }
    { CadController c(hw); c.configure(true, 0, 25, 0);
      feed(c, 0, 200, 4, 4); feed(c, -1, 200, 30, 8);
      c.staircaseStep(); CHECK(c.offset() == -1); }
    { CadController c(hw); c.configure(true, 0, 10, 0);        // cap 10% still allows a 6% frontier
      feed(c, 0, 200, 4, 4); feed(c, -1, 200, 12, 8);
      c.staircaseStep(); CHECK(c.offset() == -1); }
}
TEST(cad_safety, "UNIT-CAD-005", "Safety rung steps up on a pathological or over-cap level regardless of auto") {
    FakeCadHw hw;
    { CadController c(hw); c.configure(false, 0, 0, 0);
      feed(c, 0, CAD_SAFETY_MIN_PROBES - 1, CAD_SAFETY_MIN_PROBES - 1, 0);
      CHECK(!c.safetyStep());
      c.recordProbe(0, true);                                 // 20 probes, all busy
      CHECK(c.safetyStep() && c.offset() == 1 && hw.programmed == 1); }
    { CadController c(hw); c.configure(false, 0, 10, 0);
      feed(c, 0, CAD_STEP_MIN_PROBES - 1, 13, 0); CHECK(!c.safetyStep());
      c.recordProbe(0, true); CHECK(c.safetyStep() && c.offset() == 1); }
    { CadController c(hw); c.configure(false, 0, 0, 0);       // busycap 0: marginal is left alone
      feed(c, 0, 200, 100, 0); CHECK(!c.safetyStep()); }
    { CadController c(hw); c.configure(false, CAD_LEVEL_MAX, 0, 0);
      feed(c, CAD_LEVEL_MAX, 50, 50, 0); CHECK(!c.safetyStep()); }
    { CadController c(hw); c.configure(false, 0, 0, 0);       // adapt(): safety only when auto is off
      feed(c, 0, 20, 20, 0); feed(c, 1, 200, 0, 0);
      c.adapt(); CHECK(c.offset() == 1); }
}
TEST(cad_visit_relax, "UNIT-CAD-006", "Visits use their own offset and TX starvation relaxes the offset in force") {
    FakeCadHw hw; CadController c(hw);
    c.configure(true, 2, 0, 0);
    c.beginVisit(true);
    CHECK(c.visiting() && c.effectiveOffset() == 0 && hw.programmed == 0);
    CHECK(c.relaxOnTxStarvation() && c.effectiveOffset() == 1 && hw.programmed == 1 && c.offset() == 2);
    c.configure(true, 3, 0, 0);                               // set cad.* during a visit
    CHECK(c.offset() == 3 && hw.programmed == 1);
    c.endVisit();
    CHECK(!c.visiting() && hw.programmed == 3);
    c.beginVisit(false);
    CHECK(c.effectiveOffset() == 3);
    c.endVisit();
    CHECK(c.relaxOnTxStarvation() && c.offset() == 4 && hw.programmed == 4);
    c.configure(false, CAD_LEVEL_MAX, 0, 0);
    CHECK(!c.relaxOnTxStarvation() && c.offset() == CAD_LEVEL_MAX);
}
TEST(cad_probe_levels, "UNIT-CAD-007", "Probe levels sweep when manual and mix operating and neighbours when auto") {
    FakeCadHw hw;
    { CadController c(hw); c.configure(false, 0, 0, 0);
      int seen[CAD_SWEEP_MAX - CAD_SWEEP_MIN + 1] = {};
      for (int i = 0; i < CAD_SWEEP_MAX - CAD_SWEEP_MIN + 1; ++i) {
          int l = c.pickProbeLevel(); CHECK(l >= CAD_SWEEP_MIN && l <= CAD_SWEEP_MAX); ++seen[l - CAD_SWEEP_MIN];
      }
      for (int n : seen) CHECK(n == 1); }
    { CadController c(hw); c.configure(false, -8, 0, 0);
      for (int i = 0; i < 5; ++i) CHECK(c.pickProbeLevel() == -8); }
    { CadController c(hw); c.configure(true, 0, 0, 0);
      int expected[] = {-1, 0, 1, 0, -1, 0, 1, 0};
      for (int l : expected) CHECK(c.pickProbeLevel() == l); }
    { hw.pmin = 20; CadController c(hw); c.configure(true, 0, 0, 0);   // min eff 0: no frontier
      int expected[] = {0, 0, 1, 0};
      for (int l : expected) CHECK(c.pickProbeLevel() == l);
      hw.pmin = 0; }
}
TEST(cad_pending, "UNIT-CAD-008", "Pending verdicts book tp or fp, unresolved ones are withdrawn and reset drops them") {
    FakeCadHw hw; CadController c(hw);
    CHECK(!c.hasPending());
    c.recordProbe(0, true); c.setPending(0, 500);
    CHECK(c.hasPending() && c.pendingLevel() == 0 && c.pendingDeadline() == 500);
    c.resolvePending(1);
    CHECK(!c.hasPending() && c.stats(0).tp == 1 && c.stats(0).fp == 0);
    c.recordProbe(0, true); c.setPending(0, 900); c.resolvePending(2);
    CHECK(c.stats(0).fp == 1 && c.stats(0).probes == 2 && c.stats(0).busy == 2);
    c.recordProbe(0, true); c.setPending(0, 900); c.resolvePending(0);
    CHECK(c.stats(0).probes == 2 && c.stats(0).busy == 2);
    c.recordProbe(0, true); c.setPending(0, 900);
    c.resetStats();
    CHECK(!c.hasPending() && c.pendingDeadline() == 0 && c.stats(0).probes == 0);
    for (int i = 0; i < 0xFFF0; ++i) c.recordProbe(3, false);
    CHECK(c.stats(3).probes == 0xFFF0);
    c.recordProbe(3, true);
    CHECK(c.stats(3).probes == 0x7FF9 && c.stats(3).busy == 1);
    c.decayStats();
    CHECK(c.stats(3).probes == 0x3FFC && c.stats(3).busy == 0);
}
