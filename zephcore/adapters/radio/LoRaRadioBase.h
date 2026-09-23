/*
 * SPDX-License-Identifier: MIT
 * LoRa radio base class — shared state and algorithms.
 * Subclasses implement hw*() primitives only.
 */

#pragma once

#include <mesh/Dispatcher.h>
#include <mesh/MeshCore.h>
#include <NodePrefs.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "radio_common.h"

namespace mesh {

class LoRaRadioBase : public Radio {
public:
	LoRaRadioBase(const struct device *lora_dev, MainBoard &board,
		      NodePrefs *prefs = nullptr);

	void setPrefs(NodePrefs *prefs) { _prefs = prefs; }

	/* Callbacks */
	void setRxCallback(RadioRxCallback cb, void *user_data) {
		_rx_cb = cb;
		_rx_cb_user_data = user_data;
	}
	void setTxDoneCallback(RadioTxDoneCallback cb, void *user_data) {
		_tx_done_cb = cb;
		_tx_done_cb_user_data = user_data;
	}

	/* Radio interface (all implemented in base) */
	void begin() override;
	void reconfigure();
	void reconfigureWithParams(float freq, float bw, uint8_t sf, uint8_t cr);

	/* Temporary radio override — applies freq/bw/sf/cr without mutating
	 * _prefs.  Used by tempradio so the saved prefs survive intact and
	 * concurrent savePrefs() calls don't poison flash.  clearRadioOverride()
	 * reverts to whatever _prefs holds at that moment. */
	/* visiting_new_preset = the override moves the radio onto a preset it
	 * was not already running (`tempradio`), so the learned detPeak offset
	 * does not apply there and the visit starts at that preset's own base.
	 * Pass false when the override exists only to HOLD the radio on the
	 * preset it is already running while _prefs move ahead of it
	 * (freezeRadioParams, i.e. `set radio` / `set freq` + reboot): nothing
	 * changed on air, so the offset that was learned for it keeps running
	 * until the reboot re-reads prefs.  Either way CAD adaptation is
	 * suspended while the override is up — see cadMaintenance(). */
	void setRadioOverride(float freq, float bw, uint8_t sf, uint8_t cr,
			      bool visiting_new_preset = true);
	void clearRadioOverride();
	bool hasRadioOverride() const { return _has_radio_override; }
	int recvRaw(uint8_t *bytes, int sz) override;
	uint32_t getEstAirtimeFor(int len_bytes) override;
	float packetScore(float snr, int packet_len) override;
	bool startSendRaw(const uint8_t *bytes, int len) override;
	bool isSendComplete() override;
	void onSendFinished() override;
	bool isInRecvMode() const override;
	float getLastRSSI() const override;
	float getLastSNR() const override;
	bool isRadioReady() override;

	/* Packet statistics */
	uint32_t getPacketsRecv() const override { return (uint32_t)atomic_get(&_packets_recv); }
	uint32_t getPacketsSent() const override { return (uint32_t)atomic_get(&_packets_sent); }
	uint32_t getPacketsRecvErrors() const override { return (uint32_t)atomic_get(&_packets_recv_errors); }
	/* Virtual so radios with extra accumulators can clear them on the same
	 * `clear stats`.  Reached through a LoRaRadioBase& from
	 * RepeaterMesh::clearStats() via getRadioDriver(), so a shadowing
	 * non-virtual override would silently never run. */
	virtual void resetStats() {
		atomic_set(&_packets_recv, 0);
		atomic_set(&_packets_sent, 0);
		atomic_set(&_packets_recv_errors, 0);
	}

	/* Advanced radio features */
	int getNoiseFloor() const override;
	void triggerNoiseFloorCalibrate(int threshold) override;
	bool isReceiving() override;
	void recoverRxState() override;

	/* Extended API */
	bool isChannelActive(int threshold = 0);

	/* Power saving */
	void enableRxDutyCycle(bool enable);
	bool isRxDutyCycleEnabled() const { return _rx_duty_cycle_enabled; }
	/* Returns false when the chip has no RX boost feature (SX127x). */
	virtual bool setRxBoost(bool enable);
	bool isRxBoostEnabled() const { return _rx_boost_enabled; }

	/* External FEM/LNA gain during RX.  Boards whose front-end module has a
	 * software-selectable receive path wire that select line to the radio
	 * (lna-bypass-gpios); disabling this routes RX around the FEM's LNA,
	 * trading its gain for its supply current while leaving the antenna
	 * connected.  The transmit path and the driver's idle gating are
	 * unaffected.  Returns false on radios that do not implement the knob
	 * (everything but the native SX126x today) and on SX126x boards that
	 * wire no such line -- including boards whose only FEM control is the
	 * chip enable, where there is no gain to trade, only a path to cut. */
	virtual bool setFemRxEnable(bool enable) { (void)enable; return false; }

	/* Multi-SF receive via LoRa side detectors.  Only the LR2021 has them;
	 * every other radio reports the feature as unsupported.  `num` = 0
	 * disables.  Returns false if the radio has no side detectors or the
	 * requested set violates a chip constraint (see
	 * lr20xx_configure_side_detectors). */
	virtual bool configSideDetectors(const uint8_t *sfs, uint8_t num) {
		(void)sfs; (void)num;
		return false;
	}

	/* Read-only view of the modem config currently used by buildModemConfig().
	 * These honor temporary radio overrides for freq/bw/sf/cr and the same TX
	 * clamps as the actual lora_config() path. */
	uint32_t getActiveFrequencyHz() const;
	uint16_t getActiveBandwidthKHzX10() const;
	uint8_t getActiveSpreadingFactor() const;
	uint8_t getActiveCodingRate() const;
	uint16_t getActivePreambleLength() const;
	uint8_t getActiveSyncWord() const;
	int8_t getConfiguredTxPower() const;
	bool isTxActive() const override { return atomic_get(&_tx_active) != 0; }

	/* Duty-cycle preamble false-positive counter.
	 * Incremented by the driver whenever RX_TX_TIMEOUT fires in
	 * duty-cycle mode and the chip is silently re-armed.  High
	 * values indicate a noisy RF environment or too-loose preamble
	 * detection — each event extends real RX time past the nominal
	 * duty cycle, inflating current draw.
	 * Default returns 0 on radios that don't support the stat. */
	virtual uint32_t getDutyCycleTimeoutRestarts() const { return 0; }
	virtual void resetDutyCycleTimeoutRestarts() {}

	/* Adaptive CAD (LBT detPeak calibration) */
	void setCadParams(bool auto_enabled, int8_t offset,
			  uint16_t probe_interval_s, uint8_t busycap_pct,
			  uint8_t stored_base = 0) override;
	uint8_t cadBasePeak() override;
	void cadMaintenance() override;
	/** Deaf-aware AGC unstick + temperature-drift recalibration.
	 *  Called from Dispatcher::maintenanceLoop(); never on the packet path. */
	void radioMaintenance() override;

private:
	/* The two unrelated jobs radioMaintenance() drives; see its comment. */
	void agcIdleMaintenance(uint32_t now);
	void imageCalMaintenance(uint32_t now);
public:
	uint32_t msUntilNextMaintenance() override;
	int8_t getCadOffset() const override { return _cad_offset; }

	/* Offset bounds the controller may actually use: the static
	 * [CAD_LEVEL_MIN, CAD_LEVEL_MAX] window narrowed to whatever the
	 * hardware clamp leaves distinguishable at the current base.  Every
	 * range decision goes through these; the raw constants stay in use only
	 * for indexing _cad_stats[], which is sized to the static window. */
	int8_t cadLevelMinEff();
	int8_t cadLevelMaxEff();
	void resetCadStats() override;
	bool cadRelaxOnTxStarvation() override;
	/* Airtime-protection / stuck-detector rung, run on every maintenance pass
	 * regardless of _cad_auto.  Returns true when it moved the offset. */
	bool cadSafetyStep();
	int formatCadStatus(char *buf, int cap) override;

protected:
	/* ── Hardware primitives — subclass MUST implement ─────────── */

	virtual bool hwConfigure(const struct lora_modem_config &cfg) = 0;
	virtual void hwCancelReceive() = 0;
	virtual int hwSendAsync(uint8_t *buf, uint32_t len,
				struct k_poll_signal *sig) = 0;
	virtual int16_t hwGetCurrentRSSI() = 0;
	/** Read `n` instantaneous RSSI samples spaced `spacing_us` apart.
	 *
	 * Exists so the whole burst can be bracketed ONCE by whatever a family
	 * needs to take a reading, instead of once per sample.  On the LR
	 * families a single RSSI read stands the duty cycle down, enters
	 * continuous Rx, waits 1 ms to settle, reads, and re-arms the cycle --
	 * and that re-arm clears every IRQ and zeroes the RX-busy latch.  Called
	 * eight times for one median that is eight cycle tear-downs, eight latch
	 * wipes and 8 ms of settle, every sampling interval, whether or not a
	 * CAD probe follows.
	 *
	 * Returns the number of valid samples written (< n means the read was
	 * refused partway and the caller should abandon the burst), or a NEGATIVE
	 * value when the burst must be abandoned for a reason that is not a read
	 * failure: -EAGAIN says the receiver detected a preamble or header inside
	 * the window, so the samples describe that signal and not the floor.
	 * Worth its own return because the caller's counters exist to tell a
	 * failing bus from a busy channel, and only a family that brackets its own
	 * Rx entry is in a position to notice the latter.
	 *
	 * The default implementation is the old per-sample loop, which is already
	 * correct for radios whose RSSI read has no such bracket (SX126x bails on
	 * BUSY and touches nothing; SX127x has no duty cycle at all) and which
	 * therefore never reports -EAGAIN. */
	virtual int hwGetRssiBurst(int16_t *out, int n, uint32_t spacing_us);
	/* Non-destructive read of the radio's "currently receiving" signal —
	 * latch + raw IRQ bits, never clears.  Backs LoRaRadioBase::isReceiving(). */
	virtual bool hwIsReceiving() = 0;
	virtual void hwSetRxBoost(bool enable) = 0;

	/** GPIO-only BUSY check (no SPI). Default false for chips without duty-cycle sleep. */
	virtual bool hwIsChipBusy() { return false; }

	/* ── Adaptive-CAD primitives — defaults suit chips without hardware
	 * CAD (SX127x): probing unsupported, offset ignored. ───────────── */

	/** Blocking calibration CAD at (family base detPeak + level).
	 *  Returns 0 = free (chip in STANDBY, caller restarts RX),
	 *          1 = busy, chip in STANDBY, caller restarts RX,
	 *          2 = busy, chip left in RX on the detected signal (CAD_RX
	 *              exit mode) -- caller must NOT restart RX, and reads the
	 *              outcome later via hwCadRxOutcome(),
	 *          <0 = error / unsupported. */
	virtual int hwCadProbe(int8_t level) { (void)level; return -ENOSYS; }

	/** Outcome of the RX a hwCadProbe() == 2 left the chip in.
	 *  1 = a packet completed, so the detection was real;
	 *  2 = the chip's own CAD timeout expired with nothing decoded;
	 *  0 = not armed, or the terminal interrupt has not arrived yet.
	 *  Reads driver state only -- no chip access, no polling loop. */
	virtual int hwCadRxOutcome() { return 0; }

	/** How long the chip may stay in a CAD_RX-entered RX before it raises
	 *  its own timeout.  Bounds the wait before hwCadRxOutcome() is read. */
	virtual uint32_t hwCadRxTimeoutMs() { return 0; }
	/** Apply the operating detPeak offset for all subsequent LBT CADs. */
	virtual void hwCadSetPeakOffset(int8_t offset) { (void)offset; }
	/** Per-SF base detPeak for the current config (0 = unsupported). */
	virtual uint8_t hwCadBasePeak() { return 0; }

	/** Absolute detPeak range the driver will actually program, inclusive.
	 *  0/0 means "no known limit" and the offset range stays as-is.
	 *
	 *  This exists because the offset window and the hardware clamp are two
	 *  different things, and when they disagree the controller explores
	 *  levels that are physically identical: on the LR2021 at SF7 the base is
	 *  51 and the driver clamps to 48, so offsets -3 through -8 all programmed
	 *  the same peak.  The staircase then compared three rungs of the same
	 *  configuration, found only sampling noise between them, and random-walked
	 *  into the floor with nothing to climb back out on. */
	virtual uint8_t hwCadPeakMin() { return 0; }
	virtual uint8_t hwCadPeakMax() { return 0; }

	/* ── Receiver hygiene — see LoRaRadioBase::radioMaintenance() ─── */

	/** Unstick a jammed AGC: warm sleep to drop the analog front end, then
	 *  recalibrate on the way back up.  Semtech's stated remedy; the
	 *  datasheets do not describe it, so do not "simplify" it away on
	 *  datasheet grounds alone.  Must leave the driver out of RX so the
	 *  caller's startReceive() performs a real re-entry rather than hitting
	 *  an idempotent fast path.  Default: unsupported, no-op. */
	virtual void hwResetAgc() {}

	/** Redo the frequency-dependent calibrations (image / front end, and
	 *  PLL+AAF where the part separates them) at the current operating
	 *  frequency.  Called on temperature drift, never on the packet path.
	 *  Same RX-state contract as hwResetAgc(). Default: unsupported. */
	virtual void hwRecalibrate() {}

	/** Chip junction temperature in whole degrees C, or INT16_MIN when the
	 *  backend cannot measure it (which disables drift recalibration). */
	/* Does this family actually suffer the jammed-AGC fault the reset is a
	 * remedy for?  Only the SX126x does.  Semtech prescribe warm sleep plus
	 * recalibration for it, ZephCore inherited the idea from Arduino
	 * MeshCore's `agc_reset_interval`, and both are SX126x-era.  Neither the
	 * LR11xx UM nor the LR2021 DS describes such a fault; those parts need
	 * image/front-end recalibration on temperature drift, which is a
	 * different operation on a different trigger (hwRecalibrate()).
	 *
	 * Running it anyway is not free.  Measured on a T1000-E 2026-08-23: the
	 * 60 s after an AGC reset carried a 7.4% packet-miss rate against 0.6%
	 * elsewhere, and one deaf stretch began at one reset and ended at the
	 * next — the reset was recovering damage it had caused, on a part with
	 * no AGC fault to fix. */
	virtual bool hwNeedsAgcReset() { return false; }

	/* Does this family specify a temperature threshold for image/front-end
	 * recalibration?  LR11xx and LR2021 do; the SX126x datasheet does not,
	 * and drift recalibration stays inactive there exactly as before. */
	virtual bool hwHasDriftRecal() { return false; }

	/** Radio deaf time per duty-cycle wake transition (context restore +
	 *  PLL lock + TCXO startup where fitted), in microseconds.  Counts
	 *  against the duty-cycle preamble-catch budget: per SX126x DS rev 2.2
	 *  §13.1.7 the TCXO startup delay is inserted between the sleep and RX
	 *  periods, outside both.
	 *
	 *  This default is a fallback for backends that expose no per-device
	 *  figure; it suits XTAL parts only.  SX126x, LR11xx and LR2021 all
	 *  override it, because a board with a TCXO powers the regulator down
	 *  during duty-cycle sleep and pays the oscillator restart on every
	 *  wake — several milliseconds, dwarfing this number.  Leaving a TCXO
	 *  board on the default oversizes the sleep window and drops
	 *  window-edge preambles regardless of signal strength. */
	virtual uint32_t hwWakeupTimeUs() { return 1500; }

	/* Set to true by subclasses using the loramac-node driver backend.
	 * Disables the direction-only fast path in configureTx()/configureRx():
	 * loramac-node calls Radio.SetTxConfig() and Radio.SetRxConfig() which
	 * configure completely disjoint internal state — skipping either leaves
	 * TxTimeout/RxConfig uninitialized in the loramac-node library. */
	bool _loramac_node;

	/* ── Shared helpers available to subclasses ────────────────── */

	void buildModemConfig(struct lora_modem_config &cfg, bool tx);
	/* Shared body for configureRx()/configureTx(): builds the modem config for
	 * the given direction, honours the params-unchanged and direction-only
	 * fast paths, then programs the radio via hwConfigure(). */
	void configure(bool tx);
	void configureRx();
	void configureTx();
	void startReceive();
	void startTxThread(k_thread_stack_t *stack, size_t stack_size);

	const struct device *_dev;
	NodePrefs *_prefs;
	MainBoard *_board;
	atomic_t _in_recv_mode;
	atomic_t _tx_active;
	/* Completion latch: raised by the TX wait thread only on an affirmative
	 * success verdict, consumed once by isSendComplete(), and cleared by
	 * startSendRaw() so a completion the dispatcher never collected (it gave
	 * up on outbound_expiry first) cannot leak into the next packet.  This
	 * is the Zephyr equivalent of upstream's STATE_INT_READY -> STATE_IDLE
	 * transition; _tx_active alone cannot serve, because it is also cleared
	 * on every failure path and is read as a plain state query elsewhere. */
	atomic_t _tx_complete;
	volatile float _last_rssi;   /* word-aligned: atomic on ARM */
	volatile float _last_snr;    /* word-aligned: atomic on ARM */

	/* RX ring buffer */
	struct RxPacket {
		uint8_t data[256];
		uint16_t len;
		int16_t rssi;
		int8_t snr;
	};
	RxPacket _rx_ring[RX_RING_SIZE];
	atomic_t _rx_head;
	atomic_t _rx_tail;

	/* TX buffer + signal */
	uint8_t _tx_buf[256];
	struct k_poll_signal _tx_signal;

	/* Noise floor calibration state */
	int _noise_floor;
	int _calibration_threshold;
	uint8_t _ema_unguarded;         /* tick counter for warmup + periodic bypass */
	/* Absolute uptime deadline of the next floor sample.  The sampler used
	 * to run on every housekeeping tick, which pinned its cadence to the
	 * 5 s timer; owning its own deadline is what lets that timer go away.
	 * Advanced by NOISE_FLOOR_INTERVAL_MS after a sample lands, and by the
	 * shorter retry when an attempt is turned away because the radio was
	 * mid-packet / transmitting / in its duty-cycle sleep window. */
	int64_t _noise_floor_next_ms;
	uint8_t _noise_floor_retries;   /* consecutive blocked attempts, capped */
	/* Shared cadence for every periodic radio measurement (floor sample +
	 * CAD probe).  Runtime, from the probe.interval pref. */
	uint32_t _measure_interval_ms;
	/* Latest floor sample, published for cadMaintenance() so the CAD probe
	 * shares this measurement instead of taking its own single RSSI read.
	 * _sample_fresh is true only within the pass that produced it. */
	int16_t _sample_rssi;
	bool _sample_channel_quiet;
	bool _sample_fresh;
	/* Cycle stamp of the last host-driven RX entry, used to skip a floor
	 * sample taken before GetRssiInst has settled (DS Table 13-82). */
	uint32_t _rx_entry_cyc;
	/* Median-of-N quality accounting, surfaced by `get cad` as sp:<mean>/<%>.
	 * The median only rejects outliers if the N reads are independent; if
	 * they land inside one RSSI averaging window they are the same sample
	 * N times over and the median is decorative.  Spread (max-min of the
	 * burst) and the share of zero-spread bursts make that visible without
	 * a debug build. */
	uint32_t _rssi_bursts;
	uint32_t _rssi_spread_sum;
	uint32_t _rssi_degenerate;

	/* Adaptive CAD state */
	struct CadLevelStats {
		uint16_t probes;   /* probes run at this level */
		uint16_t busy;     /* raw busy verdicts */
		uint16_t fp;       /* busy that passed the ground-truth filter (suspected false positive) */
		uint16_t tp;       /* busy confirmed by RX activity right after */
	};
	CadLevelStats _cad_stats[CAD_NUM_LEVELS];
	bool _cad_auto;                 /* staircase acts on the stats */
	int8_t _cad_offset;             /* operating detPeak offset (levels) */
	/* The offset in force while a temporary radio override is active, i.e.
	 * on a preset the node is only visiting (`tempradio`, and the frozen
	 * old preset a `set radio` leaves running until reboot).  detPeak's base
	 * table is per-SF and per-bandwidth, so the offset learned for the
	 * configured preset addresses a different ladder there — on the LR11xx
	 * the SF7->SF12 base step alone is 16 counts.  Kept separate rather than
	 * saved-and-restored so _cad_offset never moves during the visit, which
	 * is what keeps it out of prefs: Dispatcher::maintenanceLoop() persists
	 * any change it sees in getCadOffset(). */
	int8_t _cad_temp_offset;
	/* The offset the chip is actually programmed with.  Every hwCadSetPeakOffset()
	 * goes through this; getCadOffset() deliberately does not, because that is
	 * the value the app persists. */
	int8_t cadEffectiveOffset() const {
		return _has_radio_override ? _cad_temp_offset : _cad_offset;
	}
	uint16_t _probe_interval_s; /* 0 = CAD probing disabled; drives _measure_interval_ms */
	uint8_t _cad_busycap_pct;       /* airtime cap: max % TX deferred (0 = off) */
	/* A CAD_RX probe awaiting its terminal event.  _cad_pending_level is the
	 * stats rung the result belongs to; _cad_pending_deadline_ms is when the
	 * chip guarantees it has resolved, so the answer is READ once at that
	 * point rather than polled for.  Level INT8_MIN means nothing pending. */
	int8_t _cad_pending_level;
	int64_t _cad_pending_deadline_ms;
	int64_t _cad_last_probe_ms;
	int64_t _cad_last_decay_ms;
	/* Earliest uptime at which a due-but-blocked probe may be retried.  The
	 * interval check in cadMaintenance() is against _cad_last_probe_ms,
	 * which only advances on a probe that actually ran — without this a
	 * blocked probe would report "due now" forever and spin the wake. */
	uint8_t _cad_probe_rr;          /* round-robin index (sweep) / frontier mix counter */

	int8_t pickCadProbeLevel();
	void decayCadStats();
	void cadStaircaseStep();

	/* Power saving */
	bool _rx_duty_cycle_enabled;
	bool _rx_boost_enabled;

	/* Last duty-cycle timing handed to the driver — used to log timing
	 * changes once at INF instead of on every RX restart.  0/0 = never
	 * computed; UINT32_MAX rx = continuous-RX fallback active. */
	uint32_t _dc_last_rx_us;

	uint32_t _dc_last_sleep_us;

	/* agcIdleMaintenance() bookkeeping.  RX activity is inferred by sampling the
	 * existing packet counters rather than timestamping in the RX callback,
	 * so nothing is added to the ISR path. */
	uint32_t _agc_rx_count_shadow;
	uint32_t _agc_last_activity_ms;

	/* Stuck-AGC corroboration.  The noise-floor sampler already produces an
	 * RSSI every interval; a desensitised front end reports a frozen one.
	 * Free evidence — no extra command, no extra wake. */
	int16_t  _agc_rssi_last;
	uint8_t  _agc_rssi_frozen;

	/* Diagnostics only — no behaviour depends on these.
	 *
	 * The RSSI sampler discards a whole burst if any read comes back busy,
	 * and on a duty-cycled LR1110 it completes ~0.2% of its attempts
	 * (measured 5 bursts in 9 h against 1007 on an SX1262 beside it).  That
	 * is either "a few reads land in the sleep phase" or "essentially every
	 * read is refused", and the two want opposite fixes.  These counters say
	 * which, and are reported by `get cad`. */
	uint32_t _rssi_reads_ok;
	uint32_t _rssi_reads_busy;
	uint32_t _rssi_bursts_abandoned;
	/* Attempts turned away BEFORE a burst starts, by the isRadioReady()
	 * guard — i.e. the duty-cycle sleep window.  The three counters above
	 * cannot see this: they only move once a burst is already running, so a
	 * sampler that is being refused outright reads as b0/a0, which is
	 * indistinguishable from one that is running perfectly.  That ambiguity
	 * is what makes the "does the floor go stale under duty cycle?" question
	 * unanswerable today, and it is the missing denominator for the ~0.2%
	 * completion rate noted above. */
	uint32_t _rssi_dc_blocked;

	/* Silence tracking for deafness hunting.  Reports only; the recovery
	 * decision lives in agcIdleMaintenance() and is family-gated. */
	uint32_t _silence_last_report_ms;
	int16_t  _image_cal_last_temp_c;

	/* Image-calibration temperature polling.  Deliberately much slower than
	 * the maintenance cadence — see imageCalMaintenance(). */
	uint32_t _image_cal_last_ms;
	uint32_t _image_cal_wait_ms;
	bool     _image_cal_started;
	bool     _image_cal_confirming;

	/* Start of the most recent transmit, for the post-TX quiet window the
	 * temperature poll waits out.  Start-referenced rather than end-
	 * referenced so it can be stamped at the single startSendRaw() site
	 * instead of all five _tx_active clear paths; airtime is bounded by
	 * seconds and the quiet window by a minute, so the difference does not
	 * matter. */
	uint32_t _last_tx_start_ms;

	/* Config cache — skip redundant hwConfigure() */
	struct lora_modem_config _last_cfg;
	bool _config_cached;

	/* Radio param override — when set, buildModemConfig() uses these
	 * for freq/bw/sf/cr instead of _prefs.  Everything else (tx_power,
	 * preamble) still comes from _prefs. */
	bool _has_radio_override;
	float _override_freq;
	float _override_bw;
	uint8_t _override_sf;
	uint8_t _override_cr;

	/* ISR RX callback — passed to lora_recv_async() / lora_recv_duty_cycle_async() */
	static void rxCallbackStatic(const struct device *dev, uint8_t *data,
				     uint16_t size, int16_t rssi, int8_t snr,
				     void *user_data);

private:
	/* RX notification callback */
	RadioRxCallback _rx_cb;
	void *_rx_cb_user_data;

	/* TX done callback */
	RadioTxDoneCallback _tx_done_cb;
	void *_tx_done_cb_user_data;

	/* TX completion thread */
	static void txWaitThreadFn(void *p1, void *p2, void *p3);
	uint32_t txWaitBudgetMs() const;
	struct k_thread _tx_wait_thread;
	struct k_sem _tx_start_sem;
	bool _tx_thread_running;
	/* Length of the transmit in flight, for txWaitBudgetMs().  Written by
	 * startSendRaw() before it releases _tx_start_sem, read by the wait
	 * thread after it takes that sem — the sem is the handoff, so no
	 * additional synchronisation is needed. */
	uint16_t _tx_len;

	/* Packet statistics */
	atomic_t _packets_recv;
	atomic_t _packets_sent;
	atomic_t _packets_recv_errors;
};

} /* namespace mesh */
