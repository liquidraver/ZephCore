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
#include "CadController.h"
#include "NoiseFloorEstimator.h"

namespace mesh {

class LoRaRadioBase : public Radio, private CadHw {
public:
	LoRaRadioBase(const struct device *lora_dev, MainBoard &board);

	/* Mandatory, before begin(): the radio reads freq/bw/sf/cr/power
	 * through this pointer from then on (LoRaConfig only seeds prefs). */
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

	/* Temporary radio override: freq/bw/sf/cr without touching _prefs
	 * (tempradio, and the `set radio` freeze until reboot). visiting_new_preset:
	 * the radio moves to a preset it was not running, so CAD starts at that
	 * preset's base; false holds the running one. CAD adaptation is suspended. */
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
	/* Virtual: reached through LoRaRadioBase& (RepeaterMesh::clearStats()),
	 * so radios with extra accumulators clear them on the same `clear stats`. */
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

	/* External FEM LNA in RX (lna-bypass-gpios): false routes RX around the
	 * LNA, trading its gain for its current. Returns false where the radio
	 * or the board has no such select line. */
	virtual bool setFemRxEnable(bool enable) { (void)enable; return false; }

	/* Multi-SF receive via side detectors (LR2021 only). num = 0 disables.
	 * False when unsupported or the set violates a chip constraint. */
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

	/* Duty-cycle false-preamble re-arms, counted by the driver (each one
	 * stretches RX time past the nominal cycle). 0 where unsupported. */
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
	int8_t getCadOffset() const override { return _cad.offset(); }
	void resetCadStats() override { _cad.resetStats(); }
	bool cadRelaxOnTxStarvation() override { return _cad.relaxOnTxStarvation(); }
	int formatCadStatus(char *buf, int cap) override;

protected:
	/* ── Hardware primitives — subclass MUST implement ─────────── */

	virtual bool hwConfigure(const struct lora_modem_config &cfg) = 0;
	virtual void hwCancelReceive() = 0;
	virtual int hwSendAsync(uint8_t *buf, uint32_t len,
				struct k_poll_signal *sig) = 0;
	virtual int16_t hwGetCurrentRSSI() = 0;
	/* Read `n` RSSI samples `spacing_us` apart, bracketed once (on the LR
	 * families each single read tears the duty cycle down and back up).
	 * Returns samples written (< n: refused partway, abandon), or -EAGAIN
	 * when a preamble/header landed in the window (a busy channel, not a
	 * failing bus). The default per-sample loop suits SX126x and SX127x. */
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
	/* CadHw (see CadController.h): offset, base detPeak (0 = unsupported)
	 * and the driver's detPeak clamp (0/0 = none). Report the clamp: past it
	 * several offsets program one peak and the staircase random-walks. */
	void hwCadSetPeakOffset(int8_t offset) override { (void)offset; }
	uint8_t hwCadBasePeak() override { return 0; }
	uint8_t hwCadPeakMin() override { return 0; }
	uint8_t hwCadPeakMax() override { return 0; }

	/* ── Receiver hygiene — see LoRaRadioBase::radioMaintenance() ─── */

	/* Unstick a jammed AGC: warm sleep + recalibrate (Semtech's remedy, not in
	 * the datasheets). Must leave the driver out of RX so the caller's
	 * startReceive() is a real re-entry. Default: no-op. */
	virtual void hwResetAgc() {}

	/** Redo the frequency-dependent calibrations (image / front end, and
	 *  PLL+AAF where the part separates them) at the current operating
	 *  frequency.  Called on temperature drift, never on the packet path.
	 *  Same RX-state contract as hwResetAgc(). Default: unsupported. */
	virtual void hwRecalibrate() {}

	/* Only the SX126x has the jammed-AGC fault. Elsewhere the reset is pure
	 * cost (7.4% packet miss in the following minute on a T1000-E, LLD 03 §13). */
	virtual bool hwNeedsAgcReset() { return false; }

	/* Does this family specify a temperature threshold for image/front-end
	 * recalibration?  LR11xx and LR2021 do; the SX126x datasheet does not,
	 * and drift recalibration stays inactive there exactly as before. */
	virtual bool hwHasDriftRecal() { return false; }

	/* Deaf time per duty-cycle wake (context restore + PLL + TCXO start), in us,
	 * counted against the catch budget (DS §13.1.7). The default suits XTAL
	 * parts only: a TCXO board must override it or window-edge preambles drop. */
	virtual uint32_t hwWakeupTimeUs() { return 1500; }

	/* loramac-node backend: its TX and RX configs are disjoint state, so the
	 * direction-only fast path in configure() must stay off. */
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
	void startTxThread();

	const struct device *_dev;
	NodePrefs *_prefs;
	MainBoard *_board;
	atomic_t _in_recv_mode;
	atomic_t _tx_active;
	/* Completion latch: set by the wait thread on success only, consumed once
	 * by isSendComplete(), cleared by startSendRaw() (upstream's
	 * STATE_INT_READY). _tx_active cannot serve: failures clear it too. */
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
	NoiseFloorEstimator _floor_est;
	int _calibration_threshold;
	/* Deadline of the next floor sample: an interval after a sample, the
	 * short retry after a blocked attempt. */
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
	/* Burst quality for `get cad` sp:<mean>/<%>: are the N reads independent? */
	uint32_t _rssi_bursts;
	uint32_t _rssi_spread_sum;
	uint32_t _rssi_degenerate;

	/* Adaptive CAD: the controller, plus the probe schedule that stays here. */
	CadController _cad;
	uint16_t _probe_interval_s; /* 0 = CAD probing disabled; drives _measure_interval_ms */
	int64_t _cad_last_decay_ms;

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

	/* Sampler diagnostics for `get cad` (no behaviour depends on them): reads
	 * ok/busy, bursts abandoned, and attempts refused before a burst started
	 * (duty-cycle sleep), without which "refused" and "fine" both read b0/a0. */
	uint32_t _rssi_reads_ok;
	uint32_t _rssi_reads_busy;
	uint32_t _rssi_bursts_abandoned;
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

	/* Start of the last TX, for the drift poll's post-TX quiet window
	 * (start-referenced: one stamp site; seconds against a minute). */
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
	/* Length of the TX in flight; _tx_start_sem is the handoff. */
	uint16_t _tx_len;

	/* Packet statistics */
	atomic_t _packets_recv;
	atomic_t _packets_sent;
	atomic_t _packets_recv_errors;
};

} /* namespace mesh */
