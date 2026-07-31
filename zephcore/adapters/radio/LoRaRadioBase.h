/*
 * SPDX-License-Identifier: MIT
 * LoRa radio base class — shared state and algorithms.
 * Subclasses implement hw*() primitives only.
 */

#pragma once

#include <mesh/Radio.h>
#include <mesh/Board.h>
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
	void setRadioOverride(float freq, float bw, uint8_t sf, uint8_t cr);
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
	void resetStats() {
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
	bool isTxActive() const { return atomic_get(&_tx_active) != 0; }

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
			  uint16_t probe_interval_s, uint8_t busycap_pct) override;
	void cadMaintenance() override;
	uint32_t msUntilNextMaintenance() override;
	int8_t getCadOffset() const override { return _cad_offset; }
	void resetCadStats() override;
	int formatCadStatus(char *buf, int cap) override;

protected:
	/* ── Hardware primitives — subclass MUST implement ─────────── */

	virtual bool hwConfigure(const struct lora_modem_config &cfg) = 0;
	virtual void hwCancelReceive() = 0;
	virtual int hwSendAsync(uint8_t *buf, uint32_t len,
				struct k_poll_signal *sig) = 0;
	virtual int16_t hwGetCurrentRSSI() = 0;
	/* Non-destructive read of the radio's "currently receiving" signal —
	 * latch + raw IRQ bits, never clears.  Backs LoRaRadioBase::isReceiving(). */
	virtual bool hwIsReceiving() = 0;
	virtual void hwSetRxBoost(bool enable) = 0;

	/** GPIO-only BUSY check (no SPI). Default false for chips without duty-cycle sleep. */
	virtual bool hwIsChipBusy() { return false; }

	/* ── Adaptive-CAD primitives — defaults suit chips without hardware
	 * CAD (SX127x): probing unsupported, offset ignored. ───────────── */

	/** Blocking calibration CAD at (family base detPeak + level).
	 *  Leaves the chip in STANDBY; caller restarts RX.
	 *  Returns 1 = busy, 0 = free, <0 = error / unsupported. */
	virtual int hwCadProbe(int8_t level) { (void)level; return -ENOSYS; }
	/** Apply the operating detPeak offset for all subsequent LBT CADs. */
	virtual void hwCadSetPeakOffset(int8_t offset) { (void)offset; }
	/** Per-SF base detPeak for the current config (0 = unsupported). */
	virtual uint8_t hwCadBasePeak() { return 0; }

	/** Radio deaf time per duty-cycle wake transition (context restore +
	 *  PLL lock + TCXO startup where fitted), in microseconds.  Counts
	 *  against the duty-cycle preamble-catch budget: per SX126x DS rev 2.2
	 *  §13.1.7 the TCXO startup delay is inserted between the sleep and RX
	 *  periods, outside both.  Default suits XTAL parts (LR2021). */
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
	uint16_t _probe_interval_s; /* 0 = CAD probing disabled; drives _measure_interval_ms */
	uint8_t _cad_busycap_pct;       /* airtime cap: max % TX deferred (0 = off) */
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

	/* ISR RX callback — passed to lora_recv_async() / lora_recv_duty_cycle() */
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
	struct k_thread _tx_wait_thread;
	struct k_sem _tx_start_sem;
	bool _tx_thread_running;

	/* Packet statistics */
	atomic_t _packets_recv;
	atomic_t _packets_sent;
	atomic_t _packets_recv_errors;
};

} /* namespace mesh */
