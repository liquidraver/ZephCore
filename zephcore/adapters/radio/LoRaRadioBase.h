/*
 * SPDX-License-Identifier: Apache-2.0
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
	void resetAGC() override;
	bool isReceiving() override;

	/* Extended API */
	bool isChannelActive(int threshold = 0);

	/* Power saving */
	void enableRxDutyCycle(bool enable);
	bool isRxDutyCycleEnabled() const { return _rx_duty_cycle_enabled; }
	void setRxBoost(bool enable);
	bool isRxBoostEnabled() const { return _rx_boost_enabled; }

	/* Duty-cycle preamble false-positive counter.
	 * Incremented by the driver whenever RX_TX_TIMEOUT fires in
	 * duty-cycle mode and the chip is silently re-armed.  High
	 * values indicate a noisy RF environment or too-loose preamble
	 * detection — each event extends real RX time past the nominal
	 * duty cycle, inflating current draw.
	 * Default returns 0 on radios that don't support the stat. */
	virtual uint32_t getDutyCycleTimeoutRestarts() const { return 0; }
	virtual void resetDutyCycleTimeoutRestarts() {}

	/* Adaptive Power Control */
	void setTxPowerReduction(int8_t reduction_db) override { _tx_power_reduction_db = reduction_db; }
	int8_t getTxPowerReduction() const override { return _tx_power_reduction_db; }

protected:
	/* ── Hardware primitives — subclass MUST implement ─────────── */

	virtual void hwConfigure(const struct lora_modem_config &cfg) = 0;
	virtual void hwCancelReceive() = 0;
	virtual int hwSendAsync(uint8_t *buf, uint32_t len,
				struct k_poll_signal *sig) = 0;
	virtual int16_t hwGetCurrentRSSI() = 0;
	virtual bool hwIsPreambleDetected() = 0;
	virtual void hwSetRxBoost(bool enable) = 0;
	virtual void hwResetAGC() = 0;

	/** GPIO-only BUSY check (no SPI). Default false for chips without duty-cycle sleep. */
	virtual bool hwIsChipBusy() { return false; }

	/* Set to true by subclasses using the loramac-node driver backend.
	 * Disables the direction-only fast path in configureTx()/configureRx():
	 * loramac-node calls Radio.SetTxConfig() and Radio.SetRxConfig() which
	 * configure completely disjoint internal state — skipping either leaves
	 * TxTimeout/RxConfig uninitialized in the loramac-node library. */
	bool _loramac_node;

	/* ── Shared helpers available to subclasses ────────────────── */

	void buildModemConfig(struct lora_modem_config &cfg, bool tx);
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

	/* Power saving */
	bool _rx_duty_cycle_enabled;
	bool _rx_boost_enabled;
	int8_t _tx_power_reduction_db;

	/* Config cache — skip redundant hwConfigure() */
	struct lora_modem_config _last_cfg;
	bool _config_cached;

	/* Radio param override — when set, buildModemConfig() uses these
	 * for freq/bw/sf/cr instead of _prefs.  Everything else (tx_power,
	 * preamble, APC reduction) still comes from _prefs. */
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
