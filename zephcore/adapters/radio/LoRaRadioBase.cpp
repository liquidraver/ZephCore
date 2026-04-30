/*
 * SPDX-License-Identifier: Apache-2.0
 * LoRa radio base class — shared algorithms for all radio adapters.
 */

#include "LoRaRadioBase.h"
#include "radio_common.h"
#include <mesh/LoRaConfig.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <math.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lora_radio_base, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

static uint16_t preambleLengthForSF(uint8_t sf)
{
	/* PR #1954 parity: longer preamble for lower SF. */
	return (sf <= 8) ? 32 : 16;
}

/* ── Constructor ─────────────────────────────────────────────── */

LoRaRadioBase::LoRaRadioBase(const struct device *lora_dev, MainBoard &board,
			     NodePrefs *prefs)
	: _loramac_node(false),
	  _dev(lora_dev), _prefs(prefs), _board(&board),
	  _in_recv_mode(0), _tx_active(0),
	  _last_rssi(0), _last_snr(0),
	  _rx_head(0), _rx_tail(0),
	  _noise_floor(DEFAULT_NOISE_FLOOR), _calibration_threshold(0), _ema_unguarded(0),
	  _rx_duty_cycle_enabled(IS_ENABLED(CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE)),
	  _rx_boost_enabled(true),
	  _tx_power_reduction_db(0),
	  _config_cached(false),
	  _has_radio_override(false),
	  _override_freq(0), _override_bw(0),
	  _override_sf(0), _override_cr(0),
	  _rx_cb(nullptr), _rx_cb_user_data(nullptr),
	  _tx_done_cb(nullptr), _tx_done_cb_user_data(nullptr),
	  _tx_thread_running(false),
	  _packets_recv(0), _packets_sent(0), _packets_recv_errors(0)
{
	k_poll_signal_init(&_tx_signal);
	k_sem_init(&_tx_start_sem, 0, 1);
	memset(_rx_ring, 0, sizeof(_rx_ring));
}

/* ── TX wait thread ──────────────────────────────────────────── */

void LoRaRadioBase::txWaitThreadFn(void *p1, void *p2, void *p3)
{
	LoRaRadioBase *self = static_cast<LoRaRadioBase *>(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("TX wait thread started");

	for (;;) {
		k_sem_take(&self->_tx_start_sem, K_FOREVER);

		if (!atomic_get(&self->_tx_active)) {
			continue;
		}

		LOG_DBG("TX wait: waiting for signal...");

		struct k_poll_event events[1] = {
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
						 K_POLL_MODE_NOTIFY_ONLY,
						 &self->_tx_signal),
		};

		unsigned int signaled;
		int result;
		k_poll_signal_check(&self->_tx_signal, &signaled, &result);
		if (signaled) {
			LOG_DBG("TX wait: signal already raised (result=%d)", result);
			k_poll_signal_reset(&self->_tx_signal);
			self->_board->onAfterTransmit();
			self->startReceive();
			atomic_set(&self->_tx_active, 0);
			atomic_inc(&self->_packets_sent);
			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
			continue;
		}

		int ret = k_poll(events, 1, K_MSEC(TX_TIMEOUT_MS));
		if (ret == -EAGAIN) {
			LOG_ERR("TX wait: TIMEOUT!");
			self->_board->onAfterTransmit();
			self->startReceive();
			atomic_set(&self->_tx_active, 0);
			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
			continue;
		}

		if (ret == 0 && events[0].state == K_POLL_STATE_SIGNALED) {
			k_poll_signal_reset(&self->_tx_signal);
			self->_board->onAfterTransmit();
			self->startReceive();
			atomic_set(&self->_tx_active, 0);
			atomic_inc(&self->_packets_sent);
			LOG_INF("TX complete, RX restarted");

			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
		} else {
			LOG_ERR("TX wait: k_poll returned %d, state=%d — recovering",
				ret, events[0].state);
			k_poll_signal_reset(&self->_tx_signal);
			self->_board->onAfterTransmit();
			self->startReceive();
			atomic_set(&self->_tx_active, 0);

			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
		}
	}
}

void LoRaRadioBase::startTxThread(k_thread_stack_t *stack, size_t stack_size)
{
	if (_tx_thread_running) {
		return;
	}
	k_thread_create(&_tx_wait_thread, stack, stack_size,
			txWaitThreadFn, this, NULL, NULL,
			TX_WAIT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&_tx_wait_thread, "lora_tx_wait");
	_tx_thread_running = true;
}

/* ── RX callback (static, ISR-safe) ──────────────────────────────────── */

void LoRaRadioBase::rxCallbackStatic(const struct device *dev, uint8_t *data,
				     uint16_t size, int16_t rssi, int8_t snr,
				     void *user_data)
{
	LoRaRadioBase *self = static_cast<LoRaRadioBase *>(user_data);

	/* NULL data = RX error (CRC/header error) */
	if (data == NULL && size == 0) {
		atomic_inc(&self->_packets_recv_errors);
		LOG_DBG("RX error (CRC/header), total errors: %u",
			(uint32_t)atomic_get(&self->_packets_recv_errors));
		return;
	}

	LOG_DBG("RX callback: size=%u rssi=%d snr=%d", size, rssi, snr);

	/* Ring buffer write — SPSC: only ISR writes _rx_head, only main
	 * thread writes _rx_tail.  On overflow, drop the NEW packet to
	 * preserve this invariant (ISR must never touch _rx_tail). */
	uint8_t head = (uint8_t)atomic_get(&self->_rx_head);
	uint8_t next_head = (head + 1) % RX_RING_SIZE;
	if (next_head == (uint8_t)atomic_get(&self->_rx_tail)) {
		LOG_WRN("RX ring full, dropping new packet");
		atomic_inc(&self->_packets_recv_errors);
		if (self->_rx_cb) {
			self->_rx_cb(self->_rx_cb_user_data);
		}
		return;
	}

	RxPacket *pkt = &self->_rx_ring[head];
	uint16_t copy_len = (size > sizeof(pkt->data)) ? sizeof(pkt->data) : size;
	memcpy(pkt->data, data, copy_len);
	pkt->len = copy_len;
	pkt->rssi = rssi;
	pkt->snr = snr;

	atomic_set(&self->_rx_head, next_head);
	self->_last_rssi = (float)rssi;
	self->_last_snr = (float)snr;
	atomic_inc(&self->_packets_recv);

	if (self->_rx_cb) {
		self->_rx_cb(self->_rx_cb_user_data);
	}
}

/* ── Config helpers ───────────────────────────────────────────────────── */

void LoRaRadioBase::buildModemConfig(struct lora_modem_config &cfg, bool tx)
{
	memset(&cfg, 0, sizeof(cfg));
	/* Override wins for freq/bw/sf/cr (tempradio).  Power, preamble, and
	 * other fields still come from _prefs. */
	float freq_mhz = _has_radio_override ? _override_freq
			 : (_prefs ? _prefs->freq : (LoRaConfig::FREQ_HZ / 1000000.0f));
	float bw_khz = _has_radio_override ? _override_bw
		       : (_prefs ? _prefs->bw : (float)LoRaConfig::BANDWIDTH);
	uint8_t sf = _has_radio_override ? _override_sf
		     : (_prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR);
	uint8_t cr = _has_radio_override ? _override_cr
		     : (_prefs ? _prefs->cr : LoRaConfig::CODING_RATE);
	cfg.frequency = (uint32_t)(freq_mhz * 1000000.0f);
	cfg.bandwidth = bw_khz_to_enum((uint16_t)bw_khz);
	cfg.datarate = (enum lora_datarate)sf;
	cfg.coding_rate = cr_to_enum(cr);
	cfg.preamble_len = preambleLengthForSF(sf);
	cfg.tx_power = _prefs ? (int8_t)_prefs->tx_power_dbm
			      : LoRaConfig::TX_POWER_DBM;
#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
	if (cfg.tx_power > CONFIG_ZEPHCORE_MAX_TX_POWER_DBM) {
		cfg.tx_power = CONFIG_ZEPHCORE_MAX_TX_POWER_DBM;
	}
#endif
	/* APC reduction (applied after all clamps) */
	cfg.tx_power -= _tx_power_reduction_db;
	if (cfg.tx_power < -9) cfg.tx_power = -9;

	cfg.tx = tx;
	cfg.iq_inverted = false;
	cfg.public_network = false;
	cfg.packet_crc_disable = false;

	/* LBT: driver performs hardware CAD before TX, returns -EBUSY if busy */
	if (tx) {
		cfg.cad.mode = LORA_CAD_MODE_LBT;
	}
}

/**
 * Compare radio-relevant fields of two modem configs.
 * Ignores the tx flag — that only selects TX vs RX mode, the actual
 * modem parameters (freq, SF, BW, CR, power) are what the driver
 * programs into registers.
 */
static bool configParamsEqual(const struct lora_modem_config &a,
			      const struct lora_modem_config &b)
{
	/* CRITICAL: a.tx == b.tx MUST be compared — without it, switching
	 * RX→TX skips lora_config() for TX params, breaking transmit. */
	return a.frequency == b.frequency &&
	       a.bandwidth == b.bandwidth &&
	       a.datarate == b.datarate &&
	       a.coding_rate == b.coding_rate &&
	       a.preamble_len == b.preamble_len &&
	       a.tx_power == b.tx_power &&
	       a.tx == b.tx &&
	       a.iq_inverted == b.iq_inverted &&
	       a.public_network == b.public_network;
}

/**
 * Check if only the TX/RX direction changed (all radio params identical).
 * Used to skip the full lora_config() call on TX↔RX transitions when
 * the driver already has valid TX and RX configs from previous calls.
 */
static bool onlyDirectionDiffers(const struct lora_modem_config &a,
				 const struct lora_modem_config &b)
{
	return a.frequency == b.frequency &&
	       a.bandwidth == b.bandwidth &&
	       a.datarate == b.datarate &&
	       a.coding_rate == b.coding_rate &&
	       a.preamble_len == b.preamble_len &&
	       a.tx_power == b.tx_power &&
	       a.iq_inverted == b.iq_inverted &&
	       a.public_network == b.public_network &&
	       a.tx != b.tx;
}

void LoRaRadioBase::configureRx()
{
	struct lora_modem_config cfg;
	buildModemConfig(cfg, false);

	if (_config_cached && configParamsEqual(cfg, _last_cfg)) {
		LOG_DBG("configureRx: params unchanged, skipping hwConfigure");
		return;
	}

	/* Fast path: if only the TX/RX direction changed, skip the full
	 * hwConfigure → lora_config() call.  The driver already has a valid
	 * RX config (RadioSetRxConfig) from a previous cycle — Radio.Rx(0)
	 * in hwStartReceive() will use those register values directly.
	 * This avoids the modem_acquire → modem_release → Radio.Sleep()
	 * round-trip that wastes ~5 ms on every TX→RX transition.
	 *
	 * Not used for loramac-node: Radio.SetTxConfig() and Radio.SetRxConfig()
	 * configure completely disjoint internal state (including TxTimeout).
	 * Skipping either on a direction change leaves that state uninitialized. */
	if (!_loramac_node && _config_cached && onlyDirectionDiffers(cfg, _last_cfg)) {
		LOG_DBG("configureRx: direction-only change, skip hwConfigure");
		_last_cfg = cfg;
		return;
	}

	LOG_DBG("configureRx: freq=%u bw=%d sf=%d cr=%d pwr=%d",
		cfg.frequency, (int)cfg.bandwidth, (int)cfg.datarate,
		(int)cfg.coding_rate, cfg.tx_power);

	hwConfigure(cfg);
	_last_cfg = cfg;
	_config_cached = true;
}

void LoRaRadioBase::configureTx()
{
	struct lora_modem_config cfg;
	buildModemConfig(cfg, true);

	if (_config_cached && configParamsEqual(cfg, _last_cfg)) {
		LOG_DBG("configureTx: params unchanged, skipping hwConfigure");
		return;
	}

	/* Fast path: direction-only change (RX→TX).  The driver already
	 * has a valid TX config (RadioSetTxConfig with TxTimeout=4000)
	 * from a previous cycle — Radio.Send() will use those values.
	 * Not used for loramac-node (see configureRx comment above). */
	if (!_loramac_node && _config_cached && onlyDirectionDiffers(cfg, _last_cfg)) {
		LOG_DBG("configureTx: direction-only change, skip hwConfigure");
		_last_cfg = cfg;
		return;
	}

	hwConfigure(cfg);
	_last_cfg = cfg;
	_config_cached = true;
}

/* ── Lifecycle ────────────────────────────────────────────────────────── */

void LoRaRadioBase::begin()
{
	if (!device_is_ready(_dev)) {
		LOG_ERR("LoRa device not ready");
		return;
	}

	/* Subclass begin() calls startTxThread() before calling us.
	 *
	 * RX boost and duty cycle are set via constructor defaults:
	 *   _rx_boost_enabled = true (boosted +3dB, overridable via setRxBoost())
	 *   _rx_duty_cycle_enabled = CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE
	 * Callers can override after begin() via setRxBoost() / enableRxDutyCycle().
	 */

	startReceive();

	uint32_t freq = _prefs ? (uint32_t)(_prefs->freq * 1000000.0f)
			       : LoRaConfig::FREQ_HZ;
	uint8_t sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	uint16_t bw_khz = _prefs ? (uint16_t)(_prefs->bw)
				 : (uint16_t)LoRaConfig::BANDWIDTH;
	uint8_t cr = _prefs ? _prefs->cr : LoRaConfig::CODING_RATE;
	int8_t tx_pwr = _prefs ? (int8_t)_prefs->tx_power_dbm
			       : LoRaConfig::TX_POWER_DBM;

	LOG_INF("radio started: freq=%u bw=%u sf=%u cr=%u pwr=%d",
		freq, bw_khz, sf, cr, tx_pwr);
}

void LoRaRadioBase::reconfigure()
{
	hwCancelReceive();
	atomic_set(&_in_recv_mode, 0);
	_config_cached = false;  /* Force full reconfigure */
	startReceive();

	uint32_t freq = _prefs ? (uint32_t)(_prefs->freq * 1000000.0f)
			       : LoRaConfig::FREQ_HZ;
	uint8_t sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	uint16_t bw_khz = _prefs ? (uint16_t)(_prefs->bw)
				 : (uint16_t)LoRaConfig::BANDWIDTH;
	uint8_t cr = _prefs ? _prefs->cr : LoRaConfig::CODING_RATE;
	int8_t tx_pwr = _prefs ? (int8_t)_prefs->tx_power_dbm
			       : LoRaConfig::TX_POWER_DBM;

	LOG_INF("radio reconfigured: freq=%u bw=%u sf=%u cr=%u pwr=%d",
		freq, bw_khz, sf, cr, tx_pwr);
}

void LoRaRadioBase::reconfigureWithParams(float freq, float bw, uint8_t sf, uint8_t cr)
{
	/* Callers (ObserverMesh CLI handlers) write to _prefs and call
	 * savePrefs() before invoking us — the radio just needs to pick up
	 * the new params.  Tempradio uses setRadioOverride() instead so it
	 * never touches _prefs. */
	(void)freq; (void)bw; (void)sf; (void)cr;
	reconfigure();
}

void LoRaRadioBase::setRadioOverride(float freq, float bw, uint8_t sf, uint8_t cr)
{
	_override_freq = freq;
	_override_bw = bw;
	_override_sf = sf;
	_override_cr = cr;
	_has_radio_override = true;
	reconfigure();
}

void LoRaRadioBase::clearRadioOverride()
{
	if (!_has_radio_override) {
		return;
	}
	_has_radio_override = false;
	reconfigure();
}

void LoRaRadioBase::startReceive()
{
	configureRx();

	int ret;

	if (_rx_duty_cycle_enabled) {
		/* Compute duty cycle timing from modem config (RadioLib algorithm).
		 * The driver converts these to hardware-specific units. */
		struct lora_modem_config cfg;
		buildModemConfig(cfg, false);

		uint8_t sf = (uint8_t)cfg.datarate;
		uint32_t bw_hz = bandwidth_to_hz(cfg.bandwidth);
		float bw_khz = (float)bw_hz / 1000.0f;
		uint16_t preamble_len = cfg.preamble_len;
		uint16_t min_symbols = (sf >= 7) ? 8 : 12;
		int16_t sleep_symbols = (int16_t)preamble_len -
					(int16_t)min_symbols;

		if (sleep_symbols > 0) {
			uint32_t symbol_us = (uint32_t)((float)(1 << sf) *
							 1000.0f / bw_khz);
			int16_t safe = sleep_symbols - 2;
			if (safe < 1) safe = 1;
			uint32_t sleep_us = (uint16_t)safe * symbol_us;

			uint32_t preamble_us = (preamble_len + 1) * symbol_us;
			int32_t w1 = ((int32_t)preamble_us -
				      ((int32_t)sleep_us - 1000)) / 2;
			uint32_t w2 = (min_symbols + 2) * symbol_us;
			uint32_t rx_us = (w1 > 0 && (uint32_t)w1 > w2)
					  ? (uint32_t)w1 : w2;

			ret = lora_recv_duty_cycle(_dev,
						   K_USEC(rx_us),
						   K_USEC(sleep_us),
						   rxCallbackStatic, this);
			if (ret == 0) {
				atomic_set(&_in_recv_mode, 1);
				return;
			}
			if (ret != -ENOSYS) {
				LOG_ERR("lora_recv_duty_cycle failed: %d", ret);
			}
		} else {
			LOG_WRN("Preamble too short for duty cycle "
				"(need >%d, have %d)",
				min_symbols, preamble_len);
		}
		/* Fall through to normal recv_async */
	}

	ret = lora_recv_async(_dev, rxCallbackStatic, this);
	if (ret < 0) {
		LOG_ERR("lora_recv_async failed: %d", ret);
		atomic_set(&_in_recv_mode, 0);
		return;
	}
	atomic_set(&_in_recv_mode, 1);
}

/* ── RX/TX ────────────────────────────────────────────────────────────── */

int LoRaRadioBase::recvRaw(uint8_t *bytes, int sz)
{
	uint8_t tail = (uint8_t)atomic_get(&_rx_tail);
	if (atomic_get(&_rx_head) == tail) {
		return 0;
	}

	RxPacket *pkt = &_rx_ring[tail];
	uint16_t len = pkt->len;
	if (len > (uint16_t)sz) {
		len = (uint16_t)sz;
	}

	memcpy(bytes, pkt->data, len);
	_last_rssi = (float)pkt->rssi;
	_last_snr = (float)pkt->snr;
	atomic_set(&_rx_tail, (tail + 1) % RX_RING_SIZE);
	return (int)len;
}

bool LoRaRadioBase::startSendRaw(const uint8_t *bytes, int len)
{
	if (len > (int)sizeof(_tx_buf)) {
		return false;
	}

	/* Defensive gate: callers should defer TX while radio is BUSY. */
	if (!isRadioReady()) {
		return false;
	}

	/* Last-moment hardware check before killing active RX.
	 * Closes the race between the Dispatcher's isReceiving() guard
	 * and hwCancelReceive() — if a preamble arrived in that gap,
	 * abort TX and let the Dispatcher re-queue. */
	if (hwIsPreambleDetected()) {
		return false;
	}

	_board->onBeforeTransmit();
	atomic_set(&_tx_active, 1);
	atomic_set(&_in_recv_mode, 0);

	hwCancelReceive();
	configureTx();

	memcpy(_tx_buf, bytes, len);
	k_poll_signal_reset(&_tx_signal);

	int ret = hwSendAsync(_tx_buf, (uint32_t)len, &_tx_signal);
	if (ret < 0) {
		LOG_ERR("hwSendAsync failed: %d", ret);
		_board->onAfterTransmit();
		atomic_set(&_tx_active, 0);
		startReceive();
		return false;
	}

	LOG_DBG("TX started async, len=%d", len);
	k_sem_give(&_tx_start_sem);
	return true;
}

bool LoRaRadioBase::isSendComplete()
{
	return !atomic_get(&_tx_active);
}

void LoRaRadioBase::onSendFinished()
{
	/* Nothing needed — TX state tracked via _tx_active */
}

bool LoRaRadioBase::isInRecvMode() const
{
	return atomic_get(&_in_recv_mode) != 0;
}

float LoRaRadioBase::getLastRSSI() const
{
	return _last_rssi;
}

float LoRaRadioBase::getLastSNR() const
{
	return _last_snr;
}

bool LoRaRadioBase::isRadioReady()
{
	/* BUSY high means the radio cannot accept SPI commands now
	 * (e.g. duty-cycle sleep phase on SX126x/LR11xx). */
	return !hwIsChipBusy();
}

/* ── Airtime + scoring ────────────────────────────────────────────────── */

uint32_t LoRaRadioBase::getEstAirtimeFor(int len_bytes)
{
	uint8_t sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	float bw = _prefs ? _prefs->bw : (float)LoRaConfig::BANDWIDTH;
	uint8_t cr_val = _prefs ? _prefs->cr : LoRaConfig::CODING_RATE;

	if (sf < 6) sf = 6;
	if (sf > 12) sf = 12;
	if (bw < 7.0f) bw = 125.0f;
	if (cr_val < 5) cr_val = 5;
	if (cr_val > 8) cr_val = 8;

	float t_sym = (float)(1 << sf) / (bw * 1000.0f);
	float t_preamble = (preambleLengthForSF(sf) + 4.25f) * t_sym;

	float de = (sf >= 11) ? 1.0f : 0.0f;
	float num = 8.0f * len_bytes - 4.0f * sf + 28.0f + 16.0f;
	float den = 4.0f * (sf - 2.0f * de);
	if (den < 1.0f) den = 4.0f;
	float n_payload = 8.0f + fmaxf(ceilf(num / den) * (cr_val - 4 + 4), 0.0f);

	float t_payload = n_payload * t_sym;
	return (uint32_t)((t_preamble + t_payload) * 1000.0f);
}

float LoRaRadioBase::packetScore(float snr, int packet_len)
{
	int sf = _prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR;
	if (sf < 7 || sf > 12) return 0.0f;
	if (snr < lora_snr_threshold[sf - 7]) return 0.0f;

	float success_rate = (snr - lora_snr_threshold[sf - 7]) / 10.0f;
	float collision_penalty = 1.0f - ((float)packet_len / 256.0f);
	float score = success_rate * collision_penalty;
	if (score < 0.0f) score = 0.0f;
	if (score > 1.0f) score = 1.0f;
	return score;
}

/* ── Advanced radio features ──────────────────────────────────────────── */

int LoRaRadioBase::getNoiseFloor() const
{
	return _noise_floor;
}

void LoRaRadioBase::triggerNoiseFloorCalibrate(int threshold)
{
	_calibration_threshold = threshold;

	if (!atomic_get(&_in_recv_mode) || atomic_get(&_tx_active)) {
		return;
	}

	/* Skip when the radio cannot accept commands right now
	 * (e.g. duty-cycle sleep BUSY window). */
	if (!isRadioReady()) {
		return;
	}

	/* Skip if mid-receive — don't want signal energy in the floor. */
	if (isReceiving()) {
		return;
	}

	/* Median of multiple RSSI reads (~200 us).  Rejects up to N/2-1
	 * outliers in either direction without the downward bias of min
	 * or the spike sensitivity of average.  Insertion sort is fine
	 * for N=8 (28 comparisons worst case, all in registers). */
	int16_t samples[NOISE_FLOOR_SAMPLES_PER_TICK];
	for (int i = 0; i < NOISE_FLOOR_SAMPLES_PER_TICK; i++) {
		samples[i] = hwGetCurrentRSSI();
		if (samples[i] == -128) {
			/* Chip busy or RSSI read contended — retry next tick. */
			return;
		}
	}
	/* Insertion sort — tiny array, branch-friendly on Cortex-M */
	for (int i = 1; i < NOISE_FLOOR_SAMPLES_PER_TICK; i++) {
		int16_t key = samples[i];
		int j = i - 1;
		while (j >= 0 && samples[j] > key) {
			samples[j + 1] = samples[j];
			j--;
		}
		samples[j + 1] = key;
	}
	int16_t rssi = (samples[NOISE_FLOOR_SAMPLES_PER_TICK / 2 - 1] +
			samples[NOISE_FLOOR_SAMPLES_PER_TICK / 2]) / 2;

	/* First sample after reset (DEFAULT_NOISE_FLOOR == 0): seed directly. */
	if (_noise_floor == DEFAULT_NOISE_FLOOR) {
		_noise_floor = rssi;
		if (_noise_floor < -120) _noise_floor = -120;
		if (_noise_floor > -50) _noise_floor = -50;
		_ema_unguarded = 0;
		LOG_DBG("noise_floor_cal: seed=%d", _noise_floor);
		return;
	}

	/* Threshold filter with warmup and periodic bypass.
	 *
	 * _ema_unguarded counts up from 0 on every tick.
	 *   Ticks 0..W-1 (warmup): all samples accepted for fast convergence
	 *     after seed/reset — prevents a bad seed from locking out the
	 *     real noise floor via a too-tight threshold.
	 *   Ticks W+: threshold filter active. Every Pth tick one sample
	 *     bypasses the filter so the floor can track sustained upward
	 *     shifts (new interference, antenna change).
	 *     The EMA's 1/8 weight naturally dampens isolated spikes. */
	const int W = (1 << NOISE_FLOOR_EMA_SHIFT);             /* 8  — warmup ticks */
	const int P = NOISE_FLOOR_UNGUARDED_INTERVAL;            /* 16 — periodic interval */
	bool warmup = (_ema_unguarded < W);
	bool periodic = (!warmup && (_ema_unguarded & (P - 1)) == 0);
	_ema_unguarded++;  /* wraps at 255 — harmless */

	if (!warmup && !periodic &&
	    rssi >= _noise_floor + NOISE_FLOOR_SAMPLING_THRESHOLD) {
		return;
	}

	/* EMA: floor += round_nearest((sample - floor) / W).
	 * Plain >> has downward bias (-1>>3 == -1 but +1>>3 == 0).
	 * Plain /  has a ±7 dead zone (small drifts ignored).
	 * Round-to-nearest: add half the divisor before dividing,
	 * with sign-aware bias so both directions are symmetric. */
	int diff = rssi - _noise_floor;
	int half = W / 2;                                      /* 4 */
	int step = (diff + (diff > 0 ? half : -half)) / W;
	_noise_floor += step;
	if (_noise_floor < -120) _noise_floor = -120;
	if (_noise_floor > -50) _noise_floor = -50;

	LOG_DBG("noise_floor_cal: rssi=%d, floor=%d, tick=%u",
		rssi, _noise_floor, _ema_unguarded - 1);
}

void LoRaRadioBase::resetAGC()
{
	/* Don't reset AGC while transmitting or receiving — warm sleep would
	 * abort the TX or corrupt the incoming packet.  maintenanceLoop()
	 * will retry next housekeeping cycle.
	 * Also skip if the chip is in its duty-cycle sleep phase: hwResetAGC()
	 * holds the SPI mutex with K_FOREVER and would hang for 3 s. */
	if (atomic_get(&_tx_active) || isReceiving()) {
		return;
	}
	if (_rx_duty_cycle_enabled && hwIsChipBusy()) {
		return;
	}

	hwResetAGC();

	/* Warm sleep + calibrate leaves the radio in STANDBY.
	 * Restart receive if we were in RX mode. */
	if (atomic_get(&_in_recv_mode)) {
		startReceive();
	}

	/* Reset noise floor so it reconverges from scratch (seed + warmup).
	 * Without this, a stuck _noise_floor of -120 makes the sampling threshold
	 * too low to accept normal samples, self-reinforcing the stuck value. */
	_noise_floor = DEFAULT_NOISE_FLOOR;
	_ema_unguarded = 0;
}

bool LoRaRadioBase::isReceiving()
{
	if (!atomic_get(&_in_recv_mode) || atomic_get(&_tx_active)) {
		return false;
	}
	if (hwIsPreambleDetected()) {
		return true;
	}
	return isChannelActive();
}

bool LoRaRadioBase::isChannelActive(int threshold)
{
	if (threshold == 0) {
		threshold = _calibration_threshold;
	}
	if (threshold == 0) {
		return false;
	}
	int16_t rssi = hwGetCurrentRSSI();
	return rssi > (_noise_floor + threshold);
}

/* ── Power saving ─────────────────────────────────────────────────────── */

void LoRaRadioBase::enableRxDutyCycle(bool enable)
{
	_rx_duty_cycle_enabled = enable;
	LOG_INF("RX duty cycle %s", enable ? "enabled" : "disabled");
	if (atomic_get(&_in_recv_mode)) {
		/* Restart receive to apply new duty cycle state */
		hwCancelReceive();
		atomic_set(&_in_recv_mode, 0);
		startReceive();
	}
}

void LoRaRadioBase::setRxBoost(bool enable)
{
	_rx_boost_enabled = enable;
	LOG_INF("RX boost %s (+3dB sensitivity, +2mA)",
		enable ? "enabled" : "disabled");
	if (atomic_get(&_in_recv_mode)) {
		hwSetRxBoost(enable);
	}
}

} /* namespace mesh */
