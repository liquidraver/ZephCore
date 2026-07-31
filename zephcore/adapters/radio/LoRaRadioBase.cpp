/*
 * SPDX-License-Identifier: MIT
 * LoRa radio base class — shared algorithms for all radio adapters.
 */

#include "LoRaRadioBase.h"
#include "radio_common.h"
#include <mesh/LoRaConfig.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lora_radio_base, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

static uint16_t preambleLengthForSF(uint8_t sf)
{
	/* PR #1954 parity: longer preamble for lower SF. */
	return (sf <= 8) ? 32 : 16;
}

/* Minimum preamble symbols that must land inside one open duty-cycle RX
 * window for guaranteed detection.  8 is Semtech's own figure for sniff
 * mode (AN1200.36 §4: "8 symbols in LoRa make up the time required to
 * ensure that the SX1261/2 detects a valid incoming packet"); their
 * time-synced LoRaWAN stacks budget 6, so 8 already carries margin.
 * SF5/6 need more symbols to reach sensitivity (RadioLib/LBM use 12). */
static uint16_t rxDutyDetectSymbols(uint8_t sf)
{
	uint16_t d = CONFIG_ZEPHCORE_LORA_DC_MIN_SYMBOLS;

	return (sf >= 7) ? d : (uint16_t)(d + 4);
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
	  _noise_floor_next_ms(0), _noise_floor_retries(0),
	  _measure_interval_ms(CONFIG_ZEPHCORE_NOISE_FLOOR_INTERVAL_MS),
	  _sample_rssi(0), _sample_channel_quiet(false), _sample_fresh(false),
	  _rx_entry_cyc(0),
	  _rssi_bursts(0), _rssi_spread_sum(0), _rssi_degenerate(0),
	  _cad_auto(false), _cad_offset(0), _probe_interval_s(0),
	  _cad_busycap_pct(0),
	  _cad_last_probe_ms(0), _cad_last_decay_ms(0),
	  _cad_probe_rr(0),
	  _rx_duty_cycle_enabled(IS_ENABLED(CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE)),
	  _rx_boost_enabled(true),
	  _dc_last_rx_us(0), _dc_last_sleep_us(0),
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
	memset(_cad_stats, 0, sizeof(_cad_stats));
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
	if (cfg.tx_power < -9) cfg.tx_power = -9;

	cfg.tx = tx;
	cfg.iq_inverted = false;
	cfg.public_network = false;
	cfg.packet_crc_disable = false;

	/* LBT: driver gates send_async on cad.mode == LBT.
	 * Set unconditionally so the value reaches the driver via the
	 * initial RX lora_config() call and survives configureTx()'s
	 * direction-only fast path (which skips hwConfigure). RX paths
	 * never read cad.mode, so this is harmless during receive. */
	cfg.cad.mode = LORA_CAD_MODE_LBT;

	/* 4-symbol CAD at every SF (drivers default to 2 when this is 0).
	 * Our LBT runs against mesh packets that are mostly payload airtime;
	 * payload chirps correlate less reliably per symbol than preamble
	 * upchirps, so the extra looks matter — AN1200.48 itself recommends
	 * 4 symbols at SF9+.  The drivers scale their blocking-CAD timeout
	 * from this value, so slow presets stay covered. */
	cfg.cad.symbol_num = LORA_CAD_SYMB_4;
}

uint32_t LoRaRadioBase::getActiveFrequencyHz() const
{
	float freq_mhz = _has_radio_override ? _override_freq
			 : (_prefs ? _prefs->freq : (LoRaConfig::FREQ_HZ / 1000000.0f));

	return (uint32_t)(freq_mhz * 1000000.0f + 0.5f);
}

uint16_t LoRaRadioBase::getActiveBandwidthKHzX10() const
{
	float bw_khz = _has_radio_override ? _override_bw
		       : (_prefs ? _prefs->bw : (float)LoRaConfig::BANDWIDTH);

	return (uint16_t)(bw_khz * 10.0f + 0.5f);
}

uint8_t LoRaRadioBase::getActiveSpreadingFactor() const
{
	return _has_radio_override ? _override_sf
	       : (_prefs ? _prefs->sf : LoRaConfig::SPREADING_FACTOR);
}

uint8_t LoRaRadioBase::getActiveCodingRate() const
{
	return _has_radio_override ? _override_cr
	       : (_prefs ? _prefs->cr : LoRaConfig::CODING_RATE);
}

uint16_t LoRaRadioBase::getActivePreambleLength() const
{
	return preambleLengthForSF(getActiveSpreadingFactor());
}

uint8_t LoRaRadioBase::getActiveSyncWord() const
{
	/* buildModemConfig() currently sets public_network=false, which maps
	 * Zephyr's LoRa API to the Semtech private sync word. */
	return 0x12;
}

int8_t LoRaRadioBase::getConfiguredTxPower() const
{
	int power = _prefs ? _prefs->tx_power_dbm : LoRaConfig::TX_POWER_DBM;

#ifdef CONFIG_ZEPHCORE_MAX_TX_POWER_DBM
	if (power > CONFIG_ZEPHCORE_MAX_TX_POWER_DBM) {
		power = CONFIG_ZEPHCORE_MAX_TX_POWER_DBM;
	}
#endif
	if (power < -9) {
		power = -9;
	}
	return (int8_t)power;
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
	       a.public_network == b.public_network &&
	       a.cad.mode == b.cad.mode;
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
	       a.cad.mode == b.cad.mode &&
	       a.tx != b.tx;
}

void LoRaRadioBase::configure(bool tx)
{
	struct lora_modem_config cfg;
	buildModemConfig(cfg, tx);

	const char *who = tx ? "configureTx" : "configureRx";

	if (_config_cached && configParamsEqual(cfg, _last_cfg)) {
		LOG_DBG("%s: params unchanged, skipping hwConfigure", who);
		return;
	}

	/* Fast path: if only the TX/RX direction changed, skip the full
	 * hwConfigure → lora_config() call.  The driver already has a valid
	 * config for the target direction (RadioSetRxConfig / RadioSetTxConfig
	 * with TxTimeout=4000) from a previous cycle — Radio.Rx(0) / Radio.Send()
	 * will use those register values directly.  This avoids the
	 * modem_acquire → modem_release → Radio.Sleep() round-trip that wastes
	 * ~5 ms on every TX↔RX transition.
	 *
	 * Not used for loramac-node: Radio.SetTxConfig() and Radio.SetRxConfig()
	 * configure completely disjoint internal state (including TxTimeout).
	 * Skipping either on a direction change leaves that state uninitialized. */
	if (!_loramac_node && _config_cached && onlyDirectionDiffers(cfg, _last_cfg)) {
		LOG_DBG("%s: direction-only change, skip hwConfigure", who);
		_last_cfg = cfg;
		return;
	}

	if (!tx) {
		LOG_DBG("configureRx: freq=%u bw=%d sf=%d cr=%d pwr=%d",
			cfg.frequency, (int)cfg.bandwidth, (int)cfg.datarate,
			(int)cfg.coding_rate, cfg.tx_power);
	}

	if (hwConfigure(cfg)) {
		_last_cfg = cfg;
		_config_cached = true;
	} else {
		_config_cached = false;
	}
}

void LoRaRadioBase::configureRx() { configure(false); }
void LoRaRadioBase::configureTx() { configure(true); }

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

	/* Sync _rx_boost_enabled to the driver.  The driver initialises its own
	 * rx_boost_enabled flag from DTS (rx-boosted property), which may differ
	 * from our constructor default (true).  Push our intent now so the
	 * hardware state matches _rx_boost_enabled from the moment begin()
	 * returns, before the caller applies prefs via setRxBoost(). */
	hwSetRxBoost(_rx_boost_enabled);

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
	/* CAD probe statistics are only valid for one freq/SF/BW config. */
	resetCadStats();
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
		/* Duty-cycle window sizing.  All constraints primary-sourced
		 * (SX1261/2 DS rev 2.2 §13.1.7 + AN1200.36):
		 *
		 * 1. Catch — worst case is a preamble starting D−ε symbols
		 *    before an RX window closes (detection aborts, must
		 *    complete in the NEXT window), so the total deaf time per
		 *    cycle (programmed sleep + wake transition) must satisfy
		 *      sleep + trans ≤ (P − 2D − 1)·Tsym
		 *    with 1 symbol margin for the chip's RC64k sleep timer.
		 *    Stricter than AN1200.36's own "P ≥ sleep + D" model,
		 *    which ignores the window-tail arrival case.
		 * 2. Complete — DS: "Tpreamble + Theader ≤ 2·rxPeriod +
		 *    sleepPeriod".  A preamble detected at its first symbols
		 *    restarts the chip timer with 2R+S; that budget must cover
		 *    the rest of the preamble + sync (4.25) + header (~8),
		 *    rounded up to P + 14 symbols.  The driver also sets
		 *    StopTimerOnPreamble, but this sizing keeps packets safe
		 *    under either documented timer behaviour.
		 * 3. Floor — rxPeriod ≥ (D+1)·Tsym so any single window can
		 *    detect on its own.
		 *
		 * No viable sleep budget (short preamble, or the TCXO restart
		 * eats it) → honest fall-through to continuous RX. */
		struct lora_modem_config cfg;
		buildModemConfig(cfg, false);

		const uint8_t sf = (uint8_t)cfg.datarate;
		const uint32_t bw_hz = bandwidth_to_hz(cfg.bandwidth);
		const uint16_t P = cfg.preamble_len;
		const uint16_t D = rxDutyDetectSymbols(sf);

		if (bw_hz > 0 && P > 2 * D + 1) {
			const uint32_t sym_us = (uint32_t)
				(((uint64_t)(1U << sf) * 1000000ULL) / bw_hz);
			const uint32_t trans_us = hwWakeupTimeUs();

			/* Theoretical per-cycle deaf budget, then derate by
			 * CONFIG_..._MARGIN_PCT.  The budget assumes the sleep
			 * clock and wake transition are exact; in reality the
			 * chip sleep timer runs on an RC oscillator that drifts
			 * several % over temperature and the wake transition is a
			 * "may vary" datasheet figure.  Either overshoot pushes
			 * real deaf time past the budget and drops phase-edge
			 * packets (strength-independent DC loss).  Deraging the
			 * whole budget is slightly stricter than deraging sleep
			 * alone, which is the safe direction. */
			const uint32_t deaf_budget_us =
				(uint32_t)(P - 2 * D - 1) * sym_us;
			const uint32_t deaf_us = deaf_budget_us -
				(uint32_t)(((uint64_t)deaf_budget_us *
					    CONFIG_ZEPHCORE_LORA_DC_MARGIN_PCT) /
					   100U);

			if (deaf_us > trans_us + 2000) {
				const uint32_t sleep_us = deaf_us - trans_us;
				const uint32_t complete_us =
					(uint32_t)(P + 14) * sym_us;
				uint32_t rx_us = (uint32_t)(D + 1) * sym_us;

				if (complete_us > sleep_us &&
				    rx_us < (complete_us - sleep_us + 1) / 2) {
					rx_us = (complete_us - sleep_us + 1) / 2;
				}

				if (rx_us != _dc_last_rx_us ||
				    sleep_us != _dc_last_sleep_us) {
					_dc_last_rx_us = rx_us;
					_dc_last_sleep_us = sleep_us;
					LOG_INF("rxduty: rx=%ums sleep=%ums trans=%ums (P=%u D=%u, off=%u%%)",
						rx_us / 1000, sleep_us / 1000,
						trans_us / 1000, P, D,
						(uint32_t)(((uint64_t)sleep_us * 100) /
							   (rx_us + sleep_us + trans_us)));
				}

				ret = lora_recv_duty_cycle(_dev,
							   K_USEC(rx_us),
							   K_USEC(sleep_us),
							   rxCallbackStatic, this);
				if (ret == 0) {
					_rx_entry_cyc = k_cycle_get_32();
					atomic_set(&_in_recv_mode, 1);
					return;
				}
				if (ret != -ENOSYS) {
					LOG_ERR("lora_recv_duty_cycle failed: %d", ret);
				}
				/* Fall through to continuous RX */
			} else if (_dc_last_rx_us != UINT32_MAX) {
				_dc_last_rx_us = UINT32_MAX;
				LOG_INF("rxduty: wake transition %uus exceeds deaf budget %uus — continuous RX",
					trans_us, deaf_us);
			}
		} else if (_dc_last_rx_us != UINT32_MAX) {
			_dc_last_rx_us = UINT32_MAX;
			LOG_INF("rxduty: preamble %u too short for guaranteed catch (need >%u syms) — continuous RX",
				P, 2 * D + 1);
		}
	}

	ret = lora_recv_async(_dev, rxCallbackStatic, this);
	if (ret < 0) {
		LOG_ERR("lora_recv_async failed: %d", ret);
		atomic_set(&_in_recv_mode, 0);
		return;
	}
	_rx_entry_cyc = k_cycle_get_32();
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

	/* Last-moment software check before killing active RX.  Uses the full
	 * isReceiving() (latch + non-destructive raw bits) so the final gate
	 * honors the same source of truth as the dispatcher's earlier gates.
	 * Closes the serialisation/logging gap between the dispatcher's check
	 * and the TX-state transition below. */
	if (isReceiving()) {
		return false;
	}

	_board->onBeforeTransmit();
	atomic_set(&_tx_active, 1);

	/* Phase 2: when LBT is enabled, skip the pre-emptive hwCancelReceive()
	 * and keep _in_recv_mode = 1 so the driver's send_async sees state == RX
	 * (the Phase-2 entry CAS path).  On CAD-busy the driver restores RX
	 * internally; on success the chip transitions cleanly into TX without
	 * the redundant ~1–3 ms C++ cancel-then-restart round-trip.
	 * isReceiving() returns false during the CAD window because _tx_active
	 * is set above — no extra gating needed.
	 *
	 * cad.mode = LBT is set unconditionally in buildModemConfig() today;
	 * the `lbt` flag is a placeholder for any future Kconfig that toggles
	 * the behaviour. */
	const bool lbt = true;

	if (!lbt) {
		atomic_set(&_in_recv_mode, 0);
		hwCancelReceive();
	}
	configureTx();

	memcpy(_tx_buf, bytes, len);
	k_poll_signal_reset(&_tx_signal);

	int ret = hwSendAsync(_tx_buf, (uint32_t)len, &_tx_signal);
	if (ret < 0) {
		LOG_ERR("hwSendAsync failed: %d", ret);
		_board->onAfterTransmit();
		atomic_set(&_tx_active, 0);
		/* startReceive() is safe to call here regardless of failure
		 * cause: on SX126x, recv_async early-returns if the driver
		 * already restored RX on CAD-busy (Phase 2 idempotent fast
		 * path); on LR11xx/LR20xx, the LBT branch restores RX before
		 * returning -EBUSY (Phase 2 mirror), so start_rx is also a
		 * no-op there.  On other failure modes the chip is in REST,
		 * recv_async transitions normally. */
		startReceive();
		return false;
	}

	/* TX has actually started — now we're no longer in RX. */
	atomic_set(&_in_recv_mode, 0);

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

	/* LDRO threshold must track the SX126x driver's should_enable_ldro()
	 * exactly (symbol time > 16.38 ms) so this estimate's DE matches the
	 * hardware's DE on every SF/BW pair.  The old `sf >= 11` was only
	 * correct at BW 125 kHz and diverged on every other bandwidth. */
	float de = (t_sym > 0.01638f) ? 1.0f : 0.0f;
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

	/* Own the sampling cadence rather than inheriting the caller's.  Early
	 * calls are a no-op, so this is safe to invoke from any wake. */
	int64_t now = k_uptime_get();

	/* Invalidate first: "fresh" must mean a sample landed in THIS pass, not
	 * merely at some point in the past.  cadMaintenance() runs immediately
	 * after us and treats the verdict as current-channel ground truth, so a
	 * carried-over sample would let it probe on a reading taken a full
	 * interval ago — on a different channel state entirely. */
	_sample_fresh = false;

	if (_noise_floor_next_ms != 0 && now < _noise_floor_next_ms) {
		return;
	}

	/* Due.  Any bail-out below is a blocked attempt, not a completed one —
	 * push the deadline out by the retry so msUntilNextMaintenance() cannot
	 * report "due now" on a loop.  Bounded for the same reason as the CAD
	 * probe: an unbounded retry grid makes the retry period the de-facto
	 * wake period whenever the radio is persistently busy. */
	if (_noise_floor_retries >= NOISE_FLOOR_MAX_RETRIES) {
		_noise_floor_retries = 0;
		_noise_floor_next_ms = now + _measure_interval_ms;
		return;
	}
	_noise_floor_retries++;
	_noise_floor_next_ms = now + NOISE_FLOOR_RETRY_MS;

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

	/* GetRssiInst needs time after RX entry before the first value is
	 * valid (DS Table 13-82).  isRadioReady() only clears BUSY, and the
	 * delay is measured *from* the BUSY falling edge, so BUSY alone does
	 * not prove the reading has settled.  Only host-driven RX entries are
	 * stamped: under RX duty cycle the sleep->RX wakes are chip-internal
	 * and invisible to us.  That is acceptable rather than ideal — the
	 * delay is ~0.25 ms at BW 62.5 against an RX window orders of
	 * magnitude longer, so the odds of a duty-cycle sample landing inside
	 * an unsettled window are small, and the median absorbs the odd one. */
	uint16_t bw_khz = (uint16_t)(getActiveBandwidthKHzX10() / 10);
	uint32_t since_rx_us =
		k_cyc_to_us_floor32(k_cycle_get_32() - _rx_entry_cyc);

	if (since_rx_us < rssi_settle_delay_us(bw_khz)) {
		return;
	}

	/* Median of multiple RSSI reads: no downward bias of min, no spike
	 * sensitivity of average.  Insertion sort is fine for N=8 (28
	 * comparisons worst case, all in registers).
	 *
	 * Scope, measured on-air 2026-07-29 (`get cad` sp field, BW 62.5):
	 * 84-90%% of bursts return N identical values, and the rest average
	 * ~6 dB of spread.  The reads ARE independent -- the degenerate share
	 * falls and the spread rises when ambient comes up, exactly as it
	 * should.  The burst is simply short: ~300 us against a ~200 ms
	 * SF8/BW62.5 packet, about 0.15%% of one transmission.  So a
	 * neighbour's packet is either wholly inside the burst or wholly
	 * outside it, every read sees the same level, and the median returns
	 * it rather than rejecting it.
	 *
	 * What this median actually buys is rejection of sub-300 us glitches
	 * and single bad SPI reads.  That is worth its ~300 us every 15 s, but
	 * it is NOT the defence against interference -- that is the
	 * isReceiving() guard above and the floor + SAMPLING_THRESHOLD filter
	 * below.  An earlier comment here claimed "rejects up to N/2-1
	 * outliers", which credited the median with their work.
	 *
	 * Reads are spaced by the RSSI averaging window, without which they
	 * can all fall inside one window and return the same underlying
	 * sample N times — a median of N copies of one read.  At BW 62.5 the
	 * spacing (~16 us) is already covered by the SPI transaction itself;
	 * it matters at the narrow presets, where the window grows past the
	 * whole burst. */
	uint32_t window_us = rssi_avg_window_us(bw_khz);
	int16_t samples[NOISE_FLOOR_SAMPLES_PER_TICK];
	for (int i = 0; i < NOISE_FLOOR_SAMPLES_PER_TICK; i++) {
		if (i) {
			k_busy_wait(window_us);
		}
		samples[i] = hwGetCurrentRSSI();
		if (samples[i] == -128) {
			/* Chip busy or RSSI read contended — keep the short
			 * retry deadline set above and try again shortly. */
			return;
		}
	}

	/* A full sample landed: next one is a full interval away. */
	_noise_floor_next_ms = now + _measure_interval_ms;
	_noise_floor_retries = 0;

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

	/* Burst quality, reported by `get cad`.  Sorted, so max-min is the
	 * spread.  Kept as running totals rather than an EMA so the numbers
	 * stay readable and the degenerate share is a true proportion.
	 *
	 * Reading it: a high zero-spread share on its own is NOT a fault — a
	 * quiet or steadily-occupied channel genuinely reads the same value
	 * N times at integer-dB resolution.  What would indict the sampler is
	 * a high share together with a mean of 0.0, i.e. no burst ever spans
	 * anything: that is reads landing inside one RSSI averaging window and
	 * returning one sample N times over.  A non-zero mean proves the reads
	 * are independent however high the share climbs. */
	_rssi_bursts++;
	_rssi_spread_sum += (uint32_t)(samples[NOISE_FLOOR_SAMPLES_PER_TICK - 1] -
				       samples[0]);
	if (samples[NOISE_FLOOR_SAMPLES_PER_TICK - 1] == samples[0]) {
		_rssi_degenerate++;
	}
	/* Rescale together so both derived figures survive untouched, and the
	 * printed count stays four digits however long the node is up. */
	if (_rssi_bursts >= RSSI_BURST_STATS_CAP) {
		_rssi_bursts >>= 1;
		_rssi_spread_sum >>= 1;
		_rssi_degenerate >>= 1;
	}

	/* Publish this sample for cadMaintenance().  The CAD probe needs exactly
	 * the same fact we just established — "is the channel at its floor right
	 * now?" — and used to answer it with its own single hwGetCurrentRSSI() on
	 * its own deadline.  That cost a second wake per interval (measured: two
	 * 15 s grids ~3 s apart) and made the worse decision, since one raw read
	 * is precisely what the median-of-8 exists to defend against.
	 *
	 * The verdict is taken against the floor BEFORE this sample is folded in,
	 * so it compares a new observation to the established floor rather than
	 * to one already dragged toward it. */
	_sample_rssi = rssi;
	_sample_channel_quiet = (_noise_floor == DEFAULT_NOISE_FLOOR) ||
				(rssi <= _noise_floor + CAD_PROBE_RSSI_GUARD);
	_sample_fresh = true;

	/* First sample after reset (DEFAULT_NOISE_FLOOR == 0): seed directly.
	 * The lower clamp tracks the active bandwidth — thermal noise is
	 * 10*log10(BW) so a fixed rail pins narrow-BW presets several dB high
	 * (BW 31.25 kHz sits ~3 dB below BW 62.5) and never engages at all on
	 * wide ones. */
	int16_t floor_min = noise_floor_min_dbm(getActiveBandwidthKHzX10() / 10);

	if (_noise_floor == DEFAULT_NOISE_FLOOR) {
		_noise_floor = rssi;
		if (_noise_floor < floor_min) _noise_floor = floor_min;
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
	if (_noise_floor < floor_min) _noise_floor = floor_min;
	if (_noise_floor > -50) _noise_floor = -50;

	LOG_DBG("noise_floor_cal: rssi=%d, floor=%d, tick=%u",
		rssi, _noise_floor, _ema_unguarded - 1);
}

bool LoRaRadioBase::isReceiving()
{
	if (!atomic_get(&_in_recv_mode) || atomic_get(&_tx_active)) {
		return false;
	}
	/* Driver-side latch + non-destructive IRQ read covers the full
	 * payload phase.  hwIsReceiving() never clears IRQ bits; foreign
	 * preambles release via hardware (SymbNumTimeout on SX126x non-DC
	 * or chip-internal sync timer on DC / LR11xx / LR20xx). */
	if (hwIsReceiving()) {
		return true;
	}
	return isChannelActive();
}

void LoRaRadioBase::recoverRxState()
{
	/* Called by the Dispatcher on CAD timeout when isReceiving() has been
	 * pinned true past the recovery threshold (4 s).  We must escape a
	 * stuck driver state == RX — a bare startReceive() can't do this
	 * because the driver's lora_recv_async entry CAS is REST_STATE → RX,
	 * which fails when state is already RX and would set _in_recv_mode = 0
	 * on the -EBUSY return.  Walk the chip back through REST first.
	 *
	 * The RX-restart sites in the driver (recv_async, recv_duty_cycle,
	 * restart_rx) all bulk-clear IRQ status and reset the rx_packet_active
	 * latch as part of their entry, so this sequence cleanly flushes a
	 * stuck PREAMBLE_DETECTED bit or a stale latch. */
	hwCancelReceive();
	atomic_set(&_in_recv_mode, 0);
	_config_cached = false;
	startReceive();
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

/* ── Adaptive CAD (LBT detPeak calibration) ───────────────────────────── */

void LoRaRadioBase::setCadParams(bool auto_enabled, int8_t offset,
				 uint16_t probe_interval_s, uint8_t busycap_pct)
{
	if (offset < CAD_LEVEL_MIN) offset = CAD_LEVEL_MIN;
	if (offset > CAD_LEVEL_MAX) offset = CAD_LEVEL_MAX;

	_cad_auto = auto_enabled;
	_cad_offset = offset;
	_probe_interval_s = probe_interval_s;
	_cad_busycap_pct = busycap_pct;

	/* One interval governs every periodic radio measurement, because there
	 * is only one measurement: the noise-floor sampler takes a median-of-8
	 * and the CAD probe consumes that same reading (see cadMaintenance).
	 * Splitting them into two knobs could only ever express a rate the
	 * hardware does not actually run at.
	 *
	 * 0 means "CAD probing off" — the floor sampler still has to run, so it
	 * falls back to the build-time default. */
	_measure_interval_ms = probe_interval_s
			       ? (uint32_t)probe_interval_s * 1000U
			       : (uint32_t)CONFIG_ZEPHCORE_NOISE_FLOOR_INTERVAL_MS;

	hwCadSetPeakOffset(_cad_offset);

	LOG_INF("cad: auto=%d offset=%d measure_interval=%ums busycap=%u%%",
		(int)auto_enabled, (int)offset, (unsigned)_measure_interval_ms,
		(unsigned)busycap_pct);
}

void LoRaRadioBase::resetCadStats()
{
	memset(_cad_stats, 0, sizeof(_cad_stats));
	_cad_probe_rr = 0;
}

void LoRaRadioBase::decayCadStats()
{
	for (int i = 0; i < CAD_NUM_LEVELS; i++) {
		_cad_stats[i].probes >>= 1;
		_cad_stats[i].busy >>= 1;
		_cad_stats[i].fp >>= 1;
		_cad_stats[i].tp >>= 1;
	}
}

int8_t LoRaRadioBase::pickCadProbeLevel()
{
	_cad_probe_rr++;

	if (!_cad_auto) {
		/* Dry-run: even sweep across the observation window so the
		 * user sees the whole FP-vs-detPeak curve in `get cad`. */
		int span = CAD_SWEEP_MAX - CAD_SWEEP_MIN + 1;

		return (int8_t)(CAD_SWEEP_MIN + (_cad_probe_rr % span));
	}

	/* Auto: sample the operating level AND both neighbours so the staircase
	 * can read the local FP curvature (slope below vs. above) and seek the
	 * knee.  op is the shared term of both slopes → weight it half; each
	 * neighbour a quarter.  Out-of-range neighbours fall back to op. */
	int8_t lvl;
	switch (_cad_probe_rr & 3) {
	case 1:  lvl = (int8_t)(_cad_offset - 1); break;  /* more sensitive */
	case 3:  lvl = (int8_t)(_cad_offset + 1); break;  /* less sensitive */
	default: lvl = _cad_offset; break;                /* operating (0, 2) */
	}
	if (lvl < CAD_LEVEL_MIN || lvl > CAD_LEVEL_MAX) {
		lvl = _cad_offset;
	}
	return lvl;
}

void LoRaRadioBase::cadStaircaseStep()
{
	/* Knee-seeking controller — see the CAD_KNEE_SLOPE / CAD_PLATEAU_CLEAN
	 * notes in radio_common.h.  Reads local curvature from three rungs and
	 * steps toward the knee (the most sensitive detPeak whose FP has already
	 * bottomed out), using slopes so the decision is site-floor-independent. */
	int oi = _cad_offset - CAD_LEVEL_MIN;

	auto warm = [&](int idx) -> bool {
		return idx >= 0 && idx < CAD_NUM_LEVELS &&
		       _cad_stats[idx].probes >= CAD_STEP_MIN_PROBES;
	};
	/* Per-level FALSE-positive rate in permille, or -1 when too few samples. */
	auto fp_rate = [&](int idx) -> int {
		if (!warm(idx)) {
			return -1;
		}
		return (int)(((uint32_t)_cad_stats[idx].fp * 1000U)
			     / _cad_stats[idx].probes);
	};
	/* Per-level TOTAL busy (defer) rate in permille — false + real traffic. */
	auto busy_rate = [&](int idx) -> int {
		if (!warm(idx)) {
			return -1;
		}
		return (int)(((uint32_t)_cad_stats[idx].busy * 1000U)
			     / _cad_stats[idx].probes);
	};

	int r_op = fp_rate(oi);
	if (r_op < 0) {
		return;  /* operating level not warm yet — no basis to step */
	}
	int b_op = busy_rate(oi);
	int r_up = fp_rate(oi + 1);  /* one step less sensitive */
	int r_dn = fp_rate(oi - 1);  /* frontier, one step more sensitive */

	/* Airtime protection (highest priority): if the operating level defers
	 * too large a fraction of TX attempts — real traffic included — back off
	 * to a less sensitive detPeak.  On a congested hilltop most of that busy
	 * is distant traffic we'd win on capture anyway; deferring for all of it
	 * just starves our own airtime.  Cap is `set cad.busycap` percent (0 =
	 * off); only binds on genuinely busy channels. */
	int cap_permille = (int)_cad_busycap_pct * 10;
	if (cap_permille && _cad_offset < CAD_LEVEL_MAX && b_op > cap_permille) {
		_cad_offset++;
		hwCadSetPeakOffset(_cad_offset);
		LOG_INF("cad: step up -> offset %d (airtime, busy %d cap %d)",
			(int)_cad_offset, b_op, cap_permille);
		return;
	}

	/* Step UP (less sensitive) when the level above is markedly cleaner —
	 * we're on the steep part of the curve, below the knee. */
	if (_cad_offset < CAD_LEVEL_MAX && r_up >= 0 &&
	    r_op - r_up >= CAD_KNEE_SLOPE_PERMILLE) {
		_cad_offset++;
		hwCadSetPeakOffset(_cad_offset);
		LOG_INF("cad: step up -> offset %d (op %d dn->up %d)",
			(int)_cad_offset, r_op, r_up);
		return;
	}

	/* Step DOWN (more sensitive) only on a flat plateau that is already
	 * clean: the frontier is no worse than operating (nothing to lose) AND
	 * FP here is low enough that reclaiming sensitivity is cheap.  The clean
	 * guard keeps a flat-but-noisy curve from descending to the sensitive
	 * rail; the busy-hysteresis guard keeps us from descending into the
	 * airtime cap and bouncing straight back up. */
	int b_dn = busy_rate(oi - 1);
	bool busy_ok = (cap_permille == 0) ||
		       (b_dn <= cap_permille - CAD_BUSY_DEFER_HYST_PERMILLE);
	if (_cad_offset > CAD_LEVEL_MIN && r_dn >= 0 &&
	    r_dn - r_op < CAD_KNEE_SLOPE_PERMILLE &&
	    r_op <= CAD_PLATEAU_CLEAN_PERMILLE && busy_ok) {
		_cad_offset--;
		hwCadSetPeakOffset(_cad_offset);
		LOG_INF("cad: step down -> offset %d (op %d dn %d busy %d)",
			(int)_cad_offset, r_op, r_dn, b_dn);
		return;
	}

	/* Otherwise: at the knee (steep below, flat above) or a noisy flat
	 * plateau — hold. */
}

void LoRaRadioBase::cadMaintenance()
{
	if (_probe_interval_s == 0) {
		return;
	}

	int64_t now = k_uptime_get();

	/* Periodic decay keeps the stats fresh (and counters bounded). */
	if (_cad_last_decay_ms == 0) {
		_cad_last_decay_ms = now;
	} else if (now - _cad_last_decay_ms > (int64_t)CAD_STATS_DECAY_MS) {
		decayCadStats();
		_cad_last_decay_ms = now;
	}

	/* No separate probe-interval check: the probe interval IS the measurement
	 * interval (setCadParams derives _measure_interval_ms from it), so a
	 * fresh sample means a probe is due by construction. */

	/* Ride on the noise-floor sampler rather than measuring independently.
	 *
	 * A fresh sample means the sampler ran THIS pass, which already proves
	 * everything the probe needs: the radio was idle in RX, not transmitting,
	 * not mid-packet, and out of its duty-cycle sleep window — the sampler
	 * applies exactly those guards before it reads.  So there is nothing left
	 * to re-check, no separate deadline, and no retry budget: if no sample
	 * landed this pass, the probe simply waits for the next one.
	 *
	 * This is what makes the wake cost one per interval instead of two.  It
	 * also upgrades the ground-truth prefilter from a single raw RSSI read to
	 * the sampler's median-of-8 — the probe is trying to establish that the
	 * channel is quiet, and a busy verdict taken over real traffic teaches
	 * nothing about false positives, so the outlier rejection matters here. */
	if (!_sample_fresh) {
		return;
	}
	_sample_fresh = false;

	if (!_sample_channel_quiet) {
		return;
	}

	_cad_last_probe_ms = now;

	int8_t level = pickCadProbeLevel();
	int ret = hwCadProbe(level);

	/* The probe leaves the chip in STANDBY (driver state REST) — re-arm
	 * RX immediately so an incoming packet isn't lost while we classify.
	 * In duty-cycle mode this re-enters the DC cycle (same path as the
	 * parked-RX watchdog re-arm). */
	atomic_set(&_in_recv_mode, 0);
	startReceive();

	if (ret < 0) {
		if (ret != -ENOSYS) {
			LOG_WRN("cad: probe failed (%d)", ret);
		}
		return;
	}

	CadLevelStats &s = _cad_stats[level - CAD_LEVEL_MIN];

	if (s.probes >= 0xFFF0) {
		decayCadStats();
	}
	s.probes++;

	if (ret > 0) {
		s.busy++;

		/* Ground-truth post-check: was the CAD hit a REAL signal or a
		 * correlator false positive?  A real transmitter that tripped
		 * CAD keeps radiating, so over the next preamble+header window
		 * one of two things shows up:
		 *   (a) the restarted RX syncs on it   -> isReceiving(), or
		 *   (b) instantaneous RSSI climbs above the noise floor.
		 * (b) is the important addition: the probe tears RX down to run
		 * CAD, and the STANDBY->RX restart routinely eats the preamble of
		 * a real packet, so RX never re-syncs — the old isReceiving()-only
		 * snapshot booked those strong-but-missed packets as false
		 * positives, a ~detPeak-independent floor that flattened the FP
		 * curve and drove the staircase to the ceiling.  Channel energy
		 * doesn't depend on winning the preamble race, so it recovers
		 * them.  Neither signal over the whole window => genuine FP.  A
		 * below-floor packet we can neither sync nor see stays ambiguous
		 * and counts as FP — bias toward higher detPeak (the safe side).
		 * The prefilter above guaranteed RSSI <= floor+guard pre-probe,
		 * so a rise past that threshold now is a newly-arrived signal. */
		uint8_t sf = getActiveSpreadingFactor();
		uint16_t bw_x10 = getActiveBandwidthKHzX10();
		uint32_t tsym_us = bw_x10 ? (uint32_t)(((1UL << sf) * 10000UL)
						       / bw_x10) : 1024;
		uint32_t step_ms = (3U * tsym_us) / 1000U;  /* ~3 symbols/sample */

		if (step_ms < 5) step_ms = 5;
		if (step_ms > 100) step_ms = 100;

		bool floor_valid = (_noise_floor != DEFAULT_NOISE_FLOOR);
		int16_t rssi_thresh = _noise_floor + CAD_PROBE_RSSI_GUARD;

		bool real = false;
		for (int k = 0; k < 4 && !real; k++) {  /* ~12 symbols total */
			k_sleep(K_MSEC(step_ms));
			if (isReceiving()) {
				real = true;
			} else if (floor_valid &&
				   hwGetCurrentRSSI() > rssi_thresh) {
				real = true;
			}
		}

		if (real) {
			s.tp++;
		} else {
			s.fp++;
		}
	}

	if (_cad_auto) {
		cadStaircaseStep();
	}
}

/* int64 uptime delta → the uint32 "ms from now" the maintenance contract wants.
 * Already-passed deadlines saturate at 0 (due now), far-future ones at IDLE. */
static uint32_t clampDeadline(int64_t remaining_ms)
{
	if (remaining_ms <= 0) {
		return 0;
	}
	if (remaining_ms >= (int64_t)mesh::MAINTENANCE_IDLE) {
		return mesh::MAINTENANCE_IDLE;
	}
	return (uint32_t)remaining_ms;
}

/* When does this radio next need a maintenance call?  Two independent items:
 * the noise floor sampler (always running) and the CAD calibrator (only when
 * probing is enabled).  Both hold absolute uptime deadlines, so this is a pure
 * read — it must not touch the chip, since the event loop calls it on every
 * wake to decide how long it may sleep. */
uint32_t LoRaRadioBase::msUntilNextMaintenance()
{
	int64_t now = k_uptime_get();
	uint32_t next = mesh::MAINTENANCE_IDLE;

	/* Noise floor.  A zero deadline means "never sampled yet" — due now. */
	if (_noise_floor_next_ms == 0) {
		return 0;
	}
	next = clampDeadline(_noise_floor_next_ms - now);

	if (_probe_interval_s == 0) {
		return next;
	}

	/* The CAD probe deliberately contributes NO deadline of its own.  It runs
	 * off the noise-floor sampler's measurement (see cadMaintenance), so its
	 * wake is already accounted for above.  Giving it a second deadline is
	 * what produced two independent 15 s grids ~3 s apart — one extra wake
	 * per interval, forever, on every repeater. */

	/* Stats decay. _cad_last_decay_ms == 0 means the first call latches it
	 * rather than decaying, so treat that as due now. */
	if (_cad_last_decay_ms == 0) {
		return 0;
	}
	return mesh::maintenanceSooner(
		next, clampDeadline(_cad_last_decay_ms + (int64_t)CAD_STATS_DECAY_MS - now));
}

int LoRaRadioBase::formatCadStatus(char *buf, int cap)
{
	uint8_t base = hwCadBasePeak();
	int n = 0;

	if (base == 0) {
		return snprintf(buf, cap, "cad n/a");
	}

	/* Terse on purpose — remote replies are capped at ~160 B over LoRa.
	 * Header:  a:on o:1 pk:22(b21/4s) sp:0.9/84%(312) bc:25%
	 *   a  auto on/off   o  offset   pk operating peak
	 *   b  family base   4s symbols   bc busy cap
	 *   sp RSSI burst quality: mean spread in dB across the median-of-N
	 *      reads, the share of bursts whose spread was 0, and the burst
	 *      count.  The count is not decoration: a share without its
	 *      denominator cannot be read, and the burst rate is not
	 *      derivable from uptime because the sampler's guards (TX, mid-RX,
	 *      duty-cycle sleep) block an unknown fraction of attempts.
	 * Level:  *+1(22) 22p 18b 16f 2t 72%
	 *   '*' = operating rung   level(peak)  probes busy fp tp  fp-rate%%.
	 *
	 * sp replaced the probe interval here because the interval is a pref
	 * you already set and can read back with `get probe.interval`, whereas
	 * burst spread is only observable from inside the sampler.
	 *
	 * It answers one question: are the N reads independent?  A non-zero
	 * mean proves they are, whatever the zero-spread share — a steady
	 * channel reads identically at integer-dB resolution, which is correct
	 * rather than broken.  Only mean 0.0 with a high share indicts the
	 * sampler: that is N copies of one sample from inside a single RSSI
	 * averaging window (see rssi_avg_window_us() in radio_common.h).
	 * Measured on-air 2026-07-29 at BW 62.5: 0.6/90% quiet, 0.9/84% with
	 * the floor at -103 — independent, and responding the right way.
	 * Mean is tenths of a dB.  Counters halve at RSSI_BURST_STATS_CAP, so
	 * the count is bounded to four digits and the figures describe a
	 * recent window rather than everything since boot. */
	unsigned spread_mean10 = _rssi_bursts
		? (unsigned)((_rssi_spread_sum * 10U + _rssi_bursts / 2U) /
			     _rssi_bursts)
		: 0;
	unsigned degen_pct = _rssi_bursts
		? (unsigned)((_rssi_degenerate * 100U + _rssi_bursts / 2U) /
			     _rssi_bursts)
		: 0;

	/* The burst count is bench diagnostics, and the header competes with
	 * the three level rows for a 161 B remote reply — with wide level
	 * counters the full header pushes the last row into truncation.  So
	 * print it only into the roomy local-console buffer; a remote reader
	 * still gets the mean and the share, which is the actual verdict. */
	bool room_for_count = (cap >= 200);

	n += snprintf(buf + n, cap > n ? cap - n : 0,
		      "a:%s o:%d pk:%d(b%u/4s) sp:%u.%u/%u%%",
		      _cad_auto ? "on" : "off", (int)_cad_offset,
		      (int)base + _cad_offset, base,
		      spread_mean10 / 10U, spread_mean10 % 10U, degen_pct);
	if (room_for_count) {
		n += snprintf(buf + n, cap > n ? cap - n : 0, "(%u)",
			      (unsigned)_rssi_bursts);
	}
	n += snprintf(buf + n, cap > n ? cap - n : 0, " bc:%u%%",
		      (unsigned)_cad_busycap_pct);

	/* Only the 3 rungs around the operating offset — the far rungs are mildly
	 * irrelevant; what matters is where we sit on the ladder. The window is
	 * clamped to stay inside [CAD_LEVEL_MIN, CAD_LEVEL_MAX] while still showing
	 * 3 rungs, so at either end it slides inward rather than dropping a line. */
	int cur = _cad_offset;
	if (cur < CAD_LEVEL_MIN) cur = CAD_LEVEL_MIN;
	if (cur > CAD_LEVEL_MAX) cur = CAD_LEVEL_MAX;
	int lo = cur - 1, hi = cur + 1;
	if (lo < CAD_LEVEL_MIN) { lo = CAD_LEVEL_MIN; hi = lo + 2; }
	if (hi > CAD_LEVEL_MAX) { hi = CAD_LEVEL_MAX; lo = hi - 2; }

	for (int lvl = lo; lvl <= hi; lvl++) {
		CadLevelStats &s = _cad_stats[lvl - CAD_LEVEL_MIN];

		/* Integer FP rate, rounded to nearest percent (0 when unprobed). */
		unsigned fp_pct = s.probes
			? (unsigned)(((uint32_t)s.fp * 100U + s.probes / 2) / s.probes)
			: 0;

		n += snprintf(buf + n, cap > n ? cap - n : 0,
			      "\n%c%+d(%d) %up %ub %uf %ut %u%%",
			      lvl == cur ? '*' : ' ',
			      lvl, (int)base + lvl,
			      s.probes, s.busy, s.fp, s.tp, fp_pct);
	}

	return n;
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

bool LoRaRadioBase::setRxBoost(bool enable)
{
	_rx_boost_enabled = enable;
	LOG_INF("RX boost %s (+3dB sensitivity, +2mA)",
		enable ? "enabled" : "disabled");
	if (atomic_get(&_in_recv_mode)) {
		hwSetRxBoost(enable);
	}
	return true;
}

} /* namespace mesh */
