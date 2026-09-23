/*
 * SPDX-License-Identifier: MIT
 * LoRa radio base class — shared algorithms for all radio adapters.
 */

#include "LoRaRadio.h"
#include "radio_common.h"
#include "pm_sleep_guard.h"
#include <mesh/MeshCore.h>   /* MAX_TRANS_UNIT */
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

LoRaRadio::LoRaRadio(const struct device *lora_dev, MainBoard &board)
	: _ops(kLoRaRadioOps),
	  _dev(lora_dev), _prefs(nullptr), _board(&board),
	  _in_recv_mode(0), _tx_active(0), _tx_complete(0),
	  _last_rssi(0), _last_snr(0),
	  _rx_head(0), _rx_tail(0),
	  _calibration_threshold(0),
	  _noise_floor_next_ms(0), _noise_floor_retries(0),
	  _measure_interval_ms(CONFIG_ZEPHCORE_NOISE_FLOOR_INTERVAL_MS),
	  _sample_rssi(0), _sample_channel_quiet(false), _sample_fresh(false),
	  _rx_entry_cyc(0),
	  _rssi_bursts(0), _rssi_spread_sum(0), _rssi_degenerate(0),
	  _cad(*this), _probe_interval_s(0), _cad_last_decay_ms(0),
	  _rx_duty_cycle_enabled(IS_ENABLED(CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE)),
	  _rx_boost_enabled(kLoRaRadioOps.set_rx_boost != nullptr),
	  _dc_last_rx_us(0), _dc_last_sleep_us(0),
	  _agc_rx_count_shadow(0), _agc_last_activity_ms(0),
	  _agc_rssi_last(0), _agc_rssi_frozen(0),
	  _rssi_reads_ok(0), _rssi_reads_busy(0), _rssi_bursts_abandoned(0),
	  _rssi_dc_blocked(0),
	  _silence_last_report_ms(0),
	  _image_cal_last_temp_c(INT16_MIN),
	  _image_cal_last_ms(0), _image_cal_wait_ms(0),
	  _image_cal_started(false), _image_cal_confirming(false),
	  _last_tx_start_ms(0),
	  _config_cached(false),
	  _has_radio_override(false),
	  _override_freq(0), _override_bw(0),
	  _override_sf(0), _override_cr(0),
	  _rx_cb(nullptr), _rx_cb_user_data(nullptr),
	  _tx_done_cb(nullptr), _tx_done_cb_user_data(nullptr),
	  _tx_thread_running(false), _tx_len(0),
	  _packets_recv(0), _packets_sent(0), _packets_recv_errors(0)
{
	k_poll_signal_init(&_tx_signal);
	k_sem_init(&_tx_start_sem, 0, 1);
	memset(_rx_ring, 0, sizeof(_rx_ring));
}

/* ── TX wait thread ──────────────────────────────────────────── */

/* How long to wait for TX_DONE: 2 x airtime + 1 s, never below TX_TIMEOUT_MS.
 * A flat 5 s is shorter than a long packet on slow presets (255 B at
 * SF12/BW62.5 is 28.6 s). lora_airtime() is pure math on the cached config. */
uint32_t LoRaRadio::txWaitBudgetMs() const
{
	/* _tx_len is set by startSendRaw() before the handoff; 0 is defensive. */
	uint32_t air = lora_airtime(_dev, _tx_len ? _tx_len : MAX_TRANS_UNIT);

	/* Every driver implements .airtime; 0 means a degenerate modem config. */
	if (air == 0) {
		return TX_TIMEOUT_MS;
	}
	/* Cap the doubling before adding, so a pathological airtime cannot wrap
	 * the 32-bit budget on its way into K_MSEC(). */
	if (air > (UINT32_MAX - 1000U) / 2U) {
		return UINT32_MAX - 1000U;
	}
	return MAX(TX_TIMEOUT_MS, 2U * air + 1000U);
}

void LoRaRadio::txWaitThreadFn(void *p1, void *p2, void *p3)
{
	LoRaRadio *self = static_cast<LoRaRadio *>(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("TX wait thread started");

	for (;;) {
		k_sem_take(&self->_tx_start_sem, K_FOREVER);

		/* startSendRaw() took one sleep lock for the transmit it just
		 * handed over; it is released when this iteration ends, on
		 * whichever of the paths below it takes. */
		struct TxSleepRelease {
			~TxSleepRelease() { zc_pm_unblock_sleep(); }
		} tx_sleep_release;

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
			/* A negative result is the driver reporting a lost transmit
			 * (e.g. the SX126x chip Tx timeout), not a completed one. */
			if (result < 0) {
				LOG_ERR("TX wait: driver reported failure (%d) — packet lost",
					result);
			} else {
				LOG_DBG("TX wait: signal already raised (result=%d)", result);
			}
			k_poll_signal_reset(&self->_tx_signal);
			/* Latch before the RX re-arm (a full SPI reconfigure) so no
			 * dispatcher pass sees a finished TX as pending; the _tx_active
			 * CAS in startSendRaw() makes publishing this early safe. */
			if (result >= 0) {
				atomic_set(&self->_tx_complete, 1);
			}
			self->_board->onAfterTransmit();
			self->startReceive();
			atomic_set(&self->_tx_active, 0);
			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
			continue;
		}

		uint32_t budget_ms = self->txWaitBudgetMs();

		int ret = k_poll(events, 1, K_MSEC(budget_ms));
		if (ret == -EAGAIN) {
			LOG_ERR("TX wait: TIMEOUT after %u ms (len=%u) — packet lost",
				budget_ms, (unsigned)self->_tx_len);
			self->_board->onAfterTransmit();
			self->startReceive();
			atomic_set(&self->_tx_active, 0);
			if (self->_tx_done_cb) {
				self->_tx_done_cb(self->_tx_done_cb_user_data);
			}
			continue;
		}

		if (ret == 0 && events[0].state == K_POLL_STATE_SIGNALED) {
			/* Same verdict rule as the already-raised path above. */
			int sig_result = 0;
			unsigned int sig_state = 0;

			k_poll_signal_check(&self->_tx_signal, &sig_state,
					    &sig_result);
			k_poll_signal_reset(&self->_tx_signal);
			/* Latched before the RX re-arm, as above. */
			if (sig_result >= 0) {
				atomic_set(&self->_tx_complete, 1);
			}
			self->_board->onAfterTransmit();
			self->startReceive();
			atomic_set(&self->_tx_active, 0);
			if (sig_result < 0) {
				LOG_ERR("TX failed: driver reported %d — packet lost",
					sig_result);
			} else {
				LOG_INF("TX complete, RX restarted");
			}

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

/* One radio per build, so one stack. */
K_THREAD_STACK_DEFINE(lora_tx_wait_stack, TX_WAIT_THREAD_STACK_SIZE);

void LoRaRadio::startTxThread()
{
	if (_tx_thread_running) {
		return;
	}
	k_thread_create(&_tx_wait_thread, lora_tx_wait_stack,
			K_THREAD_STACK_SIZEOF(lora_tx_wait_stack),
			txWaitThreadFn, this, NULL, NULL,
			TX_WAIT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&_tx_wait_thread, "lora_tx_wait");
	_tx_thread_running = true;
}

/* ── RX callback (static, ISR-safe) ──────────────────────────────────── */

void LoRaRadio::rxCallbackStatic(const struct device *dev, uint8_t *data,
				     uint16_t size, int16_t rssi, int8_t snr,
				     void *user_data)
{
	LoRaRadio *self = static_cast<LoRaRadio *>(user_data);

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

	/* Activity LED: after the error return, so it blinks on valid packets only. */
	self->_board->onPacketReceived();

	if (self->_rx_cb) {
		self->_rx_cb(self->_rx_cb_user_data);
	}
}

/* ── Config helpers ───────────────────────────────────────────────────── */

void LoRaRadio::buildModemConfig(struct lora_modem_config &cfg, bool tx)
{
	memset(&cfg, 0, sizeof(cfg));
	/* Override wins for freq/bw/sf/cr (tempradio).  Power, preamble, and
	 * other fields still come from _prefs. */
	float freq_mhz = _has_radio_override ? _override_freq : _prefs->freq;
	float bw_khz = _has_radio_override ? _override_bw : _prefs->bw;
	uint8_t sf = _has_radio_override ? _override_sf : _prefs->sf;
	uint8_t cr = _has_radio_override ? _override_cr : _prefs->cr;
	cfg.frequency = (uint32_t)(freq_mhz * 1000000.0f);
	cfg.bandwidth = bw_khz_to_enum((uint16_t)bw_khz);
	cfg.datarate = (enum lora_datarate)sf;
	cfg.coding_rate = cr_to_enum(cr);
	cfg.preamble_len = preambleLengthForSF(sf);
	cfg.tx_power = (int8_t)_prefs->tx_power_dbm;
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

uint32_t LoRaRadio::getActiveFrequencyHz() const
{
	float freq_mhz = _has_radio_override ? _override_freq : _prefs->freq;

	return (uint32_t)(freq_mhz * 1000000.0f + 0.5f);
}

uint16_t LoRaRadio::getActiveBandwidthKHzX10() const
{
	float bw_khz = _has_radio_override ? _override_bw : _prefs->bw;

	return (uint16_t)(bw_khz * 10.0f + 0.5f);
}

uint8_t LoRaRadio::getActiveSpreadingFactor() const
{
	return _has_radio_override ? _override_sf : _prefs->sf;
}

uint8_t LoRaRadio::getActiveCodingRate() const
{
	return _has_radio_override ? _override_cr : _prefs->cr;
}

uint16_t LoRaRadio::getActivePreambleLength() const
{
	return preambleLengthForSF(getActiveSpreadingFactor());
}

uint8_t LoRaRadio::getActiveSyncWord() const
{
	/* buildModemConfig() currently sets public_network=false, which maps
	 * Zephyr's LoRa API to the Semtech private sync word. */
	return 0x12;
}

int8_t LoRaRadio::getConfiguredTxPower() const
{
	int power = _prefs->tx_power_dbm;

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

void LoRaRadio::configure(bool tx)
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
	if (!_ops.loramac_node && _config_cached && onlyDirectionDiffers(cfg, _last_cfg)) {
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

bool LoRaRadio::hwConfigure(const struct lora_modem_config &cfg)
{
	int ret = lora_config(_dev, &cfg);

	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
		return false;
	}
	return true;
}

void LoRaRadio::configureRx() { configure(false); }
void LoRaRadio::configureTx() { configure(true); }

/* ── Lifecycle ────────────────────────────────────────────────────────── */

void LoRaRadio::begin()
{
	if (_prefs == nullptr) {
		LOG_ERR("radio begin() without setPrefs()");
		return;
	}

	startTxThread();

	if (!device_is_ready(_dev)) {
		LOG_ERR("LoRa device not ready");
		return;
	}


	startReceive();

	/* The driver's RX-boost flag starts from DTS; push ours so the
	 * hardware matches _rx_boost_enabled before prefs are applied. */
	hwSetRxBoost(_rx_boost_enabled);

	uint32_t freq = (uint32_t)(_prefs->freq * 1000000.0f);
	uint8_t sf = _prefs->sf;
	uint16_t bw_khz = (uint16_t)(_prefs->bw);
	uint8_t cr = _prefs->cr;
	int8_t tx_pwr = (int8_t)_prefs->tx_power_dbm;

	LOG_INF("radio started: freq=%u bw=%u sf=%u cr=%u pwr=%d",
		freq, bw_khz, sf, cr, tx_pwr);

	if (_ops.post_begin) {
		_ops.post_begin(_dev);
	}
}

void LoRaRadio::reconfigure()
{
	hwCancelReceive();
	atomic_set(&_in_recv_mode, 0);
	_config_cached = false;  /* Force full reconfigure */
	/* CAD probe statistics are only valid for one freq/SF/BW config. */
	resetCadStats();
	startReceive();

	uint32_t freq = (uint32_t)(_prefs->freq * 1000000.0f);
	uint8_t sf = _prefs->sf;
	uint16_t bw_khz = (uint16_t)(_prefs->bw);
	uint8_t cr = _prefs->cr;
	int8_t tx_pwr = (int8_t)_prefs->tx_power_dbm;

	LOG_INF("radio reconfigured: freq=%u bw=%u sf=%u cr=%u pwr=%d",
		freq, bw_khz, sf, cr, tx_pwr);
}

void LoRaRadio::setRadioOverride(float freq, float bw, uint8_t sf, uint8_t cr,
				     bool visiting_new_preset)
{
	_override_freq = freq;
	_override_bw = bw;
	_override_sf = sf;
	_override_cr = cr;
	_has_radio_override = true;
	reconfigure();

	/* The base detPeak is per-SF/per-BW, so a new preset starts at its own
	 * base; a freeze keeps the offset already running. */
	_cad.beginVisit(visiting_new_preset);
}

void LoRaRadio::clearRadioOverride()
{
	if (!_has_radio_override) {
		return;
	}
	_has_radio_override = false;
	reconfigure();
	_cad.endVisit();
}

void LoRaRadio::startReceive()
{
	configureRx();

	int ret;

	if (_rx_duty_cycle_enabled) {
		/* Duty-cycle window sizing (SX1261/2 DS §13.1.7, AN1200.36; LLD 03 §13):
		 *   catch:    sleep + wake <= (P - 2D - 1) * Tsym
		 *   complete: 2R + S >= (P + 14) * Tsym
		 *   floor:    R >= (D + 1) * Tsym
		 * No viable sleep budget -> continuous RX. */
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

			/* Derate the budget by DC_MARGIN_PCT: the RC sleep timer
			 * drifts and the wake time "may vary"; an overshoot drops
			 * phase-edge packets. */
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

				ret = lora_recv_duty_cycle_async(_dev,
							   K_USEC(rx_us),
							   K_USEC(sleep_us),
							   rxCallbackStatic, this);
				if (ret == 0) {
					_rx_entry_cyc = k_cycle_get_32();
					atomic_set(&_in_recv_mode, 1);
					return;
				}
				if (ret == -EBUSY) {
					/* A live TX owns the chip (CAD-busy leaves it in
					 * RX, which the driver's fast path re-arms). */
					LOG_DBG("rxduty: busy (TX in progress) — continuous RX");
				} else if (ret != -ENOSYS) {
					LOG_ERR("lora_recv_duty_cycle_async failed: %d", ret);
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

int LoRaRadio::recvRaw(uint8_t *bytes, int sz)
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

bool LoRaRadio::startSendRaw(const uint8_t *bytes, int len)
{
	if (len > (int)sizeof(_tx_buf)) {
		return false;
	}

	/* Defensive gate: callers should defer TX while radio is BUSY. */
	if (!isRadioReady()) {
		return false;
	}

	/* Final gate before leaving RX: the same isReceiving() as the dispatcher. */
	if (isReceiving()) {
		return false;
	}

	/* CAS: the wait thread publishes completion before re-arming RX, so a
	 * new send can arrive while it winds down; refuse and let the dispatcher
	 * re-queue. Before onBeforeTransmit() so a refusal lights no LED. */
	if (!atomic_cas(&_tx_active, 0, 1)) {
		LOG_DBG("startSendRaw: previous transmit still winding down");
		return false;
	}
	/* An uncollected completion belongs to the previous packet. */
	atomic_set(&_tx_complete, 0);

	/* No SoC light sleep until the wait thread concludes this transmit
	 * (LBT CAD + airtime): an ESP32 would otherwise sleep through TX_DONE.
	 * Released on every path. No-op without CONFIG_PM. */
	zc_pm_block_sleep();
	_board->onBeforeTransmit();
	_last_tx_start_ms = k_uptime_get_32();

	/* LBT: stay in RX (no cancel) so the driver's send_async takes its
	 * RX->TX entry and restores RX itself on CAD-busy. */
	configureTx();

	memcpy(_tx_buf, bytes, len);
	/* Published before the _tx_start_sem handoff below so txWaitBudgetMs()
	 * sizes the wait for this packet, not the previous one. */
	_tx_len = (uint16_t)len;
	k_poll_signal_reset(&_tx_signal);

	int ret = hwSendAsync(_tx_buf, (uint32_t)len, &_tx_signal);
	if (ret < 0) {
		if (ret == -EBUSY) {
			/* LBT refused (channel busy): the designed outcome, the
			 * dispatcher re-queues. Not an error. */
			LOG_DBG("hwSendAsync: channel busy (LBT), re-queuing");
		} else {
			LOG_ERR("hwSendAsync failed: %d", ret);
		}
		_board->onAfterTransmit();
		atomic_set(&_tx_active, 0);
		zc_pm_unblock_sleep();
		/* Safe after any failure: on CAD-busy every driver has already
		 * restored RX and recv_async is a no-op. */
		startReceive();
		return false;
	}

	/* TX has actually started — now we're no longer in RX. */
	atomic_set(&_in_recv_mode, 0);

	LOG_DBG("TX started async, len=%d", len);
	k_sem_give(&_tx_start_sem);
	return true;
}

bool LoRaRadio::isSendComplete()
{
	/* One-shot, and it owns _packets_sent, like upstream's
	 * RadioLibWrapper::isSendComplete(): the radio and dispatcher tallies
	 * move together. Use isTxActive() to ask whether a TX is in flight. */
	if (atomic_cas(&_tx_complete, 1, 0)) {
		atomic_inc(&_packets_sent);
		return true;
	}
	return false;
}

void LoRaRadio::onSendFinished()
{
	/* Nothing needed — TX state tracked via _tx_active */
}

bool LoRaRadio::isInRecvMode() const
{
	return atomic_get(&_in_recv_mode) != 0;
}

float LoRaRadio::getLastRSSI() const
{
	return _last_rssi;
}

float LoRaRadio::getLastSNR() const
{
	return _last_snr;
}

int LoRaRadio::hwGetRssiBurst(int16_t *out, int n, uint32_t spacing_us)
{
	if (_ops.rssi_burst) {
		return _ops.rssi_burst(_dev, out, n, spacing_us);
	}
	for (int i = 0; i < n; i++) {
		if (i) {
			k_busy_wait(spacing_us);
		}
		out[i] = hwGetCurrentRSSI();
		if (out[i] == -128) {
			return i;  /* refused partway; caller abandons */
		}
	}
	return n;
}

bool LoRaRadio::isRadioReady()
{
	/* BUSY high means the radio cannot accept SPI commands now
	 * (e.g. duty-cycle sleep phase on SX126x/LR11xx). */
	return !hwIsChipBusy();
}

/* ── Airtime + scoring ────────────────────────────────────────────────── */

uint32_t LoRaRadio::getEstAirtimeFor(int len_bytes)
{
	/* The params the radio is actually running (a tempradio override
	 * included), resolved through the same BW enum as buildModemConfig(). */
	uint8_t sf = getActiveSpreadingFactor();
	float bw = _has_radio_override ? _override_bw : _prefs->bw;
	uint8_t cr_val = getActiveCodingRate();

	uint8_t min_sf = _ops.loramac_node ? 6 : 5;
	if (sf < min_sf) sf = min_sf;
	if (sf > 12) sf = 12;
	if (bw < 7.0f) bw = 125.0f;
	bw = bandwidth_to_hz(bw_khz_to_enum((uint16_t)bw)) / 1000.0f;
	if (cr_val < 5) cr_val = 5;
	if (cr_val > 8) cr_val = 8;

	float t_sym = (float)(1 << sf) / (bw * 1000.0f);
	/* SX126x/LR11xx/LR20xx add two synchronization symbols at SF5/6
	 * and carry eight more header bits there.  SX127x uses the older
	 * format.  See Semtech's lr11xx_radio_get_lora_time_on_air_numerator.
	 * Treating SF5 as SF6 charged 397 ms for a 126-byte SF5/BW62.5 CR4/8
	 * packet whose Semtech airtime is 237 ms (confirmed by TX timing). */
	float fine_sync = (!_ops.loramac_node && sf <= 6) ? 1.0f : 0.0f;
	float t_preamble = (preambleLengthForSF(sf) + 4.25f + 2.0f * fine_sync) * t_sym;

	/* LDRO threshold must track the SX126x driver's should_enable_ldro()
	 * exactly (symbol time > 16.38 ms) so this estimate's DE matches the
	 * hardware's DE on every SF/BW pair.  The old `sf >= 11` was only
	 * correct at BW 125 kHz and diverged on every other bandwidth. */
	float de = (t_sym > 0.01638f) ? 1.0f : 0.0f;
	float num = 8.0f * len_bytes - 4.0f * sf + 28.0f + 16.0f - 8.0f * fine_sync;
	float den = 4.0f * (sf - 2.0f * de);
	if (den < 1.0f) den = 4.0f;
	float n_payload = 8.0f + fmaxf(ceilf(num / den) * cr_val, 0.0f);

	float t_payload = n_payload * t_sym;
	return (uint32_t)((t_preamble + t_payload) * 1000.0f);
}

float LoRaRadio::packetScore(float snr, int packet_len)
{
	int sf = _prefs->sf;
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

int LoRaRadio::getNoiseFloor() const
{
	return _floor_est.floor();
}

void LoRaRadio::triggerNoiseFloorCalibrate(int threshold)
{
	_calibration_threshold = threshold;

	/* Own the sampling cadence rather than inheriting the caller's.  Early
	 * calls are a no-op, so this is safe to invoke from any wake. */
	int64_t now = k_uptime_get();

	/* "Fresh" means sampled in THIS pass: cadMaintenance() runs right
	 * after and treats the verdict as current. */
	_sample_fresh = false;

	if (_noise_floor_next_ms != 0 && now < _noise_floor_next_ms) {
		return;
	}

	/* Due. A bail-out below is a blocked attempt: retry soon, a bounded
	 * number of times, so the wake schedule never spins on "due now". */
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

	/* Chip BUSY (duty-cycle sleep). Counted (d in `get cad`): the only
	 * record of how often the sampler is turned away. */
	if (!isRadioReady()) {
		_rssi_dc_blocked++;
		return;
	}

	/* Skip if mid-receive — don't want signal energy in the floor. */
	if (isReceiving()) {
		return;
	}

	/* GetRssiInst needs settling after RX entry (DS Table 13-82). Only
	 * host-driven entries are stamped; duty-cycle wakes are chip-internal
	 * and the median absorbs the rare unsettled read. */
	uint16_t bw_khz = (uint16_t)(getActiveBandwidthKHzX10() / 10);
	uint32_t since_rx_us =
		k_cyc_to_us_floor32(k_cycle_get_32() - _rx_entry_cyc);

	if (since_rx_us < rssi_settle_delay_us(bw_khz)) {
		return;
	}

	/* Median of N reads, spaced by the RSSI averaging window so they are
	 * independent. It rejects glitches and bad SPI reads; interference is
	 * kept out by isReceiving() and the estimator's outlier gate (LLD 03 §13). */
	uint32_t window_us = rssi_avg_window_us(bw_khz);
	int16_t samples[NOISE_FLOOR_SAMPLES_PER_TICK];
	int got = hwGetRssiBurst(samples, NOISE_FLOOR_SAMPLES_PER_TICK, window_us);

	if (got < 0) {
		/* A preamble/header landed mid-burst: the samples measured a
		 * signal. Abandoned, not a read fault. The clear _sample_fresh also
		 * keeps the CAD probe off the air on the LR families. */
		_rssi_bursts_abandoned++;
		return;
	}

	_rssi_reads_ok += (uint32_t)got;
	if (got < NOISE_FLOOR_SAMPLES_PER_TICK) {
		/* Partial burst (chip busy / contended read): counted, retried soon. */
		_rssi_reads_busy++;
		_rssi_bursts_abandoned++;
		return;
	}

	/* A full sample landed: next one is a full interval away. */
	_noise_floor_next_ms = now + _measure_interval_ms;
	_noise_floor_retries = 0;

	int16_t rssi = sortAndMedian(samples, NOISE_FLOOR_SAMPLES_PER_TICK);

	/* Burst quality for `get cad` (sp:). Sorted, so max - min is the spread.
	 * A high zero-spread share is only a fault together with a mean of 0.0. */
	_rssi_bursts++;
	_rssi_spread_sum += (uint32_t)(samples[NOISE_FLOOR_SAMPLES_PER_TICK - 1] -
				       samples[0]);
	if (samples[NOISE_FLOOR_SAMPLES_PER_TICK - 1] == samples[0]) {
		_rssi_degenerate++;
	}
	/* Rescale every counter together so the printed ratios stay right
	 * and the count stays four digits. */
	if (_rssi_bursts >= RSSI_BURST_STATS_CAP) {
		_rssi_bursts >>= 1;
		_rssi_spread_sum >>= 1;
		_rssi_degenerate >>= 1;
		_rssi_reads_ok >>= 1;
		_rssi_reads_busy >>= 1;
		_rssi_bursts_abandoned >>= 1;
		_rssi_dc_blocked >>= 1;
	}

	/* Publish for cadMaintenance(): the CAD probe rides on this sample
	 * (one wake per interval, median-of-N ground truth). Judged against
	 * the floor before this sample is folded in. */
	_sample_rssi = rssi;
	_sample_channel_quiet = _floor_est.isQuiet(rssi);
	_sample_fresh = true;

	/* Stuck-AGC evidence for agcIdleMaintenance(): a live front end
	 * dithers; a desensitised one reads the same value forever. */
	if (rssi == _agc_rssi_last) {
		if (_agc_rssi_frozen < 0xFF) {
			_agc_rssi_frozen++;
		}
	} else {
		_agc_rssi_last = rssi;
		_agc_rssi_frozen = 0;
	}

	/* The lower clamp tracks the active bandwidth (thermal noise is
	 * 10*log10(BW)); a fixed rail pins narrow presets high. */
	int16_t floor_min = noise_floor_min_dbm(getActiveBandwidthKHzX10() / 10);

	switch (_floor_est.update(rssi, floor_min)) {
	case NoiseFloorEstimator::SEEDED:
		LOG_DBG("noise_floor_cal: seed=%d", _floor_est.floor());
		break;
	case NoiseFloorEstimator::UPDATED:
		LOG_DBG("noise_floor_cal: rssi=%d, floor=%d, tick=%u",
			rssi, _floor_est.floor(), _floor_est.tick() - 1);
		break;
	case NoiseFloorEstimator::REJECTED:
		break;
	}
}

bool LoRaRadio::isReceiving()
{
	if (!atomic_get(&_in_recv_mode) || atomic_get(&_tx_active)) {
		return false;
	}
	/* Driver latch + IRQ read over the whole payload. Foreign preambles are
	 * released by the driver's SF-aware grace and header deadline, the only
	 * cases in which the poll clears bits. */
	if (hwIsReceiving()) {
		return true;
	}
	return isChannelActive();
}

/* ── Receiver hygiene ─────────────────────────────────────────────────
 *
 * Two unrelated faults, neither handled on the packet path (rationale and
 * measurements: LLD 03 §13):
 * 1. Stuck AGC (SX126x): reset only after long silence AND a frozen
 *    noise-floor reading; the reset costs an RX window, so silence alone
 *    is not proof. The noise floor is deliberately not reset.
 * 2. Calibration drift (LR families): recalibrate once the board
 *    temperature has moved IMAGE_CAL_TEMP_DELTA_C, confirmed twice,
 *    outside the post-TX window. */
/* Ten minutes: at 60 s it fired 4-8 times per quarter hour on a healthy
 * node, and a genuinely deaf receiver stays deaf. */
#define AGC_IDLE_RESET_MS        600000U

/* Consecutive identical noise-floor readings before silence is believed.  At
 * the default 15 s sampler interval this is two minutes of a frozen front end. */
#define AGC_STUCK_RSSI_SAMPLES   8U

/* How often to report an ongoing RX silence.  Diagnostic only. */
#define SILENCE_REPORT_MS        120000U

/* Half the datasheet's 10 C (SX126x DS §9.2.1, LR11xx UM), so drift is
 * corrected before the calibration is stale. */
#define IMAGE_CAL_TEMP_DELTA_C   5

/* Temperature is read hourly: it cannot move IMAGE_CAL_TEMP_DELTA_C in
 * minutes, and each chip command risks a duty-cycle sleep collision. */
#define IMAGE_CAL_POLL_MS        3600000U   /* 1 h between routine reads */
#define IMAGE_CAL_CONFIRM_MS       15000U   /* re-read before acting on a delta */
#define IMAGE_CAL_TX_QUIET_MS      60000U   /* let PA self-heating decay first */

/* Two unrelated jobs that share one precondition (chip idle) and one
 * cadence; kept in separate functions. */
void LoRaRadio::radioMaintenance()
{
	/* Never mid-TX or mid-RX: both jobs warm-sleep the chip. The RX counters
	 * cannot see a packet landing now; isReceiving() can. */
	if (atomic_get(&_tx_active) || isReceiving()) {
		return;
	}

	uint32_t now = (uint32_t)k_uptime_get_32();

	/* Silence telemetry on every family (diagnostic only). */
	uint32_t rx_now = (uint32_t)atomic_get(&_packets_recv) +
			  (uint32_t)atomic_get(&_packets_recv_errors);

	if (rx_now != _agc_rx_count_shadow || _agc_last_activity_ms == 0) {
		_silence_last_report_ms = 0;   /* traffic — reset the reporter */
	} else {
		uint32_t silent_ms = now - _agc_last_activity_ms;

		if (silent_ms >= SILENCE_REPORT_MS &&
		    (_silence_last_report_ms == 0 ||
		     (now - _silence_last_report_ms) >= SILENCE_REPORT_MS)) {
			_silence_last_report_ms = now ? now : 1;
			LOG_INF("silence: %u s no RX | in_rx=%d dc=%d rssi_ok=%u rssi_busy=%u abandoned=%u dc_blocked=%u floor=%d",
				(unsigned)(silent_ms / 1000U),
				(int)atomic_get(&_in_recv_mode),
				(int)_rx_duty_cycle_enabled,
				(unsigned)_rssi_reads_ok,
				(unsigned)_rssi_reads_busy,
				(unsigned)_rssi_bursts_abandoned,
				(unsigned)_rssi_dc_blocked,
				_floor_est.floor());
		}
	}

	agcIdleMaintenance(now);
	imageCalMaintenance(now);
}

/* Receiver watchdog: a stuck AGC shows up as total silence. */
void LoRaRadio::agcIdleMaintenance(uint32_t now)
{
	if (!hwNeedsAgcReset()) {
		return;
	}

	/* Errored frames count as proof of life: RF reached the demodulator.
	 * Counting only good packets would reset a node that is out of range. */
	uint32_t rx_total = (uint32_t)atomic_get(&_packets_recv) +
			    (uint32_t)atomic_get(&_packets_recv_errors);

	if (rx_total != _agc_rx_count_shadow || _agc_last_activity_ms == 0) {
		_agc_rx_count_shadow = rx_total;
		_agc_last_activity_ms = now ? now : 1;
	} else if ((now - _agc_last_activity_ms) >= AGC_IDLE_RESET_MS &&
		   _agc_rssi_frozen >= AGC_STUCK_RSSI_SAMPLES) {
		LOG_INF("agc: %u ms silent AND %u frozen floor samples at %d dBm — resetting AGC",
			(unsigned)(now - _agc_last_activity_ms),
			(unsigned)_agc_rssi_frozen, (int)_agc_rssi_last);
		hwResetAgc();
		/* hwResetAgc() leaves the chip out of RX by contract, so this is
		 * a genuine re-entry and re-arms the duty cycle if one is set. */
		startReceive();
		_agc_last_activity_ms = now ? now : 1;
	}
}

/* Front-end image calibration against temperature drift. */
void LoRaRadio::imageCalMaintenance(uint32_t now)
{
	if (!hwHasDriftRecal()) {
		return;
	}

	/* Own slow cadence, see IMAGE_CAL_POLL_MS. */
	if (_image_cal_started && (now - _image_cal_last_ms) < _image_cal_wait_ms) {
		return;
	}

	/* Wait out our own PA self-heating. Returns without stamping, so the
	 * retry is the next pass, not the next poll window. */
	if (_last_tx_start_ms != 0 &&
	    (now - _last_tx_start_ms) < IMAGE_CAL_TX_QUIET_MS) {
		return;
	}

	/* Board (MCU) temperature, never the radio's: only the delta matters,
	 * and a radio read during duty-cycle sleep wedged the LR1110 (LLD 03 §13). */
	float board_temp = _board ? _board->getMCUTemperature() : NAN;

	if (isnan(board_temp)) {
		/* No board temperature source: drift handling does not run. */
		return;
	}

	int16_t temp_c = (int16_t)lroundf(board_temp);

	_image_cal_started = true;
	_image_cal_last_ms = now;
	_image_cal_wait_ms = IMAGE_CAL_POLL_MS;

	if (_image_cal_last_temp_c == INT16_MIN) {
		_image_cal_last_temp_c = temp_c;   /* first reading is the baseline */
		return;
	}

	int delta = (int)temp_c - (int)_image_cal_last_temp_c;

	if (delta < 0) {
		delta = -delta;
	}
	if (delta < IMAGE_CAL_TEMP_DELTA_C) {
		_image_cal_confirming = false;
		return;
	}

	/* Measure twice: hwRecalibrate() costs an RX window. */
	if (!_image_cal_confirming) {
		_image_cal_confirming = true;
		_image_cal_wait_ms = IMAGE_CAL_CONFIRM_MS;
		LOG_DBG("imagecal: chip temp delta %d C — confirming before recalibrating",
			delta);
		return;
	}

	_image_cal_confirming = false;
	LOG_INF("imagecal: chip temp moved %d C (%d -> %d) — recalibrating",
		delta, (int)_image_cal_last_temp_c, (int)temp_c);
	hwRecalibrate();
	startReceive();
	_image_cal_last_temp_c = temp_c;
}

void LoRaRadio::recoverRxState()
{
	/* Dispatcher CAD-timeout recovery (isReceiving() pinned > 4 s). Walk
	 * the chip through REST: a bare startReceive() fails the driver's
	 * REST->RX CAS. The RX restart clears the IRQs and the latch. */
	hwCancelReceive();
	atomic_set(&_in_recv_mode, 0);
	_config_cached = false;
	startReceive();
}

bool LoRaRadio::isChannelActive(int threshold)
{
	if (threshold == 0) {
		threshold = _calibration_threshold;
	}
	if (threshold == 0) {
		return false;
	}
	int16_t rssi = hwGetCurrentRSSI();
	return rssi > (_floor_est.floor() + threshold);
}

/* ── Adaptive CAD (LBT detPeak calibration) ───────────────────────────── */

void LoRaRadio::setCadParams(bool auto_enabled, int8_t offset,
				 uint16_t probe_interval_s, uint8_t busycap_pct,
				 uint8_t stored_base)
{
	_cad.configure(auto_enabled, offset, busycap_pct, stored_base);
	_probe_interval_s = probe_interval_s;

	/* One interval for the one periodic measurement: the CAD probe rides on
	 * the floor sample. 0 = probing off, but the floor still needs sampling. */
	_measure_interval_ms = probe_interval_s
			       ? (uint32_t)probe_interval_s * 1000U
			       : (uint32_t)CONFIG_ZEPHCORE_NOISE_FLOOR_INTERVAL_MS;

	LOG_INF("cad: auto=%d offset=%d base=%u measure_interval=%ums busycap=%u%%",
		(int)auto_enabled, (int)_cad.offset(), (unsigned)hwCadBasePeak(),
		(unsigned)_measure_interval_ms, (unsigned)busycap_pct);
}

uint8_t LoRaRadio::cadBasePeak()
{
	return hwCadBasePeak();
}

void LoRaRadio::cadMaintenance()
{
	if (_probe_interval_s == 0) {
		return;
	}

	/* Nothing to learn on a preset the node is only visiting, and any move
	 * would be persisted by onCadOffsetChanged(). A sample from the visit is
	 * stale after the revert, so drop it. */
	if (_has_radio_override) {
		_sample_fresh = false;
		return;
	}

	int64_t now = k_uptime_get();

	/* Periodic decay keeps the stats fresh (and counters bounded). */
	if (_cad_last_decay_ms == 0) {
		_cad_last_decay_ms = now;
	} else if (now - _cad_last_decay_ms > (int64_t)CAD_STATS_DECAY_MS) {
		_cad.decayStats();
		_cad_last_decay_ms = now;
	}

	/* A CAD_RX probe from an earlier pass: past the chip's own deadline a
	 * terminal interrupt (packet or cadTimeout) is guaranteed, so the
	 * answer is read once, with no chip access. */
	if (_cad.hasPending() && now >= _cad.pendingDeadline()) {
		_cad.resolvePending(hwCadRxOutcome());
		_cad.adapt();
	}

	/* The probe rides on the floor sampler: a fresh sample means this pass
	 * already proved idle RX, no TX, no packet, out of duty-cycle sleep, and
	 * gave a median-of-N quiet verdict. No sample, no probe. */
	if (!_sample_fresh) {
		return;
	}
	_sample_fresh = false;

	/* One probe in flight: a second CAD would abort the first one's RX. */
	if (_cad.hasPending()) {
		return;
	}

	/* No CAD_RX ground truth, no probing: an unresolvable busy reads as
	 * FP 0 everywhere and walks the staircase to the sensitive rail. Only
	 * radios without hardware CAD (SX127x) report 0 here. */
	if (hwCadRxTimeoutMs() == 0) {
		return;
	}

	if (!_sample_channel_quiet) {
		return;
	}

	/* The sampler's isReceiving() is a burst old; a calibration probe must
	 * never cost a reception, and skipping one is free. */
	if (isReceiving()) {
		return;
	}

	int8_t level = _cad.pickProbeLevel();
	int ret = hwCadProbe(level);
	/* hwCadProbe() blocks for the whole CAD: take the time afterwards. */
	int64_t probe_done_ms = k_uptime_get();

	/* free (0), busy on CAD_ONLY (1), error: the chip is in standby, so
	 * re-enter RX. busy (2): the chip is already in RX on the signal it
	 * found; re-entering would destroy the reception being measured. */
	if (ret != 2) {
		atomic_set(&_in_recv_mode, 0);
		startReceive();
	}

	if (ret < 0) {
		if (ret != -ENOSYS) {
			LOG_WRN("cad: probe failed (%d)", ret);
		}
		return;
	}

	_cad.recordProbe(level, ret > 0);

	if (ret == 2) {
		/* The chip's cadTimeout starts when the blocking probe returns;
		 * +100 ms is interrupt-to-work-queue latency. */
		_cad.setPending(level,
				probe_done_ms + (int64_t)hwCadRxTimeoutMs() + 100);
		return;
	}

	_cad.adapt();
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
uint32_t LoRaRadio::msUntilNextMaintenance()
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

	/* The CAD probe has no deadline of its own (it rides on the floor
	 * sample). Only a pending CAD_RX verdict adds one wake. */
	if (_cad.hasPending()) {
		next = mesh::maintenanceSooner(
			next, clampDeadline(_cad.pendingDeadline() - now));
	}

	/* Stats decay. _cad_last_decay_ms == 0 means the first call latches it
	 * rather than decaying, so treat that as due now. */
	if (_cad_last_decay_ms == 0) {
		return 0;
	}
	return mesh::maintenanceSooner(
		next, clampDeadline(_cad_last_decay_ms + (int64_t)CAD_STATS_DECAY_MS - now));
}

int LoRaRadio::formatCadStatus(char *buf, int cap)
{
	uint8_t base = hwCadBasePeak();
	int n = 0;

	if (base == 0) {
		return snprintf(buf, cap, "cad n/a");
	}

	/* Terse: remote replies are capped at ~160 B. Fields (docs/ADAPTIVE_CAD.md):
	 * a auto on/off/tmp (visiting a preset), o offset, pk peak (b base / 4
	 * symbols), sp mean burst spread in dB / zero-spread share, bc busy cap.
	 * Levels: *+1(22) 22p 18b 16f 2t 72% = rung(peak) probes busy fp tp fp%. */
	unsigned spread_mean10 = _rssi_bursts
		? (unsigned)((_rssi_spread_sum * 10U + _rssi_bursts / 2U) /
			     _rssi_bursts)
		: 0;
	unsigned degen_pct = _rssi_bursts
		? (unsigned)((_rssi_degenerate * 100U + _rssi_bursts / 2U) /
			     _rssi_bursts)
		: 0;

	/* The burst counters fit only the local console buffer; a remote reader
	 * still gets the mean and the share. */
	bool room_for_count = (cap >= 200);

	const int eff_off = _cad.effectiveOffset();

	n += snprintf(buf + n, cap > n ? cap - n : 0,
		      "a:%s o:%d pk:%d(b%u/4s) sp:%u.%u/%u%%",
		      _cad.visiting() ? "tmp" : (_cad.autoEnabled() ? "on" : "off"),
		      eff_off, (int)base + eff_off, base,
		      spread_mean10 / 10U, spread_mean10 % 10U, degen_pct);
	if (room_for_count) {
		/* (bursts r<ok reads>/b<busy reads>/a<abandoned>/d<dc-blocked>) */
		n += snprintf(buf + n, cap > n ? cap - n : 0,
			      "(%u r%u/b%u/a%u/d%u)",
			      (unsigned)_rssi_bursts,
			      (unsigned)_rssi_reads_ok,
			      (unsigned)_rssi_reads_busy,
			      (unsigned)_rssi_bursts_abandoned,
			      (unsigned)_rssi_dc_blocked);
	}
	n += snprintf(buf + n, cap > n ? cap - n : 0, " bc:%u%%",
		      (unsigned)_cad.busycapPct());
	/* No CAD_RX ground truth on this radio: probing is off. */
	if (hwCadRxTimeoutMs() == 0) {
		n += snprintf(buf + n, cap > n ? cap - n : 0, " probe:n/a");
	}

	/* The 3 rungs around the operating level, inside the effective window
	 * (past the driver clamp several rungs would repeat one peak). */
	const int lmin = _cad.levelMinEff();
	const int lmax = _cad.levelMaxEff();
	int cur = eff_off;
	if (cur < lmin) cur = lmin;
	if (cur > lmax) cur = lmax;
	int lo = cur - 1, hi = cur + 1;
	if (lo < lmin) { lo = lmin; hi = lo + 2; }
	if (hi > lmax) { hi = lmax; lo = hi - 2; }
	if (lo < CAD_LEVEL_MIN) lo = CAD_LEVEL_MIN;
	if (hi > CAD_LEVEL_MAX) hi = CAD_LEVEL_MAX;

	for (int lvl = lo; lvl <= hi; lvl++) {
		const CadController::LevelStats &s = _cad.stats(lvl);

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

void LoRaRadio::enableRxDutyCycle(bool enable)
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

bool LoRaRadio::setRxBoost(bool enable)
{
	if (!_ops.set_rx_boost) {
		return false;
	}
	_rx_boost_enabled = enable;
	LOG_INF("RX boost %s (+3dB sensitivity, +2mA)",
		enable ? "enabled" : "disabled");
	if (atomic_get(&_in_recv_mode)) {
		hwSetRxBoost(enable);
	}
	return true;
}

bool LoRaRadio::setFemRxEnable(bool enable)
{
	if (!_ops.set_fem_rx || !_ops.set_fem_rx(_dev, enable)) {
		return false;
	}
	LOG_INF("FEM RX gain %s", enable ? "enabled" : "disabled");
	return true;
}

bool LoRaRadio::configSideDetectors(const uint8_t *sfs, uint8_t num)
{
	if (!_ops.side_detectors) {
		return false;
	}

	/* The driver validates against the configured SF/BW; the CLI only
	 * reaches here on a live radio. */
	int ret = _ops.side_detectors(_dev, sfs, num);

	if (ret < 0) {
		LOG_WRN("side detector config rejected: %d", ret);
		return false;
	}
	return true;
}

} /* namespace mesh */
