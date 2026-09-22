/*
 * SPDX-License-Identifier: MIT
 * LoRa radio base class — shared algorithms for all radio adapters.
 */

#include "LoRaRadioBase.h"
#include "radio_common.h"
#include "pm_sleep_guard.h"
#include <mesh/LoRaConfig.h>
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

LoRaRadioBase::LoRaRadioBase(const struct device *lora_dev, MainBoard &board,
			     NodePrefs *prefs)
	: _loramac_node(false),
	  _dev(lora_dev), _prefs(prefs), _board(&board),
	  _in_recv_mode(0), _tx_active(0), _tx_complete(0),
	  _last_rssi(0), _last_snr(0),
	  _rx_head(0), _rx_tail(0),
	  _noise_floor(DEFAULT_NOISE_FLOOR), _calibration_threshold(0), _ema_unguarded(0),
	  _noise_floor_next_ms(0), _noise_floor_retries(0),
	  _measure_interval_ms(CONFIG_ZEPHCORE_NOISE_FLOOR_INTERVAL_MS),
	  _sample_rssi(0), _sample_channel_quiet(false), _sample_fresh(false),
	  _rx_entry_cyc(0),
	  _rssi_bursts(0), _rssi_spread_sum(0), _rssi_degenerate(0),
	  _cad_auto(false), _cad_offset(0), _cad_temp_offset(0), _probe_interval_s(0),
	  _cad_busycap_pct(0), _cad_pending_level(INT8_MIN),
	  _cad_pending_deadline_ms(0),
	  _cad_last_probe_ms(0), _cad_last_decay_ms(0),
	  _cad_probe_rr(0),
	  _rx_duty_cycle_enabled(IS_ENABLED(CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE)),
	  _rx_boost_enabled(true),
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
	memset(_cad_stats, 0, sizeof(_cad_stats));
}

/* ── TX wait thread ──────────────────────────────────────────── */

/* How long to wait for TX_DONE before declaring the transmit lost.
 *
 * TX_TIMEOUT_MS alone is a fixed 5 s, which is shorter than the airtime of a
 * great many legal presets — a 255-byte packet is 28.6 s at SF12/BW62.5 and
 * 10.2 s at SF9/BW31.25 — so the wait would expire mid-transmission and
 * startReceive() would yank the radio out of TX, losing a packet that was
 * transmitting perfectly well.  Scale from the driver's own airtime instead,
 * with the same doubling the drivers' internal sync-send waits use.
 *
 * MAX(), never a bare replacement: on fast presets the scaled value is smaller
 * than 5 s (SF7/BW62.5, 64 bytes: ~1.4 s), and shortening this deadline on the
 * presets every radio in the fleet is running today would be a regression for
 * no gain.  The floor keeps existing behaviour exactly; only slow presets move.
 *
 * lora_airtime() is a pure calculation on the cached modem config, so calling
 * it from this thread costs no SPI and cannot race the radio. */
uint32_t LoRaRadioBase::txWaitBudgetMs() const
{
	/* _tx_len is published by startSendRaw() before it releases
	 * _tx_start_sem, so by the time this thread runs it is always the length
	 * of the transmit in flight — the fallback is defensive only, and uses
	 * the protocol maximum rather than a literal so it tracks MAX_TRANS_UNIT
	 * if the FIFO bound ever moves. */
	uint32_t air = lora_airtime(_dev, _tx_len ? _tx_len : MAX_TRANS_UNIT);

	/* All four drivers behind this class implement .airtime (native sx126x,
	 * lr11xx, lr20xx, loramac-node sx127x), and lora_airtime() dereferences
	 * the op without a NULL check, so there is no missing-op case to handle.
	 * A zero can still come back from a degenerate modem config; fall back
	 * to the flat budget rather than to no wait at all. */
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

void LoRaRadioBase::txWaitThreadFn(void *p1, void *p2, void *p3)
{
	LoRaRadioBase *self = static_cast<LoRaRadioBase *>(p1);
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
			/* The result carries the driver's verdict, and it has to
			 * be honoured: a driver that reports a failed transmit
			 * by raising the signal with a negative result (the
			 * SX126x does exactly this on a chip Tx timeout,
			 * -ETIMEDOUT) was previously counted here as a
			 * successful send.  Latent rather than live — that
			 * SX126x path is unreachable while an RX callback is
			 * registered, which it always is — but it is the reason
			 * the LR11xx and LR20xx timeout handlers deliberately do
			 * NOT raise the signal.  With the result honoured, a
			 * driver reporting failure is now the correct thing to
			 * do on all three. */
			if (result < 0) {
				LOG_ERR("TX wait: driver reported failure (%d) — packet lost",
					result);
			} else {
				LOG_DBG("TX wait: signal already raised (result=%d)", result);
			}
			k_poll_signal_reset(&self->_tx_signal);
			/* Latch the verdict BEFORE the RX re-arm below, not after:
			 * onAfterTransmit() + startReceive() is a full modem
			 * reconfigure over SPI, and any loop() pass that lands
			 * inside it used to see "not complete yet" and abandon a
			 * transmit that had in fact finished.  startSendRaw()'s
			 * _tx_active CAS is what makes publishing the completion
			 * this early safe. */
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
			/* Same rule as the already-raised path above: a negative
			 * result is the driver reporting a lost transmit, not a
			 * completed one. */
			int sig_result = 0;
			unsigned int sig_state = 0;

			k_poll_signal_check(&self->_tx_signal, &sig_state,
					    &sig_result);
			k_poll_signal_reset(&self->_tx_signal);
			/* Latched ahead of the RX re-arm — see the equivalent
			 * comment on the already-raised path above. */
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

	/* Activity LED ("set leds.radio rx|all").  Deliberately below the CRC and
	 * header-error early return above, so the blink means a valid packet
	 * landed rather than that something was heard on the channel.  Cheap and
	 * non-blocking: the board raises a GPIO and arms a one-shot. */
	self->_board->onPacketReceived();

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

void LoRaRadioBase::setRadioOverride(float freq, float bw, uint8_t sf, uint8_t cr,
				     bool visiting_new_preset)
{
	_override_freq = freq;
	_override_bw = bw;
	_override_sf = sf;
	_override_cr = cr;
	_has_radio_override = true;
	reconfigure();

	/* Visit a new preset at its own base detPeak.  The learned offset belongs
	 * to the preset it was measured on and the base table it addresses is
	 * per-SF/per-bandwidth, so carrying it across programs a threshold nobody
	 * measured (SF7->SF12 is 5 counts on SX126x, 16 on LR11xx).  A freeze
	 * changes nothing on air, so it keeps the offset that is already right.
	 *
	 * _cad_offset is left untouched — not snapshotted — so the configured
	 * preset gets its offset back verbatim in clearRadioOverride(), and no
	 * visitor value can reach prefs in between.  Adaptation is suspended for
	 * the duration; see cadMaintenance(). */
	_cad_temp_offset = visiting_new_preset ? 0 : _cad_offset;
	hwCadSetPeakOffset(cadEffectiveOffset());
}

void LoRaRadioBase::clearRadioOverride()
{
	if (!_has_radio_override) {
		return;
	}
	_has_radio_override = false;
	reconfigure();
	hwCadSetPeakOffset(cadEffectiveOffset());
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
					/* A concurrent TX owns the chip. Not the
					 * CAD-busy case: when the LBT branch of
					 * send_async restores RX in-driver it
					 * leaves the chip in RX, and the driver's
					 * idempotent fast-path (patch 0003)
					 * re-arms duty cycle from there — AGC
					 * reset included — rather than refusing.
					 * So -EBUSY here means the radio is
					 * genuinely mid-transmit; the fall-through
					 * to lora_recv_async will fail the same
					 * way and report it. */
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

	/* CAS, not a bare set: the wait thread publishes _tx_complete before it
	 * re-arms RX, so the dispatcher can legitimately collect a completion
	 * and come straight back here while that thread is still inside
	 * startReceive().  A plain set would let this transmit be started and
	 * then have its _tx_active cleared out from under it moments later --
	 * and would leave the driver re-entering RX on top of a live TX.
	 * Refusing instead is correct and cheap: the dispatcher re-queues and
	 * the wind-down finishes in microseconds.  Placed before
	 * onBeforeTransmit() so a refusal does not light the TX LED. */
	if (!atomic_cas(&_tx_active, 0, 1)) {
		LOG_DBG("startSendRaw: previous transmit still winding down");
		return false;
	}
	/* A completion nobody collected belongs to the packet that just went
	 * out, never to this one -- upstream's STATE_IDLE reset in the same
	 * place. */
	atomic_set(&_tx_complete, 0);

	/* No SoC light sleep from here until the wait thread has concluded this
	 * transmit (it releases the lock), or until the failure path below.
	 * Covers the blocking LBT CAD inside hwSendAsync() as well as the
	 * airtime.  With nothing else runnable during a transmit, an ESP32
	 * repeater otherwise sleeps through it and meets TX_DONE as a wake
	 * event rather than an interrupt, and the completion lands late or not
	 * at all (see the EXT1 note in the sx126x HAL).  Costs nothing worth
	 * counting: the PA dominates the current draw while keyed.  A no-op
	 * without CONFIG_PM. */
	zc_pm_block_sleep();
	_board->onBeforeTransmit();
	_last_tx_start_ms = k_uptime_get_32();

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
	/* Published before the _tx_start_sem handoff below so txWaitBudgetMs()
	 * sizes the wait for this packet, not the previous one. */
	_tx_len = (uint16_t)len;
	k_poll_signal_reset(&_tx_signal);

	int ret = hwSendAsync(_tx_buf, (uint32_t)len, &_tx_signal);
	if (ret < 0) {
		if (ret == -EBUSY) {
			/* LBT refused the transmit because the channel is busy
			 * — the designed outcome, not a fault. The dispatcher
			 * re-queues and retries. On a busy site this fires
			 * constantly, and at ERR it buries real faults and
			 * makes a healthy repeater look broken. */
			LOG_DBG("hwSendAsync: channel busy (LBT), re-queuing");
		} else {
			LOG_ERR("hwSendAsync failed: %d", ret);
		}
		_board->onAfterTransmit();
		atomic_set(&_tx_active, 0);
		zc_pm_unblock_sleep();
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
	/* One-shot, and it owns _packets_sent — the same contract as upstream's
	 * RadioLibWrapper::isSendComplete(), which self-clears STATE_INT_READY
	 * and does n_sent++ in the same breath.  Incrementing here rather than
	 * in the wait thread is what keeps the radio's "packets sent" tally and
	 * the dispatcher's flood/direct tallies in lockstep: both advance on
	 * this one call, so a completion the dispatcher never collects (it hit
	 * outbound_expiry first) is missed by both, exactly as upstream misses
	 * it.  They used to be independent counters on independent threads,
	 * which let "Total" and "Flood + Direct" disagree by thousands.
	 *
	 * This is a consuming call.  Anything that wants to know whether a
	 * transmit is in flight must use isTxActive() instead. */
	if (atomic_cas(&_tx_complete, 1, 0)) {
		atomic_inc(&_packets_sent);
		return true;
	}
	return false;
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

int LoRaRadioBase::hwGetRssiBurst(int16_t *out, int n, uint32_t spacing_us)
{
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

bool LoRaRadioBase::isRadioReady()
{
	/* BUSY high means the radio cannot accept SPI commands now
	 * (e.g. duty-cycle sleep phase on SX126x/LR11xx). */
	return !hwIsChipBusy();
}

/* ── Airtime + scoring ────────────────────────────────────────────────── */

uint32_t LoRaRadioBase::getEstAirtimeFor(int len_bytes)
{
	/* Read the params the radio is ACTUALLY running, not the saved prefs:
	 * buildModemConfig() honours _has_radio_override, so a node under
	 * `tempradio` transmits on the override preset while this used to
	 * estimate for the stored one.  Everything downstream of the estimate
	 * drifts with it — reported RX airtime, the TX airtime and duty-cycle
	 * budget that now derive from it, and outbound_expiry.  Upstream cannot
	 * drift this way because it asks the radio (getTimeOnAir()); these
	 * accessors are our equivalent.  Resolve bandwidth through the same
	 * enum as buildModemConfig(): a saved 62 or 62.5 both program 62.5 kHz,
	 * and a rounded setting must not change the airtime calculation. */
	uint8_t sf = getActiveSpreadingFactor();
	float bw = _has_radio_override ? _override_bw
		   : (_prefs ? _prefs->bw : (float)LoRaConfig::BANDWIDTH);
	uint8_t cr_val = getActiveCodingRate();

	uint8_t min_sf = _loramac_node ? 6 : 5;
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
	float fine_sync = (!_loramac_node && sf <= 6) ? 1.0f : 0.0f;
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
	 * (e.g. duty-cycle sleep BUSY window).
	 *
	 * Counted, because this refusal was invisible and a whole line of
	 * reasoning was built on the wrong counter: the r/b/a figures only move
	 * when a read fails from INSIDE an already-started burst, so they say
	 * nothing about how often the sampler is turned away here.  Whether the
	 * noise floor actually goes stale under duty cycle — and therefore
	 * whether the CAD prefilter admits probes at moments that are not quiet —
	 * cannot be answered without this number. */
	if (!isRadioReady()) {
		_rssi_dc_blocked++;
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
	int got = hwGetRssiBurst(samples, NOISE_FLOOR_SAMPLES_PER_TICK, window_us);

	if (got < 0) {
		/* A preamble or header landed inside the window, so these
		 * samples measure that signal and not the floor.  Not a sampler
		 * fault: charging it to the read counters would report a busy
		 * channel as a failing bus, which is the distinction those
		 * counters exist to draw.  Only the abandoned count moves, and
		 * the short retry deadline set above stands.
		 *
		 * Leaving _sample_fresh clear is load-bearing beyond the floor.
		 * On the LR families the CAD probe's own isReceiving() re-check
		 * below cannot see this — with a duty cycle armed it answers
		 * from a latch the sampler's re-arm has just zeroed — so this is
		 * what actually keeps a calibration CAD off the air while a
		 * neighbour is transmitting. */
		_rssi_bursts_abandoned++;
		return;
	}

	_rssi_reads_ok += (uint32_t)got;
	if (got < NOISE_FLOOR_SAMPLES_PER_TICK) {
		/* Chip busy or RSSI read contended — keep the short retry
		 * deadline set above and try again shortly.
		 *
		 * Counted, not yet tolerated: whether to accept a partial burst
		 * depends on how these reads fail, which is exactly what the
		 * counters are here to establish. */
		_rssi_reads_busy++;
		_rssi_bursts_abandoned++;
		return;
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
		/* The read counters are printed alongside the burst count as
		 * (N rX/bY/aZ), so a reader computes ratios across the two sets.
		 * Halving only the burst side made those ratios wrong by 2x
		 * after the first rescale and 4x after the next. */
		_rssi_reads_ok >>= 1;
		_rssi_reads_busy >>= 1;
		_rssi_bursts_abandoned >>= 1;
		_rssi_dc_blocked >>= 1;
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

	/* Stuck-AGC evidence, gathered from a reading we already took.  A live
	 * front end dithers by a dB or two between samples even on a quiet
	 * channel; a desensitised one returns the same number forever.  Costs
	 * nothing and needs no chip command of its own. */
	if (rssi == _agc_rssi_last) {
		if (_agc_rssi_frozen < 0xFF) {
			_agc_rssi_frozen++;
		}
	} else {
		_agc_rssi_last = rssi;
		_agc_rssi_frozen = 0;
	}

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
	 * payload phase.  hwIsReceiving() never clears IRQ bits on the poll
	 * path itself; foreign preambles are released by the driver in
	 * software, on an SF-aware grace plus a max-airtime header deadline
	 * (SX126x patch 0013, LR11xx, LR20xx alike).  It is not a hardware
	 * release — the chips latch these bits until ClearIrq, and continuous
	 * RX has no timeout to do it for them. */
	if (hwIsReceiving()) {
		return true;
	}
	return isChannelActive();
}

/* ── Receiver hygiene ─────────────────────────────────────────────────
 *
 * Two independent faults, two independent triggers, neither on the packet path.
 *
 * 1. STUCK AGC.  Semtech's remedy is a warm sleep plus recalibration; it is not
 *    in the datasheets, so it is not up for removal on datasheet reasoning.
 *    What IS a design choice is when to fire it, and the honest answer is that
 *    a periodic reset gets it backwards: a timer fires most often on a busy
 *    channel, which is exactly where a received packet has just PROVED the AGC
 *    is working, and it fires no more often on a silent one, where a
 *    desensitised receiver is invisible and nothing else will reveal it.
 *
 *    So trigger on silence instead.  Arduino MeshCore's periodic
 *    `agc_reset_interval` addressed the same fault and shipped defaulted to 0
 *    (off), which is a fair summary of how well a plain timer serves it.
 *
 *    This used to claim the reset was free because an idle node "has no traffic
 *    to miss".  That is false, and hardware disproved it: the reset fires at the
 *    END of a silent stretch, which is precisely when traffic resumes, and
 *    hwResetAgc() takes the radio out of RX for the whole warm-sleep +
 *    Calibrate(ALL) + image-cal + re-entry sequence.  A packet was measured lost
 *    to exactly that window on 2026-08-23 (T1000-E, 92 ms after the re-arm).
 *
 *    Silence is therefore treated as a prerequisite, not as proof.  Firing also
 *    requires corroboration from evidence already on hand: the noise-floor
 *    sampler reads RSSI every interval regardless, and a desensitised front end
 *    reports a frozen value.  Both conditions together, and the operation
 *    essentially never runs on a healthy node — which is the only acceptable
 *    cost for a watchdog guarding a fault nobody has observed in the field.
 *
 *    Deliberately NOT reset here: the noise floor.  The previous periodic
 *    implementation (removed in fe6e585) zeroed it on every fire, forcing a
 *    fresh seed and a full EMA warmup each time — that, not the AGC work, is
 *    what made it a net loss.  Arduino zeroes it too; we do not.
 *
 * 2. CALIBRATION DRIFT.  Image/front-end and PLL/AAF calibration are valid for
 *    a temperature range, not forever: "Image calibration is necessary if there
 *    is a frequency change > 10MHz, or a temperature change > 10 C", and the
 *    LR2021 additionally advises redoing PLL and AAF beyond +/-20 C.  A node
 *    that boots on a hot afternoon and runs into a cold night crosses both.
 *    Nothing does this automatically: the LR2021's temperature-compensation
 *    block (DS 6.12) only corrects crystal drift from TX self-heating, and it
 *    refuses to run at all when a TCXO is fitted.
 *
 *    IMAGE_CAL_TEMP_DELTA_C is deliberately tighter than either datasheet
 *    figure — cheap insurance, and the reading is a junction temperature that
 *    lags ambient. */
/* Silence alone is NOT evidence of a fault — see the rationale block above.
 * Ten minutes, not one: a genuinely deaf receiver stays deaf, so waiting costs
 * nothing, whereas a 60 s threshold fired 4-8 times per quarter hour on a
 * perfectly healthy node (measured 2026-08-23). */
#define AGC_IDLE_RESET_MS        600000U

/* Consecutive identical noise-floor readings before silence is believed.  At
 * the default 15 s sampler interval this is two minutes of a frozen front end. */
#define AGC_STUCK_RSSI_SAMPLES   8U

/* How often to report an ongoing RX silence.  Diagnostic only. */
#define SILENCE_REPORT_MS        120000U

/* Image-calibration drift threshold.  Nothing to do with the AGC reset above —
 * this is the front end.  SX126x DS §9.2.1 / LR11xx UM: image calibration is
 * required after a frequency change > 10 MHz or a temperature change > 10 C.
 * Half the datasheet figure is used so drift is corrected before it reaches the
 * point where the datasheet says the calibration is already stale. */
#define IMAGE_CAL_TEMP_DELTA_C   5

/* Temperature-drift poll cadence.  The maintenance pass itself runs every
 * CONFIG_ZEPHCORE_NOISE_FLOOR_INTERVAL_MS (15 s by default); reading the
 * junction temperature that often is 240 chip commands an hour to watch a
 * quantity that physically cannot move IMAGE_CAL_TEMP_DELTA_C in minutes.
 * Every one of those commands is an opportunity to collide with a duty-cycled
 * radio's autonomous sleep transition, so the cheapest read is the one not
 * issued.  The driver-side BUSY guards make the collision safe; this makes it
 * rare. */
#define IMAGE_CAL_POLL_MS        3600000U   /* 1 h between routine reads */
#define IMAGE_CAL_CONFIRM_MS       15000U   /* re-read before acting on a delta */
#define IMAGE_CAL_TX_QUIET_MS      60000U   /* let PA self-heating decay first */

/* Two independent jobs share this hook because they share one precondition —
 * the chip must be idle enough to accept a command — and one cadence source,
 * the maintenance loop.  They are otherwise unrelated: agcIdleMaintenance()
 * unsticks a receiver that has stopped hearing anything, imageCalMaintenance()
 * corrects front-end image calibration against temperature drift.  Keep them in
 * separate functions so neither's thresholds read as if they governed the
 * other. */
void LoRaRadioBase::radioMaintenance()
{
	/* Never mid-transmit or mid-receive: both operations warm-sleep the
	 * chip, which aborts a TX and destroys an in-flight packet.  The RX
	 * half cannot be inferred from the activity counters below — those
	 * only move at RX_DONE/CRC_ERR, so a packet whose preamble is landing
	 * right now still reads as silence.  isReceiving() is the latch that
	 * knows (HEADER_VALID promotion + preamble grace); it is the same gate
	 * checkSend() and the noise-floor sampler use before touching the chip.
	 * Bailing here just defers the work to the next maintenance pass. */
	if (atomic_get(&_tx_active) || isReceiving()) {
		return;
	}

	uint32_t now = (uint32_t)k_uptime_get_32();

	/* Deafness telemetry, every family, no action taken.
	 *
	 * agcIdleMaintenance() used to be the only thing that noticed a long
	 * silence, and it is now family-gated — so on the LR parts nothing
	 * reports it at all.  A silence marker on both boards is what lets a
	 * side-by-side capture say WHICH radio stopped hearing, and the
	 * accompanying counters say what the sampler was seeing at the time. */
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
				(int)_noise_floor);
		}
	}

	agcIdleMaintenance(now);
	imageCalMaintenance(now);
}

/* Receiver watchdog: a stuck AGC stops the demodulator hearing anything at all,
 * so prolonged total silence is the symptom.  Nothing here concerns the front
 * end or temperature. */
void LoRaRadioBase::agcIdleMaintenance(uint32_t now)
{
	if (!hwNeedsAgcReset()) {
		return;
	}

	/* Any demodulation activity counts as proof of life, errored frames
	 * included — a CRC failure still means RF reached the demodulator, which
	 * is precisely what a stuck AGC would prevent.  Counting only good
	 * packets would fire resets on a node that is merely out of range. */
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

/* Front-end image calibration against temperature drift.  Separate from the AGC
 * reset above in every respect: different symptom (degraded image rejection, not
 * a deaf demodulator), different datasheet section, different remedy
 * (hwRecalibrate(), not hwResetAgc()). */
void LoRaRadioBase::imageCalMaintenance(uint32_t now)
{
	if (!hwHasDriftRecal()) {
		return;
	}

	/* Polled on its own slow cadence rather than once per
	 * maintenance pass — see IMAGE_CAL_POLL_MS.  Backends that cannot measure
	 * (SX126x/SX127x) answer INT16_MIN from a plain inline with no bus
	 * traffic, so the early return below costs them nothing. */
	if (_image_cal_started && (now - _image_cal_last_ms) < _image_cal_wait_ms) {
		return;
	}

	/* Wait out our own PA.  The junction is still warm for a while after a
	 * transmit, and that self-heating is not the ambient drift the
	 * recalibration exists to track — acting on it would recalibrate against
	 * a temperature the chip will not be at a minute later.  Deferring
	 * returns here without stamping, so the retry is the next pass (seconds)
	 * rather than the next poll window (an hour). */
	if (_last_tx_start_ms != 0 &&
	    (now - _last_tx_start_ms) < IMAGE_CAL_TX_QUIET_MS) {
		return;
	}

	/* Temperature comes from the BOARD, never from the radio.
	 *
	 * Only the delta matters here, never the absolute value, and the MCU die
	 * sensor tracks the same ambient the front end sits in — so it answers
	 * the question just as well as the radio's junction sensor while costing
	 * the radio nothing at all.  Reading it off the chip meant a periodic SPI
	 * command aimed at a part that spends most of its time in an autonomous
	 * duty-cycle sleep phase; that read wedged the LR1110 BUSY-high 7 times
	 * in 47 minutes of measurement on 2026-08-23, each costing 12-18 s of
	 * deafness, and it cost a packet.  The junction sensor was also the worse
	 * instrument for the job: it sees PA self-heating, which is exactly the
	 * transient this path must not react to. */
	float board_temp = _board ? _board->getMCUTemperature() : NAN;

	if (isnan(board_temp)) {
		/* No board temperature source — drift handling simply does not run
		 * on this hardware, as it did not before on families without a
		 * junction sensor either. */
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

	/* Measure twice before acting.  A single reading over the threshold can
	 * be a transient — residual self-heating the quiet window did not fully
	 * cover, or a one-off bad sample — and hwRecalibrate() takes the radio
	 * out of receive.  Re-read shortly and act only if the second reading
	 * agrees. */
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

/* The offset window is [CAD_LEVEL_MIN, CAD_LEVEL_MAX], but an offset is only
 * meaningful while base+offset still lands somewhere the driver will actually
 * program.  Past the hardware clamp several offsets collapse onto one peak, and
 * the staircase cannot tell them apart — it reads sampling noise as curvature.
 * Narrow the window so every level it can reach is a distinct configuration.
 *
 * Radios that report no clamp (hwCadPeakMin/Max == 0) keep the static window,
 * which is also what a radio with no adaptive CAD at all gets. */
int8_t LoRaRadioBase::cadLevelMinEff()
{
	uint8_t base = hwCadBasePeak();
	uint8_t pmin = hwCadPeakMin();

	/* The clamp only binds when the lowest peak the static window can reach,
	 * base + CAD_LEVEL_MIN, would land below it.  Note the sign: this was
	 * written `base - CAD_LEVEL_MIN` once, which with CAD_LEVEL_MIN negative
	 * evaluates to base + 8 — always above pmin, so the narrowing never
	 * happened and the whole function was inert. */
	if (base == 0 || pmin == 0 ||
	    (int)base + CAD_LEVEL_MIN >= (int)pmin) {
		return CAD_LEVEL_MIN;
	}
	return (int8_t)((int)pmin - (int)base);
}

int8_t LoRaRadioBase::cadLevelMaxEff()
{
	uint8_t base = hwCadBasePeak();
	uint8_t pmax = hwCadPeakMax();

	if (base == 0 || pmax == 0 || (int)pmax - (int)base >= CAD_LEVEL_MAX) {
		return CAD_LEVEL_MAX;
	}
	return (int8_t)((int)pmax - (int)base);
}

void LoRaRadioBase::setCadParams(bool auto_enabled, int8_t offset,
				 uint16_t probe_interval_s, uint8_t busycap_pct,
				 uint8_t stored_base)
{
	const int8_t lo = cadLevelMinEff();
	const int8_t hi = cadLevelMaxEff();
	const uint8_t base = hwCadBasePeak();

	/* Re-anchor across a base-table change.
	 *
	 * cad_offset is persisted; the per-level probe statistics that justified
	 * it are not (RAM only, cleared by reconfigure() and lost at every
	 * reboot).  So when a firmware update moves the family base table, a
	 * converged node wakes up with an offset that names a different absolute
	 * detPeak than the one it spent days measuring.
	 *
	 * Preserve the PEAK, not the offset: the peak is the physical quantity
	 * the node actually measured, and the offset is only how we address it.
	 * A node at base 51 / offset -7 lands on base 44 / offset 0 — the same
	 * detPeak 44, now centred in its window instead of one rung off the rail.
	 *
	 * Resetting to 0 instead would throw away real convergence for no reason,
	 * and is only the right answer when the preserved peak falls outside the
	 * window the new base can reach — which the clamp below handles, because
	 * a peak that is no longer addressable is not a peak we can operate at.
	 *
	 * stored_base == 0 means "never recorded" (a node upgrading from a build
	 * without the field), and is deliberately a no-op: with no record of
	 * which base the offset came from, any adjustment would be a guess. */
	if (stored_base != 0 && base != 0 && stored_base != base) {
		int adj = (int)offset + (int)stored_base - (int)base;

		LOG_INF("cad: base %u -> %u, re-anchoring offset %d -> %d "
			"(peak %d held)",
			(unsigned)stored_base, (unsigned)base,
			(int)offset, adj, (int)stored_base + (int)offset);
		offset = (int8_t)(adj < -128 ? -128 : (adj > 127 ? 127 : adj));
	}

	if (offset < lo) offset = lo;
	if (offset > hi) offset = hi;

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

	/* Effective, not _cad_offset: applyCadPrefs() reaches here from ordinary
	 * `set cad.*` commands, which an operator can issue while a `tempradio`
	 * window is open.  Programming the configured offset then would put the
	 * visited preset back on a threshold that does not belong to it. */
	hwCadSetPeakOffset(cadEffectiveOffset());

	LOG_INF("cad: auto=%d offset=%d base=%u measure_interval=%ums busycap=%u%%",
		(int)auto_enabled, (int)offset, (unsigned)base,
		(unsigned)_measure_interval_ms, (unsigned)busycap_pct);
}

uint8_t LoRaRadioBase::cadBasePeak()
{
	return hwCadBasePeak();
}

void LoRaRadioBase::resetCadStats()
{
	memset(_cad_stats, 0, sizeof(_cad_stats));
	_cad_probe_rr = 0;

	/* Drop any probe still in flight with the table it belongs to.
	 * cadMaintenance() books a verdict into whatever stats exist when it
	 * resolves, not the ones that were current when it was armed — so a
	 * probe left pending across reconfigure() writes an old-preset sample
	 * into the fresh table, and across `set cad.reset` writes a pre-reset
	 * sample into a table the operator just cleared. */
	_cad_pending_level = INT8_MIN;
	_cad_pending_deadline_ms = 0;
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
		/* The sweep window is ABSOLUTE, so an operating offset parked
		 * outside it is never probed at all — measured: 49 minutes at
		 * offset -8 produced 0 probes at -8, which left cadSafetyStep()
		 * with no evidence and made it a no-op in precisely the
		 * configuration it exists for (auto off + an offset that cannot
		 * clear).
		 *
		 * When the operator has parked outside the window, probe where
		 * they actually are instead: the swept curve does not contain
		 * their operating point, so it cannot inform the hand-tuning it
		 * was built for either.  Inside the window the sweep already
		 * covers the operating level and is left exactly as it was. */
		if (_cad_offset < CAD_SWEEP_MIN || _cad_offset > CAD_SWEEP_MAX) {
			return _cad_offset;
		}

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
	if (lvl < cadLevelMinEff() || lvl > cadLevelMaxEff()) {
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
	int r_up = fp_rate(oi + 1);  /* one step less sensitive */
	int r_dn = fp_rate(oi - 1);  /* frontier, one step more sensitive */

	/* Airtime protection used to live here as the highest-priority rung.  It
	 * has moved to cadSafetyStep(), which the callers run BEFORE this and
	 * without the _cad_auto gate — it is a safety, not an optimisation, and
	 * gating it meant a node whose detPeak could never clear had no way back
	 * once the operator turned auto off.  What remains here is purely the
	 * knee-seeking optimiser. */
	int cap_permille = (int)_cad_busycap_pct * 10;

	/* Step UP (less sensitive) when the level above is markedly cleaner —
	 * we're on the steep part of the curve, below the knee. */
	if (_cad_offset < cadLevelMaxEff() && r_up >= 0 &&
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
		       (b_dn <= cap_permille -
				(cap_permille * CAD_BUSY_DEFER_HYST_PCT) / 100);
	if (_cad_offset > cadLevelMinEff() && r_dn >= 0 &&
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

bool LoRaRadioBase::cadSafetyStep()
{
	/* The airtime-protection rung, run unconditionally — see the constant
	 * block in radio_common.h for why it is not gated on _cad_auto.
	 *
	 * This is the proactive half of the pair.  cadRelaxOnTxStarvation() only
	 * fires once the node actually has traffic it cannot send, which on a
	 * silent mesh may be up to flood_advert_interval away (47 h by default);
	 * this one works off probe statistics, which accumulate whether or not
	 * there is anything to transmit. */
	if (_cad_offset >= cadLevelMaxEff()) {
		return false;
	}

	int oi = _cad_offset - CAD_LEVEL_MIN;
	if (oi < 0 || oi >= CAD_NUM_LEVELS) {
		return false;
	}

	uint16_t probes = _cad_stats[oi].probes;
	if (probes == 0) {
		return false;
	}

	int b_op = (int)(((uint32_t)_cad_stats[oi].busy * 1000U) / probes);
	int cap_permille = (int)_cad_busycap_pct * 10;

	/* Unambiguous: this level trips on nearly every probe, so it cannot
	 * clear for a transmit either.  Acts on few samples and ignores the
	 * cap. */
	bool pathological = probes >= CAD_SAFETY_MIN_PROBES &&
			    b_op >= CAD_SAFETY_PATHOLOGICAL_PERMILLE;
	/* Marginal: a real airtime-vs-capture tradeoff.  Keeps the original
	 * evidence bar and honours `cad.busycap 0` as the operator's choice. */
	bool over_cap = cap_permille && probes >= CAD_STEP_MIN_PROBES &&
			b_op > cap_permille;

	if (!pathological && !over_cap) {
		return false;
	}

	_cad_offset++;
	hwCadSetPeakOffset(_cad_offset);
	if (pathological) {
		LOG_WRN("cad: safety step up -> offset %d (busy %d permille over "
			"%u probes — detector too sensitive to clear, auto=%d)",
			(int)_cad_offset, b_op, (unsigned)probes, (int)_cad_auto);
	} else {
		LOG_INF("cad: step up -> offset %d (airtime, busy %d cap %d)",
			(int)_cad_offset, b_op, cap_permille);
	}
	return true;
}

bool LoRaRadioBase::cadRelaxOnTxStarvation()
{
	/* Deliberately does NOT consult _cad_auto.  Every other mover of
	 * _cad_offset is an optimiser and correctly stays out of the way when
	 * the operator has taken manual control; this one exists precisely for
	 * the case where manual control produced a node that cannot transmit,
	 * so honouring `cad.auto off` here would disable the safety exactly
	 * where it is needed.  It also ignores cad.busycap and the per-level
	 * probe statistics: 120 warm probes are half an hour away, and the
	 * caller has already established the harm directly.
	 *
	 * One step at a time, never a jump to base: on a genuinely congested
	 * site the operator's sensitive setting may be almost right, and the
	 * smallest change that restores transmission is the one to make.
	 *
	 * This one safety DOES run during a temporary override — unlike the
	 * staircase, which cadMaintenance() suspends there.  A node muted for
	 * the length of a `tempradio` window, or until the reboot a `set radio`
	 * is waiting for, is muted for real.  It steps the visitor's offset, so
	 * the recovery lasts exactly as long as the visit and still cannot reach
	 * prefs: getCadOffset() keeps reporting the configured value. */
	int8_t &off = _has_radio_override ? _cad_temp_offset : _cad_offset;

	if (off >= cadLevelMaxEff()) {
		return false;
	}

	off++;
	hwCadSetPeakOffset(cadEffectiveOffset());
	LOG_WRN("cad: TX starvation override -> offset %d (auto=%d%s) — LBT was "
		"refusing every transmit",
		(int)off, (int)_cad_auto,
		_has_radio_override ? ", temp preset" : "");
	return true;
}

void LoRaRadioBase::cadMaintenance()
{
	if (_probe_interval_s == 0) {
		return;
	}

	/* Nothing to learn on a preset the node is only visiting: the statistics
	 * would describe that preset, and every mover of _cad_offset below is
	 * persisted by Dispatcher::maintenanceLoop() -> onCadOffsetChanged() ->
	 * prefs, so letting the staircase run inside a `tempradio` window would
	 * write a visitor's offset to flash and leave it there after the revert.
	 * The chip runs at the visited preset's own base (setRadioOverride).
	 *
	 * The pending sample is dropped rather than carried: it was taken under
	 * the other preset, and consuming it after the revert would admit a
	 * probe on a quiet verdict that is a whole interval stale. */
	if (_has_radio_override) {
		_sample_fresh = false;
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

	/* A CAD_RX probe from an earlier pass, now resolvable.
	 *
	 * This replaces the four-poll confirmation window that used to run
	 * inline after every busy probe.  That window existed only because the
	 * old CAD_ONLY exit threw away the reception which triggered the
	 * detection, leaving nothing to wait for and no choice but to guess from
	 * a poll -- and it guessed badly, confirming about a quarter of busies on
	 * the best node and one in forty-five on a duty-cycled LR1110.
	 *
	 * With CAD_RX the chip keeps that reception and always resolves it with
	 * a terminal interrupt: a packet, or its own cadTimeout.  So there is a
	 * real event to observe, the answer is read once after the chip's own
	 * deadline, and it costs no chip access at all. */
	if (_cad_pending_level != INT8_MIN && now >= _cad_pending_deadline_ms) {
		CadLevelStats &ps = _cad_stats[_cad_pending_level - CAD_LEVEL_MIN];
		int outcome = hwCadRxOutcome();

		if (outcome == 1) {
			ps.tp++;
		} else if (outcome == 2) {
			ps.fp++;
		} else {
			/* Past the chip's own deadline with neither terminal
			 * interrupt seen.  Nothing to infer from that, and
			 * inventing a verdict would poison the very curve the
			 * staircase reads -- so un-count the sample entirely
			 * rather than book it as either.  Dropping the busy with
			 * it keeps busy_rate and fp_rate describing the same
			 * population. */
			if (ps.probes) ps.probes--;
			if (ps.busy) ps.busy--;
			LOG_WRN("cad: probe at %+d unresolved past deadline, discarded",
				(int)_cad_pending_level);
		}
		_cad_pending_level = INT8_MIN;

		/* Safety first, and outside the _cad_auto gate.  When it acts,
		 * skip the optimiser this pass — it has just moved the operating
		 * level and the three-rung window it reads is stale. */
		if (!cadSafetyStep() && _cad_auto) {
			cadStaircaseStep();
		}
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

	/* One probe in flight at a time.  A second CAD while the first is still
	 * in its CAD_RX window would abort that Rx -- destroying the reception
	 * being measured -- and there is only one pending slot to book it to. */
	if (_cad_pending_level != INT8_MIN) {
		return;
	}

	/* No CAD_RX ground truth on this radio means no probing at all.
	 *
	 * The staircase reads a FALSE-positive rate, so a probe whose outcome
	 * can never be established is not a weaker sample, it is a poisoned one:
	 * every busy verdict would be booked as neither fp nor tp, fp_rate would
	 * read zero at every level, the curve would look perfectly clean, and
	 * the controller would walk to the most sensitive rail on a channel it
	 * has learned nothing about.  Silently collecting unusable data is worse
	 * than collecting none.
	 *
	 * The one radio this currently excludes is the LR2021.  Its cad_timeout
	 * is 24 bits of 32 MHz periods (DS 6.3.11), i.e. 524 ms, while a
	 * max-length packet at the default SF7/BW62.5 preset runs 1704 ms -- and
	 * the datasheet is explicit that the chip "stays in Rx until a packet is
	 * demodulated or the timer reaches the timeout", so that bound truncates
	 * receptions rather than merely ending a wait.  Enabling CAD_RX there
	 * would trade a measurement for lost packets, which is the wrong way
	 * round.  `set cad.offset` still works by hand.
	 *
	 * (The SX126x's ceiling is 262 s at 15.625 us steps and the LR11xx's is
	 * 512 s at 32768 Hz RTC steps, so neither comes close to binding.) */
	if (hwCadRxTimeoutMs() == 0) {
		return;
	}

	if (!_sample_channel_quiet) {
		return;
	}

	/* Re-check immediately before the CAD.  The sampler's own isReceiving()
	 * guard ran BEFORE its 8-read burst, so by the time we get here it is
	 * about a millisecond stale — long enough for a packet to have started.
	 * Calibration must never cost a reception, and abandoning the probe is
	 * free: the next interval is 15 s away and nothing depends on this one. */
	if (isReceiving()) {
		return;
	}

	_cad_last_probe_ms = now;

	int8_t level = pickCadProbeLevel();
	int ret = hwCadProbe(level);
	/* hwCadProbe() BLOCKS for the whole CAD, so `now` is already stale here
	 * and must not be used as the base of the CAD_RX deadline below. */
	int64_t probe_done_ms = k_uptime_get();

	/* Where the chip is now depends on the verdict, and the caller must not
	 * guess:
	 *   free (0)  -> standby; re-enter RX exactly as before.
	 *   busy (2)  -> the chip is ALREADY in RX, locked on the signal CAD
	 *                found, and re-entering would tear down the reception
	 *                this probe exists to observe.  Leave it alone.
	 *   busy (1)  -> a radio still on the CAD_ONLY exit (no CAD_RX support):
	 *                standby, re-enter RX, and no ground truth is available.
	 *   error     -> standby; re-enter RX and give up on this pass. */
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

	CadLevelStats &s = _cad_stats[level - CAD_LEVEL_MIN];

	if (s.probes >= 0xFFF0) {
		decayCadStats();
	}
	s.probes++;

	if (ret > 0) {
		s.busy++;
	}

	if (ret == 2) {
		/* Book the rung and come back once the chip's own cadTimeout has
		 * passed, when a terminal interrupt is guaranteed to have landed.
		 * The extra 100 ms is interrupt-to-work-queue latency, not a
		 * safety margin against the chip: the deadline itself is the
		 * chip's.
		 *
		 * Measured from AFTER the probe, because the chip's cadTimeout
		 * starts when CAD_DONE hands it into Rx — which is exactly when
		 * the blocking hwCadProbe() returns.  Based on `now` instead, it
		 * was short by the whole CAD duration, and a 4-symbol CAD is
		 * ~131 ms at SF11/BW62.5 and ~262 ms at SF12 (the figure
		 * sx126x_cad_timeout_ms() sizes its own wait from), i.e. more
		 * than the margin.  The outcome was then read before the chip
		 * could raise its timeout, so every FALSE positive was discarded
		 * while true positives — which land early, on a packet — still
		 * counted: fp_rate read ~0 at every rung and the staircase
		 * descended to the sensitive rail on a curve it had not
		 * measured. */
		_cad_pending_level = level;
		_cad_pending_deadline_ms =
			probe_done_ms + (int64_t)hwCadRxTimeoutMs() + 100;
		return;
	}

	if (!cadSafetyStep() && _cad_auto) {
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
	 * per interval, forever, on every repeater.
	 *
	 * A CAD_RX probe awaiting its terminal event is the one exception, and it
	 * is not a second grid: it is one wake, only while a probe is actually
	 * pending, at the moment the chip has guaranteed an answer exists.  It
	 * would otherwise wait for the next sampler tick, leaving the node in the
	 * plain RX that CAD_RX entered instead of handing the duty cycle back. */
	if (_cad_pending_level != INT8_MIN) {
		next = mesh::maintenanceSooner(
			next, clampDeadline(_cad_pending_deadline_ms - now));
	}

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
	 *   a:tmp  a temporary radio override is running a preset the node is
	 *      only visiting: adaptation is suspended and o/pk describe that
	 *      visit, not the configured preset (see setRadioOverride).
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

	const int eff_off = cadEffectiveOffset();

	n += snprintf(buf + n, cap > n ? cap - n : 0,
		      "a:%s o:%d pk:%d(b%u/4s) sp:%u.%u/%u%%",
		      _has_radio_override ? "tmp" : (_cad_auto ? "on" : "off"),
		      eff_off, (int)base + eff_off, base,
		      spread_mean10 / 10U, spread_mean10 % 10U, degen_pct);
	if (room_for_count) {
		/* bursts(ok reads/busy reads/abandoned bursts/dc-blocked) — the
		 * busy and abandoned figures distinguish a sampler that is losing
		 * the odd read from one that is being refused mid-burst, and the
		 * dc figure distinguishes both from one that never gets to start
		 * because the duty-cycle sleep window turns it away.  Without the
		 * last one, "refused constantly" and "running perfectly" both
		 * print b0/a0. */
		n += snprintf(buf + n, cap > n ? cap - n : 0,
			      "(%u r%u/b%u/a%u/d%u)",
			      (unsigned)_rssi_bursts,
			      (unsigned)_rssi_reads_ok,
			      (unsigned)_rssi_reads_busy,
			      (unsigned)_rssi_bursts_abandoned,
			      (unsigned)_rssi_dc_blocked);
	}
	n += snprintf(buf + n, cap > n ? cap - n : 0, " bc:%u%%",
		      (unsigned)_cad_busycap_pct);
	/* Probing off because the radio cannot supply CAD_RX ground truth (see
	 * cadMaintenance).  Stated rather than left to be inferred from level
	 * counters that never move. */
	if (hwCadRxTimeoutMs() == 0) {
		n += snprintf(buf + n, cap > n ? cap - n : 0, " probe:n/a");
	}

	/* Only the 3 rungs around the operating offset — the far rungs are mildly
	 * irrelevant; what matters is where we sit on the ladder.  The window is
	 * clamped to the EFFECTIVE range while still showing 3 rungs, so at either
	 * end it slides inward rather than dropping a line.
	 *
	 * Effective, not the static constants: past the hardware detPeak clamp
	 * several offsets program the same peak, and showing them as separate
	 * rungs invited exactly the wrong reading — three lines of distinct
	 * statistics for one physical configuration.  With the range narrowed,
	 * every rung printed is a real one and `pk` below is what the chip got. */
	const int lmin = cadLevelMinEff();
	const int lmax = cadLevelMaxEff();
	int cur = eff_off;
	if (cur < lmin) cur = lmin;
	if (cur > lmax) cur = lmax;
	int lo = cur - 1, hi = cur + 1;
	if (lo < lmin) { lo = lmin; hi = lo + 2; }
	if (hi > lmax) { hi = lmax; lo = hi - 2; }
	if (lo < CAD_LEVEL_MIN) lo = CAD_LEVEL_MIN;
	if (hi > CAD_LEVEL_MAX) hi = CAD_LEVEL_MAX;

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
