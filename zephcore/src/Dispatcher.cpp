/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Dispatcher implementation
 */

#include <mesh/Dispatcher.h>
#include <mesh/MeshCore.h>
#include <mesh/Utils.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
LOG_MODULE_REGISTER(zephcore_dispatcher, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
#define PAYLOAD_TYPE_REQ         0x00
#define PAYLOAD_TYPE_RESPONSE    0x01
#define PAYLOAD_TYPE_TXT_MSG     0x02
#define PAYLOAD_TYPE_PATH        0x08
#endif

namespace mesh {

#define MAX_RX_DELAY_MILLIS         32000  /* upper bound for score-based RX delay */
#define MIN_TX_BUDGET_AIRTIME_DIV   2      /* require at least 1/N MTU airtime as budget before TX */

Dispatcher::Dispatcher(Radio &radio, MillisecondClock &ms, PacketManager &mgr)
	: _radio(&radio), _ms(&ms), _mgr(&mgr)
{
	outbound = nullptr;
	total_air_time = rx_air_time = 0;
	next_tx_time = 0;
	cad_busy_start = 0;
	tx_budget_ms = 0;
	last_budget_update = 0;
	duty_cycle_window_ms = 0;
	next_agc_reset_time = 0;
	_err_flags = 0;
	radio_nonrx_start = 0;
	prev_isrecv_mode = true;
	n_sent_flood = n_sent_direct = 0;
	n_recv_flood = n_recv_direct = 0;
	_tx_queued_cb = nullptr;
	_tx_queued_user_data = nullptr;
}

void Dispatcher::begin()
{
	n_sent_flood = n_sent_direct = 0;
	n_recv_flood = n_recv_direct = 0;
	_err_flags = 0;
	uint32_t now = (uint32_t)_ms->getMillis();
	radio_nonrx_start = now;
	duty_cycle_window_ms = getDutyCycleWindowMs();
	tx_budget_ms = getMaxTxBudgetMs();
	last_budget_update = now;
	next_tx_time = now;
	_radio->begin();
	prev_isrecv_mode = _radio->isInRecvMode();
}

uint8_t Dispatcher::getDutyCyclePercent() const
{
	return 10; /* EU 868 default: 10% duty cycle */
}

uint32_t Dispatcher::getMaxTxBudgetMs() const
{
	uint8_t duty_pct = getDutyCyclePercent();
	if (duty_pct == 0 || duty_cycle_window_ms == 0) {
		return 0;
	}
	return (duty_cycle_window_ms * (uint32_t)duty_pct) / 100U;
}

void Dispatcher::updateTxBudget()
{
	uint8_t duty_pct = getDutyCyclePercent();
	if (duty_pct == 0 || duty_cycle_window_ms == 0) {
		return;
	}

	uint32_t now = (uint32_t)_ms->getMillis();
	uint32_t elapsed = now - last_budget_update;
	if (elapsed == 0) {
		return;
	}

	uint32_t refill = (elapsed * (uint32_t)duty_pct) / 100U;
	if (refill > 0) {
		uint32_t max_budget = getMaxTxBudgetMs();
		tx_budget_ms += refill;
		if (tx_budget_ms > max_budget) {
			tx_budget_ms = max_budget;
		}
		last_budget_update = now;
	}
}

bool Dispatcher::isAdminPacket(const Packet *pkt)
{
	uint8_t t = pkt->getPayloadType();
	return t == PAYLOAD_TYPE_REQ || t == PAYLOAD_TYPE_RESPONSE ||
	       t == PAYLOAD_TYPE_ANON_REQ || t == PAYLOAD_TYPE_CONTROL;
}

int Dispatcher::calcRxDelay(float score, uint32_t air_time) const
{
	/* LUT: 10^(0.85 - i*0.1) - 1, i=0..10; replaces powf() (~1.9KB saved) */
	static const float lut[11] = {
		6.0793f, 4.6236f, 3.4674f, 2.5489f, 1.8184f, 1.2389f,
		0.7783f, 0.4125f, 0.1220f, -0.1089f, -0.2921f
	};
	if (score <= 0.0f) return (int)(lut[0] * (float)air_time);
	if (score >= 1.0f) return (int)(lut[10] * (float)air_time);
	float idx = score * 10.0f;
	int i = (int)idx;
	float frac = idx - (float)i;
	float val = lut[i] + frac * (lut[i + 1] - lut[i]);
	return (int)(val * (float)air_time);
}

uint32_t Dispatcher::getCADFailRetryDelay() const
{
	/* 100-200ms jittered retry: tighter than one SF8 flood airtime so we
	 * sample multiple RX duty-cycle windows, and randomized so two nodes
	 * contending on the same channel don't retry in lockstep. */
	return 100 + (sys_rand32_get() % 101);
}

uint32_t Dispatcher::getCADFailMaxDuration() const
{
	return 4000; /* ms; ~20 retry attempts before giving up */
}

void Dispatcher::loop()
{
	if (outbound) {
		if (_radio->isSendComplete()) {
			uint32_t t = (uint32_t)_ms->getMillis() - outbound_start;
			total_air_time += t;
			updateTxBudget();
			if (t >= tx_budget_ms) {
				tx_budget_ms = 0;
			} else {
				tx_budget_ms -= t;
			}
			_radio->onSendFinished();
			logTx(outbound, 2 + outbound->getPathByteLen() + outbound->payload_len);
			if (outbound->isRouteFlood()) {
				n_sent_flood++;
			} else {
				n_sent_direct++;
			}
			releasePacket(outbound);
			outbound = nullptr;
		} else if (millisHasNowPassed(outbound_expiry)) {
			_radio->onSendFinished();
			logTxFail(outbound, 2 + outbound->getPathByteLen() + outbound->payload_len);
			releasePacket(outbound);
			outbound = nullptr;
		} else {
			return;
		}
		next_agc_reset_time = futureMillis(getAGCResetInterval());
	}

	{
		Packet *pkt = _mgr->getNextInbound((uint32_t)_ms->getMillis());
		if (pkt) {
			processRecvPacket(pkt);
		}
	}
	checkRecv();
	checkSend();
}

void Dispatcher::maintenanceLoop()
{
	_radio->triggerNoiseFloorCalibrate(getInterferenceThreshold());

	/* RX mode watchdog: TX counts as "active" to avoid false triggers
	 * when the 5s housekeeping timer misses brief RX windows. */
	bool is_active = _radio->isInRecvMode() || !_radio->isSendComplete();
	if (is_active != prev_isrecv_mode) {
		prev_isrecv_mode = is_active;
		if (!is_active) {
			radio_nonrx_start = (uint32_t)_ms->getMillis();
		}
	}
	if (!is_active && (uint32_t)_ms->getMillis() - radio_nonrx_start > 8000) { /* 8s stall threshold */
		_err_flags |= ERR_EVENT_STARTRX_TIMEOUT;
	}

	/* Periodic AGC recalibration */
	if (getAGCResetInterval() > 0 && millisHasNowPassed(next_agc_reset_time)) {
		_radio->resetAGC();
		next_agc_reset_time = futureMillis(getAGCResetInterval());
	}
}

bool Dispatcher::tryParsePacket(Packet *pkt, const uint8_t *raw, int len)
{
	int i = 0;

	pkt->header = raw[i++];
	if (pkt->getPayloadVer() > PAYLOAD_VER_1) {
		LOG_WRN("tryParsePacket: unsupported packet version");
		return false;
	}

	if (pkt->hasTransportCodes()) {
		memcpy(&pkt->transport_codes[0], &raw[i], 2); i += 2;
		memcpy(&pkt->transport_codes[1], &raw[i], 2); i += 2;
	} else {
		pkt->transport_codes[0] = pkt->transport_codes[1] = 0;
	}

	pkt->path_len = raw[i++];
	uint8_t path_mode = pkt->path_len >> 6;
	if (path_mode == 3) {   /* reserved path mode */
		LOG_WRN("tryParsePacket: unsupported path mode: 3");
		return false;
	}

	uint8_t path_byte_len = (pkt->path_len & 63) * pkt->getPathHashSize();
	if (path_byte_len > MAX_PATH_SIZE || i + path_byte_len > len) {
		LOG_WRN("tryParsePacket: partial or corrupt packet, len=%d", len);
		return false;
	}

	memcpy(pkt->path, &raw[i], path_byte_len); i += path_byte_len;

	pkt->payload_len = len - i;
	if (pkt->payload_len > (int)sizeof(pkt->payload)) {
		LOG_WRN("tryParsePacket: payload too big, payload_len=%d", (uint32_t)pkt->payload_len);
		return false;
	}

	memcpy(pkt->payload, &raw[i], pkt->payload_len);
	return true;
}

void Dispatcher::checkRecv()
{
	/* k_event is a bitfield — multiple ISR arrivals coalesce into one
	 * wake, so drain the entire ring each time. */
	for (;;) {
		uint8_t raw[MAX_TRANS_UNIT + 1];
		int len = _radio->recvRaw(raw, MAX_TRANS_UNIT);
		if (len <= 0) {
			break;
		}

		logRxRaw(_radio->getLastSNR(), _radio->getLastRSSI(), raw, len);

		Packet *pkt = _mgr->allocNew();
		if (pkt == nullptr) {
			LOG_ERR("checkRecv: packet alloc failed");
			break;
		}

		float score = 0.0f;
		uint32_t air_time = 0;

		if (tryParsePacket(pkt, raw, len)) {
			pkt->_snr = (int8_t)(_radio->getLastSNR() * 4.0f); /* x4 fixed-point SNR */
			score = _radio->packetScore(_radio->getLastSNR(), len);
			air_time = _radio->getEstAirtimeFor(len);
			rx_air_time += air_time;
		} else {
			_mgr->free(pkt);
			continue;
		}

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
		/* Arduino-compatible packet logging - use printk to bypass log level filtering */
		{
			static uint8_t packet_hash[MAX_HASH_SIZE];
			static char hash_hex[MAX_HASH_SIZE * 2 + 1];
			pkt->calculatePacketHash(packet_hash);
			Utils::toHex(hash_hex, packet_hash, MAX_HASH_SIZE);

			uint8_t ptype = pkt->getPayloadType();
			if (ptype == PAYLOAD_TYPE_PATH || ptype == PAYLOAD_TYPE_REQ ||
			    ptype == PAYLOAD_TYPE_RESPONSE || ptype == PAYLOAD_TYPE_TXT_MSG) {
				printk("%s: RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d time=%u hash=%s [%02X -> %02X]\n",
					getLogDateTime(), pkt->getRawLength(), ptype,
					pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
					(int)pkt->getSNR(), (int)_radio->getLastRSSI(),
					(int)(score * 1000), air_time, hash_hex,
					(uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
			} else {
				printk("%s: RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d time=%u hash=%s\n",
					getLogDateTime(), pkt->getRawLength(), ptype,
					pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
					(int)pkt->getSNR(), (int)_radio->getLastRSSI(),
					(int)(score * 1000), air_time, hash_hex);
			}
		}
#endif
		logRx(pkt, pkt->getRawLength(), score);
		if (pkt->isRouteFlood()) {
			n_recv_flood++;
			processRecvPacket(pkt);
		} else {
			n_recv_direct++;
			processRecvPacket(pkt);
		}
	}
}

void Dispatcher::processRecvPacket(Packet *pkt)
{
	DispatcherAction action = onRecvPacket(pkt);
	if (action == ACTION_RELEASE) {
		_mgr->free(pkt);
	} else if (action == ACTION_MANUAL_HOLD) {
		/* subclass holds packet */
	} else {
		uint8_t priority = (uint8_t)((action >> 24) - 1);
		uint32_t delay = action & 0xFFFFFF;
		_mgr->queueOutbound(pkt, priority, futureMillis((int)delay));
		if (_tx_queued_cb && delay > 0) {
			_tx_queued_cb(delay, _tx_queued_user_data);
		}
	}
}

void Dispatcher::checkSend()
{
	uint32_t now = (uint32_t)_ms->getMillis();
	int count = _mgr->getOutboundCount(now);
	if (count == 0) {
		cad_busy_start = 0;
		return;
	}

	/* Duty-cycle budget gate. Matches Arduino MeshCore: defer when remaining
	 * budget < est_airtime / MIN_TX_BUDGET_AIRTIME_DIV (i.e. half an MTU's airtime).
	 *
	 * Divergence from upstream: we exempt admin packets from the gate so that
	 * remote management (admin requests, login, etc.) keeps working when a node
	 * has burned its budget. Strictly out-of-spec for EN 300 220 — admin floods
	 * still consume airtime — but a managed node that can't be reached to be
	 * disabled is worse than the marginal extra airtime. Scan is O(N) over
	 * the small (24-32) packet pool so the cost is negligible. */
	updateTxBudget();
	uint8_t duty_pct = getDutyCyclePercent();
	if (duty_pct > 0) {
		bool due_admin_queued = false;
		int total = _mgr->getOutboundTotal();
		for (int i = 0; i < total; i++) {
			Packet *pkt = _mgr->getOutboundByIdx(i);
			if (!pkt) {
				continue;
			}
			if ((int32_t)(_mgr->getOutboundSchedule(i) - now) > 0) {
				continue;
			}
			if (isAdminPacket(pkt)) {
				due_admin_queued = true;
				break;
			}
		}

		uint32_t est_airtime = _radio->getEstAirtimeFor(MAX_TRANS_UNIT);
		uint32_t threshold = est_airtime / MIN_TX_BUDGET_AIRTIME_DIV;
		if (!due_admin_queued && tx_budget_ms < threshold) {
			uint32_t needed = threshold - tx_budget_ms;
			uint32_t delay_ms = (needed * 100U + (uint32_t)duty_pct - 1U) / (uint32_t)duty_pct;
			if (_tx_queued_cb) {
				_tx_queued_cb(delay_ms + 1U, _tx_queued_user_data);
			}
			return;
		}
	}

	bool is_receiving = _radio->isReceiving();
	bool is_radio_ready = _radio->isRadioReady();
	if (is_receiving || !is_radio_ready) {
		/* Channel busy or radio not command-ready — enforce retry timer
		 * so we don't hammer checks during RX activity or BUSY windows. */
		if (!millisHasNowPassed(next_tx_time)) {
			if (_tx_queued_cb) {
				uint32_t remaining = next_tx_time - now;
				_tx_queued_cb(remaining + 1, _tx_queued_user_data);
			}
			return;
		}
		if (cad_busy_start == 0) {
			cad_busy_start = now;
		}
		if (now - cad_busy_start > getCADFailMaxDuration()) {
			_err_flags |= ERR_EVENT_CAD_TIMEOUT;
			LOG_ERR("checkSend: CAD timeout exceeded (isReceiving=%d, isRadioReady=%d, inRecvMode=%d, rssi=%.1f, snr=%.1f, noise=%d, rx_ok=%u, rx_err=%u)",
				(int)is_receiving, (int)is_radio_ready, (int)_radio->isInRecvMode(),
				(double)_radio->getLastRSSI(), (double)_radio->getLastSNR(),
				_radio->getNoiseFloor(),
				(unsigned)_radio->getPacketsRecv(),
				(unsigned)_radio->getPacketsRecvErrors());
		} else {
			uint32_t retry = getCADFailRetryDelay();
			next_tx_time = futureMillis((int)retry);
			if (_tx_queued_cb) {
				_tx_queued_cb(retry + 1, _tx_queued_user_data);
			}
			return;
		}
	}
	cad_busy_start = 0;

	outbound = _mgr->getNextOutbound(now);
	if (outbound) {
		uint8_t raw[MAX_TRANS_UNIT];
		int len = 0;
		raw[len++] = outbound->header;
		if (outbound->hasTransportCodes()) {
			memcpy(&raw[len], &outbound->transport_codes[0], 2); len += 2;
			memcpy(&raw[len], &outbound->transport_codes[1], 2); len += 2;
		}
		raw[len++] = outbound->path_len;
		len += Packet::writePath(&raw[len], outbound->path, outbound->path_len);

		if (len + outbound->payload_len > MAX_TRANS_UNIT) {
			LOG_ERR("checkSend: packet too large len=%d+%d > %d", len, outbound->payload_len, MAX_TRANS_UNIT);
			_mgr->free(outbound);
			outbound = nullptr;
		} else {
			memcpy(&raw[len], outbound->payload, outbound->payload_len);
			len += outbound->payload_len;

			uint32_t max_airtime = _radio->getEstAirtimeFor(len) * 3 / 2;
			outbound_start = now;

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
			/* Arduino-compatible packet logging - use printk to bypass log level filtering */
			{
				uint8_t ptype = outbound->getPayloadType();
				if (ptype == PAYLOAD_TYPE_PATH || ptype == PAYLOAD_TYPE_REQ ||
				    ptype == PAYLOAD_TYPE_RESPONSE || ptype == PAYLOAD_TYPE_TXT_MSG) {
					printk("%s: TX, len=%d (type=%d, route=%s, payload_len=%d) [%02X -> %02X]\n",
						getLogDateTime(), len, ptype,
						outbound->isRouteDirect() ? "D" : "F", outbound->payload_len,
						(uint32_t)outbound->payload[1], (uint32_t)outbound->payload[0]);
				} else {
					printk("%s: TX, len=%d (type=%d, route=%s, payload_len=%d)\n",
						getLogDateTime(), len, ptype,
						outbound->isRouteDirect() ? "D" : "F", outbound->payload_len);
				}
			}
#endif

			/* Final gate — close the gap between initial checks and
			 * actual TX start (serialisation + logging can take 1-5 ms). */
			bool final_is_receiving = _radio->isReceiving();
			bool final_is_radio_ready = _radio->isRadioReady();
			if (final_is_receiving || !final_is_radio_ready) {
				uint32_t retry = getCADFailRetryDelay();
				LOG_DBG("checkSend: final gate blocked TX (isReceiving=%d, isRadioReady=%d, inRecvMode=%d)",
					(int)final_is_receiving, (int)final_is_radio_ready,
					(int)_radio->isInRecvMode());
				_mgr->queueOutbound(outbound, 0, futureMillis((int)retry));
				outbound = nullptr;
				if (_tx_queued_cb) {
					_tx_queued_cb(retry, _tx_queued_user_data);
				}
				return;
			}

			bool success = _radio->startSendRaw(raw, len);
			if (!success) {
				uint32_t retry = getCADFailRetryDelay();
				LOG_ERR("checkSend: startSendRaw failed! re-queuing delay=%u", retry);
				logTxFail(outbound, outbound->getRawLength());
				_mgr->queueOutbound(outbound, 0, futureMillis((int)retry));
				outbound = nullptr;
				if (_tx_queued_cb) {
					_tx_queued_cb(retry, _tx_queued_user_data);
				}
			} else {
				outbound_expiry = futureMillis((int)max_airtime);
			}
		}
	}
}

Packet *Dispatcher::obtainNewPacket()
{
	Packet *pkt = _mgr->allocNew();
	if (pkt == nullptr) {
		_err_flags |= ERR_EVENT_FULL;
	} else {
		pkt->payload_len = pkt->path_len = 0;
		pkt->_snr = 0;
	}
	return pkt;
}

void Dispatcher::releasePacket(Packet *packet)
{
	_mgr->free(packet);
}

void Dispatcher::sendPacket(Packet *packet, uint8_t priority, uint32_t delay_millis)
{
	if (!Packet::isValidPathLen(packet->path_len) || packet->payload_len > MAX_PACKET_PAYLOAD) {
		LOG_ERR("sendPacket: rejected - path_len=%d or payload_len=%d invalid",
			packet->path_len, packet->payload_len);
		_mgr->free(packet);
	} else {
		_mgr->queueOutbound(packet, priority, futureMillis((int)delay_millis));
		if (_tx_queued_cb && delay_millis > 0) {
			_tx_queued_cb(delay_millis, _tx_queued_user_data);
		}
	}
}

bool Dispatcher::millisHasNowPassed(uint32_t timestamp) const
{
	return (int32_t)((uint32_t)_ms->getMillis() - timestamp) > 0;
}

uint32_t Dispatcher::futureMillis(int millis_from_now) const
{
	return (uint32_t)_ms->getMillis() + millis_from_now;
}

} /* namespace mesh */
