/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Dispatcher implementation
 */

#include <mesh/Dispatcher.h>
#include <mesh/MeshCore.h>
#include <mesh/Utils.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_dispatcher, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZEPHCORE_PACKET_LOGGING)
/* Payload types for [src -> dest] logging */
#define PAYLOAD_TYPE_REQ         0x00
#define PAYLOAD_TYPE_RESPONSE    0x01
#define PAYLOAD_TYPE_TXT_MSG     0x02
#define PAYLOAD_TYPE_PATH        0x08
#endif

namespace mesh {

#define MAX_RX_DELAY_MILLIS   32000

Dispatcher::Dispatcher(Radio &radio, MillisecondClock &ms, PacketManager &mgr)
	: _radio(&radio), _ms(&ms), _mgr(&mgr)
{
	outbound = nullptr;
	total_air_time = rx_air_time = 0;
	next_tx_time = 0;
	cad_busy_start = 0;
	next_agc_reset_time = 0;
	_err_flags = 0;
	_duty_cycle.init(0);
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
	radio_nonrx_start = (uint32_t)_ms->getMillis();
	_radio->begin();
	prev_isrecv_mode = _radio->isInRecvMode();
	_duty_cycle.init(getDutyCyclePercent());
}

uint8_t Dispatcher::getDutyCyclePercent() const
{
	return 10;
}

bool Dispatcher::isAdminPacket(const Packet *pkt)
{
	uint8_t t = pkt->getPayloadType();
	return t == PAYLOAD_TYPE_REQ || t == PAYLOAD_TYPE_RESPONSE ||
	       t == PAYLOAD_TYPE_ANON_REQ || t == PAYLOAD_TYPE_CONTROL;
}

int Dispatcher::calcRxDelay(float score, uint32_t air_time) const
{
	/* Lookup table: (10^(0.85 - i*0.1) - 1) for i=0..10 (score 0.0 to 1.0)
	 * Replaces powf() to save ~1.9KB flash. Linear interpolation between entries. */
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
	return 200;
}

uint32_t Dispatcher::getCADFailMaxDuration() const
{
	return 4000;
}

void Dispatcher::loop()
{
	if (outbound) {
		if (_radio->isSendComplete()) {
			uint32_t t = (uint32_t)_ms->getMillis() - outbound_start;
			total_air_time += t;
			_duty_cycle.recordTx(t, (uint32_t)_ms->getMillis());
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
	/* Noise floor calibration — one EMA tick per housekeeping cycle */
	_radio->triggerNoiseFloorCalibrate(getInterferenceThreshold());

	/* RX mode watchdog — detect if radio is stuck outside RX */
	bool is_recv = _radio->isInRecvMode();
	if (is_recv != prev_isrecv_mode) {
		prev_isrecv_mode = is_recv;
		if (!is_recv) {
			radio_nonrx_start = (uint32_t)_ms->getMillis();
		}
	}
	if (!is_recv && (uint32_t)_ms->getMillis() - radio_nonrx_start > 8000) {
		_err_flags |= ERR_EVENT_STARTRX_TIMEOUT;
	}

	/* AGC reset — periodic warm sleep + recalibration */
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
	if (path_mode == 3) {   // Reserved for future
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
	/* Drain ALL queued LoRa packets per wake.
	 * k_event is a bitfield (not a counter), so multiple ISR arrivals
	 * may only produce one wake.  We must empty the ring each time. */
	for (;;) {
		uint8_t raw[MAX_TRANS_UNIT + 1];
		int len = _radio->recvRaw(raw, MAX_TRANS_UNIT);
		if (len <= 0) {
			break;  /* ring empty — done */
		}

		logRxRaw(_radio->getLastSNR(), _radio->getLastRSSI(), raw, len);

		Packet *pkt = _mgr->allocNew();
		if (pkt == nullptr) {
			LOG_WRN("checkRecv: packet alloc failed");
			break;
		}

		float score = 0.0f;
		uint32_t air_time = 0;

		if (tryParsePacket(pkt, raw, len)) {
			pkt->_snr = (int8_t)(_radio->getLastSNR() * 4.0f);
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

	if (_radio->isReceiving()) {
		/* Channel busy — enforce retry timer so we don't hammer the check */
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
			LOG_WRN("checkSend: CAD timeout exceeded");
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
		/* Duty cycle enforcement — exempt admin packets */
		if (!isAdminPacket(outbound) && _duty_cycle.isExceeded(now)) {
			LOG_WRN("checkSend: duty cycle exceeded (%u/%u ms), re-queuing type=%d",
				_duty_cycle.window_airtime_ms, _duty_cycle.budgetMs(),
				outbound->getPayloadType());
			_mgr->queueOutbound(outbound, 0, futureMillis(5000));
			outbound = nullptr;
			if (_tx_queued_cb) {
				_tx_queued_cb(5000, _tx_queued_user_data);
			}
			return;
		}
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
			LOG_WRN("checkSend: packet too large len=%d+%d > %d", len, outbound->payload_len, MAX_TRANS_UNIT);
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

			/* Final LBT check — close the gap between initial
			 * isReceiving() and actual TX start (serialisation +
			 * logging can take 1-5 ms). */
			if (_radio->isReceiving()) {
				uint32_t retry = getCADFailRetryDelay();
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
		LOG_WRN("sendPacket: rejected - path_len=%d or payload_len=%d invalid",
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
