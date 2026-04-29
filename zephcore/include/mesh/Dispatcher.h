/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Dispatcher - packet queue and radio scheduling
 */

#pragma once

#include <mesh/MeshCore.h>
#include <mesh/Identity.h>
#include <mesh/Packet.h>
#include <mesh/Utils.h>
#include <mesh/Radio.h>
#include <mesh/Clock.h>
#include <string.h>

namespace mesh {

class PacketManager {
public:
	virtual Packet *allocNew() = 0;
	virtual void free(Packet *packet) = 0;
	virtual void queueOutbound(Packet *packet, uint8_t priority, uint32_t scheduled_for) = 0;
	virtual Packet *getNextOutbound(uint32_t now) = 0;
	virtual int getOutboundCount(uint32_t now) const = 0;
	virtual int getOutboundTotal() const = 0;
	virtual int getFreeCount() const = 0;
	virtual Packet *getOutboundByIdx(int i) = 0;
	virtual Packet *removeOutboundByIdx(int i) = 0;
	virtual uint32_t getOutboundSchedule(int i) const = 0;
	virtual bool rescheduleOutbound(int i, uint32_t new_scheduled_for) = 0;
	virtual void queueInbound(Packet *packet, uint32_t scheduled_for) = 0;
	virtual Packet *getNextInbound(uint32_t now) = 0;
};

/* Notifies event loop of pending TX so it can schedule a wake. */
typedef void (*tx_queued_callback_t)(uint32_t delay_ms, void *user_data);

typedef uint32_t DispatcherAction;

#define ACTION_RELEASE           (0)
#define ACTION_MANUAL_HOLD       (1)
#define ACTION_RETRANSMIT(pri)   (((uint32_t)1 + (pri))<<24)
#define ACTION_RETRANSMIT_DELAYED(pri, _delay)  ((((uint32_t)1 + (pri))<<24) | (_delay))

#define ERR_EVENT_FULL              (1 << 0)
#define ERR_EVENT_CAD_TIMEOUT       (1 << 1)
#define ERR_EVENT_STARTRX_TIMEOUT   (1 << 2)

class Dispatcher {
	Packet *outbound;
	uint32_t outbound_expiry, outbound_start, total_air_time, rx_air_time;
	uint32_t next_tx_time;
	uint32_t cad_busy_start;
	uint32_t tx_budget_ms;
	uint32_t last_budget_update;
	uint32_t duty_cycle_window_ms;
	uint32_t radio_nonrx_start;
	uint32_t next_agc_reset_time;
	bool prev_isrecv_mode;
	uint32_t n_sent_flood, n_sent_direct;
	uint32_t n_recv_flood, n_recv_direct;
	tx_queued_callback_t _tx_queued_cb;
	void *_tx_queued_user_data;

	void processRecvPacket(Packet *pkt);

protected:
	Radio *_radio;
	MillisecondClock *_ms;
	PacketManager *_mgr;
	uint16_t _err_flags;

	Dispatcher(Radio &radio, MillisecondClock &ms, PacketManager &mgr);
	void notifyTxQueued(uint32_t delay_ms) {
		if (_tx_queued_cb) _tx_queued_cb(delay_ms, _tx_queued_user_data);
	}
	virtual DispatcherAction onRecvPacket(Packet *pkt) = 0;
	virtual void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) { (void)snr; (void)rssi; (void)raw; (void)len; }
	virtual void logRx(Packet *packet, int len, float score) { (void)packet; (void)len; (void)score; }
	virtual void logTx(Packet *packet, int len) { (void)packet; (void)len; }
	virtual void logTxFail(Packet *packet, int len) { (void)packet; (void)len; }
	virtual const char *getLogDateTime() { return ""; }
	virtual uint8_t getDutyCyclePercent() const;
	static bool isAdminPacket(const Packet *pkt);
	virtual int calcRxDelay(float score, uint32_t air_time) const;
	virtual uint32_t getCADFailRetryDelay() const;
	virtual uint32_t getCADFailMaxDuration() const;
	virtual int getInterferenceThreshold() const { return 0; }
	virtual int getAGCResetInterval() const { return 0; }
	virtual uint32_t getDutyCycleWindowMs() const { return 3600000UL; } /* 1h default */

public:
	void begin();
	void loop();
	void maintenanceLoop();
	Packet *obtainNewPacket();
	void releasePacket(Packet *packet);
	void sendPacket(Packet *packet, uint8_t priority, uint32_t delay_millis = 0);

	uint32_t getTotalAirTime() const { return total_air_time; }
	uint32_t getReceiveAirTime() const { return rx_air_time; }
	uint32_t getNumSentFlood() const { return n_sent_flood; }
	uint32_t getNumSentDirect() const { return n_sent_direct; }
	uint32_t getNumRecvFlood() const { return n_recv_flood; }
	uint32_t getNumRecvDirect() const { return n_recv_direct; }
	uint16_t getErrFlags() const { return _err_flags; }
	void resetStats() {
		n_sent_flood = n_sent_direct = 0;
		n_recv_flood = n_recv_direct = 0;
		_err_flags = 0;
	}
	void setTxQueuedCallback(tx_queued_callback_t cb, void *user_data) {
		_tx_queued_cb = cb;
		_tx_queued_user_data = user_data;
	}
	bool millisHasNowPassed(uint32_t timestamp) const;
	uint32_t futureMillis(int millis_from_now) const;

private:
	void updateTxBudget();
	uint32_t getMaxTxBudgetMs() const;
	bool tryParsePacket(Packet *pkt, const uint8_t *raw, int len);
	void checkRecv();
	void checkSend();
};

} /* namespace mesh */
