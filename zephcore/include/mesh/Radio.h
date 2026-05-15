/*
 * SPDX-License-Identifier: Apache-2.0
 * ZephCore Radio interface - matches Dispatcher.h
 */

#pragma once

#include <stdint.h>

namespace mesh {

class Radio {
public:
	virtual void begin() {}

	virtual int recvRaw(uint8_t *bytes, int sz) = 0;
	virtual uint32_t getEstAirtimeFor(int len_bytes) = 0;
	virtual float packetScore(float snr, int packet_len) = 0;
	virtual bool startSendRaw(const uint8_t *bytes, int len) = 0;
	virtual bool isSendComplete() = 0;
	virtual void onSendFinished() = 0;

	virtual int getNoiseFloor() const { return 0; }
	virtual void triggerNoiseFloorCalibrate(int threshold) { (void)threshold; }
	virtual void resetAGC() {}

	virtual bool isInRecvMode() const = 0;
	virtual bool isReceiving() { return false; }
	virtual bool isRadioReady() { return true; }

	/* Reset the radio back into a known good RX state.  Called by the
	 * Dispatcher on CAD timeout (when isReceiving() pinned true past the
	 * recovery threshold).  Default no-op — radios that can stall should
	 * override to walk the chip through cancel → REST → fresh RX, which
	 * also bulk-clears IRQ status and any internal busy latches. */
	virtual void recoverRxState() {}
	virtual float getLastRSSI() const { return 0; }
	virtual float getLastSNR() const { return 0; }

	/* Adaptive Power Control */
	virtual void setTxPowerReduction(int8_t reduction_db) { (void)reduction_db; }
	virtual int8_t getTxPowerReduction() const { return 0; }

	/* Packet statistics */
	virtual uint32_t getPacketsRecv() const { return 0; }
	virtual uint32_t getPacketsSent() const { return 0; }
	virtual uint32_t getPacketsRecvErrors() const { return 0; }
};

} /* namespace mesh */
