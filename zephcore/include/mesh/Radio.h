/*
 * SPDX-License-Identifier: MIT
 * ZephCore Radio interface - matches Dispatcher.h
 */

#pragma once

#include <stdint.h>
#include <mesh/Maintenance.h>

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

	/* Adaptive CAD (LBT detPeak calibration).  Default no-ops for radios
	 * without hardware CAD (SX127x). */
	virtual void setCadParams(bool auto_enabled, int8_t offset,
				  uint16_t probe_interval_s, uint8_t busycap_pct) {
		(void)auto_enabled; (void)offset; (void)probe_interval_s;
		(void)busycap_pct;
	}
	/* One housekeeping tick of the CAD calibrator: maybe run a probe,
	 * update stats, maybe step the staircase (auto mode). */
	virtual void cadMaintenance() {}

	/* Milliseconds until this radio's periodic work (noise floor sampling,
	 * CAD probing/decay) next needs a call, or MAINTENANCE_IDLE when it has
	 * nothing pending.  Radios with no periodic work keep the default. */
	virtual uint32_t msUntilNextMaintenance() { return MAINTENANCE_IDLE; }
	virtual int8_t getCadOffset() const { return 0; }
	virtual void resetCadStats() {}
	/* Writes a human-readable status block; returns chars written (0 = not
	 * supported by this radio). */
	virtual int formatCadStatus(char *buf, int cap) {
		(void)buf; (void)cap; return 0;
	}

	/* Packet statistics */
	virtual uint32_t getPacketsRecv() const { return 0; }
	virtual uint32_t getPacketsSent() const { return 0; }
	virtual uint32_t getPacketsRecvErrors() const { return 0; }
};

} /* namespace mesh */
