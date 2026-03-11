/*
 * SPDX-License-Identifier: Apache-2.0
 * Static pool PacketManager
 */

#pragma once

#include <mesh/Dispatcher.h>

namespace mesh {

class StaticPoolPacketManager : public PacketManager {
public:
	Packet *allocNew() override;
	void free(Packet *packet) override;
	void queueOutbound(Packet *packet, uint8_t priority, uint32_t scheduled_for) override;
	Packet *getNextOutbound(uint32_t now) override;
	int getOutboundCount(uint32_t now) const override;
	int getOutboundTotal() const override;
	int getFreeCount() const override;
	Packet *getOutboundByIdx(int i) override;
	Packet *removeOutboundByIdx(int i) override;
	uint32_t getOutboundSchedule(int i) const override;
	bool rescheduleOutbound(int i, uint32_t new_scheduled_for) override;
	void queueInbound(Packet *packet, uint32_t scheduled_for) override;
	Packet *getNextInbound(uint32_t now) override;
};

} /* namespace mesh */
