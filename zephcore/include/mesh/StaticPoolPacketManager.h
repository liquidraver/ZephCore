#pragma once

#include <mesh/Dispatcher.h>

// ZEPHCORE: the queues and the pool are static and live in the .cpp — no heap in
// the packet path. Pool and queue sizes are compile-time (32 each).

class StaticPoolPacketManager : public mesh::PacketManager {
public:

  mesh::Packet* allocNew() override;
  void free(mesh::Packet* packet) override;
  void queueOutbound(mesh::Packet* packet, uint8_t priority, uint32_t scheduled_for) override;
  mesh::Packet* getNextOutbound(uint32_t now) override;
  int getOutboundCount(uint32_t now) const override;
  int getOutboundTotal() const override;
  int getFreeCount() const override;
  mesh::Packet* getOutboundByIdx(int i) override;
  mesh::Packet* removeOutboundByIdx(int i) override;
  // ZEPHCORE: no inbound queue (see mesh::PacketManager); schedule access instead.
  uint32_t getOutboundSchedule(int i) const override;
  bool rescheduleOutbound(int i, uint32_t new_scheduled_for) override;
  uint8_t peekNextOutboundPriority(uint32_t now) const override;
};
