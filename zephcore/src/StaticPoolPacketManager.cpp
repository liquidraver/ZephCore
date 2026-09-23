#include <mesh/StaticPoolPacketManager.h>
#include <string.h>

// ZEPHCORE: log module for the pool's warnings.
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_pktpool, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

// ZEPHCORE: static storage — no heap in the packet path. Upstream sizes the pool
// at runtime (new[]); here pool and queues are compile-time and file-local.
#define POOL_SIZE   32
#define QUEUE_SIZE  32

namespace {

struct PacketQueue {
  mesh::Packet* _table[QUEUE_SIZE];
  uint8_t _pri_table[QUEUE_SIZE];
  uint32_t _schedule_table[QUEUE_SIZE];
  int _num;

  PacketQueue() : _num(0) {
    memset(_table, 0, sizeof(_table));
  }

  int count() const { return _num; }
  mesh::Packet* itemAt(int i) const {
    if (i < 0 || i >= _num) return NULL;
    return _table[i];
  }

  int countBefore(uint32_t now) const {
    if (now == 0xFFFFFFFF) return _num;  // sentinel: count all entries regardless of schedule

    int n = 0;
    for (int j = 0; j < _num; j++) {
      if ((int32_t)(_schedule_table[j] - now) > 0) continue;   // scheduled for future... ignore for now
      n++;
    }
    return n;
  }

  mesh::Packet* get(uint32_t now) {
    uint8_t min_pri = 0xFF;
    int best_idx = -1;
    for (int j = 0; j < _num; j++) {
      if ((int32_t)(_schedule_table[j] - now) > 0) continue;   // scheduled for future... ignore for now
      if (_pri_table[j] < min_pri) {  // select most important priority amongst non-future entries
        min_pri = _pri_table[j];
        best_idx = j;
      }
    }
    if (best_idx < 0) return NULL;   // empty, or all items are still in the future

    return removeByIdx(best_idx);
  }

  mesh::Packet* removeByIdx(int i) {
    if (i < 0 || i >= _num) return NULL;  // invalid index

    mesh::Packet* item = _table[i];
    _num--;
    while (i < _num) {
      _table[i] = _table[i+1];
      _pri_table[i] = _pri_table[i+1];
      _schedule_table[i] = _schedule_table[i+1];
      i++;
    }
    return item;
  }

  bool add(mesh::Packet* packet, uint8_t priority, uint32_t scheduled_for) {
    if (_num == QUEUE_SIZE) {
      return false;
    }
    _table[_num] = packet;
    _pri_table[_num] = priority;
    _schedule_table[_num] = scheduled_for;
    _num++;
    return true;
  }

  // ZEPHCORE: eviction and schedule access (reactive backoff, re-queue priority).
  int findLowestPriority() const {   // least important entry, -1 when empty
    if (_num == 0) return -1;
    uint8_t worst = 0;
    int idx = 0;
    for (int j = 0; j < _num; j++) {
      if (_pri_table[j] >= worst) {
        worst = _pri_table[j];
        idx = j;
      }
    }
    return idx;
  }
  uint32_t scheduleAt(int i) const { return (i < _num) ? _schedule_table[i] : 0; }
  uint8_t peekPriority(uint32_t now) const {   // what get(now) would return; 0xFF if none due
    uint8_t best = 0xFF;
    for (int j = 0; j < _num; j++) {
      if ((int32_t)(_schedule_table[j] - now) > 0) continue;
      if (_pri_table[j] < best) best = _pri_table[j];
    }
    return best;
  }
  bool reschedule(int i, uint32_t new_scheduled_for) {
    if (i >= _num) return false;
    _schedule_table[i] = new_scheduled_for;
    return true;
  }
};

mesh::Packet packet_pool[POOL_SIZE];
PacketQueue unused, send_queue;
bool initialized = false;

// load up our unusued Packet pool (on first use: no static-init ordering hazard)
void init_pool() {
  if (initialized) return;
  initialized = true;
  for (int i = 0; i < POOL_SIZE; i++) {
    unused.add(&packet_pool[i], 0, 0);
  }
}

}  // namespace

mesh::Packet* StaticPoolPacketManager::allocNew() {
  init_pool();  // ZEPHCORE
  return unused.removeByIdx(0);  // just get first one (returns NULL if empty)
}

void StaticPoolPacketManager::free(mesh::Packet* packet) {
  if (packet == NULL) return;
  if (!unused.add(packet, 0, 0)) {
    LOG_WRN("free: unused queue full, possible double-free");  // ZEPHCORE: WRN, not debug-only
  }
}

void StaticPoolPacketManager::queueOutbound(mesh::Packet* packet, uint8_t priority, uint32_t scheduled_for) {
  if (packet == NULL) return;
  if (send_queue.add(packet, priority, scheduled_for)) return;

  // ZEPHCORE: queue full — evict the least important entry if the new packet is
  // more important (lower number); otherwise drop the new one (as upstream).
  int worst = send_queue.findLowestPriority();
  if (worst >= 0 && send_queue._pri_table[worst] > priority) {
    uint8_t evicted_pri = send_queue._pri_table[worst];
    mesh::Packet* evicted = send_queue.removeByIdx(worst);
    LOG_WRN("queueOutbound: FULL — evicted type=%d pri=%d for type=%d pri=%d",
            evicted->getPayloadType(), evicted_pri, packet->getPayloadType(), priority);
    free(evicted);
    send_queue.add(packet, priority, scheduled_for);
  } else {
    LOG_WRN("queueOutbound: FULL (%d entries) — dropping type=%d pri=%d",
            send_queue.count(), packet->getPayloadType(), priority);
    free(packet);
  }
}

mesh::Packet* StaticPoolPacketManager::getNextOutbound(uint32_t now) {
  return send_queue.get(now);
}

int  StaticPoolPacketManager::getOutboundCount(uint32_t now) const {
  return send_queue.countBefore(now);
}

int  StaticPoolPacketManager::getOutboundTotal() const {
  return send_queue.count();
}

int StaticPoolPacketManager::getFreeCount() const {
  return unused.count();
}

mesh::Packet* StaticPoolPacketManager::getOutboundByIdx(int i) {
  return send_queue.itemAt(i);
}
mesh::Packet* StaticPoolPacketManager::removeOutboundByIdx(int i) {
  return send_queue.removeByIdx(i);
}

// ZEPHCORE: schedule access instead of the inbound queue.
uint32_t StaticPoolPacketManager::getOutboundSchedule(int i) const {
  return send_queue.scheduleAt(i);
}
bool StaticPoolPacketManager::rescheduleOutbound(int i, uint32_t new_scheduled_for) {
  return send_queue.reschedule(i, new_scheduled_for);
}
uint8_t StaticPoolPacketManager::peekNextOutboundPriority(uint32_t now) const {
  return send_queue.peekPriority(now);
}
