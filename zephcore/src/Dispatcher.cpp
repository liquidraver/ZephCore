#include <mesh/Dispatcher.h>

// ZEPHCORE: packet logging (MESH_PACKET_LOGGING) lives in the roles' logRx/logTx
// hooks, see helpers/PacketLog.h.

#include <math.h>

// ZEPHCORE: log module backing MESH_DEBUG_PRINTLN (see MeshCore.h).
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_dispatcher, CONFIG_ZEPHCORE_LORA_LOG_LEVEL);

namespace mesh {

#define MIN_TX_BUDGET_AIRTIME_DIV  2      // require at least 1/N of estimated airtime as budget before TX

// ZEPHCORE: no MAX_RX_DELAY_MILLIS (no score-based RX delay: the adaptive
// contention window replaces it), no MIN_TX_BUDGET_RESERVE_MS (the budget gate
// lives in checkSend() only), no NOISE_FLOOR_CALIB_INTERVAL (the radio schedules
// its own sampling, see maintenanceLoop()).

// ZEPHCORE: out-of-line constructor (upstream's is inline in the header).
Dispatcher::Dispatcher(Radio& radio, MillisecondClock& ms, PacketManager& mgr)
  : _radio(&radio), _ms(&ms), _mgr(&mgr)
{
  outbound = NULL;
  outbound_priority = 0;
  total_air_time = rx_air_time = 0;
  next_tx_time = 0;
  cad_busy_start = 0;
  lbt_busy_start = 0;
  lbt_next_warn = 0;
  tx_budget_ms = 0;
  last_budget_update = 0;
  duty_cycle_window_ms = 0;
  _err_flags = 0;
  radio_nonrx_start = 0;
  prev_isrecv_mode = true;
  cad_offset_shadow = 0;
  cad_offset_shadow_valid = false;
  n_sent_flood = n_sent_direct = 0;
  n_recv_flood = n_recv_direct = 0;
  _tx_queued_cb = NULL;
  _tx_queued_user_data = NULL;
  _wake_cb = NULL;
  _wake_user_data = NULL;
}

void Dispatcher::begin() {
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

// ZEPHCORE: duty cycle as a percentage (upstream: getAirtimeBudgetFactor()).
uint8_t Dispatcher::getDutyCyclePercent() const {
  return 10;   // EU 868 default
}

uint32_t Dispatcher::getMaxTxBudgetMs() const {
  uint8_t duty_pct = getDutyCyclePercent();
  if (duty_pct == 0 || duty_cycle_window_ms == 0) {
    return 0;
  }
  return (duty_cycle_window_ms * (uint32_t)duty_pct) / 100U;
}

void Dispatcher::updateTxBudget() {
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

// ZEPHCORE: no calcRxDelay() (see top of file).

uint32_t Dispatcher::getCADFailRetryDelay() const {
  return 200;
}
uint32_t Dispatcher::getCADFailMaxDuration() const {
  return 4000;   // 4 seconds
}

// ZEPHCORE: LBT starvation window (see onLbtRefused()) and the admin exemption
// from the duty-cycle gate (see txBudgetAllowsSend()).
uint32_t Dispatcher::getTxStarvationDuration() const {
  return 60000;
}

bool Dispatcher::isAdminPacket(const Packet* pkt) {
  uint8_t t = pkt->getPayloadType();
  return t == PAYLOAD_TYPE_REQ || t == PAYLOAD_TYPE_RESPONSE ||
         t == PAYLOAD_TYPE_ANON_REQ || t == PAYLOAD_TYPE_CONTROL;
}

void Dispatcher::loop() {
  // ZEPHCORE: noise-floor sampling, the radio stall watchdog and AGC hygiene run
  // from maintenanceLoop() on their own deadlines, not on every loop() pass.

  if (outbound) {  // waiting for outbound send to be completed
    // ZEPHCORE: read isTxActive() BEFORE isSendComplete(): the radio publishes
    // completion before it drops isTxActive(), so "not active" here guarantees a
    // finished transmit is already latched and collected below.
    bool tx_active = _radio->isTxActive();
    if (_radio->isSendComplete()) {
      // ZEPHCORE: charge the COMPUTED airtime, not the wall-clock send time,
      // which includes blocking LBT and event latency (up to 8x in the field).
      uint32_t t = _radio->getEstAirtimeFor(outbound->getRawLength());
      MESH_DEBUG_PRINTLN("TX complete: air=%ums wall=%ums", t, (uint32_t)_ms->getMillis() - outbound_start);
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
      releasePacket(outbound);  // return to pool
      outbound = NULL;
    } else if (!tx_active && millisHasNowPassed(outbound_expiry)) {  // ZEPHCORE: only once the radio has concluded
      MESH_DEBUG_PRINTLN("%s Dispatcher::loop(): WARNING: outbound packed send timed out!", getLogDateTime());

      _radio->onSendFinished();
      logTxFail(outbound, 2 + outbound->getPathByteLen() + outbound->payload_len);

      releasePacket(outbound);  // return to pool
      outbound = NULL;
    } else {
      return;  // can't do any more radio activity until send is complete or timed out
    }
  }

  // ZEPHCORE: no inbound (delayed) queue.
  checkRecv();
  checkSend();
}

// ZEPHCORE: deadline-driven maintenance. The event loop calls
// msUntilNextMaintenance() after every pass and sleeps until the soonest deadline.
void Dispatcher::maintenanceLoop() {
  _radio->triggerNoiseFloorCalibrate(getInterferenceThreshold());

  // Radio stall watchdog (diagnostic only: raises ERR_EVENT_STARTRX_TIMEOUT,
  // surfaced in stats, telemetry, MQTT and the companion status frame). TX
  // counts as active; isTxActive(), never the consuming isSendComplete().
  bool is_active = _radio->isInRecvMode() || _radio->isTxActive();
  if (is_active != prev_isrecv_mode) {
    prev_isrecv_mode = is_active;
    if (!is_active) {
      radio_nonrx_start = (uint32_t)_ms->getMillis();
    }
  }
  if (!is_active && (uint32_t)_ms->getMillis() - radio_nonrx_start > RADIO_STALL_THRESHOLD_MS) {
    _err_flags |= ERR_EVENT_STARTRX_TIMEOUT;
  }

  _radio->cadMaintenance();    // adaptive CAD probing + staircase
  _radio->radioMaintenance();  // AGC unstick + temperature-drift recal (sleeps the chip)

  // Surface CAD offset changes so the role persists them.
  int8_t cad_off = _radio->getCadOffset();
  if (!cad_offset_shadow_valid) {
    cad_offset_shadow = cad_off;
    cad_offset_shadow_valid = true;
  } else if (cad_off != cad_offset_shadow) {
    cad_offset_shadow = cad_off;
    onCadOffsetChanged(cad_off);
  }
}

uint32_t Dispatcher::msUntilNextMaintenance() {
  uint32_t now = (uint32_t)_ms->getMillis();
  uint32_t next = _radio->msUntilNextMaintenance();

  // The stall verdict is a latched bit; once raised its deadline is permanently
  // in the past, so stop reporting it or the loop would spin at its minimum.
  if (!prev_isrecv_mode && !(_err_flags & ERR_EVENT_STARTRX_TIMEOUT)) {
    next = maintenanceSooner(next, maintenanceUntil(now, radio_nonrx_start + RADIO_STALL_THRESHOLD_MS));
  }
  return next;
}

bool Dispatcher::tryParsePacket(Packet* pkt, const uint8_t* raw, int len) {
  if (len < 2 || len > MAX_TRANS_UNIT) return false;  // ZEPHCORE: bound the extent before reading
  int i = 0;

  pkt->header = raw[i++];
  if (pkt->getPayloadVer() > PAYLOAD_VER_1) {
    MESH_DEBUG_PRINTLN("%s Dispatcher::checkRecv(): unsupported packet version", getLogDateTime());
    return false;
  }

  if (pkt->hasTransportCodes()) {
    if (len < 6) return false;  // ZEPHCORE: header + two codes + path length
    memcpy(&pkt->transport_codes[0], &raw[i], 2); i += 2;
    memcpy(&pkt->transport_codes[1], &raw[i], 2); i += 2;
  } else {
    pkt->transport_codes[0] = pkt->transport_codes[1] = 0;
  }

  pkt->path_len = raw[i++];
  uint8_t path_mode = pkt->path_len >> 6;  // upper 2 bits (legacy firmware: 00)
  if (path_mode == 3) {   // Reserved for future
    MESH_DEBUG_PRINTLN("%s Dispatcher::checkRecv(): unsupported path mode: 3", getLogDateTime());
    return false;
  }

  uint8_t path_byte_len = (pkt->path_len & 63) * pkt->getPathHashSize();
  if (path_byte_len > MAX_PATH_SIZE || i + path_byte_len > len) {
    MESH_DEBUG_PRINTLN("%s Dispatcher::checkRecv(): partial or corrupt packet received, len=%d", getLogDateTime(), len);
    return false;
  }

  memcpy(pkt->path, &raw[i], path_byte_len); i += path_byte_len;

  pkt->payload_len = len - i;  // payload is remainder
  if (pkt->payload_len > sizeof(pkt->payload)) {
    MESH_DEBUG_PRINTLN("%s Dispatcher::checkRecv(): packet payload too big, payload_len=%d", getLogDateTime(), (uint32_t)pkt->payload_len);
    return false;
  }

  memcpy(pkt->payload, &raw[i], pkt->payload_len);

  return true;  // success
}

void Dispatcher::checkRecv() {
  // ZEPHCORE: drain the whole RX ring per call. The mesh thread wakes on a
  // k_event bit, and several arrivals coalesce into one wake.
  for (;;) {
    Packet* pkt;
    float score;
    uint32_t air_time;
    {
      uint8_t raw[MAX_TRANS_UNIT+1];
      int len = _radio->recvRaw(raw, MAX_TRANS_UNIT);
      if (len <= 0) {
        break;  // ring empty
      }
      logRxRaw(_radio->getLastSNR(), _radio->getLastRSSI(), raw, len);

      pkt = _mgr->allocNew();
      if (pkt == NULL) {
        MESH_DEBUG_PRINTLN("%s Dispatcher::checkRecv(): WARNING: received data, no unused packets available!", getLogDateTime());
        break;
      }
      if (tryParsePacket(pkt, raw, len)) {
        pkt->_snr = (int8_t)(_radio->getLastSNR() * 4.0f);
        score = _radio->packetScore(_radio->getLastSNR(), len);
        air_time = _radio->getEstAirtimeFor(len);
        rx_air_time += air_time;
      } else {
        _mgr->free(pkt);  // put back into pool
        continue;
      }
    }
    logRx(pkt, pkt->getRawLength(), score);   // hook for custom logging

    if (pkt->isRouteFlood()) {
      n_recv_flood++;
    } else {
      n_recv_direct++;
    }
    processRecvPacket(pkt);  // ZEPHCORE: always immediately (no score-based RX delay)
  }
}

void Dispatcher::processRecvPacket(Packet* pkt) {
  DispatcherAction action = onRecvPacket(pkt);
  if (action == ACTION_RELEASE) {
    _mgr->free(pkt);
  } else if (action == ACTION_MANUAL_HOLD) {
    // sub-class is wanting to manually hold Packet instance, and call releasePacket() at appropriate time
  } else {   // ACTION_RETRANSMIT*
    uint8_t priority = (action >> 24) - 1;
    uint32_t _delay = action & 0xFFFFFF;

    _mgr->queueOutbound(pkt, priority, futureMillis(_delay));
    if (_delay > 0) notifyTxQueued(_delay);  // ZEPHCORE: wake for the deferred TX
  }
}

// ZEPHCORE: duty-cycle budget gate. Defers while the remaining budget is under
// half an MTU's airtime (as upstream) and schedules the wake for when it will be
// enough. Due admin packets (REQ/RESPONSE/ANON_REQ/CONTROL) are exempt so a node
// that burned its budget stays reachable for management — strictly outside
// EN 300 220, accepted: an unreachable managed node is worse.
bool Dispatcher::txBudgetAllowsSend(uint32_t now) {
  uint8_t duty_pct = getDutyCyclePercent();
  if (duty_pct == 0) return true;

  int total = _mgr->getOutboundTotal();
  for (int i = 0; i < total; i++) {
    Packet* pkt = _mgr->getOutboundByIdx(i);
    if (pkt && (int32_t)(_mgr->getOutboundSchedule(i) - now) <= 0 && isAdminPacket(pkt)) {
      return true;
    }
  }

  uint32_t est_airtime = _radio->getEstAirtimeFor(MAX_TRANS_UNIT);
  uint32_t threshold = est_airtime / MIN_TX_BUDGET_AIRTIME_DIV;
  if (tx_budget_ms >= threshold) return true;

  uint32_t needed = threshold - tx_budget_ms;
  uint32_t delay_ms = (needed * 100U + (uint32_t)duty_pct - 1U) / (uint32_t)duty_pct;
  notifyTxQueued(delay_ms + 1U);
  return false;
}

// ZEPHCORE: the driver's LBT refused the transmit (a TRUE reading of a busy
// channel: re-queue, never force). Warn every getCADFailMaxDuration(); after
// getTxStarvationDuration() of nothing but refusals the detector is judged too
// sensitive and relaxed one step, regardless of cad.auto (persisted via
// onCadOffsetChanged()). Any successful transmit ends the streak.
void Dispatcher::onLbtRefused(uint32_t now, int len) {
  uint32_t retry = getCADFailRetryDelay();
  LOG_INF("checkSend: startSendRaw refused (LBT busy), re-queuing delay=%u", retry);

  if (lbt_busy_start == 0) {
    lbt_busy_start = now;
    lbt_next_warn = now + getCADFailMaxDuration();
  } else {
    uint32_t streak = now - lbt_busy_start;

    if ((int32_t)(now - lbt_next_warn) >= 0) {
      _err_flags |= ERR_EVENT_CAD_TIMEOUT;
      LOG_WRN("checkSend: LBT has refused TX for %ums (len=%d, noise=%d) — channel busy or CAD too sensitive",
              (unsigned)streak, len, _radio->getNoiseFloor());
      lbt_next_warn = now + getCADFailMaxDuration();
    }
    if (streak > getTxStarvationDuration()) {
      if (_radio->cadRelaxOnTxStarvation()) {
        LOG_ERR("checkSend: TX starved %ums — relaxed CAD one step", (unsigned)streak);
      } else {
        LOG_ERR("checkSend: TX starved %ums — CAD already at its least sensitive step, channel may be genuinely busy",
                (unsigned)streak);
      }
      lbt_busy_start = now;
      lbt_next_warn = now + getCADFailMaxDuration();
    }
  }
  logTxFail(outbound, outbound->getRawLength());
  _mgr->queueOutbound(outbound, outbound_priority, futureMillis((int)retry));
  outbound = NULL;
  notifyTxQueued(retry);
}

void Dispatcher::checkSend() {
  uint32_t now = (uint32_t)_ms->getMillis();
  if (_mgr->getOutboundCount(now) == 0) {
    cad_busy_start = 0;
    // ZEPHCORE: only a genuinely EMPTY queue ends an LBT streak — a refusal
    // re-queues 100-200 ms ahead, so "nothing due now" is not "nothing pending".
    if (_mgr->getOutboundTotal() == 0) lbt_busy_start = 0;
    return;
  }

  updateTxBudget();

  if (!txBudgetAllowsSend(now)) return;  // ZEPHCORE: percentage budget, admin exempt

  // ZEPHCORE: also defer while the radio is not command-ready, and apply the
  // retry timer only to a busy channel (upstream checks it first).
  bool is_receiving = _radio->isReceiving();
  bool is_radio_ready = _radio->isRadioReady();
  if (is_receiving || !is_radio_ready) {
    if (!millisHasNowPassed(next_tx_time)) {
      notifyTxQueued(next_tx_time - now + 1);
      return;
    }
    if (cad_busy_start == 0) {
      cad_busy_start = now;   // record when CAD busy state started
    }

    if (now - cad_busy_start > getCADFailMaxDuration()) {
      _err_flags |= ERR_EVENT_CAD_TIMEOUT;

      LOG_ERR("checkSend: CAD timeout exceeded (isReceiving=%d, isRadioReady=%d, inRecvMode=%d, rssi=%.1f, snr=%.1f, noise=%d, rx_ok=%u, rx_err=%u)",
              (int)is_receiving, (int)is_radio_ready, (int)_radio->isInRecvMode(),
              (double)_radio->getLastRSSI(), (double)_radio->getLastSNR(), _radio->getNoiseFloor(),
              (unsigned)_radio->getPacketsRecv(), (unsigned)_radio->getPacketsRecvErrors());
      // channel activity has gone on too long... (Radio might be in a bad state)
      // force the pending transmit below...
      _radio->recoverRxState();  // ZEPHCORE: walk the chip REST -> fresh RX first
    } else {
      uint32_t retry = getCADFailRetryDelay();
      next_tx_time = futureMillis(retry);
      notifyTxQueued(retry + 1);  // ZEPHCORE: schedule the wake
      return;
    }
  }
  cad_busy_start = 0;  // reset busy state

  outbound_priority = _mgr->peekNextOutboundPriority(now);  // ZEPHCORE: kept for a re-queue
  outbound = _mgr->getNextOutbound(now);
  if (outbound) {
    int len = 0;
    uint8_t raw[MAX_TRANS_UNIT];

    raw[len++] = outbound->header;
    if (outbound->hasTransportCodes()) {
      memcpy(&raw[len], &outbound->transport_codes[0], 2); len += 2;
      memcpy(&raw[len], &outbound->transport_codes[1], 2); len += 2;
    }
    raw[len++] = outbound->path_len;
    len += Packet::writePath(&raw[len], outbound->path, outbound->path_len);

    if (len + outbound->payload_len > MAX_TRANS_UNIT) {
      MESH_DEBUG_PRINTLN("%s Dispatcher::checkSend(): FATAL: Invalid packet queued... too long, len=%d", getLogDateTime(), len + outbound->payload_len);
      _mgr->free(outbound);
      outbound = NULL;
    } else {
      memcpy(&raw[len], outbound->payload, outbound->payload_len); len += outbound->payload_len;

      uint32_t max_airtime = _radio->getEstAirtimeFor(len)*3/2;
      if (max_airtime < 300) max_airtime = 300;  // ZEPHCORE: IRQ/work-queue latency must not clip short packets
      outbound_start = now;

      // ZEPHCORE: final gate, closing the gap since the checks above. Also
      // covers a radio that published its completion but is still re-arming RX.
      if (_radio->isReceiving() || !_radio->isRadioReady() || _radio->isTxActive()) {
        uint32_t retry = getCADFailRetryDelay();
        _mgr->queueOutbound(outbound, outbound_priority, futureMillis((int)retry));
        outbound = NULL;
        notifyTxQueued(retry);
        return;
      }

      bool success = _radio->startSendRaw(raw, len);
      if (!success) {
        onLbtRefused(now, len);  // ZEPHCORE: re-queue (upstream drops the packet)
        return;
      }
      outbound_expiry = futureMillis(max_airtime);
      cad_busy_start = 0;  // ZEPHCORE: a transmit got out, so any refusal streak is over
      lbt_busy_start = 0;
    }
  }
}

Packet* Dispatcher::obtainNewPacket() {
  auto pkt = _mgr->allocNew();  // TODO: zero out all fields
  if (pkt == NULL) {
    _err_flags |= ERR_EVENT_FULL;
  } else {
    pkt->payload_len = pkt->path_len = 0;
    pkt->_snr = 0;
  }
  return pkt;
}

void Dispatcher::releasePacket(Packet* packet) {
  _mgr->free(packet);
}

void Dispatcher::sendPacket(Packet* packet, uint8_t priority, uint32_t delay_millis) {
  if (!Packet::isValidPathLen(packet->path_len) || packet->payload_len > MAX_PACKET_PAYLOAD) {
    MESH_DEBUG_PRINTLN("%s Dispatcher::sendPacket(): ERROR: invalid packet... path_len=%d, payload_len=%d", getLogDateTime(), (uint32_t) packet->path_len, (uint32_t) packet->payload_len);
    _mgr->free(packet);
  } else {
    _mgr->queueOutbound(packet, priority, futureMillis(delay_millis));
    // ZEPHCORE: wake the loop even for delay 0 — companion sends enqueue from the
    // system work queue, off the mesh thread, and need the drain signal.
    notifyTxQueued(delay_millis);
  }
}

// Utility function -- handles the case where millis() wraps around back to zero
//   2's complement arithmetic will handle any unsigned subtraction up to HALF the word size (32-bits in this case)
bool Dispatcher::millisHasNowPassed(uint32_t timestamp) const {
  return (int32_t)((uint32_t)_ms->getMillis() - timestamp) > 0;
}

uint32_t Dispatcher::futureMillis(int millis_from_now) const {
  return (uint32_t)_ms->getMillis() + millis_from_now;
}

}
