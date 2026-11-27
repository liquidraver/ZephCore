#pragma once

#include <mesh/MeshCore.h>
#include <mesh/Identity.h>
#include <mesh/Packet.h>
#include <mesh/Utils.h>
#include <mesh/Maintenance.h>  // ZEPHCORE: deadline-driven maintenance
#include <string.h>

namespace mesh {

/**
 * \brief  Abstraction of local/volatile clock with Millisecond granularity.
*/
class MillisecondClock {
public:
  virtual unsigned long getMillis() = 0;
};

/**
 * \brief  Abstraction of this device's packet radio.
*/
class Radio {
public:
  virtual void begin() { }

  /**
   * \brief  polls for incoming raw packet.
   * \param  bytes  destination to store incoming raw packet.
   * \param  sz   maximum packet size allowed.
   * \returns 0 if no incoming data, otherwise length of complete packet received.
  */
  virtual int recvRaw(uint8_t* bytes, int sz) = 0;

  /**
   * \returns  estimated transmit air-time needed for packet of 'len_bytes', in milliseconds.
  */
  virtual uint32_t getEstAirtimeFor(int len_bytes) = 0;

  virtual float packetScore(float snr, int packet_len) = 0;

  /**
   * \brief  starts the raw packet send. (no wait)
   * \param  bytes   the raw packet data
   * \param  len  the length in bytes
   * \returns true if successfully started
  */
  virtual bool startSendRaw(const uint8_t* bytes, int len) = 0;

  /**
   * \returns true if the previous 'startSendRaw()' completed successfully.
   * ZEPHCORE: one-shot — returns true exactly once per completed transmit and
   * consumes that completion (like RadioLibWrapper). Never use it as a state
   * query; use isTxActive() for that.
  */
  virtual bool isSendComplete() = 0;

  // ZEPHCORE: non-consuming "a transmit is in flight" query.
  virtual bool isTxActive() const { return false; }

  /**
   * \brief  a hook for doing any necessary clean up after transmit.
  */
  virtual void onSendFinished() = 0;

  // ZEPHCORE: no loop() -- the Dispatcher never polls the radio; periodic radio
  // work is deadline-driven through msUntilNextMaintenance()/radioMaintenance().

  virtual int getNoiseFloor() const { return 0; }

  virtual void triggerNoiseFloorCalibrate(int threshold) { }

  // ZEPHCORE: no setCADEnabled() (CAD is always LBT inside the driver) and no
  // resetAGC() (the AGC unstick is part of radioMaintenance()). Left out so a
  // port calling them fails to compile instead of silently doing nothing.

  virtual bool isInRecvMode() const = 0;

  /**
   * \returns  true if the radio is currently mid-receive of a packet.
  */
  virtual bool isReceiving() { return false; }

  virtual float getLastRSSI() const { return 0; }
  virtual float getLastSNR() const { return 0; }

  // ZEPHCORE: radio extensions used by the Dispatcher and the roles.

  // Radio not command-ready (e.g. BUSY high); the Dispatcher defers TX.
  virtual bool isRadioReady() { return true; }

  // Reset the radio into a known good RX state: called when isReceiving() kept
  // deferring TX for getRxStuckDuration() with no reception ending (ZEPHCORE;
  // upstream: getCADFailMaxDuration(), counted from the first hold). Radios that can
  // stall walk the chip cancel -> REST -> fresh RX, clearing IRQs and latches.
  virtual void recoverRxState() { }

  // Adaptive CAD (LBT detPeak calibration). stored_base is the family base
  // detPeak `offset` was learned against (0 = none recorded); when it differs
  // from the driver's current base the offset is re-anchored so the ABSOLUTE
  // detPeak is preserved. Read getCadOffset()/cadBasePeak() back to persist.
  virtual void setCadParams(bool auto_enabled, int8_t offset, uint16_t probe_interval_s,
                            uint8_t busycap_pct, uint8_t stored_base = 0) { }
  // Family base detPeak in force; 0 = no adaptive CAD, nothing to store.
  virtual uint8_t cadBasePeak() { return 0; }
  // One CAD calibrator tick: maybe probe, update stats, maybe step (auto mode).
  virtual void cadMaintenance() { }
  virtual int8_t getCadOffset() const { return 0; }
  virtual void resetCadStats() { }
  // Last-resort unmute after getTxStarvationDuration() of LBT refusals: one
  // step less sensitive, regardless of cad.auto. False = already at the least
  // sensitive step (channel genuinely busy, or the radio is broken).
  virtual bool cadRelaxOnTxStarvation() { return false; }
  // Human-readable status blocks; return chars written (0 = unsupported).
  virtual int formatCadStatus(char* buf, int cap) { return 0; }
  virtual int formatFreqErrorStatus(char* buf, int cap) { return 0; }
  // LBT outcome counters (clear/busy/error/gate/busy-then-RX), appended to
  // Dispatcher::formatLbtStatus(); 0 = none.
  virtual int formatLbtCounters(char* buf, int cap) { return 0; }
  virtual void resetLbtCounters() { }
  // One LBT CAD plus the RX->TX turnaround: two nodes whose CADs start closer
  // than this both read clear. Backoff slot width; 0 = unknown.
  virtual uint32_t getLbtSlotMs() { return 0; }
  // ZEPHCORE: LBT CAD length in symbols as programmed (0 = no CAD), and a RAM-only
  // request for 4, 8 or 16 (LBT-logging trial builds); false = refused.
  virtual uint8_t getLbtCadSymbols() { return 0; }
  virtual bool setLbtCadSymbols(uint8_t n) { (void)n; return false; }
  // Outcome of the last startSendRaw() as short text ("clr", "busy rssi=-97",
  // "err=-116", "gate=rx"...), for the LBT diagnostic log; 0 = none.
  virtual int formatLastTxAttempt(char* buf, int cap) { return 0; }
  // Receptions that have ended (packet or CRC/header error), wrapping; a
  // deferral waits for it to move. Constant = cannot tell (timer only).
  virtual uint32_t getRxEndSeq() { return 0; }
  // When the last reception ended, on the MillisecondClock's time base
  // (0 = unknown): the retry counts from here, not from when the mesh
  // thread got round to noticing it.
  virtual uint32_t getLastRxEndMillis() { return 0; }

  // Receiver hygiene (deaf-aware AGC unstick, temperature-drift recal). Both
  // sleep the radio, so called only from the maintenance tick.
  virtual void radioMaintenance() { }
  // Milliseconds until this radio's periodic work next needs a call, or
  // MAINTENANCE_IDLE when nothing is pending.
  virtual uint32_t msUntilNextMaintenance() { return MAINTENANCE_IDLE; }

  virtual uint32_t getPacketsRecv() const { return 0; }
  virtual uint32_t getPacketsSent() const { return 0; }
  virtual uint32_t getPacketsRecvErrors() const { return 0; }
};

/**
 * \brief  An abstraction for managing instances of Packets (eg. in a static pool),
 *        and for managing the outbound packet queue.
*/
class PacketManager {
public:
  virtual Packet* allocNew() = 0;
  virtual void free(Packet* packet) = 0;

  virtual void queueOutbound(Packet* packet, uint8_t priority, uint32_t scheduled_for) = 0;
  virtual Packet* getNextOutbound(uint32_t now) = 0;    // by priority
  virtual int getOutboundCount(uint32_t now) const = 0;
  virtual int getOutboundTotal() const = 0;
  virtual int getFreeCount() const = 0;
  virtual Packet* getOutboundByIdx(int i) = 0;
  virtual Packet* removeOutboundByIdx(int i) = 0;
  // ZEPHCORE: no inbound queue (queueInbound/getNextInbound) — received packets
  // are processed immediately; the adaptive contention window replaces upstream's
  // score-based RX delay. Instead, schedule access:
  virtual uint32_t getOutboundSchedule(int i) const = 0;
  virtual uint8_t peekNextOutboundPriority(uint32_t now) const = 0;
};

// ZEPHCORE: event-loop hooks. The mesh thread sleeps in k_event_wait(); these
// let the Dispatcher ask for a wake instead of being polled.
// Pending TX: schedule a wake in delay_ms.
typedef void (*tx_queued_callback_t)(uint32_t delay_ms, void* user_data);
// Run loop() at the next opportunity (off-main code that set state loop() drains).
typedef void (*wake_callback_t)(void* user_data);

typedef uint32_t  DispatcherAction;

#define ACTION_RELEASE           (0)
#define ACTION_MANUAL_HOLD       (1)
#define ACTION_RETRANSMIT(pri)   (((uint32_t)1 + (pri))<<24)
#define ACTION_RETRANSMIT_DELAYED(pri, _delay)  ((((uint32_t)1 + (pri))<<24) | (_delay))

#define ERR_EVENT_FULL              (1 << 0)
#define ERR_EVENT_CAD_TIMEOUT       (1 << 1)
#define ERR_EVENT_STARTRX_TIMEOUT   (1 << 2)

// ZEPHCORE: radio considered stalled after this long neither receiving nor sending.
#define RADIO_STALL_THRESHOLD_MS    8000

/**
 * \brief  The low-level task that manages detecting incoming Packets, and the queueing
 *      and scheduling of outbound Packets.
 * ZEPHCORE: times are uint32_t, not unsigned long — unsigned long is 64 bits on
 * the native_sim/native/64 build, which would change the wrap arithmetic.
*/
class Dispatcher {
  Packet* outbound;  // current outbound packet
  uint32_t outbound_expiry, outbound_start, total_air_time, rx_air_time;
  uint32_t next_tx_time;
  uint32_t cad_busy_start;
  uint32_t radio_nonrx_start;
  bool  prev_isrecv_mode;
  uint32_t n_sent_flood, n_sent_direct;
  uint32_t n_recv_flood, n_recv_direct;
  uint32_t tx_budget_ms;
  uint8_t tx_budget_frac;  // ZEPHCORE: refill remainder, hundredths of a ms
  uint32_t last_budget_update;
  uint32_t duty_cycle_window_ms;

  // ZEPHCORE: state for LBT refusal escalation, adaptive CAD persistence and
  // the event-loop callbacks (see Dispatcher.cpp).
  uint8_t outbound_priority;
  uint32_t lbt_busy_start;
  uint32_t lbt_next_warn;
  uint32_t lbt_streak_max;
  uint8_t lbt_refusals;
  bool tx_hold;               // next_tx_time is a live hold (a stale stamp must not hold)
  bool wait_rx_end;           // deferred: retry once a reception ends
  uint32_t rx_end_seq_seen;   // getRxEndSeq() when the deferral began
  int8_t cad_offset_shadow;
  bool cad_offset_shadow_valid;
  tx_queued_callback_t _tx_queued_cb;
  void* _tx_queued_user_data;
  wake_callback_t _wake_cb;
  void* _wake_user_data;

  void processRecvPacket(Packet* pkt);
  void updateTxBudget();

protected:
  Radio* _radio;
  MillisecondClock* _ms;
  PacketManager* _mgr;
  uint16_t _err_flags;

  Dispatcher(Radio& radio, MillisecondClock& ms, PacketManager& mgr);  // ZEPHCORE: in Dispatcher.cpp

  virtual DispatcherAction onRecvPacket(Packet* pkt) = 0;

  virtual void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) { }   // custom hook

  virtual void logRx(Packet* packet, int len, float score) { }   // hooks for custom logging
  virtual void logTx(Packet* packet, int len) { }
  virtual void logTxFail(Packet* packet, int len) { }
  virtual const char* getLogDateTime() { return ""; }

  virtual uint32_t getCADFailRetryDelay() const;
  virtual uint32_t getCADFailMaxDuration() const;
  // ZEPHCORE: ms since the last reception ended; 0 without a stamp or over 2 s old.
  uint32_t msSinceRxEnd(uint32_t now) const;
  virtual int getInterferenceThreshold() const { return 0; }    // disabled by default
  virtual uint32_t getDutyCycleWindowMs() const { return 3600000; }

  // ZEPHCORE: duty cycle as a percentage (upstream: getAirtimeBudgetFactor);
  // admin packets are exempt from the budget gate.
  virtual uint8_t getDutyCyclePercent() const;
  static bool isAdminPacket(const Packet* pkt);
  // How long the driver's LBT may refuse every transmit before we conclude the
  // detector is too sensitive and relax it one step (see checkSend()).
  virtual uint32_t getTxStarvationDuration() const;
  // ZEPHCORE: how long "receiving" may defer TX with no reception ending before
  // the radio is judged stuck and recoverRxState() runs.
  virtual uint32_t getRxStuckDuration() const;
  // Adaptive CAD moved the operating detPeak offset; subclasses persist it.
  virtual void onCadOffsetChanged(int8_t offset) { }
  // LBT refusals since the last transmit that got out (saturating); logged as
  // ref= on LBT lines, does not size the backoff window.
  uint8_t getLbtRefusals() const { return lbt_refusals; }
  // LBT diagnostic hook (CONFIG_ZEPHCORE_LBT_LOGGING builds only): every
  // transmit attempt and hold-back, with the backoff picked. pkt is NULL for an
  // RX-busy hold, where nothing has been dequeued yet.
  virtual void logTxAttempt(const Packet* pkt, const char* outcome, uint32_t retry_ms) { }
  void stampQueued(Packet* pkt, uint32_t delay_ms);  // ZEPHCORE: LBT logging
  void notifyTxQueued(uint32_t delay_ms) {
    if (_tx_queued_cb) _tx_queued_cb(delay_ms, _tx_queued_user_data);
  }

public:
  void begin();
  void loop();

  // ZEPHCORE: deadline-driven maintenance. msUntilNextMaintenance() is cheap
  // and side-effect free; the event loop calls it after every pass to size its
  // next sleep. Subclasses with their own timed work fold their deadlines in.
  void maintenanceLoop();
  virtual uint32_t msUntilNextMaintenance();

  Packet* obtainNewPacket();
  void releasePacket(Packet* packet);
  void sendPacket(Packet* packet, uint8_t priority, uint32_t delay_millis=0);

  uint32_t getTotalAirTime() const { return total_air_time; }
  uint32_t getReceiveAirTime() const {return rx_air_time; }
  uint32_t getRemainingTxBudget() const { return tx_budget_ms; }
  uint32_t getNumSentFlood() const { return n_sent_flood; }
  uint32_t getNumSentDirect() const { return n_sent_direct; }
  uint32_t getNumRecvFlood() const { return n_recv_flood; }
  uint32_t getNumRecvDirect() const { return n_recv_direct; }
  void resetStats() {
    n_sent_flood = n_sent_direct = n_recv_flood = n_recv_direct = 0;
    _err_flags = 0;
    resetLbtStats();  // ZEPHCORE: `clear stats` covers `get lbt.stats` too
  }

  // ZEPHCORE: error flags and event-loop callbacks.
  uint16_t getErrFlags() const { return _err_flags; }
  // `get lbt.stats`: refusal streaks and backoff, then the radio's counters.
  int formatLbtStatus(char* buf, int cap);
  void resetLbtStats();
  uint32_t getLbtStreakMaxMs() const { return lbt_streak_max; }
  void setTxQueuedCallback(tx_queued_callback_t cb, void* user_data) {
    _tx_queued_cb = cb;
    _tx_queued_user_data = user_data;
  }
  void setWakeCallback(wake_callback_t cb, void* user_data) {
    _wake_cb = cb;
    _wake_user_data = user_data;
  }
  // Safe from any thread: the callback only posts an event.
  void notifyWake() {
    if (_wake_cb) _wake_cb(_wake_user_data);
  }

  // helper methods
  bool millisHasNowPassed(uint32_t timestamp) const;
  uint32_t futureMillis(int millis_from_now) const;

  bool tryParsePacket(Packet* pkt, const uint8_t* raw, int len);

private:
  // ZEPHCORE: percentage budget and checkSend() helpers (duty-cycle gate, LBT
  // refusal escalation).
  uint32_t getMaxTxBudgetMs() const;
  bool txBudgetAllowsSend(uint32_t now);
  void onLbtRefused(uint32_t now, int len, uint32_t rx_seq);
  void waitForRxEnd(uint32_t rx_seq) { wait_rx_end = true; rx_end_seq_seen = rx_seq; }
  // Hold every transmit until t. The only writer of next_tx_time after begin().
  void holdTxUntil(uint32_t t) { next_tx_time = t; tx_hold = true; }
  void noteTxAttempt(const Packet* pkt, const char* outcome, uint32_t retry_ms);
  void checkRecv();
  void checkSend();
};

}
