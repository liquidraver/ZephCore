/*
 * SPDX-License-Identifier: MIT
 * CommonCLI - Common CLI command handlers for repeaters
 */

#pragma once

#include <zephyr/kernel.h>
#include <mesh/Mesh.h>
#include <mesh/MeshCore.h>
#include <helpers/IdentityStore.h>
#include <helpers/ClientACL.h>
#include "NodePrefs.h"

class MeshTimeSync;

/* CLI reply buffer size — callers must provide at least this many bytes */
#define CLI_REPLY_SIZE 256

/* Remote-admin replies ride in the caller's LoRa packet buffer:
 * RepeaterMesh/RoomServerMesh onPeerDataRecv declare temp[5 + this] with the
 * reply text at offset 5. Handlers that can exceed this must self-limit
 * whenever sender_timestamp != 0 (0 marks the local USB CLI). */
#define CLI_REMOTE_REPLY_SIZE 161

/* Deferred reboot types */
#define REBOOT_NONE       0
#define REBOOT_NORMAL     1
#define REBOOT_DFU        2
#define REBOOT_OTA        3

class CommonCLICallbacks {
public:
	virtual void savePrefs() = 0;
	virtual const char* getFirmwareVer() = 0;
	virtual const char* getBuildDate() = 0;
	virtual const char* getRole() = 0;
	virtual bool formatFileSystem() = 0;
	virtual void sendSelfAdvertisement(int delay_millis, bool flood) = 0;
	virtual void updateAdvertTimer() = 0;
	virtual void updateFloodAdvertTimer() = 0;
	virtual void setLoggingOn(bool enable) = 0;
	virtual void eraseLogFile() = 0;
	virtual void dumpLogFile() = 0;
	virtual void setTxPower(int8_t power_dbm) = 0;
	/* Apply RX boosted gain live; returns false when the radio has no
	 * RX boost feature (upstream PR #2844 semantics). */
	virtual bool setRxBoostedGain(bool enable) { (void)enable; return false; }
	/* Gate an external FEM/LNA in the RX direction, live.  Same semantics:
	 * false means the radio driver has no such knob.  On a supported radio
	 * whose board has no FEM wired it returns true and does nothing. */
	virtual bool setFemRxGain(bool enable) { (void)enable; return false; }
	/* Configure LR2021 side detectors (multi-SF receive); num = 0 disables.
	 * Returns false when the radio has no side detectors, or when the set
	 * violates a chip constraint — the driver is the validator. */
	virtual bool configSideDetectors(const uint8_t* sfs, uint8_t num) {
		(void)sfs; (void)num; return false;
	}
	/* Repeater-specific — default replies keep companion builds clean.
	 * Repeater overrides all four; companions get "not available". */
	virtual void formatNeighborsReply(char* reply)      { strcpy(reply, "not available"); }
	virtual void removeNeighbor(const uint8_t* pubkey, int key_len) {
		(void)pubkey; (void)key_len;
	}
	/* True when the companion transport has nothing left to deliver: TX queue
	 * drained AND no delivery-ack still held back.  Reboot-class commands poll
	 * this before resetting so the reply and its ack are not cut off by the
	 * reset they just scheduled.  Default true — the repeater's LoRa reply is
	 * covered by the fixed pre-reboot delay instead. */
	virtual bool transportTxIdle() { return true; }
	virtual void formatStatsReply(char* reply)           { strcpy(reply, "not available"); }
	virtual void formatRadioStatsReply(char* reply)      { strcpy(reply, "not available"); }
	virtual void formatPacketStatsReply(char* reply)     { strcpy(reply, "not available"); }
	virtual mesh::LocalIdentity& getSelfId() = 0;
	virtual void saveIdentity(const mesh::LocalIdentity& new_id) = 0;
	virtual void clearStats() = 0;
	virtual void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) = 0;
	/* Freeze the live radio on the given (old) params via radio override so the
	 * caller can mutate _prefs to new values, savePrefs(), and have any later
	 * savePrefs() call write the new values without clobbering the running radio.
	 * No-op if an override is already active (tempradio takes precedence). */
	virtual void freezeRadioParams(float freq, float bw, uint8_t sf, uint8_t cr) { (void)freq; (void)bw; (void)sf; (void)cr; }

	// Adaptive contention window
	virtual float getContentionEstimate() const { return -1.0f; }
	virtual float getFloodDelayFactor() const { return 0.5f; }
	virtual void setBackoffMultiplier(float m) { (void)m; }

	// Duty-cycle preamble false-positive counter (SX126x only)
	virtual uint32_t getDutyCycleTimeoutRestarts() const { return 0; }
	virtual void resetDutyCycleTimeoutRestarts() {}

	// Adaptive CAD (LBT detPeak calibration)
	virtual int formatCadStatus(char* buf, int cap) { (void)buf; (void)cap; return 0; }
	/* Carrier frequency error accumulated from received packets; 0 = this
	 * radio cannot measure it (only the LR2021 does today). */
	virtual int formatFreqErrorStatus(char* buf, int cap) { (void)buf; (void)cap; return 0; }
	virtual void applyCadPrefs() {}
	virtual void resetCadStats() {}

	// Mesh time sync (all roles wire one; nullptr = not compiled/available)
	virtual MeshTimeSync* getMeshTimeSync() { return nullptr; }

	// Sensor manager interface (for GPS)
	virtual double getNodeLat() const { return 0.0; }
	virtual double getNodeLon() const { return 0.0; }
	virtual bool setGpsEnabled(bool enabled) { return false; }
	virtual bool isGpsEnabled() const { return false; }
	virtual void formatGpsStatsReply(char* reply) { strcpy(reply, "off"); }
	/* Role default for "set gps duty default". Companion default (300s); the
	 * repeater/room overrides return ZEPHCORE_REPEATER_GPS_INTERVAL_SEC. */
	virtual uint32_t getDefaultGpsIntervalSec() const { return CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC; }
	virtual int getNumSensorSettings() const { return 0; }
	virtual const char* getSensorSettingName(int idx) const { return nullptr; }
	virtual const char* getSensorSettingValue(int idx) const { return nullptr; }
	virtual const char* getSensorSettingByKey(const char* key) const { return nullptr; }
	virtual bool setSensorSettingValue(const char* key, const char* value) { return false; }
};

class CommonCLI {
	/* Members ordered to match constructor initialization order */
	mesh::MainBoard* _board;
	mesh::RTCClock* _rtc;
	ClientACL* _acl;
	NodePrefs* _prefs;
	CommonCLICallbacks* _callbacks;
	char tmp[PRV_KEY_SIZE * 2 + 4];

	/* Deferred reboot - lets LoRa reply be sent before rebooting */
	struct k_work_delayable _reboot_work;
	uint8_t _pending_reboot;
	/* Uptime (ms) past which the reboot goes ahead even if the transport is
	 * still busy — a stalled or dropped link must not wedge the reset. */
	int64_t _reboot_deadline_ms;
	static void rebootWorkHandler(struct k_work *work);

	/* Bytes the caller already consumed at the head of the reply buffer before
	 * handing it to us — see setReplyHeaderUsed(). */
	uint8_t _reply_hdr_used;

	/* Space a handler may write at `reply`, terminator included.  Local USB
	 * gets the full buffer; a remote reply rides in the caller's packet buffer
	 * and must also leave room for whatever the caller wrote ahead of us. */
	size_t replyCap(uint32_t sender_timestamp) const {
		return (sender_timestamp == 0) ? CLI_REPLY_SIZE
					       : (CLI_REMOTE_REPLY_SIZE - _reply_hdr_used);
	}

	mesh::RTCClock* getRTCClock() { return _rtc; }
	void savePrefs();
	void scheduleReboot(uint8_t type);

public:
	CommonCLI(mesh::MainBoard& board, mesh::RTCClock& rtc, ClientACL& acl,
		  NodePrefs* prefs, CommonCLICallbacks* callbacks)
		: _board(&board), _rtc(&rtc), _acl(&acl), _prefs(prefs), _callbacks(callbacks),
		  _pending_reboot(REBOOT_NONE), _reboot_deadline_ms(0), _reply_hdr_used(0)
	{
		k_work_init_delayable(&_reboot_work, rebootWorkHandler);
	}

	/* Roles that strip the `XX|` request-tag prefix echo those 3 bytes into the
	 * head of the reply buffer and advance `reply` past them before calling us.
	 * Their buffer is still only 5 + CLI_REMOTE_REPLY_SIZE, so the self-limiting
	 * handlers have to subtract what was already written or they overrun it.
	 * Call this on EVERY dispatch, with 0 when no prefix was stripped. */
	void setReplyHeaderUsed(uint8_t n) { _reply_hdr_used = n; }

	void handleCommand(uint32_t sender_timestamp, const char* command, char* reply);
	uint8_t buildAdvertData(uint8_t node_type, uint8_t* app_data);

	/* Full adaptive-CAD reset: the learned detPeak offset, the probe statistics
	 * that justified it, and the recorded base anchor, all dropped together.
	 * Public so the companion's app-driven radio change (CMD_SET_RADIO_PARAMS,
	 * which applies live rather than at reboot) can reach the same code path
	 * instead of open-coding it.  See the definition for `preset_pending`. */
	void resetCadState(bool preset_pending);
};
