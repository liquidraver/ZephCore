/*
 * SPDX-License-Identifier: MIT
 * CommonCLI - the text CLI shared by every role
 */

#pragma once

#include <zephyr/kernel.h>
#include <mesh/Mesh.h>
#include <mesh/MeshCore.h>
#include <helpers/IdentityStore.h>
#include <helpers/ClientACL.h>
#include <helpers/RegionMap.h>
#include <helpers/SensorManager.h>
#include "NodePrefs.h"

class MeshTimeSync;

/* A local caller's reply buffer (sender_timestamp 0) */
#define CLI_REPLY_SIZE 256

/* A remote reply rides in the caller's packet buffer, temp[5 + this] with the
 * text at offset 5; see replyCap(). */
#define CLI_REMOTE_REPLY_SIZE 161

/* Deferred reboot types */
#define REBOOT_NONE       0
#define REBOOT_NORMAL     1
#define REBOOT_DFU        2
#define REBOOT_OTA        3
#define REBOOT_POWEROFF   4

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
	/* The radio knobs below return false when the radio lacks the feature. */
	virtual bool setRxBoostedGain(bool enable) { (void)enable; return false; }
	/* External FEM/LNA in RX. True and a no-op on a supporting radio whose
	 * board has no FEM wired. */
	virtual bool setFemRxGain(bool enable) { (void)enable; return false; }
	/* LR2021 multi-SF receive; num 0 disables. The driver validates the set. */
	virtual bool configSideDetectors(const uint8_t* sfs, uint8_t num) {
		(void)sfs; (void)num; return false;
	}
	/* Repeater only; other roles answer "not available". */
	virtual void formatNeighborsReply(char* reply)      { strcpy(reply, "not available"); }
	virtual void removeNeighbor(const uint8_t* pubkey, int key_len) {
		(void)pubkey; (void)key_len;
	}
	/* Reboot-class commands wait for this so the reply (and a companion's held
	 * delivery ack) is not cut off by the reset. The server roles rely on the
	 * fixed pre-reboot delay. */
	virtual bool transportTxIdle() { return true; }
	virtual void formatStatsReply(char* reply)           { strcpy(reply, "not available"); }
	virtual void formatRadioStatsReply(char* reply)      { strcpy(reply, "not available"); }
	virtual void formatPacketStatsReply(char* reply)     { strcpy(reply, "not available"); }
	virtual mesh::LocalIdentity& getSelfId() = 0;
	virtual void saveIdentity(const mesh::LocalIdentity& new_id) = 0;
	virtual void clearStats() = 0;
	virtual void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) = 0;

	/* Region CLI hooks, as upstream: `region load` starts a multi-line load in
	 * the role, `region save` / `region default` persist through it. */
	virtual void startRegionsLoad() {}
	virtual bool saveRegions() { return false; }
	virtual void onDefaultRegionChanged(const RegionEntry* r) { (void)r; }
	/* Keep the live radio on these (old) params through an override while
	 * _prefs takes the new ones for the next boot. No-op under tempradio. */
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
	/* Carrier frequency error of received packets; 0 = not measurable (only
	 * the LR2021 measures it). */
	virtual int formatFreqErrorStatus(char* buf, int cap) { (void)buf; (void)cap; return 0; }
	virtual void applyCadPrefs() {}
	virtual void resetCadStats() {}

	// Mesh time sync (all roles wire one; nullptr = not compiled/available)
	virtual MeshTimeSync* getMeshTimeSync() { return nullptr; }

	/* "set gps duty default"; the server roles override it. */
	virtual uint32_t getDefaultGpsIntervalSec() const { return CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC; }
};

class CommonCLI {
	mesh::MainBoard* _board;
	mesh::RTCClock* _rtc;
	SensorManager* _sensors;
	RegionMap* _region_map;  /* nullptr on the companion: no region CLI */
	ClientACL* _acl;         /* nullptr on the companion (it has no clients) */
	NodePrefs* _prefs;
	CommonCLICallbacks* _callbacks;
	char tmp[PRV_KEY_SIZE * 2 + 4];

	/* Deferred reboot, so the reply goes out first */
	struct k_work_delayable _reboot_work;
	uint8_t _pending_reboot;
	/* Uptime (ms) after which the reboot goes ahead even if the transport is
	 * still busy: a stalled link must not wedge the reset. */
	int64_t _reboot_deadline_ms;
	static void rebootWorkHandler(struct k_work *work);

	uint8_t _reply_hdr_used;  /* see setReplyHeaderUsed() */

	/* Bytes a handler may write at `reply`, terminator included: the whole
	 * local buffer, or what is left of a remote one after the caller's header. */
	size_t replyCap(uint32_t sender_timestamp) const {
		return (sender_timestamp == 0) ? CLI_REPLY_SIZE
					       : (CLI_REMOTE_REPLY_SIZE - _reply_hdr_used);
	}

	mesh::RTCClock* getRTCClock() { return _rtc; }
	void savePrefs();
	void scheduleReboot(uint8_t type);
	bool handleRadioCmd(uint32_t sender_timestamp, const char* command, char* reply);
	void handleGetCmd(uint32_t sender_timestamp, const char* command, char* reply);
	void handleSetCmd(uint32_t sender_timestamp, const char* command, char* reply);
	void handleRegionCmd(uint32_t sender_timestamp, char* command, char* reply);
	void handleRegionCmdCopy(uint32_t sender_timestamp, const char* command, char* reply);

public:
	CommonCLI(mesh::MainBoard& board, mesh::RTCClock& rtc, SensorManager& sensors,
		  RegionMap* region_map, ClientACL* acl, NodePrefs* prefs, CommonCLICallbacks* callbacks)
		: _board(&board), _rtc(&rtc), _sensors(&sensors), _region_map(region_map), _acl(acl),
		  _prefs(prefs), _callbacks(callbacks),
		  _pending_reboot(REBOOT_NONE), _reboot_deadline_ms(0), _reply_hdr_used(0)
	{
		k_work_init_delayable(&_reboot_work, rebootWorkHandler);
	}

	/* Bytes of the reply buffer the caller already used (a reflected "xx|"
	 * prefix). Set on every dispatch, 0 when none. */
	void setReplyHeaderUsed(uint8_t n) { _reply_hdr_used = n; }

	void handleCommand(uint32_t sender_timestamp, const char* command, char* reply);
	uint8_t buildAdvertData(uint8_t node_type, uint8_t* app_data);

	/* Drop the learned CAD offset, its probe statistics and base anchor
	 * together. Public for the companion's live radio change. See the
	 * definition for `preset_pending`. */
	void resetCadState(bool preset_pending);
};
