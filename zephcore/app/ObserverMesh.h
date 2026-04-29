/*
 * SPDX-License-Identifier: Apache-2.0
 * ObserverMesh — listen-only LoRa mesh node.
 *
 * Extends mesh::Dispatcher directly (no routing, no flooding, no ACL).
 * Every received packet is forwarded to the MQTT publisher queue.
 * CLI handles WiFi/MQTT/radio configuration.
 */

#pragma once

#include <mesh/Dispatcher.h>
#include <mesh/StaticPoolPacketManager.h>
#include <mesh/Identity.h>
#include <mesh/RNG.h>
#include <mesh/RTC.h>
#include <helpers/NodePrefs.h>
#include "RepeaterDataStore.h"
#include "observer_creds.h"

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v1.15.1-zephyr"
#endif

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   __DATE__
#endif

namespace mesh {

class ObserverMesh : public Dispatcher {
	StaticPoolPacketManager _pkt_mgr;

	/* Cached values set by logRxRaw / logRx before onRecvPacket */
	float    _last_rssi;
	float    _last_score;
	uint8_t  _last_raw[MAX_TRANS_UNIT + 1];
	int      _last_raw_len;

	/* Identity and config */
	LocalIdentity     _self_id;
	NodePrefs         _prefs;
	RepeaterDataStore *_store;
	struct ObserverCreds *_creds;
	RNG               *_rng;
	RTCClock          *_rtc;

	/* Pre-built MQTT topic strings (set in begin()) */
	char _pubkey_hex[PUB_KEY_SIZE * 2 + 1];  /* 64 hex chars + NUL */
	char _packets_topic[160];
	char _status_topic[160];

	/* Private helpers */
	void buildTopics();
	void enqueuePacket(Packet *pkt);
	void buildStatusJson(const char *status, char *out, size_t out_size);
	uint32_t _start_uptime_secs;

protected:
	/* Capture RSSI + raw bytes before packet is parsed */
	void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;
	/* Capture score (called between logRxRaw and onRecvPacket) */
	void logRx(Packet *packet, int len, float score) override;
	/* Build JSON and enqueue to MQTT publisher */
	DispatcherAction onRecvPacket(Packet *pkt) override;

public:
	ObserverMesh(Radio &radio, MillisecondClock &ms, RNG &rng, RTCClock &rtc);

	/* Initialize: load/generate identity, load/init prefs, start radio RX. */
	void begin(RepeaterDataStore *store, struct ObserverCreds *creds);

	/* Handle a single serial CLI command.
	 * reply is filled with the response string (CLI_REPLY_SIZE bytes).
	 * Returns true if the command was 'help' and the caller should print
	 * the full banner (too long for the reply buffer). */
	bool handleCLI(const char *command, char *reply, int reply_size);

	/* Publish a synthetic zero-hop advert for this observer to the packets
	 * topic so that CoreScope can place it on the map.  No-op if lat/lon
	 * are not configured in the creds struct. */
	void publishSelfAdvert();
	void publishStatus(const char *status);

	/* Accessors used by main_observer.cpp */
	NodePrefs *getNodePrefs()               { return &_prefs; }
	const LocalIdentity &getSelfId() const  { return _self_id; }
	const char *getPacketsTopic() const     { return _packets_topic; }
	const char *getStatusTopic()  const     { return _status_topic; }
	const char *getPubkeyHex()    const     { return _pubkey_hex; }
};

} /* namespace mesh */
