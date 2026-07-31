/*
 * SPDX-License-Identifier: MIT
 * ObserverMesh — listen-only LoRa mesh node implementation.
 */

#include "ObserverMesh.h"
#include "observer_creds.h"

#include <mesh/Utils.h>
#include <mesh/LoRaConfig.h>
#include <adapters/radio/LoRaRadioBase.h>
#include <adapters/rng/ZephyrRNG.h>   /* generateFirstBootIdentity (hardened keygen) */
#include <helpers/MeshcoreJson.h>

#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_observer, CONFIG_ZEPHCORE_OBSERVER_LOG_LEVEL);

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <ZephyrMQTTPublisher.h>

/* Forward declaration — implemented in ZephyrWiFiStation.c */
extern "C" {
	bool zc_wifi_station_is_connected(void);
	void zc_wifi_station_reconnect(void);
	const char *zc_wifi_station_ssid(void);
}

namespace mesh {

/* ========== Construction ========== */

ObserverMesh::ObserverMesh(Radio &radio, MillisecondClock &ms, RTCClock &rtc)
	: Dispatcher(radio, ms, _pkt_mgr),
	  _last_rssi(0.0f), _last_score(0.0f), _last_raw_len(0),
	  _store(nullptr), _creds(nullptr), _rtc(&rtc), _start_uptime_secs(0)
{
	memset(_pubkey_hex, 0, sizeof(_pubkey_hex));
	memset(_packets_topic, 0, sizeof(_packets_topic));
	memset(_status_topic, 0, sizeof(_status_topic));
}

/* ========== begin() ========== */

void ObserverMesh::begin(RepeaterDataStore *store, struct ObserverCreds *creds)
{
	_store = store;
	_creds = creds;
	_start_uptime_secs = (uint32_t)(k_uptime_get() / 1000);

	/* Initialize prefs with observer-specific defaults */
	initNodePrefs(&_prefs);
	_prefs.cr           = 5;   /* CR 4/5 */
	_prefs.tx_power_dbm = 0;   /* observer never TXes anyway */
	/* freq=869.618, bw=62.5, sf=8 already set by initNodePrefs */

	/* First boot has to be detected BEFORE loadPrefs(): the store is shared
	 * with the repeater and its no-file branch re-runs initNodePrefs(), applies
	 * *repeater* defaults over whatever the caller passed in, saves them, and
	 * returns true.  So the observer values set above are silently discarded on
	 * a fresh unit and there is no return code that says so.  Probing for the
	 * file is the only observer-local way to tell — the alternative, changing
	 * the no-file branch, would alter repeater and room-server behaviour. */
	char prefs_path[64];
	struct fs_dirent prefs_ent;
	snprintf(prefs_path, sizeof(prefs_path), "%s/prefs", _store->getBasePath());
	const bool first_boot = (fs_stat(prefs_path, &prefs_ent) < 0);

	/* Load persisted prefs (overrides defaults with saved values) */
	_store->loadPrefs(_prefs);

	if (first_boot) {
		/* Re-apply the observer defaults the shared no-file branch overwrote,
		 * then persist them so this runs exactly once.  Only the prefs file is
		 * rewritten — obs_creds (WiFi/MQTT/IATA/lat/lon) is a separate file and
		 * is never touched here. */
		_prefs.cr           = 5;
		_prefs.tx_power_dbm = 0;
		_store->savePrefs(_prefs);
		LOG_INF("First boot — saved observer prefs defaults");
	}

	/* Load or generate node identity.
	 *
	 * Use ZephyrRNG::generateFirstBootIdentity — the SAME hardened path the
	 * companion and repeater use (bootloader_random-seeded HWRNG + two-clock
	 * beat, conditioned via AES-256-CTR) — NOT LocalIdentity(_rng). The old
	 * form drew straight from ZephyrRNG::random() / sys_csrand_get, which on
	 * an ESP32 observer is unseeded (BLE never comes up to seed WDEV_RANDOM),
	 * so it derived a permanent key from a weak PRNG. generateFirstBootIdentity
	 * also owns the reserved-prefix retry (100 attempts + panic backstop),
	 * replacing the weaker 10-try loop that silently kept a reserved prefix. */
	if (!_store->loadIdentity(_self_id)) {
		LOG_INF("No identity found — generating new keypair");
		mesh::ZephyrRNG::generateFirstBootIdentity(_self_id);
		_store->saveIdentity(_self_id);
		LOG_INF("New observer identity saved");
	}

	/* Build hex pubkey string */
	Utils::toHex(_pubkey_hex, _self_id.pub_key, PUB_KEY_SIZE);
	_pubkey_hex[PUB_KEY_SIZE * 2] = '\0';

	/* Build MQTT topic strings */
	buildTopics();

	/* Log identity */
	LOG_INF("Observer ID: %.16s...", _pubkey_hex);

	/* Start radio in continuous RX mode */
	Dispatcher::begin();
}

void ObserverMesh::buildStatusJson(const char *status, char *out, size_t out_size)
{
	uint32_t now_epoch = _rtc ? _rtc->getCurrentTime() : 0;

	char radio_buf[48];
	snprintf(radio_buf, sizeof(radio_buf), "%.3f,%.1f,%u,%u",
		 (double)_prefs.freq, (double)_prefs.bw,
		 (unsigned)_prefs.sf, (unsigned)_prefs.cr);

	uint32_t uptime_secs = (uint32_t)(k_uptime_get() / 1000);
	if (uptime_secs >= _start_uptime_secs) {
		uptime_secs -= _start_uptime_secs;
	} else {
		uptime_secs = 0;
	}

	struct MeshcoreStatusJson sj = {
		status,
		now_epoch,
		_prefs.node_name,
		_pubkey_hex,
		radio_buf,
#ifdef CONFIG_ZEPHCORE_BOARD_NAME
		CONFIG_ZEPHCORE_BOARD_NAME,
#else
		"unknown",
#endif
		FIRMWARE_VERSION,
		0u,                                              /* battery_mv (observer has none) */
		uptime_secs,
		0u,                                              /* debug_flags */
		0u,                                              /* queue_len */
		((LoRaRadioBase *)_radio)->getNoiseFloor(),
		0u,                                              /* tx_air_secs */
		0u,                                              /* rx_air_secs */
		((LoRaRadioBase *)_radio)->getPacketsRecvErrors(),
	};
	meshcore_build_status_json(out, out_size, &sj);
}

void ObserverMesh::publishStatus(const char *status)
{
	/* Gate on connected: without it the 300 s status timer fills the
	 * publish queue with stale "online" messages while WiFi or the broker
	 * is down, and they all flush with old timestamps on reconnect. The
	 * on-connect status is posted via MESH_EVENT_MQTT_CONNECT, after
	 * CONNACK, so nothing is lost by skipping here. */
	if (!mqtt_publisher_is_connected()) return;

	size_t json_cap;
	char *json_buf = mqtt_publisher_stage(&json_cap);
	buildStatusJson(status, json_buf, json_cap);
	mqtt_publisher_commit(MQTT_PUB_TOPIC_STATUS, (int)strlen(json_buf));
}

void ObserverMesh::buildTopics()
{
	const char *iata = (_creds && _creds->mqtt_iata[0] != '\0')
			   ? _creds->mqtt_iata : "XXX";

	snprintf(_packets_topic, sizeof(_packets_topic),
		 "meshcore/%s/%s/packets", iata, _pubkey_hex);
	snprintf(_status_topic,  sizeof(_status_topic),
		 "meshcore/%s/%s/status",  iata, _pubkey_hex);
}

/* ========== RX hooks ========== */

void ObserverMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len)
{
	_last_rssi = rssi;
	_last_raw_len = (len <= (int)sizeof(_last_raw)) ? len : (int)sizeof(_last_raw);
	memcpy(_last_raw, raw, _last_raw_len);
}

void ObserverMesh::logRx(Packet *packet, int len, float score)
{
	(void)packet; (void)len;
	_last_score = score;
}

void ObserverMesh::enqueuePacket(Packet *pkt)
{
	/* Compute packet hash (8 bytes → 16 hex chars) */
	uint8_t hash_bytes[MAX_HASH_SIZE];
	char    hash_hex[MAX_HASH_SIZE * 2 + 1];
	pkt->calculatePacketHash(hash_bytes);
	Utils::toHex(hash_hex, hash_bytes, MAX_HASH_SIZE);
	hash_hex[MAX_HASH_SIZE * 2] = '\0';

	/* Encode raw wire bytes as hex */
	/* Each byte → 2 hex chars; max MAX_TRANS_UNIT=255 bytes → 510 chars + NUL */
	static char raw_hex[MAX_TRANS_UNIT * 2 + 1];
	Utils::toHex(raw_hex, _last_raw, _last_raw_len);
	raw_hex[_last_raw_len * 2] = '\0';

	/* Get current timestamp from RTC */
	uint32_t now_epoch = _rtc ? _rtc->getCurrentTime() : 0;

	/* Build JSON payload matching meshcoretomqtt packet format, directly
	 * in the publisher's staging buffer (main-thread-only producer). */
	size_t json_cap;
	char *json_buf = mqtt_publisher_stage(&json_cap);
	struct MeshcorePacketJson pj = {
		_prefs.node_name,
		_pubkey_hex,
		now_epoch,
		_last_raw_len,
		(unsigned)pkt->getPayloadType(),
		pkt->isRouteDirect() ? "D" : "F",
		(unsigned)pkt->payload_len,
		raw_hex,
		(int)pkt->getSNR(),
		(int)_last_rssi,
		(int)(_last_score * 1000.0f),
		hash_hex,
	};
	int json_len = meshcore_build_packet_json(json_buf, json_cap, &pj);

	if (json_len < 0 || json_len >= (int)json_cap) {
		LOG_WRN("Packet JSON truncated (len=%d)", json_len);
		json_len = (int)json_cap - 1;
	}

	mqtt_publisher_commit(MQTT_PUB_TOPIC_PACKETS, json_len);
}

DispatcherAction ObserverMesh::onRecvPacket(Packet *pkt)
{
	/* Publish every reception — no deduplication.
	 * The same flood packet heard from different repeaters is published
	 * separately, each with its own SNR/RSSI (propagation data). */
	enqueuePacket(pkt);
	harvestTimeSample(pkt);
	return ACTION_RELEASE;  /* never retransmit */
}

/* ========== Mesh time sync ========== */

void ObserverMesh::harvestTimeSample(Packet *pkt)
{
	if (pkt->getPayloadType() != PAYLOAD_TYPE_ADVERT) return;
	if (pkt->getPathHashCount() > MeshTimeSync::HOP_CAP) return;
	if (pkt->payload_len < PUB_KEY_SIZE + 4 + SIGNATURE_SIZE) return;
	/* Skip share rebroadcasts (transport codes {0,0}) — they replay stale
	 * stored adverts and would churn the original sender's tenure. */
	if (pkt->hasTransportCodes() &&
	    pkt->transport_codes[0] == 0 && pkt->transport_codes[1] == 0) {
		return;
	}

	int i = 0;
	Identity id;
	memcpy(id.pub_key, &pkt->payload[i], PUB_KEY_SIZE);
	i += PUB_KEY_SIZE;
	uint32_t timestamp;
	memcpy(&timestamp, &pkt->payload[i], 4);
	i += 4;
	const uint8_t *signature = &pkt->payload[i];
	i += SIGNATURE_SIZE;

	/* Observers have no dedup and hear every flood copy — skip the
	 * expensive Ed25519 verify when the sample cannot update the table. */
	if (!_timesync.wouldAccept(id.pub_key, timestamp)) return;

	size_t app_data_len = pkt->payload_len - (size_t)i;
	if (app_data_len > MAX_ADVERT_DATA_SIZE) app_data_len = MAX_ADVERT_DATA_SIZE;

	uint8_t message[PUB_KEY_SIZE + 4 + MAX_ADVERT_DATA_SIZE];
	int msg_len = 0;
	memcpy(&message[msg_len], id.pub_key, PUB_KEY_SIZE); msg_len += PUB_KEY_SIZE;
	memcpy(&message[msg_len], &timestamp, 4); msg_len += 4;
	memcpy(&message[msg_len], &pkt->payload[i], app_data_len); msg_len += app_data_len;

	if (!id.verify(signature, message, msg_len)) return;

	_timesync.onAdvertHeard(id.pub_key, timestamp, pkt->getPathHashCount(),
				(uint32_t)(k_uptime_get() / 1000));
}

void ObserverMesh::timeSyncTick()
{
	if (!_prefs.meshtimesync || !_rtc) return;
	/* Shared policy (suppression/pedigree) lives in runTick; no
	 * observer-side bookkeeping needs shifting on a step. */
	_timesync.runTick(*_rtc);
}

void ObserverMesh::noteTrustedTimeSync()
{
	_timesync.noteManualSync((uint32_t)(k_uptime_get() / 1000));
}

/* ========== Serial CLI ========== */

#define CLI_REPLY_SIZE 256

bool ObserverMesh::handleCLI(const char *command, char *reply, int reply_size)
{
	reply[0] = '\0';

	/* ---- help ---- */
	if (strcmp(command, "help") == 0 || command[0] == '\0') {
		return true;  /* caller prints the banner */
	}

	/* ---- get commands ---- */
	if (memcmp(command, "get ", 4) == 0) {
		const char *key = command + 4;

		if (strcmp(key, "role") == 0) {
			snprintf(reply, reply_size, "observer");

		} else if (strcmp(key, "name") == 0) {
			snprintf(reply, reply_size, "%s", _prefs.node_name);

		} else if (strcmp(key, "public.key") == 0) {
			snprintf(reply, reply_size, "%s", _pubkey_hex);

		} else if (strcmp(key, "board") == 0) {
#ifdef CONFIG_ZEPHCORE_BOARD_NAME
			snprintf(reply, reply_size, "%s", CONFIG_ZEPHCORE_BOARD_NAME);
#else
			snprintf(reply, reply_size, "unknown");
#endif
		} else if (strcmp(key, "version") == 0) {
			snprintf(reply, reply_size, "%s (%s)", FIRMWARE_VERSION, FIRMWARE_BUILD_DATE);

		} else if (strcmp(key, "radio") == 0) {
			snprintf(reply, reply_size,
				 "freq=%.3f bw=%.1f sf=%u cr=%u tx=%ddBm",
				 (double)_prefs.freq, (double)_prefs.bw,
				 _prefs.sf, _prefs.cr, _prefs.tx_power_dbm);

		} else if (strcmp(key, "wifi.ssid") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->wifi_ssid[0]) ? _creds->wifi_ssid : "(not set)");

		} else if (strcmp(key, "wifi.status") == 0) {
			snprintf(reply, reply_size, "%s",
				 zc_wifi_station_is_connected() ? "connected" : "disconnected");

		} else if (strcmp(key, "mqtt.status") == 0) {
			snprintf(reply, reply_size, "%s",
				 mqtt_publisher_is_connected() ? "connected" : "disconnected");

		} else if (strcmp(key, "mqtt.host") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->mqtt_host[0]) ? _creds->mqtt_host : "(not set)");

		} else if (strcmp(key, "mqtt.user") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->mqtt_user[0]) ? _creds->mqtt_user : "(not set)");

		} else if (strcmp(key, "mqtt.iata") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->mqtt_iata[0]) ? _creds->mqtt_iata : "(not set)");

		} else if (strcmp(key, "mqtt.port") == 0) {
			snprintf(reply, reply_size, "%u",
				 (_creds) ? (unsigned)_creds->mqtt_port : 8883u);

		} else if (strcmp(key, "meshtimesync") == 0) {
			_timesync.formatStatus(reply, reply_size,
					       _rtc ? _rtc->getCurrentTime() : 0,
					       (uint32_t)(k_uptime_get() / 1000),
					       _prefs.meshtimesync != 0);

		} else if (strcmp(key, "mqtt.tls") == 0) {
			snprintf(reply, reply_size, "%u",
				 (_creds) ? (unsigned)_creds->mqtt_tls : 1u);

		} else if (strcmp(key, "lat") == 0) {
			if (_creds && _creds->lat_e6 != 0) {
				snprintf(reply, reply_size, "%.6f", (double)_creds->lat_e6 / 1e6);
			} else {
				snprintf(reply, reply_size, "(not set)");
			}

		} else if (strcmp(key, "lon") == 0) {
			if (_creds && _creds->lon_e6 != 0) {
				snprintf(reply, reply_size, "%.6f", (double)_creds->lon_e6 / 1e6);
			} else {
				snprintf(reply, reply_size, "(not set)");
			}

		} else {
			snprintf(reply, reply_size, "ERR unknown key: %s", key);
		}
		return false;
	}

	/* ---- set commands ---- */
	if (memcmp(command, "set ", 4) == 0) {
		const char *rest = command + 4;

		/* Helper: find value after "key " */
		auto find_val = [](const char *s, const char *prefix) -> const char * {
			size_t n = strlen(prefix);
			if (memcmp(s, prefix, n) == 0 && s[n] == ' ')
				return s + n + 1;
			return nullptr;
		};

		const char *val;

		if ((val = find_val(rest, "name")) != nullptr) {
			strncpy(_prefs.node_name, val, sizeof(_prefs.node_name) - 1);
			_prefs.node_name[sizeof(_prefs.node_name) - 1] = '\0';
			_store->savePrefs(_prefs);
			snprintf(reply, reply_size, "name=%s", _prefs.node_name);

		} else if ((val = find_val(rest, "freq")) != nullptr) {
			float f = (float)atof(val);
			/* Accept Hz (e.g. 869618000) or MHz (e.g. 869.618) */
			if (f > 1000000.0f) f /= 1000000.0f;
			/* 300..1000 MHz is not the radio's limit — it is what
			 * RepeaterDataStore::loadPrefs() accepts on the way back in.
			 * Anything outside it saves fine and is then silently reset to
			 * defaults on the next boot, taking bw/sf/cr/tx_power with it, so
			 * refuse it here rather than hand back a value that won't survive. */
			if (f >= 300.0f && f <= 1000.0f) {
				_prefs.freq = f;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "freq=%.3f MHz", (double)_prefs.freq);
			} else {
				snprintf(reply, reply_size, "ERR freq must be 300-1000 MHz");
			}

		} else if ((val = find_val(rest, "sf")) != nullptr) {
			int sf = atoi(val);
			if (sf >= 7 && sf <= 12) {
				_prefs.sf = (uint8_t)sf;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "sf=%u", _prefs.sf);
			} else {
				snprintf(reply, reply_size, "ERR sf must be 7-12");
			}

		} else if ((val = find_val(rest, "bw")) != nullptr) {
			/* Accept index (0=125, 1=250, 2=500, 3=62.5, 4=41.7, 5=31.25)
			 * or kHz value directly */
			float bw;
			int idx = atoi(val);
			const float bw_table[] = { 125.0f, 250.0f, 500.0f, 62.5f, 41.7f, 31.25f };
			if (idx >= 0 && idx <= 5) {
				bw = bw_table[idx];
			} else {
				bw = (float)atof(val);
			}
			/* Lower bound 7 kHz mirrors loadPrefs()'s validator — see the freq
			 * case above for why the CLI must not accept what it will reject. */
			if (bw >= 7.0f && bw <= 500.0f) {
				_prefs.bw = bw;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "bw=%.2f kHz", (double)_prefs.bw);
			} else {
				snprintf(reply, reply_size, "ERR bw must be 7-500 kHz (or index 0-5)");
			}

		} else if ((val = find_val(rest, "cr")) != nullptr) {
			int cr = atoi(val);
			if (cr >= 5 && cr <= 8) {
				_prefs.cr = (uint8_t)cr;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "cr=%u", _prefs.cr);
			} else {
				snprintf(reply, reply_size, "ERR cr must be 5-8");
			}

		} else if ((val = find_val(rest, "meshtimesync")) != nullptr) {
			if (strcmp(val, "on") == 0) {
				_prefs.meshtimesync = 1;
				_store->savePrefs(_prefs);
				snprintf(reply, reply_size, "meshtimesync=on");
			} else if (strcmp(val, "off") == 0) {
				_prefs.meshtimesync = 0;
				_store->savePrefs(_prefs);
				snprintf(reply, reply_size, "meshtimesync=off");
			} else {
				snprintf(reply, reply_size, "ERR must be on or off");
			}

		} else if (!_creds) {
			snprintf(reply, reply_size, "ERR creds not initialized");

		} else if ((val = find_val(rest, "wifi.ssid")) != nullptr) {
			strncpy(_creds->wifi_ssid, val, sizeof(_creds->wifi_ssid) - 1);
			_creds->wifi_ssid[sizeof(_creds->wifi_ssid) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "wifi.ssid=%s (reconnecting)", _creds->wifi_ssid);
			zc_wifi_station_reconnect();

		} else if ((val = find_val(rest, "wifi.psk")) != nullptr) {
			strncpy(_creds->wifi_psk, val, sizeof(_creds->wifi_psk) - 1);
			_creds->wifi_psk[sizeof(_creds->wifi_psk) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "wifi.psk=*** (saved, reconnecting)");
			zc_wifi_station_reconnect();

		} else if ((val = find_val(rest, "mqtt.host")) != nullptr) {
			strncpy(_creds->mqtt_host, val, sizeof(_creds->mqtt_host) - 1);
			_creds->mqtt_host[sizeof(_creds->mqtt_host) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.host=%s (reconnecting)", _creds->mqtt_host);
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.port")) != nullptr) {
			int port = atoi(val);
			if (port > 0 && port <= 65535) {
				_creds->mqtt_port = (uint16_t)port;
				observer_creds_save(_creds, _store->getBasePath());
				snprintf(reply, reply_size, "mqtt.port=%u (reconnecting)", _creds->mqtt_port);
				mqtt_publisher_reconnect();
			} else {
				snprintf(reply, reply_size, "ERR port must be 1-65535");
			}

		} else if ((val = find_val(rest, "mqtt.tls")) != nullptr) {
			int tls = atoi(val);
			_creds->mqtt_tls = (tls != 0) ? 1 : 0;
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.tls=%u (reconnecting)", _creds->mqtt_tls);
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.user")) != nullptr) {
			strncpy(_creds->mqtt_user, val, sizeof(_creds->mqtt_user) - 1);
			_creds->mqtt_user[sizeof(_creds->mqtt_user) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.user=%s (reconnecting)", _creds->mqtt_user);
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.password")) != nullptr) {
			strncpy(_creds->mqtt_password, val, sizeof(_creds->mqtt_password) - 1);
			_creds->mqtt_password[sizeof(_creds->mqtt_password) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.password=*** (saved, reconnecting)");
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.iata")) != nullptr) {
			strncpy(_creds->mqtt_iata, val, sizeof(_creds->mqtt_iata) - 1);
			_creds->mqtt_iata[sizeof(_creds->mqtt_iata) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			buildTopics();  /* rebuild topic strings with new IATA */
			snprintf(reply, reply_size, "mqtt.iata=%s (topics updated, reconnecting)", _creds->mqtt_iata);
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "lat")) != nullptr) {
			double lat_d = atof(val);
			if (lat_d < -90.0 || lat_d > 90.0) {
				snprintf(reply, reply_size, "ERR lat must be -90..90");
			} else {
				_creds->lat_e6 = (int32_t)(lat_d * 1e6);
				observer_creds_save(_creds, _store->getBasePath());
				snprintf(reply, reply_size, "lat=%.6f", lat_d);
			}

		} else if ((val = find_val(rest, "lon")) != nullptr) {
			double lon_d = atof(val);
			if (lon_d < -180.0 || lon_d > 180.0) {
				snprintf(reply, reply_size, "ERR lon must be -180..180");
			} else {
				_creds->lon_e6 = (int32_t)(lon_d * 1e6);
				observer_creds_save(_creds, _store->getBasePath());
				snprintf(reply, reply_size, "lon=%.6f", lon_d);
			}

		} else {
			snprintf(reply, reply_size, "ERR unknown key");
		}
		return false;
	}

	snprintf(reply, reply_size, "ERR unknown command (type 'help')");
	return false;
}

/* ========== Self-advert ========== */

void ObserverMesh::publishSelfAdvert()
{
	if (!_creds || (_creds->lat_e6 == 0 && _creds->lon_e6 == 0)) {
		LOG_DBG("publishSelfAdvert: lat/lon not configured, skipping");
		return;
	}
	/* Skip if the node name is still the auto-generated default ("Observer-XXXXXXXX").
	 * The user must have set a meaningful name before advertising. */
	if (strncmp(_prefs.node_name, "Observer-", 9) == 0) {
		LOG_DBG("publishSelfAdvert: name is default, skipping");
		return;
	}

	/* ---- Build raw wire frame (zero-hop advert) ----
	 *
	 * Byte 0:    header = ROUTE_TYPE_DIRECT | (PAYLOAD_TYPE_ADVERT << PH_TYPE_SHIFT)
	 * Byte 1:    path_len_byte = 0  (zero hops)
	 * Bytes 2..33:  pubkey (32)
	 * Bytes 34..37: timestamp (uint32 LE)
	 * Bytes 38..101: signature (64) — signed over pubkey + ts + appdata
	 * Byte 102:  flags = Type:Chat(1) | HasLocation(0x10) | HasName(0x80)
	 * Bytes 103..106: lat_e6 (int32 LE)
	 * Bytes 107..110: lon_e6 (int32 LE)
	 * Bytes 111..: node name (null-terminated)
	 */

	uint8_t raw[192];
	int pos = 0;

	/* Header and zero-hop path */
	raw[pos++] = (uint8_t)((PAYLOAD_TYPE_ADVERT << PH_TYPE_SHIFT) | ROUTE_TYPE_DIRECT);
	raw[pos++] = 0x00;  /* path_len_byte = 0 → zero hop */

	/* Payload: pubkey */
	int pubkey_off = pos;
	memcpy(&raw[pos], _self_id.pub_key, PUB_KEY_SIZE); pos += PUB_KEY_SIZE;

	/* Payload: timestamp */
	int ts_off = pos;
	uint32_t now_ts = _rtc ? (uint32_t)_rtc->getCurrentTime() : 0;
	raw[pos++] = (uint8_t)(now_ts);
	raw[pos++] = (uint8_t)(now_ts >>  8);
	raw[pos++] = (uint8_t)(now_ts >> 16);
	raw[pos++] = (uint8_t)(now_ts >> 24);

	/* Payload: signature placeholder (64 bytes, filled below) */
	int sig_off = pos;
	memset(&raw[pos], 0, SIGNATURE_SIZE); pos += SIGNATURE_SIZE;

	/* AppData: flags | lat | lon | name */
	int appdata_off = pos;
	raw[pos++] = 0x01 | 0x10 | 0x80;  /* Type=Chat, HasLocation, HasName */

	int32_t lat = _creds->lat_e6;
	raw[pos++] = (uint8_t)(lat);
	raw[pos++] = (uint8_t)(lat >>  8);
	raw[pos++] = (uint8_t)(lat >> 16);
	raw[pos++] = (uint8_t)(lat >> 24);

	int32_t lon = _creds->lon_e6;
	raw[pos++] = (uint8_t)(lon);
	raw[pos++] = (uint8_t)(lon >>  8);
	raw[pos++] = (uint8_t)(lon >> 16);
	raw[pos++] = (uint8_t)(lon >> 24);

	const char *name = _prefs.node_name;
	size_t name_len = strlen(name);
	if (name_len >= sizeof(raw) - pos - 1) {
		name_len = sizeof(raw) - pos - 2;
	}
	memcpy(&raw[pos], name, name_len);
	pos += name_len;
	raw[pos++] = '\0';

	int raw_len = pos;

	/* Sign: pubkey + timestamp + appdata */
	uint8_t sign_input[PUB_KEY_SIZE + 4 + 128];  /* generous upper bound */
	memcpy(sign_input,                  &raw[pubkey_off], PUB_KEY_SIZE);
	memcpy(sign_input + PUB_KEY_SIZE,   &raw[ts_off],     4);
	memcpy(sign_input + PUB_KEY_SIZE + 4, &raw[appdata_off], raw_len - appdata_off);
	_self_id.sign(&raw[sig_off], sign_input, (size_t)(PUB_KEY_SIZE + 4 + raw_len - appdata_off));

	/* ---- Build JSON ---- */
	char raw_hex[sizeof(raw) * 2 + 1];
	Utils::toHex(raw_hex, raw, raw_len);
	raw_hex[raw_len * 2] = '\0';

	size_t json_cap;
	char *json_buf = mqtt_publisher_stage(&json_cap);
	struct MeshcorePacketJson pj = {
		name,
		_pubkey_hex,
		now_ts,
		raw_len,
		(unsigned)PAYLOAD_TYPE_ADVERT,
		"D",
		(unsigned)(raw_len - 2),  /* payload_len = raw_len minus header + path_len_byte */
		raw_hex,
		0,                        /* SNR (locally originated) */
		0,                        /* RSSI */
		0,                        /* score */
		"0000000000000000",       /* hash (not computed for self-advert) */
	};
	int json_len = meshcore_build_packet_json(json_buf, json_cap, &pj);

	if (json_len < 0 || json_len >= (int)json_cap) {
		LOG_WRN("publishSelfAdvert: JSON truncated");
		return;
	}

	mqtt_publisher_commit(MQTT_PUB_TOPIC_PACKETS, json_len);
	LOG_INF("Self-advert published (lat=%.6f lon=%.6f)",
		(double)_creds->lat_e6 / 1e6, (double)_creds->lon_e6 / 1e6);
}

} /* namespace mesh */
