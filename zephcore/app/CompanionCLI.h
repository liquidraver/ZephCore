/*
 * SPDX-License-Identifier: MIT
 * CompanionCLICallbacks: the companion's side of CommonCLI (prefs, identity,
 * radio, GPS, stats). main_companion owns the instance and the CommonCLI.
 */

#pragma once

#include <helpers/CommonCLI.h>
#include <helpers/StatsFormatHelper.h>
#include <ZephyrDataStore.h>
#include <ZephyrBoard.h>
#include <adapters/radio/LoRaRadio.h>
#include <adapters/gps/ZephyrGPSManager.h>
#include "CompanionMesh.h"

class CompanionCLICallbacks : public CommonCLICallbacks {
	ZephyrDataStore& _store;
	CompanionMesh& _mesh;
	mesh::LoRaRadio& _radio;
	mesh::ZephyrBoard& _board;
	mesh::MillisecondClock& _ms;
	mesh::PacketManager& _pool;
	bool (*_tx_idle)(void);

public:
	/* tx_idle: the transport has sent everything queued (main_companion
	 * knows the pending reply and which transport is active). */
	CompanionCLICallbacks(ZephyrDataStore& store, CompanionMesh& mesh, mesh::LoRaRadio& radio,
			      mesh::ZephyrBoard& board, mesh::MillisecondClock& ms,
			      mesh::PacketManager& pool, bool (*tx_idle)(void))
		: _store(store), _mesh(mesh), _radio(radio), _board(board), _ms(ms), _pool(pool),
		  _tx_idle(tx_idle) {}

	void savePrefs() override {
		_store.savePrefs(_mesh.prefs);
	}
	const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
	const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
	const char* getRole() override { return "companion"; }
	/* CLI `erase`: the factory reset, which keeps the volume marked as ours
	 * so the next boot does not format it a second time. */
	bool formatFileSystem() override { return _store.factoryReset(); }

	/* Same path as the app's advert command; delay_millis is unused. */
	void sendSelfAdvertisement(int delay_millis, bool flood) override {
		(void)delay_millis;
		_mesh.sendSelfAdvert(flood);
	}
	void updateAdvertTimer() override {}
	void updateFloodAdvertTimer() override {}

	/* Reboot gate: true once the app has everything we owe it, including a
	 * held-back v-contact ack, so a `reboot` typed there waits for its ack. */
	bool transportTxIdle() override {
		return !_mesh.vcontactConfirmPending() && _tx_idle();
	}

	/* No log file on a companion. */
	void setLoggingOn(bool enable) override { (void)enable; }
	void eraseLogFile() override {}
	void dumpLogFile() override {}

	/* No runtime TX-power API in the LoRa driver; log only, as the repeater. */
	void setTxPower(int8_t power_dbm) override {
		LOG_INF("TX power %d dBm requested (reboot to apply)", power_dbm);
	}

	bool setRxBoostedGain(bool enable) override {
		return _radio.setRxBoost(enable);
	}

	bool setFemRxGain(bool enable) override {
		return _radio.setFemRxEnable(enable);
	}

	bool configSideDetectors(const uint8_t* sfs, uint8_t num) override {
		return _radio.configSideDetectors(sfs, num);
	}

	/* Adaptive CAD */
	int formatFreqErrorStatus(char* buf, int cap) override {
		return _radio.formatFreqErrorStatus(buf, cap);
	}
	int formatCadStatus(char* buf, int cap) override {
		return _radio.formatCadStatus(buf, cap);
	}
	void applyCadPrefs() override {
		_radio.setCadParams(_mesh.prefs.cad_auto != 0,
					_mesh.prefs.cad_offset,
					_mesh.prefs.probe_interval,
					_mesh.prefs.cad_busycap,
					_mesh.prefs.cad_base);
		_mesh.prefs.cad_offset = _radio.getCadOffset();
		_mesh.prefs.cad_base = _radio.cadBasePeak();
	}
	void resetCadStats() override {
		_radio.resetCadStats();
	}

	mesh::LocalIdentity& getSelfId() override { return _mesh.self_id; }

	void saveIdentity(const mesh::LocalIdentity& new_id) override {
		_mesh.self_id = new_id;
		_store.saveMainIdentity(new_id);
	}

	void clearStats() override {
		_radio.resetStats();
		_radio.resetDutyCycleTimeoutRestarts();
		_mesh.resetStats();
	}

	/* Duty-cycle false-preamble re-arm count.  Without these the
	 * CommonCLICallbacks default answers 0 forever, so `get dc.restarts`
	 * read clean on every companion regardless of what the radio was doing
	 * — and the companion is the role the duty cycle actually runs in.
	 * Mirrors RepeaterMesh::getDutyCycleTimeoutRestarts(). */
	uint32_t getDutyCycleTimeoutRestarts() const override {
		return _radio.getDutyCycleTimeoutRestarts();
	}

	void resetDutyCycleTimeoutRestarts() override {
		_radio.resetDutyCycleTimeoutRestarts();
	}

	/* Temp radio params — deferred; stub for now. */
	void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr,
				  int timeout_mins) override {
		(void)freq; (void)bw; (void)sf; (void)cr; (void)timeout_mins;
	}

	MeshTimeSync* getMeshTimeSync() override {
		return _mesh.getMeshTimeSync();
	}

	/* GPS, through the GPS manager as on the repeater. */
	bool setGpsEnabled(bool enabled) override {
		if (!gps_is_available()) return false;
		gps_enable(enabled);
		return true;
	}
	bool isGpsEnabled() const override {
		return gps_is_enabled();
	}
	void formatGpsStatsReply(char* reply) override {
		if (!gps_is_enabled()) {
			strcpy(reply, "off");
			return;
		}
		struct gps_state_info gsi;
		gps_get_state_info(&gsi);
		static const char* const state_str[] = { "off", "standby", "acquiring" };
		const char* state = gsi.state < 3 ? state_str[gsi.state] : "unknown";
		struct gps_position pos;
		bool has_pos = gps_get_last_known_position(&pos);
		if (has_pos) {
			snprintf(reply, CLI_REPLY_SIZE,
				"on state=%s sats=%u fix=%us ago lat=%.6f lon=%.6f",
				state, gsi.satellites, gsi.last_fix_age_s,
				pos.latitude_ndeg / 1e9, pos.longitude_ndeg / 1e9);
		} else if (gsi.next_search_s > 0) {
			snprintf(reply, CLI_REPLY_SIZE,
				"on state=%s sats=%u no fix next=%us",
				state, gsi.satellites, gsi.next_search_s);
		} else {
			snprintf(reply, CLI_REPLY_SIZE,
				"on state=%s sats=%u no fix",
				state, gsi.satellites);
		}
	}

	/* stats-core / stats-radio / stats-packets, as the repeater prints them. */
	void formatStatsReply(char* reply) override {
		StatsFormatHelper::formatCoreStats(reply, _board, _ms,
			_mesh.getErrFlags(), &_pool);
	}
	void formatRadioStatsReply(char* reply) override {
		StatsFormatHelper::formatRadioStats(reply, &_radio, _radio,
			_mesh.getTotalAirTime(), _mesh.getReceiveAirTime());
	}
	void formatPacketStatsReply(char* reply) override {
		StatsFormatHelper::formatPacketStats(reply, _radio,
			_mesh.getNumSentFlood(), _mesh.getNumSentDirect(),
			_mesh.getNumRecvFlood(), _mesh.getNumRecvDirect());
	}
};
