/*
 * ZephCore - Joystick UI Task
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * JoystickUITask: joystick-based UI coordinator for the ZephCore companion.
 *
 * Input thread (Zephyr INPUT_CALLBACK_DEFINE):
 *   → enqueues key chars to _key_queue (k_msgq)
 *
 * Mesh event loop thread (main_companion.cpp):
 *   → calls loop() periodically
 *   → loop() dequeues keys, dispatches to current screen
 *   → renders when needed
 */

#pragma once

#include "joystick_display.h"
#include "joystick_defs.h"
#include "joystick_screens.h"

#include <helpers/BaseChatMesh.h>
#include <helpers/ContactInfo.h>
#include <helpers/ChannelDetails.h>
#include <helpers/NodePrefs.h>
#include <adapters/clock/ZephyrRTCClock.h>

#include <zephyr/kernel.h>
#include <stdint.h>

/* Message preview ring for channel messages */
#ifndef JOYSTICK_OFFLINE_QUEUE_SIZE
#define JOYSTICK_OFFLINE_QUEUE_SIZE 16
#endif

/* Key event queue depth */
#define JOYSTICK_KEY_QUEUE_DEPTH  24

class JoystickUITask {
public:
	JoystickUITask();

	/* Call once after mesh/prefs are loaded */
	void begin(BaseChatMesh *mesh, mesh::ZephyrRTCClock *rtc, NodePrefs *prefs);

	/* Call from mesh event loop: drains the key queue, dispatches lifecycle
	 * events on display-state transitions, and renders when due. */
	void loop();

	/* Called from CompanionMesh callbacks (mesh thread context) */
	void newMsg(uint8_t path_len, const char *from_name, const char *text, int msgcount);
	void newChannelMsg(const char *channel_name, const char *text, uint32_t ts, uint8_t path_len);
	void notify(); /* BLE connect/disconnect, ACK, etc. */

	/* Alert overlay */
	void showAlert(const char *text, int duration_ms);

	/* Screen navigation (called from screen handleInput()) */
	void gotoHomeScreen();
	void gotoContactsScreen();
	void gotoAdvertScreen();
	void gotoGPSScreen();
	void gotoSystemScreen();
	void gotoSystemTimeScreen();
	void gotoTelemetryScreen();
	void gotoToolsScreen();
	void gotoRadioStatsScreen();
	void gotoRepeatersScreen();
	void gotoChannelsScreen();
	void gotoT9InputScreen();
	void gotoT9InputScreenWithPrefix(const char *prefix);
	void gotoRenameNodeScreen();
	void gotoBLECodeScreen();
	void gotoStatsScreen();
	void gotoStopwatchScreen();
	void gotoCountdownScreen();
	void gotoSnakeScreen();
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	void gotoDoomScreen();
#endif
	void gotoUnreadScreen();
	void gotoContactForAdvert(const uint8_t *prefix, int prefix_len);
	void gotoRepeaterAdminScreen(const uint8_t *pub_key, const char *name, bool from_contacts = false);

	void forceRefresh() { _next_refresh = 0; }

	/* State accessors for screens */
	uint16_t getCachedBattMilliVolts() const { return _cached_batt_mv; }
	int getBatteryDisplayMode() const { return _battery_display_mode; }
	void toggleBatteryDisplayMode() { _battery_display_mode = (_battery_display_mode + 1) % 2; }
	uint32_t getScreenOffMillis() const { return _screen_off_ms; }
	void adjustScreenOff(int32_t delta_ms);
	uint8_t getBrightness() const { return _brightness; }
	void adjustBrightness(int delta);
	bool getWakeOnMsg() const { return _wake_on_msg; }
	void toggleWakeOnMsg();
	/* Path-hash-mode: 0/1/2 → 1/2/3 bytes per hop appended to outbound flood
	 * path. Cycles through the three valid settings; persists to prefs. */
	uint8_t getPathHashBytes() const { return _prefs ? (uint8_t)(_prefs->path_hash_mode + 1) : 1; }
	void cyclePathHashMode();
	bool isBuzzerQuiet() const;
	void toggleBuzzer();
	bool isGPSAvailable() const;
	bool getGPSState() const;
	void toggleGPS();
	/* GPS duty-cycle interval, seconds (0 = always on). Steps through a fixed
	 * preset ladder (step = +1/-1) rather than raw seconds, since the valid
	 * range spans 0..604800 — a linear per-press delta would be unusable at
	 * either end. Applied live + persists to prefs. */
	uint32_t getGpsDutySec() const;
	void adjustGpsDuty(int step);
	bool isSerialEnabled() const { return _ble_enabled; }
	void disableSerial();
	void enableSerial();
	bool isOffgridEnabled() const { return _prefs && _prefs->client_repeat != 0; }
	void setOffgridMode(bool enable);
	bool isLedsDisabled() const { return _prefs && _prefs->leds_disabled != 0; }
	void toggleLeds();
	void rebootToDFU();
	void shutdown(bool restart = false);
	void playCountdownAlarm();

	/* Message / compose accessors */
	int getUnreadCount();
	int getStoredMsgCount() const;
	int getContactMsgCount(const char *contact_name) const;
	bool getContactMsgAt(const char *contact_name, int idx, const char *&out_msg,
		uint32_t &out_ts, uint8_t *out_path = nullptr) const;
	int getChannelPreviewCountFor(const char *channel) const;
	bool getChannelPreviewFor(const char *channel, int idx, const char *&text,
		uint32_t &ts, uint8_t *path_len = nullptr) const;
	bool isComposeContact() const { return _compose_is_contact; }
	int getComposeChannelIndex() const { return _compose_channel_idx; }
	const char *getComposeChannelName() const { return _compose_channel_name; }
	const char *getComposeContactName() const { return _compose_contact_name; }
	void setComposeChannel(int idx, const char *name);
	void setComposeContact(const ContactInfo &contact);
	bool sendComposedMessage(const char *text);
	bool sendChannelMessage(const char *text);
	bool findContactByName(const char *name, ContactInfo &contact);
	int getRecentlyHeard(AdvertPath *dest, int max) const;
	bool getDiscoverSignal(const uint8_t *pubkey, int8_t &snr_out,
		uint8_t *path_len_out = nullptr) const;
	void clearDiscoverSignals();
	int getDiscoverCount() const;
	bool getDiscoverByIdx(int idx, uint8_t *pubkey_out, int8_t &snr_out, int8_t &snr_remote_out,
		uint8_t *path_len_out = nullptr) const;
	bool addRepeaterContact(const uint8_t *pubkey);
	void cancelContactPing();

	/* Mesh access (for screens, safe in mesh thread) */
	BaseChatMesh *getMesh() const { return _mesh; }
	mesh::ZephyrRTCClock *getRTC() const { return _rtc; }
	NodePrefs *getPrefs() const { return _prefs; }
	JoystickDisplay &getDisplay() { return _display; }

	/* Battery cache (updated via ui_set_battery from ui_refresh_battery's render-path call) */
	void setCachedBattMilliVolts(uint16_t mv) { _cached_batt_mv = mv; }

	/* Radio stats (fed from main companion loop) */
	void setNoiseFloor(int16_t nf) { _noise_floor = nf; }
	void setPacketStats(uint32_t recv, uint32_t sent, uint32_t errors) {
		_pkt_recv = recv; _pkt_sent = sent; _pkt_errors = errors;
	}
	int16_t getNoiseFloor() const { return _noise_floor; }
	uint32_t getPktRecv() const { return _pkt_recv; }
	uint32_t getPktSent() const { return _pkt_sent; }
	uint32_t getPktErrors() const { return _pkt_errors; }

	/* BLE status (fed from main companion loop) */
	void setBLEConnected(bool connected) { _ble_connected = connected; }
	bool isBLEConnected() const { return _ble_connected; }
	void setBLEEnabled(bool enabled) { _ble_enabled = enabled; }

	/* RepeaterAdmin callbacks */
	void onRepeaterAdminLoginResult(const uint8_t *pub_key_prefix, bool success,
		uint8_t permissions, uint32_t server_time);
	void onRepeaterAdminCliResponse(const uint8_t *pub_key_prefix, const char *text);

	/* Repeater events (dispatched from joystick_ui_hooks.cpp) */
	void onRepeaterReqResponse(const uint8_t *pub_key_prefix,
		int8_t snr_local, int8_t snr_remote, const uint8_t *data = nullptr, uint8_t data_len = 0);
	void onRepeaterDiscoverResp(const uint8_t *pub_key, int8_t snr, int8_t snr_remote, uint8_t path_len);

	bool sendContactPingZeroHop(const ContactInfo &contact, uint32_t &est_timeout);
	void registerAdminReqTag(uint32_t tag);
	void clearAdminReqTag();
	void notifyPacketSent();

	/* Key event enqueue (called from input callback, ISR/input thread) */
	static void enqueueKey(char key);

	/* Set by joystick_ui_hooks after registration */
	static void setSignalFn(void (*fn)(void));
	static void setScheduleRenderFn(void (*fn)(uint32_t delay_ms));

private:
	static void (*s_signal_fn)(void);
	static void (*s_schedule_render_fn)(uint32_t);
	JoystickDisplay _display;
	BaseChatMesh *_mesh;
	mesh::ZephyrRTCClock *_rtc;
	NodePrefs *_prefs;

	/* Screen instances */
	SplashScreen *_splash;
	HomeScreen *_home;
	ContactsScreen *_contacts;
	UnreadScreen *_unread;
	AdvertScreen *_advert;
	GPSSettingsScreen *_gps;
	SystemScreen *_system;
	SystemTimeScreen *_system_time;
	TelemetryScreen *_telemetry;
	ToolsScreen *_tools;
	RadioStatsScreen *_radio_stats;
	RepeatersScreen *_repeaters;
	ChannelsScreen *_channels;
	T9InputScreen *_t9_input;
	RenameNodeScreen *_rename_node;
	BLECodeScreen *_ble_code;
	StatsScreen *_stats;
	StopwatchScreen *_stopwatch;
	CountdownScreen *_countdown;
	SnakeScreen *_snake;
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	DoomScreen *_doom;
#endif
	RepeaterAdminScreen *_repeater_admin;
	UIScreen *_curr;  /* polymorphic — points at whichever subclass is active */

	void setCurrScreen(UIScreen *s);
	void applyBrightness();

	/* Timing */
	uint32_t _next_refresh;
	static constexpr uint32_t JOYSTICK_RENDER_MIN_MS = 40; /* min gap between input blits */
	uint32_t _last_blt_ms;
	bool _pending_render;
	bool _render_immediate;
	uint32_t _screen_off_ms;       /* user setting; display.c owns the auto-off timer */
	struct k_timer _lock_timer;    /* one-shot, fires LOCK_AFTER_MS after last activity */
	static void lockTimerCb(struct k_timer *t);
	void scheduleLockTimer();
	bool _was_display_on;          /* previous display state for transition detection */
	void onDisplayStateChanged();  /* dispatches onDisplayOff/On to _curr on transition */

	/* State */
	uint16_t _cached_batt_mv;
	int _battery_display_mode;
	uint8_t _brightness;
	bool _wake_on_msg;
	bool _ble_connected;
	bool _ble_enabled;
	int16_t _noise_floor;
	uint32_t _pkt_recv, _pkt_sent, _pkt_errors;

	/* Ping state, owned here not in CompanionMesh */
	uint8_t _pending_ping_pubkey[4];
	uint32_t _pending_ping_sent_ms;

	/* Outgoing DM tracking — retries on missing ACK, then forces flood as
	 * a last-ditch attempt, then marks failed. The BLE-app mirror is
	 * deferred until the outcome is known so the phone sees the message
	 * exactly once with a delivery indicator in the body prefix. */
	static const int MAX_PENDING_SENDS = 4;
	static const uint8_t MAX_SEND_ATTEMPTS = 5;  /* attempt 4 (0-indexed) forces flood */
	struct PendingSend {
		JoystickUITask *task;        /* back-pointer for ISR dispatch */
		bool active;
		volatile bool retry_due;     /* set by timer ISR, consumed in loop() */
		bool delivered;
		bool failed;
		uint8_t recipient_pubkey[PUB_KEY_SIZE];
		char recipient_name[32];
		uint32_t expected_ack;
		uint32_t timestamp;
		char text[MAX_TEXT_LEN + 1];
		uint8_t attempt;
		uint32_t timeout_ms;
		struct k_timer retry_timer;
	};
	PendingSend _pending_sends[MAX_PENDING_SENDS];
	static void pendingRetryTimerCb(struct k_timer *t);
	int allocPendingSendSlot();
	void schedulePendingRetry(int slot_idx);
	void doPendingSend(int slot_idx);
	void completePendingSend(int slot_idx);
	void processPendingRetries();

	/* Channel-send feedback: after broadcasting a group message we wait
	 * CHANNEL_FEEDBACK_WINDOW_MS to see if any neighbor repeated our flood
	 * (queried via ContentionTracker::extractDupeCount). Outcome decides
	 * the body prefix in the BLE-app mirror and the path_len indicator in
	 * the local _ch_previews entry. */
	static const int MAX_PENDING_CHANNEL_SENDS = 4;
	static constexpr uint32_t CHANNEL_FEEDBACK_WINDOW_MS = 5000;
	struct PendingChannelSend {
		JoystickUITask *task;
		bool active;
		volatile bool feedback_due;
		uint8_t channel_idx;
		int preview_index;          /* index in _ch_previews ring to update */
		uint32_t preview_timestamp; /* extra match guard if ring wrapped */
		uint32_t timestamp;
		uint32_t hash;
		char text[MAX_TEXT_LEN + 1];
		struct k_timer feedback_timer;
	};
	PendingChannelSend _pending_channel_sends[MAX_PENDING_CHANNEL_SENDS];
	static void pendingChannelFeedbackCb(struct k_timer *t);
	int allocPendingChannelSlot();
	void processPendingChannelFeedback();
	void completePendingChannelSend(int slot_idx, bool heard);
public:
	/* Initiate a DM with retry tracking (called by sendComposedMessage). */
	bool startPendingDM(ContactInfo &recipient, uint32_t ts, const char *text);
	/* Try to match an arriving ACK to a pending send.
	 * Returns the recipient ContactInfo* if matched (so BaseChatMesh can do
	 * its return-path-retry housekeeping), nullptr otherwise. */
	ContactInfo *tryMatchPendingAck(uint32_t ack);
	/* Initiate a channel send with feedback tracking. Returns true on
	 * successful broadcast; the body of the message is stored locally as
	 * "pending" until the feedback window expires. */
	bool startPendingChannel(uint8_t channel_idx, ChannelDetails &ch, uint32_t ts, const char *text);
private:

	/* Discover signal cache, owned here, populated via onRepeaterDiscoverResp */
	static constexpr int DISCOVER_SIGNAL_TABLE_SIZE = 16;
	struct DiscoverSignal {
		uint8_t pubkey[PUB_KEY_SIZE];
		uint8_t path_len;
		int8_t snr;		   /* SNR FROM repeater (response as received by us) */
		int8_t snr_remote; /* SNR TO repeater (request as received by repeater) */
		bool valid;
	};
	DiscoverSignal _discover_signals[DISCOVER_SIGNAL_TABLE_SIZE];

	/* Alert overlay */
	char _alert[80];
	uint32_t _alert_expiry;
	void renderAlertOverlay();

	/* Screen lock */
	static constexpr uint32_t LOCK_AFTER_MS = 60000UL;
	bool _locked;
	uint8_t _lock_step;
	void renderLockOverlay();
	void handleLockInput(char key, uint32_t now);

	/* Compose state */
	bool _compose_is_contact;
	int _compose_channel_idx;
	char _compose_channel_name[32];
	char _compose_contact_name[32];
	uint8_t _compose_contact_pubkey[PUB_KEY_SIZE];

	/* Channel message previews (ring buffer) */
	struct ChannelMsgPreview {
		char channel[32];
		char text[80];
		uint32_t timestamp;
		uint8_t path_len;
	};
	ChannelMsgPreview _ch_previews[JOYSTICK_OFFLINE_QUEUE_SIZE];
	int _ch_preview_count;
	int _ch_preview_head;

	/* Key event queue (k_msgq backed) */
	static char _key_buf[JOYSTICK_KEY_QUEUE_DEPTH * sizeof(char)];
	static struct k_msgq _key_queue;

	/* Startup */
	bool _initialized;
};
