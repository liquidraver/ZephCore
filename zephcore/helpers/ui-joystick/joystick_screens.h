/*
 * ZephCore - Joystick UI Screens
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * All screen class declarations for the joystick UI.
 */

#pragma once

#include "joystick_display.h"
#include "joystick_defs.h"
#include <helpers/ContactInfo.h>
#include <helpers/ChannelDetails.h>
#include <mesh/Mesh.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "CompanionMesh.h"

class JoystickUITask;

/* ===== UIScreen base class =====
 * Lifecycle:
 *   onEnter()      — called by setCurrScreen when this screen becomes
 *                    active. Start any per-screen k_timer here.
 *   onExit()       — called when navigating away. Stop any per-screen
 *                    k_timer here so it can't fire on a stale screen.
 *   onDisplayOff() — called when the display turns off while this screen
 *                    is active. Pause periodic timers whose work is
 *                    invisible while the screen is off (e.g. game ticks,
 *                    GPS sampling). Keep one-shot timers running if their
 *                    purpose is to fire while idle (alarms, timeouts).
 *   onDisplayOn()  — counterpart of onDisplayOff; resume what you paused.
 *   render()       — draws the current state. Triggered by signals (key
 *                    event, mesh event, screen-owned timer fire), never
 *                    polled.
 *   handleInput()  — receives one queued key character.
 */
class UIScreen {
public:
	virtual ~UIScreen() {}
	virtual int render(JoystickDisplay &display) = 0;
	virtual bool handleInput(char c) { (void)c; return false; }
	virtual void onEnter() {}
	virtual void onExit() {}
	virtual void onDisplayOff() {}
	virtual void onDisplayOn() {}
};

/* ===== Admin command size limits ===== */
#ifndef ADMIN_PASSWORD_MAX
#define ADMIN_PASSWORD_MAX	 32
#endif
#define ADMIN_CMD_MAX		 48
#define ADMIN_RESP_MAX		 256
#define ADMIN_HIST_MAX		 6

/* ===== SplashScreen ===== */
class SplashScreen : public UIScreen {
	JoystickUITask *_task;
	uint32_t _dismiss_after;
	struct k_timer _dismiss_timer;
	static void dismissTimerCb(struct k_timer *t);
public:
	SplashScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	void onEnter() override;
	void onExit() override;
};

/* ===== HomeScreen ===== */
class HomeScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
public:
	HomeScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== AdvertScreen ===== */
class AdvertScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
	int _last_rendered_selected;
	uint32_t _marquee_start_ms;
	bool _settings_open;
	struct AdvertPath _recent_adverts[16];
	int _recent_advert_count;
	uint32_t _recent_refresh_at;

	void refreshRecentAdverts();
	int getRecentAdvertCount() const;
	bool getRecentAdvertByIndex(int idx, struct AdvertPath &path) const;
public:
	AdvertScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== GPSSettingsScreen ===== */
class GPSSettingsScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
	bool _have_prev_fix;
	int32_t _prev_lat_e6, _prev_lon_e6;
	uint32_t _last_sample_ms;
	float _speed_kmh, _heading_deg;
	bool _heading_valid;
	uint32_t _heading_hold_until;
	bool _show_alt;
	uint32_t _alt_cycle_ms;
	struct k_timer _sample_timer;
	static void sampleTimerCb(struct k_timer *t);
	void sampleGPS();
public:
	GPSSettingsScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onEnter() override;
	void onExit() override;
	void onDisplayOff() override;
	void onDisplayOn() override;
};

/* ===== SystemScreen ===== */
enum SystemMode { SYSMODE_TOP=0, SYSMODE_DEVICE, SYSMODE_DISPLAY, SYSMODE_INFO, SYSMODE_POWER, SYSMODE_OFFGRID_CONFIRM };
class SystemScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected, _cat_selected;
	SystemMode _mode;
	uint32_t _dfu_confirm_time;
public:
	SystemScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== SystemTimeScreen ===== */
class SystemTimeScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
public:
	SystemTimeScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== TelemetryScreen ===== */
class TelemetryScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
public:
	TelemetryScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== ToolsScreen ===== */
class ToolsScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
public:
	ToolsScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== RadioStatsScreen ===== */
class RadioStatsScreen : public UIScreen {
	JoystickUITask *_task;
public:
	RadioStatsScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== RepeatersScreen ===== */
class RepeatersScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
	int _mode;
	int _last_sel;
	uint32_t _marquee_start_ms;
	int _modal_idx;

	void doScan();
public:
	RepeatersScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onEnter() override;
};

/* ===== ChannelsScreen ===== */
class ChannelsScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
	bool _show_msgs;
	char _msg_channel[32];
	int _msg_channel_idx;
	int _msg_scroll;
	bool _msg_details;
	int _msg_detail_scroll;

	int getChannelCount() const;
	bool getChannelByListIndex(int listIdx, ChannelDetails &ch, int *slot_out = nullptr) const;
public:
	ChannelsScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== T9InputScreen ===== */
class T9InputScreen : public UIScreen {
	JoystickUITask *_task;
	char _input[256];
	int _cursor;
	int _selected_key;
	uint32_t _last_press_time;
	int _letter_index;
	int _last_key;
	bool _confirm_exit;
	int _confirm_selected;
	bool _kb_mode_letters;

	void addLetter(char ch);
	void backspace();
public:
	T9InputScreen(JoystickUITask *task);
	void clearInput() { memset(_input, 0, sizeof(_input)); _cursor = 0; }
	void setInitialInput(const char *text) {
		if (!text || !text[0]) return;
		strncpy(_input, text, sizeof(_input) - 1);
		_input[sizeof(_input) - 1] = '\0';
		_cursor = (int)strlen(_input);
		_last_key = -1; _letter_index = 0; _last_press_time = 0;
	}
	const char *getInput() const { return _input; }
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== RenameNodeScreen ===== */
class RenameNodeScreen : public UIScreen {
	JoystickUITask *_task;
	char _input[32];
	bool _loaded;
	int _cursor;
	int _selected_key;
	uint32_t _last_press_time;
	int _letter_index;
	int _last_key;
	bool _kb_mode_letters;
public:
	RenameNodeScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== BLECodeScreen ===== */
class BLECodeScreen : public UIScreen {
	JoystickUITask *_task;
	char _pin_buf[8];
	bool _editing;
	int _digit_sel;
public:
	BLECodeScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== StatsScreen ===== */
class StatsScreen : public UIScreen {
	JoystickUITask *_task;
public:
	StatsScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== StopwatchScreen ===== */
class StopwatchScreen : public UIScreen {
	JoystickUITask *_task;
	bool _running;
	uint32_t _start_ms;
	uint32_t _elapsed_ms;
	int _laps;
	uint32_t _lap_times[10];
	int _scroll;
public:
	StopwatchScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
};

/* ===== CountdownScreen ===== */
class CountdownScreen : public UIScreen {
	JoystickUITask *_task;
	bool _running;
	uint32_t _end_ms;
	int _set_seconds;
	int _edit_field;
	bool _alarmed;
	struct k_timer _alarm_timer;
	static void alarmTimerCb(struct k_timer *t);
public:
	CountdownScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onExit() override;
};

/* ===== SnakeScreen ===== */
class SnakeScreen : public UIScreen {
	JoystickUITask *_task;
	/* runtime grid */
	int _grid_w;
	int _grid_h;
	int _max_len;
	/* snake storage (safe upper bound) */
	static constexpr int MAX_SNAKE_LEN = 1024;
	int8_t _snake_x[MAX_SNAKE_LEN];
	int8_t _snake_y[MAX_SNAKE_LEN];
	int _snake_len;
	int8_t _food_x, _food_y;
	uint32_t _next_move;
	int _score;
	bool _grid_ready;
	struct k_timer _tick_timer;
	volatile bool _tick_due;  /* set in timer ISR, consumed in render */
	static void tickTimerCb(struct k_timer *t);

	void updateGrid(JoystickDisplay &display);
	void placeFood();
	void reset();
	void advanceGame();
	void startTicking();
public:
	SnakeScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onEnter() override;
	void onExit() override;
	void onDisplayOff() override;
	void onDisplayOn() override;
};

/* ===== DoomScreen ===== */
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
class DoomScreen : public UIScreen {
	JoystickUITask *_task;
public:
	DoomScreen(JoystickUITask *task);
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onEnter() override;
};
#endif

/* ===== ContactsScreen ===== */
class ContactsScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	int _selected;
	int _filter;
	int _mode;
	int _submenu_selected;
	int _chat_selected;
	bool _chat_details;
	int _chat_detail_scroll;
	ContactInfo _active_contact;
	bool _active_contact_valid;
	int _idx_send_message;
	int _idx_edit_path;
	int _idx_reset_path;
	int _idx_favorite;
	int _idx_delete;
	int _idx_repeater_admin;
	int _idx_ping_zerohop;

	/* Edit path T9 keyboard state */
	char _editpath_hexbuf[MAX_PATH_SIZE * 2 + 1];
	int _editpath_cursor;
	int _editpath_t9_sel;
	int _editpath_t9_last_key;
	int _editpath_t9_letter_index;
	uint32_t _editpath_t9_last_press;
	bool _editpath_kb_letters;
	bool _editpath_confirm_exit;
	int _editpath_confirm_sel;
	uint32_t _header_marquee_ms;

	/* Ping state (zero hop) */
	uint32_t _ping_sent_at;		  /* k_uptime_get_32() when ping was sent, 0=idle */
	uint32_t _ping_timeout_ms;	  /* est_timeout from sendRequest (ms), 0=unknown */
	int8_t _ping_snr_local;		/* SNR (dB) of reply as received by us */
	int8_t _ping_snr_remote;	/* SNR of our ping as received by repeater, INT8_MIN if unknown */
	uint32_t _ping_rtt_ms;		  /* RTT: 0=no result yet, UINT32_MAX=timeout, else ms */
	bool _ping_modal_active;  /* modal overlay visible */
	struct k_timer _ping_timeout_timer;
	static void pingTimeoutCb(struct k_timer *t);

	int clampStart(int contactCount) const;
	int getFilteredContactCount() const;
	bool getFilteredContactByIndex(int listIndex, ContactInfo &contact) const;
	bool refreshActiveContact();
	int buildSubmenuItems(const char *items[], char text[][48], int max_items);

public:
	ContactsScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	void openForPubKey(const uint8_t *prefix, int prefix_len);
	void onPingResponse(int8_t snr_local, int8_t snr_remote, uint32_t rtt_ms);
	void onPacketSent();
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onExit() override;
};

/* ===== UnreadScreen ===== */
class UnreadScreen : public UIScreen {
	JoystickUITask *_task;
	mesh::RTCClock *_rtc;

	struct MsgEntry {
		uint32_t timestamp;
		char origin[32];
		char msg[240];
		bool read;
	};

	static const int MAX_UNREAD_MSGS = 16;
	MsgEntry _entries[MAX_UNREAD_MSGS];
	int _entry_count;
	int _visible_unread_count;
	int _head_index;
	int _selected;
	int _list_scroll;
	bool _details;
	int _detail_scroll;
	bool _transient_preview;
	uint32_t _preview_expiry;
	struct k_timer _preview_timer;
	static void previewTimerCb(struct k_timer *t);

	void normalizeUnreadState();
	const MsgEntry *getByListIndex(int idx) const;
	void markCurrentRead();

public:
	UnreadScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	void activatePreview(bool transient, uint32_t timeout_ms = 10000);
	int getUnreadCount();
	int getStoredMsgCount() const;
	/* @param initially_read true → store in history without counting as unread
	 *                       (used for sent messages, and for received messages
	 *                       when a BLE phone is connected and will sync them). */
	void addPreview(uint8_t path_len, const char *from_name, const char *msg,
			bool initially_read = false);
	/* Update the origin prefix of an existing sent-message entry to reflect
	 * delivery outcome. Looks up by (timestamp, contact_name) — these are
	 * unique-per-second for outgoing DMs. */
	void markSentEntryStatus(uint32_t ts, const char *contact_name, bool delivered);
	bool getStoredMessageForSourceAt(const char *source, uint32_t ts, const char *&out_msg) const;
	bool getLatestStoredMessageForSource(const char *source, const char *&out_msg, uint32_t *out_ts = nullptr) const;
	int getContactMsgCount(const char *contact_name) const;
	bool getContactMsgAt(const char *contact_name, int idx, const char *&out_msg,
						 uint32_t &out_ts, uint8_t *out_path = nullptr) const;
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onExit() override;
};

/* ===== RepeaterAdminScreen ===== */
class RepeaterAdminScreen : public UIScreen {
public:
	enum AdminState	 { STATE_PASSWORD_ENTRY, STATE_LOGGING_IN, STATE_SUBMENU, STATE_MAIN, STATE_CMD_INPUT };
	enum PendingKind { PENDING_NONE, PENDING_CMD, PENDING_BINARY_STATUS, PENDING_BINARY_NEIGHBOURS,
					   PENDING_BINARY_TELEMETRY, PENDING_BINARY_OWNER_INFO };

	RepeaterAdminScreen(JoystickUITask *task, mesh::RTCClock *rtc);
	void openForContact(const uint8_t *pub_key, const char *name, bool from_contacts = false);
	void onLoginResult(bool success, uint8_t permissions, uint32_t server_time);
	void onCliResponse(const char *text);
	void onReqResponse(const uint8_t *pub_key_prefix, const uint8_t *data, uint8_t data_len);
	void onPacketSent();
	int render(JoystickDisplay &display) override;
	bool handleInput(char c) override;
	void onExit() override;

private:
	struct k_timer _timeout_timer;
	static void timeoutTimerCb(struct k_timer *t);
	void onTimeout();   /* called from main thread when _timeout_timer fires */

	JoystickUITask *_task;
	mesh::RTCClock *_rtc;
	AdminState _state;
	uint8_t _contact_pubkey[PUB_KEY_SIZE];
	bool _from_contacts;
	char _repeater_name[32];
	uint32_t _admin_header_marquee_ms;
	uint8_t _permissions;
	uint32_t _server_time;

	/* Password entry */
	char _password[ADMIN_PASSWORD_MAX];
	int _pwd_len;
	int _pwd_t9_sel, _pwd_t9_last_key, _pwd_t9_letter_index;
	uint32_t _pwd_t9_last_press;
	bool _pwd_kb_letters;

	/* Submenu state */
	int _submenu_sel;

	/* Main state */
	PendingKind _pending;
	uint32_t _last_sent_at;
	uint32_t _cmd_timeout_ms;
	bool _awaiting_tx;

	/* Command history ring buffer */
	struct CmdEntry {
		char cmd[ADMIN_CMD_MAX];
		char resp[ADMIN_RESP_MAX];
		bool has_resp;
	};
	CmdEntry _hist[ADMIN_HIST_MAX];
	int _hist_head;
	int _hist_count;
	int _hist_scroll;
	int _resp_line_scroll;
	int _resp_max_line_scroll;

	/* Command compose (STATE_CMD_INPUT) */
	char _cmd_buf[ADMIN_CMD_MAX];
	int _cmd_len;
	int _cmd_t9_sel, _cmd_t9_last_key, _cmd_t9_letter_index;
	uint32_t _cmd_t9_last_press;
	bool _cmd_kb_letters;

	bool sendCLI(const char *cmd);
	void goBack();
	CmdEntry	   &histAt(int i)		{ return _hist[(_hist_head + i) % ADMIN_HIST_MAX]; }
	const CmdEntry &histAt(int i) const { return _hist[(_hist_head + i) % ADMIN_HIST_MAX]; }
};
