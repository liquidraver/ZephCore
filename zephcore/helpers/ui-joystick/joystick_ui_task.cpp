/*
 * ZephCore - Joystick UI Task
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Runs in the mesh event loop thread (main_companion.cpp).
 * Input events are queued from the Zephyr input subsystem callback.
 */

#include "joystick_ui_task.h"
#include "joystick_defs.h"
#include "joystick_ui_hooks.h"
#include <helpers/ui/ui_mesh_actions.h>
#include <helpers/ui/ui_task.h>
#include <helpers/AdvertDataHelpers.h>
#include <CompanionMesh.h>
#include <mesh/Utils.h>

#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/display.h>
#ifdef CONFIG_POWEROFF
#include <zephyr/sys/poweroff.h>
#endif
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>

#include <ZephyrSensorManager.h>
#include <adapters/board/ZephyrBoard.h>

#ifdef CONFIG_ZEPHCORE_UI_BUZZER
#include "buzzer.h"
#endif

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
#include <helpers/ui/doom_game.h>
#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(joystick_ui, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ===== Key event queue ===== */
char JoystickUITask::_key_buf[JOYSTICK_KEY_QUEUE_DEPTH * sizeof(char)];
struct k_msgq JoystickUITask::_key_queue;

void (*JoystickUITask::s_signal_fn)(void) = nullptr;
void (*JoystickUITask::s_schedule_render_fn)(uint32_t) = nullptr;

void JoystickUITask::setSignalFn(void (*fn)(void)) { s_signal_fn = fn; }
void JoystickUITask::setScheduleRenderFn(void (*fn)(uint32_t)) { s_schedule_render_fn = fn; }

void JoystickUITask::lockTimerCb(struct k_timer *t)
{
	/* ISR — atomic writes to mark locked; signal refresh so the loop wakes
	 * and the lock overlay renders. */
	auto *self = static_cast<JoystickUITask *>(k_timer_user_data_get(t));
	if (!self) return;
	self->_locked = true;
	self->_lock_step = 0;
	self->_next_refresh = 0;
	if (s_signal_fn) s_signal_fn();
}

void JoystickUITask::scheduleLockTimer()
{
	k_timer_stop(&_lock_timer);
	k_timer_start(&_lock_timer, K_MSEC(LOCK_AFTER_MS), K_NO_WAIT);
}

void JoystickUITask::onDisplayStateChanged()
{
	bool is_on = _display.isOn();
	if (is_on == _was_display_on) return;
	if (_curr) {
		if (is_on) _curr->onDisplayOn();
		else       _curr->onDisplayOff();
	}
	_was_display_on = is_on;
}

/* ===== Outgoing DM tracking ===== */

void JoystickUITask::pendingRetryTimerCb(struct k_timer *t)
{
	/* ISR — mark slot ready for retry, wake main loop. */
	PendingSend *slot = (PendingSend *)k_timer_user_data_get(t);
	if (!slot || !slot->task) return;
	slot->retry_due = true;
	if (s_signal_fn) s_signal_fn();
}

int JoystickUITask::allocPendingSendSlot()
{
	for (int i = 0; i < MAX_PENDING_SENDS; i++) {
		if (!_pending_sends[i].active) return i;
	}
	return -1;
}

void JoystickUITask::doPendingSend(int slot_idx)
{
	PendingSend &s = _pending_sends[slot_idx];
	ContactInfo *recipient = _mesh ? _mesh->lookupContactByPubKey(s.recipient_pubkey, PUB_KEY_SIZE) : nullptr;
	if (!recipient) {
		s.failed = true;
		completePendingSend(slot_idx);
		return;
	}

	/* Attempt 4 (5th, 0-indexed) — clear the saved path so this send forces
	 * flood. The clear persists so future DMs to this contact also re-discover. */
	if (s.attempt == MAX_SEND_ATTEMPTS - 1) {
		recipient->out_path_len = OUT_PATH_UNKNOWN;
		if (CompanionMesh *cm = static_cast<CompanionMesh *>(_mesh)) {
			cm->markContactsDirtyPublic();
		}
	}

	uint32_t expected_ack = 0, est_timeout = 0;
	int result = _mesh->sendMessage(*recipient, s.timestamp, s.attempt, s.text,
									expected_ack, est_timeout);
	if (result == 0) {
		/* sendMessage rejected (e.g. queue full) — back off and try again. */
		s.timeout_ms = 3000;
		schedulePendingRetry(slot_idx);
		return;
	}
	ui_signal_tx();
	s.expected_ack = expected_ack;
	/* Allow the ACK round-trip (2× est_timeout + small grace) before retrying. */
	s.timeout_ms = (est_timeout > 0) ? (est_timeout * 2 + 3000) : 8000;
	schedulePendingRetry(slot_idx);
}

void JoystickUITask::schedulePendingRetry(int slot_idx)
{
	PendingSend &s = _pending_sends[slot_idx];
	k_timer_stop(&s.retry_timer);
	k_timer_start(&s.retry_timer, K_MSEC(s.timeout_ms), K_NO_WAIT);
}

void JoystickUITask::completePendingSend(int slot_idx)
{
	PendingSend &s = _pending_sends[slot_idx];
	k_timer_stop(&s.retry_timer);
	bool delivered = s.delivered;

	/* Update the on-device history entry (UnreadScreen) to reflect outcome. */
	if (_unread) {
		_unread->markSentEntryStatus(s.timestamp, s.recipient_name, delivered);
	}

	/* Now (and only now) mirror to the BLE app with the outcome prefix. */
	ContactInfo *recipient = _mesh ? _mesh->lookupContactByPubKey(s.recipient_pubkey, PUB_KEY_SIZE) : nullptr;
	if (recipient) {
		if (CompanionMesh *cm = static_cast<CompanionMesh *>(_mesh)) {
			cm->queueLocalSentContactMessage(*recipient, s.timestamp, s.text, delivered);
		}
	}

	s.active = false;
	s.retry_due = false;
	_next_refresh = 0;
}

void JoystickUITask::processPendingRetries()
{
	for (int i = 0; i < MAX_PENDING_SENDS; i++) {
		PendingSend &s = _pending_sends[i];
		if (!s.active || !s.retry_due) continue;
		s.retry_due = false;
		if (s.delivered) {
			completePendingSend(i);
			continue;
		}
		if (s.attempt + 1 >= MAX_SEND_ATTEMPTS) {
			/* Out of retries. */
			s.failed = true;
			completePendingSend(i);
			continue;
		}
		s.attempt++;
		doPendingSend(i);
	}
}

bool JoystickUITask::startPendingDM(ContactInfo &recipient, uint32_t ts, const char *text)
{
	if (!text || !_mesh) return false;
	int slot_idx = allocPendingSendSlot();
	if (slot_idx < 0) {
		showAlert("Send queue full", 1500);
		return false;
	}

	PendingSend &s = _pending_sends[slot_idx];
	s.active = true;
	s.retry_due = false;
	s.delivered = false;
	s.failed = false;
	memcpy(s.recipient_pubkey, recipient.id.pub_key, PUB_KEY_SIZE);
	strncpy(s.recipient_name, recipient.name, sizeof(s.recipient_name) - 1);
	s.recipient_name[sizeof(s.recipient_name) - 1] = '\0';
	s.timestamp = ts;
	size_t tlen = strlen(text);
	if (tlen > MAX_TEXT_LEN) tlen = MAX_TEXT_LEN;
	memcpy(s.text, text, tlen);
	s.text[tlen] = '\0';
	s.attempt = 0;
	s.expected_ack = 0;
	s.timeout_ms = 0;

	/* Local history entry now (with pending marker — addPreview uses
	 * OUT_PATH_SENT which becomes "(>>) ContactName:"; markSentEntryStatus()
	 * rewrites the marker on outcome). */
	if (_unread) {
		_unread->addPreview(OUT_PATH_SENT, s.recipient_name, s.text, /*initially_read=*/true);
	}

	doPendingSend(slot_idx);
	return true;
}

ContactInfo *JoystickUITask::tryMatchPendingAck(uint32_t ack)
{
	if (ack == 0) return nullptr;
	for (int i = 0; i < MAX_PENDING_SENDS; i++) {
		PendingSend &s = _pending_sends[i];
		if (!s.active || s.expected_ack != ack) continue;
		s.delivered = true;
		ContactInfo *recipient = _mesh ? _mesh->lookupContactByPubKey(s.recipient_pubkey, PUB_KEY_SIZE) : nullptr;
		completePendingSend(i);
		return recipient;
	}
	return nullptr;
}

/* ===== Channel-send feedback ===== */

void JoystickUITask::pendingChannelFeedbackCb(struct k_timer *t)
{
	PendingChannelSend *slot = (PendingChannelSend *)k_timer_user_data_get(t);
	if (!slot || !slot->task) return;
	slot->feedback_due = true;
	if (s_signal_fn) s_signal_fn();
}

int JoystickUITask::allocPendingChannelSlot()
{
	for (int i = 0; i < MAX_PENDING_CHANNEL_SENDS; i++) {
		if (!_pending_channel_sends[i].active) return i;
	}
	return -1;
}

void JoystickUITask::completePendingChannelSend(int slot_idx, bool heard)
{
	PendingChannelSend &s = _pending_channel_sends[slot_idx];

	/* Update the local _ch_previews entry's path_len so the UI shows the
	 * outcome ("sent+" / "sent?"). Match by stored ring index + timestamp
	 * to defend against ring wrap-around. */
	if (s.preview_index >= 0 && s.preview_index < JOYSTICK_OFFLINE_QUEUE_SIZE &&
		_ch_previews[s.preview_index].timestamp == s.preview_timestamp) {
		_ch_previews[s.preview_index].path_len = heard ? OUT_PATH_SENT_HEARD : OUT_PATH_SENT_UNHEARD;
	}

	/* Mirror to BLE app now (deferred until outcome is known) with the
	 * heard/unheard prefix. */
	if (CompanionMesh *cm = static_cast<CompanionMesh *>(_mesh)) {
		cm->queueLocalSentChannelMessage(s.channel_idx, s.timestamp, s.text, heard);
	}

	s.active = false;
	s.feedback_due = false;
	_next_refresh = 0;
}

void JoystickUITask::processPendingChannelFeedback()
{
	for (int i = 0; i < MAX_PENDING_CHANNEL_SENDS; i++) {
		PendingChannelSend &s = _pending_channel_sends[i];
		if (!s.active || !s.feedback_due) continue;
		s.feedback_due = false;
		int dupe_count = _mesh ? _mesh->getContentionTracker().extractDupeCount(s.hash) : -1;
		bool heard = (dupe_count > 0);
		completePendingChannelSend(i, heard);
	}
}

bool JoystickUITask::startPendingChannel(uint8_t channel_idx, ChannelDetails &ch,
		uint32_t ts, const char *text)
{
	if (!text || !_mesh) return false;
	int slot_idx = allocPendingChannelSlot();
	if (slot_idx < 0) {
		showAlert("Channel queue full", 1500);
		return false;
	}

	uint32_t pkt_hash = 0;
	int text_len = (int)strlen(text);
	bool ok = _mesh->sendGroupMessage(ts, ch.channel,
			_prefs ? _prefs->node_name : "", text, text_len, &pkt_hash);
	if (!ok) return false;
	ui_signal_tx();

	/* Local preview now (with "sent" path_len; we'll rewrite to
	 * SENT_HEARD/SENT_UNHEARD on feedback). */
	_ch_preview_head = (_ch_preview_head + 1) % JOYSTICK_OFFLINE_QUEUE_SIZE;
	ChannelMsgPreview &p = _ch_previews[_ch_preview_head];
	strncpy(p.channel, ch.name, sizeof(p.channel) - 1);
	p.channel[sizeof(p.channel) - 1] = '\0';
	strncpy(p.text, text, sizeof(p.text) - 1);
	p.text[sizeof(p.text) - 1] = '\0';
	p.timestamp = ts;
	p.path_len = OUT_PATH_SENT;
	if (_ch_preview_count < JOYSTICK_OFFLINE_QUEUE_SIZE) _ch_preview_count++;

	PendingChannelSend &s = _pending_channel_sends[slot_idx];
	s.active = true;
	s.feedback_due = false;
	s.channel_idx = channel_idx;
	s.preview_index = _ch_preview_head;
	s.preview_timestamp = ts;
	s.timestamp = ts;
	s.hash = pkt_hash;
	size_t tl = (size_t)text_len;
	if (tl > MAX_TEXT_LEN) tl = MAX_TEXT_LEN;
	memcpy(s.text, text, tl);
	s.text[tl] = '\0';

	k_timer_stop(&s.feedback_timer);
	k_timer_start(&s.feedback_timer, K_MSEC(CHANNEL_FEEDBACK_WINDOW_MS), K_NO_WAIT);
	return true;
}

static bool joystick_queue_initialized;

#define ENTER_LONG_PRESS_MS  500

static uint32_t s_enter_press_ms;
static uint32_t s_up_press_ms;
static uint32_t s_down_press_ms;

static void joystick_ui_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	/* When Doom is running, forward all events to it.
	 * BACK/ESC/KEY_1 release also enqueues KEY_CANCEL so DoomScreen can stop it. */
	if (doom_game_is_running()) {
		doom_game_input(evt->code, evt->value);
		if (!evt->value &&
			(evt->code == INPUT_KEY_BACK || evt->code == INPUT_KEY_ESC ||
			 evt->code == INPUT_KEY_1)) {
			if (joystick_queue_initialized) {
				JoystickUITask::enqueueKey(KEY_CANCEL);
			}
		}
		return;
	}
#endif

	/* Long press fires on RELEASE so hold duration is known */
	if (evt->code == INPUT_KEY_ENTER) {
		if (evt->value) {
			s_enter_press_ms = k_uptime_get_32();
		} else {
			uint32_t held = k_uptime_get_32() - s_enter_press_ms;
			char key = (held >= ENTER_LONG_PRESS_MS) ? KEY_ENTER_LONG : KEY_ENTER;
			if (joystick_queue_initialized) {
				JoystickUITask::enqueueKey(key);
			}
		}
		return;
	}
	if (evt->code == INPUT_KEY_UP) {
		if (evt->value) {
			s_up_press_ms = k_uptime_get_32();
		} else {
			uint32_t held = k_uptime_get_32() - s_up_press_ms;
			char key = (held >= ENTER_LONG_PRESS_MS) ? KEY_TO_TOP : KEY_UP;
			if (joystick_queue_initialized) {
				JoystickUITask::enqueueKey(key);
			}
		}
		return;
	}
	if (evt->code == INPUT_KEY_DOWN) {
		if (evt->value) {
			s_down_press_ms = k_uptime_get_32();
		} else {
			uint32_t held = k_uptime_get_32() - s_down_press_ms;
			char key = (held >= ENTER_LONG_PRESS_MS) ? KEY_TO_BOTTOM : KEY_DOWN;
			if (joystick_queue_initialized) {
				JoystickUITask::enqueueKey(key);
			}
		}
		return;
	}

	/* All other keys: fire on press, ignore release */
	if (!evt->value) return;

	char key = 0;
	switch (evt->code) {
	case INPUT_KEY_LEFT:    key = KEY_LEFT;         break;
	case INPUT_KEY_RIGHT:   key = KEY_RIGHT;        break;
	case INPUT_KEY_BACK:
	case INPUT_KEY_ESC:     key = KEY_CANCEL;       break;
	case INPUT_KEY_1:       key = KEY_CANCEL;       break;
	/* Multi tap outputs from input_multi_tap filter */
	case INPUT_KEY_D:       key = KEY_BUZZ_TOGGLE;  break;  /* 3 taps */
	case INPUT_KEY_C:       key = KEY_GPS_TOGGLE;   break;  /* 4 taps */
	case INPUT_KEY_E:       key = KEY_LED_TOGGLE;   break;  /* 5 taps */
	default: break;
	}

	if (key && joystick_queue_initialized) {
		JoystickUITask::enqueueKey(key);
	}
}

INPUT_CALLBACK_DEFINE(NULL, joystick_ui_input_cb, NULL);

void JoystickUITask::enqueueKey(char key)
{
	k_msgq_put(&_key_queue, &key, K_NO_WAIT);
	if (s_signal_fn) s_signal_fn();
}

/* ===== Constructor ===== */
JoystickUITask::JoystickUITask()
	: _mesh(nullptr), _rtc(nullptr), _prefs(nullptr),
	  _splash(nullptr), _home(nullptr), _contacts(nullptr), _unread(nullptr),
	  _advert(nullptr), _gps(nullptr), _system(nullptr), _system_time(nullptr),
	  _telemetry(nullptr), _tools(nullptr), _radio_stats(nullptr),
	  _repeaters(nullptr), _channels(nullptr), _t9_input(nullptr),
	  _rename_node(nullptr), _ble_code(nullptr), _stats(nullptr),
	  _stopwatch(nullptr), _countdown(nullptr), _snake(nullptr),
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	  _doom(nullptr),
#endif
	  _repeater_admin(nullptr), _curr(nullptr),
	  _next_refresh(0), _last_blt_ms(0), _pending_render(false), _render_immediate(false),
	  _screen_off_ms(AUTO_OFF_MILLIS), _was_display_on(false),
	  _cached_batt_mv(0), _battery_display_mode(0),
	  _brightness(100), _wake_on_msg(true), _ble_connected(false),
	  _ble_enabled(true), _noise_floor(-120),
	  _pkt_recv(0), _pkt_sent(0), _pkt_errors(0), _alert_expiry(0),
	  _locked(false), _lock_step(0),
	  _compose_is_contact(false), _compose_channel_idx(-1),
	  _ch_preview_count(0), _ch_preview_head(JOYSTICK_OFFLINE_QUEUE_SIZE - 1),
	  _initialized(false)
{
	memset(_alert, 0, sizeof(_alert));
	memset(_compose_channel_name, 0, sizeof(_compose_channel_name));
	memset(_compose_contact_name, 0, sizeof(_compose_contact_name));
	memset(_compose_contact_pubkey, 0, sizeof(_compose_contact_pubkey));
	memset(_ch_previews, 0, sizeof(_ch_previews));
	memset(_pending_ping_pubkey, 0, sizeof(_pending_ping_pubkey));
	_pending_ping_sent_ms = 0;
	memset(_discover_signals, 0, sizeof(_discover_signals));
}

/* ===== begin() ===== */
void JoystickUITask::begin(BaseChatMesh *mesh, mesh::ZephyrRTCClock *rtc, NodePrefs *prefs)
{
	_mesh = mesh;
	_rtc = rtc;
	_prefs = prefs;

	if (prefs && prefs->display_brightness >= 10) {
		_brightness = prefs->display_brightness;
	}
	if (prefs) {
		_ble_enabled = !prefs->ble_disabled;
		_wake_on_msg = prefs->wake_on_msg != 0;
		if (prefs->screen_off_secs > 0) {
			_screen_off_ms = (uint32_t)prefs->screen_off_secs * 1000;
			mc_display_set_auto_off_ms(_screen_off_ms);
		}
	}
	applyBrightness();

	/* Lock timer: fires once when LOCK_AFTER_MS passes without activity. */
	k_timer_init(&_lock_timer, lockTimerCb, NULL);
	k_timer_user_data_set(&_lock_timer, this);
	scheduleLockTimer();

	/* Pending-DM retry timers — one per slot, one-shot, callback signals
	 * refresh and sets the slot's retry_due flag for the main loop. */
	for (int i = 0; i < MAX_PENDING_SENDS; i++) {
		_pending_sends[i].task = this;
		_pending_sends[i].active = false;
		_pending_sends[i].retry_due = false;
		k_timer_init(&_pending_sends[i].retry_timer, pendingRetryTimerCb, NULL);
		k_timer_user_data_set(&_pending_sends[i].retry_timer, &_pending_sends[i]);
	}

	/* Channel-send feedback timers. */
	for (int i = 0; i < MAX_PENDING_CHANNEL_SENDS; i++) {
		_pending_channel_sends[i].task = this;
		_pending_channel_sends[i].active = false;
		_pending_channel_sends[i].feedback_due = false;
		k_timer_init(&_pending_channel_sends[i].feedback_timer, pendingChannelFeedbackCb, NULL);
		k_timer_user_data_set(&_pending_channel_sends[i].feedback_timer, &_pending_channel_sends[i]);
	}

	/* Init key queue */
	k_msgq_init(&_key_queue, _key_buf, sizeof(char), JOYSTICK_KEY_QUEUE_DEPTH);
	joystick_queue_initialized = true;

	/* Allocate screens */
	_splash = new SplashScreen(this);
	_home = new HomeScreen(this, rtc);
	_contacts = new ContactsScreen(this, rtc);
	_unread = new UnreadScreen(this, rtc);
	_advert = new AdvertScreen(this, rtc);
	_gps = new GPSSettingsScreen(this, rtc);
	_system = new SystemScreen(this, rtc);
	_system_time = new SystemTimeScreen(this, rtc);
	_telemetry = new TelemetryScreen(this, rtc);
	_tools = new ToolsScreen(this, rtc);
	_radio_stats = new RadioStatsScreen(this);
	_repeaters = new RepeatersScreen(this, rtc);
	_channels = new ChannelsScreen(this, rtc);
	_t9_input = new T9InputScreen(this);
	_rename_node = new RenameNodeScreen(this);
	_ble_code = new BLECodeScreen(this);
	_stats = new StatsScreen(this);
	_stopwatch = new StopwatchScreen(this);
	_countdown = new CountdownScreen(this);
	_snake = new SnakeScreen(this);
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	_doom = new DoomScreen(this);
#endif
	_repeater_admin = new RepeaterAdminScreen(this, rtc);

	setCurrScreen(_splash);
	_initialized = true;
	if (s_schedule_render_fn) s_schedule_render_fn(50);

	LOG_INF("joystick UI initialized");
}

/* ===== setCurrScreen / navigation ===== */
void JoystickUITask::setCurrScreen(UIScreen *s)
{
	if (_curr == s) return;  /* no-op for re-entry */
	if (_curr) _curr->onExit();
	_curr = s;
	if (_curr) _curr->onEnter();
	_next_refresh = 0;
}

void JoystickUITask::gotoHomeScreen()         { setCurrScreen(_home); }
void JoystickUITask::gotoContactsScreen()     { setCurrScreen(_contacts); }
void JoystickUITask::gotoAdvertScreen()       { setCurrScreen(_advert); }
void JoystickUITask::gotoGPSScreen()          { setCurrScreen(_gps); }
void JoystickUITask::gotoSystemScreen()       { setCurrScreen(_system); }
void JoystickUITask::gotoSystemTimeScreen()   { setCurrScreen(_system_time); }
void JoystickUITask::gotoTelemetryScreen()    { setCurrScreen(_telemetry); }
void JoystickUITask::gotoToolsScreen()        { setCurrScreen(_tools); }
void JoystickUITask::gotoRadioStatsScreen()   { setCurrScreen(_radio_stats); }
void JoystickUITask::gotoRepeatersScreen()    { setCurrScreen(_repeaters); }
void JoystickUITask::gotoChannelsScreen()     { setCurrScreen(_channels); }
void JoystickUITask::gotoT9InputScreen()      { setCurrScreen(_t9_input); }

void JoystickUITask::gotoT9InputScreenWithPrefix(const char *prefix)
{
	if (!_t9_input) return;
	auto *t9 = static_cast<T9InputScreen *>(_t9_input);
	t9->clearInput();
	t9->setInitialInput(prefix);
	setCurrScreen(_t9_input);
}

void JoystickUITask::gotoRenameNodeScreen()   { setCurrScreen(_rename_node); }
void JoystickUITask::gotoBLECodeScreen()      { setCurrScreen(_ble_code); }
void JoystickUITask::gotoStatsScreen()        { setCurrScreen(_stats); }
void JoystickUITask::gotoStopwatchScreen()    { setCurrScreen(_stopwatch); }
void JoystickUITask::gotoCountdownScreen()    { setCurrScreen(_countdown); }
void JoystickUITask::gotoSnakeScreen()        { setCurrScreen(_snake); }
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
void JoystickUITask::gotoDoomScreen()         { setCurrScreen(_doom); }
#endif

void JoystickUITask::gotoUnreadScreen()
{
	if (!_unread) return;
	_unread->activatePreview(false, 0);
	setCurrScreen(_unread);
}

void JoystickUITask::gotoContactForAdvert(const uint8_t *prefix, int prefix_len)
{
	if (!_contacts) return;
	_contacts->openForPubKey(prefix, prefix_len);
	setCurrScreen(_contacts);
}

void JoystickUITask::gotoRepeaterAdminScreen(const uint8_t *pub_key, const char *name,
		bool from_contacts)
{
	if (!_repeater_admin) return;
	_repeater_admin->openForContact(pub_key, name,
			from_contacts);
	setCurrScreen(_repeater_admin);
}

/* ===== Alert ===== */
void JoystickUITask::showAlert(const char *text, int duration_ms)
{
	strncpy(_alert, text, sizeof(_alert) - 1);
	_alert[sizeof(_alert) - 1] = '\0';
	_alert_expiry = k_uptime_get_32() + (uint32_t)duration_ms;
}

void JoystickUITask::renderAlertOverlay()
{
	int fw = _display.fontW();
	int fh = _display.fontH();
	int w = _display.width();
	int h = _display.height();
	int pad = 5;
	int box_h = fh + 8;
	int box_w = w - 2 * pad;
	int box_x = pad;
	int box_y = (h - box_h) / 2;

	/* Fill the area white so the invert below produces clean black background */
	mc_display_fill_rect(box_x, box_y, box_w, box_h);
	/* Invert the filled area → clean black background (erases page content) */
	mc_display_invert_rect(box_x, box_y, box_w, box_h);
	/* White border */
	_display.setColor(JoystickDisplay::WHITE);
	_display.drawRect(box_x, box_y, box_w, box_h);
	/* White text centered inside */
	int text_len = (int)strlen(_alert);
	int text_w = text_len * fw;
	int text_x = box_x + (box_w - text_w) / 2;
	if (text_x < box_x + 2) text_x = box_x + 2;
	mc_display_text(text_x, box_y + (box_h - fh) / 2, _alert, false);
}

void JoystickUITask::handleLockInput(char key, uint32_t now)
{
	/* Unlock sequence: LEFT, OK, RIGHT — dedicated GPIO keys delivered
	 * instantly. The back button is unusable here: its taps pass through
	 * the longpress + multi-tap devicetree filters, so a single tap is
	 * deferred 400 ms (tap-count window), a >=1 s hold becomes KEY_POWER,
	 * and a quick re-tap becomes KEY_B — which made the old Back-OK-Back
	 * sequence timing-sensitive. A wrong key restarts the sequence, except
	 * LEFT, which always (re)arms step 1. */
	uint8_t restart = (key == KEY_LEFT) ? 1 : 0;

	switch (_lock_step) {
	case 0:
		_lock_step = restart;
		break;
	case 1:
		if (key == KEY_ENTER || key == KEY_ENTER_LONG) _lock_step = 2;
		else _lock_step = restart;
		break;
	case 2:
		if (key == KEY_RIGHT) {
			_locked = false;
			_lock_step = 0;
			scheduleLockTimer();
		} else {
			_lock_step = restart;
		}
		break;
	default:
		_lock_step = 0;
		break;
	}
	_next_refresh = 0;
}

void JoystickUITask::renderLockOverlay()
{
	int fw = _display.fontW();
	int fh = _display.fontH();
	int w = _display.width();
	int h = _display.height();

	/* Full black background */
	mc_display_fill_rect(0, 0, w, h);
	mc_display_invert_rect(0, 0, w, h);
	_display.setColor(JoystickDisplay::WHITE);
	_display.drawRect(0, 0, w, h);

	/* Title */
	const char *title = "LOCKED";
	int title_w = (int)strlen(title) * fw;
	mc_display_text((w - title_w) / 2, 6, title, false);
	mc_display_hline(2, 6 + fh + 2, w - 4);

	/* Battery + unread (cached values; refreshed on screen wake via the
	 * normal render-path ui_refresh_battery() gate, so locked-and-asleep
	 * doesn't burn power). */
	char batt_buf[16];
	if (_cached_batt_mv > 0) {
		int pct = ((int)_cached_batt_mv - kBattMinMv) * 100 / (kBattMaxMv - kBattMinMv);
		if (pct < 0) pct = 0;
		if (pct > 100) pct = 100;
		snprintf(batt_buf, sizeof(batt_buf), "Batt: %d%%", pct);
	} else {
		snprintf(batt_buf, sizeof(batt_buf), "Batt: --");
	}
	int batt_w = (int)strlen(batt_buf) * fw;
	mc_display_text((w - batt_w) / 2, 6 + fh + 6, batt_buf, false);

	char unread_buf[16];
	int unread = (_unread) ? _unread->getUnreadCount() : 0;
	snprintf(unread_buf, sizeof(unread_buf), "Unread: %d", unread);
	int unread_w = (int)strlen(unread_buf) * fw;
	mc_display_text((w - unread_w) / 2, 6 + fh + 6 + fh + 2, unread_buf, false);

	/* Three step unlock sequence */
	const char *toks[3] = { "Left", "OK", "Right" };
	const int gap = 6;
	int total_w = 0;
	for (int i = 0; i < 3; i++) total_w += (int)strlen(toks[i]) * fw;
	total_w += gap * 2;
	int seq_y = h - fh - 8;
	int x = (w - total_w) / 2;

	for (int i = 0; i < 3; i++) {
		int tw = (int)strlen(toks[i]) * fw;
		bool active = (_lock_step == (uint8_t)i);
		if (active) {
			mc_display_fill_rect(x - 1, seq_y - 1, tw + 2, fh + 2);
			mc_display_invert_rect(x - 1, seq_y - 1, tw + 2, fh + 2);
			mc_display_text(x, seq_y, toks[i], true);
		} else {
			mc_display_text(x, seq_y, toks[i], false);
		}
		x += tw + gap;
	}
}

/* ===== loop() ===== */
void JoystickUITask::loop()
{
	if (!_initialized) return;

	uint32_t now = k_uptime_get_32();

	/* Auto-off is owned by display.c (k_work_delayable in display.c rescheduled
	 * via mc_display_reset_auto_off() — fires mc_display_off() when due).
	 * Auto-lock is owned by _lock_timer (fires lockTimerCb in ISR which sets
	 * _locked = true and signals refresh). Both are fully event-driven; no
	 * deadline checks needed here. */

	/* Catch display state transitions (mainly display.c's auto-off, which
	 * happens behind our back) so the current screen can pause/resume any
	 * periodic timers it owns. */
	onDisplayStateChanged();

	/* Pending-DM retry timers may have fired in ISR. */
	processPendingRetries();
	processPendingChannelFeedback();

	/* Drain all pending keys — apply navigation in one pass, render once. */
	char keys[JOYSTICK_KEY_QUEUE_DEPTH];
	int nkeys = 0;

	while (nkeys < JOYSTICK_KEY_QUEUE_DEPTH &&
	       k_msgq_get(&_key_queue, &keys[nkeys], K_NO_WAIT) == 0) {
		nkeys++;
	}

	for (int i = 0; i < nkeys; i++) {
		char key = keys[i];

		if (!_display.isOn()) {
			/* Wake display; discard this press (same as single-key path). */
			ui_invalidate_battery_cache();
			_display.turnOn();
			onDisplayStateChanged();
			if (!_locked) {
				scheduleLockTimer();
			}
			_render_immediate = true;
			_pending_render = true;
			continue;
		}

		mc_display_reset_auto_off();

		if (_locked) {
			handleLockInput(key, now);
			_render_immediate = true;
			_pending_render = true;
			continue;
		}

		scheduleLockTimer();
		bool consumed = false;
		if (key == KEY_BUZZ_TOGGLE) {
			toggleBuzzer();
			consumed = true;
		} else if (key == KEY_GPS_TOGGLE) {
			toggleGPS();
			consumed = true;
		} else if (key == KEY_LED_TOGGLE) {
			toggleLeds();
			consumed = true;
		}
		if (!consumed && _curr) {
			_curr->handleInput(key);
		}
		_pending_render = true;
	}

	if (!_display.isOn() || !_curr) {
		return;
	}

	const bool due_timer = (now >= _next_refresh);
	if (!_pending_render && !due_timer) {
		return;
	}

	/* Throttle input-driven blits; timer/notify wakes (due_timer) stay immediate. */
	if (_pending_render && !due_timer && !_render_immediate &&
	    (now - _last_blt_ms < JOYSTICK_RENDER_MIN_MS)) {
		uint32_t wait = JOYSTICK_RENDER_MIN_MS - (now - _last_blt_ms);
		if (s_schedule_render_fn) {
			s_schedule_render_fn(wait);
		}
		_next_refresh = now + wait;
		return;
	}

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
	if (doom_game_is_running()) {
		_next_refresh = now + 500;
		if (s_schedule_render_fn) s_schedule_render_fn(500);
		return;
	}
#endif
	ui_refresh_battery();
	_display.startFrame();
	int delay_ms;
	if (_locked) {
		renderLockOverlay();
		delay_ms = 200;
	} else {
		delay_ms = _curr->render(_display);
		if (now < _alert_expiry) {
			renderAlertOverlay();
		}
		if (delay_ms <= 0) delay_ms = 100;
	}
	_next_refresh = now + (uint32_t)delay_ms;
	if (s_schedule_render_fn) s_schedule_render_fn((uint32_t)delay_ms);
	_display.endFrame();
	_last_blt_ms = now;
	_pending_render = false;
	_render_immediate = false;
}

/* ===== Buzzer / GPS helpers ===== */
bool JoystickUITask::isBuzzerQuiet() const
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	return buzzer_is_quiet();
#else
	return true;
#endif
}

void JoystickUITask::toggleBuzzer()
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	bool was_quiet = buzzer_is_quiet();
	if (was_quiet) {
		buzzer_set_quiet(false);
		buzzer_play("bon:d=16,o=7,b=200:c,p,c,p,c,p,p,8e");
	} else {
		buzzer_play("bof:d=16,o=7,b=200:c,p,c,p,c,p,p,8g5");
		buzzer_set_quiet_deferred(true);
	}
	mesh_set_buzzer_quiet(!was_quiet);
#endif
}

bool JoystickUITask::isGPSAvailable() const
{
	return gps_is_available();
}

bool JoystickUITask::getGPSState() const
{
	return gps_is_enabled();
}

void JoystickUITask::toggleGPS()
{
	if (!gps_is_available()) return;
	mesh_gps_set_enabled(!gps_is_enabled());
}

void JoystickUITask::playCountdownAlarm()
{
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	bool was_quiet = buzzer_is_quiet();
	buzzer_set_quiet(false);
	buzzer_play("alarm:d=8,o=6,b=180:c,p,c,p,c,p,c");
	if (was_quiet) {
		buzzer_set_quiet_deferred(true);
	}
#endif
}

/* ===== BLE serial toggle ===== */
void JoystickUITask::enableSerial()  { _ble_enabled = true;  mesh_ble_set_enabled(true); }
void JoystickUITask::disableSerial() { _ble_enabled = false; mesh_ble_set_enabled(false); }

/* ===== Offgrid / LEDs / DFU ===== */
void JoystickUITask::setOffgridMode(bool enable)
{
	if (_prefs) _prefs->client_repeat = enable ? 1 : 0;
	mesh_set_offgrid_mode(enable);
}

void JoystickUITask::toggleLeds()
{
	bool new_disabled = !isLedsDisabled();
	if (_prefs) _prefs->leds_disabled = new_disabled ? 1 : 0;
	ui_set_heartbeat_led(!new_disabled);
	mesh_set_leds_disabled(new_disabled);
}

void JoystickUITask::rebootToDFU()
{
	LOG_INF("joystick UI: entering BLE DFU bootloader");
	/* Blank the OLED before the reset. Without this the panel retains its
	 * last frame (the menu) across the reboot — so if the bootloader's
	 * BLE-OTA path stalls, it looks like the app froze on the menu rather
	 * than a clean handoff. Mirrors the button UI's action_enter_dfu, which
	 * does mc_display_off() before mesh_reboot_to_ota_dfu(). */
	_display.turnOff();
	mesh_reboot_to_ota_dfu();  /* GPREGRET 0xA8 + reset; never returns */
}

/* ===== Display brightness ===== */
void JoystickUITask::applyBrightness()
{
	const struct device *dev = mc_display_get_device();
	if (dev) {
		uint8_t level = (uint8_t)(((uint32_t)_brightness * 255U) / 100U);
		/* SH1106/SSD1306 OLEDs expose luminance via the contrast register.
		 * display_set_brightness() is not implemented by those drivers,
		 * display_set_contrast() maps directly to the 0x81 contrast command. */
		if (display_set_contrast(dev, level) != 0) {
			display_set_brightness(dev, level);
		}
	}
}

void JoystickUITask::adjustBrightness(int delta)
{
	int v = (int)_brightness + delta;
	if (v < 10) v = 10;
	if (v > 100) v = 100;
	_brightness = (uint8_t)v;
	applyBrightness();
	mesh_save_brightness(_brightness);
}

void JoystickUITask::adjustScreenOff(int32_t delta_ms)
{
	int32_t v = (int32_t)_screen_off_ms + delta_ms;
	if (v < 5000)  v = 5000;
	if (v > 300000) v = 300000;
	_screen_off_ms = (uint32_t)v;
	mc_display_set_auto_off_ms(_screen_off_ms);
	mc_display_reset_auto_off();
	mesh_save_screen_off_secs((uint16_t)(_screen_off_ms / 1000));
}

void JoystickUITask::cyclePathHashMode()
{
	if (!_prefs) return;
	uint8_t next = (uint8_t)((_prefs->path_hash_mode + 1) % 3);  /* 0→1→2→0 */
	_prefs->path_hash_mode = next;
	mesh_save_path_hash_mode(next);
}

void JoystickUITask::toggleWakeOnMsg()
{
	_wake_on_msg = !_wake_on_msg;
	mesh_set_wake_on_msg(_wake_on_msg);
}

/* ===== Notifications from mesh ===== */
void JoystickUITask::newMsg(uint8_t path_len, const char *from_name, const char *text, int msgcount)
{
	(void)msgcount;  /* tracked by CompanionMesh */
	if (_unread) {
		/* When BLE phone is connected it pulls offline queue and marks read;
		 * keep the message in our local history but don't count as unread. */
		_unread->addPreview(path_len, from_name, text,
				/*initially_read=*/_ble_connected);
	}
	/* Wake display if wake on msg is enabled and no BLE peer is connected.
	 * mc_display_on() already reschedules display.c's auto-off timer. */
	if (_wake_on_msg && !_ble_connected && !_display.isOn()) {
		_display.turnOn();
	}
	_next_refresh = 0;
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	if (!_ble_connected) {
		buzzer_play("MsgRcv3:d=4,o=6,b=200:32e,32g,32b,16c7");
	}
#endif
}

void JoystickUITask::newChannelMsg(const char *channel_name, const char *text,
		uint32_t ts, uint8_t path_len)
{
	/* Store in channel preview ring */
	_ch_preview_head = (_ch_preview_head + 1) % JOYSTICK_OFFLINE_QUEUE_SIZE;
	ChannelMsgPreview &p = _ch_previews[_ch_preview_head];
	strncpy(p.channel, channel_name, sizeof(p.channel) - 1);
	p.channel[sizeof(p.channel) - 1] = '\0';
	strncpy(p.text, text, sizeof(p.text) - 1);
	p.text[sizeof(p.text) - 1] = '\0';
	p.timestamp = ts;
	p.path_len = path_len;
	if (_ch_preview_count < JOYSTICK_OFFLINE_QUEUE_SIZE) {
		_ch_preview_count++;
	}
	if (_unread) {
		char ch_from[48];
		snprintf(ch_from, sizeof(ch_from), "#%s", channel_name ? channel_name : "?");
		/* BLE-connected → phone reads via offline queue, don't count as unread. */
		_unread->addPreview(path_len, ch_from, text,
				/*initially_read=*/_ble_connected);
	}
	if (_wake_on_msg && !_ble_connected && !_display.isOn()) {
		_display.turnOn();  /* display.c reschedules auto-off internally */
	}
	_next_refresh = 0;
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	if (!_ble_connected) {
		buzzer_play("kerplop:d=16,o=6,b=120:32g#,32c#");
	}
#endif
}

void JoystickUITask::notify()
{
	_next_refresh = 0;
	if (s_signal_fn) s_signal_fn();
}

/* ===== Message accessors ===== */
int JoystickUITask::getUnreadCount()
{
	if (!_unread) return 0;
	return _unread->getUnreadCount();
}

int JoystickUITask::getStoredMsgCount() const
{
	if (!_unread) return 0;
	return static_cast<const UnreadScreen *>(_unread)->getStoredMsgCount();
}

int JoystickUITask::getContactMsgCount(const char *contact_name) const
{
	if (!_unread) return 0;
	return static_cast<const UnreadScreen *>(_unread)->getContactMsgCount(contact_name);
}

bool JoystickUITask::getContactMsgAt(const char *contact_name, int idx,
		const char *&out_msg, uint32_t &out_ts,
		uint8_t *out_path) const
{
	if (!_unread) return false;
	return static_cast<const UnreadScreen *>(_unread)->getContactMsgAt(
		contact_name, idx, out_msg, out_ts, out_path);
}

int JoystickUITask::getChannelPreviewCountFor(const char *channel) const
{
	if (!channel || !channel[0]) return 0;
	int count = 0;
	for (int i = 0; i < _ch_preview_count; i++) {
		int pos = (_ch_preview_head - i + JOYSTICK_OFFLINE_QUEUE_SIZE) % JOYSTICK_OFFLINE_QUEUE_SIZE;
		if (strcmp(_ch_previews[pos].channel, channel) == 0) count++;
	}
	return count;
}

bool JoystickUITask::getChannelPreviewFor(const char *channel, int idx,
		const char *&text, uint32_t &ts,
		uint8_t *path_len) const
{
	if (!channel) return false;
	int match = 0;
	for (int i = 0; i < _ch_preview_count; i++) {
		int pos = (_ch_preview_head - i + JOYSTICK_OFFLINE_QUEUE_SIZE) % JOYSTICK_OFFLINE_QUEUE_SIZE;
		if (strcmp(_ch_previews[pos].channel, channel) != 0) continue;
		if (match == idx) {
			text = _ch_previews[pos].text;
			ts = _ch_previews[pos].timestamp;
			if (path_len) *path_len = _ch_previews[pos].path_len;
			return true;
		}
		match++;
	}
	return false;
}

/* ===== Compose helpers ===== */
void JoystickUITask::setComposeChannel(int idx, const char *name)
{
	_compose_is_contact = false;
	_compose_channel_idx = idx;
	if (name) {
		strncpy(_compose_channel_name, name, sizeof(_compose_channel_name) - 1);
		_compose_channel_name[sizeof(_compose_channel_name) - 1] = '\0';
	} else {
		_compose_channel_name[0] = '\0';
	}
}

void JoystickUITask::setComposeContact(const ContactInfo &contact)
{
	_compose_is_contact = true;
	_compose_channel_idx = -1;
	_compose_channel_name[0] = '\0';
	strncpy(_compose_contact_name, contact.name, sizeof(_compose_contact_name) - 1);
	_compose_contact_name[sizeof(_compose_contact_name) - 1] = '\0';
	memcpy(_compose_contact_pubkey, contact.id.pub_key, PUB_KEY_SIZE);
}

bool JoystickUITask::sendComposedMessage(const char *text)
{
	if (!text || !text[0] || !_mesh) return false;

	if (_compose_is_contact) {
		ContactInfo *recipient = _mesh->lookupContactByPubKey(_compose_contact_pubkey, PUB_KEY_SIZE);
		if (!recipient) return false;
		uint32_t ts = _rtc ? _rtc->getCurrentTimeUnique() : k_uptime_get_32();
		/* Goes through the pending-DM machinery: tracks expected_ack, retries
		 * on no-ACK (up to MAX_SEND_ATTEMPTS, last attempt forces flood),
		 * updates UnreadScreen + queues BLE-app mirror with outcome prefix. */
		return startPendingDM(*recipient, ts, text);
	}

	if (_compose_channel_idx == -2) {
		/* Join public #channel: PSK = SHA256(name)[0..15] */
		uint8_t psk[16];
		mesh::Utils::sha256(psk, sizeof(psk), (const uint8_t *)text, (int)strlen(text));
		if (_mesh->addChannel(text, psk, sizeof(psk))) {
			return true;
		}
		return false;
	}

	if (_compose_channel_idx >= 0) {
		ChannelDetails ch;
		if (!_mesh->getChannel(_compose_channel_idx, ch)) return false;
		uint32_t ts = _rtc ? _rtc->getCurrentTimeUnique() : k_uptime_get_32();
		/* Pending-channel machinery handles broadcast, local preview, and
		 * the deferred BLE-app mirror (queued only after the feedback
		 * window decides heard / not-heard). */
		return startPendingChannel((uint8_t)_compose_channel_idx, ch, ts, text);
	}

	return false;
}

bool JoystickUITask::sendChannelMessage(const char *text)
{
	if (!text || !text[0] || _compose_channel_idx < 0 || !_mesh) return false;
	ChannelDetails ch;
	if (!_mesh->getChannel(_compose_channel_idx, ch)) return false;
	uint32_t ts = _rtc ? _rtc->getCurrentTimeUnique() : k_uptime_get_32();
	return startPendingChannel((uint8_t)_compose_channel_idx, ch, ts, text);
}

bool JoystickUITask::findContactByName(const char *name, ContactInfo &contact)
{
	if (!name || !_mesh) return false;
	int n = _mesh->getNumContacts();
	for (int i = 0; i < n; i++) {
		ContactInfo c;
		if (_mesh->getContactByIdx(i, c) && strcmp(c.name, name) == 0) {
			contact = c;
			return true;
		}
	}
	return false;
}

/* ===== RepeaterAdmin callbacks ===== */
void JoystickUITask::onRepeaterAdminLoginResult(const uint8_t *pub_key_prefix,
		bool success, uint8_t permissions, uint32_t server_time)
{
	if (!_repeater_admin || _curr != _repeater_admin) return;
	(void)pub_key_prefix;
	_repeater_admin->onLoginResult(success, permissions, server_time);
	_next_refresh = 0;
	ui_signal_refresh();
}

void JoystickUITask::onRepeaterAdminCliResponse(const uint8_t *pub_key_prefix,
		const char *text)
{
	if (!_repeater_admin || _curr != _repeater_admin) return;
	(void)pub_key_prefix;
	_repeater_admin->onCliResponse(text);
	_next_refresh = 0;
	ui_signal_refresh();
}

/* ===== Repeater events ===== */

void JoystickUITask::onRepeaterReqResponse(const uint8_t *pub_key_prefix,
		int8_t snr_local, int8_t snr_remote,
		const uint8_t *data, uint8_t data_len)
{
	/* Route binary response to repeater admin screen if it's active and waiting */
	if (_repeater_admin && _curr == _repeater_admin && data && data_len > 0) {
		_repeater_admin->onReqResponse(pub_key_prefix,
				data, data_len);
		_next_refresh = 0;
		ui_signal_refresh();
	}

	if (memcmp(_pending_ping_pubkey, pub_key_prefix, 4) != 0) return;
	uint32_t rtt_ms = _pending_ping_sent_ms > 0
		? k_uptime_get_32() - _pending_ping_sent_ms : 0;
	memset(_pending_ping_pubkey, 0, sizeof(_pending_ping_pubkey));
	_pending_ping_sent_ms = 0;
	if (!_contacts || _curr != _contacts) return;
	_contacts->onPingResponse(snr_local, snr_remote, rtt_ms);
	_next_refresh = 0;
	ui_signal_refresh();
}

void JoystickUITask::onRepeaterDiscoverResp(const uint8_t *pub_key,
		int8_t snr, int8_t snr_remote, uint8_t path_len)
{
	DiscoverSignal *slot = nullptr;
	DiscoverSignal *empty_slot = nullptr;
	for (int i = 0; i < DISCOVER_SIGNAL_TABLE_SIZE; i++) {
		if (_discover_signals[i].valid) {
			if (memcmp(_discover_signals[i].pubkey, pub_key, PUB_KEY_SIZE) == 0) {
				slot = &_discover_signals[i];
				break;
			}
		} else if (!empty_slot) {
			empty_slot = &_discover_signals[i];
		}
	}
	if (!slot) slot = empty_slot;
	if (!slot) return;  // table full, drop
	memcpy(slot->pubkey, pub_key, PUB_KEY_SIZE);
	slot->snr = snr;
	slot->snr_remote = snr_remote;
	slot->path_len = path_len;
	slot->valid = true;
}

void JoystickUITask::notifyPacketSent()
{
	/* Stamp RTT start on first TX after a ping was queued */
	if (_pending_ping_sent_ms == 0 &&
		memcmp(_pending_ping_pubkey, "\0\0\0\0", 4) != 0) {
		_pending_ping_sent_ms = k_uptime_get_32();
	}
	if (_contacts && _curr == _contacts) {
		_contacts->onPacketSent();
	}
	if (_repeater_admin && _curr == _repeater_admin) {
		_repeater_admin->onPacketSent();
	}
}

int JoystickUITask::getRecentlyHeard(AdvertPath *dest, int max) const
{
	auto *cm = static_cast<CompanionMesh *>(_mesh);
	return cm ? cm->getRecentlyHeard(dest, max) : 0;
}

bool JoystickUITask::getDiscoverSignal(const uint8_t *pubkey, int8_t &snr_out,
		uint8_t *path_len_out) const
{
	for (int i = 0; i < DISCOVER_SIGNAL_TABLE_SIZE; i++) {
		if (_discover_signals[i].valid &&
			memcmp(_discover_signals[i].pubkey, pubkey, PUB_KEY_SIZE) == 0) {
			snr_out = _discover_signals[i].snr;
			if (path_len_out) *path_len_out = _discover_signals[i].path_len;
			return true;
		}
	}
	return false;
}

void JoystickUITask::clearDiscoverSignals()
{
	memset(_discover_signals, 0, sizeof(_discover_signals));
}

int JoystickUITask::getDiscoverCount() const
{
	int n = 0;
	for (int i = 0; i < DISCOVER_SIGNAL_TABLE_SIZE; i++) {
		if (_discover_signals[i].valid) n++;
	}
	return n;
}

bool JoystickUITask::getDiscoverByIdx(int idx, uint8_t *pubkey_out,
		int8_t &snr_out, int8_t &snr_remote_out,
		uint8_t *path_len_out) const
{
	int found = 0;
	for (int i = 0; i < DISCOVER_SIGNAL_TABLE_SIZE; i++) {
		if (!_discover_signals[i].valid) continue;
		if (found == idx) {
			if (pubkey_out) memcpy(pubkey_out, _discover_signals[i].pubkey, PUB_KEY_SIZE);
			snr_out = _discover_signals[i].snr;
			snr_remote_out = _discover_signals[i].snr_remote;
			if (path_len_out) *path_len_out = _discover_signals[i].path_len;
			return true;
		}
		found++;
	}
	return false;
}

bool JoystickUITask::addRepeaterContact(const uint8_t *pubkey)
{
	if (!_mesh) return false;
	if (_mesh->lookupContactByPubKey(pubkey, PUB_KEY_SIZE)) return false;  // already exists
	ContactInfo c = {};
	memcpy(c.id.pub_key, pubkey, PUB_KEY_SIZE);
	c.type = ADV_TYPE_REPEATER;
	c.out_path_len = OUT_PATH_UNKNOWN;
	c.lastmod = (uint32_t)_rtc->getCurrentTime();
	snprintf(c.name, sizeof(c.name), "!%02X%02X%02X%02X",
			 pubkey[0], pubkey[1], pubkey[2], pubkey[3]);
	return _mesh->addContact(c);
}

void JoystickUITask::registerAdminReqTag(uint32_t tag)
{
	auto *cm = static_cast<CompanionMesh *>(_mesh);
	if (cm) cm->setJoystickAdminTag(tag);
}

void JoystickUITask::clearAdminReqTag()
{
	auto *cm = static_cast<CompanionMesh *>(_mesh);
	if (cm) cm->clearJoystickAdminTag();
}

void JoystickUITask::cancelContactPing()
{
	auto *cm = static_cast<CompanionMesh *>(_mesh);
	if (cm) cm->clearJoystickPingTag();
	memset(_pending_ping_pubkey, 0, sizeof(_pending_ping_pubkey));
	_pending_ping_sent_ms = 0;
}

bool JoystickUITask::sendContactPingZeroHop(const ContactInfo &contact, uint32_t &est_timeout)
{
	auto *cm = static_cast<CompanionMesh *>(_mesh);
	if (!cm) return false;
	ContactInfo temp = contact;
	temp.out_path_len = 0;
	uint32_t tag = 0;
	est_timeout = 0;
	int res = cm->sendRequest(temp, REQ_TYPE_GET_STATUS, tag, est_timeout);
	if (res != MSG_SEND_FAILED) {
		memcpy(_pending_ping_pubkey, contact.id.pub_key, 4);
		_pending_ping_sent_ms = 0;
		cm->setJoystickPingTag(tag);
		ui_signal_tx();
	}
	return res != MSG_SEND_FAILED;
}

/* ===== Shutdown ===== */
void JoystickUITask::shutdown(bool restart)
{
	LOG_INF("joystick UI: shutdown (restart=%d)", restart);
#ifdef CONFIG_ZEPHCORE_UI_BUZZER
	buzzer_play("shutdown:d=16,o=5,b=120:c,p,a4,p,f4");
	/* Brief wait for melody — terminal path either way. */
	k_sleep(K_MSEC(400));
#endif

	if (restart) {
		_display.turnOff();
		sys_reboot(SYS_REBOOT_COLD);
	} else {
#ifdef CONFIG_POWEROFF
		/* Full peripheral teardown + SENSE config for sw0 wake.  Shared
		 * helper turns off display, GPS, regulators, holds LoRa in reset
		 * and arms the button SENSE so the user can actually wake the
		 * device. Same code path as the button UI's action_deep_sleep. */
		ui_prepare_for_system_off();
		sys_poweroff();
#else
		_display.turnOff();
		sys_reboot(SYS_REBOOT_COLD);
#endif
	}
}
