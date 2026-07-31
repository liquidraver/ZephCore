/*
 * ZephCore - Joystick UI Tools Screen
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "../joystick_screens.h"
#include "../joystick_ui_task.h"
#include "../joystick_ui_hooks.h"
#include "screen_helpers.h"
#include <helpers/AdvertDataHelpers.h>
#include <helpers/ui/ui_task.h>
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
#include <helpers/ui/doom_game.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <stdio.h>

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
enum ToolsItem { TOOLS_REPEATERS=0, TOOLS_STOPWATCH, TOOLS_COUNTDOWN, TOOLS_SNAKE, TOOLS_DOOM, TOOLS_COUNT };
static const char * const kToolsItems[TOOLS_COUNT] = {
	"Discover repeaters", "Stopwatch", "Countdown", "Snake", "Doom"
};
#else
enum ToolsItem { TOOLS_REPEATERS=0, TOOLS_STOPWATCH, TOOLS_COUNTDOWN, TOOLS_SNAKE, TOOLS_COUNT };
static const char * const kToolsItems[TOOLS_COUNT] = {
	"Discover repeaters", "Stopwatch", "Countdown", "Snake"
};
#endif

ToolsScreen::ToolsScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0)
{
}

int ToolsScreen::render(JoystickDisplay &display)
{
	renderScreenHeader(display, "Tools", _selected, TOOLS_COUNT);
	int start = computeListStart(_selected, TOOLS_COUNT);
	int y = kContentY + 2;
	for (int i = start; i < TOOLS_COUNT && i < start + UI_RECENT_LIST_SIZE; i++) {
		renderMenuListText(display, y, i == _selected, kToolsItems[i]);
		y += kMenuLineH;
	}
	return 500;
}

bool ToolsScreen::handleInput(char key)
{
	if (handleCommonListNavigation(key, _selected, TOOLS_COUNT)) return true;
	if (key == KEY_ENTER) {
		switch (_selected) {
		case TOOLS_REPEATERS: _task->gotoRepeatersScreen(); break;
		case TOOLS_STOPWATCH: _task->gotoStopwatchScreen(); break;
		case TOOLS_COUNTDOWN: _task->gotoCountdownScreen(); break;
		case TOOLS_SNAKE: _task->gotoSnakeScreen(); break;
#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
		case TOOLS_DOOM:       _task->gotoDoomScreen();        break;
#endif
		default: return false;
		}
		return true;
	}
	if (key == KEY_CANCEL || key == KEY_HOME) { _task->gotoHomeScreen(); return true; }
	return false;
}

#define REPEATER_SCAN_WINDOW_MS   30000   /* duration of "scanning..." indicator */
#define REPEATER_RESCAN_AFTER_MS  60000   /* auto re-scan on entry if last scan is older than this */

static const int RMODE_LIST = 0;
static const int RMODE_MODAL = 1;

/* Statically allocated scan state shared across the one screen instance */
static uint32_t s_scan_until;    /* k_uptime_get_32() when the "scanning..." window ends */
static uint32_t s_last_scan_ms;  /* k_uptime_get_32() of last scan (0 = never scanned) */

RepeatersScreen::RepeatersScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0),
	  _mode(RMODE_LIST), _last_sel(-1), _marquee_start_ms(0), _modal_idx(0)
{
	s_scan_until = 0; s_last_scan_ms = 0;
}

void RepeatersScreen::doScan()
{
	s_last_scan_ms = k_uptime_get_32();
	s_scan_until = s_last_scan_ms + REPEATER_SCAN_WINDOW_MS;
	_task->clearDiscoverSignals();

	uint8_t data[10];
	data[0] = CTL_TYPE_NODE_DISCOVER_REQ;
	data[1] = (1 << ADV_TYPE_REPEATER);
	_task->getMesh()->getRNG()->random(&data[2], 4);
	uint32_t since = 0;
	memcpy(&data[6], &since, 4);

	mesh::Packet *pkt = _task->getMesh()->createControlData(data, sizeof(data));
	if (pkt) {
		_task->getMesh()->sendZeroHop(pkt);
		ui_signal_tx();
	}
}

void RepeatersScreen::onEnter()
{
	/* Re-scan on entry if we've never scanned, or the last scan is stale. */
	uint32_t now = k_uptime_get_32();
	if (s_last_scan_ms == 0 || (now - s_last_scan_ms) >= REPEATER_RESCAN_AFTER_MS) {
		doScan();
	}
}

int RepeatersScreen::render(JoystickDisplay &display)
{
	/* Modal overlay */
	if (_mode == RMODE_MODAL) {
		int fw = display.fontW(), fh = display.fontH();
		int w = display.width(), h = display.height();

		uint8_t pubkey[PUB_KEY_SIZE];
		int8_t snr, snr_remote; uint8_t path_len;
		if (!_task->getDiscoverByIdx(_modal_idx, pubkey, snr, snr_remote, &path_len)) {
			_mode = RMODE_LIST;
			return 100;
		}
		auto *mesh = _task->getMesh();
		ContactInfo *ci = mesh->lookupContactByPubKey(pubkey, PUB_KEY_SIZE);

		char title[32];
		if (ci && ci->name[0]) {
			display.translateUTF8ToBlocks(title, ci->name, sizeof(title));
		} else {
			snprintf(title, sizeof(title), "!%02X%02X%02X%02X",
					 pubkey[0], pubkey[1], pubkey[2], pubkey[3]);
		}
		renderScrollingScreenHeader(display, title, 0, 0, _marquee_start_ms);

		int pad = 4;
		int box_h = fh + 8;
		int box_w = w - 2 * pad;
		int box_x = pad, box_y = (h - box_h) / 2;

		mc_display_fill_rect(box_x, box_y, box_w, box_h);
		mc_display_invert_rect(box_x, box_y, box_w, box_h);
		display.setColor(JoystickDisplay::WHITE);
		display.drawRect(box_x, box_y, box_w, box_h);

		int cx = box_x + box_w / 2;
		int y0 = box_y + (box_h - fh) / 2;

		char line1[24];
		snprintf(line1, sizeof(line1), "RX %+ddB TX %+ddB", (int)snr, (int)snr_remote);
		mc_display_text(cx - (int)strlen(line1) * fw / 2, y0, line1, false);

		(void)path_len;
		return 1000;
	}

	/* Repeater list */
	bool scanning = (k_uptime_get_32() < s_scan_until);
	int count = _task->getDiscoverCount();

	renderScreenHeader(display, "Repeaters", _selected, count);
	if (count == 0) {
		display.setTextSize(1);
		display.setColor(JoystickDisplay::GREEN);
		display.drawTextCentered(display.width() / 2, 24,
								 scanning ? "Listening..." : "None found");
		display.setColor(JoystickDisplay::LIGHT);
		display.drawTextCentered(display.width() / 2, display.height() - 10,
								 scanning ? "Scanning..." : "Long ENTER rescan CANCEL back");
		return 1000;
	}

	if (_selected >= count) _selected = count - 1;
	if (_selected < 0)     _selected = 0;

	if (_selected != _last_sel) {
		_marquee_start_ms = k_uptime_get_32();
		_last_sel = _selected;
	}

	int footer_y = display.height() - 12;
	int max_vis = (footer_y - kContentY) / kMenuLineH;
	if (max_vis < 1) max_vis = 1;

	int start = computeListStart(_selected, count, max_vis);
	auto *mesh = _task->getMesh();
	int y = kContentY;

	for (int i = start; i < count && i < start + max_vis; i++) {
		uint8_t pubkey[PUB_KEY_SIZE];
		int8_t snr, snr_remote; uint8_t path_len;
		if (!_task->getDiscoverByIdx(i, pubkey, snr, snr_remote, &path_len)) continue;

		ContactInfo *ci = mesh->lookupContactByPubKey(pubkey, PUB_KEY_SIZE);
		char label[52];
		char hop_label[12];
		formatHopCount(path_len, hop_label, sizeof(hop_label));
		if (ci && ci->name[0]) {
			char name[32];
			display.translateUTF8ToBlocks(name, ci->name, sizeof(name));
			snprintf(label, sizeof(label), "%s [%s]", name, hop_label);
		} else {
			snprintf(label, sizeof(label), "!%02X%02X%02X%02X [%s]",
					 pubkey[0], pubkey[1], pubkey[2], pubkey[3], hop_label);
		}
		renderMenuListText(display, y, i == _selected, label, true, 450, _marquee_start_ms);
		y += kMenuLineH;
	}

	display.setColor(JoystickDisplay::LIGHT);
	display.drawTextCentered(display.width() / 2, display.height() - 10,
							 scanning ? "Scanning..." : "");
	return scanning ? 400 : 1000;
}

bool RepeatersScreen::handleInput(char c)
{
	/* Modal input */
	if (_mode == RMODE_MODAL) {
		if (c == KEY_ENTER) {
			uint8_t pubkey[PUB_KEY_SIZE];
			int8_t snr_unused, snr_remote_unused;
			if (_task->getDiscoverByIdx(_modal_idx, pubkey, snr_unused, snr_remote_unused)) {
				ContactInfo *ci = _task->getMesh()->lookupContactByPubKey(pubkey, PUB_KEY_SIZE);
				if (ci) {
					_mode = RMODE_LIST;
					_task->gotoContactForAdvert(pubkey, 4);
				} else if (_task->addRepeaterContact(pubkey)) {
					_mode = RMODE_LIST;
					_task->showAlert("Added!", 1000);
				} else {
					_mode = RMODE_LIST;
					_task->showAlert("Failed to add", 1000);
				}
			} else {
				_mode = RMODE_LIST;
			}
			return true;
		}
		if (c == KEY_CANCEL || c == KEY_HOME) { _mode = RMODE_LIST; return true; }
		return true;
	}

	/* List input */
	int count = _task->getDiscoverCount();
	if (c == KEY_TO_TOP) { _selected = 0; return true; }
	if (c == KEY_TO_BOTTOM) { _selected = count > 0 ? count - 1 : 0; return true; }
	if (c == KEY_UP) { if (_selected > 0) _selected--; return true; }
	if (c == KEY_DOWN) { if (_selected < count - 1) _selected++; return true; }
	if (c == KEY_ENTER) {
		if (_selected < count) {
			_modal_idx = _selected;
			_marquee_start_ms = k_uptime_get_32();
			_mode = RMODE_MODAL;
		}
		return true;
	}
	if (c == KEY_ENTER_LONG) {
		_selected = 0;
		doScan();
		_task->showAlert("Rescanning...", 800);
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoToolsScreen(); return true; }
	return false;
}

CountdownScreen::CountdownScreen(JoystickUITask *task)
	: _task(task), _running(false), _end_ms(0), _set_seconds(60),
	  _edit_field(0), _alarmed(false)
{
	k_timer_init(&_alarm_timer, alarmTimerCb, NULL);
	k_timer_user_data_set(&_alarm_timer, this);
}

void CountdownScreen::alarmTimerCb(struct k_timer *t)
{
	/* ISR context — just signal refresh; render() detects the elapsed
	 * deadline against _end_ms and fires the alarm UX in main thread. */
	auto *self = static_cast<CountdownScreen *>(k_timer_user_data_get(t));
	if (self && self->_task) self->_task->notify();
}

void CountdownScreen::onExit()
{
	k_timer_stop(&_alarm_timer);
}

int CountdownScreen::render(JoystickDisplay &display)
{
	/* Alarm path: timer fired (or any wakeup arrived after deadline);
	 * fire alarm UX once and clear _running. */
	if (_running && k_uptime_get_32() >= _end_ms) {
		_running = false;
		_alarmed = true;
		_task->playCountdownAlarm();
		_task->showAlert("Countdown done!", 1000);
	}

	renderScreenHeader(display, "Countdown", 0, 0);

	int remaining = _set_seconds;
	if (_running) {
		uint32_t now = k_uptime_get_32();
		if (now < _end_ms) {
			remaining = (int)((_end_ms - now) / 1000UL) + 1;
			if (remaining < 0) remaining = 0;
		} else {
			remaining = 0;
		}
	}

	int mins = remaining / 60;
	int secs = remaining % 60;
	char time_buf[24];
	snprintf(time_buf, sizeof(time_buf), "%02d:%02d", mins, secs);

	display.setTextSize(1);
	display.setColor(_alarmed ? JoystickDisplay::YELLOW : JoystickDisplay::GREEN);
	display.drawTextCentered(display.width() / 2, 22, time_buf);

	return _running ? 200 : 500;
}

bool CountdownScreen::handleInput(char c)
{
	if (c == KEY_ENTER) {
		if (_running) {
			_running = false;
			k_timer_stop(&_alarm_timer);
		} else {
			_end_ms = k_uptime_get_32() + (uint32_t)_set_seconds * 1000UL;
			_running = true;
			_alarmed = false;
			k_timer_start(&_alarm_timer, K_MSEC((uint32_t)_set_seconds * 1000UL), K_NO_WAIT);
		}
		return true;
	}
	if (!_running && c == KEY_UP) {
		_set_seconds += 10;
		if (_set_seconds > 3600) _set_seconds = 3600;
		_alarmed = false;
		return true;
	}
	if (!_running && c == KEY_DOWN) {
		_set_seconds -= 10;
		if (_set_seconds < 10) _set_seconds = 10;
		_alarmed = false;
		return true;
	}
	if (c == KEY_ENTER_LONG) {
		_running = false;
		_alarmed = false;
		_set_seconds = 60;
		_end_ms = 0;
		k_timer_stop(&_alarm_timer);
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoToolsScreen(); return true; }
	return false;
}

StopwatchScreen::StopwatchScreen(JoystickUITask *task)
	: _task(task), _running(false), _start_ms(0), _elapsed_ms(0),
	  _laps(0), _scroll(0)
{
	memset(_lap_times, 0, sizeof(_lap_times));
}

int StopwatchScreen::render(JoystickDisplay &display)
{
	renderScreenHeader(display, "Stopwatch", 0, 0);

	uint32_t elapsed_ms = _elapsed_ms;
	if (_running) elapsed_ms += (k_uptime_get_32() - _start_ms);

	uint32_t total_secs = elapsed_ms / 1000UL;
	uint32_t mins = total_secs / 60UL;
	uint32_t secs = total_secs % 60UL;

	char time_buf[24];
	snprintf(time_buf, sizeof(time_buf), "%02lu:%02lu", (unsigned long)mins, (unsigned long)secs);
	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);
	display.drawTextCentered(display.width() / 2, 20, time_buf);

	/* Show lap times */
	if (_laps > 0) {
		int y = 32;
		for (int i = _scroll; i < _laps && y < display.height() - 1; i++) {
			char lap_buf[32];
			uint32_t lt = _lap_times[i] / 1000UL;
			snprintf(lap_buf, sizeof(lap_buf), "Lap %d: %02lu:%02lu", i + 1,
					 (unsigned long)(lt / 60UL), (unsigned long)(lt % 60UL));
			display.setColor(JoystickDisplay::LIGHT);
			display.drawTextLeftAlign(0, y, lap_buf);
			y += kLineH;
		}
	}
	return _running ? 100 : 500;
}

bool StopwatchScreen::handleInput(char c)
{
	if (c == KEY_ENTER) {
		if (_running) {
			_elapsed_ms += k_uptime_get_32() - _start_ms;
			_running = false;
		} else {
			_start_ms = k_uptime_get_32();
			_running = true;
		}
		return true;
	}
	if (c == KEY_ENTER_LONG) {
		if (_running && _laps < 10) {
			/* Record lap */
			_lap_times[_laps++] = _elapsed_ms + (k_uptime_get_32() - _start_ms);
			if (_laps > 4) _scroll = _laps - 4;
		} else {
			_running = false;
			_elapsed_ms = 0;
			_start_ms = 0;
			_laps = 0;
			_scroll = 0;
		}
		return true;
	}
	if (c == KEY_UP && _scroll > 0)           { _scroll--; return true; }
	if (c == KEY_DOWN && _scroll < _laps - 1) { _scroll++; return true; }
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoToolsScreen(); return true; }
	return false;
}

/* Game layout */
#define SNAKE_HEADER    10     /* y offset for game area */
#define SNAKE_CELL       6     /* pixels per cell */
#define SNAKE_TICK_MS  200     /* move speed */
#define SNAKE_MAX_LEN  (SnakeScreen::GRID_W * SnakeScreen::GRID_H)

/* Game state codes stored in _score (negative = special) */
#define STATE_READY    0
#define STATE_PLAYING  1
#define STATE_PAUSED   2
#define STATE_OVER     3

static int8_t s_dir_x = 1;  /* current direction */
static int8_t s_dir_y = 0;
static int8_t s_ndir_x = 1; /* queued next direction */
static int8_t s_ndir_y = 0;
static int s_state = STATE_READY;

SnakeScreen::SnakeScreen(JoystickUITask *task)
	: _task(task), _grid_w(0), _grid_h(0),
	  _max_len(0), _snake_len(0),
	  _food_x(0), _food_y(0),
	  _next_move(0), _score(0),
	  _grid_ready(false), _tick_due(false)
{
	memset(_snake_x, 0, sizeof(_snake_x));
	memset(_snake_y, 0, sizeof(_snake_y));
	k_timer_init(&_tick_timer, tickTimerCb, NULL);
	k_timer_user_data_set(&_tick_timer, this);
	reset();
}

void SnakeScreen::tickTimerCb(struct k_timer *t)
{
	auto *self = static_cast<SnakeScreen *>(k_timer_user_data_get(t));
	if (!self) return;
	self->_tick_due = true;
	if (self->_task) self->_task->notify();
}

void SnakeScreen::startTicking()
{
	k_timer_start(&_tick_timer, K_MSEC(SNAKE_TICK_MS), K_MSEC(SNAKE_TICK_MS));
}

void SnakeScreen::onEnter()
{
	if (s_state == STATE_PLAYING) startTicking();
}

void SnakeScreen::onExit()
{
	k_timer_stop(&_tick_timer);
	_tick_due = false;
}

void SnakeScreen::onDisplayOff()
{
	/* Pause the game-tick timer while the screen is off — no point burning
	 * wakes to advance a snake nobody can see. The game state is preserved;
	 * we'll resume on onDisplayOn(). */
	k_timer_stop(&_tick_timer);
	_tick_due = false;
}

void SnakeScreen::onDisplayOn()
{
	if (s_state == STATE_PLAYING) startTicking();
}

void SnakeScreen::updateGrid(JoystickDisplay &display)
{
	_grid_w = display.width() / SNAKE_CELL;
	_grid_h = (display.height() - SNAKE_HEADER) / SNAKE_CELL;

	if (_grid_w < 5) _grid_w = 5;
	if (_grid_h < 3) _grid_h = 3;

	_max_len = _grid_w * _grid_h;

	if (_max_len > MAX_SNAKE_LEN)
		_max_len = MAX_SNAKE_LEN;

	_grid_ready = true;
}

void SnakeScreen::reset()
{
	_snake_len = 3;
	s_dir_x = 1; s_dir_y = 0;
	s_ndir_x = 1; s_ndir_y = 0;
	_score = 0;
	s_state = STATE_READY;
	_tick_due = false;
	_next_move = 0;
	if (_grid_ready) {
		int cx = _grid_w / 2;
		int cy = _grid_h / 2;
		_snake_x[0] = cx;     _snake_y[0] = cy;
		_snake_x[1] = cx - 1; _snake_y[1] = cy;
		_snake_x[2] = cx - 2; _snake_y[2] = cy;
		placeFood();
	}
}

void SnakeScreen::placeFood()
{
	for (int tries = 0; tries < 200; tries++) {
		int8_t x = sys_rand32_get() % _grid_w;
		int8_t y = sys_rand32_get() % _grid_h;
		bool hit = false;
		for (int i = 0; i < _snake_len; i++) {
			if (_snake_x[i] == x && _snake_y[i] == y) {
				hit = true;
				break;
			}
		}
		if (!hit) {
			_food_x = x; _food_y = y;
			return;
		}
	}
	_food_x = 0; _food_y = 0;
}

void SnakeScreen::advanceGame()
{
	if (s_state != STATE_PLAYING) return;
	if (!_grid_ready) return;
	uint32_t now = k_uptime_get_32();
	if (now < _next_move) return;
	_next_move = now + SNAKE_TICK_MS;

	s_dir_x = s_ndir_x; s_dir_y = s_ndir_y;

	int nx = _snake_x[0] + s_dir_x;
	int ny = _snake_y[0] + s_dir_y;

	/* WALL CHECK */
	if (nx < 0 || ny < 0 || nx >= _grid_w || ny >= _grid_h) {
		s_state = STATE_OVER;
		k_timer_stop(&_tick_timer);
		return;
	}

	bool eat = (nx == _food_x && ny == _food_y);

	int body_to_check = eat ? _snake_len : (_snake_len - 1);
	for (int i = 1; i < body_to_check; i++) {
		if (_snake_x[i] == nx && _snake_y[i] == ny) {
			s_state = STATE_OVER;
			k_timer_stop(&_tick_timer);
			return;
		}
	}

	if (eat && _snake_len < _max_len) _snake_len++;

	for (int i = _snake_len - 1; i > 0; i--) {
		_snake_x[i] = _snake_x[i - 1];
		_snake_y[i] = _snake_y[i - 1];
	}
	_snake_x[0] = nx; _snake_y[0] = ny;

	if (eat) {
		_score++;
		placeFood();
	}
}

int SnakeScreen::render(JoystickDisplay &display)
{
	// Use the correct size based on the display
	updateGrid(display);

	/* Advance game on tick fire (set by k_timer ISR). */
	if (_tick_due) {
		_tick_due = false;
		if (s_state == STATE_PLAYING) {
			advanceGame();
		}
	}

	char title[24];
	snprintf(title, sizeof(title), "SNAKE Score:%d", _score);
	display.setColor(JoystickDisplay::GREEN);
	display.drawTextLeftAlign(0, 0, title);
	display.drawRect(0, SNAKE_HEADER - 1, display.width(), 1);

	/* Snake body */
	display.setColor(JoystickDisplay::GREEN);
	for (int i = 0; i < _snake_len; i++) {
		display.fillRect(_snake_x[i] * SNAKE_CELL,
						 SNAKE_HEADER + _snake_y[i] * SNAKE_CELL,
						 SNAKE_CELL, SNAKE_CELL);
	}
	/* Food (hollow center) */
	display.fillRect(_food_x * SNAKE_CELL + 1,
					 SNAKE_HEADER + _food_y * SNAKE_CELL + 1,
					 SNAKE_CELL - 2, SNAKE_CELL - 2);

	display.setColor(JoystickDisplay::LIGHT);
	if (s_state == STATE_READY) {
		display.drawTextCentered(display.width() / 2, 28, "ENTER to start");
	} else if (s_state == STATE_PAUSED) {
		display.drawTextCentered(display.width() / 2, 28, "PAUSED");
	} else if (s_state == STATE_OVER) {
		display.drawTextCentered(display.width() / 2, 28, "GAME OVER");
		display.drawTextCentered(display.width() / 2, 39, "ENTER restart");
	}
	return 50;
}

bool SnakeScreen::handleInput(char c)
{
	/* One turn per tick: if a direction change is already queued for the
	 * upcoming tick, ignore further direction input.  Also block the 180°
	 * reverse (would move head straight into body[1]).
	 * Once the tick fires, s_dir == s_ndir again and new input is accepted. */
	bool turn_queued = (s_ndir_x != s_dir_x || s_ndir_y != s_dir_y);
	if (!turn_queued) {
		if (c == KEY_UP    && s_dir_y != 1)  { s_ndir_x = 0;  s_ndir_y = -1; return true; }
		if (c == KEY_DOWN  && s_dir_y != -1) { s_ndir_x = 0;  s_ndir_y =  1; return true; }
		if (c == KEY_LEFT  && s_dir_x != 1)  { s_ndir_x = -1; s_ndir_y =  0; return true; }
		if (c == KEY_RIGHT && s_dir_x != -1) { s_ndir_x =  1; s_ndir_y =  0; return true; }
	}
	if (c == KEY_ENTER) {
		if (s_state == STATE_READY || s_state == STATE_OVER) {
			reset(); s_state = STATE_PLAYING;
			startTicking();
		} else if (s_state == STATE_PLAYING) {
			s_state = STATE_PAUSED;
			k_timer_stop(&_tick_timer);
		} else if (s_state == STATE_PAUSED) {
			s_state = STATE_PLAYING;
			startTicking();
		}
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoToolsScreen(); return true; }
	return false;
}

#ifdef CONFIG_ZEPHCORE_EASTER_EGG_DOOM
/* ===== DoomScreen ===== */

DoomScreen::DoomScreen(JoystickUITask *task) : _task(task) {}

void DoomScreen::onEnter()
{
	if (!doom_game_is_running()) {
		doom_game_start();
	}
}

int DoomScreen::render(JoystickDisplay &display)
{
	(void)display;
	return 5000;
}

bool DoomScreen::handleInput(char c)
{
	if (c == KEY_CANCEL || c == KEY_HOME) {
		doom_game_stop();
		_task->gotoToolsScreen();
		return true;
	}
	return true;
}
#endif
