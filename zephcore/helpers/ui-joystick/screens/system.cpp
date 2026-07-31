/*
 * ZephCore - Joystick UI System Screen
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "../joystick_screens.h"
#include "../joystick_ui_task.h"
#include "screen_helpers.h"
#include <adapters/gps/ZephyrGPSManager.h>
#include <helpers/time_sync.h>
#include <helpers/ui/ui_mesh_actions.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Category labels */
static const char * const kSysCatLabels[] = { "Device", "Display", "Info", "Power" };
static const int kSysCatCount = 4;

/* Device submenu items */
enum SysDevItem { SYSDEV_BUZZER=0, SYSDEV_BLUETOOTH, SYSDEV_OFFGRID, SYSDEV_LEDS, SYSDEV_BLE_CODE, SYSDEV_PATHHASH, SYSDEV_DFU, SYSDEV_TELEMETRY, SYSDEV_RENAME, SYSDEV_COUNT };

#define DFU_CONFIRM_WINDOW_MS 3000
/* Display submenu items */
enum SysDspItem { SYSDSP_BRIGHT=0, SYSDSP_SCROFF, SYSDSP_BATT, SYSDSP_WAKE, SYSDSP_COUNT };
/* Info submenu items */
enum SysInfoItem { SYSINFO_TIME=0, SYSINFO_STATS, SYSINFO_RADIO, SYSINFO_COUNT };
/* Power submenu items */
enum SysPwrItem  { SYSPWR_REBOOT=0, SYSPWR_SHUTDOWN, SYSPWR_COUNT };

static void renderSubMenu(JoystickDisplay &display, const char *title,
		int selected, const char * const *items, int count)
{
	renderScreenHeader(display, title, selected, count);
	int start = computeListStart(selected, count);
	int y = kContentY + 2;
	for (int i = start; i < count && i < start + UI_RECENT_LIST_SIZE; i++) {
		renderMenuListText(display, y, i == selected, items[i]);
		y += kMenuLineH;
	}
}

SystemScreen::SystemScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0), _cat_selected(0), _mode(SYSMODE_TOP),
	  _dfu_confirm_time(0)
{
}

int SystemScreen::render(JoystickDisplay &display)
{
	if (_mode == SYSMODE_TOP) {
		renderSubMenu(display, "System", _selected, kSysCatLabels, kSysCatCount);
		return 500;
	}
	if (_mode == SYSMODE_DEVICE) {
		char bt_label[24], buz_label[24], og_label[24], led_label[24], dfu_label[24], ph_label[24];
		snprintf(buz_label, sizeof(buz_label), "Buzzer: %s", _task->isBuzzerQuiet() ? "OFF" : "ON");
		snprintf(bt_label, sizeof(bt_label), "Bluetooth: %s", _task->isSerialEnabled() ? "ON" : "OFF");
		snprintf(og_label, sizeof(og_label), "Offgrid: %s", _task->isOffgridEnabled() ? "ON" : "OFF");
		snprintf(led_label, sizeof(led_label), "LEDs: %s", _task->isLedsDisabled() ? "OFF" : "ON");
		snprintf(ph_label, sizeof(ph_label), "Path hash: %db", (int)_task->getPathHashBytes());
		uint32_t now = k_uptime_get_32();
		bool dfu_pending = (_dfu_confirm_time != 0 &&
							(now - _dfu_confirm_time) <= DFU_CONFIRM_WINDOW_MS);
		snprintf(dfu_label, sizeof(dfu_label), "BLE DFU%s", dfu_pending ? " (confirm)" : "");
		const char *items[SYSDEV_COUNT];
		items[SYSDEV_BUZZER] = buz_label;
		items[SYSDEV_BLUETOOTH] = bt_label;
		items[SYSDEV_BLE_CODE] = "BLE Code";
		items[SYSDEV_PATHHASH] = ph_label;
		items[SYSDEV_TELEMETRY] = "Telemetry";
		items[SYSDEV_RENAME] = "Rename node";
		items[SYSDEV_OFFGRID] = og_label;
		items[SYSDEV_LEDS] = led_label;
		items[SYSDEV_DFU] = dfu_label;
		renderSubMenu(display, "Device", _selected, items, SYSDEV_COUNT);
		return 500;
	}
	if (_mode == SYSMODE_DISPLAY) {
		char bright_label[24], scroff_label[24], batt_label[24];
		snprintf(bright_label, sizeof(bright_label), "Brightness: %d%%", (int)_task->getBrightness());
		snprintf(scroff_label, sizeof(scroff_label), "Screen off: %lus",
				 (unsigned long)(_task->getScreenOffMillis() / 1000UL));
		snprintf(batt_label, sizeof(batt_label), "Battery: %s",
				 _task->getBatteryDisplayMode() == 1 ? "Voltage" : "Percent");
		const char *items[SYSDSP_COUNT];
		items[SYSDSP_BRIGHT] = bright_label;
		items[SYSDSP_SCROFF] = scroff_label;
		items[SYSDSP_BATT] = batt_label;
		items[SYSDSP_WAKE] = _task->getWakeOnMsg() ? "Wake on msg: ON" : "Wake on msg: OFF";
		renderSubMenu(display, "Display", _selected, items, SYSDSP_COUNT);
		return 500;
	}
	if (_mode == SYSMODE_INFO) {
		static const char * const kItems[SYSINFO_COUNT] = { "Time", "System Stats", "Radio Stats" };
		renderSubMenu(display, "Info", _selected, kItems, SYSINFO_COUNT);
		return 500;
	}
	if (_mode == SYSMODE_POWER) {
		static const char * const kItems[SYSPWR_COUNT] = { "Reboot", "Shutdown" };
		renderSubMenu(display, "Power", _selected, kItems, SYSPWR_COUNT);
		return 500;
	}
	if (_mode == SYSMODE_OFFGRID_CONFIRM) {
		renderScreenHeader(display, "Offgrid Mode", 0, 0);
		int cx = display.width() / 2;
		display.setColor(JoystickDisplay::LIGHT);
		display.fillRect(0, kContentY + 2, display.width(), 18);
		display.setColor(JoystickDisplay::DARK);
		display.drawTextCentered(cx, kContentY + 10, "EMERGENCY USE ONLY");
		display.setColor(JoystickDisplay::GREEN);
		display.drawTextCentered(cx, kContentY + 30, "ENTER: enable");
		display.setColor(JoystickDisplay::LIGHT);
		display.drawTextCentered(cx, kContentY + 41, "CANCEL: abort");
		return 500;
	}
	return 500;
}

bool SystemScreen::handleInput(char c)
{
	if (_mode == SYSMODE_TOP) {
		if (handleCommonListNavigation(c, _selected, kSysCatCount)) return true;
		if (c == KEY_ENTER || c == KEY_RIGHT) {
			_cat_selected = _selected;
			_mode = (SystemMode)(_selected + 1);
			_selected = 0;
			return true;
		}
		if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoHomeScreen(); return true; }
		return false;
	}
	if (_mode == SYSMODE_DEVICE) {
		if (handleCommonListNavigation(c, _selected, SYSDEV_COUNT)) return true;
		if (c == KEY_ENTER) {
			switch (_selected) {
			case SYSDEV_BLUETOOTH:
				if (_task->isSerialEnabled()) _task->disableSerial(); else _task->enableSerial();
				_task->showAlert(_task->isSerialEnabled() ? "BT enabled" : "BT disabled", 1000);
				return true;
			case SYSDEV_TELEMETRY: _task->gotoTelemetryScreen(); return true;
			case SYSDEV_RENAME:    _task->gotoRenameNodeScreen(); return true;
			case SYSDEV_BLE_CODE:  _task->gotoBLECodeScreen();    return true;
			case SYSDEV_BUZZER:
				_task->toggleBuzzer();
				_task->showAlert(_task->isBuzzerQuiet() ? "Buzzer off" : "Buzzer on", 1000);
				return true;
			case SYSDEV_OFFGRID: {
				if (_task->isOffgridEnabled()) {
					_task->setOffgridMode(false);
					_task->showAlert("Offgrid: OFF", 1000);
				} else {
					_mode = SYSMODE_OFFGRID_CONFIRM;
				}
				return true;
			}
			case SYSDEV_LEDS:
				_task->toggleLeds();
				_task->showAlert(_task->isLedsDisabled() ? "LEDs: OFF" : "LEDs: ON", 1000);
				return true;
			case SYSDEV_PATHHASH: {
				_task->cyclePathHashMode();
				char alert[24];
				snprintf(alert, sizeof(alert), "Path hash: %db", (int)_task->getPathHashBytes());
				_task->showAlert(alert, 1000);
				return true;
			}
			case SYSDEV_DFU: {
				uint32_t now = k_uptime_get_32();
				if (_dfu_confirm_time != 0 &&
					(now - _dfu_confirm_time) <= DFU_CONFIRM_WINDOW_MS) {
					_task->rebootToDFU();
				} else {
					_dfu_confirm_time = now;
					_task->showAlert("Press ENTER to confirm", 2500);
				}
				return true;
			}
			default: return false;
			}
		}
		if (c == KEY_CANCEL || c == KEY_LEFT) {
			_dfu_confirm_time = 0;
			_selected = _cat_selected;
			_mode = SYSMODE_TOP;
			return true;
		}
		if (c == KEY_HOME) { _dfu_confirm_time = 0; _task->gotoHomeScreen(); return true; }
		return false;
	}
	if (_mode == SYSMODE_DISPLAY) {
		if (handleCommonListNavigation(c, _selected, SYSDSP_COUNT)) return true;
		if (c == KEY_LEFT || c == KEY_RIGHT) {
			int delta = (c == KEY_RIGHT) ? 1 : -1;
			if (_selected == SYSDSP_BRIGHT) { _task->adjustBrightness(delta * 5); return true; }
			if (_selected == SYSDSP_SCROFF) { _task->adjustScreenOff(delta * 5000L); return true; }
			if (c == KEY_LEFT) { _selected = _cat_selected; _mode = SYSMODE_TOP; return true; }
			return false;
		}
		if (c == KEY_ENTER) {
			switch (_selected) {
			case SYSDSP_BRIGHT: _task->adjustBrightness(5); return true;
			case SYSDSP_SCROFF: _task->adjustScreenOff(5000L); return true;
			case SYSDSP_BATT:
				_task->toggleBatteryDisplayMode();
				_task->showAlert(_task->getBatteryDisplayMode() == 1 ? "Battery: Voltage" : "Battery: Percent", 1000);
				return true;
			case SYSDSP_WAKE:
				_task->toggleWakeOnMsg();
				_task->showAlert(_task->getWakeOnMsg() ? "Wake on msg: ON" : "Wake on msg: OFF", 1000);
				return true;
			default: return false;
			}
		}
		if (c == KEY_CANCEL) { _selected = _cat_selected; _mode = SYSMODE_TOP; return true; }
		if (c == KEY_HOME)   { _task->gotoHomeScreen(); return true; }
		return false;
	}
	if (_mode == SYSMODE_INFO) {
		if (handleCommonListNavigation(c, _selected, SYSINFO_COUNT)) return true;
		if (c == KEY_ENTER) {
			switch (_selected) {
			case SYSINFO_TIME:  _task->gotoSystemTimeScreen(); return true;
			case SYSINFO_STATS: _task->gotoStatsScreen();      return true;
			case SYSINFO_RADIO: _task->gotoRadioStatsScreen(); return true;
			default: return false;
			}
		}
		if (c == KEY_CANCEL || c == KEY_LEFT) { _selected = _cat_selected; _mode = SYSMODE_TOP; return true; }
		if (c == KEY_HOME) { _task->gotoHomeScreen(); return true; }
		return false;
	}
	if (_mode == SYSMODE_POWER) {
		if (handleCommonListNavigation(c, _selected, SYSPWR_COUNT)) return true;
		if (c == KEY_ENTER) {
			switch (_selected) {
			case SYSPWR_REBOOT:   _task->showAlert("Rebooting...", 1500);  _task->shutdown(true);  return true;
			case SYSPWR_SHUTDOWN: _task->showAlert("Shutting down...", 1500); _task->shutdown(false); return true;
			default: return false;
			}
		}
		if (c == KEY_CANCEL || c == KEY_LEFT) { _selected = _cat_selected; _mode = SYSMODE_TOP; return true; }
		if (c == KEY_HOME) { _task->gotoHomeScreen(); return true; }
		return false;
	}
	if (_mode == SYSMODE_OFFGRID_CONFIRM) {
		if (c == KEY_ENTER) {
			_task->setOffgridMode(true);
			_task->showAlert("Offgrid: ON", 1000);
			_mode = SYSMODE_DEVICE;
			return true;
		}
		if (c == KEY_CANCEL || c == KEY_LEFT) { _mode = SYSMODE_DEVICE; return true; }
		if (c == KEY_HOME) { _mode = SYSMODE_DEVICE; _task->gotoHomeScreen(); return true; }
		return false;
	}
	return false;
}

static void formatUnixDateTime(uint32_t t, char *timeBuf, size_t timeLen,
		char *dateBuf, size_t dateLen)
{
	uint32_t seconds = t % 60;
	uint32_t minutes = (t / 60) % 60;
	uint32_t hours = (t / 3600) % 24;
	uint32_t days = t / 86400;
	uint16_t year = 1970; uint8_t month = 1, day = 1;

	while (true) {
		bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
		uint16_t yd = leap ? 366 : 365;
		if (days >= yd) { days -= yd; year++; } else break;
	}
	static const uint8_t kMD[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	for (int i = 0; i < 12; i++) {
		uint8_t md = kMD[i];
		if (i == 1) {
			bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
			if (leap) md = 29;
		}
		if (days >= md) { days -= md; month++; } else { day += (uint8_t)days; break; }
	}
	snprintf(timeBuf, timeLen, "%02u:%02u:%02u", (unsigned)hours, (unsigned)minutes, (unsigned)seconds);
	snprintf(dateBuf, dateLen, "%04u-%02u-%02u", (unsigned)year,  (unsigned)month,   (unsigned)day);
}

SystemTimeScreen::SystemTimeScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc)
{
}

int SystemTimeScreen::render(JoystickDisplay &display)
{
	renderScreenHeader(display, "Time", 0, 1);
	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);

	char timeText[16], dateText[16];
	formatUnixDateTime(_rtc->getCurrentTime(), timeText, sizeof(timeText), dateText, sizeof(dateText));

	const char *labels[4] = { "Time:", "Date:", "TZ:", "Source:" };
	const char *values[4] = { timeText, dateText, "UTC",
							   time_sync_display_label() };
	int y = kContentY + 2;
	for (int i = 0; i < 4; i++) {
		display.setColor(JoystickDisplay::GREEN);
		display.drawTextLeftAlign(0, y, labels[i]);
		display.drawTextRightAlign(display.width() - 1, y, values[i]);
		y += kMenuLineH;
	}
	return 500;
}

bool SystemTimeScreen::handleInput(char c)
{
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoSystemScreen(); return true; }
	return false;
}

RenameNodeScreen::RenameNodeScreen(JoystickUITask *task)
	: _task(task), _loaded(false), _cursor(0), _selected_key(0),
	  _last_press_time(0), _letter_index(0), _last_key(-1),
	  _kb_mode_letters(true)
{
	memset(_input, 0, sizeof(_input));
}

int RenameNodeScreen::render(JoystickDisplay &display)
{
	if (!_loaded) {
		NodePrefs *prefs = _task->getPrefs();
		if (prefs) {
			strncpy(_input, prefs->node_name, sizeof(_input) - 1);
			_input[sizeof(_input) - 1] = '\0';
		}
		_cursor = strlen(_input);
		_loaded = true;
	}

	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);
	char display_text[36];
	snprintf(display_text, sizeof(display_text), "%s|", _input);
	display.drawTextLeftAlign(0, 0, display_text);

	renderT9Keypad(display, getT9KeyLabels(true, true), _selected_key, 12);
	return 400;
}

bool RenameNodeScreen::handleInput(char c)
{
	if (handleT9DirectionalInput(c, _selected_key)) return true;

	if (c == KEY_ENTER) {
		if (_selected_key == 3) {   /* del */
			if (_cursor > 0) _input[--_cursor] = '\0';
			resetT9State(_last_key, _letter_index, _last_press_time);
			return true;
		}
		if (_selected_key == 7) {   /* space */
			if (_cursor < (int)sizeof(_input) - 1) {
				_input[_cursor++] = ' ';
				_input[_cursor] = '\0';
			}
			resetT9State(_last_key, _letter_index, _last_press_time);
			return true;
		}
		if (_selected_key == 11) {  /* save */
			while (_cursor > 0 && _input[_cursor - 1] == ' ') _input[--_cursor] = '\0';
			NodePrefs *prefs = _task->getPrefs();
			if (prefs) {
				strncpy(prefs->node_name, _input, sizeof(prefs->node_name) - 1);
				prefs->node_name[sizeof(prefs->node_name) - 1] = '\0';
			}
			resetT9State(_last_key, _letter_index, _last_press_time);
			_loaded = false;
			_task->showAlert("Node name saved", 1000);
			_task->gotoSystemScreen();
			return true;
		}
		if (_selected_key == 15) {  /* back */
			resetT9State(_last_key, _letter_index, _last_press_time);
			_loaded = false;
			_task->gotoSystemScreen();
			return true;
		}

		const char *letters = getT9KeyLetters(true)[_selected_key];
		appendOrCycleT9Char(_input, _cursor, sizeof(_input), letters, _selected_key,
							_last_key, _letter_index, _last_press_time);
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) {
		resetT9State(_last_key, _letter_index, _last_press_time);
		_loaded = false;
		_task->gotoSystemScreen();
		return true;
	}
	return false;
}

BLECodeScreen::BLECodeScreen(JoystickUITask *task)
	: _task(task), _editing(false), _digit_sel(0)
{
	memset(_pin_buf, 0, sizeof(_pin_buf));
	NodePrefs *prefs = task->getPrefs();
	if (prefs) snprintf(_pin_buf, sizeof(_pin_buf), "%06lu", (unsigned long)prefs->ble_pin);
	else snprintf(_pin_buf, sizeof(_pin_buf), "000000");
}

int BLECodeScreen::render(JoystickDisplay &display)
{
	renderScreenHeader(display, "BLE Code", 0, 0);
	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);
	display.drawTextCentered(display.width() / 2, 22, _pin_buf);

	if (_editing) {
		/* Underline the selected digit */
		int char_w = display.fontW();
		int total_w = 6 * char_w;
		int x = display.width() / 2 - total_w / 2 + _digit_sel * char_w;
		display.setColor(JoystickDisplay::YELLOW);
		display.drawRect(x, 31, char_w, 2);
		display.setColor(JoystickDisplay::LIGHT);
		display.drawTextCentered(display.width() / 2, 41, "U/D digit L/R move");
		display.drawTextCentered(display.width() / 2, 52, "ENTER save");
	} else {
		display.setColor(JoystickDisplay::LIGHT);
		display.drawTextCentered(display.width() / 2, 38, "ENTER edit LONG random");
		display.drawTextCentered(display.width() / 2, 49, "CANCEL back");
	}
	return 500;
}

bool BLECodeScreen::handleInput(char c)
{
	if (_editing) {
		if (c == KEY_LEFT)  { if (_digit_sel > 0) _digit_sel--;   return true; }
		if (c == KEY_RIGHT) { if (_digit_sel < 5) _digit_sel++;   return true; }
		if (c == KEY_UP || c == KEY_DOWN) {
			char d = _pin_buf[_digit_sel];
			if (d < '0' || d > '9') d = '0';
			int v = d - '0';
			v = (c == KEY_UP) ? (v + 1) % 10 : (v + 9) % 10;
			_pin_buf[_digit_sel] = (char)('0' + v);
			return true;
		}
		if (c == KEY_ENTER) {
			NodePrefs *prefs = _task->getPrefs();
			if (prefs) {
				prefs->ble_pin = (uint32_t)atoi(_pin_buf);
			}
			_editing = false;
			mesh_save_and_restart();
			_task->showAlert("Restarting...", 2000);
			return true;
		}
		if (c == KEY_CANCEL || c == KEY_HOME) {
			_editing = false;
			NodePrefs *prefs = _task->getPrefs();
			if (prefs) snprintf(_pin_buf, sizeof(_pin_buf), "%06lu", (unsigned long)prefs->ble_pin);
			return true;
		}
		return false;
	}

	if (c == KEY_ENTER_LONG) {
		uint32_t r;
		sys_csrand_get(&r, sizeof(r));
		uint32_t pin = 100000UL + (r % 900000UL);
		NodePrefs *prefs = _task->getPrefs();
		if (prefs) { prefs->ble_pin = pin; }
		snprintf(_pin_buf, sizeof(_pin_buf), "%06lu", (unsigned long)pin);
		mesh_save_and_restart();
		_task->showAlert("Restarting...", 2000);
		return true;
	}
	if (c == KEY_ENTER || c == KEY_RIGHT || c == KEY_LEFT) {
		_editing = true;
		_digit_sel = 0;
		NodePrefs *prefs = _task->getPrefs();
		if (prefs) snprintf(_pin_buf, sizeof(_pin_buf), "%06lu", (unsigned long)prefs->ble_pin);
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoSystemScreen(); return true; }
	return false;
}

static const int ADVERT_ITEM_SETTINGS = 0;
static const int ADVERT_ITEM_SEND_ZEROHOP = 1;
static const int ADVERT_ITEM_SEND_FLOOD = 2;
static const int ADVERT_ITEM_BASE_COUNT = 3;

AdvertScreen::AdvertScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0), _last_rendered_selected(-1),
	  _marquee_start_ms(0), _settings_open(false),
	  _recent_advert_count(0), _recent_refresh_at(0)
{
	memset(_recent_adverts, 0, sizeof(_recent_adverts));
}

void AdvertScreen::refreshRecentAdverts()
{
	uint32_t now = k_uptime_get_32();
	if (_recent_refresh_at != 0 && (now - _recent_refresh_at) < 700) return;
	_recent_refresh_at = now;

	int total = _task->getRecentlyHeard(_recent_adverts, 16);
	_recent_advert_count = 0;
	for (int i = 0; i < total; i++) {
		if (_recent_adverts[i].recv_timestamp == 0 || _recent_adverts[i].name[0] == '\0') continue;
		if (i != _recent_advert_count) _recent_adverts[_recent_advert_count] = _recent_adverts[i];
		_recent_advert_count++;
	}
}

int AdvertScreen::getRecentAdvertCount() const { return _recent_advert_count; }

bool AdvertScreen::getRecentAdvertByIndex(int idx, struct AdvertPath &path) const
{
	if (idx < 0 || idx >= _recent_advert_count) return false;
	path = _recent_adverts[idx];
	return true;
}

int AdvertScreen::render(JoystickDisplay &display)
{
	if (_settings_open) {
		renderScreenHeader(display, "Advert settings", 0, 0);
		display.setColor(JoystickDisplay::GREEN);
		display.drawTextCentered(display.width() / 2, 28, "No settings yet");
		return 1000;
	}

	refreshRecentAdverts();
	int recent_count = getRecentAdvertCount();
	int item_count = ADVERT_ITEM_BASE_COUNT + recent_count;
	if (_selected >= item_count) _selected = item_count - 1;
	if (_selected < 0) _selected = 0;

	if (_selected != _last_rendered_selected) {
		_marquee_start_ms = k_uptime_get_32();
		_last_rendered_selected = _selected;
	}

	renderScreenHeader(display, "Advert", _selected, item_count);
	int start = computeListStart(_selected, item_count);
	int y = kContentY + 1;
	for (int i = start; i < item_count && i < start + UI_RECENT_LIST_SIZE; i++) {
		char line[48];
		if (i == ADVERT_ITEM_SETTINGS) {
			snprintf(line, sizeof(line), "Settings");
		} else if (i == ADVERT_ITEM_SEND_ZEROHOP) {
			snprintf(line, sizeof(line), "Send advert 0hop");
		} else if (i == ADVERT_ITEM_SEND_FLOOD) {
			snprintf(line, sizeof(line), "Send advert flood");
		} else {
			AdvertPath path;
			if (!getRecentAdvertByIndex(i - ADVERT_ITEM_BASE_COUNT, path)) continue;
			int hops = (int)(path.path_len & 63);
			char suffix[16];
			if (hops <= 0) snprintf(suffix, sizeof(suffix), " [direct]");
			else snprintf(suffix, sizeof(suffix), " [%dh]", hops);
			snprintf(line, sizeof(line), "%s%s", path.name, suffix);
		}
		renderMenuListText(display, y, i == _selected, line, true, 450, _marquee_start_ms);
		if (i == ADVERT_ITEM_SEND_FLOOD)
			display.drawRect(0, y + kLineH, display.width(), 1);
		y += kMenuLineH;
	}
	return 500;
}

bool AdvertScreen::handleInput(char key)
{
	if (_settings_open) {
		if (key == KEY_CANCEL || key == KEY_HOME || key == KEY_ENTER) {
			_settings_open = false;
			return true;
		}
		return false;
	}

	int item_count = ADVERT_ITEM_BASE_COUNT + getRecentAdvertCount();
	if (item_count < ADVERT_ITEM_BASE_COUNT) item_count = ADVERT_ITEM_BASE_COUNT;
	if (handleCommonListNavigation(key, _selected, item_count)) return true;

	if (key == KEY_ENTER) {
		const char *node_name = _task->getPrefs() ? _task->getPrefs()->node_name : "Node";
		auto *mesh = _task->getMesh();

		if (_selected == ADVERT_ITEM_SETTINGS) {
			_settings_open = true;
			return true;
		}
		if (_selected == ADVERT_ITEM_SEND_ZEROHOP) {
			mesh::Packet *pkt = mesh->createSelfAdvert(node_name);
			if (pkt) { mesh->sendZeroHop(pkt); _task->showAlert("Advert 0hop sent", 1000); }
			else     { _task->showAlert("Advert failed", 1000); }
			return true;
		}
		if (_selected == ADVERT_ITEM_SEND_FLOOD) {
			/* Canonical scoped path: honors prefs.path_hash_mode and the
			 * default transport scope (a bare sendFlood() would drop both). */
			if (mesh->sendSelfAdvert(true)) { _task->showAlert("Advert flood sent", 1000); }
			else                            { _task->showAlert("Advert failed", 1000); }
			return true;
		}

		AdvertPath path;
		if (getRecentAdvertByIndex(_selected - ADVERT_ITEM_BASE_COUNT, path)) {
			ContactInfo *c = mesh->lookupContactByPubKey(path.pubkey_prefix,
														  sizeof(path.pubkey_prefix));
			if (c) _task->gotoContactForAdvert(path.pubkey_prefix, sizeof(path.pubkey_prefix));
			else _task->showAlert("Not in contacts", 1000);
		}
		return true;
	}
	if (key == KEY_CANCEL || key == KEY_HOME) { _task->gotoHomeScreen(); return true; }
	return false;
}

#define GPS_SPEED_MIN_M    2.0f    /* min move to count as motion (m) */
#define GPS_HEADING_MIN_M  5.0f   /* min move to update heading (m) */
#define GPS_MAX_SPEED_KMH  300.0f
#define GPS_HEADING_HOLD_MS  4000
#define ALT_CYCLE_MS         4000  /* lat/lon ↔ altitude flip period while a fix is held */

GPSSettingsScreen::GPSSettingsScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0),
	  _have_prev_fix(false), _prev_lat_e6(0), _prev_lon_e6(0),
	  _last_sample_ms(0), _speed_kmh(0.0f), _heading_deg(0.0f),
	  _heading_valid(false), _heading_hold_until(0),
	  _show_alt(false), _alt_cycle_ms(0)
{
	k_timer_init(&_sample_timer, sampleTimerCb, NULL);
	k_timer_user_data_set(&_sample_timer, this);
}

void GPSSettingsScreen::sampleTimerCb(struct k_timer *t)
{
	/* ISR context — just wake the main loop; sampling happens in
	 * render() (main thread) via sampleGPS(). */
	auto *self = static_cast<GPSSettingsScreen *>(k_timer_user_data_get(t));
	if (self && self->_task) self->_task->notify();
}

void GPSSettingsScreen::onEnter()
{
	/* Sample GPS once per second while this screen is active. */
	k_timer_start(&_sample_timer, K_MSEC(1000), K_MSEC(1000));
}

void GPSSettingsScreen::onExit()
{
	k_timer_stop(&_sample_timer);
}

void GPSSettingsScreen::onDisplayOff()
{
	/* Pause the 1 Hz GPS sample loop while the screen is off — the user
	 * can't see speed/heading and we're just burning ADC + math. */
	k_timer_stop(&_sample_timer);
}

void GPSSettingsScreen::onDisplayOn()
{
	k_timer_start(&_sample_timer, K_MSEC(1000), K_MSEC(1000));
}

void GPSSettingsScreen::sampleGPS()
{
	if (!_task->isGPSAvailable() || !_task->getGPSState()) return;
	struct gps_position pos;
	gps_get_position(&pos);
	if (!pos.valid) return;

	uint32_t now = k_uptime_get_32();
	if (now - _last_sample_ms < 1000) return;

	int32_t lat = (int32_t)(pos.latitude_ndeg  / 1000LL);
	int32_t lon = (int32_t)(pos.longitude_ndeg / 1000LL);

	if (!_have_prev_fix) {
		_prev_lat_e6 = lat; _prev_lon_e6 = lon;
		_have_prev_fix = true; _last_sample_ms = now;
		return;
	}

	float dt = (now - _last_sample_ms) / 1000.0f;
	float dist = gpsDistanceM(_prev_lat_e6, _prev_lon_e6, lat, lon);

	if (dist >= GPS_SPEED_MIN_M && dt > 0.0f) {
		float v = (dist / dt) * 3.6f;
		if (v <= GPS_MAX_SPEED_KMH) _speed_kmh = _speed_kmh * 0.7f + v * 0.3f;
	} else {
		_speed_kmh *= 0.85f;
		if (_speed_kmh < 0.5f) _speed_kmh = 0.0f;
	}

	if (dist >= GPS_HEADING_MIN_M) {
		float bearing = gpsBearingDeg(_prev_lat_e6, _prev_lon_e6, lat, lon);
		_heading_deg = _heading_deg * 0.65f + bearing * 0.35f;
		_heading_valid = true;
		_heading_hold_until = now + GPS_HEADING_HOLD_MS;
	} else if (now > _heading_hold_until) {
		_heading_valid = false;
	}

	_prev_lat_e6 = lat; _prev_lon_e6 = lon;
	_last_sample_ms = now;
}

int GPSSettingsScreen::render(JoystickDisplay &display)
{
	sampleGPS();  /* timer fires every 1s and wakes us here; sample now */

	bool gps_enabled = _task->getGPSState();

	char top_line[24];
	char satellites_line[32];
	char lat_lon_line[40];
	char speed_line[32];

	/* Line 0 doubles as the GPS on/off toggle and the duty-cycle setting —
	 * the display only has room for 4 lines total (64px OLED), so rather
	 * than add a 5th line, _selected swaps what line 0 shows/does. */
	if (_selected == 0) {
		snprintf(top_line, sizeof(top_line), "GPS: %s", gps_enabled ? "ON" : "OFF");
	} else {
		uint32_t duty = _task->getGpsDutySec();
		if (duty == 0) {
			snprintf(top_line, sizeof(top_line), "Duty: Always on");
		} else {
			snprintf(top_line, sizeof(top_line), "Duty: %us", (unsigned)duty);
		}
	}

	bool has_fix = false;

	if (!_task->isGPSAvailable()) {
		snprintf(satellites_line, sizeof(satellites_line), "No GPS");
		snprintf(lat_lon_line, sizeof(lat_lon_line), "Lat | Lon");
		snprintf(speed_line, sizeof(speed_line), "Speed | Compass");
	} else {
		struct gps_position pos;
		gps_get_position(&pos);

		int sat = (int)pos.satellites;
		const char *fix_str;
		if (gps_enabled && pos.valid) {
			fix_str = (sat >= 4) ? "3D FIX" : (sat >= 3) ? "2D FIX" : "NO FIX";
			if (sat >= 3) has_fix = true;
		} else {
			fix_str = "NO FIX";
		}

		snprintf(satellites_line, sizeof(satellites_line), "Sat: %d (%s)", sat, fix_str);

		if (has_fix) {
			/* Each fresh fix starts on lat/lon, flips every ALT_CYCLE_MS. */
			uint32_t now = k_uptime_get_32();
			if (_alt_cycle_ms == 0) {
				_alt_cycle_ms = now + ALT_CYCLE_MS;
			} else if (now >= _alt_cycle_ms) {
				_show_alt = !_show_alt;
				_alt_cycle_ms = now + ALT_CYCLE_MS;
			}

			if (_show_alt) {
				float alt_m = (float)pos.altitude_mm / 1000.0f;
				snprintf(lat_lon_line, sizeof(lat_lon_line), "Alt: %.1f m", (double)alt_m);
			} else {
				double lat = (double)(pos.latitude_ndeg  / 1000000LL) / 1000.0;
				double lon = (double)(pos.longitude_ndeg / 1000000LL) / 1000.0;
				snprintf(lat_lon_line, sizeof(lat_lon_line), "%.4f%c %.4f%c",
						 fabs(lat), lat >= 0 ? 'N' : 'S',
						 fabs(lon), lon >= 0 ? 'E' : 'W');
			}

			if (_heading_valid) {
				snprintf(speed_line, sizeof(speed_line), "%.1f km/h | %s %.0f",
						 (double)_speed_kmh, compassDir(_heading_deg), (double)_heading_deg);
			} else {
				snprintf(speed_line, sizeof(speed_line), "%.1f km/h", (double)_speed_kmh);
			}
		} else {
			/* No fix — reset cycle so the next fix starts on lat/lon. */
			_alt_cycle_ms = 0;
			_show_alt = false;
			snprintf(lat_lon_line, sizeof(lat_lon_line), "Lat | Lon");
			snprintf(speed_line, sizeof(speed_line), "Speed | Compass");
		}
	}

	const char *lines[4] = { top_line, satellites_line, lat_lon_line, speed_line };

	renderScreenHeader(display, "GPS", _selected, 2);
	display.setTextSize(1);

	int y = kContentY + 2;
	for (int i = 0; i < 4; i++) {
		renderMenuListText(display, y, i == 0, lines[i]);
		y += kMenuLineH;
	}

	return has_fix ? 1000 : 2000;
}

bool GPSSettingsScreen::handleInput(char key)
{
	if (handleCommonListNavigation(key, _selected, 2)) return true;
	if (key == KEY_LEFT || key == KEY_RIGHT) {
		if (_selected == 1) {
			_task->adjustGpsDuty(key == KEY_RIGHT ? 1 : -1);
			return true;
		}
		return false;
	}
	if (key == KEY_ENTER) {
		if (_selected == 0) {
			_task->toggleGPS();
			_task->showAlert(_task->getGPSState() ? "GPS enabled" : "GPS disabled", 1000);
		} else {
			_task->adjustGpsDuty(1);
		}
		return true;
	}
	if (key == KEY_CANCEL || key == KEY_HOME) { _task->gotoHomeScreen(); return true; }
	return false;
}

RadioStatsScreen::RadioStatsScreen(JoystickUITask *task)
	: _task(task)
{
}

int RadioStatsScreen::render(JoystickDisplay &display)
{
	renderScreenHeader(display, "Radio Stats", 0, 0);
	display.setTextSize(1);

	char line[40];
	int y = kContentY;

	snprintf(line, sizeof(line), "Floor: %d dBm", (int)_task->getNoiseFloor());
	display.setColor(JoystickDisplay::YELLOW);
	display.drawTextLeftAlign(0, y, line);
	y += kMenuLineH;

	snprintf(line, sizeof(line), "Rx: %lu Tx: %lu",
			 (unsigned long)_task->getPktRecv(),
			 (unsigned long)_task->getPktSent());
	display.setColor(JoystickDisplay::GREEN);
	display.drawTextLeftAlign(0, y, line);
	y += kMenuLineH;

	uint32_t errs = _task->getPktErrors();
	if (errs > 0) {
		snprintf(line, sizeof(line), "Err: %lu", (unsigned long)errs);
		display.setColor(JoystickDisplay::LIGHT);
		display.drawTextLeftAlign(0, y, line);
	}
	return 1000;
}

bool RadioStatsScreen::handleInput(char key)
{
	if (key == KEY_CANCEL || key == KEY_HOME) { _task->gotoSystemScreen(); return true; }
	return false;
}

StatsScreen::StatsScreen(JoystickUITask *task)
	: _task(task)
{
}

int StatsScreen::render(JoystickDisplay &display)
{
	renderScreenHeader(display, "Stats", 0, 0);
	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);

	char line[32];
	int y = kContentY + 2;

	uint32_t uptime_s = k_uptime_get_32() / 1000UL;
	if (uptime_s < 60)
		snprintf(line, sizeof(line), "Uptime: %us", (unsigned)uptime_s);
	else if (uptime_s < 3600)
		snprintf(line, sizeof(line), "Uptime: %um", (unsigned)(uptime_s / 60));
	else
		snprintf(line, sizeof(line), "Uptime: %uh", (unsigned)(uptime_s / 3600));
	display.drawTextLeftAlign(0, y, line);
	y += kMenuLineH;

	snprintf(line, sizeof(line), "Contacts: %d/%d",
			 _task->getMesh()->getNumContacts(), MAX_CONTACTS);
	display.drawTextLeftAlign(0, y, line);
	y += kMenuLineH;

	snprintf(line, sizeof(line), "Msgs: %d", _task->getStoredMsgCount());
	display.drawTextLeftAlign(0, y, line);

	return 1000;
}

bool StatsScreen::handleInput(char c)
{
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoSystemScreen(); return true; }
	return false;
}

TelemetryScreen::TelemetryScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0)
{
}

int TelemetryScreen::render(JoystickDisplay &display)
{
	static const char * const kLabels[3] = { "Allow req", "Locations", "Env" };
	static const char * const kStates[3] = { "no", "contacts", "yes" };
	const int kItemCount = 3;
	char line[32];

	renderScreenHeader(display, "Telemetry", _selected, kItemCount);
	NodePrefs *prefs = _task->getPrefs();
	uint8_t values[3] = { 0, 0, 0 };
	if (prefs) {
		values[0] = prefs->telemetry_mode_base;
		values[1] = prefs->telemetry_mode_loc;
		values[2] = prefs->telemetry_mode_env;
	}

	int y = kContentY + 2;
	for (int i = 0; i < kItemCount; i++) {
		snprintf(line, sizeof(line), "%s: %s", kLabels[i], kStates[values[i] % kItemCount]);
		renderMenuListText(display, y, i == _selected, line);
		y += kMenuLineH;
	}
	return 500;
}

bool TelemetryScreen::handleInput(char c)
{
	const int kItemCount = 3;
	static const char * const kStates[3] = { "no", "contacts", "yes" };

	if (handleCommonListNavigation(c, _selected, kItemCount)) return true;

	if (c == KEY_ENTER) {
		NodePrefs *prefs = _task->getPrefs();
		if (prefs) {
			uint8_t *field = nullptr;
			const char *label = "";
			if (_selected == 0) { field = &prefs->telemetry_mode_base; label = "Allow req"; }
			else if (_selected == 1) { field = &prefs->telemetry_mode_loc; label = "Locations"; }
			else if (_selected == 2) { field = &prefs->telemetry_mode_env; label = "Env"; }
			if (field) {
				*field = (*field + 1) % kItemCount;
				char notice[32];
				snprintf(notice, sizeof(notice), "%s: %s", label, kStates[*field]);
				_task->showAlert(notice, 1000);
			}
		}
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) { _task->gotoSystemScreen(); return true; }
	return false;
}
