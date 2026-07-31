/*
 * ZephCore - Joystick UI Home Screen
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "../joystick_screens.h"
#include "../joystick_ui_task.h"
#include "screen_helpers.h"
#include <zephyr/kernel.h>
#include <stdio.h>
#include <ZephyrSensorManager.h>

#define HOME_ITEM_COUNT  7

HomeScreen::HomeScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0)
{
}

int HomeScreen::render(JoystickDisplay &display)
{
	static char unread_item[24];
	const char *items[HOME_ITEM_COUNT];
	int itemCount = 0;

	snprintf(unread_item, sizeof(unread_item), "Unread (%d)", _task->getUnreadCount());
	items[itemCount++] = unread_item;
	items[itemCount++] = "Channels";
	items[itemCount++] = "Contacts";
	items[itemCount++] = "Advert";
	items[itemCount++] = _task->isGPSAvailable() ? "GPS" : "System";
	items[itemCount++] = _task->isGPSAvailable() ? "System" : "Tools";
	items[itemCount++] = _task->isGPSAvailable() ? "Tools" : nullptr;
	if (!_task->isGPSAvailable()) itemCount--;

	if (_selected >= itemCount) _selected = itemCount - 1;

	display.setTextSize(1);

	/* Top bar: node name (left) + battery + icons (right) */
	char batt[16];
	uint16_t batt_mv = _task->getCachedBattMilliVolts();
	if (_task->getBatteryDisplayMode() == 1) {
		snprintf(batt, sizeof(batt), "%.1fV", (double)batt_mv / 1000.0);
	} else {
		int pct = ((int)batt_mv - kBattMinMv) * 100 / (kBattMaxMv - kBattMinMv);
		if (pct < 0) pct = 0;
		if (pct > 100) pct = 100;
		snprintf(batt, sizeof(batt), "%d%%", pct);
	}

	char icons[6] = {0};
	int ni = 0;
	if (_task->isSerialEnabled()) icons[ni++] = 'B';
	if (_task->getGPSState()) icons[ni++] = 'G';
	if (!_task->isBuzzerQuiet()) icons[ni++] = '~';
	icons[ni] = '\0';

	int batt_w = display.getTextWidth(batt);
	int icons_w = (ni > 0) ? (display.getTextWidth(icons) + 2) : 0;
	int right_w = batt_w + icons_w;

	display.setColor(JoystickDisplay::GREEN);
	display.drawTextRightAlign(display.width(), 0, batt);
	if (ni > 0) {
		display.setColor(JoystickDisplay::YELLOW);
		display.drawTextRightAlign(display.width() - batt_w - 2, 0, icons);
	}
	char nodeName[32];
	display.translateUTF8ToBlocks(nodeName, _task->getPrefs() ? _task->getPrefs()->node_name : "Node", sizeof(nodeName));
	display.setColor(JoystickDisplay::GREEN);
	display.drawTextEllipsized(0, 0, display.width() - right_w - 4, nodeName);
	display.drawRect(0, kHeaderSepY, display.width(), 1);

	/* Grid layout */
	const int top_y = kContentY;
	const int rows = 1 + ((itemCount - 1 + 1) / 2); /* 1 full width + 2 col rows */
	int row_h = (display.height() - top_y) / rows;
	if (row_h < 10) row_h = 10;
	const int col_w = display.width() / 2;
	const int text_off = (row_h - display.fontH()) / 2;

	/* Grid separators */
	display.setColor(JoystickDisplay::LIGHT);
	for (int r = 1; r <= rows; r++) {
		int y = top_y + r * row_h;
		if (y <= display.height() - 1) display.drawRect(0, y, display.width(), 1);
	}
	for (int r = 1; r < rows; r++) {
		int y = top_y + r * row_h;
		display.drawRect(col_w, y, 1, row_h);
	}

	/* Row 0: full width Unread (N) */
	{
		int y0 = top_y;
		char unread_count[12];
		snprintf(unread_count, sizeof(unread_count), "%d", _task->getUnreadCount());
		if (_selected == 0) {
			display.setColor(JoystickDisplay::YELLOW);
			display.drawTextLeftAlign(2, y0 + text_off, "> Unread");
			display.drawTextRightAlign(display.width() - 3, y0 + text_off, unread_count);
		} else {
			display.setColor(JoystickDisplay::GREEN);
			display.drawTextLeftAlign(2, y0 + text_off, "Unread");
			display.drawTextRightAlign(display.width() - 3, y0 + text_off, unread_count);
		}
	}

	/* Remaining items: 2 column cells */
	for (int i = 1; i < itemCount; i++) {
		if (!items[i]) continue;
		int rr = 1 + ((i - 1) / 2);
		int cc = (i - 1) % 2;
		int x = cc * col_w;
		int y = top_y + rr * row_h;
		if (i == _selected) {
			display.setColor(JoystickDisplay::YELLOW);
			char sel_buf[48];
			snprintf(sel_buf, sizeof(sel_buf), "> %s", items[i]);
			display.drawTextEllipsized(x + 2, y + text_off, col_w - 4, sel_buf);
		} else {
			display.setColor(JoystickDisplay::GREEN);
			display.drawTextEllipsized(x + 2, y + text_off, col_w - 4, items[i]);
		}
	}

	return 500;
}

bool HomeScreen::handleInput(char c)
{
	int itemCount = _task->isGPSAvailable() ? 7 : 6;

	if (_selected >= itemCount) _selected = itemCount - 1;

	if (c == KEY_TO_TOP) { _selected = 0; return true; }
	if (c == KEY_TO_BOTTOM) { _selected = itemCount - 1; return true; }

	if (c == KEY_UP) {
		if (_selected == 0) {
			_selected = itemCount - 1;
		} else {
			int rr = 1 + ((_selected - 1) / 2);
			int cc = (_selected - 1) % 2;
			rr--;
			if (rr == 0) {
				_selected = 0;
			} else {
				int next = 1 + (rr - 1) * 2 + cc;
				if (next >= itemCount) next = itemCount - 1;
				_selected = next;
			}
		}
		return true;
	}
	if (c == KEY_DOWN) {
		if (_selected == 0) {
			if (itemCount > 1) _selected = 1;
		} else {
			int rr = 1 + ((_selected - 1) / 2);
			int cc = (_selected - 1) % 2;
			rr++;
			int next = 1 + (rr - 1) * 2 + cc;
			if (next >= itemCount) { _selected = 0; } else { _selected = next; }
		}
		return true;
	}
	if (c == KEY_LEFT) {
		if (_selected > 1 && ((_selected - 1) % 2) == 1) _selected--;
		return true;
	}
	if (c == KEY_RIGHT) {
		if (_selected > 0 && ((_selected - 1) % 2) == 0) {
			int next = _selected + 1;
			if (next < itemCount) _selected = next;
		}
		return true;
	}
	if (c == KEY_ENTER || c == KEY_NEXT) {
		/* Map selected index to screen */
		/* Indices: 0=Unread, 1=Channels, 2=Contacts, 3=Advert, 4=GPS(if avail)|System, 5=System|Tools, 6=Tools */
		switch (_selected) {
		case 0: _task->gotoUnreadScreen();   return true;
		case 1: _task->gotoChannelsScreen(); return true;
		case 2: _task->gotoContactsScreen(); return true;
		case 3: _task->gotoAdvertScreen();   return true;
		case 4:
			if (_task->isGPSAvailable()) _task->gotoGPSScreen();
			else _task->gotoSystemScreen();
			return true;
		case 5:
			if (_task->isGPSAvailable()) _task->gotoSystemScreen();
			else _task->gotoToolsScreen();
			return true;
		case 6: _task->gotoToolsScreen(); return true;
		default: break;
		}
	}
	return false;
}

#ifndef BOOT_SCREEN_MILLIS
#define BOOT_SCREEN_MILLIS  3000
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "ZephCore"
#endif

#ifndef FIRMWARE_BUILD_DATE
#define FIRMWARE_BUILD_DATE ""
#endif

SplashScreen::SplashScreen(JoystickUITask *task)
	: _task(task), _dismiss_after(0)
{
	k_timer_init(&_dismiss_timer, dismissTimerCb, NULL);
	k_timer_user_data_set(&_dismiss_timer, this);
}

void SplashScreen::dismissTimerCb(struct k_timer *t)
{
	auto *self = static_cast<SplashScreen *>(k_timer_user_data_get(t));
	if (self && self->_task) self->_task->notify();
}

void SplashScreen::onEnter()
{
	_dismiss_after = k_uptime_get_32() + BOOT_SCREEN_MILLIS;
	k_timer_start(&_dismiss_timer, K_MSEC(BOOT_SCREEN_MILLIS), K_NO_WAIT);
}

void SplashScreen::onExit()
{
	k_timer_stop(&_dismiss_timer);
}

int SplashScreen::render(JoystickDisplay &display)
{
	if (k_uptime_get_32() >= _dismiss_after) {
		_task->gotoHomeScreen();
		return 0;
	}

	/* Match the button-UI splash: ZephCore wordmark bitmap centered at top,
	 * "MeshCore on Zephyr" beneath it, build date at the bottom. Logo and
	 * its dimensions live in helpers/ui/ui_common.c — declared in display.h. */
	int w = display.width();
	int fh = display.fontH();
	int line_h = fh + 2;
	int logo_x = (w >= ZEPHCORE_LOGO_W) ? (w - ZEPHCORE_LOGO_W) / 2 : 0;
	int y = 3;
	mc_display_xbm(logo_x, y, zephcore_logo, ZEPHCORE_LOGO_W, ZEPHCORE_LOGO_H);
	y += ZEPHCORE_LOGO_H + line_h;

	int cx = w / 2;
	display.setColor(JoystickDisplay::GREEN);
	display.drawTextCentered(cx, y + fh / 2, "MeshCore on Zephyr");
	y += line_h * 2;
	display.setColor(JoystickDisplay::LIGHT);
	display.drawTextCentered(cx, y + fh / 2, FIRMWARE_BUILD_DATE);
	return 250;
}
