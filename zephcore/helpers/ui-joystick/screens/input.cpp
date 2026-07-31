/*
 * ZephCore - Joystick UI Input Screen
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "../joystick_screens.h"
#include "../joystick_ui_task.h"
#include "screen_helpers.h"
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

T9InputScreen::T9InputScreen(JoystickUITask *task)
	: _task(task), _cursor(0), _selected_key(0),
	  _last_press_time(0), _letter_index(0), _last_key(-1),
	  _confirm_exit(false), _confirm_selected(0), _kb_mode_letters(true)
{
	memset(_input, 0, sizeof(_input));
}

int T9InputScreen::render(JoystickDisplay &display)
{
	if (_confirm_exit) {
		renderScreenHeader(display, "Draft", _confirm_selected, 2);
		display.setColor(JoystickDisplay::LIGHT);
		display.drawTextLeftAlign(0, kContentY + 2, "Keep current draft?");

		static const char * const kOptions[2] = {"keep draft", "flush draft"};
		int y = 30;
		for (int i = 0; i < 2; i++) {
			if (i == _confirm_selected) {
				display.setColor(JoystickDisplay::YELLOW);
				display.drawTextLeftAlign(0, y, "> ");
				display.drawTextLeftAlign(10, y, kOptions[i]);
			} else {
				display.setColor(JoystickDisplay::GREEN);
				display.drawTextLeftAlign(0, y, kOptions[i]);
			}
			y += kMenuLineH;
		}
		return 300;
	}

	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);
	if (_task->isComposeContact()) {
		char to_line[40];
		snprintf(to_line, sizeof(to_line), "To: %s", _task->getComposeContactName());
		display.drawTextEllipsized(0, 0, display.width(), to_line);
	} else {
		char to_line[40];
		snprintf(to_line, sizeof(to_line), "Ch: %s", _task->getComposeChannelName());
		display.drawTextEllipsized(0, 0, display.width(), to_line);
	}

	char display_text[258];
	snprintf(display_text, sizeof(display_text), "%s|", _input);
	display.drawTextLeftAlign(0, 10, display_text);

	renderT9Keypad(display, getT9KeyLabels(_kb_mode_letters, false), _selected_key, 22);
	display.setColor(JoystickDisplay::LIGHT);
	return 500;
}

bool T9InputScreen::handleInput(char c)
{
	if (_confirm_exit) {
		if (c == KEY_UP || c == KEY_DOWN) {
			_confirm_selected = (_confirm_selected + 1) % 2;
			return true;
		}
		if (c == KEY_ENTER) {
			bool flush = (_confirm_selected == 1);
			if (flush) { memset(_input, 0, sizeof(_input)); _cursor = 0; }
			resetT9State(_last_key, _letter_index, _last_press_time);
			_confirm_exit = false;
			leaveComposeInput(_task);
			return true;
		}
		if (c == KEY_CANCEL || c == KEY_HOME) { _confirm_exit = false; return true; }
		return false;
	}

	if (handleT9DirectionalInput(c, _selected_key)) return true;

	if (c == KEY_ENTER) {
		if (_selected_key == 12) {      /* mode toggle */
			_kb_mode_letters = !_kb_mode_letters;
			resetT9State(_last_key, _letter_index, _last_press_time);
			return true;
		}
		if (_selected_key == 3) {       /* delete */
			backspace();
			resetT9State(_last_key, _letter_index, _last_press_time);
			return true;
		}
		if (_selected_key == 7) {       /* space */
			addLetter(' ');
			resetT9State(_last_key, _letter_index, _last_press_time);
			return true;
		}
		if (_selected_key == 11) {      /* send */
			if (_cursor > 0) {
				if (_task->sendComposedMessage(_input))
					_task->showAlert("Message sent", 1000);
				else
					_task->showAlert("Send failed", 1000);
				memset(_input, 0, sizeof(_input));
				_cursor = 0;
			}
			resetT9State(_last_key, _letter_index, _last_press_time);
			leaveComposeInput(_task);
			return true;
		}
		if (_selected_key == 15) return false;  /* unused key */

		const char *letters = getT9KeyLetters(_kb_mode_letters)[_selected_key];
		appendOrCycleT9Char(_input, _cursor, sizeof(_input), letters, _selected_key,
							_last_key, _letter_index, _last_press_time);
		return true;
	}

	if (c == KEY_CANCEL || c == KEY_HOME) {
		if (_cursor > 0) {
			_confirm_exit = true;
			_confirm_selected = 0;
		} else {
			resetT9State(_last_key, _letter_index, _last_press_time);
			leaveComposeInput(_task);
		}
		return true;
	}
	return false;
}

void T9InputScreen::addLetter(char ch)
{
	if (_cursor < (int)sizeof(_input) - 1) {
		_input[_cursor++] = ch;
		_input[_cursor] = '\0';
	}
}

void T9InputScreen::backspace()
{
	if (_cursor > 0) {
		_cursor--;
		_input[_cursor] = '\0';
	}
}
