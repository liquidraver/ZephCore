/*
 * ZephCore - Joystick UI Screen Helpers
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * Static helper functions shared by all joystick UI screens.
 * Include this file once per .cpp file that needs it.
 */

#pragma once

#include "../joystick_display.h"
#include "../joystick_defs.h"
#include <helpers/ContactInfo.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* ===== Header separator line Y ===== */
/* (kHeaderSepY and kContentY from JoystickDefs.h) */

/* ===== List navigation helper ===== */
static inline bool handleCommonListNavigation(char key, int &selected, int itemCount)
{
	if (itemCount <= 0) return false;
	if (key == KEY_TO_TOP) { selected = 0; return true; }
	if (key == KEY_TO_BOTTOM) { selected = itemCount - 1; return true; }
	if (key == KEY_UP) { selected = (selected + itemCount - 1) % itemCount; return true; }
	if (key == KEY_DOWN) { selected = (selected + 1) % itemCount; return true; }
	return false;
}

/* ===== computeListStart ===== */
static inline int computeListStart(int selected_index, int item_count,
		int visible_count = UI_RECENT_LIST_SIZE)
{
	int start = selected_index - visible_count / 2;
	if (start < 0) start = 0;
	if (start + visible_count > item_count) start = item_count - visible_count;
	if (start < 0) start = 0;
	return start;
}

/* ===== renderScreenHeader ===== */
static inline void renderScreenHeader(JoystickDisplay &display, const char *title,
		int activeIndex, int totalItems)
{
	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);
	display.drawTextLeftAlign(0, 0, title);
	if (totalItems > 0) {
		char count[24];
		snprintf(count, sizeof(count), "%d/%d", activeIndex + 1, totalItems);
		display.drawTextRightAlign(display.width() - 1, 0, count);
	}
	display.drawRect(0, kHeaderSepY, display.width(), 1);
}

/* ===== renderScrollingScreenHeader ===== */
static inline void renderScrollingScreenHeader(JoystickDisplay &display, const char *title,
		int activeIndex, int totalItems,
		uint32_t marquee_base_ms)
{
	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);

	int title_max_w = display.width();
	if (totalItems > 0) {
		char count[24];
		snprintf(count, sizeof(count), "%d/%d", activeIndex + 1, totalItems);
		display.drawTextRightAlign(display.width() - 1, 0, count);
		title_max_w = display.width() - display.getTextWidth(count) - 4;
	}

	if (title_max_w > 0 && display.getTextWidth(title) > title_max_w) {
		int text_len = (int)strlen(title);
		if (text_len > 0) {
			static const uint32_t kPauseMs = 1500;
			uint32_t elapsed = k_uptime_get_32() - marquee_base_ms;
			uint32_t scroll_elapsed = (elapsed > kPauseMs) ? (elapsed - kPauseMs) : 0;
			int offset = (int)((scroll_elapsed / 450U) % (uint32_t)text_len);
			display.drawTextEllipsized(0, 0, title_max_w, title + offset);
		}
	} else {
		display.drawTextEllipsized(0, 0, title_max_w, title);
	}
	display.drawRect(0, kHeaderSepY, display.width(), 1);
}

/* ===== renderMenuListText ===== */
static inline void renderMenuListText(JoystickDisplay &display, int y, bool is_selected,
		const char *text, bool enable_marquee = false,
		uint32_t marquee_step_ms = 450,
		uint32_t marquee_base_ms = 0)
{
	const char *safe = text ? text : "";
	const int text_x = is_selected ? 10 : 0;
	const int max_w = display.width() - text_x;

	if (is_selected) {
		display.setColor(JoystickDisplay::YELLOW);
		display.drawTextLeftAlign(0, y, "> ");
	} else {
		display.setColor(JoystickDisplay::GREEN);
	}

	if (enable_marquee && is_selected && max_w > 0 && display.getTextWidth(safe) > max_w) {
		int tlen = (int)strlen(safe);
		if (tlen > 0) {
			const uint32_t step = marquee_step_ms ? marquee_step_ms : 1;
			uint32_t elapsed = k_uptime_get_32() - marquee_base_ms;
			static const uint32_t kPauseMs = 1500;
			uint32_t scroll_elapsed = elapsed > kPauseMs ? elapsed - kPauseMs : 0;
			int offset = (int)((scroll_elapsed / step) % (uint32_t)tlen);
			display.drawTextEllipsized(text_x, y, max_w, safe + offset);
			return;
		}
	}
	display.drawTextEllipsized(text_x, y, max_w, safe);
}

/* ===== sanitizeForDisplay ===== */
static inline void sanitizeForDisplay(const char *src, char *dst, size_t dst_size)
{
	size_t si = 0, di = 0;
	while (src[si] && di < dst_size - 1) {
		unsigned char c = (unsigned char)src[si];
		if (c == '\n') {
			dst[di++] = ' ';
		} else if (c >= 0x20 || c == '\t') {
			dst[di++] = (char)c;
		}
		si++;
	}
	dst[di] = '\0';
}

/* ===== formatAge helper ===== */
static inline void formatAge(uint32_t age_s, char *buf, size_t buf_size)
{
	if (age_s < 60) {
		snprintf(buf, buf_size, "%u sec", age_s);
	} else if (age_s < 3600) {
		snprintf(buf, buf_size, "%u min", age_s / 60);
	} else if (age_s < 86400) {
		snprintf(buf, buf_size, "%u hr", age_s / 3600);
	} else {
		snprintf(buf, buf_size, "%u day", age_s / 86400);
	}
}

/* ===== Message view constants ===== */
static const int kWrapRowSize = 28; /* chars per wrapped line (incl NUL) */
static const int kDetailMaxLines = 40;
static const int kDetailCPL = 20; /* chars per line in detail wrap */
/* kLineH is defined in JoystickDefs.h */

/* ===== formatHopCount ===== */
static inline void formatHopCount(uint8_t path_len, char *out, size_t out_len)
{
	if (out_len == 0) return;
	if (path_len == OUT_PATH_UNKNOWN) {
		snprintf(out, out_len, "flood");
		return;
	}
	if (path_len == OUT_PATH_SENT) {
		snprintf(out, out_len, "sent");
		return;
	}
	if (path_len == OUT_PATH_SENT_HEARD) {
		snprintf(out, out_len, "sent+");
		return;
	}
	if (path_len == OUT_PATH_SENT_UNHEARD) {
		snprintf(out, out_len, "sent?");
		return;
	}
	int hops = (int)(path_len & 63);
	if (hops <= 0) snprintf(out, out_len, "direct");
	else snprintf(out, out_len, "%dh", hops);
}

/* ===== formatClockHM ===== */
static inline void formatClockHM(uint32_t timestamp, char *out, size_t out_len)
{
	if (out_len == 0) return;
	int hh = (int)((timestamp / 3600UL) % 24UL);
	int mm = (int)((timestamp / 60UL) % 60UL);
	snprintf(out, out_len, "%02dh%02d", hh, mm);
}

/* ===== buildWrappedLines ===== */
static inline int buildWrappedLines(const char *src, int chars_per_line,
		char lines[][kWrapRowSize], int max_lines)
{
	if (!src || chars_per_line <= 1 || max_lines <= 0) return 0;
	int total = 0, i = 0, n = (int)strlen(src);
	while (i < n && total < max_lines) {
		while (i < n && src[i] == ' ') i++;
		if (i >= n) break;
		int line_start = i, line_end = i + chars_per_line;
		if (line_end >= n) { line_end = n; }
		else {
			int bp = line_end;
			while (bp > line_start && src[bp] != ' ') bp--;
			if (bp > line_start) line_end = bp;
		}
		int len = line_end - line_start;
		if (len > kWrapRowSize - 1) len = kWrapRowSize - 1;
		memcpy(lines[total], src + line_start, len);
		lines[total][len] = '\0';
		total++;
		i = line_end;
	}
	return total;
}

/* ===== parseMessageOriginAndPath ===== */
static inline void parseMessageOriginAndPath(const char *origin,
		char *sender_out, size_t sender_out_len,
		uint8_t *path_len_out)
{
	if (sender_out_len == 0) return;
	sender_out[0] = '\0';
	if (path_len_out) *path_len_out = OUT_PATH_UNKNOWN;
	if (!origin || !origin[0]) return;

	if (origin[1] == 'D') {
		sscanf(origin, "(D) %31[^:]", sender_out);
		return;
	}
	if (origin[1] == '>') {
		sscanf(origin, "(>>) %31[^:]", sender_out);
		if (path_len_out) *path_len_out = OUT_PATH_SENT;
		return;
	}
	int hops = 0;
	if (sscanf(origin, "(%d) %31[^:]", &hops, sender_out) >= 1) {
		if (path_len_out) *path_len_out = (hops <= 0) ? 0 : (uint8_t)hops;
	}
}

/* ===== buildMessageRouteLineForName ===== */
static inline void buildMessageRouteLineForName(const char *origin_label,
		const char * /*contact_name*/,
		uint8_t path_len,
		char *out, size_t out_len)
{
	if (out_len == 0) return;
	char hop_text[24];
	formatHopCount(path_len, hop_text, sizeof(hop_text));
	if (origin_label && origin_label[0]) {
		strncpy(out, origin_label, out_len - 1);
		out[out_len - 1] = '\0';
		size_t used = strlen(out);
		if (used + 1 < out_len) {
			out[used] = ' ';
			strncpy(out + used + 1, hop_text, out_len - used - 2);
			out[out_len - 1] = '\0';
		}
	} else {
		strncpy(out, hop_text, out_len - 1);
		out[out_len - 1] = '\0';
	}
}

/* ===== handleDetailScrollNavigation ===== */
static inline bool handleDetailScrollNavigation(char key, int &scroll, int max_scroll)
{
	if (key == KEY_UP) { if (scroll > 0) scroll--; return true; }
	if (key == KEY_DOWN) { if (scroll < max_scroll) scroll++; return true; }
	if (key == KEY_TO_TOP) { scroll = 0; return true; }
	if (key == KEY_TO_BOTTOM) { scroll = max_scroll; return true; }
	return false;
}

/* ===== getMessageDetailMaxScroll ===== */
static inline int getMessageDetailMaxScroll(const char *msg, int chars_per_line,
		int visible_lines, int prefix_lines = 0)
{
	if (!msg) return 0;
	char wrapped[kDetailMaxLines][kWrapRowSize];
	int total_lines = prefix_lines + buildWrappedLines(msg, chars_per_line, wrapped, kDetailMaxLines);
	return (total_lines > visible_lines) ? (total_lines - visible_lines) : 0;
}

static inline int getMessageDetailMaxScrollSanitized(JoystickDisplay &display,
		const char *msg, int chars_per_line,
		int visible_lines, int prefix_lines = 0)
{
	(void)display;
	char safe[400];
	sanitizeForDisplay(msg, safe, sizeof(safe));
	return getMessageDetailMaxScroll(safe, chars_per_line, visible_lines, prefix_lines);
}

/* ===== renderMessageDetailView ===== */
static inline int renderMessageDetailView(JoystickDisplay &display,
		int active_index, int total_items,
		uint32_t timestamp,
		const char *source_line, const char *route_line,
		const char *message, int &scroll)
{
	renderScreenHeader(display, "Msg detail", active_index, total_items);
	display.setTextSize(1);

	char lines[kDetailMaxLines][kWrapRowSize];
	int total_lines = 0;

	char time_buf[12];
	formatClockHM(timestamp, time_buf, sizeof(time_buf));
	snprintf(lines[total_lines++], kWrapRowSize, "Time: %s", time_buf);

	if (source_line && source_line[0] && total_lines < kDetailMaxLines) {
		strncpy(lines[total_lines], source_line, kWrapRowSize - 1);
		lines[total_lines++][kWrapRowSize - 1] = '\0';
	}
	if (route_line && route_line[0] && total_lines < kDetailMaxLines) {
		strncpy(lines[total_lines], route_line, kWrapRowSize - 1);
		lines[total_lines++][kWrapRowSize - 1] = '\0';
	}
	if (total_lines < kDetailMaxLines)
		snprintf(lines[total_lines++], kWrapRowSize, "---");
	if (message && message[0] && total_lines < kDetailMaxLines)
		total_lines += buildWrappedLines(message, kDetailCPL, &lines[total_lines], kDetailMaxLines - total_lines);

	int visible = (display.height() - kContentY) / kLineH;
	if (visible < 2) visible = 2;
	if (visible > 6) visible = 6;
	int max_scroll = (total_lines > visible) ? (total_lines - visible) : 0;
	if (scroll < 0) scroll = 0;
	if (scroll > max_scroll) scroll = max_scroll;

	int y = kContentY;
	for (int i = scroll; i < total_lines && i < scroll + visible; i++) {
		display.setColor(i < 3 ? JoystickDisplay::GREEN : JoystickDisplay::LIGHT);
		display.drawTextEllipsized(0, y, display.width(), lines[i]);
		y += kLineH;
	}
	return 300;
}

/* ===== getMessagePreviewVisibleCount ===== */
static inline int getMessagePreviewVisibleCount(JoystickDisplay &display,
		int row_h, int lines_per_msg, int header_h = 14)
{
	int available_h = display.height() - header_h - 2;
	int entry_h = row_h * (1 + lines_per_msg);
	if (entry_h <= 0) return 1;
	int count = available_h / entry_h;
	return (count < 1) ? 1 : count;
}

/* ===== getCenteredMessagePreviewStart ===== */
static inline int getCenteredMessagePreviewStart(int selected, int total, int visible_count)
{
	if (visible_count < 1) visible_count = 1;
	int start = selected - (visible_count / 2);
	if (start < 0) start = 0;
	if (start + visible_count > total) start = total - visible_count;
	if (start < 0) start = 0;
	return start;
}

/* ===== renderMessagePreviewEntry ===== */
static inline int renderMessagePreviewEntry(JoystickDisplay &display, int y, bool selected,
		const char *header, const char *message,
		int row_h, int lines_per_msg, int chars_per_line,
		int selected_body_x, int unselected_body_x)
{
	char wrapped[6][kWrapRowSize];
	int line_count = buildWrappedLines(message, chars_per_line, wrapped, lines_per_msg);
	if (selected) {
		display.setColor(JoystickDisplay::YELLOW);
		display.drawTextLeftAlign(0, y, "> ");
		display.drawTextEllipsized(10, y, display.width() - 10, header);
		y += row_h;
		display.setColor(JoystickDisplay::GREEN);
		for (int j = 0; j < line_count; j++) { display.drawTextLeftAlign(selected_body_x, y, wrapped[j]); y += row_h; }
	} else {
		display.setColor(JoystickDisplay::GREEN);
		display.drawTextEllipsized(0, y, display.width(), header);
		y += row_h;
		display.setColor(JoystickDisplay::LIGHT);
		for (int j = 0; j < line_count; j++) { display.drawTextLeftAlign(unselected_body_x, y, wrapped[j]); y += row_h; }
	}
	return y;
}

/* ===== GPS coordinate math helpers ===== */
static const float kScreenGpsDeg2Rad = 3.14159265f / 180.0f;

[[maybe_unused]] static inline float gpsDistanceM(int32_t lat1_e6, int32_t lon1_e6,
		int32_t lat2_e6, int32_t lon2_e6)
{
	float la1 = lat1_e6 * 1e-6f * kScreenGpsDeg2Rad;
	float la2 = lat2_e6 * 1e-6f * kScreenGpsDeg2Rad;
	float dlat = (lat2_e6 - lat1_e6) * 1e-6f * kScreenGpsDeg2Rad;
	float dlon = (lon2_e6 - lon1_e6) * 1e-6f * kScreenGpsDeg2Rad;
	float a = sinf(dlat / 2) * sinf(dlat / 2) +
			  cosf(la1) * cosf(la2) * sinf(dlon / 2) * sinf(dlon / 2);
	return 6371000.0f * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

[[maybe_unused]] static inline float gpsBearingDeg(int32_t lat1_e6, int32_t lon1_e6,
		int32_t lat2_e6, int32_t lon2_e6)
{
	float la1 = lat1_e6 * 1e-6f * kScreenGpsDeg2Rad;
	float la2 = lat2_e6 * 1e-6f * kScreenGpsDeg2Rad;
	float dlon = (lon2_e6 - lon1_e6) * 1e-6f * kScreenGpsDeg2Rad;
	float y = sinf(dlon) * cosf(la2);
	float x = cosf(la1) * sinf(la2) - sinf(la1) * cosf(la2) * cosf(dlon);
	float b = atan2f(y, x) / kScreenGpsDeg2Rad;
	return fmodf(b + 360.0f, 360.0f);
}

[[maybe_unused]] static inline const char *compassDir(float bearing)
{
	int idx = (int)((bearing + 22.5f) / 45.0f) % 8;
	static const char * const dirs[8] = { "N","NE","E","SE","S","SW","W","NW" };
	return dirs[idx];
}

/* ===== T9 keypad helpers ===== */
static const unsigned long T9_MULTI_TAP_MS = 1100;

static const char * const T9_KEY_LETTERS_LETTER_MODE[16] = {
	"abc", "def", "ghi", "",
	"jkl", "mno", "pqr", "",
	"stu", "vwx", "yz-", "",
	"",    ".,'", ":!?", ""
};
static const char * const T9_KEY_LETTERS_NUMBER_MODE[16] = {
	"12",  "34",  "56",  "",
	"78",  "90",  "@$",  "",
	"/\\", "+=",  "()",  "",
	"",    "_#",  "^%",  ""
};

static inline const char * const *getT9KeyLetters(bool letter_mode)
{
	return letter_mode ? T9_KEY_LETTERS_LETTER_MODE : T9_KEY_LETTERS_NUMBER_MODE;
}

static const char * const T9_KEY_LABELS_LETTER_SPECIAL[16] = {
	nullptr, nullptr, nullptr, "del",
	nullptr, nullptr, nullptr, "spc",
	nullptr, nullptr, nullptr, "send",
	"123",   nullptr, nullptr, " "
};
static const char * const T9_KEY_LABELS_NUMBER_SPECIAL[16] = {
	nullptr, nullptr, nullptr, "del",
	nullptr, nullptr, nullptr, "spc",
	nullptr, nullptr, nullptr, "send",
	"abc",   nullptr, nullptr, nullptr
};

[[maybe_unused]] static const char * const *getT9KeyLabels(bool letter_mode, bool save_action)
{
	static char label_storage[16][8];
	static const char *label_ptrs[16];

	const char * const *overlay = letter_mode ? T9_KEY_LABELS_LETTER_SPECIAL : T9_KEY_LABELS_NUMBER_SPECIAL;
	const char * const *letters = getT9KeyLetters(letter_mode);

	for (int i = 0; i < 16; i++) {
		if (overlay[i]) {
			label_ptrs[i] = overlay[i];
		} else {
			size_t n = strlen(letters[i]);
			if (n >= sizeof(label_storage[i])) n = sizeof(label_storage[i]) - 1;
			memcpy(label_storage[i], letters[i], n);
			label_storage[i][n] = '\0';
			label_ptrs[i] = label_storage[i];
		}
	}
	if (save_action) label_ptrs[11] = "save";
	return label_ptrs;
}

static inline void resetT9State(int &last_key, int &letter_index, uint32_t &last_press_time)
{
	last_key = -1;
	letter_index = 0;
	last_press_time = 0;
}

static inline void appendOrCycleT9Char(char *buf, int &cursor, size_t capacity,
		const char *letters, int key,
		int &last_key, int &letter_index,
		uint32_t &last_press_time)
{
	size_t count = strlen(letters);
	if (count == 0 || capacity < 2) return;

	uint32_t now = k_uptime_get_32();
	bool cycle = cursor > 0 && last_key == key && (now - last_press_time) <= T9_MULTI_TAP_MS;
	if (cycle) {
		letter_index = (letter_index + 1) % (int)count;
		buf[cursor - 1] = letters[letter_index];
	} else if (cursor < (int)capacity - 1) {
		letter_index = 0;
		buf[cursor++] = letters[0];
		buf[cursor] = '\0';
	}
	last_key = key;
	last_press_time = now;
}

static inline bool handleT9DirectionalInput(char c, int &selected_key)
{
	if (c == KEY_UP) { selected_key = (selected_key + 12) % 16; return true; }
	if (c == KEY_DOWN) { selected_key = (selected_key + 4) % 16; return true; }
	if (c == KEY_LEFT) { selected_key = (selected_key / 4) * 4 + ((selected_key + 3) % 4); return true; }
	if (c == KEY_RIGHT) { selected_key = (selected_key / 4) * 4 + ((selected_key + 1) % 4); return true; }
	return false;
}

static inline void renderT9Keypad(JoystickDisplay &display, const char * const *labels,
		int selected_key, int y_start, int key_height = 10)
{
	int key_width = display.width() / 4;
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			int key = row * 4 + col;
			int x = col * key_width;
			int y = y_start + row * key_height;
			if (key == selected_key) {
				display.setColor(JoystickDisplay::LIGHT);
				display.fillRect(x, y, key_width, key_height);
				display.setColor(JoystickDisplay::DARK);
			} else {
				display.setColor(JoystickDisplay::LIGHT);
			}
			display.drawTextCentered(x + key_width / 2, y + key_height / 2, labels[key]);
		}
	}
}

/* ===== buildChannelReplyPrefix ===== */
/* Builds "@[SenderName] " from a "SenderName: text" channel message.
 * Returns false for outbound messages (any OUT_PATH_SENT* marker) or if
 * no ": " separator is found. out_buf must be at least (name_len + 5)
 * bytes; 38 is always safe. */
static inline bool buildChannelReplyPrefix(
	const char *msg, uint8_t path_len,
	char *out_buf, size_t out_size
){
	if (path_len == OUT_PATH_SENT ||
		path_len == OUT_PATH_SENT_HEARD ||
		path_len == OUT_PATH_SENT_UNHEARD) return false;
	const char *sep = strstr(msg, ": ");
	if (!sep || sep == msg) return false;
	size_t nlen = (size_t)(sep - msg);
	if (nlen > 32) nlen = 32;
	if (out_size < nlen + 5) return false; /* '@' '[' name ']' ' ' '\0' */
	out_buf[0] = '@';
	out_buf[1] = '[';
	memcpy(out_buf + 2, msg, nlen);
	out_buf[2 + nlen] = ']';
	out_buf[3 + nlen] = ' ';
	out_buf[4 + nlen] = '\0';
	return true;
}

/* Helper: leave compose input and return to appropriate screen */
static inline void leaveComposeInput(class JoystickUITask *task);

#include "../joystick_ui_task.h"
static inline void leaveComposeInput(JoystickUITask *task)
{
	if (task->isComposeContact()) {
		task->gotoContactsScreen();
	} else if (task->getComposeChannelIndex() != -1) {
		task->gotoChannelsScreen();
	} else {
		task->gotoHomeScreen();
	}
}
