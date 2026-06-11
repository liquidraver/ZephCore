/*
 * ZephCore - Joystick Display Adapter
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 *
 * JoystickDisplay: DisplayDriver-compatible adapter over mc_display_*.
 *
 * Provides a C++ API compatible with the old Arduino DisplayDriver class,
 * implemented on top of ZephCore's mc_display_* (Zephyr CFB) primitives.
 *
 * Font is fixed-width (6x8 by default, actual size queried at runtime).
 * Colors map to monochrome:
 *   GREEN / LIGHT / WHITE → normal (white text on black)
 *   YELLOW → selected highlight (fill_rect + inverted text)
 *   DARK → inverted text (black text on white background)
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "display.h"
#ifdef __cplusplus
}
#endif

class JoystickDisplay {
public:
	enum Color { GREEN = 0, YELLOW = 1, DARK = 2, LIGHT = 3, WHITE = 4, NONE = 5 };

	JoystickDisplay() : _color(GREEN), _cx(0), _cy(0) {}

	/* ===== Color / size ===== */
	void setColor(Color c) { _color = c; }
	void setTextSize(int) { /* only one font size supported */ }
	Color getColor() const { return _color; }

	/* ===== Metrics ===== */
	int width() const { return (int)mc_display_width(); }
	int height() const { return (int)mc_display_height(); }
	int fontW() const { int w = mc_display_font_width(); return w ? w : 6; }
	int fontH() const { int h = mc_display_font_height(); return h ? h : 8; }
	int getTextWidth(const char *text) const {
		if (!text) return 0;
		char buf[SANITIZE_BUF];
		return (int)strlen(sanitized(buf, sizeof(buf), text)) * fontW();
	}

	/* ===== Text drawing =====
	 * All entry points transcode UTF-8 to the display charset first
	 * (utf8_to_display: Latin-1/Latin-2 mapped, emoji stripped), so screens
	 * can pass contact/channel names straight through. Measurement uses the
	 * same transcoding, keeping widths in glyphs rather than UTF-8 bytes. */
	void drawTextLeftAlign(int x, int y, const char *text) {
		if (!text) return;
		char buf[SANITIZE_BUF];
		drawSanitized(x, y, sanitized(buf, sizeof(buf), text));
	}

	void drawTextRightAlign(int right_x, int y, const char *text) {
		if (!text) return;
		int x = right_x - getTextWidth(text);
		if (x < 0) x = 0;
		drawTextLeftAlign(x, y, text);
	}

	void drawTextCentered(int cx, int cy, const char *text) {
		if (!text) return;
		int x = cx - getTextWidth(text) / 2;
		int y = cy - fontH() / 2;
		if (x < 0) x = 0;
		if (y < 0) y = 0;
		drawTextLeftAlign(x, y, text);
	}

	/* Draw text at (x, y) truncating with "..." if wider than max_w pixels */
	void drawTextEllipsized(int x, int y, int max_w, const char *text) {
		if (!text || max_w <= 0) return;
		int fw = fontW();
		int avail_chars = max_w / fw;
		if (avail_chars <= 0) return;

		char sbuf[SANITIZE_BUF];
		const char *s = sanitized(sbuf, sizeof(sbuf), text);
		int len = (int)strlen(s);
		if (len <= avail_chars) {
			drawSanitized(x, y, s);
		} else {
			/* Truncate with "…" (3 dots, but we only have ASCII) */
			int truncate_at = avail_chars - 3;
			if (truncate_at < 0) truncate_at = 0;
			char buf[128];
			int n = truncate_at < 127 ? truncate_at : 127;
			memcpy(buf, s, n);
			buf[n] = '.'; buf[n+1] = '.'; buf[n+2] = '.';
			buf[n+3] = '\0';
			drawSanitized(x, y, buf);
		}
	}

	/* Cursor based printing (for compatibility with print() pattern) */
	void setCursor(int x, int y) { _cx = x; _cy = y; }
	void print(const char *text) {
		if (!text) return;
		drawTextLeftAlign(_cx, _cy, text);
		_cx += getTextWidth(text);
	}
	void print(int n) {
		char buf[20];
		snprintf(buf, sizeof(buf), "%d", n);
		print(buf);
	}

	/* ===== Shapes ===== */
	void fillRect(int x, int y, int w, int h) {
		if (_color == DARK) return; /* DARK fill is equivalent to clearing, already black */
		mc_display_fill_rect(x, y, w, h);
	}

	/* drawRect: outline rect, or horizontal line when h equals 1 */
	void drawRect(int x, int y, int w, int h) {
		if (h <= 1) {
			mc_display_hline(x, y, w);
		} else {
			mc_display_hline(x, y, w); /* top */
			mc_display_hline(x, y + h - 1, w); /* bottom */
			mc_display_fill_rect(x, y, 1, h); /* left */
			mc_display_fill_rect(x + w - 1, y, 1, h); /* right */
		}
	}

	/* ===== Frame management ===== */
	void startFrame() {
		mc_display_clear();
	}
	void endFrame() {
		mc_display_finalize();
	}

	/* ===== Display power ===== */
	bool isOn() { return mc_display_is_on(); }
	void turnOn() { mc_display_on(); }
	void turnOff() { mc_display_off(); }

	/* UTF8 utility — explicit pre-transcoding for screens that slice text
	 * afterwards (marquee windows etc.), so byte == glyph in their math.
	 * Re-sanitizing the result at draw time is a harmless pass-through. */
	void translateUTF8ToBlocks(char *dst, const char *src, int size) {
		utf8_to_latin1(dst, src, (size_t)size);
	}

private:
	static constexpr size_t SANITIZE_BUF = 160;	/* > any display line */

	/* Returns text itself when pure ASCII (no copy), else the transcoded
	 * copy in buf. */
	static const char *sanitized(char *buf, size_t n, const char *text) {
		for (const char *p = text; *p; p++) {
			if ((uint8_t)*p >= 0x80) {
				utf8_to_display(buf, text, n);
				return buf;
			}
		}
		return text;
	}

	/* Color/invert draw logic on already-transcoded text. */
	void drawSanitized(int x, int y, const char *text) {
		if (_color == YELLOW) {
			int w = (int)strlen(text) * fontW();
			mc_display_fill_rect(x, y, w, fontH());
			mc_display_text(x, y, text, true);
		} else {
			mc_display_text(x, y, text, _color == DARK);
		}
	}

	Color _color;
	int _cx, _cy;
};
