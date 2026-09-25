/*
 * SPDX-License-Identifier: MIT
 *
 * The joystick UI's repeater / room server admin screen: login, status,
 * neighbours, telemetry, owner info and a remote CLI.
 */

#include "../joystick_screens.h"
#include "../joystick_ui_task.h"
#include "../joystick_ui_hooks.h"
#include "screen_helpers.h"
#include <helpers/BaseChatMesh.h>
#include <mesh/Utils.h>
#include <helpers/ContactInfo.h>
#include <helpers/ui/telemetry_text.h>
#include <helpers/ui/ui_task.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

/* ===== RepeaterAdminScreen ===== */

#define ADMIN_TIMEOUT_MS  12000

static const int kAdminContentY = 23;
static const int kAdminVisible = 4;

/* ===== Constructor ===== */

RepeaterAdminScreen::RepeaterAdminScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc),
	  _state(STATE_PASSWORD_ENTRY), _from_contacts(false),
	  _permissions(0), _server_time(0),
	  _pwd_len(0),
	  _pwd_t9_sel(0), _pwd_t9_last_key(-1), _pwd_t9_letter_index(0), _pwd_t9_last_press(0),
	  _pwd_kb_letters(true),
	  _submenu_sel(0),
	  _pending(PENDING_NONE),
	  _last_sent_at(0), _cmd_timeout_ms(ADMIN_TIMEOUT_MS), _awaiting_tx(false),
	  _hist_head(0), _hist_count(0), _hist_scroll(0), _resp_line_scroll(0), _resp_max_line_scroll(0),
	  _cmd_len(0),
	  _cmd_t9_sel(0), _cmd_t9_last_key(-1), _cmd_t9_letter_index(0), _cmd_t9_last_press(0),
	  _cmd_kb_letters(true)
{
	memset(_contact_pubkey, 0, sizeof(_contact_pubkey));
	memset(_repeater_name, 0, sizeof(_repeater_name));
	memset(_password, 0, sizeof(_password));
	memset(_hist, 0, sizeof(_hist));
	memset(_cmd_buf, 0, sizeof(_cmd_buf));
	k_timer_init(&_timeout_timer, timeoutTimerCb, NULL);
	k_timer_user_data_set(&_timeout_timer, this);
}

void RepeaterAdminScreen::timeoutTimerCb(struct k_timer *t)
{
	/* ISR — wake main loop; render() detects elapsed and dispatches to onTimeout(). */
	auto *self = static_cast<RepeaterAdminScreen *>(k_timer_user_data_get(t));
	if (self && self->_task) self->_task->notify();
}

void RepeaterAdminScreen::onExit()
{
	k_timer_stop(&_timeout_timer);
}

void RepeaterAdminScreen::onTimeout()
{
	if (_state == STATE_LOGGING_IN) {
		_awaiting_tx = false;
		_state = STATE_PASSWORD_ENTRY;
		_task->showAlert("Login timeout", 1500);
	} else if ((_state == STATE_SUBMENU || _state == STATE_MAIN || _state == STATE_CMD_INPUT) &&
			   _pending != PENDING_NONE) {
		_awaiting_tx = false;
		if (_hist_count > 0) {
			CmdEntry &newest = histAt(_hist_count - 1);
			if (!newest.has_resp) {
				snprintf(newest.resp, sizeof(newest.resp), "(no response)");
				newest.has_resp = true;
			}
		}
		_pending = PENDING_NONE;
		_task->clearAdminReqTag();
	}
	_last_sent_at = 0;  /* prevent re-fire from the render() elapsed-check */
}

/* ===== openForContact ===== */

void RepeaterAdminScreen::openForContact(const uint8_t *pub_key, const char *name,
		bool from_contacts)
{
	if (pub_key) memcpy(_contact_pubkey, pub_key, PUB_KEY_SIZE);
	else memset(_contact_pubkey, 0, PUB_KEY_SIZE);
	_from_contacts = from_contacts;
	strncpy(_repeater_name, name ? name : "?", sizeof(_repeater_name) - 1);
	_repeater_name[sizeof(_repeater_name) - 1] = '\0';
	_admin_header_marquee_ms = k_uptime_get_32();

	_state = STATE_PASSWORD_ENTRY;
	_pwd_len = 0;
	memset(_password, 0, sizeof(_password));
	_pwd_t9_sel = 0; _pwd_t9_last_key = -1;
	_pwd_t9_letter_index = 0; _pwd_t9_last_press = 0;
	_pwd_kb_letters = true;

	_submenu_sel = 0;
	_pending = PENDING_NONE;
	_cmd_timeout_ms = ADMIN_TIMEOUT_MS;
	_awaiting_tx = false;
	_hist_head = 0; _hist_count = 0; _hist_scroll = 0; _resp_line_scroll = 0; _resp_max_line_scroll = 0;
	memset(_hist, 0, sizeof(_hist));
}

/* ===== sendBinaryReq — used for guest-accessible requests ===== */

static bool sendBinaryReqHelper(JoystickUITask *task, const uint8_t *contact_pubkey,
		const uint8_t *req_data, uint8_t req_len,
		uint32_t &cmd_timeout_ms, bool &awaiting_tx,
		uint32_t &last_sent_at)
{
	BaseChatMesh *mesh = task->getMesh();
	if (!mesh) return false;
	ContactInfo *contact = mesh->lookupContactByPubKey(contact_pubkey, PUB_KEY_SIZE);
	if (!contact) {
		task->showAlert("Contact lost", 1500);
		return false;
	}
	uint32_t tag, est_timeout = 0;
	int result = mesh->sendRequest(*contact, req_data, req_len, tag, est_timeout);
	if (result == MSG_SEND_FAILED) {
		task->showAlert("Send failed", 1500);
		return false;
	}
	task->registerAdminReqTag(tag);
	ui_signal_tx();
	last_sent_at = k_uptime_get_32();
	awaiting_tx = true;
	cmd_timeout_ms = ADMIN_TIMEOUT_MS;
	if (est_timeout > 0) {
		uint32_t rt = est_timeout * 2 + 3000;
		if (rt > cmd_timeout_ms) cmd_timeout_ms = rt;
	}
	return true;
}

/* ===== onReqResponse — parse binary REQ responses for guest shortcuts ===== */

void RepeaterAdminScreen::onReqResponse(const uint8_t *pub_key_prefix,
										const uint8_t *data, uint8_t data_len)
{
	if (memcmp(_contact_pubkey, pub_key_prefix, 4) != 0) return;
	if (_pending != PENDING_BINARY_STATUS && _pending != PENDING_BINARY_NEIGHBOURS &&
		_pending != PENDING_BINARY_TELEMETRY && _pending != PENDING_BINARY_OWNER_INFO) return;

	char text[ADMIN_RESP_MAX];
	text[0] = '\0';

	if (_pending == PENDING_BINARY_STATUS && data_len >= 4) {
		/* RepeaterStats struct layout (byte offsets from start of payload):
		 *  0: batt_milli_volts (u16)   2: curr_tx_queue_len (u16)
		 *  4: noise_floor (i16)        6: last_rssi (i16)
		 *  8: n_packets_recv (u32)    12: n_packets_sent (u32)
		 * 16: total_air_time_secs (u32) 20: total_up_time_secs (u32)
		 * 24: n_sent_flood (u32)      28: n_sent_direct (u32)
		 * 32: n_recv_flood (u32)      36: n_recv_direct (u32)
		 * 40: err_events (u16)        42: last_snr (i16, raw×4)
		 * 44: n_direct_dups (u16)     46: n_flood_dups (u16)
		 * 48: total_rx_air_time_secs (u32)
		 * 52: n_recv_errors (u32) */
		uint16_t batt = 0, txq = 0, errf = 0, dup_d = 0, dup_f = 0;
		int16_t noise = 0, rssi = 0, lsnr = 0;
		uint32_t air_tx = 0, rx_air = 0, uptime = 0;
		uint32_t ptx = 0, prx = 0;
		uint32_t sent = 0, recvd = 0, sent_d = 0, recvd_d = 0, errors = 0;
		if (data_len >= 2) memcpy(&batt, &data[0], 2);
		if (data_len >= 4) memcpy(&txq, &data[2], 2);
		if (data_len >= 6) memcpy(&noise, &data[4], 2);
		if (data_len >= 8) memcpy(&rssi, &data[6], 2);
		if (data_len >= 12) memcpy(&prx, &data[8], 4);
		if (data_len >= 16) memcpy(&ptx, &data[12], 4);
		if (data_len >= 20) memcpy(&air_tx, &data[16], 4);
		if (data_len >= 24) memcpy(&uptime, &data[20], 4);
		if (data_len >= 28) memcpy(&sent, &data[24], 4);
		if (data_len >= 32) memcpy(&sent_d, &data[28], 4);
		if (data_len >= 36) memcpy(&recvd, &data[32], 4);
		if (data_len >= 40) memcpy(&recvd_d, &data[36], 4);
		if (data_len >= 42) memcpy(&errf, &data[40], 2);
		if (data_len >= 44) memcpy(&lsnr, &data[42], 2);
		if (data_len >= 46) memcpy(&dup_d, &data[44], 2);
		if (data_len >= 48) memcpy(&dup_f, &data[46], 2);
		if (data_len >= 52) memcpy(&rx_air, &data[48], 4);
		if (data_len >= 56) memcpy(&errors, &data[52], 4);
		char up_buf[12], air_buf[12], rair_buf[12];
		formatAge(uptime, up_buf, sizeof(up_buf));
		formatAge(air_tx, air_buf, sizeof(air_buf));
		formatAge(rx_air, rair_buf, sizeof(rair_buf));
		snprintf(text, sizeof(text),
				 "up:%s\nbatt:%umV\ntxq:%u\nnf:%ddB\nrssi:%ddBm\nsnr:%ddB\n"
				 "ptx:%u\nprx:%u\nftx:%u\nfrx:%u\ndtx:%u\ndrx:%u\n"
				 "err:%u\nef:%04X\ndup f:%u d:%u\nair:%s\nrair:%s",
				 up_buf,
				 (unsigned)batt, (unsigned)txq,
				 (int)noise, (int)rssi,
				 (int)(lsnr / 4),
				 (unsigned)ptx, (unsigned)prx,
				 (unsigned)sent, (unsigned)recvd,
				 (unsigned)sent_d, (unsigned)recvd_d,
				 (unsigned)errors, (unsigned)errf,
				 (unsigned)dup_f, (unsigned)dup_d,
				 air_buf, rair_buf);
	} else if (_pending == PENDING_BINARY_NEIGHBOURS && data_len >= 4) {
		/* Neighbours response layout (after tag):
		 *  0..1: total_count (i16)
		 *  2..3: returned_count (i16)
		 *  4+:   per entry = 4 bytes pubkey prefix + 4 bytes secs_ago + 1 byte snr */
		int16_t total = 0, returned = 0;
		memcpy(&total,    &data[0], 2);
		memcpy(&returned, &data[2], 2);
		if (returned == 0) {
			snprintf(text, sizeof(text), "-none-");
		} else {
			char *dp = text;
			char *end = text + sizeof(text) - 1;
			int offset = 4;
			for (int i = 0; i < returned && offset + 9 <= data_len && dp < end - 20; i++) {
				if (i > 0) *dp++ = '\n';
				char hex[12];
				snprintf(hex, sizeof(hex), "!%02X%02X%02X%02X",
						 data[offset], data[offset+1], data[offset+2], data[offset+3]);
				uint32_t secs_ago = 0;
				int8_t snr = 0;
				memcpy(&secs_ago, &data[offset + 4], 4);
				memcpy(&snr,      &data[offset + 8], 1);
				ContactInfo *nc = _task->getMesh()->lookupContactByPubKey(&data[offset], 4);
				char name_buf[32];
				const char *label;
				if (nc && nc->name[0]) {
					_task->getDisplay().translateUTF8ToBlocks(name_buf, nc->name, sizeof(name_buf));
					label = name_buf;
				} else {
					label = hex;
				}
				char age_buf[8];
				formatAge(secs_ago, age_buf, sizeof(age_buf));
				dp += snprintf(dp, end - dp, "%s %s %+ddB", label, age_buf, (int)(snr / 4));
				offset += 9;
			}
			*dp = '\0';
		}
	} else if (_pending == PENDING_BINARY_TELEMETRY && data_len >= 3) {
		telemetry_text(data, data_len, text, sizeof(text));
	} else if (_pending == PENDING_BINARY_OWNER_INFO && data_len > 0) {
		/* Plain text from repeater: firmware_version\nnode_name\nowner_info */
		char raw[ADMIN_RESP_MAX];
		int len = data_len < (int)(sizeof(raw) - 1) ? data_len : (int)(sizeof(raw) - 1);
		memcpy(raw, data, len);
		raw[len] = '\0';
		const char *fields[3] = { raw, "", "" };
		int fi = 0;
		for (char *p = raw; *p && fi < 2; p++) {
			if (*p == '\n') { *p = '\0'; fields[++fi] = p + 1; }
		}
		/* labels add ~19 chars, so format into a larger buffer then truncate */
		char fmt[ADMIN_RESP_MAX + 24];
		snprintf(fmt, sizeof(fmt), "FW: %s\nNode: %s\nOwner: %s", fields[0], fields[1], fields[2]);
		strncpy(text, fmt, sizeof(text) - 1);
		text[sizeof(text) - 1] = '\0';
	}

	if (text[0] && _hist_count > 0) {
		CmdEntry &newest = histAt(_hist_count - 1);
		if (!newest.has_resp) {
			int len = (int)strlen(text);
			if (len >= ADMIN_RESP_MAX) len = ADMIN_RESP_MAX - 1;
			memcpy(newest.resp, text, len);
			newest.resp[len] = '\0';
			newest.has_resp = true;
		}
	}
	_pending = PENDING_NONE;
	_task->clearAdminReqTag();
	k_timer_stop(&_timeout_timer);
}

/* ===== sendCLI ===== */

bool RepeaterAdminScreen::sendCLI(const char *cmd)
{
	BaseChatMesh *mesh = _task->getMesh();
	if (!mesh) return false;
	ContactInfo *contact = mesh->lookupContactByPubKey(_contact_pubkey, PUB_KEY_SIZE);
	if (!contact) {
		_task->showAlert("Contact lost", 1500);
		return false;
	}
	uint32_t ts = _rtc ? _rtc->getCurrentTimeUnique() : k_uptime_get_32();
	uint32_t est_timeout = 0;
	/* TXT_TYPE_CLI_DATA, not TXT_TYPE_CLI_COMMAND: this talks to repeaters of
	 * any vintage, and type 3 is only understood from v1.18 on. */
	int result = mesh->sendCommandData(*contact, ts, 0, TXT_TYPE_CLI_DATA, cmd, est_timeout);
	if (result == MSG_SEND_FAILED) {
		_task->showAlert("Send failed", 1500);
		return false;
	}
	ui_signal_tx();
	_last_sent_at = k_uptime_get_32();
	_awaiting_tx = true;
	/* est_timeout is one way, CLI response takes a second packet */
	_cmd_timeout_ms = ADMIN_TIMEOUT_MS;
	if (est_timeout > 0) {
		uint32_t rt = est_timeout * 2 + 3000;
		if (rt > _cmd_timeout_ms) _cmd_timeout_ms = rt;
	}
	k_timer_stop(&_timeout_timer);
	k_timer_start(&_timeout_timer, K_MSEC(_cmd_timeout_ms), K_NO_WAIT);
	return true;
}

/* ===== Callbacks ===== */

void RepeaterAdminScreen::onLoginResult(bool success, uint8_t permissions, uint32_t server_time)
{
	if (_state != STATE_LOGGING_IN) return;
	k_timer_stop(&_timeout_timer);
	if (success) {
		_permissions = permissions;
		_server_time = server_time;
		_state = STATE_SUBMENU;
		_submenu_sel = 0;
		_pending = PENDING_NONE;
		_hist_scroll = 0; _resp_line_scroll = 0;
		_task->showAlert("Login OK", 600);
	} else {
		_state = STATE_PASSWORD_ENTRY;
		_task->showAlert("Login failed", 1500);
	}
}

void RepeaterAdminScreen::onCliResponse(const char *text)
{
	if (!text || _pending != PENDING_CMD) return;
	if (_state != STATE_MAIN && _state != STATE_CMD_INPUT && _state != STATE_SUBMENU) return;
	k_timer_stop(&_timeout_timer);
	if (_hist_count > 0) {
		CmdEntry &newest = histAt(_hist_count - 1);
		if (!newest.has_resp) {
			int len = (int)strlen(text);
			if (len >= ADMIN_RESP_MAX) len = ADMIN_RESP_MAX - 1;
			memcpy(newest.resp, text, len);
			newest.resp[len] = '\0';
			newest.has_resp = true;
		}
	}
	_pending = PENDING_NONE;
}

void RepeaterAdminScreen::onPacketSent()
{
	/* RF TX completed — reset timeout clock so LBT/queue delay is excluded.
	 * Restart the response-timeout timer with the full window from now. */
	if (_awaiting_tx) {
		_last_sent_at = k_uptime_get_32();
		_awaiting_tx = false;
		k_timer_stop(&_timeout_timer);
		k_timer_start(&_timeout_timer, K_MSEC(_cmd_timeout_ms), K_NO_WAIT);
	}
}

/* ===== goBack ===== */

void RepeaterAdminScreen::goBack()
{
	if (_from_contacts) _task->gotoContactsScreen();
	else _task->gotoRepeatersScreen();
}

/* ===== Render helpers ===== */

static void renderAdminHeader(JoystickDisplay &display, const char *name, bool is_admin,
		uint32_t marquee_ms)
{
	int name_w = display.width();
	display.setTextSize(1);
	display.setColor(JoystickDisplay::GREEN);
	if (name_w > 0 && display.getTextWidth(name) > name_w) {
		int text_len = (int)strlen(name);
		if (text_len > 0) {
			static const uint32_t kPauseMs = 1500;
			uint32_t elapsed = k_uptime_get_32() - marquee_ms;
			uint32_t scroll_elapsed = (elapsed > kPauseMs) ? (elapsed - kPauseMs) : 0;
			int offset = (int)((scroll_elapsed / 450U) % (uint32_t)text_len);
			display.drawTextEllipsized(0, 0, name_w, name + offset);
		}
	} else {
		display.drawTextEllipsized(0, 0, name_w, name);
	}
	display.drawRect(0, kHeaderSepY, display.width(), 1);
}

/* ===== render ===== */

int RepeaterAdminScreen::render(JoystickDisplay &display)
{
	/* Timeout check: timer fired (or any wakeup arrived past deadline). */
	if (_last_sent_at > 0 &&
		(k_uptime_get_32() - _last_sent_at) > _cmd_timeout_ms) {
		onTimeout();
	}

	switch (_state) {

	case STATE_PASSWORD_ENTRY: {
		/* Header: "Password: <scrolling repeater name>" on one line */
		display.setTextSize(1);
		display.setColor(JoystickDisplay::GREEN);
		const char *prefix = "Password: ";
		int prefix_w = display.getTextWidth(prefix);
		display.drawTextLeftAlign(0, 0, prefix);
		int name_avail = display.width() - prefix_w;
		if (name_avail > 0) {
			int name_len = (int)strlen(_repeater_name);
			if (name_len > 0 && display.getTextWidth(_repeater_name) > name_avail) {
				static const uint32_t kPauseMs = 1500;
				uint32_t elapsed = k_uptime_get_32() - _admin_header_marquee_ms;
				uint32_t scroll_elapsed = (elapsed > kPauseMs) ? (elapsed - kPauseMs) : 0;
				int offset = (int)((scroll_elapsed / 450U) % (uint32_t)name_len);
				display.drawTextEllipsized(prefix_w, 0, name_avail, _repeater_name + offset);
			} else {
				display.drawTextEllipsized(prefix_w, 0, name_avail, _repeater_name);
			}
		}
		display.drawRect(0, kHeaderSepY, display.width(), 1);
		/* Password input */
		char pwd_display[ADMIN_PASSWORD_MAX + 2];
		int n = (_pwd_len < ADMIN_PASSWORD_MAX) ? _pwd_len : ADMIN_PASSWORD_MAX;
		memcpy(pwd_display, _password, n);
		pwd_display[n] = '|'; pwd_display[n + 1] = '\0';
		display.setColor(JoystickDisplay::YELLOW);
		display.drawTextLeftAlign(0, kContentY, pwd_display);
		renderT9Keypad(display, getT9KeyLabels(_pwd_kb_letters, true), _pwd_t9_sel, kContentY + kMenuLineH);
		return 300;
	}

	case STATE_LOGGING_IN: {
		renderScreenHeader(display, _repeater_name, 0, 0);
		display.setTextSize(1);
		display.setColor(JoystickDisplay::GREEN);
		display.drawTextCentered(display.width() / 2, 28, "Logging in...");
		uint32_t d = (k_uptime_get_32() / 333) % 3;
		const char *dots[] = { ".", "..", "..." };
		display.drawTextCentered(display.width() / 2, 38, dots[d]);
		return 333;
	}

	case STATE_SUBMENU: {
		renderAdminHeader(display, _repeater_name, _permissions != 0, _admin_header_marquee_ms);
		display.setTextSize(1);
		display.setColor(_permissions != 0 ? JoystickDisplay::YELLOW : JoystickDisplay::LIGHT);
		display.drawTextCentered(display.width() / 2, kHeaderSepY + 2,
								 _permissions != 0 ? "Admin" : "Guest");
		static const char * const kItems[] = { "Neighbours", "Stats", "Telemetry", "Info", "Commands", "Time Sync" };
		int count = (_permissions != 0) ? 6 : 4;
		int max_vis = (display.height() - kAdminContentY) / kMenuLineH;
		if (max_vis < 1) max_vis = 1;
		int start = computeListStart(_submenu_sel, count, max_vis);
		int y = kAdminContentY;
		for (int i = start; i < count && i < start + max_vis; i++) {
			renderMenuListText(display, y, i == _submenu_sel, kItems[i]);
			y += kMenuLineH;
		}
		return 500;
	}

	case STATE_MAIN: {
		renderAdminHeader(display, _repeater_name, _permissions != 0, _admin_header_marquee_ms);
		display.setTextSize(1);
		display.setColor(_permissions != 0 ? JoystickDisplay::YELLOW : JoystickDisplay::LIGHT);
		display.drawTextCentered(display.width() / 2, kHeaderSepY + 2,
								 _permissions != 0 ? "Admin" : "Guest");
		if (_hist_count == 0) {
			display.setColor(JoystickDisplay::LIGHT);
			display.drawTextLeftAlign(0, kAdminContentY, "(no history)");
			display.drawTextLeftAlign(0, kAdminContentY + kLineH, "Long ENTER: send cmd");
		} else {
			int logical = _hist_count - 1 - _hist_scroll;
			if (logical < 0) logical = 0;
			const CmdEntry &e = histAt(logical);
			int y = kAdminContentY;

			/* Command line */
			char cmd_line[ADMIN_CMD_MAX + 12];
			if (_hist_count > 1)
				snprintf(cmd_line, sizeof(cmd_line), "[%d/%d] > %s",
						 logical + 1, _hist_count, e.cmd);
			else
				snprintf(cmd_line, sizeof(cmd_line), "> %s", e.cmd);
			display.setColor(JoystickDisplay::YELLOW);
			display.drawTextEllipsized(0, y, display.width(), cmd_line);
			y += kLineH;

			/* Response — show all lines until display bottom */
			if (!e.has_resp && _pending != PENDING_NONE && logical == _hist_count - 1) {
				display.setColor(JoystickDisplay::LIGHT);
				display.drawTextLeftAlign(2, y, "(waiting...)");
			} else if (e.has_resp && e.resp[0]) {
				/* Count lines and compute vertical scroll limits */
				int resp_lines = 1;
				for (const char *p = e.resp; *p; p++) if (*p == '\n') resp_lines++;
				int vis_lines = (display.height() - y) / kLineH;
				_resp_max_line_scroll = (resp_lines > vis_lines) ? resp_lines - vis_lines : 0;
				if (_resp_line_scroll > _resp_max_line_scroll) _resp_line_scroll = _resp_max_line_scroll;

				/* Skip lines above the scroll offset */
				const char *rp = e.resp;
				for (int skip = 0; skip < _resp_line_scroll && rp; skip++) {
					const char *nl = strchr(rp, '\n');
					rp = nl ? nl + 1 : nullptr;
				}

				bool is_scrollable = (strcmp(e.cmd, "neighbors") == 0 || strcmp(e.cmd, "info") == 0);
				uint32_t scroll_elapsed = 0;
				if (is_scrollable) {
					static const uint32_t kNbPauseMs = 1500;
					uint32_t elapsed = k_uptime_get_32() - _admin_header_marquee_ms;
					scroll_elapsed = (elapsed > kNbPauseMs) ? (elapsed - kNbPauseMs) : 0;
				}
				while (rp && *rp && y + kLineH <= display.height()) {
					const char *nl = strchr(rp, '\n');
					int rlen = nl ? (int)(nl - rp) : (int)strlen(rp);
					display.setColor(JoystickDisplay::GREEN);
					if (is_scrollable && rlen > 0) {
						char line_buf[ADMIN_RESP_MAX];
						int copy = rlen < (int)(sizeof(line_buf) - 1) ? rlen : (int)(sizeof(line_buf) - 1);
						memcpy(line_buf, rp, copy);
						line_buf[copy] = '\0';
						int line_w = display.width();
						if (display.getTextWidth(line_buf) > line_w) {
							int offset = (int)((scroll_elapsed / 450U) % (uint32_t)copy);
							display.drawTextEllipsized(0, y, line_w, line_buf + offset);
						} else {
							char prefixed[ADMIN_RESP_MAX + 2];
							prefixed[0] = ' '; prefixed[1] = ' ';
							memcpy(prefixed + 2, line_buf, copy);
							prefixed[copy + 2] = '\0';
							display.drawTextEllipsized(0, y, line_w, prefixed);
						}
					} else {
						char resp_line[42];
						int copy = rlen < 39 ? rlen : 39;
						resp_line[0] = ' '; resp_line[1] = ' ';
						memcpy(resp_line + 2, rp, copy);
						resp_line[copy + 2] = '\0';
						display.drawTextEllipsized(0, y, display.width(), resp_line);
					}
					y += kLineH;
					rp = nl ? nl + 1 : nullptr;
				}
			} else {
				display.setColor(JoystickDisplay::LIGHT);
				display.drawTextLeftAlign(2, y, "-");
			}
		}
		return (_pending != PENDING_NONE) ? 333 : 500;
	}

	case STATE_CMD_INPUT: {
		renderScreenHeader(display, "Command", 0, 0);
		display.setTextSize(1);
		char buf[ADMIN_CMD_MAX + 2];
		int pn = (_cmd_len < ADMIN_CMD_MAX) ? _cmd_len : ADMIN_CMD_MAX;
		memcpy(buf, _cmd_buf, pn);
		buf[pn] = '|'; buf[pn + 1] = '\0';
		display.setColor(JoystickDisplay::YELLOW);
		display.drawTextEllipsized(0, kContentY, display.width(), buf);
		renderT9Keypad(display, getT9KeyLabels(_cmd_kb_letters, false),
					   _cmd_t9_sel, kContentY + 12);
		return 300;
	}

	} /* switch */
	return 500;
}

/* ===== handleInput ===== */

/* handleInput() in STATE_PASSWORD_ENTRY. */
bool RepeaterAdminScreen::handlePasswordKey(char c)
{
	if (handleT9DirectionalInput(c, _pwd_t9_sel)) return true;
	if (c == KEY_ENTER) {
		if (_pwd_t9_sel == 3) {
			if (_pwd_len > 0) _password[--_pwd_len] = '\0';
			resetT9State(_pwd_t9_last_key, _pwd_t9_letter_index, _pwd_t9_last_press);
			return true;
		}
		if (_pwd_t9_sel == 7) {
			if (_pwd_len < ADMIN_PASSWORD_MAX - 1) {
				_password[_pwd_len++] = ' ';
				_password[_pwd_len] = '\0';
			}
			resetT9State(_pwd_t9_last_key, _pwd_t9_letter_index, _pwd_t9_last_press);
			return true;
		}
		if (_pwd_t9_sel == 11) {
			BaseChatMesh *mesh = _task->getMesh();
			if (!mesh) { _task->showAlert("No mesh", 1500); return true; }
			ContactInfo *contact = mesh->lookupContactByPubKey(_contact_pubkey, PUB_KEY_SIZE);
			if (!contact) { _task->showAlert("Contact not found", 1500); return true; }
			uint32_t est_timeout = 0;
			int result = mesh->sendLogin(*contact, _password, est_timeout);
			if (result != MSG_SEND_FAILED) {
				ui_signal_tx();
				_state = STATE_LOGGING_IN;
				_last_sent_at = k_uptime_get_32();
				_awaiting_tx = true;
				_cmd_timeout_ms = ADMIN_TIMEOUT_MS;
				if (est_timeout > 0) {
					uint32_t rt = est_timeout * 2 + 3000;
					if (rt > _cmd_timeout_ms) _cmd_timeout_ms = rt;
				}
				k_timer_stop(&_timeout_timer);
				k_timer_start(&_timeout_timer, K_MSEC(_cmd_timeout_ms), K_NO_WAIT);
			} else {
				_task->showAlert("Send failed", 1500);
			}
			resetT9State(_pwd_t9_last_key, _pwd_t9_letter_index, _pwd_t9_last_press);
			return true;
		}
		if (_pwd_t9_sel == 12) {
			_pwd_kb_letters = !_pwd_kb_letters;
			resetT9State(_pwd_t9_last_key, _pwd_t9_letter_index, _pwd_t9_last_press);
			return true;
		}
		if (_pwd_t9_sel == 15) {
			resetT9State(_pwd_t9_last_key, _pwd_t9_letter_index, _pwd_t9_last_press);
			goBack();
			return true;
		}
		const char *letters = getT9KeyLetters(_pwd_kb_letters)[_pwd_t9_sel];
		if (letters && letters[0]) {
			appendOrCycleT9Char(_password, _pwd_len, ADMIN_PASSWORD_MAX, letters,
								_pwd_t9_sel, _pwd_t9_last_key, _pwd_t9_letter_index,
								_pwd_t9_last_press);
		}
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) { goBack(); return true; }
	return false;
}

/* handleInput() in STATE_SUBMENU. */
bool RepeaterAdminScreen::handleSubmenuKey(char c)
{
	bool is_admin = (_permissions != 0);
	int count = is_admin ? 6 : 4;
	if (handleCommonListNavigation(c, _submenu_sel, count)) return true;
	if (c == KEY_ENTER) {
		if (_submenu_sel == 4 && is_admin) {
			/* Commands: open free form CLI input directly */
			_cmd_len = 0;
			memset(_cmd_buf, 0, sizeof(_cmd_buf));
			_cmd_t9_sel = 0; _cmd_t9_last_key = -1;
			_cmd_t9_letter_index = 0; _cmd_t9_last_press = 0;
			_cmd_kb_letters = true;
			_hist_scroll = 0; _resp_line_scroll = 0;
			_state = STATE_CMD_INPUT;
			return true;
		}

		if (_submenu_sel == 5 && is_admin) {
			/* Time Sync: send "clock sync" CLI. The repeater reads the
			 * packet's sender_timestamp (set by sendCommandData() to our
			 * current epoch) and updates its own RTC if our time is ahead. */
			int idx;
			if (_hist_count < ADMIN_HIST_MAX) {
				idx = (_hist_head + _hist_count) % ADMIN_HIST_MAX;
				_hist_count++;
			} else {
				idx = _hist_head;
				_hist_head = (_hist_head + 1) % ADMIN_HIST_MAX;
			}
			strncpy(_hist[idx].cmd, "Time Sync", ADMIN_CMD_MAX - 1);
			_hist[idx].cmd[ADMIN_CMD_MAX - 1] = '\0';
			_hist[idx].resp[0] = '\0';
			_hist[idx].has_resp = false;
			if (sendCLI("clock sync")) {
				_pending = PENDING_CMD;
				_state = STATE_MAIN;
				_hist_scroll = 0;
			}
			return true;
		}

		/* Items 0 through 3: Neighbours, Stats, Telemetry, Info */
		struct {
			const char *label;
			uint8_t req_type; /* 0 = CLI only */
			PendingKind pending;
			const char *cli_cmd;  /* non null means admin uses CLI */
		} dispatch[] = {
			{ "neighbors",  0x06, PENDING_BINARY_NEIGHBOURS, nullptr     },
			{ "stats",      0x01, PENDING_BINARY_STATUS,     nullptr     },
			{ "telemetry",  0x03, PENDING_BINARY_TELEMETRY,  nullptr     },
			{ "info",       0x07, PENDING_BINARY_OWNER_INFO, nullptr     },
		};
		if (_submenu_sel < 0 || _submenu_sel > 3) {
			_state = STATE_MAIN; _hist_scroll = 0; return true;
		}
		const auto &d = dispatch[_submenu_sel];

		int idx;
		if (_hist_count < ADMIN_HIST_MAX) {
			idx = (_hist_head + _hist_count) % ADMIN_HIST_MAX;
			_hist_count++;
		} else {
			idx = _hist_head;
			_hist_head = (_hist_head + 1) % ADMIN_HIST_MAX;
		}
		strncpy(_hist[idx].cmd, d.label, ADMIN_CMD_MAX - 1);
		_hist[idx].cmd[ADMIN_CMD_MAX - 1] = '\0';
		_hist[idx].resp[0] = '\0';
		_hist[idx].has_resp = false;

		bool sent = false;
		if (is_admin && d.cli_cmd) {
			sent = sendCLI(d.cli_cmd);
			if (sent) _pending = PENDING_CMD;
		} else {
			/* binary request — used by guests always, and by admin for telemetry/info */
			uint8_t req[7]; int req_len;
			if (d.req_type == 0x06) {
				req[0] = 0x06; req[1] = 0; req[2] = 10;
				req[3] = 0; req[4] = 0; req[5] = 0; req[6] = 4;
				req_len = 7;
			} else {
				req[0] = d.req_type; req_len = 1;
			}
			sent = sendBinaryReqHelper(_task, _contact_pubkey, req, req_len,
									   _cmd_timeout_ms, _awaiting_tx, _last_sent_at);
			if (sent) {
				_pending = d.pending;
				k_timer_stop(&_timeout_timer);
				k_timer_start(&_timeout_timer, K_MSEC(_cmd_timeout_ms), K_NO_WAIT);
			}
		}

		if (!sent) {
			snprintf(_hist[idx].resp, ADMIN_RESP_MAX, "(send failed)");
			_hist[idx].has_resp = true;
		}
		_state = STATE_MAIN;
		_hist_scroll = 0; _resp_line_scroll = 0;
		return true;
	}
	if (c == KEY_CANCEL || c == KEY_HOME) { goBack(); return true; }
	return false;
}

/* handleInput() in STATE_MAIN. */
bool RepeaterAdminScreen::handleMainKey(char c)
{
	int max_scroll = (_hist_count > 1) ? _hist_count - 1 : 0;
	if (c == KEY_UP) {
		if (_resp_line_scroll > 0) { _resp_line_scroll--; }
		else if (_hist_scroll < max_scroll) { _hist_scroll++; _resp_line_scroll = 0; }
		return true;
	}
	if (c == KEY_DOWN) {
		if (_hist_scroll > 0) { _hist_scroll--; _resp_line_scroll = 0; }
		else if (_resp_line_scroll < _resp_max_line_scroll) { _resp_line_scroll++; }
		return true;
	}
	if (c == KEY_TO_TOP)    { _hist_scroll = max_scroll; _resp_line_scroll = 0; return true; }
	if (c == KEY_TO_BOTTOM) { _hist_scroll = 0;          _resp_line_scroll = 0; return true; }
	if (c == KEY_ENTER_LONG) {
		_cmd_len = 0;
		memset(_cmd_buf, 0, sizeof(_cmd_buf));
		_cmd_t9_sel = 0; _cmd_t9_last_key = -1;
		_cmd_t9_letter_index = 0; _cmd_t9_last_press = 0;
		_cmd_kb_letters = true;
		_state = STATE_CMD_INPUT;
		return true;
	}
	if (c == KEY_CANCEL) { _state = STATE_SUBMENU; return true; }
	if (c == KEY_HOME)   { goBack(); return true; }
	return false;
}

/* handleInput() in STATE_CMD_INPUT. */
bool RepeaterAdminScreen::handleCmdInputKey(char c)
{
	if (handleT9DirectionalInput(c, _cmd_t9_sel)) return true;
	if (c == KEY_ENTER) {
		if (_cmd_t9_sel == 3) {
			if (_cmd_len > 0) _cmd_buf[--_cmd_len] = '\0';
			resetT9State(_cmd_t9_last_key, _cmd_t9_letter_index, _cmd_t9_last_press);
			return true;
		}
		if (_cmd_t9_sel == 7) {
			if (_cmd_len < ADMIN_CMD_MAX - 1) {
				_cmd_buf[_cmd_len++] = ' ';
				_cmd_buf[_cmd_len] = '\0';
			}
			resetT9State(_cmd_t9_last_key, _cmd_t9_letter_index, _cmd_t9_last_press);
			return true;
		}
		if (_cmd_t9_sel == 11) {
			if (_cmd_len > 0) {
				/* Add entry to ring buffer */
				int idx;
				if (_hist_count < ADMIN_HIST_MAX) {
					idx = (_hist_head + _hist_count) % ADMIN_HIST_MAX;
					_hist_count++;
				} else {
					idx = _hist_head;
					_hist_head = (_hist_head + 1) % ADMIN_HIST_MAX;
				}
				strncpy(_hist[idx].cmd, _cmd_buf, ADMIN_CMD_MAX - 1);
				_hist[idx].cmd[ADMIN_CMD_MAX - 1] = '\0';
				_hist[idx].resp[0] = '\0';
				_hist[idx].has_resp = false;
				if (sendCLI(_cmd_buf)) {
					_pending = PENDING_CMD;
				} else {
					snprintf(_hist[idx].resp, ADMIN_RESP_MAX, "(send failed)");
					_hist[idx].has_resp = true;
				}
			}
			_hist_scroll = 0; _resp_line_scroll = 0;
			_state = STATE_MAIN;
			resetT9State(_cmd_t9_last_key, _cmd_t9_letter_index, _cmd_t9_last_press);
			return true;
		}
		if (_cmd_t9_sel == 12) {
			_cmd_kb_letters = !_cmd_kb_letters;
			resetT9State(_cmd_t9_last_key, _cmd_t9_letter_index, _cmd_t9_last_press);
			return true;
		}
		if (_cmd_t9_sel == 15) {
			_state = STATE_MAIN;
			resetT9State(_cmd_t9_last_key, _cmd_t9_letter_index, _cmd_t9_last_press);
			return true;
		}
		const char *letters = getT9KeyLetters(_cmd_kb_letters)[_cmd_t9_sel];
		if (letters && letters[0]) {
			appendOrCycleT9Char(_cmd_buf, _cmd_len, ADMIN_CMD_MAX, letters,
								_cmd_t9_sel, _cmd_t9_last_key, _cmd_t9_letter_index,
								_cmd_t9_last_press);
		}
		return true;
	}
	if (c == KEY_CANCEL) { _state = STATE_MAIN; return true; }
	if (c == KEY_HOME)   { goBack(); return true; }
	return false;
}

bool RepeaterAdminScreen::handleInput(char c)
{
	switch (_state) {

	case STATE_PASSWORD_ENTRY:
		return handlePasswordKey(c);

	case STATE_LOGGING_IN:
		if (c == KEY_CANCEL || c == KEY_HOME) {
			_awaiting_tx = false;
			_state = STATE_PASSWORD_ENTRY;
			return true;
		}
		return false;

	case STATE_SUBMENU:
		return handleSubmenuKey(c);

	case STATE_MAIN:
		return handleMainKey(c);

	case STATE_CMD_INPUT:
		return handleCmdInputKey(c);

	} /* switch */
	return false;
}
