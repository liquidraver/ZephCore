/*
 * ZephCore - Joystick UI Messaging Screens
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "../joystick_screens.h"
#include "../joystick_ui_task.h"
#include "screen_helpers.h"
#include <helpers/ChannelDetails.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

UnreadScreen::UnreadScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc),
	  _entry_count(0), _visible_unread_count(0),
	  _head_index(MAX_UNREAD_MSGS - 1),
	  _selected(0), _list_scroll(0),
	  _details(false), _detail_scroll(0),
	  _transient_preview(false), _preview_expiry(0)
{
	for (int i = 0; i < MAX_UNREAD_MSGS; i++) {
		_entries[i].timestamp = 0;
		_entries[i].origin[0] = '\0';
		_entries[i].msg[0] = '\0';
		_entries[i].read = true;
	}
	k_timer_init(&_preview_timer, previewTimerCb, NULL);
	k_timer_user_data_set(&_preview_timer, this);
}

void UnreadScreen::previewTimerCb(struct k_timer *t)
{
	/* ISR — just wake main loop; render() detects the expiry and dismisses. */
	auto *self = static_cast<UnreadScreen *>(k_timer_user_data_get(t));
	if (self && self->_task) self->_task->notify();
}

void UnreadScreen::onExit()
{
	k_timer_stop(&_preview_timer);
}

void UnreadScreen::markSentEntryStatus(uint32_t ts, const char *contact_name, bool delivered)
{
	if (!contact_name) return;
	for (int i = 0; i < MAX_UNREAD_MSGS; i++) {
		if (_entries[i].timestamp != ts) continue;
		/* Only touch entries that look like our own sent message to that
		 * contact — origin currently starts with "(>>" (set by addPreview
		 * when path_len == OUT_PATH_SENT). */
		if (strncmp(_entries[i].origin, "(>>", 3) != 0) continue;
		if (!strstr(_entries[i].origin, contact_name)) continue;
		snprintf(_entries[i].origin, sizeof(_entries[i].origin),
				 "(>>%s) %s:", delivered ? "+" : "X", contact_name);
		return;
	}
}

void UnreadScreen::normalizeUnreadState()
{
	if (_entry_count < 0) _entry_count = 0;
	if (_entry_count > MAX_UNREAD_MSGS) _entry_count = MAX_UNREAD_MSGS;
	if (_head_index < 0 || _head_index >= MAX_UNREAD_MSGS) _head_index = MAX_UNREAD_MSGS - 1;

	int actual = 0;
	for (int i = 0; i < _entry_count; i++) {
		int pos = (_head_index - i + MAX_UNREAD_MSGS) % MAX_UNREAD_MSGS;
		if (!_entries[pos].read) actual++;
	}
	_visible_unread_count = actual;

	if (_selected < 0) _selected = 0;
	if (_visible_unread_count == 0) {
		_selected = 0;
	} else if (_selected >= _visible_unread_count) {
		_selected = _visible_unread_count - 1;
	}
}

int UnreadScreen::getUnreadCount()
{
	normalizeUnreadState();
	return _visible_unread_count;
}

void UnreadScreen::activatePreview(bool transient, uint32_t timeout_ms)
{
	normalizeUnreadState();
	_transient_preview = transient;
	_details = false;
	_detail_scroll = 0;
	_list_scroll = 0;
	if (_selected >= _visible_unread_count)
		_selected = (_visible_unread_count > 0) ? _visible_unread_count - 1 : 0;
	if (transient) {
		_preview_expiry = k_uptime_get_32() + timeout_ms;
		k_timer_start(&_preview_timer, K_MSEC(timeout_ms), K_NO_WAIT);
	} else {
		_preview_expiry = 0;
		k_timer_stop(&_preview_timer);
	}
}

const UnreadScreen::MsgEntry *UnreadScreen::getByListIndex(int idx) const
{
	if (idx < 0 || idx >= _visible_unread_count) return nullptr;
	int found = 0;
	for (int i = 0; i < _entry_count; i++) {
		int pos = (_head_index - i + MAX_UNREAD_MSGS) % MAX_UNREAD_MSGS;
		if (!_entries[pos].read) {
			if (found == idx) return &_entries[pos];
			found++;
		}
	}
	return nullptr;
}

void UnreadScreen::addPreview(uint8_t path_len, const char *from_name, const char *msg,
		bool initially_read)
{
	normalizeUnreadState();
	_head_index = (_head_index + 1) % MAX_UNREAD_MSGS;
	if (_entry_count < MAX_UNREAD_MSGS) {
		_entry_count++;
	} else if (!_entries[_head_index].read && _visible_unread_count > 0) {
		_visible_unread_count--;
	}
	MsgEntry &entry = _entries[_head_index];
	entry.timestamp = _rtc->getCurrentTime();
	entry.read = initially_read;
	if (!initially_read && _visible_unread_count < MAX_UNREAD_MSGS) _visible_unread_count++;

	const char *safe_from = (from_name && from_name[0]) ? from_name : "unknown";
	const char *safe_msg = msg ? msg : "";
	if (path_len == OUT_PATH_UNKNOWN)
		snprintf(entry.origin, sizeof(entry.origin), "(D) %s:", safe_from);
	else if (path_len == OUT_PATH_SENT)
		snprintf(entry.origin, sizeof(entry.origin), "(>>) %s:", safe_from);
	else
		snprintf(entry.origin, sizeof(entry.origin), "(%d) %s:", (int)path_len, safe_from);
	strncpy(entry.msg, safe_msg, sizeof(entry.msg) - 1);
	entry.msg[sizeof(entry.msg) - 1] = '\0';
	_selected = 0;
}

int UnreadScreen::getStoredMsgCount() const
{
	return _entry_count;
}

bool UnreadScreen::getStoredMessageForSourceAt(const char *source_name, uint32_t timestamp,
											   const char *&out_msg) const
{
	if (!source_name || !source_name[0]) return false;
	for (int i = 0; i < _entry_count; i++) {
		int pos = (_head_index - i + MAX_UNREAD_MSGS) % MAX_UNREAD_MSGS;
		const MsgEntry &e = _entries[pos];
		if (e.timestamp != timestamp) continue;
		char sender[32] = {};
		parseMessageOriginAndPath(e.origin, sender, sizeof(sender), nullptr);
		if (strcmp(sender, source_name) != 0) continue;
		out_msg = e.msg;
		return true;
	}
	return false;
}

bool UnreadScreen::getLatestStoredMessageForSource(const char *source_name,
												   const char *&out_msg, uint32_t *out_ts) const
{
	if (!source_name || !source_name[0]) return false;
	for (int i = 0; i < _entry_count; i++) {
		int pos = (_head_index - i + MAX_UNREAD_MSGS) % MAX_UNREAD_MSGS;
		const MsgEntry &e = _entries[pos];
		char sender[32] = {};
		parseMessageOriginAndPath(e.origin, sender, sizeof(sender), nullptr);
		if (strcmp(sender, source_name) != 0) continue;
		out_msg = e.msg;
		if (out_ts) *out_ts = e.timestamp;
		return true;
	}
	return false;
}

int UnreadScreen::getContactMsgCount(const char *contact_name) const
{
	if (!contact_name || !contact_name[0]) return 0;
	int count = 0;
	for (int i = 0; i < _entry_count; i++) {
		int pos = (_head_index - i + MAX_UNREAD_MSGS) % MAX_UNREAD_MSGS;
		char sender[32] = {};
		parseMessageOriginAndPath(_entries[pos].origin, sender, sizeof(sender), nullptr);
		if (strcmp(sender, contact_name) == 0) count++;
	}
	return count;
}

bool UnreadScreen::getContactMsgAt(const char *contact_name, int idx,
								   const char *&out_msg, uint32_t &out_ts, uint8_t *out_path) const
{
	if (!contact_name || !contact_name[0]) return false;
	int count = 0;
	for (int i = 0; i < _entry_count; i++) {
		int pos = (_head_index - i + MAX_UNREAD_MSGS) % MAX_UNREAD_MSGS;
		const MsgEntry &e = _entries[pos];
		uint8_t path_len = OUT_PATH_UNKNOWN;
		char sender[32] = {};
		parseMessageOriginAndPath(e.origin, sender, sizeof(sender), &path_len);
		if (strcmp(sender, contact_name) != 0) continue;
		if (count == idx) {
			out_msg = e.msg;
			out_ts = e.timestamp;
			if (out_path) *out_path = path_len;
			return true;
		}
		count++;
	}
	return false;
}

void UnreadScreen::markCurrentRead()
{
	normalizeUnreadState();
	if (_visible_unread_count <= 0) return;
	int found = 0;
	for (int i = 0; i < _entry_count; i++) {
		int pos = (_head_index - i + MAX_UNREAD_MSGS) % MAX_UNREAD_MSGS;
		if (!_entries[pos].read) {
			if (found == _selected) {
				_entries[pos].read = true;
				_visible_unread_count--;
				if (_selected >= _visible_unread_count && _selected > 0) _selected--;
				return;
			}
			found++;
		}
	}
}

int UnreadScreen::render(JoystickDisplay &display)
{
	/* Preview expiry: dismiss transient preview when its timer elapses. */
	if (_transient_preview && _preview_expiry > 0 && k_uptime_get_32() >= _preview_expiry) {
		_transient_preview = false;
		_preview_expiry = 0;
		_details = false;
		_task->gotoHomeScreen();
		return 0;
	}

	normalizeUnreadState();
	if (_visible_unread_count == 0) {
		renderScreenHeader(display, "Unread", 0, 0);
		display.setTextSize(1);
		display.setColor(JoystickDisplay::GREEN);
		display.drawTextCentered(display.width() / 2, 30, "No unread msgs");
		return 1000;
	}

	if (_selected >= _visible_unread_count) _selected = _visible_unread_count - 1;
	if (_selected < 0) _selected = 0;

	if (_details) {
		const MsgEntry *entry = getByListIndex(_selected);
		if (!entry) { _details = false; return 200; }

		char sender_name[32] = {};
		uint8_t path_len = OUT_PATH_UNKNOWN;
		parseMessageOriginAndPath(entry->origin, sender_name, sizeof(sender_name), &path_len);

		char msg_safe[sizeof(entry->msg)];
		sanitizeForDisplay(entry->msg, msg_safe, sizeof(msg_safe));

		char route_line[48];
		buildMessageRouteLineForName("", sender_name, path_len, route_line, sizeof(route_line));

		return renderMessageDetailView(display, _selected, _visible_unread_count, entry->timestamp,
				sender_name[0] ? sender_name : "unknown",
				route_line, msg_safe, _detail_scroll);
	}

	renderScreenHeader(display, "Unread", _selected, _visible_unread_count);
	display.setTextSize(1);

	const int ROW_H = 10;
	const int LINES_PER_MSG = 2;
	const int HEADER_H = 14;
	int visible_count = getMessagePreviewVisibleCount(display, ROW_H, LINES_PER_MSG, HEADER_H);
	_list_scroll = getCenteredMessagePreviewStart(_selected, _visible_unread_count, visible_count);

	int y = HEADER_H;
	for (int li = _list_scroll; li < _visible_unread_count && y < display.height() - 2; li++) {
		const MsgEntry *entry = getByListIndex(li);
		if (!entry) continue;
		char origin_safe[sizeof(entry->origin)];
		char msg_safe[sizeof(entry->msg)];
		sanitizeForDisplay(entry->origin, origin_safe, sizeof(origin_safe));
		sanitizeForDisplay(entry->msg,    msg_safe,    sizeof(msg_safe));
		y = renderMessagePreviewEntry(display, y, li == _selected, origin_safe, msg_safe,
				ROW_H, LINES_PER_MSG, 20, 10, 4);
	}
	return 500;
}

bool UnreadScreen::handleInput(char key)
{
	normalizeUnreadState();

	if (_details) {
		const MsgEntry *entry = getByListIndex(_selected);
		int max_scroll = 0;
		if (entry) {
			JoystickDisplay &display = _task->getDisplay();
			max_scroll = getMessageDetailMaxScrollSanitized(display, entry->msg, 20, 5, 5);
		}
		if (handleDetailScrollNavigation(key, _detail_scroll, max_scroll)) return true;
		if (key == KEY_CANCEL || key == KEY_HOME || key == KEY_ENTER) {
			_details = false;
			_detail_scroll = 0;
			_list_scroll = 0;
			return true;
		}
		if (key == KEY_ENTER_LONG) {
			if (!entry) return true;
			/* Determine if this is a channel message by checking origin for '#' */
			char origin_sender[32] = {};
			uint8_t path_len_origin = OUT_PATH_UNKNOWN;
			parseMessageOriginAndPath(entry->origin, origin_sender, sizeof(origin_sender),
					&path_len_origin);
			if (origin_sender[0] == '#') {
				/* Channel message: channel name follows '#' in origin_sender.
				 * Resolve the channel slot by name — sendComposedMessage()
				 * routes only by numeric index, so passing -1 here would let
				 * the user compose and then silently fail to send (F3). */
				const char *ch_name = origin_sender + 1;
				auto *mesh = _task->getMesh();
				int ch_idx = -1;
				ChannelDetails ch_lookup;
				for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
					if (mesh->getChannel(i, ch_lookup) && ch_lookup.name[0] &&
							strcmp(ch_lookup.name, ch_name) == 0) {
						ch_idx = i;
						break;
					}
				}
				/* Unknown channel (e.g. long name truncated in origin[32]) —
				 * can't reply without a valid slot; swallow the long-press. */
				if (ch_idx < 0) return true;
				_task->setComposeChannel(ch_idx, ch_name);
				char at_buf[38];
				if (buildChannelReplyPrefix(entry->msg, path_len_origin,
						at_buf, sizeof(at_buf))) {
					_task->gotoT9InputScreenWithPrefix(at_buf);
				} else {
					_task->gotoT9InputScreen();
				}
			}
			return true;
		}
		return false;
	}

	if (_visible_unread_count > 0 &&
		handleCommonListNavigation(key, _selected, _visible_unread_count)) return true;

	if (key == KEY_RIGHT && _visible_unread_count > 0) { markCurrentRead(); return true; }
	if (key == KEY_ENTER && _visible_unread_count > 0) { _details = true; _detail_scroll = 0; return true; }
	if (key == KEY_CANCEL || key == KEY_HOME) {
		_transient_preview = false;
		_preview_expiry = 0;
		_list_scroll = 0;
		_task->gotoHomeScreen();
		return true;
	}
	return false;
}

ChannelsScreen::ChannelsScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc), _selected(0),
	  _show_msgs(false), _msg_channel_idx(-1), _msg_scroll(0),
	  _msg_details(false), _msg_detail_scroll(0)
{
	_msg_channel[0] = '\0';
}

int ChannelsScreen::getChannelCount() const
{
	auto *mesh = _task->getMesh();
	int count = 0;
	ChannelDetails ch;
	for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
		if (mesh->getChannel(i, ch) && ch.name[0]) count++;
	}
	return count;
}

bool ChannelsScreen::getChannelByListIndex(int listIdx, ChannelDetails &ch, int *slot_out) const
{
	auto *mesh = _task->getMesh();
	int matched = 0;
	for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
		if (!mesh->getChannel(i, ch) || !ch.name[0]) continue;
		if (matched == listIdx) { if (slot_out) *slot_out = i; return true; }
		matched++;
	}
	return false;
}

int ChannelsScreen::render(JoystickDisplay &display)
{
	/* Message view */
	if (_show_msgs) {
		int total = _task->getChannelPreviewCountFor(_msg_channel);

		if (total <= 0) {
			renderScreenHeader(display, _msg_channel, 0, 0);
			display.setColor(JoystickDisplay::YELLOW);
			display.drawTextCentered(display.width() / 2, 24, "No queued messages");
			display.setColor(JoystickDisplay::LIGHT);
			display.drawTextCentered(display.width() / 2, 38, "Long ENTER compose");
			display.drawTextCentered(display.width() / 2, 49, "CANCEL back");
			return 400;
		}

		if (_msg_scroll >= total) _msg_scroll = total - 1;
		if (_msg_scroll < 0)     _msg_scroll = 0;

		/* Message detail view */
		if (_msg_details) {
			const char *msg = "";
			uint32_t ts = 0;
			uint8_t path_len = OUT_PATH_UNKNOWN;
			if (!_task->getChannelPreviewFor(_msg_channel, _msg_scroll, msg, ts, &path_len)) {
				return 400;
			}

			char msg_safe[96];
			sanitizeForDisplay(msg, msg_safe, sizeof(msg_safe));

			char hop_buf[16], source_with_hops[52];
			formatHopCount(path_len, hop_buf, sizeof(hop_buf));
			snprintf(source_with_hops, sizeof(source_with_hops), "%s %s", _msg_channel, hop_buf);

			return renderMessageDetailView(display, _msg_scroll, total, ts,
					source_with_hops, "", msg_safe,
					_msg_detail_scroll);
		}

		/* Message list */
		renderScreenHeader(display, _msg_channel, _msg_scroll, total);

		const int ROW_H = 10, LINES_PER_MSG = 2;
		int visible = getMessagePreviewVisibleCount(display, ROW_H, LINES_PER_MSG, kContentY);
		int start = getCenteredMessagePreviewStart(_msg_scroll, total, visible);

		int y = kContentY;
		for (int i = start; i < total && y < display.height() - 2; i++) {
			const char *msg = "";
			uint32_t ts = 0;
			_task->getChannelPreviewFor(_msg_channel, i, msg, ts);
			char msg_safe[96];
			sanitizeForDisplay(msg, msg_safe, sizeof(msg_safe));
			char time_buf[12];
			formatClockHM(ts, time_buf, sizeof(time_buf));
			y = renderMessagePreviewEntry(display, y, i == _msg_scroll,
					time_buf, msg_safe, ROW_H, LINES_PER_MSG, 20, 10, 4);
		}
		return 400;
	}

	/* Channel list */
	int ch_count = getChannelCount();
	int item_count = ch_count + 3;  /* channels + Join #channel + Join private + Create private */

	if (_selected >= item_count) _selected = item_count - 1;
	if (_selected < 0) _selected = 0;

	renderScreenHeader(display, "Channels", _selected, item_count);
	int start = computeListStart(_selected, item_count);
	int y = kContentY;
	for (int i = start; i < item_count && i < start + UI_RECENT_LIST_SIZE; i++) {
		char line[40];
		if (i < ch_count) {
			ChannelDetails ch;
			if (!getChannelByListIndex(i, ch)) continue;
			display.translateUTF8ToBlocks(line, ch.name, sizeof(line));
		} else if (i == ch_count) {
			snprintf(line, sizeof(line), "+ Join #channel");
		} else if (i == ch_count + 1) {
			snprintf(line, sizeof(line), "+ Join private");
		} else {
			snprintf(line, sizeof(line), "+ Create private");
		}
		renderMenuListText(display, y, i == _selected, line, true);
		y += kMenuLineH;
	}

	display.setTextSize(1);
	display.setColor(JoystickDisplay::LIGHT);
	return 500;
}

bool ChannelsScreen::handleInput(char key)
{
	/* Message view input */
	if (_show_msgs) {
		int total = _task->getChannelPreviewCountFor(_msg_channel);

		if (_msg_details) {
			const char *msg = "";
			uint32_t ts = 0;
			uint8_t path_len = OUT_PATH_UNKNOWN;
			int max_scroll = 0;
			if (_task->getChannelPreviewFor(_msg_channel, _msg_scroll, msg, ts, &path_len)) {
				max_scroll = getMessageDetailMaxScrollSanitized(
					_task->getDisplay(), msg, 20, 5, 5);
			}
			if (handleDetailScrollNavigation(key, _msg_detail_scroll, max_scroll)) return true;
			if (key == KEY_CANCEL || key == KEY_HOME || key == KEY_ENTER) {
				_msg_details = false;
				_msg_detail_scroll = 0;
				return true;
			}
			if (key == KEY_ENTER_LONG) {
				_task->setComposeChannel(_msg_channel_idx, _msg_channel);
				char at_buf[38];
				if (buildChannelReplyPrefix(msg, path_len,
						at_buf, sizeof(at_buf))) {
					_task->gotoT9InputScreenWithPrefix(at_buf);
				} else {
					_task->gotoT9InputScreen();
				}
				return true;
			}
			return false;
		}

		if (key == KEY_UP && total > 0) {
			_msg_scroll = (_msg_scroll + total - 1) % total;
			return true;
		}
		if (key == KEY_DOWN && total > 0) {
			_msg_scroll = (_msg_scroll + 1) % total;
			return true;
		}
		if (key == KEY_TO_TOP && total > 0)    { _msg_scroll = 0;          return true; }
		if (key == KEY_TO_BOTTOM && total > 0) { _msg_scroll = total - 1; return true; }
		if (key == KEY_ENTER && total > 0) {
			_msg_details = true;
			_msg_detail_scroll = 0;
			return true;
		}
		if (key == KEY_ENTER_LONG) {
			_task->setComposeChannel(_msg_channel_idx, _msg_channel);
			_task->gotoT9InputScreen();
			return true;
		}
		if (key == KEY_CANCEL || key == KEY_HOME) {
			_show_msgs = false;
			_msg_details = false;
			return true;
		}
		return false;
	}

	/* Channel list */
	int ch_count = getChannelCount();
	int item_count = ch_count + 3;

	if (handleCommonListNavigation(key, _selected, item_count)) return true;

	if (key == KEY_ENTER_LONG && _selected < ch_count) {
		ChannelDetails ch;
		int slot = -1;
		if (getChannelByListIndex(_selected, ch, &slot)) {
			_task->setComposeChannel(slot, ch.name);
			_task->gotoT9InputScreen();
		}
		return true;
	}
	if (key == KEY_ENTER) {
		if (_selected < ch_count) {
			ChannelDetails ch;
			int slot = -1;
			if (getChannelByListIndex(_selected, ch, &slot)) {
				strncpy(_msg_channel, ch.name, sizeof(_msg_channel) - 1);
				_msg_channel[sizeof(_msg_channel) - 1] = '\0';
				_msg_channel_idx = slot;
				_msg_scroll = 0;
				_msg_details = false;
				_msg_detail_scroll = 0;
				_show_msgs = true;
			}
			return true;
		}
		if (_selected == ch_count) {
			_task->setComposeChannel(-2, "+Join #channel");
			_task->gotoT9InputScreen();
			return true;
		}
		if (_selected == ch_count + 1) {
			_task->setComposeChannel(-3, "+Join private");
			_task->gotoT9InputScreen();
			return true;
		}
		_task->setComposeChannel(-4, "+Create private");
		_task->gotoT9InputScreen();
		return true;
	}
	if (key == KEY_CANCEL || key == KEY_HOME) { _task->gotoHomeScreen(); return true; }
	return false;
}
