/*
 * ZephCore - Joystick UI Contacts Screen
 * Copyright (c) 2026 ZephCore
 * SPDX-License-Identifier: MIT
 */

#include "../joystick_screens.h"
#include "../joystick_ui_task.h"
#include "../joystick_ui_hooks.h"
#include "screen_helpers.h"
#include <helpers/AdvertDataHelpers.h>
#include <helpers/BaseChatMesh.h>
#include <mesh/Utils.h>
#include <helpers/ContactInfo.h>
#include <helpers/ui/ui_task.h>
#include <adapters/gps/ZephyrGPSManager.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const int CMODE_LIST = 0;
static const int CMODE_SUBMENU = 1;
static const int CMODE_MSGVIEW = 2;
static const int CMODE_EDITPATH = 3;

ContactsScreen::ContactsScreen(JoystickUITask *task, mesh::RTCClock *rtc)
	: _task(task), _rtc(rtc),
	  _selected(0), _filter(0),
	  _mode(CMODE_LIST),
	  _submenu_selected(0),
	  _chat_selected(0), _chat_details(false), _chat_detail_scroll(0),
	  _active_contact_valid(false),
	  _idx_send_message(-1), _idx_edit_path(-1), _idx_reset_path(-1), _idx_favorite(-1),
	  _idx_delete(-1), _idx_repeater_admin(-1), _idx_ping_zerohop(-1),
	  _editpath_cursor(0),
	  _editpath_t9_sel(0), _editpath_t9_last_key(-1),
	  _editpath_t9_letter_index(0), _editpath_t9_last_press(0),
	  _editpath_kb_letters(true),
	  _editpath_confirm_exit(false), _editpath_confirm_sel(0),
	  _header_marquee_ms(0),
	  _ping_sent_at(0), _ping_timeout_ms(0),
	  _ping_snr_local(0), _ping_snr_remote(INT8_MIN), _ping_rtt_ms(0),
	  _ping_modal_active(false)
{
	_active_contact = ContactInfo{};
	memset(_editpath_hexbuf, 0, sizeof(_editpath_hexbuf));
	k_timer_init(&_ping_timeout_timer, pingTimeoutCb, NULL);
	k_timer_user_data_set(&_ping_timeout_timer, this);
}

void ContactsScreen::pingTimeoutCb(struct k_timer *t)
{
	/* ISR — just wake the main loop; render() handles the timeout. */
	auto *self = static_cast<ContactsScreen *>(k_timer_user_data_get(t));
	if (self && self->_task) self->_task->notify();
}

void ContactsScreen::onExit()
{
	k_timer_stop(&_ping_timeout_timer);
}

/* Filter helpers */

static const char * const kFilterLabels[4] = {"Favorite", "Users", "Repeaters", "Room"};

static bool matchesFilter(const ContactInfo &c, int filter)
{
	switch (filter & 3) {
	case 0: return (c.flags & 0x01) != 0;
	case 1: return c.type == ADV_TYPE_CHAT;
	case 2: return c.type == ADV_TYPE_REPEATER;
	default: return c.type == ADV_TYPE_ROOM;
	}
}

int ContactsScreen::getFilteredContactCount() const
{
	int count = 0;
	int total = _task->getMesh()->getNumContacts();
	ContactInfo c;
	for (int i = 0; i < total; i++) {
		if (!_task->getMesh()->getContactByIdx(i, c)) continue;
		if (matchesFilter(c, _filter)) count++;
	}
	return count;
}

bool ContactsScreen::getFilteredContactByIndex(int listIndex, ContactInfo &contact) const
{
	int matched = 0;
	int total = _task->getMesh()->getNumContacts();
	for (int i = 0; i < total; i++) {
		if (!_task->getMesh()->getContactByIdx(i, contact)) continue;
		if (!matchesFilter(contact, _filter)) continue;
		if (matched == listIndex) return true;
		matched++;
	}
	return false;
}

bool ContactsScreen::refreshActiveContact()
{
	if (!_active_contact_valid) return false;
	ContactInfo *live = _task->getMesh()->lookupContactByPubKey(_active_contact.id.pub_key, PUB_KEY_SIZE);
	if (!live) { _active_contact_valid = false; return false; }
	_active_contact = *live;
	return true;
}

int ContactsScreen::clampStart(int contactCount) const
{
	return computeListStart(_selected, contactCount);
}

void ContactsScreen::openForPubKey(const uint8_t *prefix, int prefix_len)
{
	int total = _task->getMesh()->getNumContacts();

	/* Determine the right filter from the contact's type */
	_filter = 1; /* default to Users */
	for (int i = 0; i < total; i++) {
		ContactInfo c;
		if (!_task->getMesh()->getContactByIdx(i, c)) continue;
		if (memcmp(c.id.pub_key, prefix, prefix_len) != 0) continue;
		if (c.type == ADV_TYPE_REPEATER)  _filter = 2;
		else if (c.type == ADV_TYPE_ROOM) _filter = 3;
		else _filter = 1;
		break;
	}

	_mode = CMODE_LIST;
	_selected = 0;
	_ping_modal_active = false;
	_ping_sent_at = 0;
	_ping_rtt_ms = 0;

	/* Find the list index under the chosen filter */
	int list_idx = 0;
	for (int i = 0; i < total; i++) {
		ContactInfo c;
		if (!_task->getMesh()->getContactByIdx(i, c)) continue;
		if (!matchesFilter(c, _filter)) continue;
		if (memcmp(c.id.pub_key, prefix, prefix_len) == 0) {
			_selected = list_idx + 2;
			break;
		}
		list_idx++;
	}
}

/* Submenu builder */

int ContactsScreen::buildSubmenuItems(const char *items[], char text[][48], int max_items)
{
	int n = 0;
	_idx_send_message = _idx_repeater_admin = _idx_reset_path = -1;
	_idx_edit_path = _idx_favorite = _idx_delete = _idx_ping_zerohop = -1;

	/* GPS location row — shown only when contact has a known position */
	bool contact_has_pos = (_active_contact.gps_lat != 0 || _active_contact.gps_lon != 0);
	if (contact_has_pos && n < max_items) {
		struct gps_position pos;
		gps_get_position(&pos);
		bool own_fix = _task->getGPSState() && pos.valid;

		if (own_fix) {
			int32_t own_lat = (int32_t)(pos.latitude_ndeg / 1000LL);
			int32_t own_lon = (int32_t)(pos.longitude_ndeg / 1000LL);
			float dist_m = gpsDistanceM(own_lat, own_lon,
										_active_contact.gps_lat, _active_contact.gps_lon);
			float bearing = gpsBearingDeg(own_lat, own_lon,
										  _active_contact.gps_lat, _active_contact.gps_lon);
			const char *dir = compassDir(bearing);
			if (dist_m < 1000.0f) {
				snprintf(text[n], 48, "%s %dm", dir, (int)(dist_m + 0.5f));
			} else {
				int km10 = (int)((dist_m + 50.0f) / 100.0f);
				snprintf(text[n], 48, "%s %d.%dkm", dir, km10 / 10, km10 % 10);
			}
		} else {
			int32_t lat = _active_contact.gps_lat;
			int32_t lon = _active_contact.gps_lon;
			char ns = (lat < 0) ? 'S' : 'N';
			char ew = (lon < 0) ? 'W' : 'E';
			if (lat < 0) lat = -lat;
			if (lon < 0) lon = -lon;
			snprintf(text[n], 48, "%c%ld.%03ld %c%ld.%03ld",
					 ns, (long)(lat / 1000000L), (long)((lat % 1000000L) / 1000),
					 ew, (long)(lon / 1000000L), (long)((lon % 1000000L) / 1000));
		}
		items[n] = text[n];
		n++;
	}

	/* Public key prefix — display only */
	if (n < max_items) {
		snprintf(text[n], 48, "Key: %02X%02X%02X",
				 _active_contact.id.pub_key[0],
				 _active_contact.id.pub_key[1],
				 _active_contact.id.pub_key[2]);
		items[n] = text[n];
		n++;
	}

	if (_active_contact.type == ADV_TYPE_CHAT) {
		if (n < max_items) { _idx_send_message = n; items[n++] = "[>] Messages"; }
	} else if (_active_contact.type == ADV_TYPE_REPEATER) {
		if (n < max_items) { _idx_ping_zerohop = n; items[n++] = "[>] Ping (0 hop)"; }
		if (n < max_items) { _idx_repeater_admin = n; items[n++] = "[>] Admin"; }
	}

	if (n < max_items) { _idx_edit_path = n; items[n++] = "[~] Edit path"; }
	if (n < max_items) { _idx_reset_path = n; items[n++] = "[>] Reset path"; }
	if (n < max_items) {
		_idx_favorite = n;
		snprintf(text[n], 48, "[~] Favourite: %s", (_active_contact.flags & 0x01) ? "yes" : "no");
		items[n] = text[n];
		n++;
	}
	if (n < max_items) { _idx_delete = n; items[n++] = "[x] Delete contact"; }
	return n;
}

/* Ping modal overlay */

static void drawPingModal(JoystickDisplay &display,
		uint32_t ping_sent_at, uint32_t ping_rtt_ms,
		int8_t snr_local, int8_t snr_remote)
{
	int fw = display.fontW(), fh = display.fontH();
	int w = display.width(), h = display.height();
	int pad = 4;
	int box_h = 2 * fh + 10;
	int box_w = w - 2 * pad;
	int box_x = pad, box_y = (h - box_h) / 2;

	mc_display_fill_rect(box_x, box_y, box_w, box_h);
	mc_display_invert_rect(box_x, box_y, box_w, box_h);
	display.setColor(JoystickDisplay::WHITE);
	display.drawRect(box_x, box_y, box_w, box_h);

	int y0 = box_y + 4;
	int cx = box_x + box_w / 2;

	if (ping_sent_at > 0) {
		uint32_t elapsed = k_uptime_get_32() - ping_sent_at;
		const char *spin[] = { "Pinging.", "Pinging..", "Pinging..." };
		const char *s = spin[(elapsed / 333) % 3];
		mc_display_text(cx - (int)strlen(s) * fw / 2, y0, s, false);
		const char *hint = "CANCEL to abort";
		mc_display_text(cx - (int)strlen(hint) * fw / 2, y0 + fh + 3, hint, false);
	} else if (ping_rtt_ms == 0) {
		/* Packet queued but RF TX not yet complete — show static first frame */
		const char *s = "Pinging.";
		mc_display_text(cx - (int)strlen(s) * fw / 2, y0, s, false);
		const char *hint = "CANCEL to abort";
		mc_display_text(cx - (int)strlen(hint) * fw / 2, y0 + fh + 3, hint, false);
	} else if (ping_rtt_ms == UINT32_MAX) {
		mc_display_text(cx - 3 * fw, y0, "TIMEOUT", false);
		const char *hint = "OK to close";
		mc_display_text(cx - (int)strlen(hint) * fw / 2, y0 + fh + 3, hint, false);
	} else {
		char line1[24], line2[20];
		if (snr_remote != INT8_MIN) {
			snprintf(line1, sizeof(line1), ">%+ddB  <%+ddB",
					 (int)snr_remote, (int)snr_local);
		} else {
			snprintf(line1, sizeof(line1), "<%+ddB", (int)snr_local);
		}
		snprintf(line2, sizeof(line2), "RTT: %ums", (unsigned)ping_rtt_ms);
		mc_display_text(cx - (int)strlen(line1) * fw / 2, y0, line1, false);
		mc_display_text(cx - (int)strlen(line2) * fw / 2, y0 + fh + 3, line2, false);
	}
}

int ContactsScreen::render(JoystickDisplay &display)
{
	/* Ping timeout: timer fires when _ping_timeout_ms elapsed; mark timeout. */
	if (_ping_sent_at > 0 && _ping_timeout_ms > 0 &&
		(k_uptime_get_32() - _ping_sent_at) >= _ping_timeout_ms) {
		_ping_sent_at = 0;
		_ping_rtt_ms = UINT32_MAX;
	}

	if (_mode == CMODE_SUBMENU || _mode == CMODE_MSGVIEW) {
		if (!refreshActiveContact()) {
			_mode = CMODE_LIST;
			_task->showAlert("Contact removed", 800);
		}
	}

	/* Message view */
	if (_mode == CMODE_MSGVIEW && _active_contact_valid) {
		int total = _task->getContactMsgCount(_active_contact.name);

		if (total <= 0) {
			renderScrollingScreenHeader(display, _active_contact.name, 0, 0, _header_marquee_ms);
			display.setColor(JoystickDisplay::YELLOW);
			display.drawTextCentered(display.width() / 2, 24, "No messages");
			display.setColor(JoystickDisplay::LIGHT);
			display.drawTextCentered(display.width() / 2, 38, "Long ENTER to send");
			display.drawTextCentered(display.width() / 2, 49, "CANCEL back");
			return 1000;
		}

		if (_chat_selected >= total) _chat_selected = total - 1;
		if (_chat_selected < 0)     _chat_selected = 0;

		if (_chat_details) {
			const char *msg = nullptr;
			uint32_t ts = 0;
			uint8_t path_len = OUT_PATH_UNKNOWN;
			if (!_task->getContactMsgAt(_active_contact.name, _chat_selected, msg, ts, &path_len)) {
				_chat_details = false;
				return 200;
			}
			char msg_safe[480];
			sanitizeForDisplay(msg, msg_safe, sizeof(msg_safe));
			char route_line[48];
			buildMessageRouteLineForName("contact", _active_contact.name, path_len,
										 route_line, sizeof(route_line));
			return renderMessageDetailView(display, _chat_selected, total, ts,
										   _active_contact.name, route_line, msg_safe,
										   _chat_detail_scroll);
		}

		renderScrollingScreenHeader(display, _active_contact.name, _chat_selected, total,
									_header_marquee_ms);
		display.setTextSize(1);
		const int ROW_H = 10, LINES_PER_MSG = 2;
		int visible = getMessagePreviewVisibleCount(display, ROW_H, LINES_PER_MSG, kContentY);
		int list_scroll = getCenteredMessagePreviewStart(_chat_selected, total, visible);
		int y = kContentY;
		for (int i = list_scroll; i < total && y < display.height() - 2; i++) {
			const char *msg = nullptr;
			uint32_t ts = 0;
			uint8_t path_len = OUT_PATH_UNKNOWN;
			if (!_task->getContactMsgAt(_active_contact.name, i, msg, ts, &path_len)) continue;
			char time_buf[12], msg_safe[192];
			formatClockHM(ts, time_buf, sizeof(time_buf));
			sanitizeForDisplay(msg, msg_safe, sizeof(msg_safe));
			y = renderMessagePreviewEntry(display, y, i == _chat_selected,
										  time_buf, msg_safe, ROW_H, LINES_PER_MSG, 20, 10, 4);
		}
		return 400;
	}

	/* Submenu */
	if (_mode == CMODE_SUBMENU && _active_contact_valid) {
		const char *items[12];
		char text[12][48];
		int n = buildSubmenuItems(items, text, 12);
		if (n <= 0) { _mode = CMODE_LIST; return 300; }

		if (_submenu_selected >= n) _submenu_selected = n - 1;
		if (_submenu_selected < 0) _submenu_selected = 0;

		renderScrollingScreenHeader(display, _active_contact.name, _submenu_selected, n,
									_header_marquee_ms);
		int start = computeListStart(_submenu_selected, n);
		int y = kContentY + 2;
		for (int i = start; i < n && i < start + UI_RECENT_LIST_SIZE; i++) {
			char safe[48];
			sanitizeForDisplay(items[i], safe, sizeof(safe));
			renderMenuListText(display, y, i == _submenu_selected, safe);
			y += kMenuLineH;
		}
		if (_ping_modal_active) {
			drawPingModal(display, _ping_sent_at, _ping_rtt_ms,
						  _ping_snr_local, _ping_snr_remote);
			return (_ping_sent_at > 0) ? 150 : 400;
		}
		return 400;
	}

	/* Edit path */
	if (_mode == CMODE_EDITPATH && _active_contact_valid) {
		if (_editpath_confirm_exit) {
			renderScreenHeader(display, "Edit path", _editpath_confirm_sel, 2);
			display.setTextSize(1);
			display.setColor(JoystickDisplay::LIGHT);
			display.drawTextLeftAlign(0, kContentY + 2, "Save changes?");
			static const char * const kOpts[2] = { "save", "discard" };
			int y = kContentY + 14;
			for (int i = 0; i < 2; i++) {
				if (i == _editpath_confirm_sel) {
					display.setColor(JoystickDisplay::YELLOW);
					display.drawTextLeftAlign(0, y, "> ");
					display.drawTextLeftAlign(10, y, kOpts[i]);
				} else {
					display.setColor(JoystickDisplay::GREEN);
					display.drawTextLeftAlign(0, y, kOpts[i]);
				}
				y += kMenuLineH;
			}
			return 300;
		}
		renderScreenHeader(display, "Edit path", 0, 0);
		display.setTextSize(1);
		char disp_buf[MAX_PATH_SIZE * 2 + 3];
		snprintf(disp_buf, sizeof(disp_buf), "%s|", _editpath_hexbuf);
		display.setColor(JoystickDisplay::YELLOW);
		display.drawTextEllipsized(0, kContentY, display.width(), disp_buf);
		renderT9Keypad(display, getT9KeyLabels(_editpath_kb_letters, true),
					   _editpath_t9_sel, kContentY + 12);
		return 300;
	}

	/* Contact list */
	int contactCount = getFilteredContactCount();
	int itemCount = 2 + contactCount;

	renderScreenHeader(display, "Contacts", _selected, itemCount);

	if (_selected >= itemCount) _selected = itemCount - 1;
	if (_selected < 0)         _selected = 0;

	int start = clampStart(itemCount);
	int y = kContentY + 2;
	for (int i = start; i < itemCount && i < start + UI_RECENT_LIST_SIZE; i++) {
		char line[48];
		if (i == 0) {
			snprintf(line, sizeof(line), "Settings");
		} else if (i == 1) {
			snprintf(line, sizeof(line), "Filter: %s", kFilterLabels[_filter & 3]);
		} else {
			ContactInfo c;
			if (!getFilteredContactByIndex(i - 2, c)) continue;
			display.translateUTF8ToBlocks(line, c.name, sizeof(line));
		}
		renderMenuListText(display, y, i == _selected, line, i >= 2, 450, _header_marquee_ms);
		if (i == 1) {
			display.setColor(JoystickDisplay::LIGHT);
			display.drawRect(0, y + kLineH, display.width(), 1);
		}
		y += kMenuLineH;
	}

	if (contactCount == 0) {
		display.setColor(JoystickDisplay::LIGHT);
		display.drawTextCentered(display.width() / 2, display.height() - 10, "No match");
	}
	return 500;
}

/* Message view input: handleInput() in CMODE_MSGVIEW. */
bool ContactsScreen::handleMsgViewKey(char key)
{
	if (key == KEY_ENTER_LONG && _active_contact_valid &&
		_active_contact.type == ADV_TYPE_CHAT) {
		_task->setComposeContact(_active_contact);
		_task->gotoT9InputScreen();
		return true;
	}
	if (_chat_details) {
		const char *msg = nullptr;
		uint32_t ts = 0;
		int max_scroll = 0;
		if (_active_contact_valid &&
			_task->getContactMsgAt(_active_contact.name, _chat_selected, msg, ts)) {
			JoystickDisplay &disp = _task->getDisplay();
			max_scroll = getMessageDetailMaxScrollSanitized(disp, msg, 20, 5, 4);
		}
		if (handleDetailScrollNavigation(key, _chat_detail_scroll, max_scroll)) return true;
		if (key == KEY_ENTER || key == KEY_CANCEL || key == KEY_HOME) {
			_chat_details = false;
			_chat_detail_scroll = 0;
			return true;
		}
		return false;
	}

	int total = _task->getContactMsgCount(_active_contact.name);
	if (total > 0 && handleCommonListNavigation(key, _chat_selected, total)) return true;
	if (key == KEY_ENTER && total > 0) { _chat_details = true; _chat_detail_scroll = 0; return true; }
	if (key == KEY_CANCEL || key == KEY_HOME) { _mode = CMODE_SUBMENU; return true; }
	return false;
}

/* Submenu input: handleInput() in CMODE_SUBMENU. */
bool ContactsScreen::handleSubmenuKey(char key)
{
	/* Ping modal intercepts all input when active */
	if (_ping_modal_active) {
		if (_ping_sent_at > 0 || _ping_rtt_ms == 0) {
			/* In flight (TX done or still queued): CANCEL aborts */
			if (key == KEY_CANCEL || key == KEY_HOME) {
				_task->cancelContactPing();
				_ping_sent_at = 0;
				_ping_rtt_ms = 0;
				_ping_modal_active = false;
				_task->forceRefresh();
			}
		} else {
			/* Result/timeout: any key dismisses */
			_ping_modal_active = false;
			_task->forceRefresh();
		}
		return true;
	}

	const char *items[12];
	char text[12][48];
	int n = buildSubmenuItems(items, text, 12);

	if (handleCommonListNavigation(key, _submenu_selected, n)) return true;

	if (key == KEY_ENTER) {
		if (_submenu_selected == _idx_send_message) {
			_chat_selected = 0;
			_chat_details = false;
			_chat_detail_scroll = 0;
			_header_marquee_ms = k_uptime_get_32();
			_mode = CMODE_MSGVIEW;
			return true;
		}
		if (_submenu_selected == _idx_repeater_admin) {
			_task->gotoRepeaterAdminScreen(_active_contact.id.pub_key,
										  _active_contact.name, true);
			return true;
		}
		if (_submenu_selected == _idx_ping_zerohop) {
			uint32_t est_timeout = 0;
			_ping_rtt_ms = 0;
			_ping_snr_remote = INT8_MIN;
			if (_task->sendContactPingZeroHop(_active_contact, est_timeout)) {
				_ping_sent_at = 0; /* set in onPacketSent() after RF TX */
				_ping_timeout_ms = 5000;
				_ping_modal_active = true;
			} else {
				_task->showAlert("Ping failed", 700);
			}
			return true;
		}
		if (_submenu_selected == _idx_edit_path) {
			ContactInfo *live = _task->getMesh()->lookupContactByPubKey(
				_active_contact.id.pub_key, PUB_KEY_SIZE);
			const ContactInfo *src = live ? live : &_active_contact;
			memset(_editpath_hexbuf, 0, sizeof(_editpath_hexbuf));
			int hlen = 0;
			/* out_path_len is packed (top 2 bits = hash_size-1, bottom 6 = hop
			 * count) — decode the real byte length instead of treating the
			 * encoded byte as a raw count (breaks for path_hash_mode > 0). */
			int nbytes = 0;
			if (mesh::Packet::isValidPathLen(src->out_path_len)) {
				uint8_t hash_size = (src->out_path_len >> 6) + 1;
				nbytes = (src->out_path_len & 63) * hash_size;
			}
			for (int i = 0; i < nbytes && hlen < (int)sizeof(_editpath_hexbuf) - 2; i++) {
				snprintf(_editpath_hexbuf + hlen, 3, "%02X", src->out_path[i]);
				hlen += 2;
			}
			_editpath_cursor = hlen;
			_editpath_t9_sel = 0;
			_editpath_t9_last_key = -1;
			_editpath_t9_letter_index = 0;
			_editpath_t9_last_press = 0;
			_editpath_kb_letters = true;
			_editpath_confirm_exit = false;
			_editpath_confirm_sel = 0;
			_mode = CMODE_EDITPATH;
			return true;
		}
		if (_submenu_selected == _idx_reset_path) {
			ContactInfo *live = _task->getMesh()->lookupContactByPubKey(
				_active_contact.id.pub_key, PUB_KEY_SIZE);
			if (live) {
				/* OUT_PATH_UNKNOWN (0xFF) marks "no saved path → flood".
				 * Plain 0 would mean "0-hop direct" and the contact would
				 * render as "direct" instead of forcing a flood-rediscover. */
				live->out_path_len = OUT_PATH_UNKNOWN;
				memset(live->out_path, 0, sizeof(live->out_path));
				if (CompanionMesh *cm = static_cast<CompanionMesh *>(_task->getMesh())) {
					cm->markContactsDirtyPublic();
				}
			}
			_task->showAlert("Path reset", 800);
			return true;
		}
		if (_submenu_selected == _idx_favorite) {
			ContactInfo c = _active_contact;
			c.flags ^= 0x01;
			ContactInfo old = _active_contact;
			auto *mesh = _task->getMesh();
			if (mesh->removeContact(old)) {
				mesh->addContact(c);
				_active_contact = c;
			}
			return true;
		}
		if (_submenu_selected == _idx_delete) {
			ContactInfo c = _active_contact;
			_task->getMesh()->removeContact(c);
			_active_contact_valid = false;
			_mode = CMODE_LIST;
			_task->showAlert("Deleted", 800);
			return true;
		}
	}
	if (key == KEY_CANCEL || key == KEY_HOME) { _mode = CMODE_LIST; return true; }
	return false;
}

/* Edit path input: handleInput() in CMODE_EDITPATH. */
bool ContactsScreen::handleEditPathKey(char key)
{
	if (_editpath_confirm_exit) {
		if (key == KEY_UP || key == KEY_DOWN) {
			_editpath_confirm_sel = (_editpath_confirm_sel + 1) % 2;
			return true;
		}
		if (key == KEY_ENTER) {
			if (_editpath_confirm_sel == 0) {
				/* save */
				ContactInfo *live = _task->getMesh()->lookupContactByPubKey(
					_active_contact.id.pub_key, PUB_KEY_SIZE);
				if (live) {
					int hlen = (int)strlen(_editpath_hexbuf);
					int nbytes = hlen / 2;
					/* Encode with the mesh's *current* hash size, not always
					 * hash_size=1 — a path saved as 1-byte hops would silently
					 * mismatch every relay's isHashMatch() once path_hash_mode>0. */
					uint8_t hash_size = _task->getPathHashBytes();
					int max_bytes = hash_size * 63;
					if (max_bytes > MAX_PATH_SIZE) max_bytes = MAX_PATH_SIZE;
					if (nbytes > max_bytes) nbytes = max_bytes;
					if (nbytes % hash_size != 0) {
						_task->showAlert("Bad path length", 800);
					} else {
						for (int i = 0; i < nbytes; i++) {
							char b[3] = { _editpath_hexbuf[i*2], _editpath_hexbuf[i*2+1], '\0' };
							live->out_path[i] = (uint8_t)strtol(b, nullptr, 16);
						}
						uint8_t hop_count = (uint8_t)(nbytes / hash_size);
						live->out_path_len = (uint8_t)(((hash_size - 1) << 6) | (hop_count & 0x3F));
						_active_contact = *live;
						_task->showAlert(nbytes == 0 ? "Path cleared" : "Path saved", 800);
					}
				}
			}
			_editpath_confirm_exit = false;
			_mode = CMODE_SUBMENU;
			return true;
		}
		if (key == KEY_CANCEL || key == KEY_HOME) {
			_editpath_confirm_exit = false;
			return true;
		}
		return false;
	}
	if (handleT9DirectionalInput(key, _editpath_t9_sel)) return true;
	if (key == KEY_ENTER) {
		if (_editpath_t9_sel == 3) {  /* del */
			if (_editpath_cursor > 0) _editpath_hexbuf[--_editpath_cursor] = '\0';
			resetT9State(_editpath_t9_last_key, _editpath_t9_letter_index, _editpath_t9_last_press);
			return true;
		}
		if (_editpath_t9_sel == 11) {  /* save */
			ContactInfo *live = _task->getMesh()->lookupContactByPubKey(
				_active_contact.id.pub_key, PUB_KEY_SIZE);
			if (live) {
				int hlen = (int)strlen(_editpath_hexbuf);
				int nbytes = hlen / 2;
				/* Encode with the mesh's *current* hash size, not always
				 * hash_size=1 — a path saved as 1-byte hops would silently
				 * mismatch every relay's isHashMatch() once path_hash_mode>0. */
				uint8_t hash_size = _task->getPathHashBytes();
				int max_bytes = hash_size * 63;
				if (max_bytes > MAX_PATH_SIZE) max_bytes = MAX_PATH_SIZE;
				if (nbytes > max_bytes) nbytes = max_bytes;
				if (nbytes % hash_size != 0) {
					_task->showAlert("Bad path length", 800);
				} else {
					for (int i = 0; i < nbytes; i++) {
						char b[3] = { _editpath_hexbuf[i*2], _editpath_hexbuf[i*2+1], '\0' };
						live->out_path[i] = (uint8_t)strtol(b, nullptr, 16);
					}
					uint8_t hop_count = (uint8_t)(nbytes / hash_size);
					live->out_path_len = (uint8_t)(((hash_size - 1) << 6) | (hop_count & 0x3F));
					_active_contact = *live;
					_task->showAlert(nbytes == 0 ? "Path cleared" : "Path saved", 800);
				}
			}
			_mode = CMODE_SUBMENU;
			return true;
		}
		if (_editpath_t9_sel == 12) {  /* mode toggle */
			_editpath_kb_letters = !_editpath_kb_letters;
			resetT9State(_editpath_t9_last_key, _editpath_t9_letter_index, _editpath_t9_last_press);
			return true;
		}
		if (_editpath_t9_sel == 7 || _editpath_t9_sel == 15) return true;  /* spc/unused */
		const char *letters = getT9KeyLetters(_editpath_kb_letters)[_editpath_t9_sel];
		if (letters && letters[0]) {
			appendOrCycleT9Char(_editpath_hexbuf, _editpath_cursor,
								sizeof(_editpath_hexbuf), letters,
								_editpath_t9_sel, _editpath_t9_last_key,
								_editpath_t9_letter_index, _editpath_t9_last_press);
		}
		return true;
	}
	if (key == KEY_CANCEL || key == KEY_HOME) {
		if (_editpath_cursor > 0) {
			_editpath_confirm_exit = true;
			_editpath_confirm_sel = 0;
		} else {
			_mode = CMODE_SUBMENU;
		}
		return true;
	}
	return false;
}

bool ContactsScreen::handleInput(char key)
{
	if (_mode == CMODE_MSGVIEW) {
		return handleMsgViewKey(key);
	}

	if (_mode == CMODE_SUBMENU) {
		return handleSubmenuKey(key);
	}

	if (_mode == CMODE_EDITPATH) {
		return handleEditPathKey(key);
	}

	/* Contact list input */
	int contactCount = getFilteredContactCount();
	int itemCount = 2 + contactCount;

	if (handleCommonListNavigation(key, _selected, itemCount)) return true;

	if (key == KEY_ENTER) {
		if (_selected == 0) { return true; } /* Settings placeholder — no action yet */
		if (_selected == 1) {
			_filter = (_filter + 1) % 4;
			_selected = 1;
			return true;
		}
		ContactInfo c;
		if (getFilteredContactByIndex(_selected - 2, c)) {
			_active_contact = c;
			_active_contact_valid = true;
			_submenu_selected = 0;
			_header_marquee_ms = k_uptime_get_32();
			_mode = CMODE_SUBMENU;
		}
		return true;
	}
	if (key == KEY_CANCEL || key == KEY_HOME) {
		_task->gotoHomeScreen();
		return true;
	}
	return false;
}

void ContactsScreen::onPingResponse(int8_t snr_local, int8_t snr_remote, uint32_t rtt_ms)
{
	_ping_snr_local = snr_local;
	_ping_snr_remote = snr_remote;
	_ping_rtt_ms = (rtt_ms == 0) ? 1 : rtt_ms;
	_ping_sent_at = 0;
	_ping_modal_active = true;
	k_timer_stop(&_ping_timeout_timer);  /* got a response before timeout */
	_task->forceRefresh();
}

void ContactsScreen::onPacketSent()
{
	/* RF TX just completed — start the ping timeout clock now. */
	if (_ping_modal_active && _ping_sent_at == 0) {
		_ping_sent_at = k_uptime_get_32();
		if (_ping_timeout_ms > 0) {
			k_timer_start(&_ping_timeout_timer, K_MSEC(_ping_timeout_ms), K_NO_WAIT);
		}
		_task->forceRefresh();
	}
}
