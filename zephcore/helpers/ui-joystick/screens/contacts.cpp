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

bool ContactsScreen::handleInput(char key)
{
	/* Message view input */
	if (_mode == CMODE_MSGVIEW) {
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

	/* Submenu input */
	if (_mode == CMODE_SUBMENU) {
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

	/* Edit path input */
	if (_mode == CMODE_EDITPATH) {
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
		/* CayenneLPP tuples: [channel:1][type:1][data:N big endian] */
		char *dp = text;
		char *end = text + sizeof(text) - 1;
		int offset = 0;
		bool parse_ok = true;
		while (parse_ok && offset + 2 <= data_len && dp < end - 22) {
			uint8_t type = data[offset + 1];
			offset += 2;
			char *entry_start = dp;
			if (dp > text) *dp++ = '\n';
			bool entry_ok = false;
			switch (type) {
			case 116: { /* VOLTAGE: 2B unsigned BE ÷100 = V */
				if (offset + 2 > data_len) break;
				uint16_t raw = ((uint16_t)data[offset] << 8) | data[offset + 1];
				offset += 2;
				dp += snprintf(dp, end - dp, "batt:%u.%02uV", raw / 100, raw % 100);
				entry_ok = true;
				break;
			}
			case 103: { /* TEMPERATURE: 2B signed BE ÷10 = °C */
				if (offset + 2 > data_len) break;
				int16_t raw = ((int16_t)data[offset] << 8) | data[offset + 1];
				offset += 2;
				int abst = (raw < 0) ? -(int)raw : (int)raw;
				dp += snprintf(dp, end - dp, "temp:%s%d.%dC", (raw < 0 ? "-" : ""), abst / 10, abst % 10);
				entry_ok = true;
				break;
			}
			case 104: { /* HUMIDITY: 1B ÷2 = % */
				if (offset + 1 > data_len) break;
				dp += snprintf(dp, end - dp, "hum:%u%%", (unsigned)(data[offset] / 2));
				offset += 1;
				entry_ok = true;
				break;
			}
			case 115: { /* PRESSURE: 2B unsigned BE ÷10 = hPa */
				if (offset + 2 > data_len) break;
				uint16_t raw = ((uint16_t)data[offset] << 8) | data[offset + 1];
				offset += 2;
				dp += snprintf(dp, end - dp, "pres:%u.%uhPa", raw / 10, raw % 10);
				entry_ok = true;
				break;
			}
			case 117: { /* CURRENT: 2B unsigned BE (value in mA) */
				if (offset + 2 > data_len) break;
				uint16_t raw = ((uint16_t)data[offset] << 8) | data[offset + 1];
				offset += 2;
				dp += snprintf(dp, end - dp, "cur:%umA", (unsigned)raw);
				entry_ok = true;
				break;
			}
			case 128: { /* POWER: 2B unsigned BE = W */
				if (offset + 2 > data_len) break;
				uint16_t raw = ((uint16_t)data[offset] << 8) | data[offset + 1];
				offset += 2;
				dp += snprintf(dp, end - dp, "pwr:%uW", (unsigned)raw);
				entry_ok = true;
				break;
			}
			default:
				parse_ok = false;
				break;
			}
			if (!entry_ok) {
				dp = entry_start; /* rollback partial newline */
				parse_ok = false;
			}
		}
		*dp = '\0';
		if (dp == text) snprintf(text, sizeof(text), "no telemetry");
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
	int result = mesh->sendCommandData(*contact, ts, 0, cmd, est_timeout);
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

bool RepeaterAdminScreen::handleInput(char c)
{
	switch (_state) {

	case STATE_PASSWORD_ENTRY: {
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

	case STATE_LOGGING_IN:
		if (c == KEY_CANCEL || c == KEY_HOME) {
			_awaiting_tx = false;
			_state = STATE_PASSWORD_ENTRY;
			return true;
		}
		return false;

	case STATE_SUBMENU: {
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

	case STATE_MAIN: {
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

	case STATE_CMD_INPUT: {
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

	} /* switch */
	return false;
}
