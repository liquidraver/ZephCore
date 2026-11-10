/*
 * SPDX-License-Identifier: MIT
 * FwdFilter - operator blacklists for flood forwarding on a repeater
 */

#include "FwdFilter.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using mesh::Packet;

static const uint8_t BLOB_MAGIC = 'F';
static const uint8_t BLOB_VERSION = 1;

/* Compares the bytes both sides have: a 2-byte rule matches a 1-byte entry on
 * its first byte, and a 3-byte entry on its first two. */
bool FwdFilter::prefixMatch(const Hash& h, const uint8_t* entry, uint8_t entry_size)
{
	uint8_t n = h.len < entry_size ? h.len : entry_size;
	return n > 0 && memcmp(h.b, entry, n) == 0;
}

bool FwdFilter::isFloodReq(const Packet* pkt)
{
	return pkt->isRouteFlood() && pkt->getPayloadType() == PAYLOAD_TYPE_REQ && pkt->payload_len >= 2;
}

bool FwdFilter::isAddressedTo(const Packet* pkt, uint8_t self_hash)
{
	switch (pkt->getPayloadType()) {
	case PAYLOAD_TYPE_REQ:
	case PAYLOAD_TYPE_RESPONSE:
	case PAYLOAD_TYPE_TXT_MSG:
	case PAYLOAD_TYPE_PATH:
	case PAYLOAD_TYPE_ANON_REQ:
		return pkt->payload_len >= 1 && pkt->payload[0] == self_hash;
	default:
		return false;
	}
}

bool FwdFilter::addRepeater(const Hash& h)
{
	for (int i = 0; i < _num_rpt; i++) {
		if (_rpt[i].len == h.len && memcmp(_rpt[i].b, h.b, h.len) == 0) return true;
	}
	if (_num_rpt >= MAX_RPT) return false;
	_rpt[_num_rpt++] = h;
	return true;
}

bool FwdFilter::delRepeater(const Hash& h)
{
	for (int i = 0; i < _num_rpt; i++) {
		if (_rpt[i].len == h.len && memcmp(_rpt[i].b, h.b, h.len) == 0) {
			for (int k = i; k < _num_rpt - 1; k++) _rpt[k] = _rpt[k + 1];
			_num_rpt--;
			return true;
		}
	}
	return false;
}

int FwdFilter::addSource(const SrcRule& r)
{
	if (_num_src >= MAX_SRC) return -1;
	_src[_num_src] = r;
	return _num_src++;
}

bool FwdFilter::delSource(int idx)
{
	if (idx < 0 || idx >= _num_src) return false;
	for (int k = idx; k < _num_src - 1; k++) _src[k] = _src[k + 1];
	_num_src--;
	return true;
}

void FwdFilter::clearRules()
{
	_num_rpt = _num_src = 0;
	_req_limit = 0;
	memset(_slots, 0, sizeof(_slots));
}

bool FwdFilter::matches(const Packet* pkt)
{
	if (!_enabled || !pkt->isRouteFlood()) return false;

	uint8_t sz = pkt->getPathHashSize();
	uint8_t count = pkt->getPathHashCount();
	if ((unsigned)sz * count > MAX_PATH_SIZE) return false;  /* malformed: not ours to judge */

	for (int r = 0; r < _num_rpt; r++) {
		for (uint8_t i = 0; i < count; i++) {
			if (prefixMatch(_rpt[r], &pkt->path[i * sz], sz)) {
				_drop_rpt++;
				return true;
			}
		}
	}

	if (isFloodReq(pkt)) {
		for (int r = 0; r < _num_src; r++) {
			const SrcRule& s = _src[r];
			if (pkt->payload[1] != s.src) continue;
			if (s.has_dst && pkt->payload[0] != s.dst) continue;
			if (s.from.len && (count == 0 || !prefixMatch(s.from, pkt->path, sz))) continue;
			_drop_src++;
			return true;
		}
	}
	return false;
}

bool FwdFilter::overLimit(const Packet* pkt, uint32_t now_ms)
{
	if (!_enabled || _req_limit == 0 || !isFloodReq(pkt)) return false;

	uint8_t dst = pkt->payload[0], src = pkt->payload[1];
	LimitSlot* slot = nullptr;
	LimitSlot* spare = nullptr;  /* unused, else expired, else the oldest window */
	for (int i = 0; i < LIMIT_SLOTS; i++) {
		LimitSlot* s = &_slots[i];
		if (s->used && s->src == src && s->dst == dst) {
			slot = s;
			break;
		}
		if (!s->used) {
			if (!spare || spare->used) spare = s;
		} else if (!spare || (spare->used && (uint32_t)(now_ms - s->start_ms) > (uint32_t)(now_ms - spare->start_ms))) {
			spare = s;
		}
	}
	if (!slot || (uint32_t)(now_ms - slot->start_ms) >= LIMIT_WINDOW_MS) {
		if (!slot) slot = spare;
		slot->used = true;
		slot->src = src;
		slot->dst = dst;
		slot->count = 0;
		slot->start_ms = now_ms;
	}
	if (slot->count >= _req_limit) {
		_drop_limit++;
		return true;
	}
	slot->count++;
	return false;
}

size_t FwdFilter::save(uint8_t* buf, size_t cap) const
{
	size_t need = 6 + (size_t)_num_rpt * 4 + (size_t)_num_src * 7;
	if (cap < need) return 0;
	size_t i = 0;
	buf[i++] = BLOB_MAGIC;
	buf[i++] = BLOB_VERSION;
	buf[i++] = _enabled ? 1 : 0;
	buf[i++] = _req_limit;
	buf[i++] = _num_rpt;
	buf[i++] = _num_src;
	for (int r = 0; r < _num_rpt; r++) {
		buf[i++] = _rpt[r].len;
		memcpy(&buf[i], _rpt[r].b, 3); i += 3;
	}
	for (int r = 0; r < _num_src; r++) {
		buf[i++] = _src[r].src;
		buf[i++] = _src[r].has_dst ? 1 : 0;
		buf[i++] = _src[r].dst;
		buf[i++] = _src[r].from.len;
		memcpy(&buf[i], _src[r].from.b, 3); i += 3;
	}
	return i;
}

bool FwdFilter::load(const uint8_t* buf, size_t len)
{
	if (len < 6 || buf[0] != BLOB_MAGIC || buf[1] != BLOB_VERSION) return false;
	uint8_t n_rpt = buf[4], n_src = buf[5];
	if (n_rpt > MAX_RPT || n_src > MAX_SRC) return false;
	if (len < 6 + (size_t)n_rpt * 4 + (size_t)n_src * 7) return false;

	size_t i = 6;
	Hash rpt[MAX_RPT];
	SrcRule src[MAX_SRC];
	for (int r = 0; r < n_rpt; r++) {
		rpt[r].len = buf[i++];
		memcpy(rpt[r].b, &buf[i], 3); i += 3;
		if (rpt[r].len < 1 || rpt[r].len > 3) return false;
	}
	for (int r = 0; r < n_src; r++) {
		src[r].src = buf[i++];
		src[r].has_dst = buf[i++] != 0;
		src[r].dst = buf[i++];
		src[r].from.len = buf[i++];
		memcpy(src[r].from.b, &buf[i], 3); i += 3;
		if (src[r].from.len > 3) return false;
	}

	_enabled = (buf[2] & 1) != 0;
	_req_limit = buf[3];
	_num_rpt = n_rpt;
	_num_src = n_src;
	memcpy(_rpt, rpt, sizeof(Hash) * n_rpt);
	memcpy(_src, src, sizeof(SrcRule) * n_src);
	return true;
}

/* ---- text commands ---- */

static int hexNibble(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

/* 2, 4 or 6 hex digits. */
static bool parseHash(const char* s, FwdFilter::Hash* h)
{
	size_t n = strlen(s);
	if (n != 2 && n != 4 && n != 6) return false;
	memset(h, 0, sizeof(*h));
	for (size_t i = 0; i < n; i += 2) {
		int hi = hexNibble(s[i]), lo = hexNibble(s[i + 1]);
		if (hi < 0 || lo < 0) return false;
		h->b[i / 2] = (uint8_t)((hi << 4) | lo);
	}
	h->len = (uint8_t)(n / 2);
	return true;
}

static int fmtHash(char* out, size_t cap, const FwdFilter::Hash& h)
{
	int n = 0;
	for (uint8_t i = 0; i < h.len; i++) n += snprintf(out + n, cap - n, "%02x", h.b[i]);
	return n;
}

/* Appends `item` (with a leading separator) if it fits with room for a
 * trailing '+'; otherwise appends '+' and returns false. */
static bool appendItem(char* reply, size_t cap, size_t* len, const char* item, char sep)
{
	size_t need = strlen(item) + 1;
	if (*len + need + 2 > cap) {
		if (*len + 2 <= cap) {
			reply[(*len)++] = '+';
			reply[*len] = 0;
		}
		return false;
	}
	reply[(*len)++] = sep;
	strcpy(&reply[*len], item);
	*len += need - 1;
	return true;
}

void FwdFilter::cmdRpt(char** tok, int n, char* reply, size_t cap, bool* changed)
{
	if (n == 2) {  /* list */
		size_t len = (size_t)snprintf(reply, cap, ">");
		if (_num_rpt == 0) snprintf(reply, cap, "> (none)");
		for (int i = 0; i < _num_rpt; i++) {
			char item[8];
			fmtHash(item, sizeof(item), _rpt[i]);
			if (!appendItem(reply, cap, &len, item, ' ')) break;
		}
		return;
	}
	Hash h;
	if (n != 4 || !parseHash(tok[3], &h)) {
		snprintf(reply, cap, "Err - filter rpt [add|del <1-3 byte hex>]");
	} else if (strcmp(tok[2], "add") == 0) {
		if (addRepeater(h)) {
			*changed = true;
			snprintf(reply, cap, "OK");
		} else {
			snprintf(reply, cap, "Err - list full (%d)", MAX_RPT);
		}
	} else if (strcmp(tok[2], "del") == 0) {
		if (delRepeater(h)) {
			*changed = true;
			snprintf(reply, cap, "OK");
		} else {
			snprintf(reply, cap, "Err - not listed");
		}
	} else {
		snprintf(reply, cap, "Err - filter rpt [add|del <1-3 byte hex>]");
	}
}

void FwdFilter::cmdSrc(char** tok, int n, char* reply, size_t cap, bool* changed)
{
	static const char usage[] = "Err - filter src [add <hex> [dst <hex>] [from <hex>] | del <n>]";

	if (n == 2) {  /* list: n:src[>dst][@from] */
		size_t len = (size_t)snprintf(reply, cap, ">");
		if (_num_src == 0) snprintf(reply, cap, "> (none)");
		for (int i = 0; i < _num_src; i++) {
			char item[24];
			int k = snprintf(item, sizeof(item), "%d:%02x", i, _src[i].src);
			if (_src[i].has_dst) k += snprintf(item + k, sizeof(item) - k, ">%02x", _src[i].dst);
			if (_src[i].from.len) {
				item[k++] = '@';
				fmtHash(item + k, sizeof(item) - k, _src[i].from);
			}
			if (!appendItem(reply, cap, &len, item, ' ')) break;
		}
		return;
	}
	if (n == 4 && strcmp(tok[2], "del") == 0) {
		char* end;
		long idx = strtol(tok[3], &end, 10);
		if (*end == 0 && delSource((int)idx)) {
			*changed = true;
			snprintf(reply, cap, "OK");
		} else {
			snprintf(reply, cap, "Err - no such rule");
		}
		return;
	}
	if (n < 4 || (n != 4 && n != 6 && n != 8) || strcmp(tok[2], "add") != 0) {
		snprintf(reply, cap, "%s", usage);
		return;
	}

	Hash h;
	SrcRule r = {};
	if (!parseHash(tok[3], &h)) {
		snprintf(reply, cap, "%s", usage);
		return;
	}
	r.src = h.b[0];  /* the payload carries one source byte */
	for (int i = 4; i < n; i += 2) {
		if (!parseHash(tok[i + 1], &h)) {
			snprintf(reply, cap, "%s", usage);
			return;
		}
		if (strcmp(tok[i], "dst") == 0) {
			r.has_dst = true;
			r.dst = h.b[0];
		} else if (strcmp(tok[i], "from") == 0) {
			r.from = h;
		} else {
			snprintf(reply, cap, "%s", usage);
			return;
		}
	}
	int idx = addSource(r);
	if (idx < 0) {
		snprintf(reply, cap, "Err - list full (%d)", MAX_SRC);
		return;
	}
	*changed = true;
	if (!r.has_dst && !r.from.len) {
		snprintf(reply, cap, "OK - rule %d: src %02x matches 1/256 of nodes", idx, r.src);
	} else {
		snprintf(reply, cap, "OK - rule %d", idx);
	}
}

bool FwdFilter::command(const char* line, char* reply, size_t cap, bool* changed)
{
	*changed = false;
	while (*line == ' ') line++;

	bool is_key = strncmp(line, "idkfa", 5) == 0 && (line[5] == 0 || line[5] == ' ');
	bool is_filter = strncmp(line, "filter", 6) == 0 && (line[6] == 0 || line[6] == ' ');
	if (!is_key && !(is_filter && _enabled)) return false;

	char buf[80];
	if (strlen(line) >= sizeof(buf)) {
		if (is_key) return false;
		snprintf(reply, cap, "Err - too long");
		return true;
	}
	strcpy(buf, line);
	char* tok[9];
	int n = 0;
	for (char* p = strtok(buf, " "); p && n < 9; p = strtok(nullptr, " ")) tok[n++] = p;

	if (is_key) {
		bool en;
		if (n == 1) {
			en = true;
		} else if (n == 2 && strcmp(tok[1], "off") == 0) {
			en = false;
		} else {
			return false;
		}
		*changed = en != _enabled;
		_enabled = en;
		snprintf(reply, cap, en ? "OK - filter on" : "OK - filter off");
		return true;
	}

	if (n == 1) {
		snprintf(reply, cap, "> rpt:%d src:%d req.limit:%u/h", _num_rpt, _num_src, (unsigned)_req_limit);
	} else if (strcmp(tok[1], "rpt") == 0) {
		cmdRpt(tok, n, reply, cap, changed);
	} else if (strcmp(tok[1], "src") == 0) {
		cmdSrc(tok, n, reply, cap, changed);
	} else if (strcmp(tok[1], "req.limit") == 0 && n == 2) {
		snprintf(reply, cap, "> %u", (unsigned)_req_limit);
	} else if (strcmp(tok[1], "req.limit") == 0 && n == 3) {
		char* end;
		long v = strtol(tok[2], &end, 10);
		if (*end != 0 || v < 0 || v > 255) {
			snprintf(reply, cap, "Err - 0 (off) to 255 per hour");
		} else {
			*changed = v != _req_limit;
			_req_limit = (uint8_t)v;
			snprintf(reply, cap, "OK");
		}
	} else if (strcmp(tok[1], "stats") == 0 && n == 2) {
		snprintf(reply, cap, "> dropped rpt:%u src:%u limit:%u", (unsigned)_drop_rpt, (unsigned)_drop_src,
			 (unsigned)_drop_limit);
	} else if (strcmp(tok[1], "clear") == 0 && n == 2) {
		clearRules();
		*changed = true;
		snprintf(reply, cap, "OK");
	} else {
		snprintf(reply, cap, "Err - filter [rpt|src|req.limit|stats|clear]");
	}
	return true;
}
