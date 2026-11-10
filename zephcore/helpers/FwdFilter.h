/*
 * SPDX-License-Identifier: MIT
 * FwdFilter - operator blacklists for flood forwarding on a repeater
 *
 * Three checks on received flood packets, all inert until unlocked:
 *  - repeater list: a path entry matches a listed hash prefix;
 *  - source list: a flood REQ's source byte (and optionally its destination
 *    byte and first repeater) matches a rule;
 *  - request limit: flood REQs per (source, destination) pair per hour.
 * Pure logic and its text commands, no I/O (devdocs/lld/07-server-roles.md
 * section 25, devdocs/CLI_filter.md).
 */

#pragma once

#include <mesh/Packet.h>
#include <stddef.h>
#include <stdint.h>

class FwdFilter {
public:
	static constexpr int MAX_RPT = 16;
	static constexpr int MAX_SRC = 16;
	static constexpr int LIMIT_SLOTS = 32;
	static constexpr uint32_t LIMIT_WINDOW_MS = 60UL * 60UL * 1000UL;
	static constexpr size_t BLOB_MAX = 6 + MAX_RPT * 4 + MAX_SRC * 7;

	/* A public-key prefix of 1 to 3 bytes; len 0 is "none". */
	struct Hash {
		uint8_t len;
		uint8_t b[3];
	};
	struct SrcRule {
		uint8_t src;
		bool has_dst;
		uint8_t dst;
		Hash from;  /* first repeater in the path, optional */
	};

	bool isEnabled() const { return _enabled; }
	void setEnabled(bool en) { _enabled = en; }
	uint8_t reqLimit() const { return _req_limit; }
	void setReqLimit(uint8_t per_hour) { _req_limit = per_hour; }

	int numRepeaters() const { return _num_rpt; }
	const Hash& repeater(int i) const { return _rpt[i]; }
	bool addRepeater(const Hash& h);  /* false: list full */
	bool delRepeater(const Hash& h);  /* false: not listed */

	int numSources() const { return _num_src; }
	const SrcRule& source(int i) const { return _src[i]; }
	int addSource(const SrcRule& r);  /* index, or -1: list full */
	bool delSource(int idx);

	void clearRules();

	/* True if the packet's first payload byte is `self_hash` on a type that
	 * starts with a destination hash. */
	static bool isAddressedTo(const mesh::Packet* pkt, uint8_t self_hash);

	/* True if a list rule drops this flood packet; counts the drop. */
	bool matches(const mesh::Packet* pkt);

	/* True if this flood REQ is over its pair's hourly limit; counts the
	 * packet against the pair otherwise. Call once per distinct packet. */
	bool overLimit(const mesh::Packet* pkt, uint32_t now_ms);

	uint32_t droppedRpt() const { return _drop_rpt; }
	uint32_t droppedSrc() const { return _drop_src; }
	uint32_t droppedLimit() const { return _drop_limit; }

	/* The persisted state: enabled, limit and both lists. */
	size_t save(uint8_t* buf, size_t cap) const;  /* bytes written, 0 if cap is short */
	bool load(const uint8_t* buf, size_t len);

	/* Runs `idkfa`, `idkfa off` and, when enabled, `filter ...`.
	 * Returns false if the line is not one of ours (or the filter is locked);
	 * *changed is set when the persisted state changed. */
	bool command(const char* line, char* reply, size_t cap, bool* changed);

private:
	struct LimitSlot {
		bool used;
		uint8_t src, dst;
		uint8_t count;
		uint32_t start_ms;
	};

	bool _enabled = false;
	uint8_t _req_limit = 0;
	uint8_t _num_rpt = 0, _num_src = 0;
	Hash _rpt[MAX_RPT] = {};
	SrcRule _src[MAX_SRC] = {};
	LimitSlot _slots[LIMIT_SLOTS] = {};
	uint32_t _drop_rpt = 0, _drop_src = 0, _drop_limit = 0;

	static bool prefixMatch(const Hash& h, const uint8_t* entry, uint8_t entry_size);
	static bool isFloodReq(const mesh::Packet* pkt);
	void cmdRpt(char** tok, int n, char* reply, size_t cap, bool* changed);
	void cmdSrc(char** tok, int n, char* reply, size_t cap, bool* changed);
};
