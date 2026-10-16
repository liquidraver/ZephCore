/*
 * SPDX-License-Identifier: MIT
 * What every role that speaks zcpfs shares: the capability tag in an advert's
 * name field (lib/zcpfs/SPEC.md 6.1) and the run-time mode with its CLI.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <mesh/MeshCore.h>
#include <helpers/NodePrefs.h>
#include <helpers/AdvertDataHelpers.h>

/* Append the tag to the name field of an own advert; returns the new length. */
static inline uint8_t zcpfsTagAdvert(uint8_t *app_data, uint8_t len)
{
	if (!(app_data[0] & ADV_NAME_MASK)) {
		app_data[0] |= ADV_NAME_MASK;		/* the field is always sent, even empty */
	} else if (len >= MAX_ADVERT_DATA_SIZE) {
		/* make room: drop the name's last character, whole */
		while (len > 1 && (app_data[len - 1] & 0xC0) == 0x80) {
			len--;
		}
		len--;
	}
	app_data[len++] = 0;
	return len;
}

/* The capability an advert's name field carries: 0 if none. */
static inline int zcpfsAdvertCapability(const uint8_t *app_data, size_t len)
{
	size_t i = 1;

	if (len < 1 || !(app_data[0] & ADV_NAME_MASK)) {
		return 0;
	}
	if (app_data[0] & ADV_LATLON_MASK) {
		i += 8;
	}
	if (app_data[0] & ADV_FEAT1_MASK) {
		i += 2;
	}
	if (app_data[0] & ADV_FEAT2_MASK) {
		i += 2;
	}
	for (; i < len; i++) {
		if (app_data[i] == 0) {
			return i + 1 < len ? app_data[i + 1] : 1;
		}
	}
	return 0;
}

/* NodePrefs.zcpfs_mode (devdocs/lld/14-zcpfs.md 8.17). */
#define ZCPFS_MODE_OFF		0	/* legacy on the air: nothing sealed, nothing read */
#define ZCPFS_MODE_PASSIVE	1	/* no tag, starts nothing; reads and answers what arrives sealed */
#define ZCPFS_MODE_ON		2

/* The state of a channel (NodePrefs.zcpfs_chan, SPEC 17.3). */
#define ZCPFS_CHAN_OFF		0	/* sends legacy, reads both */
#define ZCPFS_CHAN_ON		1	/* sends sealed, reads both */
#define ZCPFS_CHAN_STRICT	2	/* sends sealed, reads sealed only */

static inline const char *zcpfsModeName(uint8_t mode)
{
	return mode == ZCPFS_MODE_ON ? "on" : mode == ZCPFS_MODE_PASSIVE ? "passive" : "off";
}

/* Results of zcpfsModeCommand(). */
#define ZCPFS_CMD_NONE		0	/* not this command */
#define ZCPFS_CMD_DONE		1	/* answered */
#define ZCPFS_CMD_CHANGED	2	/* answered, and `mode` changed: store it */

/* `get zcpfs` and `set zcpfs on|passive|off` (1 and 0 for on and off). */
static inline int zcpfsModeCommand(const char *line, uint8_t &mode, char *reply, size_t size)
{
	if (strcmp(line, "get zcpfs") == 0) {
		snprintf(reply, size, "> %s", zcpfsModeName(mode));
		return ZCPFS_CMD_DONE;
	}
	if (strncmp(line, "set zcpfs ", 10) != 0) {
		return ZCPFS_CMD_NONE;
	}

	const char *v = line + 10;
	uint8_t m;

	if (strcmp(v, "on") == 0 || strcmp(v, "1") == 0) {
		m = ZCPFS_MODE_ON;
	} else if (strcmp(v, "passive") == 0) {
		m = ZCPFS_MODE_PASSIVE;
	} else if (strcmp(v, "off") == 0 || strcmp(v, "0") == 0) {
		m = ZCPFS_MODE_OFF;
	} else {
		snprintf(reply, size, "Err - on, passive or off");
		return ZCPFS_CMD_DONE;
	}
	snprintf(reply, size, "OK - zcpfs %s", zcpfsModeName(m));
	if (m == mode) {
		return ZCPFS_CMD_DONE;
	}
	mode = m;
	return ZCPFS_CMD_CHANGED;
}

/* `get zcpfs.legacylogin` and `set zcpfs.legacylogin on|off` (1 and 0);
 * `refuse` is the stored flag, 1 for off (SPEC 19.6). */
static inline int zcpfsLegacyLoginCommand(const char *line, uint8_t &refuse, char *reply, size_t size)
{
	if (strcmp(line, "get zcpfs.legacylogin") == 0) {
		snprintf(reply, size, "> %s", refuse ? "off" : "on");
		return ZCPFS_CMD_DONE;
	}
	if (strncmp(line, "set zcpfs.legacylogin ", 22) != 0) {
		return ZCPFS_CMD_NONE;
	}

	const char *v = line + 22;
	uint8_t r;

	if (strcmp(v, "on") == 0 || strcmp(v, "1") == 0) {
		r = 0;
	} else if (strcmp(v, "off") == 0 || strcmp(v, "0") == 0) {
		r = 1;
	} else {
		snprintf(reply, size, "Err - on or off");
		return ZCPFS_CMD_DONE;
	}
	snprintf(reply, size, "OK - legacy logins %s", r ? "off" : "on");
	if (r == refuse) {
		return ZCPFS_CMD_DONE;
	}
	refuse = r;
	return ZCPFS_CMD_CHANGED;
}

/* Passive mode seals a text only to a contact that has sent something sealed
 * since this node's last advert: that contact has not yet seen the untagged
 * advert and still pins us. One entry per contact record, by key prefix. */
class ZcpfsHeard {
public:
	/* A sealed packet from S arrived at `t` (engine seconds). */
	void mark(const uint8_t *S, uint32_t t)
	{
		int k = 0;

		for (int i = 0; i < CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) {
			if (memcmp(_e[i].S, S, sizeof(_e[i].S)) == 0) {
				k = i;
				break;
			}
			if ((int32_t)(_e[i].at - _e[k].at) < 0) {
				k = i;		/* else the one heard longest ago */
			}
		}
		memcpy(_e[k].S, S, sizeof(_e[k].S));
		_e[k].at = t;
	}
	/* Own advert built at `t`. */
	void advertised(uint32_t t) { _advert_at = t; _any = true; }
	/* S spoke sealed after our last advert. */
	bool since(const uint8_t *S) const
	{
		for (int i = 0; i < CONFIG_ZEPHCORE_ZCPFS_PEERS; i++) {
			if (_e[i].at != 0 && memcmp(_e[i].S, S, sizeof(_e[i].S)) == 0) {
				return !_any || (int32_t)(_e[i].at - _advert_at) >= 0;
			}
		}
		return false;
	}

private:
	struct {
		uint8_t S[8];
		uint32_t at;
	} _e[CONFIG_ZEPHCORE_ZCPFS_PEERS] = {};
	uint32_t _advert_at = 0;
	bool _any = false;
};
