/*
 * SPDX-License-Identifier: MIT
 * ZephCore AdvertDataHelpers implementation
 */

#include "AdvertDataHelpers.h"
#include "UTF8Helpers.h"
#include <string.h>

uint8_t AdvertDataBuilder::encodeTo(uint8_t app_data[])
{
	app_data[0] = _type;
	int i = 1;

	if (_has_loc) {
		app_data[0] |= ADV_LATLON_MASK;
		memcpy(&app_data[i], &_lat, 4); i += 4;
		memcpy(&app_data[i], &_lon, 4); i += 4;
	}
	if (_extra1) {
		app_data[0] |= ADV_FEAT1_MASK;
		memcpy(&app_data[i], &_extra1, 2); i += 2;
	}
	if (_extra2) {
		app_data[0] |= ADV_FEAT2_MASK;
		memcpy(&app_data[i], &_extra2, 2); i += 2;
	}
	if (_name && *_name != 0) {
		/* Truncate on a UTF-8 code-point boundary: a byte-wise copy would
		 * emit a partial multi-byte sequence when a code point straddles
		 * MAX_ADVERT_DATA_SIZE, producing a mangled name on the wire.
		 * ADV_NAME_MASK is only set when at least one whole code point
		 * fits in the remaining space. */
		size_t name_len = mesh::validUtf8PrefixLength(_name, MAX_ADVERT_DATA_SIZE - i);
		if (name_len > 0) {
			app_data[0] |= ADV_NAME_MASK;
			memcpy(&app_data[i], _name, name_len);
			i += name_len;
		}
	}
	return i;
}

AdvertDataParser::AdvertDataParser(const uint8_t app_data[], uint8_t app_data_len)
{
	_name[0] = 0;
	_lat = _lon = 0;
	_valid = false;
	_extra1 = _extra2 = 0;
	_flags = 0;

	if (app_data_len < 1) return;
	_flags = app_data[0];

	int i = 1;
	if (_flags & ADV_LATLON_MASK) {
		if (app_data_len < (uint8_t)(i + 8)) return;
		memcpy(&_lat, &app_data[i], 4); i += 4;
		memcpy(&_lon, &app_data[i], 4); i += 4;
	}
	if (_flags & ADV_FEAT1_MASK) {
		if (app_data_len < (uint8_t)(i + 2)) return;
		memcpy(&_extra1, &app_data[i], 2); i += 2;
	}
	if (_flags & ADV_FEAT2_MASK) {
		if (app_data_len < (uint8_t)(i + 2)) return;
		memcpy(&_extra2, &app_data[i], 2); i += 2;
	}

	if (_flags & ADV_NAME_MASK) {
		int nlen = app_data_len - i;
		/* Self-defense: callers clamp app_data_len to MAX_ADVERT_DATA_SIZE
		 * today, but don't trust that — _name is MAX_ADVERT_DATA_SIZE and the
		 * NUL goes at _name[nlen], so bound nlen to sizeof(_name)-1 or a
		 * malformed/oversized advert would overflow this stack object. */
		if (nlen > (int)sizeof(_name) - 1) {
			nlen = (int)sizeof(_name) - 1;
		}
		if (nlen > 0) {
			memcpy(_name, &app_data[i], nlen);
			_name[nlen] = 0;
		}
	}
	_valid = true;
}
