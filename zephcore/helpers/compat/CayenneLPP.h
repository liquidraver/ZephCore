/*
 * SPDX-License-Identifier: MIT
 *
 * CayenneLPP - the subset of the electroniccats/CayenneLPP 1.6.1 encoder
 * (upstream MeshCore's dependency) that MeshCore's telemetry uses, with the
 * same API and wire format: [channel][type][value, big-endian], the add*
 * functions returning the new size, or 0 when the field does not fit.
 *
 * One deliberate difference: values are rounded to the nearest step after
 * scaling, where the library truncates toward zero. The library turns a
 * 3850 mV battery into 3.84 V (3.85f * 100 = 384.99997); this sends 3.85.
 * Same bytes on the wire for any decoder, closer to the reading.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

/* Type codes and multipliers, as the library's CayenneLPP.h */
#define LPP_ANALOG_INPUT        2     /* 2 bytes, 0.01 signed */
#define LPP_GENERIC_SENSOR      100   /* 4 bytes, unsigned */
#define LPP_LUMINOSITY          101   /* 2 bytes, 1 lux unsigned */
#define LPP_TEMPERATURE         103   /* 2 bytes, 0.1°C signed */
#define LPP_RELATIVE_HUMIDITY   104   /* 1 byte, 0.5% unsigned */
#define LPP_BAROMETRIC_PRESSURE 115   /* 2 bytes 0.1hPa unsigned */
#define LPP_VOLTAGE             116   /* 2 bytes 0.01V */
#define LPP_CURRENT             117   /* 2 bytes 0.001A */
#define LPP_ALTITUDE            121   /* 2 bytes 1m signed */
#define LPP_POWER               128   /* 2 bytes, 1W unsigned */
#define LPP_DISTANCE            130   /* 4 bytes, 0.001m unsigned */
#define LPP_GPS                 136   /* 3 bytes lat/lon 0.0001°, 3 bytes alt 0.01m */

#define LPP_ANALOG_INPUT_MULT        100
#define LPP_TEMPERATURE_MULT         10
#define LPP_RELATIVE_HUMIDITY_MULT   2
#define LPP_BAROMETRIC_PRESSURE_MULT 10
#define LPP_VOLTAGE_MULT             100
#define LPP_CURRENT_MULT             1000
#define LPP_DISTANCE_MULT            1000
#define LPP_GPS_LAT_LON_MULT         10000
#define LPP_GPS_ALT_MULT             100

/* Upstream sizes its telemetry buffer from MAX_PACKET_PAYLOAD; this holds it. */
#define CAYENNE_LPP_MAX_SIZE 192

class CayenneLPP {
public:
	explicit CayenneLPP(uint8_t size)
		: _maxsize(size > CAYENNE_LPP_MAX_SIZE ? CAYENNE_LPP_MAX_SIZE : size), _cursor(0) {}

	void reset(void) { _cursor = 0; }
	uint8_t getSize(void) { return _cursor; }
	uint8_t *getBuffer(void) { return _buffer; }

	uint8_t addTemperature(uint8_t channel, float value)
	{
		return addField(LPP_TEMPERATURE, channel, value, 2, LPP_TEMPERATURE_MULT, true);
	}
	uint8_t addRelativeHumidity(uint8_t channel, float value)
	{
		return addField(LPP_RELATIVE_HUMIDITY, channel, value, 1,
				LPP_RELATIVE_HUMIDITY_MULT, false);
	}
	/* The library takes an integer: a relative-scale light sensor (the T1000-E
	 * photocell's 0-100) goes through unchanged, as upstream reports it. */
	uint8_t addLuminosity(uint8_t channel, uint32_t value)
	{
		return addField(LPP_LUMINOSITY, channel, (float)value, 2, 1, false);
	}
	uint8_t addBarometricPressure(uint8_t channel, float value)
	{
		return addField(LPP_BAROMETRIC_PRESSURE, channel, value, 2,
				LPP_BAROMETRIC_PRESSURE_MULT, false);
	}
	uint8_t addVoltage(uint8_t channel, float value)
	{
		return addField(LPP_VOLTAGE, channel, value, 2, LPP_VOLTAGE_MULT, true);
	}
	/* Signed, so a bidirectional monitor (INA219) reports discharge as negative. */
	uint8_t addCurrent(uint8_t channel, float value)
	{
		return addField(LPP_CURRENT, channel, value, 2, LPP_CURRENT_MULT, true);
	}
	uint8_t addPower(uint8_t channel, float value)
	{
		return addField(LPP_POWER, channel, value, 2, 1, false);
	}
	uint8_t addAltitude(uint8_t channel, float value)
	{
		return addField(LPP_ALTITUDE, channel, value, 2, 1, true);
	}
	uint8_t addDistance(uint8_t channel, float value)
	{
		return addField(LPP_DISTANCE, channel, value, 4, LPP_DISTANCE_MULT, false);
	}
	uint8_t addAnalogInput(uint8_t channel, float value)
	{
		return addField(LPP_ANALOG_INPUT, channel, value, 2, LPP_ANALOG_INPUT_MULT, true);
	}
	uint8_t addGenericSensor(uint8_t channel, float value)
	{
		return addField(LPP_GENERIC_SENSOR, channel, value, 4, 1, false);
	}

	uint8_t addGPS(uint8_t channel, float latitude, float longitude, float altitude)
	{
		if (_cursor + 9 + 2 > _maxsize) {
			return 0;
		}
		int32_t lat = scaled(latitude, LPP_GPS_LAT_LON_MULT);
		int32_t lon = scaled(longitude, LPP_GPS_LAT_LON_MULT);
		int32_t alt = scaled(altitude, LPP_GPS_ALT_MULT);

		_buffer[_cursor++] = channel;
		_buffer[_cursor++] = LPP_GPS;
		_buffer[_cursor++] = lat >> 16;
		_buffer[_cursor++] = lat >> 8;
		_buffer[_cursor++] = lat;
		_buffer[_cursor++] = lon >> 16;
		_buffer[_cursor++] = lon >> 8;
		_buffer[_cursor++] = lon;
		_buffer[_cursor++] = alt >> 16;
		_buffer[_cursor++] = alt >> 8;
		_buffer[_cursor++] = alt;
		return _cursor;
	}

private:
	uint8_t _buffer[CAYENNE_LPP_MAX_SIZE];
	uint8_t _maxsize;
	uint8_t _cursor;

	/* value * multiplier, rounded to nearest (half away from zero). */
	static int32_t scaled(float value, uint32_t multiplier)
	{
		float x = value * (float)multiplier;

		return (int32_t)(x < 0 ? x - 0.5f : x + 0.5f);
	}

	/* The library's addField(): the magnitude scaled (rounded here, see the
	 * top of the file), then the sign applied as two's complement within the
	 * field. */
	uint8_t addField(uint8_t type, uint8_t channel, float value, uint8_t size,
			 uint32_t multiplier, bool is_signed)
	{
		if (_cursor + size + 2 > _maxsize) {
			return 0;
		}
		bool sign = value < 0;

		if (sign) {
			value = -value;
		}
		uint32_t v = (uint32_t)(value * (float)multiplier + 0.5f);

		if (is_signed && sign) {
			uint32_t mask = (size >= 4) ? 0xFFFFFFFFu : ((1u << (size * 8)) - 1);

			v = v & mask;
			v = mask - v + 1;
		}
		_buffer[_cursor++] = channel;
		_buffer[_cursor++] = type;
		for (uint8_t i = 1; i <= size; i++) {
			_buffer[_cursor + size - i] = v & 0xFF;
			v >>= 8;
		}
		_cursor += size;
		return _cursor;
	}
};
