/*
 * SPDX-License-Identifier: MIT
 * See telemetry_text.h.
 */

#include "telemetry_text.h"

#include <helpers/SensorManager.h>        /* TELEM_CHANNEL_SELF */
#include <helpers/sensors/LPPDataHelpers.h>

#include <stdio.h>

void telemetry_text(const uint8_t *data, uint8_t len, char *out, size_t cap)
{
	char *dp = out;
	char *end = out + cap;
	uint8_t pos = 0;

	*dp = '\0';
	while (pos + 2 < len && end - dp > 24) {
		uint8_t ch = data[pos];
		uint8_t type = data[pos + 1];
		const uint8_t *v = &data[pos + 2];
		uint8_t size = LPPData::getDataSize(type);

		pos += 2;
		if (ch == 0 || pos + size > len) {
			break;  /* end-of-data marker, or a truncated field */
		}
		pos += size;

		char pre[6] = "";
		if (ch != TELEM_CHANNEL_SELF) {
			snprintf(pre, sizeof(pre), "c%u ", ch);
		}
		const char *sep = (dp > out) ? "\n" : "";
		int n = 0;

		/* Scales and signs as the encoder (helpers/compat/CayenneLPP.h);
		 * LPPData's own tables miss a few (humidity scale, current sign). */
		switch (type) {
		case LPP_VOLTAGE:
			n = snprintf(dp, end - dp, "%s%s%s%.2fV", sep, pre,
				     ch == TELEM_CHANNEL_SELF ? "batt " : "",
				     (double)LPPData::getFloat(v, 2, 100, true));
			break;
		case LPP_TEMPERATURE:
			n = snprintf(dp, end - dp, "%s%stemp %.1fC", sep, pre,
				     (double)LPPData::getFloat(v, 2, 10, true));
			break;
		case LPP_RELATIVE_HUMIDITY:
			n = snprintf(dp, end - dp, "%s%shum %.1f%%", sep, pre,
				     (double)LPPData::getFloat(v, 1, 2, false));
			break;
		case LPP_BAROMETRIC_PRESSURE:
			n = snprintf(dp, end - dp, "%s%spres %.1fhPa", sep, pre,
				     (double)LPPData::getFloat(v, 2, 10, false));
			break;
		case LPP_ALTITUDE:
			n = snprintf(dp, end - dp, "%s%salt %.0fm", sep, pre,
				     (double)LPPData::getFloat(v, 2, 1, true));
			break;
		case LPP_LUMINOSITY:
			n = snprintf(dp, end - dp, "%s%slight %.0f", sep, pre,
				     (double)LPPData::getFloat(v, 2, 1, false));
			break;
		case LPP_CURRENT:  /* signed: a bidirectional monitor reports discharge < 0 */
			n = snprintf(dp, end - dp, "%s%scur %.0fmA", sep, pre,
				     (double)(LPPData::getFloat(v, 2, 1000, true) * 1000.0f));
			break;
		case LPP_POWER:
			n = snprintf(dp, end - dp, "%s%spwr %.0fW", sep, pre,
				     (double)LPPData::getFloat(v, 2, 1, false));
			break;
		case LPP_GPS:
			n = snprintf(dp, end - dp, "%s%sgps %.4f,%.4f", sep, pre,
				     (double)LPPData::getFloat(v, 3, 10000, true),
				     (double)LPPData::getFloat(v + 3, 3, 10000, true));
			break;
		default:
			break;  /* not shown; its size was skipped above */
		}
		if (n < 0 || n >= end - dp) {
			break;
		}
		dp += n;
	}
	*dp = '\0';
	if (dp == out) {
		snprintf(out, cap, "no telemetry");
	}
}
