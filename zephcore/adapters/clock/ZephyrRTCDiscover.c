/*
 * SPDX-License-Identifier: MIT
 *
 * Compact raw-I2C hardware-RTC auto-discovery. See ZephyrRTCDiscover.h.
 *
 * Register layouts (sec/min/hour/.../month/year, all BCD) and the per-chip
 * power-loss flags are carried in devicetree via the "zephcore,rtc-i2c"
 * binding, so this reader is generic — adding a new chip is a DT node, not
 * code. Maps were taken from Zephyr's own drivers (rtc_pcf8563.c,
 * rtc_ds3231.c, rtc_rv3028.c, rtc_rx8130ce.c).
 */

#include "ZephyrRTCDiscover.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephcore_rtc, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

#define RTC_COMPAT zephcore_rtc_i2c

#if IS_ENABLED(CONFIG_ZEPHCORE_RTC_AUTODISCOVER) && DT_HAS_COMPAT_STATUS_OKAY(RTC_COMPAT)

/* Validity flag lives in the seconds byte itself (PCF8563 VL bit). */
#define RTC_STATUS_IN_SECONDS 0xFF

struct rtc_desc {
	const struct device *bus;
	uint16_t addr;
	uint8_t  time_reg;     /* register of the seconds byte */
	uint8_t  date_index;   /* day-of-month offset in the 7-byte block */
	uint8_t  status_reg;   /* power-loss flag register, or RTC_STATUS_IN_SECONDS */
	uint8_t  status_mask;  /* "time unreliable" bit within status_reg */
	const char *name;
};

#define RTC_DESC_ENTRY(node)                                          \
	{                                                             \
		.bus         = DEVICE_DT_GET(DT_BUS(node)),           \
		.addr        = (uint16_t)DT_REG_ADDR(node),           \
		.time_reg    = (uint8_t)DT_PROP(node, time_reg),      \
		.date_index  = (uint8_t)DT_PROP(node, date_index),    \
		.status_reg  = (uint8_t)DT_PROP(node, status_reg),    \
		.status_mask = (uint8_t)DT_PROP(node, status_mask),   \
		.name        = DT_NODE_FULL_NAME(node),               \
	},

static const struct rtc_desc rtc_descs[] = {
	DT_FOREACH_STATUS_OKAY(RTC_COMPAT, RTC_DESC_ENTRY)
};

/* Chip we'll read/write going forward (first one found present). */
static const struct rtc_desc *s_active;
static bool s_probed;

#define BCD2BIN(x) ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))
#define BIN2BCD(x) ((((x) / 10) << 4) | ((x) % 10))

/* A byte is valid BCD if both nibbles are 0-9, and its decoded value fits the
 * field. Used to tell a real RTC apart from an unrelated I2C chip that happens
 * to share an address (e.g. an MPU-class IMU at 0x68, same as DS3231) — we must
 * never adopt and write time into such a device. */
static bool bcd_field_ok(uint8_t v, unsigned max)
{
	if ((v & 0x0F) > 9 || (v >> 4) > 9) {
		return false;
	}
	return BCD2BIN(v) <= max;
}

/* Howard Hinnant's civil<->days algorithms (proleptic Gregorian, UTC). */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
	y -= (m <= 2);
	int64_t era = (y >= 0 ? y : y - 399) / 400;
	unsigned yoe = (unsigned)(y - era * 400);
	unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
	unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return era * 146097 + (int)doe - 719468;
}

static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *d)
{
	z += 719468;
	int64_t era = (z >= 0 ? z : z - 146096) / 146097;
	unsigned doe = (unsigned)(z - era * 146097);
	unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	int yy = (int)yoe + (int)(era * 400);
	unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	unsigned mp = (5 * doy + 2) / 153;
	*d = doy - (153 * mp + 2) / 5 + 1;
	*m = mp < 10 ? mp + 3 : mp - 9;
	*y = yy + (*m <= 2);
}

/* Read the chip's power-loss flag. true => held time is unreliable. */
static bool rtc_time_unreliable(const struct rtc_desc *d, const uint8_t blk[7])
{
	if (d->status_reg == RTC_STATUS_IN_SECONDS) {
		return (blk[0] & d->status_mask) != 0;
	}

	uint8_t st;
	if (i2c_reg_read_byte(d->bus, d->addr, d->status_reg, &st) != 0) {
		return true;  /* can't confirm => don't trust it */
	}
	return (st & d->status_mask) != 0;
}

/* Probe all chips once; cache the first present one in s_active. If a present
 * chip holds a sane time, return it via epoch_out. */
static bool rtc_probe(uint32_t *epoch_out)
{
	for (size_t i = 0; i < ARRAY_SIZE(rtc_descs); i++) {
		const struct rtc_desc *d = &rtc_descs[i];
		uint8_t blk[7];

		if (!device_is_ready(d->bus)) {
			continue;
		}
		if (i2c_burst_read(d->bus, d->addr, d->time_reg, blk, sizeof(blk)) != 0) {
			continue;  /* no ACK => chip absent */
		}

		/* Mask off flag/century bits. We trust this is a real RTC (vs. an
		 * unrelated chip sharing the address) if EITHER the block is valid
		 * BCD, OR the chip's power-loss flag is set — the latter is itself
		 * proof it's an RTC that lost power and whose time registers may be
		 * garbage. Adopting in the power-loss case is essential: otherwise a
		 * battery-depleted RTC (garbage registers, VL/OSF/PORF latched) would
		 * never become the write-back target, so a sync could never
		 * re-initialise it and the clock would stay blank forever. */
		uint8_t sb = blk[0] & 0x7F, mb = blk[1] & 0x7F, hb = blk[2] & 0x3F;
		uint8_t db = blk[d->date_index] & 0x3F, ob = blk[5] & 0x1F, yb = blk[6];

		bool bcd_ok = bcd_field_ok(sb, 59) && bcd_field_ok(mb, 59) &&
			      bcd_field_ok(hb, 23) && bcd_field_ok(db, 31) &&
			      bcd_field_ok(ob, 12) && bcd_field_ok(yb, 99) &&
			      BCD2BIN(db) >= 1 && BCD2BIN(ob) >= 1;
		bool unreliable = rtc_time_unreliable(d, blk);

		if (!bcd_ok && !unreliable) {
			continue;  /* neither valid time nor a lost-power RTC => skip */
		}

		if (s_active == NULL) {
			s_active = d;  /* RTC => our write-back target */
		}

		if (unreliable) {
			LOG_WRN("%s present, power-loss flag set — clock will be set "
				"on the next GPS/app/CLI sync", d->name);
			continue;
		}

		unsigned sec   = BCD2BIN(sb);
		unsigned min   = BCD2BIN(mb);
		unsigned hour  = BCD2BIN(hb);
		unsigned day   = BCD2BIN(db);
		unsigned month = BCD2BIN(ob);
		unsigned year  = 2000 + BCD2BIN(yb);

		if (year < 2025) {
			LOG_INF("%s present, time not yet set (%04u-%02u-%02u)",
				d->name, year, month, day);
			continue;  /* RTC adopted for write-back, but no valid time */
		}

		int64_t e = days_from_civil((int)year, month, day) * 86400LL +
			    hour * 3600 + min * 60 + sec;
		if (epoch_out) {
			*epoch_out = (uint32_t)e;
		}
		LOG_INF("RTC %s: restored %04u-%02u-%02u %02u:%02u:%02u UTC",
			d->name, year, month, day, hour, min, sec);
		return true;
	}
	return false;
}

bool zephcore_rtc_restore(uint32_t *epoch_out)
{
	s_probed = true;
	return rtc_probe(epoch_out);
}

void zephcore_rtc_save(uint32_t epoch)
{
	if (!s_probed) {
		/* Restore wasn't run (unexpected) — discover now. */
		(void)rtc_probe(NULL);
		s_probed = true;
	}
	if (s_active == NULL) {
		return;
	}

	const struct rtc_desc *d = s_active;
	int y;
	unsigned m, day;
	civil_from_days((int64_t)epoch / 86400, &y, &m, &day);
	unsigned rem  = epoch % 86400;
	unsigned hour = rem / 3600;
	unsigned min  = (rem % 3600) / 60;
	unsigned sec  = rem % 60;
	unsigned dow  = (unsigned)(((epoch / 86400) + 4) % 7);  /* 1970-01-01 = Thu */

	uint8_t blk[7];
	blk[0] = BIN2BCD(sec);
	blk[1] = BIN2BCD(min);
	blk[2] = BIN2BCD(hour);
	/* weekday occupies whichever of index 3/4 the date doesn't. */
	blk[d->date_index] = BIN2BCD(day);
	blk[d->date_index == 4 ? 3 : 4] = (uint8_t)dow;
	blk[5] = BIN2BCD(m);
	blk[6] = BIN2BCD((unsigned)(y % 100));

	if (i2c_burst_write(d->bus, d->addr, d->time_reg, blk, sizeof(blk)) != 0) {
		LOG_WRN("RTC %s: time write failed", d->name);
		return;
	}

	/* Clear the power-loss flag (for chips whose flag is a separate reg;
	 * the seconds-bit chips clear it implicitly when we wrote sec above). */
	if (d->status_reg != RTC_STATUS_IN_SECONDS) {
		uint8_t st;
		if (i2c_reg_read_byte(d->bus, d->addr, d->status_reg, &st) == 0) {
			(void)i2c_reg_write_byte(d->bus, d->addr, d->status_reg,
						 st & (uint8_t)~d->status_mask);
		}
	}
	LOG_DBG("RTC %s: persisted time", d->name);
}

#else  /* no zephcore,rtc-i2c node in DT — link-compatible stubs */

bool zephcore_rtc_restore(uint32_t *epoch_out)
{
	ARG_UNUSED(epoch_out);
	return false;
}

void zephcore_rtc_save(uint32_t epoch)
{
	ARG_UNUSED(epoch);
}

#endif /* DT_HAS_COMPAT_STATUS_OKAY(zephcore_rtc_i2c) */
