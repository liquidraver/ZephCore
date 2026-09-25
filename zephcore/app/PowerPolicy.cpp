/*
 * SPDX-License-Identifier: MIT
 * See PowerPolicy.h.
 */

#include "PowerPolicy.h"

#include <adapters/board/zephyr_poweroff.h>
#include <helpers/ui/ui_task.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(zephcore_power_policy, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

#if defined(CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS) && \
	CONFIG_ZEPHCORE_AUTO_SHUTDOWN_MILLIVOLTS > 0
#define HAS_AUTO_SHUTDOWN 1
#else
#define HAS_AUTO_SHUTDOWN 0
#endif

#define CHECK_INTERVAL_MS  30000   /* one battery reading per 30 s */
#define CONFIRM_COUNT      3       /* 3 x 30 s: a TX sag cannot trigger either */
#define ALERT_REARM_MV     150     /* the alert re-arms this far above its threshold */

/* With an app connected, the power-off waits this long so the notice's
 * notify -> fetch -> send round-trip can finish. */
#define SHUTDOWN_GRACE_MS  1000

#if HAS_AUTO_SHUTDOWN
static void shutdown_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	zephcore_power_off();
}
static K_WORK_DELAYABLE_DEFINE(s_shutdown_work, shutdown_work_fn);
#endif

void PowerPolicy::tick()
{
	if (_shutting_down) {
		return;  /* committed; the grace work powers off */
	}
	uint32_t now = k_uptime_get_32();

	if (_checked && (now - _next_check_ms) < CHECK_INTERVAL_MS) {
		return;
	}
	_checked = true;
	_next_check_ms = now;

	uint16_t mv = _board.getBattMilliVolts();

	if (mv < NO_CELL_MV) {
		/* No battery (or no battery ADC): nothing to protect, nothing to say. */
		_alert_low = 0;
		_shutdown_low = 0;
		return;
	}
	bool on_battery = !_board.isExternalPowered();

	checkAlert(mv, on_battery);
	checkShutdown(mv, on_battery);
}

uint16_t PowerPolicy::alertThresholdMv() const
{
	uint16_t pref = _prefs.v_battery_alert_mv;

	if (pref != 0xFFFF) {
		return pref;
	}
	uint16_t cutoff = HAS_AUTO_SHUTDOWN ? _prefs.auto_shutdown_mv : 0;

	return cutoff ? (uint16_t)(cutoff + 200) : 3500;
}

void PowerPolicy::checkAlert(uint16_t mv, bool on_battery)
{
	uint16_t thresh = alertThresholdMv();

	if (thresh == 0 || !_hooks.alert_enabled || !_hooks.alert_enabled()) {
		return;
	}
	if (!on_battery || mv >= thresh + ALERT_REARM_MV) {
		rearmAlert();  /* charging or recovered: ready for the next discharge */
		return;
	}
	if (mv >= thresh) {
		_alert_low = 0;
		return;
	}
	if (_alert_latched || ++_alert_low < CONFIRM_COUNT) {
		return;
	}
	_alert_latched = true;
	if (_hooks.battery_alert) {
		_hooks.battery_alert(mv, thresh);
	}
}

void PowerPolicy::checkShutdown(uint16_t mv, bool on_battery)
{
#if HAS_AUTO_SHUTDOWN
	uint16_t cutoff = _prefs.auto_shutdown_mv;

	if (cutoff == 0 || mv >= cutoff) {
		_shutdown_low = 0;
		return;
	}
	/* The reading is the cell, not the supply: never cut a node on a bench
	 * cable or a charger. */
	if (!on_battery) {
		LOG_INF("auto-shutdown: %u mV below %u mV but externally powered", mv, cutoff);
		_shutdown_low = 0;
		return;
	}
	LOG_WRN("auto-shutdown: battery %u mV < %u mV (%u/%u)", mv, cutoff,
		_shutdown_low + 1, CONFIRM_COUNT);
	if (++_shutdown_low < CONFIRM_COUNT) {
		return;
	}

	LOG_WRN("auto-shutdown: confirmed, powering off");
	_shutting_down = true;
	bool grace = _hooks.notify_shutdown && _hooks.notify_shutdown();

	if (!grace) {
		/* Nobody heard it live: the next boot reports it. */
		zephcore_shutdown_reason_save(ZC_SHUTDOWN_LOW_VOLTAGE);
	}
#ifdef CONFIG_POWEROFF
	if (grace) {
		ui_show_low_battery(false);  /* draw only: the main loop must deliver */
		k_work_schedule(&s_shutdown_work, K_MSEC(SHUTDOWN_GRACE_MS));
		return;
	}
	ui_show_low_battery(true);
	zephcore_power_off();
#else
	ui_show_low_battery(true);
	LOG_WRN("auto-shutdown: CONFIG_POWEROFF not enabled, cannot power off");
#endif
#else
	ARG_UNUSED(mv);
	ARG_UNUSED(on_battery);
#endif
}

bool PowerPolicy::handleCommand(const char *line, char *reply, size_t cap)
{
#if HAS_AUTO_SHUTDOWN
	if (strcmp(line, "get autoshutdown") == 0) {
		uint16_t mv = _prefs.auto_shutdown_mv;

		if (mv == 0) {
			snprintf(reply, cap, "autoshutdown: off");
		} else {
			snprintf(reply, cap, "autoshutdown: %u mV", mv);
		}
		return true;
	}
	if (strncmp(line, "set autoshutdown ", 17) == 0) {
		const char *arg = line + 17;

		while (*arg == ' ') {
			arg++;
		}
		/* Digits only, then nothing but whitespace. */
		char *end = NULL;
		long v = strtol(arg, &end, 10);

		while (*end == ' ' || *end == '\r' || *end == '\n' || *end == '\t') {
			end++;
		}
		if (arg[0] < '0' || arg[0] > '9' || *end != '\0') {
			snprintf(reply, cap, "ERROR: numbers only (0 = off, 1-5000 mV)");
			return true;
		}
		if (v > 5000) {
			snprintf(reply, cap, "ERROR: must be 0 (off) or 1-5000 mV");
			return true;
		}
		_prefs.auto_shutdown_mv = (uint16_t)v;
		_prefs.auto_shutdown_set = 1;
		if (_hooks.prefs_dirty) {
			_hooks.prefs_dirty();
		}
		_shutdown_low = 0;
		if (v == 0) {
			snprintf(reply, cap, "OK - autoshutdown off");
		} else {
			snprintf(reply, cap, "OK - autoshutdown %ld mV", v);
		}
		return true;
	}
#else
	ARG_UNUSED(line);
	ARG_UNUSED(reply);
	ARG_UNUSED(cap);
#endif
	return false;
}
