/*
 * SPDX-License-Identifier: MIT
 *
 * The companion's low-battery policy, on every build that reads a battery,
 * with or without a UI: auto-shutdown (prefs.auto_shutdown_mv, `get|set
 * autoshutdown`) and the v-contact low-battery alert (prefs.v_battery_alert_mv).
 * Driven by the housekeeping tick, main thread only.
 *
 * Both sample the battery every 30 s and need three low readings in a row, so
 * a TX sag cannot trigger them; both skip while the board reports external
 * power. A reading under PowerPolicy::NO_CELL_MV is "no battery" (a board
 * without a cell reads 0, or a floating divider), never a reason to act.
 */

#pragma once

#include <mesh/MeshCore.h>
#include <helpers/NodePrefs.h>
#include <stddef.h>
#include <stdint.h>

class PowerPolicy {
public:
	/* Below this no Li-ion cell is running the node: no battery, not a low one. */
	static constexpr uint16_t NO_CELL_MV = 2000;

	struct Hooks {
		/* An app should hear about a low-battery shutdown now: send the notice
		 * live and return true (the power-off then waits a short grace for
		 * the app to fetch it), or return false (none connected). */
		bool (*notify_shutdown)(void);
		/* The alert threshold was crossed (once per discharge). */
		void (*battery_alert)(uint16_t mv, uint16_t threshold_mv);
		/* The alert is wanted at all (the v-contact is on). */
		bool (*alert_enabled)(void);
		/* prefs changed off the save path: coalesced write. */
		void (*prefs_dirty)(void);
	};

	PowerPolicy(mesh::MainBoard &board, NodePrefs &prefs, const Hooks &hooks)
		: _board(board), _prefs(prefs), _hooks(hooks) {}

	/* Every housekeeping tick; self-throttled to one reading per 30 s. */
	void tick();

	/* `get|set autoshutdown`; true if the line was one. */
	bool handleCommand(const char *line, char *reply, size_t cap);

	/* The alert threshold in mV, 0 = off. The 0xFFFF default is 200 mV above
	 * the auto-shutdown cutoff, so the alert comes before the shutdown; 3500 mV
	 * without a cutoff. */
	uint16_t alertThresholdMv() const;
	/* The alert threshold changed: arm it again. */
	void rearmAlert()
	{
		_alert_latched = false;
		_alert_low = 0;
	}

private:
	void checkAlert(uint16_t mv, bool on_battery);
	void checkShutdown(uint16_t mv, bool on_battery);

	mesh::MainBoard &_board;
	NodePrefs &_prefs;
	Hooks _hooks;
	uint32_t _next_check_ms = 0;
	bool _checked = false;
	uint8_t _alert_low = 0;
	bool _alert_latched = false;
	uint8_t _shutdown_low = 0;
	bool _shutting_down = false;
};
