/*
 * SPDX-License-Identifier: MIT
 * No-op stubs for the UI API on headless builds.
 *
 * Companion code (CompanionMesh, main_companion, ui_mesh_actions) calls
 * ui_init / ui_notify / ui_set_* / etc. unconditionally -- these are normally
 * provided by helpers/ui-button/ui_task.c or helpers/ui-joystick/.
 * On a Linux SBC build with no display/buttons/buzzer those source files
 * are not compiled, so we provide weak no-op implementations here.
 *
 * Signatures must match ui_task.h. Weak symbols are overridden by the
 * real ui_task.c on any board where a UI variant is enabled, so this
 * file is harmless to include in every build (we only add it when no
 * UI variant is active).
 */

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#include "ui_task.h"
#include "led_gate.h"

#define WEAK __attribute__((weak))

WEAK int  ui_init(void)                                              { return 0; }
WEAK void ui_play_startup_chime(void)                                { }
WEAK void ui_led_heartbeat_init(void)                                { }

WEAK void ui_notify(enum ui_event event)
{
	ARG_UNUSED(event);
}

WEAK void ui_set_msg_count(uint16_t count)
{
	ARG_UNUSED(count);
}

WEAK void ui_set_radio_params(uint32_t freq_hz, uint8_t sf,
			       uint16_t bw_khz_x10, uint8_t cr,
			       int8_t tx_power, int16_t noise_floor)
{
	ARG_UNUSED(freq_hz); ARG_UNUSED(sf); ARG_UNUSED(bw_khz_x10);
	ARG_UNUSED(cr); ARG_UNUSED(tx_power); ARG_UNUSED(noise_floor);
}

WEAK void ui_set_radio_runtime(uint8_t sync_word,
			       uint16_t preamble_len, bool rx_duty_cycle,
			       bool radio_ready, bool in_rx, bool tx_active)
{
	ARG_UNUSED(sync_word);
	ARG_UNUSED(preamble_len); ARG_UNUSED(rx_duty_cycle);
	ARG_UNUSED(radio_ready); ARG_UNUSED(in_rx); ARG_UNUSED(tx_active);
}

WEAK void ui_set_radio_stats(uint32_t packets_rx, uint32_t packets_tx,
			     uint32_t packets_err)
{
	ARG_UNUSED(packets_rx); ARG_UNUSED(packets_tx); ARG_UNUSED(packets_err);
}

WEAK void ui_set_gps_data(bool has_fix, uint8_t sats,
			   int32_t lat_mdeg, int32_t lon_mdeg, int32_t alt_mm)
{
	ARG_UNUSED(has_fix); ARG_UNUSED(sats);
	ARG_UNUSED(lat_mdeg); ARG_UNUSED(lon_mdeg); ARG_UNUSED(alt_mm);
}

WEAK void ui_set_battery(uint16_t mv, uint8_t pct)
{
	ARG_UNUSED(mv); ARG_UNUSED(pct);
}

WEAK void ui_set_clock(uint32_t epoch)
{
	ARG_UNUSED(epoch);
}

WEAK void ui_set_tz(int8_t hours)
{
	ARG_UNUSED(hours);
}

WEAK void ui_add_recent(const char *name, int16_t rssi, uint32_t age_s)
{
	ARG_UNUSED(name); ARG_UNUSED(rssi); ARG_UNUSED(age_s);
}

WEAK void ui_set_node_name(const char *name)
{
	ARG_UNUSED(name);
}

WEAK void ui_clear_recent(void) { }

WEAK void ui_set_gps_available(bool available)
{
	ARG_UNUSED(available);
}

WEAK void ui_set_gps_enabled(bool enabled)
{
	ARG_UNUSED(enabled);
}

WEAK void ui_set_gps_state(uint8_t state, uint32_t last_fix_age_s,
			    uint32_t next_search_s)
{
	ARG_UNUSED(state); ARG_UNUSED(last_fix_age_s); ARG_UNUSED(next_search_s);
}

WEAK void ui_set_ble_enabled(bool enabled)
{
	ARG_UNUSED(enabled);
}

WEAK void ui_set_buzzer_mode(uint8_t mode)
{
	ARG_UNUSED(mode);
}

/* Not a no-op: a headless build still has the LoRa TX LED, and the gate that
 * governs it lives outside the UI layer precisely so this case works. */
WEAK void ui_set_leds_disabled(bool disabled)
{
	zephcore_leds_set_disabled(disabled);
}

WEAK void ui_set_heartbeat_led(bool enabled)
{
	ARG_UNUSED(enabled);
}

WEAK void ui_set_offgrid_mode(bool enabled)
{
	ARG_UNUSED(enabled);
}

WEAK void ui_set_battery_provider(uint16_t (*mv)(void), uint8_t (*pct)(void))
{
	ARG_UNUSED(mv);
	ARG_UNUSED(pct);
}

WEAK void ui_show_low_battery(bool hold) { ARG_UNUSED(hold); }

WEAK void ui_refresh_battery(void) { }
WEAK void ui_invalidate_battery_cache(void) { }

WEAK void ui_notify_contact_msg(uint8_t path_len, const char *from_name,
				 const char *text, uint16_t msg_count)
{
	ARG_UNUSED(path_len); ARG_UNUSED(from_name);
	ARG_UNUSED(text); ARG_UNUSED(msg_count);
}

WEAK void ui_notify_channel_msg(const char *channel_name, const char *text,
				 uint32_t ts, uint8_t path_len,
				 uint16_t msg_count)
{
	ARG_UNUSED(channel_name); ARG_UNUSED(text); ARG_UNUSED(ts);
	ARG_UNUSED(path_len); ARG_UNUSED(msg_count);
}

WEAK void ui_notify_packet_sent(void) { }

/* Input axis flip. The real state lives in ui_common.c, which is compiled
 * whenever any UI hardware is present; a headless build has no input to flip,
 * so the setter is a no-op and the mapper is the identity. The CLI calls
 * these unconditionally. */
WEAK void zephcore_input_set_flipped(bool flipped) { ARG_UNUSED(flipped); }
WEAK bool zephcore_input_is_flipped(void) { return false; }
WEAK uint16_t zephcore_input_map_code(uint16_t code) { return code; }
