/*
 * SPDX-License-Identifier: MIT
 * ZephCore - Repeater
 *
 * The repeater role's entry point: its mesh object, and the ServerRole that
 * hands it to the shared server code (server_main_common.cpp).
 */

#include "server_main_common.h"
#include <app/RepeaterMesh.h>

#ifdef ZEPHCORE_LORA
/* RepeaterMesh requires: board, radio, ms_clock, rng, rtc, tables */
static RepeaterMesh repeater_mesh(zephyr_board, lora_radio, ms_clock, zephyr_rng, rtc_clock, mesh_tables);
#endif

static const ServerRole repeater_role = {
	.name = "Repeater",
	.name_prefix = "Repeater",
#ifdef ZEPHCORE_LORA
	.mesh = &repeater_mesh,
	.callbacks = &repeater_mesh,
	.prefs = repeater_mesh.getNodePrefs(),
	.begin = [](RepeaterDataStore *store) { repeater_mesh.begin(store); },
	.loop = [] { repeater_mesh.loop(); },
	.handle_command = [](char *command, char *reply) { repeater_mesh.handleCommand(0, command, reply); },
	.note_gps_time_sync = [] { repeater_mesh.noteGPSTimeSync(); },
#endif
};

int main(void)
{
	return server_main(repeater_role);
}
