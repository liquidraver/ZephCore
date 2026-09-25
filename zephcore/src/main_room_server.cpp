/*
 * SPDX-License-Identifier: MIT
 * ZephCore - Room Server
 *
 * The room server (shared BBS) role's entry point: its mesh object, and the
 * ServerRole that hands it to the shared server code (server_main_common.cpp).
 */

#include "server_main_common.h"
#include <app/RoomServerMesh.h>

#ifdef ZEPHCORE_LORA
/* RoomServerMesh requires: board, radio, ms_clock, rng, rtc, tables */
static RoomServerMesh room_mesh(zephyr_board, lora_radio, ms_clock, zephyr_rng, rtc_clock, mesh_tables);
#endif

static const ServerRole room_role = {
	.name = "Room Server",
	.name_prefix = "Room",
#ifdef ZEPHCORE_LORA
	.mesh = &room_mesh,
	.callbacks = &room_mesh,
	.prefs = room_mesh.getNodePrefs(),
	.begin = [](RepeaterDataStore *store) { room_mesh.begin(store); },
	.loop = [] { room_mesh.loop(); },
	.handle_command = [](char *command, char *reply) { room_mesh.handleCommand(0, command, reply); },
	.note_gps_time_sync = [] { room_mesh.noteGPSTimeSync(); },
#endif
};

int main(void)
{
	return server_main(room_role);
}
