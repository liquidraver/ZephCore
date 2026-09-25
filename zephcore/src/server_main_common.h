/*
 * SPDX-License-Identifier: MIT
 * ZephCore - shared composition root of the server roles (repeater, room server)
 *
 * The USB text CLI, the radio/GPS/RTC glue, the deadline-driven event loop and
 * the boot sequence are the same for both roles. Each role's main file owns
 * its mesh object and hands server_main() a ServerRole describing it.
 */

#pragma once

#include <stdint.h>
#include <app/RepeaterDataStore.h>
#include <helpers/CommonCLI.h>
#include <helpers/NodePrefs.h>
#include <adapters/clock/ZephyrRTCClock.h>

/* Radio + mesh includes (shared header selects LR1110 or SX126x) */
#include <src/RadioIncludes.h>

/* What the shared code needs from a role. The function pointers exist because
 * loop(), begin() and handleCommand() are not virtual on the mesh classes. */
struct ServerRole {
	const char *name;                  /* "Repeater", "Room Server": banner and logs */
	const char *name_prefix;           /* default node name is "<prefix>-<device id>" */
#ifdef ZEPHCORE_LORA
	mesh::Mesh *mesh;
	CommonCLICallbacks *callbacks;     /* savePrefs(), sendSelfAdvertisement() */
	NodePrefs *prefs;
	void (*begin)(RepeaterDataStore *store);
	void (*loop)(void);
	void (*handle_command)(char *command, char *reply);
	void (*note_gps_time_sync)(void);
#endif
};

/* Runs the role; never returns. */
int server_main(const ServerRole &role);

/* The hardware singletons the role's mesh object is built from. */
extern mesh::ZephyrRTCClock rtc_clock;
#ifdef ZEPHCORE_LORA
extern mesh::ZephyrBoard zephyr_board;
extern mesh::LoRaRadio lora_radio;
extern mesh::ZephyrMillisecondClock ms_clock;
extern mesh::ZephyrRNG zephyr_rng;
extern SimpleMeshTables mesh_tables;
#endif
