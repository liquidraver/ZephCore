/*
 * SPDX-License-Identifier: MIT
 * Event bits of the mesh thread's k_event, shared by every role.
 *
 * Each role's main loop blocks in k_event_wait() on these; ISR and work-queue
 * callbacks post them. Bits 0-6 mean the same thing in every role that has
 * them; bits 2 and 3 are the role's transport input and time tick under
 * role-specific names. Role-only bits start at MESH_EVENT_ROLE_BASE.
 */

#pragma once

#include <zephyr/sys/util.h>

#define MESH_EVENT_LORA_RX       BIT(0)  /* LoRa packet received */
#define MESH_EVENT_LORA_TX_DONE  BIT(1)  /* LoRa TX complete */

/* Transport input */
#define MESH_EVENT_BLE_RX        BIT(2)  /* companion: BLE/USB frame received */
#define MESH_EVENT_CLI_RX        BIT(2)  /* servers, observer: USB CLI line received */

/* Time tick */
#define MESH_EVENT_HOUSEKEEPING  BIT(3)  /* companion: periodic housekeeping */
#define MESH_EVENT_MAINTENANCE   BIT(3)  /* servers: a maintenance deadline came due */

#define MESH_EVENT_GPS_ACTION    BIT(4)  /* GPS state change or fix (must run on the main thread) */
#define MESH_EVENT_TX_DRAIN      BIT(5)  /* outbound packet delay expired, run checkSend */
/* BIT(6) free: was MESH_EVENT_RTC_SAVE, until ZephyrRTCClock wrote the hardware RTC itself */

#define MESH_EVENT_ROLE_BASE     7       /* first role-only bit */
