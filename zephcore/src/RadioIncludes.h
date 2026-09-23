/*
 * SPDX-License-Identifier: MIT
 * ZephCore - Common radio includes for LoRa mesh roles
 *
 * Shared by companion, repeater, and any future role that uses the
 * LoRa radio.  Selects LR1110 or SX126x based on Kconfig/devicetree.
 */

#ifndef ZEPHCORE_RADIO_INCLUDES_H
#define ZEPHCORE_RADIO_INCLUDES_H

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110) || DT_NODE_HAS_STATUS(DT_ALIAS(lora0), okay)

#include <zephyr/drivers/lora.h>
#include <helpers/LoRaConfig.h>
#include <adapters/board/ZephyrBoard.h>
#include <adapters/clock/ZephyrMillisecondClock.h>
#include <adapters/rng/ZephyrRNG.h>
#include <mesh/SimpleMeshTables.h>
#include <mesh/StaticPoolPacketManager.h>

#include <adapters/radio/LoRaRadio.h>

#define ZEPHCORE_LORA 1

#endif /* CONFIG_ZEPHCORE_RADIO_LR1110 || lora0 */

#endif /* ZEPHCORE_RADIO_INCLUDES_H */
