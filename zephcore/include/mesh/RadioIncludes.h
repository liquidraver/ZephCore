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
#include <mesh/LoRaConfig.h>
#include <adapters/board/ZephyrBoard.h>
#include <adapters/clock/ZephyrMillisecondClock.h>
#include <adapters/rng/ZephyrRNG.h>
#include <mesh/SimpleMeshTables.h>
#include <mesh/StaticPoolPacketManager.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR1110)
#include <adapters/radio/LR1110Radio.h>
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_LR2021)
#include <adapters/radio/LR2021Radio.h>
#elif IS_ENABLED(CONFIG_ZEPHCORE_RADIO_SX127X)
#include <adapters/radio/SX127xRadio.h>
#else
#include <adapters/radio/SX126xRadio.h>
#endif

#define ZEPHCORE_LORA 1

#endif /* CONFIG_ZEPHCORE_RADIO_LR1110 || lora0 */

#endif /* ZEPHCORE_RADIO_INCLUDES_H */
