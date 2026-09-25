/*
 * SPDX-License-Identifier: MIT
 *
 * The radio's live state into the UI: parameters, runtime flags, counters.
 * One place for both boots, the companion's housekeeping refresh and the
 * server loop, so the roles cannot drift apart.
 */

#pragma once

#include <adapters/radio/LoRaRadio.h>
#include "ui_task.h"

static inline void ui_push_radio_state(mesh::LoRaRadio &radio)
{
	ui_set_radio_params(radio.getActiveFrequencyHz(), radio.getActiveSpreadingFactor(),
			    radio.getActiveBandwidthKHzX10(), radio.getActiveCodingRate(),
			    radio.getConfiguredTxPower(), radio.getNoiseFloor());
	ui_set_radio_runtime(radio.getActiveSyncWord(), radio.getActivePreambleLength(),
			     radio.isRxDutyCycleEnabled(), radio.isRadioReady(),
			     radio.isInRecvMode(), radio.isTxActive());
	ui_set_radio_stats(radio.getPacketsRecv(), radio.getPacketsSent(),
			   radio.getPacketsRecvErrors());
}
