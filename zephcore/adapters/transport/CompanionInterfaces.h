/*
 * SPDX-License-Identifier: MIT
 * The companion transports as upstream BaseSerialInterfaces, for one
 * MultiSerialInterface (upstream's companion_radio/main.cpp shape): BLE, the
 * wired link (USB CDC or UART) and TCP (native Linux, WiFi).
 *
 * Each is a thin wrapper over its C transport. txIdle() is ours: the
 * reboot-class CLI commands wait for every transport's TX to drain.
 */

#pragma once

#include <helpers/BaseSerialInterface.h>
#include "companion_framing.h"

/* The wired transport (ZephyrCompanionUSB) is compiled when logging needs the
 * CDC console (debug builds), the native-USB companion is enabled, or the
 * plain-UART companion is. MUST match the CMake condition that compiles
 * ZephyrCompanionUSB.cpp. */
#define ZEPHCORE_USB_STACK \
	(IS_ENABLED(CONFIG_LOG) || IS_ENABLED(CONFIG_ZEPHCORE_COMPANION_USB) || \
	 IS_ENABLED(CONFIG_ZEPHCORE_COMPANION_SERIAL))

#if IS_ENABLED(CONFIG_BT)
#include <ZephyrBLE.h>

class ZephyrBLEInterface : public BaseSerialInterface {
public:
	void enable() override { zephcore_ble_set_enabled(true); }
	void disable() override { zephcore_ble_set_enabled(false); }
	bool isEnabled() const override { return zephcore_ble_is_enabled(); }
	bool isConnected() const override { return zephcore_ble_is_active(); }
	bool isWriteBusy() const override { return zephcore_ble_is_write_busy(); }
	size_t writeFrame(const uint8_t src[], size_t len) override {
		return zephcore_ble_send(src, (uint16_t)len);
	}
	size_t checkRecvFrame(uint8_t dest[]) override { return zephcore_ble_recv(dest); }
	bool txIdle() const { return zephcore_ble_tx_idle(); }
};
#endif

#if ZEPHCORE_USB_STACK
#include <ZephyrCompanionUSB.h>

/* Always enabled: the port is there whether or not a host has opened it. */
class ZephyrSerialInterface : public BaseSerialInterface {
public:
	void enable() override {}
	void disable() override {}
	bool isEnabled() const override { return true; }
	bool isConnected() const override { return zephcore_usb_companion_is_connected(); }
	bool isWriteBusy() const override { return zephcore_usb_companion_is_write_busy(); }
	size_t writeFrame(const uint8_t src[], size_t len) override {
		return zephcore_usb_companion_write_frame(src, len);
	}
	size_t checkRecvFrame(uint8_t dest[]) override { return zephcore_usb_companion_recv(dest); }
	bool txIdle() const { return zephcore_usb_companion_tx_idle(); }
};
#endif

#if IS_ENABLED(CONFIG_ZEPHCORE_TRANSPORT_TCP)
#include "TcpCompanionTransport.h"

class ZephyrTcpInterface : public BaseSerialInterface {
public:
	void enable() override { tcp_companion_set_enabled(true); }
	void disable() override { tcp_companion_set_enabled(false); }
	bool isEnabled() const override { return tcp_companion_is_enabled(); }
	bool isConnected() const override { return tcp_companion_is_connected(); }
	bool isWriteBusy() const override { return tcp_companion_is_write_busy(); }
	size_t writeFrame(const uint8_t src[], size_t len) override {
		return tcp_companion_send(src, (uint16_t)len);
	}
	size_t checkRecvFrame(uint8_t dest[]) override { return tcp_companion_recv(dest); }
	bool txIdle() const { return tcp_companion_tx_idle(); }
};
#endif
