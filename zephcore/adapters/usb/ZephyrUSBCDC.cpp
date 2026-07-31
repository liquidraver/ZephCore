/*
 * SPDX-License-Identifier: MIT
 * ZephCore — Unified USB CDC ACM init (companion + repeater + observer)
 *
 * See ZephyrUSBCDC.h for the contract. This implementation:
 *   - Defines a single USBD context + CDC ACM class
 *   - Registers usbd_msg_callback so the host's SET_CONTROL_LINE_STATE and
 *     SET_LINE_CODING requests reach us as events (no DTR polling)
 *   - Emits a k_event when DTR transitions high — the boot path waits on it
 *     instead of a fixed sleep
 *   - Fires a user callback on every DTR transition (companion: reset RX +
 *     flip active_iface on drop)
 *   - Handles the Arduino-style 1200-baud touch → reboot-to-bootloader flow
 *     for nRF52 boards
 */

#include "ZephyrUSBCDC.h"

#include <stdint.h>
#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_usbd, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

#include <adapters/board/ZephyrBoard.h>

/* ===== Per-role descriptor strings ====== */
#define ZEPHCORE_USB_VID         0x2fe3   /* Zephyr Project VID */

#if defined(ZEPHCORE_REPEATER)
#define ZEPHCORE_USB_PID         0x0004
#define ZEPHCORE_USB_PRODUCT     "ZephCore Repeater"
#elif defined(ZEPHCORE_OBSERVER)
#define ZEPHCORE_USB_PID         0x0005
#define ZEPHCORE_USB_PRODUCT     "ZephCore Observer"
#else
#define ZEPHCORE_USB_PID         0x0006
#define ZEPHCORE_USB_PRODUCT     "ZephCore Companion"
#endif

#define ZEPHCORE_USB_MAX_POWER   125   /* 250 mA in 2 mA units */

/* ===== USBD device + descriptors ===== */
USBD_DEVICE_DEFINE(zephcore_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   ZEPHCORE_USB_VID, ZEPHCORE_USB_PID);

USBD_DESC_LANG_DEFINE(zephcore_lang);
USBD_DESC_MANUFACTURER_DEFINE(zephcore_mfr, "ZephCore");
USBD_DESC_PRODUCT_DEFINE(zephcore_product, ZEPHCORE_USB_PRODUCT);
USBD_DESC_SERIAL_NUMBER_DEFINE(zephcore_sn);

USBD_DESC_CONFIG_DEFINE(zephcore_fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(zephcore_hs_cfg_desc, "HS Configuration");

USBD_CONFIGURATION_DEFINE(zephcore_fs_config,
			  0, ZEPHCORE_USB_MAX_POWER, &zephcore_fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(zephcore_hs_config,
			  0, ZEPHCORE_USB_MAX_POWER, &zephcore_hs_cfg_desc);

/* ===== Module state ===== */
#define USB_CDC_DTR_BIT BIT(0)
static K_EVENT_DEFINE(usb_cdc_events);

static bool s_dtr_active;
static bool s_initialized;
static zephcore_usbd_cdc_dtr_cb_t s_dtr_cb;

/* ZephyrBoard for bootloader-magic write before reset. The class is stateless
 * (only static methods over GPREGRET + sys_reboot), so a local instance here
 * is fine even when other roles also have their own instance. */
static mesh::ZephyrBoard usb_board;

static void enter_bootloader(void)
{
	LOG_WRN("Entering bootloader (1200 baud touch)");
	/* Let USB disconnect cleanly before the reset */
	k_msleep(100);
	usb_board.rebootToBootloader();
	CODE_UNREACHABLE;
}

/* ===== USBD message callback ===== */
static void usbd_msg_callback(struct usbd_context *const ctx,
			      const struct usbd_msg *msg)
{
	ARG_UNUSED(ctx);

	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		uint32_t baudrate = 0;
		if (uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_BAUD_RATE,
				       &baudrate) == 0) {
			LOG_DBG("CDC ACM baud rate: %u", baudrate);
			if (baudrate == 1200) {
				/* Host explicitly set 1200 baud — DFU intent.
				 * Adafruit nrfutil --touch 1200 closes the
				 * port immediately after; do not wait for
				 * the DTR drop to fire. */
				enter_bootloader();
			}
		}
	} else if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0;
		uint32_t baudrate = 0;
		uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_BAUD_RATE,
				   &baudrate);
		LOG_DBG("CDC ACM DTR=%u baud=%u", dtr, baudrate);

		const bool dtr_now = (dtr != 0);

		/* Arduino-style DFU: DTR drop while baud is 1200. Some host
		 * tools change baud then close (handled by LINE_CODING above),
		 * others just toggle DTR with baud still at 1200 — catch both. */
		if (s_dtr_active && !dtr_now && baudrate == 1200) {
			enter_bootloader();
		}

		s_dtr_active = dtr_now;

		if (dtr_now) {
			/* Wake the boot-time waiter; subsequent transitions
			 * leave the bit set, which is what callers expect
			 * (a "have we ever seen DTR?" gate). */
			k_event_set(&usb_cdc_events, USB_CDC_DTR_BIT);
		}

		if (s_dtr_cb) {
			s_dtr_cb(dtr_now);
		}
	} else if (msg->type == USBD_MSG_VBUS_REMOVED) {
		/* Physical unplug — VBUS lost.  On a device-side cable yank the
		 * host often never sends a clean DTR=0 line-state change, so the
		 * CONTROL_LINE_STATE release above won't fire and the companion
		 * would stay stuck on the USB interface (rejecting every BLE
		 * connection until reboot).  Treat VBUS loss as a DTR drop so the
		 * interface is handed back to BLE. */
		LOG_INF("CDC ACM: VBUS removed (device unplug)");
		if (s_dtr_active) {
			s_dtr_active = false;
			if (s_dtr_cb) {
				s_dtr_cb(false);
			}
		}
	}
}

/* ===== Setup helpers ===== */
static int register_cdc_acm(struct usbd_context *const uds_ctx,
			    const enum usbd_speed speed)
{
	struct usbd_config_node *cfg_nd;
	int err;

	if (speed == USBD_SPEED_HS) {
		cfg_nd = &zephcore_hs_config;
	} else {
		cfg_nd = &zephcore_fs_config;
	}

	err = usbd_add_configuration(uds_ctx, speed, cfg_nd);
	if (err) {
		LOG_ERR("Failed to add configuration");
		return err;
	}

	err = usbd_register_class(&zephcore_usbd, "cdc_acm_0", speed, 1);
	if (err) {
		LOG_ERR("Failed to register CDC ACM class");
		return err;
	}

	return usbd_device_set_code_triple(uds_ctx, speed,
					   USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

/* ===== Public API ===== */
extern "C" int zephcore_usbd_init(void)
{
	if (s_initialized) {
		return 0;
	}

	int err;

	LOG_INF("Initializing ZephCore USB CDC ACM (" ZEPHCORE_USB_PRODUCT ")");

	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_lang);
	if (err) {
		LOG_ERR("usbd_add_descriptor(lang) failed: %d", err);
		return err;
	}
	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_mfr);
	if (err) {
		LOG_ERR("usbd_add_descriptor(mfr) failed: %d", err);
		return err;
	}
	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_product);
	if (err) {
		LOG_ERR("usbd_add_descriptor(product) failed: %d", err);
		return err;
	}
	err = usbd_add_descriptor(&zephcore_usbd, &zephcore_sn);
	if (err) {
		LOG_ERR("usbd_add_descriptor(sn) failed: %d", err);
		return err;
	}

	if (USBD_SUPPORTS_HIGH_SPEED &&
	    usbd_caps_speed(&zephcore_usbd) == USBD_SPEED_HS) {
		err = register_cdc_acm(&zephcore_usbd, USBD_SPEED_HS);
		if (err) {
			return err;
		}
	}

	err = register_cdc_acm(&zephcore_usbd, USBD_SPEED_FS);
	if (err) {
		return err;
	}

	err = usbd_msg_register_cb(&zephcore_usbd, usbd_msg_callback);
	if (err) {
		LOG_ERR("usbd_msg_register_cb failed: %d", err);
		return err;
	}

	err = usbd_init(&zephcore_usbd);
	if (err) {
		LOG_ERR("usbd_init failed: %d", err);
		return err;
	}

	err = usbd_enable(&zephcore_usbd);
	if (err) {
		LOG_ERR("usbd_enable failed: %d", err);
		return err;
	}

	s_initialized = true;
	LOG_INF("ZephCore USB CDC initialized");
	return 0;
}

extern "C" int zephcore_usbd_wait_dtr(uint32_t timeout_ms)
{
	if (!s_initialized) {
		return -ENODEV;
	}
	uint32_t got = k_event_wait(&usb_cdc_events, USB_CDC_DTR_BIT,
				    false, K_MSEC(timeout_ms));
	return got ? 0 : -EAGAIN;
}

extern "C" bool zephcore_usbd_is_dtr_active(void)
{
	return s_dtr_active;
}

extern "C" void zephcore_usbd_set_dtr_cb(zephcore_usbd_cdc_dtr_cb_t cb)
{
	s_dtr_cb = cb;
}
