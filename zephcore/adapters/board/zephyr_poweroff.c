/*
 * SPDX-License-Identifier: MIT
 *
 * See zephyr_poweroff.h.
 *
 * On nRF52 the SoC enters System OFF (~1 µA) but GPIO output latches and
 * SENSE bits persist across the transition — so we must explicitly:
 *   - hold every power-enable GPIO LOW so external chips don't keep drawing
 *   - hold the LoRa radio in HW reset (its internal duty cycle would
 *     otherwise keep cycling autonomously, drawing mA)
 *   - configure SENSE on sw0 so a button press wakes the chip (the nRF GPIO
 *     driver doesn't honour the DT wakeup-source property, so the dtsi
 *     marker is inert without this)
 *
 * Non-nRF platforms skip the SENSE block; they rely on Zephyr's wakeup-source
 * DT property which is honoured by their respective GPIO drivers.
 *
 * Boards with a soft-power rail (the MCU latches its own supply on) add a
 * "zephcore,poweroff-gpios" node; its pins are released last, which cuts the
 * rail outright instead of leaving it latched through System OFF.
 */

#include "zephyr_poweroff.h"

#include <ZephyrGPSManager.h>
#include <ZephyrFsUtil.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
#include <hal/nrf_gpio.h>
#endif

LOG_MODULE_REGISTER(zephcore_poweroff, CONFIG_ZEPHCORE_BOARD_LOG_LEVEL);

/* ========== Shutdown reason ========== */

#define SHUTDOWN_FILE "/lfs/shutdn"

/* Written by firmware before the codes were upstream's. */
#define SHUTDOWN_LEGACY_LOW_BATTERY 1

void zephcore_shutdown_reason_save(uint8_t reason)
{
	(void)zephcore_fs_atomic_replace(SHUTDOWN_FILE, &reason, 1, "shutdown marker");
}

uint8_t zephcore_shutdown_reason(void)
{
	static bool taken;
	static uint8_t reason = ZC_SHUTDOWN_NONE;

	if (taken) {
		return reason;
	}
	taken = true;

	/* No marker is the normal case. Probe first: a failed open is logged at
	 * ERR by Zephyr's FS layer. */
	if (!zephcore_fs_exists(SHUTDOWN_FILE)) {
		return reason;
	}

	uint8_t code = 0;
	size_t len = 0;

	if (zephcore_fs_read_file(SHUTDOWN_FILE, &code, sizeof(code), &len) && len >= 1) {
		reason = (code == SHUTDOWN_LEGACY_LOW_BATTERY) ? ZC_SHUTDOWN_LOW_VOLTAGE : code;
	}
	/* Read or stray, it must not be reported twice. */
	zephcore_fs_remove(SHUTDOWN_FILE);
	return reason;
}

/* Upstream's strings (NRF52Board::getShutdownReasonString). */
const char *zephcore_shutdown_reason_str(uint8_t reason)
{
	switch (reason) {
	case ZC_SHUTDOWN_NONE:        return "None";
	case ZC_SHUTDOWN_LOW_VOLTAGE: return "Low Voltage";
	case ZC_SHUTDOWN_USER:        return "User Request";
	}
	return "Unknown";
}

/* ========== Power-off ========== */

__weak void ui_before_power_off(void)
{
}

/* Sensor and buzzer power-gate regulators. GPS has its own; never BLE (that
 * corrupts controller state across the wake). */
static void power_regulators_off(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(sensor_power))
	const struct device *sensor_reg = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(sensor_power));

	if (sensor_reg && device_is_ready(sensor_reg)) {
		regulator_disable(sensor_reg);
	}
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(buzzer_enable))
	const struct device *buzz_reg = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(buzzer_enable));

	if (buzz_reg && device_is_ready(buzz_reg)) {
		regulator_disable(buzz_reg);
	}
#endif
}

void zephcore_power_off(void)
{
	LOG_INF("Powering off");

	/* 1. The UI: heartbeat LED, display (EPD keeps its image). */
	ui_before_power_off();

	/* 2. Power-enable GPIOs and rails off so peripherals don't keep drawing. */
	gps_power_off_for_shutdown();
	power_regulators_off();

	/* 3. Hold LoRa radio in HW reset.
	 * SX126x/LR11xx duty-cycle mode would otherwise keep the radio cycling
	 * autonomously (mA) while the SoC is in System OFF.  nRF52 output
	 * latches persist across System OFF → chip stays in reset (~0 µA). */
#if DT_NODE_EXISTS(DT_ALIAS(lora0)) && DT_NODE_HAS_PROP(DT_ALIAS(lora0), reset_gpios)
	{
		static const struct gpio_dt_spec lora_reset =
			GPIO_DT_SPEC_GET(DT_ALIAS(lora0), reset_gpios);
		gpio_pin_configure_dt(&lora_reset, GPIO_OUTPUT_ACTIVE);
	}
#endif

	/* 4. Configure GPIO SENSE for sw0 button wakeup, after waiting for the
	 * user to release any held button (otherwise DETECT is already asserted
	 * when we enter System OFF and the chip never sleeps cleanly).
	 * nRF only — other platforms rely on the DT wakeup-source property. */
#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF) && DT_NODE_EXISTS(DT_ALIAS(sw0))
	{
#define _SW0_NODE  DT_ALIAS(sw0)
#define _SW0_PORT  DT_PROP(DT_GPIO_CTLR(_SW0_NODE, gpios), port)
#define _SW0_PIN   DT_GPIO_PIN(_SW0_NODE, gpios)
#define _SW0_FLAGS DT_GPIO_FLAGS(_SW0_NODE, gpios)

		static const struct gpio_dt_spec sw0 =
			GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
		gpio_pin_configure_dt(&sw0, GPIO_INPUT);

		int64_t deadline = k_uptime_get() + 5000;
		while (gpio_pin_get_dt(&sw0) && k_uptime_get() < deadline) {
			k_sleep(K_MSEC(10));
		}

		nrf_gpio_cfg_sense_input(
			NRF_GPIO_PIN_MAP(_SW0_PORT, _SW0_PIN),
			(_SW0_FLAGS & GPIO_PULL_UP)   ? NRF_GPIO_PIN_PULLUP   :
			(_SW0_FLAGS & GPIO_PULL_DOWN) ? NRF_GPIO_PIN_PULLDOWN :
						       NRF_GPIO_PIN_NOPULL,
			(_SW0_FLAGS & GPIO_ACTIVE_LOW) ? NRF_GPIO_PIN_SENSE_LOW
						       : NRF_GPIO_PIN_SENSE_HIGH);
#undef _SW0_NODE
#undef _SW0_PORT
#undef _SW0_PIN
#undef _SW0_FLAGS
	}
#endif /* CONFIG_SOC_FAMILY_NORDIC_NRF && sw0 */

	/* 5. Release the board power latch — must be dead last.
	 *
	 * Only present on soft-power boards (see zephcore,poweroff-gpios). The
	 * rail is cut here rather than in step 2 because the pins are ordered
	 * loads-first/latch-last, and because step 4 must have observed the
	 * button release first: on these boards the button is also the power-on
	 * input, so dropping the latch while it is still held would let the I/O
	 * controller re-latch the rail immediately.
	 *
	 * On battery this does not return — the supply is gone mid-loop, which
	 * is the intended outcome. On USB the rail may be held up externally, in
	 * which case we fall through to System OFF. */
#if DT_HAS_COMPAT_STATUS_OKAY(zephcore_poweroff_gpios)
	{
#define _PWROFF_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephcore_poweroff_gpios)
		static const struct gpio_dt_spec poweroff_gpios[] = {
			DT_FOREACH_PROP_ELEM_SEP(_PWROFF_NODE, gpios,
						 GPIO_DT_SPEC_GET_BY_IDX, (,))
		};

		for (size_t i = 0; i < ARRAY_SIZE(poweroff_gpios); i++) {
			gpio_pin_configure_dt(&poweroff_gpios[i], GPIO_OUTPUT_INACTIVE);
		}
#undef _PWROFF_NODE
	}
#endif

	/* 6. System OFF. */
#ifdef CONFIG_POWEROFF
	sys_poweroff();
#else
	LOG_WRN("CONFIG_POWEROFF not enabled: rebooting instead");
	sys_reboot(SYS_REBOOT_COLD);
#endif
	CODE_UNREACHABLE;
}
