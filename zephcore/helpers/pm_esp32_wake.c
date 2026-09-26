/*
 * SPDX-License-Identifier: MIT
 * ESP32 light sleep: wake pins, the `powersaving` switch and `get pm`.
 *
 * WHY NOT EXT1.  Zephyr's ESP32 GPIO driver turns GPIO_INT_WAKEUP into an EXT1
 * wake, and that failed three ways (devdocs/lld/13-power-management.md, F-1/F-2):
 * EXT1 takes RTC pads only (GPIO 0-21 on the S3, so not the XIAO's DIO1 on
 * GPIO39); on the S3 it has a single trigger polarity for all its pins, so an
 * active-low button and an active-high DIO1 cannot both be armed — the button,
 * configured first, won and DIO1 was silently refused; and before every sleep
 * the HAL moves each EXT1 pad to the RTC mux and nothing moves it back.
 *
 * WHAT THIS DOES INSTEAD.  ESP-IDF's GPIO wake (gpio_wakeup_enable()) works on
 * any pad in light sleep, each pin with its own level. The LoRa IRQ line and
 * the user button (DT alias sw0) become level wakes for the length of each
 * sleep and go back to their normal edge interrupts afterwards:
 *
 *   entry (IRQs locked): save the pin's interrupt type and enable, mask its CPU
 *     interrupt, arm a level wake at the pin's active level, keep the pad's
 *     input live through the sleep (sleep-select off).
 *   exit (IRQs already back on, power.c post_ops): disarm, restore the saved
 *     type, clear the status the level mode latched, unmask, and if the pin
 *     is at its active level, re-raise its interrupt through status_w1ts.
 *
 * The CPU interrupt stays masked while the pin is in level mode because on
 * this SoC the exit notifiers run after interrupts are re-enabled: a level
 * interrupt on a line the radio holds high until ClearIrqStatus would storm.
 * The re-raise is the missed edge: the level that woke us was reached while
 * the pin was in level mode, so no edge will come. The driver's own ISR then
 * runs the normal callbacks (the radio's dio1_isr, gpio-keys). A duplicate is
 * harmless: both only queue work that re-reads the hardware.
 *
 * The ESP32 GPIO wake source itself (ESP_SLEEP_WAKEUP_GPIO) is switched on by
 * `&gpio0 { wakeup-source; }` in boards/common/pm_esp32.overlay.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys/util.h>

#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#if defined(CONFIG_WIFI_ESP32)
#include <esp_wifi.h>
#endif

#include <stdio.h>
#include <string.h>

#include "pm_sleep_guard.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_pm_wake, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* ========== Wake pins from devicetree ========== */

#define ZC_LORA_NODE DT_ALIAS(lora0)
#if DT_NODE_HAS_PROP(ZC_LORA_NODE, dio1_gpios)
#define ZC_WAKE_RADIO_PROP dio1_gpios
#elif DT_NODE_HAS_PROP(ZC_LORA_NODE, irq_gpios)
#define ZC_WAKE_RADIO_PROP irq_gpios
#endif

#define ZC_BTN_NODE DT_ALIAS(sw0)
#define ZC_WAKE_HAS_BUTTON DT_NODE_HAS_PROP(ZC_BTN_NODE, gpios)

struct wake_pin {
	const char *name;
	struct gpio_dt_spec spec;
	uint8_t num;            /* SoC GPIO number, 0-48 */
	bool active_low;
	uint8_t saved_type;
	uint8_t saved_ena;
	uint32_t hits;          /* sleeps that ended with this pin active */
};

static struct wake_pin wake_pins[] = {
#ifdef ZC_WAKE_RADIO_PROP
	{ .name = "radio", .spec = GPIO_DT_SPEC_GET(ZC_LORA_NODE, ZC_WAKE_RADIO_PROP) },
#endif
#if ZC_WAKE_HAS_BUTTON
	{ .name = "btn", .spec = GPIO_DT_SPEC_GET(ZC_BTN_NODE, gpios) },
#endif
};

/* ========== Counters (`get pm`) ========== */

static struct {
	uint32_t entries;       /* times the idle path chose light sleep */
	uint32_t slept;         /* ... and the SoC actually slept */
	uint32_t wake_timer;
	uint32_t wake_gpio;
	uint32_t wake_other;
	uint64_t asleep_us;
} stats;

static int64_t entry_us;

/* power.c skips the sleep (and returns at once) when the time left to the
 * kernel deadline is under min-residency after its own lead time; a real light
 * sleep always lasts longer than this. Only used to tell the two apart for
 * the counters. */
#define SLEPT_MIN_US 500

/* ========== Sleep entry / exit ========== */

static void wake_state_entry(enum pm_state state)
{
	if (state != PM_STATE_STANDBY) {
		return;
	}

	stats.entries++;
	entry_us = esp_timer_get_time();

	ARRAY_FOR_EACH_PTR(wake_pins, p) {
		p->saved_type = GPIO.pin[p->num].int_type;
		p->saved_ena = GPIO.pin[p->num].int_ena;
		GPIO.pin[p->num].int_ena = 0;
		(void)gpio_wakeup_enable(p->num, p->active_low ? GPIO_INTR_LOW_LEVEL
							     : GPIO_INTR_HIGH_LEVEL);
		gpio_ll_sleep_sel_dis(&GPIO, p->num);
	}
}

static void wake_state_exit(enum pm_state state)
{
	if (state != PM_STATE_STANDBY) {
		return;
	}

	int64_t dt = esp_timer_get_time() - entry_us;
	bool slept = dt >= SLEPT_MIN_US;
	uint32_t causes = slept ? esp_sleep_get_wakeup_causes() : 0;

	if (slept) {
		stats.slept++;
		stats.asleep_us += (uint64_t)dt;
		if (causes & BIT(ESP_SLEEP_WAKEUP_GPIO)) {
			stats.wake_gpio++;
		} else if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
			stats.wake_timer++;
		} else {
			stats.wake_other++;
		}
	}

	ARRAY_FOR_EACH_PTR(wake_pins, p) {
		uint32_t bit = BIT(p->num & 31);

		(void)gpio_wakeup_disable(p->num);
		gpio_ll_sleep_sel_en(&GPIO, p->num);
		GPIO.pin[p->num].int_type = p->saved_type;
		if (p->num < 32) {
			GPIO.status_w1tc = bit;
		} else {
			GPIO.status1_w1tc.val = bit;
		}
		GPIO.pin[p->num].int_ena = p->saved_ena;

		bool active = gpio_ll_get_level(&GPIO, p->num) == (p->active_low ? 0 : 1);

		if (!active) {
			continue;
		}
		if (slept) {
			p->hits++;
		}
		if (p->saved_ena != 0 && p->saved_type != 0) {
			if (p->num < 32) {
				GPIO.status_w1ts = bit;
			} else {
				GPIO.status1_w1ts.val = bit;
			}
		}
	}
}

static struct pm_notifier wake_notifier = {
	.state_entry = wake_state_entry,
	.state_exit = wake_state_exit,
};

/* ========== Button: re-open the console window ========== */

#if ZC_WAKE_HAS_BUTTON
static struct gpio_callback button_cb;

static void button_pressed(const struct device *port, struct gpio_callback *cb,
			   gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	zc_pm_console_window_open();
}
#endif

/* ========== powersaving ========== */

static atomic_t powersave_blocked;

void zc_pm_set_powersaving(bool on)
{
	if (on) {
		if (atomic_cas(&powersave_blocked, 1, 0)) {
			zc_pm_unblock_sleep();
		}
	} else if (atomic_cas(&powersave_blocked, 0, 1)) {
		zc_pm_block_sleep();
	}
}

int zc_pm_format_stats(char *buf, size_t len)
{
	uint64_t up_ms = (uint64_t)k_uptime_get();
	uint32_t pct10 = up_ms ? (uint32_t)(stats.asleep_us / up_ms) : 0; /* 0.1 % units */
	uint32_t win_s = zc_pm_console_window_remaining_ms() / 1000U;
	int n = snprintf(buf, len,
			 "> %s asleep %u.%u%% sleeps %u/%u wake t%u g%u o%u",
			 atomic_get(&powersave_blocked) ? "off" : "on",
			 pct10 / 10U, pct10 % 10U,
			 stats.slept, stats.entries,
			 stats.wake_timer, stats.wake_gpio, stats.wake_other);

	ARRAY_FOR_EACH_PTR(wake_pins, p) {
		if (n > 0 && (size_t)n < len) {
			n += snprintf(buf + n, len - n, " %s%u", p->name, p->hits);
		}
	}
	if (n > 0 && (size_t)n < len && win_s > 0) {
		n += snprintf(buf + n, len - n, " win %us", win_s);
	}
	return (n > 0 && (size_t)n < len) ? n : (int)strlen(buf);
}

/* ========== esp_timer deadlines bound the sleep ========== */

#if defined(CONFIG_PM_CUSTOM_TICKS_HOOK)
/* power.c sizes each light sleep from the kernel's next timeout only.  Work
 * scheduled on esp_timer (its own systimer alarm, which does not wake the SoC)
 * would otherwise be slept through — the BLE controller's modem-sleep wake
 * timer ("btSlp") is the one that matters: miss it and the controller misses
 * its connection event.  ESP-IDF's own tickless idle bounds the sleep the same
 * way (esp_pm/pm_impl.c). */
int64_t pm_policy_next_custom_ticks(void)
{
	int64_t next = esp_timer_get_next_alarm_for_wake_up();

	if (next == INT64_MAX) {
		return -1;
	}
	int64_t delta = next - esp_timer_get_time();

	if (delta <= 0) {
		return 0;
	}
	return MIN((int64_t)k_us_to_ticks_floor64((uint64_t)delta), (int64_t)INT32_MAX);
}
#endif

/* ========== Init ========== */

static int pm_wake_init(void)
{
	ARRAY_FOR_EACH_PTR(wake_pins, p) {
		if (!gpio_is_ready_dt(&p->spec)) {
			LOG_ERR("wake pin %s: port not ready", p->name);
			return -ENODEV;
		}
		p->num = p->spec.pin;
#if DT_NODE_EXISTS(DT_NODELABEL(gpio1))
		if (p->spec.port == DEVICE_DT_GET(DT_NODELABEL(gpio1))) {
			p->num += 32;
		}
#endif
		p->active_low = (p->spec.dt_flags & GPIO_ACTIVE_LOW) != 0;
		LOG_INF("light-sleep wake: %s GPIO%u (%s)", p->name, p->num,
			p->active_low ? "low" : "high");
	}

#if defined(CONFIG_WIFI_ESP32)
	/* The ESP32 WiFi driver starts WiFi at boot, in NULL mode, "to enable
	 * coexistence".  Started, the WiFi library holds its modem PM lock
	 * (pm_policy_state_all_lock_get via esp_pm) and the SoC never
	 * light-sleeps: measured on the XIAO repeater, zero sleep entries with
	 * WiFi compiled in, sleeps without it.  Stop it here.  Nothing is lost:
	 * the driver's AP enable (`start ota`) and STA connect (uplink, WiFi
	 * companion) both call esp_wifi_start() themselves. */
	(void)esp_wifi_stop();
#endif

#if ZC_WAKE_HAS_BUTTON
	gpio_init_callback(&button_cb, button_pressed, BIT(wake_pins[ARRAY_SIZE(wake_pins) - 1].spec.pin));
	(void)gpio_add_callback(wake_pins[ARRAY_SIZE(wake_pins) - 1].spec.port, &button_cb);
#endif

	pm_notifier_register(&wake_notifier);
	return 0;
}

/* APPLICATION: after the radio and gpio-keys have configured their pins, so
 * the interrupt types saved at the first sleep are theirs. */
SYS_INIT(pm_wake_init, APPLICATION, 0);
