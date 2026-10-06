/*
 * SPDX-License-Identifier: MIT
 * power_diag - what wakes the CPU, how long it sleeps, and which clocks and
 * peripherals stay enabled through that sleep (CONFIG_ZEPHCORE_POWER_DIAG,
 * `get pwr <n>`).  nRF52840 only.  TEMPORARY bench instrumentation.
 *
 * Counted from Zephyr's user tracing hooks: the idle hook stamps each entry to
 * WFI, and the first interrupt after it closes the interval and is recorded as
 * the wake source.  Every interrupt is counted by IRQ number as well.
 *
 * Two windows are kept: since boot (or the last `get pwr -1`), and "battery",
 * the last stretch with VBUS absent -- opened when USB power goes away and
 * frozen when it returns, so a node can be run unplugged and read afterwards.
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>

#if IS_ENABLED(CONFIG_ZEPHCORE_POWER_DIAG) && defined(CONFIG_SOC_NRF52840)

#include <cmsis_core.h>

#define NIRQ 48

#define REG(addr) (*(volatile uint32_t *)(addr))

#define POWER_BASE 0x40000000UL
#define USBREGSTATUS REG(POWER_BASE + 0x438)
#define HFCLKSTAT    REG(POWER_BASE + 0x40C)

/* Peripherals with an ENABLE register at +0x500; bit n of the masks below. */
static const struct {
	const char *name;
	uint32_t base;
} periph[] = {
	{ "uarte0", 0x40002000UL }, { "spi0/twi0", 0x40003000UL },
	{ "spi1/twi1", 0x40004000UL }, { "saadc", 0x40007000UL },
	{ "qdec", 0x40012000UL },
	{ "comp", 0x40013000UL }, { "pwm0", 0x4001C000UL },
	{ "pdm", 0x4001D000UL }, { "pwm1", 0x40021000UL },
	{ "pwm2", 0x40022000UL }, { "spi2", 0x40023000UL },
	{ "i2s", 0x40025000UL }, { "usbd", 0x40027000UL },
	{ "uarte1", 0x40028000UL }, { "qspi", 0x40029000UL },
	{ "pwm3", 0x4002D000UL }, { "spi3", 0x4002F000UL },
};

struct pd_stats {
	uint32_t start_cyc;      /* k_cycle_get_32() when the window opened */
	uint32_t end_cyc;        /* when it was frozen (battery window only) */
	uint64_t sleep_cyc;      /* cycles spent in WFI */
	uint64_t sleep_hfxo_cyc; /* ... of which with the HF crystal running */
	uint32_t wakes;          /* WFI exits */
	uint32_t isrs;           /* interrupt entries */
	uint32_t en_or;          /* peripherals seen enabled at an idle entry */
	uint32_t en_and;         /* peripherals enabled at every idle entry */
	uint32_t wake_irq[NIRQ];
	uint32_t isr_irq[NIRQ];
};

static struct pd_stats boot_w;  /* since boot / last reset */
static struct pd_stats batt_w;  /* last VBUS-absent stretch */
static bool batt_open;
static bool batt_valid;
static bool vbus_last = true;
static bool started;

static volatile bool in_idle;
static uint32_t idle_enter_cyc;
static bool idle_hfxo;

static void window_open(struct pd_stats *w, uint32_t now)
{
	memset(w, 0, sizeof(*w));
	w->start_cyc = now;
	w->en_and = 0xFFFFFFFFu;
}

static uint32_t periph_mask(void)
{
	uint32_t m = 0;

	for (unsigned int i = 0; i < ARRAY_SIZE(periph); i++) {
		if (REG(periph[i].base + 0x500) != 0) {
			m |= BIT(i);
		}
	}
	return m;
}

void sys_trace_idle_user(void)
{
	uint32_t m = periph_mask();

	if (!started) {
		window_open(&boot_w, k_cycle_get_32());
		started = true;
	}
	boot_w.en_or |= m;
	boot_w.en_and &= m;
	if (batt_open) {
		batt_w.en_or |= m;
		batt_w.en_and &= m;
	}
	/* SRC = Xtal (bit 0) and STATE = running (bit 16) */
	idle_hfxo = (HFCLKSTAT & 0x10001u) == 0x10001u;
	idle_enter_cyc = k_cycle_get_32();
	in_idle = true;
}

static void account(struct pd_stats *w, uint32_t irq, bool woke, uint32_t slept)
{
	w->isrs++;
	if (irq < NIRQ) {
		w->isr_irq[irq]++;
	}
	if (woke) {
		w->wakes++;
		w->sleep_cyc += slept;
		if (idle_hfxo) {
			w->sleep_hfxo_cyc += slept;
		}
		if (irq < NIRQ) {
			w->wake_irq[irq]++;
		}
	}
}

void sys_trace_isr_enter_user(void)
{
	uint32_t now = k_cycle_get_32();
	uint32_t irq = __get_IPSR() - 16u;
	bool woke = in_idle;
	uint32_t slept = woke ? now - idle_enter_cyc : 0;
	bool vbus = (USBREGSTATUS & 1u) != 0;

	in_idle = false;

	if (!started) {
		window_open(&boot_w, now);
		started = true;
	}
	if (vbus != vbus_last) {
		if (!vbus) {
			window_open(&batt_w, now);
			batt_open = true;
			batt_valid = false;
		} else if (batt_open) {
			batt_w.end_cyc = now;
			batt_open = false;
			batt_valid = true;
		}
		vbus_last = vbus;
	}

	account(&boot_w, irq, woke, slept);
	if (batt_open) {
		account(&batt_w, irq, woke, slept);
	}
}

static int fmt_irqs(char *out, size_t cap, const char *tag, const uint32_t *tab)
{
	int n = snprintf(out, cap, "> %s", tag);

	for (int i = 0; i < NIRQ && n > 0 && (size_t)n < cap; i++) {
		if (tab[i]) {
			n += snprintf(out + n, cap - n, " %d:%u", i, (unsigned)tab[i]);
		}
	}
	return n;
}

static void fmt_summary(char *out, size_t cap, const char *tag, const struct pd_stats *w,
			uint32_t end)
{
	uint32_t hz = sys_clock_hw_cycles_per_sec();
	uint64_t span = (uint32_t)(end - w->start_cyc);
	uint32_t span_ms = (uint32_t)(span * 1000u / hz);
	uint32_t sleep_ms = (uint32_t)(w->sleep_cyc * 1000u / hz);
	uint32_t hfxo_ms = (uint32_t)(w->sleep_hfxo_cyc * 1000u / hz);
	uint32_t awake_ppm = span ? (uint32_t)((span - MIN(span, w->sleep_cyc)) * 1000000u / span)
				  : 0;

	snprintf(out, cap,
		 "> %s span=%ums sleep=%ums awake_ppm=%u wakes=%u isrs=%u sleep_hfxo=%ums "
		 "en_or=%05x en_and=%05x",
		 tag, (unsigned)span_ms, (unsigned)sleep_ms, (unsigned)awake_ppm,
		 (unsigned)w->wakes, (unsigned)w->isrs, (unsigned)hfxo_ms, (unsigned)w->en_or,
		 (unsigned)(w->en_and & (BIT(ARRAY_SIZE(periph)) - 1u)));
}

struct thr_walk {
	int want;
	int idx;
	char *out;
	size_t cap;
};

static void thr_cb(const struct k_thread *t, void *u)
{
	struct thr_walk *w = u;
	k_thread_runtime_stats_t st;
	const char *name;

	if (w->idx++ != w->want) {
		return;
	}
	name = k_thread_name_get((k_tid_t)t);
	if (k_thread_runtime_stats_get((k_tid_t)t, &st) != 0) {
		st.execution_cycles = 0;
	}
	snprintf(w->out, w->cap, "> t %s prio=%d run=%ums", (name && name[0]) ? name : "?",
		 t->base.prio,
		 (unsigned)(st.execution_cycles * 1000u / sys_clock_hw_cycles_per_sec()));
}

/* One single-lane SPI transaction on the QSPI pads (mode 0, ~250 kHz). */
static void pd_spi(uint8_t cmd, uint8_t *rx, size_t rx_len)
{
	REG(0x5000050CUL) = BIT(25);  /* CS# low */
	k_busy_wait(2);
	for (size_t i = 0; i < 1 + rx_len; i++) {
		uint8_t o = (i == 0) ? cmd : 0, in = 0;

		for (int b = 7; b >= 0; b--) {
			if ((o >> b) & 1u) {
				REG(0x50000508UL) = BIT(20);
			} else {
				REG(0x5000050CUL) = BIT(20);
			}
			k_busy_wait(2);
			REG(0x50000508UL) = BIT(21);
			in = (in << 1) | ((REG(0x50000510UL) >> 24) & 1u);
			k_busy_wait(2);
			REG(0x5000050CUL) = BIT(21);
		}
		if (i > 0) {
			rx[i - 1] = in;
		}
	}
	k_busy_wait(2);
	REG(0x50000508UL) = BIT(25);  /* CS# high */
}

/* One line per call:
 *  -1 reset the boot window      0 boot summary     1 boot wake IRQs
 *   2 boot all IRQs              3 battery summary  4 battery wake IRQs
 *   5 battery all IRQs           6 power/clock regs 7 peripheral enables now
 *   8 GPIO                       10.. threads (run time), then "end" */
void power_diag_line(int n, char *out, size_t cap)
{
	uint32_t now = k_cycle_get_32();

	if (n < 0) {
		unsigned int key = irq_lock();

		window_open(&boot_w, now);
		irq_unlock(key);
		snprintf(out, cap, "> reset");
	} else if (n == 0) {
		fmt_summary(out, cap, "boot", &boot_w, now);
	} else if (n == 1) {
		fmt_irqs(out, cap, "wake", boot_w.wake_irq);
	} else if (n == 2) {
		fmt_irqs(out, cap, "isr", boot_w.isr_irq);
	} else if (n == 3) {
		if (batt_open) {
			fmt_summary(out, cap, "batt(open)", &batt_w, now);
		} else if (batt_valid) {
			fmt_summary(out, cap, "batt", &batt_w, batt_w.end_cyc);
		} else {
			snprintf(out, cap, "> batt none");
		}
	} else if (n == 4) {
		fmt_irqs(out, cap, "bwake", batt_w.wake_irq);
	} else if (n == 5) {
		fmt_irqs(out, cap, "bisr", batt_w.isr_irq);
	} else if (n == 6) {
		snprintf(out, cap,
			 "> dcdcen=%u dcdcen0=%u mainreg=%u usbreg=%x hfclkstat=%05x "
			 "lfclkstat=%05x lfclksrccopy=%x radio_power=%u radio_state=%u",
			 (unsigned)REG(POWER_BASE + 0x578), (unsigned)REG(POWER_BASE + 0x580),
			 (unsigned)REG(POWER_BASE + 0x640), (unsigned)USBREGSTATUS,
			 (unsigned)HFCLKSTAT, (unsigned)REG(POWER_BASE + 0x418),
			 (unsigned)REG(POWER_BASE + 0x41C), (unsigned)REG(0x40001FFCUL),
			 (unsigned)REG(0x40001550UL));
	} else if (n == 7) {
		int k = snprintf(out, cap, "> en");

		for (unsigned int i = 0; i < ARRAY_SIZE(periph) && (size_t)k < cap; i++) {
			uint32_t v = REG(periph[i].base + 0x500);

			if (v) {
				k += snprintf(out + k, cap - k, " %s=%u", periph[i].name,
					      (unsigned)v);
			}
		}
	} else if (n == 8) {
		snprintf(out, cap,
			 "> p0 dir=%08x out=%08x in=%08x p1 dir=%08x out=%08x in=%08x",
			 (unsigned)REG(0x50000514UL), (unsigned)REG(0x50000504UL),
			 (unsigned)REG(0x50000510UL), (unsigned)REG(0x50000814UL),
			 (unsigned)REG(0x50000804UL), (unsigned)REG(0x50000810UL));
	} else if (n >= 100 && n < 148) {
		/* Pin peek: PIN_CNF as it stands, then the level the pad sits at,
		 * sampled 8 times 1 ms apart with the input buffer connected for
		 * the read only.  A floating pad reads mixed or follows a finger. */
		volatile uint32_t *cnf = (n < 132) ? &REG(0x50000700UL + 4u * (n - 100))
						   : &REG(0x50000A00UL + 4u * (n - 132));
		uint32_t in_reg = (n < 132) ? 0x50000510UL : 0x50000810UL;
		uint32_t bit = (n < 132) ? (n - 100) : (n - 132);
		uint32_t saved = *cnf;
		unsigned int lv = 0;

		*cnf = saved & ~BIT(1);  /* INPUT = connect */
		for (int i = 0; i < 8; i++) {
			k_busy_wait(1000);
			lv = (lv << 1) | ((REG(in_reg) >> bit) & 1u);
		}
		*cnf = saved;
		snprintf(out, cap, "> P%d.%02d cnf=%08x dir=%s inbuf=%s pull=%u samples=%02x",
			 n < 132 ? 0 : 1, (int)bit, (unsigned)saved, (saved & 1u) ? "out" : "in",
			 (saved & 2u) ? "off" : "on", (unsigned)((saved >> 2) & 3u), lv);
	} else if (n == 200 || n == 201) {
		/* Is the /ext flash in deep power-down?  Bit-bang RDID (0x9F) on
		 * the QSPI pads as single-lane SPI: a part in DPD ignores it and
		 * IO1, pulled up here, reads ff ff ff.  201 sends Release (0xAB)
		 * first, expects the JEDEC id, then puts the part back (0xB9).
		 * Pads: SCK P0.21, CSN P0.25, IO0 P0.20, IO1 P0.24, IO2/IO3
		 * P0.22/P0.23 held high (Wio L1, XIAO nRF52840, SenseCAP Solar). */
		static const uint8_t pins[] = { 21, 25, 20, 24, 22, 23 };
		uint32_t saved[ARRAY_SIZE(pins)];
		uint32_t saved_out = REG(0x50000504UL);
		uint8_t id[3];

		for (unsigned int i = 0; i < ARRAY_SIZE(pins); i++) {
			saved[i] = REG(0x50000700UL + 4u * pins[i]);
		}
		REG(0x50000508UL) = BIT(25) | BIT(22) | BIT(23);  /* OUTSET */
		REG(0x5000050CUL) = BIT(21) | BIT(20);            /* OUTCLR */
		REG(0x50000700UL + 4u * 25) = 1u;                 /* out */
		REG(0x50000700UL + 4u * 21) = 1u;
		REG(0x50000700UL + 4u * 20) = 1u;
		REG(0x50000700UL + 4u * 22) = 1u;
		REG(0x50000700UL + 4u * 23) = 1u;
		REG(0x50000700UL + 4u * 24) = (3u << 2);          /* in, pull-up */
		k_busy_wait(100);
		if (n == 201) {
			pd_spi(0xAB, NULL, 0);
			k_busy_wait(200);
		}
		pd_spi(0x9F, id, sizeof(id));
		if (n == 201) {
			pd_spi(0xB9, NULL, 0);
			k_busy_wait(50);
		}
		for (unsigned int i = 0; i < ARRAY_SIZE(pins); i++) {
			REG(0x50000700UL + 4u * pins[i]) = saved[i];
		}
		REG(0x50000504UL) = saved_out;
		snprintf(out, cap, "> rdid%s %02x %02x %02x", n == 201 ? " after release" : "",
			 id[0], id[1], id[2]);
	} else {
		struct thr_walk w = { n - 10, 0, out, cap };

		out[0] = '\0';
		k_thread_foreach_unlocked(thr_cb, &w);
		if (out[0] == '\0') {
			snprintf(out, cap, "end");
		}
	}
}

#else

void power_diag_line(int n, char *out, size_t cap)
{
	ARG_UNUSED(n);
	snprintf(out, cap, "> unsupported");
}

#endif
