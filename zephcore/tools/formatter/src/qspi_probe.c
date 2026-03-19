/*
 * SPDX-License-Identifier: Apache-2.0
 * Universal QSPI flash probe and erase — bare-metal nRF52840 register access
 *
 * Bypasses the Zephyr QSPI driver entirely. Directly configures the nRF52840
 * QSPI peripheral's PSEL (pin select) registers, sends standard JEDEC SPI NOR
 * commands via the custom instruction interface (CINSTR), and performs a full
 * chip erase when a flash chip responds.
 *
 * To add a new board: append an entry to known_boards[] with its QSPI pin
 * mapping and optional power-enable GPIO. No DTS or Kconfig changes needed.
 */

#include <zephyr/kernel.h>
#include "qspi_probe.h"

#if defined(CONFIG_SOC_NRF52840)

#include <nrfx.h>
#include <hal/nrf_gpio.h>

/* ── SPI NOR flash commands (JEDEC standard — all chips) ──── */

#define CMD_READ_ID      0x9F  /* Read JEDEC ID: manufacturer + type + capacity */
#define CMD_WRITE_EN     0x06  /* Write Enable (required before erase) */
#define CMD_CHIP_ERASE   0xC7  /* Full chip erase */
#define CMD_READ_STATUS  0x05  /* Read Status Register 1 */
#define STATUS_WIP       0x01  /* Write-In-Progress bit in status register */

/* ── CINSTRCONF length values (opcode + N-1 data bytes) ───── */

#define CINSTR_1B   1  /* opcode only */
#define CINSTR_2B   2  /* opcode + 1 byte response */
#define CINSTR_4B   4  /* opcode + 3 bytes response (JEDEC ID) */

/* ── Pin encoding: matches nRF52840 PSEL register format ──── */

#define QPIN(port, pin)    (((port) << 5) | (pin))
#define QPIN_NONE          0xFF
#define QPIN_PORT(p)       (((p) >> 5) & 1)
#define QPIN_PIN(p)        ((p) & 0x1F)

/* PSEL register value: pin connected */
#define PSEL_DISCONNECT    (1UL << 31)

/* ── Known board QSPI pin configurations ─────────────────── */

struct qspi_pin_config {
	const char *name;
	uint8_t sck, csn, io0, io1, io2, io3;
	uint8_t pwr_pin;  /* GPIO to drive HIGH before probe, QPIN_NONE if none */
};

/*
 * Pin table — add new boards here.
 * Sourced from ZephCore DTS + Arduino MeshCore variants.
 *
 * Config                     SCK      CSN      IO0      IO1      IO2      IO3      PWR      Flash chip       Boards
 * ───────────────────────── ──────── ──────── ──────── ──────── ──────── ──────── ──────── ──────────────── ────────────────────────────────────────
 * Wio/XIAO/Ikoka/SenseCap   P0.21    P0.25    P0.20    P0.24    P0.22    P0.23    —        P25Q16H 2MB      Wio Tracker L1, XIAO nRF52, Ikoka Stick/Nano/Handheld, SenseCap Solar
 * ThinkNode M1 / TEcho       P1.14    P1.15    P1.12    P1.13    P0.07    P0.05    —        MX25R1635F 2MB   ThinkNode M1, LilyGo TEcho
 * ThinkNode M6               P1.03    P0.23    P1.01    P1.02    P1.04    P1.05    P0.21    MX25R1635F 2MB   ThinkNode M6
 * RAK4631 / GAT562           P0.03    P0.26    P0.30    P0.29    P0.28    P0.02    —        IS25LP080D 1MB   RAK4631, RAK3401, GAT562 variants
 * LilyGo TEcho Lite          P0.04    P0.12    P0.06    P0.08    P1.09    P0.26    —        ZD25WQ32C 4MB    LilyGo TEcho Lite
 * Nano G2 Ultra              P0.08    P1.07    P0.06    P0.26    P1.04    P1.02    —        W25Q16JV 2MB     Nano G2 Ultra
 */
static const struct qspi_pin_config known_boards[] = {
	{
		.name = "Wio/XIAO/Ikoka/SenseCap",
		.sck = QPIN(0, 21), .csn = QPIN(0, 25),
		.io0 = QPIN(0, 20), .io1 = QPIN(0, 24),
		.io2 = QPIN(0, 22), .io3 = QPIN(0, 23),
		.pwr_pin = QPIN_NONE,
	},
	{
		.name = "ThinkNode M1 / TEcho",
		.sck = QPIN(1, 14), .csn = QPIN(1, 15),
		.io0 = QPIN(1, 12), .io1 = QPIN(1, 13),
		.io2 = QPIN(0,  7), .io3 = QPIN(0,  5),
		.pwr_pin = QPIN_NONE,
	},
	{
		.name = "ThinkNode M6",
		.sck = QPIN(1,  3), .csn = QPIN(0, 23),
		.io0 = QPIN(1,  1), .io1 = QPIN(1,  2),
		.io2 = QPIN(1,  4), .io3 = QPIN(1,  5),
		.pwr_pin = QPIN(0, 21),
	},
	{
		.name = "RAK4631 / GAT562",
		.sck = QPIN(0,  3), .csn = QPIN(0, 26),
		.io0 = QPIN(0, 30), .io1 = QPIN(0, 29),
		.io2 = QPIN(0, 28), .io3 = QPIN(0,  2),
		.pwr_pin = QPIN_NONE,
	},
	{
		.name = "LilyGo TEcho Lite",
		.sck = QPIN(0,  4), .csn = QPIN(0, 12),
		.io0 = QPIN(0,  6), .io1 = QPIN(0,  8),
		.io2 = QPIN(1,  9), .io3 = QPIN(0, 26),
		.pwr_pin = QPIN_NONE,
	},
	{
		.name = "Nano G2 Ultra",
		.sck = QPIN(0,  8), .csn = QPIN(1,  7),
		.io0 = QPIN(0,  6), .io1 = QPIN(0, 26),
		.io2 = QPIN(1,  4), .io3 = QPIN(1,  2),
		.pwr_pin = QPIN_NONE,
	},
};

/* ── GPIO helpers (raw register access) ───────────────────── */

static NRF_GPIO_Type *gpio_port_reg(uint8_t qpin)
{
	return QPIN_PORT(qpin) ? NRF_P1 : NRF_P0;
}

static void gpio_set_output_high(uint8_t qpin)
{
	NRF_GPIO_Type *port = gpio_port_reg(qpin);
	uint8_t pin = QPIN_PIN(qpin);

	nrf_gpio_cfg_output(qpin);
	port->OUTSET = (1UL << pin);
}

static void gpio_release(uint8_t qpin)
{
	NRF_GPIO_Type *port = gpio_port_reg(qpin);
	uint8_t pin = QPIN_PIN(qpin);

	port->OUTCLR = (1UL << pin);
	nrf_gpio_cfg_default(qpin);
}

/* ── QSPI register helpers ────────────────────────────────── */

static void qspi_disconnect_pins(void)
{
	NRF_QSPI->PSEL.SCK = PSEL_DISCONNECT;
	NRF_QSPI->PSEL.CSN = PSEL_DISCONNECT;
	NRF_QSPI->PSEL.IO0 = PSEL_DISCONNECT;
	NRF_QSPI->PSEL.IO1 = PSEL_DISCONNECT;
	NRF_QSPI->PSEL.IO2 = PSEL_DISCONNECT;
	NRF_QSPI->PSEL.IO3 = PSEL_DISCONNECT;
}

static void qspi_set_pins(const struct qspi_pin_config *cfg)
{
	NRF_QSPI->PSEL.SCK = cfg->sck;
	NRF_QSPI->PSEL.CSN = cfg->csn;
	NRF_QSPI->PSEL.IO0 = cfg->io0;
	NRF_QSPI->PSEL.IO1 = cfg->io1;
	NRF_QSPI->PSEL.IO2 = cfg->io2;
	NRF_QSPI->PSEL.IO3 = cfg->io3;
}

static bool qspi_wait_ready(int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (!NRF_QSPI->EVENTS_READY) {
		if (k_uptime_get() > deadline) {
			return false;
		}
		k_msleep(1);
	}
	NRF_QSPI->EVENTS_READY = 0;
	return true;
}

static bool qspi_activate(void)
{
	/* Single-bit SPI mode, 24-bit addressing, no DPM */
	NRF_QSPI->IFCONFIG0 = 0;

	/* 2 MHz (SCKFREQ=15 → 32MHz/16), 8µs CSN delay, SPI mode 0 */
	NRF_QSPI->IFCONFIG1 = (15UL << 28) | 128;

	NRF_QSPI->ENABLE = QSPI_ENABLE_ENABLE_Enabled;
	NRF_QSPI->EVENTS_READY = 0;
	NRF_QSPI->TASKS_ACTIVATE = 1;

	return qspi_wait_ready(500);
}

static void qspi_deactivate(void)
{
	NRF_QSPI->EVENTS_READY = 0;
	NRF_QSPI->TASKS_DEACTIVATE = 1;
	qspi_wait_ready(500);
	NRF_QSPI->ENABLE = QSPI_ENABLE_ENABLE_Disabled;
	qspi_disconnect_pins();
}

/**
 * Execute a custom instruction via CINSTR interface.
 * @param opcode  SPI NOR command opcode
 * @param len     Total bytes (opcode + response), 1-9
 * @param dat0    Output: CINSTRDAT0 register value (response bytes), or NULL
 * @return true on success, false on timeout
 */
static bool qspi_cinstr(uint8_t opcode, uint8_t len, uint32_t *dat0)
{
	NRF_QSPI->CINSTRDAT0 = 0;
	NRF_QSPI->EVENTS_READY = 0;
	NRF_QSPI->CINSTRCONF = ((uint32_t)opcode) |
				((uint32_t)len << 8) |
				(1UL << 12) |  /* LIO2: hold IO2 HIGH (not used in single-bit) */
				(1UL << 13);   /* LIO3: hold IO3 HIGH (not used in single-bit) */

	if (!qspi_wait_ready(500)) {
		return false;
	}

	if (dat0) {
		*dat0 = NRF_QSPI->CINSTRDAT0;
	}
	return true;
}

/* ── Per-config probe and erase ───────────────────────────── */

static int probe_one(const struct qspi_pin_config *cfg)
{
	uint32_t dat0;
	uint8_t mfr, type, cap;

	printk("  Probing %s pins...", cfg->name);

	/* Power-enable GPIO if needed */
	if (cfg->pwr_pin != QPIN_NONE) {
		gpio_set_output_high(cfg->pwr_pin);
		k_msleep(2);
	}

	/* Configure and activate QSPI peripheral */
	NRF_QSPI->ENABLE = QSPI_ENABLE_ENABLE_Disabled;
	qspi_disconnect_pins();
	qspi_set_pins(cfg);

	if (!qspi_activate()) {
		printk(" activate timeout\n");
		goto fail;
	}

	/* READ JEDEC ID (0x9F → 3 bytes: manufacturer, type, capacity) */
	if (!qspi_cinstr(CMD_READ_ID, CINSTR_4B, &dat0)) {
		printk(" read ID timeout\n");
		goto fail_deactivate;
	}

	mfr  = (dat0 >>  0) & 0xFF;
	type = (dat0 >>  8) & 0xFF;
	cap  = (dat0 >> 16) & 0xFF;

	/* Validate — reject bus-float (0xFF) and bus-ground (0x00) */
	if (mfr == 0xFF || mfr == 0x00) {
		printk(" no flash (ID=%02X %02X %02X)\n", mfr, type, cap);
		goto fail_deactivate;
	}

	printk(" found! JEDEC=%02X %02X %02X (%u KB)\n",
	       mfr, type, cap, (unsigned)((1UL << cap) / 1024));

	/* ── Erase ── */
	printk("  Erasing QSPI flash...");

	/* WRITE ENABLE */
	if (!qspi_cinstr(CMD_WRITE_EN, CINSTR_1B, NULL)) {
		printk(" WREN failed\n");
		goto fail_deactivate;
	}

	/* CHIP ERASE */
	if (!qspi_cinstr(CMD_CHIP_ERASE, CINSTR_1B, NULL)) {
		printk(" erase cmd failed\n");
		goto fail_deactivate;
	}

	/* Poll status register until WIP clears (chip erase: up to 60s typical) */
	int elapsed_ms = 0;
	const int poll_interval_ms = 500;
	const int timeout_ms = 120000;

	while (elapsed_ms < timeout_ms) {
		k_msleep(poll_interval_ms);
		elapsed_ms += poll_interval_ms;

		if (!qspi_cinstr(CMD_READ_STATUS, CINSTR_2B, &dat0)) {
			printk(" status poll failed\n");
			goto fail_deactivate;
		}

		if (!(dat0 & STATUS_WIP)) {
			printk(" OK (%d.%ds)\n",
			       elapsed_ms / 1000,
			       (elapsed_ms % 1000) / 100);
			qspi_deactivate();
			if (cfg->pwr_pin != QPIN_NONE) {
				gpio_release(cfg->pwr_pin);
			}
			return 0;
		}
	}

	printk(" TIMEOUT (>120s)\n");

fail_deactivate:
	qspi_deactivate();
fail:
	if (cfg->pwr_pin != QPIN_NONE) {
		gpio_release(cfg->pwr_pin);
	}
	return -1;
}

/* ── Public API ───────────────────────────────────────────── */

int qspi_probe_and_erase(void)
{
	printk("  QSPI: probing %d known pin configurations...\n",
	       (int)ARRAY_SIZE(known_boards));

	for (int i = 0; i < (int)ARRAY_SIZE(known_boards); i++) {
		if (probe_one(&known_boards[i]) == 0) {
			return 0;
		}
	}

	printk("  QSPI: no external flash found — skipped\n");
	return -1;
}

#else /* !CONFIG_SOC_NRF52840 */

int qspi_probe_and_erase(void)
{
	printk("  QSPI: not supported on this platform\n");
	return -1;
}

#endif /* CONFIG_SOC_NRF52840 */
