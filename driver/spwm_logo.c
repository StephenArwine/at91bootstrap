// Copyright (C) 2026 ZonedIn
//
// SPDX-License-Identifier: MIT

/*
 * Boot logo on the SM16169S HUB75 panel (64x32, 1/8 scan).
 *
 * A port of libspwm (firmware/TEST_SAMA7_PWM_LED/libspwm). The panel is
 * configured and the logo loaded by bit-banging DCLK / LE / 6 x SDI on the
 * PIO. The scan (GCLK bursts and the row address A/B/C) is then handed to
 * PWM channels 0-3, fed by XDMAC1 channel 31 from a descriptor loop in
 * internal SRAM. That engine needs no CPU, so the logo stays lit through the
 * kernel boot until libspwm's spwm_open() adopts the engine.
 *
 * Linux must leave XDMAC1 channel 31 alone and keep the PWM clock on: see
 * the at_xdmac "microchip,reserved-channels" patch and the &pwm node in the
 * board device tree.
 *
 * The SRAM layout (scan table at 0x110000, the two descriptors after it)
 * is exactly libspwm's engine.c, so spwm_open() can end the loop cleanly.
 * Runs with the MMU and caches off, after hw_postinit(): MCK1 is already
 * 200 MHz and is never changed again.
 */

#include "common.h"
#include "hardware.h"
#include "board.h"
#include "pmc.h"
#include "timer.h"
#include "debug.h"
#include "spwm_logo.h"

/* ---------------------------------------------------------------- pins */

enum sig { R1, G1, B1, R2, G2, B2, OE, CLK, LAT, A, B, C, NSIG };

struct pin {
	unsigned char bank, line, func;   /* func: PIO_CFGR FUNC for PWMHx, 0 = GPIO */
};

#if defined(CONFIG_BOARD_QUIRK_SAMA7G5_EK)
/* SAMA7G5-EK 40-pin header, notes/direct-wiring.md (libspwm src/pins.c). */
static const struct pin pins[NSIG] = {
	[R1]  = { 3, 30, 0 }, [G1] = { 3, 28, 0 }, [B1] = { 3, 21, 0 },
	[R2]  = { 1, 22, 0 }, [G2] = { 1, 26, 0 }, [B2] = { 3, 25, 0 },
	[OE]  = { 4,  3, 1 },                       /* PE3  PWMH0 = GCLK */
	[CLK] = { 3, 31, 0 }, [LAT] = { 3, 26, 0 },
	[A]   = { 4,  5, 1 }, [B] = { 4, 7, 1 },    /* PE5 PWMH1, PE7 PWMH2 */
	[C]   = { 3, 29, 6 },                       /* PD29 PWMH3 */
};
#define SCREEN_ON_BANK  (-1)                 /* the EK panel has its own supply */
#else
/*
 * ZI AIO. OE/A/B/C are the proven PWM pins (hardware/ZI_SAMA7_AIO/README.md
 * section 2a). CLK, LAT, the colour lines and SCREEN_ON are PROVISIONAL:
 * bank C as the README recommends, to be replaced by the pins recorded in
 * README section 2b once the schematic is final.
 */
static const struct pin pins[NSIG] = {
	[R1]  = { 2,  6, 0 }, [G1] = { 2,  7, 0 }, [B1] = { 2,  8, 0 },
	[R2]  = { 2,  9, 0 }, [G2] = { 2, 10, 0 }, [B2] = { 2, 11, 0 },
	[OE]  = { 4,  3, 1 },                       /* PE3  PWMH0 = GCLK */
	[CLK] = { 2, 12, 0 }, [LAT] = { 2, 13, 0 },
	[A]   = { 4,  5, 1 }, [B] = { 4, 7, 1 },    /* PE5 PWMH1, PE7 PWMH2 */
	[C]   = { 3, 29, 6 },                       /* PD29 PWMH3 */
};
#define SCREEN_ON_BANK  2                    /* 2N7002 gate: high = panel rail on */
#define SCREEN_ON_LINE  14
#define SCREEN_ON_SETTLE_MS 20
#endif

/* ------------------------------------------------------ panel constants */

#define PANEL_W       64
#define PANEL_H       32
#define SCAN_ROWS     8
#define NUM_ICS       8                      /* per colour chain */
#define GCLK_PER_ROW  129
#define WORDS_PER_CH  (16 * NUM_ICS)         /* DCLKs per channel load */

#define CMD_VSYNC     3
#define CMD_REG1      4
#define CMD_REG2      8
#define CMD_PREACT    14
#define REG1_VAL      0xE739                 /* bits 12:8 = scan lines - 1 */
#define REG2_VAL      0xC005
#define DCLK_SLOWDOWN CONFIG_SPWM_LOGO_DCLK_SLOWDOWN   /* extra repeats of each edge write */

/* Engine, the libspwm defaults: 4 MHz GCLK, update period 3, 3 idle periods. */
#define MCK1_HZ   200000000u
#define GCLK_HZ   4000000u
#define UPR       2
#define GAP       3

/* 16-bit linear grayscale, dim light blue 3:5:7 like the STM32 logo. */
#ifndef SPWM_LOGO_R
#define SPWM_LOGO_R 0x3333
#define SPWM_LOGO_G 0x5555
#define SPWM_LOGO_B 0x7777
#endif

/* The ZI logo, 26 x 32, as the STM32 draws it (matrix.c set_ZI_logo),
 * bit 25 = leftmost column, placed at x = 19. */
#define LOGO_W 26
#define LOGO_X 19
static const unsigned int logo[PANEL_H] = {
	0x3FFFFFF, 0x3FFFFFF, 0x3000003, 0x3000003, 0x3000003, 0x3000003,
	0x30FFFC3, 0x31FFF83, 0x31FFE03, 0x31FFC03, 0x33FF00F, 0x37FE03C,
	0x3FF8079, 0x3BE01E7, 0x33C03FF, 0x0F00FFF, 0x1E03FFB, 0x3807FE3,
	0x301FFC3, 0x303FFC3, 0x30FFFC3, 0x30FFFC3, 0x303FF03, 0x301FC03,
	0x3807807, 0x1E0001E, 0x0F00078, 0x03C00F0, 0x01E03C0, 0x0078780,
	0x001FE00, 0x000F800,
};

/* -------------------------------------------------------------- registers */

#define PIO_BASE    AT91C_BASE_PIOA          /* one PIO, banks 0x40 apart */
#define PIO_MSKR    0x00
#define PIO_CFGR    0x04
#define PIO_SODR    0x10
#define PIO_CODR    0x14
#define PIO_ODSR    0x18
#define CFGR_DIR_OUT   (1u << 8)
#define CFGR_DRVSTR_HI (3u << 16)

#ifndef AT91C_ID_PWM
#define AT91C_ID_PWM   77
#endif
#define PWM_BASE    0xE1604000u
#define PWM_ENA     0x04
#define PWM_DIS     0x08
#define PWM_SR      0x0C
#define PWM_SCM     0x20
#define PWM_DMAR    0x24
#define PWM_SCUP    0x2C
#define PWM_ISR2    0x40
#define PWM_CMR(c)  (0x200 + (c) * 0x20)
#define PWM_CDTY(c) (0x204 + (c) * 0x20)
#define PWM_CPRD0   0x20C
#define CMR_CPOL    (1u << 9)
#define CMR_DPOLI   (1u << 12)
#define NCH         4
#define SYNC_ALL    ((1u << NCH) - 1)
#define SCM_UPDM_DMA (2u << 16)

#define XDMAC_BASE  AT91C_BASE_XDMAC1
#define XDMAC_GCFG  0x04
#define XDMAC_GWAC  0x08
#define XDMAC_GE    0x1C
#define XDMAC_GD    0x20
#define XDMAC_GS    0x24
#define CHAN        31
#define CH(reg)     (0x60u + CHAN * 0x40u + (reg))
#define C_CID       0x04
#define C_CIS       0x0C
#define C_CNDA      0x18
#define C_CNDC      0x1C
#define C_CBC       0x24
#define C_CC        0x28
#define C_CDS_MSP   0x2C
#define C_CSUS      0x30
#define C_CDUS      0x34
#define PWM_TX_PERID 39
/* What Linux's at_xdmac writes for a peripheral DMA controller; set it now
 * so the probe changes nothing under the running channel. */
#define GCFG_P2M    0x10030050u
#define GWAC_P2M    0x00000F0Fu

#define CC_TYPE_PER      (1u << 0)
#define CC_DSYNC_MEM2PER (1u << 4)
#define CC_DWIDTH_WORD   (2u << 11)
#define CC_SAM_INCR      (1u << 16)
#define CC_PERID(i)      ((unsigned int)((i) & 0x7F) << 24)
#define UBC_NDE   (1u << 24)
#define UBC_NSEN  (1u << 25)
#define UBC_NDEN  (1u << 26)
#define UBC_NDV2  (2u << 27)
#define CNDC_NDE  (1u << 0)
#define CNDC_NDSUP (1u << 1)
#define CNDC_NDDUP (1u << 2)
#define CNDC_NDV2 (2u << 3)

#define SRAM_BASE 0x00100000u
#define SCAN_OFF  0x10000u                   /* libspwm SRAM_USE */
#define SRAM_END  0x18000u

/* ------------------------------------------------------------- PIO state */

static unsigned int ck_bank, ot_bank, gb_bank;      /* CLK+LAT bank, other data bank, GCLK bank */
static unsigned int ck_data, ot_data;               /* data pin bits per bank */
static unsigned int clk_bit, lat_bit, oe_bit;
static unsigned int abc_mask[5];
static unsigned int lut_ck[64], lut_ot[64];         /* 6-bit SDI word -> pin bits */
static unsigned int cur_row;

static inline unsigned int bit(enum sig s) { return 1u << pins[s].line; }

static inline void pio_w(unsigned int bank, unsigned int reg, unsigned int v)
{
	writel(v, PIO_BASE + bank * 0x40 + reg);
}

/* Write, then repeat it to stretch the edge (libspwm dclk_slowdown). */
static inline void pio_ws(unsigned int bank, unsigned int reg, unsigned int v)
{
	unsigned int i;
	for (i = 0; i <= DCLK_SLOWDOWN; i++)
		pio_w(bank, reg, v);
}

static void setup_masks(void)
{
	static const enum sig data[6] = { R1, G1, B1, R2, G2, B2 };
	unsigned int v, k;

	ck_bank = pins[CLK].bank;
	gb_bank = pins[OE].bank;
	clk_bit = bit(CLK);
	lat_bit = bit(LAT);
	oe_bit = bit(OE);
	ot_bank = ck_bank;
	for (k = 0; k < 6; k++)
		if (pins[data[k]].bank != ck_bank)
			ot_bank = pins[data[k]].bank;   /* data lives on <= 2 banks */
	ck_data = ot_data = 0;
	for (k = 0; k < 6; k++) {
		if (pins[data[k]].bank == ck_bank)
			ck_data |= bit(data[k]);
		else
			ot_data |= bit(data[k]);
	}
	for (v = 0; v < 64; v++) {
		lut_ck[v] = lut_ot[v] = 0;
		for (k = 0; k < 6; k++)
			if (v >> k & 1) {
				if (pins[data[k]].bank == ck_bank)
					lut_ck[v] |= bit(data[k]);
				else
					lut_ot[v] |= bit(data[k]);
			}
	}
	for (k = 0; k < 5; k++)
		abc_mask[k] = 0;
	abc_mask[pins[A].bank] |= bit(A);
	abc_mask[pins[B].bank] |= bit(B);
	abc_mask[pins[C].bank] |= bit(C);
}

static void set_row(unsigned int row)
{
	unsigned int v = ((row & 1) ? bit(A) : 0) | ((row & 2) ? bit(B) : 0) |
			 ((row & 4) ? bit(C) : 0);
	unsigned int b;

	cur_row = row;
	for (b = 0; b < 5; b++)
		if (abc_mask[b]) {
			pio_w(b, PIO_MSKR, abc_mask[b]);
			pio_w(b, PIO_ODSR, v & abc_mask[b]);
		}
}

/* All panel pins as GPIO outputs: GCLK high (idle), everything else low. */
static void pins_gpio(void)
{
	unsigned int mask[5] = { 0 };
	unsigned int s, b;

	for (s = 0; s < NSIG; s++)
		mask[pins[s].bank] |= bit(s);
	for (b = 0; b < 5; b++) {
		if (!mask[b])
			continue;
		pio_w(b, PIO_CODR, mask[b]);
		if (b == gb_bank)
			pio_w(b, PIO_SODR, oe_bit);
		pio_w(b, PIO_MSKR, mask[b]);
		pio_w(b, PIO_CFGR, CFGR_DIR_OUT | CFGR_DRVSTR_HI);
	}
	set_row(0);
}

/* OE/A/B/C: hand them to the PWM (func) or take them back as GPIO. */
static void pins_pwm(int on)
{
	static const enum sig sigs[4] = { OE, A, B, C };
	unsigned int i;

	for (i = 0; i < 4; i++) {
		unsigned int b = pins[sigs[i]].bank, m = bit(sigs[i]);
		if (!on)
			pio_w(b, PIO_CODR, m);
		pio_w(b, PIO_MSKR, m);
		pio_w(b, PIO_CFGR, (on ? pins[sigs[i]].func : CFGR_DIR_OUT) | CFGR_DRVSTR_HI);
	}
	if (!on)
		set_row(0);
}

/* ------------------------------------------------------ SM16169 protocol */

/* n DCLK rising edges with LE high and the data lines low: a command. */
static void send_latches(unsigned int n)
{
	unsigned int i;

	if (ot_data) {
		pio_w(ot_bank, PIO_MSKR, ot_data);
		pio_w(ot_bank, PIO_ODSR, 0);
	}
	pio_w(ck_bank, PIO_MSKR, ck_data | clk_bit);
	pio_w(ck_bank, PIO_ODSR, 0);
	pio_ws(ck_bank, PIO_SODR, lat_bit);
	for (i = 0; i < n; i++) {
		pio_ws(ck_bank, PIO_SODR, clk_bit);
		pio_ws(ck_bank, PIO_CODR, clk_bit);
	}
	pio_ws(ck_bank, PIO_CODR, lat_bit);
}

/* One DCLK: present the 6-bit word, raise LE if asked (it stays high until
 * sdi_end), rising edge. */
static void sdi_word(unsigned int v, int le)
{
	if (ot_data) {
		pio_w(ot_bank, PIO_MSKR, ot_data);
		pio_w(ot_bank, PIO_ODSR, lut_ot[v]);
	}
	pio_w(ck_bank, PIO_MSKR, ck_data | clk_bit);
	pio_ws(ck_bank, PIO_ODSR, lut_ck[v]);          /* data, CLK low */
	if (le)
		pio_w(ck_bank, PIO_SODR, lat_bit);
	pio_ws(ck_bank, PIO_SODR, clk_bit);            /* rising edge */
}

static void sdi_end(void)
{
	pio_w(ck_bank, PIO_CODR, clk_bit);
	pio_w(ck_bank, PIO_CODR, lat_bit);
}

/* The same 16-bit value per colour into every IC, MSB first; LE high during
 * the last `cmd` DCLKs (the write command). */
static void send_cfg_reg(unsigned int cmd, unsigned int val)
{
	const unsigned int n = 16 * NUM_ICS;
	unsigned int i;

	for (i = 0; i < n; i++)
		sdi_word((val >> (15 - (i & 15)) & 1) ? 0x3F : 0, i >= n - cmd);
	sdi_end();
}

static void load_config(void)
{
	send_latches(CMD_PREACT);
	send_cfg_reg(CMD_REG1, (REG1_VAL & ~(0x1Fu << 8)) | ((SCAN_ROWS - 1u) << 8));
	send_latches(CMD_PREACT);
	send_cfg_reg(CMD_REG2, REG2_VAL);
}

static void row_burst(void)
{
	unsigned int i;

	for (i = 0; i < GCLK_PER_ROW; i++) {
		pio_w(gb_bank, PIO_CODR, oe_bit);
		pio_w(gb_bank, PIO_SODR, oe_bit);     /* rising edge = one GCLK */
	}
	set_row((cur_row + 1) % SCAN_ROWS);
}

static inline unsigned int logo_on(unsigned int x, unsigned int y)
{
	if (x < LOGO_X || x >= LOGO_X + LOGO_W)
		return 0;
	return logo[y] >> (LOGO_W - 1 - (x - LOGO_X)) & 1;
}

/*
 * Load a frame (the logo, or black): 8 scans x 16 channels, each channel
 * 8 ICs x 16 bits, MSB first, farthest IC first, LE on the channel's last
 * DCLK. Panel wiring from libspwm sm_pack(): ICs 0-3 are rows (scan,
 * scan+16), ICs 4-7 rows (scan+8, scan+24); x = (3-ic)*16 + (15-ch) or
 * (7-ic)*16 + (15-ch). bursts > 0 runs that many CPU GCLK row bursts after
 * each channel (the libspwm bring-up path, used for the black frame only);
 * with 0 the hardware engine is expected to be scanning, which is how
 * libspwm loads every real frame.
 */
static void load_frame(int show_logo, unsigned int bursts)
{
	unsigned int cbits[16];
	unsigned int scan, b, i;
	int ch, ic;

	for (b = 0; b < 16; b++)
		cbits[b] = (SPWM_LOGO_R >> b & 1) | (SPWM_LOGO_G >> b & 1) << 1 |
			   (SPWM_LOGO_B >> b & 1) << 2;

	for (scan = 0; scan < SCAN_ROWS; scan++)
		for (ch = 15; ch >= 0; ch--) {
			i = 0;
			for (ic = NUM_ICS - 1; ic >= 0; ic--) {
				unsigned int x, ru, rl, up, lo;
				if (ic < 4) {
					x = (3 - ic) * 16 + (15 - ch);
					ru = scan;
					rl = scan + 16;
				} else {
					x = (7 - ic) * 16 + (15 - ch);
					ru = scan + 8;
					rl = scan + 24;
				}
				up = show_logo && logo_on(x, ru);
				lo = show_logo && logo_on(x, rl);
				for (b = 16; b-- > 0; i++)
					sdi_word((up ? cbits[b] : 0) | (lo ? cbits[b] << 3 : 0),
						 i == WORDS_PER_CH - 1);
			}
			sdi_end();
			for (b = 0; b < bursts; b++)
				row_burst();
		}
}

/* Finish the scan at row 0, then VSYNC: the loaded frame goes live. */
static void present(void)
{
	while (cur_row != 0)
		row_burst();
	send_latches(CMD_VSYNC);
}

/* ------------------------------------------------------------- engine */

static unsigned int period, burst_elems, gap_elems, elems_per_scan;
static unsigned int scan_off, tail_off, d_scan, d_tail, cc_cfg;

static inline void sram_w(unsigned int off, unsigned int v)
{
	writel(v, SRAM_BASE + off);
}

static void abc_duties(unsigned int row, unsigned int d[3])
{
	unsigned int i;
	for (i = 0; i < 3; i++)
		d[i] = (row >> i & 1) ? period : 0;
}

static unsigned int put_elems(unsigned int o, unsigned int n, unsigned int gclk,
			      const unsigned int abc[3])
{
	unsigned int k;
	for (k = 0; k < n; k++) {
		sram_w(o, gclk);
		sram_w(o + 4, abc[0]);
		sram_w(o + 8, abc[1]);
		sram_w(o + 12, abc[2]);
		o += 16;
	}
	return o;
}

static void put_desc(unsigned int off, unsigned int nda, unsigned int ubc, unsigned int sa)
{
	sram_w(off, nda);
	sram_w(off + 4, ubc);
	sram_w(off + 8, sa);
	sram_w(off + 12, PWM_BASE + PWM_DMAR);
	sram_w(off + 16, cc_cfg);
}

/* Scan table in SRAM: per row, GAP periods of (GCLK low, abc) then 129 of
 * (GCLK 50 %, abc), then a tail of GAP at row 0 for a clean stop. The scan
 * descriptor loops on itself; the tail descriptor ends the list. */
static int engine_build(void)
{
	const unsigned int next = UBC_NSEN | UBC_NDEN | UBC_NDV2;
	unsigned int o, r, abc[3];

	period = MCK1_HZ / GCLK_HZ;
	burst_elems = GCLK_PER_ROW / (UPR + 1);
	gap_elems = GAP / (UPR + 1);
	elems_per_scan = SCAN_ROWS * (burst_elems + gap_elems);
	cc_cfg = CC_TYPE_PER | CC_DSYNC_MEM2PER | CC_DWIDTH_WORD | CC_SAM_INCR |
		 CC_PERID(PWM_TX_PERID);

	o = SCAN_OFF;
	scan_off = o;
	for (r = 0; r < SCAN_ROWS; r++) {
		abc_duties(r, abc);
		o = put_elems(o, gap_elems, 0, abc);
		o = put_elems(o, burst_elems, period / 2, abc);
	}
	tail_off = o;
	abc_duties(0, abc);
	o = put_elems(o, gap_elems, 0, abc);
	o = (o + 31) & ~31u;
	d_scan = o;
	d_tail = o + 32;
	if (d_tail + 32 > SRAM_END)
		return -1;
	put_desc(d_scan, SRAM_BASE + d_scan, next | UBC_NDE | (NCH * elems_per_scan),
		 SRAM_BASE + scan_off);
	put_desc(d_tail, 0, next | (NCH * gap_elems), SRAM_BASE + tail_off);
	return 0;
}

static inline void pwm_w(unsigned int reg, unsigned int v) { writel(v, PWM_BASE + reg); }
static inline unsigned int pwm_r(unsigned int reg) { return readl(PWM_BASE + reg); }
static inline void dma_w(unsigned int reg, unsigned int v) { writel(v, XDMAC_BASE + reg); }
static inline unsigned int dma_r(unsigned int reg) { return readl(XDMAC_BASE + reg); }

static void engine_start(void)
{
	const unsigned int mask = 1u << CHAN;
	unsigned int abc[3], c;

	/* GCLK low before the PWM takes the pin: a falling edge, not counted. */
	pio_w(gb_bank, PIO_CODR, oe_bit);

	pwm_w(PWM_DIS, SYNC_ALL);
	while (pwm_r(PWM_SR) & SYNC_ALL)
		;
	for (c = 0; c < NCH; c++)
		pwm_w(PWM_CMR(c), CMR_CPOL | CMR_DPOLI);    /* MCK1, left aligned */
	pwm_w(PWM_CPRD0, period);
	abc_duties(0, abc);
	pwm_w(PWM_CDTY(0), 0);
	for (c = 1; c < NCH; c++)
		pwm_w(PWM_CDTY(c), abc[c - 1]);
	pwm_w(PWM_SCM, SYNC_ALL | SCM_UPDM_DMA);
	pwm_w(PWM_SCUP, UPR);
	(void)pwm_r(PWM_ISR2);

	sram_w(d_scan, SRAM_BASE + d_scan);     /* loop again (undo engine_stop) */

	dma_w(XDMAC_GD, mask);
	while (dma_r(XDMAC_GS) & mask)
		;
	dma_w(XDMAC_GCFG, GCFG_P2M);
	dma_w(XDMAC_GWAC, GWAC_P2M);
	dma_w(CH(C_CID), 0xFFFFFFFFu);
	(void)dma_r(CH(C_CIS));
	dma_w(CH(C_CBC), 0);
	dma_w(CH(C_CDS_MSP), 0);
	dma_w(CH(C_CSUS), 0);
	dma_w(CH(C_CDUS), 0);
	dma_w(CH(C_CC), cc_cfg);
	dma_w(CH(C_CNDA), SRAM_BASE + d_scan);
	dma_w(CH(C_CNDC), CNDC_NDE | CNDC_NDSUP | CNDC_NDDUP | CNDC_NDV2);
	dma_w(XDMAC_GE, mask);

	pins_pwm(1);
	pwm_w(PWM_ENA, 1);                    /* CHID0 starts the whole sync group */
}

/* End the loop after the current scan plus the tail: row 0, GCLK low.
 * Then the pins go back to GPIO, as libspwm's engine_stop(). */
static void engine_stop(void)
{
	const unsigned int mask = 1u << CHAN;
	unsigned int i;

	sram_w(d_scan, SRAM_BASE + d_tail);
	for (i = 0; i < 10000 && (dma_r(XDMAC_GS) & mask); i++)
		udelay(10);                     /* a scan is ~264 us; 100 ms is a fault */
	if (dma_r(XDMAC_GS) & mask) {
		dma_w(XDMAC_GD, mask);
		while (dma_r(XDMAC_GS) & mask)
			;
		dbg_info("SPWM: engine did not stop on its own\n");
	}
	udelay(200);                            /* let the last duties reach the pins */
	pwm_w(PWM_DIS, SYNC_ALL);
	pins_pwm(0);
}

/* ------------------------------------------------------------------ API */

void spwm_logo_show(void)
{
	pmc_enable_periph_clock(AT91C_ID_PIOA, PMC_PERIPH_CLK_DIVIDER_NA);
	pmc_enable_periph_clock(AT91C_ID_PWM, PMC_PERIPH_CLK_DIVIDER_NA);
	pmc_enable_periph_clock(AT91C_ID_XDMAC1, PMC_PERIPH_CLK_DIVIDER_NA);

	setup_masks();
	pins_gpio();

#if SCREEN_ON_BANK >= 0
	/* Panel rail on, now that every line is driven. */
	pio_w(SCREEN_ON_BANK, PIO_CODR, 1u << SCREEN_ON_LINE);
	pio_w(SCREEN_ON_BANK, PIO_MSKR, 1u << SCREEN_ON_LINE);
	pio_w(SCREEN_ON_BANK, PIO_CFGR, CFGR_DIR_OUT);
	pio_w(SCREEN_ON_BANK, PIO_SODR, 1u << SCREEN_ON_LINE);
	mdelay(SCREEN_ON_SETTLE_MS);
#endif

	if (engine_build()) {
		dbg_info("SPWM: scan table does not fit in SRAM, no logo\n");
		return;
	}

	/* libspwm spwm_open(): VSYNC, config registers, black frame with CPU
	 * GCLK bursts, present, start the engine. */
	send_latches(CMD_VSYNC);
	udelay(1000);
	load_config();
	load_frame(0, 1);
	present();
	engine_start();

	/* libspwm spwm_show(): the logo goes into the hidden buffer while the
	 * engine scans, then a clean stop at row 0, VSYNC, scan again. */
	load_frame(1, 0);
	engine_stop();
	send_latches(CMD_VSYNC);
	engine_start();
	dbg_info("SPWM: logo on, scan engine running (PWM ch0-3, XDMAC1 ch31)\n");
}
