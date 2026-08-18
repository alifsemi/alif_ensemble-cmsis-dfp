/**
 * @file isp_clock_decode.h
 * @brief ISP GET CLOCK decode and display - mirrors clock_decode.py
 */

#ifndef CLOCK_DECODE_H
#define CLOCK_DECODE_H

#include "services_lib_api.h"
#include <stdint.h>
#include <stdio.h>

extern void TEST_print(uint32_t services_handle, char *fmt, ...);

/*******************************************************************************
 *  S T A T U S   B I T   D E F I N I T I O N S
 ******************************************************************************/

#define PLL_CLK_STATUS_XTAL_STARTED (1 << 0)
#define PLL_CLK_STATUS_PLL_LOCKED (1 << 1)
#define PLL_CLK_STATUS_SE_CLOCK_PLL (1 << 2)

#define CLK_FREQ_MASK_A32 (0x0000FF00)
#define CLK_FREQ_SHIFT_A32 (8)
#define CLK_FREQ_MASK_M55HP (0x00FF0000)
#define CLK_FREQ_SHIFT_M55HP (16)
#define CLK_FREQ_MASK_M55HE (0xFF000000)
#define CLK_FREQ_SHIFT_M55HE (24)

/*******************************************************************************
 *  F R E Q U E N C Y   E N U M E R A T I O N
 ******************************************************************************/

/*******************************************************************************
 *  B I T - F I E L D   O V E R L A Y S
 ******************************************************************************/

typedef union {
  struct {
    uint32_t en_xtal : 1;
    uint32_t faststart : 1;
    uint32_t __reserved__0 : 1;
    uint32_t en_bfr_clkpll : 1;
    uint32_t en_bfr_dig_2x : 1;
    uint32_t en_bfr_dig : 1;
    uint32_t boost : 1;
    uint32_t xtal_cap : 4;
    uint32_t gm_pfet : 5;
    uint32_t gm_nfet : 5;
    uint32_t sel_doubler_output_duty_cycle : 2;
    uint32_t sel_doubler_input_duty_cycle : 5;
    uint32_t sel_ibg : 1;
    uint32_t ibres_cont : 2;
    uint32_t __reserved__1 : 1;
  } b;
  uint32_t w;
} xo_reg1_fields_t;

typedef union {
  struct {
    uint32_t osc_rc_32k_freq_cont : 4;
    uint32_t ret_ldo_cont_3_0 : 4;
    uint32_t ret_ldo_vbat_en : 1;
    uint32_t ret_ldo_vbat_shunt_en : 1;
    uint32_t ret_ldo_vddmain_en : 1;
    uint32_t ret_ldo_vdd_main_shunt_en : 1;
    uint32_t xtal32k_en : 1;
    uint32_t xtal32k_kick : 1;
    uint32_t lpcomp_clk32k_en : 1;
    uint32_t xtal32k_gm_cont : 4;
    uint32_t xtal32k_cap_cont : 6;
    uint32_t bor_en : 1;
    uint32_t bor_hyst : 3;
    uint32_t bor_thresh : 3;
  } b;
  uint32_t w;
} vbat_ana_reg1_fields_t;

typedef union {
  struct {
    uint32_t __reserved__ : 1;
    uint32_t pmubg_vref_cont : 4;
    uint32_t dig_ldo_18_en : 1;
    uint32_t dig_ldo_cont : 4;
    uint32_t osc_76Mrc_cont_bit0 : 1;
    uint32_t osc_76M_div_cont_fast : 3;
    uint32_t osc_76Mrc_cont : 5;
    uint32_t osc_76M_div_cont_slow : 3;
    uint32_t ana_periph_bg_ena : 1;
    uint32_t ana_periph_LDO_en : 1;
    uint32_t comp_lp_en : 1;
    uint32_t comp_lp0_in_p_sel : 2;
    uint32_t comp_lp0_in_m_sel : 2;
    uint32_t comp_lp0_hyst : 3;
  } b;
  uint32_t w;
} vbat_ana_reg2_fields_t;

/*******************************************************************************
 *  H E L P E R S
 ******************************************************************************/

static const char *clk_frequency_to_string(uint32_t freq)
{
  switch ((clock_frequency_t)freq) {
  case CLOCK_FREQUENCY_800MHZ:
    return "800MHz";
  case CLOCK_FREQUENCY_400MHZ:
    return "400MHz";
  case CLOCK_FREQUENCY_300MHZ:
    return "300MHz";
  case CLOCK_FREQUENCY_200MHZ:
    return "200MHz";
  case CLOCK_FREQUENCY_160MHZ:
    return "160MHz";
  case CLOCK_FREQUENCY_120MHZ:
    return "120MHz";
  case CLOCK_FREQUENCY_80MHZ:
    return "80MHz";
  case CLOCK_FREQUENCY_60MHZ:
    return "60MHz";
  case CLOCK_FREQUENCY_100MHZ:
    return "100MHz";
  case CLOCK_FREQUENCY_50MHZ:
    return "50MHz";
  case CLOCK_FREQUENCY_20MHZ:
    return "20MHz";
  case CLOCK_FREQUENCY_10MHZ:
    return "10MHz";
  case CLOCK_FREQUENCY_76_8_RC_MHZ:
    return "76.8MHz RC";
  case CLOCK_FREQUENCY_38_4_RC_MHZ:
    return "38.4MHz RC";
  case CLOCK_FREQUENCY_76_8_XO_MHZ:
    return "76.8MHz XO";
  case CLOCK_FREQUENCY_38_4_XO_MHZ:
    return "38.4MHz XO";
  case CLOCK_FREQUENCY_DISABLED:
    return "Disabled";
  default:
    return "Unknown";
  }
}

static void clk_print_status(uint32_t services_handle, uint32_t status)
{
	TEST_print(services_handle, "Clock Status\n");
	TEST_print(services_handle, "%s\n",
		(status & PLL_CLK_STATUS_XTAL_STARTED) ? "  HFXTAL STARTED"
							   : "  HFXTAL OFF");
	TEST_print(services_handle, "%s\n",
		(status & PLL_CLK_STATUS_PLL_LOCKED) ? "  PLL LOCKED"
							 : "  PLL OFF");
	TEST_print(services_handle, "%s\n",
		(status & PLL_CLK_STATUS_SE_CLOCK_PLL) ? "  SE CLOCK: PLL"
							   : "  SE CLOCK: HFRC");
}

/*******************************************************************************
 *  M A I N   D E C O D E R
 ******************************************************************************/

/**
 * @brief Decode and display clock_get_t - mirrors display_clock_info() in
 * clock_decode.py
 * @param clk  pointer to populated clock_get_t
 */
void display_clock_info(uint32_t services_handle, const clock_get_t *clk)
{
	/* ── Status flags ───────────────────────────────────────────────────── */
	clk_print_status(services_handle, clk->status);

	/* ── CPU frequency enumerations ─────────────────────────────────────── */
	TEST_print(services_handle, "CLK freq A32    %s\n",
		clk_frequency_to_string((clk->status & CLK_FREQ_MASK_A32) >>
					CLK_FREQ_SHIFT_A32));
	TEST_print(services_handle, "CLK freq M55-HP %s\n",
		clk_frequency_to_string((clk->status & CLK_FREQ_MASK_M55HP) >>
					CLK_FREQ_SHIFT_M55HP));
	TEST_print(services_handle, "CLK freq M55-HE %s\n",
		clk_frequency_to_string((clk->status & CLK_FREQ_MASK_M55HE) >>
					CLK_FREQ_SHIFT_M55HE));

	/* ── SE / CM0+ measured frequency ───────────────────────────────────── */
	TEST_print(services_handle, "SE frequency %.2fMHz\n", clk->se_frequency_mhz);
	TEST_print(services_handle, "LF source %s\n",
		(clk->lf_oscillator_source == OSCILLATOR_SOURCE_RC) ? "LFRC"
								 : "LFXO");
	TEST_print(services_handle, "LF frequency %uHz\n", clk->lf_frequency_hz);

	/* ── Raw register dump ───────────────────────────────────────────────── */
	TEST_print(services_handle, "\nRegisters:\n");
	TEST_print(services_handle, "%-16s: 0x%08X\n", "hostcpuclk_ctrl",
		clk->hostcpuclk_ctrl);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "hostcpuclk_div0",
		clk->hostcpuclk_div0);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "hostcpuclk_div1",
		clk->hostcpuclk_div1);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "aclk_ctrl", clk->aclk_ctrl);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "aclk_div0", clk->aclk_div0);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "OSC CTRL", clk->cgu_osc_ctrl);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "PLL_LOCK_CTRL",
		clk->cgu_pll_lock_ctrl);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "PLL_CLK_SEL",
		clk->cgu_pll_sel);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "ESCLK_SEL",
		clk->cgu_escclk_sel);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "CLK_ENA", clk->cgu_clk_ena);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "SYSTOP_CLK_DIV",
		clk->systop_clk_div);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "MISC_REG1", clk->misc_reg1);

	/* ── xo_reg1 with bit-field decode ──────────────────────────────────── */
	TEST_print(services_handle, "%-16s: 0x%08X\n", "XO_REG1", clk->xo_reg1);
	{
		xo_reg1_fields_t xo = {.w = clk->xo_reg1};
		TEST_print(services_handle, "   xtal_cap:%d  gm_pfet:%d  gm_nfet:%d\n",
			xo.b.xtal_cap, xo.b.gm_pfet, xo.b.gm_nfet);
	}

	TEST_print(services_handle, "%-16s: 0x%08X\n", "PD4_CLK_SEL",
		clk->pd4_clk_sel);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "PD4_CLK_PLL",
		clk->pd4_clk_pll);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "MISC_CTRL", clk->misc_ctrl);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "DCDC_REG1", clk->dcdc_reg1);
	TEST_print(services_handle, "%-16s: 0x%08X\n", "DCDC_REG2", clk->dcdc_reg2);

	/* ── vbat_ana_reg1 with bit-field decode ─────────────────────────────── */
	TEST_print(services_handle, "%-16s: 0x%08X\n", "VBAT_ANA_REG1",
		clk->vbat_ana_reg1);
	{
		vbat_ana_reg1_fields_t ana1 = {.w = clk->vbat_ana_reg1};
		TEST_print(services_handle,
			"   osc_rc_32k_freq_cont:%d  xtal32k_en:%d  xtal32k_gm_cont:%d"
			"  xtal32k_cap_cont:%d  bor_en:%d  bor_hyst:%d  bor_thresh:%d\n",
			ana1.b.osc_rc_32k_freq_cont, ana1.b.xtal32k_en,
			ana1.b.xtal32k_gm_cont, ana1.b.xtal32k_cap_cont, ana1.b.bor_en,
			ana1.b.bor_hyst, ana1.b.bor_thresh);
	}

	/* ── vbat_ana_reg2 with bit-field decode ─────────────────────────────── */
	TEST_print(services_handle, "%-16s: 0x%08X\n", "VBAT_ANA_REG2",
		clk->vbat_ana_reg2);
	{
		vbat_ana_reg2_fields_t ana2 = {.w = clk->vbat_ana_reg2};
		TEST_print(services_handle,
			"   pmubg_vref_cont:%d  osc_76Mrc_cont_bit0:%d"
			"  osc_76M_div_cont_fast:%d  osc_76Mrc_cont:%d"
			"  osc_76M_div_cont_slow:%d\n",
			ana2.b.pmubg_vref_cont, ana2.b.osc_76Mrc_cont_bit0,
			ana2.b.osc_76M_div_cont_fast, ana2.b.osc_76Mrc_cont,
			ana2.b.osc_76M_div_cont_slow);
	}
}

uint32_t get_clock_data(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  clock_get_t all_clock;

  SERVICES_clocks_get_data(services_handle, &all_clock, &error_code);

  display_clock_info(services_handle, (clock_get_t *)&all_clock);

  return error_code;
}
#endif /* CLOCK_DECODE_H */
