/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef CLK_H_
#define CLK_H_

#include <peripheral_types.h>

#ifndef AXI_CLOCK
#define AXI_CLOCK 400000000
#endif

#ifndef AHB_CLOCK
#define AHB_CLOCK 200000000
#endif

#ifndef APB_CLOCK
#define APB_CLOCK 100000000
#endif

#ifndef RTSS_HE_CLK
#define RTSS_HE_CLK 160000000
#endif

#ifndef HFOSC_CLK
#define HFOSC_CLK 38400000
#endif

#define SYST_PCLK               APB_CLOCK
#define I2C_PERIPHERAL_CLOCK    APB_CLOCK

#define PLL_CLK1                800000000
#define PLL_CLK3                480000000

#define CDC200_PIXCLK           AXI_CLOCK

static inline void enable_force_peripheral_functional_clk(void)
{
    CLKCTL_PER_SLV->EXPMST0_CTRL |= EXPMST0_CTRL_IPCLK_FORCE;
}

static inline void disable_force_peripheral_functional_clk(void)
{
    CLKCTL_PER_SLV->EXPMST0_CTRL &= ~EXPMST0_CTRL_IPCLK_FORCE;
}

static inline void enable_force_apb_interface_clk(void)
{
    CLKCTL_PER_SLV->EXPMST0_CTRL |= EXPMST0_CTRL_PCLK_FORCE;
}

static inline void disable_force_apb_interface_clk(void)
{
    CLKCTL_PER_SLV->EXPMST0_CTRL &= ~EXPMST0_CTRL_PCLK_FORCE;
}

static inline void enable_cgu_clk160m(void)
{
    CGU->CLK_ENA |= CLK_ENA_CLK160M;
}

static inline void disable_cgu_clk160m(void)
{
    CGU->CLK_ENA &= ~CLK_ENA_CLK160M;
}
static inline void enable_cgu_clk100m(void)
{
    CGU->CLK_ENA |= CLK_ENA_CLK100M;
}

static inline void disable_cgu_clk100m(void)
{
    CGU->CLK_ENA &= ~CLK_ENA_CLK100M;
}

static inline void enable_cgu_clk20m(void)
{
    CGU->CLK_ENA |= CLK_ENA_CLK20M;
}

static inline void disable_cgu_clk20m(void)
{
    CGU->CLK_ENA &= ~CLK_ENA_CLK20M;
}

static inline void enable_cgu_clk38p4m(void)
{
    CGU->CLK_ENA |= CLK_ENA_CLK38P4M;
}

static inline void disable_cgu_clk38p4m(void)
{
    CGU->CLK_ENA &= ~CLK_ENA_CLK38P4M;
}

static inline void enable_gpu_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_GPU_CKEN;
}

static inline void disable_gpu_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_GPU_CKEN;
}

static inline void enable_sdc_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_SDC_CKEN;
}

static inline void disable_sdc_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_SDC_CKEN;
}

static inline void enable_usb_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_USB_CKEN;
}

static inline void disable_usb_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_USB_CKEN;
}

#endif /* CLK_H_ */
