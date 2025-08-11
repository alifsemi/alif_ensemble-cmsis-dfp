/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SYS_CLOCKS_H
#define SYS_CLOCKS_H

#include "soc.h"

#define PERIPH_CLK_ENA_USB_CKEN (1U << 20) /* Enable clock supply for USB */

#if SOC_FEAT_FORCE_ENABLE_SYSTEM_CLOCKS

#define EXPMST0_CTRL_IPCLK_FORCE (1U << 31) /* Force peripherals functional clocks ON  */
#define EXPMST0_CTRL_PCLK_FORCE  (1U << 30) /* Force APB interface (PCLK) clocks ON    */

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

static inline void enable_usb_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_USB_CKEN;
}

static inline void disable_usb_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_USB_CKEN;
}
#endif

/**
  \brief System AXI Clock Frequency (AXI Clock)
*/
extern uint32_t SystemAXIClock;

/**
  \brief System AHB Clock Frequency (AHB Clock)
*/
extern uint32_t SystemAHBClock;

/**
  \brief System APB Clock Frequency (APB Clock)
*/
extern uint32_t SystemAPBClock;

/**
  \brief System REF Clock Frequency (REF Clock)
*/
extern uint32_t SystemREFClock;

/**
  \brief System HFOSC Clock Frequency (HFOSC Clock)
*/
extern uint32_t SystemHFOSCClock;

/**
  \brief  Get System AXI Clock value.

   returns the currently configured AXI clock value.
 */
uint32_t GetSystemAXIClock(void);

/**
  \brief  Get System AHB Clock value.

   returns the currently configured AHB clock value.
 */
uint32_t GetSystemAHBClock(void);

/**
  \brief  Get System APB Clock value.

   returns the currently configured APB clock value.
 */
uint32_t GetSystemAPBClock(void);

/**
  \brief  Get System REF Clock value.

   returns the currently configured REF clock value.
 */
uint32_t GetSystemREFClock(void);

/**
  \brief  Get System HFOSC Clock value.

   returns the currently configured HFOSC clock value.
 */
uint32_t GetSystemHFOSClock(void);

#endif /* SYS_CLOCKS_H */
