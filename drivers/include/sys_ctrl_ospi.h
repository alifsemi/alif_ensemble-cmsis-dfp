/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     sys_ctrl_ospi.h
 * @author   Silesh C V, Manoj A Murudi
 * @email    silesh@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     19-06-2024
 * @brief    Header file for OSPI Control.
 ******************************************************************************/
#ifndef SYS_CTRL_OSPI_H_
#define SYS_CTRL_OSPI_H_

#include "soc.h"
#include "soc_features.h"
#include "sys_clocks.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * enum OSPI_INSTANCE.
 * OSPI instances.
 */
typedef enum _OSPI_INSTANCE {
    OSPI_INSTANCE_0,
    OSPI_INSTANCE_1,
} OSPI_INSTANCE;

#if SOC_FEAT_OSPI_HAS_CLK_ENABLE

/**
  \fn          static inline void enable_ospi_clk(OSPI_INSTANCE drv_instance)
  \brief       enable OSPI clock
  \param       drv_instance: driver instance
  \return      none
*/
static inline void enable_ospi_clk(OSPI_INSTANCE drv_instance)
{
    CLKCTL_PER_SLV->OSPI_CTRL |= (1 << drv_instance);
}

/**
  \fn          static inline void disable_ospi_clk(OSPI_INSTANCE drv_instance)
  \brief       disable OSPI clock
  \param       drv_instance: driver instance
  \return      none
*/
static inline void disable_ospi_clk(OSPI_INSTANCE drv_instance)
{
    CLKCTL_PER_SLV->OSPI_CTRL &= ~(1 << drv_instance);
}

#endif /*SOC_FEAT_OSPI_HAS_CLK_ENABLE*/

#if SOC_FEAT_OSPI_CLK_SELECT

typedef enum _OSPI_CLK_SEL {
    OSPI_CLK_SEL_ACLK = 0,
    OSPI_CLK_SEL_PLL_CLK1_DIV3 = 1,
} OSPI_CLK_SEL;

#define OSPI_CLK_SEL_OFFSET (0)
#define OSPI_CLK_SEL_MASK   (1 << OSPI_CLK_SEL_OFFSET)

/**
 * @brief       Convert OSPI clock selection enum to its corresponding frequency in Hz.
 * @param       sel: OSPI clock selection enum.
 * @return      uint32_t: corresponding frequency in Hz.
 */
static inline uint32_t ospi_core_clock_enum_to_hz(OSPI_CLK_SEL sel)
{
    switch (sel) {
    case OSPI_CLK_SEL_ACLK:
        return GetSystemAXIClock();
    case OSPI_CLK_SEL_PLL_CLK1_DIV3:
        return SOC_FEAT_PLL_CLK1_MAX_HZ / 3;
    }
    return 0;
}

/**
 * @brief       Get the current OSPI clock selection.
 * @return      OSPI_CLK_SEL: current clock selection.
 */
static inline OSPI_CLK_SEL get_ospi_clk_sel(void)
{
    return (OSPI_CLK_SEL)((CGU->MISC_CLK_CTRL & OSPI_CLK_SEL_MASK) >> OSPI_CLK_SEL_OFFSET);
}

/**
 * @brief       Set the OSPI clock selection.
 * @param       sel: clock selection to set.
 * @return      none.
 */
static inline void set_ospi_clk_sel(OSPI_CLK_SEL sel)
{
    CGU->MISC_CLK_CTRL = (CGU->MISC_CLK_CTRL & ~OSPI_CLK_SEL_MASK) | (sel << OSPI_CLK_SEL_OFFSET);
}
#endif /*SOC_FEAT_OSPI_CLK_SELECT*/


/**
 * @brief       Get the current OSPI core clock frequency in Hz.
 * @return      uint32_t: current OSPI core clock frequency.
 */
static inline uint32_t ospi_get_core_clock(void)
{
#if SOC_FEAT_OSPI_CLK_SELECT
    return ospi_core_clock_enum_to_hz(get_ospi_clk_sel());
#endif
    return GetSystemAXIClock();
}

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_OSPI_H_ */
