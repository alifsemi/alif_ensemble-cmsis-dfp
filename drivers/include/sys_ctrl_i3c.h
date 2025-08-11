/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SYS_CTRL_I3C_H
#define SYS_CTRL_I3C_H

#include <stdint.h>
#include "soc_features.h"
#include "sys_clocks.h"

/**
 * enum I3C_INSTANCE
 * I3C instances
 */
typedef enum _I3C_INSTANCE {
    I3C_INSTANCE_0,   /**< I3C instance - 0       */
    I3C_INSTANCE_LP_0 /**< I3C instance - LP - 0  */
} I3C_INSTANCE;

#define HE_CLK_ENA_I3C_CKEN   (1 << 14)
#define I3C_CTRL_DMA_SEL_DMA2 (1 << 24)
#define I3C_CTRL_CKEN         (1 << 0)
/**
  \fn          static inline void enable_i3c_clock(const I3C_INSTANCE inst)
  \brief       Enables I3C clock
  \param       inst : I3C instance number
  \return      none
*/
static inline void enable_i3c_clock(const I3C_INSTANCE inst)
{
    switch (inst) {
    case I3C_INSTANCE_0:
        CLKCTL_PER_SLV->I3C_CTRL |= I3C_CTRL_CKEN;
        break;

    case I3C_INSTANCE_LP_0:
        M55HE_CFG->HE_CLK_ENA |= HE_CLK_ENA_I3C_CKEN;
        break;

    default:
        break;
    }
}

/**
  \fn          static inline void disable_i3c_clock(const I3C_INSTANCE inst)
  \brief       Disables I3C clock
  \param       inst : I3C instance number
  \return      none
*/
static inline void disable_i3c_clock(const I3C_INSTANCE inst)
{
    switch (inst) {
    case I3C_INSTANCE_0:
        CLKCTL_PER_SLV->I3C_CTRL &= ~I3C_CTRL_CKEN;
        break;

    case I3C_INSTANCE_LP_0:
        M55HE_CFG->HE_CLK_ENA &= ~HE_CLK_ENA_I3C_CKEN;
        break;

    default:
        break;
    }
}

/**
  \fn          static inline void select_i3c_dma2(const I3C_INSTANCE inst)
  \brief       Selects DMA2 for communication
  \param       inst : I3C instance number
  \return      none
*/
static inline void select_i3c_dma2(const I3C_INSTANCE inst)
{
    if (inst == I3C_INSTANCE_0) {
        CLKCTL_PER_SLV->I3C_CTRL |= I3C_CTRL_DMA_SEL_DMA2;
    }
}

/**
  \fn          static inline uint32_t get_i3c_core_clock(void)
  \brief       Gets I3C input core clock
  \return      Input Core clock value
*/
static inline uint32_t get_i3c_core_clock(void)
{
#if SOC_FEAT_I3C_CORE_CLK_AXI
    /* Gets the system AXI clock */
    return GetSystemAXIClock();
#else
    /* Gets the system APB clock */
    return GetSystemAPBClock();
#endif
}

#endif /* SYS_CTRL_I3C_H */
