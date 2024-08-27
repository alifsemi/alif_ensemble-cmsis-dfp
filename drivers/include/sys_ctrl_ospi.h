/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SYS_CTRL_OSPI_H_
#define SYS_CTRL_OSPI_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "peripheral_types.h"

/**
  \fn          static inline void enable_ospi_clk(void)
  \brief       enable OSPI clock
  \return      none
*/
static inline void enable_ospi_clk(void)
{
    CLKCTL_PER_SLV->OSPI_CTRL |= OSPI_CTRL_CKEN;
}

/**
  \fn          static inline void disable_ospi_clk(void)
  \brief       disable OSPI clock
  \return      none
*/
static inline void disable_ospi_clk(void)
{
    CLKCTL_PER_SLV->OSPI_CTRL &= ~OSPI_CTRL_CKEN;
}

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_OSPI_H_ */
