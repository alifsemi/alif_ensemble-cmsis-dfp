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

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_OSPI_H_ */
