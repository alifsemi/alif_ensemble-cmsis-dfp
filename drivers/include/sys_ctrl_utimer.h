/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     sys_ctrl_utimer.h
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     11-06-2025
 * @brief    UTIMER system control Specific Header file.
 ******************************************************************************/

#ifndef SYS_CTRL_UTIMER_H_
#define SYS_CTRL_UTIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "soc.h"

#define HE_CLK_ENA_LPUTIMER_CKEN (1U << 17) /* Enable LP-UITMER clock */

/**
  \fn          static void inline enable_lputimer_clk (void)
  \brief       Enable LPUTIMER clock.
  \param       none
  \return      none
*/
static inline void enable_lputimer_clk(void)
{
    M55HE_CFG->HE_CLK_ENA |= HE_CLK_ENA_LPUTIMER_CKEN;
}

/**
  \fn          static inline void disable_lputimer_clk (void)
  \brief       Disable LPUTIMER clock.
  \param       none
  \return      none
*/
static inline void disable_lputimer_clk(void)
{
    M55HE_CFG->HE_CLK_ENA &= ~HE_CLK_ENA_LPUTIMER_CKEN;
}

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_UTIMER_H_ */
