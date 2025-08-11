/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     sys_ctrl_rtc.h
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     24-04-2023
 * @brief    LPRTC system control Specific Header file.
 ******************************************************************************/

#ifndef SYS_CTRL_RTC_H_
#define SYS_CTRL_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "soc.h"
#include "sys_utils.h"

#define RTC_CLK_ENABLE (1U << 0U) /* Enable RTC clock */

/**
  \fn           static inline void enable_lprtc_clk (LPRTC_INSTANCE inst)
  \brief        VBAT clock enable for LPRTC
  \param        inst LPRTC instance number
  \return       none
*/
static inline void enable_lprtc_clk(LPRTC_INSTANCE inst)
{
#if SOC_FEAT_HAS_LPRTC1
    if (inst == LPRTC0_INSTANCE) {
        VBAT->RTCA_CLK_EN |= RTC_CLK_ENABLE;
    } else {
        VBAT->RTCB_CLK_EN |= RTC_CLK_ENABLE;
    }
#else
    ARG_UNUSED(inst);
    VBAT->RTC_CLK_EN |= RTC_CLK_ENABLE;
#endif
}

/**
  \fn           static inline void disable_lprtc_clk (LPRTC_INSTANCE inst)
  \brief        VBAT clock disable for LPRTC
  \param        inst LPRTC instance number
  \return       none
*/
static inline void disable_lprtc_clk(LPRTC_INSTANCE inst)
{
#if SOC_FEAT_HAS_LPRTC1
    if (inst == LPRTC0_INSTANCE) {
        VBAT->RTCA_CLK_EN &= RTC_CLK_ENABLE;
    } else {
        VBAT->RTCB_CLK_EN &= RTC_CLK_ENABLE;
    }
#else
    ARG_UNUSED(inst);
    VBAT->RTC_CLK_EN &= ~RTC_CLK_ENABLE;
#endif
}

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_RTC_H_ */
