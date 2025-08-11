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
 * @file     sys_clocks.c
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @version  V1.0.0
 * @date     29-Nov-2023
 * @brief    Clock information
 ******************************************************************************/
#include <sys_clocks.h>
#include "soc_features.h"

/*----------------------------------------------------------------------------
  System AXI Clock Variable(SYST_ACLK)
 *----------------------------------------------------------------------------*/
uint32_t SystemAXIClock   = SOC_FEAT_ACLK_MAX_HZ;

/*----------------------------------------------------------------------------
  System AHB Clock Variable(SYST_HCLK)
 *----------------------------------------------------------------------------*/
uint32_t SystemAHBClock   = SOC_FEAT_HCLK_MAX_HZ;

/*----------------------------------------------------------------------------
  System APB Clock Variable(SYST_PCLK)
 *----------------------------------------------------------------------------*/
uint32_t SystemAPBClock   = SOC_FEAT_PCLK_MAX_HZ;

/*----------------------------------------------------------------------------
  System REF Clock Variable(SYST_REFCLK)
 *----------------------------------------------------------------------------*/
uint32_t SystemREFClock   = SOC_FEAT_REFCLK_MAX_HZ;

/*----------------------------------------------------------------------------
  System HFOSC Clock Variable(HFOSC_CLK)
 *----------------------------------------------------------------------------*/
uint32_t SystemHFOSCClock = SOC_FEAT_HFOSC_MAX_HZ;

/*----------------------------------------------------------------------------
  Get System AXI Clock function
 *----------------------------------------------------------------------------*/
uint32_t GetSystemAXIClock(void)
{
    return SystemAXIClock;
}

/*----------------------------------------------------------------------------
  Get System AHB Clock function
 *----------------------------------------------------------------------------*/
uint32_t GetSystemAHBClock(void)
{
    return SystemAHBClock;
}

/*----------------------------------------------------------------------------
  Get System APB Clock function
 *----------------------------------------------------------------------------*/
uint32_t GetSystemAPBClock(void)
{
    return SystemAPBClock;
}

/*----------------------------------------------------------------------------
  Get System REF Clock function
 *----------------------------------------------------------------------------*/
uint32_t GetSystemREFClock(void)
{
    return SystemREFClock;
}

/*----------------------------------------------------------------------------
  Get System HFOSC Clock function
 *----------------------------------------------------------------------------*/
uint32_t GetSystemHFOSClock(void)
{
    return SystemHFOSCClock;
}
