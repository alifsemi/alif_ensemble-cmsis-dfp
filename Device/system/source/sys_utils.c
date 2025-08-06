/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/******************************************************************************
 * @file     sys_utils.c
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @brief    System  Utility functions
 * @version  V1.0.0
 * @date     13. May 2021
 * @bug      None
 * @Note     None
 ******************************************************************************/
#include <sys_utils.h>
#include "sys_clocks.h"

#define CNTCR_EN (1U << 0) /* Enable the counter */

/**
  \fn          void sys_busy_loop_init_ns(void)
  \brief       Initialize the REFCLK Counter Module to use as busy loop
  \note        This function is not initialized at boot up. User may
               choose to initialize based on application requirements
  \return      none
*/
void sys_busy_loop_init_ns(void)
{
    REFCLK_CNTControl->CNTCR |= CNTCR_EN;
}

/**
  \fn          int32_t sys_busy_loop_ns(uint32_t delay_ns)
  \brief       Using REFCLK counter for delay.
  \note        REFCLK Counter module should be running before calling this.
               User should call sys_busy_loop_init_ns() once to
               make sure the module is running.
               Minimum delay = 10ns (Note: depends on refclk freq)
               Maximum delay = 100ms
  \param[in]   delay_ns delay in nano seconds.
  \return      0 for Success -1 for Overflow error.
*/
int32_t sys_busy_loop_ns(uint32_t delay_ns)
{
    /*
     * Restricting the users to use this function for delays less than 104.8576ms
     */
#define SYS_MAX_DELAY_IN_NANOSECONDS (100 * 1024 * 1024)

    uint32_t curr_cntcvl, cntcvl = REFCLK_CNTRead->CNTCVL;
    uint32_t delay_in_cycles;
    uint32_t diff = 0;

    if (delay_ns > SYS_MAX_DELAY_IN_NANOSECONDS) {
        return -1;
    }

    if (SystemREFClock == 100000000U) {
        /* Fast path: Handling best-case scenario when the REFCLK is 100M */
        delay_in_cycles = delay_ns / 10;
    } else {
        delay_in_cycles =
            (uint32_t) ((((uint64_t) delay_ns * SystemREFClock) + 999999999U) / 1000000000U);
    }

    while (diff <= delay_in_cycles) {
        curr_cntcvl = REFCLK_CNTRead->CNTCVL;

        diff        = curr_cntcvl - cntcvl;
    }

    return 0;
}

/**
  \fn          void sys_busy_loop_init(void)
  \brief       Initialize the S32K Counter Module to use as busy loop
  \return      none
*/
void sys_busy_loop_init(void)
{
    S32K_CNTControl->CNTCR |= CNTCR_EN;
}

/**
  \fn          int32_t sys_busy_loop_us(uint32_t delay_us)
  \brief       Using S32K counter for delay.
               Minimum delay = 30.51us
               Maximum delay = 100ms
  \param[in]   delay_us delay in micro seconds.
  \return      0 for Success -1 for Overflow error.
*/
int32_t sys_busy_loop_us(uint32_t delay_us)
{
    /*
     * Restricting the users to use this function for delays less than 102.4ms
     */
#define SYS_MAX_DELAY_IN_MICROSECONDS (100 * 1024)

    uint32_t delay_in_cycles;
    uint32_t diff = 0;
    uint32_t cntcv, curr_cntcv;

    if (delay_us > SYS_MAX_DELAY_IN_MICROSECONDS) {
        return -1;
    }

    cntcv           = S32K_CNTRead->CNTCVL;

    delay_in_cycles = (((delay_us * 32768U) + 999999U) / 1000000U);

    while (diff <= delay_in_cycles) {
        curr_cntcv = S32K_CNTRead->CNTCVL;

        diff       = curr_cntcv - cntcv;
    }

    return 0;
}
