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
 * @file     sys_utils.h
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @brief    System Header File for Utility functions
 * @version  V1.0.0
 * @date     13. May 2021
 * @bug      None
 * @Note     None
 ******************************************************************************/
#ifndef SYS_UTILS_H
#define SYS_UTILS_H

#include <stdbool.h>
#include "alif.h"

#ifdef __cplusplus
extern "C" {
#endif

// common helper macros

/* for Unused Arguments. */
#ifndef ARG_UNUSED
#define ARG_UNUSED(arg) ((void) arg)
#endif

#ifndef ENABLE
#define ENABLE (0x1)
#endif

#ifndef DISABLE
#define DISABLE (0x0)
#endif

#ifndef MASK
/**
 * @Note : 1st arg should be higher bit & 2nd arg should be lower i.e. (h > l)
 */
#define MASK(h, l)   (((~(0U)) << (l)) & (~(0U) >> (32 - 1 - (h))))
#define MASK64(h, l) (((~(0UL)) << (l)) & (~(0UL) >> (32 - 1 - (h))))
#endif

#define DIV_ROUND_UP(n, d)               (((n) + (d) - 1) / (d))

#define BIT(nr)                          (1UL << (nr))
#define SET_BIT(REG, BIT_Msk)            ((REG) |= (BIT_Msk))
#define CLEAR_BIT(REG, BIT_Msk)          ((REG) &= ~(BIT_Msk))
#define READ_BIT(REG, BIT_Msk)           ((REG) & (BIT_Msk))
#define CLEAR_REG(REG)                   ((REG) = (0x0))
#define WRITE_REG(REG, VAL)              ((REG) = (VAL))
#define READ_REG(REG)                    ((REG))

/*Below Macros are helpful for bitwise operator (input argument 1 means bit 0)*/
#define BIT64(n)                         ((1ULL) << n)
#define REG64_SET_ONE_BIT(src, n)        (src |= (BIT64(n)))
#define REG64_CLR_ONE_BIT(src, n)        (src &= ~(BIT64(n)))
#define REG32_SET_ONE_BIT(src, n)        (src |= (BIT(n)))
#define REG32_CLR_ONE_BIT(src, n)        (src &= ~(BIT(n)))

#define REG64_SET_M_AND_N_BIT(src, m, n) (src |= ((BIT64(m) | BIT64(n))))
#define REG64_CLR_M_AND_N_BIT(src, m, n) (src &= (~(BIT64(m) | BIT64(n))))
#define REG32_SET_M_AND_N_BIT(src, m, n) (src |= ((BIT(m) | BIT(n))))
#define REG32_CLR_M_AND_N_BIT(src, m, n) (src &= (~(BIT(m) | BIT(n))))

#define REG64_SET_M_TO_N_BIT(src, h, l)  (src |= MASK64(h, l))
#define REG64_CLR_M_TO_N_BIT(src, h, l)  (src &= ~MASK64(h, l))
#define REG32_SET_M_TO_N_BIT(src, h, l)  (src |= MASK(h, l))
#define REG32_CLR_M_TO_N_BIT(src, h, l)  (src &= ~MASK(h, l))

#define REG32_TOGGLE_BIT(src, m)         (src ^= BIT(m))
#define REG64_TOGGLE_BIT(src, m)         (src ^= BIT64(m))

#define CONVERT_US_TO_NS(n)              (n * 1000U)

/* Function documentation */

/**
  \fn          void sys_busy_loop_init_ns(void)
  \brief       Initialize the REFCLK Counter Module to use as busy loop
  \note        This function is not initialized at boot up. User may
               choose to initialize based on application requirements
  \return      none
*/
void sys_busy_loop_init_ns(void);

/**
  \fn          int32_t sys_busy_loop_ns(uint32_t delay_ns)
  \brief       Using REFCLK counter for delay.
  \note        REFCLK Counter module should be running before calling this.
               User should call sys_high_res_busy_loop_init() once to
               make sure the module is running.
               Minimum delay = 10ns (Note: depends on refclk freq)
               Maximum delay = 100ms
  \param[in]   delay_ns delay in nano seconds.
  \return      0 for Success -1 for Overflow error.
*/
int32_t sys_busy_loop_ns(uint32_t delay_ns);

/**
  \fn          void sys_busy_loop_init(void)
  \brief       Initialize the S32K Counter Module to use as busy loop
  \return      none
*/
void sys_busy_loop_init(void);

/**
  \fn          int32_t sys_busy_loop_us(uint32_t delay_us)
  \brief       Using S32K counter for delay.
               Minimum delay = 30.51us
               Maximum delay = 100ms
  \param[in]   delay_us delay in micro seconds.
  \return      0 for Success -1 for Overflow error.
*/
int32_t sys_busy_loop_us(uint32_t delay_us);

/**
  \fn          bool RTSS_Is_TCM_Addr(const volatile void *local_addr)
  \brief       Return true if the local_addr is in TCM
  \param[in]   local_addr  local address
  \return      bool True: if local_addr resides in TCM
*/
__STATIC_FORCEINLINE
bool RTSS_Is_TCM_Addr(const volatile void *local_addr)
{
    uint32_t addr = (uint32_t) local_addr;

    return ((addr < (ITCM_BASE + ITCM_REGION_SIZE)) ||
            ((addr > DTCM_BASE) && (addr < (DTCM_BASE + DTCM_REGION_SIZE))));
}

/**
  \fn          uint32_t LocalToGlobal(const volatile void *local_addr)
  \brief       Return the corresponding global address
  \param[in]   local_addr  local address to convert
  \return      uint32_t global address
*/
__STATIC_INLINE
uint32_t LocalToGlobal(const volatile void *local_addr)
{
    /* Only for local TCM address, we need to map it to global address space, rest
     * for all other memories like SRAM0/1, MRAM, OctalSPI etc we can pass the address
     * as-is as those are accessed using global address
     */
    uint32_t addr = (uint32_t) local_addr;

    if ((addr >= DTCM_BASE) && (addr < (DTCM_BASE + DTCM_REGION_SIZE))) {
        return (addr & (DTCM_ALIAS_BIT - 1)) + DTCM_GLOBAL_BASE;
    } else if ((addr < (ITCM_BASE + ITCM_REGION_SIZE))) {
        return (addr & (ITCM_ALIAS_BIT - 1)) + ITCM_GLOBAL_BASE;
    } else {
        return addr;
    }
}

/**
  \fn          void* GlobalToLocal(uint32_t global_addr)
  \brief       Return the corresponding local memory alias address
  \param[in]   global_addr  address to convert
  \return      void* local memory alias address
*/
__STATIC_INLINE
void *GlobalToLocal(uint32_t global_addr)
{
    /* Only for local TCM address, we need to map it to local address space, rest
     * for all other memories like SRAM0/1, MRAM, OctalSPI etc we can pass the address
     * as-is as it is global.
     */
    uint32_t addr = global_addr;

#if CONFIG_MAP_GLOBAL_TO_LOCAL_TCM_ALIAS
    if ((addr >= DTCM_GLOBAL_BASE) && (addr < (DTCM_GLOBAL_BASE + DTCM_REGION_SIZE))) {
        return (void *) (addr - DTCM_GLOBAL_BASE + (DTCM_BASE | DTCM_ALIAS_BIT));
    } else if ((addr >= ITCM_GLOBAL_BASE) && (addr < (ITCM_GLOBAL_BASE + ITCM_REGION_SIZE))) {
        return (void *) (addr - ITCM_GLOBAL_BASE + (ITCM_BASE | ITCM_ALIAS_BIT));
    } else {
        return ((void *) addr);
    }
#else
    if ((addr >= DTCM_GLOBAL_BASE) && (addr < (DTCM_GLOBAL_BASE + DTCM_SIZE))) {
        return (void *) (addr - DTCM_GLOBAL_BASE + DTCM_BASE);
    } else if ((addr >= ITCM_GLOBAL_BASE) && (addr < (ITCM_GLOBAL_BASE + ITCM_SIZE))) {
        return (void *) (addr - ITCM_GLOBAL_BASE + ITCM_BASE);
    } else {
        return ((void *) addr);
    }
#endif /* CONFIG_MAP_GLOBAL_TO_LOCAL_TCM_ALIAS */
}

#ifdef __cplusplus
}
#endif

#endif /* SYS_UTILS_H */
