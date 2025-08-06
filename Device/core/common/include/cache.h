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
 * @file     cache.h
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @brief    Cache Utility functions
 * @version  V1.0.0
 * @date     13. May 2021
 * @bug      None
 * @Note     None
 ******************************************************************************/
#ifndef CACHE_H
#define CACHE_H

#include <stdbool.h>
#include "soc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE (128 * 1024)

// Function documentation

/**
  \fn          bool RTSS_Is_DCache_Dirty(void)
  \brief       Check whether the Data Cache line is dirty
  \return      bool True: if cache is dirty, false otherwise
*/
__STATIC_FORCEINLINE
bool RTSS_Is_DCache_Dirty(void)
{
    uint32_t mscr = MEMSYSCTL->MSCR;

    /* Return True, if Cache is active and not known to be clean */
    if ((mscr & MEMSYSCTL_MSCR_DCACTIVE_Msk) && !(mscr & MEMSYSCTL_MSCR_DCCLEAN_Msk)) {
        return true;
    }

    return false;
}

/**
  \fn          void RTSS_IsGlobalCacheClean_Required (void)
  \brief       Return True if Global Cache Clean operation is required
  return       True : If CacheOperation Required, else False
*/
bool RTSS_IsGlobalCacheClean_Required(void);

/**
  \fn          void RTSS_IsCacheInvalidate_Required_by_Addr (volatile void *addr, int32_t size)
  \brief       Return True if Cache Invalidate operation is required for the provided
               address region else return False.
  \param[in]   addr    address
  \param[in]   size   size of memory block (in number of bytes)
  return       True : If CacheOperation Required, else False
*/
bool RTSS_IsCacheInvalidate_Required_by_Addr(volatile void *addr, int32_t size);

/**
  \fn          void RTSS_IsCacheClean_Required_by_Addr (volatile void *addr, int32_t size)
  \brief       Return True if Cache Clean operation is required for the provided
               address region else return False.
  \param[in]   addr    address
  \param[in]   size   size of memory block (in number of bytes)
  return       True : If CacheOperation Required, else False
*/
bool RTSS_IsCacheClean_Required_by_Addr(volatile void *addr, int32_t size);

/**
  \fn          void RTSS_InvalidateICache_by_Addr (volatile void *addr, int32_t isize)
  \brief       Add a wrapper on the InvalidateIcache APIs so that
               TCM regions are ignored.
  \param[in]   addr    address
  \param[in]   isize   size of memory block (in number of bytes)
*/
__STATIC_FORCEINLINE
void RTSS_InvalidateICache_by_Addr(volatile void *addr, int32_t isize)
{
    if (RTSS_IsCacheInvalidate_Required_by_Addr(addr, isize)) {
        SCB_InvalidateICache_by_Addr(addr, isize);
    } else {
        __DSB();
        __ISB();
    }
}

/**
  \fn          void RTSS_InvalidateDCache_by_Addr (volatile void *addr, int32_t dsize)
  \brief       Add a wrapper on the InvalidateDcache APIs so that
               TCM regions are ignored.
  \note        Our internal benchmarks shows that if the requested size is more
               than the threshold size, it is better to invalidate the whole
               D-Cache instead of looping through the address.

               Notably, just invalidate is faster at 0.015ms, but we'd have to
               be sure there are no writeback cacheable areas.

               The breakeven point for ranged invalidate time = global clean+invalidate
               would be 43Kbyte. Considering the cost of refills, we decided to
               keep the threshold size to 128K.
  \param[in]   addr    address
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_FORCEINLINE
void RTSS_InvalidateDCache_by_Addr(volatile void *addr, int32_t dsize)
{
    if (RTSS_IsCacheInvalidate_Required_by_Addr(addr, dsize)) {
        /*
         * Considering the time required to Invalidate by address for
         * more than 128K size, it is better to do global clean and Invalidate.
         *
         * Perform the check for threshold size and decide.
         *
         */
        if (dsize < RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE) {
            SCB_InvalidateDCache_by_Addr(addr, dsize);
        } else {
            SCB_CleanInvalidateDCache();
        }
    } else {
        __DSB();
        __ISB();
    }
}

/**
  \fn          void RTSS_CleanDCache_by_Addr (volatile void *addr, int32_t dsize)
  \brief       Add a wrapper on the CleanDcache APIs so that
               TCM regions are ignored.
  \param[in]   addr    address
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_FORCEINLINE
void RTSS_CleanDCache_by_Addr(volatile void *addr, int32_t dsize)
{
    if (RTSS_IsCacheClean_Required_by_Addr(addr, dsize)) {
        /*
         * Considering the time required to Clean by address for more
         * than 128K size, it is better to do global clean.
         *
         * Perform the check for threshold size and decide.
         *
         */
        if (dsize < RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE) {
            SCB_CleanDCache_by_Addr(addr, dsize);
        } else {
            SCB_CleanDCache();
        }
    } else {
        __DSB();
        __ISB();
    }
}

/**
  \fn          void RTSS_CleanDCache (void)
  \brief       Clean the Cache only if the line is dirty.
*/
__STATIC_FORCEINLINE
void RTSS_CleanDCache(void)
{
    if (RTSS_IsGlobalCacheClean_Required() && RTSS_Is_DCache_Dirty()) {
        SCB_CleanDCache();
    }
}

/**
  \fn          void RTSS_CleanInvalidateDCache_by_Addr (volatile void *addr, int32_t dsize)
  \brief       Add a wrapper on the CleanInvalidateDcache APIs so that
               TCM regions are ignored.
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_FORCEINLINE
void RTSS_CleanInvalidateDCache_by_Addr(volatile void *addr, int32_t dsize)
{
    bool clean_req      = true;
    bool invalidate_req = true;

    clean_req           = RTSS_IsCacheClean_Required_by_Addr(addr, dsize);
    invalidate_req      = RTSS_IsCacheInvalidate_Required_by_Addr(addr, dsize);

    if (clean_req && invalidate_req) {
        /*
         * Considering the time required to CleanInvalidate by address
         * for more than 128K size, it is better to do global clean & Invalidate.
         *
         * Perform the check for threshold size and decide.
         *
         */
        if (dsize < RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE) {
            SCB_CleanInvalidateDCache_by_Addr(addr, dsize);
        } else {
            SCB_CleanInvalidateDCache();
        }
    } else if (clean_req) {
        if (dsize < RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE) {
            SCB_CleanDCache_by_Addr(addr, dsize);
        } else {
            SCB_CleanDCache();
        }
    } else if (invalidate_req) {
        if (dsize < RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE) {
            SCB_InvalidateDCache_by_Addr(addr, dsize);
        } else {
            SCB_CleanInvalidateDCache();
        }
    } else {
        __DSB();
        __ISB();
    }
}

#ifdef __cplusplus
}
#endif

#endif /* CACHE_H */
