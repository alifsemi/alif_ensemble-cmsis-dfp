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
 * @file     sau_tcm_ns_setup.h
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @version  V1.0.0
 * @date     6-August-2024
 * @brief    Set TCM Nonsecure Partitioning in SAU and TGU
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef SAU_TCM_NS_SETUP_H
#define SAU_TCM_NS_SETUP_H

#include "core_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__ARMCC_VERSION)
extern const uint32_t Image$$NS_REGION_0$$Base;
extern const uint32_t Image$$NS_REGION_0_PAD$$Base;
static const uint32_t ns_region_0_start __attribute__((weakref("Image$$NS_REGION_0$$Base")));
static const uint32_t ns_region_0_end __attribute__((weakref("Image$$NS_REGION_0_PAD$$Base")));
#elif defined(__GNUC__)
extern const uint32_t __ns_region_0_start;
extern const uint32_t __ns_region_0_end;
static const uint32_t ns_region_0_start __attribute__((weakref("__ns_region_0_start")));
static const uint32_t ns_region_0_end __attribute__((weakref("__ns_region_0_end")));
#else
#error Unknown compiler.
#endif

/*
 * sau_tcm_ns_setup ()
 * Set up the TCM Nonsecure partitioning in SAU and TGU
 */
void sau_tcm_ns_setup(void);

#ifdef __cplusplus
}
#endif

#endif /* SAU_TCM_NS_SETUP_H */
