/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef AE1C1F4051920PH_H
#define AE1C1F4051920PH_H


/* MRAM */
#define DEVICE_FEATURE_MRAM_SIZE 0x00180000    /* 5.5 MB */

/* SRAM */
#define DEVICE_FEATURE_SRAM0_PRESENT 0
#define DEVICE_FEATURE_SRAM0_SIZE 0x00400000           /* 4 MB */
#define DEVICE_FEATURE_SRAM1_PRESENT 0
#define DEVICE_FEATURE_SRAM1_SIZE 0x00280000           /* 2.5 MB */
#define DEVICE_FEATURE_SRAM2_PRESENT 0
#define DEVICE_FEATURE_SRAM2_SIZE 0x00040000           /* 256 KB */
#define DEVICE_FEATURE_SRAM3_PRESENT 0
#define DEVICE_FEATURE_SRAM3_SIZE 0x00100000           /* 1 MB */
#define DEVICE_FEATURE_SRAM4_PRESENT 1
#define DEVICE_FEATURE_SRAM4_SIZE 0x00040000           /* 256 KB */
#define DEVICE_FEATURE_SRAM5_PRESENT 1
#define DEVICE_FEATURE_SRAM5_SIZE 0x00040000           /* 256 MB */
#define DEVICE_FEATURE_SRAM6_A_PRESENT 0
#define DEVICE_FEATURE_SRAM6_A_SIZE 0x00100000           /* 1 MB */
#define DEVICE_FEATURE_SRAM6_B_PRESENT 0
#define DEVICE_FEATURE_SRAM6_B_SIZE 0x00100000           /* 1 MB */
#define DEVICE_FEATURE_SRAM7_PRESENT 0
#define DEVICE_FEATURE_SRAM7_SIZE 0x00080000           /* 512 KB */
#define DEVICE_FEATURE_SRAM8_PRESENT 0
#define DEVICE_FEATURE_SRAM8_SIZE 0x00200000           /* 2 MB */
#define DEVICE_FEATURE_SRAM9_A_PRESENT 0
#define DEVICE_FEATURE_SRAM9_A_SIZE 0x00040000           /* 256 KB */
#define DEVICE_FEATURE_SRAM9_B_PRESENT 0
#define DEVICE_FEATURE_SRAM9_B_SIZE 0x00080000           /* 512 KB */

#include "soc_E1C_feature.h"

#endif /* AE1C1F4051920PH_H */
