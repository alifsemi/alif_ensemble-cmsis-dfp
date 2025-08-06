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
 * @file     Driver_SPI_EX.h
 * @author   Manoj A Murudi
 * @email    manoj,murudi@alifsemi.com
 * @version  V1.0.0
 * @date     13-Jan-2025
 * @brief    Extended Header for SPI Driver
 * @bug      None
 * @Note     None
 ******************************************************************************/

#ifndef Driver_SPI_EX_H_
#define Driver_SPI_EX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Driver_SPI.h"

/****** SPI Extended Control Codes *****/
#define ARM_SPI_USE_CUSTOM_DMA_MCODE_TX                                                            \
    (0x20UL << ARM_SPI_CONTROL_Pos)  ///< Use User defined DMA microcode arg provides address
#define ARM_SPI_USE_CUSTOM_DMA_MCODE_RX                                                            \
    (0x21UL << ARM_SPI_CONTROL_Pos)  ///< Use User defined DMA microcode arg provides address

#ifdef __cplusplus
}
#endif

#endif /* Driver_SPI_EX_H_ */
