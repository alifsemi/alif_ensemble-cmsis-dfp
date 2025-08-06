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
 * @file     Driver_USART_EX.h
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     15-Jan-2025
 * @brief    Extended Header for USART Driver
 * @bug      None
 * @Note     None
 ******************************************************************************/

#ifndef DRIVER_USART_EX_H_
#define DRIVER_USART_EX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Driver_USART.h"

/****** USART Extended Control Codes *****/
#define ARM_USART_USE_CUSTOM_DMA_MCODE_TX                                                          \
    (0x1BUL << ARM_USART_CONTROL_Pos)  ///< Use User defined DMA microcode arg provides address
#define ARM_USART_USE_CUSTOM_DMA_MCODE_RX                                                          \
    (0x1CUL << ARM_USART_CONTROL_Pos)  ///< Use User defined DMA microcode arg provides address

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_USART_EX_H_ */
