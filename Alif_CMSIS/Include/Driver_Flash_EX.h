/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DRIVER_FLASH_EX_H_
#define DRIVER_FLASH_EX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Driver_Flash.h"

#define _ARM_Driver_Flash_EX_(n)      Driver_Flash_EX##n
#define  ARM_Driver_Flash_EX_(n) _ARM_Driver_Flash_EX_(n)

/**
\brief Flash Wrap Mode
*/
typedef enum _ARM_FLASH_WRAP_MODE {
    ARM_FLASH_WRAP_MODE_CONTINUOUS = 0,   ///< No wrap mode
    ARM_FLASH_WRAP_MODE_16BYTE,  ///< 16-byte wrap mode
    ARM_FLASH_WRAP_MODE_32BYTE,  ///< 32-byte wrap mode
    ARM_FLASH_WRAP_MODE_64BYTE   ///< 64-byte wrap mode
} ARM_FLASH_WRAP_MODE;


/**
\brief Access structure of the Flash Driver Alif extension
*/
typedef struct _ARM_DRIVER_FLASH_EX {
    int32_t                (*SetWaitCycles) (uint32_t cycles);
    int32_t                (*SetWrapMode)   (ARM_FLASH_WRAP_MODE mode);
} const ARM_DRIVER_FLASH_EX;

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_FLASH_EX_H_ */
