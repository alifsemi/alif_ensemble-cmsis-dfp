/*---------------------------------------------------------------------------
 * Copyright (c) 2025 Arm Limited (or its affiliates).
 * All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *---------------------------------------------------------------------------*/

#ifndef DEVKIT_E7_H_
#define DEVKIT_E7_H_

#include "Driver_GPIO_Private.h"
#include "Driver_ETH_MAC.h"
#include "Driver_I2C.h"
#include "Driver_SAI.h"
#include "Driver_SPI.h"
#include "Driver_USART.h"
#include "cmsis_vstream.h"

// DevKit-E7 mikroBUS pin definitions
#define MIKROBUS_AN         GPIO_PIN(0, 6)
#define MIKROBUS_INT        GPIO_PIN(8, 1)
#define MIKROBUS_PWM        GPIO_PIN(5, 6)

// CMSIS Driver instances on mikroBUS connector
#define MIKROBUS_I2C        0
#define MIKROBUS_SPI        3
#define MIKROBUS_UART       0

// CMSIS Driver instances of Board peripherals
#define CMSIS_DRIVER_ETH    0  // CMSIS Driver Ethernet MAC instance number
#define CMSIS_DRIVER_SAI    3  // CMSIS Driver SAI instance number
#define CMSIS_DRIVER_USART  4  // CMSIS Driver USART instance number

// Retarget stdio to CMSIS UART
#define RETARGET_STDIO_UART 4

// CMSIS Drivers
extern ARM_DRIVER_ETH_MAC Driver_ETH_MAC0;       /* Ethernet        */
extern ARM_DRIVER_I2C     Driver_I2C0;           /* mikroBUS I2C    */
extern ARM_DRIVER_SAI     Driver_SAI3;           /* I2S Microphones */
extern ARM_DRIVER_SPI     Driver_SPI3;           /* mikroBUS SPI    */
extern ARM_DRIVER_USART   Driver_USART0;         /* mikroBUS UART   */
extern ARM_DRIVER_USART   Driver_USART4;         /* PRG USB         */
extern vStreamDriver_t    Driver_vStreamAudioIn; /* Audio In Stream */

#ifdef CMSIS_shield_header
#include CMSIS_shield_header
#endif

#endif /* DEVKIT_E7_H_ */
