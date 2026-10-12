/*
 * Copyright 2026 Arm Limited and/or its affiliates.
 *
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
 */

#ifndef APPKIT_E8_AIML_H_
#define APPKIT_E8_AIML_H_

#include "Driver_GPIO_Private.h"
#include "Driver_ETH_MAC.h"
#include "Driver_USART.h"
#include "Driver_USBD.h"
#include "cmsis_vstream.h"

// CMSIS Driver instances of Board peripherals
#define CMSIS_DRIVER_ETH     0  // CMSIS Driver Ethernet MAC instance number
#define CMSIS_DRIVER_USART   4  // CMSIS Driver USART instance number
#define CMSIS_DRIVER_USBD    0  // CMSIS Driver USB Device instance number
#define CMSIS_DRIVER_USBD_HS 1  // CMSIS Driver USB Device high-speed support

// Retarget stdio to CMSIS UART
#define RETARGET_STDIO_UART  4

// CMSIS Drivers
extern ARM_DRIVER_ETH_MAC Driver_ETH_MAC0;        /* Ethernet         */
extern ARM_DRIVER_USART   Driver_USART4;          /* PRG USB          */
extern ARM_DRIVER_USBD    Driver_USBD0;           /* USB Device       */
extern vStreamDriver_t    Driver_vStreamVideoIn;  /* Video In Stream  */
extern vStreamDriver_t    Driver_vStreamVideoOut; /* Video Out Stream */

#ifdef CMSIS_shield_header
#include CMSIS_shield_header
#endif

#endif /* APPKIT_E8_AIML_H_ */
