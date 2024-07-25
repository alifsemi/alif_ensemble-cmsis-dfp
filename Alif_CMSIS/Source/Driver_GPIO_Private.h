/*
 * Copyright (c) 2024 ARM Limited. All rights reserved.
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
 *
 * $Date:        16. July 2024
 * $Revision:    V1.0
 *
 * Project:      GPIO Driver for Alif Semiconductor Ensemble
 */

#ifndef DRIVER_GPIO_PRIVATE_H_
#define DRIVER_GPIO_PRIVATE_H_

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_GPIO.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Pin mapping
#define GPIO_PIN(port, pin) ((8U * port) + (pin))

#define GPIO_P0(n)          (  0U + (n))
#define GPIO_P1(n)          (  8U + (n))
#define GPIO_P2(n)          ( 16U + (n))
#define GPIO_P3(n)          ( 24U + (n))
#define GPIO_P4(n)          ( 32U + (n))
#define GPIO_P5(n)          ( 40U + (n))
#define GPIO_P7(n)          ( 48U + (n))
#define GPIO_P8(n)          ( 56U + (n))
#define GPIO_P9(n)          ( 64U + (n))
#define GPIO_P10(n)         ( 72U + (n))
#define GPIO_P11(n)         ( 80U + (n))
#define GPIO_P12(n)         ( 88U + (n))
#define GPIO_P13(n)         (104U + (n))
#define GPIO_P15(n)         (112U + (n))

// GPIOx IRQ Handlers
extern void GPIO0_IRQ0Handler (void);
extern void GPIO0_IRQ1Handler (void);
extern void GPIO0_IRQ2Handler (void);
extern void GPIO0_IRQ3Handler (void);
extern void GPIO0_IRQ4Handler (void);
extern void GPIO0_IRQ5Handler (void);
extern void GPIO0_IRQ6Handler (void);
extern void GPIO0_IRQ7Handler (void);

extern void GPIO1_IRQ0Handler (void);
extern void GPIO1_IRQ1Handler (void);
extern void GPIO1_IRQ2Handler (void);
extern void GPIO1_IRQ3Handler (void);
extern void GPIO1_IRQ4Handler (void);
extern void GPIO1_IRQ5Handler (void);
extern void GPIO1_IRQ6Handler (void);
extern void GPIO1_IRQ7Handler (void);

extern void GPIO2_IRQ0Handler (void);
extern void GPIO2_IRQ1Handler (void);
extern void GPIO2_IRQ2Handler (void);
extern void GPIO2_IRQ3Handler (void);
extern void GPIO2_IRQ4Handler (void);
extern void GPIO2_IRQ5Handler (void);
extern void GPIO2_IRQ6Handler (void);
extern void GPIO2_IRQ7Handler (void);

extern void GPIO3_IRQ0Handler (void);
extern void GPIO3_IRQ1Handler (void);
extern void GPIO3_IRQ2Handler (void);
extern void GPIO3_IRQ3Handler (void);
extern void GPIO3_IRQ4Handler (void);
extern void GPIO3_IRQ5Handler (void);
extern void GPIO3_IRQ6Handler (void);
extern void GPIO3_IRQ7Handler (void);

extern void GPIO4_IRQ0Handler (void);
extern void GPIO4_IRQ1Handler (void);
extern void GPIO4_IRQ2Handler (void);
extern void GPIO4_IRQ3Handler (void);
extern void GPIO4_IRQ4Handler (void);
extern void GPIO4_IRQ5Handler (void);
extern void GPIO4_IRQ6Handler (void);
extern void GPIO4_IRQ7Handler (void);

extern void GPIO5_IRQ0Handler (void);
extern void GPIO5_IRQ1Handler (void);
extern void GPIO5_IRQ2Handler (void);
extern void GPIO5_IRQ3Handler (void);
extern void GPIO5_IRQ4Handler (void);
extern void GPIO5_IRQ5Handler (void);
extern void GPIO5_IRQ6Handler (void);
extern void GPIO5_IRQ7Handler (void);

extern void GPIO6_IRQ0Handler (void);
extern void GPIO6_IRQ1Handler (void);
extern void GPIO6_IRQ2Handler (void);
extern void GPIO6_IRQ3Handler (void);
extern void GPIO6_IRQ4Handler (void);
extern void GPIO6_IRQ5Handler (void);
extern void GPIO6_IRQ6Handler (void);
extern void GPIO6_IRQ7Handler (void);

extern void GPIO7_IRQ0Handler (void);
extern void GPIO7_IRQ1Handler (void);
extern void GPIO7_IRQ2Handler (void);
extern void GPIO7_IRQ3Handler (void);
extern void GPIO7_IRQ4Handler (void);
extern void GPIO7_IRQ5Handler (void);
extern void GPIO7_IRQ6Handler (void);
extern void GPIO7_IRQ7Handler (void);

extern void GPIO8_IRQ0Handler (void);
extern void GPIO8_IRQ1Handler (void);
extern void GPIO8_IRQ2Handler (void);
extern void GPIO8_IRQ3Handler (void);
extern void GPIO8_IRQ4Handler (void);
extern void GPIO8_IRQ5Handler (void);
extern void GPIO8_IRQ6Handler (void);
extern void GPIO8_IRQ7Handler (void);

extern void GPIO9_IRQ0Handler (void);
extern void GPIO9_IRQ1Handler (void);
extern void GPIO9_IRQ2Handler (void);
extern void GPIO9_IRQ3Handler (void);
extern void GPIO9_IRQ4Handler (void);
extern void GPIO9_IRQ5Handler (void);
extern void GPIO9_IRQ6Handler (void);
extern void GPIO9_IRQ7Handler (void);

extern void GPIO10_IRQ0Handler (void);
extern void GPIO10_IRQ1Handler (void);
extern void GPIO10_IRQ2Handler (void);
extern void GPIO10_IRQ3Handler (void);
extern void GPIO10_IRQ4Handler (void);
extern void GPIO10_IRQ5Handler (void);
extern void GPIO10_IRQ6Handler (void);
extern void GPIO10_IRQ7Handler (void);

extern void GPIO11_IRQ0Handler (void);
extern void GPIO11_IRQ1Handler (void);
extern void GPIO11_IRQ2Handler (void);
extern void GPIO11_IRQ3Handler (void);
extern void GPIO11_IRQ4Handler (void);
extern void GPIO11_IRQ5Handler (void);
extern void GPIO11_IRQ6Handler (void);
extern void GPIO11_IRQ7Handler (void);

extern void GPIO12_IRQ0Handler (void);
extern void GPIO12_IRQ1Handler (void);
extern void GPIO12_IRQ2Handler (void);
extern void GPIO12_IRQ3Handler (void);
extern void GPIO12_IRQ4Handler (void);
extern void GPIO12_IRQ5Handler (void);
extern void GPIO12_IRQ6Handler (void);
extern void GPIO12_IRQ7Handler (void);

extern void GPIO13_IRQ0Handler (void);
extern void GPIO13_IRQ1Handler (void);
extern void GPIO13_IRQ2Handler (void);
extern void GPIO13_IRQ3Handler (void);
extern void GPIO13_IRQ4Handler (void);
extern void GPIO13_IRQ5Handler (void);
extern void GPIO13_IRQ6Handler (void);
extern void GPIO13_IRQ7Handler (void);

extern void GPIO14_IRQ0Handler (void);
extern void GPIO14_IRQ1Handler (void);
extern void GPIO14_IRQ2Handler (void);
extern void GPIO14_IRQ3Handler (void);
extern void GPIO14_IRQ4Handler (void);
extern void GPIO14_IRQ5Handler (void);
extern void GPIO14_IRQ6Handler (void);
extern void GPIO14_IRQ7Handler (void);

extern void LPGPIO_IRQ0Handler (void);
extern void LPGPIO_IRQ1Handler (void);
extern void LPGPIO_IRQ2Handler (void);
extern void LPGPIO_IRQ3Handler (void);
extern void LPGPIO_IRQ4Handler (void);
extern void LPGPIO_IRQ5Handler (void);
extern void LPGPIO_IRQ6Handler (void);
extern void LPGPIO_IRQ7Handler (void);

// GPIO0 Driver access structure
extern ARM_DRIVER_GPIO Driver_GPIO0;

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_GPIO_PRIVATE_H_ */
