/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/*
 * Copyright (c) 2020 Arm Limited. All rights reserved.
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

/******************************************************************************
 * @file     M55_HE.h
 * @author   Rupesh Kumar
 * @email    rupesh@alifsemi.com
 * @brief    CMSIS Core Peripheral Access Layer Header File for
 *           M55_HE Device (configured for ARMCM55 with
 *           Integer and half and single precision,
 *           DSP extension, MVE, TrustZone)
 * @version  V1.0.0
 * @date     23. Feb 2021
 * @bug      None
 * @Note	 None
 ******************************************************************************/

#ifndef M55_HE_H
#define M55_HE_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------  Aliases for Local Resources  ------------------------ */
/* ----------  Local DMA IRQ Handler Aliases  ---------- */
#define DMALOCAL_IRQ0Handler        DMA2_IRQ0Handler
#define DMALOCAL_IRQ1Handler        DMA2_IRQ1Handler
#define DMALOCAL_IRQ2Handler        DMA2_IRQ2Handler
#define DMALOCAL_IRQ3Handler        DMA2_IRQ3Handler
#define DMALOCAL_IRQ4Handler        DMA2_IRQ4Handler
#define DMALOCAL_IRQ5Handler        DMA2_IRQ5Handler
#define DMALOCAL_IRQ6Handler        DMA2_IRQ6Handler
#define DMALOCAL_IRQ7Handler        DMA2_IRQ7Handler
#define DMALOCAL_IRQ8Handler        DMA2_IRQ8Handler
#define DMALOCAL_IRQ9Handler        DMA2_IRQ9Handler
#define DMALOCAL_IRQ10Handler       DMA2_IRQ10Handler
#define DMALOCAL_IRQ11Handler       DMA2_IRQ11Handler
#define DMALOCAL_IRQ12Handler       DMA2_IRQ12Handler
#define DMALOCAL_IRQ13Handler       DMA2_IRQ13Handler
#define DMALOCAL_IRQ14Handler       DMA2_IRQ14Handler
#define DMALOCAL_IRQ15Handler       DMA2_IRQ15Handler
#define DMALOCAL_IRQ16Handler       DMA2_IRQ16Handler
#define DMALOCAL_IRQ17Handler       DMA2_IRQ17Handler
#define DMALOCAL_IRQ18Handler       DMA2_IRQ18Handler
#define DMALOCAL_IRQ19Handler       DMA2_IRQ19Handler
#define DMALOCAL_IRQ20Handler       DMA2_IRQ20Handler
#define DMALOCAL_IRQ21Handler       DMA2_IRQ21Handler
#define DMALOCAL_IRQ22Handler       DMA2_IRQ22Handler
#define DMALOCAL_IRQ23Handler       DMA2_IRQ23Handler
#define DMALOCAL_IRQ24Handler       DMA2_IRQ24Handler
#define DMALOCAL_IRQ25Handler       DMA2_IRQ25Handler
#define DMALOCAL_IRQ26Handler       DMA2_IRQ26Handler
#define DMALOCAL_IRQ27Handler       DMA2_IRQ27Handler
#define DMALOCAL_IRQ28Handler       DMA2_IRQ28Handler
#define DMALOCAL_IRQ29Handler       DMA2_IRQ29Handler
#define DMALOCAL_IRQ30Handler       DMA2_IRQ30Handler
#define DMALOCAL_IRQ31Handler       DMA2_IRQ31Handler
#define DMALOCAL_IRQ_ABORT_Handler  DMA2_IRQ_ABORT_Handler

/* ----------  Local DMA IRQ Number Aliases  ---------- */
#define DMALOCAL_IRQ0_IRQn          DMA2_IRQ0_IRQn

/* ----------  Local MHU IRQ Handler Aliases  ---------- */
#define MHU_SESS_S_RX_IRQHandler    MHU_SECPU_M55HE_0_RX_IRQHandler
#define MHU_SESS_S_TX_IRQHandler    MHU_M55HE_SECPU_0_TX_IRQHandler
#define MHU_SESS_NS_RX_IRQHandler   MHU_SECPU_M55HE_1_RX_IRQHandler
#define MHU_SESS_NS_TX_IRQHandler   MHU_M55HE_SECPU_1_TX_IRQHandler

/* ----------  Local MHU IRQ Number Aliases  ---------- */
#define MHU_SESS_S_RX_IRQ_IRQn      MHU_SECPU_M55HE_0_RX_IRQ_IRQn
#define MHU_SESS_S_TX_IRQ_IRQn      MHU_M55HE_SECPU_0_TX_IRQ_IRQn
#define MHU_SESS_NS_RX_IRQ_IRQn     MHU_SECPU_M55HE_1_RX_IRQ_IRQn
#define MHU_SESS_NS_TX_IRQ_IRQn     MHU_M55HE_SECPU_1_TX_IRQ_IRQn

/* ----------  Local NPU IRQ Handler Aliases  ---------- */
#define LOCAL_NPU_IRQHandler        NPU_HE_IRQHandler

/* ----------  Local NPU IRQ Number Aliases  ---------- */
#define LOCAL_NPU_IRQ_IRQn          NPU_HE_IRQ_IRQn

/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* =======================================  ARM Cortex-M55 Specific Interrupt Numbers  ======================================= */
  Reset_IRQn                = -15,              /*!< -15  Reset Vector, invoked on Power up and warm reset                     */
  NonMaskableInt_IRQn       = -14,              /*!< -14  Non maskable Interrupt, cannot be stopped or preempted               */
  HardFault_IRQn            = -13,              /*!< -13  Hard Fault, all classes of Fault                                     */
  MemoryManagement_IRQn     = -12,              /*!< -12  Memory Management, MPU mismatch, including Access Violation
                                                     and No Match                                                              */
  BusFault_IRQn             = -11,              /*!< -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                     related Fault                                                             */
  UsageFault_IRQn           = -10,              /*!< -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition        */
  SecureFault_IRQn          =  -9,              /*!< -9 Secure Fault Handler                                                   */
  SVCall_IRQn               =  -5,              /*!< -5 System Service Call via SVC instruction                                */
  DebugMonitor_IRQn         =  -4,              /*!< -4 Debug Monitor                                                          */
  PendSV_IRQn               =  -2,              /*!< -2 Pendable request for system service                                    */
  SysTick_IRQn		    =  -1,
/* ======================================  AE722F80F55D5AE Specific Interrupt Numbers  ======================================= */
  DMA2_IRQ0_IRQn            =   0,              /*!< 0  Interrupt request 0. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ1_IRQn            =   1,              /*!< 1  Interrupt request 1. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ2_IRQn            =   2,              /*!< 2  Interrupt request 2. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ3_IRQn            =   3,              /*!< 3  Interrupt request 3. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ4_IRQn            =   4,              /*!< 4  Interrupt request 4. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ5_IRQn            =   5,              /*!< 5  Interrupt request 5. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ6_IRQn            =   6,              /*!< 6  Interrupt request 6. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ7_IRQn            =   7,              /*!< 7  Interrupt request 7. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ8_IRQn            =   8,              /*!< 8  Interrupt request 8. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ9_IRQn            =   9,              /*!< 9  Interrupt request 9. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ10_IRQn           =  10,              /*!< 10 Interrupt request 10. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ11_IRQn           =  11,              /*!< 11 Interrupt request 11. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ12_IRQn           =  12,              /*!< 12 Interrupt request 12. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ13_IRQn           =  13,              /*!< 13 Interrupt request 13. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ14_IRQn           =  14,              /*!< 14 Interrupt request 14. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ15_IRQn           =  15,              /*!< 15 Interrupt request 15. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ16_IRQn           =  16,              /*!< 16 Interrupt request 16. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ17_IRQn           =  17,              /*!< 17 Interrupt request 17. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ18_IRQn           =  18,              /*!< 18 Interrupt request 18. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ19_IRQn           =  19,              /*!< 19 Interrupt request 19. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ20_IRQn           =  20,              /*!< 20 Interrupt request 20. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ21_IRQn           =  21,              /*!< 21 Interrupt request 21. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ22_IRQn           =  22,              /*!< 22 Interrupt request 22. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ23_IRQn           =  23,              /*!< 23 Interrupt request 23. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ24_IRQn           =  24,              /*!< 24 Interrupt request 24. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ25_IRQn           =  25,              /*!< 25 Interrupt request 25. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ26_IRQn           =  26,              /*!< 26 Interrupt request 26. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ27_IRQn           =  27,              /*!< 27 Interrupt request 27. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ28_IRQn           =  28,              /*!< 28 Interrupt request 28. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ29_IRQn           =  29,              /*!< 29 Interrupt request 29. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ30_IRQn           =  30,              /*!< 30 Interrupt request 30. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ31_IRQn           =  31,              /*!< 31 Interrupt request 31. One per DMA request interface (32 in
                                                     total).                                                                   */
  DMA2_IRQ_ABORT_IRQn       =  32,              /*!< 32 DMAC aborted execution of a program thread.                            */
  MHU_SECPU_M55HE_0_RX_IRQ_IRQn=  37,           /*!< 37 Interrupt                                                              */
  MHU_M55HE_SECPU_0_TX_IRQ_IRQn=  38,           /*!< 38 Interrupt                                                              */
  MHU_SECPU_M55HE_1_RX_IRQ_IRQn=  39,           /*!< 39 Interrupt                                                              */
  MHU_M55HE_SECPU_1_TX_IRQ_IRQn=  40,           /*!< 40 Interrupt                                                              */
  LPUART_IRQ_IRQn           =  45,
  LPSPI_IRQ_IRQn                 =  46,
  LPI2C_IRQ_IRQn                 =  47,
  LPI2S_IRQ_IRQn                 =  48,
  LPPDM_IRQ_IRQn                 =  49,
  HCI_UART_IRQ_IRQn              =  50,
  AHI_UART_IRQ_IRQn              =  51,
  LPCPI_IRQ_IRQn                 =  54,
  NPU_HE_IRQ_IRQn                =  55,
  LPCMP_IRQ_IRQn                 =  56,
  LPGPIO_COMB_IRQ_IRQn           =  57,              /*!< 57 LPGPIO combined interrupt request                                      */
  LPRTC0_IRQ_IRQn                =  58,
  LPRTC1_IRQ_IRQn                =  59,
  LPTIMER0_IRQ_IRQn              =  60,
  LPTIMER1_IRQ_IRQn              =  61,
  LPTIMER2_IRQ_IRQn              =  62,
  LPTIMER3_IRQ_IRQn              =  63,
  REFCLK_CNTBASE0_IRQ_IRQn       =  67,
  REFCLK_CNTBASE1_IRQ_IRQn       =  68,
  REFCLK_CNTBASE2_IRQ_IRQn       =  69,
  REFCLK_CNTBASE3_IRQ_IRQn       =  70,
  S32K_CNTBASE0_IRQ_IRQn         =  71,
  S32K_CNTBASE1_IRQ_IRQn         =  72,
  SOC_ETR_IRQ_IRQn               =  73,
  SOC_CATU_IRQ_IRQn              =  74,
  OSPI0_IRQ_IRQn                 =  96,              /*!< 96 Combined interrupt request from OSPI0 routed to the interrupt
                                                     controllers in the device. For more information on the
                                                     internal interrupt requests, see Section OSPI Interrupt
                                                     Requests.                                                                 */
  AES0_IRQ_IRQn                  =  98,              /*!< 98 Combined interrupt request from AES0 routed to the interrupt
                                                     controllers in the device. For more information on the
                                                     internal interrupt requests, see Section OSPI Interrupt
                                                     Requests.                                                                 */
  BOD_IRQ_IRQn              = 100,              /*!< 100  BOD interrupt                                                        */
  USB_IRQ_IRQn              = 101,              /*!< 101  USB interrupt                                                        */
  SDMMC_IRQ_IRQn            = 102,              /*!< 102  SDMMC Interrupt request                                              */
  SDMMC_WAKEUP_IRQ_IRQn     = 103,              /*!< 103  SDMMC Wakeup event interrupt request                                 */
  CANFD0_IRQ_IRQn           = 104,              /*!< 104  CANFD Interrupt request                                              */
  CANFD1_IRQ_IRQn           = 105,              /*!< 105  HWSEM interrupt request 0                                            */
  PPU0_IRQ_IRQn             = 121,              /*!< 121  PPU0 interrupt request                                               */
  PPU1_IRQ_IRQn             = 122,              /*!< 122  PPU1 interrupt request                                               */
  UART0_IRQ_IRQn            = 124,              /*!< 124  UART0 interrupt request                                              */
  UART1_IRQ_IRQn            = 125,              /*!< 125  UART1 interrupt request                                              */
  UART2_IRQ_IRQn            = 126,              /*!< 126  UART2 interrupt request                                              */
  UART3_IRQ_IRQn            = 127,              /*!< 127  UART3 interrupt request                                              */
  UART4_IRQ_IRQn            = 128,              /*!< 128  UART4 interrupt request                                              */
  UART5_IRQ_IRQn            = 129,              /*!< 129  UART5 interrupt request                                              */
  I2C0_IRQ_IRQn             = 132,              /*!< 132  I2C0 interrupt request                                               */
  I2C1_IRQ_IRQn             = 133,              /*!< 133  I2C1 interrupt request                                               */
  I3C_IRQ_IRQn              = 136,              /*!< 136  I3C interrupt request                                                */
  SPI0_IRQ_IRQn             = 137,              /*!< 137  SPI0 interrupt request                                               */
  SPI1_IRQ_IRQn             = 138,              /*!< 138  SPI1 interrupt request                                               */
  SPI2_IRQ_IRQn             = 139,              /*!< 139  SPI2 interrupt request                                               */
  I2S0_IRQ_IRQn             = 141,              /*!< 141  I2S0 interrupt request                                               */
  I2S1_IRQ_IRQn             = 142,              /*!< 142  I2S1 interrupt request                                               */
  ADC120_DONE0_IRQ_IRQn     = 151,              /*!< 151  Sample ready                                                         */
  ADC121_DONE0_IRQ_IRQn     = 152,              /*!< 152  Sample ready                                                         */
  ADC120_DONE1_IRQ_IRQn     = 154,              /*!< 154  Averaged samples ready                                               */
  ADC121_DONE1_IRQ_IRQn     = 155,              /*!< 155  Averaged samples ready                                               */
  ADC120_CMPA_IRQ_IRQn      = 157,              /*!< 157  Comparator A interrupt                                               */
  ADC121_CMPA_IRQ_IRQn      = 158,              /*!< 158  Comparator A interrupt                                               */
  ADC120_CMPB_IRQ_IRQn      = 160,              /*!< 160  Comparator B interrupt                                               */
  ADC121_CMPB_IRQ_IRQn      = 161,              /*!< 161  Comparator B interrupt                                               */
  ADC24_DONE0_IRQ_IRQn      = 163,              /*!< 163  Sample ready                                                         */
  ADC24_DONE1_IRQ_IRQn      = 164,              /*!< 164  Averaged samples ready                                               */
  ADC24_CMPA_IRQ_IRQn       = 165,              /*!< 165  Comparator A interrupt                                               */
  ADC24_CMPB_IRQ_IRQn       = 166,              /*!< 166  Comparator B interrupt                                               */
  CMP0_IRQ_IRQn             = 167,              /*!< 167  Comparator result changed                                            */
  CMP1_IRQ_IRQn             = 168,              /*!< 168  Comparator result changed                                            */
  LPGPIO_IRQ0_IRQn          = 171,              /*!< 171  LPGPIO interrupt request 0                                           */
  LPGPIO_IRQ1_IRQn          = 172,              /*!< 172  LPGPIO interrupt request 1                                           */
  GPIO0_IRQ0_IRQn           = 179,              /*!< 179  GPIO0 interrupt request 0                                            */
  GPIO0_IRQ1_IRQn           = 180,              /*!< 180  GPIO0 interrupt request 1                                            */
  GPIO0_IRQ2_IRQn           = 181,              /*!< 181  GPIO0 interrupt request 2                                            */
  GPIO0_IRQ3_IRQn           = 182,              /*!< 182  GPIO0 interrupt request 3                                            */
  GPIO0_IRQ4_IRQn           = 183,              /*!< 183  GPIO0 interrupt request 4                                            */
  GPIO0_IRQ5_IRQn           = 184,              /*!< 184  GPIO0 interrupt request 5                                            */
  GPIO0_IRQ6_IRQn           = 185,              /*!< 185  GPIO0 interrupt request 6                                            */
  GPIO0_IRQ7_IRQn           = 186,              /*!< 186  GPIO0 interrupt request 7                                            */
  GPIO1_IRQ0_IRQn           = 187,              /*!< 187  GPIO1 interrupt request 0                                            */
  GPIO1_IRQ1_IRQn           = 188,              /*!< 188  GPIO1 interrupt request 1                                            */
  GPIO1_IRQ2_IRQn           = 189,              /*!< 189  GPIO1 interrupt request 2                                            */
  GPIO1_IRQ3_IRQn           = 190,              /*!< 190  GPIO1 interrupt request 3                                            */
  GPIO1_IRQ4_IRQn           = 191,              /*!< 191  GPIO1 interrupt request 4                                            */
  GPIO1_IRQ5_IRQn           = 192,              /*!< 192  GPIO1 interrupt request 5                                            */
  GPIO1_IRQ6_IRQn           = 193,              /*!< 193  GPIO1 interrupt request 6                                            */
  GPIO1_IRQ7_IRQn           = 194,              /*!< 194  GPIO1 interrupt request 7                                            */
  GPIO2_IRQ0_IRQn           = 195,              /*!< 195  GPIO2 interrupt request 0                                            */
  GPIO2_IRQ1_IRQn           = 196,              /*!< 196  GPIO2 interrupt request 1                                            */
  GPIO2_IRQ2_IRQn           = 197,              /*!< 197  GPIO2 interrupt request 2                                            */
  GPIO2_IRQ3_IRQn           = 198,              /*!< 198  GPIO2 interrupt request 3                                            */
  GPIO2_IRQ4_IRQn           = 199,              /*!< 199  GPIO2 interrupt request 4                                            */
  GPIO2_IRQ5_IRQn           = 200,              /*!< 200  GPIO2 interrupt request 5                                            */
  GPIO2_IRQ6_IRQn           = 201,              /*!< 201  GPIO2 interrupt request 6                                            */
  GPIO2_IRQ7_IRQn           = 202,              /*!< 202  GPIO2 interrupt request 7                                            */
  GPIO3_IRQ0_IRQn           = 203,              /*!< 203  GPIO3 interrupt request 0                                            */
  GPIO3_IRQ1_IRQn           = 204,              /*!< 204  GPIO3 interrupt request 1                                            */
  GPIO3_IRQ2_IRQn           = 205,              /*!< 205  GPIO3 interrupt request 2                                            */
  GPIO3_IRQ3_IRQn           = 206,              /*!< 206  GPIO3 interrupt request 3                                            */
  GPIO3_IRQ4_IRQn           = 207,              /*!< 207  GPIO3 interrupt request 4                                            */
  GPIO3_IRQ5_IRQn           = 208,              /*!< 208  GPIO3 interrupt request 5                                            */
  GPIO3_IRQ6_IRQn           = 209,              /*!< 209  GPIO3 interrupt request 6                                            */
  GPIO3_IRQ7_IRQn           = 210,              /*!< 210  GPIO3 interrupt request 7                                            */
  GPIO4_IRQ0_IRQn           = 211,              /*!< 211  GPIO4 interrupt request 0                                            */
  GPIO4_IRQ1_IRQn           = 212,              /*!< 212  GPIO4 interrupt request 1                                            */
  GPIO4_IRQ2_IRQn           = 213,              /*!< 213  GPIO4 interrupt request 2                                            */
  GPIO4_IRQ3_IRQn           = 214,              /*!< 214  GPIO4 interrupt request 3                                            */
  GPIO4_IRQ4_IRQn           = 215,              /*!< 215  GPIO4 interrupt request 4                                            */
  GPIO4_IRQ5_IRQn           = 216,              /*!< 216  GPIO4 interrupt request 5                                            */
  GPIO4_IRQ6_IRQn           = 217,              /*!< 217  GPIO4 interrupt request 6                                            */
  GPIO4_IRQ7_IRQn           = 218,              /*!< 218  GPIO4 interrupt request 7                                            */
  GPIO5_IRQ0_IRQn           = 219,              /*!< 219  GPIO5 interrupt request 0                                            */
  GPIO5_IRQ1_IRQn           = 220,              /*!< 220  GPIO5 interrupt request 1                                            */
  GPIO5_IRQ2_IRQn           = 221,              /*!< 221  GPIO5 interrupt request 2                                            */
  GPIO5_IRQ3_IRQn           = 222,              /*!< 222  GPIO5 interrupt request 3                                            */
  GPIO5_IRQ4_IRQn           = 223,              /*!< 223  GPIO5 interrupt request 4                                            */
  GPIO5_IRQ5_IRQn           = 224,              /*!< 224  GPIO5 interrupt request 5                                            */
  GPIO5_IRQ6_IRQn           = 225,              /*!< 225  GPIO5 interrupt request 6                                            */
  GPIO5_IRQ7_IRQn           = 226,              /*!< 226  GPIO5 interrupt request 7                                            */
  GPIO6_IRQ0_IRQn           = 227,              /*!< 227  GPIO6 interrupt request 0                                            */
  GPIO6_IRQ1_IRQn           = 228,              /*!< 228  GPIO6 interrupt request 1                                            */
  GPIO6_IRQ2_IRQn           = 229,              /*!< 229  GPIO6 interrupt request 2                                            */
  GPIO6_IRQ3_IRQn           = 230,              /*!< 230  GPIO6 interrupt request 3                                            */
  GPIO6_IRQ4_IRQn           = 231,              /*!< 231  GPIO6 interrupt request 4                                            */
  GPIO6_IRQ5_IRQn           = 232,              /*!< 232  GPIO6 interrupt request 5                                            */
  GPIO6_IRQ6_IRQn           = 233,              /*!< 233  GPIO6 interrupt request 6                                            */
  GPIO6_IRQ7_IRQn           = 234,              /*!< 234  GPIO6 interrupt request 7                                            */
  GPIO7_IRQ0_IRQn           = 235,              /*!< 235  GPIO7 interrupt request 0                                            */
  GPIO7_IRQ1_IRQn           = 236,              /*!< 236  GPIO7 interrupt request 1                                            */
  GPIO7_IRQ2_IRQn           = 237,              /*!< 237  GPIO7 interrupt request 2                                            */
  GPIO7_IRQ3_IRQn           = 238,              /*!< 238  GPIO7 interrupt request 3                                            */
  GPIO7_IRQ4_IRQn           = 239,              /*!< 239  GPIO7 interrupt request 4                                            */
  GPIO7_IRQ5_IRQn           = 240,              /*!< 240  GPIO7 interrupt request 5                                            */
  GPIO7_IRQ6_IRQn           = 241,              /*!< 241  GPIO7 interrupt request 6                                            */
  GPIO7_IRQ7_IRQn           = 242,              /*!< 242  GPIO7 interrupt request 7                                            */
  GPIO8_IRQ0_IRQn           = 243,              /*!< 243  GPIO8 interrupt request 0                                            */
  GPIO8_IRQ1_IRQn           = 244,              /*!< 244  GPIO8 interrupt request 1                                            */
  GPIO8_IRQ2_IRQn           = 245,              /*!< 245  GPIO8 interrupt request 2                                            */
  GPIO8_IRQ3_IRQn           = 246,              /*!< 246  GPIO8 interrupt request 3                                            */
  GPIO8_IRQ4_IRQn           = 247,              /*!< 247  GPIO8 interrupt request 4                                            */
  GPIO8_IRQ5_IRQn           = 248,              /*!< 248  GPIO8 interrupt request 5                                            */
  GPIO8_IRQ6_IRQn           = 249,              /*!< 249  GPIO8 interrupt request 6                                            */
  GPIO8_IRQ7_IRQn           = 250,              /*!< 250  GPIO8 interrupt request 7                                            */
  GPIO9_IRQ0_IRQn           = 251,              /*!< 251  GPIO9 interrupt request 0                                            */
  GPIO9_IRQ1_IRQn           = 252,              /*!< 252  GPIO9 interrupt request 1                                            */
  GPIO9_IRQ2_IRQn           = 253,              /*!< 253  GPIO9 interrupt request 2                                            */
  GPIO9_IRQ3_IRQn           = 254,              /*!< 254  GPIO9 interrupt request 3                                            */
  GPIO9_IRQ4_IRQn           = 255,              /*!< 255  GPIO9 interrupt request 4                                            */
  GPIO9_IRQ5_IRQn           = 256,              /*!< 256  GPIO9 interrupt request 5                                            */
  GPIO9_IRQ6_IRQn           = 257,              /*!< 257  GPIO9 interrupt request 6                                            */
  GPIO9_IRQ7_IRQn           = 258,              /*!< 258  GPIO9 interrupt request 7                                            */
  GPU2D_IRQ_IRQn            = 332,              /*!< 332  GPU2D aggregated interrupt request                                   */
  CDC_SCANLINE0_IRQ_IRQn    = 333,              /*!< 333  CDC line interrupt 0                                                 */
  CDC_SCANLINE1_IRQ_IRQn    = 334,              /*!< 334  CDC line interrupt 1                                                 */
  CDC_FIFO_WARNING0_IRQ_IRQn= 335,              /*!< 335  CDC FIFO underrun warning interrupt 0                                */
  CDC_FIFO_WARNING1_IRQ_IRQn= 336,              /*!< 336  CDC FIFO underrun warning interrupt 1                                */
  CDC_FIFO_UNDERRUN0_IRQ_IRQn= 337,             /*!< 337  CDC layer FIFO underrun interrupt 0                                  */
  CDC_FIFO_UNDERRUN1_IRQ_IRQn= 338,             /*!< 338  CDC layer FIFO underrun interrupt 1                                  */
  CDC_BUS_ERROR0_IRQ_IRQn   = 339,              /*!< 339  CDC bus error interrupt 0                                            */
  CDC_BUS_ERROR1_IRQ_IRQn   = 340,              /*!< 340  CDC bus error interrupt 1                                            */
  CDC_REG_RELOAD0_IRQ_IRQn  = 341,              /*!< 341  CDC register reload interrupt 0                                      */
  CDC_REG_RELOAD1_IRQ_IRQn  = 342,              /*!< 342  CDC register reload interrupt 1                                      */
  DSI_IRQ_IRQn              = 343,              /*!< 343  DSI error interrupt                                                  */
//  LPTIMER0_IRQ_IRQn         = 346,              /*!< 346  LPTIMER0 interrupt request                                           */
//  LPTIMER1_IRQ_IRQn         = 347,              /*!< 347  LPTIMER1 interrupt request                                           */
//  LPTIMER2_IRQ_IRQn         = 348,              /*!< 348  LPTIMER2 interrupt request                                           */
//  LPTIMER3_IRQ_IRQn         = 349,              /*!< 349  LPTIMER3 interrupt request                                           */
//  LPRTC_IRQ_IRQn            = 350,              /*!< 350  LPRTC interrupt request                                              */
  UTIMER_IRQ0_IRQn          = 377,              /*!< 377  Channel 0, interrupt request 0                                       */
  UTIMER_IRQ1_IRQn          = 378,              /*!< 378  Channel 0, interrupt request 1                                       */
  UTIMER_IRQ2_IRQn          = 379,              /*!< 379  Channel 0, interrupt request 2                                       */
  UTIMER_IRQ3_IRQn          = 380,              /*!< 380  Channel 0, interrupt request 3                                       */
  UTIMER_IRQ4_IRQn          = 381,              /*!< 381  Channel 0, interrupt request 4                                       */
  UTIMER_IRQ5_IRQn          = 382,              /*!< 382  Channel 0, interrupt request 5                                       */
  UTIMER_IRQ6_IRQn          = 383,              /*!< 383  Channel 0, interrupt request 6                                       */
  UTIMER_IRQ7_IRQn          = 384,              /*!< 384  Channel 0, interrupt request 7                                       */
  UTIMER_IRQ8_IRQn          = 385,              /*!< 385  Channel 1, interrupt request 8                                       */
  UTIMER_IRQ9_IRQn          = 386,              /*!< 386  Channel 1, interrupt request 9                                       */
  UTIMER_IRQ10_IRQn         = 387,              /*!< 387  Channel 1, interrupt request 10                                      */
  UTIMER_IRQ11_IRQn         = 388,              /*!< 388  Channel 1, interrupt request 11                                      */
  UTIMER_IRQ12_IRQn         = 389,              /*!< 389  Channel 1, interrupt request 12                                      */
  UTIMER_IRQ13_IRQn         = 390,              /*!< 390  Channel 1, interrupt request 13                                      */
  UTIMER_IRQ14_IRQn         = 391,              /*!< 391  Channel 1, interrupt request 14                                      */
  UTIMER_IRQ15_IRQn         = 392,              /*!< 392  Channel 1, interrupt request 15                                      */
  UTIMER_IRQ16_IRQn         = 393,              /*!< 393  Channel 2, interrupt request 16                                      */
  UTIMER_IRQ17_IRQn         = 394,              /*!< 394  Channel 2, interrupt request 17                                      */
  UTIMER_IRQ18_IRQn         = 395,              /*!< 395  Channel 2, interrupt request 18                                      */
  UTIMER_IRQ19_IRQn         = 396,              /*!< 396  Channel 2, interrupt request 19                                      */
  UTIMER_IRQ20_IRQn         = 397,              /*!< 397  Channel 2, interrupt request 20                                      */
  UTIMER_IRQ21_IRQn         = 398,              /*!< 398  Channel 2, interrupt request 21                                      */
  UTIMER_IRQ22_IRQn         = 399,              /*!< 399  Channel 2, interrupt request 22                                      */
  UTIMER_IRQ23_IRQn         = 400,              /*!< 400  Channel 2, interrupt request 23                                      */
  UTIMER_IRQ24_IRQn         = 401,              /*!< 401  Channel 3, interrupt request 24                                      */
  UTIMER_IRQ25_IRQn         = 402,              /*!< 402  Channel 3, interrupt request 25                                      */
  UTIMER_IRQ26_IRQn         = 403,              /*!< 403  Channel 3, interrupt request 26                                      */
  UTIMER_IRQ27_IRQn         = 404,              /*!< 404  Channel 3, interrupt request 27                                      */
  UTIMER_IRQ28_IRQn         = 405,              /*!< 405  Channel 3, interrupt request 28                                      */
  UTIMER_IRQ29_IRQn         = 406,              /*!< 406  Channel 3, interrupt request 29                                      */
  UTIMER_IRQ30_IRQn         = 407,              /*!< 407  Channel 3, interrupt request 30                                      */
  UTIMER_IRQ31_IRQn         = 408,              /*!< 408  Channel 3, interrupt request 31                                      */
} IRQn_Type;

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


/* --------  Configuration of Core Peripherals  ----------------------------------- */
#define __CM55_REV                0x0100U   /* Core revision r1p0 */
#define __NVIC_PRIO_BITS          8U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __MPU_PRESENT             1U        /* MPU present */
#define __FPU_PRESENT             1U        /* FPU present */
#define __FPU_DP                  1U        /* double precision FPU present*/
#define __DSP_PRESENT             1U        /* DSP extension present */
#define __SAUREGION_PRESENT       1U        /* SAU regions present */
#define __PMU_PRESENT             1U        /* PMU present */
#define __PMU_NUM_EVENTCNT        8U        /* PMU Event Counters */
#define __ICACHE_PRESENT          1U        /* Instruction Cache present */
#define __DCACHE_PRESENT          1U        /* Data Cache present */

#include "core_cm55.h"                      /* Processor and core peripherals */
#include "system_M55.h"                     /* System Header */
#include "M55_HE_map.h"                     /* Memory Map */
#include "system_utils.h"                   /* Utility functions */
#include "clk.h"                            /* clock functions */
#include "dma_mapping_E1C.h"                /* DMA mapping */
#include "mpu_M55.h"                        /* MPU functions */
#include "peripheral_types.h"               /* peripheral types*/
#include "pm.h"                             /* Power Management functions */

/* --------  End of section using anonymous unions and disabling warnings  -------- */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

#ifndef __STARTUP_RO_DATA_ATTRIBUTE
#define __STARTUP_RO_DATA_ATTRIBUTE  __attribute__((section("startup_ro_data")))
#endif

#ifdef __cplusplus
}
#endif

#endif  /* M55_HE_H */
