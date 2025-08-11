/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef VECTORS_H
#define VECTORS_H

#define IRQ0_Handler  DMA1_IRQ0Handler
#define IRQ1_Handler  DMA1_IRQ1Handler
#define IRQ2_Handler  DMA1_IRQ2Handler
#define IRQ3_Handler  DMA1_IRQ3Handler
#define IRQ4_Handler  DMA1_IRQ4Handler
#define IRQ5_Handler  DMA1_IRQ5Handler
#define IRQ6_Handler  DMA1_IRQ6Handler
#define IRQ7_Handler  DMA1_IRQ7Handler
#define IRQ8_Handler  DMA1_IRQ8Handler
#define IRQ9_Handler  DMA1_IRQ9Handler
#define IRQ10_Handler DMA1_IRQ10Handler
#define IRQ11_Handler DMA1_IRQ11Handler
#define IRQ12_Handler DMA1_IRQ12Handler
#define IRQ13_Handler DMA1_IRQ13Handler
#define IRQ14_Handler DMA1_IRQ14Handler
#define IRQ15_Handler DMA1_IRQ15Handler
#define IRQ16_Handler DMA1_IRQ16Handler
#define IRQ17_Handler DMA1_IRQ17Handler
#define IRQ18_Handler DMA1_IRQ18Handler
#define IRQ19_Handler DMA1_IRQ19Handler
#define IRQ20_Handler DMA1_IRQ20Handler
#define IRQ21_Handler DMA1_IRQ21Handler
#define IRQ22_Handler DMA1_IRQ22Handler
#define IRQ23_Handler DMA1_IRQ23Handler
#define IRQ24_Handler DMA1_IRQ24Handler
#define IRQ25_Handler DMA1_IRQ25Handler
#define IRQ26_Handler DMA1_IRQ26Handler
#define IRQ27_Handler DMA1_IRQ27Handler
#define IRQ28_Handler DMA1_IRQ28Handler
#define IRQ29_Handler DMA1_IRQ29Handler
#define IRQ30_Handler DMA1_IRQ30Handler
#define IRQ31_Handler DMA1_IRQ31Handler
#define IRQ32_Handler DMA1_IRQ_ABORT_Handler
#define IRQ33_Handler MHU_A32_M55HP_0_RX_IRQHandler
#define IRQ34_Handler MHU_M55HP_A32_0_TX_IRQHandler
#define IRQ35_Handler MHU_A32_M55HP_1_RX_IRQHandler
#define IRQ36_Handler MHU_M55HP_A32_1_TX_IRQHandler
#define IRQ37_Handler MHU_SECPU_M55HP_0_RX_IRQHandler
#define IRQ38_Handler MHU_M55HP_SECPU_0_TX_IRQHandler
#define IRQ39_Handler MHU_SECPU_M55HP_1_RX_IRQHandler
#define IRQ40_Handler MHU_M55HP_SECPU_1_TX_IRQHandler
#define IRQ41_Handler MHU_M55HE_M55HP_0_RX_IRQHandler
#define IRQ42_Handler MHU_M55HP_M55HE_0_TX_IRQHandler
#define IRQ43_Handler MHU_M55HE_M55HP_1_RX_IRQHandler
#define IRQ44_Handler MHU_M55HP_M55HE_1_TX_IRQHandler

#define IRQ55_Handler NPU_HP_IRQHandler
#define IRQ56_Handler LPCMP_IRQHandler
#define IRQ57_Handler LPGPIO_COMB_IRQHandler
#define IRQ58_Handler LPRTC0_IRQHandler

#define IRQ60_Handler LPTIMER0_IRQHandler
#define IRQ61_Handler LPTIMER1_IRQHandler
#define IRQ62_Handler LPTIMER2_IRQHandler
#define IRQ63_Handler LPTIMER3_IRQHandler

#endif /* VECTORS_H */
