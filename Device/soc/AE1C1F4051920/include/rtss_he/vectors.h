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

#define IRQ0_Handler  DMA2_IRQ0Handler
#define IRQ1_Handler  DMA2_IRQ1Handler
#define IRQ2_Handler  DMA2_IRQ2Handler
#define IRQ3_Handler  DMA2_IRQ3Handler
#define IRQ4_Handler  DMA2_IRQ4Handler
#define IRQ5_Handler  DMA2_IRQ5Handler
#define IRQ6_Handler  DMA2_IRQ6Handler
#define IRQ7_Handler  DMA2_IRQ7Handler
#define IRQ8_Handler  DMA2_IRQ8Handler
#define IRQ9_Handler  DMA2_IRQ9Handler
#define IRQ10_Handler DMA2_IRQ10Handler
#define IRQ11_Handler DMA2_IRQ11Handler
#define IRQ12_Handler DMA2_IRQ12Handler
#define IRQ13_Handler DMA2_IRQ13Handler
#define IRQ14_Handler DMA2_IRQ14Handler
#define IRQ15_Handler DMA2_IRQ15Handler
#define IRQ16_Handler DMA2_IRQ16Handler
#define IRQ17_Handler DMA2_IRQ17Handler
#define IRQ18_Handler DMA2_IRQ18Handler
#define IRQ19_Handler DMA2_IRQ19Handler
#define IRQ20_Handler DMA2_IRQ20Handler
#define IRQ21_Handler DMA2_IRQ21Handler
#define IRQ22_Handler DMA2_IRQ22Handler
#define IRQ23_Handler DMA2_IRQ23Handler
#define IRQ24_Handler DMA2_IRQ24Handler
#define IRQ25_Handler DMA2_IRQ25Handler
#define IRQ26_Handler DMA2_IRQ26Handler
#define IRQ27_Handler DMA2_IRQ27Handler
#define IRQ28_Handler DMA2_IRQ28Handler
#define IRQ29_Handler DMA2_IRQ29Handler
#define IRQ30_Handler DMA2_IRQ30Handler
#define IRQ31_Handler DMA2_IRQ31Handler
#define IRQ32_Handler DMA2_IRQ_ABORT_Handler

#define IRQ37_Handler MHU_SECPU_M55HE_0_RX_IRQHandler
#define IRQ38_Handler MHU_M55HE_SECPU_0_TX_IRQHandler
#define IRQ39_Handler MHU_SECPU_M55HE_1_RX_IRQHandler
#define IRQ40_Handler MHU_M55HE_SECPU_1_TX_IRQHandler

#define IRQ45_Handler LPUART_IRQHandler
#define IRQ46_Handler LPSPI_IRQHandler
#define IRQ47_Handler LPI2C0_IRQHandler
#define IRQ48_Handler LPI2S_IRQHandler
#define IRQ49_Handler LPPDM_IRQHandler

#define IRQ54_Handler LPCPI_IRQHandler
#define IRQ55_Handler NPU_HE_IRQHandler
#define IRQ56_Handler LPCMP_IRQHandler
#define IRQ57_Handler LPGPIO_COMB_IRQHandler
#define IRQ58_Handler LPRTC0_IRQHandler
#define IRQ59_Handler LPRTC1_IRQHandler

#define IRQ60_Handler LPTIMER0_IRQHandler
#define IRQ61_Handler LPTIMER1_IRQHandler

#endif /* VECTORS_H */
