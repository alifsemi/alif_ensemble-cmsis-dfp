/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     sys_ctrl_dma.h
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @version  V1.0.0
 * @date     18-Apr-2023
 * @brief    System Control Device information for DMA.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef SYS_CTRL_DMA_H

#define SYS_CTRL_DMA_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "RTE_Device.h"
#include "soc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PERIPH_CLK_ENA_DMA_CKEN (1U << 4) /* Enable clock supply for DMA0 */

#define DMA_CTRL_BOOT_MANAGER   (1U << 0)  /* 0: Secure, 1: Non-Secure */
#define DMA_CTRL_SW_RST         (1U << 16) /* Software reset of DMA */

#define DMA_SEL_FLT_ENA_Pos     (24)                          /* Glitch filter Pos for  GPIO   */
#define DMA_SEL_FLT_ENA_Msk     (0xFF << DMA_SEL_FLT_ENA_Pos) /* Glitch filter Mask for GPIO   */

#define CLK_ENA_DMA_CKEN        (1U << 4) /* Enable clock supply for DMA */

#if SOC_FEAT_HAS_FLT_ENA_IN_DMA_SEL_REG
#define HE_DMA_SEL_FLT_ENA_Pos (24)   /* Glitch filter Pos for LPGPIO    */
#define HE_DMA_SEL_FLT_ENA_Msk (0xFF) /* Glitch filter Mask for LPGPIO   */
#endif
#define HE_DMA_SEL_PDM_DMA0   (1U << 16)                     /* Select DMA0 for LPPDM           */
#define HE_DMA_SEL_I2S_DMA0   (1U << 12)                     /* Select DMA0 for LPI2S           */
#define HE_DMA_SEL_I3C_DMA0   (1U << 9)                      /* Select DMA0 for LPI3C           */
#define HE_DMA_SEL_I2C_DMA0   (1U << 8)                      /* Select DMA0 for LPI2C1          */
#define HE_DMA_SEL_LPSPI_Pos  (4)                            /* Pos to Select DMA0 for LPSPI    */
#define HE_DMA_SEL_LPSPI_Msk  (0x3U << HE_DMA_SEL_LPSPI_Pos) /* Mask to Select DMA0 for LPSPI   */
#define HE_DMA_SEL_LPSPI_DMA2 (0x0U << HE_DMA_SEL_LPSPI_Pos) /* Select DMA2 for LPSPI           */
#define HE_DMA_SEL_LPSPI_DMA0_GROUP1                                                               \
    (0x1U << HE_DMA_SEL_LPSPI_Pos) /* Select DMA0 Group1 for LPSPI    */
#define HE_DMA_SEL_LPSPI_DMA0_GROUP2                                                               \
    (0x2U << HE_DMA_SEL_LPSPI_Pos)       /* Select DMA0 Group2 for LPSPI    */
#define HE_DMA_SEL_LPUART_DMA0 (1U << 0) /* Select DMA0 for LPUART          */

/**
 * enum DMA_INSTANCE
 * DMA instances
 */
typedef enum _DMA_INSTANCE {
    DMA_INSTANCE_0,     /**< DMA0       */
    DMA_INSTANCE_LOCAL, /**< DMALOCAL   */
} DMA_INSTANCE;

#if (RTE_DMA0)
static inline void dma0_enable_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_DMA_CKEN;
}

static inline void dma0_disable_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_DMA_CKEN;
}

static inline void dma0_set_boot_manager_secure(void)
{
    CLKCTL_PER_MST->DMA_CTRL &= ~DMA_CTRL_BOOT_MANAGER;
}

static inline void dma0_set_boot_manager_nonsecure(void)
{
    CLKCTL_PER_MST->DMA_CTRL |= DMA_CTRL_BOOT_MANAGER;
}

static inline void dma0_set_boot_irq_ns_mask(uint32_t boot_irq_ns_mask)
{
    CLKCTL_PER_MST->DMA_IRQ = boot_irq_ns_mask;
}

static inline void dma0_set_boot_periph_ns_mask(uint32_t boot_periph_ns_mask)
{
    CLKCTL_PER_MST->DMA_PERIPH = boot_periph_ns_mask;
}

static inline void dma0_reset(void)
{
    CLKCTL_PER_MST->DMA_CTRL |= DMA_CTRL_SW_RST;
}

static inline void dma0_set_glitch_filter(uint32_t glitch_filter)
{
    CLKCTL_PER_MST->DMA_GLITCH_FLT = glitch_filter;
}

static inline void lppdm_select_dma0(void)
{
    M55HE_CFG->HE_DMA_SEL |= HE_DMA_SEL_PDM_DMA0;
}

static inline void lpi2s_select_dma0(void)
{
    M55HE_CFG->HE_DMA_SEL |= HE_DMA_SEL_I2S_DMA0;
}

static inline void lpi3c_select_dma0(void)
{
    M55HE_CFG->HE_DMA_SEL |= HE_DMA_SEL_I3C_DMA0;
}

static inline void lpi2c1_select_dma0(void)
{
    M55HE_CFG->HE_DMA_SEL |= HE_DMA_SEL_I2C_DMA0;
}

static inline void lpspi_select_dma0(uint8_t group)
{
    M55HE_CFG->HE_DMA_SEL |= ((group << HE_DMA_SEL_LPSPI_Pos) & HE_DMA_SEL_LPSPI_Msk);
}

static inline void lpuart_select_dma0(void)
{
    M55HE_CFG->HE_DMA_SEL |= HE_DMA_SEL_LPUART_DMA0;
}
#endif /* RTE_DMA0 */

#if ((RTE_DMA1) || (RTE_DMA2))
static inline void dmalocal_enable_periph_clk(void)
{
    M55LOCAL_CFG->CLK_ENA |= CLK_ENA_DMA_CKEN;
}

static inline void dmalocal_disable_periph_clk(void)
{
    M55LOCAL_CFG->CLK_ENA &= ~CLK_ENA_DMA_CKEN;
}

static inline void dmalocal_set_boot_manager_secure(void)
{
    M55LOCAL_CFG->DMA_CTRL &= ~DMA_CTRL_BOOT_MANAGER;
}

static inline void dmalocal_set_boot_manager_nonsecure(void)
{
    M55LOCAL_CFG->DMA_CTRL |= DMA_CTRL_BOOT_MANAGER;
}

static inline void dmalocal_set_boot_irq_ns_mask(uint32_t boot_irq_ns_mask)
{
    M55LOCAL_CFG->DMA_IRQ = boot_irq_ns_mask;
}

static inline void dmalocal_set_boot_periph_ns_mask(uint32_t boot_periph_ns_mask)
{
    M55LOCAL_CFG->DMA_PERIPH = boot_periph_ns_mask;
}

static inline void dmalocal_reset(void)
{
    M55LOCAL_CFG->DMA_CTRL |= DMA_CTRL_SW_RST;
}

#if SOC_FEAT_HAS_FLT_ENA_IN_DMA_SEL_REG
static inline void dmalocal_set_glitch_filter(uint8_t glitch_filter)
{
    M55LOCAL_CFG->DMA_SEL |= (glitch_filter << DMA_SEL_FLT_ENA_Pos);
}
#endif

#if SOC_FEAT_DMA2_HAS_FLT_ENA0_REG
static inline void dma2_set_glitch_filter0(uint32_t glitch_filter)
{
    M55HE_CFG->HE_FLT_ENA0 = glitch_filter;
}
#endif
#if SOC_FEAT_DMA2_HAS_FLT_ENA1_REG
static inline void dma2_set_glitch_filter1(uint32_t glitch_filter)
{
    M55HE_CFG->HE_FLT_ENA1 = glitch_filter;
}
#endif

#if RTE_LPSPI_SELECT_DMA2_GROUP
static inline void lpspi_select_dma2_group(uint8_t group)
{
    M55HE_CFG->HE_DMA_SEL |= ((group << HE_DMA_SEL_LPSPI_Pos) & HE_DMA_SEL_LPSPI_Msk);
}
#endif

#endif /* ((RTE_DMA1) || (RTE_DMA2)) */

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_DMA_H */
