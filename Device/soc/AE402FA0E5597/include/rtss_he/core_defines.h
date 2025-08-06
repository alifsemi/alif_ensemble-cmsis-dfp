/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef CORE_DEFINES_H
#define CORE_DEFINES_H

#include "soc_features.h"

#define HWSEM_MASTERID             (0x410FD222)

#define ITCM_ALIAS_BIT             (0x01000000UL)
#define DTCM_ALIAS_BIT             (0x01000000UL)
#define ITCM_BASE                  (0x00000000UL)
#define DTCM_BASE                  (0x20000000UL)
#define ITCM_REGION_SIZE           (0x02000000UL)
#define DTCM_REGION_SIZE           (0x02000000UL)
#define ITCM_SIZE                  SOC_FEAT_HE_ITCM_SIZE
#define DTCM_SIZE                  SOC_FEAT_HE_DTCM_SIZE
#define ITCM_GLOBAL_BASE           SOC_FEAT_HE_ITCM_BASE
#define DTCM_GLOBAL_BASE           SOC_FEAT_HE_DTCM_BASE

#define CORE_ID                    SOC_FEAT_M55_HE_CORE_ID
#define CORE_NAME                  "M55-HE"
#define CORE_SUBSYSTEM_NAME        "RTSS-HE"

#define CORE_DEFAULT_CLK           SOC_FEAT_M55_HE_MAX_HZ

#define WDTLOCAL_CTRL_BASE         WDT_HE_CTRL_BASE

/* ----------  Local Aliases  ---------- */

#define DMALOCAL_IRQ0Handler       DMA2_IRQ0Handler
#define DMALOCAL_IRQ1Handler       DMA2_IRQ1Handler
#define DMALOCAL_IRQ2Handler       DMA2_IRQ2Handler
#define DMALOCAL_IRQ3Handler       DMA2_IRQ3Handler
#define DMALOCAL_IRQ4Handler       DMA2_IRQ4Handler
#define DMALOCAL_IRQ5Handler       DMA2_IRQ5Handler
#define DMALOCAL_IRQ6Handler       DMA2_IRQ6Handler
#define DMALOCAL_IRQ7Handler       DMA2_IRQ7Handler
#define DMALOCAL_IRQ8Handler       DMA2_IRQ8Handler
#define DMALOCAL_IRQ9Handler       DMA2_IRQ9Handler
#define DMALOCAL_IRQ10Handler      DMA2_IRQ10Handler
#define DMALOCAL_IRQ11Handler      DMA2_IRQ11Handler
#define DMALOCAL_IRQ12Handler      DMA2_IRQ12Handler
#define DMALOCAL_IRQ13Handler      DMA2_IRQ13Handler
#define DMALOCAL_IRQ14Handler      DMA2_IRQ14Handler
#define DMALOCAL_IRQ15Handler      DMA2_IRQ15Handler
#define DMALOCAL_IRQ16Handler      DMA2_IRQ16Handler
#define DMALOCAL_IRQ17Handler      DMA2_IRQ17Handler
#define DMALOCAL_IRQ18Handler      DMA2_IRQ18Handler
#define DMALOCAL_IRQ19Handler      DMA2_IRQ19Handler
#define DMALOCAL_IRQ20Handler      DMA2_IRQ20Handler
#define DMALOCAL_IRQ21Handler      DMA2_IRQ21Handler
#define DMALOCAL_IRQ22Handler      DMA2_IRQ22Handler
#define DMALOCAL_IRQ23Handler      DMA2_IRQ23Handler
#define DMALOCAL_IRQ24Handler      DMA2_IRQ24Handler
#define DMALOCAL_IRQ25Handler      DMA2_IRQ25Handler
#define DMALOCAL_IRQ26Handler      DMA2_IRQ26Handler
#define DMALOCAL_IRQ27Handler      DMA2_IRQ27Handler
#define DMALOCAL_IRQ28Handler      DMA2_IRQ28Handler
#define DMALOCAL_IRQ29Handler      DMA2_IRQ29Handler
#define DMALOCAL_IRQ30Handler      DMA2_IRQ30Handler
#define DMALOCAL_IRQ31Handler      DMA2_IRQ31Handler
#define DMALOCAL_IRQ_ABORT_Handler DMA2_IRQ_ABORT_Handler

#define DMALOCAL_IRQ0_IRQn         DMA2_IRQ0_IRQn

#define MHU_SESS_S_RX_BASE         MHU_SECPU_M55HE_0_RX_BASE
#define MHU_SESS_S_TX_BASE         MHU_M55HE_SECPU_0_TX_BASE
#define MHU_SESS_NS_RX_BASE        MHU_SECPU_M55HE_1_RX_BASE
#define MHU_SESS_NS_TX_BASE        MHU_M55HE_SECPU_1_TX_BASE
#define MHU_RTSS_S_RX_BASE         MHU_M55HP_M55HE_0_RX_BASE
#define MHU_RTSS_S_TX_BASE         MHU_M55HE_M55HP_0_TX_BASE
#define MHU_RTSS_NS_RX_BASE        MHU_M55HP_M55HE_1_RX_BASE
#define MHU_RTSS_NS_TX_BASE        MHU_M55HE_M55HP_1_TX_BASE

/* ----------  Local MHU IRQ Aliases  ---------- */
#define MHU_SESS_S_RX_IRQHandler   MHU_SECPU_M55HE_0_RX_IRQHandler
#define MHU_SESS_S_TX_IRQHandler   MHU_M55HE_SECPU_0_TX_IRQHandler
#define MHU_SESS_NS_RX_IRQHandler  MHU_SECPU_M55HE_1_RX_IRQHandler
#define MHU_SESS_NS_TX_IRQHandler  MHU_M55HE_SECPU_1_TX_IRQHandler
#define MHU_RTSS_S_RX_IRQHandler   MHU_M55HP_M55HE_0_RX_IRQHandler
#define MHU_RTSS_S_TX_IRQHandler   MHU_M55HE_M55HP_0_TX_IRQHandler
#define MHU_RTSS_NS_RX_IRQHandler  MHU_M55HP_M55HE_1_RX_IRQHandler
#define MHU_RTSS_NS_TX_IRQHandler  MHU_M55HE_M55HP_1_TX_IRQHandler

#define MHU_SESS_S_RX_IRQ_IRQn     MHU_SECPU_M55HE_0_RX_IRQ_IRQn
#define MHU_SESS_S_TX_IRQ_IRQn     MHU_M55HE_SECPU_0_TX_IRQ_IRQn
#define MHU_SESS_NS_RX_IRQ_IRQn    MHU_SECPU_M55HE_1_RX_IRQ_IRQn
#define MHU_SESS_NS_TX_IRQ_IRQn    MHU_M55HE_SECPU_1_TX_IRQ_IRQn
#define MHU_RTSS_S_RX_IRQ_IRQn     MHU_M55HP_M55HE_0_RX_IRQ_IRQn
#define MHU_RTSS_S_TX_IRQ_IRQn     MHU_M55HE_M55HP_0_TX_IRQ_IRQn
#define MHU_RTSS_NS_RX_IRQ_IRQn    MHU_M55HP_M55HE_1_RX_IRQ_IRQn
#define MHU_RTSS_NS_TX_IRQ_IRQn    MHU_M55HE_M55HP_1_TX_IRQ_IRQn

/* ----------  Local NPU IRQ Aliases  ---------- */
#define NPULOCAL_IRQHandler        NPU_HE_IRQHandler

#define NPULOCAL_IRQ_IRQn          NPU_HE_IRQ_IRQn

#define NPULOCAL_BASE              NPU_HE_BASE

#define DMALOCAL_SEC_BASE          DMA2_SEC_BASE
#define DMALOCAL_NS_BASE           DMA2_NS_BASE

#define EVTRTRLOCAL                ((EVTRTR_Type *) EVTRTR2_BASE)
#define M55LOCAL_CFG               ((M55_CFG_Common_Type *) M55HE_CFG_BASE)

typedef struct { /*!< M55_HE_CFG and M55_HP_CFG common registers structure                      */
    __IOM uint32_t DMA_CTRL;   /*!< (@ 0x00000000) DMA Boot Control Register   */
    __IOM uint32_t DMA_IRQ;    /*!< (@ 0x00000004) DMA Boot IRQ Non-Secure Register    */
    __IOM uint32_t DMA_PERIPH; /*!< (@ 0x00000008) DMA Boot Peripheral Non-Secure Register */
    __IOM uint32_t DMA_SEL;    /*!< (@ 0x0000000C) DMA Select Register    */
    __IOM uint32_t CLK_ENA;    /*!< (@ 0x00000010) Local Peripheral Clock Enable Register    */
} M55_CFG_Common_Type;

#endif /* CORE_DEFINES_H */
