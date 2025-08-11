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
 * @file     evtrtr.h
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @version  V1.0.0
 * @date     5-May-2023
 * @brief    System Control Device information for Event Router
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef EVTRTR_H

#define EVTRTR_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "RTE_Components.h"
#include CMSIS_device_header
#include "RTE_Device.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(RTE_DMA0)
#define RTE_EVTRTR0 RTE_DMA0
#endif
#if defined(RTE_DMA1)
#define RTE_EVTRTR1 RTE_DMA1
#endif
#if defined(RTE_DMA2)
#define RTE_EVTRTR2 RTE_DMA2
#endif

#define DMA_CTRL_ACK_TYPE_Pos                                                                      \
    (16U)                          /* HandShake Type 0: Peripheral, 1: Event Router               */
#define DMA_CTRL_ENA     (1U << 4) /* Enable DMA Channel                                          */
#define DMA_CTRL_SEL_Pos (0U)      /* Select DMA group from 0 to 3 for DMA0. For Local, only GRP0 */
#define DMA_CTRL_SEL_Msk (0x3)     /* Select DMA group from 0 to 3 for DMA0. For Local, only GRP0 */

#define DMA_REQ_CTRL_CB  (1U << 12) /* Enable peripheral Burst DMA request  : DMACBREQ        */
#define DMA_REQ_CTRL_CS  (1U << 8)  /* Enable peripheral Single DMA request : DMACSREQ        */
#define DMA_REQ_CTRL_CLB (1U << 4)  /* Enable peripheral Last Burst DMA request  : DMACLBREQ  */
#define DMA_REQ_CTRL_CLS (1U << 0)  /* Enable peripheral Last Single DMA request  : DMACLSREQ */

/**
 * enum DMA_ACK_COMPLETION
 * Defines the completion of REQ-ACK with DMA
 */
typedef enum _DMA_ACK_COMPLETION {
    DMA_ACK_COMPLETION_PERIPHERAL, /**< Peripheral completes with DMA    */
    DMA_ACK_COMPLETION_EVTRTR,     /**< Event Router completes with DMA  */
} DMA_ACK_COMPLETION;

#define EVTRTR_DMA_CHANNEL_MAX_Msk 0x1F /* Channel range 0-31 */

/**
\brief Event Router Configuration
*/
typedef struct _EVTRTR_CONFIG {

    /*!< Instance number  */
    const uint8_t instance;

    /*!< Group number */
    const uint8_t group;

    /*!< Channel number  */
    const uint8_t channel;

    /*!< Enable handshake */
    const bool enable_handshake;
} EVTRTR_CONFIG;

#if (RTE_EVTRTR0)
static inline void evtrtr0_enable_dma_channel(uint8_t channel, uint8_t group,
                                              DMA_ACK_COMPLETION ack_type)
{
    uint32_t index = channel & EVTRTR_DMA_CHANNEL_MAX_Msk;

    EVTRTR0->EVTRTR_DMA_CTRL[index] =
        DMA_CTRL_ENA | (DMA_CTRL_SEL_Msk & group) | (ack_type << DMA_CTRL_ACK_TYPE_Pos);
}

static inline void evtrtr0_disable_dma_channel(uint8_t channel)
{
    uint32_t index                  = channel & EVTRTR_DMA_CHANNEL_MAX_Msk;

    EVTRTR0->EVTRTR_DMA_CTRL[index] = 0;
}

static inline void evtrtr0_enable_dma_req(void)
{
    EVTRTR0->EVTRTR_DMA_REQ_CTRL |=
        (DMA_REQ_CTRL_CB | DMA_REQ_CTRL_CLB | DMA_REQ_CTRL_CS | DMA_REQ_CTRL_CLS);
}

static inline void evtrtr0_disable_dma_req(void)
{
    EVTRTR0->EVTRTR_DMA_REQ_CTRL = 0;
}

static inline void evtrtr0_enable_dma_handshake(uint8_t channel, uint8_t group)
{
    volatile uint32_t *dma_ack_type_addr =
        &EVTRTR0->EVTRTR_DMA_ACK_TYPE0 + (DMA_CTRL_SEL_Msk & group);

    __disable_irq();
    *dma_ack_type_addr |= (1 << (channel & EVTRTR_DMA_CHANNEL_MAX_Msk));
    __enable_irq();
}

static inline void evtrtr0_disable_dma_handshake(uint8_t channel, uint8_t group)
{
    volatile uint32_t *dma_ack_type_addr =
        &EVTRTR0->EVTRTR_DMA_ACK_TYPE0 + (DMA_CTRL_SEL_Msk & group);

    __disable_irq();
    *dma_ack_type_addr &= ~(1 << (channel & EVTRTR_DMA_CHANNEL_MAX_Msk));
    __enable_irq();
}
#endif /* RTE_EVTRTR0 */

#if (RTE_EVTRTR1 || RTE_EVTRTR2)
static inline void evtrtrlocal_enable_dma_channel(uint8_t channel, uint8_t group,
                                                  DMA_ACK_COMPLETION ack_type)
{
    uint32_t index = channel & EVTRTR_DMA_CHANNEL_MAX_Msk;

    EVTRTRLOCAL->EVTRTR_DMA_CTRL[index] =
        DMA_CTRL_ENA | (DMA_CTRL_SEL_Msk & group) | (ack_type << DMA_CTRL_ACK_TYPE_Pos);
}

static inline void evtrtrlocal_disable_dma_channel(uint8_t channel)
{
    uint32_t index                      = channel & EVTRTR_DMA_CHANNEL_MAX_Msk;

    EVTRTRLOCAL->EVTRTR_DMA_CTRL[index] = 0;
}

static inline void evtrtrlocal_enable_dma_req(void)
{
    EVTRTRLOCAL->EVTRTR_DMA_REQ_CTRL |=
        (DMA_REQ_CTRL_CB | DMA_REQ_CTRL_CLB | DMA_REQ_CTRL_CS | DMA_REQ_CTRL_CLS);
}

static inline void evtrtrlocal_disable_dma_req(void)
{
    EVTRTRLOCAL->EVTRTR_DMA_REQ_CTRL = 0;
}

static inline void evtrtrlocal_enable_dma_handshake(uint8_t channel, uint8_t group)
{
    volatile uint32_t *dma_ack_type_addr =
        &EVTRTRLOCAL->EVTRTR_DMA_ACK_TYPE0 + (DMA_CTRL_SEL_Msk & group);

    __disable_irq();
    *dma_ack_type_addr |= (1 << (channel & EVTRTR_DMA_CHANNEL_MAX_Msk));
    __enable_irq();
}

static inline void evtrtrlocal_disable_dma_handshake(uint8_t channel, uint8_t group)
{
    volatile uint32_t *dma_ack_type_addr =
        &EVTRTRLOCAL->EVTRTR_DMA_ACK_TYPE0 + (DMA_CTRL_SEL_Msk & group);

    __disable_irq();
    *dma_ack_type_addr &= ~(1 << (channel & EVTRTR_DMA_CHANNEL_MAX_Msk));
    __enable_irq();
}
#endif /* (RTE_EVTRTR1 || RTE_EVTRTR2) */

/* Generic wrappper for dma functions */
static inline void evtrtr_enable_dma_channel(const uint8_t instance, uint8_t channel, uint8_t group,
                                             DMA_ACK_COMPLETION ack_type)
{
#if (RTE_EVTRTR0)
    if (instance == 0) {
        evtrtr0_enable_dma_channel(channel, group, ack_type);
    }
#endif
#if (RTE_EVTRTR1 || RTE_EVTRTR2)
    if (instance != 0) {
        evtrtrlocal_enable_dma_channel(channel, group, ack_type);
    }
#endif
}

static inline void evtrtr_disable_dma_channel(const uint8_t instance, uint8_t channel)
{
#if (RTE_EVTRTR0)
    if (instance == 0) {
        evtrtr0_disable_dma_channel(channel);
    }
#endif
#if (RTE_EVTRTR1 || RTE_EVTRTR2)
    if (instance != 0) {
        evtrtrlocal_disable_dma_channel(channel);
    }
#endif
}

static inline void evtrtr_enable_dma_req(const uint8_t instance)
{
#if (RTE_EVTRTR0)
    if (instance == 0) {
        evtrtr0_enable_dma_req();
    }
#endif
#if (RTE_EVTRTR1 || RTE_EVTRTR2)
    if (instance != 0) {
        evtrtrlocal_enable_dma_req();
    }
#endif
}

static inline void evtrtr_disable_dma_req(const uint8_t instance)
{
#if (RTE_EVTRTR0)
    if (instance == 0) {
        evtrtr0_disable_dma_req();
    }
#endif
#if (RTE_EVTRTR1 || RTE_EVTRTR2)
    if (instance != 0) {
        evtrtrlocal_disable_dma_req();
    }
#endif
}

static inline void evtrtr_enable_dma_handshake(const uint8_t instance, uint8_t channel,
                                               uint8_t group)
{
#if (RTE_EVTRTR0)
    if (instance == 0) {
        evtrtr0_enable_dma_handshake(channel, group);
    }
#endif
#if (RTE_EVTRTR1 || RTE_EVTRTR2)
    if (instance != 0) {
        evtrtrlocal_enable_dma_handshake(channel, group);
    }
#endif
}

static inline void evtrtr_disable_dma_handshake(const uint8_t instance, uint8_t channel,
                                                uint8_t group)
{
#if (RTE_EVTRTR0)
    if (instance == 0) {
        evtrtr0_disable_dma_handshake(channel, group);
    }
#endif
#if (RTE_EVTRTR1 || RTE_EVTRTR2)
    if (instance != 0) {
        evtrtrlocal_disable_dma_handshake(channel, group);
    }
#endif
}
#ifdef __cplusplus
}
#endif
#endif /* EVTRTR_H */
