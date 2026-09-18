/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DRIVER_I3C_PRIVATE_H
#define DRIVER_I3C_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I3C.h"
#include "i3c.h"
#include "sys_ctrl_i3c.h"

/* Check if DMA Support is enable? */
#if (RTE_I3C_DMA_ENABLE || RTE_LPI3C_DMA_ENABLE)
#define I3C_DMA_ENABLE 1
#else
#define I3C_DMA_ENABLE 0
#endif

#if I3C_DMA_ENABLE
#include <DMA_Common.h>

#define I3C_DMA_MCODE_SIZE  96  /* PL330 microcode buffer size in bytes */
#endif

#define I3C_TARGET_SLAVE_TYPE_I2C (1U << 7U) /* Represents slave type */

#define I3C_BCR_MAX_DATA_SPEED_LIMIT  (1U << 0)
#define I3C_GETMXDS_MAX_SDR_FSCL_Msk  (0x7U)

/**
\brief I3C target discovery stage
*/
typedef enum _I3C_DISC_STEP {
    I3C_DISC_IDLE = 0,
    I3C_DISC_GETBCR,
    I3C_DISC_GETMXDS
} I3C_DISC_STEP;

/**
\brief I3C target discovery type
*/
typedef struct _I3C_DISC_TYPE {
    I3C_DISC_STEP step;
    uint8_t       pos;
    uint8_t       rx[8];
} I3C_DISC_TYPE;

/**
\brief I3C Driver states.
*/
typedef volatile struct _I3C_DRIVER_STATE {
    uint32_t initialized: 1;  /* Driver initialized    */
    uint32_t powered    : 1;  /* Driver powered        */
    uint32_t is_master  : 1;  /* Driver master mode    */
    uint32_t enabled    : 1;  /* Driver enabled        */
    uint32_t reserved   : 28; /* Reserved              */
} I3C_DRIVER_STATE;

/**
\brief I3C Target's profile
*/
typedef struct _I3C_TARGET_PROFILE {
    uint8_t addr;      /* DA (I3C) or SA (I2C); bit7 = I3C_TARGET_SLAVE_TYPE_I2C */
    uint8_t bcr;       /* BCR */
    uint8_t speed_wr;  /* SPEED for Tx */
    uint8_t speed_rd;  /* SPEED for Rx */
    uint8_t disc_done; /* 1: GETBCR/GETMXDS finished for this DAT slot */
} I3C_TARGET_PROFILE;

/**
\brief I3C Slave Device Address info
*/
typedef struct _I3C_TARGET_TABLE {
    uint32_t           datp;                        /* DAT (Device Address Table) offset     */
    uint32_t           maxdevs;                     /* maximum number of slaves supported    */
    I3C_TARGET_PROFILE profile[I3C_MAX_DEVS];       /* Array of target profile               */
    uint32_t           freepos;                     /* bitmask of used addresses             */
    uint32_t           last_asgd_addr_pos;          /* Last assigned slave address positions */
} I3C_TARGET_TABLE;

#if I3C_DMA_ENABLE
typedef struct _I3C_DMA_HW_CONFIG {
    DMA_PERIPHERAL_CONFIG dma_tx; /* DMA Tx interface */
    DMA_PERIPHERAL_CONFIG dma_rx; /* DMA Rx interface */
} I3C_DMA_HW_CONFIG;
#endif

/**
\brief I3C Device Resources
*/
typedef struct _I3C_RESOURCES {
    I3C_Type              *regs;        /* Pointer to i3c regs                                */
    ARM_I3C_SignalEvent_t cb_event;     /* Pointer to call back function                      */
    uint32_t              core_clk;     /* i3c core clock frequency                           */
    I3C_TARGET_TABLE      targets;      /* i3c target's basic info                            */
    I3C_DISC_TYPE         disc;
    i3c_xfer_t            xfer;         /* i3c transfer structure                             */
    ARM_I3C_STATUS        status;       /* i3c driver status                                  */
    I3C_DRIVER_STATE      state;        /* I3C driver state                                   */
#if RTE_I3C_BLOCKING_MODE_ENABLE
    bool blocking_mode;                 /* I3C blocking mode transfer enable                  */
#endif
    bool               adaptive_mode; /* I3C slave I2C/I3C adaptive mode                    */
    IRQn_Type          irq;           /* i3c interrupt number                               */
    uint32_t           irq_priority;  /* i3c interrupt priority                             */
    const I3C_INSTANCE instance;      /* I3C Instance number                                */
#if I3C_DMA_ENABLE
    const bool            dma_enable;       /* I3C dma enable                                     */
    ARM_DMA_SignalEvent_t dma_cb;           /* Pointer to DMA  Callback                           */
    I3C_DMA_HW_CONFIG    *dma_cfg;          /* DMA Controller configuration                       */
    const uint32_t        dma_irq_priority; /* DMA IRQ priority number                            */
    uint8_t               dma_tx_mcode[I3C_DMA_MCODE_SIZE] __ALIGNED(4); /* TX microcode buffer   */
    uint8_t               dma_rx_mcode[I3C_DMA_MCODE_SIZE] __ALIGNED(4); /* RX microcode buffer   */
    uint8_t               dma_rx_scratch[4] __ALIGNED(4); /* RX tail scratch for unaligned len    */
#endif
} I3C_RESOURCES;

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_I3C_PRIVATE_H */
