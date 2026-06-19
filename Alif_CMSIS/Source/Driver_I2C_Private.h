/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DRIVER_I2C_PRIVATE_H_
#define DRIVER_I2C_PRIVATE_H_

#include "Driver_I2C_EX.h"
#include "i2c.h"
#include "sys_ctrl_i2c.h"

/**** system includes ****/
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#if (RTE_I2C0_DMA_ENABLE || RTE_I2C1_DMA_ENABLE || RTE_I2C2_DMA_ENABLE || RTE_I2C3_DMA_ENABLE ||   \
     RTE_LPI2C1_DMA_ENABLE)
#define I2C_DMA_ENABLE 1
#else
#define I2C_DMA_ENABLE 0
#endif

#if I2C_DMA_ENABLE
#include <DMA_Common.h>
#include "dma_opcode.h"
#endif

#if I2C_DMA_ENABLE
/* Microcode buffer size for the per-instance TX and RX microcodes.
 * Sized for a single DMALP chunk (max 256 iterations):
 * 96 B fits the worst case.
 */
#define I2C_DMA_MCODE_SIZE 96
#endif

typedef volatile struct _I2C_DRIVER_STATE {
    uint32_t initialized : 1;  /**< Driver Initialized */
    uint32_t powered     : 1;  /**< Driver powered     */
    uint32_t master_setup: 1;  /**< i2c master setup   */
    uint32_t slave_setup : 1;  /**< i2c master setup   */
    uint32_t reserved    : 28; /**< Reserved           */
} I2C_DRIVER_STATE;

#if I2C_DMA_ENABLE
typedef struct _I2C_DMA_HW_CONFIG {
    DMA_PERIPHERAL_CONFIG dma_tx; /* Tx interface */
    DMA_PERIPHERAL_CONFIG dma_rx; /* Rx interface */
} I2C_DMA_HW_CONFIG;
#endif

/* @brief Structure to save contexts for a i2c channel */
typedef struct _I2C_RESOURCES {
    ARM_I2C_SignalEvent_t cb_event;     /* Event callback                          */
    I2C_Type             *regs;         /* i2c register base address               */
    ARM_I2C_STATUS        status;       /* I2C status                              */
    I2C_DRIVER_STATE      state;        /* i2c driver state                        */
    i2c_transfer_info_t   transfer;     /* Transfer structure for I2C              */
    uint32_t              clk;          /* system clock                            */
    uint32_t              addr_mode;    /*  I2C_ADDRESS_MODE                       */
    uint32_t              slv_addr;     /* slave address                           */
    uint32_t              tar_addr;     /* target slave device address             */
    uint32_t              irq_priority; /* i2c interrupt priority                  */
    IRQn_Type             irq_num;      /* i2c interrupt vector number             */
    i2c_speed_mode_t      speed_mode;   /* I2C speed mode                          */
    uint8_t               mode;         /* current working mode as master or slave */
    uint8_t wr_mode_info; /* Write-Read combined mode - Bit0 - On/Off, Bits7-4 - Tar reg addr size
                           */
#if I2C_DMA_ENABLE
    const bool            dma_enable;       /* I2C dma enable                          */
    const uint32_t        dma_irq_priority; /* DMA IRQ priority number                 */
    ARM_DMA_SignalEvent_t dma_cb;           /* I2S DMA Callback                        */
    I2C_DMA_HW_CONFIG    *dma_cfg;          /* DMA Controller configuration            */
    uint16_t             *dma_tx_scratch;    /* 16-bit DATA_CMD scratch buf            */
    uint32_t              dma_tx_scratch_sz; /* Scratch buf capacity                   */
    const uint8_t        *dma_xfer_src;     /* Pointer into the user's 8-bit TX buffer */
    uint32_t              dma_xfer_remaining; /* Bytes left after the current chunk    */
    uint32_t              dma_xfer_total;   /* Original transfer size                  */
    bool                  dma_xfer_pending; /* If true, no STOP on final chunk         */
    uint8_t               dma_mcode[I2C_DMA_MCODE_SIZE] __ALIGNED(32); /* microcode    */
#endif
    uint8_t            tx_fifo_threshold; /* Tx Fifo Buffer threshold                */
    uint8_t            rx_fifo_threshold; /* Rx Fifo Buffer threshold                */
    uint32_t           scl_stuck_timeout; /* SCL Stuck at Low Timeout                */
    const I2C_INSTANCE instance;          /* I2C Instance number                     */
} I2C_RESOURCES;

#define I2C_SLAVE_MODE         (0) /* Indicate that the device working as slave */
#define I2C_MASTER_MODE        (1) /* Indicate that the device working as master */

#define I2C_DIR_TRANSMITTER    (0) /* direction transmitter  */
#define I2C_DIR_RECEIVER       (1) /* direction receiver     */

#define I2C_0_TARADDR          (0x50) /* I2C target address     */

/* 7-bit reserved-range boundaries (inclusive of valid window). */
#define I2C_7BIT_ADDR_MIN          0x08U
#define I2C_7BIT_ADDR_MAX          0x77U
#define I2C_7BIT_ADDR_MASK         0x7FU

/* 10-bit address spans the full 10-bit field */
#define I2C_10BIT_ADDR_MAX         0x3FFU

#endif /* DRIVER_I2C_PRIVATE_H_ */
