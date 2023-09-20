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

#include "Driver_I2C.h"
#include "i2c.h"

/**** system includes ****/
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

typedef volatile struct _I2C_DRIVER_STATE
{
    uint32_t initialized : 1;               /**< Driver Initialized */
    uint32_t powered     : 1;               /**< Driver powered     */
    uint32_t master_setup: 1;               /**< i2c master setup   */
    uint32_t slave_setup : 1;               /**< i2c master setup   */
    uint32_t reserved    : 28;              /**< Reserved           */
} I2C_DRIVER_STATE;

/* @brief Structure to save contexts for a i2c channel */
typedef struct _I2C_RESOURCES
{
    ARM_I2C_SignalEvent_t   cb_event;           /* Event callback                          */
    I2C_Type                *regs;              /* i2c register base address               */
    ARM_I2C_STATUS          status;             /* I2C status                              */
    I2C_DRIVER_STATE        state;              /* i2c driver state                        */
    i2c_transfer_info_t     transfer;           /* Transfer structure for I2C              */
    i2c_speed_mode_t        speed_mode;         /* I2C speed mode                          */
    uint32_t                clk;                /* system clock                            */
    uint8_t                 mode;               /* current working mode as master or slave */
    uint32_t                addr_mode;          /*  I2C_ADDRESS_MODE                       */
    uint32_t                slv_addr;           /* slave address                           */
    uint32_t                tar_addr;           /* target slave device address             */
    IRQn_Type               irq_num;            /* i2c interrupt vector number             */
    uint32_t                irq_priority;       /* i2c interrupt priority                  */
    uint8_t                 tx_fifo_threshold;  /* Tx Fifo Buffer threshold */
    uint8_t                 rx_fifo_threshold;  /* Rx Fifo Buffer threshold */

} I2C_RESOURCES;

#define I2C_SLAVE_MODE                              (0)          /* Indicate that the device working as slave */
#define I2C_MASTER_MODE                             (1)          /* Indicate that the device working as master */

#define I2C_DIR_TRANSMITTER                         (0)          /* direction transmitter  */
#define I2C_DIR_RECEIVER                            (1)          /* direction receiver     */

#define I2C_0_TARADDR                               (0x50)       /* I2C target address     */

#define I2C_7BIT_ADDRESS_MASK                       (0x7F)       /* 7bit I2C address mask  */

#define I2C_10BIT_ADDRESS_MASK                      (0x3FF)      /* 10bit I2C address mask */

#endif /* DRIVER_I2C_PRIVATE_H_ */
