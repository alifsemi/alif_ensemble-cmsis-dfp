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

#ifdef  __cplusplus
extern "C"
{
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I3C.h"
#include "i3c.h"

/* Check if DMA Support is enable? */
#if RTE_I3C_DMA_ENABLE
#define I3C_DMA_ENABLE  1
#else
#define I3C_DMA_ENABLE  0
#endif

#if I3C_DMA_ENABLE
#include <DMA_Common.h>
#endif

/**
\brief I3C Driver states.
*/
typedef volatile struct _I3C_DRIVER_STATE
{
  uint32_t initialized    : 1; /* Driver initialized    */
  uint32_t powered        : 1; /* Driver powered        */
  uint32_t master_enabled : 1; /* Driver master enabled */
  uint32_t slave_enabled  : 1; /* Driver slave  enabled */
  uint32_t reserved       : 28;/* Reserved              */
} I3C_DRIVER_STATE;

#if I3C_DMA_ENABLE
typedef struct _I3C_DMA_HW_CONFIG
{
  DMA_PERIPHERAL_CONFIG dma_tx; /* DMA Tx interface */
  DMA_PERIPHERAL_CONFIG dma_rx; /* DMA Rx interface */
} I3C_DMA_HW_CONFIG;
#endif

/**
\brief I3C Device Resources
*/
typedef struct _I3C_RESOURCES
{
  I3C_Type              *regs;             /* Pointer to i3c regs                                */
  ARM_I3C_SignalEvent_t  cb_event;         /* Pointer to call back function                      */

#if I3C_DMA_ENABLE
  ARM_DMA_SignalEvent_t  dma_cb;           /* Pointer to DMA  Callback                           */
#endif

  uint32_t               core_clk;         /* i3c core clock frequency                           */
  uint32_t               datp;             /* DAT (Device Address Table) offset                  */
  uint32_t               maxdevs;          /* maximum number of slaves supported                 */
  uint8_t                addrs[MAX_DEVS];  /* Assigned dynamic(i3c) or static address(i2c slave) */
  uint32_t               freepos;          /* bitmask of used addresses                          */
  I3C_XFER               xfer;             /* i3c transfer structure                             */
  ARM_I3C_STATUS         status;           /* i3c driver status                                  */
  I3C_DRIVER_STATE       state;            /* I3C driver state                                   */
  IRQn_Type              irq;              /* i3c interrupt number                               */
  uint32_t               irq_priority;     /* i3c interrupt priority                             */

#if I3C_DMA_ENABLE
  I3C_DMA_HW_CONFIG     *dma_cfg;          /* DMA Controller configuration                       */
  const uint32_t         dma_irq_priority; /* DMA IRQ priority number                            */
#endif

}I3C_RESOURCES;


#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_I3C_PRIVATE_H */
