/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/* System Includes */
#include <string.h>
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* Project Includes */
#include "Driver_I3C.h"
#include "Driver_I3C_Private.h"
#include "i3c.h"
#include "sys_ctrl_i3c.h"

#if !(RTE_I3C)
#error "I3C is not enabled in the RTE_Device.h"
#endif

#if (defined(RTE_Drivers_I3C) && !RTE_I3C)
#error "I3C not configured in RTE_Device.h!"
#endif

#define ARM_I3C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
  ARM_I3C_API_VERSION,
  ARM_I3C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I3C_CAPABILITIES DriverCapabilities =
{
  1,   /* Supports legacy i2c device */
  0    /* Reserved (must be zero)    */
};


#if I3C_DMA_ENABLE
/**
  \fn          int32_t I3C_DMA_Initialize(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       Initialize DMA for I3C
  \param[in]   dma_periph   Pointer to DMA resources
  \return      \ref         execution_status
*/
__STATIC_INLINE int32_t I3C_DMA_Initialize(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t        status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Initializes DMA interface */
    status = dma_drv->Initialize();
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I3C_DMA_PowerControl(ARM_POWER_STATE state,
                                            DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       PowerControl DMA for I3C
  \param[in]   state  Power state
  \param[in]   dma_periph     Pointer to DMA resources
  \return      \ref execution_status
*/
__STATIC_INLINE int32_t I3C_DMA_PowerControl(ARM_POWER_STATE state,
                                             DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t        status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Initializes DMA interface */
    status = dma_drv->PowerControl(state);
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I3C_DMA_Allocate(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       Allocate a channel for I3C
  \param[in]   dma_periph  Pointer to DMA resources
  \return      \ref        execution_status
*/
__STATIC_INLINE int32_t I3C_DMA_Allocate(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t        status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Allocate handle for peripheral */
    status = dma_drv->Allocate(&dma_periph->dma_handle);
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Enable the channel in the Event Router */
    if(dma_periph->evtrtr_cfg.instance == 0)
    {
        evtrtr0_enable_dma_channel(dma_periph->evtrtr_cfg.channel,
                                   dma_periph->evtrtr_cfg.group,
                                   DMA_ACK_COMPLETION_PERIPHERAL);
        evtrtr0_enable_dma_handshake(dma_periph->evtrtr_cfg.channel,
                                     dma_periph->evtrtr_cfg.group);
    }
    else
    {
        evtrtrlocal_enable_dma_channel(dma_periph->evtrtr_cfg.channel,
                                       DMA_ACK_COMPLETION_PERIPHERAL);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I3C_DMA_DeAllocate(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       De-allocate channel of I3C
  \param[in]   dma_periph  Pointer to DMA resources
  \return      \ref        execution_status
*/
__STATIC_INLINE int32_t I3C_DMA_DeAllocate(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t        status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* De-Allocate handle  */
    status = dma_drv->DeAllocate(&dma_periph->dma_handle);
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Disable the channel in the Event Router */
    if(dma_periph->evtrtr_cfg.instance == 0)
    {
        evtrtr0_disable_dma_channel(dma_periph->evtrtr_cfg.channel);
        evtrtr0_disable_dma_handshake(dma_periph->evtrtr_cfg.channel,
                                      dma_periph->evtrtr_cfg.group);
    }
    else
    {
        evtrtrlocal_disable_dma_channel(dma_periph->evtrtr_cfg.channel);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I3C_DMA_Start(DMA_PERIPHERAL_CONFIG *dma_periph,
                                     ARM_DMA_PARAMS *dma_params)
  \brief       Start I3C DMA transfer
  \param[in]   dma_periph     Pointer to DMA resources
  \param[in]   dma_params     Pointer to DMA parameters
  \return      \ref           execution_status
*/
__STATIC_INLINE int32_t I3C_DMA_Start(DMA_PERIPHERAL_CONFIG *dma_periph,
                                      ARM_DMA_PARAMS *dma_params)
{
    int32_t        status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Start transfer */
    status = dma_drv->Start(&dma_periph->dma_handle, dma_params);
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I3C_DMA_Stop(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       Stop I3C DMA transfer
  \param[in]   dma_periph   Pointer to DMA resources
  \return      \ref         execution_status
*/
__STATIC_INLINE int32_t I3C_DMA_Stop(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t        status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Stop transfer */
    status = dma_drv->Stop(&dma_periph->dma_handle);
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I3C_DMA_Start_TX(I3C_RESOURCES *i3c,
                                        const void    *src_addr,
                                        uint32_t       len)
  \brief       Start sending data to I3C TX FIFO using DMA
  \param[in]   i3c      : Pointer to i3c resources structure
  \param[in]   src_addr : Pointer to source address
  \param[in]   len      : number of bytes
  \return      execution_status
*/
static int32_t I3C_DMA_Start_TX(I3C_RESOURCES *i3c,
                                const void    *src_addr,
                                uint32_t       len)
{
    int32_t        status;
    ARM_DMA_PARAMS dma_params;

    /* Start the DMA engine for sending the data to I3C */
    dma_params.peri_reqno    = (int8_t)i3c->dma_cfg->dma_tx.dma_periph_req;
    dma_params.dir           = ARM_DMA_MEM_TO_DEV;
    dma_params.cb_event      = i3c->dma_cb;
    dma_params.src_addr      = src_addr;
    dma_params.dst_addr      = i3c_get_dma_tx_addr(i3c->regs);

    dma_params.num_bytes     = len;
    /* i3c TX/RX FIFO is 4-byte(word) aligned,
     *  if length is not 4-byte aligned(multiple of 4),
     *   make it aligned by adding extra length.
     */
    if(len % 4)
    {
        dma_params.num_bytes += (4 - (len % 4));
    }

    dma_params.irq_priority  = i3c->dma_irq_priority;

    /* i3c TX/RX FIFO is 4-byte(word) aligned */
    dma_params.burst_size = BS_BYTE_4;
    dma_params.burst_len  = i3c_get_tx_empty_buf_thld(i3c->regs);
    if( dma_params.burst_len > 16)
    {
        dma_params.burst_len = 16;
    }

    /* Start DMA transfer */
    status = I3C_DMA_Start(&i3c->dma_cfg->dma_tx, &dma_params);
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I3C_DMA_Start_RX(I3C_RESOURCES *i3c,
                                        void          *dst_addr
                                        uint32_t       len)
  \brief       Start receiving data from I3C RX FIFO using DMA
  \param[in]   i3c      : Pointer to i3c resources structure
  \param[in]   dst_addr : Pointer to destination address
  \param[in]   len      : number of bytes
  \return      execution_status
*/
static int32_t I3C_DMA_Start_RX(I3C_RESOURCES *i3c,
                                void          *dst_addr,
                                uint32_t       len)
{
    ARM_DMA_PARAMS dma_params;
    int32_t        status;

    /* Start the DMA engine for sending the data to i3c */
    dma_params.peri_reqno    = (int8_t)i3c->dma_cfg->dma_rx.dma_periph_req;
    dma_params.dir           = ARM_DMA_DEV_TO_MEM;
    dma_params.cb_event      = i3c->dma_cb;
    dma_params.src_addr      = i3c_get_dma_rx_addr(i3c->regs);
    dma_params.dst_addr      = dst_addr;

    dma_params.num_bytes     = len;
    /* i3c TX/RX FIFO is 4-byte(word) aligned,
     *  if length is not 4-byte aligned(multiple of 4),
     *   make it aligned by adding extra length.
     */
    if(len % 4)
    {
        dma_params.num_bytes += (4 - (len % 4));
    }

    dma_params.irq_priority  = i3c->dma_irq_priority;

    /* i3c TX/RX FIFO is 4-byte(word) aligned */
    dma_params.burst_size = BS_BYTE_4;
    dma_params.burst_len  = i3c_get_rx_buf_thld(i3c->regs);
    if( dma_params.burst_len > 16)
    {
        dma_params.burst_len = 16;
    }

    /* Start DMA transfer */
    status = I3C_DMA_Start(&i3c->dma_cfg->dma_rx, &dma_params);
    if(status)
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}
#endif /* I3C_DMA_ENABLE */


/**
  \fn           int I3cMasterGetAddrPos(I3C_RESOURCES *i3c, uint8_t addr)
  \brief        Get already assigned slave address position from DAT(Device Address Table).
                For i3c, dynamic address and for i2c, static address is used for communication.
  \param[in]    i3c     : Pointer to i3c resources structure
  \param[in]    addr    : Assigned Slave Address;
                           Dynamic Address for i3c, Static Address for i2c slave device
  \return       Assigned slave address position from DAT(Device Address Table) index OR
                ARM_DRIVER_ERROR in case slave is not already assigned.
*/
static int I3cMasterGetAddrPos(I3C_RESOURCES *i3c, uint8_t addr)
{
  uint32_t pos;

  for (pos = 0; pos < i3c->maxdevs; pos++)
  {
    if (addr == i3c->addrs[pos])
      return pos;
  }

  return ARM_DRIVER_ERROR;
}

/**
  \fn           int I3cMasterGetFreePos(I3C_RESOURCES *i3c)
  \brief        Get free position from DAT(Device Address Table)
  \param[in]    i3c     : Pointer to i3c resources structure
  \return       Free position from DAT OR
                ARM_DRIVER_ERROR in case DAT is Full.
                Maximum 8 Slave Devices are supported (\ref register DEVICE_ADDR_TABLE_POINTER)
*/
static int I3cMasterGetFreePos(I3C_RESOURCES *i3c)
{
  uint32_t i;

  if (!(i3c->freepos & GENMASK(i3c->maxdevs - 1, 0)))
    return ARM_DRIVER_ERROR;

  for (i = 0; i < i3c->maxdevs; i++)
  {
    if (i3c->freepos & (1 << i))
      return i;
  }

  return ARM_DRIVER_ERROR;
}

/**
  \fn           ARM_DRIVER_VERSION I3C_GetVersion(void)
  \brief        Get i3c driver version
  \return       i3c driver version
*/
static ARM_DRIVER_VERSION I3C_GetVersion(void)
{
  return DriverVersion;
}

/**
  \fn           ARM_I3C_CAPABILITIES I3C_GetCapabilities(void)
  \brief        Get i3c driver capabilities
  \return       i3c driver capabilities
*/
static ARM_I3C_CAPABILITIES I3C_GetCapabilities(void)
{
  return DriverCapabilities;
}

/**
  \fn           int I3Cx_MasterSendCommand(I3C_RESOURCES *i3c, I3C_CMD *ccc)
  \brief        Send an I3C command to the slave
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    ccc      : Pointer to i3c command structure
  \return       \ref execution_status
*/
static int I3Cx_MasterSendCommand(I3C_RESOURCES *i3c, I3C_CMD *ccc)
{
  int32_t index;

#if I3C_DMA_ENABLE
  int32_t ret;
#endif

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  if (!ccc)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (ccc->cmd_id == I3C_CCC_ENTDAA)
    return ARM_DRIVER_ERROR_PARAMETER;

  index = I3cMasterGetAddrPos(i3c, ccc->addr);
  if (index < 0)
    return ARM_DRIVER_ERROR;

  i3c->status.busy = 1;

  if (ccc->rw) /* command read */
  {

#if (!I3C_DMA_ENABLE) /* update only if DMA disable */
    i3c->xfer.tx_buf = NULL;
    i3c->xfer.tx_len = 0;
    i3c->xfer.rx_buf = ccc->data;
    i3c->xfer.rx_len = ccc->len;
#endif

    i3c_ccc_get(i3c->regs, &(i3c->xfer), index, ccc->cmd_id, ccc->len);

#if I3C_DMA_ENABLE
    ret = I3C_DMA_Start_RX(i3c, ccc->data, ccc->len);
    if(ret)
    {
      return ARM_DRIVER_ERROR;
    }
#endif

  }    /* command read  */
  else /* command write */
  {

#if (!I3C_DMA_ENABLE) /* update only if DMA disable */
    i3c->xfer.rx_buf = NULL;
    i3c->xfer.rx_len = 0;
    i3c->xfer.tx_buf = ccc->data;
    i3c->xfer.tx_len = ccc->len;
#endif

    i3c_ccc_set(i3c->regs, &(i3c->xfer), index, ccc->cmd_id, ccc->len);

#if I3C_DMA_ENABLE
    ret = I3C_DMA_Start_TX(i3c, ccc->data, ccc->len);
    if(ret)
    {
      return ARM_DRIVER_ERROR;
    }
#endif

  }

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_MasterTransmit(I3C_RESOURCES *i3c,  uint8_t  addr,
                                        const uint8_t *data, uint16_t len)
  \brief        Write data to the slave
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    addr     : Assigned Slave Address;
                            Dynamic Address for i3c, Static Address for i2c slave device
  \param[in]    data     : Pointer to buffer with data which needs to be transmit to slave
  \param[in]    len      : Number of bytes needs to be transmit
  \return       \ref execution_status
*/
static int I3Cx_MasterTransmit(I3C_RESOURCES *i3c,  uint8_t  addr,
                               const uint8_t *data, uint16_t len)
{
  int32_t index;

#if I3C_DMA_ENABLE
  int32_t ret;
#endif

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->state.master_enabled == 0U)
      return ARM_DRIVER_ERROR;

  if (!data || !len)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  index = I3cMasterGetAddrPos(i3c, addr);
  if (index < 0)
    return ARM_DRIVER_ERROR_PARAMETER;

  i3c->status.busy = 1;

#if (!I3C_DMA_ENABLE) /* update only if DMA disable */
  i3c->xfer.rx_buf = NULL;
  i3c->xfer.rx_len = 0;
  i3c->xfer.tx_buf = data;
  i3c->xfer.tx_len = len;
#endif

  i3c_master_tx(i3c->regs, &(i3c->xfer), index, len);

#if I3C_DMA_ENABLE
  ret = I3C_DMA_Start_TX(i3c, data, len);
  if(ret)
  {
    return ARM_DRIVER_ERROR;
  }
#endif

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_MasterReceive(I3C_RESOURCES *i3c,  uint8_t  addr,
                                             uint8_t *data, uint16_t len)
  \brief        Read data from the slave
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    addr     : Assigned Slave Address;
                            Dynamic Address for i3c, Static Address for i2c slave device
  \param[in]    data     : Pointer to buffer for data to receive from slave
  \param[in]    len      : Number of bytes needs to be receive
  \return       \ref execution_status
*/
static int I3Cx_MasterReceive(I3C_RESOURCES *i3c,  uint8_t  addr,
                                    uint8_t *data, uint16_t len)
{
  int32_t index;

#if I3C_DMA_ENABLE
  int32_t ret;
#endif

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->state.master_enabled == 0U)
      return ARM_DRIVER_ERROR;

  if (!data || !len)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  index = I3cMasterGetAddrPos(i3c, addr);
  if (index < 0)
    return ARM_DRIVER_ERROR_PARAMETER;

  i3c->status.busy = 1;

#if (!I3C_DMA_ENABLE) /* update only if DMA disable */
  i3c->xfer.rx_buf = data;
  i3c->xfer.rx_len = len;
  i3c->xfer.tx_buf = NULL;
  i3c->xfer.tx_len = 0;
#endif

  i3c_master_rx(i3c->regs, &(i3c->xfer), index, len);

#if I3C_DMA_ENABLE
  ret = I3C_DMA_Start_RX(i3c, data, len);
  if(ret)
  {
    return ARM_DRIVER_ERROR;
  }
#endif

  return ARM_DRIVER_OK;
}

/*
 * Note for I3C DMA:
 * For proper Master and Slave communication,
 *  There should be fix protocol between Master and Slave,
 *  in which both should be knowing well in advanced that
 *  how much data is going to transmit/receive from both the sides.
 *  (currently different-different transfer data length from
 *   Master and Slave is not supported.)
 */

/**
  \fn           int I3Cx_SlaveTransmit(I3C_RESOURCES *i3c,
                                       const uint8_t *data,
                                       uint16_t       len)
  \brief        Write data to the master
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    data     : Pointer to buffer with data
                            which needs to be transmit to master
  \param[in]    len      : Number of bytes needs to be transmit
  \return       \ref execution_status
*/
static int I3Cx_SlaveTransmit(I3C_RESOURCES *i3c,
                              const uint8_t *data,
                              uint16_t       len)
{
#if I3C_DMA_ENABLE
  int32_t ret;
#endif

  /* Checking for power done initialization */
  if (i3c->state.powered == 0U)
      return ARM_DRIVER_ERROR;

  /* Checking for slave initialization */
  if (i3c->state.slave_enabled == 0U)
      return ARM_DRIVER_ERROR;

  /* Parameter check */
  if (!data || !len)
      return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
      return ARM_DRIVER_ERROR_BUSY;

  i3c->status.busy = 1;

#if (!I3C_DMA_ENABLE) /* update only if DMA disable */
  i3c->xfer.tx_buf = data;
  i3c->xfer.tx_len = len;
  i3c->xfer.rx_buf = NULL;
  i3c->xfer.rx_len = 0;
#endif

  i3c_slave_tx(i3c->regs, &(i3c->xfer), len);

#if I3C_DMA_ENABLE
  ret = I3C_DMA_Start_TX(i3c, data, len);
  if(ret)
  {
    return ARM_DRIVER_ERROR;
  }
#endif

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_SlaveReceive(I3C_RESOURCES *i3c,
                                      uint8_t       *data,
                                      uint32_t       len)
  \brief        Read data from the master
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    data     : Pointer to buffer for data
                            to receive from master
  \param[in]    len      : Number of bytes needs to be receive
  \return       \ref execution_status
*/
static int I3Cx_SlaveReceive(I3C_RESOURCES *i3c,
                             uint8_t       *data,
                             uint32_t       len)
{
#if I3C_DMA_ENABLE
  int32_t ret;
#endif

  /* Checking for power done initialization */
  if (i3c->state.powered == 0U)
      return ARM_DRIVER_ERROR;

  if (i3c->state.slave_enabled == 0U)
      return ARM_DRIVER_ERROR;

  /* Parameter check */
  if (!data || !len)
      return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
      return ARM_DRIVER_ERROR_BUSY;

  i3c->status.busy = 1;

#if (!I3C_DMA_ENABLE) /* update only if DMA disable */
  /* Buffer initialization for TX/RX */
  i3c->xfer.rx_buf = data;
  i3c->xfer.rx_len = len;
  i3c->xfer.tx_buf = NULL;
  i3c->xfer.tx_len = 0;
#endif

  i3c_slave_rx(i3c->regs, &(i3c->xfer));

#if I3C_DMA_ENABLE
  ret = I3C_DMA_Start_RX(i3c, data, len);
  if(ret)
  {
    return ARM_DRIVER_ERROR;
  }
#endif

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_MasterAssignDA(I3C_RESOURCES *i3c,
                                        uint8_t       *dyn_addr,
                                        uint8_t        sta_addr)
  \brief        Assign dynamic address to the i3c slave using SETDASA;
                Note: Only required for i3c slave devices;
                      i2c slave device uses static address
                      for communication \ref I3Cx_AttachI2Cdev.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    dyn_addr : Pointer to char where dynamic address
                            will be copied.
  \param[in]    sta_addr : Static address of i3c device
  \return       \ref execution_status
*/
static int I3Cx_MasterAssignDA(I3C_RESOURCES *i3c,
                               uint8_t       *dyn_addr,
                               uint8_t        sta_addr)
{
  int32_t  pos;

  if (!dyn_addr || !sta_addr)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  i3c->status.busy = 1;

  /* Find the first unused index in freepos, note that this also
   * corresponds to the first unused location in the DAT
   */
  pos = I3cMasterGetFreePos(i3c);

  /* the dat is full */
  if (pos < 0) {
    i3c->status.busy = 0;
    return ARM_DRIVER_ERROR;
  }

  /* reserve the index */
  i3c->freepos &= ~(BIT(pos));
  *dyn_addr = pos + 0x09; /* we start assigning addresses from 0x09 */
  i3c->addrs[pos] = *dyn_addr;

  /* ok, we have space in the dat, program the dat in index pos */
  i3c_add_slv_to_dat(i3c->regs, pos, *dyn_addr, sta_addr);

  i3c_send_ccc_cmd(i3c->regs, I3C_CCC_SETDASA, pos);

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_AttachI2Cdev(I3C_RESOURCES *i3c, uint8_t sta_addr)
  \brief        Attach legacy i2c device to the i3c bus.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    sta_addr : Static address of legacy i2c device
  \return       \ref execution_status
*/
static int I3Cx_AttachI2Cdev(I3C_RESOURCES *i3c, uint8_t sta_addr)
{
  int32_t  pos;

  if (!sta_addr)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  /* Find the first unused index in freepos, note that this also
   * corresponds to the first unused location in the DAT(Device Address Table)
   */
  pos = I3cMasterGetFreePos(i3c);

  /* DAT(Device Address Table) is full? */
  if (pos < 0)
  {
    /* error: DAT is full */
    return ARM_DRIVER_ERROR;
  }

  /* reserve the index */
  i3c->freepos &= ~(BIT(pos));

  /* use static address for communication. */
  i3c->addrs[pos] = sta_addr;

  /* ok, we have space in the DAT, store the static address and
   * mark as i2c legacy device is present.
   */

  /* Program the DAT(device address table) in index pos. */
  /* for i2c slave dynamic address is not available. */
  i3c_add_slv_to_dat(i3c->regs, pos, 0, sta_addr);

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_Detachdev(I3C_RESOURCES *i3c, uint8_t addr)
  \brief        Detach already attached i2c/i3c device from the i3c bus.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    addr     : Static  address of already attached i2c device
                                        OR
                           Dynamic address of already attached i3c device
  \return       \ref execution_status
*/
static int I3Cx_Detachdev(I3C_RESOURCES *i3c, uint8_t addr)
{
  int32_t  pos;

  if (!addr)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  /* Get already attached i2c device address index in
   * DAT (device address table). */
  pos = I3cMasterGetAddrPos(i3c, addr);

  /* i2c i3c is not attached to DAT? */
  if (pos < 0)
  {
    /* err: i2c slave device is not attached to DAT,
     * first attach i2c device \ref I3Cx_AttachI2Cdev */
    return ARM_DRIVER_ERROR;
  }

  /* free the index */
  i3c->freepos |= (BIT(pos));
  i3c->addrs[pos] = 0;

  /* clear the DAT index pos */
  i3c_remove_slv_from_dat(i3c->regs, pos);

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_Control(I3C_RESOURCES *i3c,
                                     uint32_t       control,
                                     uint32_t       arg)
  \brief        Control i3c master and slave.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    control  : Operation
  \param[in]    arg      : Argument of operation
  \return       \ref execution_status
*/
static int32_t I3Cx_Control(I3C_RESOURCES *i3c,
                            uint32_t       control,
                            uint32_t       arg)
{
  I3C_I2C_SPEED_MODE i2c_speed_mode = 0;
  uint8_t slv_addr = 0;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  switch(control)
  {
  case I3C_MASTER_SET_BUS_MODE:

    i3c->datp    = i3c_get_dat_addr(i3c->regs);
    i3c->maxdevs = i3c_get_dat_depth(i3c->regs);
    i3c->freepos = GENMASK(i3c->maxdevs - 1, 0);

    i3c_master_init(i3c->regs);

    /* set state as master enabled. */
    i3c->state.master_enabled = 1;

    switch(arg)
    {
    case I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS:
    case I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS:
    case I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS:
    case I3C_BUS_MODE_MIXED_LIMITED:

      if(arg == I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_FMP_1_MBPS;

      if(arg == I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_FM_400_KBPS;

      if(arg == I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_SS_100_KBPS;

      if(arg == I3C_BUS_MODE_MIXED_LIMITED)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_LIMITED;

      if (!(i3c->core_clk))
        return ARM_DRIVER_ERROR;

      /* i2c clock configuration for selected Speed mode. */
      i2c_clk_cfg(i3c->regs, i3c->core_clk, i2c_speed_mode);

    /* fall through */
    case I3C_BUS_MODE_PURE:

      if (!(i3c->core_clk))
        return ARM_DRIVER_ERROR;

      /* i3c clock configuration */
      i3c_clk_cfg(i3c->regs, i3c->core_clk);
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    break;

  case I3C_SLAVE_SET_ADDR:

      i3c->datp    = i3c_get_dat_addr(i3c->regs);
      i3c->maxdevs = i3c_get_dat_depth(i3c->regs);
      i3c->freepos = GENMASK(i3c->maxdevs - 1, 0);

      /* Initialize and Enable i3c Slave */
      slv_addr = arg;
      i3c_slave_init(i3c->regs, slv_addr);

      /* set state as slave enabled. */
      i3c->state.slave_enabled = 1;
      break;

  default:
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_Initialize(I3C_RESOURCES         *i3c,
                                        ARM_I3C_SignalEvent_t  cb_event)
  \brief        Initialize the i3c device.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    cb_event : Pointer to I3C Event
                            \ref ARM_I3C_SignalEvent_t
  \return       \ref execution_status
*/
static int32_t I3Cx_Initialize(I3C_RESOURCES         *i3c,
                               ARM_I3C_SignalEvent_t  cb_event)
{
  if (i3c->state.initialized == 1)
    return ARM_DRIVER_OK;

  if(!cb_event)
    return ARM_DRIVER_ERROR_PARAMETER;

  /* set the user callback event. */
  i3c->cb_event = cb_event;

#if I3C_DMA_ENABLE
  i3c->dma_cfg->dma_rx.dma_handle = -1;
  i3c->dma_cfg->dma_tx.dma_handle = -1;

  /* Initialize DMA for I3C-Tx */
  if(I3C_DMA_Initialize(&i3c->dma_cfg->dma_tx) != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Initialize DMA for I3C-Rx */
  if(I3C_DMA_Initialize(&i3c->dma_cfg->dma_rx) != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;
#endif

  i3c->core_clk = GetSystemAPBClock();

  /* set the state as initialized. */
  i3c->state.initialized = 1;
  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_Uninitialize(I3C_RESOURCES  *i3c)
  \brief        Uninitialize the i3c device
  \param[in]    i3c      : Pointer to i3c resources structure
  \return       \ref execution_status
*/
static int32_t I3Cx_Uninitialize(I3C_RESOURCES  *i3c)
{
#if I3C_DMA_ENABLE
  i3c->dma_cfg->dma_rx.dma_handle = -1;
  i3c->dma_cfg->dma_tx.dma_handle = -1;
#endif

  i3c->state.initialized = 0;
  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_PowerControl(I3C_RESOURCES   *i3c,
                                          ARM_POWER_STATE  state)
  \brief        i3c power control
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    state    : Power state
  \return       \ref execution_status
*/
static int32_t I3Cx_PowerControl(I3C_RESOURCES   *i3c,
                                 ARM_POWER_STATE  state)
{
  switch (state)
  {
    case ARM_POWER_OFF:

      /* Disable i3c IRQ */
      NVIC_DisableIRQ(i3c->irq);

      /* Clear Any Pending i3c IRQ */
      NVIC_ClearPendingIRQ(i3c->irq);

#if I3C_DMA_ENABLE
      /* Disable i3c DMA */
      i3c_dma_disable(i3c->regs);

      /* Deallocate DMA channel for Tx */
      if(I3C_DMA_DeAllocate(&i3c->dma_cfg->dma_tx))
        return ARM_DRIVER_ERROR;

      /* Deallocate DMA channel for Rx */
      if(I3C_DMA_DeAllocate(&i3c->dma_cfg->dma_rx))
        return ARM_DRIVER_ERROR;
#endif /* I3C_DMA_ENABLE */

      /* i3c EXPMST0 control configuration:
       *  Disable i3c clock. */
      disable_i3c_clock();

      /* Reset the power state. */
      i3c->state.powered = 0;
      break;

    case ARM_POWER_FULL:

      if (i3c->state.initialized == 0U)
        return ARM_DRIVER_ERROR;

      if (i3c->state.powered)
        break;

      /* i3c EXPMST0 control configuration:
       *  Enable i3c clock. */
      enable_i3c_clock();

#if I3C_DMA_ENABLE
      /* if DMA2 is selected? */
      if(i3c->dma_cfg->dma_tx.evtrtr_cfg.instance == 2)
      {
        select_i3c_dma2();
      }
      /* else: default DMA0 is selected. */

      /* Enable i3c DMA */
      i3c_dma_enable(i3c->regs);
#endif /* I3C_DMA_ENABLE */

      /* Enable i3c IRQ */
      NVIC_ClearPendingIRQ(i3c->irq);
      NVIC_SetPriority(i3c->irq, i3c->irq_priority);
      NVIC_EnableIRQ(i3c->irq);

      /* Set the state as powered */
      i3c->state.powered = 1;
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

#if I3C_DMA_ENABLE
  /* Power Control DMA for I3C-Tx */
  if(I3C_DMA_PowerControl(state, &i3c->dma_cfg->dma_tx) != ARM_DRIVER_OK)
  {
    i3c->state.powered = 0;
    return ARM_DRIVER_ERROR;
  }

  /* Power Control DMA for I3C-Rx */
  if(I3C_DMA_PowerControl(state, &i3c->dma_cfg->dma_rx) != ARM_DRIVER_OK)
  {
    i3c->state.powered = 0;
    return ARM_DRIVER_ERROR;
  }

  if(state == ARM_POWER_FULL)
  {
    /* Try to allocate a DMA channel for TX */
    if(I3C_DMA_Allocate(&i3c->dma_cfg->dma_tx))
    {
      i3c->state.powered = 0;
      return ARM_DRIVER_ERROR;
    }

    /* Try to allocate a DMA channel for RX */
    if(I3C_DMA_Allocate(&i3c->dma_cfg->dma_rx))
    {
      i3c->state.powered = 0;
      return ARM_DRIVER_ERROR;
    }
  }
#endif /* I3C_DMA_ENABLE */

  return ARM_DRIVER_OK;
}


#if I3C_DMA_ENABLE
/**
  \fn          static void  I3Cx_DMACallback(uint32_t event, int8_t peri_num,
                                            I3C_RESOURCES *i3c)
  \brief       Callback function from DMA for I3C
  \param[in]   event     Event from DMA
  \param[in]   peri_num  Peripheral number
  \param[in]   I3C       Pointer to I3C resources
  \return      none
*/
static void I3Cx_DMACallback(uint32_t event, int8_t peri_num,
                            I3C_RESOURCES *i3c)
{
    if(!i3c->cb_event)
        return;

    /* Transfer Completed */
    if(event & ARM_DMA_EVENT_COMPLETE)
    {
        switch(peri_num)
        {
        case I3C_DMA_TX_PERIPH_REQ:
          /* For DMA TX,
           *  Success/Error decision will be taken by
           *   Interrupt Handler based on status of Response-Queue.
           *   (as this callback will be always called
           *    irrespective of slave gives ACK/NACK.)
           */
          break;

        case I3C_DMA_RX_PERIPH_REQ:
          /* For DMA RX,
           *  Success decision will be taken here(DMA RX Callback).
           *  Error decision will be taken by Interrupt Handler
           *   based on status of Response-Queue.
           */

           /* clear transfer status. */
           i3c->xfer.status = I3C_XFER_STATUS_NONE;

           /* clear busy flag. */
           i3c->status.busy = 0;

           /* Mark event as success and call the user callback */
           i3c->cb_event(ARM_I3C_EVENT_TRANSFER_DONE);
           break;

        default:
           break;
        }
    }

    /* Abort Occurred */
    if(event & ARM_DMA_EVENT_ABORT)
    {

    }
}
#endif /* RTE_I3C_DMA_ENABLE */


/* I3C Driver Instance */
#if (RTE_I3C)

#if RTE_I3C_DMA_ENABLE
static void I3C_DMACallback(uint32_t event, int8_t peri_num);
static I3C_DMA_HW_CONFIG I3Cx_DMA_HW_CONFIG =
{
    .dma_rx =
    {
        .dma_drv        = &ARM_Driver_DMA_(I3C_DMA),
        .dma_periph_req = I3C_DMA_RX_PERIPH_REQ,
        .evtrtr_cfg =
        {
             .instance = I3C_DMA,
             .group    = I3C_DMA_GROUP,
             .channel  = I3C_DMA_RX_PERIPH_REQ,
             .enable_handshake = I3C_DMA_HANDSHAKE_ENABLE,
        },
    },
    .dma_tx =
    {
        .dma_drv        = &ARM_Driver_DMA_(I3C_DMA),
        .dma_periph_req = I3C_DMA_TX_PERIPH_REQ,
        .evtrtr_cfg =
        {
             .instance = I3C_DMA,
             .group    = I3C_DMA_GROUP,
             .channel  = I3C_DMA_TX_PERIPH_REQ,
             .enable_handshake = I3C_DMA_HANDSHAKE_ENABLE,
        },

    },
};
#endif /* RTE_I3C_DMA_ENABLE */

/* I3C Device Resources */
static I3C_RESOURCES i3c =
{
  .regs         = (I3C_Type *)I3C_BASE,
  .cb_event     = NULL,
  .xfer         = {0},
  .status       = {0},
  .state        = {0},
  .irq          = (IRQn_Type) I3C_IRQ_IRQn,
  .irq_priority = RTE_I3C_IRQ_PRI,

#if RTE_I3C_DMA_ENABLE
  .dma_cb            = I3C_DMACallback,
  .dma_cfg           = &I3Cx_DMA_HW_CONFIG,
  .dma_irq_priority  = RTE_I3C_DMA_IRQ_PRI,
#endif

};

#if RTE_I3C_DMA_ENABLE
/**
  \fn          static void  I3C_DMACallback (uint32_t event, int8_t peri_num)
  \param[in]   event     Event from DMA
  \param[in]   peri_num  Peripheral number
  \brief       Callback function from DMA for I3C
*/
static void I3C_DMACallback(uint32_t event, int8_t peri_num)
{
    I3Cx_DMACallback(event, peri_num, &i3c);
}
#endif /* RTE_I3C_DMA_ENABLE */

void I3C_IRQHandler(void)
{
  I3C_XFER *xfer = &(i3c.xfer);
  uint32_t event = 0;

  i3c_irq_handler(i3c.regs, xfer);

  /* check status: Transfer Error? */
  if(xfer->status & I3C_XFER_STATUS_ERROR)
  {
    /* error: Resume i3c controller and
     *        clear error status. */
    i3c_resume(i3c.regs);
    i3c_clear_xfer_error(i3c.regs);

#if RTE_I3C_DMA_ENABLE
    /* Stop DMA TX transfer */
    if(xfer->status & I3C_XFER_STATUS_ERROR_TX)
    {
      I3C_DMA_Stop(&i3c.dma_cfg->dma_tx);
    }

    /* Stop DMA RX transfer */
    if(xfer->status & I3C_XFER_STATUS_ERROR_RX)
    {
      I3C_DMA_Stop(&i3c.dma_cfg->dma_rx);
    }
#endif /* RTE_I3C_DMA_ENABLE */

    /* mark event as Transfer Error. */
    event = ARM_I3C_EVENT_TRANSFER_ERROR;
  } /* if I3C_XFER_STATUS_ERROR */

  /* check status: Transfer Success? */
  else if(xfer->status & I3C_XFER_STATUS_DONE)
  {
    /* mark event as Transfer done. */
    event = ARM_I3C_EVENT_TRANSFER_DONE;

#if RTE_I3C_DMA_ENABLE
    /* DMA TX,
     *  Success/Error decision will be taken by
     *   Interrupt Handler based on status of Response-Queue.
     *
     * DMA RX,
     *   Success decision will be taken in DMA_RX callback.
     *   Error decision will be taken by Interrupt Handler
     *    based on status of Response-Queue.
     */
    if( (xfer->status & I3C_XFER_STATUS_MST_RX_DONE) ||
        (xfer->status & I3C_XFER_STATUS_SLV_RX_DONE) ||
        (xfer->status & I3C_XFER_STATUS_CCC_GET_DONE) )
    {
        /* for DMA RX Success, mark event as 0. */
        event = 0;

        /* clear transfer status. */
        xfer->status = I3C_XFER_STATUS_NONE;
    }
#endif /* RTE_I3C_DMA_ENABLE */
  }/* else if I3C_XFER_STATUS_DONE*/

  /* check status: Slave dynamic address assign? (only for Slave mode) */
  else if(xfer->status & I3C_XFER_STATUS_SLV_DYN_ADDR_ASSGN)
  {
    /* mark event as Slave dynamic address assignment done. */
    event = ARM_I3C_EVENT_SLV_DYN_ADDR_ASSGN;
  }
  else
  {
    /* control should not come here. */
  }

  if(event)
  {
    /* clear transfer status. */
    xfer->status = I3C_XFER_STATUS_NONE;

    /* clear busy flag. */
    i3c.status.busy = 0;

    /* call the user callback */
    if(i3c.cb_event)
      i3c.cb_event(event);
  }
}

/* wrapper functions for I3C */
static int32_t I3C_Initialize(ARM_I3C_SignalEvent_t cb_event)
{
  return (I3Cx_Initialize(&i3c, cb_event));
}

static int32_t I3C_Uninitialize(void)
{
  return (I3Cx_Uninitialize(&i3c));
}

static int32_t I3C_PowerControl(ARM_POWER_STATE state)
{
  return (I3Cx_PowerControl(&i3c, state));
}

static int32_t I3C_MasterTransmit(uint8_t addr, const uint8_t *data, uint16_t len)
{
  return (I3Cx_MasterTransmit(&i3c, addr, data, len));
}

static int32_t I3C_MasterReceive(uint8_t addr, uint8_t *data, uint16_t len)
{
  return (I3Cx_MasterReceive(&i3c, addr, data, len));
}

static int32_t I3C_SlaveTransmit(const uint8_t *data, uint16_t len)
{
  return (I3Cx_SlaveTransmit(&i3c, data, len));
}

static int32_t I3C_SlaveReceive(uint8_t *data, uint16_t len)
{
  return (I3Cx_SlaveReceive(&i3c, data, len));
}

static int32_t I3C_MasterSendCommand(I3C_CMD *ccc)
{
  return (I3Cx_MasterSendCommand(&i3c, ccc));
}

static int32_t I3C_Control(uint32_t control, uint32_t arg)
{
  return (I3Cx_Control(&i3c, control, arg));
}

static int32_t I3C_MasterAssignDA(uint8_t *dyn_addr, uint8_t sta_addr)
{
  return (I3Cx_MasterAssignDA(&i3c, dyn_addr, sta_addr));
}

static int32_t I3C_AttachI2Cdev(uint8_t sta_addr)
{
  return (I3Cx_AttachI2Cdev(&i3c, sta_addr));
}

static int32_t I3C_Detachdev(uint8_t addr)
{
  return (I3Cx_Detachdev(&i3c, addr));
}


/* I3C Driver Control Block */
extern ARM_DRIVER_I3C Driver_I3C;
ARM_DRIVER_I3C Driver_I3C =
{
  I3C_GetVersion,
  I3C_GetCapabilities,
  I3C_Initialize,
  I3C_Uninitialize,
  I3C_PowerControl,
  I3C_MasterTransmit,
  I3C_MasterReceive,
  I3C_SlaveTransmit,
  I3C_SlaveReceive,
  I3C_Control,
  I3C_MasterSendCommand,
  I3C_MasterAssignDA,
  I3C_AttachI2Cdev,
  I3C_Detachdev
};
#endif /* RTE_I3C */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
