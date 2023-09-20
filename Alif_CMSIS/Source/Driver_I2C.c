/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Includes */
#include <stddef.h>
#include <stdint.h>
#include "stdio.h"

/* system includes */
#include "Driver_I2C.h"
#include "Driver_I2C_Private.h"
#include "i2c.h"

/* Driver version */
#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
    1,  /* supports 10-bit addressing */
    0   /* reserved */
};

/**
 * @brief   get i2c version
 * @note    none
 * @param   none
 * @retval  driver version
 */
static ARM_DRIVER_VERSION ARM_I2C_GetVersion(void)
{
    return DriverVersion;
}

/**
 * @brief   get i2c capabilites
 * @note    none
 * @param   none
 * @retval  driver capabilites
 */
static ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 * @brief   Set i2c Target address
 * @param   I2C  : Pointer to I2C resources structure
 * @param   addr : Target slave address
 * @retval  none
 */
static void I2C_SetTargetAddress(I2C_RESOURCES *I2C, const uint32_t addr)
{
    /* addr is different from current target address */
    if ((addr & (~ARM_I2C_ADDRESS_10BIT)) != I2C->tar_addr)
    {
        if(addr & ARM_I2C_ADDRESS_10BIT)
        {
            I2C->addr_mode = I2C_10BIT_ADDRESS;
        }
        else
        {
            I2C->addr_mode = I2C_7BIT_ADDRESS;
        }
        /* set target address */
        i2c_set_target_addr(I2C->regs, addr, I2C->addr_mode, I2C->transfer.curr_stat);

        I2C->tar_addr = (addr & (~ARM_I2C_ADDRESS_10BIT));
    }
}
/**
 * @brief   get i2c bus speed
 * @note    implemented only ARM_I2C_BUS_SPEED_STANDARD
 * @param   I2C  : Pointer to I2C resources structure
 * @param   i2c_bus_speed    : i2c bus speed
 *          ARM_I2C_BUS_SPEED_STANDARD /
 *          ARM_I2C_BUS_SPEED_FAST     /
 *          ARM_I2C_BUS_SPEED_FAST_PLUS
 * @retval  none
 */
static int32_t I2C_GetBusSpeed(I2C_RESOURCES *I2C, uint32_t i2c_bus_speed)
{
    int32_t speed;

    switch (i2c_bus_speed)
    {
        case ARM_I2C_BUS_SPEED_STANDARD:
            /* Standard Speed (100kHz) */
            speed = I2C_IC_CON_SPEED_STANDARD;
            I2C->speed_mode = I2C_SPEED_STANDARD;
            break;

        case ARM_I2C_BUS_SPEED_FAST:
            /* Fast Speed (400kHz) */
            speed = I2C_IC_CON_SPEED_FAST;
            I2C->speed_mode = I2C_SPEED_FAST;
            break;

        case ARM_I2C_BUS_SPEED_FAST_PLUS:
            /* Fast+ Speed (1MHz) */
            speed = I2C_IC_CON_SPEED_FAST;
            I2C->speed_mode = I2C_SPEED_FASTPLUS;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return speed;
}

/**
 * @brief   CMSIS-Driver i2c initialize
 * @note    it will use interrupt method for data send and receive.
 * @param   cb_event    : Pointer to \ref ARM_I2C_SignalEvent
 * @param   i2c         : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_OK : successfully initialized
 */
static int32_t ARM_I2C_Initialize(ARM_I2C_SignalEvent_t cb_event,
                                   I2C_RESOURCES       *I2C)
{
    /* Driver is already initialized */
    if (I2C->state.initialized == 1)
        return ARM_DRIVER_OK;

    /* default setting */
    I2C->addr_mode = I2C_7BIT_ADDRESS;
    I2C->tar_addr  = I2C_0_TARADDR;

    I2C->tar_addr &= I2C_7BIT_ADDRESS_MASK;
    I2C->slv_addr &= I2C_7BIT_ADDRESS_MASK;

    /* Disable device before init it */
    i2c_disable(I2C->regs);

    I2C->transfer.err_state = I2C_ERR_NONE;
    I2C->transfer.next_cond = I2C_MODE_STOP;

    I2C->transfer.tx_over = 0U;
    I2C->transfer.rx_over = 0;

    /* set the user callback event. */
    I2C->cb_event = cb_event;

    /* set the flag as initialized. */
    I2C->state.initialized = 1;

    return ARM_DRIVER_OK;
}
/**
 * @brief   CMSIS-Driver i2c uninitialize
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_OK : successfully uninitialized
 */
static int32_t ARM_I2C_Uninitialize(I2C_RESOURCES *I2C)
{
    int ret = ARM_DRIVER_OK;

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0)
        return ARM_DRIVER_OK;

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 1)
        return ARM_DRIVER_ERROR;

    i2c_disable(I2C->regs);

    /* initialize all variables to 0 */

    /* initialize the tx_buffer */
    I2C->transfer.tx_buf               = NULL;
    I2C->transfer.tx_total_num         = 0U;
    I2C->transfer.tx_curr_cnt          = 0U;
    I2C->transfer.curr_cnt             = 0U;
    I2C->transfer.tx_over              = 0U;

    /* initialize the rx_buffer */
    I2C->transfer.rx_buf               = NULL;
    I2C->transfer.rx_total_num         = 0U;
    I2C->transfer.rx_curr_cnt          = 0U;
    I2C->transfer.rx_curr_tx_index     = 0U;
    I2C->transfer.rx_over              = 0U;

    I2C->transfer.pending              = 0U;
    I2C->transfer.next_cond            = I2C_MODE_STOP;
    I2C->transfer.err_state            = I2C_ERR_NONE;
    I2C->transfer.curr_stat            = I2C_TRANSFER_NONE;

    /* Clear driver status \ref ARM_I2C_STATUS */
    I2C->status.busy                   = 0U;
    I2C->status.mode                   = 0U;
    I2C->status.direction              = 0U;
    I2C->status.arbitration_lost       = 0U;
    I2C->status.bus_error              = 0U;
    I2C->addr_mode                     = I2C_7BIT_ADDRESS;
    I2C->tar_addr                      = 0U;

    /* Reset the flags. */
    I2C->state.initialized = 0U;

    return ret;
}

/**
 * @func    : CMSIS Driver I2C Power Control
 * @brief   : Power the driver and enable the NVIC
 * @param   : state : Power state
 * @param   : I2C   : Pointer to i2c resources structure
 * @return  : ARM_DRIVER_OK
 */
static int32_t ARM_I2C_PowerControl(ARM_POWER_STATE state, I2C_RESOURCES *I2C)
{
    switch (state)
    {
        case ARM_POWER_FULL:

             /* check for Driver initialization */
             if (I2C->state.initialized == 0)
             {
                 return ARM_DRIVER_ERROR;
             }

             /* check for the power is done before initialization or not */
             if (I2C->state.powered == 1)
             {
                 return ARM_DRIVER_OK;
             }

             /* Disable all interrupts */
             i2c_master_disable_tx_interrupt(I2C->regs);
             i2c_master_disable_rx_interrupt(I2C->regs);
             i2c_slave_disable_tx_interrupt(I2C->regs);
             i2c_slave_disable_rx_interrupt(I2C->regs);

             /* Clear Any Pending Irq */
             NVIC_ClearPendingIRQ(I2C->irq_num);

             /* Set Priority */
             NVIC_SetPriority(I2C->irq_num, I2C->irq_priority);

             /* Enable IRQ */
             NVIC_EnableIRQ(I2C->irq_num);

             i2c_set_tx_threshold(I2C->regs, I2C->tx_fifo_threshold);
             i2c_set_rx_threshold(I2C->regs, I2C->rx_fifo_threshold);

             I2C->state.powered = 1;

            break;
        case ARM_POWER_OFF:

             if (I2C->state.powered == 0)
             {
                 return ARM_DRIVER_OK;
             }
             /* Disabling interrupts */
             if (I2C->mode == I2C_MASTER_MODE)
             {
                i2c_master_disable_tx_interrupt(I2C->regs);
                i2c_master_disable_rx_interrupt(I2C->regs);
             }
             else
             {
                i2c_slave_disable_tx_interrupt(I2C->regs);
                i2c_slave_disable_rx_interrupt(I2C->regs);
             }

             /* Disable the IRQ */
             NVIC_DisableIRQ(I2C->irq_num);

             /* Clearing pending */
             NVIC_ClearPendingIRQ(I2C->irq_num);

             I2C->state.powered = 0;
            break;
        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}


/**
 * @brief   CMSIS-Driver i2c master transmit
 *          Start sending data to i2c transmitter.
 * @note    I2C_FLAG_MASTER_SETUP flag should be enabled first /ref ARM_I2C_BUS_SPEED
 * @param   data : Pointer to buffer with data to send to i2c transmitter
 * @param   num  : Number of data items to send
 * @param   I2C  : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  transmit count              : For data transmit count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_MasterTransmit(I2C_RESOURCES *I2C,
                                      uint32_t        addr,
                                      const uint8_t   *data,
                                      uint32_t        num,
                                      bool            xfer_pending)
{

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0)
        return ARM_DRIVER_ERROR;

    /* addr 7bit addr: 0x7F , 10bit addr: 0x3FF */
    if ((data == NULL) || (num == 0U) || ((addr & (~ARM_I2C_ADDRESS_10BIT)) > 0x3FF))
    {
        /* Invalid parameters */
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (I2C->state.master_setup == 0U)
    {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* set status busy  */
    I2C->status.busy             = 1;

    /* fill the I2C transfer structure as per user detail */
    I2C->transfer.tx_buf        = (uint8_t *)data;
    I2C->transfer.tx_total_num  = num;
    I2C->transfer.tx_curr_cnt   = 0U;
    I2C->transfer.curr_cnt      = 0U;
    I2C->transfer.tx_over       = 0U;
    I2C->transfer.pending       = xfer_pending;
    I2C->transfer.curr_stat     = I2C_TRANSFER_MST_TX;

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.mode             = I2C_MASTER_MODE;
    I2C->status.direction        = I2C_DIR_TRANSMITTER;
    I2C->status.arbitration_lost = 0;
    I2C->status.bus_error        = 0;

    I2C_SetTargetAddress(I2C, addr);

    /* Enabling the tx interrupt */
    i2c_master_enable_tx_interrupt(I2C->regs);

    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c master receive
 *          Start receiving data from i2c receiver.
 * @note    none
 * @param   data : Pointer to buffer for data to receive from i2c receiver
 * @param   addr : Target slave address
 * @param   num  : Number of data items to receive
 * @param   I2C  : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  received count              : For data receive count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_MasterReceive(I2C_RESOURCES *I2C,
                                     uint32_t        addr,
                                     uint8_t         *data,
                                     uint32_t        num,
                                     bool            xfer_pending)
{
    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0)
        return ARM_DRIVER_ERROR;

    /* addr 7bit addr: 0x7F , 10bit addr: 0x3FF */
    if ((data == NULL) || (num == 0U) || ((addr & (~ARM_I2C_ADDRESS_10BIT)) > 0x3FF))
    {
        /* Invalid parameters */
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (I2C->state.master_setup == 0U)
    {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* set status busy  */
    I2C->status.busy             = 1;

    /* fill the i2c transfer structure as per user detail */
    I2C->transfer.rx_buf           = (uint8_t *)data;
    I2C->transfer.rx_total_num     = num;
    I2C->transfer.rx_curr_cnt      = 0U;
    I2C->transfer.curr_cnt         = 0U;
    I2C->transfer.rx_curr_tx_index = 0U;

    I2C->transfer.pending   = xfer_pending;
    I2C->transfer.curr_stat = I2C_TRANSFER_MST_RX;

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.mode             = I2C_MASTER_MODE;
    I2C->status.direction        = I2C_DIR_RECEIVER;
    I2C->status.arbitration_lost = 0;
    I2C->status.bus_error        = 0;

    I2C->transfer.rx_over = 0U;

    I2C_SetTargetAddress(I2C, addr);

    /* Enabling the rx interrupt */
    i2c_master_enable_rx_interrupt(I2C->regs);

    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c slave transmit
 *          Start sending data to i2c master.
 * @note    master_setup bit should be enabled first /ref ARM_I2C_BUS_SPEED
 * @param   addr : Target slave address
 * @param   data : Pointer to buffer with data to send to i2c master
 * @param   num  : Number of data items to send
 * @param   I2C  : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  transmit count              : For data transmit count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_SlaveTransmit(I2C_RESOURCES *I2C,
                                     const uint8_t *data,
                                           uint32_t num)
{

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0)
        return ARM_DRIVER_ERROR;

    if((data == NULL) || (num == 0U))
        return ARM_DRIVER_ERROR_PARAMETER;

    /* Check Slave mode is enabled */
    if (I2C->state.slave_setup == 0U)
    {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* set status busy  */
    I2C->status.busy             = 1;

    /* fill the i2c transfer structure as per user detail */
    I2C->transfer.tx_buf        = (uint8_t *)data;
    I2C->transfer.tx_total_num  = num;
    I2C->transfer.tx_curr_cnt   = 0U;
    I2C->transfer.curr_cnt      = 0U;
    I2C->transfer.tx_over       = 0U;
    I2C->transfer.curr_stat = I2C_TRANSFER_SLV_TX;

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.mode             = I2C_SLAVE_MODE;
    I2C->status.direction        = I2C_DIR_TRANSMITTER;
    I2C->status.arbitration_lost = 0;
    I2C->status.bus_error        = 0;

    /* Enabling the tx interrupt */
    i2c_slave_enable_tx_interrupt(I2C->regs);

    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c slave receive
 *          Start receiving data from i2c master.
 * @note    none
 * @param   data : Pointer to buffer for data to receive from i2c master
 * @param   num  : Number of data items to receive
 * @param   I2C  : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  received count              : For data receive count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_SlaveReceive(I2C_RESOURCES *I2C,
                                    uint8_t         *data,
                                    uint32_t         num)
{

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0)
        return ARM_DRIVER_ERROR;

    if((data == NULL) || (num == 0U))
        return ARM_DRIVER_ERROR_PARAMETER;

    /* Check Slave mode is enabled */
    if (I2C->state.slave_setup == 0U)
    {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* set status busy  */
    I2C->status.busy             = 1;

    /* fill the i2c transfer structure as per user detail */
    I2C->transfer.rx_buf        = (uint8_t *)data;
    I2C->transfer.rx_total_num  = num;
    I2C->transfer.rx_curr_cnt   = 0U;
    I2C->transfer.curr_cnt      = 0U;
    I2C->transfer.curr_stat     = I2C_TRANSFER_SLV_RX;

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.mode             = I2C_SLAVE_MODE;
    I2C->status.direction        = I2C_DIR_RECEIVER;
    I2C->status.bus_error        = 0;

    I2C->transfer.rx_over = 0U;

    /* Enabling the rx interrupt */
    i2c_slave_enable_rx_interrupt(I2C->regs);

    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c get transfer data count
 * @note    it can be either transmit or receive data count which perform last
 *          (useful only in interrupt mode)
 * @param   I2C   : Pointer to i2c resources structure
 * @retval  transfer data count
 */
static int32_t ARM_I2C_GetDataCount(const I2C_RESOURCES *I2C)
{
    /* return common count for both tx/rx */
    return I2C->transfer.curr_cnt;
}

/**
 * @brief   CMSIS-Driver i2c control
 *          Control i2c Interface.
 * @note    none
 * @param   control : Operation
 * @param   arg     : Argument of operation (optional)
 * @param   I2C     : Pointer to i2c resources structure
 * @retval  common \ref execution_status and driver specific \ref i2c_execution_status
 */
static int32_t ARM_I2C_Control(I2C_RESOURCES  *I2C,
                               uint32_t         control,
                               uint32_t         arg)
{
    int32_t speed;

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0)
        return ARM_DRIVER_ERROR;

    switch (control)
    {
        case ARM_I2C_OWN_ADDRESS:

             speed = I2C_GetBusSpeed(I2C, ARM_I2C_BUS_SPEED_STANDARD);

             if(arg & ARM_I2C_ADDRESS_10BIT)
             {
                 /* Sets 10 bit addr mode */
                 I2C->addr_mode = I2C_10BIT_ADDRESS;
             }
             else
             {
                 /* Sets 7 bit addr mode */
                 I2C->addr_mode = I2C_7BIT_ADDRESS;
             }

             i2c_slave_init(I2C->regs, arg, I2C->addr_mode);

             /* Sets bus speed*/
             i2c_set_bus_speed(I2C->regs, speed);

             I2C->mode = I2C_SLAVE_MODE;
             /* setup slave flag */
             I2C->state.slave_setup = 1;

            break;

        case ARM_I2C_BUS_SPEED:

             speed = I2C_GetBusSpeed(I2C, arg);
             /* arg is i2c bus speed */
             i2c_master_init(I2C->regs, I2C->tar_addr);

             /* Sets master clock settings*/
             i2c_master_set_clock(I2C->regs, (I2C->clk / 1000), I2C->speed_mode);

             /* Sets bus speed*/
             i2c_set_bus_speed(I2C->regs, speed);

             I2C->mode = I2C_MASTER_MODE;
             /* setup master flag */
             I2C->state.master_setup = 1;

            break;

        case ARM_I2C_BUS_CLEAR:
            /* disable device, clear all the interrupt, enable device. */
            i2c_disable(I2C->regs);
            i2c_clear_all_interrupt(I2C->regs);

            I2C->transfer.next_cond  = I2C_MODE_STOP;
            I2C->transfer.curr_stat  = I2C_TRANSFER_NONE;
            I2C->transfer.err_state  = I2C_ERR_NONE;
            I2C->transfer.tx_over = 0U;
            I2C->transfer.rx_over = 0U;

            i2c_enable(I2C->regs);

            break;

        case ARM_I2C_ABORT_TRANSFER:

            /* only useful in interrupt method,
             * no effect on polling method.
             */

            /* i2c is half-duplex, at a time it can be either tx or rx
             * not sure which one is running, so clearing both. */

            /* if tx interrupt flag is enable then only disable transmit interrupt */
            if (I2C->mode == I2C_MASTER_MODE)
            {
                i2c_master_disable_tx_interrupt(I2C->regs);
            }
            else
            {
                i2c_slave_disable_tx_interrupt(I2C->regs);
            }

            if (I2C->mode == I2C_MASTER_MODE)
            {
                i2c_master_disable_rx_interrupt(I2C->regs);
            }
            else
            {
                i2c_slave_disable_rx_interrupt(I2C->regs);
            }

            /* Reset the tx_buffer */
            I2C->transfer.tx_total_num = 0U;
            I2C->transfer.rx_total_num = 0U;
            I2C->transfer.curr_stat = I2C_TRANSFER_NONE;

            /* clear busy status bit. */
            I2C->status.busy = 0U;

            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c get status
 * @note    none
 * @param   I2C : Pointer to i2c resources structure
 * @retval  ARM_i2c_STATUS
 */
static ARM_I2C_STATUS ARM_I2C_GetStatus(const I2C_RESOURCES  *I2C)
{
    return I2C->status;
}

/* I2C0 Driver Instance */
#if (RTE_I2C0)

/* I2C0 Driver Resources */
static I2C_RESOURCES I2C0_RES =
{
  .regs                  = (I2C_Type *)I2C0_BASE,
  .clk                   = (uint32_t)I2C_PERIPHERAL_CLOCK,
  .irq_num               = (IRQn_Type)I2C0_IRQ_IRQn,
  .irq_priority          = (uint32_t)RTE_I2C0_IRQ_PRIORITY,
  .tx_fifo_threshold     = RTE_I2C0_TX_FIFO_THRESHOLD,
  .rx_fifo_threshold     = RTE_I2C0_RX_FIFO_THRESHOLD
};

static int32_t I2C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
  return ARM_I2C_Initialize(cb_event, &I2C0_RES);
}

static int32_t I2C0_Uninitialize(void)
{
  return ARM_I2C_Uninitialize(&I2C0_RES);
}

static int32_t I2C0_PowerControl(ARM_POWER_STATE state)
{
  return ARM_I2C_PowerControl(state, &I2C0_RES);
}

static int32_t I2C0_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num,  bool xfer_pending)
{
  return ARM_I2C_MasterTransmit(&I2C0_RES, addr, data, num, xfer_pending);
}

static int32_t I2C0_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
  return (ARM_I2C_MasterReceive(&I2C0_RES, addr, data, num, xfer_pending));
}

static int32_t I2C0_SlaveTransmit(const uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveTransmit(&I2C0_RES, data, num));
}

static int32_t I2C0_SlaveReceive(uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveReceive(&I2C0_RES, data, num));
}

static int32_t I2C0_GetDataCount(void)
{
  return (ARM_I2C_GetDataCount(&I2C0_RES));
}

static int32_t I2C0_Control(uint32_t control, uint32_t arg)
{
  return (ARM_I2C_Control(&I2C0_RES, control, arg));
}

static ARM_I2C_STATUS I2C0_GetStatus(void)
{
  return (ARM_I2C_GetStatus(&I2C0_RES));
}

void I2C0_IRQHandler(void)
{
  i2c_transfer_info_t *transfer = &(I2C0_RES.transfer);
  ARM_I2C_STATUS *i2c_stat = &(I2C0_RES.status);

  if (transfer->curr_stat == I2C_TRANSFER_MST_TX)
  {
      i2c_master_tx_isr(I2C0_RES.regs, transfer);
  }
  if (transfer->curr_stat == I2C_TRANSFER_MST_RX)
  {
      i2c_master_rx_isr(I2C0_RES.regs, transfer);
  }

  if (transfer->curr_stat == I2C_TRANSFER_SLV_TX)
  {
      /* slave transmit*/
      i2c_slave_tx_isr(I2C0_RES.regs, transfer);
  }
  if (transfer->curr_stat == I2C_TRANSFER_SLV_RX)
  {
      /* slave receive */
      i2c_slave_rx_isr(I2C0_RES.regs, transfer);
  }

  /* Check the ISR response */
  if (transfer->status & I2C_TRANSFER_STATUS_DONE)
  {
    /* set busy flag to 0U */
    i2c_stat->busy = 0U;

    I2C0_RES.cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_INCOMPLETE)
  {
    I2C0_RES.cb_event(ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_TRANSMIT)
  {
    I2C0_RES.cb_event(ARM_I2C_EVENT_SLAVE_TRANSMIT);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_RECEIVE)
  {
    I2C0_RES.cb_event(ARM_I2C_EVENT_SLAVE_RECEIVE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ADDRESS_NACK)
  {
    I2C0_RES.cb_event(ARM_I2C_EVENT_ADDRESS_NACK);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_GENERAL_CALL)
  {
    I2C0_RES.cb_event(ARM_I2C_EVENT_GENERAL_CALL);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ARBITRATION_LOST)
  {
    i2c_stat->arbitration_lost = 1U;

    I2C0_RES.cb_event(ARM_I2C_EVENT_ARBITRATION_LOST);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_BUS_ERROR)
  {
    i2c_stat->bus_error = 1U;
    I2C0_RES.cb_event(ARM_I2C_EVENT_BUS_ERROR);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_BUS_CLEAR)
  {
    I2C0_RES.cb_event(ARM_I2C_EVENT_BUS_CLEAR);
  }

  transfer->status = I2C_TRANSFER_STATUS_NONE;
}

/* I2C0 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C0;
ARM_DRIVER_I2C Driver_I2C0 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C0_Initialize,
    I2C0_Uninitialize,
    I2C0_PowerControl,
    I2C0_MasterTransmit,
    I2C0_MasterReceive,
    I2C0_SlaveTransmit,
    I2C0_SlaveReceive,
    I2C0_GetDataCount,
    I2C0_Control,
    I2C0_GetStatus
};
#endif /* RTE_I2C0 */

/* I2C1 Driver Instance */
#if (RTE_I2C1)

/* I2C1 Driver Resources */
static I2C_RESOURCES I2C1_RES =
{
  .regs                  = (I2C_Type *)I2C1_BASE,
  .clk                   = (uint32_t)I2C_PERIPHERAL_CLOCK,
  .irq_num               = (IRQn_Type)I2C1_IRQ_IRQn,
  .irq_priority          = (uint32_t)RTE_I2C1_IRQ_PRIORITY,
  .tx_fifo_threshold     = RTE_I2C1_TX_FIFO_THRESHOLD,
  .rx_fifo_threshold     = RTE_I2C1_RX_FIFO_THRESHOLD
};

static int32_t I2C1_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
  return ARM_I2C_Initialize(cb_event, &I2C1_RES);
}

static int32_t I2C1_Uninitialize(void)
{
  return ARM_I2C_Uninitialize(&I2C1_RES);
}

static int32_t I2C1_PowerControl(ARM_POWER_STATE state)
{
  return ARM_I2C_PowerControl(state, &I2C1_RES);
}

static int32_t I2C1_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num, bool xfer_pending)
{
  return ARM_I2C_MasterTransmit(&I2C1_RES, addr, data, num, xfer_pending);
}

static int32_t I2C1_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
  return (ARM_I2C_MasterReceive(&I2C1_RES, addr, data, num, xfer_pending));
}

static int32_t I2C1_SlaveTransmit(const uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveTransmit(&I2C1_RES, data, num));
}

static int32_t I2C1_SlaveReceive(uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveReceive(&I2C1_RES, data, num));
}

static int32_t I2C1_GetDataCount(void)
{
  return (ARM_I2C_GetDataCount(&I2C1_RES));
}

static int32_t I2C1_Control(uint32_t control, uint32_t arg)
{
  return (ARM_I2C_Control(&I2C1_RES, control, arg));
}

static ARM_I2C_STATUS I2C1_GetStatus(void)
{
  return (ARM_I2C_GetStatus(&I2C1_RES));
}

void I2C1_IRQHandler(void)
{
  i2c_transfer_info_t *transfer = &(I2C1_RES.transfer);
  ARM_I2C_STATUS *i2c_stat = &(I2C1_RES.status);

  if (transfer->curr_stat == I2C_TRANSFER_MST_TX)
  {
     i2c_master_tx_isr(I2C1_RES.regs, transfer);
  }
  if(transfer->curr_stat == I2C_TRANSFER_MST_RX)
  {
     i2c_master_rx_isr(I2C1_RES.regs, transfer);
  }

  if(transfer->curr_stat == I2C_TRANSFER_SLV_TX)
  {
     /* slave transmit*/
     i2c_slave_tx_isr(I2C1_RES.regs, transfer);
  }
  if(transfer->curr_stat == I2C_TRANSFER_SLV_RX)
  {
     /* slave receive */
     i2c_slave_rx_isr(I2C1_RES.regs, transfer);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_DONE)
  {
     /* set busy flag to 0U */
     i2c_stat->busy = 0U;

     I2C1_RES.cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_INCOMPLETE)
  {
     I2C1_RES.cb_event(ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_TRANSMIT)
  {
     I2C1_RES.cb_event(ARM_I2C_EVENT_SLAVE_TRANSMIT);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_RECEIVE)
  {
     I2C1_RES.cb_event(ARM_I2C_EVENT_SLAVE_RECEIVE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ADDRESS_NACK)
  {
     I2C1_RES.cb_event(ARM_I2C_EVENT_ADDRESS_NACK);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_GENERAL_CALL)
  {
     I2C1_RES.cb_event(ARM_I2C_EVENT_GENERAL_CALL);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ARBITRATION_LOST)
  {
     i2c_stat->arbitration_lost = 1U;

     I2C1_RES.cb_event(ARM_I2C_EVENT_ARBITRATION_LOST);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_BUS_ERROR)
  {
     i2c_stat->bus_error = 1U;

     I2C1_RES.cb_event(ARM_I2C_EVENT_BUS_ERROR);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_BUS_CLEAR)
  {
     I2C1_RES.cb_event(ARM_I2C_EVENT_BUS_CLEAR);
  }

  transfer->status = I2C_TRANSFER_STATUS_NONE;
}

/* I2C1 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C1;
ARM_DRIVER_I2C Driver_I2C1 = {
  ARM_I2C_GetVersion,
  ARM_I2C_GetCapabilities,
  I2C1_Initialize,
  I2C1_Uninitialize,
  I2C1_PowerControl,
  I2C1_MasterTransmit,
  I2C1_MasterReceive,
  I2C1_SlaveTransmit,
  I2C1_SlaveReceive,
  I2C1_GetDataCount,
  I2C1_Control,
  I2C1_GetStatus
};
#endif /* RTE_I2C1 */

/* I2C2 Driver Instance */
#if (RTE_I2C2)

/* I2C2 Driver Resources */
static I2C_RESOURCES I2C2_RES =
{
  .regs                  = (I2C_Type *)I2C2_BASE,
  .clk                   = (uint32_t)I2C_PERIPHERAL_CLOCK,
  .irq_num               = (IRQn_Type)I2C2_IRQ_IRQn,
  .irq_priority          = (uint32_t)RTE_I2C2_IRQ_PRIORITY,
  .tx_fifo_threshold     = RTE_I2C2_TX_FIFO_THRESHOLD,
  .rx_fifo_threshold     = RTE_I2C2_RX_FIFO_THRESHOLD
};

static int32_t I2C2_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
  return ARM_I2C_Initialize(cb_event, &I2C2_RES);
}

static int32_t I2C2_Uninitialize(void)
{
  return ARM_I2C_Uninitialize(&I2C2_RES);
}

static int32_t I2C2_PowerControl(ARM_POWER_STATE state)
{
  return ARM_I2C_PowerControl(state, &I2C2_RES);
}

static int32_t I2C2_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num, bool xfer_pending)
{
  return ARM_I2C_MasterTransmit(&I2C2_RES, addr, data, num, xfer_pending);
}

static int32_t I2C2_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
  return (ARM_I2C_MasterReceive(&I2C2_RES, addr, data, num, xfer_pending));
}

static int32_t I2C2_SlaveTransmit(const uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveTransmit(&I2C2_RES, data, num));
}

static int32_t I2C2_SlaveReceive(uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveReceive(&I2C2_RES, data, num));
}

static int32_t I2C2_GetDataCount(void)
{
  return (ARM_I2C_GetDataCount(&I2C2_RES));
}

static int32_t I2C2_Control(uint32_t control, uint32_t arg)
{
  return (ARM_I2C_Control(&I2C2_RES, control, arg));
}

static ARM_I2C_STATUS I2C2_GetStatus(void)
{
  return (ARM_I2C_GetStatus(&I2C2_RES));
}

void I2C2_IRQHandler(void)
{
  i2c_transfer_info_t *transfer = &(I2C2_RES.transfer);
  ARM_I2C_STATUS *i2c_stat = &(I2C2_RES.status);

  /* Check for master mode */
  if (I2C2_RES.mode == I2C_MASTER_MODE)
  {
      if (transfer->curr_stat == I2C_TRANSFER_MST_TX)
      {
           i2c_master_tx_isr(I2C2_RES.regs, transfer);
      }
      if(transfer->curr_stat == I2C_TRANSFER_MST_RX)
      {
           i2c_master_rx_isr(I2C2_RES.regs, transfer);
      }
  }
  else /* Slave mode */
  {
       if(transfer->curr_stat == I2C_TRANSFER_SLV_TX)
      {
         /* slave transmit*/
         i2c_slave_tx_isr(I2C2_RES.regs, transfer);
      }
      if(transfer->curr_stat == I2C_TRANSFER_SLV_RX)
      {
         /* slave receive */
          i2c_slave_rx_isr(I2C2_RES.regs, transfer);
      }
  }/* Slave mode */

  if (transfer->status & I2C_TRANSFER_STATUS_DONE)
  {

      /* set busy flag to 0U */
      i2c_stat->busy = 0U;

      I2C2_RES.cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_INCOMPLETE)
  {
     I2C2_RES.cb_event(ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_TRANSMIT)
  {
     I2C2_RES.cb_event(ARM_I2C_EVENT_SLAVE_TRANSMIT);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_RECEIVE)
  {
     I2C2_RES.cb_event(ARM_I2C_EVENT_SLAVE_RECEIVE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ADDRESS_NACK)
  {
     I2C2_RES.cb_event(ARM_I2C_EVENT_ADDRESS_NACK);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_GENERAL_CALL)
  {
     I2C2_RES.cb_event(ARM_I2C_EVENT_GENERAL_CALL);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ARBITRATION_LOST)
  {
     i2c_stat->arbitration_lost = 1U;

     I2C2_RES.cb_event(ARM_I2C_EVENT_ARBITRATION_LOST);
  }

   if (transfer->status & I2C_TRANSFER_STATUS_BUS_ERROR)
  {
     i2c_stat->bus_error = 1U;

     I2C2_RES.cb_event(ARM_I2C_EVENT_BUS_ERROR);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_BUS_CLEAR)
  {
     I2C2_RES.cb_event(ARM_I2C_EVENT_BUS_CLEAR);
  }

  transfer->status = I2C_TRANSFER_STATUS_NONE;
}

/* I2C2 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C2;
ARM_DRIVER_I2C Driver_I2C2 = {
  ARM_I2C_GetVersion,
  ARM_I2C_GetCapabilities,
  I2C2_Initialize,
  I2C2_Uninitialize,
  I2C2_PowerControl,
  I2C2_MasterTransmit,
  I2C2_MasterReceive,
  I2C2_SlaveTransmit,
  I2C2_SlaveReceive,
  I2C2_GetDataCount,
  I2C2_Control,
  I2C2_GetStatus
};
#endif /* RTE_I2C2 */

/* I2C3 Driver Instance */
#if (RTE_I2C3)

/* I2C3 Driver Resources */
static I2C_RESOURCES I2C3_RES =
{
  .regs                  = (I2C_Type *)I2C3_BASE,
  .clk                   = (uint32_t)I2C_PERIPHERAL_CLOCK,
  .irq_num               = (IRQn_Type)I2C3_IRQ_IRQn,
  .irq_priority          = (uint32_t)RTE_I2C3_IRQ_PRIORITY,
  .tx_fifo_threshold     = RTE_I2C3_TX_FIFO_THRESHOLD,
  .rx_fifo_threshold     = RTE_I2C3_RX_FIFO_THRESHOLD
};

static int32_t I2C3_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
  return ARM_I2C_Initialize(cb_event, &I2C3_RES);
}

static int32_t I2C3_Uninitialize(void)
{
  return ARM_I2C_Uninitialize(&I2C3_RES);
}

static int32_t I2C3_PowerControl(ARM_POWER_STATE state)
{
  return ARM_I2C_PowerControl(state, &I2C3_RES);
}

static int32_t I2C3_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num, bool xfer_pending)
{
  return ARM_I2C_MasterTransmit(&I2C3_RES, addr, data, num, xfer_pending);
}

static int32_t I2C3_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
  return (ARM_I2C_MasterReceive(&I2C3_RES, addr, data, num, xfer_pending));
}

static int32_t I2C3_SlaveTransmit(const uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveTransmit(&I2C3_RES, data, num));
}

static int32_t I2C3_SlaveReceive(uint8_t *data, uint32_t num)
{
  return (ARM_I2C_SlaveReceive(&I2C3_RES, data, num));
}

static int32_t I2C3_GetDataCount(void)
{
  return (ARM_I2C_GetDataCount(&I2C3_RES));
}

static int32_t I2C3_Control(uint32_t control, uint32_t arg)
{
  return (ARM_I2C_Control(&I2C3_RES, control, arg));
}

static ARM_I2C_STATUS I2C3_GetStatus(void)
{
  return (ARM_I2C_GetStatus(&I2C3_RES));
}

void I2C3_IRQHandler(void)
{
  i2c_transfer_info_t *transfer = &(I2C3_RES.transfer);
  ARM_I2C_STATUS *i2c_stat = &(I2C3_RES.status);

  /* Check for master mode */
  if (I2C3_RES.mode == I2C_MASTER_MODE)
  {
      if (transfer->curr_stat == I2C_TRANSFER_MST_TX)
      {
           i2c_master_tx_isr(I2C3_RES.regs, transfer);
      }
      if(transfer->curr_stat == I2C_TRANSFER_MST_RX)
      {
           i2c_master_rx_isr(I2C3_RES.regs, transfer);
      }
  }
  else /* Slave mode */
  {
       if(transfer->curr_stat == I2C_TRANSFER_SLV_TX)
      {
         /* slave transmit*/
         i2c_slave_tx_isr(I2C3_RES.regs, transfer);
      }
      if(transfer->curr_stat == I2C_TRANSFER_SLV_RX)
      {
         /* slave receive */
          i2c_slave_rx_isr(I2C3_RES.regs, transfer);
      }
  }/* Slave mode */

  if (transfer->status & I2C_TRANSFER_STATUS_DONE)
  {

      /* set busy flag to 0U */
      i2c_stat->busy = 0U;


      I2C3_RES.cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_INCOMPLETE)
  {
     I2C3_RES.cb_event(ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_TRANSMIT)
  {
     I2C3_RES.cb_event(ARM_I2C_EVENT_SLAVE_TRANSMIT);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_SLAVE_RECEIVE)
  {
     I2C3_RES.cb_event(ARM_I2C_EVENT_SLAVE_RECEIVE);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ADDRESS_NACK)
  {
     I2C3_RES.cb_event(ARM_I2C_EVENT_ADDRESS_NACK);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_GENERAL_CALL)
  {
     I2C3_RES.cb_event(ARM_I2C_EVENT_GENERAL_CALL);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_ARBITRATION_LOST)
  {
     i2c_stat->arbitration_lost = 1U;

     I2C3_RES.cb_event(ARM_I2C_EVENT_ARBITRATION_LOST);
  }

   if (transfer->status & I2C_TRANSFER_STATUS_BUS_ERROR)
  {
     i2c_stat->bus_error = 1U;

     I2C3_RES.cb_event(ARM_I2C_EVENT_BUS_ERROR);
  }

  if (transfer->status & I2C_TRANSFER_STATUS_BUS_CLEAR)
  {
     I2C3_RES.cb_event(ARM_I2C_EVENT_BUS_CLEAR);
  }

  transfer->status = I2C_TRANSFER_STATUS_NONE;
}

/* I2C3 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C3;
ARM_DRIVER_I2C Driver_I2C3 = {
  ARM_I2C_GetVersion,
  ARM_I2C_GetCapabilities,
  I2C3_Initialize,
  I2C3_Uninitialize,
  I2C3_PowerControl,
  I2C3_MasterTransmit,
  I2C3_MasterReceive,
  I2C3_SlaveTransmit,
  I2C3_SlaveReceive,
  I2C3_GetDataCount,
  I2C3_Control,
  I2C3_GetStatus
};
#endif /* RTE_I2C3 */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
