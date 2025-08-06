/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* system includes */
#include "Driver_LPI2C_Private.h"
#include "sys_utils.h"

#define ARM_I2C_DRV_VERISON ARM_DRIVER_VERSION_MAJOR_MINOR(0, 1) /*DRIVER VERSION*/

#if defined(RTE_Drivers_LPI2C)

/* 1 Mega bits */
#define LPI2C_1_MEGA_BIT 1000000

#if !RTE_LPI2C0_BUS_SPEED
#error "Invalid LPI2C Bitrate"
#else
#define LPI2C_1BYTE_TIME (DIV_ROUND_UP(LPI2C_1_MEGA_BIT, RTE_LPI2C0_BUS_SPEED) * 8)
#endif

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion        = {ARM_I2C_API_VERSION, ARM_I2C_DRV_VERISON};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
    0, /* 10 bit addressing */
    0  /* reserved          */
};

/**
 * @brief   get lpi2c version
 * @note    none
 * @param   none
 * @retval  driver version
 */
ARM_DRIVER_VERSION ARM_I2C_GetVersion(void)
{
    return DriverVersion;
}

/**
 * @brief   get lpi2c capabilites
 * @note    none
 * @param   none
 * @retval  driver capabilites
 */
static ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 * @brief   lpi2c initialize
 * @note    it will use initialize the lpi2c driver.
 * @param   cb_event    : Pointer to \ref ARM_LPI2C_SignalEvent
 * @param   LPI2C_RES       : Pointer to lpi2c resources structure
 * @retval  ARM_DRIVER_OK : successfully initialized
 */
static int32_t ARM_LPI2C_Initialize(ARM_I2C_SignalEvent_t cb_event, LPI2C_RESOURCES *LPI2C_RES)
{
    if (!cb_event) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    LPI2C_RES->cb_event          = cb_event;

    LPI2C_RES->state.initialized = 1U;

    return ARM_DRIVER_OK;
}

/**
 * @func   : int32_t ARM_LPI2C_Uninitialize(LPI2C_RESOURCES *LPI2C_RES)
 * @brief  : lpi2c uninitialize
 * @note   : it will use uninitialize the lpi2c driver.
 * @param  : LPI2C_RES         : Pointer to lpi2c resources structure
 * @retval : ARM_DRIVER_OK : successfully initialized
 */
static int32_t ARM_LPI2C_Uninitialize(LPI2C_RESOURCES *LPI2C_RES)
{

    /* check lpi2c driver is initialized or not */
    if (LPI2C_RES->state.initialized == 0) {
        return ARM_DRIVER_OK;
    }

    /* check lpi2c driver is powered or not */
    if (LPI2C_RES->state.powered == 1) {
        return ARM_DRIVER_ERROR;
    }

    LPI2C_RES->cb_event          = 0U;

    LPI2C_RES->state.initialized = 0U;

    return ARM_DRIVER_OK;
}

/**
 * @func    : int32_t ARM_LPI2C_PowerControl (ARM_POWER_STATE state, LPI2C_RESOURCES *LPI2C_RES)
 * @brief   : Power the driver and enable the NVIC
 * @param   : state : Power state
 * @param   : LPI2C_RES   : Pointer to lpi2c resources structure
 * @return  : ARM_DRIVER_OK
 */
static int32_t ARM_LPI2C_PowerControl(ARM_POWER_STATE state, LPI2C_RESOURCES *LPI2C_RES)
{
    switch (state) {
    case ARM_POWER_FULL:

        /* check for Driver initialization */
        if (LPI2C_RES->state.initialized == 0) {
            return ARM_DRIVER_ERROR;
        }

        /* check for the power is done before initialization or not */
        if (LPI2C_RES->state.powered == 1) {
            return ARM_DRIVER_OK;
        }

        /* Clear Any Pending Irq */
        NVIC_ClearPendingIRQ(LPI2C_RES->irq_num);

        /* Set Priority */
        NVIC_SetPriority(LPI2C_RES->irq_num, LPI2C_RES->irq_priority);

        /* Enable IRQ */
        NVIC_EnableIRQ(LPI2C_RES->irq_num);

        LPI2C_RES->state.powered = 1;

        break;
    case ARM_POWER_OFF:

        if (LPI2C_RES->state.powered == 0) {
            return ARM_DRIVER_OK;
        }

        /* Disable the IRQ */
        NVIC_DisableIRQ(LPI2C_RES->irq_num);

        /* Clearing pending */
        NVIC_ClearPendingIRQ(LPI2C_RES->irq_num);

        LPI2C_RES->state.powered = 0;
        break;
    case ARM_POWER_LOW:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 * @func   : int32_t ARM_LPI2C_SlaveTransmit(LPI2C_RESOURCES *LPI2C_RES,
                                        const uint8_t *data,
                                              uint32_t num)
 * @brief  : lpi2c slave transmit
 *         : Start sending data to i2c master.
 * @param  : data : Pointer to buffer with data to send to i2c master
 * @param  : num  : Number of data items to send
 * @param  : LPI2C_RES  : Pointer to lpi2c resources structure
 * @retval : ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval : ARM_DRIVER_ERROR            : error in driver
 * @retval : ARM_DRIVER_OK               : success in interrupt case
 */
static int32_t ARM_LPI2C_SlaveTransmit(LPI2C_RESOURCES *LPI2C_RES, const uint8_t *data,
                                       uint32_t num)
{

    /* check lpi2c driver is initialized or not */
    if (LPI2C_RES->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* check lpi2c driver is powered or not */
    if (LPI2C_RES->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    if (LPI2C_RES->status.busy == 1U) {
        return ARM_DRIVER_ERROR;
    }

    /* Set busy state */
    LPI2C_RES->status.busy           = 1U;

    LPI2C_RES->transfer.tx_buf       = data;
    LPI2C_RES->transfer.tx_total_num = num;
    LPI2C_RES->transfer.tx_curr_cnt  = 0U;

    LPI2C_RES->transfer.rx_buf       = NULL;
    LPI2C_RES->transfer.rx_total_num = 0U;
    LPI2C_RES->transfer.rx_curr_cnt  = 0U;

    /* Writing data to fifo */
    lpi2c_send(LPI2C_RES->regs, &LPI2C_RES->transfer, LPI2C_1BYTE_TIME);

    if (LPI2C_RES->transfer.status & LPI2C_XFER_STAT_COMPLETE) {
        LPI2C_RES->status.busy = 0U;
        /* receive complete successfully. */
        LPI2C_RES->cb_event(ARM_I2C_EVENT_TRANSFER_DONE);

        LPI2C_RES->transfer.status = LPI2C_XFER_STAT_NONE;
    }

    return ARM_DRIVER_OK;
}

/**
 * @func   : int32_t ARM_LPI2C_SlaveTransmit(LPI2C_RESOURCES *LPI2C_RES,
                                       const uint8_t *data,
                                             uint32_t num)
 * @brief  : lpi2c slave receive
 *         : Start receiving data from i2c master.
 * @note   : none
 * @param  : data : Pointer to buffer for data to receive from i2c master
 * @param  : num  : Number of data items to receive
 * @param  : LPI2C_RES  : Pointer to lpi2c resources structure
 * @retval : ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval : ARM_DRIVER_ERROR            : error in driver
 * @retval : ARM_DRIVER_OK               : success in interrupt case
 */
static int32_t ARM_LPI2C_SlaveReceive(LPI2C_RESOURCES *LPI2C_RES, uint8_t *data, uint32_t num)
{
    /* check lpi2c driver is initialized or not */
    if (LPI2C_RES->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* check lpi2c driver is powered or not */
    if (LPI2C_RES->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    if (LPI2C_RES->status.busy == 1U) {
        return ARM_DRIVER_ERROR;
    }

    /* Set busy state */
    LPI2C_RES->status.busy           = 1U;

    LPI2C_RES->transfer.rx_buf       = data;
    LPI2C_RES->transfer.rx_total_num = num;
    LPI2C_RES->transfer.rx_curr_cnt  = 0U;

    LPI2C_RES->transfer.tx_buf       = NULL;
    LPI2C_RES->transfer.tx_total_num = 0U;
    LPI2C_RES->transfer.tx_curr_cnt  = 0U;

    return ARM_DRIVER_OK;
}

/**
 * @brief  : CMSIS-Driver lpi2c get transfer data count
 * @note   : it can be either transmit or receive data count which perform last
 *           (useful only in interrupt mode)
 * @param  : LPI2C_RES   : Pointer to lpi2c resources structure
 * @retval : transfer data count
 */
static int32_t ARM_LPI2C_GetDataCount(const LPI2C_RESOURCES *LPI2C_RES)
{
    int32_t ret;

    if (LPI2C_RES->transfer.tx_buf != NULL) {
        ret = LPI2C_RES->transfer.tx_curr_cnt;
    } else {
        ret = LPI2C_RES->transfer.rx_curr_cnt;
    }

    return ret;
}

/**
 * @brief  : CMSIS-Driver lpi2c get status
 * @note   : none
 * @param  : LPI2C_RES : Pointer to lpi2c resources structure
 * @retval : ARM_i2c_STATUS
 */
static ARM_I2C_STATUS ARM_LPI2C_GetStatus(const LPI2C_RESOURCES *LPI2C_RES)
{
    return LPI2C_RES->status;
}

/* LPI2C Driver Instance */
#if (RTE_LPI2C0)

/* LPI2C Driver Resources */
static LPI2C_RESOURCES LPI2C0_RES = {.regs         = (LPI2C_TYPE *) LPI2C0_BASE,
                                     .irq_num      = (IRQn_Type) LPI2C0_IRQ_IRQn,
                                     .irq_priority = (uint32_t) RTE_LPI2C0_IRQ_PRIORITY};

void LPI2C0_IRQHandler(void)
{
    LPI2C_RESOURCES *res        = &LPI2C0_RES;

    LPI2C_XFER_INFO_T *transfer = &(res->transfer);

    lpi2c_irq_handler(res->regs, &(res->transfer));

    if (transfer->status == LPI2C_XFER_STAT_COMPLETE) {
        res->status.busy = 0U;
        /* receive complete successfully. */
        res->cb_event(ARM_I2C_EVENT_TRANSFER_DONE);

        transfer->status = LPI2C_XFER_STAT_NONE;
    }
}

static int32_t LPI2C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_LPI2C_Initialize(cb_event, &LPI2C0_RES);
}

static int32_t LPI2C0_Uninitialize(void)
{
    return ARM_LPI2C_Uninitialize(&LPI2C0_RES);
}

static int32_t LPI2C0_PowerControl(ARM_POWER_STATE state)
{
    return ARM_LPI2C_PowerControl(state, &LPI2C0_RES);
}

static int32_t LPI2C0_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num,
                                     bool xfer_pending)
{
    ARG_UNUSED(addr);
    ARG_UNUSED(data);
    ARG_UNUSED(num);
    ARG_UNUSED(xfer_pending);
    /* lpi2c is slave-only */
    return ARM_DRIVER_ERROR;
}

static int32_t LPI2C0_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{

    ARG_UNUSED(addr);
    ARG_UNUSED(data);
    ARG_UNUSED(num);
    ARG_UNUSED(xfer_pending);
    /* lpi2c is slave-only */
    return ARM_DRIVER_ERROR;
}

static int32_t LPI2C0_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return ARM_LPI2C_SlaveTransmit(&LPI2C0_RES, data, num);
}

static int32_t LPI2C0_SlaveReceive(uint8_t *data, uint32_t num)
{
    return ARM_LPI2C_SlaveReceive(&LPI2C0_RES, data, num);
}

static int32_t LPI2C0_GetDataCount(void)
{
    return ARM_LPI2C_GetDataCount(&LPI2C0_RES);
}

static int32_t LPI2C0_Control(uint32_t control, uint32_t arg)
{
    ARG_UNUSED(control);
    ARG_UNUSED(arg);
    return ARM_DRIVER_ERROR;
}

static ARM_I2C_STATUS LPI2C0_GetStatus(void)
{
    return ARM_LPI2C_GetStatus(&LPI2C0_RES);
}

/* LPI2C Driver Control Block */
extern ARM_DRIVER_I2C Driver_LPI2C0;
ARM_DRIVER_I2C        Driver_LPI2C0 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    LPI2C0_Initialize,
    LPI2C0_Uninitialize,
    LPI2C0_PowerControl,
    LPI2C0_MasterTransmit,
    LPI2C0_MasterReceive,
    LPI2C0_SlaveTransmit,
    LPI2C0_SlaveReceive,
    LPI2C0_GetDataCount,
    LPI2C0_Control,
    LPI2C0_GetStatus
};

#endif /*(RTE_LPI2C0)*/
#endif /* RTE_Drivers_LPI2C0 */
