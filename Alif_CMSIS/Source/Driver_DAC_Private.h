/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DRIVER_DAC_PRIVATE_H_
#define DRIVER_DAC_PRIVATE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/* System includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* Project includes */
#include "dac.h"
#include "Driver_DAC.h"
#include "sys_ctrl_dac.h"

/**
 @brief   : DAC flags to check the DAC initialization, DAC power done and DAC started.
 */
typedef enum {
    DAC_FLAG_DRV_INIT_DONE    = (1U << 0),  /* DAC Driver is Initialized */
    DAC_FLAG_DRV_POWER_DONE   = (1U << 1),  /* DAC Driver is Powered */
    DAC_FLAG_DRV_STARTED      = (1U << 2)   /* DAC Driver is Started */
} DAC_DRIVER_STATE;

/**
 * struct DAC_RESOURCES: structure representing a DAC device
 * @regs     : Register address of the DAC
 * @flags    : DAC driver flags
 * @config   : DAC configuration information
 */
typedef struct _DAC_resources
{
    DAC120_Type         *regs;         /* DAC register address */
    DAC_DRIVER_STATE     flags;        /* DAC Driver Flags */
    DAC_INSTANCE         instance;     /* DAC Driver instance */
    uint16_t             bypass_val;   /* DAC input data in bypass mode */
    uint8_t              input_mux_val;/* DAC input data source  */
}DAC_RESOURCES;

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_DAC_PRIVATE_H_ */
