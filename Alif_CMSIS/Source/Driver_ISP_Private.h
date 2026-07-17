/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     Driver_ISP_Private.h
 * @author   Yogender Kumar Arya
 * @email    yogender.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     06-Sep-2025
 * @brief    ISP driver Specific Header file.
 ******************************************************************************/

#ifndef DRIVER_ISP_PRIVATE_H_
#define DRIVER_ISP_PRIVATE_H_

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_ISP.h"
#include "vsi_comm_video.h"
#include "vsi_comm_isp.h"
#include "vsi_comm_sns.h"
#include "vsi_comm_calib.h"
#include "mpi_isp.h"

#include "mpi_isp_binning.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief ISP Driver states. */
typedef volatile struct _ISP_DRIVER_STATE {
    uint32_t initialized: 1;  /**< Driver Initialized    */
    uint32_t powered    : 1;  /**< Driver powered        */
    uint32_t configured : 1;  /**< Driver configured     */
    uint32_t streaming  : 1;  /**< Streaming on          */
    uint32_t reserved   : 28; /**< Reserved              */
} ISP_DRIVER_STATE;

/** \brief Resources for a ISP instance */
typedef struct _ISP_RESOURCES {
    //    ISP_Type       *regs;           /**< Pointer to regs                  */
    ISP_CALIB_DATA_S     *isp_calib_info; /**< Calibration information of ISP modules */
    ISP_PORT_ATTR_S      *isp_port_attr;  /**< ISP Port Attributes              */
    ISP_CHN_ATTR_S       *isp_chan_attr;
    ARM_ISP_SignalEvent_t cb_event;       /**< Pointer to call back function    */
    uint32_t              irq_priority;   /**< Interrupt priority               */
    ISP_DEV               isp_dev_id;     /**< ISP device ID                    */
    ISP_PORT              isp_port_id;
    ISP_CHN               isp_chn_id;
    ISP_BINNING_ATTR_S   *isp_binning_attr; /**< ISP Binning Attributes         */
    ISP_DRIVER_STATE      state;          /**< ISP driver status                */
} ISP_RESOURCES;

extern int VSI_MPI_ISP_Init(ISP_DEV IspDev);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_ISP_PRIVATE_H_ */
