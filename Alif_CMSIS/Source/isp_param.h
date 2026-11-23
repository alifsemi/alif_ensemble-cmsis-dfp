/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     isp_param.h
 * @brief    ISP Sensor Parameter definitions.
 *
 *           ISP_SENSOR_PARAMS and helpers shared by sensor-specific
 *           *_isp_param.c files. Each sensor defines its own
 *           ISP_SENSOR_PARAMS object (calibration, port, channel, AE defaults)
 *           and attaches it via CAMERA_SENSOR_DEVICE::isp_params.
 *
 *           To add support for a new sensor:
 *           1. Create <sensor_name>_isp_param.c defining the ISP_SENSOR_PARAMS
 *              object and related calibration/port/channel/AE data.
 *           2. Declare extern const ISP_SENSOR_PARAMS <sensor>_isp_params in the
 *              sensor driver and attach it as .isp_params.
 *           3. Add the file to the sensor's component entry in the PDSC file.
 *           Driver_ISP.c does NOT need to be modified.
 ******************************************************************************/

#ifndef ISP_PARAM_H_
#define ISP_PARAM_H_

#include "vsios_type.h"
#include "vsi_comm_isp.h"
#include "vsi_comm_sns.h"
#include "isp.h"
#include "mpi_isp_calib.h"
#include "vsi_comm_awb.h"

/**
 * \brief ISP Sensor Parameters
 */
typedef struct ISP_SENSOR_PARAMS {
    ISP_CALIB_DATA_S       *calib;       /**< Calibration information of ISP modules */
    ISP_PORT_ATTR_S        *port;        /**< ISP Port Attributes */
    ISP_CHN_ATTR_S         *chan;        /**< ISP Channel Attributes */
    const AE_SNS_DEFAULT_S *ae_defaults; /**< AE Sensor Defaults */
} ISP_SENSOR_PARAMS;

void isp_param_set_crop(vsi_u32_t top, vsi_u32_t left, vsi_u32_t width, vsi_u32_t height);
void isp_param_set_output_dimensions(vsi_u32_t width, vsi_u32_t height);

#endif /* ISP_PARAM_H_ */
