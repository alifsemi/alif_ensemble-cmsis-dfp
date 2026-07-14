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
 * @brief    Extern declarations for ISP default configuration parameters.
 *           The actual definitions are in the sensor-specific isp_param file
 *           (e.g. mt9m114_isp_param.c, arx3a0_isp_param.c, ov5675_isp_param.c)
 *           selected by the RTE camera sensor component.
 *
 *           To add support for a new sensor:
 *           1. Create <sensor_name>_isp_param.c defining calibration_data,
 *              port_attr, and chan_attr.
 *           2. Add the file to the sensor's component entry in the PDSC file.
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

/* Default ISP calibration data - defined in sensor-specific *_isp_param.c */
extern ISP_CALIB_DATA_S calibration_data;

/* Default ISP port attributes - defined in sensor-specific *_isp_param.c */
extern ISP_PORT_ATTR_S  port_attr;

/* Default ISP channel attributes - defined in sensor-specific *_isp_param.c */
extern ISP_CHN_ATTR_S   chan_attr;

/* Runtime crop and output configuration - defined in sensor-specific *_isp_param.c */
void isp_param_set_crop(vsi_u32_t top, vsi_u32_t left, vsi_u32_t width, vsi_u32_t height);
void isp_param_set_output_dimensions(vsi_u32_t width, vsi_u32_t height);

#endif /* ISP_PARAM_H_ */
