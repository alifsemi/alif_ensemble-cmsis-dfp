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
 * @file         Driver_ISP.c
 * @author       Yogender Kumar Arya
 * @email        yogender.kumar@alifsemi.com
 * @version      V1.0.0
 * @date         06-September-2025
 * @brief        CMSIS-Driver for Image Signal Processor
 * @bug          None.
 * @Note         None.
 ******************************************************************************/

/* System Includes */
#include <string.h>
#include "RTE_Device.h"
#include "sys_utils.h"
#include "soc.h"
#include "isp_ctrl_params.h"
#include "Driver_ISP_Private.h"

#if defined(RTE_Drivers_ISP)

/* Project Includes */
#include "Camera_Sensor.h"

/* ISP Includes */
#include "vsi_comm_isp.h"
#include "vsios_type.h"
#include "isp.h"

#include "mpi_isp.h"
#include "mpi_isp_calib.h"
#include "vsi_comm_awb.h"
#include "vsios_log.h"

#if (RTE_ISP_AE_MODULE)
#include "vsi_comm_ae.h"
#include "vsi_comm_sns.h"
#include "sensor_attributes.h"
#endif /* RTE_ISP_AE_MODULE */

#if (RTE_ISP_BINNING_MODULE)
#include "mpi_isp_binning.h"
#endif /* RTE_ISP_BINNING_MODULE */

#if (RTE_ISP_BLS_MODULE)
#include "mpi_isp_bls.h"
#endif /* RTE_ISP_BLS_MODULE */

#if (RTE_ISP_DMSC_MODULE)
#include "mpi_isp_dmsc.h"
#endif /* RTE_ISP_DMSC_MODULE */

#if (RTE_ISP_FLT_MODULE)
#include "mpi_isp_flt.h"
#endif /* RTE_ISP_FLT_MODULE */

#if (RTE_ISP_CCM_MODULE)
#include "mpi_isp_ccm.h"
#endif /* RTE_ISP_CCM_MODULE */

#if (RTE_ISP_GAMMAOUT_MODULE)
#include "mpi_isp_gamma_out.h"
#endif /* RTE_ISP_GAMMAOUT_MODULE */

#if (RTE_ISP_CSM_MODULE)
#include "mpi_isp_csm.h"
#endif /* RTE_ISP_CSM_MODULE */

#if (RTE_ISP_EXPM_MODULE)
#include "mpi_isp_expm.h"
#endif /* RTE_ISP_EXPM_MODULE */

#if (RTE_ISP_WBM_MODULE)
#include "mpi_isp_wbm.h"
#endif /* RTE_ISP_WBM_MODULE */

/* CMSIS ISP driver Includes */
#include "Driver_ISP.h"

#include "isp_param.h"

extern void VSI_ISP_IrqProcessFrameEnd(ISP_PORT IspPort);

#if (RTE_ISP_WB_MODULE)
extern ISP_AWB_FUNC_S vsiAwbAlgo;
#endif /* RTE_ISP_WB_MODULE */

#if (RTE_ISP_AE_MODULE)
extern ISP_AE_FUNC_S vsiAeAlgo;
#endif /* RTE_ISP_AE_MODULE */

#define ARM_ISP_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(0, 1) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = { ARM_ISP_API_VERSION, ARM_ISP_DRV_VERSION };

/* Driver Capabilities */
static const ARM_ISP_CAPABILITIES DriverCapabilities = {
    1, /* AE supported */
    1, /* BLS */
    1, /* Demosaic */
    1, /* Filter */
    1, /* CCM */
    1, /* CSM */
    1, /* White Balancing */
    1, /* ae_stat - Auto-Exposure Statistics */
    1, /* Gamma-out */
    1, /* wb_stat - White-Balancing Statistics */
    1, /* Binning */
    1, /* Scaling */
    0  /* Reserved (must be zero) */
};

#define LIB_LOG_LEVEL VSI_LOG_LEVEL_INFO

static int log_level(void)
{
    return LIB_LOG_LEVEL;
}

#if (RTE_ISP_AE_MODULE)
/* Cached sensor values for AE - written in ISR context, read in thread context.
 * All fields are written only from ISP_IRQHandler; readers must hold a
 * critical section (PRIMASK) while snapshotting.
 */
static struct {
    uint32_t intLine;
    uint32_t again;
    uint32_t dgain;
} ae_sensor_cache = {0};

/*
 * AE Sensor Function Callbacks - bridge ISP AE algorithm to CMSIS sensor driver
 */

static int AE_GetDefaults(ISP_PORT IspPort, AE_SNS_DEFAULT_S *pAeSnsDft)
{
    (void)IspPort;
    if (!pAeSnsDft) {
        return -1;
    }
    *pAeSnsDft = sensor_attributes;
    pAeSnsDft->linesPer500ms =
        pAeSnsDft->fullLines * pAeSnsDft->fps / (2 * ISP_SNS_FPS_ACCU);
    return 0;
}

static int AE_IntTimeUpdate(ISP_PORT IspPort, vsi_u32_t *pIntLine)
{
    (void)IspPort;
    if (!pIntLine) {
        return -1;
    }

    ae_sensor_cache.intLine = *pIntLine;
    return 0;
}

static int AE_GainUpdate(ISP_PORT IspPort, vsi_u32_t *pAgain, vsi_u32_t *pDgain)
{
    (void)IspPort;
    if (!pAgain || !pDgain) {
        return -1;
    }

    ae_sensor_cache.again = *pAgain;
    ae_sensor_cache.dgain = *pDgain;
    return 0;
}


static int AE_QueryExpInfo(ISP_PORT IspPort, vsi_u32_t *pIntLine,
                           vsi_u32_t *pAgain, vsi_u32_t *pDgain)
{
    (void)IspPort;
    if (!pIntLine || !pAgain || !pDgain) {
        return -1;
    }
    *pIntLine = ae_sensor_cache.intLine;
    *pAgain = ae_sensor_cache.again;
    *pDgain = ae_sensor_cache.dgain;
    return 0;
}

/*
 * Public API functions for application
 */

int ISP_Sensor_AEGetCachedValues(uint32_t *pIntLine, uint32_t *pAgain, uint32_t *pDgain)
{
    uint32_t primask;

    if (!pIntLine || !pAgain || !pDgain) {
        return -1;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *pIntLine = ae_sensor_cache.intLine;
    *pAgain   = ae_sensor_cache.again;
    *pDgain   = ae_sensor_cache.dgain;
    __set_PRIMASK(primask);
    return 0;
}

#endif /* RTE_ISP_AE_MODULE */


#if (RTE_ISP_AE_MODULE)
int ISP_Sensor_AEIsStable(ISP_RESOURCES *isp)
{
    int ret = 0;
    ISP_EXPOSURE_INFO_S info = {0};

    ret = VSI_MPI_ISP_QueryExposureInfo(isp->isp_port_id, &info);
    if (ret) {
        return -1;
    }
    return (int)info.isStable;
}
#endif /* RTE_ISP_AE_MODULE */

/*
 * fn        ARM_DRIVER_VERSION ISP_GetVersion(void)
 * brief     get ISP version
 * return    ISP version
 */
static ARM_DRIVER_VERSION ISP_GetVersion(void)
{
    return DriverVersion;
}

/*
 * fn        ARM_ISP_CAPABILITIES ISP_GetCapabilities(void)
 * brief     get ISP capabilities
 * return    ISP driver capabilities
 */
static ARM_ISP_CAPABILITIES ISP_GetCapabilities(void)
{
    return DriverCapabilities;
}

/*
 * fn        int32_t ISP_Init (ARM_ISP_SignalEvent_t cb_event,
 *                              ISP_RESOURCES *isp)
 * brief     Initialize ISP Interface.
 * param[in] cb_event pointer to ARM_ISP_SignalEvent_t.
 * param[in] isp Pointer to ISP resource.
 * return    @ref execution_status.
 */
static int32_t ISP_Init(ARM_ISP_SignalEvent_t cb_event, CAMERA_SENSOR_DEVICE *cam_sensor,
                        ISP_RESOURCES *isp)
{
    int32_t         ret;
    ISP_DEV_ATTR_S  devAttr;
    ISP_PORT_ATTR_S isp_port_config = {};

    if (isp->state.initialized == 1) {
        return ARM_DRIVER_OK;
    }

    if (!cb_event) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    isp->cb_event = cb_event;
    isp->state.streaming = 0;

    /* Pass a print function instead of NULL to enable ISP log output. */
    VsiLogLevelSet(&log_level, NULL);

    /* Init ISP system. */
    ret           = VSI_MPI_ISP_Init(isp->isp_dev_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    /* Configure ISP work-mode. */
    devAttr.ispWorkMode = WORK_MODE_NORMAL;
    ret                 = VSI_MPI_ISP_SetDevAttr(isp->isp_dev_id, &devAttr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    ret = VSI_MPI_ISP_SetCalib(isp->isp_port_id, isp->isp_calib_info);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    // Technically it should be set when we are setting AWB.
    // The algorithm must be first registered and then Init.
#if (RTE_ISP_WB_MODULE)
    if (isp->isp_calib_info->modules.wb.opType == OP_TYPE_AUTO) {
        ret = VSI_MPI_ISP_AwbRegCallBack(isp->isp_port_id, &vsiAwbAlgo);
        if (ret) {
            return ARM_DRIVER_ERROR;
        }
    }
#endif /* RTE_ISP_WB_MODULE */

#if (RTE_ISP_AE_MODULE)
    /* Register AE sensor function table */
    {
        AE_SNS_FUNC_S aeSnsFunc = {0};

        aeSnsFunc.pfnGetAeDefault  = AE_GetDefaults;
        aeSnsFunc.pfnIntTimeUpdate = AE_IntTimeUpdate;
        aeSnsFunc.pfnGainUpdate    = AE_GainUpdate;
        aeSnsFunc.pfnQueryExpInfo  = AE_QueryExpInfo;

        ret = VSI_MPI_ISP_InitAeSnsFunc(isp->isp_port_id, &aeSnsFunc);
        if (ret) {
            return ARM_DRIVER_ERROR;
        }
    }

    /* Register AE algorithm if auto mode */
    if (isp->isp_calib_info->modules.ae.opType == OP_TYPE_AUTO) {
        ret = VSI_MPI_ISP_AeRegCallBack(isp->isp_port_id, &vsiAeAlgo);
        if (ret) {
            return ARM_DRIVER_ERROR;
        }
    }
#endif /* RTE_ISP_AE_MODULE */

#if (RTE_ISP_BINNING_MODULE)
    if (isp->isp_binning_attr && isp->isp_binning_attr->enable) {
        ret = VSI_MPI_ISP_SetBinningAttr(isp->isp_port_id, isp->isp_binning_attr);
        if (ret) {
            return ARM_DRIVER_ERROR;
        }
    }
#endif /* RTE_ISP_BINNING_MODULE */

    ret = VSI_MPI_ISP_GetPortAttr(isp->isp_port_id, &isp_port_config);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    // Update ISP input rectangles with runtime sensor dimensions:
    // snsRect: Sensor rectangle
    // inFormRect: Input format rectangle from sensor
    isp->isp_port_attr->snsRect.width     = cam_sensor->width;
    isp->isp_port_attr->snsRect.height    = cam_sensor->height;
    isp->isp_port_attr->inFormRect.width  = cam_sensor->width;
    isp->isp_port_attr->inFormRect.height = cam_sensor->height;
    isp->isp_port_attr->iSRect.width      = cam_sensor->width;
    isp->isp_port_attr->iSRect.height     = cam_sensor->height;


    isp_port_config.ispInputType       = isp->isp_port_attr->ispInputType;
    isp_port_config.ispMode            = isp->isp_port_attr->ispMode;
    isp_port_config.hdrMode            = isp->isp_port_attr->hdrMode;
    isp_port_config.stichMode          = isp->isp_port_attr->stichMode;
    isp_port_config.pixelFormat        = isp->isp_port_attr->pixelFormat;

    isp_port_config.snsRect.top        = isp->isp_port_attr->snsRect.top;
    isp_port_config.snsRect.left       = isp->isp_port_attr->snsRect.left;
    isp_port_config.snsRect.width      = isp->isp_port_attr->snsRect.width;
    isp_port_config.snsRect.height     = isp->isp_port_attr->snsRect.height;

    isp_port_config.inFormRect.top     = isp->isp_port_attr->inFormRect.top;
    isp_port_config.inFormRect.left    = isp->isp_port_attr->inFormRect.left;
    isp_port_config.inFormRect.width   = isp->isp_port_attr->inFormRect.width;
    isp_port_config.inFormRect.height  = isp->isp_port_attr->inFormRect.height;

    /* Compute centered square crop from runtime sensor dimensions.
     * RECT_S field naming is swapped in the libisp:
     * RECT_S.top  -> ISP_OUT_H_OFFS (horizontal/left offset)
     * RECT_S.left -> ISP_OUT_V_OFFS (vertical/top offset)
     */
    vsi_u32_t side = cam_sensor->width < cam_sensor->height
                   ? cam_sensor->width : cam_sensor->height;
    vsi_u32_t v_off = cam_sensor->height > side ? (cam_sensor->height - side) / 2 : 0;
    vsi_u32_t h_off = cam_sensor->width  > side ? (cam_sensor->width  - side) / 2 : 0;

    isp->isp_port_attr->outFormRect.top    = h_off;  /* libisp: horizontal offset */
    isp->isp_port_attr->outFormRect.left   = v_off;  /* libisp: vertical offset */
    isp->isp_port_attr->outFormRect.width  = side;
    isp->isp_port_attr->outFormRect.height = side;

    isp_port_config.outFormRect.top    = isp->isp_port_attr->outFormRect.top;
    isp_port_config.outFormRect.left   = isp->isp_port_attr->outFormRect.left;
    isp_port_config.outFormRect.width  = isp->isp_port_attr->outFormRect.width;
    isp_port_config.outFormRect.height = isp->isp_port_attr->outFormRect.height;

    isp_port_config.iSRect.top         = isp->isp_port_attr->iSRect.top;
    isp_port_config.iSRect.left        = isp->isp_port_attr->iSRect.left;
    isp_port_config.iSRect.width       = isp->isp_port_attr->iSRect.width;
    isp_port_config.iSRect.height      = isp->isp_port_attr->iSRect.height;

    isp_port_config.snsFps             = 0;

    ret = VSI_MPI_ISP_SetPortAttr(isp->isp_port_id, &isp_port_config);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    /* Patch AE measurement window in calib struct to match the finalized crop
     * before SetCalib writes it to HW.
     */
    isp->isp_calib_info->modules.aem.blockWin.hOffs = isp->isp_port_attr->outFormRect.top;
    isp->isp_calib_info->modules.aem.blockWin.vOffs = isp->isp_port_attr->outFormRect.left;
    isp->isp_calib_info->modules.aem.blockWin.hSize = isp->isp_port_attr->outFormRect.width;
    isp->isp_calib_info->modules.aem.blockWin.vSize = isp->isp_port_attr->outFormRect.height;

    /* Patch WB measurement window in calib struct to match the finalized crop
     * before SetCalib writes it to HW.
     */
    isp->isp_calib_info->modules.wbm.measRect.hOffs = isp->isp_port_attr->outFormRect.top;
    isp->isp_calib_info->modules.wbm.measRect.vOffs = isp->isp_port_attr->outFormRect.left;
    isp->isp_calib_info->modules.wbm.measRect.hSize = isp->isp_port_attr->outFormRect.width;
    isp->isp_calib_info->modules.wbm.measRect.vSize = isp->isp_port_attr->outFormRect.height;

    ret = VSI_MPI_ISP_SetCalib(isp->isp_port_id, isp->isp_calib_info);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    // Set-up ISP channel attributes which control the way ISP output data to memory.
    ret = VSI_MPI_ISP_SetChnAttr(isp->isp_chn_id, isp->isp_chan_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    isp->state.initialized = 1;

    return ARM_DRIVER_OK;
}

/*
 *  fn        int32_t ISP_Uninit (ISP_RESOURCES *isp)
 *  brief     Uninitialize ISP Interface.
 *  param[in] isp Pointer to ISP resource.
 *  return    @ref execution_status.
 */
static int32_t ISP_Uninit(ISP_RESOURCES *isp)
{
    int32_t ret;

    if (isp->state.streaming == 1) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (isp->state.powered == 1) {
        return ARM_DRIVER_ERROR;
    }

    if (isp->state.initialized == 0) {
        return ARM_DRIVER_OK;
    }

#if (RTE_ISP_WB_MODULE)
    ret = VSI_MPI_ISP_AwbUnRegCallBack(isp->isp_port_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
#endif /* RTE_ISP_WB_MODULE */

#if (RTE_ISP_AE_MODULE)
    ret = VSI_MPI_ISP_AeUnRegCallBack(isp->isp_port_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
#endif /* RTE_ISP_AE_MODULE */

    ret = VSI_MPI_ISP_Exit(isp->isp_dev_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    isp->cb_event = NULL;
    isp->state.initialized = 0;

    return ARM_DRIVER_OK;
}

/*
 * fn        int32_t ISP_PowerCtrl (ARM_POWER_STATE state,
 *                                   ISP_RESOURCES *isp)
 * brief     Control ISP Interface Power.
 * param[in] state Power state.
 * return    @ref execution_status.
 */
static int32_t ISP_PowerCtrl(ARM_POWER_STATE state, ISP_RESOURCES *isp)
{
    if (isp->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    switch (state) {
    case ARM_POWER_OFF:
        if (isp->state.streaming == 1) {
            return ARM_DRIVER_ERROR_BUSY;
        }

        if (isp->state.powered == 0) {
            return ARM_DRIVER_OK;
        }

        /* Disabling IRQ */
        NVIC_DisableIRQ(ISP_IRQ_IRQn);
        NVIC_DisableIRQ(ISP_MI_IRQ_IRQn);
        NVIC_ClearPendingIRQ(ISP_IRQ_IRQn);
        NVIC_ClearPendingIRQ(ISP_MI_IRQ_IRQn);

        isp->state.powered = 0;
        break;

    case ARM_POWER_FULL:
        if (isp->state.powered == 1) {
            return ARM_DRIVER_OK;
        }

        /* Enabling IRQ */
        NVIC_ClearPendingIRQ(ISP_IRQ_IRQn);
        NVIC_ClearPendingIRQ(ISP_MI_IRQ_IRQn);
        NVIC_SetPriority(ISP_IRQ_IRQn, isp->irq_priority);
        NVIC_SetPriority(ISP_MI_IRQ_IRQn, isp->irq_priority);
        NVIC_EnableIRQ(ISP_IRQ_IRQn);
        NVIC_EnableIRQ(ISP_MI_IRQ_IRQn);

        isp->state.powered = 1;
        break;

    case ARM_POWER_LOW:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/*
 * fn        int32_t ISP_start(ISP_RESOURCES *isp)
 * brief     Start ISP Image capture.
 * return    @ref execution_status.
 */
static int32_t ISP_start(ISP_RESOURCES *isp)
{
    int32_t ret = 0;

    if (isp->state.streaming == 1)  {
        return ARM_DRIVER_OK;
    }

    // Library call to start capture using ISP library

    ret = VSI_MPI_ISP_EnableDev(isp->isp_dev_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    ret = VSI_MPI_ISP_EnableChn(isp->isp_chn_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    ret = VSI_MPI_ISP_EnablePort(isp->isp_port_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    isp->state.streaming = 1;
    return ARM_DRIVER_OK;
}

/*
 * fn        int32_t ISP_stop(ISP_RESOURCES *isp)
 * brief     Stop ISP Image capture.
 * return    @ref execution_status.
 */
static int32_t ISP_stop(ISP_RESOURCES *isp)
{
    int32_t ret = 0;

    if (isp->state.streaming == 0) {
        return ARM_DRIVER_OK;
    }

    // Library call to stop capture using ISP library
    ret = VSI_MPI_ISP_DisablePort(isp->isp_port_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    ret = VSI_MPI_ISP_DisableChn(isp->isp_chn_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    ret = VSI_MPI_ISP_DisableDev(isp->isp_dev_id);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }

    isp->state.streaming = 0;

    return ARM_DRIVER_OK;
}
#if (RTE_ISP_AE_MODULE)
static int32_t ISP_set_param_ae(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_AE)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_ae_param *ae = &params->ae;
    ISP_EXPOSURE_ATTR_S exp_attr = {0};

    exp_attr.opType = (ae->op_mode == ISP_OP_AUTO) ? OP_TYPE_AUTO : OP_TYPE_MANUAL;
    exp_attr.manualAttr.intTime = ae->int_time;
    exp_attr.manualAttr.again   = ae->again;
    exp_attr.manualAttr.dgain   = ae->dgain;
    exp_attr.autoAttr.expTimeRange.min = ae->int_time_min;
    exp_attr.autoAttr.expTimeRange.max = ae->int_time_max;
    exp_attr.autoAttr.againRange.min   = ae->again_min;
    exp_attr.autoAttr.againRange.max   = ae->again_max;
    exp_attr.autoAttr.dgainRange.min   = ae->dgain_min;
    exp_attr.autoAttr.dgainRange.max   = ae->dgain_max;
    exp_attr.autoAttr.aeTarget         = ae->ae_target;
    exp_attr.autoAttr.dampOver         = ae->damp_over;
    exp_attr.autoAttr.dampUnder        = ae->damp_under;
    exp_attr.autoAttr.tolerance        = ae->tolerance;
    exp_attr.autoAttr.aeRunInterval    = ae->run_interval;
    exp_attr.autoAttr.antiflicker.enable      = ae->antiflicker_enable;
    exp_attr.autoAttr.antiflicker.flickerFreq = ae->antiflicker_freq;
    exp_attr.autoAttr.gainThreshold = ae->gain_threshold;
    exp_attr.autoAttr.aeMode        = (ISP_AE_MODE_E)ae->ae_mode;
    exp_attr.autoAttr.aeDelayAttr.blackDelayFrame = ae->ae_delay_attr.black_delay_frame;
    exp_attr.autoAttr.aeDelayAttr.whiteDelayFrame = ae->ae_delay_attr.white_delay_frame;
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
            exp_attr.autoAttr.weight[r][c] = ae->weight[r][c];
        }
    }
    if (ae->ae_route_total_nodes > ISP_AE_ROUTE_MAX_NODES) {
        exp_attr.autoAttr.aeRoute.totalNum = ISP_AE_ROUTE_MAX_NODES;
    } else {
        exp_attr.autoAttr.aeRoute.totalNum = ae->ae_route_total_nodes;
    }
    for (int i = 0; i < (int)exp_attr.autoAttr.aeRoute.totalNum; i++) {
        exp_attr.autoAttr.aeRoute.routeNode[i].intTime = ae->ae_route[i].int_time;
        exp_attr.autoAttr.aeRoute.routeNode[i].again   = ae->ae_route[i].again;
        exp_attr.autoAttr.aeRoute.routeNode[i].dgain   = ae->ae_route[i].dgain;
    }
    ret = VSI_MPI_ISP_SetExposureAttr(isp->isp_port_id, &exp_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_WB_MODULE)
static int32_t ISP_set_param_wb(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_WB)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_wb_param *wb = &params->wb;
    ISP_WB_ATTR_S wb_attr = {0};

    wb_attr.enable = wb->enable;
    wb_attr.opType = (wb->op_mode == ISP_OP_AUTO) ? OP_TYPE_AUTO : OP_TYPE_MANUAL;
    wb_attr.manualAttr.wbGain.rGain  = wb->r_gain;
    wb_attr.manualAttr.wbGain.grGain = wb->gr_gain;
    wb_attr.manualAttr.wbGain.gbGain = wb->gb_gain;
    wb_attr.manualAttr.wbGain.bGain  = wb->b_gain;
    wb_attr.autoAttr.runInterval    = wb->run_interval;
    wb_attr.autoAttr.speed          = wb->speed;
    wb_attr.autoAttr.tolerance      = wb->tolerance;
    wb_attr.autoAttr.initColorTemp  = wb->init_color_temp;
    wb_attr.autoAttr.calibParam.centLine.rgParam   = wb->calib.center_line.rg_param;
    wb_attr.autoAttr.calibParam.centLine.bgParam   = wb->calib.center_line.bg_param;
    wb_attr.autoAttr.calibParam.centLine.distParam = wb->calib.center_line.dist_param;
    wb_attr.autoAttr.calibParam.rgMin = wb->calib.rg_min;
    wb_attr.autoAttr.calibParam.rgMax = wb->calib.rg_max;
    for (int i = 0; i < 16; i++) {
        wb_attr.autoAttr.calibParam.wpRange0.wpLCurve.rg[i]   = wb->calib.wp_range0.left.rg[i];
        wb_attr.autoAttr.calibParam.wpRange0.wpLCurve.dist[i] = wb->calib.wp_range0.left.dist[i];
        wb_attr.autoAttr.calibParam.wpRange0.wpRCurve.rg[i]   = wb->calib.wp_range0.right.rg[i];
        wb_attr.autoAttr.calibParam.wpRange0.wpRCurve.dist[i] = wb->calib.wp_range0.right.dist[i];
        wb_attr.autoAttr.calibParam.wpRange1.wpLCurve.rg[i]   = wb->calib.wp_range1.left.rg[i];
        wb_attr.autoAttr.calibParam.wpRange1.wpLCurve.dist[i] = wb->calib.wp_range1.left.dist[i];
        wb_attr.autoAttr.calibParam.wpRange1.wpRCurve.rg[i]   = wb->calib.wp_range1.right.rg[i];
        wb_attr.autoAttr.calibParam.wpRange1.wpRCurve.dist[i] = wb->calib.wp_range1.right.dist[i];
    }
    for (int i = 0; i < ILLUMINANT_TYPE_CNT; i++) {
        wb_attr.autoAttr.calibParam.illuminant[i].illuType  =
            (ISP_ILLUMINANT_TYPE_E)wb->calib.illuminant[i].illu_type;
        wb_attr.autoAttr.calibParam.illuminant[i].colorTemp = wb->calib.illuminant[i].color_temp;
        wb_attr.autoAttr.calibParam.illuminant[i].wbGain.rGain  = wb->calib.illuminant[i].r_gain;
        wb_attr.autoAttr.calibParam.illuminant[i].wbGain.grGain = wb->calib.illuminant[i].gr_gain;
        wb_attr.autoAttr.calibParam.illuminant[i].wbGain.gbGain = wb->calib.illuminant[i].gb_gain;
        wb_attr.autoAttr.calibParam.illuminant[i].wbGain.bGain  = wb->calib.illuminant[i].b_gain;
    }
    ret = VSI_MPI_ISP_SetWbAttr(isp->isp_port_id, &wb_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_BLS_MODULE)
static int32_t ISP_set_param_bls(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_BLS)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_bls_param *bls = &params->bls;
    ISP_BLS_ATTR_S bls_attr = {0};

    bls_attr.enable = bls->enable;
    bls_attr.opType = (bls->op_mode == ISP_OP_AUTO) ? OP_TYPE_AUTO : OP_TYPE_MANUAL;
    for (int i = 0; i < 4; i++) {
        bls_attr.manualAttr.blackLevel[i] = bls->black_level[i];
    }
    for (int i = 0; i < ISP_AUTO_STRENGTH_NUN; i++) {
        for (int j = 0; j < 4; j++) {
            bls_attr.autoAttr.blackLevel[i][j] = bls->auto_black_level[i][j];
        }
    }
    ret = VSI_MPI_ISP_SetBlsAttr(isp->isp_port_id, &bls_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_DMSC_MODULE)
static int32_t ISP_set_param_dmsc(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_DMSC)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_dmsc_param *dmsc = &params->dmsc;
    ISP_DMSC_ATTR_S dmsc_attr = {0};

    dmsc_attr.enable           = dmsc->enable;
    dmsc_attr.threshold        = dmsc->threshold;
    dmsc_attr.cacAttr.enable   = dmsc->cac_enable;
    dmsc_attr.cacAttr.hClipMode = dmsc->cac_h_clip_mode;
    dmsc_attr.cacAttr.vClipMode = dmsc->cac_v_clip_mode;
    dmsc_attr.cacAttr.hStart    = dmsc->cac_h_start;
    dmsc_attr.cacAttr.vStart    = dmsc->cac_v_start;
    dmsc_attr.cacAttr.aBlue     = dmsc->cac_a_blue;
    dmsc_attr.cacAttr.aRed      = dmsc->cac_a_red;
    dmsc_attr.cacAttr.bBlue     = dmsc->cac_b_blue;
    dmsc_attr.cacAttr.bRed      = dmsc->cac_b_red;
    dmsc_attr.cacAttr.cBlue     = dmsc->cac_c_blue;
    dmsc_attr.cacAttr.cRed      = dmsc->cac_c_red;
    dmsc_attr.cacAttr.xNormShift  = dmsc->cac_x_norm_shift;
    dmsc_attr.cacAttr.xNormFactor = dmsc->cac_x_norm_factor;
    dmsc_attr.cacAttr.yNormShift  = dmsc->cac_y_norm_shift;
    dmsc_attr.cacAttr.yNormFactor = dmsc->cac_y_norm_factor;
    ret = VSI_MPI_ISP_SetDmscAttr(isp->isp_port_id, &dmsc_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_FLT_MODULE)
static int32_t ISP_set_param_flt(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_FLT)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_flt_param *flt = &params->flt;
    ISP_FLT_ATTR_S flt_attr = {0};

    flt_attr.enable  = flt->enable;
    flt_attr.opType  = (flt->op_mode == ISP_OP_AUTO) ? OP_TYPE_AUTO : OP_TYPE_MANUAL;
    flt_attr.manualAttr.denoiseLevel = flt->denoise_level;
    flt_attr.manualAttr.sharpenLevel  = flt->sharpen_level;
    for (int i = 0; i < ISP_AUTO_STRENGTH_NUN; i++) {
        flt_attr.autoAttr.denoiseLevel[i] = flt->auto_denoise_level[i];
        flt_attr.autoAttr.sharpenLevel[i] = flt->auto_sharpen_level[i];
    }
    ret = VSI_MPI_ISP_SetFltAttr(isp->isp_port_id, &flt_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif


#if (RTE_ISP_CCM_MODULE)
static int32_t ISP_set_param_ccm(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_CCM)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_ccm_param *ccm = &params->ccm;
    ISP_CCM_ATTR_S ccm_attr = {0};

    ccm_attr.opType = (ccm->op_mode == ISP_OP_AUTO) ? OP_TYPE_AUTO : OP_TYPE_MANUAL;
    memcpy(ccm_attr.manualAttr.colorMatrix, ccm->color_matrix,
        sizeof(ccm_attr.manualAttr.colorMatrix));
    ccm_attr.manualAttr.rOffset = ccm->r_offset;
    ccm_attr.manualAttr.gOffset = ccm->g_offset;
    ccm_attr.manualAttr.bOffset = ccm->b_offset;
    for (int i = 0; i < ILLUMINANT_TYPE_CNT; i++) {
        memcpy(ccm_attr.autoAttr.illuminantCCM[i].colorMatrix,
            ccm->auto_ccm[i].color_matrix, sizeof(ccm_attr.autoAttr.illuminantCCM[i].colorMatrix));
        ccm_attr.autoAttr.illuminantCCM[i].rOffset   = ccm->auto_ccm[i].r_offset;
        ccm_attr.autoAttr.illuminantCCM[i].gOffset   = ccm->auto_ccm[i].g_offset;
        ccm_attr.autoAttr.illuminantCCM[i].bOffset   = ccm->auto_ccm[i].b_offset;
        ccm_attr.autoAttr.illuminantCCM[i].colorTemp = ccm->auto_ccm[i].color_temp;
    }
    ret = VSI_MPI_ISP_SetCcmAttr(isp->isp_port_id, &ccm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_GAMMAOUT_MODULE)
static int32_t ISP_set_param_gamma(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_GAMMA_OUT)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_gamma_param *gamma = &params->gamma_out;
    ISP_GAMMA_OUT_ATTR_S gamma_attr = {0};

    gamma_attr.enable = gamma->enable;
    memcpy(gamma_attr.gammaY, gamma->gamma_y, sizeof(gamma_attr.gammaY));
    ret = VSI_MPI_ISP_SetGammaOutAttr(isp->isp_port_id, &gamma_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_CSM_MODULE)
static int32_t ISP_set_param_csm(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_CSM)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_csm_param *csm = &params->csm;
    ISP_CSM_ATTR_S csm_attr = {0};

    csm_attr.type         = (ISP_CSM_TYPE_E)csm->type;
    csm_attr.quantization = (ISP_CSM_QUANTIZATION_E)csm->range;
    memcpy(csm_attr.coef, csm->coef, sizeof(csm_attr.coef));
    ret = VSI_MPI_ISP_SetCsmAttr(isp->isp_port_id, &csm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_EXPM_MODULE)
static int32_t ISP_set_param_expm(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_AEM)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_aem_param *aem = &params->aem;
    ISP_EXPM_ATTR_S expm_attr = {0};

    expm_attr.enable         = aem->enable;
    expm_attr.expAltMode     = aem->alt_mode;
    expm_attr.blockWin.hOffs = aem->h_offs;
    expm_attr.blockWin.vOffs = aem->v_offs;
    expm_attr.blockWin.hSize = aem->h_size;
    expm_attr.blockWin.vSize = aem->v_size;
    ret = VSI_MPI_ISP_SetExpmAttr(isp->isp_port_id, &expm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_WBM_MODULE)
static int32_t ISP_set_param_wbm(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_WBM)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_wbm_param *wbm = &params->wbm;
    ISP_WBM_ATTR_S wbm_attr = {0};

    wbm_attr.enable             = wbm->enable;
    wbm_attr.measMode           = (ISP_WBM_MODE_E)wbm->meas_mode;
    wbm_attr.measRect.hOffs     = wbm->h_offs;
    wbm_attr.measRect.vOffs     = wbm->v_offs;
    wbm_attr.measRect.hSize     = wbm->h_size;
    wbm_attr.measRect.vSize     = wbm->v_size;
    wbm_attr.wpRange.maxY       = wbm->max_y;
    wbm_attr.wpRange.refCr_MaxR = wbm->ref_cr_max_r;
    wbm_attr.wpRange.minY_MaxG  = wbm->min_y_max_g;
    wbm_attr.wpRange.refCb_MaxB = wbm->ref_cb_max_b;
    wbm_attr.wpRange.maxCSum    = wbm->max_c_sum;
    wbm_attr.wpRange.minC       = wbm->min_c;
    ret = VSI_MPI_ISP_SetWbmAttr(isp->isp_port_id, &wbm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_BINNING_MODULE)
static int32_t ISP_set_param_binning(ISP_RESOURCES *isp, const struct isp_params *params)
{
    if (!(params->valid_mask & ISP_PARAM_MASK_BINNING)) {
        return ARM_DRIVER_OK;
    }
    int ret;
    const struct isp_binning_param *binning = &params->binning;
    ISP_BINNING_ATTR_S binning_attr = {0};

    binning_attr.enable   = binning->enable;
    binning_attr.binHStep = binning->h_step;
    binning_attr.binVStep = binning->v_step;
    ret = VSI_MPI_ISP_SetBinningAttr(isp->isp_port_id, &binning_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}
#endif


static int32_t ISP_set_param(ISP_RESOURCES *isp, const struct isp_params *params)
{
    int32_t ret = ARM_DRIVER_OK;

    if (!params) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
#if (RTE_ISP_AE_MODULE)
    ret = ISP_set_param_ae(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_WB_MODULE)
    ret = ISP_set_param_wb(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_BLS_MODULE)
    ret = ISP_set_param_bls(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_DMSC_MODULE)
    ret = ISP_set_param_dmsc(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_FLT_MODULE)
    ret = ISP_set_param_flt(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_CCM_MODULE)
    ret = ISP_set_param_ccm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_GAMMAOUT_MODULE)
    ret = ISP_set_param_gamma(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_CSM_MODULE)
    ret = ISP_set_param_csm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_EXPM_MODULE)
    ret = ISP_set_param_expm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_WBM_MODULE)
    ret = ISP_set_param_wbm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_BINNING_MODULE)
    ret = ISP_set_param_binning(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
    if (params->valid_mask & ISP_PARAM_MASK_AUTO_ROUTE) {
        ISP_AUTO_ROUTE_S route = {0};

        for (int i = 0; i < ISP_AUTO_STRENGTH_NUN; i++) {
            route.autoRoute[i] = params->auto_route.auto_route[i];
        }
        ret = VSI_MPI_ISP_SetAutoRoute(isp->isp_port_id, &route);
        if (ret != VSI_SUCCESS) {
            return ARM_DRIVER_ERROR;
        }
    }
    return ARM_DRIVER_OK;
}


#if (RTE_ISP_AE_MODULE)
static int32_t ISP_get_param_ae(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret = 0;
    ISP_EXPOSURE_ATTR_S exp_attr = {0};
    struct isp_ae_param *ae = &params->ae;

    if (!(params->valid_mask & ISP_PARAM_MASK_AE)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetExposureAttr(isp->isp_port_id, &exp_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    ae->op_mode      = (exp_attr.opType == OP_TYPE_AUTO) ? ISP_OP_AUTO : ISP_OP_MANUAL;
    ae->int_time     = exp_attr.manualAttr.intTime;
    ae->again        = exp_attr.manualAttr.again;
    ae->dgain        = exp_attr.manualAttr.dgain;
    ae->int_time_min = exp_attr.autoAttr.expTimeRange.min;
    ae->int_time_max = exp_attr.autoAttr.expTimeRange.max;
    ae->again_min    = exp_attr.autoAttr.againRange.min;
    ae->again_max    = exp_attr.autoAttr.againRange.max;
    ae->dgain_min    = exp_attr.autoAttr.dgainRange.min;
    ae->dgain_max    = exp_attr.autoAttr.dgainRange.max;
    ae->ae_target    = exp_attr.autoAttr.aeTarget;
    ae->damp_over    = exp_attr.autoAttr.dampOver;
    ae->damp_under   = exp_attr.autoAttr.dampUnder;
    ae->tolerance    = exp_attr.autoAttr.tolerance;
    ae->run_interval = exp_attr.autoAttr.aeRunInterval;
    ae->antiflicker_enable = exp_attr.autoAttr.antiflicker.enable;
    ae->antiflicker_freq   = exp_attr.autoAttr.antiflicker.flickerFreq;
    ae->gain_threshold = exp_attr.autoAttr.gainThreshold;
    ae->ae_mode        = (enum isp_ae_mode)exp_attr.autoAttr.aeMode;
    ae->ae_delay_attr.black_delay_frame = exp_attr.autoAttr.aeDelayAttr.blackDelayFrame;
    ae->ae_delay_attr.white_delay_frame = exp_attr.autoAttr.aeDelayAttr.whiteDelayFrame;
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
            ae->weight[r][c] = exp_attr.autoAttr.weight[r][c];
        }
    }
    if (exp_attr.autoAttr.aeRoute.totalNum > ISP_AE_ROUTE_MAX_NODES) {
        ae->ae_route_total_nodes = ISP_AE_ROUTE_MAX_NODES;
    } else {
        ae->ae_route_total_nodes = exp_attr.autoAttr.aeRoute.totalNum;
    }
    for (int i = 0; i < (int)ae->ae_route_total_nodes; i++) {
        ae->ae_route[i].int_time = exp_attr.autoAttr.aeRoute.routeNode[i].intTime;
        ae->ae_route[i].again    = exp_attr.autoAttr.aeRoute.routeNode[i].again;
        ae->ae_route[i].dgain    = exp_attr.autoAttr.aeRoute.routeNode[i].dgain;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_WB_MODULE)
static int32_t ISP_get_param_wb(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_wb_param *wb = &params->wb;
    ISP_WB_ATTR_S wb_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_WB)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetWbAttr(isp->isp_port_id, &wb_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    wb->enable  = wb_attr.enable;
    wb->op_mode = (wb_attr.opType == OP_TYPE_AUTO) ? ISP_OP_AUTO : ISP_OP_MANUAL;
    wb->r_gain  = wb_attr.manualAttr.wbGain.rGain;
    wb->gr_gain = wb_attr.manualAttr.wbGain.grGain;
    wb->gb_gain = wb_attr.manualAttr.wbGain.gbGain;
    wb->b_gain  = wb_attr.manualAttr.wbGain.bGain;
    wb->run_interval    = wb_attr.autoAttr.runInterval;
    wb->speed           = wb_attr.autoAttr.speed;
    wb->tolerance       = wb_attr.autoAttr.tolerance;
    wb->init_color_temp = wb_attr.autoAttr.initColorTemp;
    wb->calib.center_line.rg_param   = wb_attr.autoAttr.calibParam.centLine.rgParam;
    wb->calib.center_line.bg_param   = wb_attr.autoAttr.calibParam.centLine.bgParam;
    wb->calib.center_line.dist_param = wb_attr.autoAttr.calibParam.centLine.distParam;
    wb->calib.rg_min = wb_attr.autoAttr.calibParam.rgMin;
    wb->calib.rg_max = wb_attr.autoAttr.calibParam.rgMax;
    for (int i = 0; i < 16; i++) {
        wb->calib.wp_range0.left.rg[i]   = wb_attr.autoAttr.calibParam.wpRange0.wpLCurve.rg[i];
        wb->calib.wp_range0.left.dist[i] = wb_attr.autoAttr.calibParam.wpRange0.wpLCurve.dist[i];
        wb->calib.wp_range0.right.rg[i]   = wb_attr.autoAttr.calibParam.wpRange0.wpRCurve.rg[i];
        wb->calib.wp_range0.right.dist[i] = wb_attr.autoAttr.calibParam.wpRange0.wpRCurve.dist[i];
        wb->calib.wp_range1.left.rg[i]   = wb_attr.autoAttr.calibParam.wpRange1.wpLCurve.rg[i];
        wb->calib.wp_range1.left.dist[i] = wb_attr.autoAttr.calibParam.wpRange1.wpLCurve.dist[i];
        wb->calib.wp_range1.right.rg[i]   = wb_attr.autoAttr.calibParam.wpRange1.wpRCurve.rg[i];
        wb->calib.wp_range1.right.dist[i] = wb_attr.autoAttr.calibParam.wpRange1.wpRCurve.dist[i];
    }
    for (int i = 0; i < ILLUMINANT_TYPE_CNT; i++) {
        wb->calib.illuminant[i].illu_type =
            (enum isp_illuminant_type)wb_attr.autoAttr.calibParam.illuminant[i].illuType;
        wb->calib.illuminant[i].color_temp =
            wb_attr.autoAttr.calibParam.illuminant[i].colorTemp;
        wb->calib.illuminant[i].r_gain  =
            wb_attr.autoAttr.calibParam.illuminant[i].wbGain.rGain;
        wb->calib.illuminant[i].gr_gain =
            wb_attr.autoAttr.calibParam.illuminant[i].wbGain.grGain;
        wb->calib.illuminant[i].gb_gain =
            wb_attr.autoAttr.calibParam.illuminant[i].wbGain.gbGain;
        wb->calib.illuminant[i].b_gain  =
            wb_attr.autoAttr.calibParam.illuminant[i].wbGain.bGain;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_BLS_MODULE)
static int32_t ISP_get_param_bls(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_bls_param *bls = &params->bls;
    ISP_BLS_ATTR_S bls_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_BLS)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetBlsAttr(isp->isp_port_id, &bls_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    bls->enable = bls_attr.enable;
    bls->op_mode = (bls_attr.opType == OP_TYPE_AUTO) ? ISP_OP_AUTO : ISP_OP_MANUAL;
    for (int i = 0; i < 4; i++) {
        bls->black_level[i] = bls_attr.manualAttr.blackLevel[i];
    }
    for (int i = 0; i < ISP_AUTO_STRENGTH_NUN; i++) {
        for (int j = 0; j < 4; j++) {
            bls->auto_black_level[i][j] = bls_attr.autoAttr.blackLevel[i][j];
        }
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_DMSC_MODULE)
static int32_t ISP_get_param_dmsc(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_dmsc_param *dmsc = &params->dmsc;
    ISP_DMSC_ATTR_S dmsc_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_DMSC)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetDmscAttr(isp->isp_port_id, &dmsc_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    dmsc->enable          = dmsc_attr.enable;
    dmsc->threshold       = dmsc_attr.threshold;
    dmsc->cac_enable      = dmsc_attr.cacAttr.enable;
    dmsc->cac_h_clip_mode = dmsc_attr.cacAttr.hClipMode;
    dmsc->cac_v_clip_mode = dmsc_attr.cacAttr.vClipMode;
    dmsc->cac_h_start     = dmsc_attr.cacAttr.hStart;
    dmsc->cac_v_start     = dmsc_attr.cacAttr.vStart;
    dmsc->cac_a_blue      = dmsc_attr.cacAttr.aBlue;
    dmsc->cac_a_red       = dmsc_attr.cacAttr.aRed;
    dmsc->cac_b_blue      = dmsc_attr.cacAttr.bBlue;
    dmsc->cac_b_red       = dmsc_attr.cacAttr.bRed;
    dmsc->cac_c_blue      = dmsc_attr.cacAttr.cBlue;
    dmsc->cac_c_red       = dmsc_attr.cacAttr.cRed;
    dmsc->cac_x_norm_shift  = dmsc_attr.cacAttr.xNormShift;
    dmsc->cac_x_norm_factor = dmsc_attr.cacAttr.xNormFactor;
    dmsc->cac_y_norm_shift  = dmsc_attr.cacAttr.yNormShift;
    dmsc->cac_y_norm_factor = dmsc_attr.cacAttr.yNormFactor;
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_FLT_MODULE)
static int32_t ISP_get_param_flt(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_flt_param *flt = &params->flt;
    ISP_FLT_ATTR_S flt_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_FLT)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetFltAttr(isp->isp_port_id, &flt_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    flt->enable  = flt_attr.enable;
    flt->op_mode = (flt_attr.opType == OP_TYPE_AUTO) ? ISP_OP_AUTO : ISP_OP_MANUAL;
    flt->denoise_level = flt_attr.manualAttr.denoiseLevel;
    flt->sharpen_level  = flt_attr.manualAttr.sharpenLevel;
    for (int i = 0; i < ISP_AUTO_STRENGTH_NUN; i++) {
        flt->auto_denoise_level[i] = flt_attr.autoAttr.denoiseLevel[i];
        flt->auto_sharpen_level[i] = flt_attr.autoAttr.sharpenLevel[i];
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_CCM_MODULE)
static int32_t ISP_get_param_ccm(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_ccm_param *ccm = &params->ccm;
    ISP_CCM_ATTR_S ccm_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_CCM)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetCcmAttr(isp->isp_port_id, &ccm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    ccm->op_mode = (ccm_attr.opType == OP_TYPE_AUTO) ? ISP_OP_AUTO : ISP_OP_MANUAL;
    memcpy(ccm->color_matrix, ccm_attr.manualAttr.colorMatrix, sizeof(ccm->color_matrix));
    ccm->r_offset = ccm_attr.manualAttr.rOffset;
    ccm->g_offset = ccm_attr.manualAttr.gOffset;
    ccm->b_offset = ccm_attr.manualAttr.bOffset;
    for (int i = 0; i < ILLUMINANT_TYPE_CNT; i++) {
        memcpy(ccm->auto_ccm[i].color_matrix,
               ccm_attr.autoAttr.illuminantCCM[i].colorMatrix,
               sizeof(ccm->auto_ccm[i].color_matrix));
        ccm->auto_ccm[i].r_offset   = ccm_attr.autoAttr.illuminantCCM[i].rOffset;
        ccm->auto_ccm[i].g_offset   = ccm_attr.autoAttr.illuminantCCM[i].gOffset;
        ccm->auto_ccm[i].b_offset   = ccm_attr.autoAttr.illuminantCCM[i].bOffset;
        ccm->auto_ccm[i].color_temp = ccm_attr.autoAttr.illuminantCCM[i].colorTemp;
    }
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_GAMMAOUT_MODULE)
static int32_t ISP_get_param_gamma(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_gamma_param *gamma = &params->gamma_out;
    ISP_GAMMA_OUT_ATTR_S gamma_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_GAMMA_OUT)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetGammaOutAttr(isp->isp_port_id, &gamma_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    gamma->enable = gamma_attr.enable;
    memcpy(gamma->gamma_y, gamma_attr.gammaY, sizeof(gamma->gamma_y));
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_CSM_MODULE)
static int32_t ISP_get_param_csm(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_csm_param *csm = &params->csm;
    ISP_CSM_ATTR_S csm_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_CSM)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetCsmAttr(isp->isp_port_id, &csm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    csm->type  = (enum isp_csm_type)csm_attr.type;
    csm->range = (enum isp_csm_range)csm_attr.quantization;
    memcpy(csm->coef, csm_attr.coef, sizeof(csm->coef));
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_EXPM_MODULE)
static int32_t ISP_get_param_expm(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_aem_param *aem = &params->aem;
    ISP_EXPM_ATTR_S expm_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_AEM)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetExpmAttr(isp->isp_port_id, &expm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    aem->enable   = expm_attr.enable;
    aem->alt_mode = expm_attr.expAltMode;
    aem->h_offs   = expm_attr.blockWin.hOffs;
    aem->v_offs   = expm_attr.blockWin.vOffs;
    aem->h_size   = expm_attr.blockWin.hSize;
    aem->v_size   = expm_attr.blockWin.vSize;
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_WBM_MODULE)
static int32_t ISP_get_param_wbm(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_wbm_param *wbm = &params->wbm;
    ISP_WBM_ATTR_S wbm_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_WBM)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetWbmAttr(isp->isp_port_id, &wbm_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    wbm->enable     = wbm_attr.enable;
    wbm->meas_mode  = (enum isp_wbm_mode)wbm_attr.measMode;
    wbm->h_offs     = wbm_attr.measRect.hOffs;
    wbm->v_offs     = wbm_attr.measRect.vOffs;
    wbm->h_size     = wbm_attr.measRect.hSize;
    wbm->v_size     = wbm_attr.measRect.vSize;
    wbm->max_y      = wbm_attr.wpRange.maxY;
    wbm->ref_cr_max_r = wbm_attr.wpRange.refCr_MaxR;
    wbm->min_y_max_g  = wbm_attr.wpRange.minY_MaxG;
    wbm->ref_cb_max_b = wbm_attr.wpRange.refCb_MaxB;
    wbm->max_c_sum    = wbm_attr.wpRange.maxCSum;
    wbm->min_c        = wbm_attr.wpRange.minC;
    return ARM_DRIVER_OK;
}
#endif

#if (RTE_ISP_BINNING_MODULE)
static int32_t ISP_get_param_binning(ISP_RESOURCES *isp, struct isp_params *params)
{
    int ret;
    struct isp_binning_param *binning = &params->binning;
    ISP_BINNING_ATTR_S binning_attr = {0};

    if (!(params->valid_mask & ISP_PARAM_MASK_BINNING)) {
        return ARM_DRIVER_OK;
    }

    ret = VSI_MPI_ISP_GetBinningAttr(isp->isp_port_id, &binning_attr);
    if (ret) {
        return ARM_DRIVER_ERROR;
    }
    binning->enable = binning_attr.enable;
    binning->h_step = binning_attr.binHStep;
    binning->v_step = binning_attr.binVStep;
    return ARM_DRIVER_OK;
}
#endif

static int32_t ISP_get_param(ISP_RESOURCES *isp, struct isp_params *params)
{
    int32_t ret = ARM_DRIVER_OK;

    if (!params) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

#if (RTE_ISP_AE_MODULE)
    ret = ISP_get_param_ae(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_WB_MODULE)
    ret = ISP_get_param_wb(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_BLS_MODULE)
    ret = ISP_get_param_bls(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_DMSC_MODULE)
    ret = ISP_get_param_dmsc(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_FLT_MODULE)
    ret = ISP_get_param_flt(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_CCM_MODULE)
    ret = ISP_get_param_ccm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_GAMMAOUT_MODULE)
    ret = ISP_get_param_gamma(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_CSM_MODULE)
    ret = ISP_get_param_csm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_EXPM_MODULE)
    ret = ISP_get_param_expm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_WBM_MODULE)
    ret = ISP_get_param_wbm(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
#if (RTE_ISP_BINNING_MODULE)
    ret = ISP_get_param_binning(isp, params);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif
    if (params->valid_mask & ISP_PARAM_MASK_AUTO_ROUTE) {
        ISP_AUTO_ROUTE_S route = {0};

        ret = VSI_MPI_ISP_GetAutoRoute(isp->isp_port_id, &route);
        if (ret) {
            return ARM_DRIVER_ERROR;
        }
        for (int i = 0; i < ISP_AUTO_STRENGTH_NUN; i++) {
            params->auto_route.auto_route[i] = route.autoRoute[i];
        }
    }
    return ARM_DRIVER_OK;
}

/*
 * fn        int32_t ISP_control (uint32_t control,
 *                                 uint32_t arg,
 *                                 ISP_RESOURCES *isp)
 * brief     Control the ISP.
 * param[in] control ISP control code operation
 * param[in] arg Argument of operation.
 * param[in] isp Pointer to ISP resources.
 * return    \ref execution_status.
 */
static int32_t ISP_control(uint32_t control, uint32_t arg, ISP_RESOURCES *isp)
{
    int32_t ret = 0;

    if (isp->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    switch (control) {
    case ISP_CONTROL_QBUF:
        ret = (VSI_MPI_ISP_QBUF(isp->isp_chn_id, (VIDEO_BUF_S *) arg) == VSI_SUCCESS) ?
              ARM_DRIVER_OK : ARM_DRIVER_ERROR;
        break;
    case ISP_CONTROL_DQBUF:
        ret = (VSI_MPI_ISP_DQBUF(isp->isp_chn_id, (VIDEO_BUF_S *) arg, 0) == VSI_SUCCESS) ?
              ARM_DRIVER_OK : ARM_DRIVER_ERROR;
        break;
    case ISP_PROCESS_FRAME_END:
        VSI_ISP_IrqProcessFrameEnd(isp->isp_port_id);
        ret = ARM_DRIVER_OK;
        break;
    case ISP_CONTROL_SET_PARAM:
        ret = ISP_set_param(isp, (const struct isp_params *)arg);
        break;
    case ISP_CONTROL_GET_PARAM:
        ret = ISP_get_param(isp, (struct isp_params *)arg);
        break;
#if (RTE_ISP_AE_MODULE)
    case ISP_CONTROL_AE_GET_CACHED:
        ret = ISP_Sensor_AEGetCachedValues(
            &((struct isp_ae_cached_values *)arg)->int_line,
            &((struct isp_ae_cached_values *)arg)->again,
            &((struct isp_ae_cached_values *)arg)->dgain);
        if (ret) {
            ret = ARM_DRIVER_ERROR;
        }
        break;
    case ISP_CONTROL_AE_IS_STABLE:
        ret = ISP_Sensor_AEIsStable(isp);
        break;
#endif
    case ISP_CONTROL_SET_CROP: {
        const struct isp_crop_info *crop = (const struct isp_crop_info *)arg;

        if (!crop) {
            ret = ARM_DRIVER_ERROR_PARAMETER;
            break;
        }

        /* Do not allow changing crop while actively streaming */
        if (isp->state.streaming) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        isp_param_set_crop(crop->top, crop->left, crop->width, crop->height);

        /* Read current HW port config, update only outFormRect, then write back */
        ISP_PORT_ATTR_S isp_port_config;

        ret = VSI_MPI_ISP_GetPortAttr(isp->isp_port_id, &isp_port_config);
        if (ret) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        isp_port_config.outFormRect = isp->isp_port_attr->outFormRect;

        ret = VSI_MPI_ISP_SetPortAttr(isp->isp_port_id, &isp_port_config);
        if (ret) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        /* outFormRect change affects AE/AWB stats windows and the binning/resize pipeline.
         * Re-applying SetChnAttr() recomputes the resize module to satisfy the resolution
         * constraints between binning, scaler, and output.
         */
        ret = VSI_MPI_ISP_SetChnAttr(isp->isp_chn_id, isp->isp_chan_attr);
        if (ret) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        /* Update AE measurement window to match the new crop dimensions. */
        isp->isp_calib_info->modules.aem.blockWin.hOffs = isp->isp_port_attr->outFormRect.top;
        isp->isp_calib_info->modules.aem.blockWin.vOffs = isp->isp_port_attr->outFormRect.left;
        isp->isp_calib_info->modules.aem.blockWin.hSize = isp->isp_port_attr->outFormRect.width;
        isp->isp_calib_info->modules.aem.blockWin.vSize = isp->isp_port_attr->outFormRect.height;
        ret = VSI_MPI_ISP_SetExpmAttr(isp->isp_port_id, &isp->isp_calib_info->modules.aem);
        if (ret) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        /* Update WB measurement window to match the new crop dimensions. */
        isp->isp_calib_info->modules.wbm.measRect.hOffs = isp->isp_port_attr->outFormRect.top;
        isp->isp_calib_info->modules.wbm.measRect.vOffs = isp->isp_port_attr->outFormRect.left;
        isp->isp_calib_info->modules.wbm.measRect.hSize = isp->isp_port_attr->outFormRect.width;
        isp->isp_calib_info->modules.wbm.measRect.vSize = isp->isp_port_attr->outFormRect.height;
        ret = VSI_MPI_ISP_SetWbmAttr(isp->isp_port_id, &isp->isp_calib_info->modules.wbm);
        if (ret) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        break;
    }
    case ISP_CONTROL_SET_OUTPUT: {

        const struct isp_output_info *out = (const struct isp_output_info *)arg;

        if (!out) {
            ret = ARM_DRIVER_ERROR_PARAMETER;
            break;
        }

        /* Do not allow changing output dimensions while actively streaming */
        if (isp->state.streaming) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        isp_param_set_output_dimensions(out->width, out->height);

        /* Re-apply channel attributes to write output dimensions to HW registers */
        ret = VSI_MPI_ISP_SetChnAttr(isp->isp_chn_id, isp->isp_chan_attr);
        if (ret) {
            ret = ARM_DRIVER_ERROR;
            break;
        }

        ret = ARM_DRIVER_OK;

        break;
    }
    default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}
#if (RTE_ISP)

/* ISP sensor access structure */
static CAMERA_SENSOR_DEVICE *sensor;

#if (RTE_ISP_BINNING_MODULE)
static ISP_BINNING_ATTR_S binning_attr = {
    .enable   = RTE_ISP_BINNING_ENABLE,
    .binHStep = RTE_ISP_BINNING_HSTEP,
    .binVStep = RTE_ISP_BINNING_VSTEP,
};
#endif /* RTE_ISP_BINNING_MODULE */

ISP_RESOURCES ISP_RES = {
    .isp_calib_info = &calibration_data,
    .isp_port_attr  = &port_attr,
    .isp_chan_attr  = &chan_attr,
    .cb_event       = NULL,
    .irq_priority   = RTE_ISP_IRQ_PRIORITY,
    .isp_dev_id     = 0,
    .isp_port_id    = {
        0, /* Device ID */
        0, /* Port ID */
    },
    .isp_chn_id     = {
        0, /* Device ID */
        0, /* Port ID */
        0, /* Channel ID */
    },
#if (RTE_ISP_BINNING_MODULE)
    .isp_binning_attr = &binning_attr,
#endif /* RTE_ISP_BINNING_MODULE */
    .state = {0},
};

/*
 * fn        int32_t ISP_Initialize (ARM_ISP_SignalEvent_t cb_event)
 * brief     Initialize ISP Interface.
 * param[in] cb_event Pointer to ARM_ISP_SignalEvent_t.
 * return    @ref execution_status.
 */
static int32_t ISP_Initialize(ARM_ISP_SignalEvent_t cb_event)
{
    sensor = Get_Camera_Sensor();
    return ISP_Init(cb_event, sensor, &ISP_RES);
}

/*
 * fn        int32_t ISP_Uninitialize (void)
 * brief     Uninitialize ISP Interface.
 * return    @ref execution_status.
 */
static int32_t ISP_Uninitialize(void)
{
    return ISP_Uninit(&ISP_RES);
}

/*
 * fn        int32_t ISP_PowerControl (ARM_POWER_STATE state)
 * brief     Control ISP Interface Power.
 * param[in] state Power state.
 * return    @ref execution_status.
 */
static int32_t ISP_PowerControl(ARM_POWER_STATE state)
{
    return ISP_PowerCtrl(state, &ISP_RES);
}

/*
 * fn        int32_t ISP_Start()
 * brief     ISP Start Image Capture.
 * return    @ref execution_status.
 */
static int32_t ISP_Start(void)
{
    return ISP_start(&ISP_RES);
}

/*
 * fn        int32_t ISP_StopCapture()
 * brief     ISP Stop Image Capture.
 * return    @ref execution_status.
 */
static int32_t ISP_Stop(void)
{
    return ISP_stop(&ISP_RES);
}

/*
 * fn        int32_t ISP_Control(uint32_t control, uint32_t arg)
 * brief     Control ISP.
 * param[in] control ISP configuration.
 * param[in] arg Argument of operation (optional).
 * return    @ref execution_status.
 */
static int32_t ISP_Control(uint32_t control, uint32_t arg)
{
    return ISP_control(control, arg, &ISP_RES);
}

void ISP_ISRHandler(ISP_RESOURCES *isp)
{
    uint32_t reg;
    uint32_t event = 0;

    reg            = isp_get_interrupt_status(isp->isp_port_id);

    if (reg) {
        isp_irq_handler_clear_intr_status(isp->isp_port_id, reg);
    }

    if (reg & ISP_INTR_DATALOSS) {
        event |= ARM_ISP_EVENT_DATALOSS_DETECTED;
    }

    if (reg & ISP_INTR_SIZE_ERR) {
        event |= ARM_ISP_EVENT_SIZE_ERR_DETECTED;
    }

    if (reg & ISP_INTR_AWB_DONE) {
        event |= ARM_ISP_EVENT_AWB_DONE;
    }

    if (reg & ISP_INTR_FRAME_IN) {
        event |= ARM_ISP_EVENT_FRAME_IN_DETECTED;
    }

    if (reg & ISP_INTR_VSYNC) {
        event |= ARM_ISP_EVENT_FRAME_VSYNC_DETECTED;
    }

    if (reg & ISP_INTR_HSYNC) {
        event |= ARM_ISP_EVENT_FRAME_HSYNC_DETECTED;
    }

    if (reg & ISP_INTR_EXPM_COMPLETE) {
        event |= ARM_ISP_EVENT_FRAME_IN_DETECTED;
    }

    if (event && isp->cb_event) {
        isp->cb_event(event);
    }
}

void ISP_MI_ISRHandler(ISP_RESOURCES *isp)
{
    uint32_t event = 0;
    uint32_t reg;

    reg = isp_mi_get_interrupt_status(isp->isp_port_id);

    if (reg) {
        isp_mi_irq_handler_clear_intr_status(isp->isp_port_id, reg);
    }

    if (reg & ISP_MI_INTR_MP_FRAME_END) {
        event |= ARM_ISP_MI_EVENT_MP_FRAME_END_DETECTED;
    }

    if (reg & ISP_MI_INTR_MBLK_LINE) {
        event |= ARM_ISP_MI_EVENT_MBLK_LINE_DETECTED;
    }

    if (reg & ISP_MI_INTR_FILL_MP_Y) {
        event |= ARM_ISP_MI_EVENT_FILL_MP_Y_DETECTED;
    }

    if (reg & ISP_MI_INTR_WRAP_MP_Y) {
        event |= ARM_ISP_MI_EVENT_MP_Y_WRAP_DETECTED;
    }

    if (reg & ISP_MI_INTR_WRAP_MP_CB) {
        event |= ARM_ISP_MI_EVENT_MP_CB_WRAP_DETECTED;
    }

    if (reg & ISP_MI_INTR_WRAP_MP_CR) {
        event |= ARM_ISP_MI_EVENT_MP_CR_WRAP_DETECTED;
    }

    if (event && isp->cb_event) {
        isp->cb_event(event);
    }
}

void ISP_IRQHandler(void)
{
    ISP_ISRHandler(&ISP_RES);
}

void ISP_MI_IRQHandler(void)
{
    ISP_MI_ISRHandler(&ISP_RES);
}

extern ARM_DRIVER_ISP Driver_ISP;
ARM_DRIVER_ISP        Driver_ISP = {
    ISP_GetVersion,
    ISP_GetCapabilities,
    ISP_Initialize,
    ISP_Uninitialize,
    ISP_PowerControl,
    ISP_Start,
    ISP_Stop,
    ISP_Control,
};
#endif
/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
#endif /* RTE_Drivers_ISP */
