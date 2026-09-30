/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     Driver_CPI_Private.h
 * @author   Chandra Bhushan Singh
 * @email    chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     27-March-2023
 * @brief    CMSIS Driver Private Header file.
 ******************************************************************************/

#ifndef DRIVER_CPI_PRIVATE_H_

#define DRIVER_CPI_PRIVATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* Project Includes */
#include "Driver_CPI.h"
#include "Camera_Sensor.h"

#include "cpi.h"

#include "sys_ctrl_cpi.h"

/**
 * enum CPI_INSTANCE.
 * CPI instances.
 */
typedef enum _CPI_INSTANCE {
    CPI_INSTANCE_CPI0, /**< CPI instance as CPI                                */
    CPI_INSTANCE_LPCPI /**< CPI instance as LPCPI                              */
} CPI_INSTANCE;

#if SOC_FEAT_CPI_HAS_CROPPING
/** \brief CPI horizontal Configuration */
typedef struct _CPI_HORIZONTAL_CONFIG {
    uint16_t           hbp;    /**< Horizontal Back Porch                      */
    uint16_t           hfp;    /**< Horizontal front Porch                     */
    CPI_HORZ_CROP_MODE hfp_en; /**< Enable horizontal cropping                 */
} CPI_HORIZONTAL_CONFIG;

/** \brief CPI Vertical Configuration */
typedef struct _CPI_VERTICAL_CONFIG {
    uint16_t           vbp;    /**< Vertical Back Porch                        */
    uint16_t           vfp;    /**< Vertical front Porch                       */
    CPI_VERT_CROP_MODE vfp_en; /**< Enable vertical cropping                   */
} CPI_VERTICAL_CONFIG;
#endif

/** \brief CPI FIFO Configuration */
typedef struct _CPI_FIFO_CONFIG {
    uint8_t read_watermark;  /**< FIFO Read  Water mark 0,1: illegal           */
    uint8_t write_watermark; /**< FIFO Write Water mark 0,1: illegal           */
} CPI_FIFO_CONFIG;

/** \brief CPI Configurations */
typedef struct _CPI_CONFIG {
#if SOC_FEAT_HAS_ISP
    bool                   axi_port_en;     /**< CPI AXI port                                   */
    bool                   isp_port_en;     /**< CPI ISP port                                   */
#endif
#if SOC_FEAT_CPI_HAS_CROPPING
    CPI_HORIZONTAL_CONFIG *horizontal_cfg;  /**< Horizontal Configuration                       */
    CPI_VERTICAL_CONFIG   *vertical_cfg;    /**< Vertical Configuration                         */
#endif
    uint32_t              framebuff_saddr;  /**< Frame Buffer Start Address Configuration       */
#if SOC_FEAT_CPI_HAS_STREAM_ENABLE
    uint32_t              framebuff_saddrB; /**< Sequential FrameBuffer StartAddr Configuration */
    uint32_t              framebuff_saddrC; /**< Sequential FrameBuffer StartAddr Configuration */
    uint32_t              framebuff_saddrD; /**< Sequential FrameBuffer StartAddr Configuration */
#endif
    CPI_FIFO_CONFIG       *fifo;            /**< FIFO Configuration                             */
} CPI_CONFIG;

/** \brief CPI Status */
typedef struct CPI_DRIVER_STATE {
    uint32_t initialized      : 1;  /**< Driver Initialized                                     */
    uint32_t powered          : 1;  /**< Driver powered                                         */
    uint32_t sensor_configured: 1;  /**< Camera sensor configured                               */
    uint32_t reserved         : 29; /**< Reserved                                               */
} CPI_DRIVER_STATE;

/** \brief CPI Device Resource Structure */
typedef struct _CPI_RESOURCES {
    ARM_CPI_SignalEvent_t cb_event;           /**< CPI Application Event Callback                 */
    CPI_Type              *regs;              /**< CPI Register Base Address                      */
    CPI_INSTANCE          drv_instance;       /**< CPI driver instances                           */
    CPI_DRIVER_STATE      status;             /**< CPI Status                                     */
    uint8_t               irq_priority;       /**< CPI Interrupt Priority                         */
    IRQn_Type             irq_num;            /**< CPI Interrupt Vector Number                    */
    CPI_ROW_ROUNDUP       row_roundup;        /**< CPI row roundup                                */
    CPI_MODE_SELECT       capture_mode;       /**< CPI capture mode                               */
    uint32_t              irq_mask;           /**< CPI interrupt mask                             */
    CPI_CONFIG            *cnfg;              /**< CPI Configurations                             */
#if SOC_FEAT_CPI_HAS_STREAM_ENABLE
    uint32_t              num_framebuffers;   /**< CPI number of active frame buffers             */
    bool                  stream_mode_enable; /**< Streaming mode configuration                   */
    bool                  stream_mode_active; /**< Streaming mode currently active                */
#endif
    CAMERA_SENSOR_DEVICE  *cam[2];            /**< Registered sensors: instance 0 and 1           */
    uint8_t               num_sensors;        /**< Count of sensors present (1 or 2)              */
    uint8_t               active_sensor;      /**< Currently selected sensor instance index       */
    uint8_t               sensor_inited[2];   /**< ops->Init() done for the instance              */
    CAMERA_SENSOR_DEVICE  *cam_sensor;        /**< Alias of cam[active_sensor]                    */
} CPI_RESOURCES;

#define DEFAULT_WRITE_WMARK 0x18

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_CPI_PRIVATE_H_ */
