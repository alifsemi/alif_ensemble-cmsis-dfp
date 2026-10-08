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
 * @file     Camera_Sensor.h
 * @author   Tanay Rami and Chandra Bhushan Singh
 * @version  V1.1.0
 *             -Removed enums for clock source, interface, polarity, hsync mode,
 *             data mode, and data mask.
 *             -Included low level header file 'cpi.h'
 *             -Replaced data types in CAMERA_SENSOR_INFO structure with CPI low
 *             level file data types.
 * @date     19-April-2023
 * @brief    Camera Sensor Device definitions.
 ******************************************************************************/

#ifndef CAMERA_SENSOR_H_
#define CAMERA_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "RTE_Device.h"
#include "cpi.h"
#if defined(CSI)
#include "csi.h"
#endif

/* Dual camera: board RTE enable plus SoC second-camera feature. */
#if defined(RTE_SECOND_CAMERA_ENABLE) && (RTE_SECOND_CAMERA_ENABLE) && \
    defined(SOC_FEAT_HAS_CAM2) && (SOC_FEAT_HAS_CAM2)
#define CAMERA_DUAL_SENSOR_SUPPORT 1
#else
#define CAMERA_DUAL_SENSOR_SUPPORT 0
#endif

#define CAMERA_SENSOR_INSTANCE_0 0u
#define CAMERA_SENSOR_INSTANCE_1 1u
#define CAMERA_SENSOR_MAX_COUNT  (CAMERA_DUAL_SENSOR_SUPPORT ? 2u : 1u)

/****** CAMERA_SENSOR used for registering camera sensor *****/
#define CAMERA_SENSOR_(inst, sensor)                                                               \
    CAMERA_SENSOR_DEVICE *Get_Camera_Sensor_##inst(void)                                           \
    {                                                                                              \
        return &sensor;                                                                            \
    }

#define CAMERA_SENSOR(inst, sensor) CAMERA_SENSOR_(inst, sensor)

/****** LPCAMERA_SENSOR used for registering low power camera sensor *****/
#define LPCAMERA_SENSOR(sensor)                                                                    \
    CAMERA_SENSOR_DEVICE *Get_LPCamera_Sensor(void)                                                \
    {                                                                                              \
        return &sensor;                                                                            \
    }

/**
\brief MIPI DPHY used by CSI2 for the sensor
*/
typedef enum _DPHY_PORT {
    DPHY_PORT_CSI2_NATIVE = 0, /* CSI2 RX DPHY */
    DPHY_PORT_DSI_AS_RX   = 1, /* DSI DPHY in RX mode */
} DPHY_PORT;

/**
\brief Camera Sensor interface
*/
typedef enum _CAMERA_SENSOR_INTERFACE {
    CAMERA_SENSOR_INTERFACE_PARALLEL, /* Camera sensor parallel interface */
    CAMERA_SENSOR_INTERFACE_MIPI      /* Camera sensor serial interface */
} CAMERA_SENSOR_INTERFACE;

/**
\brief CPI information structure
*/
typedef struct _CPI_INFO {
    CPI_WAIT_VSYNC          vsync_wait;      /* CPI VSYNC Wait */
    CPI_CAPTURE_DATA_ENABLE vsync_mode;      /* CPI VSYNC Mode */
    CPI_SIG_POLARITY        pixelclk_pol;    /* CPI Pixel Clock Polarity */
    CPI_SIG_POLARITY        hsync_pol;       /* CPI HSYNC Polarity */
    CPI_SIG_POLARITY        vsync_pol;       /* CPI VSYNC Polarity */
    CPI_DATA_MODE           data_mode;       /* CPI Data Mode */
    CPI_DATA_ENDIANNESS     data_endianness; /* CPI MSB/LSB */
    CPI_CODE10ON8_CODING    code10on8;       /* CPI code10on8 enable/disable */
    CPI_DATA_MASK           data_mask;       /* CPI Data Mask */
    CPI_COLOR_MODE_CONFIG   csi_mode;        /* CPI CSI Color mode */
} CPI_INFO;

#if defined(CSI)
/**
\brief CSI override CPI color mode structure
*/
typedef struct _CSI_OVERRIDE_CPI_COLOR {
    bool                  override;       /* CPI color mode override by CPI */
    CPI_COLOR_MODE_CONFIG cpi_color_mode; /* CPI color mode to be override */
} CSI_OVERRIDE_CPI_COLOR;

/**
\brief CSI Pkt2PktTime, Time between Packets (includes the duration of the LS
       Packet + PHY LowPower to High-Speed time + any eventual camera added delay
       + PHY HighSpeed to Low-Power time).
*/
typedef struct _CSI_PKT2PKT_TIME {
    bool  line_sync_pkt_enable; /* LS/LE Packets are enabled */
    float time_ns;              /* Time between Packets in ns */
} CSI_PKT2PKT_TIME;

/**
\brief CSI information structure
*/
typedef struct _CSI_INFO {
    uint32_t               frequency;    /* CSI clock frequency */
    CSI_DATA_TYPE          dt;           /* CSI data type */
    uint8_t                n_lanes;      /* CSI number of data lanes */
    CSI_VC_ID              vc_id;        /* CSI virtual channel ID */
    CSI_OVERRIDE_CPI_COLOR cpi_cfg;      /* CSI override CPI color mode */
    CSI_PKT2PKT_TIME       pkt2pkt_time; /* CSI Time between Packets */
} CSI_INFO;
#endif

/**
\brief CAMERA Sensor Device Operations
*/
typedef struct _CAMERA_SENSOR_OPERATIONS {
    int32_t (*Init)(void);        /* Initialize Camera Sensor device */
    int32_t (*Uninit)(void);      /* De-initialize Camera Sensor device */
    int32_t (*Start)(void);       /* Start Camera Sensor device */
    int32_t (*Snapshot)(uint8_t); /* Start Camera Sensor device in snapshot mode*/
    int32_t (*Stop)(void);        /* Stop Camera Sensor device */
    int32_t (*Control)(uint32_t control, uint32_t arg); /* Control Camera Sensor device */
    int32_t (*Suspend)(void);     /* Suspend the camera sensor to sleep state */
    int32_t (*Resume)(void);      /* Resume the camera sensor from sleep state */
} CAMERA_SENSOR_OPERATIONS;

/**
\brief CAMERA Sensor Device
*/
typedef struct _CAMERA_SENSOR_DEVICE {
    CAMERA_SENSOR_INTERFACE   interface; /* Camera Sensor interface */
    int                       width;     /* Frame Width */
    int                       height;    /* Frame Height */
    CPI_INFO                 *cpi_info;  /* CPI Camera Sensor device Information */
#if defined(CSI)
    CSI_INFO                 *csi_info;  /* CSI Camera Sensor device Information */
    DPHY_PORT                 dphy_port; /* MIPI DPHY port: CSI2 native / DSI-as-RX */
#endif
    CAMERA_SENSOR_OPERATIONS *ops;       /* Camera Sensor device Operations */
} CAMERA_SENSOR_DEVICE;

CAMERA_SENSOR_DEVICE *Camera_Sensor_Get(uint8_t instance);
uint8_t               Camera_Sensor_GetCount(void);

/* I2C C1/C2 analog-switch GPIO. Called by Driver_CPI; not an application API. */
void    Camera_Sensor_I2C_Mux_Switch(uint8_t instance);
int32_t Camera_Sensor_I2C_Mux_Initialize(void);
void    Camera_Sensor_I2C_Mux_Uninitialize(void);

CAMERA_SENSOR_DEVICE *Get_LPCamera_Sensor(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_SENSOR_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
