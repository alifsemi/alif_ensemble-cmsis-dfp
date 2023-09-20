/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     Driver_DSI_Private.h
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     17-April-2023
 * @brief    DSI driver Specific Header file.
 ******************************************************************************/

#ifndef DRIVER_DSI_PRIVATE_H_
#define DRIVER_DSI_PRIVATE_H_

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_MIPI_DSI.h"

#include "dsi.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/** \brief DSI Driver states. */
typedef volatile struct _DSI_DRIVER_STATE {
    uint32_t initialized       : 1;                    /**< Driver Initialized        */
    uint32_t powered           : 1;                    /**< Driver powered            */
    uint32_t host_configured   : 1;                    /**< Driver host configured    */
    uint32_t dpi_configured    : 1;                    /**< Driver DPI configured     */
    uint32_t panel_initialized : 1;                    /**< Driver panel initialized  */
    uint32_t reserved          : 27;                   /**< Reserved                  */
} DSI_DRIVER_STATE;

/** \brief DSI DPI frame info. */
typedef struct _DSI_FRAME_INFO{
    uint32_t hsa_time;                                 /**<  hsa timing     */
    uint32_t hbp_time;                                 /**<  hpb timing     */
    uint32_t hfp_time;                                 /**<  hfp timing     */
    uint32_t hactive_time;                             /**<  hactive timing */
    uint32_t vsa_line;                                 /**<  vsa lines      */
    uint32_t vbp_line;                                 /**<  vbp lines      */
    uint32_t vfp_line;                                 /**<  vfp lines      */
    uint32_t vactive_line;                             /**<  vactive lines  */
}DSI_FRAME_INFO;

/** \brief MIPI DSI DPI signal polarity. */
typedef struct _DSI_DPI_POLARITY {
    uint8_t dataen_active_low;                         /**< dataen polarity  */
    uint8_t vsync_active_low;                          /**< vsync polarity   */
    uint8_t hsync_active_low;                          /**< hsync polarity   */
    uint8_t shutd_active_low;                          /**< shutd polarity   */
    uint8_t colorm_active_low;                         /**< colorm polarity  */
}DSI_DPI_POLARITY;

/** \brief DSI DPI Info */
typedef struct _DSI_DPI_INFO{
    uint8_t             color_code;                    /**< color coding           */
    DSI_VIDEO_MODE      vid_mode;                      /**< video mode             */
    uint32_t            vid_pkt_size;                  /**< video packet size      */
    uint32_t            vid_num_chunks;                /**< video number of chunks */
    uint32_t            vid_null_size;                 /**< video null packet size */
    DSI_DPI_POLARITY    *dpi_pol;                      /**< DPI interface polarity */
    DSI_FRAME_INFO      *frame_info;                   /**< frame info             */
}DSI_DPI_INFO;

/** \brief MIPI DSI DPHY TIME CFG */
typedef struct _DSI_DPHY_TMR_CFG {
    uint32_t clklp2hs_time;                            /**< clock lane low power to high speed transition timing */
    uint32_t clkhs2lp_time;                            /**< clock lane high speed to low power transition timing */
    uint32_t lp2hs_time;                               /**< data lane low power to high speed transition timing  */
    uint32_t hs2lp_time;                               /**< data lane high speed to low power transition timing  */
}DSI_DPHY_TMR_CFG;

/**
  \brief MIPI DSI driver info
  */
typedef struct _DSI_RESOURCES{
    DSI_Type                        *reg_base;         /**< Pointer to regs                  */
    ARM_MIPI_DSI_SignalEvent_t      cb_event;          /**< Pointer to call back function    */
    DSI_N_LANES                     n_lanes;           /**< Number of lanes                  */
    DSI_VC_ID                       vc_id;             /**< Virtual channel ID               */
    uint32_t                        tx_ecs_clk_div;    /**< Tx escape clock divider value    */
    DSI_DPHY_TMR_CFG                *dphy_tmr;         /**< Pointer to dphy timing           */
    DSI_DPI_INFO                    *dpi_info;         /**< Pointer to DPI info              */
    IRQn_Type                       irq;               /**< Interrupt number                 */
    uint32_t                        irq_priority;      /**< Interrupt priority               */
    DSI_DRIVER_STATE                state;             /**< DSI driver status                */
}DSI_RESOURCES;

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_DSI_PRIVATE_H_ */
