/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     Driver_MIPI_DSI.c
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     18-April-2022
 * @brief    CMSIS-Driver for MIPI DSI.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "Driver_MIPI_DSI.h"
#include "dsi.h"
#include "Driver_DSI_Private.h"
#include "RTE_Device.h"
#include "DPHY_init.h"
#include "display.h"

#define ARM_MIPI_DSI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /*driver version*/

#if !(RTE_MIPI_DSI)
#error "MIPI DSI is not enabled in the RTE_Device.h"
#endif

#if (!defined(RTE_Drivers_MIPI_DSI))
#error "MIPI DSI not configured in RTE_Components.h!"
#endif

/*Driver Version*/
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_MIPI_DSI_API_VERSION,
    ARM_MIPI_DSI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MIPI_DSI_CAPABILITIES DriverCapabilities =
{
    0, /* Not supports reentrant_operation */
    1, /* DPI interface supported*/
    0, /* DBI interface not supported*/
    0  /* reserved (must be zero) */
};

/**
  \fn          ARM_DRIVER_VERSION ARM_MIPI_DSI_GetVersion (void)
  \brief       Get MIPI DSI driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ARM_MIPI_DSI_GetVersion (void)
{
    return DriverVersion;
}

/**
  \fn          ARM_MIPI_DSI_CAPABILITIES MIPI_DSI_GetCapabilities (void)
  \brief       Get MIPI DSI driver capabilities
  \return      \ref ARM_MIPI_DPHY_CAPABILITIES
*/
static ARM_MIPI_DSI_CAPABILITIES ARM_MIPI_DSI_GetCapabilities (void)
{
    return DriverCapabilities;
}

/**
  \fn          int32_t DSI_Initialize (ARM_MIPI_DSI_SignalEvent_t cb_event,
                                       DISPLAY_PANEL_DEVICE *display_panel,
                                       DSI_RESOURCES *dsi)
  \brief       Initialize MIPI DSI Interface.
  \param[in]   cb_event Pointer to ARM_MIPI_DSI_SignalEvent_t.
  \param[in]   display_panel Pointer to display panel resources.
  \param[in]   dsi Pointer to DSI resources.
  \return      \ref execution_status.
  */
static int32_t DSI_Initialize (ARM_MIPI_DSI_SignalEvent_t cb_event,
                               DISPLAY_PANEL_DEVICE *display_panel,
                               DSI_RESOURCES *dsi)
{
    DISPLAY_PANEL_FRAME_INFO *frame_info = display_panel->Frame_info;
    uint32_t pixclk, htotal, vtotal;
    uint32_t frequency;
    int32_t ret = ARM_DRIVER_OK;


    if(display_panel == NULL)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if(dsi->state.initialized == 1)
    {
        return ARM_DRIVER_OK;
    }

    if (!cb_event)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    dsi->cb_event = cb_event;


    /* LCD Manufacturer provides the Frame timing values
     *     HTOTAL = WIDTH + HSYNC + HFP + HBP
     *     VTOTAL = HEIGHT + VSYNC + VFP + VBP
     * Calculate the pixel clock for DPI controller
     *     PIXCLK = FPS x HTOTAL x VTOTAL
     * Calculate the pixel clock divider
     *     PIXCLK_DIV = CDC200_PIXCLK_SOURCE / PIXCLK
     */
    htotal = (frame_info->hsync_time
              + frame_info->hbp_time
              + frame_info->hfp_time
              + frame_info->hactive_time);

    vtotal = (frame_info->vsync_line
              + frame_info->vbp_line
              + frame_info->vfp_line
              + frame_info->vactive_line);

    pixclk = (htotal * vtotal * RTE_CDC200_DPI_FPS);

    /* SCALE = LANEBYTECLK / PIXCLK
     * MIPI data rate must be exactly equal, not greater than, for 1.5 scale to work
     * MIPI data rate + 33% allows for scaling times 2
     *    24 x 1.333 / 16 = 2
     * LANEBYTECLK = PIXCLK * SCALE
     * lanebyteclk frequency is 1/4th of the DPHY frequency
     * PLL frequency = LANEBYTECLK * 4
     *               = PIXCLK * SCALE * 4
     */
    frequency = pixclk * 2 * 4;

    /*Checking LCD Panel supports MIPI DSI DPHY data rate*/
    if((frequency * 2) > (display_panel->Info->max_bitrate * 1000000))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /*DPHY initialization*/
    ret  = DSI_DPHY_Initialize(frequency);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    /*LCD Panel Initialization*/
    ret = display_panel->Ops->Init();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    dsi->dpi_info->vid_pkt_size = display_panel->Frame_info->hactive_time;
    dsi->dpi_info->frame_info->hsa_time = display_panel->Frame_info->hsync_time << 1;
    dsi->dpi_info->frame_info->hbp_time = display_panel->Frame_info->hbp_time << 1;
    dsi->dpi_info->frame_info->hfp_time = display_panel->Frame_info->hfp_time << 1;
    dsi->dpi_info->frame_info->hactive_time = display_panel->Frame_info->hactive_time << 1;
    dsi->dpi_info->frame_info->vsa_line = display_panel->Frame_info->vsync_line;
    dsi->dpi_info->frame_info->vbp_line = display_panel->Frame_info->vbp_line;
    dsi->dpi_info->frame_info->vfp_line = display_panel->Frame_info->vfp_line;
    dsi->dpi_info->frame_info->vactive_line = display_panel->Frame_info->vactive_line;

    dsi->state.initialized = 1;

    return ret;
}

/**
  \fn          int32_t DSI_Uninitialize (DISPLAY_PANEL_DEVICE *display_panel, DSI_RESOURCES *dsi)
  \brief       uninitialize MIPI DSI Interface.
  \param[in]   display_panel Pointer to display panel resources.
  \param[in]   dsi Pointer to DSI resources.
  \return      \ref execution_status.
  */
static int32_t DSI_Uninitialize (DISPLAY_PANEL_DEVICE *display_panel, DSI_RESOURCES *dsi)
{
    int32_t ret = ARM_DRIVER_OK;
    dsi->cb_event = NULL;

    if(dsi->state.initialized == 0)
    {
        return ARM_DRIVER_OK;
    }

    if(display_panel == NULL)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (dsi->state.powered == 1)
    {
        return ARM_DRIVER_ERROR;
    }

    /*DPHY Uninitialization*/
    ret  = DSI_DPHY_Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    /*LCD Panel Un-Initialization*/
    ret = display_panel->Ops->Uninit();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    dsi->state.initialized = 0;
    return ret;
}

/**
  \fn          int32_t DSI_PowerControl (ARM_POWER_STATE state,
                                         DSI_RESOURCES *dsi)
  \brief       Control DSI Interface Power.
  \param[in]   state  Power state.
  \param[in]   dsi Pointer to DSI resources.
  \return      \ref execution_status.
  */
static int32_t DSI_PowerControl (ARM_POWER_STATE state,
                                 DSI_RESOURCES *dsi)
{

    if (dsi->state.initialized == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            if (dsi->state.powered == 0)
            {
                return ARM_DRIVER_OK;
            }

            NVIC_DisableIRQ (dsi->irq);
            NVIC_ClearPendingIRQ (dsi->irq);

            dsi->state.powered = 0;
            break;
        }
        case ARM_POWER_FULL:
        {
            if (dsi->state.powered == 1)
            {
                return ARM_DRIVER_OK;
            }

            NVIC_ClearPendingIRQ (dsi->irq);
            NVIC_SetPriority (dsi->irq, dsi->irq_priority);
            NVIC_EnableIRQ (dsi->irq);

            dsi->state.powered = 1;

            break;
        }
        case ARM_POWER_LOW:
        default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t DSI_ConfigureHost (DSI_RESOURCES *dsi)
  \brief       Configure DSI Host Interface.
  \param[in]   dsi  Pointer to DSI resources.
  \return      \ref execution_status.
  */
static int32_t DSI_ConfigureHost (DSI_RESOURCES *dsi)
{

    if (dsi->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    dsi_irq0_enable(dsi->reg_base, DSI_IRQ0_ACK_WITH_ERR_0  | \
                                   DSI_IRQ0_ACK_WITH_ERR_1  | \
                                   DSI_IRQ0_ACK_WITH_ERR_2  | \
                                   DSI_IRQ0_ACK_WITH_ERR_3  | \
                                   DSI_IRQ0_ACK_WITH_ERR_4  | \
                                   DSI_IRQ0_ACK_WITH_ERR_5  | \
                                   DSI_IRQ0_ACK_WITH_ERR_6  | \
                                   DSI_IRQ0_ACK_WITH_ERR_7  | \
                                   DSI_IRQ0_ACK_WITH_ERR_8  | \
                                   DSI_IRQ0_ACK_WITH_ERR_9  | \
                                   DSI_IRQ0_ACK_WITH_ERR_10 | \
                                   DSI_IRQ0_ACK_WITH_ERR_11 | \
                                   DSI_IRQ0_ACK_WITH_ERR_12 | \
                                   DSI_IRQ0_ACK_WITH_ERR_13 | \
                                   DSI_IRQ0_ACK_WITH_ERR_14 | \
                                   DSI_IRQ0_ACK_WITH_ERR_15 | \
                                   DSI_IRQ0_DPHY_ERRORS_0   | \
                                   DSI_IRQ0_DPHY_ERRORS_1   | \
                                   DSI_IRQ0_DPHY_ERRORS_2   | \
                                   DSI_IRQ0_DPHY_ERRORS_3   | \
                                   DSI_IRQ0_DPHY_ERRORS_4);

    dsi_irq1_enable(dsi->reg_base, DSI_IRQ1_TO_HS_TX          | \
                                   DSI_IRQ1_TO_LP_RX          | \
                                   DSI_IRQ1_ECC_SINGLE_ERR    | \
                                   DSI_IRQ1_ECC_MILTI_ERR     | \
                                   DSI_IRQ1_CRC_ERR           | \
                                   DSI_IRQ1_PKT_SIZE_ERR      | \
                                   DSI_IRQ1_EOPT_ERR          | \
                                   DSI_IRQ1_DPI_PLD_WR_ERR    | \
                                   DSI_IRQ1_GEN_CMD_WR_ERR    | \
                                   DSI_IRQ1_GEN_PLD_WR_ERR    | \
                                   DSI_IRQ1_GEN_PLD_SEND_ERR  | \
                                   DSI_IRQ1_GEN_PLD_RD_ERR    | \
                                   DSI_IRQ1_GEN_PLD_RECEV_ERR);

    dsi->state.host_configured = 1;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t  DSI_ConfigureDPI (DSI_RESOURCES *dsi)
  \brief       Configure DSI DPI Interface.
  \param[in]   dsi  Pointer to DSI resources.
  \return      \ref execution_status.
*/
static int32_t DSI_ConfigureDPI (DSI_RESOURCES *dsi)
{
    DSI_DPI_INFO *dpi_info = (DSI_DPI_INFO *)dsi->dpi_info;
    DSI_DPI_POLARITY *dpi_pol = (DSI_DPI_POLARITY *)dsi->dpi_info->dpi_pol;
    DSI_FRAME_INFO *frame_info = (DSI_FRAME_INFO *)dsi->dpi_info->frame_info;

    if(dsi->state.host_configured == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    dsi_dpi_set_color_coding(dsi->reg_base, dpi_info->color_code);

    if(dpi_pol->dataen_active_low == DSI_POLARITY_ACTIVE_LOW)
    {
        dsi_dpi_set_dataen_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_LOW);
    }
    else
    {
        dsi_dpi_set_dataen_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_HIGH);
    }

    if(dpi_pol->vsync_active_low == DSI_POLARITY_ACTIVE_LOW)
    {
        dsi_dpi_set_vsync_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_LOW);
    }
    else
    {
        dsi_dpi_set_vsync_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_HIGH);
    }

    if(dpi_pol->hsync_active_low == DSI_POLARITY_ACTIVE_LOW)
    {
        dsi_dpi_set_hsync_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_LOW);
    }
    else
    {
        dsi_dpi_set_hsync_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_HIGH);
    }

    if(dpi_pol->shutd_active_low == DSI_POLARITY_ACTIVE_LOW)
    {
        dsi_dpi_set_shutd_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_LOW);
    }
    else
    {
        dsi_dpi_set_shutd_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_HIGH);
    }

    if(dpi_pol->colorm_active_low == DSI_POLARITY_ACTIVE_LOW)
    {
        dsi_dpi_set_colorm_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_LOW);
    }
    else
    {
        dsi_dpi_set_colorm_polarity(dsi->reg_base,DSI_POLARITY_ACTIVE_HIGH);
    }

    dsi_set_video_packet_size(dsi->reg_base, dpi_info->vid_pkt_size);

    dsi_set_video_number_chunks(dsi->reg_base, dpi_info->vid_num_chunks);

    dsi_set_video_null_packet_size(dsi->reg_base, dpi_info->vid_null_size);

    dsi_set_video_hsa_time(dsi->reg_base, frame_info->hsa_time);

    dsi_set_video_hbp_time(dsi->reg_base, frame_info->hbp_time);

    dsi_set_video_hline_time(dsi->reg_base, (frame_info->hsa_time   \
                                             + frame_info->hbp_time \
                                             + frame_info->hfp_time \
                                             + frame_info->hactive_time));

    dsi_set_video_vsa_lines(dsi->reg_base, frame_info->vsa_line);

    dsi_set_video_vbp_lines(dsi->reg_base, frame_info->vbp_line);

    dsi_set_video_vfp_lines(dsi->reg_base, frame_info->vfp_line);

    dsi_set_video_vactive_lines(dsi->reg_base, frame_info->vactive_line);

    dsi->state.dpi_configured = 1;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t DSI_Control (ARM_MIPI_DSI_CONTROL control,
                                    uint32_t arg,
                                    DSI_RESOURCES *dsi)
  \brief       Control DSI Interface.
  \param[in]   control DSI host and DPI Configuration.
  \param[in]   arg Argument of operation (optional)
  \param[in]   dsi Pointer to DSI resources.
  \return      \ref execution_status.
  */
static int32_t DSI_Control (ARM_MIPI_DSI_CONTROL control,
                            uint32_t arg,
                            DSI_RESOURCES *dsi)
{
    ARG_UNUSED(arg);
    int32_t ret = ARM_DRIVER_OK;

    switch(control)
    {
        case DSI_CONFIGURE_HOST:
        {
            ret = DSI_ConfigureHost(dsi);
            if(ret != ARM_DRIVER_OK)
            {
                return ret;
            }
            break;
        }
        case DSI_CONFIGURE_DPI:
        {
            ret = DSI_ConfigureDPI(dsi);
            if(ret != ARM_DRIVER_OK)
            {
                return ret;
            }
            break;
        }
        default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    return ret;
}

/**
  \fn          int32_t  DSI_StartCommandMode (DISPLAY_PANEL_DEVICE *display_panel,
                                              DSI_RESOURCES *dsi)
  \brief       Configure DSI to start Command mode.
  \param[in]   display_panel Pointer to display panel resources.
  \param[in]   dsi  Pointer to DSI resources.
  \return      \ref execution_status.
*/
static int32_t DSI_StartCommandMode (DISPLAY_PANEL_DEVICE *display_panel,
                                     DSI_RESOURCES *dsi)
{
    int32_t ret  = ARM_DRIVER_OK;

    DSI_DPI_INFO *dpi_info = (DSI_DPI_INFO *)dsi->dpi_info;
    DSI_DPHY_TMR_CFG *dphy_tmr = (DSI_DPHY_TMR_CFG *)dsi->dphy_tmr;

    if(dsi->state.host_configured == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    dsi_power_up_disable(dsi->reg_base);

    dsi_auto_clklane_enable(dsi->reg_base);

    dsi_set_phy_clklp2hs_time(dsi->reg_base, dphy_tmr->clklp2hs_time);

    dsi_set_phy_clkhs2lp_time(dsi->reg_base, dphy_tmr->clkhs2lp_time);

    dsi_set_phy_lp2hs_time(dsi->reg_base, dphy_tmr->lp2hs_time);

    dsi_set_phy_hs2lp_time(dsi->reg_base, dphy_tmr->hs2lp_time);

    dsi_command_mode_enable(dsi->reg_base);

    dsi_set_video_mode_type(dsi->reg_base, dpi_info->vid_mode);

    dsi_command_transmission_enable(dsi->reg_base);

    dsi_set_tx_escap_clock_divider(dsi->reg_base, dsi->tx_ecs_clk_div);

    dsi_set_command_mode_config(dsi->reg_base, DSI_CMD_MODE_CFG_GEN_SW_0P_TX | \
                                               DSI_CMD_MODE_CFG_GEN_SW_1P_TX | \
                                               DSI_CMD_MODE_CFG_GEN_SW_2P_TX | \
                                               DSI_CMD_MODE_CFG_GEN_SR_0P_TX | \
                                               DSI_CMD_MODE_CFG_GEN_SR_1P_TX | \
                                               DSI_CMD_MODE_CFG_GEN_SR_2P_TX | \
                                               DSI_CMD_MODE_CFG_GEN_LW_TX    | \
                                               DSI_CMD_MODE_CFG_DCS_SW_0P_TX | \
                                               DSI_CMD_MODE_CFG_DCS_SW_1P_TX | \
                                               DSI_CMD_MODE_CFG_DCS_SR_0P_TX | \
                                               DSI_CMD_MODE_CFG_DCS_LW_TX    | \
                                               DSI_CMD_MODE_CFG_MAX_RD_PKT_SIZE);


    dsi_power_up_enable(dsi->reg_base);

    /*Configure LCD Panel*/
    ret = display_panel->Ops->Control(DISPALY_PANEL_CONFIG);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    dsi->state.panel_initialized = 1;

    return ret;
}

/**
  \fn          int32_t  DSI_StartVideoMode (DISPLAY_PANEL_DEVICE *display_panel,
                                            DSI_RESOURCES *dsi)
  \brief       Configure DSI to start Video mode.
  \param[in]   display_panel Pointer to display panel resources.
  \param[in]   dsi  Pointer to DSI resources.
  \return      \ref execution_status.
*/
static int32_t DSI_StartVideoMode (DISPLAY_PANEL_DEVICE *display_panel,
                                   DSI_RESOURCES *dsi)
{
    int32_t ret  = ARM_DRIVER_OK;

    if((dsi->state.dpi_configured == 0) && (dsi->state.panel_initialized) == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    dsi_power_up_disable(dsi->reg_base);

    dsi_auto_clklane_disable(dsi->reg_base);

    dsi_phy_txrequestclkhs_enable(dsi->reg_base);

    dsi_video_mode_enable(dsi->reg_base);

    dsi_power_up_enable(dsi->reg_base);

    /*Start LCD Panel*/
    ret = display_panel->Ops->Start();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    return ret;
}

/**
  \fn          int32_t  DSI_Stop (DISPLAY_PANEL_DEVICE *display_panel,
                                  DSI_RESOURCES *dsi)
  \brief       Shutdown DSI.
  \param[in]   display_panel Pointer to display panel resources.
  \param[in]   dsi  Pointer to DSI resources
  \return      \ref execution_status
*/
static int32_t DSI_Stop (DISPLAY_PANEL_DEVICE *display_panel,
                         DSI_RESOURCES *dsi)
{
    int32_t ret  = ARM_DRIVER_OK;

    if(dsi->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    dsi_power_up_disable(dsi->reg_base);

    /*Stop LCD Panel*/
    ret = display_panel->Ops->Stop();
    if(ret != ARM_DRIVER_OK)
        return ret;

    return ret;
}

/**
  \fn          void MIPI_DSI_ISR (DSI_RESOURCES *dsi)
  \brief       MIPI DSI interrupt service routine
  \param[in]   dsi  Pointer to DSI resources
*/
static void MIPI_DSI_ISR (DSI_RESOURCES *dsi)
{

    uint32_t int_st = dsi_irq0_status(dsi->reg_base);

    if(int_st & (DSI_IRQ0_ACK_WITH_ERR_0  | DSI_IRQ0_ACK_WITH_ERR_1  | \
                 DSI_IRQ0_ACK_WITH_ERR_2  | DSI_IRQ0_ACK_WITH_ERR_3  | \
                 DSI_IRQ0_ACK_WITH_ERR_4  | DSI_IRQ0_ACK_WITH_ERR_5  | \
                 DSI_IRQ0_ACK_WITH_ERR_6  | DSI_IRQ0_ACK_WITH_ERR_7  | \
                 DSI_IRQ0_ACK_WITH_ERR_8  | DSI_IRQ0_ACK_WITH_ERR_9  | \
                 DSI_IRQ0_ACK_WITH_ERR_10 | DSI_IRQ0_ACK_WITH_ERR_11 | \
                 DSI_IRQ0_ACK_WITH_ERR_12 | DSI_IRQ0_ACK_WITH_ERR_13 | \
                 DSI_IRQ0_ACK_WITH_ERR_14 | DSI_IRQ0_ACK_WITH_ERR_15))
    {
        dsi->cb_event(DSI_PHY_ERROR_EVENT);
    }

    if(int_st & (DSI_IRQ0_DPHY_ERRORS_0 | DSI_IRQ0_DPHY_ERRORS_1 | \
                 DSI_IRQ0_DPHY_ERRORS_3 | DSI_IRQ0_DPHY_ERRORS_4))
    {
        dsi->cb_event(DSI_ACK_ERROR_EVENT);
    }

    int_st = dsi_irq1_status(dsi->reg_base);

    if(int_st & (DSI_IRQ1_TO_HS_TX | DSI_IRQ1_TO_LP_RX              | \
                 DSI_IRQ1_ECC_SINGLE_ERR | DSI_IRQ1_ECC_MILTI_ERR   | \
                 DSI_IRQ1_CRC_ERR | DSI_IRQ1_PKT_SIZE_ERR           | \
                 DSI_IRQ1_EOPT_ERR))
    {
        dsi->cb_event(DSI_PKT_ERROR_EVENT);
    }

    if(int_st & (DSI_IRQ1_DPI_PLD_WR_ERR | DSI_IRQ1_GEN_CMD_WR_ERR    | \
                 DSI_IRQ1_GEN_PLD_WR_ERR | DSI_IRQ1_GEN_PLD_SEND_ERR  | \
                 DSI_IRQ1_GEN_PLD_RD_ERR | DSI_IRQ1_GEN_PLD_RECEV_ERR | \
                 DSI_IRQ1_DPI_BUFF_PLD_UNDER))
    {
        dsi->cb_event(DSI_DPI_ERROR_EVENT);
    }
}

/* DSI LCD Panel access structure */
static DISPLAY_PANEL_DEVICE *display_panel;

static DSI_FRAME_INFO DPI_FRAME_INFO;

static DSI_DPI_POLARITY DPI_POL =
{
    .dataen_active_low = RTE_MIPI_DSI_DPI_DATAEN_ACTIVE_LOW,
    .vsync_active_low  = RTE_MIPI_DSI_DPI_VSYNC_ACTIVE_LOW,
    .hsync_active_low  = RTE_MIPI_DSI_DPI_HSYNC_ACTIVE_LOW,
    .shutd_active_low  = RTE_MIPI_DSI_DPI_SHUTD_ACTIVE_LOW,
    .colorm_active_low = RTE_MIPI_DSI_DPI_COLORM_ACTIVE_LOW,
};

static DSI_DPI_INFO DPI_INFO =
{
    .color_code     = RTE_MIPI_DSI_DPI_COLOR_CODE,
    .vid_mode       = RTE_MIPI_DSI_VID_MODE_TYPE,
    .vid_num_chunks = RTE_MIPI_DSI_VID_NUM_CHUNKS,
    .vid_null_size  = RTE_MIPI_DSI_VID_NULL_SIZE,
    .dpi_pol        = &DPI_POL,
    .frame_info     = &DPI_FRAME_INFO,
};

static DSI_DPHY_TMR_CFG DPHY_TMR_CFG =
{
    .clklp2hs_time = RTE_MIPI_DSI_PHY_CLKLP2HS_TIME,
    .clkhs2lp_time = RTE_MIPI_DSI_PHY_CLKHS2LP_TIME,
    .lp2hs_time    = RTE_MIPI_DSI_PHY_LP2HS_TIME,
    .hs2lp_time    = RTE_MIPI_DSI_PHY_HS2LP_TIME,
};

static DSI_RESOURCES DSI_INFO =
{
    .reg_base       = (DSI_Type*)DSI_BASE,
    .n_lanes        = RTE_MIPI_DSI_N_LANES,
    .vc_id          = RTE_MIPI_DSI_VC_ID,
    .tx_ecs_clk_div = RTE_MIPI_DSI_TX_ESC_CLK_DIVISION,
    .dphy_tmr       = &DPHY_TMR_CFG,
    .dpi_info       = &DPI_INFO,
    .irq            = DSI_IRQ_IRQn,
    .irq_priority   = RTE_MIPI_DSI_IRQ_PRI,
};

/**
  \fn          void DSI_DCS_Short_Write (uint8_t cmd, uint8_t data)
  \brief       Perform MIPI DSI DCS Short write.
  \param[in]   cmd is DCS command info.
  \param[in]   data to send.
*/
void DSI_DCS_Short_Write (uint8_t cmd, uint8_t data)
{
    dsi_dcs_short_write(DSI_INFO.reg_base, cmd, data, DSI_INFO.vc_id);
    sys_busy_loop_us(100);
}

/**
  \fn          void DSI_DCS_CMD_Short_Write (uint8_t cmd)
  \brief       Perform MIPI DSI DCS Short write only command.
  \param[in]   cmd is DCS command info.
*/
void DSI_DCS_CMD_Short_Write (uint8_t cmd)
{
    dsi_dcs_cmd_short_write(DSI_INFO.reg_base, cmd, DSI_INFO.vc_id);
    sys_busy_loop_us(100);
}

/**
  \fn          void DSI_DCS_Long_Write (uint8_t cmd, uint32_t data)
  \brief       Perform MIPI DSI DCS Short write.
  \param[in]   cmd is DCS command info.
  \param[in]   data of four bytes to send.
*/
void DSI_DCS_Long_Write (uint8_t cmd, uint32_t data)
{
    dsi_dcs_long_write(DSI_INFO.reg_base, cmd, data, DSI_INFO.vc_id);
    sys_busy_loop_us(100);
}

/**
  \fn          int32_t  ARM_MIPI_DSI_Initialize (ARM_MIPI_DSI_SignalEvent_t cb_event)
  \brief       Initialize MIPI DSI Interface.
  \param[in]   cb_event Pointer to ARM_MIPI_DSI_SignalEvent_t
  \return      \ref execution_status
  */
static int32_t ARM_MIPI_DSI_Initialize (ARM_MIPI_DSI_SignalEvent_t cb_event)
{
    display_panel = get_display_panel();
    return DSI_Initialize (cb_event, display_panel, &DSI_INFO);
}

/**
  \fn          int32_t ARM_MIPI_DSI_Uninitialize (void)
  \brief       uninitialize MIPI DSI Interface.
  \return      \ref execution_status
  */
static int32_t ARM_MIPI_DSI_Uninitialize (void)
{
    return DSI_Uninitialize (display_panel, &DSI_INFO);
}

/**
  \fn          int32_t ARM_MIPI_DSI_PowerControl (ARM_POWER_STATE state)
  \brief       Control DSI Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
  */
static int32_t ARM_MIPI_DSI_PowerControl (ARM_POWER_STATE state)
{
    return DSI_PowerControl (state, &DSI_INFO);
}

/**
  \fn          int32_t ARM_MIPI_DSI_Control (ARM_MIPI_DSI_CONTROL control, uint32_t arg)
  \brief       Control DSI Interface.
  \param[in]   control DSI host and DPI Configuration.
  \param[in]   arg Argument of operation (optional)
  \return      \ref execution_status
  */
static int32_t ARM_MIPI_DSI_Control (ARM_MIPI_DSI_CONTROL control, uint32_t arg)
{
    return DSI_Control (control, arg, &DSI_INFO);
}

/**
  \fn          int32_t  ARM_MIPI_DSI_StartCommandMode (void)
  \brief       Configure DSI to start Command mode.
  \return      \ref execution_status
*/
static int32_t ARM_MIPI_DSI_StartCommandMode (void)
{
    return DSI_StartCommandMode (display_panel, &DSI_INFO);
}

/**
  \fn          int32_t  ARM_MIPI_DSI_StartVideoMode (void)
  \brief       Configure DSI to start Video mode.
  \return      \ref execution_status
*/
static int32_t ARM_MIPI_DSI_StartVideoMode (void)
{
    return DSI_StartVideoMode (display_panel, &DSI_INFO);
}

/**
  \fn          int32_t  ARM_MIPI_DSI_Stop (void)
  \brief       Shutdown DSI.
  \return      \ref execution_status
*/
static int32_t ARM_MIPI_DSI_Stop (void)
{
    return DSI_Stop (display_panel, &DSI_INFO);
}

/**
  \fn          void DSI_IRQHandler (void)
  \brief       DSi IRQ Handler.
*/
void DSI_IRQHandler(void)
{
    MIPI_DSI_ISR (&DSI_INFO);
}

/**
  \brief Access structure of the  MIPI DSI Driver.
  */
extern ARM_DRIVER_MIPI_DSI Driver_MIPI_DSI;

ARM_DRIVER_MIPI_DSI Driver_MIPI_DSI =
{
    ARM_MIPI_DSI_GetVersion,
    ARM_MIPI_DSI_GetCapabilities,
    ARM_MIPI_DSI_Initialize,
    ARM_MIPI_DSI_Uninitialize,
    ARM_MIPI_DSI_PowerControl,
    ARM_MIPI_DSI_Control,
    ARM_MIPI_DSI_StartCommandMode,
    ARM_MIPI_DSI_StartVideoMode,
    ARM_MIPI_DSI_Stop
};
