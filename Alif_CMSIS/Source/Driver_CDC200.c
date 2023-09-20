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
 * @file     Driver_CDC200.c
 * @author   Girish BN and Prasanna Ravi
 * @email    girish.bn@alifsemi.com and prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     30-Sep-2021
 * @brief    Display controller CDC200 driver source file.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "Driver_CDC200.h"
#include "Driver_CDC_Private.h"
#include "sys_ctrl_cdc.h"
#include "cdc.h"
#include "system_utils.h"
#include "RTE_Device.h"
#include "display.h"

#define ARM_CDC200_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /*driver version*/

#if !(RTE_CDC200)
#error "CDC200 is not enabled in the RTE_Device.h"
#endif

#if (!defined(RTE_Drivers_CDC200))
#error "CDC200 not configured in RTE_Components.h!"
#endif

/*Error checking on RTE parameters*/
#if ((RTE_PANEL_HSYNC_TIME - 1) < 0) || ((RTE_PANEL_HSYNC_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal sync size"
#endif

#if ((RTE_PANEL_VSYNC_LINE - 1) < 0) || ((RTE_PANEL_VSYNC_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical sync size"
#endif

#if ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME - 1) < 0) || ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal back porch"
#endif

#if ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE - 1) < 0) || ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical back porch"
#endif

#if ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME - 1) < 0) \
                           || ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal active width"
#endif

#if ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE - 1) < 0) \
                           || ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical active width"
#endif

#if ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME + RTE_PANEL_HFP_TIME - 1) < 0) \
                           || ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME + RTE_PANEL_HFP_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal total width"
#endif

#if ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE + RTE_PANEL_VFP_LINE - 1) < 0) \
                           || ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE + RTE_PANEL_VFP_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical total width"
#endif

#if (RTE_MIPI_DSI)
#include "Driver_MIPI_DSI.h"
/*MIPI DSI driver instance*/
extern ARM_DRIVER_MIPI_DSI Driver_MIPI_DSI;
/*MIPI DSI driver callback*/
void MIPI_DSI_Event_Callback (uint32_t int_event);
#endif

/*Driver Version*/
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_CDC200_API_VERSION,
    ARM_CDC200_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_CDC200_CAPABILITIES DriverCapabilities =
{
    0, /* Not supports reentrant_operation */
    1, /* DPI interface supported */
    0  /* reserved (must be zero) */
};

//
//  Functions
//
/**
  \fn          ARM_DRIVER_VERSION CDC200_GetVersion (void)
  \brief       Get CDC200 driver version.
  \return      \ref ARM_DRIVER_VERSION.
*/
static ARM_DRIVER_VERSION CDC200_GetVersion (void)
{
    return DriverVersion;
}

/**
  \fn          ARM_CDC200_CAPABILITIES CDC200_GetCapabilities (void)
  \brief       Get CDC200 driver capabilities.
  \return      \ref ARM_CDC200_CAPABILITIES.
*/
static ARM_CDC200_CAPABILITIES CDC200_GetCapabilities (void)
{
    return DriverCapabilities;
}

/**
  \fn          static int32_t CDC200_Init (ARM_CDC200_SignalEvent_t cb_event, CDC_RESOURCES *cdc)
  \brief       Initialize CDC200 Interface.
  \param[in]   cb_event Pointer to ARM_CDC200_SignalEvent_t.
  \param[in]   cdc Pointer to CDC resources.
  \return      \ref execution_status.
*/
static int32_t CDC200_Init (ARM_CDC200_SignalEvent_t cb_event, CDC_RESOURCES *cdc)
{
    int32_t ret = ARM_DRIVER_OK;

    if (cdc->state.initialized == 1)
    {
        return ARM_DRIVER_OK;
    }

    if (!cb_event)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    cdc->cb_event = cb_event;

#if (RTE_MIPI_DSI)
    /*Initializing MIPI DSI, if the LCD Panel is MIPI DSI LCD Panel*/
    ret = Driver_MIPI_DSI.Initialize (MIPI_DSI_Event_Callback);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    cdc->state.initialized = 1;

    return ret;
}

/**
  \fn          static int32_t CDC200_Uninit (CDC_RESOURCES *cdc)
  \brief       uninitialize CDC200 Interface.
  \param[in]   cdc Pointer to CDC resources.
  \return      \ref execution_status.
  */
static int32_t CDC200_Uninit (CDC_RESOURCES *cdc)
{
    int32_t ret = ARM_DRIVER_OK;
    cdc->cb_event = NULL;

    if (cdc->state.initialized == 0)
    {
        return ARM_DRIVER_OK;
    }

    if (cdc->state.powered == 1)
    {
        return ARM_DRIVER_ERROR;
    }

#if (RTE_MIPI_DSI)
    /*Uninitializing MIPI DSI, if the LCD Panel is MIPI DSI LCD Panel*/
    ret = Driver_MIPI_DSI.Uninitialize ();
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

#endif

    cdc->state.initialized = 0;
    return ret;
}

/**
  \fn          static int32_t CDC200_PowerCtrl (ARM_POWER_STATE state, CDC_RESOURCES *cdc)
  \brief       Control CDC200 Interface Power.
  \param[in]   state Power state.
  \param[in]   cdc Pointer to CDC resources.
  \return      \ref execution_status.
  */
static int32_t CDC200_PowerCtrl (ARM_POWER_STATE state, CDC_RESOURCES *cdc)
{
    uint32_t htotal, vtotal;
    int pixclk_div;
    int32_t ret = ARM_DRIVER_OK;

    if (cdc->state.initialized == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:

            if (cdc->state.powered == 0)
            {
                return ARM_DRIVER_OK;
            }

#if (RTE_MIPI_DSI)
            /*Disable MIPI DSI*/
            ret = Driver_MIPI_DSI.PowerControl (ARM_POWER_OFF);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif
            /*Disabling Line IRQ*/
            NVIC_DisableIRQ (CDC_SCANLINE0_IRQ_IRQn);
            NVIC_ClearPendingIRQ (CDC_SCANLINE0_IRQ_IRQn);

            /* Disabling pixel clock */
            disable_cdc_pixel_clk ();

            /* Disabling Source clock */
            disable_dpi_periph_clk ();

            cdc->state.powered = 0;
            break;

        case ARM_POWER_FULL:

            if (cdc->state.powered == 1)
            {
                return ARM_DRIVER_OK;
            }

#if (RTE_MIPI_DSI)
            /*Enable MIPI DSI*/
            ret = Driver_MIPI_DSI.PowerControl (ARM_POWER_FULL);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif

            /* LCD Manufacturer provides the Frame timing values
             *     HTOTAL = WIDTH + HSYNC + HFP + HBP
             *     VTOTAL = HEIGHT + VSYNC + VFP + VBP
             * Calculate the pixel clock for DPI controller
             *     PIXCLK = FPS x HTOTAL x VTOTAL
             * Calculate the pixel clock divider
             *     PIXCLK_DIV = CDC200_PIXCLK_SOURCE / PIXCLK
             */
            htotal = (cdc->frame_info->hsync_time +
                      cdc->frame_info->hbp_time   +
                      cdc->frame_info->hfp_time   +
                      cdc->frame_info->hactive_time);

            vtotal = (cdc->frame_info->vsync_line +
                      cdc->frame_info->vbp_line   +
                      cdc->frame_info->vfp_line   +
                      cdc->frame_info->vactive_line);

            pixclk_div = (int)((float)CDC200_PIXCLK / (htotal * vtotal * RTE_CDC200_DPI_FPS) + 0.5f);

            /*Checking clk divider is less than 2 because 0 and 1 are illegal value*/
            if (pixclk_div < 2 || pixclk_div > 511)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            /* Enabling Source clock */
            enable_dpi_periph_clk ();

            /* Configuring pixel clock */
            set_cdc_pixel_clk (RTE_CDC200_CLK_SEL, pixclk_div);

            /*Enabling Line IRQ*/
            NVIC_ClearPendingIRQ (CDC_SCANLINE0_IRQ_IRQn);
            NVIC_SetPriority (CDC_SCANLINE0_IRQ_IRQn, cdc->irq_priority);
            NVIC_EnableIRQ (CDC_SCANLINE0_IRQ_IRQn);

            cdc->state.powered = 1;
            break;

        case ARM_POWER_LOW:
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/**
 \fn          static int32_t ARM_CDC200_Control (uint32_t control, uint32_t arg)
 \brief       Control the display controller.
 \param[in]   control CDC200 contol code operation.
                - \ref CDC200_CONFIGURE_DISPLAY :         Configure Display
                - \ref CDC200_FRAMEBUF_UPDATE :           Update layer Frame buffer
                - \ref CDC200_SCANLINE0_EVENT :           Enable/Disable Scanline0 event
                - \ref CDC200_CONFIGURE_LAYER :           Configure Layer
                - \ref CDC200_LAYER_ON :                  Turn On the Layer
                - \ref CDC200_LAYER_OFF :                 Turn Off the Layer
                - \ref CDC200_CONFIGURE_LAYER_WINDOW :    Configure Layer window
                - \ref CDC200_CONFIGURE_BG_COLOR :        Configure Background color
                - \ref CDC200_CONFIGURE_LAYER_BLENDING :  Configure Layer blending
 \param[in]   arg Argument of operation.
               - CDC200_CONFIGURE_DISPLAY :         Frame buffer address
               - CDC200_FRAMEBUF_UPDATE :           Frame buffer address
               - CDC200_SCANLINE0_EVENT :           ENABLE/DISABLE
               - CDC200_CONFIGURE_LAYER :           Pointer to layer info \ref ARM_CDC200_LAYER_INFO
               - CDC200_LAYER_ON :                  layer index /ref ARM_CDC200_LAYER_INDEX
               - CDC200_LAYER_OFF :                 layer index /ref ARM_CDC200_LAYER_INDEX
               - CDC200_CONFIGURE_LAYER_WINDOW :    Pointer to layer info \ref ARM_CDC200_LAYER_INFO
               - CDC200_CONFIGURE_BG_COLOR :        Background color setting
                                                      - /ref ARM_CDC200_BGC_BLUE(x)
                                                      - /ref ARM_CDC200_BGC_GREEN(x)
                                                      - /ref ARM_CDC200_BGC_RED(x)
               - CDC200_CONFIGURE_LAYER_BLENDING :  Pointer to layer info \ref ARM_CDC200_LAYER_INFO
 \return      \ref execution_status.
 */
static int32_t CDC200_control (uint32_t control, uint32_t arg, CDC_RESOURCES *cdc)
{
    uint32_t ret = ARM_DRIVER_OK;

    if (cdc->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        case CDC200_CONFIGURE_DISPLAY:
        {
            if (arg == NULL)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            cdc_cfg_info_t cdc_info;
            cdc_layer_info_t layer_info;

            /*Setup display controller*/
            cdc_info.timing_info.hactive = cdc->frame_info->hactive_time;
            cdc_info.timing_info.hfp = cdc->frame_info->hfp_time;
            cdc_info.timing_info.hbp = cdc->frame_info->hbp_time;
            cdc_info.timing_info.hsync = cdc->frame_info->hsync_time;
            cdc_info.timing_info.vactive = cdc->frame_info->vactive_line;
            cdc_info.timing_info.vfp = cdc->frame_info->vfp_line;
            cdc_info.timing_info.vbp = cdc->frame_info->vbp_line;
            cdc_info.timing_info.vsync = cdc->frame_info->vsync_line;
            cdc_info.bgc.red = cdc->bgc->red;
            cdc_info.bgc.green = cdc->bgc->green;
            cdc_info.bgc.blue = cdc->bgc->blue;
            cdc_info.line_irq_pos = (cdc->frame_info->vsync_line +
                                     cdc->frame_info->vbp_line   +
                                     cdc->frame_info->vactive_line);
            cdc_info.sh_rld = CDC_SHADOW_RELOAD_IMR;

            layer_info.fb_addr = arg;
            layer_info.line_length_in_pixels = cdc->frame_info->hactive_time;
            layer_info.const_alpha = cdc->const_alpha;
            layer_info.blend_factor = cdc->blend_factor;
            layer_info.num_lines = cdc->frame_info->vactive_line;
            layer_info.pix_format = cdc->pixel_format;
            layer_info.win_info.h_start_pos = (cdc->frame_info->hsync_time +
                                               cdc->frame_info->hbp_time    );
            layer_info.win_info.h_stop_pos = ((cdc->frame_info->hsync_time +
                                               cdc->frame_info->hbp_time   +
                                               cdc->frame_info->hactive_time) - 1);
            layer_info.win_info.v_start_pos = (cdc->frame_info->vsync_line +
                                               cdc->frame_info->vbp_line    );
            layer_info.win_info.v_stop_pos = ((cdc->frame_info->vsync_line +
                                               cdc->frame_info->vbp_line   +
                                               cdc->frame_info->vactive_line) - 1);
            layer_info.sh_rld = CDC_SHADOW_RELOAD_IMR;

            cdc_set_cfg (cdc->regs, &cdc_info);
            cdc_set_layer_cfg (cdc->regs, CDC_LAYER_1, &layer_info);
            cdc_layer_on (cdc->regs, CDC_LAYER_1, CDC_SHADOW_RELOAD_IMR);

#if (RTE_MIPI_DSI)
            /*MIPI DSI Configure Host*/
            ret = Driver_MIPI_DSI.Control (DSI_CONFIGURE_HOST, 0);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }

            /*Start Command mode and  Configure LCD Panel*/
            ret = Driver_MIPI_DSI.StartCommandMode ();
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }

            /*MIPI DSI Configure DPI*/
            ret = Driver_MIPI_DSI.Control (DSI_CONFIGURE_DPI, 0);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif

            cdc->state.configured = 1;
            break;
        }
        case CDC200_FRAMEBUF_UPDATE:
        {
            if (arg == NULL)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            /*Update the buffer start address for new buffer content*/
            cdc_set_layer_fb_addr (cdc->regs, CDC_LAYER_1, CDC_SHADOW_RELOAD_IMR, arg);
            break;
        }
        case CDC200_SCANLINE0_EVENT:
        {
            /*Enable/Disable Scanline0 IRQ*/
            if(arg == ENABLE)
            {
                cdc_irq_enable (cdc->regs, CDC_IRQ_LINE);
            }
            else if (arg == DISABLE)
            {
                cdc_irq_disable (cdc->regs, CDC_IRQ_LINE);
            }
            else
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            break;
        }
        case CDC200_CONFIGURE_LAYER:
        {
            if (arg == NULL)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            ARM_CDC200_LAYER_INFO *cdc200_layer_info = (ARM_CDC200_LAYER_INFO *)arg;
            cdc_layer_info_t layer_info;

            /* Configure Layer */
            layer_info.win_info.v_start_pos = cdc200_layer_info->win_info.v_start_pos;
            layer_info.win_info.v_stop_pos = cdc200_layer_info->win_info.v_stop_pos;
            layer_info.win_info.h_start_pos = cdc200_layer_info->win_info.h_start_pos;
            layer_info.win_info.h_stop_pos = cdc200_layer_info->win_info.h_stop_pos;
            layer_info.pix_format = (CDC_PIXEL_FORMAT)cdc200_layer_info->pix_format;
            layer_info.const_alpha = cdc200_layer_info->const_alpha;
            layer_info.blend_factor = (CDC_BLEND_FACTOR)cdc200_layer_info->blend_factor;
            layer_info.fb_addr = cdc200_layer_info->fb_addr;
            layer_info.line_length_in_pixels = cdc200_layer_info->line_length_in_pixels;
            layer_info.num_lines = cdc200_layer_info->num_lines;
            layer_info.sh_rld = CDC_SHADOW_RELOAD_IMR;

            cdc_set_layer_cfg (cdc->regs, (CDC_LAYER)cdc200_layer_info->layer_idx, &layer_info);
            break;
        }
        case CDC200_LAYER_ON:
        {
            if((arg != ARM_CDC200_LAYER_1) && (arg != ARM_CDC200_LAYER_2))
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            /* Turn On layer */
            cdc_layer_on (cdc->regs, arg, CDC_SHADOW_RELOAD_IMR);
            break;
        }
        case CDC200_LAYER_OFF:
        {
            if((arg != ARM_CDC200_LAYER_1) && (arg != ARM_CDC200_LAYER_2))
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            /* Turn Off layer */
            cdc_layer_off (cdc->regs, arg, CDC_SHADOW_RELOAD_IMR);
            break;
        }
        case CDC200_CONFIGURE_LAYER_WINDOW:
        {
            if (arg == NULL)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            ARM_CDC200_LAYER_INFO *cdc200_layer_info = (ARM_CDC200_LAYER_INFO *)arg;
            cdc_window_info_t win_info;

            /* Configure Layer window*/
            win_info.v_start_pos = cdc200_layer_info->win_info.v_start_pos;
            win_info.v_stop_pos = cdc200_layer_info->win_info.v_stop_pos;
            win_info.h_start_pos = cdc200_layer_info->win_info.h_start_pos;
            win_info.h_stop_pos = cdc200_layer_info->win_info.h_stop_pos;

            cdc_set_layer_fb_window (cdc->regs, (CDC_LAYER)cdc200_layer_info->layer_idx, CDC_SHADOW_RELOAD_IMR, &win_info);
            break;
        }
        case CDC200_CONFIGURE_BG_COLOR:
        {
            cdc_backgnd_color_info_t bgc_info;

            /* Configure Background color*/
            bgc_info.red = _FLD2VAL(ARM_CDC200_BGC_RED, arg);
            bgc_info.green = _FLD2VAL(ARM_CDC200_BGC_GREEN, arg);
            bgc_info.blue = _FLD2VAL(ARM_CDC200_BGC_BLUE, arg);

            cdc_set_backgnd_color(cdc->regs, CDC_SHADOW_RELOAD_IMR, &bgc_info);
            break;
        }
        case CDC200_CONFIGURE_LAYER_BLENDING:
        {
            if (arg == NULL)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            ARM_CDC200_LAYER_INFO *cdc200_layer_info = (ARM_CDC200_LAYER_INFO *)arg;

            /* Configure Layer Blending*/
            cdc_set_layer_blending (cdc->regs, (CDC_LAYER)cdc200_layer_info->layer_idx, CDC_SHADOW_RELOAD_IMR,
                                    cdc200_layer_info->const_alpha, (CDC_BLEND_FACTOR)cdc200_layer_info->blend_factor);
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
 \fn            static int32_t CDC200_GetVerticalPos (CDC_RESOURCES *cdc)
 \brief         Get current vertical position count.
 \param[in]     cdc Pointer to CDC resources.
 \return        return current vertical position.
 */
static int32_t CDC200_GetVerticalPos (CDC_RESOURCES *cdc)
{
    CDC_FRAME_INFO *frame_info = (CDC_FRAME_INFO *)cdc->frame_info;

    return (int) cdc_get_y_position_status (cdc->regs) - frame_info->vsync_line - frame_info->vbp_line;
}
/**
 \fn            static int32_t CDC200_Start (CDC_RESOURCES *cdc)
 \brief         Start the display controller.
 \param[in]     cdc Pointer to CDC resources.
 \return        \ref execution_status.
 */
static int32_t CDC200_Start (CDC_RESOURCES *cdc)
{
    uint32_t ret = ARM_DRIVER_OK;

    if (cdc->state.configured == 0)
    {
        return ARM_DRIVER_ERROR;
    }

#if (RTE_MIPI_DSI)
    /*Start MIPI DSI and LCD Panel*/
    ret = Driver_MIPI_DSI.StartVideoMode ();
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    /*Enable Display Controller CDC200*/
    cdc_global_enable (cdc->regs);

    return ret;
}

/**
 \fn            static int32_t CDC200_Stop (CDC_RESOURCES *cdc)
 \brief         Stop the display controller.
 \param[in]     cdc Pointer to CDC resources.
 \return        \ref execution_status.
 */
static int32_t CDC200_Stop (CDC_RESOURCES *cdc)
{
    uint32_t ret = ARM_DRIVER_OK;

    if (cdc->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    /*Disable Display Controller CDC200*/
    cdc_global_disable (cdc->regs);

#if (RTE_MIPI_DSI)
    /*Stop MIPI DSI and LCD Panel*/
    ret = Driver_MIPI_DSI.Stop ();
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    return ret;
}

/**
  \fn          static void CDC200_ISR (CDC_RESOURCES *cdc)
  \brief       CDC200 interrupt service routine
  \param[in]   cdc  Pointer to CDC resources
*/
static void CDC200_ISR (CDC_RESOURCES *cdc)
{
    uint32_t irq_st = cdc_get_irq_status (cdc->regs);

    if (!(cdc->cb_event))
    {
        return;
    }

    if (irq_st & CDC_IRQ_LINE)
    {
        cdc->cb_event (ARM_CDC_SCANLINE0_EVENT);
        cdc_irq_clear (cdc->regs, CDC_IRQ_LINE);
    }
}

#if (RTE_CDC200)

CDC_FRAME_INFO FRAME_INFO =
{
    .hsync_time   = RTE_PANEL_HSYNC_TIME,
    .hbp_time     = RTE_PANEL_HBP_TIME,
    .hfp_time     = RTE_PANEL_HFP_TIME,
    .hactive_time = RTE_PANEL_HACTIVE_TIME,
    .vsync_line   = RTE_PANEL_VSYNC_LINE,
    .vbp_line     = RTE_PANEL_VBP_LINE,
    .vfp_line     = RTE_PANEL_VFP_LINE,
    .vactive_line = RTE_PANEL_VACTIVE_LINE,
};

cdc_backgnd_color_info_t BGC_INFO =
{
        .red   = RTE_CDC200_BGC_RED,
        .green = RTE_CDC200_BGC_GREEN,
        .blue  = RTE_CDC200_BGC_BLUE
};

CDC_RESOURCES CDC_INFO =
{
    .regs           = (CDC_Type*) CDC_BASE,
    .cb_event       = NULL,
    .bgc            = &BGC_INFO,
    .frame_info     = &FRAME_INFO,
    .pixel_format   = RTE_CDC200_PIXEL_FORMAT,
    .const_alpha    = RTE_CDC200_CONSTANT_ALPHA,
    .blend_factor   = RTE_CDC200_BLEND_FACTOR,
    .irq_priority   = RTE_CDC200_IRQ_PRI,
    .state          = {0},
};

#if (RTE_MIPI_DSI)
/**
  \fn          void MIPI_DSI_Event_Callback (uint32_t int_event)
  \brief       Signal MIPI DSI Events.
  \param[in]   int_event  \ref MIPI DSI event types.
  \return      none.
*/
void MIPI_DSI_Event_Callback (uint32_t int_event)
{
    ARG_UNUSED(int_event);
    CDC_INFO.cb_event (ARM_CDC_DSI_ERROR_EVENT);
}
#endif

/**
  \fn          int32_t CDC200_Initialize (ARM_CDC200_SignalEvent_t cb_event)
  \brief       Initialize CDC200 Interface.
  \param[in]   cb_event Pointer to ARM_CDC200_SignalEvent_t.
  \return      \ref execution_status.
  */
static int32_t CDC200_Initialize (ARM_CDC200_SignalEvent_t cb_event)
{
    return CDC200_Init (cb_event, &CDC_INFO);
}

/**
  \fn          int32_t CDC200_Uninitialize (void)
  \brief       Uninitialize CDC200 Interface.
  \return      \ref execution_status.
  */
static int32_t CDC200_Uninitialize (void)
{
    return CDC200_Uninit (&CDC_INFO);
}

/**
  \fn          int32_t CDC200_PowerControl (ARM_POWER_STATE state)
  \brief       Control CDC200 Interface Power.
  \param[in]   state  Power state.
  \return      \ref execution_status.
  */
static int32_t CDC200_PowerControl (ARM_POWER_STATE state)
{
    return CDC200_PowerCtrl (state, &CDC_INFO);
}

/**
 \fn            int32_t CDC200_Control (uint32_t control, uint32_t arg)
 \brief         Control the display controller.
 \param[in]     control CDC200 Configuration.
 \param[in]     arg Argument of operation (optional).
 \return        \ref execution_status.
 */
static int32_t CDC200_Control (uint32_t control, uint32_t arg)
{
    return CDC200_control (control, arg, &CDC_INFO);
}

/**
 \fn            int32_t CDC200_GetVerticalPosition (void)
 \brief         Get current vertical position count.
 \return        return current vertical Position.
 */
static int32_t CDC200_GetVerticalPosition (void)
{
    return CDC200_GetVerticalPos (&CDC_INFO);
}

/**
 \fn            int32_t CDC200_StartDisplay (void)
 \brief         Start the display controller.
 \return        \ref execution_status.
 */
static int32_t CDC200_StartDisplay (void)
{
    return CDC200_Start (&CDC_INFO);
}

/**
 \fn            int32_t CDC200_StopDisplay (void)
 \brief         Stop the display controller.
 \return        \ref execution_status.
 */
static int32_t CDC200_StopDisplay (void)
{
    return CDC200_Stop (&CDC_INFO);
}

/**
  \fn          void CDC200_SCANLINE0_IRQHandler (void)
  \brief       CDC200 SCANLINE0 IRQ Handler.
*/
void CDC_SCANLINE0_IRQHandler(void)
{
    CDC200_ISR (&CDC_INFO);
}

extern ARM_DRIVER_CDC200 Driver_CDC200;

ARM_DRIVER_CDC200 Driver_CDC200 =
{
    CDC200_GetVersion,
    CDC200_GetCapabilities,
    CDC200_Initialize,
    CDC200_Uninitialize,
    CDC200_PowerControl,
    CDC200_Control,
    CDC200_GetVerticalPosition,
    CDC200_StartDisplay,
    CDC200_StopDisplay
};
#endif
