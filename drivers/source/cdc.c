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
 * @file     cdc.c
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     10-April-2023
 * @brief    Low level driver Specific Source file.
 ******************************************************************************/

#include <cdc.h>

/**
 * @fn      void cdc_set_cfg (CDC_Type *const cdc, const cdc_cfg_info_t *const info)
 * @brief   Configure the CDC with given information.
 * @param   cdc   Pointer to the cdc register map structure. See {@ref CDC_Type} for details.
 * @param   info  Pointer to the cdc configuration structure. See {@ref cdc_cfg_info_t} for details.
 * @retval  none
 */
void cdc_set_cfg (CDC_Type *const cdc, const cdc_cfg_info_t *const info)
{
    /* The number of horizontal sync pixels and the number of vertical sync
     * lines (minus 1).*/
    const uint32_t sync_size = (((info->timing_info.hsync - 1) << 16 ) +
                                (info->timing_info.vsync - 1)           );

    /* The accumulated number of horizontal sync and back porch pixels
     * and the accumulated number of vertical sync and back porch lines
     * (minus 1).*/
    const uint32_t back_porch = ((info->timing_info.hbp << 16) +
                                 info->timing_info.vbp         +
                                 sync_size                       );

    /* The accumulated number of horizontal sync, back porch and active
     * pixels and the accumulated number of vertical sync, back porch
     * and active lines (minus 1).*/
    const uint32_t active_width = ((info->timing_info.hactive << 16) +
                                   info->timing_info.vactive         +
                                   back_porch                          );

    /* The accumulated number of horizontal sync, back porch, active
     * and front porch pixels and the accumulated number of vertical
     * sync, back porch, active and front porch lines (minus 1). */
    const uint32_t total_width = ((info->timing_info.hfp << 16) +
                                  info->timing_info.vfp         +
                                  active_width                    );

    /* Set frame timings */
    cdc->CDC_SYNC_SIZE_CFG = sync_size;
    cdc->CDC_BP_CFG = back_porch;
    cdc->CDC_ACTW_CFG = active_width;
    cdc->CDC_TOTALW_CFG = total_width;

    /* Set Background color */
    cdc->CDC_BACKGND_COLOR = ((info->bgc.red << 16)  |
                              (info->bgc.green << 8) |
                              info->bgc.blue          );

    /* Set scanline irq position */
    cdc->CDC_LINE_IRQ_POS = info->line_irq_pos;

    /* Trigger Shadow register update */
    cdc->CDC_SRCTRL = (1UL << info->sh_rld);
}

/**
 * @fn      void cdc_set_layer_cfg (CDC_Type *const cdc, const CDC_LAYER layer, const cdc_layer_info_t *const info)
 * @brief   Configure the CDC layer with given information.
 * @param   cdc   Pointer to the cdc register map structure. See {@ref CDC_Type} for details.
 * @param   layer The layer number needs to be configure. See {@ref CDC_LAYER} for details.
 * @param   info  Pointer to the cdc layer information structure. See {@ref cdc_layer_info_t} for details.
 * @retval  none
 */
void cdc_set_layer_cfg (CDC_Type *const cdc, const CDC_LAYER layer, const cdc_layer_info_t *const info)
{
    /* Color FB Length variable */
    uint32_t fb_length;

    /* Calculate frame buffer length */
    switch (info->pix_format)
    {
        case CDC_PIXEL_FORMAT_ARGB8888:
        case CDC_PIXEL_FORMAT_RGBA8888:
            /* In ARGB8888/RGBA8888 standard One pixel handled by 4 byte,
             * so line length size multiplied with 4 */
            fb_length = (((info->line_length_in_pixels * 4) << 16) |
                         ((info->line_length_in_pixels * 4) + BUS_WIDTH));
            break;
        case CDC_PIXEL_FORMAT_RGB888:
            /* In RGB888 standard One pixel handled by 3 byte,
             * so line length size multiplied with 3 */
            fb_length = (((info->line_length_in_pixels * 3) << 16) |
                         ((info->line_length_in_pixels * 3) + BUS_WIDTH));
            break;
        case CDC_PIXEL_FORMAT_RGB565:
        case CDC_PIXEL_FORMAT_ARGB4444:
        case CDC_PIXEL_FORMAT_ARGB1555:
            /* In RGB565/ARGB4444/ARGB1555 standard One pixel handled by 2 byte,
             * so width size multiplied with 2 */
            fb_length = (((info->line_length_in_pixels * 2) << 16) |
                         ((info->line_length_in_pixels * 2) + BUS_WIDTH));
            break;
        case CDC_PIXEL_FORMAT_AL44:
        case CDC_PIXEL_FORMAT_AL8:
        default:
            return ;
    }


    if (layer == CDC_LAYER_1)
    {
        /* Mask the global shadow reload */
        cdc->CDC_L1_REL_CTRL = CDC_Ln_REL_CTRL_SH_MASK;

        /* Set layer Window */
        cdc->CDC_L1_WIN_HPOS = ((info->win_info.h_stop_pos << 16) |
                                info->win_info.h_start_pos         );
        cdc->CDC_L1_WIN_VPOS = ((info->win_info.v_stop_pos << 16) |
                                info->win_info.v_start_pos         );

        /* Set pixel format */
        cdc->CDC_L1_PIX_FORMAT = info->pix_format;

        /* Set constant Alpha */
        cdc->CDC_L1_CONST_ALPHA = info->const_alpha;

        /* Set layer blending factor */
        if (info->blend_factor == CDC_BLEND_FACTOR_CONST_ALPHA)
        {
            cdc->CDC_L1_BLEND_CFG = ((CDC_BLEND_CONST_ALPHA << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT) |
                                     (CDC_BLEND_CONST_ALPHA_INV)                               );
        }
        else
        {
            cdc->CDC_L1_BLEND_CFG = ((CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT) |
                                     (CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA_INV)                               );
        }

        /* Set color frame buffer Address */
        cdc->CDC_L1_CFB_ADDR = info->fb_addr;

        /* Set the pitch and the line length of the color frame buffer*/
        cdc->CDC_L1_CFB_LENGTH = fb_length;

        /* Set color frame buffer lines */
        cdc->CDC_L1_CFB_LINES = info->num_lines;

        /* Trigger shadow register update */
        cdc->CDC_L1_REL_CTRL |= (1UL << info->sh_rld);
    }
    else
    {
        /* Mask the global shadow reload */
        cdc->CDC_L2_REL_CTRL = CDC_Ln_REL_CTRL_SH_MASK;

        /* Set layer Window */
        cdc->CDC_L2_WIN_HPOS = ((info->win_info.h_stop_pos << 16) |
                                info->win_info.h_start_pos         );
        cdc->CDC_L2_WIN_VPOS = ((info->win_info.v_stop_pos << 16) |
                                info->win_info.v_start_pos         );

        /* Set pixel format */
        cdc->CDC_L2_PIX_FORMAT = info->pix_format;

        /* Set constant Alpha */
        cdc->CDC_L2_CONST_ALPHA = info->const_alpha;

        /* Set layer blending factor */
        if (info->blend_factor == CDC_BLEND_FACTOR_CONST_ALPHA)
        {
            cdc->CDC_L2_BLEND_CFG = ((CDC_BLEND_CONST_ALPHA << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT) |
                                     (CDC_BLEND_CONST_ALPHA_INV)                               );
        }
        else
        {
            cdc->CDC_L2_BLEND_CFG = ((CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT) |
                                     (CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA_INV)                               );
        }

        /* Set color frame buffer Address */
        cdc->CDC_L2_CFB_ADDR = info->fb_addr;

        /* Set the pitch and the line length of the color frame buffer*/
        cdc->CDC_L2_CFB_LENGTH = fb_length;

        /* Set color frame buffer lines */
        cdc->CDC_L2_CFB_LINES = info->num_lines;

        /* Trigger shadow register update */
        cdc->CDC_L2_REL_CTRL |= (1UL << info->sh_rld);
    }

}

/**
 * @fn      void cdc_layer_on (CDC_Type *const cdc, const CDC_LAYER layer, const CDC_SHADOW_RELOAD sh_rld)
 * @brief   CDC layer on.
 * @param   cdc     Pointer to the cdc register map structure. See {@ref CDC_Type} for details.
 * @param   layer   The layer number needs to be configure. See {@ref CDC_LAYER} for details.
 * @param   sh_rld  The shadow register update method. See {@ref CDC_SHADOW_RELOAD} for details.
 * @retval  none
 */
void cdc_layer_on (CDC_Type *const cdc, const CDC_LAYER layer, const CDC_SHADOW_RELOAD sh_rld)
{
    if (layer == CDC_LAYER_1)
    {
        /* Enable Layer on */
        cdc->CDC_L1_CTRL |= CDC_LAYER_ON;

        /* Trigger shadow register update */
        cdc->CDC_L1_REL_CTRL |= (1UL << sh_rld);
    }
    else
    {
        /* Enable Layer on */
        cdc->CDC_L2_CTRL |= CDC_LAYER_ON;

        /* Trigger shadow register update */
        cdc->CDC_L2_REL_CTRL |= (1UL << sh_rld);
    }
}

/**
 * @fn      void cdc_layer_off (CDC_Type *const cdc, const CDC_LAYER layer, const CDC_SHADOW_RELOAD sh_rld)
 * @brief   CDC layer off.
 * @param   cdc     Pointer to the cdc register map structure. See {@ref CDC_Type} for details.
 * @param   layer   The layer number needs to be configure. See {@ref _CDC_LAYER} for details.
 * @param   sh_rld  The shadow register update method. See {@ref CDC_SHADOW_RELOAD} for details.
 * @retval  none
 */
void cdc_layer_off (CDC_Type *const cdc, const CDC_LAYER layer, const CDC_SHADOW_RELOAD sh_rld)
{
    if (layer == CDC_LAYER_1)
    {
        /* Enable Layer off */
        cdc->CDC_L1_CTRL &= ~CDC_LAYER_ON;

        /* Trigger shadow register update */
        cdc->CDC_L1_REL_CTRL |= (1UL << sh_rld);
    }
    else
    {
        /* Enable Layer off */
        cdc->CDC_L2_CTRL &= ~CDC_LAYER_ON;

        /* Trigger shadow register update */
        cdc->CDC_L2_REL_CTRL |= (1UL << sh_rld);
    }
}

/**
 * @fn      void cdc_set_layer_fb_addr (CDC_Type *const cdc, const CDC_LAYER layer,
 *                                      const CDC_SHADOW_RELOAD sh_rld, const uint32_t fb_addr)
 * @brief   Set layer frame buffer address..
 * @param   cdc      Pointer to the cdc register map structure. See {@ref CDC_Type} for details.
 * @param   layer    The layer number needs to be configure. See {@ref CDC_LAYER} for details.
 * @param   sh_rld   The shadow register update method. See {@ref CDC_SHADOW_RELOAD} for details.
 * @param   fb_addr  The Color FB start address.
 * @retval  none
 */
void cdc_set_layer_fb_addr (CDC_Type *const cdc, const CDC_LAYER layer,
                            const CDC_SHADOW_RELOAD sh_rld, const uint32_t fb_addr)
{
    if (layer == CDC_LAYER_1)
    {
        /* Set layer frame buffer */
        cdc->CDC_L1_CFB_ADDR = fb_addr;

        /* Trigger shadow register update */
        cdc->CDC_L1_REL_CTRL |= (1UL << sh_rld);
    }
    else
    {
        /* Set layer frame buffer */
        cdc->CDC_L2_CFB_ADDR = fb_addr;

        /* Trigger shadow register update */
        cdc->CDC_L2_REL_CTRL |= (1UL << sh_rld);
    }
}

/**
 * @fn      void cdc_set_layer_fb_window (CDC_Type *const cdc, const CDC_LAYER layer,
 *                                        const CDC_SHADOW_RELOAD sh_rld, const cdc_window_info_t *win_info)
 * @brief   Set layer frame buffer window.
 * @param   cdc       Pointer to the cdc register map structure. See {@ref CDC_Type} for details.
 * @param   layer     The layer number needs to be configure. See {@ref CDC_LAYER} for details.
 * @param   sh_rld    The shadow register update method. See {@ref CDC_SHADOW_RELOAD} for details.
 * @param   win_info  Pointer to the layer window information structure. See {@ref cdc_window_info_t} for details.
 * @retval  none
 */
void cdc_set_layer_fb_window (CDC_Type *const cdc, const CDC_LAYER layer,
                              const CDC_SHADOW_RELOAD sh_rld, const cdc_window_info_t *win_info)
{
    if (layer == CDC_LAYER_1)
    {
        /* Set layer Window */
        cdc->CDC_L1_WIN_HPOS = ((win_info->h_stop_pos << 16) |
                                win_info->h_start_pos         );
        cdc->CDC_L1_WIN_VPOS = ((win_info->v_stop_pos << 16) |
                                win_info->v_start_pos         );

        /* Trigger shadow register update */
        cdc->CDC_L1_REL_CTRL |= (1UL << sh_rld);
    }
    else
    {
        /* Set layer Window */
        cdc->CDC_L2_WIN_HPOS = ((win_info->h_stop_pos << 16) |
                                win_info->h_start_pos         );
        cdc->CDC_L2_WIN_VPOS = ((win_info->v_stop_pos << 16) |
                                win_info->v_start_pos         );

        /* Trigger shadow register update */
        cdc->CDC_L2_REL_CTRL |= (1UL << sh_rld);
    }
}

/**
 * @fn      void cdc_set_layer_blending (CDC_Type *const cdc, const CDC_LAYER layer, const CDC_SHADOW_RELOAD sh_rld,
 *                                       const uint8_t const_alpha, const CDC_BLEND_FACTOR blend_factor             )
 * @brief   Set layer blending.
 * @param   cdc           Pointer to the cdc register map structure. See {@ref CDC_Type} for details.
 * @param   layer         The layer number needs to be configure. See {@ref CDC_LAYER} for details.
 * @param   sh_rld        The shadow register update method. See {@ref CDC_SHADOW_RELOAD} for details.
 * @param   const_alpha   The layer constant alpha range from 0 (fully transparent) to 255 or 1.0 (fully opaque).
 * @param   blend_factor  The layer blending factor selection. See {@ref CDC_BLEND_FACTOR} for details.
 * @retval  none
 */
void cdc_set_layer_blending (CDC_Type *const cdc, const CDC_LAYER layer, const CDC_SHADOW_RELOAD sh_rld,
                             const uint8_t const_alpha, const CDC_BLEND_FACTOR blend_factor             )
{
    if (layer == CDC_LAYER_1)
    {
        /* Set constant alpha */
        cdc->CDC_L1_CONST_ALPHA = const_alpha;

        /* Set layer blending factor */
        if (blend_factor == CDC_BLEND_FACTOR_CONST_ALPHA)
        {
            cdc->CDC_L1_BLEND_CFG = ((CDC_BLEND_CONST_ALPHA  << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT) |
                                     (CDC_BLEND_CONST_ALPHA_INV)                                );
        }
        else
        {
            cdc->CDC_L1_BLEND_CFG = ((CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT) |
                                     (CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA_INV)                               );
        }

        /* Trigger shadow register update */
        cdc->CDC_L1_REL_CTRL |= (1UL << sh_rld);
    }
    else
    {
        /* Set constant alpha */
         cdc->CDC_L2_CONST_ALPHA = const_alpha;

         /* Set layer blending factor */
         if (blend_factor == CDC_BLEND_FACTOR_CONST_ALPHA)
         {
             cdc->CDC_L2_BLEND_CFG = ((CDC_BLEND_CONST_ALPHA << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT) |
                                      (CDC_BLEND_CONST_ALPHA_INV)                               );
         }
         else
         {
             cdc->CDC_L2_BLEND_CFG = ((CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA << CDC_Ln_BLEND_CFG_F1_SEL_SHIFT ) |
                                      (CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA_INV)                                );
         }

         /* Trigger shadow register update */
         cdc->CDC_L2_REL_CTRL |= (1UL << sh_rld);
    }
}
