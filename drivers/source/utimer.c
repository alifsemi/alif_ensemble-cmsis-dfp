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
 * @file     utimer.h
 * @author   Girish BN, Manoj A Murudi
 * @email    girish.bn@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     02-April-2023
 * @brief    Low Level driver file for UTIMER.
 * @bug      None.
 * @Note     None
 *******************************************************************************/

#include "utimer.h"

/**
  \fn          void utimer_config_direction (UTIMER_Type *utimer, uint8_t channel, UTIMER_TYPE type, utimer_channel_config *ch_config)
  \brief       configure counter type for the UTIMER instance.
  \param[in]   utimer      Pointer to the UTIMER register map
  \param[in]   channel     channel number
  \param[in]   dir         counter direction
  \param[in]   ch_config   Pointer to the UTIMER channel specific config structure
  \return      none
*/
void utimer_config_direction (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER_DIR dir, utimer_channel_config *ch_config)
{
	UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    switch (dir)
    {
        case UTIMER_COUNTER_UP:
        {
            utimer_ch->UTIMER_CNTR_CTRL |= CNTR_CTRL_EN;

            if (ch_config->fixed_buffer)
            {
                utimer_ch->UTIMER_CNTR_CTRL |= CNTR_CTRL_SAWTOOTH_ONE_SHOT;
            }
            break;
        }
        case UTIMER_COUNTER_DOWN:
        {
            utimer_ch->UTIMER_CNTR_CTRL |= (CNTR_CTRL_EN|CNTR_CTRL_DIR_DOWN);

            if (ch_config->fixed_buffer)
            {
                utimer_ch->UTIMER_CNTR_CTRL |= CNTR_CTRL_SAWTOOTH_ONE_SHOT;
            }
            break;
        }
        case UTIMER_COUNTER_TRIANGLE:
        {
            if (ch_config->fixed_buffer)
            {
                utimer_ch->UTIMER_CNTR_CTRL |= (CNTR_CTRL_EN|CNTR_CTRL_TRIANGLE_ONE_SHOT);
            }
            else
            {
                if (ch_config->buf_trough_n_crest)
                {
                    utimer_ch->UTIMER_CNTR_CTRL |= (CNTR_CTRL_EN|CNTR_CTRL_TRIANGLE_BUF_TROUGH_CREST);
                }
                else
                {
                    utimer_ch->UTIMER_CNTR_CTRL |= (CNTR_CTRL_EN|CNTR_CTRL_TRIANGLE_BUF_TROUGH);
                }
            }
            break;
       }
    }
}

/**
  \fn          void utimer_config_mode (UTIMER_Type *utimer, uint8_t channel, UTIMER_MODE mode, utimer_channel_config *ch_config)
  \brief       configure counter mode for the UTIMER instance.
  \param[in]   utimer      Pointer to the UTIMER register map
  \param[in]   channel     channel number
  \param[in]   mode        counter mode
  \param[in]   ch_config   Pointer to the UTIMER channel specific config structure
  \return      none
*/
void utimer_config_mode (UTIMER_Type *utimer, uint8_t channel, UTIMER_MODE mode, utimer_channel_config *ch_config)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    switch (mode)
    {
        case UTIMER_MODE_BASIC:
        {
            /* enabling drive output */
            utimer_driver_output_enable(utimer, channel, ch_config);

            if (ch_config->driver_A)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_A |= (COMPARE_CTRL_DRV_DRIVER_EN|
                                                    (ch_config->driver_a_at_cycle_end<<2)|
                                                    (ch_config->driver_a_start_state<<4)|
                                                    (ch_config->driver_a_stop_state<<6));
            }

            if (ch_config->driver_B)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_B |= (COMPARE_CTRL_DRV_DRIVER_EN|
                                                    (ch_config->driver_b_at_cycle_end<<2)|
                                                    (ch_config->driver_b_start_state<<4)|
                                                    (ch_config->driver_b_stop_state<<6));
            }
            break;
        }
        case UTIMER_MODE_BUFFERING:
        {
            /* enabling drive output */
            utimer_driver_output_enable(utimer, channel, ch_config);

            utimer_ch->UTIMER_BUF_OP_CTRL |= BUF_OP_CTRL_CNTR_BUF_EN;

            if (ch_config->buffering_type)
            {//SET: Double Buffering
                utimer_ch->UTIMER_BUF_OP_CTRL |= BUF_OP_CTRL_CNTR_BUF_OP_BIT1;
            }
            else
            {//CLEAR: Single Buffering
                utimer_ch->UTIMER_BUF_OP_CTRL |= BUF_OP_CTRL_CNTR_BUF_OP_BIT0;
            }

            if (ch_config->driver_A)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_A |= (COMPARE_CTRL_DRV_DRIVER_EN|
                                                    (ch_config->driver_a_at_cycle_end<<2)|
                                                    (ch_config->driver_a_start_state<<4)|
                                                    (ch_config->driver_a_stop_state<<6));
            }

            if (ch_config->driver_B)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_B |= (COMPARE_CTRL_DRV_DRIVER_EN|
                                                    (ch_config->driver_b_at_cycle_end << 2)|
                                                    (ch_config->driver_b_start_state << 4)|
                                                    (ch_config->driver_b_stop_state << 6));
            }
            break;
        }
        case UTIMER_MODE_TRIGGERING:
        {
            /* disabling drive output */
            utimer_driver_output_disable(utimer, channel);
            break;
        }
        case UTIMER_MODE_CAPTURING:
        {
            /* disabling drive output */
            utimer_driver_output_disable(utimer, channel);

            if (ch_config->buffer_operation)
            {
                utimer_ch->UTIMER_BUF_OP_CTRL |= (BUF_OP_CTRL_CAPTURE_BUF_EN|
                                                 (ch_config->capt_buffer_type_A << 16)|
                                                 (ch_config->capt_buffer_type_B << 18));
            }
            break;
        }

        case UTIMER_MODE_COMPARING:
        {
            /* enabling drive output */
            utimer_driver_output_enable(utimer, channel, ch_config);

            if (ch_config->driver_A)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_A |= (COMPARE_CTRL_DRV_COMPARE_EN|COMPARE_CTRL_DRV_DRIVER_EN|
                                                    (ch_config->driver_a_at_cycle_end << 2)|
                                                    (ch_config->driver_a_at_comp_match)|
                                                    (ch_config->driver_a_start_state << 4)|
                                                    (ch_config->driver_a_stop_state << 6));

                utimer_ch->UTIMER_BUF_OP_CTRL |= ((ch_config->comp_buffer_at_crest << 24)|
                                                 (ch_config->comp_buffer_at_trough << 25));

                if (ch_config->buffer_operation)
                {
                    utimer_ch->UTIMER_BUF_OP_CTRL |= (BUF_OP_CTRL_COMPARE_BUF_EN|(ch_config->buffering_type << 26));
                }

                if (ch_config->fixed_buffer)
                {
                    utimer_ch->UTIMER_BUF_OP_CTRL |= BUF_OP_CTRL_FORCE_COMPARE_BUF_OP;
                }
                if (ch_config->dma_ctrl)
                {
                    utimer_ch->UTIMER_COMPARE_CTRL_A |= COMPARE_CTRL_DRV_DMA_CLEAR_EN;
                }
            }

            if (ch_config->driver_B)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_B |= (COMPARE_CTRL_DRV_DRIVER_EN|COMPARE_CTRL_DRV_COMPARE_EN|
                                                    (ch_config->driver_b_at_cycle_end << 2)|
                                                    (ch_config->driver_b_at_comp_match)|
                                                    (ch_config->driver_b_start_state << 4)|
                                                    (ch_config->driver_b_stop_state << 6));

                utimer_ch->UTIMER_BUF_OP_CTRL |= ((ch_config->comp_buffer_at_crest << 28)|
                                                 (ch_config->comp_buffer_at_trough << 29));

                if (ch_config->buffer_operation)
                {
                    utimer_ch->UTIMER_BUF_OP_CTRL |= (BUF_OP_CTRL_COMPARE_BUF_EN|(ch_config->buffering_type << 30));
                }
                if (ch_config->fixed_buffer)
                {
                    utimer_ch->UTIMER_BUF_OP_CTRL |= BUF_OP_CTRL_FORCE_COMPARE_BUF_OP;
                }
                if (ch_config->dma_ctrl)
                {
                    utimer_ch->UTIMER_COMPARE_CTRL_B |= COMPARE_CTRL_DRV_DMA_CLEAR_EN;
                }
            }
            break;
        }

        case UTIMER_MODE_DEAD_TIME:
        {
            /* enabling drive output */
            utimer_driver_output_enable(utimer, channel, ch_config);

            utimer_ch->UTIMER_DEAD_TIME_CTRL |= DEAD_TIME_CTRL_DT_EN;
            if (ch_config->buffer_operation)
            {
                utimer_ch->UTIMER_DEAD_TIME_CTRL |= DEAD_TIME_CTRL_DT_BUF_EN;
            }

            if (ch_config->driver_A)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_A |= (COMPARE_CTRL_DRV_DRIVER_EN|COMPARE_CTRL_DRV_COMPARE_EN|
                                                    (ch_config->driver_a_at_cycle_end<<2)|
                                                    (ch_config->driver_a_at_comp_match)|
                                                    (ch_config->driver_a_start_state<<4)|
                                                    (ch_config->driver_a_stop_state<<6));
            }
            if (ch_config->driver_B)
            {
                utimer_ch->UTIMER_COMPARE_CTRL_B |= (COMPARE_CTRL_DRV_DRIVER_EN|COMPARE_CTRL_DRV_COMPARE_EN|
                                                    (ch_config->driver_b_at_cycle_end << 2)|
                                                    (ch_config->driver_b_at_comp_match)|
                                                    (ch_config->driver_b_start_state << 4)|
                                                    (ch_config->driver_b_stop_state << 6));
            }
            break;
        }
    }
}

/**
  \fn          void utimer_set_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER counter_type, uint32_t value)
  \brief       set counter value for the UTIMER instance.
  \param[in]   utimer       Pointer to the UTIMER register map
  \param[in]   channel      channel number
  \param[in]   counter_type counter type
  \param[in]   value        counter value
  \return      none
*/
void utimer_set_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER counter_type, uint32_t value)
{
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    switch (counter_type)
    {
        case UTIMER_CNTR:
        {
            utimer_ch->UTIMER_CNTR = value;
            break;
        }
        case UTIMER_CNTR_PTR:
        {
            utimer_ch->UTIMER_CNTR_PTR = value;
            break;
        }
        case UTIMER_CNTR_PTR_BUF1:
        {
            utimer_ch->UTIMER_CNTR_PTR_BUF1 = value;
            break;
        }
        case UTIMER_CNTR_PTR_BUF2:
        {
            utimer_ch->UTIMER_CNTR_PTR_BUF2 = value;
            break;
        }
        case UTIMER_DT_UP:
        {
            utimer_ch->UTIMER_DT_UP = value;
            break;
        }
        case UTIMER_DT_UP_BUF1:
        {
            utimer_ch->UTIMER_DT_UP_BUF1 = value;
            break;
        }
        case UTIMER_DT_DOWN:
        {
            utimer_ch->UTIMER_DT_DOWN = value;
            break;
        }
        case UTIMER_DT_DOWN_BUF1:
        {
            utimer_ch->UTIMER_DT_DOWN_BUF1 = value;
            break;
        }
        case UTIMER_COMPARE_A:
        {
            utimer_ch->UTIMER_COMPARE_A = value;
            break;
        }
        case UTIMER_COMPARE_B:
        {
            utimer_ch->UTIMER_COMPARE_B = value;
            break;
        }
        case UTIMER_COMPARE_A_BUF1:
        {
            utimer_ch->UTIMER_COMPARE_A_BUF1 = value;
            break;
        }
        case UTIMER_COMPARE_B_BUF1:
        {
            utimer_ch->UTIMER_COMPARE_B_BUF1 = value;
            break;
        }
        case UTIMER_COMPARE_A_BUF2:
        {
            utimer_ch->UTIMER_COMPARE_A_BUF2 = value;
            break;
        }
        case UTIMER_COMPARE_B_BUF2:
        {
            utimer_ch->UTIMER_COMPARE_B_BUF2 = value;
            break;
        }
        default:
            break;
    }
}

/**
  \fn          uint32_t utimer_get_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER counter, utimer_channel_config *ch_config)
  \brief       get counter direction for the UTIMER instance.
  \param[in]   utimer         Pointer to the UTIMER register map
  \param[in]   channel        channel number
  \param[in]   counter_type   counter type
  \return      current counter value
*/
uint32_t utimer_get_count (UTIMER_Type *utimer, uint8_t channel, UTIMER_COUNTER counter_type)
{
    uint32_t value = 0;
    UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    switch (counter_type)
    {
        case UTIMER_CNTR:
        {
            value = utimer_ch->UTIMER_CNTR;
            break;
        }
        case UTIMER_CNTR_PTR:
        {
            value = utimer_ch->UTIMER_CNTR_PTR;
            break;
        }
        case UTIMER_CNTR_PTR_BUF1:
        {
            value = utimer_ch->UTIMER_CNTR_PTR_BUF1;
            break;
        }
        case UTIMER_CNTR_PTR_BUF2:
        {
            value = utimer_ch->UTIMER_CNTR_PTR_BUF2;
            break;
        }
        case UTIMER_DT_UP:
        {
            value = utimer_ch->UTIMER_DT_UP;
            break;
        }
        case UTIMER_DT_UP_BUF1:
        {
            value = utimer_ch->UTIMER_DT_UP_BUF1;
            break;
        }
        case UTIMER_DT_DOWN:
        {
            value = utimer_ch->UTIMER_DT_DOWN;
            break;
        }
        case UTIMER_DT_DOWN_BUF1:
        {
            value = utimer_ch->UTIMER_DT_DOWN_BUF1;
            break;
        }
        case UTIMER_COMPARE_A:
        {
            value = utimer_ch->UTIMER_COMPARE_A;
            break;
        }
        case UTIMER_COMPARE_B:
        {
            value = utimer_ch->UTIMER_COMPARE_B;
            break;
        }
        case UTIMER_COMPARE_A_BUF1:
        {
            value = utimer_ch->UTIMER_COMPARE_A_BUF1;
            break;
        }
        case UTIMER_COMPARE_B_BUF1:
        {
            value = utimer_ch->UTIMER_COMPARE_B_BUF1;
            break;
        }
        case UTIMER_COMPARE_A_BUF2:
        {
            value = utimer_ch->UTIMER_COMPARE_A_BUF2;
            break;
        }
        case UTIMER_COMPARE_B_BUF2:
        {
            value = utimer_ch->UTIMER_COMPARE_B_BUF2;
            break;
        }
        case UTIMER_CAPTURE_A:
        {
            value = utimer_ch->UTIMER_CAPTURE_A;
            break;
        }
        case UTIMER_CAPTURE_B:
        {
            value = utimer_ch->UTIMER_CAPTURE_B;
            break;
        }
        case UTIMER_CAPTURE_A_BUF1:
        {
            value = utimer_ch->UTIMER_CAPTURE_A_BUF1;
            break;
        }
        case UTIMER_CAPTURE_B_BUF1:
        {
            value = utimer_ch->UTIMER_CAPTURE_B_BUF1;
            break;
        }
        case UTIMER_CAPTURE_A_BUF2:
        {
            value = utimer_ch->UTIMER_CAPTURE_A_BUF2;
            break;
        }
        case UTIMER_CAPTURE_B_BUF2:
        {
            value = utimer_ch->UTIMER_CAPTURE_B_BUF2;
            break;
        }
    }
    return value;
}

/**
  \fn          void utimer_config_trigger (UTIMER_Type *utimer, uint8_t channel, UTIMER_TRIGGER_CONFIG *config_control, utimer_channel_config *ch_config)
  \brief       configure trigger for the UTIMER instance.
  \param[in]   utimer          Pointer to the UTIMER register map
  \param[in]   channel         channel number
  \param[in]   config_control  Pointer to a trigger configure type argument
  \param[in]   ch_config       Pointer to the UTIMER channel specific config structure
  \return      none
*/
void utimer_config_trigger (UTIMER_Type *utimer, uint8_t channel, UTIMER_TRIGGER_CONFIG *config_control, utimer_channel_config *ch_config)
{
	UTIMER_CHANNEL_Type *utimer_ch = (UTIMER_CHANNEL_Type*) (&utimer->UTIMER0_START_0_SRC) + channel;

    if(config_control->src_type == UTIMER_SRC_0)
    {
        switch (config_control->trigger_target)
        {
            case UTIMER_TRIGGER_START:
            {
                utimer_ch->UTIMER_START_1_SRC &= ~(CNTR_SRC1_PGM_EN);   /* disabling pgm_en */
                utimer_ch->UTIMER_START_0_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_STOP:
            {
                utimer_ch->UTIMER_STOP_1_SRC &= ~(CNTR_SRC1_PGM_EN);   /* disabling pgm_en */
                utimer_ch->UTIMER_STOP_0_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_CLEAR:
            {
                utimer_ch->UTIMER_CLEAR_1_SRC &= ~(CNTR_SRC1_PGM_EN);   /* disabling pgm_en */
                utimer_ch->UTIMER_CLEAR_0_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_UPCOUNT:
            {
                utimer_ch->UTIMER_UP_0_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_DOWNCOUNT:
            {
                utimer_ch->UTIMER_DOWN_0_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_CAPTURE_A:
            {
                utimer_ch->UTIMER_TRIG_CAPTURE_SRC_A_0 |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_CAPTURE_B:
            {
                utimer_ch->UTIMER_TRIG_CAPTURE_SRC_B_0 |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_DMA_CLEAR_A:
            {
                utimer_ch->UTIMER_DMA_CLEAR_SRC_A_0 |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_DMA_CLEAR_B:
            {
                utimer_ch->UTIMER_DMA_CLEAR_SRC_B_0 |= config_control->trigger_type;
                break;
            }
        }
    }

    if(config_control->src_type == UTIMER_SRC_1)
    {
        switch (config_control->trigger_target)
        {
            case UTIMER_TRIGGER_START:
            {
                utimer_ch->UTIMER_START_1_SRC &= ~(CNTR_SRC1_PGM_EN);   /* disabling pgm_en */
                utimer_ch->UTIMER_START_1_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_STOP:
            {
                utimer_ch->UTIMER_STOP_1_SRC &= ~(CNTR_SRC1_PGM_EN);   /* disabling pgm_en */
                utimer_ch->UTIMER_STOP_1_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_CLEAR:
            {
                utimer_ch->UTIMER_CLEAR_1_SRC &= ~(CNTR_SRC1_PGM_EN);   /* disabling pgm_en */
                utimer_ch->UTIMER_CLEAR_1_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_UPCOUNT:
            {
                utimer_ch->UTIMER_UP_1_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_DOWNCOUNT:
            {
                utimer_ch->UTIMER_DOWN_1_SRC |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_CAPTURE_A:
            {
                utimer_ch->UTIMER_TRIG_CAPTURE_SRC_A_1 |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_CAPTURE_B:
            {
                utimer_ch->UTIMER_TRIG_CAPTURE_SRC_B_1 |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_DMA_CLEAR_A:
            {
                utimer_ch->UTIMER_DMA_CLEAR_SRC_A_1 |= config_control->trigger_type;
                break;
            }
            case UTIMER_TRIGGER_DMA_CLEAR_B:
            {
                utimer_ch->UTIMER_DMA_CLEAR_SRC_B_1 |= config_control->trigger_type;
                break;
            }
        }
    }

    if(config_control->src_type == UTIMER_FAULT_TRIGGER)
    {
        if (ch_config->driver_A) {
            utimer_ch->UTIMER_FAULT_CTRL |= (config_control->trigger_type | ch_config->fault_type << 8);
        }
        if (ch_config->driver_B) {
            utimer_ch->UTIMER_FAULT_CTRL |= (config_control->trigger_type << 16 | ch_config->fault_type << 24);
        }
    }

    if(config_control->src_type == UTIMER_CNTR_PAUSE_TRIGGER)
    {
        utimer_ch->UTIMER_CNTR_PAUSE_SRC |= config_control->trigger_type;
    }
}
