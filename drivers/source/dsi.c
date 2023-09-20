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
 * @file     dsi.c
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     17-April-2023
 * @brief    Low level driver Specific Source file.
 ******************************************************************************/

#include "dsi.h"

/**
  \fn          void dsi_dpi_set_dataen_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
  \brief       Set dsi dpi dataen line polarity.
  \param[in]   dsi     Pointer to the dsi register map.
  \param[in]   polarity   dsi dataen polarity to set.
  \return      none.
*/
void dsi_dpi_set_dataen_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
{
    if(polarity == DSI_POLARITY_ACTIVE_HIGH)
    {
        dsi->DSI_DPI_CFG_POL &= ~(DSI_DATAEN_ACTIVE_LOW_MASK);
    }
    else
    {
        dsi->DSI_DPI_CFG_POL |= (DSI_DATAEN_ACTIVE_LOW_MASK);
    }
}

/**
  \fn          void dsi_dpi_set_vsync_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
  \brief       Set dsi dpi vsync line polarity.
  \param[in]   dsi     Pointer to the dsi register map.
  \param[in]   polarity   dsi vsync polarity to set.
  \return      none.
*/
void dsi_dpi_set_vsync_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
{
    if(polarity == DSI_POLARITY_ACTIVE_HIGH)
    {
        dsi->DSI_DPI_CFG_POL &= ~(DSI_VSYNC_ACTIVE_LOW_MASK);
    }
    else
    {
        dsi->DSI_DPI_CFG_POL |= (DSI_VSYNC_ACTIVE_LOW_MASK);
    }
}

/**
  \fn          void dsi_dpi_set_hsync_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
  \brief       Set dsi dpi hsync line polarity.
  \param[in]   dsi     Pointer to the dsi register map.
  \param[in]   polarity   dsi hsync polarity to set.
  \return      none.
*/
void dsi_dpi_set_hsync_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
{
    if(polarity == DSI_POLARITY_ACTIVE_HIGH)
    {
        dsi->DSI_DPI_CFG_POL &= ~(DSI_HSYNC_ACTIVE_LOW_MASK);
    }
    else
    {
        dsi->DSI_DPI_CFG_POL |= (DSI_HSYNC_ACTIVE_LOW_MASK);
    }
}

/**
  \fn          void dsi_dpi_set_shutd_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
  \brief       Set dsi dpi shutd line polarity.
  \param[in]   dsi     Pointer to the dsi register map.
  \param[in]   polarity   dsi shutd polarity to set.
  \return      none.
*/
void dsi_dpi_set_shutd_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
{
    if(polarity == DSI_POLARITY_ACTIVE_HIGH)
    {
        dsi->DSI_DPI_CFG_POL &= ~(DSI_SHUTD_ACTIVE_LOW_MASK);
    }
    else
    {
        dsi->DSI_DPI_CFG_POL |= (DSI_SHUTD_ACTIVE_LOW_MASK);
    }
}

/**
  \fn          void dsi_dpi_set_colorm_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
  \brief       Set dsi dpi colorm line polarity.
  \param[in]   dsi     Pointer to the dsi register map.
  \param[in]   polarity   dsi colorm polarity to set.
  \return      none.
*/
void dsi_dpi_set_colorm_polarity(DSI_Type *dsi, DSI_POLARITY polarity)
{
    if(polarity == DSI_POLARITY_ACTIVE_HIGH)
    {
        dsi->DSI_DPI_CFG_POL &= ~(DSI_COLORM_ACTIVE_LOW_MASK);
    }
    else
    {
        dsi->DSI_DPI_CFG_POL |= (DSI_COLORM_ACTIVE_LOW_MASK);
    }
}

/**
  \fn          void dsi_dcs_short_write(DSI_Type *dsi, uint8_t cmd, uint8_t data, uint8_t vc_id)
  \brief       Perform dsi dcs Short write.
  \param[in]   cmd is DCS command info.
  \param[in]   data to send.
  \param[in]   vc_id virtual channel ID.
  \return      none.
*/
void dsi_dcs_short_write(DSI_Type *dsi, uint8_t cmd, uint8_t data, uint8_t vc_id)
{
    dsi->DSI_GEN_HDR = (DSI_DCS_SHORT_WRITE_DATA_TYPE << DSI_GEN_DT) | \
                         (vc_id << DSI_GEN_VC) | (cmd << DSI_GEN_WC_LSBYTE) | \
                         (data << DSI_GEN_WC_MSBYTE);
}

/**
  \fn          void dsi_dcs_cmd_short_write(DSI_Type *dsi, uint8_t cmd, uint8_t vc_id)
  \brief       Perform dsi DCS Short write only command.
  \param[in]   cmd is DCS command info.
  \param[in]   vc_id virtual channel ID.
  \return      none.
*/
void dsi_dcs_cmd_short_write(DSI_Type *dsi, uint8_t cmd, uint8_t vc_id)
{
    dsi->DSI_GEN_HDR = (DSI_DCS_SHORT_WRITE_NODATA_TYPE << DSI_GEN_DT) | \
                       (vc_id << DSI_GEN_VC) | (cmd << DSI_GEN_WC_LSBYTE);
}

/**
  \fn          void dsi_dcs_long_write(DSI_Type *dsi, uint8_t cmd, uint32_t data, uint8_t vc_id)
  \brief       Perform dsi DCS Short write.
  \param[in]   cmd is DCS command info.
  \param[in]   data of four bytes to send.
  \param[in]   vc_id virtual channel ID.
  \return      none.
*/
void dsi_dcs_long_write(DSI_Type *dsi, uint8_t cmd, uint32_t data, uint8_t vc_id)
{
    dsi->DSI_GEN_PLD_DATA = (cmd << DSI_GEN_PLD_B1) | (cmd << DSI_GEN_PLD_B2)  | \
                            ((data << DSI_GEN_PLD_B3) & DSI_GEN_PLD_B3_MASK)   | \
                            ((data >> 8) << DSI_GEN_PLD_B4);

    dsi->DSI_GEN_PLD_DATA = ((data >> 16) << DSI_GEN_PLD_B1) | ((data >> 24) << DSI_GEN_PLD_B2);

    dsi->DSI_GEN_HDR = (DSI_DCS_LONG_WRITE_DATA_TYPE << DSI_GEN_DT) | (vc_id << DSI_GEN_VC) | \
                       (DSI_DCS_LONG_WRITE_DATA_LEN << DSI_GEN_WC_LSBYTE);
}

/**
  \fn          DSI_LANE_STOPSTATE dsi_get_lane_stopstate_status(DSI_Type *dsi, DSI_LANE lane)
  \brief       Get dsi lane stopstate status.
  \param[in]   dsi     Pointer to the dsi register map.
  \param[in]   lane    dsi lane.
  \return      dsi lane stopstate status.
*/
DSI_LANE_STOPSTATE dsi_get_lane_stopstate_status(DSI_Type *dsi, DSI_LANE lane)
{
    DSI_LANE_STOPSTATE ret = 0;

    switch (lane)
    {
        case DSI_LANE_CLOCK:
            ret = (dsi->DSI_PHY_STATUS & DSI_PHY_STOPSTATECLKLANE_MASK) >> DSI_PHY_STOPSTATECLKLANE;
            break;
        case DSI_LANE_0:
            ret =  (dsi->DSI_PHY_STATUS & DSI_PHY_STOPSTATELANE_0_MASK) >> DSI_PHY_STOPSTATELANE_0;
            break;
        case DSI_LANE_1:
            ret = (dsi->DSI_PHY_STATUS & DSI_PHY_STOPSTATELANE_1_MASK) >> DSI_PHY_STOPSTATELANE_1;
            break;
        default:
            break;
    }

    return ret;
}
