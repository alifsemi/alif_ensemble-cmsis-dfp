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
 * @file     DPHY_init.h
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     24-Feb-2022
 * @brief    Driver Specific Header file for DPHY Driver.
 ******************************************************************************/

#ifndef DPHY_INIT_H_
#define DPHY_INIT_H_

#include "RTE_Device.h"
#include <stdbool.h>

/**
 * enum DPHY_CLK_MODE
 * DPHY clock lane mode
 */
typedef enum _DPHY_CLK_MODE {
    DPHY_CLK_MODE_CONTINUOUS,           /**< DPHY clock mode continues     */
    DPHY_CLK_MODE_NON_CONTINUOUS,       /**< DPHY clock mode Non continues */
}DPHY_CLK_MODE;

#if (RTE_MIPI_DSI)
/**
  \fn          int32_t DSI_DPHY_Initialize (uint32_t frequency)
  \brief       Initialize MIPI DSI DPHY Interface.
  \param[in]   frequency to configure DPHY PLL.
  \return      \ref execution_status
  */
int32_t DSI_DPHY_Initialize (uint32_t frequency);

/**
  \fn          int32_t DSI_DPHY_Uninitialize (void)
  \brief       Uninitialize MIPI DSI DPHY Interface.
  \return      \ref execution_status
  */
int32_t DSI_DPHY_Uninitialize (void);
#endif

#if (RTE_MIPI_CSI2)
/**
  \fn          int32_t CSI2_DPHY_Initialize (uint32_t frequency, uint8_t n_lanes, DPHY_CLK_MODE clk_mode)
  \brief       Initialize MIPI CSI2 DPHY Interface.
  \param[in]   frequency to configure DPHY PLL.
  \param[in]   n_lanes number of lanes.
  \param[in]   clk_mode select DPHY clock mode \ref DPHY_CLK_MODE.
  \return      \ref execution_status
  */
int32_t CSI2_DPHY_Initialize (uint32_t frequency, uint8_t n_lanes, DPHY_CLK_MODE clk_mode);

/**
  \fn          int32_t CSI2_DPHY_Uninitialize (void)
  \brief       Uninitialize MIPI CSI2 DPHY Interface.
  \return      \ref execution_status
  */
int32_t CSI2_DPHY_Uninitialize (void);
#endif


#endif /* DPHY_INIT_H_ */
