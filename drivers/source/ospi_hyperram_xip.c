/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     ospi_hyperram_xip.c
 * @author   Silesh C V, Manoj A Murudi
 * @email    silesh@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     19-Jul-2023
 * @brief    Implementation of the OSPI hyperram XIP init library.
 ******************************************************************************/

#include <stdint.h>
#include <stddef.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "ospi_hyperram_xip.h"
#include "ospi.h"
#include "soc.h"
#include "sys_clocks.h"
#include "sys_utils.h"

#include "sys_ctrl_aes.h"

/**
  \fn          int ospi_set_speed(OSPI_Type *ospi, AES_Type *aes, const ospi_hyperram_xip_config
  *config) \brief       Set OSPI bus speed for spi transfer. \param[in]   ospi : Pointer to the OSPI
  register map. \param[in]   aes  : Pointer to the AES register map. \param[in]   config  : Pointer
  to hyperram configuration information. \return      -1 on configuration error, 0 on success
 */
static int ospi_set_speed(OSPI_Type *ospi, AES_Type *aes, const ospi_hyperram_xip_config *config)
{
    uint32_t baud;

    if (config->bus_speed == 0) {
        return -1;
    }

    baud = (GetSystemAXIClock() / config->bus_speed);

    if (baud == 0) {
        return -1;
    }

    if (baud < 4) {
#if SOC_FEAT_AES_BAUD2_DELAY_VAL
        {
            aes_set_baud2_delay(aes);
        }
#elif SOC_FEAT_AES_OSPI_SIGNALS_DELAY
        {
            aes_set_signal_delay(aes, config->signal_delay);
        }
#else
        {
            ARG_UNUSED(aes);
            return -1;
        }
#endif
    }
    ospi_set_baud(ospi, baud);

    return 0;
}

/**
  \fn          int ospi_hyperram_xip_init(const ospi_hyperram_xip_config *config)
  \brief       Initialize OSPI Hyerbus xip configuration. After a successful return
               from this function, the OSPI XIP region (for the OSPI instance specified
               in the ospi_hyerram_xip_config input parameter) will be active and can be
               used to directly read/write the memory area provided by the hyperram device.
  \param[in]   config    Pointer to hyperram configuration information
  \return      -1 on configuration error, 0 on success
*/
int ospi_hyperram_xip_init(const ospi_hyperram_xip_config *config)
{
    OSPI_Type *ospi          = NULL;
    AES_Type  *aes           = NULL;
    bool       is_dual_octal = 0;

    if (!config) {
        return -1;
    }

    /* Setup the OSPI/AES register map pointers based on the OSPI instance */
    if (config->instance == OSPI_INSTANCE_0) {
        ospi = (OSPI_Type *) OSPI0_BASE;
        aes  = (AES_Type *) AES0_BASE;
    }
#ifdef RTE_OSPI1
    else {
        ospi = (OSPI_Type *) OSPI1_BASE;
        aes  = (AES_Type *) AES1_BASE;
    }
#endif

#if SOC_FEAT_OSPI_HAS_CLK_ENABLE
    enable_ospi_clk(config->instance);
#endif

    ospi_set_ddr_drive_edge(ospi, config->ddr_drive_edge);

    if (ospi_set_speed(ospi, aes, config)) {
        return -1;
    }

    aes_set_rxds_delay(aes, config->rxds_delay);

    if (config->spi_mode == OSPI_SPI_MODE_DUAL_OCTAL) {
        is_dual_octal = 1;
    }

    /* If the user has provided a function pointer to initialize the hyperram, call it */
    if (config->hyperram_init) {
        ospi_control_ss(ospi, config->slave_select, SPI_SS_STATE_ENABLE);
        config->hyperram_init(ospi, config->wait_cycles);
        ospi_control_ss(ospi, config->slave_select, SPI_SS_STATE_DISABLE);
    }

    ospi_set_dfs(ospi, config->dfs);

    /* Initialize OSPI hyperbus xip configuration */
    ospi_hyperbus_xip_init(ospi, config->wait_cycles, is_dual_octal);

#if SOC_FEAT_OSPI_HAS_XIP_SER
    ospi_control_xip_ss(ospi, config->slave_select, SPI_SS_STATE_ENABLE);
#endif

    aes_enable_xip(aes);

    return 0;
}
