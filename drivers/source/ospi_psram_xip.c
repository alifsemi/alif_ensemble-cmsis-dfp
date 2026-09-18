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
 * @file     ospi_psram_xip.c
 * @author   Silesh C V, Manoj A Murudi
 * @email    silesh@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     19-Jul-2023
 * @brief    Implementation of the OSPI PS-RAM XIP init library.
 ******************************************************************************/

#include <ospi_psram_xip.h>
#include <stdint.h>
#include <stddef.h>

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "ospi.h"
#include "soc.h"
#include "sys_clocks.h"
#include "sys_utils.h"

#include "sys_ctrl_aes.h"

/**
  \fn          int ospi_set_speed(OSPI_Type *ospi, AES_Type *aes, const ospi_psram_xip_config
  *config) \brief       Set OSPI bus speed for spi transfer. \param[in]   ospi : Pointer to the OSPI
  register map. \param[in]   aes  : Pointer to the AES register map. \param[in]   config  : Pointer
  to ram configuration information. \return      -1 on configuration error, 0 on success
 */
static int ospi_set_speed(OSPI_Type *ospi, AES_Type *aes, const ospi_psram_xip_config *config)
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
#else
        {
            ARG_UNUSED(aes);
            return -1;
        }
#endif
    }

#if SOC_FEAT_AES_OSPI_SIGNALS_DELAY
    // Configure OSPI signal delays if the values are calibrated for the chosen frequency
    if (config->signal_delay &&
        config->signal_delay->idx == config->instance &&
        config->signal_delay->sclk_freq == config->bus_speed) {
        aes_set_signal_delay(aes, config->signal_delay);
    } else {
        // Set zero signal delays and default RXDS for non-calibrated frequncy (backwards compatibility)
        ospi_delay_cfg_t signal_delay = { 0 };
        signal_delay.idx = config->instance;
        signal_delay.rxds[0] = config->rxds_delay;
        signal_delay.rxds[1] = config->rxds_delay;
        aes_set_signal_delay(aes, &signal_delay);
    }
#else
    aes_set_rxds_delay(aes, config->rxds_delay);
#endif

    ospi_set_baud(ospi, baud);

    return 0;
}

/**
  \fn          int ospi_psram_xip_init(ospi_psram_xip_config *config)
  \brief       Initialize OSPI PS-RAM xip configuration. After a successful return
               from this function, the OSPI XIP region (for the OSPI instance specified
               in the ospi_psram_xip_config input parameter) will be active and can be
               used to directly read/write the memory area provided by the ram device.
               Fills configuration parameters from RTE config if the config_mode is set to CONFIG_MODE_RTE,
               otherwise uses the parameters as passed in the config structure.
  \param[in]   config    Pointer to ram configuration information
  \return      -1 on configuration error, 0 on success
*/
int ospi_psram_xip_init(ospi_psram_xip_config *config)
{
    OSPI_Type *ospi          = NULL;
    AES_Type  *aes           = NULL;
    bool       is_dual_octal = 0;

    if (!config) {
        return -1;
    }

    if (config->config_mode != CONFIG_MODE_RTE &&
        config->config_mode != CONFIG_MODE_USER_PARAMETERS) {
        return -1;
    }

    if (config->instance == OSPI_INSTANCE_0) {
        ospi = (OSPI_Type *) OSPI0_BASE;
        aes  = (AES_Type *) AES0_BASE;

        if (config->config_mode == CONFIG_MODE_RTE) {
            config->spi_frf = RTE_OSPI0_SPI_FRAME_FORMAT;
            config->bus_speed = RTE_OSPI0_BUS_SPEED;
            config->ddr_drive_edge = RTE_OSPI0_DDR_DRIVE_EDGE;
            config->rxds_delay = RTE_OSPI0_RXDS_DELAY;
#if SOC_FEAT_AES_OSPI_SIGNALS_DELAY
            config->signal_delay = NULL;
#endif
            config->dfs = RTE_OSPI0_DFS;
            config->slave_select = RTE_OSPI0_CHIP_SELECTION_PIN;
            config->wait_cycles = RTE_OSPI0_WAIT_CYCLES;
        }
#ifdef RTE_OSPI1
    } else if (config->instance == OSPI_INSTANCE_1) {
        ospi = (OSPI_Type *) OSPI1_BASE;
        aes  = (AES_Type *) AES1_BASE;

        if (config->config_mode == CONFIG_MODE_RTE) {
            config->spi_frf = RTE_OSPI1_SPI_FRAME_FORMAT;
            config->bus_speed = RTE_OSPI1_BUS_SPEED;
            config->ddr_drive_edge = RTE_OSPI1_DDR_DRIVE_EDGE;
            config->rxds_delay = RTE_OSPI1_RXDS_DELAY;
#if SOC_FEAT_AES_OSPI_SIGNALS_DELAY
            config->signal_delay = NULL;
#endif
            config->dfs = RTE_OSPI1_DFS;
            config->slave_select = RTE_OSPI1_CHIP_SELECTION_PIN;
            config->wait_cycles = RTE_OSPI1_WAIT_CYCLES;
        }
#endif
    } else {
        return -1;
    }

#if SOC_FEAT_OSPI_HAS_CLK_ENABLE
    enable_ospi_clk(config->instance);
#endif

    ospi_set_ddr_drive_edge(ospi, config->ddr_drive_edge);

    if (ospi_set_speed(ospi, aes, config)) {
        return -1;
    }

    if (config->spi_frf == OSPI_SPI_FRF_DUAL_OCTAL) {
        is_dual_octal = 1;
    }

    /* If the user has provided a function pointer to initialize the ram, call it */
    if (config->ram_init) {
        ospi_control_ss(ospi, config->slave_select, SPI_SS_STATE_ENABLE);
        if (config->ram_init(ospi, aes)) {
            return -1;
        }
        ospi_control_ss(ospi, config->slave_select, SPI_SS_STATE_DISABLE);
    }

    ospi_set_dfs(ospi, config->dfs);

    if (config->ram_type == RAM_TYPE_HYPERRAM) {
        /* Initialize OSPI hyperbus xip configuration */
        ospi_hyperbus_xip_init(ospi, config->wait_cycles, is_dual_octal);
    } else if (config->ram_type == RAM_TYPE_PSRAM) {
        /* Initialize OSPI psram xip configuration */
        if (config->wait_cycles == 0) {
            return -1;
        }
        ospi_psram_xip_cfg(ospi, config->wait_cycles-1, is_dual_octal);
    } else {
        return -1;
    }

    /* hack to reduce data overhead cycle by 1 clock cycle */
    ospi_set_tx_threshold(ospi, 0);

#if SOC_FEAT_OSPI_HAS_XIP_SER
    ospi_control_xip_ss(ospi, config->slave_select, SPI_SS_STATE_ENABLE);
#endif

    aes_enable_xip(aes);

    return 0;
}
