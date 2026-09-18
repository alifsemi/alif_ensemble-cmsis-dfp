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
 * @file     ospi_psram_xip.h
 * @author   Silesh C V, Manoj A Murudi
 * @email    silesh@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     19-Jul-2023
 * @brief    Public header file for OSPI PSRAM XIP init library.
 ******************************************************************************/

#ifndef OSPI_PSRAM_XIP_H
#define OSPI_PSRAM_XIP_H

#include <stdint.h>

#include "soc.h"
#include "soc_features.h"
#include "sys_ctrl_ospi.h"
#include "sys_ctrl_aes.h"
#include "ospi_delay.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * enum OSPI_SPI_FRF.
 * OSPI spi frame formats.
 */
typedef enum _OSPI_SPI_FRF {
    OSPI_SPI_FRF_STANDARD,
    OSPI_SPI_FRF_DUAL,
    OSPI_SPI_FRF_QUAD,
    OSPI_SPI_FRF_OCTAL,
    OSPI_SPI_FRF_DUAL_OCTAL
} OSPI_SPI_FRF;

/**
 * enum RAM_TYPE.
 * RAM type.
 */
typedef enum _RAM_TYPE {
    RAM_TYPE_PSRAM,
    RAM_TYPE_HYPERRAM
} RAM_TYPE;

/**
 * enum CONFIG_MODE.
 * Configuration mode.
 */
typedef enum _CONFIG_MODE {
    CONFIG_MODE_RTE = 0,
    CONFIG_MODE_USER_PARAMETERS
} CONFIG_MODE;

typedef struct _ospi_psram_xip_config {
    /**< The OSPI instance to be setup in ram XIP mode */
    OSPI_INSTANCE instance;

    /**< OSPI bus speed */
    uint32_t bus_speed;

    /**< Optional device specific initialization needed by the ram device  */
    int32_t (*ram_init)(OSPI_Type *ospi, AES_Type *aes);

    /**< Drive edge configuration for the OSPI */
    uint8_t ddr_drive_edge;

    /**< Delay applied to the OSPI RXDS signal */
    uint8_t rxds_delay;

    /**< Wait cycles needed by the ram device */
    uint8_t wait_cycles;

    /**< Slave select (Chip select) line used for the ram device */
    uint8_t slave_select;

    /**< Data Frame Size used for the ram device */
    uint8_t dfs;

    /**< OSPI frame format for the ram device */
    OSPI_SPI_FRF spi_frf;

    /**< RAM type */
    RAM_TYPE ram_type;

    /**< Use config parameters as passed or overwrite from build time RTE config */
    CONFIG_MODE config_mode;

    bool rxds_sig_en;
#if SOC_FEAT_AES_OSPI_SIGNALS_DELAY
    const ospi_delay_cfg_t *signal_delay;
#endif

} ospi_psram_xip_config;


int ospi_psram_xip_init(ospi_psram_xip_config *config);

#ifdef __cplusplus
}
#endif
#endif /* OSPI_PSRAM_XIP_H */
