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
 * @file     ospi_hyperram_xip.h
 * @author   Silesh C V, Manoj A Murudi
 * @email    silesh@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     19-Jul-2023
 * @brief    Public header file for OSPI hyperram XIP init library.
 ******************************************************************************/

#ifndef OSPI_HYPERRAM_XIP_H
#define OSPI_HYPERRAM_XIP_H

#include <stdint.h>

#include "soc.h"
#include "soc_features.h"
#include "sys_ctrl_ospi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * enum OSPI_SPI_MODE.
 * OSPI spi mode.
 */
typedef enum _OSPI_SPI_MODE {
    OSPI_SPI_MODE_OCTAL,
    OSPI_SPI_MODE_DUAL_OCTAL
} OSPI_SPI_MODE;

typedef struct _ospi_hyperram_xip_config {
    /**< The OSPI instance to be setup in hyperram XIP mode */
    OSPI_INSTANCE instance;

    /**< OSPI bus speed */
    uint32_t bus_speed;

    /**< Optional device specific initialization needed by the hyperram device  */
    void (*hyperram_init)(OSPI_Type *, uint8_t);

    /**< Drive edge configuration for the OSPI */
    uint8_t ddr_drive_edge;

    /**< Delay applied to the OSPI RXDS signal */
    uint8_t rxds_delay;

    /**< Wait cycles needed by the hyperram device */
    uint8_t wait_cycles;

    /**< Slave select (Chip select) line used for the hyperram device */
    uint8_t slave_select;

    /**< Data Frame Size used for the hyperram device */
    uint8_t dfs;

    /**< OSPI transfer type for the hyperram device */
    OSPI_SPI_MODE spi_mode;

#if SOC_FEAT_AES_OSPI_SIGNALS_DELAY
    /**< Delay applied to the OSPI baud2 signal delay */
    uint8_t signal_delay;
#endif
} ospi_hyperram_xip_config;

/**
  \fn          int ospi_hyperram_xip_init(const ospi_hyperram_xip_config *config)
  \brief       Initialize OSPI Hyerbus xip configuration. After a successful return
               from this function, the OSPI XIP region (for the OSPI instance specified
               in the ospi_hyerram_xip_config input parameter) will be active and can be
               used to directly read/write the memory area provided by the hyperram device.
  \param[in]   config    Pointer to hyperram configuration information
  \return      -1 on configuration error, 0 on success
*/
int ospi_hyperram_xip_init(const ospi_hyperram_xip_config *config);
#ifdef __cplusplus
}
#endif
#endif /* OSPI_HYPERRAM_XIP_H */
