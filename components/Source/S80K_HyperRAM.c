/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     S80K_HyperRAM.c
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     22-May-2025
 * @brief    S80K Hyperram init library.
 ******************************************************************************/

#include "S80K_HyperRAM.h"

/**
  \fn          void s80k_hyperram_init(OSPI_Type *ospi, uint8_t wait_cycles)
  \brief       S80K HyperRAM init function.
  \param[in]   ospi : Pointer to the OSPI register map.
  \param[in]   wait_cycles  : hyperram wait cycle value.
  \return      none
 */
void s80k_hyperram_init(OSPI_Type *ospi, uint8_t wait_cycles)
{
    ospi_transfer_t ospi_config;
    uint32_t        buff[3];

    /* changing wrap size of S80K RAM for RTSS-HP and RTSS-HE as needs it to be 32 bytes
     * and also reducing wait cycles to increase performance */

    /*
     * CA bit assignment for Configuration Register 0 write operation
     * bit[47] - bit[40] -> 60h
     * bit[39] - bit[32] -> 00h
     * bit[31] - bit[24] -> 01h
     * bit[23] - bit[16] -> 00h
     * bit[15] - bit[8]  -> 00h
     * bit[7]  - bit[0]  -> 00h
     */
    buff[0]                    = 0x60000100;                         /* bit[8] - bit[47] */
    buff[1]                    = 0x0;                                /* bit[0] - bit[7] */
    buff[2]                    = ((1 << S80K_RAM_OPERATION_MODE_POS) /* Normal operation */
               | (0 << S80K_RAM_DRIVE_STRENGTH_POS) /* 34 ohm output drive strength (default) */
               | (((wait_cycles - 5) & 0xF) << S80K_RAM_INIT_LATENCY_POS) /* initial latency */
               | (0 << S80K_RAM_FIXED_LATENCY_EN_POS) /* variable latency */
               | (1 << S80K_RAM_WRAPPED_BURST_SEQ_POS) /* standard wrapped burst sequence */
               | (2 << S80K_RAM_BURST_LEN_POS)); /* 16-word (32-byte) wrap */

    ospi_config.spi_frf        = SPI_FRF_OCTAL;
    ospi_config.ddr            = 1;
    ospi_config.inst_len       = SPI_CTRLR0_INST_L_0bit;
    ospi_config.addr_len       = 12;
    ospi_config.dummy_cycle    = 0;
    ospi_config.tx_total_cnt   = 3;
    ospi_config.tx_current_cnt = 0;
    ospi_config.tx_buff        = buff;

    ospi_set_dfs(ospi, 16);
    ospi_hyperbus_send(ospi, &ospi_config);
}
