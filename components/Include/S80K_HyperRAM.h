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
 * @file     S80K_HyperRAM.h
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     22-May-2025
 * @brief    Header file for S80K Hyperram init library.
 ******************************************************************************/

#ifndef S80KS_HYPERRAM_H_
#define S80KS_HYPERRAM_H_

#include "ospi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define S80K_RAM_BURST_LEN_POS         0
#define S80K_RAM_WRAPPED_BURST_SEQ_POS 2
#define S80K_RAM_FIXED_LATENCY_EN_POS  3
#define S80K_RAM_INIT_LATENCY_POS      4
#define S80K_RAM_DRIVE_STRENGTH_POS    12
#define S80K_RAM_OPERATION_MODE_POS    15

/**
  \fn          void s80k_hyperram_init(OSPI_Type *ospi, uint8_t wait_cycles)
  \brief       S80K HyperRAM init function.
  \param[in]   ospi : Pointer to the OSPI register map.
  \param[in]   wait_cycles  : hyperram wait cycle value.
  \return      none
 */
void s80k_hyperram_init(OSPI_Type *ospi, uint8_t wait_cycles);

#ifdef __cplusplus
}
#endif

#endif /* S80KS_HYPERRAM_H */
