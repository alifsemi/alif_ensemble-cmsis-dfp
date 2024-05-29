/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     we310f5_SPI.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     03-Feb-2024
 * @brief    Communication channel as SPI related functions for WE310F5 device.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef _WE310F5_SPI_H
#define _WE310F5_SPI_H

#include <stdint.h>
#include "we310f5.h"


/* Function Declarations */
/**
  \fn     int32_t we310f5_spi_wait_cond(WIFI_RESOURCES *wifi, uint32_t waitCond)
  \brief         Wait for SPI transfer to complete.
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \param[in]     waitCond   Value (SPI busy i.e 1 or idle i.e 0) on which
                            wait should over
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
  \note:  'timeoutMSretryCnt' value should be updated in \ref WIFI_RESOURCES
           wi-fi resource as per need, if 0, it waits forever.
*/
int32_t we310f5_spi_wait_cond(WIFI_RESOURCES *wifi, uint32_t waitCond);

/**
  \fn            int32_t we310f5_spi_transfer(WIFI_RESOURCES *wifi, uint8_t *in,
                              uint8_t *out, uint32_t len, uint32_t cmd)
  \brief         It will do SPI transfer and waits to complete the transfer
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \param[in]     in         Address of input buffer
  \param[in]     out        Address of output buffer
  \param[in]     len        Number of bytes to be transferred
  \param[in]     cmd        Wait for device read/write
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_spi_transfer(WIFI_RESOURCES *wifi, uint8_t *in, uint8_t *out, \
        uint32_t len, uint32_t cmd);


/**
  \fn            int32_t we310f5_spi_init(void)
  \brief         Initialization of SPI

  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_spi_init(void);

#endif /*  _WE310F5_SPI_H   */
