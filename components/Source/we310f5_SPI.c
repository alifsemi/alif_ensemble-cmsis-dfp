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
 * @file     we310f5_SPI.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     03-Feb-2024
 * @brief    Communication channel as SPI related function for WE310F5 device
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include <we310f5.h>
#include <we310f5_SPI.h>
#include <WiFi_utils.h>
#include <WiFi_we310f5_HW_Config.h>
#include "Driver_Common.h"
#include "Driver_SPI.h"
#include "Driver_GPIO.h"

extern ARM_DRIVER_SPI  ARM_Driver_SPI_(WIFI_SPI_INSTANCE);                      /* SPI_Driver               */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(WIFI_SPI_GPIO_PORT_NUM);                /* GPIO12 Driver            */

static ARM_DRIVER_SPI  *pSPIhandle   = &ARM_Driver_SPI_(WIFI_SPI_INSTANCE);         /* SPI Driver Instance      */
static ARM_DRIVER_GPIO *pChipSelLine = &ARM_Driver_GPIO_(WIFI_SPI_GPIO_PORT_NUM);   /* GPIO12 Driver instance   */
static volatile uint32_t spiTxferDone = 0;
/**
  \fn            void wifi_spi_irq_callback(uint32_t event)
  \brief         Callback when SPI transfer triggers
  \param[in]     event SPI event type
  \return        none
*/
static void we310f5_spi_irq_callback(uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        spiTxferDone =  1;
    }
}

/**
  \fn            void we310f5_spi_chip_select(WIFI_RESOURCES *wifi, uint8_t cspin)
  \brief         Control chip select line
  \param[in]     wifi       Pointer to \ref WIFI_RESOURCES wi-fi resource
  \param[in]     enable     Enable or Disable
  \return        none
*/
void we310f5_spi_chip_select(WIFI_RESOURCES *wifi, bool enable)
{
    if(!enable)
    {
        pChipSelLine->SetValue(WIFI_SPI_CS_GPIO_PIN_NUM, GPIO_PIN_OUTPUT_STATE_LOW);
    }
    else
    {
        pChipSelLine->SetValue(WIFI_SPI_CS_GPIO_PIN_NUM, GPIO_PIN_OUTPUT_STATE_HIGH);
    }

    return;
}

/**
  \fn            int32_t we310f5_spi_wait_cond(WIFI_RESOURCES *wifi, uint32_t waitCond)
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
int32_t we310f5_spi_wait_cond(WIFI_RESOURCES *wifi, uint32_t waitCond)
{
    int32_t ret = ARM_DRIVER_ERROR;
    uint32_t tMSretryCnt = wifi->timeoutMSretryCnt;

    if(tMSretryCnt)
    {
        do
        {
            delay_in_ms(1);

            if(pSPIhandle->GetStatus().busy == waitCond)
            {
                ret = ARM_DRIVER_OK;
                break;
            }
        }while(tMSretryCnt--);
    }
    else
    {
        while(pSPIhandle->GetStatus().busy != waitCond);
        ret = ARM_DRIVER_OK;
    }

    return ret;
}

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
        uint32_t len, uint32_t cmd)
{
    int32_t ret = ARM_DRIVER_ERROR;

    we310f5_reset_spi_packet(out, len);

    if(cmd != INVALID_CMD)
    {
        // As per diagram :Figure 11 Host write data sequence (SPI Interface)
        we310f5_wait_cond(wifi, cmd);
    }

    spiTxferDone = 0;
    ret =pSPIhandle->Transfer((void*)in, (void*)out, len);
    if (ret != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    ret = wait_cond((uint32_t *) &spiTxferDone, 1, wifi->timeoutMSretryCnt);
    if (ret != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR_TIMEOUT;
    }
    we310f5_spi_wait_cond(wifi, 0);
    if (ret != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR_TIMEOUT;
    }

    return ret;
}

/**
  \fn            int32_t we310f5_spi_init(void)
  \brief         Initialization of SPI

  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t we310f5_spi_init(void)
{
    int32_t ret = ARM_DRIVER_ERROR;
    uint32_t control_code = -1;

    //--------------------------------------------------------------------------
    ret = pSPIhandle->Initialize(we310f5_spi_irq_callback);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    ret = pSPIhandle->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    control_code = (ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_UNUSED  | \
                    ARM_SPI_CPOL1_CPHA0 | ARM_SPI_DATA_BITS(8));
    ret = pSPIhandle->Control(control_code, SPI_BAUD_RATE);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    //--------------------------------------------------------------------------

    // WIFI_SS:     P12_7
    //--------------------------------------------------------------------------
    ret = pChipSelLine->Initialize(  WIFI_SPI_CS_GPIO_PIN_NUM, NULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    ret = pChipSelLine->PowerControl(WIFI_SPI_CS_GPIO_PIN_NUM, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    ret = pChipSelLine->SetDirection(WIFI_SPI_CS_GPIO_PIN_NUM, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    return ARM_DRIVER_OK;
}
