/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     LPSPI_SPI_baremetal.c
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     19-July-2024
 * @brief    baremetal demo application for LPSPI and SPI0 on Balletto.
 *           - Data transfer between LPSPI(master) and SPI0(slave).
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
#include <stdio.h>

#include "Driver_SPI.h"
#include "pinconf.h"
#include "Driver_GPIO.h"

#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* Use below macro to specify transfer type
 * 1 - Uses SPI Transfer function
 * 0 - Uses SPI Send & Receive function
 * */
#define DATA_TRANSFER_TYPE  0

#define  LPSPI          LP   /* LPSPI instance */
#define  SPI0           0    /* SPI0 instance */
#define  GPIO1          1    /* GPIO 1 port */
#define  GPIO_PIN_3     3    /* GPIO pin 3 */

volatile uint8_t lpspi_cb_status = 0;
volatile uint8_t spi0_cb_status = 0;

extern ARM_DRIVER_SPI ARM_Driver_SPI_(LPSPI);
ARM_DRIVER_SPI *ptrLPSPI = &ARM_Driver_SPI_(LPSPI);

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI0);
ARM_DRIVER_SPI *ptrSPI0 = &ARM_Driver_SPI_(SPI0);

extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO1);
ARM_DRIVER_GPIO *gpioDrv = &ARM_Driver_GPIO_(GPIO1);

/**
 * @fn      static int32_t pinmux_config(void)
 * @brief   LPSPI & SPI0 pinmux configuration.
 * @note    none.
 * @param   none.
 * @retval  none.
 */
static int32_t pinmux_config(void)
{
    int32_t ret;

    /* pinmux configurations for LPSPI pins (using A version pins) */
    ret = pinconf_set(PORT_1, PIN_3, PINMUX_ALTERNATE_FUNCTION_4, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for LPSPI_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_1, PIN_4, PINMUX_ALTERNATE_FUNCTION_3, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for LPSPI_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_1, PIN_5, PINMUX_ALTERNATE_FUNCTION_3, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for LPSPI_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_2, PIN_0, PINMUX_ALTERNATE_FUNCTION_4, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for LPSPI_SS_PIN\n");
        return ret;
    }

    /* pinmux configurations for SPI0 pins (using B version pins) */
    ret = pinconf_set(PORT_5, PIN_0, PINMUX_ALTERNATE_FUNCTION_3, 0);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_5, PIN_1, PINMUX_ALTERNATE_FUNCTION_3, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_5, PIN_2, PINMUX_ALTERNATE_FUNCTION_4, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_5, PIN_3, PINMUX_ALTERNATE_FUNCTION_3, PADCTRL_READ_ENABLE);
    if (ret)
    {
        printf("ERROR: Failed to configure PINMUX for SPI0_SS_PIN\n");
        return ret;
    }

    return ret;
}

/**
 * @fn      void LPSPI_cb_func (uint32_t event)
 * @brief   LPSPI callback function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  none.
 */
static void LPSPI_cb_func (uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        lpspi_cb_status = 1;
    }
}

/**
 * @fn      void SPI0_cb_func (uint32_t event)
 * @brief   SPI0 callback function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  none.
 */
static void SPI0_cb_func (uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        spi0_cb_status = 1;
    }
}

/**
 * @fn      void lpspi_spi0_transfer(void)
 * @brief   demo application function for data transfer.
 * @note    none.
 * @param   none.
 * @retval  none.
 */
void lpspi_spi0_transfer(void)
{
    uint32_t lpspi_tx_buff, spi0_rx_buff = 0;
    int32_t ret = ARM_DRIVER_OK;
    uint32_t lpspi_control, spi0_control;
    uint32_t arg = ARM_GPIO_FLEXIO_VOLT_1V8;
#if DATA_TRANSFER_TYPE
    uint32_t spi0_tx_buff, lpspi_rx_buff = 0;
#endif

    printf("*** Demo app using SPI0 & LPSPI is starting ***\n");

    ret = pinmux_config();
    if (ret)
    {
        printf("Error in pinmux configuration\n");
        return;
    }

    /* LPSPI MISO, MOSI, CLK pins are connected to flexio pins.
     * config any of the LPSPI pins (flexio) to 1.8V.
     */
    ret = gpioDrv->Initialize(GPIO_PIN_3, NULL);
    if ((ret != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to power off \n");
    }
    ret = gpioDrv->PowerControl(GPIO_PIN_3, ARM_POWER_FULL);
    if ((ret != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to power off \n");
    }
    ret = gpioDrv->Control(GPIO_PIN_3, ARM_GPIO_CONFIG_FLEXIO, &arg);
    if ((ret != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to power off \n");
    }

    /* LPSPI Configuration as master */
    ret = ptrLPSPI->Initialize(LPSPI_cb_func);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize the LPSPI\n");
        return;
    }

    ret = ptrLPSPI->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to power LPSPI\n");
        goto error_lpspi_uninitialize;
    }

    lpspi_control = (ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(32));

    /* Baudrate is 1MHz */
    ret = ptrLPSPI->Control(lpspi_control, 1000000);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure LPSPI\n");
        goto error_lpspi_power_off;
    }

    /* SPI0 Configuration as slave */
    ret = ptrSPI0->Initialize(SPI0_cb_func);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize the SPI0\n");
        goto error_lpspi_power_off;
    }

    ret = ptrSPI0->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to power SPI0\n");
        goto error_spi0_uninitialize;
    }

    spi0_control = (ARM_SPI_MODE_SLAVE | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(32));

    ret = ptrSPI0->Control(spi0_control, NULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure SPI0\n");
        goto error_spi0_power_off;
    }

    ret = ptrLPSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to enable the slave select of LPSPI\n");
        goto error_spi0_power_off;
    }

#if DATA_TRANSFER_TYPE
    lpspi_tx_buff = 0xAAAAAAAA;
    spi0_tx_buff = 0x55555555;

    ret = ptrSPI0->Transfer(&spi0_tx_buff, &spi0_rx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed SPI0 to configure as tx_rx\n");
        goto error_spi0_power_off;
    }

    ret = ptrLPSPI->Transfer(&lpspi_tx_buff, &lpspi_rx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: LPSPI Failed to configure as tx_rx\n");
        goto error_spi0_power_off;
    }

#else
    lpspi_tx_buff = 0x12345678;

    ret = ptrSPI0->Receive(&spi0_rx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: SPI0 Failed to configure as receive only\n");
        goto error_spi0_power_off;
    }

    ret = ptrLPSPI->Send(&lpspi_tx_buff, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: LPSPI Failed to configure as send only\n");
        goto error_spi0_power_off;
    }
#endif

    while(1)
    {
        if (lpspi_cb_status && spi0_cb_status)
        {
            lpspi_cb_status = 0;
            spi0_cb_status = 0;
            break;
        }
    }

    while (!((ptrLPSPI->GetStatus().busy == 0) && (ptrSPI0->GetStatus().busy == 0)));
    printf("Data Transfer completed\n");

    printf("SPI0 received value 0x%x\n", spi0_rx_buff);
#if DATA_TRANSFER_TYPE
    printf("LPSPI received value 0x%x\n", lpspi_rx_buff);
#endif

error_spi0_power_off :
    ret = ptrSPI0->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("Failed to Power off SPI0\n");
    }

error_spi0_uninitialize :
    ret = ptrSPI0->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("Failed to Un-Initialized SPI0\n");
    }

error_lpspi_power_off :
    ret = ptrLPSPI->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("Failed to Power off LPSPI\n");
    }

error_lpspi_uninitialize :
    ret = ptrLPSPI->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("Failed to Un-Initialized LPSPI\n");
    }

    printf("*** Demo app using SPI0 & LPSPI is ended ***\n");
}

int main()
{
    #if defined(RTE_Compiler_IO_STDOUT_User)
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
    #endif
    lpspi_spi0_transfer();
}
