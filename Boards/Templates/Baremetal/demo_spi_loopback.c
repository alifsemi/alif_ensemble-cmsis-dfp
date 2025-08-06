/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     : demo_spi_loopback.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 29-May-2023
 * @brief    : baremetal demo application for SPI0 and SPI1.
 *             - Data transfer between SPI0(master) and SPI1(slave).
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include "Driver_SPI.h"
#include "pinconf.h"
#include "RTE_Components.h"
#include "board_config.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

// Set to 0: Use application-defined SPI pin configuration (via board_spi_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* Use below macro to specify transfer type
 * 1 - Uses SPI Transfer function
 * 0 - Uses SPI Send & Receive function
 * */
#define DATA_TRANSFER_TYPE             0

#define SPI1                           1 /* SPI1 instance */
#define SPI0                           0 /* SPI0 instance */

volatile uint8_t spi1_cb_status = 0;
volatile uint8_t spi0_cb_status = 0;

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI1);
ARM_DRIVER_SPI       *ptrSPI1 = &ARM_Driver_SPI_(SPI1);

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI0);
ARM_DRIVER_SPI       *ptrSPI0 = &ARM_Driver_SPI_(SPI0);

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_spi_pins_config(void)
 * @brief   Configure additional spi0 and spi1 pinmux settings not handled
 *          by the board support library.
 * @retval  execution status.
 */
static int32_t board_spi_pins_config(void)
{
    int32_t ret = ARM_DRIVER_OK;

    /* pinmux configurations for SPI0 pins */
    ret         = pinconf_set(PORT_(BOARD_SPI0_MISO_GPIO_PORT),
                      BOARD_SPI0_MISO_GPIO_PIN,
                      BOARD_SPI0_MISO_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI0_MOSI_GPIO_PORT),
                      BOARD_SPI0_MOSI_GPIO_PIN,
                      BOARD_SPI0_MOSI_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI0_SCLK_GPIO_PORT),
                      BOARD_SPI0_SCLK_GPIO_PIN,
                      BOARD_SPI0_SCLK_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI0_SS0_GPIO_PORT),
                      BOARD_SPI0_SS0_GPIO_PIN,
                      BOARD_SPI0_SS0_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_SS_PIN\n");
        return ret;
    }

    /* pinmux configurations for SPI1 pins */
    ret = pinconf_set(PORT_(BOARD_SPI1_MISO_GPIO_PORT),
                      BOARD_SPI1_MISO_GPIO_PIN,
                      BOARD_SPI1_MISO_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI1_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI1_MOSI_GPIO_PORT),
                      BOARD_SPI1_MOSI_GPIO_PIN,
                      BOARD_SPI1_MOSI_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI1_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI1_SCLK_GPIO_PORT),
                      BOARD_SPI1_SCLK_GPIO_PIN,
                      BOARD_SPI1_SCLK_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI1_SCLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI1_SS0_GPIO_PORT),
                      BOARD_SPI1_SS0_GPIO_PIN,
                      BOARD_SPI1_SS0_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI1_SS_PIN\n");
        return ret;
    }

    return ret;
}
#endif

/**
 * @fn      static void SPI0_cb_func (uint32_t event)
 * @brief   SPI0 callback function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  none.
 */
static void SPI0_cb_func(uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE) {
        spi0_cb_status = 1;
    }
}

/**
 * @fn      static void SPI1_cb_func (uint32_t event)
 * @brief   SPI1 callback function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  none.
 */
static void SPI1_cb_func(uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE) {
        spi1_cb_status = 1;
    }
}

/**
 * @fn      static void spi0_spi1_transfer(void)
 * @brief   demo application function for data transfer.
 * @note    none.
 * @param   none.
 * @retval  none.
 */
static void spi0_spi1_transfer(void)
{
    uint32_t spi0_tx_buff, spi1_rx_buff = 0;
    int32_t  ret = ARM_DRIVER_OK;
    uint32_t spi1_control, spi0_control;
#if DATA_TRANSFER_TYPE
    uint32_t spi1_tx_buff, spi0_rx_buff = 0;
#endif

    /*
     * H/W connections on devkit:
     * short SPI0 MISO (P5_0 -> J12-13 pin) and SPI1 MISO (P8_3 -> J14-15 pin).
     * short SPI0 MOSI (P5_1 -> J12-15 pin) and SPI1 MOSI (P8_4 -> J14-17 pin).
     * short SPI0 SCLK (P5_3 -> J14-5 pin) and SPI1 SCLK (P8_5 -> J14-19 pin).
     * short SPI0 SS (P5_2 -> J12-17 pin) and SPI1 SS (P6_4 -> J12-22 pin).
     * */

    printf("*** Demo app using SPI0 & SPI1 is starting ***\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The spi0 and spi1 pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_spi_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#endif

    /* SPI0 Configuration as master */
    ret = ptrSPI0->Initialize(SPI0_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize the SPI0\n");
        return;
    }

    ret = ptrSPI0->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power SPI0\n");
        goto error_spi0_uninitialize;
    }

    spi0_control = (ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_CPOL0_CPHA0 |
                    ARM_SPI_DATA_BITS(32));

    /* Baudrate is 1MHz */
    ret          = ptrSPI0->Control(spi0_control, 1000000);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure SPI0\n");
        goto error_spi0_power_off;
    }

    /* SPI1 Configuration as slave */
    ret = ptrSPI1->Initialize(SPI1_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize the SPI1\n");
        goto error_spi0_power_off;
    }

    ret = ptrSPI1->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power SPI1\n");
        goto error_spi1_uninitialize;
    }

    spi1_control = (ARM_SPI_MODE_SLAVE | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(32));

    ret          = ptrSPI1->Control(spi1_control, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure SPI1\n");
        goto error_spi1_power_off;
    }

    ret = ptrSPI0->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to enable the slave select of SPI0\n");
        goto error_spi1_power_off;
    }

#if DATA_TRANSFER_TYPE
    spi0_tx_buff = 0xAAAAAAAA;
    spi1_tx_buff = 0x55555555;

    ret          = ptrSPI1->Transfer(&spi1_tx_buff, &spi1_rx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed SPI1 to configure as tx_rx\n");
        goto error_spi1_power_off;
    }

    ret = ptrSPI0->Transfer(&spi0_tx_buff, &spi0_rx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed SPI0 to configure as tx_rx\n");
        goto error_spi1_power_off;
    }

#else
    spi0_tx_buff = 0x12345678;

    ret          = ptrSPI1->Receive(&spi1_rx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI1 Failed to configure as receive only\n");
        goto error_spi1_power_off;
    }

    ret = ptrSPI0->Send(&spi0_tx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 Failed to configure as send only\n");
        goto error_spi1_power_off;
    }
#endif

    while (1) {
        if (spi0_cb_status && spi1_cb_status) {
            spi0_cb_status = 0;
            spi1_cb_status = 0;
            break;
        }
    }

    while (!((ptrSPI0->GetStatus().busy == 0) && (ptrSPI1->GetStatus().busy == 0))) {
    }
    printf("Data Transfer completed\n");

    printf("SPI1 received value : 0x%" PRIx32 "\n", spi1_rx_buff);
#if DATA_TRANSFER_TYPE
    printf("SPI0 received value : 0x%" PRIx32 "\n", spi0_rx_buff);
#endif

error_spi1_power_off:
    ret = ptrSPI1->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR in SPI1 power off\n");
    }

error_spi1_uninitialize:
    ret = ptrSPI1->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR in SPI1 un-initialization\n");
    }

error_spi0_power_off:
    ret = ptrSPI0->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR in SPI0 power off\n");
    }

error_spi0_uninitialize:
    ret = ptrSPI0->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR in SPI0 un-initialization\n");
    }

    printf("*** Demo app using SPI0 & SPI1 is ended ***\n");
}

int main()
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        while (1) {
        }
    }
#endif
    spi0_spi1_transfer();
}
