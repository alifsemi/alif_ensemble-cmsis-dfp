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
 * @file     : demo_lpspi.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 29-May-2023
 * @brief    : baremetal demo application for LPSPI and SPI0.
 *             - Data transfer between LPSPI(master) and SPI0(slave).
 * @bug      : None.
 * @Note     : Demo application will work only on M55-HE core.
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

#include "app_utils.h"

// Set to 0: Use application-defined lpspi and spi pin configuration (via
// board_lpspi_pins_config()). Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#if !defined(RTSS_HE)
#error "This Demo application works only on RTSS_HE"
#endif

/* Use below macro to specify transfer type
 * 1 - Uses SPI Transfer function
 * 0 - Uses SPI Send & Receive function
 * */
#define DATA_TRANSFER_TYPE 1

#define SELECTED_LPSPI     LP /* LPSPI instance */
#define SELECTED_SPI       0  /* SPI0 instance */

volatile uint8_t lpspi_cb_status;
volatile uint8_t spi0_cb_status ;

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SELECTED_LPSPI);
ARM_DRIVER_SPI       *ptrLPSPI = &ARM_Driver_SPI_(SELECTED_LPSPI);

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SELECTED_SPI);
ARM_DRIVER_SPI       *ptrSPI0 = &ARM_Driver_SPI_(SELECTED_SPI);

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_lpspi_pins_config(void)
 * @brief   Configure additional lpspi pinmux settings not handled
 *          by the board support library.
 * @retval  execution status.
 */
static int32_t board_lpspi_pins_config(void)
{
    int32_t ret;

    /* pinmux configurations for LPSPI pins */
    ret = pinconf_set(PORT_(BOARD_LPSPI_MISO_GPIO_PORT),
                      BOARD_LPSPI_MISO_GPIO_PIN,
                      BOARD_LPSPI_MISO_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPSPI_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_LPSPI_MOSI_GPIO_PORT),
                      BOARD_LPSPI_MOSI_GPIO_PIN,
                      BOARD_LPSPI_MOSI_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPSPI_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_LPSPI_SCLK_GPIO_PORT),
                      BOARD_LPSPI_SCLK_GPIO_PIN,
                      BOARD_LPSPI_SCLK_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPSPI_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_LPSPI_SS_GPIO_PORT),
                      BOARD_LPSPI_SS_GPIO_PIN,
                      BOARD_LPSPI_SS_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPSPI_SS_PIN\n");
        return ret;
    }

    /* pinmux configurations for SPI0 pins */
    ret = pinconf_set(PORT_(BOARD_SPI0_MISO_GPIO_PORT),
                      BOARD_SPI0_MISO_GPIO_PIN,
                      BOARD_SPI0_MISO_ALTERNATE_FUNCTION,
                      PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI0_MOSI_GPIO_PORT),
                      BOARD_SPI0_MOSI_GPIO_PIN,
                      BOARD_SPI0_MOSI_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI0_SCLK_GPIO_PORT),
                      BOARD_SPI0_SCLK_GPIO_PIN,
                      BOARD_SPI0_SCLK_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_SPI0_SS0_GPIO_PORT),
                      BOARD_SPI0_SS0_GPIO_PIN,
                      BOARD_SPI0_SS0_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI0_SS_PIN\n");
        return ret;
    }
    return ret;
}
#endif

/**
 * @fn      void LPSPI_cb_func (uint32_t event)
 * @brief   LPSPI callback function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  none.
 */
static void LPSPI_cb_func(uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE) {
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
static void SPI0_cb_func(uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE) {
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
    int32_t  ret = ARM_DRIVER_OK;
    uint32_t lpspi_control, spi0_control;
#if DATA_TRANSFER_TYPE
    uint32_t spi0_tx_buff, lpspi_rx_buff = 0;
#endif

    /*
     * H/W connections on devkit:
     * short LPSPI MISO (P7_4 -> J12-27 pin) and SPI0 MISO (P5_0 -> J12-13 pin).
     * short LPSPI MOSI (P7_5 -> J15-9 pin) and SPI0 MOSI (P5_1 -> J12-15 pin).
     * short LPSPI SCLK (P7_6 -> J15-8 pin) and SPI0 SCLK (P5_3 -> J14-5 pin).
     * short LPSPI SS (P7_7 -> J15-10 pin) and SPI0 SS (P5_2 -> J12-17 pin).
     * */

    printf("*** Demo app using SPI0 & LPSPI is starting ***\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The lpspi_spi pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_lpspi_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#endif

    /* config flexio pins to 1.8V */
    uint32_t      error_code = SERVICES_REQ_SUCCESS;
    uint32_t      service_error_code;
    run_profile_t runp;

    /* Initialize the SE services */
    se_services_port_init();

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("Get Current run config failed\n");
        WAIT_FOREVER_LOOP
    }

    runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
    /* Set the new run configuration */
    error_code          = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("Set new run config failed\n");
        WAIT_FOREVER_LOOP
    }

    /* LPSPI Configuration as master */
    ret = ptrLPSPI->Initialize(LPSPI_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize the LPSPI\n");
        return;
    }

    ret = ptrLPSPI->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power LPSPI\n");
        goto error_lpspi_uninitialize;
    }

    lpspi_control = (ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_CPOL0_CPHA0 |
                     ARM_SPI_DATA_BITS(32));

    /* Baudrate is 1MHz */
    ret           = ptrLPSPI->Control(lpspi_control, 1000000);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure LPSPI\n");
        goto error_lpspi_power_off;
    }

    /* SPI0 Configuration as slave */
    ret = ptrSPI0->Initialize(SPI0_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize the SPI0\n");
        goto error_lpspi_power_off;
    }

    ret = ptrSPI0->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power SPI0\n");
        goto error_spi0_uninitialize;
    }

    spi0_control = (ARM_SPI_MODE_SLAVE | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_DATA_BITS(32));

    ret          = ptrSPI0->Control(spi0_control, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure SPI0\n");
        goto error_spi0_power_off;
    }

    ret = ptrLPSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to enable the slave select of LPSPI\n");
        goto error_spi0_power_off;
    }

#if DATA_TRANSFER_TYPE
    lpspi_tx_buff = 0xAAAAAAAA;
    spi0_tx_buff  = 0x55555555;

    ret           = ptrSPI0->Transfer(&spi0_tx_buff, &spi0_rx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed SPI0 to configure as tx_rx\n");
        goto error_spi0_power_off;
    }

    ret = ptrLPSPI->Transfer(&lpspi_tx_buff, &lpspi_rx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: LPSPI Failed to configure as tx_rx\n");
        goto error_spi0_power_off;
    }

#else
    lpspi_tx_buff = 0x12345678;

    ret           = ptrSPI0->Receive(&spi0_rx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 Failed to configure as receive only\n");
        goto error_spi0_power_off;
    }

    ret = ptrLPSPI->Send(&lpspi_tx_buff, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: LPSPI Failed to configure as send only\n");
        goto error_spi0_power_off;
    }
#endif

    while (1) {
        if (lpspi_cb_status && spi0_cb_status) {
            lpspi_cb_status = 0;
            spi0_cb_status  = 0;
            break;
        }
    }

    while (!((ptrLPSPI->GetStatus().busy == 0) && (ptrSPI0->GetStatus().busy == 0))) {
    }
    printf("Data Transfer completed\n");

    printf("SPI0 received value 0x%" PRIx32 "\n", spi0_rx_buff);
#if DATA_TRANSFER_TYPE
    printf("LPSPI received value 0x%" PRIx32 "\n", lpspi_rx_buff);
#endif

error_spi0_power_off:
    ret = ptrSPI0->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("Failed to Power off SPI0\n");
    }

error_spi0_uninitialize:
    ret = ptrSPI0->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("Failed to Un-Initialized SPI0\n");
    }

error_lpspi_power_off:
    ret = ptrLPSPI->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("Failed to Power off LPSPI\n");
    }

error_lpspi_uninitialize:
    ret = ptrLPSPI->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("Failed to Un-Initialized LPSPI\n");
    }

    printf("*** Demo app using SPI0 & LPSPI is ended ***\n");
}

int main()
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif
    lpspi_spi0_transfer();
}
