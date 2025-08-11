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
 * @file     : demo_sdio.c
 * @author   : Deepak Kumar
 * @email    : deepak@alifsemi.com
 * @version  : V0.0.1
 * @date     : 28-Nov-2022
 * @brief    : Baremeetal sdio driver test Application.
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/
/* System Includes */
#include "RTE_Device.h"
#include <stdio.h>
#include <inttypes.h>
#include "se_services_port.h"

/* include for SD Driver */
#include "sdio.h"
#include "sd.h"

/* include for Pin Mux config */
#include "pinconf.h"
#include "board_config.h"

#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#include "Driver_Common.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */
#include "Driver_IO.h"
#include "board_config.h"
#include "app_utils.h"

// Set to 0: Use application-defined SDC A revision pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#define BAREMETAL_SD_TEST_RAW_SECTOR                                                               \
    0x2000  // start reading and writing raw data from partition sector
volatile unsigned char sdbuffer[512 * 4] __attribute__((section("sd_dma_buf")))
__attribute__((aligned(32)));

const diskio_t   *p_SD_Driver  = &SD_Driver;
volatile uint32_t dma_done_irq = 0;

/**
  \fn           sd_cb(uint16_t cmd_status, uint16_t xfer_status)
  \brief        SD interrupt callback
  \param[in]    uint16_t cmd_status
  \param[in]    uint16_t xfer_status
  \return       none
*/
void sd_cb(uint16_t cmd_status, uint16_t xfer_status)
{
    /* dummy callback definition to resolve linking error */
    /* data transfer in SDIO mode needs Stack (wifi/bluetooth) */
    ARG_UNUSED(cmd_status);
    ARG_UNUSED(xfer_status);
}

#ifdef BOARD_SD_RESET_GPIO_PORT
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

/**
  \fn           sd_reset_cb(void)
  \brief        Perform SD reset sequence
  \return       none
  */
void sd_reset_cb(void)
{
    int              status;
    ARM_DRIVER_GPIO *gpioSD_RST = &ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

    status = gpioSD_RST->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (status != ARM_DRIVER_OK) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to toggle sd reset pin\n");
#endif
    }
    sys_busy_loop_us(100);
    status = gpioSD_RST->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (status != ARM_DRIVER_OK) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to toggle sd reset pin\n");
#endif
    }

    return;
}
#endif

/**
  \fn           BareMetalSDTest(uint32_t startSec, uint32_t EndSector)
  \brief        Baremetal SD driver Test Function
  \param[in]    starSecr - Test Read/Write start sector number
  \param[in]    EndSector - Test Read/Write End sector number
  \return       none
*/
void BareMetalSDIOTest(void)
{

    sd_param_t sd_param;

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    int32_t ret;
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
#ifdef BOARD_SD_RESET_GPIO_PORT
    uint32_t status;

    pinconf_set(PORT_(BOARD_SD_RESET_GPIO_PORT), BOARD_SD_RESET_GPIO_PIN, 0, 0);  // SD reset

    ARM_DRIVER_GPIO *sd_rst_gpio = &ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

    status = sd_rst_gpio->Initialize(BOARD_SD_RESET_GPIO_PIN, NULL);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to initialize SD RST GPIO\n");
#endif
    }

    status = sd_rst_gpio->PowerControl(BOARD_SD_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to powered full\n");
#endif
    }

    status = sd_rst_gpio->SetDirection(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to configure\n");
#endif
    }
    status = sd_rst_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to toggle sd reset pin\n");
#endif
    }

#endif
    /*
     * NOTE: The SDC A revision pins used in this test application are not configured
     * in the board support library. Therefore, pins are configured manually here.
     */
    pinconf_set(PORT_(BOARD_SD_CMD_A_GPIO_PORT),
                BOARD_SD_CMD_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // cmd
    pinconf_set(PORT_(BOARD_SD_CLK_A_GPIO_PORT),
                BOARD_SD_CLK_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // clk
    pinconf_set(PORT_(BOARD_SD_D0_A_GPIO_PORT),
                BOARD_SD_D0_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_7,
                PADCTRL_READ_ENABLE);  // d0

#if RTE_SDC_BUS_WIDTH == SDMMC_4_BIT_MODE
    pinconf_set(PORT_(BOARD_SD_D1_A_GPIO_PORT),
                BOARD_SD_D1_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_7,
                PADCTRL_READ_ENABLE);  // d1
    pinconf_set(PORT_(BOARD_SD_D2_A_GPIO_PORT),
                BOARD_SD_D2_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_7,
                PADCTRL_READ_ENABLE);  // d2
    pinconf_set(PORT_(BOARD_SD_D3_A_GPIO_PORT),
                BOARD_SD_D3_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // d3
#endif

#if RTE_SDC_BUS_WIDTH == SDMMC_8_BIT_MODE
    pinconf_set(PORT_(BOARD_SD_D4_A_GPIO_PORT),
                BOARD_SD_D4_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // d4
    pinconf_set(PORT_(BOARD_SD_D5_A_GPIO_PORT),
                BOARD_SD_D5_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_5,
                PADCTRL_READ_ENABLE);  // d5
    pinconf_set(PORT_(BOARD_SD_D6_A_GPIO_PORT),
                BOARD_SD_D6_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_5,
                PADCTRL_READ_ENABLE);  // d6
    pinconf_set(PORT_(BOARD_SD_D7_A_GPIO_PORT),
                BOARD_SD_D7_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_5,
                PADCTRL_READ_ENABLE);  // d7
#endif
#endif

    sd_param.dev_id       = SDMMC_DEV_ID;
    sd_param.clock_id     = RTE_SDC_CLOCK_SELECT;
    sd_param.bus_width    = RTE_SDC_BUS_WIDTH;
    sd_param.dma_mode     = RTE_SDC_DMA_SELECT;
    sd_param.app_callback = sd_cb;

#ifdef BOARD_SD_RESET_GPIO_PORT
    sd_param.reset_cb     = sd_reset_cb;
#else
    sd_param.reset_cb     = 0;
#endif

    if (p_SD_Driver->disk_initialize(&sd_param)) {
        printf("SD initialization failed...\n");
        return;
    }

    for (int32_t i = 0x0; i < 0x1000; i++) {
        sdio_read_cia((uint8_t *) &sdbuffer[i], 0, i);  // cccr
        printf("0x%" PRIx32 ": 0x%" PRIx8 "\n", i, sdbuffer[i]);
    }
}

int main()
{
    uint32_t service_error_code;
    uint32_t error_code = SERVICES_REQ_SUCCESS;
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* Initialize the SE services */
    se_services_port_init();

    /* Enable SDMMC Clocks */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC 100MHz clock enable = %" PRIu32 "\n", error_code);
        return 0;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_20M,
                                              true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC 20MHz clock enable = %" PRIu32 "\n", error_code);
        return 0;
    }

    /* Enter the Baremetal demo Application.  */
    BareMetalSDIOTest();

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              false,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC 100MHz clock disable = %" PRIu32 "\n", error_code);
        return 0;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_20M,
                                              false,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC 20MHz clock disable = %" PRIu32 "\n", error_code);
        return 0;
    }

    return 0;
}
