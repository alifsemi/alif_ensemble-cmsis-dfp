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
 * @file     : demo_sd_fatfs.c
 * @author   : Deepak Kumar
 * @email    : deepak@alifsemi.com
 * @version  : V0.0.1
 * @date     : 02-Sep-2024
 * @brief    : Test App for baremetal generic fatfs.
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/
/* System Includes */
#include "RTE_Device.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "se_services_port.h"
#include "ff.h"
#include "diskio.h"

#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#include "Driver_Common.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/* include for Pin Mux config */
#include "pinconf.h"
#include "board_config.h"
#include "app_utils.h"

// Set to 0: Use application-defined SDC A revision pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#define MEDIA_NAME                     "/SD_DISK/"

#define TEST_FILE                      "/TestFile34.txt"
/* Define Test Requirement <Test File Name> */
// #define FILE_CREATE_TEST TEST_FILE
#define FILE_READ_TEST                 TEST_FILE
// #define FILE_WRITE_TEST TEST_FILE

#define K                              (1024)

/* Tasks Pool size, stack size, and pointers */
#define STACK_POOL_SIZE                (40 * K)
#define SD_STACK_SIZE                  (10 * K)
#define SD_BLK_SIZE                    512
#define NUM_BLK_TEST                   1
#define SD_TEST_ITTR_CNT               10

/* Buffer for FileX FF_Disk_t sector cache. This must be large enough for at least one sector ,
 * which are typically 512 bytes in size. */
unsigned char filebuffer[(SD_BLK_SIZE * NUM_BLK_TEST) + 1] __attribute__((section("sd_dma_buf")))
__attribute__((aligned(512)));
FATFS sd_card __attribute__((section("sd_dma_buf"))) __attribute__((aligned(512)));
FIL test_file;

/**
  \fn           mySD_Thread_entry(unsigned long int args)
  \brief        ThreadX and FileX integrated SD driver Test Function
  \param[in]    args - Thread argument
  \return       none
  */
void SD_Baremetal_fatfs_test()
{
    FRESULT fr;

#if defined(FILE_READ_TEST)
    UINT br;
#endif
#if defined(FILE_WRITE_TEST)
    UINT bw;
#endif

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    int32_t ret;
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The SDC A revision pins used in this test application are not configured
     * in the board support library. Therefore, pins are configured manually here.
     */

#ifdef BOARD_SD_RESET_GPIO_PORT

    pinconf_set(PORT_(BOARD_SD_RESET_GPIO_PORT), BOARD_SD_RESET_GPIO_PIN, 0, 0); //SD reset

#endif

    pinconf_set(PORT_(BOARD_SD_CMD_A_GPIO_PORT),
                BOARD_SD_CMD_A_GPIO_PIN,
                BOARD_SD_CMD_ALTERNATE_FUNCTION,
                PADCTRL_READ_ENABLE | PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA);  // cmd
    pinconf_set(PORT_(BOARD_SD_CLK_A_GPIO_PORT),
                BOARD_SD_CLK_A_GPIO_PIN,
                BOARD_SD_CLK_ALTERNATE_FUNCTION,
                PADCTRL_READ_ENABLE | PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA);  // clk
    pinconf_set(PORT_(BOARD_SD_D0_A_GPIO_PORT),
                BOARD_SD_D0_A_GPIO_PIN,
                BOARD_SD_D0_ALTERNATE_FUNCTION,
                PADCTRL_READ_ENABLE | PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA);  // d0
#if RTE_SDC_BUS_WIDTH == SDMMC_4_BIT_MODE
    pinconf_set(PORT_(BOARD_SD_D1_A_GPIO_PORT),
                BOARD_SD_D1_A_GPIO_PIN,
                BOARD_SD_D1_ALTERNATE_FUNCTION,
                PADCTRL_READ_ENABLE | PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA);  // d1
    pinconf_set(PORT_(BOARD_SD_D2_A_GPIO_PORT),
                BOARD_SD_D2_A_GPIO_PIN,
                BOARD_SD_D2_ALTERNATE_FUNCTION,
                PADCTRL_READ_ENABLE | PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA);  // d2
    pinconf_set(PORT_(BOARD_SD_D3_A_GPIO_PORT),
                BOARD_SD_D3_A_GPIO_PIN,
                BOARD_SD_D3_ALTERNATE_FUNCTION,
                PADCTRL_READ_ENABLE | PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA);  // d3
#endif
#endif

    /* Open the SD disk. and initialize SD controller */
    fr = f_mount(&sd_card, MEDIA_NAME, 1);

    /* Check the media open status.  */
    if (fr) {
        printf("media open fail status = %" PRId16 "...\n", fr);
        WAIT_FOREVER_LOOP
    }
    printf("SD Mounted Successfully...\n");

#ifdef FILE_CREATE_TEST

    /* Create a file in the root directory.  */
    fr = f_open(&test_file, FILE_CREATE_TEST, FA_CREATE_ALWAYS | FA_WRITE);

    if (fr) {
        printf("File open status: %" PRId32 "\n", fr);
        /* Error opening file, break the loop.  */
        WAIT_FOREVER_LOOP
    }

#elif defined(FILE_READ_TEST)

    /* Open the test file.  */
    fr = f_open(&test_file, FILE_READ_TEST, FA_READ);

    /* Check the file open status.  */
    if (fr) {
        printf("File open status: %" PRId16 "\n", fr);
        /* Error opening file, break the loop.  */
        WAIT_FOREVER_LOOP
    }

    printf("Reading Data from File...%s\n", FILE_READ_TEST);
    memset(filebuffer, '\0', sizeof(filebuffer));

    while (1) {

        fr = f_read(&test_file, filebuffer, (SD_BLK_SIZE * NUM_BLK_TEST), &br);

        /* Check the file read status.  */
        if (fr) {
            /* Error performing file read, break the loop.  */
            printf("File read status: %" PRId16 "\n", fr);
            break;
        }

        if (f_eof(&test_file)) {
            printf("End of File Reached...\n");
            break;
        }

        printf("size = %" PRIu16 "\n %s\n", br, (const char *) filebuffer);
    }

#elif defined(FILE_WRITE_TEST)

    /* Open the test file.  */
    fr = f_open(&test_file, FILE_WRITE_TEST, FA_CREATE_ALWAYS | FA_WRITE);

    /* Check the file open status.  */
    if (fr) {
        printf("File open status: %" PRIi16 "\n", fr);
        /* Error opening file, break the loop.  */
        WAIT_FOREVER_LOOP
    }

    printf("Writing Data in File...%s\n", FILE_WRITE_TEST);
    memset(filebuffer, '\0', sizeof(filebuffer));

    for (int32_t i = 0; i < SD_TEST_ITTR_CNT; i++) {

        memset(filebuffer, 'F', (SD_BLK_SIZE * NUM_BLK_TEST));

        /* Write a string to the test file.  */
        fr = f_write(&test_file, (void *) filebuffer, (SD_BLK_SIZE * NUM_BLK_TEST), &bw);

        /* Check the file write status.  */
        if (fr) {
            printf("ittr: %" PRId32 " File write status: %" PRIi16 "\n", i, fr);
            break;
        }
    }
#else
#error "No Test Defined...\n"

#endif

    printf("Closing File...%s\n", TEST_FILE);

    /* Close the test file.  */
    fr = f_close(&test_file);

    /* Check the file close status.  */
    if (fr) {
        printf("File close status: %" PRId16 "\n", fr);
        /* Error closing the file, break the loop.  */
        WAIT_FOREVER_LOOP
    }

    printf("File R/W Test Completed!!!\n");
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

    SD_Baremetal_fatfs_test();

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
