/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/***************************************************************************************
 * @file     : demo_hyperram_e8.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 20-May-2025
 * @brief    : Demo program for the ISSI Hyperram device using XIP write/read.
 ***************************************************************************************/

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>

#include "ospi_hyperram_xip.h"
#include "board_config.h"
#include "Driver_IO.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /*RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

#define DDR_DRIVE_EDGE      1
#define RXDS_DELAY          17
#define SIGNAL_DELAY        22
#define OSPI_BUS_SPEED      100000000           /* 100MHz */
#define OSPI_DFS            32
#define OSPI_XIP_BASE       0xA0000000
#define HRAM_SIZE_BYTES     (64 * 1024 * 1024)  /* 64MB */
#define SLAVE_SELECT        0
#define WAIT_CYCLES         6

static const ospi_hyperram_xip_config issi_config = {
    .instance       = OSPI_INSTANCE_0,
    .bus_speed      = OSPI_BUS_SPEED,
    .hyperram_init  = NULL, /* No special initialization needed by issi hyperram device */
    .ddr_drive_edge = DDR_DRIVE_EDGE,
    .rxds_delay     = RXDS_DELAY,
    .signal_delay   = SIGNAL_DELAY,
    .wait_cycles    = WAIT_CYCLES,
    .dfs            = OSPI_DFS,
    .slave_select   = SLAVE_SELECT,
    .spi_mode       = OSPI_SPI_MODE_OCTAL};

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_IS66_HYPERRAM_RESET_GPIO_PORT);
ARM_DRIVER_GPIO       *GPIODrv = &ARM_Driver_GPIO_(BOARD_IS66_HYPERRAM_RESET_GPIO_PORT);

static int32_t hyperram_reset(void)
{
    int32_t ret;

    ret = GPIODrv->Initialize(BOARD_IS66_HYPERRAM_RESET_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        return -1;
    }

    ret = GPIODrv->PowerControl(BOARD_IS66_HYPERRAM_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return -1;
    }

    ret = GPIODrv->SetDirection(BOARD_IS66_HYPERRAM_RESET_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return -1;
    }

    ret = GPIODrv->SetValue(BOARD_IS66_HYPERRAM_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return -1;
    }

    ret = GPIODrv->SetValue(BOARD_IS66_HYPERRAM_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return -1;
    }

    return 0;
}

int main(void)
{
    uint32_t *const ptr          = (uint32_t *) OSPI_XIP_BASE;
    uint32_t        total_errors = 0, random_val;
    int32_t         ret;

#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    printf("ISSI HyperRAM demo app started\n");

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        goto error_exit;
    }

    ret = hyperram_reset();
    if (ret != 0) {
        printf("Error in reset sequence: %" PRId32 "\n", ret);
        goto error_exit;
    }

    if (ospi_hyperram_xip_init(&issi_config) < 0) {
        printf("Hyperram XIP init failed\n");
        goto error_exit;
    }

    printf("Writing data to the XIP region:\n");

    srand(1);
    for (uint32_t i = 0; i < (HRAM_SIZE_BYTES / sizeof(uint32_t)); i++) {
        ptr[i] = rand();
    }

    printf("Reading back:\n");

    srand(1);
    for (uint32_t i = 0; i < (HRAM_SIZE_BYTES / sizeof(uint32_t)); i++) {
        random_val = rand();
        if (ptr[i] != random_val) {
            printf("Data error at addr %" PRIx32 ", got %" PRIx32 ", expected %" PRIx32 "\n",
                   (i * sizeof(uint32_t)),
                   ptr[i],
                   random_val);
            total_errors++;
        }
    }

    printf("Done, total errors = %" PRIu32 "\n", total_errors);

error_exit:

    WAIT_FOREVER_LOOP
}
