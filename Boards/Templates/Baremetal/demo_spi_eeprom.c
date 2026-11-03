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
 * @file     : demo_spi_eeprom.c
 * @author   : Ganesh Ramani
 * @email    : Ganesh.Ramani@alifsemi.com
 * @version  : V1.0.0
 * @date     : 02-Nov-2026
 * @brief    : baremetal demo application EEPROM READ
 *              Read an external SPI EEPROM through SPI0 using hardware TMOD = 3
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include "Driver_SPI.h"
#include "spi.h"
#include "pinconf.h"
#include "RTE_Components.h"
#include "board_config.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif

/* Set to 1 to use board_pins_config() with Conductor-generated pins. */
#ifndef USE_CONDUCTOR_TOOL_PINS_CONFIG
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0
#endif

#define EEPROM_SPI_INSTANCE  0
#define EEPROM_READ_OPCODE   0x03U
#define EEPROM_READ_ADDRESS  0x0000U
#define EEPROM_READ_LENGTH   32U
#define EEPROM_SPI_BAUDRATE  1000000U
#define EEPROM_SPI_MODE      ARM_SPI_CPOL0_CPHA0

#if (EEPROM_READ_ADDRESS > 0xFFFFU)
#error "This demo uses a two-byte EEPROM address"
#endif

extern ARM_DRIVER_SPI ARM_Driver_SPI_(EEPROM_SPI_INSTANCE);
static ARM_DRIVER_SPI *const ptrSPI0 = &ARM_Driver_SPI_(EEPROM_SPI_INSTANCE);

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
static int32_t board_spi_eeprom_pins_config(void)
{
    int32_t ret = ARM_DRIVER_OK;

    /* pinmux configurations for SPI0 pins */
    ret  = pinconf_set(PORT_(BOARD_SPI0_MISO_GPIO_PORT),
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

    return ret;
}
#endif

static void spi_eeprom_demo(void)
{
    int32_t ret;
    uint8_t command[] = {
        EEPROM_READ_OPCODE,
        (uint8_t) (EEPROM_READ_ADDRESS >> 8),
        (uint8_t) EEPROM_READ_ADDRESS
    };
    uint8_t data[EEPROM_READ_LENGTH];
    spi_transfer_t transfer = {
        .tx_buff      = command,
        .rx_buff      = data,
        .tx_total_cnt = sizeof(command),
        .rx_total_cnt = sizeof(data),
        .frame_size   = 8U,
        .is_master    = true
    };

    printf("*** SPI0 EEPROM read demo ***\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    ret = board_pins_config();
#else
    ret = board_spi_eeprom_pins_config();
#endif
    if (ret != 0) {
        printf("ERROR: SPI0 pin configuration failed: %" PRId32 "\n", ret);
        return;
    }

    /* CMSIS setup: Driver_SPI0 -> ARM_SPI0_* -> ARM_SPI_* -> spi.c. */
    ret = ptrSPI0->Initialize(NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 initialization failed: %" PRId32 "\n", ret);
        return;
    }

    ret = ptrSPI0->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 power-on failed: %" PRId32 "\n", ret);
        goto uninitialize;
    }

    ret = ptrSPI0->Control(ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT |
                          EEPROM_SPI_MODE | ARM_SPI_DATA_BITS(8), EEPROM_SPI_BAUDRATE);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 configuration failed: %" PRId32 "\n", ret);
        goto power_off;
    }

    ret = ptrSPI0->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 slave selection failed: %" PRId32 "\n", ret);
        goto power_off;
    }

    /*
     * Direct low-level path:
     * spi_eeprom_read_blocking -> TMOD=3, NDF=31,
     * preload 3 command frames -> select slave -> poll RX FIFO.
     * Hardware ignores RX during the command, then clocks in 32 data frames.
     * No dummy TX frames, CMSIS callback, or GetStatus() wait is needed.
     * Blocking completion includes waiting for the hardware BUSY bit to clear.
     */
    ret = spi_eeprom_read_blocking((SPI_Type *) SPI0_BASE, &transfer);
    if (ret != 0) {
        printf("ERROR: EEPROM read failed: %" PRId32 ", transfer status: %u\n",
               ret, (unsigned int) transfer.status);
    } else {
        printf("EEPROM data at 0x%04X (%" PRIu32 " bytes):\n",
               (unsigned int) EEPROM_READ_ADDRESS, transfer.rx_current_cnt);
        for (uint32_t i = 0U; i < sizeof(data); i++) {
            printf("%02X%s", (unsigned int) data[i],
                   (((i + 1U) % 16U) == 0U || (i + 1U) == sizeof(data)) ? "\n" : " ");
        }
    }

    /* The helper has completed or stopped the controller before returning. */
    ret = ptrSPI0->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 slave deselection failed: %" PRId32 "\n", ret);
    }

power_off:
    ret = ptrSPI0->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 power-off failed: %" PRId32 "\n", ret);
    }

uninitialize:
    ret = ptrSPI0->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SPI0 uninitialization failed: %" PRId32 "\n", ret);
    }
}

int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    if (stdout_init() != ARM_DRIVER_OK) {
        while (1) {
        }
    }
#endif
    spi_eeprom_demo();
    return 0;
}
