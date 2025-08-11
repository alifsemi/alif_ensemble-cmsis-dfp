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
 * @file     : demo_spi_microwire.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 28-Jan-2024
 * @brief    : Baremetal demo app configures the BOARD_MW_SPI_MASTER_INSTANCE instance in master
 *             mode with Microwire frame format and configures the BOARD_MW_SPI_SLAVE_INSTANCE
 *instance in slave mode with Microwire frame format. The data transfer occurs between master and
 *slave or slave and master based on the MASTER_TO_SLAVE_TRANSFER macro.
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include "pinconf.h"
#include "Driver_SPI.h"
#include "RTE_Components.h"
#include "board_config.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

// Set to 0: Use application-defined SPI pin configuration (via board_spi_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* assign 1: To enable master to slave transfer
 *        0: To enable slave to master transfer */
#define MASTER_TO_SLAVE_TRANSFER       1

static volatile uint32_t cb_spi_master_status, cb_spi_slave_status;

extern ARM_DRIVER_SPI  ARM_Driver_SPI_(BOARD_MW_SPI_MASTER_INSTANCE);
static ARM_DRIVER_SPI *masterDrv = &ARM_Driver_SPI_(BOARD_MW_SPI_MASTER_INSTANCE);

extern ARM_DRIVER_SPI  ARM_Driver_SPI_(BOARD_MW_SPI_SLAVE_INSTANCE);
static ARM_DRIVER_SPI *slaveDrv = &ARM_Driver_SPI_(BOARD_MW_SPI_SLAVE_INSTANCE);

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_spi_pins_config(void)
 * @brief   Configure additional MW pinmux settings not handled
 *          by the board support library.
 * @retval  execution status.
 */
static int32_t board_spi_pins_config(void)
{
    int32_t ret = 0;

    /* pinmux configurations for SPI Master pins  */
    ret         = pinconf_set(PORT_(BOARD_MW_SPI_MASTER_MISO_GPIO_PORT),
                      BOARD_MW_SPI_MASTER_MISO_GPIO_PIN,
                      BOARD_MW_SPI_MASTER_MISO_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_MASTER_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_MW_SPI_MASTER_MOSI_GPIO_PORT),
                      BOARD_MW_SPI_MASTER_MOSI_GPIO_PIN,
                      BOARD_MW_SPI_MASTER_MOSI_ALTERNATE_FUNCTION,
                      0);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_MASTER_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_MW_SPI_MASTER_SCLK_GPIO_PORT),
                      BOARD_MW_SPI_MASTER_SCLK_GPIO_PIN,
                      BOARD_MW_SPI_MASTER_SCLK_ALTERNATE_FUNCTION,
                      0);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_MASTER_CLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_MW_SPI_MASTER_SS0_GPIO_PORT),
                      BOARD_MW_SPI_MASTER_SS0_GPIO_PIN,
                      BOARD_MW_SPI_MASTER_SS0_ALTERNATE_FUNCTION,
                      0);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_MASTER_SS_PIN\n");
        return ret;
    }

    /* pinmux configurations for SPI Slave pins  */
    ret = pinconf_set(PORT_(BOARD_MW_SPI_SLAVE_MISO_GPIO_PORT),
                      BOARD_MW_SPI_SLAVE_MISO_GPIO_PIN,
                      BOARD_MW_SPI_SLAVE_MISO_ALTERNATE_FUNCTION,
                      0);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_SLAVE_MISO_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_MW_SPI_SLAVE_MOSI_GPIO_PORT),
                      BOARD_MW_SPI_SLAVE_MOSI_GPIO_PIN,
                      BOARD_MW_SPI_SLAVE_MOSI_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_SLAVE_MOSI_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_MW_SPI_SLAVE_SCLK_GPIO_PORT),
                      BOARD_MW_SPI_SLAVE_SCLK_GPIO_PIN,
                      BOARD_MW_SPI_SLAVE_SCLK_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_SLAVE_SCLK_PIN\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_MW_SPI_SLAVE_SS0_GPIO_PORT),
                      BOARD_MW_SPI_SLAVE_SS0_GPIO_PIN,
                      BOARD_MW_SPI_SLAVE_SS0_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for SPI_SLAVE_SS_PIN\n");
        return ret;
    }
    return 0;
}
#endif

/**
 * @fn      void SPI_Master_Callback_func (uint32_t event)
 * @brief   SPI Master call back function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  None
 */
static void SPI_Master_Callback_func(uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE) {
        cb_spi_master_status = 1;
    }
}

/**
 * @fn      void SPI_Slave_Callback_func (uint32_t event)
 * @brief   SPI Slave call back function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  None
 */
static void SPI_Slave_Callback_func(uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE) {
        cb_spi_slave_status = 1;
    }
}

/**
 * @fn      void MW_Demo_func(void)
 * @brief   Microwire application function.
 * @note    none.
 * @param   none.
 * @retval  None
 */
static void MW_Demo_func(void)
{
    uint32_t       master_control, slave_control, baudrate = BOARD_MW_SPI_MASTER_BUS_SPEED;
    int32_t        status;
    ARM_SPI_STATUS spi_master_status, spi_slave_status;

    /* Single buffer is used to store both control code and data.
     * Therefore, Buffer width size should be of 4 bytes irrespective of frame format size
     * and control code size (control code size can be configured in RTE_Device.h) */
    uint32_t master_tx_buff[4], slave_rx_buff[4] = {0};
#if !MASTER_TO_SLAVE_TRANSFER
    uint32_t master_rx_buff[4] = {0}, slave_tx_buff[4];
#endif

    printf("Start of MicroWire demo application \n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    status = board_pins_config();
    if (status != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return;
    }

#else
    /*
     * NOTE: The spi master and spi slave pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    status = board_spi_pins_config();
    if (status != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return;
    }
#endif

    /* SPI master instance initialization */
    status = masterDrv->Initialize(SPI_Master_Callback_func);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize SPI Master \n");
        return;
    }

    status = masterDrv->PowerControl(ARM_POWER_FULL);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power SPI Master \n");
        goto error_spi_master_uninitialize;
    }

    master_control = (ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_MICROWIRE |
                      ARM_SPI_DATA_BITS(32));

    status         = masterDrv->Control(master_control, baudrate);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure SPI Master\n");
        goto error_spi_master_power_off;
    }

    /* SPI slave instance initialization */
    status = slaveDrv->Initialize(SPI_Slave_Callback_func);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize SPI Slave \n");
        return;
    }

    status = slaveDrv->PowerControl(ARM_POWER_FULL);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power SPI Slave \n");
        goto error_spi_slave_uninitialize;
    }

    slave_control = (ARM_SPI_MODE_SLAVE | ARM_SPI_MICROWIRE | ARM_SPI_DATA_BITS(32));

    status        = slaveDrv->Control(slave_control, 0);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure SPI Slave\n");
        goto error_spi_slave_power_off;
    }

    status = masterDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure SPI Master\n");
        goto error_spi_master_power_off;
    }

#if MASTER_TO_SLAVE_TRANSFER

    printf("\nData transfer from master to slave \n");

    master_tx_buff[0] = 0x1111;     /* control word 1 */
    master_tx_buff[1] = 0x11111111; /* data 1 */
    master_tx_buff[2] = 0x2222;     /* control word 2 */
    master_tx_buff[3] = 0x22222222; /* data 2 */

    /* The second parameter should be the total number of data to be transferred,
     * excluding the control frame number. */
    status            = slaveDrv->Receive(slave_rx_buff, 2);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed SPI Slave to configure as rx\n");
        goto error_spi_slave_power_off;
    }

    /* The second parameter should be the total number of data to be transferred,
     * excluding the control frame number. */
    status = masterDrv->Send(master_tx_buff, 2);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed SPI Master to configure as tx\n");
        goto error_spi_master_power_off;
    }

#else

    /* assign control codes to master tx buffer */
    master_tx_buff[0] = 0x1111; /* control word 1 */
    master_tx_buff[1] = 0x2222; /* control word 2 */

    /* assign slave tx buffer */
    slave_tx_buff[0]  = 0x11111111; /* data 1 */
    slave_tx_buff[1]  = 0x22222222; /* data 2 */

    printf("\nData transfer from slave to master \n");

    /* The third parameter should be the total number of data to be transferred,
     * excluding the control frame number. */
    status = slaveDrv->Transfer(slave_tx_buff, slave_rx_buff, 2);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed SPI Slave to configure as tx\n");
        goto error_spi_slave_power_off;
    }

    /* The third parameter should be the total number of data to be transferred,
     * excluding the control frame number. */
    status = masterDrv->Transfer(master_tx_buff, master_rx_buff, 2);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed SPI Master to configure as rx\n");
        goto error_spi_master_power_off;
    }

#endif

    /* waiting for a transfer complete interrupt */
    while (!(cb_spi_master_status && cb_spi_slave_status)) {
    }
    printf("Data Transfer is completed\n");

    spi_master_status = masterDrv->GetStatus();
    spi_slave_status  = slaveDrv->GetStatus();
    while ((spi_master_status.busy == 1) || (spi_slave_status.busy == 1)) {
        spi_master_status = masterDrv->GetStatus();
        spi_slave_status  = slaveDrv->GetStatus();
    }

    masterDrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    if (status != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure SPI Master\n");
        goto error_spi_master_power_off;
    }

#if MASTER_TO_SLAVE_TRANSFER
    (memcmp(master_tx_buff, slave_rx_buff, 4) == 0)
        ? printf("Master tx & Slave rx buffers are same\n")
        : printf("Master tx & Slave rx rx buffers are different\n");

#else
    (memcmp(master_tx_buff, slave_rx_buff, 2) == 0)
        ? printf("Master tx & Slave rx buffers are same\n")
        : printf("Master tx & Slave rx buffers are different\n");
    (memcmp(slave_tx_buff, master_rx_buff, 2) == 0)
        ? printf("Slave tx & Master rx buffers are same\n")
        : printf("Slave tx & Master rx buffers are different\n");

#endif

error_spi_master_power_off:
    status = masterDrv->PowerControl(ARM_POWER_OFF);
    if (status != ARM_DRIVER_OK) {
        printf("Error in SPI Master Power Off \n");
    }

error_spi_master_uninitialize:
    status = masterDrv->Uninitialize();
    if (status != ARM_DRIVER_OK) {
        printf("Error in SPI Master Uninitialize \n");
    }

error_spi_slave_power_off:
    status = slaveDrv->PowerControl(ARM_POWER_OFF);
    if (status != ARM_DRIVER_OK) {
        printf("Error in SPI Slave Power Off \n");
    }

error_spi_slave_uninitialize:
    status = slaveDrv->Uninitialize();
    if (status != ARM_DRIVER_OK) {
        printf("Error in SPI Slave Uninitialize \n");
    }

    printf("\nEnd of MicroWire demo application \n");
}

int main(void)
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
    MW_Demo_func();
}
