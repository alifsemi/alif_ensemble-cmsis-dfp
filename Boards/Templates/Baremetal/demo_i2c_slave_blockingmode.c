/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     : demo_i2c_slave_blockingmode.c
 * @author   : Vijay Kumar
 * @email    : vijay.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 31-Aug-2026
 * @brief    : Baremetal demo application to verify I2C Slave functionality
 *             in Blocking mode
 *             Code will verify below cases:
 *             1) Master transmits 30 bytes and Slave receives 30 bytes
 *             2) Slave transmits 29 bytes and Master receives 29 bytes
 *
 *             For E7: I2C1 instance is taken as Slave (PIN used P7_2 and P7_3)
 *             HE core is used to run the Slave application.
 *
 *             E7: Hardware setup:
 *             - Connecting GPIO pins of I2C0 to I2C1
 *               SDA pin P3_5(J11) to P7_2(J15)
 *               SCL pin P3_4(J11) to P7_3(J15).
 *
 * @bug      : None.
 * @Note     : Performs Slave transmit and receive operations in Blocking mode.
 ******************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "app_utils.h"

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "board_config.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/* I2C configuration */
#define ADDRESS_MODE_7BIT  1                 /* I2C 7 bit addressing mode     */
#define ADDRESS_MODE_10BIT 2                 /* I2C 10 bit addressing mode    */
#define ADDRESS_MODE       ADDRESS_MODE_7BIT /* 7 bit addressing mode chosen  */

/* Slave I2C instance */
extern ARM_DRIVER_I2C ARM_Driver_I2C_(BOARD_SLAVE_I2C_INSTANCE);
static ARM_DRIVER_I2C *I2C_SlvDrv = &ARM_Driver_I2C_(BOARD_SLAVE_I2C_INSTANCE);

/* Slave address */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
#define SAR_ADDRS       (0x2D0) /* 10 bit Slave Own Address,     use by Slave  */
#else
#define SAR_ADDRS       (0x40)  /* 7 bit Slave Own Address,      use by Slave  */
#endif

/* Test data */
/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT   30
/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT   29

/* Slave TX Data */
static uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT] = {"/*!Test Message from Slave!*/"};
/* slave receive buffer */
static uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT];

/**
 * @fn      static void I2C_slave_blocking_mode_demo(void)
 * @brief   Performs I2C Slave blocking comm demo
 * @note    none
 * @param   none
 * @retval  none
 */
static void I2C_slave_blocking_mode_demo(void)
{
    int32_t ret;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> I2C Slave Blocking Test Starting up!!! <<< \r\n");

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Pin configuration failed: %" PRId32 "\r\n", ret);
        return;
    }

    /* Get driver version */
    version = I2C_SlvDrv->GetVersion();
    printf("I2C version API: 0x%" PRIx16 " driver: 0x%" PRIx16 "\r\n",
              version.api,
              version.drv);

    /* Initialize I2C slave Blocking mode does not require callback */
    ret = I2C_SlvDrv->Initialize(NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Slave Initialize failed\r\n");
        return;
    }

    /* I2C Slave Power Control */
    ret = I2C_SlvDrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Slave PowerControl failed\r\n");
        goto error_uninitialize;
    }

    /* Slave Control Configure slave own address */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS, (SAR_ADDRS | ARM_I2C_ADDRESS_10BIT));
#else
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS, SAR_ADDRS);
#endif
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C slave control failed\n");
        goto error_poweroff;
    }

    printf("\r\nSlave initialized successfully\r\n");

    /* Slave Receive */
    printf("\r\n------- Slave receive -------\r\n");
    printf("Slave waiting for MasterTransmit...\r\n");

    /* Blocking call this waits until the master starts the transaction */

    ret = I2C_S lvDrv->SlaveReceive(SLV_RX_BUF, MST_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SlaveReceive failed: %" PRId32 "\r\n", ret);
        goto error_poweroff;
    }

    printf("Received bytes:\r\n");
    for (uint32_t i = 0; i < MST_BYTE_TO_TRANSMIT; i++) {
        printf("%c", SLV_RX_BUF[i]);
    }
    static const uint8_t expected_master_data[] = "/*!Test Message from Master!*/";

    /* Compare data received from master */
    if (memcmp(&SLV_RX_BUF, &expected_master_data, MST_BYTE_TO_TRANSMIT) != 0) {
        printf("\r\nERROR: Slave receive data mismatch\r\n");
        printf("\n ---STOP--- \r\n wait forever >>> \n");
        WAIT_FOREVER_LOOP
    }
    printf("\r\nSlaveReceive completed successfully\r\n");
    printf("\r\nSlave receive data verified successfully\r\n");

    /*  Small delay before next transaction */
    sys_busy_loop_us(500);

    /* Slave Transmit */
    printf("\r\n------- Slave transmit -------\r\n");
    printf("Slave waiting for Master read request...\r\n");
    /*
     * SlaveTransmit() may block waiting for RD_REQ.
     * The MasterReceive() is running on the OTHER CORE,
     * so it can generate the RD_REQ while this call waits.
     */
    ret = I2C_SlvDrv->SlaveTransmit(SLV_TX_BUF, SLV_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: SlaveTransmit failed: %" PRId32 "\r\n", ret);
        goto error_poweroff;
    }
    printf("SlaveTransmit completed successfully\r\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_poweroff :

    ret = I2C_SlvDrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Slave Power OFF failed\r\n");
    }

error_uninitialize :

    ret = I2C_SlvDrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Slave Uninitialize failed\r\n");
    }
    printf("\r\nI2C Slave blocking demo exiting\r\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP
}

/**
 * @fn      int main(void)
 * @brief   Entry point for I2C Slave blocking mode application
 * @note    none
 * @param   none
 * @retval  none
 */
int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t ret;

    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

#if RTE_I2C1_BLOCKING_MODE_ENABLE
    I2C_slave_blocking_mode_demo();
#else
#error "I2C1 Blocking mode is not enabled in RTE_Device.h"
#endif

    WAIT_FOREVER_LOOP
}
