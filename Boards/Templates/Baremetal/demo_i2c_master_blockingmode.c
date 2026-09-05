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
 * @file     : demo_i2c_master_blockingmode.c
 * @author   : Vijay Kumar
 * @email    : vijay.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 31-Aug-2026
 * @brief    : Baremetal demo application to verify I2C Master functionality
 *             in Blocking mode
 *             Code will verify below cases:
 *             1) Master transmits 30 bytes and Slave receives 30 bytes
 *             2) Slave transmits 29 bytes and Master receives 29 bytes
 *
 *             For E7: I2C0 instance is taken as Master (PIN used P3_4 and P3_5)
 *             HP core is used to run the Master application.
 *
 *             E7: Hardware setup:
 *             - Connecting GPIO pins of I2C0 to I2C1
 *               SDA pin P3_5(J11) to P7_2(J15)
 *               SCL pin P3_4(J11) to P7_3(J15).
 *
 * @bug      : None.
 * @Note     : Performs Master transmit and receive operations in Blocking mode.
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

/* Master I2C instance */
extern ARM_DRIVER_I2C ARM_Driver_I2C_(BOARD_MASTER_I2C_INSTANCE);
static ARM_DRIVER_I2C *I2C_MstDrv = &ARM_Driver_I2C_(BOARD_MASTER_I2C_INSTANCE);

/* Slave address */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
#define TAR_ADDRS       (0x2D0) /* 10 bit Target(Slave) Address, use by Master */
#else
#define TAR_ADDRS       (0x40)  /* 7 bit Target(Slave) Address, use by Master  */
#endif

#define STOP            (0x00)

/* Test data */
/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT   30
/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT   29

/* Master TX Data */
static uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT] = {"/*!Test Message from Master!*/"};
/* master receive buffer */
static uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT];

/**
 * @fn      static void I2C_master_blocking_mode_demo(void)
 * @brief   Performs I2C master blocking comm demo
 * @note    none
 * @param   none
 * @retval  none
 */

static void I2C_master_blocking_mode_demo(void)
{
    int32_t ret;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> I2C Master Blocking Test Starting up!!! <<< \r\n");

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Pin configuration failed: %" PRId32 "\r\n", ret);
        return;
    }

    /* Get driver version */
    version = I2C_MstDrv->GetVersion();
    printf("I2C version API: 0x%" PRIx16 " driver: 0x%" PRIx16 "\r\n",
           version.api,
           version.drv);

    /* Initialize I2C master Driver and blocking mode callback is not required */
    ret = I2C_MstDrv->Initialize(NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Master Initialize failed\r\n");
        return;
    }

    /* I2C Master Power control  */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Master PowerControl failed\r\n");
        goto error_uninitialize;
    }

    /* I2C Master Control */
     ret = I2C_MstDrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS);
     if (ret != ARM_DRIVER_OK) {
         printf("\r\n Error: I2C master control failed\n");
         goto error_poweroff;
     }

    printf("\r\nMaster initialized successfully\r\n");

     /* I2C Master Transmit*/
    printf("\r\n------- Master transmit -------\r\n");
    printf("Master transmitting %d bytes to slave...\r\n", MST_BYTE_TO_TRANSMIT);

#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_MstDrv->MasterTransmit((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                               (uint8_t *) MST_TX_BUF,
                               MST_BYTE_TO_TRANSMIT,
                               STOP);

#else
     ret = I2C_MstDrv->MasterTransmit(TAR_ADDRS, MST_TX_BUF, MST_BYTE_TO_TRANSMIT, STOP);

#endif
    if (ret != ARM_DRIVER_OK) {
	printf("ERROR: MasterTransmit failed: %" PRId32 "\r\n", ret);
        goto error_poweroff;
    }
    printf("MasterTransmit completed successfully\r\n");

    /* At this point the slave application should have completed SlaveReceive() */
    /*  Small delay before next transaction */
    sys_busy_loop_us(500);

    /* I2C Master Receive */
    printf("\r\n------- Master receive -------\r\n");
    printf("Master requesting %d bytes from slave...\r\n", SLV_BYTE_TO_TRANSMIT);

#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_MstDrv->MasterReceive((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                                    (uint8_t *) MST_RX_BUF,
                                    SLV_BYTE_TO_TRANSMIT,
                                    STOP);
#else
    ret = I2C_MstDrv->MasterReceive(TAR_ADDRS, (uint8_t *) MST_RX_BUF, SLV_BYTE_TO_TRANSMIT, STOP);
#endif
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Master Receive failed\n");
        goto error_poweroff;
    }
    printf("MasterReceive completed successfully\r\n");

    /* Compare data received from slave */
    static const uint8_t expected_slave_data[] = {"/*!Test Message from Slave!*/"};

    printf("\r\nMaster received data:\r\n ");

    for (int i = 0; i < SLV_BYTE_TO_TRANSMIT; i++) {
         printf("%c", MST_RX_BUF[i]);
    }
    if (memcmp(&MST_RX_BUF, &expected_slave_data, SLV_BYTE_TO_TRANSMIT) != 0) {
         printf("\r\nERROR: Master receive data mismatch\r\n");
         printf("\n ---STOP--- \r\n wait forever >>> \n");
         WAIT_FOREVER_LOOP
    }
    printf("\r\nMaster receive data verified successfully\r\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_poweroff :
    /* Power off I2C peripheral */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Master Power OFF failed\r\n");
    }

error_uninitialize :
    /* Un-initialize I2C driver */
    ret = I2C_MstDrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: I2C Master Uninitialize failed\r\n");
    }
    printf("\r\nI2C Master blocking demo exiting\r\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP
}

/**
 * @fn      int main(void)
 * @brief   entry point for I2C Master blocking application
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

#if RTE_I2C0_BLOCKING_MODE_ENABLE
    I2C_master_blocking_mode_demo();
#else
#error "I2C0 Blocking mode is not enabled in RTE_Device.h"
#endif

    WAIT_FOREVER_LOOP
}
