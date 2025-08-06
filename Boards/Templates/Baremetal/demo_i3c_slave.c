/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/*******************************************************************************
 * @file     : demo_i3c_slave.c
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 27-May-2023
 * @brief    : Baremetal to verify slave side loop back test
 *
 *             Slave receives n-bytes of data from Master
 *              and sends back received data again to Master.
 *
 *             I3C slave configuration.
 *              - In control API parameter:
 *                control : I3C_SET_SLAVE_ADDR(set slave address)
 *                arg     : I3C_SLAVE_ADDRESS macro is defined to set address
 *                          of slave
 *
 *             Hardware Setup:
 *              Required two boards one for Master and one for Slave
 *               (as there is only one i3c instance is available on ASIC).
 *
 *             For E7: Connect SDA to SDA and SCL to SCL and GND to GND.
 *              - SDA P7_6 -> SDA P7_6
 *              - SCL P7_7 -> SCL P7_7
 *              - GND      -> GND
 *             For E1C: Connect SDA to SDA and SCL to SCL and GND to GND.
 *              - SDA P0_6 -> SDA P0_6
 *              - SCL P0_5 -> SCL P0_5
 *              - GND      -> GND
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <inttypes.h>

/* Project Includes */
/* I3C Driver */
#include "Driver_I3C.h"
#include "app_utils.h"

/* PINMUX Driver */
#include "board_config.h"

#include "RTE_Device.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/* i3c Driver instance 0 */
extern ARM_DRIVER_I3C  Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

/* set slave address */
#define I3C_SLAVE_ADDRESS (0X48)

/* receive data from i3c */
uint8_t __ALIGNED(4) rx_data[4];

uint32_t tx_cnt;
uint32_t rx_cnt;

/* flag for callback event. */
volatile uint32_t cb_event;

void i3c_slave_loopback_demo(void);

/* i3c callback events */
typedef enum _I3C_CB_EVENT {
    I3C_CB_EVENT_SUCCESS = (1 << 0),
    I3C_CB_EVENT_ERROR   = (1 << 1)
} I3C_CB_EVENT;

/**
  \fn          int32_t hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      0:success; -1:failure
*/
int32_t hardware_init(void)
{
    int32_t ret = 0;
#if BOARD_I3C_FLEXIO_PRESENT
    /* for I3C_D(PORT_7 PIN_6(SDA)/PIN_7(SCL)) instance,
     *  for I3C in I3C mode (not required for I3C in I2C mode)
     *  GPIO voltage level(flex) has to be change to 1.8-V power supply.
     *
     *  GPIO_CTRL Register field VOLT:
     *   Select voltage level for the 1.8-V/3.3-V (flex) I/O pins
     *    0x0: I/O pin will be used with a 3.3-V power supply
     *    0x1: I/O pin will be used with a 1.8-V power supply
     */

    /* Configure GPIO flex I/O pins to 1.8-V:
     *  P7_6 and P7_7 pins are part of GPIO flex I/O pins.
     */
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
#endif

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Pin configuration failed: %" PRId32 "\n", ret);
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          void I3C_callback(UINT event)
  \brief       i3c isr callback
  \param[in]   event: i3c Event
  \return      none
*/
static void I3C_callback(uint32_t event)
{
    if (event & ARM_I3C_EVENT_TRANSFER_DONE) {
        cb_event = I3C_CB_EVENT_SUCCESS;
    }
    if (event & ARM_I3C_EVENT_TRANSFER_ERROR) {
        cb_event = I3C_CB_EVENT_ERROR;
    }
}

/**
  \fn          void i3c_slave_loopback_demo(void)
  \brief       Baremetal to verify i3c slave mode
               This demo does:
                 - initialize i3c driver;
                 - set slave address and initialize slave
                 - Receive and transmit byte from master
  \return      none
*/
void i3c_slave_loopback_demo(void)
{
    int ret = 0;
    int len = 0;

    ARM_DRIVER_VERSION version;

    printf("\r\n \t\t >>> Slave loop back demo starting up!!! <<< \r\n");

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();
    printf("\r\n i3c version api:0x%" PRIx16 " driver:0x%" PRIx16 " \r\n",
           version.api,
           version.drv);

    if ((version.api < ARM_DRIVER_VERSION_MAJOR_MINOR(7U, 0U)) ||
        (version.drv < ARM_DRIVER_VERSION_MAJOR_MINOR(7U, 0U)))
    {
        printf("\r\n Error: >>>Old driver<<< Please use new one \r\n");
        return;
    }

    /* Initialize i3c hardware pins using PinMux Driver. */
    ret = hardware_init();
    if (ret != 0) {
        printf("\r\n Error: i3c hardware_init failed.\r\n");
        return;
    }

    /* Initialize I3C driver */
#if RTE_I3C_BLOCKING_MODE_ENABLE
    ret = I3Cdrv->Initialize(NULL);
#else
    ret = I3Cdrv->Initialize(I3C_callback);
#endif
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Initialize failed.\r\n");
        return;
    }

    /* Power up I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Power Up failed.\r\n");
        goto error_poweroff;
    }

    /* Control I3C interface */
    ret = I3Cdrv->Control(I3C_SLAVE_SET_ADDR, I3C_SLAVE_ADDRESS);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Control failed.\r\n");
        goto error_uninitialize;
    }

    while (1) {
        len      = 4;

        /* clear callback event flag. */
        cb_event = 0;

        /* Slave Receive */
        ret      = I3Cdrv->SlaveReceive(rx_data, len);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: I3C Slave Receive failed. \r\n");
            goto error_poweroff;
        }

#if !RTE_I3C_BLOCKING_MODE_ENABLE
        /* wait for callback event. */
        while (!((cb_event == I3C_CB_EVENT_SUCCESS) || (cb_event == I3C_CB_EVENT_ERROR))) {
        }

        if (cb_event == I3C_CB_EVENT_ERROR) {
            printf("\nError: I3C Slave Receive failed\n");
            WAIT_FOREVER_LOOP
        }
#endif
        rx_cnt   += 1;

        /* clear callback event flag. */
        cb_event  = 0;

        /* For loop back test,
         * Slave will send received data back to Master.
         */

        /* Slave Transmit*/
        ret       = I3Cdrv->SlaveTransmit(rx_data, len);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: I3C slave Transmit failed. \r\n");
            goto error_poweroff;
        }

#if !RTE_I3C_BLOCKING_MODE_ENABLE
        /* wait for callback event. */
        while (!((cb_event == I3C_CB_EVENT_SUCCESS) || (cb_event == I3C_CB_EVENT_ERROR))) {
        }

        if (cb_event == I3C_CB_EVENT_ERROR) {
            printf("\nError: I2C Slave Transmit failed\n");
            WAIT_FOREVER_LOOP
        }
#endif
        tx_cnt += 1;
    }

error_poweroff:

    /* Power off I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize I3C driver */
    ret = I3Cdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Uninitialize failed.\r\n");
    }

    printf("\r\n I3C demo exiting ...\r\n");
}

/* Define main entry point.  */
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
    /* Enter the I3C.  */
    i3c_slave_loopback_demo();
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
