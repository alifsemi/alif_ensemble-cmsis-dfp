/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/******************************************************************************
 * @file     : demo_bmi323.c
 * @author   : Shreehari H K
 * @email    : shreehari.hk@alifsemi.com
 * @version  : V1.0.0
 * @date     : 28-Oct-2024
 * @brief    : Baremetal app to verify BMI323 IMU sensor
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "app_utils.h"

/* Project Includes */
#include "board_config.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "Driver_IO.h"
#include "Driver_IMU.h"

/* IMU Driver instance */
extern ARM_DRIVER_IMU  BMI323;
static ARM_DRIVER_IMU *Drv_IMU = &BMI323;

/**
  \fn          int32_t hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   none
  \return      0:success; -1:failure
*/
static int32_t hardware_init(void)
{
    /* for I3C_D(PORT_7 PIN_6(SDA)/PIN_7(SCL)) instance,
     *  for I3C in I3C mode (not required for I3C in I2C mode)
     *  GPIO voltage level(flex) has to be change to 1.8-V power supply.
     *
     *  GPIO_CTRL Register field VOLT:
     *   Select voltage level for the 1.8-V/3.3-V (flex) I/O pins
     *   0x0: I/O pin will be used with a 3.3-V power supply
     *   0x1: I/O pin will be used with a 1.8-V power supply
     */

    /* Configure GPIO flex I/O pins to 1.8-V:
     *  P7_6 and P7_7 pins are part of GPIO flex I/O pins.
     */
    int32_t       ret        = 0;
    uint32_t      error_code = SERVICES_REQ_SUCCESS;
    uint32_t      service_error_code;
    run_profile_t runp;

    /* config flexio pins to 1.8V */
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

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return ret;
    }
    return ARM_DRIVER_OK;
}

/**
  \fn          void imu_bmi323_demo(void)
  \brief       TestApp to verify IMU
  \            This demo performs below operations:
  \            Sets-Up the BMI323 Inertial Measurement Unit
  \            Fetches Accelermeter, Gyroscope and Temperature data
  \return      none
*/
static void imu_bmi323_demo(void)
{
    ARM_DRIVER_VERSION  version;
    ARM_IMU_COORDINATES data;
    float               temperature;
    ARM_IMU_STATUS      status;
    int32_t             ret;
    uint8_t             iter;

    printf("\r\n BMI323 IMU demo Starting...\r\n");

    /* Initialize i3c hardware pins using PinMux Driver. */
    ret = hardware_init();
    if (ret != 0) {
        printf("\r\n Error: i3c hardware_init failed.\r\n");
        return;
    }

    /* IMU version */
    version = Drv_IMU->GetVersion();
    printf("\r\n IMU version api:0x%" PRId16 " driver:0x%" PRId16 " \r\n",
           version.api,
           version.drv);

    /* IMU initialization */
    ret = Drv_IMU->Initialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: IMU Initialize failed.\r\n");
        goto error_uninitialize;
    }

    /* IMU power up */
    ret = Drv_IMU->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: IMU Power-up failed.\r\n");
        goto error_poweroff;
    }

    while (1) {
        /* Enable interrupt */
        ret = Drv_IMU->Control(IMU_SET_INTERRUPT, true);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: Enabling interrupt \r\n");
            goto error_poweroff;
        }

        /* Gets IMU status */
        status = Drv_IMU->GetStatus();

        if (status.data_rcvd) {
            /* Disable interrupt */
            ret = Drv_IMU->Control(IMU_SET_INTERRUPT, false);
            if (ret != ARM_DRIVER_OK) {
                printf("\r\n Error: Disabling interrupt \r\n");
                goto error_poweroff;
            }

            if (status.drdy_status & IMU_ACCELEROMETER_DATA_READY) {
                /* Read Accelerometer data */
                ret = Drv_IMU->Control(IMU_GET_ACCELEROMETER_DATA, (uint32_t) &data);
                if (ret != ARM_DRIVER_OK) {
                    printf("\r\n Error: IMU Accelerometer data \r\n");
                    goto error_poweroff;
                }

                printf("\t\tAccel Data--> x:%" PRId16 "mg, y:%" PRId16 "mg, z:%" PRId16 "mg\r\n",
                       data.x,
                       data.y,
                       data.z);
            }

            if (status.drdy_status & IMU_GYRO_DATA_READY) {
                /* Read Gyroscope data */
                ret = Drv_IMU->Control(IMU_GET_GYROSCOPE_DATA, (uint32_t) &data);
                if (ret != ARM_DRIVER_OK) {
                    printf("\r\n Error: IMU Gyroscope data \r\n");
                    goto error_poweroff;
                }

                printf("\t\tGyro Data-->  x:%" PRId16 "mdps, y:%" PRId16 "mdps, z:%" PRId16
                       "mdps\r\n",
                       data.x,
                       data.y,
                       data.z);
            }

            if (status.drdy_status & IMU_TEMPERATURE_DATA_READY) {
                /* Read Temperature data */
                ret = Drv_IMU->Control(IMU_GET_TEMPERATURE_DATA, (uint32_t) &temperature);
                if (ret != ARM_DRIVER_OK) {
                    printf("\r\n Error: IMU Temperature data \r\n");
                    goto error_poweroff;
                }

                printf("\t\tTemp Data-->  %fC\r\n\r\n", temperature);
            }
            /* wait for 1 sec */
            for (iter = 0; iter < 10; iter++) {
                sys_busy_loop_us(100000);
            }
        }
    }

error_poweroff:
    /* Power off IMU driver*/
    ret = Drv_IMU->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        /* Error in IMU Power OFF. */
        printf("ERROR: Could not power OFF IMU\n");
        return;
    }

error_uninitialize:
    /* Un-initialize IMU driver */
    ret = Drv_IMU->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        /* Error in IMU uninitialize. */
        printf("ERROR: Could not unintialize IMU\n");
        return;
    }

    printf("\r\n BMI323 IMU demo exiting...\r\n");
}

int main()
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    int32_t ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif
    /* Enter the Demo.  */
    imu_bmi323_demo();

    return 0;
}
