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
 * @file     : demo_gt911.c
 * @author   : Chandra Bhushan Singh
 * @email    : chandrabhushan.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 08-August-2023
 * @brief    : Baremetal demo application to verify GT911 touch screen.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "board_config.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/*touch screen driver */
#include "Driver_Touch_Screen.h"

/* Touch screen driver instance */
extern ARM_DRIVER_TOUCH_SCREEN  GT911;
static ARM_DRIVER_TOUCH_SCREEN *Drv_Touchscreen = &GT911;

void touchscreen_demo();

#define ACTIVE_TOUCH_POINTS 5

/**
 * @function    void touchscreen_demo()
\brief          Bare Metal TestApp to verify GT911 touch screen.
                This demo application does:
                    - initialize i2c AND gpio9 port hardware pins
                    - initialize UART hardware pins if print redirection to UART is chosen
                    - initialize USART driver if printf redirection to UART is chosen.
                    - initialize GT911 Touch screen driver with call back function.
                    - check if touch screen is pressed or not
                    - if pressed then print up to 5 coordinate positions where display touch screen
was touched. \param[in]   none \return      none
  */
void touchscreen_demo()
{
    int32_t            ret;
    int32_t            count = 0;
    ARM_DRIVER_VERSION version;
    ARM_TOUCH_STATE    state;

    /* Initialize i2c and GPIO9 hardware pins using PinMux Driver. */
    /* Initialize UART4 hardware pins using PinMux driver if printf redirection to UART is selected
     */
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

    /* Touch screen version */
    version = Drv_Touchscreen->GetVersion();
    printf("\r\n Touchscreen driver version api:0x%" PRIx16 ""
           "driver:0x%" PRIx16 " \r\n",
           version.api,
           version.drv);

    /* Initialize GT911 touch screen */
    ret = Drv_Touchscreen->Initialize();
    if (ret != ARM_DRIVER_OK) {
        /* Error in GT911 touch screen initialize */
        printf("\r\n Error: GT911 touch screen initialization failed.\r\n");
        goto error_GT911_uninitialize;
    }

    /* Power ON GT911 touch screen */
    ret = Drv_Touchscreen->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        /* Error in GT911 touch screen power up */
        printf("\r\n Error: GT911 touch screen Power Up failed.\r\n");
        goto error_GT911_uninitialize;
    }

    while (1) {
        /* Reading GT911 touch screen press status */
        ret = Drv_Touchscreen->GetState(&state);
        if (ret != ARM_DRIVER_OK) {
            /* Error in GT911 touch screen read status */
            printf("\r\n Error: GT911 touch screen read  status failed.\r\n");
            goto error_GT911_poweroff;
        }

        if (state.numtouches) {
            for (count = 1; count <= ACTIVE_TOUCH_POINTS; count++) {
                /* Print coordinates positions of pressing on GT911 touch screen up to max touch
                 * points set */
                printf("x%" PRId32 ": %" PRId16 " y%" PRId32 ": %" PRId16 " \r\n",
                       count,
                       state.coordinates[count - 1].x,
                       count,
                       state.coordinates[count - 1].y);
            }
            memset(state.coordinates, 0, sizeof(state.coordinates));
        } else {
            /* Clear coordinates when no touch is detected to avoid printing old data */
            memset(state.coordinates, 0, sizeof(state.coordinates));
        }
    }

error_GT911_poweroff:
    /* Received error Power off GT911 touch screen peripheral */
    ret = Drv_Touchscreen->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        /* Error in GT911 Touch screen Power OFF. */
        printf("ERROR: Could not power OFF touch screen\n");
        return;
    }

error_GT911_uninitialize:
    /* Received error Un-initialize Touch screen driver */
    ret = Drv_Touchscreen->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        /* Error in GT911 Touch screen uninitialize. */
        printf("ERROR: Could not unintialize touch screen\n");
        return;
    }
}

/* Define main entry point.  */
int main()
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

    touchscreen_demo();
    return 0;
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
