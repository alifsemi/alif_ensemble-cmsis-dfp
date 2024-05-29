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
 * @file     Demo_wifi_baremetal.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    Demo app to use Wi-Fi module
 *
 ******************************************************************************/

/* System Includes */
#include <stdio.h>

#include <RTE_Components.h>
#include CMSIS_device_header
#include "pinconf.h"

#if defined(RTE_Compiler_IO_STDIN)
#include "retarget_stdin.h"
#endif  /* RTE_Compiler_IO_STDIN */

#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* Project Includes */
#include "Driver_WiFi.h"

/* Wi-Fi Driver instance 0 */
extern ARM_DRIVER_WIFI Driver_WiFi0;
static ARM_DRIVER_WIFI *wifiDrv     = &Driver_WiFi0;

#define SPI_PORT                    PORT_12
#define SPI_MISO_PIN                PIN_4
#define SPI_MOSI_PIN                PIN_5
#define SPI_CLOCK_PIN               PIN_6
#define SPI_SS0_PIN                 PIN_7

#define SPI_MISO_PIN_AF             PINMUX_ALTERNATE_FUNCTION_2
#define SPI_MOSI_PIN_AF             PINMUX_ALTERNATE_FUNCTION_2
#define SPI_CLOCK_PIN_AF            PINMUX_ALTERNATE_FUNCTION_2
#define SPI_SS0_PIN_AF              PINMUX_ALTERNATE_FUNCTION_3


void myCallBack(uint32_t event, void *ptr)
{

}

int32_t wifi_hw_init()
{
    int32_t ret;
    uint8_t control;

    /* Doing pinmux configuration for SPI pin's */
    // Output = WIFI SS = SPI_CS (Chip Select)
    ret = pinconf_set(SPI_PORT, SPI_SS0_PIN, SPI_SS0_PIN_AF, control);
    if( ret < 0)
        return ret;

    ret = pinconf_set(SPI_PORT, SPI_MISO_PIN, SPI_MISO_PIN_AF, PADCTRL_READ_ENABLE);
    if( ret < 0)
        return ret;

    //PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA | PADCTRL_SLEW_RATE_FAST;
    control = PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA;
    ret = pinconf_set(SPI_PORT, SPI_MOSI_PIN, SPI_MOSI_PIN_AF, control);
    if( ret < 0)
        return ret;

    ret = pinconf_set(SPI_PORT, SPI_CLOCK_PIN, SPI_CLOCK_PIN_AF, control);
    if( ret < 0)
        return ret;

    // Output = WIFI Enable (Power Telit module)
    ret = pinconf_set(PORT_13, PIN_0, PINMUX_ALTERNATE_FUNCTION_0, 0);
    if( ret < 0)
        return ret;

    // Output = WIFI Reset
    ret = pinconf_set(PORT_13, PIN_2, PINMUX_ALTERNATE_FUNCTION_0, 0);
    if( ret < 0)
        return ret;

    // Input = WIFI_IRQ
    ret = pinconf_set(PORT_13, PIN_3, PINMUX_ALTERNATE_FUNCTION_0,
             PADCTRL_READ_ENABLE);
    if( ret < 0)
        return ret;

    return ret;
}
/**
  @fn           int main(void)
  @brief        Application Entry : Test-app for Wi-Fi.
  @return       exit code
*/
int main(void)
{
    int32_t ret;
    uint8_t info[128] = {0};

    /* Log Retargeting Initialization */
#if defined(RTE_Compiler_IO_STDIN_User)
    ret = stdin_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
#endif

#if defined(RTE_Compiler_IO_STDOUT_User)
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
#endif

    ret = wifi_hw_init();
    if( ret < 0)
        return ret;

    ret = wifiDrv->Initialize(myCallBack);
    if( ret < 0)
        return ret;

    ret = wifiDrv->PowerControl(ARM_POWER_FULL);
    if( ret < 0)
        return ret;

    ret = wifiDrv->GetModuleInfo((char*)info, 126);
    printf(" WiFi Version => %s \n", info);

    return 0;
}

/********************** (c) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
