/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     : CDC_Baremetal.c
 * @author   : Ahmad Rashed
 * @email    : ahmad.rashed@alifsemi.com
 * @version  : V1.0.0
 * @date     : 22-Nov-2022
 * @brief    : Baremetal demo application code for CDC200 driver
 *              - Fills memory buffer with colors to test LCD panels
 *
 *              Hardware setup:
 *              - Must use either Focus LCDs E50RA-I-MW550-N or E43GB-I-MW405-C
 *              - GPIO port P4_4 is used to enable/disable boost converter for LED backlight
 *              - GPIO port P4_6 is used to drive LCD panel reset
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>

#include <RTE_Device.h>
#include <RTE_Components.h>
#include CMSIS_device_header

/* include the CDC200 driver */
#include "Driver_CDC200.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


#define DIMAGE_X       (RTE_PANEL_HACTIVE_TIME)
#define DIMAGE_Y       (RTE_PANEL_VACTIVE_LINE)
#if RTE_CDC200_PIXEL_FORMAT == 0
    /* ARGB8888 32-bit Format (4-bytes) */
#define PIXEL_BYTES    (4)
#elif RTE_CDC200_PIXEL_FORMAT == 1
    /*  RGB8888 24-bit Format (3-bytes) */
#define PIXEL_BYTES    (3)
#elif RTE_CDC200_PIXEL_FORMAT == 2
    /*  RGB565  16-bit Format (2-bytes) */
#define PIXEL_BYTES    (2)
#endif

static uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][PIXEL_BYTES] __attribute__((section("lcd_frame_buf"))) = {0};

extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

/**
 *    @func         : void display_callback()
 *    @brief        : CDC demo callback
 *                  - normally is not called
 *    @return       : NONE
*/
static void display_callback(uint32_t event)
{
	if(event & ARM_CDC_DSI_ERROR_EVENT)
	{
		/* Transfer Error: Received Hardware error */
		while(1);
	}
}

/**
 *    @func         : void CDC_demo()
 *    @brief        : CDC demo
 *                  - initialize the CDC200 controller
 *                  - write various colors to the memory address.
 *    @return       : NONE
*/
static void CDC_demo()
{
    int32_t ret       = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> CDC demo starting up!!! <<< \r\n");

    version = CDCdrv->GetVersion();
    printf("\r\n CDC version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize CDC driver */
    ret = CDCdrv->Initialize(display_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC init failed\n");
        return;
    }

    /* Power control CDC */
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC Power up failed\n");
        goto error_uninitialize;
    }

    /* configure CDC controller */
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)lcd_image);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC controller configuration failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%X <<<\n",(uint32_t)lcd_image);

    /* Start CDC */
    ret = CDCdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC Start failed\n");
        goto error_poweroff;
    }

    while(1) {
        memset(lcd_image, 0x00, sizeof(lcd_image));
        for(uint32_t count = 0; count < 20; count++)
            sys_busy_loop_us(100 * 1000);

        memset(lcd_image, 0xFF, sizeof(lcd_image));
        for(uint32_t count = 0; count < 20; count++)
            sys_busy_loop_us(100 * 1000);
    }

    /* Stop CDC */
    ret = CDCdrv->Stop();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC Stop failed\n");
        goto error_poweroff;
    }

error_poweroff:

        /* Power off CDC */
        ret = CDCdrv->PowerControl(ARM_POWER_OFF);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: CDC Power OFF failed.\r\n");
        }

error_uninitialize:

        /* Un-initialize CDC driver */
        ret = CDCdrv->Uninitialize();
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: CDC Uninitialize failed.\r\n");
        }

        printf("\r\n XXX CDC demo exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
    #if defined(RTE_Compiler_IO_STDOUT_User)
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
    #endif
    /* Enter the demo Application.  */
    CDC_demo();
    return 0;
}
