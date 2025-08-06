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
 * @file     : demo_display_parallel_freertos.c
 * @author   : Chandra Bhushan Singh
 * @email    : chandrabhushan.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 16-Oct-2023
 * @brief    : FreeRTOS demo application code for parallel display
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>

#include <RTE_Components.h>
#include CMSIS_device_header
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

/* include the CDC200 driver */
#include "Driver_CDC200.h"

/* PINMUX Driver */
#include "pinconf.h"
#include "board_config.h"
/* SE Services */
#include "se_services_port.h"

/*RTOS Includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

// Set to 0: Use application-defined cdc200 pin configuration (via board_cdc200_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

extern ARM_DRIVER_CDC200  Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

/*Define for FreeRTOS*/
#define STACK_SIZE                    1024
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

/* Thread id of thread */
TaskHandle_t display_xHandle;

/****************************** FreeRTOS functions **********************/

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &IdleTcb;
    *ppxIdleTaskStackBuffer = IdleStack;
    *pulIdleTaskStackSize   = IDLE_TASK_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pxTask;

    ASSERT_HANG_LOOP
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t  **ppxTimerTaskStackBuffer,
                                    uint32_t      *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &TimerTcb;
    *ppxTimerTaskStackBuffer = TimerStack;
    *pulTimerTaskStackSize   = TIMER_SERVICE_TASK_STACK_SIZE;
}

void vApplicationIdleHook(void)
{
    ASSERT_HANG_LOOP
}

/*****************Only for FreeRTOS use *************************/

#define DIMAGE_X (RTE_PANEL_HACTIVE_TIME)
#define DIMAGE_Y (RTE_PANEL_VACTIVE_LINE)

#if (RTE_CDC200_PIXEL_FORMAT == 0)
/* ARGB8888 32-bit Format (4-bytes) */
#define PIXEL_BYTES (4)
#elif (RTE_CDC200_PIXEL_FORMAT == 1)
/* RGB888 24-bit Format (3-bytes) */
#define PIXEL_BYTES (3)
#elif (RTE_CDC200_PIXEL_FORMAT == 2)
/* RGB565  16-bit Format (2-bytes) */
#define PIXEL_BYTES (2)
#elif (RTE_CDC200_PIXEL_FORMAT == 3)
/* RGBA8888 32-bit Format (3-bytes) */
#define PIXEL_BYTES (4)
#elif (RTE_CDC200_PIXEL_FORMAT == 6)
/* ARGB1555 16-bit Format (2-bytes) */
#define PIXEL_BYTES (2)
#elif (RTE_CDC200_PIXEL_FORMAT == 7)
/* ARGB4444 16-bit Format (2-bytes) */
#define PIXEL_BYTES (2)
#endif

static uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][PIXEL_BYTES]
    __attribute__((section(".bss.lcd_frame_buf")));

/**
 *   @func     : void display_callback(uint32_t event)
 *   @brief    : Parallel display demo callback
 *                  - normally is not called
 *   \param[in]: event: Display Event
 *   @return   : NONE
 */
static void display_callback(uint32_t event)
{
    /* We are not handling any error for our parallel display demo app */
    ;
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_cdc200_pins_config(void)
 * @brief   Configure additional cdc200 pinmux settings not handled
 *          by the board support library.
 * @retval  execution status.
 */

int board_cdc200_pins_config(void)
{
    int ret;

    /* Configure Pin for cdc_pclk */
    ret = pinconf_set(PORT_(BOARD_CDC_PCLK_GPIO_PORT),
                      BOARD_CDC_PCLK_GPIO_PIN,
                      BOARD_CDC_PCLK_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_PCLK \n");
        return ret;
    }

    /* Configure Pin for cdc_de */
    ret = pinconf_set(PORT_(BOARD_CDC_DE_GPIO_PORT),
                      BOARD_CDC_DE_GPIO_PIN,
                      BOARD_CDC_DE_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_DE \n");
        return ret;
    }

    /* Configure Pin for cdc_hsync */
    ret = pinconf_set(PORT_(BOARD_CDC_HSYNC_GPIO_PORT),
                      BOARD_CDC_HSYNC_GPIO_PIN,
                      BOARD_CDC_HSYNC_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_HSYNC \n");
        return ret;
    }

    /* Configure Pin for cdc_vsync */
    ret = pinconf_set(PORT_(BOARD_CDC_VSYNC_GPIO_PORT),
                      BOARD_CDC_VSYNC_GPIO_PIN,
                      BOARD_CDC_VSYNC_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_VSYNC \n");
        return ret;
    }

    /* Configure Pin for cdc_d0 */
    ret = pinconf_set(PORT_(BOARD_CDC_D0_GPIO_PORT),
                      BOARD_CDC_D0_GPIO_PIN,
                      BOARD_CDC_D0_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D0 \n");
        return ret;
    }

    /* Configure Pin for cdc_d1 */
    ret = pinconf_set(PORT_(BOARD_CDC_D1_GPIO_PORT),
                      BOARD_CDC_D1_GPIO_PIN,
                      BOARD_CDC_D1_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D1 \n");
        return ret;
    }

    /* Configure Pin for cdc_d2 */
    ret = pinconf_set(PORT_(BOARD_CDC_D2_GPIO_PORT),
                      BOARD_CDC_D2_GPIO_PIN,
                      BOARD_CDC_D2_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D2 \n");
        return ret;
    }

    /* Configure Pin for cdc_d3 */
    ret = pinconf_set(PORT_(BOARD_CDC_D3_GPIO_PORT),
                      BOARD_CDC_D3_GPIO_PIN,
                      BOARD_CDC_D3_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D3 \n");
        return ret;
    }

    /* Configure Pin for cdc_d4 */
    ret = pinconf_set(PORT_(BOARD_CDC_D4_GPIO_PORT),
                      BOARD_CDC_D4_GPIO_PIN,
                      BOARD_CDC_D4_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D4 \n");
        return ret;
    }

    /* Configure Pin for cdc_d5 */
    ret = pinconf_set(PORT_(BOARD_CDC_D5_GPIO_PORT),
                      BOARD_CDC_D5_GPIO_PIN,
                      BOARD_CDC_D5_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D5 \n");
        return ret;
    }

    /* Configure Pin for cdc_d6 */
    ret = pinconf_set(PORT_(BOARD_CDC_D6_GPIO_PORT),
                      BOARD_CDC_D6_GPIO_PIN,
                      BOARD_CDC_D6_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D6 \n");
        return ret;
    }

    /* Configure Pin for cdc_d7 */
    ret = pinconf_set(PORT_(BOARD_CDC_D7_GPIO_PORT),
                      BOARD_CDC_D7_GPIO_PIN,
                      BOARD_CDC_D7_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D7 \n");
        return ret;
    }

    /* Configure Pin for cdc_d8 */
    ret = pinconf_set(PORT_(BOARD_CDC_D8_GPIO_PORT),
                      BOARD_CDC_D8_GPIO_PIN,
                      BOARD_CDC_D8_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D8 \n");
        return ret;
    }

    /* Configure Pin for cdc_d9 */
    ret = pinconf_set(PORT_(BOARD_CDC_D9_GPIO_PORT),
                      BOARD_CDC_D9_GPIO_PIN,
                      BOARD_CDC_D9_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D9 \n");
        return ret;
    }

    /* Configure Pin for cdc_d10 */
    ret = pinconf_set(PORT_(BOARD_CDC_D10_GPIO_PORT),
                      BOARD_CDC_D10_GPIO_PIN,
                      BOARD_CDC_D10_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D10 \n");
        return ret;
    }

    /* Configure Pin for cdc_d11 */
    ret = pinconf_set(PORT_(BOARD_CDC_D11_GPIO_PORT),
                      BOARD_CDC_D11_GPIO_PIN,
                      BOARD_CDC_D11_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D11 \n");
        return ret;
    }

    /* Configure Pin for cdc_d12 */
    ret = pinconf_set(PORT_(BOARD_CDC_D12_GPIO_PORT),
                      BOARD_CDC_D12_GPIO_PIN,
                      BOARD_CDC_D12_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D12 \n");
        return ret;
    }

    /* Configure Pin for cdc_d13 */
    ret = pinconf_set(PORT_(BOARD_CDC_D13_GPIO_PORT),
                      BOARD_CDC_D13_GPIO_PIN,
                      BOARD_CDC_D13_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D13 \n");
        return ret;
    }

    /* Configure Pin for cdc_d14 */
    ret = pinconf_set(PORT_(BOARD_CDC_D14_GPIO_PORT),
                      BOARD_CDC_D14_GPIO_PIN,
                      BOARD_CDC_D14_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D14 \n");
        return ret;
    }

    /* Configure Pin for cdc_d15 */
    ret = pinconf_set(PORT_(BOARD_CDC_D15_GPIO_PORT),
                      BOARD_CDC_D15_GPIO_PIN,
                      BOARD_CDC_D15_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D15 \n");
        return ret;
    }

    /* Configure Pin for cdc_d16 */
    ret = pinconf_set(PORT_(BOARD_CDC_D16_GPIO_PORT),
                      BOARD_CDC_D16_GPIO_PIN,
                      BOARD_CDC_D16_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D16 \n");
        return ret;
    }

    /* Configure Pin for cdc_d17 */
    ret = pinconf_set(PORT_(BOARD_CDC_D17_GPIO_PORT),
                      BOARD_CDC_D17_GPIO_PIN,
                      BOARD_CDC_D17_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D17 \n");
        return ret;
    }

    /* Configure Pin for cdc_d18 */
    ret = pinconf_set(PORT_(BOARD_CDC_D18_GPIO_PORT),
                      BOARD_CDC_D18_GPIO_PIN,
                      BOARD_CDC_D18_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D18 \n");
        return ret;
    }

    /* Configure Pin for cdc_d19 */
    ret = pinconf_set(PORT_(BOARD_CDC_D19_GPIO_PORT),
                      BOARD_CDC_D19_GPIO_PIN,
                      BOARD_CDC_D19_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D19 \n");
        return ret;
    }

    /* Configure Pin for cdc_d20 */
    ret = pinconf_set(PORT_(BOARD_CDC_D20_GPIO_PORT),
                      BOARD_CDC_D20_GPIO_PIN,
                      BOARD_CDC_D20_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D20 \n");
        return ret;
    }

    /* Configure Pin for cdc_d21 */
    ret = pinconf_set(PORT_(BOARD_CDC_D21_GPIO_PORT),
                      BOARD_CDC_D21_GPIO_PIN,
                      BOARD_CDC_D21_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D21 \n");
        return ret;
    }

    /* Configure Pin for cdc_d22 */
    ret = pinconf_set(PORT_(BOARD_CDC_D22_GPIO_PORT),
                      BOARD_CDC_D22_GPIO_PIN,
                      BOARD_CDC_D22_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D22 \n");
        return ret;
    }

    /* Configure Pin for cdc_d23 */
    ret = pinconf_set(PORT_(BOARD_CDC_D23_GPIO_PORT),
                      BOARD_CDC_D23_GPIO_PIN,
                      BOARD_CDC_D23_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for CDC_D23 \n");
        return ret;
    }

    return 0;
}
#endif

/**
 *    @func         : void parallel_display_demo_thread_entry(void *pvParameters)
 *    @brief        : parallel display demo using FreeRTOS operating system.
 *                  - initialize the CDC200 controller
 *                  - write various colors to the memory address.
 *    @param        : pvParameters.
 *    @return       : NONE
 */
void parallel_display_demo_thread_entry(void *pvParameters)
{
    int32_t            ret = 0;
    uint32_t           service_error_code;
    uint32_t           error_code;
    ARM_DRIVER_VERSION version;

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }

#else
    /*
     * NOTE: The cdc200 pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_cdc200_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }
#endif

    printf("\r\n >>> CDC demo with FreeRTOS is starting up!!! <<< \r\n");

    /* Initialize the SE services */
    se_services_port_init();

    /* Enable MIPI Clocks */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              true,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 100MHz clock enable = %d\n", error_code);
        return;
    }

    error_code =
        SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, true, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC) clock enable = %d\n", error_code);
        goto error_disable_100mhz_clk;
    }

    version = CDCdrv->GetVersion();
    printf("\r\n CDC version api:%X driver:%X...\r\n", version.api, version.drv);

    /* Initialize CDC driver */
    ret = CDCdrv->Initialize(display_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC init failed\n");
        goto error_disable_hfosc_clk;
    }

    /* Power control CDC */
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC Power up failed\n");
        goto error_uninitialize;
    }

    /* configure CDC controller */
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t) lcd_image);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC controller configuration failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%X <<<\n", (uint32_t) lcd_image);

    /* Start CDC */
    ret = CDCdrv->Start();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC Start failed\n");
        goto error_poweroff;
    }

    while (1) {
        memset(lcd_image, 0x00, sizeof(lcd_image));
        for (uint32_t count = 0; count < 20; count++) {
            sys_busy_loop_us(100 * 1000);
        }

        memset(lcd_image, 0xFF, sizeof(lcd_image));
        for (uint32_t count = 0; count < 20; count++) {
            sys_busy_loop_us(100 * 1000);
        }
    }

    /* Stop CDC controller */
    ret = CDCdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC Stop failed\n");
        goto error_poweroff;
    }

error_poweroff:
    /* Power off CDC */
    ret = CDCdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC Power OFF failed.\r\n");
    }
error_uninitialize:
    /* Un-initialize CDC driver */
    ret = CDCdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC Uninitialize failed.\r\n");
    }
error_disable_hfosc_clk:
    error_code =
        SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, false, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC)  clock disable = %d\n", error_code);
    }
error_disable_100mhz_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              false,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 100MHz clock disable = %d\n", error_code);
    }

    printf("\r\n XXX CDC demo exiting XXX...\r\n");
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start the FreeRTOS Kernel
 *---------------------------------------------------------------------------*/
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

    /* System Initialization */
    SystemCoreClockUpdate();

    /* Create application main thread */
    BaseType_t xReturned = xTaskCreate(parallel_display_demo_thread_entry,
                                       "parallel_display_demo_thread_entry",
                                       216,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &display_xHandle);
    if (xReturned != pdPASS) {

        vTaskDelete(display_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
