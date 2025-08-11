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
 * @file     : demo_dac_freertos.c
 * @author   : Nisarga A M
 * @email    : nisarga.am@alifsemi.com
 * @version  : V1.0.0
 * @date     : 06-June-2023
 * @brief    : TestApp to verify DAC(Digital to Analog converter) interface using
 *             FreeRTOS as an operating system.
 *             -As DAC is 12 bit resolution, If the input value is maximum than the
                  maximum DAC input value(0xFFF)then the input value will be set to
                  DAC maximum input value.
               -And If the input value is equal to maximum dac input value then
                  input value will be set to 0.
               -If the input value is not greater than the maximum dac input value
                 then input value will be incremented by 1000.

               E7: Hardware Setup :
                -when the application uses DAC0 channel,then in Engineering board
                 connect DAC0 to P2_2 pin,according to DAC input the output will be
                 observed in P2_2 pin through the logic analyzer.

                -And when the application uses DAC1 channel,then in Engineering board
                 connect DAC1 to P2_3 pin,according to DAC input the output will be
                 observed in P2_3 pin through the logic analyzer.
               E1C: Hardware Setup :
                - Connect the logic analyzer to pin P2_3 to observe the DAC0 output.
                  By varying the DAC input values, the output will be displayed on
                  pin P2_3.
 ******************************************************************************/
/* System Includes */
#include <stdio.h>

#include "RTE_Components.h"
#include CMSIS_device_header

/* Project Includes */
/* include for DAC Driver */
#include "Driver_DAC.h"
#include "pinconf.h"
#include "board_config.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

// Set to 0: Use application-defined DAC12 pin configuration (via board_dac12_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* DAC Driver instance */
extern ARM_DRIVER_DAC  ARM_Driver_DAC_(BOARD_DAC12_INSTANCE);
static ARM_DRIVER_DAC *DACdrv = &ARM_Driver_DAC_(BOARD_DAC12_INSTANCE);

/*Define for FreeRTOS*/
#define STACK_SIZE                    1024
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

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

TaskHandle_t dac_xHandle = NULL;

/* DAC maximum resolution is 12-bit */
#define DAC_MAX_INPUT_VALUE (0xFFF)

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_dac12_pins_config(void)
 * @brief   Configure DAC12 pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_dac12_pins_config(void)
{
    int32_t status;

    /* Configure DAC120 output */
    status = pinconf_set(PORT_(BOARD_DAC120_OUT_GPIO_PORT),
                         BOARD_DAC120_OUT_GPIO_PIN,
                         BOARD_DAC120_ALTERNATE_FUNCTION,
                         PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return status;
    }

#if (RTE_DAC1)
    /* Configure DAC121 output */
    status = pinconf_set(PORT_(BOARD_DAC121_OUT_GPIO_PORT),
                         BOARD_DAC121_OUT_GPIO_PIN,
                         BOARD_DAC121_ALTERNATE_FUNCTION,
                         PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return status;
    }
#endif
    return APP_SUCCESS;
}
#endif

/**
 @fn           void dac_demo_Thread_entry()
 @brief        DAC demo :
               This initializes the DAC. And then in a loop,
               according to the input value, the output will change.
               If the input value is maximum than the maximum DAC input
               value then the input value will be set to DAC maximum input value.
               And If the input value is equal to maximum dac input value then
               input value will be set to 0.If the input value is not greater
               than the maximum dac input value then input value will be incremented by 1000.
 @return       none
*/
void dac_demo_Thread_entry()
{
    uint32_t           input_value = 0;
    int32_t            ret         = 0;
    ARM_DRIVER_VERSION version;
    const TickType_t   xDelay = (1 / portTICK_PERIOD_MS); /* delay for 1 MS */

    printf("\r\n >>> DAC demo starting up!!! <<< \r\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }

#else
    /*
     * NOTE: The DAC12 pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_dac12_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }
#endif

    version = DACdrv->GetVersion();
    printf("\r\n DAC version api:%X driver:%X...\r\n", version.api, version.drv);

    /* Initialize DAC driver */
    ret = DACdrv->Initialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC init failed\n");
        return;
    }

    /* Enable the power for DAC */
    ret = DACdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Power up failed\n");
        goto error_uninitialize;
    }

    /* Set DAC IBAIS output current */
    ret = DACdrv->Control(ARM_DAC_SELECT_IBIAS_OUTPUT, ARM_DAC_1500UA_OUT_CUR);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Setting DAC output current failed failed\n");
        goto error_poweroff;
    }

    /* Set DAC capacitance  */
    ret = DACdrv->Control(ARM_DAC_CAPACITANCE_HP_MODE, ARM_DAC_8PF_CAPACITANCE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Setting DAC capacitance failed\n");
        goto error_poweroff;
    }

    /* start dac */
    ret = DACdrv->Start();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Start failed\n");
        goto error_poweroff;
    }

    input_value = 0;

    while (1) {
        /* set dac input */
        ret = DACdrv->SetInput(input_value);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: DAC Set Input failed\n");
            goto error_stop;
        }

        /* Sleep for 1 ms */
        vTaskDelay(xDelay);

        /* If the input value is equal to maximum dac input value then input
           value will be set to 0 */
        if (input_value == DAC_MAX_INPUT_VALUE) {
            input_value = 0;
        } else {
            /* If the input value is not greater than the maximum dac input value then input
               value will be incremented by 1000 */
            input_value += 1000;
        }

        /* If the input value is maximum than the maximum DAC input value then the input
           value will be set to DAC maximum input value */
        if (input_value > DAC_MAX_INPUT_VALUE) {
            input_value = DAC_MAX_INPUT_VALUE;
        }
    }

error_stop:

    /* Stop the DAC driver */
    ret = DACdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Stop failed.\r\n");
    }

error_poweroff:

    /* Power off DAC peripheral */
    ret = DACdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize DAC driver */
    ret = DACdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Uninitialize failed.\r\n");
    }

    printf("\r\n XXX DAC demo exiting XXX...\r\n");
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

    /* System Initialization */
    SystemCoreClockUpdate();

    /* Create application main thread */
    BaseType_t xReturned = xTaskCreate(dac_demo_Thread_entry,
                                       "DACFreertos",
                                       256,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &dac_xHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(dac_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}

/********************** (c) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
