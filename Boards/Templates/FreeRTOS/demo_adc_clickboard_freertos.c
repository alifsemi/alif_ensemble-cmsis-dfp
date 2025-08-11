/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */
/*******************************************************************************
 * @file     : demo_adc_clickboard_freertos.c
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 12-FEB-2024
 * @brief    : Freertos demo application code click board analog input
 *              - Internal input analog signal corresponding output is digital value.
 *              - the input from the Click Board (CLICK_ANA) is internally connected to
 *                the ADC121 instance channel_0(j11 Pin 8).
 *              - the converted digital value are stored in user provided memory
 *                address.
 *
 *             Hardware Connection:
 *             Connect MIKROE Click board J29 and J30 for input.
 * @note       CLICK_ANA is then connected to the voltage level translator
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include "app_utils.h"

/* include Driver */
#include "Driver_ADC.h"
#include "board_config.h"

/* Rtos include */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "se_services_port.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/* single shot conversion scan use ARM_ADC_SINGLE_SHOT_CH_CONV*/

#define ADC_CONVERSION ARM_ADC_SINGLE_SHOT_CH_CONV

/* Instance for ADC12 */
extern ARM_DRIVER_ADC  ARM_Driver_ADC12(BOARD_CLICKBOARD_ANA_ADC12_INSTANCE);
static ARM_DRIVER_ADC *pxADCDrv = &ARM_Driver_ADC12(BOARD_CLICKBOARD_ANA_ADC12_INSTANCE);

#define CLICK_BOARD_INPUT ARM_ADC_CHANNEL_0
#define NUM_CHANNELS      (8)

static void prvAdcPotentiometerDemo(void *pvParameters);

/*Define for FreeRTOS*/
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

TaskHandle_t adc_xHandle;

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

#define ADC_INT_AVG_SAMPLE_RDY 0x01

/* Demo purpose adc_sample*/
uint32_t ulAdcSample[NUM_CHANNELS];

volatile uint32_t ulNumSamples = 0;

/*
 * @func   : static void prvAdcConversionCallBack(uint32_t ulEvent,
 *                                   uint8_t  ucChannel,
 *                                   uint32_t ulSampleOutput)
 * @brief  : adc conversion isr callback
 * @return : NONE
 */
static void prvAdcConversionCallBack(uint32_t ulEvent, uint8_t ucChannel, uint32_t ulSampleOutput)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (ulEvent & ARM_ADC_EVENT_CONVERSION_COMPLETE) {
        ulNumSamples           += 1;

        /* Store the value for the respected channels */
        ulAdcSample[ucChannel]  = ulSampleOutput;
        /* Conversion Completed */
        xResult                 = xTaskNotifyFromISR(adc_xHandle,
                                     ADC_INT_AVG_SAMPLE_RDY,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 *    @func   : static void prvAdcPotentiometerDemo( void *pvParameters )
 *    @brief  : ADC Click Board demo
 *             - test to verify the click board analog input of adc.
 *             - Internal input of click board in analog signal corresponding
 *               output is digital value.
 *             - converted value is the allocated user memory address.
 *    @return : NONE
 */
static void prvAdcPotentiometerDemo(void *pvParameters)
{
    int32_t            lRet        = 0;
    uint32_t           ulErrorCode = SERVICES_REQ_SUCCESS;
    uint32_t           ulServiceErrorCode;
    float              fTemp;
    ARM_DRIVER_VERSION xVersion;

    /* Initialize the SE services */
    se_services_port_init();

    /* enable the 160 MHz clock */
    ulErrorCode = SERVICES_clocks_enable_clock(se_services_s_handle,
                                               /*clock_enable_t*/ CLKEN_CLK_160M,
                                               /*bool enable   */ true,
                                               &ulServiceErrorCode);
    if (ulErrorCode) {
        printf("SE Error: 160 MHz clk enable = %d\n", ulErrorCode);
        return;
    }

    printf("\t\t\n >>> ADC demo starting up!!! <<< \r\n");

    xVersion = pxADCDrv->GetVersion();
    printf("\r\n ADC version api:%X driver:%X...\r\n", xVersion.api, xVersion.drv);

    /* pin mux and configuration for all device IOs requested from pins.h*/
    lRet = board_pins_config();
    if (lRet != 0) {
        printf("ERROR: Board pin configuration failed: %d\n", lRet);
        return;
    }

    /* Initialize ADC driver */
    lRet = pxADCDrv->Initialize(prvAdcConversionCallBack);
    if (lRet != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC init failed\n");
        return;
    }

    /* Power control ADC */
    lRet = pxADCDrv->PowerControl(ARM_POWER_FULL);
    if (lRet != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Power up failed\n");
        goto error_uninitialize;
    }

    /* set conversion mode */
    lRet = pxADCDrv->Control(ARM_ADC_CONVERSION_MODE_CTRL, ADC_CONVERSION);
    if (lRet != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC select conversion mode failed\n");
        goto error_poweroff;
    }

    /* set initial channel */
    lRet = pxADCDrv->Control(ARM_ADC_CHANNEL_INIT_VAL, CLICK_BOARD_INPUT);
    if (lRet != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC channel init failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%X <<<\n",
           (uint32_t) (ulAdcSample + CLICK_BOARD_INPUT));
    /* Start ADC */
    lRet = pxADCDrv->Start();
    if (lRet != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Start failed\n");
        goto error_poweroff;
    }

    /* wait for timeout */
    while (ulNumSamples < 1) {
    }

    /* wait till conversion comes ( isr callback ) */
    if (xTaskNotifyWait(NULL, ADC_INT_AVG_SAMPLE_RDY, NULL, portMAX_DELAY) != pdFALSE) {
        /* Stop ADC */
        lRet = pxADCDrv->Stop();
        if (lRet != ARM_DRIVER_OK) {
            printf("\r\n Error: ADC stop failed\n");
            goto error_poweroff;
        }
        printf("\n >>> ADC conversion completed \n");
        printf(" Converted value are stored in user allocated memory address.\n");
    } else {
        printf("\n Error: ADC conversion Failed \n");
    }

    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_poweroff:

    /* Power off ADC peripheral */
    lRet = pxADCDrv->PowerControl(ARM_POWER_OFF);
    if (lRet != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize ADC driver */
    lRet = pxADCDrv->Uninitialize();
    if (lRet != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Uninitialize failed.\r\n");
    }
    /* disable the 160MHz clock */
    ulErrorCode = SERVICES_clocks_enable_clock(se_services_s_handle,
                                               /*clock_enable_t*/ CLKEN_CLK_160M,
                                               /*bool enable   */ false,
                                               &ulServiceErrorCode);
    if (ulErrorCode) {
        printf("SE Error: 160 MHz clk disable = %d\n", ulErrorCode);
        return;
    }

    printf("\r\n ADC demo exiting...\r\n");

    /* thread delete */
    vTaskDelete(NULL);
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start the FreeRTOS Kernel
 *---------------------------------------------------------------------------*/
int main(void)
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
    BaseType_t xReturned = xTaskCreate(prvAdcPotentiometerDemo,
                                       "prvAdcPotentiometerDemo",
                                       256,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &adc_xHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(adc_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}
