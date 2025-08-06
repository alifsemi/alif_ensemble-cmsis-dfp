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
 * @file     : demo_lpuart_freertos.c
 * @author   : Tanay Rami
 * @email    : tanay@alifsemi.com
 * @version  : V1.0.0
 * @date     : 27-May-2023
 * @brief    : Taken Reference from "uart4_testapp.c"
 * @bug      : None.
 * @Note     : Updated for B0 PinMux.
 ******************************************************************************/

/* Includes ----------------------------------------------------------------- */

/* System Includes */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Project Includes */
/* include for UART Driver */
#include "Driver_USART.h"
#include "pinconf.h"

/*RTOS Includes*/
#include "RTE_Components.h"
#include CMSIS_device_header
#include "board_config.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

#if !defined(RTSS_HE)
#error "This Demo application works only on RTSS_HE"
#endif

// Set to 0: Use application-defined lpuart pin configuration (via board_lpuart_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/*Define for FreeRTOS*/
#define STACK_SIZE                     1024
#define TIMER_SERVICE_TASK_STACK_SIZE  configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE           configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

/* LPUART Driver */
#define UART LP
/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);

/*Define for FreeRTOS objects */

#define UART_CB_TX_EVENT         (1U << 0)
#define UART_CB_RX_EVENT         (1U << 1)
#define UART_CB_RX_TIMEOUT       (1U << 2)
#define UART_CB_RX_BREAK         (1U << 3)
#define UART_CB_RX_FRAMING_ERROR (1U << 4)
#define UART_CB_RX_PARITY_ERROR  (1U << 5)
#define UART_CB_RX_OVERFLOW      (1U << 6)

TaskHandle_t Uart_xHandle;

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

/**
 * @function   void UART_callback(uint32_t event)
 * @brief      UART isr callabck
 * @note       none
 * @param      event: USART Event
 * @retval     none
 */
void myUART_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_USART_EVENT_SEND_COMPLETE) {
        /* Send Success: Wakeup Thread */
        xResult =
            xTaskNotifyFromISR(Uart_xHandle, UART_CB_TX_EVENT, eSetBits, &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        /* Receive Success: Wakeup Thread */
        xResult =
            xTaskNotifyFromISR(Uart_xHandle, UART_CB_RX_EVENT, eSetBits, &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_USART_EVENT_RX_TIMEOUT) {
        /* Receive Timeout: Wakeup Thread */
        xResult = xTaskNotifyFromISR(Uart_xHandle,
                                     UART_CB_RX_TIMEOUT,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_USART_EVENT_RX_BREAK) {
        /* Receive Break: Wakeup Thread */
        xResult =
            xTaskNotifyFromISR(Uart_xHandle, UART_CB_RX_BREAK, eSetBits, &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_USART_EVENT_RX_FRAMING_ERROR) {
        /* Receive Framing Error: Wakeup Thread */
        xResult = xTaskNotifyFromISR(Uart_xHandle,
                                     UART_CB_RX_FRAMING_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_USART_EVENT_RX_PARITY_ERROR) {
        /* Receive Parity Error: Wakeup Thread */
        xResult = xTaskNotifyFromISR(Uart_xHandle,
                                     UART_CB_RX_PARITY_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_USART_EVENT_RX_OVERFLOW) {
        /* Receive Overflow: Wakeup Thread */
        xResult = xTaskNotifyFromISR(Uart_xHandle,
                                     UART_CB_RX_OVERFLOW,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_lpuart_pins_config(void)
 * @brief   Configure additional lpuart pinmux settings not handled
 *          by the board support library.
 * @retval  execution status.
 */
int board_lpuart_pins_config(void)
{
    int32_t ret;

    /* LPUART_RX_A */
    ret = pinconf_set(PORT_(BOARD_LPUART_RX_GPIO_PORT),
                      BOARD_LPUART_RX_GPIO_PIN,
                      BOARD_LPUART_RX_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret) {
        return ret;
    }

    /* LPUART_TX_A */
    ret = pinconf_set(PORT_(BOARD_LPUART_TX_GPIO_PORT),
                      BOARD_LPUART_TX_GPIO_PIN,
                      BOARD_LPUART_TX_ALTERNATE_FUNCTION,
                      0);
    if (ret) {
        return ret;
    }

    return ret;
}
#endif

void Uart_Thread(void *pvParameters)
{
    uint8_t            cmd    = 0;
    uint32_t           ret    = 0;
    uint32_t           events = 0;
    uint32_t           ulNotificationValue;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> UART testApp starting up!!!...<<< \r\n");

    version = USARTdrv->GetVersion();
    printf("\r\n UART version api:%X driver:%X...\r\n", version.api, version.drv);

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }

#else
    /*
     * NOTE: The lpuart pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_lpuart_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }
#endif

    /* Initialize UART driver */
    ret = USARTdrv->Initialize(myUART_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Initialize.\r\n");
        return;
    }

    /* Power up UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Power Up.\r\n");
        goto error_uninitialize;
    }

    /* Configure UART to 115200 Bits/sec */
    ret = USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 |
                                ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 |
                                ARM_USART_FLOW_CONTROL_NONE,
                            115200);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Control.\r\n");
        goto error_poweroff;
    }

    /* Enable Receiver and Transmitter lines */
    ret = USARTdrv->Control(ARM_USART_CONTROL_TX, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Control TX.\r\n");
        goto error_poweroff;
    }

    ret = USARTdrv->Control(ARM_USART_CONTROL_RX, 1);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Control RX.\r\n");
        goto error_poweroff;
    }

    printf("\r\n Press Enter or any character on serial terminal to receive a message:\r\n");

    ret = USARTdrv->Send("\r\nPress Enter or any character to receive a message\r\n", 53);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Send.\r\n");
        goto error_poweroff;
    }
    xTaskNotifyWait(NULL, UART_CB_TX_EVENT, NULL, portMAX_DELAY);

    while (1) {
        /* Get byte from UART */
        ret = USARTdrv->Receive(&cmd, 1);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error in UART Receive.\r\n");

            goto error_poweroff;
        }

        /* wait till Receive complete or Receive timeout event comes in isr callback */
        xTaskNotifyWait(NULL,
                        (UART_CB_RX_EVENT | UART_CB_RX_TIMEOUT | UART_CB_RX_BREAK),
                        &ulNotificationValue,
                        portMAX_DELAY);

        /* check for any RX error. */
        if (ulNotificationValue & (UART_CB_RX_BREAK | UART_CB_RX_FRAMING_ERROR |
                                   UART_CB_RX_PARITY_ERROR | UART_CB_RX_OVERFLOW)) {
            /* clear error flags. */
            if (ulNotificationValue & (UART_CB_RX_BREAK)) {
                printf("\r\n RX Break sequence detected.\r\n");
                ulTaskNotifyValueClear(Uart_xHandle, UART_CB_RX_BREAK);
            }

            if (ulNotificationValue & (UART_CB_RX_FRAMING_ERROR)) {
                printf("\r\n RX Framing Error.\r\n");
                ulTaskNotifyValueClear(Uart_xHandle, UART_CB_RX_FRAMING_ERROR);
            }

            if (ulNotificationValue & (UART_CB_RX_PARITY_ERROR)) {
                printf("\r\n RX Parity Error.\r\n");
                ulTaskNotifyValueClear(Uart_xHandle, UART_CB_RX_PARITY_ERROR);
            }

            if (ulNotificationValue & (UART_CB_RX_OVERFLOW)) {
                printf("\r\n RX Overflow Error.\r\n");
                ulTaskNotifyValueClear(Uart_xHandle, UART_CB_RX_OVERFLOW);
            }
        }

        if (ulNotificationValue & (UART_CB_RX_EVENT)) {
            if (cmd == 13) /* CR, send greeting  */ {
                USARTdrv->Send("\r\nHello World!\r\n", 16);
            } else /* else send back received character. */ {
                USARTdrv->Send(&cmd, 1);
            }
            xTaskNotifyWait(NULL, UART_CB_TX_EVENT, NULL, portMAX_DELAY);
        }
    }

    printf("\r\n XXX UART demo thread exiting XXX...\r\n");

error_poweroff:

    /* Received error Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Power OFF.\r\n");
    }

error_uninitialize:

    /* Received error Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in UART Uninitialize.\r\n");
    }

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
    BaseType_t xReturned =
        xTaskCreate(Uart_Thread, "UartThread", 256, NULL, configMAX_PRIORITIES - 1, &Uart_xHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(Uart_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}
