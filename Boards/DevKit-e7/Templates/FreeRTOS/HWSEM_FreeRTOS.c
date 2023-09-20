/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     HWSEM_FreeRTOS.c
 * @author   Khushboo Singh
 * @email    khushboo.singh@alifsemi.com
 * @version  V1.0.0
 * @date     27-June-2022
 * @brief    TestApp to verify HWSEM interface with FreeRTOS.
 *             It locks the HWSEM, sends the message through UART and then unlocks.
 *               This application will run on two cores
 *               to demonstrate the working of the Hardware Semaphore.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Includes --------------------------------------------------------------------------- */

/* System Includes */
#include <stdio.h>
#include <string.h>

/* Project Includes */

/* include for HwSem Driver */
#include "RTE_Components.h"
#include CMSIS_device_header

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* include for UART Driver */
#include "Driver_USART.h"
/* pin configuration driver */
#include <pinconf.h>
/* HWSEM Driver */
#include "Driver_HWSEM.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


#ifdef M55_HP
const char * startup_msg = "\n<<<<M55_HP : HWSEM testApp starting up>>>>\r\n";
const char * msg = "\nPrinting from M55_HP";
const char * acq_msg = "\nM55_HP acquiring the semaphore, printing 10 messages\r\n";
const char * rel_msg = "\nM55_HP releasing the semaphore\r\n\n";
#else
const char * startup_msg = "\n<<<<M55_HE : HWSEM testApp starting up>>>>\r\n";
const char * msg = "\nPrinting from M55_HE";
const char * acq_msg = "\nM55_HE acquiring the semaphore, printing 10 messages\r\n";
const char * rel_msg = "\nM55_HE releasing the semaphore\r\n\n";
#endif


/*Define for FreeRTOS*/
#define STACK_SIZE     1024
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH // 512
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE // 1024

StackType_t IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

#define UART_CB_TX_EVENT          1U << 0
#define UART_CB_RX_EVENT          1U << 1
#define UART_CB_RX_TIMEOUT        1U << 2

#define HWSEM_CB_EVENT            1U << 3

#define SEMID                     0

/* HWSEM Driver instance */
#define HWSEM                     0
/* Mention the Uart instance */
#define UART                      4

/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);

/* HWSEM Driver */
extern ARM_DRIVER_HWSEM ARM_Driver_HWSEM_(HWSEM);

/* UART Driver instance */
static ARM_DRIVER_HWSEM *HWSEMdrv = &ARM_Driver_HWSEM_(HWSEM);

TaskHandle_t Hwsem_xHandle;

void Hwsem_Thread(void *pvParameters);

/* ****************************** FreeRTOS functions **********************/
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
      StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &IdleTcb;
    *ppxIdleTaskStackBuffer = IdleStack;
    *pulIdleTaskStackSize = IDLE_TASK_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pxTask;

    for (;;);
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
      StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &TimerTcb;
    *ppxTimerTaskStackBuffer = TimerStack;
    *pulTimerTaskStackSize = TIMER_SERVICE_TASK_STACK_SIZE;
}

void vApplicationIdleHook(void)
{
    for (;;);
}
/*****************Only for FreeRTOS use *************************/

/**
 * @function    void myUART_callback(uint32_t event)
 * @brief       UART isr callabck
 * @note        none
 * @param       event: USART Event
 * @retval      none
 */
void myUART_callback(uint32_t event)
{
    if (event & ARM_USART_EVENT_SEND_COMPLETE)
    {
        /* Send Success */
        xTaskNotifyFromISR(Hwsem_xHandle, UART_CB_TX_EVENT, eSetBits, NULL);
    }

    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {

    }

    if (event & ARM_USART_EVENT_RX_TIMEOUT)
    {

    }
}

/**
 * @function   void HWSEM_callback(uint32_t event)
 * @brief      HWSEM isr callabck
 * @note       none
 * @param      event: HWSEM Event
 * @retval     none
 */
void myHWSEM_callback(int32_t event, uint8_t sem_id)
{
    if (event & HWSEM_AVAILABLE_CB_EVENT)
    {
        /* Success: Wakeup Thread */
        xTaskNotifyFromISR(Hwsem_xHandle,HWSEM_CB_EVENT, eSetBits, NULL);
   }
}

/**
 * @function    int32_t hardware_init(void)
 * @brief       Uart initialization
 * @note        none
 * @param       void
 * @retval      execution status
 */
int32_t hardware_init(void)
{
    int32_t ret = ARM_DRIVER_OK;
    ARM_DRIVER_VERSION version;

    /* UART4_RX_B */
    ret = pinconf_set(PORT_12, PIN_1, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_READ_ENABLE);

    if (ret)
    {
        return ARM_DRIVER_ERROR;
    }

    /* UART4_TX_B */
    ret = pinconf_set(PORT_12, PIN_2, PINMUX_ALTERNATE_FUNCTION_2, 0);

    if (ret)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Initialize UART driver */
    ret = USARTdrv->Initialize(myUART_callback);

    if(ret)
    {
        printf("\r\n Error in UART Initialize.\r\n");
        return ARM_DRIVER_ERROR;
    }

    /* Power up UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_FULL);

    if(ret)
    {
        printf("\r\n Error in UART Power Up.\r\n");
        goto error_uninitialize;
    }

    /* Configure UART */
    ret =  USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                                      ARM_USART_DATA_BITS_8 |
                                      ARM_USART_PARITY_NONE |
                                      ARM_USART_STOP_BITS_1 |
                                      ARM_USART_FLOW_CONTROL_NONE,
                                      115200);

    if(ret)
    {
        printf("\r\n Error in UART Control.\r\n");
        goto error_poweroff;
    }

    /* Enable Transmitter lines */
    ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);
    if(ret)
    {
        printf("\r\n Error in UART Control.\r\n");
        goto error_poweroff;
    }

    return ret;

error_poweroff:
    /* Received error Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Power OFF.\r\n");
    }

error_uninitialize:
    /* Received error Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Uninitialize.\r\n");
    }
    return ret;
}

/**
 * @function    void Hwsem_Thread(void *pvParameters)
 * @brief       TestApp to verify HWSEM interface
 *              Get the lock, send message through UART
 *              and then unlock
 * @note        none
 * @param       none
 * @retval      none
 */
void Hwsem_Thread(void *pvParameters)
{
    int32_t ret_hwsem, ret_uart, len, init = 1;
    char uart_msg[30];
    ARM_DRIVER_VERSION version;

    version = HWSEMdrv->GetVersion();
    printf("\r\n HWSEM version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize HWSEM driver */
    ret_hwsem = HWSEMdrv->Initialize(myHWSEM_callback);

    if(ret_hwsem != ARM_DRIVER_OK)
    {
        printf("\r\n HWSEM initialization failed\r\n");
        goto error_initialize;
    }

    while(1)
    {
        /* Acquire the lock */
        ret_hwsem = HWSEMdrv->TryLock();

        if(ret_hwsem == ARM_DRIVER_ERROR)
        {
            printf("\r\n HWSEM lock failed\r\n");
            goto error_lock;
        }

        /* If HWSEM already locked, then wait for the interrupt */
        while(ret_hwsem == ARM_DRIVER_ERROR_BUSY)
        {
            ret_hwsem = xTaskNotifyWait(NULL,HWSEM_CB_EVENT,NULL, portMAX_DELAY);

            if(ret_hwsem == pdTRUE)
            {
                /* Acquire the lock */
                ret_hwsem = HWSEMdrv->TryLock();
            }
            else
            {
                ret_hwsem = ARM_DRIVER_ERROR_BUSY;
            }
        }

        if (ret_hwsem != ARM_DRIVER_OK)
        {
            printf("\r\n HWSEM lock failed\n");
            goto error_lock;
        }

        /* Initialize the UART Driver */
        if(init)
        {
            ret_uart = hardware_init();
            if(ret_uart != 0)
            {
                printf("\r\n Error in UART hardware_init.\r\n");
                goto error_lock;
            }
            init = 0;
            ret_uart = USARTdrv->Send(startup_msg, strlen(startup_msg));
            if(ret_uart != ARM_DRIVER_OK)
            {
                printf("\r\nError in UART Send.\r\n");
            }

            /* wait for event flag after UART call */
            xTaskNotifyWait(NULL, UART_CB_TX_EVENT, NULL, portMAX_DELAY);
        }


        ret_uart = USARTdrv->Send(acq_msg, strlen(acq_msg));
        if(ret_uart != ARM_DRIVER_OK)
        {
            printf("\r\nError in UART Send.\r\n");
        }

        /* wait for event flag after UART call */
        xTaskNotifyWait(NULL, UART_CB_TX_EVENT, NULL, portMAX_DELAY);

        /* Print 10 messages from each core */
        for(int iter = 1; iter <= 10; iter++)
        {
            len = sprintf(uart_msg, "%s %d\r\n", msg, iter);

            ret_uart = USARTdrv->Send(uart_msg, len);
            if(ret_uart != ARM_DRIVER_OK)
            {
                printf("\r\nError in UART Send.\r\n");
            }

            /* wait for event flag after UART call */
            xTaskNotifyWait(NULL, UART_CB_TX_EVENT, NULL, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ret_uart = USARTdrv->Send(rel_msg, strlen(rel_msg));
        if(ret_uart != ARM_DRIVER_OK)
        {
            printf("\r\nError in UART Send.\r\n");
        }

        /* wait for event flag after UART call */
        xTaskNotifyWait(NULL, UART_CB_TX_EVENT, NULL, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Unlock the HW Semaphore */
        HWSEMdrv->Unlock();
        if(ret_hwsem == ARM_DRIVER_ERROR)
        {
            printf("\r\n HWSEM unlock failed\r\n");
            goto error_lock;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

    }

    /* Uninitialize the HWSEM Driver */
    HWSEMdrv->Uninitialize();

error_hw_init:
    /* Unlock the HW Semaphore */
    HWSEMdrv->Unlock();
error_lock:
    /* Uninitialize the HWSEM Driver */
    HWSEMdrv->Uninitialize();

error_initialize:
    return;
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start the FreeRTOS Kernel
 *---------------------------------------------------------------------------*/
int main(void)
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
    /* System Initialization */
    SystemCoreClockUpdate();

    /* Create application main thread */
    BaseType_t xReturned = xTaskCreate(Hwsem_Thread, "HwsemThread", 1024, NULL,configMAX_PRIORITIES-1, &Hwsem_xHandle);
    if (xReturned != pdPASS)
    {
        vTaskDelete(Hwsem_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}
