/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     : demo_i2c_freertos.c
 * @brief    : FreeRtos demo application to verify I2C Master and
 *             Slave functionality with FreeRtos as an operating system
 *
 *             Code will verify below cases:
 *             1) Master transmit 30 bytes and Slave receive 30 bytes
 *             2) Slave transmit 29 bytes and Master receive 29 bytes
 *             I2C1 instance is taken as Master (PIN P7_2 and P7_3)
 *             I2C0 instance is taken as Slave  (PIN P3_5 and P3_4)
 *
 *             E7: Hardware setup:
 *             - Connecting GPIO pins of I2C1 TO I2C0
 *               SDA pin P7_2(J15) to P3_5(J11)
 *               SCL pin P7_3(J15) to P3_4(J11).
 *             E1C: Hardware setup:
 *             - Connecting GPIO pins of I2C1 TO I2C0
 *               SDA pin P7_2(J14) to P7_0(J14)
 *               SCL pin P7_3(J14) to P7_1(J14).
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "app_utils.h"
#include "board_config.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "event_groups.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/* Defining Address modes */
#define ADDRESS_MODE_7BIT  1                 /* I2C 7 bit addressing mode     */
#define ADDRESS_MODE_10BIT 2                 /* I2C 10 bit addressing mode    */
#define ADDRESS_MODE       ADDRESS_MODE_7BIT /* 7 bit addressing mode chosen  */

#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
#define TAR_ADDRS (0X2D0) /* 10 bit Target(Slave) Address, use by Master */
#define SAR_ADDRS (0X2D0) /* 10 bit Slave Own Address,     use by Slave  */
#else
#define TAR_ADDRS (0X40) /* 7 bit Target(Slave) Address, use by Master  */
#define SAR_ADDRS (0X40) /* 7 bit Slave Own Address,     use by Slave   */
#endif

/* Communication commands*/
#define STOP                          (0X00)

/*Define for FreeRTOS notification objects */
#define I2C_MST_TRANSFER_DONE         0x01
#define I2C_SLV_TRANSFER_DONE         0x02
#define I2C_TWO_WAY_TRANSFER_DONE     (I2C_MST_TRANSFER_DONE | I2C_SLV_TRANSFER_DONE)
/*Define for FreeRTOS*/
#define TASK_STACK_SIZE               512
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE
static StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
static StaticTask_t IdleTcb;
static StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
static StaticTask_t TimerTcb;

/* I2C Driver instance */
extern ARM_DRIVER_I2C  ARM_Driver_I2C_(BOARD_MASTER_I2C_INSTANCE);
static ARM_DRIVER_I2C *I2C_MstDrv = &ARM_Driver_I2C_(BOARD_MASTER_I2C_INSTANCE);

extern ARM_DRIVER_I2C  ARM_Driver_I2C_(BOARD_SLAVE_I2C_INSTANCE);
static ARM_DRIVER_I2C *I2C_SlvDrv = &ARM_Driver_I2C_(BOARD_SLAVE_I2C_INSTANCE);

/* Task handle */
static TaskHandle_t i2c_xHandle;
/* Event group Handle */
static EventGroupHandle_t i2c_event_group;

#if (RTE_I2C0_DMA_ENABLE || RTE_I2C1_DMA_ENABLE)
#define I2C_DMA_ENABLED 1
#else
#define I2C_DMA_ENABLED 0
#endif

#if I2C_DMA_ENABLED
/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT 11

/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT 10

/* Tx and Rx buffers are in multiples of 2bytes
 * as the DMA processes in 2bytes fashion only */
/* Master parameter set */
/* Master TX Data */
static uint16_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT] = {
    /* MST_TX_BUF data = "Master_Data" */
    77,
    97,
    115,
    116,
    101,
    114,
    95,
    68,
    97,
    116,
    97};

/* master receive buffer */
static uint16_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT];
/* Master parameter set END  */

/* Slave parameter set */
/* slave receive buffer */
static uint16_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT];

/* Slave TX Data */
static uint16_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT] = {
    /* SLV_TX_BUF data =  "Slave_Data" */
    83,
    108,
    97,
    118,
    101,
    95,
    68,
    97,
    116,
    97};
/* Slave parameter set END */
#else

/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT 30

/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT 29
/* Master parameter set */

/* Master TX Data (Any random value). */
static uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT] = {"!*!Test Message from Master!*!"};

/* master receive buffer */
static uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT];
/* Master parameter set END  */

/* Slave parameter set */
/* slave receive buffer */
static uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT];

/* Slave TX Data */
static uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT] = {"!*!Test Message from Slave!*!"};
/* Slave parameter set END */
#endif

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
    ARG_UNUSED(pxTask);
    ARG_UNUSED(pcTaskName);
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

/**
 * @fn      static void i2c_mst_transfer_callback(uint32_t event)
 * @brief   I2C master event callback
 * @note    none
 * @param   event : Callback Event
 * @retval  none
 */
static void i2c_mst_transfer_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer or receive is finished */
        xResult = xEventGroupSetBitsFromISR(i2c_event_group,
                                            I2C_MST_TRANSFER_DONE,
                                            &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @fn      static void i2c_slv_transfer_callback(uint32_t event)
 * @brief   I2C slave event callback
 * @note    none
 * @param   event : Callback Event
 * @retval  none
 */
static void i2c_slv_transfer_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer or receive is finished */
        xResult = xEventGroupSetBitsFromISR(i2c_event_group,
                                            I2C_SLV_TRANSFER_DONE,
                                            &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @fn      static void I2C_Thread(void *pvParameters)
 * @brief   I2C communication task
 * @note    none
 * @param   pvParameters : Task Parameter
 * @retval  none
 */
static void I2C_Thread(void *pvParameters)
{
    int                ret = 0;
    ARM_DRIVER_VERSION version;
    EventBits_t        events = 0;
    ARG_UNUSED(pvParameters);

    printf("\r\n >>> I2C demo Thread starting up!!! <<< \r\n");

    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }

    version = I2C_MstDrv->GetVersion();
    printf("\r\n I2C version api:0x%X driver:0x%X...\r\n", version.api, version.drv);

    /* Initialize Master I2C driver */
    ret = I2C_MstDrv->Initialize(i2c_mst_transfer_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C master init failed\n");
        return;
    }

    /* Initialize Slave I2C driver */
    ret = I2C_SlvDrv->Initialize(i2c_slv_transfer_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C slave init failed\n");
        return;
    }

    /* I2C Master Power control  */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Master Power up failed\n");
        goto error_uninitialize;
    }

    /* I2C Slave Power control */
    ret = I2C_SlvDrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Slave Power up failed\n");
        goto error_uninitialize;
    }

    /* I2C Master Control */
    ret = I2C_MstDrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Master Control failed\n");
        goto error_poweroff;
    }

    /* I2C Slave Control */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS, (SAR_ADDRS | ARM_I2C_ADDRESS_10BIT));
#else
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS, SAR_ADDRS);
#endif
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Slave Control failed\n");
        goto error_poweroff;
    }

    printf("\n---------Master transmit/slave receive---------\n");
    /* I2C Slave Receive */
    ret = I2C_SlvDrv->SlaveReceive((uint8_t *) SLV_RX_BUF, MST_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Slave Receive failed\n");
        goto error_poweroff;
    }

    /* I2C Master Transmit */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    I2C_MstDrv->MasterTransmit((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                               (uint8_t *) MST_TX_BUF,
                               MST_BYTE_TO_TRANSMIT,
                               STOP);
#else
    I2C_MstDrv->MasterTransmit(TAR_ADDRS, (uint8_t *) MST_TX_BUF, MST_BYTE_TO_TRANSMIT, STOP);
#endif
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Master Transmit failed\n");
        goto error_poweroff;
    }

    /* wait for 2-way communication to complete */
    events = xEventGroupWaitBits(i2c_event_group,
                                 I2C_TWO_WAY_TRANSFER_DONE,
                                 pdTRUE,
                                 pdTRUE,
                                 portMAX_DELAY);

    if (events != I2C_TWO_WAY_TRANSFER_DONE) {
        printf("Error: Master transmit/slave receive failed \n");
    }

    /* Compare received data. */
#if I2C_DMA_ENABLED
    if (memcmp(&SLV_RX_BUF, &MST_TX_BUF, (MST_BYTE_TO_TRANSMIT * 2)))
#else
    if (memcmp(&SLV_RX_BUF, &MST_TX_BUF, MST_BYTE_TO_TRANSMIT))
#endif
    {
        printf("\n Error: Master transmit/slave receive failed \n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        WAIT_FOREVER_LOOP
    }

    printf("\n---------Master receive/slave transmit---------\n");
    /* I2C Slave Transmit */
    ret = I2C_SlvDrv->SlaveTransmit((uint8_t *) SLV_TX_BUF, SLV_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Slave Transmit failed\n");
        goto error_poweroff;
    }

    /* I2C Master Receive */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_MstDrv->MasterReceive((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                                    (uint8_t *) MST_RX_BUF,
                                    SLV_BYTE_TO_TRANSMIT,
                                    STOP);
#else
    ret = I2C_MstDrv->MasterReceive(TAR_ADDRS, (uint8_t *) MST_RX_BUF, SLV_BYTE_TO_TRANSMIT, STOP);
#endif
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Master Receive failed\n");
        goto error_poweroff;
    }

    /* wait for 2-way communication to complete */
    events = xEventGroupWaitBits(i2c_event_group,
                                 I2C_TWO_WAY_TRANSFER_DONE,
                                 pdTRUE,
                                 pdTRUE,
                                 portMAX_DELAY);

    if (events != I2C_TWO_WAY_TRANSFER_DONE) {
        printf("Error: Master receive/slave transmit failed \n");
    }

    /* Compare received data. */
#if I2C_DMA_ENABLED
    if (memcmp(&SLV_TX_BUF, &MST_RX_BUF, (SLV_BYTE_TO_TRANSMIT * 2)))
#else
    if (memcmp(&SLV_TX_BUF, &MST_RX_BUF, SLV_BYTE_TO_TRANSMIT))
#endif
    {
        printf("\n Error: Master receive/slave transmit failed\n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        WAIT_FOREVER_LOOP
    }
    printf("\n >>> I2C Communication completed without any error <<< \n");

error_poweroff:
    /* Power off I2C peripheral */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Master Power OFF failed\r\n");
    }

    ret = I2C_SlvDrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Slave Power OFF failed\r\n");
    }

error_uninitialize:
    /* Un-initialize I2C driver */
    ret = I2C_MstDrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n I2C Master Uninitialize failed\r\n");
    }

    ret = I2C_SlvDrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n I2C Slave Uninitialize failed\r\n");
    }

    printf("\r\n >>> I2C demo thread exiting <<<\r\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP
}

/**
 * @fn      int main(void)
 * @brief   Entry point for I2C comm
 * @note    none
 * @param   none
 * @retval  none
 */
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
    BaseType_t xReturned = xTaskCreate(I2C_Thread,
                                       "I2C_Thread",
                                       TASK_STACK_SIZE,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &i2c_xHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(i2c_xHandle);
        return -1;
    }

    /* Create an event group */
    i2c_event_group = xEventGroupCreate();

    /* Start thread execution */
    vTaskStartScheduler();
}
