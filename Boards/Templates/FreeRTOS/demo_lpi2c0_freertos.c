/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/******************************************************************************
 * @file     : demo_lpi2c0_freertos.c
 * @author   : Shreehari H K
 * @email    : shreehari.hk@alifsemi.com
 * @version  : V1.0.0
 * @date     : 05-Sept-2024
 * @brief    : TestApp to verify I2C Master and LPI2C0 Slave functionality
 *             using FreeRTOS without any operating system.
 * @bug      : None
 * @note     : Code will verify:
 *              1.)Master transmit and Slave receive
 *              2.)Master receive  and Slave transmit
 *                  I2C0 instance is taken as Master and
 *                  LPI2C0(Slave-only) instance is taken as Slave.
 *
 *             Hardware Connection:
 *             I2C0 SDA(P3_5) -> LPI2C0 SDA(P5_3)
 *             I2C0 SCL(P3_4) -> LPI2C0 SCL(P5_2)
 ******************************************************************************/
/* Include */
#include <stdio.h>
#include <string.h>

/* RTOS include */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "pinconf.h"
#include "board_config.h"

#if !defined(RTSS_HE)
#error "This Demo application works only on RTSS_HE"
#endif

// Set to 0: Use application-defined lpi2c0 pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

/* I2C Driver instance */
extern ARM_DRIVER_I2C  Driver_I2C0;
static ARM_DRIVER_I2C *I2C_mstdrv = &Driver_I2C0;

extern ARM_DRIVER_I2C  Driver_LPI2C0;
static ARM_DRIVER_I2C *LPI2C_slvdrv = &Driver_LPI2C0;

/*Define for FreeRTOS*/
#define STACK_SIZE                    512
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE
#define MASTER_TASK_PRIORITY          2U
#define SLAVE_TASK_PRIORITY           (MASTER_TASK_PRIORITY - 1U)

/* LPI2C0 callback events */
typedef enum _LPI2C_CB_EVENT {
    LPI2C_CB_EVENT_SUCCESS = (1 << 0),
    LPI2C_CB_EVENT_ERROR   = (1 << 1)
} LPI2C_CB_EVENT;

static StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
static StaticTask_t IdleTcb;
static StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
static StaticTask_t TimerTcb;

static TaskHandle_t master_taskHandle;
static TaskHandle_t slave_taskHandle;

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

#define TAR_ADDRS            (0x40) /* Target(Slave) Address, use by Master */
#define RESTART              (0x01)
#define STOP                 (0x00)

/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT 21

/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT 22

/* Master parameter set */

/* Master TX Data (Any random value). */
static uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT + 1] = {"Test_Message_to_Slave"};

/* master receive buffer */
static uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT + 1];

/* Master parameter set END  */

/* Slave parameter set */

/* slave receive buffer */
static uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT + 1];

/* Slave TX Data (Any random value). */
static uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT + 1] = {"Test_Message_to_Master"};

/* Slave parameter set END */

/**
 * @fn      static void i2c_mst_tranfer_callback(uint32_t event)
 * @brief   I2C Callback function for events
 * @note    none
 * @param   event: I2C event
 * @retval  none
 */
static void i2c_mst_tranfer_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t xResult                  = pdFALSE;

    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer Success */
        xResult = xTaskNotifyFromISR(master_taskHandle,
                                     LPI2C_CB_EVENT_SUCCESS,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else {
        /* Transfer Error */
        xResult = xTaskNotifyFromISR(master_taskHandle,
                                     LPI2C_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @fn      static void i2c_slv_transfer_callback(uint32_t event)
 * @brief   LPI2C Callback function for events
 * @note    none
 * @param   event: LPI2C event
 * @retval  none
 */
static void i2c_slv_transfer_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t xResult                  = pdFALSE;

    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer Success */
        xResult = xTaskNotifyFromISR(slave_taskHandle,
                                     LPI2C_CB_EVENT_SUCCESS,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else {
        /* Transfer Error */
        xResult = xTaskNotifyFromISR(slave_taskHandle,
                                     LPI2C_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_lpi2c_pins_config(void)
 * @brief   Configure lpi2c pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_lpi2c_pins_config(void)
{
    int32_t ret;

#if BOARD_LPI2C0_USE_FLEXIO_PINS
    /* for LPI2C0 Rev-A GPIO voltage level(flex)
     * has to be changed to 1.8-V power supply.
     * configure flexio pins to 1.8V
     */
    uint32_t      error_code = SERVICES_REQ_SUCCESS;
    uint32_t      service_error_code;
    run_profile_t runp;

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
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("Set new run config failed\n");
        WAIT_FOREVER_LOOP
    }
#endif

    /* LPI2C0_SDA */
    ret = pinconf_set(PORT_(BOARD_LPI2C0_SDA_GPIO_PORT),
                      BOARD_LPI2C0_SDA_GPIO_PIN,
                      BOARD_LPI2C0_SDA_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPI2C0_SDA_PIN\n");
        return ret;
    }

    /* LPI2C0_SCL */
    ret = pinconf_set(PORT_(BOARD_LPI2C0_SCL_GPIO_PORT),
                      BOARD_LPI2C0_SCL_GPIO_PIN,
                      BOARD_LPI2C0_SCL_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPI2C0_SCL_PIN\n");
        return ret;
    }

    /* I2C0_SDA */
    ret = pinconf_set(PORT_(BOARD_I2C0_SDA_GPIO_PORT),
                      BOARD_I2C0_SDA_GPIO_PIN,
                      BOARD_I2C0_SDA_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for I2C0_SDA_PIN\n");
        return ret;
    }

    /* I2C0_SCL */
    ret = pinconf_set(PORT_(BOARD_I2C0_SCL_GPIO_PORT),
                      BOARD_I2C0_SCL_GPIO_PIN,
                      BOARD_I2C0_SCL_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for I2C0_SCL_PIN\n");
        return ret;
    }

    return ret;
}
#endif

/**
 * @fn      static void i2c_master_task(void *pvParameters)
 * @brief   Performs Master data comm through I2C
 * @note    none
 * @param   pvParameters : Task parameter
 * @retval  none
 */
static void i2c_master_task(void *pvParameters)
{
    int32_t            ret  = 0;
    uint8_t            iter = 0;
    ARM_DRIVER_VERSION version;
    uint32_t           mst_event = 0;

    version                      = I2C_mstdrv->GetVersion();
    printf("\r\n I2C version api:0x%X driver:0x%X...\r\n", version.api, version.drv);

    /* Initialize I2C driver */
    ret = I2C_mstdrv->Initialize(i2c_mst_tranfer_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Master init failed\n");
        goto master_error_uninitialize;
    }

    /* Power control I2C */
    ret = I2C_mstdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Master Power up failed\n");
        goto master_error_uninitialize;
    }

    /* Set I2C bus-speed to (400kHz) fast mode */
    ret = I2C_mstdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Master control failed\n");
        goto master_error_poweroff;
    }

    vTaskDelay(5);

    printf("\n----------------Master transmit/slave receive--------------\n");

    I2C_mstdrv->MasterTransmit(TAR_ADDRS, MST_TX_BUF, MST_BYTE_TO_TRANSMIT, STOP);

    /* Waiting for Master callback */
    if (xTaskNotifyWait(NULL,
                        (LPI2C_CB_EVENT_SUCCESS | LPI2C_CB_EVENT_ERROR),
                        &mst_event,
                        portMAX_DELAY) != pdFALSE) {
        if (mst_event == LPI2C_CB_EVENT_ERROR) {
            printf("\r\nError: Master Tx failed\r\n");
            goto master_error_poweroff;
        }
    }

    vTaskDelay(5);

    printf("\n----------------Master receive/slave transmit--------------\n");

    for (iter = 0; iter < SLV_BYTE_TO_TRANSMIT; iter++) {
        I2C_mstdrv->MasterReceive(TAR_ADDRS, &MST_RX_BUF[iter], 1, STOP);

        /* wait for master callback. */
        if (xTaskNotifyWait(NULL,
                            (LPI2C_CB_EVENT_SUCCESS | LPI2C_CB_EVENT_ERROR),
                            &mst_event,
                            portMAX_DELAY) != pdFALSE) {
            if (mst_event == LPI2C_CB_EVENT_ERROR) {
                printf("\r\nError: Master Rx failed\r\n");
                goto master_error_poweroff;
            }
        }
    }

    vTaskDelay(5);

    /* Compare received data. */
    if (memcmp(&SLV_TX_BUF, &MST_RX_BUF, SLV_BYTE_TO_TRANSMIT)) {
        printf("\n Error: Master receive/Slave transmit failed\n");
        goto master_error_poweroff;
    }

    printf("\n >>> LPI2C0 transfer completed without any error\n");

master_error_poweroff:
    /* Power off peripheral */
    ret = I2C_mstdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Master Power OFF failed.\r\n");
    }

master_error_uninitialize:
    /* Un-initialize I2C driver */
    ret = I2C_mstdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Master Uninitialize failed.\r\n");
    }

    vTaskDelete(NULL);
}

/**
 * @fn      static void i2c_slave_task(void *pvParameters)
 * @brief   Performs Slave data comm through LPI2C0
 * @note    none
 * @param   pvParameters : Task parameter
 * @retval  none
 */
static void i2c_slave_task(void *pvParameters)
{
    int32_t            ret = 0;
    ARM_DRIVER_VERSION version;
    uint32_t           slv_event = 0;

    version                      = LPI2C_slvdrv->GetVersion();
    printf("\r\n LPI2C0 version api:0x%X driver:0x%X...\r\n", version.api, version.drv);

    /* Initialize LPI2C0 driver */
    ret = LPI2C_slvdrv->Initialize(i2c_slv_transfer_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Slave init failed\n");
        goto slave_error_uninitialize;
    }

    /* Power control LPI2C0 */
    ret = LPI2C_slvdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Slave Power up failed\n");
        goto slave_error_uninitialize;
    }

    /* Perform LPI2C0 reception */
    LPI2C_slvdrv->SlaveReceive(SLV_RX_BUF, MST_BYTE_TO_TRANSMIT);

    /* Waiting for Slave callback */
    if (xTaskNotifyWait(NULL,
                        (LPI2C_CB_EVENT_SUCCESS | LPI2C_CB_EVENT_ERROR),
                        &slv_event,
                        portMAX_DELAY) != pdFALSE) {
        if (slv_event == LPI2C_CB_EVENT_ERROR) {
            printf("\r\nError: Slave Rx failed\r\n");
            goto slave_error_poweroff;
        }
    }

    /* Compare received data. */
    if (memcmp(&SLV_RX_BUF, &MST_TX_BUF, MST_BYTE_TO_TRANSMIT)) {
        printf("\n Error: Master transmit/Slave receive failed \n");
        goto slave_error_poweroff;
    }

    LPI2C_slvdrv->SlaveTransmit(SLV_TX_BUF, SLV_BYTE_TO_TRANSMIT);

    /* Waiting for Slave callback */
    if (xTaskNotifyWait(NULL,
                        (LPI2C_CB_EVENT_SUCCESS | LPI2C_CB_EVENT_ERROR),
                        &slv_event,
                        portMAX_DELAY) != pdFALSE) {
        if (slv_event == LPI2C_CB_EVENT_ERROR) {
            printf("\r\nError: Slave Tx failed\r\n");
            goto slave_error_poweroff;
        }
    }

slave_error_poweroff:
    ret = LPI2C_slvdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Slave Power OFF failed.\r\n");
    }

slave_error_uninitialize:
    ret = LPI2C_slvdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Slave Uninitialize failed.\r\n");
    }

    vTaskDelete(NULL);
}
/**
 * @fn      static void LPI2C0_demo(void)
 * @brief   Performs LPI2C0 demo
 * @note    none
 * @param   none
 * @retval  none
 */
static void LPI2C0_demo(void)
{
    BaseType_t xReturned = 0;
    int32_t    ret_val   = 0;

    printf("\r\n >>> LPI2C0 FreeRTOS demo starting up !!! <<< \r\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret_val = board_pins_config();
    if (ret_val != 0) {
        printf("Error in pin-mux configuration: %d\n", ret_val);
        return;
    }

#else
    /*
     * NOTE: The lpi2c0 pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret_val = board_lpi2c_pins_config();
    if (ret_val != 0) {
        printf("Error in pin-mux configuration: %d\n", ret_val);
        return;
    }
#endif

    /* Create application main thread */
    xReturned = xTaskCreate(i2c_master_task,
                            "i2c_master_thread",
                            STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES - 1,
                            &master_taskHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(master_taskHandle);
        return;
    }

    /* Create application main thread */
    xReturned = xTaskCreate(i2c_slave_task,
                            "i2c_slave_thread",
                            STACK_SIZE,
                            NULL,
                            configMAX_PRIORITIES - 2,
                            &slave_taskHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(slave_taskHandle);
        return;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}

/**
 * @fn      int main(void)
 * @brief   Entry function of LPI2C0
 * @note    none
 * @param   none
 * @retval  none
 */
int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    int32_t ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* Invokes LPI2C0 demo */
    LPI2C0_demo();
}
