/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/****************************************************************************
 * @file     : demo_i2s_freertos.c
 * @author   : Sudhir Sreedharan
 * @email    : sudhir@alifsemi.com
 * @version  : V1.0.0
 * @date     : 24-Jan-2024
 * @brief    : Test Application for I2S for Devkit
 *             For HP, I2S1 is configured as master transmitter (DAC).
 *             For HE, LPI2S will be used as DAC.
 *             I2S3(ADC) is configured as master receiver SPH0645LM4H-1 device 24bit
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/

/*System Includes */
#include <stdio.h>
#include <string.h>

/* Project Includes */
#include <Driver_SAI.h>
#include <pinconf.h>
#include "board_config.h"
#include "RTE_Components.h"

#include "app_utils.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#if BOARD_WM8904_CODEC_PRESENT
#include "WM8904_driver.h"
#endif

/*RTOS Includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

/* Enable this macro to play the predefined sample */
//#define DAC_PREDEFINED_SAMPLES

#ifdef DAC_PREDEFINED_SAMPLES
/*Audio samples */
#include "i2s_samples.h"
#endif

#if (BOARD_WM8904_CODEC_PRESENT) && !defined(RTE_Drivers_WM8904_CODEC)
#error "WM8904 codec driver not configured in RTE_Components.h"
#endif

// Set to 0: Use application-defined I2S pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#if BOARD_WM8904_CODEC_PRESENT
extern ARM_DRIVER_WM8904 WM8904;
static ARM_DRIVER_WM8904 *wm8904 = &WM8904;
#endif

/* 1 to send the data stream continuously , 0 to send data only once */
#define REPEAT_TX                      1

#define DAC_SEND_COMPLETE_EVENT    (1U << 0)
#define ADC_RECEIVE_COMPLETE_EVENT (1U << 1)
#define ADC_RECEIVE_OVERFLOW_EVENT (1U << 2)

#define ADC_RECEIVE_TIMEOUT        (10)
#define DAC_SEND_TIMEOUT           (10)

/*
 * With the below configuration, the FreeRTOS heap size should be
 * minimum of 102400 bytes
 *
 * #define configTOTAL_HEAP_SIZE                 ((size_t)102400)
 */
#define NUM_SAMPLES_IN_SINGLE_POOL 8000
#define SAMPLES_POOL_CNT           3

/*Define for FreeRTOS*/
#define DAC_STACK_SIZE             1024
#define ADC_STACK_SIZE             1024

static TaskHandle_t       xDacHandle;
static EventGroupHandle_t xEventGroupHandleDac;

#define DAC_TASK_PRIORITY 3

#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
static TaskHandle_t       xAdcHandle;
static EventGroupHandle_t xEventGroupHandleAdc;

#define ADC_TASK_PRIORITY configMAX_PRIORITIES - 1
#endif

QueueHandle_t xQueueRx;

/* message information for queueing */
typedef struct xSAMPLESMSGQ {
    void    *pvBuf;
    uint32_t ulNumItems;
} SamplesMsgq_t;

static uint32_t ulWlen         = 24;
static uint32_t ulSamplingRate = 48000; /* 48Khz audio sampling rate */

/**
  \fn          void prvDacCallback( uint32_t ulEvent )
  \brief       Callback routine from the i2s driver
  \param[in]   ulEvent Event for which the callback has been called
*/
static void prvDacCallback(uint32_t ulEvent)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;

    xHigherPriorityTaskWoken = pdFALSE;

    if (ulEvent & ARM_SAI_EVENT_SEND_COMPLETE) {
        /* Send Success: Wake-up Thread. */
        xResult = xEventGroupSetBitsFromISR(xEventGroupHandleDac,
                                            (const EventBits_t) DAC_SEND_COMPLETE_EVENT,
                                            &xHigherPriorityTaskWoken);

        if (xResult == pdPASS) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_i2s_dac_pins_config(void)
 * @brief   Configure I2S DAC pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_i2s_dac_pins_config(void)
{
    int32_t status;

    /* Configure DAC I2S SDO */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_SDO_GPIO_PORT),
                         BOARD_DAC_OUTPUT_SDO_GPIO_PIN,
                         BOARD_DAC_OUTPUT_SDO_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure DAC I2S WS */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_WS_GPIO_PORT),
                         BOARD_DAC_OUTPUT_WS_GPIO_PIN,
                         BOARD_DAC_OUTPUT_WS_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure DAC I2S SCLK */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_SCLK_GPIO_PORT),
                         BOARD_DAC_OUTPUT_SCLK_GPIO_PIN,
                         BOARD_DAC_OUTPUT_SCLK_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

#if BOARD_WM8904_CODEC_PRESENT
/**
 * @fn      void board_wm8904_i2c_pins_config(void)
 * @brief   Initialize the pinmux for I2C
 * @retval  status
 */
static int32_t board_wm8904_i2c_pins_config(void)
{
    int32_t status;

    /* I2C_SDA */
    status = pinconf_set(PORT_(BOARD_WM8904_CODEC_I2C_SDA_GPIO_PORT),
                         BOARD_WM8904_CODEC_I2C_SDA_GPIO_PIN,
                         BOARD_WM8904_CODEC_I2C_SDA_ALTERNATE_FUNCTION,
                         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
    if (status) {
        return status;
    }

    /* I2C_SCL */
    status = pinconf_set(PORT_(BOARD_WM8904_CODEC_I2C_SCL_GPIO_PORT),
                         BOARD_WM8904_CODEC_I2C_SCL_GPIO_PIN,
                         BOARD_WM8904_CODEC_I2C_SCL_ALTERNATE_FUNCTION,
                         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

/**
  \fn          void prvDacTask( void * pvParameters )
  \brief       DAC thread for master transmission
  \param[in]   pvParameters Task parameters
*/
static void prvDacTask(void *pvParameters)
{
    ARM_DRIVER_VERSION   xVersion;
    ARM_DRIVER_SAI      *xI2SDrv;
    ARM_SAI_CAPABILITIES xCap;
    int32_t              lStatus;
    SamplesMsgq_t        xSamplesMsg;
    EventBits_t          xEvents;

#if SOC_FEAT_CLK76P8M_CLK_ENABLE
    uint32_t error_code        = SERVICES_REQ_SUCCESS;
    uint32_t service_error_code;

    /* Initialize the SE services */
    se_services_port_init();

/* enable the HFOSCx2 clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                           /*clock_enable_t*/ CLKEN_HFOSCx2,
                           /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: clk enable = %d\n", error_code);
    }
#endif

    extern ARM_DRIVER_SAI ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);

    (void) pvParameters;

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    lStatus = board_pins_config();
    if (lStatus != 0) {
        printf("Error in pin-mux configuration: %d\n", lStatus);
        return;
    }

#else
    /*
     * NOTE: The I2S DAC pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    lStatus = board_i2s_dac_pins_config();
    if (lStatus != 0) {
        printf("Error in pin-mux configuration: %d\n", lStatus);
        return;
    }

#if BOARD_WM8904_CODEC_PRESENT
    /* Configure the i2c pins to program WM8904 codec */
    lStatus = board_wm8904_i2c_pins_config();
    if (lStatus != 0) {
        printf("I2C pinmux failed\n");
        return;
    }
#endif
#endif

#if BOARD_WM8904_CODEC_PRESENT
    /* WM8904 codec init */
    lStatus = wm8904->Initialize();
    if (lStatus) {
        printf("WM8904 codec Init failed status = %d\n", lStatus);
        goto error_codec_initialize;
    }

    lStatus = wm8904->PowerControl(ARM_POWER_FULL);
    if (lStatus) {
        printf("WM8904 codec Power up failed status = %d\n", lStatus);
        goto error_codec_power;
    }
#endif

    /* Use the I2S as Trasmitter */
    xI2SDrv  = &ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);

    /* Verify the I2S API version for compatibility*/
    xVersion = xI2SDrv->GetVersion();
    printf("I2S API Version = %d\n", xVersion.api);

    /* Verify if I2S protocol is supported */
    xCap = xI2SDrv->GetCapabilities();
    if (!xCap.protocol_i2s) {
        printf("I2S is not supported\n");
        return;
    }

    /* Initializes I2S interface */
    lStatus = xI2SDrv->Initialize(prvDacCallback);
    if (lStatus) {
        printf("DAC Init failed Status = %d\n", lStatus);
        goto error_initialize;
    }

    /* Enable the power for I2S */
    lStatus = xI2SDrv->PowerControl(ARM_POWER_FULL);
    if (lStatus) {
        printf("DAC Power Failed Status = %d\n", lStatus);
        goto error_power;
    }

    /* configure I2S Transmitter to Asynchronous Master */
    lStatus = xI2SDrv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                   ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(ulWlen),
                               (ulWlen * 2),
                               ulSamplingRate);
    if (lStatus) {
        printf("DAC Control Status = %d\n", lStatus);
        goto error_control;
    }

    /* enable Transmitter */
    lStatus = xI2SDrv->Control(ARM_SAI_CONTROL_TX, 1, 0);
    if (lStatus) {
        printf("DAC TX status = %d\n", lStatus);
        goto error_control;
    }

    do {
#if defined(DAC_PREDEFINED_SAMPLES)
        xSamplesMsg.pvBuf = (void *) ulHelloSamples24bit48khz;
        xSamplesMsg.ulNumItems =
            sizeof(ulHelloSamples24bit48khz) / sizeof(ulHelloSamples24bit48khz[0]);

        /* Transmit the samples */
        lStatus = xI2SDrv->Send(xSamplesMsg.pvBuf, xSamplesMsg.ulNumItems);
        if (lStatus) {
            printf("DAC Send status = %d\n", lStatus);
            goto error_send;
        }

        /* Wait till we get the flag */
        xEvents = xEventGroupWaitBits(xEventGroupHandleDac,
                                      (const EventBits_t) DAC_SEND_COMPLETE_EVENT,
                                      pdTRUE,  /* clear bits on exit */
                                      pdFALSE, /* logical OR. Don't wait for all bits to set */
                                      pdMS_TO_TICKS(DAC_SEND_TIMEOUT * 1000));

        if (!(xEvents & DAC_SEND_COMPLETE_EVENT)) {
            printf("Timeout occurred while sending the data to DAC\n");

            vPortFree(xSamplesMsg.pvBuf);
            goto error_send;
        }

#else
        /* Read the samples from the queue */
        if (xQueueReceive(xQueueRx, &xSamplesMsg, portMAX_DELAY)) {
            /* Transmit the samples */
            lStatus = xI2SDrv->Send(xSamplesMsg.pvBuf, xSamplesMsg.ulNumItems);
            if (lStatus) {
                printf("DAC Send status = %d\n", lStatus);

                vPortFree(xSamplesMsg.pvBuf);
                goto error_send;
            }

            /* Wait till we get the flag */
            xEvents = xEventGroupWaitBits(xEventGroupHandleDac,
                                          (const EventBits_t) DAC_SEND_COMPLETE_EVENT,
                                          pdTRUE,  /* clear bits on exit */
                                          pdFALSE, /* logical OR. Don't wait for all bits to set */
                                          pdMS_TO_TICKS(DAC_SEND_TIMEOUT * 1000));

            if (!xEvents) {
                printf("Timeout occurred while sending the data to DAC\n");

                vPortFree(xSamplesMsg.pvBuf);
                goto error_send;
            }

            vPortFree(xSamplesMsg.pvBuf);
        }
#endif
    } while (REPEAT_TX);

    /* Stop the TX */
    lStatus = xI2SDrv->Control(ARM_SAI_CONTROL_TX, 0, 0);
    if (lStatus) {
        printf("DAC TX status = %d\n", lStatus);
        goto error_control;
    }

error_send:
error_control:
    xI2SDrv->PowerControl(ARM_POWER_OFF);
error_power:
    xI2SDrv->Uninitialize();
error_initialize:
#if BOARD_WM8904_CODEC_PRESENT
error_codec_power:
    wm8904->PowerControl(ARM_POWER_OFF);
    wm8904->Uninitialize();
#endif
#if BOARD_WM8904_CODEC_PRESENT
error_codec_initialize:
#endif
    vTaskSuspend(xDacHandle);
}

#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
/**
  \fn          void prvAdcCallback(uint32_t ulEvent)
  \brief       Callback routine from the i2s driver
  \param[in]   ulEvent Event for which the callback has been called
*/
static void prvAdcCallback(uint32_t ulEvent)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;

    xHigherPriorityTaskWoken = pdFALSE;

    if (ulEvent & ARM_SAI_EVENT_RECEIVE_COMPLETE) {
        /* Send Success: Wake-up Thread. */
        xResult = xEventGroupSetBitsFromISR(xEventGroupHandleAdc,
                                            (const EventBits_t) ADC_RECEIVE_COMPLETE_EVENT,
                                            &xHigherPriorityTaskWoken);

        if (xResult == pdPASS) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (ulEvent & ARM_SAI_EVENT_RX_OVERFLOW) {
        /* Send Success: Wake-up Thread. */
        xResult = xEventGroupSetBitsFromISR(xEventGroupHandleAdc,
                                            (const EventBits_t) ADC_RECEIVE_OVERFLOW_EVENT,
                                            &xHigherPriorityTaskWoken);

        if (xResult == pdPASS) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_i2s_adc_pins_config(void)
 * @brief   Configure I2S ADC pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_i2s_adc_pins_config(void)
{
    int32_t status;

    /* Configure ADC I2S WS */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_WS_GPIO_PORT),
                         BOARD_MIC_INPUT_WS_GPIO_PIN,
                         BOARD_MIC_INPUT_WS_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure ADC I2S SCLK */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_SCLK_GPIO_PORT),
                         BOARD_MIC_INPUT_SCLK_GPIO_PIN,
                         BOARD_MIC_INPUT_SCLK_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure ADC I2S SDI */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_SDI_GPIO_PORT),
                         BOARD_MIC_INPUT_SDI_GPIO_PIN,
                         BOARD_MIC_INPUT_SDI_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

/**
  \fn          void prvAdcTask( void * pvParameters )
  \brief       ADC Task
  \param[in]   pvParameters Task parameters
*/
static void prvAdcTask(void *pvParameters)
{
    ARM_DRIVER_VERSION   xVersion;
    ARM_DRIVER_SAI      *xI2SDrv;
    ARM_SAI_CAPABILITIES xCap;
    int32_t              lStatus;
    SamplesMsgq_t        xSamplesMsg;
    EventBits_t          xEvents;

    extern ARM_DRIVER_SAI ARM_Driver_SAI_(BOARD_MIC_INPUT_I2S_INSTANCE);
    (void) pvParameters;

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
    /*
     * NOTE: The I2S ADC pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    lStatus = board_i2s_adc_pins_config();
    if (lStatus != 0) {
        printf("Error in pin-mux configuration: %d\n", lStatus);
        return;
    }
#endif
    /* Use the I2S as Receiver */
    xI2SDrv  = &ARM_Driver_SAI_(BOARD_MIC_INPUT_I2S_INSTANCE);

    /* Verify the I2S API version for compatibility*/
    xVersion = xI2SDrv->GetVersion();
    printf("I2S API Version = %d\n", xVersion.api);

    /* Verify if I2S protocol is supported */
    xCap = xI2SDrv->GetCapabilities();
    if (!xCap.protocol_i2s) {
        printf("I2S is not supported\n");
        return;
    }

    /* Initializes I2S interface */
    lStatus = xI2SDrv->Initialize(prvAdcCallback);
    if (lStatus) {
        printf("ADC Init failed Status = %d\n", lStatus);
        goto error_adc_initialize;
    }

    /* Enable the power for I2S */
    lStatus = xI2SDrv->PowerControl(ARM_POWER_FULL);
    if (lStatus) {
        printf("ADC Power failed Status = %d\n", lStatus);
        goto error_adc_power;
    }

    /* configure I2S Receiver to Asynchronous Master */
    lStatus = xI2SDrv->Control(ARM_SAI_CONFIGURE_RX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                   ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(ulWlen),
                               (ulWlen * 2),
                               ulSamplingRate);
    if (lStatus) {
        printf("ADC Control Status = %d\n", lStatus);
        goto error_adc_control;
    }

    /* enable Receiver */
    lStatus = xI2SDrv->Control(ARM_SAI_CONTROL_RX, 1, 0);
    if (lStatus) {
        printf("ADC RX Status = %d\n", lStatus);
        goto error_adc_control;
    }

    for (;;) {
        /* Get one block, If not, return error */
        xSamplesMsg.pvBuf = pvPortMalloc(NUM_SAMPLES_IN_SINGLE_POOL * 4);
        if (!xSamplesMsg.pvBuf) {
            printf(" ADC: Not enough memory\n");
            goto error_adc_alloc;
        }

        xSamplesMsg.ulNumItems = NUM_SAMPLES_IN_SINGLE_POOL;
        memset((void *) xSamplesMsg.pvBuf, 0x0, xSamplesMsg.ulNumItems * 4);

        /* Receive data */
        lStatus = xI2SDrv->Receive(xSamplesMsg.pvBuf, xSamplesMsg.ulNumItems);
        if (lStatus) {
            printf("ADC Receive status = %d\n", lStatus);

            vPortFree(xSamplesMsg.pvBuf);
            goto error_adc_receive;
        }

        /* Wait till we get the flag */
        xEvents = xEventGroupWaitBits(xEventGroupHandleAdc,
                                      (const EventBits_t) ADC_RECEIVE_COMPLETE_EVENT,
                                      pdTRUE,  /* clear bits on exit */
                                      pdFALSE, /* logical OR. Don't wait for all bits to set */
                                      pdMS_TO_TICKS(ADC_RECEIVE_TIMEOUT * 1000));

        if (!(xEvents & ADC_RECEIVE_COMPLETE_EVENT)) {
            printf("\r\nTimeout occurred while receiving the data from ADC\r\n");

            vPortFree(xSamplesMsg.pvBuf);
            goto error_adc_receive;
        }

        xEvents = xEventGroupWaitBits(xEventGroupHandleAdc,
                                      (const EventBits_t) ADC_RECEIVE_OVERFLOW_EVENT,
                                      pdTRUE,  /* clear bits on exit */
                                      pdFALSE, /* logical OR. Don't wait for all bits to set */
                                      0);

        if (xEvents) {
            printf("\r\nADC: Overflow\r\n");
        }

        if (xQueueSend(xQueueRx, (void *) &xSamplesMsg, portMAX_DELAY) != pdPASS) {
            // Failed to post the message.

            vPortFree(xSamplesMsg.pvBuf);
        }
    }

    /* Stop the RX */
    lStatus = xI2SDrv->Control(ARM_SAI_CONTROL_RX, 0, 0);
    if (lStatus) {
        printf("DAC RX lStatus = %d\n", lStatus);
        goto error_adc_control;
    }

error_adc_alloc:
error_adc_receive:
error_adc_control:
    xI2SDrv->PowerControl(ARM_POWER_OFF);
error_adc_power:
    xI2SDrv->Uninitialize();
error_adc_initialize:
    vTaskSuspend(xAdcHandle);
}
#endif

/* Define main entry point.  */
int main(void)
{
    extern void SystemCoreClockUpdate(void);
    extern int  stdout_init(void);

    BaseType_t xStatus;

#if (defined(RTE_CMSIS_Compiler_STDOUT_Custom))
    {
        int32_t lRet;
        lRet = stdout_init();
        if (lRet != ARM_DRIVER_OK) {
            while (1) {
            }
        }
    }
#endif

    /* System Initialization */
    SystemCoreClockUpdate();

    /* Allocate the message queues for Rx */
    xQueueRx = xQueueCreate(SAMPLES_POOL_CNT, sizeof(SamplesMsgq_t));
    if (!xQueueRx) {
        printf("Could not create RX Queue \n");

        return -1;
    }

    /* Assigns a human readable name to queue and adds queue to queue registry */
    vQueueAddToRegistry(xQueueRx, "RX QUEUE");

#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
    {
        /* Initialize the ADC event flags group */
        xEventGroupHandleAdc = xEventGroupCreate();

        if (!xEventGroupHandleAdc) {
            printf("Could not create ADC Event Group \n");

            return -1;
        }

        /* Create the ADC Task */
        xStatus = xTaskCreate(prvAdcTask,
                              "ADC Task",
                              ADC_STACK_SIZE / sizeof(size_t),
                              NULL,
                              ADC_TASK_PRIORITY,
                              &xAdcHandle);
        if (xStatus != pdPASS) {
            vTaskDelete(xAdcHandle);
            printf("Could not create ADC Task \n");

            return -1;
        }
    }
#endif

    /* Initialize the DAC event flags group */
    xEventGroupHandleDac = xEventGroupCreate();

    if (!xEventGroupHandleDac) {
        printf("Could not create DAC Event Group \n");

        return -1;
    }

    /* Create the DAC Task */
    xStatus = xTaskCreate(prvDacTask,
                          "DAC TASK",
                          DAC_STACK_SIZE / sizeof(size_t),
                          NULL,
                          DAC_TASK_PRIORITY,
                          &xDacHandle);
    if (xStatus != pdPASS) {
        vTaskDelete(xDacHandle);
        printf("Could not create DAC Task \n");

        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();

    return 0;
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
