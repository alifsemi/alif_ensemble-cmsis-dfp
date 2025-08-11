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
 * @file     : demo_pdm_freertos.c
 * @author   : Nisarga A M
 * @email    : nisarga.am@alifsemi.com
 * @version  : V1.0.0
 * @date     : 29-May-2023
 * @brief    : TestApp to verify PDM(Pulse Density Modulation) interface using
 *             FreeRTOS as an operating system.
 *             -> Select the PDM channel
 *             -> Select the mode of operation in the Control API.
 *             -> Select the mode of operation in the Control API.
 *             -> Then build the project and flash the generated axf file on the target
 *             -> Then Start playing some audio or speak near to the PDM microphone which
 *                is on B0 Flat board.
 *             -> Once the sample count reaches the maximum value,the PCM samples will be
 *                stored in the particular buffer.
 *             -> Export the memory and To play the PCM data, use pcmplay.c file which
 *                will generate the pcm_samples.pcm audio file
 *             -> Use ffplay command to play the audio.
 *             -> No hardware setup is required for PDM channel 4 and channel 5.
 *             Hardware setup:
 *             -> Flat board has internal PDM Mics which is connected to channel 4
 *                and channel 5 internally, no hardware connection is required for
 *                channel 4 and 5.
 *                For channel 4 and channel 5
 *             -> Clock line:
 *                 PDM_C2 (PDM_C2_A) -> P6_7
 *             -> Data line:
 *                 PDM_D2 (PDM_D2_B) -> P5_4
 *
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include <inttypes.h>

/* Project Includes */
/* include for PDM Driver */
#include "Driver_PDM.h"
#include "pinconf.h"
#include "board_config.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "RTE_Components.h"
#include "app_utils.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

// Set to 0: Use application-defined PDM pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* PDM driver instance */
extern ARM_DRIVER_PDM  Driver_PDM;
static ARM_DRIVER_PDM *PDMdrv = &Driver_PDM;

/*Define for FreeRTOS*/
#define STACK_SIZE                    1024
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

/* Store the number of samples */
/* For 30000 samples user can hear maximum up to 4 sec of audio
 * to store maximum samples then change the scatter file and increase the memory */
#define NUM_SAMPLE          30000

/* channel number used for channel configuration and status register */
#define CHANNEL_4           4
#define CHANNEL_5           5

/* PDM Channel 4 configurations */
#define CH4_PHASE           0x0000001F
#define CH4_GAIN            0x0000000D
#define CH4_PEAK_DETECT_TH  0x00060002
#define CH4_PEAK_DETECT_ITV 0x0004002D

/* PDM Channel 5 configurations */
#define CH5_PHASE           0x00000003
#define CH5_GAIN            0x00000013
#define CH5_PEAK_DETECT_TH  0x00060002
#define CH5_PEAK_DETECT_ITV 0x00020027

TaskHandle_t pdm_xHandle;

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

typedef enum {
    PDM_CALLBACK_ERROR_EVENT           = (1 << 0),
    PDM_CALLBACK_WARNING_EVENT         = (1 << 1),
    PDM_CALLBACK_AUDIO_DETECTION_EVENT = (1 << 2)
} PDM_CB_EVENTS;

PDM_CH_CONFIG pdm_coef_reg;

/* For Demo purpose use channel 4  and channel 5 */
/* To store the PCM samples for any Channel */
uint16_t sample_buf[NUM_SAMPLE];

/* Channel 4 FIR coefficient */
uint32_t ch4_fir[18] = {0x00000001,
                        0x00000003,
                        0x00000003,
                        0x000007F4,
                        0x00000004,
                        0x000007ED,
                        0x000007F5,
                        0x000007F4,
                        0x000007D3,
                        0x000007FE,
                        0x000007BC,
                        0x000007E5,
                        0x000007D9,
                        0x00000793,
                        0x00000029,
                        0x0000072C,
                        0x00000072,
                        0x000002FD};

/* Channel 5 FIR coefficient */
uint32_t ch5_fir[18] = {0x00000000,
                        0x000007FF,
                        0x00000000,
                        0x00000004,
                        0x00000004,
                        0x000007FC,
                        0x00000000,
                        0x000007FB,
                        0x000007E4,
                        0x00000000,
                        0x0000002B,
                        0x00000009,
                        0x00000016,
                        0x00000049,
                        0x00000793,
                        0x000006F8,
                        0x00000045,
                        0x00000178};

static void PDM_fifo_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;
    /* Save received events */
    /* Optionally, user can define specific actions for an event */

    if (event & ARM_PDM_EVENT_ERROR) {
        xTaskNotifyFromISR(pdm_xHandle,
                           PDM_CALLBACK_ERROR_EVENT,
                           eSetBits,
                           &xHigherPriorityTaskWoken);

        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_PDM_EVENT_CAPTURE_COMPLETE) {
        xTaskNotifyFromISR(pdm_xHandle,
                           PDM_CALLBACK_WARNING_EVENT,
                           eSetBits,
                           &xHigherPriorityTaskWoken);

        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_PDM_EVENT_AUDIO_DETECTION) {
        xTaskNotifyFromISR(pdm_xHandle,
                           PDM_CALLBACK_AUDIO_DETECTION_EVENT,
                           eSetBits,
                           &xHigherPriorityTaskWoken);

        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_lppdm_pins_config(void)
 * @brief   Configure PDM pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_pdm_pins_config(void)
{
    int32_t status;

    /* channel 0_1 data line */
    status = pinconf_set(PORT_(BOARD_PDM_D0_A_GPIO_PORT),
                         BOARD_PDM_D0_A_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_3,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 2_3 data line */
    status = pinconf_set(PORT_(BOARD_PDM_D1_C_GPIO_PORT),
                         BOARD_PDM_D1_C_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_4,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 4_5 data line */
    status = pinconf_set(PORT_(BOARD_PDM_D2_B_GPIO_PORT),
                         BOARD_PDM_D2_B_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_3,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 6_7 data line */
    status = pinconf_set(PORT_(BOARD_PDM_D3_A_GPIO_PORT),
                         BOARD_PDM_D3_A_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_3,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* Channel 0_1 clock line */
    status = pinconf_set(PORT_(BOARD_PDM_C0_A_GPIO_PORT),
                         BOARD_PDM_C0_A_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_3,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 2_3 clock line */
    status = pinconf_set(PORT_(BOARD_PDM_C1_C_GPIO_PORT),
                         BOARD_PDM_C1_C_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_4,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 4_5 clock line */
    status = pinconf_set(PORT_(BOARD_PDM_C2_A_GPIO_PORT),
                         BOARD_PDM_C2_A_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_3,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 6_7 clock line */
    status = pinconf_set(PORT_(BOARD_PDM_C3_A_GPIO_PORT),
                         BOARD_PDM_C3_A_GPIO_PIN,
                         PINMUX_ALTERNATE_FUNCTION_3,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

/**
 * @fn         : void pdm_demo_thread_entry(void *pvParameters)
 * @brief      : PDM fifo callback
 *               -> Initialize the PDM module.
 *               -> Enable the Power for the PDM module
 *               -> Select the mode of operation in Control API.The
 *                    mode which user has selected will be applies to
 *                    all the channel which user has selected.
 *               -> Select the Bypass DC blocking IIR filter for reference.
 *               -> Select the PDM channel and use the selected channel
 *                    configuration and status register values.
 *               -> Play some audio and start capturing the data.
 *               -> Once all data has stored in the particular buffer ,
 *                  call back event will be set and it will stop capturing
 *                  data.
 *               -> Once all the data capture is done , go to the particular
 *                  memory location which user has given for storing PCM data
 *                  samples.
 *               -> Then export the memory and give the total size of
 *                  the buffer memory and select the particular bin file and export
 *                  the memory.
 *               -> Play the PCM sample file using ffplay command.
 * @return     : none
 */
void pdm_demo_thread_entry(void *pvParameters)
{
    int32_t            ret = 0;
    ARM_DRIVER_VERSION version;
    uint32_t           ulNotificationValue;

    printf("\r\n >>> PDM demo starting up!!! <<< \r\n");

    version = PDMdrv->GetVersion();
    printf("\r\n PDM version api:%X driver:%X...\r\n", version.api, version.drv);

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }

#else
    /*
     * NOTE: The PDM pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_pdm_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }
#endif

#if SOC_FEAT_CLK76P8M_CLK_ENABLE
    uint32_t error_code = SERVICES_REQ_SUCCESS;
    uint32_t service_error_code;

    /* Initialize the SE services */
    se_services_port_init();

    /* enable the HFOSCx2 clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_HFOSCx2,
                                              /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: clk enable = %" PRId32 "\n", error_code);
    }
#endif

    /* Initialize PDM driver */
    ret = PDMdrv->Initialize(PDM_fifo_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM init failed\n");
        return;
    }

    /* Enable the power for PDM */
    ret = PDMdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Power up failed\n");
        goto error_uninitialize;
    }

    /* To select the PDM channel 4 and channel 5 */
    ret = PDMdrv->Control(ARM_PDM_SELECT_CHANNEL,
                          ARM_PDM_MASK_CHANNEL_4 | ARM_PDM_MASK_CHANNEL_5,
                          NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM channel select control failed\n");
        goto error_poweroff;
    }

    /* Select Standard voice PDM mode */
    ret = PDMdrv->Control(ARM_PDM_MODE, ARM_PDM_MODE_AUDIOFREQ_8K_DECM_64, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Standard voice control mode failed\n");
        goto error_poweroff;
    }

    /* Select the DC blocking IIR filter */
    ret = PDMdrv->Control(ARM_PDM_BYPASS_IIR_FILTER, ENABLE, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM DC blocking IIR control failed\n");
        goto error_poweroff;
    }

    /* Set Channel 4 Phase value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PHASE, CHANNEL_4, CH4_PHASE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 4 Gain value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_GAIN, CHANNEL_4, CH4_GAIN);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 4 Peak detect threshold value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_TH, CHANNEL_4, CH4_PEAK_DETECT_TH);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 4 Peak detect ITV value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_ITV, CHANNEL_4, CH4_PEAK_DETECT_ITV);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    pdm_coef_reg.ch_num = 4;
    memcpy(pdm_coef_reg.ch_fir_coef,
           ch4_fir,
           sizeof(pdm_coef_reg.ch_fir_coef)); /* Channel 4 fir coefficient */
    pdm_coef_reg.ch_iir_coef = 0x00000004;    /* Channel IIR Filter Coefficient */

    ret                      = PDMdrv->Config(&pdm_coef_reg);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 5 Phase value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PHASE, CHANNEL_5, CH5_PHASE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 5 Gain value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_GAIN, CHANNEL_5, CH5_GAIN);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 5 Peak detect threshold value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_TH, CHANNEL_5, CH5_PEAK_DETECT_TH);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 5 Peak detect ITV value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_ITV, CHANNEL_5, CH5_PEAK_DETECT_ITV);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    pdm_coef_reg.ch_num = 5;
    memcpy(pdm_coef_reg.ch_fir_coef,
           ch5_fir,
           sizeof(pdm_coef_reg.ch_fir_coef)); /* Channel 5 fir coefficient */
    pdm_coef_reg.ch_iir_coef = 0x00000004;    /* Channel IIR Filter Coefficient */

    ret                      = PDMdrv->Config(&pdm_coef_reg);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    printf("\n------> Start Speaking or Play some Audio!------> \n");

    /* Receive the audio samples */
    ret = PDMdrv->Receive((uint16_t *) sample_buf, NUM_SAMPLE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Receive failed\n");
        goto error_capture;
    }

    /* wait for the call back event */
    xTaskNotifyWait(NULL,
                    PDM_CALLBACK_ERROR_EVENT | PDM_CALLBACK_WARNING_EVENT |
                        PDM_CALLBACK_AUDIO_DETECTION_EVENT,
                    &ulNotificationValue,
                    portMAX_DELAY);

    /* PDM channel audio detection event */
    if (ulNotificationValue & PDM_CALLBACK_AUDIO_DETECTION_EVENT) {
        printf("\n PDM audio detect event: data in the audio channel");
    }

    /* PDM fifo alomost full warning event */
    if (ulNotificationValue & PDM_CALLBACK_WARNING_EVENT) {
        printf("\n PDM warning event : Fifo almost full\n");
    }

    /* PDM fifo overflow error event */
    if (ulNotificationValue & PDM_CALLBACK_ERROR_EVENT) {
        printf("\n PDM error event: Fifo overflow \n");
    }

    printf("\n------> Stop recording ------> \n");
    printf("\n--> PCM samples will be stored in 0x%p address and size of buffer is %d\n",
           sample_buf,
           sizeof(sample_buf));
    printf("\n ---END--- \r\n <<< wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_capture:
error_poweroff:
    ret = PDMdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\n Error: PDM power off failed\n");
    }

error_uninitialize:
    ret = PDMdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\n Error: PDM Uninitialize failed\n");
    }

#if SOC_FEAT_CLK76P8M_CLK_ENABLE
    /* disable the HFOSCx2 clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSCx2,
                                              false,
                                              &service_error_code);
    if (error_code) {
        printf("SE Error: HFOSCx2 clk disable = %" PRIu32 "\n", error_code);
    }
#endif

    printf("\r\n XXX PDM demo exiting XXX...\r\n");
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
    BaseType_t xReturned = xTaskCreate(pdm_demo_thread_entry,
                                       "PDMFreertos",
                                       256,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &pdm_xHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(pdm_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}
