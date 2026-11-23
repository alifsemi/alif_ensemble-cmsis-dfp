/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     : demo_dual_camera_isp_freertos.c
 * @author   : Chandra Bhushan Singh
 * @email    : chandrabhushan.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 23-October-2026
 * @brief    : TestApp to verify dual camera capture through ISP
 *             with FREERTOS as an Operating System.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* CPI / ISP Driver */
#include "Driver_CPI.h"
#include "Driver_ISP.h"
#include "vsi_comm_video.h"
#include "RTE_Components.h"
#include "RTE_Device.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "board_config.h"
/* PINMUX Driver */
#include "pinconf.h"
#include "cache.h"

/* AIPL includes */
#include "aipl_error.h"
#include "aipl_color_conversion.h"

/* SE Services */
#include "se_services_port.h"

/* RTOS Includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "app_utils.h"

#include CMSIS_device_header

#if !defined(RTE_ISP) || (RTE_ISP == 0) || !RTE_CPI_ISP_PORT || \
    (defined(RTE_CPI_AXI_PORT) && RTE_CPI_AXI_PORT) || \
    (defined(RTE_CPI_STREAMING_ENABLE) && RTE_CPI_STREAMING_ENABLE)
#error "Dual camera ISP demo requires RTE_ISP=1 and RTE_CPI_ISP_PORT=1; " \
       "disable RTE_CPI_AXI_PORT and RTE_CPI_STREAMING_ENABLE"
#endif

#if (RTE_ISP_OUTPUT_FORMAT != 32)
#error "RTE_ISP_OUTPUT_FORMAT must be 32 (YUYV)"
#endif

/* Set to 0: Use application-defined arx3A0 pin configuration.
 * Set to 1: Use Conductor-generated pin configuration (from pins.h).
 */
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* Camera Driver instance 0 */
extern ARM_DRIVER_CPI  Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

/* Define for FreeRTOS */
#define STACK_SIZE                    1024
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

/* Thread id of thread */
TaskHandle_t camera_xHandle;

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

/*****************Only for FreeRTOS use *************************/

/* ISP output size (both sensors are scaled/cropped to this) */
#define ISP_OUTPUT_X          (RTE_ISP_OUTPUT_WIDTH)
#define ISP_OUTPUT_Y          (RTE_ISP_OUTPUT_HEIGHT)
#define ISP_OUTPUT_SIZE_Y     ((ISP_OUTPUT_X) * (ISP_OUTPUT_Y) * 2)

#define RGB_BYTES_PER_PIXEL   3U
#define RGB888_BUFFER_SIZE \
    ((uint32_t) (ISP_OUTPUT_X) * (uint32_t) (ISP_OUTPUT_Y) * RGB_BYTES_PER_PIXEL)

/* ISP YUYV dump buffers */
uint8_t y_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_Y]
    __attribute__((section(".bss.camera_frame_buf"), aligned(32)));

VIDEO_BUF_S buffer_array[RTE_ISP_BUFFER_COUNT];

/* Per-camera RGB888 pools (ISP output size) */
uint8_t isp_rgb_buffer_0[RGB888_BUFFER_SIZE]
    __attribute__((section(".bss.camera_frame_bayer_to_rgb_buf"), aligned(32)));
uint8_t isp_rgb_buffer_1[RGB888_BUFFER_SIZE]
    __attribute__((section(".bss.camera_frame_bayer_to_rgb_buf"), aligned(32)));

/* Camera / ISP callback events */
typedef enum {
    CAM_CB_EVENT_ERROR      = (1 << 0),
    ISP_MI_FRAME_DUMP_EVENT = (1 << 1)
} CAMERA_CB_EVENTS;

static void isp_buffer_init(void)
{
    for (int i = 0; i < RTE_ISP_BUFFER_COUNT; i++) {
        buffer_array[i].index                = i;
        buffer_array[i].imageSize            = ISP_OUTPUT_SIZE_Y;
        buffer_array[i].numPlanes            = 1;
        buffer_array[i].planes[0].dmaPhyAddr = (vsi_dma_t) y_buffer[i];
    }
}

/**
 * \fn          void camera_callback(uint32_t event)
 * \brief       Camera isr callback
 * \param[in]   event: Camera Event
 * \return      none
 */
void camera_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_ISP_MI_EVENT_MP_FRAME_END_DETECTED) {
        /* Transfer Success: ISP frame dump detected, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     ISP_MI_FRAME_DUMP_EVENT,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN) {
        /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN) {
        /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_CPI_EVENT_ERR_HARDWARE) {
        /* Transfer Error: Received Hardware error, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_CPI_EVENT_MIPI_CSI2_ERROR) {
        /* Transfer Error: Received MIPI CSI2 error, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_ISP_EVENT_DATALOSS_DETECTED) {
        /* Transfer Error: ISP data loss, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_ISP_EVENT_SIZE_ERR_DETECTED) {
        /* Transfer Error: ISP size error, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * \fn          int32_t i2c_pinmux(void)
 * \brief       i2c hardware pin initialization:
 *                 - PIN-MUX configuration
 *                 - PIN-PAD configuration
 * \param[in]   none
 * \return      0:success; -1:failure
 */
int32_t i2c_pinmux(void)
{
    int32_t ret;

    /* Configure GPIO Pin : P7_2 as I2C1_SDA_C
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_SDA_GPIO_PORT),
                      BOARD_CAMERA_I2C_SDA_GPIO_PIN,
                      BOARD_CAMERA_I2C_SDA_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: i3c PINMUX and PINPAD failed.\r\n");
        return ret;
    }

    /* Configure GPIO Pin : P7_3 as I2C1_SCL_C
     * Pad function: PADCTRL_READ_ENABLE
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_SCL_GPIO_PORT),
                      BOARD_CAMERA_I2C_SCL_GPIO_PIN,
                      BOARD_CAMERA_I2C_SCL_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: i3c PINMUX and PINPAD failed.\r\n");
        return ret;
    }

    return 0;
}
#endif

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * \fn          int32_t camera_pinmux(void)
 * \brief       Camera hardware pin initialization:
 *                  - PIN-MUX configuration
 * \param[in]   none
 * \return      0:success; -1:failure
 */
int32_t camera_pinmux(void)
{
    int32_t ret;

    ret = pinconf_set(PORT_(BOARD_CAM_XVCLK_A_GPIO_PORT),
                      BOARD_CAM_XVCLK_A_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_6,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Camera Pin-Mux failed.\r\n");
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_MUX_GPIO_PORT),
                      BOARD_CAMERA_I2C_MUX_GPIO_PIN,
                      BOARD_CAMERA_I2C_MUX_ALTERNATE_FUNCTION,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Camera I2C mux Pin-Mux failed.\r\n");
        return ret;
    }

    return 0;
}
#endif

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * \fn          int32_t hardware_init(void)
 * \brief       - i2c hardware pin initialization:
 *                  - PIN-MUX configuration
 *                  - PIN-PAD configuration
 *              - Camera hardware pin initialization:
 *                  - PIN-MUX configuration
 * \param[in]   none
 * \return      0:success; -1:failure
 */
int32_t hardware_init(void)
{
    int32_t ret;

    /* i2c pinmux. */
    ret = i2c_pinmux();
    if (ret != 0) {
        printf("\r\n Error in i3c pinmux.\r\n");
        return ret;
    }

    /* Camera pinmux. */
    ret = camera_pinmux();
    if (ret != 0) {
        printf("\r\n Error in Camera pinmux.\r\n");
        return ret;
    }

    return 0;
}
#endif

/**
 * \fn          void camera_demo_thread_entry(void *pvParameters)
 * \brief       TestApp to verify dual camera capture with
 *              FREERTOS as an Operating System.
 *              This demo thread does:
 *                   - initialize i2c and Camera hardware pins
 *                     using PinMux Driver;
 *                   - initialize DPHY Tx.
 *                   - initialize Camera driver
 *                   - capture one ISP frame from each camera instance
 *                   - captured data will be stored in to allocated
 *                     frame buffer address
 *                   - stop Camera capture
 *                   - convert ISP YUYV to RGB888 with aipl_color_convert_yuy2_to_rgb888
 *                   - dump captured image data from memory address
 *                     using any debugger
 * @param       pvParameters.
 * \return      none
 */
void camera_demo_thread_entry(void *pvParameters)
{
    int32_t            ret           = 0;
    uint32_t           actual_events = 0;
    uint32_t           service_error_code;
    uint32_t           error_code;
    run_profile_t      runp = {0};
    ARM_DRIVER_VERSION version;
    int                i;

    ARG_UNUSED(pvParameters);

    printf("\r\n \t\t >>> Dual Camera ISP demo with FreeRTOS is starting up!!! <<< \r\n");
    printf("\r\n \t\t ISP output: %" PRIu32 "x%" PRIu32 " YUYV\r\n",
           (uint32_t) ISP_OUTPUT_X,
           (uint32_t) ISP_OUTPUT_Y);

    printf("\n \t ISP YUYV buffer pool size: 0x%0X  pool addr: 0x%08" PRIX32 " \r\n ",
           (unsigned int) (RTE_ISP_BUFFER_COUNT * ISP_OUTPUT_SIZE_Y),
           (uint32_t) y_buffer);
    printf("\n \t RGB inst0 pool size: 0x%0X  pool addr: 0x%08" PRIX32 " \r\n ",
           (unsigned int) RGB888_BUFFER_SIZE,
           (uint32_t) isp_rgb_buffer_0);
    printf("\n \t RGB inst1 pool size: 0x%0X  pool addr: 0x%08" PRIX32 " \r\n ",
           (unsigned int) RGB888_BUFFER_SIZE,
           (uint32_t) isp_rgb_buffer_1);

    isp_buffer_init();

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h */
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        goto error_exit_no_teardown;
    }
#else
    /*
     * NOTE: The I2C and  Camera pins used in this test application are not configured
     * in the board support library. Therefore, it is being configured manually here.
     */
    ret = hardware_init();
    if (ret != 0) {
        printf("Error: CAMERA Hardware Initialize failed: %" PRId32 "\n", ret);
        goto error_exit_no_teardown;
    }
#endif

    /* Initialize the SE services */
    se_services_port_init();

    /* Enable MIPI Clocks */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              true,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 100MHz clock enable = %" PRId32 "\n", error_code);
        goto error_exit_no_teardown;
    }

    error_code =
        SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, true, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC) clock enable = %" PRId32 "\n", error_code);
        goto error_disable_100mhz_clk;
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nSE: get_run_cfg error = %" PRId32 "\n", error_code);
        goto error_disable_hfosc_clk;
    }

    /*
     * Note:
     * This demo uses a specific profile setting that only enables the
     * items it needs. For example, it only requests the RAM regions and
     * peripheral power that are relevant for this demo. If you want to adapt
     * this example for your own use case, you should adjust the profile setting
     * accordingly. You can either add any additional items that you need, or
     * remove the request altogether to use the default setting that turns on
     * almost everything.
     */

    runp.memory_blocks  = MRAM_MASK | SRAM0_MASK | SRAM1_MASK;
    runp.phy_pwr_gating = MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK | MIPI_RX_DPHY_MASK | LDO_PHY_MASK;

    /* Set the new run configuration */
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nSE: set_run_cfg error = %" PRId32 "\n", error_code);
        goto error_disable_hfosc_clk;
    }

    version = CAMERAdrv->GetVersion();
    printf("\r\n Camera driver version api:0x%" PRIx16 " driver:0x%" PRIx16 " \r\n", version.api,
           version.drv);

    ret = CAMERAdrv->Initialize(camera_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Initialize failed.\r\n");
        goto error_disable_hfosc_clk;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power Up failed.\r\n");
        goto error_uninitialize_camera;
    }

    /* Control configuration for camera controller */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                                 ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                                 ARM_CPI_EVENT_ERR_HARDWARE |
                                 ARM_CPI_EVENT_MIPI_CSI2_ERROR);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    uint32_t current_camera = 0;

    while (1) {
        uint8_t *rgb_buf = (current_camera == 0U) ? isp_rgb_buffer_0 : isp_rgb_buffer_1;

        printf("\r\n Let's Start Capturing Camera Frame via ISP (instance %" PRIu32
               ", %" PRIu32 "x%" PRIu32 ")...\r\n",
               current_camera,
               (uint32_t) ISP_OUTPUT_X,
               (uint32_t) ISP_OUTPUT_Y);

        /* Rebinds CPI mux/CSI and ISP params for this instance */
        ret = CAMERAdrv->Control(CPI_SELECT_CAMERA_SENSOR, current_camera);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: CAMERA select instance %" PRIu32 " failed.\r\n", current_camera);
            goto error_poweroff_camera;
        }

        for (i = 0; i < RTE_ISP_BUFFER_COUNT; i++) {
            ret = CAMERAdrv->Control(ISP_CONTROL_QBUF, (uint32_t) &buffer_array[i]);
            if (ret != ARM_DRIVER_OK) {
                printf("\r\n Error: ISP QBUF failed.\r\n");
                goto error_poweroff_camera;
            }
        }

	/* Discard notifications left by the previous, now-stopped capture. */
        (void) xTaskNotifyWait(0,
                               ISP_MI_FRAME_DUMP_EVENT | CAM_CB_EVENT_ERROR,
                               &actual_events,
                               0);

        ret = CAMERAdrv->CaptureVideo((void *) 0xDEADC0DE);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: CAMERA CaptureVideo failed.\r\n");
            goto error_poweroff_camera;
        }

        /* wait till any event success/error comes in isr callback.
         * 5 second timeout, same as the ThreadX demo.
         */
        if (xTaskNotifyWait(0,
                            ISP_MI_FRAME_DUMP_EVENT | CAM_CB_EVENT_ERROR,
                            &actual_events,
                            pdMS_TO_TICKS(5000)) != pdTRUE) {
            printf("Error: CAMERA xTaskNotifyWait failed.\n");
            if (CAMERAdrv->Stop() != ARM_DRIVER_OK) {
                printf("\r\n Error: CAMERA stop Capture failed.\r\n");
                goto error_exit_no_teardown;
            }
            goto error_poweroff_camera;
        }

        if (actual_events & CAM_CB_EVENT_ERROR) {
            /* Error: ISP Capture Frame failed. */
            printf("\r\n \t\t >> Error: ISP Capture Frame failed. \r\n");
            if (CAMERAdrv->Stop() != ARM_DRIVER_OK) {
                printf("\r\n Error: CAMERA stop Capture failed.\r\n");
                goto error_exit_no_teardown;
            }
            goto error_poweroff_camera;
        }

        /* Okay, we have received Success: ISP frame dump detected.
         * now stop Camera Capture.
         */
        ret = CAMERAdrv->Control(ISP_PROCESS_FRAME_END, 0);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: ISP_PROCESS_FRAME_END failed.\r\n");
            (void) CAMERAdrv->Stop();
            goto error_poweroff_camera;
        }

        ret = CAMERAdrv->Stop();
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: CAMERA stop Capture failed.\r\n");
            goto error_exit_no_teardown;
        }

        SCB_InvalidateDCache_by_Addr((uint32_t *) y_buffer[0], ISP_OUTPUT_SIZE_Y);

        ret = aipl_color_convert_yuy2_to_rgb888(y_buffer[0],
                                                rgb_buf,
                                                ISP_OUTPUT_X,
                                                ISP_OUTPUT_X,
                                                ISP_OUTPUT_Y);
        if (ret != 0) {
            printf("\r\n Error: CAMERA image conversion failed.\r\n");
            goto error_poweroff_camera;
        }

        printf("\n AIPL RGB888 image memory address is :");
        printf("\n \t starting_addr: 0x%" PRIX32 " \r\n \t ending_addr  : 0x%" PRIX32 "\n",
               (uint32_t) rgb_buf,
               (uint32_t) (rgb_buf + RGB888_BUFFER_SIZE - 1));

        printf("\n To dump memory using ARM DS with Ulink Pro Debugger or Trace32 :");
        printf("\n  Use below commands in Commands tab: update user directory name \r\n");

        printf("Ulink:\n   dump binary memory /home/user/camera_dump/cam_image%" PRIu32
               "_rgb888.bin 0x%" PRIX32 " 0x%" PRIX32 " \r\n",
               current_camera,
               (uint32_t) rgb_buf,
               (uint32_t) (rgb_buf + RGB888_BUFFER_SIZE - 1));
        printf("T32:\n   data.save.binary /home/user/camera_dump/cam_image%" PRIu32
               "_rgb888.bin 0x%" PRIX32 "--0x%" PRIX32 " \r\n",
               current_camera,
               (uint32_t) rgb_buf,
               (uint32_t) (rgb_buf + RGB888_BUFFER_SIZE - 1));

        printf("\n  This command will dump memory from staring address to ending address \r");
        printf("\n  and store it in to given path with filename.\r\n\r\n");

        printf("\r\n\r\n XXX Camera demo thread is halting here! XXX...\r\n");
        printf("\r\n Dump the captured image from memory using a debugger!!!\r\n");

        current_camera = (current_camera == 0U) ? 1U : 0U;
    }

error_poweroff_camera:
    /* Power off CAMERA peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power OFF failed.\r\n");
    }

error_uninitialize_camera:
    /* Un-initialize CAMERA driver */
    ret = CAMERAdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");
    }

error_disable_hfosc_clk:
    error_code =
        SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, false, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC)  clock disable = %" PRId32 "\n", error_code);
    }

error_disable_100mhz_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              false,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 100MHz clock disable = %" PRId32 "\n", error_code);
    }

error_exit_no_teardown:
    printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");

    /* wait forever */
    WAIT_FOREVER_LOOP
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start the FreeRTOS Kernel
 *----------------------------------------------------------------------------
 */
int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t ret;

    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* System Initialization */
    SystemCoreClockUpdate();

    /* Create application main thread */
    BaseType_t xReturned = xTaskCreate(camera_demo_thread_entry,
                                       "camera_demo_thread_entry",
                                       STACK_SIZE,
                                       0,
                                       configMAX_PRIORITIES - 1,
                                       &camera_xHandle);
    if (xReturned != pdPASS) {

        vTaskDelete(camera_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
