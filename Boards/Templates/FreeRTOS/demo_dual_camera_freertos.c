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
 * @file     : demo_dual_camera_freertos.c
 * @author   : Chandra Bhushan Singh
 * @email    : chandrabhushan.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 12-October-2026
 * @brief    : TestApp to verify dual camera capture (ARX3A0 and OV5675)
 *             with FREERTOS as an Operating System.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

//* System Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* Cpi Driver */
#include "Driver_CPI.h"
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
#include "aipl_crop.h"
#include "aipl_demosaic.h"

/* SE Services */
#include "se_services_port.h"

/*RTOS Includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "app_utils.h"

// Set to 0: Use application-defined arx3A0 pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI  Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

/*Define for FreeRTOS*/
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

/* Dual camera,
 *   instance 0 : ARX3A0  resolution from RTE
 *   instance 1 : OV5675  resolution from RTE
 */
#define FRAME_WIDTH_0         RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH
#define FRAME_HEIGHT_0        RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT
#define FRAME_WIDTH_1         RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH
#define FRAME_HEIGHT_1        RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT

#define FRAME_SIZE_0          ((FRAME_WIDTH_0) * (FRAME_HEIGHT_0))
#define FRAME_SIZE_1          ((FRAME_WIDTH_1) * (FRAME_HEIGHT_1))
#define FRAMEBUFFER_POOL_SIZE ((FRAME_SIZE_0 > FRAME_SIZE_1) ? FRAME_SIZE_0 : FRAME_SIZE_1)

/* Shared Bayer capture buffer. Sized for the larger of the two frames.
 */
uint8_t framebuffer_pool[FRAMEBUFFER_POOL_SIZE]
    __attribute__((section(".bss.camera_frame_buf"), aligned(8)));

/* AIPL aipl_demosaic_rgb888 writes packed RGB888: width x height x 3. */
#define RGB_BYTES_PER_PIXEL    3U
#define RGB888_BUFFER_SIZE(w, h)                                                                   \
    ((uint32_t) (w) * (uint32_t) (h) * RGB_BYTES_PER_PIXEL)

#define RGB888_BUFFER_SIZE_0   RGB888_BUFFER_SIZE(FRAME_WIDTH_0, FRAME_HEIGHT_0)
#define RGB888_BUFFER_SIZE_1   RGB888_BUFFER_SIZE(FRAME_WIDTH_1, FRAME_HEIGHT_1)

/* Per-sensor RGB888 pools. */
uint8_t bayer_to_rgb_buffer_0[RGB888_BUFFER_SIZE_0]
    __attribute__((section(".bss.camera_frame_bayer_to_rgb_buf"), aligned(8)));
uint8_t bayer_to_rgb_buffer_1[RGB888_BUFFER_SIZE_1]
    __attribute__((section(".bss.camera_frame_bayer_to_rgb_buf"), aligned(8)));

/* Camera callback events */
typedef enum {
    CAM_CB_EVENT_CAPTURE_STOPPED = (1 << 0),
    CAM_CB_EVENT_ERROR           = (1 << 1)
} CAMERA_CB_EVENTS;

/**
  \fn          void camera_callback(uint32_t event)
  \brief       Camera isr callback
  \param[in]   event: Camera Event
  \return      none
  */
void camera_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED) {
        /* Transfer Success: Capture Stop detected, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_CAPTURE_STOPPED,
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
        /* CSI PHY/packet error. Without this, capture just times out. */
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
  \fn          int32_t i2c_pinmux(void)
  \brief       i2c hardware pin initialization:
                  - PIN-MUX configuration
                  - PIN-PAD configuration
  \param[in]   none
  \return      0:success; -1:failure
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
  \fn          int32_t camera_pinmux(void)
  \brief       Camera hardware pin initialization:
                   - PIN-MUX configuration
  \param[in]   none
  \return      0:success; -1:failure
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
  \fn          int32_t hardware_init(void)
  \brief       - i2c hardware pin initialization:
                   - PIN-MUX configuration
                   - PIN-PAD configuration
               - Camera hardware pin initialization:
                   - PIN-MUX configuration
  \param[in]   none
  \return      0:success; -1:failure
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
  \fn          void camera_demo_thread_entry(void *pvParameters)
  \brief       TestApp to verify dual camera capture with
               FREERTOS as an Operating System.
               This demo thread does:
                    - initialize i3c and Camera hardware pins
                      using PinMux Driver;
                    - initialize DPHY Tx.
                    - initialize Camera driver
                    - capture one frame from each camera instance
                    - captured data will be stored in to allocated
                      frame buffer address
                    - stop Camera capture
                    - demosaic RAW8 to RGB888 with aipl_demosaic_rgb888
                    - dump captured image data from memory address
                      using any debugger
                    - display image
  @param       pvParameters.
  \return      none
  */
void camera_demo_thread_entry(void *pvParameters)
{
    int32_t            ret           = 0;
    uint32_t           actual_events = 0;
    uint32_t           service_error_code;
    uint32_t           error_code;
    run_profile_t      runp = {0};
    ARM_DRIVER_VERSION version;
    ARG_UNUSED(pvParameters);

    printf("\r\n \t\t >>> Dual Camera Sensor demo with FreeRTOS is starting up!!! <<< \r\n");
    printf("\r\n \t\t instance 0 ARX3A0: %" PRIu32 "x%" PRIu32 " \r\n",
           (uint32_t) FRAME_WIDTH_0,
           (uint32_t) FRAME_HEIGHT_0);
    printf("\r\n \t\t instance 1 OV5675: %" PRIu32 "x%" PRIu32 " \r\n",
           (uint32_t) FRAME_WIDTH_1,
           (uint32_t) FRAME_HEIGHT_1);

    /* Allocated memory address for
     *   - Camera frame buffer and
     *   - Camera frame buffers for Bayer to RGB Conversion
     */
    printf("\n \t frame buffer        pool size: 0x%0X  pool addr: 0x%08" PRIx32 " \r\n ",
           FRAMEBUFFER_POOL_SIZE,
           (uint32_t) framebuffer_pool);

    printf("\n \t bayer_to_rgb inst0 pool size: 0x%0X  pool addr: 0x%08" PRIX32 " \r\n ",
           RGB888_BUFFER_SIZE_0,
           (uint32_t) bayer_to_rgb_buffer_0);
    printf("\n \t bayer_to_rgb inst1 pool size: 0x%0X  pool addr: 0x%08" PRIX32 " \r\n ",
           RGB888_BUFFER_SIZE_1,
           (uint32_t) bayer_to_rgb_buffer_1);

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h */
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#else
    /*
     * NOTE: The I2C and  Camera pins used in this test application are not configured
     * in the board support library. Therefore, it is being configured manually here.
     */
    ret = hardware_init();
    if (ret != 0) {
        printf("Error: CAMERA Hardware Initialize failed: %" PRId32 "\n", ret);
        return;
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
        return;
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
    error_code          = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
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
        goto error_uninitialize_camera;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /*Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_HARDWARE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    uint32_t current_camera = 0;

    while (1) {
        uint32_t frame_width  = (current_camera == 0U) ? FRAME_WIDTH_0 : FRAME_WIDTH_1;
        uint32_t frame_height = (current_camera == 0U) ? FRAME_HEIGHT_0 : FRAME_HEIGHT_1;
        uint32_t frame_bytes = frame_width * frame_height;
        uint8_t *rgb_buf     = (current_camera == 0U) ? bayer_to_rgb_buffer_0
                                                      : bayer_to_rgb_buffer_1;
        uint32_t rgb_bytes   = (current_camera == 0U) ? RGB888_BUFFER_SIZE_0 : RGB888_BUFFER_SIZE_1;

        printf("\r\n Let's Start Capturing Camera Frame (instance %" PRIu32 ", %" PRIu32 "x%" PRIu32
               ")...\r\n",
               current_camera,
               frame_width,
               frame_height);
        memset(framebuffer_pool, 0x00, frame_bytes);

        ret = CAMERAdrv->Control(CPI_SELECT_CAMERA_SENSOR, current_camera);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: CAMERA select instance %" PRIu32 " failed.\r\n", current_camera);
            goto error_poweroff_camera;
        }

        ret = CAMERAdrv->CaptureFrame(framebuffer_pool);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: CAMERA Capture Frame failed.\r\n");
            goto error_poweroff_camera;
        }

        /* wait till any event success/error comes in isr callback.
         * 5 second timeout, same as the ThreadX demo.
         */
        if (xTaskNotifyWait(0,
                            CAM_CB_EVENT_CAPTURE_STOPPED | CAM_CB_EVENT_ERROR,
                            &actual_events,
                            pdMS_TO_TICKS(5000)) != pdTRUE) {
            printf("Error: CAMERA xTaskNotifyWait failed.\n");
            goto error_poweroff_camera;
        }

        if (!(actual_events & CAM_CB_EVENT_CAPTURE_STOPPED) &&
            (actual_events & CAM_CB_EVENT_ERROR)) {
            /* Error: Camera Capture Frame failed. */
            printf("\r\n \t\t >> Error: CAMERA Capture Frame failed. \r\n");
            goto error_poweroff_camera;
        }

        /* Okay, we have received Success: Camera Capture Frame stop detected.
         * now stop Camera Capture.
         */
        ret = CAMERAdrv->Stop();
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: CAMERA stop Capture failed.\r\n");
            goto error_poweroff_camera;
        }

        ret = aipl_demosaic_rgb888(framebuffer_pool,
                                   rgb_buf,
                                   frame_width,
                                   frame_width,
                                   frame_height,
                                   AIPL_BAYER_GBRG);
        if (ret != 0) {
            printf("\r\n Error: CAMERA image conversion failed.\r\n");
            goto error_poweroff_camera;
        }

        printf("\n AIPL RGB888 image memory address is :");
        printf("\n \t starting_addr: 0x%" PRIX32 " \r\n \t ending_addr  : 0x%" PRIX32 "\n",
               (uint32_t) rgb_buf,
               (uint32_t) (rgb_buf + rgb_bytes - 1));

        printf("\n To dump memory using ARM DS with Ulink Pro Debugger or Trace32 :");
        printf("\n  Use below commands in Commands tab: update user directory name \r\n");

        printf("Ulink:\n   dump binary memory /home/user/camera_dump/cam_image%" PRIu32
               "_rgb888.bin 0x%" PRIX32 " 0x%" PRIX32 " \r\n",
               current_camera,
               (uint32_t) rgb_buf,
               (uint32_t) (rgb_buf + rgb_bytes - 1));
        printf("T32:\n   data.save.binary /home/user/camera_dump/cam_image%" PRIu32
               "_rgb888.bin 0x%" PRIX32 "--0x%" PRIX32 " \r\n",
               current_camera,
               (uint32_t) rgb_buf,
               (uint32_t) (rgb_buf + rgb_bytes - 1));

        printf("\n  This command will dump memory from staring address to ending address \r");
        printf("\n  and store it in to given path with filename.\r\n\r\n");

        printf("\r\n\r\n XXX Camera demo thread is halting here! XXX...\r\n");
        printf("\r\n Now User can dump captured/converted image data from memory address using any "
               "debugger!!!\r\n");

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

    printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");

    /* wait forever */
    WAIT_FOREVER_LOOP
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
