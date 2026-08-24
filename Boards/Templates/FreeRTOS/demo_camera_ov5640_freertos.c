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
 * @file     : demo_camera_ov5640_freertos.c
 * @author   : Chandra Bhushan Singh
 * @email    : chandrabhushan.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 11-Mar-2026
 * @brief    : TestApp to verify OV5640 Camera Sensor with
 *             FREERTOS as an Operating System.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <inttypes.h>

/* Project Includes */
/* Camera Controller Driver */
#include "Driver_CPI.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/* PINMUX Driver */
#include "pinconf.h"
#include "board_config.h"

/* Rtos include */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "app_utils.h"
#include "Driver_IO.h"

/* Set to 0: Use application-defined ov5640 camera pin configuration. */
/* Set to 1: Use Conductor-generated pin configuration (from pins.h). */
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI  Driver_LPCPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_LPCPI;

/* Define for FreeRTOS */
#define CAMERA_TASK_STACK_SIZE        216
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

/* Thread id of thread */
TaskHandle_t camera_xHandle;

extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(BOARD_LPCAM_ENBUF_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_LPCAM_ENA =
    &ARM_Driver_GPIO_(BOARD_LPCAM_ENBUF_GPIO_PORT);

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

/* @Note: OV5640 Camera Sensor configurations
 *        are directly borrowed from OV5640 Camera Sensor drivers,
 *        for detail refer OV5640 driver.
 *
 * Selected OV5640 Camera Sensor configurations:
 *   - Interface     : Parallel
 *   - Resolution    : 160X120
 *   - Output Format : RGB565
 */

/* Supported OV5640 Camera Sensor Output Format.
 */
#define FRAME_WIDTH  (160)
#define FRAME_HEIGHT (120)

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for Camera frame buffer:
 *  which will be frame width x frame height X 2 for RGB565
 */
#define FRAMEBUFFER_POOL_SIZE ((FRAME_WIDTH) * (FRAME_HEIGHT) * 2)

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
uint8_t framebuffer_pool[FRAMEBUFFER_POOL_SIZE] __attribute__((section(".bss.camera_frame_buf")));

/* Camera callback events */
typedef enum {
    CAM_CB_EVENT_FRAME_VSYNC_DETECTED = (1 << 0),
    CAM_CB_EVENT_CAPTURE_STOPPED      = (1 << 1),
    CAM_CB_EVENT_ERROR                = (1 << 2)
} CAMERA_CB_EVENTS;

/*
 * \fn          void camera_callback(uint32_t event)
 * \brief       Camera isr callback
 * \param[in]   event: Camera Event
 * \return      none
 */
void camera_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED) {
        /* Transfer Success: Frame VSYNC detected, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_FRAME_VSYNC_DETECTED,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

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
        /* Transfer Error: Received Input FIFO over-run, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(camera_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN) {
        /* Transfer Error: Received Output FIFO over-run, Wake-up Thread. */
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
}

#if !(USE_CONDUCTOR_TOOL_PINS_CONFIG)
/*
 * \fn          int32_t i2c_pinmux(void)
 * \brief       i2c hardware pin initialization:
 *                - PIN-MUX configuration
 *                - PIN-PAD configuration
 * \param[in]   none
 * \return      0:success; -1:failure
 */
int32_t i2c_pinmux(void)
{
    int32_t ret;

    /* Configure GPIO Pin as i2c_sda
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_LPCAM_I2C_SDA_GPIO_PORT),
                      BOARD_LPCAM_I2C_SDA_GPIO_PIN,
                      BOARD_LPCAM_I2C_SDA_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Configure GPIO Pin as i2c_scl
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_LPCAM_I2C_SCL_GPIO_PORT),
                      BOARD_LPCAM_I2C_SCL_GPIO_PIN,
                      BOARD_LPCAM_I2C_SCL_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return 0;
}

/**
 * @fn      static int32_t board_camera_pins_config(void)
 * @brief   Configure lpcamera pinmux settings not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_camera_pins_config(void)
{
    int32_t ret;

    /* Configure GPIO Pin as lpcam_hsync_c */
    ret = pinconf_set(PORT_(BOARD_LPCAM_HSYNC_GPIO_PORT),
                      BOARD_LPCAM_HSYNC_GPIO_PIN,
                      BOARD_LPCAM_HSYNC_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Configure GPIO Pin as lpcam_vsync_c */
    ret = pinconf_set(PORT_(BOARD_LPCAM_VSYNC_GPIO_PORT),
                      BOARD_LPCAM_VSYNC_GPIO_PIN,
                      BOARD_LPCAM_VSYNC_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Configure GPIO Pin as lpcam_pclk_c */
    ret = pinconf_set(PORT_(BOARD_LPCAM_PCLK_GPIO_PORT),
                      BOARD_LPCAM_PCLK_GPIO_PIN,
                      BOARD_LPCAM_PCLK_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Data Lines: D0-D7 */
    ret = pinconf_set(PORT_(BOARD_LPCAM_D0_GPIO_PORT),
                      BOARD_LPCAM_D0_GPIO_PIN,
                      BOARD_LPCAM_D0_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_LPCAM_D1_GPIO_PORT),
                      BOARD_LPCAM_D1_GPIO_PIN,
                      BOARD_LPCAM_D1_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_LPCAM_D2_GPIO_PORT),
                      BOARD_LPCAM_D2_GPIO_PIN,
                      BOARD_LPCAM_D2_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_LPCAM_D3_GPIO_PORT),
                      BOARD_LPCAM_D3_GPIO_PIN,
                      BOARD_LPCAM_D3_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_LPCAM_D4_GPIO_PORT),
                      BOARD_LPCAM_D4_GPIO_PIN,
                      BOARD_LPCAM_D4_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_LPCAM_D5_GPIO_PORT),
                      BOARD_LPCAM_D5_GPIO_PIN,
                      BOARD_LPCAM_D5_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_LPCAM_D6_GPIO_PORT),
                      BOARD_LPCAM_D6_GPIO_PIN,
                      BOARD_LPCAM_D6_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_LPCAM_D7_GPIO_PORT),
                      BOARD_LPCAM_D7_GPIO_PIN,
                      BOARD_LPCAM_D7_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return 0;
}

#endif

#if !(USE_CONDUCTOR_TOOL_PINS_CONFIG)
/*
 * \fn          int32_t hardware_init(void)
 * \brief       - i2c hardware pin initialization:
 *                - PIN-MUX configuration
 *                - PIN-PAD configuration
 *              - Camera hardware pin initialization:
 *                - PIN-MUX configuration
 * \param[in]   none
 * \return      0:success; -1:failure
 */
int32_t hardware_init(void)
{
    int32_t ret;

    /* i2c pinmux. */
    ret = i2c_pinmux();
    if (ret != 0) {
        printf("\r\n Error in i2c pinmux.\r\n");
        return ret;
    }

    /* Camera pinmux. */
    ret = board_camera_pins_config();
    if (ret != 0) {
        printf("\r\n Error in Camera pinmux.\r\n");
        return ret;
    }

    return 0;
}
#endif

/*
 * \fn          void camera_demo_thread_entry(void *pvParameters)
 * \brief       TestApp to verify OV5640 Camera Sensor with
 *               FREERTOS as an Operating System.
 *
 *              This demo thread does:
 *                - initialize i2c and Camera hardware pins
 *                   using PinMux Driver;
 *                - initialize Camera driver
 *                - capture one frame
 *                  - captured data will be stored in to allocated
 *                    frame buffer address
 *                - stop Camera capture
 *                - dump captured RGB565 image data from memory address
 *                   using any debugger
 * @param       pvParameters.
 * \return      none
 */
void camera_demo_thread_entry(void *pvParameters)
{
    int32_t  ret           = 0;
    uint32_t actual_events = 0;

    ARM_DRIVER_VERSION version;

    ARG_UNUSED(pvParameters);

    printf("\r\n \t\t >>> OV5640 Camera Sensor demo with FREERTOS is starting up!!! <<< \r\n");

    /* Allocated memory address for Camera RGB565 frame buffer. */
    printf("\n \t frame buffer        pool size: %u  pool addr: 0x%08" PRIX32 " \r\n ",
            FRAMEBUFFER_POOL_SIZE, (uint32_t) framebuffer_pool);

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
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = hardware_init();
    if (ret != 0) {
        printf("Error: CAMERA Hardware Initialize failed: %" PRId32 "\n", ret);
        return;
    }
#endif

    /* Enable the camera buffer level shifter */
    ret = GPIO_Driver_LPCAM_ENA->Initialize(BOARD_LPCAM_ENBUF_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        return;
    }

    ret = GPIO_Driver_LPCAM_ENA->PowerControl(BOARD_LPCAM_ENBUF_GPIO_PIN,
                                            ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return;
    }

    ret = GPIO_Driver_LPCAM_ENA->SetDirection(BOARD_LPCAM_ENBUF_GPIO_PIN,
                                            GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return;
    }

    ret = GPIO_Driver_LPCAM_ENA->SetValue(BOARD_LPCAM_ENBUF_GPIO_PIN,
                                        GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return;
    }

    version = CAMERAdrv->GetVersion();
    printf("\r\n Camera driver version api:0x%" PRIx16 " driver:0x%" PRIx16 " \r\n", version.api,
            version.drv);

    ret = CAMERAdrv->Initialize(camera_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Initialize failed.\r\n");
        return;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power Up failed.\r\n");
        goto error_uninitialize;
    }

    /* Wait sometime for Camera Sensor to setup,
     *  otherwise captured image quality will not be good.
     *  User can adjust this delay as per Camera Sensor.
     *
     * @Observation for OV5640 Camera Sensor:
     *  - Proper delay is required for:
     *    - Camera Sensor to setup after Soft Reset.
     *    - Camera Sensor Lens to come-out from Shutter and gets steady,
     *       otherwise captured image will be less bright/dull.
     *       adjust this delay if captured image is not proper.
     */

    /* Control configuration for CPI */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Configuration failed.\r\n");
        goto error_poweroff;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Camera Sensor Configuration failed.\r\n");
        goto error_poweroff;
    }

    /* Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_HARDWARE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Camera Events Configuration failed.\r\n");
        goto error_poweroff;
    }

    printf("\r\n Wait for sometime for Camera Sensor to setup,");
    printf("\r\n  otherwise captured image quality will not be good,");
    printf("\r\n  User can adjust this delay as per Camera Sensor.\r\n");

    /* Let's Start Capturing Camera Frame...
     *   CPI will capture one frame,
     *   (store data in to allocated frame buffer address)
     *   then it gets stop.
     */
    printf("\r\n Let's Start Capturing Camera Frame...\r\n");
    ret = CAMERAdrv->CaptureFrame(framebuffer_pool);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Capture Frame failed.\r\n");
        goto error_poweroff;
    }

    /* wait till any event to comes in isr callback */
    xTaskNotifyWait(0,
                    CAM_CB_EVENT_CAPTURE_STOPPED | CAM_CB_EVENT_ERROR,
                    &actual_events,
                    portMAX_DELAY);

    if (!(actual_events & CAM_CB_EVENT_CAPTURE_STOPPED) && (actual_events & CAM_CB_EVENT_ERROR)) {
        /* Error: Camera Capture Frame failed. */
        printf("\r\n \t\t >> Error: CAMERA Capture Frame failed. \r\n");
        goto error_poweroff;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Okay, we have received Success: Camera Capture Frame stop detected.
     * now stop Camera Capture.
     */
    ret = CAMERAdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA stop Capture failed.\r\n");
        goto error_poweroff;
    }

    /* How to dump captured RGB565 image data from memory address?
     *  To dump memory using ARM DS(Development Studio) and Ulink Pro Debugger
     *
     *  Use below command in "Commands" tab:
     *   dump binary memory path_with_filename.fileformat starting_address ending_address
     *
     *   example:(update user directory name)
     *    dump binary memory /home/user/camera_dump/cam_image0_160p.bin 0x20020000 0x200295FF
     *
     *   This command will dump memory from starting address to ending address
     *   and store it in to given path with filename.
     */
    printf("\n To dump memory using ARM DS and Ulink Pro Debugger:");
    printf("\n  Use below command in Commands tab: update user directory name \r\n");
    printf("\n   dump binary memory /home/user/camera_dump/cam_image0_160p.bin ");
    printf("0x%" PRIX32 " 0x%" PRIX32 " \r\n", (uint32_t) framebuffer_pool,
           (uint32_t) (framebuffer_pool + FRAMEBUFFER_POOL_SIZE - 1));

    printf("\n  This command will dump memory from starting address to ending address \r");
    printf("\n  and store it in to given path with filename.\r\n\r\n");

    printf("\r\n\r\n XXX Camera demo thread is halting here! XXX...\r\n");
    printf("\r\n Now User can dump captured RGB565 image data ");
    printf("from memory address using any debugger!!!\r\n");

    /* wait forever. */
    WAIT_FOREVER_LOOP

error_poweroff : 
    /* Power off CAMERA peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power OFF failed.\r\n");
    }

error_uninitialize:
    /* Un-initialize CAMERA driver */
    ret = CAMERAdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");
    }

    printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start the FreeRTOS Kernel
 *----------------------------------------------------------------------------
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
    BaseType_t xReturned = xTaskCreate(camera_demo_thread_entry,
                                       "camera_demo_thread_entry",
                                       CAMERA_TASK_STACK_SIZE,
                                       0,
                                       configMAX_PRIORITIES - 1,
                                       &camera_xHandle);
    if (xReturned != pdPASS) {
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
