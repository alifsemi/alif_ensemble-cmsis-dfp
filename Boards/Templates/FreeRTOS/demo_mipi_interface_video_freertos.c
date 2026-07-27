/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     demo_mipi_interface_video_freertos.c
 * @author   Chandra Bhushan Singh
 * @email    chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     04-Sep-2025
 * @brief    TestApp to verify ARX3A0 Camera Sensor and ILI9806E LCD Panel
 *           with FREERTOS as an Operating System.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include <inttypes.h>

#include "RTE_Components.h"
#include "RTE_Device.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

#include "board_config.h"

#include "pinconf.h"

#include "Driver_CPI.h"
#include "Driver_CDC200.h"

#include "se_services_port.h"

/* AIPL
 *
 * This application requires the AIPL pack. In the RTE configuration
 * (RTE -> Graphics), enable all AIPL-related components before building.
 */
#include "aipl_error.h"
#include "aipl_crop.h"
#include "aipl_demosaic.h"

/*RTOS Includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "app_utils.h"

#if defined(RTE_CPI_AXI_PORT)
#if (RTE_CPI_AXI_PORT != 1)
#error "Enable the CPI AXI port in RTE_Device.h"
#endif
#endif

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

/*CDC200 Driver instance*/
extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

/*Define for FreeRTOS*/
#define STACK_SIZE                    1024
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

/* Thread id of thread */
TaskHandle_t video_xHandle;

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
    (void) pcTaskName;

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
/* Camera Sensor Selection
 * Supports: ARX3A0, MT9M114, OV5675
 *
 * Selection is driven by the RTE components enabled in RTE_Components.h.
 * Enable the corresponding CAMERA_SENSOR driver in the RTE configuration.
 *
 *   - RTE_Drivers_CAMERA_SENSOR_ARX3A0  (560x560,  IPI-16 RAW8)
 *   - RTE_Drivers_CAMERA_SENSOR_MT9M114 (1280x720, IPI-16 RAW8)
 *   - RTE_Drivers_CAMERA_SENSOR_OV5675  (1296x972, IPI-16 RAW8)
 *
 * Note: Sensors output RAW10, but CPI interface default is IPI-16 RAW8.
 *       CPI color mode can be changed via RTE configuration if needed.
 */

#if defined(RTE_Drivers_CAMERA_SENSOR_ARX3A0)
  #define SELECTED_CAMERA_SENSOR     "ARX3A0"
  #define CAM_FRAME_WIDTH            RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH
  #define CAM_FRAME_HEIGHT           RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT

#elif defined(RTE_Drivers_CAMERA_SENSOR_MT9M114)
  #define SELECTED_CAMERA_SENSOR     "MT9M114"
  #define CAM_FRAME_WIDTH            RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH
  #define CAM_FRAME_HEIGHT           RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT

#elif defined(RTE_Drivers_CAMERA_SENSOR_OV5675)
  #define SELECTED_CAMERA_SENSOR     "OV5675"
  #define CAM_FRAME_WIDTH            RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH
  #define CAM_FRAME_HEIGHT           RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT

#else
  #error "Enable one RTE_Drivers_CAMERA_SENSOR_* definition"
#endif

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

#if BOARD_CAMERA_HAS_STREAM_BIT_ENABLED
#define N_FRAMEBUFF            2
#define MAX_FRAMEBUFFERS       N_FRAMEBUFF

static void *framebuffers[N_FRAMEBUFF];

static volatile uint32_t completed_buf_idx;
#endif

#define CAMERA_FRAMEBUFFER_ATTR  __attribute__((section(".bss.camera_frame_buf")))

/* pool size for Camera frame buffer:
 *  which will be camera frame width x frame height
 */
#define CAMERA_FRAMEBUFFER_POOL_SIZE   ((CAM_FRAME_WIDTH) * (CAM_FRAME_HEIGHT))

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
#if BOARD_CAMERA_HAS_STREAM_BIT_ENABLED
uint8_t cam_framebuffer_pool[CAMERA_FRAMEBUFFER_POOL_SIZE * MAX_FRAMEBUFFERS]
        CAMERA_FRAMEBUFFER_ATTR;
#else
uint8_t cam_framebuffer_pool[CAMERA_FRAMEBUFFER_POOL_SIZE]
        CAMERA_FRAMEBUFFER_ATTR;
#endif

/* @Note: ILI9806E LCD Panel configurations
 *        are directly borrowed from ILI9806E LCD Panel drivers,
 *        for detail refer ILI9806E LCD Panel driver.
 *
 * Selected ILI9806E LCD Panel configurations:
 *   - Interface     : MIPI DSI
 *   - Resolution    : 480X800
 *   - Input Format  : RGB888
 */

/* ILI9806E LCD Panel Resolution. */
#define ILI9806E_Panel_RESOLUTION_480x800          0
#define ILI9806E_Panel_RESOLUTION                  ILI9806E_Panel_RESOLUTION_480x800

#if (ILI9806E_Panel_RESOLUTION == ILI9806E_Panel_RESOLUTION_480x800)
#define LCD_FRAME_WIDTH        (480)
#define LCD_FRAME_HEIGHT       (800)
#endif

/*Number of bytes per pixel in RGB888*/
#define RGB_BYTES_PER_PIXEL 3

/* Allocate LCD Panel buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

#define LCD_FRAMEBUFFER_ATTR  __attribute__((section(".bss.lcd_frame_buf")))

/* pool size for LCD Panel buffer:
 *  which will be LCD frame width x frame height
 */
#define LCD_FRAMEBUFFER_POOL_SIZE   ((LCD_FRAME_WIDTH) * (LCD_FRAME_HEIGHT) * RGB_BYTES_PER_PIXEL)

/* pool area for LCD panel frame buffer.
 *  Allocated in the "lcd_frame_buf" section.
 */
uint8_t lcd_framebuffer_pool[LCD_FRAMEBUFFER_POOL_SIZE]
        LCD_FRAMEBUFFER_ATTR;

/* Crop the captured camera image to LCD panel resolution using AIPL crop. */
#if (ILI9806E_Panel_RESOLUTION == ILI9806E_Panel_RESOLUTION_480x800)
#define CRP_FRAME_WIDTH        (480)
#define CRP_FRAME_HEIGHT       (480)
#endif

/* Required convert captured image data format to RGB image format.
 *
 *  - for ARX3A0 Camera sensor,
 *     selected Bayer output format:
 *      in-order to get the color image,
 *       Bayer format must be converted in to RGB format using AIPL demosaic.
 */

#define BAYER_TO_RGB_FRAMEBUFFER_ATTR   \
    __attribute__((section(".bss.camera_frame_bayer_to_rgb_buf")))

/* pool size for Camera frame buffer for Bayer to RGB conversion:
 *   frame width x frame height x RGB bytes per pixel.
 */
#define BAYER_TO_RGB_BUFFER_POOL_SIZE   \
    ((CAM_FRAME_WIDTH) * (CAM_FRAME_HEIGHT) * RGB_BYTES_PER_PIXEL)

/* pool area for Camera frame buffer for Bayer to RGB conversion.
 *  Allocated in the "camera_frame_bayer_to_rgb_buf" section.
 */
uint8_t bayer_to_rgb_buffer_pool[BAYER_TO_RGB_BUFFER_POOL_SIZE]
        BAYER_TO_RGB_FRAMEBUFFER_ATTR;

/* Camera callback events */
typedef enum {
    CAM_CB_EVENT_ERROR        = (1 << 0),
    DISP_CB_EVENT_ERROR       = (1 << 1),
    CAM_VSYNC_CB_EVENT        = (1 << 2)
} CB_EVENTS;

volatile uint32_t softreset_interval_counter;

/**
 * \fn          void Camera_callback(uint32_t event)
 * \brief       Camera isr callback
 * \param[in]   event: Camera Event
 * \return      none
 */
void Camera_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED) {
        /* Transfer Success: Capture Stop detected, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(video_xHandle,
                                     CAM_VSYNC_CB_EVENT,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
#if BOARD_CAMERA_HAS_STREAM_BIT_ENABLED
        completed_buf_idx = (event >> ARM_CPI_VSYNC_BUF_IDX_Pos) & 0x3U;
#else
        if (softreset_interval_counter % 2) {
            CAMERAdrv->Control(CPI_SOFTRESET, 0);
        }

        softreset_interval_counter++;
#endif
    }

    if (event & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN) {
        /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(video_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN) {
        /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(video_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (event & ARM_CPI_EVENT_MIPI_CSI2_ERROR) {
        /* Transfer Error: Received Hardware error, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(video_xHandle,
                                     CAM_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * \fn          void Display_callback(uint32_t event)
 * \brief       Display isr callback
 * \param[in]   event: Display Event
 * \return      none
 */
void Display_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdFALSE;

    if (event & ARM_CDC_DSI_ERROR_EVENT) {
        /* Transfer Error: Received Hardware error, Wake-up Thread. */
        xResult = xTaskNotifyFromISR(video_xHandle,
                                     DISP_CB_EVENT_ERROR,
                                     eSetBits,
                                     &xHigherPriorityTaskWoken);
        if (xResult == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * \fn          int32_t i2c_pinmux(void)
 * \brief       i2c hardware pin initialization:
 *                 - PIN-MUX configuration
 *                 - PIN-PAD configuration
 * \param[in]   none
 * \return      0:success; -1:failure
 */
int i2c_pinmux(void)
{
    int32_t ret;

    /* Configure GPIO Pin : P7_2 as I2C1_SDA_C
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_I2C1_SDA_C_GPIO_PORT),
                      BOARD_I2C1_SDA_C_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_5,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: i2c PINMUX and PINPAD failed.\r\n");
        return ret;
    }

    /* Configure GPIO Pin : P7_3 as I2C1_SCL_C
     * Pad function: PADCTRL_READ_ENABLE
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_I2C1_SCL_C_GPIO_PORT),
                      BOARD_I2C1_SCL_C_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_5,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: i2c PINMUX and PINPAD failed.\r\n");
        return ret;
    }

    return 0;
}

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

    return 0;
}

/**
 * \fn          int hardware_init(void)
 * \brief       - i2c hardware pin initialization:
 *                 - PIN-MUX configuration
 *                 - PIN-PAD configuration
 *              - Camera hardware pin initialization:
 *                 - PIN-MUX configuration
 * \param[in]   none
 * \return      0:success; -1:failure
 */
int hardware_init(void)
{
    int32_t ret;

    /* i2c pinmux. */
    ret = i2c_pinmux();
    if (ret != 0) {
        printf("\r\n Error in i2c pinmux.\r\n");
        return ret;
    }

    /* Camera pinmux. */
    ret = camera_pinmux();
    if (ret != 0) {
        printf("\r\n Error in Camera pinmux.\r\n");
        return ret;
    }

    /*MPU setup*/
    MPU_Setup();

    return 0;
}

/**
 * \fn          void video_demo_thread_entry(void *pvParameters)
 * \brief       TestApp to verify ARX3A0 Camera Sensor and ILI9806E LCD Panel
 *              with FreeRTOS as an Operating System.
 *              This demo thread does:
 *                  - initialize i3c and Camera hardware pins
 *                  - initialize CDC200 driver.
 *                  - initialize Camera driver with Camera Resolution.
 *                  - captured data will display on LCD Panel.
 *                  - convert captured image format
 *                    into RGB888 image format
 *                  - for ARX3A0 Camera sensor,
 *                    it gives image in frame resolution560x560
 *                  - for ILI9806E LCD Panel,
 *                    it supports resolution 480x800
 *                  - in-order to display the camera image on LCD panel,
 *                    we are cropping and interpolate the captured image
 *                    to 480x480.
 *                  - Stream captured image on LCD Panel
 * \param[in]   thread_input : thread input
 * \return      none
 */
void video_demo_thread_entry(void *pvParameters)
{
    int32_t   ret;
    uint32_t actual_events = 0;
    uint32_t  service_error_code;
    uint32_t  error_code;

    run_profile_t runp = {0};

    ARM_DRIVER_VERSION version;

#if BOARD_CONFIGURE_LVDS_MUX
    /* MIPI LVDS multiplexer pin */
    ret = board_gpios_config();
    if (ret != 0) {
        printf("Error in gpio pin configuration: %"PRId32"\n", ret);
        return;
    }
#endif

    printf("\r\n >>> %s Camera Sensor and ILI9806E LCD Panel demo ",
           SELECTED_CAMERA_SENSOR);
    printf("with FreeRTOS is starting up!!! <<< \r\n");

    /* Allocated memory address for
     *   - Camera frame buffer and
     *   - Camera frame buffer for Bayer to RGB Conversion.
     */
    printf("\n \t camera frame buffer pool size: 0x%"PRIx32" pool addr: 0x%"PRIx32" \r\n ",
            CAMERA_FRAMEBUFFER_POOL_SIZE, (uint32_t) cam_framebuffer_pool);

    printf("\n \t bayer_to_rgb buffer pool size: 0x%"PRIx32" pool addr: 0x%"PRIx32" \r\n ",
            BAYER_TO_RGB_BUFFER_POOL_SIZE, (uint32_t) bayer_to_rgb_buffer_pool);

/* Allocated memory address for
     *   - LCD frame buffer.
     */
    printf("\n \t lcd frame buffer pool size: 0x%"PRIx32" pool addr: 0x%"PRIx32" \r\n ",
            LCD_FRAMEBUFFER_POOL_SIZE, (uint32_t) lcd_framebuffer_pool);

    /* Initialize i3c and Camera hardware pins using PinMux Driver. */
    ret = hardware_init();
    if (ret != 0) {
        printf("\r\n Error: CAMERA Hardware Initialize failed.\r\n");
        return;
    }

    /* Initialize the SE services */
    se_services_port_init();

    /* Enable MIPI Clocks */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M,
                                              true, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 100MHz clock enable = %" PRId32 "\n", error_code);
        return;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC,
                                              true, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC) clock enable = %" PRId32 "\n", error_code);
        goto error_disable_100mhz_clk;
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
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

    runp.memory_blocks = MRAM_MASK | SRAM0_MASK | SRAM1_MASK;
    runp.phy_pwr_gating = MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK | MIPI_RX_DPHY_MASK | LDO_PHY_MASK;

    /* Set the new run configuration */
    error_code = SERVICES_set_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
    if (error_code) {
        printf("\r\nSE: set_run_cfg error = %" PRId32 "\n", error_code);
        goto error_disable_hfosc_clk;
    }

    version = CAMERAdrv->GetVersion();
    printf("\nCamera driver version api:%"PRIu16" driver:%"PRIu16"\r\n", version.api, version.drv);

    version = CDCdrv->GetVersion();
    printf("\n CDC driver version api:%"PRIu16" driver:%"PRIu16" \r\n", version.api, version.drv);

    /* Initialize Display Driver CDC200*/
    ret = CDCdrv->Initialize(Display_callback);
    if (ret != 0) {
        printf("\r\n Error: CDC200 Initialize failed.\r\n");
        return;
    }

    /* Initialize Camera Driver*/
    ret = CAMERAdrv->Initialize(Camera_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Initialize failed.\r\n");
        goto error_uninitialize_cdc;
    }

    /* Power on CDC200 peripheral*/
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != 0) {
        printf("\r\n Error: CDC200 Power Up failed.\r\n");
        goto error_uninitialize_camera;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power Up failed.\r\n");
        goto error_poweroff_cdc;
    }

    /* Configure the Display and set Frame buffer*/
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)(lcd_framebuffer_pool));
    if (ret != 0) {
        printf("\r\n Error: CDC200 Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Configure Camera */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
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
                             ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED|
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Start Display*/
    ret = CDCdrv->Start();
    if (ret != 0) {
        printf("\r\n Error: Starting CDC200 failed.\r\n");
        goto error_poweroff_camera;
    }

    printf("\r\n Let's Start Capturing Camera Frame...\r\n");

#if BOARD_CAMERA_HAS_STREAM_BIT_ENABLED
    framebuffers[0] = cam_framebuffer_pool;
    framebuffers[1] = cam_framebuffer_pool + CAMERA_FRAMEBUFFER_POOL_SIZE;

    /*Start Camera and capture image in given frame buffer*/
    ret = CAMERAdrv->CaptureVideo((void *)framebuffers);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Capturing Frame failed.\r\n");
        goto error_poweroff_camera;
    }
#else
    /*Start Camera and capture image in given frame buffer*/
    ret = CAMERAdrv->CaptureVideo(cam_framebuffer_pool);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Capturing Frame failed.\r\n");
        goto error_poweroff_camera;
    }
#endif

    /* Convert the camera captured image into RGB888,
     * crop and interpolate the image and
     * copy it to Display Frame buffer
     */
    for (;;) {
        /* wait till any event to comes in isr callback */
        xTaskNotifyWait(NULL,
                        CAM_VSYNC_CB_EVENT |
                        CAM_CB_EVENT_ERROR |
                        DISP_CB_EVENT_ERROR,
                        &actual_events,
                        portMAX_DELAY);

        if ((actual_events & CAM_CB_EVENT_ERROR) || (actual_events & DISP_CB_EVENT_ERROR)) {
            /* Error: Camera or Display failed. */
            printf("\r\n \t\t >> Error: Camera or Display failed. \r\n");
            goto error_poweroff_camera;
        }

#if BOARD_CAMERA_HAS_STREAM_BIT_ENABLED
        uint8_t *src_frame_buffer =
            (uint8_t *)framebuffers[completed_buf_idx & (N_FRAMEBUFF - 1U)];

        ret = aipl_demosaic_rgb888(src_frame_buffer,
                                   bayer_to_rgb_buffer_pool,
                                   CAM_FRAME_WIDTH,
                                   CAM_FRAME_WIDTH,
                                   CAM_FRAME_HEIGHT,
                                   AIPL_BAYER_GBRG);
#else
        ret = aipl_demosaic_rgb888(cam_framebuffer_pool,
                                   bayer_to_rgb_buffer_pool,
                                   CAM_FRAME_WIDTH,
                                   CAM_FRAME_WIDTH,
                                   CAM_FRAME_HEIGHT,
                                   AIPL_BAYER_GBRG);
#endif
        if (ret != 0) {
            printf("\r\n Error: CAMERA image conversion failed.\r\n");
            goto error_poweroff_camera;
        }

        ret = aipl_crop(bayer_to_rgb_buffer_pool,
                        lcd_framebuffer_pool,
                        CAM_FRAME_WIDTH,
                        CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT,
                        AIPL_COLOR_RGB888,
                        (CAM_FRAME_WIDTH - CRP_FRAME_WIDTH) / 2,
                        (CAM_FRAME_HEIGHT - CRP_FRAME_HEIGHT) / 2,
                        (CAM_FRAME_WIDTH + CRP_FRAME_WIDTH) / 2,
                        (CAM_FRAME_HEIGHT + CRP_FRAME_HEIGHT) / 2);
        if (ret != 0) {
            printf("\r\n Error: CAMERA AIPL crop failed.\r\n");
            goto error_poweroff_camera;
        }
    }

error_poweroff_camera:
    /* Power off CAMERA peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power OFF failed.\r\n");
    }

error_poweroff_cdc:
    /* Power off CDC200 peripheral */
    ret = CDCdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC200 Power OFF failed.\r\n");
    }

error_uninitialize_camera:
    /* Un-initialize CAMERA driver */
    ret = CAMERAdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");
    }

error_uninitialize_cdc:
    /* Un-initialize CDC200 driver */
    ret = CDCdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC200 Uninitialize failed.\r\n");
    }

error_disable_hfosc_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC,
                                              false, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC)  clock disable = %d\n", error_code);
    }

error_disable_100mhz_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M,
                                              false, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 100MHz clock disable = %d\n", error_code);
    }

    printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");

    /* wait forever */
    WAIT_FOREVER_LOOP
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start the FreeRTOS Kernel
 *---------------------------------------------------------------------------
 */
int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    int32_t    ret;

    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* System Initialization */
    SystemCoreClockUpdate();

    /* Create application main thread */
    BaseType_t xReturned = xTaskCreate(video_demo_thread_entry,
                                       "video_demo_thread_entry",
                                       216,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &video_xHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(video_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
