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
 * @file     : demo_camera_mt9m114_freertos.c
 * @author   : Chandra Bhushan Singh
 * @email    : chandrabhushan.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 07-Sept-2023
 * @brief    : TestApp to verify MT9M114 Camera Sensor with
 *             FREERTOS as an Operating System.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>

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

#define INSTANCE_CPI                   1
#define INSTANCE_LPCPI                 0

// Set to 0: Use application-defined mt9m114 camera pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#if INSTANCE_LPCPI
/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI  Driver_LPCPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_LPCPI;
#endif

#if INSTANCE_CPI
/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI  Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;
#endif

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

/* @Note: MT9M114 Camera Sensor configurations
 *        are directly borrowed from MT9M114 Camera Sensor drivers,
 *        for detail refer MT9M114 driver.
 *
 * Selected MT9M114 Camera Sensor configurations:
 *   - Interface     : Parallel
 *   - Resolution    : VGA 640x480
 *   - Output Format : RAW Bayer10
 */

/* Supported MT9M114 Camera Sensor Output Format.
 *  (Currently supports only RAW BAYER10 Format.)
 */
#define MT9M114_CAMERA_OUTPUT_FORMAT_RAW_BAYER10 0

/* User can select from supported MT9M114 Camera Sensor Output Format. */
#define MT9M114_USER_SELECT_CAMERA_OUTPUT_FORMAT MT9M114_CAMERA_OUTPUT_FORMAT_RAW_BAYER10

/* For MT9M114 Camera Sensor RAW BAYER Output Format:
 * As per data-sheet "AND9534/D"
 *  section: Obtaining Bayer Data (Table 18),
 *   When using any of the RAW Bayer modes,
 *    it is essential that the user adds on the
 *    additional border pixels 8x8(WxH) for demosaic.
 */
#if (MT9M114_USER_SELECT_CAMERA_OUTPUT_FORMAT == MT9M114_CAMERA_OUTPUT_FORMAT_RAW_BAYER10)
#define MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_WIDTH  8
#define MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_HEIGHT 8

#define MT9M114_ADDITIONAL_WIDTH                          MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_WIDTH
#define MT9M114_ADDITIONAL_HEIGHT                         MT9M114_RAW_BAYER_FORMAT_ADDITIONAL_BORDER_HEIGHT
#else
#define MT9M114_ADDITIONAL_WIDTH  0
#define MT9M114_ADDITIONAL_HEIGHT 0
#endif

#if (MT9M114_CAMERA_RESOLUTION == MT9M114_CAMERA_RESOLUTION_VGA_640x480)
#define FRAME_WIDTH  (640 + MT9M114_ADDITIONAL_WIDTH)
#define FRAME_HEIGHT (480 + MT9M114_ADDITIONAL_HEIGHT)
#endif

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for Camera frame buffer:
 *  which will be frame width x frame height
 */
#define FRAMEBUFFER_POOL_SIZE ((FRAME_WIDTH) * (FRAME_HEIGHT))

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
uint8_t framebuffer_pool[FRAMEBUFFER_POOL_SIZE] __attribute__((section(".bss.camera_frame_buf")));

/* (optional)
 * if required convert captured image data format to any other image format.
 *
 *  - for MT9M114 Camera sensor,
 *     selected Bayer output format:
 *      in-order to get the color image,
 *       Bayer format must be converted in to RGB format.
 *       User can use below provided
 *        "Open-Source" code for Bayer to RGB Conversion
 *        which uses DC1394 library.
 */
/* Enable image conversion Bayer to RGB. */
#define IMAGE_CONVERSION_BAYER_TO_RGB_EN 0

/* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
/* @Note: Bayer to RGB configurations
 *        are directly borrowed from "Open-Source" code for
 *        Bayer to RGB Conversion, for detail refer bayer2rgb.c.
 *
 * Selected Bayer to RGB configurations:
 *   - converted image format : tiff
 *   - bpp bit per pixel      : 8-bit
 */
#define TIFF_HDR_NUM_ENTRY   8
#define TIFF_HDR_SIZE        10 + TIFF_HDR_NUM_ENTRY * 12

/* bpp bit per pixel
 *  Valid parameters are:
 *   -  8-bit
 *   - 16-bit
 */
#define BITS_PER_PIXEL_8_BIT 8
#define BITS_PER_PIXEL       BITS_PER_PIXEL_8_BIT

/* pool size for Camera frame buffer for Bayer to RGB conversion:
 *   which will be frame width x frame height x (bpp / 8) * 3 + tiff header(106 Bytes).
 */
#define BAYER_TO_RGB_BUFFER_POOL_SIZE                                                              \
    ((FRAME_WIDTH) * (FRAME_HEIGHT) * (BITS_PER_PIXEL / 8) * 3 + TIFF_HDR_SIZE)

/* pool area for Camera frame buffer for Bayer to RGB conversion.
 *  Allocated in the "camera_frame_bayer_to_rgb_buf" section.
 */
uint8_t bayer_to_rgb_buffer_pool[BAYER_TO_RGB_BUFFER_POOL_SIZE]
    __attribute__((section(".bss.camera_frame_bayer_to_rgb_buf")));

/* Optional:
 *  Camera Image Conversions
 */
typedef enum {
    BAYER_TO_RGB_CONVERSION = (1 << 0),
} IMAGE_CONVERSION;

#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

/* Camera callback events */
typedef enum {
    CAM_CB_EVENT_FRAME_VSYNC_DETECTED = (1 << 0),
    CAM_CB_EVENT_CAPTURE_STOPPED      = (1 << 1),
    CAM_CB_EVENT_ERROR                = (1 << 2)
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

    /* Configure GPIO Pin as i2c_sda
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_SDA_GPIO_PORT),
                      BOARD_CAMERA_I2C_SDA_GPIO_PIN,
                      BOARD_CAMERA_I2C_SDA_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Configure GPIO Pin as i2c_scl
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_SCL_GPIO_PORT),
                      BOARD_CAMERA_I2C_SCL_GPIO_PIN,
                      BOARD_CAMERA_I2C_SCL_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return 0;
}

#if INSTANCE_CPI

/**
 * @fn      static int32_t board_camera_pins_config(void)
 * @brief   Configure camera pinmux settings not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_camera_pins_config(void)
{
    int32_t ret;

    /* Configure GPIO Pin as cam_hsync_a */
    ret = pinconf_set(PORT_(BOARD_CAM_HSYNC_GPIO_PORT),
                      BOARD_CAM_HSYNC_GPIO_PIN,
                      BOARD_CAM_HSYNC_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Configure GPIO Pin as cam_vsync_a */
    ret = pinconf_set(PORT_(BOARD_CAM_VSYNC_GPIO_PORT),
                      BOARD_CAM_VSYNC_GPIO_PIN,
                      BOARD_CAM_VSYNC_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Configure GPIO Pin as cam_pclk_a */
    ret = pinconf_set(PORT_(BOARD_CAM_PCLK_GPIO_PORT),
                      BOARD_CAM_PCLK_GPIO_PIN,
                      BOARD_CAM_PCLK_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Data Lines: D0-D7 */
    ret = pinconf_set(PORT_(BOARD_CAM_D0_GPIO_PORT),
                      BOARD_CAM_D0_GPIO_PIN,
                      BOARD_CAM_D0_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAM_D1_GPIO_PORT),
                      BOARD_CAM_D1_GPIO_PIN,
                      BOARD_CAM_D1_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAM_D2_GPIO_PORT),
                      BOARD_CAM_D2_GPIO_PIN,
                      BOARD_CAM_D2_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAM_D3_GPIO_PORT),
                      BOARD_CAM_D3_GPIO_PIN,
                      BOARD_CAM_D3_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAM_D4_GPIO_PORT),
                      BOARD_CAM_D4_GPIO_PIN,
                      BOARD_CAM_D4_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAM_D5_GPIO_PORT),
                      BOARD_CAM_D5_GPIO_PIN,
                      BOARD_CAM_D5_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAM_D6_GPIO_PORT),
                      BOARD_CAM_D6_GPIO_PIN,
                      BOARD_CAM_D6_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_CAM_D7_GPIO_PORT),
                      BOARD_CAM_D7_GPIO_PIN,
                      BOARD_CAM_D7_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return 0;
}

#endif

#if INSTANCE_LPCPI
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

    ret = pinconf_set(PORT_(BOARD_LPCAM_D0_GPIO_PORT),
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
#endif

#if !(USE_CONDUCTOR_TOOL_PINS_CONFIG)
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

/* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
/**
  \fn          int32_t camera_image_conversion(IMAGE_CONVERSION image_conversion,
                                               uint8_t  *src,
                                               uint8_t  *dest,
                                               uint32_t  frame_width,
                                               uint32_t  frame_height)
  \brief       Convert image data from one format to any other image format.
                - Supported conversions
                - Bayer(RAW) to RGB Conversion
                - User can use below provided
                  "Open-Source" Bayer to RGB Conversion code
                  which uses DC1394 library.
                - This code will,
                - Add header for tiff image format
                - Convert RAW Bayer to RGB depending on
                - bpp bit per pixel 8/16 bit
                - DC1394 Color Filter
                - DC1394 Bayer interpolation methods
                - Output image size will be
                - width x height x (bpp / 8) x 3 + tiff header(106 Bytes)
  \param[in]   image_conversion : image conversion methods \ref IMAGE_CONVERSION
  \param[in]   src              : Source address, Pointer to already available
  image data Address
  \param[in]   dest             : Destination address,Pointer to Address,
  where converted image data will be stored.
  \param[in]   frame_width      : image frame width
  \param[in]   frame_height     : image frame height
  \return      success          :  0
  \return      failure          : -1
  */
int32_t camera_image_conversion(IMAGE_CONVERSION image_conversion, uint8_t *src, uint8_t *dest,
                                uint32_t frame_width, uint32_t frame_height)
{
    /* Bayer to RGB Conversion. */
    extern int32_t bayer_to_RGB(uint8_t *src, uint8_t *dest, uint32_t width, uint32_t height);

    int32_t ret = 0;

    switch (image_conversion) {
    case BAYER_TO_RGB_CONVERSION:
        {
            printf("\r\n Start Bayer to RGB Conversion: \r\n");
            printf("\t Frame Buffer Addr: 0x%X \r\n \t Bayer_to_RGB Addr: 0x%X\n",
                   (uint32_t) src,
                   (uint32_t) dest);
            ret = bayer_to_RGB(src, dest, frame_width, frame_height);
            if (ret != 0) {
                printf("\r\n Error: CAMERA image conversion: Bayer to RGB failed.\r\n");
                return -1;
            }
            break;
        }

    default:
        {
            return -1;
        }
    }

    return 0;
}
#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

/**
  \fn          void camera_demo_thread_entry(void *pvParameters)
  \brief       TestApp to verify MT9M114 Camera Sensor with
                FREERTOS as an Operating System.

               This demo thread does:
                 - initialize i2c and Camera hardware pins
                    using PinMux Driver;
                 - initialize Camera driver
                 - capture one frame
                   - captured data will be stored in to allocated
                     frame buffer address
                 - stop Camera capture
                 - (optional)
                   -if required convert captured image format
                    in to any other image format
                   - for MT9M114 Camera sensor,
                        - selected Bayer output format;
                        - in-order to get the color image,
                           Bayer format must be converted in to RGB format.
                        - User can use below provided
                           "Open-Source" code for Bayer to RGB Conversion
                           which uses DC1394 library.
                 - dump captured/converted image data from memory address
                    using any debugger
                 - display image
  @param       pvParameters.
  \return      none
*/
void camera_demo_thread_entry(void *pvParameters)
{
    int32_t  ret           = 0;
    uint32_t actual_events = 0;

    ARM_DRIVER_VERSION version;

    printf("\r\n \t\t >>> MT9M114 Camera Sensor demo with FREERTOS is starting up!!! <<< \r\n");

    /* Allocated memory address for
     *   - Camera frame buffer and
     *   - (Optional) Camera frame buffer for Bayer to RGB Conversion.
     */
    printf("\n \t frame buffer        pool size: 0x%0X  pool addr: 0x%0X \r\n ",
           FRAMEBUFFER_POOL_SIZE,
           (uint32_t) framebuffer_pool);

#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
    printf("\n \t bayer_to_rgb buffer pool size: 0x%0X  pool addr: 0x%0X \r\n ",
           BAYER_TO_RGB_BUFFER_POOL_SIZE,
           (uint32_t) bayer_to_rgb_buffer_pool);
#endif

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h */
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }
#else
    /*
     * NOTE: The I2C and  Camera pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = hardware_init();
    if (ret != 0) {
        printf("Error: CAMERA Hardware Initialize failed: %d\n", ret);
        return;
    }
#endif

    version = CAMERAdrv->GetVersion();
    printf("\r\n Camera driver version api:0x%X driver:0x%X \r\n", version.api, version.drv);

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
     * @Observation for MT9M114 Camera Sensor:
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
        goto error_uninitialize;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Camera Sensor Configuration failed.\r\n");
        goto error_uninitialize;
    }

    /* Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
                                 ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED |
                                 ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                                 ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                                 ARM_CPI_EVENT_ERR_HARDWARE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Camera Events Configuration failed.\r\n");
        goto error_uninitialize;
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
    xTaskNotifyWait(NULL,
                    CAM_CB_EVENT_CAPTURE_STOPPED | CAM_CB_EVENT_ERROR,
                    &actual_events,
                    portMAX_DELAY);

    if (!(actual_events & CAM_CB_EVENT_CAPTURE_STOPPED) && (actual_events & CAM_CB_EVENT_ERROR)) {
        /* Error: Camera Capture Frame failed. */
        printf("\r\n \t\t >> Error: CAMERA Capture Frame failed. \r\n");
        goto error_poweroff;
    }

    /* Okay, we have received Success: Camera Capture Frame stop detected.
     * now stop Camera Capture.
     */
    ret = CAMERAdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA stop Capture failed.\r\n");
        goto error_poweroff;
    }

    /* (optional)
     * if required convert captured image data format to any other image format.
     *  - for MT9M114 Camera sensor,
     *     selected Bayer output format:
     *      in-order to get the color image,
     *       Bayer format must be converted in to RGB format.
     *       User can use below provided
     *        "Open-Source" code for Bayer to RGB Conversion
     *        which uses DC1394 library.
     */
    /* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
    ret = camera_image_conversion(BAYER_TO_RGB_CONVERSION,
                                  framebuffer_pool,
                                  bayer_to_rgb_buffer_pool,
                                  FRAME_WIDTH,
                                  FRAME_HEIGHT);
    if (ret != 0) {
        printf("\r\n Error: CAMERA image conversion failed.\r\n");
        return;
    }
#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

    /* How to dump captured/converted image data from memory address?
     *  To dump memory using ARM DS(Development Studio) and Ulink Pro Debugger
     *
     *  Use below command in "Commands" tab:
     *   dump binary memory path_with_filename.fileformat starting_address ending_address
     *
     *   example:(update user directory name)
     *    dump binary memory /home/user/camera_dump/cam_image0_640p.bin 0x8000000 0x804D33F
     *
     *   Bayer to RGB:
     *    dump binary memory /home/user/camera_dump/cam_image0_Bayer_to_RGB_640p.tif 0x8000000
     * 0x80E7A29
     *
     *   This command will dump memory from staring address to ending address
     *   and store it in to given path with filename.
     */
    printf("\n To dump memory using ARM DS and Ulink Pro Debugger:");
    printf("\n  Use below command in Commands tab: update user directory name \r\n");

#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
    printf("\n   dump binary memory /home/user/camera_dump/cam_image0_Bayer_to_RGB_640p.tif 0x%X "
           "0x%X \r\n",
           (uint32_t) bayer_to_rgb_buffer_pool,
           (uint32_t) (bayer_to_rgb_buffer_pool + BAYER_TO_RGB_BUFFER_POOL_SIZE - 1));
#else
    printf("\n   dump binary memory /home/user/camera_dump/cam_image0_640p.bin 0x%X 0x%X \r\n",
           (uint32_t) framebuffer_pool,
           (uint32_t) (framebuffer_pool + FRAMEBUFFER_POOL_SIZE - 1));
#endif

    printf("\n  This command will dump memory from staring address to ending address \r");
    printf("\n  and store it in to given path with filename.\r\n\r\n");

    printf("\r\n\r\n XXX Camera demo thread is halting here! XXX...\r\n");
    printf("\r\n Now User can dump captured/converted image data from memory address using any "
           "debugger!!!\r\n");

    /* wait forever. */
    WAIT_FOREVER_LOOP

error_poweroff:
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
                                       216,
                                       NULL,
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
