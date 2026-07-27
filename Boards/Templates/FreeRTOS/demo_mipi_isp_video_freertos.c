/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     demo_mipi_isp_video_freertos.c
 * @author   Shivakumar Malke
 * @email    shivakumar.malke@alifsemi.com
 * @version  V1.0.0
 * @date     2026-07-22
 * @brief    ISP video streaming demo for ILI9806E LCD panel using FreeRTOS.
 *           Supports OV5675 / ARX3A0 / MT9M114 camera sensors.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "event_groups.h"
#include "app_utils.h"

/* Camera Controller Driver */
#include "Driver_CPI.h"

/* ISP Driver */
#include "Driver_ISP.h"
#include "vsi_comm_video.h"

/* CDC200 Display Driver */
#include "Driver_CDC200.h"

/* RTE configuration */
#include "RTE_Device.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif

/* Board / Pin support */
#include "pinconf.h"
#include "board_config.h"

/* SE Services */
#include "se_services_port.h"

/* AIPL
 *
 * This application requires the AIPL pack. In the RTE configuration
 * (RTE -> Graphics), enable all AIPL-related components before building.
 */
#include "aipl_error.h"
#include "aipl_color_conversion.h"
#include "alif_logo.h"

/* Device header */
#include CMSIS_device_header

/* RTE build-time sanity checks */
#if !defined(RTE_ISP) || (RTE_ISP == 0) || !RTE_CPI_ISP_PORT || \
    (defined(RTE_CPI_AXI_PORT) && RTE_CPI_AXI_PORT) || \
    (defined(RTE_CPI_STREAMING_ENABLE) && RTE_CPI_STREAMING_ENABLE)
#error "ISP video streaming demo requires RTE_ISP=1 and RTE_CPI_ISP_PORT=1; " \
       "disable RTE_CPI_AXI_PORT and RTE_CPI_STREAMING_ENABLE"
#endif

/* Driver instances */
extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

extern ARM_DRIVER_ISP Driver_ISP;

/* Camera sensor resolution — determined by the active RTE component */
#if defined(RTE_Drivers_CAMERA_SENSOR_OV5675)
#define CAM_FRAME_WIDTH   (RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT  (RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT)
#define CAMERA_SENSOR_NAME "OV5675"
#elif defined(RTE_Drivers_CAMERA_SENSOR_ARX3A0)
#define CAM_FRAME_WIDTH   (RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT  (RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT)
#define CAMERA_SENSOR_NAME "ARX3A0"
#elif defined(RTE_Drivers_CAMERA_SENSOR_MT9M114)
#define CAM_FRAME_WIDTH   (RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT  (RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT)
#define CAMERA_SENSOR_NAME "MT9M114"
#else
#error "No camera sensor selected in RTE! Enable OV5675, ARX3A0, or MT9M114."
#endif

/* ILI9806E LCD Panel: 480x800 RGB888 (3 bytes/pixel) */
#if !defined(RTE_CDC200_PIXEL_FORMAT) || (RTE_CDC200_PIXEL_FORMAT != 1)
#error "ISP video streaming demo requires RTE_CDC200_PIXEL_FORMAT=1 (RGB888)"
#endif
#define LCD_FRAME_WIDTH        (480)
#define LCD_FRAME_HEIGHT       (800)
#define RGB_BYTES_PER_PIXEL    (3)

#define LCD_FRAMEBUFFER_POOL_SIZE  ((LCD_FRAME_WIDTH) * (LCD_FRAME_HEIGHT) * RGB_BYTES_PER_PIXEL)

uint8_t lcd_framebuffer_pool[LCD_FRAMEBUFFER_POOL_SIZE]
        __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));

/*
 * ISP output buffers
 * The ISP scaler crops/scales the sensor input to ISP_OUTPUT_X x ISP_OUTPUT_Y.
 * YUYV (format 32) is used so a single Y plane holds the interleaved data.
 */
enum {
    ISP_PLANAR = 1,
    ISP_SEMIPLANAR,
    ISP_INTERLEAVED,
    ISP_NONE,
};

/* ISP output dimensions — must match RTE_ISP_OUTPUT_WIDTH / HEIGHT in RTE_Device.h */
#define ISP_OUTPUT_X    (RTE_ISP_OUTPUT_WIDTH)
#define ISP_OUTPUT_Y    (RTE_ISP_OUTPUT_HEIGHT)

#if (ISP_OUTPUT_X > LCD_FRAME_WIDTH) || (ISP_OUTPUT_Y > LCD_FRAME_HEIGHT)
#error "ISP output exceeds LCD framebuffer; adjust RTE_ISP_OUTPUT_WIDTH/HEIGHT."
#endif

#if (RTE_ISP_OUTPUT_FORMAT == 32) /* YUYV (YUV422 interleaved) */
#define ISP_OUTPUT_SIZE_Y      (ISP_OUTPUT_X * ISP_OUTPUT_Y * 2)
#define ISP_PITCH              ISP_OUTPUT_X
#define ISP_AUX_BUFFER_TYPE    ISP_INTERLEAVED
#define ISP_OUTPUT_TOTAL_SIZE  (ISP_OUTPUT_SIZE_Y)
#else
#error "RTE_ISP_OUTPUT_FORMAT must be 32 (YUYV) for aipl_color_convert_yuy2_to_rgb888()."
#endif

#if defined(ISP_OUTPUT_SIZE_Y)
uint8_t y_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_Y]
        __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif
#if defined(ISP_OUTPUT_SIZE_CB)
uint8_t cb_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_CB]
        __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif
#if defined(ISP_OUTPUT_SIZE_CR)
uint8_t cr_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_CR]
        __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif
#if defined(ISP_OUTPUT_SIZE_CBCR)
uint8_t cbcr_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_CBCR]
        __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));
#endif

VIDEO_BUF_S buffer_array[RTE_ISP_BUFFER_COUNT];

/* FreeRTOS task and event group */
#define STACK_SIZE  (4096U / sizeof(StackType_t))

static TaskHandle_t     video_task_handle;
static EventGroupHandle_t event_group;

/* FreeRTOS hook implementations */
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &IdleTcb;
    *ppxIdleTaskStackBuffer = IdleStack;
    *pulIdleTaskStackSize   = IDLE_TASK_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t  **ppxTimerTaskStackBuffer,
                                    uint32_t      *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &TimerTcb;
    *ppxTimerTaskStackBuffer = TimerStack;
    *pulTimerTaskStackSize   = TIMER_SERVICE_TASK_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    ARG_UNUSED(pxTask);
    ARG_UNUSED(pcTaskName);
    ASSERT_HANG_LOOP
}

void vApplicationIdleHook(void)
{
    /* Intentionally empty — __WFI() not used here to allow RTOS tick to fire */
}

/* Callback event bits */
typedef enum {
    CAM_CB_EVENT_ERROR       = (1 << 0),
    DISP_CB_EVENT_ERROR      = (1 << 1),
    CAM_VSYNC_CB_EVENT       = (1 << 2),
    ISP_VSYNC_CB_EVENT       = (1 << 3),
    ISP_MI_FRAME_DUMP_EVENT  = (1 << 4),
    ISP_CB_EVENT_ERROR       = (1 << 5),
} CB_EVENTS;

#define ALL_WAIT_BITS  (CAM_CB_EVENT_ERROR      | \
                        DISP_CB_EVENT_ERROR     | \
                        CAM_VSYNC_CB_EVENT      | \
                        ISP_VSYNC_CB_EVENT      | \
                        ISP_MI_FRAME_DUMP_EVENT | \
                        ISP_CB_EVENT_ERROR)

/* Soft-reset counter (issued every other VSYNC to keep sensor stable) */
static uint32_t softreset_interval_counter;

/* Camera / ISP / Display callbacks */
void Camera_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (event & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED) {
        xEventGroupSetBitsFromISR(event_group, CAM_VSYNC_CB_EVENT, &xHigherPriorityTaskWoken);
        if (softreset_interval_counter % 2) {
            CAMERAdrv->Control(CPI_SOFTRESET, 0);
        }
        softreset_interval_counter++;
    }

#if (!RTE_ISP)
    if (event & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN) {
        xEventGroupSetBitsFromISR(event_group, CAM_CB_EVENT_ERROR, &xHigherPriorityTaskWoken);
    }
    if (event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN) {
        xEventGroupSetBitsFromISR(event_group, CAM_CB_EVENT_ERROR, &xHigherPriorityTaskWoken);
    }
#endif

    if (event & ARM_CPI_EVENT_MIPI_CSI2_ERROR) {
        xEventGroupSetBitsFromISR(event_group, CAM_CB_EVENT_ERROR, &xHigherPriorityTaskWoken);
    }

    if (event & ARM_ISP_EVENT_FRAME_VSYNC_DETECTED) {
        xEventGroupSetBitsFromISR(event_group, ISP_VSYNC_CB_EVENT, &xHigherPriorityTaskWoken);
    }

    if (event & ARM_ISP_MI_EVENT_MP_FRAME_END_DETECTED) {
        xEventGroupSetBitsFromISR(event_group, ISP_MI_FRAME_DUMP_EVENT, &xHigherPriorityTaskWoken);
    }

    if (event & ARM_ISP_EVENT_DATALOSS_DETECTED) {
        xEventGroupSetBitsFromISR(event_group, ISP_CB_EVENT_ERROR, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Display_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (event & ARM_CDC_DSI_ERROR_EVENT) {
        xEventGroupSetBitsFromISR(event_group, DISP_CB_EVENT_ERROR, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * Apply pending auto-exposure settings from the ISP to the sensor.
 * Uses the Driver_ISP.Control API to retrieve cached AE values and stability.
 * Safe to call every loop: it only updates the sensor when values change.
 */
static void camera_ae_update(void)
{
    static uint32_t prev_intLine;
    static uint32_t prev_again;
    static uint32_t prev_dgain;

    struct isp_ae_cached_values cached = {0};
    int32_t ret;
    static int prev_stable = -1;

    ret = Driver_ISP.Control(ISP_CONTROL_AE_GET_CACHED, (uint32_t)&cached);
    if (ret != ARM_DRIVER_OK) {
        return;
    }

    ret = Driver_ISP.Control(ISP_CONTROL_AE_IS_STABLE, 0);
    if (ret >= 0) {
        int stable = (ret == 1) ? 1 : 0;

        if (stable != prev_stable) {
            printf("  AE: stability changed %s\r\n", stable ? "STABLE" : "NOT STABLE");
            prev_stable = stable;
        }
    }

    if (cached.int_line != prev_intLine) {
        printf("  AE: intLine changed %u -> %u\r\n", prev_intLine, cached.int_line);

        ret = CAMERAdrv->Control(CPI_ISP_CAMERA_SENSOR_EXPOSURE, cached.int_line);
        if (ret != ARM_DRIVER_OK) {
            printf("  ERROR: CPI_ISP_CAMERA_SENSOR_EXPOSURE failed with ret=%d\r\n", (int)ret);
        } else {
            printf("  AE: Setting EXPOSURE intLine=%u\r\n", cached.int_line);
        }

        prev_intLine = cached.int_line;
    }

    if (cached.again != prev_again || cached.dgain != prev_dgain) {
        printf("  AE: gain changed again=%u->%u dgain=%u->%u\r\n",
               prev_again, cached.again, prev_dgain, cached.dgain);
        uint32_t totalGain = (uint32_t)(((uint64_t)cached.again * cached.dgain) / 16U);

        ret = CAMERAdrv->Control(CPI_ISP_CAMERA_SENSOR_GAIN, totalGain);
        if (ret != ARM_DRIVER_OK) {
            printf("  ERROR: CPI_ISP_CAMERA_SENSOR_GAIN failed with ret=%d\r\n", (int)ret);
        } else {
            printf("  AE: Setting GAIN totalGain=0x%08x\r\n", totalGain);
        }

        prev_again = cached.again;
        prev_dgain = cached.dgain;
    }
}

/* Alif logo overlay (software CLUT lookup for RGB888) */
#define ALIF_LOGO_X        100U
#define ALIF_LOGO_Y        600U

static void draw_alif_logo_sw(uint32_t x_pos, uint32_t y_pos)
{
    const aipl_image_t *logo = get_alif_logo();
    const uint8_t      *src  = (const uint8_t *)logo->data;
    const uint8_t      *lut  = get_alif_lut();
    uint8_t            *dst  = lcd_framebuffer_pool;

    for (uint32_t y = 0; y < logo->height; y++) {
        uint32_t dst_y = y_pos + y;

        if (dst_y >= LCD_FRAME_HEIGHT) {
            break;
        }

        for (uint32_t x = 0; x < logo->width; x++) {
            uint32_t dst_x = x_pos + x;

            if (dst_x >= LCD_FRAME_WIDTH) {
                break;
            }

            uint8_t idx = src[y * logo->pitch + x];
            const uint8_t *entry = &lut[idx * 4U];

            /* Skip fully transparent CLUT entries */
            if (entry[3] == 0U) {
                continue;
            }

            /* ARGB8888 CLUT is little-endian B,G,R,A in memory */
            uint8_t *pixel = &dst[(dst_y * LCD_FRAME_WIDTH + dst_x) * 3U];

            pixel[0] = entry[2];   /* R */
            pixel[1] = entry[1];   /* G */
            pixel[2] = entry[0];   /* B */
        }
    }
}


static void draw_alif_logo(uint32_t x_pos, uint32_t y_pos)
{
    draw_alif_logo_sw(x_pos, y_pos);
}

/* Pin-mux helpers */
static int i2c_pinmux(void)
{
    int ret;

    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_SDA_GPIO_PORT),
                      BOARD_CAMERA_I2C_SDA_GPIO_PIN,
                      BOARD_CAMERA_I2C_SDA_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: i2c PINMUX and PINPAD failed.\r\n");
        return -1;
    }

    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_SCL_GPIO_PORT),
                      BOARD_CAMERA_I2C_SCL_GPIO_PIN,
                      BOARD_CAMERA_I2C_SCL_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: i2c PINMUX and PINPAD failed.\r\n");
        return -1;
    }

    return 0;
}

static int camera_pinmux(void)
{
    int ret;

    ret = pinconf_set(PORT_(BOARD_CAM_XVCLK_A_GPIO_PORT),
                      BOARD_CAM_XVCLK_A_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_6,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Camera Pin-Mux failed.\r\n");
        return -1;
    }

    return 0;
}

static int hardware_init(void)
{
    int ret;

    ret = i2c_pinmux();
    if (ret != 0) {
        printf("\r\n Error in i2c pinmux.\r\n");
        return -1;
    }

    ret = camera_pinmux();
    if (ret != 0) {
        printf("\r\n Error in Camera pinmux.\r\n");
        return -1;
    }

    MPU_Setup();

    return 0;
}

/* ISP buffer initialisation */
static void isp_buffer_init(void)
{
    for (int i = 0; i < RTE_ISP_BUFFER_COUNT; i++) {
        buffer_array[i].index     = i;
        buffer_array[i].imageSize = ISP_OUTPUT_TOTAL_SIZE;

        switch (ISP_AUX_BUFFER_TYPE) {
        case ISP_PLANAR:
            buffer_array[i].numPlanes = 3;
            break;
        case ISP_SEMIPLANAR:
            buffer_array[i].numPlanes = 2;
            break;
        case ISP_NONE:
        case ISP_INTERLEAVED:
        default:
            buffer_array[i].numPlanes = 1;
            break;
        }

        buffer_array[i].planes[0].dmaPhyAddr = (vsi_dma_t)y_buffer[i];
#if defined(ISP_OUTPUT_SIZE_CB)
        buffer_array[i].planes[1].dmaPhyAddr = (vsi_dma_t)cb_buffer[i];
#endif
#if defined(ISP_OUTPUT_SIZE_CR)
        buffer_array[i].planes[2].dmaPhyAddr = (vsi_dma_t)cr_buffer[i];
#endif
#if defined(ISP_OUTPUT_SIZE_CBCR)
        buffer_array[i].planes[1].dmaPhyAddr = (vsi_dma_t)cbcr_buffer[i];
#endif
    }
}

/* Main video task */
void video_task_entry(void *pvParameters)
{
    ARG_UNUSED(pvParameters);

    int32_t  ret = 0;
    uint32_t service_error_code;
    uint32_t error_code;
    run_profile_t runp = {0};

    printf("\r\n >>> ISP Video Streaming demo with FreeRTOS is starting up! <<< \r\n");
    printf("\r\n Sensor: %s (%dx%d)\r\n", CAMERA_SENSOR_NAME, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT);
    printf("\r\n ISP output: %dx%d  format=%d\r\n",
           ISP_OUTPUT_X, ISP_OUTPUT_Y, RTE_ISP_OUTPUT_FORMAT);

    /* Board GPIOs (camera power/reset) */
    ret = board_gpios_config();
    if (ret != 0) {
        printf("\r\n Error: board_gpios_config failed.\r\n");
        return;
    }

    /* ISP buffer array */
    isp_buffer_init();

    /* I2C and camera pin-mux */
    ret = hardware_init();
    if (ret != 0) {
        printf("\r\n Error: hardware_init failed.\r\n");
        return;
    }

    /* SE services + MIPI clocks */
    se_services_port_init();

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              true,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 100MHz clock enable = %" PRId32 "\n", error_code);
        return;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSC,
                                              true,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4MHz(HFOSC) clock enable = %" PRId32 "\n", error_code);
        goto error_disable_100mhz_clk;
    }

    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nSE: get_run_cfg error = %" PRId32 "\n", error_code);
        goto error_disable_hfosc_clk;
    }

    runp.memory_blocks  = MRAM_MASK | SRAM0_MASK | SRAM1_MASK;
    runp.phy_pwr_gating = MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK | MIPI_RX_DPHY_MASK | LDO_PHY_MASK;

    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nSE: set_run_cfg error = %" PRId32 "\n", error_code);
        goto error_disable_hfosc_clk;
    }

    /* CDC200 Initialize */
    ret = CDCdrv->Initialize(Display_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC200 Initialize failed.\r\n");
        goto error_disable_hfosc_clk;
    }

    /* Camera Initialize */
    ret = CAMERAdrv->Initialize(Camera_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Initialize failed.\r\n");
        goto error_uninitialize_cdc;
    }

    /* Power up CDC200 */
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC200 PowerControl failed.\r\n");
        goto error_uninitialize_camera;
    }

    /* Power up Camera */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA PowerControl failed.\r\n");
        goto error_poweroff_cdc;
    }

    /* Configure display with LCD framebuffer */
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)lcd_framebuffer_pool);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC200 Configure Display failed.\r\n");
        goto error_poweroff_camera;
    }
    /*
     * Fill the LCD framebuffer with white. The display is not started yet,
     * so the panel stays blank until the first video frame and logo are ready.
     */
    memset(lcd_framebuffer_pool, 0xFF, LCD_FRAMEBUFFER_POOL_SIZE);
    SCB_CleanDCache_by_Addr((uint32_t *)lcd_framebuffer_pool, LCD_FRAMEBUFFER_POOL_SIZE);

    /* Configure CPI controller */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Configure camera sensor (loads init register table) */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Configure failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Enable camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED |
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN  |
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Event Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Queue ISP output buffers */
    for (int i = 0; i < RTE_ISP_BUFFER_COUNT; i++) {
        ret = CAMERAdrv->Control(ISP_CONTROL_QBUF, (uint32_t)&buffer_array[i]);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: ISP QBUF[%d] failed.\r\n", i);
            goto error_poweroff_camera;
        }
    }

    /* Display is left off until the first frame is rendered below. */

    printf("\r\n Let's Start Capturing Camera Frame...\r\n");
    printf("\r\n Entering main event loop\r\n");

    /*
     * Start continuous video capture.
     * The argument is ignored in ISP mode because the CPI AXI port is disabled.
     * ISP buffers were already queued via ISP_CONTROL_QBUF.
     */
    ret = CAMERAdrv->CaptureVideo((void *)0xDEADC0DE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA CaptureVideo failed.\r\n");
        goto error_poweroff_camera;
    }

    /* Main event loop */
    for (;;) {
        EventBits_t actual_events = xEventGroupWaitBits(event_group,
                                                        ALL_WAIT_BITS,
                                                        pdTRUE,
                                                        pdFALSE,
                                                        pdMS_TO_TICKS(1000));

        /* Error events */
        if (actual_events & (CAM_CB_EVENT_ERROR | DISP_CB_EVENT_ERROR | ISP_CB_EVENT_ERROR)) {
            printf("\r\n \t\t >> Error: Camera or Display failed. events=0x%08" PRIx32 "\r\n",
                   (uint32_t)actual_events);
            goto error_poweroff_camera;
        }

        /* Process frame end and convert ISP output to display format */
        if (actual_events & ISP_MI_FRAME_DUMP_EVENT) {
            CAMERAdrv->Control(ISP_PROCESS_FRAME_END, 0);
            /* ISP wrote via DMA, CPU needs fresh YUYV data before AIPL reads it */
            SCB_InvalidateDCache_by_Addr((uint32_t *)y_buffer[0], ISP_OUTPUT_SIZE_Y);

            /* Convert ISP YUYV output → RGB888 → LCD framebuffer */
            aipl_color_convert_yuy2_to_rgb888(y_buffer[0],
                                              lcd_framebuffer_pool,
                                              ISP_PITCH,
                                              ISP_OUTPUT_X,
                                              ISP_OUTPUT_Y);

            /* Overlay the Alif logo using the appropriate renderer */
            draw_alif_logo(ALIF_LOGO_X, ALIF_LOGO_Y);

            /* CPU/GPU wrote the framebuffer, display DMA needs fresh data */
            SCB_CleanDCache_by_Addr((uint32_t *)lcd_framebuffer_pool, LCD_FRAMEBUFFER_POOL_SIZE);

            /* Apply pending auto-exposure/gain changes */
            camera_ae_update();

            /*
             * Turn on the LCD after the first few frames are rendered.
             * This avoids an initial white flash while the display is blank.
             */
            static int display_started;
            static int display_start_frame_count;

            display_start_frame_count++;
            if (!display_started && display_start_frame_count >= 3) {
                ret = CDCdrv->Start();
                if (ret != ARM_DRIVER_OK) {
                    printf("\r\n Error: CDC200 Start failed.\r\n");
                    goto error_poweroff_camera;
                }
                display_started = 1;
            }
        }
    }

error_poweroff_camera:
    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA PowerControl(OFF) failed.\r\n");
    }

error_poweroff_cdc:
    ret = CDCdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC200 PowerControl(OFF) failed.\r\n");
    }

error_uninitialize_camera:
    ret = CAMERAdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");
    }

error_uninitialize_cdc:
    ret = CDCdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC200 Uninitialize failed.\r\n");
    }

error_disable_hfosc_clk:
    SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, false, &service_error_code);

error_disable_100mhz_clk:
    SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M, false, &service_error_code);

    printf("\r\n XXX Video task is exiting XXX\r\n");
    WAIT_FOREVER_LOOP
}

/* main() */
int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);

    if (stdout_init() != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#elif defined(RTE_CMSIS_Compiler_STDOUT)
    stdout_init();
#endif

    SystemCoreClockUpdate();

    /* Create event group */
    event_group = xEventGroupCreate();
    if (event_group == NULL) {
        /* Failed to allocate event group — halt */
        WAIT_FOREVER_LOOP
    }

    /* Create main task */
    BaseType_t xReturned = xTaskCreate(video_task_entry,
                                       "video_task",
                                       STACK_SIZE,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &video_task_handle);
    if (xReturned != pdPASS) {
        WAIT_FOREVER_LOOP
    }

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    return 0;
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
