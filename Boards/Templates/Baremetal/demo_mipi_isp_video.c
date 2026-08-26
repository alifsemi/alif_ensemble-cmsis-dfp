/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
#include "RTE_Components.h"
#include CMSIS_device_header
#include <RTE_Device.h>

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "Driver_CDC200.h"
#include "Driver_CPI.h"
#include "Driver_Common.h"
#include "Driver_ISP.h"
#include "aipl_color_conversion.h"
#include "aipl_image.h"
#include "alif_logo.h"
#include "board_config.h"
#include "pinconf.h"
#include "se_services_port.h"
#include "aipm.h"
#include "vsi_comm_video.h"

/* -----------------------------------------------------------------------
 * Camera (ISP)
 * ---------------------------------------------------------------------*/

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

#if !defined(RTE_ISP) || (RTE_ISP == 0) || !RTE_CPI_ISP_PORT || \
    (defined(RTE_CPI_AXI_PORT) && RTE_CPI_AXI_PORT) || \
    (defined(RTE_CPI_STREAMING_ENABLE) && RTE_CPI_STREAMING_ENABLE)
#error "ISP video streaming demo requires RTE_ISP=1 and RTE_CPI_ISP_PORT=1; " \
       "disable RTE_CPI_AXI_PORT and RTE_CPI_STREAMING_ENABLE"
#endif

/* ISP output buffer layout, based on RTE_ISP_OUTPUT_FORMAT */
enum {
    ISP_PLANAR = 1,
    ISP_SEMIPLANAR,
    ISP_INTERLEAVED,
    ISP_NONE,
};

#define ISP_OUTPUT_X (RTE_ISP_OUTPUT_WIDTH)
#define ISP_OUTPUT_Y (RTE_ISP_OUTPUT_HEIGHT)

#if (RTE_ISP_OUTPUT_FORMAT == 32) /* YUYV (YUV422 packed) output from ISP */
#define ISP_OUTPUT_SIZE_Y   (ISP_OUTPUT_X * ISP_OUTPUT_Y * 2)
#define ISP_PITCH           ISP_OUTPUT_X
#define ISP_AUX_BUFFER_TYPE ISP_INTERLEAVED
#define ISP_OUTPUT_TOTAL_SIZE (ISP_OUTPUT_SIZE_Y)
#else
#error "Unsupported RTE_ISP_OUTPUT_FORMAT for this demo (expected YUYV/32)"
#endif

#define OUT_IMAGE_PITCH  ISP_PITCH
#define OUT_IMAGE_WIDTH  ISP_OUTPUT_X
#define OUT_IMAGE_HEIGHT ISP_OUTPUT_Y

static uint8_t y_buffer[RTE_ISP_BUFFER_COUNT][ISP_OUTPUT_SIZE_Y]
    __attribute__((section(".bss.lcd_frame_buf"), aligned(32)));

static VIDEO_BUF_S buffer_array[RTE_ISP_BUFFER_COUNT];

/* Camera Driver instance */
extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

/* ISP Driver instance (for auto-exposure/gain feedback) */
extern ARM_DRIVER_ISP Driver_ISP;

typedef enum {
    CAM_CB_EVENT_NONE            = 0,
    CAM_CB_EVENT_ERROR           = (1 << 0),
    CAM_CB_EVENT_CAPTURE_STOPPED = (1 << 1),
    ISP_MI_FRAME_DUMP_EVENT      = (1 << 2),
} CAM_CB_EVENT;

static volatile CAM_CB_EVENT g_cam_cb_events;

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
        case ISP_INTERLEAVED:
        case ISP_NONE:
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

static void camera_callback(uint32_t event)
{
    switch (event) {
    case ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED:
        g_cam_cb_events |= CAM_CB_EVENT_CAPTURE_STOPPED;
        break;
    case ARM_ISP_MI_EVENT_MP_FRAME_END_DETECTED:
    case ARM_ISP_MI_EVENT_FILL_MP_Y_DETECTED:
    case ARM_ISP_MI_EVENT_MP_Y_WRAP_DETECTED:
        g_cam_cb_events |= ISP_MI_FRAME_DUMP_EVENT;
        break;
    case ARM_ISP_EVENT_FRAME_VSYNC_DETECTED:
    case ARM_ISP_EVENT_FRAME_IN_DETECTED:
    case ARM_ISP_EVENT_AWB_DONE:
    case ARM_CPI_EVENT_CAMERA_FRAME_HSYNC_DETECTED:
    case ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED:
        break;
    case ARM_CPI_EVENT_ERR_HARDWARE:
    case ARM_CPI_EVENT_MIPI_CSI2_ERROR:
    case ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN:
    case ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN:
    default:
        g_cam_cb_events |= CAM_CB_EVENT_ERROR | CAM_CB_EVENT_CAPTURE_STOPPED;
        break;
    }
}

/* -----------------------------------------------------------------------
 * Camera I2C and clock pin-mux helpers (same as the working reference)
 * ---------------------------------------------------------------------*/

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

static int camera_init(void)
{
    int ret = CAMERAdrv->Initialize(camera_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Initialize failed.\r\n");
        return ret;
    }
    printf("\r\n CAMERA Initialized.\r\n");

    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power Up failed.\r\n");
        return ret;
    }

    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CPI Configuration failed.\r\n");
        return ret;
    }

    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        return ret;
    }

    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                              ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
                              ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                              ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                              ARM_CPI_EVENT_ERR_HARDWARE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        return ret;
    }

    for (int i = 0; i < RTE_ISP_BUFFER_COUNT; i++) {
        ret = CAMERAdrv->Control(ISP_CONTROL_QBUF, (uint32_t) &buffer_array[i]);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: ISP buffer configuration failed.\r\n");
            return ret;
        }
    }

    printf("CPI camera Initialization Success\r\n");
    return ret;
}

static int camera_capture(void)
{
    g_cam_cb_events = CAM_CB_EVENT_NONE;

    /* It is safe to use a dummy buffer address because RTE_CPI_AXI_PORT is disabled */
    int ret = CAMERAdrv->CaptureFrame((uint8_t *)0xABCDABCD);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Capture Frame failed.\r\n");
        return ret;
    }

    while (!(g_cam_cb_events & ISP_MI_FRAME_DUMP_EVENT)) {
        __WFI();
    }

    ret = CAMERAdrv->Control(ISP_PROCESS_FRAME_END, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ISP Process Frame End failed.\r\n");
        return ret;
    }

    if (g_cam_cb_events & CAM_CB_EVENT_ERROR) {
        printf("\r\n Error: g_cam_cb_events = 0x%X\r\n", g_cam_cb_events);
        ret = 0xFFFF;
    }
    return ret;
}

/* ---------------------------------------------------------------------------
 * Apply pending auto-exposure settings from the ISP to the sensor.
 * Uses the Driver_ISP.Control API to retrieve cached AE values and stability.
 * Safe to call every loop: it only updates the sensor when values change.
 * Ported from isp_viewfinder_logo's demo_mipi_isp_video_freertos.c.
 * --------------------------------------------------------------------------- */
static void camera_ae_update(void)
{
    static uint32_t prev_intLine;
    static uint32_t prev_again;
    static uint32_t prev_dgain;
    static int prev_stable = -1;

    struct isp_ae_cached_values cached = {0};
    int32_t ae_ret;

    ae_ret = Driver_ISP.Control(ISP_CONTROL_AE_GET_CACHED, (uint32_t)&cached);
    if (ae_ret != ARM_DRIVER_OK) {
        return;
    }

    ae_ret = Driver_ISP.Control(ISP_CONTROL_AE_IS_STABLE, 0);
    if (ae_ret >= 0) {
        int stable = (ae_ret == 1) ? 1 : 0;
        if (stable != prev_stable) {
            printf("  AE: stability changed %s\r\n", stable ? "STABLE" : "NOT STABLE");
            prev_stable = stable;
        }
    }

    if (cached.int_line != prev_intLine) {
        printf("  AE: intLine changed %u -> %u\r\n", prev_intLine, cached.int_line);

        ae_ret = CAMERAdrv->Control(CPI_ISP_CAMERA_SENSOR_EXPOSURE, cached.int_line);
        if (ae_ret != ARM_DRIVER_OK) {
            printf("  ERROR: CPI_ISP_CAMERA_SENSOR_EXPOSURE failed with ret=%d\r\n", (int)ae_ret);
        } else {
            printf("  AE: Setting EXPOSURE intLine=%u\r\n", cached.int_line);
        }

        prev_intLine = cached.int_line;
    }

    if (cached.again != prev_again || cached.dgain != prev_dgain) {
        printf("  AE: gain changed again=%u->%u dgain=%u->%u\r\n",
               prev_again, cached.again, prev_dgain, cached.dgain);
        uint32_t totalGain = (uint32_t)(((uint64_t)cached.again * cached.dgain) / 16U);

        ae_ret = CAMERAdrv->Control(CPI_ISP_CAMERA_SENSOR_GAIN, totalGain);
        if (ae_ret != ARM_DRIVER_OK) {
            printf("  ERROR: CPI_ISP_CAMERA_SENSOR_GAIN failed with ret=%d\r\n", (int)ae_ret);
        } else {
            printf("  AE: Setting GAIN totalGain=0x%08x\r\n", totalGain);
        }

        prev_again = cached.again;
        prev_dgain = cached.dgain;
    }
}

/* -----------------------------------------------------------------------
 * Display (CDC200)
 * ---------------------------------------------------------------------*/

#define MY_DISP_HOR_RES (RTE_PANEL_HACTIVE_TIME)
#define MY_DISP_VER_RES (RTE_PANEL_VACTIVE_LINE)

#if !defined(RTE_CDC200_PIXEL_FORMAT) || (RTE_CDC200_PIXEL_FORMAT != 1)
#error "ISP video streaming demo requires RTE_CDC200_PIXEL_FORMAT=1 (RGB888)"
#endif

#if (ISP_OUTPUT_X > MY_DISP_HOR_RES) || (ISP_OUTPUT_Y > MY_DISP_VER_RES)
#error "ISP output exceeds LCD framebuffer; adjust RTE_ISP_OUTPUT_WIDTH/HEIGHT."
#endif

#pragma pack(1)
/* RGB888 24-bit Format (3-bytes) */
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Pixel;
#pragma pack()

static Pixel lcd_buffer_1[MY_DISP_VER_RES][MY_DISP_HOR_RES]
    __attribute__((section(".bss.lcd_frame_buf1"), aligned(32)));
static Pixel lcd_buffer_2[MY_DISP_VER_RES][MY_DISP_HOR_RES]
    __attribute__((section(".bss.lcd_frame_buf2"), aligned(32)));

enum {
    BUFFER_1 = 0,
    BUFFER_2 = 1,
    NUM_BUFFERS
};

static Pixel *buffers[NUM_BUFFERS] = { (Pixel *)&lcd_buffer_1, (Pixel *)&lcd_buffer_2 };
static uint8_t current_buffer;

extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

static void *disp_active_buffer(void)
{
    return buffers[current_buffer];
}

static void *disp_inactive_buffer(void)
{
    return buffers[(current_buffer + 1) % NUM_BUFFERS];
}

static void disp_next_frame(void)
{
    current_buffer = (current_buffer + 1) % NUM_BUFFERS;

    CDCdrv->Control(CDC200_FRAMEBUF_UPDATE, (uint32_t)buffers[current_buffer]);
}

static void disp_callback(uint32_t event)
{
    if (event & ARM_CDC_DSI_ERROR_EVENT) {
        printf("\r\n Error: CDC DSI error event.\r\n");
        __BKPT(0);
    }
}

/* -----------------------------------------------------------------------
 * Software (CPU-only) rendering: camera frame + Alif logo overlay.
 * Ported from the isp_viewfinder_logo FreeRTOS project's
 * draw_alif_logo_sw(), adapted from RGB888 to our RGB888 buffer layout.
 * ---------------------------------------------------------------------*/
static void draw_alif_logo_sw(uint32_t x_pos, uint32_t y_pos, Pixel *fb)
{
    const aipl_image_t *logo = get_alif_logo();
    const uint8_t      *src  = (const uint8_t *)logo->data;
    const uint8_t      *lut  = get_alif_lut();
    uint8_t            *dst  = (uint8_t *)fb;

    for (uint32_t y = 0; y < (uint32_t)logo->height; y++) {
        uint32_t dst_y = y_pos + y;
        if (dst_y >= MY_DISP_VER_RES) {
            break;
        }

        for (uint32_t x = 0; x < (uint32_t)logo->width; x++) {
            uint32_t dst_x = x_pos + x;
            if (dst_x >= MY_DISP_HOR_RES) {
                break;
            }

            uint8_t idx = src[y * logo->pitch + x];
            const uint8_t *entry = &lut[idx * 4U];

            /* Skip fully transparent CLUT entries */
            if (entry[3] == 0U) {
                continue;
            }

            /* ARGB8888 CLUT is little-endian B,G,R,A in memory */
            uint8_t *pixel = &dst[(dst_y * MY_DISP_HOR_RES + dst_x) * 3U];
            pixel[0] = entry[2];  /* R */
            pixel[1] = entry[1];  /* G */
            pixel[2] = entry[0];  /* B */
        }
    }
}

static int display_init(void)
{
    int ret = CDCdrv->Initialize(disp_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC init failed\r\n");
        return ret;
    }

    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC Power up failed\r\n");
        return ret;
    }

    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)disp_active_buffer());
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC controller configuration failed\r\n");
        return ret;
    }

    ret = CDCdrv->Start();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CDC Start failed\r\n");
        return ret;
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * Demo application
 * ---------------------------------------------------------------------*/

static void demo_mipi_isp_video(void)
{
    int32_t board_init_ret = board_gpios_config();
    if (board_init_ret) {
        printf("\r\n Error: board_gpios_config failed (%" PRId32 ").\r\n",
               board_init_ret);
        __BKPT(0);
    }

    /* ISP buffer array */
    isp_buffer_init();

    /* I2C and camera clock pin-mux */
    if (i2c_pinmux() != 0) {
        printf("\r\n Error: i2c_pinmux failed.\r\n");
        __BKPT(0);
    }

    if (camera_pinmux() != 0) {
        printf("\r\n Error: camera_pinmux failed.\r\n");
        __BKPT(0);
    }

    /* Initialize the SE services */
    se_services_port_init();

    board_init_ret = board_clocks_config(CLKEN_HFOSC_MASK | CLKEN_CLK_100M_MASK);
    if (board_init_ret) {
        printf("\r\n Error: board_clocks_config failed (%" PRId32 ").\r\n",
               board_init_ret);
        __BKPT(0);
    }

    /* Patch the run profile to power the MIPI D-PHYs, enable the
     * camera/CDC200/MIPI CSI/GPU IP clocks, and reserve the SRAM blocks
     * used for the camera/display frame buffers. */
    run_profile_t runp = {0};
    uint32_t service_error_code;
    uint32_t error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nError: SE get_run_cfg failed (%" PRId32 ")\r\n", error_code);
        __BKPT(0);
    }

    runp.memory_blocks = MRAM_MASK | SRAM0_MASK | SRAM1_MASK;
    runp.phy_pwr_gating = MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK | MIPI_RX_DPHY_MASK | LDO_PHY_MASK;
    runp.ip_clock_gating = CAMERA_MASK | MIPI_CSI_MASK | MIPI_DSI_MASK | CDC200_MASK | GPU_MASK;

    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nError: SE set_run_cfg failed (%" PRId32 ")\r\n", error_code);
        __BKPT(0);
    }

    int ret = camera_init();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: camera_init failed (%d).\r\n", ret);
        __BKPT(1);
    }

    printf("\r\n Sensor: %s (%dx%d)\r\n",
           CAMERA_SENSOR_NAME, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT);
    printf("\r\n ISP output: %dx%d  format=%d\r\n",
           ISP_OUTPUT_X, ISP_OUTPUT_Y, RTE_ISP_OUTPUT_FORMAT);

    ret = display_init();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: display_init failed (%d).\r\n", ret);
        __BKPT(2);
    }

    const aipl_image_t *logo = get_alif_logo();

    printf("\r\n Let's Start Capturing Camera Frame...\r\n");
    while (ret == ARM_DRIVER_OK) {
        ret = camera_capture();
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: CAMERA Capture Frame failed.\r\n");
            break;
        }

        /* Top OUT_IMAGE_HEIGHT rows show the live feed; the logo is
         * centered in the remaining section below it. */
        uint32_t logo_section_y = OUT_IMAGE_HEIGHT;
        uint32_t logo_section_height = MY_DISP_VER_RES - OUT_IMAGE_HEIGHT;
        uint32_t logo_x = (MY_DISP_HOR_RES - (uint32_t)logo->width) / 2;
        uint32_t logo_y = logo_section_y +
                           (logo_section_height - (uint32_t)logo->height) / 2;

        /* CPU-only rendering: convert straight into the inactive display
         * buffer and overlay the logo via a manual CLUT lookup. */
        Pixel *fb = (Pixel *)disp_inactive_buffer();

        /* Fill background since we double-buffer: any area not covered
         * by the camera feed or an opaque logo pixel would otherwise
         * show whatever was last written to this buffer (black, on
         * freshly zeroed BSS). */
        memset(fb, 0xFE, (size_t)MY_DISP_HOR_RES * MY_DISP_VER_RES * sizeof(Pixel));

        aipl_error_t aipl_ret = aipl_color_convert_yuy2_to_rgb888(
            y_buffer[0], (uint8_t *)fb, MY_DISP_HOR_RES,
            OUT_IMAGE_WIDTH, OUT_IMAGE_HEIGHT);
        if (aipl_ret != AIPL_ERR_OK) {
            printf("\r\nError: Camera format conversion from yuy2 to rgb888 failed (%s)\r\n",
                   aipl_error_str(aipl_ret));
            __BKPT(0);
        }

        draw_alif_logo_sw(logo_x, logo_y, fb);

        SCB_CleanDCache_by_Addr((uint32_t *)fb,
                                 (int32_t)(MY_DISP_HOR_RES * MY_DISP_VER_RES * sizeof(Pixel)));

        disp_next_frame();

        /* Apply pending auto-exposure/gain changes */
        camera_ae_update();
    }

    printf("\r\n XXX Camera demo is exiting XXX...\r\n");

    while (1) {
        __WFI();
    }
}

int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    if (stdout_init() != ARM_DRIVER_OK) {
        while (1) {
            __WFI();
        }
    }
#elif defined(RTE_CMSIS_Compiler_STDOUT)
    stdout_init();
#endif

    demo_mipi_isp_video();

    return 0;
}

