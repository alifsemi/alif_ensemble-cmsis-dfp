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
 * @file     : demo_mipi_camera_capture.c
 * @author   : Rohan P
 * @email    : rohan.p@alifsemi.com
 * @date     : 12-Aug-2026
 * @brief    : Baremetal application code for image capture from camera sensors.
 * @Note     : None.
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

/* Cpi Driver */
#include "Driver_CPI.h"
#include "RTE_Components.h"
#include "alif.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "board_config.h"

/* PINMUX Driver */
#include "pinconf.h"

/* SE Services */
#include "se_services_port.h"

// Set to 0: Use application-defined arx3A0 pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

/* Camera Sensor Selection
 * Supports: ARX3A0, MT9M114, OV5675
 *
 * Driver_CPI captures from camera sensor instance 0 (see CPI_Fetch_Sensors()
 * in Driver_CPI.c), so the frame buffer must match that sensor. A sensor is
 * selected when all of the following hold in RTE_Device.h / RTE_Components.h:
 *   - its CAMERA_SENSOR component is enabled (RTE_Drivers_CAMERA_SENSOR_*)
 *   - the sensor itself is enabled
 *   - RTE_<SENSOR>_CAMERA_SENSOR_INSTANCE is 0
 * With two sensors registered (dual camera), only the instance 0 sensor
 * decides the frame buffer size.
 *
 *   - ARX3A0  (560x560,  IPI-16 RAW8)
 *   - MT9M114 (1280x720, IPI-16 RAW8)
 *   - OV5675  (1296x972, IPI-16 RAW8)
 *
 * Note: Sensors output RAW10, but CPI interface default is IPI-16 RAW8.
 *       CPI color mode can be changed via RTE configuration if needed.
 */
#if defined(RTE_Drivers_CAMERA_SENSOR_ARX3A0) && \
    (RTE_ARX3A0_CAMERA_SENSOR_CSI_ENABLE) && \
    (RTE_ARX3A0_CAMERA_SENSOR_INSTANCE == 0)
  #define SELECTED_CAMERA_SENSOR     "ARX3A0"
  #define FRAME_WIDTH                RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH
  #define FRAME_HEIGHT               RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT
  #define BYTES_PER_PIXEL 1

#elif defined(RTE_Drivers_CAMERA_SENSOR_MT9M114) && \
      (RTE_MT9M114_CAMERA_SENSOR_MIPI_ENABLE) && \
      (RTE_MT9M114_CAMERA_SENSOR_INSTANCE == 0)
  #define SELECTED_CAMERA_SENSOR     "MT9M114"
  #define FRAME_WIDTH                 RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH
  #define FRAME_HEIGHT                RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT

  /* MT9M114 image configuration 0 and 1 uses IPI-16 RAW8 format.
   * RAW8 requires 1 byte per pixel for the framebuffer.
   */
  #if (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 0 ||  \
       RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 1)
     #define BYTES_PER_PIXEL 1
  #elif (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG >= 2 &&  \
          RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG <= 5)
     #define BYTES_PER_PIXEL 2
  #else
     #error "Unsupported RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG"
  #endif

#elif defined(RTE_Drivers_CAMERA_SENSOR_OV5675) && \
      (RTE_OV5675_CAMERA_SENSOR_ENABLE) && \
      (RTE_OV5675_CAMERA_SENSOR_INSTANCE == 0)
  #define SELECTED_CAMERA_SENSOR     "OV5675"
  #define FRAME_WIDTH                RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH
  #define FRAME_HEIGHT               RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT
  #define BYTES_PER_PIXEL 1

#else
  #error "No enabled camera sensor with RTE instance 0 (ARX3A0, MT9M114, OV5675)"
#endif

/* Camera Sensor configurations
 * Resolution and IPI format based on selected sensor:
 *   - ARX3A0  : 560x560,  IPI-16 RAW8
 *   - MT9M114 : 1280x720, IPI-16 RAW8
 *   - OV5675  : 1296x972, IPI-16 RAW8
 */

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for Camera frame buffer:
 *  which will be frame width x frame height x bytes per pixel
 */
#define FRAME_BUFFER_POOL_SIZE ((FRAME_WIDTH) * (FRAME_HEIGHT) * (BYTES_PER_PIXEL))

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
uint8_t framebuffer_pool[FRAME_BUFFER_POOL_SIZE]
    __attribute__((section(".bss.camera_frame_buf"), aligned(32)));

/* Camera callback events */
typedef enum {
    CAM_CB_EVENT_CAPTURE_STOPPED = (1 << 0),
    CAM_CB_EVENT_ERROR           = (1 << 1)
} CAMERA_CB_EVENTS;

volatile uint32_t event_flag;

/**
 * \fn          void camera_callback(uint32_t event)
 * \brief       Camera isr callback
 * \param[in]   event: Camera Event
 * \return      none
 */
void camera_callback(uint32_t event)
{
    /* Capture finished normally. */
    if (event & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED) {
        event_flag |= CAM_CB_EVENT_CAPTURE_STOPPED;
    }

    /* FIFO overrun or hardware error occurred. */
    if (event & (ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                 ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                 ARM_CPI_EVENT_ERR_HARDWARE)) {
        event_flag |= CAM_CB_EVENT_ERROR;
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
        printf("\r\n Error: i2c PINMUX and PINPAD failed.\r\n");
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
#if defined(BOARD_CAMERA_I2C_MUX_GPIO_PORT) && \
    defined(BOARD_CAMERA_I2C_MUX_GPIO_PIN) && \
    defined(BOARD_CAMERA_I2C_MUX_ALTERNATE_FUNCTION)
    ret = pinconf_set(PORT_(BOARD_CAMERA_I2C_MUX_GPIO_PORT),
                      BOARD_CAMERA_I2C_MUX_GPIO_PIN,
                      BOARD_CAMERA_I2C_MUX_ALTERNATE_FUNCTION,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Camera I2C mux Pin-Mux failed.\r\n");
        return ret;
    }
#endif
    return 0;
}

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
        printf("\r\n Error in i2c pinmux.\r\n");
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
 * \fn          void camera_demo(void)
 * \brief       Baremetal camera capture.
 *              This demo does:
 *                   - initialize the camera I2C and clock pins
 *                   - enable the MIPI clocks
 *                   - initialize the camera driver
 *                   - capture one frame into framebuffer_pool
 *                   - stop the capture
 *                   - print the debugger commands that dump the frame
 * \return      none
 */
void camera_demo(void)
{
    int32_t ret = 0;
    uint32_t service_error_code;
    uint32_t error_code;
    ARM_DRIVER_VERSION version;
    run_profile_t      runp = {0};

    printf("\r\n \t\t >>> %s Camera Sensor demo baremetal is starting up!!! <<< \r\n",
           SELECTED_CAMERA_SENSOR);
    printf("\r\n \t\t Resolution: %dx%d \r\n", FRAME_WIDTH, FRAME_HEIGHT);

    /* Allocated memory address for
     *   - Camera frame buffer
     */

    printf("\n \t frame buffer        pool size: 0x%0X  pool addr: 0x%08" PRIx32 " \r\n ",
           FRAME_BUFFER_POOL_SIZE,
           (uint32_t) framebuffer_pool);

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h */
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#else

    ret = hardware_init();
    if (ret != 0) {
        printf("Error: CAMERA Hardware Initialize failed: %" PRId32 "\n", ret);
        return;
    }
#endif

    /* Initialize SE services */
    se_services_port_init();

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              true, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE : MIPI 100MHz clock enable = %" PRId32 "\n", error_code);
        return;
    }
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC,
                                              true, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC) clock enable = %" PRId32 "\n", error_code);
        goto error_disable_100mhz_clk;
    }

    /*Get current run configuration from SE */

    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nSE: get_run_cfg error = %" PRId32 "\n", error_code);
        goto error_disable_hfosc_clk;
    }

    runp.memory_blocks = MRAM_MASK | SRAM0_MASK | SRAM1_MASK;
    runp.phy_pwr_gating = MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK |
                  MIPI_RX_DPHY_MASK | LDO_PHY_MASK;

    /*Set new configuration */

    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("\r\nSE: set_run_cfg error = %" PRId32 "\n", error_code);
        goto error_disable_hfosc_clk;
    }

    version = CAMERAdrv->GetVersion();
    printf("\r\n Camera driver version api:0x%" PRIx16 " driver:0x%" PRIx16 " \r\n",
            version.api, version.drv);

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
        printf("\r\n Error: CPI configuration failed.\r\n");
        goto error_poweroff_camera;

    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
       printf("\r\n Error: CAMERA SENSOR configuration failed.\r\n");
       goto error_poweroff_camera;

    }

    /*Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE, ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
            ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
            ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
            ARM_CPI_EVENT_ERR_HARDWARE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA SENSOR Event configuration failed.\r\n");
        goto error_poweroff_camera;

    }

    event_flag = 0;

    printf("\r\n Let's Start Capturing Camera Frame...\r\n");
    ret = CAMERAdrv->CaptureFrame(framebuffer_pool);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error : Camera Capture Frame failed.\r\n");
        goto error_poweroff_camera;

    }
    /* Wait for capture to finish or error to occur */
    while (!(event_flag & (CAM_CB_EVENT_CAPTURE_STOPPED | CAM_CB_EVENT_ERROR))) {
    }


    if (event_flag & CAM_CB_EVENT_ERROR) {
        printf("\r\n\t\t>>Error: CAMERA Capture Frame Failed.\r\n");
        goto error_poweroff_camera;
    }

    if (event_flag & CAM_CB_EVENT_CAPTURE_STOPPED) {
        printf("\r\n Capture complete\r\n");
    }

    ret = CAMERAdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA stop Capture failed.\r\n");
        goto error_poweroff_camera;
    }
    printf("\n To dump memory using ARM DS with Ulink Pro Debugger or Trace32 or Jlink Debugger:");
    printf("\n Use below commands in Commands tab: update user directory name \r\n");

    printf("\r\n Ulink:\r\n");
    printf(" dump binary memory /home/$USER/camera_dump/%s_%dx%d.bin 0x%08" PRIX32 " 0x%08" PRIX32 "\n",
            SELECTED_CAMERA_SENSOR, FRAME_WIDTH, FRAME_HEIGHT,
            (uint32_t) framebuffer_pool,
            (uint32_t) (framebuffer_pool + FRAME_BUFFER_POOL_SIZE - 1));
    printf(" T32:\n");
    printf("\r data.save.binary /home/$USER/camera_dump/%s_%dx%d.bin 0x%08" PRIX32 "--0x%08" PRIX32 "\n",
            SELECTED_CAMERA_SENSOR, FRAME_WIDTH, FRAME_HEIGHT,
            (uint32_t) framebuffer_pool,
            (uint32_t) (framebuffer_pool + FRAME_BUFFER_POOL_SIZE - 1));
    printf(" Jlink:\n");
    printf("\r savebin /home/$USER/camera_dump/%s_%dx%d.bin 0x%08" PRIX32 " 0x%08" PRIX32 "\n",
           SELECTED_CAMERA_SENSOR, FRAME_WIDTH, FRAME_HEIGHT,
           (uint32_t) framebuffer_pool,
           (uint32_t) FRAME_BUFFER_POOL_SIZE);

    printf("\n This command will dump memory from starting address to ending address");
    printf("\n and store it in to given path with filename.\n\n");

    printf("\r\n XXX Camera demo is halting here! XXX...\r\n");
    printf("\r\n Now User can dump captured/converted image data from memory address using any "
              "debugger!!!\r\n");
    printf("\r\n This application dumps the raw image data. To convert it into a viewable image format,\r\n please use an open-source tool.\r\n");


error_poweroff_camera:

    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Power OFF failed.\r\n");
    }

error_uninitialize_camera:
    ret = CAMERAdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");
    }

error_disable_hfosc_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSC,
                                              false,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE: MIPI 38.4Mhz(HFOSC) clock disable = %" PRId32 "\n", error_code);

    }
error_disable_100mhz_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              false,
                                              &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE_MIPI 100MHZ clock disable = %" PRId32 "\n", error_code);

    }

    printf("\r\n XXX Camera demo is exiting XXX...\r\n");

}

int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        while (1) {
        }
    }
#endif

    SystemCoreClockUpdate();

    camera_demo();

    while (1) {
    }

}
