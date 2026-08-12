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

#include "Driver_IO.h"

// set to 0: enable selfie camera (cam1)
// set to 1: enable standard camera (cam2)
#define STANDARD_CAM_EN 0



extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

#if STANDARD_CAM_EN
/* Switch Camera target. */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAMERA_I2C_C1_C2_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_SWITCH_CAM =
        &ARM_Driver_GPIO_(BOARD_CAMERA_I2C_C1_C2_GPIO_PORT);
#endif

#ifndef CAMERA_SENSOR_SELECT
#define CAMERA_SENSOR_SELECT         CAMERA_SENSOR_ARX3A0
#endif

#define CAMERA_SENSOR_ARX3A0         0
#define CAMERA_SENSOR_MT9M114        1
#define CAMERA_SENSOR_OV5675         2


#if (CAMERA_SENSOR_SELECT == CAMERA_SENSOR_ARX3A0)
  #define SELECTED_CAMERA_SENSOR     "ARX3A0"
  #define FRAME_WIDTH                RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH
  #define FRAME_HEIGHT               RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT
  #define BYTES_PER_PIXEL 1

#elif (CAMERA_SENSOR_SELECT == CAMERA_SENSOR_MT9M114)
  #define SELECTED_CAMERA_SENSOR     "MT9M114"
  #define FRAME_WIDTH                 RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH
  #define FRAME_HEIGHT                RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT
  #if (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 0 ||  \
       RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 1)
     #define BYTES_PER_PIXEL 1
  #elif(RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG >= 2 &&  \
        RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG <= 5)
     #define BYTES_PER_PIXEL 2
  #endif

#elif (CAMERA_SENSOR_SELECT == CAMERA_SENSOR_OV5675)
  #define SELECTED_CAMERA_SENSOR     "OV5675"
  #define FRAME_WIDTH                RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH
  #define FRAME_HEIGHT               RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT
  #define BYTES_PER_PIXEL 1

#else
  #error "Invalid CAMERA_SENSOR_SELECT! Valid: ARX3A0, MT9M114, OV5675"
#endif


#define FRAME_BUFFER_POOL_SIZE ((FRAME_WIDTH) * (FRAME_HEIGHT) * (BYTES_PER_PIXEL))

uint8_t framebuffer_pool[FRAME_BUFFER_POOL_SIZE]
    __attribute__((section(".bss.camera_frame_buf"), aligned(32)));

typedef enum {
    CAM_CB_EVENT_CAPTURE_STOPPED = (1 << 0),
    CAM_CB_EVENT_ERROR           = (1 << 1)
} CAMERA_CB_EVENTS;

volatile uint32_t event_flag;

void camera_callback(uint32_t event)
{
	/*Capture finished normally */

	if (event & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED) {
		event_flag |= CAM_CB_EVENT_CAPTURE_STOPPED;
    }

	/*FIFO overrun or hardware error occurred */

	if (event & (ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
			    ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
			    ARM_CPI_EVENT_ERR_HARDWARE)) {
          event_flag |= CAM_CB_EVENT_ERROR;
	}
}


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


    printf("\n \t frame buffer pool size: 0x%0X  pool addr: 0x%08" PRIx32 " \r\n ",
    		FRAME_BUFFER_POOL_SIZE, (uint32_t)framebuffer_pool);


    ret = hardware_init();
    if (ret != 0) {
         printf("Error: CAMERA Hardware Initialize failed: %" PRId32 "\n", ret);
         return;
        }


    /* Initialize SE services */
    se_services_port_init();

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
    		                                  CLKEN_CLK_100M,
											  true, &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
        printf("SE : MIPI 100MHz clock enable = %"PRId32 "\n", error_code);
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

#if STANDARD_CAM_EN
    ret = GPIO_Driver_SWITCH_CAM->Initialize(BOARD_CAMERA_I2C_C1_C2_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: GPIO Initialize failed (pin: %d, ret=%" PRId32 ")\r\n",
               BOARD_CAMERA_I2C_C1_C2_GPIO_PIN, ret);
        goto error_disable_hfosc_clk;
    }

    ret = GPIO_Driver_SWITCH_CAM->PowerControl(BOARD_CAMERA_I2C_C1_C2_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: GPIO PowerControl failed (pin: %d, ret=%" PRId32 ")\r\n",
               BOARD_CAMERA_I2C_C1_C2_GPIO_PIN, ret);
        goto error_uninitialize_gpio;
    }

    ret = GPIO_Driver_SWITCH_CAM->SetDirection(BOARD_CAMERA_I2C_C1_C2_GPIO_PIN,
                                               GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: GPIO SetDirection failed (pin: %d, ret=%" PRId32 ")\r\n",
               BOARD_CAMERA_I2C_C1_C2_GPIO_PIN, ret);
        goto error_poweroff_gpio;
    }

    ret = GPIO_Driver_SWITCH_CAM->SetValue(BOARD_CAMERA_I2C_C1_C2_GPIO_PIN,
                                           GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: GPIO SetValue failed (pin: %d, ret=%" PRId32 ")\r\n",
               BOARD_CAMERA_I2C_C1_C2_GPIO_PIN, ret);
        goto error_poweroff_gpio;
    }
#endif

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
        printf("Capture complete\r\n");
    }

    ret = CAMERAdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
    	 printf("\r\n Error: CAMERA stop Capture failed.\r\n");
         goto error_poweroff_camera;
    }

    printf("\n To dump memory using ARM DS with Ulink Pro Debugger or Trace32 :");
    printf("\n  Use below commands in Commands tab: update user directory name \r\n");

    printf("Ulink: ~/%s_%dx%d.bin 0x%" PRIX32 " 0x%" PRIX32 "\n",
           SELECTED_CAMERA_SENSOR, FRAME_WIDTH, FRAME_HEIGHT,
           (uint32_t) framebuffer_pool,
           (uint32_t) (framebuffer_pool + FRAME_BUFFER_POOL_SIZE - 1));
    printf("T32:\n   data.save.binary ~/cam_image0_%s_%dx%d.bin 0x%" PRIx32
            "--0x%" PRIX32 " \r\n",
           SELECTED_CAMERA_SENSOR, FRAME_WIDTH, FRAME_HEIGHT,
           (uint32_t) framebuffer_pool,
           (uint32_t) (framebuffer_pool + FRAME_BUFFER_POOL_SIZE - 1));

    printf("\n  This command will dump memory from staring address to ending address \r");
    printf("\n  and store it in to given path with filename.\r\n\r\n");

    printf("\r\n\r\n XXX Camera demo is halting here! XXX...\r\n");
    printf("\r\n Now User can dump captured/converted image data from memory address using any "
              "debugger!!!\r\n");


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

#if STANDARD_CAM_EN
error_poweroff_gpio:
    /* Power off GPIO peripheral */
    ret = GPIO_Driver_SWITCH_CAM->PowerControl(BOARD_CAMERA_I2C_C1_C2_GPIO_PIN, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: GPIO Power OFF failed.\r\n");
    }

error_uninitialize_gpio:
    /* Un-initialize GPIO driver */
    ret = GPIO_Driver_SWITCH_CAM->Uninitialize(BOARD_CAMERA_I2C_C1_C2_GPIO_PIN);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: GPIO Uninitialize failed.\r\n");
    }
#endif

error_disable_hfosc_clk:
    error_code =
    		SERVICES_clocks_enable_clock(se_services_s_handle,
    				                     CLKEN_HFOSC,
    				                     false,
										 &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
    	printf("SE: MIPI 38.4Mhz(HFOSC) clock disable = %" PRId32 "\n", error_code);

    }
error_disable_100mhz_clk:
    error_code =
    		SERVICES_clocks_enable_clock(se_services_s_handle,
    				                     CLKEN_CLK_100M,
    				                     false,
										 &service_error_code);
    if (error_code != SERVICES_REQ_SUCCESS) {
    	printf("SE_MIPI 100MHZ clock disable = %"PRId32"\n", error_code);

    }

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













