/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     MT9M114_Camera_Sensor.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     29-Sep-2021
 * @brief    ONsemi MT9M114 Camera Sensor driver.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
/* System Includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "Camera_Sensor.h"
#include "Camera_Sensor_i2c.h"
#include "Driver_Common.h"

/* Proceed only if MT9M114 Camera Sensor is enabled. */
#if (RTE_MT9M114_CAMERA_SENSOR_CPI_ENABLE || RTE_MT9M114_CAMERA_SENSOR_LPCPI_ENABLE)

/* MT9M114 Camera Sensor Slave Address. */
#define MT9M114_CAMERA_SENSOR_SLAVE_ADDR                      0x48

/* MT9M114 Camera Sensor CHIP-ID registers */
#define MT9M114_CHIP_ID_REGISTER                              0x0000
#define MT9M114_CHIP_ID_REGISTER_VALUE                        0x2481

/* MT9M114 Camera Sensor Command registers */
#define MT9M114_COMMAND_REGISTER                              0x0080
#define MT9M114_COMMAND_REGISTER_SET_STATE                   (1 << 1)
#define MT9M114_COMMAND_REGISTER_OK                          (1 << 15)

/* MT9M114 Camera Sensor Sysctl registers */
#define MT9M114_SYSCTL_REGISTER_RESET_AND_MISC_CONTROL        0x001A
#define MT9M114_SYSCTL_REGISTER_SLEW_RATE_CONTROL             0x001e

/* MT9M114 Camera Sensor Camera Output Format Control registers */
#define MT9M114_CAM_OUTPUT_FORMAT_REGISTER                        0xc86c
#define MT9M114_CAM_OUTPUT_FORMAT_REGISTER_FORMAT_BAYER          (2 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_REGISTER_BAYER_FORMAT_RAWR10   (0 << 10)

/* MT9M114 Camera Sensor System Manager registers */
#define MT9M114_SYSMGR_NEXT_STATE                             0xdc00

/* MT9M114 Camera Sensor System States */
#define MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE                 0x28

#define MT9M114_SYS_STATE_STREAMING                           0x31
#define MT9M114_SYS_STATE_START_STREAMING                     0x34

#define MT9M114_SYS_STATE_ENTER_SUSPEND                       0x40
#define MT9M114_SYS_STATE_SUSPENDED                           0x41

#define MT9M114_SYS_STATE_ENTER_STANDBY                       0x50
#define MT9M114_SYS_STATE_STANDBY                             0x52
#define MT9M114_SYS_STATE_LEAVE_STANDBY                       0x54

/**
\brief MT9M114 Camera Sensor Register Array Structure
        used for Camera Resolution Configuration.
*/
typedef struct _MT9M114_REG {
  uint16_t reg_addr;             /* MT9M114 Camera Sensor Register Address                     */
  uint8_t  reg_size;             /* MT9M114 Camera Sensor Register Size: only valid 1/2/4 Byte */
  uint32_t reg_value;            /* MT9M114 Camera Sensor Register Value                       */
} MT9M114_REG;

/**
\brief MT9M114 Camera Sensor Resolution VGA 640x480
\note  Register Values are generated using
       MT9M114(SOC1040) Register Wizard Tool with
       below settings.
        - Image Timing :
          - Image                : VGA Binning
          - Frame Rate           : 5 Frame Per Second
          - Horizontal Mirror    : Enable
          - Vertical Flip        : Enable
        - PLL Setting :
          - Input Frequency      : 24 MHz
          - Target PLL Frequency : 96 MHz
          - Output mode          : Parallel
*/
const MT9M114_REG mt9m114_cam_resolution_VGA_640x480[] =
{
  { 0xC97E, 2, 0x01    }, /* cam_sysctl_pll_enable = 1 */
  { 0xC980, 2, 0x0120  }, /* cam_sysctl_pll_divider_m_n = 288 */
  { 0xC982, 2, 0x0700  }, /* cam_sysctl_pll_divider_p = 1792 */
  { 0xC984, 2, 0x8000  }, /* cam_port_output_control = 32768 (No pixel clock slow down) */
  { 0xC800, 2, 0x0000  }, /* cam_sensor_cfg_y_addr_start = 0 */
  { 0xC802, 2, 0x0000  }, /* cam_sensor_cfg_x_addr_start = 0 */
  { 0xC804, 2, 0x03CD  }, /* cam_sensor_cfg_y_addr_end = 973 */
  { 0xC806, 2, 0x050D  }, /* cam_sensor_cfg_x_addr_end = 1293 */
  { 0xC808, 4, 0x2DC6C00 }, /* cam_sensor_cfg_pixclk = 48000000 */
  { 0xC80C, 2, 0x0001  }, /* cam_sensor_cfg_row_speed = 1 */
  { 0xC80E, 2, 0x01C3  }, /* cam_sensor_cfg_fine_integ_min = 451 */
  { 0xC810, 2, 0x28F8  }, /* cam_sensor_cfg_fine_integ_max = 10488 */
  { 0xC812, 2, 0x036C  }, /* cam_sensor_cfg_frame_length_lines = 876 */
  { 0xC814, 2, 0x29E3  }, /* cam_sensor_cfg_line_length_pck = 10723 */
  { 0xC816, 2, 0x00E0  }, /* cam_sensor_cfg_fine_correction = 224 */
  { 0xC818, 2, 0x01E3  }, /* cam_sensor_cfg_cpipe_last_row = 483 */
  { 0xC826, 2, 0x0020  }, /* cam_sensor_cfg_reg_0_data = 32 */
  { 0xC834, 2, 0x0333  }, /* cam_sensor_control_read_mode = 819, H and V flip */
  { 0xC854, 2, 0x0000  }, /* cam_crop_window_xoffset = 0 */
  { 0xC856, 2, 0x0000  }, /* cam_crop_window_yoffset = 0 */
  { 0xC858, 2, 0x0280  }, /* cam_crop_window_width = 640 */
  { 0xC85A, 2, 0x01E0  }, /* cam_crop_window_height = 480 */
  { 0xC85C, 1, 0x03    }, /* cam_crop_cropmode = 3 */
  { 0xC868, 2, 0x0280  }, /* cam_output_width = 640 */
  { 0xC86A, 2, 0x01E0  }, /* cam_output_height = 480 */
  { 0xC878, 1, 0x00    }, /* cam_aet_aemode = 0 */
  { 0xC88C, 2, 0x051C  }, /* cam_aet_max_frame_rate = 1308, (5 fps) */
  { 0xC88E, 2, 0x051C  }, /* cam_aet_min_frame_rate = 1308, (5 fps) */
  { 0xC914, 2, 0x0000  }, /* cam_stat_awb_clip_window_xstart = 0 */
  { 0xC916, 2, 0x0000  }, /* cam_stat_awb_clip_window_ystart = 0 */
  { 0xC918, 2, 0x027F  }, /* cam_stat_awb_clip_window_xend = 639 */
  { 0xC91A, 2, 0x01DF  }, /* cam_stat_awb_clip_window_yend = 479 */
  { 0xC91C, 2, 0x0000  }, /* cam_stat_ae_initial_window_xstart = 0 */
  { 0xC91E, 2, 0x0000  }, /* cam_stat_ae_initial_window_ystart = 0 */
  { 0xC920, 2, 0x007F  }, /* cam_stat_ae_initial_window_xend = 127 */
  { 0xC922, 2, 0x005F  }, /* cam_stat_ae_initial_window_yend = 95 */
  { 0xA404, 2, 0x0003  }, /* Adaptive Weighted AE for lowlights
                             @NOTE: User can set as per Camera environment
                             refer data-sheet. */
};

/* For debugging,
 *  if required Enable MT9M114 Camera Sensor Test-Pattern.
 */
#define MT9M114_CAMERA_TEST_PATTERN_ENABLE     0

/* if MT9M114 Camera Sensor Test-Pattern is Enabled. */
#if MT9M114_CAMERA_TEST_PATTERN_ENABLE

/* Enable any one of Test-Pattern:
 *  - Color Bar
 *  - Walking 1's 08-bit
 *  - Walking 1's 10-bit
 */
#define MT9M114_CAMERA_TEST_PATTERN_COLOR_BAR_ENABLE            1   /* OR */
#define MT9M114_CAMERA_TEST_PATTERN_WALKING_1s_08_BIT_ENABLE    0   /* OR */
#define MT9M114_CAMERA_TEST_PATTERN_WALKING_1s_10_BIT_ENABLE    0
#endif

/* if MT9M114 Camera Sensor Test-Pattern is Enabled. */
#if MT9M114_CAMERA_TEST_PATTERN_ENABLE
const MT9M114_REG mt9m114_cam_testPattern[] =
{
  /* select operation mode as test-pattern generator. */
  {0xc84c, 1, 0x02},

  /* MT9M114 Camera Test Pattern: Color Bar */
#if MT9M114_CAMERA_TEST_PATTERN_COLOR_BAR_ENABLE
  {0xc84d, 1, 0x04},

  /* MT9M114 Camera Test Pattern: Walking 1's 08-bit */
#elif MT9M114_CAMERA_TEST_PATTERN_WALKING_1s_08_BIT_ENABLE
  {0xc84d, 1, 0x0B},

  /* MT9M114 Camera Test Pattern: Walking 1's 10-bit */
#elif MT9M114_CAMERA_TEST_PATTERN_WALKING_1s_10_BIT_ENABLE
  {0xc84d, 1, 0x0A},
#endif
};
#endif /* end of MT9M114_CAMERA_TEST_PATTERN_ENABLE */

/* I2C Driver Instance */
extern ARM_DRIVER_I2C ARM_Driver_I2C_(RTE_MT9M114_CAMERA_SENSOR_I2C_INSTANCE);

/**
\brief MT9M114 Camera Sensor slave i2c Configuration
        \ref CAMERA_SENSOR_SLAVE_I2C_CONFIG
*/
CAMERA_SENSOR_SLAVE_I2C_CONFIG mt9m114_camera_sensor_i2c_cnfg =
{
  .drv_i2c                        = &ARM_Driver_I2C_(RTE_MT9M114_CAMERA_SENSOR_I2C_INSTANCE),
  .bus_speed                      = ARM_I2C_BUS_SPEED_STANDARD,
  .cam_sensor_slave_addr          = MT9M114_CAMERA_SENSOR_SLAVE_ADDR,
  .cam_sensor_slave_reg_addr_type = CAMERA_SENSOR_I2C_REG_ADDR_TYPE_16BIT,
};

/* Wrapper function for Delay
 * Delay for millisecond:
 *  Provide busy loop delay
 */
#define MT9M114_DELAY_mSEC(msec)       sys_busy_loop_us(msec * 1000)

/* Wrapper function for i2c read
 *  read register value from MT9M114 Camera Sensor registers
 *   using i2c read API \ref camera_sensor_i2c_read
 *
 *  for MT9M114 Camera Sensor specific i2c configurations
 *   see \ref mt9m114_camera_sensor_i2c_cnfg
 */
#define MT9M114_READ_REG(reg_addr, reg_value, reg_size) \
        camera_sensor_i2c_read(&mt9m114_camera_sensor_i2c_cnfg, \
                                reg_addr,  \
                                reg_value, \
                                reg_size);

/* Wrapper function for i2c write
 *  write register value to MT9M114 Camera Sensor registers
 *   using i2c write API \ref camera_sensor_i2c_write.
 *
 *  for MT9M114 Camera Sensor specific i2c configurations
 *   see \ref mt9m114_camera_sensor_i2c_cnfg
 */
#define MT9M114_WRITE_REG(reg_addr, reg_value, reg_size) \
        camera_sensor_i2c_write(&mt9m114_camera_sensor_i2c_cnfg, \
                                 reg_addr,  \
                                 reg_value, \
                                 reg_size);

/**
  \fn           int32_t mt9m114_bulk_write_reg(const MT9M114_REG mt9m114_reg[],
                                                        uint32_t total_num)
  \brief        write array of registers value to MT9M114 Camera Sensor registers.
  \param[in]    mt9m114_reg : MT9M114 Camera Sensor Register Array Structure
                              \ref MT9M114_REG
  \param[in]    total_num   : total number of registers(size of array)
  \return       \ref execution_status
*/
static int32_t mt9m114_bulk_write_reg(const MT9M114_REG mt9m114_reg[],
                                               uint32_t total_num)
{
  uint32_t i  = 0;
  int32_t ret = 0;

  for(i = 0; i < total_num; i++)
  {
    ret = MT9M114_WRITE_REG(mt9m114_reg[i].reg_addr, mt9m114_reg[i].reg_value, \
                            mt9m114_reg[i].reg_size);
    if(ret != ARM_DRIVER_OK)
      return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_soft_reset(void)
  \brief        Software Reset MT9M114 Camera Sensor
  \param[in]    none
  \return       \ref execution_status
*/
static int32_t mt9m114_soft_reset(void)
{
  int32_t ret = 0;

  ret = MT9M114_WRITE_REG(MT9M114_SYSCTL_REGISTER_RESET_AND_MISC_CONTROL, 0x0001, 2);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  MT9M114_DELAY_mSEC(10);

  ret = MT9M114_WRITE_REG(MT9M114_SYSCTL_REGISTER_RESET_AND_MISC_CONTROL, 0x0000, 2);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* @Observation: more delay is required for Camera Sensor
   *               to setup after Soft Reset.
   */
  MT9M114_DELAY_mSEC(100);

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_camera_init(ARM_CAMERA_RESOLUTION cam_resolution,
                                                          uint8_t cam_output_format)
  \brief        Initialize MT9M114 Camera Sensor.
                 this function will
                  - configure Camera Sensor resolution registers as per input parameter.
                     (currently supports only VGA 640x480(WxH) Camera resolution)
                  - configure Camera Sensor output format registers as per input parameter
                     \ref MT9M114_USER_SELECT_CAMERA_OUTPUT_FORMAT.
                     (currently supports only RAW Bayer10 Foramat)
                  - configure Camera Sensor slew rate.
  \param[in]    cam_resolution    : Camera Sensor Resolution
                                     \ref ARM_CAMERA_RESOLUTION
  \param[in]    cam_output_format : Camera Sensor Output Format
                                     \ref MT9M114_USER_SELECT_CAMERA_OUTPUT_FORMAT
  \return       \ref execution_status
*/
static int32_t mt9m114_camera_init(void)
{
  uint32_t total_num     = 0;
  uint16_t output_format = 0;
  int32_t  ret = 0;

  total_num = (sizeof(mt9m114_cam_resolution_VGA_640x480) / sizeof(MT9M114_REG));
  ret = mt9m114_bulk_write_reg(mt9m114_cam_resolution_VGA_640x480, total_num);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Configure Camera Sensor slew rate */
  ret = MT9M114_WRITE_REG(MT9M114_SYSCTL_REGISTER_SLEW_RATE_CONTROL, 0x0, 2);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;


  output_format =  MT9M114_CAM_OUTPUT_FORMAT_REGISTER_FORMAT_BAYER         |   \
                       MT9M114_CAM_OUTPUT_FORMAT_REGISTER_BAYER_FORMAT_RAWR10;


  ret = MT9M114_WRITE_REG(MT9M114_CAM_OUTPUT_FORMAT_REGISTER, output_format, 2);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  return ARM_DRIVER_OK;
}

/* if MT9M114 Camera Sensor Test-Pattern is Enabled. */
#if MT9M114_CAMERA_TEST_PATTERN_ENABLE

/**
  \fn           int32_t mt9m114_camera_testPattern_config(void)
  \brief        Configure MT9M114 Camera Test-Pattern.
  \param[in]    none
  \return       \ref execution_status
*/
static int32_t mt9m114_camera_testPattern_config(void)
{
  uint32_t total_num     = 0;
  int32_t  ret = 0;

  total_num = (sizeof(mt9m114_cam_testPattern) / sizeof(MT9M114_REG));

  ret = mt9m114_bulk_write_reg(mt9m114_cam_testPattern, total_num);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  return ARM_DRIVER_OK;
}

#endif /* end of MT9M114_CAMERA_TEST_PATTERN_ENABLE */

/**
  \fn           int32_t mt9m114_wait_for_command(uint32_t command)
  \brief        wait for System State command to complete.
  \param[in]    command  : MT9M114 Camera Sensor command
  \return       \ref execution_status
*/
static int32_t mt9m114_wait_for_command(uint32_t command)
{
  uint32_t i = 0;
  uint32_t reg_value = 0;
  int32_t  ret = 0;

  /* wait for System State command to complete. */
  for(i = 0; i < 2000; ++i)
  {
    ret = MT9M114_READ_REG(MT9M114_COMMAND_REGISTER, &reg_value,2);
    if(ret != ARM_DRIVER_OK)
      return ARM_DRIVER_ERROR;

    if(!(reg_value & command))
      break;

    MT9M114_DELAY_mSEC(1);
  }

  if(reg_value & command)
    return ARM_DRIVER_ERROR;

  if(!(reg_value & MT9M114_COMMAND_REGISTER_OK))
    return ARM_DRIVER_ERROR;

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_set_system_state(uint8_t next_state)
  \brief        Set the desired next System State.
  \param[in]    next_state  : System State which needs to be set.
                  - \ref Valid MT9M114 Camera Sensor System States:
                    - MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE
                    - MT9M114_SYS_STATE_START_STREAMING
                    - MT9M114_SYS_STATE_ENTER_SUSPEND
                    - MT9M114_SYS_STATE_ENTER_STANDBY
                    - MT9M114_SYS_STATE_LEAVE_STANDBY
  \return       \ref execution_status
*/
static int32_t mt9m114_set_system_state(uint8_t next_state)
{
  int32_t ret = 0;

  /* Set the desired next System State */
  ret = MT9M114_WRITE_REG(MT9M114_SYSMGR_NEXT_STATE, next_state, 1);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Issue the Set State Command */
  ret = MT9M114_WRITE_REG(MT9M114_COMMAND_REGISTER,\
           MT9M114_COMMAND_REGISTER_OK | MT9M114_COMMAND_REGISTER_SET_STATE, 2);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Wait for the FW to complete the command. */
  ret = mt9m114_wait_for_command(MT9M114_COMMAND_REGISTER_SET_STATE);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_system_change_config(void)
  \brief        Issue Change-Config Command.
                This command must be issued after any change in
                 sensor sub-system registers to take effect,
                 for detail refer data-sheet.
                Change system state to MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE.
                The Change−Config performs the following operations:
                 1. Requests the sensor to stop STREAMING
                 2. Waits until the sensor stops STREAMING (this
                    can take an entire frame time depending on when
                    the command was issued)
                 3. When the sensor stops streaming, reconfigures all
                    subsystems including the sensor
                 4. Restarts the sensor
                 5. Command completes
  \param[in]    none
  \return       \ref execution_status
*/
static __inline int32_t mt9m114_system_change_config(void)
{
  return mt9m114_set_system_state(MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
}

/**
  \fn           int32_t mt9m114_stream_start(void)
  \brief        Start MT9M114 Camera Sensor Streaming,
                 change system state to MT9M114_SYS_STATE_START_STREAMING.
  \param[in]    none
  \return       \ref execution_status
*/
static __inline int32_t mt9m114_stream_start(void)
{
  return mt9m114_set_system_state(MT9M114_SYS_STATE_START_STREAMING);
}

/**
  \fn           int32_t mt9m114_stream_stop(void)
  \brief        Stop MT9M114 Camera Sensor Streaming,
                 change system state to MT9M114_SYS_STATE_ENTER_SUSPEND.
  \param[in]    none
  \return       \ref execution_status
*/
static __inline int32_t mt9m114_stream_stop(void)
{
  return mt9m114_set_system_state(MT9M114_SYS_STATE_ENTER_SUSPEND);
}

/**
  \fn           int32_t mt9m114_Init(void)
  \brief        Initialize MT9M114 Camera Sensor
                 this function will
                  - initialize i2c using i3c instance
                  - software reset MT9M114 Camera Sensor
                  - read MT9M114 chip-id, proceed only it is correct.
                  - initialize MT9M114 Camera Sensor as per input parameter
                    - Camera Resolution
                       (currently supports only VGA 640x480(WxH) Camera resolution)
                    - Camera Output Format
                       (currently supports only RAW Bayer10 format)
                    - Issue Change-Config Command to re-configure
                       all the MT9M114 Camera Sensor sub-system and registers.
                       this command must be issued after any change in
                       sensor registers to take effect, for detail refer data-sheet.
  \param[in]    cam_resolution  : Camera Resolution \ref ARM_CAMERA_RESOLUTION
  \return       \ref execution_status
*/
int32_t mt9m114_Init(void)
{
  int32_t  ret = 0;
  uint32_t rcv_data = 0;

  /* Initialize i2c using i3c driver instance depending on
   *  MT9M114 Camera Sensor specific i2c configurations
   *   \ref mt9m114_camera_sensor_i2c_cnfg
   */
  ret = camera_sensor_i2c_init(&mt9m114_camera_sensor_i2c_cnfg);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Soft Reset MT9M114 Camera Sensor */
  ret = mt9m114_soft_reset();
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Read MT9M114 Camera Sensor CHIP ID */
  ret = MT9M114_READ_REG(MT9M114_CHIP_ID_REGISTER, &rcv_data, 2);
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Proceed only if CHIP ID is correct. */
  if(rcv_data != MT9M114_CHIP_ID_REGISTER_VALUE)
    return ARM_DRIVER_ERROR;

  /* @NOTE: By-default after Soft-Reset Camera Sensor will be in streaming state,
   *        As per Hardware Jumper(P2 jumper) settings,
   *        if required then Stop Streaming using \ref mt9m114_stream_stop.
   *
   *        Suspend any stream
   *        ret = mt9m114_stream_stop();
   *        if(ret != ARM_DRIVER_OK)
   *          return ARM_DRIVER_ERROR;
   *        MT9M114_DELAY_mSEC(10);
   */
  /* Initialize the MT9M114 Camera Sensor */
  ret = mt9m114_camera_init();
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* Check if MT9M114 Camera Sensor Test-Pattern is Enabled? */
#if MT9M114_CAMERA_TEST_PATTERN_ENABLE

  /* yes then configure camera Test-Pattern. */
  ret = mt9m114_camera_testPattern_config();
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

#endif  /* end of MT9M114_CAMERA_TEST_PATTERN_ENABLE */

  /* @NOTE: Issue Change-Config Command to re-configure
   *        all the MT9M114 Camera Sensor sub-system and registers.
   *
   *        This command must be issued after any change in
   *        sensor sub-system registers to take effect,
   *        for detail refer data-sheet.
   */
  ret = mt9m114_system_change_config();
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_Start(void)
  \brief        Start MT9M114 Camera Sensor Streaming.
  \param[in]    none
  \return       \ref execution_status
*/
int32_t mt9m114_Start(void)
{
  int32_t ret = 0;

  /* Start streaming */
  ret = mt9m114_stream_start();
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  /* @Observation: Proper Delay is required for
   *               Camera Sensor Lens to come-out from Shutter and gets steady,
   *               otherwise captured image will not be proper.
   *               adjust delay if captured image is less bright/dull.
   *
   *               As it is directly depends on Camera Setup(Light and other Environment parameters),
   *               user can add extra delay if required.
   *               MT9M114_DELAY_mSEC(2000);
   */

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_Stop(void)
  \brief        Stop MT9M114 Camera Sensor Streaming.
  \param[in]    none
  \return       \ref execution_status
*/
int32_t mt9m114_Stop(void)
{
  int32_t ret = 0;

  /* Suspend any stream */
  ret = mt9m114_stream_stop();
  if(ret != ARM_DRIVER_OK)
    return ARM_DRIVER_ERROR;

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_Control(uint32_t control, uint32_t arg)
  \brief        Control MT9M114 Camera Sensor.
  \param[in]    control  : Operation
  \param[in]    arg      : Argument of operation
  \return       \ref execution_status
*/
int32_t mt9m114_Control(uint32_t control, uint32_t arg)
{
  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t mt9m114_Uninit(void)
  \brief        Un-initialize MT9M114 Camera Sensor.
  \param[in]    none
  \return       \ref execution_status
*/
int32_t mt9m114_Uninit(void)
{
  return ARM_DRIVER_OK;
}

/**
\brief MT9M114 Camera Sensor Operations
        \ref CAMERA_SENSOR_OPERATIONS
*/
CAMERA_SENSOR_OPERATIONS mt9m114_ops =
{
  .Init    = mt9m114_Init,
  .Uninit  = mt9m114_Uninit,
  .Start   = mt9m114_Start,
  .Stop    = mt9m114_Stop,
  .Control = mt9m114_Control,
};

#if RTE_CPI
/**
\brief CPI MT9M114 Camera Sensor Configurations
        \ref CPI_INFO
*/
CPI_INFO cpi_mt9m114_config =
{
  .interface       = CPI_INTERFACE_PARALLEL,
  .vsync_wait      = RTE_MT9M114_CAMERA_SENSOR_CPI_VSYNC_WAIT,
  .vsync_mode      = RTE_MT9M114_CAMERA_SENSOR_CPI_VSYNC_MODE,
  .pixelclk_pol    = RTE_MT9M114_CAMERA_SENSOR_CPI_PIXEL_CLK_POL,
  .hsync_pol       = RTE_MT9M114_CAMERA_SENSOR_CPI_HSYNC_POL,
  .vsync_pol       = RTE_MT9M114_CAMERA_SENSOR_CPI_VSYNC_POL,
  .data_mode       = RTE_MT9M114_CAMERA_SENSOR_CPI_DATA_MODE,
  .data_endianness = RTE_MT9M114_CAMERA_SENSOR_CPI_DATA_ENDIANNESS,
  .code10on8       = RTE_MT9M114_CAMERA_SENSOR_CPI_CODE10ON8,
  .data_mask       = RTE_MT9M114_CAMERA_SENSOR_CPI_DATA_MASK,
};

/**
\brief CPI MT9M114 Camera Sensor Device Structure
       Contains:
        - CPI MT9M114 Camera Sensor Configurations
        - MT9M114 Camera Sensor Operations
        \ref CAMERA_SENSOR_DEVICE
*/
static CAMERA_SENSOR_DEVICE cpi_mt9m114_camera_sensor =
{
  .width       = RTE_MT9M114_CAMERA_SENSOR_FRAME_WIDTH,
  .height      = RTE_MT9M114_CAMERA_SENSOR_FRAME_HEIGHT,
  .cpi_info    = &cpi_mt9m114_config,
  .ops         = &mt9m114_ops,
};

/* Registering CPI sensor */
CAMERA_SENSOR(cpi_mt9m114_camera_sensor)
#endif

#if RTE_LPCPI
/**
\brief LPCPI MT9M114 Camera Sensor Configurations
        \ref CAMERA_SENSOR_CONFIG
*/
CPI_INFO lpcpi_mt9m114_config =
{
  .interface       = CPI_INTERFACE_PARALLEL,
  .pixelclk_pol    = RTE_MT9M114_CAMERA_SENSOR_LPCPI_PIXEL_CLK_POL,
  .hsync_pol       = RTE_MT9M114_CAMERA_SENSOR_LPCPI_HSYNC_POL,
  .vsync_pol       = RTE_MT9M114_CAMERA_SENSOR_LPCPI_VSYNC_POL,
  .vsync_wait      = RTE_MT9M114_CAMERA_SENSOR_LPCPI_VSYNC_WAIT,
  .vsync_mode      = RTE_MT9M114_CAMERA_SENSOR_LPCPI_VSYNC_MODE,
  .data_mode       = RTE_MT9M114_CAMERA_SENSOR_LPCPI_DATA_MODE,
  .data_endianness = RTE_MT9M114_CAMERA_SENSOR_LPCPI_DATA_ENDIANNESS,
  .code10on8       = RTE_MT9M114_CAMERA_SENSOR_LPCPI_CODE10ON8,
};

/**
\brief LPCPI MT9M114 Camera Sensor Device Structure
       Contains:
        - LPCPI MT9M114 Camera Sensor Configurations
        - MT9M114 Camera Sensor Operations
        \ref CAMERA_SENSOR_DEVICE
*/
static CAMERA_SENSOR_DEVICE lpcpi_mt9m114_camera_sensor =
{
  .width       = RTE_MT9M114_CAMERA_SENSOR_FRAME_WIDTH,
  .height      = RTE_MT9M114_CAMERA_SENSOR_FRAME_HEIGHT,
  .cpi_info    = &lpcpi_mt9m114_config,
  .ops         = &mt9m114_ops,
};

/* Registering CPI sensor */
LPCAMERA_SENSOR(lpcpi_mt9m114_camera_sensor)
#endif



#endif /* RTE_MT9M114_CAMERA_SENSOR_ENABLE */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
