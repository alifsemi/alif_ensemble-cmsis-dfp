/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* System Includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "Camera_Sensor.h"
#include "Camera_Sensor_i2c.h"
#include "Driver_IO.h"
#include "Driver_CPI.h"
#include "sys_ctrl_cpi.h"
#include "sys_utils.h"

/* Proceed only if AR0246 Camera Sensor is enabled. */
#if RTE_AR0246_CAMERA_SENSOR_CSI_ENABLE

/* I2C Instance */
#if (RTE_AR0246_CAMERA_SENSOR_I2C_INSTANCE == 4)
#define CAMERA_SENSOR_I2C_INSTANCE I3C
#else
#define CAMERA_SENSOR_I2C_INSTANCE RTE_AR0246_CAMERA_SENSOR_I2C_INSTANCE
#endif

/* AR0246 Camera Sensor Slave Address. */
#define AR0246_CAMERA_SENSOR_SLAVE_ADDR 0x10

/* AR0246 Camera Sensor CHIP-ID registers */
#define AR0246_CHIP_ID_REGISTER         0x3000
#define AR0246_CHIP_ID_REGISTER_VALUE   0x1F56

/* AR0246 Camera Sensor registers index */
#define AR0246_RESET_REGISTER           0x301A

/* Wrapper function for Delay
 * Delay for microsecond:
 * Provide delay using PMU(Performance Monitoring Unit).
 */
#define AR0246_DELAY_uSEC(usec)         sys_busy_loop_us(usec)

/* Wrapper function for i2c read
 *  read register value from AR0246 Camera Sensor registers
 *   using i2c read API \ref camera_sensor_i2c_read
 *
 *  for AR0246 Camera Sensor specific i2c configurations
 *   see \ref AR0246_camera_sensor_i2c_cnfg
 */
#define AR0246_READ_REG(reg_addr, reg_value, reg_size)                                             \
    camera_sensor_i2c_read(&AR0246_camera_sensor_i2c_cnfg,                                         \
                           reg_addr,                                                               \
                           reg_value,                                                              \
                           (CAMERA_SENSOR_I2C_REG_SIZE) reg_size)

/* Wrapper function for i2c write
 *  write register value to AR0246 Camera Sensor registers
 *   using i2c write API \ref camera_sensor_i2c_write.
 *
 *  for AR0246 Camera Sensor specific i2c configurations
 *   see \ref AR0246_camera_sensor_i2c_cnfg
 */
#define AR0246_WRITE_REG(reg_addr, reg_value, reg_size)                                            \
    camera_sensor_i2c_write(&AR0246_camera_sensor_i2c_cnfg,                                        \
                            reg_addr,                                                              \
                            reg_value,                                                             \
                            (CAMERA_SENSOR_I2C_REG_SIZE) reg_size)

/* AR0246 Camera reset GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(RTE_AR0246_CAMERA_SENSOR_RESET_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_CAM_RST =
    &ARM_Driver_GPIO_(RTE_AR0246_CAMERA_SENSOR_RESET_GPIO_PORT);

/* AR0246 Camera power GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(RTE_AR0246_CAMERA_SENSOR_POWER_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_CAM_PWR =
    &ARM_Driver_GPIO_(RTE_AR0246_CAMERA_SENSOR_POWER_GPIO_PORT);

/* I2C Driver Instance */
extern ARM_DRIVER_I2C ARM_Driver_I2C_(CAMERA_SENSOR_I2C_INSTANCE);

/**
  \brief AR0246 Camera Sensor Register Array Structure
  used for Camera Resolution Configuration.
  */
typedef struct _AR0246_REG {
    uint16_t reg_addr;  /* AR0246 Camera Sensor Register Address*/
    uint16_t reg_value; /* AR0246 Camera Sensor Register Value*/
} AR0246_REG;

/**
  \brief AR0246 Camera Sensor slave i2c Configuration
  \ref CAMERA_SENSOR_SLAVE_I2C_CONFIG
  */
static CAMERA_SENSOR_SLAVE_I2C_CONFIG AR0246_camera_sensor_i2c_cnfg = {
    .drv_i2c                        = &ARM_Driver_I2C_(CAMERA_SENSOR_I2C_INSTANCE),
    .bus_speed                      = ARM_I2C_BUS_SPEED_STANDARD,
    .cam_sensor_slave_addr          = AR0246_CAMERA_SENSOR_SLAVE_ADDR,
    .cam_sensor_slave_reg_addr_type = CAMERA_SENSOR_I2C_REG_ADDR_TYPE_16BIT,
};

static const AR0246_REG cfg_684x472_12bit_40fps_2Lane_sensor_reg_settings[] = {
    /* PLL_27MHz_891Mbps_148p5MHz_12bit(device, clock_multiplier) */
    {0x3030, 0x84},  // PLL_MULTIPLIER
    {0x302E, 0x2},   // PRE_PLL_DIVIDER
    {0x302C, 0x2},   // P1_DIVIDER
    {0x302A, 0x6},   // P2_DIVIDER
    {0x3038, 0x4},   // P3_DIVIDER
    {0x3036, 0x6},   // P4_DIVIDER

    /* MIPI_TIMING_891Mbps_148p5MHz_12bit(device) */
    {0x31B0, 0x4A},    // Frame preamble
    {0x31B2, 0x24},    // Line preamble
    {0x31B4, 0x51C7},  // MIPI timing0
    {0x31B6, 0x4247},  // MIPI timing1
    {0x31B8, 0x60C9},  // MIPI timing2
    {0x31BA, 0x28A},   // MIPI timing3
    {0x31BC, 0x8B88},  // MIPI timing4
    {0x3342, 0x122C},  // MIPI F1 PDT EDT

    /* Design_Linear_Analog_Recommended_Settings(device) */
    /* Design_Eclipse_Recommended_Settings(device) */
    {0x352A, 0x6F6F},
    {0x352E, 0x6698},
    {0x3530, 0x2508},
    {0x3528, 0xFB84},
    {0x3C70, 0x642C},
    {0x3428, 0x0209},
    {0x3520, 0x6804},
    {0x3540, 0x3597},
    {0x350A, 0x0654},
    {0x3532, 0x4C82},
    {0x3534, 0x4E60},
    {0x353E, 0x981C},
    {0x351A, 0x4FFF},
    {0x3092, 0x0926},
    {0x3522, 0x6897},
    {0x351E, 0x5B5E},
    {0x3086, 0x1101},
    {0x30BA, 0x013E},
    {0x3364, 232},
    {0x3508, 0x40BB},
    {0x3782, 0x0160},
    {0x3784, 0x0160},
    {0x3750, 0x0160},
    {0x3752, 0x0160},
    {0x3096, 0x0160},
    {0x3098, 0x0160},
    {0x58E2, 0x3FEA},
    {0x58E6, 0x0BAC},
    {0x3522, 0x6897},
    {0x351E, 0x5B5E},
    {0x3534, 0x4E64},
    {0x353C, 0x221C},
    {0x3516, 0xFFF4},

    /* Design_eHDR_Recommended_Settings(device) */
    /* HDR_MEC_Enable(device) */
    {0x3D02, 0x3833},
    {0x3D64, 0xED8},  // CLIP_1_0
    {0x3D66, 0xED8},  // CLIP_1_1
    {0x3D68, 0xED8},  // CLIP_1_2
    {0x3D6A, 0xED8},  // CLIP_1_3
    {0x3D6C, 0xED8},  // CLIP_2_0
    {0x3D6E, 0xED8},  // CLIP_2_1
    {0x3D70, 0xED8},  // CLIP_2_2
    {0x3D72, 0xED8},  // CLIP_2_3
    {0x3D74, 0xED8},  // CLIP_3_0
    {0x3D76, 0xED8},  // CLIP_3_1
    {0x3D78, 0xED8},  // CLIP_3_2
    {0x3D7A, 0xED8},  // CLIP_3_3
    {0x3D28, 0xACD},  // T1_STR_DEC_TH
    {0x30B0, 0x820},

    /* HDR_MEC_SC_Enable(device) */
    {0x3D00, 0x300F},
    {0x3D02, 0x3833},
    {0x3D28, 0xACD},
    {0x3D2A, 0xED8},
    {0x3D34, 0xACD0},
    {0x3D36, 0xED80},
    {0x3D5E, 0x0},
    {0x3D40, 0xFFFE},
    {0x3D42, 0xFFFF},
    {0x3D60, 0xF0F},
    {0x3D64, 0xED8},  // CLIP_1_0
    {0x3D66, 0xED8},  // CLIP_1_1
    {0x3D68, 0xED8},  // CLIP_1_2
    {0x3D6A, 0xED8},  // CLIP_1_3
    {0x3D6C, 0xED8},  // CLIP_2_0
    {0x3D6E, 0xED8},  // CLIP_2_1
    {0x3D70, 0xED8},  // CLIP_2_2
    {0x3D72, 0xED8},  // CLIP_2_3
    {0x3D74, 0xED8},  // CLIP_3_0
    {0x3D76, 0xED8},  // CLIP_3_1
    {0x3D78, 0xED8},  // CLIP_3_2
    {0x3D7A, 0xED8},  // CLIP_3_3
    {0x30B0, 0x820},

    /* DR_MEC_MC_settings(device) */
    {0x3124, 0x6D},
    {0x3126, 0x3C},
    {0x3116, 0xC001},

    /* Sequencer_select(device) */
    {0x2512, 0xA000},
    {0x2510, 0x0720},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0x2123},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0x2760},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0xFFFF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x0F8C},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20E0},
    {0x2510, 0x8055},
    {0x2510, 0xA0E1},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3088},
    {0x2510, 0x3282},
    {0x2510, 0xA681},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FE},
    {0x2510, 0x9070},
    {0x2510, 0x891D},
    {0x2510, 0x867F},
    {0x2510, 0x20FF},
    {0x2510, 0x20FC},
    {0x2510, 0x893F},
    {0x2510, 0x0F92},
    {0x2510, 0x20E0},
    {0x2510, 0x0F8F},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20E0},
    {0x2510, 0x9770},
    {0x2510, 0x20FC},
    {0x2510, 0x8054},
    {0x2510, 0x896C},
    {0x2510, 0x200A},
    {0x2510, 0x9030},
    {0x2510, 0x200A},
    {0x2510, 0x8040},
    {0x2510, 0x8948},
    {0x2510, 0x200A},
    {0x2510, 0x1597},
    {0x2510, 0x8808},
    {0x2510, 0x200A},
    {0x2510, 0x1F96},
    {0x2510, 0x20FF},
    {0x2510, 0x20E0},
    {0x2510, 0xA0C0},
    {0x2510, 0x200A},
    {0x2510, 0x3044},
    {0x2510, 0x3088},
    {0x2510, 0x3282},
    {0x2510, 0x2004},
    {0x2510, 0x1FAA},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20FF},
    {0x2510, 0x20E0},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x20FF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x3108},
    {0x2510, 0x2400},
    {0x2510, 0x2401},
    {0x2510, 0x3244},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x3108},
    {0x2510, 0x2400},
    {0x2510, 0x2401},
    {0x2510, 0x2702},
    {0x2510, 0x3242},
    {0x2510, 0x3108},
    {0x2510, 0x2420},
    {0x2510, 0x2421},
    {0x2510, 0x2703},
    {0x2510, 0x3242},
    {0x2510, 0x3108},
    {0x2510, 0x2420},
    {0x2510, 0x2421},
    {0x2510, 0x2704},
    {0x2510, 0x3244},
    {0x2510, 0x7FFF},
    {0x2510, 0x2000},
    {0x2510, 0x8801},
    {0x2510, 0x008F},
    {0x2510, 0x8855},
    {0x2510, 0x3101},
    {0x2510, 0x2000},
    {0x2510, 0x3041},
    {0x2510, 0x1051},
    {0x2510, 0x3102},
    {0x2510, 0x3041},
    {0x2510, 0x3181},
    {0x2510, 0x2000},
    {0x2510, 0x3041},
    {0x2510, 0x0014},
    {0x2510, 0x3188},
    {0x2510, 0x3041},
    {0x2510, 0x003B},
    {0x2510, 0x3282},
    {0x2510, 0x3104},
    {0x2510, 0x30C1},
    {0x2510, 0xB0E4},
    {0x2510, 0xA992},
    {0x2510, 0x1007},
    {0x2510, 0xB800},
    {0x2510, 0x1025},
    {0x2510, 0x0051},
    {0x2510, 0x1014},
    {0x2510, 0x1006},
    {0x2510, 0x0033},
    {0x2510, 0xC020},
    {0x2510, 0xB1E0},
    {0x2510, 0x0030},
    {0x2510, 0x0007},
    {0x2510, 0xAA5A},
    {0x2510, 0x8095},
    {0x2510, 0xA228},
    {0x2510, 0x100F},
    {0x2510, 0x00A8},
    {0x2510, 0x1082},
    {0x2510, 0x02D6},
    {0x2510, 0xA620},
    {0x2510, 0x8891},
    {0x2510, 0x11BD},
    {0x2510, 0x1051},
    {0x2510, 0x0023},
    {0x2510, 0xCA36},
    {0x2510, 0x10A9},
    {0x2510, 0x99CD},
    {0x2510, 0x2004},
    {0x2510, 0x0C4E},
    {0x2510, 0x113B},
    {0x2510, 0xC000},
    {0x2510, 0x0342},
    {0x2510, 0x00C1},
    {0x2510, 0x10D5},
    {0x2510, 0x06C5},
    {0x2510, 0x1342},
    {0x2510, 0x0295},
    {0x2510, 0x9008},
    {0x2510, 0x200F},
    {0x2510, 0x06CB},
    {0x2510, 0x000E},
    {0x2510, 0x1022},
    {0x2510, 0x1015},
    {0x2510, 0xA84B},
    {0x2510, 0x002C},
    {0x2510, 0x80B9},
    {0x2510, 0x104B},
    {0x2510, 0x1056},
    {0x2510, 0x0024},
    {0x2510, 0x9B0F},
    {0x2510, 0x0535},
    {0x2510, 0x0692},
    {0x2510, 0x102C},
    {0x2510, 0x009A},
    {0x2510, 0x3002},
    {0x2510, 0x2007},
    {0x2510, 0x0028},
    {0x2510, 0x8091},
    {0x2510, 0x1024},
    {0x2510, 0x0021},
    {0x2510, 0x002F},
    {0x2510, 0xCD26},
    {0x2510, 0x10AD},
    {0x2510, 0x9000},
    {0x2510, 0x108E},
    {0x2510, 0x1A28},
    {0x2510, 0x122F},
    {0x2510, 0x11B5},
    {0x2510, 0x129D},
    {0x2510, 0x1C59},
    {0x2510, 0x0095},
    {0x2510, 0x074B},
    {0x2510, 0x1021},
    {0x2510, 0x0020},
    {0x2510, 0x1095},
    {0x2510, 0x1030},
    {0x2510, 0x104B},
    {0x2510, 0xB106},
    {0x2510, 0xC490},
    {0x2510, 0xA882},
    {0x2510, 0x8255},
    {0x2510, 0xB100},
    {0x2510, 0xC802},
    {0x2510, 0x8801},
    {0x2510, 0x00A9},
    {0x2510, 0x0022},
    {0x2510, 0x00D1},
    {0x2510, 0x0201},
    {0x2510, 0x98BB},
    {0x2510, 0x2006},
    {0x2510, 0x0D8A},
    {0x2510, 0x0036},
    {0x2510, 0x001D},
    {0x2510, 0x9B3F},
    {0x2510, 0x101B},
    {0x2510, 0x0039},
    {0x2510, 0x1001},
    {0x2510, 0x0040},
    {0x2510, 0x101F},
    {0x2510, 0x104C},
    {0x2510, 0x3081},
    {0x2510, 0x100A},
    {0x2510, 0x002A},
    {0x2510, 0x3044},
    {0x2510, 0x2001},
    {0x2510, 0x1023},
    {0x2510, 0x102A},
    {0x2510, 0x2020},
    {0x2510, 0x121D},
    {0x2510, 0x04B8},
    {0x2510, 0x10B8},
    {0x2510, 0x1040},
    {0x2510, 0xB500},
    {0x2510, 0x01A3},
    {0x2510, 0x022B},
    {0x2510, 0x30D0},
    {0x2510, 0x00CC},
    {0x2510, 0x3141},
    {0x2510, 0x3041},
    {0x2510, 0x0044},
    {0x2510, 0x3142},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3110},
    {0x2510, 0x2000},
    {0x2510, 0x00B5},
    {0x2510, 0x996B},
    {0x2510, 0x3041},
    {0x2510, 0x1044},
    {0x2510, 0x3120},
    {0x2510, 0x3041},
    {0x2510, 0x1045},
    {0x2510, 0x3144},
    {0x2510, 0x3041},
    {0x2510, 0x987B},
    {0x2510, 0x3148},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3182},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3194},
    {0x2510, 0x2001},
    {0x2510, 0x3041},
    {0x2510, 0x3088},
    {0x2510, 0x31A0},
    {0x2510, 0x2000},
    {0x2510, 0x807D},
    {0x2510, 0x2006},
    {0x2510, 0x8815},
    {0x2510, 0x8877},
    {0x2510, 0x0A92},
    {0x2510, 0x2201},
    {0x2510, 0x2000},
    {0x2510, 0x2206},
    {0x2510, 0x2002},
    {0x2510, 0x8055},
    {0x2510, 0x3001},
    {0x2510, 0x2005},
    {0x2510, 0x8C61},
    {0x2510, 0x8801},
    {0x2510, 0x1112},
    {0x2510, 0x151F},
    {0x2510, 0x089F},
    {0x2510, 0x0057},
    {0x2510, 0x0036},
    {0x2510, 0x986E},
    {0x2510, 0x0023},
    {0x2510, 0x10CC},
    {0x2510, 0x00B9},
    {0x2510, 0x0040},
    {0x2510, 0x1035},
    {0x2510, 0x1321},
    {0x2510, 0x9826},
    {0x2510, 0x00C8},
    {0x2510, 0xC85B},
    {0x2510, 0x2008},
    {0x2510, 0x1048},
    {0x2510, 0x3281},
    {0x2510, 0x00AA},
    {0x2510, 0x3044},
    {0x2510, 0x11AB},
    {0x2510, 0x102A},
    {0x2510, 0x08C9},
    {0x2510, 0x010C},
    {0x2510, 0x1149},
    {0x2510, 0x10D7},
    {0x2510, 0x1154},
    {0x2510, 0x1140},
    {0x2510, 0x1041},
    {0x2510, 0x9A25},
    {0x2510, 0xBAA0},
    {0x2510, 0xB06D},
    {0x2510, 0x004D},
    {0x2510, 0x1046},
    {0x2510, 0x1020},
    {0x2510, 0x0047},
    {0x2510, 0xB064},
    {0x2510, 0x1047},
    {0x2510, 0x0025},
    {0x2510, 0x99C5},
    {0x2510, 0xC810},
    {0x2510, 0x1025},
    {0x2510, 0x00B5},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x3250},
    {0x2510, 0x8801},
    {0x2510, 0x010F},
    {0x2510, 0x8855},
    {0x2510, 0x3101},
    {0x2510, 0x3041},
    {0x2510, 0x1051},
    {0x2510, 0x3102},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3181},
    {0x2510, 0x3041},
    {0x2510, 0x0014},
    {0x2510, 0x3188},
    {0x2510, 0x3041},
    {0x2510, 0x003B},
    {0x2510, 0x3282},
    {0x2510, 0x3104},
    {0x2510, 0x30C1},
    {0x2510, 0xB0E4},
    {0x2510, 0xA992},
    {0x2510, 0x1007},
    {0x2510, 0xB800},
    {0x2510, 0x1025},
    {0x2510, 0x0051},
    {0x2510, 0x1014},
    {0x2510, 0x1006},
    {0x2510, 0x0033},
    {0x2510, 0xC020},
    {0x2510, 0xB1E0},
    {0x2510, 0x0030},
    {0x2510, 0x0007},
    {0x2510, 0xAA5A},
    {0x2510, 0xA028},
    {0x2510, 0x8295},
    {0x2510, 0x100F},
    {0x2510, 0x0128},
    {0x2510, 0x1002},
    {0x2510, 0x02D6},
    {0x2510, 0xA620},
    {0x2510, 0x8891},
    {0x2510, 0x11BD},
    {0x2510, 0x1051},
    {0x2510, 0x0023},
    {0x2510, 0xCA36},
    {0x2510, 0x10A9},
    {0x2510, 0x99CD},
    {0x2510, 0x2004},
    {0x2510, 0x0C4E},
    {0x2510, 0x113B},
    {0x2510, 0xC000},
    {0x2510, 0x0342},
    {0x2510, 0x00C1},
    {0x2510, 0x10D5},
    {0x2510, 0x06C5},
    {0x2510, 0x13C2},
    {0x2510, 0x0295},
    {0x2510, 0x9008},
    {0x2510, 0x200E},
    {0x2510, 0x06CB},
    {0x2510, 0x000E},
    {0x2510, 0x1022},
    {0x2510, 0x1015},
    {0x2510, 0xA84B},
    {0x2510, 0x002C},
    {0x2510, 0x80B9},
    {0x2510, 0x104B},
    {0x2510, 0x1056},
    {0x2510, 0x0024},
    {0x2510, 0x9B0F},
    {0x2510, 0x05B5},
    {0x2510, 0x0612},
    {0x2510, 0x10AC},
    {0x2510, 0x009A},
    {0x2510, 0x3002},
    {0x2510, 0x2006},
    {0x2510, 0x0028},
    {0x2510, 0x8091},
    {0x2510, 0x1024},
    {0x2510, 0x0021},
    {0x2510, 0x002F},
    {0x2510, 0xCD26},
    {0x2510, 0x112D},
    {0x2510, 0x9000},
    {0x2510, 0x100E},
    {0x2510, 0x1A28},
    {0x2510, 0x122F},
    {0x2510, 0x11B5},
    {0x2510, 0x129D},
    {0x2510, 0x1CD9},
    {0x2510, 0x0015},
    {0x2510, 0x074B},
    {0x2510, 0x1021},
    {0x2510, 0x0020},
    {0x2510, 0x1095},
    {0x2510, 0x1030},
    {0x2510, 0x104B},
    {0x2510, 0xB106},
    {0x2510, 0xC490},
    {0x2510, 0xA882},
    {0x2510, 0x8255},
    {0x2510, 0xB100},
    {0x2510, 0xC802},
    {0x2510, 0x8801},
    {0x2510, 0x00A9},
    {0x2510, 0x0022},
    {0x2510, 0x0151},
    {0x2510, 0x0181},
    {0x2510, 0x98BB},
    {0x2510, 0x2007},
    {0x2510, 0x0D0A},
    {0x2510, 0x0036},
    {0x2510, 0x001D},
    {0x2510, 0x9B3F},
    {0x2510, 0x101B},
    {0x2510, 0x00B9},
    {0x2510, 0x0040},
    {0x2510, 0x101F},
    {0x2510, 0x104C},
    {0x2510, 0x1001},
    {0x2510, 0x3081},
    {0x2510, 0x002A},
    {0x2510, 0x108A},
    {0x2510, 0x3044},
    {0x2510, 0x1023},
    {0x2510, 0x102A},
    {0x2510, 0x2020},
    {0x2510, 0x121D},
    {0x2510, 0x04B8},
    {0x2510, 0x10B8},
    {0x2510, 0x1040},
    {0x2510, 0xB500},
    {0x2510, 0x01A3},
    {0x2510, 0x02AB},
    {0x2510, 0x004C},
    {0x2510, 0x30D0},
    {0x2510, 0x3141},
    {0x2510, 0x3041},
    {0x2510, 0x0044},
    {0x2510, 0x3142},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3110},
    {0x2510, 0x2000},
    {0x2510, 0x00B5},
    {0x2510, 0x996B},
    {0x2510, 0x3041},
    {0x2510, 0x1044},
    {0x2510, 0x3120},
    {0x2510, 0x3041},
    {0x2510, 0x1045},
    {0x2510, 0x3144},
    {0x2510, 0x3041},
    {0x2510, 0x987B},
    {0x2510, 0x3148},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3182},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x3184},
    {0x2510, 0x2000},
    {0x2510, 0x3190},
    {0x2510, 0x3041},
    {0x2510, 0x2000},
    {0x2510, 0x31A0},
    {0x2510, 0x3088},
    {0x2510, 0x2001},
    {0x2510, 0x807D},
    {0x2510, 0x2006},
    {0x2510, 0x8815},
    {0x2510, 0x8877},
    {0x2510, 0x0992},
    {0x2510, 0x2201},
    {0x2510, 0x2000},
    {0x2510, 0x2206},
    {0x2510, 0x2004},
    {0x2510, 0x8055},
    {0x2510, 0x3001},
    {0x2510, 0x2005},
    {0x2510, 0x8C61},
    {0x2510, 0x8801},
    {0x2510, 0x1012},
    {0x2510, 0x151F},
    {0x2510, 0x089F},
    {0x2510, 0x0057},
    {0x2510, 0x0036},
    {0x2510, 0x986E},
    {0x2510, 0x0023},
    {0x2510, 0x10CC},
    {0x2510, 0x00B9},
    {0x2510, 0x0040},
    {0x2510, 0x1035},
    {0x2510, 0x1321},
    {0x2510, 0x9826},
    {0x2510, 0x00C8},
    {0x2510, 0xC85B},
    {0x2510, 0x2008},
    {0x2510, 0x10C8},
    {0x2510, 0x002A},
    {0x2510, 0x3281},
    {0x2510, 0x2000},
    {0x2510, 0x102B},
    {0x2510, 0x3044},
    {0x2510, 0x2001},
    {0x2510, 0x102A},
    {0x2510, 0x09C9},
    {0x2510, 0x000C},
    {0x2510, 0x1149},
    {0x2510, 0x1157},
    {0x2510, 0x10D4},
    {0x2510, 0x1140},
    {0x2510, 0x1041},
    {0x2510, 0x9A25},
    {0x2510, 0xBAA0},
    {0x2510, 0xB06D},
    {0x2510, 0x004D},
    {0x2510, 0x1046},
    {0x2510, 0x1020},
    {0x2510, 0x0047},
    {0x2510, 0xB064},
    {0x2510, 0x1047},
    {0x2510, 0x0025},
    {0x2510, 0x99C5},
    {0x2510, 0xC810},
    {0x2510, 0x1025},
    {0x2510, 0x00B5},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},
    {0x2510, 0x7FFF},

    /* Char_Recommended_Settings_select(device) */
    {0x3514, 0x998F},
    {0x350E, 0x39EE},
    {0x3510, 0xEE99},
    {0x3516, 0xFFF4},
    {0x350A, 0x0055},
    {0x353E, 0x981C},
    {0x33E4, 0x80},
    {0x350C, 0x6648},
    {0x3512, 0x9999},

    /* Gain_Table_select(device) */
    {0x5914, 0x4001},  // SENSOR_GAIN_TABLE_SEL
    {0x5910, 0x6080},  // SENSOR_GAIN_REG1
    {0x5910, 0x5882},  // SENSOR_GAIN_REG1
    {0x5910, 0x688D},  // SENSOR_GAIN_REG1
    {0x5910, 0x7897},  // SENSOR_GAIN_REG1
    {0x5910, 0x989D},  // SENSOR_GAIN_REG1
    {0x5910, 0xA89A},  // SENSOR_GAIN_REG1
    {0x5910, 0xC886},  // SENSOR_GAIN_REG1
    {0x5910, 0xC8BD},  // SENSOR_GAIN_REG1
    {0x5910, 0xC90B},  // SENSOR_GAIN_REG1
    {0x5910, 0xC97A},  // SENSOR_GAIN_REG1
    {0x5910, 0xCA16},  // SENSOR_GAIN_REG1
    {0x5910, 0xCAF2},  // SENSOR_GAIN_REG1
    {0x5910, 0xCC29},  // SENSOR_GAIN_REG1
    {0x5910, 0xCDE0},  // SENSOR_GAIN_REG1
    {0x5910, 0x6080},  // SENSOR_GAIN_REG1
    {0x5910, 0xC8F0},  // SENSOR_GAIN_REG1
    {0x5910, 0xC953},  // SENSOR_GAIN_REG1
    {0x5910, 0xC9DF},  // SENSOR_GAIN_REG1
    {0x5910, 0x0006},  // SENSOR_GAIN_REG1
    {0x5910, 0x0001},  // SENSOR_GAIN_REG1
    {0x5910, 0x0002},  // SENSOR_GAIN_REG1
    {0x5910, 0x0003},  // SENSOR_GAIN_REG1
    {0x5910, 0x0005},  // SENSOR_GAIN_REG1
    {0x5910, 0x0004},  // SENSOR_GAIN_REG1
    {0x5910, 0x0001},  // SENSOR_GAIN_REG1
    {0x5910, 0x0003},  // SENSOR_GAIN_REG1
    {0x5910, 0x0004},  // SENSOR_GAIN_REG1
    {0x5910, 0x0004},  // SENSOR_GAIN_REG1
    {0x5910, 0x0007},  // SENSOR_GAIN_REG1
    {0x5910, 0x0006},  // SENSOR_GAIN_REG1
    {0x5910, 0x0000},  // SENSOR_GAIN_REG1
    {0x5910, 0x0003},  // SENSOR_GAIN_REG1
    {0x5910, 0x0002},  // SENSOR_GAIN_REG1
    {0x5910, 0x0006},  // SENSOR_GAIN_REG1
    {0x5910, 0x0005},  // SENSOR_GAIN_REG1
    {0x5910, 0x0001},  // SENSOR_GAIN_REG1
    {0x5910, 0x5940},  // SENSOR_GAIN_REG1
    {0x5910, 0x0000},  // SENSOR_GAIN_REG1
    {0x5910, 0x0001},  // SENSOR_GAIN_REG1
    {0x5910, 0x0002},  // SENSOR_GAIN_REG1
    {0x5910, 0x0003},  // SENSOR_GAIN_REG1
    {0x5910, 0x0004},  // SENSOR_GAIN_REG1
    {0x5910, 0x0005},  // SENSOR_GAIN_REG1
    {0x5910, 0x0006},  // SENSOR_GAIN_REG1
    {0x5910, 0x0007},  // SENSOR_GAIN_REG1
    {0x5910, 0x9941},  // SENSOR_GAIN_REG1
    {0x5910, 0x0010},  // SENSOR_GAIN_REG1
    {0x5910, 0x0011},  // SENSOR_GAIN_REG1
    {0x5910, 0x0012},  // SENSOR_GAIN_REG1
    {0x5910, 0x0013},  // SENSOR_GAIN_REG1
    {0x5910, 0x0014},  // SENSOR_GAIN_REG1
    {0x5910, 0x0015},  // SENSOR_GAIN_REG1
    {0x5910, 0x0016},  // SENSOR_GAIN_REG1
    {0x5910, 0x0017},  // SENSOR_GAIN_REG1
    {0x5910, 0x5942},  // SENSOR_GAIN_REG1
    {0x5910, 0x0020},  // SENSOR_GAIN_REG1
    {0x5910, 0x0021},  // SENSOR_GAIN_REG1
    {0x5910, 0x0022},  // SENSOR_GAIN_REG1
    {0x5910, 0x0023},  // SENSOR_GAIN_REG1
    {0x5910, 0x0024},  // SENSOR_GAIN_REG1
    {0x5910, 0x0025},  // SENSOR_GAIN_REG1
    {0x5910, 0x0026},  // SENSOR_GAIN_REG1
    {0x5910, 0x0027},  // SENSOR_GAIN_REG1
    {0x5910, 0xD943},  // SENSOR_GAIN_REG1
    {0x5910, 0x0030},  // SENSOR_GAIN_REG1
    {0x5910, 0x0031},  // SENSOR_GAIN_REG1
    {0x5910, 0x0032},  // SENSOR_GAIN_REG1
    {0x5910, 0x0033},  // SENSOR_GAIN_REG1
    {0x5910, 0x0034},  // SENSOR_GAIN_REG1
    {0x5910, 0x0035},  // SENSOR_GAIN_REG1
    {0x5910, 0x0036},  // SENSOR_GAIN_REG1
    {0x5910, 0x0037},  // SENSOR_GAIN_REG1
    {0x5910, 0x5944},  // SENSOR_GAIN_REG1
    {0x5910, 0x0040},  // SENSOR_GAIN_REG1
    {0x5910, 0x0041},  // SENSOR_GAIN_REG1
    {0x5910, 0x0042},  // SENSOR_GAIN_REG1
    {0x5910, 0x0043},  // SENSOR_GAIN_REG1
    {0x5910, 0x0044},  // SENSOR_GAIN_REG1
    {0x5910, 0x0045},  // SENSOR_GAIN_REG1
    {0x5910, 0x0046},  // SENSOR_GAIN_REG1
    {0x5910, 0x0047},  // SENSOR_GAIN_REG1
    {0x5910, 0xD945},  // SENSOR_GAIN_REG1
    {0x5910, 0x0050},  // SENSOR_GAIN_REG1
    {0x5910, 0x0051},  // SENSOR_GAIN_REG1
    {0x5910, 0x0052},  // SENSOR_GAIN_REG1
    {0x5910, 0x0053},  // SENSOR_GAIN_REG1
    {0x5910, 0x0054},  // SENSOR_GAIN_REG1
    {0x5910, 0x0055},  // SENSOR_GAIN_REG1
    {0x5910, 0x0056},  // SENSOR_GAIN_REG1
    {0x5910, 0x0057},  // SENSOR_GAIN_REG1
    {0x5910, 0xD946},  // SENSOR_GAIN_REG1
    {0x5910, 0x0060},  // SENSOR_GAIN_REG1
    {0x5910, 0x0061},  // SENSOR_GAIN_REG1
    {0x5910, 0x0062},  // SENSOR_GAIN_REG1
    {0x5910, 0x0063},  // SENSOR_GAIN_REG1
    {0x5910, 0x0064},  // SENSOR_GAIN_REG1
    {0x5910, 0x0065},  // SENSOR_GAIN_REG1
    {0x5910, 0x0066},  // SENSOR_GAIN_REG1
    {0x5910, 0x0067},  // SENSOR_GAIN_REG1
    {0x5910, 0x9947},  // SENSOR_GAIN_REG1
    {0x5910, 0x0070},  // SENSOR_GAIN_REG1
    {0x5910, 0x0071},  // SENSOR_GAIN_REG1
    {0x5910, 0x0072},  // SENSOR_GAIN_REG1
    {0x5910, 0x0073},  // SENSOR_GAIN_REG1
    {0x5910, 0x0074},  // SENSOR_GAIN_REG1
    {0x5910, 0x0075},  // SENSOR_GAIN_REG1
    {0x5910, 0x0076},  // SENSOR_GAIN_REG1
    {0x5910, 0x0077},  // SENSOR_GAIN_REG1

    /* DDC_HDR(device) */
    /* DDC_Noise_Model(device) */
    {0x5002, 0x0DC3},
    {0x51CC, 0x149},
    {0x51D8, 0x44D},
    {0x51CE, 0x700},
    {0x51D0, 0x8},
    {0x51D2, 0x10},
    {0x51D4, 0x18},
    {0x51D6, 0x20},
    {0x5202, 0x0DC3},
    {0x51EA, 0x149},
    {0x51FC, 0x44D},
    {0x51EC, 0x700},
    {0x51EE, 0x8},
    {0x51F0, 0x10},
    {0x51F2, 0x18},
    {0x51F4, 0x20},
    {0x5402, 0x0DC3},
    {0x5560, 0x149},  // T4_NOISE_GAIN_THRESHOLD0
    {0x556C, 0x44D},  // T4_NOISE_GAIN_THRESHOLD1
    {0x5562, 0x700},  // T4_NOISE_GAIN_THRESHOLD2
    {0x5564, 0x8},    // T4_NOISE_FLOOR0
    {0x5566, 0x10},   // T4_NOISE_FLOOR1
    {0x5568, 0x18},   // T4_NOISE_FLOOR2
    {0x556A, 0x20},   // T4_NOISE_FLOOR3

    /* DDC_FUSE_and_SINGLE(device) */
    {0x31E0, 0x0001},
    {0x5000, 0x1181},
    {0x5200, 0x1181},
    {0x5400, 0x1181},
    {0x50A2, 0x2},  // BMT0
    {0x50A4, 0xC},  // BMT1
    {0x50A6, 0x0F0F},
    {0x50A8, 0x0F0F},
    {0x50AA, 0x050F},
    {0x50AC, 0x0101},
    {0x50AE, 0x0101},
    {0x50B0, 0x0101},
    {0x50B2, 0x03FF},
    {0x50B4, 0x0F0F},
    {0x50B6, 0x090F},
    {0x50B8, 0x050F},

    /* eHDR_3exp_ALTM_Output_Settings(device) */
    {0x31AE, 0x202},   // 2_Lane Output_Settings
    {0x31AC, 0x140C},  // Data_Format RAW20:OUTPUT12 Output_Settings
    {0x31D0, 0x0},     // COMPAND EN Output_Settings
    {0x3082, 0x8},     // Op_Mode_Control Output_Settings
    {0x30BA, 0x13E},   // Num_exp_max=2, SET_PARK_ROW = 1, decomp_dither_en=1, dither_en=1,
                      // pix_lsb_truncation=1 Output_Settings
    {0x247A, 0x73},
    {0x3110, 0x0011},

    /* eHDR_3exp_1080p_30fps_Timing_Exposure(device)  #Timing */
    {0x3002, 76},
    {0x3006, 1019},
    {0x3004, 284},
    {0x3008, 1651},

    /* Binning */
    {0x30A2, 3},
    {0x30A6, 3},

    {0x3040, 0x3000},
    {0x3238, 0x44},  // Exposure Ratios Timing
    {0x300C, 4044},  // LLPCK Timing
    {0x300A, 919},   // FLL Timing
    {0x3012, 830},   // CIT1=1024 Timing
    {0x3212, 0x4D},  // CIT2=64 Timing
    {0x3216, 0x5},   // CIT3=4 Timing
    {0x5900, 0x0},   // Gain t1 2.2xLCG Timing
    {0x5902, 0x0},   // Gain t2 1.67xLCG Timing
    {0x5904, 0x0},

    /* ALTM_Recommended_Highlight(device) */
    /* ALTM_Common_Recommended_Settings(device) */
    {0x2400, 0x5802},
    {0x2402, 0x0002},
    {0x2438, 0x0000},
    {0x243A, 0x077F},
    {0x243C, 0x0000},
    {0x243E, 0x0437},
    {0x2456, 0x0010},
    {0x2458, 0x0020},
    {0x245A, 0xFFFF},
    {0x245C, 0x0800},
    {0x245E, 0x0004},
    {0x2460, 0x0040},
    {0x2462, 0x0080},
    {0x2464, 0x0},
    {0x2476, 0x0290},
    {0x2478, 0xFFFF},
    {0x2466, 0xA000},

    /* ALTM_Highlight_logamma(device) */
    {0x2410, 0x0018},
    {0x2412, 0x0024},

    /* ALTM_B2Y_Default_Filter(device) */
    {0x242A, 0x002C},
    {0x242C, 0x0002},
    {0x242E, 0x0004},
    {0x2430, 0x000A},
    {0x2432, 0x0003},
    {0x2434, 0x000F},

    /* ALTM_Sharpness_Tuning(device) */
    {0x2414, 0x0BA0},
    {0x2416, 0x0FA0},
    {0x2418, 0xC350},
    {0x241A, 0xFA00},
    {0x241C, 0x0005},
    {0x241E, 0x000A},
    {0x2420, 0x0032},
    {0x2422, 0x00A5},
    {0x2424, 0x00A5},
    {0x2426, 0x0001},

    /* ALTM_Sharpness_OFF(device) */
    {0x2428, 0x0000},

    /* Data_Pedestal_0(device) */
    {0x301E, 0x0000},
};

/**
  \fn           int32_t AR0246_Bulk_Write_Reg(const AR0246_REG AR0246_reg[],
                                              uint32_t total_num, uint32_t reg_size))
  \brief        write array of registers value to AR0246 Camera Sensor registers.
  \param[in]    AR0246_reg : AR0246 Camera Sensor Register Array Structure \ref AR0246_REG
  \param[in]    total_num   : total number of registers(size of array)
  \return       \ref execution_status
  */
static int32_t AR0246_Bulk_Write_Reg(const AR0246_REG AR0246_reg[], uint32_t total_num,
                                     uint32_t reg_size)
{
    uint32_t i   = 0;
    int32_t  ret = 0;

    for (i = 0; i < total_num; i++) {
        ret = AR0246_WRITE_REG(AR0246_reg[i].reg_addr, AR0246_reg[i].reg_value, reg_size);
        if (ret != ARM_DRIVER_OK) {
            return ret;
        }
    }

    return ARM_DRIVER_OK;
}

/**
  \fn           void AR0246_Sensor_Enable_Clk_Src(void)
  \brief        Enable AR0246 Camera Sensor external clock source configuration.
  \param[in]    none
  \return       none
  */
static void AR0246_Sensor_Enable_Clk_Src(void)
{
    set_cpi_pixel_clk(CPI_PIX_CLKSEL_400MZ, RTE_AR0246_CAMERA_SENSOR_CSI_CLK_SCR_DIV);
}

/**
  \fn           void AR0246_Sensor_Disable_Clk_Src(void)
  \brief        Disable AR0246 Camera Sensor external clock source configuration.
  \param[in]    none
  \return       none
  */
static void AR0246_Sensor_Disable_Clk_Src(void)
{
    clear_cpi_pixel_clk();
}

/**
  \fn           int32_t AR0246_Camera_Hard_Reseten(void)
  \brief        Hard Reset AR0246 Camera Sensor
  \param[in]    none
  \return       \ref execution_status
  */
static int32_t AR0246_Camera_Hard_Reseten(void)
{
    int32_t ret = 0;

    ret         = GPIO_Driver_CAM_RST->Initialize(RTE_AR0246_CAMERA_SENSOR_RESET_PIN_NO, NULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->PowerControl(RTE_AR0246_CAMERA_SENSOR_RESET_PIN_NO, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->SetDirection(RTE_AR0246_CAMERA_SENSOR_RESET_PIN_NO,
                                            GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->Initialize(RTE_AR0246_CAMERA_SENSOR_POWER_PIN_NO, NULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->PowerControl(RTE_AR0246_CAMERA_SENSOR_POWER_PIN_NO, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->SetDirection(RTE_AR0246_CAMERA_SENSOR_POWER_PIN_NO,
                                            GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->SetValue(RTE_AR0246_CAMERA_SENSOR_RESET_PIN_NO,
                                        GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    AR0246_DELAY_uSEC(2000);

    ret = GPIO_Driver_CAM_PWR->SetValue(RTE_AR0246_CAMERA_SENSOR_POWER_PIN_NO,
                                        GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    AR0246_DELAY_uSEC(1000);

    ret = GPIO_Driver_CAM_RST->SetValue(RTE_AR0246_CAMERA_SENSOR_RESET_PIN_NO,
                                        GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    AR0246_DELAY_uSEC(100000);

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t AR0246_Camera_Soft_Reseten(void)
  \brief        Software Reset AR0246 Camera Sensor
  \param[in]    none
  \return       \ref execution_status
  */
static int32_t AR0246_Camera_Soft_Reseten(void)
{
    int32_t ret = 0;

    ret         = AR0246_WRITE_REG(AR0246_RESET_REGISTER, 0x0001, 2);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* @Observation: more delay is required for Camera Sensor
     *               to setup after Soft Reset.
     */
    AR0246_DELAY_uSEC(10000);

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t AR0246_Init(void)
  \brief        Initialize AR0246 Camera Sensor
  this function will
  - initialize i2c using i3c instance
  - software reset AR0246 Camera Sensor
  - read AR0246 chip-id, proceed only it is correct.
  \return       \ref execution_status
  */
int32_t AR0246_Init(void)
{
    int32_t  ret      = 0;
    uint32_t rcv_data = 0;

    /*Enable camera sensor clock source config*/
    AR0246_Sensor_Enable_Clk_Src();

    /*camera sensor resten*/
    ret = AR0246_Camera_Hard_Reseten();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Initialize i2c using i3c driver instance depending on
     *  AR0246 Camera Sensor specific i2c configurations
     *   \ref AR0246_camera_sensor_i2c_cnfg
     */
    ret = camera_sensor_i2c_init(&AR0246_camera_sensor_i2c_cnfg);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Soft Reset AR0246 Camera Sensor */
    ret = AR0246_Camera_Soft_Reseten();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Read AR0246 Camera Sensor CHIP ID */
    ret = AR0246_READ_REG(AR0246_CHIP_ID_REGISTER, &rcv_data, 2);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Proceed only if CHIP ID is correct. */
    if (rcv_data != AR0246_CHIP_ID_REGISTER_VALUE) {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    /*start streaming*/
    ret = AR0246_READ_REG(AR0246_RESET_REGISTER, &rcv_data, 2);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = AR0246_WRITE_REG(AR0246_RESET_REGISTER, rcv_data | (1U << 2), 2);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t AR0246_Start(void)
  \brief        Start AR0246 Camera Sensor Streaming.
  \param[in]    none
  \return       \ref execution_status
  */
int32_t AR0246_Start(void)
{
    int32_t  ret      = 0;
    uint32_t rcv_data = 0;

    /* Start streaming */
    ret               = AR0246_READ_REG(AR0246_RESET_REGISTER, &rcv_data, 2);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return AR0246_WRITE_REG(AR0246_RESET_REGISTER, rcv_data | (1U << 2), 2);
}

/**
  \fn           int32_t AR0246_Stop(void)
  \brief        Stop AR0246 Camera Sensor Streaming.
  \param[in]    none
  \return       \ref execution_status
  */
int32_t AR0246_Stop(void)
{
    /* Suspend any stream */
    return AR0246_WRITE_REG(AR0246_RESET_REGISTER, 0x0000, 2);
}

/**
  \fn           int32_t AR0246_Control(uint32_t control, uint32_t arg)
  \brief        Control AR0246 Camera Sensor.
  \param[in]    control  : Operation
  \param[in]    arg      : Argument of operation
  \return       \ref execution_status
  */
int32_t AR0246_Control(uint32_t control, uint32_t arg)
{
    ARG_UNUSED(arg);

    switch (control) {
    case CPI_CAMERA_SENSOR_CONFIGURE:
        {
            uint32_t rcv_data;
            int32_t  ret = AR0246_READ_REG(AR0246_RESET_REGISTER, &rcv_data, 2);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }

            ret = AR0246_WRITE_REG(AR0246_RESET_REGISTER, rcv_data & ~(1U << 2), 2);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }

            uint32_t total_num =
                (sizeof(cfg_684x472_12bit_40fps_2Lane_sensor_reg_settings) / sizeof(AR0246_REG));
            return AR0246_Bulk_Write_Reg(cfg_684x472_12bit_40fps_2Lane_sensor_reg_settings,
                                         total_num,
                                         2);
        }
    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }
}

/**
  \fn           int32_t AR0246_Uninit(void)
  \brief        Un-initialize AR0246 Camera Sensor.
  \param[in]    none
  \return       \ref execution_status
  */
int32_t AR0246_Uninit(void)
{

    /*Disable camera sensor clock source config*/
    AR0246_Sensor_Disable_Clk_Src();

    return ARM_DRIVER_OK;
}

/**
\brief AR0246 Camera Sensor CSi informations
\ref CSI_INFO
*/
static CSI_INFO AR0246_csi_info = {
    .frequency                = RTE_AR0246_CAMERA_SENSOR_CSI_FREQ,
    .dt                       = RTE_AR0246_CAMERA_SENSOR_CSI_DATA_TYPE,
    .n_lanes                  = RTE_AR0246_CAMERA_SENSOR_CSI_N_LANES,
    .vc_id                    = RTE_AR0246_CAMERA_SENSOR_CSI_VC_ID,
    .cpi_cfg.override         = RTE_AR0246_CAMERA_SENSOR_OVERRIDE_CPI_COLOR_MODE,
    .cpi_cfg.cpi_color_mode   = RTE_AR0246_CAMERA_SENSOR_CPI_COLOR_MODE
};

/**
\brief AR0246 Camera Sensor Operations
\ref CAMERA_SENSOR_OPERATIONS
*/
static CAMERA_SENSOR_OPERATIONS AR0246_ops = {
    .Init    = AR0246_Init,
    .Uninit  = AR0246_Uninit,
    .Start   = AR0246_Start,
    .Stop    = AR0246_Stop,
    .Control = AR0246_Control,
};

/**
\brief AR0246 Camera Sensor Device Structure
\ref CAMERA_SENSOR_DEVICE
*/
static CAMERA_SENSOR_DEVICE AR0246_camera_sensor = {
    .interface = CAMERA_SENSOR_INTERFACE_MIPI,
    .width     = RTE_AR0246_CAMERA_SENSOR_FRAME_WIDTH,
    .height    = RTE_AR0246_CAMERA_SENSOR_FRAME_HEIGHT,
    .csi_info  = &AR0246_csi_info,
    .ops       = &AR0246_ops,
};

/* Registering CPI sensor */
CAMERA_SENSOR(AR0246_camera_sensor)

#endif /* RTE_AR0246_CAMERA_SENSOR_ENABLE */
