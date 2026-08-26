/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
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
#include "board_config.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "Camera_Sensor.h"
#include "Camera_Sensor_i2c.h"
#include "Driver_IO.h"
#include "Driver_CPI.h"
#include "sys_utils.h"
#include "sys_ctrl_cpi.h"

#if (RTE_OV5675_CAMERA_SENSOR_ENABLE)

/* Wrapper function for i2c read
 *  read register value from OV5675 Camera Sensor registers
 *   using i2c read API \ref camera_sensor_i2c_read
 *
 *  for OV5675 Camera Sensor specific i2c configurations
 *   see \ref OV5675_camera_sensor_i2c_cnfg
 */
#define OV5675_READ_REG(reg_addr, reg_value, reg_size)                                             \
    camera_sensor_i2c_read(&OV5675_camera_sensor_i2c_cnfg,                                         \
                           reg_addr,                                                               \
                           reg_value,                                                              \
                           (CAMERA_SENSOR_I2C_REG_SIZE) reg_size)

/* Wrapper function for i2c write
 *  write register value to OV5675 Camera Sensor registers
 *   using i2c write API \ref camera_sensor_i2c_write.
 *
 *  for OV5675 Camera Sensor specific i2c configurations
 *   see \ref OV5675_camera_sensor_i2c_cnfg
 */
#define OV5675_WRITE_REG(reg_addr, reg_value, reg_size)                                            \
    camera_sensor_i2c_write(&OV5675_camera_sensor_i2c_cnfg,                                        \
                            reg_addr,                                                              \
                            reg_value,                                                             \
                            (CAMERA_SENSOR_I2C_REG_SIZE) reg_size)

/* Supported I2C slave addresses for OV5675 sensor modules */
#define OV5675_CAMERA_SENSOR_SLAVE_ADDR_A 0x10
#define OV5675_CAMERA_SENSOR_SLAVE_ADDR_B 0x36

#define OV5675_CHIP_ID_REGISTER_VALUE   0x5675

#define OV5675_REG_CHIPID_H             0x300b
#define OV5675_REG_CHIPID_L             0x300c

#define OV5675_REG_MODE_SELECT          0x0100
#define OV5675_MODE_STANDBY             0x00
#define OV5675_MODE_STREAMING           0x01

/*Helper macro*/
#define ARRAY_SIZE(x)                   (sizeof(x) / sizeof((x)[0]))

/* ISP AE exposure registers (20-bit, stored in 1/16 row units) */
#define OV5675_COARSE_INTEGRATION_TIME_H    0x3500
#define OV5675_COARSE_INTEGRATION_TIME_M    0x3501
#define OV5675_COARSE_INTEGRATION_TIME_L    0x3502
#define OV5675_AEC_MANUAL_REG               0x3503
#define OV5675_AEC_MANUAL_VAL               0x08
/* ISP AE gain registers (Q7 format: 0x0080 = 1x, 0x0780 = 15x max) */
#define OV5675_GLOBAL_GAIN_H                0x3508
#define OV5675_GLOBAL_GAIN_L                0x3509

/**
 * @brief OV5675 Camera Sensor Register Array Structure
 *      used for Camera Configuration.
 */
typedef struct _OV5675_REG {
    uint16_t reg_addr;  /* OV5675 Camera Sensor Register Address*/
    uint16_t reg_value; /* OV5675 Camera Sensor Register Value*/
} OV5675_REG;

/* Wrapper function for Delay
 * Delay for microsecond:
 * Provide busy loop delay
 */
#define OV5675_DELAY_uSEC(usec) sys_busy_loop_us(usec)

/* OV5675 Camera power GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(BOARD_CAMERA_POWER_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_CAM_PWR =
    &ARM_Driver_GPIO_(BOARD_CAMERA_POWER_GPIO_PORT);

/* OV5675 Camera reset GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(BOARD_CAMERA_RESET_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_CAM_RST =
    &ARM_Driver_GPIO_(BOARD_CAMERA_RESET_GPIO_PORT);

/* I2C Driver Instance */
extern ARM_DRIVER_I2C ARM_Driver_I2C_(RTE_OV5675_CAMERA_SENSOR_I2C_INSTANCE);

/**
 * @brief OV5675 Camera Sensor slave i2c Configuration
 * @ref CAMERA_SENSOR_SLAVE_I2C_CONFIG
 */
static CAMERA_SENSOR_SLAVE_I2C_CONFIG OV5675_camera_sensor_i2c_cnfg = {
    .drv_i2c                        = &ARM_Driver_I2C_(RTE_OV5675_CAMERA_SENSOR_I2C_INSTANCE),
    .bus_speed                      = ARM_I2C_BUS_SPEED_STANDARD,
    .cam_sensor_slave_addr          = OV5675_CAMERA_SENSOR_SLAVE_ADDR_A,
    .cam_sensor_slave_reg_addr_type = CAMERA_SENSOR_I2C_REG_ADDR_TYPE_16BIT,
};

/* OV5675 Camera Sensor Resolution Selection
 * Controlled via RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG in RTE_Device.h
 *
 *   0 => 1296x972  (default, 2x2 binned, RAW10)
 *   1 => 1920x1080 (cropped, no binning, RAW10)
 *   2 => 1280x720  (2x2 binned, RAW10)
 *   3 => 640x480   (VGA, 4x4 binned, RAW10)
 */
#if (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG < 0) || \
    (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG > 3)
#error "RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG out of range (0-3)"
#endif

#if (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 0)
/**
 * @brief OV5675 Camera Sensor Resolution 1296x972 (2x2 binned, RAW10)
 *        MIPI data rate: 900 Mbps, 2 lanes
 */
static const OV5675_REG OV5675_1296x972_regs[] = {
    /* MIPI PLL / Clock config */
    {0x0103, 0x01}, /* SC_CTRL0103: software reset */
    {0x0100, 0x00}, /* SC_CTRL0100: streaming off (standby) */
    {0x0300, 0x04}, /* SC_CMMN_PLL_CTRL0: PLL pre-divider = 4 */
    {0x0302, 0x8d}, /* SC_CMMN_PLL_CTRL2: PLL multiplier = 141 */
    {0x0303, 0x00}, /* SC_CMMN_PLL_CTRL3: PLL divider = 1 */
    {0x030d, 0x26}, /* SC_CMMN_PLL_CTRL13: MIPI PLL divider */
    /* Sensor analog control */
    {0x3002, 0x21}, /* SC_CTRL3002: PSRAM reset / PLL bypass */
    {0x3107, 0x23}, /* SC_CTRL3107: MIPI bit clock select */
    {0x3501, 0x20}, /* AEC/AGC long exposure[15:8] */
    {0x3503, 0x0c}, /* AEC/AGC manual mode control */
    {0x3508, 0x03}, /* AEC/AGC long gain[9:8] */
    {0x3509, 0x00}, /* AEC/AGC long gain[7:0] */
    /* Analog control group */
    {0x3600, 0x66}, /* ANACTRL0: analog control */
    {0x3602, 0x30}, /* ANACTRL2: analog control */
    {0x3610, 0xa5}, /* ANACTRL10: analog control */
    {0x3612, 0x93}, /* ANACTRL12: analog control */
    {0x3620, 0x80}, /* ANACTRL20: analog control */
    {0x3642, 0x0e}, /* ANACTRL42: analog control */
    {0x3661, 0x00}, /* ANACTRL61: analog control */
    {0x3662, 0x08}, /* ANACTRL62: analog binning control (2x2) */
    {0x3664, 0xf3}, /* ANACTRL64: analog control */
    {0x3665, 0x9e}, /* ANACTRL65: analog control */
    {0x3667, 0xa5}, /* ANACTRL67: analog control */
    {0x366e, 0x55}, /* ANACTRL6e: analog control */
    {0x366f, 0x55}, /* ANACTRL6f: analog control */
    {0x3670, 0x11}, /* ANACTRL70: analog control */
    {0x3671, 0x11}, /* ANACTRL71: analog control */
    {0x3672, 0x11}, /* ANACTRL72: analog control */
    {0x3673, 0x11}, /* ANACTRL73: analog control */
    /* Timing control */
    {0x3714, 0x28}, /* SENCTRL14: sensor control */
    {0x371a, 0x3e}, /* SENCTRL1a: sensor control */
    {0x3733, 0x10}, /* SENCTRL33: sensor control */
    {0x3734, 0x00}, /* SENCTRL34: sensor control */
    {0x373d, 0x24}, /* SENCTRL3d: sensor control */
    {0x3764, 0x20}, /* SENCTRL64: sensor control */
    {0x3765, 0x20}, /* SENCTRL65: sensor control */
    {0x3766, 0x12}, /* SENCTRL66: sensor control */
    {0x37a1, 0x14}, /* SENCTRL_a1: sensor control */
    {0x37a8, 0x1c}, /* SENCTRL_a8: sensor control */
    {0x37ab, 0x0f}, /* SENCTRL_ab: sensor control */
    {0x37c2, 0x14}, /* SENCTRL_c2: PCLK period, 2x2 binning */
    {0x37cb, 0x00}, /* SENCTRL_cb: sensor control */
    {0x37cc, 0x00}, /* SENCTRL_cc: sensor control */
    {0x37cd, 0x00}, /* SENCTRL_cd: sensor control */
    {0x37ce, 0x00}, /* SENCTRL_ce: sensor control */
    {0x37d8, 0x02}, /* SENCTRL_d8: sensor control */
    {0x37d9, 0x04}, /* SENCTRL_d9: sensor control */
    {0x37dc, 0x04}, /* SENCTRL_dc: sensor control */
    /* Window / crop / output size */
    {0x3800, 0x00}, /* H_CROP_START[11:8] = 0 */
    {0x3801, 0x00}, /* H_CROP_START[7:0]  = 0 */
    {0x3802, 0x00}, /* V_CROP_START[11:8] = 0 */
    {0x3803, 0x00}, /* V_CROP_START[7:0]  = 0 */
    {0x3804, 0x0a}, /* H_CROP_END[11:8]   = 0x0a */
    {0x3805, 0x3f}, /* H_CROP_END[7:0]    = 0x3f => 2623 */
    {0x3806, 0x07}, /* V_CROP_END[11:8]   = 0x07 */
    {0x3807, 0xb7}, /* V_CROP_END[7:0]    = 0xb7 => 1975 */
    {0x3808, 0x05}, /* X_OUTPUT_SIZE[11:8] = 0x05 */
    {0x3809, 0x10}, /* X_OUTPUT_SIZE[7:0]  = 0x10 => 1296 */
    {0x380a, 0x03}, /* Y_OUTPUT_SIZE[11:8] = 0x03 */
    {0x380b, 0xcc}, /* Y_OUTPUT_SIZE[7:0]  = 0xcc => 972 */
    {0x380c, 0x02}, /* H_TOTAL_SIZE[11:8]  = 0x02 (HTS MSB) */
    {0x380d, 0xee}, /* H_TOTAL_SIZE[7:0]   = 0xee => HTS=750 */
    {0x380e, 0x07}, /* V_TOTAL_SIZE[11:8]  = 0x07 (VTS MSB) */
    {0x380f, 0xd0}, /* V_TOTAL_SIZE[7:0]   = 0xd0 => VTS=2000 */
    {0x3811, 0x08}, /* H_WIN_OFFSET[7:0]   = 8 */
    {0x3813, 0x0d}, /* V_WIN_OFFSET[7:0]   = 13 */
    {0x3814, 0x03}, /* H_ODD_INC  = 3 (2x binning) */
    {0x3815, 0x01}, /* H_EVEN_INC = 1 */
    {0x3816, 0x03}, /* V_ODD_INC  = 3 (2x binning) */
    {0x3817, 0x01}, /* V_EVEN_INC = 1 */
    {0x381e, 0x02}, /* TIMING_CTRL1e: frame ctrl */
    {0x3820, 0x8b}, /* TIMING_FORMAT1: flip/mirror, binning on */
    {0x3821, 0x01}, /* TIMING_FORMAT2: mirror on */
    {0x3832, 0x04}, /* TIMING_CTRL32: PCLK auto */
    /* Strobe / FREX */
    {0x3c80, 0x01}, /* STROBE_CTRL80: strobe control */
    {0x3c82, 0x00}, /* STROBE_CTRL82 */
    {0x3c83, 0xc8}, /* STROBE_CTRL83 */
    {0x3c8c, 0x0f}, /* STROBE_CTRL8c */
    {0x3c8d, 0xa0}, /* STROBE_CTRL8d */
    {0x3c90, 0x07}, /* STROBE_CTRL90 */
    {0x3c91, 0x00}, /* STROBE_CTRL91 */
    {0x3c92, 0x00}, /* STROBE_CTRL92 */
    {0x3c93, 0x00}, /* STROBE_CTRL93 */
    {0x3c94, 0xd0}, /* STROBE_CTRL94 */
    {0x3c95, 0x50}, /* STROBE_CTRL95 */
    {0x3c96, 0x35}, /* STROBE_CTRL96 */
    {0x3c97, 0x00}, /* STROBE_CTRL97 */
    /* BLC (Black Level Calibration) */
    {0x4001, 0xe0}, /* BLC_CTRL01: BLC enable */
    {0x4008, 0x00}, /* BLC_CTRL08: BLC start line */
    {0x4009, 0x07}, /* BLC_CTRL09: BLC end line */
    {0x400f, 0x80}, /* BLC_CTRL0f */
    {0x4013, 0x02}, /* BLC_CTRL13 */
    {0x4040, 0x00}, /* BLC_CTRL40: BLC anchor frame start[9:8] */
    {0x4041, 0x03}, /* BLC_CTRL41: BLC anchor frame end */
    {0x404c, 0x50}, /* BLC_CTRL4c */
    {0x404e, 0x20}, /* BLC_CTRL4e */
    /* VFIFO / format */
    {0x4500, 0x06}, /* VFIFO_CTRL00 */
    {0x4503, 0x00}, /* VFIFO_CTRL03 */
    {0x450a, 0x04}, /* VFIFO_CTRL0a */
    /* MIPI */
    {0x4809, 0x04}, /* MIPI_CTRL09: MIPI lane delay */
    {0x480c, 0x12}, /* MIPI_CTRL0c: HS settle time */
    {0x4819, 0x70}, /* MIPI_CTRL19: MIPI timing */
    {0x4825, 0x32}, /* MIPI_CTRL25: LP timing */
    {0x4826, 0x32}, /* MIPI_CTRL26: LP timing */
    {0x482a, 0x06}, /* MIPI_CTRL2a */
    {0x4833, 0x08}, /* MIPI_CTRL33 */
    {0x4837, 0x0d}, /* MIPI_PCLK_PERIOD: MIPI pixel clock period */
    /* ISP control */
    {0x5000, 0x77}, /* ISP_CTRL00: DPC/BLC/WB/Gamma enable */
    /* LSC (Lens Shading Correction) */
    {0x5b00, 0x01}, /* LSC_CTRL00 */
    {0x5b01, 0x10}, /* LSC_CTRL01 */
    {0x5b02, 0x01}, /* LSC_CTRL02 */
    {0x5b03, 0xdb}, /* LSC_CTRL03 */
    {0x5b05, 0x6c}, /* LSC_CTRL05 */
    {0x5e10, 0xfc}, /* CTRL5e10 */
    /* AEC/AGC fine tuning */
    {0x3500, 0x00}, /* AEC_LONG_EXPO[19:16] */
    {0x3501, 0x1F}, /* AEC_LONG_EXPO[15:8] */
    {0x3502, 0x20}, /* AEC_LONG_EXPO[7:0] */
    {0x3503, 0x08}, /* AEC manual mode */
    {0x3508, 0x04}, /* AEC_LONG_GAIN[9:8] */
    {0x3509, 0x00}, /* AEC_LONG_GAIN[7:0] */
    {0x3832, 0x48}, /* TIMING_CTRL32: updated */
    /* DPC (Defect Pixel Correction) */
    {0x5780, 0x3e}, /* DPC_CTRL00 */
    {0x5781, 0x0f}, /* DPC_CTRL01 */
    {0x5782, 0x44}, /* DPC_CTRL02 */
    {0x5783, 0x02}, /* DPC_CTRL03 */
    {0x5784, 0x01}, /* DPC_CTRL04 */
    {0x5785, 0x01}, /* DPC_CTRL05 */
    {0x5786, 0x00}, /* DPC_CTRL06 */
    {0x5787, 0x04}, /* DPC_CTRL07 */
    {0x5788, 0x02}, /* DPC_CTRL08 */
    {0x5789, 0x0f}, /* DPC_CTRL09 */
    {0x578a, 0xfd}, /* DPC_CTRL0a */
    {0x578b, 0xf5}, /* DPC_CTRL0b */
    {0x578c, 0xf5}, /* DPC_CTRL0c */
    {0x578d, 0x03}, /* DPC_CTRL0d */
    {0x578e, 0x08}, /* DPC_CTRL0e */
    {0x578f, 0x0c}, /* DPC_CTRL0f */
    {0x5790, 0x08}, /* DPC_CTRL10 */
    {0x5791, 0x06}, /* DPC_CTRL11 */
    {0x5792, 0x00}, /* DPC_CTRL12 */
    {0x5793, 0x52}, /* DPC_CTRL13 */
    {0x5794, 0xa3}, /* DPC_CTRL14 */
    {0x4003, 0x40}, /* BLC_CTRL03: BLC redo trigger */
    {0x3107, 0x01}, /* SC_CTRL3107: updated */
    /* Strobe/FREX final settings */
    {0x3c80, 0x08}, /* STROBE_CTRL80: updated */
    {0x3c83, 0xb1}, /* STROBE_CTRL83: updated */
    {0x3c8c, 0x10}, /* STROBE_CTRL8c: updated */
    {0x3c8d, 0x00}, /* STROBE_CTRL8d: updated */
    {0x3c90, 0x00}, /* STROBE_CTRL90: updated */
    {0x3c94, 0x00}, /* STROBE_CTRL94: updated */
    {0x3c95, 0x00}, /* STROBE_CTRL95: updated */
    {0x3c96, 0x00}, /* STROBE_CTRL96: updated */
    {0x37cb, 0x09}, /* SENCTRL_cb: updated */
    {0x37cc, 0x15}, /* SENCTRL_cc: updated */
    {0x37cd, 0x1f}, /* SENCTRL_cd: updated */
    {0x37ce, 0x1f}, /* SENCTRL_ce: updated */
};
#endif /* RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 0 */

#if (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 1)
/**
 * @brief OV5675 Camera Sensor Resolution 1920x1080 (cropped, RAW10)
 *        MIPI data rate: 900 Mbps, 2 lanes
 */
static const OV5675_REG OV5675_1920x1080_regs[] = {
    /* MIPI PLL / Clock config */
    {0x0103, 0x01}, /* SC_CTRL0103: software reset */
    {0x0100, 0x00}, /* SC_CTRL0100: streaming off (standby) */
    {0x0300, 0x04}, /* SC_CMMN_PLL_CTRL0: PLL pre-divider = 4 */
    {0x0302, 0x8d}, /* SC_CMMN_PLL_CTRL2: PLL multiplier = 141 */
    {0x0303, 0x00}, /* SC_CMMN_PLL_CTRL3: PLL divider = 1 */
    {0x030d, 0x26}, /* SC_CMMN_PLL_CTRL13: MIPI PLL divider */
    /* Sensor analog control */
    {0x3002, 0x21}, /* SC_CTRL3002 */
    {0x3107, 0x23}, /* SC_CTRL3107: MIPI bit clock select */
    {0x3501, 0x20}, /* AEC/AGC long exposure[15:8] */
    {0x3503, 0x0c}, /* AEC/AGC manual mode control */
    {0x3508, 0x03}, /* AEC/AGC long gain[9:8] */
    {0x3509, 0x00}, /* AEC/AGC long gain[7:0] */
    /* Analog control group */
    {0x3600, 0x66}, /* ANACTRL0 */
    {0x3602, 0x30}, /* ANACTRL2 */
    {0x3610, 0xa5}, /* ANACTRL10 */
    {0x3612, 0x93}, /* ANACTRL12 */
    {0x3620, 0x80}, /* ANACTRL20 */
    {0x3642, 0x0e}, /* ANACTRL42 */
    {0x3661, 0x00}, /* ANACTRL61 */
    {0x3662, 0x10}, /* ANACTRL62: no binning for 1080p */
    {0x3664, 0xf3}, /* ANACTRL64 */
    {0x3665, 0x9e}, /* ANACTRL65 */
    {0x3667, 0xa5}, /* ANACTRL67 */
    {0x366e, 0x55}, /* ANACTRL6e */
    {0x366f, 0x55}, /* ANACTRL6f */
    {0x3670, 0x11}, /* ANACTRL70 */
    {0x3671, 0x11}, /* ANACTRL71 */
    {0x3672, 0x11}, /* ANACTRL72 */
    {0x3673, 0x11}, /* ANACTRL73 */
    /* Timing control */
    {0x3714, 0x24}, /* SENCTRL14: full resolution setting */
    {0x371a, 0x3e}, /* SENCTRL1a */
    {0x3733, 0x10}, /* SENCTRL33 */
    {0x3734, 0x00}, /* SENCTRL34 */
    {0x373d, 0x24}, /* SENCTRL3d */
    {0x3764, 0x20}, /* SENCTRL64 */
    {0x3765, 0x20}, /* SENCTRL65 */
    {0x3766, 0x12}, /* SENCTRL66 */
    {0x37a1, 0x14}, /* SENCTRL_a1 */
    {0x37a8, 0x1c}, /* SENCTRL_a8 */
    {0x37ab, 0x0f}, /* SENCTRL_ab */
    {0x37c2, 0x04}, /* SENCTRL_c2: no binning */
    {0x37cb, 0x00}, /* SENCTRL_cb */
    {0x37cc, 0x00}, /* SENCTRL_cc */
    {0x37cd, 0x00}, /* SENCTRL_cd */
    {0x37ce, 0x00}, /* SENCTRL_ce */
    {0x37d8, 0x02}, /* SENCTRL_d8 */
    {0x37d9, 0x08}, /* SENCTRL_d9: updated for 1080p */
    {0x37dc, 0x04}, /* SENCTRL_dc */
    /* Window / crop / output size - 1920x1080 crop from 2624x1976 */
    {0x3800, 0x00}, /* H_CROP_START[11:8] = 0 */
    {0x3801, 0x00}, /* H_CROP_START[7:0]  = 0 */
    {0x3802, 0x00}, /* V_CROP_START[11:8] = 0 */
    {0x3803, 0x04}, /* V_CROP_START[7:0]  = 4 */
    {0x3804, 0x0a}, /* H_CROP_END[11:8]   = 0x0a */
    {0x3805, 0x3f}, /* H_CROP_END[7:0]    = 0x3f => 2623 */
    {0x3806, 0x07}, /* V_CROP_END[11:8]   = 0x07 */
    {0x3807, 0xb3}, /* V_CROP_END[7:0]    = 0xb3 => 1971 */
    {0x3808, 0x07}, /* X_OUTPUT_SIZE[11:8] = 0x07 */
    {0x3809, 0x80}, /* X_OUTPUT_SIZE[7:0]  = 0x80 => 1920 */
    {0x380a, 0x04}, /* Y_OUTPUT_SIZE[11:8] = 0x04 */
    {0x380b, 0x38}, /* Y_OUTPUT_SIZE[7:0]  = 0x38 => 1080 */
    {0x380c, 0x02}, /* H_TOTAL_SIZE[11:8]  = 0x02 (HTS MSB) */
    {0x380d, 0xee}, /* H_TOTAL_SIZE[7:0]   = 0xee => HTS=750 */
    {0x380e, 0x07}, /* V_TOTAL_SIZE[11:8]  = 0x07 (VTS MSB) */
    {0x380f, 0xe4}, /* V_TOTAL_SIZE[7:0]   = 0xe4 => VTS=2020 */
    {0x3811, 0x10}, /* H_WIN_OFFSET[7:0]   = 16 */
    {0x3813, 0x0d}, /* V_WIN_OFFSET[7:0]   = 13 */
    {0x3814, 0x01}, /* H_ODD_INC  = 1 (no binning) */
    {0x3815, 0x01}, /* H_EVEN_INC = 1 */
    {0x3816, 0x01}, /* V_ODD_INC  = 1 (no binning) */
    {0x3817, 0x01}, /* V_EVEN_INC = 1 */
    {0x381e, 0x02}, /* TIMING_CTRL1e */
    {0x3820, 0x88}, /* TIMING_FORMAT1: no binning, flip off */
    {0x3821, 0x01}, /* TIMING_FORMAT2: mirror on */
    {0x3832, 0x04}, /* TIMING_CTRL32 */
    /* Strobe / FREX */
    {0x3c80, 0x01}, /* STROBE_CTRL80 */
    {0x3c82, 0x00}, /* STROBE_CTRL82 */
    {0x3c83, 0xc8}, /* STROBE_CTRL83 */
    {0x3c8c, 0x0f}, /* STROBE_CTRL8c */
    {0x3c8d, 0xa0}, /* STROBE_CTRL8d */
    {0x3c90, 0x07}, /* STROBE_CTRL90 */
    {0x3c91, 0x00}, /* STROBE_CTRL91 */
    {0x3c92, 0x00}, /* STROBE_CTRL92 */
    {0x3c93, 0x00}, /* STROBE_CTRL93 */
    {0x3c94, 0xd0}, /* STROBE_CTRL94 */
    {0x3c95, 0x50}, /* STROBE_CTRL95 */
    {0x3c96, 0x35}, /* STROBE_CTRL96 */
    {0x3c97, 0x00}, /* STROBE_CTRL97 */
    /* BLC */
    {0x4001, 0xe0}, /* BLC_CTRL01 */
    {0x4008, 0x02}, /* BLC_CTRL08: BLC start line (full res) */
    {0x4009, 0x0d}, /* BLC_CTRL09: BLC end line */
    {0x400f, 0x80}, /* BLC_CTRL0f */
    {0x4013, 0x02}, /* BLC_CTRL13 */
    {0x4040, 0x00}, /* BLC_CTRL40 */
    {0x4041, 0x07}, /* BLC_CTRL41: updated for full res */
    {0x404c, 0x50}, /* BLC_CTRL4c */
    {0x404e, 0x20}, /* BLC_CTRL4e */
    /* VFIFO / format */
    {0x4500, 0x06}, /* VFIFO_CTRL00 */
    {0x4503, 0x00}, /* VFIFO_CTRL03 */
    {0x450a, 0x04}, /* VFIFO_CTRL0a */
    /* MIPI */
    {0x4809, 0x04}, /* MIPI_CTRL09 */
    {0x480c, 0x12}, /* MIPI_CTRL0c */
    {0x4819, 0x70}, /* MIPI_CTRL19 */
    {0x4825, 0x32}, /* MIPI_CTRL25 */
    {0x4826, 0x32}, /* MIPI_CTRL26 */
    {0x482a, 0x06}, /* MIPI_CTRL2a */
    {0x4833, 0x08}, /* MIPI_CTRL33 */
    {0x4837, 0x0d}, /* MIPI_PCLK_PERIOD */
    /* ISP */
    {0x5000, 0x77}, /* ISP_CTRL00 */
    /* LSC */
    {0x5b00, 0x01}, /* LSC_CTRL00 */
    {0x5b01, 0x10}, /* LSC_CTRL01 */
    {0x5b02, 0x01}, /* LSC_CTRL02 */
    {0x5b03, 0xdb}, /* LSC_CTRL03 */
    {0x5b05, 0x6c}, /* LSC_CTRL05 */
    {0x5e10, 0xfc}, /* CTRL5e10 */
    /* AEC/AGC fine tuning */
    {0x3500, 0x00}, /* AEC_LONG_EXPO[19:16] */
    {0x3501, 0x3e}, /* AEC_LONG_EXPO[15:8] */
    {0x3502, 0x60}, /* AEC_LONG_EXPO[7:0] */
    {0x3503, 0x08}, /* AEC manual mode */
    {0x3508, 0x04}, /* AEC_LONG_GAIN[9:8] */
    {0x3509, 0x00}, /* AEC_LONG_GAIN[7:0] */
    {0x3832, 0x48}, /* TIMING_CTRL32: updated */
    /* DPC */
    {0x5780, 0x3e}, /* DPC_CTRL00 */
    {0x5781, 0x0f}, /* DPC_CTRL01 */
    {0x5782, 0x44}, /* DPC_CTRL02 */
    {0x5783, 0x02}, /* DPC_CTRL03 */
    {0x5784, 0x01}, /* DPC_CTRL04 */
    {0x5785, 0x01}, /* DPC_CTRL05 */
    {0x5786, 0x00}, /* DPC_CTRL06 */
    {0x5787, 0x04}, /* DPC_CTRL07 */
    {0x5788, 0x02}, /* DPC_CTRL08 */
    {0x5789, 0x0f}, /* DPC_CTRL09 */
    {0x578a, 0xfd}, /* DPC_CTRL0a */
    {0x578b, 0xf5}, /* DPC_CTRL0b */
    {0x578c, 0xf5}, /* DPC_CTRL0c */
    {0x578d, 0x03}, /* DPC_CTRL0d */
    {0x578e, 0x08}, /* DPC_CTRL0e */
    {0x578f, 0x0c}, /* DPC_CTRL0f */
    {0x5790, 0x08}, /* DPC_CTRL10 */
    {0x5791, 0x06}, /* DPC_CTRL11 */
    {0x5792, 0x00}, /* DPC_CTRL12 */
    {0x5793, 0x52}, /* DPC_CTRL13 */
    {0x5794, 0xa3}, /* DPC_CTRL14 */
    {0x4003, 0x40}, /* BLC_CTRL03: BLC redo trigger */
    {0x3107, 0x01}, /* SC_CTRL3107: updated */
    /* Strobe/FREX final settings */
    {0x3c80, 0x08}, /* STROBE_CTRL80: updated */
    {0x3c83, 0xb1}, /* STROBE_CTRL83: updated */
    {0x3c8c, 0x10}, /* STROBE_CTRL8c: updated */
    {0x3c8d, 0x00}, /* STROBE_CTRL8d: updated */
    {0x3c90, 0x00}, /* STROBE_CTRL90: updated */
    {0x3c94, 0x00}, /* STROBE_CTRL94: updated */
    {0x3c95, 0x00}, /* STROBE_CTRL95: updated */
    {0x3c96, 0x00}, /* STROBE_CTRL96: updated */
    {0x37cb, 0x09}, /* SENCTRL_cb: updated */
    {0x37cc, 0x15}, /* SENCTRL_cc: updated */
    {0x37cd, 0x1f}, /* SENCTRL_cd: updated */
    {0x37ce, 0x1f}, /* SENCTRL_ce: updated */
};
#endif /* RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 1 */

#if (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 2)
/**
 * @brief OV5675 Camera Sensor Resolution 1280x720 (2x2 binned, 16:9 crop, RAW10)
 *        MIPI data rate: 900 Mbps, 2 lanes
 *        2x2 binning over 2560x1440 window => 1280x720 output, wide FOV
 */
static const OV5675_REG OV5675_1280x720_regs[] = {
    /* MIPI PLL / Clock config */
    {0x0103, 0x01}, /* SC_CTRL0103: software reset */
    {0x0100, 0x00}, /* SC_CTRL0100: streaming off (standby) */
    {0x0300, 0x04}, /* SC_CMMN_PLL_CTRL0: PLL pre-divider = 4 */
    {0x0302, 0x8d}, /* SC_CMMN_PLL_CTRL2: PLL multiplier = 141 */
    {0x0303, 0x00}, /* SC_CMMN_PLL_CTRL3: PLL divider = 1 */
    {0x030d, 0x26}, /* SC_CMMN_PLL_CTRL13: MIPI PLL divider */
    /* Sensor analog control */
    {0x3002, 0x21}, /* SC_CTRL3002 */
    {0x3107, 0x23}, /* SC_CTRL3107: MIPI bit clock select */
    {0x3501, 0x20}, /* AEC/AGC long exposure[15:8] */
    {0x3503, 0x0c}, /* AEC/AGC manual mode control */
    {0x3508, 0x03}, /* AEC/AGC long gain[9:8] */
    {0x3509, 0x00}, /* AEC/AGC long gain[7:0] */
    /* Analog control group */
    {0x3600, 0x66}, /* ANACTRL0 */
    {0x3602, 0x30}, /* ANACTRL2 */
    {0x3610, 0xa5}, /* ANACTRL10 */
    {0x3612, 0x93}, /* ANACTRL12 */
    {0x3620, 0x80}, /* ANACTRL20 */
    {0x3642, 0x0e}, /* ANACTRL42 */
    {0x3661, 0x00}, /* ANACTRL61 */
    {0x3662, 0x08}, /* ANACTRL62: analog binning control (2x2) */
    {0x3664, 0xf3}, /* ANACTRL64 */
    {0x3665, 0x9e}, /* ANACTRL65 */
    {0x3667, 0xa5}, /* ANACTRL67 */
    {0x366e, 0x55}, /* ANACTRL6e */
    {0x366f, 0x55}, /* ANACTRL6f */
    {0x3670, 0x11}, /* ANACTRL70 */
    {0x3671, 0x11}, /* ANACTRL71 */
    {0x3672, 0x11}, /* ANACTRL72 */
    {0x3673, 0x11}, /* ANACTRL73 */
    /* Timing control */
    {0x3714, 0x24}, /* SENCTRL14: full resolution setting */
    {0x371a, 0x3e}, /* SENCTRL1a */
    {0x3733, 0x10}, /* SENCTRL33 */
    {0x3734, 0x00}, /* SENCTRL34 */
    {0x373d, 0x24}, /* SENCTRL3d */
    {0x3764, 0x20}, /* SENCTRL64 */
    {0x3765, 0x20}, /* SENCTRL65 */
    {0x3766, 0x12}, /* SENCTRL66 */
    {0x37a1, 0x14}, /* SENCTRL_a1 */
    {0x37a8, 0x1c}, /* SENCTRL_a8 */
    {0x37ab, 0x0f}, /* SENCTRL_ab */
    {0x37c2, 0x14}, /* SENCTRL_c2: 2x2 binning */
    {0x37cb, 0x00}, /* SENCTRL_cb */
    {0x37cc, 0x00}, /* SENCTRL_cc */
    {0x37cd, 0x00}, /* SENCTRL_cd */
    {0x37ce, 0x00}, /* SENCTRL_ce */
    {0x37d8, 0x02}, /* SENCTRL_d8 */
    {0x37d9, 0x04}, /* SENCTRL_d9 (2x binning, same as 1296x972) */
    {0x37dc, 0x04}, /* SENCTRL_dc */
    /* Window / crop / output size - 1280x720 (2x2 binned)
     * Same full crop window as 1296x972 to guarantee valid VSYNC timing.
     * 2x2 binning reduces 2624x1976 => ~1312x988, ISP crops to 1280x720.
     */
    {0x3800, 0x00}, /* H_CROP_START[11:8] = 0 */
    {0x3801, 0x00}, /* H_CROP_START[7:0]  = 0 */
    {0x3802, 0x00}, /* V_CROP_START[11:8] = 0 */
    {0x3803, 0x00}, /* V_CROP_START[7:0]  = 0 */
    {0x3804, 0x0a}, /* H_CROP_END[11:8]   = 0x0a */
    {0x3805, 0x3f}, /* H_CROP_END[7:0]    = 0x3f => 2623 */
    {0x3806, 0x07}, /* V_CROP_END[11:8]   = 0x07 */
    {0x3807, 0xb7}, /* V_CROP_END[7:0]    = 0xb7 => 1975 (same as 1296x972) */
    {0x3808, 0x05}, /* X_OUTPUT_SIZE[11:8] = 0x05 */
    {0x3809, 0x00}, /* X_OUTPUT_SIZE[7:0]  = 0x00 => 1280 */
    {0x380a, 0x02}, /* Y_OUTPUT_SIZE[11:8] = 0x02 */
    {0x380b, 0xd0}, /* Y_OUTPUT_SIZE[7:0]  = 0xd0 => 720 */
    {0x380c, 0x02}, /* H_TOTAL_SIZE[11:8]  = 0x02 */
    {0x380d, 0xee}, /* H_TOTAL_SIZE[7:0]   = 0xee => HTS=750 */
    {0x380e, 0x07}, /* V_TOTAL_SIZE[11:8]  = 0x07 */
    {0x380f, 0xd0}, /* V_TOTAL_SIZE[7:0]   = 0xd0 => VTS=2000 (same as 1296x972) */
    {0x3811, 0x08}, /* H_WIN_OFFSET[7:0]   = 8 (same as 1296x972) */
    {0x3813, 0x0d}, /* V_WIN_OFFSET[7:0]   = 13 (same as 1296x972) */
    {0x3814, 0x03}, /* H_ODD_INC  = 3 (2x binning) */
    {0x3815, 0x01}, /* H_EVEN_INC = 1 */
    {0x3816, 0x03}, /* V_ODD_INC  = 3 (2x binning) */
    {0x3817, 0x01}, /* V_EVEN_INC = 1 */
    {0x381e, 0x02}, /* TIMING_CTRL1e */
    {0x3820, 0x8b}, /* TIMING_FORMAT1: binning on (same as 1296x972) */
    {0x3821, 0x01}, /* TIMING_FORMAT2: mirror on */
    {0x3832, 0x04}, /* TIMING_CTRL32 */
    /* Strobe / FREX */
    {0x3c80, 0x01}, /* STROBE_CTRL80 */
    {0x3c82, 0x00}, /* STROBE_CTRL82 */
    {0x3c83, 0xc8}, /* STROBE_CTRL83 */
    {0x3c8c, 0x0f}, /* STROBE_CTRL8c */
    {0x3c8d, 0xa0}, /* STROBE_CTRL8d */
    {0x3c90, 0x07}, /* STROBE_CTRL90 */
    {0x3c91, 0x00}, /* STROBE_CTRL91 */
    {0x3c92, 0x00}, /* STROBE_CTRL92 */
    {0x3c93, 0x00}, /* STROBE_CTRL93 */
    {0x3c94, 0xd0}, /* STROBE_CTRL94 */
    {0x3c95, 0x50}, /* STROBE_CTRL95 */
    {0x3c96, 0x35}, /* STROBE_CTRL96 */
    {0x3c97, 0x00}, /* STROBE_CTRL97 */
    /* BLC */
    {0x4001, 0xe0}, /* BLC_CTRL01 */
    {0x4008, 0x02}, /* BLC_CTRL08: BLC start line */
    {0x4009, 0x0d}, /* BLC_CTRL09: BLC end line */
    {0x400f, 0x80}, /* BLC_CTRL0f */
    {0x4013, 0x02}, /* BLC_CTRL13 */
    {0x4040, 0x00}, /* BLC_CTRL40 */
    {0x4041, 0x07}, /* BLC_CTRL41 */
    {0x404c, 0x50}, /* BLC_CTRL4c */
    {0x404e, 0x20}, /* BLC_CTRL4e */
    /* VFIFO / format */
    {0x4500, 0x06}, /* VFIFO_CTRL00 */
    {0x4503, 0x00}, /* VFIFO_CTRL03 */
    {0x450a, 0x04}, /* VFIFO_CTRL0a */
    /* MIPI */
    {0x4809, 0x04}, /* MIPI_CTRL09 */
    {0x480c, 0x12}, /* MIPI_CTRL0c */
    {0x4819, 0x70}, /* MIPI_CTRL19 */
    {0x4825, 0x32}, /* MIPI_CTRL25 */
    {0x4826, 0x32}, /* MIPI_CTRL26 */
    {0x482a, 0x06}, /* MIPI_CTRL2a */
    {0x4833, 0x08}, /* MIPI_CTRL33 */
    {0x4837, 0x0d}, /* MIPI_PCLK_PERIOD */
    /* ISP */
    {0x5000, 0x77}, /* ISP_CTRL00 */
    /* LSC */
    {0x5b00, 0x01}, /* LSC_CTRL00 */
    {0x5b01, 0x10}, /* LSC_CTRL01 */
    {0x5b02, 0x01}, /* LSC_CTRL02 */
    {0x5b03, 0xdb}, /* LSC_CTRL03 */
    {0x5b05, 0x6c}, /* LSC_CTRL05 */
    {0x5e10, 0xfc}, /* CTRL5e10 */
    /* AEC/AGC fine tuning */
    {0x3500, 0x00}, /* AEC_LONG_EXPO[19:16] */
    {0x3501, 0x2c}, /* AEC_LONG_EXPO[15:8] */
    {0x3502, 0x00}, /* AEC_LONG_EXPO[7:0] */
    {0x3503, 0x08}, /* AEC manual mode */
    {0x3508, 0x04}, /* AEC_LONG_GAIN[9:8] */
    {0x3509, 0x00}, /* AEC_LONG_GAIN[7:0] */
    {0x3832, 0x48}, /* TIMING_CTRL32: updated */
    /* DPC */
    {0x5780, 0x3e}, /* DPC_CTRL00 */
    {0x5781, 0x0f}, /* DPC_CTRL01 */
    {0x5782, 0x44}, /* DPC_CTRL02 */
    {0x5783, 0x02}, /* DPC_CTRL03 */
    {0x5784, 0x01}, /* DPC_CTRL04 */
    {0x5785, 0x01}, /* DPC_CTRL05 */
    {0x5786, 0x00}, /* DPC_CTRL06 */
    {0x5787, 0x04}, /* DPC_CTRL07 */
    {0x5788, 0x02}, /* DPC_CTRL08 */
    {0x5789, 0x0f}, /* DPC_CTRL09 */
    {0x578a, 0xfd}, /* DPC_CTRL0a */
    {0x578b, 0xf5}, /* DPC_CTRL0b */
    {0x578c, 0xf5}, /* DPC_CTRL0c */
    {0x578d, 0x03}, /* DPC_CTRL0d */
    {0x578e, 0x08}, /* DPC_CTRL0e */
    {0x578f, 0x0c}, /* DPC_CTRL0f */
    {0x5790, 0x08}, /* DPC_CTRL10 */
    {0x5791, 0x06}, /* DPC_CTRL11 */
    {0x5792, 0x00}, /* DPC_CTRL12 */
    {0x5793, 0x52}, /* DPC_CTRL13 */
    {0x5794, 0xa3}, /* DPC_CTRL14 */
    {0x4003, 0x40}, /* BLC_CTRL03: BLC redo trigger */
    {0x3107, 0x01}, /* SC_CTRL3107: updated */
    /* Strobe/FREX final settings */
    {0x3c80, 0x08}, /* STROBE_CTRL80: updated */
    {0x3c83, 0xb1}, /* STROBE_CTRL83: updated */
    {0x3c8c, 0x10}, /* STROBE_CTRL8c: updated */
    {0x3c8d, 0x00}, /* STROBE_CTRL8d: updated */
    {0x3c90, 0x00}, /* STROBE_CTRL90: updated */
    {0x3c94, 0x00}, /* STROBE_CTRL94: updated */
    {0x3c95, 0x00}, /* STROBE_CTRL95: updated */
    {0x3c96, 0x00}, /* STROBE_CTRL96: updated */
    {0x37cb, 0x09}, /* SENCTRL_cb: updated */
    {0x37cc, 0x15}, /* SENCTRL_cc: updated */
    {0x37cd, 0x1f}, /* SENCTRL_cd: updated */
    {0x37ce, 0x1f}, /* SENCTRL_ce: updated */
};
#endif /* RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 2 */

#if (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 3)
/**
 * @brief OV5675 Camera Sensor Resolution 640x480 (VGA, 4x4 binned, RAW10)
 *        MIPI data rate: 900 Mbps, 2 lanes
 */
static const OV5675_REG OV5675_640x480_regs[] = {
    /* MIPI PLL / Clock config */
    {0x0103, 0x01}, /* SC_CTRL0103: software reset */
    {0x0100, 0x00}, /* SC_CTRL0100: streaming off (standby) */
    {0x0300, 0x04}, /* SC_CMMN_PLL_CTRL0: PLL pre-divider = 4 */
    {0x0302, 0x8d}, /* SC_CMMN_PLL_CTRL2: PLL multiplier = 141 */
    {0x0303, 0x00}, /* SC_CMMN_PLL_CTRL3: PLL divider = 1 */
    {0x030d, 0x26}, /* SC_CMMN_PLL_CTRL13: MIPI PLL divider */
    /* Sensor analog control */
    {0x3002, 0x21}, /* SC_CTRL3002 */
    {0x3107, 0x23}, /* SC_CTRL3107: MIPI bit clock select */
    {0x3501, 0x20}, /* AEC/AGC long exposure[15:8] */
    {0x3503, 0x0c}, /* AEC/AGC manual mode control */
    {0x3508, 0x03}, /* AEC/AGC long gain[9:8] */
    {0x3509, 0x00}, /* AEC/AGC long gain[7:0] */
    /* Analog control group */
    {0x3600, 0x66}, /* ANACTRL0 */
    {0x3602, 0x30}, /* ANACTRL2 */
    {0x3610, 0xa5}, /* ANACTRL10 */
    {0x3612, 0x93}, /* ANACTRL12 */
    {0x3620, 0x80}, /* ANACTRL20 */
    {0x3642, 0x0e}, /* ANACTRL42 */
    {0x3661, 0x00}, /* ANACTRL61 */
    {0x3662, 0x04}, /* ANACTRL62: 4x binning mode */
    {0x3664, 0xf3}, /* ANACTRL64 */
    {0x3665, 0x9e}, /* ANACTRL65 */
    {0x3667, 0xa5}, /* ANACTRL67 */
    {0x366e, 0x55}, /* ANACTRL6e */
    {0x366f, 0x55}, /* ANACTRL6f */
    {0x3670, 0x11}, /* ANACTRL70 */
    {0x3671, 0x11}, /* ANACTRL71 */
    {0x3672, 0x11}, /* ANACTRL72 */
    {0x3673, 0x11}, /* ANACTRL73 */
    /* Timing control */
    {0x3714, 0x28}, /* SENCTRL14 */
    {0x371a, 0x3e}, /* SENCTRL1a */
    {0x3733, 0x10}, /* SENCTRL33 */
    {0x3734, 0x00}, /* SENCTRL34 */
    {0x373d, 0x24}, /* SENCTRL3d */
    {0x3764, 0x20}, /* SENCTRL64 */
    {0x3765, 0x20}, /* SENCTRL65 */
    {0x3766, 0x12}, /* SENCTRL66 */
    {0x37a1, 0x14}, /* SENCTRL_a1 */
    {0x37a8, 0x1c}, /* SENCTRL_a8 */
    {0x37ab, 0x0f}, /* SENCTRL_ab */
    {0x37c2, 0x24}, /* SENCTRL_c2: 4x4 binning */
    {0x37cb, 0x00}, /* SENCTRL_cb */
    {0x37cc, 0x00}, /* SENCTRL_cc */
    {0x37cd, 0x00}, /* SENCTRL_cd */
    {0x37ce, 0x00}, /* SENCTRL_ce */
    {0x37d8, 0x02}, /* SENCTRL_d8 */
    {0x37d9, 0x02}, /* SENCTRL_d9 */
    {0x37dc, 0x04}, /* SENCTRL_dc */
    /* Window / crop / output size - 640x480 (4x4 binned from 2624x1976) */
    {0x3800, 0x00}, /* H_CROP_START[11:8] = 0 */
    {0x3801, 0x00}, /* H_CROP_START[7:0]  = 0 */
    {0x3802, 0x00}, /* V_CROP_START[11:8] = 0 */
    {0x3803, 0x00}, /* V_CROP_START[7:0]  = 0 */
    {0x3804, 0x0a}, /* H_CROP_END[11:8]   = 0x0a */
    {0x3805, 0x3f}, /* H_CROP_END[7:0]    = 0x3f => 2623 */
    {0x3806, 0x07}, /* V_CROP_END[11:8]   = 0x07 */
    {0x3807, 0xb7}, /* V_CROP_END[7:0]    = 0xb7 => 1975 */
    {0x3808, 0x02}, /* X_OUTPUT_SIZE[11:8] = 0x02 */
    {0x3809, 0x80}, /* X_OUTPUT_SIZE[7:0]  = 0x80 => 640 */
    {0x380a, 0x01}, /* Y_OUTPUT_SIZE[11:8] = 0x01 */
    {0x380b, 0xe0}, /* Y_OUTPUT_SIZE[7:0]  = 0xe0 => 480 */
    {0x380c, 0x02}, /* H_TOTAL_SIZE[11:8]  = 0x02 */
    {0x380d, 0xee}, /* H_TOTAL_SIZE[7:0]   = 0xee => HTS=750 */
    {0x380e, 0x07}, /* V_TOTAL_SIZE[11:8]  = 0x07 */
    {0x380f, 0xd0}, /* V_TOTAL_SIZE[7:0]   = 0xd0 => VTS=2000 */
    {0x3811, 0x08}, /* H_WIN_OFFSET[7:0]   = 8 */
    {0x3813, 0x0d}, /* V_WIN_OFFSET[7:0]   = 13 */
    {0x3814, 0x07}, /* H_ODD_INC  = 7 (4x binning) */
    {0x3815, 0x01}, /* H_EVEN_INC = 1 */
    {0x3816, 0x07}, /* V_ODD_INC  = 7 (4x binning) */
    {0x3817, 0x01}, /* V_EVEN_INC = 1 */
    {0x381e, 0x02}, /* TIMING_CTRL1e */
    {0x3820, 0x8f}, /* TIMING_FORMAT1: 4x binning on */
    {0x3821, 0x01}, /* TIMING_FORMAT2: mirror on */
    {0x3832, 0x04}, /* TIMING_CTRL32 */
    /* Strobe / FREX */
    {0x3c80, 0x01}, /* STROBE_CTRL80 */
    {0x3c82, 0x00}, /* STROBE_CTRL82 */
    {0x3c83, 0xc8}, /* STROBE_CTRL83 */
    {0x3c8c, 0x0f}, /* STROBE_CTRL8c */
    {0x3c8d, 0xa0}, /* STROBE_CTRL8d */
    {0x3c90, 0x07}, /* STROBE_CTRL90 */
    {0x3c91, 0x00}, /* STROBE_CTRL91 */
    {0x3c92, 0x00}, /* STROBE_CTRL92 */
    {0x3c93, 0x00}, /* STROBE_CTRL93 */
    {0x3c94, 0xd0}, /* STROBE_CTRL94 */
    {0x3c95, 0x50}, /* STROBE_CTRL95 */
    {0x3c96, 0x35}, /* STROBE_CTRL96 */
    {0x3c97, 0x00}, /* STROBE_CTRL97 */
    /* BLC */
    {0x4001, 0xe0}, /* BLC_CTRL01 */
    {0x4008, 0x00}, /* BLC_CTRL08: start line */
    {0x4009, 0x03}, /* BLC_CTRL09: end line (VGA) */
    {0x400f, 0x80}, /* BLC_CTRL0f */
    {0x4013, 0x02}, /* BLC_CTRL13 */
    {0x4040, 0x00}, /* BLC_CTRL40 */
    {0x4041, 0x03}, /* BLC_CTRL41 */
    {0x404c, 0x50}, /* BLC_CTRL4c */
    {0x404e, 0x20}, /* BLC_CTRL4e */
    /* VFIFO / format */
    {0x4500, 0x06}, /* VFIFO_CTRL00 */
    {0x4503, 0x00}, /* VFIFO_CTRL03 */
    {0x450a, 0x04}, /* VFIFO_CTRL0a */
    /* MIPI */
    {0x4809, 0x04}, /* MIPI_CTRL09 */
    {0x480c, 0x12}, /* MIPI_CTRL0c */
    {0x4819, 0x70}, /* MIPI_CTRL19 */
    {0x4825, 0x32}, /* MIPI_CTRL25 */
    {0x4826, 0x32}, /* MIPI_CTRL26 */
    {0x482a, 0x06}, /* MIPI_CTRL2a */
    {0x4833, 0x08}, /* MIPI_CTRL33 */
    {0x4837, 0x0d}, /* MIPI_PCLK_PERIOD */
    /* ISP */
    {0x5000, 0x77}, /* ISP_CTRL00 */
    /* LSC */
    {0x5b00, 0x01}, /* LSC_CTRL00 */
    {0x5b01, 0x10}, /* LSC_CTRL01 */
    {0x5b02, 0x01}, /* LSC_CTRL02 */
    {0x5b03, 0xdb}, /* LSC_CTRL03 */
    {0x5b05, 0x6c}, /* LSC_CTRL05 */
    {0x5e10, 0xfc}, /* CTRL5e10 */
    /* AEC/AGC fine tuning */
    {0x3500, 0x00}, /* AEC_LONG_EXPO[19:16] */
    {0x3501, 0x3e}, /* AEC_LONG_EXPO[15:8] */
    {0x3502, 0x60}, /* AEC_LONG_EXPO[7:0] */
    {0x3503, 0x08}, /* AEC manual mode */
    {0x3508, 0x04}, /* AEC_LONG_GAIN[9:8] */
    {0x3509, 0x00}, /* AEC_LONG_GAIN[7:0] */
    {0x3832, 0x48}, /* TIMING_CTRL32: updated */
    /* DPC */
    {0x5780, 0x3e}, /* DPC_CTRL00 */
    {0x5781, 0x0f}, /* DPC_CTRL01 */
    {0x5782, 0x44}, /* DPC_CTRL02 */
    {0x5783, 0x02}, /* DPC_CTRL03 */
    {0x5784, 0x01}, /* DPC_CTRL04 */
    {0x5785, 0x01}, /* DPC_CTRL05 */
    {0x5786, 0x00}, /* DPC_CTRL06 */
    {0x5787, 0x04}, /* DPC_CTRL07 */
    {0x5788, 0x02}, /* DPC_CTRL08 */
    {0x5789, 0x0f}, /* DPC_CTRL09 */
    {0x578a, 0xfd}, /* DPC_CTRL0a */
    {0x578b, 0xf5}, /* DPC_CTRL0b */
    {0x578c, 0xf5}, /* DPC_CTRL0c */
    {0x578d, 0x03}, /* DPC_CTRL0d */
    {0x578e, 0x08}, /* DPC_CTRL0e */
    {0x578f, 0x0c}, /* DPC_CTRL0f */
    {0x5790, 0x08}, /* DPC_CTRL10 */
    {0x5791, 0x06}, /* DPC_CTRL11 */
    {0x5792, 0x00}, /* DPC_CTRL12 */
    {0x5793, 0x52}, /* DPC_CTRL13 */
    {0x5794, 0xa3}, /* DPC_CTRL14 */
    {0x4003, 0x40}, /* BLC_CTRL03: BLC redo trigger */
    {0x3107, 0x01}, /* SC_CTRL3107: updated */
    /* Strobe/FREX final settings */
    {0x3c80, 0x08}, /* STROBE_CTRL80: updated */
    {0x3c83, 0xb1}, /* STROBE_CTRL83: updated */
    {0x3c8c, 0x10}, /* STROBE_CTRL8c: updated */
    {0x3c8d, 0x00}, /* STROBE_CTRL8d: updated */
    {0x3c90, 0x00}, /* STROBE_CTRL90: updated */
    {0x3c94, 0x00}, /* STROBE_CTRL94: updated */
    {0x3c95, 0x00}, /* STROBE_CTRL95: updated */
    {0x3c96, 0x00}, /* STROBE_CTRL96: updated */
    {0x37cb, 0x09}, /* SENCTRL_cb: updated */
    {0x37cc, 0x15}, /* SENCTRL_cc: updated */
    {0x37cd, 0x1f}, /* SENCTRL_cd: updated */
    {0x37ce, 0x1f}, /* SENCTRL_ce: updated */
};
#endif /* RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 3 */

/**
 * @brief Set OV5675 coarse integration time (exposure lines).
 * @param intLine  : integration time in rows (from ISP AE).
 */
static int32_t OV5675_Camera_Exposure_Set(uint32_t intLine)
{
    uint32_t reg_val = intLine << 3;
    int32_t  ret;

    ret = OV5675_WRITE_REG(OV5675_COARSE_INTEGRATION_TIME_H, (reg_val >> 16) & 0x0FU, 1);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
    ret = OV5675_WRITE_REG(OV5675_COARSE_INTEGRATION_TIME_M, (reg_val >> 8)  & 0xFFU, 1);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
    return OV5675_WRITE_REG(OV5675_COARSE_INTEGRATION_TIME_L,  reg_val        & 0xFFU, 1);
}

/**
 * @brief Set OV5675 analog gain from ISP AE Q16.16 value.
 * @param gain_q16_16 : total gain in Q16.16 format (0x10000 = 1x).
 */
static int32_t OV5675_Camera_Gain_Set(uint32_t gain_q16_16)
{
    /* OV5675 gain register 0x3508-0x3509: Q7 format (0x0080 = 1x, 0x0780 = 15x).
     * Convert from Q16.16: reg_val = gain_q16_16 * 128 / 65536 = gain_q16_16 >> 9.
     */
    uint32_t reg_val = (gain_q16_16 + 0x100U) >> 9U;  /* rounded */
    int32_t ret;

    if (reg_val < 0x0080U) {
        reg_val = 0x0080U;  /* floor: 1x */
    }
    if (reg_val > 0x0780U) {
        reg_val = 0x0780U;  /* ceil:  15x */
    }
    ret = OV5675_WRITE_REG(OV5675_GLOBAL_GAIN_H, (reg_val >> 8) & 0xFFU, 1);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
    return OV5675_WRITE_REG(OV5675_GLOBAL_GAIN_L, reg_val & 0xFFU, 1);
}


/**
 * @fn           void OV5675_Sensor_Enable_Clk_Src(void)
 * @brief        Enable OV5675 Camera Sensor external clock source configuration.
 * @param[in]    none
 * @return       none
 */
static void OV5675_Sensor_Enable_Clk_Src(void)
{
    set_cpi_pixel_clk(CPI_PIX_CLKSEL_AXI, RTE_OV5675_CAMERA_SENSOR_MIPI_CSI_CLK_SCR_DIV);
}

/**
 * @fn           void OV5675_Sensor_Disable_Clk_Src(void)
 * @brief        Disable OV5675 Camera Sensor external clock source configuration.
 * @param[in]    none
 * @return       none
 */
static void OV5675_Sensor_Disable_Clk_Src(void)
{
    clear_cpi_pixel_clk();
}

/**
 * @fn           int32_t OV5675_Bulk_Write_Reg(const OV5675_REG OV5675_reg[],
 *                                     uint32_t total_num, uint32_t reg_size)
 * @brief        write array of registers value to OV5675 Camera Sensor registers.
 * @param[in]    OV5675_reg : OV5675 Camera Sensor Register Array Structure
 * @param[in]    total_num   : total number of registers(size of array)
 * @param[in]    reg_size    : register size in bytes.
 * @return       \ref execution_status
 */
static int32_t OV5675_Bulk_Write_Reg(const OV5675_REG OV5675_reg[], uint32_t total_num,
                                     uint32_t reg_size)
{
    uint32_t i   = 0;
    int32_t  ret = 0;

    for (i = 0; i < total_num; i++) {
        ret = OV5675_WRITE_REG(OV5675_reg[i].reg_addr, OV5675_reg[i].reg_value, reg_size);
        if (ret != ARM_DRIVER_OK) {
            return ret;
        }
    }

    return ARM_DRIVER_OK;
}

/**
 * @fn           int32_t OV5675_Camera_Hard_Reseten(void)
 * @brief        Hard Reset OV5675 Camera Sensor
 * @param[in]    none
 * @return       \ref execution_status
 */
static int32_t OV5675_Camera_Hard_Reseten(void)
{
    int32_t ret = 0;

    ret         = GPIO_Driver_CAM_RST->Initialize(BOARD_CAMERA_RESET_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->PowerControl(BOARD_CAMERA_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->SetDirection(BOARD_CAMERA_RESET_GPIO_PIN,
                                            GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->Initialize(BOARD_CAMERA_POWER_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->PowerControl(BOARD_CAMERA_POWER_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->SetDirection(BOARD_CAMERA_POWER_GPIO_PIN,
                                            GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->SetValue(BOARD_CAMERA_POWER_GPIO_PIN,
                                        GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->SetValue(BOARD_CAMERA_RESET_GPIO_PIN,
                                        GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    OV5675_DELAY_uSEC(20000);

    return ARM_DRIVER_OK;
}

/**
 * @fn           int32_t OV5675_Init(void)
 * @brief        Initialize OV5675 Camera Sensor
 *               this function will
 *               - initialize i2c instance
 *               - software reset OV5675 Camera Sensor
 *               - read OV5675 chip-id, proceed only
 *                 it is correct.
 * @return       \ref execution_status
 */
static int32_t OV5675_Init(void)
{
    int32_t  ret      = 0;
    uint32_t rcv_data = 0;

    /* Enable camera sensor clock source config*/
    OV5675_Sensor_Enable_Clk_Src();

    /*camera sensor resten*/
    ret               = OV5675_Camera_Hard_Reseten();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Initialize i2c using i3c driver instance depending on
     *  OV5675 Camera Sensor specific i2c configurations
     *   \ref OV5675_camera_sensor_i2c_cnfg
     */
    ret = camera_sensor_i2c_init(&OV5675_camera_sensor_i2c_cnfg);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Read OV5675 Camera Sensor CHIP ID */
    ret = OV5675_READ_REG(OV5675_REG_CHIPID_H, &rcv_data, 1);
    if (ret != ARM_DRIVER_OK) {
        /* Module not found with first known i2c slave addr;
         * Try the second known address
         */
        OV5675_camera_sensor_i2c_cnfg.cam_sensor_slave_addr = OV5675_CAMERA_SENSOR_SLAVE_ADDR_B;
        ret = OV5675_READ_REG(OV5675_REG_CHIPID_H, &rcv_data, 1);
        if (ret != ARM_DRIVER_OK) {
            return ret;
        }
    }

    uint16_t chipid = rcv_data << 8;

    ret             = OV5675_READ_REG(OV5675_REG_CHIPID_L, &rcv_data, 1);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    chipid |= rcv_data;

    /* Proceed only if CHIP ID is correct. */
    if (chipid != OV5675_CHIP_ID_REGISTER_VALUE) {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return OV5675_WRITE_REG(OV5675_REG_MODE_SELECT, OV5675_MODE_STANDBY, 1);
}

/**
 * @fn           int32_t OV5675_Start(void)
 * @brief        Start OV5675 Camera Sensor Streaming.
 * @param[in]    none
 * @return       \ref execution_status
 */
static int32_t OV5675_Start(void)
{
    /* Start streaming */
    return OV5675_WRITE_REG(OV5675_REG_MODE_SELECT, OV5675_MODE_STREAMING, 1);
}

/**
 * @fn           int32_t OV5675_Stop(void)
 * @brief        Stop OV5675 Camera Sensor Streaming.
 * @param[in]    none
 * @return       \ref execution_status
 */
static int32_t OV5675_Stop(void)
{
    /* Suspend stream */
    return OV5675_WRITE_REG(OV5675_REG_MODE_SELECT, OV5675_MODE_STANDBY, 1);
}

/**
 * @fn           int32_t OV5675_Control(uint32_t control, uint32_t arg)
 * @brief        Control OV5675 Camera Sensor.
 * @param[in]    control  : Operation
 * @param[in]    arg      : Argument of operation
 * @return       \ref execution_status
 */
static int32_t OV5675_Control(uint32_t control, uint32_t arg)
{
    int32_t ret;

    ARG_UNUSED(arg);
    switch (control) {
    case CPI_CAMERA_SENSOR_CONFIGURE:
#if (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 1)
        ret = OV5675_Bulk_Write_Reg(OV5675_1920x1080_regs,
                                    ARRAY_SIZE(OV5675_1920x1080_regs), 1);
#elif (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 2)
        ret = OV5675_Bulk_Write_Reg(OV5675_1280x720_regs,
                                    ARRAY_SIZE(OV5675_1280x720_regs), 1);
#elif (RTE_OV5675_CAMERA_SENSOR_IMAGE_CONFIG == 3)
        ret = OV5675_Bulk_Write_Reg(OV5675_640x480_regs,
                                    ARRAY_SIZE(OV5675_640x480_regs), 1);
#else
        ret = OV5675_Bulk_Write_Reg(OV5675_1296x972_regs,
                                    ARRAY_SIZE(OV5675_1296x972_regs), 1);
#endif
        if (ret != ARM_DRIVER_OK) {
            return ret;
        }
#if (RTE_ISP)
        /* Disable internal AE; ISP will drive exposure/gain. */
        return OV5675_WRITE_REG(OV5675_AEC_MANUAL_REG, OV5675_AEC_MANUAL_VAL, 1);
#else
        return ARM_DRIVER_OK;
#endif

    case CPI_ISP_CAMERA_SENSOR_EXPOSURE:
        return OV5675_Camera_Exposure_Set(arg & 0xFFFFU);

    case CPI_ISP_CAMERA_SENSOR_GAIN:
        return OV5675_Camera_Gain_Set(arg);

    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }
}

/**
 * @fn           int32_t OV5675_Uninit(void)
 * @brief        Un-initialize OV5675 Camera Sensor.
 * @param[in]    none
 * @return       \ref execution_status
 */
static int32_t OV5675_Uninit(void)
{
    int32_t ret;

    /*Disable camera sensor clock source config*/
    OV5675_Sensor_Disable_Clk_Src();

    ret = GPIO_Driver_CAM_RST->SetValue(BOARD_CAMERA_RESET_GPIO_PIN,
                                        GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->PowerControl(BOARD_CAMERA_RESET_GPIO_PIN, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_RST->Uninitialize(BOARD_CAMERA_RESET_GPIO_PIN);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->SetValue(BOARD_CAMERA_POWER_GPIO_PIN,
                                        GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_CAM_PWR->PowerControl(BOARD_CAMERA_POWER_GPIO_PIN, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return GPIO_Driver_CAM_PWR->Uninitialize(BOARD_CAMERA_POWER_GPIO_PIN);
}

/**
 * @brief OV5675 Camera Sensor CSI information
 * \ref CSI_INFO
 */
static CSI_INFO OV5675_csi_info = {
    .frequency                = RTE_OV5675_CAMERA_SENSOR_CSI_FREQ,
    .dt                       = RTE_OV5675_CAMERA_SENSOR_CSI_DATA_TYPE,
    .n_lanes                  = RTE_OV5675_CAMERA_SENSOR_CSI_N_LANES,
    .vc_id                    = RTE_OV5675_CAMERA_SENSOR_CSI_VC_ID,
    .cpi_cfg.override         = RTE_OV5675_CAMERA_SENSOR_OVERRIDE_CPI_COLOR_MODE,
    .cpi_cfg.cpi_color_mode   = RTE_OV5675_CAMERA_SENSOR_CPI_COLOR_MODE
};

/**
 * @brief OV5675 Camera Sensor Operations
 * \ref CAMERA_SENSOR_OPERATIONS
 */
static CAMERA_SENSOR_OPERATIONS OV5675_ops = {
    .Init    = OV5675_Init,
    .Uninit  = OV5675_Uninit,
    .Start   = OV5675_Start,
    .Stop    = OV5675_Stop,
    .Control = OV5675_Control,
};

/**
 * @brief OV5675 Camera Sensor Device Structure
 * \ref CAMERA_SENSOR_DEVICE
 */
static CAMERA_SENSOR_DEVICE OV5675_camera_sensor = {
    .interface = CAMERA_SENSOR_INTERFACE_MIPI,
    .width     = RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH,
    .height    = RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT,
    .csi_info  = &OV5675_csi_info,
    .ops       = &OV5675_ops,
};

/* Registering CPI sensor */
CAMERA_SENSOR(OV5675_camera_sensor)

#endif /* RTE_OV5675_CAMERA_SENSOR_ENABLE */
