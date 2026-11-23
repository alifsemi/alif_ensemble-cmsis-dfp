/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit : https://alifsemi.com/license
 */

#ifndef SENSOR_ATTRIBUTES_H_
#define SENSOR_ATTRIBUTES_H_

#include "RTE_Components.h"
#include "vsi_comm_sns.h"

#if defined(__GNUC__) || defined(__clang__) || defined(__ARMCC_VERSION)
#define SENSOR_ATTR_MAYBE_UNUSED __attribute__((unused))
#else
#define SENSOR_ATTR_MAYBE_UNUSED
#endif

/**
 * @brief Sensor-specific AE default attributes, selected at compile time
 * by the active RTE_Drivers_CAMERA_SENSOR_* macro.
 * Consumed by AE_GetDefaults() in Driver_ISP.c.
 *
 * One table is included per enabled camera sensor component. Single- or
 * multi-camera builds are supported; AE_GetDefaults() uses the active
 * sensor via isp_params->ae_defaults.
 */

#if defined(RTE_Drivers_CAMERA_SENSOR_MT9M114)

static AE_SNS_DEFAULT_S mt9m114_sensor_attributes SENSOR_ATTR_MAYBE_UNUSED = {
    /* MT9M114 @ 30fps, 1280x720 MIPI
     * cam_sensor_cfg_frame_length_lines (0xC812) = 1000
     */
    .fullLinesMax = 0xFFFF,
    .fullLinesStd = 1000,
    .fullLines    = 1000,
    .fps = 30 * ISP_SNS_FPS_ACCU,
    /* linesPer500ms = fullLines * fps / (2 * ISP_SNS_FPS_ACCU)
     *               = 1000 * 3000 / 200 = 15000
     * (also computed dynamically in AE_GetAeDefault)
     */
    .linesPer500ms = 15000,
    /* maxIntLine = fullLines - 2 blanking rows = 998 */
    .maxIntLine = 998,
    .minIntLine = 1,
    .intLineStep = 1,
    /* Gains in Q10 format (ISP_SNS_GAIN_ACCU=1024 -> 1024=1.0x).
     * Register 0x3028: bits[6:4]=coarse(0-3 -> 1x,2x,4x,8x),
     *                  bits[3:0]=fine(0-15 -> 1.0x..1.9375x in 1/16 steps)
     * maxAgain=8192 -> coarse=3, fine=0 -> 8x analog gain.
     */
    .maxAgain  = 8192,
    .minAgain  = 1024,
    .againStep = 1,
    .maxDgain  = 1024,
    .minDgain  = 1024,
    .dgainStep = 1,
    .aeRunInterval = 3,
    .aeTarget = 128,
    .dampOver  = 0x20,
    .dampUnder = 0x20,
    .tolerance = 1,
    /* initExposure = intLine * again * dgain / ISP_SNS_GAIN_ACCU
     *              = 500 * 1024 * 1024 / 1024 = 512000
     * (500 lines at 1x gain — mid-range starting point)
     */
    .initExposure  = 512000,
    .aeMode        = AE_MODE_FIX_FRAME_RATE,
    .gainThreshold = 1024,
};

#endif /* RTE_Drivers_CAMERA_SENSOR_MT9M114 */

#if defined(RTE_Drivers_CAMERA_SENSOR_OV5675)

static AE_SNS_DEFAULT_S ov5675_sensor_attributes SENSOR_ATTR_MAYBE_UNUSED = {
    /* OV5675 @ 30fps, 1296x972 MIPI (2-lane, 450MHz)
     * VTS (0x380E-0x380F) = 0x07D0 = 2000
     * HTS (0x380C-0x380D) = 0x02EE = 750
     * Pixel rate = 2 lanes * 450Mbps / 10bit = 90Mpps
     * FPS = 90Mpps / (750 * 2000) = 30fps (verify against hardware)
     */
    .fullLinesMax = 0xFFFF,
    .fullLinesStd = 2000,
    .fullLines    = 2000,
    .fps = 30 * ISP_SNS_FPS_ACCU,
    /* linesPer500ms = fullLines * fps / (2 * ISP_SNS_FPS_ACCU)
     *               = 2000 * 3000 / 200 = 30000
     */
    .linesPer500ms = 30000,
    /* maxIntLine = VTS - 4 blanking rows = 1996
     * OV5675 requires minimum 4 rows blanking overhead.
     */
    .maxIntLine = 1996,
    .minIntLine = 2,
    .intLineStep = 1,
    /* OV5675 gain register 0x3508-0x3509 uses Q7 format (0x0080 = 1x).
     * ISP AE uses Q10 format (1024 = 1x).
     * Driver converts Q16.16 totalGain to OV5675 Q7 register.
     * maxAgain = 15360 Q10 -> 15x analog (register 0x0780).
     */
    .maxAgain  = 15360,
    .minAgain  = 1024,
    .againStep = 1,
    .maxDgain  = 1024,
    .minDgain  = 1024,
    .dgainStep = 1,
    .aeRunInterval = 6,
    .aeTarget = 100,
    .dampOver  = 0x10,
    .dampUnder = 0x10,
    .tolerance = 30,
    /* initExposure = intLine * again_Q10
     *             = 498 * 8192 = 4079616
     * Matches init table: 0x3500-0x3502 = 0x001F20 -> 498 lines,
     *                     0x3508-0x3509 = 0x0400 -> 8x gain (Q7) = 8192 Q10.
     */
    .initExposure  = 4079616,
    .aeMode        = AE_MODE_FIX_FRAME_RATE,
    .gainThreshold = 15360,
};

#endif /* RTE_Drivers_CAMERA_SENSOR_OV5675 */

#if defined(RTE_Drivers_CAMERA_SENSOR_ARX3A0)

static AE_SNS_DEFAULT_S arx3a0_sensor_attributes SENSOR_ATTR_MAYBE_UNUSED = {
    /* ARX3A0 @ 90fps, 560x560
     * FRAME_LENGTH_LINES (0x300A) = 0x04A6 = 1190
     */
    .fullLinesMax = 0xFFFF,
    .fullLinesStd = 0x04A6,
    .fullLines    = 0x04A6,
    .fps = 90 * ISP_SNS_FPS_ACCU,
    /* linesPer500ms = fullLines * fps / (2 * ISP_SNS_FPS_ACCU)
     *               = 1190 * 9000 / 200 = 53550
     * (also computed dynamically in AE_GetAeDefault)
     */
    .linesPer500ms = 53550,
    /* maxIntLine = sensor init COARSE_INTEGRATION_TIME = 0x0441 = 1089.
     * Do NOT use fullLines-1=1189: ARX3A0 driver warns against increasing
     * integration time above the init value as it interferes with frame timing.
     * (FRAME_LENGTH_LINES=1190 needs >=101 rows blanking overhead.)
     */
    .maxIntLine = 0x0441,
    .minIntLine = 0x1,
    .intLineStep = 1,
    /* Gains in Q10 format (ISP_SNS_GAIN_ACCU=1024 -> 1024=1.0x).
     * Application converts: totalGain_q16 = (again * dgain) / 16
     *   again=1024,  dgain=1024 -> (1024*1024)/16  = 0x10000  = 1x
     *   again=10752, dgain=1024 -> (10752*1024)/16 = 0xA8000 = 10.5x
     * ARX3A0_Camera_Gain_Set uses digital gain to reach >8x analog:
     * 0xA8000 -> analog=8x, digital=84/64=1.3125x, total=10.5x = sensor default.
     * dgain is fixed at 1x; the ARX3A0 driver handles digital gain internally.
     */
    .maxAgain  = 10 * 1024,
    .minAgain  = 1 * 1024,
    .againStep = 1,
    .maxDgain  = 8 * 1024,
    .minDgain  = 1 * 1024,
    .dgainStep = 1,
    .aeRunInterval = 1,
    .aeTarget = 100,
    .dampOver  = 0x40,
    .dampUnder = 0x40,
    .tolerance = 10,
    /* initExposure = intLine * again * dgain / ISP_SNS_GAIN_ACCU
     *              = 1089 * 10752 * 1024 / 1024 = 11709888
     * Matches sensor init: intLine=0x0441=1089, gain=10.5x (GLOBAL_GAIN=0x2035)
     */
    .initExposure  = 11709888,
    .aeMode        = AE_MODE_FIX_FRAME_RATE,
    .gainThreshold = 12000,
};

#endif /* RTE_Drivers_CAMERA_SENSOR_ARX3A0 */

#endif /* SENSOR_ATTRIBUTES_H_ */
