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
 * @file     ov5675_isp_param.c
 * @brief    ISP calibration and pipeline settings for the OV5675 sensor.
 *           This file is compiled when the OV5675 camera sensor component
 *           is selected in the RTE configuration.
 *
 *           calibration_data holds the OV5675 low-lux calibration.
 ******************************************************************************/

#include "RTE_Components.h"
#include "RTE_Device.h"

#if defined(RTE_Drivers_ISP) && defined(RTE_ISP) && (RTE_ISP == 1)

#include "isp_param.h"

/* ---------------------------------------------------------------------------
 * ISP Calibration Data - OV5675
 * --------------------------------------------------------------------------- */
ISP_CALIB_DATA_S calibration_data = {
    .modules = {
        .autoRoute = {
            .autoRoute = {100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200,
                    102400, 204800, 409600, 819200, 1638400, 3276800},
        },
        .bls = {
            .enable = 1,
            .opType = OP_TYPE_MANUAL,
            .manualAttr = {
                .blackLevel = {256, 256, 256, 256},
            },
            .autoAttr = {
                .blackLevel  = {
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                    {256, 256, 256, 256},
                },
            },
        },
        .aem = {
            .enable = 1,
            .expAltMode = 0,
            .blockWin = {
                .hOffs = 0,
                .vOffs = 0,
                .hSize = RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH,
                .vSize = RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT,
            },
        },

        .ae = {
            .opType = OP_TYPE_AUTO,
            .manualAttr = {
                .intTime = 33000,
                .again = 8 * 1024,
                .dgain = 1024,
            },
            .autoAttr = {
                .expTimeRange = {
                    .min =  100,
                    .max =  16000,
                },
                .againRange = {
                    .min = 1 * 1024,
                    .max = 15 * 1024,
                },
                .dgainRange = {
                    .min = 1 * 1024,
                    .max = 1 * 1024,
                },
                .aeRunInterval = 2,
                .aeTarget = 50,
                .dampOver = 30,
                .dampUnder = 30,
                .tolerance = 12,
                .antiflicker = {
                    .enable = 0,
                    .flickerFreq = 100,
                },
                .aeMode = AE_MODE_FIX_FRAME_RATE,
                .gainThreshold = 15360,
                .aeRoute = {
                    .totalNum = 0,
                },
                .aeDelayAttr = {
                    .blackDelayFrame = 0,
                    .whiteDelayFrame = 0,
                },
                .weight = {
                    {1, 1, 1, 1, 1},
                    {1, 1, 1, 1, 1},
                    {1, 1, 1, 1, 1},
                    {1, 1, 1, 1, 1},
                    {1, 1, 1, 1, 1}
                },
            },
        },

        .wbm = {
            .enable   = 1,
            .measMode = ISP_AWB_MEAS_MODE_RGB,
            .measRect = {
                .hOffs = 0,
                .vOffs = 0,
                .hSize = RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH,
                .vSize = RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT,
            },
            .wpRange = {
                .maxY       = 0xEB,
                .refCr_MaxR = 0x80,
                .minY_MaxG  = 0xC0,
                .refCb_MaxB = 0x80,
                .maxCSum    = 0x14,
                .minC       = 0x14,
            },
        },

        .wb = {
            .enable = 1,
            .opType = OP_TYPE_AUTO,
            .manualAttr = {
                .wbGain = {0x100, 0x100, 0x100, 0x100},
            },
            .autoAttr = {
                .runInterval = 12,
                .speed = 64,
                .tolerance = 1,
                .initColorTemp = 5000,
                .calibParam = {
                    .centLine = {-766376, -642393, -2921800},
                    .rgMin = 1086700,
                    .rgMax = 3050800,
                    .wpRange0 = {
                        .wpLCurve = {
                            .rg = { 790659, 953496, 1087350, 1277770,
                                1427190, 1521530, 1594220, 1791900,
                                1965800, 2113520, 2264770, 2450610,
                                2600660, 2750720, 2818650,
                                3050800 },
                            .dist = { 560682, 506246, 480332, 408497,
                                256789, 269600, 317996, 416285,
                                245149, 80289, -42413, -71649,
                                -4396, 111498, 229011, 453350 },
                        },
                        .wpRCurve = {
                            .rg = { 796080, 971478, 1096380, 1295500,
                                1423700, 1550280, 1700330, 1850390,
                                2000440, 2150500, 2300550, 2450610,
                                2630520, 2750720, 2900770,
                                3050800 },
                            .dist = { -48178, 37442, 108593, 173398,
                                161051, 170806, 219416, 259086,
                                288821, 304528, 301610, 271649,
                                266912,  88502,  -67514,
                                -253350 },
                        },
                    },
                    .wpRange1 = {
                        .wpLCurve = {
                            .rg = { 791651, 1289310, 1508700, 1631550,
                                1762950, 1861430, 1976370, 2091310,
                                2206250, 2321190, 2490840, 2629990,
                                2687900, 2780950, 2895890,
                                3010800 },
                            .dist = { -178231, 46924, 124591, 108020,
                                168323, 171752, 194948, 210209,
                                216012, 209423, 225067, 235329,
                                111501, -30197, -152124, -292180 },
                        },
                        .wpRCurve = {
                            .rg =  { 797383, 1275640, 1482030, 1601860,
                                1792600, 1869890, 1959780, 2076980,
                                2281190, 2332430, 2431000, 2574250,
                                2654400, 2765620, 2839520,
                                3010800 },
                            .dist = { 449646, 289261, 93411, 142743,
                                266621, 286629, 206521, 53599,
                                -134047, -144737, -124515, -148586,
                                -49479,  73539,  227056, 312180 },
                        },
                    },
                    .illuminant[ILLUMINANT_A] = {
                        .illuType = ILLUMINANT_A,
                        .colorTemp = 2800,
                        .wbGain = { 0x124, 0x100, 0x100, 0x205 },
                    },
                    .illuminant[ILLUMINANT_TL84] = {
                        .illuType = ILLUMINANT_TL84,
                        .colorTemp = 3800,
                        .wbGain = { 0x15B, 0x100, 0x100, 0x1B5 },
                    },
                    .illuminant[ILLUMINANT_CWF] = {
                        .illuType = ILLUMINANT_CWF,
                        .colorTemp = 4100,
                        .wbGain = { 0x178, 0x100, 0x100, 0x1C5 },
                    },
                    .illuminant[ILLUMINANT_D50] = {
                        .illuType = ILLUMINANT_D50,
                        .colorTemp = 5000,
                        .wbGain = { 0x182, 0x100, 0x100, 0x181 },
                    },
                    .illuminant[ILLUMINANT_D65] = {
                        .illuType = ILLUMINANT_D65,
                        .colorTemp = 6500,
                        .wbGain = { 0x1D4, 0x100, 0x100, 0x140 },
                    },
                },
            },
        },

        .dmsc = {
            .enable    = 1,
            .threshold = 10,
            .cacAttr   = {
                .enable      = 0,
                .hClipMode   = 0,
                .vClipMode   = 0,
                .hStart      = 0,
                .vStart      = 0,
                .aBlue       = 0,
                .aRed        = 0,
                .bBlue       = 0,
                .bRed        = 0,
                .cBlue       = 0,
                .cRed        = 0,
                .xNormShift  = 7,
                .xNormFactor = 29,
                .yNormShift  = 7,
                .yNormFactor = 29,
            },
        },

        .flt = {
            .enable = 1,
            .opType = OP_TYPE_MANUAL,
            .manualAttr = {
                .denoiseLevel = 6,
                .sharpenLevel = 2,
            },
            .autoAttr = {
                .denoiseLevel = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                .sharpenLevel = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            },
        },

        .ccm = {
            .opType = OP_TYPE_AUTO,
            .manualAttr = {
                .colorMatrix = {
                    128, 0, 0,
                    0, 128, 0,
                    0, 0, 128
                },
                .rOffset = 0,
                .gOffset = 0,
                .bOffset = 0,
            },
            .autoAttr = {
                .illuminantCCM[ILLUMINANT_A] = {
                    .colorTemp   = 2800,
                    .colorMatrix = {249, -38, -83, -76, 287, -83, -41, -170, 339},
                    .rOffset = 0, .gOffset = 0, .bOffset = 0,
                },
                .illuminantCCM[ILLUMINANT_TL84] = {
                    .colorTemp   = 3800,
                    .colorMatrix = {292, -85, -79, -71, 273, -74, -17, -125, 269},
                    .rOffset = 0, .gOffset = 0, .bOffset = 0,
                },
                .illuminantCCM[ILLUMINANT_CWF] = {
                    .colorTemp   = 4100,
                    .colorMatrix = {356, -163, -65, -79, 267, -60, -20, -116, 265},
                    .rOffset = 0, .gOffset = 0, .bOffset = 0,
                },
                .illuminantCCM[ILLUMINANT_D50] = {
                    .colorTemp   = 5000,
                    .colorMatrix = {277, -86, -64, -55, 274, -91, -8, -137, 273},
                    .rOffset = 0, .gOffset = 0, .bOffset = 0,
                },
                .illuminantCCM[ILLUMINANT_D65] = {
                    .colorTemp   = 6500,
                    .colorMatrix = {295, -122, -45, -52, 268, -88, -7, -109, 245},
                    .rOffset = 0, .gOffset = 0, .bOffset = 0,
                },
            },
        },

        .gammaOut = {
            .enable = 1,
            .gammaY = {
                0, 142, 230, 305, 373, 436, 495, 557, 629,
                697, 761, 821, 876, 925, 968, 1003, 1023
            },
        },

        .csm = {
            .type = ISP_CSM_TYPE_601,
            .quantization = ISP_CSM_LIM_RANGE,
        },
    },
};

/* ---------------------------------------------------------------------------
 * ISP Port Attribute - OV5675
 * Native resolution: 2592x1944 (5MP)
 * --------------------------------------------------------------------------- */
ISP_PORT_ATTR_S port_attr = {
    .ispInputType = INPUT_TYPE_SENSOR,
    .ispMode      = ISP_MODE_RAW,
    .hdrMode      = HDR_MODE_LINEAR,
    .pixelFormat  = PIXEL_FORMAT_GBRG10,
    .snsRect = {
        .top    = 0,
        .left   = 0,
        .width  = RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH,
        .height = RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT,
    },
    .inFormRect = {
        .top    = 0,
        .left   = 0,
        .width  = RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH,
        .height = RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT,
    },
    .iSRect = {
        .top    = 0,
        .left   = 0,
        .width  = RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH,
        .height = RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT,
    },
    .outFormRect = {
        .top    = 0,
        .left   = 0,
        .width  = RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH,
        .height = RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT,
    },
};

/* ---------------------------------------------------------------------------
 * ISP Channel Attribute - OV5675
 * --------------------------------------------------------------------------- */
ISP_CHN_ATTR_S chan_attr = {
    .transBus = TRANS_BUS_ONLINE,
    .chnFormat = {
        .width       = RTE_ISP_OUTPUT_WIDTH,
        .height      = RTE_ISP_OUTPUT_HEIGHT,
        .pixelFormat = RTE_ISP_OUTPUT_FORMAT,
    },
};

void isp_param_set_crop(vsi_u32_t top, vsi_u32_t left, vsi_u32_t width, vsi_u32_t height)
{
    /* RECT_S field naming is swapped in the libisp:
     * RECT_S.top  -> ISP_OUT_H_OFFS (horizontal/left offset)
     * RECT_S.left -> ISP_OUT_V_OFFS (vertical/top offset)
     * We swap here so our API uses conventional image coordinates.
     */
    port_attr.outFormRect.top    = left;
    port_attr.outFormRect.left   = top;
    port_attr.outFormRect.width  = width;
    port_attr.outFormRect.height = height;
}

void isp_param_set_output_dimensions(vsi_u32_t width, vsi_u32_t height)
{
    chan_attr.chnFormat.width  = width;
    chan_attr.chnFormat.height = height;
}

#endif /* defined(RTE_Drivers_ISP) && defined(RTE_ISP) && (RTE_ISP == 1) */
