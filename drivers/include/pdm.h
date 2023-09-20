/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef PDM_H_
#define PDM_H_

#include <stdint.h>
#include <stdbool.h>

/**
  * @brief LPPDM (LPPDM)
  */

typedef struct {                                   /*!< (@ 0x43002000) LPPDM Structure                                            */
  volatile uint32_t  PDM_CTL0;                     /*!< (@ 0x00000000) PDM Audio Control Register 0                               */
  volatile uint32_t  PDM_CTL1;                     /*!< (@ 0x00000004) PDM Audio Control Register 1                               */
  volatile uint32_t  PDM_FIFO_WATERMARK_H;         /*!< (@ 0x00000008) FIFO Watermark Register                                    */
  volatile const  uint32_t  PDM_FIFO_STAT;         /*!< (@ 0x0000000C) FIFO Status Register                                       */
  volatile const  uint32_t  PDM_ERROR_IRQ;         /*!< (@ 0x00000010) FIFO Error Interrupt Status Register                       */
  volatile const  uint32_t  PDM_WARN_IRQ;          /*!< (@ 0x00000014) FIFO Warning Interrupt Status Register                     */
  volatile const  uint32_t  PDM_AUDIO_DETECT_IRQ;  /*!< (@ 0x00000018) Audio Detection Interrupt Status Register                  */
  volatile uint32_t  PDM_IRQ_ENABLE;               /*!< (@ 0x0000001C) Interrupt Enable Register                                  */
  volatile uint32_t  PDM_CH0_CH1_AUDIO_OUT;        /*!< (@ 0x00000020) Channels 0 and 1 Audio Output Register                     */
  volatile uint32_t  PDM_CH2_CH3_AUDIO_OUT;        /*!< (@ 0x00000024) Channels 2 and 3 Audio Output Register                     */
  volatile uint32_t  PDM_CH4_CH5_AUDIO_OUT;        /*!< (@ 0x00000028) Channels 4 and 5 Audio Output Register                     */
  volatile uint32_t  PDM_CH6_CH7_AUDIO_OUT;        /*!< (@ 0x0000002C) Channels 6 and 7 Audio Output Register                     */
  volatile const  uint32_t  RESERVED[4];
  volatile uint32_t  PDM_CH0_FIR_COEF_0;           /*!< (@ 0x00000040) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_1;           /*!< (@ 0x00000044) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_2;           /*!< (@ 0x00000048) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_3;           /*!< (@ 0x0000004C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_4;           /*!< (@ 0x00000050) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_5;           /*!< (@ 0x00000054) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_6;           /*!< (@ 0x00000058) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_7;           /*!< (@ 0x0000005C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_8;           /*!< (@ 0x00000060) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_9;           /*!< (@ 0x00000064) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH0_FIR_COEF_10;          /*!< (@ 0x00000068) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH0_FIR_COEF_11;          /*!< (@ 0x0000006C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH0_FIR_COEF_12;          /*!< (@ 0x00000070) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH0_FIR_COEF_13;          /*!< (@ 0x00000074) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH0_FIR_COEF_14;          /*!< (@ 0x00000078) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH0_FIR_COEF_15;          /*!< (@ 0x0000007C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH0_FIR_COEF_16;          /*!< (@ 0x00000080) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH0_FIR_COEF_17;          /*!< (@ 0x00000084) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED1[14];
  volatile uint32_t  PDM_CH0_IIR_COEF_SEL;         /*!< (@ 0x000000C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                      */
  volatile uint32_t  PDM_CH0_PHASE;                /*!< (@ 0x000000C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH0_GAIN;                 /*!< (@ 0x000000C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH0_PKDET_TH;             /*!< (@ 0x000000CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH0_PKDET_ITV;            /*!< (@ 0x000000D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH0_PKDET_STAT;    /*!< (@ 0x000000D4) Channel (n) Peak Detector Status Register                  */
  volatile const  uint32_t  RESERVED2[26];
  volatile uint32_t  PDM_CH1_FIR_COEF_0;           /*!< (@ 0x00000140) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_1;           /*!< (@ 0x00000144) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_2;           /*!< (@ 0x00000148) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_3;           /*!< (@ 0x0000014C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_4;           /*!< (@ 0x00000150) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_5;           /*!< (@ 0x00000154) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_6;           /*!< (@ 0x00000158) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_7;           /*!< (@ 0x0000015C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_8;           /*!< (@ 0x00000160) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_9;           /*!< (@ 0x00000164) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH1_FIR_COEF_10;          /*!< (@ 0x00000168) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH1_FIR_COEF_11;          /*!< (@ 0x0000016C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH1_FIR_COEF_12;          /*!< (@ 0x00000170) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH1_FIR_COEF_13;          /*!< (@ 0x00000174) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH1_FIR_COEF_14;          /*!< (@ 0x00000178) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH1_FIR_COEF_15;          /*!< (@ 0x0000017C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH1_FIR_COEF_16;          /*!< (@ 0x00000180) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH1_FIR_COEF_17;          /*!< (@ 0x00000184) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED3[14];
  volatile uint32_t  PDM_CH1_IIR_COEF_SEL;         /*!< (@ 0x000001C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                      */
  volatile uint32_t  PDM_CH1_PHASE;                /*!< (@ 0x000001C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH1_GAIN;                 /*!< (@ 0x000001C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH1_PKDET_TH;             /*!< (@ 0x000001CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH1_PKDET_ITV;            /*!< (@ 0x000001D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH1_PKDET_STAT;    /*!< (@ 0x000001D4) Channel (n) Peak Detector Status Register                  */
  volatile const  uint32_t  RESERVED4[26];
  volatile uint32_t  PDM_CH2_FIR_COEF_0;           /*!< (@ 0x00000240) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_1;           /*!< (@ 0x00000244) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_2;           /*!< (@ 0x00000248) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_3;           /*!< (@ 0x0000024C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_4;           /*!< (@ 0x00000250) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_5;           /*!< (@ 0x00000254) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_6;           /*!< (@ 0x00000258) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_7;           /*!< (@ 0x0000025C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_8;           /*!< (@ 0x00000260) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_9;           /*!< (@ 0x00000264) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH2_FIR_COEF_10;          /*!< (@ 0x00000268) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH2_FIR_COEF_11;          /*!< (@ 0x0000026C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH2_FIR_COEF_12;          /*!< (@ 0x00000270) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH2_FIR_COEF_13;          /*!< (@ 0x00000274) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH2_FIR_COEF_14;          /*!< (@ 0x00000278) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH2_FIR_COEF_15;          /*!< (@ 0x0000027C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH2_FIR_COEF_16;          /*!< (@ 0x00000280) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH2_FIR_COEF_17;          /*!< (@ 0x00000284) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED5[14];
  volatile uint32_t  PDM_CH2_IIR_COEF_SEL;         /*!< (@ 0x000002C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                      */
  volatile uint32_t  PDM_CH2_PHASE;                /*!< (@ 0x000002C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH2_GAIN;                 /*!< (@ 0x000002C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH2_PKDET_TH;             /*!< (@ 0x000002CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH2_PKDET_ITV;            /*!< (@ 0x000002D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH2_PKDET_STAT;    /*!< (@ 0x000002D4) Channel (n) Peak Detector Status Register                  */
  volatile const  uint32_t  RESERVED6[26];
  volatile uint32_t  PDM_CH3_FIR_COEF_0;           /*!< (@ 0x00000340) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_1;           /*!< (@ 0x00000344) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_2;           /*!< (@ 0x00000348) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_3;           /*!< (@ 0x0000034C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_4;           /*!< (@ 0x00000350) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_5;           /*!< (@ 0x00000354) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_6;           /*!< (@ 0x00000358) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_7;           /*!< (@ 0x0000035C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_8;           /*!< (@ 0x00000360) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_9;           /*!< (@ 0x00000364) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH3_FIR_COEF_10;          /*!< (@ 0x00000368) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH3_FIR_COEF_11;          /*!< (@ 0x0000036C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH3_FIR_COEF_12;          /*!< (@ 0x00000370) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH3_FIR_COEF_13;          /*!< (@ 0x00000374) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH3_FIR_COEF_14;          /*!< (@ 0x00000378) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH3_FIR_COEF_15;          /*!< (@ 0x0000037C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH3_FIR_COEF_16;          /*!< (@ 0x00000380) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH3_FIR_COEF_17;          /*!< (@ 0x00000384) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED7[14];
  volatile uint32_t  PDM_CH3_IIR_COEF_SEL;         /*!< (@ 0x000003C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                       */
  volatile uint32_t  PDM_CH3_PHASE;                /*!< (@ 0x000003C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH3_GAIN;                 /*!< (@ 0x000003C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH3_PKDET_TH;             /*!< (@ 0x000003CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH3_PKDET_ITV;            /*!< (@ 0x000003D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH3_PKDET_STAT;    /*!< (@ 0x000003D4) Channel (n) Peak Detector Status Register                  */
  volatile const  uint32_t  RESERVED8[26];
  volatile uint32_t  PDM_CH4_FIR_COEF_0;           /*!< (@ 0x00000440) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_1;           /*!< (@ 0x00000444) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_2;           /*!< (@ 0x00000448) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_3;           /*!< (@ 0x0000044C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_4;           /*!< (@ 0x00000450) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_5;           /*!< (@ 0x00000454) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_6;           /*!< (@ 0x00000458) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_7;           /*!< (@ 0x0000045C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_8;           /*!< (@ 0x00000460) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_9;           /*!< (@ 0x00000464) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH4_FIR_COEF_10;          /*!< (@ 0x00000468) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH4_FIR_COEF_11;          /*!< (@ 0x0000046C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH4_FIR_COEF_12;          /*!< (@ 0x00000470) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH4_FIR_COEF_13;          /*!< (@ 0x00000474) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH4_FIR_COEF_14;          /*!< (@ 0x00000478) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH4_FIR_COEF_15;          /*!< (@ 0x0000047C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH4_FIR_COEF_16;          /*!< (@ 0x00000480) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH4_FIR_COEF_17;          /*!< (@ 0x00000484) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED9[14];
  volatile uint32_t  PDM_CH4_IIR_COEF_SEL;         /*!< (@ 0x000004C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                      */
  volatile uint32_t  PDM_CH4_PHASE;                /*!< (@ 0x000004C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH4_GAIN;                 /*!< (@ 0x000004C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH4_PKDET_TH;             /*!< (@ 0x000004CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH4_PKDET_ITV;            /*!< (@ 0x000004D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH4_PKDET_STAT;    /*!< (@ 0x000004D4) Channel (n) Peak Detector Status Register                  */
  volatile const  uint32_t  RESERVED10[26];
  volatile uint32_t  PDM_CH5_FIR_COEF_0;           /*!< (@ 0x00000540) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_1;           /*!< (@ 0x00000544) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_2;           /*!< (@ 0x00000548) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_3;           /*!< (@ 0x0000054C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_4;           /*!< (@ 0x00000550) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_5;           /*!< (@ 0x00000554) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_6;           /*!< (@ 0x00000558) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_7;           /*!< (@ 0x0000055C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_8;           /*!< (@ 0x00000560) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_9;           /*!< (@ 0x00000564) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH5_FIR_COEF_10;          /*!< (@ 0x00000568) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH5_FIR_COEF_11;          /*!< (@ 0x0000056C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH5_FIR_COEF_12;          /*!< (@ 0x00000570) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH5_FIR_COEF_13;          /*!< (@ 0x00000574) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH5_FIR_COEF_14;          /*!< (@ 0x00000578) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH5_FIR_COEF_15;          /*!< (@ 0x0000057C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH5_FIR_COEF_16;          /*!< (@ 0x00000580) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH5_FIR_COEF_17;          /*!< (@ 0x00000584) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED11[14];
  volatile uint32_t  PDM_CH5_IIR_COEF_SEL;         /*!< (@ 0x000005C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                      */
  volatile uint32_t  PDM_CH5_PHASE;                /*!< (@ 0x000005C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH5_GAIN;                 /*!< (@ 0x000005C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH5_PKDET_TH;             /*!< (@ 0x000005CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH5_PKDET_ITV;            /*!< (@ 0x000005D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH5_PKDET_STAT;    /*!< (@ 0x000005D4) Channel (n) Peak Detector Status Register                  */
  volatile const  uint32_t  RESERVED12[26];
  volatile uint32_t  PDM_CH6_FIR_COEF_0;           /*!< (@ 0x00000640) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_1;           /*!< (@ 0x00000644) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_2;           /*!< (@ 0x00000648) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_3;           /*!< (@ 0x0000064C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_4;           /*!< (@ 0x00000650) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_5;           /*!< (@ 0x00000654) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_6;           /*!< (@ 0x00000658) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_7;           /*!< (@ 0x0000065C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_8;           /*!< (@ 0x00000660) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_9;           /*!< (@ 0x00000664) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH6_FIR_COEF_10;          /*!< (@ 0x00000668) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH6_FIR_COEF_11;          /*!< (@ 0x0000066C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH6_FIR_COEF_12;          /*!< (@ 0x00000670) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH6_FIR_COEF_13;          /*!< (@ 0x00000674) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH6_FIR_COEF_14;          /*!< (@ 0x00000678) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH6_FIR_COEF_15;          /*!< (@ 0x0000067C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH6_FIR_COEF_16;          /*!< (@ 0x00000680) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH6_FIR_COEF_17;          /*!< (@ 0x00000684) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED13[14];
  volatile uint32_t  PDM_CH6_IIR_COEF_SEL;         /*!< (@ 0x000006C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                      */
  volatile uint32_t  PDM_CH6_PHASE;                /*!< (@ 0x000006C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH6_GAIN;                 /*!< (@ 0x000006C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH6_PKDET_TH;             /*!< (@ 0x000006CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH6_PKDET_ITV;            /*!< (@ 0x000006D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH6_PKDET_STAT;    /*!< (@ 0x000006D4) Channel (n) Peak Detector Status Register                  */
  volatile const  uint32_t  RESERVED14[26];
  volatile uint32_t  PDM_CH7_FIR_COEF_0;           /*!< (@ 0x00000740) Channel (n) FIR Filter Coefficient 0 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_1;           /*!< (@ 0x00000744) Channel (n) FIR Filter Coefficient 1 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_2;           /*!< (@ 0x00000748) Channel (n) FIR Filter Coefficient 2 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_3;           /*!< (@ 0x0000074C) Channel (n) FIR Filter Coefficient 3 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_4;           /*!< (@ 0x00000750) Channel (n) FIR Filter Coefficient 4 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_5;           /*!< (@ 0x00000754) Channel (n) FIR Filter Coefficient 5 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_6;           /*!< (@ 0x00000758) Channel (n) FIR Filter Coefficient 6 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_7;           /*!< (@ 0x0000075C) Channel (n) FIR Filter Coefficient 7 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_8;           /*!< (@ 0x00000760) Channel (n) FIR Filter Coefficient 8 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_9;           /*!< (@ 0x00000764) Channel (n) FIR Filter Coefficient 9 Register              */
  volatile uint32_t  PDM_CH7_FIR_COEF_10;          /*!< (@ 0x00000768) Channel (n) FIR Filter Coefficient 10 Register             */
  volatile uint32_t  PDM_CH7_FIR_COEF_11;          /*!< (@ 0x0000076C) Channel (n) FIR Filter Coefficient 11 Register             */
  volatile uint32_t  PDM_CH7_FIR_COEF_12;          /*!< (@ 0x00000770) Channel (n) FIR Filter Coefficient 12 Register             */
  volatile uint32_t  PDM_CH7_FIR_COEF_13;          /*!< (@ 0x00000774) Channel (n) FIR Filter Coefficient 13 Register             */
  volatile uint32_t  PDM_CH7_FIR_COEF_14;          /*!< (@ 0x00000778) Channel (n) FIR Filter Coefficient 14 Register             */
  volatile uint32_t  PDM_CH7_FIR_COEF_15;          /*!< (@ 0x0000077C) Channel (n) FIR Filter Coefficient 15 Register             */
  volatile uint32_t  PDM_CH7_FIR_COEF_16;          /*!< (@ 0x00000780) Channel (n) FIR Filter Coefficient 16 Register             */
  volatile uint32_t  PDM_CH7_FIR_COEF_17;          /*!< (@ 0x00000784) Channel (n) FIR Filter Coefficient 17 Register             */
  volatile const  uint32_t  RESERVED15[14];
  volatile uint32_t  PDM_CH7_IIR_COEF_SEL;         /*!< (@ 0x000007C0) Channel (n) IIR Filter Coefficient Selection
                                                                    Register                                                      */
  volatile uint32_t  PDM_CH7_PHASE;                /*!< (@ 0x000007C4) Channel (n) Phase Control Register                         */
  volatile uint32_t  PDM_CH7_GAIN;                 /*!< (@ 0x000007C8) Channel (n) Gain Control Register                          */
  volatile uint32_t  PDM_CH7_PKDET_TH;             /*!< (@ 0x000007CC) Channel (n) Peak Detector Threshold Register               */
  volatile uint32_t  PDM_CH7_PKDET_ITV;            /*!< (@ 0x000007D0) Channel (n) Peak Detector Interval Register                */
  volatile const  uint32_t  PDM_CH7_PKDET_STAT;    /*!< (@ 0x000007D4) Channel (n) Peak Detector Status Register                  */
} PDM_Type;

/* Taking the difference of two different channel gain register address value */
#define PDM_CH_OFFSET                 ((uint32_t *)&(pdm->PDM_CH1_GAIN) - (uint32_t *)&(pdm->PDM_CH0_GAIN))

#define PDM0_IRQ_ENABLE               (0xFF03)                  /* To enable the interrupt status register */
#define PDM_BYPASS_IIR                (1 << 2)                  /* Bypass DC blocking IIR filter           */
#define PDM_BYPASS_FIR                (1 << 3)                  /* Bypass FIR filter                       */
#define PDM_PEAK_DETECT_NODE          (1 << 4)                  /* Peak detection node                     */
#define PDM_DMA_HANDSHAKE             (1 << 24)                 /* DMA handshaking signals for flow control*/
#define PDM_SAMPLE_ADV                (1 << 17)                 /* Sample advance                          */

#define PDM_INTERRUPT_STATUS_VALUE    (0x1)                     /* To check the interrupt status           */

#define PDM_FIFO_ALMOST_FULL_IRQ      (0x1 << 0)                /* FIFO almost full Interrupt              */
#define PDM_FIFO_OVERFLOW_IRQ         (0x1 << 1)                /* FIFO overflow interrupt                 */
#define PDM_AUDIO_DETECT_IRQ_STAT     (0xFF << 8)               /* Audio detect interrupt                  */
#define PDM_CHANNEL_ENABLE            (0xFF)                    /* To check the which channel is enabled   */
#define PDM_MODES                     (0xFF << 16)              /* To check for the PDM modes              */

#define PDM_FIFO_CLEAR                (1 << 31)                 /* To clear FIFO clear bit                 */

#define PDM_MAX_FIR_COEFFICIENT       18                        /* PDM channel FIR length                  */
#define PDM_MAX_DMA_CHANNEL           1                         /* PDM DMA maximum channel                 */

#define PDM_AUDIO_CH_0_1              0                         /* PDM audio channel 0 and 1               */
#define PDM_AUDIO_CH_2_3              1                         /* PDM audio channel 2 and 3               */
#define PDM_AUDIO_CH_4_5              2                         /* PDM audio channel 4 and 5               */
#define PDM_AUDIO_CH_6_7              3                         /* PDM audio channel 6 and 7               */
#define PDM_CLK_MODE                  16                        /* PDM clock frequency mode                */

#define PDM_AUDIO_CHANNEL             (0x3)
#define PDM_CHANNEL_0_1               (PDM_AUDIO_CHANNEL << 0)  /* check for channel 0 and 1               */
#define PDM_CHANNEL_2_3               (PDM_AUDIO_CHANNEL << 2)  /* check for channel 2 and 3               */
#define PDM_CHANNEL_4_5               (PDM_AUDIO_CHANNEL << 4)  /* check for channel 4 and 5               */
#define PDM_CHANNEL_6_7               (PDM_AUDIO_CHANNEL << 6)  /* check for channel 6 and 7               */
#define PDM_CHANNEL_0                 (1 << 0)
#define PDM_CHANNEL_1                 (1 << 1)
#define PDM_CHANNEL_2                 (1 << 2)
#define PDM_CHANNEL_3                 (1 << 3)
#define PDM_CHANNEL_4                 (1 << 4)
#define PDM_CHANNEL_5                 (1 << 5)
#define PDM_CHANNEL_6                 (1 << 6)
#define PDM_CHANNEL_7                 (1 << 7)

typedef enum _PDM_TRANSFER_STATUS
{
    PDM_CAPTURE_STATUS_NONE,        /* PDM capture status none     */
    PDM_AUDIO_STATUS_DETECTION,     /* PDM status audio detection  */
    PDM_CAPTURE_STATUS_COMPLETE,    /* PDM capture status complete */
    PDM_ERROR_DETECT,               /* PDM error detection status  */
}PDM_TRANSFER_STATUS;

/**
 @brief struct pdm_transfer_t:- To store PDM Receive Configuration
 */
typedef struct{
    uint32_t curr_cnt;                    /* Current count value                                      */
    uint32_t total_cnt;                   /* Total count value                                        */
    void *buf;                            /* Channel audio output values are stored in this address   */
    volatile PDM_TRANSFER_STATUS status;  /* transfer status                                          */
}pdm_transfer_t;

/**
 @fn          void pdm_bypass_iir(PDM_Type *pdm, bool arg)
 @brief       Select the Bypass DC blocking IIR filter
 @param[in]   pdm : Pointer to the PDM register map
 @param[in]   arg : Enable or disable the bypass IIR filter
 @return      none
 */
static inline void pdm_bypass_iir(PDM_Type *pdm, bool arg)
{
    if(arg)  /* Enable the bypass IIR filter */
        pdm->PDM_CTL1 |= PDM_BYPASS_IIR;

    else  /* Disable the bypass IIR filter */
        pdm->PDM_CTL1 &= ~(PDM_BYPASS_IIR);
}

/**
 @fn          void pdm_bypass_fir(PDM_Type *pdm, bool arg)
 @brief       To select the Bypass FIR filter
 @param[in]   pdm : Pointer to the PDM register map
 @param[in]   arg : Enable or disable the bypass FIR filter
 @return      none
 */
static inline void pdm_bypass_fir(PDM_Type *pdm, bool arg)
{
    if(arg)  /* Enable the bypass FIR filter */
        pdm->PDM_CTL1 |= PDM_BYPASS_FIR;

    else  /* Disable the bypass FIR filter */
        pdm->PDM_CTL1 &= ~(PDM_BYPASS_FIR);
}

/**
 @fn          void pdm_peak_detect(PDM_Type *pdm, bool arg)
 @brief       To select the Bypass FIR filter
 @param[in]   pdm : Pointer to the PDM register map
 @param[in]   arg : Enable or disable the peak detection node
 @return      none
 */
static inline void pdm_peak_detect(PDM_Type *pdm, bool arg)
{
    if(arg)  /* peak detection after gain stage */
        pdm->PDM_CTL1 |= PDM_PEAK_DETECT_NODE;

    else  /* peak detection before gain stage */
        pdm->PDM_CTL1 &= ~(PDM_PEAK_DETECT_NODE);
}

/**
 @fn          void pdm_sample_advance(PDM_Type *pdm, bool arg)
 @brief       To select the Sample advance
 @param[in]   pdm : Pointer to the PDM register map
 @param[in]   arg : Enable or disable the Sample advance
 @return      none
 */
static inline void pdm_sample_advance(PDM_Type *pdm, bool arg)
{
    if(arg)  /* Enable the Sample advance */
        pdm->PDM_CTL1 |= PDM_SAMPLE_ADV;

    else  /* Disable the Sample advance */
        pdm->PDM_CTL1 &= ~(PDM_SAMPLE_ADV);
}

/**
 @fn          void pdm_dma_handshake(PDM_Type *pdm, bool arg)
 @brief       To Use DMA handshaking signals for flow control
              (Not yet implemented)
 @param[in]   pdm : Pointer to the PDM register map
 @param[in]   arg : Enable or disable the DMA Handshake
 @return      none
 */
static inline void pdm_dma_handshake(PDM_Type *pdm, bool arg)
{
    if(arg)  /* Enable the DMA handshake */
        pdm->PDM_CTL1 |= PDM_DMA_HANDSHAKE;

    else  /* Disable the DMA handshake */
        pdm->PDM_CTL1 &= ~(PDM_DMA_HANDSHAKE);
}

/**
 @fn          void pdm_enable_irq(PDM_Type *pdm)
 @brief       Enable the Fifo overflow IRQ
 @param[in]   pdm : Pointer to the PDM register map
 @return      None
 */
static inline void pdm_dma_enable_irq(PDM_Type *pdm)
{

    pdm->PDM_IRQ_ENABLE &= ~(PDM0_IRQ_ENABLE); /* Clear IRQ */

    /* Enable the Interrupt */
    pdm->PDM_IRQ_ENABLE |= (PDM_FIFO_OVERFLOW_IRQ);
}

/**
 @fn          void pdm_enable_irq(PDM_Type *pdm)
 @brief       Enable the IRQ
 @param[in]   pdm : Pointer to the PDM register map
 @return      None
 */
static inline void pdm_enable_irq(PDM_Type *pdm)
{
    uint32_t audio_ch;

    pdm->PDM_IRQ_ENABLE &= ~(PDM0_IRQ_ENABLE); /* Clear IRQ */

    /* get user enabled channel */
    audio_ch = ((pdm->PDM_CTL0)) & PDM_CHANNEL_ENABLE;

    /* Enable the Interrupt */
    pdm->PDM_IRQ_ENABLE |= (( audio_ch  << 8) | (PDM_FIFO_ALMOST_FULL_IRQ | PDM_FIFO_OVERFLOW_IRQ));
}

/**
 @fn          void pdm_enable_fifo_clear(PDM_Type *pdm)
 @brief       Enable the fifo clear bit
 @param[in]   pdm : Pointer to the PDM register map
 @return      None
 */
static inline void pdm_enable_fifo_clear(PDM_Type *pdm)
{
    pdm->PDM_CTL0 |= PDM_FIFO_CLEAR;
}

/**
 @fn          void pdm_disable_fifo_clear(PDM_Type *pdm)
 @brief       Disable the fifo clear bit
 @param[in]   pdm : Pointer to the PDM register map
 @return      None
 */
static inline void pdm_disable_fifo_clear(PDM_Type *pdm)
{
    pdm->PDM_CTL0 &= ~(PDM_FIFO_CLEAR);
}

/**
 @fn          void pdm_clear_modes(PDM_Type *pdm)
 @brief       Clear the PDM modes
 @param[in]   pdm : Pointer to the PDM register map
 @return      None
 */
static inline void pdm_clear_modes(PDM_Type *pdm)
{
    pdm->PDM_CTL0 &= ~(PDM_MODES);
}

/**
 @fn          void pdm_enable_modes(PDM_Type *pdm, uint32_t arg))
 @brief       Enable the PDM modes
 @param[in]   pdm : Pointer to the PDM register map
 @param[in]   arg : Select the pdm frequency modes
 @return      None
 */
static inline void pdm_enable_modes(PDM_Type *pdm, uint32_t arg)
{
    pdm->PDM_CTL0 |= (arg << PDM_CLK_MODE);
}

/**
 @fn          void pdm_clear_channel(PDM_Type *pdm)
 @brief       Clear the PDM channels
 @param[in]   pdm : Pointer to the PDM register map
 @return      None
 */
static inline void pdm_clear_channel(PDM_Type *pdm)
{
    pdm->PDM_CTL0 &= ~(PDM_CHANNEL_ENABLE);
}

/**
 @fn          void pdm_set_fifo_watermark(PDM_Type *pdm, uint32_t fifo_watermark))
 @brief       Set the pdm fifo watermark value
 @param[in]   pdm            : Pointer to the PDM register map
 @param[in]   fifo_watermark : Threshold to trigger FIFO almost full warning interrupt
 @return      None
 */
static inline void pdm_set_fifo_watermark(PDM_Type *pdm, uint32_t fifo_watermark)
{
    pdm->PDM_FIFO_WATERMARK_H |= fifo_watermark;
}

/**
 @fn          void pdm_enable_multi_ch(PDM_Type *pdm, uint32_t arg))
 @brief       Enable the PDM multiple channels
 @param[in]   pdm : Pointer to the PDM register map
 @param[in]   arg : Enable the pdm multiple channels
 @return      None
 */
static inline void pdm_enable_multi_ch(PDM_Type *pdm, uint32_t arg)
{
    pdm->PDM_CTL0 |= arg;
}

/**
 @fn          void pdm_set_ch_iir_coef(PDM_Type *pdm, uint8_t ch_num, uint32_t ch_iir_coef)
 @brief       Set the pdm channel IIR filter coefficient value
 @param[in]   pdm         : Pointer to the PDM register map
 @param[in]   ch_num      : Select the pdm channel
 @param[in]   ch_iir_coef : Set the pdm channel IIR filter coefficient value
 @return      None
 */
static inline void pdm_set_ch_iir_coef(PDM_Type *pdm, uint8_t ch_num, uint32_t ch_iir_coef)
{
    uint32_t *CH_CNFG = (uint32_t *)&(pdm->PDM_CH0_IIR_COEF_SEL) + PDM_CH_OFFSET * ch_num;

    *CH_CNFG = ch_iir_coef;
}

/**
 @fn          void pdm_set_ch_phase(PDM_Type *pdm, uint8_t ch_num, uint32_t ch_phase)
 @brief       Set the pdm channel phase control value
 @param[in]   pdm      : Pointer to the PDM register map
 @param[in]   ch_num   : Select the pdm channel
 @param[in]   ch_phase : Set the pdm channel phase control value
 @return      None
 */
static inline void pdm_set_ch_phase(PDM_Type *pdm, uint8_t ch_num, uint32_t ch_phase)
{
    uint32_t *CH_CNFG = (uint32_t *)&(pdm->PDM_CH0_PHASE) + PDM_CH_OFFSET * ch_num;

    *CH_CNFG = ch_phase;
}

/**
 @fn          void pdm_set_ch_gain(PDM_Type *pdm, uint8_t ch_num, uint32_t ch_gain)
 @brief       Set the pdm channel gain control value
 @param[in]   pdm      : Pointer to the PDM register map
 @param[in]   ch_num   : Select the pdm channel
 @param[in]   ch_gain  : Set the pdm channel gain control value
 @return      None
 */
static inline void pdm_set_ch_gain(PDM_Type *pdm, uint8_t ch_num, uint32_t ch_gain)
{
    uint32_t *CH_CNFG = (uint32_t *)&(pdm->PDM_CH0_GAIN) + PDM_CH_OFFSET * ch_num;

    *CH_CNFG = ch_gain;
}

/**
 @fn          void pdm_set_peak_detect_th(PDM_Type *pdm, uint8_t ch_num,
                                          uint32_t ch_peak_detect_th)
 @brief       Set the pdm channel Peak detector threshold value
 @param[in]   pdm      : Pointer to the PDM register map
 @param[in]   ch_num   : Select the pdm channel
 @param[in]   ch_peak_detect_th  : Set the pdm channel Peak detector
                                   threshold value
 @return      None
 */
static inline void pdm_set_peak_detect_th(PDM_Type *pdm, uint8_t ch_num,
                                          uint32_t ch_peak_detect_th)
{
    uint32_t *CH_CNFG = (uint32_t *)&(pdm->PDM_CH0_PKDET_TH) + PDM_CH_OFFSET * ch_num;

    *CH_CNFG = ch_peak_detect_th;
}

/**
 @fn          void pdm_set_peak_detect_th(PDM_Type *pdm, uint8_t ch_num,
                                          uint32_t ch_peak_detect_itv)
 @brief       Set the pdm channel Peak detector interval value
 @param[in]   pdm      : Pointer to the PDM register map
 @param[in]   ch_num   : Select the pdm channel
 @param[in]   ch_peak_detect_itv  : Set the pdm channel Peak detector
                                    interval value
 @return      None
 */
static inline void pdm_set_peak_detect_itv(PDM_Type *pdm, uint8_t ch_num,
                                           uint32_t ch_peak_detect_itv)
{
    uint32_t *CH_CNFG = (uint32_t *)&(pdm->PDM_CH0_PKDET_ITV) + PDM_CH_OFFSET * ch_num;

    *CH_CNFG = ch_peak_detect_itv;
}

/**
 @fn          void pdm_set_fir_coeff(PDM_Type *pdm, uint8_t ch_num,
                                     uint32_t ch_fir_coef[PDM_MAX_FIR_COEFFICIENT])
 @brief       Set the pdm channel Peak detector interval value
 @param[in]   pdm      : Pointer to the PDM register map
 @param[in]   ch_num   : Select the pdm channel
 @param[in]   ch_fir_coef  : Set the pdm channel Fir coefficient values
 @return      None
 */
static inline void pdm_set_fir_coeff(PDM_Type *pdm, uint8_t ch_num, uint32_t ch_fir_coef[PDM_MAX_FIR_COEFFICIENT])
{
    uint32_t i;

    uint32_t *ch_n_fir_coef_0 =  (uint32_t *)&(pdm->PDM_CH0_FIR_COEF_0) + PDM_CH_OFFSET  * ch_num;

    for(i = 0; i< PDM_MAX_FIR_COEFFICIENT; i++)
    {
        *(ch_n_fir_coef_0) = ch_fir_coef[i];
         ch_n_fir_coef_0 ++;
    }
}

/**
 @fn          uint32_t pdm_get_active_channels(PDM_Type *pdm)
 @brief       return PDM active channels
 @param[in]   pdm : Pointer to the PDM register map
 @return      PDM active channels
 */
static inline uint32_t pdm_get_active_channels(PDM_Type *pdm)
{
    return pdm->PDM_CTL0 & PDM_CHANNEL_ENABLE;
}

/**
 @fn          uint32_t* pdm_get_ch0_1_addr(PDM_Type *pdm)
 @brief       return PDM channel 0 and 1 reg address
 @param[in]   pdm : Pointer to the PDM register map
 @return      return the address
 */
static inline volatile uint32_t* pdm_get_ch0_1_addr(PDM_Type *pdm)
{
    return &(pdm->PDM_CH0_CH1_AUDIO_OUT);
}

/**
 @fn          uint32_t* pdm_get_ch2_3_addr(PDM_Type *pdm)
 @brief       return PDM channel 2 and 3 reg address
 @param[in]   pdm : Pointer to the PDM register map
 @return      return the address
 */
static inline volatile uint32_t* pdm_get_ch2_3_addr(PDM_Type *pdm)
{
    return &(pdm->PDM_CH2_CH3_AUDIO_OUT);
}

/**
 @fn          uint32_t* pdm_get_ch4_5_addr(PDM_Type *pdm)
 @brief       return PDM channel 4 and 5 reg address
 @param[in]   pdm : Pointer to the PDM register map
 @return      return the address
 */
static inline volatile uint32_t* pdm_get_ch4_5_addr(PDM_Type *pdm)
{
    return &(pdm->PDM_CH4_CH5_AUDIO_OUT);
}

/**
 @fn          uint32_t* pdm_get_ch6_7_addr(PDM_Type *pdm)
 @brief       return PDM channel 6 and 7 reg address
 @param[in]   pdm : Pointer to the PDM register map
 @return      return the address
 */
static inline volatile uint32_t* pdm_get_ch6_7_addr(PDM_Type *pdm)
{
    return &(pdm->PDM_CH6_CH7_AUDIO_OUT);
}

/**
 @fn          void pdm_disable_error_irq(PDM_Type *pdm)
 @brief       Disable pdm error irq
 @param[in]   pdm : Pointer to the PDM register map
 @return      None
 */
static inline void pdm_disable_error_irq(PDM_Type *pdm)
{
    pdm->PDM_IRQ_ENABLE &= ~PDM_FIFO_OVERFLOW_IRQ;
}
/**
  @fn          void pdm_error_detect_irq_handler(PDM_Type *pdm);
  @brief       IRQ handler for the error interrupt
  @param[in]   pdm      : Pointer to the PDM register map
  @return      none
*/
void pdm_error_detect_irq_handler(PDM_Type *pdm);

/**
  @fn          void pdm_audio_detect_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer );
  @brief       IRQ handler for the audio detect interrupt
  @param[in]   pdm      : Pointer to the PDM register map
  @param[in]   transfer     : The transfer structure of the PDM instance
  @return      none
*/
void pdm_audio_detect_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer );

/**
  @fn          void pdm_warning_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer);
  @brief       IRQ handler for the PDM warning interrupt.
  @param[in]   pdm      : Pointer to the PDM register map
  @param[in]   transfer : The transfer structure of the PDM instance
  @return      none
*/
void pdm_warning_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer);

#endif /* PDM_H_ */
