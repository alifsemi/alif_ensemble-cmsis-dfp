/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SOC_FEATURES_H
#define SOC_FEATURES_H

/* Cores */
#define SOC_FEAT_HAS_CORE_M55_HP                   (1)
#define SOC_FEAT_HAS_CORE_M55_HE                   (1)
#define SOC_FEAT_HAS_CORE_A32_0                    (1)
#define SOC_FEAT_HAS_CORE_A32_1                    (1)

#define SOC_FEAT_M55_HE_CORE_ID                    (3)
#define SOC_FEAT_M55_HP_CORE_ID                    (2)
#define SOC_FEAT_A32_0_CORE_ID                     (0)
#define SOC_FEAT_A32_1_CORE_ID                     (1)

/* Clocks */
#define SOC_FEAT_M55_HE_MAX_HZ                     (160000000)
#define SOC_FEAT_M55_HP_MAX_HZ                     (400000000)
#define SOC_FEAT_A32_MAX_HZ                        (800000000)

#define SOC_FEAT_ACLK_MAX_HZ                       (400000000)
#define SOC_FEAT_HCLK_MAX_HZ                       (200000000)
#define SOC_FEAT_PCLK_MAX_HZ                       (100000000)
#define SOC_FEAT_REFCLK_MAX_HZ                     (100000000)
#define SOC_FEAT_HFOSC_MAX_HZ                      (38400000)
#define SOC_FEAT_PLL_CLK1_MAX_HZ                   (800000000)
#define SOC_FEAT_PLL_CLK3_MAX_HZ                   (480000000)

/* Memory Regions */
#define SOC_FEAT_MRAM_SIZE                         (0x00580000)
#define SOC_FEAT_MRAM_BASE                         (0x80000000)

#define SOC_FEAT_HAS_BULK_SRAM                     (1)

#define SOC_FEAT_BULK_SRAM0_SIZE                   (0x00400000)
#define SOC_FEAT_BULK_SRAM0_BASE                   (0x02000000)

#define SOC_FEAT_BULK_SRAM1_SIZE                   (0x00400000)
#define SOC_FEAT_BULK_SRAM1_BASE                   (0x08000000)

#define SOC_FEAT_HP_ITCM_SIZE                      (0x00040000)
#define SOC_FEAT_HP_ITCM_BASE                      (0x50000000)

#define SOC_FEAT_HP_DTCM_SIZE                      (0x00100000)
#define SOC_FEAT_HP_DTCM_BASE                      (0x50800000)

#define SOC_FEAT_HE_ITCM_SIZE                      (0x00040000)
#define SOC_FEAT_HE_ITCM_BASE                      (0x58000000)

#define SOC_FEAT_HE_DTCM_SIZE                      (0x00040000)
#define SOC_FEAT_HE_DTCM_BASE                      (0x58800000)

/* Peripheral Features */

#define SOC_FEAT_HAS_EVTRTR0                       (1)

#define SOC_FEAT_HAS_FLT_ENA_IN_DMA_SEL_REG        (1)
#define SOC_FEAT_DMA2_HAS_FLT_ENA0_REG             (0)
#define SOC_FEAT_DMA2_HAS_FLT_ENA1_REG             (0)

#define SOC_FEAT_HAS_UTIMER4_15                    (1)
#define SOC_FEAT_QEC_HAS_SEP_CHANNELS              (1)

#define SOC_FEAT_HAS_LPTIMER2_3                    (1)
#define SOC_FEAT_LPTIMER_HAS_PWM                   (1)

#define SOC_FEAT_HAS_LPRTC1                        (0)

#define SOC_FEAT_HAS_BLE                           (0)

#define SOC_FEAT_USB_EP_TOTAL                      (16)

#define SOC_FEAT_LPSPI_HAS_MASTER_SLAVE            (1)

#define SOC_FEAT_CRC_RESULT_REFLECT_ENABLE         (0)

#define SOC_FEAT_ADC_REG_ALIASING                  (1)

#define SOC_FEAT_OSPI_HAS_XIP_SER                  (0)
#define SOC_FEAT_OSPI_HAS_CLK_ENABLE               (1)
#define SOC_FEAT_OSPI_ADDR_IN_SINGLE_FIFO_LOCATION (1)
#define SOC_FEAT_AES_BAUD2_DELAY_VAL               (0)
#define SOC_FEAT_AES_OSPI_SIGNALS_DELAY            (1)
#define SOC_FEAT_AES_OSPI_HAS_XIP_WRITE_HC_DFS     (1)

#define SOC_FEAT_I2S0_MASTER_MODE                  (1)
#define SOC_FEAT_I2S1_MASTER_MODE                  (1)
#define SOC_FEAT_I2S2_MASTER_MODE                  (1)
#define SOC_FEAT_I2S3_MASTER_MODE                  (1)
#define SOC_FEAT_LPI2S_MASTER_MODE                 (1)
#define SOC_FEAT_I2S_HAS_EXT_AUDIO_CLK             (1)

#define SOC_FEAT_CLK76P8M_CLK_ENABLE               (1)

#define SOC_FEAT_HSCMP_REG_ALIASING                (1)

#define SOC_FEAT_HSCMP_INT_STATUS_MASK             (3U)
#define SOC_FEAT_HSCMP_WINDOW_ENABLE_VAL           (3)
#define SOC_FEAT_DAC_REG_ALIASING                  (1)

#define SOC_FEAT_CANFD0_CANFD1_CTRL                (0)

#define SOC_FEAT_GPIO_HAS_CLOCK_ENABLE             (1)
#define SOC_FEAT_GPIO_HAS_HARDWARE_CTRL_MODE       (1)
#define SOC_FEAT_GPIO_HAS_HW_BIT_MANIPULATION      (1)
#define SOC_FEAT_GPIO1_FLEXIO_PIN_MASK             (0U)
#define SOC_FEAT_GPIO7_FLEXIO_PIN_MASK             ((1U << 4) | (1U << 5) | (1U << 6) | (1U << 7))
#define SOC_FEAT_LPGPIO_HAS_PIN2_7                 (1)
#define SOC_FEAT_LPGPIO_FLEXIO_PIN_MASK            ((1U << 0) | (1U << 1) | (1U << 2) | (1U << 3))
#define SOC_FEAT_GPIO_HAS_PORT10_14                (1)
#define SOC_FEAT_GPIO_HAS_PORT16_17                (1)

#define SOC_FEAT_FORCE_ENABLE_SYSTEM_CLOCKS        (1)

#define SOC_FEAT_I3C_CORE_CLK_AXI                  (0)

#define SOC_FEAT_I2C_HAS_RESTART_CAP               (1)

#define SOC_FEAT_CPI_HAS_CROPPING       (1)
#define SOC_FEAT_HAS_ISP                (1)

#define SOC_FEAT_U55_M1_CAN_ACCESS_HIGHER_ADDRESS (1)

#endif /* SOC_FEATURES_H */
