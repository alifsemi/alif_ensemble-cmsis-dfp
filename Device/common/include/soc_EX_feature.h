/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SOC_EX_FEATURE_H
#define SOC_EX_FEATURE_H

/* LPTIMER device specific macros for Ex device */
#define DEVICE_FEATURE_LPTIMER_MAX_CHANNELS   4U

/* UTIMER device specific macros for Ex device */
#define DEVICE_FEATURE_UTIMER_MAX_CHANNELS    16U
#define DEVICE_FEATURE_QEC_SEPARATE_CHANNELS

/* GPIO/LPGPIO device specific macros for Ex device */
#define DEVICE_FEATURE_LPGPIO_MAX_PINS        8U
#define DEVICE_FEATURE_GPIO1_FLEXIO_PIN_MASK  0U
#define DEVICE_FEATURE_GPIO7_FLEXIO_PIN_MASK  ((1U << 4) | (1U << 5) | (1U << 6) | (1U << 7))
#define DEVICE_FEATURE_LPGPIO_FLEXIO_PIN_MASK ((1U << 0) | (1U << 1) | (1U << 2) | (1U << 3))

/* LPSPI device specific macros for Ex device */
#define DEVICE_FEATURE_LPSPI_MASTER_ONLY

/* DMA device specific macros for EX device */
#define DEVICE_FEATURE_DMALOCAL_DMASEL_GPIO_GLITCH_FILTER_ENABLE

/* OSPI specific macros for Ex device */
#define DEVICE_FEATURE_OSPI_HAS_XIP_SER       1

/* I2S Master/Slave */
#define DEVICE_FEATURE_I2S0_MASTER_MODE       1
#define DEVICE_FEATURE_I2S1_MASTER_MODE       1
#define DEVICE_FEATURE_I2S2_MASTER_MODE       1
#define DEVICE_FEATURE_I2S3_MASTER_MODE       1
#define DEVICE_FEATURE_LPI2S_MASTER_MODE      1

#define DEVICE_FEATURE_I2S_EXT_AUDIO_CLK_PRESENT 1

#endif  /* SOC_EX_FEATURE_H */
