/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SOC_E1C_FEATURE_H
#define SOC_E1C_FEATURE_H

/* LPTIMER device specific macros for E1C device */
#define DEVICE_FEATURE_LPTIMER_MAX_CHANNELS   2U

/* UTIMER device specific macros for E1C device */
#define DEVICE_FEATURE_UTIMER_MAX_CHANNELS    4U

/* GPIO/LPGPIO device specific macros for E1C device */
#define DEVICE_FEATURE_LPGPIO_MAX_PINS        2U
#define DEVICE_FEATURE_GPIO1_FLEXIO_PIN_MASK  ((1U << 4) | (1U << 5) | (1U << 6) | (1U << 7))
#define DEVICE_FEATURE_GPIO7_FLEXIO_PIN_MASK  0U
#define DEVICE_FEATURE_LPGPIO_FLEXIO_PIN_MASK (1U << 0)
#define DEVICE_FEATURE_GPIO_HAS_CLOCK_ENABLE

/* DMA device specific macros for E1C device */
#define DEVICE_FEATURE_DMA2_GPIO_GLITCH_FILTER_ENABLE0
#define DEVICE_FEATURE_DMA2_GPIO_GLITCH_FILTER_ENABLE1

/* OSPI specific macros for E1C device */
#define DEVICE_FEATURE_OSPI_ADDRESS_IN_SINGLE_FIFO_LOCATION 1
#define DEVICE_FEATURE_OSPI_CTRL_CLK_ENABLE                 1

/* CANFD Clock control macro for E1C device */
#define DEVICE_FEATURE_CANFD0_CANFD1_CONTROL    1U

/* I2S Master/Slave */
#define DEVICE_FEATURE_I2S0_MASTER_MODE       1
#define DEVICE_FEATURE_I2S1_MASTER_MODE       0
#define DEVICE_FEATURE_LPI2S_MASTER_MODE      1

#define DEVICE_FEATURE_I2S_EXT_AUDIO_CLK_PRESENT 0

#endif  /* SOC_E1C_FEATURE_H */
