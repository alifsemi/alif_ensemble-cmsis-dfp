/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     sys_ctrl_gpio.h
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     24-04-2023
 * @brief    GPIO system control Specific Header file.
 ******************************************************************************/

#ifndef SYS_CTRL_GPIO_H_
#define SYS_CTRL_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "soc.h"
#include "RTE_Device.h"

/* CLKCTL_PER_SLV GPIO_CTRLn field definitions */
#define GPIO_CTRL_DB_CKEN (1U << 12U) /* GPIO Debounce clock enable */
#if SOC_FEAT_GPIO_HAS_CLOCK_ENABLE
#define GPIO_CTRL_CKEN (1U << 16U) /* GPIO clock enable */
#endif

#if RTE_GPIO16
#define GPIO16_CKEN    (1U << 10U)
#define GPIO16_DB_CKEN (1U << 8U)
#endif
#if RTE_GPIO17
#define GPIO17_CKEN    (1U << 11U)
#define GPIO17_DB_CKEN (1U << 9U)
#endif

#if SOC_FEAT_GPIO_HAS_CLOCK_ENABLE
/**
  \fn          static void inline enable_gpio_clk (uint8_t instance)
  \brief       Enable GPIO clock from EXPMST0.
  \param       instance     instance number
  \return      none
*/
static inline void enable_gpio_clk(GPIO_INSTANCE instance)
{
    switch (instance) {
#if RTE_GPIO16
    case GPIO16_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN |= GPIO16_CKEN;
        break;
#endif
#if RTE_GPIO17
    case GPIO17_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN |= GPIO17_CKEN;
        break;
#endif
    default:
        CLKCTL_PER_SLV->GPIO_CTRL[instance] |= GPIO_CTRL_CKEN;
        break;
    }
}

/**
  \fn          static inline void disable_gpio_clk (uint8_t instance)
  \brief       Disable GPIO clock from EXPMST0.
  \param       instance     instance number
  \return      none
*/
static inline void disable_gpio_clk(GPIO_INSTANCE instance)
{
    switch (instance) {
#if RTE_GPIO16
    case GPIO16_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN &= ~GPIO16_CKEN;
        break;
#endif
#if RTE_GPIO17
    case GPIO17_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN &= ~GPIO17_CKEN;
        break;
#endif
    default:
        CLKCTL_PER_SLV->GPIO_CTRL[instance] &= ~GPIO_CTRL_CKEN;
        break;
    }
}
#endif

/**
  \fn          static void inline enable_gpio_debounce_clk (uint8_t instance)
  \brief       Enable GPIO Debounce clock from EXPMST0.
  \param       instance     instance number
  \return      none
*/
static inline void enable_gpio_debounce_clk(GPIO_INSTANCE instance)
{
    switch (instance) {
#if RTE_GPIO16
    case GPIO16_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN |= GPIO16_DB_CKEN;
        break;
#endif
#if RTE_GPIO17
    case GPIO17_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN |= GPIO17_DB_CKEN;
        break;
#endif
    default:
        CLKCTL_PER_SLV->GPIO_CTRL[instance] |= GPIO_CTRL_DB_CKEN;
        break;
    }
}

/**
  \fn          static inline void disable_gpio_debounce_clk (uint8_t instance)
  \brief       Disable GPIO Debounce clock from EXPMST0.
  \param       instance     instance number
  \return      none
*/
static inline void disable_gpio_debounce_clk(GPIO_INSTANCE instance)
{
    switch (instance) {
#if RTE_GPIO16
    case GPIO16_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN &= ~GPIO16_DB_CKEN;
        break;
#endif
#if RTE_GPIO17
    case GPIO17_INSTANCE:
        AON->RTSS_HE_LPUART_CKEN &= ~GPIO17_DB_CKEN;
        break;
#endif
    default:
        CLKCTL_PER_SLV->GPIO_CTRL[instance] &= ~GPIO_CTRL_DB_CKEN;
        break;
    }
}

/**
  \fn          static inline void set_gpio_debounce_clkdiv (uint16_t clk_div, uint8_t instance)
  \brief       Set GPIO Debounce clock divider.
  \param       clk_div     debounce clock divider
  \param       instance    instance number
  \return      none
*/
static inline void set_gpio_debounce_clkdiv(uint16_t clk_div, GPIO_INSTANCE instance)
{
    if ((instance != GPIO16_INSTANCE) && (instance != GPIO17_INSTANCE)) {
        CLKCTL_PER_SLV->GPIO_CTRL[instance] |= clk_div;
    }
}

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_GPIO_H_ */
