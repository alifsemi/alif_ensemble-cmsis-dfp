/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     sys_ctrl_i2c.h
 * @author   Shreehari H K
 * @email    shreehari.hk@alifsemi.com
 * @version  V1.0.0
 * @date     21-Apr-2025
 * @brief    System control header file for i2c
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef SYS_CTRL_I2C_H
#define SYS_CTRL_I2C_H

#include <stdint.h>
#include "sys_clocks.h"

#define HE_CLK_ENA_I2C_CKEN (1 << 13) /* Clock enable bit for LPI2C1 */

/**
 * enum I2C_INSTANCE
 * I2C instances
 */
typedef enum _I2C_INSTANCE {
    I2C_INSTANCE_0,   /**< I2C instance - 0       */
    I2C_INSTANCE_1,   /**< I2C instance - 1       */
    I2C_INSTANCE_2,   /**< I2C instance - 2       */
    I2C_INSTANCE_3,   /**< I2C instance - 3       */
    I2C_INSTANCE_LP_1 /**< I2C instance - LP - 1  */
} I2C_INSTANCE;

/**
  \fn          static inline void enable_i2c_clock(const I2C_INSTANCE inst)
  \brief       Enables I2C clock
  \param       inst : I2C instance number
  \return      none
*/
static inline void enable_i2c_clock(const I2C_INSTANCE inst)
{
    if (inst == I2C_INSTANCE_LP_1) {
        M55HE_CFG->HE_CLK_ENA |= HE_CLK_ENA_I2C_CKEN;
    }
}

/**
  \fn          static inline void disable_i2c_clock(const I2C_INSTANCE inst)
  \brief       Disables I2C clock
  \param       inst : I2C instance number
  \return      none
*/
static inline void disable_i2c_clock(const I2C_INSTANCE inst)
{
    if (inst == I2C_INSTANCE_LP_1) {
        M55HE_CFG->HE_CLK_ENA &= ~HE_CLK_ENA_I2C_CKEN;
    }
}

/**
  \fn          static inline uint32_t get_i2c_core_clock(void)
  \brief       Gets I2C input core clock
  \return      Input Core clock value
*/
static inline uint32_t get_i2c_core_clock(void)
{
    /* Gets the system APB clock */
    return GetSystemAPBClock();
}

#endif /* SYS_CTRL_I2C_H */
