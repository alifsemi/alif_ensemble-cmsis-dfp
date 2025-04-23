/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     sys_ctrl_usb.h
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     25-Apr-2023
 * @brief    System Control Device information for USB
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef SYS_CTRL_USB_H

#define SYS_CTRL_USB_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "peripheral_types.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define USB_CTRL2_PHY_POR  (1U << 8)

static inline void usb_ctrl2_phy_power_on_reset_set()
{
    CLKCTL_PER_MST -> USB_CTRL2 |= USB_CTRL2_PHY_POR; 
}


static inline void usb_ctrl2_phy_power_on_reset_clear()
{
    CLKCTL_PER_MST -> USB_CTRL2 &= ~USB_CTRL2_PHY_POR; 
}

#ifdef  __cplusplus
}
#endif
#endif /* SYS_CTRL_USB_H */