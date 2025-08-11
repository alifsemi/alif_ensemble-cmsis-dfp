/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     sys_ctrl_canfd.h
 * @author   Shreehari H K
 * @email    shreehari.hk@alifsemi.com
 * @version  V1.0.0
 * @date     05-July-2023
 * @brief    System control header file for canfd
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef SYS_CTRL_CANFD_H_
#define SYS_CTRL_CANFD_H_

#include <stdbool.h>
#include "RTE_Device.h"
#include "soc.h"
#include "soc_features.h"

#define CANFD_CLK_SRC_38P4MHZ_CLK 38400000U                       /* 38.4 MHz */
#define CANFD_CLK_SRC_160MHZ_CLK  160000000U                      /* 160 MHz  */
#define CANFD_MAX_CLK_SPEED       (CANFD_CLK_SRC_160MHZ_CLK / 2U) /* 80 MHz   */

/* CLKCTL_PER_SLV CANFD_CTRL CANFD Control field definitions */
#if SOC_FEAT_CANFD0_CANFD1_CTRL
#define CANFD1_CTRL_FD_ENA      (1U << 26U) /* CANFD1 FD Enable                        */
#define CANFD1_CTRL_CLK_SEL_Pos (25U)       /* CANFD1 Clock Selection                  */
#define CANFD1_CTRL_CKEN        (1U << 24U) /* CANFD1 Clock Enable                     */
#define CANFD1_CTRL_CKDIV_Pos   (16U)       /* CANFD1 Clock Divisor position           */

#define CANFD0_CTRL_FD_ENA      (1U << 10U) /* CANFD0 FD Enable                        */
#define CANFD0_CTRL_CLK_SEL_Pos (9U)        /* CANFD0 Clock Selection                  */
#define CANFD0_CTRL_CKEN        (1U << 8U)  /* CANFD0 Clock Enable                     */
#define CANFD0_CTRL_CKDIV_Pos   (0U)        /* CANFD0 Clock Divisor position           */

#else
#define CANFD0_CTRL_FD_ENA      (1U << 20U) /* CANFD0 FD Enable                        */
#define CANFD0_CTRL_CLK_SEL_Pos (16U)       /* CANFD0 Clock Selection                  */
#define CANFD0_CTRL_CKEN        (1U << 12U) /* CANFD0 Clock Enable                     */
#define CANFD0_CTRL_CKDIV_Pos   (0U)        /* CANFD0 Clock Divisor position           */
#endif

typedef enum _CANFD_INSTANCE {
    CANFD_INSTANCE_0, /**< CANFD instance - 0   */
    CANFD_INSTANCE_1  /**< CANFD instance - 1   */
} CANFD_INSTANCE;

/**
  \fn          static inline void canfd_clock_enable(const CANFD_INSTANCE instance,
  \                                                  const bool clk_sel,
  \                                                  const uint8_t clk_div)
  \brief       Enables CANFD clock
  \param[in]   instance : CANFD instance
  \param[in]   clk_sel  : Clock selection (160 MHz / 38.4 MHz)
  \param[in]   clk_div  : clock divider value
  \return      none
*/
static inline void canfd_clock_enable(const CANFD_INSTANCE instance, const bool clk_sel,
                                      const uint8_t clk_div)
{
    switch (instance) {
#if (RTE_CANFD0)
    case CANFD_INSTANCE_0:
        /* Enables clock for CANFD0 module */
        CLKCTL_PER_SLV->CANFD_CTRL = (CANFD0_CTRL_CKEN | (clk_sel << CANFD0_CTRL_CLK_SEL_Pos) |
                                      (clk_div << CANFD0_CTRL_CKDIV_Pos));
        break;
#endif  // RTE_CANFD0

#if (RTE_CANFD1)
    case CANFD_INSTANCE_1:
        /* Enables clock for CANFD1 module */
        CLKCTL_PER_SLV->CANFD_CTRL = (CANFD1_CTRL_CKEN | (clk_sel << CANFD1_CTRL_CLK_SEL_Pos) |
                                      (clk_div << CANFD1_CTRL_CKDIV_Pos));
        break;
#endif  // RTE_CANFD1

    default:
        break;
    }
}

/**
  \fn          static inline void canfd_clock_disable(const CANFD_INSTANCE instance)
  \brief       Disables CANFD clock
  \param[in]   instance : CANFD instance
  \return      none
*/
static inline void canfd_clock_disable(const CANFD_INSTANCE instance)
{
    switch (instance) {
#if (RTE_CANFD0)
    case CANFD_INSTANCE_0:
        /* Disables clock for CANFD0 module */
        CLKCTL_PER_SLV->CANFD_CTRL &= ~CANFD0_CTRL_CKEN;
        break;
#endif  // RTE_CANFD0

#if (RTE_CANFD1)
    case CANFD_INSTANCE_1:
        /* Disables clock for CANFD1 module */
        CLKCTL_PER_SLV->CANFD_CTRL &= ~CANFD1_CTRL_CKEN;
        break;
#endif  // RTE_CANFD1

    default:
        break;
    }
}

/**
  \fn          static inline void canfd_setup_fd_mode(const CANFD_INSTANCE instance,
  \                                                   const bool enable)
  \brief       enable/disables CANFD Fast data mode
  \param[in]   instance : CANFD instance
  \param[in]   enable   : Command to enable/disable for Fast data mode
  \return      none
*/
static inline void canfd_setup_fd_mode(const CANFD_INSTANCE instance, const bool enable)
{
    switch (instance) {
#if (RTE_CANFD0)
    case CANFD_INSTANCE_0:
        if (enable) {
            CLKCTL_PER_SLV->CANFD_CTRL |= CANFD0_CTRL_FD_ENA;
        } else {
            CLKCTL_PER_SLV->CANFD_CTRL &= ~CANFD0_CTRL_FD_ENA;
        }
        break;
#endif  // RTE_CANFD0

#if (RTE_CANFD1)
    case CANFD_INSTANCE_1:
        if (enable) {
            CLKCTL_PER_SLV->CANFD_CTRL |= CANFD1_CTRL_FD_ENA;
        } else {
            CLKCTL_PER_SLV->CANFD_CTRL &= ~CANFD1_CTRL_FD_ENA;
        }
        break;
#endif  // RTE_CANFD1

    default:
        break;
    }
}

/**
  \fn          static inline bool canfd_in_fd_mode (const CANFD_INSTANCE instance)
  \brief       returns canfd mode (FD / Classic 2.0)
  \param[in]   instance : CANFD instance
  \return      true - In FD mode/ false - Classic 2.0 mode)
*/
static inline bool canfd_in_fd_mode(const CANFD_INSTANCE instance)
{
    switch (instance) {
#if (RTE_CANFD0)
    case CANFD_INSTANCE_0:
        /* Returns fd mode status of CANFD0 */
        return (CLKCTL_PER_SLV->CANFD_CTRL & (CANFD0_CTRL_FD_ENA));
#endif  // RTE_CANFD0

#if (RTE_CANFD1)
    case CANFD_INSTANCE_1:
        /* Returns fd mode status of CANFD0 */
        return (CLKCTL_PER_SLV->CANFD_CTRL & (CANFD1_CTRL_FD_ENA));
#endif  // RTE_CANFD1

    default:
        break;
    }
    return false;
}

#endif /* SYS_CTRL_CANFD_H_ */
