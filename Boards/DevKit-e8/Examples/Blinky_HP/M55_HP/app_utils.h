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
 * @file     app_utils.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @brief    Common Header File for all testapps
 * @version  V1.0.0
 * @date     22-Jul-2025
 * @bug      None
 * @Note     None
 ******************************************************************************/
#ifndef APP_UTILS_H
#define APP_UTILS_H

#include "sys_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WAIT_FOREVER_LOOP               while (1) {\
                                        }

#define ASSERT_HANG_LOOP                for (;;) {\
                                        }

#define APP_ERROR                       (-1)
#define APP_SUCCESS                     (0)


#ifdef __cplusplus
}
#endif

#endif /* SYS_UTILS_H */
