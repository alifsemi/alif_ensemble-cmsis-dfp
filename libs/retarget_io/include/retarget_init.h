/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     retarget_init.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     07-Jun-2025
 * @brief    retargetting initialization header file
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef RETARGET_STD_IN_OUT_INIT_H__
#define RETARGET_STD_IN_OUT_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(RTE_CMSIS_Compiler_STDIN)
/**
  Initialize stdin

  \return          0 on success, or -1 on error.
*/
int stdin_init(void);

#endif /* RTE_CMSIS_Compiler_STDIN */

#if defined(RTE_CMSIS_Compiler_STDOUT)
/**
  Initialize stdout

  \return          0 on success, or -1 on error.
*/
int stdout_init(void);

#endif /* RTE_CMSIS_Compiler_STDOUT */

#if defined(RTE_CMSIS_Compiler_STDERR)
/**
  Initialize stderr

  \return          0 on success, or -1 on error.
*/
int stderr_init(void);

#endif /* RTE_CMSIS_Compiler_STDERR */

#ifdef __cplusplus
}
#endif

#endif /* RETARGET_STD_IN_OUT_INIT_H__ */
