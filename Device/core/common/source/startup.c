/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "soc.h"
#include "system.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

extern __NO_RETURN void __PROGRAM_START(void);

#if defined(__clang__) && !defined(__ARMCC_VERSION)

#undef __PROGRAM_START
#define __PROGRAM_START __clang_copy_zero_init

#endif

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__attribute__((naked)) __NO_RETURN void Reset_Handler(void)
{
    /* Set up the main stack */

    /*
     * Function must be naked to ensure the compiler doesn't use the
     * stack on entry.
     *
     * Only basic asm (no parameters) is permitted for naked functions,
     * so we have to get the values in by text substitution.
     */
#define xstr(s) str(s)
#define str(s)  #s
    __asm("LDR     R0, =" xstr(__STACK_LIMIT) "\n\t"
                                              "LDR     R1, =" xstr(
                                                  __INITIAL_SP) "\n\t"
                                                                "MSR     MSPLIM, R0\n\t"
                                                                "MSR     MSP, R1\n\t"
                                                                "BL      Reset_Handler_C");
#undef xstr
#undef str
}

__attribute__((used)) __NO_RETURN void Reset_Handler_C(void)
{
    SystemInit();      /* CMSIS System Initialization */
    __PROGRAM_START(); /* Enter PreMain (C library entry point) */
}
