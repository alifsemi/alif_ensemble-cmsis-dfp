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
 * @file     : demo_lvgl.c
 * @author   : Ahmad Rashed
 * @email    : ahmad.rashed@alifsemi.com
 * @version  : V1.0.0
 * @date     : 28-March-2022
 * @brief    : lvgl application code
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/

#include "lvgl.h"
#include "demos/lv_demos.h"

#include <RTE_Components.h>
#include CMSIS_device_header

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "Driver_Common.h"
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

extern void lv_port_disp_init(void);

volatile uint32_t ms_ticks = 0;
void              SysTick_Handler(void)
{
    ms_ticks++;
    lv_tick_inc(1);
}
void delay(uint32_t nticks)
{
    nticks += ms_ticks;
    while (ms_ticks < nticks) {
    }
}
#define TICKS_PER_SECOND 1000

/* Define main entry point.  */
int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif
    SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);

    lv_port_disp_init();

#if LV_USE_DEMO_WIDGETS
    lv_demo_widgets();
#endif

#if LV_USE_DEMO_BENCHMARK
    lv_demo_benchmark();
#endif

    while (1) {
        lv_task_handler();
    }

    return 0;
}
