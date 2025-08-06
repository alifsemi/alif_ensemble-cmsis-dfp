/*---------------------------------------------------------------------------
 * Copyright (c) 2025 Arm Limited (or its affiliates). All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *---------------------------------------------------------------------------*/

#include <stdio.h>

#include "main.h"
#include "cmsis_os2.h"
#include "cmsis_vio.h"

/* Thread attributes for the app_main thread */
osThreadAttr_t attr_app_main = {
    .name = "app_main",
};

/* Loop counter */
uint32_t core1_loop_counter = 0xFFFFFFFFU;

/*
  Application main thread.
*/
__NO_RETURN void app_main_thread(void *argument)
{
    (void) argument;

    /* Initialize loop counter */
    core1_loop_counter = 0;

    while (1) {
        /* Switch LED1 on */
        vioSetSignal(vioLED1, vioLEDon);
        /* Wait a bit */
        osDelay(500);
        /* Switch LED1 off */
        vioSetSignal(vioLED1, vioLEDoff);
        /* Wait a bit */
        osDelay(500);

        /* Increment loop counter */
        core1_loop_counter += 1;
    }
}

/*
  Application initialization.
*/
int app_main(void)
{
    osKernelInitialize(); /* Initialize CMSIS-RTOS2 */
    osThreadNew(app_main_thread, NULL, &attr_app_main);
    osKernelStart(); /* Start thread execution */
    return 0;
}
