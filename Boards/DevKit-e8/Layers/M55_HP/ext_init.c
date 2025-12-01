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

#include "RTE_Components.h"
#include CMSIS_device_header

#include "ext_init.h"
#include "pinconf.h"
#include "board_defs.h"

#include "Driver_IO.h"

/* RTL8201 reset pin, GPIO port 11 */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(11);
static ARM_DRIVER_GPIO *DRV_GPIO11 = &ARM_Driver_GPIO_(11);

/*
  Initialize reset and power I/Os to the external devices.
*/
void ext_init(void)
{
    /* Re-configure ETH_RST_B pin as GPIO11_6 */
    pinconf_set(PORT_11, PIN_6, PINMUX_ALTERNATE_FUNCTION_0, PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

    /* Manually release ETH PYR from reset using ETH_RST_B */
    DRV_GPIO11->Initialize(6, NULL);
    DRV_GPIO11->PowerControl(6, ARM_POWER_FULL);
    DRV_GPIO11->SetValue(6, GPIO_PIN_OUTPUT_STATE_HIGH);
    DRV_GPIO11->SetDirection(6, GPIO_PIN_DIRECTION_OUTPUT);
}
