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

#include "Driver_IO.h"

/* MT9M114 Camera reset GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(RTE_MT9M114_CAMERA_SENSOR_MIPI_RESET_GPIO_PORT);
static ARM_DRIVER_GPIO *IO_Driver_CAM_RST =
    &ARM_Driver_GPIO_(RTE_MT9M114_CAMERA_SENSOR_MIPI_RESET_GPIO_PORT);

/* MT9M114 Camera power GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(RTE_MT9M114_CAMERA_SENSOR_MIPI_POWER_GPIO_PORT);
static ARM_DRIVER_GPIO *IO_Driver_CAM_PWR =
    &ARM_Driver_GPIO_(RTE_MT9M114_CAMERA_SENSOR_MIPI_POWER_GPIO_PORT);

/* GT911 Touch Controller reset port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(RTE_GT911_TOUCH_RESET_GPIO_PORT);
static ARM_DRIVER_GPIO *IO_Driver_TOUCH_RST = &ARM_Driver_GPIO_(RTE_GT911_TOUCH_RESET_GPIO_PORT);

/* GLCD reset port */
#define BOARD_LCD_RESET_GPIO_PORT LP
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(BOARD_LCD_RESET_GPIO_PORT);
static ARM_DRIVER_GPIO *IO_Driver_LCD_RST = &ARM_Driver_GPIO_(BOARD_LCD_RESET_GPIO_PORT);

/*
  Initialize reset and power I/Os to the external devices.
*/
void ext_init(void)
{
    IO_Driver_CAM_PWR->Initialize(5, NULL);
    IO_Driver_CAM_PWR->PowerControl(5, ARM_POWER_FULL);
    IO_Driver_CAM_PWR->SetValue(5, GPIO_PIN_OUTPUT_STATE_LOW);
    IO_Driver_CAM_PWR->SetDirection(5, GPIO_PIN_DIRECTION_OUTPUT);

    IO_Driver_CAM_RST->Initialize(1, NULL);
    IO_Driver_CAM_RST->PowerControl(1, ARM_POWER_FULL);
    IO_Driver_CAM_RST->SetValue(1, GPIO_PIN_OUTPUT_STATE_LOW);
    IO_Driver_CAM_RST->SetDirection(1, GPIO_PIN_DIRECTION_OUTPUT);

    IO_Driver_TOUCH_RST->Initialize(RTE_GT911_TOUCH_RESET_PIN_NO, NULL);
    IO_Driver_TOUCH_RST->PowerControl(RTE_GT911_TOUCH_RESET_PIN_NO, ARM_POWER_FULL);
    IO_Driver_TOUCH_RST->SetValue(RTE_GT911_TOUCH_RESET_PIN_NO, GPIO_PIN_OUTPUT_STATE_LOW);
    IO_Driver_TOUCH_RST->SetDirection(RTE_GT911_TOUCH_RESET_PIN_NO, GPIO_PIN_DIRECTION_OUTPUT);

    IO_Driver_LCD_RST->Initialize(RTE_ILI9488_PANEL_BL_LED_PIN_NO, NULL);
    IO_Driver_LCD_RST->PowerControl(RTE_ILI9488_PANEL_BL_LED_PIN_NO, ARM_POWER_FULL);
    IO_Driver_LCD_RST->SetValue(RTE_ILI9488_PANEL_BL_LED_PIN_NO, GPIO_PIN_OUTPUT_STATE_LOW);
    IO_Driver_LCD_RST->SetDirection(RTE_ILI9488_PANEL_BL_LED_PIN_NO, GPIO_PIN_DIRECTION_OUTPUT);
}
