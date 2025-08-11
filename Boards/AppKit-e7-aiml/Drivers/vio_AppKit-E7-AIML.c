/******************************************************************************
 * @file     vio_AppKit-E7-AIML.c
 * @brief    Virtual I/O implementation for Alif Semiconductor Ensemble AppKit-E7-AIML
 * @version  V1.0.0
 * @date     11. April 2025
 ******************************************************************************/
/*
 * Copyright (c) 2025 Arm Limited (or its affiliates).
 * All rights reserved.
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
 */

/*! \page vio_AppKit Physical I/O Mapping

The table below lists the physical I/O mapping of this CMSIS-Driver VIO implementation.

Virtual Resource  | Variable       | Physical Resource on Ensemble AppKit-E7-AIML   |
:-----------------|:---------------|:-----------------------------------------------|
vioBUTTON0        | vioSignalIn.0  | Joystick Select Button                         |
vioJOYup          | vioSignalIn.2  | Joystick Up                                    |
vioJOYdown        | vioSignalIn.3  | Joystick Down                                  |
vioJOYleft        | vioSignalIn.4  | Joystick Left                                  |
vioJOYright       | vioSignalIn.5  | Joystick Right                                 |
vioJOYselect      | vioSignalIn.6  | Joystick Select Button                         |
vioLED0           | vioSignalOut.0 | RGB LED Red                                    |
vioLED1           | vioSignalOut.1 | RGB LED Green                                  |
vioLED2           | vioSignalOut.2 | RGB LED Blue                                   |
*/

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include <string.h>
#include "cmsis_vio.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#if !defined CMSIS_VOUT || !defined CMSIS_VIN
#include "Driver_GPIO.h"
#include "Driver_GPIO_Private.h"

/* Definitions from board_defs.h                                                          */
/* Ideally board_defs.h would be included here and pack component would provide the file. */
// #include "board_defs.h"

#define BOARD_LEDRGB0_R_GPIO_PORT     12
#define BOARD_LEDRGB0_R_GPIO_PIN      3

#define BOARD_LEDRGB0_G_GPIO_PORT     7
#define BOARD_LEDRGB0_G_GPIO_PIN      4

#define BOARD_LEDRGB0_B_GPIO_PORT     12
#define BOARD_LEDRGB0_B_GPIO_PIN      0

#define BOARD_JOY_SW_A_GPIO_PORT      15
#define BOARD_JOY_SW_A_GPIO_PIN       0

#define BOARD_JOY_SW_B_GPIO_PORT      15
#define BOARD_JOY_SW_B_GPIO_PIN       1

#define BOARD_JOY_SW_C_GPIO_PORT      15
#define BOARD_JOY_SW_C_GPIO_PIN       2

#define BOARD_JOY_SW_D_GPIO_PORT      15
#define BOARD_JOY_SW_D_GPIO_PIN       3

#define BOARD_JOY_SW_CENTER_GPIO_PORT 15
#define BOARD_JOY_SW_CENTER_GPIO_PIN  4

#endif

// VIO input, output definitions
#define VIO_VALUE_NUM 3U  // Number of values

// VIO input, output variables
__USED uint32_t vioSignalIn;              // Memory for incoming signal
__USED uint32_t vioSignalOut;             // Memory for outgoing signal
__USED int32_t  vioValue[VIO_VALUE_NUM];  // Memory for value used in vioGetValue/vioSetValue

#if !defined CMSIS_VOUT
#define LED_0_PIN GPIO_PIN(BOARD_LEDRGB0_R_GPIO_PORT, BOARD_LEDRGB0_R_GPIO_PIN)
#define LED_1_PIN GPIO_PIN(BOARD_LEDRGB0_G_GPIO_PORT, BOARD_LEDRGB0_G_GPIO_PIN)
#define LED_2_PIN GPIO_PIN(BOARD_LEDRGB0_B_GPIO_PORT, BOARD_LEDRGB0_B_GPIO_PIN)
#endif

#if !defined CMSIS_VIN
#define JOY_UP_PIN GPIO_PIN(BOARD_JOY_SW_A_GPIO_PORT, BOARD_JOY_SW_A_GPIO_PIN) /* RIGHT - 45 DEG   \
                                                                                */
#define JOY_DOWN_PIN                                                                               \
    GPIO_PIN(BOARD_JOY_SW_B_GPIO_PORT, BOARD_JOY_SW_B_GPIO_PIN) /* LEFT  - 45 DEG */
#define JOY_LEFT_PIN                                                                               \
    GPIO_PIN(BOARD_JOY_SW_C_GPIO_PORT, BOARD_JOY_SW_C_GPIO_PIN) /* RIGHT + 45 DEG */
#define JOY_RIGHT_PIN                                                                              \
    GPIO_PIN(BOARD_JOY_SW_D_GPIO_PORT, BOARD_JOY_SW_D_GPIO_PIN) /* LEFT  + 45 DEG */
#define JOY_SELECT_PIN                                                                             \
    GPIO_PIN(BOARD_JOY_SW_CENTER_GPIO_PORT, BOARD_JOY_SW_CENTER_GPIO_PIN) /* SELECT/CENTER */

/* There is no physical button available, vioBUTTON0 is mapped to Joystick Select/Center */
#define BUTTON_0_PIN GPIO_PIN(BOARD_JOY_SW_CENTER_GPIO_PORT, BOARD_JOY_SW_CENTER_GPIO_PIN)
#endif

// Initialize test input, output.
void vioInit(void)
{
#if !defined(CMSIS_VOUT) || !defined(CMSIS_VIN)
    ARM_DRIVER_GPIO *gpio = &Driver_GPIO;
#endif

    vioSignalIn  = 0U;
    vioSignalOut = 0U;

    memset(vioValue, 0, sizeof(vioValue));

#if !defined CMSIS_VOUT
    gpio->Setup(LED_0_PIN, NULL);
    gpio->Setup(LED_1_PIN, NULL);
    gpio->Setup(LED_2_PIN, NULL);

    gpio->SetDirection(LED_0_PIN, ARM_GPIO_OUTPUT);
    gpio->SetDirection(LED_1_PIN, ARM_GPIO_OUTPUT);
    gpio->SetDirection(LED_2_PIN, ARM_GPIO_OUTPUT);

    // Turn off all LEDs
    gpio->SetOutput(LED_0_PIN, 0U);
    gpio->SetOutput(LED_1_PIN, 0U);
    gpio->SetOutput(LED_2_PIN, 0U);
#endif

#if !defined CMSIS_VIN
#if 0 /* Reconfiguring pin causes issue - disable setup */
  gpio->Setup(JOY_UP_PIN, NULL);
  gpio->Setup(JOY_DOWN_PIN, NULL);
  gpio->Setup(JOY_LEFT_PIN, NULL);
  gpio->Setup(JOY_RIGHT_PIN, NULL);
  gpio->Setup(JOY_SELECT_PIN, NULL);
#endif
    gpio->SetDirection(JOY_UP_PIN, ARM_GPIO_INPUT);
    gpio->SetDirection(JOY_DOWN_PIN, ARM_GPIO_INPUT);
    gpio->SetDirection(JOY_LEFT_PIN, ARM_GPIO_INPUT);
    gpio->SetDirection(JOY_RIGHT_PIN, ARM_GPIO_INPUT);
    gpio->SetDirection(JOY_SELECT_PIN, ARM_GPIO_INPUT);

    gpio->SetPullResistor(JOY_UP_PIN, ARM_GPIO_PULL_UP);
    gpio->SetPullResistor(JOY_DOWN_PIN, ARM_GPIO_PULL_UP);
    gpio->SetPullResistor(JOY_LEFT_PIN, ARM_GPIO_PULL_UP);
    gpio->SetPullResistor(JOY_RIGHT_PIN, ARM_GPIO_PULL_UP);
    gpio->SetPullResistor(JOY_SELECT_PIN, ARM_GPIO_PULL_UP);
#endif
}

// Set signal output.
void vioSetSignal(uint32_t mask, uint32_t signal)
{
#if !defined CMSIS_VOUT
    ARM_DRIVER_GPIO *gpio = &Driver_GPIO;
#endif

    vioSignalOut &= ~mask;
    vioSignalOut |= mask & signal;

#if !defined CMSIS_VOUT
    // Output signals to LEDs
    if ((mask & vioLED0) != 0U) {
        if ((signal & vioLED0) != 0U) {
            gpio->SetOutput(LED_0_PIN, 1U);
        } else {
            gpio->SetOutput(LED_0_PIN, 0U);
        }
    }

    if ((mask & vioLED1) != 0U) {
        if ((signal & vioLED1) != 0U) {
            gpio->SetOutput(LED_1_PIN, 1U);
        } else {
            gpio->SetOutput(LED_1_PIN, 0U);
        }
    }

    if ((mask & vioLED2) != 0U) {
        if ((signal & vioLED2) != 0U) {
            gpio->SetOutput(LED_2_PIN, 1U);
        } else {
            gpio->SetOutput(LED_2_PIN, 0U);
        }
    }
#endif
}

// Get signal input.
uint32_t vioGetSignal(uint32_t mask)
{
    uint32_t signal;
#if !defined CMSIS_VIN
    ARM_DRIVER_GPIO *gpio = &Driver_GPIO;
    uint32_t         val;
#endif

#if !defined CMSIS_VIN
    // Get input signals from buttons and joystick
    if ((mask & vioBUTTON0) != 0U) {
        val = gpio->GetInput(BUTTON_0_PIN);
        if (val == 0U) {
            vioSignalIn |= vioBUTTON0;
        } else {
            vioSignalIn &= ~vioBUTTON0;
        }
    }
    if ((mask & vioJOYup) != 0U) {
        val = gpio->GetInput(JOY_UP_PIN);
        if (val == 0U) {
            vioSignalIn |= vioJOYup;
        } else {
            vioSignalIn &= ~vioJOYup;
        }
    }
    if ((mask & vioJOYdown) != 0U) {
        val = gpio->GetInput(JOY_DOWN_PIN);
        if (val == 0U) {
            vioSignalIn |= vioJOYdown;
        } else {
            vioSignalIn &= ~vioJOYdown;
        }
    }
    if ((mask & vioJOYleft) != 0U) {
        val = gpio->GetInput(JOY_LEFT_PIN);
        if (val == 0U) {
            vioSignalIn |= vioJOYleft;
        } else {
            vioSignalIn &= ~vioJOYleft;
        }
    }
    if ((mask & vioJOYright) != 0U) {
        val = gpio->GetInput(JOY_RIGHT_PIN);
        if (val == 0U) {
            vioSignalIn |= vioJOYright;
        } else {
            vioSignalIn &= ~vioJOYright;
        }
    }
    if ((mask & vioJOYselect) != 0U) {
        val = gpio->GetInput(JOY_SELECT_PIN);
        if (val == 0U) {
            vioSignalIn |= vioJOYselect;
        } else {
            vioSignalIn &= ~vioJOYselect;
        }
    }
#endif

    signal = vioSignalIn & mask;

    return signal;
}

// Set value output.
void vioSetValue(uint32_t id, int32_t value)
{
    uint32_t index = id;
#if !defined CMSIS_VOUT
    // Add user variables here:

#endif

    if (index >= VIO_VALUE_NUM) {
        return; /* return in case of out-of-range index */
    }

    vioValue[index] = value;

#if !defined CMSIS_VOUT
    // Add user code here:

#endif
}

// Get value input.
int32_t vioGetValue(uint32_t id)
{
    uint32_t index = id;
    int32_t  value = 0;
#if !defined CMSIS_VIN
    // Add user variables here:

#endif

    if (index >= VIO_VALUE_NUM) {
        return value; /* return default in case of out-of-range index */
    }

#if !defined CMSIS_VIN
// Add user code here:

//   vioValue[index] = ...;
#endif

    value = vioValue[index];

    return value;
}
