/*
 * Copyright (c) 2024 ARM Limited. All rights reserved.
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
 *
 * $Date:        16. July 2024
 * $Revision:    V1.0
 *
 * Project:      GPIO Driver for Alif Semiconductor
 */

#include "Driver_GPIO_Private.h"

#include "pinconf.h"
#include "gpio.h"
#include "soc_features.h"
#include "RTE_Device.h"

#if defined(RTE_Drivers_GPIO)

/* Pin mapping for Ensemble Ex family
   0 ..   7: P0_0..7  (GPIO0)
   8 ..  15: P1_0..7  (GPIO1)
  16 ..  23: P2_0..7  (GPIO2)
  24 ..  31: P3_0..7  (GPIO3)
  32 ..  39: P4_0..7  (GPIO4)
  40 ..  47: P5_0..7  (GPIO5)
  48 ..  55: P6_0..7  (GPIO6)
  56 ..  63: P7_0..7  (GPIO7)
  64 ..  71: P8_0..7  (GPIO8)
  72 ..  79: P9_0..7  (GPIO9)
  80 ..  87: P10_0..7 (GPIO10)
  88 ..  95: P11_0..7 (GPIO11)
  96 .. 103: P12_0..7 (GPIO12)
 104 .. 111: P13_0..7 (GPIO13)
 112 .. 119: P14_0..7 (GPIO14)
 120 .. 127: P15_0..7 (LPGPIO) */

/* Pin mapping for Ensemble E1C family
   0 ..   7: P0_0..7  (GPIO0)
   8 ..  15: P1_0..7  (GPIO1)
  16 ..  23: P2_0..7  (GPIO2)
  24 ..  31: P3_0..7  (GPIO3)
  32 ..  39: P4_0..7  (GPIO4)
  40 ..  47: P5_0..7  (GPIO5)
  48 ..  55: P6_0..7  (GPIO6)
  56 ..  63: P7_0..7  (GPIO7)
  64 ..  71: P8_0..7  (GPIO8)
  72 ..  79: P9_0..7  (GPIO9)
 120 .. 121: P15_0..1 (LPGPIO) */

#define GPIO_MAX_PINS 8U
#if (SOC_FEAT_LPGPIO_HAS_PIN2_7)
#define LPGPIO_MAX_PINS 8U
#else
#define LPGPIO_MAX_PINS 2U
#endif

/* GPIO Resources struct */
typedef struct _GPIO_RESOURCES {
    const uint32_t         base;                    /**< GPIO PORT Base Address>**/
    const IRQn_Type        irqn;                    /**< GPIO PORT IRQ base Num>**/
    const uint8_t          max_pins;                /**< GPIO PORT Pin Num>**/
    ARM_GPIO_SignalEvent_t cb_event[GPIO_MAX_PINS]; /**< GPIO Signal Event callback functions>**/
} GPIO_RESOURCES;

/* GPIO_RESOURCES array for each GPIO port */
static GPIO_RESOURCES gpio_res[] = {
#if (RTE_GPIO0)
    {GPIO0_BASE, GPIO0_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO1)
    {GPIO1_BASE, GPIO1_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO2)
    {GPIO2_BASE, GPIO2_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO3)
    {GPIO3_BASE, GPIO3_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO4)
    {GPIO4_BASE, GPIO4_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO5)
    {GPIO5_BASE, GPIO5_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO6)
    {GPIO6_BASE, GPIO6_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO7)
    {GPIO7_BASE, GPIO7_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO8)
    {GPIO8_BASE, GPIO8_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO9)
    {GPIO9_BASE, GPIO9_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO10)
    {GPIO10_BASE, GPIO10_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO11)
    {GPIO11_BASE, GPIO11_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO12)
    {GPIO12_BASE, GPIO12_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO13)
    {GPIO13_BASE, GPIO13_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_GPIO14)
    {GPIO14_BASE, GPIO14_IRQ0_IRQn, GPIO_MAX_PINS, {NULL}},
#endif
#if (RTE_LPGPIO)
    {LPGPIO_BASE, LPGPIO_IRQ0_IRQn, LPGPIO_MAX_PINS, {NULL}},
#endif
};

#ifndef CMSIS_GPIO_DISABLE_EVENTS
/* Common GPIO IRQ Handler */
static void GPIO_IRQHandler(uint32_t gpio_num, uint32_t pin_num)
{
    GPIO_Type     *gpio = (GPIO_Type *) gpio_res[gpio_num].base;
    ARM_GPIO_Pin_t pin;
    uint32_t       event;

    /* Clear pin interrupt */
    gpio_interrupt_eoi(gpio, pin_num);

    if (gpio_res[gpio_num].cb_event[pin_num] != NULL) {
        pin = (8U * gpio_num) + pin_num;

        /* Determine interrupt polarity/edge */
        if (gpio->GPIO_INT_POLARITY & (1U << pin_num)) {
            /* Active-high polarity / Rising edge */
            event = ARM_GPIO_EVENT_RISING_EDGE;
        } else {
            event = ARM_GPIO_EVENT_FALLING_EDGE;
        }

        gpio_res[gpio_num].cb_event[pin_num](pin, event);
    }
}

#if (RTE_GPIO0)
void GPIO0_IRQ0Handler(void)
{
    GPIO_IRQHandler(0U, 0U);
}
void GPIO0_IRQ1Handler(void)
{
    GPIO_IRQHandler(0U, 1U);
}
void GPIO0_IRQ2Handler(void)
{
    GPIO_IRQHandler(0U, 2U);
}
void GPIO0_IRQ3Handler(void)
{
    GPIO_IRQHandler(0U, 3U);
}
void GPIO0_IRQ4Handler(void)
{
    GPIO_IRQHandler(0U, 4U);
}
void GPIO0_IRQ5Handler(void)
{
    GPIO_IRQHandler(0U, 5U);
}
void GPIO0_IRQ6Handler(void)
{
    GPIO_IRQHandler(0U, 6U);
}
void GPIO0_IRQ7Handler(void)
{
    GPIO_IRQHandler(0U, 7U);
}
#endif

#if (RTE_GPIO1)
void GPIO1_IRQ0Handler(void)
{
    GPIO_IRQHandler(1U, 0U);
}
void GPIO1_IRQ1Handler(void)
{
    GPIO_IRQHandler(1U, 1U);
}
void GPIO1_IRQ2Handler(void)
{
    GPIO_IRQHandler(1U, 2U);
}
void GPIO1_IRQ3Handler(void)
{
    GPIO_IRQHandler(1U, 3U);
}
void GPIO1_IRQ4Handler(void)
{
    GPIO_IRQHandler(1U, 4U);
}
void GPIO1_IRQ5Handler(void)
{
    GPIO_IRQHandler(1U, 5U);
}
void GPIO1_IRQ6Handler(void)
{
    GPIO_IRQHandler(1U, 6U);
}
void GPIO1_IRQ7Handler(void)
{
    GPIO_IRQHandler(1U, 7U);
}
#endif

#if (RTE_GPIO2)
void GPIO2_IRQ0Handler(void)
{
    GPIO_IRQHandler(2U, 0U);
}
void GPIO2_IRQ1Handler(void)
{
    GPIO_IRQHandler(2U, 1U);
}
void GPIO2_IRQ2Handler(void)
{
    GPIO_IRQHandler(2U, 2U);
}
void GPIO2_IRQ3Handler(void)
{
    GPIO_IRQHandler(2U, 3U);
}
void GPIO2_IRQ4Handler(void)
{
    GPIO_IRQHandler(2U, 4U);
}
void GPIO2_IRQ5Handler(void)
{
    GPIO_IRQHandler(2U, 5U);
}
void GPIO2_IRQ6Handler(void)
{
    GPIO_IRQHandler(2U, 6U);
}
void GPIO2_IRQ7Handler(void)
{
    GPIO_IRQHandler(2U, 7U);
}
#endif

#if (RTE_GPIO3)
void GPIO3_IRQ0Handler(void)
{
    GPIO_IRQHandler(3U, 0U);
}
void GPIO3_IRQ1Handler(void)
{
    GPIO_IRQHandler(3U, 1U);
}
void GPIO3_IRQ2Handler(void)
{
    GPIO_IRQHandler(3U, 2U);
}
void GPIO3_IRQ3Handler(void)
{
    GPIO_IRQHandler(3U, 3U);
}
void GPIO3_IRQ4Handler(void)
{
    GPIO_IRQHandler(3U, 4U);
}
void GPIO3_IRQ5Handler(void)
{
    GPIO_IRQHandler(3U, 5U);
}
void GPIO3_IRQ6Handler(void)
{
    GPIO_IRQHandler(3U, 6U);
}
void GPIO3_IRQ7Handler(void)
{
    GPIO_IRQHandler(3U, 7U);
}
#endif

#if (RTE_GPIO4)
void GPIO4_IRQ0Handler(void)
{
    GPIO_IRQHandler(4U, 0U);
}
void GPIO4_IRQ1Handler(void)
{
    GPIO_IRQHandler(4U, 1U);
}
void GPIO4_IRQ2Handler(void)
{
    GPIO_IRQHandler(4U, 2U);
}
void GPIO4_IRQ3Handler(void)
{
    GPIO_IRQHandler(4U, 3U);
}
void GPIO4_IRQ4Handler(void)
{
    GPIO_IRQHandler(4U, 4U);
}
void GPIO4_IRQ5Handler(void)
{
    GPIO_IRQHandler(4U, 5U);
}
void GPIO4_IRQ6Handler(void)
{
    GPIO_IRQHandler(4U, 6U);
}
void GPIO4_IRQ7Handler(void)
{
    GPIO_IRQHandler(4U, 7U);
}
#endif

#if (RTE_GPIO5)
void GPIO5_IRQ0Handler(void)
{
    GPIO_IRQHandler(5U, 0U);
}
void GPIO5_IRQ1Handler(void)
{
    GPIO_IRQHandler(5U, 1U);
}
void GPIO5_IRQ2Handler(void)
{
    GPIO_IRQHandler(5U, 2U);
}
void GPIO5_IRQ3Handler(void)
{
    GPIO_IRQHandler(5U, 3U);
}
void GPIO5_IRQ4Handler(void)
{
    GPIO_IRQHandler(5U, 4U);
}
void GPIO5_IRQ5Handler(void)
{
    GPIO_IRQHandler(5U, 5U);
}
void GPIO5_IRQ6Handler(void)
{
    GPIO_IRQHandler(5U, 6U);
}
void GPIO5_IRQ7Handler(void)
{
    GPIO_IRQHandler(5U, 7U);
}
#endif

#if (RTE_GPIO6)
void GPIO6_IRQ0Handler(void)
{
    GPIO_IRQHandler(6U, 0U);
}
void GPIO6_IRQ1Handler(void)
{
    GPIO_IRQHandler(6U, 1U);
}
void GPIO6_IRQ2Handler(void)
{
    GPIO_IRQHandler(6U, 2U);
}
void GPIO6_IRQ3Handler(void)
{
    GPIO_IRQHandler(6U, 3U);
}
void GPIO6_IRQ4Handler(void)
{
    GPIO_IRQHandler(6U, 4U);
}
void GPIO6_IRQ5Handler(void)
{
    GPIO_IRQHandler(6U, 5U);
}
void GPIO6_IRQ6Handler(void)
{
    GPIO_IRQHandler(6U, 6U);
}
void GPIO6_IRQ7Handler(void)
{
    GPIO_IRQHandler(6U, 7U);
}
#endif

#if (RTE_GPIO7)
void GPIO7_IRQ0Handler(void)
{
    GPIO_IRQHandler(7U, 0U);
}
void GPIO7_IRQ1Handler(void)
{
    GPIO_IRQHandler(7U, 1U);
}
void GPIO7_IRQ2Handler(void)
{
    GPIO_IRQHandler(7U, 2U);
}
void GPIO7_IRQ3Handler(void)
{
    GPIO_IRQHandler(7U, 3U);
}
void GPIO7_IRQ4Handler(void)
{
    GPIO_IRQHandler(7U, 4U);
}
void GPIO7_IRQ5Handler(void)
{
    GPIO_IRQHandler(7U, 5U);
}
void GPIO7_IRQ6Handler(void)
{
    GPIO_IRQHandler(7U, 6U);
}
void GPIO7_IRQ7Handler(void)
{
    GPIO_IRQHandler(7U, 7U);
}
#endif

#if (RTE_GPIO8)
void GPIO8_IRQ0Handler(void)
{
    GPIO_IRQHandler(8U, 0U);
}
void GPIO8_IRQ1Handler(void)
{
    GPIO_IRQHandler(8U, 1U);
}
void GPIO8_IRQ2Handler(void)
{
    GPIO_IRQHandler(8U, 2U);
}
void GPIO8_IRQ3Handler(void)
{
    GPIO_IRQHandler(8U, 3U);
}
void GPIO8_IRQ4Handler(void)
{
    GPIO_IRQHandler(8U, 4U);
}
void GPIO8_IRQ5Handler(void)
{
    GPIO_IRQHandler(8U, 5U);
}
void GPIO8_IRQ6Handler(void)
{
    GPIO_IRQHandler(8U, 6U);
}
void GPIO8_IRQ7Handler(void)
{
    GPIO_IRQHandler(8U, 7U);
}
#endif

#if (RTE_GPIO9)
void GPIO9_IRQ0Handler(void)
{
    GPIO_IRQHandler(9U, 0U);
}
void GPIO9_IRQ1Handler(void)
{
    GPIO_IRQHandler(9U, 1U);
}
void GPIO9_IRQ2Handler(void)
{
    GPIO_IRQHandler(9U, 2U);
}
void GPIO9_IRQ3Handler(void)
{
    GPIO_IRQHandler(9U, 3U);
}
void GPIO9_IRQ4Handler(void)
{
    GPIO_IRQHandler(9U, 4U);
}
void GPIO9_IRQ5Handler(void)
{
    GPIO_IRQHandler(9U, 5U);
}
void GPIO9_IRQ6Handler(void)
{
    GPIO_IRQHandler(9U, 6U);
}
void GPIO9_IRQ7Handler(void)
{
    GPIO_IRQHandler(9U, 7U);
}
#endif

#if (RTE_GPIO10)
void GPIO10_IRQ0Handler(void)
{
    GPIO_IRQHandler(10U, 0U);
}
void GPIO10_IRQ1Handler(void)
{
    GPIO_IRQHandler(10U, 1U);
}
void GPIO10_IRQ2Handler(void)
{
    GPIO_IRQHandler(10U, 2U);
}
void GPIO10_IRQ3Handler(void)
{
    GPIO_IRQHandler(10U, 3U);
}
void GPIO10_IRQ4Handler(void)
{
    GPIO_IRQHandler(10U, 4U);
}
void GPIO10_IRQ5Handler(void)
{
    GPIO_IRQHandler(10U, 5U);
}
void GPIO10_IRQ6Handler(void)
{
    GPIO_IRQHandler(10U, 6U);
}
void GPIO10_IRQ7Handler(void)
{
    GPIO_IRQHandler(10U, 7U);
}
#endif

#if (RTE_GPIO11)
void GPIO11_IRQ0Handler(void)
{
    GPIO_IRQHandler(11U, 0U);
}
void GPIO11_IRQ1Handler(void)
{
    GPIO_IRQHandler(11U, 1U);
}
void GPIO11_IRQ2Handler(void)
{
    GPIO_IRQHandler(11U, 2U);
}
void GPIO11_IRQ3Handler(void)
{
    GPIO_IRQHandler(11U, 3U);
}
void GPIO11_IRQ4Handler(void)
{
    GPIO_IRQHandler(11U, 4U);
}
void GPIO11_IRQ5Handler(void)
{
    GPIO_IRQHandler(11U, 5U);
}
void GPIO11_IRQ6Handler(void)
{
    GPIO_IRQHandler(11U, 6U);
}
void GPIO11_IRQ7Handler(void)
{
    GPIO_IRQHandler(11U, 7U);
}
#endif

#if (RTE_GPIO12)
void GPIO12_IRQ0Handler(void)
{
    GPIO_IRQHandler(12U, 0U);
}
void GPIO12_IRQ1Handler(void)
{
    GPIO_IRQHandler(12U, 1U);
}
void GPIO12_IRQ2Handler(void)
{
    GPIO_IRQHandler(12U, 2U);
}
void GPIO12_IRQ3Handler(void)
{
    GPIO_IRQHandler(12U, 3U);
}
void GPIO12_IRQ4Handler(void)
{
    GPIO_IRQHandler(12U, 4U);
}
void GPIO12_IRQ5Handler(void)
{
    GPIO_IRQHandler(12U, 5U);
}
void GPIO12_IRQ6Handler(void)
{
    GPIO_IRQHandler(12U, 6U);
}
void GPIO12_IRQ7Handler(void)
{
    GPIO_IRQHandler(12U, 7U);
}
#endif

#if (RTE_GPIO13)
void GPIO13_IRQ0Handler(void)
{
    GPIO_IRQHandler(13U, 0U);
}
void GPIO13_IRQ1Handler(void)
{
    GPIO_IRQHandler(13U, 1U);
}
void GPIO13_IRQ2Handler(void)
{
    GPIO_IRQHandler(13U, 2U);
}
void GPIO13_IRQ3Handler(void)
{
    GPIO_IRQHandler(13U, 3U);
}
void GPIO13_IRQ4Handler(void)
{
    GPIO_IRQHandler(13U, 4U);
}
void GPIO13_IRQ5Handler(void)
{
    GPIO_IRQHandler(13U, 5U);
}
void GPIO13_IRQ6Handler(void)
{
    GPIO_IRQHandler(13U, 6U);
}
void GPIO13_IRQ7Handler(void)
{
    GPIO_IRQHandler(13U, 7U);
}
#endif

#if (RTE_GPIO14)
void GPIO14_IRQ0Handler(void)
{
    GPIO_IRQHandler(14U, 0U);
}
void GPIO14_IRQ1Handler(void)
{
    GPIO_IRQHandler(14U, 1U);
}
void GPIO14_IRQ2Handler(void)
{
    GPIO_IRQHandler(14U, 2U);
}
void GPIO14_IRQ3Handler(void)
{
    GPIO_IRQHandler(14U, 3U);
}
void GPIO14_IRQ4Handler(void)
{
    GPIO_IRQHandler(14U, 4U);
}
void GPIO14_IRQ5Handler(void)
{
    GPIO_IRQHandler(14U, 5U);
}
void GPIO14_IRQ6Handler(void)
{
    GPIO_IRQHandler(14U, 6U);
}
void GPIO14_IRQ7Handler(void)
{
    GPIO_IRQHandler(14U, 7U);
}
#endif

#if (RTE_LPGPIO)
void LPGPIO_IRQ0Handler(void)
{
    GPIO_IRQHandler(15U, 0U);
}
void LPGPIO_IRQ1Handler(void)
{
    GPIO_IRQHandler(15U, 1U);
}
#if (SOC_FEAT_LPGPIO_HAS_PIN2_7)
void LPGPIO_IRQ2Handler(void)
{
    GPIO_IRQHandler(15U, 2U);
}
void LPGPIO_IRQ3Handler(void)
{
    GPIO_IRQHandler(15U, 3U);
}
void LPGPIO_IRQ4Handler(void)
{
    GPIO_IRQHandler(15U, 4U);
}
void LPGPIO_IRQ5Handler(void)
{
    GPIO_IRQHandler(15U, 5U);
}
void LPGPIO_IRQ6Handler(void)
{
    GPIO_IRQHandler(15U, 6U);
}
void LPGPIO_IRQ7Handler(void)
{
    GPIO_IRQHandler(15U, 7U);
}
#endif
#endif

#endif /* CMSIS_GPIO_DISABLE_EVENTS */

/* Setup GPIO Interface */
static int32_t GPIO_Setup(ARM_GPIO_Pin_t pin, ARM_GPIO_SignalEvent_t cb_event)
{
    GPIO_Type *gpio;
    uint32_t   gpio_num;
    uint32_t   pin_num;
    int32_t    status;

    gpio_num = pin >> 3U;
    pin_num  = pin & 0x7U;

    if (pin_num >= gpio_res[gpio_num].max_pins) {
        status = ARM_GPIO_ERROR_PIN;
    } else {
        status = ARM_DRIVER_OK;

        /* Setup I/O signal multiplexer */
        pinconf_set(gpio_num,
                    pin_num,
                    PINMUX_ALTERNATE_FUNCTION_0,
                    PADCTRL_READ_ENABLE | PADCTRL_SLEW_RATE_FAST | PADCTRL_DRIVER_DISABLED_HIGH_Z |
                        PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);

        gpio = (GPIO_Type *) gpio_res[gpio_num].base;

        /* Default direction is input */
        gpio_set_direction_input(gpio, pin_num);

        if (cb_event == NULL) {
            /* Disable interrupt if no callback installed */
            gpio_disable_interrupt(gpio, pin_num);
        } else {
#ifndef CMSIS_GPIO_DISABLE_EVENTS
            /* Save pin callback event */
            gpio_res[gpio_num].cb_event[pin_num] = cb_event;

            /* Enable peripheral interrupt generation */
            gpio_mask_interrupt(gpio, pin_num);

            /* Enable NVIC interrupt generation */
            NVIC_EnableIRQ((IRQn_Type) (gpio_res[gpio_num].irqn + pin_num));

            /* Enable interrupt */
            gpio_enable_interrupt(gpio, pin_num);
#else
            status = ARM_DRIVER_ERROR_UNSUPPORTED;
#endif /* CMSIS_GPIO_DISABLE_EVENTS */
        }
    }

    return status;
}

/* Set GPIO Direction */
static int32_t GPIO_SetDirection(ARM_GPIO_Pin_t pin, ARM_GPIO_DIRECTION direction)
{
    GPIO_Type *gpio;
    uint32_t   gpio_num;
    uint32_t   pin_num;
    int32_t    status;

    gpio_num = pin >> 3U;
    pin_num  = pin & 0x7U;

    if (pin_num >= gpio_res[gpio_num].max_pins) {
        status = ARM_GPIO_ERROR_PIN;
    } else {
        status = ARM_DRIVER_OK;

        gpio   = (GPIO_Type *) gpio_res[gpio_num].base;

        if (direction == ARM_GPIO_INPUT) {
            gpio_set_direction_input(gpio, pin_num);
        } else if (direction == ARM_GPIO_OUTPUT) {
            gpio_set_direction_output(gpio, pin_num);
        } else {
            status = ARM_DRIVER_ERROR_PARAMETER;
        }
    }

    return status;
}

/* Set GPIO Output Mode */
static int32_t GPIO_SetOutputMode(ARM_GPIO_Pin_t pin, ARM_GPIO_OUTPUT_MODE mode)
{
    uint32_t gpio_num;
    uint32_t pin_num;
    int32_t  status;
    uint8_t  alt_func;
    uint8_t  pad_ctrl;

    gpio_num = pin >> 3U;
    pin_num  = pin & 0x7U;

    if (pin_num >= gpio_res[gpio_num].max_pins) {
        status = ARM_GPIO_ERROR_PIN;
    } else {
        status = ARM_DRIVER_OK;

        /* Read current configuration */
        if (pinconf_get(gpio_num, pin_num, &alt_func, &pad_ctrl) != 0) {
            status = ARM_GPIO_ERROR_PIN;
        } else {
            if (mode == ARM_GPIO_PUSH_PULL) {
                pad_ctrl &= ~(PADCTRL_DRIVER_OPEN_DRAIN);
            } else if (mode == ARM_GPIO_OPEN_DRAIN) {
                pad_ctrl |= PADCTRL_DRIVER_OPEN_DRAIN;
            } else {
                status = ARM_DRIVER_ERROR_PARAMETER;
            }
        }

        if (status == ARM_DRIVER_OK) {
            /* Apply new configuration */
            (void) pinconf_set(gpio_num, pin_num, PINMUX_ALTERNATE_FUNCTION_0, pad_ctrl);
        }
    }

    return status;
}

/* Set GPIO Pull Resistor */
static int32_t GPIO_SetPullResistor(ARM_GPIO_Pin_t pin, ARM_GPIO_PULL_RESISTOR resistor)
{
    uint32_t gpio_num;
    uint32_t pin_num;
    int32_t  status;
    uint8_t  alt_func;
    uint8_t  pad_ctrl;

    gpio_num = pin >> 3U;
    pin_num  = pin & 0x7U;

    if (pin_num >= gpio_res[gpio_num].max_pins) {
        status = ARM_GPIO_ERROR_PIN;
    } else {
        status   = ARM_DRIVER_OK;

        gpio_num = pin >> 3U;
        pin_num  = pin & 0x7U;

        /* Read current configuration */
        if (pinconf_get(gpio_num, pin_num, &alt_func, &pad_ctrl) != 0) {
            status = ARM_GPIO_ERROR_PIN;
        } else {
            pad_ctrl &= ~(PADCTRL_DRIVER_DISABLED_HIGH_Z | PADCTRL_DRIVER_DISABLED_PULL_UP |
                          PADCTRL_DRIVER_DISABLED_PULL_DOWN | PADCTRL_DRIVER_DISABLED_BUS_REPEATER);

            if (resistor == ARM_GPIO_PULL_NONE) {
                pad_ctrl |= PADCTRL_DRIVER_DISABLED_HIGH_Z;
            } else if (resistor == ARM_GPIO_PULL_UP) {
                pad_ctrl |= PADCTRL_DRIVER_DISABLED_PULL_UP;
            } else if (resistor == ARM_GPIO_PULL_DOWN) {
                pad_ctrl |= PADCTRL_DRIVER_DISABLED_PULL_DOWN;
            } else {
                status = ARM_DRIVER_ERROR_PARAMETER;
            }
        }

        if (status == ARM_DRIVER_OK) {
            /* Apply new configuration */
            (void) pinconf_set(gpio_num, pin_num, PINMUX_ALTERNATE_FUNCTION_0, pad_ctrl);
        }
    }

    return status;
}

/* Set GPIO Event Trigger */
static int32_t GPIO_SetEventTrigger(ARM_GPIO_Pin_t pin, ARM_GPIO_EVENT_TRIGGER trigger)
{
#ifndef CMSIS_GPIO_DISABLE_EVENTS
    GPIO_Type *gpio;
    uint32_t   gpio_num;
    uint32_t   pin_num;
    int32_t    status;

    gpio_num = pin >> 3U;
    pin_num  = pin & 0x7U;

    if (pin_num >= gpio_res[gpio_num].max_pins) {
        status = ARM_GPIO_ERROR_PIN;
    } else {
        status   = ARM_DRIVER_OK;

        gpio_num = pin >> 3U;
        pin_num  = pin & 0x7U;

        gpio     = (GPIO_Type *) gpio_res[gpio_num].base;

        if (trigger == ARM_GPIO_TRIGGER_NONE) {
            /* No trigger, just disable interrupt */
            gpio_disable_interrupt(gpio, pin_num);
        } else if (trigger == ARM_GPIO_TRIGGER_RISING_EDGE) {
            /* Rising edge triggers interrupt */
            gpio_interrupt_set_edge_trigger(gpio, pin_num);
            gpio_interrupt_set_polarity_high(gpio, pin_num);
        } else if (trigger == ARM_GPIO_TRIGGER_FALLING_EDGE) {
            /* Falling edge triggers interrupt */
            gpio_interrupt_set_edge_trigger(gpio, pin_num);
            gpio_interrupt_set_polarity_low(gpio, pin_num);
        } else if (trigger == ARM_GPIO_TRIGGER_EITHER_EDGE) {
            /* Either rising or falling edge triggers interrupt */
            gpio_interrupt_set_both_edge_trigger(gpio, pin_num);
        } else {
            status = ARM_DRIVER_ERROR_PARAMETER;
        }
    }
#else
    (void) pin;
    (void) trigger;
    int32_t status = ARM_DRIVER_ERROR_UNSUPPORTED;
#endif /* CMSIS_GPIO_DISABLE_EVENTS */
    return status;
}

/* Set GPIO Output Level */
static void GPIO_SetOutput(ARM_GPIO_Pin_t pin, uint32_t val)
{
    GPIO_Type *gpio;
    uint32_t   gpio_num;
    uint32_t   pin_num;

    gpio_num = pin >> 3U;
    pin_num  = pin & 0x7U;

    if (pin_num < gpio_res[gpio_num].max_pins) {

        gpio = (GPIO_Type *) gpio_res[gpio_num].base;

        if (val == 0U) {
            gpio_set_value_low(gpio, pin_num);
        } else {
            gpio_set_value_high(gpio, pin_num);
        }
    }
}

/* Get GPIO Input Level */
static uint32_t GPIO_GetInput(ARM_GPIO_Pin_t pin)
{
    GPIO_Type *gpio;
    uint32_t   gpio_num;
    uint32_t   pin_num;
    uint32_t   val;

    gpio_num = pin >> 3U;
    pin_num  = pin & 0x7U;

    if (pin_num >= gpio_res[gpio_num].max_pins) {
        val = 0U;
    } else {

        gpio = (GPIO_Type *) gpio_res[gpio_num].base;

        val  = gpio_get_value(gpio, pin_num);
    }

    return val;
}

/* GPIO Driver access structure */
ARM_DRIVER_GPIO Driver_GPIO = {
    GPIO_Setup,
    GPIO_SetDirection,
    GPIO_SetOutputMode,
    GPIO_SetPullResistor,
    GPIO_SetEventTrigger,
    GPIO_SetOutput,
    GPIO_GetInput
};

#endif /* RTE_Drivers_GPIO */
