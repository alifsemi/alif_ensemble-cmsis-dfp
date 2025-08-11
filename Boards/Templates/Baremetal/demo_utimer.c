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
 * @file     : demo_utimer.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 30-May-2023
 * @brief    : Baremetal demo application for UTIMER.
 *             - Configuring the UTIMER Channel 0 for 500ms basic mode.
 *             - Configuring the UTIMER Channel 1 for 500ms, 1000ms, 1500ms buffering mode.
 *             - Configuring the UTIMER Channel 3(E7) and Channel 2(E1C) for counter start
 *triggering mode.
 *             - Configuring the UTIMER Channel 4(E7) and Channel 3(E1C) for driver A, double
 *buffering capture mode.
 *             - Configuring the UTIMER Channel 5 for driver A, double buffering compare mode (only
 *for E7).
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/

/* History:
 *  Version 1.0.1
 *     update for latest devkit
 *  Version 1.0.0
 *     initial version
 */

#include <stdio.h>
#include <inttypes.h>
#include "Driver_UTIMER.h"
#include "Driver_IO.h"
#include "pinconf.h"
#include "board_config.h"
#include "app_utils.h"

#include "RTE_Components.h"
#include CMSIS_device_header
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

// Set to 0: Use application-defined UTIMER pin configuration (from board_utimer_pins_config).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* UTIMER0 Driver instance */
extern ARM_DRIVER_UTIMER Driver_UTIMER0;
ARM_DRIVER_UTIMER       *ptrUTIMER = &Driver_UTIMER0;

#ifdef BOARD_TRIGGER_MODE_UTIMER_INSTANCE
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_UT_TRIGGER_MODE_GPO0_GPIO_PORT);
ARM_DRIVER_GPIO       *ptrTrig0GPO = &ARM_Driver_GPIO_(BOARD_UT_TRIGGER_MODE_GPO0_GPIO_PORT);

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PORT);
ARM_DRIVER_GPIO       *ptrTrig1GPO = &ARM_Driver_GPIO_(BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PORT);
#endif

#ifdef BOARD_CAPTURE_MODE_UTIMER_INSTANCE
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PORT);
ARM_DRIVER_GPIO       *ptrCapt0GPO = &ARM_Driver_GPIO_(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PORT);

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PORT);
ARM_DRIVER_GPIO       *ptrCapt1GPO = &ARM_Driver_GPIO_(BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PORT);
#endif

/* Delay to wait for UTIMER interrupt */
#define UTIMER_DELAY_MS(n)      for (uint32_t count = 0; count < (n + 20); count++) {  \
                                    sys_busy_loop_us(1000); \
                                }

/**
 * @function    int gpio_init(ARM_UTIMER_MODE mode)
 * @brief       GPIO initialization using gpio driver
 * @note        none
 * @param       mode
 * @retval      execution status
 */
static int32_t gpio_init(ARM_UTIMER_MODE mode)
{
    int32_t ret;

    if (mode == ARM_UTIMER_MODE_TRIGGERING) {
#if BOARD_TRIGGER_MODE_UTIMER_INSTANCE
        ret = ptrTrig0GPO->Initialize(BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN, NULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to initialize BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN as GPIO\n");
            return -1;
        }

        ret = ptrTrig0GPO->PowerControl(BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN, ARM_POWER_FULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to Power up BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN\n");
            return -1;
        }

        ret = ptrTrig0GPO->SetDirection(BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN,
                                        GPIO_PIN_DIRECTION_OUTPUT);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set direction for BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN\n");
            return -1;
        }

        ret =
            ptrTrig0GPO->SetValue(BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set value for BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN\n");
            return -1;
        }

        ret = ptrTrig1GPO->Initialize(BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN, NULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to initialize BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN as GPIO\n");
            return -1;
        }

        ret = ptrTrig1GPO->PowerControl(BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN, ARM_POWER_FULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to Power up BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN\n");
            return -1;
        }

        ret = ptrTrig1GPO->SetDirection(BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN,
                                        GPIO_PIN_DIRECTION_OUTPUT);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set direction for BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN\n");
            return -1;
        }

        ret = ptrTrig1GPO->SetValue(BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set value for BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN\n");
            return -1;
        }
#endif
    }

    else if (mode == ARM_UTIMER_MODE_CAPTURING) {
#if BOARD_CAPTURE_MODE_UTIMER_INSTANCE
        ret = ptrCapt0GPO->Initialize(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN, NULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to initialize BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN as GPIO\n");
            return -1;
        }

        ret = ptrCapt0GPO->PowerControl(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN, ARM_POWER_FULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to Power up BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN\n");
            return -1;
        }

        ret = ptrCapt0GPO->SetDirection(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN,
                                        GPIO_PIN_DIRECTION_OUTPUT);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set direction for BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN\n");
            return -1;
        }

        ret = ptrCapt0GPO->SetValue(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set value for BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN\n");
            return -1;
        }

        ret = ptrCapt1GPO->Initialize(BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN, NULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to initialize BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN as GPIO\n");
            return -1;
        }

        ret = ptrCapt1GPO->PowerControl(BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN, ARM_POWER_FULL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to Power up BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN\n");
            return -1;
        }

        ret = ptrCapt1GPO->SetDirection(BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN,
                                        GPIO_PIN_DIRECTION_OUTPUT);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set direction for BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN\n");
            return -1;
        }

        ret = ptrCapt1GPO->SetValue(BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set value for BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN\n");
            return -1;
        }
#endif
    } else {
        return -1;
    }

    return ARM_DRIVER_OK;
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @function    int32_t board_utimer_pins_config(void)
 * @brief       UTIMER pinmux config
 * @note        none
 * @param       mode
 * @retval      execution status
 */
static int32_t board_utimer_pins_config(void)
{
    int32_t ret;

#ifdef BOARD_TRIGGER_MODE_UTIMER_INSTANCE
    /* UITMER Trigger mode pins config */
    ret = pinconf_set(PORT_(BOARD_UT_TRIGGER_MODE_T0_GPO_GPIO_PORT),
                      BOARD_UT_TRIGGER_MODE_T0_GPO_GPIO_PIN,
                      BOARD_UT_TRIGGER_MODE_T0_GPO_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_UT_TRIGGER_MODE_T1_GPO_GPIO_PORT),
                      BOARD_UT_TRIGGER_MODE_T1_GPO_GPIO_PIN,
                      BOARD_UT_TRIGGER_MODE_T1_GPO_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
    /* GPIO init for Trigger mode */
    ret = pinconf_set(PORT_(BOARD_UT_TRIGGER_MODE_GPO0_GPIO_PORT),
                      BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_0,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PORT),
                      BOARD_UT_TRIGGER_MODE_GPO1_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_0,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
#endif

#ifdef BOARD_CAPTURE_MODE_UTIMER_INSTANCE
    /* UITMER Capture mode pins config */
    ret = pinconf_set(PORT_(BOARD_UT_CAPTURE_MODE_T0_GPO_GPIO_PORT),
                      BOARD_UT_CAPTURE_MODE_T0_GPO_GPIO_PIN,
                      BOARD_UT_CAPTURE_MODE_T0_GPO_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_UT_CAPTURE_MODE_T1_GPO_GPIO_PORT),
                      BOARD_UT_CAPTURE_MODE_T1_GPO_GPIO_PIN,
                      BOARD_UT_CAPTURE_MODE_T1_GPO_ALTERNATE_FUNCTION,
                      PADCTRL_READ_ENABLE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
    /* GPIO init for Capture mode */
    ret = pinconf_set(PORT_(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PORT),
                      BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_0,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
    ret = pinconf_set(PORT_(BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PORT),
                      BOARD_UT_CAPTURE_MODE_GPO1_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_0,
                      0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
#endif

#ifdef BOARD_COMPARE_MODE_UTIMER_INSTANCE
    /* UITMER Compare mode pins config */
    ret = pinconf_set(PORT_(BOARD_UT_COMPARE_MODE_T0_GPO_GPIO_PORT),
                      BOARD_UT_COMPARE_MODE_T0_GPO_GPIO_PIN,
                      BOARD_UT_COMPARE_MODE_T0_GPO_ALTERNATE_FUNCTION,
                      PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return ret;
    }
#endif

    return 0;
}
#endif

#ifdef BOARD_BASIC_MODE_UTIMER_INSTANCE
static volatile uint32_t cb_basic_status;
/**
 * @function    void utimer_basic_mode_cb_func(event)
 * @brief       utimer basic mode callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_basic_mode_cb_func(uint8_t event)
{
    if (event & ARM_UTIMER_EVENT_OVER_FLOW) {
        cb_basic_status++;
    }
}

/**
 * @function    void utimer_basic_mode_app(void)
 * @brief       utimer basic mode application
 * @note        none
 * @param       none
 * @retval      none
 */
static void utimer_basic_mode_app(void)
{
    int32_t  ret;
    uint8_t  channel = BOARD_BASIC_MODE_UTIMER_INSTANCE;
    uint32_t count_array[2];

    /* utimer channel 0 is configured for utimer basic mode (config counter ptr reg for 500ms) */

    printf("*** utimer demo application for basic mode started ***\n");
    /* For E7:
     * System CLOCK frequency (F)= 400Mhz
     *
     * Time for 1 count T = 1/F = 1/(400*10^6) = 0.0025 * 10^-6
     *
     * To Increment or Decrement Timer by 1 count, takes 0.0025 micro sec
     *
     * So count for 500ms = (500*(10^-3))/(0.0025*(10^-6)) = 20000000
     *
     * DEC = 20000000
     * HEX = 0xBEBC200
     *
     * For E1C:
     * System CLOCK frequency (F)= 160Mhz
     *
     * Time for 1 count T = 1/F = 1/(160*10^6) = 6.25 * 10^-9
     *
     * To Increment or Decrement Timer by 1 count, takes 6.25 nano sec
     *
     * So count for 500ms = (500*(10^-3))/(6.25*(10^-9)) = 80000000
     *
     * DEC = 80000000
     */
    count_array[0] = 0x00000000;                               /*< initial counter value >*/
    count_array[1] = BOARD_UTIMER_500_MILLI_SEC_COUNTER_VALUE; /*< over flow count value >*/

    ret            = ptrUTIMER->Initialize(channel, utimer_basic_mode_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power up \n", channel);
        goto error_basic_mode_uninstall;
    }

    ret = ptrUTIMER->ConfigCounter(channel, ARM_UTIMER_MODE_BASIC, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " mode configuration failed \n", channel);
        goto error_basic_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR, count_array[0]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_basic_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR_PTR, count_array[1]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_basic_mode_poweroff;
    }

    printf("utimer channel '%" PRIu8 "'configured on basic mode for 500 ms\r\n", channel);

    ret = ptrUTIMER->Start(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to start \n", channel);
        goto error_basic_mode_poweroff;
    } else {
        printf("utimer channel '%" PRIu8 "': timer started\n", channel);
    }

    UTIMER_DELAY_MS(500U);

    if (cb_basic_status) {
        cb_basic_status = 0;
        printf("utimer channel %" PRIu8 " :500ms timer expired \n", channel);
    } else {
        printf("Error :utimer basic mode timeout \n");
    }

    ret = ptrUTIMER->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to stop \n", channel);
    } else {
        printf("utimer channel %" PRIu8 " :timer stopped\n", channel);
    }

error_basic_mode_poweroff:

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power off \n", channel);
    }

error_basic_mode_uninstall:

    ret = ptrUTIMER->Uninitialize(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to un-initialize \n", channel);
    }

    printf("*** demo application: basic mode completed *** \r\n\n");
}
#endif

#ifdef BOARD_BUFFER_MODE_UTIMER_INSTANCE
static volatile uint32_t cb_buffer_status;
/**
 * @function    void utimer_buffering_mode_cb_func(event)
 * @brief       utimer buffer mode callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_buffering_mode_cb_func(uint8_t event)
{
    if (event & ARM_UTIMER_EVENT_OVER_FLOW) {
        cb_buffer_status++;
    }
}

/**
 * @function    void utimer_buffering_mode_app(void)
 * @brief       utimer buffer mode application
 * @note        none
 * @param       none
 * @retval      none
 */
static void utimer_buffering_mode_app(void)
{
    int32_t  ret;
    uint8_t  channel = BOARD_BUFFER_MODE_UTIMER_INSTANCE;
    uint8_t  index;
    uint32_t count_array[4];

    /* utimer channel 1 is configured for utimer buffer mode (selected double buffering)
     * configuring counter ptr, buf1, buf2 reg's as 500ms, 1 sec, 1.5 sec respectively */

    printf("*** utimer demo application for buffering mode started ***\n");
    /* For E7:
     * System CLOCK frequency (F)= 400Mhz
     *
     * Time for 1 count T = 1/F = 1/(400*10^6) = 0.0025 * 10^-6
     *
     * To Increment or Decrement Timer by 1 count, takes 0.0025 micro sec
     *
     * So count for 500ms = (500*(10^-3))/(0.0025*(10^-6))
     * DEC = 200000000
     * HEX = 0xBEBC200
     *
     * So count for 1000ms = (1000*(10^-3))/(0.0025*(10^-6))
     * DEC = 400000000
     * HEX = 0x17D78400
     *
     * So count for 1500ms = (1500*(10^-3))/(0.0025*(10^-6))
     * DEC = 60000000
     * HEX = 0x23C34600
     *
     * For E1C:
     * System CLOCK frequency (F)= 160Mhz
     *
     * Time for 1 count T = 1/F = 1/(160*10^6) = 6.25 * 10^-9
     *
     * To Increment or Decrement Timer by 1 count, takes 6.25 nano sec
     *
     * So count for 500ms = (500*(10^-3))/(6.25*(10^-9)) = 80000000
     *
     * So count for 1000ms = (1000*(10^-3))/(6.25*(10^-9)) = 160000000
     *
     * So count for 1500ms = (1500*(10^-3))/(6.25*(10^-9)) = 240000000
     */

    count_array[0] = 0x00000000; /*< Initial counter value>*/
    count_array[1] =
        BOARD_UTIMER_500_MILLI_SEC_COUNTER_VALUE; /*< Over flow count value for First Iteration>*/
    count_array[2] =
        BOARD_UTIMER_1000_MILLI_SEC_COUNTER_VALUE; /*< Over flow count value for Second Iteration>*/
    count_array[3] =
        BOARD_UTIMER_1500_MILLI_SEC_COUNTER_VALUE; /*< Over flow count value for Third Iteration>*/

    ret = ptrUTIMER->Initialize(channel, utimer_buffering_mode_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power up \n", channel);
        goto error_buffering_mode_uninstall;
    }

    ret = ptrUTIMER->ConfigCounter(channel, ARM_UTIMER_MODE_BUFFERING, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " mode configuration failed \n", channel);
        goto error_buffering_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR, count_array[0]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_buffering_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR_PTR, count_array[1]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_buffering_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR_PTR_BUF1, count_array[2]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_buffering_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR_PTR_BUF2, count_array[3]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_buffering_mode_poweroff;
    }

    printf("channel '%" PRIu8 "'configured on buffering mode for 500, 1000, and 1500 ms \r\n",
           channel);

    ret = ptrUTIMER->Start(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to start \n", channel);
        goto error_buffering_mode_poweroff;
    } else {
        printf("utimer channel %" PRIu8 " :timer started\n", channel);
    }

    for (index = 1; index <= 3; index++) {
        UTIMER_DELAY_MS(500U * index);

        if (cb_buffer_status) {
            cb_buffer_status = 0;
            printf("utimer channel %" PRIu8 ": %" PRIu32 " ms timer expired\n",
                   channel,
                   (uint32_t) (500 * index));
        } else {
            printf("Error :Utimer buffer mode timeout \n");
        }
    }

    ret = ptrUTIMER->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to stop \n", channel);
    } else {
        printf("utimer channel %" PRIu8 ": timer stopped\n", channel);
    }

error_buffering_mode_poweroff:

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("uTIMER channel %" PRIu8 " failed power off \n", channel);
    }

error_buffering_mode_uninstall:

    ret = ptrUTIMER->Uninitialize(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to un-initialize \n", channel);
    }

    printf("*** demo application: buffering mode completed *** \r\n\n");
}
#endif

#ifdef BOARD_TRIGGER_MODE_UTIMER_INSTANCE
static volatile uint32_t cb_trigger_status;
/**
 * @function    void utimer_trigger_mode_cb_func(event)
 * @brief       utimer trigger mode callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_trigger_mode_cb_func(uint8_t event)
{
    if (event & ARM_UTIMER_EVENT_OVER_FLOW) {
        cb_trigger_status = 1;
    }
}

/**
 * @function    void utimer_trigger_mode_app(void)
 * @brief       utimer trigger mode application
 * @note        none
 * @param       none
 * @retval      none
 */
static void utimer_trigger_mode_app(void)
{
    int32_t  ret;
    uint32_t value   = 0;
    uint8_t  channel = BOARD_TRIGGER_MODE_UTIMER_INSTANCE;
    uint32_t count_array[2];

    ARM_UTIMER_TRIGGER_CONFIG trig_config = {.triggerTarget = ARM_UTIMER_TRIGGER_START,
                                             .triggerSrc    = ARM_UTIMER_SRC_1,
                                             .trigger       = ARM_UTIMER_SRC1_DRIVE_A_RISING_B_0};

    /*
     * utimer channel 3 (E7) and channel 2 (E1C) is configured for utimer trigger mode.
     * chan_event_a_rising_b_0 event from pinmux is used for triggering counter start.

     **/

    printf("*** utimer demo application for trigger mode started ***\n");
    /* For E7:
     * System CLOCK frequency (F)= 400Mhz
     *
     * Time for 1 count T = 1/F = 1/(400*10^6) = 0.0025 * 10^-6
     *
     * To Increment or Decrement Timer by 1 count, takes 0.0025 micro sec
     *
     * So count for 500ms = (500*(10^-3))/(0.0025*(10^-6)) = 200000000
     *
     * DEC = 200000000
     * HEX = 0xBEBC200
     *
     * For E1C:
     * System CLOCK frequency (F)= 160Mhz
     *
     * Time for 1 count T = 1/F = 1/(160*10^6) = 6.25 * 10^-9
     *
     * To Increment or Decrement Timer by 1 count, takes 6.25 nano sec
     *
     * So count for 500ms = (500*(10^-3))/(6.25*(10^-9)) = 80000000
     */

    count_array[0] = 0;                                        /*< initial counter value >*/
    count_array[1] = BOARD_UTIMER_500_MILLI_SEC_COUNTER_VALUE; /*< over flow count value >*/

    ret            = gpio_init(ARM_UTIMER_MODE_TRIGGERING);
    if (ret) {
        printf("gpio init failed\n");
    }

    ret = ptrUTIMER->Initialize(channel, utimer_trigger_mode_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power up \n", channel);
        goto error_trigger_mode_uninstall;
    }

    ret = ptrUTIMER->ConfigCounter(channel, ARM_UTIMER_MODE_TRIGGERING, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " mode configuration failed \n", channel);
        goto error_trigger_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR, count_array[0]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_trigger_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR_PTR, count_array[1]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_trigger_mode_poweroff;
    }

    /* Config Trigger for counter start using chan_event_a_rising_b_0 */
    ret = ptrUTIMER->ConfigTrigger(channel, &trig_config);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " trigger configuration failed \n", channel);
        goto error_trigger_mode_poweroff;
    } else {
        printf("utimer channel %" PRIu8 " triggered for counter start \n", channel);
    }

    value = ptrUTIMER->GetCount(channel, ARM_UTIMER_CNTR);
    printf("counter value before triggering : %" PRIu32 "\n", value);

    ret = ptrTrig0GPO->SetValue(BOARD_UT_TRIGGER_MODE_GPIO0_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if ((ret != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to configure\n");
    }

    value = ptrUTIMER->GetCount(channel, ARM_UTIMER_CNTR);
    printf("counter value immediately after triggering : %" PRIu32 "\n", value);

    while (1) {
        if (cb_trigger_status) {
            cb_trigger_status = 0;
            printf("overflow interrupt is generated\n");
            break;
        }
    }

    ret = ptrUTIMER->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to stop \n", channel);
    } else {
        printf("utimer channel %" PRIu8 " :timer stopped\n", channel);
    }

error_trigger_mode_poweroff:

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power off \n", channel);
    }

error_trigger_mode_uninstall:

    ret = ptrUTIMER->Uninitialize(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to un-initialize \n", channel);
    }

    printf("*** demo application: trigger mode completed *** \r\n\n");
}
#endif

#ifdef BOARD_CAPTURE_MODE_UTIMER_INSTANCE
static volatile uint32_t cb_capture_status;
/**
 * @function    void utimer_capture_mode_cb_func(event)
 * @brief       utimer capture mode callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_capture_mode_cb_func(uint8_t event)
{
    if (event == ARM_UTIMER_EVENT_CAPTURE_A) {
        cb_capture_status++;
    }

    ptrCapt0GPO->SetValue(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
}

/**
 * @function    void utimer_capture_mode_app(void)
 * @brief       utimer capture mode application
 * @note        none
 * @param       none
 * @retval      none
 */
static void utimer_capture_mode_app(void)
{
    int32_t  ret;
    uint8_t  channel = BOARD_CAPTURE_MODE_UTIMER_INSTANCE;
    uint32_t count_array[2];

    ARM_UTIMER_TRIGGER_CONFIG trig_config = {.triggerTarget = ARM_UTIMER_TRIGGER_CAPTURE_A,
                                             .triggerSrc    = ARM_UTIMER_SRC_1,
                                             .trigger       = ARM_UTIMER_SRC1_DRIVE_A_RISING_B_0};

    /*
     * utimer channel 4 (E7) and channel 3 (E1C) is configured for utimer input capture mode
     * (selected driver A, double buffer). chan_event_a_rising_b_0 event from pinmux is used to
     * trigger input capture counter value.
     */

    printf("*** utimer demo application for capture mode started ***\n");
    /* For E7:
     * System CLOCK frequency (F)= 400Mhz
     *
     * Time for 1 count T = 1/F = 1/(400*10^6) = 0.0025 * 10^-6
     *
     * To Increment or Decrement Timer by 1 count, takes 0.0025 micro sec
     *
     * So count for 1 sec = 1/(0.0025*(10^-6)) = 400000000
     *
     * DEC = 400000000
     * HEX = 0x17D78400
     *
     * For E1C:
     * System CLOCK frequency (F)= 160Mhz
     *
     * Time for 1 count T = 1/F = 1/(160*10^6) = 6.25 * 10^-9
     *
     * To Increment or Decrement Timer by 1 count, takes 6.25 nano sec
     *
     * So count for 1000 sec = (1000*(10^-3))/(6.25*(10^-9)) = 160000000
     */
    count_array[0] = 0;                                         /*< initial counter value >*/
    count_array[1] = BOARD_UTIMER_1000_MILLI_SEC_COUNTER_VALUE; /*< over flow count value >*/

    /* GPIO pin confg */
    ret            = gpio_init(ARM_UTIMER_MODE_CAPTURING);
    if (ret) {
        printf("gpio init failed\n");
    }

    ret = ptrUTIMER->Initialize(channel, utimer_capture_mode_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power up \n", channel);
        goto error_capture_mode_uninstall;
    }

    ret = ptrUTIMER->ConfigCounter(channel, ARM_UTIMER_MODE_CAPTURING, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " mode configuration failed \n", channel);
        goto error_capture_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR, count_array[0]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_capture_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR_PTR, count_array[1]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_capture_mode_poweroff;
    }

    /* Config Trigger for counter start using chan_event_a_rising_b_0 */
    ret = ptrUTIMER->ConfigTrigger(channel, &trig_config);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " trigger configuration failed \n", channel);
        goto error_capture_mode_poweroff;
    } else {
        printf("utimer channel %" PRIu8 " triggered for counter start \n", channel);
    }

    ret = ptrUTIMER->Start(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to start \n", channel);
        goto error_capture_mode_poweroff;
    } else {
        printf("utimer channel '%" PRIu8 "': timer started\n", channel);
    }

    for (int index = 0; index < 3; index++) {
        /* Delay of 100 ms */
        UTIMER_DELAY_MS(100U);
        ptrCapt0GPO->SetValue(BOARD_UT_CAPTURE_MODE_GPO0_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        if ((ret != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to configure\n");
        }

        while (1) {
            if (cb_capture_status) {
                printf("current counter value is captured\n");
                cb_capture_status = 0;
                break;
            }
        }
    }

    ret = ptrUTIMER->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to stop \n", channel);
    } else {
        printf("utimer channel %" PRIu8 " :timer stopped \n", channel);
    }

    printf("counter value at capture a : 0x%" PRIu32 " \n",
           ptrUTIMER->GetCount(channel, ARM_UTIMER_CAPTURE_A));
    printf("counter value at capture a buf1 : 0x%" PRIu32 " \n",
           ptrUTIMER->GetCount(channel, ARM_UTIMER_CAPTURE_A_BUF1));
    printf("counter value at capture a buf2 : 0x%" PRIu32 " \n",
           ptrUTIMER->GetCount(channel, ARM_UTIMER_CAPTURE_A_BUF2));

error_capture_mode_poweroff:

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power off \n", channel);
    }

error_capture_mode_uninstall:

    ret = ptrUTIMER->Uninitialize(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to un-initialize \n", channel);
    }

    printf("*** demo application: capture mode completed *** \r\n\n");
}
#endif

#ifdef BOARD_COMPARE_MODE_UTIMER_INSTANCE
static volatile uint32_t cb_compare_a_status     ;
static volatile uint32_t cb_compare_a_buf1_status;
static volatile uint32_t cb_compare_a_buf2_status;
static volatile uint32_t cb_overflow_status      ;

/**
 * @function    void utimer_compare_mode_cb_func(event)
 * @brief       utimer compare mode callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_compare_mode_cb_func(uint8_t event)
{
    if (event == ARM_UTIMER_EVENT_COMPARE_A) {
        cb_compare_a_status = 1;
    }
    if (event == ARM_UTIMER_EVENT_COMPARE_A_BUF1) {
        cb_compare_a_buf1_status = 1;
    }
    if (event == ARM_UTIMER_EVENT_COMPARE_A_BUF2) {
        cb_compare_a_buf2_status = 1;
    }
    if (event == ARM_UTIMER_EVENT_OVER_FLOW) {
        cb_overflow_status = 1;
    }
}

/**
 * @function    void utimer_compare_mode_app(void)
 * @brief       utimer compare mode application
 * @note        none
 * @param       none
 * @retval      none
 */
static void utimer_compare_mode_app(void)
{
    int32_t  ret;
    uint8_t  channel = BOARD_COMPARE_MODE_UTIMER_INSTANCE;
    uint32_t count_array[5];

    /*
     * utimer channel 5 is configured for utimer compare mode (driver A, double buffer is enabled).
     * observe driver A output signal from P1_2.
     */
    printf("*** utimer demo application for compare mode started ***\n");
    /* For E7:
     * System CLOCK frequency (F)= 400Mhz
     *
     * Time for 1 count T = 1/F = 1/(400*10^6) = 0.0025 * 10^-6
     *
     * To Increment or Decrement Timer by 1 count, takes 0.0025 micro sec
     *
     * So count for 1 sec = 1/(0.0025*(10^-6)) = 400000000
     * DEC = 400000000
     * HEX = 0x17D78400
     *
     * So count for 250ms = (250*(10^-3)/(0.0025*(10^-6)) = 100000000
     * DEC = 100000000
     * HEX = 0x5F5E100
     *
     * So count for 500ms = (500*(10^-3)/(0.0025*(10^-6)) = 200000000
     * DEC = 200000000
     * HEX = 0xBEBC200
     *
     * So count for 750ms = (750*(10^-3)/(0.0025*(10^-6)) = 300000000
     * DEC = 300000000
     * HEX = 0x11E1A300
     */
    count_array[0] = 0x000000000;                               /*< initial counter value >*/
    count_array[1] = BOARD_UTIMER_1000_MILLI_SEC_COUNTER_VALUE; /*< over flow count value >*/
    count_array[2] = BOARD_UTIMER_250_MILLI_SEC_COUNTER_VALUE;  /*< compare a/b value>*/
    count_array[3] = BOARD_UTIMER_500_MILLI_SEC_COUNTER_VALUE;  /*< compare a/b buf1 value>*/
    count_array[4] = BOARD_UTIMER_750_MILLI_SEC_COUNTER_VALUE;  /*< compare a/b buf2 value>*/

    ret            = ptrUTIMER->Initialize(channel, utimer_compare_mode_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power up \n", channel);
        goto error_compare_mode_uninstall;
    }

    ret = ptrUTIMER->ConfigCounter(channel, ARM_UTIMER_MODE_COMPARING, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " mode configuration failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR, count_array[0]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR_PTR, count_array[1]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_COMPARE_A, count_array[2]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_COMPARE_A_BUF1, count_array[3]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_COMPARE_A_BUF2, count_array[4]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->Start(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to start \n", channel);
        goto error_compare_mode_poweroff;
    } else {
        printf("utimer channel %" PRIu8 " :timer started\n", channel);
    }

    for (int index = 0; index <= 11; index++) {
        while (1) {
            if (cb_compare_a_status) {
                cb_compare_a_status = 0;
                printf("compare_a reg value is matched to counter value\n");
                break;
            }
            if (cb_compare_a_buf1_status) {
                cb_compare_a_buf1_status = 0;
                printf("compare_a_buf1 reg value is matched to counter value\n");
                break;
            }
            if (cb_compare_a_buf2_status) {
                cb_compare_a_buf2_status = 0;
                printf("compare_a_buf2 reg value is matched to counter value\n");
                break;
            }
            if (cb_overflow_status) {
                cb_overflow_status = 0;
                printf("Interrupt: Overflow occurred\n");
                break;
            }
        }
    }

    ret = ptrUTIMER->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to stop \n", channel);
    } else {
        printf("utimer channel %" PRIu8 ": timer stopped\n", channel);
    }

error_compare_mode_poweroff:

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power off \n", channel);
    }

error_compare_mode_uninstall:

    ret = ptrUTIMER->Uninitialize(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to un-initialize \n", channel);
    }

    printf("*** demo application: compare mode completed *** \r\n\n");
}
#endif

int main()
{
    int32_t ret;
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        while (1) {
        }
    }
#endif

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
#else
    /*
     * NOTE: The UTIMER pins used in this test application are not configured
     * in the board support library. Therefore, pins are configured manually here.
     */
    ret = board_utimer_pins_config();
#endif
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return ret;
    }

#ifdef BOARD_BASIC_MODE_UTIMER_INSTANCE
    utimer_basic_mode_app();
#endif
#ifdef BOARD_BUFFER_MODE_UTIMER_INSTANCE
    utimer_buffering_mode_app();
#endif
#ifdef BOARD_TRIGGER_MODE_UTIMER_INSTANCE
    utimer_trigger_mode_app();
#endif
#ifdef BOARD_CAPTURE_MODE_UTIMER_INSTANCE
    utimer_capture_mode_app();
#endif
#ifdef BOARD_COMPARE_MODE_UTIMER_INSTANCE
    utimer_compare_mode_app();
#endif
}
