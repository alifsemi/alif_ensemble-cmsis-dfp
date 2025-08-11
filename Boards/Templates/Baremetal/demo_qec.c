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
 * @file     : demo_qec.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 21-July-2023
 * @brief    : Baremetal demo application for QEC0 channel.
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include "Driver_UTIMER.h"
#include "Driver_IO.h"
#include "pinconf.h"
#include "board_config.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

// Set to 0: Use application-defined QEC pin configuration (via board_qec_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* UTIMER0 Driver instance */
extern ARM_DRIVER_UTIMER Driver_UTIMER0;
ARM_DRIVER_UTIMER       *ptrUTIMER = &Driver_UTIMER0;

/* BOARD_QEC_X_GPO_GPIO_PORT Driver instance */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_QEC_X_GPO_GPIO_PORT);
ARM_DRIVER_GPIO       *ptrQECxGPIO = &ARM_Driver_GPIO_(BOARD_QEC_X_GPO_GPIO_PORT);

/* BOARD_QEC_Y_GPO_GPIO_PORT Driver instance */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_QEC_Y_GPO_GPIO_PORT);
ARM_DRIVER_GPIO       *ptrQECyGPIO = &ARM_Driver_GPIO_(BOARD_QEC_Y_GPO_GPIO_PORT);

/* BOARD_QEC_Z_GPO_GPIO_PORT Driver instance */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_QEC_Z_GPO_GPIO_PORT);
ARM_DRIVER_GPIO       *ptrQECzGPIO = &ARM_Driver_GPIO_(BOARD_QEC_Z_GPO_GPIO_PORT);

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @function    int board_qec_pins_config(void)
 * @brief       QEC h/w pin init using pinmux driver
 * @note        none
 * @param       none
 * @retval      execution status
 */
static int32_t board_qec_pins_config(void)
{
    int32_t ret;

    ret = pinconf_set(PORT_(BOARD_QEC0_X_B_GPIO_PORT),
                      BOARD_QEC0_X_B_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_3,
                      PADCTRL_READ_ENABLE | PADCTRL_SCHMITT_TRIGGER_ENABLE |
                          PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("Error in PINMUX: %" PRId32 "\n", ret);
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_QEC0_Y_B_GPIO_PORT),
                      BOARD_QEC0_Y_B_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_3,
                      PADCTRL_READ_ENABLE | PADCTRL_SCHMITT_TRIGGER_ENABLE |
                          PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("Error in PINMUX: %" PRId32 "\n", ret);
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_QEC0_Z_B_GPIO_PORT),
                      BOARD_QEC0_Z_B_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_3,
                      PADCTRL_READ_ENABLE | PADCTRL_SCHMITT_TRIGGER_ENABLE |
                          PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("Error in PINMUX: %" PRId32 "\n", ret);
        return ret;
    }

    /* Config as GPIO */
    ret = pinconf_set(PORT_(BOARD_QEC_X_GPO_GPIO_PORT),
                      BOARD_QEC_X_GPO_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_0,
                      PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("Error in PINMUX: %" PRId32 "\n", ret);
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_QEC_Y_GPO_GPIO_PORT),
                      BOARD_QEC_Y_GPO_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_0,
                      PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("Error in PINMUX: %" PRId32 "\n", ret);
        return ret;
    }

    ret = pinconf_set(PORT_(BOARD_QEC_Z_GPO_GPIO_PORT),
                      BOARD_QEC_Z_GPO_GPIO_PIN,
                      PINMUX_ALTERNATE_FUNCTION_0,
                      PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("Error in PINMUX: %" PRId32 "\n", ret);
        return ret;
    }

    return 0;
}
#endif

/**
 * @function    int gpio_init(void)
 * @brief       GPIO initialization using gpio driver
 * @note        none
 * @param       none
 * @retval      execution status
 */
static int32_t gpio_init(void)
{
    int32_t ret = ARM_DRIVER_OK;

    ret         = ptrQECxGPIO->Initialize(BOARD_QEC_X_GPO_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize GPIO1_PIN0 as GPIO\n");
        return -1;
    }
    ret = ptrQECxGPIO->PowerControl(BOARD_QEC_X_GPO_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to Power up GPIO1_PIN0\n");
        return -1;
    }
    ret = ptrQECxGPIO->SetDirection(BOARD_QEC_X_GPO_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set direction for GPIO1_PIN0\n");
        return -1;
    }
    ret = ptrQECxGPIO->SetValue(BOARD_QEC_X_GPO_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set value for GPIO1_PIN0\n");
        return -1;
    }

    ret = ptrQECyGPIO->Initialize(BOARD_QEC_Y_GPO_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize GPIO1_PIN1 as GPIO\n");
        return -1;
    }
    ret = ptrQECyGPIO->PowerControl(BOARD_QEC_Y_GPO_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to Power up GPIO1_PIN1\n");
        return -1;
    }
    ret = ptrQECyGPIO->SetDirection(BOARD_QEC_Y_GPO_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set direction for GPIO1_PIN1\n");
        return -1;
    }
    ret = ptrQECyGPIO->SetValue(BOARD_QEC_Y_GPO_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set value for GPIO1_PIN1\n");
        return -1;
    }

    ret = ptrQECzGPIO->Initialize(BOARD_QEC_Z_GPO_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize GPIO1_PIN2 as GPIO\n");
        return -1;
    }
    ret = ptrQECzGPIO->PowerControl(BOARD_QEC_Z_GPO_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to Power up GPIO1_PIN2\n");
        return -1;
    }
    ret = ptrQECzGPIO->SetDirection(BOARD_QEC_Z_GPO_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set direction for GPIO1_PIN2\n");
        return -1;
    }
    ret = ptrQECzGPIO->SetValue(BOARD_QEC_Z_GPO_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set value for GPIO1_PIN2\n");
        return -1;
    }

    return 0;
}

/**
 * @function    void qec0_app(void)
 * @brief       QEC0 demo application
 * @note        none
 * @param       none
 * @retval      none
 */
static void qec0_app(void)
{
    int32_t  ret;
    uint8_t  channel                         = BOARD_ENCODER_UTIMER_INSTANCE;
    uint32_t init_count                      = 0;

    ARM_UTIMER_TRIGGER_CONFIG upcount_trig   = {.triggerTarget = ARM_UTIMER_TRIGGER_UPCOUNT,
                                                .triggerSrc    = ARM_UTIMER_SRC_0,
                                                .trigger       = ARM_UTIMER_SRC0_TRIG0_RISING};

    ARM_UTIMER_TRIGGER_CONFIG downcount_trig = {.triggerTarget = ARM_UTIMER_TRIGGER_DOWNCOUNT,
                                                .triggerSrc    = ARM_UTIMER_SRC_0,
                                                .trigger       = ARM_UTIMER_SRC0_TRIG1_RISING};

    ARM_UTIMER_TRIGGER_CONFIG clear_trig     = {.triggerTarget = ARM_UTIMER_TRIGGER_CAPTURE_A,
                                                .triggerSrc    = ARM_UTIMER_SRC_0,
                                                .trigger       = ARM_UTIMER_SRC0_TRIG2_RISING};

    /*
     * utimer channel 12 is configured as QEC0.
     * For testing purpose, QEC inputs are connected to gpio's. And GPIO's are toggled accordingly.
     *
     * H/W connection : short P1_0 and P8_4, short P1_1 and P8_5, short P1_2 and P8_6.
     **/

    printf("*** QEC demo application started ***\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
#else
    /* Configure pin mux manually */
    ret = board_qec_pins_config();
#endif
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Pin configuration failed: %" PRId32 "\n", ret);
        return;
    }

    ret = gpio_init();
    if (ret) {
        printf("gpio init failed\n");
        return;
    }

    ret = ptrUTIMER->Initialize(channel, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power up \n", channel);
        goto error_qec_uninstall;
    }

    ret =
        ptrUTIMER->ConfigCounter(channel, ARM_UTIMER_MODE_TRIGGERING, ARM_UTIMER_COUNTER_TRIANGLE);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " mode configuration failed \n", channel);
        goto error_qec_poweroff;
    }

    ret = ptrUTIMER->SetCount(channel, ARM_UTIMER_CNTR, init_count);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        goto error_qec_poweroff;
    }

    ret = ptrUTIMER->ConfigTrigger(channel, &upcount_trig);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " trigger configuration failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("utimer channel %" PRIu8 " triggered for up count using Trig0\n", channel);
    }

    ret = ptrUTIMER->ConfigTrigger(channel, &downcount_trig);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " trigger configuration failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("utimer channel %" PRIu8 " triggered for down count using Trig1\n", channel);
    }

    ret = ptrUTIMER->ConfigTrigger(channel, &clear_trig);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " trigger configuration failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("utimer channel %" PRIu8 " triggered for counter clear using Trig3\n", channel);
    }

    /* Toggling gpio's connected to x for 20 times to increment cnt value for 10 times */
    for (int i = 0; i < 20; i++) {
        ret = ptrQECxGPIO->SetValue(BOARD_QEC_X_GPO_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set value for GPIO1_PIN0\n");
        }
    }

    printf("counter value after counter increment : %" PRIu32 "\n",
           ptrUTIMER->GetCount(channel, ARM_UTIMER_CNTR));

    /* Toggling gpio's connected to x for 10 times to increment cnt value for 5 times */
    for (int i = 0; i < 10; i++) {
        ret = ptrQECyGPIO->SetValue(BOARD_QEC_Y_GPO_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set value for GPIO1_PIN1\n");
        }
    }

    printf("counter value after counter decrement: %" PRIu32 "\n",
           ptrUTIMER->GetCount(channel, ARM_UTIMER_CNTR));

    /* Making z event as high to clear count value */
    ret = ptrQECzGPIO->SetValue(BOARD_QEC_Z_GPO_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set value for GPIO1_PIN2\n");
    }

    printf("counter value after counter clear: %" PRIu32 "\n",
           ptrUTIMER->GetCount(channel, ARM_UTIMER_CNTR));

    ret = ptrUTIMER->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to stop \n", channel);
    } else {
        printf("utimer channel %" PRIu8 " :timer stopped\n", channel);
    }

error_qec_poweroff:

    ret = ptrUTIMER->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power off \n", channel);
    }

error_qec_uninstall:

    ret = ptrUTIMER->Uninitialize(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to un-initialize \n", channel);
    }

    printf("*** demo application: QEC completed *** \r\n\n");
}

int main()
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        while (1) {
        }
    }
#endif
    qec0_app();
}
