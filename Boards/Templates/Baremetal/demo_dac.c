/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     : demo_dac.c
 * @author   : Nisarga A M
 * @email    : nisarga.am@alifsemi.com
 * @version  : V1.0.0
 * @date     : 22-Feb-2022
 * @brief    : -As DAC is 12 bit resolution, If the input value is maximum than the
                maximum DAC input value(0xFFF)then the input value will be set to
                DAC maximum input value.
               -And If the input value is equal to maximum dac input value then
                  input value will be set to 0.
               -If the input value is not greater than the maximum dac input value
                 then input value will be incremented by 1000.

               E7: Hardware Setup :
                -when the application uses DAC0 channel,then connect DAC0 to P2_2
                 GPIO pin,according to DAC input the output will be observed in P2_2
                 GPIO pin through the logic analyzer.

               -And when the application uses DAC1 channel,then connect DAC1 to
                P2_3 GPIO pin,according to DAC input the output will be observed
                in P2_3 GPIO pin through the logic analyzer.
               E1C: Hardware Setup :
               - Connect the logic analyzer to pin P2_3 to observe the DAC0 output.
                 By varying the DAC input values, the output will be displayed on
                 pin P2_3.
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include <inttypes.h>

#include "RTE_Components.h"
#include CMSIS_device_header
#include "pinconf.h"
#include "board_config.h"
#include "app_utils.h"

/* Project Includes */
/* include for DAC Driver */
#include "Driver_DAC.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

// Set to 0: Use application-defined DAC12 pin configuration (via board_dac12_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* DAC Driver instance */
extern ARM_DRIVER_DAC  ARM_Driver_DAC_(BOARD_DAC12_INSTANCE);
static ARM_DRIVER_DAC *DACdrv = &ARM_Driver_DAC_(BOARD_DAC12_INSTANCE);

/* DAC maximum resolution is 12-bit */
#define DAC_MAX_INPUT_VALUE (0xFFF)

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_dac12_pins_config(void)
 * @brief   Configure DAC12 pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_dac12_pins_config(void)
{
    int32_t status;

    /* Configure DAC120 output */
    status = pinconf_set(PORT_(BOARD_DAC120_OUT_GPIO_PORT),
                         BOARD_DAC120_OUT_GPIO_PIN,
                         BOARD_DAC120_ALTERNATE_FUNCTION,
                         PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return status;
    }

#if (RTE_DAC1)
    /* Configure DAC121 output */
    status = pinconf_set(PORT_(BOARD_DAC121_OUT_GPIO_PORT),
                         BOARD_DAC121_OUT_GPIO_PIN,
                         BOARD_DAC121_ALTERNATE_FUNCTION,
                         PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return status;
    }
#endif
    return APP_SUCCESS;
}
#endif

/**
 @fn           void dac_demo()
 @brief        DAC demo :
               This initializes the DAC. And then in a loop,
               according to the input value, the output will change.
               If the input value is maximum than the maximum DAC input
               value then the input value will be set to DAC maximum input value.
               And If the input value is equal to maximum dac input value then
               input value will be set to 0.If the input value is not greater
               than the maximum dac input value then input value will be incremented by 1000.
 @return       none
*/
static void dac_demo(void)
{
    uint32_t           input_value = 0;
    int32_t            ret         = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> DAC demo starting up!!! <<< \r\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The DAC12 pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_dac12_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#endif

    version = DACdrv->GetVersion();
    printf("\r\n DAC version api:%" PRIu16 " driver:%" PRIu16 "...\r\n", version.api, version.drv);

    /* Initialize DAC driver */
    ret = DACdrv->Initialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC init failed\n");
        return;
    }

    /* Enable the power for DAC */
    ret = DACdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Power up failed\n");
        goto error_uninitialize;
    }

    /* Set DAC IBAIS output current */
    ret = DACdrv->Control(ARM_DAC_SELECT_IBIAS_OUTPUT, ARM_DAC_1100UA_OUT_CUR);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Setting DAC output current failed failed\n");
        goto error_uninitialize;
    }

    /* Set DAC capacitance  */
    ret = DACdrv->Control(ARM_DAC_CAPACITANCE_HP_MODE, ARM_DAC_8PF_CAPACITANCE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: Setting DAC capacitance failed\n");
        goto error_uninitialize;
    }

    /* start dac */
    ret = DACdrv->Start();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Start failed\n");
        goto error_poweroff;
    }

    input_value = 0;

    while (1) {
        /* set dac input */
        ret = DACdrv->SetInput(input_value);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: DAC Set Input failed\n");
            goto error_stop;
        }

        /* Sleep for n micro second */
        sys_busy_loop_us(1000);

        /* If the input value is equal to maximum dac input value then input
           value will be set to 0 */
        if (input_value == DAC_MAX_INPUT_VALUE) {
            input_value = 0;
        } else {
            /* If the input value is not greater than the maximum dac input value then input
               value will be incremented by 1000 */
            input_value += 1000;
        }

        /* If the input value is maximum than the maximum DAC input value then the input
           value will be set to DAC maximum input value */
        if (input_value > DAC_MAX_INPUT_VALUE) {
            input_value = DAC_MAX_INPUT_VALUE;
        }
    }

error_stop:

    /* Stop the DAC driver */
    ret = DACdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Stop failed.\r\n");
    }

error_poweroff:

    /* Power off DAC peripheral */
    ret = DACdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize DAC driver */
    ret = DACdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: DAC Uninitialize failed.\r\n");
    }

    printf("\r\n XXX DAC demo exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif
    dac_demo();

    return 0;
}

/********************** (c) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
