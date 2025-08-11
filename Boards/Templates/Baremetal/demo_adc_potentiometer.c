/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */
/*******************************************************************************
 * @file     : demo_adc_potentiometer.c
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 15-SEPT-2023
 * @brief    : Baremetal demo application code for potentiometer input
 *              - Internal input of potentiometer in analog signal corresponding
 *                output is digital value.
 *              - the input from the potentiometer is internally connected to
 *                the ADC121 instance channel_1(j11 Pin 10).
 *              - the converted digital value are stored in user provided memory
 *                address.
 *
 *            Hardware Connection:
 *            Analog variable register R53 is connected internally to the ADC121
 *            channel_1 which volatge vary from 0 to 1.8V
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <inttypes.h>
#include "app_utils.h"

/* include for ADC Driver */
#include "Driver_ADC.h"
#include "board_config.h"

#include "se_services_port.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

/* single shot conversion scan use ARM_ADC_SINGLE_SHOT_CH_CONV*/

#define ADC_CONVERSION ARM_ADC_SINGLE_SHOT_CH_CONV

/* Instance for ADC12 */
extern ARM_DRIVER_ADC  ARM_Driver_ADC12(BOARD_POTENTIOMETER_ADC12_INSTANCE);
static ARM_DRIVER_ADC *ADCdrv = &ARM_Driver_ADC12(BOARD_POTENTIOMETER_ADC12_INSTANCE);

#define NUM_CHANNELS (8)

/* Demo purpose adc_sample*/
uint32_t adc_sample[NUM_CHANNELS];

volatile uint32_t num_samples = 0;

/*
 * @func   : void adc_conversion_callback(uint32_t event, uint8_t channel, uint32_t sample_output)
 * @brief  : adc conversion isr callback
 * @return : NONE
 */
static void adc_conversion_callback(uint32_t event, uint8_t channel, uint32_t sample_output)
{
    if (event & ARM_ADC_EVENT_CONVERSION_COMPLETE) {
        num_samples         += 1;

        /* Store the value for the respected channels */
        adc_sample[channel]  = sample_output;
    }
}

/**
 *    @func   : void adc_potentiometer_demo()
 *    @brief  : ADC Potentiometer demo
 *             - test to verify the potentiometer analog input
 *             - Internal input of potentiometer in analog signal corresponding
 *               output is digital value.
 *             - converted value is the allocated user memory address.
 *    @return : NONE
 */
void adc_potentiometer_demo()
{
    int32_t            ret        = 0;
    uint32_t           error_code = SERVICES_REQ_SUCCESS;
    uint32_t           service_error_code;
    ARM_DRIVER_VERSION version;

    /* Initialize the SE services */
    se_services_port_init();

    /* enable the 160 MHz clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_CLK_160M,
                                              /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        printf("SE Error: 160 MHz clk enable = %" PRId32 "\n", error_code);
        return;
    }

    printf("\t\t\n >>> ADC demo starting up!!! <<< \r\n");

    version = ADCdrv->GetVersion();
    printf("\r\n ADC version api:%" PRIx16 " driver:%" PRIx16 "...\r\n", version.api, version.drv);

    /* pinmux and configurations for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret) {
        printf("ERROR: Board pin configuration failed: %" PRId32 "\n", ret);
    }

    /* Initialize ADC driver */
    ret = ADCdrv->Initialize(adc_conversion_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC init failed\n");
        return;
    }

    /* Power control ADC */
    ret = ADCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Power up failed\n");
        goto error_uninitialize;
    }

    /* set conversion mode */
    ret = ADCdrv->Control(ARM_ADC_CONVERSION_MODE_CTRL, ADC_CONVERSION);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC select conversion mode failed\n");
        goto error_poweroff;
    }

    /* set initial channel */
    ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, BOARD_POTENTIOMETER_ADC12_INPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC channel init failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%" PRIx32 " <<<\n",
           (uint32_t) (adc_sample + BOARD_POTENTIOMETER_ADC12_INPUT));
    /* Start ADC */
    ret = ADCdrv->Start();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Start failed\n");
        goto error_poweroff;
    }

    /* wait for timeout */
    while (!(num_samples == 1)) {
    }

    printf("\n Potentiometer conversion completed \n");

    /* Stop ADC */
    ret = ADCdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Stop failed\n");
        goto error_poweroff;
    }

    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_poweroff:

    /* Power off ADC peripheral */
    ret = ADCdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize ADC driver */
    ret = ADCdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Uninitialize failed.\r\n");
    }
    /* disable the 160MHz clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_CLK_160M,
                                              /*bool enable   */ false,
                                              &service_error_code);
    if (error_code) {
        printf("SE Error: 160 MHz clk disable = %" PRId32 "\n", error_code);
        return;
    }

    printf("\r\n ADC demo exiting...\r\n");
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
    /* Enter the demo Application.  */
    adc_potentiometer_demo();
    return 0;
}
