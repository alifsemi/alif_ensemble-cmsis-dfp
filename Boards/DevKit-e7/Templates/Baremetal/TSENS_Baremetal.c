/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */
/**************************************************************************//**
 * @file     : TSENS_Baremetal.c
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 21-AUG-2023
 * @brief    : Baremetal demo application code for ADC driver temperature sensor
 *              - Internal input of temperature  in analog signal corresponding
 *                output is digital value.
 *              - Converted digital value are stored in user provided memory
 *                address.
 *
 *            Hardware Connection:
 *            Common temperature sensor is internally connected to ADC12 6th channel
 *            of each instance.
 *            no hardware setup required.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include "system_utils.h"

/* include for ADC Driver */
#include "Driver_ADC.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* single shot conversion scan use ARM_ADC_SINGLE_SHOT_CH_CONV*/
/* continuous conversion scan use ARM_ADC_CONTINOUS_CH_CONV */

//#define ADC_CONVERSION    ARM_ADC_SINGLE_SHOT_CH_CONV
#define ADC_CONVERSION    ARM_ADC_CONTINOUS_CH_CONV

/* Instance for ADC12 */
extern ARM_DRIVER_ADC Driver_ADC122;
static ARM_DRIVER_ADC *ADCdrv = &Driver_ADC122;

#define TEMPERATURE_SENSOR                  ARM_ADC_CHANNEL_6
#define COMP_A_THLD_VALUE                   (0X00)                                                /* Comparator A threshold value */
#define COMP_B_THLD_VALUE                   (0x00)                                                /* Comparator B threshold value */

#define NUM_CHANNELS             (8)

/* store comparator result */
uint32_t comp_value[6] = {0};

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
    if (event & ARM_ADC_EVENT_CONVERSION_COMPLETE)
    {
        num_samples += 1;

        /* Store the value for the respected channels */
        adc_sample[channel] = sample_output;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A)
    {
        comp_value[0] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B)
    {
        comp_value[1] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A)
    {
        comp_value[2] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B)
    {
        comp_value[3] += 1;
    }
    if(event & ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B)
    {
        comp_value[4] += 1;
    }
    if(event & ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B)
    {
        comp_value[5] += 1;
    }
}

/**
 *    @func         : void TSENS_demo()
 *    @brief        : Tsens demo
 *                  - test to verify the temperature sensor of adc.
 *                  - Internal input of temperature  in analog signal corresponding
 *                    output is digital value.
 *                  - converted value is the allocated user memory address.
 *    @return       : NONE
*/
void TSENS_demo()
{

    uint32_t events   = 0;
    uint32_t ret      = 0;
    ARM_DRIVER_VERSION version;
    ARM_ADC_CAPABILITIES capabilities;

    printf("\r\n >>> ADC demo starting up!!! <<< \r\n");

    version = ADCdrv->GetVersion();
    printf("\r\n ADC version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize ADC driver */
    ret = ADCdrv->Initialize(adc_conversion_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC init failed\n");
        return;
    }

    /* Power control ADC */
    ret = ADCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Power up failed\n");
        goto error_uninitialize;
    }

#if (ADC_CONVERSION == ARM_ADC_SINGLE_SHOT_CH_CONV)

    /* set conversion mode */
    ret = ADCdrv->Control(ARM_ADC_CONVERSION_MODE_CTRL, ADC_CONVERSION);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Comparator failed\n");
        goto error_poweroff;
    }

    /* set initial channel */
    ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, TEMPERATURE_SENSOR);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC channel failed\n");
        goto error_poweroff;
    }
#endif

#if (ADC_CONVERSION == ARM_ADC_CONTINOUS_CH_CONV)

    /* set conversion mode */
    ret = ADCdrv->Control(ARM_ADC_CONVERSION_MODE_CTRL, ADC_CONVERSION);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC set conversion mode failed\n");
        goto error_poweroff;
    }

    /* set channel */
    ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, TEMPERATURE_SENSOR);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC set initial channel failed\n");
        goto error_poweroff;
    }
#endif

    /* set comparator a value */
    ret = ADCdrv->Control(ARM_ADC_COMPARATOR_A, COMP_A_THLD_VALUE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC set Comparator A  threshold failed\n");
        goto error_poweroff;
    }

    /* set comparator b value */
    ret = ADCdrv->Control(ARM_ADC_COMPARATOR_B, COMP_B_THLD_VALUE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC set Comparator B threshold failed\n");
        goto error_poweroff;
    }

    /* select the threshold comparison */
    ret = ADCdrv->Control(ARM_ADC_THRESHOLD_COMPARISON, ARM_ADC_ABOVE_A_AND_ABOVE_B);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Threshold comparison failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%X <<<\n",(uint32_t)adc_sample);
    /* Start ADC */
    ret = ADCdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Start failed\n");
        goto error_poweroff;
    }

    /* wait for timeout */
    if (ADC_CONVERSION == ARM_ADC_CONTINOUS_CH_CONV)
    {
        while(!(num_samples == 1000));
    }
    else
    {
        /* single shot conversion */
        while(!(num_samples == 1));
    }

    /* Stop ADC */
    ret = ADCdrv->Stop();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Stop failed\n");
        goto error_poweroff;
    }

    printf("\n >>> ADC conversion completed \n");
    printf(" Converted value are stored in user allocated memory address.\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    while(1);

error_poweroff:

        /* Power off ADC peripheral */
        ret = ADCdrv->PowerControl(ARM_POWER_OFF);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: ADC Power OFF failed.\r\n");
        }

error_uninitialize:

        /* Un-initialize ADC driver */
        ret = ADCdrv->Uninitialize();
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: ADC Uninitialize failed.\r\n");
        }

        printf("\r\n ADC demo exiting...\r\n");
}

/* Define main entry point.  */
int main()
{
    #if defined(RTE_Compiler_IO_STDOUT_User)
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
    #endif
    /* Enter the demo Application.  */
    TSENS_demo();
    return 0;
}
