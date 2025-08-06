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
 * @file     : demo_adc.c
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 22-Feb-2022
 * @brief    : Baremetal demo application code for ADC driver
 *              - Input in analog signal corresponding output is digital value.
 *              - Converted digital value are stored in user provided memory
 *                address.
 *                ADC has two conversion mode
 *                1) Single shot conversion
 *                  - single channel scan
 *                2) Continuous conversion
 *                  - single channel scan
 *                  - Multiple channel scan
 *
 *              Hardware setup:
 *              - GPIO port are reserved for ADC12 instance 0 input:
 *                P0_0(channel0)
 *                P0_1(channel1)
 *                P0_2(channel2)
 *                P0_3(channel3)
 *                P0_4(channel4)
 *                P0_5(channel5)
 *              - GPIO port are reserved for ADC12 instance 1 input:
 *                P0_6(channel0)
 *                P0_7(channel1)
 *                P1_0(channel2)
 *                P1_1(channel3)
 *                P1_2(channel4)
 *                P1_3(channel5)
 *              - GPIO port are reserved for ADC12 instance 2 input: (not supported in E1C)
 *                P1_4(channel0)
 *                P1_5(channel1)
 *                P1_6(channel2)
 *                P1_7(channel3)
 *                P2_0(channel4)
 *                P2_1(channel5)
 *
 *              - GPIO port are reserved for ADC24 instance input:
 *                P0_0 (+ve input ) and P0_4 (-ve input) channel 0
 *                P0_1 (+ve input ) and P0_5 (-ve input) channel 1
 *                P0_2 (+ve input ) and P0_6 (-ve input) channel 2
 *                P0_3 (+ve input ) and P0_7 (-ve input) channel 2
 *
 *             1. Single shot conversion (One-time scan):
 *                The ADC performs a one-time conversion of the selected channel(s).
 *              - Single Channel Scan (Only one specific channel):
 *                - The user can select the desired channel using:
 *                  ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, ARM_ADC_CHANNEL_#);
 *                  use ARM_ADC_CHANNEL_#, where # denotes the channel number.
 *
 *             2. Continuous Conversion (Repeated Scanning):
 *                The ADC repeatedly scans channels based on the configuration.
 *              - Single channel scan (Scans only one specific channel repeatedly)
 *                - The user can select the desired channel using:
 *                  ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, ARM_ADC_CHANNEL_#);
 *                  use ARM_ADC_CHANNEL_#, where # denotes the channel number.
 *
 *              - Multiple channel scan
 *                - Scans all available channels continuously (if no channels are masked).
 *                - To mask specific channels, use the following API:
 *                  ADCdrv->Control(ARM_ADC_SEQUENCER_MSK_CH_CTRL, MASK_CHANNEL);
 *
 *             - SEQUENCER_INIT:
 *                - Specifies the input channel from which the ADC will start conversion.
 *                - By default, all channels are unmasked, and the ADC is configured for
 *                  single-shot conversion.
 *
 *              Comparator
 *              - comparing channels for both the scan for below threshold
 *                " Above / below threshold A
 *                " Above / below threshold B
 *                " Between/outside threshold A and B
 *
 *              ADC configurations for Demo testApp:
 *                Single channel scan(Default scan) (For E1C use pin P0_0)
 *                 - GPIO pin P1_4 are connected to Regulated DC Power supply.
 *                    DC Power supply:
 *                     - +ve connected to P1_4 (ADC2 channel 0) at 1.0V
 *                     - -ve connect to GND.
 *
 *                Continuous Channel scan
 *                - Used ADC instance 2,all channels(0-5) are connected to dc supply
 *                - channel 2 and 4 are masked using MASK_CHANNEL macro.
 *                - GND both dc supply channel -ve
 *              Differential input
 *              -ADC12 (For E1C use pin P0_0 and P0_1)
 *                GPIO pin P1_4 and P1_5 are connected to Regulated DC Power supply.
 *                2 channel DC Power supply:
 *                - +ve connected to P1_4 (ADC122 channel 0) at 1.0V and
 *                  +ve connected to P1_5 (ADC122 channel 1) at 0.4V
 *                - -ve connect to GND.
 *              -ADC24
                  GPIO pin P1_4 and P1_5 are connected to Regulated DC Power supply.
 *                2 channel DC Power supply:
 *                - +ve connected to P0_0 (ADC122 channel 0) at 1.0V and
 *                  +ve connected to P0_4 (ADC122 channel 1) at 0.5V
 *                - -ve connect to GND.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <inttypes.h>

/* include for ADC Driver */
#include "Driver_ADC.h"

/* PINMUX include */
#include "pinconf.h"
#include "board_config.h"

#include "se_services_port.h"
#include "RTE_Components.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

// Set to 0: Use application-defined ADC pin configuration (via board_adc_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* single shot conversion scan use ARM_ADC_SINGLE_SHOT_CH_CONV*/
/* continuous conversion scan use ARM_ADC_CONTINOUS_CH_CONV */

#define ADC_CONVERSION                 ARM_ADC_SINGLE_SHOT_CH_CONV
// #define ADC_CONVERSION    ARM_ADC_CONTINOUS_CH_CONV

/* For rotating through one channel use ARM_ADC_FIXED_CHANNEL_SCAN */
/* For continuous rotating through all channel use ARM_ADC_MULTIPLE_CHANNEL_SCAN*/

/* @note : When conversion is selected ARM_ADC_CH_SINGLE_SHOT_SCAN
 *         ADC_SCAN should be in ARM_ADC_SINGLE_CH_SCAN
 * */
#define ADC_SCAN                       ARM_ADC_SINGLE_CH_SCAN
// #define ADC_SCAN       ARM_ADC_MULTIPLE_CH_SCAN

/* Macro */
#define ADC_12                         1
#define ADC_24                         0

/* For ADC12 use ADC_INSTANCE ADC12  */
/* For ADC24 use ADC_INSTANCE ADC24  */
#define ADC_INSTANCE                   ADC_12
// #define ADC_INSTANCE         ADC_24

#if (ADC_INSTANCE == ADC_12)
/* Instance for ADC12 */
extern ARM_DRIVER_ADC  ARM_Driver_ADC12(BOARD_ADC12_INSTANCE);
static ARM_DRIVER_ADC *ADCdrv = &ARM_Driver_ADC12(BOARD_ADC12_INSTANCE);
#else
/* Instance for ADC24 */
extern ARM_DRIVER_ADC  Driver_ADC24;
static ARM_DRIVER_ADC *ADCdrv = &Driver_ADC24;
#endif

#define COMP_A_THLD_VALUE (0X00) /* Comparator A threshold value */
#define COMP_B_THLD_VALUE (0x00) /* Comparator B threshold value */
#define MASK_CHANNEL                                                                               \
    (ARM_ADC_MASK_CHANNEL_2 | ARM_ADC_MASK_CHANNEL_4) /* Masking particular channels  */

#define MAX_NUM_THRESHOLD (6)
#define NUM_CHANNELS      (8)

/* store comparator result */
uint32_t comp_value[MAX_NUM_THRESHOLD] = {0};

/* Demo purpose adc_sample*/
uint32_t adc_sample[NUM_CHANNELS];

volatile uint32_t num_samples;

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_adc_pins_config(void)
 * @brief   Configure ADC122 and ADC24 pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_adc_pins_config(void)
{
    int32_t ret = 0U;
    if (ADC_INSTANCE == ADC_12) {
        if (ADC_SCAN == ARM_ADC_SINGLE_CH_SCAN) {
            /* ADC122 channel 0 */
            ret = pinconf_set(PORT_(BOARD_ADC12_CH0_GPIO_PORT),
                              BOARD_ADC12_CH0_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
        }

        if (ADC_SCAN == ARM_ADC_MULTIPLE_CH_SCAN) {
            /* ADC122 channel 0 */
            ret = pinconf_set(PORT_(BOARD_ADC12_CH0_GPIO_PORT),
                              BOARD_ADC12_CH0_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* channel 1 */
            ret = pinconf_set(PORT_(BOARD_ADC12_CH1_GPIO_PORT),
                              BOARD_ADC12_CH1_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* channel 2 */
            ret = pinconf_set(PORT_(BOARD_ADC12_CH2_GPIO_PORT),
                              BOARD_ADC12_CH2_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* channel 3 */
            ret = pinconf_set(PORT_(BOARD_ADC12_CH3_GPIO_PORT),
                              BOARD_ADC12_CH3_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* channel 4 */
            ret = pinconf_set(PORT_(BOARD_ADC12_CH4_GPIO_PORT),
                              BOARD_ADC12_CH4_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* channel 5 */
            ret = pinconf_set(PORT_(BOARD_ADC12_CH5_GPIO_PORT),
                              BOARD_ADC12_CH5_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
        }
    }

    if (ADC_INSTANCE == ADC_24) {
        if (ADC_SCAN == ARM_ADC_SINGLE_CH_SCAN) {
            /* ADC24 channel 0 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH0_POS_GPIO_PORT),
                              BOARD_ADC24_CH0_POS_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
            /* ADC24 channel 0 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH0_NEG_GPIO_PORT),
                              BOARD_ADC24_CH0_NEG_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
        }

        if (ADC_SCAN == ARM_ADC_MULTIPLE_CH_SCAN) {
            /* ADC24 channel 0 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH0_POS_GPIO_PORT),
                              BOARD_ADC24_CH0_POS_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
            /* ADC24 channel 0 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH0_NEG_GPIO_PORT),
                              BOARD_ADC24_CH0_NEG_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* ADC24 channel 1 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH1_POS_GPIO_PORT),
                              BOARD_ADC24_CH1_POS_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
            /* ADC24 channel 1 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH1_NEG_GPIO_PORT),
                              BOARD_ADC24_CH1_NEG_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* ADC24 channel 2 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH2_POS_GPIO_PORT),
                              BOARD_ADC24_CH2_POS_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
            /* ADC24 channel 2 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH2_NEG_GPIO_PORT),
                              BOARD_ADC24_CH2_NEG_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }

            /* ADC24 channel 3 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH3_POS_GPIO_PORT),
                              BOARD_ADC24_CH3_POS_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
            /* ADC24 channel 3 */
            ret = pinconf_set(PORT_(BOARD_ADC24_CH3_NEG_GPIO_PORT),
                              BOARD_ADC24_CH3_NEG_GPIO_PIN,
                              PINMUX_ALTERNATE_FUNCTION_7,
                              PADCTRL_READ_ENABLE);
            if (ret) {
                printf("ERROR: Failed to configure PINMUX \r\n");
                return ret;
            }
        }
    }
    return ret;
}
#endif

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
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A) {
        comp_value[0] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B) {
        comp_value[1] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A) {
        comp_value[2] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B) {
        comp_value[3] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B) {
        comp_value[4] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B) {
        comp_value[5] += 1;
    }
}

/**
 *    @func         : void ADC_demo()
 *    @brief        : ADC demo
 *                  - test to verify the adc.
 *                  - input analog signal corresponding convert into digital value
 *                  - converted value is the allocated user memory address.
 *    @return       : NONE
 */
void ADC_demo()
{
    int32_t            ret        = 0;
    uint32_t           error_code = SERVICES_REQ_SUCCESS;
    uint32_t           service_error_code;
    ARM_DRIVER_VERSION version;

    /* Initialize the SE services */
    se_services_port_init();

    /* enable the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_CLK_160M,
                                              /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: clk enable = %" PRId32 "\n", error_code);
    }

    printf("\r\n >>> ADC demo starting up!!! <<< \r\n");

    version = ADCdrv->GetVersion();
    printf("\r\n ADC version api:%" PRIx16 " driver:%" PRIx16 "...\r\n", version.api, version.drv);

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The ADC122 and ADC24 pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_adc_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#endif

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

#if (ADC_CONVERSION == ARM_ADC_SINGLE_SHOT_CH_CONV)

    /* set conversion mode */
    ret = ADCdrv->Control(ARM_ADC_CONVERSION_MODE_CTRL, ADC_CONVERSION);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Comparator failed\n");
        goto error_poweroff;
    }

    /* set initial channel */
    ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, ARM_ADC_CHANNEL_0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC channel failed\n");
        goto error_poweroff;
    }
#endif

#if (ADC_CONVERSION == ARM_ADC_CONTINOUS_CH_CONV)

    /* set conversion mode */
    ret = ADCdrv->Control(ARM_ADC_CONVERSION_MODE_CTRL, ADC_CONVERSION);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC setting conversion mode failed\n");
        goto error_poweroff;
    }

    if (ADC_SCAN == ARM_ADC_SINGLE_CH_SCAN) {
        /* set channel */
        ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, ARM_ADC_CHANNEL_0);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: ADC setting channel failed\n");
            goto error_poweroff;
        }
    } else /* Multiple channel scan */ {
        /* set sequencer controller */
        ret = ADCdrv->Control(ARM_ADC_SEQUENCER_CTRL, ARM_ADC_MULTIPLE_CH_SCAN);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: ADC sequencer controller failed\n");
            goto error_poweroff;
        }

        /* set channel */
        ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, ARM_ADC_CHANNEL_0);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: ADC setting channel failed\n");
            goto error_poweroff;
        }

        /* Masking the channel */
        ret = ADCdrv->Control(ARM_ADC_SEQUENCER_MSK_CH_CTRL, MASK_CHANNEL);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: ADC sequencer masking channel failed\n");
            goto error_poweroff;
        }
    }
#endif

    /* set comparator a value */
    ret = ADCdrv->Control(ARM_ADC_COMPARATOR_A, COMP_A_THLD_VALUE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC set Comparator A threshold failed\n");
        goto error_poweroff;
    }

    /* set comparator b value */
    ret = ADCdrv->Control(ARM_ADC_COMPARATOR_B, COMP_B_THLD_VALUE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC set comparator B  threshold failed\n");
        goto error_poweroff;
    }

    /* select the threshold comparison */
    ret = ADCdrv->Control(ARM_ADC_THRESHOLD_COMPARISON, ARM_ADC_ABOVE_A_AND_ABOVE_B);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Threshold comparison failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%" PRIx32 " <<<\n", (uint32_t) adc_sample);
    /* Start ADC */
    ret = ADCdrv->Start();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Start failed\n");
        goto error_poweroff;
    }

    /* wait for timeout */
    if (ADC_CONVERSION == ARM_ADC_CONTINOUS_CH_CONV) {
        while (num_samples < 1000) {
        }
    } else {
        /* single shot conversion */
        while (num_samples < 1) {
        }
    }

    /* Stop ADC */
    ret = ADCdrv->Stop();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Stop failed\n");
        goto error_poweroff;
    }

    printf("\n >>> ADC conversion completed \n");
    printf(" Converted value are stored in user allocated memory address.\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_poweroff:

    /* Power off ADC peripheral */
    ret = ADCdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Power OFF failed.\r\n");
    }
    /* disable the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_CLK_160M,
                                              /*bool enable   */ false,
                                              &service_error_code);
    if (error_code) {
        printf("SE: clk enable = %" PRId32 "\n", error_code);
    }

error_uninitialize:

    /* Un-initialize ADC driver */
    ret = ADCdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Uninitialize failed.\r\n");
    }

    printf("\r\n ADC demo exiting...\r\n");
}

/* Define main entry point.  */
int main(void)
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
    ADC_demo();
    return 0;
}
