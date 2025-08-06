/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     : demo_i2s.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 3-May-2023
 * @brief    : Test Application for I2S for E7 Devkit
 *              For HP, I2S1 is configured as master transmitter (DAC).
 *              For HE, LPI2S will be used as DAC.
 *                I2S3(ADC) is configured as master receiver SPH0645LM4H-1 device 24bit
 *              Test Application for I2S for E1C Devkit using WM8904 codec
 *                I2S0 is configured as master transmitter (DAC).
 *                LPI2S(ADC) is configured as master receiver SPH0645LM4H-1 device 24bit
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/

/*System Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* Project Includes */
#include <Driver_SAI.h>
#include <pinconf.h>
#include "board_config.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

#if BOARD_WM8904_CODEC_PRESENT
#include "WM8904_driver.h"
#endif

/* Enable this macro to play the predefined sample */
// #define DAC_PREDEFINED_SAMPLES

// Set to 0: Use application-defined I2S pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#ifdef DAC_PREDEFINED_SAMPLES
/*Audio samples */
#include "i2s_samples.h"
#endif

#if (BOARD_WM8904_CODEC_PRESENT) && !defined(RTE_Drivers_WM8904_CODEC)
#error "WM8904 codec driver not configured in RTE_Components.h"
#endif

#define NUM_SAMPLES 40000

void DAC_Init(void);
#if !defined(DAC_PREDEFINED_SAMPLES)
int32_t ADC_Init(void);
int32_t Receiver(void);

/* Buffer for ADC samples */
static uint32_t sample_buf[NUM_SAMPLES];
#endif

/* 1 to send the data stream continuously , 0 to send data only once */
#define REPEAT_TX                  1

#define DAC_SEND_COMPLETE_EVENT    (1U << 0)
#define ADC_RECEIVE_COMPLETE_EVENT (1U << 1)
#define ADC_RECEIVE_OVERFLOW_EVENT (1U << 2)

static volatile uint32_t event_flag;

static uint32_t wlen                = 24;
static uint32_t sampling_rate       = 48000; /* 48Khz audio sampling rate */

extern ARM_DRIVER_SAI  ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);
static ARM_DRIVER_SAI *i2s_dac = &ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);

#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
extern ARM_DRIVER_SAI  ARM_Driver_SAI_(BOARD_MIC_INPUT_I2S_INSTANCE);
static ARM_DRIVER_SAI *i2s_adc = &ARM_Driver_SAI_(BOARD_MIC_INPUT_I2S_INSTANCE);
#endif

#if BOARD_WM8904_CODEC_PRESENT
extern ARM_DRIVER_WM8904  WM8904;
static ARM_DRIVER_WM8904 *wm8904 = &WM8904;
#endif

/**
  \fn          void dac_callback(uint32_t event)
  \brief       Callback routine from the i2s driver
  \param[in]   event: Event for which the callback has been called
*/
static void dac_callback(uint32_t event)
{
    if (event & ARM_SAI_EVENT_SEND_COMPLETE) {
        /* Send Success: Wake-up routine. */
        event_flag |= DAC_SEND_COMPLETE_EVENT;
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_i2s_dac_pins_config(void)
 * @brief   Configure I2S DAC pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_i2s_dac_pins_config(void)
{
    int32_t status;

    /* Configure DAC I2S SDO */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_SDO_GPIO_PORT),
                         BOARD_DAC_OUTPUT_SDO_GPIO_PIN,
                         BOARD_DAC_OUTPUT_SDO_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure DAC I2S WS */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_WS_GPIO_PORT),
                         BOARD_DAC_OUTPUT_WS_GPIO_PIN,
                         BOARD_DAC_OUTPUT_WS_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure DAC I2S SCLK */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_SCLK_GPIO_PORT),
                         BOARD_DAC_OUTPUT_SCLK_GPIO_PIN,
                         BOARD_DAC_OUTPUT_SCLK_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

#if BOARD_WM8904_CODEC_PRESENT
/**
  \fn          void board_wm8904_i2c_pins_config(void)
  \brief       Initialize the pinmux for I2C
  \return      status
*/
static int32_t board_wm8904_i2c_pins_config(void)
{
    int32_t status;

    /* I2C_SDA */
    status = pinconf_set(PORT_(BOARD_WM8904_CODEC_I2C_SDA_GPIO_PORT),
                         BOARD_WM8904_CODEC_I2C_SDA_GPIO_PIN,
                         BOARD_WM8904_CODEC_I2C_SDA_ALTERNATE_FUNCTION,
                         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
    if (status) {
        return status;
    }

    /* I2C_SCL */
    status = pinconf_set(PORT_(BOARD_WM8904_CODEC_I2C_SCL_GPIO_PORT),
                         BOARD_WM8904_CODEC_I2C_SCL_GPIO_PIN,
                         BOARD_WM8904_CODEC_I2C_SCL_ALTERNATE_FUNCTION,
                         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

/**
  \fn          void DAC_Init(void)
  \brief       DAC routine for master transmission
  \param[in]   None
*/
void DAC_Init(void)
{
    ARM_DRIVER_VERSION   version;
    ARM_SAI_CAPABILITIES cap;
    int32_t              status;
    uint32_t             buf_len = 0;
    uint32_t            *buf;

#if SOC_FEAT_CLK76P8M_CLK_ENABLE
    uint32_t error_code = SERVICES_REQ_SUCCESS;
    uint32_t service_error_code;

    /* Initialize the SE services */
    se_services_port_init();

    /* enable the HFOSCx2 clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_HFOSCx2,
                                              /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: clk enable = %" PRId32 "\n", error_code);
    }
#endif

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    status = board_pins_config();
    if (status != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return;
    }

#else
    /*
     * NOTE: The I2S DAC pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    status = board_i2s_dac_pins_config();
    if (status != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return;
    }

#if BOARD_WM8904_CODEC_PRESENT
    /* Configure the i2c pins to program WM8904 codec */
    status = board_wm8904_i2c_pins_config();
    if (status != 0) {
        printf("I2C pinmux failed\n");
        return;
    }
#endif
#endif

#if BOARD_WM8904_CODEC_PRESENT
    /* WM8904 codec init */
    status = wm8904->Initialize();
    if (status) {
        printf("WM8904 codec Init failed status = %" PRId32 "\n", status);
        goto error_codec_initialize;
    }

    status = wm8904->PowerControl(ARM_POWER_FULL);
    if (status) {
        printf("WM8904 codec Power up failed status = %" PRId32 "\n", status);
        goto error_codec_power;
    }
#endif

    /* Verify the I2S API version for compatibility */
    version = i2s_dac->GetVersion();
    printf("I2S API version = %" PRIu16 "\n", version.api);

    /* Verify if I2S protocol is supported */
    cap = i2s_dac->GetCapabilities();
    if (!cap.protocol_i2s) {
        printf("I2S is not supported\n");
        return;
    }

    /* Initializes I2S interface */
    status = i2s_dac->Initialize(dac_callback);
    if (status) {
        printf("DAC Init failed status = %" PRId32 "\n", status);
        goto error_dac_initialize;
    }

    /* Enable the power for I2S */
    status = i2s_dac->PowerControl(ARM_POWER_FULL);
    if (status) {
        printf("DAC Power Failed status = %" PRId32 "\n", status);
        goto error_dac_power;
    }

    /* configure I2S Transmitter to Asynchronous Master */
    status = i2s_dac->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                  ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(wlen),
                              wlen * 2,
                              sampling_rate);
    if (status) {
        printf("DAC Control status = %" PRId32 "\n", status);
        goto error_dac_control;
    }

    /* enable Transmitter */
    status = i2s_dac->Control(ARM_SAI_CONTROL_TX, 1, 0);
    if (status) {
        printf("DAC TX status = %" PRId32 "\n", status);
        goto error_dac_control;
    }

#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
    status = ADC_Init();
    if (status) {
        printf("ADC Init failed status = %" PRId32 "\n", status);
        goto error_adc_control;
    }

    /* enable Receiver */
    status = i2s_adc->Control(ARM_SAI_CONTROL_RX, 1, 0);
    if (status) {
        printf("ADC RX status = %" PRId32 "\n", status);
        goto error_adc_control;
    }
#endif

    do {
#ifdef DAC_PREDEFINED_SAMPLES
        /* Using predefined samples */
        buf_len = sizeof(hello_samples_24bit_48khz) / sizeof(hello_samples_24bit_48khz[0]);
        buf     = (uint32_t *) hello_samples_24bit_48khz;
#else
        buf_len = NUM_SAMPLES;
        buf     = (uint32_t *) sample_buf;

        /* Function to receive digital signal from mic */
        status  = Receiver();
        if (status) {
            printf("ADC Receive failed status = %" PRId32 "\n", status);
            goto error_adc_receive;
        }
#endif

        /* Transmit the samples */
        status = i2s_dac->Send(buf, buf_len);
        if (status) {
            printf("DAC Send status = %" PRId32 "\n", status);
            goto error_adc_dac;
        }

        /* Wait for the completion event */
        while (1) {
            if (event_flag & DAC_SEND_COMPLETE_EVENT) {
                event_flag &= ~DAC_SEND_COMPLETE_EVENT;
                break;
            }
        }
    } while (REPEAT_TX);

#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
    /* Stop the RX */
    status = i2s_adc->Control(ARM_SAI_CONTROL_RX, 0, 0);
    if (status) {
        printf("ADC RX status = %" PRId32 "\n", status);
        goto error_adc_control;
    }
#endif

    /* Stop the TX */
    status = i2s_dac->Control(ARM_SAI_CONTROL_TX, 0, 0);
    if (status) {
        printf("DAC TX status = %" PRId32 "\n", status);
        goto error_adc_dac;
    }

error_adc_dac:
#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
error_adc_control:
error_adc_receive:
    i2s_adc->PowerControl(ARM_POWER_OFF);
    i2s_adc->Uninitialize();
#endif
error_dac_control:
    i2s_dac->PowerControl(ARM_POWER_OFF);
error_dac_power:
    i2s_dac->Uninitialize();
error_dac_initialize:
#if BOARD_WM8904_CODEC_PRESENT
error_codec_power:
    wm8904->PowerControl(ARM_POWER_OFF);
    wm8904->Uninitialize();
error_codec_initialize:
#endif
#if SOC_FEAT_CLK76P8M_CLK_ENABLE
    /* disable the HFOSCx2 clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSCx2,
                                              false,
                                              &service_error_code);
    if (error_code) {
        printf("SE Error: HFOSCx2 clk disable = %" PRIu32 "\n", error_code);
    }
#endif
    WAIT_FOREVER_LOOP
}

#if !defined(DAC_PREDEFINED_SAMPLES) && defined(BOARD_MIC_INPUT_I2S_INSTANCE)
/**
  \fn          void adc_callback(uint32_t event)
  \brief       Callback routine from the i2s driver
  \param[in]   event Event for which the callback has been called
*/
static void adc_callback(uint32_t event)
{
    if (event & ARM_SAI_EVENT_RECEIVE_COMPLETE) {
        /* Receive Success: Wake-up routine. */
        event_flag |= ADC_RECEIVE_COMPLETE_EVENT;
    }

    if (event & ARM_SAI_EVENT_RX_OVERFLOW) {
        /* Receive Error: fifo overflow occurred. */
        event_flag |= ADC_RECEIVE_OVERFLOW_EVENT;
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_i2s_adc_pins_config(void)
 * @brief   Configure I2S ADC pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_i2s_adc_pins_config(void)
{
    int32_t status;

    /* Configure ADC I2S WS */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_WS_GPIO_PORT),
                         BOARD_MIC_INPUT_WS_GPIO_PIN,
                         BOARD_MIC_INPUT_WS_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure ADC I2S SCLK */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_SCLK_GPIO_PORT),
                         BOARD_MIC_INPUT_SCLK_GPIO_PIN,
                         BOARD_MIC_INPUT_SCLK_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure ADC I2S SDI */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_SDI_GPIO_PORT),
                         BOARD_MIC_INPUT_SDI_GPIO_PIN,
                         BOARD_MIC_INPUT_SDI_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

/**
  \fn          int32_t ADC_Init(void)
  \brief       ADC routine to initialize ADC
  \param[in]   None
  \return      status
*/
int32_t ADC_Init(void)
{
    ARM_DRIVER_VERSION   version;
    ARM_SAI_CAPABILITIES cap;
    int32_t              status;

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
    /*
     * NOTE: The I2S ADC pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    status = board_i2s_adc_pins_config();
    if (status != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return status;
    }
#endif

    /* Verify the I2S API version for compatibility*/
    version = i2s_adc->GetVersion();
    printf("I2S API version = %" PRIu16 "\n", version.api);

    /* Verify if I2S protocol is supported */
    cap = i2s_adc->GetCapabilities();
    if (!cap.protocol_i2s) {
        printf("I2S is not supported\n");
        return APP_ERROR;
    }

    /* Initializes I2S interface */
    status = i2s_adc->Initialize(adc_callback);
    if (status) {
        printf("ADC Init failed status = %" PRId32 "\n", status);
        return status;
    }

    /* Enable the power for I2S */
    status = i2s_adc->PowerControl(ARM_POWER_FULL);
    if (status) {
        printf("ADC Power failed status = %" PRId32 "\n", status);
        return status;
    }

    /* configure I2S Receiver to Asynchronous Master */
    status = i2s_adc->Control(ARM_SAI_CONFIGURE_RX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                  ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(wlen),
                              wlen * 2,
                              sampling_rate);
    if (status) {
        printf("ADC Control status = %" PRId32 "\n", status);
        return status;
    }

    return APP_SUCCESS;
}

/**
  \fn          int32_t Receiver(void)
  \brief       Function performing reception from mic
  \param[in]   None
  \return      status
*/
int32_t Receiver(void)
{
    int32_t status;

    /* Receive data */
    status = i2s_adc->Receive((uint32_t *) sample_buf, NUM_SAMPLES);
    if (status) {
        printf("ADC Receive status = %" PRId32 "\n", status);
        return status;
    }

    /* Wait for the completion event */
    while (1) {
        if (event_flag & ADC_RECEIVE_COMPLETE_EVENT) {
            event_flag &= ~ADC_RECEIVE_COMPLETE_EVENT;
            break;
        }

        if (event_flag & ADC_RECEIVE_OVERFLOW_EVENT) {
            event_flag &= ~ADC_RECEIVE_OVERFLOW_EVENT;
        }
    }

    return APP_SUCCESS;
}
#endif

/**
  \fn          int main(void)
  \brief       Application Main
  \return      int application exit status
*/
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

    DAC_Init();
    return 0;
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
