/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     : demo_lppdm.c
 * @author   : Nisarga A M
 * @email    : nisarga.am@alifsemi.com
 * @version  : V1.0.0
 * @date     : 15-Jan-2023
 * @brief    : Baremetal code for LPPDM(Low power Pulse Density Modulation).
 *             -> Select the PDM channel
 *             -> Select the mode of operation in the Control API.
 *             -> Select the mode of operation in the Control API.
 *             -> Then build the project and flash the generated axf file on the target
 *             -> Then Start playing some audio or speak near to the PDM microphone which
 *                is on Flat board.
 *             -> Once the sample count reaches the maximum value,the PCM samples will be
 *                stored in the particular buffer.
 *             -> Export the memory and To play the PCM data, use pcmplay.c file which
 *                will generate the pcm_samples.pcm audio file
 *             -> Use ffplay command to play the audio.
 *             E7: Hardware setup:
 *             -> Connect Flat board PDM Microphone PDM data line to LPPDM data
 *                line of P3_5 (J11 on Flat board)
 *              For channel 0 and channel 1
 *             -> Clock line:
                  pin P6_7 (on Flat board J15) --> pin P3_4 (on Flat board J11)
               -> Data line:
                  pin P5_4 (on Flat board J14) --> pin P3_5 (on Flat board J11)
 *             E1C: Hardware setup:
 *             -> Connect the Spark DevKit PDM Microphone's PDM data line to LPPDM data
 *                line (P5_6)
 *             -> For channel 0 and channel 1 as follows:
 *             -> Clock Line Connection:
 *                pin P7_4 --> pin P5_4
 *             -> Data Line Connection:
 *                pin P7_5 --> pin P5_6
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

/* Project Includes */
/* include for PDM Driver */
#include "Driver_PDM.h"
#include "pinconf.h"
#include "board_config.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#if !defined(RTSS_HE)
#error "This Demo application works only on RTSS_HE"
#endif

#include "app_utils.h"

// Set to 0: Use application-defined LPPDM pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* Store the number of samples */
/* For 40000 samples user can hear maximum up to 4 sec of audio
 * to store maximum samples then change the scatter file and increase the memory */
#define NUM_SAMPLE                     40000

/* Channel Configuration:
 * The CHANNEL_0 and CHANNEL_1 macros define the channel numbers used
 * for channel configuration.
 * To configure other channels (e.g., channel 2 and channel 3), update the macros as follows:
 * #define CHANNEL_0  2
 * #define CHANNEL_1  3
 */
#define CHANNEL_0                      0
#define CHANNEL_1                      1

/* PDM Channel 0 configurations */
#define CH0_PHASE                      0x00000003
#define CH0_GAIN                       0x00000013
#define CH0_PEAK_DETECT_TH             0x00060002
#define CH0_PEAK_DETECT_ITV            0x00020027

/* PDM Channel 1 configurations */
#define CH1_PHASE                      0x0000001F
#define CH1_GAIN                       0x0000000D
#define CH1_PEAK_DETECT_TH             0x00060002
#define CH1_PEAK_DETECT_ITV            0x0004002D

/* PDM driver instance */
extern ARM_DRIVER_PDM  Driver_LPPDM;
static ARM_DRIVER_PDM *PDMdrv = &Driver_LPPDM;

PDM_CH_CONFIG pdm_coef_reg;

/* For Demo purpose use channel 0  and channel 1 */
/* To store the PCM samples for Channel 0 and channel 1 */
uint16_t sample_buf[NUM_SAMPLE];

/* Channel 0 FIR coefficient */
uint32_t ch0_fir[18] = {0x00000000,
                        0x000007FF,
                        0x00000000,
                        0x00000004,
                        0x00000004,
                        0x000007FC,
                        0x00000000,
                        0x000007FB,
                        0x000007E4,
                        0x00000000,
                        0x0000002B,
                        0x00000009,
                        0x00000016,
                        0x00000049,
                        0x00000793,
                        0x000006F8,
                        0x00000045,
                        0x00000178};

/* Channel 1 FIR coefficient */
uint32_t ch1_fir[18] = {0x00000001,
                        0x00000003,
                        0x00000003,
                        0x000007F4,
                        0x00000004,
                        0x000007ED,
                        0x000007F5,
                        0x000007F4,
                        0x000007D3,
                        0x000007FE,
                        0x000007BC,
                        0x000007E5,
                        0x000007D9,
                        0x00000793,
                        0x00000029,
                        0x0000072C,
                        0x00000072,
                        0x000002FD};

/* PDM callback events */
typedef enum {
    PDM_CALLBACK_ERROR_EVENT           = (1 << 0),
    PDM_CALLBACK_WARNING_EVENT         = (1 << 1),
    PDM_CALLBACK_AUDIO_DETECTION_EVENT = (1 << 2)
} PDM_CB_EVENTS;

volatile int32_t call_back_event = 0;

void pdm_demo();

static void PDM_fifo_callback(uint32_t event)
{
    if (event & ARM_PDM_EVENT_ERROR) {
        call_back_event = PDM_CALLBACK_ERROR_EVENT;
    }

    if (event & ARM_PDM_EVENT_CAPTURE_COMPLETE) {
        call_back_event = PDM_CALLBACK_WARNING_EVENT;
    }

    if (event & ARM_PDM_EVENT_AUDIO_DETECTION) {
        call_back_event = PDM_CALLBACK_AUDIO_DETECTION_EVENT;
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_lppdm_pins_config(void)
 * @brief   Configure LPPDM pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_lppdm_pins_config(void)
{
    int32_t status;

    /* channel 0_1 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D0_GPIO_PORT),
                         BOARD_LPPDM_D0_GPIO_PIN,
                         BOARD_LPPDM_D0_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 2_3 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D1_GPIO_PORT),
                         BOARD_LPPDM_D1_GPIO_PIN,
                         BOARD_LPPDM_D1_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 4_5 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D2_GPIO_PORT),
                         BOARD_LPPDM_D2_GPIO_PIN,
                         BOARD_LPPDM_D2_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 6_7 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D3_GPIO_PORT),
                         BOARD_LPPDM_D3_GPIO_PIN,
                         BOARD_LPPDM_D3_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* Channel 0_1 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C0_GPIO_PORT),
                         BOARD_LPPDM_C0_GPIO_PIN,
                         BOARD_LPPDM_C0_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 2_3 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C1_GPIO_PORT),
                         BOARD_LPPDM_C1_GPIO_PIN,
                         BOARD_LPPDM_C1_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 4_5 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C2_GPIO_PORT),
                         BOARD_LPPDM_C2_GPIO_PIN,
                         BOARD_LPPDM_C2_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 6_7 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C3_GPIO_PORT),
                         BOARD_LPPDM_C3_GPIO_PIN,
                         BOARD_LPPDM_C3_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

/**
 * @fn         : void PDM_fifo_callback(uint32_t event)
 * @brief      : PDM fifo callback
 *               -> Initialize the LPPDM module.
 *               -> Enable the Power for the LPPDM module
 *               -> Select the mode of operation in Control API.The
 *                    mode which user has selected will be applies to
 *                    all the channel which user has selected.
 *               -> Select the Bypass DC blocking IIR filter for reference.
 *               -> Select the LPPDM channel and use the selected channel
 *                    configuration and status register values.
 *               -> Play some audio and start capturing the data.
 *               -> Once all data has stored in the particular buffer ,
 *                  call back event will be set and it will stop capturing
 *                  data.
 *               -> Once all the data capture is done , go to the particular
 *                  memory location which user has given for storing PCM data
 *                  samples.
 *               -> Then export the memory and give the total size of
 *                  the buffer memory and select the particular bin file and export
 *                  the memory.
 *               -> Play the PCM sample file using ffplay command.
 * @return     : none
 */
void pdm_demo()
{
    int32_t            ret = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> PDM demo starting up!!! <<< \r\n");

    version = PDMdrv->GetVersion();
    printf("\r\n PDM version api:%" PRIx16 " driver:%" PRIx16 "...\r\n", version.api, version.drv);

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The LPPDM pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_lppdm_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#endif

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

    /* Initialize PDM driver */
    ret = PDMdrv->Initialize(PDM_fifo_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM init failed\n");
        return;
    }

    /* Enable the power for PDM */
    ret = PDMdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Power up failed\n");
        goto error_uninitialize;
    }

    /* PDM Channel Selection:
     * This code selects PDM channel 0 and channel 1 for operation.
     * To select different channels (e.g., channel 2 and channel 3), update the macro parameter
     * in the PDMdrv->Control function as follows:
     * (ARM_PDM_MASK_CHANNEL_2 | ARM_PDM_MASK_CHANNEL_3)
     * Note: These macros are defined in Driver_PDM.h.
     */
    ret = PDMdrv->Control(ARM_PDM_SELECT_CHANNEL,
                          (ARM_PDM_MASK_CHANNEL_0 | ARM_PDM_MASK_CHANNEL_1),
                          0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM channel select control failed\n");
        goto error_poweroff;
    }

    /* Select Standard voice PDM mode */
    ret = PDMdrv->Control(ARM_PDM_MODE, ARM_PDM_MODE_AUDIOFREQ_8K_DECM_64, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Standard voice control mode failed\n");
        goto error_poweroff;
    }

    /* Select the DC blocking IIR filter */
    ret = PDMdrv->Control(ARM_PDM_BYPASS_IIR_FILTER, ENABLE, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM DC blocking IIR control failed\n");
        goto error_poweroff;
    }

    /* Set Channel 0 Phase value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PHASE, CHANNEL_0, CH0_PHASE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 0 Gain value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_GAIN, CHANNEL_0, CH0_GAIN);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 0 Peak detect threshold value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_TH, CHANNEL_0, CH0_PEAK_DETECT_TH);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 0 Peak detect ITV value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_ITV, CHANNEL_0, CH0_PEAK_DETECT_ITV);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    pdm_coef_reg.ch_num = CHANNEL_0;
    memcpy(pdm_coef_reg.ch_fir_coef,
           ch0_fir,
           sizeof(pdm_coef_reg.ch_fir_coef)); /* Channel 0 fir coefficient */
    pdm_coef_reg.ch_iir_coef = 0x00000004;    /* Channel IIR Filter Coefficient */

    ret                      = PDMdrv->Config(&pdm_coef_reg);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Phase value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PHASE, CHANNEL_1, CH1_PHASE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Gain value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_GAIN, CHANNEL_1, CH1_GAIN);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Peak detect threshold value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_TH, CHANNEL_1, CH1_PEAK_DETECT_TH);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Peak detect ITV value */
    ret = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_ITV, CHANNEL_1, CH1_PEAK_DETECT_ITV);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Channel 1 configuration values */
    pdm_coef_reg.ch_num = CHANNEL_1; /* Channel 1 */
    memcpy(pdm_coef_reg.ch_fir_coef,
           ch1_fir,
           sizeof(pdm_coef_reg.ch_fir_coef)); /* Channel 1 fir coefficient*/
    pdm_coef_reg.ch_iir_coef = 0x00000004;    /* Channel IIR Filter Coefficient */

    ret                      = PDMdrv->Config(&pdm_coef_reg);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    printf("\n------> Start Speaking or Play some Audio!------> \n");

    /* Receive the audio samples */
    ret = PDMdrv->Receive((uint16_t *) sample_buf, NUM_SAMPLE);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Receive failed\n");
        goto error_capture;
    }

    /* wait for the call back event */
    while (call_back_event == 0) {
    }

    /* PDM fifo overflow error event */
    if (call_back_event == PDM_CALLBACK_ERROR_EVENT) {
        printf("\n PDM error event: Fifo overflow \n");
    }

    /* PDM fifo alomost full warning event */
    if (call_back_event == PDM_CALLBACK_WARNING_EVENT) {
        printf("\n PDM warning event : Fifo almost full\n");
    }

    /* PDM channel audio detection event */
    if (call_back_event == PDM_CALLBACK_AUDIO_DETECTION_EVENT) {
        printf("\n PDM audio detect event: data in the audio channel");
    }

    call_back_event = 0;

    printf("\n------> Stop recording ------> \n");
    printf("\n--> PCM samples will be stored in 0x%" PRIxPTR ""
           "address and size of buffer is %" PRIu16 "\n",
           (uintptr_t) sample_buf,
           sizeof(sample_buf));
    printf("\n ---END--- \r\n <<< wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_capture:
error_poweroff:
    ret = PDMdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\n Error: PDM power off failed\n");
    }

error_uninitialize:
    ret = PDMdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\n Error: PDM Uninitialize failed\n");
    }

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

    printf("\r\n XXX PDM demo exiting XXX...\r\n");
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
    pdm_demo();

    return 0;
}
