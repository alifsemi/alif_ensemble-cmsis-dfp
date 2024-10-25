/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     CMP_baremetal.c
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     26-July-2024
 * @brief    Baremetal code for analog Comparator.
 *              - CMP0 instance is used - in RTE_Device.h we set the input muxes
 *              - Input A is set by RTE_CMP0_SEL_POSITIVE
 *              - Input A is set to RTE_CMP0_POSITIVE_PIN_PO_00 (analog pin P0_0)
 *              - Input B is set by RTE_CMP0_SEL_NEGATIVE
 *              - Input B is set to RTE_CMP_NEGATIVE_DAC6(which provide 0.9v)
 *              Hardware setup (1 wires needed):
 *              - Connect P0_0(+ve pin) to P4_7(GPIO output) and DAC6 is set as negative
 *                pin,check CMP0 output in the pin P8_0 using saleae logic analyzer.
 *              - If +ve input is greater than -ve input, interrupt will be generated,
 *                and the output will be high.
 *              - If -ve input is greater than +ve input, interrupt will be generated,
 *                and the output will be low.
 ******************************************************************************/

/* System Includes */
#include "Driver_GPIO.h"
#include <stdio.h>
#include "system_utils.h"
#include "pinconf.h"
#include "Driver_UTIMER.h"

/* include for Comparator Driver */
#include "Driver_CMP.h"
#include "RTE_Components.h"

#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* LED configurations */
#define GPIO4_PORT                      4      /* Use LED0_R          */
#define LED0_R                          PIN_7  /* LED0_R gpio pin     */

/* To read the HSCMP0 output status set CMP_OUTPIN as 0 and for HSCMP1 set
 * CMP_OUTPIN as 1 */
#define CMP8_PORT                       8
#define CMP_OUTPIN                      0

#define NUM_TAPS                        3  /* Number of filter taps                 */
#define SAMPLING_RATE                   8  /* Set the prescaler values from 0 to 31 */
#define LPCMP                0
#define HSCMP                1

/* To configure for HSCMP, use CMP_INSTANCE HSCMP */
/* To configure for LPCMP, use CMP_INSTANCE LPCMP */
#define CMP_INSTANCE         HSCMP

#define ERROR    -1
#define SUCCESS   0

extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO4_PORT);
ARM_DRIVER_GPIO *ledDrv = &ARM_Driver_GPIO_(GPIO4_PORT);

extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(CMP8_PORT);
ARM_DRIVER_GPIO *CMPout = &ARM_Driver_GPIO_(CMP8_PORT);

#if(CMP_INSTANCE == LPCMP)
extern ARM_DRIVER_CMP Driver_LPCMP;
static ARM_DRIVER_CMP *CMPdrv = &Driver_LPCMP;
#else
extern ARM_DRIVER_CMP Driver_CMP0;
static ARM_DRIVER_CMP *CMPdrv = &Driver_CMP0;
#endif

#define CMP_CALLBACK_EVENT_SUCCESS  1

volatile int32_t call_back_event = 0;
volatile uint32_t call_back_counter = 0;
uint32_t value =0;

/**
 * @fn          void cmp_pinmux_config(void)
 * @brief       Initialize the pinmux for CMP output
 * @return      status
*/
static int32_t cmp_pinmux_config(void)
{
    int32_t status;

    /* Configure HSCMP0 output */
    status = pinconf_set(PORT_8, PIN_0, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* Configure HSCMP1 output */
    status = pinconf_set(PORT_8, PIN_1, PINMUX_ALTERNATE_FUNCTION_7, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* LPCMP_IN0 input to the positive terminal of LPCMP */
    status = pinconf_set(PORT_2, PIN_4, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* CMP0_IN0 input to the positive terminal of HSCMP0 */
    status = pinconf_set(PORT_0, PIN_0, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* CMP1_IN0 input to the positive terminal of HSCMP1 */
    status = pinconf_set(PORT_0, PIN_1, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* VREF_IN0 input to the negative terminal of HSCMP0 and HSCMP1 */
    status = pinconf_set(PORT_2, PIN_0, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    return SUCCESS;
}

/**
 * @fn        led_init(void)
 * @brief     - Initialize the LED0_R
 *            - Enable the power for LED0_R
 *            - Set direction for LED0_R
 *            - Set value for LED0_R
 * @param[in]  None
 * return      status
 */
static int32_t led_init(void)
{
    int32_t ret1 = 0;

    /* gpio12 pin3 can be used as Red LED of LED0 */
    pinconf_set(GPIO4_PORT, LED0_R, PINMUX_ALTERNATE_FUNCTION_0, 0);

    /* Initialize the LED0_R */
    ret1 = ledDrv->Initialize(LED0_R, NULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return ERROR;
    }

    ret1 = CMPout->Initialize(CMP_OUTPIN, NULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return ERROR;
    }

    /* Enable the power for LED0_R */
    ret1 = ledDrv->PowerControl(LED0_R, ARM_POWER_FULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize_LED;
    }

    /* Enable the power for LED0_R */
    ret1 = CMPout->PowerControl(CMP_OUTPIN, ARM_POWER_FULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize_LED;
    }

   ret1 = ledDrv->SetDirection(LED0_R, GPIO_PIN_DIRECTION_OUTPUT);
   if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off_LED;
    }

   ret1 = CMPout->SetDirection(CMP_OUTPIN, GPIO_PIN_DIRECTION_OUTPUT);
   if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off_LED;
    }

    ret1 = ledDrv->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_HIGH);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off_LED;
    }

    return SUCCESS;

error_power_off_LED:
    /* Power-off the LED0_R */
    ret1 = ledDrv->PowerControl(LED0_R, ARM_POWER_OFF);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("ERROR: Failed to power off \n");
    }

error_uninitialize_LED:
    /* Uninitialize the LED0_R */
    ret1 = ledDrv->Uninitialize(LED0_R);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("Failed to Un-initialize \n");
    }
    return ERROR;
}

/**
 * @fn        CMP_get_status(void)
 * @brief     - Get the Status of CMP output pin.
 * @param[in]  None
 * return      status
 */
static int32_t CMP_get_status(void)
{
    int32_t ret = 0;
    ret = CMPout->GetValue(CMP_OUTPIN, &value);
    if(ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to toggle LEDs\n");
        goto error_power_off_LED;
    }
    return value;

    error_power_off_LED:
    /* Power-off the CMP_OUTPIN */
    ret = CMPout->PowerControl(CMP_OUTPIN, ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)  {
        printf("ERROR: Failed to power off \n");
    }

    /* Uninitialize the CMP_OUTPIN */
    ret = CMPout->Uninitialize(CMP_OUTPIN);
    if(ret != ARM_DRIVER_OK)  {
        printf("Failed to Un-initialize \n");
    }
    return ERROR;
}
/**
 * @fn         led_toggle(void)
 * @brief      - set LED0_R for toggle
 * @param[in]  None
 * return      status
 */
static int32_t led_toggle(void)
{
    int32_t ret1 = 0;

    ret1 = ledDrv->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_TOGGLE);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to toggle LEDs\n");
        goto error_power_off_LED;
    }
    return SUCCESS;

error_power_off_LED:
    /* Power-off the LED0_R */
    ret1 = ledDrv->PowerControl(LED0_R, ARM_POWER_OFF);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("ERROR: Failed to power off \n");
    }

    /* Uninitialize the LED0_R */
    ret1 = ledDrv->Uninitialize(LED0_R);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("Failed to Un-initialize \n");
    }
    return ERROR;
}


/**
 * @fn       void CMP_filter_callback(uint32_t event)
 * @brief    - This code expects the LED Blinky application to be running
 *             and the pin P12_3 should toggle every 1 second.
 *           - The comparator compares the voltage of P12_3 which is connected
 *             to positive comparator input which is compared to the 0.9v DAC6.
 *           - When the comparator input changes from HIGH to LOW or from LOW to HIGH,
 *             interrupt will be generated and it will set the call back event
 *           - According to the interrupt generation the call_back_counter will be incremented.
 * @return   None
 */
static void CMP_filter_callback(uint32_t event)
{
    if(event & ARM_CMP_FILTER_EVENT_OCCURRED)
    {
        /* Received Comparator filter event */
        call_back_event = CMP_CALLBACK_EVENT_SUCCESS;
    }
    call_back_counter++;
}

static void CMP_demo_entry()
{
    int32_t ret = 0;
    uint32_t loop_count = 10;
    ARM_DRIVER_VERSION version;
    ARM_COMPARATOR_CAPABILITIES capabilities;
    int8_t status = 0;

    printf("\r\n >>> Comparator demo starting up!!! <<< \r\n");

    /* Configure the CMP output pins */
    if(cmp_pinmux_config())
    {
        printf("CMP pinmux failed\n");
    }

    /* Initialize the configurations for LED0_R */
    if(led_init())
    {
        printf("Error: LED initialization failed\n");
        return;
    }

    version = CMPdrv->GetVersion();
    printf("\r\n Comparator version api:%X driver:%X...\r\n", version.api, version.drv);

    /* Initialize the Comparator driver */
    ret = CMPdrv->Initialize(CMP_filter_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator init failed\n");
        return;
    }

    /* Enable the power for Comparator */
    ret = CMPdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Power up failed\n");
        goto error_uninitialize;
    }

#if(CMP_INSTANCE == HSCMP)
    /* Prescaler function for the comparator */
    ret = CMPdrv->Control(ARM_CMP_PRESCALER_CONTROL, SAMPLING_RATE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Prescaler control failed\n");
        goto error_poweroff;
    }

    /* Filter function for analog comparator*/
    ret = CMPdrv->Control(ARM_CMP_FILTER_CONTROL, NUM_TAPS);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Filter control failed\n");
        goto error_poweroff;
    }
#endif

    /* Start the Comparator module */
    ret = CMPdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Start failed\n");
        goto error_poweroff;
    }

    while(loop_count --)
    {
#if(CMP_INSTANCE == HSCMP)
        /* Toggle the LED0_R */
        if(led_toggle())
        {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_poweroff;
        }
#endif

        /* wait for the call back event */
        while(call_back_event == 0);
        call_back_event = 0;

#if(CMP_INSTANCE == HSCMP)
        /* Introducing a delay to stabilize input voltage for comparator measurement*/
        sys_busy_loop_us(100000);

        /* Check the status of the CMP output pin */
        status = CMP_get_status();

        /* If user give +ve input voltage more than -ve input voltage, status will be set to 1*/
        if(status == 1)
        {
            printf("\n CMP positive input voltage is greater than negative input voltage\n");
        }
        /* If user give -ve input voltage more than +ve input voltage, status will be set to 0*/
        else if(status == 0)
        {
            printf("\n CMP negative input voltage is greater than the positive input voltage\n");
        }
        else
        {
            printf("ERROR: Status detection is failed\n");
            goto error_poweroff;
        }
#endif
    }

    /* Stop the Comparator module */
    ret = CMPdrv->Stop();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Stop failed\n");
        goto error_poweroff;
    }

    printf("\n Comparator Filter event completed and the call_back_counter value is %d\n",call_back_counter );

error_poweroff:
    /* Power off Comparator peripheral */
    ret = CMPdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Power OFF failed.\r\n");
    }

error_uninitialize:
    /* UnInitialize comparator driver */
    ret = CMPdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: comparator Un-initialize failed.\r\n");
    }

}

/* Define main entry point */
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

    /* Enter the demo Application */
    CMP_demo_entry();

    return 0;
}
