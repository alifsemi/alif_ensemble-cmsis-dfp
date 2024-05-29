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
 * @file     WiFi_we310f5_HW.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    WiFi hardware functions
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include <Driver_GPIO.h>
#include "we310f5_SPI.h"
#include "WiFi_we310f5_HW_Config.h"
#include "WiFi_utils.h"
#include "system_utils.h"


/* Wi-Fi Enable */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(WIFI_ENABLE_GPIO_PORT_NUM);                     /* GPIO13 Driver            */
static ARM_DRIVER_GPIO *pWifiEnable  = &ARM_Driver_GPIO_(WIFI_ENABLE_GPIO_PORT_NUM);    /* GPIO13 Driver instance   */

/* Wi-Fi Wake-up */
//extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(WIFI_WAKEUP_GPIO_PORT_NUM);                     /* GPIO13 Driver            */
//static ARM_DRIVER_GPIO *pWifiWakeup  = &ARM_Driver_GPIO_(WIFI_WAKEUP_GPIO_PORT_NUM);    /* GPIO13 Driver instance   */

/* Wi-Fi RESETp */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(WIFI_RESET_GPIO_PORT_NUM);                      /* GPIO13 Driver            */
static ARM_DRIVER_GPIO *pWifiReset  = &ARM_Driver_GPIO_(WIFI_RESET_GPIO_PORT_NUM);      /* GPIO13 Driver instance   */

/* Wi-Fi Interrupt */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(WIFI_INTR_GPIO_PORT_NUM);                       /* GPIO13 Driver            */
static ARM_DRIVER_GPIO *pWifiIntr  = &ARM_Driver_GPIO_(WIFI_INTR_GPIO_PORT_NUM);        /* GPIO13 Driver instance   */

/**
  \fn            int32_t we310f5_device_status()
  \brief         Get device status for read/write operation
  \return        Read or Write status
*/
int32_t we310f5_device_status(void)
{
    volatile uint32_t interrupt_bit = 0;

    pWifiIntr->GetValue(WIFI_INTR_GPIO_PIN_NUM, (uint32_t *)&interrupt_bit);

    return interrupt_bit;
}

/**
  \fn            void wifi_irq_callback(uint32_t event)
  \brief         Callback when Wi-Fi interrupt occurs
  \param[in]     event
  \return        none
*/
static void we310f5_irq_callback(uint32_t event)
{
    ARG_UNUSED(event);
    we310f5_device_status();

//    if(wifi_res->callBack)
//        wifi_res->callBack(event, NULL);
}

/**
  \fn            void we310f5_int_ctrl(bool enable)
  \brief         Control Wi-Fi interrupt
  \param[in]     enable     Disable or Enable flag
  \return        none
*/
static void we310f5_int_ctrl(bool enable)
{
    uint32_t arg = ARM_GPIO_IRQ_POLARITY_HIGH
            | ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE
            | ARM_GPIO_IRQ_SENSITIVE_EDGE;

    enable ? pWifiIntr->Control(WIFI_INTR_GPIO_PIN_NUM, ARM_GPIO_ENABLE_INTERRUPT, &arg): \
             pWifiIntr->Control(WIFI_INTR_GPIO_PIN_NUM, ARM_GPIO_DISABLE_INTERRUPT, NULL);
}

/**
  \fn            int32_t we310f5_hw_init()
  \brief         It will send AT command to Telit chip and wait for response.
                        process the received response.
  \return        execution status
                   - \ref WIFI_SUCCESS                : Operation successful
                   - \ref WIFI_FAILED                 : Operation failed
*/
int32_t we310f5_hw_init()
{
    int32_t     ret = ARM_DRIVER_ERROR;

    // SPI Initialization
    ret = we310f5_spi_init();
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    // WIFI_ENABLE: P13_0
    //--------------------------------------------------------------------------
    ret = pWifiEnable->Initialize(  WIFI_ENABLE_GPIO_PIN_NUM, NULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    ret = pWifiEnable->PowerControl(WIFI_ENABLE_GPIO_PIN_NUM, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    // Turn OFF
    ret = pWifiEnable->SetDirection(WIFI_ENABLE_GPIO_PIN_NUM, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    // Turn ON
    ret = pWifiEnable->SetValue(    WIFI_ENABLE_GPIO_PIN_NUM, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
    //--------------------------------------------------------------------------

    // WIFI_RESET:  P13_2
    //--------------------------------------------------------------------------
    ret = pWifiReset->Initialize(  WIFI_RESET_GPIO_PIN_NUM, NULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
    ret = pWifiReset->PowerControl(WIFI_RESET_GPIO_PIN_NUM, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    ret = pWifiReset->SetDirection(WIFI_RESET_GPIO_PIN_NUM, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    // ON* = Active Low Reset: Hold in Reset
    ret = pWifiReset->SetValue(    WIFI_RESET_GPIO_PIN_NUM, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    // ON* = Active Low Reset (Release from Reset)
    ret = pWifiReset->SetValue(    WIFI_RESET_GPIO_PIN_NUM, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
    //--------------------------------------------------------------------------

    // WIFI_IRQ:    P13_3
    //--------------------------------------------------------------------------
    ret = pWifiIntr->Initialize(  WIFI_INTR_GPIO_PIN_NUM, we310f5_irq_callback);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    ret = pWifiIntr->PowerControl(WIFI_INTR_GPIO_PIN_NUM, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    ret = pWifiIntr->SetDirection(WIFI_INTR_GPIO_PIN_NUM, GPIO_PIN_DIRECTION_INPUT);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
    //--------------------------------------------------------------------------

    // Configure IRQ pin for Positive Edge Trigger.
    we310f5_int_ctrl(true);

    return ARM_DRIVER_OK;
}
