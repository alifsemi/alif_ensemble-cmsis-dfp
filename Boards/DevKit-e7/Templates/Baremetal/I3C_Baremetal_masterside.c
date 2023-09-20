/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/**************************************************************************//**
 * @file     I3C_Baremetal_masterside.c
 * @author   Prabhakar kumar
 * @email    prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     27-May-2023
 * @brief    Baremetal to verify master and slave loop back test
 *
 *           Master sends n-bytes of data to Slave.
 *            Slave sends back received data to Master.
 *            then Master will compare send and received data
 *            (Master will continue sending data in loop,
 *             Master will stop if send and received data does not match).
 *
 *           I3C master configuration
 *            Select appropriate i3c Speed mode as per i2c or i3c slave device.
 *             I3C_BUS_MODE_PURE                             : Only Pure I3C devices
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 Mbps
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 Kbps
 *             I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 Kbps
 *
 *           Hardware Setup:
 *            Required two boards one for Master and one for Slave
 *             (as there is only one i3c instance is available on ASIC).
 *
 *           Connect SDA to SDA and SCL to SCL and GND to GND.
 *            - SDA P7_6 -> SDA P7_6
 *            - SCL P7_7 -> SCL P7_7
 *            - GND      -> GND
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


/* System Includes */
#include <stdio.h>
#include "string.h"

/* Project Includes */
/* I3C Driver */
#include "Driver_I3C.h"
#include "system_utils.h"

/* PINMUX Driver */
#include "pinconf.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* i3c Driver instance 0 */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

/* I3C slave target address */
#define I3C_SLV_TAR           (0x48)

/* transmit buffer from i3c */
uint8_t tx_data[4] = {0x00, 0x01, 0x02, 0x03};

/* receive buffer from i3c */
uint8_t rx_data[4] = {0x00};

uint32_t tx_cnt = 0;
uint32_t rx_cnt = 0;

/* flag for callback event. */
volatile uint32_t cb_event;

void i3c_master_loopback_demo();

/* i3c callback events */
typedef enum _I3C_CB_EVENT{
    I3C_CB_EVENT_SUCCESS        = (1 << 0),
    I3C_CB_EVENT_ERROR          = (1 << 1)
}I3C_CB_EVENT;

/**
  \fn          int32_t hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      0:success; -1:failure
*/
int32_t hardware_init(void)
{
    /* for I3C_D instance,
     *  for I3C in I3C mode (not required for I3C in I2C mode)
     *  GPIO voltage level(flex) has to be change to 1.8-V power supply.
     *
     *  GPIO_CTRL Register field VOLT:
     *   Select voltage level for the 1.8-V/3.3-V (flex) I/O pins
     *    0x0: I/O pin will be used with a 3.3-V power supply
     *    0x1: I/O pin will be used with a 1.8-V power supply
     */
#define GPIO_CTRL_BASE   (0x1A609000)

    uint32_t *p_gpio_flex = (uint32_t *)(GPIO_CTRL_BASE);
    *p_gpio_flex = 1;

    /* I3C_SDA_D */
    pinconf_set(PORT_7, PIN_6, PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
                PADCTRL_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS);

    /* I3C_SCL_D */
    pinconf_set(PORT_7, PIN_7, PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
                PADCTRL_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS);

    return ARM_DRIVER_OK;
}

/**
  \fn          void I3C_callback(UINT event)
  \brief       i3c isr callback
  \param[in]   event: i3c Event
  \return      none
*/
static void I3C_callback(uint32_t event)
{
    if (event & ARM_I3C_EVENT_TRANSFER_DONE)
    {
        cb_event = I3C_CB_EVENT_SUCCESS;
    }
    if (event & ARM_I3C_EVENT_TRANSFER_ERROR)
    {
        cb_event = I3C_CB_EVENT_ERROR;
    }
}

/**
  \fn          void i3c_master_loopback_demo(void)
  \brief       TestApp to verify i3c master mode loopback
               This demo does:
                 - initialize i3c driver;
                 - 1 byte of data transmitted from master and slave receive 1 byte and
                   same 1 byte of data received by slave is transmitted through slave
                   transmit and master receive 1byte.
  \return      none
*/
void i3c_master_loopback_demo(void)
{
    uint32_t   ret    = 0;
    uint32_t   len    = 0;
    int32_t    cmp    = 0;
    uint8_t    slave_addr = 0x00;

    ARM_DRIVER_VERSION version;

    printf("\r\n \t\t >>> Master loop back demo starting up!!! <<< \r\n");

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();
    printf("\r\n i3c version api:0x%X driver:0x%X \r\n",  \
                           version.api, version.drv);

    /* Initialize i3c hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != 0)
    {
        printf("\r\n Error: i3c hardware_init failed.\r\n");
        return;
    }

    /* Initialize I3C driver */
    ret = I3Cdrv->Initialize(I3C_callback);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Initialize failed.\r\n");
        return;
    }

    /* Power up I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Power Up failed.\r\n");
        goto error_poweroff;
    }

    /* i3c Speed Mode Configuration:
     *  I3C_BUS_MODE_PURE                             : Only Pure I3C devices
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 Mbps
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 Kbps
     *  I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 Kbps
     */
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,  \
                          I3C_BUS_MODE_PURE);

    sys_busy_loop_us(1000);

    /* Assign Dynamic Address to i3c slave */
    printf("\r\n >> i3c: Get dynamic addr for static addr:0x%X.\r\n",I3C_SLV_TAR);

    /* clear callback event flag. */
    cb_event = 0;

    ret = I3Cdrv->MasterAssignDA(&slave_addr, I3C_SLV_TAR);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
        goto error_poweroff;
    }
    printf("\r\n >> i3c: Received dyn_addr:0x%X for static addr:0x%X. \r\n",   \
                                 slave_addr,I3C_SLV_TAR);

    /* wait for callback event. */
    while(!((cb_event == I3C_CB_EVENT_SUCCESS) || (cb_event == I3C_CB_EVENT_ERROR)));

    /* Delay */
    sys_busy_loop_us(1000);

    /* clear callback event flag. */
    cb_event = 0;

    /* Observation:
     *  Master needs to send "MasterAssignDA" two times,
     *  First time slave is not giving ACK.
     */

    /* Assign Dynamic Address to i3c slave */
    ret = I3Cdrv->MasterAssignDA(&slave_addr, I3C_SLV_TAR);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
        goto error_poweroff;
    }

    /* wait for callback event. */
    while(!((cb_event == I3C_CB_EVENT_SUCCESS) || (cb_event == I3C_CB_EVENT_ERROR)));

    if(cb_event == I3C_CB_EVENT_ERROR)
    {
        printf("\nError: I3C MasterAssignDA failed\n");
        while(1);
    }

    sys_busy_loop_us(1000);

    while(1)
    {
        len = 4;

        /* Delay */
        sys_busy_loop_us(100);

        /* fill any random TX data. */
        tx_data[0] += 1;
        tx_data[1] += 1;
        tx_data[2] += 1;
        tx_data[3] += 1;

        /* clear callback event flag. */
        cb_event = 0;

        /* Master transmit */
        ret = I3Cdrv->MasterTransmit(slave_addr, tx_data, len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Master Transmit failed. \r\n");
            goto error_poweroff;
        }

        /* wait for callback event. */
        while(!((cb_event == I3C_CB_EVENT_SUCCESS) || (cb_event == I3C_CB_EVENT_ERROR)));

        if(cb_event == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C Master transmit Failed\n");
            while(1);
        }

        tx_cnt += 1;

        /* Delay */
        sys_busy_loop_us(1000);

        /* clear rx_data buffer */
        rx_data[0] = 0x00;
        rx_data[1] = 0x00;
        rx_data[2] = 0x00;
        rx_data[3] = 0x00;

        /* clear callback event flag. */
        cb_event = 0;

        /* Master receive */
        ret = I3Cdrv->MasterReceive(slave_addr, rx_data, len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Master Receive failed. \r\n");
            goto error_poweroff;
        }

        /* wait for callback event. */
        while(!((cb_event == I3C_CB_EVENT_SUCCESS) || (cb_event == I3C_CB_EVENT_ERROR)));

        if(cb_event == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C Master Receive failed.\n");
            while(1);
        }

        rx_cnt += 1;

        /* compare tx and rx data, stop if data does not match */
        cmp = memcmp(tx_data, rx_data, len);
        if(cmp != 0)
        {
           printf("\nError: TX and RX data mismatch.\n");
           while(1);
        }
}
error_poweroff:

    /* Power off I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
         printf("\r\n Error: I3C Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize I3C driver */
    ret = I3Cdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Uninitialize failed.\r\n");
    }

    printf("\r\n I3C demo exiting...\r\n");
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
    /* Enter the ThreadX kernel.  */
    i3c_master_loopback_demo();
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
