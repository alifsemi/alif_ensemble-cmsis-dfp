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
 * @file     i2c_baremetal.c
 * @brief    Baremetal demo application to verify I2C Master and
 *           Slave functionality without any operating system
 *
 *           Code will verify below cases:
 *           1) Master transmit 30 bytes and Slave receive 30 bytes
 *           2) Slave transmit 29 bytes and Master receive 29 bytes
 *           I2C1 instance is taken as Master (PIN used P7_2 and P7_3)
 *           I2C0 instance is taken as Slave  (PIN used P7_0 and P7_1)
 *
 *           Hardware setup:
 *           - Connecting GPIO pins of I2C1 TO I2C0
 *             SDA pin P7_2(J14) to P7_0(J14)
 *             SCL pin P7_3(J14) to P7_1(J14).
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "pinconf.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


#define ADDRESS_MODE_7BIT   1                   /* I2C 7 bit addressing mode     */
#define ADDRESS_MODE_10BIT  2                   /* I2C 10 bit addressing mode    */
#define ADDRESS_MODE        ADDRESS_MODE_7BIT   /* 7 bit addressing mode chosen  */

/* I2C Driver instance */
extern ARM_DRIVER_I2C Driver_I2C1;
static ARM_DRIVER_I2C *I2C_MstDrv = &Driver_I2C1;

extern ARM_DRIVER_I2C Driver_I2C0;
static ARM_DRIVER_I2C *I2C_SlvDrv = &Driver_I2C0;

static volatile uint32_t mst_cb_status = 0;
static volatile uint32_t slv_cb_status = 0;

#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    #define TAR_ADDRS       (0X2D0)  /* 10 bit Target(Slave) Address, use by Master */
    #define SAR_ADDRS       (0X2D0)  /* 10 bit Slave Own Address,     use by Slave  */
#else
    #define TAR_ADDRS       (0X40)   /* 7 bit Target(Slave) Address, use by Master  */
    #define SAR_ADDRS       (0X40)   /* 7 bit Slave Own Address,     use by Slave   */
#endif

#define STOP            (0X00)
#if (RTE_I2C0_DMA_ENABLE && RTE_I2C1_DMA_ENABLE)
    #define I2C_DMA_ENABLED     1
#else
    #define I2C_DMA_ENABLED     0
#endif
#if I2C_DMA_ENABLED
    /* master transmit and slave receive */
    #define MST_BYTE_TO_TRANSMIT            11
    /* slave transmit and master receive */
    #define SLV_BYTE_TO_TRANSMIT            10
    /* Tx and Rx buffers are in multiples of 2bytes
     * as the DMA processes in 2bytes fashion only */
    /* Master parameter set */
    /* Master TX Data */
    static uint16_t __ALIGNED(4) MST_TX_BUF[MST_BYTE_TO_TRANSMIT] =
    {
        /* MST_TX_BUF data = "Master_Data" */
        77, 97, 115, 116, 101, 114, 95, 68, 97, 116, 97
    };
    /* master receive buffer */
    static uint16_t __ALIGNED(4) MST_RX_BUF[SLV_BYTE_TO_TRANSMIT];
    /* Master parameter set END  */
    /* Slave parameter set */
    /* slave receive buffer */
    static uint16_t __ALIGNED(4) SLV_RX_BUF[MST_BYTE_TO_TRANSMIT];
    static uint16_t __ALIGNED(4) SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT] =
    {
        /* SLV_TX_BUF data =  "Slave_Data" */
        83, 108, 97, 118, 101, 95, 68, 97, 116, 97
    };
    /* Slave parameter set END */
#else
    /* master transmit and slave receive */
    #define MST_BYTE_TO_TRANSMIT            30
    /* slave transmit and master receive */
    #define SLV_BYTE_TO_TRANSMIT            29
    /* Master parameter set */
    /* Master TX Data */
    static uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT] =
    {
        "!*!Test Message from Master!*!"
    };
    /* master receive buffer */
    static uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT];
    /* Master parameter set END  */
    /* Slave parameter set */
    /* slave receive buffer */
    static uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT];
    /* Slave TX Data */
    static uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT] =
    {
        "!*!Test Message from Slave!*!"
    };
    /* Slave parameter set END */
#endif

typedef enum _I2C_CB_EVENT{
    I2C_CB_EVENT_TRANSFER_DONE        = (1 << 0),
    I2C_CB_EVENT_ADDRESS_NACK         = (1 << 1),
    I2C_CB_EVENT_TRANSFER_INCOMPLETE  = (1 << 2)
}I2C_CB_EVENT;

/**
 * @fn      static void i2c_mst_conversion_callback(uint32_t event)
 * @brief   I2C master event callback
 * @note    none
 * @param   event : Callback Event
 * @retval  none
 */
static void i2c_mst_conversion_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        /* Transfer or receive is finished */
        mst_cb_status |= I2C_CB_EVENT_TRANSFER_DONE;
    }

    if (event & ARM_I2C_EVENT_ADDRESS_NACK)
    {
        /* Address NACKED */
        mst_cb_status |= I2C_CB_EVENT_ADDRESS_NACK;
    }

    if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE)
    {
        /* Transfer or receive is incomplete */
        mst_cb_status |= I2C_CB_EVENT_TRANSFER_INCOMPLETE;
    }

}

/**
 * @fn      static void i2c_slv_conversion_callback(uint32_t event)
 * @brief   I2C slave event callback
 * @note    none
 * @param   event : Callback Event
 * @retval  none
 */
static void i2c_slv_conversion_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        /* Transfer or receive is finished */
           slv_cb_status = I2C_CB_EVENT_TRANSFER_DONE;
    }
}

/**
 * @fn      static void hardware_init(void)
 * @brief   I2C0 and I2C1 pinmux configuration.
 * @note    none
 * @param   none
 * @retval  none
 */
static void hardware_init(void)
{
    /* I2C0_SDA_C */
    pinconf_set(PORT_7, PIN_0, PINMUX_ALTERNATE_FUNCTION_4,
                (PADCTRL_READ_ENABLE));

    /* I2C0_SCL_C */
    pinconf_set(PORT_7, PIN_1, PINMUX_ALTERNATE_FUNCTION_4,
                (PADCTRL_READ_ENABLE));

    /* I2C1_SDA_C */
    pinconf_set(PORT_7, PIN_2, PINMUX_ALTERNATE_FUNCTION_4,
                (PADCTRL_READ_ENABLE));

    /* I2C1_SCL_C */
    pinconf_set(PORT_7, PIN_3, PINMUX_ALTERNATE_FUNCTION_4,
                (PADCTRL_READ_ENABLE));
}

/**
 * @fn      static void I2C_demo(void)
 * @brief   Performs I2C master and slave comm demo
 * @note    none
 * @param   none
 * @retval  none
 */
static void I2C_demo(void)
{
    int   ret      = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> I2C demo starting up!!! <<< \r\n");

    /* Pinmux */
    hardware_init();

    version = I2C_MstDrv->GetVersion();
    printf("\r\n I2C version api:0x%X driver:0x%X...\r\n",
            version.api, version.drv);
    /* Initialize Master I2C driver */
    ret = I2C_MstDrv->Initialize(i2c_mst_conversion_callback);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C master init failed\n");
        return;
    }

    /* Initialize Slave I2C driver */
    ret = I2C_SlvDrv->Initialize(i2c_slv_conversion_callback);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C slave init failed\n");
        return;
    }

    /* I2C Master Power control  */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Master Power up failed\n");
        goto error_uninitialize;
    }

    /* I2C Slave Power control */
    ret = I2C_SlvDrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Slave Power up failed\n");
        goto error_uninitialize;
    }

    /* I2C Master Control */
    ret = I2C_MstDrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C master control failed\n");
        goto error_poweroff;
    }

    /* I2C Slave Control */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS,
                             (SAR_ADDRS | ARM_I2C_ADDRESS_10BIT));
#else
    ret = I2C_SlvDrv->Control(ARM_I2C_OWN_ADDRESS, SAR_ADDRS);
#endif
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C slave control failed\n");
        goto error_poweroff;
    }

    printf("\n-------Master transmit/slave receive-------\n");

    /* Clear mst/slv cb_status */
    mst_cb_status = 0;
    slv_cb_status = 0;

    /* I2C Slave Receive */
    ret = I2C_SlvDrv->SlaveReceive((uint8_t*)SLV_RX_BUF,
                                   MST_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Slave Receive failed\n");
        goto error_poweroff;
    }

    /* delay */
    sys_busy_loop_us(500);

     /* I2C Master Transmit*/
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    I2C_MstDrv->MasterTransmit((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                               (uint8_t*)MST_TX_BUF,
                               MST_BYTE_TO_TRANSMIT,
                               STOP);
#else
    I2C_MstDrv->MasterTransmit(TAR_ADDRS, (uint8_t*)MST_TX_BUF,
                               MST_BYTE_TO_TRANSMIT, STOP);
#endif
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Master Transmit failed\n");
        goto error_poweroff;
    }

    /* wait for master/slave callback. */
    while(mst_cb_status == 0);

    if (mst_cb_status & I2C_CB_EVENT_ADDRESS_NACK)
    {
        printf("\r\n Error: Slave NACKED\n");
        goto error_poweroff;
    }
    else if (mst_cb_status & I2C_CB_EVENT_TRANSFER_INCOMPLETE)
    {
        printf("\r\n Error: Transfer incomplete\n");
        goto error_poweroff;
    }

    while(slv_cb_status == 0);

    /* Compare received data. */
#if I2C_DMA_ENABLED
    if (memcmp(&SLV_RX_BUF, &MST_TX_BUF, (MST_BYTE_TO_TRANSMIT * 2)))
#else
    if (memcmp(&SLV_RX_BUF, &MST_TX_BUF, MST_BYTE_TO_TRANSMIT))
#endif
    {
        printf("\n Error: Master transmit/slave receive failed \n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        while(1);
    }

    printf("\n-------Master receive/slave transmit-------\n");

    /* Clear mst/slv cb_status */
    mst_cb_status = 0;
    slv_cb_status = 0;

    /* I2C Master Receive */
#if (ADDRESS_MODE == ADDRESS_MODE_10BIT)
    ret = I2C_MstDrv->MasterReceive((TAR_ADDRS | ARM_I2C_ADDRESS_10BIT),
                                    (uint8_t*)MST_RX_BUF,
                                    SLV_BYTE_TO_TRANSMIT,
                                    STOP);
#else
    ret = I2C_MstDrv->MasterReceive(TAR_ADDRS, (uint8_t*)MST_RX_BUF,
                                    SLV_BYTE_TO_TRANSMIT, STOP);
#endif
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Master Receive failed\n");
        goto error_poweroff;
    }

    sys_busy_loop_us(1000);

    /* I2C Slave Transmit */
    I2C_SlvDrv->SlaveTransmit((uint8_t*)SLV_TX_BUF,
                              SLV_BYTE_TO_TRANSMIT);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Slave Transmit failed\n");
        goto error_poweroff;
    }

    /* wait for master/slave callback. */
    while(mst_cb_status == 0);

    /* Slave Address NACKED */
    if (mst_cb_status & I2C_CB_EVENT_ADDRESS_NACK)
    {
        printf("\r\n Error: Slave NACKED\n");
        goto error_poweroff;
    }
    else if (mst_cb_status & I2C_CB_EVENT_TRANSFER_INCOMPLETE) /* Transfer incomplete */
    {
        printf("\r\n Error: Transfer incomplete\n");
        goto error_poweroff;
    }

    while(slv_cb_status == 0);

    sys_busy_loop_us(1000);

    /* Compare received data. */
#if I2C_DMA_ENABLED
    if (memcmp(&SLV_TX_BUF, &MST_RX_BUF, (SLV_BYTE_TO_TRANSMIT * 2)))
#else
    if (memcmp(&SLV_TX_BUF, &MST_RX_BUF, SLV_BYTE_TO_TRANSMIT))
#endif
    {
        printf("\n Error: Master receive/slave transmit failed\n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        while(1);
    }

    printf("\n >>> I2C conversion completed without any error <<< \n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    while(1);

error_poweroff:
    /* Power off I2C peripheral */
    ret = I2C_MstDrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Master Power OFF failed\r\n");
    }
    ret = I2C_SlvDrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C Slave Power OFF failed\r\n");
    }

error_uninitialize:
    /* Un-initialize I2C driver */
    ret = I2C_MstDrv->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n I2C Master Uninitialize failed\r\n");
    }
    ret = I2C_SlvDrv->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n I2C Slave Uninitialize failed\r\n");
    }
    printf("\r\n I2C demo thread exiting...\r\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    while(1);
}

/**
 * @fn      int main(void)
 * @brief   Entry point for I2C comm
 * @note    none
 * @param   none
 * @retval  none
 */
int main (void)
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

    I2C_demo();
    while(1);
}
