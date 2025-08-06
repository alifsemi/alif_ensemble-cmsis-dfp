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
 * @file     : demo_lpi2c0.c
 * @brief    : TestApp to verify I2C Master and LPI2C0 Slave functionality
 *           : using Baremetal without any operating system.
 * @note     : Code will verify:
 *              1.)Master transmit and Slave receive
 *              2.)Master receive  and Slave transmit
 *                  I2C0 instance is taken as Master and
 *                  LPI2C0(Slave-only) instance is taken as Slave.
 *
 *              Hardware Connection:
 *              I2C0 SDA(P3_5) -> LPI2C0 SDA(P5_3)
 *              I2C0 SCL(P3_4) -> LPI2C0 SCL(P5_2)
 ******************************************************************************/

/* Include */
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "pinconf.h"
#include "app_utils.h"
#include "board_config.h"

#if !defined(RTSS_HE)
#error "This Demo application works only on RTSS_HE"
#endif

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

// Set to 0: Use application-defined LPI2C0 pin configuration (via board_lpi2c_pins_config()).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* I2C Driver instance */
extern ARM_DRIVER_I2C  Driver_I2C0;
static ARM_DRIVER_I2C *I2C_mstdrv = &Driver_I2C0;

extern ARM_DRIVER_I2C  Driver_LPI2C0;
static ARM_DRIVER_I2C *LPI2C_slvdrv = &Driver_LPI2C0;

volatile uint32_t mst_cb_status;
volatile uint32_t slv_cb_status;

#define TAR_ADDRS            (0X40) /* Target(Slave) Address, use by Master */
#define RESTART              (0X01)
#define STOP                 (0X00)

/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT 21

/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT 22

/* Master parameter set */

/* Master TX Data (Any random value). */
static uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT + 1] = {"Test_Message_to_Slave"};

/* master receive buffer */
static uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT + 1];

/* Master parameter set END  */

/* Slave parameter set */

/* slave receive buffer */
static uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT + 1];

/* Slave TX Data (Any random value). */
static uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT + 1] = {"Test_Message_to_Master"};

/* Slave parameter set END */

/**
 * @fn      static void i2c_mst_tranfer_callback(uint32_t event)
 * @brief   I2C Callback function for events
 * @note    none
 * @param   event: I2C event
 * @retval  none
 */
static void i2c_mst_tranfer_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer or receive is finished */
        mst_cb_status = 1;
    }
}

/**
 * @fn      static void i2c_slv_transfer_callback(uint32_t event)
 * @brief   LPI2C Callback function for events
 * @note    none
 * @param   event: LPI2C event
 * @retval  none
 */
static void i2c_slv_transfer_callback(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer or receive is finished */
        slv_cb_status = 1;
    }
}

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_lpi2c_pins_config(void)
 * @brief   Configure additional lpi2c pinmux settings not handled
 *          by the board support library.
 * @retval  execution status.
 */
static int32_t board_lpi2c_pins_config(void)
{
    int32_t ret;

#if BOARD_LPI2C0_USE_FLEXIO_PINS
    /* for LPI2C0 Rev-A GPIO voltage level(flex)
     * has to be changed to 1.8-V power supply.
     * configure flexio pins to 1.8V
     */
    uint32_t      error_code = SERVICES_REQ_SUCCESS;
    uint32_t      service_error_code;
    run_profile_t runp;

    /* Initialize the SE services */
    se_services_port_init();

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("Get Current run config failed\n");
        WAIT_FOREVER_LOOP
    }

    runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
    /* Set the new run configuration */
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("Set new run config failed\n");
        WAIT_FOREVER_LOOP
    }
#endif

    /* LPI2C0_SDA */
    ret = pinconf_set(PORT_(BOARD_LPI2C0_SDA_GPIO_PORT),
                      BOARD_LPI2C0_SDA_GPIO_PIN,
                      BOARD_LPI2C0_SDA_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPI2C0_SDA_PIN\n");
        return ret;
    }

    /* LPI2C0_SCL */
    ret = pinconf_set(PORT_(BOARD_LPI2C0_SCL_GPIO_PORT),
                      BOARD_LPI2C0_SCL_GPIO_PIN,
                      BOARD_LPI2C0_SCL_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for LPI2C0_SCL_PIN\n");
        return ret;
    }

    /* I2C0_SDA */
    ret = pinconf_set(PORT_(BOARD_I2C0_SDA_GPIO_PORT),
                      BOARD_I2C0_SDA_GPIO_PIN,
                      BOARD_I2C0_SDA_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for I2C0_SDA_PIN\n");
        return ret;
    }

    /* I2C0_SCL */
    ret = pinconf_set(PORT_(BOARD_I2C0_SCL_GPIO_PORT),
                      BOARD_I2C0_SCL_GPIO_PIN,
                      BOARD_I2C0_SCL_ALTERNATE_FUNCTION,
                      (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                       PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA));
    if (ret) {
        printf("ERROR: Failed to configure PINMUX for I2C0_SCL_PIN\n");
        return ret;
    }

    return ret;
}
#endif

/**
 * @fn      static void LPI2C0_demo(void)
 * @brief   Performs LPI2C0 Demo
 * @note    none
 * @param   none
 * @retval  none
 */
static void LPI2C0_demo(void)
{
    int32_t            ret  = 0;
    uint8_t            iter = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> LPI2C0 demo starting up !!! <<< \r\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The lpi2c pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_lpi2c_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#endif

    version = I2C_mstdrv->GetVersion();
    printf("\r\n I2C version api:0x%" PRIx16 " driver:0x%" PRIx16 "...\r\n",
           version.api,
           version.drv);

    version = LPI2C_slvdrv->GetVersion();
    printf("\r\n LPI2C0 version api:0x%" PRIx16 " driver:0x%" PRIx16 "...\r\n",
           version.api,
           version.drv);

    /* Initialize I2C driver */
    ret = I2C_mstdrv->Initialize(i2c_mst_tranfer_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C master init failed\n");
        return;
    }

    /* Initialize I2C driver */
    ret = LPI2C_slvdrv->Initialize(i2c_slv_transfer_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C slave init failed\n");
        return;
    }

    /* Power control I2C */
    ret = I2C_mstdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Power up failed\n");
        goto error_uninitialize;
    }

    /* Power control I2C */
    ret = LPI2C_slvdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Power up failed\n");
        goto error_uninitialize;
    }

    ret = I2C_mstdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C master init failed\n");
        goto error_poweroff;
    }

    printf("\n----------------Master transmit/slave receive--------------\n");

    LPI2C_slvdrv->SlaveReceive(SLV_RX_BUF, MST_BYTE_TO_TRANSMIT);

    /* delay */
    sys_busy_loop_us(1000);

    I2C_mstdrv->MasterTransmit(TAR_ADDRS, MST_TX_BUF, MST_BYTE_TO_TRANSMIT, STOP);

    /* wait for master/slave callback. */
    while (mst_cb_status == 0) {
    }
    mst_cb_status = 0;

    while (slv_cb_status == 0) {
    }
    slv_cb_status = 0;

    /* Compare received data. */
    if (memcmp(&SLV_RX_BUF, &MST_TX_BUF, MST_BYTE_TO_TRANSMIT)) {
        printf("\n Error: Master transmit/slave receive failed \n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        WAIT_FOREVER_LOOP
    }

    printf("\n----------------Master receive/slave transmit--------------\n");

    for (iter = 0; iter < SLV_BYTE_TO_TRANSMIT; iter++) {
        I2C_mstdrv->MasterReceive(TAR_ADDRS, &MST_RX_BUF[iter], 1, STOP);

        LPI2C_slvdrv->SlaveTransmit(&SLV_TX_BUF[iter], 1);

        /* wait for master callback. */
        while (mst_cb_status == 0) {
        }
        mst_cb_status = 0;

        while (slv_cb_status == 0) {
        }
        slv_cb_status = 0;
    }

    /* Compare received data. */
    if (memcmp(&SLV_TX_BUF, &MST_RX_BUF, SLV_BYTE_TO_TRANSMIT)) {
        printf("\n Error: Master receive/slave transmit failed\n");
        printf("\n ---Stop--- \r\n wait forever >>> \n");
        WAIT_FOREVER_LOOP
    }

    ret = I2C_mstdrv->Uninitialize();
    if (ret == ARM_DRIVER_OK) {
        printf("\r\n I2C Master Uninitialized\n");
        goto error_uninitialize;
    }
    ret = LPI2C_slvdrv->Uninitialize();
    if (ret == ARM_DRIVER_OK) {
        printf("\r\n I2C Slave Uninitialized\n");
        goto error_uninitialize;
    }

    printf("\n >>> LPI2C0 transfer completed without any error\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    WAIT_FOREVER_LOOP

error_poweroff:
    /* Power off peripheral */
    ret = I2C_mstdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Power OFF failed.\r\n");
    }
    ret = LPI2C_slvdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: LPI2C0 Power OFF failed.\r\n");
    }

error_uninitialize:
    /* Un-initialize I2C driver */
    ret = I2C_mstdrv->Uninitialize();
    ret = LPI2C_slvdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I2C Uninitialize failed.\r\n");
    }
    printf("\r\n  LPI2C0 demo exiting...\r\n");
}

/**
 * @fn      int main(void)
 * @brief   Entry function of LPI2C0
 * @note    none
 * @param   none
 * @retval  none
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
    /* Enter the demo Application.  */
    LPI2C0_demo();

    return 0;
}
