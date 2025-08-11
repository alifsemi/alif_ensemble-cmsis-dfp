/*------------------------------------------------------------------------------
 * MDK Middleware - Component
 * Copyright (c) 2004-2024 Arm Limited (or its affiliates). All rights reserved.
 *------------------------------------------------------------------------------
 */

#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_stdout.h"
#include "retarget_init.h"
#endif /* RTE_Compiler_IO_STDOUT */
#include <stdio.h>
#include <inttypes.h>

#include "cmsis_os2.h"
#include "rl_usb.h"

#include "se_services_port.h"
#include "app_utils.h"

/* Main stack size must be multiple of 8 Bytes  */
#define APP_MAIN_STK_SZ (4096U)
static uint64_t             app_main_stk[APP_MAIN_STK_SZ / 8];
static const osThreadAttr_t app_main_attr = {
    .stack_mem  = &app_main_stk[0],
    .stack_size = sizeof(app_main_stk)
};

/*----------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------
 */
__NO_RETURN static void app_main(void *argument)
{
    (void) argument;
    printf("USB Device App Started\n");
    /* SE -Service calls  */
    uint32_t      error_code         = 0;
    uint32_t      service_error_code = 0;
    run_profile_t runp               = {0};
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    int32_t    ret;

    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif
    /* Initialize the SE services */
    se_services_port_init();
    /* Example code to enable the CLKEN_USB clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_CLK_20M,
                                              /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: USB 20MHz clock enable: %" PRId32 "\n", error_code);
        WAIT_FOREVER_LOOP
    }
    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("SE: Failed to get run cfg: %" PRId32 "\n", error_code);
        WAIT_FOREVER_LOOP
    }
    runp.phy_pwr_gating |= USB_PHY_MASK;
    runp.memory_blocks   = SRAM0_MASK | MRAM_MASK;

    /* Set the current run configuration to SE */
    error_code           = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("SE: Failed to set run cfg: %" PRId32 "\n", error_code);
        WAIT_FOREVER_LOOP
    }
    USBD_Initialize(0U); /* USB Device 0 Initialization */
    USBD_Connect(0U);    /* USB Device 0 Connect        */

    osThreadExit();
}

int main(void)
{

    /* System Initialization */
    SystemCoreClockUpdate();

    osKernelInitialize();                        /* Initialize CMSIS-RTOS */
    osThreadNew(app_main, NULL, &app_main_attr); /* Create application main thread */
    osKernelStart();                             /* Start thread execution */
    ASSERT_HANG_LOOP
}
