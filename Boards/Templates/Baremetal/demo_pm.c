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
 * @file     : demo_pm.c
 * @author   : Raj Ranjan
 * @email    : raj.ranjan@alifsemi.com
 * @version  : V2.0.0
 * @date     : 23-Feb-2023
 * @brief    : This testcase uses UART for selecting the different sleep modes
 *              supported by the core.
 *              UART2 will be used by the HE core(RX - P1_0, TX - P1_1)
 *              UART4 will be used by the HP core(RX - P12_1, TX - P12_2)
 *              Wakeup Sources:
 *                 HP - RTC     : Default set to 20sec
 *                 HE - LPTIMER : Default set to 20sec
 *                 HP & HE - LPGPIO P15_4 : Press the JOYSWITCH(SW1) to wakeup
 *
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <RTE_Components.h>
#include CMSIS_device_header
#include "se_services_port.h"
#include "clock_runtime.h"
#include "app_utils.h"

#if defined(RTE_CMSIS_Compiler_STDOUT) || defined(RTE_CMSIS_Compiler_STDIN)
#include "retarget_init.h"
#endif

#if defined(RTE_CMSIS_Compiler_STDIN)
#include "retarget_stdin.h"
#endif /* RTE_CMSIS_Compiler_STDIN */

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "pinconf.h"

#define DEBUG_PM 0

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#if defined(BALLETTO_DEVICE) || defined(ENSEMBLE_SOC_E1C)
#define APP_MEM_RET_BLOCKS  (SRAM4_1_MASK | SRAM4_2_MASK | SRAM4_3_MASK | SRAM4_4_MASK | \
                             SRAM5_1_MASK | SRAM5_2_MASK | SRAM5_3_MASK | SRAM5_4_MASK | \
                             SRAM5_5_MASK)
#else
#define APP_MEM_RET_BLOCKS  (SRAM4_1_MASK | SRAM4_2_MASK | SRAM5_1_MASK | SRAM5_2_MASK)
#endif

#if defined(RTSS_HE)
/*******************************   RTC       **********************************/

/* Project Includes */
#include "Driver_RTC.h"

/* RTC Driver instance 0 */
extern ARM_DRIVER_RTC  Driver_RTC0;
static ARM_DRIVER_RTC *RTCdrv = &Driver_RTC0;

/**
  \fn           void alarm_callback(uint32_t event)
  \brief        rtc alarm callback
  \return       none
*/
static void rtc_callback(uint32_t event)
{
    if (event & ARM_RTC_EVENT_ALARM_TRIGGER) {
        /* User code for call back */
        printf("\r\n RTC CB\r\n");
    }
    return;
}

/**
  @fn           void rtc_error_uninitialize(void)
  @brief        RTC un-initializtion:
  @return       none
*/
static int rtc_error_uninitialize(void)
{
    int ret = -1;

    /* Un-initialize RTC driver */
    ret     = RTCdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC Uninitialize failed.\r\n");
    }
    return ret;
}

/**
  @fn           int rtc_error_power_off(void)
  @brief        RTC power-off:
  @return       none
*/
static int rtc_error_power_off(void)
{
    int ret = -1;

    /* Power off RTC peripheral */
    ret     = RTCdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC Power OFF failed.\r\n");
        return ret;
    }

    ret = rtc_error_uninitialize();

    return ret;
}

/**
  @fn           int set_rtc()
  @brief        set RTC timeout value (in second i.e. timeout = 10 means 10 sec)
  @return       none
*/
static int set_rtc(uint32_t timeout)
{
    uint32_t val = 0;
    int      ret;

    ret = RTCdrv->ReadCounter(&val);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC read failed\n");
        rtc_error_power_off();
        return ret;
    }

    ret = RTCdrv->Control(ARM_RTC_SET_ALARM, val + timeout);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC Could not set alarm\n");
        rtc_error_power_off();
        return ret;
    }
    printf("\r\n Setting alarm after %" PRIu32 " (curr %" PRIu32 ", future %" PRIu32
           ") counts \r\n",
           timeout,
           val,
           (val + timeout));
    return ret;
}

/**
  @fn           int rtc_init(void)
  @brief        RTC Initialization only:
  @return       none
*/
static int rtc_init(void)
{
    int32_t              ret = -1;
    ARM_RTC_CAPABILITIES capabilities;

    capabilities = RTCdrv->GetCapabilities();
    if (!capabilities.alarm) {
        printf("\r\n Error: RTC alarm capability is not available.\n");
        return ret;
    }

    /* Initialize RTC driver */
    ret = RTCdrv->Initialize(rtc_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC init failed\r\n");
        return ret;
    }

    /* Enable the power for RTC */
    ret = RTCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: RTC Power up failed\n");
        rtc_error_uninitialize();
        return ret;
    }

    return ret;
}

/*******************************   RTC       **********************************/
#else /* RTSS_HP */
/*******************************   LPTIMER   **********************************/

/* Project Includes */
#include "Driver_LPTIMER.h"

/* LPTIMER Driver instance 0 */
extern ARM_DRIVER_LPTIMER  Driver_LPTIMER0;
static ARM_DRIVER_LPTIMER *lptimerDrv = &Driver_LPTIMER0;

#define LPTIMER_CHANNEL 0

/**
  \fn           void lptimer_callback(uint8_t event)
  \brief        lptimer callback
  \return       none
*/
static void lptimer_callback(uint8_t event)
{
    int32_t ret = 0;

    if (event == ARM_LPTIMER_EVENT_UNDERFLOW) {
        /* User code for call back */
        printf("\r\n LPTIMER CB\r\n");
        ret = lptimerDrv->Stop(LPTIMER_CHANNEL);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to Stop channel %" PRId16 "\n", LPTIMER_CHANNEL);
        }
    }

    return;
}

/**
  @fn           void lptimer_error_uninitialize(void)
  @brief        LPTIMER un-initializtion:
  @return       none
*/
static int lptimer_error_uninitialize(void)
{
    int32_t ret = -1;

    /* Un-initialize lptimer driver */
    ret         = lptimerDrv->Uninitialize(LPTIMER_CHANNEL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: LPTIMER Uninitialize failed.\r\n");
    }
    return ret;
}

/**
  @fn           int lptimer_error_power_off(void)
  @brief        LPTIMER power-off:
  @return       none
*/
static int lptimer_error_power_off(void)
{
    int32_t ret = -1;

    /* Power off LPTIMER peripheral */
    ret         = lptimerDrv->PowerControl(LPTIMER_CHANNEL, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: LPTIMER Power OFF failed.\r\n");
        return ret;
    }

    ret = lptimer_error_uninitialize();

    return ret;
}

/**
  @fn           int set_lptimer(uint32_t  timeout)
  @brief        set lptimer timeout value (in second i.e. timeout = 10 means 10 sec)
  @return       none
*/
static int set_lptimer(uint32_t timeout)
{
    int32_t  ret;
    uint32_t count = 0;

    /*
     *Configuring the lptimer channel for the timeout in seconds
     *Clock Source depends on RTE_LPTIMER_CHANNEL_CLK_SRC in RTE_Device.h
     *RTE_LPTIMER_CHANNEL_CLK_SRC = 0 : 32.768KHz freq (Default)
     */

    count          = timeout * (32768);

    /**< Loading the counter value >*/
    ret            = lptimerDrv->Control(LPTIMER_CHANNEL, ARM_LPTIMER_SET_COUNT1, &count);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: channel '%" PRId16 "'failed to load count\r\n", LPTIMER_CHANNEL);
        lptimer_error_power_off();
        return ret;
    }

    printf("\r\n Setting lptimer for %" PRIu32 " seconds \r\n", timeout);

    ret = lptimerDrv->Start(LPTIMER_CHANNEL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: failed to start channel '%" PRId16 "'\r\n", LPTIMER_CHANNEL);
        lptimer_error_uninitialize();
        return ret;
    }

    return ret;
}

/**
  @fn           int32_t lptimer_init(void)
  @brief        LPTIMER Initialization
  @return       Status
*/
static int32_t lptimer_init(void)
{
    int32_t ret = -1;

    /* Initialize LPTIMER driver */
    ret         = lptimerDrv->Initialize(LPTIMER_CHANNEL, lptimer_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: LPTIMER init failed\r\n");
        return ret;
    }

    /* Enable the power for LPTIMER */
    ret = lptimerDrv->PowerControl(LPTIMER_CHANNEL, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: LPTIMER Power up failed\n");
        lptimer_error_uninitialize();
        return ret;
    }

    return ret;
}

/*******************************   LPTIMER   **********************************/
#endif

/*******************************   LPGPIO    **********************************/

#include "Driver_IO.h"

static uint32_t volatile gpioevent;

static void gpio_cb(uint32_t event)
{
    gpioevent = event;
    printf("\r\n GPIO CB\r\n");
}

static void lpgpio_init(void)
{
    extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(LP);
    ARM_DRIVER_GPIO       *gpio15Drv = &ARM_Driver_GPIO_(LP);
    int32_t                ret;
    const uint8_t          pin_no[] = { PIN_0,
                                        PIN_1,
#if (LPGPIO_MAX_PIN >= 2)
                                        PIN_4
#endif
                                      };

    uint32_t control_code = ARM_GPIO_IRQ_SENSITIVE_EDGE | ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE |
                            ARM_GPIO_IRQ_POLARITY_LOW;

    for (uint8_t i = 0; i < ARRAY_SIZE(pin_no); i++) {
        ret = pinconf_set(PORT_15,
                          pin_no[i],
                          PINMUX_ALTERNATE_FUNCTION_0,
                          PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
        if (ret) {
            WAIT_FOREVER_LOOP
        }

        ret = gpio15Drv->Initialize(pin_no[i], gpio_cb);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: LPGPIO init failed\r\n");
            WAIT_FOREVER_LOOP
        }

        ret = gpio15Drv->PowerControl(pin_no[i], ARM_POWER_FULL);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: LPGPIO Power Control failed\r\n");
            WAIT_FOREVER_LOOP
        }

        ret = gpio15Drv->SetDirection(pin_no[i], GPIO_PIN_DIRECTION_INPUT);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: LPGPIO Direction Set failed\r\n");
            WAIT_FOREVER_LOOP
        }

        ret = gpio15Drv->Control(pin_no[i], ARM_GPIO_ENABLE_INTERRUPT, &control_code);
        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error: LPGPIO Interrupt Set failed\r\n");
            WAIT_FOREVER_LOOP
        }
    }
}

/*******************************   LPGPIO    **********************************/

/*******************************   PM   ***************************************/

/* Project Includes */
#include "pm.h"

#define POWER_MSG_CNT 6

/**
  @brief enum pm_sleep_type:-
 */
typedef enum _PM_SLEEP_TYPE {
    PM_SLEEP_TYPE_NORMAL_SLEEP = 1,   /*!< Device is in Full operation                */
    PM_SLEEP_TYPE_DEEP_SLEEP,         /*!< Device Core clock will be off              */
    PM_SLEEP_TYPE_SUBSYS_OFF_STOP,    /*!< Device will be off, SoC can go to STOP     */
    PM_SLEEP_TYPE_SUBSYS_OFF_IDLE,    /*!< Device will be off, SoC can go to IDLE     */
    PM_SLEEP_TYPE_SUBSYS_OFF_STANDBY, /*!< Device will be off, SoC can go to STANDBY  */

    PM_SLEEP_TYPE_MAX = 0x7FFFFFFFUL
} PM_SLEEP_TYPE;

/* Data Required for PM */
static char power_msg[POWER_MSG_CNT][64] = {
    "\r\n\t1. Normal Sleep\r\n",
    "\r\n\t2. Deep Sleep\r\n",
    "\r\n\t3. Subsystem Off, Permitting STOP\r\n",
    "\r\n\t4. Subsystem Off, Permitting IDLE\r\n",
    "\r\n\t5. Subsystem Off, Permitting STANDBY\r\n",
    "\r\n\t6. Change Sleep Duration\r\n",
};

/**
  @fn           void pm_usage_menu(void)
  @brief        Power Management Menu
  @return       none
*/
static void pm_usage_menu(void)
{
    int i;

    printf("\r\nSelect Below Sleep Modes... \r\n");

    for (i = 0; i < POWER_MSG_CNT; i++) {
        printf("%s", &power_msg[i][0]);
    }
    printf("\r\n");

    return;
}

/**
  @fn           void pm_display_wakeup_reason()
  @brief        Display the wakeup reason
  @return       none
*/
static void pm_display_wakeup_reason(void)
{
    bool pending = false;
    struct {
        IRQn_Type irq;
        const char *reason;
    } wakeup_irqs[] = {
#if defined(RTSS_HP)
        { LPTIMER0_IRQ_IRQn, "LPTIMER0" },
#else
        { LPRTC0_IRQ_IRQn,   "RTC" },
#endif
        { LPGPIO_IRQ0_IRQn,  "LPGPIO0" },
        { LPGPIO_IRQ1_IRQn,  "LPGPIO1" },
#if (LPGPIO_MAX_PIN >= 2)
        { LPGPIO_IRQ4_IRQn,  "LPGPIO4" },
#endif
    };

    printf("\r\nWakeup Interrupt Reasons: ");
    for (size_t i = 0; i < ARRAY_SIZE(wakeup_irqs); ++i) {
        if (NVIC_GetPendingIRQ(wakeup_irqs[i].irq)) {
            printf("%s ", wakeup_irqs[i].reason);
            pending = true;
        }
    }
    if (!pending) {
        printf("None");
    }
    printf("\n");
}

static bool pm_is_any_wakeup_irq_pending(void)
{
#if defined(RTSS_HP)
    IRQn_Type wakeup_irqs[] = { LPTIMER0_IRQ_IRQn,
                                LPGPIO_IRQ0_IRQn,
                                LPGPIO_IRQ1_IRQn,
#if (LPGPIO_MAX_PIN >= 2)
                                LPGPIO_IRQ4_IRQn
#endif
                            };
#else
    IRQn_Type wakeup_irqs[] = { LPRTC0_IRQ_IRQn,
                                LPGPIO_IRQ0_IRQn,
                                LPGPIO_IRQ1_IRQn,
#if (LPGPIO_MAX_PIN >= 2)
                                LPGPIO_IRQ4_IRQn
#endif
                            };
#endif
    for (size_t i = 0; i < ARRAY_SIZE(wakeup_irqs); ++i) {
        if (NVIC_GetPendingIRQ(wakeup_irqs[i])) {
            return true;
        }
    }
    return false;
}

/**
  @fn           int main(void)
  @brief        Application Entry : Testapp for supported power modes.
  @return       exit code
*/
int main(void)
{
    PM_SLEEP_TYPE   selectedSleepType = PM_SLEEP_TYPE_NORMAL_SLEEP;
    PM_RESET_STATUS last_reset_reason;
    int32_t         ret           = -1;
    uint32_t        sleepDuration = 20; /*  Making Default sleep duration as 20s */
    uint32_t        service_error_code;
    uint32_t        error_code = SERVICES_REQ_SUCCESS;
    off_profile_t   offp       = {0 };
    run_profile_t   runp       = {0 };
    uint8_t         tempstr[4]; /* max length of the string */
    uint32_t        delay_count = 0;

    /* Get the last reason for the reboot */
    last_reset_reason           = pm_get_subsystem_reset_status();

    /* Initialize the SE services */
    se_services_port_init();

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }

    /*
     * Note:
     * This demo uses a specific profile setting that only enables the
     * items it needs. For example, it only requests the RAM regions and
     * peripheral power that are relevant for this demo. If you want to adapt
     * this example for your own use case, you should adjust the profile setting
     * accordingly. You can either add any additional items that you need, or
     * remove the request altogether to use the default setting that turns on
     * almost everything.
     */
    runp.power_domains = PD_SYST_MASK | PD_SSE700_AON_MASK;
#if defined(RTSS_HP)
    runp.memory_blocks = SRAM2_MASK | SRAM3_MASK | MRAM_MASK;
#else
    runp.memory_blocks = SRAM4_1_MASK | SRAM4_2_MASK | SRAM5_1_MASK | SRAM5_2_MASK;

    if (!RTSS_Is_TCM_Addr((const volatile void *) SCB->VTOR)) {
        runp.memory_blocks |= MRAM_MASK;
    }

#endif

    /* Set the new run configuration */
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }

    /* enable the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_HFOSC,
                                              /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }

    /* Update the system clock information */
    system_update_clock_values();

    /* Log Retargeting Initialization */

#if defined(RTE_CMSIS_Compiler_STDIN_Custom)
    ret = stdin_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* If it is POR, UART will take some time to show up */
    if (PM_RESET_STATUS_POR_OR_SOC_OR_HOST_RESET == last_reset_reason &&
        !pm_is_any_wakeup_irq_pending()) {
#if defined(RTSS_HP)
        /* Add Delay of 1sec so that uart can show up */
        delay_count = 1;
#else
        /* Add Delay of 10sec so that external FTDI USB-UART can show up */
        delay_count = 10;
#endif

        for (uint32_t count = 0; count < (delay_count * 10); count++) {
            sys_busy_loop_us(100 * 1000);
        }
    }

    printf("\r\n=========================================================\r\n");
#if defined(RTSS_HE)
    printf("\r\n    RTSS_HE: PM Test Application\r\n");
#else
    printf("\r\n    RTSS_HP: PM Test Application\r\n");
#endif
    printf("\r\n=========================================================\r\n");

    switch (last_reset_reason) {
    case PM_RESET_STATUS_POR_OR_SOC_OR_HOST_RESET:
        printf("\r\nLast Reset Reason = POR_OR_SOC_OR_HOST_RESET\n");
        break;
    case PM_RESET_STATUS_NSRST_RESET:
        printf("\r\nLast Reset Reason = NSRST_RESET\n");
        break;
    case PM_RESET_STATUS_EXTERNAL_SYS_RESET:
        printf("\r\nLast Reset Reason = EXTERNAL_SYS_RESET\n");
        break;
    default:
#if DEBUG_PM
        printf("\r\nLast Reset Reason = %" PRIx32 "\n", last_reset_reason);
#endif
        break;
    }

    /* Display the wakeup reason */
    pm_display_wakeup_reason();

    /* Enable GPIO15, as wakeup source */
    lpgpio_init();

#if defined(RTSS_HE)
    /* RTC Initialization */
    ret = rtc_init();
    if (ret != ARM_DRIVER_OK) {
        printf(" RTC Initialization failed (%" PRId32 ")\n", ret);
        return ret;
    }
#else
    /* LPTIMER Initialization */
    ret = lptimer_init();
    if (ret != ARM_DRIVER_OK) {
        printf(" LPTIMER Initialization failed (%" PRId32 ")\n", ret);
        return ret;
    }
#endif

    while (1) {
        pm_usage_menu();
#if defined(RTSS_HE)
        printf("\r\nRTSS_HE: Enter Sleep mode option:  ");
#else
        printf("\r\nRTSS_HP: Enter Sleep mode option:  ");
#endif
        scanf("%" PRIu32 "", (uint32_t *) &selectedSleepType);
        printf("%" PRId32 "\n", (int32_t) selectedSleepType);

        switch (selectedSleepType) {

        case PM_SLEEP_TYPE_NORMAL_SLEEP:

#if defined(RTSS_HE)
            ret = set_rtc(sleepDuration);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
#else
            ret = set_lptimer(sleepDuration);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
#endif
            printf("\r\nCore : Enter Normal Sleep...\r\n");

            // Disable all interrupt
            __disable_irq();

            // Go for Normal Sleep
            pm_core_enter_normal_sleep();  // setting wake Up source

            pm_display_wakeup_reason();

            // Enable IRQ
            __enable_irq();

            printf("\r\nCore : Exit Normal Sleep...\r\n");
            break;

        case PM_SLEEP_TYPE_DEEP_SLEEP:

#if defined(RTSS_HE)
            ret = set_rtc(sleepDuration);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
#else
            ret = set_lptimer(sleepDuration);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
#endif
            printf("\r\nCore : Enter Deep Sleep...\r\n");

            // Disable all interrupt
            __disable_irq();

            // Go for Deep Sleep
            pm_core_enter_deep_sleep();

            pm_display_wakeup_reason();

            // Enable IRQ
            __enable_irq();

            printf("\r\nCore : Exit Deep Sleep...\r\n");
            break;

        case PM_SLEEP_TYPE_SUBSYS_OFF_STOP:
        case PM_SLEEP_TYPE_SUBSYS_OFF_IDLE:
        case PM_SLEEP_TYPE_SUBSYS_OFF_STANDBY:

            /* Get the current off configuration from SE */
            error_code = SERVICES_get_off_cfg(se_services_s_handle, &offp, &service_error_code);
            if (error_code) {
                printf("\r\nSE: get_off_cfg error = %" PRIu32 "\n", error_code);
                WAIT_FOREVER_LOOP
            }

            if (selectedSleepType == PM_SLEEP_TYPE_SUBSYS_OFF_STANDBY) {
                offp.power_domains = PD_SSE700_AON_MASK;
            } else if (selectedSleepType == PM_SLEEP_TYPE_SUBSYS_OFF_IDLE) {
                offp.power_domains = PD_SSE700_AON_MASK | PD_SYST_MASK;
            } else {
                offp.power_domains = PD_VBAT_AON_MASK;
            }

            offp.aon_clk_src  = CLK_SRC_LFXO;
            offp.stby_clk_src = CLK_SRC_HFXO;
#if defined(RTSS_HP)
            offp.ewic_cfg      = EWIC_VBAT_TIMER | EWIC_VBAT_GPIO;
            offp.wakeup_events = WE_LPTIMER0 | WE_LPGPIO4 | WE_LPGPIO0 | WE_LPGPIO1;
#else
            offp.ewic_cfg      = EWIC_RTC_A | EWIC_VBAT_GPIO;
            offp.wakeup_events = WE_LPRTC | WE_LPGPIO4 | WE_LPGPIO0 | WE_LPGPIO1;
#endif
            offp.vtor_address  = SCB->VTOR;
            offp.memory_blocks = MRAM_MASK;

#if defined(RTSS_HE)
            /*
             * Enable the HE TCM retention only if the VTOR is present.
             * This is just for this test application.
             */
            if (RTSS_Is_TCM_Addr((const volatile void *) SCB->VTOR)) {
                offp.memory_blocks = APP_MEM_RET_BLOCKS | SERAM_MASK;
            } else {
                /* Enable SERAM if HE VTOR is in MRAM */
                offp.memory_blocks |= SERAM_MASK;
            }
#else
            /*
             * Retention is not possible with HP-TCM
             */
            if (RTSS_Is_TCM_Addr((const volatile void *) SCB->VTOR)) {
                printf("\r\nHP TCM Retention is not possible \n");
                continue;
            } else {
                offp.memory_blocks = MRAM_MASK;
            }
#endif

            error_code = SERVICES_set_off_cfg(se_services_s_handle, &offp, &service_error_code);
            if (error_code) {
                printf("\r\nSE: set_off_cfg error = %" PRIu32 "\n", error_code);
                WAIT_FOREVER_LOOP
            }

#if defined(RTSS_HE)
            /* Enable RTC as a wakeup source */
            ret = set_rtc(sleepDuration);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
#else
            ret = set_lptimer(sleepDuration);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
#endif
            printf("\r\nCore : Enter Subsystem off, ...\r\n");
#if defined(RTSS_HP)
            printf("\r\nWakeup Source Set : LPTIMER0 & GPIO P15_0, P15_1, P15_4 \r\n");
#else
            printf("\r\nWakeup Source Set : RTC & GPIO P15_0, P15_1, P15_4 \r\n");
#endif
            printf("\r\nVTOR = %" PRIx32 "\n", offp.vtor_address);

            // Disable all interrupt
            __disable_irq();

            // Go for Sleep
            pm_core_enter_deep_sleep_request_subsys_off();

            pm_display_wakeup_reason();

            // Enable IRQ
            __enable_irq();

            printf("\r\nCore : Subsystem didn't Poweroff...\r\n");
            break;

        default:
            printf("\r\nModify the Sleep duration, Enter 'y' to continue : ");
            scanf("%3s", tempstr);
            if ((tempstr[0] == 'y') || (tempstr[0] == 'Y')) {
                printf("\r\nEnter Sleep duration (in sec) : ");
                scanf("%" PRIu32 "", &sleepDuration);
                printf("%" PRIu32 "", sleepDuration);
                printf("\n");
            } else {
                printf("\n");
                continue;
            }
            break;
        }
    }

#if defined(RTSS_HE)
    /* enable the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_HFOSC,
                                              /*bool enable   */ false,
                                              &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
#endif

    return 0;
}

/********************** (c) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
