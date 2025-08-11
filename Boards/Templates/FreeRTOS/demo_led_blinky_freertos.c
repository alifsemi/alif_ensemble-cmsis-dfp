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
 * @file     : demo_led_blinky_freertos.c
 * @author   : Manoj A Murudi
 * @email    : manoj.murudi@alifsemi.com
 * @version  : V1.0.0
 * @date     : 25-May-2023
 * @brief    : FreeRTOS DEMO application for LED blink.
 *             - Toggle LED0_R and LED1_B alternately
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* Includes */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <RTE_Components.h>
#include CMSIS_device_header
#include "pinconf.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "Driver_IO.h"
#include "board_config.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

// Set to 0: Use application-defined LED pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG  0

/*Define for FreeRTOS*/
#define STACK_SIZE                    1024
#define TIMER_SERVICE_TASK_STACK_SIZE configTIMER_TASK_STACK_DEPTH  // 512
#define IDLE_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE      // 1024

StackType_t  IdleStack[2 * IDLE_TASK_STACK_SIZE];
StaticTask_t IdleTcb;
StackType_t  TimerStack[2 * TIMER_SERVICE_TASK_STACK_SIZE];
StaticTask_t TimerTcb;

/****************************** FreeRTOS functions **********************/

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t  **ppxIdleTaskStackBuffer,
                                   uint32_t      *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &IdleTcb;
    *ppxIdleTaskStackBuffer = IdleStack;
    *pulIdleTaskStackSize   = IDLE_TASK_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pxTask;

    ASSERT_HANG_LOOP
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t  **ppxTimerTaskStackBuffer,
                                    uint32_t      *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &TimerTcb;
    *ppxTimerTaskStackBuffer = TimerStack;
    *pulTimerTaskStackSize   = TIMER_SERVICE_TASK_STACK_SIZE;
}

void vApplicationIdleHook(void)
{
    ASSERT_HANG_LOOP
}

/*****************Only for FreeRTOS use *************************/

/* Define the FreeRTOS object control blocks...  */
#define DEMO_STACK_SIZE 1024

TaskHandle_t led_xHandle;

/* GPIO port used for LEDRGB0_R */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LEDRGB0_R_GPIO_PORT);
ARM_DRIVER_GPIO       *gpioDrvLed0R = &ARM_Driver_GPIO_(BOARD_LEDRGB0_R_GPIO_PORT);

/* GPIO port used for LEDRGB0_G */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LEDRGB0_G_GPIO_PORT);
ARM_DRIVER_GPIO       *gpioDrvLed0G = &ARM_Driver_GPIO_(BOARD_LEDRGB0_G_GPIO_PORT);

/* GPIO port used for LEDRGB0_B */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LEDRGB0_B_GPIO_PORT);
ARM_DRIVER_GPIO       *gpioDrvLed0B = &ARM_Driver_GPIO_(BOARD_LEDRGB0_B_GPIO_PORT);

#if (BOARD_LEDRGB_COUNT > 1)
/* GPIO port used for LEDRGB1_R */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LEDRGB1_R_GPIO_PORT);
ARM_DRIVER_GPIO       *gpioDrvLed1R = &ARM_Driver_GPIO_(BOARD_LEDRGB1_R_GPIO_PORT);

/* GPIO port used for LEDRGB1_G */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LEDRGB1_G_GPIO_PORT);
ARM_DRIVER_GPIO       *gpioDrvLed1G = &ARM_Driver_GPIO_(BOARD_LEDRGB1_G_GPIO_PORT);

/* GPIO port used for LEDRGB1_B */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LEDRGB1_B_GPIO_PORT);
ARM_DRIVER_GPIO       *gpioDrvLed1B = &ARM_Driver_GPIO_(BOARD_LEDRGB1_B_GPIO_PORT);
#endif

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_led_pins_config(void)
 * @brief   Configure LED pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_led_pins_config(void)
{
    int32_t status;

    status = pinconf_set(PORT_(BOARD_LEDRGB0_R_GPIO_PORT), BOARD_LEDRGB0_R_GPIO_PIN,
                         BOARD_LEDRGB0_R_ALTERNATE_FUNCTION, PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return APP_ERROR;
    }

    status = pinconf_set(PORT_(BOARD_LEDRGB0_B_GPIO_PORT), BOARD_LEDRGB0_B_GPIO_PIN,
                         BOARD_LEDRGB0_B_ALTERNATE_FUNCTION, PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return APP_ERROR;
    }

    status = pinconf_set(PORT_(BOARD_LEDRGB0_G_GPIO_PORT), BOARD_LEDRGB0_G_GPIO_PIN,
                         BOARD_LEDRGB0_G_ALTERNATE_FUNCTION, PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return APP_ERROR;
    }

#if (BOARD_LEDRGB_COUNT > 1)
    status = pinconf_set(PORT_(BOARD_LEDRGB1_R_GPIO_PORT), BOARD_LEDRGB1_R_GPIO_PIN,
                         BOARD_LEDRGB1_R_ALTERNATE_FUNCTION, PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return APP_ERROR;
    }
    status = pinconf_set(PORT_(BOARD_LEDRGB1_B_GPIO_PORT), BOARD_LEDRGB1_B_GPIO_PIN,
                         BOARD_LEDRGB1_B_ALTERNATE_FUNCTION, PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return APP_ERROR;
    }

    status = pinconf_set(PORT_(BOARD_LEDRGB1_G_GPIO_PORT), BOARD_LEDRGB1_G_GPIO_PIN,
                         BOARD_LEDRGB1_G_ALTERNATE_FUNCTION, PADCTRL_OUTPUT_DRIVE_STRENGTH_2MA);
    if (status) {
        return APP_ERROR;
    }
#endif

    return APP_SUCCESS;
}
#endif

/**
  \fn         void led_demo_Thread(void *pvParameters)
  \brief      LED blinky demo Thread
  \param[in]  pvParameters : thread input
  \return     none
*/
void led_demo_Thread(void *pvParameters)
{
    /*
     * gpio12 pin3 can be used as Red LED of LED0.
     * gpio7 pin4 can be used as Green LED of LED0.
     * gpio12 pin0 can be used as Blue LED of LED0.
     *
     * gpio6 pin2 can be used as Red LED of LED1.
     * gpio6 pin4 can be used as Green LED of LED1.
     * gpio6 pin6 can be used as Blue LED of LED1.
     *
     * This demo application is about.
     *   - Blink LED0_R and LED1_R, then LED0_B and LED1_B, then LED0_G and LED1_G simultaneously in
     * rotation. For E1C: gpio4 pin7 can be used as Red LED of LED0. gpio4 pin5 can be used as Green
     * LED of LED0. gpio4 pin3 can be used as Blue LED of LED0.
     *
     * This demo application is about.
     *   - Blink LED 0 with RGB color in a sequence.
     */

    int32_t ret1   = 0;
    int32_t ret2   = 0;
    uint8_t LED0_R = BOARD_LEDRGB0_R_GPIO_PIN;
    uint8_t LED0_G = BOARD_LEDRGB0_G_GPIO_PIN;
    uint8_t LED0_B = BOARD_LEDRGB0_B_GPIO_PIN;
#if (BOARD_LEDRGB_COUNT > 1)
    uint8_t LED1_R = BOARD_LEDRGB1_R_GPIO_PIN;
    uint8_t LED1_G = BOARD_LEDRGB1_G_GPIO_PIN;
    uint8_t LED1_B = BOARD_LEDRGB1_B_GPIO_PIN;
#endif
    const TickType_t xDelay = (1000 / portTICK_PERIOD_MS);

    printf("led blink demo application for FreeRTOS started\n\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    ret1 = board_pins_config();
    if (ret1 != 0) {
        printf("Error in pin-mux configuration: %"PRId32"\n", ret1);
        return;
    }
#else
    /*
     * NOTE: The LED pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret1 = board_led_pins_config();
    if (ret1 != 0) {
        printf("Error in pin-mux configuration %"PRId32"\n", ret1);
        return;
    }
#endif

    ret1 = gpioDrvLed0R->Initialize(LED0_R, NULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return;
    }
    ret1 = gpioDrvLed0G->Initialize(LED0_G, NULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return;
    }
    ret1 = gpioDrvLed0B->Initialize(LED0_B, NULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return;
    }

    ret1 = gpioDrvLed0R->PowerControl(LED0_R, ARM_POWER_FULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize;
    }
    ret1 = gpioDrvLed0G->PowerControl(LED0_G, ARM_POWER_FULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize;
    }
    ret1 = gpioDrvLed0B->PowerControl(LED0_B, ARM_POWER_FULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize;
    }

    ret1 = gpioDrvLed0R->SetDirection(LED0_R, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off;
    }
    ret1 = gpioDrvLed0G->SetDirection(LED0_G, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off;
    }
    ret1 = gpioDrvLed0B->SetDirection(LED0_B, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off;
    }

#if (BOARD_LEDRGB_COUNT > 1)
    ret2 = gpioDrvLed1R->Initialize(LED1_R, NULL);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return;
    }
    ret2 = gpioDrvLed1G->Initialize(LED1_G, NULL);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return;
    }
    ret2 = gpioDrvLed1B->Initialize(LED1_B, NULL);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return;
    }

    ret2 = gpioDrvLed1R->PowerControl(LED1_R, ARM_POWER_FULL);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        return;
    }
    ret2 = gpioDrvLed1G->PowerControl(LED1_G, ARM_POWER_FULL);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        return;
    }
    ret2 = gpioDrvLed1B->PowerControl(LED1_B, ARM_POWER_FULL);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        return;
    }

    ret2 = gpioDrvLed1R->SetDirection(LED1_R, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        return;
    }
    ret2 = gpioDrvLed1G->SetDirection(LED1_G, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        return;
    }
    ret2 = gpioDrvLed1B->SetDirection(LED1_B, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        return;
    }
#endif

    while (1) {
        /* Toggle Red LED */
        ret1 = gpioDrvLed0R->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_HIGH);
#if (BOARD_LEDRGB_COUNT > 1)
        ret2 = gpioDrvLed1R->SetValue(LED1_R, GPIO_PIN_OUTPUT_STATE_HIGH);
#endif
        if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_power_off;
        }

        /* delay for 1 Sec */
        vTaskDelay(xDelay);

        ret1 = gpioDrvLed0R->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_LOW);
#if (BOARD_LEDRGB_COUNT > 1)
        ret2 = gpioDrvLed1R->SetValue(LED1_R, GPIO_PIN_OUTPUT_STATE_LOW);
#endif
        if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_power_off;
        }

        /* delay for 1 Sec */
        vTaskDelay(xDelay);

        /* Toggle Green LED */
        ret1 = gpioDrvLed0G->SetValue(LED0_G, GPIO_PIN_OUTPUT_STATE_HIGH);
#if (BOARD_LEDRGB_COUNT > 1)
        ret2 = gpioDrvLed1G->SetValue(LED1_G, GPIO_PIN_OUTPUT_STATE_HIGH);
#endif
        if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_power_off;
        }

        /* delay for 1 Sec */
        vTaskDelay(xDelay);

        ret1 = gpioDrvLed0G->SetValue(LED0_G, GPIO_PIN_OUTPUT_STATE_LOW);
#if (BOARD_LEDRGB_COUNT > 1)
        ret2 = gpioDrvLed1G->SetValue(LED1_G, GPIO_PIN_OUTPUT_STATE_LOW);
#endif
        if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_power_off;
        }

        /* delay for 1 Sec */
        vTaskDelay(xDelay);

        /* Toggle Blue LED */
        ret1 = gpioDrvLed0B->SetValue(LED0_B, GPIO_PIN_OUTPUT_STATE_HIGH);
#if (BOARD_LEDRGB_COUNT > 1)
        ret2 = gpioDrvLed1B->SetValue(LED1_B, GPIO_PIN_OUTPUT_STATE_HIGH);
#endif
        if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_power_off;
        }

        /* delay for 1 Sec */
        vTaskDelay(xDelay);

        ret1 = gpioDrvLed0B->SetValue(LED0_B, GPIO_PIN_OUTPUT_STATE_LOW);
#if (BOARD_LEDRGB_COUNT > 1)
        ret2 = gpioDrvLed1B->SetValue(LED1_B, GPIO_PIN_OUTPUT_STATE_LOW);
#endif
        if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_power_off;
        }

        /* delay for 1 Sec */
        vTaskDelay(xDelay);
    };

error_power_off:

    ret1 = gpioDrvLed0R->PowerControl(LED0_R, ARM_POWER_OFF);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off \n");
    } else {
        printf("LEDs power off \n");
    }
    ret1 = gpioDrvLed0G->PowerControl(LED0_G, ARM_POWER_OFF);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off \n");
    } else {
        printf("LEDs power off \n");
    }
    ret1 = gpioDrvLed0B->PowerControl(LED0_B, ARM_POWER_OFF);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off \n");
    } else {
        printf("LEDs power off \n");
    }
#if (BOARD_LEDRGB_COUNT > 1)
    ret2 = gpioDrvLed1R->PowerControl(LED1_R, ARM_POWER_OFF);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off \n");
    } else {
        printf("LEDs power off \n");
    }
    ret2 = gpioDrvLed1G->PowerControl(LED1_G, ARM_POWER_OFF);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off \n");
    } else {
        printf("LEDs power off \n");
    }
    ret2 = gpioDrvLed1B->PowerControl(LED1_B, ARM_POWER_OFF);
    if (ret2 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off \n");
    } else {
        printf("LEDs power off \n");
    }
#endif

error_uninitialize:

    ret1 = gpioDrvLed0R->Uninitialize(LED0_R);
    if (ret1 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize \n");
    } else {
        printf("Un-initialized \n");
    }
    ret1 = gpioDrvLed0G->Uninitialize(LED0_G);
    if (ret1 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize \n");
    } else {
        printf("Un-initialized \n");
    }
    ret1 = gpioDrvLed0B->Uninitialize(LED0_B);
    if (ret1 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize \n");
    } else {
        printf("Un-initialized \n");
    }
#if (BOARD_LEDRGB_COUNT > 1)
    ret2 = gpioDrvLed1R->Uninitialize(LED1_R);
    if (ret2 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize \n");
    } else {
        printf("Un-initialized \n");
    }
    ret2 = gpioDrvLed1G->Uninitialize(LED1_G);
    if (ret2 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize \n");
    } else {
        printf("Un-initialized \n");
    }
    ret2 = gpioDrvLed1B->Uninitialize(LED1_B);
    if (ret2 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize \n");
    } else {
        printf("Un-initialized \n");
    }
#endif
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start the FreeRTOS Kernel
 *---------------------------------------------------------------------------*/
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
    /* System Initialization */
    SystemCoreClockUpdate();
    /* Create application main thread */
    BaseType_t xReturned = xTaskCreate(led_demo_Thread,
                                       "led_demo_Thread",
                                       216,
                                       NULL,
                                       configMAX_PRIORITIES - 1,
                                       &led_xHandle);
    if (xReturned != pdPASS) {
        vTaskDelete(led_xHandle);
        return -1;
    }

    /* Start thread execution */
    vTaskStartScheduler();
}
/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
