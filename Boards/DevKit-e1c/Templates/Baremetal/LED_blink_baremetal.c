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
 * @file     LED_blink_baremetal.c
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     19-July-2024
 * @brief    DEMO application for LED blink on Bellatto.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
#include <stdio.h>

#include "Driver_GPIO.h"
#include "pinconf.h"

#include <RTE_Components.h>
#include CMSIS_device_header
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* Uncomment to use the pin configuration provided by the conductor tool */
//#define USE_CONDUCTOR_PIN_CONFIG

#ifdef USE_CONDUCTOR_PIN_CONFIG
#include "conductor_board_config.h"
#endif


/* LED0 gpio pins */
#define GPIO4_PORT                      4   /*< LED port >*/
#define PIN7                            7   /*< LED0_R gpio pin >*/
#define PIN5                            5   /*< LED0_G gpio pin >*/
#define PIN3                            3   /*< LED0_B gpio pin >*/

/* GPIO port used for LED0_R & LED0_B */
extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO4_PORT);
ARM_DRIVER_GPIO *gpioDrv4 = &ARM_Driver_GPIO_(GPIO4_PORT);

uint32_t volatile ms_ticks = 0;

void SysTick_Handler (void) {
  ms_ticks++;
}
void delay(uint32_t nticks)
{
      uint32_t c_ticks;

      c_ticks = ms_ticks;
      while ((ms_ticks - c_ticks) < nticks) ;
}


/**
  \fn         void led_blink_app(void)
  \brief      LED blinky function
  \param[in]  none
  \return     none
*/
void led_blink_app (void)
{
  /*
   * gpio4 pin7 can be used as Red LED of LED0.
   * gpio4 pin5 can be used as Green LED of LED0.
   * gpio4 pin3 can be used as Blue LED of LED0.
   *
   * This demo application is about.
   *   - Blink LED 0 with RGB color in a sequence.
   */

    int32_t ret1;
    uint8_t LED0_R = PIN7;
    uint8_t LED0_G = PIN5;
    uint8_t LED0_B = PIN3;

    printf("led blink demo application started\n\n");

#ifdef USE_CONDUCTOR_PIN_CONFIG
    ret1 = conductor_pins_config();

    if (ret1 != 0) {
        printf("ERROR: Conductor pin configuration failed\n");
        return;
    }
#else
    /* pinmux configurations for all GPIOs */
    pinconf_set(GPIO4_PORT, LED0_R, PINMUX_ALTERNATE_FUNCTION_0, 0);
    pinconf_set(GPIO4_PORT, LED0_G, PINMUX_ALTERNATE_FUNCTION_0, 0);
    pinconf_set(GPIO4_PORT, LED0_B, PINMUX_ALTERNATE_FUNCTION_0, 0);
#endif

    ret1 = gpioDrv4->Initialize(LED0_R, NULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize LED0_R\n");
        return;
    }
    ret1 = gpioDrv4->Initialize(LED0_G, NULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize LED0_G\n");
        return;
    }
    ret1 = gpioDrv4->Initialize(LED0_B, NULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize LED0_B\n");
        return;
    }

    ret1 = gpioDrv4->PowerControl(LED0_R, ARM_POWER_FULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full LED0_R\n");
        goto error_uninitialize;
    }
    ret1 = gpioDrv4->PowerControl(LED0_G, ARM_POWER_FULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full LED0_G\n");
        goto error_uninitialize;
    }
    ret1 = gpioDrv4->PowerControl(LED0_B, ARM_POWER_FULL);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full LED0_B\n");
        goto error_uninitialize;
    }

    ret1 = gpioDrv4->SetDirection(LED0_R, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure LED0_R\n");
        goto error_power_off;
    }
    ret1 = gpioDrv4->SetDirection(LED0_G, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure LED0_G\n");
        goto error_power_off;
    }
    ret1 = gpioDrv4->SetDirection(LED0_B, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure LED0_B\n");
        goto error_power_off;
    }

    while (1)
    {
        /* Toggle Red LED */
        ret1 = gpioDrv4->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_HIGH);
        if (ret1 != ARM_DRIVER_OK) {
            printf("ERROR: Failed to toggle LEDs LED0_R\n");
            goto error_power_off;
        }

        /* wait for 1 Sec */
        delay(1000);

        ret1 = gpioDrv4->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_LOW);
        if (ret1 != ARM_DRIVER_OK) {
            printf("ERROR: Failed to toggle LEDs LED0_R\n");
            goto error_power_off;
        }

        /* wait for 1 Sec */
        delay(1000);


        /* Toggle Green LED */
        ret1 = gpioDrv4->SetValue(LED0_G, GPIO_PIN_OUTPUT_STATE_HIGH);
        if (ret1 != ARM_DRIVER_OK) {
            printf("ERROR: Failed to toggle LEDs LED0_G\n");
            goto error_power_off;
        }

        /* wait for 1 Sec */
        delay(1000);

        ret1 = gpioDrv4->SetValue(LED0_G, GPIO_PIN_OUTPUT_STATE_LOW);
        if (ret1 != ARM_DRIVER_OK) {
            printf("ERROR: Failed to toggle LEDs LED0_G\n");
            goto error_power_off;
        }

        /* wait for 1 Sec */
        delay(1000);


        /* Toggle Blue LED */
        ret1 = gpioDrv4->SetValue(LED0_B, GPIO_PIN_OUTPUT_STATE_HIGH);
        if (ret1 != ARM_DRIVER_OK) {
            printf("ERROR: Failed to toggle LEDs LED0_B\n");
            goto error_power_off;
        }

        /* wait for 1 Sec */
        delay(1000);

        ret1 = gpioDrv4->SetValue(LED0_B, GPIO_PIN_OUTPUT_STATE_LOW);
        if (ret1 != ARM_DRIVER_OK) {
            printf("ERROR: Failed to toggle LEDs LED0_B\n");
            goto error_power_off;
        }

        /* wait for 1 Sec */
        delay(1000);
    }

error_power_off:

    ret1 = gpioDrv4->PowerControl(LED0_R, ARM_POWER_OFF);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off LED0_R\n");
    }
    ret1 = gpioDrv4->PowerControl(LED0_G, ARM_POWER_OFF);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off LED0_G\n");
    }
    ret1 = gpioDrv4->PowerControl(LED0_B, ARM_POWER_OFF);
    if (ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to power off LED0_B\n");
    }

error_uninitialize:

    ret1 = gpioDrv4->Uninitialize(LED0_R);
    if (ret1 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize LED0_R\n");
    }
    ret1 = gpioDrv4->Uninitialize(LED0_G);
    if (ret1 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize LED0_G\n");
    }
    ret1 = gpioDrv4->Uninitialize(LED0_B);
    if (ret1 != ARM_DRIVER_OK) {
        printf("Failed to Un-initialize LED0_B\n");
    }
}

/* Define main entry point.  */
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
    /* Configure Systick for each millisec */
    SysTick_Config(SystemCoreClock/1000);

    led_blink_app();
    return 0;
}

