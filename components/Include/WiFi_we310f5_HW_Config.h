/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     WiFi_we310f5_HW_Config.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     03-Feb-2024
 * @brief    Hardware configuration
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef _WIFI_WE310F5_HW_H
#define _WIFI_WE310F5_HW_H

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h> WE310F5 WiFi Driver Configuration

// <o> WiFi Driver Number (Driver_WiFi#) <0-1>
// <i> Defines exported WiFi driver control block number (Driver_WiFi#)
// <i> Default: 0
#define WIFI_WE310F5_DRIVER_NUMBER              0

// <o> Wi-Fi hardware connected via Driver_SPI#
//     <0=>  SPI0
//     <1=>  SPI1
//     <2=>  SPI2
//     <3=>  SPI3
// <i> Defines the SPI driver control block number (Driver_SPI#)
// <i> Select SPI
// <i> Default: 3
#define WIFI_SPI_INSTANCE                       3

// <o> Wi-Fi SPI Signal GPIO Driver_GPIO#
//     <0=>GPIO0
//     <1=>GPIO1
//     <2=>GPIO2
//     <3=>GPIO3
//     <4=>GPIO4
//     <5=>GPIO5
//     <6=>GPIO6
//     <7=>GPIO7
//     <8=>GPIO8
//     <9=>GPIO9
//     <10=>GPIO10
//     <11=>GPIO11
//     <12=>GPIO12
//     <13=>GPIO13
//     <14=>GPIO14
// <i> Default: 13
// <i> Defines the mapped GPIO for Wi-Fi Enable Signal (Driver_GPIO#)
// <i> Select GPIO
#define WIFI_SPI_GPIO_PORT_NUM                  12

// <o>Wi-Fi SPI Chip Select (Interface Pins) PIN#
//     <0=>PIN0
//     <1=>PIN1
//     <2=>PIN2
//     <3=>PIN3
//     <4=>PIN4
//     <5=>PIN5
//     <6=>PIN6
//     <7=>PIN7
// <i>Defines the mapped pins for Wi-Fi Interface (PIN#)
// <i> Select PINS
// <i> Default: 7
#define WIFI_SPI_CS_GPIO_PIN_NUM                7

// <o> Wi-Fi Enable Signal Driver_GPIO#
//     <0=>GPIO0
//     <1=>GPIO1
//     <2=>GPIO2
//     <3=>GPIO3
//     <4=>GPIO4
//     <5=>GPIO5
//     <6=>GPIO6
//     <7=>GPIO7
//     <8=>GPIO8
//     <9=>GPIO9
//     <10=>GPIO10
//     <11=>GPIO11
//     <12=>GPIO12
//     <13=>GPIO13
//     <14=>GPIO14
// <i> Default: 13
// <i> Defines the mapped GPIO for Wi-Fi Enable Signal (Driver_GPIO#)
// <i> Select GPIO
#define WIFI_ENABLE_GPIO_PORT_NUM               13

// <o>Wi-Fi Enable (Interface Pins) PIN#
//     <0=>PIN0
//     <1=>PIN1
//     <2=>PIN2
//     <3=>PIN3
//     <4=>PIN4
//     <5=>PIN5
//     <6=>PIN6
//     <7=>PIN7
// <i>Defines the mapped pins for Wi-Fi Interface (PIN#)
// <i> Select PINS
// <i> Default: 0
#define WIFI_ENABLE_GPIO_PIN_NUM                0

// <o> Wi-Fi Wake-up Signal Driver_GPIO#
//     <0=>GPIO0
//     <1=>GPIO1
//     <2=>GPIO2
//     <3=>GPIO3
//     <4=>GPIO4
//     <5=>GPIO5
//     <6=>GPIO6
//     <7=>GPIO7
//     <8=>GPIO8
//     <9=>GPIO9
//     <10=>GPIO10
//     <11=>GPIO11
//     <12=>GPIO12
//     <13=>GPIO13
//     <14=>GPIO14
// <i> Default: 13
// <i> Defines the mapped GPIO for Wi-Fi Wake-up Signal (Driver_GPIO#)
// <i> Select GPIO
#define WIFI_WAKEUP_GPIO_PORT_NUM               13

// <o>Wi-Fi Wake-up (Interface Pins) PIN#
//     <0=>PIN0
//     <1=>PIN1
//     <2=>PIN2
//     <3=>PIN3
//     <4=>PIN4
//     <5=>PIN5
//     <6=>PIN6
//     <7=>PIN7
// <i>Defines the mapped pins for Wi-Fi Interface (PIN#)
// <i> Select PINS
// <i> Default: 1
#define WIFI_WAKEUP_GPIO_PIN_NUM                1

// <o> Wi-Fi Reset Signal Driver_GPIO#
//     <0=>GPIO0
//     <1=>GPIO1
//     <2=>GPIO2
//     <3=>GPIO3
//     <4=>GPIO4
//     <5=>GPIO5
//     <6=>GPIO6
//     <7=>GPIO7
//     <8=>GPIO8
//     <9=>GPIO9
//     <10=>GPIO10
//     <11=>GPIO11
//     <12=>GPIO12
//     <13=>GPIO13
//     <14=>GPIO14
// <i> Default: 13
// <i> Defines the mapped GPIO for Wi-Fi Reset Signal (Driver_GPIO#)
// <i> Select GPIO
#define WIFI_RESET_GPIO_PORT_NUM                13

// <o>Wi-Fi Reset (Interface Pins) PIN#
//     <0=>PIN0
//     <1=>PIN1
//     <2=>PIN2
//     <3=>PIN3
//     <4=>PIN4
//     <5=>PIN5
//     <6=>PIN6
//     <7=>PIN7
// <i>Defines the mapped pins for Wi-Fi Interface (PIN#)
// <i> Select PINS
// <i> Default: 2
#define WIFI_RESET_GPIO_PIN_NUM                 2

// <o> Wi-Fi Interrupt Signal Driver_GPIO#
//     <0=>GPIO0
//     <1=>GPIO1
//     <2=>GPIO2
//     <3=>GPIO3
//     <4=>GPIO4
//     <5=>GPIO5
//     <6=>GPIO6
//     <7=>GPIO7
//     <8=>GPIO8
//     <9=>GPIO9
//     <10=>GPIO10
//     <11=>GPIO11
//     <12=>GPIO12
//     <13=>GPIO13
//     <14=>GPIO14
// <i> Default: 13
// <i> Defines the mapped GPIO for Wi-Fi Interrupt Signal (Driver_GPIO#)
// <i> Select GPIO
#define WIFI_INTR_GPIO_PORT_NUM               13

// <o>Wi-Fi Interrupt (Interface Pins) PIN#
//     <0=>PIN0
//     <1=>PIN1
//     <2=>PIN2
//     <3=>PIN3
//     <4=>PIN4
//     <5=>PIN5
//     <6=>PIN6
//     <7=>PIN7
// <i>Defines the mapped pins for Wi-Fi Interface (PIN#)
// <i> Select PINS
// <i> Default: 3
#define WIFI_INTR_GPIO_PIN_NUM                  3

// <o>AT Command Buffer Size (Transmit) in [bytes] <0-4096:4>
// <i> Default: 256
#define AT_TX_BUFF_SIZE                         256

// <o>AT Command Buffer Size (Receive) in [bytes] <0-4096:4>
// <i> Default: 512
#define AT_RX_BUFF_SIZE                         512

// <o>Delay to receive read banner in [milli sec] <1-1000>
// <i> Default: 50
#define DELAY_BETWEEN_READ_RESP                 50

// <o>SPI Speed
// <i> Default: 1000000
#define SPI_BAUD_RATE                           1000000

// </h>

//--------------- <<< end of configuration section >>> -------------------------

#endif  /* _WIFI_WE310F5_HW_H */
