/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     retarget_config.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     16-Aug-2023
 * @brief    Disable Semihosting
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef RETARGET_STD_IN_OUT_CONFIG_H__
#define RETARGET_STD_IN_OUT_CONFIG_H__

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifdef  __cplusplus
extern "C"
{
#endif

// <e> Console UART
// <i> Console UART for logs

// <o> Maximum Outstanding Line-fills <0-7;LP>
// <i> Select UART Console
// <i> Default: 4
#define PRINTF_UART_CONSOLE                 4

// <o> Baudrate for Logs Line-fills <110;300;600;1200;2400;4800;9600;14400;19200;38400;57600;115200;230400;460800;921600>
// <i> Select UART Console
// <i> Default: 115200
#define PRINTF_UART_CONSOLE_BAUD_RATE       115200

/* UART Related Macros */
#define _UART_BASE_(n)                                          UART##n##_BASE
#define UART_BASE(n)                                            _UART_BASE_(n)

#if (PRINTF_UART_CONSOLE == LP)     /* Instance A */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_7
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_6
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_7
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 0)    /* Instance A */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_0
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_0
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_1
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 1)    /* Instance A */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_0
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_4
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_5
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 2)    /* Instance A */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_1
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_0
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_1
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_1
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_1
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 3)    /* Instance B */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_2
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_2
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_3
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 4)    /* Instance B */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_12
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_1
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_2
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 5)    /* Instance A */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_3
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_4
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_5
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 6)    /* Instance A */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_10
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_5
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_6
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_1
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_1
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#elif (PRINTF_UART_CONSOLE == 7)    /* Instance B */
    #define PRINTF_UART_CONSOLE_PORT_NUM                        PORT_9
    #define PRINTF_UART_CONSOLE_RX_PIN                          PIN_3
    #define PRINTF_UART_CONSOLE_TX_PIN                          PIN_4
    #define PRINTF_UART_CONSOLE_RX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_2
    #define PRINTF_UART_CONSOLE_TX_PINMUX_FUNCTION              PINMUX_ALTERNATE_FUNCTION_1
    #define PRINTF_UART_CONSOLE_RX_PADCTRL                      PADCTRL_READ_ENABLE
    #define PRINTF_UART_CONSOLE_TX_PADCTRL                      0
#endif /* PRINTF_UART_CONSOLE */

#ifdef  __cplusplus
}
#endif

// </e> Console UART

#endif /* RETARGET_STD_IN_OUT_H__ */