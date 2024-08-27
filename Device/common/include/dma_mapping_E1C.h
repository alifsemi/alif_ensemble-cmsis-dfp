/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DMA_MAPPING_E1C_H
#define DMA_MAPPING_E1C_H

#include "RTE_Device.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                         DMA2 peripheral map                              */
/******************************************************************************/

/************************* DMA2 Group 0 Mapping *******************************/
#define CMP0_DMA                       2
#define CMP0_DMA_PERIPH_REQ            0
#define CMP0_DMA_GROUP                 0
#define CMP0_DMA_HANDSHAKE_ENABLE      0

#define CMP1_DMA                       2
#define CMP1_DMA_PERIPH_REQ            1
#define CMP1_DMA_GROUP                 0
#define CMP1_DMA_HANDSHAKE_ENABLE      0

#define AHI_UART_DMA                   2
#define AHI_UART_DMA_RX_PERIPH_REQ     2
#define AHI_UART_DMA_TX_PERIPH_REQ     3
#define AHI_UART_DMA_GROUP             0
#define AHI_UART_DMA_HANDSHAKE_ENABLE  1

#define ADC120_DMA                     2
#define ADC120_DMA_DONE1_PERIPH_REQ    4
#define ADC120_DMA_GROUP               0
#define ADC120_DMA_HANDSHAKE_ENABLE    0

#define ADC121_DMA                     2
#define ADC121_DMA_DONE1_PERIPH_REQ    5
#define ADC121_DMA_GROUP               0
#define ADC121_DMA_HANDSHAKE_ENABLE    0

#define LPCMP_DMA                      2
#define LPCMP_DMA_PERIPH_REQ           6
#define LPCMP_DMA_GROUP                0
#define LPCMP_DMA_HANDSHAKE_ENABLE     0

#define BOD_DMA                        2
#define BOD_DMA_PERIPH_REQ             7
#define BOD_DMA_GROUP                  0
#define BOD_DMA_HANDSHAKE_ENABLE       0

#define UART0_DMA                      2
#define UART0_DMA_RX_PERIPH_REQ        8
#define UART0_DMA_TX_PERIPH_REQ        16
#define UART0_DMA_GROUP                0
#define UART0_DMA_HANDSHAKE_ENABLE     1

#define UART1_DMA                      2
#define UART1_DMA_RX_PERIPH_REQ        9
#define UART1_DMA_TX_PERIPH_REQ        17
#define UART1_DMA_GROUP                0
#define UART1_DMA_HANDSHAKE_ENABLE     1

#define UART2_DMA                      2
#define UART2_DMA_RX_PERIPH_REQ        10
#define UART2_DMA_TX_PERIPH_REQ        18
#define UART2_DMA_GROUP                0
#define UART2_DMA_HANDSHAKE_ENABLE     1

#define UART3_DMA                      2
#define UART3_DMA_RX_PERIPH_REQ        11
#define UART3_DMA_TX_PERIPH_REQ        19
#define UART3_DMA_GROUP                0
#define UART3_DMA_HANDSHAKE_ENABLE     1

#define UART4_DMA                      2
#define UART4_DMA_RX_PERIPH_REQ        12
#define UART4_DMA_TX_PERIPH_REQ        20
#define UART4_DMA_GROUP                0
#define UART4_DMA_HANDSHAKE_ENABLE     1

#define UART5_DMA                      2
#define UART5_DMA_RX_PERIPH_REQ        13
#define UART5_DMA_TX_PERIPH_REQ        21
#define UART5_DMA_GROUP                0
#define UART5_DMA_HANDSHAKE_ENABLE     1

#define I2C0_DMA                       2
#define I2C0_DMA_RX_PERIPH_REQ         24
#define I2C0_DMA_TX_PERIPH_REQ         28
#define I2C0_DMA_GROUP                 0
#define I2C0_DMA_HANDSHAKE_ENABLE      1

#define I2C1_DMA                       2
#define I2C1_DMA_RX_PERIPH_REQ         25
#define I2C1_DMA_TX_PERIPH_REQ         29
#define I2C1_DMA_GROUP                 0
#define I2C1_DMA_HANDSHAKE_ENABLE      1

#define HCI_UART_DMA                   2
#define HCI_UART_DMA_RX_PERIPH_REQ     26
#define HCI_UART_DMA_TX_PERIPH_REQ     27
#define HCI_UART_DMA_GROUP             0
#define HCI_UART_DMA_HANDSHAKE_ENABLE  1


#define LPGPIO_DMA                     2
#define LPGPIO_DMA_PIN0_PERIPH_REQ     30
#define LPGPIO_DMA_PIN1_PERIPH_REQ     31
#define LPGPIO_DMA_GROUP               0
#define LPGPIO_DMA_HANDSHAKE_ENABLE    0


/************************* DMA2 Group 1 Mapping *******************************/
#define LPTIMER0_DMA                   2
#define LPTIMER0_DMA_PERIPH_REQ        0
#define LPTIMER0_DMA_GROUP             1
#define LPTIMER0_DMA_HANDSHAKE_ENABLE  0

#define LPTIMER1_DMA                   2
#define LPTIMER1_DMA_PERIPH_REQ        1
#define LPTIMER1_DMA_GROUP             1
#define LPTIMER1_DMA_HANDSHAKE_ENABLE  0

#define CANFD1_DMA                     2
#define CANFD1_DMA_RX_PERIPH_REQ       4
#define CANFD1_DMA_TX_PERIPH_REQ       5
#define CANFD1_DMA_GROUP               1
#define CANFD1_DMA_HANDSHAKE_ENABLE    0

#define LPI2S_DMA                      2
#define LPI2S_DMA_WS_PERIPH_REQ        6
#define LPI2S_DMA_GROUP                1
#define LPI2S_DMA_HANDSHAKE_ENABLE     1

#define BLE_ISO_SDU_REF_DMA            2
#define BLE_ISO_SDU_REF_DMA_PERIPH_REQ 7
#define BLE_ISO_SDU_REF_DMA_GROUP      1
#define BLE_ISO_SDU_REF_DMA_HANDSHAKE_ENABLE 0

#define LPUART_DMA                     2
#define LPUART_DMA_RX_PERIPH_REQ       8
#define LPUART_DMA_TX_PERIPH_REQ       9
#define LPUART_DMA_GROUP               1
#define LPUART_DMA_HANDSHAKE_ENABLE    1

#define ADC24_DMA                      2
#define ADC24_DMA_DONE0_PERIPH_REQ     10
#define ADC24_DMA_DONE1_PERIPH_REQ     11
#define ADC24_DMA_GROUP                1
#define ADC24_DMA_HANDSHAKE_ENABLE     0

#define LPSPI_DMA                      2
#define LPSPI_DMA_RX_PERIPH_REQ        12
#define LPSPI_DMA_TX_PERIPH_REQ        13
#define LPSPI_DMA_GROUP                1
#define LPSPI_DMA_HANDSHAKE_ENABLE     1

#define LPI2S_DMA                      2
#define LPI2S_DMA_RX_PERIPH_REQ        14
#define LPI2S_DMA_TX_PERIPH_REQ        15
#define LPI2S_DMA_GROUP                1
#define LPI2S_DMA_HANDSHAKE_ENABLE     1

#define OSPI0_DMA                      2
#define OSPI0_DMA_RX_PERIPH_REQ        16
#define OSPI0_DMA_TX_PERIPH_REQ        18
#define OSPI0_DMA_GROUP                1
#define OSPI0_DMA_HANDSHAKE_ENABLE     1

#define I3C_DMA                        2
#define I3C_DMA_RX_PERIPH_REQ          20
#define I3C_DMA_TX_PERIPH_REQ          21
#define I3C_DMA_GROUP                  1
#define I3C_DMA_HANDSHAKE_ENABLE       1

#define CANFD0_DMA                     2
#define CANFD0_DMA_RX_PERIPH_REQ       22
#define CANFD0_DMA_TX_PERIPH_REQ       23
#define CANFD0_DMA_GROUP               1
#define CANFD0_DMA_HANDSHAKE_ENABLE    0

#define I2S0_DMA                       2
#define I2S0_DMA_WS_PERIPH_REQ         2
#define I2S0_DMA_RX_PERIPH_REQ         24
#define I2S0_DMA_TX_PERIPH_REQ         28
#define I2S0_DMA_GROUP                 1
#define I2S0_DMA_HANDSHAKE_ENABLE      1
#define I2S0_DMA_WS_HANDSHAKE_ENABLE   0

#define I2S1_DMA                       2
#define I2S1_DMA_WS_PERIPH_REQ         3
#define I2S1_DMA_RX_PERIPH_REQ         25
#define I2S1_DMA_TX_PERIPH_REQ         29
#define I2S1_DMA_GROUP                 1
#define I2S1_DMA_HANDSHAKE_ENABLE      1
#define I2S0_DMA_WS_HANDSHAKE_ENABLE   0

/************************* DMA2 Group 2 Mapping *******************************/
#define UT0_DMA                        2
#define UT0_DMA_T0_PERIPH_REQ          0
#define UT0_DMA_T1_PERIPH_REQ          1
#define UT0_DMA_GROUP                  2
#define UT0_DMA_HANDSHAKE_ENABLE       0

#define UT1_DMA                        2
#define UT1_DMA_T0_PERIPH_REQ          2
#define UT1_DMA_T1_PERIPH_REQ          3
#define UT1_DMA_GROUP                  2
#define UT1_DMA_HANDSHAKE_ENABLE       0

#define UT2_DMA                        2
#define UT2_DMA_T0_PERIPH_REQ          4
#define UT2_DMA_T1_PERIPH_REQ          5
#define UT2_DMA_GROUP                  2
#define UT2_DMA_HANDSHAKE_ENABLE       0

#define UT3_DMA                        2
#define UT3_DMA_T0_PERIPH_REQ          6
#define UT3_DMA_T1_PERIPH_REQ          7
#define UT3_DMA_GROUP                  2
#define UT3_DMA_HANDSHAKE_ENABLE       0

#define SPI0_DMA                       2
#define SPI0_DMA_RX_PERIPH_REQ         16
#define SPI0_DMA_TX_PERIPH_REQ         20
#define SPI0_DMA_GROUP                 2
#define SPI0_DMA_HANDSHAKE_ENABLE      1

#define SPI1_DMA                       2
#define SPI1_DMA_RX_PERIPH_REQ         17
#define SPI1_DMA_TX_PERIPH_REQ         21
#define SPI1_DMA_GROUP                 2
#define SPI1_DMA_HANDSHAKE_ENABLE      1

#define SPI2_DMA                       2
#define SPI2_DMA_RX_PERIPH_REQ         18
#define SPI2_DMA_TX_PERIPH_REQ         22
#define SPI2_DMA_GROUP                 2
#define SPI2_DMA_HANDSHAKE_ENABLE      1

#if (RTE_LPSPI_SELECT_DMA2_GROUP == 2)
#define LPSPI_DMA                      2
#define LPSPI_DMA_RX_PERIPH_REQ        24
#define LPSPI_DMA_TX_PERIPH_REQ        25
#define LPSPI_DMA_GROUP                2
#define LPSPI_DMA_HANDSHAKE_ENABLE     1
#endif

#define LPCAM_DMA                      2
#define LPCAM_DMA_VSYNC_PERIPH_REQ     26
#define LPCAM_DMA_HSYNC_PERIPH_REQ     27
#define LPCAM_DMA_GROUP                2
#define LPCAM_DMA_HANDSHAKE_ENABLE     0

#define CDC_DMA                        2
#define CDC_DMA_VSYNC_PERIPH_REQ       28
#define CDC_DMA_HSYNC_PERIPH_REQ       29
#define CDC_DMA_GROUP                  2
#define CDC_DMA_HANDSHAKE_ENABLE       0

#define LPPDM_DMA                      2
#define LPPDM_DMA_PERIPH_REQ           30
#define LPPDM_DMA_GROUP                2
#define LPPDM_DMA_HANDSHAKE_ENABLE     1

/************************* DMA2 Group 3 Mapping *******************************/
#define GPIO3_DMA                      2
#define GPIO3_DMA_PIN0_PERIPH_REQ      0
#define GPIO3_DMA_PIN1_PERIPH_REQ      1
#define GPIO3_DMA_PIN2_PERIPH_REQ      2
#define GPIO3_DMA_PIN3_PERIPH_REQ      3
#define GPIO3_DMA_PIN4_PERIPH_REQ      4
#define GPIO3_DMA_PIN5_PERIPH_REQ      5
#define GPIO3_DMA_PIN6_PERIPH_REQ      6
#define GPIO3_DMA_PIN7_PERIPH_REQ      7
#define GPIO3_DMA_GROUP                3
#define GPIO3_DMA_HANDSHAKE_ENABLE     0

#define GPIO4_DMA                      2
#define GPIO4_DMA_PIN0_PERIPH_REQ      8
#define GPIO4_DMA_PIN1_PERIPH_REQ      9
#define GPIO4_DMA_PIN2_PERIPH_REQ      10
#define GPIO4_DMA_PIN3_PERIPH_REQ      11
#define GPIO4_DMA_PIN4_PERIPH_REQ      12
#define GPIO4_DMA_PIN5_PERIPH_REQ      13
#define GPIO4_DMA_PIN6_PERIPH_REQ      14
#define GPIO4_DMA_PIN7_PERIPH_REQ      15
#define GPIO4_DMA_GROUP                3
#define GPIO4_DMA_HANDSHAKE_ENABLE     0

#define GPIO7_DMA                      2
#define GPIO7_DMA_PIN0_PERIPH_REQ      16
#define GPIO7_DMA_PIN1_PERIPH_REQ      17
#define GPIO7_DMA_PIN2_PERIPH_REQ      18
#define GPIO7_DMA_PIN3_PERIPH_REQ      19
#define GPIO7_DMA_PIN4_PERIPH_REQ      20
#define GPIO7_DMA_PIN5_PERIPH_REQ      21
#define GPIO7_DMA_PIN6_PERIPH_REQ      22
#define GPIO7_DMA_PIN7_PERIPH_REQ      23
#define GPIO7_DMA_GROUP                3
#define GPIO7_DMA_HANDSHAKE_ENABLE     0

#define GPIO1_DMA                      2
#define GPIO1_DMA_PIN0_PERIPH_REQ      24
#define GPIO1_DMA_PIN1_PERIPH_REQ      25
#define GPIO1_DMA_PIN2_PERIPH_REQ      26
#define GPIO1_DMA_PIN3_PERIPH_REQ      27
#define GPIO1_DMA_PIN4_PERIPH_REQ      28
#define GPIO1_DMA_PIN5_PERIPH_REQ      29
#define GPIO1_DMA_PIN6_PERIPH_REQ      30
#define GPIO1_DMA_PIN7_PERIPH_REQ      31
#define GPIO1_DMA_GROUP                3
#define GPIO1_DMA_HANDSHAKE_ENABLE     0

#if RTE_GPIO3
#define GPIO3_DMA_GLITCH_FILTER ((RTE_GPIO3_PIN0_DMA_GLITCH_FILTER_ENABLE << 0)|\
                                 (RTE_GPIO3_PIN1_DMA_GLITCH_FILTER_ENABLE << 1)|\
                                 (RTE_GPIO3_PIN2_DMA_GLITCH_FILTER_ENABLE << 2)|\
                                 (RTE_GPIO3_PIN3_DMA_GLITCH_FILTER_ENABLE << 3)|\
                                 (RTE_GPIO3_PIN4_DMA_GLITCH_FILTER_ENABLE << 4)|\
                                 (RTE_GPIO3_PIN5_DMA_GLITCH_FILTER_ENABLE << 5)|\
                                 (RTE_GPIO3_PIN6_DMA_GLITCH_FILTER_ENABLE << 6)|\
                                 (RTE_GPIO3_PIN7_DMA_GLITCH_FILTER_ENABLE << 7))
#else
#define GPIO3_DMA_GLITCH_FILTER 0
#endif

#if RTE_GPIO4
#define GPIO4_DMA_GLITCH_FILTER ((RTE_GPIO4_PIN0_DMA_GLITCH_FILTER_ENABLE << 8)|\
                                 (RTE_GPIO4_PIN1_DMA_GLITCH_FILTER_ENABLE << 9)|\
                                 (RTE_GPIO4_PIN2_DMA_GLITCH_FILTER_ENABLE << 10)|\
                                 (RTE_GPIO4_PIN3_DMA_GLITCH_FILTER_ENABLE << 11)|\
                                 (RTE_GPIO4_PIN4_DMA_GLITCH_FILTER_ENABLE << 12)|\
                                 (RTE_GPIO4_PIN5_DMA_GLITCH_FILTER_ENABLE << 13)|\
                                 (RTE_GPIO4_PIN6_DMA_GLITCH_FILTER_ENABLE << 14)|\
                                 (RTE_GPIO4_PIN7_DMA_GLITCH_FILTER_ENABLE << 15))
#else
#define GPIO4_DMA_GLITCH_FILTER 0
#endif

#if RTE_GPIO7
#define GPIO7_DMA_GLITCH_FILTER ((RTE_GPIO7_PIN0_DMA_GLITCH_FILTER_ENABLE << 16)|\
                                 (RTE_GPIO7_PIN1_DMA_GLITCH_FILTER_ENABLE << 17)|\
                                 (RTE_GPIO7_PIN2_DMA_GLITCH_FILTER_ENABLE << 18)|\
                                 (RTE_GPIO7_PIN3_DMA_GLITCH_FILTER_ENABLE << 19)|\
                                 (RTE_GPIO7_PIN4_DMA_GLITCH_FILTER_ENABLE << 20)|\
                                 (RTE_GPIO7_PIN5_DMA_GLITCH_FILTER_ENABLE << 21)|\
                                 (RTE_GPIO7_PIN6_DMA_GLITCH_FILTER_ENABLE << 22)|\
                                 (RTE_GPIO7_PIN7_DMA_GLITCH_FILTER_ENABLE << 23))
#else
#define GPIO7_DMA_GLITCH_FILTER 0
#endif

#if RTE_GPIO1
#define GPIO1_DMA_GLITCH_FILTER ((RTE_GPIO1_PIN0_DMA_GLITCH_FILTER_ENABLE << 24)|\
                                 (RTE_GPIO1_PIN1_DMA_GLITCH_FILTER_ENABLE << 25)|\
                                 (RTE_GPIO1_PIN2_DMA_GLITCH_FILTER_ENABLE << 26)|\
                                 (RTE_GPIO1_PIN3_DMA_GLITCH_FILTER_ENABLE << 27)|\
                                 (RTE_GPIO1_PIN4_DMA_GLITCH_FILTER_ENABLE << 28)|\
                                 (RTE_GPIO1_PIN5_DMA_GLITCH_FILTER_ENABLE << 29)|\
                                 (RTE_GPIO1_PIN6_DMA_GLITCH_FILTER_ENABLE << 30)|\
                                 (RTE_GPIO1_PIN7_DMA_GLITCH_FILTER_ENABLE << 31))
#else
#define GPIO1_DMA_GLITCH_FILTER 0
#endif

#if RTE_LPGPIO
#define LPGPIO_DMA_GLITCH_FILTER ((RTE_LPGPIO_PIN0_DMA_GLITCH_FILTER_ENABLE << 8)|\
                                  (RTE_LPGPIO_PIN1_DMA_GLITCH_FILTER_ENABLE << 9))
#else
#define LPGPIO_DMA_GLITCH_FILTER 0
#endif


#define DMA2_GLITCH_FILTER0  (GPIO3_DMA_GLITCH_FILTER | \
                              GPIO4_DMA_GLITCH_FILTER | \
                              GPIO7_DMA_GLITCH_FILTER | \
                              GPIO1_DMA_GLITCH_FILTER)
#define DMA2_GLITCH_FILTER1  (LPGPIO_DMA_GLITCH_FILTER)

#ifdef __cplusplus
}
#endif

#endif /* DMA_MAPPING_E1C_H */
