
/*
 * RTE_Component.h header file, for both M55_HP and M55_HE devices of Ensemble pack.
 * Update manually the macro definitions as per the requirement.
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

/*
 * Define the Device Header File:
 */
#define CMSIS_device_header                 "alif.h"

/* AlifSemiconductor::CMSIS Driver.CMP */
#define RTE_Drivers_MHU                     1 /* Driver MHU                               */

/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.PINCONF */
#define RTE_Drivers_LL_PINCONF              1 /* Driver PinPAD and PinMux                 */
/* AlifSemiconductor::CMSIS Driver.USART */
#define RTE_Drivers_USART                   1 /* Driver UART                              */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.IO */
#define RTE_Drivers_IO                      1 /* Driver IO                                */

/* AlifSemiconductor::Device.SOC Peripherals.CANFD */
#define RTE_Drivers_CANFD                   1 /* Driver CANFD                             */
/* AlifSemiconductor::CMSIS Driver.CMP */
#define RTE_Drivers_CMP                     1 /* Driver CMP                               */
/* AlifSemiconductor::Device.SOC Peripherals.CRC */
#define RTE_Drivers_CRC                     1 /* Driver CRC                               */
/* AlifSemiconductor::Device.SOC Peripherals.DAC */
#define RTE_Drivers_DAC                     1 /* Driver ADC                               */
/* AlifSemiconductor::SD Driver */
#define RTE_Drivers_SD                      1 /* Driver SD                                */
/* AlifSemiconductor::Device.SOC Peripherals.OSPI Controller */
#define RTE_Drivers_OSPI                    1 /* Driver OSPI                              */
/* AlifSemiconductor::BSP.OSPI Hyper RAM XIP     */
#define RTE_Drivers_XIP_HYPERRAM            1 /* Driver Hyper RAM XIP                     */
/* AlifSemiconductor::BSP.OSPI FLASH XIP Core   */
#define RTE_Drivers_ISSI_FLASH_XIP_CORE     1 /* Driver OSPI XIP CORE                     */
/* AlifSemiconductor::BSP.OSPI FLASH XIP Utility  */
#define RTE_Drivers_ISSI_FLASH_XIP_UTILITY  1 /* Driver OSPI XIP UTILITY                  */
/* AlifSemiconductor::BSP.External peripherals.OSPI Flash ISSI     */
#define RTE_Drivers_ISSI_FLASH   1 /* Driver Flash ISSI                        */
/* AlifSemiconductor::CMSIS Driver.SAI.I2S */
#define RTE_Drivers_SAI                     1 /* Driver SAI                               */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.I3C */
#define RTE_Drivers_I3C                     1 /* Driver I3C                               */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.I3C_I2C */
#define RTE_Drivers_I2C_I3C                 1 /* Driver I3C-I2C                           */
/* AlifSemiconductor::Device.SOC Peripherals.ICM42670P */
#define RTE_Drivers_ICM42670P               1 /* Driver ICM42670P                         */
/* AlifSemiconductor::Device.SOC Peripherals.LPI2C */
#define RTE_Drivers_LPI2C                   1 /* Driver Low Power I2C                     */
/* AlifSemiconductor::Device.SOC Peripherals.PDM */
#define RTE_Drivers_PDM                     1 /* Driver PDM                               */
/* AlifSemiconductor::CMSIS Driver.SPI.SPI */
#define RTE_Drivers_SPI                     1 /* Driver SPI                               */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.LPTIMER */
#define RTE_Drivers_LPTIMER                 1 /* Driver LPTIMER                           */
/* AlifSemiconductor::Device.SOC Peripherals.MRAM Flash */
#define RTE_Drivers_MRAM                    1 /* Driver MRAM FLASH                        */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.RTC */
#define RTE_Drivers_RTC                     1 /* Driver RTC                               */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.UTIMER */
#define RTE_Drivers_UTIMER                  1 /* Driver UTIMER                            */
/* AlifSemiconductor::Device.SOC Peripherals.I2C */
#define RTE_Drivers_I2C                     1 /* Driver I2C                               */
/* AlifSemiconductor::Device.SOC Peripherals.MIPI DSI */
#define RTE_Drivers_MIPI_DSI                1 /* Driver MIPI DSI                          */
/* AlifSemiconductor::BSP.External peripherals Ethernet PHY */
#define RTE_Drivers_PHY_GENERIC             1 /* Driver Phy generic                       */
/* AlifSemiconductor::BSP.External peripherals Ethernet MAC */
#define RTE_Drivers_ETH_MAC                 1 /* Driver Ethernet MAC                      */
/* AlifSemiconductor::CMSIS Driver.GPIO */
#define RTE_Drivers_GPIO                    1 /* Driver GPIO                              */
/* AlifSemiconductor::Device.SOC Peripherals.WM8904 */
#define RTE_Drivers_WM8904_CODEC            1 /* Driver WM8904                            */
/* AlifSemiconductor::BSP.External peripherals.BMI323 */
#define RTE_Drivers_BMI323                  1 /* Driver BMI323                            */
/* AlifSemiconductor::CMSIS Driver.MCI                */
#define RTE_Drivers_MCI                     1 /* Driver MCI                               */
/* AlifSemiconductor::Device.SOC STDIN Retargetting */
//#define RTE_CMSIS_Compiler_STDIN            /* Enable/Disable Re targetting for STDIN   */
/* AlifSemiconductor::Device.SOC STDOUT Retargetting */
//#define RTE_CMSIS_Compiler_STDOUT           /* Enable/Disable Re targetting for STDOUT  */
/* AlifSemiconductor::Device.SOC STDERR Retargetting */
//#define RTE_CMSIS_Compiler_STDERR           /* Enable/Disable Re targetting for STDERR  */
#endif /* RTE_COMPONENTS_H */
