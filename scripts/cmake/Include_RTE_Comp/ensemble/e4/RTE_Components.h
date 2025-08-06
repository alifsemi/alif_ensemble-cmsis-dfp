
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

/* AlifSemiconductor::Device.SOC Peripherals.ADC */
#define RTE_Drivers_ADC                     1 /* Driver ADC                               */
/* AlifSemiconductor::Device.SOC Peripherals.CANFD */
#define RTE_Drivers_CANFD                   1 /* Driver CANFD                             */
/* AlifSemiconductor::Device.SOC Peripherals.CDC200 */
#define RTE_Drivers_CDC200                  1 /* Driver CDC200                            */
/* AlifSemiconductor::BSP.External peripherals.ILI9806E LCD panel  */
#define RTE_Drivers_MIPI_DSI_ILI9806E_PANEL 1 /* Driver ILI9806E LCD panel                */
/* AlifSemiconductor::BSP.External peripherals.ILI6122 LCD panel   */
#define RTE_Drivers_CDC_ILI6122_PANEL       1 /* Driver ILI6122  LCD panel                */
/* AlifSemiconductor::BSP.External peripherals.ILI6122 LCD panel   */
#define RTE_Drivers_MIPI_DSI_ILI9488_PANEL  1 /* Driver ILI9488  LCD panel                */
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
#define RTE_Drivers_ISSI_FLASH              1 /* Driver Flash ISSI                        */
/* AlifSemiconductor::CMSIS Driver.Touchscreen.GT911 */
#define RTE_Drivers_GT911                   1 /* Driver GT911                             */
/* AlifSemiconductor::Device.SOC Peripherals.HWSEM */
#define RTE_Drivers_HWSEM                   1 /* Driver HWSEM                             */
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
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.WDT */
#define RTE_Drivers_WDT                     1 /* Driver WDT                               */
/* AlifSemiconductor::Device.SOC Peripherals.WiFi */
#define RTE_Drivers_WIFI                    0 /* Driver WiFi                              */
/* AlifSemiconductor::Device.SOC Peripherals.DMA */
#define RTE_Drivers_DMA                     1 /* Driver DMA                               */
/* AlifSemiconductor::Device.SOC Peripherals.I2C */
#define RTE_Drivers_I2C                     1 /* Driver I2C                               */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.CAMERA Sensor MT9M114 */
#define RTE_Drivers_CAMERA_SENSOR_MT9M114   1 /* Driver CAMREA Sensor                     */
/* AlifSemiconductor::CMSIS Driver.SOC Peripherals.CAMERA Sensor HM0360  */
#define RTE_Drivers_CAMERA_SENSOR_HM0360    1 /* Driver CAMREA Sensor                     */
/* AlifSemiconductor::BSP.External peripherals.CAMERA Sensor ARX3A0 */
#define RTE_Drivers_CAMERA_SENSOR_ARX3A0    1 /* Driver CAMERA Sensor                     */
/* AlifSemiconductor::BSP.External peripherals.CAMERA Sensor AR0144 */
#define RTE_Drivers_CAMERA_SENSOR_AR0144    1 /* Driver CAMERA Sensor                     */
/* AlifSemiconductor::BSP.External peripherals.CAMERA Sensor AR0145 */
#define RTE_Drivers_CAMERA_SENSOR_AR0145    1 /* Driver CAMERA Sensor                     */
/* AlifSemiconductor::BSP.External peripherals.CAMERA Sensor AR0246 */
#define RTE_Drivers_CAMERA_SENSOR_AR0246    1 /* Driver CAMERA Sensor                     */
/* AlifSemiconductor::BSP.External peripherals.CAMERA Sensor OV5647 */
#define RTE_Drivers_CAMERA_SENSOR_OV5647    1 /* Driver CAMERA Sensor                     */
/* AlifSemiconductor::BSP.External peripherals.CAMERA Sensor OV5675 */
#define RTE_Drivers_CAMERA_SENSOR_OV5675    1 /* Driver CAMERA Sensor                     */
/* AlifSemiconductor::Device.SOC Peripherals.MIPI CSI2 */
#define RTE_Drivers_MIPI_CSI2               1 /* Driver MIPI CSI2                         */
/* AlifSemiconductor::Device.SOC Peripherals.MIPI DSI */
#define RTE_Drivers_MIPI_DSI                1 /* Driver MIPI DSI                          */
/* AlifSemiconductor::Device.SOC Peripherals MIPI_DSI_CSI2_DPHY */
#define RTE_Drivers_MIPI_DSI_CSI2_DPHY      1 /* Driver MIPI DSI CSI2 DHY                 */
/* AlifSemiconductor::BSP.External peripherals Ethernet PHY */
#define RTE_Drivers_PHY_GENERIC             1 /* Driver Phy generic                       */
/* AlifSemiconductor::BSP.External peripherals Ethernet MAC */
#define RTE_Drivers_ETH_MAC                 1 /* Driver Ethernet MAC                      */
/* AlifSemiconductor::Device.SOC Peripherals.CPI */
#define RTE_Drivers_CPI                     1 /* Driver CPI                               */
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
