# Setting path variables for Alif CMSIS directory
set (DRIVERS_INC            "${ALIF_CMSIS_DRIVER_DIR}/Include;\
                             ${ALIF_CMSIS_DRIVER_DIR}/Source;\
                             ${ALIF_ENSEMBLE_DRIVERS_DIR}/include;\
                             ${ALIF_CMSIS_DRIVER_DIR}/Include/config;\
                             ${ALIF_COMPONENTS_DIR}/Include")

# Setting variables for OSPI drivers
set (OSPI_XIP_DIR                       "${ALIF_DEV_SRC_DIR}/ospi_xip")
set (OSPI_SRC_DIR                       "${OSPI_XIP_DIR}/source")
set (ALIF_CMSIS_DRIVER_SRC_DIR          "${ALIF_CMSIS_DRIVER_DIR}/Source")
set (ALIF_ENSEMBLE_DRIVERS_SRC_DIR      "${ALIF_ENSEMBLE_DRIVERS_DIR}/source")
set (ALIF_COMPONENTS_SRC_DIR            "${ALIF_COMPONENTS_DIR}/Source")

set (OSPI_INC
    ${OSPI_SRC_DIR}/issi_flash ${OSPI_SRC_DIR}/ospi
    ${OSPI_XIP_DIR}/include    ${OSPI_XIP_DIR}/config)

include_directories (${OSPI_INC})
include_directories (${DRIVERS_INC})

#To avoid misconfiguration
if(${ENABLE_MIPI_DSI_ILI9806E_PANEL} AND ${ENABLE_CDC_ILI6122E_PANEL} AND ${ENABLE_MIPI_DSI_ILI9488E_PANEL})
    message(STATUS        "${Blue}📢[INFO] ALL LCD ILI9806E ILI6122E ILI6122E IS ENABLED, Make sure only one should be enabled ${ColourReset}")
endif()

set(LP_DOMAIN_EN        OFF)
if(${RTSS} STREQUAL HE)
    set(LP_DOMAIN_EN        ON)
endif()

# Creating a Library file for Driver Source Files
set (DRIVER_LIB     "DRIVER")
add_library (${DRIVER_LIB} STATIC)

target_sources(${DRIVER_LIB} PRIVATE
    #MHU Driver
    $<$<BOOL:${ENABLE_MHU}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/mhu_driver.c>
    $<$<BOOL:${ENABLE_MHU}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/mhu_receiver.c>
    $<$<BOOL:${ENABLE_MHU}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/mhu_sender.c>

    #Pin Config
    $<$<BOOL:${ENABLE_PIN_CONF}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/pinconf.c>

    #UART Driver
    $<$<BOOL:${ENABLE_USART}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/uart.c>
    $<$<BOOL:${ENABLE_USART}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_USART.c>

    #IO Driver
    $<$<BOOL:${ENABLE_IO}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_IO.c>

    #ADC Driver
    $<$<BOOL:${ENABLE_ADC}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/adc.c>
    $<$<BOOL:${ENABLE_ADC}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_ADC.c>

    #CANFD Driver
    $<$<BOOL:${ENABLE_CANFD}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/canfd.c>
    $<$<BOOL:${ENABLE_CANFD}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_CAN.c>

    #CDC200 Driver
    $<$<BOOL:${ENABLE_CDC200}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/cdc.c>
    $<$<BOOL:${ENABLE_CDC200}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_CDC200.c>

    #MIPI DSI ILI9806E Panel Driver
    $<$<AND:$<BOOL:${ENABLE_CDC200}>,$<BOOL:${ENABLE_MIPI_DSI_ILI9806E_PANEL}>>:${ALIF_COMPONENTS_SRC_DIR}/ILI9806E_LCD_panel.c>
    $<$<AND:$<BOOL:${ENABLE_CDC200}>,$<BOOL:${ENABLE_CDC_ILI6122E_PANEL}>>:${ALIF_COMPONENTS_SRC_DIR}/ILI6122_LCD_panel.c>
    $<$<AND:$<BOOL:${ENABLE_CDC200}>,$<BOOL:${ENABLE_MIPI_DSI_ILI9488E_PANEL}>>:${ALIF_COMPONENTS_SRC_DIR}/ILI9488_LCD_panel.c>

    #CMP Driver
    $<$<BOOL:${ENABLE_CMP}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/cmp.c>
    $<$<BOOL:${ENABLE_CMP}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_CMP.c>

    #CRC Driver
    $<$<BOOL:${ENABLE_CRC}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/crc.c>
    $<$<BOOL:${ENABLE_CRC}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_CRC.c>

    #DAC Driver
    $<$<BOOL:${ENABLE_DAC}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_DAC.c>

    #SD Driver
    $<$<BOOL:${ENABLE_SD}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/sd.c>
    $<$<BOOL:${ENABLE_SD}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/sdio.c>
    $<$<BOOL:${ENABLE_SD}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/sd_host.c>

    #OSPI Driver
    $<$<BOOL:${ENABLE_OSPI}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/ospi.c>
    $<$<BOOL:${ENABLE_OSPI}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_OSPI.c>

    #Hyper RAM XIP Driver
    $<$<BOOL:${ENABLE_XIP_HYPERRAM}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/ospi_hyperram_xip.c>
    $<$<BOOL:${ENABLE_XIP_HYPERRAM}>:${ALIF_COMPONENTS_SRC_DIR}/S80K_HyperRAM.c>

    #ISSI Flash XIP Core
    $<$<BOOL:${ENABLE_ISSI_FLASH_XIP_CORE}>:${OSPI_SRC_DIR}/ospi/ospi_drv.c>
    $<$<BOOL:${ENABLE_ISSI_FLASH_XIP_CORE}>:${OSPI_SRC_DIR}/issi_flash/issi_flash.c>

    #ISSI Flash XIP Utility
    $<$<BOOL:${ENABLE_ISSI_FLASH_XIP_UTILITY}>:${OSPI_XIP_DIR}/app/app_issi_flash.c>

    #ISSI FLASH Driver ENABLE_ISSI_FLASH
    $<$<BOOL:${ENABLE_ISSI_FLASH}>:${ALIF_COMPONENTS_SRC_DIR}/IS25WX256.c>

    #GT911 Driver
    $<$<BOOL:${ENABLE_GT911}>:${ALIF_COMPONENTS_SRC_DIR}/GT911_touch_driver.c>

    #HWSEM Driver
    $<$<BOOL:${ENABLE_HWSEM}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_HWSEM.c>

    #I2S Driver
    $<$<BOOL:${ENABLE_I2S}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/i2s.c>
    $<$<BOOL:${ENABLE_I2S}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_I2S.c>

    #I3C Driver
    $<$<BOOL:${ENABLE_I3C}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/i3c.c>
    $<$<BOOL:${ENABLE_I3C}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_I3C.c>

    #I2C_I3C Driver
    $<$<BOOL:${ENABLE_I3C_I2C}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_I3C_I2C.c>

    #ICM42670P Driver
    $<$<BOOL:${ENABLE_ICM42670P}>:${ALIF_COMPONENTS_SRC_DIR}/ICM42670P_IMU.c>

    #LP I2C Driver
    $<$<AND:$<BOOL:${ENABLE_LPI2C}>,$<BOOL:${LP_DOMAIN_EN}>>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/lpi2c.c>
    $<$<AND:$<BOOL:${ENABLE_LPI2C}>,$<BOOL:${LP_DOMAIN_EN}>>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_LPI2C.c>

    #PDM Driver
    $<$<BOOL:${ENABLE_PDM}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/pdm.c>
    $<$<BOOL:${ENABLE_PDM}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_PDM.c>

    #SPI Driver
    $<$<BOOL:${ENABLE_SPI}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/spi.c>
    $<$<BOOL:${ENABLE_SPI}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_SPI.c>

    #LP Timer Driver
    $<$<BOOL:${ENABLE_LPTIMER}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_LPTIMER.c>

    #MRAM Driver
    $<$<BOOL:${ENABLE_MRAM}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/mram.c>
    $<$<BOOL:${ENABLE_MRAM}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_MRAM.c>

    #RTC Driver
    $<$<BOOL:${ENABLE_RTC}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_RTC.c>

    #UTIMER Driver
    $<$<BOOL:${ENABLE_UTIMER}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/utimer.c>
    $<$<BOOL:${ENABLE_UTIMER}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_UTIMER.c>

    #WDT Driver
    $<$<BOOL:${ENABLE_WDT}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_WDT.c>

    #WIFI Driver

    #DMA Driver
    $<$<BOOL:${ENABLE_DMA}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/dma_ctrl.c>
    $<$<BOOL:${ENABLE_DMA}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/dma_op.c>
    $<$<BOOL:${ENABLE_DMA}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_DMA.c>

    #I2C Driver
    $<$<BOOL:${ENABLE_I2C}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/i2c.c>
    $<$<BOOL:${ENABLE_I2C}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_I2C.c>

    #Camera MT9M114 Driver
    $<$<BOOL:${ENABLE_MT9M114_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/MT9M114_Camera_Sensor.c>

    #Camera HM0360  Driver
    $<$<BOOL:${ENABLE_HM0360_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/HM0360_camera_sensor.c>

    #Camera ARX3A0  Driver
    $<$<BOOL:${ENABLE_ARX3A0_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/arx3A0_camera_sensor.c>

    #Camera AR0144  Driver
    $<$<BOOL:${ENABLE_AR0144_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/ar0144_camera_sensor.c>

    #Camera AR0145  Driver
    $<$<BOOL:${ENABLE_AR0145_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/ar0145_camera_sensor.c>

    #Camera AR0246  Driver
    $<$<BOOL:${ENABLE_AR0246_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/ar0246_camera_sensor.c>

    #Camera OV5647  Driver
    $<$<BOOL:${ENABLE_OV5647_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/OV5647_camera_sensor.c>

    #Camera OV5675  Driver
    $<$<BOOL:${ENABLE_OV5675_CAMERA}>:${ALIF_COMPONENTS_SRC_DIR}/OV5675_camera_sensor.c>

    #MIPI CSI2 Driver
    $<$<BOOL:${ENABLE_MIPI_CSI2}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/csi.c>
    $<$<BOOL:${ENABLE_MIPI_CSI2}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/DPHY_CSI2.c>
    $<$<BOOL:${ENABLE_MIPI_CSI2}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_MIPI_CSI2.c>

    #MIPI DSI Driver
    $<$<BOOL:${ENABLE_MIPI_DSI}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/dsi.c>
    $<$<BOOL:${ENABLE_MIPI_DSI}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/DPHY_DSI.c>
    $<$<BOOL:${ENABLE_MIPI_DSI}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_MIPI_DSI.c>

    #MIPI DSI CSI2 PHY
    $<$<BOOL:${ENABLE_MIPI_DSI_CSI2_DPHY}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/DPHY_Common.c>

    #Phy Generic (Ethernet PHY)
    $<$<BOOL:${ENABLE_PHY_GENERIC}>:${ALIF_COMPONENTS_SRC_DIR}/driver_phy.c>

    #Ethernet MAC driver
    $<$<AND:$<BOOL:${ENABLE_ETH_MAC}>,$<OR:$<STREQUAL:${OS},CMSIS-RTOS>,$<STREQUAL:${OS},CMSIS-RTOS2>>>:
      ${ALIF_CMSIS_DRIVER_SRC_DIR}/driver_mac.c>

    #CPI Driver
    $<$<BOOL:${ENABLE_CPI}>:${ALIF_ENSEMBLE_DRIVERS_SRC_DIR}/cpi.c>
    $<$<BOOL:${ENABLE_CPI}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_CPI.c>
    $<$<BOOL:${ENABLE_CPI}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Camera_Sensor_i2c.c>

    #GPIO Driver
    $<$<BOOL:${ENABLE_GPIO}>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_GPIO.c>

    #WM8904 Driver
    $<$<BOOL:${ENABLE_WM8904}>:${ALIF_COMPONENTS_SRC_DIR}/WM8904_codec_i2c.c>
    $<$<BOOL:${ENABLE_WM8904}>:${ALIF_COMPONENTS_SRC_DIR}/WM8904_driver.c>

    #BMI323 Driver
    $<$<BOOL:${ENABLE_WM8904}>:${ALIF_COMPONENTS_SRC_DIR}/BMI323_IMU.c>

    #MCI Driver
    $<$<AND:$<BOOL:${ENABLE_MCI}>,$<STREQUAL:${OS},FREE-RTOS>>:${ALIF_CMSIS_DRIVER_SRC_DIR}/Driver_MCI.c>
)
