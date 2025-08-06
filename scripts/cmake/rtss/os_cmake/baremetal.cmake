# Bare-Metal related files and directories
set (BARE_METAL_APP_DIR         "${ALIF_BOARD_DIR}/Templates/Baremetal")
set (TEST_APP_INC_PATHS         "${ALIF_BOARD_DIR}/Templates/bayer2rgb"
                                "${ALIF_BOARD_DIR}/Templates/Common/Include"
                                "${BARE_METAL_APP_DIR}/Include"
                                "${BARE_METAL_APP_DIR}/FatFS"
                                "${ALIF_BOARD_DIR}/ospi_xip")


include_directories ("${TEST_APP_INC_PATHS}")

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_uart4.c               ENABLE_USART       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_uart2.c               ENABLE_USART       TEST_APP_SRCS   "test-apps")

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_lpuart.c          ENABLE_USART       TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_lpuart")
endif()

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_adc.c                 ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_adc_clickboard.c      ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_adc_exttrigger.c      ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_adc_potentiometer.c   ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_tsens.c               ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_bmi323.c              ENABLE_BMI323      TEST_APP_SRCS   "test-apps")

if( ${EN_APP_FLAG} AND (TEST_APP STREQUAL "demo_canfd_blockingmode"))
    message(STATUS      "✏️ CANFD IS BY_DEFAULT DISABLED, SO ENABLING CANFD BLOCKING MODE")
    CHANGE_MACRO_VAL("#define RTE_CANFD0_BLOCKING_MODE_ENABLE      0"     "${DEVICE_SKU_RTE}/RTE_Device.h"
        "#define RTE_CANFD0_BLOCKING_MODE_ENABLE      1"    canfd_blocking_status_ret    ON)
    message(STATUS      "🕵️‍♂️ FORCEFULLY CANFD BLOCKING MODE IS ENABLED")
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_canfd_blockingmode.c  ENABLE_CANFD   TEST_APP_SRCS   "test-apps")

else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_canfd_blockingmode")

endif()

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_canfd_busmonitor.c    ENABLE_CANFD       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_canfd_extloopback.c   ENABLE_CANFD       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_canfd_intloopback.c   ENABLE_CANFD       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_canfd_normalmode.c    ENABLE_CANFD       TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     AND     ENABLE_CDC200    ENABLE_IO      ENABLE_MIPI_DSI     ENABLE_MIPI_DSI_CSI2_DPHY)
set(TMP1_FLAG           OFF)
if((ENABLE_MIPI_DSI_ILI9806E_PANEL AND NOT ENABLE_MIPI_DSI_ILI9488E_PANEL) OR
    (NOT ENABLE_MIPI_DSI_ILI9806E_PANEL AND ENABLE_MIPI_DSI_ILI9488E_PANEL))
    set(TMP1_FLAG       ON)
endif()
eval_flags(TMP2_FLAG    AND     TMP_FLAG  TMP1_FLAG)
if( ${TMP2_FLAG} AND (NOT ${ENABLE_CDC_ILI6122E_PANEL}))
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_cdc200.c          ON       TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_cdc200")
endif()

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_cmp.c                 ENABLE_CMP         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_crc.c                 ENABLE_CRC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_dac.c                 ENABLE_DAC         TEST_APP_SRCS   "test-apps")

if (EN_APP_FLAG  AND (${TEST_APP} STREQUAL "demo_dphy_loopback" ))  #TODO
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/dphy_loopback.c        ENABLE_IO          DPHY_LOOPBACK_TEST_APP_DEP_SRCS   "dependency")
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_dphy_loopback.c   ENABLE_IO          TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_dphy_loopback")
endif()

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_sdio.c                ENABLE_SD          TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_sd.c                  ENABLE_SD          TEST_APP_SRCS   "test-apps")

COND_FILE_ADD(${BARE_METAL_APP_DIR}/FatFS/diskio.c             ENABLE_SD   SD_TEST_APP_DEP_SRCS   "dependency")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/FatFS/ff.c                 ENABLE_SD   SD_TEST_APP_DEP_SRCS   "dependency")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/FatFS/ffsystem.c           ENABLE_SD   SD_TEST_APP_DEP_SRCS   "dependency")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/FatFS/ffunicode.c          ENABLE_SD   SD_TEST_APP_DEP_SRCS   "dependency")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_sd_fatfs.c            ENABLE_SD          TEST_APP_SRCS   "test-apps" )

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_flash_issi.c          ENABLE_ISSI_FLASH      TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_gt911.c               ENABLE_GT911           TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     AND     ENABLE_HWSEM       ENABLE_USART)
if(${TMP_FLAG})
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_hwsem.c           ${TMP_FLAG}        TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_hwsem")
endif()

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_i2s.c                 ENABLE_I2S         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_i3c_master.c          ENABLE_I3C         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_i3c_slave.c           ENABLE_I3C         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_i3c_i2cmixbus.c       ENABLE_I3C         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_icm42670p.c           ENABLE_ICM42670P   TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_led_blinky.c          ENABLE_IO          TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_led_breathe.c         ENABLE_IO          TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     AND     ENABLE_UTIMER       ENABLE_IO)
if(${TMP_FLAG})
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_qec.c             ${TMP_FLAG}        TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_qec")
endif()

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_lpi2c0.c          ENABLE_LPI2C       TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_lpi2c0")
endif()

if (NOT ${ENABLE_E1C_DEVKIT})
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_pdm.c             ENABLE_PDM         TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_pdm")
endif()

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_lppdm.c           ENABLE_PDM         TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_lppdm")
endif()

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_spi_loopback.c        ENABLE_SPI         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_spi_microwire.c       ENABLE_SPI         TEST_APP_SRCS   "test-apps")

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_lpspi.c           ENABLE_SPI         TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_lpspi")
endif()

COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_lptimer.c             ENABLE_LPTIMER     TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_mram.c                ENABLE_MRAM        TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_rtc.c                 ENABLE_RTC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_utimer.c              ENABLE_UTIMER      TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_wdt.c                 ENABLE_WDT         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_dma_memcpy.c          ENABLE_DMA         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_i2c.c                 ENABLE_I2C         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_i3c_i2cbusproxy.c     ENABLE_I2C         TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     OR     ENABLE_RTC          ENABLE_LPTIMER)
if(${TMP_FLAG})
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_pm.c              ${TMP_FLAG}        TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_pm")
endif()

eval_flags(TMP_FLAG     AND     ENABLE_CDC200    ENABLE_IO      ENABLE_CDC_ILI6122E_PANEL)
eval_flags(TMP1_FLAG    OR      ENABLE_MIPI_DSI_ILI9806E_PANEL  ENABLE_MIPI_DSI_ILI9488E_PANEL)

if( ${TMP_FLAG} AND (NOT ${TMP1_FLAG}) AND (NOT ${ENABLE_MIPI_DSI}) AND (NOT ${ENABLE_MIPI_DSI_CSI2_DPHY}))
    add_definitions(-DRTE_MIPI_DSI=0)
    add_definitions(-DRTE_ILI6122_PANEL=1)
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_parallel_display.c    ON      TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_parallel_display")
endif()

#TODO: It needs to enabled only when LVGL pack is downloaded.
if (EN_APP_FLAG  AND (${TEST_APP} STREQUAL "demo_lvgl" ))
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/lv_port_disp.c         ENABLE_ADC         LVGL_TEST_APP_DEP_SRCS   "dependency")
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_lvgl.c            ENABLE_ADC         TEST_APP_SRCS            "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_lvgl")
endif()

if (${ENABLE_E7_DEVKIT})
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_hyperram_e7.c     ENABLE_XIP_HYPERRAM    TEST_APP_SRCS   "test-apps")
elseif (${ENABLE_E8_DEVKIT})
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_hyperram_e8.c     ENABLE_XIP_HYPERRAM    TEST_APP_SRCS   "test-apps")
elseif (${ENABLE_E1C_DEVKIT})
    COND_FILE_ADD(${BARE_METAL_APP_DIR}/demo_hyperram_e1c.c    ENABLE_XIP_HYPERRAM    TEST_APP_SRCS   "test-apps")
elseif (${ENABLE_E4_DEVKIT})
    message(STATUS             "${Yellow}⚠️ [WARNING] demo_hyperram_e4 is missing ${ColourReset}")
endif()

file(GLOB   ALL_DEMO_FILES    "${BARE_METAL_APP_DIR}/demo_*")
