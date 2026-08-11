# FreeRTOS OS related Directories and files
message(STATUS   "CMSIS_FREERTOS_PATH : ${CMSIS_FREERTOS_PATH}")
set(FREERTOS_CMSIS_DIR             "${CMSIS_FREERTOS_PATH}/CMSIS/RTOS2/FreeRTOS")
set(FREERTOS_SRC_DIR               "${CMSIS_FREERTOS_PATH}/Source")
set(FREERTOS_PORTABLE_DIR          "${FREERTOS_SRC_DIR}/portable/GCC/ARM_CM55_NTZ/non_secure")
set(FREERTOS_CONFIG_HEADER_PATH    "${CMSIS_FREERTOS_PATH}/Config/ARMCM")
set(FREE_RTOS_CONFIG_CHANGE        ON)

file (GLOB FREERTOS_SRC            "${FREERTOS_SRC_DIR}/*.c")
file (GLOB FREERTOS_PORTABLE_SRC   "${FREERTOS_PORTABLE_DIR}/*.c")
file (GLOB FREERTOS_MEMMANG_SRC    "${FREERTOS_SRC_DIR}/portable/MemMang/heap_4.c")

# Including related header files to the project
set(FREE_RTOS_INC   "${FREERTOS_SRC_DIR}/include;${FREERTOS_PORTABLE_DIR};   \
                     ${FREERTOS_CONFIG_HEADER_PATH};${FREERTOS_CMSIS_DIR}/Include;")

include_directories (${FREE_RTOS_INC})

# Collecting all OS related files under one variable name
file (GLOB OS_C_SRC ${FREERTOS_SRC} ${FREERTOS_PORTABLE_SRC} ${FREERTOS_MEMMANG_SRC})


#--------------------------------------------------------------------------------------------------
if(${FREE_RTOS_CONFIG_CHANGE})
    set(FREE_RTOS_CONFIG_FILE       "${FREERTOS_CONFIG_HEADER_PATH}/FreeRTOSConfig.h")

    # Change FreeRTOS Config as per Alif's Board Need
    set(CHANGE_LOG  "${HCyan}CHANGING SOME OF THE PARAMETERS AS PER ALIF NEED ...${ColourReset}")
    message(STATUS  " ${CHANGE_LOG}")
    SAVE_ORIG_FILE("${FREE_RTOS_CONFIG_FILE}"    nFileName   configChange)

    CHANGE_MACRO_VAL("#define configENABLE_TRUSTZONE                1"
            "${FREE_RTOS_CONFIG_FILE}" "#define configENABLE_TRUSTZONE                0"   ret  ON)

    if(${ret})
        set(CHANGE_MSG              "${CHANGE_MSG}  \
            TRUSTZONE ENABLE/DISABLE       0 TO      1 (configENABLE_TRUSTZONE)\n")
    endif()

    CHANGE_MACRO_VAL("#define configRUN_FREERTOS_SECURE_ONLY        0"   "${FREE_RTOS_CONFIG_FILE}"
            "#define configRUN_FREERTOS_SECURE_ONLY        1"   ret  ON)

    if(${ret})
        set(CHANGE_MSG      "${CHANGE_MSG}          \
            SECURE ENABLE/DISABLE          0 TO      1 (configRUN_FREERTOS_SECURE_ONLY)\n")
    endif()

    CHANGE_MACRO_VAL("#define configUSE_TIMERS                      0"   "${FREE_RTOS_CONFIG_FILE}"
        "#define configUSE_TIMERS                      1"  ret  ON)

    if(${ret})
        set(CHANGE_MSG      "${CHANGE_MSG}          \
        TIMERS ENABLE/DISABLE          0 TO      1 (configUSE_TIMERS)\n")
    endif()

    set(CHANGE_MSG_FILENAME         "${FREERTOS_CONFIG_HEADER_PATH}/changes.txt")

    if(${configChange})
        file(WRITE      "${CHANGE_MSG_FILENAME}"    ${CHANGE_MSG})
    else()
        file(READ       "${CHANGE_MSG_FILENAME}"       CHANGE_MSG)
    endif()
endif()
#--------------------------------------------------------------------------------------------------

# Collecting all OS related files under one variable name
# Creating Library file for OS files
set (OS_LIB_FILE                "FREERTOS")
add_library (${OS_LIB_FILE}     STATIC      ${OS_C_SRC})

###################################################################################################

# Free-RTOS Alif testApps files and directories
set (FREE_RTOS_APP_DIR          "${ALIF_BOARD_DIR}/Templates/FreeRTOS")
set (TEST_APP_INC_PATHS         "${ALIF_BOARD_DIR}/Templates/bayer2rgb"
                                "${ALIF_BOARD_DIR}/Templates/Common/Include")


include_directories ("${TEST_APP_INC_PATHS}")

COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_uart4_freertos.c               ENABLE_USART       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_uart2_freertos.c               ENABLE_USART       TEST_APP_SRCS   "test-apps")

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_lpuart_freertos.c          ENABLE_USART       TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_lpuart_freertos")
endif()

COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_adc_freertos.c                 ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_adc_clickboard_freertos.c      ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_adc_exttrigger_freertos.c      ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_adc_potentiometer_freertos.c   ENABLE_ADC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_tsens_freertos.c               ENABLE_ADC         TEST_APP_SRCS   "test-apps")
# COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_bmi323_freertos.c            ENABLE_BMI323      TEST_APP_SRCS   "test-apps")

#COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_canfd_busmonitor_freertos.c   ENABLE_CANFD       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_canfd_extloopback_freertos.c   ENABLE_CANFD       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_canfd_intloopback_freertos.c   ENABLE_CANFD       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_canfd_normalmode_freertos.c    ENABLE_CANFD       TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     AND     ENABLE_CDC200    ENABLE_IO      ENABLE_MIPI_DSI     ENABLE_MIPI_DSI_CSI2_DPHY)
set(TMP1_FLAG           OFF)
if((ENABLE_MIPI_DSI_ILI9806E_PANEL AND NOT ENABLE_MIPI_DSI_ILI9488E_PANEL) OR
    (NOT ENABLE_MIPI_DSI_ILI9806E_PANEL AND ENABLE_MIPI_DSI_ILI9488E_PANEL))
    set(TMP1_FLAG       ON)
endif()
eval_flags(TMP2_FLAG    AND     TMP_FLAG  TMP1_FLAG)
if( ${TMP2_FLAG} AND (NOT ${ENABLE_CDC_ILI6122E_PANEL}))
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_cdc200_freertos.c          ON       TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_cdc200_freertos")
endif()

COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_cmp_freertos.c                 ENABLE_CMP         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_crc_freertos.c                 ENABLE_CRC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_dac_freertos.c                 ENABLE_DAC         TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     OR      ENABLE_ISSI_FLASH  ENABLE_MX66UW1G_FLASH)
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_flash_freertos.c               ${TMP_FLAG}        TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_gt911_freertos.c               ENABLE_GT911       TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     AND     ENABLE_HWSEM       ENABLE_USART)
if(${TMP_FLAG})
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_hwsem_freertos.c           ${TMP_FLAG}        TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_hwsem_freertos")
endif()

COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_i2s_freertos.c                 ENABLE_I2S         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_i3c_master_freertos.c          ENABLE_I3C         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_i3c_slave_freertos.c           ENABLE_I3C         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_i3c_i2cmixbus_freertos.c       ENABLE_I3C         TEST_APP_SRCS   "test-apps")
#COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_icm42670p_freertos.c          ENABLE_ICM42670P   TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_led_blinky_freertos.c          ENABLE_IO          TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_led_breathe_freertos.c         ENABLE_IO          TEST_APP_SRCS   "test-apps")

eval_flags(TMP_FLAG     AND     ENABLE_UTIMER       ENABLE_IO)
if(${TMP_FLAG})
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_qec_freertos.c             ${TMP_FLAG}        TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_qec")
endif()

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_lpi2c0_freertos.c          ENABLE_LPI2C       TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_lpi2c0_freertos")
endif()

if (NOT (ENABLE_E1C_DEVKIT OR ENABLE_E1C_STARTKIT))
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_pdm_freertos.c             ENABLE_PDM         TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_pdm_freertos")
endif()

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_lppdm_freertos.c           ENABLE_PDM         TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_lppdm_freertos")
endif()

COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_spi_loopback_freertos.c        ENABLE_SPI         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_spi_microwire_freertos.c       ENABLE_SPI         TEST_APP_SRCS   "test-apps")

if(${RTSS} STREQUAL HE)
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_lpspi_freertos.c           ENABLE_SPI         TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST   "demo_lpspi_freertos")
endif()

COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_lptimer_freertos.c             ENABLE_LPTIMER     TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_mram_freertos.c                ENABLE_MRAM        TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_rtc_freertos.c                 ENABLE_RTC         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_utimer_freertos.c              ENABLE_UTIMER      TEST_APP_SRCS   "test-apps")
#COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_wdt_freertos.c                ENABLE_WDT         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_dma_memcpy_freertos.c          ENABLE_DMA         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_i2c_freertos.c                 ENABLE_I2C         TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_i3c_i2cbusproxy_freertos.c     ENABLE_I2C         TEST_APP_SRCS   "test-apps")

# eval_flags(TMP_FLAG     OR     ENABLE_RTC          ENABLE_LPTIMER)
# if(${TMP_FLAG})
    # COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_pm_freertos.c            ${TMP_FLAG}        TEST_APP_SRCS   "test-apps")
# else()
    # list(APPEND     RM_TEST_APPS_LIST      "demo_pm_freertos")
# endif()

eval_flags(TMP_FLAG     AND     ENABLE_CDC200    ENABLE_IO      ENABLE_CDC_ILI6122E_PANEL)
eval_flags(TMP1_FLAG    OR      ENABLE_MIPI_DSI_ILI9806E_PANEL  ENABLE_MIPI_DSI_ILI9488E_PANEL)

if( ${TMP_FLAG} AND (NOT ${TMP1_FLAG}) AND (NOT ${ENABLE_MIPI_DSI}) AND (NOT ${ENABLE_MIPI_DSI_CSI2_DPHY}))
    add_definitions(-DRTE_MIPI_DSI=0)
    add_definitions(-DRTE_ILI6122_PANEL=1)
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_parallel_display_freertos.c    ON      TEST_APP_SRCS   "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_parallel_display_freertos")
endif()

#TODO: It needs to enabled only when LVGL pack is downloaded.
if (EN_APP_FLAG  AND (${TEST_APP} STREQUAL "demo_lvgl" ))
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/lv_port_disp_freertos.c         ENABLE_ADC         LVGL_TEST_APP_DEP_SRCS   "dependency")
    COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_lvgl_freertos.c            ENABLE_ADC         TEST_APP_SRCS            "test-apps")
else()
    list(APPEND     RM_TEST_APPS_LIST      "demo_lvgl")
endif()

COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_hyperram_freertos.c            ENABLE_XIP_PSRAM       TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_camera_arx3a0_freertos.c       ENABLE_ARX3A0_CAMERA   TEST_APP_SRCS   "test-apps")
COND_FILE_ADD(${FREE_RTOS_APP_DIR}/demo_camera_mt9m114_freertos.c      ENABLE_MT9M114_CAMERA  TEST_APP_SRCS   "test-apps")

file(GLOB   ALL_DEMO_FILES    "${FREE_RTOS_APP_DIR}/demo_*")
