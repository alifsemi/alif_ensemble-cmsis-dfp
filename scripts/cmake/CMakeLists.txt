#/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
# * Use, distribution and modification of this code is permitted under the
# * terms stated in the Alif Semiconductor Software License Agreement
# *
# * You should have received a copy of the Alif Semiconductor Software
# * License Agreement with this file. If not, please write to:
# * contact@alifsemi.com, or visit: https://alifsemi.com/license
# *
# */

cmake_minimum_required(VERSION 3.25.0 FATAL_ERROR)

# These can be also set via command line
get_filename_component(ALIF_DEV_SRC_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/../.."     ABSOLUTE)

set(ALIF_BOARD_DIR              ${ALIF_DEV_SRC_DIR}/Boards)
set(ALIF_CMSIS_DRIVER_DIR       ${ALIF_DEV_SRC_DIR}/Alif_CMSIS)
set(ALIF_DEVICE_DIR             ${ALIF_DEV_SRC_DIR}/Device)
set(ALIF_COMPONENTS_DIR         ${ALIF_DEV_SRC_DIR}/components)
set(ALIF_ENSEMBLE_DRIVERS_DIR   ${ALIF_DEV_SRC_DIR}/drivers)

# Library directory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY          "${CMAKE_BINARY_DIR}/lib")

#string(TOLOWER $CACHE{COMPILER} TMP)
#set(CMAKE_TOOLCHAIN_FILE        ${ALIF_DEV_SRC_DIR}/scripts/toolchain/toolchain-${TMP}.cmake)
#include (${CMAKE_TOOLCHAIN_FILE})

set(USER_CMAKE_SCRIPT_DIR       "${CMAKE_CURRENT_SOURCE_DIR}/rtss")

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m"                CACHE INTERNAL "")
  set(ColourBold  "${Esc}[1m"               CACHE INTERNAL "")
  set(Red         "${Esc}[31m"              CACHE INTERNAL "")
  set(Green       "${Esc}[32m"              CACHE INTERNAL "")
  set(Yellow      "${Esc}[33m"              CACHE INTERNAL "")
  set(Blue        "${Esc}[34m"              CACHE INTERNAL "")
  set(Magenta     "${Esc}[35m"              CACHE INTERNAL "")
  set(Cyan        "${Esc}[36m"              CACHE INTERNAL "")
  set(White       "${Esc}[37m"              CACHE INTERNAL "")
  set(HCyan       "${Esc}[38;5;45m"         CACHE INTERNAL "")
endif()

#To Debug Cmake environment
#set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

# Modify the `PATH` for the build process
#set(ENV{PATH}   $ENV{COMPILER_BIN_PATH}:$ENV{PATH})

# Including Utilities cmake
include (${USER_CMAKE_SCRIPT_DIR}/utilities_func.cmake)

CONVERT_PRESET_VAR_TO_CMAKE_VAR(BOOT                    BOOT                    CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(DEVICE                  DEVICE                  CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(BOARD_NAME              BOARD_NAME              CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(CPU                     CPU                     CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(RTSS                    RTSS                    CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(BOARD_REV               BOARD_REV               CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(TEST_APP                TEST_APP                CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(COMPILER                COMPILER                CACHE)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(CMSIS_PACK_PATH         CMSIS_PACK_PATH         ENV)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(COMPILER_BIN_PATH       COMPILER_BIN_PATH       ENV)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(CMSIS_COMPILER_PATH     CMSIS_COMPILER_PATH     ENV)
CONVERT_PRESET_VAR_TO_CMAKE_VAR(SE_TOOLS_IMAGE_DIR      SE_TOOLS_IMAGE_DIR      ENV)

#Error Check
ERR_CHECK_MSG(FATAL_ERROR    CMSIS_PACK_PATH        "CMSIS PACK"        Red)
ERR_CHECK_MSG(FATAL_ERROR    COMPILER_BIN_PATH      "COMPILER"          Red)
ERR_CHECK_MSG(WARNING        CMSIS_COMPILER_PATH    "CMSIS COMPILER"    Magenta)

# Extract Device Series
string(SUBSTRING    ${DEVICE}   1   1   DEV_FAMILY)

# Device Known device name
if (DEV_FAMILY STREQUAL "E")
    set(DEVICE_FAMILY_NAME      "ENSEMBLE")
elseif(DEV_FAMILY STREQUAL "B")
    set(DEVICE_FAMILY_NAME      "BALLETTO")
else ()
    message (FATAL_ERROR "${Red}DEFINE PROPER DEVICE NAME ${ColourReset}")
endif()

if( ("${BOARD_NAME}" STREQUAL "DevKit-e1c") AND ("${RTSS}" STREQUAL "HP") )
    message(FATAL_ERROR     "\n E1C Board doesn't have HP sub-system ${ColourReset}")
endif()

# Setting the Toolchain path
set(CMAKE_TOOLCHAIN_FILE    "${USER_CMAKE_SCRIPT_DIR}/toolchains/toolchain.cmake")

# Setting Properssor Name
set (PROC                   "${CPU}_${RTSS}")

# Setting Paths for OS selection cmake files
set (OS_CMAKE_DIR           "${USER_CMAKE_SCRIPT_DIR}/os_cmake")

# Setting path for Driver cmake files
set (DRIVERS_CMAKE_DIR      "${USER_CMAKE_SCRIPT_DIR}/drivers_cmake")

# Setting path for Device related cmake file
set (DEVICE_CMAKE_DIR       "${USER_CMAKE_SCRIPT_DIR}/device_cmake")

# Setting path for se_host_services related cmake file
set (HOST_SERVICES_CMAKE_DIR "${USER_CMAKE_SCRIPT_DIR}/host_services_cmake")

set (DEVICE_PATH            "${ALIF_DEVICE_DIR}")
set (DRIVERS_CMAKE_FILE     "${DRIVERS_CMAKE_DIR}/drivers.cmake")
set (RETARGET_SRC_DIR       "${ALIF_DEV_SRC_DIR}/libs")
set (BOARD_DIR              "${ALIF_DEV_SRC_DIR}/libs/board_config")
set (BOARD_CMAKE_FILE       "${USER_CMAKE_SCRIPT_DIR}/board_cmake/board.cmake")

string(REGEX REPLACE    "^[^-]*-"   ""      BOARD_ABERRATION    "${BOARD_NAME}")
string(TOUPPER  "${BOARD_ABERRATION}"   BOARD_ABERRATION_IN_UPPER)
string(FIND     "${DEVICE}"    "A${BOARD_ABERRATION_IN_UPPER}"      found_subStr)

# Determine if DEVICE starts with "AE1C"
string(SUBSTRING "${DEVICE}" 0 4 BOARD_ABERRATION_ID_4)
string(SUBSTRING "${DEVICE}" 0 3 BOARD_ABERRATION_ID_3)

if(BOARD_ABERRATION_ID_4 STREQUAL "AE1C")
    set(BOARD_ABERRATION_ID "${BOARD_ABERRATION_ID_4}")
else()
    set(BOARD_ABERRATION_ID "${BOARD_ABERRATION_ID_3}")
endif()

if(found_subStr EQUAL -1)
    message(STATUS     "${Red} ⚠️ [WARNING] BOARD NAME (${BOARD_NAME}) AND DEVICE (${DEVICE}) IS NOT IN SYNC !!!${ColourReset}")
endif()

if(BOARD_ABERRATION_ID STREQUAL "AE1C")
    add_definitions(-DENSEMBLE_SOC_E1C)
elseif(BOARD_ABERRATION_ID STREQUAL "AE4" OR BOARD_ABERRATION_ID STREQUAL "AE8")
    add_definitions(-DENSEMBLE_SOC_GEN2)
elseif(BOARD_ABERRATION_ID STREQUAL "AB1")
    add_definitions(-DBALLETTO_DEVICE)
endif()


#Include Retarget Include path
set(RETARGET_IO_INC     "${CMSIS_COMPILER_PATH}/include;${RETARGET_SRC_DIR}/retarget_io/include")
set(RETARGET_IO_INC     "${RETARGET_IO_INC};${RETARGET_SRC_DIR}/retarget_io/config/${BOARD_ABERRATION}")
set(RETARGET_CFG_FILE   "${RETARGET_SRC_DIR}/retarget_io/config/${BOARD_ABERRATION}/retarget_config.h")

include_directories("${RETARGET_IO_INC};${CMSIS_PACK_PATH}/CMSIS/RTOS2/Include")
include("${USER_CMAKE_SCRIPT_DIR}/toolchains/compiler_config.cmake")

# Including Device related cmake
include (${DEVICE_CMAKE_DIR}/device.cmake)

set(CMAKE_TEMP_FOLDER       "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/.temp")
file(MAKE_DIRECTORY         ${CMAKE_TEMP_FOLDER})
set(TMP_CFG_FILE            "${CMAKE_TEMP_FOLDER}/.cfg" )
#execute_process(COMMAND     attrib +h   ${CMAKE_TEMP_FOLDER})

get_rte_macros()

set(MSG           "-- 📌 SELECTED COMPILER        --> ${HCyan}${COMPILER}${ColourReset}\n")
set(MSG     "${MSG}-- 📌 SELECTED DEVICE          --> ${HCyan}${DEVICE} (${BOARD_ABERRATION})${ColourReset}\n")
set(MSG     "${MSG}-- 📌 SELECTED CORE            --> ${HCyan}${CPU} (${RTSS}, Rev-${BOARD_REV})${ColourReset}\n")
set(MSG     "${MSG}-- 📌 SELECTED BOOT            --> ${HCyan}${BOOT}${ColourReset}\n")
set(MSG     "${MSG}-- 📌 SELECTED BOARD           --> ${HCyan}${BOARD_NAME}${ColourReset}\n")
set(MSG     "${MSG}-- 📌 SELECTED DEVICE FAMILY   --> ${HCyan}${DEVICE_FAMILY_NAME} ${ColourReset}\n")

# Including cmake for Drivers
include (${BOARD_CMAKE_FILE})

# Checking if Test app name is given
if(${TEST_APP} STREQUAL "ALL")
    set(EN_APP_FLAG        0    CACHE INTERNAL "")
else()
    set(EN_APP_FLAG        1    CACHE INTERNAL "")      # If Test App name is given, then make the flag high
endif()

if( ${ENABLE_STDIN} OR ${ENABLE_STDOUT} OR ${ENABLE_STDERR})

    if(${ENABLE_USART})
        set(RETARGET_EN_NO_SEMIHOSTING        ON)
    else()
        message(STATUS     "${Yellow} ⚠️ [WARNING] RETARGETTING IS ENABLED BUT UART DRIVER IS NOT ENABLED ${ColourReset}")
    endif()
endif()

if(${RETARGET_EN_NO_SEMIHOSTING})
    # Collect all retargeting source files
    file(GLOB_RECURSE   RETARGET_IO_SRCS    "${RETARGET_SRC_DIR}/retarget_io/source/*.c")
    list(APPEND         RETARGET_IO_SRCS    "${RETARGET_IO_SRC}")

    set(TMP_FILE2               "${CMAKE_TEMP_FOLDER}/.tmp2")

    if (EXISTS "${TMP_FILE2}")
        file(READ       ${TMP_FILE2}       PREV_RETARGET_CFG)
    else()
        set(PREV_RETARGET_CFG              "")
    endif()

    GET_MACRO_VALUE(${RETARGET_CFG_FILE}    PRINTF_UART_CONSOLE      uart_console_num)

    if ("${uart_console_num}" MATCHES "^[0-9]+$")
        set(BLOCKING_MODE_FLAG          RTE_UART${uart_console_num}_BLOCKING_MODE_ENABLE)
    else()
        GET_MACRO_STR_VALUE(${RETARGET_CFG_FILE}    PRINTF_UART_CONSOLE      uart_console_num)
        set(BLOCKING_MODE_FLAG          RTE_${uart_console_num}UART_BLOCKING_MODE_ENABLE)
    endif()

    GET_MACRO_VALUE("${DEVICE_SKU_RTE}/RTE_Device.h"   ${BLOCKING_MODE_FLAG}     uart_blocking_status)

    if( NOT(${uart_blocking_status}))
        message(STATUS      "✏️ RE-TARGETTING IS ENABLED, BUT UART POLLING_MODE IS NOT ENABLED")
        CHANGE_MACRO_VAL("#define ${BLOCKING_MODE_FLAG} 0"
            "${DEVICE_SKU_RTE}/RTE_Device.h"     "#define ${BLOCKING_MODE_FLAG} 1"
            uart_blocking_status_ret    ON)
        if(${uart_blocking_status_ret})
            message(STATUS      "🕵️‍♂️ FORCEFULLY UART POLLING_MODE IS ENABLED")
        else()
            message(STATUS      "⚠️ SETTING FORCEFULLY UART POLLING_MODE IS FAILED")
        endif()
    endif()

    if( NOT ("${BLOCKING_MODE_FLAG}" STREQUAL "${PREV_RETARGET_CFG}"))
        if( NOT ("" STREQUAL "${PREV_RETARGET_CFG}"))
            message(STATUS    " --------------- RETARGET CONFIG CHANGED --------------------")
        endif()

        CHANGE_MACRO_VAL("#define ${PREV_RETARGET_CFG} 1"
            "${DEVICE_SKU_RTE}/RTE_Device.h"     "#define ${PREV_RETARGET_CFG} 0"    uart_blocking_status_ret    ON)
        file(WRITE      ${TMP_FILE2}    "${BLOCKING_MODE_FLAG}")
    else()
        message(STATUS      "🕵️‍♂️ FORCEFULLY CHANGED UART POLLING_MODE WILL BE USED")
    endif()
endif()

if(${RETARGET_EN_NO_SEMIHOSTING})
    set(MSG     "${MSG}-- 📌 SELECTED STANDARD I/O    --> ${HCyan}UART (#${uart_console_num})${ColourReset}")
else()
    set(MSG     "${MSG}-- 📌 SELECTED STANDARD I/O    --> ${HCyan}DEBUGGER CONSOLE${ColourReset}")
endif()

# Mark the language as C so that CMake doesn't try to test the C++
# cross-compiler's ability to compile a simple program because that will fail
project(ALIF_RTSS_${PROC}_${TEST_APP}_Testapps
        VERSION 2.0.0
        DESCRIPTION "COLLECTION_OF_ALIF_TESTAPPS "
        LANGUAGES C ASM)

message("")
message(${MSG})
file(WRITE          ${TMP_CFG_FILE}     "${MSG}\n")

# Directory paths of OS cmake files
if (OS STREQUAL NONE)
    set(OS_CMAKE_FILE           "${OS_CMAKE_DIR}/baremetal.cmake")
    message (STATUS "📌 SELECTED OS              --> ${HCyan}Bare-Metal${ColourReset}\n")
    file(APPEND          ${TMP_CFG_FILE}     "-- 📌 SELECTED OS              --> ${HCyan}Bare-Metal${ColourReset}\n")

else()
    # OS is not proper
    file(APPEND          ${TMP_CFG_FILE}     "${Red}PROPER OS IS NOT GIVEN or \"${OS}\" OS IS NOT SUPPORTED ${ColourReset}\n")
    message (FATAL_ERROR "${Red}PROPER OS IS NOT GIVEN or \"${OS}\" OS IS NOT SUPPORTED ${ColourReset}")

endif ()

include (${OS_CMAKE_FILE})

#Remove Dependancy
resolve_dependancy()

# Including cmake for Drivers
include (${DRIVERS_CMAKE_FILE})

# Including Host Services cmake
include (${HOST_SERVICES_CMAKE_DIR}/se_host_services.cmake)

# Creating a Library file for Retarget IO & Common Source Files
if ((RETARGET_IO_SRCS) OR (COMMON_SRCS))
    set(COMMON_LIB     "COMMON")

    if(${RETARGET_EN_NO_SEMIHOSTING})
        list(APPEND     COMMON_SRCS     ${RETARGET_IO_SRCS})
    endif()

    if(NOT COMMON_SRCS)
        message(WARNING "COMMON_SRCS is empty. No source files found!")
    else()
        add_library(${COMMON_LIB}   STATIC      ${COMMON_SRCS})
    endif()
endif()


# Collecting all the Test application names
list(LENGTH    TEST_APP_SRCS  TOTAL_NUM_OF_APPLICATION)
message("\n<--- Test Applications to be Built -->")

string (REPLACE     ","     ";"     TEST_APPS           ${TEST_APP} )

if (EN_APP_FLAG)
    list(LENGTH     TEST_APPS   TOTAL_NUM_OF_REQUESTED_APPLICATION)
    list(FIND       TEST_APPS   "ALL"   ALL_APPS_FLAG)

    if( NOT ALL_APPS_FLAG EQUAL -1)
        set(EN_APP_FLAG     0   CACHE INTERNAL "")
        MATH(EXPR TOTAL_NUM_OF_REQUESTED_APPLICATION "${TOTAL_NUM_OF_APPLICATION} - ${TOTAL_NUM_OF_REQUESTED_APPLICATION} + 1")
        set(EXCLUDED_APP_LIST   "${TEST_APPS}")
        set(TEST_APPS           "ALL")
        list(REMOVE_ITEM ${EXCLUDED_APP_LIST} "ALL")
    endif()
    message ("<--- ${TOTAL_NUM_OF_REQUESTED_APPLICATION} application requested to build out of ${TOTAL_NUM_OF_APPLICATION} -->")

else ()
    set(TOTAL_NUM_OF_REQUESTED_APPLICATION ${TOTAL_NUM_OF_APPLICATION})
    message ("<--- All ${TOTAL_NUM_OF_APPLICATION} applications will build -->")

endif()

set(TMP_FILE                "${CMAKE_TEMP_FOLDER}/.tmp" )
set(TMP_FILE1               "${CMAKE_TEMP_FOLDER}/.tmp1")
set(GENERATED_TEST_APP_LIST     "")
set(REQUESTED_TEST_APP_LIST     "")

file(WRITE      ${TMP_FILE}     ${OUTPUT_DIR})
file(WRITE      ${TMP_FILE1}    "")
file(APPEND     ${TMP_FILE}     "\nRequested: ${TOTAL_NUM_OF_REQUESTED_APPLICATION}, Total: ${TOTAL_NUM_OF_APPLICATION}")
SET(INVALID_TESTAPP_CNT     0)

foreach (testApp ${TEST_APPS})
    set(TEST_FOUND_FLAG "Not Found")
    foreach (testsourcefile ${TEST_APP_SRCS})

        # Collectiong Test app names one by one
        get_filename_component (tempname "${testsourcefile}" NAME)
        string (REPLACE ".c" "" testname ${tempname} )

        if("${testname}" STREQUAL "demo_sd_fatfs")
            set(addonsourcefiles        ${SD_TEST_APP_DEP_SRCS})
        elseif("${testname}" STREQUAL "demo_dphy_loopback")
            set(addonsourcefiles        ${DPHY_LOOPBACK_TEST_APP_DEP_SRCS})
        elseif("${testname}" STREQUAL "demo_lvgl")
            set(addonsourcefiles        ${LVGL_TEST_APP_DEP_SRCS})
        else()
            set(addonsourcefiles        "")
        endif()

        # If the Flag is high, build only for the given Test Application
        if (EN_APP_FLAG)

            if (testname STREQUAL testApp)

                message (STATUS "🎯 App Name --> ${Yellow}${testname}${ColourReset}")
                set(TEST_FOUND_FLAG "found")

                file(APPEND ${TMP_FILE} "\n${testname}")
                list(APPEND     ${REQUESTED_TEST_APP_LIST}      ${testname})

                # Calling the build function
                BUILD_PROJECT ()

                break ()

            endif ()

        # Build for all the test Applications
        elseif (NOT EN_APP_FLAG)
            list(FIND       EXCLUDED_APP_LIST   ${testname}   EXCLUDED_APP_LIST_FLAG)

            if (EXCLUDED_APP_LIST_FLAG EQUAL -1)
                message (STATUS "🎯 App Name --> ${Yellow}${testname}${ColourReset}")

                file(APPEND ${TMP_FILE} "\n${testname}")
                list(APPEND     ${REQUESTED_TEST_APP_LIST}      ${testname})

                # Calling the build function
                BUILD_PROJECT ()

            endif()

        endif ()

    endforeach (testsourcefile )

    if ( (TEST_FOUND_FLAG STREQUAL "Not Found") AND (NOT testApp STREQUAL "ALL" ) )
        message (STATUS "🚨 App Name --> ${Yellow}${testApp}    => [ :${Red}${TEST_FOUND_FLAG} ]${ColourReset}")
        MATH(EXPR INVALID_TESTAPP_CNT "${INVALID_TESTAPP_CNT}+1")
    endif ()

endforeach (testApp)

message("\n")
message("${HCyan}Disabled/Removed (because of improper flags selection) App List as follows : ${ColourReset}")

foreach(element IN LISTS RM_TEST_APPS_LIST)
    message(STATUS  "🚫 ${Yellow} ${element} ${ColourReset}")
endforeach()

list(LENGTH    ALL_DEMO_FILES       TOTAL_AVAILABLE_DEMO_FILES_CNT)
list(LENGTH    RM_TEST_APPS_LIST    REMOVED_TEST_APP_CNT)
string(FIND    "${TEST_APP}"        "ALL"       POS_ALL)
string(FIND    "${TEST_APP}"        "ALL,"      POS_ALL_COMMA)

if ((NOT POS_ALL EQUAL -1 OR NOT POS_ALL_COMMA EQUAL -1))
    MATH(EXPR   TOTAL_ACCOUNTED_TEST_APP_CNT  "${REMOVED_TEST_APP_CNT}+${TOTAL_NUM_OF_APPLICATION}")

    if(NOT (${TOTAL_AVAILABLE_DEMO_FILES_CNT} STREQUAL ${TOTAL_ACCOUNTED_TEST_APP_CNT}))
        message("${HCyan}\nNot Included Test App List as follows (in current config) : ${ColourReset}")
        foreach(element IN LISTS ALL_DEMO_FILES)
            # Collectiong Test app names one by one
            get_filename_component (tempname "${element}" NAME)
            string (REPLACE ".c" "" testname ${tempname})

            set(FOUND_APP_FLAG      OFF)
            foreach(testAppName IN LISTS TEST_APP_SRCS)
                # Collectiong Test app names one by one
                get_filename_component (tempname1 "${testAppName}" NAME)
                string (REPLACE ".c" "" testname1 ${tempname1})
                if(testname1 STREQUAL testname)
                    set(FOUND_APP_FLAG      ON)
                endif()
            endforeach()

            if(NOT ${FOUND_APP_FLAG})
                foreach(testAppName IN LISTS RM_TEST_APPS_LIST)
                    if(testAppName STREQUAL testname)
                        set(FOUND_APP_FLAG      ON)
                    endif()
                endforeach()
            endif()
            if(NOT ${FOUND_APP_FLAG})
                message(STATUS  "🔴 ${Yellow} ${testname} ${ColourReset}")
            endif()
        endforeach()
    endif()
endif()

message("")
file(APPEND ${TMP_FILE} "\n${INVALID_TESTAPP_CNT}")

# Replace multiple spaces with a single space
string(REGEX REPLACE " + " " " REFINED_LINKER_CMD       "${LINKER_CMD}")

# Convert the string into a CMake list (splitting by spaces)
string(REPLACE  " "     ";"     LINKER_FLAGS            "${REFINED_LINKER_CMD}")

if(EXISTS ${LINKER_SCRIPT_FILE})
    execute_process(
        COMMAND ${CMAKE_C_COMPILER} ${LINKER_FLAGS} ${LINKER_INC_PATH} ${LINKER_SCRIPT_FILE} -o ${CMAKE_LINKER_SCRIPT}
        RESULT_VARIABLE LINKER_COMMAND_RESULT
        OUTPUT_VARIABLE LINKER_COMMAND_OUTPUT
        ERROR_VARIABLE  LINKER_COMMAND_ERROR
        #COMMAND_ECHO STDOUT
    )

    # Check if the command was successful
    if(LINKER_COMMAND_RESULT EQUAL 0)
        message(STATUS "✅ ${Green} ${CMAKE_LINKER_SCRIPT} ${ColourReset} LINKER FILE GENERATED SUCCESSFULLY "
            "USING ${Cyan}${LINKER_SCRIPT_FILE} ${ColourReset} ${COMMAND_OUTPUT}")
        if ((COMPILER STREQUAL ARMCLANG) AND EXISTS ${LINKER_SCRIPT_FILE} )
            file(REMOVE ${LINKER_SCRIPT_FILE})
        endif()
    else()
        message(FATAL_ERROR "❌ ${Red}${CMAKE_LINKER_SCRIPT} ${ColourReset} FAILED TO GENERATE LINKER FILE FROM "
            "${Cyan}${LINKER_SCRIPT_FILE} ${ColourReset} ${COMMAND_ERROR}")
    endif()

else()
    message(FATAL_ERROR             "${Red}🚨 LINKER FILE NOT FOUND ...${ColourReset}")

endif()

#--------------------------------------------------------------------------------------------------------
enable_testing()

add_test(NAME BUILD_STATISTICS COMMAND bash ${CMAKE_SOURCE_DIR}/build_stats.sh    ${CMAKE_TEMP_FOLDER})
add_test(NAME SE_FLASHING_TOOL COMMAND bash ${CMAKE_SOURCE_DIR}/se_flashing.sh)
#--------------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------------
include(CPack)
#--------------------------------------------------------------------------------------------------------
