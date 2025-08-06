# Basic Input check Starts -------------------------------
if (DEV_FAMILY STREQUAL "E")
    if("${BOARD_NAME}" STREQUAL "DevKit-e7")
        set(RTE_COMP_DIR   "${CMAKE_SOURCE_DIR}/Include_RTE_Comp/ensemble/e7")
        set(ENABLE_E7_DEVKIT    ON      CACHE   BOOL    "Enable/disable E7  Devkit.")
        set(ENABLE_E1C_DEVKIT   OFF     CACHE   BOOL    "Enable/disable E1C Devkit.")
        set(ENABLE_E8_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E8  Devkit.")
        set(ENABLE_E4_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E4  Devkit.")
    elseif("${BOARD_NAME}" STREQUAL "DevKit-e8")
        set(RTE_COMP_DIR   "${CMAKE_SOURCE_DIR}/Include_RTE_Comp/ensemble/e8")
        set(ENABLE_E7_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E7  Devkit.")
        set(ENABLE_E1C_DEVKIT   OFF     CACHE   BOOL    "Enable/disable E1C Devkit.")
        set(ENABLE_E8_DEVKIT    ON      CACHE   BOOL    "Enable/disable E8  Devkit.")
        set(ENABLE_E4_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E4  Devkit.")
    elseif("${BOARD_NAME}" STREQUAL "DevKit-e1c")
        set(RTE_COMP_DIR   "${CMAKE_SOURCE_DIR}/Include_RTE_Comp/ensemble/e1c")
        set(ENABLE_E7_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E7  Devkit.")
        set(ENABLE_E1C_DEVKIT   ON      CACHE   BOOL    "Enable/disable E1C Devkit.")
        set(ENABLE_E8_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E8  Devkit.")
        set(ENABLE_E4_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E4  Devkit.")
    elseif("${BOARD_NAME}" STREQUAL "DevKit-e4")
        set(RTE_COMP_DIR   "${CMAKE_SOURCE_DIR}/Include_RTE_Comp/ensemble/e1c")
        set(ENABLE_E7_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E7  Devkit.")
        set(ENABLE_E1C_DEVKIT   OFF     CACHE   BOOL    "Enable/disable E1C Devkit.")
        set(ENABLE_E8_DEVKIT    OFF     CACHE   BOOL    "Enable/disable E8  Devkit.")
        set(ENABLE_E4_DEVKIT    ON      CACHE   BOOL    "Enable/disable E4  Devkit.")
    endif()
else ()
    message (FATAL_ERROR "${Red}\n Unknown Device Family \n${ColourReset}")
endif ()

if(EXISTS "${RTE_COMP_DIR}/RTE_Components.h")
    file (GLOB RTE_COMPONENTS_FILE  "${RTE_COMP_DIR}/RTE_Components.h")
    file (STRINGS ${RTE_COMPONENTS_FILE}  RTEcomponentFile)
else ()
    message (FATAL_ERROR "${Red}\n RTE_Components.h is missing \n${ColourReset}")
endif ()

if (NOT( (BOOT STREQUAL TCM) OR (BOOT STREQUAL MRAM)))
    message(FATAL_ERROR  "${Red}\n BOOT MODE IS NOT GIVEN OR \"${BOOT}\" BOOT IS INVALID BOOT MODE ... !!!\n${ColourReset}")
endif ()

# Basic Input check Ends ---------------------------------

string(TOLOWER          ${RTSS}             rtss)
string(TOLOWER          ${BOOT}             boot)
set(RTSS_SUB_SYS_NAME   "rtss_${rtss}")

# Binary Directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY          ${CMAKE_BINARY_DIR}/exec/${DEVICE}/${RTSS}/${BOOT})

#Make Directory
if( NOT EXISTS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

# Linker File Details
if (COMPILER STREQUAL ARMCLANG)
    set(LINKER_CMD                     "--target=arm-arm-none-eabi -mcpu=cortex-m55  -E -P -x c")
    set(SCRIPT_DIR                     "${DEVICE_PATH}/core/${RTSS_SUB_SYS_NAME}/linker")
    set(COMPILER_LINKER_SPEC_NAME           "ac6")
    set(COMPILER_LINKER_INPUT_FILE_EXT      "sct.tmp")
    set(COMPILER_LINKER_OUTPUT_FILE_EXT     "sct")

    file(READ       "${SCRIPT_DIR}/linker_ac6_${boot}.sct"       LINKER_CONTENT)
    string(REPLACE      ";"     "@SEMICOLON@"        LINES_TEMP         "${LINKER_CONTENT}") # Temporary replacement
    string(REPLACE      "\n"    ";"     LINES       "${LINES_TEMP}")
    list(REMOVE_AT      LINES   0   1)
    string(REPLACE      ";"     "\n"    NEW_LINKER_CONTENT_TMP    "${LINES}")
    string(REPLACE      "@SEMICOLON@"     ";"    NEW_LINKER_CONTENT    "${NEW_LINKER_CONTENT_TMP}")
    file(WRITE      "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/linker_ac6_${boot}.${COMPILER_LINKER_INPUT_FILE_EXT}"  "${NEW_LINKER_CONTENT}")
    set(SCRIPT_DIR                     ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

elseif ( (COMPILER STREQUAL GCC) OR (COMPILER STREQUAL CLANG))
    set(LINKER_CMD                          "-E -P -x c")
    set(SCRIPT_DIR                          "${DEVICE_PATH}/core/${RTSS_SUB_SYS_NAME}/linker")
    set(COMPILER_LINKER_SPEC_NAME           "gnu")
    set(COMPILER_LINKER_INPUT_FILE_EXT      "ld.src")
    set(COMPILER_LINKER_OUTPUT_FILE_EXT     "ld")

else ()
    message(FATAL_ERROR  "${Red}INVALID COMPILER ... !!!${ColourReset}")

endif ()

# Setting paths for Device directories
set (DEVICE_CORE_PATH           "${DEVICE_PATH}/core/${RTSS_SUB_SYS_NAME}")
set (DEVICE_SKU_DIR             "${DEVICE_PATH}/soc/${DEVICE}")
set (DEVICE_COMMON_SRC          "${DEVICE_PATH}/core/common/source")
set (DEVICE_COMMON_INC          "${DEVICE_PATH}/core/common/include")
set (DEVICE_CORE_CONFIG_DIR     "${DEVICE_CORE_PATH}/config")
set (DEVICE_SYSTEM_INC          "${DEVICE_PATH}/system/include")
set (DEVICE_SYSTEM_SRC          "${DEVICE_PATH}/system/source")
set (DEVICE_SKU_CONFIG          "${DEVICE_SKU_DIR}/config")
set (DEVICE_SKU_RTE             "${DEVICE_SKU_DIR}/rte")
set (DEVICE_SKU_INC             "${DEVICE_SKU_DIR}/include/${RTSS_SUB_SYS_NAME}")
set (OUTPUT_DIR                 ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} CACHE INTERNAL "")
set (DEVICE_INC                 "${DEVICE_COMMON_INC};${DEVICE_CORE_CONFIG_DIR};${DEVICE_SYSTEM_INC};"
                                "${DEVICE_SKU_DIR}/include/;${DEVICE_SKU_CONFIG};${DEVICE_SKU_RTE};${DEVICE_SKU_INC}")
set (LINKER_INPUT_FILENAME      "linker_${COMPILER_LINKER_SPEC_NAME}_${boot}.${COMPILER_LINKER_INPUT_FILE_EXT}")
set (LINKER_OUTPUT_FILENAME     "linker_${COMPILER_LINKER_SPEC_NAME}_${boot}.${COMPILER_LINKER_OUTPUT_FILE_EXT}")
set (LINKER_SCRIPT_FILE         "${SCRIPT_DIR}/${LINKER_INPUT_FILENAME}")
set (CMAKE_LINKER_SCRIPT        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${LINKER_OUTPUT_FILENAME}")
set (LINKER_INC_PATH            "-I${DEVICE_SKU_CONFIG}")

# Assigning RTE components header file to a variable
include_directories (${RTE_COMP_DIR})
include_directories (${DEVICE_INC})

# Collecting Source files
file (GLOB DEVICE_SRC           "${DEVICE_COMMON_SRC}/*.c" "${DEVICE_SYSTEM_SRC}/*.c")

set (DEVICE_LIB     "DEVICE")
add_library (${DEVICE_LIB} STATIC ${DEVICE_SRC})
