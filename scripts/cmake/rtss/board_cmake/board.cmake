# Setting paths to generate board library
set(BOARD_DEPENDANT_DIR     "${ALIF_BOARD_DIR}/${BOARD_NAME}")

# Include path
if(DEFINED DEVICE_INC AND DEVICE_INC)
    include_directories (${DEVICE_INC})
else()
    message(STATUS    "${Yellow} DEVICE_INC IS NULL, SO INCLUDING ONLY BOARD INC ${ColourReset}")
endif()

include_directories ("${BOARD_DIR};${BOARD_DEPENDANT_DIR}")

set(BOARDS_SPEC_HEADERS "${BOARD_DEPENDANT_DIR}/board_defs.h;${BOARD_DEPENDANT_DIR}/gpios.h;${BOARD_DEPENDANT_DIR}/pins.h")

set(ALL_HEADERS_EXIST           TRUE)

foreach(singleHeader ${BOARDS_SPEC_HEADERS})
    if(NOT EXISTS "${singleHeader}")
        message(STATUS           "${Yellow}Missing header: ${Cyan}${singleHeader} ${ColourReset}")
        set(ALL_HEADERS_EXIST       FALSE)
    endif()
endforeach()

# Collecting Source files
if (EXISTS  "${BOARD_DIR}/board_config.c")
    set(BOARD_SRCS                  "${BOARD_DIR}/board_config.c")
    #file (GLOB BOARD_SRCS           "${BOARD_DIR}/*.c" "${BOARD_DIR_1}/*.c")
    set(ALL_BOARD_SRC_EXIST         TRUE)
endif()

if(ALL_HEADERS_EXIST AND ALL_BOARD_SRC_EXIST)
    set (BOARD_LIB              "BOARD")
    add_library (${BOARD_LIB}   STATIC  ${BOARD_SRCS})
else()
    message(STATUS "⚠️ ${Yellow} Some headers or sources files are missing.${Cyan}SO BOARD-LIB SUPPORT NOT ENABLED \n${ColourReset}")
    set (BOARD_LIB              "")
    message(STATUS      "HEADERS :\n ${BOARDS_SPEC_HEADERS} \n SRC:\n ${BOARD_SRCS}\n")
endif()