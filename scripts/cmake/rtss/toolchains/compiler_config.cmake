# Set System Processor
set (CMAKE_SYSTEM_PROCESSOR                 cortex-m55)

# Skip compiler test execution
set (CMAKE_C_COMPILER_WORKS                 1)

# Cmake standards
set (CMAKE_C_STANDARD_REQUIRED              ON)
set (CMAKE_C_EXTENSIONS                     ON)
set (CMAKE_CROSSCOMPILING                   true)

# C-Compiler configurations
set (C_OPT_LEVEL                            "-O0")
set (C_DEBUG_LEVEL                          "-g")
set (C_WARNINGS_ERRORS                      "-Wall -Wextra -Werror -Wno-unused-function -Wvla  \
                                            -Wno-error=cpp -fdata-sections -ffunction-sections \
                                            -fshort-enums -funsigned-char")

# Coding Language Configurations
set (C_LANGUAGE_MODE                        "-std=c99")
set (CMAKE_C_STANDARD                       99)

# Assembler Configurations
set (ASM_DEBUG_LEVEL                        "-g3")
set (ASM_OPT_LEVEL                          "-O0")
set (ASM_WARNINGS_ERRORS                    "-Wall")
set (ASSEMLBER_ADDITIONAL_FLAGS             "-masm=auto")

if ( (COMPILER STREQUAL ARMCLANG) OR (COMPILER STREQUAL CLANG) )
    set (COMPILER_ADDITIONAL_FLAGS          "-ferror-limit=2048 -Wno-ignored-optimization-argument \
                                            -Wno-unused-command-line-argument")
    set (RETARGET_IO_SRC                    "${CMSIS_COMPILER_PATH}/source/armcc/retarget_io.c")

    if(COMPILER STREQUAL CLANG)
        set (RETARGET_IO_SRC                "${CMSIS_COMPILER_PATH}/source/clang/retarget_syscalls.c")
    endif()

elseif (COMPILER STREQUAL GCC)
    # Changing C99 to GNU11 because of asm instruction (which comes under compiler retargetting)
    set (COMPILER_ADDITIONAL_FLAGS          "-fmax-errors=2048")
    set (RETARGET_IO_SRC                    "${CMSIS_COMPILER_PATH}/source/gcc/retarget_syscalls.c")
    string(REPLACE      "-Werror"       ""      CMAKE_C_FLAGS_SPEC_FILE     "${C_WARNINGS_ERRORS}")
    set_source_files_properties("${RETARGET_IO_SRC}"     PROPERTIES      COMPILE_FLAGS    "-w")

endif()

# Checking if user has provided C-Compiler arguments or not
if (COMPILER_USER_ARG)
    set (C_COMPILER_FLAGS                   "${COMPILER_USER_ARG}")

else ()
    set (C_COMPILER_FLAGS                   "${C_LANGUAGE_MODE} ${C_OPT_LEVEL} ${C_DEBUG_LEVEL} \
                                            ${C_WARNINGS_ERRORS} ${COMPILER_ADDITIONAL_FLAGS}")

endif (COMPILER_USER_ARG)

# Checking if user has provided ASM-Compiler arguments or not
if (ASM_USER_ARGS)
    set (ASM_FLAGS                          "${ASM_USER_ARGS}")
else ()
    set (ASM_FLAGS                          "${ASM_DEBUG_LEVEL} ${ASM_OPT_LEVEL} \
                                            ${ASM_WARNINGS_ERRORS} \
                                            ${ASSEMLBER_ADDITIONAL_FLAGS}")
endif (ASM_USER_ARGS)

# Include directories wrt CMSIS pack
include_directories (${CMSIS_PACK_PATH}/CMSIS/Core/Include)
include_directories (${CMSIS_PACK_PATH}/Device/ARM/ARMCM55/Include)
include_directories (${CMSIS_PACK_PATH}/CMSIS/Driver/Include)

# Files and directories needed for Free-RTOS from CMSIS-PACK
if (OS STREQUAL FREERTOS)
    include_directories (${CMSIS_PACK_PATH}/CMSIS/RTOS2/Include)
endif ()
