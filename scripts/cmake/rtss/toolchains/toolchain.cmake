# Macro Definitions
add_definitions (-D_RTE_)
add_definitions (-D${CPU}_${RTSS})
add_definitions (-D${DEVICE})
add_definitions (-DRTSS_${RTSS})

set (CPU_NAME                           ${CMAKE_SYSTEM_PROCESSOR})
set (FLOAT_ABI                          hard)
set (CMAKE_SYSTEM_NAME                  Generic)
set (ARCH                               armv8.1-m.main)
set (FPU                                fpv5-sp-d16)
set (HW_FLAGS                           "-mcpu=${CPU_NAME} -mfloat-abi=${FLOAT_ABI} -mthumb -mlittle-endian")

if (COMPILER STREQUAL ARMCLANG)
    # specify the cross compiler
    set (CMAKE_C_COMPILER               armclang)
    set (CMAKE_CXX_COMPILER             armclang)
    set (CMAKE_C_LINKER_PREFERENCE      armlink)
    set (CMAKE_ASM_LINKER_PREFERENCE    armlink)
    set (CMAKE_ASM_COMPILER             armclang)
    set (CMAKE_ASM_COMPILER_AR          armar)

    set (COMPILER_VERSION               6.18)
    set (CPU_COMPILE_DEF                CPU_CORTEX_M55)     # Flags for cortex-m55
    set (TARGET_NAME                    arm-arm-none-eabi)

    # Flags for C and Assembly source files
    set(CMAKE_C_FLAGS                   "${C_COMPILER_FLAGS} ${HW_FLAGS} --target=${TARGET_NAME}  -mfpu=${FPU} -xc  -MD -MP")
    set(CMAKE_CXX_FLAGS                 "${CMAKE_C_FLAGS} -fno-exceptions  -fno-rtti")
    set(CMAKE_ASM_FLAGS                 "${ASM_FLAGS} ${HW_FLAGS} --target=${TARGET_NAME} -x assembler-with-cpp")
    add_compile_options(-fcolor-diagnostics)

    # Tell linker that reset interrupt handler is our entry point
    add_link_options(--map --entry=Reset_Handler --diag_suppress 6312,6314)

    # Link Options
    add_link_options(--info=sizes   --scatter=${CMAKE_LINKER_SCRIPT})

elseif(COMPILER STREQUAL GCC)
    # specify the cross compiler
    set (CMAKE_C_COMPILER               arm-none-eabi-gcc)
    set (CMAKE_CXX_COMPILER             arm-none-eabi-g++)
    set (CMAKE_C_LINKER_PREFERENCE      arm-none-eabi-ld)
    set (CMAKE_ASM_LINKER_PREFERENCE    arm-none-eabi-ld)
    set (CMAKE_ASM_COMPILER             arm-none-eabi-gcc)
    set (CMAKE_ASM_COMPILER_AR          arm-none-eabi-ar)

    # Flags for C and Assembly source files
    set (CMAKE_C_FLAGS                  "${C_COMPILER_FLAGS} ${HW_FLAGS} -fdata-sections")
    set (CMAKE_CXX_FLAGS                "${CMAKE_C_FLAGS} -fno-exceptions  -fno-rtti")
    set (CMAKE_ASM_FLAGS                "${ASM_FLAGS} ${HW_FLAGS} -fdata-sections ")

    # Tell linker that reset interrupt handler is our entry point
    add_link_options(-Wl,--gc-sections)
    add_link_options(-Wl,--no-warn-rwx-segment)

    # Link Options
    add_link_options(
        ${HW_FLAGS}
        -march=${ARCH}
        -mfpu=${FPU}
        -Xlinker
        -print-memory-usage
        -T${CMAKE_LINKER_SCRIPT})

    add_compile_options(-fdiagnostics-color=always)

    if(NOT (${RETARGET_EN_NO_SEMIHOSTING}))
        add_link_options(--specs=rdimon.specs)
    else()
        add_link_options(--specs=nosys.specs)
    endif()

elseif(COMPILER STREQUAL CLANG)
    # specify the cross compiler
    set (CMAKE_C_COMPILER               clang)
    set (CMAKE_CXX_COMPILER             clang)
    set (CMAKE_C_LINKER_PREFERENCE      lld)
    set (CMAKE_ASM_LINKER_PREFERENCE    lld)
    set (CMAKE_ASM_COMPILER             clang)
    set (CMAKE_ASM_COMPILER_AR          llvm-ar)

    set (COMPILER_VERSION               18.0.0)
    set (TARGET_NAME                    arm-none-eabi)

    # Flags for C and Assembly source files
    set(CMAKE_C_FLAGS                   "${C_COMPILER_FLAGS} ${HW_FLAGS} -Wno-int-conversion --target=${TARGET_NAME}")
    set(CMAKE_CXX_FLAGS                 "${CMAKE_C_FLAGS} -fno-exceptions  -fno-rtti")
    set(CMAKE_ASM_FLAGS                 "${ASM_FLAGS} ${HW_FLAGS} --target=${TARGET_NAME}")
    add_compile_options(-fdiagnostics-color=always)

    if(${RETARGET_EN_NO_SEMIHOSTING})
        set(CRT_LIBS                    -lcrt0)
    else()
        set(CRT_LIBS                    -lcrt0-semihost  -lsemihost)
    endif()

    # Tell linker that reset interrupt handler is our entry point
    add_link_options(
        -Wl,--print-memory-usage
        ${CRT_LIBS}
        -mthumb
        -mfloat-abi=${FLOAT_ABI}
        -mlittle-endian
        -e Reset_Handler
        -T${CMAKE_LINKER_SCRIPT}
    )
endif()
