# MACRO DEF_BOOL_VAR_BASED_ON_MACRO to read macro value (integer value 0-9) and if it is non
#       zero value it will define another boolean variable which cmake build can be used
# argv[0] - header file content
# argv[1] - marco name which will be searched
# argv[2] - another variable of boolean type which will be ON or OFF based on 2nd argument
# argv[3] - description of 3rd argument
macro (DEF_BOOL_VAR_BASED_ON_MACRO      header      macro_name      macro_val   varMsg)

    FOREACH(arg ${header})
        string(REGEX MATCHALL "^[ \t]*#(define|DEFINE)[ \t]+${macro_name}[ \t]+[0-9]+[ \t]*.*" foundDefines "${arg}")

        if (foundDefines)
            string(REGEX MATCH "[ \t]+[0-9]+" tmp "${foundDefines}")
            string(REGEX MATCH "[0-9]+" tmp "${tmp}")

            if(${tmp})
                set(${macro_val}    ON      CACHE   BOOL    ${varMsg})
                #add_definitions(-D${macro_name})
                break()
            endif()

        endif (foundDefines)
    endforeach()

    if(NOT DEFINED ${macro_val})
        set(${macro_val}    OFF     CACHE   BOOL    ${varMsg})
        add_definitions(-U${macro_name})
    endif()

endmacro ()

# MACRO DEF_BOOL_VAR_BASED_ON_DEF_MACRO_ONLY will used to check if macro is defined or not and
#       if it is defined it will define another boolean variable which cmake build can be used
# argv[0] - header file content
# argv[1] - marco name which will be searched
# argv[2] - another variable of boolean type which will be ON or OFF based on 2nd argument
# argv[3] - description of 3rd argument
macro (DEF_BOOL_VAR_BASED_ON_DEF_MACRO_ONLY      header      macro_name      macro_val   varMsg)

    FOREACH(arg ${header})
        string(REGEX MATCHALL "^[ \t]*#(define|DEFINE)[ \t]+${macro_name}[ \t]+.*" foundDefines "${arg}")

        if (foundDefines)
            set(${macro_val}    ON      CACHE   BOOL    ${varMsg})
            break()
        endif (foundDefines)
    endforeach()

    if(NOT DEFINED ${macro_val})
        set(${macro_val}    OFF     CACHE   BOOL    ${varMsg})
        add_definitions(-U${macro_name})
    endif()

endmacro ()

# MACRO GET_MACRO_VALUE to get macro value (integer value 0-9)
# argv[0] - header file name with path
# argv[1] - marco name which will be searched
# argv[2] - macro value (integer 0-9) which will read for given macro name
macro (GET_MACRO_VALUE    headerFileName      macro_name      macro_val)

    file (STRINGS ${headerFileName}  file_content)
    FOREACH(arg ${file_content})
        string(REGEX MATCHALL "^[ \t]*#(define|DEFINE)[ \t]+${macro_name}[ \t]+[0-9]+[ \t]*.*" foundDefines "${arg}")

        if (foundDefines)
            string(REGEX MATCH "[ \t]+[0-9]+" tmp "${foundDefines}")
            string(REGEX MATCH "[0-9]+" tmp "${tmp}")

            set(${macro_val}     ${tmp})
        endif (foundDefines)
    endforeach()

endmacro ()

# MACRO GET_MACRO_STR_VALUE to get macro value (integer value 0-9)
# argv[0] - header file name with path
# argv[1] - marco name which will be searched
# argv[2] - macro value (some string) which will read for given macro name
macro (GET_MACRO_STR_VALUE    headerFileName      macro_name      macro_val)

    file (STRINGS ${headerFileName}  file_content)

    FOREACH(arg ${file_content})
        string(REGEX MATCHALL "^[ \t]*#(define|DEFINE)[ \t]+${macro_name}[ \t]+[A-Za-z_][A-Za-z0-9_]*[ \t]*.*" foundDefines "${arg}")

        if (foundDefines)
            string(REGEX MATCH "^[ \t]*#(define|DEFINE)[ \t]+${macro_name}[ \t]+([A-Za-z_][A-Za-z0-9_]*)" _ "${foundDefines}")
            set(${macro_val}     ${CMAKE_MATCH_2})
        endif (foundDefines)
    endforeach()

endmacro ()

# MACRO CHECK_MACRO_DEF to see macro is defined or not
# argv[0] - header file content
# argv[1] - marco name which will be searched
# argv[2] - flag : TRUE if macro enable/exit FLASE if not exist
macro (CHECK_MACRO_DEF    file_content      macro_name      macroExist)

    set(${macroExist}           OFF)
    FOREACH(arg     ${file_content})
        string(REGEX MATCHALL "^[ \t]*#(define|DEFINE)[ \t]+${macro_name}[ \t]*.*" foundDefines "${arg}")

        if (foundDefines)
            set(${macroExist}     ON)
        endif (foundDefines)
    endforeach()

endmacro ()

# MACRO CHANGE_MACRO_VAL to read macro value (integer value 0-9) and change that macro value
# argv[0] - marco name which will be searched
# argv[1] - header file name with path
# argv[2] - change value which will be updated if that macro is found
# argv[3] - return value. If value is updated it will return 1 otherwise 0
# argv[4] - flag which will used to exact match or tolerance of space/tabs.
macro (CHANGE_MACRO_VAL     macro_name   header_file     change_val    return_val    exactStr)
    set (${return_val}              0)
    file (READ      "${header_file}"     header_content)

    if(${exactStr})
        set(matchCriteria       "${macro_name}")
    else()
        set(matchCriteria    "[ |\\t]*#(define|DEFINE)+[ |\\t]+${macro_name}[ |\\t]+")
    endif()

    # The string should be exactly as present in the file, including the spaces/tabs
    string(REGEX MATCH     ${matchCriteria}     macrodef    "${header_content}")

    # Check if the match was successful
    if(macrodef)
        if(${exactStr})
            string (REPLACE "${macrodef}" "${change_val}" changed_header_content "${header_content}")
        else()
            string (REPLACE "${macrodef}" "${macrodef}${change_val}" changed_header_content "${header_content}")
        endif()

        if(NOT "${changed_header_content}" STREQUAL "${macro_name}")
            file(WRITE      "${header_file}"        "${changed_header_content}")
            set (${return_val}              1)
        endif()
    endif()

endmacro ()

# MACRO CHANGE_SPEC_MACRO_VAL to read and change the macro value(special which contain ())
# argv[0] - 1st parameter
# argv[1] - 2nd parameter
# argv[2] - 3rd parameter
# argv[3] - header file name with path
# argv[4] - change value which will be updated if that macro is found
# argv[5] - return value. If value is updated it will return 1 otherwise 0
macro (CHANGE_SPEC_MACRO_VAL    prm1  prm2  prm3  fileNameWithPath    change_val    return_val)
    set(${return_val}              0)

    file(READ      "${fileNameWithPath}"     file_content)
    set(matchCriteria    "[ |\\t]*#(define|DEFINE)+[ |\\t]+${prm1}[ |\\t]+\\(\\(${prm2}\\)${prm3}\\)")

    # The string should be exactly as present in the file, including the spaces/tabs
    string(REGEX MATCHALL  ${matchCriteria}  macrodef    "${file_content}")

    # Check if the match was successful
    if(macrodef)
        string (REPLACE "${macrodef}" "${change_val}" changed_header_content "${file_content}")
        if(NOT "${changed_header_content}" STREQUAL "${check_def}")
            file(WRITE      "${fileNameWithPath}"        "${changed_header_content}")
            set (${return_val}              1)
        endif()
    endif()

endmacro ()

# MACRO SAVE_ORIG_FILE to create backup file of original file
# argv[0] - header file name with path which need to modify
# argv[1] - back-up file name with path
# argv[2] - return value. If changed it will return 1 otherwise 0.
macro (SAVE_ORIG_FILE   fileNameWithPath    newFileNameWithPath     return_val)
    set(${return_val}              0)
    file(READ  ${fileNameWithPath}   file_content)

    get_filename_component(filePath     "${fileNameWithPath}"   DIRECTORY)
    get_filename_component(fileName     "${fileNameWithPath}"   NAME)

    # FileName
    string(REPLACE      "."     "_"     newFileName    "${fileName}")
    set(${newFileNameWithPath}      "${filePath}/${newFileName}.bak")

    if( NOT EXISTS ${${newFileNameWithPath}})
        file(WRITE  ${${newFileNameWithPath}}   "${file_content}")
        set(${return_val}              1)
    endif()

endmacro ()

# MACRO GET_MACRO_DEF_CNT will get count of occuration of given macro in header file
#                    (excluding comments i.e. // #define MARCO /* #define MACRO will be not counted,
#                    only #define MACRO will be counted
# argv[0] - header file name with path
# argv[1] - macro name which will be searched in given header file
# argv[2] - it will return number number of times it occurs in given header file excluding comments
macro (GET_MACRO_DEF_CNT     header     macro_name  macro_def_cnt)
    set(tmp    0)
    FOREACH(arg ${header})
        string(REGEX MATCHALL   "^[ \t]*#(define|DEFINE)[ \t]*([^a-zA-Z0-9_]|^)${macro_name}([^a-zA-Z0-9_]|$)[ \t]*"  foundDefines    "${arg}")

        if (foundDefines)
            MATH(EXPR tmp     "${tmp}+1")
        endif (foundDefines)
    endforeach()
    set(${macro_def_cnt} ${tmp})

endmacro ()

# Macro definition to Compile w.r.t OS types
macro (BUILD_PROJECT)

    set(EXECUTABLE ${testname})

    # Adding executable file name
    add_executable (${EXECUTABLE} ${testsourcefile}  ${addonsourcefiles})

    if (COMPILER STREQUAL GCC)
        target_link_options(${EXECUTABLE} PRIVATE -Wl,--whole-archive -Wl,--start-group)
        target_link_libraries(${EXECUTABLE} PRIVATE   -Wl,--whole-archive    PRIVATE     ${COMMON_LIB}   -Wl,--no-whole-archive)
    endif()

    if (OS STREQUAL FREERTOS)

        # Linking all the library files to the test application
        target_link_libraries(${EXECUTABLE} PRIVATE ${BOARD_LIB} ${SE_HOST_SERVICES_LIB} ${DRIVER_LIB} ${DEVICE_LIB} ${OS_LIB_FILE})

    elseif (OS STREQUAL THREADX)

        # Linking all the library files to the test application
        target_link_libraries(${EXECUTABLE} PRIVATE ${BOARD_LIB} ${SE_HOST_SERVICES_LIB} ${DRIVER_LIB} ${DEVICE_LIB} ${OS_LIB_FILE})

    elseif (OS STREQUAL NONE)

        # Linking all the library files to the test application
        target_link_libraries(${EXECUTABLE} PRIVATE ${BOARD_LIB} ${SE_HOST_SERVICES_LIB} ${DRIVER_LIB} ${DEVICE_LIB})

    endif ()

    # Improve clean target
    set_target_properties(${EXECUTABLE} PROPERTIES ADDITIONAL_CLEAN_FILES
        "${OUTPUT_DIR}/${EXECUTABLE}.bin;${OUTPUT_DIR}/${EXECUTABLE}.hex;${OUTPUT_DIR}/${EXECUTABLE}.map")

    if (COMPILER STREQUAL GCC)

        target_link_options(${EXECUTABLE} PRIVATE  -Wl,-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${EXECUTABLE}.map)
        target_link_libraries(${EXECUTABLE} PRIVATE m)
        target_link_options(${EXECUTABLE} PRIVATE -Wl,--end-group -Wl,--no-whole-archive)
        set_target_properties(${EXECUTABLE} PROPERTIES OUTPUT_NAME ${EXECUTABLE}.elf)

        add_custom_command(TARGET  ${EXECUTABLE}
           POST_BUILD
           COMMAND echo "${EXECUTABLE}" >> "${TMP_FILE1}"
            && echo  "${Cyan} ${EXECUTABLE}.elf  BUILD SUCCESSFUL *******************************${ColourReset}"
            && arm-none-eabi-objcopy -O binary     ${OUTPUT_DIR}/${EXECUTABLE}.elf  ${OUTPUT_DIR}/${EXECUTABLE}.bin
            && echo  "${Green}Generated Bin files for ${Yellow}${EXECUTABLE}   ${ColourReset}"
           COMMAND  arm-none-eabi-objcopy -O ihex       ${OUTPUT_DIR}/${EXECUTABLE}.elf  ${OUTPUT_DIR}/${EXECUTABLE}.hex
            &&  echo  "${Green}Generated Hex files for ${Yellow}${EXECUTABLE}   ${ColourReset}"
        )

    elseif (COMPILER STREQUAL ARMCLANG)

        target_link_options(${EXECUTABLE} PRIVATE  --list=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${EXECUTABLE}.map)
        target_link_libraries(${EXECUTABLE} PRIVATE ${COMMON_LIB})

        add_custom_command(TARGET  ${EXECUTABLE}
           POST_BUILD
           COMMAND  echo "${EXECUTABLE}" >> "${TMP_FILE1}"
           && echo  "${Cyan} ${EXECUTABLE}.elf  BUILD SUCCESSFUL *******************************${ColourReset}"
           && fromelf -v --bin --output=${OUTPUT_DIR}/${EXECUTABLE}.bin  ${OUTPUT_DIR}/${EXECUTABLE}.elf
           && echo  "${Green}Generated Bin files for ${Yellow}${EXECUTABLE}.elf   ${ColourReset}"
           COMMAND  fromelf -v --i32 --output=${OUTPUT_DIR}/${EXECUTABLE}.hex  ${OUTPUT_DIR}/${EXECUTABLE}.elf
           && echo  "${Green}Generated Hex files for ${Yellow}${EXECUTABLE}.elf   ${ColourReset}"
        )

    elseif (COMPILER STREQUAL CLANG)

        target_link_options(${EXECUTABLE} PRIVATE   -Wl,-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${EXECUTABLE}.map)
        target_link_libraries(${EXECUTABLE} PRIVATE ${COMMON_LIB})
        set_target_properties(${EXECUTABLE}     PROPERTIES      OUTPUT_NAME     ${EXECUTABLE}.elf)

        add_custom_command(TARGET  ${EXECUTABLE}
           POST_BUILD
           COMMAND  echo "${EXECUTABLE}" >> "${TMP_FILE1}"
           && echo  "${Cyan} ${EXECUTABLE}.elf  BUILD SUCCESSFUL *******************************${ColourReset}"
           && llvm-objcopy --output-target binary  ${OUTPUT_DIR}/${EXECUTABLE}.elf  ${OUTPUT_DIR}/${EXECUTABLE}.bin
           && echo  "${Green}Generated Bin files for ${Yellow}${EXECUTABLE}.elf   ${ColourReset}"
           COMMAND  llvm-objcopy --output-target ihex  ${OUTPUT_DIR}/${EXECUTABLE}.elf  ${OUTPUT_DIR}/${EXECUTABLE}.hex
         )

    endif ()

    IF (EXISTS "${SE_TOOLS_IMAGE_DIR}")
        add_custom_command(TARGET  ${EXECUTABLE}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${OUTPUT_DIR}/${EXECUTABLE}.bin" "${SE_TOOLS_IMAGE_DIR}/${EXECUTABLE}_${CPU}_${RTSS}.bin"  &&
            echo "${Cyan}Copied binary ${EXECUTABLE}_${CPU}_${RTSS}.bin ${ColourReset} to ${Green} ${SE_TOOLS_IMAGE_DIR} ${ColourReset}"
        )
    endif ()

    #To Support Package Preset
    install(TARGETS ${EXECUTABLE} DESTINATION bin)

    # Set global property (all targets are impacted)
    #set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
    #set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK    "${CMAKE_COMMAND} -E time")

    # Set property for my_target only
    #set_property(TARGET ${EXECUTABLE} PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")

    # get_target_property(INCLUDE_DIRS ${EXECUTABLE} INCLUDE_DIRECTORIES)
    # message(STATUS "Include Directories for my_target: ${INCLUDE_DIRS}")

    #get_target_property(LIBS ${EXECUTABLE} LINK_LIBRARIES)
    #message(DEBUG       "\n${EXECUTABLE} Linked Libraries: ${LIBS}\n")

endmacro (BUILD_PROJECT)

# MACRO GET_GIT_PARAMS will get git parameters
# argv[0] - repo name with path
# argv[1] - branch Name
# argv[2] - message type i.e. VERBOSE or FATAL_ERROR or etc.
macro (GET_GIT_PARAMS repoPath branchName messageType )

    string(REGEX REPLACE "[ \t]+$" "" my_repo_withpath ${repoPath})
    execute_process(
        COMMAND git -C ${my_repo_withpath} rev-parse --abbrev-ref HEAD
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

   execute_process(
        COMMAND git -C ${my_repo_withpath} rev-parse --short HEAD
        OUTPUT_VARIABLE GIT_SHA_ID
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    string(LENGTH ${my_repo_withpath} repoPath_length)
    math(EXPR repoPath_length "${repoPath_length} - 1")
    string(SUBSTRING ${my_repo_withpath} ${repoPath_length} 1 repoPath_last_char)

    if( repoPath_last_char STREQUAL "/")
        string(SUBSTRING ${my_repo_withpath} 0 ${repoPath_length} my_repo_withpath)
    endif()

    string(FIND ${my_repo_withpath} "/" last_slash REVERSE)
    math(EXPR last_slash "+ ${last_slash} + 1")
    string(SUBSTRING ${my_repo_withpath} ${last_slash} -1 repoName)

    if (${branchName} STREQUAL ${GIT_BRANCH})
        message(VERBOSE "Branch (${repoName}) Check : ${Cyan} PASS ${ColourReset}")
    else ()
        if (${branchName} STREQUAL ${GIT_SHA_ID})
            message(VERBOSE "Branch (${repoName}) Check : ${Cyan} PASS ${ColourReset}")
        else()
            message(${messageType} "Branch (${repoName}) Check : ${Red} FAIL \n(checkout proper branch as per A/B series device)${ColourReset}")
        endif()
    endif ()

endmacro ()

# MACRO RM_ENTRY will define a macro to move an item from one list to another
# argv[0] - src_list i.e source list
# argv[1] - dst_list i.e destination list
# argv[2] - element name which will be removed from source list and have in destination list
# argv[3] - flag to decide removed item will be added in destination list or not
macro(RM_ENTRY      src_list dst_list       elementName_to_remove       update_dst_flag)
    get_filename_component(F_NAME   ${elementName_to_remove}    NAME_WE)
    list(REMOVE_ITEM    ${src_list}     ${elementName_to_remove})
    if(${update_dst_flag})
        list(APPEND         ${dst_list}     ${F_NAME})
    endif()
endmacro()

# MACRO ERR_CHECK_MSG will custom error check function
# argv[0] - message type i.e. VERBOSE or FATAL_ERROR or etc.
# argv[1] - variable which needs to check whether exist or not
# argv[2] - msg_key which will be used to message identifier i.e [ERROR] or [WARNING] or etc
# argv[3] - msg_color which will be used to print message in that color
macro(ERR_CHECK_MSG     msg_type    var_to_check   msg_key  msg_color)
    if(NOT ${var_to_check})
        message(${msg_type}     "${${msg_color}}${msg_key} IS NOT ASSIGNED TO ANY VALUE...${ColourReset}")
    elseif (NOT EXISTS ${${var_to_check}})
        message(${msg_type}     "⛔${${msg_color}}GIVEN ${msg_key} PATH IS NOT AVAILABLE...${ColourReset}")
    endif()
endmacro()

# MACRO CONVERT_PRESET_VAR_TO_CMAKE_VAR will convert preset cache/enviroment into cmake variable function
# argv[0] - presetVar is preset/enviroment variable
# argv[1] - cmakeVar is variable which cmake build system will use
# argv[2] - presetVarType describe the variable is preset or enviroment type
macro(CONVERT_PRESET_VAR_TO_CMAKE_VAR     presetVar    cmakeVar   presetVarType)
    if(DEFINED ${presetVarType}{${presetVar}})
        set(${cmakeVar}         $${presetVarType}{${presetVar}})
    else()
        message(STATUS        "${Yellow}⚠️[WARNING] ${presetVar} is not defined in preset ${ColourReset}")
    endif()
endmacro()

# MACRO COND_FILE_ADD will add files in variable conditionally and prepare statistics
# argv[0] - fileNameWithPath file name with path
# argv[1] - cond if this condition is true, given file will be added
# argv[2] - cmake variable which will be appended with given file if condition is true.
# argv[3] - type of given file i.e "test-apps" or "dependency".
macro(COND_FILE_ADD     fileNameWithPath    cond    testApp   typeOfFile)
    #message(STATUS        "${Yellow}⚠️[WARNING] ${fileNameWithPath} is not defined in preset ${${cond}} ${ColourReset}")

    if(EXISTS ${fileNameWithPath})
        if( ${typeOfFile} STREQUAL "test-apps")
            list(APPEND     ALL_TEST_APPS_SRC_LIST      "${fileNameWithPath}")
        elseif(${typeOfFile} STREQUAL "dependency")
            list(APPEND     TEST_APPS_DEPENDENCY_SRC_LIST      "${fileNameWithPath}")
        else()
            message(STATUS      "${Yellow}⚠️[WARNING] File type is not given for given file ${fileNameWithPath}" )
        endif()

        if(${cond})
            list(APPEND     ${testApp}      "${fileNameWithPath}")
        else()
            #message(STATUS        "${Yellow}⚠️[WARNING] ${cond} No proper flags are enabled for file ${fileNameWithPath} ${ColourReset}")
        endif()
    else()
        message(STATUS        "${Yellow}⚠️[WARNING] ${fileNameWithPath} doesn't exit ${ColourReset}")
    endif()

endmacro()

# FUNCTION get_rte_macros will read all defined macros in given file i.e. RTE_components.h
function(get_rte_macros)
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MHU         ENABLE_MHU          "Enable/disable MHU Driver.")

    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_LL_PINCONF  ENABLE_PIN_CONF     "Enable/disable PinPAD and PinMux Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_USART       ENABLE_USART        "Enable/disable USART Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_IO          ENABLE_IO           "Enable/disable IO Driver.")

    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_ADC                        ENABLE_ADC       "Enable/disable ADC Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CANFD                      ENABLE_CANFD     "Enable/disable CANFD commands.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CDC200                     ENABLE_CDC200    "Enable/disable CDC200 Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MIPI_DSI_ILI9806E_PANEL    ENABLE_MIPI_DSI_ILI9806E_PANEL   "Enable/disable MIPI DSI ILI9806E_PANEL Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CDC_ILI6122_PANEL          ENABLE_CDC_ILI6122E_PANEL        "Enable/disable CDC ILI6122_PANEL Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MIPI_DSI_ILI9488_PANEL     ENABLE_MIPI_DSI_ILI9488E_PANEL   "Enable/disable MIPI DSI ILI9488E_PANEL Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CMP             ENABLE_CMP          "Enable/disable Comparator Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CRC             ENABLE_CRC          "Enable/disable CRC Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_DAC             ENABLE_DAC          "Enable/disable DAC Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_SD              ENABLE_SD           "Enable/disable SD Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_OSPI            ENABLE_OSPI         "Enable/disable OSPI Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_XIP_HYPERRAM            ENABLE_XIP_HYPERRAM             "Enable/disable XIP Hyper RAM Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_ISSI_FLASH_XIP_CORE     ENABLE_ISSI_FLASH_XIP_CORE      "Enable/disable ISSI FLASH XIP CORE Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_ISSI_FLASH_XIP_UTILITY  ENABLE_ISSI_FLASH_XIP_UTILITY   "Enable/disable ISSI FLASH XIP UTILITY Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_ISSI_FLASH      ENABLE_ISSI_FLASH   "Enable/disable ISSI FLASH Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_GT911           ENABLE_GT911        "Enable/disable GT911 Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_HWSEM           ENABLE_HWSEM        "Enable/disable Hardware Semaphores Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_SAI             ENABLE_I2S          "Enable/disable I2S Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_I3C             ENABLE_I3C          "Enable/disable I3C Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_ICM42670P       ENABLE_ICM42670P    "Enable/disable ICM42670P Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_I2C_I3C         ENABLE_I2C_I2C      "Enable/disable I3C_I2C Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_LPI2C           ENABLE_LPI2C        "Enable/disable LPI2C Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_PDM             ENABLE_PDM          "Enable/disable PDM Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_SPI             ENABLE_SPI          "Enable/disable SPI Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_LPTIMER         ENABLE_LPTIMER      "Enable/disable LPTIMER Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MRAM            ENABLE_MRAM         "Enable/disable MRAM Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_RTC             ENABLE_RTC          "Enable/disable RTC Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_UTIMER          ENABLE_UTIMER       "Enable/disable UTIMER Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_WDT             ENABLE_WDT          "Enable/disable WDT Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_WIFI            ENABLE_WIFI         "Enable/disable WIFI Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_DMA             ENABLE_DMA          "Enable/disable DMA Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_I2C             ENABLE_I2C          "Enable/disable I2C Driver.")

    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_MT9M114       ENABLE_MT9M114_CAMERA       "Enable/disable MT9M114 Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_HM0360        ENABLE_HM0360_CAMERA        "Enable/disable HM0360  Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_ARX3A0        ENABLE_ARX3A0_CAMERA        "Enable/disable ARX3A0  Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_AR0144        ENABLE_AR0144_CAMERA        "Enable/disable AR0144  Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_AR0145        ENABLE_AR0145_CAMERA        "Enable/disable AR0145  Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_AR0246        ENABLE_AR0246_CAMERA        "Enable/disable AR0246  Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_OV5647        ENABLE_OV5647_CAMERA        "Enable/disable OV5647  Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CAMERA_SENSOR_OV5675        ENABLE_OV5675_CAMERA        "Enable/disable OV5675  Camera Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MIPI_CSI2                   ENABLE_MIPI_CSI2            "Enable/disable MIPI CSI2 Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MIPI_DSI                    ENABLE_MIPI_DSI             "Enable/disable MIPI DSI Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MIPI_DSI_CSI2_DPHY          ENABLE_MIPI_DSI_CSI2_DPHY   "Enable/disable MIPI DSI Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_PHY_GENERIC                 ENABLE_PHY_GENERIC          "Enable/disable MIPI DSI Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_ETH_MAC                     ENABLE_ETH_MAC              "Enable/disable ETH MAC Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_CPI                         ENABLE_CPI                  "Enable/disable CPI Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_GPIO                        ENABLE_GPIO                 "Enable/disable GPIO Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_WM8904_CODEC                ENABLE_WM8904               "Enable/disable WM8904 Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_BMI323                      ENABLE_BMI323               "Enable/disable BMI323 Driver.")
    DEF_BOOL_VAR_BASED_ON_MACRO("${RTEcomponentFile}"   RTE_Drivers_MCI                         ENABLE_MCI                 "Enable/disable MCI Driver.")

    DEF_BOOL_VAR_BASED_ON_DEF_MACRO_ONLY("${RTEcomponentFile}"   RTE_CMSIS_Compiler_STDIN    ENABLE_STDIN    "Enable/disable retarget STDIN  Driver.")
    DEF_BOOL_VAR_BASED_ON_DEF_MACRO_ONLY("${RTEcomponentFile}"   RTE_CMSIS_Compiler_STDOUT   ENABLE_STDOUT   "Enable/disable retarget STDOUT Driver.")
    DEF_BOOL_VAR_BASED_ON_DEF_MACRO_ONLY("${RTEcomponentFile}"   RTE_CMSIS_Compiler_STDERR   ENABLE_STDERR   "Enable/disable retarget STDERR Driver.")

    if(${ENABLE_STDIN})
        CHECK_MACRO_DEF("${RTEcomponentFile}"   RTE_CMSIS_Compiler_STDIN   STDIN_MACRO_DEFINED)

        if(NOT ${STDIN_MACRO_DEFINED})
            add_definitions(-DRTE_CMSIS_Compiler_STDIN)
        endif()
        add_definitions(-DRTE_CMSIS_Compiler_STDIN_Custom)
    endif()

    if(${ENABLE_STDOUT})
        CHECK_MACRO_DEF("${RTEcomponentFile}"   RTE_CMSIS_Compiler_STDOUT   STDOUT_MACRO_DEFINED)

        if(NOT ${STDOUT_MACRO_DEFINED})
            add_definitions(-DRTE_CMSIS_Compiler_STDOUT)
        endif()
        add_definitions(-DRTE_CMSIS_Compiler_STDOUT_Custom)
    endif()

    if(${ENABLE_STDERR})
        CHECK_MACRO_DEF("${RTEcomponentFile}"   RTE_CMSIS_Compiler_STDERR   STDERR_MACRO_DEFINED)

        if(NOT ${STDERR_MACRO_DEFINED})
            add_definitions(-DRTE_CMSIS_Compiler_STDERR)
        endif()
        add_definitions(-DRTE_CMSIS_Compiler_STDERR_Custom)
    endif()

endfunction()

# MACRO RESOLVE_TESTAPP_DEPENDANCY will remove or handle all dependency for given testapp and macro
# argv[0] - testAppNameWithPath testapp file name with path
# argv[1] - macro_name which will be validated and add given testapp with needed macros
macro(RESOLVE_TESTAPP_DEPENDANCY testAppNameWithPath macro_name)

    set(${macro_name}    OFF  PARENT_SCOPE)

    if(EXISTS ${testAppNameWithPath})
        get_filename_component(TEST_APP_FILENAME "${testAppNameWithPath}" NAME_WE)
        # Find position of "SUBSTRING"
        string(FIND     "${TEST_APP}"   ${TEST_APP_FILENAME}  POSITION)
        string(FIND     "${TEST_APP}"   "ALL"           ALL_STR_POSITION)

        if (( "${TEST_APP}" STREQUAL "ALL") OR ( "${TEST_APP}" STREQUAL "${TEST_APP_FILENAME}") OR ( (NOT (POSITION GREATER -1)) AND (ALL_STR_POSITION GREATER -1)))
            set(${macro_name}    ON      PARENT_SCOPE)
        endif()
    endif()
endmacro()

# FUNCTION resolve_dependancy will remove or handle all dependency
function(resolve_dependancy)
    if(NOT ${ENABLE_UTIMER})
        RESOLVE_TESTAPP_DEPENDANCY(${BARE_METAL_APP_DIR}/demo_adc_exttrigger.c    ENABLE_UTIMER)
    endif()
endfunction()

# FUNCTION eval_flags will evaluate all given flags based on given operation
# argv[0] - final_res it will contain final output/flag after performing all operations
# argv[1] - operation it will be logical operation given by user (AND/OR/NOR etc)
# Note: It will read 'n' number of flags so no restrication of number of flags. any number of flags can be given
# Limitation: Only one type of operation can be performed. For 2nd type of operation need to call again this function
function(eval_flags final_res operation)

    set(res             ON)

    if("${operation}" STREQUAL "OR")
        set(res         OFF)
    endif()

    foreach(flag IN LISTS ARGN)
        if(NOT DEFINED ${flag})
            message(STATUS         "${Yellow}⚠️[WARNING]  Flag ${flag} is not defined. Assuming OFF ${ColourReset}")
            set(${flag}         OFF)
        endif()

        if("${operation}"   STREQUAL    "AND"   AND     NOT ${flag})
            set(res             OFF)
            break()
        elseif("${operation}"   STREQUAL    "OR"    AND     ${flag})
            set(res             ON)
            break()
        endif()
    endforeach()

    set(${final_res}        ${res}          PARENT_SCOPE)

endfunction()
