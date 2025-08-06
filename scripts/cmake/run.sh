#!/bin/bash

# Number of Arguments passed
ARGUMENTS_PASSED=$#

NC='\033[0m' # No Color
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
BROWN='\033[0;37m'
HIGH_CYAN='\033[38;5;45m'
HIGH_YELLOW='\033[38;5;11m'

total_start_time=$SECONDS
final_result=0

set -e

# Get all configure presets
EXISTING_CONFIGURE_PRESETS=$(cmake --list-presets=configure |  grep -oP '"\K[^"]+(?=")')

# Get all Build presets
EXISTING_BUILD_PRESETS=$(cmake --list-presets=build | grep -oP '"\K[^"]+(?=")')

# Get all Test presets
EXISTING_TEST_PRESETS=$(cmake --list-presets=test | grep -oP '"\K[^"]+(?=")')

# Get all configure presets
EXISTING_WORKFLOW_PRESETS=$(cmake --list-presets=workflow |  grep -oP '"\K[^"]+(?=")')

if [ $ARGUMENTS_PASSED -gt 0 ] ; then
    start_time=$SECONDS

    config_param=""
    build_param=""
    preset_param=""
    list_param=""
    custom_build_dir_param=""

    arg_option=""
    config_param_cnt=0
    build_param_cnt=0
    preset_param_cnt=0
    list_param_cnt=0
    custom_build_dir_param_cnt=0
    fresh_build_flag=0

    echo ""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c|-C)
                arg_option="config_param"
                config_param_cnt=$((config_param_cnt+1))
                ;;
            -b|-B)
                arg_option="build_param"
                build_param_cnt=$((build_param_cnt+1))
                ;;
            -p|-P)
                arg_option="preset_param"
                preset_param_cnt=$((preset_param_cnt+1))
                ;;
            -l|-L)
                arg_option="list_param"
                list_param_cnt=$((list_param_cnt+1))
                ;;
            -f|-F)
                fresh_build_flag=$((fresh_build_flag+1))
                rm -rf build
                echo -e "${YELLOW}Fresh Build has been triggered and build dir will be deleted.${NC}"
                ;;
            -d|-D)
                arg_option="custom_build_dir_param"
                custom_build_dir_param_cnt=$((custom_build_dir_param_cnt+1))
                ;;
            -help|-HELP|--h|--H|-h|-H|help|HELP)
                # Insert help message here
                echo -e "${CYAN}<--- ./run.sh HELP INFORMATION --->${NC}\n"
                echo -e "${GREEN}script usage --> \n"
                echo -e "Usage: ./run.sh [options]\n"
                echo -e "For Example : "
                echo -e "              ./run.sh  ${NC} --> Run all existing workflow"

                echo -e "${YELLOW}Optional Arguments [options]            --> \n"
                echo -e "     ./run.sh  -p|P                              --> Run default preset"
                echo -e "     ./run.sh  -p -c -DVAR=1                     --> Run default preset with variable VAR overide with 1 and run"
                echo -e "     ./run.sh  -p|P config,build,test            --> Run config,build,test as configuration,build,test preset respectively"
                echo -e "                 ${RED}Note:${NC} Order of preset should be same as above"
                echo -e "${YELLOW}"
                echo -e "     ./run.sh  -p|P config,build,test -c -DVAR=1 --> Run config,build,test as configuration,build,test preset respectively with variable overide"
                echo -e "     ./run.sh  -l|L                              --> List all available presets and exit"
                echo -e "     ./run.sh  -p|P -b -DBUILD_ARG=USR_ARG       --> Run default preset with change in build argument."
                echo -e "     ./run.sh  -f|F                              --> Run fresh and clean build"
                echo -e "${NC}"
                exit 1
                ;;
            -*)

                # Append options
                [[ $arg_option == "config_param" ]] && config_param+=" $1"
                [[ $arg_option == "build_param" ]]  && build_param+=" $1"
                [[ $arg_option == "preset_param" ]] && preset_param+=" $1"
                [[ $arg_option == "list_param" ]]   && list_param+=" $1"
                [[ $arg_option == "custom_build_dir_param" ]]   && custom_build_dir_param+=" $1"
                ;;
            *)
                # Input received, reset warning flag
                [[ $arg_option == "config_param" ]] && config_param+=" $1"
                [[ $arg_option == "build_param" ]]  && build_param+=" $1"
                [[ $arg_option == "preset_param" ]] && preset_param+=" $1"
                [[ $arg_option == "list_param" ]]   && list_param+=" $1"
                [[ $arg_option == "custom_build_dir_param" ]]   && custom_build_dir_param+=" $1"
                ;;
        esac
        shift
    done

    # Check if str is empty AND num is greater than zero
    if [[ -z "$config_param" && "$config_param_cnt" -gt 0 ]]; then
        echo -e "⚠️ ${YELLOW}WARNING: config option is selected, but input is not provided for config !!!${NC}"
    fi

    # Check if str is empty AND num is greater than zero
    if [[ -z "$build_param" && "$build_param_cnt" -gt 0 ]]; then
        echo -e "⚠️ ${YELLOW}WARNING: build option is selected, but input is not provided for build !!!${NC}"
    fi

    # Trim leading spaces
    config_param="${config_param#" "}"
    build_param="${build_param#" "}"
    preset_param="${preset_param#" "}"
    custom_build_dir_param="${custom_build_dir_param#" "}"

    #cfg_num_args=$(echo "$config_param"  | grep -o " " | wc -l)
    #build_num_args=$(echo "$build_param" | grep -o " " | wc -l)

    # Print output
    #echo "config_param : $config_param"
    #echo "build  param : $build_param"
    #echo "Test   param : $preset_param"

    # Check if str is empty AND num is greater than zero
    if [[ "$list_param_cnt" -gt 0 ]]; then
        echo -e "⚙️ ${CYAN} CONFIG    PRESETS --> ${PURPLE}" ${EXISTING_CONFIGURE_PRESETS//$'\n'/,} "${NC}"
        echo -e "🛠️ ${CYAN} BUILD     PRESETS --> ${PURPLE}" ${EXISTING_BUILD_PRESETS//$'\n'/,} "${NC}"
        echo -e "📤 ${CYAN} TEST      PRESETS --> ${PURPLE}" ${EXISTING_TEST_PRESETS//$'\n'/,} "${NC}"
        echo -e "🔄 ${CYAN} WORKFLOW  PRESETS --> ${PURPLE}" ${EXISTING_WORKFLOW_PRESETS//$'\n'/,} "${NC}\n"
        exit 0
    fi

    custom_build_dir=""

    if [[ "$custom_build_dir_param_cnt" -gt 0 ]]; then
        datetimeStr=$(date +"%H%M%S_%d%m%y")

        if [[ ! -f "CMakePresets_json.bak" ]] ; then
            cp -v "CMakePresets.json" "CMakePresets_json.bak"
        else
            echo -e "${YELLOW}Already backup of CMakePresets.json is created ${NC}"
        fi

        if [[ -n "$custom_build_dir_param" ]]; then
            if [[ "$custom_build_dir_param_cnt" -eq 1 ]]; then
                num_of_arg_for_custom_buid_dir=$(echo $custom_build_dir_param | wc -w)
                if [[ "$num_of_arg_for_custom_buid_dir" -gt 1 ]]; then
                    echo -e "${YELLOW}Custom build directory needs only one argument i.e. folder_name/path, considering first word ${NC}"
                fi
            else
                echo -e "${YELLOW}Custom build directory needs only one argument, i.e. folder_name/path, found more than 1 args, considering 1st arg ${NC}"
            fi
            custom_build_dir="${custom_build_dir_param%% *}"
            echo -e "${YELLOW}Custom Build Directory option has been opted so${NC} $custom_build_dir ${YELLOW} will be build dir.${NC}"
        else
            echo -e "${YELLOW}Custom Build Directory option has been opted so${NC} build/build_HHMMSS_DDMMYY ${YELLOW} will be build dir.${NC}"
            custom_build_dir="build_$datetimeStr"
        fi
        #default_build_dir_str="\"binaryDir\": \"\${sourceDir}/build/\${presetName}\","
        custom_build_dir_str="\"binaryDir\": \"\${sourceDir}/build/${custom_build_dir}/\${presetName}\","
        custom_build_dir_replace_str="$(printf '\t\t\t%s' "$custom_build_dir_str")"
        sed -i "/\"binaryDir\":/c\\$custom_build_dir_replace_str" "CMakePresets.json"
    fi

    # In below array (list) of configuration
    # Format : CONFIGURATION,BUILD_SETTING
    declare -a cfg_build_presets=(
        "armclang,armclang_build,armclang_build_test"
        "gcc,gcc_build,gcc_build_test"
    )

    # Check if str is empty AND num is greater than zero
    if [[ "$preset_param_cnt" -gt 0 ]]; then

        if [[ -z "$preset_param" ]]; then
            echo -e "⚠️ ${YELLOW}WARNING: Since no preset is selected, default will be selected  !!!${NC}"
            #Select Proper combination of config-build-test
            IFS=',' read -ra cfg_build_presets_array <<< "$cfg_build_presets"
            selected_cfg_preset=${cfg_build_presets_array[0]}
            selected_build_preset=${cfg_build_presets_array[1]}
            selected_test_preset=${cfg_build_presets_array[2]}
        else
            # Set IFS (Internal Field Separator) to comma (config,build,test)
            echo -e "${CYAN}📢 [INFO]: please make sure preset should be in order of CONFIG,BUILD,TEST PRESET !!!${NC}"

            IFS=',' read -ra preset_param_array <<< "$preset_param"

            if [[ $EXISTING_CONFIGURE_PRESETS =~ "${preset_param_array[0]}" ]]; then
                selected_cfg_preset=${preset_param_array[0]}
            else
                echo -e "${RED}Didn't find ${BLUE} ${preset_param_array[0]} ${RED}config preset in list ${BLUE}" {$EXISTING_CONFIGURE_PRESETS} "${NC}"
            fi

            if [[ $EXISTING_BUILD_PRESETS  =~ "${preset_param_array[1]}" ]]; then
                selected_build_preset=${preset_param_array[1]}
            else
                echo -e "${RED}Didn't find ${BLUE} ${preset_param_array[1]} ${RED}config preset in list ${BLUE}" {$EXISTING_BUILD_PRESETS} "${NC}"
            fi

            if [[ $EXISTING_TEST_PRESETS  =~ "${preset_param_array[2]}" ]]; then
                selected_test_preset=${preset_param_array[2]}
            else
                echo -e "${RED}Didn't find ${BLUE} ${preset_param_array[2]} ${RED}config preset in list ${BLUE}" {$EXISTING_TEST_PRESETS} "${NC}"
            fi
        fi

        echo -e " Config : ${GREEN}${selected_cfg_preset}${NC}"
        echo -e " Build  : ${GREEN}${selected_build_preset}${NC}"
        echo -e " Testing: ${GREEN}${selected_test_preset}${NC}"

        config_cmake_result=0
        build_cmake_result=0
        test_ctest_result=0

        if [[ "$fresh_build_flag" -gt 0 ]]; then
            cmake -E time cmake --fresh --preset=$selected_cfg_preset $config_param       || {  config_cmake_result=$?; echo -e "\n ${RED} \"$selected_cfg_preset\" CONFIG FAILED \n ${NC}"  ; } && \
            cmake --build --clean-first --preset=$selected_build_preset $build_param -- --|| {  build_cmake_result=$?; echo -e "\n ${RED} \"$selected_build_preset\" BUILD-CONFIG FAILED \n ${NC}"; }
        else
            cmake -E time cmake  --preset=$selected_cfg_preset $config_param       || {  config_cmake_result=$?; echo -e "\n ${RED} \"$selected_cfg_preset\" CONFIG FAILED \n ${NC}"  ; } && \
            cmake --build --preset=$selected_build_preset $build_param -- --|| {  build_cmake_result=$?; echo -e "\n ${RED} \"$selected_build_preset\" BUILD-CONFIG FAILED \n ${NC}"; }
        fi

        final_result=$((config_cmake_result + build_cmake_result))
        #if [[ "$final_result" == 0 ]]; then
        ctest --preset $selected_test_preset || { test_ctest_result=$?; echo -e "\n ${RED} \"$selected_build_preset\" TEST-CONFIG FAILED \n ${NC}" ; }
        #fi

        final_result=$((final_result + test_ctest_result))

        elapsed_1=$(( SECONDS - start_time ))
        eval "echo Elapsed time : $(date -ud "@$elapsed_1" +'$((%s/3600/24)) days %H hr %M min %S sec')"

    else

        # Loop through each preset and run configure & build
        for setting in "${cfg_build_presets[@]}";
            do
            start_time=$SECONDS
            #CONFIGURE_PRESETS="${setting%%,*}"
            #BUILD_PRESETS="${setting##*,}"
            IFS=',' read -r CONFIGURE_PRESETS BUILD_PRESETS TEST_PRESETS <<< "$setting"

            #echo "  Repo name   : $repoName   Branch Name : $branchName"
            echo -e "\n\nRunning CMake with configuration preset: ${GREEN} \"$CONFIGURE_PRESETS\" ${NC} and build preset ${HIGH_CYAN} \"$BUILD_PRESETS\" ${NC} + test preset ${HIGH_YELLOW} \"$TEST_PRESETS\" ${NC}"
            cmake -E time cmake --fresh --preset=$CONFIGURE_PRESETS $config_param || {  config_cmake_result=$?; echo -e "\n ${RED} \"$selected_cfg_preset\" CONFIG FAILED \n ${NC}"  ; }
            cmake --build  --preset=$BUILD_PRESETS $build_param -- --|| {  build_cmake_result=$?; echo -e "\n ${RED} \"$selected_build_preset\" BUILD-CONFIG FAILED \n ${NC}"; }
            ctest --preset $TEST_PRESETS || { test_ctest_result=$?; echo -e "\n ${RED} \"$selected_build_preset\" TEST-CONFIG FAILED \n ${NC}" ; }

            final_result=$((config_cmake_result + build_cmake_result + test_ctest_result))
            elapsed_1=$(( SECONDS - start_time ))
            eval "echo Elapsed time : $(date -ud "@$elapsed_1" +'$((%s/3600/24)) days %H hr %M min %S sec')"
        done
    fi

    if [[ "$custom_build_dir_param_cnt" -gt 0 ]]; then
        git checkout CMakePresets.json
    fi

else

    # Loop through each preset and run configure & build
    for WORKFLOW_PRESET in $EXISTING_WORKFLOW_PRESETS; do
        start_time=$SECONDS
        echo -e "\n\nRunning CMake with workflow preset: ${GREEN} \"$WORKFLOW_PRESET\" ${NC}"
        cmake --workflow --fresh --preset $WORKFLOW_PRESET
        elapsed_1=$(( SECONDS - start_time ))
        eval "echo Elapsed time : $(date -ud "@$elapsed_1" +'$((%s/3600/24)) days %H hr %M min %S sec')"
    done

fi

elapsed=$(( SECONDS - total_start_time ))
echo -e "\n\t"
eval "echo Total Elapsed time :: $(date -ud "@$elapsed" +'$((%s/3600/24)) days %H hr %M min %S sec')"
echo -e "\n"
exit $final_result
