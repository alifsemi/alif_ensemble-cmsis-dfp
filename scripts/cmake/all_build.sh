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

#set -x

# Get all configure presets

start_time=$SECONDS

total_cfg_run_cnt=0
skipped_cfg_run_cnt=0
actual_cfg_run_cnt=0
fail_cfg_run_cnt=0
success_cfg_run_cnt=0
total_cmakeError=-1
collective_total_Error=0
number_of_iter=0

CURR_OS=("NONE")
CURR_COMPILER=("armclang" "gcc")
CURR_DEV=("AE722F80F55D5" "AE1C1F4051920" "AE822FA0E5597" "AE402FA0E5597")
CURR_BOARDS=("DevKit-e7" "DevKit-e1c" "DevKit-e8" "DevKit-e4")
CURR_RTSS=("HE" "HP")
CURR_BOOT=("TCM" "MRAM")

for comp in ${!CURR_COMPILER[@]}
do
    for bootType in ${!CURR_BOOT[@]}
    do
        for rtssType in ${!CURR_RTSS[@]}
        do
            for devName in ${!CURR_DEV[@]}
            do
                for brdName in ${!CURR_BOARDS[@]}
                do
                    devShortName="${CURR_BOARDS[brdName]#*-}"
                    alifDevShortName="A$(echo "$devShortName" | tr '[:lower:]' '[:upper:]')"
                    echo "Device : ${CURR_DEV[devName]}, Board: ${CURR_BOARDS[brdName]}, ShortName:${alifDevShortName}."
                    number_of_iter=$((number_of_iter + 1))

                    echo "🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱 # ${number_of_iter} starts 🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱🧱"
                    if [[ "${CURR_DEV[devName]}" == "$alifDevShortName"* ]]; then
                        for osName in ${!CURR_OS[@]}
                        do
                            echo "======================================================================================"
                            if [ -d build ]; then
                                echo -e "Deleting Build directory..."
                                rm -rf build
                            fi

                            total_cfg_run_cnt=$((total_cfg_run_cnt + 1))
                            echo "🚩 Building for ${CURR_COMPILER[comp]}, ${CURR_DEV[devName]}, ${CURR_OS[osName]}, ${CURR_BOOT[bootType]} ${CURR_RTSS[rtssType]} ${CURR_BOARDS[brdName]}"

                            if [ "${CURR_DEV[devName]}" = "AE1C1F4051920" ] && [ "${CURR_RTSS[rtssType]}" = "HP" ]; then
                                echo -e "🚨 ${CURR_DEV[devName]} does not have ${CURR_RTSS[rtssType]} subsystem"
                                skipped_cfg_run_cnt=$((skipped_cfg_run_cnt + 1))
                                continue
                            fi

                            actual_cfg_run_cnt=$((actual_cfg_run_cnt + 1))
                            ./run.sh -p ${CURR_COMPILER[comp]},${CURR_COMPILER[comp]}_build,${CURR_COMPILER[comp]}_build_test -c --fresh -DBOOT=${CURR_BOOT[bootType]} -DDEVICE=${CURR_DEV[devName]} -DBOARD_NAME=${CURR_BOARDS[brdName]} -DOS=${CURR_OS[osName]} -DRTSS=${CURR_RTSS[rtssType]} -b --clean-first

                            tmp=$?
                            if [[ "$total_cmakeError" -eq -1 ]] ; then 
                                total_cmakeError=$tmp
                            else
                                total_cmakeError=$((total_cmakeError + tmp))
                            fi
                            echo -e "🚫Error Status total_cmakeError: $total_cmakeError and tmp: $tmp"
                            if [ "$tmp" -ne 0 ] ; then
                                fail_cfg_run_cnt=$((fail_cfg_run_cnt + 1))
                            fi
                            echo "======================================================================================"
                        done
                    fi
                    echo "🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩 # ${number_of_iter} ends 🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩🧩"
                    echo -e "\n"
                done
            done
        done
    done
done
echo -e ""
echo -e " 🏆Success: $((actual_cfg_run_cnt - fail_cfg_run_cnt)), Failed: $fail_cfg_run_cnt, Skipped: $skipped_cfg_run_cnt"
echo -e " 🔥Total Run => ($actual_cfg_run_cnt/$total_cfg_run_cnt)"
collective_total_Error=$((total_cmakeError))

echo -e "\n\n"
total_run_elapsed_time=$(( SECONDS - start_time ))
eval "echo  Total Elapsed Time: $(date -ud "@$total_run_elapsed_time" +'$((%s/3600/24)) days %H hr %M min %S sec')"
echo -e "\n"

exit $collective_total_Error