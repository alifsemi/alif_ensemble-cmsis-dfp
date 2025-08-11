#!/bin/bash 

NC='\033[0m' # No Color
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
BROWN='\033[0;37m'
HIGH_CYAN='\033[38;5;45m'

curr_temp=$1

echo -e ""
echo -e "🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥\n"
cat $curr_temp/.cfg
echo -e "\n🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥\n"

echo -e ""
echo -e "✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨\n"

if [ -e $curr_temp/.tmp ]
then
    line1=$(sed -n '1p' $curr_temp/.tmp)
    line2=$(sed -n '2p' $curr_temp/.tmp)
    available_elf_files=$(find $line1 -maxdepth 1 -type f -name '*.elf' | wc -l)
    requested_application_cnt=$(echo $line2 | cut -d "," -f 1 | cut -d " " -f 2)
    invalid_application_cnt=`tail -1 "$curr_temp/.tmp"`

    generated_elf_files_cnt=0
    old_elf_files_cnt=0

    # First 2 line is already read in previous lines
    for i in `seq 1 $((requested_application_cnt - invalid_application_cnt))`
    do
        k=$((i+2))         # Skipping first two line
        line=$(sed -n "${k}p" $curr_temp/.tmp)

        eval "grep -sqF \"${line}\" \"$curr_temp/.tmp1\""
        if [ $? -eq 0 ] ; then
            execGeneratedFlag=1
        else
            execGeneratedFlag=0 # do something else if the string is not found
        fi

        elf_file=$line.elf
        if [ -e "$line1/$elf_file" ] ; then
            if [ ${execGeneratedFlag} -eq 1 ]; then
                echo -e $'\U2705'" ${GREEN}-- Generate App Name          --> $elf_file ${NC}"
                generated_elf_files_cnt=$((generated_elf_files_cnt+1))
            else
                echo -e $'\U2757'" ${YELLOW}-- Couldn't Generate App Name --> $elf_file ${BLUE}(last successful build elf exist) ${NC}"
                old_elf_files_cnt=$((old_elf_files_cnt+1))
            fi

        else
            echo -e '\U26D4'" ${RED}-- Couldn't Generate App Name --> $elf_file ${NC}"
        fi
    done

    failed_elf_cnt=$((requested_application_cnt - invalid_application_cnt - generated_elf_files_cnt - old_elf_files_cnt))
    echo -e "\n✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨✨"

    echo -e "\n${HIGH_CYAN}-- Application Build Statistics  [ ${GREEN}Generated: $generated_elf_files_cnt, ${RED}Failed: $failed_elf_cnt, Invalid: $invalid_application_cnt, ${YELLOW}$line2 ] \n${NC}"
    #rm   $curr_temp/.tmp
    #rm   $curr_temp/.tmp1
fi
