/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     WiFi_utils.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     04-Feb-2024
 * @brief    Generic functions
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
 
#include "WiFi_utils.h"
#include "system_utils.h"

/**
  \fn            int32_t get_nth_pos_delimiterStr(uint8_t *input, \
                            uint8_t *output, uint8_t *delimiter, uint32_t pos,\
                            uint16_t nStrings)
  \brief         This function extracts string for given position.
  \param[in,out]    input       Address of input string
  \param[out]       output      Array of pointers of retrieved strings
  \param[in]        delimiter   Delimiter value
  \param[in]        pos         Array of positions
  \param[in]        nStrings    Number of strings needs to extracted
  \return           Success

  \note  input string will be modified, also position starts from 0.
  Caution : Don't pass pointers of constant pointer
*/
int32_t get_nth_pos_delimiterStr(uint8_t *input, uint8_t **output, \
        const char *delimiter, uint16_t *nPos, uint16_t nStrings)
{
    uint32_t cnt = 0, posCnt = 0;
    uint8_t *tmp;

    if( !(input) || !(output))
        return -1;

    tmp = (uint8_t*) STRTOK((char*)input, (const char*) delimiter);

    while(tmp != NULL)
    {
        if (nPos)
        {
            if (nPos[posCnt] == cnt)
            {
                *output = tmp;
                output++;
                posCnt++;
            }
        }
        else
        {
            *output = tmp;
            output++;
            posCnt++;
        }

        if (posCnt == nStrings)
            break;

        tmp = (uint8_t*) STRTOK(NULL, (const char*) delimiter);
        cnt++;
    }

    return cnt;
}


/**
  \fn            void delay_in_ms (uint32_t delayMS)
  \brief         It will introduce delay in milli-sec.
  \param[in]     delayMS Delay in milli-sec
  \return        void
*/
void delay_in_ms(uint32_t delayMS)
{
    uint32_t cnt;

    for(cnt = 0; cnt < delayMS; cnt++)
    {
        sys_busy_loop_us(1000);  // 1 MS
    }

    return;
}

/**
  \fn            int32_t wait_cond(uint32_t *condVar, uint32_t waitCond,
                                   uint32_t tMSretryCnt)
  \brief         Wait for 'condVar' variable to be equal to 'waitCond'
  \param[in]     condVar    Address of Monitoring variable
  \param[in]     waitCond   Value on which wait should over
  \param[in]     timoutInMS Timeout value in milli-sec; 0 indicate for-ever wait
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t wait_cond(uint32_t *condVar, uint32_t waitCond, uint32_t timoutInMS)
{
    int32_t ret = -1;

    if(timoutInMS)
    {
        do
        {
            delay_in_ms(1);

            if(*condVar == waitCond)
            {
                ret = 0;
                break;
            }
        }while(timoutInMS--);
    }
    else
    {
        while(*condVar != waitCond);
        ret = 0;
    }

    return ret;
}

/**
  \fn            int32_t str_wait_cond(uint8_t* str, uint8_t* waitStr,
                                   uint32_t tMSretryCnt)
  \brief         Wait for string 'str' to be equal to 'waitStr'
  \param[in]     str       String which needs to monitor
  \param[in]     waitStr   String for which 'str' should wait
  \param[in]     timoutInMS Timeout value in milli-sec; 0 indicate for-ever wait
  \return        execution status
                   - \ref ARM_DRIVER_OK                : Operation successful
                   - \ref ARM_DRIVER_ERROR             : Operation failed
*/
int32_t str_wait_cond(uint8_t* str, uint8_t* waitStr, uint32_t timoutInMS)
{
    int32_t ret = -1;

    if(timoutInMS)
    {
        do
        {
            delay_in_ms(1);

            if(!STRCMP((const char*)str, (const char*) waitStr))
            {
                ret = 0;
                break;
            }
        }while(timoutInMS--);
    }
    else
    {
        while(STRCMP((const char*)str, (const char*) waitStr));
        ret = 0;
    }

    return ret;
}
