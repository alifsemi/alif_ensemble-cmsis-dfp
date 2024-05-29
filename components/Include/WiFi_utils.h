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
 * @file     WiFi_utils.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     04-Feb-2024
 * @brief    Generic functions
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
 
#ifndef _WIFI_UTILS_H
#define _WIFI_UTILS_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define SPRINTF(dst, fmt, ...)      sprintf((char*)dst , fmt, ## __VA_ARGS__)
#define STRNCPY(dst, src, n)        strncpy((char*)dst, (const char*)src, n)
#define STRCPY(dst, src)            strcpy((char*)dst, (const char*)src)
#define STRLEN(str)                 strlen((const char*) str)
#define MEMSET(src, val, nBytes)    memset((void*)src, (int)val, (size_t) nBytes)
#define MEMCPY(dst, src, nBytes)    memcpy((void*)dst, (void*)src, (size_t) nBytes)
#define ATOI(src)                   atoi((const char*) src)
#define STRSTR(str, key)            strstr((const char *)str, (const char *)key)
#define STRTOK(str, delims)         strtok((char *)str, (const char *)delims)
#define STRCMP(str1, str2)          strcmp((const char *)str1, (const char *)str2)

#define WIFI_SUCCESS                0
#define WIFI_FAIL                   -1

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
        const char *delimiter, uint16_t *nPos, uint16_t nStrings);

/**
  \fn            void delay_in_ms (uint32_t delayMS)
  \brief         It will introduce delay in milli-sec.
  \param[in]     delayMS Delay in milli-sec
  \return        void
*/
void delay_in_ms(uint32_t delayMS);

/**
  \fn            void we310f5_reset_spi_packet (uint8_t *packet, uint16_t size)
  \brief         Reset (i.e. zeros) input buffer for given size
  \param[in]     packet Address of input buffer
  \param[in]     size   Count number of bytes to be reset
  \return        none
*/
void we310f5_reset_spi_packet(void *packet, uint16_t size);

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
int32_t wait_cond(uint32_t *condVar, uint32_t waitCond, uint32_t timoutInMS);

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
int32_t str_wait_cond(uint8_t* str, uint8_t* waitStr, uint32_t timoutInMS);

#endif  /* _WIFI_UTILS_H */
