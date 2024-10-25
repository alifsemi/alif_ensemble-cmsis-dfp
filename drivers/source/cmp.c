/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "cmp.h"

/**
  @fn          void cmp_irq_handler(CMP_Type *cmp)
  @brief       Clear the interrupt status
  @param[in]   cmp    Pointer to the CMP register map
  @return      none
*/
void cmp_irq_handler(CMP_Type *cmp)
{
    uint8_t int_status = (cmp->CMP_INTERRUPT_STATUS & (DEVICE_FEATURE_HSCMP_INT_STATUS_MASK));

    if(int_status == CMP_FILTER_EVENT0_CLEAR)
    {
        cmp->CMP_INTERRUPT_STATUS = CMP_FILTER_EVENT0_CLEAR;
    }

    if(int_status == CMP_FILTER_EVENT1_CLEAR)
    {
        cmp->CMP_INTERRUPT_STATUS = CMP_FILTER_EVENT1_CLEAR;
    }

}
