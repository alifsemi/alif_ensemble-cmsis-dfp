/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     rtc.h
 * @author   Tanay Rami, Manoj A Murudi
 * @email    tanay@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     23-March-2023
 * @brief    Device Specific Low Level Header file for RTC Driver.
 ******************************************************************************/

#ifndef RTC_H_
#define RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <soc.h>

/* CCR register fields */
#define CCR_LPRTC_PSCLR_EN (1U << 4U) /* Enable prescaler  */
#define CCR_LPRTC_WEN      (1U << 3U) /* Wrap enable       */
#define CCR_LPRTC_EN       (1U << 2U) /* Enable counter    */
#define CCR_LPRTC_MASK     (1U << 1U) /* Mask interrupts   */
#define CCR_LPRTC_IEN      (1U << 0U) /* Enable interrupts */

/**
  \fn           static inline void lprtc_counter_enable (LPRTC_Type *lprtc)
  \brief        Enable lprtc counter
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_counter_enable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR |= (CCR_LPRTC_EN);
}

/**
  \fn           static inline void lprtc_counter_disable (LPRTC_Type *lprtc)
  \brief        Disable lprtc counter
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_counter_disable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR &= ~(CCR_LPRTC_EN);
}

/**
  \fn           static inline void lprtc_prescaler_enable (LPRTC_Type *lprtc)
  \brief        Enable lprtc prescaler counter
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_prescaler_enable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR |= (CCR_LPRTC_PSCLR_EN);
}

/**
  \fn           static inline void lprtc_prescaler_disable (LPRTC_Type *lprtc)
  \brief        Disable lprtc prescaler counter
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_prescaler_disable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR &= ~(CCR_LPRTC_PSCLR_EN);
}

/**
  \fn           static inline void lprtc_counter_wrap_enable (LPRTC_Type *lprtc)
  \brief        Enable lprtc counter wrap
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_counter_wrap_enable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR |= (CCR_LPRTC_WEN);
}

/**
  \fn           static inline void lprtc_counter_wrap_disable (LPRTC_Type *lprtc)
  \brief        Disable lprtc counter wrap
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_counter_wrap_disable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR &= ~(CCR_LPRTC_WEN);
}

/**
  \fn           static inline void lprtc_interrupt_enable (LPRTC_Type *lprtc)
  \brief        Enable lprtc interrupt generation
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_interrupt_enable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR |= (CCR_LPRTC_IEN);
}

/**
  \fn           static inline void lprtc_interrupt_control_disable (LPRTC_Type *lprtc)
  \brief        Disable lprtc interrupt generation
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_interrupt_disable(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR &= ~(CCR_LPRTC_IEN);
}

/**
  \fn           static inline void lprtc_interrupt_mask (LPRTC_Type *lprtc)
  \brief        Mask lprtc interrupt generation
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_interrupt_mask(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR |= (CCR_LPRTC_MASK);
}

/**
  \fn           static inline void lprtc_interrupt_unmask (LPRTC_Type *lprtc)
  \brief        Unmask lprtc interrupt generation
  \param[in]    lprtc  : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_interrupt_unmask(LPRTC_Type *lprtc)
{
    lprtc->LPRTC_CCR &= ~(CCR_LPRTC_MASK);
}

/**
  \fn           static inline void lprtc_interrupt_ack (LPRTC_Type *lprtc)
  \brief        Acknowledge lprtc interrupt
  \param[in]    lprtc   : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_interrupt_ack(LPRTC_Type *lprtc)
{
    /* read to clear match interrupt. */
    (void) (lprtc->LPRTC_EOI);
}

/**
  \fn           static inline void lprtc_load_prescaler (LPRTC_Type *lprtc, uint32_t value)
  \brief        Load lprtc prescaler value
  \param[in]    value        : lprtc prescaler value
  \param[in]    lplprtc      : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_load_prescaler(LPRTC_Type *lprtc, uint32_t value)
{
    lprtc->LPRTC_CPSR = value;
}

/**
  \fn           static inline void lprtc_load_count (LPRTC_Type *lprtc, uint32_t value)
  \brief        Load lprtc counter value
  \param[in]    value     : lprtc prescaler value
  \param[in]    lplprtc   : Pointer to lprtc register block
  \return       none
*/
static inline void lprtc_load_count(LPRTC_Type *lprtc, uint32_t value)
{
    lprtc->LPRTC_CLR = value;
}

/**
  \fn           static inline void lprtc_load_counter_match_register (LPRTC_Type *lprtc, uint32_t
  value) \brief        Interrupt match register,When the internal counter matches this register, an
  interrupt is generated, provided interrupt generation is enabled. \param[in]    value : lprtc
  counter match register value \param[in]    lprtc : Pointer to lprtc register block \return none
*/
static inline void lprtc_load_counter_match_register(LPRTC_Type *lprtc, uint32_t value)
{
    lprtc->LPRTC_CMR = value;
}

/**
  \fn           static inline uint32_t lprtc_get_count (LPRTC_Type *lprtc)
  \brief        Read lprtc current counter value
  \param[in]    lprtc   : Pointer to lprtc register block
  \return       lprtc current counter value
*/
static inline uint32_t lprtc_get_count(LPRTC_Type *lprtc)
{
    return lprtc->LPRTC_CCVR;
}

#ifdef __cplusplus
}
#endif

#endif /* RTC_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
