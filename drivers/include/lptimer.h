/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     lptimer.h
 * @author   Girish BN, Manoj A Murudi
 * @email    girish.bn@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     27-March-2023
 * @brief    Low Level header file for LPTIMER.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef LPTIMER_H_
#define LPTIMER_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* LPTIMER Control Register bit Definition */
#define LPTIMER_CONTROL_REG_TIMER_ENABLE_BIT                0x01U
#define LPTIMER_CONTROL_REG_TIMER_MODE_BIT                  0x02U
#define LPTIMER_CONTROL_REG_TIMER_INTERRUPT_MASK_BIT        0x04U
#define LPTIMER_CONTROL_REG_TIMER_PWM_BIT                   0x08U
#define LPTIMER_CONTROL_REG_TIMER_ON_100PWM_BIT             0x10U

/**
  * @brief LPTIMER (LPTIMER)
  */

typedef struct {                                     /*!< (@ 0x42001000) LPTIMER Structure                       */
    volatile uint32_t  LPTIMER0_LOADCOUNT;           /*!< (@ 0x00000000) Timer (n) Load Count Register           */
    volatile uint32_t  LPTIMER0_CURRENTVAL;          /*!< (@ 0x00000004) Timer (n) Current Value Register        */
    volatile uint32_t  LPTIMER0_CONTROLREG;          /*!< (@ 0x00000008) Timer (n) Control Register              */
    volatile uint32_t  LPTIMER0_EOI;                 /*!< (@ 0x0000000C) Timer (n) End-of-Interrupt Register     */
    volatile uint32_t  LPTIMER0_INTSTAT;             /*!< (@ 0x00000010) Timer (n) Interrupt Status Register     */
    volatile uint32_t  LPTIMER1_LOADCOUNT;           /*!< (@ 0x00000014) Timer (n) Load Count Register           */
    volatile uint32_t  LPTIMER1_CURRENTVAL;          /*!< (@ 0x00000018) Timer (n) Current Value Register        */
    volatile uint32_t  LPTIMER1_CONTROLREG;          /*!< (@ 0x0000001C) Timer (n) Control Register              */
    volatile uint32_t  LPTIMER1_EOI;                 /*!< (@ 0x00000020) Timer (n) End-of-Interrupt Register     */
    volatile uint32_t  LPTIMER1_INTSTAT;             /*!< (@ 0x00000024) Timer (n) Interrupt Status Register     */
    volatile uint32_t  LPTIMER2_LOADCOUNT;           /*!< (@ 0x00000028) Timer (n) Load Count Register           */
    volatile uint32_t  LPTIMER2_CURRENTVAL;          /*!< (@ 0x0000002C) Timer (n) Current Value Register        */
    volatile uint32_t  LPTIMER2_CONTROLREG;          /*!< (@ 0x00000030) Timer (n) Control Register              */
    volatile uint32_t  LPTIMER2_EOI;                 /*!< (@ 0x00000034) Timer (n) End-of-Interrupt Register     */
    volatile uint32_t  LPTIMER2_INTSTAT;             /*!< (@ 0x00000038) Timer (n) Interrupt Status Register     */
    volatile uint32_t  LPTIMER3_LOADCOUNT;           /*!< (@ 0x0000003C) Timer (n) Load Count Register           */
    volatile uint32_t  LPTIMER3_CURRENTVAL;          /*!< (@ 0x00000040) Timer (n) Current Value Register        */
    volatile uint32_t  LPTIMER3_CONTROLREG;          /*!< (@ 0x00000044) Timer (n) Control Register              */
    volatile uint32_t  LPTIMER3_EOI;                 /*!< (@ 0x00000048) Timer (n) End-of-Interrupt Register     */
    volatile uint32_t  LPTIMER3_INTSTAT;             /*!< (@ 0x0000004C) Timer (n) Interrupt Status Register     */
    volatile uint32_t  RESERVED[20];
    volatile uint32_t  LPTIMERS_INTSTATUS;           /*!< (@ 0x000000A0) Timers Interrupt Status Register        */
    volatile uint32_t  LPTIMERS_EOI;                 /*!< (@ 0x000000A4) Timers End-of-Interrupt Register        */
    volatile uint32_t  LPTIMERS_RAWINTSTATUS;        /*!< (@ 0x000000A8) Timers Raw Interrupt Status Register    */
    volatile uint32_t  LPTIMERS_COMP_VERSION;        /*!< (@ 0x000000AC) Reserved                                */
    volatile uint32_t  LPTIMER0_LOADCOUNT2;          /*!< (@ 0x000000B0) Timer (n) Load Count2 Register          */
    volatile uint32_t  LPTIMER1_LOADCOUNT2;          /*!< (@ 0x000000B4) Timer (n) Load Count2 Register          */
    volatile uint32_t  LPTIMER2_LOADCOUNT2;          /*!< (@ 0x000000B8) Timer (n) Load Count2 Register          */
    volatile uint32_t  LPTIMER3_LOADCOUNT2;          /*!< (@ 0x000000BC) Timer (n) Load Count2 Register          */
} LPTIMER_Type;                                      /*!< Size = 192 (0xc0)                                      */

/** \brief Channel specific registers */
typedef struct {
    volatile uint32_t  LPTIMER_LOADCOUNT;           /*!< CHANNEL Load Count Register                            */
    volatile uint32_t  LPTIMER_CURRENTVAL;          /*!< CHANNEL Current Value Register                         */
    volatile uint32_t  LPTIMER_CONTROLREG;          /*!< CHANNEL Control Register                               */
    volatile uint32_t  LPTIMER_EOI;                 /*!< CHANNEL End-of-Interrupt Register                      */
    volatile uint32_t  LPTIMER_INTSTAT;             /*!< CHANNEL Interrupt Status Register                      */
} LPTIMER_CHANNEL_Type;

/**
  \fn          static inline void lptimer_set_mode_userdefined (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Set user-defined mode for specified LPTIMER channel
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_set_mode_userdefined (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_CONTROLREG |=  LPTIMER_CONTROL_REG_TIMER_MODE_BIT;
}

/**
  \fn          static inline void lptimer_set_mode_freerunning (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Set free run mode for specified LPTIMER channel
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_set_mode_freerunning (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_CONTROLREG &= ~LPTIMER_CONTROL_REG_TIMER_MODE_BIT;
}

/**
  \fn          static inline void lptimer_load_count (LPTIMER_Type *lptimer, uint8_t channel, uint32_t value)
  \brief       Load counter value
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \param[in]   value     Pointer to variable which stores value to be assigned to counter
  \return      none
*/
static inline void lptimer_load_count (LPTIMER_Type *lptimer, uint8_t channel, uint32_t *value)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_LOADCOUNT = *value;
}

/**
  \fn          static inline void lptimer_load_max_count (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Load maximum counter value
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_load_max_count (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_LOADCOUNT = 0xFFFFFFFF;
}

/**
  \fn          static inline uint32_t lptimer_get_count (LPTIMER_Type *lptimer, uint8_t channel, uint32_t *value)
  \brief       Get current counter value
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      counter value
*/
static inline uint32_t lptimer_get_count (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    return lptimer_ch->LPTIMER_CURRENTVAL;
}

/**
  \fn          static inline void lptimer_enable_counter (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Enable timer
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_enable_counter (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_CONTROLREG |= LPTIMER_CONTROL_REG_TIMER_ENABLE_BIT;
}

/**
  \fn          static inline void lptimer_disable_counter (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Disable timer
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_disable_counter (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_CONTROLREG &= ~LPTIMER_CONTROL_REG_TIMER_ENABLE_BIT;
}

/**
  \fn          static inline void lptimer_clear_interrupt (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Clear pending cahnnel interrupt
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_clear_interrupt (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    (void) (lptimer_ch->LPTIMER_EOI);
}

/**
  \fn          static inline void lptimer_mask_interrupt (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Mask channel interrupt
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_mask_interrupt (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_CONTROLREG |= LPTIMER_CONTROL_REG_TIMER_INTERRUPT_MASK_BIT;
}

/**
  \fn          static inline void lptimer_unmask_interrupt (LPTIMER_Type *lptimer, uint8_t channel)
  \brief       Unmask channel interrupt
  \param[in]   lptimer   Pointer to the LPTIMER register map
  \param[in]   channel   lptimer channel
  \return      none
*/
static inline void lptimer_unmask_interrupt (LPTIMER_Type *lptimer, uint8_t channel)
{
    LPTIMER_CHANNEL_Type *lptimer_ch = (LPTIMER_CHANNEL_Type *)lptimer + channel;
    lptimer_ch->LPTIMER_CONTROLREG &= ~LPTIMER_CONTROL_REG_TIMER_INTERRUPT_MASK_BIT;
}

#ifdef __cplusplus
}
#endif

#endif /* LPTIMER_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
