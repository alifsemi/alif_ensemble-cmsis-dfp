/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DAC_H_
#define DAC_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/**
  * @brief DAC120 (DAC120)
  */
typedef struct {                                /*!< (@ 0x49028000) DAC120 Structure                                           */
    volatile uint32_t  DAC_REG1;                /*!< (@ 0x00000000) REG1 DAC Control Register                                  */
    volatile uint32_t  DAC_IN;                  /*!< (@ 0x00000004) DAC Input Value Register                                   */
} DAC120_Type;                                  /*!< Size = 8 (0x8)                                                            */

/* DAC  Control register */
#define DAC_EN                   (1U << 0)   /* Enable DAC */
#define DAC_RESET_B              (1U << 27)  /* 0=Reset,this will reset the DAC */
#define DAC_HP_MODE_EN           (1U << 18)  /* To enable the dac output buffer */
#define DAC_MAX_INPUT            (0xFFFU)    /* Maximum input for the DAC is 4095(DAC 12 bit resolution) */
#define DAC_MAX_BYP_VAL_Msk      (0x7FFU)    /* DAC input data in bypass mode */
#define DAC_INPUT_BYP_MUX_Pos     1          /* Set DAC input source in bypass mode */
#define DAC_BYP_VAL_Pos           2          /* DAC input data bypass mode */
/**
 @fn           void dac_enable(DAC120_Type *dac)
 @brief        Enable the DAC.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_enable(DAC120_Type *dac)
{
    /* Enable the DAC */
    dac->DAC_REG1 |= (DAC_EN);
}

/**
 @fn           void dac_disable(DAC120_Type *dac)
 @brief        Disable the DAC.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_disable(DAC120_Type *dac)
{
    /* Disable the DAC */
    dac->DAC_REG1 &= ~(DAC_EN);
}

/**
 @fn           void dac_set_config(DAC120_Type *dac, uint8_t input_mux_val, uint16_t bypass_val)
 @brief        Configure the DAC
 @param[in]    input_mux_val : To select the Dac input data source
               bypass_val    : DAC input data in bypass mode
 @param[in]    dac   : Pointer to dac Type
 @return       none
 */
static inline void dac_set_config(DAC120_Type *dac, uint8_t input_mux_val, uint16_t bypass_val)
{
    /* Adding dac input mux and bypass value to the register */
    dac->DAC_REG1 |= (input_mux_val << DAC_INPUT_BYP_MUX_Pos | ((bypass_val & DAC_MAX_BYP_VAL_Msk) << DAC_BYP_VAL_Pos));
}

/**
 @fn           void dac_clear_config (DAC120_Type *dac)
 @brief        Clear the DAC configuration.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_clear_config(DAC120_Type *dac)
{
    /* Clear the DAC configuration */
    dac->DAC_REG1 = 0U;
}

/**
 @fn           dac_hp_mode_enable(DAC120_Type *dac)
 @brief        Enable HP mode of DAC.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_hp_mode_enable(DAC120_Type *dac)
{
    /* To enable the output buffer of DAC */
    dac->DAC_REG1 |= DAC_HP_MODE_EN;
}

/**
 @fn           void dac_reset_deassert(DAC120_Type *dac)
 @brief        DAC reset released.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_reset_deassert(DAC120_Type *dac)
{
    /* DAC reset released */
    dac->DAC_REG1 |= (DAC_RESET_B);
}

/**
 @fn           void dac_reset_assert(DAC120_Type *dac)
 @brief        DAC reset asserted.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_reset_assert(DAC120_Type *dac)
{
    /* DAC reset asserted */
    dac->DAC_REG1 &= ~(DAC_RESET_B);
}

/**
 @fn           void dac_input(DAC120_Type *dac, uint32_t value)
 @brief        Set the DAC input.
 @param[in]    value : DAC input
 @param[in]    dac   : Pointer to dac Type
 @return       none
 */
static inline void dac_input(DAC120_Type *dac, uint32_t value)
{
    /* set the DAC input */
    dac->DAC_IN = value;
}

#endif /* DAC_H */
