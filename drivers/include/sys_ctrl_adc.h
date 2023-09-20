/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SYS_CTRL_ADC_H_
#define SYS_CTRL_ADC_H_

#include "peripheral_types.h"

/**
 * enum ADC_INSTANCE.
 * ADC instances.
 */
typedef enum _ADC_INSTANCE
{
    ADC_INSTANCE_0,                         /* ADC instance - 0 */
    ADC_INSTANCE_1,                         /* ADC instance - 1 */
    ADC_INSTANCE_2,                         /* ADC instance - 2 */
    ADC24_INSTANCE,                         /* ADC24 instance   */
} ADC_INSTANCE;

/* ADC120 */
static inline void enable_adc0_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC0_CKEN;
}

static inline void disable_adc0_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL &= ~ADC_CTRL_ADC0_CKEN;
}

/* ADC121 */
static inline void enable_adc1_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC1_CKEN;
}

static inline void disable_adc1_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL &= ~ADC_CTRL_ADC1_CKEN;
}

/* ADC122 */
static inline void enable_adc2_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC2_CKEN;
}

static inline void disable_adc2_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL &= ~ADC_CTRL_ADC2_CKEN;
}

/* ADC24 */
static inline void enable_adc24_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC24_CKEN;
}

static inline void disable_adc24_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL &= ~ADC_CTRL_ADC24_CKEN;
}

/**
  \fn     static inline void control_adc_cmp_periph_clk(void)
  \brief  Enable ADC, CMP Control register.
  \param  none.
  \return none.
 */
static inline void enable_adc_cmp_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC0_CKEN;
    CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP0_CLKEN;
}

/**
  \fn     static inline void disable_adc_cmp_periph_clk(void)
  \brief  Disable ADC, CMP Control register.
  \param  none.
  \return none.
 */
static inline void disable_adc_cmp_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL = 0U;
    CLKCTL_PER_SLV->CMP_CTRL = 0U;
}

/**
  \fn     static inline void enable_adc24(void)
  \brief  Enable ADC24 from control register
  \param  none.
  \return none.
 */
static inline void enable_adc24(void)
{
    __disable_irq();

    AON->PMU_PERIPH |= PMU_PERIPH_ADC24_EN;

    __enable_irq();
}

/**
  \fn     static inline void disable_adc24(void)
  \brief  Disable ADC24 from control register
  \param  none.
  \return none.
 */
static inline void disable_adc24(void)
{
    __disable_irq();

    AON->PMU_PERIPH &= ~PMU_PERIPH_ADC24_EN;

    __enable_irq();
}

/*
 * @func           : void set_adc24_output_rate(uint32_t rate)
 * @brief          : set output rate for adc24
 * @parameter[1]   : bias : value for setting bias
 * @return         : NONE
 */
static inline void set_adc24_bias(uint32_t bias)
{
    __disable_irq();

    AON->PMU_PERIPH |= bias;

    __enable_irq();
}

/*
 * @func           : void set_adc24_output_rate(uint32_t rate)
 * @brief          : set output rate for adc24
 * @parameter[1]   : rate : value for setting output rate
 * @return         : NONE
 */
static inline void set_adc24_output_rate(uint32_t rate)
{
    __disable_irq();

    AON->PMU_PERIPH |= rate;

    __enable_irq();
}

/*
 * @func           : void enable_adc_pga_gain(uint32_t gain_setting)
 * @brief          : Enable the pga gain for adc instances
 * @parameter[1]   : instance : adc controller instance
 * @parameter[2]   : gain     : setting to configure
 * @return         : NONE
 */
static inline void enable_adc_pga_gain(uint8_t instance, uint32_t gain)
{
    __disable_irq();

    switch (instance)
    {
        case ADC_INSTANCE_0:
        {
            AON->PMU_PERIPH |= (PMU_PERIPH_ADC1_PGA_EN | (gain << PMU_PERIPH_ADC1_PGA_GAIN_Pos));
            break;
        }
        case ADC_INSTANCE_1:
        {
            AON->PMU_PERIPH |= (PMU_PERIPH_ADC2_PGA_EN | (gain << PMU_PERIPH_ADC2_PGA_GAIN_Pos));
            break;
        }
        case ADC_INSTANCE_2:
        {
            AON->PMU_PERIPH |= (PMU_PERIPH_ADC3_PGA_EN | (gain << PMU_PERIPH_ADC3_PGA_GAIN_Pos));
            break;
        }
        case ADC24_INSTANCE:
        {
            AON->PMU_PERIPH |= (PMU_PERIPH_ADC24_PGA_EN | (gain << PMU_PERIPH_ADC24_PGA_GAIN_Pos));
            break;
        }
    }
    __enable_irq();
}

/*
 * @func           : void disable_adc_Pga_gain(uint32_t gain_setting)
 * @brief          : Disable the pga gain for adc instances
 * @parameter[1]   : instance : adc controller instance
 * @return         : NONE
 */
static inline void disable_adc_pga_gain(uint8_t instance)
{
    __disable_irq();

    switch (instance)
    {
        case ADC_INSTANCE_0:
        {
            AON->PMU_PERIPH &= ~(PMU_PERIPH_ADC1_PGA_EN | PMU_PERIPH_ADC1_PGA_GAIN_Msk);
            break;
        }
        case ADC_INSTANCE_1:
        {
            AON->PMU_PERIPH &= ~(PMU_PERIPH_ADC2_PGA_EN | PMU_PERIPH_ADC2_PGA_GAIN_Msk);
            break;
        }
        case ADC_INSTANCE_2:
        {
            AON->PMU_PERIPH &= ~(PMU_PERIPH_ADC3_PGA_EN | PMU_PERIPH_ADC3_PGA_GAIN_Msk);
            break;
        }
        case ADC24_INSTANCE:
        {
            AON->PMU_PERIPH &= ~(PMU_PERIPH_ADC24_PGA_EN | PMU_PERIPH_ADC24_PGA_GAIN_Msk);
            break;
        }
    }

    __enable_irq();
}

#endif /* SYS_CTRL_ADC_H_ */
