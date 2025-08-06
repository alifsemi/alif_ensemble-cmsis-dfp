/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SYS_CTRL_CMP_H_
#define SYS_CTRL_CMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "soc.h"
#include "sys_utils.h"

#if SOC_FEAT_HSCMP_REG_ALIASING
#define CMP0_ENABLE  (1U << 28) /* To enable the CMP0  */
#define CMP1_ENABLE  (1U << 28) /* To enable the CMP1  */
#define CMP2_ENABLE  (1U << 28) /* To enable the CMP2  */
#define CMP3_ENABLE  (1U << 28) /* To enable the CMP3  */
#define LPCMP_ENABLE (1U << 24) /* To enable the LPCMP */
#else
#define CMP0_ENABLE  (1U << 28) /* To enable the CMP0  */
#define CMP1_ENABLE  (1U << 29) /* To enable the CMP1  */
#define CMP2_ENABLE  (1U << 30) /* To enable the CMP2  */
#define CMP3_ENABLE  (1U << 31) /* To enable the CMP3  */
#define LPCMP_ENABLE (1U << 24) /* To enable the LPCMP */
#endif

/* CLKCTL_PER_SLV CMP field definitions */
#if SOC_FEAT_HSCMP_REG_ALIASING
#define CMP_CTRL_CMP0_CLKEN (1U << 0U)  /* Enable CMP0 clock */
#define CMP_CTRL_CMP1_CLKEN (1U << 4U)  /* Enable CMP1 clock */
#define CMP_CTRL_CMP2_CLKEN (1U << 8U)  /* Enable CMP2 clock */
#define CMP_CTRL_CMP3_CLKEN (1U << 12U) /* Enable CMP3 clock */
#define LPCMP_CTRL_CLKEN    (1U << 0U)  /* Enable 32-kHz clock to LPCMP comparator */
#else
#define CMP_CTRL_CMP0_CLKEN (1U << 0U)                          /* Enable CMP0 clock */
#define CMP_CTRL_CMP1_CLKEN ((1U << 4U) | CMP_CTRL_CMP0_CLKEN)  /* Enable CMP1 clock */
#define CMP_CTRL_CMP2_CLKEN ((1U << 8U) | CMP_CTRL_CMP0_CLKEN)  /* Enable CMP2 clock */
#define CMP_CTRL_CMP3_CLKEN ((1U << 12U) | CMP_CTRL_CMP0_CLKEN) /* Enable CMP3 clock */
#define LPCMP_CTRL_CLKEN    (1 << 14) /* Enable 32-kHz clock to LPCMP comparator */
#endif

#if SOC_FEAT_HSCMP_REG_ALIASING
#define COMP0_HS_IN_P_SEL_Pos          (0U)
#define COMP0_HS_IN_M_SEL_Pos          (2U)
#define COMP0_HS_HYST_Pos              (4U)

#define COMP1_HS_IN_P_SEL_Pos          (0U)
#define COMP1_HS_IN_M_SEL_Pos          (2U)
#define COMP1_HS_HYST_Pos              (4U)

#define COMP2_HS_IN_P_SEL_Pos          (0U)
#define COMP2_HS_IN_M_SEL_Pos          (2U)
#define COMP2_HS_HYST_Pos              (4U)

#define COMP3_HS_IN_P_SEL_Pos          (0U)
#define COMP3_HS_IN_M_SEL_Pos          (2U)
#define COMP4_HS_HYST_Pos              (4U)
#else
#define COMP0_HS_IN_P_SEL_Pos          (0U)
#define COMP0_HS_IN_M_SEL_Pos          (2U)
#define COMP0_HS_HYST_Pos              (4U)

#define COMP1_HS_IN_P_SEL_Pos          (7U)
#define COMP1_HS_IN_M_SEL_Pos          (9U)
#define COMP1_HS_HYST_Pos              (11U)

#define COMP2_HS_IN_P_SEL_Pos          (14U)
#define COMP2_HS_IN_M_SEL_Pos          (16U)
#define COMP2_HS_HYST_Pos              (18U)

#define COMP3_HS_IN_P_SEL_Pos          (21U)
#define COMP3_HS_IN_M_SEL_Pos          (23U)
#define COMP4_HS_HYST_Pos              (25U)
#endif

/**
 * enum CMP_INSTANCE.
 * CMP instances.
 */
typedef enum _CMP_INSTANCE {
    CMP_INSTANCE_0, /**< CMP instance - 0  */
    CMP_INSTANCE_1, /**< CMP instance - 1  */
    CMP_INSTANCE_2, /**< CMP instance - 2  */
    CMP_INSTANCE_3, /**< CMP instance - 3  */
    CMP_INSTANCE_LP /**< CMP instance - LP */
} CMP_INSTANCE;

/**
 * @fn          static inline void enable_cmp_clk(uint8_t instance)
 * @brief       Enable CMP input clock
 * @param[in]   instance : Comparator instances
 * @return      none
 */
static inline void enable_cmp_clk(uint8_t instance)
{
    switch (instance) {
    case CMP_INSTANCE_0:
        CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP0_CLKEN;
        break;

    case CMP_INSTANCE_1:
        CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP1_CLKEN;
        break;

    case CMP_INSTANCE_2:
        CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP2_CLKEN;
        break;

    case CMP_INSTANCE_3:
        CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP3_CLKEN;
        break;

    case CMP_INSTANCE_LP:
#if SOC_FEAT_HSCMP_REG_ALIASING
        LPCMP->LPCOMP_CTRL |= LPCMP_CTRL_CLKEN;
#else
        ANA->VBAT_ANA_REG1 |= LPCMP_CTRL_CLKEN;
#endif
        break;
    }
}

/**
 * @fn          static inline void disable_cmp_clk(uint8_t instance)
 * @brief       Disable CMP input clock
 * @param[in]   instance : Comparator instances
 * @return      none
 */
static inline void disable_cmp_clk(uint8_t instance)
{
    switch (instance) {
    case CMP_INSTANCE_0:
        CLKCTL_PER_SLV->CMP_CTRL &= ~CMP_CTRL_CMP0_CLKEN;
        break;

    case CMP_INSTANCE_1:
        CLKCTL_PER_SLV->CMP_CTRL &= ~CMP_CTRL_CMP1_CLKEN;
        break;

    case CMP_INSTANCE_2:
        CLKCTL_PER_SLV->CMP_CTRL &= ~CMP_CTRL_CMP2_CLKEN;
        break;

    case CMP_INSTANCE_3:
        CLKCTL_PER_SLV->CMP_CTRL &= ~CMP_CTRL_CMP3_CLKEN;
        break;

    case CMP_INSTANCE_LP:
#if SOC_FEAT_HSCMP_REG_ALIASING
        LPCMP->LPCOMP_CTRL &= ~LPCMP_CTRL_CLKEN;
#else
        ANA->VBAT_ANA_REG1 &= ~LPCMP_CTRL_CLKEN;
#endif
        break;
    }
}

/**
 * @fn          static inline uint32_t *get_cmp_base(uint8_t instance)
 * @brief       Get the base address of the comparator instance
 * @param[in]   instance:  Comparator instance number
 * @return      Pointer to the base address of the corresponding comparator
 *              instance
 */
static inline uint32_t *get_cmp_base(uint8_t instance)
{
#if SOC_FEAT_HSCMP_REG_ALIASING
    switch (instance) {
    case CMP_INSTANCE_0:
        return (uint32_t *) CMP0_BASE;
    case CMP_INSTANCE_1:
        return (uint32_t *) CMP1_BASE;
    case CMP_INSTANCE_2:
        return (uint32_t *) CMP2_BASE;
    case CMP_INSTANCE_3:
        return (uint32_t *) CMP3_BASE;
    default:
        return NULL;
    }
#else

    ARG_UNUSED(instance);

    /* Comparator configuration register is provided at CMP0 base address */
    return (uint32_t *) CMP0_BASE;
#endif
}

/**
 * @fn          static inline void enable_cmp(uint8_t instance)
 * @brief       Enable the comparator module
 * @param[in]   instance  : Comparator instances
 * @return      None
 */
static inline void enable_cmp(uint8_t instance)
{
    /* Get the base address of the specified comparator instance and
     * assign it to cmp_reg */
    volatile uint32_t *cmp_reg = get_cmp_base(instance);

    switch (instance) {
    case CMP_INSTANCE_0:
        /* Enable the CMP0 module */
        *cmp_reg |= CMP0_ENABLE;
        break;

    case CMP_INSTANCE_1:
        /* Enable the CMP1 module */
        *cmp_reg |= CMP1_ENABLE;
        break;

    case CMP_INSTANCE_2:
        /* Enable the CMP2 module */
        *cmp_reg |= CMP2_ENABLE;
        break;

    case CMP_INSTANCE_3:
        /* Enable the CMP3 module */
        *cmp_reg |= CMP3_ENABLE;
        break;

    case CMP_INSTANCE_LP:
#if SOC_FEAT_HSCMP_REG_ALIASING
        LPCMP->LPCOMP_CTRL |= LPCMP_ENABLE;
#else
        /* Enable the LPCMP module */
        ANA->VBAT_ANA_REG2 |= LPCMP_ENABLE;
#endif
        break;
    }
}

/**
 * @fn          static inline void disable_cmp(uint8_t instance)
 * @brief       Disable the comparator module
 * @param[in]   instance  : Comparator instances
 * @return      None
 */
static inline void disable_cmp(uint8_t instance)
{
    /* Get the base address of the specified comparator instance and
     * assign it to cmp_reg */
    volatile uint32_t *cmp_reg = get_cmp_base(instance);

    switch (instance) {
    case CMP_INSTANCE_0:
        /* Disable the CMP0 module */
        *cmp_reg &= ~CMP0_ENABLE;
        break;

    case CMP_INSTANCE_1:
        /* Disable the CMP1 module */
        *cmp_reg &= ~CMP1_ENABLE;
        break;

    case CMP_INSTANCE_2:
        /* Disable the CMP2 module */
        *cmp_reg &= ~CMP2_ENABLE;
        break;

    case CMP_INSTANCE_3:
        /* Disable the CMP3 module */
        *cmp_reg &= ~CMP3_ENABLE;
        break;

    case CMP_INSTANCE_LP:
#if SOC_FEAT_HSCMP_REG_ALIASING
        LPCMP->LPCOMP_CTRL &= ~LPCMP_ENABLE;
#else
        /* Disable the LPCMP module */
        ANA->VBAT_ANA_REG2 &= ~LPCMP_ENABLE;
#endif
        break;
    }
}

/**
 * @fn          void cmp_set_config(uint8_t instance, uint32_t config_value)
 * @brief       Set the configuration value for the specified CMP instance
 *              For LPCMP, the configuration is applied to the appropriate register.
 * @param[in]   instance     : Comparator instances
 * @param[in]   config_value : Configuration value to be applied
 * @return      None
 */
static void cmp_set_config(uint8_t instance, uint32_t config_value)
{
    if (instance == CMP_INSTANCE_LP) {
#if SOC_FEAT_HSCMP_REG_ALIASING
        LPCMP->LPCOMP_CTRL |= config_value;
#else
        /* Set LPCMP configuration value in the VBAT ANA register 2 */
        ANA->VBAT_ANA_REG2 |= config_value;
#endif
    } else {
        /* Get the base address of the specified comparator instance and
         * assign it to cmp_reg */
        volatile uint32_t *cmp_reg  = get_cmp_base(instance);

        /* Set the configuration value in reg1 of the CMP instance */
        *cmp_reg                   |= config_value;
    }
}

/**
 * @fn          void lpcmp_clear_config(void)
 * @brief       Clear LPCMP configuration value in Vbat reg2.
 * @param[in]   None
 * @return      None
 */
static void lpcmp_clear_config(void)
{
#if SOC_FEAT_HSCMP_REG_ALIASING
    LPCMP->LPCOMP_CTRL &= ~LPCMP_MSK_CTRL_VAL;
#else
    /* Clear LPCMP configuration values in Vbat reg2 */
    ANA->VBAT_ANA_REG2 &= ~LPCMP_MSK_CTRL_VAL;
#endif
}

/**
 * @fn          uint8_t cmp_int_mask(void)
 * @brief       HSCMP device specific interrupt status
 * @param[in]   None
 * @return      HSCMP device specific interrupt status macro
 */
static inline uint8_t cmp_int_mask(void)
{
    return SOC_FEAT_HSCMP_INT_STATUS_MASK;
}

/**
 * @fn          uint8_t cmp_window_enable_value(void)
 * @brief       HSCMP device specific window function
 * @param[in]   None
 * @return      HSCMP device specific window function macro
 */
static inline uint8_t cmp_window_enable_value(void)
{
    return SOC_FEAT_HSCMP_WINDOW_ENABLE_VAL;
}

#endif /* SYS_CTRL_CMP_H_ */
