/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     sys_ctrl_aes.h
 * @author   Silesh C V, Manoj A Murudi
 * @email    silesh@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     30-May-2023
 * @brief    AES control.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef SYS_CTRL_AES_H
#define SYS_CTRL_AES_H

#include <stdbool.h>
#include "soc.h"
#include "soc_features.h"
#include "ospi_delay.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AES_CONTROL_DECRYPT_EN  (1U << 0)
#define AES_CONTROL_RESET_LOGIC (1U << 1)
#define AES_CONTROL_XIP_EN      (1U << 4)
#define AES_CONTROL_LD_KEY      (1U << 7)

static inline void aes_enable_xip(AES_Type *aes)
{
    aes->AES_CONTROL |= AES_CONTROL_XIP_EN;
}

static inline void aes_disable_xip(AES_Type *aes)
{
    aes->AES_CONTROL &= ~AES_CONTROL_XIP_EN;
}

#if SOC_FEAT_AES_BAUD2_DELAY_VAL
#define AES_INTERRUPT_MASK_BAUD2_DELAY (1U << 30)

static inline void aes_set_baud2_delay(AES_Type *aes)
{
    aes->AES_INTERRUPT_MASK |= AES_INTERRUPT_MASK_BAUD2_DELAY;
}
#endif

#if SOC_FEAT_AES_OSPI_SIGNALS_DELAY
#define AES_SIGNAL_0_DELAY_POS  (0)
#define AES_SIGNAL_1_DELAY_POS  (8)
#define AES_SIGNAL_2_DELAY_POS  (16)
#define AES_SIGNAL_3_DELAY_POS  (24)
#define AES_SIGNAL_4_DELAY_POS  (0)
#define AES_SIGNAL_5_DELAY_POS  (8)
#define AES_SIGNAL_6_DELAY_POS  (16)
#define AES_SIGNAL_7_DELAY_POS  (24)
#define AES_SIGNAL_8_DELAY_POS  (0)
#define AES_SIGNAL_9_DELAY_POS  (8)
#define AES_SIGNAL_10_DELAY_POS (16)
#define AES_SIGNAL_11_DELAY_POS (24)
#define AES_SIGNAL_12_DELAY_POS (0)
#define AES_SIGNAL_13_DELAY_POS (8)
#define AES_SIGNAL_14_DELAY_POS (16)
#define AES_SIGNAL_15_DELAY_POS (24)
#define AES_TXD_DM_0_DELAY_POS  (0)
#define AES_TXD_DM_1_DELAY_POS  (8)
#define AES_DM_OE_N_DELAY_0_POS (16)
#define AES_DM_OE_N_DELAY_1_POS (24)
#define AES_SCLK_DELAY_POS      (0)
#define AES_SCLK_N_DELAY_POS    (8)

static inline void aes_set_rxds_delay(AES_Type *aes, const uint8_t rxds_delay[2])
{
    aes->AES_RXDS_DELAY = ((uint32_t)rxds_delay[0] << AES_SIGNAL_0_DELAY_POS) |
                          ((uint32_t)rxds_delay[1] << AES_SIGNAL_1_DELAY_POS);
}


static inline void aes_set_rxd_delay(AES_Type *aes, const uint8_t delay_val[16])
{
    aes->AES_RXD_DELAY_0 =
        ((uint32_t)delay_val[3] << AES_SIGNAL_3_DELAY_POS) | ((uint32_t)delay_val[2] << AES_SIGNAL_2_DELAY_POS) |
        ((uint32_t)delay_val[1] << AES_SIGNAL_1_DELAY_POS) | ((uint32_t)delay_val[0] << AES_SIGNAL_0_DELAY_POS);
    aes->AES_RXD_DELAY_1 =
        ((uint32_t)delay_val[7] << AES_SIGNAL_7_DELAY_POS) | ((uint32_t)delay_val[6] << AES_SIGNAL_6_DELAY_POS) |
        ((uint32_t)delay_val[5] << AES_SIGNAL_5_DELAY_POS) | ((uint32_t)delay_val[4] << AES_SIGNAL_4_DELAY_POS);
    aes->AES_RXD_DELAY_2 =
        ((uint32_t)delay_val[11] << AES_SIGNAL_11_DELAY_POS) | ((uint32_t)delay_val[10] << AES_SIGNAL_10_DELAY_POS) |
        ((uint32_t)delay_val[9] << AES_SIGNAL_9_DELAY_POS) | ((uint32_t)delay_val[8] << AES_SIGNAL_8_DELAY_POS);
    aes->AES_RXD_DELAY_3 =
        ((uint32_t)delay_val[15] << AES_SIGNAL_15_DELAY_POS) | ((uint32_t)delay_val[14] << AES_SIGNAL_14_DELAY_POS) |
        ((uint32_t)delay_val[13] << AES_SIGNAL_13_DELAY_POS) | ((uint32_t)delay_val[12] << AES_SIGNAL_12_DELAY_POS);
}

static inline void aes_set_txd_delay(AES_Type *aes, const uint8_t delay_val[16])
{
    aes->AES_TXD_DELAY_0 =
        ((uint32_t)delay_val[3] << AES_SIGNAL_3_DELAY_POS) | ((uint32_t)delay_val[2] << AES_SIGNAL_2_DELAY_POS) |
        ((uint32_t)delay_val[1] << AES_SIGNAL_1_DELAY_POS) | ((uint32_t)delay_val[0] << AES_SIGNAL_0_DELAY_POS);
    aes->AES_TXD_DELAY_1 =
        ((uint32_t)delay_val[7] << AES_SIGNAL_7_DELAY_POS) | ((uint32_t)delay_val[6] << AES_SIGNAL_6_DELAY_POS) |
        ((uint32_t)delay_val[5] << AES_SIGNAL_5_DELAY_POS) | ((uint32_t)delay_val[4] << AES_SIGNAL_4_DELAY_POS);
    aes->AES_TXD_DELAY_2 =
        ((uint32_t)delay_val[11] << AES_SIGNAL_11_DELAY_POS) | ((uint32_t)delay_val[10] << AES_SIGNAL_10_DELAY_POS) |
        ((uint32_t)delay_val[9] << AES_SIGNAL_9_DELAY_POS) | ((uint32_t)delay_val[8] << AES_SIGNAL_8_DELAY_POS);
    aes->AES_TXD_DELAY_3 =
        ((uint32_t)delay_val[15] << AES_SIGNAL_15_DELAY_POS) | ((uint32_t)delay_val[14] << AES_SIGNAL_14_DELAY_POS) |
        ((uint32_t)delay_val[13] << AES_SIGNAL_13_DELAY_POS) | ((uint32_t)delay_val[12] << AES_SIGNAL_12_DELAY_POS);
}

static inline void aes_set_ssioen_delay(AES_Type *aes, const uint8_t delay_val[16])
{
    aes->AES_SSI_OE_N_DELAY_0 =
        ((uint32_t)delay_val[3] << AES_SIGNAL_3_DELAY_POS) | ((uint32_t)delay_val[2] << AES_SIGNAL_2_DELAY_POS) |
        ((uint32_t)delay_val[1] << AES_SIGNAL_1_DELAY_POS) | ((uint32_t)delay_val[0] << AES_SIGNAL_0_DELAY_POS);
    aes->AES_SSI_OE_N_DELAY_1 =
        ((uint32_t)delay_val[7] << AES_SIGNAL_7_DELAY_POS) | ((uint32_t)delay_val[6] << AES_SIGNAL_6_DELAY_POS) |
        ((uint32_t)delay_val[5] << AES_SIGNAL_5_DELAY_POS) | ((uint32_t)delay_val[4] << AES_SIGNAL_4_DELAY_POS);
    aes->AES_SSI_OE_N_DELAY_2 =
        ((uint32_t)delay_val[11] << AES_SIGNAL_11_DELAY_POS) | ((uint32_t)delay_val[10] << AES_SIGNAL_10_DELAY_POS) |
        ((uint32_t)delay_val[9] << AES_SIGNAL_9_DELAY_POS) | ((uint32_t)delay_val[8] << AES_SIGNAL_8_DELAY_POS);
    aes->AES_SSI_OE_N_DELAY_3 =
        ((uint32_t)delay_val[15] << AES_SIGNAL_15_DELAY_POS) | ((uint32_t)delay_val[14] << AES_SIGNAL_14_DELAY_POS) |
        ((uint32_t)delay_val[13] << AES_SIGNAL_13_DELAY_POS) | ((uint32_t)delay_val[12] << AES_SIGNAL_12_DELAY_POS);
}

static inline void aes_set_txddm_delay(AES_Type *aes, const uint8_t txddm_delay[2], const uint8_t dmoen_delay[2])
{
    aes->AES_TXD_DM_DELAY =
        ((uint32_t)dmoen_delay[1] << AES_DM_OE_N_DELAY_1_POS) |
        ((uint32_t)dmoen_delay[0] << AES_DM_OE_N_DELAY_0_POS) |
        ((uint32_t)txddm_delay[1] << AES_TXD_DM_1_DELAY_POS) |
        ((uint32_t)txddm_delay[0] << AES_TXD_DM_0_DELAY_POS);
}

static inline void aes_set_ssn_delay(AES_Type *aes, const uint8_t ssn_delay[2])
{
    aes->AES_SS_N_DELAY = ((ssn_delay[0] << AES_SIGNAL_0_DELAY_POS) | (ssn_delay[1] << AES_SIGNAL_1_DELAY_POS));
}

static inline void aes_set_sclk_delay(AES_Type *aes, uint8_t sclk_delay, uint8_t sclkn_delay)
{
    aes->AES_SCLK_DELAY =
        ((uint32_t)sclk_delay << AES_SCLK_DELAY_POS) |
        ((uint32_t)sclkn_delay << AES_SCLK_N_DELAY_POS);
}

/* Apply a full TXD + RXD delay configuration  */
static inline void aes_set_signal_delay(AES_Type *aes, const ospi_delay_cfg_t *cfg)
{
    aes_set_txd_delay(aes, cfg->txd);
    aes_set_rxd_delay(aes, cfg->rxd);
    aes_set_ssioen_delay(aes, cfg->ssioen);
    aes_set_rxds_delay(aes, cfg->rxds);
    aes_set_txddm_delay(aes, cfg->txddm, cfg->dmoen);
    aes_set_sclk_delay(aes, cfg->sclk, cfg->sclkn);
    aes_set_ssn_delay(aes, cfg->ssn);
}
#else

static inline void aes_set_rxds_delay(AES_Type *aes, uint8_t rxds_delay)
{
    aes->AES_RXDS_DELAY = rxds_delay;
}
#endif

#if SOC_FEAT_AES_HAS_ADDR_CTRL_SHIM
typedef struct _aes_addr_ctrl {
    /* Number of bits in the lower portion of the address */
    uint8_t addr_lower_bits;
    /* Bit position of the lowest bit in the upper portion of the address */
    uint8_t addr_upper_shift;
    /* Enable array mode for SS0 line (lower 256MB) */
    bool ss0_array_mode_en;
    /* Enable array mode for SS1 line (upper 256MB) */
    bool ss1_array_mode_en;
    /* Mask bits in the lower portion of the address to drive low */
    uint16_t addr_mask;
} aes_addr_ctrl;

#define AES_ADDR_CTRL_ARRAY_MASK_POS        (0)
#define AES_ADDR_CTRL_SS0_ARRAY_MODE_POS    (18)
#define AES_ADDR_CTRL_SS1_ARRAY_MODE_POS    (19)
#define AES_ADDR_CTRL_ARRAY_MODE_SHIFT_POS  (22)
#define AES_ADDR_CTRL_ARRAY_MODE_SPLIT_POS  (28)

static inline void aes_control_address(AES_Type *aes, aes_addr_ctrl *addr_cfg)
{
    aes->AES_ADDR_CONTROL = ((addr_cfg->addr_mask << AES_ADDR_CTRL_ARRAY_MASK_POS) |
                             (addr_cfg->ss0_array_mode_en << AES_ADDR_CTRL_SS0_ARRAY_MODE_POS) |
                             (addr_cfg->ss1_array_mode_en << AES_ADDR_CTRL_SS1_ARRAY_MODE_POS) |
                             (addr_cfg->addr_upper_shift << AES_ADDR_CTRL_ARRAY_MODE_SHIFT_POS) |
                             (addr_cfg->addr_lower_bits << AES_ADDR_CTRL_ARRAY_MODE_SPLIT_POS));
}
#endif

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_AES_H */
