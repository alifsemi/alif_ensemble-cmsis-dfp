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
 * @file     sd_host.h
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     09-June-2023
 * @brief    SD Host Controller Register mapping.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef __SD_HOST_H__
#define __SD_HOST_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stdint.h"
#include "soc.h"
#include "sd_types.h"

/* Forward declaration */
struct _sd_handle_t;

#define SDMMC_HC_VERSION_REG    (SDMMC_BASE + 0xFEU)
#define SDHC_HC_VERSION_REG_Msk 0xFFFFU
#define SDHC_IRQ_NUM            SDMMC_IRQ_IRQn
#define SDHC_WAKEUP_IRQ_NUM     SDMMC_WAKEUP_IRQ_IRQn

/* ADMA Descriptor 64-bit format (matching original packed struct layout):
 * Bits 0-15:   Attribute
 * Bits 16-31:  Length
 * Bits 32-63:  Address
 */
#define ADMA2_DESC_ATTR_MASK   0x000000000000FFFFULL
#define ADMA2_DESC_LEN_SHIFT   16
#define ADMA2_DESC_LEN_MASK    0x00000000FFFF0000ULL
#define ADMA2_DESC_ADDR_SHIFT  32
#define ADMA2_DESC_ADDR_MASK   0xFFFFFFFF00000000ULL

/* ADMA Descriptor Constant */
#define SDHC_ADMA2_DESC_MAX_LEN          65536U
#define SDHC_ADMA2_MAX_DESC              32U
#define SDHC_ADMA2_DESC_VALID            (0x1U << 0U)
#define SDHC_ADMA2_DESC_END              (0x1U << 1U)
#define SDHC_ADMA2_DESC_INT              (0x1U << 2U)
#define SDHC_ADMA2_DESC_TRAN             (0x1U << 5U)

/* Host Controller Specific Constant */
#define SDHC_HC_SPEC_V3                  0x0002U /**< HC spec version 3 */
#define SDHC_HC_SPEC_V2                  0x0001U /**< HC spec version 2 */
#define SDHC_HC_SPEC_V1                  0x0000U /**< HC spec version 1 */
#define SDHC_HC_SPEC_VER_Msk             0x00FFU /**< Host Specification version mask */

/* Software Reset Register */
#define SDHC_SW_RST_ALL_Pos              0U
#define SDHC_SW_RST_ALL_Msk              (1U << SDHC_SW_RST_ALL_Pos)
#define SDHC_SW_RST_CMD_Pos              1U
#define SDHC_SW_RST_CMD_Msk              (1U << SDHC_SW_RST_CMD_Pos)
#define SDHC_SW_RST_DAT_Pos              2U
#define SDHC_SW_RST_DAT_Msk              (1U << SDHC_SW_RST_DAT_Pos)

/* CMD and Response register mapping */
#define SDHC_CMD_IDX_Pos                 8U
#define SDHC_CMD_TYPE_Pos                6U
#define SDHC_CMD_RSP_SEL_Pos             0U

/* Present state */
#define SDHC_CMD_INHIBIT_Msk             1U
#define SDHC_DAT_INHIBIT_Msk             2U
#define SDHC_CARD_INSRT_Msk              0x00010000U
#define SDHC_CARD_INSRT_Pos              0x10U
#define RD_XFER_ACTIVE_Msk                0x200U
#define WR_XFER_ACTIVE_Msk                0x100U
#define XFER_ACTIVE_Msk                   0x300U
#define RD_XFER_ACTIVE_Pos                0x9U
#define WR_XFER_ACTIVE_Pos                0x8U
#define DMA_IRQ_Pos                       0x3U
#define DMA_IRQ_Msk                       (1U << DMA_IRQ_Pos)
#define NORMAL_INT_STAT_XFER_COMPLETE_Pos 1U
#define NORMAL_INT_STAT_XFER_COMPLETE_Msk (1U << NORMAL_INT_STAT_XFER_COMPLETE_Pos)
#define SDHC_CMD_LINE_LVL_UP_Pos         23U
#define SDHC_CMD_LINE_LVL_UP_Msk         (1U << SDHC_CMD_LINE_LVL_UP_Pos)

/* Power Control */
#define SDHC_PC_BUS_PWR_OFF              0x0U        /**< Bus Power Off      */
#define SDHC_PC_BUS_PWR_VDD1_Msk         0x00000001U /**< Bus Power Control  */
#define SDHC_PC_BUS_VSEL_Msk             0x0000000EU /**< Bus Voltage Select */
#define SDHC_PC_BUS_VSEL_3V3_Msk         0x0000000EU /**< Bus Voltage 3.3V   */
#define SDHC_PC_BUS_VSEL_3V0_Msk         0x0000000CU /**< Bus Voltage 3.0V   */
#define SDHC_PC_BUS_VSEL_1V8_Msk         0x0000000AU /**< Bus Voltage 1.8V   */
#define SDHC_PC_EMMC_HW_RST_Msk          0x00000010U /**< HW reset for eMMC  */

/* Clock Control */
#define SDHC_INTERNAL_CLK_EN_Msk         0x1U
#define SDHC_INTERNAL_CLK_STABLE_Msk     0x2U
#define SDHC_CLK_EN_Msk                  0x4U
#define SDHC_PLL_EN_Msk                  0x8U
#define SDHC_CLK_GEN_SEL_Pos             0x5U
#define SDHC_DIV_CLK_MODE                0x0U
#define SDHC_PROG_CLK_MODE               0x1U
#define SDHC_CLK_GEN_SEL_Msk             (SDHC_DIV_CLK_MODE << SDHC_CLK_GEN_SEL_Pos)
#define SDHC_UPPER_FREQ_SEL_Pos          6U
#define SDHC_UPPER_FREQ_SEL_Msk          (0x3U << SDHC_UPPER_FREQ_SEL_Pos)
#define SDHC_FREQ_SEL_Pos                8U
#define SDHC_FREQ_SEL_Msk                (0xFFU << SDHC_FREQ_SEL_Pos)
#define SDHC_CLK_200KHz_DIV              (2U << SDHC_UPPER_FREQ_SEL_Pos)
#define SDHC_CLK_400KHz_DIV              (1U << SDHC_UPPER_FREQ_SEL_Pos)
#define SDHC_CLK_UPPR_FREQ_SEL           SDHC_CLK_400KHz_DIV
#define SDHC_CLK_800KHz_DIV              0x80U
#define SDHC_CLK_1_5MHz_DIV              0x40U
#define SDHC_CLK_3MHz_DIV                0x20U
#define SDHC_CLK_6MHz_DIV                0x10U
#define SDHC_CLK_12_5MHz_DIV             0x4U
#define SDHC_CLK_25MHz_DIV               0x2U
#define SDHC_CLK_50MHz_DIV               0x1U
#define SDHC_CLK_100MHz_DIV              0x0U
#define SDHC_INIT_FREQ                   SDHC_CLK_400KHz_DIV
#define SDHC_INIT_CLK_DIVSOR_Msk         (SDHC_INIT_FREQ)

/* Host Capabilities */
#define SDHC_HOST_SD_CAP_VOLT_3V3_Msk    0x01000000U /*!< 3.3V support */
#define SDHC_HOST_SD_CAP_VOLT_3V0_Msk    0x02000000U /*!< 3.0V support */
#define SDHC_HOST_SD_CAP_VOLT_1V8_Msk    0x04000000U /*!< 1.8V support */

/* Xfer Mode Control */
#define SDHC_XFER_MODE_DMA_EN_Pos        0U
#define SDHC_XFER_MODE_DMA_EN_Msk        (1U << SDHC_XFER_MODE_DMA_EN_Pos)
#define SDHC_XFER_MODE_BLK_CNT_EN_Pos    1U
#define SDHC_XFER_MODE_BLK_CNT_Msk       (1U << SDHC_XFER_MODE_BLK_CNT_EN_Pos)
#define SDHC_XFER_MODE_AUTO_CMD_EN_Pos   2U
#define SDHC_XFER_MODE_AUTO_CMD_DISABLE  0U
#define SDHC_XFER_MODE_AUTO_CMD12        1U
#define SDHC_XFER_MODE_AUTO_CMD23        2U
#define SDHC_XFER_MODE_AUTO_CMD_AUTO_SEL 3U
#define SDHC_XFER_MODE_AUTO_CMD_EN_Msk \
    (SDHC_XFER_MODE_AUTO_CMD12 << SDHC_XFER_MODE_AUTO_CMD_EN_Pos)
#define SDHC_XFER_MODE_DATA_XFER_DIR_Pos   4U
#define SDHC_XFER_MODE_DATA_XFER_RD_Msk    (1U << SDHC_XFER_MODE_DATA_XFER_DIR_Pos)
#define SDHC_XFER_MODE_DATA_XFER_WR_Msk    (0U << SDHC_XFER_MODE_DATA_XFER_DIR_Pos)
#define SDHC_XFER_MODE_MULTI_BLK_SEL_Pos   5U
#define SDHC_XFER_MODE_MULTI_BLK_SEL_Msk   (1U << SDHC_XFER_MODE_MULTI_BLK_SEL_Pos)
#define SDHC_XFER_MODE_RESP_ERR_CHK_Pos    7U
#define SDHC_XFER_MODE_RESP_ERR_CHK_EN_Msk (1U << SDHC_XFER_MODE_RESP_ERR_CHK_Pos)
#define SDHC_SIGNAL_DISABLE                 0x0U

/* DMA */
#define SDHC_HC_DMA_ADMA2_32_Msk           0x00000010U /**< ADMA2 Mode - 32 bit */

/* Wakeup Control Register Mask */
#define SDHC_WKUP_CARD_IRQ_Msk             0x00000001U /*!< Card Interrupt           */
#define SDHC_WKUP_CARD_INSRT_Msk           0x00000002U /*!< Card Insertion           */
#define SDHC_WKUP_CARD_REM_Msk             0x00000004U /*!< Card Removal             */

/* Normal IRQs Mask */
#define SDHC_INTR_CC_Msk                   0x00000001U /*!< Command Complete         */
#define SDHC_INTR_TC_Msk                   0x00000002U /*!< Transfer Complete        */
#define SDHC_INTR_BGE_Msk                  0x00000004U /*!< Block Gap Event          */
#define SDHC_INTR_DMA_Msk                  0x00000008U /*!< DMA Interrupt            */
#define SDHC_INTR_BWR_Msk                  0x00000010U /*!< Buffer Write Ready       */
#define SDHC_INTR_BRR_Msk                  0x00000020U /*!< Buffer Read Ready        */
#define SDHC_INTR_CARD_INSRT_Msk           0x00000040U /*!< Card Insert              */
#define SDHC_INTR_CARD_REM_Msk             0x00000080U /*!< Card Remove              */
#define SDHC_INTR_CARD_Msk                 0x00000100U /*!< Card Interrupt           */
#define SDHC_INTR_INT_A_Msk                0x00000200U /*!< INT A Interrupt          */
#define SDHC_INTR_INT_B_Msk                0x00000400U /*!< INT B Interrupt          */
#define SDHC_INTR_INT_C_Msk                0x00000800U /*!< INT C Interrupt          */
#define SDHC_INTR_RE_TUNING_Msk            0x00001000U /*!< Re-Tuning Interrupt      */
#define SDHC_INTR_BOOT_ACK_RECV_Msk        0x00002000U /*!< Boot Ack Recv Irq        */
#define SDHC_INTR_BOOT_TERM_Msk            0x00004000U /*!< Boot Terminate Interrupt */
#define SDHC_INTR_ERR_Msk                  0x00008000U /*!< Error Interrupt          */
#define SDHC_NORM_INTR_ALL_Msk             0x0000FFFFU

/* Error IRQs Mask */
#define SDHC_INTR_ERR_CT_Msk               0x00000001U /*!< Command Timeout Error    */
#define SDHC_INTR_ERR_CCRC_Msk             0x00000002U /*!< Command CRC Error        */
#define SDHC_INTR_ERR_CEB_Msk              0x00000004U /*!< Command End Bit Error    */
#define SDHC_INTR_ERR_CI_Msk               0x00000008U /*!< Command Index Error      */
#define SDHC_INTR_ERR_DT_Msk               0x00000010U /*!< Data Timeout Error       */
#define SDHC_INTR_ERR_DCRC_Msk             0x00000020U /*!< Data CRC Error           */
#define SDHC_INTR_ERR_DEB_Msk              0x00000040U /*!< Data End Bit Error       */
#define SDHC_INTR_ERR_CUR_LMT_Msk          0x00000080U /*!< Current Limit Error      */
#define SDHC_INTR_ERR_AUTO_CMD12_Msk       0x00000100U /*!< Auto CMD12 Error         */
#define SDHC_INTR_ERR_ADMA_Msk             0x00000200U /*!< ADMA Error               */
#define SDHC_INTR_ERR_TR_Msk               0x00001000U /*!< Tuning Error             */
#define SDHC_INTR_VEND_SPF_ERR_Msk         0x0000E000U /*!< Vendor Specific Error    */
#define SDHC_ERROR_INTR_ALL_Msk            0x0000F3FFU /*!< Mask for error bits      */

/* Timeout Constants */
#define SDHC_POWER_CYCLE_DELAY_US         1000U      /*!< Power cycle delay in microseconds */
#define SDHC_VOLTAGE_SWITCH_DELAY_US      10000U     /*!< Voltage switch delay in microseconds */
#define SDHC_CLOCK_STABLE_DELAY_US        1000U      /*!< Clock stable delay in microseconds */
#define SDHC_POST_CLOCK_ENABLE_DELAY_US   1000U      /*!< Delay after clock enable */
#define SDHC_POST_1P8V_CLOCK_DELAY_US     1000U      /*!< Clock enable delay for 1.8V switch */
#define SDHC_BUS_IDLE_TIMEOUT             0x1000U    /*!< Bus idle timeout count */
#define SDHC_POWER_CYCLE_TIMEOUT          0xFFFFU    /*!< Power cycle timeout count */
#define SDHC_CLOCK_STABLE_TIMEOUT         1000U      /*!< Clock stable timeout in microseconds */

/* Clock Constants */
#define SDHC_BASE_CLK_FREQ_MASK           0xFF00U     /*!< Base clock frequency mask */
#define SDHC_BASE_CLK_FREQ_POS            7U          /*!< Base clock frequency position */
#define SDHC_MHZ_TO_HZ                    1000000U    /*!< MHz to Hz conversion factor */
#define SDHC_MAX_DIVIDER                  0x3FFU      /*!< Maximum clock divider value */
#define SDHC_DIVIDER_LOW_MASK             0xFFU       /*!< Lower 8 bits of divider */
#define SDHC_DIVIDER_HIGH_MASK            0x03U       /*!< Upper 2 bits of divider */
#define SDHC_HS_MODE_THRESHOLD_HZ         25000000U   /*!< High speed mode threshold (25MHz) */
#define SDHC_UHS_SDR50_THRESHOLD_HZ       50000000U   /*!< SDR50/SDR104 threshold (50MHz) */
#define SDHC_INIT_CLOCK_FREQ_HZ           400000U     /*!< Initialization clock freq (400KHz) */
#define SDHC_DEFAULT_BASE_CLK_MHZ         50U         /*!< Default base clock in MHz */

/* Timeout Control */
#define SDHC_MAX_TIMEOUT_VALUE            0xE         /*!< Maximum timeout value for TOUT_CTRL */

/* SD Status */
#define SDHC_STATUS_Msk                    0x00001E00U
#define SDHC_STATUS_Pos                    9U

/* Host Control 1 */
#define SDHC_HOST_CTRL1_LED_ON             0x1U
#define SDHC_HOST_CTRL1_4_BIT_WIDTH        0x2U /*!< Host control 1 4bit mode */
#define SDHC_HOST_CTRL1_HIGH_SPEED_MODE_EN 0x4U /*!< Host control 1 High speed enable */
#define SDHC_HOST_CTRL1_SDMA_MODE          0x0U
#define SDHC_HOST_CTRL1_ADMA2_MODE         0x1U
#define SDHC_HOST_CTRL1_ADMA3_MODE         0x2U
#define SDHC_HOST_CTRL1_ADMA32_MODE_Msk    (0x2U << 3U)
#define SDHC_HOST_CTRL1_ADMA64MODE         0x3U
#define SDHC_HOST_CTRL1_DMA_SEL            SD_SEL_SDMA
#define SDHC_HOST_CTRL1_DMA_SEL_1BIT_MODE  0x0U

/* Host Control 2*/
#define SDHC_HOST_CTRL2_ASYNC_INT_EN_Msk   (1U << 14U)
#define SDHC_HOST_CTRL2_VER4_EN_Msk        (1U << 12U)
#define SDHC_HOST_CTRL2_CMD23_EN_Msk       (1U << 11U)
#define SDHC_HOST_CTRL2_SIGNALING_EN_Msk   (1U << 3U)

/* UHS Mode Select - HOST_CTRL2 bits [2:0] */
#define SDHC_HOST_CTRL2_UHS_MODESEL_Pos    0U
#define SDHC_HOST_CTRL2_UHS_MODESEL_Msk    (0x7U << SDHC_HOST_CTRL2_UHS_MODESEL_Pos)
#define SDHC_UHS_MODESEL_SDR12             0x0U  /*!< Default Speed (<=25MHz, 3.3V) */
#define SDHC_UHS_MODESEL_SDR25             0x1U  /*!< High Speed   (<=50MHz, 3.3V/1.8V) */
#define SDHC_UHS_MODESEL_SDR50             0x2U  /*!< SDR50        (50MHz, 1.8V) */
#define SDHC_UHS_MODESEL_SDR104            0x3U  /*!< SDR104       (>50MHz, 1.8V) */
#define SDHC_UHS_MODESEL_DDR50             0x4U  /*!< DDR50        (50MHz DDR, 1.8V) */

#define HC_CLOCK_DISABLE(pHsd) \
    (pHsd->regs->SDMMC_CLK_CTRL_R &= ~SDHC_CLK_EN_Msk)
#define HC_CLOCK_ENABLE(pHsd) \
    (pHsd->regs->SDMMC_CLK_CTRL_R |= SDHC_CLK_EN_Msk)

/**
  \fn          int sdhc_card_present(struct _sd_handle_t *pHsd, int (*card_det_cb)(void))
  \brief       Check if SD card is present using GPIO callback or PSTATE register
  \param[in]   pHsd: SD host handle
  \param[in]   card_det_cb: Optional GPIO card detect callback
  \return      1 if card present, 0 if not
*/
int sdhc_card_present(struct _sd_handle_t *pHsd, int (*card_det_cb)(void));

/**
  \fn          SD_DRV_STATUS sdhc_init(sd_handle_t *pHsd, sd_param_t *p_sd_param)
  \brief       Initialize SD host controller
  \param[in]   pHsd: SD host handle
  \param[in]   p_sd_param: SD parameters
  \return      SD driver status
*/
SD_DRV_STATUS sdhc_init(sd_handle_t *pHsd, sd_param_t *p_sd_param);

/* Low-level host controller API declarations */
SDHC_STATUS sdhc_send_cmd(sd_handle_t *pHsd, sd_cmd_t *pCmd);
SDHC_STATUS sdhc_set_io(sdmmc_io_t *p_sdmmc_io_param,
                         SDMMC_SET_IO_CMD set_io_cmd);
SDHC_STATUS sdhc_reset(sd_handle_t *pHsd, uint8_t reset_val);
void        sdhc_power_cycle(sd_handle_t *pHsd);
SDHC_STATUS sdhc_xfer_dma_setup(sd_handle_t *pHsd, sd_data_t *data);
SDHC_STATUS sdhc_set_blk_size(sd_handle_t *pHsd, uint32_t blk_size);
void        sdhc_set_block_count(sd_handle_t *pHsd, uint32_t blk_cnt);
SDHC_STATUS sdhc_check_xfer_done(sd_handle_t *pHsd, uint32_t timeout_cnt);
uint32_t    sdhc_get_response1(sd_handle_t *pHsd);
uint32_t    sdhc_get_response2(sd_handle_t *pHsd);
uint32_t    sdhc_get_response3(sd_handle_t *pHsd);
uint32_t    sdhc_get_response4(sd_handle_t *pHsd);
void        sdhc_enable_irq(sd_handle_t *pHsd, uint16_t mask);
void        sdhc_set_emmc_ctrl(sd_handle_t *pHsd, uint8_t value);
uint32_t    sdhc_get_blk_size(sd_handle_t *pHsd);
void        sdhc_set_led(sd_handle_t *pHsd, bool enable);

#ifdef __cplusplus
}

#endif

#endif /* __SD_HOST_H__ */
