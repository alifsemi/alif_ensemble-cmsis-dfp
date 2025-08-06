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
 * @file     sd.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    SD Driver APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "sd.h"
#include "sys_ctrl_sd.h"
#include "string.h"
#include "sys_utils.h"

#if defined(SDMMC_PRINTF_DEBUG) || defined(SDMMC_PRINT_WARN) || defined(SDMMC_PRINT_ERR) ||    \
    defined(SDMMC_PRINTF_SD_STATE_DEBUG) || defined(SDMMC_PRINT_SEC_DATA)
#include <stdio.h>
#include <inttypes.h>
#endif

/* Global SD Driver Callback definitions */
const diskio_t SD_Driver = {
    sd_init,
    sd_uninit,
    sd_state,
    sd_info,
    sd_read,
    sd_write,
    sd_set_io,
    NULL
};

/* Global SD Handle */
sd_handle_t Hsd __attribute__((section("sd_dma_buf")));

/**
 * \fn           sd_set_io
 * \brief        SD set IO common api for power and clock
 * \param[in]    SD error number
 * \return       sd driver state
 */
SD_DRV_STATUS sd_set_io(sdmmc_io_t *p_sdmmc_io_param, SDMMC_SET_IO_CMD set_io_cmd)
{
    sd_handle_t *pHsd      = &Hsd;
    SDMMC_HC_STATUS status = SDMMC_HC_STATUS_OK;

    switch (set_io_cmd) {
    case SDMMC_SET_IO_CLK:
        if (p_sdmmc_io_param->sdmmc_clock == SDMMC_CLK_DISABLE) {
            status = hc_set_clk_freq(pHsd, 0);
        } else if (p_sdmmc_io_param->sdmmc_clock == SDMMC_CLK_ENABLE) {
            status = hc_set_clk_freq(pHsd, 1);
        }
        break;

    case SDMMC_SET_IO_VOL:
        if (p_sdmmc_io_param->sdmmc_vol == SDMMC_VOL_1P8V) {
            status = hc_set_bus_power(pHsd, SDMMC_PC_BUS_VSEL_1V8_Msk);
        } else if (p_sdmmc_io_param->sdmmc_vol == SDMMC_VOL_3P3V) {
            status = hc_set_bus_power(pHsd, SDMMC_PC_BUS_VSEL_3V3_Msk);
        }
        break;

    case SDMMC_SET_IO_PWR:
        if (p_sdmmc_io_param->sdmmc_power == SDMMC_POWER_OFF) {
            status = hc_set_bus_power(pHsd, SDMMC_POWER_OFF);
        } else {
            status = hc_set_bus_power(pHsd, SDMMC_POWER_ON);
        }
        break;

    case SDMMC_SET_IO_BUS_WIDTH:
        status = hc_set_bus_width(pHsd, p_sdmmc_io_param->sdmmc_bus_width);
        break;

    default:
        return SD_DRV_STATUS_ERR;
    }

    return status ? SD_DRV_STATUS_CARD_INIT_ERR : SD_DRV_STATUS_OK;
}

/**
 * \fn           sd_switch_voltage
 * \brief        switch sd line volatge
 * \param[in]    SD Global Handle pointer
 * \param[in]    new voltage
 * \return       sd driver status
 */
SD_DRV_STATUS sd_switch_voltage(sd_handle_t *pHsd, uint8_t req_vol)
{
    uint32_t   status;
    sdmmc_io_t sdmmc_io_param;

#ifdef SDMMC_PRINT_WARN
    printf("Switching to 1.8v...\n");
#endif
    pHsd->sd_cmd.cmdidx = CMD11;
    pHsd->sd_cmd.arg    = 0;

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    sdmmc_io_param.sdmmc_vol = req_vol;
    status                   = sd_set_io(&sdmmc_io_param, SDMMC_SET_IO_VOL);

    if (status) {

#ifdef SDMMC_PRINT_ERR
        printf("Error Switching to UHS-I voltage...\n");
#endif
        return SD_DRV_STATUS_CARD_INIT_ERR;
    } else {
        return SD_DRV_STATUS_OK;
    }
}

/**
  \fn           SDErrorHandler
  \brief        SD Driver error Handler
  \param[in]    SD error number
  \return       sd driver state
  */
SD_DRV_STATUS sd_error_handler(void)
{
    sd_handle_t *pHsd = &Hsd;
    hc_reset(pHsd, SDMMC_SW_RST_DAT_Msk | SDMMC_SW_RST_CMD_Msk);
    return ~SD_DRV_STATUS_OK;
}

/**
  \fn           SD_host_init
  \brief        initialize host controller
  \param[in]    SD Global Handle pointer
  \return       sd driver status
  */
SD_DRV_STATUS sd_host_init(sd_handle_t *pHsd, sd_param_t *p_sd_param)
{

    uint8_t  powerlevel;
    uint16_t reg;

    /* Enable SDMMC Clock */
    enable_sd_periph_clk();

    /* set some default values */
    pHsd->regs  = SDMMC;
    pHsd->state = SD_CARD_STATE_INIT;

    /* copy initial device parameters */
    memcpy((void *) &pHsd->sd_param.dev_id,
           (const void *) &p_sd_param->dev_id,
           sizeof(pHsd->sd_param));

    if (Hsd.sd_param.reset_cb) {
        pHsd->sd_param.reset_cb();
    }

    /* Get the Host Controller version */
    pHsd->hc_version = *((volatile uint16_t *) (SDMMC_HC_VERSION_REG)) & SDMMC_HC_VERSION_REG_Msk;

    /* Get the Host Controller Capabilities */
    hc_get_capabilities(pHsd, &pHsd->hc_caps);

    /* Disable the SD Voltage supply */
    hc_set_bus_power(pHsd, SDMMC_POWER_OFF);

    /* Soft reset Host controller cmd and data lines */
    hc_reset(pHsd, (uint8_t) (SDMMC_SW_RST_ALL_Msk));

    /* Get the host voltage capability */
    if ((pHsd->hc_caps & SDMMC_HOST_SD_CAP_VOLT_3V3_Msk) != 0U) {
        powerlevel = SDMMC_PC_BUS_VSEL_3V3_Msk;
    } else if ((pHsd->hc_caps & SDMMC_HOST_SD_CAP_VOLT_3V0_Msk) != 0U) {
        powerlevel = SDMMC_PC_BUS_VSEL_3V0_Msk;
    } else if ((pHsd->hc_caps & SDMMC_HOST_SD_CAP_VOLT_1V8_Msk) != 0U) {
        powerlevel = SDMMC_PC_BUS_VSEL_1V8_Msk;
    } else {
        powerlevel = 0U;
    }

    hc_set_bus_power(pHsd, (uint8_t)(powerlevel));
    hc_set_tout(pHsd, 0xE);

    hc_config_interrupt(pHsd);

#ifdef SDMMC_IRQ_MODE
    NVIC_ClearPendingIRQ(SDMMC_WAKEUP_IRQ_NUM);
    NVIC_SetPriority(SDMMC_WAKEUP_IRQ_NUM, RTE_SDC_WAKEUP_IRQ_PRI);
    NVIC_EnableIRQ(SDMMC_WAKEUP_IRQ_NUM);

    NVIC_ClearPendingIRQ(SDMMC_IRQ_NUM);
    NVIC_SetPriority(SDMMC_IRQ_NUM, RTE_SDC_IRQ_PRI);
    NVIC_EnableIRQ(SDMMC_IRQ_NUM);

    /* Card Insertion and Removal State and Signal Enable */
    pHsd->regs->SDMMC_NORMAL_INT_SIGNAL_EN_R = SDMMC_INTR_CARD_INSRT_Msk | SDMMC_INTR_CARD_REM_Msk;

    pHsd->regs->SDMMC_WUP_CTRL_R =
        SDMMC_WKUP_CARD_IRQ_Msk | SDMMC_WKUP_CARD_INSRT_Msk | SDMMC_WKUP_CARD_REM_Msk;
#endif

    if (pHsd->sd_param.dma_mode == SDMMC_HOST_CTRL1_SDMA_MODE) {
        hc_config_dma(pHsd,
                      (uint8_t) (SDMMC_HOST_CTRL1_SDMA_MODE | SDMMC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    } else if (pHsd->sd_param.dma_mode == SDMMC_HOST_CTRL1_ADMA2_MODE) {
        hc_config_dma(pHsd,
                      (uint8_t) (SDMMC_HOST_CTRL1_ADMA32_MODE_Msk |
                                 SDMMC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    } else if (pHsd->sd_param.dma_mode == SDMMC_HOST_CTRL1_ADMA3_MODE) {
        /* TODO: ADMA3 mode, switching to default ADMA2 mode */
        hc_config_dma(pHsd,
                      (uint8_t) (SDMMC_HOST_CTRL1_ADMA32_MODE_Msk |
                                 SDMMC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    } else {
        /* Wrong input given by user swithing to default ADMA2 mode */
        hc_config_dma(pHsd,
                      (uint8_t) (SDMMC_HOST_CTRL1_ADMA32_MODE_Msk |
                                 SDMMC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    }

    /* Default clock settings to 400KHz */
    pHsd->sd_card.cardtype = SDMMC_CARD_SDHC;
    pHsd->sd_card.busspeed = SDMMC_CLK_400_KHZ;

    reg = SDMMC_CLK_GEN_SEL_Msk | SDMMC_INIT_CLK_DIVSOR_Msk | SDMMC_PLL_EN_Msk | SDMMC_CLK_EN_Msk |
          SDMMC_INTERNAL_CLK_EN_Msk;

    hc_set_clk_freq(pHsd, reg);

    return SD_DRV_STATUS_OK;
}

/**
  \fn           SD_card_init
  \brief        initialize card
  \param[in]    sd global handle pointer
  \return       sd driver status
  */
SD_DRV_STATUS sd_card_init(sd_handle_t *pHsd, sd_param_t *p_sd_param)
{

    uint16_t       reg;
    uint32_t       ocr;
    uint8_t        re_init_cnt    = 2;
    const uint32_t clk_div_tbl[4] = {
        SDMMC_CLK_12_5MHz_DIV,
        SDMMC_CLK_25MHz_DIV,
        SDMMC_CLK_50MHz_DIV,
        SDMMC_CLK_100MHz_DIV
    };

    /* Default settings */
    pHsd->sd_card.cardtype        = SDMMC_CARD_SDHC;
    pHsd->sd_card.busspeed        = SDMMC_CLK_400_KHZ;
    ocr                           = (SDMMC_CMD41_HCS |
                                     SDMMC_CMD41_3V3 |
                                     SDMMC_OCR_S18R);

    reg = SDMMC_CLK_GEN_SEL_Msk | SDMMC_INIT_CLK_DIVSOR_Msk | SDMMC_PLL_EN_Msk | SDMMC_CLK_EN_Msk |
          SDMMC_INTERNAL_CLK_EN_Msk;

    hc_set_clk_freq(pHsd, reg);

    pHsd->state = SD_CARD_STATE_IDLE;

    sys_busy_loop_us(100);

    /* Reset the command structure */
    pHsd->sd_cmd.cmdidx    = 0;
    pHsd->sd_cmd.arg       = 0;
    pHsd->sd_cmd.xfer_mode = 0;

    /* Check and wait till the card is present and Reset It */
    if (hc_identify_card(pHsd) != SDMMC_HC_STATUS_OK) {
        /* Card Not present */
        return SD_DRV_STATUS_TIMEOUT_ERR;
    }

RE_INIT:

    /* Reset the SD/UHS Cards */
    if (hc_go_idle(pHsd) != SDMMC_HC_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    sys_busy_loop_us(100);

    /* Get the card interface condition */
    if (hc_get_card_ifcond(pHsd) != SDMMC_HC_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    /* Get the card operating condition */
    if (hc_get_card_opcond(pHsd, ocr) != SDMMC_HC_STATUS_OK) {
        if (hc_get_emmc_card_opcond(pHsd) != SDMMC_HC_STATUS_OK) {
            return SD_DRV_STATUS_CARD_INIT_ERR;  // No Valid Card Found
        } else {
            pHsd->sd_card.cardtype = SDMMC_CARD_MMC;
        }
    }

    if (pHsd->sd_card.flags == SDMMC_1P8V_FLAG) {

        if (sd_switch_voltage(pHsd, SDMMC_VOL_1P8V)) {

            if (re_init_cnt--) {

                if (Hsd.sd_param.reset_cb) {
                    pHsd->sd_param.reset_cb();
                }

                /* 1.8V switch Failed Re-init with 3.3V */
                ocr                 &= ~SDMMC_OCR_S18R;
                pHsd->sd_card.flags &= ~SDMMC_1P8V_FLAG;

                /* Disable the SD Voltage supply */
                hc_set_bus_power(pHsd, SDMMC_PC_BUS_VSEL_3V3_Msk);

                sys_busy_loop_us(SDMMC_RESET_DELAY_US);

                hc_reset(pHsd, SDMMC_SW_RST_CMD_Msk | SDMMC_SW_RST_DAT_Msk);
                goto RE_INIT;

            } else {
                return SD_DRV_STATUS_CARD_INIT_ERR;
            }
        }
    }

    if (!(pHsd->sd_card.sdio_mode)) {

        /* Get the card ID CMD2 */
        if (hc_get_card_cid(pHsd) != SDMMC_HC_STATUS_OK) {
            return SD_DRV_STATUS_CARD_INIT_ERR;
        }
    }

    /* Get the card Relative Address CMD3 */
    if (hc_get_rca(pHsd, &pHsd->sd_card.relcardadd) != SDMMC_HC_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    if (!(pHsd->sd_card.sdio_mode)) {

        /* Get the CSD register */
        if (hc_get_card_csd(pHsd) != SDMMC_HC_STATUS_OK) {
            return SD_DRV_STATUS_CARD_INIT_ERR;
        }

        sdmmc_decode_card_csd(pHsd);

        if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {
            /* Get the SCR register */
            if (hc_get_card_scr(pHsd) != SDMMC_HC_STATUS_OK) {
                return SD_DRV_STATUS_CARD_INIT_ERR;
            }
        }

        /* Change the Card State from Identification to Ready */
        pHsd->state = SD_CARD_STATE_STBY;
    }

    /* Select a card */
    if (hc_sel_card(pHsd, pHsd->sd_card.relcardadd) != SDMMC_HC_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    if (!(pHsd->sd_card.sdio_mode)) {

        if (pHsd->sd_param.bus_width >= SDMMC_4_BIT_MODE) {
            if (hc_set_bus_width(pHsd, pHsd->sd_param.bus_width) != SDMMC_HC_STATUS_OK) {
                return SD_DRV_STATUS_CARD_INIT_ERR;
            }
        }

        if (hc_set_blk_size(pHsd, SDMMC_BLK_SIZE_512_Msk) != SDMMC_HC_STATUS_OK) {
            return SD_DRV_STATUS_CARD_INIT_ERR;
        }
    }

    /* Check Configured running clock */
    if (p_sd_param->clock_id < 4) {
        reg = clk_div_tbl[p_sd_param->clock_id];
    } else {
        reg = SDMMC_CLK_50MHz_DIV; /* Switch default to 50MHz */
    }

    reg = (reg << SDMMC_FREQ_SEL_Pos) | SDMMC_CLK_GEN_SEL_Msk | SDMMC_PLL_EN_Msk |
          SDMMC_CLK_EN_Msk | SDMMC_INTERNAL_CLK_EN_Msk;

    hc_set_clk_freq(pHsd, reg);

    return SD_DRV_STATUS_OK;
}

/**
  \fn           SD_init
  \brief        main SD initialize function
  \param[in]    device ID
  \return       sd driver status
  */
SD_DRV_STATUS sd_init(sd_param_t *p_sd_param)
{

    SD_DRV_STATUS errcode = SD_DRV_STATUS_OK;

    /* Initialize Host controller */
    errcode               = sd_host_init(&Hsd, p_sd_param);

    if (errcode != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_HOST_INIT_ERR;
    }

    /* Initialize SD Memory/Combo Card */
    errcode = sd_card_init(&Hsd, p_sd_param);

    if (errcode != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    return SD_DRV_STATUS_OK;
}

/**
  \fn           sdmmc_decode_card_ext_csd
  \brief        decode emmc ext csd data
  \param[in]    sd handle pointer
  \param[in]    raw ext csd data pointer
  \return       none
  */
void sdmmc_decode_card_ext_csd(sd_handle_t *pHsd, uint8_t *praw_ext_csd)
{

    RTSS_InvalidateDCache_by_Addr(praw_ext_csd, SDMMC_EXT_CSD_SIZE);

    pHsd->mmc_ext_csd.sector_cnt = *((uint32_t *) (praw_ext_csd + 212));

    pHsd->mmc_ext_csd.bus_width  = praw_ext_csd[SDMMC_EXT_CSD_CMD_BUS_WIDTH];
    pHsd->mmc_ext_csd.hs_mode    = praw_ext_csd[SDMMC_EXT_CSD_CMD_HS_MODE];

    /* get the device type */
    pHsd->mmc_ext_csd.device_type =
        ((1 << 7U) & praw_ext_csd[196U]) | ((1 << 6U) & praw_ext_csd[196U]) |
        ((1 << 5U) & praw_ext_csd[196U]) | ((1 << 5U) & praw_ext_csd[196U]) |
        ((1 << 3U) & praw_ext_csd[196U]) | ((1 << 2U) & praw_ext_csd[196U]) |
        ((1 << 1U) & praw_ext_csd[196U]) | ((1 << 0U) & praw_ext_csd[196U]);
    pHsd->mmc_ext_csd.mmc_ext_csd_ver  = praw_ext_csd[192];
    pHsd->mmc_ext_csd.power_class      = praw_ext_csd[187] & 0xF;
    pHsd->mmc_ext_csd.mmc_drv_strength = praw_ext_csd[197];
    pHsd->mmc_ext_csd.cache_size       = (praw_ext_csd[252] << 24U) + (praw_ext_csd[251] << 16U) +
                                   (praw_ext_csd[250] << 8U) + (praw_ext_csd[249] << 0U);

    pHsd->sd_card.sectorcount  = pHsd->mmc_ext_csd.sector_cnt;
    pHsd->sd_card.sectorsize   = SDMMC_BLK_SIZE_512_Msk;
    pHsd->sd_card.logblocksize = SDMMC_BLK_SIZE_512_Msk;

    return;
}

/**
  \fn           sdmmc_decode_card_csd
  \brief        decode card csd data
  \param[in]    sd handle pointer
  \return       none
  */
void sdmmc_decode_card_csd(sd_handle_t *pHsd)
{
    uint32_t blk_len;
    uint32_t device_size;
    uint32_t mult, c_size;

    if (((pHsd->csd[RAW_CSD_BUF_IDX3] & CSD_STRUCT_Msk) >> CSD_STRUCT_Pos) == 0U) {
        blk_len = (uint32_t) 1U << ((uint32_t) (pHsd->csd[RAW_CSD_BUF_IDX2] & READ_BLK_LEN_Msk) >>
                                    CSD_V1_BLK_LEN_Pos);
        mult =
            (uint32_t) 1U << ((uint32_t) ((pHsd->csd[RAW_CSD_BUF_IDX1] & C_SIZE_MULT_Msk) >> 7U) +
                              (uint32_t) 2U);
        device_size  = (pHsd->csd[RAW_CSD_BUF_IDX1] & C_SIZE_LOWER_Msk) >> C_SIZE_LOWER_Pos;
        device_size |= (pHsd->csd[RAW_CSD_BUF_IDX2] & C_SIZE_UPPER_Msk) << C_SIZE_UPPER_Pos;
        device_size  = (device_size + 1U) * mult;
        device_size  = device_size * blk_len;
        pHsd->sd_card.sectorcount  = (device_size / SDMMC_BLK_SIZE_512_Msk);
        pHsd->sd_card.sectorsize   = blk_len;
        pHsd->sd_card.logblocksize = blk_len;
    } else if (((pHsd->csd[RAW_CSD_BUF_IDX3] & CSD_STRUCT_Msk) >> CSD_STRUCT_Pos) == 1U) {
        c_size = ((pHsd->csd[RAW_CSD_BUF_IDX1] & CSD_V2_C_SIZE_Msk) >> CSD_V2_C_SIZE_Pos);

        if (c_size >= SDMMC_SDHC_MAX_SECTOR_CNT) {
            pHsd->sd_card.cardtype = SDMMC_CARD_SDXC;
        }

        pHsd->sd_card.sectorcount = (c_size + 1U) * 1024U;
        pHsd->sd_card.card_class =
            (uint16_t) ((pHsd->csd[RAW_CSD_BUF_IDX2] & CSD_CCC_Msk) >> CSD_CCC_SHIFT);
        pHsd->sd_card.sectorsize   = SDMMC_BLK_SIZE_512_Msk;
        pHsd->sd_card.logblocksize = SDMMC_BLK_SIZE_512_Msk;
    } else if (((pHsd->csd[RAW_CSD_BUF_IDX3] & CSD_STRUCT_Msk) >> CSD_STRUCT_Pos) == 3U) {
        /* get ext csd data for eMMC */
        if (hc_get_card_ext_csd(pHsd, &pHsd->sd_cmd.card_buffer[0]) == SDMMC_HC_STATUS_OK) {
            if (hc_check_xfer_done(pHsd, SDMMC_DATA_TIMEOUT) == SDMMC_HC_STATUS_OK) {
                sdmmc_decode_card_ext_csd(pHsd, &pHsd->sd_cmd.card_buffer[0]);
            }
        }
    }
    return;
}

/**
  \fn           SD_uninit
  \brief        SD uninitialize function
  \param[in]    device ID
  \return       sd driver status
  */
SD_DRV_STATUS sd_uninit(uint8_t devId)
{
    ARG_UNUSED(devId);

    /* release the card */
    sd_handle_t *pHsd         = &Hsd;
    pHsd->sd_cmd.cmdidx       = CMD7;
    pHsd->sd_cmd.data_present = 0;
    pHsd->sd_cmd.arg          = 0x00000000;  // any other RCA to perform card de-selection

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    /* turn off power supply to the card */
    hc_set_bus_power(pHsd, (uint8_t) (0));
    return SD_DRV_STATUS_OK;
}

/**
  \fn           sd_state
  \brief        sd state
  \param[in]    void
  \return       sd state
  */
SD_CARD_STATE sd_state(void)
{
    sd_handle_t *pHsd = &Hsd;
    uint32_t     status;

    hc_get_card_status(pHsd, &status);

    return status;
}

/**
  \fn           sd_info
  \brief        returns sd card info
  \param[in]    sd_cardinfo_t *
  \return       sd driver status
  */
SD_DRV_STATUS sd_info(sd_cardinfo_t *pinfo)
{

    sd_handle_t *pHsd = &Hsd;

    if (pinfo == NULL) {
        return SD_DRV_STATUS_ERR;
    }

    memcpy(pinfo, &pHsd->sd_card, sizeof(sd_cardinfo_t));

    return SD_DRV_STATUS_OK;
}
/**
  \fn           SD_DRV_STATUS SD_read(uint32_t sec, uint32_t blk_cnt, volatile unsigned char *
  dest_buff) \brief        read sd sector \param[in]    sec - input sector number to read \param[in]
  blk_cnt - number of block to read \param[in]    dest_buff - Destination buffer pointer \return sd
  driver status
  */
SD_DRV_STATUS sd_read(uint32_t sec, uint16_t blk_cnt, volatile unsigned char *dest_buff)
{

    sd_handle_t *pHsd        = &Hsd;
    uint32_t     timeout_cnt = SDMMC_DATA_TIMEOUT * blk_cnt;
    uint8_t      retry_cnt   = 1;

    if (dest_buff == NULL) {
        return SD_DRV_STATUS_RD_ERR;
    }

#ifdef SDMMC_PRINTF_DEBUG
    printf("SD READ Dest Buff: 0x%08" PRIxPTR " Sec: %" PRIu32 ", Block Count: %" PRIu16 "\n",
           (uintptr_t) dest_buff,
           sec,
           blk_cnt);
#endif

    /* Change the Card State from Tran to Data */
    pHsd->state = SD_CARD_STATE_DATA;

#ifdef SDMMC_IRQ_MODE

    (void) timeout_cnt;
    (void) retry_cnt;
    hc_read_setup(pHsd, LocalToGlobal((const volatile void *) dest_buff), sec, blk_cnt);

#else

retry:
    hc_read_setup(pHsd, LocalToGlobal((const volatile void *) dest_buff), sec, blk_cnt);

    if (hc_check_xfer_done(pHsd, timeout_cnt) == SDMMC_HC_STATUS_OK) {
        RTSS_InvalidateDCache_by_Addr(dest_buff, blk_cnt * SDMMC_BLK_SIZE_512_Msk);
    } else {
        /* Soft reset Host controller cmd and data lines */
        hc_reset(pHsd, (uint8_t) (SDMMC_SW_RST_DAT_Msk | SDMMC_SW_RST_CMD_Msk));

        if (!retry_cnt--) {
            return SD_DRV_STATUS_RD_ERR;
        }
        goto retry;
    }
#endif

    /* Change the Card State from Data to Tran */
    pHsd->state = SD_CARD_STATE_TRAN;

#ifdef SDMMC_PRINT_SEC_DATA
    uint32_t  j = 0;
    uint32_t *p = (uint32_t *) dest_buff;

    RTSS_InvalidateDCache_by_Addr(dest_buff, blk_cnt * 512);

    while (j < (128 * blk_cnt)) {
        printf("0x%08" PRIx32 ": %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\n",
               j * 4,
               p[j + 0],
               p[j + 1],
               p[j + 2],
               p[j + 3]);
        j += 4;
    }
#endif

    return SD_DRV_STATUS_OK;
}

/**
  \fn           SD_DRV_STATUS SD_write(uint32_t sec, uint32_t blk_cnt, volatile unsigned char *
  src_buff) \brief        Write sd sector \param[in]    sector - input sector number to write
  \param[in]    blk_cnt - number of block to write
  \param[in]    src_buff - Source buffer pointer
  \return       sd driver status
  */
SD_DRV_STATUS sd_write(uint32_t sector, uint32_t blk_cnt, volatile unsigned char *src_buff)
{

    sd_handle_t *pHsd = &Hsd;
#ifndef SDMMC_IRQ_MODE
    int     timeout_cnt = 2000 * blk_cnt;
    uint8_t retry_cnt   = 1;
#endif
    uint8_t *aligned_buff = (uint8_t *) src_buff;
    if (src_buff == NULL) {
        return SD_DRV_STATUS_WR_ERR;
    }

#ifdef SDMMC_PRINTF_DEBUG
    printf("SD WRITE Src Buff: 0x%08" PRIxPTR " Sec: %" PRId32 ", Block Count: %" PRId32 "\n",
           (uintptr_t) src_buff,
           sector,
           blk_cnt);
#endif

    if ((uint32_t) src_buff & (SDMMC_BLK_SIZE_512_Msk - 1)) {
#ifdef SDMMC_PRINT_WARN
        printf("SD Write ADMA access un-aligned buffer ...\n");
#endif
        if (blk_cnt <= SDMMC_CACHED_NUM_BLK) {
            memcpy(&pHsd->sd_cmd.card_buffer[0], (const void *) src_buff, blk_cnt * 512);
            aligned_buff = &pHsd->sd_cmd.card_buffer[0];
        } else {
            /* return write error in case of un-aligned buffer and
             * length more than internal buffer size(4K)
             */
            return SD_DRV_STATUS_WR_ERR;
        }
    }

    /* Clean the DCache */
    RTSS_CleanDCache_by_Addr(aligned_buff, blk_cnt * SDMMC_BLK_SIZE_512_Msk);

#ifndef SDMMC_IRQ_MODE
retry:
#endif

    hc_write_setup(pHsd, LocalToGlobal((const volatile void *) aligned_buff), sector, blk_cnt);

#ifndef SDMMC_IRQ_MODE
    if (hc_check_xfer_done(pHsd, timeout_cnt) != SDMMC_HC_STATUS_OK) {
        hc_reset(pHsd, (uint8_t) (SDMMC_SW_RST_DAT_Msk | SDMMC_SW_RST_CMD_Msk));

        if (!retry_cnt--) {
            return SD_DRV_STATUS_WR_ERR;
        }
        goto retry;
    }
#endif

#ifdef SDMMC_PRINT_SEC_DATA
    uint32_t  j = 0;
    uint32_t *p = (uint32_t *) src_buff;

    while (j < (128 * blk_cnt)) {
        printf("0x%08" PRIx32 ": %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\n",
               j * 4,
               p[j + 0],
               p[j + 1],
               p[j + 2],
               p[j + 3]);
        j += 4;
    }
#endif

    return SD_DRV_STATUS_OK;
}
