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
#include "sd_host.h"
#include "sdio.h"
#include "sys_ctrl_sd.h"
#include "string.h"
#include "sys_utils.h"

/* Global SD Handle */
sd_handle_t Hsd __attribute__((section("sd_dma_buf")));

/* Global SD Driver Callback definitions */
const diskio_t SD_Driver = {
    sd_init,
    sd_uninit,
    sd_state,
    sd_info,
    sd_read,
    sd_write,
    sd_set_io,
#ifdef SDMMC_IRQ_MODE
    NULL
#endif
};

/**
 * \fn           SD_DRV_STATUS sd_set_io(sdmmc_io_t *p_sdmmc_io_param, SDMMC_SET_IO_CMD set_io_cmd)
 * \brief        SD set IO wrapper for voltage/clock/power control
 * \param[in]    p_sdmmc_io_param - SD IO parameters
 * \param[in]    set_io_cmd - IO command type
 * \return       SD driver status
 */
SD_DRV_STATUS sd_set_io(sdmmc_io_t *p_sdmmc_io_param, SDMMC_SET_IO_CMD set_io_cmd)
{
    return (SD_DRV_STATUS)sdhc_set_io(p_sdmmc_io_param, set_io_cmd);
}

/**
 * \fn           static SD_DRV_STATUS sd_error_handler(void)
 * \brief        SD Driver error Handler
 * \param[in]    SD error number
 * \return       sd driver state
 */
static SD_DRV_STATUS sd_error_handler(void)
{
    sd_handle_t *pHsd = &Hsd;
    sdhc_reset(pHsd, SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk);
    return ~SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_switch_voltage(sd_handle_t *pHsd, uint8_t req_vol)
 * \brief        switch sd line volatge
 * \param[in]    SD Global Handle pointer
 * \param[in]    new voltage
 * \return       sd driver status
 */
static SD_DRV_STATUS sd_switch_voltage(sd_handle_t *pHsd, uint8_t req_vol)
{
    uint32_t   status;
    sdmmc_io_t sdmmc_io_param = {0};

    pHsd->sd_cmd.cmdidx   = CMD11;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = 0;

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    sdmmc_io_param.sdmmc_vol = req_vol;
    status = sd_set_io(&sdmmc_io_param, SDMMC_SET_IO_VOL);

    if (status) {
        SD_LOG_DBG("1.8V switch: failed");
        return SD_DRV_STATUS_CARD_INIT_ERR;
    } else {
        SD_LOG_DBG("1.8V switch: success");
        return SD_DRV_STATUS_OK;
    }
}

/**
 * \fn           static SD_DRV_STATUS sd_identify_card(sd_handle_t *pHsd)
 * \brief        Identify the inserted cards
 * \param[in]    Global SD Handle pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_identify_card(sd_handle_t *pHsd)
{

    uint32_t        isCardPresent = false;
    SD_DRV_STATUS status;

    sdhc_reset(pHsd,
             (uint8_t) (SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk));
    SD_LOG_DBG("Waiting for SD Card to be inserted...");
    /* Card Insertion and Removal State and Signal Enable */
    uint16_t irq_mask = SDHC_INTR_CC_Msk | SDHC_INTR_TC_Msk | SDHC_INTR_DMA_Msk |
                        SDHC_INTR_BWR_Msk | SDHC_INTR_BRR_Msk |
                        SDHC_INTR_CARD_INSRT_Msk | SDHC_INTR_CARD_REM_Msk;
    sdhc_enable_irq(pHsd, irq_mask);

    /* Check card present via GPIO callback or PSTATE register */
    isCardPresent = sdhc_card_present(pHsd, pHsd->sd_param.card_det_cb);
    pHsd->sd_card.iscardpresent = isCardPresent;

    if (!isCardPresent) {
        SD_LOG_ERR("SD Card not detected");
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    SD_LOG_DBG("SD Card Detected (GPIO check)");

    status = SD_DRV_STATUS_OK;

    return status;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_card_ifcond(sd_handle_t *pHsd)
 * \brief        Get Card Interface condition
 * \param[in]    Global sd Handle pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_card_ifcond(sd_handle_t *pHsd)
{

    uint32_t resp_ifcond;

    /* Change the Card State from Idle to Identification */
    pHsd->state         = SD_CARD_STATE_IDENT;

    pHsd->sd_cmd.cmdidx   = CMD8;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = SDMMC_CMD8_VOL_PATTERN;
    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    resp_ifcond = sdhc_get_response1(pHsd);

    if (resp_ifcond != SDMMC_CMD8_VOL_PATTERN) {
        /* SD Version 1 Card */
        pHsd->sd_card.cardversion = SDMMC_CARD_SDSC;
        pHsd->sd_card.cardtype    = SDMMC_CARD_SDSC;
        pHsd->sd_card.f8flag      = true;
    } else {
        /* SD Version 2 or Later Card */
        pHsd->sd_card.cardversion = SDMMC_CARD_SDHC;
        pHsd->sd_card.cardtype    = SDMMC_CARD_SDHC;
        pHsd->sd_card.f8flag      = false;
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_go_idle(sd_handle_t * pHsd)
 * \brief        put the card in idle mode
 * \param[in]    Global sd Handle pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_go_idle(sd_handle_t *pHsd)
{

    pHsd->sd_cmd.cmdidx   = CMD0;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_NONE;
    pHsd->sd_cmd.arg      = 0;
    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->state = SD_CARD_STATE_IDLE;

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_emmc_card_opcond(sd_handle_t *pHsd)
 * \brief        Get eMMC Card operating voltage condition
 * \param[in]    Global sd Handle pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_emmc_card_opcond(sd_handle_t *pHsd)
{

    uint32_t resp_OPcond;
    uint32_t timeout   = SDMMC_CMD_OCR_TOUT;
    uint32_t switch1v8 = false;

    sdhc_power_cycle(pHsd);

    sdhc_reset(pHsd, SDHC_SW_RST_CMD_Msk | SDHC_SW_RST_DAT_Msk);
    sd_go_idle(pHsd);

    do {

        pHsd->sd_cmd.cmdidx   = CMD1;
        pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
        pHsd->sd_cmd.arg      = (SDMMC_CMD41_HCS | SDMMC_CMD41_3V3);  // | SD_OCR_S18R);
        if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
            sd_error_handler();
        }

        resp_OPcond = sdhc_get_response1(pHsd);
        switch1v8   = resp_OPcond & SDMMC_CMD41_S18A;
        resp_OPcond = sdhc_get_response1(pHsd);
        resp_OPcond = resp_OPcond & SDMMC_OCR_READY;
    } while ((!resp_OPcond) && (timeout--));

    if (timeout == SDMMC_MAX_TIMEOUT_32) {
        return SD_DRV_STATUS_ERR;
    }

    /* UHS-I Specific Initializations */
    if (switch1v8) {
        pHsd->sd_card.flags |= SDMMC_1P8V_FLAG;
    }

    /* detected card is eMMC */
    sdhc_set_emmc_ctrl(pHsd, 1);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_card_opcond(sd_handle_t *pHsd, uint32_t req_ocr)
 * \brief        Get Card operating voltage condition
 * \param[in]    Global sd Handle pointer
 * \param[in]    requested OCR value
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_card_opcond(sd_handle_t *pHsd, uint32_t req_ocr)
{

    uint32_t resp_OPcond;
    uint32_t timeout   = SDMMC_CMD_OCR_TOUT;   /* Added extra delay for diff types of card */
    uint32_t switch1v8 = false;
    uint32_t ocr;
    uint32_t sdio_func_number = 0;

    /* Get the SD/SDIO card operating condition */
    if (sdio_get_opcond(pHsd, 0, &resp_OPcond) == SD_DRV_STATUS_OK) {
        /* IO functionality found in this card */

        sdio_func_number = (resp_OPcond & CMD5_RESP_NIOF_PRES_Msk) >>
                          CMD5_RESP_NIOF_PRES_Pos;
        ocr              = (resp_OPcond & CMD5_RESP_OCR_3V3_Msk);

        if (sdio_func_number && ocr) {
            sdio_get_opcond(pHsd, ocr, &resp_OPcond);
        }

        if (resp_OPcond & CMD5_RESP_IO_READY_Msk) {

            /* update the global instance */
            pHsd->sd_card.sdio.sdio_opcd.func_number = sdio_func_number;
            pHsd->sd_card.sdio_mode                  = true;
        }

    } else {
        /* Memory */

        sdhc_power_cycle(pHsd);

        sdhc_reset(pHsd, SDHC_SW_RST_CMD_Msk | SDHC_SW_RST_DAT_Msk);
        sd_go_idle(pHsd);
        sd_get_card_ifcond(pHsd);

        do {
            /* just to indicate SD Card that the next cmd is APP CMD */
            pHsd->sd_cmd.cmdidx   = CMD55;
            pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
            pHsd->sd_cmd.arg      = 0x0;
            if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
                sd_error_handler();
            }

            pHsd->sd_cmd.cmdidx       = CMD41;
            pHsd->sd_cmd.rsp_type     = SDMMC_RESP_R48;
            pHsd->sd_cmd.arg          = req_ocr;

            if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
                sd_error_handler();
            }

            resp_OPcond = sdhc_get_response1(pHsd);
            switch1v8   = resp_OPcond & SDMMC_CMD41_S18A;
            resp_OPcond = sdhc_get_response1(pHsd);
            resp_OPcond = resp_OPcond & SDMMC_OCR_READY;
        } while ((!resp_OPcond) && (timeout--));
    }

    if (timeout == SDMMC_MAX_TIMEOUT_32) {
        return SD_DRV_STATUS_ERR;
    }

    if (!(sdhc_get_response1(pHsd) & SDMMC_CMD41_HCS)) {
        pHsd->sd_card.cardversion = SDMMC_CARD_SDSC;
        pHsd->sd_card.cardtype    = SDMMC_CARD_SDSC;
    }

    /* UHS-I Specific Initializations */
    if (switch1v8) {
        pHsd->sd_card.flags |= SDMMC_1P8V_FLAG;
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_card_cid(sd_handle_t *pHsd)
 * \brief        Get Card Identification information
 * \param[in]    Global sd Handle pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_card_cid(sd_handle_t *pHsd)
{

    pHsd->sd_cmd.cmdidx   = CMD2;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R136;
    pHsd->sd_cmd.arg      = 0x0;
    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_card.cid[0] = sdhc_get_response1(pHsd);
    pHsd->sd_card.cid[1] = sdhc_get_response2(pHsd);
    pHsd->sd_card.cid[2] = sdhc_get_response3(pHsd);
    pHsd->sd_card.cid[3] = sdhc_get_response4(pHsd);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_rca(sd_handle_t *pHsd, uint32_t *prca)
 * \brief        Get the Card Relative Address
 * \param[in]    Global sd Handle pointer
 * \param[in]    RCA destination pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_rca(sd_handle_t *pHsd, uint32_t *prca)
{
    /* Get the card Relative Addr */
    pHsd->sd_cmd.cmdidx   = CMD3;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;

    if (pHsd->sd_card.cardtype == SDMMC_CARD_MMC) {
        pHsd->sd_cmd.arg = EMMC_DEFAULT_RCA;
        *prca            = EMMC_DEFAULT_RCA;
    }
    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {
        *prca = sdhc_get_response1(pHsd) & SDMMC_RCA_Msk;
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_card_csd(sd_handle_t *pHsd)
 * \brief        Get Card Specific Data
 * \param[in]    Global sd Handle pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_card_csd(sd_handle_t *pHsd)
{

    pHsd->sd_cmd.cmdidx   = CMD9;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R136;
    pHsd->sd_cmd.arg      = pHsd->sd_card.relcardadd;
    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        return SD_DRV_STATUS_ERR;
    }

    /* update the global instance */
    pHsd->sd_card.csd[0] = sdhc_get_response1(pHsd);
    pHsd->sd_card.csd[1] = sdhc_get_response2(pHsd);
    pHsd->sd_card.csd[2] = sdhc_get_response3(pHsd);
    pHsd->sd_card.csd[3] = sdhc_get_response4(pHsd);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_sel_card(sd_handle_t *pHsd, uint32_t rca)
 * \brief        Select a card for further operation
 * \param[in]    Global sd Handle pointer
 * \param[in]    Card Relative Address
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_sel_card(sd_handle_t *pHsd, uint32_t rca)
{

    SD_LOG_DBG("CMD7: Selecting card with RCA 0x%x", rca);

    /* Select the card to transition to transfer state */
    pHsd->sd_cmd.cmdidx   = CMD7;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = rca;
    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        SD_LOG_ERR("CMD7 failed");
        sd_error_handler();
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    /* Change the Card State from Ready to Tran */
    pHsd->state = SD_CARD_STATE_TRAN;

    SD_LOG_DBG("CMD7: Card selected, state set to TRAN");

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_set_blk_size(sd_handle_t *pHsd, uint32_t blk_size)
 * \brief        Set the Block Size (protocol + register)
 * \param[in]    Global sd Handle pointer
 * \param[in]    Block size
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_set_blk_size(sd_handle_t *pHsd, uint32_t blk_size)
{

    if (sdhc_get_blk_size(pHsd) != blk_size) {

        /* set the block size */
        pHsd->sd_cmd.cmdidx   = CMD16;
        pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
        pHsd->sd_cmd.arg      = blk_size;
        if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
            sd_error_handler();
        }

        sdhc_set_blk_size(pHsd, blk_size);
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_card_ext_csd(sd_handle_t *pHsd, uint8_t *pbuff)
 * \brief        Get eMMC Extended CSD Data
 * \param[in]    Global sd Handle pointer
 * \param[in]    buffer pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_card_ext_csd(sd_handle_t *pHsd, uint8_t *pbuff)
{

    /* Select a card */
    if (sd_sel_card(pHsd, pHsd->sd_card.relcardadd) != SD_DRV_STATUS_OK) {
        sd_error_handler();
    }

    if (sd_set_blk_size(pHsd, SDMMC_BLK_SIZE_512_Msk) != SD_DRV_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_cmd.cmdidx       = CMD8;
    pHsd->sd_cmd.rsp_type     = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg          = 0;
    pHsd->sd_cmd.data_present = true;
    pHsd->sd_cmd.retries      = SDMMC_DATA_RETRIES;
    pHsd->sd_cmd.data.buffer    = (uint32_t) pbuff;
    pHsd->sd_cmd.data.blk_size  = SDMMC_BLK_SIZE_512_Msk;
    pHsd->sd_cmd.data.blk_cnt   = 1;
    pHsd->sd_cmd.data.direction = SD_DATA_DIR_READ;
    pHsd->sd_cmd.xfer_mode      = SDHC_XFER_MODE_DATA_XFER_RD_Msk |
                                  SDHC_XFER_MODE_DMA_EN_Msk;

    sdhc_xfer_dma_setup(pHsd, &pHsd->sd_cmd.data);

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_cmd.data_present = false;
    pHsd->sd_cmd.retries      = SDMMC_CMD_RETRIES;

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_card_scr(sd_handle_t *pHsd)
 * \brief        Get SCR
 * \param[in]    Global sd Handle pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_card_scr(sd_handle_t *pHsd)
{

    /* just to indicate SD Card that the next cmd is APP CMD */
    pHsd->sd_cmd.cmdidx   = CMD55;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = 0x0;

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_cmd.cmdidx   = CMD51;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = 0;

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    /* update the global instance */
    pHsd->sd_card.scr[0] = sdhc_get_response1(pHsd);
    pHsd->sd_card.scr[1] = sdhc_get_response2(pHsd);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_get_card_status(sd_handle_t *pHsd, uint32_t *pstatus)
 * \brief        Get the Card Status
 * \param[in]    Global sd Handle pointer
 * \param[in]    status pointer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_get_card_status(sd_handle_t *pHsd, uint32_t *pstatus)
{

    uint32_t status;

    /* Check current card status */
    pHsd->sd_cmd.cmdidx   = CMD13;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = pHsd->sd_card.relcardadd;

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        SD_LOG_ERR("CMD7 failed");
        sd_error_handler();
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    status   = sdhc_get_response1(pHsd);

    *pstatus = (status & SDHC_STATUS_Msk) >> SDHC_STATUS_Pos;

    SD_LOG_DBG("Card Status: %" PRIx32, status);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_set_bus_width(sd_handle_t *pHsd, uint8_t buswidth)
 * \brief        Configure required bus width for Host and Card
 * \param[in]    pHsd - Global SD Handle pointer
 * \param[in]    buswidth - number of Data lines for data transfer
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_set_bus_width(sd_handle_t *pHsd, uint8_t buswidth)
{

    uint32_t status;
    uint32_t timeout_cnt = SDMMC_CMD_TIMEOUT;
    sdmmc_io_t io_param = {0};

    do {

        sd_get_card_status(pHsd, &status);
        if (!timeout_cnt--) {
            return SD_DRV_STATUS_ERR; /* SD Must be TRAN state to change Bus width */
        }
    } while (status != SD_CARD_STATE_TRAN);

    if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {
        if (buswidth > SDMMC_4_BIT_MODE) {
            /* invalid initial parameter, switching back to max */
            /* supported bus width for sd card */
            SD_LOG_WRN("invalid bus width, switching to 4-bit mode");
            buswidth                 = SDMMC_4_BIT_MODE;
            pHsd->sd_param.bus_width = SDMMC_4_BIT_MODE;
        }
    }

    /* just to indicate Card that the next cmd is APP CMD if not MMC/eMMC Card */
    if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {

        pHsd->sd_cmd.cmdidx   = CMD55;
        pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
        pHsd->sd_cmd.arg      = pHsd->sd_card.relcardadd;

        if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
            sd_error_handler();
        }

        pHsd->sd_cmd.cmdidx   = CMD6;
        pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
        pHsd->sd_cmd.arg      = 0x2;
        if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
            sd_error_handler();
        }

    } else {

        /* Send CMD6 to write protect boot partition 1 */
        pHsd->sd_cmd.cmdidx   = CMD6;
        pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
        pHsd->sd_cmd.arg      = SDMMC_EXT_CSD_WRITE_Msk |
                           SDMMC_EXT_CSD_IDX_Msk(SDMMC_EXT_CSD_CMD_BOOT_CFG) |
                           (SDMMC_EXT_CSD_BOOT_WR_PROTECT << SDMMC_EXT_CSD_VAL_Pos);
        if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
            sd_error_handler();
        }
        sys_busy_loop_us(SDMMC_CMD6_DELAY);

        /* Switch HS timing */
        pHsd->sd_cmd.arg = SDMMC_EXT_CSD_WRITE_Msk |
                           SDMMC_EXT_CSD_IDX_Msk(SDMMC_EXT_CSD_CMD_HS_MODE) |
                           (SDMMC_EXT_CSD_HS_MODE << SDMMC_EXT_CSD_VAL_Pos);
        if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
            sd_error_handler();
        }
        sys_busy_loop_us(SDMMC_CMD6_DELAY);

        /* Send ACMD6 to change Bus width */
        pHsd->sd_cmd.arg = SDMMC_EXT_CSD_WRITE_Msk |
                           SDMMC_EXT_CSD_IDX_Msk(SDMMC_EXT_CSD_CMD_BUS_WIDTH) |
                           (buswidth << SDMMC_EXT_CSD_VAL_Pos);
        if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
            sd_error_handler();
        }
        sys_busy_loop_us(SDMMC_CMD6_DELAY);
    }

    io_param.sdmmc_bus_width = (sdmmc_bus_width_t)buswidth;
    sd_set_io(&io_param, SDMMC_SET_IO_BUS_WIDTH);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_set_block_count(sd_handle_t *pHsd, uint32_t blk_cnt)
 * \brief        Send CMD23 to set block count for multi-block transfer
 * \param[in]    pHsd - Global SD Handle pointer
 * \param[in]    blk_cnt - Block count value
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_set_block_count(sd_handle_t *pHsd, uint32_t blk_cnt)
{
    pHsd->sd_cmd.cmdidx   = CMD23;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = blk_cnt;

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        return SD_DRV_STATUS_ERR;
    }

    sdhc_set_block_count(pHsd, blk_cnt);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_check_speed_mode(sd_handle_t *pHsd, uint8_t mode)
 * \brief        Check if SD card supports a specific speed mode using CMD6
 * \param[in]    Global SD Handle pointer
 * \param[in]    Speed mode to check
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_check_speed_mode(sd_handle_t *pHsd, uint8_t mode)
{
    /* CMD6 CHECK mode - query supported functions */
    pHsd->sd_cmd.cmdidx       = CMD6;
    pHsd->sd_cmd.rsp_type     = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg          = SDMMC_CMD6_CHECK_ARG;
    pHsd->sd_cmd.data_present = true;
    pHsd->sd_cmd.retries      = SDMMC_DATA_RETRIES;
    pHsd->sd_cmd.data.buffer    = (uint32_t) pHsd->sd_cmd.card_buffer;
    pHsd->sd_cmd.data.blk_size  = SDMMC_CMD6_BLK_SIZE;
    pHsd->sd_cmd.data.blk_cnt   = 1;
    pHsd->sd_cmd.data.direction = SD_DATA_DIR_READ;
    pHsd->sd_cmd.xfer_mode      = SDHC_XFER_MODE_DATA_XFER_RD_Msk |
                                  SDHC_XFER_MODE_BLK_CNT_Msk |
                                  SDHC_XFER_MODE_DMA_EN_Msk;

    /* Set up DMA for 64-byte switch status data transfer */
    sdhc_xfer_dma_setup(pHsd, &pHsd->sd_cmd.data);

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        SD_LOG_DBG("CMD6 CHECK: send_cmd failed");
        pHsd->sd_cmd.data_present = false;
        pHsd->sd_cmd.xfer_mode = 0;
        pHsd->sd_cmd.retries = SDMMC_CMD_RETRIES;
        return SD_DRV_STATUS_ERR;
    }

    pHsd->sd_cmd.data_present = false;
    pHsd->sd_cmd.xfer_mode = 0;
    pHsd->sd_cmd.retries = SDMMC_CMD_RETRIES;

    /* Wait for data transfer to complete */
    if (sdhc_check_xfer_done(pHsd, SDMMC_CMD6_TIMEOUT_US) != SDHC_STATUS_OK) {
        SD_LOG_DBG("CMD6 CHECK: xfer_done timeout");
        return SD_DRV_STATUS_ERR;
    }

    /* Check if card supports the requested speed mode */
    uint8_t *status = (uint8_t *)pHsd->sd_cmd.card_buffer;

    /* Byte 13 contains bus speed support bits. */
    switch (mode) {
    case SDMMC_SPEED_MODE_HS:
        if (!(status[13] & SDMMC_CMD6_SPEED_HS_BIT)) {  /* Bit 1 for HS */
            SD_LOG_WRN("Card does not support HS mode (status[13]=0x%x)", status[13]);
            return SD_DRV_STATUS_ERR;
        }
        SD_LOG_DBG("Card supports HS mode");
        break;
    case SDMMC_SPEED_MODE_SDR50:
        if (!(status[13] & SDMMC_CMD6_SPEED_SDR50_BIT)) {  /* Bit 2 for SDR50 */
            SD_LOG_WRN("Card does not support SDR50 mode (status[13]=0x%x)", status[13]);
            return SD_DRV_STATUS_ERR;
        }
        SD_LOG_DBG("Card supports SDR50 mode");
        break;
    case SDMMC_SPEED_MODE_SDR104:
        if (!(status[13] & SDMMC_CMD6_SPEED_SDR104_BIT)) {  /* Bit 3 for SDR104 */
            SD_LOG_WRN("Card does not support SDR104 mode (status[13]=0x%x)", status[13]);
            return SD_DRV_STATUS_ERR;
        }
        SD_LOG_DBG("Card supports SDR104 mode");
        break;
    default:
        SD_LOG_DBG("Default speed mode (DS)");
        break;
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_switch_speed_mode(sd_handle_t *pHsd, uint8_t mode)
 * \brief        Switch SD card to a specific speed mode using CMD6
 * \param[in]    Global SD Handle pointer
 * \param[in]    Speed mode to switch to
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_switch_speed_mode(sd_handle_t *pHsd, uint8_t mode)
{
    /* CMD6 to set function with data transfer */
    pHsd->sd_cmd.cmdidx       = CMD6;
    pHsd->sd_cmd.rsp_type     = SDMMC_RESP_R48;
    /* Apply access mode while leaving other function groups unchanged. */
    pHsd->sd_cmd.arg          = (SDMMC_CMD6_SWITCH_MODE_APPLY << 31) | SDMMC_CMD6_CHECK_ARG;
    pHsd->sd_cmd.arg         &= ~(0xFU << 0);
    pHsd->sd_cmd.arg         |= (mode & 0xF) << 0;
    pHsd->sd_cmd.data_present = true;
    pHsd->sd_cmd.retries      = SDMMC_DATA_RETRIES;
    pHsd->sd_cmd.data.buffer    = (uint32_t) pHsd->sd_cmd.card_buffer;
    pHsd->sd_cmd.data.blk_size  = SDMMC_CMD6_BLK_SIZE;
    pHsd->sd_cmd.data.blk_cnt   = 1;
    pHsd->sd_cmd.data.direction = SD_DATA_DIR_READ;
    pHsd->sd_cmd.xfer_mode      = SDHC_XFER_MODE_DATA_XFER_RD_Msk |
                                  SDHC_XFER_MODE_DMA_EN_Msk |
                                  SDHC_XFER_MODE_BLK_CNT_Msk;

    /* Set up DMA for 64-byte data transfer */
    sdhc_xfer_dma_setup(pHsd, &pHsd->sd_cmd.data);

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        pHsd->sd_cmd.data_present = false;
        pHsd->sd_cmd.xfer_mode = 0;
        pHsd->sd_cmd.retries = SDMMC_CMD_RETRIES;
        return SD_DRV_STATUS_ERR;
    }

    pHsd->sd_cmd.data_present = false;
    pHsd->sd_cmd.xfer_mode = 0;
    pHsd->sd_cmd.retries = SDMMC_CMD_RETRIES;

    /* Wait for data transfer to complete */
    if (sdhc_check_xfer_done(pHsd, SDMMC_CMD6_TIMEOUT_US) != SDHC_STATUS_OK) {
        return SD_DRV_STATUS_ERR;
    }

    sys_busy_loop_us(SDMMC_CMD6_DELAY);

    /* Check if card accepted the speed switch (byte 16 of response) */
    uint8_t *status = (uint8_t *)pHsd->sd_cmd.card_buffer;

    if ((status[16] & 0xF) != mode) {
        SD_LOG_WRN("Card did not accept speed mode %d (status[16]=0x%x)", mode, status[16]);
        return SD_DRV_STATUS_ERR;
    }

    SD_LOG_DBG("Speed mode %d switch: accepted", mode);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           static SD_DRV_STATUS sd_card_init(sd_handle_t *pHsd, sd_param_t *p_sd_param)
 * \brief        initialize card
 * \param[in]    sd global handle pointer
 * \return       sd driver status
 */
static SD_DRV_STATUS sd_card_init(sd_handle_t *pHsd, sd_param_t *p_sd_param)
{
    uint32_t       ocr;
    uint8_t        re_init_cnt    = 1;
    sdmmc_io_t     io_param;

    /* Check card detect using low-level API (GPIO callback or PSTATE register) */
    if (!sdhc_card_present(pHsd, p_sd_param->card_det_cb)) {
        SD_LOG_ERR("SD card not detected");
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    /* Default settings */
    pHsd->sd_card.cardtype        = SDMMC_CARD_SDHC;
    pHsd->sd_card.busspeed        = SDMMC_CLK_400_KHZ;
    ocr                           = (SDMMC_CMD41_HCS |
                                     SDMMC_CMD41_3V3);
#if SOC_FEAT_SDMMC_SUPPORT_1V8
    ocr                          |= SDMMC_OCR_S18R;
#endif

    /* Set 400KHz initialization clock */
    io_param.sdmmc_clock = SDMMC_CLK_400KHZ;
    sd_set_io(&io_param, SDMMC_SET_IO_CLK);

    pHsd->state = SD_CARD_STATE_IDLE;

    sys_busy_loop_us(100);

    /* Reset the command structure */
    pHsd->sd_cmd.cmdidx    = 0;
    pHsd->sd_cmd.rsp_type  = SDMMC_RESP_NONE;
    pHsd->sd_cmd.arg       = 0;
    pHsd->sd_cmd.xfer_mode = 0;
    pHsd->sd_cmd.retries   = SDMMC_CMD_RETRIES;

    /* Check and wait till the card is present and Reset It */
    if (sd_identify_card(pHsd) != SD_DRV_STATUS_OK) {
        /* Card Not present */
        return SD_DRV_STATUS_TIMEOUT_ERR;
    }

RE_INIT:

    /* Reset the SD/UHS Cards */
    if (sd_go_idle(pHsd) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    sys_busy_loop_us(100);

    /* Get the card interface condition */
    if (sd_get_card_ifcond(pHsd) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    /* Get the card operating condition */
    if (sd_get_card_opcond(pHsd, ocr) != SD_DRV_STATUS_OK) {
        if (sd_get_emmc_card_opcond(pHsd) != SD_DRV_STATUS_OK) {
            return SD_DRV_STATUS_CARD_INIT_ERR;  // No Valid Card Found
        } else {
            pHsd->sd_card.cardtype = SDMMC_CARD_MMC;
        }
    }

    if (pHsd->sd_card.flags & SDMMC_1P8V_FLAG) {

        if (sd_switch_voltage(pHsd, SDMMC_VOL_1P8V)) {

            if (re_init_cnt--) {

                if (pHsd->sd_param.pwr_cb) {
                    pHsd->sd_param.pwr_cb(0);
                }

                /* 1.8V switch Failed Re-init with 3.3V */
                ocr                 &= ~SDMMC_OCR_S18R;
                pHsd->sd_card.flags &= ~SDMMC_1P8V_FLAG;

                /* Set 3.3v SD Voltage supply */
                io_param.sdmmc_vol = SDMMC_VOL_3P3V;
                sd_set_io(&io_param, SDMMC_SET_IO_VOL);

                sys_busy_loop_us(SDMMC_RESET_DELAY_US);

                sdhc_reset(pHsd, SDHC_SW_RST_CMD_Msk | SDHC_SW_RST_DAT_Msk);
                goto RE_INIT;

            } else {
                return SD_DRV_STATUS_CARD_INIT_ERR;
            }
        }
    }

    if (!(pHsd->sd_card.sdio_mode)) {
        /* Get the card ID CMD2 */
        if (sd_get_card_cid(pHsd) != SD_DRV_STATUS_OK) {
            return SD_DRV_STATUS_CARD_INIT_ERR;
        }
    }

    /* Get the card Relative Address CMD3 */
    if (sd_get_rca(pHsd, &pHsd->sd_card.relcardadd) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    if (!(pHsd->sd_card.sdio_mode)) {
        /* Get the CSD register */
        if (sd_get_card_csd(pHsd) != SD_DRV_STATUS_OK) {
            return SD_DRV_STATUS_CARD_INIT_ERR;
        }

        sdmmc_decode_card_csd(pHsd);

        if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {
            /* Get the SCR register */
            if (sd_get_card_scr(pHsd) != SD_DRV_STATUS_OK) {
                return SD_DRV_STATUS_CARD_INIT_ERR;
            }

			/* Select a card */
			if (sd_sel_card(pHsd, pHsd->sd_card.relcardadd) != SD_DRV_STATUS_OK) {
				return SD_DRV_STATUS_CARD_INIT_ERR;
			}
        }

        /* Change the Card State from Identification to Ready */
        pHsd->state = SD_CARD_STATE_STBY;
    }

    /* Determine required speed mode based on configured frequency */
    uint8_t target_speed_mode = SDMMC_SPEED_MODE_DEFAULT;

    if (p_sd_param->clock_freq > SDMMC_CLK_100_MHZ_HZ) {
        target_speed_mode = SDMMC_SPEED_MODE_SDR104;
    } else if (p_sd_param->clock_freq > SDMMC_CLK_50_MHZ_HZ) {
        target_speed_mode = SDMMC_SPEED_MODE_SDR50;
    } else if (p_sd_param->clock_freq == SDMMC_CLK_50_MHZ_HZ) {
        target_speed_mode = SDMMC_SPEED_MODE_HS;
    }

    SD_LOG_DBG("Target speed mode: %d (clock: %d Hz)",
        target_speed_mode, p_sd_param->clock_freq);

    if (target_speed_mode != SDMMC_SPEED_MODE_DEFAULT) {
        if (sd_check_speed_mode(pHsd, target_speed_mode) != SD_DRV_STATUS_OK) {
            SD_LOG_WRN("CMD6 CHECK failed, resetting DAT/CMD lines");
            sdhc_reset(pHsd, (uint8_t)(SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk));
        }

        if (sd_switch_speed_mode(pHsd, target_speed_mode) != SD_DRV_STATUS_OK) {
            SD_LOG_WRN("Speed mode %d switch failed, resetting DAT/CMD lines",
                          target_speed_mode);
            sdhc_reset(pHsd, (uint8_t)(SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk));
            /* Use default speed after failed switch */
            io_param.sdmmc_clock = SDMMC_CLK_25MHZ;
            sd_set_io(&io_param, SDMMC_SET_IO_CLK);
        } else {
            SD_LOG_INF("Speed mode %d switch: success", target_speed_mode);
            /* Set clock frequency after successful speed mode switch */
            io_param.sdmmc_clock = (p_sd_param->clock_freq > SDMMC_CLK_25_MHZ) ?
                                  SDMMC_CLK_ENABLE : SDMMC_CLK_25MHZ;
            sd_set_io(&io_param, SDMMC_SET_IO_CLK);
        }
    } else {
        /* Default speed mode: set clock frequency */
        io_param.sdmmc_clock = (p_sd_param->clock_freq > SDMMC_CLK_25_MHZ) ?
                              SDMMC_CLK_ENABLE : SDMMC_CLK_25MHZ;
        sd_set_io(&io_param, SDMMC_SET_IO_CLK);
    }

    if (!(pHsd->sd_card.sdio_mode)) {
        if (pHsd->sd_param.bus_width >= SDMMC_4_BIT_MODE) {
            SD_LOG_DBG("Setting bus width to %d",
                        (pHsd->sd_param.bus_width == SDMMC_8_BIT_MODE) ? 8 :
                        (pHsd->sd_param.bus_width == SDMMC_4_BIT_MODE) ? 4 : 1);
            if (sd_set_bus_width(pHsd, pHsd->sd_param.bus_width) != SD_DRV_STATUS_OK) {
                SD_LOG_ERR("Failed to set bus width to %d",
                            (pHsd->sd_param.bus_width == SDMMC_8_BIT_MODE) ? 8 :
                            (pHsd->sd_param.bus_width == SDMMC_4_BIT_MODE) ? 4 : 1);
                return SD_DRV_STATUS_CARD_INIT_ERR;
            }
        }

        if (sd_set_blk_size(pHsd, SDMMC_BLK_SIZE_512_Msk) != SD_DRV_STATUS_OK) {
            SD_LOG_ERR("Failed to set block size to 512 bytes");
            return SD_DRV_STATUS_CARD_INIT_ERR;
        }
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sd_status(void)
 * \brief        Check SD card presence and status
 * \return       sd driver status (OK if card present and ready, ERR if not)
 */
SD_DRV_STATUS sd_status(void)
{
    uint32_t card_status;
    sd_handle_t *pHsd = &Hsd;

#ifdef BOARD_SD_CARD_DETECT_GPIO_PORT
    uint32_t pin_state;
    ARM_DRIVER_GPIO *cd_gpio = &ARM_Driver_GPIO_(BOARD_SD_CARD_DETECT_GPIO_PORT);

    cd_gpio->GetValue(BOARD_SD_CARD_DETECT_GPIO_PIN, &pin_state);

    SD_LOG_DBG("SD card detect pin state: %d", pin_state);

    /* Card detect: pin state 0 = card present (active low) */
    if (pin_state != 0) {
        return SD_DRV_STATUS_ERR;
    }
#endif

    /* Check card state via CMD13 */
    if (sd_get_card_status(pHsd, &card_status) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_ERR;
    }

    SD_LOG_DBG("Card Status: %" PRIx32, card_status);

    /* Card is ready if in TRAN or STBY state */
    if ((card_status & 0x1F) == SD_CARD_STATE_TRAN ||
        (card_status & 0x1F) == SD_CARD_STATE_STBY) {
        return SD_DRV_STATUS_OK;
    }

    return SD_DRV_STATUS_ERR;
}

/**
 * \fn           SD_DRV_STATUS sd_init(sd_param_t *p_sd_param)
 * \brief        main SD initialize function
 * \param[in]    device ID
 * \return       sd driver status
 */
SD_DRV_STATUS sd_init(sd_param_t *p_sd_param)
{

    SD_DRV_STATUS errcode = SD_DRV_STATUS_OK;
    sd_handle_t *pHsd = &Hsd;

    /* Initialize Host controller */
    errcode = sdhc_init(pHsd, p_sd_param);

    if (errcode != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_HOST_INIT_ERR;
    }

    /* Initialize SD Memory/Combo Card */
    errcode = sd_card_init(pHsd, p_sd_param);

    if (errcode != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_CARD_INIT_ERR;
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           void sdmmc_decode_card_ext_csd(sd_handle_t *pHsd, uint8_t *praw_ext_csd)
 * \brief        decode emmc ext csd data
 * \param[in]    sd handle pointer
 * \param[in]    raw ext csd data pointer
 * \return       none
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
    pHsd->mmc_ext_csd.cache_size = (praw_ext_csd[252] << 24U) +
                                  (praw_ext_csd[251] << 16U) +
                                  (praw_ext_csd[250] << 8U) +
                                  (praw_ext_csd[249] << 0U);

    pHsd->sd_card.sectorcount  = pHsd->mmc_ext_csd.sector_cnt;
    pHsd->sd_card.sectorsize   = SDMMC_BLK_SIZE_512_Msk;
    pHsd->sd_card.logblocksize = SDMMC_BLK_SIZE_512_Msk;

    return;
}

/**
 * \fn           void sdmmc_decode_card_csd(sd_handle_t *pHsd)
 * \brief        decode card csd data
 * \param[in]    sd handle pointer
 * \return       none
 */
void sdmmc_decode_card_csd(sd_handle_t *pHsd)
{
    uint32_t blk_len;
    uint32_t device_size;
    uint32_t mult, c_size;

    if (((pHsd->sd_card.csd[RAW_CSD_BUF_IDX3] & CSD_STRUCT_Msk) >> CSD_STRUCT_Pos) == 0U) {
        blk_len = (uint32_t) 1U <<
                 ((uint32_t) (pHsd->sd_card.csd[RAW_CSD_BUF_IDX2] & READ_BLK_LEN_Msk) >>
                  CSD_V1_BLK_LEN_Pos);
        mult = (uint32_t) 1U <<
               ((uint32_t) ((pHsd->sd_card.csd[RAW_CSD_BUF_IDX1] & C_SIZE_MULT_Msk) >>
                7U) + (uint32_t) 2U);
        device_size = (pHsd->sd_card.csd[RAW_CSD_BUF_IDX1] & C_SIZE_LOWER_Msk) >>
                     C_SIZE_LOWER_Pos;
        device_size |= (pHsd->sd_card.csd[RAW_CSD_BUF_IDX2] & C_SIZE_UPPER_Msk) <<
                      C_SIZE_UPPER_Pos;
        device_size  = (device_size + 1U) * mult;
        device_size  = device_size * blk_len;
        pHsd->sd_card.sectorcount  = (device_size / SDMMC_BLK_SIZE_512_Msk);
        pHsd->sd_card.sectorsize   = blk_len;
        pHsd->sd_card.logblocksize = blk_len;
    } else if (((pHsd->sd_card.csd[RAW_CSD_BUF_IDX3] & CSD_STRUCT_Msk) >> CSD_STRUCT_Pos) == 1U) {
        c_size = ((pHsd->sd_card.csd[RAW_CSD_BUF_IDX1] & CSD_V2_C_SIZE_Msk) >> CSD_V2_C_SIZE_Pos);

        if (c_size >= SDMMC_SDHC_MAX_SECTOR_CNT) {
            pHsd->sd_card.cardtype = SDMMC_CARD_SDXC;
        }

        pHsd->sd_card.sectorcount = (c_size + 1U) * 1024U;
        pHsd->sd_card.card_class =
            (uint16_t) ((pHsd->sd_card.csd[RAW_CSD_BUF_IDX2] & CSD_CCC_Msk) >> CSD_CCC_SHIFT);
        pHsd->sd_card.sectorsize   = SDMMC_BLK_SIZE_512_Msk;
        pHsd->sd_card.logblocksize = SDMMC_BLK_SIZE_512_Msk;
    } else if (((pHsd->sd_card.csd[RAW_CSD_BUF_IDX3] & CSD_STRUCT_Msk) >> CSD_STRUCT_Pos) == 3U) {
        /* get ext csd data for eMMC */
        if (sd_get_card_ext_csd(pHsd, &pHsd->sd_cmd.card_buffer[0]) == SD_DRV_STATUS_OK) {
            if (sdhc_check_xfer_done(pHsd, SDMMC_DATA_TIMEOUT) == SDHC_STATUS_OK) {
                sdmmc_decode_card_ext_csd(pHsd, &pHsd->sd_cmd.card_buffer[0]);
            }
        }
    }
    return;
}

/**
 * \fn           SD_DRV_STATUS sd_uninit(uint8_t devId)
 * \brief        SD uninitialize function
 * \param[in]    device ID
 * \return       sd driver status
 */
SD_DRV_STATUS sd_uninit(uint8_t devId)
{
    ARG_UNUSED(devId);

    /* release the card */
    sd_handle_t *pHsd         = &Hsd;
    pHsd->sd_cmd.cmdidx       = CMD7;
    pHsd->sd_cmd.rsp_type     = SDMMC_RESP_R48;
    pHsd->sd_cmd.data_present = false;
    pHsd->sd_cmd.arg          = SDMMC_DESELECT_RCA;  // any other RCA to perform card de-selection

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        sd_error_handler();
    }

    /* turn off power supply to the card */
    sdmmc_io_t io_param;
    io_param.sdmmc_power = SDMMC_POWER_OFF;
    sd_set_io(&io_param, SDMMC_SET_IO_PWR);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_CARD_STATE sd_state(void)
 * \brief        sd state
 * \param[in]    void
 * \return       sd state
 */
SD_CARD_STATE sd_state(void)
{
    sd_handle_t *pHsd = &Hsd;
    uint32_t     status;

    sd_get_card_status(pHsd, &status);

    return status;
}

/**
 * \fn           SD_DRV_STATUS sd_info(sd_cardinfo_t *pinfo)
 * \brief        returns sd card info
 * \param[in]    sd_cardinfo_t * pinfo
 * \return       sd driver status
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
 * \fn           static SD_DRV_STATUS sd_xfer_setup(sd_handle_t *pHsd, sd_data_t *data,
 *                                                 uint8_t direction)
 * \brief        Setup transfer parameter and start transfer
 * \param[in]    Global sd Handle pointer
 * \param[in]    Data phase descriptor
 * \param[in]    Transfer direction (SD_DATA_DIR_READ or SD_DATA_DIR_WRITE)
 * \return       SD driver status
 */
static SD_DRV_STATUS sd_xfer_setup(sd_handle_t *pHsd, sd_data_t *data, uint8_t direction)
{
    /* Send CMD23 for multi-block transfers (SDHC/SDXC only).
     * Per SD spec 4.3.4.4, CMD23 is only valid immediately before
     * CMD18/CMD25. Sending it before single-block CMD17/CMD24 is an
     * illegal sequence: the card sets ILLEGAL_COMMAND in the next
     * response and the CMD24 write data phase never starts. */
    if ((pHsd->sd_card.cardtype != SDMMC_CARD_SDSC) && (data->blk_cnt > 1)) {
        if (sd_set_block_count(pHsd, data->blk_cnt) != SD_DRV_STATUS_OK) {
            return SD_DRV_STATUS_ERR;
        }
    }

    sdhc_xfer_dma_setup(pHsd, data);

    pHsd->sd_cmd.arg = data->sector;
    if (pHsd->sd_card.cardtype == SDMMC_CARD_SDSC) {
        pHsd->sd_cmd.arg <<= 9;
    }

    pHsd->sd_cmd.data_present = true;
    pHsd->sd_cmd.retries      = SDMMC_DATA_RETRIES;

    if (direction == SD_DATA_DIR_READ) {
        if (data->blk_cnt == 1) {
            pHsd->sd_cmd.cmdidx    = CMD17;
            pHsd->sd_cmd.rsp_type  = SDMMC_RESP_R48;
            pHsd->sd_cmd.xfer_mode = SDHC_XFER_MODE_DATA_XFER_RD_Msk |
                                     SDHC_XFER_MODE_BLK_CNT_Msk |
                                     SDHC_XFER_MODE_DMA_EN_Msk;
        } else {
            pHsd->sd_cmd.cmdidx    = CMD18;
            pHsd->sd_cmd.rsp_type  = SDMMC_RESP_R48;
            pHsd->sd_cmd.xfer_mode = SDHC_XFER_MODE_DATA_XFER_RD_Msk |
                                     SDHC_XFER_MODE_MULTI_BLK_SEL_Msk |
                                     SDHC_XFER_MODE_BLK_CNT_Msk |
                                     SDHC_XFER_MODE_DMA_EN_Msk;
        }
    } else {
        if (data->blk_cnt == 1) {
            pHsd->sd_cmd.cmdidx    = CMD24;
            pHsd->sd_cmd.rsp_type  = SDMMC_RESP_R48;
            pHsd->sd_cmd.xfer_mode = SDHC_XFER_MODE_DATA_XFER_WR_Msk |
                                     SDHC_XFER_MODE_BLK_CNT_Msk |
                                     SDHC_XFER_MODE_DMA_EN_Msk;
        } else {
            pHsd->sd_cmd.cmdidx    = CMD25;
            pHsd->sd_cmd.rsp_type  = SDMMC_RESP_R48;
            pHsd->sd_cmd.xfer_mode = SDHC_XFER_MODE_DATA_XFER_WR_Msk |
                                     SDHC_XFER_MODE_MULTI_BLK_SEL_Msk |
                                     SDHC_XFER_MODE_BLK_CNT_Msk |
                                     SDHC_XFER_MODE_DMA_EN_Msk;
        }
    }

    sdhc_set_led(pHsd, true);

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        pHsd->sd_cmd.data_present = false;
        pHsd->sd_cmd.xfer_mode = 0;
        pHsd->sd_cmd.retries = SDMMC_CMD_RETRIES;
        sdhc_set_led(pHsd, false);
        return SD_DRV_STATUS_ERR;
    }

    pHsd->sd_cmd.data_present = false;
    pHsd->sd_cmd.xfer_mode = 0;
    pHsd->sd_cmd.retries = SDMMC_CMD_RETRIES;
    sdhc_set_led(pHsd, false);

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sd_read(uint32_t sec, uint16_t blk_cnt,
 *                                   volatile unsigned char *dest_buff)
 * \brief        read sd sector
 * \param[in]    sec - input sector number to read
 * \param[in]    blk_cnt - number of block to read
 * \param[in]    dest_buff - Destination buffer pointer
 * \return       sd driver status
 */
SD_DRV_STATUS sd_read(uint32_t sec, uint16_t blk_cnt, volatile unsigned char *dest_buff)
{
    sd_handle_t *pHsd        = &Hsd;
    uint32_t     timeout_cnt = SDMMC_DATA_TIMEOUT * blk_cnt;
    uint8_t      retry_cnt   = 1;

    SD_LOG_DBG("SD READ Dest Buff: 0x%08" PRIxPTR " Sec: %" PRId32
                  ", Block Count: %" PRId32,
                  (uintptr_t) dest_buff,
                  sec,
                  blk_cnt);

    if (dest_buff == NULL) {
        return SD_DRV_STATUS_RD_ERR;
    }

    pHsd->sd_cmd.data.buffer    = (uint32_t) dest_buff;
    pHsd->sd_cmd.data.blk_size  = SDMMC_BLK_SIZE_512_Msk;
    pHsd->sd_cmd.data.blk_cnt   = blk_cnt;
    pHsd->sd_cmd.data.sector    = sec;
    pHsd->sd_cmd.data.direction = SD_DATA_DIR_READ;

#ifdef SDMMC_IRQ_MODE

    (void) timeout_cnt;
    (void) retry_cnt;
    if (sd_xfer_setup(pHsd, &pHsd->sd_cmd.data, SD_DATA_DIR_READ) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_RD_ERR;
    }

#else

retry:
    if (sd_xfer_setup(pHsd, &pHsd->sd_cmd.data, SD_DATA_DIR_READ) != SD_DRV_STATUS_OK) {
        SD_LOG_DBG("SD read setup failed");
        return SD_DRV_STATUS_RD_ERR;
    }

    if (sdhc_check_xfer_done(pHsd, timeout_cnt) == SDHC_STATUS_OK) {
        RTSS_InvalidateDCache_by_Addr(dest_buff, blk_cnt * SDMMC_BLK_SIZE_512_Msk);
    } else {
        /* Soft reset Host controller cmd and data lines */
        sdhc_reset(pHsd, (uint8_t) (SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk));

        if (!retry_cnt--) {
            return SD_DRV_STATUS_RD_ERR;
        }
        goto retry;
    }
#endif

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sd_write(uint32_t sector, uint32_t blk_cnt,
 *                                   volatile unsigned char *src_buff)
 * \brief        Write sd sector
 * \param[in]    sector - input sector number to write
 * \param[in]    blk_cnt - number of block to write
 * \param[in]    src_buff - Source buffer pointer
 * \return       sd driver status
 */
SD_DRV_STATUS sd_write(uint32_t sector, uint32_t blk_cnt,
                        volatile unsigned char *src_buff)
{

    sd_handle_t *pHsd = &Hsd;

#ifndef SDMMC_IRQ_MODE
    int     timeout_cnt = 1000 * blk_cnt;
    uint8_t retry_cnt   = 1;
#endif
    uint8_t *aligned_buff = (uint8_t *) src_buff;

    if (src_buff == NULL) {
        return SD_DRV_STATUS_WR_ERR;
    }

    SD_LOG_DBG("SD WRITE Src Buff: 0x%08" PRIxPTR " Sec: %" PRId32
                  ", Block Count: %" PRId32,
                  (uintptr_t) src_buff,
                  sector,
                  blk_cnt);

    if ((uint32_t) src_buff & (SDMMC_BLK_SIZE_512_Msk - 1)) {
        SD_LOG_WRN("SD Write ADMA access un-aligned buffer ...");
        if (blk_cnt <= SDMMC_CACHED_NUM_BLK) {
            memcpy(&pHsd->sd_cmd.card_buffer[0], (const void *) src_buff, blk_cnt * 512);
            aligned_buff = &pHsd->sd_cmd.card_buffer[0];
        } else {
            /* Return error if the unaligned write exceeds the internal buffer. */
            return SD_DRV_STATUS_WR_ERR;
        }
    }

    /* Clean the DCache */
    RTSS_CleanDCache_by_Addr(aligned_buff, blk_cnt * SDMMC_BLK_SIZE_512_Msk);

    pHsd->sd_cmd.data.buffer    = (uint32_t) aligned_buff;
    pHsd->sd_cmd.data.blk_size  = SDMMC_BLK_SIZE_512_Msk;
    pHsd->sd_cmd.data.blk_cnt   = (uint16_t) blk_cnt;
    pHsd->sd_cmd.data.sector    = sector;
    pHsd->sd_cmd.data.direction = SD_DATA_DIR_WRITE;

#ifndef SDMMC_IRQ_MODE
retry:
#endif

    if (sd_xfer_setup(pHsd, &pHsd->sd_cmd.data, SD_DATA_DIR_WRITE) != SD_DRV_STATUS_OK) {
        SD_LOG_ERR("SD write setup failed");
        return SD_DRV_STATUS_WR_ERR;
    }

#ifndef SDMMC_IRQ_MODE
    if (sdhc_check_xfer_done(pHsd, timeout_cnt) != SDHC_STATUS_OK) {
        SD_LOG_ERR("SD WRITE: xfer done timeout");
        sdhc_reset(pHsd, (uint8_t) (SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk));

        if (!retry_cnt--) {
            return SD_DRV_STATUS_WR_ERR;
        }
        goto retry;
    }
#endif

    return SD_DRV_STATUS_OK;
}
