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
 * @file     sdio.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    SDIO Driver APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "sd_host.h"
#include "sd.h"
#include "sdio.h"

extern sd_handle_t Hsd;

/**
 * \fn           SD_DRV_STATUS sdio_reset(sd_handle_t *pHsd)
 * \brief        Reset the IO/Combo Card
 * \param[in]    Global sd Handle pointer
 * \return       Host controller driver status
 */
SD_DRV_STATUS sdio_reset(sd_handle_t *pHsd)
{
    pHsd->sd_cmd.cmdidx   = SDIO_RW_DIRECT;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R1;
    pHsd->sd_cmd.arg      = 0;
    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        return SD_DRV_STATUS_ERR;
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sdio_get_opcond(sd_handle_t *pHsd, uint32_t ocr, \
 *                                      uint32_t *rocr)
 * \brief        Get IO card operating condition (CMD5)
 * \param[in]    Global sd Handle pointer
 * \param[in]    ocr value
 * \param[in]    ocr response destination pointer
 * \return       Host controller driver status
 */
SD_DRV_STATUS sdio_get_opcond(sd_handle_t *pHsd, uint32_t ocr, uint32_t *rocr)
{

    pHsd->sd_cmd.cmdidx   = SDIO_SEND_OP_COND;
    pHsd->sd_cmd.rsp_type = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg      = ocr;

    if (sdhc_send_cmd(pHsd, &pHsd->sd_cmd) != SDHC_STATUS_OK) {
        return SD_DRV_STATUS_ERR;
    }

    *rocr = pHsd->regs->SDMMC_RESP01_R;
    if (*rocr == SDMMC_CMD8_VOL_PATTERN) {
        return SD_DRV_STATUS_ERR;
    }

    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sdio_rw_direct(sd_handle_t *pHsd, ...)
 * \brief        SDIO CMD52 Read/Write Direct
 * \param[in]    Global sd Handle pointer
 * \param[in]    rwFlag - read/write flag
 * \param[in]    fn - function number
 * \param[in]    addr - register address
 * \param[in]    writeData - data to write
 * \param[in]    readPtr - read data pointer
 * \return       Host controller driver status
 */
SD_DRV_STATUS sdio_rw_direct(sd_handle_t *pHsd, uint32_t rwFlag, uint32_t fn, uint32_t addr,
                                uint8_t writeData, uint8_t *readPtr)
{
    SD_DRV_STATUS status = SD_DRV_STATUS_OK;
    sd_cmd_t        sdio_cmd;
    uint32_t        resp;

    if ((fn > SDIO_MAX_FUNCTION) || (addr & ~(SDIO_MAX_CIA_ADDR))) {
        return SD_DRV_STATUS_ERR;
    }

    sdio_cmd.xfer_mode  = SDHC_XFER_MODE_DATA_XFER_RD_Msk | SDHC_XFER_MODE_DMA_EN_Msk;
    sdio_cmd.cmdidx     = SDIO_RW_DIRECT;
    sdio_cmd.rsp_type   = SDMMC_RESP_R1;
    sdio_cmd.retries    = 0;
    sdio_cmd.arg        = rwFlag ? SDIO_RW_FLAG_Msk : 0x00000000;
    sdio_cmd.arg       |= fn << SDIO_FN_Pos;
    sdio_cmd.arg       |= (rwFlag && readPtr) ? SDIO_RAW_FLAG_Msk : 0x00000000;
    sdio_cmd.arg       |= addr << SDIO_REG_ADDR_Pos;
    sdio_cmd.arg       |= rwFlag ? writeData : 0x00000000;

    if (sdhc_send_cmd(pHsd, &sdio_cmd) != SDHC_STATUS_OK) {
        return SD_DRV_STATUS_ERR;
    }

    resp = pHsd->regs->SDMMC_RESP01_R;

    if ((resp & SDIO_R5_ERROR) || (resp & SDIO_R5_FUNCTION_NUMBER) ||
        (resp & SDIO_R5_OUT_OF_RANGE)) {
        return SD_DRV_STATUS_ERR;
    }

    if (readPtr) {
        *readPtr = resp & 0xFF;
    }

    return status;
}

/**
 * \fn           SD_DRV_STATUS sdio_rw_extended(sd_handle_t *pHsd, ...)
 * \brief        SDIO CMD53 Read/Write Extended
 * \param[in]    Global sd Handle pointer
 * \param[in]    rwFlag - read/write flag
 * \param[in]    fn - function number
 * \param[in]    addr - register address
 * \param[in]    incr_addr - increment address flag
 * \param[in]    buf - data buffer
 * \param[in]    blk_cnt - block count
 * \param[in]    blkSize - block size
 * \return       Host controller driver status
 */
SD_DRV_STATUS sdio_rw_extended(sd_handle_t *pHsd, uint32_t rwFlag, uint32_t fn,
                              uint32_t addr, uint32_t incr_addr, uint8_t *buf,
                              uint32_t blk_cnt, uint32_t blkSize)
{

    SDHC_STATUS status      = SDHC_STATUS_OK;
    uint32_t    timeout_cnt = SDMMC_DATA_TIMEOUT * (blk_cnt ? blk_cnt : 1);
    uint8_t     retry_cnt   = 1;

    if ((fn > SDIO_MAX_FUNCTION) || (addr & ~(SDIO_MAX_CIA_ADDR))) {
        status = SDHC_STATUS_ERR;
        goto exit;
    }

    /* Prepare data parameters in global structure */
    pHsd->sd_cmd.data.buffer    = (uint32_t) buf;
    pHsd->sd_cmd.data.blk_size  = (uint16_t) blkSize;
    pHsd->sd_cmd.data.blk_cnt   = (uint16_t) (blk_cnt ? blk_cnt : 1);
    pHsd->sd_cmd.data.direction = rwFlag ? SD_DATA_DIR_WRITE : SD_DATA_DIR_READ;

    /* Prepare command parameters */
    pHsd->sd_cmd.cmdidx       = SDIO_RW_EXTENDED;
    pHsd->sd_cmd.rsp_type     = SDMMC_RESP_R48;
    pHsd->sd_cmd.arg          = rwFlag ? SDIO_RW_FLAG_Msk : 0x00000000;
    pHsd->sd_cmd.arg         |= fn << SDIO_FN_Pos;
    pHsd->sd_cmd.arg         |= incr_addr ? SDIO_RW_EXT_INCR_ADDR_Msk : 0x00000000;
    pHsd->sd_cmd.arg         |= addr << SDIO_REG_ADDR_Pos;
    pHsd->sd_cmd.data_present = 1;
    pHsd->sd_cmd.retries      = 0;

    if (blk_cnt) {
        pHsd->sd_cmd.arg |= SDIO_RW_EXT_BLK_MODE_Msk | blk_cnt;
    } else {
        pHsd->sd_cmd.arg |= (blkSize == 512) ? 0 : blkSize;
    }

    /* Change the Card State from Tran to Data */
    pHsd->state = SD_CARD_STATE_DATA;

retry:
    /* Configure DMA */
    status = sdhc_xfer_dma_setup(pHsd, &pHsd->sd_cmd.data);
    if (status != SDHC_STATUS_OK) {
        goto exit;
    }

    /* Set up transfer mode */
    pHsd->sd_cmd.xfer_mode =
        (rwFlag ? SDHC_XFER_MODE_DATA_XFER_WR_Msk : SDHC_XFER_MODE_DATA_XFER_RD_Msk) |
        SDHC_XFER_MODE_DMA_EN_Msk;
    if (blk_cnt) {
        pHsd->sd_cmd.xfer_mode |= SDHC_XFER_MODE_MULTI_BLK_SEL_Msk;
    }

    pHsd->sd_cmd.data_present = 1;

    /* Send command */
    status = sdhc_send_cmd(pHsd, &pHsd->sd_cmd);
    if (status != SDHC_STATUS_OK) {
        goto exit;
    }

    pHsd->sd_cmd.data_present = 0;
    pHsd->sd_cmd.xfer_mode = 0;

    /* Wait for transfer completion */
    if (sdhc_check_xfer_done(pHsd, timeout_cnt) != SDHC_STATUS_OK) {
        /* Soft reset Host controller cmd and data lines */
        sdhc_reset(pHsd, (uint8_t) (SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk));

        if (!retry_cnt--) {
            status = SDHC_STATUS_ERR;
            goto exit;
        }
        goto retry;
    }

    /* Invalidate DCache for read operations */
    if (!rwFlag) {
        RTSS_InvalidateDCache_by_Addr(buf, pHsd->sd_cmd.data.blk_cnt *
                                       pHsd->sd_cmd.data.blk_size);
    }

    /* Change the Card State from Data to Tran */
    pHsd->state = SD_CARD_STATE_TRAN;

exit:
    return (status == SDHC_STATUS_OK) ? SD_DRV_STATUS_OK : SD_DRV_STATUS_ERR;
}

/**
 * \fn           SD_DRV_STATUS sdio_read_cia(uint8_t *pcia, uint8_t fn, uint8_t offset)
 * \brief        read SDIO Common I/O Area with given offset
 * \param[in]    pointer to Common I/O Area
 * \param[in]    Function number(0-7)
 * \param[in]    Function offset(0x00 - 0xFF)
 * \return       SD driver status
 */
SD_DRV_STATUS sdio_read_cia(uint8_t *pcia, uint8_t fn, uint8_t offset)
{
    sd_handle_t *pHsd = &Hsd;
    if (sdio_rw_direct(pHsd, 0, fn, offset, 0, pcia) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_RD_ERR;
    }
    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sdio_write_cia(uint8_t cia, uint8_t fn, uint8_t offset)
 * \brief        write SDIO Common I/O Area with given offset
 * \param[in]    pointer to Common I/O Area
 * \param[in]    Function number(0-7)
 * \param[in]    Function offset(0x00 - 0xFF)
 * \return       SD driver status
 */
SD_DRV_STATUS sdio_write_cia(uint8_t cia, uint8_t fn, uint8_t offset)
{
    sd_handle_t *pHsd = &Hsd;
    if (sdio_rw_direct(pHsd, 1, fn, offset, cia, 0) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_RD_ERR;
    }
    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sdio_read_cccr(uint8_t *pcccr)
 * \brief        read SDIO Common I/O Area with given offset
 * \param[in]    pointer to Common I/O Area
 * \param[in]    Function number(0-7)
 * \param[in]    Function offset(0x00 - 0xFF)
 * \return       SD driver status
 */
SD_DRV_STATUS sdio_read_cccr(uint8_t *pcccr)
{
    sd_handle_t *pHsd = &Hsd;
    if (sdio_rw_direct(pHsd, 0, 0, 0, 0, pcccr) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_RD_ERR;
    }
    return SD_DRV_STATUS_OK;
}

/**
 * \fn           SD_DRV_STATUS sdio_write_cccr(uint8_t cccr)
 * \brief        write SDIO Card control register
 * \param[in]    cccr to be written
 * \return       SD driver status
 */
SD_DRV_STATUS sdio_write_cccr(uint8_t cccr)
{
    sd_handle_t *pHsd = &Hsd;
    if (sdio_rw_direct(pHsd, 1, 0, 0, cccr, 0) != SD_DRV_STATUS_OK) {
        return SD_DRV_STATUS_RD_ERR;
    }
    return SD_DRV_STATUS_OK;
}
