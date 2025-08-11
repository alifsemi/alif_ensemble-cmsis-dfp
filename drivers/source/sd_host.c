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
 * @file     sd_host.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    SD Host Controller Driver APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "sd_host.h"
#include "sd.h"
#include "sys_utils.h"

#if defined(SDMMC_PRINTF_DEBUG) || defined(SDMMC_PRINT_WARN) || defined(SDMMC_PRINT_ERR) || \
    defined(SDMMC_PRINTF_SD_STATE_DEBUG) || defined(SDMMC_PRINT_SEC_DATA)
#include <stdio.h>
#include <inttypes.h>
#endif

extern sd_handle_t  Hsd;
static adma2_desc_t adma_desc_tbl[32] __attribute__((section("sd_dma_buf")))
__attribute__((aligned(512)));

static volatile uint16_t nis, eis, cc;

#ifdef SDMMC_IRQ_MODE
void SDMMC_IRQHandler(void)
{

    uint16_t xfer_done = 0;

    /* get the current interrupt status */
    eis                = Hsd.regs->SDMMC_ERROR_INT_STAT_R;
    nis                = Hsd.regs->SDMMC_NORMAL_INT_STAT_R;

    /* clear the current interrupt status */
    while (Hsd.regs->SDMMC_ERROR_INT_STAT_R) {
        uint16_t eis_tmp                 = Hsd.regs->SDMMC_ERROR_INT_STAT_R;
        Hsd.regs->SDMMC_ERROR_INT_STAT_R = eis_tmp;
    }

    while (Hsd.regs->SDMMC_NORMAL_INT_STAT_R) {
        uint16_t nis_tmp                  = Hsd.regs->SDMMC_NORMAL_INT_STAT_R;
        Hsd.regs->SDMMC_NORMAL_INT_STAT_R = nis_tmp;
    }

    if (eis) {
        hc_reset(&Hsd, (uint8_t) (SDMMC_SW_RST_DAT_Msk | SDMMC_SW_RST_CMD_Msk));
    }

    if (nis & SDMMC_INTR_CC_Msk) {
        cc = SDMMC_INTR_CC_Msk;
    }

    if (nis & SDMMC_INTR_TC_Msk) {
        xfer_done = SDMMC_INTR_TC_Msk;
    }

    if (Hsd.sd_param.app_callback) {
        if (cc || xfer_done) {
            Hsd.sd_param.app_callback(cc, xfer_done);
        }
    }
}

void SDMMC_WAKEUP_IRQHandler(void)
{
}

#endif

/**
  \fn          static uint8_t get_cmd_rsp_type(uint8_t Cmd)
  \brief       get expected response of given command.
  \param[in]   cmd - input cmd index
  \return      expected response type.
  */
static uint8_t hc_get_cmd_rsp_type(uint8_t Cmd)
{

    uint8_t ret_val = 0;

    switch (Cmd) {
    case CMD0:
        ret_val = SDMMC_RESP_NONE;
        break;
    case CMD1:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD2:
        ret_val = SDMMC_RESP_R136;
        break;
    case CMD3:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD5:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD6:
        ret_val = SDMMC_RESP_R48;
        break;
    case ACMD6:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD7:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD8:
        ret_val = SDMMC_RESP_R48;
        if (Hsd.sd_card.cardtype == SDMMC_CARD_MMC) {
            ret_val |= SDMMC_CMD_R_DATA_PRES_SEL_Msk;
        }
        break;
    case CMD9:
        ret_val = SDMMC_RESP_R136;
        break;
    case CMD11:
    case CMD13:
    case CMD16:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD17:
    case CMD18:
        ret_val = SDMMC_CMD_R_DATA_PRES_SEL_Msk | SDMMC_RESP_R48;
        break;
    case CMD23:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD24:
    case CMD25:
        ret_val = SDMMC_CMD_R_DATA_PRES_SEL_Msk | SDMMC_RESP_R48;
        break;
    case CMD41:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD52:
        ret_val = SDMMC_RESP_R1;
        break;
    case CMD53:
        ret_val = SDMMC_RESP_R48;
        break;
    case CMD55:
        ret_val = SDMMC_RESP_R48;
        break;
    default:
        ret_val = SDMMC_RESP_NONE;
        break;
    }

    return ret_val;
}

/**
  \fn         SDMMC_HC_STATUS hc_send_cmd(sd_handle_t *pHsd, sd_cmd_t *pCmd)
  \brief       Configure Host controller to send SD command
  \param[in]   pHsd - Global SD Handle pointer
  \param[in]   pcmd - command info pointer
  \return      Host controller driver API status.
  */
SDMMC_HC_STATUS hc_send_cmd(sd_handle_t *pHsd, sd_cmd_t *pCmd)
{

    uint16_t cmd, timeout_cnt    = SDMMC_MAX_TIMEOUT_16;
    uint8_t  rsp_type, retry_cnt = 0;

    rsp_type = hc_get_cmd_rsp_type(pCmd->cmdidx);

    cmd      = (pCmd->cmdidx << SDMMC_CMD_IDX_Pos) | (rsp_type);

RETRY:
    pHsd->regs->SDMMC_ERROR_INT_STAT_R  = SDMMC_ERROR_INTR_ALL_Msk;
    pHsd->regs->SDMMC_NORMAL_INT_STAT_R = SDMMC_NORM_INTR_ALL_Msk;
    cc                                  = 0;

    hc_check_bus_idle(pHsd);

    pHsd->regs->SDMMC_XFER_MODE_R = 0;

    if (pCmd->data_present) {

        pHsd->regs->SDMMC_XFER_MODE_R = pCmd->xfer_mode;
    }

    pHsd->regs->SDMMC_ARGUMENT_R = pCmd->arg;
    pHsd->regs->SDMMC_CMD_R      = cmd;

#ifndef SDMMC_IRQ_MODE
    cc = pHsd->regs->SDMMC_NORMAL_INT_STAT_R & SDMMC_INTR_CC_Msk;
#endif

    while (timeout_cnt-- && (!cc)) {
    }

#ifdef SDMMC_PRINTF_DEBUG
    printf("CMD: 0x%04" PRIx16 ", ARG: 0x%08" PRIx32 " XFER: 0x%04" PRIx16 " RSP01: 0x%08" PRIx32
           " PSTATE: 0x%08" PRIx32 " cc:%" PRId16 "\n",
           cmd,
           pCmd->arg,
           pCmd->xfer_mode,
           pHsd->regs->SDMMC_RESP01_R,
           pHsd->regs->SDMMC_PSTATE_REG,
           cc);
#endif

    if ((timeout_cnt == SDMMC_MAX_TIMEOUT_16) || eis) {

        if (!retry_cnt--) {
            return SDMMC_HC_STATUS_ERR;
        } else {
            goto RETRY;
        }
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_reset(sd_handle_t *pHsd, uint8_t reset_val)
  \brief        software reset of the controller
  \param[in]    Global SD Handle pointer
  \param[in]    Reset Value
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_reset(sd_handle_t *pHsd, uint8_t reset_val)
{

    uint8_t  curr_reset_val    = 0;
    uint16_t timeout           = SDMMC_MAX_TIMEOUT_16;

    /* Reset the HC lines */
    pHsd->regs->SDMMC_SW_RST_R = reset_val;

    do {
        curr_reset_val = pHsd->regs->SDMMC_SW_RST_R;
        sys_busy_loop_us(100);
    } while ((curr_reset_val == 1) && timeout--);

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_set_tout(sd_handle_t *pHsd, uint8_t ToutVal)
  \brief        software reset of the controller
  \param[in]    Global SD Handle pointer
  \param[in]    Timeout Value
  \return       Host controller driver status
  */
void hc_set_tout(sd_handle_t *pHsd, uint8_t tout)
{

    /* Set the timeout value */
    pHsd->regs->SDMMC_TOUT_CTRL_R = tout;

    return;
}

/**
  \fn           SDMMC_HC_STATUS hc_power_cycle(sd_handle_t *pHsd)
  \brief        software reset and power cycle of the controller
  \param[in]    Global SD Handle pointer
  \return       Host controller driver status
  */
void hc_power_cycle(sd_handle_t *pHsd)
{

    pHsd->regs->SDMMC_PWR_CTRL_R &= ~SDMMC_PC_BUS_PWR_VDD1_Msk; // disable vdd1
    sys_busy_loop_us(1000);
    pHsd->regs->SDMMC_PWR_CTRL_R |= SDMMC_PC_BUS_PWR_VDD1_Msk;  // enable vdd1
    sys_busy_loop_us(1000);

    pHsd->regs->SDMMC_CLK_CTRL_R        |= SDMMC_CLK_EN_Msk;

    pHsd->regs->SDMMC_NORMAL_INT_STAT_R  = SDMMC_NORM_INTR_ALL_Msk;
    pHsd->regs->SDMMC_ERROR_INT_STAT_R   = SDMMC_ERROR_INTR_ALL_Msk;

    hc_reset(pHsd, SDMMC_SW_RST_CMD_Msk | SDMMC_SW_RST_DAT_Msk);

    return;
}

/**
  \fn           SDMMC_HC_STATUS hc_set_bus_power(sd_handle_t *pHsd, uint8_t bus_power) {
  \brief        Set required sd bus power supply
  \param[in]    Global SD Handle pointer
  \param[in]    required bus voltage equivalent register value as per host controller data sheet
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_set_bus_power(sd_handle_t *pHsd, uint8_t bus_power)
{

    uint32_t status, timeout_cnt = 0xFFFF;

    /* Disable the power supply */
    pHsd->regs->SDMMC_PWR_CTRL_R &= ~SDMMC_PC_BUS_PWR_VDD1_Msk;

    /* If selected power is zero, return from here */
    if (bus_power == SDMMC_PC_BUS_PWR_OFF) {
        return SDMMC_HC_STATUS_OK;
    }

    if (bus_power == SDMMC_PC_BUS_VSEL_1V8_Msk) {
        /* set the 1.8v */
        /* delay or check bus gating status */
        sys_busy_loop_us(10000);

        /* Disable clock */
        HC_CLOCK_DISABLE(pHsd);

        pHsd->regs->SDMMC_PWR_CTRL_R = SDMMC_PC_BUS_VSEL_1V8_Msk;

        /* Delay 10ms eventhough spec says 5ms after switching to 1.8v */
        sys_busy_loop_us(10000);
        pHsd->regs->SDMMC_PWR_CTRL_R   |= SDMMC_PC_BUS_PWR_VDD1_Msk;
        pHsd->regs->SDMMC_HOST_CTRL2_R |= SDMMC_HOST_CTRL2_SIGNALING_EN_Msk;

        /* Enable clock and let is stable */
        HC_CLOCK_ENABLE(pHsd);

        /* check CMD line status after voltage switch */
        do {
            status = pHsd->regs->SDMMC_PSTATE_REG & SDMMC_CMD_LINE_LVL_UP_Msk;
            sys_busy_loop_us(1);
        } while (!status && timeout_cnt--);

        return status ? SDMMC_HC_STATUS_OK : SDMMC_HC_STATUS_ERR;
    } else {
        /* set the 3.3v */
        pHsd->regs->SDMMC_PWR_CTRL_R    = bus_power;
        pHsd->regs->SDMMC_HOST_CTRL2_R &= ~SDMMC_HOST_CTRL2_SIGNALING_EN_Msk;
    }

    /* Enable the power supply */
    pHsd->regs->SDMMC_PWR_CTRL_R |= SDMMC_PC_BUS_PWR_VDD1_Msk;

    /* Enable Clock */
    pHsd->regs->SDMMC_CLK_CTRL_R |= SDMMC_CLK_EN_Msk;

    /* .2ms delay after power on/off */
    sys_busy_loop_us(2000);

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_set_clk_freq(sd_handle_t *pHsd, uint32_t clk_freq)
  \brief        Sets required clock
  \param[in]    Global SD Handle pointer
  \param[in]    required bus clock equivalent register value as per host controller data sheet
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_set_clk_freq(sd_handle_t *pHsd, uint16_t clk_freq)
{

    uint16_t reg;

    /* Disable clock */
    pHsd->regs->SDMMC_CLK_CTRL_R = 0;

    /* If selected frequency is zero, return from here */
    if (clk_freq == 0U) {
        return SDMMC_HC_STATUS_OK;
    }

    pHsd->regs->SDMMC_CLK_CTRL_R = clk_freq;

    sys_busy_loop_us(1000);

    /* SD clk stability */
    do {
        reg = pHsd->regs->SDMMC_CLK_CTRL_R;
        sys_busy_loop_us(1);
    } while ((reg & SDMMC_INTERNAL_CLK_STABLE_Msk) == 0);

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_config_dma(sd_handle_t *pHsd,uint8_t dmaMask)
  \brief        Host Controller DMA configuration
  \param[in]    Global SD Handle pointer
  \param[in]    dmaMask - Host ctrl 1 register value
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_config_dma(sd_handle_t *pHsd, uint8_t dmaMask)
{

    /* Host Version 4 Param */
    pHsd->regs->SDMMC_HOST_CTRL2_R = SDMMC_HOST_CTRL2_ASYNC_INT_EN_Msk |
                                     SDMMC_HOST_CTRL2_VER4_EN_Msk | SDMMC_HOST_CTRL2_CMD23_EN_Msk;

    pHsd->regs->SDMMC_HOST_CTRL1_R = dmaMask;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_set_bus_width(sd_handle_t *pHsd, uint8_t buswidth)
  \brief        Configure required bit for communication for Host and Card
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    buswidth - number of Data lines for data transfer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_set_bus_width(sd_handle_t *pHsd, uint8_t buswidth)
{

    uint8_t  regs;
    uint32_t status;
    uint32_t timeout_cnt = SDMMC_CMD_TIMEOUT;

    do {

        hc_get_card_status(pHsd, &status);
        if (!timeout_cnt--) {
            return SDMMC_HC_STATUS_INV_STATE; /* SD Must be TRAN state to change Bus width */
        }
    } while (status != SD_CARD_STATE_TRAN);

    if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {
        if (buswidth > SDMMC_4_BIT_MODE) {
            /* invalid initial parameter, switching back to max supported bus width for sd card */
#ifdef SDMMC_PRINT_WARN
            printf("invalid initial parameter, switching back to max supported bus width for sd "
                   "card\n");
#endif
            buswidth                 = SDMMC_4_BIT_MODE;
            pHsd->sd_param.bus_width = SDMMC_4_BIT_MODE;
        }
    }

    regs = pHsd->regs->SDMMC_HOST_CTRL1_R;

    if (buswidth == SDMMC_1_BIT_MODE) {
        regs = regs | SDMMC_1_BIT_WIDTH_Msk;
    } else if (buswidth == SDMMC_4_BIT_MODE) {
        regs = regs | SDMMC_4_BIT_WIDTH_Msk | SDMMC_HOST_CTRL1_HIGH_SPEED_MODE_EN;
    } else {
        regs = regs | SDMMC_8_BIT_WIDTH_Msk | SDMMC_HOST_CTRL1_HIGH_SPEED_MODE_EN;
    }

    pHsd->regs->SDMMC_HOST_CTRL1_R = regs;

    /* just to indicate Card that the next cmd is APP CMD if not MMC/eMMC Card */
    if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {

        pHsd->sd_cmd.cmdidx = CMD55;
        pHsd->sd_cmd.arg    = pHsd->sd_card.relcardadd;

        if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
            sd_error_handler();
        }

        pHsd->sd_cmd.cmdidx = CMD6;
        pHsd->sd_cmd.arg    = 0x2;
        if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
            sd_error_handler();
        }

    } else {

        /* Send CMD6 to write protect boot partition 1 */
        pHsd->sd_cmd.cmdidx = CMD6;
        pHsd->sd_cmd.arg    = SDMMC_EXT_CSD_WRITE_Msk |
                           SDMMC_EXT_CSD_IDX_Msk(SDMMC_EXT_CSD_CMD_BOOT_CFG) |
                           (SDMMC_EXT_CSD_BOOT_WR_PROTECT << SDMMC_EXT_CSD_VAL_Pos);
        if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
            sd_error_handler();
        }
        sys_busy_loop_us(SDMMC_CMD6_DELAY);

        /* Switch HS timing */
        pHsd->sd_cmd.arg = SDMMC_EXT_CSD_WRITE_Msk |
                           SDMMC_EXT_CSD_IDX_Msk(SDMMC_EXT_CSD_CMD_HS_MODE) |
                           (SDMMC_EXT_CSD_HS_MODE << SDMMC_EXT_CSD_VAL_Pos);
        if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
            sd_error_handler();
        }
        sys_busy_loop_us(SDMMC_CMD6_DELAY);

        /* Send ACMD6 to change Bus width */
        pHsd->sd_cmd.arg = SDMMC_EXT_CSD_WRITE_Msk |
                           SDMMC_EXT_CSD_IDX_Msk(SDMMC_EXT_CSD_CMD_BUS_WIDTH) |
                           (buswidth << SDMMC_EXT_CSD_VAL_Pos);
        if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
            sd_error_handler();
        }
        sys_busy_loop_us(SDMMC_CMD6_DELAY);
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           void hc_config_interrupt(sd_handle_t *pHsd)
  \brief        Configure Error and interrupt in Host Controller
  \param[in]    pHsd - Global SD Handle pointer
  \return       none
  */
void hc_config_interrupt(sd_handle_t *pHsd)
{

    /* Enable all interrupt status except card interrupt initially */
    pHsd->regs->SDMMC_NORMAL_INT_STAT_EN_R   = SDMMC_NORM_INTR_ALL_Msk & (~SDMMC_INTR_CARD_Msk);

    pHsd->regs->SDMMC_ERROR_INT_STAT_EN_R    = SDMMC_ERROR_INTR_ALL_Msk;

    /* Disable all interrupt signals by default. */
    pHsd->regs->SDMMC_NORMAL_INT_SIGNAL_EN_R = 0x0U;

    pHsd->regs->SDMMC_ERROR_INT_SIGNAL_EN_R  = 0x0U;
}

/**
  \fn           SDMMC_HC_STATUS hc_identify_card(sd_handle_t *pHsd)
  \brief        Indentify the inserted cards
  \param[in]    Global SD Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_identify_card(sd_handle_t *pHsd)
{

    uint32_t        isCardPresent = 0, timeout = 0xFFFFU;
    SDMMC_HC_STATUS status;

    hc_reset(pHsd,
             (uint8_t) (SDMMC_SW_RST_DAT_Msk | SDMMC_SW_RST_CMD_Msk));  // reset CMD and DATA Line

#ifdef SDMMC_PRINTF_DEBUG
    printf("Waiting for SD Card to be inserted...\n");
#endif

    /* Card Insertion and Removal State and Signal Enable */
    pHsd->regs->SDMMC_NORMAL_INT_SIGNAL_EN_R =
        SDMMC_INTR_CC_Msk | SDMMC_INTR_TC_Msk | SDMMC_INTR_DMA_Msk | SDMMC_INTR_BWR_Msk |
        SDMMC_INTR_BRR_Msk | SDMMC_INTR_CARD_INSRT_Msk | SDMMC_INTR_CARD_REM_Msk;

    pHsd->regs->SDMMC_NORMAL_INT_STAT_EN_R =
        SDMMC_INTR_CC_Msk | SDMMC_INTR_TC_Msk | SDMMC_INTR_DMA_Msk | SDMMC_INTR_BWR_Msk |
        SDMMC_INTR_BRR_Msk | SDMMC_INTR_CARD_INSRT_Msk | SDMMC_INTR_CARD_REM_Msk;

    pHsd->regs->SDMMC_ERROR_INT_STAT_EN_R = SDMMC_ERROR_INTR_ALL_Msk;

    do {

        isCardPresent               = pHsd->regs->SDMMC_PSTATE_REG;
        isCardPresent               = (isCardPresent & SDMMC_CARD_INSRT_Msk);

        pHsd->sd_card.iscardpresent = isCardPresent;
#ifdef SDMMC_PRINTF_DEBUG
        printf("SD Card Detected...\n");
#endif
        sys_busy_loop_us(1);

    } while (timeout-- && (!isCardPresent));

    status = SDMMC_HC_STATUS_OK;

    if (timeout == SDMMC_MAX_TIMEOUT_16) {
        pHsd->error_code = SD_DRV_STATUS_TIMEOUT_ERR;
        status           = SDMMC_HC_STATUS_ERR;
    }

    pHsd->regs->SDMMC_NORMAL_INT_SIGNAL_EN_R =
        SDMMC_INTR_CC_Msk | SDMMC_INTR_TC_Msk | SDMMC_INTR_DMA_Msk | SDMMC_INTR_BWR_Msk |
        SDMMC_INTR_BRR_Msk | SDMMC_INTR_CARD_INSRT_Msk | SDMMC_INTR_CARD_REM_Msk;

    pHsd->regs->SDMMC_NORMAL_INT_STAT_EN_R =
        SDMMC_INTR_CC_Msk | SDMMC_INTR_TC_Msk | SDMMC_INTR_DMA_Msk | SDMMC_INTR_BWR_Msk |
        SDMMC_INTR_BRR_Msk | SDMMC_INTR_CARD_INSRT_Msk | SDMMC_INTR_CARD_REM_Msk;

    return status;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_card_ifcond(sd_handle_t *pHsd)
  \brief        Get Card Interface condition
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_card_ifcond(sd_handle_t *pHsd)
{

    uint32_t resp_ifcond;

    /* Change the Card State from Idle to Identification */
    pHsd->state         = SD_CARD_STATE_IDENT;

    pHsd->sd_cmd.cmdidx = CMD8;
    pHsd->sd_cmd.arg    = SDMMC_CMD8_VOL_PATTERN;
    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    resp_ifcond = pHsd->regs->SDMMC_RESP01_R;

    if (resp_ifcond != SDMMC_CMD8_VOL_PATTERN) {
        /* SD Version 1 Card */
        pHsd->sd_card.cardversion = SDMMC_CARD_SDSC;
        pHsd->sd_card.cardtype    = SDMMC_CARD_SDSC;
        pHsd->sd_card.f8flag      = 1;
    } else {
        /* SD Version 2 or Later Card */
        pHsd->sd_card.cardversion = SDMMC_CARD_SDHC;
        pHsd->sd_card.cardtype    = SDMMC_CARD_SDHC;
        pHsd->sd_card.f8flag      = 0;
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_emmc_card_opcond(sd_handle_t *pHsd)
  \brief        Get eMMC Card operating voltage condition
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_emmc_card_opcond(sd_handle_t *pHsd)
{

    uint32_t resp_OPcond;
    uint32_t timeout   = 0xFFU;
    uint32_t switch1v8 = 0;

    hc_power_cycle(pHsd);

    hc_reset(pHsd, SDMMC_SW_RST_CMD_Msk | SDMMC_SW_RST_DAT_Msk);
    hc_go_idle(pHsd);

    do {

        pHsd->sd_cmd.cmdidx = CMD1;
        pHsd->sd_cmd.arg    = (SDMMC_CMD41_HCS | SDMMC_CMD41_3V3);  // | SD_OCR_S18R);
        if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
            sd_error_handler();
        }

        resp_OPcond = pHsd->regs->SDMMC_RESP01_R;
        switch1v8   = resp_OPcond & SDMMC_CMD41_S18A;
        resp_OPcond = pHsd->regs->SDMMC_RESP01_R;
        resp_OPcond = resp_OPcond & SDMMC_OCR_READY;
    } while ((!resp_OPcond) && (timeout--));

    if (timeout == SDMMC_MAX_TIMEOUT_32) {
        return SDMMC_HC_STATUS_ERR;
    }

    /* UHS-I Specific Initializations */
    if (switch1v8) {
        pHsd->sd_card.flags |= SDMMC_1P8V_FLAG;
    }

    /* detected card is eMMC */
    pHsd->regs->SDMMC_EMMC_CTRL_R |= 1;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_card_opcond(sd_handle_t *pHsd)
  \brief        Get Card operating voltage condition
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_card_opcond(sd_handle_t *pHsd, uint32_t req_ocr)
{

    uint32_t resp_OPcond;
    uint32_t timeout   = 0x1FFU;         /* Added extra delay for diff types of card */
    uint32_t switch1v8 = 0;
    uint32_t ocr;
    uint32_t sdio_func_number = 0;

    /* Get the SD/SDIO card operating condition */
    if (hc_get_io_opcond(pHsd, 0, &resp_OPcond) == SDMMC_HC_STATUS_OK) {
        /* IO functionality found in this card */

        sdio_func_number = (resp_OPcond & CMD5_RESP_NIOF_PRES_Msk) >> CMD5_RESP_NIOF_PRES_Pos;
        ocr              = (resp_OPcond & CMD5_RESP_OCR_3V3_Msk);

        if (sdio_func_number && ocr) {
            hc_get_io_opcond(pHsd, ocr, &resp_OPcond);
        }

        if (resp_OPcond & CMD5_RESP_IO_READY_Msk) {

            /* update the global instance */
            pHsd->sd_card.sdio.sdio_opcd.func_number = sdio_func_number;
            pHsd->sd_card.sdio_mode                  = 1;
        }

    } else {
        /* Memory */

        hc_power_cycle(pHsd);

        hc_reset(pHsd, SDMMC_SW_RST_CMD_Msk | SDMMC_SW_RST_DAT_Msk);
        hc_go_idle(pHsd);
        hc_get_card_ifcond(pHsd);

        do {
            /* just to indicate SD Card that the next cmd is APP CMD */
            pHsd->sd_cmd.cmdidx = CMD55;
            pHsd->sd_cmd.arg    = 0x0;
            if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
                sd_error_handler();
            }

            pHsd->sd_cmd.cmdidx       = CMD41;
            pHsd->sd_cmd.arg          = req_ocr;

            if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
                sd_error_handler();
            }

            resp_OPcond = pHsd->regs->SDMMC_RESP01_R;
            switch1v8   = resp_OPcond & SDMMC_CMD41_S18A;
            resp_OPcond = pHsd->regs->SDMMC_RESP01_R;
            resp_OPcond = resp_OPcond & SDMMC_OCR_READY;
        } while ((!resp_OPcond) && (timeout--));
    }

    if (timeout == SDMMC_MAX_TIMEOUT_32) {
        return SDMMC_HC_STATUS_ERR;
    }

    if (!(pHsd->regs->SDMMC_RESP01_R & SDMMC_CMD41_HCS)) {
        pHsd->sd_card.cardversion = SDMMC_CARD_SDSC;
        pHsd->sd_card.cardtype    = SDMMC_CARD_SDSC;
    }

    /* UHS-I Specific Initializations */
    if (switch1v8) {
        pHsd->sd_card.flags |= SDMMC_1P8V_FLAG;
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_card_cid(sd_handle_t *pHsd)
  \brief        Get Card Identification information
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_card_cid(sd_handle_t *pHsd)
{

    pHsd->sd_cmd.cmdidx = CMD2;
    pHsd->sd_cmd.arg    = 0x0;
    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->cid[0] = pHsd->regs->SDMMC_RESP01_R;
    pHsd->cid[1] = pHsd->regs->SDMMC_RESP23_R;
    pHsd->cid[2] = pHsd->regs->SDMMC_RESP45_R;
    pHsd->cid[3] = pHsd->regs->SDMMC_RESP67_R;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_card_ext_csd(sd_handle_t *pHsd, uint8_t *pbuff)
  \brief        Get Card Specific Data
  \param[in]    Global sd Handle pointer
  \param[in]    buffer pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_card_ext_csd(sd_handle_t *pHsd, uint8_t *pbuff)
{

    /* Select a card */
    if (hc_sel_card(pHsd, pHsd->sd_card.relcardadd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    if (hc_set_blk_size(pHsd, SDMMC_BLK_SIZE_512_Msk) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    hc_dma_config(pHsd, (uint32_t) LocalToGlobal(pbuff), 1);

    pHsd->sd_cmd.cmdidx       = CMD8;
    pHsd->sd_cmd.arg          = 0;
    pHsd->sd_cmd.data_present = 1;
    pHsd->sd_cmd.xfer_mode    = SDMMC_XFER_MODE_DATA_XFER_RD_Msk | SDMMC_XFER_MODE_DMA_EN_Msk;

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_cmd.data_present = 0;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_card_csd(sd_handle_t *pHsd)
  \brief        Get Card Specific Data
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_card_csd(sd_handle_t *pHsd)
{

    pHsd->sd_cmd.cmdidx = CMD9;
    pHsd->sd_cmd.arg    = pHsd->sd_card.relcardadd;
    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        return SDMMC_HC_STATUS_ERR;
    }

    /* update the global instance */
    pHsd->csd[0] = pHsd->regs->SDMMC_RESP01_R;
    pHsd->csd[1] = pHsd->regs->SDMMC_RESP23_R;
    pHsd->csd[2] = pHsd->regs->SDMMC_RESP45_R;
    pHsd->csd[3] = pHsd->regs->SDMMC_RESP67_R;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_card_scr(sd_handle_t *pHsd)
  \brief        Get SCR
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_card_scr(sd_handle_t *pHsd)
{

    /* just to indicate SD Card that the next cmd is APP CMD */
    pHsd->sd_cmd.cmdidx = CMD55;
    pHsd->sd_cmd.arg    = 0x0;

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_cmd.cmdidx = CMD51;
    pHsd->sd_cmd.arg    = 0;

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    /* update the global instance */
    pHsd->scr[0] = pHsd->regs->SDMMC_RESP01_R;
    pHsd->scr[1] = pHsd->regs->SDMMC_RESP23_R;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_go_idle(sd_handle_t *pHsd)
  \brief        put the card in idle mode
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_go_idle(sd_handle_t *pHsd)
{

    pHsd->sd_cmd.cmdidx = CMD0;
    pHsd->sd_cmd.arg    = 0;
    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->state = SD_CARD_STATE_IDLE;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_sel_card(sd_handle_t *pHsd, uint32_t rca)
  \brief        Select a card for further operation
  \param[in]    Global sd Handle pointer
  \param[in]    Card Relative Address
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_sel_card(sd_handle_t *pHsd, uint32_t rca)
{

    /* Select the card to transition to transfer state */
    pHsd->sd_cmd.cmdidx = CMD7;
    pHsd->sd_cmd.arg    = rca;
    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    /* Change the Card State from Ready to Tran */
    pHsd->state = SD_CARD_STATE_TRAN;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_set_blk_size(sd_handle_t *pHsd, uint32_t BlkSize)
  \brief        Set the Block Size
  \param[in]    Global sd Handle pointer
  \param[in]    Block size
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_set_blk_size(sd_handle_t *pHsd, uint32_t blk_size)
{

    if (pHsd->regs->SDMMC_BLOCKSIZE_R != blk_size) {

        /* set the block size */
        pHsd->sd_cmd.cmdidx = CMD16;
        pHsd->sd_cmd.arg    = blk_size;
        if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
            sd_error_handler();
        }

        pHsd->regs->SDMMC_BLOCKSIZE_R = blk_size;
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_rca(sd_handle_t *pHsd, uint32_t *prca)
  \brief        Get the Card Relative Address
  \param[in]    Global sd Handle pointer
  \param[in]    RCA destination pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_rca(sd_handle_t *pHsd, uint32_t *prca)
{
    /* Get the card Relative Addr */
    pHsd->sd_cmd.cmdidx = CMD3;

    if (pHsd->sd_card.cardtype == SDMMC_CARD_MMC) {
        pHsd->sd_cmd.arg = EMMC_DEFAULT_RCA;
        *prca            = EMMC_DEFAULT_RCA;
    }
    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    if (pHsd->sd_card.cardtype != SDMMC_CARD_MMC) {
        *prca = pHsd->regs->SDMMC_RESP01_R & SDMMC_RCA_Msk;
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_capabilities(sd_handle_t *pHsd, uint32_t *phcaps)
  \brief        Get the Host Controller Capabilities
  \param[in]    Global sd Handle pointer
  \param[in]    Capabilities pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_capabilities(sd_handle_t *pHsd, uint32_t *phcaps)
{

    /* Get the card Relative Addr */
    *phcaps = pHsd->regs->SDMMC_CAPABILITIES1_R;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_response1(sd_handle_t *pHsd)
  \brief        Get the Card Response1
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t hc_get_response1(sd_handle_t *pHsd)
{

    /* Get the card Response1 */
    return pHsd->regs->SDMMC_RESP01_R;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_response2(sd_handle_t *pHsd)
  \brief        Get the Card Response2
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t hc_get_response2(sd_handle_t *pHsd)
{

    /* Get the card Response2 */
    return pHsd->regs->SDMMC_RESP23_R;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_response3(sd_handle_t *pHsd)
  \brief        Get the Card Response3
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t hc_get_response3(sd_handle_t *pHsd)
{

    /* Get the card Response3 */
    return pHsd->regs->SDMMC_RESP45_R;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_response4(sd_handle_t *pHsd)
  \brief        Get the Card Response1
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t hc_get_response4(sd_handle_t *pHsd)
{

    /* Get the card Response4 */
    return pHsd->regs->SDMMC_RESP67_R;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_card_status(sd_handle_t *pHsd, uint32_t *pstatus)
  \brief        Get the Card Status
  \param[in]    Global sd Handle pointer
  \param[in]    status pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_card_status(sd_handle_t *pHsd, uint32_t *pstatus)
{

    uint32_t status;

    /* Check current card status */
    pHsd->sd_cmd.cmdidx = CMD13;
    pHsd->sd_cmd.arg    = pHsd->sd_card.relcardadd;

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    status   = hc_get_response1(pHsd);

    *pstatus = (status & SDMMC_STATUS_Msk) >> SDMMC_STATUS_Pos;

#ifdef SDMMC_PRINTF_DEBUG
    printf("Card Status: %" PRIx32 "\n", status);
#endif

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_set_blk_cnt(sd_handle_t *pHsd, uint32_t blk_cnt)
  \brief        Set the Block Count
  \param[in]    Global sd Handle pointer
  \param[in]    Block Count
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_set_blk_cnt(sd_handle_t *pHsd, uint32_t blk_cnt)
{
    pHsd->sd_cmd.cmdidx = CMD23;
    pHsd->sd_cmd.arg    = blk_cnt;

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->regs->SDMMC_BLOCKCOUNT_R = blk_cnt;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_dma_config(sd_handle_t *pHsd, uint32_t buff, uint16_t blk_cnt) {
  \brief        Setup read parameter and start reading sector
  \param[in]    Global sd Handle pointer
  \param[in]    destination buffer
  \param[in]    Block Count
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_dma_config(sd_handle_t *pHsd, uint32_t buff, uint16_t blk_cnt)
{

    if ((pHsd->sd_card.cardtype != SDMMC_CARD_SDSC)) {
        hc_set_blk_cnt(pHsd, blk_cnt);
    }

    /* Configure DMA buffer */
    if (pHsd->sd_param.dma_mode == SDMMC_HOST_CTRL1_ADMA2_MODE) {

        uint32_t desc_num   = 0;
        uint32_t total_desc = 0, bytes_per_desc;

        bytes_per_desc      = blk_cnt * SDMMC_BLK_SIZE_512_Msk;

        if (bytes_per_desc > SDMMC_ADMA2_DESC_MAX_LEN) {

            total_desc = bytes_per_desc / SDMMC_ADMA2_DESC_MAX_LEN;
            if (bytes_per_desc % SDMMC_ADMA2_DESC_MAX_LEN) {
                total_desc += 1;
            }
        }

        do {
            adma_desc_tbl[desc_num].addr = buff + (desc_num * SDMMC_ADMA2_DESC_MAX_LEN);
            adma_desc_tbl[desc_num].len  = bytes_per_desc;
            adma_desc_tbl[desc_num].attr = SDMMC_ADMA2_DESC_TRAN | SDMMC_ADMA2_DESC_VALID;
        } while (desc_num++ < total_desc);

        adma_desc_tbl[desc_num - 1].attr |= SDMMC_ADMA2_DESC_END;
#ifdef SDMMC_PRINTF_DEBUG
        printf("ADMA Desc: 0x%" PRIx32 ", addr: 0x%" PRIx32 ", Len: 0x%" PRIx16 ", Attr: 0x%" PRIx16
               "\n",
               (uint32_t) &adma_desc_tbl[0],
               adma_desc_tbl[0].addr,
               adma_desc_tbl[0].len,
               adma_desc_tbl[0].attr);
#endif

        RTSS_CleanDCache_by_Addr(&adma_desc_tbl[0], sizeof(adma_desc_tbl));
        pHsd->regs->SDMMC_ADMA_SA_LOW_R = (uint32_t) LocalToGlobal((&adma_desc_tbl[0]));
    } else {
        pHsd->regs->SDMMC_ADMA_SA_LOW_R = (uint32_t) LocalToGlobal((const volatile void *) buff);
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_read_setup(sd_handle_t *pHsd, uint32_t buff, uint32_t sector,
  uint16_t blk_cnt) { \brief        Setup read parameter and start reading sector \param[in] Global
  sd Handle pointer \param[in]    destination buffer \param[in]    sector number to read \param[in]
  Block Count \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_read_setup(sd_handle_t *pHsd, uint32_t buff, uint32_t sector, uint16_t blk_cnt)
{

    hc_dma_config(pHsd, buff, blk_cnt);

    pHsd->sd_cmd.arg = sector;
    if (pHsd->sd_card.cardtype == SDMMC_CARD_SDSC) {
        pHsd->sd_cmd.arg <<= 9;
    }

    pHsd->sd_cmd.data_present = 1;

    if (blk_cnt == 1) {
        pHsd->sd_cmd.cmdidx    = CMD17;
        pHsd->sd_cmd.xfer_mode = SDMMC_XFER_MODE_DATA_XFER_RD_Msk | SDMMC_XFER_MODE_DMA_EN_Msk;
    } else {

        pHsd->sd_cmd.cmdidx    = CMD18;
        pHsd->sd_cmd.xfer_mode = SDMMC_XFER_MODE_DATA_XFER_RD_Msk |
                                 SDMMC_XFER_MODE_MULTI_BLK_SEL_Msk |
                                 (SDMMC_XFER_MODE_AUTO_CMD12 << SDMMC_XFER_MODE_AUTO_CMD_EN_Pos) |
                                 SDMMC_XFER_MODE_DMA_EN_Msk;

        if (pHsd->sd_param.dma_mode == SDMMC_HOST_CTRL1_SDMA_MODE) {
            pHsd->sd_cmd.xfer_mode |= SDMMC_XFER_MODE_BLK_CNT_Msk;
        }
    }

    pHsd->regs->SDMMC_HOST_CTRL1_R |= 1;  // led caution on

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_cmd.data_present = 0;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_write_setup(sd_handle_t *pHsd, uint32_t buff, uint32_t sector,
  uint16_t blk_cnt) \brief        Setup write parameter and start writing sector \param[in] Global
  sd Handle pointer \param[in]    source buffer \param[in]    sector number to write \param[in]
  Block Count \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_write_setup(sd_handle_t *pHsd, uint32_t buff, uint32_t sector, uint16_t blk_cnt)
{

    hc_dma_config(pHsd, buff, blk_cnt);

    pHsd->sd_cmd.arg = sector;
    if (pHsd->sd_card.cardtype == SDMMC_CARD_SDSC) {
        pHsd->sd_cmd.arg <<= 9;
    }

    pHsd->sd_cmd.data_present = 1;

    if (blk_cnt == 1) {
        pHsd->sd_cmd.cmdidx    = CMD24;
        pHsd->sd_cmd.xfer_mode = SDMMC_XFER_MODE_DMA_EN_Msk;
    } else {
        pHsd->sd_cmd.cmdidx    = CMD25;
        pHsd->sd_cmd.xfer_mode = SDMMC_XFER_MODE_DATA_XFER_WR_Msk |
                                 SDMMC_XFER_MODE_MULTI_BLK_SEL_Msk | SDMMC_XFER_MODE_BLK_CNT_Msk |
                                 SDMMC_XFER_MODE_DMA_EN_Msk;

        if (pHsd->sd_param.dma_mode == SDMMC_HOST_CTRL1_SDMA_MODE) {
            pHsd->sd_cmd.xfer_mode |= SDMMC_XFER_MODE_BLK_CNT_Msk;
        }
    }

    pHsd->regs->SDMMC_HOST_CTRL1_R |= SDMMC_HOST_CTRL1_LED_ON;  // led caution on

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        sd_error_handler();
    }

    pHsd->sd_cmd.data_present = 0;

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_check_xfer_done(sd_handle_t *pHsd, uint32_t timeout_cnt)
  \brief        Check for transfer complete
  \param[in]    Global sd Handle pointer
  \param[in]    timeout Count
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_check_xfer_done(sd_handle_t *pHsd, uint32_t timeout_cnt)
{
    uint32_t pstate = 0, dma_irq = 0, xfer_comp = 0;

    /* check for transfer active state */
    while (timeout_cnt) {
#ifndef SDMMC_IRQ_MODE
        nis = pHsd->regs->SDMMC_NORMAL_INT_STAT_R;
#endif
        dma_irq   = nis;
        pstate    = pHsd->regs->SDMMC_PSTATE_REG;

        pstate    = pstate & XFER_ACTIVE_Msk;

        xfer_comp = dma_irq & NORMAL_INT_STAT_XFER_COMPLETE_Msk;
        if (xfer_comp && !pstate) {
            break;
        }

        sys_busy_loop_us(1);

        timeout_cnt--;
    }

    pHsd->regs->SDMMC_HOST_CTRL1_R ^= SDMMC_HOST_CTRL1_LED_ON;  // led caution off
#ifdef SDMMC_PRINTF_SD_STATE_DEBUG
    printf("PSTATE REG:0x%08" PRIx32 "\tAUTO_CMD_STAT:0x%04h" PRIx16 "\tERROR_INT_STAT_R:0x%" PRIx8
           "\n",
           pHsd->regs->SDMMC_PSTATE_REG,
           pHsd->regs->SDMMC_AUTO_CMD_STAT_R,
           (uint8_t) pHsd->regs->SDMMC_ERROR_INT_STAT_R);
#endif

    if (!timeout_cnt) {
        return SDMMC_HC_STATUS_ERR;
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_check_bus_idle(sd_handle_t *pHsd)
  \brief        Check the CMD line is idle or nor
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_check_bus_idle(sd_handle_t *pHsd)
{
    uint32_t pstate, cmdinhibit, datinhibit, timeout = 0x100000;

    do {

        pstate     = pHsd->regs->SDMMC_PSTATE_REG;
        cmdinhibit = pstate & SDMMC_CMD_INHIBIT_Msk;
        datinhibit = pstate & SDMMC_DAT_INHIBIT_Msk;

        if (!timeout--) {
            return SDMMC_HC_STATUS_ERR;
        }

    } while (cmdinhibit || (pHsd->sd_cmd.data_present && datinhibit));

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_io_reset(sd_handle_t *pHsd)
  \brief        Reset the IO/Combo Card
  \param[in]    Global sd Handle pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_io_reset(sd_handle_t *pHsd)
{
    pHsd->sd_cmd.cmdidx = SDIO_RW_DIRECT;
    pHsd->sd_cmd.arg    = 0;
    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        return SDMMC_HC_STATUS_ERR;
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_get_io_opcond(sd_handle_t *pHsd, uint32_t *)
  \brief        Reset the IO/Combo Card
  \param[in]    Global sd Handle pointer
  \param[in]    ocr response destination pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_get_io_opcond(sd_handle_t *pHsd, uint32_t ocr, uint32_t *rocr)
{

    pHsd->sd_cmd.cmdidx = SDIO_SEND_OP_COND;
    pHsd->sd_cmd.arg    = ocr;

    if (hc_send_cmd(pHsd, &pHsd->sd_cmd) != SDMMC_HC_STATUS_OK) {
        return SDMMC_HC_STATUS_ERR;
    }

    *rocr = pHsd->regs->SDMMC_RESP01_R;
    if (*rocr == SDMMC_CMD8_VOL_PATTERN) {
        return SDMMC_HC_STATUS_ERR;
    }

    return SDMMC_HC_STATUS_OK;
}

/**
  \fn           SDMMC_HC_STATUS hc_io_rw_direct(sd_handle_t *pHsd, uint32_t *)
  \brief        Reset the IO/Combo Card
  \param[in]    Global sd Handle pointer
  \param[in]    ocr response destination pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_io_rw_direct(sd_handle_t *pHsd, uint32_t rwFlag, uint32_t fn, uint32_t addr,
                                uint8_t writeData, uint8_t *readPtr)
{
    SDMMC_HC_STATUS status = SDMMC_HC_STATUS_OK;
    sd_cmd_t        sdio_cmd;
    uint32_t        resp;

    if ((fn > SDIO_MAX_FUNCTION) || (addr & ~(SDIO_MAX_CIA_ADDR))) {
        return SDMMC_HC_STATUS_ERR;
    }

    sdio_cmd.xfer_mode  = SDMMC_XFER_MODE_DATA_XFER_RD_Msk | SDMMC_XFER_MODE_DMA_EN_Msk;
    sdio_cmd.cmdidx     = SDIO_RW_DIRECT;
    sdio_cmd.arg        = rwFlag ? SDIO_RW_FLAG_Msk : 0x00000000;
    sdio_cmd.arg       |= fn << SDIO_FN_Pos;
    sdio_cmd.arg       |= (rwFlag && readPtr) ? SDIO_RAW_FLAG_Msk : 0x00000000;
    sdio_cmd.arg       |= addr << SDIO_REG_ADDR_Pos;
    sdio_cmd.arg       |= rwFlag ? writeData : 0x00000000;

    if (hc_send_cmd(pHsd, &sdio_cmd) != SDMMC_HC_STATUS_OK) {
        return SDMMC_HC_STATUS_ERR;
    }

    resp = pHsd->regs->SDMMC_RESP01_R;

    if ((resp & SDIO_R5_ERROR) || (resp & SDIO_R5_FUNCTION_NUMBER) ||
        (resp & SDIO_R5_OUT_OF_RANGE)) {
        return SDMMC_HC_STATUS_ERR;
    }

    if (readPtr) {
        *readPtr = resp & 0xFF;
    }

    return status;
}

/**
  \fn           SDMMC_HC_STATUS hc_io_rw_extended(sd_handle_t *pHsd, uint32_t *)
  \brief        Reset the IO/Combo Card
  \param[in]    Global sd Handle pointer
  \param[in]    ocr response destination pointer
  \return       Host controller driver status
  */
SDMMC_HC_STATUS hc_io_rw_extended(sd_handle_t *pHsd, uint32_t rwFlag, uint32_t fn, uint32_t addr,
                                  uint32_t incr_addr, uint8_t *buf, uint32_t blk_cnt,
                                  uint32_t blkSize)
{

    SDMMC_HC_STATUS status = SDMMC_HC_STATUS_OK;
    sd_cmd_t        sdio_cmd;

    if ((fn > SDIO_MAX_FUNCTION) || (addr & ~(SDIO_MAX_CIA_ADDR))) {
        status = SDMMC_HC_STATUS_ERR;
        goto exit;
    }

    /* Configure DMA */
    pHsd->regs->SDMMC_ADMA_SA_LOW_R  = (uint32_t) LocalToGlobal((const volatile void *) buf);

    sdio_cmd.cmdidx                  = SDIO_RW_EXTENDED;
    sdio_cmd.arg                     = rwFlag ? SDIO_RW_FLAG_Msk : 0x00000000;
    sdio_cmd.arg                    |= fn << SDIO_FN_Pos;
    sdio_cmd.arg                    |= incr_addr ? SDIO_RW_EXT_INCR_ADDR_Msk : 0x00000000;
    sdio_cmd.arg                    |= addr << SDIO_REG_ADDR_Pos;
    sdio_cmd.xfer_mode =
        (rwFlag ? SDMMC_XFER_MODE_DATA_XFER_WR_Msk : SDMMC_XFER_MODE_DATA_XFER_RD_Msk) |
        SDMMC_XFER_MODE_DMA_EN_Msk;
    if (blk_cnt) {
        sdio_cmd.arg                   |= SDIO_RW_EXT_BLK_MODE_Msk | blk_cnt;
        sdio_cmd.xfer_mode             |= SDMMC_XFER_MODE_MULTI_BLK_SEL_Msk;
        pHsd->regs->SDMMC_BLOCKCOUNT_R  = blk_cnt;

    } else {
        sdio_cmd.arg                   |= (blkSize == 512) ? 0 : blkSize;
        pHsd->regs->SDMMC_BLOCKCOUNT_R  = 1;
    }

exit:
    return status;
}
