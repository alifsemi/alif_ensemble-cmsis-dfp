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
#include "sd_types.h"
#include "sd_host.h"
#include "sys_ctrl_sd.h"
#include "string.h"

extern sd_handle_t  Hsd;
static uint64_t adma_desc_tbl[32] __attribute__((section("sd_dma_buf")))
__attribute__((aligned(512)));
static volatile uint16_t nis, eis, cc;
static volatile uint16_t eis_pending;

#ifdef SDMMC_IRQ_MODE
void SDMMC_IRQHandler(void)
{
    SDMMC_Type *regs = Hsd.regs;
    uint16_t eis_tmp, nis_tmp;

    /* Read and clear — accumulate bits that arrive between the initial
     * read and the clear so TC (which fires almost simultaneously with CC) is not lost.
     */
    eis = nis = 0;

    do {
        eis_tmp = regs->SDMMC_ERROR_INT_STAT_R;
        if (eis_tmp) {
            regs->SDMMC_ERROR_INT_STAT_R = eis_tmp;
            eis |= eis_tmp;
        }

        nis_tmp = regs->SDMMC_NORMAL_INT_STAT_R;
        if (nis_tmp) {
            regs->SDMMC_NORMAL_INT_STAT_R = nis_tmp;
            nis |= nis_tmp;
        }
    } while (eis_tmp || (nis_tmp & ~SDHC_INTR_ERR_Msk));

    /* Ignore command timeout if CC is also set (false positive) */
    if ((eis & SDHC_INTR_ERR_CT_Msk) && (nis & SDHC_INTR_CC_Msk)) {
        eis &= ~SDHC_INTR_ERR_CT_Msk;
    }

    /* Ignore data timeout if TC is also set (false positive) */
    if ((eis & SDHC_INTR_ERR_DT_Msk) && (nis & SDHC_INTR_TC_Msk)) {
        eis &= ~SDHC_INTR_ERR_DT_Msk;
    }

    if (eis) {
        SD_LOG_DBG("ISR EIS: 0x%04x NIS: 0x%04x", eis, nis);
        eis_pending |= eis;
    }

    if (nis & SDHC_INTR_CC_Msk) {
        cc = SDHC_INTR_CC_Msk;
    }

    if (Hsd.sd_param.app_callback) {
        uint16_t xfer_done = (nis & SDHC_INTR_TC_Msk) ? SDHC_INTR_TC_Msk : 0;
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
  \fn           static void sdhc_set_tout(sd_handle_t *pHsd, uint8_t ToutVal)
  \brief        software reset of the controller
  \param[in]    Global SD Handle pointer
  \param[in]    Timeout Value
  \return       none
  */
static void sdhc_set_tout(sd_handle_t *pHsd, uint8_t tout)
{
    /* Set the timeout value */
    pHsd->regs->SDMMC_TOUT_CTRL_R = tout;
    return;
}

/**
  \fn           SDHC_STATUS sdhc_set_bus_power(sd_handle_t *pHsd, uint8_t bus_power) {
  \brief        Set required sd bus power supply
  \param[in]    Global SD Handle pointer
  \param[in]    required bus voltage equivalent register value as per \
                  host controller data sheet
  \return       Host controller driver status
  */
static SDHC_STATUS sdhc_set_bus_power(sd_handle_t *pHsd, uint8_t bus_power)
{
    SDMMC_Type *regs = pHsd->regs;
    uint32_t status, timeout_cnt = SDHC_POWER_CYCLE_TIMEOUT;

    /* Disable the power supply */
    regs->SDMMC_PWR_CTRL_R &= ~SDHC_PC_BUS_PWR_VDD1_Msk;

    if (bus_power == SDHC_PC_BUS_PWR_OFF) {
        /* Call power callback to turn off power completely */
        if (pHsd->sd_param.pwr_cb != NULL) {
            pHsd->sd_param.pwr_cb(0);
        }
        return SDHC_STATUS_OK;
    }

    if (bus_power == SDHC_PC_BUS_VSEL_1V8_Msk) {
        /* 1.8V voltage switch sequence per SD spec:
         * - Wait 10ms for bus gating
         * - Disable clock
         * - Call VSEL callback to set GPIO to 1.8V
         * - Set voltage select to 1.8V
         * - Wait 10ms (spec minimum 5ms)
         * - Enable power
         * - Set signaling enable
         * - Enable clock
         * - Check CMD line level
         */
        sys_busy_loop_us(SDHC_VOLTAGE_SWITCH_DELAY_US);
        HC_CLOCK_DISABLE(pHsd);

        /* Call VSEL GPIO callback if available (voltage: 1 = 1.8V) */
        if (pHsd->sd_param.vsel_cb) {
            pHsd->sd_param.vsel_cb(1);
        }

        regs->SDMMC_PWR_CTRL_R = SDHC_PC_BUS_VSEL_1V8_Msk;
        sys_busy_loop_us(SDHC_VOLTAGE_SWITCH_DELAY_US);
        regs->SDMMC_PWR_CTRL_R |= SDHC_PC_BUS_PWR_VDD1_Msk;
        regs->SDMMC_HOST_CTRL2_R |= SDHC_HOST_CTRL2_SIGNALING_EN_Msk;
        HC_CLOCK_ENABLE(pHsd);

        /* check CMD line status after voltage switch */
        do {
            status = regs->SDMMC_PSTATE_REG & SDHC_CMD_LINE_LVL_UP_Msk;
            sys_busy_loop_us(1);
        } while (!status && timeout_cnt--);

        return status ? SDHC_STATUS_OK : SDHC_STATUS_ERR;
    } else if (bus_power == SDHC_PC_BUS_VSEL_3V3_Msk) {
        /* Explicit 3.3V voltage switch sequence */
        /* Call VSEL GPIO callback if available (voltage: 0 = 3.3V) */
        if (pHsd->sd_param.vsel_cb) {
            pHsd->sd_param.vsel_cb(0);
        }

        regs->SDMMC_PWR_CTRL_R = SDHC_PC_BUS_VSEL_3V3_Msk;
        sys_busy_loop_us(SDHC_VOLTAGE_SWITCH_DELAY_US);
        regs->SDMMC_PWR_CTRL_R |= SDHC_PC_BUS_PWR_VDD1_Msk;
        regs->SDMMC_HOST_CTRL2_R &= ~SDHC_HOST_CTRL2_SIGNALING_EN_Msk;
        /* Clock will be enabled via sdhc_set_clk_freq */

        /* Call power callback during 3.3V power-on sequence */
        if (pHsd->sd_param.pwr_cb != NULL) {
            pHsd->sd_param.pwr_cb(1);
        }

        return SDHC_STATUS_OK;
    } else {
        /* SDMMC_POWER_ON: preserve currently selected VSEL bits */
        sys_busy_loop_us(SDHC_VOLTAGE_SWITCH_DELAY_US);
        regs->SDMMC_PWR_CTRL_R |= SDHC_PC_BUS_PWR_VDD1_Msk;

        /* Call power callback if not already powered */
        if (pHsd->sd_param.pwr_cb != NULL) {
            pHsd->sd_param.pwr_cb(1);
        }

        return SDHC_STATUS_OK;
    }
}

/**
  \fn           SDHC_STATUS sdhc_check_bus_idle(sd_handle_t *pHsd)
  \brief        Check the CMD line is idle or nor
  \param[in]    Global SD Handle pointer
  \return       Host controller driver status
  */
static SDHC_STATUS sdhc_check_bus_idle(sd_handle_t *pHsd)
{
    SDMMC_Type *regs = pHsd->regs;
    uint32_t pstate, cmdinhibit, datinhibit, timeout = SDHC_BUS_IDLE_TIMEOUT;

    do {

        pstate     = regs->SDMMC_PSTATE_REG;
        cmdinhibit = pstate & SDHC_CMD_INHIBIT_Msk;
        datinhibit = pstate & SDHC_DAT_INHIBIT_Msk;

        if (!timeout--) {
            return SDHC_STATUS_BUSY;
        }

        /* Small delay to allow hardware to clear inhibit bits */
        if (timeout % 1000 == 0) {
            sys_busy_loop_us(1);
        }

    } while (cmdinhibit || (pHsd->sd_cmd.data_present && datinhibit));

    return SDHC_STATUS_OK;
}

/**
  \fn           SDHC_STATUS sdhc_set_clk_freq(sd_handle_t *pHsd, uint32_t clk_freq)
  \brief        Sets required clock
  \param[in]    Global SD Handle pointer
  \param[in]    required bus clock frequency in Hz
  \return       Host controller driver status
  */
static SDHC_STATUS sdhc_set_clk_freq(sd_handle_t *pHsd, uint32_t clk_freq)
{
    SDMMC_Type *regs = pHsd->regs;
    uint16_t reg;
    uint16_t div;

    /* Check bus is idle before changing clock */
    if (sdhc_check_bus_idle(pHsd) != SDHC_STATUS_OK) {
        return SDHC_STATUS_BUSY;
    }

    /* Disable clock */
    HC_CLOCK_DISABLE(pHsd);

    if (clk_freq == 0U) {
        return SDHC_STATUS_OK;
    }

    /* Calculate divider from frequency
     * SD Clock = Base Clock / (2 * div)
     * Base clock read from capabilities register, default 50MHz
     * Divider is 10-bit (SDHC_MAX_DIVIDER max), split across FREQ_SEL and UPPER_FREQ_SEL
     */
    uint32_t base_clk_mhz = (regs->SDMMC_CAPABILITIES1_R & SDHC_BASE_CLK_FREQ_MASK) >>
                            SDHC_BASE_CLK_FREQ_POS;
    if (!base_clk_mhz) {
        base_clk_mhz = SDHC_DEFAULT_BASE_CLK_MHZ;
    }
    uint32_t base_clk_hz = base_clk_mhz * SDHC_MHZ_TO_HZ;

    if (clk_freq >= base_clk_hz) {
        div = 0;
    } else {
        div = (base_clk_hz + (2 * clk_freq) - 1) / (2 * clk_freq);
        if (div > SDHC_MAX_DIVIDER) {
            div = SDHC_MAX_DIVIDER;
        }
    }

    /* Build clock control value from the split divider fields. */
    reg  = (uint16_t)((div & SDHC_DIVIDER_LOW_MASK) << SDHC_FREQ_SEL_Pos);
    reg |= (uint16_t)(((div >> 8) & SDHC_DIVIDER_HIGH_MASK) << SDHC_UPPER_FREQ_SEL_Pos);
    reg |= SDHC_PLL_EN_Msk | SDHC_INTERNAL_CLK_EN_Msk;

    regs->SDMMC_CLK_CTRL_R = reg;

    uint32_t timeout = SDHC_CLOCK_STABLE_TIMEOUT;
    do {
        reg = regs->SDMMC_CLK_CTRL_R;
        sys_busy_loop_us(1);
        timeout--;
    } while ((reg & SDHC_INTERNAL_CLK_STABLE_Msk) == 0 && timeout > 0);

    if (timeout == 0) {
        /* Clock not stable - return error */
        return SDHC_STATUS_ERR;
    }

    /* Enable SD clock output only after internal clock is stable */
    reg |= SDHC_CLK_EN_Msk;
    regs->SDMMC_CLK_CTRL_R = reg;

    /* Set or clear HS mode bit in HOST_CTRL1 based on clock frequency */
    if (clk_freq > SDHC_HS_MODE_THRESHOLD_HZ) {
        regs->SDMMC_HOST_CTRL1_R |= SDHC_HOST_CTRL1_HIGH_SPEED_MODE_EN;
    } else {
        regs->SDMMC_HOST_CTRL1_R &= ~SDHC_HOST_CTRL1_HIGH_SPEED_MODE_EN;
    }

    /* Update UHS Mode Select in HOST_CTRL2 based on clock frequency and voltage */
    uint16_t ctrl2 = regs->SDMMC_HOST_CTRL2_R;
    uint8_t  uhs_mode;

    if (ctrl2 & SDHC_HOST_CTRL2_SIGNALING_EN_Msk) {
        /* 1.8V signaling enabled - UHS-I modes */
        if (clk_freq > SDHC_UHS_SDR50_THRESHOLD_HZ) {
            uhs_mode = SDHC_UHS_MODESEL_SDR104;
        } else if (clk_freq > SDHC_HS_MODE_THRESHOLD_HZ) {
            uhs_mode = SDHC_UHS_MODESEL_SDR50;
        } else {
            uhs_mode = SDHC_UHS_MODESEL_SDR25;
        }
    } else {
        /* 3.3V signaling - legacy modes */
        if (clk_freq > SDHC_HS_MODE_THRESHOLD_HZ) {
            uhs_mode = SDHC_UHS_MODESEL_SDR25;
        } else {
            uhs_mode = SDHC_UHS_MODESEL_SDR12;
        }
    }

    ctrl2 &= ~SDHC_HOST_CTRL2_UHS_MODESEL_Msk;
    ctrl2 |= (uint16_t)(uhs_mode << SDHC_HOST_CTRL2_UHS_MODESEL_Pos);
    regs->SDMMC_HOST_CTRL2_R = ctrl2;

    SD_LOG_DBG("clk: %luHz (%s) div=%u",
               (unsigned long)clk_freq,
               (clk_freq > SDHC_UHS_SDR50_THRESHOLD_HZ) ? "SDR104" :
               (clk_freq > SDHC_HS_MODE_THRESHOLD_HZ)  ? "SDR50/HS" :
               (clk_freq > SDHC_INIT_CLOCK_FREQ_HZ)    ? "DS" : "DS-400K",
               div);

    /* Add delay after enabling clock for stability */
    sys_busy_loop_us(SDHC_POST_CLOCK_ENABLE_DELAY_US);

    return SDHC_STATUS_OK;
}

/**
  \fn           SDHC_STATUS sdhc_set_bus_width(sd_handle_t *pHsd, uint8_t buswidth)
  \brief        Set bus width in Host Controller register only
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    buswidth - number of Data lines for data transfer
  \return       Host controller driver status
  */
static SDHC_STATUS sdhc_set_bus_width(sd_handle_t *pHsd, uint8_t buswidth)
{

    uint8_t regs;

    regs = pHsd->regs->SDMMC_HOST_CTRL1_R;

    /* Clear existing bus width bits */
    regs &= ~(SDMMC_1_BIT_WIDTH_Msk | SDMMC_4_BIT_WIDTH_Msk | SDMMC_8_BIT_WIDTH_Msk);

    if (buswidth == SDMMC_1_BIT_MODE) {
        regs = regs | SDMMC_1_BIT_WIDTH_Msk;
    } else if (buswidth == SDMMC_4_BIT_MODE) {
        regs = regs | SDMMC_4_BIT_WIDTH_Msk;
    } else {
        regs = regs | SDMMC_8_BIT_WIDTH_Msk;
    }

    pHsd->regs->SDMMC_HOST_CTRL1_R = regs;

    return SDHC_STATUS_OK;
}

/**
  \fn           void sdhc_config_default_intr(sd_handle_t *pHsd)
  \brief        Configure SDHC interrupt with default settings
  \param[in]    pHsd - Global SD Handle pointer
  \return       none
  */
static void sdhc_config_default_intr(sd_handle_t *pHsd)
{
    SDMMC_Type *regs = pHsd->regs;

    /* Enable all interrupt status except card interrupt initially */
    regs->SDMMC_NORMAL_INT_STAT_EN_R =
        SDHC_NORM_INTR_ALL_Msk & (~SDHC_INTR_CARD_Msk);

    regs->SDMMC_ERROR_INT_STAT_EN_R    = SDHC_ERROR_INTR_ALL_Msk;

    /* Disable all interrupt signals by default. */
    regs->SDMMC_NORMAL_INT_SIGNAL_EN_R = SDHC_SIGNAL_DISABLE;

    regs->SDMMC_ERROR_INT_SIGNAL_EN_R  = SDHC_SIGNAL_DISABLE;
}

/**
  \fn           SDHC_STATUS sdhc_get_capabilities(sd_handle_t *pHsd, uint32_t *phcaps)
  \brief        Get the Host Controller Capabilities
  \param[in]    Global sd Handle pointer
  \param[in]    Capabilities pointer
  \return       Host controller driver status
  */
static SDHC_STATUS sdhc_get_capabilities(sd_handle_t *pHsd, uint32_t *phcaps)
{

    /* Get the card Relative Addr */
    *phcaps = pHsd->regs->SDMMC_CAPABILITIES1_R;

    return SDHC_STATUS_OK;
}

/**
 * \fn           sdhc_set_io
 * \brief        SDHC set IO common api for power and clock
 * \param[in]    p_sdmmc_io_param - SD IO parameters
 * \param[in]    set_io_cmd - IO command type
 * \return       SDHC status
 */
SDHC_STATUS sdhc_set_io(sdmmc_io_t *p_sdmmc_io_param, SDMMC_SET_IO_CMD set_io_cmd)
{
    sd_handle_t *pHsd   = &Hsd;
    SDHC_STATUS  status = SDHC_STATUS_OK;


    switch (set_io_cmd) {
    case SDMMC_SET_IO_CLK:
        switch (p_sdmmc_io_param->sdmmc_clock) {
        case SDMMC_CLK_DISABLE:
            status = sdhc_set_clk_freq(pHsd, 0);
            break;
        case SDMMC_CLK_400KHZ:
            status = sdhc_set_clk_freq(pHsd, SDHC_INIT_CLOCK_FREQ_HZ);
            break;
        case SDMMC_CLK_12P5MHZ:
            status = sdhc_set_clk_freq(pHsd, 12500000U);
            break;
        case SDMMC_CLK_25MHZ:
            status = sdhc_set_clk_freq(pHsd, SDMMC_CLK_25_MHZ);
            break;
        case SDMMC_CLK_50MHZ:
            status = sdhc_set_clk_freq(pHsd, SDMMC_CLK_50_MHZ);
            break;
        default:
            status = sdhc_set_clk_freq(pHsd, pHsd->sd_param.clock_freq);
            break;
        }
        break;

    case SDMMC_SET_IO_VOL:
        if (p_sdmmc_io_param->sdmmc_vol == SDMMC_VOL_1P8V) {
            status = sdhc_set_bus_power(pHsd, SDHC_PC_BUS_VSEL_1V8_Msk);
        } else if (p_sdmmc_io_param->sdmmc_vol == SDMMC_VOL_3P3V) {
            status = sdhc_set_bus_power(pHsd, SDHC_PC_BUS_VSEL_3V3_Msk);
        }
        break;

    case SDMMC_SET_IO_PWR:
        if (p_sdmmc_io_param->sdmmc_power == SDMMC_POWER_OFF) {
            status = sdhc_set_bus_power(pHsd, SDHC_PC_BUS_PWR_OFF);
        } else {
            status = sdhc_set_bus_power(pHsd, SDMMC_POWER_ON);
        }
        break;

    case SDMMC_SET_IO_BUS_WIDTH:
        status = sdhc_set_bus_width(pHsd, p_sdmmc_io_param->sdmmc_bus_width);
        break;

    default:
        return SDHC_STATUS_ERR;
    }

    SD_LOG_DBG("set_io: cmd=%d status=%d", set_io_cmd, status);

    return status;
}

/**
  \fn         SDHC_STATUS sdhc_send_cmd(sd_handle_t *pHsd, sd_cmd_t *pCmd)
  \brief       Configure Host controller to send SD command
  \param[in]   pHsd - Global SD Handle pointer
  \param[in]   pcmd - command info pointer
  \return      Host controller driver API status.
  */
SDHC_STATUS sdhc_send_cmd(sd_handle_t *pHsd, sd_cmd_t *pCmd)
{
    SDMMC_Type *regs = pHsd->regs;
    uint16_t cmd, timeout_cnt = SDMMC_MAX_TIMEOUT_16;
    uint8_t  retry_cnt        = pCmd->retries;

    cmd = (pCmd->cmdidx << SDHC_CMD_IDX_Pos) | (pCmd->rsp_type);

    if (pCmd->data_present) {
        cmd |= SDMMC_CMD_R_DATA_PRES_SEL_Msk;
    }

RETRY:
    timeout_cnt = SDMMC_MAX_TIMEOUT_16;

    /* Drop any stale error flag from a previous command. Do NOT issue a
     * software reset here: SW_RST_DAT clears the ADMA system address
     * register that was already programmed for this transfer, causing
     * the data phase to fetch a bogus descriptor and hang silently.
     */
    eis = eis_pending = false;

#ifndef SDMMC_IRQ_MODE
    /* Polling mode: clear leftover status. In IRQ mode the ISR is the
     * sole owner of the status registers; clearing them here could eat
     * a latched Transfer Complete that has not been serviced yet.
     */
    regs->SDMMC_ERROR_INT_STAT_R  = SDHC_ERROR_INTR_ALL_Msk;
    regs->SDMMC_NORMAL_INT_STAT_R = SDHC_NORM_INTR_ALL_Msk;
#endif
    cc                                  = false;

    if (sdhc_check_bus_idle(pHsd) != SDHC_STATUS_OK) {
        return SDHC_STATUS_BUSY;
    }

    regs->SDMMC_XFER_MODE_R = 0;

    if (pCmd->data_present) {
        regs->SDMMC_BLOCKSIZE_R  = pCmd->data.blk_size;
        regs->SDMMC_BLOCKCOUNT_R = pCmd->data.blk_cnt;
        regs->SDMMC_XFER_MODE_R  = pCmd->xfer_mode;
    }

    regs->SDMMC_ARGUMENT_R = pCmd->arg;
    regs->SDMMC_CMD_R = cmd;

#ifndef SDMMC_IRQ_MODE
    cc = regs->SDMMC_NORMAL_INT_STAT_R & SDHC_INTR_CC_Msk;
#endif

    while (timeout_cnt-- && (!cc)) {
#ifndef SDMMC_IRQ_MODE
        cc = regs->SDMMC_NORMAL_INT_STAT_R & SDHC_INTR_CC_Msk;
#else
        /* In IRQ mode the ISR is the sole owner of the interrupt status
         * register and sets cc on Command Complete. Touching the status
         * register here races the ISR and can steal CC (lost callback) or
         * perturb a coalesced Transfer Complete. Just wait on cc.
         */
#endif

    }

    SD_LOG_DBG("CMD: 0x%04" PRIx16 ", ARG: 0x%08" PRIx32 " XFER: 0x%04" PRIx16
           " RSP01: 0x%08" PRIx32 " PSTATE: 0x%08" PRIx32 " cc:%" PRId16 "",
           cmd,
           pCmd->arg,
           pCmd->xfer_mode,
           regs->SDMMC_RESP01_R,
           regs->SDMMC_PSTATE_REG,
           cc);

    if ((timeout_cnt == SDMMC_MAX_TIMEOUT_16) || eis_pending) {

        SD_LOG_DBG("CMD%d failed: %s, retries left: %d",
                   pCmd->cmdidx,
                   eis_pending ? "EIS" : "timeout",
                   retry_cnt);

        if (!retry_cnt--) {
            return SDHC_STATUS_ERR;
        }

        /* Recover the controller, then re-program the DMA address:
         * the DAT-line reset clears the ADMA system address register,
         * so a data command must never be re-issued without it.
         */
        sdhc_reset(pHsd, (uint8_t)(SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk));

        if (pCmd->data_present) {
            sdhc_xfer_dma_setup(pHsd, &pCmd->data);
        }
        goto RETRY;
    }

    return SDHC_STATUS_OK;
}

/**
  \fn           SDHC_STATUS sdhc_reset(sd_handle_t *pHsd, uint8_t reset_val)
  \brief        software reset of the controller
  \param[in]    Global SD Handle pointer
  \param[in]    Reset Value
  \return       Host controller driver status
  */
SDHC_STATUS sdhc_reset(sd_handle_t *pHsd, uint8_t reset_val)
{
    SDMMC_Type *regs = pHsd->regs;
    uint8_t  curr_reset_val    = 0;
    uint16_t timeout           = SDMMC_MAX_TIMEOUT_16;

    regs->SDMMC_SW_RST_R = reset_val;

    do {
        curr_reset_val = regs->SDMMC_SW_RST_R;
        sys_busy_loop_us(100);
    } while ((curr_reset_val & reset_val) && timeout--);

    return SDHC_STATUS_OK;
}

/**
  \fn           SDHC_STATUS sdhc_power_cycle(sd_handle_t *pHsd)
  \brief        software reset and power cycle of the controller
  \param[in]    Global SD Handle pointer
  \return       Host controller driver status
  */
void sdhc_power_cycle(sd_handle_t *pHsd)
{
    SDMMC_Type *regs = pHsd->regs;

    regs->SDMMC_PWR_CTRL_R &= ~SDHC_PC_BUS_PWR_VDD1_Msk; // disable vdd1
    sys_busy_loop_us(SDHC_POWER_CYCLE_DELAY_US);
    regs->SDMMC_PWR_CTRL_R |= SDHC_PC_BUS_PWR_VDD1_Msk;  // enable vdd1
    sys_busy_loop_us(SDHC_POWER_CYCLE_DELAY_US);

    regs->SDMMC_CLK_CTRL_R        |= SDHC_CLK_EN_Msk;

    regs->SDMMC_NORMAL_INT_STAT_R  = SDHC_NORM_INTR_ALL_Msk;
    regs->SDMMC_ERROR_INT_STAT_R   = SDHC_ERROR_INTR_ALL_Msk;

    sdhc_reset(pHsd, SDHC_SW_RST_CMD_Msk | SDHC_SW_RST_DAT_Msk);

    return;
}

/**
  \fn           static SDHC_STATUS sdhc_config_dma(sd_handle_t *pHsd,uint8_t dmaMask)
  \brief        Host Controller DMA configuration
  \param[in]    Global SD Handle pointer
  \param[in]    dmaMask - Host ctrl 1 register value
  \return       Host controller driver status
  */
static SDHC_STATUS sdhc_config_dma(sd_handle_t *pHsd, uint8_t dmaMask)
{

    /* Host Version 4 Param */
    pHsd->regs->SDMMC_HOST_CTRL2_R = SDHC_HOST_CTRL2_ASYNC_INT_EN_Msk |
                                     SDHC_HOST_CTRL2_VER4_EN_Msk;

    pHsd->regs->SDMMC_HOST_CTRL1_R = dmaMask;

    return SDHC_STATUS_OK;
}

/**
  \fn           void sdhc_enable_irq(sd_handle_t *pHsd, uint16_t mask)
  \brief        Enable SDHC IRQ signals and status
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    mask - Interrupt mask to enable
  \return       none
  */
void sdhc_enable_irq(sd_handle_t *pHsd, uint16_t mask)
{
    SDMMC_Type *regs = pHsd->regs;
    regs->SDMMC_NORMAL_INT_SIGNAL_EN_R = mask;
    regs->SDMMC_NORMAL_INT_STAT_EN_R   = mask;
    regs->SDMMC_ERROR_INT_STAT_EN_R    = SDHC_ERROR_INTR_ALL_Msk;
}

/**
  \fn           static void sdhc_disable_irq(sd_handle_t *pHsd, uint16_t mask)
  \brief        Disable SDHC IRQ signals
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    mask - Interrupt mask to disable
  \return       none
  */
static void sdhc_disable_irq(sd_handle_t *pHsd, uint16_t mask)
{
    pHsd->regs->SDMMC_NORMAL_INT_SIGNAL_EN_R &= ~mask;
}

/**
  \fn           void sdhc_set_emmc_ctrl(sd_handle_t *pHsd, uint8_t value)
  \brief        Set eMMC control register
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    value - Value to set
  \return       none
  */
void sdhc_set_emmc_ctrl(sd_handle_t *pHsd, uint8_t value)
{
    pHsd->regs->SDMMC_EMMC_CTRL_R = value;
}

/**
  \fn           uint32_t sdhc_get_blk_size(sd_handle_t *pHsd)
  \brief        Get block size register value
  \param[in]    pHsd - Global SD Handle pointer
  \return       Block size value
  */
uint32_t sdhc_get_blk_size(sd_handle_t *pHsd)
{
    return pHsd->regs->SDMMC_BLOCKSIZE_R;
}

/**
  \fn           void sdhc_set_led(sd_handle_t *pHsd, bool enable)
  \brief        Set LED control in host control register
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    enable - true to enable LED, false to disable
  \return       none
  */
void sdhc_set_led(sd_handle_t *pHsd, bool enable)
{
    if (enable) {
        pHsd->regs->SDMMC_HOST_CTRL1_R |= SDHC_HOST_CTRL1_LED_ON;
    } else {
        pHsd->regs->SDMMC_HOST_CTRL1_R &= ~SDHC_HOST_CTRL1_LED_ON;
    }
}

/**
  \fn           SD_host_init
  \brief        initialize host controller
  \param[in]    SD Global Handle pointer
  \return       sd driver status
  */
SD_DRV_STATUS sdhc_init(sd_handle_t *pHsd, sd_param_t *p_sd_param)
{
    uint8_t powerlevel;

    /* clear the global SD Handle */
    memset(&Hsd, 0, sizeof(Hsd));

    /* Enable SDMMC Clock */
    enable_sd_periph_clk();

    pHsd->regs  = SDMMC;
    pHsd->state = SD_CARD_STATE_INIT;
    SDMMC_Type *regs = pHsd->regs;

    /* copy default device parameters */
    memcpy((void *) &pHsd->sd_param.dev_id,
           (const void *) &p_sd_param->dev_id,
           sizeof(pHsd->sd_param));

    if (pHsd->sd_param.pwr_cb) {
        pHsd->sd_param.pwr_cb(1);
    }

    /* Get the Host Controller version */
    pHsd->hc_version = regs->SDMMC_HOST_CNTRL_VERS_R &
                      SDHC_HC_VERSION_REG_Msk;

    /* Get the Host Controller Capabilities */
    sdhc_get_capabilities(pHsd, &pHsd->hc_caps);

    /* Disable the SD Voltage supply */
    sdhc_set_bus_power(pHsd, SDMMC_POWER_OFF);

    /* Soft reset Host controller cmd and data lines */
    sdhc_reset(pHsd, (uint8_t) (SDHC_SW_RST_ALL_Msk));

    /* Get the host voltage capability */
    if ((pHsd->hc_caps & SDHC_HOST_SD_CAP_VOLT_3V3_Msk) != 0U) {
        powerlevel = SDHC_PC_BUS_VSEL_3V3_Msk;
    } else if ((pHsd->hc_caps & SDHC_HOST_SD_CAP_VOLT_3V0_Msk) != 0U) {
        powerlevel = SDHC_PC_BUS_VSEL_3V0_Msk;
    } else if ((pHsd->hc_caps & SDHC_HOST_SD_CAP_VOLT_1V8_Msk) != 0U) {
        powerlevel = SDHC_PC_BUS_VSEL_1V8_Msk;
    } else {
        powerlevel = 0U;
    }

    sdhc_set_bus_power(pHsd, (uint8_t)(powerlevel));
    sdhc_set_tout(pHsd, SDHC_MAX_TIMEOUT_VALUE);

    sdhc_config_default_intr(pHsd);

#ifdef SDMMC_IRQ_MODE
    NVIC_ClearPendingIRQ(SDHC_WAKEUP_IRQ_NUM);
    NVIC_SetPriority(SDHC_WAKEUP_IRQ_NUM, RTE_SDC_WAKEUP_IRQ_PRI);
    NVIC_EnableIRQ(SDHC_WAKEUP_IRQ_NUM);

    NVIC_ClearPendingIRQ(SDHC_IRQ_NUM);
    NVIC_SetPriority(SDHC_IRQ_NUM, RTE_SDC_IRQ_PRI);
    NVIC_EnableIRQ(SDHC_IRQ_NUM);

    /* Enable IRQ-mode transfer and card-detect signals. */
    regs->SDMMC_NORMAL_INT_SIGNAL_EN_R =
        SDHC_INTR_TC_Msk | SDHC_INTR_CC_Msk | SDHC_INTR_DMA_Msk |
        SDHC_INTR_CARD_INSRT_Msk | SDHC_INTR_CARD_REM_Msk;
    regs->SDMMC_ERROR_INT_SIGNAL_EN_R = SDHC_ERROR_INTR_ALL_Msk;

    regs->SDMMC_WUP_CTRL_R =
        SDHC_WKUP_CARD_IRQ_Msk | SDHC_WKUP_CARD_INSRT_Msk | SDHC_WKUP_CARD_REM_Msk;
#endif

    if (pHsd->sd_param.dma_mode == SDHC_HOST_CTRL1_SDMA_MODE) {
        sdhc_config_dma(pHsd,
                      (uint8_t) (SDHC_HOST_CTRL1_SDMA_MODE |
                                SDHC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    } else if (pHsd->sd_param.dma_mode == SDHC_HOST_CTRL1_ADMA2_MODE) {
        sdhc_config_dma(pHsd,
                      (uint8_t) (SDHC_HOST_CTRL1_ADMA32_MODE_Msk |
                                 SDHC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    } else if (pHsd->sd_param.dma_mode == SDHC_HOST_CTRL1_ADMA3_MODE) {
        sdhc_config_dma(pHsd,
                      (uint8_t) (SDHC_HOST_CTRL1_ADMA32_MODE_Msk |
                                 SDHC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    } else {
        sdhc_config_dma(pHsd,
                      (uint8_t) (SDHC_HOST_CTRL1_ADMA32_MODE_Msk |
                                 SDHC_HOST_CTRL1_DMA_SEL_1BIT_MODE));
    }

    /* Default clock settings to 400KHz */
    pHsd->sd_card.cardtype = SDMMC_CARD_SDHC;
    pHsd->sd_card.busspeed = SDMMC_CLK_400_KHZ;

    sdhc_set_clk_freq(pHsd, SDHC_INIT_CLOCK_FREQ_HZ);

    return SD_DRV_STATUS_OK;
}

/**
  \fn           SDHC_STATUS sdhc_set_blk_size(sd_handle_t *pHsd, uint32_t blk_size)
  \brief        Set the Block Size in Host Controller register only
  \param[in]    Global sd Handle pointer
  \param[in]    Block size
  \return       Host controller driver status
  */
SDHC_STATUS sdhc_set_blk_size(sd_handle_t *pHsd, uint32_t blk_size)
{

    pHsd->regs->SDMMC_BLOCKSIZE_R = blk_size;

    return SDHC_STATUS_OK;
}

/**
  \fn           SDHC_STATUS sdhc_get_response1(sd_handle_t *pHsd)
  \brief        Get the Card Response1
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t sdhc_get_response1(sd_handle_t *pHsd)
{

    /* Get the card Response1 */
    return pHsd->regs->SDMMC_RESP01_R;
}

/**
  \fn           SDHC_STATUS sdhc_get_response2(sd_handle_t *pHsd)
  \brief        Get the Card Response2
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t sdhc_get_response2(sd_handle_t *pHsd)
{
    /* Get the card Response2 */
    return pHsd->regs->SDMMC_RESP23_R;
}

/**
  \fn           SDHC_STATUS sdhc_get_response3(sd_handle_t *pHsd)
  \brief        Get the Card Response3
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t sdhc_get_response3(sd_handle_t *pHsd)
{
    /* Get the card Response3 */
    return pHsd->regs->SDMMC_RESP45_R;
}

/**
  \fn           SDHC_STATUS sdhc_get_response4(sd_handle_t *pHsd)
  \brief        Get the Card Response1
  \param[in]    Global sd Handle pointer
  \return       response
  */
uint32_t sdhc_get_response4(sd_handle_t *pHsd)
{
    /* Get the card Response4 */
    return pHsd->regs->SDMMC_RESP67_R;
}

/**
  \fn           void sdhc_set_block_count(sd_handle_t *pHsd, uint32_t blk_cnt)
  \brief        Set the Block Count register
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    blk_cnt - Block count value
  \return       none
  */
void sdhc_set_block_count(sd_handle_t *pHsd, uint32_t blk_cnt)
{
    pHsd->regs->SDMMC_BLOCKCOUNT_R = blk_cnt;
}

/**
  \fn           SDHC_STATUS sdhc_xfer_dma_setup(sd_handle_t *pHsd, sd_data_t *data)
  \brief        Setup DMA for data transfer
  \param[in]    pHsd - Global SD Handle pointer
  \param[in]    data - Data transfer parameters
  \return       Host controller driver status
  */
SDHC_STATUS sdhc_xfer_dma_setup(sd_handle_t *pHsd, sd_data_t *data)
{
    SDMMC_Type *regs = pHsd->regs;

    /* Configure DMA descriptor */
    if (pHsd->sd_param.dma_mode == SDHC_HOST_CTRL1_ADMA2_MODE) {

        uint32_t desc_num  = 0;
        uint32_t remaining = data->blk_cnt * data->blk_size;
        uint32_t offset    = 0;
        uint32_t chunk;

        while (remaining > 0 && desc_num < SDHC_ADMA2_MAX_DESC) {
            chunk = (remaining > SDHC_ADMA2_DESC_MAX_LEN) ? SDHC_ADMA2_DESC_MAX_LEN : remaining;

            adma_desc_tbl[desc_num] = ((uint64_t)(SDHC_ADMA2_DESC_TRAN | SDHC_ADMA2_DESC_VALID)) |
                                      ((uint64_t)(chunk & 0xFFFFU) << ADMA2_DESC_LEN_SHIFT) |
                                      ((uint64_t)LocalToGlobal((const void *)(data->buffer + offset)) << ADMA2_DESC_ADDR_SHIFT);

            remaining -= chunk;
            offset += chunk;
            desc_num++;
        }

        if (desc_num == 0) {
            return SDHC_STATUS_ERR;
        }

        adma_desc_tbl[desc_num - 1] |= SDHC_ADMA2_DESC_END;

        for (uint32_t idx = 0; idx < desc_num; idx++) {
            SD_LOG_DBG("ADMA2 desc[%" PRIu32 "]: 0x%08" PRIx32 " 0x%08" PRIx32,
                         idx, ((uint32_t)((adma_desc_tbl[idx] >> ADMA2_DESC_LEN_SHIFT) &
						(ADMA2_DESC_LEN_MASK >> ADMA2_DESC_LEN_SHIFT)) << 16U) |
                         (uint32_t)(adma_desc_tbl[idx] & ADMA2_DESC_ATTR_MASK),
                         (uint32_t)((adma_desc_tbl[idx] >> ADMA2_DESC_ADDR_SHIFT) & (ADMA2_DESC_ADDR_MASK >> ADMA2_DESC_ADDR_SHIFT)));
        }

        RTSS_CleanDCache_by_Addr(&adma_desc_tbl[0], sizeof(adma_desc_tbl));
        regs->SDMMC_ADMA_SA_LOW_R = (uint32_t) LocalToGlobal(&adma_desc_tbl[0]);
    } else {
        regs->SDMMC_ADMA_SA_LOW_R =
            (uint32_t) LocalToGlobal((const void *) data->buffer);
    }

    return SDHC_STATUS_OK;
}

/**
  \fn           SDHC_STATUS sdhc_check_xfer_done(sd_handle_t *pHsd, uint32_t timeout_cnt)
  \brief        Check for transfer complete
  \param[in]    Global sd Handle pointer
  \param[in]    timeout Count
  \return       Host controller driver status
  */
SDHC_STATUS sdhc_check_xfer_done(sd_handle_t *pHsd, uint32_t timeout_cnt)
{
    SDMMC_Type *regs = pHsd->regs;

    do {
        if (eis_pending) {
            SD_LOG_DBG("xfer_done: error pending 0x%04x", eis_pending);
            sdhc_set_led(pHsd, false);
            return SDHC_STATUS_ERR;
        }

#ifndef SDMMC_IRQ_MODE
        nis = regs->SDMMC_NORMAL_INT_STAT_R;
#endif
        if ((nis & NORMAL_INT_STAT_XFER_COMPLETE_Msk) &&
            !(regs->SDMMC_PSTATE_REG & XFER_ACTIVE_Msk)) {
            break;
        }

        sys_busy_loop_us(1);
    } while (--timeout_cnt);

    sdhc_set_led(pHsd, false);
    SD_LOG_DBG("PSTATE:0x%08x AUTO_CMD_STAT:0x%04x EIS:0x%02x",
           regs->SDMMC_PSTATE_REG,
           regs->SDMMC_AUTO_CMD_STAT_R,
           (uint8_t)regs->SDMMC_ERROR_INT_STAT_R);

    return timeout_cnt ? SDHC_STATUS_OK : SDHC_STATUS_ERR;
}

/**
  \fn          int sdhc_card_present(struct _sd_handle_t *pHsd, int (*card_det_cb)(void))
  \brief       Check if SD card is present using GPIO callback or PSTATE register
  \param[in]   pHsd: SD host handle
  \param[in]   card_det_cb: Optional GPIO card detect callback
  \return      1 if card present, 0 if not
*/
int sdhc_card_present(struct _sd_handle_t *pHsd, int (*card_det_cb)(void))
{
    /* Use GPIO callback if provided */
    if (card_det_cb) {
        return card_det_cb();
    }

    /* Fall back to PSTATE register card detect bit */
    uint32_t pstate = pHsd->regs->SDMMC_PSTATE_REG;
    return (pstate & SDHC_CARD_INSRT_Msk) ? 1 : 0;
}
