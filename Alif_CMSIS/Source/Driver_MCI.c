/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     Driver_MCI.c
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V1.0.0
 * @date     24-Jan-2025
 * @brief    MCI Driver
 * @bug      None.
 ******************************************************************************/
#include "RTE_Components.h"
#include "stdio.h"
#include "string.h"
#include "Driver_MCI.h"
#include "sd.h"
#include "board_config.h"
#include "Driver_IO.h"

#if defined(RTE_FileSystem_Drive_MC_0)
#include "fs_memory_card.h"
#include "fs_mc.h"
#endif

#if defined(RTE_Drivers_MCI)

#define ARM_MCI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {ARM_MCI_API_VERSION, ARM_MCI_DRV_VERSION};

const diskio_t    *p_SD_Driver                = &SD_Driver;
extern sd_handle_t Hsd;
volatile uint32_t  dma_done_irq;
static uint32_t    g_block_count;
static uint8_t    *gp_buff;
static ARM_MCI_SignalEvent_t p_arm_mci_event_cb;

/* Driver Capabilities */
static const ARM_MCI_CAPABILITIES DriverCapabilities = {
    1, /* cd_state          */
    1, /* cd_event          */
    0, /* wp_state          */
    1, /* vdd               */
    1, /* vdd_1v8           */
    1, /* vccq              */
    1, /* vccq_1v8          */
    0, /* vccq_1v2          */
    1, /* data_width_4      */
    1, /* data_width_8      */
    0, /* data_width_4_ddr  */
    0, /* data_width_8_ddr  */
    1, /* high_speed        */
    1, /* uhs_signaling     */
    0, /* uhs_tuning        */
    1, /* uhs_sdr50         */
    1, /* uhs_sdr104        */
    1, /* uhs_ddr50         */
    0, /* uhs_driver_type_a */
    0, /* uhs_driver_type_c */
    0, /* uhs_driver_type_d */
    1, /* sdio_interrupt    */
    0, /* read_wait         */
    0, /* suspend_resume    */
    1, /* mmc_interrupt     */
    0, /* mmc_boot          */
    0, /* rst_n             */
    0, /* ccs               */
    0, /* ccs_timeout       */
    0  /* Reserved          */
};

/**
  \fn           sd_cb(uint32_t status)
  \brief        SD interrupt callback
  \param[in]    uint32_t status
  \return       none
*/
void sd_cb(uint16_t cmd_status, uint16_t xfer_status)
{
    uint32_t arm_mci_event = 0;

    if (xfer_status) {
        dma_done_irq = SDMMC_INTR_TC_Msk;
    }

    if (cmd_status & SDMMC_INTR_CC_Msk) {
        arm_mci_event = ARM_MCI_EVENT_COMMAND_COMPLETE;
    }
    if (xfer_status & SDMMC_INTR_TC_Msk) {
        arm_mci_event |= ARM_MCI_EVENT_TRANSFER_COMPLETE;
    }

    if (p_arm_mci_event_cb) {
        p_arm_mci_event_cb(arm_mci_event);
    }
}

/**
 * \fn           sd_pwr_cb(uint8_t power_on)
 * \brief        SD power callback
 * \param[in]    power_on: 0=off, 1=on
 * \return       none
 */
#ifdef BOARD_SD_RESET_GPIO_PORT
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);
void sd_pwr_cb(uint8_t power_on)
{
    int status;

    ARM_DRIVER_GPIO *sd_pwr_gpio = &ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

    if (power_on) {
        status = sd_pwr_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        if (status) {
            SD_LOG_ERR("Failed to turn on SD power pin");
        }
    } else {
        status = sd_pwr_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        if (status) {
            SD_LOG_ERR("Failed to turn off SD power pin");
        }
        sys_busy_loop_us(SDMMC_RESET_DELAY_US);
    }

    return;
}
#endif

/**
 * \fn           sd_card_det_cb(void)
 * \brief        Check SD card detect GPIO
 * \return       1 if card present, 0 if not
 */
#ifdef BOARD_SD_CARD_DETECT_GPIO_PORT
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_SD_CARD_DETECT_GPIO_PORT);
int sd_card_det_cb(void)
{
    uint32_t pin_state;
    ARM_DRIVER_GPIO *cd_gpio = &ARM_Driver_GPIO_(BOARD_SD_CARD_DETECT_GPIO_PORT);

    cd_gpio->GetValue(BOARD_SD_CARD_DETECT_GPIO_PIN, &pin_state);

    SD_LOG_DBG("SD card detect pin state: %d", pin_state);

    /* Card detect: pin state 0 = card present (active low) */
    return (pin_state == 0) ? 1 : 0;
}
#endif

/**
 * \fn           sd_vsel_cb(uint8_t voltage)
 * \brief        Set SD VSEL GPIO for voltage selection
 * \param[in]    voltage: 0 = 3.3V, 1 = 1.8V
 * \return       none
 */
#ifdef BOARD_SD_VSEL_GPIO_PORT
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_SD_VSEL_GPIO_PORT);
void sd_vsel_cb(uint8_t voltage)
{
    int status;
    ARM_DRIVER_GPIO *vsel_gpio = &ARM_Driver_GPIO_(BOARD_SD_VSEL_GPIO_PORT);

    /* VSEL: 0 = 3.3V, 1 = 1.8V */
    status = vsel_gpio->SetValue(BOARD_SD_VSEL_GPIO_PIN, voltage ? GPIO_PIN_OUTPUT_STATE_HIGH : GPIO_PIN_OUTPUT_STATE_LOW);
    if (status) {
        SD_LOG_ERR("Failed to set SD VSEL pin to %d", voltage);
    }

    SD_LOG_DBG("SD VSEL set to %s", voltage ? "1.8V" : "3.3V");
}
#endif

/**
 @fn       ARM_DRIVER_VERSION ARM_MCI_GetVersion(void)
 @brief    ARM_MCI VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ARM_MCI_GetVersion(void)
{
    return DriverVersion;
}

/**
 @fn       ARM_MCI_CAPABILITIES ARM_MCI_GetCapabilities(void)
 @brief    ARM_MCI_GET CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_MCI_CAPABILITIES ARM_MCI_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           : int32_t ARM_MCI_Initialize(ARM_MCI_SignalEvent_t cb_event)
 @brief        : Initialize the MCI Interface
 @parameter[1] : cb_event : Pointer to \ref ARM_MCI_SignalEvent_t
 @return       : execution_status
**/
static int32_t ARM_MCI_Initialize(ARM_MCI_SignalEvent_t cb_event)
{
    int status;
    sd_param_t sd_param;

#ifdef BOARD_SD_RESET_GPIO_PORT
    ARM_DRIVER_GPIO *sd_rst_gpio = &ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

    status = sd_rst_gpio->Initialize(BOARD_SD_RESET_GPIO_PIN, NULL);
    if (status) {
        SD_LOG_ERR("Failed to initialize SD RST GPIO");
    }

    status = sd_rst_gpio->PowerControl(BOARD_SD_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (status) {
        SD_LOG_ERR("Failed to power SD RST GPIO");
    }

    status = sd_rst_gpio->SetDirection(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (status) {
        SD_LOG_ERR("Failed to configure SD RST GPIO direction");
    }

    status = sd_rst_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (status) {
        SD_LOG_ERR("Failed to set SD reset pin high");
    }

#endif

#ifdef BOARD_SD_CARD_DETECT_GPIO_PORT
    ARM_DRIVER_GPIO *cd_gpio = &ARM_Driver_GPIO_(BOARD_SD_CARD_DETECT_GPIO_PORT);

    status = cd_gpio->Initialize(BOARD_SD_CARD_DETECT_GPIO_PIN, NULL);
    if (status) {
        SD_LOG_ERR("Failed to initialize SD CD GPIO");
    }

    status = cd_gpio->PowerControl(BOARD_SD_CARD_DETECT_GPIO_PIN, ARM_POWER_FULL);
    if (status) {
        SD_LOG_ERR("Failed to power SD CD GPIO");
    }

    status = cd_gpio->SetDirection(BOARD_SD_CARD_DETECT_GPIO_PIN, GPIO_PIN_DIRECTION_INPUT);
    if (status) {
        SD_LOG_ERR("Failed to configure SD CD GPIO direction");
    }
#endif

#ifdef BOARD_SD_VSEL_GPIO_PORT
    ARM_DRIVER_GPIO *vsel_gpio = &ARM_Driver_GPIO_(BOARD_SD_VSEL_GPIO_PORT);

    status = vsel_gpio->Initialize(BOARD_SD_VSEL_GPIO_PIN, NULL);
    if (status) {
        SD_LOG_ERR("Failed to initialize SD VSEL GPIO");
    }

    status = vsel_gpio->PowerControl(BOARD_SD_VSEL_GPIO_PIN, ARM_POWER_FULL);
    if (status) {
        SD_LOG_ERR("Failed to power SD VSEL GPIO");
    }

    status = vsel_gpio->SetDirection(BOARD_SD_VSEL_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (status) {
        SD_LOG_ERR("Failed to configure SD VSEL GPIO direction");
    }

    /* Default to 3.3V */
    status = vsel_gpio->SetValue(BOARD_SD_VSEL_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (status) {
        SD_LOG_ERR("Failed to set SD VSEL to 3.3V");
    }
#endif

    p_arm_mci_event_cb    = cb_event;
    sd_param.dev_id       = SDMMC_DEV_ID;
    sd_param.clock_freq   = RTE_SDC_CLOCK_SELECT;
    sd_param.bus_width    = RTE_SDC_BUS_WIDTH;
    sd_param.dma_mode     = RTE_SDC_DMA_SELECT;
    sd_param.app_callback = sd_cb;

#ifdef BOARD_SD_RESET_GPIO_PORT
    sd_param.pwr_cb     = sd_pwr_cb;
#else
    sd_param.pwr_cb     = 0;
#endif

#ifdef BOARD_SD_CARD_DETECT_GPIO_PORT
    sd_param.card_det_cb = sd_card_det_cb;
#else
    sd_param.card_det_cb = 0;
#endif

#ifdef BOARD_SD_VSEL_GPIO_PORT
    sd_param.vsel_cb = sd_vsel_cb;
#else
    sd_param.vsel_cb = 0;
#endif

    if (p_SD_Driver->disk_initialize(&sd_param)) {
        SD_LOG_ERR("SD initialization failed");
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
 @fn           : int32_t ARM_MCI_Uninitialize(void)
 @brief        : Un-Initialize the MCI Interface
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ARM_MCI_Uninitialize(void)
{
    if (p_SD_Driver->disk_uninitialize(SDMMC_DEV_ID) != SD_DRV_STATUS_OK) {
        SD_LOG_ERR("SD uninitialize failed");
        return ARM_DRIVER_ERROR;
    }

    p_arm_mci_event_cb = NULL;

    return ARM_DRIVER_OK;
}

/**
 @fn           : int32_t ARM_MCI_PowerControl(ARM_POWER_STATE status)
 @brief        : Control MCI Interface power
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ARM_MCI_PowerControl(ARM_POWER_STATE state)
{
    sd_handle_t *pHsd = &Hsd;

    switch (state) {
    case ARM_POWER_OFF:
        sdhc_power_cycle(pHsd);
        return ARM_DRIVER_OK;

    case ARM_POWER_LOW:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
        return ARM_DRIVER_OK;
        break;
    }
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
 @fn           : int32_t ARM_MCI_CardPower(uint32_t volate)
 @brief        : Control MCI Interface power
 @parameter    : input voltage
 @return       : execution_status
**/
static int32_t ARM_MCI_CardPower(uint32_t voltage)
{
    sd_handle_t *pHsd = &Hsd;
    sdmmc_io_t  io_param;

    switch (voltage & ARM_MCI_POWER_VDD_Msk) {
    case ARM_MCI_POWER_VDD_OFF:
        io_param.sdmmc_power = SDMMC_POWER_OFF;
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_PWR) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        return ARM_DRIVER_OK;

    case ARM_MCI_POWER_VDD_3V3:
        io_param.sdmmc_vol = SDMMC_VOL_3P3V;
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_VOL) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        io_param.sdmmc_power = SDMMC_POWER_ON;
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_PWR) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        return ARM_DRIVER_OK;

    case ARM_MCI_POWER_VDD_1V8:
        io_param.sdmmc_vol = SDMMC_VOL_1P8V;
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_VOL) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        io_param.sdmmc_power = SDMMC_POWER_ON;
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_PWR) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        return ARM_DRIVER_OK;

    default:
        break;
    }
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
 @fn           : int32_t ARM_MCI_ReadCD(void)
 @brief        : Read Card detect
 @parameter    : None
 @return       : execution_status
**/
static int32_t ARM_MCI_ReadCD(void)
{
    sd_handle_t *pHsd = &Hsd;

#ifdef BOARD_SD_CARD_DETECT_GPIO_PORT
    return sdhc_card_present(pHsd, sd_card_det_cb);
#else
    return sdhc_card_present(pHsd, NULL);
#endif
}

/**
 @fn           : int32_t ARM_MCI_ReadWP(void)
 @brief        : Read Write Protect
 @parameter    : None
 @return       : execution_status
**/
static int32_t ARM_MCI_ReadWP(void)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
 @fn           : int32_t ARM_MCI_SendCommand(uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t
*response)
 @brief        : Send Command to Card
 @parameter    : cmd, argument, flags and pointer to store response.
 @return       : execution_status
**/
static int32_t ARM_MCI_SendCommand(uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response)
{
    sd_handle_t *pHsd = &Hsd;
    sd_cmd_t     hc_cmd;

    if (response == NULL) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((cmd == MC_CMD_READ_MULTIPLE_BLOCK) || (cmd == MC_CMD_READ_SINGLE_BLOCK)) {
        dma_done_irq = 0;
        if (p_SD_Driver->disk_read(arg, g_block_count, (volatile uint8_t *) gp_buff) != SD_DRV_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        while (!dma_done_irq) {
        }
        RTSS_InvalidateDCache_by_Addr(gp_buff, g_block_count * 512);

    } else if ((cmd == MC_CMD_WRITE_MULTIPLE_BLOCK) || (cmd == MC_CMD_WRITE_SINGLE_BLOCK)) {
        dma_done_irq = 0;
        RTSS_CleanDCache_by_Addr(gp_buff, g_block_count * 512);
        if (p_SD_Driver->disk_write(arg, g_block_count, (volatile uint8_t *) gp_buff) != SD_DRV_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        while (!dma_done_irq) {
        }

    } else {
        hc_cmd.arg          = arg;
        hc_cmd.cmdidx       = cmd;
        hc_cmd.data_present = 0;
        hc_cmd.xfer_mode    = 0;
        hc_cmd.retries      = 0;

        if (sdhc_send_cmd(pHsd, &hc_cmd) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
    }

    *response = sdhc_get_response1(&Hsd);

    if ((flags & ARM_MCI_RESPONSE_LONG) == ARM_MCI_RESPONSE_LONG) {
        *(response + 1) = sdhc_get_response2(&Hsd);
        *(response + 2) = sdhc_get_response3(&Hsd);
        *(response + 3) = sdhc_get_response4(&Hsd);
    }

    return ARM_DRIVER_OK;
}

/**
 @fn           : int32_t ARM_MCI_SetupTransfer(uint8_t  *data, uint32_t block_count, uint32_t
block_size, uint32_t mode)
 @brief        : Prepare DMA for transfer
 @parameter    : read/write data buffer, block count, block size, and mode
 @return       : execution_status
**/
static int32_t ARM_MCI_SetupTransfer(uint8_t *data, uint32_t block_count, uint32_t block_size,
                                     uint32_t mode)
{
    if ((data == NULL) || (block_count == 0U) || (block_size == 0U)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    g_block_count = block_count;
    gp_buff       = data;

    return ARM_DRIVER_OK;
}

/**
 @fn           : int32_t ARM_MCI_AbortTransfer(void)
 @brief        : Stop the on-going transfer
 @parameter    : None
 @return       : execution_status
**/
static int32_t ARM_MCI_AbortTransfer(void)
{
    sd_handle_t *pHsd = &Hsd;

    if (sdhc_reset(pHsd, (uint8_t)(SDHC_SW_RST_DAT_Msk | SDHC_SW_RST_CMD_Msk)) != SDHC_STATUS_OK) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
 @fn           : int32_t ARM_MCI_Control(uint32_t control, uint32_t arg)
 @brief        : Control the MCI Features
 @parameter    : Control and Value
 @return       : execution_status
**/
static int32_t ARM_MCI_Control(uint32_t control, uint32_t arg)
{
    sd_handle_t *pHsd = &Hsd;
    sdmmc_io_t   io_param;

    switch (control) {
    case ARM_MCI_BUS_SPEED:
        switch (arg) {
        case SDMMC_CLK_400KHZ:
            io_param.sdmmc_clock = SDMMC_CLK_400KHZ;
            break;
        case SDMMC_CLK_12P5MHZ:
            io_param.sdmmc_clock = SDMMC_CLK_12P5MHZ;
            break;
        case SDMMC_CLK_25MHZ:
            io_param.sdmmc_clock = SDMMC_CLK_25MHZ;
            break;
        case SDMMC_CLK_50MHZ:
            io_param.sdmmc_clock = SDMMC_CLK_50MHZ;
            break;
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_CLK) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        return ARM_DRIVER_OK;

    case ARM_MCI_BUS_SPEED_MODE:
        switch (arg) {
        case ARM_MCI_BUS_HIGH_SPEED:
            io_param.sdmmc_clock = SDMMC_CLK_50MHZ;
            break;
        case ARM_MCI_BUS_UHS_SDR50:
            io_param.sdmmc_clock = SDMMC_CLK_50MHZ;
            break;
        case ARM_MCI_BUS_UHS_SDR104:
            io_param.sdmmc_clock = SDMMC_CLK_50MHZ;
            break;
        case ARM_MCI_BUS_UHS_DDR50:
            io_param.sdmmc_clock = SDMMC_CLK_50MHZ;
            break;
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_CLK) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        return ARM_DRIVER_OK;

    case ARM_MCI_BUS_CMD_MODE:
        if (arg == ARM_MCI_BUS_CMD_PUSH_PULL) {
            return ARM_DRIVER_OK;
        } else if (arg == ARM_MCI_BUS_CMD_OPEN_DRAIN) {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        } else {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        break;

    case ARM_MCI_BUS_DATA_WIDTH:
        switch (arg) {
        case ARM_MCI_BUS_DATA_WIDTH_1:
            io_param.sdmmc_bus_width = SDMMC_BUS_WIDTH_1BIT;
            break;
        case ARM_MCI_BUS_DATA_WIDTH_4:
            io_param.sdmmc_bus_width = SDMMC_BUS_WIDTH_4BIT;
            break;
        case ARM_MCI_BUS_DATA_WIDTH_8:
            io_param.sdmmc_bus_width = SDMMC_BUS_WIDTH_8BIT;
            break;
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        if (sdhc_set_io(&io_param, SDMMC_SET_IO_BUS_WIDTH) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        return ARM_DRIVER_OK;

    case ARM_MCI_CONTROL_RESET:
        if (sdhc_reset(pHsd, (uint8_t)SDHC_SW_RST_ALL_Msk) != SDHC_STATUS_OK) {
            return ARM_DRIVER_ERROR;
        }
        return ARM_DRIVER_OK;

    case ARM_MCI_CONTROL_CLOCK_IDLE:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_MCI_DATA_TIMEOUT:
        return ARM_DRIVER_OK;

    case ARM_MCI_MONITOR_SDIO_INTERRUPT:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_MCI_CONTROL_READ_WAIT:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_MCI_DRIVER_STRENGTH:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 @fn           : ARM_MCI_STATUS ARM_MCI_GetStatus(void)
 @brief        : Gets the driver status
 @parameter    : ARM_MCI_STATUS
 @return       : execution_status
**/
static ARM_MCI_STATUS ARM_MCI_GetStatus(void)
{
    ARM_MCI_STATUS mci_status;
    memset(&mci_status, 0, sizeof(mci_status));

    SD_CARD_STATE card_state = p_SD_Driver->disk_status();

    if (card_state == SD_CARD_STATE_DATA || card_state == SD_CARD_STATE_RCV ||
        card_state == SD_CARD_STATE_PRG) {
        mci_status.transfer_active = 1;
    }

    if (card_state < SD_CARD_STATE_IDLE) {
        mci_status.command_error = 1;
    }

    return mci_status;
}

/**
 @fn           : ARM_MCI_SignalEvent(uint32_t event)
 @brief        : Signal the events
 @parameter    : event
 @return       : None
**/
void ARM_MCI_SignalEvent(uint32_t event)
{
}

// End MCI Interface

extern ARM_DRIVER_MCI Driver_MCI0;
ARM_DRIVER_MCI        Driver_MCI0 = {
    ARM_MCI_GetVersion,
    ARM_MCI_GetCapabilities,
    ARM_MCI_Initialize,
    ARM_MCI_Uninitialize,
    ARM_MCI_PowerControl,
    ARM_MCI_CardPower,
    ARM_MCI_ReadCD,
    ARM_MCI_ReadWP,
    ARM_MCI_SendCommand,
    ARM_MCI_SetupTransfer,
    ARM_MCI_AbortTransfer,
    ARM_MCI_Control,
    ARM_MCI_GetStatus
};

#endif  // RTE_DRIVER_MCI
