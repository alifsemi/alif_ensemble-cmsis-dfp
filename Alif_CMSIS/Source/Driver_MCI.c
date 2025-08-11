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
    0, /* cd_state          */
    0, /* cd_event          */
    0, /* wp_state          */
    1, /* vdd               */
    0, /* vdd_1v8           */
    1, /* vccq              */
    0, /* vccq_1v8          */
    0, /* vccq_1v2          */
    1, /* data_width_4      */
    0, /* data_width_8      */
    0, /* data_width_4_ddr  */
    0, /* data_width_8_ddr  */
    1, /* high_speed        */
    0, /* uhs_signaling     */
    0, /* uhs_tuning        */
    0, /* uhs_sdr50         */
    0, /* uhs_sdr104        */
    0, /* uhs_ddr50         */
    0, /* uhs_driver_type_a */
    0, /* uhs_driver_type_c */
    0, /* uhs_driver_type_d */
    0, /* sdio_interrupt    */
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

    p_arm_mci_event_cb(arm_mci_event);
}

/**
 * \fn           sd_reset_cb(void)
 * \brief        Perform SD reset sequence
 * \return       none
 */
#ifdef BOARD_SD_RESET_GPIO_PORT
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);
void sd_reset_cb(void)
{
    int status;

    ARM_DRIVER_GPIO *sd_rst_gpio = &ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

    status = sd_rst_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to toggle sd reset pin\n");
#endif
    }

    sys_busy_loop_us(SDMMC_RESET_DELAY_US);

    status = sd_rst_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to toggle sd reset pin\n");
#endif
    }

    return;
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
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to initialize SD RST GPIO\n");
#endif
    }

    status = sd_rst_gpio->PowerControl(BOARD_SD_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to powered full\n");
#endif
    }

    status = sd_rst_gpio->SetDirection(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to configure\n");
#endif
    }

    status = sd_rst_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (status) {
#ifdef SDMMC_PRINT_ERR
        printf("ERROR: Failed to toggle sd reset pin\n");
#endif
    }

#endif

    p_arm_mci_event_cb    = cb_event;
    sd_param.dev_id       = SDMMC_DEV_ID;
    sd_param.clock_id     = RTE_SDC_CLOCK_SELECT;
    sd_param.bus_width    = RTE_SDC_BUS_WIDTH;
    sd_param.dma_mode     = RTE_SDC_DMA_SELECT;
    sd_param.app_callback = sd_cb;

#ifdef BOARD_SD_RESET_GPIO_PORT
    sd_param.reset_cb     = sd_reset_cb;
#else
    sd_param.reset_cb     = 0;
#endif

    if (p_SD_Driver->disk_initialize(&sd_param)) {
#ifdef SDMMC_PRINT_ERR
        printf("SD initialization failed...\n");
#endif
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
    if (p_SD_Driver->disk_uninitialize(SDMMC_DEV_ID)) {
        printf("SD initialization failed...\n");
        return ARM_DRIVER_ERROR;
    }

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
    switch (state) {
    case ARM_POWER_OFF:
        break;

    case ARM_POWER_LOW:
        break;

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
    switch (voltage & ARM_MCI_POWER_VDD_Msk) {
    case ARM_MCI_POWER_VDD_OFF:
        return ARM_DRIVER_OK;

    case ARM_MCI_POWER_VDD_3V3:
        return ARM_DRIVER_OK;

    default:
        break;
    }
    return ARM_DRIVER_ERROR;
}

/**
 @fn           : int32_t ARM_MCI_ReadCD(void)
 @brief        : Read Card detect
 @parameter    : None
 @return       : execution_status
**/
static int32_t ARM_MCI_ReadCD(void)
{
    return ARM_DRIVER_OK;
}

/**
 @fn           : int32_t ARM_MCI_ReadWP(void)
 @brief        : Read Write Protect
 @parameter    : None
 @return       : execution_status
**/
static int32_t ARM_MCI_ReadWP(void)
{
    return ARM_DRIVER_OK;
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

    if ((cmd == MC_CMD_READ_MULTIPLE_BLOCK) || (cmd == MC_CMD_READ_SINGLE_BLOCK)) {
        p_SD_Driver->disk_read(arg, g_block_count, (volatile uint8_t *) gp_buff);
        while (!dma_done_irq) {
        }
        RTSS_InvalidateDCache_by_Addr(gp_buff, g_block_count * 512);

    } else if ((cmd == MC_CMD_WRITE_MULTIPLE_BLOCK) || (cmd == MC_CMD_WRITE_SINGLE_BLOCK)) {
        RTSS_CleanDCache_by_Addr(gp_buff, g_block_count * 512);
        p_SD_Driver->disk_write(arg, g_block_count, (volatile uint8_t *) gp_buff);
        while (!dma_done_irq) {
        }

    } else {
        hc_cmd.arg          = arg;
        hc_cmd.cmdidx       = cmd;
        hc_cmd.data_present = 0;
        hc_cmd.xfer_mode    = 0;

        hc_send_cmd(pHsd, &hc_cmd);
    }

    *response = hc_get_response1(&Hsd);

    if (flags ==  ARM_MCI_RESPONSE_LONG) {
        *(response + 1) = hc_get_response2(&Hsd);
        *(response + 2) = hc_get_response3(&Hsd);
        *(response + 3) = hc_get_response4(&Hsd);
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
    sd_handle_t *pHsd           = &Hsd;
    uint32_t     unused_sec_arg = 0xff;
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
    switch (control) {
    case ARM_MCI_BUS_SPEED:
        break;

    case ARM_MCI_BUS_SPEED_MODE:
        break;

    case ARM_MCI_BUS_CMD_MODE:
        /* Implement external pull-up control to support MMC cards in open-drain mode */
        /* Default mode is push-pull and is configured in Driver_MCI0.Initialize()    */
        if (arg == ARM_MCI_BUS_CMD_PUSH_PULL) {
            /* Configure external circuit to work in push-pull mode */
        } else if (arg == ARM_MCI_BUS_CMD_OPEN_DRAIN) {
            /* Configure external circuit to work in open-drain mode */
        } else {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        break;

    case ARM_MCI_BUS_DATA_WIDTH:
        switch (arg) {
        case ARM_MCI_BUS_DATA_WIDTH_1:
            return ARM_DRIVER_OK;
            break;
        case ARM_MCI_BUS_DATA_WIDTH_4:
            return ARM_DRIVER_OK;
            break;
        case ARM_MCI_BUS_DATA_WIDTH_8:
            break;
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        break;

    case ARM_MCI_CONTROL_RESET:
        break;

    case ARM_MCI_CONTROL_CLOCK_IDLE:
        break;

    case ARM_MCI_DATA_TIMEOUT:
        return ARM_DRIVER_OK;
        break;

    case ARM_MCI_MONITOR_SDIO_INTERRUPT:
        break;

    case ARM_MCI_CONTROL_READ_WAIT:
        break;

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
    mci_status.command_active = 0;
    if (p_SD_Driver->disk_status() < SD_CARD_STATE_IDLE) {
#ifdef SDMMC_PRINT_WARN
        printf("SD invalid status...\n");
#endif
        return mci_status;
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
