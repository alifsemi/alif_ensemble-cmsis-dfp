/*
 * Copyright (c) 2013-2020 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Project:      MCI (Memory Card Interface) Driver definitions
 */

#ifndef DRIVER_MCI_H_
#define DRIVER_MCI_H_

#include "Driver_Common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MCI API version */
#define ARM_MCI_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/* MCI driver version */
#define ARM_MCI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/****** MCI Card Power *****/
#define ARM_MCI_POWER_VDD_Msk            (0x0FU)
#define ARM_MCI_POWER_VDD_Pos            (0U)
#define ARM_MCI_POWER_VDD_OFF            (0x00U)
#define ARM_MCI_POWER_VDD_3V3            (0x01U)
#define ARM_MCI_POWER_VDD_1V8            (0x02U)
#define ARM_MCI_POWER_VDD_1V2            (0x03U)
#define ARM_MCI_POWER_VCCQ_Msk           (0xF0U)
#define ARM_MCI_POWER_VCCQ_Pos           (4U)
#define ARM_MCI_POWER_VCCQ_OFF           (0x00U << ARM_MCI_POWER_VCCQ_Pos)
#define ARM_MCI_POWER_VCCQ_3V3           (0x01U << ARM_MCI_POWER_VCCQ_Pos)
#define ARM_MCI_POWER_VCCQ_1V8           (0x02U << ARM_MCI_POWER_VCCQ_Pos)
#define ARM_MCI_POWER_VCCQ_1V2           (0x03U << ARM_MCI_POWER_VCCQ_Pos)

/****** MCI Send Command Flags *****/
#define ARM_MCI_RESPONSE_NONE            (0U)
#define ARM_MCI_RESPONSE_SHORT           (1U << 0U)
#define ARM_MCI_RESPONSE_SHORT_BUSY      (2U << 0U)
#define ARM_MCI_RESPONSE_LONG            (3U << 0U)
#define ARM_MCI_RESPONSE_IDX             (1U << 2U)
#define ARM_MCI_RESPONSE_CRC             (1U << 3U)
#define ARM_MCI_WAIT_BUSY                (1U << 8U)
#define ARM_MCI_TRANSFER_DATA            (1U << 9U)
#define ARM_MCI_TRANSFER_SEND            (1U << 10U)
#define ARM_MCI_TRANSFER_READ            (0U << 10U)
#define ARM_MCI_TRANSFER_BLOCK           (1U << 11U)
#define ARM_MCI_TRANSFER_STREAM          (0U << 11U)
#define ARM_MCI_TRANSFER_SDIO_MULTIPLE   (1U << 12U)

/****** MCI Setup Transfer Mode *****/
#define ARM_MCI_TRANSFER_BUS_WIDTH_1     (0U)
#define ARM_MCI_TRANSFER_BUS_WIDTH_4     (1U)
#define ARM_MCI_TRANSFER_BUS_WIDTH_8     (2U)
#define ARM_MCI_TRANSFER_BUS_WIDTH_Msk   (3U)
#define ARM_MCI_TRANSFER_BUS_WIDTH_Pos   (0U)
#define ARM_MCI_TRANSFER_CARD_SD         (0U << 2U)
#define ARM_MCI_TRANSFER_CARD_MMC        (1U << 2U)
#define ARM_MCI_TRANSFER_CARD_SDIO       (2U << 2U)
#define ARM_MCI_TRANSFER_CARD_Msk        (3U << 2U)
#define ARM_MCI_TRANSFER_CARD_Pos        (2U)

/****** MCI Control Codes *****/
#define ARM_MCI_BUS_SPEED                (0x01U)
#define ARM_MCI_BUS_SPEED_MODE           (0x02U)
#define ARM_MCI_BUS_CMD_MODE             (0x03U)
#define ARM_MCI_BUS_DATA_WIDTH           (0x04U)
#define ARM_MCI_CONTROL_RESET            (0x05U)
#define ARM_MCI_CONTROL_CLOCK_IDLE       (0x06U)
#define ARM_MCI_DATA_TIMEOUT             (0x07U)
#define ARM_MCI_MONITOR_SDIO_INTERRUPT   (0x08U)
#define ARM_MCI_CONTROL_READ_WAIT        (0x09U)
#define ARM_MCI_DRIVER_STRENGTH          (0x0AU)

/****** MCI Bus Speed Modes *****/
#define ARM_MCI_BUS_HIGH_SPEED           (1U << 0U)
#define ARM_MCI_BUS_UHS_SDR12            (1U << 1U)
#define ARM_MCI_BUS_UHS_SDR25            (1U << 2U)
#define ARM_MCI_BUS_UHS_SDR50            (1U << 3U)
#define ARM_MCI_BUS_UHS_SDR104           (1U << 4U)
#define ARM_MCI_BUS_UHS_DDR50            (1U << 5U)

/****** MCI Bus Data Width *****/
#define ARM_MCI_BUS_DATA_WIDTH_1         (0U)
#define ARM_MCI_BUS_DATA_WIDTH_4         (1U)
#define ARM_MCI_BUS_DATA_WIDTH_8         (2U)
#define ARM_MCI_BUS_DATA_WIDTH_4_DDR     (3U)
#define ARM_MCI_BUS_DATA_WIDTH_8_DDR     (4U)

/****** MCI Bus Command Mode *****/
#define ARM_MCI_BUS_CMD_PUSH_PULL        (0U)
#define ARM_MCI_BUS_CMD_OPEN_DRAIN       (1U)

/****** MCI Driver Strength *****/
#define ARM_MCI_DRIVER_TYPE_B            (0U)
#define ARM_MCI_DRIVER_TYPE_A            (1U)
#define ARM_MCI_DRIVER_TYPE_C            (2U)
#define ARM_MCI_DRIVER_TYPE_D            (3U)

/****** MCI Events *****/
#define ARM_MCI_EVENT_COMMAND_COMPLETE   (1U << 0U)
#define ARM_MCI_EVENT_TRANSFER_COMPLETE  (1U << 1U)
#define ARM_MCI_EVENT_SDIO_INTERRUPT     (1U << 2U)
#define ARM_MCI_EVENT_CARD_INSERTED      (1U << 3U)
#define ARM_MCI_EVENT_CARD_REMOVED       (1U << 4U)

/****** MMC Commands *****/
#define MC_CMD_GO_IDLE_STATE             0U
#define MC_CMD_ALL_SEND_CID              2U
#define MC_CMD_SET_RELATIVE_ADDR         3U
#define MC_CMD_SET_DSR                   4U
#define MC_CMD_SELECT_CARD               7U
#define MC_CMD_SEND_CSD                  9U
#define MC_CMD_SEND_CID                  10U
#define MC_CMD_STOP_TRANSMISSION         12U
#define MC_CMD_SEND_STATUS               13U
#define MC_CMD_SET_BLOCKLEN              16U
#define MC_CMD_READ_SINGLE_BLOCK         17U
#define MC_CMD_READ_MULTIPLE_BLOCK       18U
#define MC_CMD_WRITE_SINGLE_BLOCK        24U
#define MC_CMD_WRITE_MULTIPLE_BLOCK      25U
#define MC_CMD_PROGRAM_CSD               27U
#define MC_CMD_SET_WRITE_PROT            28U
#define MC_CMD_CLR_WRITE_PROT            29U
#define MC_CMD_SEND_WRITE_PROT           30U
#define MC_CMD_ERASE_GROUP_START         35U
#define MC_CMD_ERASE_GROUP_END           36U
#define MC_CMD_ERASE                     38U
#define MC_CMD_APP_CMD                   55U
#define MC_CMD_GEN_CMD                   56U

/**
  \brief MCI Status
*/
typedef struct _ARM_MCI_STATUS {
  uint32_t command_active   : 1;  ///< Command active flag
  uint32_t command_timeout  : 1;  ///< Command timeout flag
  uint32_t command_error    : 1;  ///< Command error flag
  uint32_t transfer_active  : 1;  ///< Transfer active flag
  uint32_t transfer_timeout : 1;  ///< Transfer timeout flag
  uint32_t transfer_error   : 1;  ///< Transfer error flag
  uint32_t sdio_interrupt   : 1;  ///< SDIO interrupt flag
  uint32_t ccs              : 1;  ///< CCS flag
  uint32_t reserved         : 24; ///< Reserved
} ARM_MCI_STATUS;

/**
  \brief MCI Capabilities
*/
typedef struct _ARM_MCI_CAPABILITIES {
  uint32_t cd_state          : 1;  ///< Card Detect State available
  uint32_t cd_event          : 1;  ///< Card Detect Event available
  uint32_t wp_state          : 1;  ///< Write Protect State available
  uint32_t vdd               : 1;  ///< Specifies VDD support
  uint32_t vdd_1v8           : 1;  ///< Specifies 1.8V VDD support
  uint32_t vccq              : 1;  ///< Specifies VCCQ support
  uint32_t vccq_1v8          : 1;  ///< Specifies 1.8V VCCQ support
  uint32_t vccq_1v2          : 1;  ///< Specifies 1.2V VCCQ support
  uint32_t data_width_4      : 1;  ///< Data Width 4 supported
  uint32_t data_width_8      : 1;  ///< Data Width 8 supported
  uint32_t data_width_4_ddr  : 1;  ///< Data Width 4 DDR supported
  uint32_t data_width_8_ddr  : 1;  ///< Data Width 8 DDR supported
  uint32_t high_speed        : 1;  ///< High Speed supported
  uint32_t uhs_signaling     : 1;  ///< UHS Signaling supported
  uint32_t uhs_tuning        : 1;  ///< UHS Tuning supported
  uint32_t uhs_sdr50         : 1;  ///< UHS SDR50 supported
  uint32_t uhs_sdr104        : 1;  ///< UHS SDR104 supported
  uint32_t uhs_ddr50         : 1;  ///< UHS DDR50 supported
  uint32_t uhs_driver_type_a : 1;  ///< UHS Driver Type A supported
  uint32_t uhs_driver_type_c : 1;  ///< UHS Driver Type C supported
  uint32_t uhs_driver_type_d : 1;  ///< UHS Driver Type D supported
  uint32_t sdio_interrupt    : 1;  ///< SDIO Interrupt supported
  uint32_t read_wait         : 1;  ///< Read Wait supported
  uint32_t suspend_resume    : 1;  ///< Suspend/Resume supported
  uint32_t mmc_interrupt     : 1;  ///< MMC Interrupt supported
  uint32_t mmc_boot          : 1;  ///< MMC Boot supported
  uint32_t rst_n             : 1;  ///< Reset (RST_n) supported
  uint32_t ccs               : 1;  ///< CCS supported
  uint32_t ccs_timeout       : 1;  ///< CCS Timeout supported
  uint32_t reserved          : 1;  ///< Reserved
} ARM_MCI_CAPABILITIES;

/**
  \brief MCI Signal Event Callback
*/
typedef void (*ARM_MCI_SignalEvent_t) (uint32_t event);  ///< Pointer to \ref ARM_MCI_SignalEvent : Signal MCI Card Event.

/**
\brief  Access structure of the MCI Driver.
*/
typedef struct _ARM_DRIVER_MCI {
  ARM_DRIVER_VERSION   (*GetVersion)     (void);                           ///< Pointer to \ref ARM_MCI_GetVersion : Get driver version.
  ARM_MCI_CAPABILITIES (*GetCapabilities)(void);                           ///< Pointer to \ref ARM_MCI_GetCapabilities : Get driver capabilities.
  int32_t              (*Initialize)     (ARM_MCI_SignalEvent_t cb_event); ///< Pointer to \ref ARM_MCI_Initialize : Initialize MCI Interface.
  int32_t              (*Uninitialize)   (void);                           ///< Pointer to \ref ARM_MCI_Uninitialize : De-initialize MCI Interface.
  int32_t              (*PowerControl)   (ARM_POWER_STATE state);          ///< Pointer to \ref ARM_MCI_PowerControl : Control MCI Interface Power.
  int32_t              (*CardPower)      (uint32_t voltage);               ///< Pointer to \ref ARM_MCI_CardPower : Set card power supply voltage.
  int32_t              (*ReadCD)         (void);                           ///< Pointer to \ref ARM_MCI_ReadCD : Read Card Detect (CD) state.
  int32_t              (*ReadWP)         (void);                           ///< Pointer to \ref ARM_MCI_ReadWP : Read Write Protect (WP) state.
  int32_t              (*SendCommand)    (uint32_t cmd,
                                          uint32_t arg,
                                          uint32_t flags,
                                          uint32_t *response);             ///< Pointer to \ref ARM_MCI_SendCommand : Send Command to card and get the response.
  int32_t              (*SetupTransfer)  (uint8_t *data,
                                          uint32_t block_count,
                                          uint32_t block_size,
                                          uint32_t mode);                  ///< Pointer to \ref ARM_MCI_SetupTransfer : Setup data transfer operation.
  int32_t              (*AbortTransfer)  (void);                           ///< Pointer to \ref ARM_MCI_AbortTransfer : Abort current data transfer.
  int32_t              (*Control)        (uint32_t control, uint32_t arg); ///< Pointer to \ref ARM_MCI_Control : Control MCI Interface.
  ARM_MCI_STATUS       (*GetStatus)      (void);                           ///< Pointer to \ref ARM_MCI_GetStatus : Get MCI status.
} const ARM_DRIVER_MCI;

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_MCI_H_ */
