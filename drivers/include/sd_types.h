/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     sd_types.h
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     15-June-2026
 * @brief    Shared SD/SDMMC types, constants, and logging macros.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef _SD_TYPES_H_
#define _SD_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "sys_utils.h"

/**
 * @brief  Host controller driver status enum definition
 */
typedef enum _SDHC_STATUS {
    SDHC_STATUS_OK,
    SDHC_STATUS_ERR,
    SDHC_STATUS_INV_STATE,
    SDHC_STATUS_BUSY
} SDHC_STATUS;

/* SD Driver Logging Configuration */
#define SD_LOG_LEVEL 2  /* 0=off, 1=err, 2=inf, 3=wrn, 4=dbg */

#if SD_LOG_LEVEL > 0
#include <stdio.h>
#include <inttypes.h>
#endif

/* Logging macros - formatted (with va_args) */
#if SD_LOG_LEVEL >= 1
#define SD_LOG_ERR(fmt, ...) printf("[SD ERR] " fmt "\n", ##__VA_ARGS__)
#else
#define SD_LOG_ERR(fmt, ...) ((void)0)
#endif

#if SD_LOG_LEVEL >= 2
#define SD_LOG_INF(fmt, ...) printf("[SD INF] " fmt "\n", ##__VA_ARGS__)
#else
#define SD_LOG_INF(fmt, ...) ((void)0)
#endif

#if SD_LOG_LEVEL >= 3
#define SD_LOG_WRN(fmt, ...) printf("[SD WRN] " fmt "\n", ##__VA_ARGS__)
#else
#define SD_LOG_WRN(fmt, ...) ((void)0)
#endif

#if SD_LOG_LEVEL >= 4
#define SD_LOG_DBG(fmt, ...) printf("[SD DBG] " fmt "\n", ##__VA_ARGS__)
#else
#define SD_LOG_DBG(fmt, ...) ((void)0)
#endif

#define SDMMC_IRQ_MODE

#define SDMMC_CACHED_NUM_BLK      8U

/**
 * @brief sd set io commands
 * flags used to control clocl, vol, power, bus width
 */
typedef enum _SET_IO_CMD {
    SDMMC_SET_IO_CLK,
    SDMMC_SET_IO_VOL,
    SDMMC_SET_IO_PWR,
    SDMMC_SET_IO_BUS_WIDTH,
} SDMMC_SET_IO_CMD;

/* SD Clk freq */
#define SDMMC_CLK_400_KHZ                 400000U    /*!< 400KHz */
#define SDMMC_CLK_25_MHZ                  25000000U  /*!< 25MHz  */
#define SDMMC_CLK_50_MHZ                  50000000U  /*!< 50MHz  */
#define SDMMC_CLK_100_MHZ                 100000000U /*!< 100MHz */
#define SDMMC_BASE_CLK                    SD_CLK_100_MHZ

/* Supported Data Bus */
#define SDMMC_1_BIT_MODE                  0x0U
#define SDMMC_4_BIT_MODE                  0x1U
#define SDMMC_8_BIT_MODE                  0x2U
#define SDMMC_1_BIT_WIDTH_Msk             0x1U  /*!< Bus Width 1 */
#define SDMMC_4_BIT_WIDTH_Msk             0x2U  /*!< Bus Width 4 */
#define SDMMC_8_BIT_WIDTH_Msk             0x20U /*!< Bus Width 8 */

/* Card Types */
#define SDMMC_CARD_SDHC                   2U /*!< SDHC, SDXC Ver 2    */

/* CMD_R */
#define SDMMC_CMD_R_DATA_PRES_SEL_Pos     5U
#define SDMMC_CMD_R_DATA_PRES_SEL_Msk     (1U << SDMMC_CMD_R_DATA_PRES_SEL_Pos)

/* Blk Size */
#define SDMMC_BLK_SIZE_512_Msk            0x0200U     /*!< Blk Size 512 */

/* Time out constant */
#define SDMMC_MAX_TIMEOUT_32              0xFFFFFFFFU
#define SDMMC_MAX_TIMEOUT_16              0xFFFFU

/* Retry counts — per-command, set in sd_cmd_t.retries */
#define SDMMC_CMD_RETRIES  0   /* Non-data commands: no retry */
#define SDMMC_DATA_RETRIES 2   /* Data commands (CMD17/18/24/25) */

/**
 * @brief  SDIO operating condition structure
 */
typedef struct _sdio_opcond_t {
    uint8_t isInitialized;  /*!< indicates Card is initialized              */
    uint8_t func_number;    /*!< Number of function available in the card   */
    uint8_t memory_present; /*!< Memory Present Flag                        */
    uint8_t s18a;           /*!< low voltage support flag                   */
    uint8_t ocr;            /*!< operating voltage                          */
} sdio_opcond_t;

/**
 * @brief  SDIO card information structure
 */
typedef struct _sdio_t {
    sdio_opcond_t sdio_opcd; /*!< sdio card information */
} sdio_t;

/**
 * @brief  SD driver status enum definition
 */
typedef enum _SD_DRV_STATUS {
    SD_DRV_STATUS_OK,
    SD_DRV_STATUS_ERR,
    SD_DRV_STATUS_HOST_INIT_ERR,
    SD_DRV_STATUS_CARD_INIT_ERR,
    SD_DRV_STATUS_RD_ERR,
    SD_DRV_STATUS_WR_ERR,
    SD_DRV_STATUS_TIMEOUT_ERR
} SD_DRV_STATUS;

/**
 * @brief  SD Card status enum definition
 */
typedef enum _SD_CARD_STATE {
    SD_CARD_STATE_INIT = -1,
    SD_CARD_STATE_IDLE,
    SD_CARD_STATE_READY,
    SD_CARD_STATE_IDENT,
    SD_CARD_STATE_STBY,
    SD_CARD_STATE_TRAN,
    SD_CARD_STATE_DATA,
    SD_CARD_STATE_RCV,
    SD_CARD_STATE_PRG,
    SD_CARD_STATE_DIS,
    SD_CARD_STATE_RESV
} SD_CARD_STATE;

/**
 * @brief MMC High Speed timing enum
 */
typedef enum _mmc_timing_mode_t {
    MMC_LEGACY,
    MMC_HS,
    MMC_HS200,
    MMC_HS400
} mmc_timing_mode_t;

/**
 * @brief MMC ext csd version
 */
typedef enum _mmc_ext_csd_ver_t {
    MMC_5P1,
    MMC_5P0,
    MMC_4P5,
    MMC_4P4,
    MMC_4P3,
    MMC_4P2,
    MMC_4P1,
    MMC_4P0
} mmc_ext_csd_ver_t;

/**
 * @brief SDMMC Power ON/OFF
 */
typedef enum _sdmmc_power_t {
    SDMMC_POWER_OFF,
    SDMMC_POWER_ON
} sdmmc_power_t;

/**
 * @brief SDMMC Voltage
 */
typedef enum _sdmmc_vol_t {
    SDMMC_VOL_1P8V,
    SDMMC_VOL_3P3V
} sdmmc_vol_t;

/**
 * @brief SDMMC Clocks
 */
typedef enum _sdmmc_clock_t {
    SDMMC_CLK_DISABLE,
    SDMMC_CLK_ENABLE,
    SDMMC_CLK_400KHZ,
    SDMMC_CLK_12P5MHZ,
    SDMMC_CLK_25MHZ,
    SDMMC_CLK_50MHZ
} sdmmc_clock_t;

/**
 * @brief SDMMC Bus Width
 */
typedef enum _sdmmc_bus_width_t {
    SDMMC_BUS_WIDTH_1BIT,
    SDMMC_BUS_WIDTH_4BIT,
    SDMMC_BUS_WIDTH_8BIT
} sdmmc_bus_width_t;

/**
 * @brief SDMMC IO definition
 */
typedef struct _sdmmc_io_t {
    sdmmc_power_t     sdmmc_power;
    sdmmc_vol_t       sdmmc_vol;
    sdmmc_clock_t     sdmmc_clock;
    sdmmc_bus_width_t sdmmc_bus_width;
} sdmmc_io_t;

/**
 * @brief MMC ext csd register
 */
typedef struct _mmc_ext_csd {
    uint32_t          sector_cnt;
    uint32_t          cache_size;
    mmc_timing_mode_t hs_mode;
    mmc_ext_csd_ver_t mmc_ext_csd_ver;
    uint8_t           bus_width;
    uint8_t           device_type;
    uint8_t           power_class;
    uint8_t           mmc_drv_strength;
} mmc_ext_csd_t;

/**
 * @brief  SD Card Information Structure definition
 */
typedef struct _sd_cardinfo_t {
    uint32_t cardtype;      /*!< Specifies the card Type                        */
    uint32_t cardversion;   /*!< Specifies the card version                     */
    uint32_t relcardadd;    /*!< Specifies the Relative Card Address            */
    uint32_t sectorcount;   /*!< Specifies the Card Capacity in blocks          */
    uint32_t sectorsize;    /*!< Specifies one block size in bytes              */
    uint32_t logblocknbr;   /*!< Specifies the Card logical Capacity in blocks  */
    uint32_t logblocksize;  /*!< Specifies logical block size in bytes          */
    uint32_t busspeed;      /*!< Clock                                          */
    uint32_t csd[4];        /*!< SD card specific data table                    */
    uint32_t cid[4];        /*!< SD card identification number table            */
    uint32_t scr[2];        /*!< SD card Configuration Register                 */
    sdio_t   sdio;          /*!< sdio card information                          */
    uint16_t card_class;    /*!< Specifies the class of the card class          */
    uint16_t flags;         /*!< SD Card Supported Flags                        */
    uint8_t  iscardpresent; /*!< is card present flag                           */
    uint8_t  f8flag;        /*!< CMD8 support flag, set after good resp of CMD8 */
    uint8_t  sdio_mode;     /*!< sdio only mode flag                            */
} sd_cardinfo_t;

/**
 * @brief  SD data phase descriptor
 */
typedef struct _sd_data_t {
    uint32_t buffer;    /*!< DMA buffer address (local)     */
    uint32_t sector;    /*!< Sector number for read/write   */
    uint16_t blk_size;  /*!< Block size in bytes            */
    uint16_t blk_cnt;   /*!< Number of blocks to transfer   */
    uint8_t  direction; /*!< Transfer direction (read/write) */
} sd_data_t;

/**
 * @brief  SD command structure definition
 */
typedef struct _sd_cmd_t {
    uint8_t   card_buffer[SDMMC_CACHED_NUM_BLK * SDMMC_BLK_SIZE_512_Msk]
        __attribute__((aligned(512)));
    sd_data_t data;         /*!< Data phase descriptor      */
    uint32_t  arg;          /*!< SD Command Argument        */
    uint16_t  xfer_mode;    /*!< SD Command transfer mode   */
    uint8_t   cmdidx;       /*!< SD Command index           */
    uint8_t   rsp_type;     /*!< Expected response type     */
    uint8_t   data_present; /*!< SD Command uses Data lines */
    uint8_t   retries;      /*!< Max retries on error/timeout */
} sd_cmd_t;

/**
 * @brief SD Default init Parameters
 */
typedef struct _sd_param_t {
    uint8_t dev_id;                           /*!< SD Device ID                    */
    uint8_t bus_width;                        /*!< SD Bus Width 0: 1 Bit 1: 4Bit   */
    uint8_t dma_mode;                         /*!< SD DMA Mode 0: SDMA 1: ADMA2    */
    uint8_t operation_mode;                   /*!< SD operation mode 0: Polling 1: Interrupt */
    uint32_t clock_freq;                      /*!< SD Clock frequency in Hz        */
    void (*app_callback)(uint16_t cmd_complete,
        uint16_t xfer_complete);              /*!< SD App Callback function pointer */
    void (*pwr_cb)(uint8_t power_on);         /*!< SD Power Callback function pointer (power_on: 0=off, 1=on) */
    int (*card_det_cb)(void);                 /*!< SD Card Detect Callback function pointer (returns 1 if present, 0 if not) */
    void (*vsel_cb)(uint8_t voltage);         /*!< SD VSEL Callback function pointer (voltage: 0=3.3V, 1=1.8V) */
} sd_param_t;

/**
 * @brief  Global SD Handle Information Structure definition
 */
typedef struct _sd_handle_t {
    SDMMC_Type   *regs;        /*!< SD controller registers base address   */
    sd_cmd_t      sd_cmd;      /*!< SD Command info                        */
    sd_cardinfo_t sd_card;     /*!< SD Card information                    */
    uint32_t      hc_caps;     /*!< Host Controller capabilities           */

    __IO uint32_t context;     /*!< SD transfer context                    */
    __IO uint32_t error_code;  /*!< SD Card Error codes                    */
    SD_CARD_STATE state;       /*!< SD card State                          */
    uint16_t      hc_version;  /*!< Host controller version                */
    sd_param_t    sd_param;    /*!< SD Default Config Parameters           */
    mmc_ext_csd_t mmc_ext_csd; /*!< mmc extended card specific data        */
} sd_handle_t;

#ifdef __cplusplus
}
#endif

#endif /* _SD_TYPES_H_ */
