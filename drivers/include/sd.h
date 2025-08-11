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
 * @file     sd.h
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    exposed SD Driver variables and APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef _SD_H_
#define _SD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sd_host.h"
#include "sdio.h"
#include "sys_utils.h"

// #define SDMMC_PRINT_ERR
// #define SDMMC_PRINT_WARN
// #define SDMMC_PRINTF_DEBUG
// #define SDMMC_PRINTF_SD_STATE_DEBUG
// #define SDMMC_PRINT_SEC_DATA
#define SDMMC_IRQ_MODE

#define SDMMC_RESET_DELAY_US      1000U
#define SDMMC_CACHED_NUM_BLK      8U
#define SDMMC_EXT_CSD_SIZE        512U
#define SDMMC_DATA_TIMEOUT        0xFFFU
#define SDMMC_CMD_TIMEOUT         0xFFU
#define SDMMC_SDHC_MAX_SECTOR_CNT 0xFFFFU
#define SDMMC_SWITCH_MODE_Pos     0x1FU
#define SDMMC_SWITCH_MODE_Msk     0x1U

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

/**
 * @brief SD support flags
 * used by SD subsystem to determine support for SD card features.
 */
typedef enum _sdmmc_support_flag_t {
    SDMMC_HIGH_CAPACITY_FLAG    = BIT(1),
    SDMMC_4BITS_WIDTH           = BIT(2),
    SDMMC_SDHC_FLAG             = BIT(3),
    SDMMC_SDXC_FLAG             = BIT(4),
    SDMMC_1P8V_FLAG             = BIT(5),
    SDMMC_3P0V_FLAG             = BIT(6),
    SDMMC_CMD23_FLAG            = BIT(7),
    SDMMC_SPEED_CLASS_CTRL_FLAG = BIT(8),
    SDMMC_MEM_PRESENT_FLAG      = BIT(9),
} sdmmc_support_flag_t;

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
 * @brief MMC Switch Mode
 */
typedef enum _sdmmc_switch_t {
    SDMMC_SWITCH_CHECK,
    SDMMC_SWITCH_SET
} sd_switch_type_t;

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
 * @brief MMC device types enum
 */
typedef enum _mmc_dev_type_t {
    MMC_HS400_DDR_1P2V,
    MMC_HS400_DDR_1P8V,
    MMC_HS200_DDR_1P2V,
    MMC_HS200_DDR_1P8V,
    MMC_HS_DDR_1P2V,
    MMC_HS_DDR_1P8V,
    MMC_HS_52MHZ,
    MMC_HS_26MHZ
} mmc_dev_type_t;

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
    uint8_t           bus_width;
    mmc_timing_mode_t hs_mode;
    uint8_t           device_type;
    mmc_ext_csd_ver_t mmc_ext_csd_ver;
    uint8_t           power_class;
    uint8_t           mmc_drv_strength;
    uint32_t          cache_size;
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
    uint16_t card_class;    /*!< Specifies the class of the card class          */
    uint16_t flags;         /*!< SD Card Supported Flags                        */
    uint8_t  iscardpresent; /*!< is card present flag                           */
    uint8_t  f8flag;        /*!< CMD8 support flag, set after good resp of CMD8 */
    uint8_t  sdio_mode;     /*!< sdio only mode flag                            */
    sdio_t   sdio;          /*!< sdio card information                          */
} sd_cardinfo_t;

/**
 * @brief  SD command structure definition
 */
typedef struct _sd_cmd_t {
    uint32_t arg;          /*!< SD Command Argument        */
    uint16_t xfer_mode;    /*!< SD Command transfer mode   */
    uint8_t  cmdidx;       /*!< SD Command index           */
    uint8_t  data_present; /*!< SD Command uses Data lines */
    uint8_t  card_buffer[SDMMC_CACHED_NUM_BLK * SDMMC_BLK_SIZE_512_Msk]
        __attribute__((aligned(512)));
} sd_cmd_t;

/**
 * @brief SD Default init Parameters
 */
typedef struct _sd_param_t {
    uint8_t dev_id;                           /*!< SD Device ID                               */
    uint8_t clock_id;                         /*!< SD Clock id 0: 12.5MHz 1: 25MHz 2: 50MHz   */
    uint8_t bus_width;                        /*!< SD Bus Width 0: 1 Bit 1: 4Bit              */
    uint8_t dma_mode;                         /*!< SD DMA Mode 0: SDMA 1: ADMA2               */
    uint8_t operation_mode;                   /*!< SD operation mode 0: Polling 1: Interrupt  */
    void (*app_callback)(uint16_t, uint16_t); /*!< SD Application Callback function pointer   */
    void (*reset_cb)(void);                    /*!< SD Application Callback function pointer   */
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
    uint32_t      csd[4];      /*!< SD card specific data table            */
    uint32_t      cid[4];      /*!< SD card identification number table    */
    uint32_t      scr[2];      /*!< SD card Configuration Register         */
    SD_CARD_STATE state;       /*!< SD card State                          */
    uint16_t      hc_version;  /*!< Host controller version                */
    sd_param_t    sd_param;    /*!< SD Default Config Parameters           */
    mmc_ext_csd_t mmc_ext_csd; /*!< mmc extended card specific data        */
} sd_handle_t;

/**
 * @brief  Disk IO Driver structure definition
 */
typedef struct _diskio_t {
    SD_DRV_STATUS (*disk_initialize)(sd_param_t *); /*!< Initialize Disk Drive      */
    SD_DRV_STATUS (*disk_uninitialize)(uint8_t);    /*!< Un-initialize Disk Drive   */
    SD_CARD_STATE (*disk_status)(void);             /*!< Get Disk Status            */
    SD_DRV_STATUS (*disk_info)(sd_cardinfo_t *);    /*!< Get Disk information       */
    SD_DRV_STATUS (*disk_read)
    (uint32_t, uint16_t, volatile uint8_t *); /*!< Read Sector(s)             */
    SD_DRV_STATUS (*disk_write)
    (uint32_t, uint32_t, volatile uint8_t *); /*!< Write Sector(s)            */
    SD_DRV_STATUS (*disk_set_io)
    (sdmmc_io_t *, SDMMC_SET_IO_CMD);         /*!< Set SDMMC IO, power, clk   */
#ifdef SDMMC_IRQ_MODE
    void (*disk_cb)(uint16_t, uint16_t);
#endif
} diskio_t;

extern const diskio_t SD_Driver;

/* SD Driver function forward declaration */
SD_CARD_STATE sd_state(void);
SD_DRV_STATUS sd_init(sd_param_t *);
SD_DRV_STATUS sd_info(sd_cardinfo_t *);
SD_DRV_STATUS sd_uninit(uint8_t);
SD_DRV_STATUS sd_io_init(sd_handle_t *);
SD_DRV_STATUS sdio_read_cia(uint8_t *pcia, uint8_t fn, uint8_t offset);
SD_DRV_STATUS sd_host_init(sd_handle_t *, sd_param_t *);
SD_DRV_STATUS sd_card_init(sd_handle_t *, sd_param_t *);
SD_DRV_STATUS sd_write(uint32_t, uint32_t, volatile unsigned char *);
SD_DRV_STATUS sd_read(uint32_t, uint16_t, volatile unsigned char *);
SD_DRV_STATUS sd_set_io(sdmmc_io_t *p_sdmmc_io, SDMMC_SET_IO_CMD set_io_cmd);
SD_DRV_STATUS sd_error_handler();
void          sdmmc_decode_card_csd(sd_handle_t *);
void          sdmmc_decode_card_ext_csd(sd_handle_t *, uint8_t *);
#ifdef SDMMC_IRQ_MODE
void sd_cb(uint16_t, uint16_t);
#endif
SDMMC_HC_STATUS hc_send_cmd(sd_handle_t *, sd_cmd_t *);
SDMMC_HC_STATUS hc_reset(sd_handle_t *, uint8_t);
void            hc_power_cycle(sd_handle_t *pHsd);
SDMMC_HC_STATUS hc_set_bus_power(sd_handle_t *pHsd, uint8_t req_vol);
SDMMC_HC_STATUS hc_set_clk_freq(sd_handle_t *, uint16_t);
void            hc_set_tout(sd_handle_t *, uint8_t);
SDMMC_HC_STATUS hc_dma_config(sd_handle_t *pHsd, uint32_t buff, uint16_t len);
SDMMC_HC_STATUS hc_identify_card(sd_handle_t *);
SDMMC_HC_STATUS hc_get_card_ifcond(sd_handle_t *);
SDMMC_HC_STATUS hc_get_card_opcond(sd_handle_t *pHsd, uint32_t ocr);
SDMMC_HC_STATUS hc_get_emmc_card_opcond(sd_handle_t *);
SDMMC_HC_STATUS hc_get_card_cid(sd_handle_t *);
SDMMC_HC_STATUS hc_get_card_csd(sd_handle_t *);
SDMMC_HC_STATUS hc_get_card_ext_csd(sd_handle_t *, uint8_t *);
SDMMC_HC_STATUS hc_get_card_scr(sd_handle_t *);
SDMMC_HC_STATUS hc_config_dma(sd_handle_t *, uint8_t);
SDMMC_HC_STATUS hc_set_bus_width(sd_handle_t *, uint8_t);
SDMMC_HC_STATUS hc_switch_1v8(sd_handle_t *);
SDMMC_HC_STATUS hc_go_idle(sd_handle_t *);
SDMMC_HC_STATUS hc_sel_card(sd_handle_t *, uint32_t);
SDMMC_HC_STATUS hc_set_blk_size(sd_handle_t *, uint32_t);
SDMMC_HC_STATUS hc_set_blk_cnt(sd_handle_t *, uint32_t);
SDMMC_HC_STATUS hc_read_setup(sd_handle_t *, uint32_t, uint32_t, uint16_t);
SDMMC_HC_STATUS hc_write_setup(sd_handle_t *, uint32_t, uint32_t, uint16_t);
SDMMC_HC_STATUS hc_check_xfer_done(sd_handle_t *, uint32_t);
SDMMC_HC_STATUS hc_get_rca(sd_handle_t *, uint32_t *);
SDMMC_HC_STATUS hc_get_card_status(sd_handle_t *pHsd, uint32_t *);
SDMMC_HC_STATUS hc_get_capabilities(sd_handle_t *, uint32_t *);
uint32_t        hc_get_response1(sd_handle_t *);
uint32_t        hc_get_response2(sd_handle_t *);
uint32_t        hc_get_response3(sd_handle_t *);
uint32_t        hc_get_response4(sd_handle_t *);
void            hc_config_interrupt(sd_handle_t *);
SDMMC_HC_STATUS hc_io_reset(sd_handle_t *);
SDMMC_HC_STATUS hc_check_bus_idle(sd_handle_t *);
SDMMC_HC_STATUS hc_get_io_opcond(sd_handle_t *, uint32_t, uint32_t *);
SDMMC_HC_STATUS hc_io_rw_direct(sd_handle_t *pHsd, uint32_t rwFlag, uint32_t fn, uint32_t addr,
                                uint8_t writeData, uint8_t *readPtr);

#ifdef __cplusplus
}
#endif

#endif
