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

#include "sd_types.h"

/* ======================================================================== */
/* SD Protocol Constants                                                    */
/* ======================================================================== */

#define SDMMC_RESET_DELAY_US      1000U
#define SDMMC_EXT_CSD_SIZE        512U
#define SDMMC_DATA_TIMEOUT        0xFFFU
#define SDMMC_CMD_TIMEOUT         0xFFU
#define SDMMC_CMD_OCR_TOUT        0x1FFU
#define SDMMC_SDHC_MAX_SECTOR_CNT 0xFFFFU
#define SDMMC_SWITCH_MODE_Pos     0x1FU
#define SDMMC_SWITCH_MODE_Msk     0x1U

/* SDMMC Device ID */
#define SDMMC_DEV_ID                      1U

/* Card Types */
#define SDMMC_CARD_SDSC                   1U /*!< SDSC, Ver 1 Cards  */
#define SDMMC_CARD_SDXC                   3U /*!< SDXC               */
#define SDMMC_CARD_MMC                    4U /*!< MMC Cards          */
#define SDMMC_CARD_SDIO                   5U /*!< SDIO Cards         */
#define SDMMC_CARD_COMBO                  6U /*!< SD Combo Cards     */

/* Command index */
#define APP_CMD_PREFIX                    0x80U /* App CMD prefix       */
#define CMD0                              0x00U
#define CMD1                              0x01U
#define CMD2                              0x02U
#define CMD3                              0x03U
#define CMD4                              0x04U
#define CMD5                              0x05U
#define CMD6                              0x06U
#define ACMD6                             (APP_CMD_PREFIX + 0x06U)
#define CMD7                              0x07U
#define CMD8                              0x08U
#define CMD9                              0x09U
#define CMD10                             0x0AU
#define CMD11                             0x0BU
#define CMD12                             0x0CU
#define CMD13                             0x0DU
#define ACMD13                            (APP_CMD_PREFIX + 0x0DU)
#define CMD16                             0x10U
#define CMD17                             0x11U
#define CMD18                             0x12U
#define CMD19                             0x13U
#define CMD21                             0x15U
#define CMD23                             0x17U
#define ACMD23                            (APP_CMD_PREFIX + 0x17U)
#define CMD24                             0x18U
#define CMD25                             0x19U
#define CMD32                             0x20U
#define CMD33                             0x21U
#define CMD35                             0x23U
#define CMD36                             0x24U
#define CMD38                             0x26U
#define CMD41                             0x29U
#define ACMD41                            (APP_CMD_PREFIX + 0x29U)
#define ACMD42                            (APP_CMD_PREFIX + 0x2AU)
#define ACMD51                            (APP_CMD_PREFIX + 0x33U)
#define CMD51                             0x33U
#define CMD52                             0x34U
#define CMD53                             0x35U
#define CMD55                             0x37U
#define CMD58                             0x3AU

/* CMD_R */
#define SDMMC_CMD_R_CRC_CHK_EN_Msk        8U
#define SDMMC_CMD_R_CMD_IDX_CHK_EN_Msk    0x10U

/* Response type */
#define SDMMC_RESP_NONE                   0U /*!< No response expected     */
#define SDMMC_RESP_R136                   1U /*!< 128Bit response expected */
#define SDMMC_RESP_R48                    2U /*!< Single response expected */
#define SDMMC_RESP_R48B                   3U /*!< check busy after resp    */

#define SDMMC_RESP_R1 (SDMMC_RESP_R48 | SDMMC_CMD_R_CRC_CHK_EN_Msk | \
    SDMMC_CMD_R_CMD_IDX_CHK_EN_Msk)

/* Card CSD */
#define RAW_CSD_BUF_IDX0                  0U
#define RAW_CSD_BUF_IDX1                  1U
#define RAW_CSD_BUF_IDX2                  2U
#define RAW_CSD_BUF_IDX3                  3U
#define CSD_SPEC_VER_Msk                  0x003C0000U
#define READ_BLK_LEN_Msk                  0x00000F00U
#define C_SIZE_MULT_Msk                   0x00000380U
#define C_SIZE_LOWER_Msk                  0xFFC00000U
#define C_SIZE_LOWER_Pos                  22U
#define C_SIZE_UPPER_Msk                  0x00000003U
#define C_SIZE_UPPER_Pos                  0x0000000AU
#define CSD_STRUCT_Msk                    0x00C00000U
#define CSD_STRUCT_Pos                    22U
#define CSD_V2_C_SIZE_Msk                 0x3FFFFF00U
#define CSD_V2_C_SIZE_Pos                 0x8U
#define CSD_V1_BLK_LEN_Pos                0x8U
#define CSD_CCC_Msk                       0xFFF00000U
#define CSD_CCC_SHIFT                     20U
#define CSD_CCC_CLASS5_Msk                0x20U

/* ext CSD Command Index */
#define SDMMC_EXT_CSD_CMD_BOOT_CFG        0xAFU
#define SDMMC_EXT_CSD_CMD_BUS_WIDTH       0xB7U
#define SDMMC_EXT_CSD_CMD_HS_MODE         0xB9U
#define SDMMC_EXT_CSD_HS_MODE             0x1U
#define SDMMC_EXT_CSD_BOOT_WR_PROTECT     0x1U

/* CMD6 Argument Format */
#define SDMMC_CMD6_BLK_SIZE               64U
#define SDMMC_CMD6_DELAY                  5000U
#define SDMMC_CMD6_ACCESS_MODE_CMD        0U
#define SDMMC_CMD6_ACCESS_MODE_SET_BIT    1U
#define SDMMC_CMD6_ACCESS_MODE_CLR_BIT    2U
#define SDMMC_CMD6_ACCESS_MODE_WR_BYTE    3U
#define SDMMC_CMD6_SWITCH_MODE_APPLY      1U
#define SDMMC_CMD6_SWITCH_MODE_TEST       0U

/* SD Card CMD6 Speed Mode Selection (Function Group 6) */
#define SDMMC_CMD6_GRP6_POS               16U
#define SDMMC_CMD6_GRP6_Msk(MODE)         ((MODE) << SDMMC_CMD6_GRP6_POS)
#define SDMMC_CMD6_GRP1_POS               8U
#define SDMMC_CMD6_GRP1_Msk(MODE)         ((MODE) << SDMMC_CMD6_GRP1_POS)

/* CMD6 Speed Mode Support Bits (Byte 13) */
#define SDMMC_CMD6_SPEED_DS_BIT           0x01U  /* Bit 0: Default Speed */
#define SDMMC_CMD6_SPEED_HS_BIT           0x02U  /* Bit 1: High Speed */
#define SDMMC_CMD6_SPEED_SDR50_BIT        0x04U  /* Bit 2: SDR50 */
#define SDMMC_CMD6_SPEED_SDR104_BIT       0x08U  /* Bit 3: SDR104 */

/* SD Card Speed Modes */
#define SDMMC_SPEED_MODE_DEFAULT          0U
#define SDMMC_SPEED_MODE_HS               1U
#define SDMMC_SPEED_MODE_SDR12            0U
#define SDMMC_SPEED_MODE_SDR25            1U
#define SDMMC_SPEED_MODE_SDR50            2U
#define SDMMC_SPEED_MODE_SDR104           3U
#define SDMMC_SPEED_MODE_DDR50            4U

#define SDMMC_EXT_CSD_WRITE_Pos           0x18U
#define SDMMC_EXT_CSD_WRITE_Msk \
    (SDMMC_CMD6_ACCESS_MODE_WR_BYTE << SDMMC_EXT_CSD_WRITE_Pos)
#define SDMMC_EXT_CSD_IDS_Pos             0x10
#define SDMMC_EXT_CSD_IDX_Msk(IDX)        ((IDX) << SDMMC_EXT_CSD_IDS_Pos)
#define SDMMC_EXT_CSD_VAL_Pos             0x8U
#define SDMMC_EXT_CSD_VAL_Msk(VAL)        ((VAL) << SDMMC_EXT_CSD_VAL_Pos)

/* CMD6 Switch Function Constants */
#define SDMMC_CMD6_CHECK_ARG              0x00FFFFFFU /*!< CMD6 CHECK mode argument */
#define SDMMC_CMD6_FUNCTION_GROUP_MASK    0xFFFFF0U   /*!< Function group mask for CMD6 */

/* Clock Frequency Constants */
#define SDMMC_CLK_400_KHZ_HZ              400000U     /*!< 400 KHz in Hz */
#define SDMMC_CLK_50_MHZ_HZ               50000000U   /*!< 50 MHz in Hz */
#define SDMMC_CLK_100_MHZ_HZ              100000000U  /*!< 100 MHz in Hz */

/* Card Deselection */
#define SDMMC_DESELECT_RCA                0x00000000U /*!< RCA for card deselection */

/* Card Interface Conditions constants */
#define SDMMC_CMD8_VOL_PATTERN            0x1AAU /*!< CMD8 Voltage Pattern */

/* Card Operating Conditions constants */
#define SDMMC_OCR_READY                   0x80000000U /*!< OCR Ready */
#define SDMMC_OCR_S18R                    0x1000000U
#define SDMMC_ROCR_S18A                   SDMMC_OCR_S18R
#define SDMMC_CMD41_HCS                   0x40000000U /*!< ACMD41 High Capacity */
#define SDMMC_CMD41_3V3                   0x00300000U /*!< ACMD41 3.3v support */
#define SDMMC_CMD41_S18A                  SDMMC_OCR_S18R /*!< ACMD41 1.8v switch */

/* CMD6 Timeout */
#define SDMMC_CMD6_TIMEOUT_US             100000U  /* 100ms timeout for CMD6 operations */

/* SD Relative Card Address Constant */
#define SDMMC_RCA_Pos                     0x10U
#define SDMMC_RCA_Msk                     0xFFFF0000U
#define EMMC_DEFAULT_RCA                  0x12340000U

/**
 * @brief  SD data transfer direction
 */
#define SD_DATA_DIR_READ   1U
#define SD_DATA_DIR_WRITE  0U

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
 * @brief MMC Switch Mode
 */
typedef enum _sdmmc_switch_t {
    SDMMC_SWITCH_CHECK,
    SDMMC_SWITCH_SET
} sd_switch_type_t;

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
 * @brief  Disk IO Driver structure definition
 */
typedef struct _diskio_t {
    SD_DRV_STATUS (*disk_initialize)(sd_param_t *); /*!< Initialize Disk Drive      */
    SD_DRV_STATUS (*disk_uninitialize)(uint8_t);    /*!< Un-initialize Disk Drive   */
    SD_CARD_STATE (*disk_status)(void);             /*!< Get Disk Status            */
    SD_DRV_STATUS (*disk_info)(sd_cardinfo_t *);    /*!< Get Disk information       */
    SD_DRV_STATUS (*disk_read)
    (uint32_t, uint16_t, volatile uint8_t *);       /*!< Read Sector(s)             */
    SD_DRV_STATUS (*disk_write)
    (uint32_t, uint32_t, volatile uint8_t *);       /*!< Write Sector(s)            */
    SD_DRV_STATUS (*disk_set_io)
    (sdmmc_io_t *, SDMMC_SET_IO_CMD);               /*!< Set SDMMC IO, power, clk   */
#ifdef SDMMC_IRQ_MODE
    void (*disk_cb)(uint16_t, uint16_t);
#endif
} diskio_t;

extern const diskio_t SD_Driver;

/* SD Driver function forward declaration */
SD_CARD_STATE sd_state(void);
SD_DRV_STATUS sd_init(sd_param_t *p_sd_param);
SD_DRV_STATUS sd_status(void);
SD_DRV_STATUS sd_info(sd_cardinfo_t *pinfo);
SD_DRV_STATUS sd_uninit(uint8_t devId);
SD_DRV_STATUS sd_io_init(sd_handle_t *pHsd);
SD_DRV_STATUS sdio_read_cia(uint8_t *pcia, uint8_t fn, uint8_t offset);
SD_DRV_STATUS sdio_write_cia(uint8_t cia, uint8_t fn, uint8_t offset);
SD_DRV_STATUS sdio_read_cccr(uint8_t *pcccr);
SD_DRV_STATUS sdio_write_cccr(uint8_t cccr);
SD_DRV_STATUS sd_write(uint32_t sector, uint32_t blk_cnt,
                       volatile unsigned char *src_buff);
SD_DRV_STATUS sd_read(uint32_t sec, uint16_t blk_cnt,
                      volatile unsigned char *dest_buff);
SD_DRV_STATUS sd_set_io(sdmmc_io_t *p_sdmmc_io_param, SDMMC_SET_IO_CMD set_io_cmd);
void          sdmmc_decode_card_csd(sd_handle_t *pHsd);
void          sdmmc_decode_card_ext_csd(sd_handle_t *pHsd, uint8_t *praw_ext_csd);
#ifdef SDMMC_IRQ_MODE
void sd_cb(uint16_t cmd_complete, uint16_t xfer_complete);
#endif
#ifdef __cplusplus
}
#endif

#endif
