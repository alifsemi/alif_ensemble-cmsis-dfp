/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     canfd.h
 * @author   Shreehari H K
 * @email    shreehari.hk@alifsemi.com
 * @version  V1.0.0
 * @date     05-July-2023
 * @brief    Header file for canfd
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
#ifndef CANFD_H_
#define CANFD_H_

#include <stdint.h>
#include <stdbool.h>

/* CANFD register structure */
typedef struct _CANFD_Type
{
    volatile const uint32_t   RESERVED[38];
    volatile const uint32_t   CANFD_TTS;                    /*!< (@ 0x00000098) Transmission Time Stamp Register                           */
    volatile const uint32_t   RESERVED1;
    volatile       uint8_t    CANFD_CFG_STAT;               /*!< (@ 0x000000A0) Configuration and Status Register                          */
    volatile       uint8_t    CANFD_TCMD;                   /*!< (@ 0x000000A1) Command Register                                           */
    volatile       uint8_t    CANFD_TCTRL;                  /*!< (@ 0x000000A2) Transmit Control Register                                  */
    volatile       uint8_t    CANFD_RCTRL;                  /*!< (@ 0x000000A3) Receive Control Register                                   */
    volatile       uint8_t    CANFD_RTIE;                   /*!< (@ 0x000000A4) Receive and Transmit Interrupt Enable Register             */
    volatile       uint8_t    CANFD_RTIF;                   /*!< (@ 0x000000A5) Receive and Transmit Interrupt Flag Register               */
    volatile       uint8_t    CANFD_ERRINT;                 /*!< (@ 0x000000A6) Error Interrupt Enable and Flag Register                   */
    volatile       uint8_t    CANFD_LIMIT;                  /*!< (@ 0x000000A7) Warning Limits Register                                    */
    volatile       uint8_t    CANFD_S_SEG_1;                /*!< (@ 0x000000A8) Slow Speed Bit Timing 1 Register (Segment 1)               */
    volatile       uint8_t    CANFD_S_SEG_2;                /*!< (@ 0x000000A9) Slow Speed Bit Timing 2 Register (Segment 2)               */
    volatile       uint8_t    CANFD_S_SJW;                  /*!< (@ 0x000000AA) Slow Speed Bit Timing 3 Register (Synchronization
                                                                     Jump Width)                                                */
    volatile       uint8_t    CANFD_S_PRESC;                /*!< (@ 0x000000AB) Slow Speed Prescaler Register                              */
    volatile       uint8_t    CANFD_F_SEG_1;                /*!< (@ 0x000000AC) Fast Speed Bit Timing 1 Register (Segment 1)               */
    volatile       uint8_t    CANFD_F_SEG_2;                /*!< (@ 0x000000AD) Fast Speed Bit Timing 2 Register (Segment 2)               */
    volatile       uint8_t    CANFD_F_SJW;                  /*!< (@ 0x000000AE) Fast Speed Bit Timing 3 Register (Synchronization
                                                                      Jump Width)                                                */
    volatile       uint8_t    CANFD_F_PRESC;                /*!< (@ 0x000000AF) Fast Speed Prescaler Register                              */
    volatile const uint8_t    CANFD_EALCAP;                 /*!< (@ 0x000000B0) Error and Arbitration Lost Capture Register                */
    volatile       uint8_t    CANFD_TDC;                    /*!< (@ 0x000000B1) Transmitter Delay Compensation Register                    */
    volatile const uint8_t    CANFD_RECNT;                  /*!< (@ 0x000000B2) Receive Error Counter Register                             */
    volatile const uint8_t    CANFD_TECNT;                  /*!< (@ 0x000000B3) Transmit Error Counter Register                            */
    volatile       uint8_t    CANFD_ACFCTRL;                /*!< (@ 0x000000B4) Acceptance Filter Control Register                         */
    volatile       uint8_t    CANFD_TIMECFG;                /*!< (@ 0x000000B5) CiA 603 Time-Stamping Register                             */
    volatile       uint8_t    CANFD_ACF_EN_0;               /*!< (@ 0x000000B6) Acceptance Filter Enable 0 Register                        */
    volatile       uint8_t    CANFD_ACF_EN_1;               /*!< (@ 0x000000B7) Acceptance Filter Enable 1 Register                        */

    union {
     volatile      uint32_t   CANFD_ACF_0_3_CODE;           /*!< (@ 0x000000B8 - 0x000000BB) Acceptance CODE Register 0-3                  */
     volatile      uint32_t   CANFD_ACF_0_3_MASK;           /*!< (@ 0x000000B8 - 0x000000BB) Acceptance MASK Register 0-3                  */
    };

    volatile const uint8_t    CANFD_VER_0;                  /*!< (@ 0x000000BC) CANFD Version                                              */
    volatile const uint8_t    CANFD_VER_1;                  /*!< (@ 0x000000BD) CANFD Version                                              */
    volatile const uint16_t   RESERVED2;
    volatile const uint32_t   RESERVED3[2];
    volatile const uint16_t   RESERVED4;
    volatile       uint8_t    CANFD_MEM_PROT;               /*!< (@ 0x000000CA) Memory Protection Register                                 */
    volatile       uint8_t    CANFD_MEM_STAT;               /*!< (@ 0x000000CB) Memory Status Register                                     */
    volatile       uint8_t    CANFD_MEM_ES_0;               /*!< (@ 0x000000CC) Memory Error Stimulation 0 Register                        */
    volatile       uint8_t    CANFD_MEM_ES_1;               /*!< (@ 0x000000CD) Memory Error Stimulation 1 Register                        */
    volatile       uint8_t    CANFD_MEM_ES_2;               /*!< (@ 0x000000CE) Memory Error Stimulation 2 Register                        */
    volatile       uint8_t    CANFD_MEM_ES_3;               /*!< (@ 0x000000CF) Memory Error Stimulation 3 Register                        */
    volatile       uint8_t    CANFD_SRCFG;                  /*!< (@ 0x000000D0) Spatial Redundancy Configuration Register                  */
    volatile const uint8_t    RESERVED5;
    volatile const uint16_t   RESERVED6;
}CANFD_Type;

/* Hardware related configuration */
#define CANFD_RX_MSG_BUFFER_OFFSET               0U              /* Rx buffer starts at offset 0 */
#define CANFD_TX_MSG_BUFFER_OFFSET               80U             /* Tx buffer starts at offset 80th byte */

#define CANFD_MAX_ACCEPTANCE_FILTERS             3U
#define CANFD_MAX_ERROR_WARN_LIMIT               128U
#define CANFD_NOM_DATA_FRAME_SIZE_MIN            0U
#define CANFD_NOM_DATA_FRAME_SIZE_MAX            8U
#define CANFD_FAST_DATA_FRAME_SIZE_MIN           0U
#define CANFD_FAST_DATA_FRAME_SIZE_MAX           64U
#define CANFD_DECREMENT(x, pos)                  (x - pos)

/* Macros for Configuration and status register */
#define CANFD_CFG_STAT_RESET                     (1U << 7U)
#define CANFD_CFG_STAT_LOOPBACK_MODE_EXTERNAL    (1U << 6U)
#define CANFD_CFG_STAT_LOOPBACK_MODE_INTERNAL    (1U << 5U)
#define CANFD_CFG_STAT_SINGLE_SHOT_MODE_TX_PTB   (1U << 4U)
#define CANFD_CFG_STAT_SINGLE_SHOT_MODE_TX_STB   (1U << 3U)
#define CANFD_CFG_STAT_RACTIVE_STATUS            (1U << 2U)
#define CANFD_CFG_STAT_TACTIVE_STATUS            (1U << 1U)
#define CANFD_CFG_STAT_BUS_OFF_STATUS            (1U << 0U)

/* Macros for Command Register */
#define CANFD_TCMD_TX_BUFFER_SELECT              (1U << 7U)        /* 0-Primary, 1-Secondary */
#define CANFD_TCMD_LISTEN_ONLY_MODE              (1U << 6U)
#define CANFD_TCMD_STANDBY_MODE                  (1U << 5U)
#define CANFD_TCMD_PRIMARY_TX_EN                 (1U << 4U)
#define CANFD_TCMD_ABORT_PRIMARY_TX              (1U << 3U)
#define CANFD_TCMD_ONE_FRAME_SEC_TX_EN           (1U << 2U)
#define CANFD_TCMD_ALL_FRAMES_SEC_TX_EN          (1U << 1U)
#define CANFD_TCMD_ABORT_SEC_TX                  (1U << 0U)

/* Macros for Transmit Control Register */
#define CANFD_TCTRL_ISO_FD                       (1U << 7U)
#define CANFD_TCTRL_SEC_BUF_NEXT_SLOT            (1U << 6U)
#define CANFD_TCTRL_SEC_BUF_TX_MODE              (1U << 5U)        /* 0-FIFO, 1-Priority */
#define CANFD_TCTRL_SEC_BUF_TX_STATUS_Pos        (0U)
#define CANFD_TCTRL_SEC_BUF_TX_STATUS_Msk        (3U << CANFD_TCTRL_SEC_BUF_TX_STATUS_Pos)

/* Macros for Reception Control Register */
#define CANFD_RCTRL_SELF_ACK                     (1U << 7U)
#define CANFD_RCTRL_RX_BUF_OVF_MODE              (1U << 6U)        /* 0 - Old msg overwritten, 1 - New msg discarded */
#define CANFD_RCTRL_RX_BUF_OVF_Pos               (5U)
#define CANFD_RCTRL_RX_BUF_OVF_Msk               (1U << CANFD_RCTRL_RX_BUF_OVF_Pos)
#define CANFD_RCTRL_RX_BUF_RELEASE               (1U << 4U)
#define CANFD_RCTRL_RX_BUF_STR_ALL_FRAMES        (1U << 3U)        /* 1 - Stores both proper and error frames */
#define CANFD_RCTRL_RX_BUF_STATUS_Pos            (0U)
#define CANFD_RCTRL_RX_BUF_STATUS_Msk            (3U << CANFD_RCTRL_RX_BUF_STATUS_Pos)

/* Macros for Receive and Transmit Interrupt enable register */
#define CANFD_RTIE_RX_INTR_EN                    (1U << 7U)
#define CANFD_RTIE_RX_OVERRUN_INTR_EN            (1U << 6U)
#define CANFD_RTIE_RBUF_FULL_INTR_EN             (1U << 5U)
#define CANFD_RTIE_RBUF_ALMST_FULL_INTR_EN       (1U << 4U)
#define CANFD_RTIE_TX_PRIMARY_INTR_EN            (1U << 3U)
#define CANFD_RTIE_TX_SEC_INTR_EN                (1U << 2U)
#define CANFD_RTIE_ERROR_INTR_EN                 (1U << 1U)
#define CANFD_RTIE_SEC_TBUF_FULL_INTR_FLAG       (1U << 0U)

/* Macros for Receive and Transmit Interrupt Flag register */
#define CANFD_RTIF_REG_Msk                       (255U)
#define CANFD_RTIF_RX_INTR_FLAG                  (1U << 7U)
#define CANFD_RTIF_RX_OVERRUN_INTR_FLAG          (1U << 6U)
#define CANFD_RTIF_RBUF_FULL_INTR_FLAG           (1U << 5U)
#define CANFD_RTIF_RBUF_ALMST_FULL_INTR_FLAG     (1U << 4U)
#define CANFD_RTIF_TX_PRIMARY_INTR_FLAG          (1U << 3U)
#define CANFD_RTIF_TX_SEC_INTR_FLAG              (1U << 2U)
#define CANFD_RTIF_ERROR_INTR_FLAG               (1U << 1U)
#define CANFD_RTIF_ABORT_INTR_FLAG               (1U << 0U)

/* Macros for Error Interrupt Enable and Flag Register */
#define CANFD_ERRINT_REG_Msk                     (21U)             /* Mask for Error Flags */
#define CANFD_ERRINT_EN_Msk                      (42)              /* Mask for Error interrupt enabling */
#define CANFD_ERRINT_EWARN_LMT_RCHD              (1U << 7U)
#define CANFD_ERRINT_EPASSIVE_MODE               (1U << 6U)
#define CANFD_ERRINT_EPASSIVE_INTR_EN            (1U << 5U)
#define CANFD_ERRINT_EPASSIVE_INTR_FLAG          (1U << 4U)
#define CANFD_ERRINT_ARBTR_LOST_INTR_EN          (1U << 3U)
#define CANFD_ERRINT_ARBTR_LOST_INTR_FLAG        (1U << 2U)
#define CANFD_ERRINT_BUS_ERROR_INTR_EN           (1U << 1U)
#define CANFD_ERRINT_BUS_ERROR_INTR_FLAG         (1U << 0U)

/* Macros for Warning Limits Register */
#define CANFD_LIMIT_ALMST_FULL_WARN_LMT_Pos      (4U)
#define CANFD_LIMIT_ALMST_FULL_WARN_LMT_Msk      (15U << CANFD_LIMIT_ALMST_FULL_WARN_LMT_Pos) /* Max limit is 8 */
#define CANFD_LIMIT_ALMST_FULL_WARN_LMT(x)       ((x << CANFD_LIMIT_ALMST_FULL_WARN_LMT_Pos) & (CANFD_LIMIT_ALMST_FULL_WARN_LMT_Msk))

#define CANFD_LIMIT_ERR_WARN_LMT_Pos             (0U)
#define CANFD_LIMIT_ERR_WARN_LMT_Msk             (15U << CANFD_LIMIT_ERR_WARN_LMT_Pos)
#define CANFD_LIMIT_ERR_WARN_LMT(x)              ((x << CANFD_LIMIT_ERR_WARN_LMT_Pos) & (CANFD_LIMIT_ERR_WARN_LMT_Msk))

/* Macros for Error and Arbitration Lost Capture Register */
#define CANFD_EALCAP_KIND_OF_ERROR_Pos           (5U)
#define CANFD_EALCAP_KIND_OF_ERROR_Msk           (7U << CANFD_EALCAP_KIND_OF_ERROR_Pos)
#define CANFD_EALCAP_ARBTR_LOST_CAPTURE_Pos      (0U)
#define CANFD_EALCAP_ARBTR_LOST_CAPTURE_Msk      (31U << CANFD_EALCAP_ARBTR_LOST_CAPTURE_Pos)

/* Macros for Transmitter Delay Compensation Register */
#define CANFD_TDC_TX_DELAY_COMP_EN               (1U << 7U)
#define CANFD_TDC_SEC_SMPL_PT_OFFSET_Pos         (0U)
#define CANFD_TDC_SEC_SMPL_PT_OFFSET_Msk         (0x7FU << CANFD_TDC_SEC_SMPL_PT_OFFSET_Pos)

/* Macros for Acceptance Filter Control Register */
#define CANFD_ACFCTRL_ACPT_MASK_SEL              (1U << 5U)     /* Acceptance mask, 0- Acceptance code */
#define CANFD_ACFCTRL_ACPT_FLTR_ADDR_Pos         (0U)
#define CANFD_ACFCTRL_ACPT_FLTR_ADDR_Msk         (3U << CANFD_ACFCTRL_ACPT_FLTR_ADDR_Pos)

/* Macros for Acceptance filter Enable Register */
#define CANFD_ACF_EN_ACPT_FLTR_MAX_Msk           (7U << 0U)
#define CANFD_ACF_EN_ACPT_FLTR_EN_Msk            (7U << 0U)

/* Macros for Acceptance CODE and MASK Register */
#define CANFD_ACFX_ACPT_MASK_CODE_Pos            (0U)
#define CANFD_ACFX_ACPT_MASK_CODE_Msk            (0x1FFFFFFFU << CANFD_ACFX_ACPT_MASK_CODE_Pos)
#define CANFD_ACFX_ACPT_CODE(x)                  (x & CANFD_ACFX_ACPT_MASK_CODE_Msk)
#define CANFD_ACFX_ACPT_Msk(x)                   (x & CANFD_ACFX_ACPT_MASK_CODE_Msk)

#define CANFD_ACFX_ACPT_MASK_IDE_BIT_CHK_Pos     (30U)
#define CANFD_ACFX_ACPT_MASK_IDE_BIT_CHK_EN      (1 << 30U)     /* 1- Accepts either std or ext frame, 0 - accepts both */
#define CANFD_ACFX_ACPT_MASK_IDE_BIT_VAL_Pos     (29U)
#define CANFD_ACFX_ACPT_MASK_IDE_BIT_VAL         (1 << 29U)     /* 1 - Accepts only extended frame, 0-Accepts only std frames */

/* Macros for Memory Protection Register */
#define CANFD_MEM_PROT_MEM_ADDR_ERR_INTR_FLAG    (1U << 4U)
#define CANFD_MEM_PROT_MEM_DATA_ERR_INTR_FLAG    (1U << 3U)
#define CANFD_MEM_PROT_MEM_DATA_WARN_INTR_FLAG   (1U << 2U)
#define CANFD_MEM_PROT_MEM_DATA_WARN_INTR_EN     (1U << 1U)
#define CANFD_MEM_PROT_MEM_PROTECTION_EN         (1U << 0U)

/* Macros for Memory Status Register bit info*/
#define CANFD_MEM_STAT_HOSTSIDE_MEM_ERR_LOC_Pos  (3U)
#define CANFD_MEM_STAT_HOSTSIDE_MEM_ERR_LOC_Msk  (3U << CANFD_MEM_STAT_HOSTSIDE_MEM_ERR_LOC_Pos)
#define CANFD_MEM_STAT_TX_BLOCK_STATUS           (1U << 2U)
#define CANFD_MEM_STAT_TX_STOP_STATUS            (1U << 1U)
#define CANFD_MEM_STAT_ACPT_FLTR_ACCEPT          (1U << 0U)    /* 0-Filter enabled, 1-Filter Disabled */

/* Macros for Memory Error Stimulation 0 Register */
#define CANFD_MEM_ES0_MEM_ADDR_ERR_EN            (1U << 7U)
#define CANFD_MEM_ES0_MEM_FIRST_ERR_EN           (1U << 6U)
#define CANFD_MEM_ES0_MEM_ERR_BIT_POS_1_Msk      (3FU << 0U)

/* Macros for Memory Error Stimulation 1 Register */
#define CANFD_MEM_ES1_MEM_SECOND_ERR_EN          (1U << 6U)
#define CANFD_MEM_ES1_MEM_ERR_BIT_POS_2_Msk      (3FU << 0U)

/* Macros for Memory Error Stimulation 2 Register */
#define CANFD_MEM_ES2_MEM_ERR_NO_ERR_CNTR        (15U << 4U)
#define CANFD_MEM_ES2_MEM_ERR_EN_CNTR            (15U << 0U)

/* Macros for Memory Error Stimulation 3 Register */
#define CANFD_MEM_ES3_MEM_ERR_SIDE_Pos           (2U)
#define CANFD_MEM_ES3_MEM_ERR_SIDE_Msk           (1U << 2U)
#define CANFD_MEM_ES3_MEM_ERR_SIDE               (1U << 2U)    /* 0-Host side, 1-CAN side */
#define CANFD_MEM_ES3_MEM_ERR_LOC_Pos            (0U)
#define CANFD_MEM_ES3_MEM_ERR_LOC_Msk            (3U << 0U)
#define CANFD_MEM_ES3_MEM_ERR_LOC(x)             (x & CANFD_MEM_ES3_MEM_ERR_LOC_Msk)


/* Macros for Spatial Redundancy Configuration Register */
#define CANFD_SRCFG_ERR_EN_CANFD_CLK_DOMAIN      (1U << 4U)    /* 0-No error stimulation, 1-Error stimulation */
#define CANFD_SRCFG_ERR_EN_CANFD_HOST_CLK_DOMAIN (1U << 3U)    /* 0-No error stimulation, 1-Error stimulation */
#define CANFD_SRCFG_ERR_INTR_FLAG                (1U << 2U)
#define CANFD_SRCFG_INSTANCE_SEL                 (1U << 1U)
#define CANFD_SRCFG_EN                           (1U << 0U)

/* Macros for CAN Msg access */
#define CANFD_MSG_ESI_Pos                        (31U)
#define CANFD_MSG_ESI_Msk                        (1U << CANFD_MSG_ESI_Pos)
#define CANFD_MSG_ESI(x)                         (x << 31U)
#define CANFD_MSG_IDE_Pos                        (7U)
#define CANFD_MSG_IDE_Msk                        (1U << CANFD_MSG_IDE_Pos)
#define CANFD_MSG_IDE(x)                         ((x << CANFD_MSG_IDE_Pos) & CANFD_MSG_IDE_Msk)
#define CANFD_MSG_RTR_Pos                        (6U)
#define CANFD_MSG_RTR_Msk                        (1U << CANFD_MSG_RTR_Pos)
#define CANFD_MSG_RTR(x)                         ((x << CANFD_MSG_RTR_Pos) & CANFD_MSG_RTR_Msk)
#define CANFD_MSG_FDF_Pos                        (5U)
#define CANFD_MSG_FDF_Msk                        (1U << CANFD_MSG_FDF_Pos)
#define CANFD_MSG_FDF(x)                         ((x << CANFD_MSG_FDF_Pos) & CANFD_MSG_FDF_Msk)
#define CANFD_MSG_BRS_Pos                        (4U)
#define CANFD_MSG_BRS_Msk                        (1U << CANFD_MSG_BRS_Pos)
#define CANFD_MSG_BRS(x)                         ((x << CANFD_MSG_BRS_Pos) & CANFD_MSG_BRS_Msk)
#define CANFD_MSG_DLC_Pos                        (0U)
#define CANFD_MSG_DLC_Msk                        (15U << CANFD_MSG_DLC_Pos)
#define CANFD_MSG_DLC(x)                         (x & CANFD_MSG_DLC_Msk)

/* Macros for Bit time segments */
#define CANFD_BIT_PROP_SEG_Pos                   0U
#define CANFD_BIT_PHASE_SEG1_Pos                 8U
#define CANFD_BIT_PHASE_SEG2_Pos                 16
#define CANFD_BIT_SJW_Pos                        24U

/* Macros for Interrupt events */
#define CANFD_TX_ABORT_EVENT                     (1U << 0U)
#define CANFD_ERROR_EVENT                        (1U << 1U)
#define CANFD_TX_COMPLETE_EVENT                  (1U << 3U)
#define CANFD_RBUF_ALMOST_FULL_EVENT             (1U << 4U)
#define CANFD_RBUF_FULL_EVENT                    (1U << 5U)
#define CANFD_RBUF_OVERRUN_EVENT                 (1U << 6U)
#define CANFD_RBUF_AVAILABLE_EVENT               (1U << 7U)
#define CANFD_BUS_ERROR_EVENT                    (1U << 8U)
#define CANFD_ARBTR_LOST_EVENT                   (1U << 10U)
#define CANFD_ERROR_PASSIVE_EVENT                (1U << 12U)

/* CANFD Acceptance Filter Operation codes */
typedef enum _CANFD_ACPT_FLTR_OP
{
  CANFD_ACPT_FLTR_OP_ADD_EXACT_ID,          /* Add exact id filter */
  CANFD_ACPT_FLTR_OP_REMOVE_EXACT_ID,       /* Remove exact id filter */
  CANFD_ACPT_FLTR_OP_ADD_MASKABLE_ID,       /* Add maskable id filter */
  CANFD_ACPT_FLTR_OP_REMOVE_MASKABLE_ID     /* Remove maskable id filter */
}CANFD_ACPT_FLTR_OP;

/* CANFD Message errors*/
typedef enum _CANFD_MSG_ERROR
{
    CANFD_MSG_ERROR_NONE          = 0x0,
    CANFD_MSG_ERROR_BIT           = 0x1,
    CANFD_MSG_ERROR_FORM          = 0x2,
    CANFD_MSG_ERROR_STUFF         = 0x3,
    CANFD_MSG_ERROR_ACK           = 0x4,
    CANFD_MSG_ERROR_CRC           = 0x5
}CANFD_MSG_ERROR;

/* CANFD Bus Status */
typedef enum _CANFD_BUS_STATUS
{
    CANFD_BUS_STATUS_ON   = 0x0,
    CANFD_BUS_STATUS_OFF  = 0x1
}CANFD_BUS_STATUS;

/* CANFD Acceptance filter status */
typedef enum _CANFD_ACPT_FLTR_STATUS
{
    CANFD_ACPT_FLTR_STATUS_NONE         = 0x0,
    CANFD_ACPT_FLTR_STATUS_FREE         = 0x1,
    CANFD_ACPT_FLTR_STATUS_OCCUPIED     = 0x2
}CANFD_ACPT_FLTR_STATUS;


/* CANFD Transmit Buffer Registers' structure:
 * for Hardware register access */
typedef struct _tbuf_regs_t
{
    uint32_t    can_id;
    uint32_t    control;
    uint8_t     data[CANFD_FAST_DATA_FRAME_SIZE_MAX];     /* Maximum payload size is 64 bytes */
}tbuf_regs_t;

/* CANFD Receive Buffer Registers' structure:
 * for Hardware register access */
typedef struct _rbuf_regs_t
{
    uint32_t    can_id;
    uint8_t     control;
    uint8_t     status;
    uint8_t     reserved[2U];
    uint8_t     data[CANFD_FAST_DATA_FRAME_SIZE_MAX];     /* Maximum payload size is 64 bytes */
}rbuf_regs_t;

/* Current tx info */
typedef struct _canfd_tx_info_t
{
  uint32_t id;                          /* CAN identifier                                      */
  uint32_t frame_type       : 1;        /* frame type - Normal/Extended                        */
  uint32_t rtr              : 1;        /* Remote transmission request frame                   */
  uint32_t edl              : 1;        /* Flexible data-rate format extended data length      */
  uint32_t brs              : 1;        /* Flexible data-rate format with bitrate switch       */
  uint32_t esi              : 1;        /* Flexible data-rate format error state indicator     */
  uint32_t dlc              : 4;        /* Data length code                                    */
  uint32_t reserved         : 23;
}canfd_tx_info_t;

/* Current Rx info */
typedef struct _canfd_rx_info_t
{
  uint32_t id;                          /* CAN identifier with frame format specifier (bit 31) */
  uint32_t frame_type       : 1;        /* frame type - Normal/Extended                        */
  uint32_t rtr              : 1;        /* Remote transmission request frame                   */
  uint32_t edl              : 1;        /* Flexible data-rate format extended data length      */
  uint32_t brs              : 1;        /* Flexible data-rate format with bitrate switch       */
  uint32_t esi              : 1;        /* Flexible data-rate format error state indicator     */
  uint32_t dlc              : 4;        /* Data length code                                    */
  uint32_t status           : 8;        /* KOER and a bit to show LBMI msg                     */
  uint32_t reserved         : 15;
}canfd_rx_info_t;

/* CANFD Driver Data Transfer info */
typedef struct _canfd_transfer_t
{
    const    uint8_t                *tx_ptr;      /* Pointer to Tx buffer                        */
    uint8_t                         *rx_ptr;      /* Pointer to Rx buffer                        */
    volatile uint8_t                tx_count;     /* Current Transmission completed data count   */
    volatile uint8_t                rx_count;     /* Current Reception completed data count      */
    canfd_tx_info_t                 tx_header;    /* Tx Data header                              */
    canfd_rx_info_t                 rx_header;    /* Rx Data header                              */
}canfd_transfer_t;

/**
  \fn          static inline void canfd_reset(CANFD_Type* canfd)
  \brief       Resets CANFD instance.
  \param[in]   canfd  : Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_reset(CANFD_Type* canfd)
{
    /* Resets CANFD module*/
    canfd->CANFD_CFG_STAT |= CANFD_CFG_STAT_RESET;
}

/**
  \fn          static inline CANFD_BUS_STATUS canfd_get_bus_status(CANFD_Type* canfd)
  \brief       Fetches the current bus status
  \param[in]   canfd : Pointer to the CANFD register map
  \return      bus state - CANFD_BUS_STATUS_ON/CANFD_BUS_STATUS_OFF
*/
static inline CANFD_BUS_STATUS canfd_get_bus_status(CANFD_Type* canfd)
{
    /* Returns current bus status*/
    return (canfd->CANFD_CFG_STAT & CANFD_CFG_STAT_BUS_OFF_STATUS);
}

/**
  \fn          static inline bool canfd_tx_active(CANFD_Type* canfd)
  \brief       Fetches the msg transmission status
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      transmission status - Active/Inactive
*/
static inline bool canfd_tx_active(CANFD_Type* canfd)
{
    return ((canfd->CANFD_TCMD & CANFD_TCMD_PRIMARY_TX_EN) != 0);
}

/**
  \fn          static inline bool canfd_comm_active(CANFD_Type* canfd)
  \brief       Fetches message communication status
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      Message comm status(Comm active/Inactive)
*/
static inline bool canfd_comm_active(CANFD_Type* canfd)
{
    return (canfd->CANFD_CFG_STAT & (CANFD_CFG_STAT_TACTIVE_STATUS | CANFD_CFG_STAT_RACTIVE_STATUS));
}

/**
  \fn          static inline uint8_t canfd_get_tx_error_count(CANFD_Type* canfd)
  \brief       Fetches the latest Transmission error count
  \param[in]   canfd : Pointer to the CANFD register map
  \return      Transmission error count
*/
static inline uint8_t canfd_get_tx_error_count(CANFD_Type* canfd)
{
    return canfd->CANFD_TECNT;
}

/**
  \fn          static inline uint8_t canfd_get_rx_error_count(CANFD_Type* canfd)
  \brief       Fetches the latest Reception error count
  \param[in]   canfd : Pointer to the CANFD register map
  \return      Reception error count
*/
static inline uint8_t canfd_get_rx_error_count(CANFD_Type* canfd)
{
    return canfd->CANFD_RECNT;
}

/**
  \fn          static inline CANFD_MSG_ERROR canfd_get_last_error_code(CANFD_Type* canfd)
  \brief       Fetches the latest error occurred
  \param[in]   canfd : Pointer to the CANFD register map
  \return      last found error type
*/
static inline CANFD_MSG_ERROR canfd_get_last_error_code(CANFD_Type* canfd)
{
    return ((canfd->CANFD_EALCAP & CANFD_EALCAP_KIND_OF_ERROR_Msk)
                                    >> CANFD_EALCAP_KIND_OF_ERROR_Pos);
}

/**
  \fn          static inline void canfd_abort_tx(CANFD_Type* canfd)
  \brief       Abort the current message transmission
  \param[in]   canfd   : Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_abort_tx(CANFD_Type* canfd)
{
    /* Aborts current primary buffer transmission */
    canfd->CANFD_TCMD |= CANFD_TCMD_ABORT_PRIMARY_TX;
}

/**
  \fn          static inline bool canfd_error_passive_mode(CANFD_Type* canfd)
  \brief       Returns the passive mode status
  \param[in]   canfd   : Pointer to the CANFD register map
  \return      passive mode status
*/
static inline bool canfd_error_passive_mode(CANFD_Type* canfd)
{
    return ((canfd->CANFD_ERRINT & CANFD_ERRINT_EPASSIVE_MODE) != 0);
}

/**
  \fn          static inline bool canfd_err_warn_limit_reached(CANFD_Type* canfd)
  \brief       Returns the passive mode status
  \param[in]   canfd   : Pointer to the CANFD register map
  \return      Warning limit reached status
*/
static inline bool canfd_err_warn_limit_reached(CANFD_Type* canfd)
{
    return ((canfd->CANFD_ERRINT & CANFD_ERRINT_EWARN_LMT_RCHD) != 0);
}

/**
  \fn          static inline bool canfd_rx_msg_available(CANFD_Type* canfd)
  \brief       Returns Rx msg availability status
  \param[in]   canfd   : Pointer to the CANFD register map
  \return      Rx msg availability status
*/
static inline bool canfd_rx_msg_available(CANFD_Type* canfd)
{
    return ((canfd->CANFD_RCTRL & CANFD_RCTRL_RX_BUF_STATUS_Msk)!= 0);
}

/**
  \fn          static inline void canfd_reset_acpt_fltr(CANFD_Type* canfd)
  \brief       Resets all acceptance filters
  \param[in]   canfd  : Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_reset_acpt_fltrs(CANFD_Type* canfd)
{
    /* Disable filter */
    canfd->CANFD_ACF_EN_0 &= ~CANFD_ACF_EN_ACPT_FLTR_MAX_Msk;
}

/**
  \fn          static inline bool canfd_acpt_fltr_configured(CANFD_Type* canfd)
  \brief       Returns accpetance filters configured status
  \param[in]   canfd   : Pointer to the CANFD register map
  \return      filters configured status
*/
static inline bool canfd_acpt_fltr_configured(CANFD_Type* canfd)
{
   return ((canfd->CANFD_ACF_EN_0 & CANFD_ACF_EN_ACPT_FLTR_MAX_Msk) != 0);
}

/**
  \fn          static inline void canfd_disable_acpt_fltr(CANFD_Type* canfd, const uint8_t filter)
  \brief       Resets and disables the particular acceptance filter
  \param[in]   canfd  : Pointer to the CANFD register map
  \param[in]   filter : Acceptance filter number
  \return      none
*/
static inline void canfd_disable_acpt_fltr(CANFD_Type* canfd, const uint8_t filter)
{
    /* Disable filter */
    canfd->CANFD_ACF_EN_0 &= ~(1U << filter);
}

/**
  \fn          static inline void canfd_enable_tx_interrupts(CANFD_Type* canfd)
  \brief       Enables CANFD Tx interrupts
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_enable_tx_interrupts(CANFD_Type* canfd)
{
    /* Enables CANFD Tx interrupts */
    canfd->CANFD_RTIE     |= (CANFD_RTIE_TX_PRIMARY_INTR_EN      |
                              CANFD_RTIE_TX_SEC_INTR_EN);
}

/**
  \fn          static inline void canfd_disable_tx_interrupts(CANFD_Type* canfd)
  \brief       Disables CANFD Tx interrupts
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_disable_tx_interrupts(CANFD_Type* canfd)
{
    /* Disables CANFD Tx interrupts */
    canfd->CANFD_RTIE     &= ~(CANFD_RTIE_TX_PRIMARY_INTR_EN     |
                               CANFD_RTIE_TX_SEC_INTR_EN);
}

/**
  \fn          static inline void canfd_enable_rx_interrupts(CANFD_Type* canfd)
  \brief       Enables CANFD Rx interrupts
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_enable_rx_interrupts(CANFD_Type* canfd)
{
    /* Enables CANFD Rx interrupts */
    canfd->CANFD_RTIE     |= (CANFD_RTIE_RX_INTR_EN              |
                              CANFD_RTIE_RX_OVERRUN_INTR_EN      |
                              CANFD_RTIE_RBUF_FULL_INTR_EN       |
                              CANFD_RTIE_RBUF_ALMST_FULL_INTR_EN);
}

/**
  \fn          static inline void canfd_disable_rx_interrupts(CANFD_Type* canfd)
  \brief       Disables CANFD Rx interrupts
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_disable_rx_interrupts(CANFD_Type* canfd)
{
    /* Enables CANFD Rx interrupts */
    canfd->CANFD_RTIE     &= ~(CANFD_RTIE_RX_INTR_EN             |
                              CANFD_RTIE_RX_OVERRUN_INTR_EN      |
                              CANFD_RTIE_RBUF_FULL_INTR_EN       |
                              CANFD_RTIE_RBUF_ALMST_FULL_INTR_EN);
}


/**
  \fn          static inline void canfd_enable_error_interrupts(CANFD_Type* canfd)
  \brief       Enables CANFD error interrupts
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_enable_error_interrupts(CANFD_Type* canfd)
{
    /* Enables error interrupts */
    canfd->CANFD_RTIE     |= CANFD_RTIE_ERROR_INTR_EN;

    canfd->CANFD_ERRINT   |= (CANFD_ERRINT_EPASSIVE_INTR_EN      |
                              CANFD_ERRINT_ARBTR_LOST_INTR_EN    |
                              CANFD_ERRINT_BUS_ERROR_INTR_EN);
}

/**
  \fn          static inline void canfd_disable_error_interrupts(CANFD_Type* canfd)
  \brief       Disables CANFD error interrupts
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_disable_error_interrupts(CANFD_Type* canfd)
{
    /* Enables error interrupts */
    canfd->CANFD_RTIE     &= ~CANFD_RTIE_ERROR_INTR_EN;

    canfd->CANFD_ERRINT   &= ~(CANFD_ERRINT_EPASSIVE_INTR_EN     |
                              CANFD_ERRINT_ARBTR_LOST_INTR_EN    |
                              CANFD_ERRINT_BUS_ERROR_INTR_EN);
}

/**
  \fn          static inline void canfd_clear_interrupts(CANFD_Type* canfd)
  \brief       Clears all CANFD interrupts
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_clear_interrupts(CANFD_Type* canfd)
{
    /* Clears data and error interrupts */
    canfd->CANFD_RTIF   = 0U;
    canfd->CANFD_ERRINT &= ~CANFD_ERRINT_REG_Msk;
}

/**
  \fn          static inline void canfd_enable_normal_mode(CANFD_Type* canfd)
  \brief       Enables Normal Mode operation.
  \param[in]   canfd :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_enable_normal_mode(CANFD_Type* canfd)
{
    /* Disables the CANFD reset, Internal and External loopback*/
    canfd->CANFD_CFG_STAT &= ~(CANFD_CFG_STAT_RESET | CANFD_CFG_STAT_LOOPBACK_MODE_INTERNAL |
                              CANFD_CFG_STAT_LOOPBACK_MODE_EXTERNAL);

    /* Disables Listen only mode */
    canfd->CANFD_TCMD     &= ~CANFD_TCMD_LISTEN_ONLY_MODE;

    /* Enables CANFD Rx, Tx and error interrupts */
    canfd_enable_tx_interrupts(canfd);
    canfd_enable_rx_interrupts(canfd);
    canfd_enable_error_interrupts(canfd);
}

/**
  \fn          static inline void canfd_enable_external_loop_back_mode(CANFD_Type* canfd)
  \brief       Enables External LoopBack Mode of CANFD instance.
  \param[in]   canfd  : Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_enable_external_loop_back_mode(CANFD_Type* canfd)
{
    /* Disables the CANFD reset, Internal and External loopback*/
    canfd->CANFD_CFG_STAT &= ~(CANFD_CFG_STAT_RESET | CANFD_CFG_STAT_LOOPBACK_MODE_INTERNAL);

    /* Disables Listen only mode */
    canfd->CANFD_TCMD     &= ~CANFD_TCMD_LISTEN_ONLY_MODE;

    /* Enables CANFD Rx, Tx and error interrupts */
    canfd_enable_tx_interrupts(canfd);
    canfd_enable_rx_interrupts(canfd);
    canfd_enable_error_interrupts(canfd);

    /* Enables external loopback with Self ACK */
    canfd->CANFD_CFG_STAT |= CANFD_CFG_STAT_LOOPBACK_MODE_EXTERNAL;
    canfd->CANFD_RCTRL    |= CANFD_RCTRL_SELF_ACK;
}

/**
  \fn          static inline void canfd_enable_internal_loop_back_mode(CANFD_Type* canfd)
  \brief       Enables Internal LoopBack Mode.
  \param[in]   canfd  :  Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_enable_internal_loop_back_mode(CANFD_Type* canfd)
{
    /* Disables the CANFD reset, Internal and External loopback*/
    canfd->CANFD_CFG_STAT  &= ~(CANFD_CFG_STAT_RESET | CANFD_CFG_STAT_LOOPBACK_MODE_EXTERNAL);

    /* Disables Listen only mode */
    canfd->CANFD_TCMD      &= ~CANFD_TCMD_LISTEN_ONLY_MODE;

    /* Enables CANFD Rx, Tx and error interrupts */
    canfd_enable_tx_interrupts(canfd);
    canfd_enable_rx_interrupts(canfd);
    canfd_enable_error_interrupts(canfd);

    /* Enables internal loopback with */
    canfd->CANFD_CFG_STAT  |= CANFD_CFG_STAT_LOOPBACK_MODE_INTERNAL;
}

/**
  \fn          static inline void canfd_enable_listen_only_mode(CANFD_Type* canfd)
  \brief       Enables Listen only Mode of CANFD instance.
  \param[in]   canfd  : Pointer to the CANFD register map
  \return      none
*/
static inline void canfd_enable_listen_only_mode(CANFD_Type* canfd)
{
    /* Disables the CANFD reset, Internal and External loopback*/
    canfd->CANFD_CFG_STAT &= ~(CANFD_CFG_STAT_RESET | CANFD_CFG_STAT_LOOPBACK_MODE_INTERNAL |
                               CANFD_CFG_STAT_LOOPBACK_MODE_EXTERNAL);

    /* Disables CANFD Tx interrupts */
    canfd_disable_tx_interrupts(canfd);

    /* Enables CANFD Rx and error interrupts */
    canfd_enable_rx_interrupts(canfd);
    canfd_enable_error_interrupts(canfd);

    /* Enables Listen only mode */
    canfd->CANFD_TCMD   |= CANFD_TCMD_LISTEN_ONLY_MODE;
}

/**
  \fn          void canfd_set_nominal_bit_time(CANFD_Type* canfd,
  \                       const uint32_t bitrate_seg, const uint8_t prescaler)
  \brief       Sets the slow speed bit-timing of CANFD instance.
  \param[in]   canfd       : Pointer to the CANFD register map
  \param[in]   bitrate_seg : Segments - Propagation, Sampling
  \param[in]   prescaler   : Prescaler value
  \return      none
*/
void canfd_set_nominal_bit_time(CANFD_Type* canfd,
               const uint32_t bitrate_seg, const uint8_t prescaler);

/**
  \fn          void canfd_set_fd_bit_time(CANFD_Type* canfd,
  \                  const uint32_t bitrate_seg, const uint8_t prescaler)
  \brief       Sets the fast speed bit-timing of CANFD instance.
  \param[in]   canfd       : Pointer to the CANFD register map
  \param[in]   bitrate_seg : Segments - Propagation, Sampling
  \param[in]   prescaler   : Prescaler value
  \return      none
*/
void canfd_set_fd_bit_time(CANFD_Type* canfd,
          const uint32_t bitrate_seg, const uint8_t prescaler);

/**
  \fn          void canfd_enable_acpt_fltr(CANFD_Type* canfd, const uint8_t filter,
                                     const uint32_t ac_code, uint32_t ac_mask, const CANFD_ACPT_FLTR_OP op_code
  \brief       Configures and enables the particular acceptance filter.
  \param[in]   canfd   : Pointer to the CANFD register map
  \param[in]   filter  : Acceptance filter number
  \param[in]   ac_code : Acceptance code
  \param[in]   ac_mask : Acceptance mask
  \param[in]   op_code : Acceptance filter operation code
  \return      none
*/
void canfd_enable_acpt_fltr(CANFD_Type* canfd, const uint8_t filter,
                            const uint32_t ac_code, uint32_t ac_mask, const CANFD_ACPT_FLTR_OP op_code);

/**
  \fn          CANFD_ACPT_FLTR_STATUS canfd_get_acpt_fltr_status(CANFD_Type* canfd,
  \                                                              uint8_t filter)
  \brief       Retrieves whether the filter is free or occupied.
  \param[in]   canfd  : Pointer to the CANFD register map
  \param[in]   filter : Acceptance filter number
  \return      status of the filter (Free/Occupied)
*/
CANFD_ACPT_FLTR_STATUS canfd_get_acpt_fltr_status(CANFD_Type* canfd, const uint8_t filter);

/**
  \fn          void canfd_get_acpt_fltr_data(CANFD_Type* canfd, const uint8_t filter,
                                             uint32_t *ac_code, uint32_t *ac_mask, const CANFD_ACPT_FLTR_OP op_code)
  \brief       Retrieves the acceptance filter data.
  \param[in]   canfd   : Pointer to the CANFD register map
  \param[in]   filter  : Acceptance filter number
  \param[in]   ac_code : Acceptance code
  \param[in]   ac_mask : Acceptance mask
  \param[in]   op_code : Acceptance filter operation code
  \return      none
*/
void canfd_get_acpt_fltr_data(CANFD_Type* canfd, const uint8_t filter,
                              uint32_t *ac_code, uint32_t *ac_mask, const CANFD_ACPT_FLTR_OP op_code);

/**
  \fn          void canfd_setup_tx_retrans(CANFD_Type* canfd, const bool enable)
  \brief       Enables/Disables the Tx msg retransmission
  \param[in]   canfd  : Pointer to the CANFD register map
  \param[in]   enable : Command to enable/disable msg retransmission
  \return      none
*/
void canfd_setup_tx_retrans(CANFD_Type* canfd, const bool enable);

/**
  \fn          void canfd_setup_tx_delay_comp(CANFD_Type* canfd,
                            const uint8_t offset, const bool enable)
  \brief       Enables/Disables the Tx delay compensation
  \param[in]   canfd  : Pointer to the CANFD register map
  \param[in]   offset : Secondary sampling point offest value
  \param[in]   enable : Command to enable/disable TDC
  \return      none
*/
void canfd_setup_tx_delay_comp(CANFD_Type* canfd,
                      const uint8_t offset, const bool enable);

/**
  \fn          void canfd_set_err_warn_limit(CANFD_Type* canfd,
                                             const uint8_t ewl)
  \brief       Configures Warning limits for errors
  \param[in]   canfd : Pointer to the CANFD register map
  \param[in]   ewl   : Limit value for Error warning
  \return      none
*/
void canfd_set_err_warn_limit(CANFD_Type* canfd, const uint8_t ewl);

/**
  \fn          void canfd_send(CANFD_Type* canfd, canfd_tx_info_t tx_header,
  \                            const uint8_t *data, const uint8_t size)
  \brief       Prepares and transmits the message
  \param[in]   canfd      : Pointer to the CANFD register map
  \param[in]   tx_header  : Header of tx message
  \param[in]   data       : Message payload
  \return      none
*/
void canfd_send(CANFD_Type* canfd, const canfd_tx_info_t tx_header,
                          const uint8_t *data, const uint8_t size);

/**
  \fn          void canfd_receive(CANFD_Type* canfd,
  \                             canfd_data_transfer_t *dest_data))
  \brief       Fetches the data from Rx buffer
  \param[in]   canfd         : Pointer to the CANFD register map
  \param[in]   dest_data     : Destination Data pointer
  \return      none
*/
void canfd_receive(CANFD_Type* canfd, canfd_transfer_t *dest_data);

/**
  \fn          void canfd_clear_interrupt(CANFD_Type* canfd, const uint32_t event)
  \brief       Clears the interrupt
  \param[in]   canfd : Pointer to the CANFD register map
  \param[in]   event : Interrupt event
  \return      none
*/
void canfd_clear_interrupt(CANFD_Type* canfd, const uint32_t event);

/**
  \fn          uint32_t canfd_irq_handler(CANFD_Type* canfd)
  \brief       Returns the interrupt event
  \param[in]   canfd  : Pointer to the CANFD register map
  \return      CANFD interrupt event
*/
uint32_t canfd_irq_handler(CANFD_Type* canfd);

#endif /* CANFD_H_ */
