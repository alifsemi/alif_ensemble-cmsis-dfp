/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef I2C_H_
#define I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "soc.h"

/*!< FIFO Depth for Tx & Rx  */
#define I2C_FIFO_DEPTH                       32

#define I2C_SLAVE_10BIT_ADDR_MODE            (1 << 3)  /* 10 bit address mode for slave mode */
#define I2C_MASTER_10BIT_ADDR_MODE           (1 << 12) /* 10 bit address mode for master mode */

/* Enable I2C */
#define I2C_IC_ENABLE_I2C_ENABLE             (1)

/* Disable I2C */
#define I2C_IC_ENABLE_I2C_DISABLE            (0)

/* Field of IC_ENABLE register */
#define I2C_IC_ENABLE_ABORT                  (1 << 1)
#define I2C_IC_SDA_STUCK_RECOVERY_ENABLE     (1 << 3)

/* Field of IC_ENABLE_STATUS register*/
#define I2C_ENABLE_STATUS_IC_EN              (1 << 0)

/* i2c Status Register Fields. */
#define I2C_IC_STATUS_ACTIVITY               (0x01) /* (1 << 0) */
#define I2C_IC_STATUS_TRANSMIT_FIFO_NOT_FULL (0x02) /* (1 << 1) */
#define I2C_IC_STATUS_TFE                    (0x04) /* (1 << 2) */
#define I2C_IC_STATUS_RECEIVE_FIFO_NOT_EMPTY (0x08) /* (1 << 3) */
#define I2C_IC_STATUS_RFF                    (0x10) /* (1 << 4) */
#define I2C_IC_STATUS_MASTER_ACT             (0x20) /* (1 << 5) */
#define I2C_IC_STATUS_SLAVE_ACT              (0x40) /* (1 << 6) */
#define I2C_IC_STATUS_SDA_STUCK_NOT_RECOVERED (1 << 11)

/* Perform a Restart/Stop request */
#define I2C_IC_DATA_CMD_RESTART              (1 << 10)
#define I2C_IC_DATA_CMD_STOP                 (1 << 9)
/* Perform a write request */
#define I2C_IC_DATA_CMD_WRITE_REQ            (0)
/* Perform a read request */
#define I2C_IC_DATA_CMD_READ_REQ             (1 << 8)

/* i2c master 10 bit addr support */
#define I2C_IC_CON_10BITADDR_MASTER          (1 << 4)

/* Speed modes of IC_CON */
#define I2C_IC_CON_SPEED_MASK                (0x6)
#define I2C_IC_CON_SPEED_STANDARD            (0x2)
#define I2C_IC_CON_SPEED_FAST                (0x4)
#define I2C_IC_CON_SPEED_HIGH                (0x6)

/* Working mode of IC_CON */
#define I2C_IC_CON_MST_SLV_MODE_MASK         (0x41)
#define I2C_IC_CON_ENABLE_MASTER_MODE        (0x41)
#define I2C_IC_CON_ENA_SLAVE_MODE            (0)

#define I2C_HS_MADDR_I2C_HS_MAR_MASK                (0x7 << 0)

/* I2C interrupt control */
#define I2C_IC_INT_DISABLE_ALL               (0x0)

/* Interrupt Register Fields */
#define I2C_IC_INTR_STAT_GEN_CALL            (1 << 11)
#define I2C_IC_INTR_STAT_START_DET           (1 << 10)
#define I2C_IC_INTR_STAT_STOP_DET            (1 << 9)
#define I2C_IC_INTR_STAT_ACTIVITY            (1 << 8)
#define I2C_IC_INTR_STAT_RX_DONE             (1 << 7)

#define I2C_IC_INTR_STAT_TX_ABRT             (1 << 6) /* raw interrupt status */
#define I2C_IC_INTR_STAT_RD_REQ              (1 << 5)
#define I2C_IC_INTR_STAT_TX_EMPTY            (1 << 4)
#define I2C_IC_INTR_STAT_TX_OVER             (1 << 3) /* raw interrupt status */

#define I2C_IC_INTR_STAT_RX_FULL             (1 << 2)
#define I2C_IC_INTR_STAT_RX_OVER             (1 << 1) /* raw interrupt status */
#define I2C_IC_INTR_STAT_RX_UNDER            (1 << 0) /* raw interrupt status */

/* Interrupt enable mask as master */
#define I2C_IC_INT_MST_TX_ENABLE                                                                   \
    (I2C_IC_INTR_STAT_TX_EMPTY | I2C_IC_INTR_STAT_TX_OVER | I2C_IC_INTR_STAT_TX_ABRT |             \
     I2C_IC_INTR_STAT_STOP_DET)

#define I2C_IC_INT_DMA_MST_TX_ENABLE                                                               \
    (I2C_IC_INTR_STAT_TX_OVER | I2C_IC_INTR_STAT_TX_ABRT | I2C_IC_INTR_STAT_STOP_DET)

#define I2C_IC_INT_MST_RX_ENABLE                                                                   \
    (I2C_IC_INTR_STAT_TX_EMPTY | I2C_IC_INTR_STAT_RX_FULL | I2C_IC_INTR_STAT_RX_OVER |             \
     I2C_IC_INTR_STAT_RX_UNDER | I2C_IC_INTR_STAT_TX_ABRT | I2C_IC_INTR_STAT_STOP_DET)

#define I2C_IC_INT_DMA_MST_RX_ENABLE                                                               \
    (I2C_IC_INTR_STAT_RX_OVER | I2C_IC_INTR_STAT_RX_UNDER |                                        \
     I2C_IC_INTR_STAT_TX_ABRT | I2C_IC_INTR_STAT_STOP_DET)
/* Interrupt enable mask as slave */
#define I2C_IC_INT_SLV_TX_ENABLE                                                                   \
    (I2C_IC_INTR_STAT_RD_REQ | I2C_IC_INTR_STAT_TX_ABRT | I2C_IC_INTR_STAT_STOP_DET)

#define I2C_IC_INT_DMA_SLV_TX_ENABLE                                                               \
    (I2C_IC_INTR_STAT_RD_REQ | I2C_IC_INTR_STAT_TX_ABRT | I2C_IC_INTR_STAT_STOP_DET)

#define I2C_IC_INT_SLV_RX_ENABLE                                                                   \
    (I2C_IC_INTR_STAT_RX_FULL | I2C_IC_INTR_STAT_RX_OVER | I2C_IC_INTR_STAT_RX_UNDER |             \
     I2C_IC_INTR_STAT_STOP_DET)

#define I2C_IC_INT_DMA_SLV_RX_ENABLE                                                               \
    (I2C_IC_INTR_STAT_RX_OVER | I2C_IC_INTR_STAT_RX_UNDER | I2C_IC_INTR_STAT_STOP_DET)

/* I2C_TX_ABRT_SOURCE Register Bit Fields */
#define I2C_IC_TX_ABRT_7B_ADDR_NOACK   (1 << 0)
#define I2C_IC_TX_ABRT_10ADDR1_NOACK   (1 << 1)
#define I2C_IC_TX_ABRT_10ADDR2_NOACK   (1 << 2)

#define I2C_IC_TX_ABRT_TXDATA_NOACK    (1 << 3)

#define I2C_IC_TX_ABRT_GCALL_NOACK     (1 << 4)
#define I2C_IC_TX_ABRT_GCALL_READ      (1 << 5)
#define I2C_IC_TX_ABRT_HS_ACKDET       (1 << 6)
#define I2C_IC_TX_ABRT_SBYTE_ACKDET    (1 << 7)
#define I2C_IC_TX_ABRT_HS_NORSTRT      (1 << 8)
#define I2C_IC_TX_ABRT_SBYTE_NORSTRT   (1 << 9)
#define I2C_IC_TX_ABRT_10B_RD_NORSTRT  (1 << 10)
#define I2C_IC_TX_ABRT_MASTER_DIS      (1 << 11)
#define I2C_IC_TX_ABRT_ARB_LOST        (1 << 12)
#define I2C_IC_TX_ABRT_SLVFLUSH_TXFIFO (1 << 13)
#define I2C_IC_TX_ABRT_SLV_ARBLOST     (1 << 14)
#define I2C_IC_TX_ABRT_SLVRD_INTX      (1 << 15)
#define I2C_IC_TX_ABRT_USER_ABRT            (1 << 16)
#define I2C_IC_TX_ABRT_SDA_STUCK_AT_LOW     (1 << 17)
#define I2C_IC_TX_ABRT_DEVICE_NOACK         (1 << 18)
#define I2C_IC_TX_ABRT_DEVICE_SLVADDR_NOACK (1 << 19)
#define I2C_IC_TX_ABRT_DEVICE_WRITE         (1 << 20)

/* Combined bits for i2c abort source */
#define I2C_MST_ABRT_ADDR_NOACK                                                                    \
        (I2C_IC_TX_ABRT_7B_ADDR_NOACK | I2C_IC_TX_ABRT_10ADDR1_NOACK |                             \
         I2C_IC_TX_ABRT_10ADDR2_NOACK | I2C_IC_TX_ABRT_DEVICE_SLVADDR_NOACK)
#define I2C_MST_ABRT_DATA_NOACK (I2C_IC_TX_ABRT_TXDATA_NOACK)

#define I2C_ABRT_ARBITRATION_LOST    (I2C_IC_TX_ABRT_ARB_LOST | I2C_IC_TX_ABRT_SLV_ARBLOST)
#define I2C_MST_ABRT_GCALL            (I2C_IC_TX_ABRT_GCALL_NOACK | I2C_IC_TX_ABRT_GCALL_READ)
#define I2C_MST_ABRT_UNEXPECTED_ACK_DET   (I2C_IC_TX_ABRT_HS_ACKDET | I2C_IC_TX_ABRT_SBYTE_ACKDET)
#define I2C_MST_ABRT_NORSTRT                                                                       \
    (I2C_IC_TX_ABRT_HS_NORSTRT | I2C_IC_TX_ABRT_SBYTE_NORSTRT | I2C_IC_TX_ABRT_10B_RD_NORSTRT)

/* Enabling of I2C Tx and Rx transfer through DMA */
#define I2C_DMACR_TX_DMA_ENABLE (1 << 1)
#define I2C_DMACR_RX_DMA_ENABLE (1 << 0)

/* register configuration
 * -------------------------------------------------------------------------------------------------------------
 */
#ifndef I2C_ALLOW_RESTART
#define I2C_ALLOW_RESTART (1) /* allow restart configuration */
#endif

#ifndef I2C_DYNAMIC_TAR_UPDATE_SUPPORT
#define I2C_DYNAMIC_TAR_UPDATE_SUPPORT (0) /* Dynamic target address update support */
#endif

/* Fields of IC_CON register */
/*  I2C IP Config Dependencies. */
#if I2C_ALLOW_RESTART
#define I2C_IC_CON_MASTER_RESTART_EN (1 << 5)
#else
#define I2C_IC_CON_MASTER_RESTART_EN (0x00)
#endif

#define I2C_SPECIAL_START_BYTE 0
#if I2C_SPECIAL_START_BYTE
#define I2C_IC_TAR_SPECIAL     (1 << 11)
#define I2C_IC_TAR_GC_OR_START (1 << 10)
#else
#define I2C_IC_TAR_SPECIAL     (0x00)
#define I2C_IC_TAR_GC_OR_START (0x00)
#endif

/* register configuration
 * ---------------------------------------------------------------------------------------- */
#define I2C_IC_TAR_7BIT_ADDR_MASK            (0x7F) /* 7bit  I2C address mask for target address register  */
#define I2C_IC_SAR_7BIT_ADDR_MASK            (0x7F) /* 7bit  I2C address mask for slave  address register  */
#define I2C_IC_TAR_10BIT_ADDR_MASK           (0x3FF) /* 10bit I2C address mask for target address register \
                                                      */
#define I2C_IC_SAR_10BIT_ADDR_MASK           (0x3FF) /* 10bit I2C address mask for slave  address register \
                                                      */
#define I2C_FS_SPIKE_LENGTH_NS               (50)
#define I2C_HS_SPIKE_LENGTH_NS               (10)

/* The below macros calculations are wrt configuration
 * IC_CLK_FREQ_OPTIMIZATION = 1
 */
/* Min SCL High Time is 5 cycles. High Time = HCNT + spike_len + 3 */
#define I2C_ENSURE_MIN_SCL_HCNT(x, spk_len)                                                        \
    ((((x) + (spk_len) + 3) < 5) ? (5 - ((spk_len) + 3)) : (x))

/* Min SCL Low Time is 6 cycles */
#define I2C_ENSURE_MIN_SCL_LCNT(x)           (((x) < 6) ? 6 : (x))

#define I2C_MIN_SS_HIGH_TIME_NS              (4400)
#define I2C_MIN_SS_LOW_TIME_NS               (5200)

#define I2C_MIN_FS_HIGH_TIME_NS              (790)
#define I2C_MIN_FS_LOW_TIME_NS               (1600)

#define I2C_MIN_FS_PLUS_HIGH_TIME_NS         (290)
#define I2C_MIN_FS_PLUS_LOW_TIME_NS          (550)

/* For 3.4 MHz SCL period is 290ns.
 * As per I2C Spec:
 * SCL High period = HCNT(60) + Spike Len (10) + 30 (Offset) = 100ns
 * SCL Low period  = LCNT = 190ns
 */
#define I2C_MIN_HS_HIGH_TIME_NS              (60)
#define I2C_MIN_HS_LOW_TIME_NS               (190)

/* Macros for write-read mode */
#define I2C_WRITE_READ_MODE_EN               0x80U
#define I2C_WRITE_READ_TAR_REG_ADDR_SIZE_Msk 0xFU
#define I2C_WRITE_READ_TAR_REG_ADDR_SIZE_Pos 0x0U
#define I2C_WRITE_READ_TAR_REG_ADDR_SIZE(x)                                                        \
    (x & I2C_WRITE_READ_TAR_REG_ADDR_SIZE_Msk >> I2C_WRITE_READ_TAR_REG_ADDR_SIZE_Pos)

/* I2C Bus possible speed modes */
typedef enum i2c_speed_mode {
    I2C_SPEED_STANDARD =
        1, /* Bidirectional, Standard-mode (Sm), with a bit rate up to 100 kbit/s               */
    I2C_SPEED_FAST = 2,    /* Bidirectional, Fast-mode (Fm), with a bit rate up to 400 kbit/s    */
    I2C_SPEED_FASTPLUS = 3, /* Bidirectional, Fast-mode Plus (Fm+), with a bitrate up to 1Mbit/s */
    I2C_SPEED_HIGH     = 4 /* Bidirectional, High-Speed-mode (HS), with a bitrate up to 3.4Mbit/s */
} i2c_speed_mode_t;


/* I2C Addressing Mode */
typedef enum i2c_address_mode {
    I2C_7BIT_ADDRESS  = 0, /* Use 7bit address mode  */
    I2C_10BIT_ADDRESS = 1  /* Use 10bit address mode */
} i2c_address_mode_t;

/* I2C transfer state */
typedef enum _I2C_XFER_STATE {
    I2C_XFER_NONE   = 0, /* Transfer state none      */
    I2C_XFER_MST_TX = 1, /* Transfer state master tx */
    I2C_XFER_MST_RX = 2, /* Transfer state master rx */
    I2C_XFER_SLV_TX = 3, /* Transfer state slave tx  */
    I2C_XFER_SLV_RX = 4  /* Transfer state slave rx  */
} I2C_XFER_STATE;

/**
 * enum I2C_XFER_STS.
 * Status of an ongoing I2C transfer.
 */
typedef enum _I2C_XFER_EVENT{
    I2C_XFER_EVENT_NONE             = 0,         /* Xfer event: None                    */
    I2C_XFER_EVENT_DONE             = (1 << 0),  /* Xfer event: Done (success)          */
    I2C_XFER_EVENT_GCALL            = (1 << 1),  /* Xfer event: General call            */
    I2C_XFER_EVENT_BUS_CLEAR        = (1 << 2),  /* Xfer event: Bus clear               */
    I2C_XFER_EVENT_INCOMPLETE       = (1 << 3),  /* Xfer event: Incomplete              */
    I2C_XFER_EVENT_ADDR_NOACK       = (1 << 4),  /* Xfer event: Slave address no ack    */
    I2C_XFER_EVENT_GCALL_ERR        = (1 << 5),  /* Xfer event: Error in General call   */
    I2C_XFER_EVENT_UNEXPECTED_ACK   = (1 << 6),  /* Xfer event: Unexpected ack rcvd: either for HS mode or Start byte */
    I2C_XFER_EVENT_NO_RESTART       = (1 << 7),  /* Xfer event: Current comm requires Restart mode but it's disabled  */
    I2C_XFER_EVENT_MASTER_DIS       = (1 << 8),  /* Xfer event: Current comm needs master mode but it's disabled      */
    I2C_XFER_EVENT_ARBITRATION_LOST = (1 << 9),  /* Xfer event: Arbitration lost        */
    I2C_XFER_EVENT_TX_FIFO_FLUSH    = (1 << 10), /* Xfer event: Flush the previous (stale) tx fifo as master sent fresh read cmd */
    I2C_XFER_EVENT_RX_IN_TX_MODE    = (1 << 11), /* Xfer event: Read command sent in Tx mode  */
    I2C_XFER_EVENT_USER_ABORT       = (1 << 12), /* Xfer event: User abort                    */
    I2C_XFER_EVENT_SDA_STUCK_AT_LOW = (1 << 13), /* Xfer event: SDA is stuck at low for IC_SDA_STUCK_AT_LOW_TIMEOUT   */
    I2C_XFER_EVENT_DEV_ID_NOACK     = (1 << 14), /* Xfer event: No ack for Device ID transfer */
    I2C_XFER_EVENT_DEV_ID_WRITE     = (1 << 15), /* Xfer event: Some data available in Tx FIFO during Device ID communication */
    I2C_XFER_EVENT_UNDEF_TX_ABORT   = (1 << 16), /* Xfer event: Undefined Tx abort            */
} I2C_XFER_EVENT;

/* i2c Transfer Information (Run-Time) */
typedef struct i2c_transfer_info {
    bool xfer_pending; /* Transfer pending (no STOP) pending for interrupt only                   */
    const uint8_t    *tx_buf;       /* Pointer to out data buffer       */
    uint32_t          tx_total_num; /* Total number of data to be send */
    volatile uint32_t tx_curr_cnt;  /* current Number of data sent from total num  */
    uint8_t *rx_buf; /* Pointer to in data buffer                                               */
    uint32_t rx_total_num;              /* Total number of data to be received              */
    volatile uint32_t rx_curr_cnt;      /* Number of data received      */
    volatile uint32_t rx_curr_tx_index; /* current index Number which needs to send while receive.
                                         */
    volatile uint32_t curr_cnt;  /* common current count update in ARM_I2C_GetDataCount function  */
    volatile uint32_t tx_over;   /* i2c tx overflow count   */
    volatile uint32_t rx_over;   /* i2c rx overflow count   */
    volatile bool     abort;     /* i2c transfer abort */
    volatile I2C_XFER_STATE curr_stat; /* \ref I2C_XFER_STATE "current working state for i2c
                                              device"          */
    volatile bool     cmd_bus_clr;        /* user command to clear bus */
    volatile I2C_XFER_EVENT evt_sts;  /* \ref to I2C_XFER_EVENT for data transfer event status  */
    volatile bool                wr_mode; /* write-read mode */
} i2c_transfer_info_t;

/**
 * @brief   Enable i2c device
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_enable(I2C_Type *i2c)
{
    i2c->I2C_ENABLE = I2C_IC_ENABLE_I2C_ENABLE;

    while (!(i2c->I2C_ENABLE_STATUS & I2C_ENABLE_STATUS_IC_EN)) {
    }
}

/**
 * @brief   Disable i2c device
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_disable(I2C_Type *i2c)
{
    i2c->I2C_ENABLE = I2C_IC_ENABLE_I2C_DISABLE;

    while (i2c->I2C_ENABLE_STATUS & I2C_ENABLE_STATUS_IC_EN) {
    }
}

/**
 * @brief   Recover the I2C SDA stuck at low
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_master_recover_sda(I2C_Type *i2c)
{
    i2c->I2C_ENABLE |= I2C_IC_SDA_STUCK_RECOVERY_ENABLE;
}

/**
 * @brief   Initiate transfer abort (sends STOP on bus)
 * @note    Generates TX_ABRT with USER_ABRT source
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_master_abort(I2C_Type *i2c)
{
    i2c->I2C_ENABLE |= I2C_IC_ENABLE_ABORT;
}

/**
 * @brief   Sets I2C bus speed
 * @note    none
 * @param   i2c   : Pointer to i2c register map
 * @param   speed : Bus speed
 * @retval  none
 */
static inline void i2c_set_bus_speed(I2C_Type *i2c, const uint8_t speed)
{
    i2c_disable(i2c);
    i2c->I2C_CON = ((i2c->I2C_CON & ~(I2C_IC_CON_SPEED_MASK)) | speed);
    i2c_enable(i2c);
}

/**
 * @brief   read data buffer's address
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  Address of I2C data buffer
 */
static inline volatile void *i2c_get_data_addr(I2C_Type *i2c)
{
    return ((volatile void *) &i2c->I2C_DATA_CMD);
}

/**
 * @brief   Set High Speed master address
 * @note    none
 * @param   i2c  : Pointer to i2c register map
 * @param   addr : high speed address
 * @retval  None
 */
static inline void i2c_master_set_hs_maddr(I2C_Type *i2c, uint32_t addr)
{
    i2c_disable(i2c);
    /* Set High Speed Master address */
    i2c->I2C_HS_MADDR = (addr & I2C_HS_MADDR_I2C_HS_MAR_MASK);
    i2c_enable(i2c);
}

/**
 * @brief   read data from RX FiFo Buffer
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  received data (8-bit)
 */
static inline uint8_t i2c_read_data_from_buffer(I2C_Type *i2c)
{
    return (i2c->I2C_DATA_CMD) & 0xFFU;
}

/**
 * @brief   enable(unmask) i2c interrupt (0-mask 1-unmask)
 * @note    none
 * @param   i2c  : Pointer to i2c register map
 * @param   mask : interrupt register bits which needs to be enable
 * @retval  none
 */
static inline void i2c_unmask_interrupt(I2C_Type *i2c, uint32_t mask)
{
    i2c->I2C_INTR_MASK |= mask;
}

/**
 * @brief   disable(mask) i2c interrupt 0-mask  1-unmask
 * @note    none
 * @param   i2c  : Pointer to i2c register map
 * @param   mask : interrupt register bits which needs to be disable
 * @retval  none
 */
static inline void i2c_mask_interrupt(I2C_Type *i2c, uint32_t mask)
{
    i2c->I2C_INTR_MASK &= ~mask;
}

/**
 * @brief   clear all combined and individual i2c interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_clear_all_interrupt(I2C_Type *i2c)
{
    /* clear all combined and individual interrupt. */
    (void) i2c->I2C_CLR_INTR;
}

/**
 * @brief   Enable master tx interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_master_enable_tx_interrupt(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_MST_TX_ENABLE);
}

/**
 * @brief   Enable master rx interrupt
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_master_enable_rx_interrupt(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_MST_RX_ENABLE);
}

/**
 * @brief   Disable master tx interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_master_disable_tx_interrupt(I2C_Type *i2c)
{
    i2c_mask_interrupt(i2c, I2C_IC_INT_MST_TX_ENABLE);
}

/**
 * @brief   Disable master rx interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_master_disable_rx_interrupt(I2C_Type *i2c)
{
    i2c_mask_interrupt(i2c, I2C_IC_INT_MST_RX_ENABLE);
}

/**
 * @brief   Enable slave tx interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_slave_enable_tx_interrupt(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_SLV_TX_ENABLE);
}

/**
 * @brief   Enable slave rx interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_slave_enable_rx_interrupt(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_SLV_RX_ENABLE);
}

/**
 * @brief   Disable slave tx interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_slave_disable_tx_interrupt(I2C_Type *i2c)
{
    i2c_mask_interrupt(i2c, I2C_IC_INT_SLV_TX_ENABLE);
}

/**
 * @brief   Disable slave rx interrupt
 * @note    none
 * @param   i2c : Pointer to i2c register map
 * @retval  none
 */
static inline void i2c_slave_disable_rx_interrupt(I2C_Type *i2c)
{
    i2c_mask_interrupt(i2c, I2C_IC_INT_SLV_RX_ENABLE);
}

/**
 * @brief   check whether Restart Condition is enabled in I2C Master
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  1 Restart enabled, 0 not enabled
 */
static inline bool i2c_master_check_restart_cond(I2C_Type *i2c)
{
    return ((i2c->I2C_CON & I2C_IC_CON_MASTER_RESTART_EN) != 0);
}

/**
 * @brief   Enables Restart condition for I2C Master
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  None
 */
static inline void i2c_master_enable_restart_cond(I2C_Type *i2c)
{
    i2c->I2C_CON |= I2C_IC_CON_MASTER_RESTART_EN;
}

/**
 * @brief   Sets the Tx FIFO threshold value
 * @note    none
 * @param   i2c       : Pointer to i2c register map
 * @param   threshold : Tx Fifo threshold value
 * @retval  None
 */
static inline void i2c_set_tx_threshold(I2C_Type *i2c, const uint8_t threshold)
{
    i2c->I2C_TX_TL = threshold;
}

/**
 * @brief   Sets the Rx FIFO threshold value
 * @note    none
 * @param   i2c       : Pointer to i2c register map
 * @param   threshold : Rx Fifo threshold value
 * @retval  None
 */
static inline void i2c_set_rx_threshold(I2C_Type *i2c, const uint8_t threshold)
{
    i2c->I2C_RX_TL = threshold;
}

/**
 * @brief   Enables I2C Tx DMA channel
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  None
 */
static inline void i2c_enable_tx_dma(I2C_Type *i2c)
{
    i2c->I2C_DMA_CR |= I2C_DMACR_TX_DMA_ENABLE;
}

/**
 * @brief   Disables I2C Tx DMA channel
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  None
 */
static inline void i2c_disable_tx_dma(I2C_Type *i2c)
{
    i2c->I2C_DMA_CR &= (~I2C_DMACR_TX_DMA_ENABLE);
}

/**
 * @brief   Enables I2C Rx DMA channel
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  None
 */
static inline void i2c_enable_rx_dma(I2C_Type *i2c)
{
    i2c->I2C_DMA_CR |= I2C_DMACR_RX_DMA_ENABLE;
}

/**
 * @brief   Disables I2C Rx DMA channel
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  None
 */
static inline void i2c_disable_rx_dma(I2C_Type *i2c)
{
    i2c->I2C_DMA_CR &= (~I2C_DMACR_RX_DMA_ENABLE);
}

/**
 * @brief   Returns I2C Rx DMA enable status
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  true if RX DMA is enabled, false otherwise
 */
static inline bool i2c_is_rx_dma_enable(I2C_Type *i2c)
{
    return ((i2c->I2C_DMA_CR & I2C_DMACR_RX_DMA_ENABLE) != 0);
}

/**
 * @brief   Returns I2C Tx DMA enable status
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  true if TX DMA is enabled, false otherwise
 */
static inline bool i2c_is_tx_dma_enable(I2C_Type *i2c)
{
    return ((i2c->I2C_DMA_CR & I2C_DMACR_TX_DMA_ENABLE) != 0);
}

/**
 * @brief       Set DMA Transmit data level
 * @param       i2c : Pointer to the I2C register map
 * @retval      none
 */
static inline void i2c_set_dma_tx_level(I2C_Type *i2c, uint8_t data_level)
{
    i2c->I2C_DMA_TDLR = data_level;
}

/**
 * @brief   Set DMA Receive data level
 * @param   i2c : Pointer to the I2C register map
 * @retval  none
 */
static inline void i2c_set_dma_rx_level(I2C_Type *i2c, uint8_t data_level)
{
    i2c->I2C_DMA_RDLR = data_level;
}

/**
 * @brief
 * @param    i2c : Pointer to the I2C register map
 * @retval   none
 */
static inline void i2c_enable_dma_master_tx(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_DMA_MST_TX_ENABLE);
}

/**
 * @brief
 * @param    i2c : Pointer to the I2C register map
 * @retval   none
 */
static inline void i2c_enable_dma_master_rx(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_DMA_MST_RX_ENABLE);
}

/**
 * @brief    Set i2c slave for DMA receive
 * @param    i2c : Pointer to the I2C register map
 * @retval   none
 */
static inline void i2c_enable_dma_slave_tx(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_DMA_SLV_TX_ENABLE);
}

/**
 * @brief    Set i2c slave for DMA receive
 * @param    i2c : Pointer to the I2C register map
 * @retval   none
 */
static inline void i2c_enable_dma_slave_rx(I2C_Type *i2c)
{
    i2c_unmask_interrupt(i2c, I2C_IC_INT_DMA_SLV_RX_ENABLE);
}

/**
 * @brief   Check whether the TX FIFO is completely empty.
 * @note    Reads IC_STATUS.TFE.
 * @param   i2c    : Pointer to i2c register map
 * @retval  true   if the TX FIFO is empty, false otherwise
 */
static inline bool i2c_tx_fifo_empty(I2C_Type *i2c)
{
    return ((i2c->I2C_STATUS & I2C_IC_STATUS_TFE) != 0U);
}

/**
 * @brief   Read the current TX FIFO occupancy.
 * @note    Reads IC_TXFLR.
 * @param   i2c    : Pointer to i2c register map
 * @retval  Number of valid entries currently in the TX FIFO (0..FIFO_DEPTH)
 */
static inline uint32_t i2c_tx_fifo_level(I2C_Type *i2c)
{
    return i2c->I2C_TXFLR;
}

/**
 * @brief   Check whether a TX_ABRT has latched in the raw interrupt status.
 * @note    Reads IC_RAW_INTR_STAT.TX_ABRT, which latches independently of
 *          the IRQ mask
 * @param   i2c    : Pointer to i2c register map
 * @retval  true   if TX_ABRT is latched, false otherwise
 */
static inline bool i2c_tx_abort_pending(I2C_Type *i2c)
{
    return ((i2c->I2C_RAW_INTR_STAT & I2C_IC_INTR_STAT_TX_ABRT) != 0U);
}

/**
 * @brief    set i2c target address for slave device in master mode
 * @param    i2c       : Pointer to i2c resources structure
 * @param    address   : i2c 7-bit or 10-bit slave address
 * @param    addr_mode : Addressing mode (10Bit/7Bit)
 * @param    cur_state : Current transfer state (Master Tx/ Master Rx)
 * @retval   none
 */
void i2c_set_target_addr(I2C_Type *i2c, const uint32_t address, const i2c_address_mode_t addr_mode,
                         const I2C_XFER_STATE cur_state);

/**
 * @brief   Setup i2c master clock configuration
 * @note    none
 * @param   i2c          : Pointer to i2c register map
 * @param   clk_khz      : Clock
 * @param   speed_mode   : Speed
 *          I2C_SPEED_STANDARD /
 *          I2C_SPEED_FAST /
 *          I2C_SPEED_FAST_PLUS /
 *          I2C_SPEED_HIGH
 * @retval  none
 */
void i2c_master_set_clock(I2C_Type *i2c, const uint32_t clk_khz, uint8_t speed_mode);

/**
 * @brief   initialize i2c master
 * @note    none
 * @param   i2c          : Pointer to i2c register map
 * @param   tar_addr     : target address
 * @retval  none
 */
void i2c_master_init(I2C_Type *i2c, const uint32_t tar_addr);

/**
 * @brief   initialize i2c slave
 * @note    none
 * @param   i2c          : Pointer to i2c register map
 * @param   slave_addr   : i2c slave address
 * param    addr_mode    : Addressing mode (10Bit/7Bit)
 * @retval  none
 */
void i2c_slave_init(I2C_Type *i2c, uint32_t slave_addr, i2c_address_mode_t addr_mode);

/**
 * @brief    i2c master transmit data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_master_tx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer);

/**
 * @brief    i2c master receive data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_master_rx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer);

/**
 * @brief    i2c slave transmit data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_slave_tx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer);

/**
 * @brief    i2c slave receive data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_slave_rx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H_ */
