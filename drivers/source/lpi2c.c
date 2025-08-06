/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
#include "lpi2c.h"
#include "sys_utils.h"

/**
 * @func   : void lpi2c_send(LPI2C_TYPE *lpi2c,
 *                           LPI2C_XFER_INFO_T *transfer,
 *                           const uint32_t delay)
 * @brief  : writing to the register for transmit data
 * @param  : lpi2c    : Pointer to lpi2c register map
 * @param  : transfer : Pointer to LPI2C_XFER_INFO_T
 * @param  : delay    : delay in us
 * @retval : callback event
 */
void lpi2c_send(LPI2C_TYPE *lpi2c, LPI2C_XFER_INFO_T *transfer, const uint32_t delay)
{
    uint32_t len = transfer->tx_total_num;

    while (len) {
        /* Send a byte if FIFO is not FULL*/
        if (!(lpi2c->LPI2C_OUTBOND_DATA & LPI2C_FIFO_FULL)) {
            /*writing data to fifo*/
            lpi2c->LPI2C_DATA = transfer->tx_buf[transfer->tx_curr_cnt++];
            len--;
        }
    }

    /* Wait until all bytes are transferred */
    while (!(lpi2c->LPI2C_OUTBOND_DATA & LPI2C_FIFO_EMPTY)) {
    }

    /* wait till 1 byte of xfer time*/
    sys_busy_loop_us(delay);

    /* transfer complete */
    transfer->status = LPI2C_XFER_STAT_COMPLETE;
}

/**
 * @func   : void lpi2c_irq_handler(LPI2C_TYPE *lpi2c, LPI2C_XFER_INFO_T *transfer)
 * @brief  : lpi2c irq handler
 * @param  : lpi2c    : Pointer to lpi2c register map
 * @param  : transfer : Pointer to LPI2C_XFER_INFO_T
 * @retval : callback event
 */
void lpi2c_irq_handler(LPI2C_TYPE *lpi2c, LPI2C_XFER_INFO_T *transfer)
{
    uint8_t rx_len = 0;

    rx_len         = (lpi2c->LPI2C_INBOND_DATA & LPI2C_AVL_DATA);

    if (rx_len <= (transfer->rx_total_num - transfer->rx_curr_cnt)) {
        while (rx_len--) {
            /* Storing receive value to the buffer */
            transfer->rx_buf[transfer->rx_curr_cnt++] = lpi2c->LPI2C_DATA;
        }

        /* If complete data is received, then success */
        if (transfer->rx_curr_cnt == (transfer->rx_total_num)) {
            transfer->status = LPI2C_XFER_STAT_COMPLETE;
        }
    }
}
