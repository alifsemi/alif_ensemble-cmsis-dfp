/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "i3c.h"
#include "string.h"
#include "sys_utils.h"

/**
  \fn           static void i3c_wr_tx_fifo(I3C_Type        *i3c,
                                           const  uint8_t  *bytes,
                                           uint32_t  nbytes)
  \brief        Write data to i3c TX FIFO
  \param[in]    i3c     : Pointer to i3c resources structure
  \param[in]    bytes   : Pointer to buffer with data which
                           needs to be write to i3c transmitter
  \param[in]    nbytes  : Number of bytes needs to be write
  \return       none
*/
static void i3c_wr_tx_fifo(I3C_Type *i3c, const uint8_t *bytes, uint32_t nbytes)
{
    uint32_t len_in_words = nbytes / 4;
    uint32_t i, tmp;

    /* FIXME Add check for MAX TX FIFO Length */
    for (i = 0; i < len_in_words; i++) {
        /* FIXME Fix Memory Alignment issue */
        tmp                    = *((uint32_t *) bytes);
        i3c->I3C_TX_DATA_PORT  = tmp;
        bytes                 += 4;
    }

    /* write the remaining bytes in the last incomplete word */
    if (nbytes & 3) {
        tmp = 0;
        memcpy(&tmp, bytes, nbytes & 3);
        i3c->I3C_TX_DATA_PORT = tmp;
    }
}

/**
  \fn           static void i3c_read_rx_fifo(I3C_Type *i3c,
                                             uint8_t  *bytes,
                                             uint32_t  nbytes)
  \brief        Read data from i3c RX FIFO
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    bytes    : Pointer to buffer for data
                            to receive from i3c RX FIFO
  \param[in]    nbytes   : Number of bytes needs to be receive
  \return       none
*/
static void i3c_read_rx_fifo(I3C_Type *i3c, uint8_t *bytes, uint32_t nbytes)
{
    uint32_t len_in_words = nbytes / 4;
    uint32_t i, tmp;

    /* FIXME Add check for MAX RX FIFO Length */
    for (i = 0; i < len_in_words; i++) {
        /* FIXME Fix Memory Alignment issue */
        *((uint32_t *) bytes)  = i3c->I3C_RX_DATA_PORT;
        bytes                 += 4;
    }

    /* read the last word and copy the actual remaining data */
    if (nbytes & 3) {
        tmp = i3c->I3C_RX_DATA_PORT;
        memcpy(bytes, &tmp, nbytes & 3);
    }
}

/**
  \fn           static void i3c_set_tx_buf_thld(I3C_Type *i3c,
  \                                             const uint16_t len)
  \brief        Set Tx buffer threshold
  \param[in]    xfer : Transfer data structure
  \param[in]    len  : Data length
  \return       none
*/
static void i3c_set_tx_buf_thld(I3C_Type *i3c, const uint16_t len)
{
    uint16_t loc_len = (len / 4);
    uint8_t  rem     = (len % 4);
    uint32_t temp =
        (i3c->I3C_DATA_BUFFER_THLD_CTRL & (~I3C_DATA_BUFFER_THLD_CTRL_TX_EMPTY_BUF_THLD_Msk));

    /* If data length is more than 4 bytes then perform the following,
     * else set 0*/
    if (len > 4) {
        if (rem) {
            /* Set 1 extra location */
            temp |= (((loc_len + 1) << I3C_DATA_BUFFER_THLD_CTRL_TX_EMPTY_BUF_THLD_Pos) &
                     I3C_DATA_BUFFER_THLD_CTRL_TX_EMPTY_BUF_THLD_Msk);
        } else {
            /* Set actual number of locations */
            temp |= ((loc_len << I3C_DATA_BUFFER_THLD_CTRL_TX_EMPTY_BUF_THLD_Pos) &
                     I3C_DATA_BUFFER_THLD_CTRL_TX_EMPTY_BUF_THLD_Msk);
        }
    }

    i3c->I3C_DATA_BUFFER_THLD_CTRL = temp;
}

/**
  \fn           static void i3c_set_rx_buf_thld(I3C_Type *i3c,
  \                                             const uint16_t len)
  \brief        Set Rx buffer threshold
  \param[in]    xfer : Transfer data structure
  \param[in]    len  : Data length
  \return       none
*/
static void i3c_set_rx_buf_thld(I3C_Type *i3c, const uint16_t len)
{
    uint16_t loc_len = (len / 4);
    uint8_t  rem     = (len % 4);
    uint32_t temp = (i3c->I3C_DATA_BUFFER_THLD_CTRL & (~I3C_DATA_BUFFER_THLD_CTRL_RX_BUF_THLD_Msk));

    /* If data length is more than 4 bytes then perform the following,
     * else set 0*/
    if (len > 4) {
        if (rem) {
            temp |= (((loc_len + 1) << I3C_DATA_BUFFER_THLD_CTRL_RX_BUF_THLD_Pos) &
                     I3C_DATA_BUFFER_THLD_CTRL_RX_BUF_THLD_Msk);
        } else {
            temp |= ((loc_len << I3C_DATA_BUFFER_THLD_CTRL_RX_BUF_THLD_Pos) &
                     I3C_DATA_BUFFER_THLD_CTRL_RX_BUF_THLD_Msk);
        }
    }

    i3c->I3C_DATA_BUFFER_THLD_CTRL = temp;
}

/**
  \fn           static void i3c_bus_reset(I3C_Type *i3c,
  \                                       const uint32_t core_clk,
  \                                       const uint32_t scl_timeout_cnt,
  \                                       const uint8_t bus_rst_type)
  \brief        Performs bus reset
  \param[in]    i3c              : Pointer to i3c register set structure
  \param[in]    core_clk         : core clock
  \param[in]    scl_timeout_cnt  : SCL Timeout count in microsec
  \param[in]    bus_type         : bus reset type
  \return
*/
static void i3c_bus_reset(I3C_Type *i3c, const uint32_t core_clk, const uint32_t scl_timeout_cnt,
                          const uint8_t bus_rst_type)
{
    uint32_t scl_timing;
    /* Calculate core clk period */
    uint32_t core_period             = DIV_ROUND_UP(REF_CLK_RATE, core_clk);
    uint32_t rst_ctrl                = i3c->I3C_RESET_CTRL;

    /* set SCL Low Master Extended Timeout Register. */
    scl_timing                       = DIV_ROUND_UP(CONVERT_US_TO_NS(scl_timeout_cnt), core_period);
    i3c->I3C_SCL_LOW_MST_EXT_TIMEOUT = I3C_SCL_LOW_MST_EXT_TIMEOUT_COUNT(scl_timing);

    if (bus_rst_type == I3C_BUS_RST_HDR_EXIT) {
        rst_ctrl &= ~I3C_RESET_CTRL_BUS_RESET_TYPE_SCL_LOW_PAT;
    } else {
        rst_ctrl |= I3C_RESET_CTRL_BUS_RESET_TYPE_SCL_LOW_PAT;
    }
    /* Resets the bus */
    i3c->I3C_RESET_CTRL = (rst_ctrl | I3C_RESET_CTRL_BUS_RESET);
}

/**
  \fn           static void i3c_set_port(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        Sets i3c message port
  \param[in]    i3c      : Pointer to i3c register set structure
  \param[in]    xfer     : Pointer to i3c transfer structure
  \return       none
*/
static void i3c_set_port(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    bool is_master = true;

    switch (xfer->xfer_cmd.cmd_type) {
    case I3C_XFER_TYPE_DATA:
        xfer->xfer_cmd.cmd_id = 0U;

        /* Checks about instance mastership */
        if ((i3c->I3C_DEVICE_CTRL_EXTENDED & I3C_DEVICE_CTRL_EXTENDED_DEV_OPERATION_MODE_Msk) ==
            I3C_DEVICE_CTRL_EXTENDED_DEV_OP_MODE_SLV) {
            is_master = false;
        }

        if (is_master) {
            if (xfer->rx_len) {
                xfer->xfer_cmd.port_id = I3C_MST_RX_TID;
            } else {
                xfer->xfer_cmd.port_id = I3C_MST_TX_TID;
            }
        } else {
            if (xfer->rx_len) {
                xfer->xfer_cmd.port_id = I3C_SLV_RX_TID;
            } else {
                xfer->xfer_cmd.port_id = I3C_SLV_TX_TID;
            }
        }
        break;
    case I3C_XFER_CCC_SET:
        xfer->xfer_cmd.port_id = I3C_CCC_SET_TID;
        break;
    case I3C_XFER_CCC_GET:
        xfer->xfer_cmd.port_id = I3C_CCC_GET_TID;
        break;
    case I3C_XFER_TYPE_ADDR_ASSIGN:
        xfer->xfer_cmd.port_id = I3C_ADDR_ASSIGN_TID;
        break;
    default:
        xfer->xfer_cmd.port_id = I3C_INVALID_TID;
        break;
    }
}

/**
  \fn           static void i3c_fetch_error_type(i3c_xfer_t *xfer)
  \brief        Fetches the error type
  \param[in]    xfer : Transfer data structure
  \return       none
*/
static void i3c_fetch_error_type(i3c_xfer_t *xfer)
{
    switch (xfer->error) {
    case I3C_RESPONSE_QUEUE_PORT_ERR_CRC:
        xfer->error = I3C_COMM_ERROR_CRC;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_PARITY:
        xfer->error = I3C_COMM_ERROR_PARITY;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_FRAME:
        xfer->error = I3C_COMM_ERROR_FRAME;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_IBA_NACK:
        xfer->error = I3C_COMM_ERROR_IBA_NACK;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_ADDRESS_NACK:
        xfer->error = I3C_COMM_ERROR_ADDR_NACK;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_OVER_UNDER_FLOW:
        xfer->error = I3C_COMM_ERROR_BUF_UNDR_OVR_FLW;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_XFER_ABORT:
        xfer->error = I3C_COMM_ERROR_XFER_ABORT;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_I2C_W_NACK:
        xfer->error = I3C_COMM_ERROR_I2C_SLV_W_NACK;
        break;
    case I3C_RESPONSE_QUEUE_PORT_ERR_PEC_OR_EARLY_TERM:
        xfer->error = I3C_COMM_ERROR_PEC_OR_EARLY_TERM;
        break;
    default:
        break;
    }
}

/**
  \fn           static bool i3c_check_response(I3C_Type *i3c,
  \                                            i3c_xfer_t *xfer,
  \                                            uint32_t *resp)
  \brief        Fetches the transfer response
  \param[in]    i3c      : Pointer to i3c register set structure
  \param[in]    xfer     : Pointer to i3c transfer structure
  \param[in]    resp     : Received response
  \return       Response received status
*/
static bool i3c_check_response(I3C_Type *i3c, i3c_xfer_t *xfer, uint32_t *resp)
{
    if (i3c_resp_rcvd(i3c)) {
        *resp       = i3c->I3C_RESPONSE_QUEUE_PORT;
        xfer->error = I3C_RESPONSE_QUEUE_PORT_ERR_STATUS(*resp);
        return true;
    }

    return false;
}

/**
  \fn           static bool i3c_fetch_xfer_error(i3c_xfer_t *xfer,
  \                                              uint32_t *resp,
  \                                              uint16_t len)
  \brief        Fetches the tx transfer errors
  \param[in]    xfer     : Pointer to i3c transfer structure
  \param[in]    resp     : Received response
  \param[in]    len      : Data len
  \return       Error status
*/
static bool i3c_fetch_xfer_error(i3c_xfer_t *xfer, uint32_t *resp, const uint16_t len)
{
    if ((xfer->error) || (I3C_RESPONSE_QUEUE_PORT_DATA_LEN(*resp) != len) ||
        (I3C_RESPONSE_QUEUE_PORT_TID(*resp) != xfer->xfer_cmd.port_id))
    {
        /* Fetches error type */
        i3c_fetch_error_type(xfer);
        return true;
    }

    return false;
}

/**
  \fn           static void i3c_dispatch_xfer_cmd(I3C_Type *i3c,
  \                                               i3c_xfer_t *xfer)
  \brief        Add commands to i3c Command Queue
  \param[in]    i3c  : Pointer to i3c resources structure
  \param[in]    xfer : Pointer to i3c transfer structure
  \return       none
*/
static void i3c_dispatch_xfer_cmd(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    if (xfer->xfer_cmd.cmd_type == I3C_XFER_TYPE_ADDR_ASSIGN) {
        /* Issue ccc command */
        i3c->I3C_COMMAND_QUEUE_PORT = I3C_COMMAND_QUEUE_PORT_SPEED(xfer->xfer_cmd.speed) |
                                      I3C_COMMAND_QUEUE_PORT_DEV_INDEX(xfer->xfer_cmd.addr_index) |
                                      I3C_COMMAND_QUEUE_PORT_CMD(xfer->xfer_cmd.cmd_id) |
                                      I3C_COMMAND_QUEUE_PORT_TOC | I3C_COMMAND_QUEUE_PORT_ROC |
                                      I3C_COMMAND_QUEUE_PORT_DEV_COUNT(xfer->xfer_cmd.addr_depth) |
                                      I3C_COMMAND_QUEUE_PORT_ADDR_ASSGN_CMD |
                                      I3C_COMMAND_QUEUE_PORT_TID(xfer->xfer_cmd.port_id);
    } else {
        /* Perform the following if it is not Address command */
        xfer->xfer_cmd.cmd_hi = I3C_COMMAND_QUEUE_PORT_ARG_DATA_LEN(xfer->xfer_cmd.data_len) |
                                I3C_COMMAND_QUEUE_PORT_TRANSFER_ARG;

        xfer->xfer_cmd.cmd_lo = I3C_COMMAND_QUEUE_PORT_SPEED(xfer->xfer_cmd.speed) |
                                I3C_COMMAND_QUEUE_PORT_DEV_INDEX(xfer->xfer_cmd.addr_index) |
                                I3C_COMMAND_QUEUE_PORT_CMD(xfer->xfer_cmd.cmd_id) |
                                I3C_COMMAND_QUEUE_PORT_TOC |
                                I3C_COMMAND_QUEUE_PORT_TID(xfer->xfer_cmd.port_id) |
                                I3C_COMMAND_QUEUE_PORT_ROC | I3C_COMMAND_QUEUE_PORT_PEC;

        /* Add Command present macro if command is present or
         * is a HDR-DDR communicatoin */
        if ((xfer->xfer_cmd.cmd_id) || (xfer->xfer_cmd.speed == I3C_SPEED_HDR_DDR)) {
            xfer->xfer_cmd.cmd_lo |= I3C_COMMAND_QUEUE_PORT_CP;

            /* Add Defining byte */
            if (xfer->xfer_cmd.def_byte) {
                xfer->xfer_cmd.cmd_hi |=
                    I3C_COMMAND_QUEUE_PORT_ARG_DATA_DB(xfer->xfer_cmd.def_byte);
                xfer->xfer_cmd.cmd_lo |= I3C_COMMAND_QUEUE_PORT_DBP;
            }
        }

        /* Add Port read macro if reception is required */
        if (xfer->rx_len) {
            xfer->xfer_cmd.cmd_lo |= I3C_COMMAND_QUEUE_PORT_READ_TRANSFER;
        }

        /* Issue transfer arguments */
        if (xfer->xfer_cmd.cmd_hi) {
            i3c->I3C_COMMAND_QUEUE_PORT = xfer->xfer_cmd.cmd_hi;
        }

        /* Issue transfer command */
        if (xfer->xfer_cmd.cmd_lo) {
            i3c->I3C_COMMAND_QUEUE_PORT = xfer->xfer_cmd.cmd_lo;
        }
    }
}

/**
  \fn           static void i3c_send(I3C_Type *i3c,
  \                                  i3c_xfer_t *xfer,
  \                                  const uint16_t sts_len)
  \brief        Send I3C data to data buffer
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    xfer    : Pointer to i3c transfer structure
  \param[in]    sts_len : Data Tx buffer status len
  \return       none
*/
static void i3c_send(I3C_Type *i3c, i3c_xfer_t *xfer, const uint16_t sts_len)
{
    const uint8_t *tbuf = (const uint8_t *) xfer->tx_buf;

    /* If available tx slots are lesser than required slots then,
     * send data of available slots */
    if ((xfer->tx_len - xfer->tx_cur_cnt) >= sts_len) {
        i3c_wr_tx_fifo(i3c, &tbuf[xfer->tx_cur_cnt], sts_len);
        xfer->tx_cur_cnt += sts_len;
    } else {
        /* Send required bytes of data */
        i3c_wr_tx_fifo(i3c, &tbuf[xfer->tx_cur_cnt], ((xfer->tx_len - xfer->tx_cur_cnt)));
        xfer->tx_cur_cnt += (xfer->tx_len - xfer->tx_cur_cnt);
    }
}

/**
  \fn           static bool i3c_send_blocking(I3C_Type *i3c,
  \                                           i3c_xfer_t *xfer)
  \brief        Send I3C data in blocking mode
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    xfer    : Pointer to i3c transfer structure
  \return       Data Send status
*/
static bool i3c_send_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint16_t dbuf_len = 0U;
    uint32_t nresp    = 0U;

    if (i3c_check_response(i3c, xfer, &nresp)) {
        /* Check for error and perform below */
        if (i3c_fetch_xfer_error(xfer, &nresp, 0)) {
            /* Mark as Tx Error */
            xfer->status = I3C_XFER_STATUS_ERROR_TX;
            return false;
        }
    }

    /* Fetch for available tx slots and send data if
     * available */
    dbuf_len = i3c_get_empty_tx_buf_len(i3c);

    if (dbuf_len) {
        i3c_send(i3c, xfer, dbuf_len);
    }

    return true;
}

/**
  \fn           static void i3c_receive(I3C_Type *i3c,
  \                                     i3c_xfer_t *xfer,
  \                                     const uint16_t sts_len)
  \brief        Receive data from Rx buffer
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    xfer    : Pointer to i3c transfer structure
  \param[in]    sts_len : Data Rx buffer status len
  \return       none
*/
static void i3c_receive(I3C_Type *i3c, i3c_xfer_t *xfer, const uint16_t sts_len)
{
    uint8_t *rbuf = (uint8_t *) xfer->rx_buf;

    /* If available rx data is lesser than required number of bytes then,
     * receive currently available data */
    if ((xfer->rx_len - xfer->rx_cur_cnt) >= sts_len) {
        i3c_read_rx_fifo(i3c, &rbuf[xfer->rx_cur_cnt], sts_len);
        xfer->rx_cur_cnt += sts_len;
    } else {
        /* Read required bytes of data */
        i3c_read_rx_fifo(i3c, &rbuf[xfer->rx_cur_cnt], ((xfer->rx_len - xfer->rx_cur_cnt)));
        xfer->rx_cur_cnt += (xfer->rx_len - xfer->rx_cur_cnt);
    }
}

/**
  \fn           static bool i3c_receive_blocking(I3C_Type *i3c,
  \                                              i3c_xfer_t *xfer,
  \                                              uint32_t *resp)
  \brief        Receive I3C data in blocking mode
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    xfer    : Pointer to i3c transfer structure
  \param[in]    resp    : Pointer to i3c transfer response
  \return       Data Read status
*/
static bool i3c_receive_blocking(I3C_Type *i3c, i3c_xfer_t *xfer, uint32_t *resp)
{
    uint16_t dbuf_len = 0U;

    if (i3c_check_response(i3c, xfer, resp)) {
        /* Check for error and perform below */
        if (i3c_fetch_xfer_error(xfer, resp, xfer->rx_len)) {
            /* Mark as Rx Error */
            xfer->status = I3C_XFER_STATUS_ERROR_RX;
            return false;
        }
    }

    /* Fetch for available rx data and read if available */
    dbuf_len = i3c_get_avail_rx_buf_len(i3c);

    if (dbuf_len) {
        i3c_receive(i3c, xfer, dbuf_len);
    }

    return true;
}

/**
  \fn           static void i3c_wait_validate_tx_resp_blocking(I3C_Type *i3c,
  \                                                            i3c_xfer_t *xfer)
  \brief        Wait for Tx response and verify in blocking mode
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    xfer    : Pointer to i3c transfer structure
  \return       none
*/
static void i3c_wait_validate_tx_resp_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint32_t nresp = 0U;

    /* Waits unless Tx buffer gets empty */
    while (!i3c_tx_buf_empty(i3c)) {
    }

    /* Waits till some response received */
    while (!i3c_check_response(i3c, xfer, &nresp)) {
    }

    if (i3c_fetch_xfer_error(xfer, &nresp, 0)) {
        /* Mark as Tx error */
        xfer->status = I3C_XFER_STATUS_ERROR_TX;
    } else {
        if (i3c_is_master(i3c)) {
            /* Mark as Transfer DONE */
            xfer->status = (I3C_XFER_STATUS_MST_TX_DONE | I3C_XFER_STATUS_DONE);
        } else {
            /* mark as Transfer DONE */
            xfer->status = (I3C_XFER_STATUS_SLV_TX_DONE | I3C_XFER_STATUS_DONE);
        }
    }
}

/**
  \fn           static void i3c_wait_validate_rx_resp_blocking(I3C_Type *i3c,
  \                                                            i3c_xfer_t *xfer,
  \                                                            uint32_t *resp)
  \brief        Wait for response and verify in blocking mode
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    xfer    : Pointer to i3c transfer structure
  \param[in]    resp    : Pointer to i3c xfer resp
  \return       none
*/
static void i3c_wait_validate_rx_resp_blocking(I3C_Type *i3c, i3c_xfer_t *xfer, uint32_t *resp)
{
    /* Check if response is already not received */
    if (!(*resp)) {
        /* Waits till some response received */
        while (!i3c_check_response(i3c, xfer, resp)) {
        }
    }

    if (i3c_fetch_xfer_error(xfer, resp, xfer->rx_len)) {
        /* Mark as Rx Error */
        xfer->status = I3C_XFER_STATUS_ERROR_RX;
    } else {
        if (i3c_is_master(i3c)) {
            /* Mark as Master Rx DONE */
            xfer->status = (I3C_XFER_STATUS_MST_RX_DONE | I3C_XFER_STATUS_DONE);
        } else {
            /* mark as Slave Rx DONE */
            xfer->status = (I3C_XFER_STATUS_SLV_RX_DONE | I3C_XFER_STATUS_DONE);
        }
    }
}

/**
  \fn           void i3c_error_handler(I3C_Type *i3c,
                                       i3c_xfer_t *xfer,
                                       const uint32_t status,
                                       const uint32_t resp)
  \brief        handles I3C error
  \param[in]    i3c    : Pointer to i3c register set structure
  \param[in]    xfer   : Pointer to i3c transfer structure
  \param[in]    status : interrupt status
  \param[in]    resp   : Rx Response
  \return       none
*/
static void i3c_error_handler(I3C_Type *i3c, i3c_xfer_t *xfer, const uint32_t status,
                              const uint32_t resp)
{
    uint32_t tid = 0U;

    xfer->status = I3C_XFER_STATUS_ERROR;

    if (status & I3C_INTR_STATUS_TRANSFER_ABORT_STS) {
        /* Message abort happened */
        i3c_clear_intr(i3c, I3C_INTR_STATUS_TRANSFER_ABORT_STS);
        xfer->status |= I3C_XFER_STATUS_ERROR_XFER_ABORT;
    } else {
        tid = I3C_RESPONSE_QUEUE_PORT_TID(resp);
        switch (tid) {
        case I3C_MST_TX_TID:
        case I3C_SLV_TX_TID:
        case I3C_CCC_SET_TID:
            i3c_disable_intr(i3c, I3C_INTR_STATUS_TX_THLD_STS);
            xfer->status |= I3C_XFER_STATUS_ERROR_TX;
            break;

        case I3C_MST_RX_TID:
        case I3C_SLV_RX_TID:
        case I3C_CCC_GET_TID:
            i3c_disable_intr(i3c, I3C_INTR_STATUS_RX_THLD_STS);
            xfer->status |= I3C_XFER_STATUS_ERROR_RX;
            break;

        case I3C_ADDR_ASSIGN_TID:
            xfer->tx_len  = I3C_RESPONSE_QUEUE_PORT_DATA_LEN(resp);
            xfer->status |= I3C_XFER_STATUS_ERROR_ADDR_ASSIGN;
            break;
        }
    }
}

/**
  \fn           static void i3c_ibi_handler(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        i3c interrupt service routine for In-band-interrupts
  \param[in]    i3c   : Pointer to i3c register set structure
  \param[in]    xfer  : Pointer to i3c transfer structure
  \return       none
*/
static void i3c_ibi_handler(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint32_t ibi_resp = 0U;

    /* Checks for the IBI status as a master */
    ibi_resp          = i3c->I3C_IBI_QUEUE_STATUS;

    if (ibi_resp) {
        ibi_resp = I3C_IBI_QUEUE_STATUS_IBI_ID(ibi_resp);

        if (ibi_resp == I3C_HOT_JOIN_ID) {
            /* Hot-Join is requested by new slave */
            xfer->status = (I3C_XFER_STATUS_SLV_HOT_JOIN_REQ | I3C_XFER_STATUS_DONE);
        }
        /* Checks if it is a IBI slave interrupt request */
        else if (I3C_IBI_QUEUE_STATUS_IBI_ID_RW(ibi_resp))
        {
            /* IBI slave interrupt requested by new slave.
             * Store it's address */
            xfer->addr_len = (ibi_resp >> I3C_IBI_QUEUE_STATUS_IBI_ID_RW_Pos);

            xfer->status   = (I3C_XFER_STATUS_IBI_SLV_INTR_REQ | I3C_XFER_STATUS_DONE);
        }
        /* Checks if it is a Mastership request.
         * If yes store the requester's address */
        else if (!(I3C_IBI_QUEUE_STATUS_IBI_ID_RW(ibi_resp)))
        {
            xfer->addr_len = (ibi_resp >> I3C_IBI_QUEUE_STATUS_IBI_ID_RW_Pos);

            /* Mastership is requested by new slave */
            xfer->status   = (I3C_XFER_STATUS_IBI_MASTERSHIP_REQ | I3C_XFER_STATUS_DONE);
        }
    } else {
        /* Checks the IBI status as a slave */
        ibi_resp = ((i3c->I3C_SLV_INTR_REQ & I3C_SLV_INTR_REQ_IBI_STS_Msk) >>
                    I3C_SLV_INTR_REQ_IBI_STS_Pos);

        /* Checks if the master has acked the IBI request */
        if (ibi_resp == I3C_SLV_INTR_REQ_IBI_STS_MST_ACKED) {
            xfer->status = I3C_XFER_STATUS_DONE;
        }
        /* Checks if IBI is not initiated only */
        else if (ibi_resp == I3C_SLV_INTR_REQ_IBI_STS_NATMPTED) {
            xfer->status = I3C_XFER_STATUS_ERROR;
        }
    }
}

/**
  \fn           int32_t i3c_slave_req_bus_mastership(I3C_Type *i3c)
  \brief        Sends mastership request to master
  \param[in]    i3c     : Pointer to i3c register set structure
  \return       exec status
*/
int32_t i3c_slave_req_bus_mastership(I3C_Type *i3c)
{
    /* Returns -1 if the device dynamic address is invalid */
    if (!(i3c->I3C_DEVICE_ADDR & I3C_DEVICE_ADDR_DYNAMIC_ADDR_VALID)) {
        return -1;
    }

    /* Returns -1 if the device doesn't have Secondary Master capability */
    if ((i3c->I3C_HW_CAPABILITY & I3C_HW_CAPABILITY_DEVICE_ROLE_CONFIG_Msk) !=
        I3C_DEVICE_ROLE_SECONDARY_MASTER)
    {
        return -1;
    }

    /* Returns -1 if Master Request option is disabled by master */
    if (!(i3c->I3C_SLV_EVENT_STATUS & I3C_SLV_EVENT_STATUS_MR_EN)) {
        return -1;
    }

    /* Returns -1 if either MR or SIR is already requested */
    if (i3c->I3C_SLV_INTR_REQ & (I3C_SLV_INTR_REQ_MR | I3C_SLV_INTR_REQ_SIR)) {
        return -1;
    }

    /* Accept master acceptance command */
    i3c->I3C_DEVICE_CTRL_EXTENDED &= ~I3C_DEVICE_CTRL_EXTENDED_REQMST_ACK_CTRL;

    /* Enable updated ownership interrupt */
    i3c_enable_intr(i3c,
                    (I3C_INTR_STATUS_EN_BUSOWNER_UPDATED_STS_EN |
                     I3C_INTR_STATUS_EN_IBI_UPDATED_STS_EN));
    /* Enable Master request */
    i3c->I3C_SLV_INTR_REQ |= I3C_SLV_INTR_REQ_MR;

    return 0;
}

/**
  \fn           int32_t i3c_slave_tx_slv_intr_req(I3C_Type *i3c)
  \brief        Sends Slave Interrupt request to master
  \param[in]    i3c     : Pointer to i3c register set structure
  \return       exec status
*/
int32_t i3c_slave_tx_slv_intr_req(I3C_Type *i3c)
{
    /* Returns -1 if the device dynamic address is invalid */
    if (!(i3c->I3C_DEVICE_ADDR & I3C_DEVICE_ADDR_DYNAMIC_ADDR_VALID)) {
        return -1;
    }

    /* Returns -1 if Slave Interrupt Request option is disabled by master */
    if (!(i3c->I3C_SLV_EVENT_STATUS & I3C_SLV_EVENT_STATUS_SIR_EN)) {
        return -1;
    }

    /* Returns -1 if either MR or SIR is already requested */
    if (i3c->I3C_SLV_INTR_REQ & (I3C_SLV_INTR_REQ_MR | I3C_SLV_INTR_REQ_SIR)) {
        return -1;
    }

    /* Enable updated ownership interrupt */
    i3c_enable_intr(i3c, I3C_INTR_STATUS_EN_IBI_UPDATED_STS_EN);

    /* Enable Slave Interrupt request */
    i3c->I3C_SLV_INTR_REQ &= I3C_SLV_INTR_REQ_SIR_CTRL;
    i3c->I3C_SLV_INTR_REQ |= I3C_SLV_INTR_REQ_SIR;

    return 0;
}

/**
  \fn           uint8_t i3c_get_slv_dyn_addr(I3C_Type *i3c,
                                             const uint8_t static_addr)
  \brief        Gets slave's dynamic from DAT for the
                given static address
  \param[in]    i3c         : Pointer to i3c register set structure
  \param[in]    static_addr : Slave's static address
  \return       none
*/
uint8_t i3c_get_slv_dyn_addr(I3C_Type *i3c, const uint8_t static_addr)
{
    uint8_t  iter      = 0U;
    uint32_t datp      = i3c_get_dat_addr(i3c);
    uint32_t dat_addr  = 0U;
    uint32_t dest_addr = 0U;

    for (iter = 0U; iter < I3C_MAX_DEVS; iter++) {
        /* DAT address = i3c Base + DAT Base + (Pos * 4) */
        dat_addr  = (uint32_t) i3c + datp + (iter << 2);
        dest_addr = *((volatile uint32_t *) (dat_addr));

        if ((dest_addr & (I3C_DAT_STATIC_ADDR_Msk)) == static_addr) {
            return ((dest_addr & I3C_DAT_DYNAMIC_ADDR_Msk) >> I3C_DAT_DYNAMIC_ADDR_Pos);
        }
    }
    return 0U;
}

/**
  \fn           void i3c_sec_master_get_dct(I3C_Type *i3c,
  \                                         i3c_dev_char_t *data,
  \                                         const uint8_t slv_cnt)
  \brief        Get Device Characteritics Table of slaves.
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    data    : Device characteristics data
  \param[in]    slv_cnt : Slaves count
  \return       None
*/
void i3c_sec_master_get_dct(I3C_Type *i3c, i3c_dev_char_t *data, const uint8_t slv_cnt)
{
    uint8_t  iter;
    uint32_t dctp = i3c_get_dct_addr(i3c);

    volatile uint32_t *dct_ptr;

    /* DCT address = i3c Base + DCT Offset */
    dct_ptr = (volatile uint32_t *) ((uint32_t) i3c + dctp);

    /* Fetches slaves information received through
     * DEFSLVS CCC */
    for (iter = 0U; iter < slv_cnt; iter++) {
        data->static_addr  = I3C_SEC_DEV_CHAR_TABLE1_STATIC_ADDR(*dct_ptr);
        data->bcr          = I3C_SEC_DEV_CHAR_TABLE1_BCR_TYPE(*dct_ptr);
        data->dcr          = I3C_SEC_DEV_CHAR_TABLE1_DCR_TYPE(*dct_ptr);
        data->dynamic_addr = I3C_SEC_DEV_CHAR_TABLE1_DYNAMIC_ADDR(*dct_ptr);

        data++;
        dct_ptr++;
    }
}

/**
  \fn           void i3c_master_get_dct(I3C_Type *i3c,
  \                                     i3c_dev_prime_info_t *data,
  \                                     const uint8_t addr)
  \brief        Get Device Characteritics Table of requested slave.
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    data    : Device characteristics data
  \param[in]    addr    : Slave's dynamic address
  \return       None
*/
void i3c_master_get_dct(I3C_Type *i3c, i3c_dev_prime_info_t *data, const uint8_t addr)
{
    uint8_t            iter;
    volatile uint32_t *dct_ptr;
    volatile uint32_t *ptr;
    uint32_t           dctp    = i3c_get_dct_addr(i3c);
    uint8_t            dev_cnt = i3c_get_dct_cur_idx(i3c);

    /* Finds */
    for (iter = 0U; iter < dev_cnt; iter++) {
        /* DCT address = i3c Base + DCT Offset + (iter * 16)*/
        dct_ptr = (volatile uint32_t *) ((uint32_t) i3c + dctp + (iter << 4));
        ptr     = (volatile uint32_t *) ((uint32_t) dct_ptr + I3C_DCT_DYNAMIC_ADDR_OFFSET);
        if (((*ptr) & I3C_DCT_DYNAMIC_ADDR_Msk) == addr) {
            /* If the DCT slave address is matching with addr,
             * then store its PID and device characteristics */
            data->dev_char.dynamic_addr = ((*ptr) & I3C_DCT_DYNAMIC_ADDR_Msk);
            data->dev_char.static_addr  = 0U;
            ptr--;
            data->dev_char.dcr = ((*ptr) & I3C_DCT_DCR_Msk);
            data->dev_char.bcr = (((*ptr) & I3C_DCT_BCR_Msk) >> I3C_DCT_BCR_Pos);
            ptr--;
            data->pid.dcr     = ((*ptr) & I3C_DCT_PID_DCR_Msk);
            data->pid.inst_id = (((*ptr) & I3C_DCT_INST_ID_Msk) >> I3C_DCT_INST_ID_Pos);

            ptr--;
            data->pid.part_id     = ((*ptr) & I3C_DCT_PART_ID_Msk);
            data->pid.pid_sel     = (((*ptr) & I3C_DCT_ID_SEL_Msk) >> I3C_DCT_ID_SEL_Pos);
            data->pid.mipi_mfg_id = (((*ptr) & I3C_DCT_MIPI_MFG_ID_Msk) >> I3C_DCT_MIPI_MFG_ID_Pos);
            break;
        }
    }
}

/**
  \fn           void i3c_master_bus_reset(I3C_Type *i3c, const uint32_t core_clk,
  \                                       const uint32_t scl_timeout_cnt,
  \                                       const uint8_t bus_rst_type)
  \brief        Resets the bus as a master
  \param[in]    i3c              : Pointer to i3c register set structure
  \param[in]    core_clk         : core clock
  \param[in]    scl_timeout_cnt  : SCL Timeout count in microsec
  \param[in]    bus_type         : bus reset type
  \return       none
*/
void i3c_master_bus_reset(I3C_Type *i3c, const uint32_t core_clk, const uint32_t scl_timeout_cnt,
                          const uint8_t bus_rst_type)
{
    /* Enable Bus Reset interrupt */
    i3c_enable_intr(i3c, I3C_INTR_STATUS_EN_BUS_RESET_DONE_STS_EN);

    /* Resets the bus */
    i3c_bus_reset(i3c, core_clk, scl_timeout_cnt, bus_rst_type);
}

/**
  \fn           void i3c_master_bus_reset_blocking(I3C_Type *i3c, const uint32_t core_clk,
  \                                                const uint32_t scl_timeout_cnt,
  \                                                const uint8_t bus_rst_type)
  \brief        Resets the bus as a master in blocking mode
  \param[in]    i3c              : Pointer to i3c register set structure
  \param[in]    core_clk         : core clock
  \param[in]    scl_timeout_cnt  : SCL Timeout count in microsec
  \param[in]    bus_type         : bus reset type
  \return       none
*/
void i3c_master_bus_reset_blocking(I3C_Type *i3c, const uint32_t core_clk,
                                   const uint32_t scl_timeout_cnt, const uint8_t bus_rst_type)
{
    while (!(i3c->I3C_PRESENT_STATE & I3C_PRESENT_STATE_MASTER_IDLE)) {
    }

    /* Resets the bus */
    i3c_bus_reset(i3c, core_clk, scl_timeout_cnt, bus_rst_type);

    /* wait unless bus reset done */
    while (i3c->I3C_RESET_CTRL & I3C_RESET_CTRL_BUS_RESET) {
    }
}

/**
  \fn           void i3c_master_tx_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        Perform master data transmission in blocking mode.
  \param[in]    i3c      : Pointer to i3c register set structure
  \param[in]    xfer     : Pointer to i3c transfer structure
  \return       none
*/
void i3c_master_tx_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    while (!(i3c->I3C_PRESENT_STATE & I3C_PRESENT_STATE_MASTER_IDLE)) {
    }

    xfer->xfer_cmd.cmd_id  = 0U;
    xfer->xfer_cmd.port_id = I3C_MST_TX_TID;

    /* Dispatch commands to i3c Command Queue */
    i3c_dispatch_xfer_cmd(i3c, xfer);

    while (1) {
        if (i3c_send_blocking(i3c, xfer)) {
            /* If all bytes are transmitted then perform the following */
            if (xfer->tx_cur_cnt >= xfer->tx_len) {
                i3c_wait_validate_tx_resp_blocking(i3c, xfer);
                break;
            }
        } else {
            /* Error in Tx*/
            break;
        }
    }
}

/**
  \fn           void i3c_master_rx_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        Perform master data reception in blocking mode.
  \param[in]    i3c      : Pointer to i3c register set structure
  \param[in]    xfer     : Pointer to i3c transfer structure
  \return       none
*/
void i3c_master_rx_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint32_t nresp = 0U;

    while (!(i3c->I3C_PRESENT_STATE & I3C_PRESENT_STATE_MASTER_IDLE)) {
    }

    xfer->xfer_cmd.cmd_id  = 0U;
    xfer->xfer_cmd.port_id = I3C_MST_RX_TID;

    /* Dispatch commands to i3c Command Queue */
    i3c_dispatch_xfer_cmd(i3c, xfer);

    while (1U) {
        if (i3c_receive_blocking(i3c, xfer, &nresp)) {
            /* If all bytes are received then perform the following */
            if (xfer->rx_cur_cnt >= xfer->rx_len) {
                i3c_wait_validate_rx_resp_blocking(i3c, xfer, &nresp);
                break;
            }
        } else {
            /* Error in Rx*/
            break;
        }
    }
}

/**
  \fn           void i3c_slave_tx_blocking(I3C_Type *i3c,
                                           i3c_xfer_t *xfer)
  \brief        Performs slave data transmission in blocking mode.
  \param[in]    i3c      : Pointer to i3c register set structure
  \param[in]    xfer     : Pointer to i3c transfer structure
  \return       none
*/
void i3c_slave_tx_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    xfer->xfer_cmd.cmd_id       = 0U;
    xfer->xfer_cmd.port_id      = I3C_SLV_TX_TID;

    /* Add commands to i3c Command Queue.
     * As per mipi_i3c_databook Section 2.7.13
     * no command required for transmit,
     * only data length is required
     */
    i3c->I3C_COMMAND_QUEUE_PORT = I3C_COMMAND_QUEUE_PORT_ARG_DATA_LEN(xfer->xfer_cmd.data_len) |
                                  I3C_COMMAND_QUEUE_PORT_SLV_PORT_TID(I3C_SLV_TX_TID);

    while (1) {
        if (i3c_send_blocking(i3c, xfer)) {
            /* If all bytes are transmitted then perform the following */
            if (xfer->tx_cur_cnt >= xfer->tx_len) {
                i3c_wait_validate_tx_resp_blocking(i3c, xfer);
                break;
            }
        } else {
            /* Error in Tx*/
            break;
        }
    }
}

/**
  \fn           void i3c_slave_rx_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        Performs slave data reception in blocking mode.
  \param[in]    i3c      : Pointer to i3c register set structure
  \param[in]    xfer     : Pointer to i3c transfer structure
  \return       none
*/
void i3c_slave_rx_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint32_t nresp         = 0U;

    xfer->xfer_cmd.cmd_id  = 0U;
    xfer->xfer_cmd.port_id = I3C_SLV_RX_TID;

    while (1U) {
        if (i3c_receive_blocking(i3c, xfer, &nresp)) {
            /* If all bytes are received then perform the following */
            if (xfer->rx_cur_cnt >= xfer->rx_len) {
                i3c_wait_validate_rx_resp_blocking(i3c, xfer, &nresp);
                break;
            }
        } else {
            /* Error in Rx*/
            break;
        }
    }
}

/**
  \fn           void i3c_send_xfer_cmd_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        performs command transfer in blocking mode
  \param[in]    i3c      : Pointer to i3c register set structure
  \param[in]    xfer     : Pointer to i3c transfer structure
  \return       none
*/
void i3c_send_xfer_cmd_blocking(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint32_t nresp = 0U;

    while (!(i3c->I3C_PRESENT_STATE & I3C_PRESENT_STATE_MASTER_IDLE)) {
    }

    if (xfer->xfer_cmd.cmd_type == I3C_XFER_CCC_SET) {
        xfer->xfer_cmd.port_id = I3C_CCC_SET_TID;
    } else if (xfer->xfer_cmd.cmd_type == I3C_XFER_CCC_GET) {
        xfer->xfer_cmd.port_id = I3C_CCC_GET_TID;
    } else if (xfer->xfer_cmd.cmd_type == I3C_XFER_TYPE_ADDR_ASSIGN) {
        xfer->xfer_cmd.port_id = I3C_ADDR_ASSIGN_TID;
    }

    /* Dispatch commands to i3c Command Queue */
    i3c_dispatch_xfer_cmd(i3c, xfer);

    /* Waits till some response received */
    while (!i3c_check_response(i3c, xfer, &nresp)) {
    }

    if (i3c_fetch_xfer_error(xfer, &nresp, xfer->rx_len)) {
        /* Mark as Error */
        xfer->status = I3C_XFER_STATUS_ERROR;

        xfer->rx_len = I3C_RESPONSE_QUEUE_PORT_DATA_LEN(nresp);
    } else {
        if (xfer->xfer_cmd.cmd_type == I3C_XFER_CCC_GET) {
            if (xfer->rx_buf) {
                /* Reads the received data */
                i3c_read_rx_fifo(i3c, xfer->rx_buf, xfer->rx_len);
            }
        }

        /* mark as Transfer DONE */
        xfer->status = I3C_XFER_STATUS_DONE;
    }
}

/**
  \fn           void i3c_slow_bus_clk_cfg(I3C_Type *i3c,
  \                                       const uint32_t  core_clk)
  \brief        i3c slow bus clock configuration for i3c slave device
  \note         This function sets the clock timings as follows:
                OD - 2MHz, PP - 2MHz, SDR1-SDR4 - null
  \param[in]    i3c       : Pointer to i3c register set structure
  \param[in]    core_clk  : core clock
  \return       none
*/
void i3c_slow_bus_clk_cfg(I3C_Type *i3c, const uint32_t core_clk)
{
    unsigned long core_rate, core_period;
    uint32_t      scl_timing;
    uint8_t       hcnt, lcnt;

    core_rate   = core_clk;

    /* Calculate core clk period */
    core_period = DIV_ROUND_UP(REF_CLK_RATE, core_rate);

    /* Calculate SCL push-pull High and Low count for
     *  I3C transfers targeted to I3C devices.*/
    hcnt        = DIV_ROUND_UP(I3C_SLOW_BUS_THIGH_NS, core_period);
    if (hcnt < I3C_SCL_I3C_TIMING_CNT_MIN) {
        hcnt = I3C_SCL_I3C_TIMING_CNT_MIN;
    }

    lcnt = DIV_ROUND_UP(core_rate, I3C_BUS_SDR4_SCL_RATE) - hcnt;
    if (lcnt < I3C_SCL_I3C_TIMING_CNT_MIN) {
        lcnt = I3C_SCL_I3C_TIMING_CNT_MIN;
    }

    scl_timing =
        (I3C_SCL_I3C_PP_TIMING_I3C_PP_HCNT(hcnt) | I3C_SCL_I3C_PP_TIMING_I3C_PP_LCNT(lcnt));
    i3c->I3C_SCL_I3C_PP_TIMING = scl_timing;

    /* set the Bus free time for initiating the transfer in master mode.*/
    if (!(i3c->I3C_DEVICE_CTRL & I3C_DEVICE_CTRL_I2C_SLAVE_PRESENT)) {
        i3c->I3C_BUS_FREE_AVAIL_TIMING = I3C_BUS_FREE_AVAIL_TIMING_BUS_FREE_TIME(lcnt);
    }

    /* SCL open-drain High and Low count (I3C) for
     *  I3C transfers targeted to I3C devices*/
    lcnt = DIV_ROUND_UP(I3C_SLOW_BUS_TLOW_OD_NS, core_period);
    scl_timing =
        (I3C_SCL_I3C_OD_TIMING_I3C_OD_HCNT(hcnt) | I3C_SCL_I3C_OD_TIMING_I3C_OD_LCNT(lcnt));
    i3c->I3C_SCL_I3C_OD_TIMING   = scl_timing;

    /* set SCL Extended Low Count Timing Register. */
    i3c->I3C_SCL_EXT_LCNT_TIMING = 0U;
}

/**
  \fn           void i3c_normal_bus_clk_cfg(I3C_Type *i3c,
  \                                         const uint32_t  core_clk)
  \brief        i3c normal bus clock configuration for i3c slave device
  \note         This function sets the clock timings as per MIPI I3C std
  \param[in]    i3c       : Pointer to i3c register set structure
  \param[in]    core_clk  : core clock
  \return       none
*/
void i3c_normal_bus_clk_cfg(I3C_Type *i3c, const uint32_t core_clk)
{
    unsigned long core_rate, core_period;
    uint32_t      scl_timing;
    uint8_t       hcnt, lcnt;

    core_rate   = core_clk;

    /* Calculate core clk period */
    core_period = DIV_ROUND_UP(REF_CLK_RATE, core_rate);

    /* Calculate SCL push-pull High and Low count for
     *  I3C transfers targeted to I3C devices.*/
    hcnt        = DIV_ROUND_UP(I3C_NORMAL_BUS_THIGH_NS, core_period) - 1;
    if (hcnt < I3C_SCL_I3C_TIMING_CNT_MIN) {
        hcnt = I3C_SCL_I3C_TIMING_CNT_MIN;
    }

    lcnt = DIV_ROUND_UP(core_rate, I3C_BUS_SDR0_SCL_RATE) - hcnt;
    if (lcnt < I3C_SCL_I3C_TIMING_CNT_MIN) {
        lcnt = I3C_SCL_I3C_TIMING_CNT_MIN;
    }

    scl_timing =
        (I3C_SCL_I3C_PP_TIMING_I3C_PP_HCNT(hcnt) | I3C_SCL_I3C_PP_TIMING_I3C_PP_LCNT(lcnt));
    i3c->I3C_SCL_I3C_PP_TIMING = scl_timing;

    /* set the Bus free time for initiating the transfer in master mode.*/
    if (!(i3c->I3C_DEVICE_CTRL & I3C_DEVICE_CTRL_I2C_SLAVE_PRESENT)) {
        i3c->I3C_BUS_FREE_AVAIL_TIMING = I3C_BUS_FREE_AVAIL_TIMING_BUS_FREE_TIME(lcnt);
    }

    /* SCL open-drain High and Low count (I3C) for
     *  I3C transfers targeted to I3C devices*/
    lcnt = DIV_ROUND_UP(I3C_NORMAL_BUS_TLOW_OD_NS, core_period);
    scl_timing =
        (I3C_SCL_I3C_OD_TIMING_I3C_OD_HCNT(hcnt) | I3C_SCL_I3C_OD_TIMING_I3C_OD_LCNT(lcnt));
    i3c->I3C_SCL_I3C_OD_TIMING    = scl_timing;

    /* set SCL Extended Low Count Timing Register. */

    /* Calculate the minimum low count for SDR1 */
    lcnt                          = DIV_ROUND_UP(core_rate, I3C_BUS_SDR1_SCL_RATE) - hcnt;
    scl_timing                    = I3C_SCL_EXT_LCNT_TIMING_I3C_EXT_LCNT_1(lcnt);

    /* Calculate the minimum low count for SDR2 */
    lcnt                          = DIV_ROUND_UP(core_rate, I3C_BUS_SDR2_SCL_RATE) - hcnt;
    scl_timing                   |= I3C_SCL_EXT_LCNT_TIMING_I3C_EXT_LCNT_2(lcnt);

    /* Calculate the minimum low count for SDR3 */
    lcnt                          = DIV_ROUND_UP(core_rate, I3C_BUS_SDR3_SCL_RATE) - hcnt;
    scl_timing                   |= I3C_SCL_EXT_LCNT_TIMING_I3C_EXT_LCNT_3(lcnt);

    /* Calculate the minimum low count for SDR4 */
    lcnt                          = DIV_ROUND_UP(core_rate, I3C_BUS_SDR4_SCL_RATE) - hcnt;
    scl_timing                   |= I3C_SCL_EXT_LCNT_TIMING_I3C_EXT_LCNT_4(lcnt);

    i3c->I3C_SCL_EXT_LCNT_TIMING  = scl_timing;
}

/**
  \fn           void i2c_clk_cfg(I3C_Type           *i3c,
                                 uint32_t            core_clk,
                                 I3C_I2C_SPEED_MODE  i2c_speed_mode)
  \brief        i3c clock configuration for legacy i2c slave device
  \param[in]    i3c             : Pointer to i3c register set structure
  \param[in]    core_clk        : core clock
  \param[in]    i2c_speed_mode  : i2c Speed mode
                 I3C_I2C_SPEED_MODE_FMP_1_MBPS  : Fast Mode Plus 1   MBPS
                 I3C_I2C_SPEED_MODE_FM_400_KBPS : Fast Mode      400 KBPS
                 I3C_I2C_SPEED_MODE_SS_100_KBPS : Standard Mode  100 KBPS
  \return        none
*/
void i2c_clk_cfg(I3C_Type *i3c, uint32_t core_clk, I3C_I2C_SPEED_MODE i2c_speed_mode)
{
    unsigned long core_rate   = 0U;
    unsigned long core_period = 0U;
    uint16_t      hcnt        = 0U;
    uint16_t      lcnt        = 0U;
    uint32_t      scl_timing  = 0U;

    core_rate                 = core_clk;

    /* Calculate core clk period */
    core_period               = DIV_ROUND_UP(REF_CLK_RATE, core_rate);

    /* Speed Mode: Fast Mode Plus >1 MBPS (approximately 3.124 MBPS) */
    if (i2c_speed_mode == I3C_I2C_SPEED_MODE_FMP_1_MBPS) {
        /* Calculate the SCL clock high period and low period count for
         *  I2C Fast Mode Plus transfers. */
        lcnt = DIV_ROUND_UP(I3C_BUS_I2C_FMP_TLOW_MIN_NS, core_period);
        hcnt = DIV_ROUND_UP(core_rate, I3C_BUS_I2C_FM_PLUS_SCL_RATE) - lcnt;

        scl_timing =
            (I3C_SCL_I2C_FMP_TIMING_I2C_FMP_HCNT(hcnt) | I3C_SCL_I2C_FMP_TIMING_I2C_FMP_LCNT(lcnt));

        i3c->I3C_SCL_I2C_FMP_TIMING = scl_timing;
    }

    /* Speed Mode: Fast Mode 400 KBPS */
    if (i2c_speed_mode == I3C_I2C_SPEED_MODE_FM_400_KBPS) {
        /* Calculate the SCL clock high period and low period count for
         *  I2C Fast Mode transfers. */
        lcnt = DIV_ROUND_UP(I3C_BUS_I2C_FM_TLOW_MIN_NS, core_period);
        hcnt = DIV_ROUND_UP(core_rate, I3C_BUS_I2C_FM_SCL_RATE) - lcnt;

        scl_timing =
            (I3C_SCL_I2C_FM_TIMING_I2C_FM_HCNT(hcnt) | I3C_SCL_I2C_FM_TIMING_I2C_FM_LCNT(lcnt));

        /* Set the high and low period count for FM mode */
        i3c->I3C_SCL_I2C_FM_TIMING = scl_timing;
    }

    /* Speed Mode: Standard Mode 100 KBPS */
    if (i2c_speed_mode == I3C_I2C_SPEED_MODE_SS_100_KBPS) {
        /* Calculate the SCL clock high period and low period count for
         *  I2C Fast Mode transfers. */
        lcnt = DIV_ROUND_UP(I3C_BUS_I2C_SS_TLOW_MIN_NS, core_period);
        hcnt = DIV_ROUND_UP(core_rate, I3C_BUS_I2C_SS_SCL_RATE) - lcnt;

        scl_timing =
            (I3C_SCL_I2C_FM_TIMING_I2C_FM_HCNT(hcnt) | I3C_SCL_I2C_FM_TIMING_I2C_FM_LCNT(lcnt));

        i3c->I3C_SCL_I2C_FM_TIMING = scl_timing;
    }

    /* set the Bus free time for initiating the transfer in master mode.*/
    i3c->I3C_BUS_FREE_AVAIL_TIMING  = I3C_BUS_FREE_AVAIL_TIMING_BUS_FREE_TIME(lcnt);

    /* Set as legacy i2c device is present. */
    i3c->I3C_DEVICE_CTRL           |= I3C_DEVICE_CTRL_I2C_SLAVE_PRESENT;
}

/**
  \fn           void i3c_master_init(I3C_Type *i3c)
  \brief        Initialize i3c master.
                 This function will :
                  - Set mode as master
                  - Clear Command Queue and Data buffer Queue
                  - Enable i3c controller
  \param[in]    i3c  : Pointer to i3c register
                        set structure
  \return       none
*/
void i3c_master_init(I3C_Type *i3c)
{
    i3c->I3C_QUEUE_THLD_CTRL &=
        ~(I3C_QUEUE_THLD_CTRL_IBI_STATUS_THLD_Msk | I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD_Msk);

    i3c->I3C_DATA_BUFFER_THLD_CTRL &= ~I3C_DATA_BUFFER_THLD_CTRL_RX_BUF_THLD_Msk;

    /* Set operation mode as master */
    i3c->I3C_DEVICE_CTRL_EXTENDED  &= ~(I3C_DEVICE_CTRL_EXTENDED_DEV_OPERATION_MODE_Msk |
                                       I3C_DEVICE_CTRL_EXTENDED_DEV_OP_MODE_SLV);

    /* configure as a master */
    i3c->I3C_SLV_CHAR_CTRL         |= I3C_SLV_CHAR_CTRL_DEVICE_ROLE_MASTER;

    /* Enable i3c controller. */
    i3c->I3C_DEVICE_CTRL           |= I3C_DEVICE_CTRL_ENABLE;
}

/**
  \fn           void i3c_slave_init(I3C_Type *i3c,
                                    const uint8_t  slv_addr,
                                    const uint32_t core_clk)
  \brief        Initialize i3c slave.
                 This function will :
                  - set slave static address
                  - set secondary master as slave mode
                  - Enable i3c controller
  \param[in]    i3c       : Pointer to i3c register
                             set structure
  \param[in]    slv_addr  : Slave own Address
  \param[in]    core_clk  : core clock
  \return       none
*/
void i3c_slave_init(I3C_Type *i3c, const uint8_t slv_addr, const uint32_t core_clk)
{
    int32_t  val;
    uint32_t clk_period;
    uint32_t timing;

    /* Calculate core clk period */
    clk_period            = DIV_ROUND_UP(REF_CLK_RATE, core_clk);

    /* As per mipi_i3c_user Section 5 */

    /* DEVICE_ADDR to set static addr with its valid bit */
    i3c->I3C_DEVICE_ADDR |= (I3C_DEVICE_ADDR_STATIC_ADDR_VALID | slv_addr);

    /* Response buffer threshold */
    i3c->I3C_QUEUE_THLD_CTRL &=
        ~(I3C_QUEUE_THLD_CTRL_IBI_STATUS_THLD_Msk | I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD_Msk);

    i3c->I3C_DATA_BUFFER_THLD_CTRL &= ~I3C_DATA_BUFFER_THLD_CTRL_RX_BUF_THLD_Msk;

    /* Slave bus control DEVICE_CTRL_EXTENDED */
    val                             = i3c->I3C_DEVICE_CTRL_EXTENDED;
    val                            |= I3C_DEVICE_CTRL_EXTENDED_REQMST_ACK_CTRL;
    val                            &= ~I3C_DEVICE_CTRL_EXTENDED_DEV_OPERATION_MODE_Msk;
    val                            |= I3C_DEVICE_CTRL_EXTENDED_DEV_OP_MODE_SLV;
    i3c->I3C_DEVICE_CTRL_EXTENDED   = val;

    /* configure as a slave */
    i3c->I3C_SLV_CHAR_CTRL         &= ~I3C_SLV_CHAR_CTRL_DEVICE_ROLE_MASTER;

    timing                          = DIV_ROUND_UP(I3C_BUS_AVAILABLE_TIME_NS, clk_period);

    /* Sets bus available time to 1us as per MIPI spec */
    i3c->I3C_BUS_FREE_AVAIL_TIMING  = I3C_BUS_FREE_AVAIL_TIMING_BUS_AVAILABLE_TIME(timing);
    timing                          = DIV_ROUND_UP(I3C_BUS_IDLE_TIME_NS, clk_period);

    i3c->I3C_BUS_IDLE_TIMING        = I3C_BUS_IDLE_TIMING_BUS_IDLE_TIME(timing);

    /* Enable i3c controller. */
    i3c->I3C_DEVICE_CTRL           |= I3C_DEVICE_CTRL_ENABLE;
}

/**
  \fn           void i3c_master_setup_cmd(I3C_Type *i3c,
  \                                      const i3c_xfer_t xfer)
  \brief        Set command queue intterupt
  \param[in]    i3c     : Pointer to i3c register set structure
  \param[in]    xfer    : Transfer command structure
  \return       none
*/
void i3c_master_setup_cmd(I3C_Type *i3c, const i3c_xfer_t xfer)
{
    uint32_t temp =
        (i3c->I3C_QUEUE_THLD_CTRL &
         (~(I3C_QUEUE_THLD_CTRL_CMD_EMPTY_BUF_THLD_Msk | I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD_Msk)));

    /* Set response threshold to 1 */
    i3c->I3C_QUEUE_THLD_CTRL = (temp | I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD(1));

    temp                     = I3C_INTR_STATUS_EN_CMD_QUEUE_READY_STS_EN;

    switch (xfer.xfer_cmd.cmd_type) {
    case I3C_XFER_CCC_SET:
        /* Set tx buffer threshold */
        i3c_set_tx_buf_thld(i3c, xfer.tx_len);
        if (xfer.tx_len) {
            /* Enable Tx Threshold status interrupt */
            temp |= I3C_INTR_STATUS_EN_TX_THLD_STS_EN;
        }
        break;

    case I3C_XFER_CCC_GET:
        /* Set rx buffer threshold */
        i3c_set_rx_buf_thld(i3c, xfer.rx_len);

        /* Enable Rx Threshold status interrupt */
        temp |= I3C_INTR_STATUS_EN_RX_THLD_STS_EN;
        break;

    case I3C_XFER_TYPE_ADDR_ASSIGN:
        break;

    default:
        return;
    }

    /* Enable interrupt */
    i3c_enable_intr(i3c, temp);
}

/**
  \fn           void i3c_setup_tx(I3C_Type *i3c,
  \                               const uint16_t len)
  \brief        Setup Data Tx
  \param[in]    i3c    : Pointer to i3c register set structure
  \param[in]    len    : Data length in bytes
  \return       none
*/
void i3c_setup_tx(I3C_Type *i3c, const uint16_t len)
{
    uint32_t temp =
        (i3c->I3C_QUEUE_THLD_CTRL &
         (~(I3C_QUEUE_THLD_CTRL_CMD_EMPTY_BUF_THLD_Msk | I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD_Msk)));

    /* Set response threshold to 1 */
    temp                     |= I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD(1);
    i3c->I3C_QUEUE_THLD_CTRL  = temp;

    /* Set tx buffer threshold */
    i3c_set_tx_buf_thld(i3c, len);

    if (i3c_is_dma_enable(i3c)) {
        /* Enable only command queue ready status interrupt if
         * DMA is enabled */
        temp = I3C_INTR_STATUS_EN_CMD_QUEUE_READY_STS_EN;
    } else {
        /* Enable command queue ready status with
         * Tx threshold status interrupt if DMA is not enabled */
        temp = (I3C_INTR_STATUS_EN_TX_THLD_STS_EN | I3C_INTR_STATUS_EN_CMD_QUEUE_READY_STS_EN);
    }

    /* Enable interrupt */
    i3c_enable_intr(i3c, temp);
}

/**
  \fn           void i3c_setup_rx(I3C_Type *i3c,
  \                               const uint16_t len)
  \brief        Setup Data Rx
  \param[in]    i3c    : Pointer to i3c register set structure
  \param[in]    len    : Data length in bytes
  \return       none
*/
void i3c_setup_rx(I3C_Type *i3c, const uint16_t len)
{
    uint32_t temp =
        (i3c->I3C_QUEUE_THLD_CTRL &
         (~(I3C_QUEUE_THLD_CTRL_CMD_EMPTY_BUF_THLD_Msk | I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD_Msk)));

    /* Set response threshold to 1 */
    temp                     |= I3C_QUEUE_THLD_CTRL_RESP_BUF_THLD(1);
    i3c->I3C_QUEUE_THLD_CTRL  = temp;

    /* Set rx buffer threshold */
    i3c_set_rx_buf_thld(i3c, len);

    if (i3c_is_dma_enable(i3c)) {
        /* Enable only command queue ready status interrupt if
         * DMA is enabled */
        temp = I3C_INTR_STATUS_EN_CMD_QUEUE_READY_STS_EN;
    } else {
        if (i3c_is_master(i3c)) {
            /* Enable command queue ready status with
             * Rx threshold status interrupt if master */
            temp = (I3C_INTR_STATUS_EN_RX_THLD_STS_EN | I3C_INTR_STATUS_EN_CMD_QUEUE_READY_STS_EN);
        } else {
            /* Enable Rx threshold status interrupt if slave */
            temp = I3C_INTR_STATUS_EN_RX_THLD_STS_EN;
        }
    }

    /* Enable interrupt */
    i3c_enable_intr(i3c, temp);
}

/**
  \fn           void i3c_master_irq_handler(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        i3c interrupt service routine for master
  \param[in]    i3c  : Pointer to i3c register set structure
  \param[in]    xfer : Pointer to i3c transfer structure
  \return       none
*/
void i3c_master_irq_handler(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint32_t status = 0U;
    uint32_t nresp  = 0U;
    uint32_t resp   = 0U;
    uint32_t tid    = 0U;
    uint32_t rx_len = 0U;

    status          = i3c->I3C_INTR_STATUS;

    if (!(status & i3c->I3C_INTR_STATUS_EN)) {
        /* there are no interrupts that we are interested in */
        i3c_clear_intr(i3c, I3C_INTR_STATUS_ALL);
        return;
    }

    if (status & I3C_INTR_STATUS_RX_THLD_STS) {
        if ((xfer->rx_buf) && (xfer->rx_cur_cnt < xfer->rx_len)) {
            resp = i3c_get_avail_rx_buf_len(i3c);
            i3c_receive(i3c, xfer, resp);

            if (xfer->rx_cur_cnt >= xfer->rx_len) {
                i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_RX_THLD_STS_EN);
            }
        }
    } else if (status & I3C_INTR_STATUS_TX_THLD_STS) {
        /* write data to tx port (if any) */
        if ((xfer->tx_buf) && (xfer->tx_cur_cnt < xfer->tx_len)) {
            resp = i3c_get_empty_tx_buf_len(i3c);
            i3c_send(i3c, xfer, resp);

            if (xfer->tx_cur_cnt >= xfer->tx_len) {
                i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_TX_THLD_STS_EN);
            }
        }
    }

    if (status & I3C_INTR_STATUS_CMD_QUEUE_READY_STS) {
        i3c->I3C_QUEUE_THLD_CTRL &= ~I3C_QUEUE_THLD_CTRL_CMD_EMPTY_BUF_THLD_Msk;
        i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_CMD_QUEUE_READY_STS_EN);
        i3c_set_port(i3c, xfer);
        if (xfer->xfer_cmd.port_id) {
            /* Dispatch commands to i3c Command Queue */
            i3c_dispatch_xfer_cmd(i3c, xfer);
        }
        xfer->xfer_cmd.cmd_type = I3C_XFER_TYPE_NONE;
    }

    /* Checks for Busowner updated status */
    if (status & I3C_INTR_STATUS_BUSOWNER_UPDATED_STS) {
        /* Disable Bus Ownership update interrupt */
        i3c_disable_intr(i3c,
                         I3C_INTR_STATUS_EN_BUSOWNER_UPDATED_STS_EN |
                             I3C_INTR_STATUS_EN_RX_THLD_STS_EN);

        i3c_receive(i3c, xfer, xfer->rx_len);

        xfer->status = (I3C_XFER_STATUS_BUSOWNER_UPDATED | I3C_XFER_STATUS_DONE);
        i3c_clear_intr(i3c, I3C_INTR_STATUS_BUSOWNER_UPDATED_STS);
    }

    /* we are only interested in a response interrupt,
     * make sure we have a response */
    nresp = i3c->I3C_QUEUE_STATUS_LEVEL;

    if (I3C_QUEUE_STATUS_LEVEL_RESP_BUF_BLR(nresp)) {
        resp        = i3c->I3C_RESPONSE_QUEUE_PORT;

        rx_len      = I3C_RESPONSE_QUEUE_PORT_DATA_LEN(resp);
        xfer->error = I3C_RESPONSE_QUEUE_PORT_ERR_STATUS(resp);

        tid         = I3C_RESPONSE_QUEUE_PORT_TID(resp);
    }

    /* Checks for the availability of IBI */
    if ((I3C_QUEUE_STATUS_LEVEL_IBI_BUF_BLR(nresp)) && (status & I3C_INTR_STATUS_IBI_THLD_STS)) {
        i3c_ibi_handler(i3c, xfer);
    }

    if (xfer->error) {
        /* Fetches error type */
        i3c_fetch_error_type(xfer);

        /* Invokes error handler */
        i3c_error_handler(i3c, xfer, status, resp);
    } else {
        switch (tid) {
        case I3C_MST_TX_TID:
        case I3C_CCC_SET_TID:
            if (tid == I3C_MST_TX_TID) {
                xfer->status = I3C_XFER_STATUS_MST_TX_DONE;
            }

            else if (tid == I3C_CCC_SET_TID) {
                xfer->status = I3C_XFER_STATUS_CCC_SET_DONE;
            }

            /* mark all success event also as Transfer DONE */
            xfer->status |= I3C_XFER_STATUS_DONE;
            break;

        case I3C_MST_RX_TID:
        case I3C_CCC_GET_TID:
            if (xfer->rx_len) {
                i3c_receive(i3c, xfer, rx_len);

                if (xfer->rx_cur_cnt >= xfer->rx_len) {
                    i3c_disable_intr(i3c, I3C_INTR_STATUS_RX_THLD_STS);
                }

                if (tid == I3C_MST_RX_TID) {
                    xfer->status |= I3C_XFER_STATUS_MST_RX_DONE;
                }

                else if (tid == I3C_CCC_GET_TID) {
                    xfer->status |= I3C_XFER_STATUS_CCC_GET_DONE;
                }

                /* mark all success event also as Transfer DONE */
                xfer->status |= I3C_XFER_STATUS_DONE;
            }
            break;

        case I3C_ADDR_ASSIGN_TID:
            /* mark all success event also as Transfer DONE */
            xfer->status |= (I3C_XFER_STATUS_ADDR_ASSIGN_DONE | I3C_XFER_STATUS_DONE);
            break;

        default:
            break;
        }
    }

    /* Clear the bus reset status */
    if (status & I3C_INTR_STATUS_BUS_RESET_DONE_STS) {
        /* Disable Bus Reset interrupt */
        i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_BUS_RESET_DONE_STS_EN);

        i3c->I3C_INTR_STATUS |= I3C_INTR_STATUS_BUS_RESET_DONE_STS;
        xfer->status         |= (I3C_XFER_STATUS_DONE | I3C_XFER_STATUS_BUS_RESET_DONE);
    }
}

/**
  \fn           void i3c_slave_irq_handler(I3C_Type *i3c, i3c_xfer_t *xfer)
  \brief        i3c interrupt service routine for slave
  \param[in]    i3c  : Pointer to i3c register set structure
  \param[in]    xfer : Pointer to i3c transfer structure
  \return       none
*/
void i3c_slave_irq_handler(I3C_Type *i3c, i3c_xfer_t *xfer)
{
    uint32_t status = 0U;
    uint32_t nresp  = 0U;
    uint32_t resp   = 0U;
    uint32_t tid    = 0U;
    uint16_t rx_len = 0U;

    status          = i3c->I3C_INTR_STATUS;

    if (!(status & i3c->I3C_INTR_STATUS_EN)) {
        /* there are no interrupts that we are interested in */
        i3c_clear_intr(i3c, I3C_INTR_STATUS_ALL);
    } else {
        if (status & I3C_INTR_STATUS_RX_THLD_STS) {
            resp = i3c_get_avail_rx_buf_len(i3c);
            i3c_receive(i3c, xfer, resp);

            if (xfer->rx_cur_cnt >= xfer->rx_len) {
                i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_RX_THLD_STS_EN);
            }
        }

        else if (status & I3C_INTR_STATUS_TX_THLD_STS) {
            /* write data to tx port (if any) */
            if ((xfer->tx_buf) && (xfer->tx_cur_cnt < xfer->tx_len)) {
                resp = i3c_get_empty_tx_buf_len(i3c);
                i3c_send(i3c, xfer, resp);

                if (xfer->tx_cur_cnt >= xfer->tx_len) {
                    i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_TX_THLD_STS_EN);
                }
            }
        }

        if (status & I3C_INTR_STATUS_CMD_QUEUE_READY_STS) {
            i3c->I3C_QUEUE_THLD_CTRL &= ~I3C_QUEUE_THLD_CTRL_CMD_EMPTY_BUF_THLD_Msk;
            i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_CMD_QUEUE_READY_STS_EN);

            i3c_set_port(i3c, xfer);
            if (xfer->xfer_cmd.port_id == I3C_SLV_TX_TID) {
                /* As per mipi_i3c_databook Section 2.7.13
                 * no command required for transmit,
                 * only data length is required
                 */
                i3c->I3C_COMMAND_QUEUE_PORT =
                    I3C_COMMAND_QUEUE_PORT_ARG_DATA_LEN(xfer->xfer_cmd.data_len) |
                    I3C_COMMAND_QUEUE_PORT_SLV_PORT_TID(xfer->xfer_cmd.port_id);
            }
            xfer->xfer_cmd.cmd_type = I3C_XFER_TYPE_NONE;
        }

        /* Checking for dynamic address valid */
        if (status & I3C_INTR_STATUS_DYN_ADDR_ASSGN_STS) {
            i3c_clear_intr(i3c, I3C_INTR_STATUS_DYN_ADDR_ASSGN_STS);
            xfer->status = (I3C_XFER_STATUS_DONE | I3C_XFER_STATUS_SLV_DYN_ADDR_ASSGN);
        }
        /* Checks for IBI updated status */
        else if (status & I3C_INTR_STATUS_IBI_UPDATED_STS) {
            i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_IBI_UPDATED_STS_EN);
            i3c_ibi_handler(i3c, xfer);
            i3c_clear_intr(i3c, I3C_INTR_STATUS_IBI_UPDATED_STS);
        }
        /* Checks for Busowner updated status */
        else if (status & I3C_INTR_STATUS_BUSOWNER_UPDATED_STS) {
            /* Disable updated ownership interrupt */
            i3c_disable_intr(i3c, I3C_INTR_STATUS_EN_BUSOWNER_UPDATED_STS_EN);
            xfer->status = (I3C_XFER_STATUS_DONE | I3C_XFER_STATUS_BUSOWNER_UPDATED);
            i3c_clear_intr(i3c, I3C_INTR_STATUS_BUSOWNER_UPDATED_STS);
        } else if (status & I3C_INTR_STATUS_CCC_UPDATED_STS) {
            if (i3c->I3C_SLV_EVENT_STATUS & I3C_SLV_EVENT_STATUS_MWL_UPDATED) {
                i3c->I3C_SLV_EVENT_STATUS |= I3C_SLV_EVENT_STATUS_MWL_UPDATED;
            } else if (i3c->I3C_SLV_EVENT_STATUS & I3C_SLV_EVENT_STATUS_MRL_UPDATED) {
                i3c->I3C_SLV_EVENT_STATUS |= I3C_SLV_EVENT_STATUS_MRL_UPDATED;
                /* Resume the device as it is halted for Rd len updated reason */
                i3c_resume(i3c);
            }

            i3c_clear_intr(i3c, I3C_INTR_STATUS_CCC_UPDATED_STS);

            xfer->status = (I3C_XFER_STATUS_DONE | I3C_XFER_STATUS_SLV_CCC_UPDATED);
        }
        /* we are only interested in a response interrupt,
         *  make sure we have a response */
        nresp = i3c->I3C_QUEUE_STATUS_LEVEL;
        nresp = I3C_QUEUE_STATUS_LEVEL_RESP_BUF_BLR(nresp);

        if (!nresp) {
            return;
        }

        resp        = i3c->I3C_RESPONSE_QUEUE_PORT;

        rx_len      = I3C_RESPONSE_QUEUE_PORT_DATA_LEN(resp);
        xfer->error = I3C_RESPONSE_QUEUE_PORT_ERR_STATUS(resp);

        if (xfer->error) {
            /* Fetches error type */
            i3c_fetch_error_type(xfer);

            /* Invokes error handler */
            i3c_error_handler(i3c, xfer, status, resp);
        } else {
            tid = I3C_RESPONSE_QUEUE_PORT_TID(resp);

            switch (tid) {
            case I3C_SLV_TX_TID:
                /* mark all success event also as Transfer DONE */
                xfer->status |= (I3C_XFER_STATUS_DONE | I3C_XFER_STATUS_SLV_TX_DONE);
                break;

            case I3C_SLV_RX_TID:
                if (xfer->rx_len) {
                    i3c_receive(i3c, xfer, rx_len);

                    if (xfer->rx_cur_cnt >= xfer->rx_len) {
                        i3c_disable_intr(i3c, I3C_INTR_STATUS_RX_THLD_STS);
                    }

                    /* mark all success event also as Transfer DONE */
                    xfer->status |= (I3C_XFER_STATUS_DONE | I3C_XFER_STATUS_SLV_RX_DONE);
                }
                break;
            case I3C_SLV_DEFSLVS_TID:
                if (rx_len) {
                    /* mark all success event also as Transfer DONE */
                    xfer->status |= (I3C_XFER_STATUS_DONE | I3C_XFER_STATUS_DEFSLV_LIST);

                    i3c_clear_intr(i3c, I3C_INTR_STATUS_DEFSLV_STS);

                    /* Store number of slaves' DEFSLVS data rcvd */
                    xfer->addr_len = rx_len;
                }
                break;
            default:
                break;
            }
        }
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
