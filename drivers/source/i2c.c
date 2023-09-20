/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "i2c.h"

/**
 * @brief   Set scl count
 * @note    none
 * @param   i2c        : Pointer to i2c register map
 * @param   clk_khz    : Clock set SCL
 * @param   speed_mode : Speed
 * @retval  none
 */
static void i2c_set_scl_cnt(I2C_Type *i2c,uint32_t clk_khz, uint8_t speed_mode)
 {

  uint32_t clk_ns;
  clk_ns = 1000000 / clk_khz;

  uint32_t standard_speed_scl_hcnt;       /* value for I2C_SS_SCL_HCNT */
  uint32_t standard_speed_scl_lcnt;       /* value for ic_ss_scl_lcnt */
  uint32_t fast_speed_scl_hcnt;           /* value for ic_fs_scl_hcnt */
  uint32_t fast_speed_scl_lcnt;           /* value for I2C_FS_SCL_LCNT */

  if (clk_khz <= 1000000) {
    if (speed_mode == I2C_SPEED_STANDARD)
    {
        /* Calculate count value for standard speed */
        standard_speed_scl_hcnt = I2C_MIN_SS_HIGH_TIME_NS/clk_ns;
        if ((I2C_MIN_SS_HIGH_TIME_NS % clk_ns) != 0) {
          standard_speed_scl_hcnt += 1;
        }
        standard_speed_scl_lcnt = I2C_MIN_SS_LOW_TIME_NS/clk_ns;
        if ((I2C_MIN_SS_LOW_TIME_NS % clk_ns) != 0) {
        standard_speed_scl_lcnt += 1;
        }
    }

    if (speed_mode == I2C_SPEED_FAST)
    {
        /* Calculate count value for fast speed */
        fast_speed_scl_hcnt = I2C_MIN_FS_HIGH_TIME_NS/clk_ns;
        if ((I2C_MIN_FS_HIGH_TIME_NS % clk_ns) != 0) {
          fast_speed_scl_hcnt += 1;
        }
        fast_speed_scl_lcnt = I2C_MIN_FS_LOW_TIME_NS/clk_ns;
        if ((I2C_MIN_FS_LOW_TIME_NS % clk_ns) != 0) {
          fast_speed_scl_lcnt += 1;
        }
    }

    if (speed_mode == I2C_SPEED_FASTPLUS)
    {
        /* Calculate count value for fast speed */
        fast_speed_scl_hcnt = I2C_MIN_FS_PLUS_HIGH_TIME_NS/clk_ns;
        if ((I2C_MIN_FS_HIGH_TIME_NS % clk_ns) != 0) {
          fast_speed_scl_hcnt += 1;
        }
        fast_speed_scl_lcnt = I2C_MIN_FS_PLUS_LOW_TIME_NS/clk_ns;
        if ((I2C_MIN_FS_LOW_TIME_NS % clk_ns) != 0) {
          fast_speed_scl_lcnt += 1;
        }
    }
    } else {
        if (speed_mode == I2C_SPEED_STANDARD)
        {
            /* Calculate count value for standard speed */
            standard_speed_scl_hcnt = I2C_MIN_SS_HIGH_TIME_NS*clk_ns;
            standard_speed_scl_lcnt = I2C_MIN_SS_LOW_TIME_NS*clk_ns;
        }
        /* Calculate count value for fast speed */
        if (speed_mode == I2C_SPEED_FAST)
        {
            fast_speed_scl_hcnt = I2C_MIN_FS_HIGH_TIME_NS*clk_ns;
            fast_speed_scl_lcnt = I2C_MIN_FS_LOW_TIME_NS*clk_ns;
        }

        if (speed_mode == I2C_SPEED_FASTPLUS)
        {
            fast_speed_scl_hcnt = I2C_MIN_FS_PLUS_HIGH_TIME_NS*clk_ns;
            fast_speed_scl_lcnt = I2C_MIN_FS_PLUS_LOW_TIME_NS*clk_ns;
        }
    }
    if (standard_speed_scl_hcnt < I2C_MIN_SS_SCL_HCNT(i2c->I2C_FS_SPKLEN)) {
        standard_speed_scl_hcnt = I2C_MIN_SS_SCL_HCNT(i2c->I2C_FS_SPKLEN);
    }
    if (standard_speed_scl_lcnt < I2C_MIN_SS_SCL_LCNT(i2c->I2C_FS_SPKLEN)) {
        standard_speed_scl_lcnt = I2C_MIN_SS_SCL_LCNT(i2c->I2C_FS_SPKLEN);
    }
    if (fast_speed_scl_hcnt < I2C_MIN_FS_SCL_HCNT(i2c->I2C_FS_SPKLEN)) {
        fast_speed_scl_hcnt = I2C_MIN_FS_SCL_HCNT(i2c->I2C_FS_SPKLEN);
    }
    if (fast_speed_scl_lcnt < I2C_MIN_FS_SCL_LCNT(i2c->I2C_FS_SPKLEN)) {
        fast_speed_scl_lcnt = I2C_MIN_FS_SCL_LCNT(i2c->I2C_FS_SPKLEN);
    }

  i2c_disable(i2c);
  i2c->I2C_SS_SCL_HCNT = standard_speed_scl_hcnt;
  i2c->I2C_SS_SCL_LCNT = standard_speed_scl_lcnt;
  i2c->I2C_FS_SCL_HCNT = fast_speed_scl_hcnt;
  i2c->I2C_FS_SCL_LCNT = fast_speed_scl_lcnt;
  i2c_enable(i2c);

 }

 /**
  * @brief   Set spike length
  * @note    none
  * @param   i2c     : Pointer to i2c register map
  * @param   clk_khz : Clock set SCL
  * @retval  none
  */
static void i2c_set_spike_len(I2C_Type *i2c, uint32_t clk_khz)
{
 /* Reference has take from the databook section 2.15 */
 /* Reference has take from the databook section 2.15 */
  uint32_t clk_ns;
  uint32_t fs_spike_length = 0;

  if (clk_khz <= 1000000)
  {
      clk_ns = 1000000 / clk_khz;
      fs_spike_length = I2C_FS_SPIKE_LENGTH_NS/clk_ns;
      if ((I2C_FS_SPIKE_LENGTH_NS % clk_ns) != 0) {
          fs_spike_length += 1;
      }

  }
  else {
      clk_ns = clk_khz / 1000000;
      fs_spike_length = I2C_FS_SPIKE_LENGTH_NS*clk_ns;
  }

  i2c_disable(i2c);
  i2c->I2C_FS_SPKLEN = fs_spike_length;
  i2c_enable(i2c);
}

/**
 * @brief   check whether i2c is ready to transmit, 1 ready, 0 not ready
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  1 ready to transmit, 0 not ready to transmit
 */
static inline int32_t i2c_tx_ready(I2C_Type *i2c)
{
  return ( (i2c->I2C_STATUS & I2C_IC_STATUS_TRANSMIT_FIFO_NOT_FULL) ? 1 : 0);
}

/**
 * @brief   check whether i2c is ready to receive, 1 ready, 0 not ready
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  1 ready to transmit, 0 not ready to transmit
 */
static inline int32_t i2c_rx_ready(I2C_Type *i2c)
{
  return ( (i2c->I2C_STATUS & I2C_IC_STATUS_RECEIVE_FIFO_NOT_EMPTY) ? 1 : 0);
}

/**
 * @brief   check any error for master
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  none
 */
static int32_t i2c_master_check_error(I2C_Type *i2c, i2c_transfer_info_t *transfer)
{
  uint32_t status;
  int32_t ercd = I2C_ERR_NONE;

  status = i2c->I2C_RAW_INTR_STAT;

  /*transmit abort*/
  if (status & I2C_IC_INTR_STAT_TX_ABRT)
  {
    status = i2c->I2C_TX_ABRT_SOURCE;

    if (status & I2C_MST_ABRT_LOST_BUS)
    {
        ercd = I2C_ERR_LOST_BUS;
    }
    else if (status & I2C_MST_ABRT_ADDR_NOACK)
    {
        ercd = I2C_ERR_ADDR_NOACK;
    }
    /* master got ack from slave for 7/10bit addr then
     * master sends data, no ack for the data */
    else if (status & I2C_MST_ABRT_DATA_NOACK)
    {
        ercd = I2C_ERR_DATA_NOACK;
    }
    else if (status & I2C_IC_TX_ABRT_10B_RD_NORSTRT)
    {
        ercd = I2C_ERR_10B_RD_NORSTRT;
    }
    else
    {
        ercd = I2C_ERR_UNDEF;
    }
    status = i2c->I2C_CLR_TX_ABRT;
  }
  else
  {
    /*during transmit set once TX_fifo is at max buffer_length and processor \
     * sends another i2c cmd by writing to IC_DATA_CMD */
    if (status & I2C_IC_INTR_STAT_TX_OVER)
    {
      transfer->tx_over++;
      /* clear reg */
      status = i2c->I2C_CLR_TX_OVER;
    }

    /* RX_fifo full and received one more byte | RX_fifo is empty \
     * and trying to read from ic_data_cmd */
    if (status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
    {
        transfer->rx_over++;
        /* clear reg */
        status = i2c->I2C_CLR_RX_OVER;
        status = i2c->I2C_CLR_RX_UNDER;
    }
  }
  return ercd;
}

/**
 * @brief   check any error for slave
 * @note    none
 * @param   i2c    : Pointer to i2c register map
 * @retval  none
 */
static int32_t i2c_slave_check_error(I2C_Type *i2c, i2c_transfer_info_t *transfer)
{
    uint32_t status;
    int32_t ercd = I2C_ERR_NONE;

    status = i2c->I2C_RAW_INTR_STAT;

    if (status & I2C_IC_INTR_STAT_START_DET)
    {
        status = i2c->I2C_CLR_START_DET;
    }

    if (status & I2C_IC_INTR_STAT_STOP_DET)
    {
        status = i2c->I2C_CLR_STOP_DET;
    }

    if (status & I2C_IC_INTR_STAT_GEN_CALL)
    {
        status = i2c->I2C_CLR_GEN_CALL;
    }

    if (status & I2C_IC_INTR_STAT_TX_OVER)
    {
        transfer->tx_over ++;
        status = i2c->I2C_CLR_TX_OVER;
    }

    if (status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
    {
        transfer->rx_over ++;
        status = i2c->I2C_CLR_RX_OVER;
        status = i2c->I2C_CLR_RX_UNDER;
    }
    if (status & I2C_IC_INTR_STAT_TX_ABRT)
    {
        status = i2c->I2C_TX_ABRT_SOURCE;
        if(status & I2C_SLV_ABRT_LOST_BUS)
        {
            ercd = I2C_ERR_LOST_BUS;
        }

        status = i2c->I2C_CLR_TX_ABRT;
    }

    return ercd;
}

/**
 * @brief    set i2c target address for slave device in master mode
 * @param    i2c       : Pointer to i2c resources structure
 * @param    address   : i2c 7-bit or 10-bit slave address
 * @param    addr_mode : Addressing mode (10Bit/7Bit)
 * @param    cur_state : Current transfer state (Master Tx/ Master Rx)
 * @retval   none
 */
void i2c_set_target_addr(I2C_Type *i2c, const uint32_t address, const i2c_address_mode_t addr_mode, const I2C_TRANSFER_STATE cur_state)
{
    int32_t ic_tar_reg = i2c->I2C_TAR;

    i2c_disable(i2c);

    /* Assign slave address */
    ic_tar_reg = (I2C_IC_TAR_10BIT_ADDR_MASK & address);

    if(addr_mode == I2C_10BIT_ADDRESS)
    {
        /* Configuring master to 10 Bit addressing mode*/
        ic_tar_reg |= I2C_MASTER_10BIT_ADDR_MODE ;

        if(cur_state == I2C_TRANSFER_MST_RX)
        {
            /* When I2C master is in 10 bit Receive mode, the Restart condition must be enabled */
            if(!(i2c_master_check_restart_cond(i2c)))
            {
                i2c_master_enable_restart_cond(i2c);
            }
        }
    }
    else
    {
        /* Configuring master to 7 Bit addressing mode*/
        ic_tar_reg &= (~I2C_MASTER_10BIT_ADDR_MODE);
    }
    /* update the 10bit(0-9) of the ic_tar target register as per our address. */
    i2c->I2C_TAR = ic_tar_reg;
    i2c_enable(i2c);
}

/**
 * @brief   Setup i2c master clock configuration
 * @note    none
 * @param   i2c          : Pointer to i2c register map
 * @param   clk_khz      : Clock
 * @param   speed_mode   : Speed
 *          ARM_I2C_BUS_SPEED_STANDARD /
 *          I2C_IC_CON_SPEED_FAST /
 *          ARM_I2C_BUS_SPEED_FAST_PLUS
 * @retval  none
 */
void i2c_master_set_clock(I2C_Type *i2c, const uint32_t clk_khz, uint8_t speed_mode)
{
    /* Clock setting */
    i2c_set_spike_len(i2c, clk_khz);

    /* set high count and low count for bus speed modes */
    i2c_set_scl_cnt(i2c, clk_khz, speed_mode);

    /* Master code settings */
    /* only in High speed master mode */
    i2c->ic_hs_maddr = 0;
}

/**
 * @brief   initialize i2c master
 * @note    none
 * @param   i2c          : Pointer to i2c register map
 * @param   tar_addr     : target address
 * @retval  none
 */
void i2c_master_init(I2C_Type *i2c,  const uint32_t tar_addr)
{
    uint32_t ic_con_reg_value = 0;

    i2c_disable(i2c);

    /* disable all i2c interrupt */
    i2c->I2C_INTR_MASK = I2C_IC_INT_DISABLE_ALL;

    /* Set to 7bit addressing and update target address */
    i2c->I2C_TAR = (tar_addr & I2C_IC_TAR_10BIT_ADDR_MASK) | I2C_IC_TAR_SPECIAL | I2C_IC_TAR_GC_OR_START;

    /* master mode, restart enabled */
    ic_con_reg_value = I2C_IC_CON_ENABLE_MASTER_MODE | I2C_IC_CON_MASTER_RESTART_EN;

    /* Set final IC_CON value */
    i2c->I2C_CON = ic_con_reg_value;

    i2c_enable(i2c);
}

/**
 * @brief   initialize i2c slave
 * @note    none
 * @param   i2c          : Pointer to i2c register map
 * @param   slave_addr   : i2c slave address
 * param    addr_mode    : Addressing mode (10Bit/7Bit)
 * @retval  none
 */
void i2c_slave_init(I2C_Type *i2c, uint32_t slave_addr, i2c_address_mode_t addr_mode)
{
    int32_t ic_con_reg = 0;

    /* Disable i2c */
    i2c_disable(i2c);

    /* Disable All i2c interrupt */
    i2c->I2C_INTR_MASK = I2C_IC_INT_DISABLE_ALL;

    /* Set slave address as a slave */
    i2c->I2C_SAR = slave_addr & I2C_IC_SAR_10BIT_ADDR_MASK;

    if(addr_mode == I2C_10BIT_ADDRESS)
    {
        /* Configuring slave with 10 Bit addressing */
        ic_con_reg = I2C_IC_CON_ENA_SLAVE_MODE | I2C_IC_CON_MASTER_RESTART_EN | I2C_SLAVE_10BIT_ADDR_MODE ;
    }
    else
    {
        /* Configuring slave with 7 Bit addressing */
        ic_con_reg = I2C_IC_CON_ENA_SLAVE_MODE | I2C_IC_CON_MASTER_RESTART_EN;
    }

    /* Stores to control register */
    i2c->I2C_CON = ic_con_reg;

    /* Enable i2c */
    i2c_enable(i2c);
}

/**
 * @brief    i2c master transmit data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_master_tx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer)
{

    uint32_t i2c_int_status; /* i2c interrupt status */
    uint16_t last_cond = 0;
    uint16_t xmit_data = 0;
    uint8_t  xmit_end  = 0;

    if (transfer->pending)
    {
        /* transfer is pending set last command to Restart */
        last_cond = I2C_IC_DATA_CMD_RESTART;
    }
    else
    {
        /* transfer is completed (not pending) set last command to Stop. */
        last_cond = I2C_IC_DATA_CMD_STOP;
    }

    i2c_int_status = (i2c->I2C_INTR_STAT);

    if (transfer->tx_buf)
    {
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_EMPTY)
        {
            xmit_end = 0;

            while (i2c_tx_ready(i2c))
            {
                xmit_data = (uint16_t)( transfer->tx_buf[transfer->tx_curr_cnt] ) | I2C_IC_DATA_CMD_WRITE_REQ;

                /* send last byte separately with Restart or Stop condition
                 * remaining byte send as a last condition none.
                 */
                if (transfer->tx_curr_cnt == (transfer->tx_total_num-1))
                {
                    xmit_end = 1;
                    xmit_data |= last_cond;
                }
                else
                {
                    xmit_data |= I2C_IC_DATA_CMD_NONE;
                }

                transfer->tx_curr_cnt++;
                transfer->curr_cnt = transfer->tx_curr_cnt;

                /* Updating transmitting data to FIFO */
                i2c->I2C_DATA_CMD = xmit_data;

                if (xmit_end)
                {
                    /* transmitted all the bytes, Mask the TX_EMPTY interrupt */
                    i2c_mask_interrupt(i2c, I2C_IC_INTR_STAT_TX_EMPTY);
                    break;
                }
            }
        }
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_OVER)
        {
            transfer->tx_over++;
        }

        if (i2c_int_status & I2C_IC_INTR_STAT_TX_ABRT)
        {
            transfer->err_state = i2c_master_check_error(i2c, transfer);

            if( transfer->err_state == I2C_ERR_LOST_BUS)
            {
                /* mark event as master lost arbitration. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
                transfer->status |= I2C_TRANSFER_STATUS_ARBITRATION_LOST;
            }
            if( transfer->err_state == I2C_ERR_ADDR_NOACK)
            {
                 /* mark event as slave not acknowledge 7bit/10bit addr. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
                transfer->status |= I2C_TRANSFER_STATUS_ADDRESS_NACK;
            }
            if( transfer->err_state == I2C_ERR_DATA_NOACK)
            {
                /* mark event as slave not acknowledge for the data. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }
            if( transfer->err_state == I2C_ERR_UNDEF)
            {
                transfer->status |= I2C_TRANSFER_STATUS_BUS_ERROR;
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }

            if (transfer->err_state != I2C_ERR_NONE)
            {
                transfer->curr_stat = I2C_TRANSFER_NONE;

                /* clear busy status bit. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }
        }

        if(i2c_int_status & I2C_IC_INTR_STAT_STOP_DET)
        {
            transfer->curr_stat = I2C_TRANSFER_NONE;
            /* mark event as master receive complete successfully. */
            transfer->status |= I2C_TRANSFER_STATUS_DONE;

            /* transmitted all the bytes, disable the transmit interrupt */
            i2c_master_disable_tx_interrupt(i2c);
        }
    }
    else
    {
        i2c_master_disable_tx_interrupt(i2c);
        transfer->curr_stat = I2C_TRANSFER_NONE;

        transfer->status |= I2C_TRANSFER_STATUS_DONE;
        transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
    }
    /* Clear Interrupt */
    i2c_int_status = i2c->I2C_CLR_INTR;
}

/**
 * @brief    i2c master receive data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_master_rx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer)
{
    uint32_t i2c_int_status;
    uint16_t last_cond = 0;
    uint16_t xmit_data = 0;

    if (transfer->pending)
    {
        /* transfer is pending set last command to Restart */
        last_cond = I2C_IC_DATA_CMD_RESTART;
    }
    else
    {
        /* transfer is completed (not pending) set last command to Stop. */
        last_cond = I2C_IC_DATA_CMD_STOP;
    }

    i2c_int_status = (i2c->I2C_INTR_STAT);

    if (transfer->rx_buf)
    {
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_EMPTY)
        {
            while (i2c_tx_ready(i2c))
            {
                /* completed sending all the read commands? */
                if (transfer->rx_curr_tx_index >= transfer->rx_total_num)
                {
                    /* added all the read commands to FIFO.
                     * now we have to read from i2c so disable TX interrupt. */
                    i2c_mask_interrupt(i2c, I2C_IC_INTR_STAT_TX_EMPTY);
                    break;
                }
                xmit_data = I2C_IC_DATA_CMD_READ_REQ;

                /* send last byte separately with Restart or Stop condition
                 * remaining byte send as a last condition none.
                 */
                if (transfer->rx_curr_tx_index == (transfer->rx_total_num - 1) )
                {
                    /* last byte send with Restart/ Stop condition. */
                    xmit_data |= last_cond;
                }
                else
                {
                    xmit_data |= I2C_IC_DATA_CMD_NONE;
                }
                transfer->rx_curr_tx_index++;

                /* Updating transmitting data to FIFO */
                i2c->I2C_DATA_CMD = xmit_data;
            }
        }

        /* transmitted all the read condition, waiting for i2c to receive data from slave.
         * IC_INTR_STAT_RX_FULL set when i2c receives reaches or goes above RX_TL threshold (0 in our case) */
        if (i2c_int_status & I2C_IC_INTR_STAT_RX_FULL)
        {
            while (i2c_rx_ready(i2c))
            {
                /* rx ready, data is available into data buffer read it. */
                transfer->rx_buf[transfer->rx_curr_cnt] = i2c_read_data_from_buffer(i2c);

                transfer->rx_curr_cnt++;
                transfer->curr_cnt = transfer->rx_curr_cnt;

                /* received all the bytes */
                if (transfer->rx_curr_cnt >= transfer->rx_total_num)
                {
                    /* received all the bytes disable the RX interrupt and update callback event. */
                    i2c_mask_interrupt(i2c, I2C_IC_INTR_STAT_RX_FULL);
                    break;
                }
            }
        }

        if (i2c_int_status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
        {
            transfer->rx_over++;
        }

        /* we got Transfer abort check the error condition */
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_ABRT)
        {

            transfer->err_state = i2c_master_check_error(i2c, transfer);

            if( transfer->err_state == I2C_ERR_LOST_BUS)
            {
                /* mark event as master lost arbitration. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
                transfer->status |= I2C_TRANSFER_STATUS_ARBITRATION_LOST;
            }
            if( transfer->err_state == I2C_ERR_ADDR_NOACK)
            {
                 /* mark event as slave not acknowledge 7bit/10bit addr. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
                transfer->status |= I2C_TRANSFER_STATUS_ADDRESS_NACK;

            }
            if( transfer->err_state == I2C_ERR_DATA_NOACK)
            {
                /* mark event as slave not acknowledge for the data. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }
            if( transfer->err_state == I2C_ERR_UNDEF)
            {
                transfer->status |= I2C_TRANSFER_STATUS_BUS_ERROR;
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }

            if(transfer->err_state == I2C_ERR_10B_RD_NORSTRT)
            {
                /* This condition will occur only when master is performing read operation
                 * in 10 bit addressing mode but restart condition is disabled */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }

            if (transfer->err_state != I2C_ERR_NONE)
            {
                i2c_master_disable_rx_interrupt(i2c);
                transfer->curr_stat = I2C_TRANSFER_NONE;

                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }
            /* Clear and Mask the TX_ABRT Interrupt */
            i2c->I2C_CLR_TX_ABRT;
            i2c_mask_interrupt(i2c, I2C_IC_INTR_STAT_TX_ABRT);
        }

        if(i2c_int_status & I2C_IC_INTR_STAT_STOP_DET)
        {
            if(transfer->rx_curr_cnt < transfer->rx_total_num)
            {
                /* Checks if there are pending data present in Rx FIFO that are expected */
                if(i2c->I2C_RXFLR >= (transfer->rx_total_num - (transfer->rx_curr_cnt+1U)))
                {
                    while(transfer->rx_total_num - transfer->rx_curr_cnt)
                    {
                        /* rx ready, data is available into data buffer read it. */
                        transfer->rx_buf[transfer->rx_curr_cnt] = i2c_read_data_from_buffer(i2c);

                        transfer->rx_curr_cnt++;
                        transfer->curr_cnt = transfer->rx_curr_cnt;
                    }
                }
            }

            /* Checks if expected nuber of bytes received*/
            if(transfer->rx_curr_cnt >= transfer->rx_total_num)
            {
                transfer->curr_stat = I2C_TRANSFER_NONE;
                /* mark event as master receive complete successfully. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
            }
            else
            {
                transfer->curr_stat = I2C_TRANSFER_NONE;
                /* mark event as master receive incomplete. */
                transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
            }
            /* Stop bit detected, disable the Receive interrupt */
            i2c_master_disable_rx_interrupt(i2c);
        }
    }
    else
    {
        i2c_master_disable_rx_interrupt(i2c);
        transfer->curr_stat = I2C_TRANSFER_NONE;
        /* clear busy status bit. */
        transfer->status |= I2C_TRANSFER_STATUS_DONE;
        transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
    }
    /* Clear Interrupt */
    i2c_int_status = i2c->I2C_CLR_INTR;
}

/**
 * @brief    i2c slave transmit data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_slave_tx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer)
{
    uint32_t i2c_int_status;
    uint16_t xmit_data = 0;
    uint8_t xmit_end  = 0;

    i2c_int_status =(i2c->I2C_INTR_STAT);

    /* Slave error state check */
    transfer->err_state = i2c_slave_check_error(i2c, transfer);

    /* Transfer buffer has data to transmit */
    if(transfer->tx_buf)
    {
        /* Slave is Active */
        if(i2c->I2C_STATUS & I2C_IC_STATUS_SLAVE_ACT)
        {
            /* checking FIFO is full ready to transmit data */
            while(i2c_tx_ready(i2c))
            {
                   xmit_data = (uint16_t)( transfer->tx_buf[transfer->tx_curr_cnt] ) | I2C_IC_DATA_CMD_WRITE_REQ;

                    /* remaining byte send as a last condition none.*/
                   if (transfer->tx_curr_cnt >= (transfer->tx_total_num-1))
                   {
                       xmit_end = 1;
                   }

                   transfer->tx_curr_cnt++;
                   transfer->curr_cnt = transfer->tx_curr_cnt;

                   /* Updating transmitting data to FIFO */
                   i2c->I2C_DATA_CMD = xmit_data;

                   if (xmit_end)
                   {
                       /* transmitted all the bytes, disable the transmit interrupt */
                       i2c_slave_disable_tx_interrupt(i2c);
                       transfer->curr_stat = I2C_TRANSFER_NONE;

                       /* mark event as slave transmit complete successfully. */
                       transfer->status |= I2C_TRANSFER_STATUS_DONE;

                       break;
                   }/* (xmit_end) */

            } /* while(i2c_tx_ready(i2c_reg_ptr)) END*/

        }/* (i2c_reg_ptr->ic_status & I2C_IC_STATUS_SLAVE_ACT) END*/

    }/* (i2c_info_ptr->transfer.tx_buf) END*/

    if (i2c_int_status & I2C_IC_INTR_STAT_TX_OVER)
    {
        transfer->tx_over ++;
    }

    /* Slave Transmit Abort/bus error */
    if(transfer->err_state == I2C_ERR_LOST_BUS)
    {
        i2c_slave_disable_tx_interrupt(i2c);
        /* mark event as slave lost bus */
        transfer->status |= I2C_TRANSFER_STATUS_BUS_ERROR;
        transfer->status |= I2C_TRANSFER_STATUS_DONE;
        transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
    }

    /* Clear Interrupt */
    i2c_int_status = i2c->I2C_CLR_INTR;
}

/**
 * @brief    i2c slave receive data using interrupt method
 * @param    i2c      : Pointer to i2c register map
 * @param    transfer : Pointer to i2c_transfer_info_t
 * @retval   callback event
 */
void i2c_slave_rx_isr(I2C_Type *i2c, i2c_transfer_info_t *transfer)
{
    uint32_t i2c_int_status;

    i2c_int_status = (i2c->I2C_INTR_STAT);

    /* Checking for the RX full interrupt */
    if (i2c_int_status & I2C_IC_INTR_STAT_RX_FULL)
    {
        /* Ready to receive data, FIFO has data */
        while (i2c_rx_ready(i2c))
        {
            /* rx ready, data is available into data buffer read it. */
            transfer->rx_buf[transfer->rx_curr_cnt] = i2c_read_data_from_buffer(i2c);

            transfer->rx_curr_cnt++;
            transfer->curr_cnt = transfer->rx_curr_cnt;

            /* received all the bytes? */
            if (transfer->rx_curr_cnt >= transfer->rx_total_num)
            {
                /* received all the bytes disable the RX interrupt and update callback event. */
                i2c_slave_disable_rx_interrupt(i2c);

                transfer->curr_stat = I2C_TRANSFER_NONE;
                /* mark event as Slave Receive complete successfully. */
                transfer->status |= I2C_TRANSFER_STATUS_DONE;
                break;
            }/* received all the bytes */

        }/* while (i2c_rx_ready(i2c_reg_ptr)) END*/

    }/* (i2c_int_status & I2C_IC_INTR_STAT_RX_FULL) END */

    if (i2c_int_status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
    {
        transfer->rx_over++;
    }

    if(i2c_int_status & I2C_IC_INTR_STAT_STOP_DET)
    {
        if(transfer->rx_curr_cnt < transfer->rx_total_num)
        {
            /* Checks if there are pending data present in Rx FIFO that are expected */
            if(i2c->I2C_RXFLR >= (transfer->rx_total_num - (transfer->rx_curr_cnt+1U)))
            {
                while(transfer->rx_total_num - transfer->rx_curr_cnt)
                {
                    /* rx ready, data is available into data buffer read it. */
                    transfer->rx_buf[transfer->rx_curr_cnt] = i2c_read_data_from_buffer(i2c);

                    transfer->rx_curr_cnt++;
                    transfer->curr_cnt = transfer->rx_curr_cnt;
                }
            }
        }
        if (transfer->rx_curr_cnt >= transfer->rx_total_num)
        {
            transfer->curr_stat = I2C_TRANSFER_NONE;
            /* mark event as Slave Receive complete successfully. */
            transfer->status |= I2C_TRANSFER_STATUS_DONE;
        }
        else
        {
            transfer->curr_stat = I2C_TRANSFER_NONE;
            /* mark event as Slave Receive incomplete. */
            transfer->status |= I2C_TRANSFER_STATUS_INCOMPLETE;
        }
        i2c_slave_disable_rx_interrupt(i2c);
    }
    /* Clear Interrupt */
    i2c_int_status = i2c->I2C_CLR_INTR;
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
