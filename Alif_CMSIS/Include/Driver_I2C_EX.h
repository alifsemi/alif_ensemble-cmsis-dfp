/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     Driver_I2C_EX.h
 * @author   Shreehari H K
 * @email    shreehari.hk@alifsemi.com
 * @version  V1.0.0
 * @date     23-Feb-2025
 * @brief    Extension of CMSIS Driver_I2C.h
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef DRIVER_I2C_EX_H_
#define DRIVER_I2C_EX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Driver_I2C.h"

/****** I2C Extended Control Codes *****/

#define ARM_I2C_MODE_WRITE_READ                                                                    \
    (0x05UL)  ///< Set Write-Read Combined mode; arg:(enable | target reg addr size in bytes)
#define ARM_I2C_HS_MASTER_ADDR                                                                     \
    (0x06UL)  ///< Set High speed addr of master; arg:addr (0-7)

/****** I2C Write-Read - Target register address size in bytes *****/
#define ARM_I2C_TAR_REG_ADDR_SIZE_Pos 0x0U
#define ARM_I2C_TAR_REG_ADDR_SIZE_Msk 0x0FU
#define ARM_I2C_TAR_REG_ADDR_SIZE(x)                                                               \
    (((x) & ARM_I2C_TAR_REG_ADDR_SIZE_Msk) >> ARM_I2C_TAR_REG_ADDR_SIZE_Pos)
#define ARM_I2C_WRITE_READ_MODE_Pos 0x7U
#define ARM_I2C_WRITE_READ_MODE_EN  (1U << ARM_I2C_WRITE_READ_MODE_Pos)
#define ARM_I2C_WRITE_READ_MODE_DIS (0U << ARM_I2C_WRITE_READ_MODE_Pos)

/****** I2C Extended Event *****/
#define ARM_I2C_EVENT_GCALL_ERROR         (1UL << 16) ///< Error during General Call xfer; paired with BUS_ERROR
#define ARM_I2C_EVENT_UNEXPECTED_ACK      (1UL << 17) ///< Unexpected ACK (HS-mode master code or Start byte); paired with BUS_ERROR
#define ARM_I2C_EVENT_RESTART_DISABLED    (1UL << 18) ///< Xfer needs RESTART but RESTART_EN=0; paired with BUS_ERROR
#define ARM_I2C_EVENT_MASTER_DISABLED     (1UL << 19) ///< Master op while master mode disabled
#define ARM_I2C_EVENT_RX_IN_TX_MODE       (1UL << 20) ///< Read cmd issued while controller in Tx mode
#define ARM_I2C_EVENT_USER_ABORT          (1UL << 21) ///< Xfer aborted via Control(ABORT_TRANSFER); paired with TRANSFER_DONE | TRANSFER_INCOMPLETE
#define ARM_I2C_EVENT_SDA_STUCK_LOW       (1UL << 22) ///< SDA held low past IC_SDA_STUCK_AT_LOW_TIMEOUT; paired with BUS_ERROR
#define ARM_I2C_EVENT_SCL_STUCK_LOW       (1UL << 23) ///< SCL held low past IC_SCL_STUCK_AT_LOW_TIMEOUT; paired with BUS_ERROR
#define ARM_I2C_EVENT_DEV_ID_NACK         (1UL << 24) ///< No ACK during Device-ID transfer;
#define ARM_I2C_EVENT_DEV_ID_TX_DATA      (1UL << 25) ///< Tx FIFO not empty during Device-ID xfer;
#define ARM_I2C_EVENT_TX_FIFO_FLUSHED     (1UL << 26) ///< Stale Tx FIFO flushed (master switched to fresh read)
#define ARM_I2C_EVENT_UNDEF_TX_ABORT      (1UL << 27) ///< Undefined Tx abort
#define ARM_I2C_EVENT_MASTER_ON_HOLD      (1UL << 28) ///< Master is holding the bus

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_I2C_EX_H_ */
