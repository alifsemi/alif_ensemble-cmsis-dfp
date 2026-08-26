/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Includes */
#include <stddef.h>
#include <stdint.h>

/* system includes */
#include "Driver_I2C_Private.h"

#if I2C_DMA_ENABLE
#include "sys_utils.h"
#include "dma_config.h"
#endif

/* Driver version */
#define ARM_I2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 12)

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {ARM_I2C_API_VERSION, ARM_I2C_DRV_VERSION};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
    1, /* supports 10-bit addressing */
    0  /* reserved */
};

/**
 * @brief   get i2c version
 * @note    none
 * @param   none
 * @retval  driver version
 */
static ARM_DRIVER_VERSION ARM_I2C_GetVersion(void)
{
    return DriverVersion;
}

/**
 * @brief   get i2c capabilites
 * @note    none
 * @param   none
 * @retval  driver capabilites
 */
static ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 * @brief   Set i2c Target address
 * @param   I2C  : Pointer to I2C resources structure
 * @param   addr : Target slave address
 * @retval  none
 */
static void I2C_SetTargetAddress(I2C_RESOURCES *I2C, const uint32_t addr)
{
    /* addr is different from current target address */
    if ((addr & (~ARM_I2C_ADDRESS_10BIT)) != I2C->tar_addr) {
        if (addr & ARM_I2C_ADDRESS_10BIT) {
            I2C->addr_mode = I2C_10BIT_ADDRESS;
        } else {
            I2C->addr_mode = I2C_7BIT_ADDRESS;
        }
        /* set target address */
        i2c_set_target_addr(I2C->regs, addr, I2C->addr_mode, I2C->transfer.curr_stat);

        I2C->tar_addr = (addr & (~ARM_I2C_ADDRESS_10BIT));
    }
}
/**
 * @brief   get i2c bus speed
 * @note    implemented only ARM_I2C_BUS_SPEED_STANDARD
 * @param   I2C  : Pointer to I2C resources structure
 * @param   i2c_bus_speed    : i2c bus speed
 *          ARM_I2C_BUS_SPEED_STANDARD /
 *          ARM_I2C_BUS_SPEED_FAST     /
 *          ARM_I2C_BUS_SPEED_FAST_PLUS
 * @retval  none
 */
static int32_t I2C_GetBusSpeed(I2C_RESOURCES *I2C, uint32_t i2c_bus_speed)
{
    int32_t speed;

    switch (i2c_bus_speed) {
    case ARM_I2C_BUS_SPEED_STANDARD:
        /* Standard Speed (100kHz) */
        speed           = I2C_IC_CON_SPEED_STANDARD;
        I2C->speed_mode = I2C_SPEED_STANDARD;
        break;

    case ARM_I2C_BUS_SPEED_FAST:
        /* Fast Speed (400kHz) */
        speed           = I2C_IC_CON_SPEED_FAST;
        I2C->speed_mode = I2C_SPEED_FAST;
        break;

    case ARM_I2C_BUS_SPEED_FAST_PLUS:
        /* Fast+ Speed (1MHz) */
        speed           = I2C_IC_CON_SPEED_FAST;
        I2C->speed_mode = I2C_SPEED_FASTPLUS;
        break;

    case ARM_I2C_BUS_SPEED_HIGH:
        /* High Speed (3.4MHz) */
        speed = I2C_IC_CON_SPEED_HIGH;
        I2C->speed_mode = I2C_SPEED_HIGH;
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return speed;
}

#if I2C_DMA_ENABLE

/**
 * @brief   Initialises I2C DMA
 * @param   dma_periph : Pointer to DMA resources
 * @retval  execution_status
 */
__STATIC_INLINE int32_t I2C_DMA_Initialize(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Initializes DMA interface */
    status                  = dma_drv->Initialize();
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
 * @brief   PowerControls I2C DMA
 * @param   state      : Power state
 * @param   dma_periph : Pointer to DMA resources
 * @retval  execution_status
 */
__STATIC_INLINE int32_t I2C_DMA_PowerControl(ARM_POWER_STATE        state,
                                             DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Controls power of DMA interface */
    status                  = dma_drv->PowerControl(state);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
 * @brief   Allocate a channel for I2C
 * @param   dma_periph : Pointer to DMA resources
 * @retval  execution_status
 */
__STATIC_INLINE int32_t I2C_DMA_Allocate(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Allocate handle for peripheral */
    status                  = dma_drv->Allocate(&dma_periph->dma_handle);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    /* Enable the channel in the Event Router */
    evtrtr_enable_dma_channel(dma_periph->evtrtr_cfg.instance,
                              dma_periph->evtrtr_cfg.channel,
                              dma_periph->evtrtr_cfg.group,
                              DMA_ACK_COMPLETION_PERIPHERAL);

    evtrtr_enable_dma_handshake(dma_periph->evtrtr_cfg.instance,
                                dma_periph->evtrtr_cfg.channel,
                                dma_periph->evtrtr_cfg.group);

    return ARM_DRIVER_OK;
}

/**
 * @brief   De-allocate a channel for I2C
 * @param   dma_periph : Pointer to DMA resources
 * @retval  execution_status
 */
__STATIC_INLINE int32_t I2C_DMA_DeAllocate(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* De-Allocate handle  */
    status                  = dma_drv->DeAllocate(&dma_periph->dma_handle);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    /* Disable the channel in the Event Router */
    evtrtr_disable_dma_channel(dma_periph->evtrtr_cfg.instance, dma_periph->evtrtr_cfg.channel);

    evtrtr_disable_dma_handshake(dma_periph->evtrtr_cfg.instance,
                                 dma_periph->evtrtr_cfg.channel,
                                 dma_periph->evtrtr_cfg.group);

    return ARM_DRIVER_OK;
}

/**
 * @brief   Start I2C DMA transfer
 * @param   dma_periph : Pointer to DMA resources
 * @param   dma_params : Pointer to DMA parameters
 * @retval  execution_status
 */
__STATIC_INLINE int32_t I2C_DMA_Start(DMA_PERIPHERAL_CONFIG *dma_periph, ARM_DMA_PARAMS *dma_params)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Start transfer */
    status                  = dma_drv->Start(&dma_periph->dma_handle, dma_params);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
 * @brief   Stop I2C DMA transfer
 * @param   dma_periph : Pointer to DMA resources
 * @retval  execution_status
 */
__STATIC_INLINE int32_t I2C_DMA_Stop(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Stop transfer */
    status                  = dma_drv->Stop(&dma_periph->dma_handle);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/* 16-bit DATA_CMD words used as fixed sources for Master-RX command issuance */
static const uint16_t I2C_DMA_READ_REQ_WORD      = I2C_IC_DATA_CMD_READ_REQ;
static const uint16_t I2C_DMA_READ_REQ_STOP_WORD = I2C_IC_DATA_CMD_READ_REQ | I2C_IC_DATA_CMD_STOP;

/**
 * @brief   User-provided microcode for a DMA channel
 * @param   dma_periph : Pointer to DMA resources
 * @param   dma_mcode  : Global address of the microcode buffer
 * @retval  execution_status
 */
__STATIC_INLINE int32_t I2C_DMA_Usermcode(DMA_PERIPHERAL_CONFIG *dma_periph, uint32_t dma_mcode)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    status = dma_drv->Control(&dma_periph->dma_handle, ARM_DMA_USER_PROVIDED_MCODE, dma_mcode);
    if (status) {
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}

/**
 * @brief   Build microcode that streams 16-bit DATA_CMD words from the scratch
 *          buffer to I2Cx->I2C_DATA_CMD. Used for Master TX, Slave TX, and the
 *          W/R prefix bytes of Master RX combined transfers.
 * @param   I2C        : Pointer to I2C resources
 * @param   dma_periph : TX-channel DMA peripheral config
 * @param   chunk_cnt  : Number of 16-bit scratch entries to push (1..DMA_MAX_LP_CNT)
 * @retval  true on success
 */
static bool I2C_DMA_GenOpcode_TxShadow(I2C_RESOURCES *I2C, DMA_PERIPHERAL_CONFIG *dma_periph,
                                       uint32_t chunk_cnt)
{
    dma_opcode_buf op_buf;
    dma_ccr_t      ccr;
    dma_loop_t     lp_args;
    uint16_t       lp_start;
    uint8_t        periph_num = dma_periph->dma_periph_req;
    uint8_t        dma_handle = (uint8_t) dma_periph->dma_handle;

    if ((chunk_cnt == 0) || (chunk_cnt > DMA_MAX_LP_CNT)) {
        return false;
    }

    op_buf.buf      = I2C->dma_mcode;
    op_buf.buf_size = I2C_DMA_MCODE_SIZE;
    op_buf.off      = 0;

    ccr.value                  = 0;
    ccr.value_b.src_inc        = DMA_BURST_INCREMENTING;
    ccr.value_b.src_burst_size = BS_BYTE_2;
    ccr.value_b.src_cache_ctrl = DMA_SRC_CACHE_CTRL;
    ccr.value_b.dst_inc        = DMA_BURST_FIXED;
    ccr.value_b.dst_burst_size = BS_BYTE_2;

    if (!dma_construct_move(ccr.value, DMA_REG_CCR, &op_buf)) {
        return false;
    }
    if (!dma_construct_move(LocalToGlobal(I2C->dma_tx_scratch), DMA_REG_SAR, &op_buf)) {
        return false;
    }
    if (!dma_construct_move(LocalToGlobal(i2c_get_data_addr(I2C->regs)), DMA_REG_DAR, &op_buf)) {
        return false;
    }

    if (!dma_construct_loop(DMA_LC_0, (uint8_t) chunk_cnt, &op_buf)) {
        return false;
    }
    lp_start = (uint16_t) op_buf.off;

    if (!dma_construct_flushperiph(periph_num, &op_buf)) {
        return false;
    }
    if (!dma_construct_wfp(DMA_XFER_SINGLE, periph_num, &op_buf)) {
        return false;
    }
    if (!dma_construct_load(DMA_XFER_FORCE, &op_buf)) {
        return false;
    }
    if (!dma_construct_storeperiph(DMA_XFER_SINGLE, periph_num, &op_buf)) {
        return false;
    }

    if ((op_buf.off - lp_start) > DMA_MAX_BACKWARD_JUMP) {
        return false;
    }
    lp_args.jump      = (uint8_t) (op_buf.off - lp_start);
    lp_args.lc        = DMA_LC_0;
    lp_args.nf        = 1;
    lp_args.xfer_type = DMA_XFER_FORCE;
    if (!dma_construct_loopend(&lp_args, &op_buf)) {
        return false;
    }

    if (!dma_construct_wmb(&op_buf)) {
        return false;
    }
    if (!dma_construct_send_event(dma_handle, &op_buf)) {
        return false;
    }
    if (!dma_construct_end(&op_buf)) {
        return false;
    }

    RTSS_CleanDCache_by_Addr(op_buf.buf, op_buf.buf_size);
    return true;
}

/**
 * @brief   Build microcode that issues READ_REQ commands followed by an
 *          optional READ_REQ|STOP terminator. Used for Master RX (data phase).
 *          The optional W/R prefix (from scratch buffer) is emitted first when
 *          prefix_cnt > 0.
 * @param   I2C        : Pointer to I2C resources
 * @param   dma_periph : TX-channel DMA peripheral config
 * @param   prefix_cnt : Number of W/R prefix bytes from dma_tx_scratch (0 if none)
 * @param   rx_total   : Number of READ_REQ words to push (1..DMA_MAX_LP_CNT, total reads)
 * @param   emit_stop  : True to OR STOP into the last READ_REQ word
 * @retval  true on success
 */
static bool I2C_DMA_GenOpcode_MasterRxCmd(I2C_RESOURCES *I2C, DMA_PERIPHERAL_CONFIG *dma_periph,
                                          uint32_t prefix_cnt, uint32_t rx_total, bool emit_stop)
{
    dma_opcode_buf op_buf;
    dma_ccr_t      ccr;
    dma_loop_t     lp_args;
    uint16_t       lp_start;
    uint8_t        periph_num = dma_periph->dma_periph_req;
    uint8_t        dma_handle = (uint8_t) dma_periph->dma_handle;

    if ((rx_total == 0) || (rx_total > DMA_MAX_LP_CNT) || (prefix_cnt > DMA_MAX_LP_CNT)) {
        return false;
    }

    op_buf.buf      = I2C->dma_mcode;
    op_buf.buf_size = I2C_DMA_MCODE_SIZE;
    op_buf.off      = 0;

    if (!dma_construct_move(LocalToGlobal(i2c_get_data_addr(I2C->regs)), DMA_REG_DAR, &op_buf)) {
        return false;
    }

    /* W/R prefix phase: source = scratch (incrementing) */
    if (prefix_cnt > 0) {
        ccr.value                  = 0;
        ccr.value_b.src_inc        = DMA_BURST_INCREMENTING;
        ccr.value_b.src_burst_size = BS_BYTE_2;
        ccr.value_b.src_cache_ctrl = DMA_SRC_CACHE_CTRL;
        ccr.value_b.dst_inc        = DMA_BURST_FIXED;
        ccr.value_b.dst_burst_size = BS_BYTE_2;
        if (!dma_construct_move(ccr.value, DMA_REG_CCR, &op_buf)) {
            return false;
        }
        if (!dma_construct_move(LocalToGlobal(I2C->dma_tx_scratch), DMA_REG_SAR, &op_buf)) {
            return false;
        }

        if (!dma_construct_loop(DMA_LC_0, (uint8_t) prefix_cnt, &op_buf)) {
            return false;
        }
        lp_start = (uint16_t) op_buf.off;
        if (!dma_construct_flushperiph(periph_num, &op_buf)) {
            return false;
        }
        if (!dma_construct_wfp(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return false;
        }
        if (!dma_construct_load(DMA_XFER_FORCE, &op_buf)) {
            return false;
        }
        if (!dma_construct_storeperiph(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return false;
        }
        if ((op_buf.off - lp_start) > DMA_MAX_BACKWARD_JUMP) {
            return false;
        }
        lp_args.jump      = (uint8_t) (op_buf.off - lp_start);
        lp_args.lc        = DMA_LC_0;
        lp_args.nf        = 1;
        lp_args.xfer_type = DMA_XFER_FORCE;
        if (!dma_construct_loopend(&lp_args, &op_buf)) {
            return false;
        }
    }

    /* READ_REQ phase: source = fixed (READ_REQ_WORD), N-1 iterations */
    ccr.value                  = 0;
    ccr.value_b.src_inc        = DMA_BURST_FIXED;
    ccr.value_b.src_burst_size = BS_BYTE_2;
    ccr.value_b.src_cache_ctrl = DMA_SRC_CACHE_CTRL;
    ccr.value_b.dst_inc        = DMA_BURST_FIXED;
    ccr.value_b.dst_burst_size = BS_BYTE_2;
    if (!dma_construct_move(ccr.value, DMA_REG_CCR, &op_buf)) {
        return false;
    }
    if (!dma_construct_move(LocalToGlobal(&I2C_DMA_READ_REQ_WORD), DMA_REG_SAR, &op_buf)) {
        return false;
    }

    if (rx_total > 1) {
        if (!dma_construct_loop(DMA_LC_0, (uint8_t) (rx_total - 1), &op_buf)) {
            return false;
        }
        lp_start = (uint16_t) op_buf.off;
        if (!dma_construct_flushperiph(periph_num, &op_buf)) {
            return false;
        }
        if (!dma_construct_wfp(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return false;
        }
        if (!dma_construct_load(DMA_XFER_FORCE, &op_buf)) {
            return false;
        }
        if (!dma_construct_storeperiph(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return false;
        }
        if ((op_buf.off - lp_start) > DMA_MAX_BACKWARD_JUMP) {
            return false;
        }
        lp_args.jump      = (uint8_t) (op_buf.off - lp_start);
        lp_args.lc        = DMA_LC_0;
        lp_args.nf        = 1;
        lp_args.xfer_type = DMA_XFER_FORCE;
        if (!dma_construct_loopend(&lp_args, &op_buf)) {
            return false;
        }
    }

    /* Final READ_REQ word: optionally with STOP */
    if (emit_stop) {
        if (!dma_construct_move(LocalToGlobal(&I2C_DMA_READ_REQ_STOP_WORD), DMA_REG_SAR, &op_buf)) {
            return false;
        }
    }
    if (!dma_construct_flushperiph(periph_num, &op_buf)) {
        return false;
    }
    if (!dma_construct_wfp(DMA_XFER_SINGLE, periph_num, &op_buf)) {
        return false;
    }
    if (!dma_construct_load(DMA_XFER_FORCE, &op_buf)) {
        return false;
    }
    if (!dma_construct_storeperiph(DMA_XFER_SINGLE, periph_num, &op_buf)) {
        return false;
    }

    if (!dma_construct_wmb(&op_buf)) {
        return false;
    }
    if (!dma_construct_send_event(dma_handle, &op_buf)) {
        return false;
    }
    if (!dma_construct_end(&op_buf)) {
        return false;
    }

    RTSS_CleanDCache_by_Addr(op_buf.buf, op_buf.buf_size);
    return true;
}

/**
 * @brief   Pack a TX chunk (Master TX / Slave TX) from the user buffer into
 *          the 16-bit scratch, applying WRITE_REQ flags and an optional STOP
 *          on the final entry of the final chunk.
 * @param   I2C       : Pointer to I2C resources
 * @param   add_stop  : True if STOP is permitted on the terminating byte
 *                      (i.e. master mode and !xfer_pending; never for slave)
 * @retval  Number of scratch entries written (== chunk_count, 0 on error)
 */
static uint32_t i2c_dma_tx_load_chunk(I2C_RESOURCES *I2C, bool add_stop)
{
    uint32_t remaining = I2C->dma_xfer_remaining;
    uint32_t chunk;
    uint32_t i;
    uint32_t chunk_limit;

    if ((remaining == 0) || (I2C->dma_xfer_src == NULL) || (I2C->dma_tx_scratch == NULL)) {
        return 0;
    }

    chunk_limit = (I2C->dma_tx_scratch_sz > DMA_MAX_LP_CNT)
                      ? DMA_MAX_LP_CNT
                      : I2C->dma_tx_scratch_sz;
    chunk       = (remaining > chunk_limit) ? chunk_limit : remaining;

    for (i = 0; i < chunk; i++) {
        I2C->dma_tx_scratch[i] =
            (uint16_t) I2C->dma_xfer_src[i] | I2C_IC_DATA_CMD_WRITE_REQ;
    }

    /* STOP applies only to the final byte of the final chunk for a master
     * transfer that is not pending. Slave transfers must never set STOP.
     */
    if (add_stop && (chunk == remaining) && (!I2C->dma_xfer_pending)) {
        I2C->dma_tx_scratch[chunk - 1] |= I2C_IC_DATA_CMD_STOP;
    }

    I2C->dma_xfer_src        += chunk;
    I2C->dma_xfer_remaining  -= chunk;

    RTSS_CleanDCache_by_Addr(I2C->dma_tx_scratch,
        (int32_t) (I2C->dma_tx_scratch_sz * sizeof(uint16_t)));
    return chunk;
}

/**
 * @brief   Submit the next TX chunk: pack scratch, build microcode,
 *          and start the TX DMA channel. Used by both the initial transfer
 *          setup and the in-callback re-arm path for Master TX / Slave TX.
 * @param   I2C      : Pointer to I2C resources
 * @param   add_stop : True for master TX (STOP allowed on final byte),
 *                     false for slave TX (slave never emits STOP).
 * @retval  ARM_DRIVER_OK on success, error code otherwise.
 */
static int32_t i2c_dma_tx_submit_chunk(I2C_RESOURCES *I2C, bool add_stop)
{
    ARM_DMA_PARAMS p;
    uint32_t       chunk;

    chunk = i2c_dma_tx_load_chunk(I2C, add_stop);
    if (chunk == 0U) {
        return ARM_DRIVER_ERROR;
    }

    if (!I2C_DMA_GenOpcode_TxShadow(I2C, &I2C->dma_cfg->dma_tx, chunk)) {
        return ARM_DRIVER_ERROR;
    }
    if (I2C_DMA_Usermcode(&I2C->dma_cfg->dma_tx, LocalToGlobal(I2C->dma_mcode))
        != ARM_DRIVER_OK) {
        return ARM_DRIVER_ERROR;
    }

    p.peri_reqno   = (int8_t) I2C->dma_cfg->dma_tx.dma_periph_req;
    p.dir          = ARM_DMA_MEM_TO_DEV;
    p.cb_event     = I2C->dma_cb;
    p.src_addr     = (const void *) I2C->dma_tx_scratch;
    p.dst_addr     = i2c_get_data_addr(I2C->regs);
    p.num_bytes    = chunk * sizeof(uint16_t);
    p.irq_priority = I2C->dma_irq_priority;
    p.burst_size   = BS_BYTE_2;
    p.burst_len    = 1U;

    return I2C_DMA_Start(&I2C->dma_cfg->dma_tx, &p);
}

/**
 * @brief   Submit the next Master-RX READ_REQ command chunk on the TX
 *          channel. The RX-side data drain runs once for the full transfer
 *          via the stock DMA path; only the command-issuance microcode
 *          needs re-arming per chunk.
 * @param   I2C        : Pointer to I2C resources
 * @param   prefix_cnt : W/R prefix entries to emit before READ_REQs
 *                       (non-zero only on the first chunk of a W/R combined
 *                       transfer; 0 for plain reads and all re-arm chunks).
 * @retval  ARM_DRIVER_OK on success, error code otherwise.
 */
static int32_t i2c_dma_rx_cmd_submit_chunk(I2C_RESOURCES *I2C, uint32_t prefix_cnt)
{
    ARM_DMA_PARAMS p;
    uint32_t       remaining = I2C->dma_xfer_remaining;
    uint32_t       chunk;
    bool           emit_stop;

    if (remaining == 0U) {
        return ARM_DRIVER_ERROR;
    }

    chunk     = (remaining > DMA_MAX_LP_CNT) ? DMA_MAX_LP_CNT : remaining;
    emit_stop = (chunk == remaining) && (!I2C->dma_xfer_pending);

    if (!I2C_DMA_GenOpcode_MasterRxCmd(I2C, &I2C->dma_cfg->dma_tx, prefix_cnt, chunk,
                                       emit_stop)) {
        return ARM_DRIVER_ERROR;
    }
    if (I2C_DMA_Usermcode(&I2C->dma_cfg->dma_tx, LocalToGlobal(I2C->dma_mcode))
        != ARM_DRIVER_OK) {
        return ARM_DRIVER_ERROR;
    }

    p.peri_reqno   = (int8_t) I2C->dma_cfg->dma_tx.dma_periph_req;
    p.dir          = ARM_DMA_MEM_TO_DEV;
    p.cb_event     = I2C->dma_cb;
    p.src_addr     = (const void *) I2C->dma_tx_scratch;
    p.dst_addr     = i2c_get_data_addr(I2C->regs);
    p.num_bytes    = (prefix_cnt + chunk) * sizeof(uint16_t);
    p.irq_priority = I2C->dma_irq_priority;
    p.burst_size   = BS_BYTE_2;
    p.burst_len    = 1U;

    if (I2C_DMA_Start(&I2C->dma_cfg->dma_tx, &p) != ARM_DRIVER_OK) {
        return ARM_DRIVER_ERROR;
    }

    I2C->dma_xfer_remaining -= chunk;
    return ARM_DRIVER_OK;
}
#endif /* I2C_DMA_ENABLE */

/**
 * @brief   CMSIS-Driver i2c initialize
 * @note    it will use interrupt method for data send and receive.
 * @param   cb_event    : Pointer to \ref ARM_I2C_SignalEvent
 * @param   i2c         : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_OK : successfully initialized
 */
static int32_t ARM_I2C_Initialize(ARM_I2C_SignalEvent_t cb_event, I2C_RESOURCES *I2C)
{
    /* Driver is already initialized */
    if (I2C->state.initialized == 1) {
        return ARM_DRIVER_OK;
    }

    /* default setting */
    I2C->addr_mode              = I2C_7BIT_ADDRESS;
    I2C->tar_addr               = I2C_0_TARADDR;

    I2C->tar_addr              &= I2C_7BIT_ADDR_MASK;
    I2C->slv_addr              &= I2C_7BIT_ADDR_MASK;

    I2C->transfer.abort         = false;

    I2C->transfer.tx_over       = 0U;
    I2C->transfer.rx_over       = 0U;
    I2C->transfer.xfer_pending  = 0U;

    /* set the user callback event. */
    I2C->cb_event               = cb_event;

    /* Get the I2C core clock */
    I2C->clk                    = get_i2c_core_clock();

    /* set the flag as initialized. */
    I2C->state.initialized      = 1;

#if I2C_DMA_ENABLE
    if (I2C->dma_enable) {
        I2C->dma_cfg->dma_rx.dma_handle = -1;
        I2C->dma_cfg->dma_tx.dma_handle = -1;

        /* Initialize DMA for I2C-Tx */
        if (I2C_DMA_Initialize(&I2C->dma_cfg->dma_tx) != ARM_DRIVER_OK) {
            return ARM_DRIVER_ERROR;
        }

        /* Initialize DMA for I2C-Rx */
        if (I2C_DMA_Initialize(&I2C->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
            return ARM_DRIVER_ERROR;
        }
    }
#endif

    return ARM_DRIVER_OK;
}
/**
 * @brief   CMSIS-Driver i2c uninitialize
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_OK : successfully uninitialized
 */
static int32_t ARM_I2C_Uninitialize(I2C_RESOURCES *I2C)
{
    int ret = ARM_DRIVER_OK;

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0) {
        return ARM_DRIVER_OK;
    }

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 1) {
        return ARM_DRIVER_ERROR;
    }

#if I2C_DMA_ENABLE
    if (I2C->dma_enable) {
        I2C->dma_cfg->dma_rx.dma_handle = -1;
        I2C->dma_cfg->dma_tx.dma_handle = -1;
    }
#endif

    /* initialize all variables to 0 */

    /* initialize the tx_buffer */
    I2C->transfer.tx_buf           = NULL;
    I2C->transfer.tx_total_num     = 0U;
    I2C->transfer.tx_curr_cnt      = 0U;
    I2C->transfer.curr_cnt         = 0U;
    I2C->transfer.tx_over          = 0U;

    /* initialize the rx_buffer */
    I2C->transfer.rx_buf           = NULL;
    I2C->transfer.rx_total_num     = 0U;
    I2C->transfer.rx_curr_cnt      = 0U;
    I2C->transfer.rx_curr_tx_index = 0U;
    I2C->transfer.rx_over          = 0U;

    I2C->transfer.xfer_pending     = 0U;
    I2C->transfer.abort            = false;
    I2C->transfer.curr_stat        = I2C_XFER_NONE;

    /* Clear driver status \ref ARM_I2C_STATUS */
    I2C->status.busy               = 0U;
    I2C->status.mode               = 0U;
    I2C->status.direction          = 0U;
    I2C->status.arbitration_lost   = 0U;
    I2C->status.bus_error          = 0U;
    I2C->addr_mode                 = I2C_7BIT_ADDRESS;
    I2C->tar_addr                  = 0U;

    /* Reset the flags. */
    I2C->state.initialized         = 0U;

    return ret;
}

/**
 * @func    : CMSIS Driver I2C Power Control
 * @brief   : Power the driver and enable the NVIC
 * @param   : state : Power state
 * @param   : I2C   : Pointer to i2c resources structure
 * @return  : ARM_DRIVER_OK
 */
static int32_t ARM_I2C_PowerControl(ARM_POWER_STATE state, I2C_RESOURCES *I2C)
{
    switch (state) {
    case ARM_POWER_FULL:
        /* check for Driver initialization */
        if (I2C->state.initialized == 0) {
            return ARM_DRIVER_ERROR;
        }

        /* check for the power is done before initialization or not */
        if (I2C->state.powered == 1) {
            return ARM_DRIVER_OK;
        }

        enable_i2c_clock(I2C->instance);
        /* Disable device before initializing it */
        i2c_disable(I2C->regs);

        /* Disable all interrupts */
        i2c_master_disable_tx_interrupt(I2C->regs);
        i2c_master_disable_rx_interrupt(I2C->regs);
        i2c_slave_disable_tx_interrupt(I2C->regs);
        i2c_slave_disable_rx_interrupt(I2C->regs);

        /* Clear Any Pending Irq */
        NVIC_ClearPendingIRQ(I2C->irq_num);

        /* Set Priority */
        NVIC_SetPriority(I2C->irq_num, I2C->irq_priority);

        /* Enable IRQ */
        NVIC_EnableIRQ(I2C->irq_num);

        i2c_set_tx_threshold(I2C->regs, I2C->tx_fifo_threshold);
        i2c_set_rx_threshold(I2C->regs, I2C->rx_fifo_threshold);
        i2c_set_scl_stuck_timeout(I2C->regs, I2C->scl_stuck_timeout);

#if I2C_DMA_ENABLE
        if (I2C->dma_enable) {
            /* Power On DMA for I2C-Tx */
            if (I2C_DMA_PowerControl(state, &I2C->dma_cfg->dma_tx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }

            /* Power On DMA for I2C-Rx */
            if (I2C_DMA_PowerControl(state, &I2C->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }

            /* Allocate DMA Tx channel */
            if (I2C_DMA_Allocate(&I2C->dma_cfg->dma_tx) == ARM_DRIVER_ERROR) {
                return ARM_DRIVER_ERROR;
            }

            /* Allocate DMA Rx channel */
            if (I2C_DMA_Allocate(&I2C->dma_cfg->dma_rx) == ARM_DRIVER_ERROR) {
                return ARM_DRIVER_ERROR;
            }
        }
#endif
        I2C->state.powered = 1;
        break;
    case ARM_POWER_OFF:
        if (I2C->state.powered == 0) {
            return ARM_DRIVER_OK;
        }
#if I2C_DMA_ENABLE
        if (I2C->dma_enable) {
            i2c_disable_tx_dma(I2C->regs);
            i2c_disable_rx_dma(I2C->regs);

            /* Deallocate DMA Tx channel */
            if (I2C_DMA_DeAllocate(&I2C->dma_cfg->dma_tx) == ARM_DRIVER_ERROR) {
                return ARM_DRIVER_ERROR;
            }

            /* Deallocate DMA Rx channel */
            if (I2C_DMA_DeAllocate(&I2C->dma_cfg->dma_rx) == ARM_DRIVER_ERROR) {
                return ARM_DRIVER_ERROR;
            }

            /* Power Off DMA for I2C-Tx */
            if (I2C_DMA_PowerControl(state, &I2C->dma_cfg->dma_tx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }

            /* Power Off DMA for I2C-Rx */
            if (I2C_DMA_PowerControl(state, &I2C->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }
        }
#endif
        /* Disabling interrupts */
        if (I2C->mode == I2C_MASTER_MODE) {
            i2c_master_disable_tx_interrupt(I2C->regs);
            i2c_master_disable_rx_interrupt(I2C->regs);
        } else {
            i2c_slave_disable_tx_interrupt(I2C->regs);
            i2c_slave_disable_rx_interrupt(I2C->regs);
        }

        /* Disable the IRQ */
        NVIC_DisableIRQ(I2C->irq_num);

        /* Clearing pending */
        NVIC_ClearPendingIRQ(I2C->irq_num);

        /* Disable device before de-initializing it */
        i2c_disable(I2C->regs);
        disable_i2c_clock(I2C->instance);

        I2C->state.powered = 0;
        break;
    case ARM_POWER_LOW:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c master transmit
 *          Start sending data to i2c transmitter.
 * @note    I2C_FLAG_MASTER_SETUP flag should be enabled first /ref ARM_I2C_BUS_SPEED
 * @param   data         : Pointer to buffer with data to send to i2c transmitter
 * @param   num          : Number of data items to send
 * @param   I2C          : Pointer to i2c resources structure
 * @param   xfer_pending : Transfer operation is pending - Stop condition will not be
 *                         generated
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 */
static int32_t ARM_I2C_MasterTransmit(I2C_RESOURCES *I2C, uint32_t addr, const uint8_t *data,
                                      uint32_t num, bool xfer_pending)
{
    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

#if SOC_FEAT_I2C_HAS_RESTART_CAP
    I2C->transfer.xfer_pending = xfer_pending;
#else
    /* Error when RESTART is requested */
    if (xfer_pending) {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
#endif

    /* addr 7bit addr: 0x7F , 10bit addr: 0x3FF */
    if ((data == NULL) || (num == 0U) ||
        ((addr & (~ARM_I2C_ADDRESS_10BIT)) > I2C_10BIT_ADDR_MAX)) {
        /* Invalid parameters */
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (I2C->state.master_setup == 0U) {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy) {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.busy             = 1;
    I2C->status.mode             = I2C_MASTER_MODE;
    I2C->status.direction        = I2C_DIR_TRANSMITTER;
    I2C->status.arbitration_lost = 0;
    I2C->status.bus_error        = 0;

    /* fill the I2C transfer structure as per user detail */
    I2C->transfer.tx_curr_cnt    = 0U;
    I2C->transfer.curr_cnt       = 0U;
    I2C->transfer.tx_over        = 0U;
    I2C->transfer.curr_stat      = I2C_XFER_MST_TX;

    I2C_SetTargetAddress(I2C, addr);

    /* Clear all interrupts */
    i2c_clear_all_interrupt(I2C->regs);

#if I2C_DMA_ENABLE
    if (I2C->dma_enable) {
        /* Clear transfer state */
        I2C->transfer.tx_buf       = NULL;
        I2C->transfer.rx_buf       = NULL;
        I2C->transfer.tx_curr_cnt  = 0U;
        I2C->transfer.rx_curr_cnt  = 0U;
        I2C->transfer.tx_total_num = 0U;
        I2C->transfer.rx_total_num = 0U;
        I2C->transfer.wr_mode      = false;

        /* Pass the user buffer in chunks */
        I2C->dma_xfer_src       = data;
        I2C->dma_xfer_remaining = num;
        I2C->dma_xfer_total     = num;
        I2C->dma_xfer_pending   = xfer_pending;

        if (i2c_dma_tx_submit_chunk(I2C, true) != ARM_DRIVER_OK) {
            I2C->status.busy        = 0U;
            I2C->transfer.curr_stat = I2C_XFER_NONE;
            return ARM_DRIVER_ERROR;
        }

        /* Prepare the I2C controller for DMA transmission */
        i2c_enable_tx_dma(I2C->regs);
        i2c_set_dma_tx_level(I2C->regs, 0U);

        i2c_enable_dma_master_tx(I2C->regs);

    } else
#endif
    {
        /* Update Tx buf pointer, total num bytes and pending
         * fields of I2C transfer structure as per user detail */
        I2C->transfer.tx_buf       = (const uint8_t *) data;
        I2C->transfer.tx_total_num = num;

        /* enable master tx interrupt */
        i2c_master_enable_tx_interrupt(I2C->regs);
    }

    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c master receive
 *          Start receiving data from i2c receiver.
 * @note    none
 * @param   data         : Pointer to buffer for data to receive from i2c receiver
 * @param   addr         : Target slave address
 * @param   num          : Number of data items to receive
 * @param   I2C          : Pointer to i2c resources structure
 * @param   xfer_pending : Transfer operation is pending - Stop condition will not be
 *                         generated
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 */
static int32_t ARM_I2C_MasterReceive(I2C_RESOURCES *I2C, uint32_t addr, uint8_t *data, uint32_t num,
                                     bool xfer_pending)
{
    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

#if SOC_FEAT_I2C_HAS_RESTART_CAP
    I2C->transfer.xfer_pending = xfer_pending;
#else
    /* Error when RESTART is requested */
    if (xfer_pending) {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
#endif

    /* addr 7bit addr: 0x7F , 10bit addr: 0x3FF */
    if ((data == NULL) || (num == 0U) ||
        ((addr & (~ARM_I2C_ADDRESS_10BIT)) > I2C_10BIT_ADDR_MAX)) {
        /* Invalid parameters */
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (I2C->state.master_setup == 0U) {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy) {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.busy               = 1;
    I2C->status.mode               = I2C_MASTER_MODE;
    I2C->status.direction          = I2C_DIR_RECEIVER;
    I2C->status.arbitration_lost   = 0;
    I2C->status.bus_error          = 0;

    /* fill the i2c transfer structure as per user detail */
    I2C->transfer.rx_buf           = (uint8_t *) data;
    I2C->transfer.rx_total_num     = num;
    I2C->transfer.rx_curr_cnt      = 0U;
    I2C->transfer.curr_cnt         = 0U;
    I2C->transfer.rx_curr_tx_index = 0U;
    I2C->transfer.rx_over          = 0U;
    I2C->transfer.curr_stat        = I2C_XFER_MST_RX;

    /* Clear all interrupts */
    i2c_clear_all_interrupt(I2C->regs);

    I2C_SetTargetAddress(I2C, addr);

#if I2C_DMA_ENABLE
    if (I2C->dma_enable) {
        ARM_DMA_PARAMS dma_rx_params;
        uint32_t       prefix_cnt = 0U;

        /* Clear transfer state */
        I2C->transfer.tx_buf       = NULL;
        I2C->transfer.rx_buf       = NULL;
        I2C->transfer.tx_curr_cnt  = 0U;
        I2C->transfer.rx_curr_cnt  = 0U;
        I2C->transfer.tx_total_num = 0U;
        I2C->transfer.rx_total_num = 0U;
        I2C->transfer.wr_mode      = false;

        if (I2C->wr_mode_info & I2C_WRITE_READ_MODE_EN) {
            prefix_cnt = I2C_WRITE_READ_TAR_REG_ADDR_SIZE(I2C->wr_mode_info);
            if (prefix_cnt > I2C->dma_tx_scratch_sz) {
                I2C->status.busy        = 0U;
                I2C->transfer.curr_stat = I2C_XFER_NONE;
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            I2C->transfer.tx_buf       = (uint8_t *) data;
            I2C->transfer.tx_total_num = prefix_cnt;
            I2C->transfer.tx_curr_cnt  = 0U;
            I2C->transfer.wr_mode      = true;

            /* Pack the W/R-prefix bytes into the scratch without STOP
             * (RESTART before the read phase is handled by the I2C HW).
             */
            I2C->dma_xfer_src       = data;
            I2C->dma_xfer_remaining = prefix_cnt;
            I2C->dma_xfer_pending   = true; /* prevent STOP on prefix */
            (void) i2c_dma_tx_load_chunk(I2C, false);
        }

        /* Set up READ_REQ command-issuance. dma_xfer_remaining now
         * tracks how many READ_REQ words are left across chunks; the
         * DMA callback re-arms the TX channel for transfers > DMA_MAX_LP_CNT.
         */
        I2C->dma_xfer_remaining = num;
        I2C->dma_xfer_total     = num;
        I2C->dma_xfer_pending   = xfer_pending;

        /* Build + submit the first TX-channel chunk (optional W/R prefix +
         * up to DMA_MAX_LP_CNT READ_REQs, with STOP only on the very last
         * chunk and only when not pending).
         *
         * Order matters: start the RX channel (DEV->MEM) FIRST so it
         * waits for data, THEN kick off the TX channel via
         * submit_chunk.
         */

        dma_rx_params.peri_reqno   = (int8_t) I2C->dma_cfg->dma_rx.dma_periph_req;
        dma_rx_params.dir          = ARM_DMA_DEV_TO_MEM;
        dma_rx_params.cb_event     = I2C->dma_cb;
        dma_rx_params.src_addr     = i2c_get_data_addr(I2C->regs);
        dma_rx_params.dst_addr     = (void *) data;
        dma_rx_params.num_bytes    = num;
        dma_rx_params.irq_priority = I2C->dma_irq_priority;
        dma_rx_params.burst_size   = BS_BYTE_1;
        dma_rx_params.burst_len    = 1U;

        i2c_enable_rx_dma(I2C->regs);
        i2c_set_dma_rx_level(I2C->regs, 0U);

        if (I2C_DMA_Start(&I2C->dma_cfg->dma_rx, &dma_rx_params) != ARM_DRIVER_OK) {
            i2c_disable_rx_dma(I2C->regs);
            I2C->status.busy        = 0U;
            I2C->transfer.curr_stat = I2C_XFER_NONE;
            return ARM_DRIVER_ERROR;
        }

        /* TX-channel kickoff via submit_chunk (READ_REQ microcode) */
        if (i2c_dma_rx_cmd_submit_chunk(I2C, prefix_cnt) != ARM_DRIVER_OK) {
            (void) I2C_DMA_Stop(&I2C->dma_cfg->dma_rx);
            i2c_disable_rx_dma(I2C->regs);
            I2C->status.busy        = 0U;
            I2C->transfer.curr_stat = I2C_XFER_NONE;
            return ARM_DRIVER_ERROR;
        }

        i2c_enable_tx_dma(I2C->regs);
        i2c_set_dma_tx_level(I2C->regs, 0U);

        i2c_enable_dma_master_rx(I2C->regs);

        return ARM_DRIVER_OK;
    }
#endif
    if (I2C->wr_mode_info & I2C_WRITE_READ_MODE_EN) {
        /* fill the i2c transfer structure required for Write-Read xfer */
        I2C->transfer.tx_buf       = (uint8_t *) data;
        I2C->transfer.tx_total_num = I2C_WRITE_READ_TAR_REG_ADDR_SIZE(I2C->wr_mode_info);
        I2C->transfer.tx_curr_cnt  = 0U;
        I2C->transfer.wr_mode      = true;
    }
    /* enable master rx interrupt */
    i2c_master_enable_rx_interrupt(I2C->regs);
    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c slave transmit
 *          Start sending data to i2c master.
 * @note    master_setup bit should be enabled first /ref ARM_I2C_BUS_SPEED
 * @param   addr : Target slave address
 * @param   data : Pointer to buffer with data to send to i2c master
 * @param   num  : Number of data items to send
 * @param   I2C  : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  transmit count              : For data transmit count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_SlaveTransmit(I2C_RESOURCES *I2C, const uint8_t *data, uint32_t num)
{
    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    if ((data == NULL) || (num == 0U)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Check Slave mode is enabled */
    if (I2C->state.slave_setup == 0U) {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy) {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.busy             = 1;
    I2C->status.mode             = I2C_SLAVE_MODE;
    I2C->status.direction        = I2C_DIR_TRANSMITTER;
    I2C->status.arbitration_lost = 0;
    I2C->status.bus_error        = 0;

    /* fill the i2c transfer structure as per user detail */
    I2C->transfer.tx_curr_cnt    = 0U;
    I2C->transfer.curr_cnt       = 0U;
    I2C->transfer.tx_over        = 0U;
    I2C->transfer.curr_stat      = I2C_XFER_SLV_TX;

    /* Clear stale software-clearable interrupts from a previous transaction
     * WITHOUT touching RD_REQ. CLR_INTR would clear a pending RD_REQ
     * latched when the master started its read phase mid-transaction (e.g.
     * WRITE + RESTART + READ, where the app arms SlaveTransmit after
     * SlaveReceive has completed).
     */
    (void) I2C->regs->I2C_CLR_TX_ABRT;
    (void) I2C->regs->I2C_CLR_STOP_DET;

#if I2C_DMA_ENABLE
    if (I2C->dma_enable) {
        /* Clear transfer state */
        I2C->transfer.tx_buf       = NULL;
        I2C->transfer.rx_buf       = NULL;
        I2C->transfer.tx_curr_cnt  = 0U;
        I2C->transfer.rx_curr_cnt  = 0U;
        I2C->transfer.tx_total_num = 0U;
        I2C->transfer.rx_total_num = 0U;
        I2C->transfer.wr_mode      = false;

        I2C->dma_xfer_src       = data;
        I2C->dma_xfer_remaining = num;
        I2C->dma_xfer_total     = num;

        i2c_set_dma_tx_level(I2C->regs, 0U);

        if (i2c_dma_tx_submit_chunk(I2C, false) != ARM_DRIVER_OK) {
            I2C->status.busy        = 0U;
            I2C->transfer.curr_stat = I2C_XFER_NONE;
            return ARM_DRIVER_ERROR;
        }

        /* Unmask RD_REQ + TX_ABRT + STOP_DET. The slave-TX ISR enables
         * TDMAE on RD_REQ.
         */
        i2c_enable_dma_slave_tx(I2C->regs);
    } else
#endif
    {
        /* Update Tx buf pointer and total num bytes
         * of I2C transfer structure as per user detail */
        I2C->transfer.tx_buf       = (const uint8_t *) data;
        I2C->transfer.tx_total_num = num;

        /* enable slave tx interrupt */
        i2c_slave_enable_tx_interrupt(I2C->regs);
    }

    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c slave receive
 *          Start receiving data from i2c master.
 * @note    none
 * @param   data : Pointer to buffer for data to receive from i2c master
 * @param   num  : Number of data items to receive
 * @param   I2C  : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  received count              : For data receive count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_SlaveReceive(I2C_RESOURCES *I2C, uint8_t *data, uint32_t num)
{
#if I2C_DMA_ENABLE
    ARM_DMA_PARAMS dma_params;
#endif

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    if ((data == NULL) || (num == 0U)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Check Slave mode is enabled */
    if (I2C->state.slave_setup == 0U) {
        /* error master mode is not configured (mode not selected)
         * master_setup  should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    if (I2C->status.busy) {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Update driver status \ref ARM_I2C_STATUS */
    I2C->status.busy          = 1;
    I2C->status.mode          = I2C_SLAVE_MODE;
    I2C->status.direction     = I2C_DIR_RECEIVER;
    I2C->status.bus_error     = 0;

    /* fill the i2c transfer structure as per user detail */
    I2C->transfer.rx_curr_cnt = 0U;
    I2C->transfer.curr_cnt    = 0U;
    I2C->transfer.curr_stat   = I2C_XFER_SLV_RX;
    I2C->transfer.rx_over     = 0U;

    /* Clear all interrupts */
    i2c_clear_all_interrupt(I2C->regs);

#if I2C_DMA_ENABLE
    if (I2C->dma_enable) {
        /* Clear transfer state */
        I2C->transfer.tx_buf       = NULL;
        I2C->transfer.rx_buf       = NULL;
        I2C->transfer.tx_curr_cnt  = 0U;
        I2C->transfer.rx_curr_cnt  = 0U;
        I2C->transfer.tx_total_num = 0U;
        I2C->transfer.rx_total_num = 0U;
        I2C->transfer.wr_mode      = false;

        I2C->transfer.rx_buf       = (uint8_t *) data;
        I2C->transfer.rx_total_num = num;
        I2C->dma_xfer_remaining    = 0U;
        I2C->dma_xfer_total        = num;

        dma_params.peri_reqno   = (int8_t) I2C->dma_cfg->dma_rx.dma_periph_req;
        dma_params.dir          = ARM_DMA_DEV_TO_MEM;
        dma_params.cb_event     = I2C->dma_cb;
        dma_params.src_addr     = i2c_get_data_addr(I2C->regs);
        dma_params.dst_addr     = (void *) data;
        dma_params.num_bytes    = num;
        dma_params.irq_priority = I2C->dma_irq_priority;
        dma_params.burst_size   = BS_BYTE_1;
        dma_params.burst_len    = 1U;

        i2c_enable_rx_dma(I2C->regs);
        i2c_set_dma_rx_level(I2C->regs, 0U);

        if (I2C_DMA_Start(&I2C->dma_cfg->dma_rx, &dma_params) != ARM_DRIVER_OK) {
            /* RDMAE was set above; clear it (no DMA channel was started). */
            i2c_disable_rx_dma(I2C->regs);
            I2C->status.busy        = 0U;
            I2C->transfer.curr_stat = I2C_XFER_NONE;
            return ARM_DRIVER_ERROR;
        }

        i2c_enable_dma_slave_rx(I2C->regs);
    } else
#endif
    {
        /* Update Rx buf pointer and total num bytes
         * of I2C transfer structure as per user detail */
        I2C->transfer.rx_buf       = (uint8_t *) data;
        I2C->transfer.rx_total_num = num;

        /* enable slave rx interrupt */
        i2c_slave_enable_rx_interrupt(I2C->regs);
    }
    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c get transfer data count
 * @note    Returns data count
 * @param   I2C   : Pointer to i2c resources structure
 * @retval  transfer data count
 */
static int32_t ARM_I2C_GetDataCount(const I2C_RESOURCES *I2C)
{
#if I2C_DMA_ENABLE
    if (I2C->dma_enable && I2C->status.busy) {
        uint32_t dma_count = 0;
        ARM_DRIVER_DMA *dma_drv;
        DMA_Handle_Type *handle;

        if (I2C->status.direction == I2C_DIR_TRANSMITTER) {
            dma_drv = I2C->dma_cfg->dma_tx.dma_drv;
            handle  = (DMA_Handle_Type *)&I2C->dma_cfg->dma_tx.dma_handle;
            (void) dma_drv->GetStatus(handle, &dma_count);
            /* TX DMA transfers 16-bit DATA_CMD words; convert to byte count.
             * curr_cnt holds bytes completed in prior chunks; dma_count is
             * the in-flight progress within the current chunk.
             */
            return (int32_t)(I2C->transfer.curr_cnt + dma_count / 2U);
        } else {
            dma_drv = I2C->dma_cfg->dma_rx.dma_drv;
            handle  = (DMA_Handle_Type *)&I2C->dma_cfg->dma_rx.dma_handle;
            (void) dma_drv->GetStatus(handle, &dma_count);
            /* RX channel runs once for the full transfer; dma_count is
             * already the full-transfer byte progress.
             */
            return (int32_t) dma_count;
        }
    }
#endif

    return (int32_t) I2C->transfer.curr_cnt;
}

/**
 * @brief   CMSIS-Driver i2c control
 *          Control i2c Interface.
 * @note    none
 * @param   control : Operation
 * @param   arg     : Argument of operation (optional)
 * @param   I2C     : Pointer to i2c resources structure
 * @retval  common \ref execution_status and driver specific \ref i2c_execution_status
 */
static int32_t ARM_I2C_Control(I2C_RESOURCES *I2C, uint32_t control, uint32_t arg)
{
    int32_t speed;

    /* check i2c driver is initialized or not */
    if (I2C->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* check i2c driver is powered or not */
    if (I2C->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    switch (control) {
    case ARM_I2C_OWN_ADDRESS:

        speed = I2C_GetBusSpeed(I2C, ARM_I2C_BUS_SPEED_STANDARD);

        if (arg & ARM_I2C_ADDRESS_10BIT) {
            if ((arg & (~ARM_I2C_ADDRESS_10BIT)) > I2C_10BIT_ADDR_MAX) {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
            /* Sets 10 bit addr mode */
            I2C->addr_mode = I2C_10BIT_ADDRESS;
        } else {
            /* Reject reserved blocks: 0x00..0x07 and 0x78..0x7F. */
            if (arg < I2C_7BIT_ADDR_MIN || arg > I2C_7BIT_ADDR_MAX) {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
            /* Sets 7 bit addr mode */
            I2C->addr_mode = I2C_7BIT_ADDRESS;
        }

        i2c_slave_init(I2C->regs, arg, I2C->addr_mode);

        /* Sets bus speed*/
        i2c_set_bus_speed(I2C->regs, speed);

        I2C->mode              = I2C_SLAVE_MODE;
        /* setup slave flag */
        I2C->state.slave_setup = 1;

        break;

    case ARM_I2C_BUS_SPEED:

        speed = I2C_GetBusSpeed(I2C, arg);
        /* Unsupported speed */
        if (speed == ARM_DRIVER_ERROR_UNSUPPORTED) {
            return speed;
        }
        /* arg is i2c bus speed */
        i2c_master_init(I2C->regs, I2C->tar_addr);

        /* Sets master clock settings*/
        i2c_master_set_clock(I2C->regs, (I2C->clk / 1000), I2C->speed_mode);

        /* Sets bus speed*/
        i2c_set_bus_speed(I2C->regs, speed);

        I2C->mode               = I2C_MASTER_MODE;
        /* setup master flag */
        I2C->state.master_setup = 1;

        break;

    case ARM_I2C_BUS_CLEAR:

        if (I2C->mode != I2C_MASTER_MODE) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
        /* Enable SDA low stuck recovery */
        I2C->transfer.cmd_bus_clr = true;
        i2c_master_recover_sda(I2C->regs);

        break;

    case ARM_I2C_ABORT_TRANSFER:

        /* I2C protocol: only the master can terminate a bus cycle */
        if (I2C->mode != I2C_MASTER_MODE) {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Mark abort early so DMA callback / ISR won't overwrite curr_cnt */
        I2C->transfer.abort = true;

        if (I2C->status.busy) {
            /* Active master transfer: emit TX_ABRT(USER_ABRT) +
             * STOP_DET; the ISR handles teardown and app callback.
             */
            i2c_master_abort(I2C->regs);
        } else {
            /* No active transfer: local cleanup only. */
            i2c_master_disable_tx_interrupt(I2C->regs);
            i2c_master_disable_rx_interrupt(I2C->regs);

            I2C->transfer.tx_total_num = 0U;
            I2C->transfer.rx_total_num = 0U;
            I2C->transfer.curr_stat    = I2C_XFER_NONE;
            I2C->transfer.abort        = false;
            I2C->status.busy           = 0U;
        }

        break;

    case ARM_I2C_MODE_WRITE_READ:
        /* Write-Read combined mode selection */
        if (arg & ARM_I2C_WRITE_READ_MODE_EN) {
            I2C->wr_mode_info  = I2C_WRITE_READ_MODE_EN;
            I2C->wr_mode_info |= I2C_WRITE_READ_TAR_REG_ADDR_SIZE(ARM_I2C_TAR_REG_ADDR_SIZE(arg));
        } else {
            I2C->wr_mode_info &= ~I2C_WRITE_READ_MODE_EN;
        }
        break;

    case ARM_I2C_HS_MASTER_ADDR:
        if (I2C->mode != I2C_MASTER_MODE) {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        /* Sets High speed master address */
        i2c_master_set_hs_maddr(I2C->regs, arg);
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 * @brief   CMSIS-Driver i2c get status
 * @note    none
 * @param   I2C : Pointer to i2c resources structure
 * @retval  ARM_i2c_STATUS
 */
static ARM_I2C_STATUS ARM_I2C_GetStatus(const I2C_RESOURCES *I2C)
{
    return I2C->status;
}

/**
 * @brief   CMSIS-Driver i2c irq error handler
 * @note    none
 * @param   I2C_RES  : Pointer to i2c resources structure
 * @retval  none
 */
void I2C_HandleIRQError(I2C_RESOURCES *I2C_RES)
{
    i2c_transfer_info_t *transfer = &(I2C_RES->transfer);
    ARM_I2C_STATUS      *i2c_stat = &(I2C_RES->status);

#if I2C_DMA_ENABLE
        /* Stop both DMA channels on error. Master RX uses both TX (cmd issuance)
         * and RX (data drain); stopping only one based on direction leaves the
         * other channel stalled. Stopping an inactive channel is harmless.
         */
        if (I2C_RES->dma_enable) {
            uint32_t dma_count = 0;

            I2C_DMA_Stop(&I2C_RES->dma_cfg->dma_tx);
            I2C_DMA_Stop(&I2C_RES->dma_cfg->dma_rx);

            /* Capture partial count for GetDataCount */
            if (i2c_stat->direction == I2C_DIR_TRANSMITTER) {
                (void) I2C_RES->dma_cfg->dma_tx.dma_drv->GetStatus(
                           &I2C_RES->dma_cfg->dma_tx.dma_handle, &dma_count);
                transfer->curr_cnt += dma_count / 2U;
            } else {
                (void) I2C_RES->dma_cfg->dma_rx.dma_drv->GetStatus(
                           &I2C_RES->dma_cfg->dma_rx.dma_handle, &dma_count);
                transfer->curr_cnt = dma_count;
            }
        }
#endif
    if (transfer->evt_sts & I2C_XFER_EVENT_INCOMPLETE) {
        I2C_RES->cb_event(ARM_I2C_EVENT_TRANSFER_DONE |
                           ARM_I2C_EVENT_TRANSFER_INCOMPLETE);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_ADDR_NOACK) {
        I2C_RES->cb_event(ARM_I2C_EVENT_ADDRESS_NACK);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_ARBITRATION_LOST) {
        i2c_stat->arbitration_lost = 1U;

        I2C_RES->cb_event(ARM_I2C_EVENT_ARBITRATION_LOST);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_USER_ABORT) {
        I2C_RES->cb_event(ARM_I2C_EVENT_USER_ABORT    |
                          ARM_I2C_EVENT_TRANSFER_DONE |
                          ARM_I2C_EVENT_TRANSFER_INCOMPLETE);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_GCALL_ERR) {
        i2c_stat->bus_error = 1U;
        I2C_RES->cb_event((ARM_I2C_EVENT_GCALL_ERROR  |
                           ARM_I2C_EVENT_BUS_ERROR));

    } else if (transfer->evt_sts & I2C_XFER_EVENT_NO_RESTART) {
        I2C_RES->cb_event(ARM_I2C_EVENT_RESTART_DISABLED);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_SDA_STUCK_AT_LOW) {
        i2c_stat->bus_error = 1U;
        I2C_RES->cb_event(ARM_I2C_EVENT_SDA_STUCK_LOW  |
                          ARM_I2C_EVENT_BUS_ERROR);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_SCL_STUCK_AT_LOW) {
        i2c_stat->bus_error = 1U;
        I2C_RES->cb_event(ARM_I2C_EVENT_SCL_STUCK_LOW  |
                          ARM_I2C_EVENT_BUS_ERROR);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_UNEXPECTED_ACK) {
        I2C_RES->cb_event(ARM_I2C_EVENT_UNEXPECTED_ACK |
                          ARM_I2C_EVENT_BUS_ERROR);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_DEV_ID_NOACK) {
        I2C_RES->cb_event(ARM_I2C_EVENT_DEV_ID_NACK);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_MASTER_DIS) {
        I2C_RES->cb_event(ARM_I2C_EVENT_MASTER_DISABLED);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_RX_IN_TX_MODE) {
        I2C_RES->cb_event(ARM_I2C_EVENT_RX_IN_TX_MODE);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_DEV_ID_WRITE) {
        I2C_RES->cb_event(ARM_I2C_EVENT_DEV_ID_TX_DATA);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_TX_FIFO_FLUSH) {
        I2C_RES->cb_event(ARM_I2C_EVENT_TX_FIFO_FLUSHED);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_UNDEF_TX_ABORT) {
        I2C_RES->cb_event(ARM_I2C_EVENT_UNDEF_TX_ABORT);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_MASTER_ON_HOLD) {
        I2C_RES->cb_event(ARM_I2C_EVENT_MASTER_ON_HOLD);
    }
}

/**
 * @brief   CMSIS-Driver i2c irq status handler
 * @note    none
 * @param   I2C_RES : Pointer to i2c resources structure
 * @retval  none
 */
void I2C_HandleIRQStatus(I2C_RESOURCES *I2C_RES)
{
    i2c_transfer_info_t *transfer = &(I2C_RES->transfer);
    ARM_I2C_STATUS      *i2c_stat = &(I2C_RES->status);

    /* Check the ISR response */
    if (transfer->evt_sts & I2C_XFER_EVENT_DONE) {

#if I2C_DMA_ENABLE
        /* Successful end-of-transaction: curr_cnt = full transfer size */
        if (I2C_RES->dma_enable) {
            transfer->curr_cnt = I2C_RES->dma_xfer_total;
        }
#endif

        I2C_RES->cb_event(ARM_I2C_EVENT_TRANSFER_DONE);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_GCALL) {
        I2C_RES->cb_event(ARM_I2C_EVENT_GENERAL_CALL | ARM_I2C_EVENT_TRANSFER_DONE);

    } else if (transfer->evt_sts & I2C_XFER_EVENT_BUS_CLEAR) {
        I2C_RES->cb_event(ARM_I2C_EVENT_BUS_CLEAR);

    } else {
        I2C_HandleIRQError(I2C_RES);
    }
    /* set busy flag to 0U */
    i2c_stat->busy = 0U;
    transfer->evt_sts = I2C_XFER_EVENT_NONE;
}

/**
 * @brief   CMSIS-Driver i2c irq handler
 * @note    none
 * @param   I2C_RES : Pointer to i2c resources structure
 * @retval  none
 */
void I2C_IRQHandler(I2C_RESOURCES *I2C_RES)
{
    i2c_transfer_info_t *transfer = &(I2C_RES->transfer);

    /* Stale-IRQ guard: an DMA abort cleanup path
     * may have cleared curr_stat before the caller's unmask sequence
     * runs. The resulting level-sensitive IRQ (TX_EMPTY, etc.) would
     * re-enter forever. Silence it here.
     */
    if (transfer->curr_stat == I2C_XFER_NONE) {
        i2c_mask_interrupt(I2C_RES->regs, 0xFFFFFFFFU);
        i2c_clear_all_interrupt(I2C_RES->regs);
        return;
    }

#if I2C_DMA_ENABLE
    I2C_XFER_STATE prev_state = transfer->curr_stat;
#endif

    /* Check for master mode */
    if (I2C_RES->mode == I2C_MASTER_MODE) {
        if (transfer->curr_stat == I2C_XFER_MST_TX) {
            /* Master transmit*/
            i2c_master_tx_isr(I2C_RES->regs, transfer);
        }
        if (transfer->curr_stat == I2C_XFER_MST_RX) {
            /* Master receive */
            i2c_master_rx_isr(I2C_RES->regs, transfer);
        }
    } else /* Slave mode */ {
        if (transfer->curr_stat == I2C_XFER_SLV_TX) {
            /* slave transmit*/
            i2c_slave_tx_isr(I2C_RES->regs, transfer);
        }
        if (transfer->curr_stat == I2C_XFER_SLV_RX) {
            /* slave receive */
            i2c_slave_rx_isr(I2C_RES->regs, transfer);
        }
    } /* Slave mode */

#if I2C_DMA_ENABLE
    /* Stop the DMA channel(s) when transfer completes via STOP_DET.
     * TX transfers: stop TX channel. RX transfers: stop RX channel.
     */
    if (I2C_RES->dma_enable && (transfer->curr_stat == I2C_XFER_NONE)) {
        if ((prev_state == I2C_XFER_MST_TX) || (prev_state == I2C_XFER_SLV_TX)
             || (prev_state == I2C_XFER_MST_RX)) {
            (void) I2C_DMA_Stop(&I2C_RES->dma_cfg->dma_tx);
        }
        if ((prev_state == I2C_XFER_MST_RX) || (prev_state == I2C_XFER_SLV_RX)) {
            (void) I2C_DMA_Stop(&I2C_RES->dma_cfg->dma_rx);
        }
    }
#endif

    /* Handle the status only if any event occurred */
    if (transfer->evt_sts != I2C_XFER_EVENT_NONE) {
        I2C_HandleIRQStatus(I2C_RES);
    }
}

#if I2C_DMA_ENABLE

typedef enum {
    I2C_DMA_DRAIN_DONE,    /* bus went idle cleanly — caller emits TRANSFER_DONE */
    I2C_DMA_DRAIN_ABORT,   /* TX_ABRT latched — caller returns without emit so the
                            * pending I2C ISR delivers the error event
                            */
    I2C_DMA_DRAIN_TIMEOUT, /* ~3 ms elapsed — caller emits BUS_ERROR | INCOMPLETE  */
} i2c_dma_drain_result_t;

/* Bound on the FIFO+shift-register drain. Worst case is Standard mode
 * (100 kHz) with a full 32-deep TX FIFO: 32 * 90 us ≈ 2.88 ms.
 */
#define I2C_DMA_DRAIN_TIMEOUT_US 3000U

/* sys_busy_loop_us() resolves to a single tick (~30.51 us) */
#define I2C_DMA_DRAIN_POLL_STEP_US 30U
#define I2C_DMA_DRAIN_MAX_ITERS                                                                    \
    ((I2C_DMA_DRAIN_TIMEOUT_US + I2C_DMA_DRAIN_POLL_STEP_US - 1U) / I2C_DMA_DRAIN_POLL_STEP_US)

/* One Standard-mode (100 kHz) byte time covers the shift register tail
 * after IC_STATUS.TFE asserts. Faster modes finish sooner; the extra
 * wait is harmless.
 */
#define I2C_DMA_SHIFT_TAIL_US 90U

/* Poll for the FIFO+shift-register drain that follows DMA COMPLETE on a
 * TX transfer. DMA COMPLETE only means the bytes are
 * in the I2C TX FIFO; the controller may still be clocking them onto SDA
 * and a late slave NACK or arbitration loss raises TX_ABRT *after* DMA
 * reports success.
 *
 * Called from the DMA ISR — the I2C ISR cannot run until this returns,
 * so we inspect IC_STATUS / IC_RAW_INTR_STAT directly instead of waiting
 * for the TX_ABRT handler. TFE reports FIFO empty but not shift-register
 * empty, hence the fixed ~90 us tail wait on the clean path.
 */
static i2c_dma_drain_result_t i2c_dma_wait_drain(I2C_RESOURCES *I2C)
{
    for (uint32_t i = 0U; i < I2C_DMA_DRAIN_MAX_ITERS; i++) {
        if (i2c_tx_abort_pending(I2C->regs)) {
            return I2C_DMA_DRAIN_ABORT;
        }
        if (i2c_tx_fifo_empty(I2C->regs)) {
            sys_busy_loop_us(I2C_DMA_SHIFT_TAIL_US);
            /* Re-check for an abort that fired during the tail wait. */
            if (i2c_tx_abort_pending(I2C->regs)) {
                return I2C_DMA_DRAIN_ABORT;
            }
            return I2C_DMA_DRAIN_DONE;
        }
        sys_busy_loop_us(I2C_DMA_DRAIN_POLL_STEP_US);
    }
    return I2C_DMA_DRAIN_TIMEOUT;
}

/**
 * @brief   Callback function from DMA for I2C
 * @param   event    : Event from DMA
 * @param   peri_num : Peripheral number
 * @param   I2C      : Pointer to I2C resources
 * @retval  execution_status
 */
static void I2C_DMACallback(uint32_t event, int8_t peri_num, I2C_RESOURCES *I2C)
{
    if (!I2C->cb_event) {
        return;
    }

    /* The I2C ISR owns end-of-transaction delivery. If it has already
     * driven the transfer back to I2C_XFER_NONE (STOP_DET path), any DMA event
     * arriving afterwards is a late echo.
     */
    if (I2C->transfer.curr_stat == I2C_XFER_NONE) {
        return;
    }

    /* DMA ABORT: the I2C controller is wedged silently (no STOP_DET /
     * TX_ABRT will fire on its own. Perform full inline teardown here so the
     * driver can recover without depending on the I2C ISR.
     */
    if ((event & ARM_DMA_EVENT_ABORT) && (!I2C->transfer.abort)) {
        uint32_t dma_count = 0U;

        /* Mask all I2C IRQs first so stray events can't race teardown. */
        i2c_mask_interrupt(I2C->regs, 0xFFFFFFFFU);

        /* Stop both DMA channels */
        (void) I2C_DMA_Stop(&I2C->dma_cfg->dma_tx);
        (void) I2C_DMA_Stop(&I2C->dma_cfg->dma_rx);

        /* Clear DMA enable bits on the I2C controller. */
        i2c_disable_tx_dma(I2C->regs);
        i2c_disable_rx_dma(I2C->regs);

        /* Best-effort bus-release for master. Only set ABORT if the
         * controller is currently active — ABORT is self-clearing only
         * when the HW performs the abort. Setting it on an idle
         * controller leaves the bit latched and silently aborts the
         * *next* MasterTransmit/Receive.
         */
        if ((I2C->status.mode == I2C_MASTER_MODE) &&
            ((I2C->regs->I2C_STATUS & I2C_IC_STATUS_MASTER_ACT) != 0U)) {
            i2c_master_abort(I2C->regs);
        }
        /* Capture partial count for GetDataCount(). */
        if (I2C->status.direction == I2C_DIR_TRANSMITTER) {
            (void) I2C->dma_cfg->dma_tx.dma_drv->GetStatus(
                       &I2C->dma_cfg->dma_tx.dma_handle, &dma_count);
            I2C->transfer.curr_cnt += dma_count / 2U;
        } else {
            (void) I2C->dma_cfg->dma_rx.dma_drv->GetStatus(
                       &I2C->dma_cfg->dma_rx.dma_handle, &dma_count);
            I2C->transfer.curr_cnt = dma_count;
        }

        I2C->transfer.curr_stat = I2C_XFER_NONE;
        I2C->dma_xfer_pending   = false;
        I2C->status.busy        = 0U;

        I2C->cb_event(ARM_I2C_EVENT_TRANSFER_INCOMPLETE);

        return;
    }

    /* TX-channel chunk completed. For transfers larger than one chunk we
     * pack the next slice into scratch (TX-side) or generate the next
     * READ_REQ batch (RX cmd issuance), rebuild microcode, and restart the
     * channel.
     *
     * The final user event is delivered by the I2C ISR on STOP_DET
     * (master) or the slave drain path — not by the DMA callback. When
     * dma_xfer_remaining reaches 0 we do nothing here.
     *
     * Exception: when xfer_pending=true (RESTART requested), no STOP is
     * sent, so STOP_DET never fires. In that case, we signal DONE here
     * once the final chunk completes.
     */
    if (event & ARM_DMA_EVENT_COMPLETE) {
        if ((I2C->dma_xfer_remaining > 0U) && (!I2C->transfer.abort)) {
            switch (I2C->transfer.curr_stat) {
            case I2C_XFER_MST_TX:
                I2C->transfer.curr_cnt = I2C->dma_xfer_total - I2C->dma_xfer_remaining;
                if (i2c_dma_tx_submit_chunk(I2C, true) != ARM_DRIVER_OK) {
                    goto rearm_failed;
                }
                break;
            case I2C_XFER_SLV_TX:
                I2C->transfer.curr_cnt = I2C->dma_xfer_total - I2C->dma_xfer_remaining;
                if (i2c_dma_tx_submit_chunk(I2C, false) != ARM_DRIVER_OK) {
                    goto rearm_failed;
                }
                break;
            case I2C_XFER_MST_RX:
                /* Re-arm fires on TX-channel COMPLETE only; the RX-channel
                 * COMPLETE arrives after the full drain finishes and at
                 * that point dma_xfer_remaining is already 0.
                 */
                if (peri_num == I2C->dma_cfg->dma_tx.dma_periph_req) {
                    I2C->transfer.curr_cnt = I2C->dma_xfer_total - I2C->dma_xfer_remaining;
                    if (i2c_dma_rx_cmd_submit_chunk(I2C, 0U) != ARM_DRIVER_OK) {
                        goto rearm_failed;
                    }
                }
                break;
            default:
                break;
            }
            return;

rearm_failed:
            i2c_mask_interrupt(I2C->regs, 0xFFFFFFFFU);
            (void) I2C_DMA_Stop(&I2C->dma_cfg->dma_tx);
            (void) I2C_DMA_Stop(&I2C->dma_cfg->dma_rx);
            i2c_disable_tx_dma(I2C->regs);
            i2c_disable_rx_dma(I2C->regs);

            if ((I2C->status.mode == I2C_MASTER_MODE) &&
                ((I2C->regs->I2C_STATUS & I2C_IC_STATUS_MASTER_ACT) != 0U)) {
                i2c_master_abort(I2C->regs);
            }

            I2C->transfer.curr_stat = I2C_XFER_NONE;
            I2C->dma_xfer_pending   = false;
            I2C->status.busy        = 0U;
            I2C->cb_event(ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
            return;
        } else if ((I2C->dma_xfer_remaining == 0U) &&
                   (I2C->transfer.curr_stat == I2C_XFER_MST_TX) &&
                   (I2C->dma_xfer_pending) &&
                   (!I2C->transfer.abort)) {
            /* xfer_pending=true: no STOP will be sent, so STOP_DET won't
             * fire. Wait for the FIFO+shift-register drain to finish
             * before signalling completion. A late TX_ABRT raised during
             * drain is delivered via the pending I2C ISR by returning
             * from this DMA cb without emit (interrupts must stay
             * unmasked until then, so the disable+emit only happens on
             * the clean-drain path).
             */
            switch (i2c_dma_wait_drain(I2C)) {
            case I2C_DMA_DRAIN_ABORT:
                /* Pending I2C TX_ABRT ISR routes through HandleIRQError. */
                return;
            case I2C_DMA_DRAIN_TIMEOUT:
                i2c_master_disable_tx_interrupt(I2C->regs);
                i2c_disable_tx_dma(I2C->regs);
                I2C->status.bus_error   = 1U;
                I2C->transfer.curr_cnt  = I2C->dma_xfer_total -
                                          i2c_tx_fifo_level(I2C->regs);
                I2C->transfer.curr_stat = I2C_XFER_NONE;
                I2C->status.busy        = 0U;
                I2C->cb_event(ARM_I2C_EVENT_BUS_ERROR |
                              ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                break;
            case I2C_DMA_DRAIN_DONE:
            default:
                i2c_master_disable_tx_interrupt(I2C->regs);
                i2c_disable_tx_dma(I2C->regs);
                I2C->transfer.curr_cnt  = I2C->dma_xfer_total;
                I2C->transfer.curr_stat = I2C_XFER_NONE;
                I2C->status.busy        = 0U;
                I2C->cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
                break;
            }
        } else if ((I2C->dma_xfer_remaining == 0U) &&
                   (I2C->transfer.curr_stat == I2C_XFER_MST_RX) &&
                   (I2C->dma_xfer_pending) &&
                   (!I2C->transfer.abort)) {
            /* xfer_pending=true: no STOP will be sent, so STOP_DET won't
             * fire. Signal completion here instead.
             */
            if (peri_num == I2C->dma_cfg->dma_tx.dma_periph_req) {
                /* TX channel completed first; RX channel will complete later */
                i2c_master_disable_tx_interrupt(I2C->regs);
                i2c_disable_tx_dma(I2C->regs);
            } else {
                i2c_master_disable_rx_interrupt(I2C->regs);
                i2c_disable_rx_dma(I2C->regs);
                I2C->transfer.curr_cnt  = I2C->dma_xfer_total;
                I2C->transfer.curr_stat = I2C_XFER_NONE;
                I2C->status.busy        = 0U;
                I2C->cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
            }
        } else if ((I2C->dma_xfer_remaining == 0U) &&
                   (I2C->transfer.curr_stat == I2C_XFER_SLV_RX) &&
                   (!I2C->transfer.abort)) {
            /* Slave RX DMA completed all expected bytes. Signal DONE.
             * Normally STOP_DET would signal completion, but in W/R combined
             * mode the master sends RESTART instead of STOP.
             */
            i2c_slave_disable_rx_interrupt(I2C->regs);
            i2c_disable_rx_dma(I2C->regs);
            I2C->transfer.curr_cnt  = I2C->dma_xfer_total;
            I2C->transfer.curr_stat = I2C_XFER_NONE;
            I2C->status.busy        = 0U;
            I2C->cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
        } else if ((I2C->dma_xfer_remaining == 0U) &&
                   (I2C->transfer.curr_stat == I2C_XFER_SLV_TX) &&
                   (!I2C->transfer.abort)) {
            /* Slave TX DMA finished pushing all bytes into the TX FIFO.
             * When the master is using xfer_pending master-RX, no STOP
             * fires and STOP_DET will not signal completion, so the DMA
             * cb must emit it. Wait for FIFO+shift drain first so a late
             * TX_ABRT (arbitration loss, slave-flush, etc.) is delivered
             * via the pending I2C ISR through HandleIRQError instead of
             * being masked by a premature TRANSFER_DONE.
             */
            switch (i2c_dma_wait_drain(I2C)) {
            case I2C_DMA_DRAIN_ABORT:
                /* Pending I2C TX_ABRT ISR routes through HandleIRQError. */
                return;
            case I2C_DMA_DRAIN_TIMEOUT:
                i2c_slave_disable_tx_interrupt(I2C->regs);
                i2c_disable_tx_dma(I2C->regs);
                I2C->status.bus_error   = 1U;
                I2C->transfer.curr_cnt  = I2C->dma_xfer_total -
                                          i2c_tx_fifo_level(I2C->regs);
                I2C->transfer.curr_stat = I2C_XFER_NONE;
                I2C->status.busy        = 0U;
                I2C->cb_event(ARM_I2C_EVENT_BUS_ERROR |
                              ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                break;
            case I2C_DMA_DRAIN_DONE:
            default:
                i2c_slave_disable_tx_interrupt(I2C->regs);
                i2c_disable_tx_dma(I2C->regs);
                I2C->transfer.curr_cnt  = I2C->dma_xfer_total;
                I2C->transfer.curr_stat = I2C_XFER_NONE;
                I2C->status.busy        = 0U;
                I2C->cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
                break;
            }
        }
    }
}
#endif

/* I2C0 Driver Instance */
#if (RTE_I2C0)

#if RTE_I2C0_DMA_ENABLE
static uint16_t I2C0_DMA_TX_SCRATCH[RTE_I2C0_DMA_SCRATCH_SIZE];
static void I2C0_DMACallback(uint32_t event, int8_t peri_num);

static I2C_DMA_HW_CONFIG I2C0_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C0_DMA),
        .dma_periph_req = I2C0_DMA_RX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C0_DMA,
            .group            = I2C0_DMA_GROUP,
            .channel          = I2C0_DMA_RX_PERIPH_REQ,
            .enable_handshake = I2C0_DMA_HANDSHAKE_ENABLE,
        },
    },
    .dma_tx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C0_DMA),
        .dma_periph_req = I2C0_DMA_TX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C0_DMA,
            .group            = I2C0_DMA_GROUP,
            .channel          = I2C0_DMA_TX_PERIPH_REQ,
            .enable_handshake = I2C0_DMA_HANDSHAKE_ENABLE,
        },
    },
};
#endif

/* I2C0 Driver Resources */
static I2C_RESOURCES I2C0_RES = {
    .regs              = (I2C_Type *) I2C0_BASE,
    .irq_num           = (IRQn_Type) I2C0_IRQ_IRQn,
    .irq_priority      = (uint32_t) RTE_I2C0_IRQ_PRIORITY,
#if RTE_I2C0_DMA_ENABLE
    .dma_enable        = RTE_I2C0_DMA_ENABLE,
    .dma_irq_priority  = RTE_I2C0_DMA_IRQ_PRI,
    .dma_cb            = I2C0_DMACallback,
    .dma_cfg           = &I2C0_DMA_HW_CONFIG,
    .dma_tx_scratch     = I2C0_DMA_TX_SCRATCH,
    .dma_tx_scratch_sz  = RTE_I2C0_DMA_SCRATCH_SIZE,
#endif
    .tx_fifo_threshold = RTE_I2C0_TX_FIFO_THRESHOLD,
    .rx_fifo_threshold = RTE_I2C0_RX_FIFO_THRESHOLD,
    .scl_stuck_timeout = RTE_I2C0_SCL_STUCK_LOW_TIMEOUT,
    .instance          = I2C_INSTANCE_0
};

static int32_t I2C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C0_RES);
}

static int32_t I2C0_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C0_RES);
}

static int32_t I2C0_PowerControl(ARM_POWER_STATE state)
{
    return ARM_I2C_PowerControl(state, &I2C0_RES);
}

static int32_t I2C0_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num,
                                   bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C0_RES, addr, data, num, xfer_pending);
}

static int32_t I2C0_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterReceive(&I2C0_RES, addr, data, num, xfer_pending);
}

static int32_t I2C0_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveTransmit(&I2C0_RES, data, num);
}

static int32_t I2C0_SlaveReceive(uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveReceive(&I2C0_RES, data, num);
}

static int32_t I2C0_GetDataCount(void)
{
    return ARM_I2C_GetDataCount(&I2C0_RES);
}

static int32_t I2C0_Control(uint32_t control, uint32_t arg)
{
    return ARM_I2C_Control(&I2C0_RES, control, arg);
}

static ARM_I2C_STATUS I2C0_GetStatus(void)
{
    return ARM_I2C_GetStatus(&I2C0_RES);
}

void I2C0_IRQHandler(void)
{
    I2C_IRQHandler(&I2C0_RES);
}

#if RTE_I2C0_DMA_ENABLE
static void I2C0_DMACallback(uint32_t event, int8_t peri_num)
{
    I2C_DMACallback(event, peri_num, &I2C0_RES);
}
#endif

/* I2C0 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C0;
ARM_DRIVER_I2C        Driver_I2C0 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C0_Initialize,
    I2C0_Uninitialize,
    I2C0_PowerControl,
    I2C0_MasterTransmit,
    I2C0_MasterReceive,
    I2C0_SlaveTransmit,
    I2C0_SlaveReceive,
    I2C0_GetDataCount,
    I2C0_Control,
    I2C0_GetStatus
};
#endif /* RTE_I2C0 */

/* I2C1 Driver Instance */
#if (RTE_I2C1)

#if RTE_I2C1_DMA_ENABLE
static uint16_t I2C1_DMA_TX_SCRATCH[RTE_I2C1_DMA_SCRATCH_SIZE];
static void I2C1_DMACallback(uint32_t event, int8_t peri_num);

static I2C_DMA_HW_CONFIG I2C1_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C1_DMA),
        .dma_periph_req = I2C1_DMA_RX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C1_DMA,
            .group            = I2C1_DMA_GROUP,
            .channel          = I2C1_DMA_RX_PERIPH_REQ,
            .enable_handshake = I2C1_DMA_HANDSHAKE_ENABLE,
        },
    },
    .dma_tx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C1_DMA),
        .dma_periph_req = I2C1_DMA_TX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C1_DMA,
            .group            = I2C1_DMA_GROUP,
            .channel          = I2C1_DMA_TX_PERIPH_REQ,
            .enable_handshake = I2C1_DMA_HANDSHAKE_ENABLE,
        },
    },
};
#endif

/* I2C1 Driver Resources */
static I2C_RESOURCES I2C1_RES = {
    .regs              = (I2C_Type *) I2C1_BASE,
    .irq_num           = (IRQn_Type) I2C1_IRQ_IRQn,
    .irq_priority      = (uint32_t) RTE_I2C1_IRQ_PRIORITY,
#if RTE_I2C1_DMA_ENABLE
    .dma_enable        = RTE_I2C1_DMA_ENABLE,
    .dma_irq_priority  = RTE_I2C1_DMA_IRQ_PRI,
    .dma_cb            = I2C1_DMACallback,
    .dma_cfg           = &I2C1_DMA_HW_CONFIG,
    .dma_tx_scratch     = I2C1_DMA_TX_SCRATCH,
    .dma_tx_scratch_sz  = RTE_I2C1_DMA_SCRATCH_SIZE,
#endif
    .tx_fifo_threshold = RTE_I2C1_TX_FIFO_THRESHOLD,
    .rx_fifo_threshold = RTE_I2C1_RX_FIFO_THRESHOLD,
    .scl_stuck_timeout = RTE_I2C1_SCL_STUCK_LOW_TIMEOUT,
    .instance          = I2C_INSTANCE_1
};

static int32_t I2C1_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C1_RES);
}

static int32_t I2C1_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C1_RES);
}

static int32_t I2C1_PowerControl(ARM_POWER_STATE state)
{
    return ARM_I2C_PowerControl(state, &I2C1_RES);
}

static int32_t I2C1_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num,
                                   bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C1_RES, addr, data, num, xfer_pending);
}

static int32_t I2C1_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterReceive(&I2C1_RES, addr, data, num, xfer_pending);
}

static int32_t I2C1_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveTransmit(&I2C1_RES, data, num);
}

static int32_t I2C1_SlaveReceive(uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveReceive(&I2C1_RES, data, num);
}

static int32_t I2C1_GetDataCount(void)
{
    return ARM_I2C_GetDataCount(&I2C1_RES);
}

static int32_t I2C1_Control(uint32_t control, uint32_t arg)
{
    return ARM_I2C_Control(&I2C1_RES, control, arg);
}

static ARM_I2C_STATUS I2C1_GetStatus(void)
{
    return ARM_I2C_GetStatus(&I2C1_RES);
}

void I2C1_IRQHandler(void)
{
    I2C_IRQHandler(&I2C1_RES);
}

#if RTE_I2C1_DMA_ENABLE
static void I2C1_DMACallback(uint32_t event, int8_t peri_num)
{
    I2C_DMACallback(event, peri_num, &I2C1_RES);
}
#endif

/* I2C1 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C1;
ARM_DRIVER_I2C        Driver_I2C1 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C1_Initialize,
    I2C1_Uninitialize,
    I2C1_PowerControl,
    I2C1_MasterTransmit,
    I2C1_MasterReceive,
    I2C1_SlaveTransmit,
    I2C1_SlaveReceive,
    I2C1_GetDataCount,
    I2C1_Control,
    I2C1_GetStatus
};
#endif /* RTE_I2C1 */

/* I2C2 Driver Instance */
#if (RTE_I2C2)

#if RTE_I2C2_DMA_ENABLE
static uint16_t I2C2_DMA_TX_SCRATCH[RTE_I2C2_DMA_SCRATCH_SIZE];
static void I2C2_DMACallback(uint32_t event, int8_t peri_num);

static I2C_DMA_HW_CONFIG I2C2_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C2_DMA),
        .dma_periph_req = I2C2_DMA_RX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C2_DMA,
            .group            = I2C2_DMA_GROUP,
            .channel          = I2C2_DMA_RX_PERIPH_REQ,
            .enable_handshake = I2C2_DMA_HANDSHAKE_ENABLE,
        },
    },
    .dma_tx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C2_DMA),
        .dma_periph_req = I2C2_DMA_TX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C2_DMA,
            .group            = I2C2_DMA_GROUP,
            .channel          = I2C2_DMA_TX_PERIPH_REQ,
            .enable_handshake = I2C2_DMA_HANDSHAKE_ENABLE,
        },
    },
};
#endif

/* I2C2 Driver Resources */
static I2C_RESOURCES I2C2_RES = {
    .regs              = (I2C_Type *) I2C2_BASE,
    .irq_num           = (IRQn_Type) I2C2_IRQ_IRQn,
    .irq_priority      = (uint32_t) RTE_I2C2_IRQ_PRIORITY,
#if RTE_I2C2_DMA_ENABLE
    .dma_enable        = RTE_I2C2_DMA_ENABLE,
    .dma_irq_priority  = RTE_I2C2_DMA_IRQ_PRI,
    .dma_cb            = I2C2_DMACallback,
    .dma_cfg           = &I2C2_DMA_HW_CONFIG,
    .dma_tx_scratch     = I2C2_DMA_TX_SCRATCH,
    .dma_tx_scratch_sz  = RTE_I2C2_DMA_SCRATCH_SIZE,
#endif
    .tx_fifo_threshold = RTE_I2C2_TX_FIFO_THRESHOLD,
    .rx_fifo_threshold = RTE_I2C2_RX_FIFO_THRESHOLD,
    .scl_stuck_timeout = RTE_I2C2_SCL_STUCK_LOW_TIMEOUT,
    .instance          = I2C_INSTANCE_2
};

static int32_t I2C2_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C2_RES);
}

static int32_t I2C2_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C2_RES);
}

static int32_t I2C2_PowerControl(ARM_POWER_STATE state)
{
    return ARM_I2C_PowerControl(state, &I2C2_RES);
}

static int32_t I2C2_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num,
                                   bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C2_RES, addr, data, num, xfer_pending);
}

static int32_t I2C2_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterReceive(&I2C2_RES, addr, data, num, xfer_pending);
}

static int32_t I2C2_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveTransmit(&I2C2_RES, data, num);
}

static int32_t I2C2_SlaveReceive(uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveReceive(&I2C2_RES, data, num);
}

static int32_t I2C2_GetDataCount(void)
{
    return ARM_I2C_GetDataCount(&I2C2_RES);
}

static int32_t I2C2_Control(uint32_t control, uint32_t arg)
{
    return ARM_I2C_Control(&I2C2_RES, control, arg);
}

static ARM_I2C_STATUS I2C2_GetStatus(void)
{
    return ARM_I2C_GetStatus(&I2C2_RES);
}

void I2C2_IRQHandler(void)
{
    I2C_IRQHandler(&I2C2_RES);
}

#if RTE_I2C2_DMA_ENABLE
static void I2C2_DMACallback(uint32_t event, int8_t peri_num)
{
    I2C_DMACallback(event, peri_num, &I2C2_RES);
}
#endif

/* I2C2 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C2;
ARM_DRIVER_I2C        Driver_I2C2 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C2_Initialize,
    I2C2_Uninitialize,
    I2C2_PowerControl,
    I2C2_MasterTransmit,
    I2C2_MasterReceive,
    I2C2_SlaveTransmit,
    I2C2_SlaveReceive,
    I2C2_GetDataCount,
    I2C2_Control,
    I2C2_GetStatus
};
#endif /* RTE_I2C2 */

/* I2C3 Driver Instance */
#if (RTE_I2C3)

#if RTE_I2C3_DMA_ENABLE
static uint16_t I2C3_DMA_TX_SCRATCH[RTE_I2C3_DMA_SCRATCH_SIZE];
static void I2C3_DMACallback(uint32_t event, int8_t peri_num);

static I2C_DMA_HW_CONFIG I2C3_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C3_DMA),
        .dma_periph_req = I2C3_DMA_RX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C3_DMA,
            .group            = I2C3_DMA_GROUP,
            .channel          = I2C3_DMA_RX_PERIPH_REQ,
            .enable_handshake = I2C3_DMA_HANDSHAKE_ENABLE,
        },
    },
    .dma_tx = {
        .dma_drv        = &ARM_Driver_DMA_(I2C3_DMA),
        .dma_periph_req = I2C3_DMA_TX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = I2C3_DMA,
            .group            = I2C3_DMA_GROUP,
            .channel          = I2C3_DMA_TX_PERIPH_REQ,
            .enable_handshake = I2C3_DMA_HANDSHAKE_ENABLE,
         },
    },
};
#endif

/* I2C3 Driver Resources */
static I2C_RESOURCES I2C3_RES = {
    .regs              = (I2C_Type *) I2C3_BASE,
    .irq_num           = (IRQn_Type) I2C3_IRQ_IRQn,
    .irq_priority      = (uint32_t) RTE_I2C3_IRQ_PRIORITY,
#if RTE_I2C3_DMA_ENABLE
    .dma_enable        = RTE_I2C3_DMA_ENABLE,
    .dma_irq_priority  = RTE_I2C3_DMA_IRQ_PRI,
    .dma_cb            = I2C3_DMACallback,
    .dma_cfg           = &I2C3_DMA_HW_CONFIG,
    .dma_tx_scratch     = I2C3_DMA_TX_SCRATCH,
    .dma_tx_scratch_sz  = RTE_I2C3_DMA_SCRATCH_SIZE,
#endif
    .tx_fifo_threshold = RTE_I2C3_TX_FIFO_THRESHOLD,
    .rx_fifo_threshold = RTE_I2C3_RX_FIFO_THRESHOLD,
    .scl_stuck_timeout = RTE_I2C3_SCL_STUCK_LOW_TIMEOUT,
    .instance          = I2C_INSTANCE_3
};

static int32_t I2C3_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C3_RES);
}

static int32_t I2C3_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C3_RES);
}

static int32_t I2C3_PowerControl(ARM_POWER_STATE state)
{
    return ARM_I2C_PowerControl(state, &I2C3_RES);
}

static int32_t I2C3_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num,
                                   bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C3_RES, addr, data, num, xfer_pending);
}

static int32_t I2C3_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterReceive(&I2C3_RES, addr, data, num, xfer_pending);
}

static int32_t I2C3_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveTransmit(&I2C3_RES, data, num);
}

static int32_t I2C3_SlaveReceive(uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveReceive(&I2C3_RES, data, num);
}

static int32_t I2C3_GetDataCount(void)
{
    return ARM_I2C_GetDataCount(&I2C3_RES);
}

static int32_t I2C3_Control(uint32_t control, uint32_t arg)
{
    return ARM_I2C_Control(&I2C3_RES, control, arg);
}

static ARM_I2C_STATUS I2C3_GetStatus(void)
{
    return ARM_I2C_GetStatus(&I2C3_RES);
}

void I2C3_IRQHandler(void)
{
    I2C_IRQHandler(&I2C3_RES);
}

#if RTE_I2C3_DMA_ENABLE
static void I2C3_DMACallback(uint32_t event, int8_t peri_num)
{
    I2C_DMACallback(event, peri_num, &I2C3_RES);
}
#endif

/* I2C3 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C3;
ARM_DRIVER_I2C        Driver_I2C3 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C3_Initialize,
    I2C3_Uninitialize,
    I2C3_PowerControl,
    I2C3_MasterTransmit,
    I2C3_MasterReceive,
    I2C3_SlaveTransmit,
    I2C3_SlaveReceive,
    I2C3_GetDataCount,
    I2C3_Control,
    I2C3_GetStatus
};
#endif /* RTE_I2C3 */

/* LPI2C1 Driver Instance */
#if (RTE_LPI2C1)

#if RTE_LPI2C1_DMA_ENABLE
static uint16_t LPI2C1_DMA_TX_SCRATCH[RTE_LPI2C1_DMA_SCRATCH_SIZE];
static void LPI2C1_DMACallback(uint32_t event, int8_t peri_num);

static I2C_DMA_HW_CONFIG LPI2C1_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(LPI2C1_DMA),
        .dma_periph_req = LPI2C1_DMA_RX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = LPI2C1_DMA,
            .group            = LPI2C1_DMA_GROUP,
            .channel          = LPI2C1_DMA_RX_PERIPH_REQ,
            .enable_handshake = LPI2C1_DMA_HANDSHAKE_ENABLE,
        },
    },
    .dma_tx = {
        .dma_drv        = &ARM_Driver_DMA_(LPI2C1_DMA),
        .dma_periph_req = LPI2C1_DMA_TX_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = LPI2C1_DMA,
            .group            = LPI2C1_DMA_GROUP,
            .channel          = LPI2C1_DMA_TX_PERIPH_REQ,
            .enable_handshake = LPI2C1_DMA_HANDSHAKE_ENABLE,
        },
    },
};
#endif

/* LPI2C1 Driver Resources */
static I2C_RESOURCES LPI2C1_RES = {
    .regs              = (I2C_Type *) LPI2C1_BASE,
    .irq_num           = (IRQn_Type) LPI2C1_IRQ_IRQn,
    .irq_priority      = (uint32_t) RTE_LPI2C1_IRQ_PRIORITY,
#if RTE_LPI2C1_DMA_ENABLE
    .dma_enable        = RTE_LPI2C1_DMA_ENABLE,
    .dma_irq_priority  = RTE_LPI2C1_DMA_IRQ_PRI,
    .dma_cb            = LPI2C1_DMACallback,
    .dma_cfg           = &LPI2C1_DMA_HW_CONFIG,
    .dma_tx_scratch     = LPI2C1_DMA_TX_SCRATCH,
    .dma_tx_scratch_sz  = RTE_LPI2C1_DMA_SCRATCH_SIZE,
#endif
    .tx_fifo_threshold = RTE_LPI2C1_TX_FIFO_THRESHOLD,
    .rx_fifo_threshold = RTE_LPI2C1_RX_FIFO_THRESHOLD,
    .scl_stuck_timeout = RTE_LPI2C1_SCL_STUCK_LOW_TIMEOUT,
    .instance          = I2C_INSTANCE_LP_1
};

static int32_t LPI2C1_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &LPI2C1_RES);
}

static int32_t LPI2C1_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&LPI2C1_RES);
}

static int32_t LPI2C1_PowerControl(ARM_POWER_STATE state)
{
    return ARM_I2C_PowerControl(state, &LPI2C1_RES);
}

static int32_t LPI2C1_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num,
                                     bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&LPI2C1_RES, addr, data, num, xfer_pending);
}

static int32_t LPI2C1_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterReceive(&LPI2C1_RES, addr, data, num, xfer_pending);
}

static int32_t LPI2C1_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveTransmit(&LPI2C1_RES, data, num);
}

static int32_t LPI2C1_SlaveReceive(uint8_t *data, uint32_t num)
{
    return ARM_I2C_SlaveReceive(&LPI2C1_RES, data, num);
}

static int32_t LPI2C1_GetDataCount(void)
{
    return ARM_I2C_GetDataCount(&LPI2C1_RES);
}

static int32_t LPI2C1_Control(uint32_t control, uint32_t arg)
{
    return ARM_I2C_Control(&LPI2C1_RES, control, arg);
}

static ARM_I2C_STATUS LPI2C1_GetStatus(void)
{
    return ARM_I2C_GetStatus(&LPI2C1_RES);
}

void LPI2C1_IRQHandler(void)
{
    I2C_IRQHandler(&LPI2C1_RES);
}

#if RTE_LPI2C1_DMA_ENABLE
static void LPI2C1_DMACallback(uint32_t event, int8_t peri_num)
{
    I2C_DMACallback(event, peri_num, &LPI2C1_RES);
}
#endif

/* I2CLP1 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2CLP1;
ARM_DRIVER_I2C        Driver_I2CLP1 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    LPI2C1_Initialize,
    LPI2C1_Uninitialize,
    LPI2C1_PowerControl,
    LPI2C1_MasterTransmit,
    LPI2C1_MasterReceive,
    LPI2C1_SlaveTransmit,
    LPI2C1_SlaveReceive,
    LPI2C1_GetDataCount,
    LPI2C1_Control,
    LPI2C1_GetStatus
};
#endif /* RTE_LPI2C1 */
/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
