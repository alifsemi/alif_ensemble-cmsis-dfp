/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Include */
#include "Driver_ADC_Private.h"
#include "sys_ctrl_analog.h"
#include "sys_utils.h"

#if ADC_DMA_ENABLE
#include "dma_opcode.h"
#include "dma_config.h"
#endif

#if defined(RTE_Drivers_ADC)

#define ARM_ADC_DRV_VERISON ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /*DRIVER VERSION*/

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion        = {ARM_ADC_API_VERSION, ARM_ADC_DRV_VERISON};

/* Driver Capabilities */
static const ARM_ADC_CAPABILITIES DriverCapabilities = {
    1, /* Resolution 12 or 20 bits*/
    0  /* Reserved                */
};

#if ADC_DMA_ENABLE

/*
 * @func         : int32_t ADC_DMA_Initialize(DMA_PERIPHERAL_CONFIG *dma_periph)
 * @brief        : Initialize the DMA driver
 * @parameter    : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @return       : ARM_DRIVER_OK    : DMA driver initialized successfully
 *                 ARM_DRIVER_ERROR : DMA driver reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_Initialize(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    return dma_periph->dma_drv->Initialize() ? ARM_DRIVER_ERROR : ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_DMA_Uninitialize(DMA_PERIPHERAL_CONFIG *dma_periph)
 * @brief        : Un-initialize the DMA driver
 * @parameter    : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @return       : ARM_DRIVER_OK    : DMA driver un-initialized successfully
 *                 ARM_DRIVER_ERROR : DMA driver reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_Uninitialize(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    return dma_periph->dma_drv->Uninitialize() ? ARM_DRIVER_ERROR : ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_DMA_PowerControl(ARM_POWER_STATE state,
 *                                              DMA_PERIPHERAL_CONFIG *dma_periph)
 * @brief        : Drive the DMA controller power state
 * @parameter[1] : state      : target power state
 * @parameter[2] : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @return       : ARM_DRIVER_OK    : power state applied successfully
 *                 ARM_DRIVER_ERROR : DMA driver reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_PowerControl(ARM_POWER_STATE        state,
                                             DMA_PERIPHERAL_CONFIG *dma_periph)
{
    return dma_periph->dma_drv->PowerControl(state) ? ARM_DRIVER_ERROR : ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_DMA_Allocate(DMA_PERIPHERAL_CONFIG *dma_periph)
 * @brief        : Allocate a DMA channel and wire the event-router
 *                 request/ack + handshake lines.
 * @parameter    : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @return       : ARM_DRIVER_OK    : channel allocated and evtrtr configured
 *                 ARM_DRIVER_ERROR : DMA driver Allocate reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_Allocate(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    if (dma_periph->dma_drv->Allocate(&dma_periph->dma_handle)) {
        return ARM_DRIVER_ERROR;
    }
    evtrtr_enable_dma_channel(dma_periph->evtrtr_cfg.instance,
                              dma_periph->evtrtr_cfg.channel,
                              dma_periph->evtrtr_cfg.group,
                              DMA_ACK_COMPLETION_PERIPHERAL);
    evtrtr_enable_dma_handshake(dma_periph->evtrtr_cfg.instance,
                                dma_periph->evtrtr_cfg.channel,
                                dma_periph->evtrtr_cfg.group);
    return ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_DMA_DeAllocate(DMA_PERIPHERAL_CONFIG *dma_periph)
 * @brief        : Deallocate the DMA channel and event-router wiring
 *                 previously set up by ADC_DMA_Allocate.
 * @parameter    : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @return       : ARM_DRIVER_OK    : channel released and evtrtr disabled
 *                 ARM_DRIVER_ERROR : DMA driver DeAllocate reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_DeAllocate(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    if (dma_periph->dma_drv->DeAllocate(&dma_periph->dma_handle)) {
        return ARM_DRIVER_ERROR;
    }
    evtrtr_disable_dma_channel(dma_periph->evtrtr_cfg.instance,
                               dma_periph->evtrtr_cfg.channel);
    evtrtr_disable_dma_handshake(dma_periph->evtrtr_cfg.instance,
                                 dma_periph->evtrtr_cfg.channel,
                                 dma_periph->evtrtr_cfg.group);
    return ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_DMA_Usermcode(DMA_PERIPHERAL_CONFIG *dma_periph,
 *                                           uint32_t mcode_addr)
 * @brief        : Point the DMA channel at a user-provided mcode program.
 * @parameter[1] : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @parameter[2] : mcode_addr : global-address view of the mcode entry point
 * @return       : ARM_DRIVER_OK    : mcode installed
 *                 ARM_DRIVER_ERROR : DMA driver Control reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_Usermcode(DMA_PERIPHERAL_CONFIG *dma_periph,
                                          uint32_t              mcode_addr)
{
    return dma_periph->dma_drv->Control(&dma_periph->dma_handle,
                                        ARM_DMA_USER_PROVIDED_MCODE,
                                        mcode_addr)
             ? ARM_DRIVER_ERROR
             : ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_DMA_Start(DMA_PERIPHERAL_CONFIG *dma_periph,
 *                                       ARM_DMA_PARAMS *dma_params)
 * @brief        : Start the DMA transfer described by dma_params.
 * @parameter[1] : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @parameter[2] : dma_params : pointer to ARM_DMA_PARAMS describing the transfer
 * @return       : ARM_DRIVER_OK    : transfer started
 *                 ARM_DRIVER_ERROR : DMA driver Start reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_Start(DMA_PERIPHERAL_CONFIG *dma_periph,
                                      ARM_DMA_PARAMS        *dma_params)
{
    return dma_periph->dma_drv->Start(&dma_periph->dma_handle, dma_params)
             ? ARM_DRIVER_ERROR
             : ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_DMA_Stop(DMA_PERIPHERAL_CONFIG *dma_periph)
 * @brief        : Halt the DMA channel used by this ADC.
 * @parameter    : dma_periph : pointer to DMA_PERIPHERAL_CONFIG for the RX channel
 * @return       : ARM_DRIVER_OK    : channel stopped
 *                 ARM_DRIVER_ERROR : DMA driver Stop reported failure
 */
__STATIC_INLINE int32_t ADC_DMA_Stop(DMA_PERIPHERAL_CONFIG *dma_periph)
{
    return dma_periph->dma_drv->Stop(&dma_periph->dma_handle) ? ARM_DRIVER_ERROR
                                                              : ARM_DRIVER_OK;
}

/*
 * @func         : uint32_t ADC_DMA_DeriveChunkSize(ADC_RESOURCES *ADC,
 *                                                  uint32_t active_ch,
 *                                                  uint32_t samples_per_buf)
 * @brief        : Pick the per-chunk sample count. Legacy path is bounded
 *                 by the mcode buffer budget; slim path is bounded by the
 *                 DMALP LC1/LC0 iteration cap.
 * @parameter[1] : ADC             : pointer to ADC_RESOURCES structure
 * @parameter[2] : active_ch       : number of unmasked channels per sweep
 * @parameter[3] : samples_per_buf : app buffer capacity in samples
 * @return       : chunk size in samples (>= active_ch)
 */
#if SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT
static uint32_t ADC_DMA_DeriveChunkSize(ADC_RESOURCES *ADC,
                                        uint32_t active_ch,
                                        uint32_t samples_per_buf)
{
    uint32_t max_c;
    uint32_t c;

    (void) ADC;

    /* Slim mcode chunk = outer * inner * active_ch, with each of outer
     * and inner in [1, ADC_DMA_LOOP_MAX]. Pick the largest c that (a) is a
     * multiple of active_ch, (b) divides samples_per_buf, and (c) has
     * cycles = c/active_ch factorable into outer*inner both <= LOOP_MAX.
     */
    max_c = ADC_DMA_LOOP_MAX * ADC_DMA_LOOP_MAX * active_ch;
    if (max_c > samples_per_buf) {
        max_c = samples_per_buf;
    }
    max_c -= (max_c % active_ch);

    for (c = max_c; c >= active_ch; c -= active_ch) {
        uint32_t cycles;
        uint32_t outer_cnt;

        if ((samples_per_buf % c) != 0U) {
            continue;
        }
        cycles = c / active_ch;
        if (cycles <= ADC_DMA_LOOP_MAX) {
            return c;
        }
        /* cycles > LOOP_MAX here, so outer_cnt=1 always fails; start at 2. */
        for (outer_cnt = 2U; outer_cnt <= ADC_DMA_LOOP_MAX; outer_cnt++) {
            if (((cycles % outer_cnt) == 0U) &&
                ((cycles / outer_cnt) <= ADC_DMA_LOOP_MAX)) {
                return c;
            }
        }
    }
    return active_ch;
}
#else
static uint32_t ADC_DMA_DeriveChunkSize(ADC_RESOURCES *ADC,
                                        uint32_t active_ch,
                                        uint32_t samples_per_buf)
{
    uint32_t block  = ADC_DMA_SAMPLE_BLOCK_BYTES;
    uint32_t budget = (ADC->mcode_size - ADC_DMA_CONST_TABLE_BYTES
                                       - ADC_DMA_FOOTER_BYTES) / block;
    uint32_t c;

    /* Largest chunk (samples) that (a) fits the mcode budget, (b) divides
     * samples_per_buf so no short final chunk is needed, and (c) is a whole
     * multiple of active_ch so channel-cycling in the unrolled mcode restarts
     * cleanly on each chunk. active_ch itself is always a valid fallback.
     */
    for (c = budget; c >= active_ch; c--) {
        if (((c % active_ch) == 0U) && ((samples_per_buf % c) == 0U)) {
            return c;
        }
    }
    return active_ch;
}
#endif

/*
 * @func         : void ADC_DMA_DrainChunk(ADC_RESOURCES *ADC, uint32_t n_samples)
 * @brief        : Absorb the current chunk's samples captured at
 *                 ADC_Stop() or DMA abort.
 * @parameter[1] : ADC       : pointer to ADC_RESOURCES structure
 * @parameter[2] : n_samples : sample count to absorb from this chunk
 * @return       : none
 */
static void ADC_DMA_DrainChunk(ADC_RESOURCES *ADC, uint32_t n_samples)
{
    conv_info_t *conv = &ADC->conv;
    uint32_t    *chunk_base = conv->active_buf +
                              ADC->dma_state.chunk_idx * ADC->dma_state.chunk_samples;

    RTSS_InvalidateDCache_by_Addr((volatile void *) chunk_base,
                                  n_samples * 4U);
    conv->buffer_idx += n_samples;
}

/*
 * @func         : void ADC_DMA_SnapshotAndStop(ADC_RESOURCES *ADC)
 * @brief        : Halt the DMA channel and fold the partly-filled chunk's
 *                 sample count into buffer_idx.
 * @parameter[1] : ADC : pointer to ADC_RESOURCES structure
 * @return       : none
 */
static void ADC_DMA_SnapshotAndStop(ADC_RESOURCES *ADC)
{
    uint32_t bytes_done = 0U;
    uint32_t written_samples;

    (void) ADC_DMA_Stop(&ADC->dma_cfg->dma_rx);
    ADC->dma_state.armed = 0U;

    (void) ADC->dma_cfg->dma_rx.dma_drv->GetStatus(
        &ADC->dma_cfg->dma_rx.dma_handle, &bytes_done);

    written_samples = bytes_done / 4U;
    if (written_samples > ADC->dma_state.chunk_samples) {
        written_samples = ADC->dma_state.chunk_samples;
    }

    if (written_samples > 0U) {
        ADC_DMA_DrainChunk(ADC, written_samples);
    }
}

/*
 * @func         : uint32_t ADC_DMA_BuildMcode(ADC_RESOURCES *ADC, void *dst_buf)
 * @brief        : Build the per-chunk DMA mcode program. The legacy form is
 *                 fully unrolled with a per-sample intr-clear step; the slim
 *                 form (compiled when SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT is
 *                 set) is a fixed-size header + nested DMALP body + footer.
 * @parameter[1] : ADC     : pointer to ADC_RESOURCES structure
 * @parameter[2] : dst_buf : first chunk's destination base (buf_a)
 * @return       : instruction bytes emitted, or 0 on overflow / invalid inputs
 */
#if SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT
static uint32_t ADC_DMA_BuildMcode(ADC_RESOURCES *ADC, void *dst_buf)
{
    dma_opcode_buf op_buf;
    dma_ccr_t      ccr_sample;
    dma_loop_t     lp_lc0, lp_lc1;
    uint8_t        periph_num = ADC->dma_cfg->dma_rx.dma_periph_req;
    uint8_t        dma_handle = (uint8_t) ADC->dma_cfg->dma_rx.dma_handle;
    uint32_t       active     = ADC->active_channels;
    uint32_t       ch_list[9];
    uint32_t       num_ch = 0U;
    uint32_t       work, i;
    uint32_t       n = ADC->dma_state.chunk_samples;
    uint32_t       cycles;
    uint32_t       outer_cnt = 1U;
    uint32_t       inner_cnt;
    uint32_t       lc0_body_start, lc1_body_start;
    uintptr_t      dar_base;

    /* Ordered list of unmasked channel indices (LSB first). */
    work = active;
    while (work) {
        ch_list[num_ch++] = (uint32_t) __builtin_ctz(work);
        work             &= work - 1U;
    }
    if (num_ch == 0U) {
        return 0U;
    }
    if ((n == 0U) || ((n % num_ch) != 0U)) {
        return 0U;
    }

    /* Factor cycles into outer*inner with both in [1, ADC_DMA_LOOP_MAX].
     * DeriveChunkSize already picked n so a valid factoring exists in the
     * common case; bail cleanly if we get an unfactorable n.
     */
    cycles    = n / num_ch;
    inner_cnt = cycles;
    if (cycles > ADC_DMA_LOOP_MAX) {
        inner_cnt = 0U;
        /* cycles > LOOP_MAX here, so outer_cnt=1 always fails; start at 2. */
        for (outer_cnt = 2U; outer_cnt <= ADC_DMA_LOOP_MAX; outer_cnt++) {
            if (((cycles % outer_cnt) == 0U) &&
                ((cycles / outer_cnt) <= ADC_DMA_LOOP_MAX)) {
                inner_cnt = cycles / outer_cnt;
                break;
            }
        }
        if (inner_cnt == 0U) {
            return 0U;
        }
    }

    op_buf.buf      = ADC->dma_mcode;
    op_buf.buf_size = ADC->mcode_size;
    op_buf.off      = 0U;

    /* Sample-read CCR: 4 B in, 4 B out per store */
    ccr_sample.value                    = 0U;
    ccr_sample.value_b.dst_burst_len    = 0U;
    ccr_sample.value_b.src_burst_len    = 0U;
    ccr_sample.value_b.dst_burst_size   = BS_BYTE_4;
    ccr_sample.value_b.src_burst_size   = BS_BYTE_4;
    ccr_sample.value_b.dst_cache_ctrl   = DMA_DEST_CACHE_CTRL;
    ccr_sample.value_b.src_cache_ctrl   = 0U;
    ccr_sample.value_b.dst_inc          = DMA_BURST_INCREMENTING;
    ccr_sample.value_b.src_inc          = DMA_BURST_FIXED;
    ccr_sample.value_b.dst_prot_ctrl    = DMA_DEST_PROT_CTRL;
    ccr_sample.value_b.src_prot_ctrl    = DMA_SRC_PROT_CTRL;
    ccr_sample.value_b.endian_swap_size = 0U;

    if (!dma_construct_move(ccr_sample.value, DMA_REG_CCR, &op_buf)) {
        return 0U;
    }
    dar_base = LocalToGlobal(dst_buf);
    if (!dma_construct_move(dar_base, DMA_REG_DAR, &op_buf)) {
        return 0U;
    }

    if (!dma_construct_loop(DMA_LC_1, (uint8_t) outer_cnt, &op_buf)) {
        return 0U;
    }
    lc1_body_start = op_buf.off;

    if (!dma_construct_loop(DMA_LC_0, (uint8_t) inner_cnt, &op_buf)) {
        return 0U;
    }
    lc0_body_start = op_buf.off;

    for (i = 0U; i < num_ch; i++) {
        uintptr_t sar_ch = LocalToGlobal(
            (void *) &ADC->regs->ADC_SAMPLE_REG_[ch_list[i]]);

        if (!dma_construct_move(sar_ch, DMA_REG_SAR, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_flushperiph(periph_num, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_wfp(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_loadperiph(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_store(DMA_XFER_SINGLE, &op_buf)) {
            return 0U;
        }
    }

    lp_lc0.lc        = DMA_LC_0;
    lp_lc0.xfer_type = DMA_XFER_SINGLE;
    lp_lc0.nf        = true;
    lp_lc0.jump      = (uint8_t) (op_buf.off - lc0_body_start);
    if (!dma_construct_loopend(&lp_lc0, &op_buf)) {
        return 0U;
    }

    lp_lc1.lc        = DMA_LC_1;
    lp_lc1.xfer_type = DMA_XFER_SINGLE;
    lp_lc1.nf        = true;
    lp_lc1.jump      = (uint8_t) (op_buf.off - lc1_body_start);
    if (!dma_construct_loopend(&lp_lc1, &op_buf)) {
        return 0U;
    }

    if (!dma_construct_wmb(&op_buf)) {
        return 0U;
    }
    if (!dma_construct_send_event(dma_handle, &op_buf)) {
        return 0U;
    }
    if (!dma_construct_end(&op_buf)) {
        return 0U;
    }

    RTSS_CleanDCache_by_Addr(ADC->dma_mcode, ADC->mcode_size);
    return op_buf.off;
}
#else
static uint32_t ADC_DMA_BuildMcode(ADC_RESOURCES *ADC, void *dst_buf)
{
    dma_opcode_buf op_buf;
    dma_ccr_t      ccr_sample, ccr_intrclr;
    uint8_t        periph_num = ADC->dma_cfg->dma_rx.dma_periph_req;
    uint8_t        dma_handle = (uint8_t) ADC->dma_cfg->dma_rx.dma_handle;
    uint32_t       active     = ADC->active_channels;
    uintptr_t      adc_intr_addr;
    uintptr_t      adc_clr_src;
    uint32_t       ch_list[9];
    uint32_t       num_ch = 0U;
    uint32_t       work, i;
    uint32_t       n = ADC->dma_state.chunk_samples;
    uint32_t       block  = ADC_DMA_SAMPLE_BLOCK_BYTES;
    uint32_t       budget = (ADC->mcode_size - ADC_DMA_CONST_TABLE_BYTES
                                             - ADC_DMA_FOOTER_BYTES) / block;

    if ((n == 0U) || (n > budget)) {
        return 0U;
    }

    /* Ordered list of unmasked channel indices (LSB first). */
    work = active;
    while (work) {
        ch_list[num_ch++] = (uint32_t) __builtin_ctz(work);
        work             &= work - 1U;
    }
    if (num_ch == 0U) {
        return 0U;
    }

    /* Embed the 32-bit ADC_INTR_DONE0_CLEAR constant at mcode[0..3]. The
     * per-sample intr-clear step loads it via DMAMOV SAR, then a single
     * DMASTP writes it to ADC_INTERRUPT (W1C for DONE0 only).
     */
    *((uint32_t *) ADC->dma_mcode) = ADC_INTR_DONE0_CLEAR;
    adc_clr_src                    = LocalToGlobal(ADC->dma_mcode);

    op_buf.buf      = ADC->dma_mcode + ADC_DMA_CONST_TABLE_BYTES;
    op_buf.buf_size = ADC->mcode_size - ADC_DMA_CONST_TABLE_BYTES;
    op_buf.off      = 0U;

    /* Sample-read CCR: 4 B in, 1×4 B out (dst_bs=4, dst_inc=fixed). Each
     * ADC_SAMPLE_REG_ read lands as one uint32_t into buffer[i].
     */
    ccr_sample.value                    = 0U;
    ccr_sample.value_b.dst_burst_len    = 0U;
    ccr_sample.value_b.src_burst_len    = 0U;
    ccr_sample.value_b.dst_burst_size   = BS_BYTE_4;
    ccr_sample.value_b.src_burst_size   = BS_BYTE_4;
    ccr_sample.value_b.dst_cache_ctrl   = DMA_DEST_CACHE_CTRL;
    ccr_sample.value_b.src_cache_ctrl   = 0U;
    ccr_sample.value_b.dst_inc          = DMA_BURST_FIXED;
    ccr_sample.value_b.src_inc          = DMA_BURST_FIXED;
    ccr_sample.value_b.dst_prot_ctrl    = DMA_DEST_PROT_CTRL;
    ccr_sample.value_b.src_prot_ctrl    = DMA_SRC_PROT_CTRL;
    ccr_sample.value_b.endian_swap_size = 0U;

    /* Intr-clear CCR */
    ccr_intrclr.value                    = 0U;
    ccr_intrclr.value_b.dst_burst_len    = 0U;
    ccr_intrclr.value_b.src_burst_len    = 0U;
    ccr_intrclr.value_b.dst_burst_size   = BS_BYTE_4;
    ccr_intrclr.value_b.src_burst_size   = BS_BYTE_4;
    ccr_intrclr.value_b.dst_cache_ctrl   = 0U;
    ccr_intrclr.value_b.src_cache_ctrl   = DMA_SRC_CACHE_CTRL;
    ccr_intrclr.value_b.dst_inc          = DMA_BURST_FIXED;
    ccr_intrclr.value_b.src_inc          = DMA_BURST_FIXED;
    ccr_intrclr.value_b.dst_prot_ctrl    = DMA_DEST_PROT_CTRL;
    ccr_intrclr.value_b.src_prot_ctrl    = DMA_SRC_PROT_CTRL;
    ccr_intrclr.value_b.endian_swap_size = 0U;

    adc_intr_addr = LocalToGlobal((void *) &ADC->regs->ADC_INTERRUPT);

    for (i = 0U; i < n; i++) {
        uint32_t  ch      = ch_list[i % num_ch];
        uintptr_t dar_i   = LocalToGlobal((uint8_t *) dst_buf + i * 4U);
        uintptr_t sar_ch  = LocalToGlobal((void *) &ADC->regs->ADC_SAMPLE_REG_[ch]);

        if (!dma_construct_flushperiph(periph_num, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_wfp(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_move(ccr_intrclr.value, DMA_REG_CCR, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_move(adc_clr_src, DMA_REG_SAR, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_move(adc_intr_addr, DMA_REG_DAR, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_load(DMA_XFER_SINGLE, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_storeperiph(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return 0U;
        }

        if (!dma_construct_move(ccr_sample.value, DMA_REG_CCR, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_move(sar_ch, DMA_REG_SAR, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_move(dar_i, DMA_REG_DAR, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_loadperiph(DMA_XFER_SINGLE, periph_num, &op_buf)) {
            return 0U;
        }
        if (!dma_construct_store(DMA_XFER_SINGLE, &op_buf)) {
            return 0U;
        }
    }

    if (!dma_construct_wmb(&op_buf)) {
        return 0U;
    }
    if (!dma_construct_send_event(dma_handle, &op_buf)) {
        return 0U;
    }
    if (!dma_construct_end(&op_buf)) {
        return 0U;
    }

    RTSS_CleanDCache_by_Addr(ADC->dma_mcode, ADC->mcode_size);
    return op_buf.off;
}
#endif

/*
 * @func         : void ADC_DMA_PatchDars(ADC_RESOURCES *ADC, void *chunk_base)
 * @brief        : Patch DAR immediates in the built mcode to point at
 *                 chunk_base. Legacy path patches one imm per sample; slim
 *                 path patches only the one header DAR imm.
 * @parameter[1] : ADC        : pointer to ADC_RESOURCES structure
 * @parameter[2] : chunk_base : destination base for the next chunk
 * @return       : none
 */
#if SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT
static void ADC_DMA_PatchDars(ADC_RESOURCES *ADC, void *chunk_base)
{
    uint8_t  *mcode = ADC->dma_mcode;
    uintptr_t base  = LocalToGlobal(chunk_base);

    mcode[ADC_DMA_HEADER_DAR_OFFSET + 0U] = (uint8_t) (base);
    mcode[ADC_DMA_HEADER_DAR_OFFSET + 1U] = (uint8_t) (base >> 8);
    mcode[ADC_DMA_HEADER_DAR_OFFSET + 2U] = (uint8_t) (base >> 16);
    mcode[ADC_DMA_HEADER_DAR_OFFSET + 3U] = (uint8_t) (base >> 24);
    RTSS_CleanDCache_by_Addr(ADC->dma_mcode, ADC->mcode_size);
}
#else
static void ADC_DMA_PatchDars(ADC_RESOURCES *ADC, void *chunk_base)
{
    uint32_t  n = ADC->dma_state.chunk_samples;
    uint32_t  i;
    uint8_t  *mcode = ADC->dma_mcode;
    uintptr_t base;

    for (i = 0U; i < n; i++) {
        uint32_t off = ADC_DMA_CONST_TABLE_BYTES + i * ADC_DMA_SAMPLE_BLOCK_BYTES +
                       ADC_DMA_DAR_OFFSET_IN_BLOCK;
        base = LocalToGlobal((uint8_t *) chunk_base + i * 4U);
        mcode[off + 0U] = (uint8_t) (base);
        mcode[off + 1U] = (uint8_t) (base >> 8);
        mcode[off + 2U] = (uint8_t) (base >> 16);
        mcode[off + 3U] = (uint8_t) (base >> 24);
    }
    RTSS_CleanDCache_by_Addr(ADC->dma_mcode, ADC->mcode_size);
}
#endif

/*
 * @func         : void ADC_DMA_FillParams(ADC_RESOURCES *ADC, ARM_DMA_PARAMS *p,
 *                                         void *dst_buf)
 * @brief        : Populate ARM_DMA_PARAMS for a DMA restart on this chunk.
 * @parameter[1] : ADC     : pointer to ADC_RESOURCES structure
 * @parameter[2] : p       : ARM_DMA_PARAMS to fill
 * @parameter[3] : dst_buf : chunk destination base
 * @return       : none
 */
__STATIC_INLINE void ADC_DMA_FillParams(ADC_RESOURCES *ADC, ARM_DMA_PARAMS *p,
                                        void *dst_buf)
{
    p->peri_reqno   = (int8_t) ADC->dma_cfg->dma_rx.dma_periph_req;
    p->burst_len    = 1U;
    p->burst_size   = BS_BYTE_4;
    p->dir          = ARM_DMA_DEV_TO_MEM;
    p->src_addr     = (volatile const void *) &ADC->regs->ADC_SAMPLE_REG_[0];
    p->dst_addr     = (volatile void *) dst_buf;
    /* num_bytes is derived from chunk_samples (one 4 B write per sample). */
    p->num_bytes    = ADC->dma_state.chunk_samples * 4U;
    p->irq_priority = ADC->dma_irq_priority;
    p->cb_event     = ADC->dma_cb;
}

/*
 * @func         : int32_t ADC_DMA_ArmChunk(ADC_RESOURCES *ADC, void *dst_buf,
 *                                          uint32_t invalidate_bytes)
 * @brief        : Arm the DMA channel to fill the next chunk at dst_buf.
 * @parameter[1] : ADC              : pointer to ADC_RESOURCES structure
 * @parameter[2] : dst_buf          : destination base for the next chunk
 * @parameter[3] : invalidate_bytes : dcache invalidate range in bytes
 * @return       : ARM_DRIVER_OK    : channel armed; sequencer will resume
 *                 ARM_DRIVER_ERROR : Usermcode or Start reported failure;
 *                                    caller must tear the capture down
 */
__STATIC_INLINE int32_t ADC_DMA_ArmChunk(ADC_RESOURCES *ADC, void *dst_buf,
                                         uint32_t invalidate_bytes)
{
    ARM_DMA_PARAMS params;

    ADC_DMA_PatchDars(ADC, dst_buf);
    RTSS_InvalidateDCache_by_Addr(dst_buf, invalidate_bytes);
    ADC_DMA_FillParams(ADC, &params, dst_buf);
    if (ADC_DMA_Usermcode(&ADC->dma_cfg->dma_rx,
                          LocalToGlobal(ADC->dma_mcode +
                                        ADC_DMA_CONST_TABLE_BYTES)) != ARM_DRIVER_OK) {
        return ARM_DRIVER_ERROR;
    }
    if (ADC_DMA_Start(&ADC->dma_cfg->dma_rx, &params) != ARM_DRIVER_OK) {
        return ARM_DRIVER_ERROR;
    }
    ADC->dma_state.armed = 1U;
    return ARM_DRIVER_OK;
}

/*
 * @func         : void ADC_DMA_TeardownAndNotifyStopped(ADC_RESOURCES *ADC)
 * @brief        : Clear conv/StartN state and emit STOPPED after an
 *                 unrecoverable DMA/arm failure.
 * @parameter    : ADC : pointer to ADC_RESOURCES structure
 * @return       : none
 */
static void ADC_DMA_TeardownAndNotifyStopped(ADC_RESOURCES *ADC)
{
    conv_info_t *conv    = &ADC->conv;
    uint32_t     partial = conv->buffer_idx;

    adc_mask_interrupt(ADC->regs);
    adc_disable_continuous_conv(ADC->regs);
    ADC->dma_state.armed  = 0U;

    conv->buf_a           = NULL;
    conv->buf_b           = NULL;
    conv->active_buf      = NULL;
    conv->samples_per_buf = 0U;
    conv->buffer_idx      = 0U;
    conv->active_buf_idx  = 0U;
    ADC->busy             = 0U;

    if (ADC->cb_event) {
        ADC->cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_STOPPED, 0U, partial);
    }
}

/*
 * @func         : void ADCx_DMACallback(uint32_t event, int8_t peri_num,
 *                                       ADC_RESOURCES *ADC)
 * @brief        : Shared DMA-complete dispatcher for all ADC instances.
 *                 Called by each per-instance ADCxxx_DMACallback shim.
 * @parameter[1] : event    : DMA event mask from the DMA driver
 * @parameter[2] : peri_num : DMA peripheral number (unused)
 * @parameter[3] : ADC      : pointer to ADC_RESOURCES structure
 * @return       : none
 */
static void ADCx_DMACallback(uint32_t event, int8_t peri_num, ADC_RESOURCES *ADC)
{
    conv_info_t   *conv = &ADC->conv;
    void          *next_base;
    uint32_t       event_id;
    bool           is_mid_buffer;

    (void) peri_num;

    if (conv->active_buf == NULL) {
        /* Stopped or one-shot completed — drop stale events. */
        return;
    }

    /* DMA aborted. Snapshot the partial chunk (so its samples reach
     * buffer_idx), then tear the capture down and fire STOPPED with the
     * total sample count — same shape as an app-initiated Stop().
     */
    if (event & ARM_DMA_EVENT_ABORT) {
        ADC_DMA_SnapshotAndStop(ADC);
        ADC_DMA_TeardownAndNotifyStopped(ADC);
        return;
    }

    /* Transfer just completed; channel is idle until we re-arm. */
    ADC->dma_state.armed = 0U;

    is_mid_buffer = (ADC->dma_state.chunk_idx + 1U) < ADC->dma_state.chunks_per_buf;

    /* Mid-buffer: re-arm DMA on the NEXT chunk BEFORE draining the just-
     * finished one. The next DMA target region is disjoint from the drain
     * range, so the two operations do not race, and this lets DMA capture
     * samples during the cache-invalidate instead of waiting for it.
     */
    if (is_mid_buffer) {
        next_base = (uint8_t *) conv->active_buf +
                    (ADC->dma_state.chunk_idx + 1U) * ADC->dma_state.chunk_samples * 4U;
        if (ADC_DMA_ArmChunk(ADC, next_base,
                             ADC->dma_state.chunk_samples * 4U) != ARM_DRIVER_OK) {
            /* Re-arm failed; drain the just-finished chunk so its
             * samples are counted, then emit STOPPED.
             */
            ADC_DMA_DrainChunk(ADC, ADC->dma_state.chunk_samples);
            ADC->dma_state.chunk_idx++;
            ADC_DMA_TeardownAndNotifyStopped(ADC);
            return;
        }
    }

    /* Absorb the just-finished chunk */
    ADC_DMA_DrainChunk(ADC, ADC->dma_state.chunk_samples);
    ADC->dma_state.chunk_idx++;

    if (is_mid_buffer) {
        return;
    }

    /* Buffer full. Fire event id from the buffer we just finished. */
    event_id = (conv->active_buf_idx == 0U)
                 ? ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_A
                 : ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_B;

    if (conv->buf_b == NULL) {
        /* One-shot: halt the sequencer and DMA; app must call StartN to re-arm. */
        adc_mask_interrupt(ADC->regs);
        adc_disable_continuous_conv(ADC->regs);
        (void) ADC_DMA_Stop(&ADC->dma_cfg->dma_rx);
        conv->buf_a      = NULL;
        conv->active_buf = NULL;
        ADC->busy        = 0U;
        if (ADC->cb_event) {
            ADC->cb_event(event_id, 0U, conv->samples_per_buf);
        }
        return;
    }

    /* Ping-pong flip: point at the other buffer, reset chunk counters,
     * fire the just-filled buffer's event, then re-arm DMA. Callback fires
     * BEFORE re-arm so the app can call Stop() from the callback and get a
     * clean boundary (both buffers pristine). If Stop() runs, it clears
     * active_buf; we detect that and skip the re-arm.
     */
    conv->active_buf_idx ^= 1U;
    conv->active_buf     = (conv->active_buf_idx == 0U) ? conv->buf_a : conv->buf_b;
    ADC->dma_state.chunk_idx      = 0U;
    conv->buffer_idx     = 0U;

    if (ADC->cb_event) {
        ADC->cb_event(event_id, 0U, conv->samples_per_buf);
    }

    if (conv->active_buf == NULL) {
        /* Callback called Stop(); nothing to re-arm. */
        return;
    }

    if (ADC_DMA_ArmChunk(ADC, conv->active_buf,
                         conv->samples_per_buf * 4U) != ARM_DRIVER_OK) {
        ADC_DMA_TeardownAndNotifyStopped(ADC);
    }
}
#endif /* ADC_DMA_ENABLE */

/*
 * @func      : void Analog_config()
 * @brief     : vbat comparator value and register configuration
 * @parameter : NONE
 * @return    : NONE
 */
static void Analog_Config(void)
{
    /* Analog configuration Vbat register2 */
    enable_analog_peripherals();

    /* Enables ADC voltage reference and internal buffer for ADC operation */
    enable_adc_ref_voltage();
}

/*
 * @func           : int32_t ADC_Initialize(ADC_RESOURCES *ADC, ARM_ADC_SignalEvent_t cb_event)
 * @brief          : initialize the device
 * @parameter[1]   : adc      : Pointer to /ref ADC_RESOURCES structure
 * @parameter[2]   : cb_event : Pointer to /ref ARM_ADC_Signal_Event_t cb_event
 * @return         : ARM_DRIVER_OK              : if driver initialized successfully
 *                 : ARM_DRIVER_ERROR_PARAMETER : if parameter is invalid or not
 */
static int32_t ADC_Initialize(ADC_RESOURCES *ADC, ARM_ADC_SignalEvent_t cb_event)
{
    int ret = ARM_DRIVER_OK;

    if (!cb_event) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* User call back Event */
    ADC->cb_event  = cb_event;

    /* active_channels default mirrors the hardware reset state. */
    if (ADC->drv_instance == ADC_INSTANCE_ADC24_0) {
        ADC->active_channels = (1UL << ADC24_MAX_DIFFERENTIAL_CHANNEL) - 1UL;
    } else {
        ADC->active_channels = 0xFFUL;
    }

    /* No StartN buffer armed yet. */
    ADC->conv.buf_a           = NULL;
    ADC->conv.buf_b           = NULL;
    ADC->conv.active_buf      = NULL;
    ADC->conv.samples_per_buf = 0U;
    ADC->conv.buffer_idx      = 0U;
    ADC->conv.active_buf_idx  = 0U;

#if ADC_DMA_ENABLE
    if (ADC->dma_enable) {
        if (ADC_DMA_Initialize(&ADC->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
            return ARM_DRIVER_ERROR;
        }
    }
#endif

    /* Setting flag to initialize */
    ADC->state    |= ADC_FLAG_DRV_INIT_DONE;

    return ret;
}

/*
 * @func           : int32_t ADC_Uninitialize (ARM_ADC_SignalEvent_t cb_event)
 * @brief          : Uninitialize the device
 * @parameter[in]  : ADC    : Pointer to the structure ADC_RESOURCES
 * @return         : ARM_DRIVER_OK              : if adc is successfully initialized
 *                 : ARM_DRIVER_ERROR_PARAMETER : if adc device is invalid
 */
static int32_t ADC_Uninitialize(ADC_RESOURCES *ADC)
{
    int ret = ARM_DRIVER_OK;

    /* parameter checking */
    if (!ADC) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Checking initialized has done or not */
    if (!(ADC->state & ADC_FLAG_DRV_INIT_DONE)) {
        return ARM_DRIVER_OK;
    }

    /* set call back to NULL */
    ADC->cb_event          = NULL;

    /* Reset last read channel */
    ADC->conv.read_channel = 0;

    /* Drop any StartN buffer registration */
    ADC->conv.buf_a           = NULL;
    ADC->conv.buf_b           = NULL;
    ADC->conv.active_buf      = NULL;
    ADC->conv.samples_per_buf = 0U;
    ADC->conv.buffer_idx      = 0U;
    ADC->conv.active_buf_idx  = 0U;

#if ADC_DMA_ENABLE
    if (ADC->dma_enable) {
        (void) ADC_DMA_Uninitialize(&ADC->dma_cfg->dma_rx);
    }
#endif

    /* flags */
    ADC->state             = 0;

    return ret;
}

/*
 * @func         : int32_t ADC_PowerControl(ARM_POWER_status status, ADC_RESOURCES *adc)
 * @brief        : power the driver and enable NVIC
 * @parameter[1] : ADC              : pointer to /ref ADC_RESOURCES
 * @parameter[2] : state            : power state
 * @return       : ARM_DRIVER_OK    : if power done successful
 *                 ARM_DRIVER_ERROR : if initialize is not done
 */
static int32_t ADC_PowerControl(ADC_RESOURCES *ADC, ARM_POWER_STATE state)
{
    int32_t ret = ARM_DRIVER_OK;

    switch (state) {
    case ARM_POWER_FULL:

        if (!(ADC->state & ADC_FLAG_DRV_INIT_DONE)) {
            return ARM_DRIVER_ERROR;
        }

        if ((ADC->state & ADC_FLAG_DRV_POWER_DONE)) {
            return ARM_DRIVER_OK;
        }

        /* Clear Any Pending IRQ */
        NVIC_ClearPendingIRQ(ADC->intr_done0_irq_num);
        NVIC_ClearPendingIRQ(ADC->intr_done1_irq_num);
        NVIC_ClearPendingIRQ(ADC->intr_cmpa_irq_num);
        NVIC_ClearPendingIRQ(ADC->intr_cmpb_irq_num);

        /* Set priority */
        NVIC_SetPriority(ADC->intr_done0_irq_num, ADC->intr_done0_irq_priority);
        NVIC_SetPriority(ADC->intr_done1_irq_num, ADC->intr_done1_irq_priority);
        NVIC_SetPriority(ADC->intr_cmpa_irq_num, ADC->intr_cmpa_irq_priority);
        NVIC_SetPriority(ADC->intr_cmpb_irq_num, ADC->intr_cmpb_irq_priority);

        /* Enable the NIVC */
        NVIC_EnableIRQ(ADC->intr_done0_irq_num);
        NVIC_EnableIRQ(ADC->intr_done1_irq_num);
        NVIC_EnableIRQ(ADC->intr_cmpa_irq_num);
        NVIC_EnableIRQ(ADC->intr_cmpb_irq_num);
        ;

        /* adc clock enable */
        adc_set_clk_control(ADC->drv_instance, true);

        /* Enabling comparator clock */
        enable_analog_periph_clk();

        /*function include vbat and comparator address and it value */
        Analog_Config();

        /* set differential control for ADC12 */
        adc_set_differential_ctrl(ADC->drv_instance, ADC->differential_enable);

        adc_set_comparator_ctrl(ADC->drv_instance, ADC->comparator_enable, ADC->comparator_bias);

        if (ADC->differential_enable == ADC_DIFFERENTIAL_ENABLE ||
            (ADC->drv_instance == ADC_INSTANCE_ADC24_0)) {
            /* check adc instances pga enabled */
            if (ADC->pga_enable) {
                /* set pga gain */
                enable_adc_pga_gain(ADC->drv_instance, ADC->pga_value);
            }
        }

        if (ADC->drv_instance == ADC_INSTANCE_ADC24_0) {
            /* enable adc24 from control register */
            enable_adc24(ADC->drv_instance);

            /* set output rate from control register */
            set_adc24_output_rate(ADC->drv_instance, ADC->output_rate);

            /* Set adc24 bias from control register */
            set_adc24_bias(ADC->drv_instance, ADC->bias);

            /* Enabling continuous sampling */
            adc24_enable_continous_sample(ADC->regs);
        } else {
            /* set Sample width value for ADC12 */
            adc_set_sample_width(ADC->regs, ADC->sample_width);
        }

        /* set user channel input */
        adc_init_channel_select(ADC->regs, ADC->conv.user_input);

        /* set the clock divisor */
        adc_set_clk_div(ADC->regs, ADC->clock_div);

        /* set avg sample value */
        adc_set_avg_sample(ADC->regs, ADC->avg_sample_num);

        /* set number of n shift bits */
        adc_set_n_shift_bit(ADC->regs, ADC->shift_n_bit, ADC->shift_left_or_right);

        /* set sequencer control to single channel scan */
        adc_set_single_ch_scan_mode(ADC->regs, &ADC->conv);

        /* Disable the interrupt (mask the interrupt(0xF))*/
        adc_mask_interrupt(ADC->regs);

        /* Set the power flag enabled */
        ADC->state |= ADC_FLAG_DRV_POWER_DONE;

        break;

    case ARM_POWER_OFF:

        /* Disable ADC NVIC */
        NVIC_DisableIRQ(ADC->intr_done0_irq_num);
        NVIC_DisableIRQ(ADC->intr_done1_irq_num);
        NVIC_DisableIRQ(ADC->intr_cmpa_irq_num);
        NVIC_DisableIRQ(ADC->intr_cmpb_irq_num);

        /* Clear Any Pending IRQ */
        NVIC_ClearPendingIRQ(ADC->intr_done0_irq_num);
        NVIC_ClearPendingIRQ(ADC->intr_done1_irq_num);
        NVIC_ClearPendingIRQ(ADC->intr_cmpa_irq_num);
        NVIC_ClearPendingIRQ(ADC->intr_cmpb_irq_num);

        /* set the clock divisor */
        adc_set_clk_div(ADC->regs, ADC_CLOCK_DIV_MIN_VALUE);

        /* set avg sample value */
        adc_set_avg_sample(ADC->regs, ADC_AVG_SAMPLES_FOR_AVG_MIN);

        /* set Sample width value */
        adc_set_sample_width(ADC->regs, ADC_SAMPLE_WIDTH_MIN_VALUE);

        /* set number of n shift bits */
        adc_set_n_shift_bit(ADC->regs, 0, 0);

        /* Disable the interrupt (mask the interrupt(0xF)) */
        adc_mask_interrupt(ADC->regs);

        if (ADC->differential_enable == ADC_DIFFERENTIAL_ENABLE ||
            (ADC->drv_instance == ADC_INSTANCE_ADC24_0)) {
            /* check adc instances pga enabled */
            if (ADC->pga_value) {
                /* Disable pga gain */
                disable_adc_pga_gain(ADC->drv_instance);
            }
        }

        if (ADC->drv_instance == ADC_INSTANCE_ADC24_0) {
            /* disable adc24 from control register */
            disable_adc24(ADC->drv_instance);

            /* set output rate from control register */
            set_adc24_output_rate(ADC->drv_instance, 0U);
        }

        disable_analog_peripherals();

        disable_adc_ref_voltage();

        /* Disabling CMP clock */
        disable_analog_periph_clk();

        /* adc clock disable */
        adc_set_clk_control(ADC->drv_instance, false);

        /* Reset the power status of ADC */
        ADC->state &= ~ADC_FLAG_DRV_POWER_DONE;

        break;

    case ARM_POWER_LOW:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    }

#if ADC_DMA_ENABLE
    if (ADC->dma_enable) {
        if (ADC_DMA_PowerControl(state, &ADC->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
            return ARM_DRIVER_ERROR;
        }
    }
#endif

    return ret;
}

/*
 * @func        : int32_t ADC_Start( ADC_RESOURCES *ADC)
 * @brief       : Start the adc and initialize interrupt
 * @parameter   : ADC  : pointer to ADC_RESOURCES structure
 * @return      : ARM_DRIVER_OK              : if the function are return successful
 *                ARM_DRIVER_ERROR_PARAMETER : if parameter are invalid
 */
static int32_t ADC_Start(ADC_RESOURCES *ADC)
{
    /* Check Power done or not */
    if (!(ADC->state & ADC_FLAG_DRV_POWER_DONE)) {
        return ARM_DRIVER_ERROR;
    }

    if (ADC->busy == 1U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* setup conversion status */
    ADC->conv.status = ADC_CONV_STAT_NONE;

    /* active the conv busy flag */
    ADC->busy        = 1U;

    /* enable the interrupt(unmask the interrupt 0x0)*/
    adc_unmask_interrupt(ADC->regs);

    if (ADC->ext_trig_val) {
        /* Enable the trigger */
        adc_enable_external_trigger(ADC->regs, ADC->ext_trig_val);
    } else {
        /* Start the ADC conversion mode */
        if (ADC->conv.mode == ADC_CONV_MODE_SINGLE_SHOT) {
            /* Enable single shot conversion */
            adc_enable_single_shot_conv(ADC->regs);
        } else {
            /* Enable continuous conversion */
            adc_enable_continuous_conv(ADC->regs);
        }
    }

    return ARM_DRIVER_OK;
}

/*
 * @func      : int32_t ADC_Stop( ADC_RESOURCES *adc)
 * @brief     : Disable the adc
 * @parameter : ADC  : pointer to ADC_RESOURCES structure
 * @return    : ARM_DRIVER_OK : if function return successfully
 */
static int32_t ADC_Stop(ADC_RESOURCES *ADC)
{
    uint32_t partial;
    bool     was_capturing;

    /* Check Power done or not */
    if (!(ADC->state & ADC_FLAG_DRV_POWER_DONE)) {
        return ARM_DRIVER_ERROR;
    }

    /* Disable the interrupt(mask the interrupt 0xF)*/
    adc_mask_interrupt(ADC->regs);

    if (ADC->ext_trig_val) {
        /* Disable the trigger */
        adc_disable_external_trigger(ADC->regs, ADC->ext_trig_val);
    } else {
        /* Disable the adc */
        if (ADC->conv.mode == ADC_CONV_MODE_SINGLE_SHOT) {
            adc_disable_single_shot_conv(ADC->regs);
        } else {
            adc_disable_continuous_conv(ADC->regs);
        }
    }

#if ADC_DMA_ENABLE
    if (ADC->dma_enable && ADC->dma_state.armed &&
        (ADC->conv.active_buf != NULL)) {
        /* Snapshot how far the current chunk got before we stop it, and
         * fold the partial count into buffer_idx so the STOPPED callback
         * reports total written samples.
         */
        ADC_DMA_SnapshotAndStop(ADC);
    }
#if SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT
    if (ADC->dma_enable) {
        ADC->regs->ADC_CONTROL &= ~ADC_CONTROL_SAMPLE_INT_CLR;
    }
#endif
#endif

    /* Snapshot ping-pong state: if a StartN capture was in progress,
     * report the partial count and drop the buffer registration
     * so future stores are locked out.
     */
    was_capturing = (ADC->conv.active_buf != NULL) ||
                    (ADC->conv.buf_a != NULL);
    partial       = ADC->conv.buffer_idx;

    ADC->conv.buf_a           = NULL;
    ADC->conv.buf_b           = NULL;
    ADC->conv.active_buf      = NULL;
    ADC->conv.samples_per_buf = 0U;
    ADC->conv.buffer_idx      = 0U;
    ADC->conv.active_buf_idx  = 0U;
    ADC->busy                 = 0U;

    if (was_capturing && ADC->cb_event) {
        ADC->cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_STOPPED, 0U, partial);
    }

    return ARM_DRIVER_OK;
}

/*
 * @func         : int32_t ADC_StartN(ADC_RESOURCES *ADC,
 *                                    void *buf_a, void *buf_b,
 *                                    uint32_t samples_per_buf)
 * @brief        : Arm ping-pong buffers for continuous-mode
 *                 scan. The DONE0 ISR fills buf_a first; on the
 *                 buffer-fill boundary it flips to buf_b and the
 *                 per-instance handler fires
 *                 ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_A. Next boundary flips
 *                 back and fires BUF_B, and so on. The sequencer never
 *                 stops between halves.
 *                 If buf_b is NULL the call is one-shot: buf_a fills once,
 *                 BUF_A event fires.
 *
 *                 samples_per_buf must be a whole multiple of the number of
 *                 unmasked channels so every sweep fits exactly. Buffers
 *                 hold raw ADC_SAMPLE_REG_ values as uint32_t so both ADC12
 *                 and ADC24 instances share the same layout.
 *
 * @parameter[1] : ADC             : pointer to ADC_RESOURCES structure
 * @parameter[2] : buf_a           : caller-owned primary buffer, non-NULL
 * @parameter[3] : buf_b           : caller-owned secondary buffer for
 *                                   ping-pong, or NULL for one-shot
 * @parameter[4] : samples_per_buf : sample capacity of each buffer
 * @return       : ARM_DRIVER_OK              : buffer(s) armed
 *                 ARM_DRIVER_ERROR           : not powered
 *                 ARM_DRIVER_ERROR_PARAMETER : bad args, wrong mode,
 *                                              samples_per_buf not a whole
 *                                              multiple of enabled
 *                                              channels
 */
static int32_t ADC_StartN(ADC_RESOURCES *ADC,
                          void          *buf_a,
                          void          *buf_b,
                          uint32_t       samples_per_buf)
{
    uint32_t popcount = (uint32_t) __builtin_popcount(ADC->active_channels);

    if (!(ADC->state & ADC_FLAG_DRV_POWER_DONE)) {
        return ARM_DRIVER_ERROR;
    }

    if ((buf_a == NULL) || (samples_per_buf == 0U)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* StartN is only defined for continuous conversion */
    if (ADC->conv.mode != ADC_CONV_MODE_CONTINUOUS) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Buffer must end on a sweep boundary. */
    if ((popcount == 0U) || ((samples_per_buf % popcount) != 0U)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    ADC->conv.buf_a           = buf_a;
    ADC->conv.buf_b           = buf_b;
    ADC->conv.samples_per_buf = samples_per_buf;
    ADC->conv.buffer_idx      = 0U;
    ADC->conv.active_buf_idx  = 0U;
    ADC->conv.active_buf      = buf_a;

#if ADC_DMA_ENABLE
    if (ADC->dma_enable) {
        ARM_DMA_PARAMS params;
        uint32_t       chunk;

        /* Chunk size is derived — never fails: worst-case fallback is
         * active_ch itself, which the popcount check above guarantees is
         * a divisor of samples_per_buf.
         */
        chunk = ADC_DMA_DeriveChunkSize(ADC, popcount, samples_per_buf);
        ADC->dma_state.chunk_samples  = chunk;
        ADC->dma_state.chunks_per_buf = samples_per_buf / chunk;
        ADC->dma_state.chunk_idx      = 0U;

        RTSS_InvalidateDCache_by_Addr(buf_a, samples_per_buf * 4U);
        if (buf_b != NULL) {
            RTSS_InvalidateDCache_by_Addr(buf_b, samples_per_buf * 4U);
        }

#if SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT
        /* Enable auto-clear of DONE0 on ADC_SAMPLE_REG_ reads so the slim
         * DMA mcode can skip the per-sample intr-clear step.
         */
        ADC->regs->ADC_CONTROL |= ADC_CONTROL_SAMPLE_INT_CLR;
#endif

        if (ADC_DMA_BuildMcode(ADC, buf_a) == 0U) {
            ADC->conv.buf_a      = NULL;
            ADC->conv.buf_b      = NULL;
            ADC->conv.active_buf = NULL;
            return ARM_DRIVER_ERROR;
        }

        ADC_DMA_FillParams(ADC, &params, buf_a);

        if (ADC_DMA_Usermcode(&ADC->dma_cfg->dma_rx,
                              LocalToGlobal(ADC->dma_mcode +
                                            ADC_DMA_CONST_TABLE_BYTES))
              != ARM_DRIVER_OK) {
            ADC->conv.buf_a      = NULL;
            ADC->conv.buf_b      = NULL;
            ADC->conv.active_buf = NULL;
            return ARM_DRIVER_ERROR;
        }
        if (ADC_DMA_Start(&ADC->dma_cfg->dma_rx, &params) != ARM_DRIVER_OK) {
            ADC->conv.buf_a      = NULL;
            ADC->conv.buf_b      = NULL;
            ADC->conv.active_buf = NULL;
            return ARM_DRIVER_ERROR;
        }
        ADC->dma_state.armed = 1U;

        /* DMA services DONE0 exclusively - keep the CPU IRQ masked. */
        NVIC_DisableIRQ(ADC->intr_done0_irq_num);
    }
#endif

    /* Re-arm case: sequencer already running */
    if (ADC->busy) {
        return ARM_DRIVER_OK;
    }

    /* First-arm case: start the sequencer */
    ADC->conv.status = ADC_CONV_STAT_NONE;
    ADC->busy        = 1U;
    adc_unmask_interrupt(ADC->regs);
    if (ADC->ext_trig_val) {
        adc_enable_external_trigger(ADC->regs, ADC->ext_trig_val);
    } else {
        adc_enable_continuous_conv(ADC->regs);
    }

    return ARM_DRIVER_OK;
}

/*
 * @func      : uint32_t ADC_GetSampleCount(ADC_RESOURCES *ADC)
 * @brief     : Return the number of samples written into the caller's
 *              buffer since the last StartN. Snapshot value; safe to
 *              call at any time. Reset to 0 by the next StartN; preserved
 *              across Stop so the caller can read the final count.
 * @parameter : ADC : pointer to ADC_RESOURCES structure
 * @return    : samples-written count
 */
static uint32_t ADC_GetSampleCount(ADC_RESOURCES *ADC)
{
    return ADC->conv.buffer_idx;
}

/*
 * @func         : in32_t ADC_Control(uint32_t control , uint32_t arg, ADC_RESOURCES adc)
 * @brief        : control the following
 *                 - ARM_SET_SHIFT_CONTROL             : to control shift control of bits
 *                 - ARM_SET_SEQUENCER_CTRL            : selecting sample individual or rotate
 * through each unmasked sample
 *                 - ARM_ADC_SEQUENCER_MSK_CTRL        : to control masking of the channel
 *                 - ARM_ADC_CHANNEL_INIT_VAL          : to select initial channel for storing
 *                 - ARM_SET_ADC_COMPARATOR_A          : to set comparator a value
 *                 - ARM_SET_ADC_COMPARATOR_B          : to set comparator b value
 *                 - ARM_SET_ADC_THRESHOLD_COMPARISON  : to set the threshold comparison
 *                 - ARM_ADC_SET_CONVERSION_MODE       : to set conversion mode
 * @parameter[1] : ADC  : pointer to ADC_RESOURCES structure
 * @parameter[2] : Control : Selecting the operation
 * @parameter[3] : arg     : values for the the operation
 * @return[1]    : ARM_DRIVER_OK              : if function return successfully
 * @return[2]    : ARM_DRIVER_ERROR_PARAMETER : if adc parameter are invalid
 */
static int32_t ADC_Control(ADC_RESOURCES *ADC, uint32_t Control, uint32_t arg)
{
    int ret = ARM_DRIVER_OK;

    /* Check Power done or not */
    if (!(ADC->state & ADC_FLAG_DRV_POWER_DONE)) {
        return ARM_DRIVER_ERROR;
    }

    switch (Control) {
    case ARM_ADC_SHIFT_CTRL:

        /*selecting the mode for the shifting bit left(0) or right(1) */
        if (arg) {
            adc_output_right_shift(ADC->regs);
        } else {
            adc_output_left_shift(ADC->regs);
        }

        break;

    case ARM_ADC_SEQUENCER_CTRL:

        if (!(arg == 0 || arg == 1)) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /*selecting the mode of control for taking single scan(1) or multiple channel scan(0)*/
        if (arg == ADC_SCAN_MODE_SINGLE_CH) {
            adc_set_single_ch_scan_mode(ADC->regs, &ADC->conv);
            ADC->active_channels = 1UL << ADC->conv.read_channel;
        } else {
            adc_set_multi_ch_scan_mode(ADC->regs, &ADC->conv);
        }

        break;

    case ARM_ADC_SEQUENCER_MSK_CH_CTRL:

        if (!(arg < ADC_MSK_ALL_CHANNELS)) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* set channel to be masked */
        adc_sequencer_msk_ch_control(ADC->regs, arg);

        /* Cache the enabled-channels bitfield */
        if (ADC->drv_instance == ADC_INSTANCE_ADC24_0) {
            ADC->active_channels = ((1UL << ADC24_MAX_DIFFERENTIAL_CHANNEL) - 1UL) & ~arg;
        } else {
            ADC->active_channels = 0xFFUL & ~arg;
        }

        break;

    case ARM_ADC_CHANNEL_INIT_VAL:

        if (!(arg < ADC_MAX_INIT_CHANNEL)) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        if (ADC->differential_enable == ADC_DIFFERENTIAL_ENABLE) {
            /* check for differential input channels
             * 3 input channels are used in differential mode
             * which are channel 0,1 and 2
             */
            if (arg > ADC_MAX_DIFFERENTIAL_CHANNEL) {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
        }

        if (ADC->drv_instance == ADC_INSTANCE_ADC24_0) {
            /* 4 Differential input channels  are there in ADC24 */
            if (arg > ADC24_MAX_DIFFERENTIAL_CHANNEL) {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
        }

        /* select the initial value */
        adc_init_channel_select(ADC->regs, arg);

        /* Store first channel to start conversion */
        ADC->conv.read_channel = arg;

        /* In single-channel scan mode the sequencer visits exactly this
         * channel per sweep; keep active_channels in sync.
         */
        if (ADC->conv.sequencer_ctrl_status == ADC_SCAN_MODE_SINGLE_CH) {
            ADC->active_channels = 1UL << arg;
        }

        break;

    case ARM_ADC_COMPARATOR_A:
        /* set comparator A */
        adc_set_comparator_A(ADC->regs, arg);
        break;

    case ARM_ADC_COMPARATOR_B:
        /* set comparator B */
        adc_set_comparator_B(ADC->regs, arg);
        break;

    case ARM_ADC_THRESHOLD_COMPARISON:

        if (!(arg < 3)) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
        /* set comparison control bit */
        adc_set_comparator_ctrl_bit(ADC->regs, arg);

        break;

    case ARM_ADC_CONVERSION_MODE_CTRL:

        if (!(arg == 0 || arg == 1)) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* set conversion mode */
        if (arg) {
            ADC->conv.mode = ADC_CONV_MODE_SINGLE_SHOT;
#if ADC_DMA_ENABLE
            if (ADC->dma_enable) {
                (void) ADC_DMA_DeAllocate(&ADC->dma_cfg->dma_rx);
            }
#endif
        } else {
            ADC->conv.mode = ADC_CONV_MODE_CONTINUOUS;
#if ADC_DMA_ENABLE
            if (ADC->dma_enable) {
                if (ADC_DMA_Allocate(&ADC->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
                    return ARM_DRIVER_ERROR;
                }
            }
#endif
        }
        break;

    case ARM_ADC_EXTERNAL_TRIGGER_ENABLE:

        if (arg > ADC_EXTERNAL_TRIGGER_MAX_VAL) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        ADC->ext_trig_val = arg;
        break;

    case ARM_ADC_EXTERNAL_TRIGGER_DISABLE:

        if (arg > ADC_EXTERNAL_TRIGGER_MAX_VAL) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        ADC->ext_trig_val = arg;
        break;

    case ARM_ADC_HARDWARE_AVERAGING_CTRL:

        /* argument is power of 2 */
        if ((arg & (arg - 1)) == 0) {
            /* Check if the value is between 2 to 256 */
            if (arg < ADC_AVG_SAMPLES_FOR_AVG_MIN || arg > ADC_AVG_SAMPLES_FOR_AVG_MAX) {
                return ARM_DRIVER_ERROR;
            }
        }

        /* set average sample number */
        adc_set_avg_sample(ADC->regs, arg);
        break;

    case ARM_ADC_INPUT_CLOCK_DIV_CTRL:

        /* check for CLOCK INPUT */
        if (arg > ADC_CLOCK_DIV_MIN_VALUE || arg < ADC_CLOCK_DIV_MAX_VALUE) {
            return ARM_DRIVER_ERROR;
        }

        /* set the clock divisor */
        adc_set_clk_div(ADC->regs, arg);
        break;

    case ARM_ADC_SAMPLE_WIDTH_CTRL:

        /* check for sample width input */
        if (ADC->drv_instance != ADC_INSTANCE_ADC24_0) {
            if ((arg < ADC_SAMPLE_WIDTH_MIN_VALUE || arg > ADC_SAMPLE_WIDTH_MAX_VALUE)) {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
        }

        /* set Sample width value for ADC12 and ADC24*/
        adc_set_sample_width(ADC->regs, arg);
        break;

    case ARM_ADC_DIFFERENTIAL_MODE_CTRL:

        if (arg) {
            if (ADC->drv_instance != ADC_INSTANCE_ADC24_0) {
                adc_set_differential_ctrl(ADC->drv_instance, ENABLE);
            }

            /* set pga gain */
            enable_adc_pga_gain(ADC->drv_instance, ADC->pga_value);
        } else {
            /* Disable differential */
            if (ADC->drv_instance != ADC_INSTANCE_ADC24_0) {
                adc_set_differential_ctrl(ADC->drv_instance, DISABLE);
            }
            /* Disable pga gain */
            disable_adc_pga_gain(ADC->drv_instance);
        }
        break;

    case ARM_ADC_SET_PGA_GAIN_CTRL:

        /* check for pga gain input */
        if (arg > ADC_PGA_GAIN_MAX_VALUE) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* set pga gain */
        enable_adc_pga_gain(ADC->drv_instance, arg);
        break;

    case ARM_ADC_24_BIAS_CTRL:

        /* check for bias control input */
        if (arg > ADC_24_BIAS_MAX_VALUE) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        set_adc24_bias(ADC->drv_instance, arg);
        break;

    case ARM_ADC_24_OUTPUT_RATE_CTRL:

        /* check for the arg input */
        if (arg < ADC_24_OUPUT_RATE_MAX_VALUE) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* set output rate from control register */
        set_adc24_output_rate(ADC->drv_instance, arg);
        break;

    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ret;
}

/* RTE_ADC120 */
#if (RTE_ADC120)

#if RTE_ADC120_DMA_ENABLE
static void              ADC120_DMACallback(uint32_t event, int8_t peri_num);
static ADC_DMA_HW_CONFIG ADC120_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(ADC120_DMA),
        .dma_periph_req = ADC120_DMA_DONE0_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = ADC120_DMA,
            .group            = ADC120_DMA_GROUP,
            .channel          = ADC120_DMA_DONE0_PERIPH_REQ,
            .enable_handshake = ADC120_DMA_HANDSHAKE_ENABLE,
        },
    },
};
static uint8_t adc120_dma_mcode[RTE_ADC120_DMA_MCODE_SIZE] __ALIGNED(32);
#endif /* RTE_ADC120_DMA_ENABLE */

static ADC_RESOURCES ADC120_RES = {
    .cb_event                = NULL,                              /* ARM_ADC_SignalEvent_t        */
    .regs                    = (ADC_Type *) ADC120_BASE,          /* ADC register base address    */
    .conv.user_input         = RTE_ADC120_INPUT_NUM,              /* user input                   */
    .drv_instance            = ADC_INSTANCE_ADC12_0,              /* Driver instances             */
    .intr_done0_irq_num      = (IRQn_Type) ADC120_DONE0_IRQ_IRQn, /* ADC DONE0 IRQ number         */
    .intr_done1_irq_num      = (IRQn_Type) ADC120_DONE1_IRQ_IRQn, /* ADC DONE1 IRQ number         */
    .intr_cmpa_irq_num       = (IRQn_Type) ADC120_CMPA_IRQ_IRQn,  /* ADC CMPA IRQ number          */
    .intr_cmpb_irq_num       = (IRQn_Type) ADC120_CMPB_IRQ_IRQn,  /* ADC CMPB IRQ number          */
    .busy                    = 0,                                 /* ADC busy                     */
    .intr_done0_irq_priority = RTE_ADC120_DONE0_IRQ_PRIORITY,     /* ADC done0 irq priority       */
    .intr_done1_irq_priority = RTE_ADC120_DONE1_IRQ_PRIORITY,     /* ADC done1 irq priority       */
    .intr_cmpa_irq_priority  = RTE_ADC120_CMPA_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .intr_cmpb_irq_priority  = RTE_ADC120_CMPB_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .clock_div               = RTE_ADC120_CLOCK_DIV,              /* clock divisor                */
    .avg_sample_num          = RTE_ADC120_AVG_SAMPLE_NUM,         /* average sample number        */
    .sample_width            = RTE_ADC120_SAMPLE_WIDTH,           /* sample width                 */
    .shift_n_bit             = RTE_ADC120_SHIFT_N_BIT,            /* number of shift bit          */
    .shift_left_or_right     = RTE_ADC120_SHIFT_LEFT_OR_RIGHT,    /* shifting left to right       */
    .differential_enable     = RTE_ADC120_DIFFERENTIAL_EN,
    .comparator_enable       = RTE_ADC120_COMPARATOR_EN,
    .comparator_bias         = RTE_ADC120_COMPARATOR_BIAS,
    .pga_enable              = RTE_ADC120_PGA_EN,
    .pga_value               = RTE_ADC120_PGA_GAIN,
#if RTE_ADC120_DMA_ENABLE
    .dma_enable              = RTE_ADC120_DMA_ENABLE,
    .dma_cb                  = ADC120_DMACallback,
    .dma_cfg                 = &ADC120_DMA_HW_CONFIG,
    .dma_irq_priority        = RTE_ADC120_DMA_IRQ_PRI,
    .dma_mcode               = adc120_dma_mcode,
    .mcode_size              = RTE_ADC120_DMA_MCODE_SIZE,
#endif
};

#if RTE_ADC120_DMA_ENABLE
static void ADC120_DMACallback(uint32_t event, int8_t peri_num)
{
    ADCx_DMACallback(event, peri_num, &ADC120_RES);
}
#endif /* RTE_ADC120_DMA_ENABLE */

/**
 @fn        : void ADC120_DONE0_IRQHandler(void)
 @brief     : DONE0 (AVG SAMPLE RDY) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC120_DONE0_IRQHandler(void)
{
    conv_info_t *conv = &(ADC120_RES.conv);
    uint32_t filled;

    adc_done0_irq_handler(ADC120_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_BUF_A_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_A_FULL;
        ADC120_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_A, 0U, filled);
        if (conv->buf_b == NULL) {
            /* One-shot: halt the sequencer; app must call StartN to re-arm */
            adc_mask_interrupt(ADC120_RES.regs);
            adc_disable_continuous_conv(ADC120_RES.regs);
            conv->buf_a      = NULL;
            conv->active_buf = NULL;
            ADC120_RES.busy  = 0U;
        }
    }
    if (conv->status & ADC_CONV_STAT_BUF_B_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_B_FULL;
        ADC120_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_B, 0U, filled);
    }
    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC120_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status    = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC120_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                            conv->curr_channel,
                            conv->sampled_value);
    }
}

/**
 @fn        : void ADC120_DONE1_IRQHandler (void)
 @brief     : DONE1 (All sample taken) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC120_DONE1_IRQHandler(void)
{
    conv_info_t *conv = &(ADC120_RES.conv);

    adc_done1_irq_handler(ADC120_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC120_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status    = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC120_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                            conv->curr_channel,
                            conv->sampled_value);
    }
}

/**
 @fn        : void ADC120_CMPA_IRQHandler (void)
 @brief     : CMPA Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC120_CMPA_IRQHandler(void)
{
    conv_info_t *conv = &(ADC120_RES.conv);

    adc_cmpa_irq_handler(ADC120_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_A);

        ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_A);

        ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B);

        ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B, 0, 0);
    }
}

/**
 @fn        : void ADC120_CMPB_IRQHandler (void)
 @brief     : CMPB Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC120_CMPB_IRQHandler(void)
{
    conv_info_t *conv = &(ADC120_RES.conv);

    adc_cmpb_irq_handler(ADC120_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_B);

        ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_B);

        ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B);

        ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B, 0, 0);
    }
}

/**
 @fn       ARM_DRIVER_VERSION ADC120_GetVersion(void)
 @brief    Get ADC120 VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ADC120_GetVersion(void)
{
    return DriverVersion;
}
/**
 @fn       ARM_ADC120_CAPABILITIES ADC120_GetCapabilities(void)
 @brief    Get ADC120 CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_ADC_CAPABILITIES ADC120_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           : int32_t ADC120_Initialize(ARM_ADC_SignalEvent_t cb_event)
 @brief        : Initialize the ADC Interface
 @parameter[1] : cb_event : Pointer to \ref ARM_ADC_SignalEvent_t
 @return       : execution_status
**/
static int32_t ADC120_Initialize(ARM_ADC_SignalEvent_t cb_event)
{
    return ADC_Initialize(&ADC120_RES, cb_event);
}

/**
 @fn           : int32_t ADC120_Uninitialize(void)
 @brief        : Un-Initialize the ADC Interface
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC120_Uninitialize(void)
{
    return ADC_Uninitialize(&ADC120_RES);
}

/**
 @fn           : int32_t ADC120_Start(void)
 @brief        : start ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC120_Start(void)
{
    return ADC_Start(&ADC120_RES);
}

/**
 @fn           : int32_t ADC120_Stop(void)
 @brief        : stop ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC120_Stop(void)
{
    return ADC_Stop(&ADC120_RES);
}

/**
 @fn           : int32_t ADC120_PowerControl(ARM_POWER_STATE status)
 @brief        : Control ADC Interface power
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC120_PowerControl(ARM_POWER_STATE status)
{
    return ADC_PowerControl(&ADC120_RES, status);
}

/**
 @fn           : int32_t ADC121_Control(uint32_t Control, uint32_t arg)
 @brief        : Control ADC Interface
 @parameter[1] : Control : control operation
 @parameter[2] : arg     : Argument for operation
 @return       : execution_status
**/
static int32_t ADC120_Control(uint32_t Control, uint32_t arg)
{
    return ADC_Control(&ADC120_RES, Control, arg);
}

/**
 * @fn           : int32_t ADC120_StartN(void *buf_a, void *buf_b,
 *                                       uint32_t samples_per_buf)
 * @brief        : Arm ping-pong buffered continuous scan for ADC120.
 * @parameter[1] : buf_a           : uint32_t[] primary buffer
 * @parameter[2] : buf_b           : uint32_t[] secondary buffer;
 *                                   NULL for one-shot
 * @parameter[3] : samples_per_buf : whole multiple of unmasked channel count
 * @return       : execution_status
 */
static int32_t ADC120_StartN(void *buf_a, void *buf_b, uint32_t samples_per_buf)
{
    return ADC_StartN(&ADC120_RES, buf_a, buf_b, samples_per_buf);
}

/**
 * @fn           : uint32_t ADC120_GetSampleCount(void)
 * @brief        : Samples written into the current StartN buffer.
 * @return       : samples-written count
 */
static uint32_t ADC120_GetSampleCount(void)
{
    return ADC_GetSampleCount(&ADC120_RES);
}

extern ARM_DRIVER_ADC Driver_ADC120;
ARM_DRIVER_ADC        Driver_ADC120 = {
    ADC120_GetVersion,
    ADC120_GetCapabilities,
    ADC120_Initialize,
    ADC120_Uninitialize,
    ADC120_Start,
    ADC120_Stop,
    ADC120_PowerControl,
    ADC120_Control,
    ADC120_StartN,
    ADC120_GetSampleCount
};
#endif /* RTE_ADC120 */

/* RTE_ADC121 */
#if (RTE_ADC121)

#if RTE_ADC121_DMA_ENABLE
static void              ADC121_DMACallback(uint32_t event, int8_t peri_num);
static ADC_DMA_HW_CONFIG ADC121_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(ADC121_DMA),
        .dma_periph_req = ADC121_DMA_DONE0_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = ADC121_DMA,
            .group            = ADC121_DMA_GROUP,
            .channel          = ADC121_DMA_DONE0_PERIPH_REQ,
            .enable_handshake = ADC121_DMA_HANDSHAKE_ENABLE,
        },
    },
};
static uint8_t adc121_dma_mcode[RTE_ADC121_DMA_MCODE_SIZE] __ALIGNED(32);
#endif /* RTE_ADC121_DMA_ENABLE */

static ADC_RESOURCES ADC121_RES = {
    .cb_event                = NULL,                              /* ARM_ADC_SignalEvent_t        */
    .regs                    = (ADC_Type *) ADC121_BASE,          /* ADC register base address    */
    .conv.user_input         = RTE_ADC121_INPUT_NUM,              /* user input                   */
    .drv_instance            = ADC_INSTANCE_ADC12_1,              /* Driver instances             */
    .intr_done0_irq_num      = (IRQn_Type) ADC121_DONE0_IRQ_IRQn, /* ADC DONE0 number             */
    .intr_done1_irq_num      = (IRQn_Type) ADC121_DONE1_IRQ_IRQn, /* ADC DONE1 IRQ number         */
    .intr_cmpa_irq_num       = (IRQn_Type) ADC121_CMPA_IRQ_IRQn,  /* ADC CMPA IRQ number          */
    .intr_cmpb_irq_num       = (IRQn_Type) ADC121_CMPB_IRQ_IRQn,  /* ADC CMPB IRQ number          */
    .busy                    = 0,                                 /* ADC busy                     */
    .intr_done0_irq_priority = RTE_ADC121_DONE0_IRQ_PRIORITY,     /* ADC done0 irq priority       */
    .intr_done1_irq_priority = RTE_ADC121_DONE1_IRQ_PRIORITY,     /* ADC done1 irq priority       */
    .intr_cmpa_irq_priority  = RTE_ADC121_CMPA_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .intr_cmpb_irq_priority  = RTE_ADC121_CMPB_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .clock_div               = RTE_ADC121_CLOCK_DIV,              /* clock divisor                */
    .avg_sample_num          = RTE_ADC121_AVG_SAMPLE_NUM,         /* average sample number        */
    .sample_width            = RTE_ADC121_SAMPLE_WIDTH,           /* sample width                 */
    .shift_n_bit             = RTE_ADC121_SHIFT_N_BIT,            /* number of shift bit          */
    .shift_left_or_right     = RTE_ADC121_SHIFT_LEFT_OR_RIGHT,    /* shifting left to right       */
    .differential_enable     = RTE_ADC121_DIFFERENTIAL_EN,
    .comparator_enable       = RTE_ADC121_COMPARATOR_EN,
    .comparator_bias         = RTE_ADC121_COMPARATOR_BIAS,
    .pga_enable              = RTE_ADC121_PGA_EN,
    .pga_value               = RTE_ADC121_PGA_GAIN,
#if RTE_ADC121_DMA_ENABLE
    .dma_enable              = RTE_ADC121_DMA_ENABLE,
    .dma_cb                  = ADC121_DMACallback,
    .dma_cfg                 = &ADC121_DMA_HW_CONFIG,
    .dma_irq_priority        = RTE_ADC121_DMA_IRQ_PRI,
    .dma_mcode               = adc121_dma_mcode,
    .mcode_size              = RTE_ADC121_DMA_MCODE_SIZE,
#endif
};

#if RTE_ADC121_DMA_ENABLE
static void ADC121_DMACallback(uint32_t event, int8_t peri_num)
{
    ADCx_DMACallback(event, peri_num, &ADC121_RES);
}
#endif /* RTE_ADC121_DMA_ENABLE */

/**
 @fn        : void ADC121_DONE0_IRQHandler(void)
 @brief     : DONE0 (AVG SAMPLE RDY) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC121_DONE0_IRQHandler(void)
{
    conv_info_t *conv = &(ADC121_RES.conv);
    uint32_t filled;

    adc_done0_irq_handler(ADC121_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_BUF_A_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_A_FULL;
        ADC121_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_A, 0U, filled);
        if (conv->buf_b == NULL) {
            /* One-shot: halt the sequencer; app must call StartN to re-arm */
            adc_mask_interrupt(ADC121_RES.regs);
            adc_disable_continuous_conv(ADC121_RES.regs);
            conv->buf_a      = NULL;
            conv->active_buf = NULL;
            ADC121_RES.busy  = 0U;
        }
    }
    if (conv->status & ADC_CONV_STAT_BUF_B_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_B_FULL;
        ADC121_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_B, 0U, filled);
    }
    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC121_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status    = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC121_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                            conv->curr_channel,
                            conv->sampled_value);
    }
}

/**
 @fn        : void ADC121_DONE1_IRQHandler (void)
 @brief     : DONE1 (All sample taken) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC121_DONE1_IRQHandler(void)
{
    conv_info_t *conv = &(ADC121_RES.conv);

    adc_done1_irq_handler(ADC121_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC121_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status    = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC121_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                            conv->curr_channel,
                            conv->sampled_value);
    }
}

/**
 @fn        : void ADC121_CMPA_IRQHandler (void)
 @brief     : CMPA Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC121_CMPA_IRQHandler(void)
{
    conv_info_t *conv = &(ADC121_RES.conv);

    adc_cmpa_irq_handler(ADC121_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_A);

        ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_A);

        ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B);

        ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B, 0, 0);
    }
}

/**
 @fn        : void ADC122_CMPB_IRQHandler (void)
 @brief     : CMPB Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC121_CMPB_IRQHandler(void)
{
    conv_info_t *conv = &(ADC121_RES.conv);

    adc_cmpb_irq_handler(ADC121_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_B);

        ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_B);

        ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B);

        ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B, 0, 0);
    }
}

/**
 @fn       ARM_DRIVER_VERSION ADC121_GetVersion(void)
 @brief    Get ADC121 VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ADC121_GetVersion(void)
{
    return DriverVersion;
}
/**
 @fn       ARM_ADC121_CAPABILITIES ADC121_GetCapabilities(void)
 @brief    Get ADC121 CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_ADC_CAPABILITIES ADC121_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           : int32_t ADC121_Initialize(ARM_ADC_SignalEvent_t cb_event)
 @brief        : Initialize the ADC Interface
 @parameter[1] : cb_event : Pointer to \ref ARM_ADC_SignalEvent_t
 @return       : execution_status
**/
static int32_t ADC121_Initialize(ARM_ADC_SignalEvent_t cb_event)
{
    return ADC_Initialize(&ADC121_RES, cb_event);
}

/**
 @fn           : int32_t ADC121_Uninitialize(void)
 @brief        : Un-Initialize the ADC Interface
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC121_Uninitialize(void)
{
    return ADC_Uninitialize(&ADC121_RES);
}

/**
 @fn           : int32_t ADC121_Start(void)
 @brief        : start ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC121_Start(void)
{
    return ADC_Start(&ADC121_RES);
}

/**
 @fn           : int32_t ADC121_Stop(void)
 @brief        : stop ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC121_Stop(void)
{
    return ADC_Stop(&ADC121_RES);
}

/**
 @fn           : int32_t ADC121_PowerControl(ARM_POWER_STATE status)
 @brief        : Control ADC Interface power
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC121_PowerControl(ARM_POWER_STATE status)
{
    return ADC_PowerControl(&ADC121_RES, status);
}

/**
 @fn           : int32_t ADC121_Control(uint32_t Control, uint32_t arg)
 @brief        : Control ADC Interface
 @parameter[1] : Control : control operation
 @parameter[2] : arg     : Argument for operation
 @return       : execution_status
**/
static int32_t ADC121_Control(uint32_t Control, uint32_t arg)
{
    return ADC_Control(&ADC121_RES, Control, arg);
}

/**
 * @fn           : int32_t ADC121_StartN(void *buf_a, void *buf_b,
 *                                       uint32_t samples_per_buf)
 * @brief        : Arm ping-pong buffered continuous scan for ADC121.
 * @parameter[1] : buf_a           : uint32_t[] primary buffer
 * @parameter[2] : buf_b           : uint32_t[] secondary buffer;
 *                                   NULL for one-shot
 * @parameter[3] : samples_per_buf : whole multiple of unmasked channel count
 * @return       : execution_status
 */
static int32_t ADC121_StartN(void *buf_a, void *buf_b, uint32_t samples_per_buf)
{
    return ADC_StartN(&ADC121_RES, buf_a, buf_b, samples_per_buf);
}

/**
 * @fn           : uint32_t ADC121_GetSampleCount(void)
 * @brief        : Samples written into the current StartN buffer.
 * @return       : samples-written count
 */
static uint32_t ADC121_GetSampleCount(void)
{
    return ADC_GetSampleCount(&ADC121_RES);
}

extern ARM_DRIVER_ADC Driver_ADC121;
ARM_DRIVER_ADC        Driver_ADC121 = {
    ADC121_GetVersion,
    ADC121_GetCapabilities,
    ADC121_Initialize,
    ADC121_Uninitialize,
    ADC121_Start,
    ADC121_Stop,
    ADC121_PowerControl,
    ADC121_Control,
    ADC121_StartN,
    ADC121_GetSampleCount
};
#endif /* RTE_ADC121 */

/* RTE_ADC122 */
#if (RTE_ADC122)

#if RTE_ADC122_DMA_ENABLE
static void              ADC122_DMACallback(uint32_t event, int8_t peri_num);
static ADC_DMA_HW_CONFIG ADC122_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(ADC122_DMA),
        .dma_periph_req = ADC122_DMA_DONE0_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = ADC122_DMA,
            .group            = ADC122_DMA_GROUP,
            .channel          = ADC122_DMA_DONE0_PERIPH_REQ,
            .enable_handshake = ADC122_DMA_HANDSHAKE_ENABLE,
        },
    },
};
static uint8_t adc122_dma_mcode[RTE_ADC122_DMA_MCODE_SIZE] __ALIGNED(32);
#endif /* RTE_ADC122_DMA_ENABLE */

static ADC_RESOURCES ADC122_RES = {
    .cb_event                = NULL,                              /* ARM_ADC_SignalEvent_t        */
    .regs                    = (ADC_Type *) ADC122_BASE,          /* ADC register base address    */
    .conv.user_input         = RTE_ADC122_INPUT_NUM,              /* user input                   */
    .drv_instance            = ADC_INSTANCE_ADC12_2,              /* Driver instances             */
    .intr_done0_irq_num      = (IRQn_Type) ADC122_DONE0_IRQ_IRQn, /* ADC DONE0 IRQ number         */
    .intr_done1_irq_num      = (IRQn_Type) ADC122_DONE1_IRQ_IRQn, /* ADC DONE1 IRQ number         */
    .intr_cmpa_irq_num       = (IRQn_Type) ADC122_CMPA_IRQ_IRQn,  /* ADC CMPA IRQ number          */
    .intr_cmpb_irq_num       = (IRQn_Type) ADC122_CMPB_IRQ_IRQn,  /* ADC CMPB IRQ number          */
    .busy                    = 0,                                 /* ADC busy                     */
    .intr_done0_irq_priority = RTE_ADC122_DONE0_IRQ_PRIORITY,     /* ADC done0 irq priority       */
    .intr_done1_irq_priority = RTE_ADC122_DONE1_IRQ_PRIORITY,     /* ADC done1 irq priority       */
    .intr_cmpa_irq_priority  = RTE_ADC122_CMPA_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .intr_cmpb_irq_priority  = RTE_ADC122_CMPB_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .clock_div               = RTE_ADC122_CLOCK_DIV,              /* clock divisor                */
    .avg_sample_num          = RTE_ADC122_AVG_SAMPLE_NUM,         /* average sample number        */
    .sample_width            = RTE_ADC122_SAMPLE_WIDTH,           /* sample width                 */
    .shift_n_bit             = RTE_ADC122_SHIFT_N_BIT,            /* number of shift bit          */
    .shift_left_or_right     = RTE_ADC122_SHIFT_LEFT_OR_RIGHT,    /* shifting left to right       */
    .differential_enable     = RTE_ADC122_DIFFERENTIAL_EN,
    .comparator_enable       = RTE_ADC122_COMPARATOR_EN,
    .comparator_bias         = RTE_ADC122_COMPARATOR_BIAS,
    .pga_enable              = RTE_ADC122_PGA_EN,
    .pga_value               = RTE_ADC122_PGA_GAIN,
#if RTE_ADC122_DMA_ENABLE
    .dma_enable              = RTE_ADC122_DMA_ENABLE,
    .dma_cb                  = ADC122_DMACallback,
    .dma_cfg                 = &ADC122_DMA_HW_CONFIG,
    .dma_irq_priority        = RTE_ADC122_DMA_IRQ_PRI,
    .dma_mcode               = adc122_dma_mcode,
    .mcode_size              = RTE_ADC122_DMA_MCODE_SIZE,
#endif
};

#if RTE_ADC122_DMA_ENABLE
static void ADC122_DMACallback(uint32_t event, int8_t peri_num)
{
    ADCx_DMACallback(event, peri_num, &ADC122_RES);
}
#endif /* RTE_ADC122_DMA_ENABLE */

/**
 @fn        : void ADC122_DONE0_IRQHandler(void)
 @brief     : DONE0 (AVG SAMPLE RDY) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC122_DONE0_IRQHandler(void)
{
    conv_info_t *conv = &(ADC122_RES.conv);
    uint32_t filled;

    adc_done0_irq_handler(ADC122_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_BUF_A_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_A_FULL;
        ADC122_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_A, 0U, filled);
        if (conv->buf_b == NULL) {
            /* One-shot: halt the sequencer; app must call StartN to re-arm */
            adc_mask_interrupt(ADC122_RES.regs);
            adc_disable_continuous_conv(ADC122_RES.regs);
            conv->buf_a      = NULL;
            conv->active_buf = NULL;
            ADC122_RES.busy  = 0U;
        }
    }
    if (conv->status & ADC_CONV_STAT_BUF_B_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_B_FULL;
        ADC122_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_B, 0U, filled);
    }
    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC122_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status    = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC122_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                            conv->curr_channel,
                            conv->sampled_value);
    }
}

/**
 @fn        : void ADC122_DONE1_IRQHandler (void)
 @brief     : DONE1 (All sample taken) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC122_DONE1_IRQHandler(void)
{
    conv_info_t *conv = &(ADC122_RES.conv);

    adc_done1_irq_handler(ADC122_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC122_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status    = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC122_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                            conv->curr_channel,
                            conv->sampled_value);
    }
}

/**
 @fn        : void ADC122_CMPA_IRQHandler (void)
 @brief     : CMPA Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC122_CMPA_IRQHandler(void)
{
    conv_info_t *conv = &(ADC122_RES.conv);

    adc_cmpa_irq_handler(ADC122_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_A);

        ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_A);

        ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B);

        ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B, 0, 0);
    }
}

/**
 @fn        : void ADC122_CMPB_IRQHandler (void)
 @brief     : CMPB Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC122_CMPB_IRQHandler(void)
{
    conv_info_t *conv = &(ADC122_RES.conv);

    adc_cmpb_irq_handler(ADC122_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_B);

        ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_B);

        ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B);

        ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B, 0, 0);
    }
}

/**
 @fn       ARM_DRIVER_VERSION ADC1_GetVersion(void)
 @brief    Get ADC1 VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ADC122_GetVersion(void)
{
    return DriverVersion;
}
/**
 @fn       ARM_ADC122_CAPABILITIES ADC122_GetCapabilities(void)
 @brief    Get ADC122 CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_ADC_CAPABILITIES ADC122_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           : int32_t ADC122_Initialize(ARM_ADC_SignalEvent_t cb_event)
 @brief        : Initialize the ADC Interface
 @parameter[1] : cb_event : Pointer to \ref ARM_ADC_SignalEvent_t
 @return       : execution_status
**/
static int32_t ADC122_Initialize(ARM_ADC_SignalEvent_t cb_event)
{
    return ADC_Initialize(&ADC122_RES, cb_event);
}

/**
 @fn           : int32_t ADC122_Uninitialize(void)
 @brief        : Un-Initialize the ADC Interface
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC122_Uninitialize(void)
{
    return ADC_Uninitialize(&ADC122_RES);
}

/**
 @fn           : int32_t ADC122_Start(void)
 @brief        : start ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC122_Start(void)
{
    return ADC_Start(&ADC122_RES);
}

/**
 @fn           : int32_t ADC122_Stop(void)
 @brief        : stop ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC122_Stop(void)
{
    return ADC_Stop(&ADC122_RES);
}

/**
 @fn           : int32_t ADC122_PowerControl(ARM_POWER_STATE status)
 @brief        : Control ADC Interface power
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC122_PowerControl(ARM_POWER_STATE status)
{
    return ADC_PowerControl(&ADC122_RES, status);
}

/**
 @fn           : int32_t ADC122_Control(uint32_t Control, uint32_t arg)
 @brief        : Control ADC Interface
 @parameter[1] : Control : control operation
 @parameter[2] : arg     : Argument for operation
 @return       : execution_status
**/
static int32_t ADC122_Control(uint32_t Control, uint32_t arg)
{
    return ADC_Control(&ADC122_RES, Control, arg);
}

/**
 * @fn           : int32_t ADC122_StartN(void *buf_a, void *buf_b,
 *                                       uint32_t samples_per_buf)
 * @brief        : Arm ping-pong buffered continuous scan for ADC122.
 * @parameter[1] : buf_a           : uint32_t[] primary buffer
 * @parameter[2] : buf_b           : uint32_t[] secondary buffer;
 *                                   NULL for one-shot
 * @parameter[3] : samples_per_buf : whole multiple of unmasked channel count
 * @return       : execution_status
 */
static int32_t ADC122_StartN(void *buf_a, void *buf_b, uint32_t samples_per_buf)
{
    return ADC_StartN(&ADC122_RES, buf_a, buf_b, samples_per_buf);
}

/**
 * @fn           : uint32_t ADC122_GetSampleCount(void)
 * @brief        : Samples written into the current StartN buffer.
 * @return       : samples-written count
 */
static uint32_t ADC122_GetSampleCount(void)
{
    return ADC_GetSampleCount(&ADC122_RES);
}

extern ARM_DRIVER_ADC Driver_ADC122;
ARM_DRIVER_ADC        Driver_ADC122 = {
    ADC122_GetVersion,
    ADC122_GetCapabilities,
    ADC122_Initialize,
    ADC122_Uninitialize,
    ADC122_Start,
    ADC122_Stop,
    ADC122_PowerControl,
    ADC122_Control,
    ADC122_StartN,
    ADC122_GetSampleCount
};
#endif /* RTE_ADC122 */

/* RTE_ADC24 */
#if (RTE_ADC24)

#if RTE_ADC24_DMA_ENABLE
static void              ADC24_DMACallback(uint32_t event, int8_t peri_num);
static ADC_DMA_HW_CONFIG ADC24_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(ADC24_DMA),
        .dma_periph_req = ADC24_DMA_DONE0_PERIPH_REQ,
        .evtrtr_cfg     = {
            .instance         = ADC24_DMA,
            .group            = ADC24_DMA_GROUP,
            .channel          = ADC24_DMA_DONE0_PERIPH_REQ,
            .enable_handshake = ADC24_DMA_HANDSHAKE_ENABLE,
        },
    },
};
static uint8_t adc24_dma_mcode[RTE_ADC24_DMA_MCODE_SIZE] __ALIGNED(32);
#endif /* RTE_ADC24_DMA_ENABLE */

static ADC_RESOURCES ADC24_RES = {
    .cb_event                = NULL,                             /* ARM_ADC_SignalEvent_t        */
    .regs                    = (ADC_Type *) ADC24_BASE,          /* ADC register base address    */
    .conv.user_input         = RTE_ADC24_INPUT_NUM,              /* user input                   */
    .drv_instance            = ADC_INSTANCE_ADC24_0,             /* Driver instances             */
    .intr_done0_irq_num      = (IRQn_Type) ADC24_DONE0_IRQ_IRQn, /* ADC DONE0 IRQ number         */
    .intr_done1_irq_num      = (IRQn_Type) ADC24_DONE1_IRQ_IRQn, /* ADC DONE1 IRQ number         */
    .intr_cmpa_irq_num       = (IRQn_Type) ADC24_CMPA_IRQ_IRQn,  /* ADC CMPA IRQ number          */
    .intr_cmpb_irq_num       = (IRQn_Type) ADC24_CMPB_IRQ_IRQn,  /* ADC CMPB IRQ number          */
    .busy                    = 0,                                /* ADC busy                     */
    .intr_done0_irq_priority = RTE_ADC24_DONE0_IRQ_PRIORITY,     /* ADC done0 irq priority       */
    .intr_done1_irq_priority = RTE_ADC24_DONE1_IRQ_PRIORITY,     /* ADC done1 irq priority       */
    .intr_cmpa_irq_priority  = RTE_ADC24_CMPA_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .intr_cmpb_irq_priority  = RTE_ADC24_CMPB_IRQ_PRIORITY,      /* ADC cmpa irq priority        */
    .clock_div               = RTE_ADC24_CLOCK_DIV,              /* clock divisor                */
    .avg_sample_num          = RTE_ADC24_AVG_SAMPLE_NUM,         /* average sample number        */
    .shift_n_bit             = RTE_ADC24_SHIFT_N_BIT,            /* number of shift bit          */
    .shift_left_or_right     = RTE_ADC24_SHIFT_LEFT_OR_RIGHT,    /* shifting left to right       */
    .pga_enable              = RTE_ADC24_PGA_EN,
    .pga_value               = RTE_ADC24_PGA_GAIN,
    .bias                    = RTE_ADC24_BIAS,
    .output_rate             = RTE_ADC24_OUTPUT_RATE,
#if RTE_ADC24_DMA_ENABLE
    .dma_enable              = RTE_ADC24_DMA_ENABLE,
    .dma_cb                  = ADC24_DMACallback,
    .dma_cfg                 = &ADC24_DMA_HW_CONFIG,
    .dma_irq_priority        = RTE_ADC24_DMA_IRQ_PRI,
    .dma_mcode               = adc24_dma_mcode,
    .mcode_size              = RTE_ADC24_DMA_MCODE_SIZE,
#endif
};

#if RTE_ADC24_DMA_ENABLE
static void ADC24_DMACallback(uint32_t event, int8_t peri_num)
{
    ADCx_DMACallback(event, peri_num, &ADC24_RES);
}
#endif /* RTE_ADC24_DMA_ENABLE */

/**
 @fn        : void ADC24_CMPB_IRQHandler (void)
 @brief     : DONE0 (AVG SAMPLE RDY) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC24_DONE0_IRQHandler(void)
{
    conv_info_t *conv = &(ADC24_RES.conv);
    uint32_t filled;

    adc_done0_irq_handler(ADC24_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_BUF_A_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_A_FULL;
        ADC24_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_A, 0U, filled);
        if (conv->buf_b == NULL) {
            /* One-shot: halt the sequencer; app must call StartN to re-arm */
            adc_mask_interrupt(ADC24_RES.regs);
            adc_disable_continuous_conv(ADC24_RES.regs);
            conv->buf_a      = NULL;
            conv->active_buf = NULL;
            ADC24_RES.busy   = 0U;
        }
    }
    if (conv->status & ADC_CONV_STAT_BUF_B_FULL) {
        filled           = conv->samples_per_buf;
        conv->status    &= ~ADC_CONV_STAT_BUF_B_FULL;
        ADC24_RES.cb_event(ARM_ADC_EVENT_CONTINUOUS_CONV_BUF_B, 0U, filled);
    }
    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC24_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status   = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC24_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                           conv->curr_channel,
                           conv->sampled_value);
    }
}

/**
 @fn        : void ADC24_DONE1_IRQHandler (void)
 @brief     : DONE1 (All sample taken) Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC24_DONE1_IRQHandler(void)
{
    conv_info_t *conv = &(ADC24_RES.conv);

    adc_done1_irq_handler(ADC24_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_COMPLETE) {
        /* set busy flag to 0U */
        ADC24_RES.busy = 0U;

        /* clearing conversion complete status */
        conv->status   = (conv->status & ~ADC_CONV_STAT_COMPLETE);

        ADC24_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE,
                           conv->curr_channel,
                           conv->sampled_value);
    }
}

/**
 @fn        : void ADC24_CMPA_IRQHandler (void)
 @brief     : CMPA Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC24_CMPA_IRQHandler(void)
{
    conv_info_t *conv = &(ADC24_RES.conv);

    adc_cmpa_irq_handler(ADC24_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_A);

        ADC24_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_A) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_A);

        ADC24_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B);

        ADC24_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B, 0, 0);
    }
}

/**
 @fn        : void ADC24_CMPB_IRQHandler (void)
 @brief     : CMPB Interrupt Handler
 @parameter : NONE
 @return    : NONE
**/
void ADC24_CMPB_IRQHandler(void)
{
    conv_info_t *conv = &(ADC24_RES.conv);

    adc_cmpb_irq_handler(ADC24_RES.regs, conv);

    if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_ABOVE_B);

        ADC24_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_BELOW_B);

        ADC24_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B, 0, 0);
    }

    if (conv->status & ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B) {
        /* clearing comparator status */
        conv->status = (conv->status & ~ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B);

        ADC24_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B, 0, 0);
    }
}

/**
 @fn       ARM_DRIVER_VERSION ADC24_GetVersion(void)
 @brief    Get ADC24 VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ADC24_GetVersion(void)
{
    return DriverVersion;
}
/**
 @fn       ARM_ADC24_CAPABILITIES ADC24_GetCapabilities(void)
 @brief    Get ADC24 CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_ADC_CAPABILITIES ADC24_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           : int32_t ADC24_Initialize(ARM_ADC_SignalEvent_t cb_event)
 @brief        : Initialize the ADC Interface
 @parameter[1] : cb_event : Pointer to \ref ARM_ADC_SignalEvent_t
 @return       : execution_status
**/
static int32_t ADC24_Initialize(ARM_ADC_SignalEvent_t cb_event)
{
    return ADC_Initialize(&ADC24_RES, cb_event);
}

/**
 @fn           : int32_t ADC24_Uninitialize(void)
 @brief        : Un-Initialize the ADC Interface
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC24_Uninitialize(void)
{
    return ADC_Uninitialize(&ADC24_RES);
}

/**
 @fn           : int32_t ADC24_Start(void)
 @brief        : start ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC24_Start(void)
{
    return ADC_Start(&ADC24_RES);
}

/**
 @fn           : int32_t ADC24_Stop(void)
 @brief        : stop ADC driver
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC24_Stop(void)
{
    return ADC_Stop(&ADC24_RES);
}

/**
 @fn           : int32_t ADC24_PowerControl(ARM_POWER_STATE status)
 @brief        : Control ADC Interface power
 @parameter    : NONE
 @return       : execution_status
**/
static int32_t ADC24_PowerControl(ARM_POWER_STATE status)
{
    return ADC_PowerControl(&ADC24_RES, status);
}

/**
 @fn           : int32_t ADC24_Control(uint32_t Control, uint32_t arg)
 @brief        : Control ADC Interface
 @parameter[1] : Control : control operation
 @parameter[2] : arg     : Argument for operation
 @return       : execution_status
**/
static int32_t ADC24_Control(uint32_t Control, uint32_t arg)
{
    return ADC_Control(&ADC24_RES, Control, arg);
}

/**
 * @fn           : int32_t ADC24_StartN(void *buf_a, void *buf_b,
 *                                      uint32_t samples_per_buf)
 * @brief        : Arm ping-pong buffered continuous scan for ADC24.
 * @parameter[1] : buf_a           : uint32_t[] primary buffer
 * @parameter[2] : buf_b           : uint32_t[] secondary buffer;
 *                                   NULL for one-shot
 * @parameter[3] : samples_per_buf : whole multiple of unmasked channel count
 * @return       : execution_status
 */
static int32_t ADC24_StartN(void *buf_a, void *buf_b, uint32_t samples_per_buf)
{
    return ADC_StartN(&ADC24_RES, buf_a, buf_b, samples_per_buf);
}

/**
 * @fn           : uint32_t ADC24_GetSampleCount(void)
 * @brief        : Samples written into the current StartN buffer.
 * @return       : samples-written count
 */
static uint32_t ADC24_GetSampleCount(void)
{
    return ADC_GetSampleCount(&ADC24_RES);
}

extern ARM_DRIVER_ADC Driver_ADC24;
ARM_DRIVER_ADC        Driver_ADC24 = {
    ADC24_GetVersion,
    ADC24_GetCapabilities,
    ADC24_Initialize,
    ADC24_Uninitialize,
    ADC24_Start,
    ADC24_Stop,
    ADC24_PowerControl,
    ADC24_Control,
    ADC24_StartN,
    ADC24_GetSampleCount
};
#endif /* RTE_ADC24 */
#endif /* RTE_Drivers_ADC */
