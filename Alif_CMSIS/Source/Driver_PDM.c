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
 * @file     Driver_PDM.h
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     12-Jan-2023
 * @brief    CMSIS-Driver for PDM.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Project Includes */
#include "Driver_PDM_Private.h"
#include "sys_ctrl_pdm.h"
#include "sys_utils.h"

#if defined(RTE_Drivers_PDM)

#define ARM_PDM_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 2) /*  Driver version */

/*Driver version*/
static const ARM_DRIVER_VERSION DriverVersion        = {ARM_PDM_API_VERSION, ARM_PDM_DRV_VERSION};

/*Driver Capabilities   */
static const ARM_PDM_CAPABILITIES DriverCapabilities = {
    1, /* supports Mono mode           */
    1, /* supports synchronous Receive */
    0  /* Reserved ( must be ZERO)     */
};

/**
 @fn           ARM_DRIVER_VERSION PDM_GetVersion(void)
 @brief        get PDM version
 @param        none
 @return       driver version
 */
static ARM_DRIVER_VERSION PDM_GetVersion(void)
{
    return DriverVersion;
}

/**
 @fn           ARM_PDM_CAPABILITIES PDM_GetCapabilities(void)
 @brief        get PDM Capabilites
 @param        none
 @return       driver Capabilites
 */
static ARM_PDM_CAPABILITIES PDM_GetCapabilities(void)
{
    return DriverCapabilities;
}

#if PDM_DMA_ENABLE
/**
  \fn          int32_t PDM_DMA_Initialize(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       Initialize DMA for PDM
  \param[in]   dma_periph   Pointer to DMA resources
  \return      \ref         execution_status
*/
static inline int32_t PDM_DMA_Initialize(DMA_PERIPHERAL_CONFIG *dma_periph)
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
  \fn          int32_t PDM_DMA_PowerControl(ARM_POWER_STATE state,
                                            DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       PowerControl DMA for PDM
  \param[in]   state  Power state
  \param[in]   dma_periph     Pointer to DMA resources
  \return      \ref execution_status
*/
static inline int32_t PDM_DMA_PowerControl(ARM_POWER_STATE state, DMA_PERIPHERAL_CONFIG *dma_periph)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Initializes DMA interface */
    status                  = dma_drv->PowerControl(state);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PDM_DMA_Allocate(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       Allocate a channel for PDM
  \param[in]   dma_periph  Pointer to DMA resources
  \return      \ref        execution_status
*/
static inline int32_t PDM_DMA_Allocate(DMA_PERIPHERAL_CONFIG *dma_periph)
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
  \fn          int32_t PDM_DMA_DeAllocate(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       De-allocate channel of PDM
  \param[in]   dma_periph  Pointer to DMA resources
  \return      \ref        execution_status
*/
static inline int32_t PDM_DMA_DeAllocate(DMA_PERIPHERAL_CONFIG *dma_periph)
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
  \fn          int32_t PDM_DMA_Start(DMA_PERIPHERAL_CONFIG *dma_periph,
                                     ARM_DMA_PARAMS *dma_params)
  \brief       Start PDM DMA transfer
  \param[in]   dma_periph     Pointer to DMA resources
  \param[in]   dma_params     Pointer to DMA parameters
  \return      \ref           execution_status
*/
static inline int32_t PDM_DMA_Start(DMA_PERIPHERAL_CONFIG *dma_periph, ARM_DMA_PARAMS *dma_params)
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
  \fn          int32_t PDM_DMA_Stop(DMA_PERIPHERAL_CONFIG *dma_periph)
  \brief       Stop PDM DMA transfer
  \param[in]   dma_periph   Pointer to DMA resources
  \return      \ref         execution_status
*/
static inline int32_t PDM_DMA_Stop(DMA_PERIPHERAL_CONFIG *dma_periph)
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

/**
  \fn          int32_t PDM_DMA_GetStatus(DMA_PERIPHERAL_CONFIG *dma_periph, uint32_t *count)
  \brief       Status of PDM DMA transfer
  \param[in]   dma_periph   Pointer to DMA resources
  \param[in]   count        Current transfer count
  \return      \ref         execution_status
*/
static inline int32_t PDM_DMA_GetStatus(DMA_PERIPHERAL_CONFIG *dma_periph, uint32_t *count)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Stop transfer */
    status                  = dma_drv->GetStatus(&dma_periph->dma_handle, count);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PDM_DMA_Usermcode(DMA_PERIPHERAL_CONFIG *dma_periph,
                                         uint32_t dma_mcode)
  \brief       Use Custom Microcode for PDM
  \param[in]   dma_periph  Pointer to DMA resources
  \param[in]   dma_mcode  Pointer to DMA microcode
  \return      \ref        execution_status
*/
__STATIC_INLINE int32_t PDM_DMA_Usermcode(DMA_PERIPHERAL_CONFIG *dma_periph, uint32_t dma_mcode)
{
    int32_t         status;
    ARM_DRIVER_DMA *dma_drv = dma_periph->dma_drv;

    /* Use User provided custom microcode */
    status = dma_drv->Control(&dma_periph->dma_handle, ARM_DMA_USER_PROVIDED_MCODE, dma_mcode);
    if (status) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
 @fn          int32_t PDM_DMA_GenerateOpcode(DMA_PERIPHERAL_CONFIG *dma_periph,
                                             PDM_RESOURCES *PDM_RES)
  @brief       Generate PDM DMA microcode
  \param[in]   dma_periph   Pointer to DMA resources
  @param[in]   PDM_RES : Pointer to PDM resources
  \return      \ref         execution_status
*/
int32_t PDM_DMA_GenerateOpcode(DMA_PERIPHERAL_CONFIG *dma_periph, PDM_RESOURCES *PDM_RES)
{
    uint8_t        dma_handle      = (uint8_t) dma_periph->dma_handle;
    uint32_t       dst_addr        = LocalToGlobal(PDM_RES->transfer.buf);
    uint32_t       active_channels = pdm_get_active_channels(PDM_RES->regs);
    uint8_t        periph_num      = PDM_RES->dma_cfg->dma_rx.dma_periph_req;
    dma_opcode_buf op_buf;
    dma_loop_t     lp_args;
    dma_ccr_t      ccr;
    DMA_XFER       xfer_type;
    bool           ret;
    uint32_t       total_bytes, req_burst, burst;
    uint16_t       lp_start_lc1, lp_start_lc0, lc0, lc1;
    uint32_t       src_addr;
    uint8_t        tmp, num_active_channels;

    op_buf.buf                   = (uint8_t *) &PDM_RES->dma_mcode;
    op_buf.buf_size              = PDM_DMA_MICROCODE_SIZE;
    op_buf.off                   = 0;

    ccr.value                    = 0;

    ccr.value_b.dst_burst_len    = 0;
    ccr.value_b.src_burst_len    = 0;
    ccr.value_b.dst_burst_size   = BS_BYTE_4;
    ccr.value_b.src_burst_size   = BS_BYTE_4;
    ccr.value_b.dst_cache_ctrl   = DMA_DEST_CACHE_CTRL;
    ccr.value_b.src_cache_ctrl   = 0;
    ccr.value_b.dst_inc          = DMA_BURST_INCREMENTING;
    ccr.value_b.src_inc          = DMA_BURST_FIXED;
    ccr.value_b.dst_prot_ctrl    = DMA_DEST_PROT_CTRL;
    ccr.value_b.src_prot_ctrl    = DMA_SRC_PROT_CTRL;
    ccr.value_b.endian_swap_size = 0x0;

    ret                          = dma_construct_move(ccr.value, DMA_REG_CCR, &op_buf);
    if (!ret) {
        return ret;
    }

    ret = dma_construct_move(dst_addr, DMA_REG_DAR, &op_buf);
    if (!ret) {
        return ret;
    }

    src_addr = LocalToGlobal(pdm_get_ch0_1_addr(PDM_RES->regs));
    ret      = dma_construct_move(src_addr, DMA_REG_SAR, &op_buf);
    if (!ret) {
        return ret;
    }

    num_active_channels = 0;
    tmp                 = active_channels;
    while (tmp) {
        tmp = tmp & (tmp - 1);
        num_active_channels++;
    }
    /* Stereo Mode only */
    num_active_channels = num_active_channels / 2;

    burst               = (1 << BS_BYTE_4);
    total_bytes         = PDM_RES->transfer.total_cnt * 2;
    req_burst           = total_bytes / (burst * num_active_channels);

    while (req_burst) {
        if (req_burst >= (DMA_MAX_LP_CNT * DMA_MAX_LP_CNT)) {
            lc0       = DMA_MAX_LP_CNT;
            lc1       = DMA_MAX_LP_CNT;
            req_burst = req_burst - (DMA_MAX_LP_CNT * DMA_MAX_LP_CNT);
        } else if (req_burst >= DMA_MAX_LP_CNT) {
            lc0       = DMA_MAX_LP_CNT;
            lc1       = (uint16_t) (req_burst / lc0);
            req_burst = req_burst - (lc0 * lc1);
        } else {
            lc0       = (uint16_t) req_burst;
            lc1       = 0;
            req_burst = 0;
        }

        lp_start_lc1 = 0;
        if (lc1) {
            ret = dma_construct_loop(DMA_LC_1, (uint8_t) lc1, &op_buf);
            if (!ret) {
                return ret;
            }
            lp_start_lc1 = op_buf.off;
        }

        if (lc0 == 0) {
            return ret;
        }

        ret = dma_construct_loop(DMA_LC_0, (uint8_t) lc0, &op_buf);
        if (!ret) {
            return ret;
        }

        lp_start_lc0 = op_buf.off;

        xfer_type    = DMA_XFER_SINGLE;

        ret          = dma_construct_flushperiph(periph_num, &op_buf);
        if (!ret) {
            return ret;
        }

        ret = dma_construct_wfp(xfer_type, periph_num, &op_buf);
        if (!ret) {
            return ret;
        }

        if (active_channels & PDM_CHANNEL_0_1) {
            ret = dma_construct_loadperiph(xfer_type, periph_num, &op_buf);
            if (!ret) {
                return ret;
            }

            ret = dma_construct_store(xfer_type, &op_buf);
            if (!ret) {
                return ret;
            }
        }

        if (active_channels & PDM_CHANNEL_2_3) {
            ret = dma_construct_add(DMA_REG_SAR, 0x4, &op_buf);
            if (!ret) {
                return ret;
            }
            ret = dma_construct_loadperiph(xfer_type, periph_num, &op_buf);
            if (!ret) {
                return ret;
            }

            ret = dma_construct_store(xfer_type, &op_buf);
            if (!ret) {
                return ret;
            }

            ret = dma_construct_addneg(DMA_REG_SAR, 0x4, &op_buf);
            if (!ret) {
                return ret;
            }
        }

        if (active_channels & PDM_CHANNEL_4_5) {
            ret = dma_construct_add(DMA_REG_SAR, 0x8, &op_buf);
            if (!ret) {
                return ret;
            }
            ret = dma_construct_loadperiph(xfer_type, periph_num, &op_buf);
            if (!ret) {
                return ret;
            }

            ret = dma_construct_store(xfer_type, &op_buf);
            if (!ret) {
                return ret;
            }

            ret = dma_construct_addneg(DMA_REG_SAR, 0x8, &op_buf);
            if (!ret) {
                return ret;
            }
        }

        if (active_channels & PDM_CHANNEL_6_7) {
            ret = dma_construct_add(DMA_REG_SAR, 0xC, &op_buf);
            if (!ret) {
                return ret;
            }
            ret = dma_construct_loadperiph(xfer_type, periph_num, &op_buf);
            if (!ret) {
                return ret;
            }

            ret = dma_construct_store(xfer_type, &op_buf);
            if (!ret) {
                return ret;
            }

            ret = dma_construct_addneg(DMA_REG_SAR, 0xC, &op_buf);
            if (!ret) {
                return ret;
            }
        }

        if ((op_buf.off - lp_start_lc0) > DMA_MAX_BACKWARD_JUMP) {
            return false;
        }
        lp_args.jump      = (uint8_t) (op_buf.off - lp_start_lc0);
        lp_args.lc        = DMA_LC_0;
        lp_args.nf        = 1;
        lp_args.xfer_type = DMA_XFER_FORCE;
        ret               = dma_construct_loopend(&lp_args, &op_buf);
        if (!ret) {
            return ret;
        }

        if (lc1) {
            if ((op_buf.off - lp_start_lc1) > DMA_MAX_BACKWARD_JUMP) {
                return false;
            }
            lp_args.jump      = (uint8_t) (op_buf.off - lp_start_lc1);
            lp_args.lc        = DMA_LC_1;
            lp_args.nf        = 1;
            lp_args.xfer_type = DMA_XFER_FORCE;
            ret               = dma_construct_loopend(&lp_args, &op_buf);
            if (!ret) {
                return ret;
            }
        }
    }

    ret = dma_construct_wmb(&op_buf);
    if (!ret) {
        return ret;
    }

    ret = dma_construct_send_event(dma_handle, &op_buf);
    if (!ret) {
        return ret;
    }

    ret = dma_construct_end(&op_buf);
    if (!ret) {
        return ret;
    }

    /* If mcode is not in TCM, Flush the Cache now */
    RTSS_CleanDCache_by_Addr(op_buf.buf, op_buf.buf_size);

    return true;
}
#endif

/**
 @fn          void PDM_ERROR_IRQ_handler(PDM_RESOURCES *PDM_RES)
 @brief       IRQ handler for the error interrupt
 @param[in]   PDM_RES : Pointer to PDM resources
 @return      none
 */
void PDM_ERROR_IRQ_handler(PDM_RESOURCES *PDM_RES)
{
    pdm_error_detect_irq_handler(PDM_RES->regs);

    /* call user callback */
    PDM_RES->cb_event(ARM_PDM_EVENT_ERROR);
}

/**
 @fn          void PDM_AUDIO_DETECT_IRQ_handler(PDM_RESOURCES *PDM_RES)
 @brief       IRQ handler for the audio detect interrupt
 @param[in]   PDM_RES : Pointer to PDM resources
 @return      none
 */
void PDM_AUDIO_DETECT_IRQ_handler(PDM_RESOURCES *PDM_RES)
{
    pdm_transfer_t *transfer = &PDM_RES->transfer;

    pdm_audio_detect_irq_handler(PDM_RES->regs, transfer);

    if (transfer->status == PDM_AUDIO_STATUS_DETECTION) {
        transfer->status = PDM_CAPTURE_STATUS_NONE;

        /* call user callback */
        PDM_RES->cb_event(ARM_PDM_EVENT_AUDIO_DETECTION);
    }
}

/**
 @fn          void PDM_WARNING_IRQ_handler(PDM_RESOURCES *PDM_RES)
 @brief       IRQ handler for the PDM warning interrupt
 @param[in]   PDM_RES : Pointer to PDM resources
 @return      none
 */
void PDM_WARNING_IRQ_handler(PDM_RESOURCES *PDM_RES)
{
    pdm_transfer_t *transfer = &(PDM_RES->transfer);

    pdm_warning_irq_handler(PDM_RES->regs, transfer);

    if (transfer->status == PDM_CAPTURE_STATUS_COMPLETE) {
        transfer->status = PDM_CAPTURE_STATUS_NONE;

        /* call user callback */
        PDM_RES->cb_event(ARM_PDM_EVENT_CAPTURE_COMPLETE);
    }
}

#if PDM_DMA_ENABLE
/**
 * @fn         void PDMx_DMACallback(uint32_t event, int8_t peri_num, PDM_RESOURCES *PDM_RES)
 * @brief      DMA Callback function for PDM.
 * @note       none.
 * @param[in]  PDM_RES : Pointer to pdm resources structure.
 * @param[in]  event : Event from DMA
 * @param[in]  peri_num : peripheral request number
 * @retval     none
 */
static void PDMx_DMACallback(uint32_t event, int8_t peri_num, PDM_RESOURCES *PDM_RES)
{
    ARG_UNUSED(peri_num);

    if (!PDM_RES->cb_event) {
        return;
    }

    if (event & ARM_DMA_EVENT_COMPLETE) {
        /* Disable the PDM error irq */
        pdm_disable_error_irq(PDM_RES->regs);

        PDM_RES->cb_event(ARM_PDM_EVENT_CAPTURE_COMPLETE);
    }

    /* Abort Occurred */
    if (event & ARM_DMA_EVENT_ABORT) {
        /*
         * There is no event for indicating error in PDM driver.
         * Let the application get timeout and restart the PDM.
         *
         */
    }
}
#endif

/**
 * @fn      ARM_PDM_STATUS PDMx_GetStatus(PDM_RESOURCES *PDM_RES)
 * @brief   CMSIS-Driver PDM get status
 * @note    none.
 * @param   PDM_RES    : Pointer to PDM resources structure
 * @retval  ARM_PDM_STATUS
 */
static ARM_PDM_STATUS PDMx_GetStatus(PDM_RESOURCES *PDM_RES)
{
    return PDM_RES->status;
}

/**
@fn          int32_t PDMx_Initialize(ARM_PDM_SignalEvent_t cb_event, PDM_RESOURCES *PDM_RES)
@brief       Initialize the PDM interface
@param[in]   PDM_RES : Pointer to PDM resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if PDM device is invalid
             ARM_DRIVER_OK              : if PDM successfully initialized or already initialized
 */
static int32_t PDMx_Initialize(ARM_PDM_SignalEvent_t cb_event, PDM_RESOURCES *PDM_RES)
{
    if (!cb_event) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (PDM_RES->state.initialized == 1) {
        return ARM_DRIVER_OK;
    }

    /* User call back Event */
    PDM_RES->cb_event = cb_event;

#if PDM_DMA_ENABLE
    if (PDM_RES->dma_enable) {
        PDM_RES->dma_cfg->dma_rx.dma_handle = -1;

        /* Initialize DMA for PDM-Rx */
        if (PDM_DMA_Initialize(&PDM_RES->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
            return ARM_DRIVER_ERROR;
        }
    }
#endif

    /* Setting the state */
    PDM_RES->state.initialized = 1;

    return ARM_DRIVER_OK;
}

/**
@fn          int32_t PDMx_Uninitialize(PDM_RESOURCES *PDM_RES)
@brief       UnInitialize the PDM interface
@param[in]   PDM_RES : Pointer to PDM resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if PDM device is invalid
             ARM_DRIVER_OK              : if PDM successfully initialized or already initialized
 */
static int32_t PDMx_Uninitialize(PDM_RESOURCES *PDM_RES)
{
    if (PDM_RES->state.initialized == 0) {
        return ARM_DRIVER_OK;
    }

    /* set call back to NULL */
    PDM_RES->cb_event = NULL;

#if PDM_DMA_ENABLE
    if (PDM_RES->dma_enable) {
        PDM_RES->dma_cfg->dma_rx.dma_handle = -1;
    }
#endif

    /* Reset the state */
    PDM_RES->state.initialized = 0U;

    return ARM_DRIVER_OK;
}

/**
 @fn          int32_t PDMx_Channel_Config(PDM_CH_CONFIG *cnfg, PDM_RESOURCES *PDM_RES)
 @brief       PDM channel configurations
 @param[in]   PDM_RES : Pointer to PDM resources
 @param[in]   cngf : Pointer to PDM_CH_CONFIG
 @return      ARM_DRIVER_OK : if function return successfully
 */
static int32_t PDMx_Channel_Config(PDM_CH_CONFIG *cnfg, PDM_RESOURCES *PDM_RES)
{
    if (PDM_RES->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    if (PDM_RES->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* Store the fir coefficient values */
    pdm_set_fir_coeff(PDM_RES->regs, cnfg->ch_num, cnfg->ch_fir_coef);

    /* Store the iir coefficient values */
    pdm_set_ch_iir_coef(PDM_RES->regs, cnfg->ch_num, cnfg->ch_iir_coef);

    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t PDMx_PowerControl (ARM_POWER_STATE state,
                                          PDM_RESOURCES *PDM_RES)
 @brief        CMSIS-DRIVER PDM power control
 @param[in]    state : Power state
 @param[in]    PDM_RES   : Pointer to PDM resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if PDM device is invalid
               ARM_DRIVER_OK               : if PDM successfully uninitialized or already not
 initialized
 */
static int32_t PDMx_PowerControl(ARM_POWER_STATE state, PDM_RESOURCES *PDM_RES)
{
    if (PDM_RES->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    switch (state) {
    case ARM_POWER_OFF:

        /* Clear the fifo clear bit */
        pdm_disable_fifo_clear(PDM_RES->regs);

        if (PDM_RES->instance == PDM_INSTANCE_LPPDM) {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM_RES->warning_irq);

            /* Disable the NIVC */
            NVIC_DisableIRQ(PDM_RES->warning_irq);

            /* Disable LPPDM clock */
            disable_lppdm_periph_clk();
        } else {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM_RES->warning_irq);
            NVIC_ClearPendingIRQ(PDM_RES->error_irq);
            NVIC_ClearPendingIRQ(PDM_RES->audio_detect_irq);

            /* Disable the NIVC */
            NVIC_DisableIRQ(PDM_RES->warning_irq);
            NVIC_DisableIRQ(PDM_RES->error_irq);
            NVIC_DisableIRQ(PDM_RES->audio_detect_irq);

            /* Disable PDM clock */
            disable_pdm_periph_clk();
        }

#if PDM_DMA_ENABLE
        if (PDM_RES->dma_enable) {
            /* Disable the DMA handshake */
            pdm_dma_handshake(PDM_RES->regs, false);

            /* DeAllocate DMA for PDM-Rx */
            if (PDM_DMA_DeAllocate(&PDM_RES->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }
            /* Power Control DMA for PDM-Rx */
            if (PDM_DMA_PowerControl(state, &PDM_RES->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }
        }
#endif

        /* Reset the power status of PDM */
        PDM_RES->state.powered = 0;

        break;

    case ARM_POWER_FULL:

        if (PDM_RES->state.initialized == 0) {
            return ARM_DRIVER_ERROR;
        }

        if (PDM_RES->state.powered == 1) {
            return ARM_DRIVER_OK;
        }

        if (PDM_RES->instance == PDM_INSTANCE_LPPDM) {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM_RES->warning_irq);

            /* Set priority */
            NVIC_SetPriority(PDM_RES->warning_irq, PDM_RES->warning_irq_priority);

            /* Enable the NIVC */
            NVIC_EnableIRQ(PDM_RES->warning_irq);

            /* Enable LPPDM clock */
            enable_lppdm_periph_clk();
        } else {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM_RES->warning_irq);
            NVIC_ClearPendingIRQ(PDM_RES->error_irq);
            NVIC_ClearPendingIRQ(PDM_RES->audio_detect_irq);

            /* Set priority */
            NVIC_SetPriority(PDM_RES->warning_irq, PDM_RES->warning_irq_priority);
            NVIC_SetPriority(PDM_RES->error_irq, PDM_RES->error_irq_priority);
            NVIC_SetPriority(PDM_RES->audio_detect_irq, PDM_RES->audio_irq_priority);

            /* Enable the NIVC */
            NVIC_EnableIRQ(PDM_RES->warning_irq);
            NVIC_EnableIRQ(PDM_RES->error_irq);
            NVIC_EnableIRQ(PDM_RES->audio_detect_irq);

            /* Enable PDM clock */
            enable_pdm_periph_clk();
        }

        /* Set the FIFO clear bit */
        pdm_enable_fifo_clear(PDM_RES->regs);

        /* Set the fifo watermark value */
        pdm_set_fifo_watermark(PDM_RES->regs, PDM_RES->fifo_watermark);

#if PDM_DMA_ENABLE
        if (PDM_RES->dma_enable) {
            /* Power Control DMA for PDM-Rx */
            if (PDM_DMA_PowerControl(state, &PDM_RES->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }
            /* Allocate DMA for PDM-Rx */
            if (PDM_DMA_Allocate(&PDM_RES->dma_cfg->dma_rx) != ARM_DRIVER_OK) {
                return ARM_DRIVER_ERROR;
            }

            /* Enable the DMA handshake */
            pdm_dma_handshake(PDM_RES->regs, true);
        }
#endif

        /* Set the power state enabled */
        PDM_RES->state.powered = 1;

        break;

    case ARM_POWER_LOW:

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t PDMx_Control (uint32_t control,
                                    uint32_t arg1, uint32_t arg2,
                                    PDM_RESOURCES *PDM_RES)
 @brief        CMSIS-Driver PDM control.
               Control PDM Interface.
 @param[in]    control : Operation \ref Driver_PDM.h : PDM control codes
 @param[in]    arg1     : Argument of operation (optional)
  @param[in]   arg2     : Argument of operation (optional)
 @param[in]    PDM_RES     : Pointer to PDM resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if PDM device is invalid
               ARM_DRIVER_OK               : if PDM successfully uninitialized or already not
 initialized
 */
static int32_t PDMx_Control(uint32_t control, uint32_t arg1, uint32_t arg2, PDM_RESOURCES *PDM_RES)
{
    /* Verify whether the driver is initialized and powered*/
    if (PDM_RES->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    switch (control) {
    case ARM_PDM_SELECT_RESOLUTION:

        if (arg1 != ARM_PDM_16BIT_RESOLUTION) {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        break;

    case ARM_PDM_CHANNEL_PHASE:

        if (arg1 > PDM_MAX_CHANNEL || arg2 > PDM_MAX_PHASE_CTRL) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* Store the channel phase control values */
        pdm_set_ch_phase(PDM_RES->regs, arg1, arg2);

        break;

    case ARM_PDM_CHANNEL_GAIN:

        if (arg1 > PDM_MAX_CHANNEL || arg2 > PDM_MAX_GAIN_CTRL) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* Store the Gain value */
        pdm_set_ch_gain(PDM_RES->regs, arg1, arg2);

        break;

    case ARM_PDM_CHANNEL_PEAK_DETECT_TH:

        if (arg1 > PDM_MAX_CHANNEL) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* Store the Peak Detector Threshold */
        pdm_set_peak_detect_th(PDM_RES->regs, arg1, arg2);

        break;

    case ARM_PDM_CHANNEL_PEAK_DETECT_ITV:

        if (arg1 > PDM_MAX_CHANNEL) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* Store the Peak Detector Interval */
        pdm_set_peak_detect_itv(PDM_RES->regs, arg1, arg2);

        break;

    case ARM_PDM_SELECT_CHANNEL:

        if (arg2) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* Clear PDM channels */
        pdm_clear_channel(PDM_RES->regs);

        /* Enable PDM multi channel */
        pdm_enable_multi_ch(PDM_RES->regs, arg1);

        break;

    case ARM_PDM_MODE:

        if (arg2) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* Clear the PDM modes */
        pdm_clear_modes(PDM_RES->regs);

        /* Select the PDM modes */
        pdm_enable_modes(PDM_RES->regs, arg1);

        break;

    case ARM_PDM_BYPASS_IIR_FILTER:

        if (arg2) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* To select the Bypass IIR filter */
        pdm_bypass_iir(PDM_RES->regs, arg1);

        break;

    case ARM_PDM_BYPASS_FIR_FILTER:

        if (arg2) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* To select the Bypass FIR filter */
        pdm_bypass_fir(PDM_RES->regs, arg1);

        break;

    case ARM_PDM_PEAK_DETECTION_NODE:

        if (arg2) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* To select the peak detect node */
        pdm_peak_detect(PDM_RES->regs, arg1);

        break;

    case ARM_PDM_SAMPLE_ADVANCE:

        if (arg2) {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        /* To select the sample advance */
        pdm_sample_advance(PDM_RES->regs, arg1);

        break;
    }
    return ARM_DRIVER_OK;
}

/**
@fn         int32_t PDMx_Receive(void *data, uint32_t num, PDM_RESOURCES *PDM_RES)
@brief      -> clear and set the fifo clear bit
            -> Store the user enabled channel
            -> Store the fifo watermark value
            -> Enable the PDM IRQ
@param[in]  data     : Pointer storing buffer address
@param[in]  num      : Total number of samples
@param[in]  PDM_RES      : Pointer to PDM resources
@return     ARM_DRIVER_OK : if function return successfully
*/
static int32_t PDMx_Receive(void *data, uint32_t num, PDM_RESOURCES *PDM_RES)
{
    if (PDM_RES->state.initialized == 0) {
        return ARM_DRIVER_ERROR;
    }

    if (PDM_RES->state.powered == 0) {
        return ARM_DRIVER_ERROR;
    }

    /* clear the fifo clear bit */
    pdm_disable_fifo_clear(PDM_RES->regs);

    PDM_RES->transfer.total_cnt = num;
    PDM_RES->transfer.buf       = data;
    PDM_RES->transfer.curr_cnt  = 0;
    PDM_RES->status.rx_busy     = 1;
    PDM_RES->status.rx_overflow = 0;

#if PDM_DMA_ENABLE
    if (PDM_RES->dma_enable) {
        ARM_DMA_PARAMS dma_params;

        if (!PDM_DMA_GenerateOpcode(&PDM_RES->dma_cfg->dma_rx, PDM_RES)) {
            return ARM_DRIVER_ERROR;
        }

        if (PDM_DMA_Usermcode(&PDM_RES->dma_cfg->dma_rx, LocalToGlobal(&PDM_RES->dma_mcode))) {
            return ARM_DRIVER_ERROR;
        }

        /* Start the DMA engine for sending the data to PDM */
        dma_params.peri_reqno = (int8_t) PDM_RES->dma_cfg->dma_rx.dma_periph_req;
        dma_params.dir        = ARM_DMA_DEV_TO_MEM;
        dma_params.cb_event   = PDM_RES->dma_cb;
        /* Enable PDM DMA */
        pdm_dma_enable_irq(PDM_RES->regs);

        /* Each PCM sample is represented by 16-bits resolution (2 bytes) */
        dma_params.num_bytes    = num * 2;
        dma_params.irq_priority = PDM_RES->dma_irq_priority;
        dma_params.burst_len    = 1;
        dma_params.burst_size   = BS_BYTE_4;

        /* Start DMA transfer */
        if (PDM_DMA_Start(&PDM_RES->dma_cfg->dma_rx, &dma_params) != ARM_DRIVER_OK) {
            return ARM_DRIVER_ERROR;
        }
    } else
#endif
    {
#if PDM_BLOCKING_MODE_ENABLE

        /* Check if blocking mode (polling) is enabled */
        if (PDM_RES->blocking_mode) {
            pdm_transfer_t *transfer = &(PDM_RES->transfer);

            /* Block execution until PDM receives all PCM samples */
            pdm_receive_blocking(PDM_RES->regs, transfer);

            /* Check for error detection status */
            if (transfer->status == PDM_ERROR_DETECT) {
                PDM_RES->status.rx_overflow = 1U;
                transfer->status            = PDM_CAPTURE_STATUS_NONE;
            }

            /* Check for capture complete status */
            if (transfer->status == PDM_CAPTURE_STATUS_COMPLETE) {
                PDM_RES->status.rx_busy = 0U;
                transfer->status        = PDM_CAPTURE_STATUS_NONE;
            }
        } else
#endif
        {
            /* Enable irq */
            pdm_enable_irq(PDM_RES->regs);
        }
    }
    return ARM_DRIVER_OK;
}

/* RTE_PDM */
#if RTE_PDM

#if RTE_PDM_DMA_ENABLE
static void              PDM_DMACallback(uint32_t event, int8_t peri_num);
static PDM_DMA_HW_CONFIG PDM0_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(PDM_DMA),
        .dma_periph_req = PDM_DMA_PERIPH_REQ,
        .evtrtr_cfg     = {.instance         = PDM_DMA,
                           .group            = PDM_DMA_GROUP,
                           .channel          = PDM_DMA_PERIPH_REQ,
                           .enable_handshake = PDM_DMA_HANDSHAKE_ENABLE},
    }};
#endif

static PDM_RESOURCES PDM_RES = {.cb_event             = NULL,
                                .regs                 = (PDM_Type *) PDM_BASE,
                                .transfer             = {0},
                                .state                = {0},
                                .instance             = PDM_INSTANCE_PDM0,
                                .fifo_watermark       = RTE_PDM_FIFO_WATERMARK,
                                .status               = {0},
                                .error_irq            = (IRQn_Type) PDM_ERROR_IRQ_IRQn,
                                .warning_irq          = (IRQn_Type) PDM_WARN_IRQ_IRQn,
                                .audio_detect_irq     = (IRQn_Type) PDM_AUDIO_DET_IRQ_IRQn,
                                .error_irq_priority   = (uint32_t) RTE_PDM_IRQ_PRIORITY,
                                .warning_irq_priority = (uint32_t) RTE_PDM_IRQ_PRIORITY,
                                .audio_irq_priority   = (uint32_t) RTE_PDM_IRQ_PRIORITY,
#if RTE_PDM_DMA_ENABLE
                                .dma_cb           = PDM_DMACallback,
                                .dma_cfg          = &PDM0_DMA_HW_CONFIG,
                                .dma_enable       = RTE_PDM_DMA_ENABLE,
                                .dma_irq_priority = RTE_PDM_DMA_IRQ_PRIORITY,
#endif
#if PDM_BLOCKING_MODE_ENABLE
                                .blocking_mode = RTE_PDM_BLOCKING_MODE_ENABLE
#endif
};

/* Function Name: PDM_Initialize */
static int32_t PDM_Initialize(ARM_PDM_SignalEvent_t cb_event)
{
    return PDMx_Initialize(cb_event, &PDM_RES);
}

/* Function Name: PDM_Uninitialize */
static int32_t PDM_Uninitialize(void)
{
    return PDMx_Uninitialize(&PDM_RES);
}

/* Function Name: PDM_PowerControl */
static int32_t PDM_PowerControl(ARM_POWER_STATE state)
{
    return PDMx_PowerControl(state, &PDM_RES);
}

/* Function Name: PDM_Channel_Config */
static int32_t PDM_Channel_Config(PDM_CH_CONFIG *cnfg)
{
    return PDMx_Channel_Config(cnfg, &PDM_RES);
}

/* Function Name: PDM_Control */
static int32_t PDM_Control(uint32_t control, uint32_t arg1, uint32_t arg2)
{
    return PDMx_Control(control, arg1, arg2, &PDM_RES);
}

/* Function Name: PDM_Receive */
static int32_t PDM_Receive(void *data, uint32_t num)
{
    return PDMx_Receive(data, num, &PDM_RES);
}

/*Function Name : PDM_WARNNING_IRQHANDLER */
void PDM_WARN_IRQHandler(void)
{
    PDM_WARNING_IRQ_handler(&PDM_RES);
}

/*Function Name : PDM_ERROR_IRQHandler */
void PDM_ERROR_IRQHandler(void)
{
    PDM_ERROR_IRQ_handler(&PDM_RES);
}

/*Function Name : PDM_AUDIO_DET_IRQHandler */
void PDM_AUDIO_DET_IRQHandler(void)
{
    PDM_AUDIO_DETECT_IRQ_handler(&PDM_RES);
}

/* Function Name: PDM_GetStatus */
static ARM_PDM_STATUS PDM_GetStatus(void)
{
    return PDMx_GetStatus(&PDM_RES);
}

#if RTE_PDM_DMA_ENABLE
void PDM_DMACallback(uint32_t event, int8_t peri_num)
{
    PDMx_DMACallback(event, peri_num, &PDM_RES);
}
#endif

extern ARM_DRIVER_PDM Driver_PDM;
ARM_DRIVER_PDM        Driver_PDM = {
    PDM_GetVersion,
    PDM_GetCapabilities,
    PDM_Initialize,
    PDM_Uninitialize,
    PDM_PowerControl,
    PDM_Control,
    PDM_Channel_Config,
    PDM_Receive,
    PDM_GetStatus
};
#endif /* RTE_PDM */

/* RTE_LPPDM */
#if RTE_LPPDM

#if RTE_LPPDM_DMA_ENABLE

static void              LPPDM_DMACallback(uint32_t event, int8_t peri_num);
static PDM_DMA_HW_CONFIG LPPDM_DMA_HW_CONFIG = {
    .dma_rx = {
        .dma_drv        = &ARM_Driver_DMA_(LPPDM_DMA),
        .dma_periph_req = LPPDM_DMA_PERIPH_REQ,
        .evtrtr_cfg     = {.instance         = LPPDM_DMA,
                           .group            = LPPDM_DMA_GROUP,
                           .channel          = LPPDM_DMA_PERIPH_REQ,
                           .enable_handshake = LPPDM_DMA_HANDSHAKE_ENABLE},
    }};
#endif

static PDM_RESOURCES LPPDM_RES = {.cb_event             = NULL,
                                  .regs                 = (PDM_Type *) LPPDM_BASE,
                                  .transfer             = {0},
                                  .state                = {0},
                                  .instance             = PDM_INSTANCE_LPPDM,
                                  .fifo_watermark       = RTE_LPPDM_FIFO_WATERMARK,
                                  .warning_irq          = (IRQn_Type) LPPDM_IRQ_IRQn,
                                  .warning_irq_priority = (uint32_t) RTE_LPPDM_IRQ_PRIORITY,
#if RTE_LPPDM_DMA_ENABLE
                                  .dma_cb           = LPPDM_DMACallback,
                                  .dma_cfg          = &LPPDM_DMA_HW_CONFIG,
                                  .dma_enable       = RTE_LPPDM_DMA_ENABLE,
                                  .dma_irq_priority = RTE_LPPDM_DMA_IRQ_PRIORITY,
#endif
#if PDM_BLOCKING_MODE_ENABLE
                                  .blocking_mode = RTE_LPPDM_BLOCKING_MODE_ENABLE
#endif
};

/* Function Name: LPPDM_Initialize */
static int32_t LPPDM_Initialize(ARM_PDM_SignalEvent_t cb_event)
{
    return PDMx_Initialize(cb_event, &LPPDM_RES);
}

/* Function Name: LPPDM_Uninitialize */
static int32_t LPPDM_Uninitialize(void)
{
    return PDMx_Uninitialize(&LPPDM_RES);
}

/* Function Name: LPPDM_PowerControl */
static int32_t LPPDM_PowerControl(ARM_POWER_STATE status)
{
    return PDMx_PowerControl(status, &LPPDM_RES);
}

/* Function Name: LPPDM_Control */
static int32_t LPPDM_Control(uint32_t control, uint32_t arg1, uint32_t arg2)
{
    return PDMx_Control(control, arg1, arg2, &LPPDM_RES);
}

/* Function Name: LPPDM_Capture */
static int32_t LPPDM_Receive(void *data, uint32_t num)
{
    return PDMx_Receive(data, num, &LPPDM_RES);
}

/* Function Name: LPPDM_Channel_Config */
static int32_t LPPDM_Channel_Config(PDM_CH_CONFIG *cnfg)
{
    return PDMx_Channel_Config(cnfg, &LPPDM_RES);
}

/*Function Name : LPPDM_IRQHandler */
void LPPDM_IRQHandler(void)
{
    PDM_WARNING_IRQ_handler(&LPPDM_RES);
}

/* Function Name: LPPDM_GetStatus */
static ARM_PDM_STATUS LPPDM_GetStatus(void)
{
    return PDMx_GetStatus(&LPPDM_RES);
}

#if RTE_LPPDM_DMA_ENABLE
void LPPDM_DMACallback(uint32_t event, int8_t peri_num)
{
    PDMx_DMACallback(event, peri_num, &LPPDM_RES);
}
#endif

extern ARM_DRIVER_PDM Driver_LPPDM;
ARM_DRIVER_PDM        Driver_LPPDM = {
    PDM_GetVersion,
    PDM_GetCapabilities,
    LPPDM_Initialize,
    LPPDM_Uninitialize,
    LPPDM_PowerControl,
    LPPDM_Control,
    LPPDM_Channel_Config,
    LPPDM_Receive,
    LPPDM_GetStatus
};
#endif /* RTE_LPPDM */
#endif /* RTE_Drivers_PDM */
