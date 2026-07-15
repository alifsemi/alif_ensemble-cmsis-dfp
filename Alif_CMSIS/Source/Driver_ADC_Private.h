/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

#ifndef DRIVER_ADC_PRIVATE_H_
#define DRIVER_ADC_PRIVATE_H_

/*---System include ----*/
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_ADC.h"
#include "adc.h"
#include "sys_ctrl_adc.h"

/* Check whether any ADC instance opts into DMA. */
#if (RTE_ADC120_DMA_ENABLE || RTE_ADC121_DMA_ENABLE || \
     RTE_ADC122_DMA_ENABLE || RTE_ADC24_DMA_ENABLE)
#define ADC_DMA_ENABLE 1
#else
#define ADC_DMA_ENABLE 0
#endif

#if ADC_DMA_ENABLE
#include <DMA_Common.h>

/* ADC DMA path — chunked ping-pong
 *
 * Two mcode shapes exist, selected by the SoC-family capability macro
 * SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT:
 *
 *  - Legacy : fully-unrolled 46 B per-sample block. Each sample
 *    step: FLUSHP + WFP + intr-clear preamble + single DMAST of the 4 B
 *    ADC_SAMPLE_REG_ word. The mcode also holds a 4 B constant table
 *    (ADC_INTERRUPT W1C value) at [0..3] and a DMASEV+DMAEND footer.
 *
 *  - Slim (SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT = 1):
 *    ADC_CONTROL.SAMPLE_INT_CLR auto-clears the pending interrupt bits on
 *    every sample-reg read, so the intr-clear preamble is unnecessary. A
 *    one-time header (MOV CCR + MOV DAR) precedes a DMALP LC1 × LC0 loop
 *    wrapping a per-channel unrolled body of FLUSHP + WFP + MOV SAR +
 *    LDPS + ST. Total mcode is (28 + 13 * num_ch) B regardless of
 *    chunk_samples; the only per-arm patch is the 4 B DAR immediate in
 *    the header.
 *
 * Buffers hold raw uint32_t sample words for both ADC12 and ADC24.
 *
 * Legacy chunk sizing:
 *     chunk ~= (mcode_size - ADC_DMA_CONST_TABLE_BYTES
 *                          - ADC_DMA_FOOTER_BYTES)
 *              / ADC_DMA_SAMPLE_BLOCK_BYTES
 * Reference points (default 4 KB mcode):
 *     1 KB mcode  ->  ~22 samples/chunk
 *     4 KB mcode  ->  ~88 samples/chunk
 *    16 KB mcode  ->  ~352 samples/chunk
 *
 * Each instance's buffer size is set by RTE_ADCxxx_DMA_MCODE_SIZE
 * in RTE_Device.h.
 */
#if SOC_FEAT_ADC_HAS_SAMPLE_INT_CLR_BIT
#define ADC_DMA_CONST_TABLE_BYTES     0U    /* no data prelude — entry point is at [0] */
#define ADC_DMA_FOOTER_BYTES          8U    /* DMAWMB + DMASEV + DMAEND + headroom     */
#define ADC_DMA_HEADER_DAR_OFFSET     8U    /* DAR imm inside the one-time header      */
#define ADC_DMA_LOOP_MAX              256U  /* DMALP iterations per level              */
#else
#define ADC_DMA_CONST_TABLE_BYTES     4U    /* mcode[0..3] = ADC_INTR_DONE0_CLEAR */
#define ADC_DMA_FOOTER_BYTES          8U    /* DMASEV(2) + DMAEND(1) + headroom   */
#define ADC_DMA_SAMPLE_BLOCK_BYTES    46U   /* FLUSHP + WFP + intr-clear + sample */
#define ADC_DMA_DAR_OFFSET_IN_BLOCK   39U   /* DAR imm inside the sample-read     */
#endif

typedef struct _ADC_DMA_HW_CONFIG {
    DMA_PERIPHERAL_CONFIG dma_rx; /* DMA Rx interface (ADC -> memory) */
} ADC_DMA_HW_CONFIG;

/* Runtime state for the chunked DMA ping-pong path. Kept out of the HAL
 * `conv_info_t` (drivers/include/adc.h) since the HAL never reads these;
 * only Driver_ADC.c's DMA code manages them.
 *
 * chunk_samples / chunks_per_buf are derived at StartN from active_ch,
 * samples_per_buf, and the mcode budget. chunk_idx tracks progress inside
 * the active app buffer. `armed` is set after every successful DMA start
 * and cleared at DMA-complete entry / by Stop() after ADC_DMA_Stop; it
 * guards the partial-compact so a Stop() called from the buffer-boundary
 * callback (after the flip but before the re-arm) does not compact stale
 * data on the freshly-flipped-to buffer.
 */
typedef struct _ADC_DMA_STATE {
    volatile uint32_t chunk_samples;
    volatile uint32_t chunks_per_buf;
    volatile uint32_t chunk_idx;
    volatile uint8_t  armed;
} ADC_DMA_STATE;
#endif

typedef enum {
    ADC_FLAG_DRV_INIT_DONE  = (1U << 0), /* ADC Driver is Initialized */
    ADC_FLAG_DRV_POWER_DONE = (1U << 1), /* ADC Driver is Powered     */
} ADC_FLAG_Type;

/* Access structure for the saving the ADC Setting and status*/
typedef struct _ADC_RESOURCES {
    ARM_ADC_SignalEvent_t cb_event;     /* ADC APPLICATION CALLBACK EVENT                       */
    ADC_Type             *regs;         /* ADC register base address                            */
    conv_info_t           conv;         /* ADC conversion information                           */
    ADC_INSTANCE          drv_instance; /* ADC Driver instances                                 */
    IRQn_Type intr_done0_irq_num;       /* ADC avg sample ready interrupt number                */
    IRQn_Type intr_done1_irq_num;       /* ADC all sample taken interrupt number                */
    IRQn_Type intr_cmpa_irq_num;        /* ADC comparator A interrupt number                    */
    IRQn_Type intr_cmpb_irq_num;        /* ADC comparator B interrupt number                    */
    uint8_t   ext_trig_val;             /* ADC external trigger enable value                    */
    uint8_t   busy;                     /* ADC conversion busy flag                             */
    uint32_t  intr_done0_irq_priority;  /* ADC done0 Irq Priority                               */
    uint32_t  intr_done1_irq_priority;  /* ADC done1 Irq Priority                               */
    uint32_t  intr_cmpa_irq_priority;   /* ADC cmpa Irq Priority                                */
    uint32_t  intr_cmpb_irq_priority;   /* ADC cmpb Irq Priority                                */
    uint32_t  state;                    /* ADC state                                            */
    uint32_t  clock_div;                /* ADC clock divisor                                    */
    uint32_t  avg_sample_num;           /* ADC average sample number                            */
    uint32_t  sample_width;             /* ADC sample width                                     */
    uint32_t  shift_n_bit;              /* ADC number of bits to shift                          */
    uint32_t  shift_left_or_right;      /* ADC shift bit left or right                          */
    bool      differential_enable;      /* ADC12 differential enable                            */
    bool      comparator_enable;        /* ADC12 comparator enable                              */
    uint8_t   comparator_bias;          /* ADC12 comparator bias                                */
    uint32_t  pga_enable;               /* ADC Programmable gain amplifier(PGA) enable          */
    uint32_t  pga_value;                /* ADC Programmable gain amplifier(PGA)                 */
    uint32_t  bias;                     /* ADC24 bias control value                             */
    uint32_t  output_rate;              /* ADC24 output rate                                    */
    uint32_t  active_channels;          /* Enabled-channels bitfield                            */
#if ADC_DMA_ENABLE
    const bool            dma_enable;       /* ADC DMA enable (per instance)                      */
    ARM_DMA_SignalEvent_t dma_cb;           /* DMA-complete callback                              */
    ADC_DMA_HW_CONFIG    *dma_cfg;          /* DMA controller wiring                              */
    const uint32_t        dma_irq_priority; /* DMA IRQ priority                                   */
    uint8_t              *dma_mcode;        /* Per-instance PL330 mcode scratch                   */
    const uint32_t        mcode_size;       /* Per-instance mcode buffer size (bytes)             */
    ADC_DMA_STATE         dma_state;        /* Runtime chunk tracking + armed flag                */
#endif
} ADC_RESOURCES;

#endif /* DRIVER_ADC_PRIVATE_H_ */
