/*
 * Copyright 2026 Arm Limited and/or its affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <string.h>

#include "RTE_Components.h"
#include CMSIS_device_header
#include "RTE_Device.h"

#include "cmsis_vstream.h"
#include "Driver_PDM.h"

#ifndef AUDIO_CFG_CHANNELS
#define AUDIO_CFG_CHANNELS 2U
#endif
#if (AUDIO_CFG_CHANNELS != 1U) && (AUDIO_CFG_CHANNELS != 2U)
#error "AppKit-E8 AudioIn supports one or two channels"
#endif

#ifndef AUDIO_CFG_PDM_CHANNEL
#define AUDIO_CFG_PDM_CHANNEL 4U
#endif
#if (AUDIO_CFG_PDM_CHANNEL != 4U) && (AUDIO_CFG_PDM_CHANNEL != 5U)
#error "AppKit-E8 AudioIn supports PDM channel 4 or 5"
#endif

#if AUDIO_CFG_CHANNELS == 2U
#define AUDIO_CHANNEL_MASK (ARM_PDM_MASK_CHANNEL_4 | ARM_PDM_MASK_CHANNEL_5)
#else
#define AUDIO_CHANNEL_MASK (1U << AUDIO_CFG_PDM_CHANNEL)
#endif
#define AUDIO_FRAME_SIZE (AUDIO_CFG_CHANNELS * sizeof(int16_t))

/* 64-decimation FIR and phase settings from the pack's PDM demo.
 * Use the same filter and gain on both channels for balanced stereo.
 */
#ifndef AUDIO_CFG_PDM_PHASE_4
#define AUDIO_CFG_PDM_PHASE_4 0x1FU
#endif
#ifndef AUDIO_CFG_PDM_PHASE_5
#define AUDIO_CFG_PDM_PHASE_5 0x03U
#endif
#ifndef AUDIO_CFG_PDM_GAIN
#define AUDIO_CFG_PDM_GAIN 0x0DU
#endif

/* The queue is published by the interrupt-driven PDM backend. */
#if RTE_PDM_DMA_ENABLE || RTE_PDM_BLOCKING_MODE_ENABLE
#error "AppKit-E8 AudioIn requires interrupt-driven PDM (blocking mode and DMA disabled)"
#endif

#define FLAGS_INIT       (1U << 0) /* Initialization completed                  */
#define FLAGS_START      (1U << 1) /* Capture start attempted; shutdown is owed */
#define FLAGS_SINGLE     (1U << 2) /* Selected mode, retained after completion  */
#define FLAGS_TERMINATED (1U << 3) /* Explicit Stop/Uninitialize is required    */
#define FLAGS_PDM_INIT   (1U << 4) /* Lower initialization attempted            */
#define FLAGS_PDM_POWER  (1U << 5) /* Lower power-up attempted                  */

extern ARM_DRIVER_PDM Driver_PDM;

/* Stream Buffer Type */
typedef struct {
    uint8_t *data;       /* Data buffer pointer             */
    uint32_t block_num;  /* Number of blocks in data buffer */
    uint32_t block_size; /* Size of block in data buffer    */
} StreamBuf_t;

/* Driver State */
static struct {
    vStreamEvent_t    callback;     /* Callback function for stream events */
    StreamBuf_t       buf;          /* Stream buffer information structure */
    volatile uint32_t idx_in;       /* Current or next capture destination */
    volatile uint32_t idx_get;      /* Next logical FIFO entry to acquire  */
    volatile uint32_t idx_rel;      /* Oldest application acquisition      */
    uint64_t          cnt_pending;  /* Completed logical entries           */
    uint32_t          cnt_acquired; /* Distinct storage blocks acquired    */
    volatile uint8_t  active;       /* Streaming active flag               */
    volatile uint8_t  overflow;     /* Buffer overflow flag                */
    volatile uint8_t  eos;          /* End of stream flag                  */
    volatile uint8_t  flags;        /* Handle status flags                 */
} hAudioIn;

static int32_t Configure(void)
{
    uint32_t      channel;
    uint32_t      phase;
    PDM_CH_CONFIG filter = {.ch_num      = AUDIO_CFG_PDM_CHANNEL,
                            .ch_fir_coef = {0x001,
                                            0x003,
                                            0x003,
                                            0x7F4,
                                            0x004,
                                            0x7ED,
                                            0x7F5,
                                            0x7F4,
                                            0x7D3,
                                            0x7FE,
                                            0x7BC,
                                            0x7E5,
                                            0x7D9,
                                            0x793,
                                            0x029,
                                            0x72C,
                                            0x072,
                                            0x2FD},
                            .ch_iir_coef = 0U};
    hAudioIn.flags |= FLAGS_PDM_POWER;
    if (Driver_PDM.PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK) {
        return VSTREAM_ERROR;
    }
    if ((Driver_PDM.Control(ARM_PDM_SELECT_CHANNEL, AUDIO_CHANNEL_MASK, 0U) != ARM_DRIVER_OK) ||
        (Driver_PDM.Control(ARM_PDM_MODE, ARM_PDM_MODE_AUDIOFREQ_16K_DECM_64, 0U) !=
         ARM_DRIVER_OK) ||
        (Driver_PDM.Control(ARM_PDM_BYPASS_IIR_FILTER, 1U, 0U) != ARM_DRIVER_OK))
    {
        return VSTREAM_ERROR;
    }
    for (channel = 4U; channel <= 5U; channel++) {
        if ((AUDIO_CHANNEL_MASK & (1U << channel)) == 0U) {
            continue;
        }
        filter.ch_num = (uint8_t) channel;
        if (channel == 4U) {
            phase = AUDIO_CFG_PDM_PHASE_4;
        } else {
            phase = AUDIO_CFG_PDM_PHASE_5;
        }
        if ((Driver_PDM.Control(ARM_PDM_CHANNEL_PHASE,
                                channel,
                                phase) !=
             ARM_DRIVER_OK) ||
            (Driver_PDM.Control(ARM_PDM_CHANNEL_GAIN, channel, AUDIO_CFG_PDM_GAIN) !=
             ARM_DRIVER_OK) ||
            (Driver_PDM.Config(&filter) != ARM_DRIVER_OK))
        {
            return VSTREAM_ERROR;
        }
    }
    return VSTREAM_OK;
}

static int32_t ReceiveBlock(void)
{
    uint32_t index = hAudioIn.idx_in * hAudioIn.buf.block_size;
    return Driver_PDM.Receive(&hAudioIn.buf.data[index], hAudioIn.buf.block_size / sizeof(int16_t));
}

/* Test storage against the unreleased logical FIFO window */
static uint32_t BufferOccupied(uint32_t index)
{
    uint32_t distance;

    if (index >= hAudioIn.idx_rel) {
        distance = index - hAudioIn.idx_rel;
    } else {
        distance = hAudioIn.buf.block_num - (hAudioIn.idx_rel - index);
    }
    return distance < (hAudioIn.cnt_pending + hAudioIn.cnt_acquired);
}

/* End progression without cancelling completed entries or acquisitions */
static void CaptureTerminate(void)
{
    hAudioIn.active = 0U;
    hAudioIn.flags |= FLAGS_TERMINATED;
    if ((hAudioIn.flags & FLAGS_PDM_POWER) != 0U) {
        if (Driver_PDM.PowerControl(ARM_POWER_OFF) == ARM_DRIVER_OK) {
            hAudioIn.flags &= ~(FLAGS_PDM_POWER | FLAGS_START);
        }
    }
    hAudioIn.eos = 1U;
    if (hAudioIn.callback != NULL) {
        hAudioIn.callback(VSTREAM_EVENT_EOS);
    }
}

/* PDM IRQ sources have the same priority, so this callback cannot preempt itself */
static void Driver_PDM_Callback(uint32_t event)
{
    if (hAudioIn.active == 0U) {
        return;
    }
    if ((event & ARM_PDM_EVENT_ERROR) != 0U) {
        /* A hardware FIFO error may have left the current block incomplete */
        CaptureTerminate();
    } else if ((event & ARM_PDM_EVENT_CAPTURE_COMPLETE) != 0U) {
        /* Block was filled with audio samples */
        hAudioIn.cnt_pending++;
        hAudioIn.idx_in = (hAudioIn.idx_in + 1U) % hAudioIn.buf.block_num;

        if ((hAudioIn.flags & FLAGS_SINGLE) != 0U) {
            hAudioIn.active = 0U;
        }

        if (hAudioIn.callback != NULL) {
            /* Consumption here may free the next destination before the occupancy test */
            hAudioIn.callback(VSTREAM_EVENT_DATA);
        }

        if (hAudioIn.active != 0U) {
            /* Stop before another completion could overflow the logical entry count */
            if (hAudioIn.cnt_pending >= (UINT64_MAX - hAudioIn.cnt_acquired)) {
                CaptureTerminate();
            } else {
                if (BufferOccupied(hAudioIn.idx_in) != 0U) {
                    hAudioIn.overflow = 1U;
                    if (hAudioIn.callback != NULL) {
                        /* Finish notification before ReceiveBlock enables the overwrite */
                        hAudioIn.callback(VSTREAM_EVENT_OVERFLOW);
                    }
                }
                if (ReceiveBlock() != ARM_DRIVER_OK) {
                    CaptureTerminate();
                }
            }
        }
    }
}

/* Initialize streaming interface */
static int32_t Initialize(vStreamEvent_t callback)
{
    int32_t rval;

    if ((hAudioIn.flags & FLAGS_INIT) != 0U) {
        return VSTREAM_OK;
    }
    if (hAudioIn.flags != 0U) {
        /* Retry Uninitialize after partial initialization or failed cleanup */
        return VSTREAM_ERROR;
    }

    hAudioIn.callback = callback;
    hAudioIn.flags   |= FLAGS_PDM_INIT;

    rval = VSTREAM_ERROR;

    if (Driver_PDM.Initialize(Driver_PDM_Callback) == ARM_DRIVER_OK) {
        rval = Configure();
        if (rval == VSTREAM_OK) {
            hAudioIn.flags |= FLAGS_INIT;
        }
    }

    return rval;
}

/* De-initialize streaming interface */
static int32_t Uninitialize(void)
{
    int32_t rval;

    hAudioIn.active   = 0U;
    hAudioIn.callback = NULL;
    hAudioIn.flags   |= FLAGS_TERMINATED;

    rval = VSTREAM_OK;

    if ((hAudioIn.flags & FLAGS_PDM_POWER) != 0U) {
        if (Driver_PDM.PowerControl(ARM_POWER_OFF) != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hAudioIn.flags &= ~(FLAGS_INIT | FLAGS_PDM_POWER | FLAGS_START);
        }
    }

    if ((rval == VSTREAM_OK) && ((hAudioIn.flags & FLAGS_PDM_INIT) != 0U)) {
        if (Driver_PDM.Uninitialize() != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hAudioIn.flags &= ~FLAGS_PDM_INIT;
        }
    }

    if (rval == VSTREAM_OK) {
        /* Clear the handle after shutdown has made its memory safe to reclaim */
        memset(&hAudioIn, 0, sizeof(hAudioIn));
    }

    return rval;
}

/* Set streaming data buffer */
static int32_t SetBuf(void *buf, uint32_t buf_size, uint32_t block_size)
{
    int32_t rval;

    if (buf == NULL) {
        /* Buffer not specified */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((buf_size == 0U) || (block_size == 0U) || (block_size > buf_size)) {
        /* Invalid buffer or block size */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((((uintptr_t)buf % sizeof(int16_t)) != 0U) || ((block_size % AUDIO_FRAME_SIZE) != 0U)) {
        /* Buffer must be aligned to int16_t and block size must be a multiple of AUDIO_FRAME_SIZE */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if (((hAudioIn.flags & FLAGS_INIT) == 0U)) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioIn.active != 0U) {
        /* Stream is active */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioIn.cnt_acquired != 0U) {
        /* Application still owns blocks */
        rval = VSTREAM_ERROR;
    }
    else if ((hAudioIn.flags & (FLAGS_START | FLAGS_TERMINATED)) != 0U) {
        /* Stream has been started or terminated, call Stop() first */
        rval = VSTREAM_ERROR;
    }
    else {
        rval = VSTREAM_OK;

        /* Set buffer */
        hAudioIn.buf.data       = buf;
        hAudioIn.buf.block_size = block_size;
        hAudioIn.buf.block_num  = buf_size / block_size;

        /* Reset buffer indexes */
        hAudioIn.idx_in       = 0U;
        hAudioIn.idx_get      = 0U;
        hAudioIn.idx_rel      = 0U;
        hAudioIn.cnt_pending  = 0U;
        hAudioIn.cnt_acquired = 0U;

        /* Reset states and flags */
        hAudioIn.overflow = 0U;
        hAudioIn.eos      = 0U;
        hAudioIn.flags   &= FLAGS_INIT | FLAGS_PDM_INIT | FLAGS_PDM_POWER;
    }
    return rval;
}

/* Start streaming */
static int32_t Start(uint32_t mode)
{
    int32_t rval;

    if ((mode != VSTREAM_MODE_SINGLE) && (mode != VSTREAM_MODE_CONTINUOUS)) {
        /* Invalid mode */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((hAudioIn.flags & FLAGS_INIT) == 0) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioIn.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioIn.active != 0U) {
        /* An active stream may only be started again in the same mode */
        if (((hAudioIn.flags & FLAGS_SINGLE) != 0U) == (mode == VSTREAM_MODE_SINGLE)) {
            rval = VSTREAM_OK;
        } else {
            rval = VSTREAM_ERROR;
        }
    }
    else if ((hAudioIn.flags & FLAGS_TERMINATED) != 0U) {
        /* Stream has been terminated, call Stop() to reset */
        rval = VSTREAM_ERROR;
    }
    else if (BufferOccupied(hAudioIn.idx_in) != 0U) {
        /* An inactive stream starts only with a reusable first destination */
        rval = VSTREAM_ERROR;
    }
    else {
        rval = VSTREAM_OK;

        if ((hAudioIn.flags & FLAGS_PDM_POWER) == 0U) {
            rval = Configure();
        }

        if (rval == VSTREAM_OK) {
            if (mode == VSTREAM_MODE_SINGLE) {
                hAudioIn.flags |= FLAGS_SINGLE;
            } else {
                hAudioIn.flags &= ~FLAGS_SINGLE;
            }

            /* Mark the stream as active and indicate that capture has started */
            hAudioIn.active = 1U;
            hAudioIn.flags |= FLAGS_START;

            if (ReceiveBlock() != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }
        }

        if (rval != VSTREAM_OK) {
            /* Error, clear active flag and mark stream for termination */
            hAudioIn.active = 0U;
            hAudioIn.flags |= FLAGS_TERMINATED;
        }
    }
    return rval;
}

/* Stop streaming, cancel acquisitions and discard incomplete entries */
static int32_t Stop(void)
{
    int32_t rval;
    vStreamEvent_t cb;

    if ((hAudioIn.flags & FLAGS_INIT) == 0U) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else {
        /* Disable event callback */
        cb = hAudioIn.callback;
        hAudioIn.callback = NULL;

        /* Suppress rearming */
        hAudioIn.active = 0U;
        hAudioIn.flags |= FLAGS_TERMINATED;

        rval = VSTREAM_OK;
        if ((hAudioIn.flags & FLAGS_PDM_POWER) != 0U) {
            if (Driver_PDM.PowerControl(ARM_POWER_OFF) != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            } else {
                hAudioIn.flags &= ~FLAGS_PDM_POWER;
            }
        }

        /* Restore event callback */
        hAudioIn.callback = cb;

        if (rval == VSTREAM_OK) {
            /* Stop discards buffered data while preserving the buffer configuration */
            hAudioIn.idx_in       = 0U;
            hAudioIn.idx_get      = 0U;
            hAudioIn.idx_rel      = 0U;
            hAudioIn.cnt_pending  = 0U;
            hAudioIn.cnt_acquired = 0U;

            hAudioIn.flags &= FLAGS_INIT | FLAGS_PDM_INIT;
        }
    }
    return rval;
}

/* Get pointer to a writable data block */
static void *GetBlock(void)
{
    uint32_t primask;
    void *block;

    primask = __get_PRIMASK();
    __disable_irq();

    if (hAudioIn.buf.data == NULL) {
        /* Buffer not set */
        block = NULL;
    }
    else if (hAudioIn.cnt_pending == 0U) {
        /* No completed FIFO entries are waiting to be acquired */
        block = NULL;
    }
    else if (hAudioIn.cnt_acquired == hAudioIn.buf.block_num) {
        /* All physical blocks are acquired; pending entries must wait for release */
        block = NULL;
    }
    else {
        block = &hAudioIn.buf.data[hAudioIn.idx_get * hAudioIn.buf.block_size];

        /* Advance one logical entry */
        hAudioIn.idx_get = (hAudioIn.idx_get + 1U) % hAudioIn.buf.block_num;
        hAudioIn.cnt_pending--;
        hAudioIn.cnt_acquired++;
    }

    __set_PRIMASK(primask);

    return block;
}

/* Release the oldest acquired block */
static int32_t ReleaseBlock(void)
{
    uint32_t primask;
    int32_t rval;

    primask = __get_PRIMASK();
    __disable_irq();

    if (hAudioIn.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioIn.cnt_acquired == 0U) {
        /* No blocks have been acquired */
        rval = VSTREAM_ERROR;
    }
    else {
        rval = VSTREAM_OK;

        /* Increment index of the oldest acquired block */
        hAudioIn.idx_rel = (hAudioIn.idx_rel + 1U) % hAudioIn.buf.block_num;

        /* Decrement the number of acquired blocks */
        hAudioIn.cnt_acquired--;
    }

    __set_PRIMASK(primask);

    return rval;
}

/* Get Interface status */
static vStreamStatus_t GetStatus(void)
{
    uint32_t primask;
    vStreamStatus_t status = {0};

    primask = __get_PRIMASK();
    __disable_irq();

    /* Get status */
    status.active   = hAudioIn.active;
    status.overflow = hAudioIn.overflow;
    status.eos      = hAudioIn.eos;

    hAudioIn.overflow = 0U;
    hAudioIn.eos      = 0U;

    __set_PRIMASK(primask);
    return status;
}

/* Driver access structure */
vStreamDriver_t Driver_vStreamAudioIn = {
    Initialize,
    Uninitialize,
    SetBuf,
    Start,
    Stop,
    GetBlock,
    ReleaseBlock,
    GetStatus
};
