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
#include "Driver_SAI.h"
#include "WM8904_driver.h"
#include "WM8904_codec_i2c.h"
#include "WM8904_Private.h"
#include "board_defs.h"
#include "vstream_audio_out_config.h"

#if RTE_I2S2_BLOCKING_MODE_ENABLE || RTE_I2S2_DMA_ENABLE
#error "AppKit-E8 AudioOut requires interrupt-driven I2S2 (blocking mode and DMA disabled)"
#endif

#define FLAGS_INIT        (1U << 0) /* Initialization completed                   */
#define FLAGS_START       (1U << 1) /* Playback start attempted; shutdown is owed */
#define FLAGS_SINGLE      (1U << 2) /* Selected mode, retained after completion   */
#define FLAGS_SAI_INIT    (1U << 3) /* Lower initialization attempted             */
#define FLAGS_SAI_POWER   (1U << 4) /* Lower power-up attempted                   */
#define FLAGS_CODEC_INIT  (1U << 5) /* Codec initialization attempted             */
#define FLAGS_CODEC_POWER (1U << 6) /* Codec power-up attempted                   */
#define FLAGS_TERMINATED  (1U << 7) /* Explicit Stop/Uninitialize is required     */

#define SEND_NONE    0U
#define SEND_PAYLOAD 1U
#define SEND_SILENCE 2U

/* Low Level Driver Instances */
extern ARM_DRIVER_SAI     ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);
extern ARM_DRIVER_I2C     ARM_Driver_I2C_(RTE_WM8904_CODEC_I2C_INSTANCE);
extern ARM_DRIVER_WM8904  WM8904;
static ARM_DRIVER_SAI    *DriverSAI = &ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);
static ARM_DRIVER_WM8904 *DriverCodec = &WM8904;

/* Register access uses the I2C callback installed by the codec driver. */
static WM8904_CODEC_SLAVE_I2C_CONFIG CodecI2C = {
    .drv_i2c                 = &ARM_Driver_I2C_(RTE_WM8904_CODEC_I2C_INSTANCE),
    .bus_speed               = ARM_I2C_BUS_SPEED_STANDARD,
    .wm8904_codec_slave_addr = WM8904_SLAVE
};

#if VSTREAM_AUDIO_OUT_SAMPLE_BITS == 16U
#define AUDIO_SAMPLE_SIZE ((uint32_t)sizeof(int16_t))
#define AUDIO_CODEC_WL    0U
#elif VSTREAM_AUDIO_OUT_SAMPLE_BITS == 24U
#define AUDIO_SAMPLE_SIZE ((uint32_t)sizeof(int32_t))
#define AUDIO_CODEC_WL    2U
#else
#define AUDIO_SAMPLE_SIZE ((uint32_t)sizeof(int32_t))
#define AUDIO_CODEC_WL    3U
#endif
#define AUDIO_FRAME_SIZE (2U * AUDIO_SAMPLE_SIZE)

/* SYSCLK = MCLK = 12.288 MHz */
#if VSTREAM_AUDIO_OUT_SAMPLE_RATE == 8000U
#define AUDIO_CODEC_RATE ((9U << 10) | 0U) /* SYSCLK/fs = 1536 */
#elif VSTREAM_AUDIO_OUT_SAMPLE_RATE == 16000U
#define AUDIO_CODEC_RATE ((6U << 10) | 2U) /* SYSCLK/fs = 768  */
#else
#define AUDIO_CODEC_RATE ((3U << 10) | 5U) /* SYSCLK/fs = 256  */
#endif

/* Stream Buffer Type */
typedef struct {
    uint8_t *data;       /* Data buffer pointer             */
    uint32_t block_num;  /* Number of blocks in data buffer */
    uint32_t block_size; /* Size of block in data buffer    */
} StreamBuf_t;

/* Driver State */
static struct {
    vStreamEvent_t    callback;     /* Callback function for stream events         */
    StreamBuf_t       buf;          /* Stream buffer information structure         */
    volatile uint32_t idx_get;      /* Next block acquired by the application      */
    volatile uint32_t idx_rel;      /* Next acquired block submitted for output    */
    volatile uint32_t idx_out;      /* Oldest queued block, including the transfer */
    volatile uint32_t owned;        /* Blocks acquired but not yet submitted       */
    volatile uint32_t queued;       /* Blocks submitted, including the transfer    */
    volatile uint8_t  active;       /* Streaming active flag                       */
    volatile uint8_t  underflow;    /* Buffer underflow flag                       */
    volatile uint8_t  eos;          /* End of stream flag                          */
    volatile uint8_t  sending;      /* Current transfer type                       */
    volatile uint8_t  flags;        /* Handle status flags                         */
} hAudioOut;

/* Substitute output keeps the interrupt path alive during continuous underflow.
 * Each silence transfer contains 64 stereo frames and produces no DATA event.
 */
static const uint32_t audio_silence[(64U * AUDIO_FRAME_SIZE) / sizeof(uint32_t)] = {0};

/* Start committed output, or a bounded silence transfer during underflow. */
static int32_t SendBlock(void)
{
    int32_t status;
    uint32_t buf_index;

    if (hAudioOut.queued != 0U) {
        buf_index = hAudioOut.idx_out * hAudioOut.buf.block_size;
        hAudioOut.sending = SEND_PAYLOAD;
        status = DriverSAI->Send(&hAudioOut.buf.data[buf_index], hAudioOut.buf.block_size / AUDIO_SAMPLE_SIZE);
    }
    else {
        hAudioOut.sending = SEND_SILENCE;
        status = DriverSAI->Send(audio_silence, sizeof(audio_silence) / AUDIO_SAMPLE_SIZE);
    }
    if (status != ARM_DRIVER_OK) {

        /* Reset states and flags */
        hAudioOut.sending = SEND_NONE;
        return VSTREAM_ERROR;
    }

    return VSTREAM_OK;
}

/* These SAI controls do not block in the required interrupt-driven configuration. */
static int32_t StopTransfer(void)
{
    int32_t rval;

    rval = VSTREAM_OK;

    if ((hAudioOut.flags & FLAGS_START) != 0U) {
        if (DriverSAI->Control(ARM_SAI_ABORT_SEND, 0U, 0U) != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        }
        /* Try to disable TX even when aborting the current transfer failed. */
        if (DriverSAI->Control(ARM_SAI_CONTROL_TX, 0U, 0U) != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        }
        if (rval == VSTREAM_OK) {
            hAudioOut.flags &= ~FLAGS_START;
        }
    }
    return rval;
}

/* Termination retains all acquisitions and queued payloads until explicit cleanup. */
static void PlaybackTerminate(void)
{
    hAudioOut.active  = 0U;
    hAudioOut.sending = SEND_NONE;
    hAudioOut.flags  |= FLAGS_TERMINATED;

    (void)StopTransfer();

    hAudioOut.eos = 1U;

    if (hAudioOut.callback != NULL) {
        hAudioOut.callback(VSTREAM_EVENT_EOS);
    }
}

/* Lower SAI callbacks originate from one serialized interrupt source. */
static void DriverSAI_Callback(uint32_t cb_event)
{
    uint32_t event;

    if (hAudioOut.active == 0U) {
        return;
    }
    if ((cb_event & ARM_SAI_EVENT_FRAME_ERROR) != 0U) {
        PlaybackTerminate();
    }
    else if (((cb_event & ARM_SAI_EVENT_SEND_COMPLETE) != 0U) && (hAudioOut.sending != SEND_NONE)) {
        event = 0U;

        if (hAudioOut.sending == SEND_PAYLOAD) {
            /* SAI has read the block; remaining samples may still drain from its FIFO. */
            hAudioOut.idx_out = (hAudioOut.idx_out + 1U) % hAudioOut.buf.block_num;
            hAudioOut.queued--;
            event = VSTREAM_EVENT_DATA;
            if ((hAudioOut.flags & FLAGS_SINGLE) != 0U) {
                hAudioOut.active = 0U;
            }
        }

        hAudioOut.sending = SEND_NONE;

        if ((hAudioOut.callback != NULL) && (event != 0U)) {
            /* Callback commitment can supply the next payload before the demand test. */
            hAudioOut.callback(event);
        }

        if (hAudioOut.active != 0U) {
            if (hAudioOut.queued == 0U) {
                /* Another transfer is required; use silence if no payload was committed. */
                hAudioOut.underflow = 1U;
                if (hAudioOut.callback != NULL) {
                    hAudioOut.callback(VSTREAM_EVENT_UNDERFLOW);
                }
            }
            if (SendBlock() != VSTREAM_OK) {
                PlaybackTerminate();
            }
        }
    }
    else if (((cb_event & ARM_SAI_EVENT_TX_UNDERFLOW) != 0U) && ((hAudioOut.flags & FLAGS_SINGLE) == 0U)) {
        hAudioOut.underflow = 1U;

        if (hAudioOut.callback != NULL) {
            hAudioOut.callback(VSTREAM_EVENT_UNDERFLOW);
        }
    }
}

/* Configure digital playback after the pack's headphone power-up sequence. */
static int32_t ConfigureCodec(void)
{
    uint16_t clocks;

    if (WM8904_codec_i2c_read(&CodecI2C, WM8904_CLOCK_RATES2, &clocks) != ARM_DRIVER_OK) {
        return VSTREAM_ERROR;
    }

    /* Disable SYSCLK while changing clock configuration (Rev 4.1, p.100). */
    if ((WM8904_codec_i2c_write(&CodecI2C, WM8904_CLOCK_RATES2, clocks & ~(1U << 2)) != ARM_DRIVER_OK) ||
        (WM8904_codec_i2c_write(&CodecI2C, WM8904_CLOCK_RATES0, 0U) != ARM_DRIVER_OK) ||
        (WM8904_codec_i2c_write(&CodecI2C, WM8904_CLOCK_RATES1, AUDIO_CODEC_RATE) != ARM_DRIVER_OK) ||
        (WM8904_codec_i2c_write(&CodecI2C, WM8904_AUDIO_INTERFACE1,
                              (AUDIO_CODEC_WL << 2) | 2U) != ARM_DRIVER_OK) ||
        (WM8904_codec_i2c_write(&CodecI2C, WM8904_CLOCK_RATES2, clocks) != ARM_DRIVER_OK) ||
        (DriverCodec->SetVolume(VSTREAM_AUDIO_OUT_VOLUME) != ARM_DRIVER_OK)) {
        return VSTREAM_ERROR;
    }
    return VSTREAM_OK;
}

/* Initialize streaming interface */
static int32_t Initialize(vStreamEvent_t callback)
{
    int32_t rval;

    if ((hAudioOut.flags & FLAGS_INIT) != 0U) {
        return VSTREAM_OK;
    }
    if (hAudioOut.flags != 0U) {
        return VSTREAM_ERROR;
    }

    hAudioOut.callback = callback;
    hAudioOut.flags  |= FLAGS_CODEC_INIT;

    rval = VSTREAM_ERROR;

    if (DriverCodec->Initialize() == ARM_DRIVER_OK) {
        hAudioOut.flags |= FLAGS_CODEC_POWER;

        if ((DriverCodec->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK) &&
            (ConfigureCodec() == VSTREAM_OK)) {
            hAudioOut.flags |= FLAGS_SAI_INIT;

            if (DriverSAI->Initialize(DriverSAI_Callback) == ARM_DRIVER_OK) {
                hAudioOut.flags |= FLAGS_SAI_POWER;

                if (DriverSAI->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK) {
                    if (DriverSAI->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                           ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(VSTREAM_AUDIO_OUT_SAMPLE_BITS),
                                           VSTREAM_AUDIO_OUT_SAMPLE_BITS * 2U,
                                           VSTREAM_AUDIO_OUT_SAMPLE_RATE) == ARM_DRIVER_OK) {
                        hAudioOut.flags |= FLAGS_INIT;
                        rval = VSTREAM_OK;
                    }
                }
            }
        }
    }

    return rval;
}

/* De-initialize streaming interface */
static int32_t Uninitialize(void)
{
    int32_t rval;

    hAudioOut.active   = 0U;
    hAudioOut.callback = NULL;
    hAudioOut.sending  = SEND_NONE;
    hAudioOut.flags   |= FLAGS_TERMINATED;

    rval = StopTransfer();
    if ((rval == VSTREAM_OK) && ((hAudioOut.flags & FLAGS_SAI_POWER) != 0U)) {
        if (DriverSAI->PowerControl(ARM_POWER_OFF) != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hAudioOut.flags &= ~(FLAGS_INIT | FLAGS_SAI_POWER);
        }
    }

    if ((rval == VSTREAM_OK) && ((hAudioOut.flags & FLAGS_SAI_INIT) != 0U)) {
        if (DriverSAI->Uninitialize() != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hAudioOut.flags &= ~FLAGS_SAI_INIT;
        }
    }

    if ((rval == VSTREAM_OK) && ((hAudioOut.flags & FLAGS_CODEC_INIT) != 0U)) {
        /* A codec transfer error can close I2C. Reopen it so cleanup remains retryable. */
        if (WM8904_codec_i2c_init(&CodecI2C) != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        }
    }

    if ((rval == VSTREAM_OK) && ((hAudioOut.flags & FLAGS_CODEC_POWER) != 0U)) {
        /* The pack's PowerControl(OFF) skips powered codecs; reset explicitly. */
        if (WM8904_codec_i2c_write(&CodecI2C, WM8904_SW_RESET_ID, 0U) != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hAudioOut.flags &= ~FLAGS_CODEC_POWER;
        }
    }

    if ((rval == VSTREAM_OK) && ((hAudioOut.flags & FLAGS_CODEC_INIT) != 0U)) {
        if (DriverCodec->Uninitialize() != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hAudioOut.flags &= ~FLAGS_CODEC_INIT;
        }
    }

    if (rval == VSTREAM_OK) {
        /* Clear the handle after shutdown has made its memory safe to reclaim */
        memset(&hAudioOut, 0, sizeof(hAudioOut));
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
    else if ((((uintptr_t)buf % AUDIO_SAMPLE_SIZE) != 0U) || ((block_size % AUDIO_FRAME_SIZE) != 0U)) {
        /* Buffer or block size not aligned */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if (((hAudioOut.flags & FLAGS_INIT) == 0U)) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioOut.active != 0U) {
        /* Stream is active */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioOut.owned != 0U) {
        /* Application still owns blocks */
        rval = VSTREAM_ERROR;
    }
    else if ((hAudioOut.flags & (FLAGS_START | FLAGS_TERMINATED)) != 0U) {
        /* Stream has been started or terminated, call Stop() first */
        rval = VSTREAM_ERROR;
    }
    else {
        rval = VSTREAM_OK;

        /* Set buffer */
        hAudioOut.buf.data       = buf;
        hAudioOut.buf.block_size = block_size;
        hAudioOut.buf.block_num  = buf_size / block_size;

        /* Reset buffer indexes */
        hAudioOut.idx_get        = 0U;
        hAudioOut.idx_rel        = 0U;
        hAudioOut.idx_out        = 0U;
        hAudioOut.owned          = 0U;
        hAudioOut.queued         = 0U;
        hAudioOut.sending        = SEND_NONE;

        /* Reset states and flags */
        hAudioOut.underflow      = 0U;
        hAudioOut.eos            = 0U;
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
    else if ((hAudioOut.flags & FLAGS_INIT) == 0U) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioOut.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioOut.active != 0U) {
        /* An active stream may only be started again in the same mode */
        if (((hAudioOut.flags & FLAGS_SINGLE) != 0U) == (mode == VSTREAM_MODE_SINGLE)) {
            rval = VSTREAM_OK;
        } else {
            rval = VSTREAM_ERROR;
        }
    }
    else if ((hAudioOut.flags & FLAGS_TERMINATED) != 0U) {
        /* Stream has been terminated, call Stop() to reset */
        rval = VSTREAM_ERROR;
    }
    else {
        if (mode == VSTREAM_MODE_SINGLE) {
            hAudioOut.flags |= FLAGS_SINGLE;
        } else {
            hAudioOut.flags &= ~FLAGS_SINGLE;
        }

        hAudioOut.active = 1U;

        rval = VSTREAM_OK;
        if ((hAudioOut.flags & FLAGS_START) == 0U) {
            hAudioOut.flags |= FLAGS_START;
            if (DriverSAI->Control(ARM_SAI_CONTROL_TX, 1U, 0U) != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }
        }

        if (rval == VSTREAM_OK) {
            rval = SendBlock();
        }

        if (rval != VSTREAM_OK) {
            hAudioOut.active  = 0U;
            hAudioOut.sending = SEND_NONE;
            hAudioOut.flags  |= FLAGS_TERMINATED;
            (void)StopTransfer();
        }
    }
    return rval;
}

/* Stop streaming, cancel acquisitions and discard incomplete entries */
static int32_t Stop(void)
{
    int32_t rval;
    vStreamEvent_t cb;

    if ((hAudioOut.flags & FLAGS_INIT) == 0U) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else {
        /* Disable event callback */
        cb = hAudioOut.callback;
        hAudioOut.callback = NULL;

        /* Suppress rearming */
        hAudioOut.active  = 0U;
        hAudioOut.sending = SEND_NONE;
        hAudioOut.flags  |= FLAGS_TERMINATED;

        rval = StopTransfer();

        /* Restore event callback */
        hAudioOut.callback = cb;

        if (rval == VSTREAM_OK) {
            hAudioOut.idx_get = 0U;
            hAudioOut.idx_rel = 0U;
            hAudioOut.idx_out = 0U;
            hAudioOut.owned   = 0U;
            hAudioOut.queued  = 0U;
            hAudioOut.flags  &= FLAGS_INIT | FLAGS_SAI_INIT | FLAGS_SAI_POWER | FLAGS_CODEC_INIT | FLAGS_CODEC_POWER;
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

    if (hAudioOut.buf.data == NULL) {
        /* Buffer not set */
        block = NULL;
    }
    else if ((hAudioOut.owned + hAudioOut.queued) >= hAudioOut.buf.block_num) {
        /* All physical blocks are owned or queued; cannot acquire more */
        block = NULL;
    }
    else {
        block = &hAudioOut.buf.data[hAudioOut.idx_get * hAudioOut.buf.block_size];

        /* Advance index for the next acquisition */
        hAudioOut.idx_get = (hAudioOut.idx_get + 1U) % hAudioOut.buf.block_num;
        hAudioOut.owned++;
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

    if (hAudioOut.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    }
    else if (hAudioOut.owned == 0U) {
        /* No blocks are currently owned */
        rval = VSTREAM_ERROR;
    }
    else {
        rval = VSTREAM_OK;

        /* Increment index of the oldest owned block */
        hAudioOut.idx_rel = (hAudioOut.idx_rel + 1U) % hAudioOut.buf.block_num;

        /* Decrement the number of owned blocks and increment the queued blocks */
        hAudioOut.owned--;
        hAudioOut.queued++;
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
    status.active    = hAudioOut.active;
    status.underflow = hAudioOut.underflow;
    status.eos       = hAudioOut.eos;

    /* Clear status */
    hAudioOut.underflow = 0U;
    hAudioOut.eos       = 0U;

    __set_PRIMASK(primask);

    return status;
}

/* Driver access structure */
vStreamDriver_t Driver_vStreamAudioOut = {
    Initialize,
    Uninitialize,
    SetBuf,
    Start,
    Stop,
    GetBlock,
    ReleaseBlock,
    GetStatus
};
