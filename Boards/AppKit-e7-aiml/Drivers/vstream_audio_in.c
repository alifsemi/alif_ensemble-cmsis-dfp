/*---------------------------------------------------------------------------
 * Copyright (c) 2025 Arm Limited (or its affiliates).
 * All rights reserved.
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
 *---------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include "cmsis_vstream.h"
#include "Driver_SAI.h"

/* Connect to hardware via Driver_SAI#                 */
/* Select driver control block for hardware interface. */
#ifndef AUDIO_CFG_SAI_INSTANCE
#define AUDIO_CFG_SAI_INSTANCE 3
#endif

/* Number of bits per sample <0=>8 <1=>16 <2=>24 <3=>32  */
/* Defines number of bits of information in each sample. */
#ifndef AUDIO_CFG_BIT_DEPTH
#define AUDIO_CFG_BIT_DEPTH 16
#endif

/* Number of samples per second */
/* Defines audio sampling rate. */
#ifndef AUDIO_CFG_SAMPLE_RATE
#define AUDIO_CFG_SAMPLE_RATE 16000U
#endif

/* SAI Mode Definition */
#define AUDIO_CFG_SAI_MODE     ARM_SAI_MODE_MASTER
#define AUDIO_CFG_SAI_SYNC     ARM_SAI_ASYNCHRONOUS
#define AUDIO_CFG_SAI_PROTOCOL ARM_SAI_PROTOCOL_I2S

/* Definitions */
#define AUDIO_FLAGS_INIT       (1U << 0)
#define AUDIO_FLAGS_SINGLE     (1U << 1)

/* Low Level Driver Instance */
extern ARM_DRIVER_SAI ARM_Driver_SAI_(AUDIO_CFG_SAI_INSTANCE);

typedef struct audio_buf_s {
    void    *data;       /* Data buffer pointer     */
    uint32_t block_num;  /* Number of data blocks   */
    uint32_t block_size; /* Data block size         */
} AudioBuf_t;

typedef struct audio_cb_s {
    ARM_DRIVER_SAI   *drv;      /* SAI driver instance     */
    vStreamEvent_t    callback; /* Audio IN callback       */
    vStreamStatus_t   status;   /* Audio IN stream status  */
    AudioBuf_t        buf;      /* Receive buffer info     */
    volatile uint32_t idx_in;   /* Index of block currently receiving data            */
    volatile uint32_t idx_rel;  /* Index of block to be released on ReleaseBlock call */
    volatile uint32_t idx_get;  /* Index of block to be returned on GetBlock call     */
    uint8_t           flags;    /* Audio IN stream control flags                      */
} AudioInHandle_t;

/* Driver Control Block */
static AudioInHandle_t hAudioIn;

/**
  \brief Callback routine from the SAI driver.

  \param[in]  event  Event for which the callback has been called.
*/
static void Driver_SAI_Callback(uint32_t event)
{
    uint32_t buf_block;
    uint32_t buf_index;
    uint32_t num;

    if (event & ARM_SAI_EVENT_RECEIVE_COMPLETE) {
        if (hAudioIn.flags & AUDIO_FLAGS_SINGLE) {
            /* Single mode: Stop SAI receiver */
            hAudioIn.drv->Control(ARM_SAI_CONTROL_RX, 0U, 0U);
        }

        /* Increment receive index */
        hAudioIn.idx_in = (hAudioIn.idx_in + 1U) % hAudioIn.buf.block_num;

        /* Call application callback function */
        if (hAudioIn.callback != NULL) {
            hAudioIn.callback(VSTREAM_EVENT_DATA);
        }

        if (hAudioIn.idx_in == hAudioIn.idx_rel) {
            /* Out of space, buffers shall be released */
            hAudioIn.status.overflow = 1U;

            if (hAudioIn.callback != NULL) {
                hAudioIn.callback(VSTREAM_EVENT_OVERFLOW);
            }
        }

        if (hAudioIn.idx_in != hAudioIn.idx_rel) {
            /* Determine current buffer index */
            buf_index = hAudioIn.idx_in * hAudioIn.buf.block_size;

            /* Set number of bytes to receive */
            num       = hAudioIn.buf.block_size / (AUDIO_CFG_BIT_DEPTH / 8U);

            /* Trigger new receive */
            hAudioIn.drv->Receive(&hAudioIn.buf.data[buf_index], num);
        }
    }
    if (event & ARM_SAI_EVENT_SEND_COMPLETE) {
    }
    if (event & ARM_SAI_EVENT_TX_UNDERFLOW) {
    }
    if (event & ARM_SAI_EVENT_RX_OVERFLOW) {
    }
}

/* Initialize streaming interface */
static int32_t Initialize(vStreamEvent_t event_cb)
{
    int32_t status;

    hAudioIn.callback = event_cb;
    hAudioIn.flags    = 0U;

    /* Initialize SAI */
    hAudioIn.drv      = &ARM_Driver_SAI_(AUDIO_CFG_SAI_INSTANCE);

    if (hAudioIn.drv->Initialize(Driver_SAI_Callback) != ARM_DRIVER_OK) {
        status = VSTREAM_ERROR;
    } else if (hAudioIn.drv->PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK) {
        status = VSTREAM_ERROR;
    } else {
        /* Configure receive channel */
        status = hAudioIn.drv->Control(ARM_SAI_CONFIGURE_RX | AUDIO_CFG_SAI_MODE |
                                           AUDIO_CFG_SAI_SYNC | AUDIO_CFG_SAI_PROTOCOL |
                                           ARM_SAI_DATA_SIZE(AUDIO_CFG_BIT_DEPTH),
                                       AUDIO_CFG_BIT_DEPTH * 2,
                                       AUDIO_CFG_SAMPLE_RATE);
        if (status != ARM_DRIVER_OK) {
            status = VSTREAM_ERROR;
        } else {
            /* Driver is initialized */
            hAudioIn.flags |= AUDIO_FLAGS_INIT;

            status          = VSTREAM_OK;
        }
    }

    return status;
}

/* De-initialize streaming interface */
static int32_t Uninitialize(void)
{

    /* Stop SAI receiver */
    hAudioIn.drv->Control(ARM_SAI_CONTROL_RX, 0U, 0U);

    if (hAudioIn.status.active == 1U) {
        /* Abort SAI receive transfer */
        hAudioIn.drv->Control(ARM_SAI_ABORT_RECEIVE, 0U, 0U);
    }

    /* Power down and uninitialize SAI */
    hAudioIn.drv->PowerControl(ARM_POWER_OFF);
    hAudioIn.drv->Uninitialize();

    /* Clear audio control block structure */
    memset(&hAudioIn, 0, sizeof(hAudioIn));

    return VSTREAM_OK;
}
/* Set streaming data buffer */
static int32_t SetBuf(void *buf, uint32_t buf_size, uint32_t block_size)
{
    int32_t status;

    if (buf == NULL) {
        status = VSTREAM_ERROR_PARAMETER;
    } else if ((buf_size == 0U) || (block_size == 0U)) {
        status = VSTREAM_ERROR_PARAMETER;
    } else {
        hAudioIn.buf.data       = buf;
        hAudioIn.buf.block_num  = buf_size / block_size;
        hAudioIn.buf.block_size = block_size;

        status                  = VSTREAM_OK;
    }

    return status;
}
/* Start streaming */
static int32_t Start(uint32_t mode)
{
    int32_t  rval;
    uint32_t num;

    if (hAudioIn.status.active == 1U) {
        /* Already active */
        rval = VSTREAM_OK;
    } else {
        if (mode == VSTREAM_MODE_SINGLE) {
            /* Single mode */
            hAudioIn.flags |= AUDIO_FLAGS_SINGLE;
        } else {
            /* Continuous mode */
            hAudioIn.flags &= ~AUDIO_FLAGS_SINGLE;
        }

        /* Set number of bytes to receive */
        num = hAudioIn.buf.block_size / (AUDIO_CFG_BIT_DEPTH / 8U);

        /* Register SAI receive operation */
        if (hAudioIn.drv->Receive(hAudioIn.buf.data, num) != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            /* Enable SAI receiver */
            if (hAudioIn.drv->Control(ARM_SAI_CONTROL_RX, 1U, 0U) != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            } else {
                rval = VSTREAM_OK;
            }
        }
        /* Streaming is active */
        hAudioIn.status.active = 1U;
    }
    return rval;
}
/* Stop streaming */
static int32_t Stop(void)
{
    int32_t status;
    int32_t rval;

    if (hAudioIn.status.active == 0U) {
        /* Not active */
        rval = VSTREAM_OK;
    } else {
        /* Disable SAI receiver */
        status = hAudioIn.drv->Control(ARM_SAI_CONTROL_RX, 0U, 0U);
        if (status == ARM_DRIVER_OK) {
            rval = VSTREAM_OK;
        } else {
            rval = VSTREAM_ERROR;
        }

        hAudioIn.status.active = 0U;
    }

    return rval;
}

/* Get pointer to a data block */
static void *GetBlock(void)
{
    uint32_t buf_index;
    void    *p;

    /* Check if there are blocks filled with data */
    if (hAudioIn.idx_get == hAudioIn.idx_in) {
        /* No data available */
        p = NULL;
    } else {
        /* Determine buffer index */
        buf_index        = hAudioIn.idx_get * hAudioIn.buf.block_size;

        /* Set return pointer */
        p                = &hAudioIn.buf.data[buf_index];

        /* Increment index of block returned by Get */
        hAudioIn.idx_get = (hAudioIn.idx_get + 1U) % hAudioIn.buf.block_num;
    }

    return p;
}

/* Release data block */
static int32_t ReleaseBlock(void)
{
    int32_t status;

    /* Check if there is anything to release */
    if (hAudioIn.idx_rel == hAudioIn.idx_get) {
        /* No blocks to release */
        status = VSTREAM_ERROR;
    } else {
        /* Increment index of the released block */
        hAudioIn.idx_rel = (hAudioIn.idx_rel + 1U) % hAudioIn.buf.block_num;

        status           = VSTREAM_OK;
    }

    return status;
}

/* Get Audio Interface status */
static vStreamStatus_t GetStatus(void)
{
    vStreamStatus_t status;

    /* Get audio in status */
    status                   = hAudioIn.status;

    hAudioIn.status.overflow = 0U;
    hAudioIn.status.eos      = 0U;

    return status;
}

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
