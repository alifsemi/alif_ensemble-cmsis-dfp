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

#include "RTE_Components.h"
#include CMSIS_device_header

#include "cmsis_vstream.h"
#include "Driver_CDC200.h"

/* Handle Flags Definitions */
#define FLAGS_INIT      (1U << 0)
#define FLAGS_START     (1U << 1)
#define FLAGS_SINGLE    (1U << 2)
#define FLAGS_LIMIT_OWN (1U << 3)
#define FLAGS_BUF_EMPTY (1U << 4)
#define FLAGS_BUF_FULL  (1U << 5)

/* Low Level Driver Instance */
extern ARM_DRIVER_CDC200  Driver_CDC200;
static ARM_DRIVER_CDC200 *DriverCDC = &Driver_CDC200;

/* Stream Buffer Type */
typedef struct {
    uint8_t    *data;       /* Data buffer pointer             */
    uint32_t block_num;  /* Number of blocks in data buffer */
    uint32_t block_size; /* Size of block in data buffer    */
} StreamBuf_t;

// Video Driver Configuration Parameters
typedef struct {
    vStreamEvent_t    callback;  /* VideoOut callback       */
    StreamBuf_t       buf;       /* VideoOut stream buffer  */
    volatile uint32_t idx_get;   /* Index of block to be returned on GetBlock call     */
    volatile uint32_t idx_rel;   /* Index of block to be released on ReleaseBlock call */
    volatile uint32_t idx_out;   /* Index of block currently beeing streamed           */
    volatile uint8_t  active;    /* Streaming active flag */
    volatile uint8_t  underflow; /* Buffer underflow flag */
    volatile uint8_t  eos;       /* End of stream flag    */
    uint8_t           flags;
} StreamHandle_t;

/* vStream Handle */
static StreamHandle_t hVideoOut = {0};

/* Low Level Driver Callback */
static void DriverCDC_Callback(uint32_t cb_event)
{
    uint32_t event;
    uint32_t buf_index;

    event = 0U;

    if (cb_event & ARM_CDC_SCANLINE0_EVENT) {
        /* Start of frame */
        event             |= VSTREAM_EVENT_DATA;

        /* Increment index of the block beeing streamed */
        hVideoOut.idx_out  = (hVideoOut.idx_out + 1) % hVideoOut.buf.block_num;

        /* Clear buffer full flag */
        hVideoOut.flags   &= ~FLAGS_BUF_FULL;

        if (hVideoOut.idx_out == hVideoOut.idx_rel) {
            /* Buffer is empty */
            hVideoOut.flags |= FLAGS_BUF_EMPTY;

            if ((hVideoOut.flags & FLAGS_SINGLE) == 0U) {
                /* Continuous mode, no new stream buffer */
                hVideoOut.underflow  = 1U;

                event               |= VSTREAM_EVENT_UNDERFLOW;
            }
        }

        if (hVideoOut.flags & FLAGS_SINGLE) {
            /* Single mode, only clear active flag */
            hVideoOut.active = 0U;
        } else {
            /* Continuous mode, determine streaming buffer index */
            buf_index = hVideoOut.idx_out * hVideoOut.buf.block_size;

            /* Set new frame buffer address */
            DriverCDC->Control(CDC200_FRAMEBUF_UPDATE, (uint32_t) &hVideoOut.buf.data[buf_index]);
        }
    }

    if (cb_event & ARM_CDC_DSI_ERROR_EVENT) {
        /* Error, stream stopped */
        event            |= VSTREAM_EVENT_EOS;

        hVideoOut.active  = 0U;
        hVideoOut.eos     = 1U;
    }

    if ((hVideoOut.callback != NULL) && (event != 0U)) {
        /* Call application callback function */
        hVideoOut.callback(event);
    }
}

/* Initialize streaming interface */
static int32_t Initialize(vStreamEvent_t event_cb)
{
    int32_t rval;
    int32_t status;

    hVideoOut.callback  = event_cb;
    hVideoOut.active    = 0U;
    hVideoOut.underflow = 0U;
    hVideoOut.eos       = 0U;
    hVideoOut.flags     = 0U;

    rval                = VSTREAM_OK;

    /* Initialize and configure low level driver */
    status              = DriverCDC->Initialize(DriverCDC_Callback);
    if (status != ARM_DRIVER_OK) {
        rval = VSTREAM_ERROR;
    } else {
        status = DriverCDC->PowerControl(ARM_POWER_FULL);
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            status = DriverCDC->Stop();
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }

            /* Enable scanline event (i.e. start of frame) */
            status = DriverCDC->Control(CDC200_SCANLINE0_EVENT, 1U);
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }

            /* Configure display */
            status = DriverCDC->Control(CDC200_CONFIGURE_DISPLAY, 0U);
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }
        }
    }

    if (rval == VSTREAM_OK) {
        hVideoOut.flags = FLAGS_INIT;
    }

    return rval;
}

/* De-initialize streaming interface */
static int32_t Uninitialize(void)
{
    int32_t rval;
    int32_t status;

    hVideoOut.flags    = 0U;
    hVideoOut.callback = NULL;
    hVideoOut.buf.data = NULL;

    /* De-initialize low level driver */
    status             = DriverCDC->Uninitialize();
    if (status != ARM_DRIVER_OK) {
        rval = VSTREAM_ERROR;
    } else {
        rval = VSTREAM_OK;
    }
    return rval;
}

/* Set streaming data buffer */
static int32_t SetBuf(void *buf, uint32_t buf_size, uint32_t block_size)
{
    int32_t rval;

    if (buf == NULL) {
        rval = VSTREAM_ERROR_PARAMETER;
    } else if ((buf_size == 0U) || (block_size == 0U) || (block_size > buf_size)) {
        rval = VSTREAM_ERROR_PARAMETER;
    } else if ((hVideoOut.flags & FLAGS_INIT) == 0) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.active == 1U) {
        /* Streaming is active */
        rval = VSTREAM_ERROR;
    } else {
        /* Set buffer */
        hVideoOut.buf.data        = (uint8_t *)buf;
        hVideoOut.buf.block_num   = buf_size / block_size;
        hVideoOut.buf.block_size  = block_size;

        /* Buffer is empty */
        hVideoOut.flags          |= FLAGS_BUF_EMPTY;

        /* Reset indexes */
        hVideoOut.idx_out         = 0U;
        hVideoOut.idx_get         = 0U;
        hVideoOut.idx_rel         = 0U;

        rval                      = VSTREAM_OK;
    }

    return rval;
}

/* Start streaming */
static int32_t Start(uint32_t mode)
{
    int32_t rval;
    int32_t status;
    void   *buf;

    if ((hVideoOut.flags & FLAGS_INIT) == 0) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    } else if ((hVideoOut.flags & FLAGS_BUF_EMPTY) != 0) {
        /* Buffer is empty */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.active == 1U) {
        /* Already active */
        rval = VSTREAM_OK;
    } else {
        rval             = VSTREAM_OK;

        /* Set active status */
        hVideoOut.active = 1U;

        /* Set pointer to frame buffer */
        buf              = &hVideoOut.buf.data[hVideoOut.idx_rel * hVideoOut.buf.block_size];

        if (mode == VSTREAM_MODE_SINGLE) {
            /* Single mode */
            hVideoOut.flags |= FLAGS_SINGLE;
        } else {
            /* Continuous mode */
            hVideoOut.flags &= ~FLAGS_SINGLE;
        }

        /* Set frame buffer address */
        status = DriverCDC->Control(CDC200_FRAMEBUF_UPDATE, (uint32_t) buf);
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            /* Start streaming */
            if ((hVideoOut.flags & FLAGS_START) == 0) {
                hVideoOut.flags |= FLAGS_START;
                /* Underlying driver supports only continuous mode, hence  */
                /* each subsequent call only updates frame buffer address. */

                /* Start display stream */
                status           = DriverCDC->Start();
                if (status != ARM_DRIVER_OK) {
                    rval = VSTREAM_ERROR;
                }
            }
        }
        if (rval != VSTREAM_OK) {
            /* Clear active flag */
            hVideoOut.active = 0U;
        }
    }

    return rval;
}

/* Stop streaming */
static int32_t Stop(void)
{
    int32_t rval;
    int32_t status;

    if ((hVideoOut.flags & FLAGS_INIT) == 0) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.active == 0U) {
        /* Not active */
        rval = VSTREAM_OK;
    } else {
        /* Stop the stream */
        status = DriverCDC->Stop();
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            rval = VSTREAM_OK;
        }

        /* Enable call of the CDC driver Start() function */
        hVideoOut.flags  &= ~FLAGS_START;

        hVideoOut.active  = 0U;
    }

    return rval;
}

/* Get pointer to a data block */
static void *GetBlock(void)
{
    uint32_t buf_index;
    void    *p;

    if (hVideoOut.buf.data == NULL) {
        /* Buffer not set */
        p = NULL;
    } else if ((hVideoOut.flags & FLAGS_LIMIT_OWN) != 0) {
        /* App already owns all the blocks */
        p = NULL;
    } else {
        /* Determine buffer index */
        buf_index         = hVideoOut.idx_get * hVideoOut.buf.block_size;

        /* Set return pointer */
        p                 = &hVideoOut.buf.data[buf_index];

        /* Increment index of block returned by Get */
        hVideoOut.idx_get = (hVideoOut.idx_get + 1U) % hVideoOut.buf.block_num;

        if (hVideoOut.idx_get == hVideoOut.idx_rel) {
            /* App owns all the blocks, set the limit flag */
            hVideoOut.flags |= FLAGS_LIMIT_OWN;
        }
    }

    return p;
}

/* Release data block */
static int32_t ReleaseBlock(void)
{
    int32_t rval;

    if (hVideoOut.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    } else if ((hVideoOut.idx_rel == hVideoOut.idx_get) &&
               ((hVideoOut.flags & FLAGS_LIMIT_OWN) == 0)) {
        /* No blocks to release */
        rval = VSTREAM_ERROR;
    } else {
        /* Increment index of the block to be released next */
        hVideoOut.idx_rel = (hVideoOut.idx_rel + 1U) % hVideoOut.buf.block_num;

        if (hVideoOut.idx_rel == hVideoOut.idx_out) {
            /* Buffer is full */
            hVideoOut.flags |= FLAGS_BUF_FULL;
        }

        /* Clear the limit get flag */
        hVideoOut.flags &= ~FLAGS_LIMIT_OWN;

        /* Clear the buffer empty flag */
        hVideoOut.flags &= ~FLAGS_BUF_EMPTY;

        rval             = VSTREAM_OK;
    }

    return rval;
}

/* Get Audio Interface status */
static vStreamStatus_t GetStatus(void)
{
    vStreamStatus_t status;

    /* Get status */
    status.active       = hVideoOut.active;
    status.underflow    = hVideoOut.underflow;
    status.eos          = hVideoOut.eos;

    /* Clear status */
    hVideoOut.underflow = 0U;
    hVideoOut.eos       = 0U;

    return status;
}

vStreamDriver_t Driver_vStreamVideoOut = {
    Initialize,
    Uninitialize,
    SetBuf,
    Start,
    Stop,
    GetBlock,
    ReleaseBlock,
    GetStatus
};
