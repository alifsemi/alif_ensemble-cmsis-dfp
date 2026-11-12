/*
 * Copyright 2025-2026 Arm Limited and/or its affiliates.
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

#include "cmsis_vstream.h"
#include "RTE_Device.h"
#include "Driver_CPI.h"
#include "Driver_ISP.h"
#include "cpi.h"
#include "sys_utils.h"
#include "vsi_comm_video.h"


#if (RTE_CPI_AXI_PORT != 0) || (RTE_CPI_STREAMING_ENABLE != 0)
#error "ISP capture requires RTE_CPI_AXI_PORT=0 and RTE_CPI_STREAMING_ENABLE=0"
#endif

/* ISP output plane sizes */
#define ISP_FRAME_PIXELS (RTE_ISP_OUTPUT_WIDTH * RTE_ISP_OUTPUT_HEIGHT)

#if (RTE_ISP_OUTPUT_FORMAT == 20) || (RTE_ISP_OUTPUT_FORMAT == 37)
#define ISP_PLANE0_SIZE ISP_FRAME_PIXELS
#elif (RTE_ISP_OUTPUT_FORMAT == 21) || (RTE_ISP_OUTPUT_FORMAT == 22) || (RTE_ISP_OUTPUT_FORMAT == 32)
#define ISP_PLANE0_SIZE (ISP_FRAME_PIXELS * 2U)
#elif (RTE_ISP_OUTPUT_FORMAT == 23) || (RTE_ISP_OUTPUT_FORMAT == 40)
#define ISP_PLANE0_SIZE ISP_FRAME_PIXELS
#define ISP_PLANE1_SIZE (ISP_FRAME_PIXELS / 2U)
#elif (RTE_ISP_OUTPUT_FORMAT == 25) || (RTE_ISP_OUTPUT_FORMAT == 41)
#define ISP_PLANE0_SIZE ISP_FRAME_PIXELS
#define ISP_PLANE1_SIZE ISP_FRAME_PIXELS
#elif (RTE_ISP_OUTPUT_FORMAT == 30)
#define ISP_PLANE0_SIZE ISP_FRAME_PIXELS
#define ISP_PLANE1_SIZE (ISP_FRAME_PIXELS / 2U)
#define ISP_PLANE2_SIZE (ISP_FRAME_PIXELS / 2U)
#elif (RTE_ISP_OUTPUT_FORMAT == 31)
#define ISP_PLANE0_SIZE ISP_FRAME_PIXELS
#define ISP_PLANE1_SIZE (ISP_FRAME_PIXELS / 4U)
#define ISP_PLANE2_SIZE (ISP_FRAME_PIXELS / 4U)
#elif (RTE_ISP_OUTPUT_FORMAT == 39)
#define ISP_PLANE0_SIZE ISP_FRAME_PIXELS
#define ISP_PLANE1_SIZE ISP_FRAME_PIXELS
#define ISP_PLANE2_SIZE ISP_FRAME_PIXELS
#else
#error "Unsupported RTE_ISP_OUTPUT_FORMAT"
#endif

#ifndef ISP_PLANE1_SIZE
#define ISP_PLANE1_SIZE 0U
#endif
#ifndef ISP_PLANE2_SIZE
#define ISP_PLANE2_SIZE 0U
#endif
#define ISP_FRAME_SIZE (ISP_PLANE0_SIZE + ISP_PLANE1_SIZE + ISP_PLANE2_SIZE)

/* Handle Flags Definitions */
#define FLAGS_INIT              (1U << 0) /* Initialization completed                              */
#define FLAGS_START             (1U << 1) /* Capture start attempted; low-level stop is still owed */
#define FLAGS_SINGLE            (1U << 2) /* Selected mode; retained after single completion       */
#define FLAGS_TERMINATED        (1U << 3) /* Explicit cleanup is required before reuse             */
#define FLAGS_CPI_INIT          (1U << 4) /* Low-level initialization attempted; cleanup is owed    */
#define FLAGS_CPI_POWER         (1U << 5) /* Low-level power-up attempted; power-off is owed        */

#define CAPTURE_STOPPED         (1U << 0) /* CPI has reported capture-stop                         */
#define CAPTURE_FRAME_END       (1U << 1) /* ISP frame-end processing has completed                */

#define CPI_EVENTS (ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED | \
                    ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN | \
                    ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN | ARM_CPI_EVENT_ERR_HARDWARE)

/* Low Level Driver Instance */
extern ARM_DRIVER_CPI  Driver_CPI;
static ARM_DRIVER_CPI *DriverCPI = &Driver_CPI;
extern ARM_DRIVER_ISP  Driver_ISP;
static ARM_DRIVER_ISP *DriverISP = &Driver_ISP;

/* Stream Buffer Type */
typedef struct {
    uint8_t *data;       /* Data buffer pointer             */
    uint32_t block_num;  /* Number of blocks in data buffer */
    uint32_t block_size; /* Size of block in data buffer    */
} StreamBuf_t;

// Video Driver Configuration Parameters
typedef struct {
    vStreamEvent_t    callback;      /* VideoOut callback                                  */
    StreamBuf_t       buf;           /* VideoOut stream buffer                             */
    volatile uint32_t idx_get;       /* Index of block to be returned on GetBlock call     */
    volatile uint32_t idx_rel;       /* Index of block to be released on ReleaseBlock call */
    volatile uint32_t idx_in;        /* Index of block currently being streamed            */
    uint64_t          cnt_pending;   /* Completed FIFO entries not yet acquired             */
    uint32_t          cnt_acquired;  /* Outstanding acquisitions in circular FIFO order    */
    volatile uint8_t  capture;       /* Progress of the current CPI/ISP snapshot           */
    volatile uint8_t  active;        /* Streaming active flag                              */
    volatile uint8_t  overflow;      /* Buffer overflow flag                               */
    volatile uint8_t  eos;           /* End of stream flag                                 */
    volatile uint8_t  flags;         /* Handle status flags                                */
} StreamHandle_t;

/* vStream Handle */
static StreamHandle_t hVideoIn = {0};

/**
  Check whether a block belongs to the unreleased reader window.

  Acquired entries precede unread entries in logical FIFO order. After overflow,
  several pending entries can refer to successive uses of the same storage.

  \param[in] index Block index, less than the configured number of blocks.
  \retval 1 The block is acquired or unread, including pending drain candidates.
  \retval 0 The block is outside the reader window.
 */
static uint32_t BufferOccupied(uint32_t index) {
    uint32_t distance;
    uint64_t unreleased;

    /* Determine the distance from the oldest unreleased block to the given index (idx_in) */
    if (index >= hVideoIn.idx_rel) {
        /* Forward distance from the oldest unreleased block without wrapping */
        distance = index - hVideoIn.idx_rel;
    } else {
        /* Forward distance through the end of the ring and back to index */
        distance = hVideoIn.buf.block_num - (hVideoIn.idx_rel - index);
    }
    /* Determine the number of unreleased blocks */
    unreleased = hVideoIn.cnt_acquired + hVideoIn.cnt_pending;

    return distance < unreleased;
}

/* Queue one stream block for ISP output */
static int32_t ISP_QueueBlock(uint32_t index)
{
    VIDEO_BUF_S buffer;
    uint8_t    *data;

    data = &hVideoIn.buf.data[index * hVideoIn.buf.block_size];

    memset(&buffer, 0, sizeof(buffer));
    /* Reuse one ISP queue slot after stopping the previous snapshot. */
    buffer.index                = 0U;
    buffer.imageSize            = ISP_FRAME_SIZE;
    buffer.numPlanes            = 1U;
    buffer.planes[0].dmaPhyAddr = LocalToGlobal(data);
    buffer.planes[0].size       = ISP_PLANE0_SIZE;
    buffer.planes[0].pUserAddr  = data;

#if (ISP_PLANE1_SIZE != 0)
    buffer.numPlanes            = 2U;
    buffer.planes[1].dmaPhyAddr = LocalToGlobal(&data[ISP_PLANE0_SIZE]);
    buffer.planes[1].size       = ISP_PLANE1_SIZE;
    buffer.planes[1].pUserAddr  = &data[ISP_PLANE0_SIZE];
#endif
#if (ISP_PLANE2_SIZE != 0)
    buffer.numPlanes            = 3U;
    buffer.planes[2].dmaPhyAddr = LocalToGlobal(&data[ISP_PLANE0_SIZE + ISP_PLANE1_SIZE]);
    buffer.planes[2].size       = ISP_PLANE2_SIZE;
    buffer.planes[2].pUserAddr  = &data[ISP_PLANE0_SIZE + ISP_PLANE1_SIZE];
#endif

    return DriverISP->Control(ISP_CONTROL_QBUF, (uint32_t)&buffer);
}

/* Shared CPI/ISP callback: publish completed frames and rearm continuous snapshots */
static void DriverCPI_Callback(uint32_t cb_event)
{
    uint32_t error;
    void    *buf;

    if (hVideoIn.active == 0U) {
        /* Stream is inactive, ignore late callbacks */
        return;
    }

    /* An error takes precedence over completion of a potentially partial frame */
    error = cb_event & (ARM_CPI_EVENT_MIPI_CSI2_ERROR                |
                        ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN  |
                        ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                        ARM_CPI_EVENT_ERR_HARDWARE                   |
                        ARM_ISP_EVENT_SIZE_ERR_DETECTED              |
                        ARM_ISP_EVENT_DATALOSS_DETECTED);
    if (error == 0U) {
        /* No error, check for capture stop and frame end events */
        if (cb_event & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED) {
            /* CPI has stopped capturing */
            hVideoIn.capture |= CAPTURE_STOPPED;
        }

        if (cb_event & ARM_ISP_MI_EVENT_MP_FRAME_END_DETECTED) {
            /* ISP memory interface completed writing the main-picture frame */
            if ((hVideoIn.capture & CAPTURE_FRAME_END) == 0U) {
                /* Process frame-end stats and ISP bookkeeping */
                error = (DriverISP->Control(ISP_PROCESS_FRAME_END, 0U) != ARM_DRIVER_OK);
                if (error == 0U) {
                    hVideoIn.capture |= CAPTURE_FRAME_END;
                }
            }
        }
    }

    /* CPI stop and ISP frame end can arrive in either order; unrelated IRQs do nothing */
    if ((error == 0U) &&
        ((hVideoIn.capture & (CAPTURE_STOPPED | CAPTURE_FRAME_END)) == (CAPTURE_STOPPED | CAPTURE_FRAME_END))) {
        hVideoIn.capture = 0U;

        if ((hVideoIn.flags & FLAGS_SINGLE) != 0U) {
            /* Single mode, stop further capture */
            hVideoIn.active = 0U;
        }

        /* Keep one FIFO entry per completion, including overwritten entries.
         * GetBlock limits outstanding acquisitions to distinct physical blocks.
         */
        hVideoIn.cnt_pending++;

        /* Advance the input buffer index */
        hVideoIn.idx_in = (hVideoIn.idx_in + 1U) % hVideoIn.buf.block_num;

        if (hVideoIn.callback != NULL) {
            /* Deliver completion first; callback consumption may free the next destination */
            hVideoIn.callback(VSTREAM_EVENT_DATA);
        }

        /* Single mode ends here, rearm continuous capture */
        if (hVideoIn.active != 0U) {
            /* Check if the current input buffer is already occupied */
            if (BufferOccupied(hVideoIn.idx_in) != 0U) {
                /* Notify the application before overwriting occupied storage */
                hVideoIn.overflow = 1U;

                if (hVideoIn.callback != NULL) {
                    /* The OVERFLOW callback must return before the next overwrite can start */
                    hVideoIn.callback(VSTREAM_EVENT_OVERFLOW);
                }
            }

            /* CPI is stopped. Queue the next address before starting ISP output.     */
            /* Short-circuit on failure so later stages cannot use an invalid buffer. */
            if (DriverISP->Stop() != ARM_DRIVER_OK) {
                error = 1U;
            }
            else if (ISP_QueueBlock(hVideoIn.idx_in) != ARM_DRIVER_OK) {
                error = 1U;
            }
            else if (DriverISP->Start() != ARM_DRIVER_OK) {
                error = 1U;
            }

            if (error == 0U) {
                /* Get current buffer address */
                buf = &hVideoIn.buf.data[hVideoIn.idx_in * hVideoIn.buf.block_size];
                hVideoIn.capture = 0U;

                /* Next CPI frame buffer address is current buffer address */
                cpi_set_framebuff_start_addr(CPI, LocalToGlobal(buf));

                /* Re-start CPI capture in snapshot mode */
                cpi_start_capture(CPI, CPI_MODE_SELECT_SNAPSHOT);
            }
        }
    }

    if (error != 0U) {
        /* On error, stop the capture and report EOS */
        cpi_stop_capture(CPI);

        hVideoIn.active  = 0U;
        hVideoIn.capture = 0U;
        hVideoIn.eos     = 1U;
        hVideoIn.flags  |= FLAGS_TERMINATED;

        if (hVideoIn.callback != NULL) {
            hVideoIn.callback(VSTREAM_EVENT_EOS);
        }
    }
}

/* Initialize streaming interface */
static int32_t Initialize(vStreamEvent_t event_cb)
{
    int32_t  rval;
    int32_t  status;

    if ((hVideoIn.flags & FLAGS_INIT) != 0U) {
        return VSTREAM_OK;
    }
    if (hVideoIn.flags != 0U) {
        /* Finish cleanup of an earlier initialization or shutdown failure first */
        return VSTREAM_ERROR;
    }

    hVideoIn.callback = event_cb;
    hVideoIn.active   = 0U;
    hVideoIn.capture  = 0U;
    hVideoIn.overflow = 0U;
    hVideoIn.eos      = 0U;
    hVideoIn.flags    = 0U;

    rval = VSTREAM_OK;

    /* Initialize and configure low level driver */
    hVideoIn.flags |= FLAGS_CPI_INIT;
    status = DriverCPI->Initialize(DriverCPI_Callback);
    if (status != ARM_DRIVER_OK) {
        rval = VSTREAM_ERROR;
    } else {
        /* Enable power and configure the CPI interface */
        hVideoIn.flags |= FLAGS_CPI_POWER;
        status = DriverCPI->PowerControl(ARM_POWER_FULL);
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoIn.flags |= FLAGS_START;
            status = DriverCPI->Stop();
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            } else {
                hVideoIn.flags &= ~FLAGS_START;
            }

            status = DriverCPI->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0U);
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }

            status = DriverCPI->Control(CPI_CONFIGURE, 0U);
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }

            /* Set configuration */
            status = DriverCPI->Control(CPI_EVENTS_CONFIGURE, CPI_EVENTS);
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }
        }
    }

    if (rval == VSTREAM_OK) {
        /* Initialization successful */
        hVideoIn.flags |= FLAGS_INIT;
    }

    return rval;
}

/* De-initialize streaming interface */
static int32_t Uninitialize(void)
{
    int32_t rval;
    int32_t status;

    rval = VSTREAM_OK;

    /* Disable event callback */
    hVideoIn.callback = NULL;

    /* Suppress ISR processing */
    hVideoIn.active  = 0U;
    hVideoIn.capture = 0U;
    hVideoIn.flags  |= FLAGS_TERMINATED;

    if ((hVideoIn.flags & FLAGS_START) != 0U) {
        status = DriverCPI->Stop();
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoIn.flags &= ~FLAGS_START;
        }
    }

    if ((rval == VSTREAM_OK) && ((hVideoIn.flags & FLAGS_CPI_POWER) != 0U)) {
        status = DriverCPI->PowerControl(ARM_POWER_OFF);
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoIn.flags &= ~(FLAGS_INIT | FLAGS_CPI_POWER);
        }
    }

    if ((rval == VSTREAM_OK) && ((hVideoIn.flags & FLAGS_CPI_INIT) != 0U)) {
        status = DriverCPI->Uninitialize();
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoIn.flags &= ~FLAGS_CPI_INIT;
        }
    }

    if (rval == VSTREAM_OK) {
        /* Clear the handle only after shutdown has made its memory safe to reclaim */
        memset(&hVideoIn, 0, sizeof(hVideoIn));
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
    else if (block_size < ISP_FRAME_SIZE) {
        /* ISP is configured for frame that is larger than the block size */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((((uintptr_t)buf & 31U) != 0U) || ((block_size & 31U) != 0U)) {
        /* Buffer and block size shall be 32-byte aligned (cache line alignment) */
        /* Cache lines must not overlap adjacent blocks                          */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if (((ISP_PLANE0_SIZE & 7U) != 0U) || ((ISP_PLANE1_SIZE & 7U) != 0U) || ((ISP_PLANE2_SIZE & 7U) != 0U)) {
        /* Keep plane boundaries aligned to 8 bytes for ISP memory transfers */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if (((hVideoIn.flags & FLAGS_INIT) == 0U)) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else if (hVideoIn.active != 0U) {
        /* Stream is active */
        rval = VSTREAM_ERROR;
    }
    else if (hVideoIn.cnt_acquired != 0U) {
        /* Application still owns blocks */
        rval = VSTREAM_ERROR;
    }
    else if ((hVideoIn.flags & (FLAGS_START | FLAGS_TERMINATED)) != 0U) {
        /* Stream has been started or terminated, call Stop() first */
        rval = VSTREAM_ERROR;
    }
    else {
        rval = VSTREAM_OK;

        /* Set buffer */
        hVideoIn.buf.data       = (uint8_t *)buf;
        hVideoIn.buf.block_num  = buf_size / block_size;
        hVideoIn.buf.block_size = block_size;

        /* Reset buffer indexes */
        hVideoIn.idx_in       = 0U;
        hVideoIn.idx_get      = 0U;
        hVideoIn.idx_rel      = 0U;
        hVideoIn.cnt_pending  = 0U;
        hVideoIn.cnt_acquired = 0U;

        /* Reset states and flags */
        hVideoIn.capture  = 0U;
        hVideoIn.overflow = 0U;
        hVideoIn.eos      = 0U;
        hVideoIn.flags   &= FLAGS_INIT | FLAGS_CPI_INIT | FLAGS_CPI_POWER;
    }

    return rval;
}

/* Start streaming */
static int32_t Start(uint32_t mode)
{
    int32_t rval;
    int32_t status;
    void   *buf;

    if ((mode != VSTREAM_MODE_SINGLE) && (mode != VSTREAM_MODE_CONTINUOUS)) {
        /* Invalid mode */
        rval = VSTREAM_ERROR_PARAMETER;
    } else if ((hVideoIn.flags & FLAGS_INIT) == 0) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    } else if (hVideoIn.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    } else if (hVideoIn.active != 0U) {
        /* An active stream may only be started again in the same mode */
        if (((hVideoIn.flags & FLAGS_SINGLE) != 0U) == (mode == VSTREAM_MODE_SINGLE)) {
            rval = VSTREAM_OK;
        } else {
            rval = VSTREAM_ERROR;
        }
    } else if ((hVideoIn.flags & FLAGS_TERMINATED) != 0U) {
        /* Status reads do not recover a terminated stream */
        rval = VSTREAM_ERROR;
    } else if (BufferOccupied(hVideoIn.idx_in) != 0U) {
        /* An inactive stream starts only with a reusable first destination */
        rval = VSTREAM_ERROR;
    } else {
        rval = VSTREAM_OK;

        /* Set pointer to frame buffer */
        buf = &hVideoIn.buf.data[hVideoIn.idx_in * hVideoIn.buf.block_size];

        /* Configure CPI events before starting the capture */
        status = DriverCPI->Control(CPI_EVENTS_CONFIGURE, CPI_EVENTS);
        if ((status == ARM_DRIVER_OK) && ((hVideoIn.flags & FLAGS_START) != 0U)) {
            status = DriverISP->Stop();
        }
        if (status == ARM_DRIVER_OK) {
            status = ISP_QueueBlock(hVideoIn.idx_in);
        }

        if (mode == VSTREAM_MODE_SINGLE) {
            /* Single mode */
            hVideoIn.flags |= FLAGS_SINGLE;

        } else {
            /* Continuous mode is a sequence of driver-managed snapshots */
            hVideoIn.flags &= ~FLAGS_SINGLE;
        }

        if (status == ARM_DRIVER_OK) {
            /* Mark the stream as active and indicate that capture has started */
            hVideoIn.active  = 1U;
            hVideoIn.capture = 0U;
            hVideoIn.flags  |= FLAGS_START;

            /* Start capturing a frame */
            status = DriverCPI->CaptureFrame(buf);
        }

        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
            /* Clear active flag */
            hVideoIn.active  = 0U;
            hVideoIn.capture = 0U;
            hVideoIn.flags  |= FLAGS_TERMINATED;

            status = DriverCPI->Stop();
            if (status == ARM_DRIVER_OK) {
                hVideoIn.flags &= ~FLAGS_START;
            }
        }
    }

    return rval;
}

/* Stop streaming */
static int32_t Stop(void)
{
    int32_t rval;
    int32_t status;
    vStreamEvent_t cb;

    if ((hVideoIn.flags & FLAGS_INIT) == 0U) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    } else if ((hVideoIn.active == 0U) && ((hVideoIn.flags & (FLAGS_START | FLAGS_TERMINATED)) == 0U)) {
        /* No low-level capture or shutdown cleanup is pending */
        rval = VSTREAM_OK;
    } else {
        /* Disable event callback */
        cb = hVideoIn.callback;
        hVideoIn.callback = NULL;

        /* Suppress ISR processing */
        hVideoIn.active  = 0U;
        hVideoIn.capture = 0U;
        hVideoIn.flags  |= FLAGS_TERMINATED;

        status = DriverCPI->Stop();
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            rval = VSTREAM_OK;
            hVideoIn.flags &= ~FLAGS_START;
        }
        /* Restore event callback */
        hVideoIn.callback = cb;
    }

    if (rval == VSTREAM_OK) {
        /* Stop discards buffered data while preserving the buffer configuration */
        hVideoIn.idx_in       = 0U;
        hVideoIn.idx_get      = 0U;
        hVideoIn.idx_rel      = 0U;
        hVideoIn.cnt_pending  = 0U;
        hVideoIn.cnt_acquired = 0U;

        hVideoIn.capture = 0U;
        hVideoIn.flags  &= FLAGS_INIT | FLAGS_CPI_INIT | FLAGS_CPI_POWER;
    }

    return rval;
}

/* Acquire the next FIFO entry without skipping entries affected by overflow */
static void *GetBlock(void)
{
    uint32_t primask;
    void    *p;

    primask = __get_PRIMASK();
    __disable_irq();

    if (hVideoIn.buf.data == NULL) {
        /* Buffer not set */
        p = NULL;
    }
    else if (hVideoIn.cnt_pending == 0U) {
        /* No completed FIFO entries are waiting to be acquired */
        p = NULL;
    }
    else if (hVideoIn.cnt_acquired == hVideoIn.buf.block_num) {
        /* All physical blocks are acquired; pending entries must wait for release */
        p = NULL;
    }
    else {
        p = &hVideoIn.buf.data[hVideoIn.idx_get * hVideoIn.buf.block_size];

        /* Advance one logical entry */
        hVideoIn.idx_get = (hVideoIn.idx_get + 1U) % hVideoIn.buf.block_num;
        hVideoIn.cnt_pending--;
        hVideoIn.cnt_acquired++;
    }

    __set_PRIMASK(primask);

    if (p != NULL) {
        /* Invalidate cache after reserving the block */
        RTSS_InvalidateDCache_by_Addr(p, hVideoIn.buf.block_size);
    }

    return p;
}

/* Release the oldest acquisition without affecting circular capture progression */
static int32_t ReleaseBlock(void)
{
    int32_t  rval;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    rval = VSTREAM_ERROR;

    if (hVideoIn.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    } else if (hVideoIn.cnt_acquired == 0U) {
        /* No blocks have been acquired */
        rval = VSTREAM_ERROR;
    }
    else {
        /* Increment index of the oldest acquired block */
        hVideoIn.idx_rel = (hVideoIn.idx_rel + 1U) % hVideoIn.buf.block_num;

        /* Decrement the number of acquired blocks */
        hVideoIn.cnt_acquired--;

        rval = VSTREAM_OK;
    }

    __set_PRIMASK(primask);

    return rval;
}

/* Get Video Interface status */
static vStreamStatus_t GetStatus(void)
{
    vStreamStatus_t status = {0};
    uint32_t        primask;

    primask = __get_PRIMASK();
    __disable_irq();

    /* Get status */
    status.active   = hVideoIn.active;
    status.overflow = hVideoIn.overflow;
    status.eos      = hVideoIn.eos;

    /* Clear status */
    hVideoIn.overflow = 0U;
    hVideoIn.eos      = 0U;

    __set_PRIMASK(primask);

    return status;
}

vStreamDriver_t Driver_vStreamVideoIn = {
    Initialize,
    Uninitialize,
    SetBuf,
    Start,
    Stop,
    GetBlock,
    ReleaseBlock,
    GetStatus
};
