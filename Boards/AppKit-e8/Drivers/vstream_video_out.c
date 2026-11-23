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
#include "Driver_CDC200.h"
#include "display.h"
#include "sys_utils.h"
#include "cache.h"

/* Handle Flags Definitions */
#define FLAGS_INIT              (1U << 0) /* Initialization completed                              */
#define FLAGS_START             (1U << 1) /* Display start attempted; low-level stop is still owed */
#define FLAGS_SINGLE            (1U << 2) /* Selected mode; retained after single completion       */
#define FLAGS_TERMINATED        (1U << 3) /* Explicit cleanup is required before reuse             */
#define FLAGS_CDC_INIT          (1U << 4) /* Low-level initialization attempted; cleanup is owed    */
#define FLAGS_CDC_POWER         (1U << 5) /* Low-level power-up attempted; power-off is owed        */

#define TRANSFER_PENDING        (1U << 0) /* Waiting for the vertical blanking reload              */
#define TRANSFER_ACTIVE         (1U << 1) /* Committed payload is being scanned                    */
#define TRANSFER_ERROR          (1U << 2) /* Queue failure awaiting notification from the ISR      */

#if (RTE_CDC200_IRQ_PRI != RTE_MIPI_DSI_IRQ_PRI)
#error "CDC and DSI callbacks require equal interrupt priorities"
#endif

/* Low Level Driver Instance */
extern ARM_DRIVER_CDC200  Driver_CDC200;
static ARM_DRIVER_CDC200 *DriverCDC = &Driver_CDC200;

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
    volatile uint32_t idx_out;       /* Index of oldest committed block                    */
    volatile uint32_t idx_pending;   /* Index waiting for vertical blanking reload         */
    uint32_t          cnt_pending;   /* Committed payload blocks awaiting completion       */
    uint32_t          cnt_acquired;  /* Outstanding acquisitions in circular FIFO order    */
    volatile uint8_t  transfer;      /* Progress of the current display transfer           */
    volatile uint8_t  pending_wait;  /* Exclude a partial period when reusing an address    */
    volatile uint8_t  active;        /* Streaming active flag                              */
    volatile uint8_t  underflow;     /* Buffer underflow flag                              */
    volatile uint8_t  eos;           /* End of stream flag                                 */
    volatile uint8_t  flags;         /* Handle status flags                                */
} StreamHandle_t;

/* vStream Handle */
static StreamHandle_t hVideoOut = {0};

/* Queue the next committed payload. Caller serializes against the ISR. */
static void QueueBlock(void)
{
    uint32_t buf_index;
    int32_t  status;

    if ((hVideoOut.active != 0U) && (hVideoOut.cnt_pending != 0U) &&
        ((hVideoOut.transfer & (TRANSFER_PENDING | TRANSFER_ACTIVE | TRANSFER_ERROR)) == 0U)) {
        buf_index = hVideoOut.idx_out * hVideoOut.buf.block_size;

        status = DriverCDC->Control(CDC200_FRAMEBUF_UPDATE_VSYNC, (uint32_t)&hVideoOut.buf.data[buf_index]);
        if (status == ARM_DRIVER_OK) {
            hVideoOut.idx_pending = hVideoOut.idx_out;
            hVideoOut.pending_wait = (CDC->CDC_LAYER_CFG[0].CDC_L_CFB_ADDR == LocalToGlobal(&hVideoOut.buf.data[buf_index]));
            hVideoOut.transfer |= TRANSFER_PENDING;
        } else {
            /* Deliver errors from the next scanline ISR, never from ReleaseBlock */
            hVideoOut.transfer |= TRANSFER_ERROR;
        }
    }
}

/* Low Level Driver Callback */
static void DriverCDC_Callback(uint32_t cb_event)
{
    uint32_t event;
    uint32_t buf_index;

    /* CDC and DSI callbacks are serialized by equal interrupt priorities. */
    if (hVideoOut.active == 0U) {
        return;
    }
    event = 0U;

    if ((cb_event & ARM_CDC_DSI_ERROR_EVENT) || ((hVideoOut.transfer & TRANSFER_ERROR) != 0U)) {
        hVideoOut.active       = 0U;
        hVideoOut.eos          = 1U;
        hVideoOut.transfer     = 0U;
        hVideoOut.pending_wait = 0U;
        hVideoOut.flags       |= FLAGS_TERMINATED;
        event |= VSTREAM_EVENT_EOS;
    }

    if ((cb_event & ARM_CDC_SCANLINE0_EVENT) && (hVideoOut.active != 0U)) {
        if ((hVideoOut.transfer & TRANSFER_PENDING) != 0U) {
            buf_index = hVideoOut.idx_pending * hVideoOut.buf.block_size;
            /* Confirm that the vertical blanking reload reached the working register */
            if (hVideoOut.pending_wait != 0U) {
                /* A matching address alone does not prove a complete new scan */
                hVideoOut.pending_wait = 0U;
            } else if (CDC->CDC_LAYER_CFG[0].CDC_L_CFB_ADDR == LocalToGlobal(&hVideoOut.buf.data[buf_index])) {
                hVideoOut.transfer &= ~TRANSFER_PENDING;
                hVideoOut.transfer |= TRANSFER_ACTIVE;
            }
        }

        if ((hVideoOut.transfer & TRANSFER_ACTIVE) != 0U) {
            /* One complete payload was scanned - publish storage before notification */
            hVideoOut.transfer &= ~TRANSFER_ACTIVE;
            hVideoOut.idx_out = (hVideoOut.idx_out + 1U) % hVideoOut.buf.block_num;
            hVideoOut.cnt_pending--;

            event |= VSTREAM_EVENT_DATA;
            if ((hVideoOut.flags & FLAGS_SINGLE) != 0U) {
                hVideoOut.active = 0U;
            }
        } else if ((hVideoOut.transfer & TRANSFER_PENDING) == 0U) {
            /* Another display period repeated uncommitted storage - No payload DATA */
            hVideoOut.underflow = 1U;
            event |= VSTREAM_EVENT_UNDERFLOW;
        }
    }

    if ((hVideoOut.callback != NULL) && (event != 0U)) {
        hVideoOut.callback(event);
    }

    /* Callback consumption/commitment may have supplied the next payload */
    if ((cb_event & ARM_CDC_SCANLINE0_EVENT) != 0U) {
        QueueBlock();
    }
}

/* Initialize streaming interface */
static int32_t Initialize(vStreamEvent_t event_cb)
{
    int32_t rval;
    int32_t status;

    if ((hVideoOut.flags & FLAGS_INIT) != 0U) {
        return VSTREAM_OK;
    }
    if (hVideoOut.flags != 0U) {
        /* Finish cleanup of an earlier initialization or shutdown failure first */
        return VSTREAM_ERROR;
    }

    hVideoOut.callback     = event_cb;
    hVideoOut.active       = 0U;
    hVideoOut.transfer     = 0U;
    hVideoOut.pending_wait = 0U;
    hVideoOut.underflow    = 0U;
    hVideoOut.eos          = 0U;
    hVideoOut.flags        = 0U;

    rval = VSTREAM_OK;

    /* Initialize and configure low level driver */
    hVideoOut.flags |= FLAGS_CDC_INIT;
    status = DriverCDC->Initialize(DriverCDC_Callback);
    if (status != ARM_DRIVER_OK) {
        rval = VSTREAM_ERROR;
    } else {
        hVideoOut.flags |= FLAGS_CDC_POWER;
        status = DriverCDC->PowerControl(ARM_POWER_FULL);
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoOut.flags |= FLAGS_START;

            status = DriverCDC->Stop();
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            } else {
                hVideoOut.flags &= ~FLAGS_START;
            }

            /* Enable the end-of-active-image scanline event */
            status = DriverCDC->Control(CDC200_SCANLINE0_EVENT, 1U);
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }

            /* Apply the selected panel configuration before the first output transfer */
            status = DriverCDC->Control(CDC200_CONFIGURE_DISPLAY, 0U);
            if (status != ARM_DRIVER_OK) {
                rval = VSTREAM_ERROR;
            }
        }
    }

    if (rval == VSTREAM_OK) {
        hVideoOut.flags |= FLAGS_INIT;
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
    hVideoOut.callback = NULL;

    /* Suppress ISR processing */
    hVideoOut.active       = 0U;
    hVideoOut.transfer     = 0U;
    hVideoOut.pending_wait = 0U;
    hVideoOut.flags       |= FLAGS_TERMINATED;

    if ((hVideoOut.flags & FLAGS_START) != 0U) {
        status = DriverCDC->Stop();
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoOut.flags &= ~FLAGS_START;
        }
    }

    if ((rval == VSTREAM_OK) && ((hVideoOut.flags & FLAGS_CDC_POWER) != 0U)) {
        status = DriverCDC->PowerControl(ARM_POWER_OFF);
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoOut.flags &= ~(FLAGS_INIT | FLAGS_CDC_POWER);
        }
    }

    if ((rval == VSTREAM_OK) && ((hVideoOut.flags & FLAGS_CDC_INIT) != 0U)) {
        status = DriverCDC->Uninitialize();
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            hVideoOut.flags &= ~FLAGS_CDC_INIT;
        }
    }

    if (rval == VSTREAM_OK) {
        /* Clear the handle only after shutdown has made its memory safe to reclaim */
        memset(&hVideoOut, 0, sizeof(hVideoOut));
    }

    return rval;
}

/* Set streaming data buffer */
static int32_t SetBuf(void *buf, uint32_t buf_size, uint32_t block_size)
{
    DISPLAY_PANEL_DEVICE *panel;
    static const uint8_t bytes_per_pixel[] = {4U, 3U, 2U, 4U, 1U, 1U, 2U, 2U};
    int32_t rval;

    panel = Get_Display_Panel();

    if (buf == NULL) {
        /* Buffer not specified */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((buf_size == 0U) || (block_size == 0U) || (block_size > buf_size)) {
        /* Invalid buffer or block size */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((((uintptr_t)buf & 31U) != 0U) || ((block_size & 31U) != 0U)) {
        /* Keep cache lines from overlapping adjacent blocks */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((panel == NULL) || (RTE_CDC200_PIXEL_FORMAT >= sizeof(bytes_per_pixel)) ||
             (panel->hactive_time == 0U) || (panel->vactive_line == 0U)) {
        /* Unsupported display configuration */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((block_size / bytes_per_pixel[RTE_CDC200_PIXEL_FORMAT] / panel->vactive_line) < panel->hactive_time) {
        /* Require a complete panel frame without overflowing size arithmetic */
        rval = VSTREAM_ERROR_PARAMETER;
    }
    else if ((hVideoOut.flags & FLAGS_INIT) == 0U) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    }
    else if (hVideoOut.active != 0U) {
        /* Stream is active */
        rval = VSTREAM_ERROR;
    }
    else if (hVideoOut.cnt_acquired != 0U) {
        /* Application still owns blocks */
        rval = VSTREAM_ERROR;
    }
    else if ((hVideoOut.flags & (FLAGS_START | FLAGS_TERMINATED)) != 0U) {
        /* Stream has been started or terminated, call Stop() first */
        rval = VSTREAM_ERROR;
    }
    else {
        rval = VSTREAM_OK;

        /* Set buffer */
        hVideoOut.buf.data       = (uint8_t *)buf;
        hVideoOut.buf.block_num  = buf_size / block_size;
        hVideoOut.buf.block_size = block_size;

        /* Reset buffer indexes */
        hVideoOut.idx_out       = 0U;
        hVideoOut.idx_get       = 0U;
        hVideoOut.idx_rel       = 0U;
        hVideoOut.idx_pending   = 0U;
        hVideoOut.cnt_pending   = 0U;
        hVideoOut.cnt_acquired  = 0U;

        /* Reset states and flags */
        hVideoOut.transfer      = 0U;
        hVideoOut.pending_wait  = 0U;
        hVideoOut.underflow     = 0U;
        hVideoOut.eos           = 0U;
        hVideoOut.flags        &= FLAGS_INIT | FLAGS_CDC_INIT | FLAGS_CDC_POWER;
    }

    return rval;
}

/* Start streaming */
static int32_t Start(uint32_t mode)
{
    int32_t  rval;
    int32_t  status;
    uint32_t primask;
    void    *buf;

    if ((mode != VSTREAM_MODE_SINGLE) && (mode != VSTREAM_MODE_CONTINUOUS)) {
        /* Invalid mode */
        rval = VSTREAM_ERROR_PARAMETER;
    } else if ((hVideoOut.flags & FLAGS_INIT) == 0U) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.active != 0U) {
        /* An active stream may only be started again in the same mode */
        if (((hVideoOut.flags & FLAGS_SINGLE) != 0U) == (mode == VSTREAM_MODE_SINGLE)) {
            rval = VSTREAM_OK;
        } else {
            rval = VSTREAM_ERROR;
        }
    } else if ((hVideoOut.flags & FLAGS_TERMINATED) != 0U) {
        /* Status reads do not recover a terminated stream */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.cnt_pending == 0U) {
        /* Output requires at least one committed payload */
        rval = VSTREAM_ERROR;
    } else {
        rval = VSTREAM_OK;

        /* Enable scanline interrupts to track output completion and underflow */
        status = DriverCDC->Control(CDC200_SCANLINE0_EVENT, 1U);
        if (status == ARM_DRIVER_OK) {
            primask = __get_PRIMASK();
            __disable_irq();

            /* Discard scanline interrupts from the preceding physical refresh */
            cdc_irq_clear(CDC, CDC_IRQ_LINE);
            NVIC_ClearPendingIRQ(CDC_SCANLINE0_IRQ_IRQn);

            if (mode == VSTREAM_MODE_SINGLE) {
                hVideoOut.flags |= FLAGS_SINGLE;
            } else {
                hVideoOut.flags &= ~FLAGS_SINGLE;
            }

            buf = &hVideoOut.buf.data[hVideoOut.idx_out * hVideoOut.buf.block_size];

            if ((hVideoOut.flags & FLAGS_START) == 0U) {
                /* Keep the stop obligation if setup or start fails partway */
                hVideoOut.flags |= FLAGS_START;

                status = DriverCDC->Control(CDC200_FRAMEBUF_UPDATE, (uint32_t)buf);
                if (status == ARM_DRIVER_OK) {
                    hVideoOut.transfer = TRANSFER_ACTIVE;
                    hVideoOut.active = 1U;
                    __set_PRIMASK(primask);
                    /* Panel start may block and may generate a callback */
                    status = DriverCDC->Start();
                    __disable_irq();
                }
            } else {
                /* Retained refresh needs a vertical-blanking address update */
                status = DriverCDC->Control(CDC200_FRAMEBUF_UPDATE_VSYNC, (uint32_t)buf);
                if (status == ARM_DRIVER_OK) {
                    hVideoOut.idx_pending = hVideoOut.idx_out;
                    hVideoOut.pending_wait = (CDC->CDC_LAYER_CFG[0].CDC_L_CFB_ADDR == LocalToGlobal(buf));
                    hVideoOut.transfer = TRANSFER_PENDING;
                    hVideoOut.active = 1U;
                }
            }
            if (status != ARM_DRIVER_OK) {
                hVideoOut.active = 0U;
                hVideoOut.transfer = 0U;
                hVideoOut.flags |= FLAGS_TERMINATED;
            }
            __set_PRIMASK(primask);
        }

        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
            /* Clear active flag */
            hVideoOut.active       = 0U;
            hVideoOut.transfer     = 0U;
            hVideoOut.pending_wait = 0U;
            hVideoOut.flags       |= FLAGS_TERMINATED;

            status = DriverCDC->Stop();
            if (status == ARM_DRIVER_OK) {
                hVideoOut.flags &= ~FLAGS_START;
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

    if ((hVideoOut.flags & FLAGS_INIT) == 0U) {
        /* Not initialized */
        rval = VSTREAM_ERROR;
    } else if ((hVideoOut.active == 0U) && ((hVideoOut.flags & (FLAGS_START | FLAGS_TERMINATED)) == 0U)) {
        /* No low-level display or shutdown cleanup is pending */
        rval = VSTREAM_OK;
    } else {
        /* Disable event callback */
        cb = hVideoOut.callback;
        hVideoOut.callback = NULL;

        /* Suppress ISR processing */
        hVideoOut.active       = 0U;
        hVideoOut.transfer     = 0U;
        hVideoOut.pending_wait = 0U;
        hVideoOut.flags       |= FLAGS_TERMINATED;

        status = DriverCDC->Stop();
        if (status != ARM_DRIVER_OK) {
            rval = VSTREAM_ERROR;
        } else {
            rval = VSTREAM_OK;
            hVideoOut.flags &= ~FLAGS_START;
        }
        /* Restore event callback */
        hVideoOut.callback = cb;
    }

    if (rval == VSTREAM_OK) {
        /* Stop discards pending output while preserving the buffer configuration */
        hVideoOut.idx_out       = 0U;
        hVideoOut.idx_get       = 0U;
        hVideoOut.idx_rel       = 0U;
        hVideoOut.idx_pending   = 0U;
        hVideoOut.pending_wait  = 0U;
        hVideoOut.cnt_pending   = 0U;
        hVideoOut.cnt_acquired  = 0U;

        hVideoOut.transfer = 0U;
        hVideoOut.flags  &= FLAGS_INIT | FLAGS_CDC_INIT | FLAGS_CDC_POWER;
    }

    return rval;
}

/* Get pointer to a data block */
static void *GetBlock(void)
{
    uint32_t primask;
    void    *p;

    primask = __get_PRIMASK();
    __disable_irq();

    if (hVideoOut.buf.data == NULL) {
        p = NULL;
    } else if ((hVideoOut.cnt_acquired + hVideoOut.cnt_pending) == hVideoOut.buf.block_num) {
        /* Every block is acquired, queued, or transferring a payload */
        p = NULL;
    } else {
        /* Get pointer to the next available block for acquisition */
        p = &hVideoOut.buf.data[hVideoOut.idx_get * hVideoOut.buf.block_size];

        /* Update the index and count of acquired blocks */
        hVideoOut.idx_get = (hVideoOut.idx_get + 1U) % hVideoOut.buf.block_num;
        hVideoOut.cnt_acquired++;
    }

    __set_PRIMASK(primask);

    return p;
}

/* Release the oldest acquisition and commit it for output */
static int32_t ReleaseBlock(void)
{
    int32_t  rval;
    uint32_t primask;
    uint32_t buf_index;

    if (hVideoOut.buf.data == NULL) {
        /* Buffer not set */
        rval = VSTREAM_ERROR;
    } else if (hVideoOut.cnt_acquired == 0U) {
        /* No blocks have been acquired */
        rval = VSTREAM_ERROR;
    } else {
        /* Get the buffer index for the block to be released */
        buf_index = hVideoOut.idx_rel * hVideoOut.buf.block_size;

        /* Finish CPU writes before publishing the payload to the display queue */
        RTSS_CleanDCache_by_Addr(&hVideoOut.buf.data[buf_index], hVideoOut.buf.block_size);
        __DMB();

        primask = __get_PRIMASK();
        __disable_irq();

        /* Move the oldest acquisition into the pending output queue */
        hVideoOut.idx_rel = (hVideoOut.idx_rel + 1U) % hVideoOut.buf.block_num;
        hVideoOut.cnt_acquired--;
        hVideoOut.cnt_pending++;

        /* Queue the released block for output */
        QueueBlock();

        __set_PRIMASK(primask);

        rval = VSTREAM_OK;
    }

    return rval;
}

/* Get Video Interface status */
static vStreamStatus_t GetStatus(void)
{
    vStreamStatus_t status = {0};
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    /* Get status */
    status.active       = hVideoOut.active;
    status.underflow    = hVideoOut.underflow;
    status.eos          = hVideoOut.eos;

    /* Clear status */
    hVideoOut.underflow = 0U;
    hVideoOut.eos       = 0U;
    __set_PRIMASK(primask);

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
