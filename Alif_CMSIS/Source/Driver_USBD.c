/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
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
#include "usbd_lib.h"
#include "usbd.h"
#include "Driver_USBD.h"

#define EP_NUM(ep_addr)         (ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK)

#define ARM_USBD_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */
extern usbd_dev_t * const usbd_dev_ptr[1];
/* Driver Version */
static const ARM_DRIVER_VERSION usbd_driver_version = { 
    ARM_USBD_API_VERSION,
    ARM_USBD_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
    1, /* vbus_detection */
    0, /* event_vbus_on */
    0, /* event_vbus_off */
    0  /* reserved */
};
USB_DRIVER usb_drv __attribute__((section("usb_dma_buf")));
static USBD_DRV_info usbd_drv;

//
// Functions
//
/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ARM_USBD_GetVersion(void)
{
    return usbd_driver_version;
}

static ARM_USBD_CAPABILITIES ARM_USBD_GetCapabilities(void)
{
    return usbd_driver_capabilities;
}

static int32_t ARM_USBD_Initialize(ARM_USBD_SignalDeviceEvent_t cb_device_event,
                                   ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
{
    USB_DRIVER *drv = &usb_drv;
    drv->cb_device_event = cb_device_event;
    drv->cb_endpoint_event = cb_endpoint_event;
    usbd_drv.drv_status.initialized = 1;
    return ARM_DRIVER_OK;

}

static int32_t ARM_USBD_Uninitialize(void)
{
    return ARM_DRIVER_OK;
}

static int32_t ARM_USBD_PowerControl(ARM_POWER_STATE state)
{
    int32_t status;
    switch (state)
    {
    case ARM_POWER_OFF:
        usbd_drv.drv_status.initialized = 0;
        usbd_drv.drv_status.powered = 0;
        NVIC_ClearPendingIRQ (USB_IRQ_IRQn);
        NVIC_DisableIRQ(USB_IRQ_IRQn);
        break;
    case ARM_POWER_LOW:
         return ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    case ARM_POWER_FULL:
        if(usbd_drv.drv_status.initialized == 0U)
            return ARM_DRIVER_ERROR;
        status =  usbd_initialize(&usb_drv);
        if(status != ARM_DRIVER_OK)
        {
            return status;
        }
        usbd_drv.drv_status.powered = 1;
        break;
    }
    return ARM_DRIVER_OK;
}

static int32_t ARM_USBD_DeviceConnect(void)
{
    int32_t status;
    status = usbd_connect(&usb_drv);
    if(status != ARM_DRIVER_OK)
    {
        return status;
    }
    else
    {
        return status;
    }
}

static int32_t ARM_USBD_DeviceDisconnect(void)
{
    if (usbd_drv.drv_status.powered == 0U) {
         return ARM_DRIVER_ERROR;
     }
    usbd_disconnect(&usb_drv);
    return ARM_DRIVER_OK;
}

static ARM_USBD_STATE ARM_USBD_DeviceGetState(void)
{
    ARM_USBD_STATE state;

    memset(&state, 0, sizeof(ARM_USBD_STATE));
    return state;
}

static int32_t ARM_USBD_DeviceRemoteWakeup(void)
{
    //Remote wakeup was disabled
    return ARM_DRIVER_OK;
}

static int32_t ARM_USBD_DeviceSetAddress(uint8_t dev_addr)
{
    int32_t status;
    status = usbd_SetDeviceAddress(&usb_drv, dev_addr);
    if(status != ARM_DRIVER_OK)
    {
        return status;
    }
    else
    {
        return status;
    }
}

static int32_t ARM_USBD_ReadSetupPacket(uint8_t *setup)
{
    int32_t status;
    memcpy(setup, &usb_drv.setup_data, USB_SETUP_PKT_SIZE);
    return ARM_DRIVER_OK;
}

static int32_t ARM_USBD_EndpointConfigure(uint8_t  ep_addr,
                                          uint8_t  ep_type,
                                          uint16_t ep_max_packet_size)
{
    uint8_t ep_num;
    uint8_t    ep_dir;
    uint32_t status;
    uint8_t ep_interval = 0;
    /* upper layer has to send ep interval to the low level driver */
    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK)? USB_DIR_IN : USB_DIR_OUT;
    status =  usbd_endpoint_create(&usb_drv, ep_type, ep_num, ep_dir, ep_max_packet_size, ep_interval);
    if(status != ARM_DRIVER_OK)
    {
        return status;
    }
    else
    {
        return status;
    }
}

static int32_t ARM_USBD_EndpointUnconfigure(uint8_t ep_addr)
{
    uint8_t ep_num;
    uint8_t    ep_dir;
    int32_t status;
    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK)? USB_DIR_IN : USB_DIR_OUT;
    status = usbd_ep_disable(&usb_drv, ep_num, ep_dir);
    if(status != ARM_DRIVER_OK)
    {
        return status;
    }
    else
    {
        return status;
    }
}

static int32_t ARM_USBD_EndpointStall(uint8_t ep_addr, bool stall)
{
    uint8_t ep_num;
    uint8_t ep_dir;
    int32_t status;
    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK)? USB_DIR_IN : USB_DIR_OUT;
    status = usbd_ep_stall(&usb_drv, ep_num, ep_dir, stall);
    if( status != ARM_DRIVER_OK)
    {
        return status;
    }
    else
    {
        return status;
    }
}

static int32_t ARM_USBD_EndpointTransfer(uint8_t ep_addr, uint8_t *data, uint32_t num)
{
    uint8_t ep_num;
    uint8_t ep_idx;
    uint8_t    ep_dir;
    uint32_t status;
    ep_num = EP_NUM(ep_addr);
    ep_idx =  ep_num + ((ep_addr & 0x80U) >> 3);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK)? USB_DIR_IN : USB_DIR_OUT;
    usbd_dev_ptr[0]->data_ptr->endpoint_active[ep_idx] = 0U;
    if((data == NULL) && (num == 0))
    {
        return ARM_DRIVER_OK;
    }
    if(ep_num == 0)
    {
        if(ep_dir != 0)
        {
            status = usbd_ep0_send(&usb_drv, ep_num, ep_dir, data, num);
            if(status != ARM_DRIVER_OK)
            {
                return status;
            }
            else
            {
                return status;
            }
        }
        else
        {
            status = usbd_ep0_recv(&usb_drv, ep_num, ep_dir, data , num);
            if(status != ARM_DRIVER_OK)
            {
                return status;
            }
            else
            {
                return status;
            }
        }
    }
    else
    {
        if(ep_dir != 0)
        {
            status = usbd_bulk_send(&usb_drv, ep_num, ep_dir, data, num);
            if(status != ARM_DRIVER_OK)
            {
                return status;
            }
            else
            {
                return status;
            }
        }
        else
        {
           status = usbd_bulk_recv(&usb_drv, ep_num, ep_dir, data, num);
            if(status != ARM_DRIVER_OK)
            {
                return status;
            }
            else
            {
                return status;
            }
        }
    }
    return ARM_DRIVER_OK;
}

static uint32_t ARM_USBD_EndpointTransferGetResult(uint8_t ep_addr)
{
    uint8_t ep_num;
    uint8_t    ep_dir;
    uint32_t status;
    uint32_t transferred;
    ep_num = EP_NUM(ep_addr);
    if(ep_num == 0)
    {
        transferred = usb_drv.actual_length;
    }
    else
    {
        transferred = usb_drv.num_bytes;
    }
    return transferred;
}

static int32_t ARM_USBD_EndpointTransferAbort(uint8_t ep_addr)
{
    uint8_t ep_num;
    uint8_t    ep_dir;
    uint32_t status;
    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK)? USB_DIR_IN : USB_DIR_OUT;
    status = usbd_ep_transfer_abort(&usb_drv, ep_num, ep_dir);
    if(status != ARM_DRIVER_OK)
    {
        return status;
    }
    else
    {
       return status;
    }
}

static uint16_t ARM_USBD_GetFrameNumber(void)
{
    return ARM_DRIVER_OK;
}

static void ARM_USBD_SignalDeviceEvent(uint32_t event)
{
    // function body
}

static void ARM_USBD_SignalEndpointEvent(uint8_t ep_addr, uint32_t ep_event)
{
    // function body
}

// End USBD Interface

extern \
ARM_DRIVER_USBD Driver_USBD0;
ARM_DRIVER_USBD Driver_USBD0 =
{
    ARM_USBD_GetVersion,
    ARM_USBD_GetCapabilities,
    ARM_USBD_Initialize,
    ARM_USBD_Uninitialize,
    ARM_USBD_PowerControl,
    ARM_USBD_DeviceConnect,
    ARM_USBD_DeviceDisconnect,
    ARM_USBD_DeviceGetState,
    ARM_USBD_DeviceRemoteWakeup,
    ARM_USBD_DeviceSetAddress,
    ARM_USBD_ReadSetupPacket,
    ARM_USBD_EndpointConfigure,
    ARM_USBD_EndpointUnconfigure,
    ARM_USBD_EndpointStall,
    ARM_USBD_EndpointTransfer,
    ARM_USBD_EndpointTransferGetResult,
    ARM_USBD_EndpointTransferAbort,
    ARM_USBD_GetFrameNumber
};
